// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FRACTAL_TRIE_H
#define _URCU_FRACTAL_TRIE_H

/*
 * urcu/fractal-trie.h
 *
 * Userspace RCU library - Fractal Trie
 *
 * For use with URCU_API_MAP (API mapping of liburcu), include this
 * file _after_ including your URCU flavor.  The modern
 * <urcu/urcu-*.h> flavor headers clear the API mapping at the end of
 * the header; with those, use cds_ft_group_create_flavor() (explicit
 * flavor argument) instead of cds_ft_group_create().
 *
 * A concurrent, RCU-protected ordered trie mapping variable or
 * fixed-length byte keys to user-defined nodes. Keys are opaque
 * byte sequences with no reserved or sentinel values; unlike
 * tries that rely on NUL or other terminal characters, any byte
 * value may appear at any position in a key. Keys are ordered
 * most-significant-byte first: the byte at offset 0 is most
 * significant and the last byte least significant -- i.e.
 * lexicographically by byte sequence, the big-endian form of
 * an integer key. This is the order used by ordered iteration
 * and the range queries (<=, >=, <, >). Lookups and traversals
 * are wait-free under the RCU read-side lock and can proceed
 * concurrently with mutations. The internal structure is
 * acyclic by construction: no sequence of concurrent updates can
 * introduce a cycle, ensuring that all lookups and traversals
 * complete in bounded time. Supports exact lookup, partial
 * (prefix) match, ordered iteration, duplicate key chains, and
 * range queries (<=, >=, <, >). A longest-match lookup reports
 * the deepest trie position matching a prefix of the input key,
 * even if that position has no external node.
 *
 * Performance characteristics:
 *
 * - Lookup: At most 2 cache-line accesses per key byte.
 * - Ordered traversal: At most 3 cache-line accesses per node.
 *
 * Prefix / path compression:
 *
 * Chains of single-child nodes along a shared prefix are
 * path-compressed automatically and transparently; no configuration
 * is required.  Compression can be disabled at compile time with
 * -DNO_FEATURE_FT_COMPRESS, and the related skip-compressed lookup
 * optimization with -DNO_FEATURE_FT_SKIP_COMPRESSED.
 *
 * Memory use:
 *
 * Internal nodes are reclaimed via RCU (call_rcu), so readers never
 * access freed memory even for the trie's own internal structure.
 * Internal nodes adapt to the key population automatically; memory
 * efficiency is comparable to adaptive radix tree schemes with no
 * user tuning or configuration.
 *
 * Resident memory (RSS) overstates the actual *cache-hot* working set:
 * Fractal Trie can keep a denser cache-hot set than other trie
 * implementations even though its RSS is higher.
 *
 * Graft, graft-swap, detach, and merge:
 *
 * The graft, graft-swap, detach, and merge operations move (or,
 * for merge, combine) entire sub-tries between trie instances
 * within the same group. The write-side cost is more than a single
 * pointer store (so "O(1)" below means independent of subtree size,
 * not literally one store). Each of these ops also unlinks content
 * from a source trie (graft-swap from the destination too), so --
 * when concurrent readers are possible -- it may issue a
 * synchronize_rcu to drain readers of the unlinked content before its
 * nodes are reclaimed. What concurrent readers see, however, is
 * published atomically: a reader observes either the complete prior
 * state or the complete result, never a partial state.
 *
 * - Graft (cds_ft_graft) attaches the content of a source trie
 *   at a key position in a destination trie. The destination
 *   must have no content at or below the graft point. On
 *   success the source trie becomes empty and can be reused or
 *   destroyed.
 *
 * - Graft-swap (cds_ft_graft_swap) exchanges the content at a
 *   key position in a destination trie with the content of
 *   another trie. Unlike plain graft, the destination does not
 *   need to be empty at the graft point: any pre-existing
 *   content is moved into the swap trie. Concurrent readers
 *   see either the old content or the new content, never an
 *   empty intermediate state.
 *
 * - Detach (cds_ft_detach) removes the sub-structure rooted at
 *   a key position and returns it as a new, independent trie
 *   instance whose key lengths are relative to the detach
 *   point.
 *
 * - Merge-at (cds_ft_merge_at) combines a source sub-trie into a
 *   destination while re-keying it: the content under a source
 *   prefix is unioned into the destination under a possibly
 *   different prefix, each moved key keeping whatever suffix
 *   follows the prefix. Unlike graft, the destination may already
 *   hold entries under that prefix -- the two sub-tries are unioned
 *   and same-key duplicate chains are concatenated. It expresses
 *   re-keying patterns (archive moves, tier promotion, partition
 *   rename) that a detach + graft pair cannot on a fixed-length
 *   group, where the source and destination prefixes must be of
 *   equal length so the moved keys keep the fixed length.
 *
 * - Merge (cds_ft_merge) is the same-prefix case of merge-at: a
 *   single key is both the source selector and the destination
 *   attach point, so nothing is re-keyed; a zero-length key merges
 *   two whole tries at the root.
 *
 * - Rekey (cds_ft_rekey_graft / cds_ft_rekey_merge) moves a subtree
 *   to a new key WITHIN one trie -- the same-trie analog of graft and
 *   merge-at (graft requires an empty destination; merge unions into an
 *   occupied one). The source and destination keys must be disjoint, and
 *   the trie must be EAGER (not speculative). Same-trie moves are their
 *   own entry points because they are made coherent for concurrent
 *   readers; cds_ft_merge_at is cross-trie only.
 *
 * Root-level operations (key_len 0) work with both fixed-length
 * and variable-length key groups. Non-root operations (key_len
 * > 0) require a variable-length key group
 * (CDS_FT_LEN_VARIABLE): in a fixed-length group every key
 * must equal the group's fixed length, so a shorter prefix
 * needed to address an interior graft or detach point cannot be
 * expressed, and the call returns
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR.
 *
 * Together, these operations serve as the transplant, exchange,
 * and split primitives for bulk operations. A typical bulk-load
 * pattern populates a staging trie offline -- with no RCU or
 * locking overhead -- and then grafts it into the live trie in
 * a single O(1) step, keeping the writer critical section
 * minimal regardless of the number of nodes being moved. A
 * complementary bulk-removal pattern detaches (or graft-swaps)
 * a sub-trie in O(1), waits for a single grace period, and
 * then drains the detached trie locally, reducing the cost
 * from one grace period per node to one grace period total.
 *
 * Reader contract:
 *
 * Every read-side operation (lookup, traversal, iteration,
 * count, ...) must satisfy the discipline matching the trie's
 * mode:
 *
 *   Concurrent mode (default):
 *     - the RCU read-side lock is held by the caller for the
 *       duration of the read, OR
 *     - the caller guarantees mutual exclusion against concurrent
 *       mutations on this trie (e.g. by holding a lock that
 *       serialises all mutations).
 *
 *   Exclusive mode:
 *     - the caller guarantees mutual exclusion against any
 *       concurrent access on this trie.  Exclusive-mode mutations
 *       skip RCU synchronisation primitives (synchronize_rcu,
 *       call_rcu) on the assumption that there are no concurrent
 *       readers, so holding the RCU read-side lock on its own
 *       does not protect against use-after-free in this mode.
 *
 * The library never calls rcu_read_lock() on the caller's behalf.
 * If FEATURE_FT_EXCL_VALIDATE is compiled in, the library
 * *validates* the discipline above and aborts on violation -- it
 * does not enforce the discipline.  Compliance is the caller's
 * responsibility.  The validator catches:
 *
 *   - writer/writer overlap in any mode;
 *   - reader/writer overlap in exclusive mode;
 *   - reader/writer overlap in concurrent mode for readers that did
 *     not hold the RCU read-side lock at entry (i.e. readers
 *     asserting the mutex-claim path).  Readers that did hold the
 *     RCU read-side lock are permitted to overlap with a single
 *     writer.
 *
 * Detection in concurrent mode is point-in-time: a reader that
 * holds neither the RCU read-side lock nor an external mutex but
 * does not overlap with any writer in this run is not detected.
 *
 * Per-function comments below that say "the RCU read-side lock
 * must be held" describe the concurrent-mode case; in exclusive
 * mode the caller's mutual exclusion replaces that requirement.
 *
 * Iterator Lifecycle and RCU Locking:
 *
 * The `cds_ft_iter` object can operate in two cache modes, selected via
 * cds_ft_iter_set_cache_mode():
 *
 * CDS_FT_ITER_CACHED (default):
 *
 * The iterator reuses its current position (the result node) to
 * accelerate subsequent sequential operations like cds_ft_next(),
 * cds_ft_prev(), or cds_ft_remove_all() (cds_ft_remove() and
 * cds_ft_replace() take the target node explicitly; see their note
 * below.)
 *
 * Because the position is an RCU-protected pointer, the RCU read-side
 * lock must be held CONTINUOUSLY between the operation that populates
 * the iterator (e.g., cds_ft_lookup) and the operation that reuses it.
 * If the RCU read-side lock is dropped, the cached position may
 * reference memory reclaimed after a grace period.
 *
 * If you need to drop the RCU read-side lock between operations, call
 * cds_ft_iter_bind_key() WHILE STILL HOLDING the lock, before dropping it:
 * cds_ft_iter_bind_key() snapshots the iterator's current key into its
 * own storage and clears the cached position.  The next position-reusing
 * operation (e.g., cds_ft_remove_all) then falls back to a safe, fresh
 * top-down traversal by that key.  Snapshotting (rather than merely
 * clearing the node pointer) matters because in a speculative group
 * configured with a leaf-key offset the cached key is held as a live
 * REFERENCE into the result node, valid only while the read-side lock
 * is held; bind copies it out before that node can be reclaimed.
 *
 * Example A (Continuous lock -- fast):
 * rcu_read_lock();
 * cds_ft_lookup(ft, iter);
 * cds_ft_remove(ft, iter, cds_ft_iter_node(iter)); // node still live; no re-descent
 * rcu_read_unlock();
 *
 * Example B (Dropped lock -- snapshot the key, re-derive from it):
 * Bind the key while STILL HOLDING the lock; after the unlock the cached
 * position is gone but the key is retained, so a position-reusing operation
 * re-descends by it.  cds_ft_remove_all takes no node argument and, with the
 * cached position cleared, derives everything from the key under the writer
 * mutex alone:
 *
 * rcu_read_lock();
 * cds_ft_lookup(ft, iter);
 * cds_ft_iter_bind_key(iter);         // snapshot the key + drop cached position
 * rcu_read_unlock();
 * ...
 * lock(&writer_mutex);
 * cds_ft_remove_all(ft, iter, &head); // re-descends by the snapshotted key
 * unlock(&writer_mutex);
 *
 * cds_ft_remove() and cds_ft_replace() take the target node explicitly
 * (@node / @old_node -- normally cds_ft_iter_node(iter), the iterator's
 * current position) and dereference it directly via @node->prev, with no
 * re-descent.  The RCU read-side lock must therefore be held CONTINUOUSLY
 * from when that node was obtained until the call, so the node has not
 * been reclaimed.  After a dropped lock the node is stale: re-look it up
 * (under the lock) and pass the fresh node -- bind resets the iterator's
 * cached position but cannot refresh a node you already hold.
 *
 * Note: CDS_FT_ITER_UNCACHED iterators do not require this -- the cached
 * position is discarded (and any referenced key snapshotted) automatically
 * after each operation.
 *
 * Debug validation (URCU_FRACTAL_TRIE_DEBUG_PATH):
 *
 * Building with URCU_FRACTAL_TRIE_DEBUG_PATH defined enables run-time
 * detection of stale cached positions.  Each position records an RCU
 * grace-period snapshot when populated and checks it on reuse; if a
 * full grace period has elapsed since the position was populated the
 * cached pointer may reference freed memory, so the program aborts
 * with a diagnostic.  Since struct cds_ft_iter is opaque, this option
 * does not affect the application ABI -- only the library needs to be
 * rebuilt.
 *
 * CDS_FT_ITER_UNCACHED:
 *
 * The iterator discards its cached position after each operation
 * returns. Every subsequent operation performs a fresh top-down
 * traversal from the root for the current key. The RCU read-side lock
 * only needs to be held during each individual operation and while
 * accessing the returned node -- it may be dropped between operations.
 *
 * This mode is suited for iteration patterns where the caller needs
 * to drop the RCU read-side lock between steps (e.g. to perform
 * blocking work or acquire other locks).
 *
 * Example C (Uncached mode, per-step locking with refcount):
 * cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);
 * rcu_read_lock();
 * cds_ft_for_each_rcu(ft, iter) {
 *         struct my_entry *e = cds_ft_entry(
 *                 cds_ft_iter_node(iter),
 *                 struct my_entry, ft_node);
 *         refcount_inc(&e->refcount);
 *         rcu_read_unlock();
 *
 *         do_blocking_work(e);
 *         refcount_dec(&e->refcount);
 *
 *         rcu_read_lock();
 * }
 * rcu_read_unlock();
 *
 * Include this file _after_ including your URCU flavor.
 */

#include <urcu/compiler.h>
#include <urcu-call-rcu.h>
#include <urcu-flavor.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque types forward declarations. */
struct cds_ft;
struct cds_ft_attr;
struct cds_ft_group_attr;
struct cds_ft_iter;
struct cds_ft_group;
struct cds_ft_cell;	/* opaque ordered-list cell handle (cell batch API) */

/* Fractal Trie lookup and mutation constants. */
#define CDS_FT_LEN_DEFAULT		SIZE_MAX

/* Fractal Trie attribute constants. */
#define CDS_FT_LEN_VARIABLE		SIZE_MAX
#define CDS_FT_MAX_LEN_UNLIMITED	SIZE_MAX
#define CDS_FT_KEY_MAP_SIZE		256

/*
 * Status codes returned by Fractal Trie operations.
 *
 * Success codes are >= 0. Error codes are < 0. Callers can test
 * (ret < 0) to catch all errors.
 */
enum cds_ft_status {
	/* Success return codes (>= 0). */
	CDS_FT_STATUS_OK			= 0,	/* Operation completed successfully. */
	CDS_FT_STATUS_NOT_FOUND			= 1,	/* No node found. */
	CDS_FT_STATUS_DUPLICATE_FOUND		= 2,	/* Duplicate node exists. */
	CDS_FT_STATUS_INTERNAL_MATCH		= 3,	/* Match ends at an internal node. */

	/* Error return codes (< 0). */
	CDS_FT_STATUS_INVALID_ARGUMENT_ERROR	= -1,	/* Invalid argument. */
	CDS_FT_STATUS_MEMORY_ERROR		= -2,	/* Memory allocation failure. */
	CDS_FT_STATUS_OVERFLOW_ERROR		= -3,	/* Buffer too small for key length. */
	CDS_FT_STATUS_BUSY_ERROR		= -4,	/* Resource busy. */
	CDS_FT_STATUS_POPULATED_ERROR		= -5,	/* Destination already populated. */
	CDS_FT_STATUS_INTEGRITY_ERROR		= -6,	/* Integrity verification failure. */
	CDS_FT_STATUS_NOT_SUPPORTED		= -7	/* Operation unavailable for this trie's configuration (e.g. node-cursor ordered walk on a list-off trie), or feature not compiled in. */
};

/*
 * Lookup optimization hint for a Fractal Trie group.
 *
 * Selects the trie's descent encoding to favour one lookup style
 * over the other.  Both styles return precise (verified) results;
 * the choice only affects per-step cost on the descent and which
 * lookup API is the cheapest to use.
 */
enum cds_ft_lookup_optimization {
	/*
	 * CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE (default):
	 *   Cheapest for cds_ft_lookup_candidate_key and
	 *   cds_ft_speculative_lookup_key.  cds_ft_eager_lookup_key
	 *   still works, at a small extra per-step cost.
	 */
	CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE = 0,

	/*
	 * CDS_FT_LOOKUP_OPTIMIZE_EAGER:
	 *   Cheapest for cds_ft_eager_lookup_key.
	 *   cds_ft_lookup_candidate_key and cds_ft_speculative_lookup_key
	 *   still work, without the speculative descent's savings.
	 */
	CDS_FT_LOOKUP_OPTIMIZE_EAGER,
};

/*
 * NUMA placement policy for a Fractal Trie group's internal-node
 * allocator.  Affects where the kernel places pages backing the
 * group's internal/compressed-node arenas.  No effect on caller-
 * provided external node storage.
 */
enum cds_ft_numa_policy {
	/*
	 * CDS_FT_NUMA_DEFAULT: honor an explicit per-process NUMA policy, but
	 * interleave the library's arenas when none is set.  Concretely:
	 *
	 *   - process set MPOL_BIND / PREFERRED / LOCAL (numactl --membind /
	 *     --preferred / --localalloc, set_mempolicy, numa_set_*): the
	 *     library applies no mbind() of its own; the kernel honors that
	 *     policy.  An explicit application intent is never overridden.
	 *   - process set MPOL_INTERLEAVE (numactl --interleave): upgraded to
	 *     the library's THP-friendly 2 MiB-granular interleave.
	 *   - no process policy set (would be first-touch): the library
	 *     interleaves its own arenas across the allowed nodes itself
	 *     (equivalent to CDS_FT_NUMA_INTERLEAVE).
	 *
	 * That last case is why this is NOT plain first-touch: a Fractal Trie
	 * is a shared, read-mostly, random-access structure, so first-touch
	 * piles it onto the builder's node -- one memory controller bottlenecks
	 * every other node's readers, and Linux auto-NUMA-balancing thrashes
	 * the unbound pages (PTE-scan + TLB-shootdown storms).  Interleaving the
	 * library's OWN arenas (it never touches application memory) avoids that
	 * by default while still deferring to any policy the application stated.
	 *
	 * This is the default value of a freshly-created group attr.  To force a
	 * placement regardless of process policy use INTERLEAVE or LOCAL; the env
	 * var CDS_FT_NUMA_INTERLEAVE=0 globally disables the library's mbind().
	 */
	CDS_FT_NUMA_DEFAULT = 0,

	/*
	 * CDS_FT_NUMA_INTERLEAVE: round-robin the trie's memory
	 * across the calling thread's allowed NUMA nodes in 2 MiB
	 * chunks.  Best for workloads
	 * with concurrent readers distributed across NUMA nodes: every
	 * reader sees the same balanced cross-node access pattern, so
	 * the worst-case cross-NUMA cost is bounded by (NR_NODES - 1) /
	 * NR_NODES of accesses.  Enables transparent hugepage collapse
	 * by keeping each 2 MiB chunk on a single node.
	 */
	CDS_FT_NUMA_INTERLEAVE,

	/*
	 * CDS_FT_NUMA_LOCAL: explicit first-touch placement -- pages
	 * land on whichever NUMA node first faults them (typically the
	 * writer thread).  Mechanically the library skips its own mbind();
	 * the distinction from DEFAULT is that LOCAL forces first-touch even
	 * when no process policy is set, whereas DEFAULT interleaves in that
	 * case.  Use LOCAL when you specifically want the trie on the faulting
	 * thread's node.
	 *
	 * Best for single-threaded workloads and for tries accessed
	 * exclusively by threads on the writer's node.  Multi-node-
	 * distributed readers pay full cross-NUMA latency for every
	 * access since all pages live on one node.
	 */
	CDS_FT_NUMA_LOCAL,
};

/*
 * Iterator cache mode.
 *
 * Controls whether the iterator reuses its cached position across
 * operations or re-descends from the root each time.
 */
enum cds_ft_iter_cache_mode {
	/*
	 * CDS_FT_ITER_CACHED (default):
	 *   The iterator reuses its current position between operations,
	 *   so next/prev/remove_all continue from it instead of
	 *   re-descending from the root.
	 *   The RCU read-side lock must be held CONTINUOUSLY between the
	 *   operation that populates the iterator and any operation that
	 *   reuses its position; otherwise the position may reference
	 *   memory reclaimed after a grace period.
	 */
	CDS_FT_ITER_CACHED = 0,

	/*
	 * CDS_FT_ITER_UNCACHED:
	 *   The iterator discards its cached position after each
	 *   operation returns. Every subsequent operation performs a
	 *   fresh top-down traversal from the root for the current key.
	 *   The RCU read-side lock must be held only during each
	 *   individual operation and while accessing the returned
	 *   node -- it may be dropped between operations.
	 *
	 *   This mode is suitable for iteration patterns where the
	 *   caller needs to drop the RCU read-side lock between steps
	 *   (e.g. to perform blocking work, acquire mutexes, or call
	 *   into subsystems that must not run in an RCU critical
	 *   section).
	 *
	 *   Example (uncached, per-step locking with refcount):
	 *
	 *     cds_ft_iter_set_cache_mode(iter,
	 *                               CDS_FT_ITER_UNCACHED);
	 *     rcu_read_lock();
	 *     cds_ft_for_each_rcu(ft, iter) {
	 *             struct my_entry *e = cds_ft_entry(
	 *                     cds_ft_iter_node(iter),
	 *                     struct my_entry, ft_node);
	 *             refcount_inc(&e->refcount);
	 *             rcu_read_unlock();
	 *
	 *             do_blocking_work(e);
	 *             refcount_dec(&e->refcount);
	 *
	 *             rcu_read_lock();
	 *     }
	 *     rcu_read_unlock();
	 */
	CDS_FT_ITER_UNCACHED = 1,
};

/*
 * Duplicate nodes with the same key are chained into a doubly-linked
 * list. The last item of this list has a NULL next pointer.
 * The node needs to be zeroed or initialized with cds_ft_node_init
 * before being inserted into a Fractal Trie.
 *
 * The prev pointer is library-internal and must not be accessed by the
 * application; walk duplicate chains through next, via
 * cds_ft_node_next_rcu() or the cds_ft_for_each_duplicate*() macros.
 *
 * Note that removal from a Fractal Trie does _not_ reset node->next,
 * because it can still be accessed by concurrent RCU readers. After
 * removal of a node, the user needs to re-initialize the node after
 * a grace period (e.g. via call_rcu() or synchronize_rcu()) before
 * being allowed to re-insert it.
 *
 * This structure is required to be naturally aligned.
 */
struct cds_ft_node {
	void *prev;			/* library-internal back-reference; do not access */
	struct cds_ft_node *next;
};

/*
 * Library-internal: bit 1 of cds_ft_node.next is a removal tombstone.  The
 * mutation side sets it when the node leaves the trie (so a position-based
 * remove can detect an already-removed node in O(1) without re-descending),
 * and every chain traversal masks it off.  Bit 0 is reserved for the
 * transactional engine's in-band proxy tag, so that the duplicate chain can be
 * committed as a concurrent transaction (a mid-commit "next" carries the proxy
 * on bit 0; the accessor below resolves it once the chain rides the engine).
 * cds_ft_node is naturally (pointer) aligned, so both low bits are always free.
 *
 * Applications must therefore not read cds_ft_node.next directly: walk
 * duplicate chains via cds_ft_node_next_rcu() (which masks the bit), or
 * the cds_ft_for_each_duplicate*() macros built on it, and treat a
 * removed node as opaque until re-initialized with cds_ft_node_init().
 */
#define CDS_FT_NODE_REMOVED_FLAG	2UL

/*
 * Library-internal: bit 0 of cds_ft_node.next is the transactional engine's
 * in-band proxy tag (equal to URCU_TXN_TAG; asserted in the library).  A
 * duplicate-chain successor read observes it only while a concurrent bulk
 * commit (e.g. a merge appending a src run at this chain's tail) is in flight.
 */
#define CDS_FT_NODE_TXN_PROXY_TAG	1UL

/*
 * Library-internal back-end for cds_ft_node_next_rcu()'s proxy path: resolve a
 * raw successor value that carries the engine proxy tag (an in-flight commit)
 * to the committed successor, then strip the removal tombstone.  Deliberately
 * NOT inlined -- a duplicate-chain walk is not a fast path, and out-of-lining it
 * keeps the transactional engine (and its headers) opaque to API users; reach
 * it only through cds_ft_node_next_rcu().
 */
extern struct cds_ft_node *cds_ft_node_next_resolve(void *raw);

static inline
struct cds_ft_node *_cds_ft_node_next_rcu(void *raw)
{
	if (caa_unlikely((uintptr_t) raw & CDS_FT_NODE_TXN_PROXY_TAG))
		return cds_ft_node_next_resolve(raw);
	return (struct cds_ft_node *) ((uintptr_t) raw & ~CDS_FT_NODE_REMOVED_FLAG);
}

/*
 * rcu_dereference of a duplicate's successor.  Common case: strip the removal
 * tombstone (CDS_FT_NODE_REMOVED_FLAG) inline.  If the value instead carries the
 * engine proxy tag (a bulk commit mid-flight on this chain's tail), resolve it
 * out of line via cds_ft_node_next_resolve().
 */
#define cds_ft_node_next_rcu(node)					\
	_cds_ft_node_next_rcu((void *) rcu_dereference((node)->next))

#define cds_ft_entry(ptr, type, member)		caa_container_of(ptr, type, member)

/*
 * cds_ft_node_init - Initialize Fractal Trie node.
 * @node: The node.
 */
static inline
void cds_ft_node_init(struct cds_ft_node *node)
{
	node->prev = NULL;
	node->next = NULL;
}

/*
 * External-node arena with end-of-zone over-read safety, suitable for
 * embedding application leaf structures (those containing a struct
 * cds_ft_node) when the caller needs a trailing readable pad past
 * stored keys for SIMD-friendly comparison.
 *
 * Pattern: app embeds struct cds_ft_node in its own leaf struct,
 * allocates each leaf from this arena via cds_ft_external_arena_alloc,
 * and inserts into the trie.  Every allocation is guaranteed at least
 * 32 bytes of safely-over-readable padding past its last byte, so the
 * library's SIMD leaf compare never faults reading past a stored key.
 *
 * Lifetime: all-or-nothing.  cds_ft_external_arena_destroy frees
 * every allocation served by the arena.  The caller is responsible
 * for ensuring no RCU readers (and no remaining trie keys) reach
 * any allocation before destroy.
 *
 * Allocations are 8-byte aligned and zero-initialised.
 *
 * NUMA: each range is placed with CDS_FT_NUMA_DEFAULT, deferring to the
 * process NUMA policy.  Under a process interleave policy (e.g. numactl
 * --interleave) the range is interleaved in 2 MiB chunks (MPOL_BIND per
 * chunk); under LOCAL/PREFERRED/BIND the kernel honors that at fault
 * time; with no process policy, pages fall back to first-touch.  Prefer
 * setting a process NUMA policy over first-touching from one thread,
 * which would pin the whole arena to a single node.
 *
 * Thread-safety: cds_ft_external_arena_alloc is internally
 * synchronised by a per-arena mutex, so multiple writer threads may
 * share a single arena.  cds_ft_external_arena_destroy is NOT
 * synchronised and must be called once, with no concurrent users.
 */
struct cds_ft_external_arena;
struct cds_ft_external_arena_attr;

/*
 * cds_ft_optimize - Per-arena page-size policy: trade RSS against throughput.
 *
 * CDS_FT_OPTIMIZE_THROUGHPUT (default): advise 2 MiB transparent hugepages.
 *   Cuts DTLB misses on large tries (measured +7% T1 / +14% T192 on the
 *   internal node arena).  A 2 MiB-advised range faults a whole hugepage on
 *   first touch, so a small / sparse trie pays a ~2 MiB RSS floor per range.
 *
 * CDS_FT_OPTIMIZE_RSS: advise 4 KiB pages (MADV_NOHUGEPAGE).  No hugepage
 *   floor -- a small or sparse trie faults only the pages it touches -- at the
 *   cost of more DTLB pressure once the working set is large.  Choose this for
 *   workloads with many small tries / arenas.
 */
enum cds_ft_optimize {
	CDS_FT_OPTIMIZE_THROUGHPUT = 0,
	CDS_FT_OPTIMIZE_RSS = 1,
};

/*
 * cds_ft_external_arena_attr_create - Allocate an external-arena attr.
 *
 * Initialized to the defaults (CDS_FT_OPTIMIZE_THROUGHPUT).  Destroy it with
 * cds_ft_external_arena_attr_destroy().  Returns CDS_FT_STATUS_OK on success,
 * or CDS_FT_STATUS_MEMORY_ERROR on allocation failure.
 */
enum cds_ft_status cds_ft_external_arena_attr_create(
		struct cds_ft_external_arena_attr **attr);

/*
 * cds_ft_external_arena_attr_destroy - Free an external-arena attr.
 *
 * Frees an attr allocated by cds_ft_external_arena_attr_create().  @attr may
 * be NULL, in which case this is a no-op.
 */
void cds_ft_external_arena_attr_destroy(struct cds_ft_external_arena_attr *attr);

/*
 * cds_ft_external_arena_attr_set_optimize - Set the arena's page-size policy.
 *
 * See enum cds_ft_optimize.  Returns CDS_FT_STATUS_OK, or
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @opt.
 */
enum cds_ft_status cds_ft_external_arena_attr_set_optimize(
		struct cds_ft_external_arena_attr *attr, enum cds_ft_optimize opt);

/*
 * cds_ft_external_arena_create - Create a leaf-allocation arena.
 *
 * @attr: external-arena attributes, or NULL for defaults
 *   (CDS_FT_OPTIMIZE_THROUGHPUT).  The arena copies what it needs; the caller
 *   may destroy @attr immediately afterwards.
 *
 * The arena starts empty; ranges are mmap'd on demand by
 * cds_ft_external_arena_alloc.
 *
 * Returns the arena handle on success, NULL on memory error.
 */
struct cds_ft_external_arena *cds_ft_external_arena_create(
		const struct cds_ft_external_arena_attr *attr);

/*
 * cds_ft_external_arena_alloc - Allocate from @arena.
 * @arena: Arena returned by cds_ft_external_arena_create.
 * @size:  Number of bytes the caller needs.  Rounded up internally
 *         to the next power of two (the slot's "size class");
 *         requests below 16 bytes are bumped to 16 bytes, requests
 *         above 4 MiB are rejected with NULL.
 *
 * Returns a pointer aligned to its size class with the slot
 * zero-initialised, or NULL if the size class is out of range or
 * a backing-range allocation fails.
 */
void *cds_ft_external_arena_alloc(struct cds_ft_external_arena *arena,
		size_t size);

/*
 * cds_ft_external_arena_free - Return @ptr to @arena's freelist.
 * @arena: Arena that allocated @ptr.
 * @ptr:   Pointer previously returned by cds_ft_external_arena_alloc.
 *
 * No size argument is needed.  The freed slot becomes available for
 * reuse by subsequent allocations.
 *
 * Caller must ensure no RCU reader still references @ptr (e.g.
 * via synchronize_rcu after removal from any published trie) -- the
 * arena does not defer reclamation.
 */
void cds_ft_external_arena_free(struct cds_ft_external_arena *arena,
		void *ptr);

/*
 * cds_ft_external_arena_destroy - Free @arena and every allocation.
 * @arena: Arena to destroy (may be NULL).
 *
 * Caller must ensure no RCU readers reach any allocation served
 * by this arena before calling.
 */
void cds_ft_external_arena_destroy(struct cds_ft_external_arena *arena);

/*
 * Key-based lookup API
 */

/*
 * cds_ft_lookup_candidate_key - Fast candidate lookup by key (no
 *                               validation).  The foundation of the
 *                               speculative path.
 * @ft: The Fractal Trie.
 * @key: Pointer to the key (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes.
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @key_readable_pad: Number of bytes past the end of @key (starting at
 *                    key + key_len) that are safe to load without
 *                    faulting.  0 means none is promised (the safe
 *                    default); a value >= 32 enables the fastest
 *                    comparison path.  cds_ft_external_arena reserves
 *                    such a trailing pad on every allocation.  Passing a
 *                    value larger than what is actually safe to read is
 *                    undefined behavior.
 * @result_node: Candidate node output. Set to a node if a candidate is
 *               found, or NULL if not found or on error.
 *
 * The fastest lookup: the returned node is a CANDIDATE that may not be
 * an exact match.  The caller MUST compare the returned node's key
 * against the lookup key to confirm; if the keys do not match, the
 * lookup key is not in the trie.  cds_ft_speculative_lookup_key wraps
 * this with the validating compare and is the recommended entry point
 * for callers that want an exact match.
 *
 * Returns CDS_FT_STATUS_OK on success (candidate found),
 * CDS_FT_STATUS_NOT_FOUND if no candidate, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_candidate_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node);

/*
 * cds_ft_speculative_lookup_key - Speculative descent + caller-side
 *                                 key validation.  The recommended
 *                                 lookup; matches the trie's default
 *                                 SPECULATIVE optimization.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes (must be the resolved byte count, not
 *           CDS_FT_LEN_DEFAULT).
 * @key_readable_pad: Padding past @key end (see cds_ft_lookup_candidate_key).
 * @key_offset: Byte offset from the (struct cds_ft_node *) stored in
 *              the trie to the start of the user-stored key bytes.
 *              Typically computed as
 *              offsetof(user_struct, key_field) -
 *              offsetof(user_struct, ft_node_field).
 * @result_node: Result output. Set to the matched node on OK, NULL on
 *               NOT_FOUND.
 *
 * Equivalent to cds_ft_lookup_candidate_key followed by a memcmp of
 * @key against the candidate's stored key bytes (at @key_offset).
 * cds_ft_eager_lookup_key returns the same result with library-side
 * validation instead.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
static inline
enum cds_ft_status cds_ft_speculative_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		size_t key_offset, struct cds_ft_node **result_node)
{
	struct cds_ft_node *found = NULL;
	enum cds_ft_status status;

	status = cds_ft_lookup_candidate_key(ft, key, key_len,
			key_readable_pad, &found);
	if (status != CDS_FT_STATUS_OK)
		return status;
	/* key_len == 0: nothing to validate (and @key may be NULL). */
	if (key_len != 0 && memcmp(key, (const uint8_t *) found + key_offset, key_len) != 0) {
		if (result_node)
			*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	if (result_node)
		*result_node = found;
	return CDS_FT_STATUS_OK;
}

/*
 * cds_ft_eager_lookup_key - Exact lookup by key with library-side
 *                           validation (precise descent).
 * @ft: The Fractal Trie.
 * @key: Pointer to the key (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes; same as cds_ft_speculative_lookup_key
 *           except CDS_FT_LEN_DEFAULT is also accepted (resolved to the
 *           trie's configured fixed length).
 * @key_readable_pad: Padding past @key end (see cds_ft_lookup_candidate_key).
 * @result_node: Node output. Set to the first node of the duplicate chain
 *               if a match is found, or NULL if not found or on error.
 *
 * Same result as cds_ft_speculative_lookup_key, but the library validates
 * the key during a precise descent, so no @key_offset is needed.  Fastest
 * when the group is tuned with CDS_FT_LOOKUP_OPTIMIZE_EAGER; on the default
 * speculative-tuned trie it still works but pays a small extra per-step
 * cost.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_eager_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_partial_key - Look up by key, find closest partial match.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @match_len: Length of the matching sub-key (output).
 *             The returned node's key is the first @match_len
 *             bytes of @key.
 * @result_node: Node output. Set to the first node of a duplicate chain if
 *               a match is found. If no node matches the full key, set to
 *               the closest ancestor (partial match). Set to NULL if no
 *               match is found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *match_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_longest_match_key - Find how far the key matches the trie.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @match_len: Length of the longest matching sub-key (output).
 *             This is the deepest position in the trie that matches
 *             a prefix of @key, including internal nodes with no
 *             external nodes attached. The matching sub-key is the
 *             first @match_len bytes of @key.
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if the longest match has an external node (status is
 *               CDS_FT_STATUS_OK). Set to NULL if the longest match
 *               ends at an internal node (status is
 *               CDS_FT_STATUS_INTERNAL_MATCH), if no match is found,
 *               or on error.
 *
 * Returns CDS_FT_STATUS_OK if the longest match has an external node.
 * Returns CDS_FT_STATUS_INTERNAL_MATCH if the longest match ends at
 * an internal node with no external nodes attached.
 * Returns CDS_FT_STATUS_NOT_FOUND if no prefix of @key matches any
 * node in the trie.
 * Returns a negative cds_ft_status on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_longest_match_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *match_len,
		struct cds_ft_node **result_node);

/*
 * Iterator-based lookup API
 *
 * These functions use a cds_ft_iter to hold input key, output key,
 * result node, status, and cached position. Set the input key
 * with cds_ft_iter_set_key() before calling. The @ft argument must be
 * the trie the iterator was created for (cds_ft_iter_create): the
 * descent uses @ft while the key handling uses the iterator's bound
 * trie, so passing a different trie mixes their key mappings and
 * produces undefined results. On return, the iterator
 * holds the result key (cds_ft_iter_get_key()), result node
 * (cds_ft_iter_node()), status (cds_ft_iter_status()), and cached
 * position. The status is also returned by the function for
 * convenience.
 *
 * Scope: a scoped iterator (cds_ft_iter_set_prefix_len()) confines
 * cds_ft_next() / _prev() / _first() / _last() and the range lookups to
 * keys sharing that prefix.
 *
 * The RCU read-side lock must be held while calling these functions
 * and while accessing the returned node or reusing the iterator's
 * cached position.
 */

/*
 * cds_ft_lookup - Look up a node by key (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result node, status,
 *        and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_partial - Look up by key, find closest partial match
 *                         (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result node, status,
 *        and cached position. The iterator's key length is set to
 *        the matching sub-key length, and the result node is the
 *        closest ancestor with external nodes.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_longest_match - Find how far the key matches the trie
 *                               (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator's key length is set to the longest
 *        matching sub-key length. This is the deepest position in
 *        the trie that matches a prefix of the input key, including
 *        internal nodes with no external nodes attached. The result
 *        node (cds_ft_iter_node()) is set to the first node of a
 *        duplicate chain if the longest match has an external node
 *        (status is CDS_FT_STATUS_OK), or NULL if the longest match
 *        ends at an internal node (status is
 *        CDS_FT_STATUS_INTERNAL_MATCH). The cached position is
 *        populated.
 *
 * Returns CDS_FT_STATUS_OK if the longest match has an external node.
 * Returns CDS_FT_STATUS_INTERNAL_MATCH if the longest match ends at
 * an internal node with no external nodes attached.
 * Returns CDS_FT_STATUS_NOT_FOUND if no prefix of the iterator's key
 * matches any node in the trie.
 * Returns a negative cds_ft_status on error.
 * The status is also stored in the iterator (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_longest_match(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_le - Look up first node with key <= iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key <= the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_ge - Look up first node with key >= iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key >= the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_lt - Look up first node with key < iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key < the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_gt - Look up first node with key > iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key > the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_first - Look up the node with the lowest key.
 * @ft: The Fractal Trie.
 * @iter: Iterator. On return, holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie holds no key within the iterator's scoped prefix (or is
 * empty), or a negative cds_ft_status on error. The status is also
 * stored in the iterator (cds_ft_iter_status()).  On NOT_FOUND or
 * error, the iterator's key length is left unchanged (the key buffer
 * past the scoped prefix is unspecified).
 */
enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_last - Look up the node with the greatest key.
 * @ft: The Fractal Trie.
 * @iter: Iterator. On return, holds the result key, key length,
 *        node, status, and cached position.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie holds no key within the iterator's scoped prefix (or is
 * empty), or a negative cds_ft_status on error. The status is also
 * stored in the iterator (cds_ft_iter_status()).  On NOT_FOUND or
 * error, the iterator's key length is left unchanged (the key buffer
 * past the scoped prefix is unspecified).
 */
enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_next - Find the next node in lexicographical order.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at the current node.
 *        On return, advanced to the next node. The iterator holds
 *        the result key, key length, node, status, and cached
 *        position.
 *
 * Equivalent to cds_ft_lookup_gt().
 */
static inline
enum cds_ft_status cds_ft_next(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return cds_ft_lookup_gt(ft, iter);
}

/*
 * cds_ft_prev - Find the previous node in lexicographical order.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at the current node.
 *        On return, moved to the previous node. The iterator holds
 *        the result key, key length, node, status, and cached
 *        position.
 *
 * Equivalent to cds_ft_lookup_lt().
 */
static inline
enum cds_ft_status cds_ft_prev(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return cds_ft_lookup_lt(ft, iter);
}

/*
 * cds_ft_for_each_rcu - Iterate through all (or prefix-scoped) nodes in key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 *
 * The iterator holds the current key (cds_ft_iter_get_key()),
 * node (cds_ft_iter_node()), and status (cds_ft_iter_status()) at
 * each step. Check (cds_ft_iter_status(iter) < 0) after the loop
 * to detect errors.
 *
 * Use cds_ft_iter_set_key and cds_ft_iter_set_prefix_len on @iter to
 * perform prefix-scoped iteration.
 *
 * An RCU read-side lock must be held when the macro invokes trie
 * operations (cds_ft_lookup_first, cds_ft_next) and while accessing
 * the returned node. For CDS_FT_ITER_CACHED iterators, this
 * means the RCU read-side critical section must span the entire
 * loop, since the cached position can reference memory reclaimed
 * after a grace period. For
 * CDS_FT_ITER_UNCACHED iterators, the lock may be dropped and
 * reacquired within the loop body, because the cached position is
 * discarded after each operation:
 *
 *   cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);
 *   rcu_read_lock();
 *   cds_ft_for_each_rcu(ft, iter) {
 *           ...access node under lock...
 *           rcu_read_unlock();
 *           ...blocking work...
 *           rcu_read_lock();
 *   }
 *   rcu_read_unlock();
 */
#define cds_ft_for_each_rcu(ft, iter)					\
	for (cds_ft_lookup_first((ft), (iter));				\
			cds_ft_iter_node(iter);				\
			cds_ft_next((ft), (iter)))

/*
 * cds_ft_for_each_entry_rcu - Iterate through all (or prefix-scoped) entries in key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 * @pos: Pointer to the containing structure (__typeof__(*(pos)) *),
 *       set at each iteration step.
 * @member: Name of the struct cds_ft_node member within @pos's type.
 *
 * This is the cds_ft_entry() equivalent of cds_ft_for_each_rcu().
 * @pos is only valid when cds_ft_iter_node(@iter) is non-NULL; it
 * must not be used after the loop exits.
 *
 * Best suited for tries with unique keys, where each key position
 * has exactly one node:
 *
 *   cds_ft_for_each_entry_rcu(ft, iter, entry, ft_node) {
 *           ...use entry directly...
 *   }
 *
 * When duplicates are present, prefer cds_ft_for_each_rcu() for
 * the outer trie loop combined with a
 * cds_ft_for_each_duplicate_entry*_rcu() inner loop (see
 * "Duplicate traversal macros" below).
 *
 * An RCU read-side lock must be held when the macro invokes trie
 * operations and while accessing the returned entry. See
 * cds_ft_for_each_rcu() for the locking rules for each cache mode.
 */
#define cds_ft_for_each_entry_rcu(ft, iter, pos, member)			\
	for (cds_ft_lookup_first((ft), (iter));					\
			cds_ft_iter_node(iter) != NULL ?			\
				((pos) = cds_ft_entry(cds_ft_iter_node(iter),	\
					__typeof__(*(pos)), member), 1) : 0;	\
			cds_ft_next((ft), (iter)))

/*
 * cds_ft_for_each_batched_rcu - In-order traversal, batched (amortized calls).
 * @ft: The Fractal Trie (struct cds_ft *).
 * @cell: Loop variable (const struct cds_ft_cell *), set to each ordered cell.
 * @buf: Caller scratch array of const struct cds_ft_cell *[@cap].
 * @cap: Capacity of @buf (a larger batch amortizes the call boundary further;
 *       32-64 is plenty).
 *
 * Batched in-order traversal built on the iterator-free cds_ft_cell_next_batch():
 * it refills @buf one library call per @cap cells and iterates it INLINE, so the
 * ordered cell-list walk pays the call boundary once per batch instead of once
 * per cell.  There is NO iterator object in scope, so none of the
 * stale-position hazards of an iterator apply.  From @cell, recover the
 * head node with cds_ft_cell_node(@cell, off) (off =
 * cds_ft_cell_node_offset(), cached once outside the loop) and the key
 * with cds_ft_cell_get_key(ft, @cell, ...).
 *
 * ORDERED-LIST ONLY: stepping in key order needs the cell list, so on a list-off
 * trie (cds_ft_group_attr_set_ordered_list(attr, false)) this loop iterates
 * NOTHING (cds_ft_cell_next_batch returns CDS_FT_STATUS_NOT_SUPPORTED, which the
 * macro cannot surface).  Code that may run on either kind of trie must branch
 * on cds_ft_group_ordered_list(group) and use cds_ft_for_each_rcu() when it is false.
 * Same RCU read-lock requirement as cds_ft_for_each_rcu().
 *
 * Despite the batching, this expands to a SINGLE flat loop: `break` and
 * `continue` in the body behave exactly as in a plain for loop (`break`
 * terminates the whole traversal, not just the current batch).
 *
 *   const struct cds_ft_cell *cell, *batch[64];
 *   size_t off = cds_ft_cell_node_offset();
 *   uint8_t key[256];		// >= cds_ft_group_max_key_len(group)
 *   size_t key_len;
 *   rcu_read_lock();
 *   cds_ft_for_each_batched_rcu(ft, cell, batch, 64) {
 *           struct cds_ft_node *node = cds_ft_cell_node(cell, off);
 *           cds_ft_cell_get_key(ft, cell, key, sizeof key, &key_len);
 *           ...use node and its key...
 *   }
 *   rcu_read_unlock();
 */
#define cds_ft_for_each_batched_rcu(ft, cell, buf, cap)				\
	for (struct { const struct cds_ft_cell *cur; size_t n, i; int started; } \
			_ftb = { NULL, 0, 0, 0 };				\
		(_ftb.i < _ftb.n ||						\
			((!_ftb.started || _ftb.cur != NULL) &&			\
			(cds_ft_cell_next_batch((ft), _ftb.cur, (buf), (cap),	\
				&_ftb.n, &_ftb.cur),				\
			 _ftb.started = 1, _ftb.i = 0, _ftb.n > 0))) &&		\
			(((cell) = (buf)[_ftb.i]), 1);				\
		_ftb.i++)

/*
 * cds_ft_for_each_reverse_batched_rcu - Reverse in-order traversal, batched.
 * The descending-key-order counterpart of cds_ft_for_each_batched_rcu(): visits
 * every cell from the largest key down, batching via cds_ft_cell_prev_batch().
 * Same arguments, RCU rules, and ORDERED-LIST-ONLY contract (list-off iterates
 * nothing; branch on cds_ft_group_ordered_list() and use cds_ft_for_each_reverse_rcu()).
 */
#define cds_ft_for_each_reverse_batched_rcu(ft, cell, buf, cap)			\
	for (struct { const struct cds_ft_cell *cur; size_t n, i; int started; } \
			_ftb = { NULL, 0, 0, 0 };				\
		(_ftb.i < _ftb.n ||						\
			((!_ftb.started || _ftb.cur != NULL) &&			\
			(cds_ft_cell_prev_batch((ft), _ftb.cur, (buf), (cap),	\
				&_ftb.n, &_ftb.cur),				\
			 _ftb.started = 1, _ftb.i = 0, _ftb.n > 0))) &&		\
			(((cell) = (buf)[_ftb.i]), 1);				\
		_ftb.i++)

/*
 * cds_ft_for_each_reverse_rcu - Iterate through all (or prefix-scoped) nodes in reverse key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 *
 * The iterator holds the current key (cds_ft_iter_get_key()),
 * node (cds_ft_iter_node()), and status (cds_ft_iter_status()) at
 * each step. Check (cds_ft_iter_status(iter) < 0) after the loop
 * to detect errors.
 *
 * Use cds_ft_iter_set_key and cds_ft_iter_set_prefix_len on @iter to
 * perform prefix-scoped iteration.
 *
 * An RCU read-side lock must be held when the macro invokes trie
 * operations and while accessing the returned node. See
 * cds_ft_for_each_rcu() for the locking rules for each cache mode.
 */
#define cds_ft_for_each_reverse_rcu(ft, iter)				\
	for (cds_ft_lookup_last((ft), (iter));				\
			cds_ft_iter_node(iter);				\
			cds_ft_prev((ft), (iter)))

/*
 * cds_ft_for_each_entry_reverse_rcu - Iterate through all (or prefix-scoped) entries in reverse key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 * @pos: Pointer to the containing structure (__typeof__(*(pos)) *),
 *       set at each iteration step.
 * @member: Name of the struct cds_ft_node member within @pos's type.
 *
 * This is the cds_ft_entry() equivalent of cds_ft_for_each_reverse_rcu().
 * @pos is only valid when cds_ft_iter_node(@iter) is non-NULL; it
 * must not be used after the loop exits.
 *
 * Best suited for tries with unique keys. When duplicates are
 * present, prefer cds_ft_for_each_reverse_rcu() for the outer
 * trie loop combined with a cds_ft_for_each_duplicate_entry*_rcu()
 * inner loop (see "Duplicate traversal macros" below).
 *
 * An RCU read-side lock must be held when the macro invokes trie
 * operations and while accessing the returned entry. See
 * cds_ft_for_each_rcu() for the locking rules for each cache mode.
 */
#define cds_ft_for_each_entry_reverse_rcu(ft, iter, pos, member)		\
	for (cds_ft_lookup_last((ft), (iter));					\
			cds_ft_iter_node(iter) != NULL ?			\
				((pos) = cds_ft_entry(cds_ft_iter_node(iter),	\
					__typeof__(*(pos)), member), 1) : 0;	\
			cds_ft_prev((ft), (iter)))

/*
 * Duplicate traversal macros
 *
 * Expected usage patterns:
 *
 * (1) Unique keys (or only the chain head matters):
 *     Use cds_ft_for_each_entry_rcu() directly:
 *
 *       cds_ft_for_each_entry_rcu(ft, iter, entry, ft_node) {
 *               ...use entry...
 *       }
 *
 * (2) Duplicate keys:
 *     Use cds_ft_for_each_rcu() for the outer trie traversal,
 *     then a cds_ft_for_each_duplicate_entry*_rcu() inner loop
 *     to walk the chain with typed entries:
 *
 *       cds_ft_for_each_rcu(ft, iter) {
 *               struct cds_ft_node *node = cds_ft_iter_node(iter);
 *               cds_ft_for_each_duplicate_entry_rcu(entry, node, ft_node) {
 *                       ...use entry...
 *               }
 *       }
 *
 *     The outer loop positions the iterator at each distinct key;
 *     extracting the containing struct there would be redundant
 *     since the inner loop re-derives it starting from the same
 *     chain head.
 */

/*
 * cds_ft_for_each_duplicate_rcu - Iterate through duplicates.
 * @pos: struct cds_ft_node *, start of duplicate list and loop cursor.
 *
 * Iterate through duplicates returned by cds_ft_lookup*()
 * This must be done while rcu_read_lock() is held.
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 * _NOT_ safe against node removal within iteration.
 */
#define cds_ft_for_each_duplicate_rcu(pos)				\
	for (; (pos) != NULL; (pos) = cds_ft_node_next_rcu(pos))

/*
 * cds_ft_for_each_duplicate_entry_rcu - Iterate through duplicate entries.
 * @pos: Pointer to the containing structure (__typeof__(*(pos)) *),
 *       set at each iteration step.
 * @node: struct cds_ft_node *, start of duplicate list and loop cursor.
 * @member: Name of the struct cds_ft_node member within @pos's type.
 *
 * This is the cds_ft_entry() equivalent of cds_ft_for_each_duplicate_rcu().
 * Iterate through duplicates returned by cds_ft_lookup*().
 * This must be done while rcu_read_lock() is held.
 * @pos is only valid when @node is non-NULL; it must not be used
 * after the loop exits.
 * _NOT_ safe against node removal within iteration.
 */
#define cds_ft_for_each_duplicate_entry_rcu(pos, node, member)		\
	for (; (node) != NULL ?						\
			((pos) = cds_ft_entry(node,			\
				__typeof__(*(pos)), member), 1) : 0;	\
			(node) = cds_ft_node_next_rcu(node))

/*
 * cds_ft_for_each_duplicate_safe_rcu - Iterate through duplicates.
 * @pos: struct cds_ft_node *, start of duplicate list and loop cursor.
 * @p: struct cds_ft_node *, temporary pointer to next.
 *
 * Iterate through duplicates returned by cds_ft_lookup*().
 * Safe against node removal within iteration.
 * This must be done while rcu_read_lock() is held.
 */
#define cds_ft_for_each_duplicate_safe_rcu(pos, p)			\
	for (; (pos) != NULL ?						\
			((p) = cds_ft_node_next_rcu(pos), 1) : 0;	\
			(pos) = (p))

/*
 * cds_ft_for_each_duplicate_entry_safe_rcu - Iterate through duplicate entries.
 * @pos: Pointer to the containing structure (__typeof__(*(pos)) *),
 *       set at each iteration step.
 * @node: struct cds_ft_node *, start of duplicate list and loop cursor.
 * @p: struct cds_ft_node *, temporary pointer to next.
 * @member: Name of the struct cds_ft_node member within @pos's type.
 *
 * This is the cds_ft_entry() equivalent of cds_ft_for_each_duplicate_safe_rcu().
 * Iterate through duplicates returned by cds_ft_lookup*().
 * Safe against node removal within iteration.
 * This must be done while rcu_read_lock() is held.
 * @pos is only valid when @node is non-NULL; it must not be used
 * after the loop exits.
 */
#define cds_ft_for_each_duplicate_entry_safe_rcu(pos, node, p, member)	\
	for (; (node) != NULL ?						\
			((pos) = cds_ft_entry(node,			\
				__typeof__(*(pos)), member),		\
			(p) = cds_ft_node_next_rcu(node), 1) : 0;	\
			(node) = (p))

/*
 * Mutation API
 *
 * "Atomically" in this section is with respect to concurrent RCU readers:
 * a reader observes either the complete prior state or the complete result,
 * never a partial one.  It does NOT make an operation atomic against other
 * concurrent updates; each operation below states the caller's
 * mutual-exclusion requirement.
 */

/*
 * cds_ft_insert - Insert @node at @key, allowing duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be inserted (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.  On failure, @node has not been published and is left
 * reusable: the same node may be passed to a subsequent insert
 * attempt.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): may run concurrently with
 *   cds_ft_insert and cds_ft_remove on the same trie, including on the
 *   same key.  Mutual exclusion against the remaining update operations
 *   (cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

/*
 * cds_ft_insert_unique - Insert @node at @key, without duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 * @result_node: Node output. Set to @node if successfully inserted. Set to
 *               the existing node if a duplicate exists. Set to NULL on
 *               error.
 *
 * Returns CDS_FT_STATUS_OK on success (node inserted, *@result_node
 * is @node). Returns CDS_FT_STATUS_DUPLICATE_FOUND if a duplicate
 * exists (*@result_node is the existing node). Returns a negative
 * cds_ft_status on error.  On failure (including DUPLICATE_FOUND),
 * @node has not been published and is left reusable for a subsequent
 * insert attempt.
 *
 * Pointers to existing nodes returned by this function are only safe to
 * dereference as long as the writer mutual exclusion is held, or if the
 * caller wraps the operation in their own RCU read-side critical
 * section.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every update operation (cds_ft_insert,
 *   cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove, cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node);

/*
 * cds_ft_insert_replace - Insert @node at @key, replacing the existing
 *                         duplicate chain.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be inserted (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 * @result_node: Node output. Set to the head of the previously existing
 *               duplicate chain if a key match existed, or NULL if no
 *               previous node existed at this key. Set to NULL on error.
 *
 * Atomically inserts @node at @key, replacing any existing duplicate
 * chain. If a chain was replaced, *@result_node points to the head
 * of the old chain. A grace period must be observed (e.g.,
 * synchronize_rcu, call_rcu) after success before reclaiming the old
 * chain's memory.
 *
 * Returns CDS_FT_STATUS_OK on success (node inserted, no prior node
 * existed). Returns CDS_FT_STATUS_DUPLICATE_FOUND on success when a
 * prior duplicate chain was replaced (*@result_node is the old head).
 * Returns CDS_FT_STATUS_BUSY_ERROR if the publish lost an
 * expected-value CAS or a guarded node was frozen: nothing was
 * published and the call may be retried.
 * Returns a negative cds_ft_status on error.  On error, @node has
 * not been published and is left reusable for a subsequent insert
 * attempt.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every update operation (cds_ft_insert,
 *   cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove, cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node);

/*
 * cds_ft_replace - Replace an existing node by a new node at the same key.
 * @ft: The Fractal Trie.
 * @iter: Identifies the key at which @old_node sits (set by the lookup
 *        that returned @old_node).  Used only for its key, to locate
 *        @old_node's slot.
 * @old_node: Node to replace (normally cds_ft_iter_node(iter)), currently
 *            present in the trie.  Dereferenced directly via
 *            @old_node->prev, so it must be live: hold the RCU read-side
 *            lock continuously from when @old_node was obtained until this
 *            call (concurrent mode; in exclusive mode the caller's mutual
 *            exclusion replaces that).  After a dropped lock, re-look up
 *            @old_node first -- cds_ft_iter_bind_key() snapshots the
 *            iterator's key to resume across the dropped lock, but
 *            cannot refresh a node you already hold.
 * @new_node: Node to insert in place of @old_node. Must be
 *            initialized with cds_ft_node_init() before this call.
 *
 * Atomically replaces @old_node with @new_node in the duplicate
 * chain at the iterator's key position. The @new_node inherits the
 * position of @old_node in the chain. A grace period must be
 * observed (e.g., synchronize_rcu, call_rcu) after success before
 * reclaiming @old_node memory.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * @old_node is not found at the iterator position, or a negative
 * cds_ft_status on error.  A publish that loses an expected-value CAS
 * is absorbed internally (the call re-derives and re-attempts), so it
 * is never reported as success without @new_node installed.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every update operation (cds_ft_insert,
 *   cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove, cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node);

/*
 * cds_ft_remove - Remove @node at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Identifies the key at which @node sits (set by the lookup that
 *        returned @node).  Used only for its key, to locate @node's
 *        slot.
 * @node: Node to remove (normally cds_ft_iter_node(iter)).  Dereferenced
 *        directly via @node->prev, so it must be live: hold the RCU
 *        read-side lock continuously from when @node was obtained until
 *        this call (concurrent mode; in exclusive mode the caller's
 *        mutual exclusion replaces that).  After a dropped lock, re-look
 *        up @node first -- cds_ft_iter_bind_key() snapshots the
 *        iterator's key to resume across the dropped lock, but cannot
 *        refresh a node you already hold.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the node is not found, or a negative cds_ft_status on error.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming @node memory.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): may run concurrently with
 *   cds_ft_insert and cds_ft_remove on the same trie, including on the
 *   same key.  Mutual exclusion against the remaining update operations
 *   (cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node);

/*
 * cds_ft_remove_all - Remove the entire duplicate chain at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Iterator position identifying the key.
 *        If the iterator holds a valid cached position from a prior
 *        lookup, remove_all reuses it to avoid a full traversal.
 *        WARNING (CDS_FT_ITER_CACHED only): If you intend to drop the
 *        RCU read-side lock between the positioning lookup and this
 *        call, you must call cds_ft_iter_bind_key() WHILE STILL HOLDING
 *        that lock to snapshot the key and clear the cached position;
 *        remove_all then re-descends by the key.  CDS_FT_ITER_UNCACHED
 *        iterators handle this automatically.
 * @result_node: Node output. Set to the head of the removed duplicate
 *               chain on success, or NULL if no node is found or on
 *               error.
 *
 * Removes the key and all associated duplicate nodes from the trie.
 * On success, *@result_node points to the head of the removed chain;
 * the caller can traverse it with cds_ft_for_each_duplicate_rcu()
 * under rcu_read_lock, or reclaim all nodes after a grace period.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming memory of any nodes in the chain.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node is found at the iterator position, or a negative
 * cds_ft_status on error.  CDS_FT_STATUS_MEMORY_ERROR reports an
 * allocation failure, and CDS_FT_STATUS_BUSY_ERROR a lock lost to a
 * concurrent writer, while restructuring the trie around the removed
 * key.  Both leave the key NOT removed (the chain is still reachable;
 * *@result_node is NULL) and both may be retried -- but only the
 * BUSY_ERROR retry can succeed without the caller first freeing memory.  An allocation
 * failure while pruning an already-emptied internal holder is NOT an
 * error: the key's removal is published before the holder is pruned, so
 * the removal has already succeeded (CDS_FT_STATUS_OK).  Completing the
 * prune is deferred to a later mutation through that slot.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every update operation (cds_ft_insert,
 *   cds_ft_insert_unique, cds_ft_insert_replace, cds_ft_replace,
 *   cds_ft_remove, cds_ft_remove_all) is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock, so any mix of update operations may be called concurrently.
 */
enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node);

/*
 * Graft and detach API
 *
 * These operations move entire sub-tries between trie instances
 * within the same group. They appear as single operations to
 * concurrent RCU readers: a reader sees either the complete sub-trie
 * or nothing, never a partial state.
 *
 * Graft attaches the content of a source trie at a key position in a
 * destination trie. Detach removes the sub-structure at a key position
 * and returns it as a new trie instance. These are the transplant and
 * split primitives for bulk operations.
 *
 * Root-level operations (key_len 0) work with both fixed-length and
 * variable-length key groups. Non-root operations (key_len > 0)
 * require a variable-length key group (CDS_FT_LEN_VARIABLE), because
 * a fixed-length group rejects any key whose length differs from the
 * group's fixed length.
 *
 * Efficient bulk-load pattern:
 *
 * A common usage is to populate a sub-trie offline and then graft it
 * into the main trie in a single O(1) operation. Because the source
 * trie has no concurrent readers or writers during population, no RCU
 * read-side lock and no mutual exclusion are needed for the inserts.
 * Concurrent RCU readers see either the pre-graft state or the
 * post-graft state, never a partial view -- so the duration of writer
 * mutual exclusion on the main trie is short and bounded (independent
 * of the number of nodes being grafted, modulo the O(depth) descent and
 * key-count propagation). This pattern is well suited for batch loading,
 * sharding, and periodic bulk updates where minimizing the writer
 * critical section on the live trie is important.
 *
 *   // Phase 1: populate offline, no locking needed.
 *   cds_ft_create(group, NULL, &staging);
 *   for each (key, node) in batch:
 *       cds_ft_insert(staging, key, key_len, node);
 *
 *   // Phase 2: graft into the live trie, O(1).  Under fine-grained
 *   // writer locking the source must be exclusive, so make it
 *   // exclusive once population is complete:
 *   cds_ft_make_exclusive(staging);
 *   // No writer mutex needed under CDS_FT_WRITER_LOCK_FINE (the
 *   // default): concurrent grafts into one live destination are
 *   // supported, and serializing them here would discard exactly the
 *   // parallelism fine-grained locking exists to provide.  Under
 *   // CDS_FT_WRITER_LOCK_COARSE the library serializes writers itself.
 *   cds_ft_graft(live_trie, prefix, prefix_len, staging);
 *   // staging is now empty but still valid; it can be reused
 *   // for the next batch or destroyed with cds_ft_destroy().
 *
 * Both source and destination tries must belong to the same group.
 * Update concurrency is documented per operation below and depends on
 * the group's writer strategy (cds_ft_group_attr_set_writer_strategy).
 * Do NOT call these operations from within
 * an RCU read-side critical section: they can block internally on
 * synchronize_rcu() to drain readers, which deadlocks (or never
 * completes) inside a read-side critical section. No RCU read-side lock
 * is required. The source trie must not be the same object as the
 * destination trie.
 *
 * Efficient bulk-removal pattern:
 *
 * Detach (or graft_swap) removes an entire sub-trie from the live
 * trie in a single O(1) operation and drains it before returning:
 * the operation performs one internal grace period, after which no
 * concurrent reader can still hold a reference to any node in the
 * detached sub-trie. The returned trie is therefore purely local --
 * the caller can iterate it and free all external nodes directly,
 * with no further synchronize_rcu and no grace period per individual
 * node. This reduces the cost of removing N nodes from N grace
 * periods (or N call_rcu callbacks) to the single grace period the
 * operation already performed, followed by a local iteration.
 *
 *   // Phase 1: detach from the live trie, O(1) under lock.
 *   lock(&writer_mutex);
 *   cds_ft_detach(live_trie, prefix, prefix_len, &detached);
 *   unlock(&writer_mutex);
 *
 *   // Phase 2: detach already drained in-flight readers and returned
 *   // a purely local trie, so free directly -- no synchronize_rcu here.
 *   while (cds_ft_lookup_first(detached, iter) == CDS_FT_STATUS_OK) {
 *           struct cds_ft_node *node, *p;
 *           cds_ft_remove_all(detached, iter, &node);
 *           cds_ft_for_each_duplicate_safe_rcu(node, p) {
 *                   free(cds_ft_entry(node, struct my_entry, ft_node));
 *           }
 *   }
 *   cds_ft_destroy(detached);
 */

/*
 * cds_ft_graft - Attach a source trie at a key position in the destination.
 * @dst_ft: Destination Fractal Trie.
 * @key: Key identifying the graft point (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: Graft at the root (NIL prefix).
 * @src_ft: Source Fractal Trie. Must be in the same group as @dst_ft.
 *          Under fine-grained writer locking (CDS_FT_WRITER_LOCK_FINE)
 *          @src_ft must be EXCLUSIVE (cds_ft_make_exclusive), i.e. have
 *          no concurrent readers or writers -- a live concurrent source
 *          is rejected with CDS_FT_STATUS_BUSY_ERROR (see below). On
 *          success, @src_ft becomes empty. The caller retains ownership
 *          of the (now empty) @src_ft object.
 *
 * Attaches the entire content of @src_ft at the position identified
 * by @key in @dst_ft as a single operation visible to concurrent RCU
 * readers. Fails if the destination already has any content (internal
 * or external nodes) at or below the graft point. On success,
 * @src_ft is left empty; it remains a valid trie in the group and
 * can be reused or destroyed.
 *
 * The operation validates that @key_len plus the maximum used key
 * length of @src_ft does not exceed the group's maximum key length.
 *
 * Root-level graft (@key_len 0) works with both fixed-length and
 * variable-length key groups. Non-root graft (@key_len > 0)
 * requires a variable-length key group (CDS_FT_LEN_VARIABLE).
 *
 * Source-exclusivity requirement (fine-grained locking).  A cross-trie
 * graft consumes @src_ft's whole content.  Under CDS_FT_WRITER_LOCK_FINE
 * it holds only @dst_ft's writer lock -- an exclusive @src_ft is private,
 * so no second lock is taken and there is no cross-trie deadlock.  A LIVE
 * (concurrent, non-exclusive) @src_ft would require a second lock with no
 * lock order and is therefore rejected with CDS_FT_STATUS_BUSY_ERROR
 * BEFORE anything is modified; make it exclusive first with
 * cds_ft_make_exclusive() (the "offline staging trie" pattern above
 * already builds @src_ft privately).  @dst_ft may be a live concurrent
 * trie.  On ANY failure -- BUSY, POPULATED, OVERFLOW or a memory error --
 * @src_ft is left UNCHANGED (the graft is assembled invisibly and only
 * commits once it cannot fail), so no content is ever stranded.
 *
 * Returns CDS_FT_STATUS_OK on success (@src_ft left empty).
 * Returns CDS_FT_STATUS_BUSY_ERROR if @src_ft is a live concurrent trie
 * under fine-grained writer locking (make it exclusive first).
 * Returns CDS_FT_STATUS_POPULATED_ERROR if the graft point is already
 * populated (destination has content at or below @key).
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the grafted keys would
 * exceed the group's maximum key length.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if the tries are not
 * in the same group, or if @src_ft is the same object as @dst_ft.
 * Returns a negative cds_ft_status on other errors.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): the DESTINATION may be a live
 *   trie carrying concurrent writers -- several cross-trie attaches
 *   (cds_ft_graft, cds_ft_graft_swap, cds_ft_merge_at) may run
 *   concurrently on the same destination.  The SOURCE must be EXCLUSIVE,
 *   which is what removes the need to exclude writers on it.  Exclusion
 *   against the point-update operations remains the caller's
 *   responsibility.
 *
 *   A ROOT-LEVEL attach into an EMPTY destination (key_len 0 /
 *   dst_key_len 0) arbitrates against those concurrent peers on the
 *   destination's root: a peer that populates the destination first wins,
 *   and this call then reports POPULATED_ERROR (cds_ft_graft) or falls
 *   through to the ordinary merge into a populated destination
 *   (cds_ft_merge_at).  A peer holding the root mid-attach yields
 *   CDS_FT_STATUS_BUSY_ERROR; the caller may retry.  In every outcome the
 *   losing side keeps its keys -- neither op's content is discarded.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock per trie, so any mix of update operations may be called
 *   concurrently.
 */
enum cds_ft_status cds_ft_graft(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft);

/*
 * cds_ft_graft_swap - Swap trie content with content at the graft point.
 * @dst_ft: Destination Fractal Trie.
 * @key: Key identifying the graft point (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: Graft at the root (NIL prefix).
 * @swap_ft: Fractal Trie to exchange content with. Must be in the same
 *           group as @dst_ft. Under fine-grained writer locking
 *           (CDS_FT_WRITER_LOCK_FINE) @swap_ft must be EXCLUSIVE
 *           (cds_ft_make_exclusive), i.e. have no concurrent readers or
 *           writers -- a live concurrent @swap_ft is rejected with
 *           CDS_FT_STATUS_BUSY_ERROR (see below). On entry, its content is
 *           grafted into @dst_ft at @key. On success, it receives the
 *           content that was previously at @key in @dst_ft, or is empty if
 *           the graft point had no content. The caller retains ownership.
 *
 * Exchanges the content at @key in @dst_ft with the content of
 * @swap_ft.  Concurrent RCU readers traversing @dst_ft observe
 * the swap atomically: they see either the old content or the new
 * content, never an empty intermediate state.
 *
 * On success, the previous content at the graft point (if any) is
 * placed into @swap_ft. The caller can check cds_ft_empty(@swap_ft)
 * to determine whether there was pre-existing content. If @swap_ft
 * is non-empty, the "Efficient bulk-removal pattern" described above
 * applies: after a single grace period, the caller can drain
 * @swap_ft locally without per-node grace periods.
 *
 * On success @swap_ft also inherits @dst_ft's access discipline: its
 * exclusive/concurrent mode is set to match @dst_ft (so an exclusive
 * @swap_ft may end up concurrent again if @dst_ft is a live trie).  This
 * changes the RCU rules the caller must follow on @swap_ft afterward
 * (including the drain above), so re-establish the desired mode with
 * cds_ft_make_exclusive() / cds_ft_make_concurrent() if it matters.
 *
 * Source-exclusivity requirement (fine-grained locking).  @swap_ft is the
 * consumed source of the exchange, so like cds_ft_graft it must be
 * EXCLUSIVE under CDS_FT_WRITER_LOCK_FINE: the op then holds only
 * @dst_ft's writer lock, with no cross-trie deadlock.  A LIVE (concurrent,
 * non-exclusive) @swap_ft is rejected with CDS_FT_STATUS_BUSY_ERROR before
 * anything is modified; make it exclusive first with cds_ft_make_exclusive().
 * @dst_ft may be a live concurrent trie.  On any failure -- BUSY, OVERFLOW
 * or a memory error -- both tries are left UNCHANGED.
 *
 * The operation validates that @key_len plus the maximum used key
 * length of @swap_ft does not exceed the group's maximum key length.
 *
 * Root-level swap (@key_len 0) works with both fixed-length and
 * variable-length key groups. Non-root swap (@key_len > 0)
 * requires a variable-length key group (CDS_FT_LEN_VARIABLE).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_BUSY_ERROR if @swap_ft is a live concurrent trie
 * under fine-grained writer locking (make it exclusive first).
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the grafted keys would
 * exceed the group's maximum key length.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if the tries are not
 * in the same group, or if @swap_ft is the same object as @dst_ft.
 * Returns a negative cds_ft_status on other errors.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): the DESTINATION may be a live
 *   trie carrying concurrent writers -- several cross-trie attaches
 *   (cds_ft_graft, cds_ft_graft_swap, cds_ft_merge_at) may run
 *   concurrently on the same destination.  The SOURCE must be EXCLUSIVE,
 *   which is what removes the need to exclude writers on it.  Exclusion
 *   against the point-update operations remains the caller's
 *   responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock per trie, so any mix of update operations may be called
 *   concurrently.
 */
enum cds_ft_status cds_ft_graft_swap(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *swap_ft);

/*
 * cds_ft_detach - Detach the sub-structure at a key position.
 * @ft: The Fractal Trie.
 * @key: Key identifying the detach point (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: Detach at the root (detach everything).
 * @result_ft: Output. On success, set to a new trie containing
 *             the content that was at @key. The new trie belongs to
 *             the same group. The caller takes ownership.
 *
 * Removes the sub-structure rooted at @key as a single operation
 * visible to concurrent RCU readers and returns it as a new trie
 * instance. The new trie supports all normal operations (lookup,
 * iteration, further detach, graft, destroy). The key lengths
 * within the detached trie are relative to the detach point
 * (i.e., the @key prefix is stripped).
 *
 * See "Efficient bulk-removal pattern" above for how to drain and
 * free the detached sub-trie with a single grace period.
 *
 * Root-level detach (@key_len 0) works with both fixed-length and
 * variable-length key groups. Non-root detach (@key_len > 0)
 * requires a variable-length key group (CDS_FT_LEN_VARIABLE).
 *
 * The returned trie is in exclusive mode: no RCU reader can be
 * inside it at return.  A subsequent graft of the detached trie
 * therefore needs no grace-period drain of its own, coalescing
 * detach+graft into a single grace period.  Callers that publish
 * the detached trie to concurrent readers must call
 * cds_ft_make_concurrent() first.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_NOT_FOUND if nothing exists at @key.
 * Returns a negative cds_ft_status on error (including memory
 * allocation failure for the result trie).
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every other update operation on the affected tries
 *   is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock per trie, so any mix of update operations may be called
 *   concurrently.
 */
enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft **result_ft);

/*
 * cds_ft_merge - Move @src_ft's content under @key into @dst_ft.
 * @dst_ft: Destination Fractal Trie.
 * @key: Key prefix that selects the @src_ft sub-trie to move and
 *       the @dst_ft attach point.  May be NULL if @key_len is 0.
 * @key_len: Length of @key in bytes.  Use 0 to merge every key in
 *           @src_ft into @dst_ft (whole-trie merge).
 * @src_ft: Source Fractal Trie.  Must belong to the same group as
 *          @dst_ft and must not equal @dst_ft.  Under the default
 *          CDS_FT_WRITER_LOCK_FINE it must be EXCLUSIVE
 *          (cds_ft_make_exclusive), exactly as for cds_ft_merge_at and
 *          cds_ft_graft -- a live concurrent source would need a second
 *          trie's writer lock with no lock order, and is rejected with
 *          CDS_FT_STATUS_BUSY_ERROR.  Detach the region first if the source
 *          must stay live to its readers: cds_ft_detach returns the detached
 *          trie exclusive, so detach-then-merge is the supported idiom.
 *
 * Moves @src_ft's content under prefix @key into @dst_ft at the
 * same prefix, preserving original key bytes.  @src_ft keys that
 * do not start with @key are left untouched in @src_ft.  Unlike
 * cds_ft_graft, @dst_ft may already contain entries under @key.
 *
 * The @key_len == 0 case is a whole-trie merge: every key in
 * @src_ft is moved into @dst_ft, and @src_ft becomes empty on
 * success.
 *
 * Atomicity: the entire merge is published to concurrent RCU
 * readers of @dst_ft as a single atomic transition.  A reader
 * sees either the complete pre-merge @dst_ft or the complete
 * post-merge @dst_ft -- never a partially-applied merge, and
 * never a half-spliced duplicate chain at any key.  Same-key
 * duplicate chains under @key are concatenated.  On an ordered-list
 * group the moved keys' ordered-list links are spliced as part of
 * the same atomic commit, so ordered iteration over @dst_ft observes
 * the merge atomically too.
 *
 * @src_ft's moved nodes are reclaimed under the usual RCU discipline;
 * @src_ft retains only the keys that do not start with @key.
 *
 * Returns CDS_FT_STATUS_OK on success (including the no-op case
 * where @src_ft has no content under @key).
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if either trie
 * pointer is NULL, if @dst_ft == @src_ft, if the tries are not in
 * the same group, or if @key_len exceeds the group's maximum key
 * length.
 * Returns a negative cds_ft_status on memory allocation failure, with both
 * tries left individually valid AND no moved entry stranded.  The merge is
 * built invisibly and the source is emptied only once the destination
 * placement is fully secured (every node drawn from a pre-filled reserve, the
 * single publish slot pre-allocated), so the source unlink is the last
 * fallible step and any allocation shortfall rolls the whole operation back
 * cleanly: the merge never leaks.
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): NOT concurrency-safe.  Mutual
 *   exclusion against every other update operation on the affected tries
 *   is the caller's responsibility.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock per trie, so any mix of update operations may be called
 *   concurrently.
 */
enum cds_ft_status cds_ft_merge(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft);

/*
 * cds_ft_merge_at - Cross-key variant of cds_ft_merge.
 * @dst_ft: Destination Fractal Trie.
 * @dst_key: Attach prefix in @dst_ft.  May be NULL if
 *           @dst_key_len is 0.
 * @dst_key_len: Length of @dst_key in bytes.
 * @src_ft: Source Fractal Trie.  Must belong to the same group as
 *          @dst_ft and must NOT equal @dst_ft -- a same-trie move is a
 *          REKEY (see cds_ft_rekey_graft / cds_ft_rekey_merge).  May be
 *          in either exclusive or concurrent mode.
 * @src_key: Source-side prefix selecting which @src_ft sub-trie to
 *           move.  May be NULL if @src_key_len is 0.
 * @src_key_len: Length of @src_key in bytes.
 *
 * Same as cds_ft_merge except that the source-side prefix
 * (@src_key) and destination-side prefix (@dst_key) may differ:
 * @src_ft's content under @src_key is moved into @dst_ft at
 * @dst_key.  Each moved key K = @src_key || S becomes
 * @dst_key || S in @dst_ft.  cds_ft_merge is the special case
 * where @dst_key == @src_key.
 *
 * Useful for re-keying patterns (archive moves, tier promotion,
 * partition rename) that cannot be expressed via the public
 * cds_ft_detach + cds_ft_graft pair on fixed-length groups.
 *
 * @src_ft must differ from @dst_ft (cross-trie only): the operation has the
 * same whole-operation-atomic contract as cds_ft_merge, with @src_key
 * selecting the source subtree to move and @dst_key serving as both the
 * destination attach point and the prefix that replaces @src_key on each
 * moved key.  To move a subtree to a new key WITHIN one trie, use the
 * dedicated rekey entry points cds_ft_rekey_graft (empty destination) /
 * cds_ft_rekey_merge (union into an occupied destination); a same-trie
 * cds_ft_merge_at is rejected with CDS_FT_STATUS_INVALID_ARGUMENT_ERROR.
 *
 * Returns the same statuses as cds_ft_merge.  In addition,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR is returned if either
 * @src_key_len or @dst_key_len exceeds the group's maximum key
 * length; if -- for a fixed-length key group -- @dst_key_len !=
 * @src_key_len (a fixed-length group accepts only keys of its fixed
 * length, so the moved keys keep that length only when the source and
 * destination prefixes are equally long); or if @src_ft == @dst_ft (a
 * same-trie move must use cds_ft_rekey_graft / cds_ft_rekey_merge).
 *
 * Update concurrency depends on the group's writer strategy
 * (cds_ft_group_attr_set_writer_strategy):
 *
 *   CDS_FT_WRITER_LOCK_FINE (the default): the DESTINATION may be a live
 *   trie carrying concurrent writers -- several cross-trie attaches
 *   (cds_ft_graft, cds_ft_graft_swap, cds_ft_merge_at) may run
 *   concurrently on the same destination.  The SOURCE must be EXCLUSIVE,
 *   which is what removes the need to exclude writers on it.  Exclusion
 *   against the point-update operations remains the caller's
 *   responsibility.
 *
 *   A ROOT-LEVEL attach into an EMPTY destination (key_len 0 /
 *   dst_key_len 0) arbitrates against those concurrent peers on the
 *   destination's root: a peer that populates the destination first wins,
 *   and this call then reports POPULATED_ERROR (cds_ft_graft) or falls
 *   through to the ordinary merge into a populated destination
 *   (cds_ft_merge_at).  A peer holding the root mid-attach yields
 *   CDS_FT_STATUS_BUSY_ERROR; the caller may retry.  In every outcome the
 *   losing side keeps its keys -- neither op's content is discarded.
 *
 *   CDS_FT_WRITER_LOCK_COARSE: writers serialize on one FT-wide writer
 *   lock per trie, so any mix of update operations may be called
 *   concurrently.
 */
enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len);

/*
 * cds_ft_rekey_graft - Move a subtree to a new, unoccupied key within one trie.
 * @ft: Fractal Trie to rekey in place.
 * @dst_key: Destination prefix -- the new key position.  Must be ABSENT
 *           (no content at or below it), else CDS_FT_STATUS_POPULATED_ERROR.
 * @dst_key_len: Length of @dst_key in bytes.
 * @src_key: Source prefix selecting the subtree to move.
 * @src_key_len: Length of @src_key in bytes.
 *
 * Moves @ft's content under @src_key to @dst_key within the SAME trie: each
 * moved key K = @src_key || S becomes @dst_key || S.  This is the same-trie
 * analog of cds_ft_graft (the destination must be empty) as cds_ft_merge_at is
 * of cds_ft_merge.  @src_key and @dst_key must be DISJOINT -- neither a prefix
 * of the other (else the move would be circular).
 *
 * The trie must be EAGER (not created with speculative leaf keys): a move
 * re-parents each leaf under @dst_key but cannot rewrite its app-owned stored
 * key, so a speculative trie is refused with CDS_FT_STATUS_INVALID_ARGUMENT_ERROR.
 *
 * The group must be VARIABLE-length, for the same reason cds_ft_detach and
 * cds_ft_graft take a non-root key only there: the move is staged through a
 * detached subtree, whose keys are stripped of the prefix and so are shorter
 * than a fixed-length group's one key length.  A fixed-length group is refused
 * with CDS_FT_STATUS_INVALID_ARGUMENT_ERROR and the trie is left untouched.
 *
 * Returns CDS_FT_STATUS_OK (including when @src_key is absent -- a no-op),
 * CDS_FT_STATUS_POPULATED_ERROR if @dst_key is occupied, CDS_FT_STATUS_MEMORY_ERROR,
 * or CDS_FT_STATUS_INVALID_ARGUMENT_ERROR (NULL @ft, a key length exceeding the
 * group maximum, a fixed-length group, overlapping keys, or a speculative trie),
 * CDS_FT_STATUS_OVERFLOW_ERROR (a moved key would exceed the group maximum
 * length).
 */
enum cds_ft_status cds_ft_rekey_graft(struct cds_ft *ft,
		const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *src_key, size_t src_key_len);

/*
 * cds_ft_rekey_merge - Move a subtree to a new key within one trie, unioning
 *                      into any content already at the destination.
 * @ft: Fractal Trie to rekey in place.
 * @dst_key: Destination prefix -- the new key position.  May already hold
 *           content: the moved subtree is unioned into it and same-key
 *           duplicate chains are concatenated (as cds_ft_merge_at does).
 * @dst_key_len: Length of @dst_key in bytes.
 * @src_key: Source prefix selecting the subtree to move.
 * @src_key_len: Length of @src_key in bytes.
 *
 * The merge (occupied-destination) counterpart of cds_ft_rekey_graft, standing
 * to it as cds_ft_merge_at stands to cds_ft_graft.  Same disjoint-key, EAGER and
 * variable-length-group requirements.  Returns the same statuses as
 * cds_ft_rekey_graft except it never returns CDS_FT_STATUS_POPULATED_ERROR (an
 * occupied @dst_key is merged into).
 */
enum cds_ft_status cds_ft_rekey_merge(struct cds_ft *ft,
		const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *src_key, size_t src_key_len);

/*
 * Trie lifecycle
 *
 * A group is a set of Fractal Trie instances that share allocation
 * arenas and configuration (key length, lookup optimization, NUMA /
 * page-size policy, ordered-list mode).  Groups exist for the bulk
 * operations (cds_ft_graft, cds_ft_graft_swap, cds_ft_detach,
 * cds_ft_merge): those move or combine whole sub-tries between tries,
 * which is sound and cheap only when the tries share a group's arenas
 * and key mapping.  Create a group with cds_ft_group_create(), then
 * create tries within it with cds_ft_create().
 */

/*
 * _cds_ft_group_create - API used by the cds_ft_group_create wrappers
 * below.  Do not use directly.
 */
enum cds_ft_status _cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor);

/*
 * cds_ft_group_create_flavor - Create a Fractal Trie group tied to an
 *                              explicit RCU flavor.
 * @attr: Fractal Trie attributes.
 * @result_ft_group: Fractal Trie group output. Set to the newly created
 *                   trie group on success, or NULL on error.
 * @flavor: Flavor of liburcu used to synchronize the group's tries
 *          (e.g. &urcu_memb_flavor, &urcu_qsbr_flavor).
 *
 * The @attr pointer is used to specify the Fractal Trie attributes. If
 * NULL, use default attribute values. The @attr can be destroyed
 * by the caller immediately after cds_ft_group_create_flavor()
 * returns. The caller keeps ownership of @attr. Default attributes
 * select a variable key length.
 *
 * This is the form to use with the modern <urcu/urcu-*.h> flavor
 * headers, which clear the URCU API mapping at the end of the header
 * (urcu/map/clear.h), making cds_ft_group_create() unavailable.
 * Mirrors cds_lfht_new_flavor().
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
static inline
enum cds_ft_status cds_ft_group_create_flavor(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	return _cds_ft_group_create(attr, result_ft_group, flavor);
}

#ifdef URCU_API_MAP
/*
 * cds_ft_group_create - Create a Fractal Trie group.
 * @attr: Fractal Trie attributes.
 * @result_ft_group: Fractal Trie group output. Set to the newly created
 *                   trie group on success, or NULL on error.
 *
 * Same as cds_ft_group_create_flavor(), binding the group to the RCU
 * flavor selected by the URCU API mapping of the previously included
 * flavor header (legacy <urcu.h> / <urcu-qsbr.h>-style include, or
 * URCU_API_MAP defined before a modern flavor header).
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
static inline
enum cds_ft_status cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group)
{
	return _cds_ft_group_create(attr, result_ft_group, &rcu_flavor);
}
#endif /* URCU_API_MAP */

/*
 * cds_ft_group_destroy - Destroy a Fractal Trie group.
 * @ft_group: The Fractal Trie group.
 *
 * Return CDS_FT_STATUS_OK on success, or CDS_FT_STATUS_BUSY_ERROR
 * if it is not possible to destroy the group because trie instances
 * created from it still exist.
 */
enum cds_ft_status cds_ft_group_destroy(struct cds_ft_group *ft_group);

/*
 * cds_ft_create - Create a Fractal Trie.
 * @ft_group: The Fractal Trie group.
 * @attr: Per-instance Fractal Trie attributes (may be NULL for defaults).
 * @result_ft: Fractal Trie output. Set to the newly created trie on
 *          success, or NULL on error.
 *
 * If @attr is NULL, defaults are used (exclusive = false).  The @attr
 * can be destroyed by the caller immediately after cds_ft_create()
 * returns; the caller keeps ownership of @attr.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_create(struct cds_ft_group *ft_group,
		const struct cds_ft_attr *attr,
		struct cds_ft **result_ft);

/*
 * cds_ft_destroy - Destroy a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * There should be no more concurrent insert, delete, nor look-up
 * performed on the Fractal Trie while it is being destroyed (ensured
 * by the caller).
 *
 * The trie should be drained first: destroying a non-empty trie does
 * not reclaim its remaining internal nodes and ordered-list cells, and
 * leaves the application's external nodes unreachable
 * with their linkage fields dangling.  See the "Efficient
 * bulk-removal pattern" above for draining a trie with a single
 * grace period.
 */
void cds_ft_destroy(struct cds_ft *ft);

/*
 * Trie properties
 */

/*
 * cds_ft_empty - Test whether a Fractal Trie contains any nodes.
 * @ft: The Fractal Trie.
 *
 * Returns true if the trie contains no nodes, false otherwise.
 *
 * An RCU read-side lock must be held while calling this function.
 * The result is a snapshot: concurrent updates may change the
 * emptiness state at any time.
 */
bool cds_ft_empty(struct cds_ft *ft);

/*
 * cds_ft_count_keys - Return the number of unique keys in a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the number of distinct keys that have at least one external
 * node. Duplicates at the same key are counted as one.
 *
 * O(1) when the group enables order statistics
 * (cds_ft_group_attr_set_rank_stats); otherwise (the default) O(size) -- a
 * structural enumeration of the trie.  The RCU read-side lock must be held
 * while calling this function. Concurrent updates may occur, so the result is
 * an approximation when updates are in progress.
 */
unsigned long cds_ft_count_keys(struct cds_ft *ft);

/*
 * cds_ft_count_keys_prefix - Return the number of unique keys under a prefix.
 * @ft: The Fractal Trie.
 * @prefix: The key prefix to count under.
 * @prefix_len: Length of the prefix in bytes. Use 0 to count all keys
 *              (equivalent to cds_ft_count_keys).
 *
 * Returns the number of distinct keys whose key starts with @prefix,
 * in O(prefix_len) (it reads a maintained count, not a full scan).
 *
 * O(prefix_len) when the group enables order statistics
 * (cds_ft_group_attr_set_rank_stats); otherwise (the default) O(size of the
 * matching subtree) -- the prefix descent is the same, but the count is then a
 * structural enumeration.  The RCU read-side lock must be held while calling
 * this function.
 */
unsigned long cds_ft_count_keys_prefix(struct cds_ft *ft,
		const uint8_t *prefix, size_t prefix_len);

/*
 * cds_ft_lookup_nth - Lookup the nth key in forward (smallest-first) order.
 * @ft: The Fractal Trie.
 * @iter: Iterator (must be created via cds_ft_iter_create).
 * @n: 0-indexed rank from the first (smallest) key.
 *
 * Finds the key at rank @n in the trie's sorted key order. On success the
 * iterator points to the first external node of the nth key and the
 * result key is accessible via cds_ft_iter_get_key().
 *
 * Returns CDS_FT_STATUS_OK on success or CDS_FT_STATUS_NOT_FOUND if
 * @n >= the number of keys. O(depth) when the group enables order statistics
 * (cds_ft_group_attr_set_rank_stats); otherwise (the default) O(n), iterating
 * forward from the first key.
 * The RCU read-side lock must be held.
 */
enum cds_ft_status cds_ft_lookup_nth(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n);

/*
 * cds_ft_lookup_nth_last - Lookup the nth key from the last (largest) key.
 * @ft: The Fractal Trie.
 * @iter: Iterator (must be created via cds_ft_iter_create).
 * @n: 0-indexed rank from the last (largest) key. 0 is the largest key.
 *
 * With order statistics enabled (cds_ft_group_attr_set_rank_stats) this
 * descends from the right (largest children first) using per-node key
 * counters, so concurrent updates to the low end of the key space do
 * not affect the traversal, in O(depth); otherwise (the default) it positions
 * at the last key and steps back @n, in O(n).  On success the
 * iterator points at the nth-from-last key (cds_ft_iter_node() /
 * cds_ft_iter_get_key()); on NOT_FOUND it is left unpositioned
 * (cds_ft_iter_node() returns NULL).
 * Returns CDS_FT_STATUS_NOT_FOUND if @n >= the number of keys.
 * The RCU read-side lock must be held.
 */
enum cds_ft_status cds_ft_lookup_nth_last(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n);

/*
 * cds_ft_iter_skip_forward - Skip forward by @n keys from the current position.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at a valid key.
 * @n: Number of keys to skip forward. 0 is a no-op.
 *
 * With order statistics enabled (cds_ft_group_attr_set_rank_stats) this
 * traverses locally from the current position, touching only nodes
 * between the start and end positions, so concurrent mutations in
 * unrelated key ranges do not affect the result, in O(depth); otherwise
 * (the default) it advances @n keys via cds_ft_next, in O(n).  On success the
 * iterator is repositioned at the target
 * key (cds_ft_iter_node() / cds_ft_iter_get_key()); on NOT_FOUND it is
 * left unpositioned (cds_ft_iter_node() returns NULL).  Returns
 * CDS_FT_STATUS_NOT_FOUND if the target is out of range. The RCU
 * read-side lock must be held.
 */
enum cds_ft_status cds_ft_iter_skip_forward(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n);

/*
 * cds_ft_iter_skip_reverse - Skip backward by @n keys from the current position.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at a valid key.
 * @n: Number of keys to skip backward. 0 is a no-op.
 *
 * With order statistics enabled (cds_ft_group_attr_set_rank_stats) this
 * traverses locally from the current position, touching only nodes
 * between the start and end positions, so concurrent mutations in
 * unrelated key ranges do not affect the result, in O(depth); otherwise
 * (the default) it steps back @n keys via cds_ft_prev, in O(n).  On success the
 * iterator is repositioned at the target
 * key (cds_ft_iter_node() / cds_ft_iter_get_key()); on NOT_FOUND it is
 * left unpositioned (cds_ft_iter_node() returns NULL).  Returns
 * CDS_FT_STATUS_NOT_FOUND if @n exceeds the number of preceding keys.
 * The RCU read-side lock must be held.
 */
enum cds_ft_status cds_ft_iter_skip_reverse(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n);

/*
 * cds_ft_count_entries - Return the number of external nodes in a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the total number of external nodes (user-visible nodes)
 * currently stored in the trie, including duplicates. Each node in a
 * duplicate chain is counted individually.
 *
 * This function has O(n) time complexity where n is the number of
 * external nodes. The RCU read-side lock must be held while calling
 * this function. Concurrent updates may occur during the traversal,
 * so the result is an approximation when updates are in progress.
 */
unsigned long cds_ft_count_entries(struct cds_ft *ft);

/*
 * cds_ft_max_used_key_len - Return the maximum key length inserted.
 * @ft: The Fractal Trie.
 *
 * Returns the maximum key length that has been successfully inserted
 * into this trie instance. This is a conservative (over-)estimate:
 * it is updated on insert but not decremented on remove or detach.
 * Use cds_ft_recompute_stats() to obtain the exact value.
 *
 * Returns 0 if the trie is empty or has never had a key inserted.
 *
 * This function uses a relaxed atomic load and does not require
 * the RCU read-side lock to be held.
 */
size_t cds_ft_max_used_key_len(const struct cds_ft *ft);

/*
 * cds_ft_recompute_stats - Recompute conservative statistics.
 * @ft: The Fractal Trie.
 *
 * Iterate through the trie to recompute the exact maximum used key
 * length. The caller must hold the RCU read-side lock (for
 * iteration) and ensure mutual exclusion with other writers (because
 * this operation can lower the value, unlike insert which only ever
 * increases it).
 *
 * Typically used after a batch of removals or a detach operation
 * when the caller needs an accurate max_used_key_len for a
 * subsequent graft validation.
 *
 * Returns CDS_FT_STATUS_OK on success.
 */
enum cds_ft_status cds_ft_recompute_stats(struct cds_ft *ft);

/*
 * Group queries
 */

/*
 * cds_ft_group_key_len - Return the key length configured for a group.
 * @group: The Fractal Trie group.
 *
 * Returns the fixed key length if the group is in fixed-length mode.
 * Returns CDS_FT_LEN_VARIABLE if the group supports variable-length keys.
 */
size_t cds_ft_group_key_len(const struct cds_ft_group *group);

/*
 * cds_ft_group_max_key_len - Return the maximum key length allowed by a group.
 * @group: The Fractal Trie group.
 *
 * Returns the maximum length (in bytes) of any key that can be stored
 * in a trie of this group.
 *
 * This value is intended for use by callers to allocate buffers for
 * output parameters (e.g., result_key). Even if the group is configured
 * as "unlimited," this function returns a finite, implementation-defined
 * maximum.
 */
size_t cds_ft_group_max_key_len(const struct cds_ft_group *group);

/*
 * cds_ft_group_key_map - Return the key map configured for a group.
 * @group: The Fractal Trie group.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 *
 * The key map is the order-preserving byte permutation set at group
 * creation; see cds_ft_group_attr_set_key_map for its meaning and
 * constraints.  Returns CDS_FT_STATUS_OK if a non-identity map is
 * configured (the output arrays are populated), or CDS_FT_STATUS_NOT_FOUND
 * if the key map is the identity function.
 */
enum cds_ft_status cds_ft_group_key_map(const struct cds_ft_group *group, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key);

/*
 * cds_ft_group_ordered_list - Whether a group maintains the key-ordered cell list.
 * @group: The Fractal Trie group.
 *
 * True when the group enabled the ordered list (the default;
 * cds_ft_group_attr_set_ordered_list(attr, false) disables it).  Immutable for
 * the life of the group.  It is the precondition for the cell-cursor ordered
 * walk (cds_ft_cell_next_batch / cds_ft_cell_prev_batch and the
 * cds_ft_for_each_batched_rcu macros): a list-off group has no cell list to
 * step, so generic code should branch to cds_ft_for_each_rcu() when this
 * returns false.  (Note: cds_ft_node_get_key() is broader -- it also works on a
 * list-off trie that has an in-leaf key, since materializing ONE key needs no
 * stepping.)
 */
bool cds_ft_group_ordered_list(const struct cds_ft_group *group);

/*
 * cds_ft_group_rank_stats - Whether a group maintains per-node order-statistics
 *   key counts.
 * @group: The Fractal Trie group.
 *
 * True when the group enabled order statistics
 * (cds_ft_group_attr_set_rank_stats(attr, true); the default is OFF).
 * Immutable for the life of the group.  When true the rank / select / count
 * queries (cds_ft_count_keys / _prefix, cds_ft_lookup_nth / _last,
 * cds_ft_iter_skip_forward / _reverse) read a maintained per-node count and
 * run in O(1) / O(depth); when false they remain correct but fall back to a
 * full enumeration (count) or first/last + next/prev iteration (select / skip).
 */
bool cds_ft_group_rank_stats(const struct cds_ft_group *group);

/*
 * Attributes
 */

/*
 * cds_ft_group_attr_create - Create a Fractal Trie attribute structure.
 * @result: Attribute output. Set to the newly created attribute
 *          structure on success, or NULL on error.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_group_attr_create(struct cds_ft_group_attr **result);

/*
 * cds_ft_group_attr_destroy - Destroy a Fractal Trie attribute structure.
 * @attr: Fractal Trie attributes.
 */
void cds_ft_group_attr_destroy(struct cds_ft_group_attr *attr);

/*
 * cds_ft_group_attr_set_key_len - Set Fractal Trie key length attribute.
 * @attr: Fractal Trie attributes.
 * @key_len: Key length.
 * - CDS_FT_LEN_VARIABLE for variable length keys.
 * - > 0 for fixed length keys.
 * - 0 for a trie fixed to NIL keys only.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_group_attr_set_key_len(struct cds_ft_group_attr *attr, size_t key_len);

/*
 * cds_ft_group_attr_set_max_key_len - Set the maximum key length attribute.
 * @attr: Fractal Trie attributes.
 * @max_key_len: Maximum key length in bytes:
 * - n > 0: Limits keys to a maximum of n bytes.
 * - 0: Limits keys to length 0 (only NIL keys allowed).
 * - CDS_FT_MAX_LEN_UNLIMITED: No user-defined limit.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @max_key_len
 * exceeds implementation-defined limits.
 */
enum cds_ft_status cds_ft_group_attr_set_max_key_len(struct cds_ft_group_attr *attr, size_t max_key_len);

/*
 * cds_ft_group_attr_set_key_map - Set Fractal Trie key map attribute.
 * @attr: Fractal Trie attributes.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 *
 * Both arrays must be permutations of 0..CDS_FT_KEY_MAP_SIZE-1, and
 * @ordinal_to_key must be the exact inverse of @key_to_ordinal (i.e.
 * ordinal_to_key[key_to_ordinal[i]] == i for all i).  This bijection
 * defines the trie's key ordering; maps that are not exact inverses
 * are rejected.
 *
 * The map is an order-preserving bijection only: it re-orders the byte
 * alphabet but never folds distinct bytes onto one ordinal.  Folding
 * (e.g. case-insensitive matching) is intentionally unsupported here:
 * the speculative lookup fast path validates by comparing external key
 * bytes directly (memcmp against the candidate's stored key), so a
 * non-injective map would let descent reach a leaf that validation then
 * rejects.  For case-folding or other canonicalization, normalize keys
 * to a canonical form in the application before insert and lookup,
 * leaving this map a bijection (or identity).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if either pointer is
 * NULL or the maps are not exact inverse permutations.
 */
enum cds_ft_status cds_ft_group_attr_set_key_map(struct cds_ft_group_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key);

/*
 * cds_ft_group_attr_set_lookup_optimization - Select the trie's
 *                                             descent encoding for
 *                                             this group.
 * @attr: Fractal Trie group attributes.
 * @opt: One of enum cds_ft_lookup_optimization
 *       (SPECULATIVE or EAGER).
 *
 * SPECULATIVE is the default for a freshly created group attr and the
 * faster descent.  EAGER does a strict per-step exact compare at every
 * node (via cds_ft_eager_lookup_key); select it explicitly if you need
 * that.
 * Both modes return identical results and differ only in descent cost.
 *
 * Requirement: on 64-bit architectures where SPECULATIVE uses the
 * skip-compressed encoding (which stores data in unused high pointer
 * bits), the external node pointers (struct cds_ft_node *) stored in
 * the trie must not carry metadata in their upper bits -- strip any
 * pointer authentication (AArch64 PAC) or memory tagging (MTE)
 * signature before passing the pointer to the insertion API.
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @opt value.
 * Never returns NOT_SUPPORTED: both modes work on every architecture.
 */
enum cds_ft_status cds_ft_group_attr_set_lookup_optimization(
		struct cds_ft_group_attr *attr,
		enum cds_ft_lookup_optimization opt);

/*
 * cds_ft_group_attr_set_speculative_key_offset - Provide the leaf-key
 *                                                offset for speculative
 *                                                inequality lookups.
 * @attr: Fractal Trie group attributes.
 * @key_offset: Byte offset from the (struct cds_ft_node *) stored in the
 *              trie to the start of the caller-stored key bytes -- the
 *              same value passed to cds_ft_speculative_lookup_key,
 *              typically offsetof(user_struct, key_field) -
 *              offsetof(user_struct, ft_node_field).
 *
 * KEY REPRESENTATION CONTRACT: the bytes at @key_offset must be exactly
 * the bytes the application passed to cds_ft_insert() / the lookup APIs
 * for this node -- NOT the application's native scalar.  For integer
 * keys that is the big-endian form emitted by the cds_ft_u64_to_key()
 * API family, not the host-order integer.  The library applies the group's
 * key map to these bytes when copying them into the iterator's result
 * key, so a non-identity cds_ft_key_map is supported (the stored key is
 * remapped to ordinal order on copy); an identity map is a plain copy.
 *
 * Their byte order must therefore match what the trie was built with
 * (unlike the candidate / speculative point lookups, where the
 * caller-side validation never makes the library interpret the stored
 * bytes).
 *
 * Optional, and only meaningful on a SPECULATIVE group on a
 * skip-compressed-capable arch (64-bit).  When provided, the
 * ordered-inequality lookups (cds_ft_lookup_ge/gt/le/lt,
 * cds_ft_next/prev) capture their result key by copying it from the
 * matched leaf -- which already stores the full key for caller-side
 * validation -- which is faster than rebuilding it from the trie
 * structure during descent.  Because those lookups read the bytes from
 * the live leaf, the key bytes at @key_offset must be present before
 * the node is inserted and must not change while the node is in the
 * trie (the same stability requirement
 * cds_ft_group_attr_set_key_len_offset states for the length).
 *
 * When not set (or on an EAGER / non-skip-compressed group) the
 * inequality lookups rebuild the key from the trie structure instead
 * -- same result, without the leaf-copy fast path.  Key length is
 * taken from the group's fixed key_len (or the leaf's trie depth for
 * variable-length keys), so no separate length offset is required.
 *
 * key_offset is interpreted modulo pointer arithmetic, so a key stored
 * before the node in the embedding struct (a "negative" offset) is
 * supported, exactly as for cds_ft_speculative_lookup_key.
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for a NULL @attr.
 */
enum cds_ft_status cds_ft_group_attr_set_speculative_key_offset(
		struct cds_ft_group_attr *attr,
		size_t key_offset);

/*
 * cds_ft_group_attr_set_key_len_offset - Declare where the caller's leaf stores
 *   its key length, generalizing the ordered cell list to variable-length keys.
 *
 * @offset: byte offset from the (struct cds_ft_node *) stored in the trie to a
 *   size_t in the caller's leaf holding that entry's key length.
 *
 * Optional, and only meaningful together with
 * cds_ft_group_attr_set_ordered_list on a VARIABLE-length-key group: it
 * lets cds_ft_next / cds_ft_prev obtain the result key length directly from
 * the leaf.  Unused for fixed-length groups (the length is the group's
 * fixed key_len).  The caller must store the length as a size_t at @offset
 * before insertion and not change it while the node is in the trie.
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for a NULL @attr.
 */
enum cds_ft_status cds_ft_group_attr_set_key_len_offset(
		struct cds_ft_group_attr *attr,
		size_t offset);

/*
 * cds_ft_group_attr_set_ordered_list - Enable (@ordered_list true) or disable
 *   (@ordered_list false) the library-owned ordered sibling list.  Enabled is
 *   the DEFAULT in a library built with FEATURE_FT_ORD_CELL (the default build).
 *
 * When ENABLED, the library maintains the distinct keys in key order to
 * accelerate cds_ft_next / cds_ft_prev and cds_ft_for_each*_batched.  This is
 * entirely library-internal: struct cds_ft_node is unchanged and the
 * application declares no offset and stores no ordering fields.  Forming the
 * iteration result key does require a key source, though: either
 * cds_ft_group_attr_set_speculative_key_offset (plus
 * cds_ft_group_attr_set_key_len_offset for a variable-length-key group), or an
 * identity-mapped group (where the key is recovered from the trie structure).
 * No application-leaf order storage is ever needed.
 *
 * When DISABLED (for a write-heavy group that never iterates in key order),
 * the group uses less memory per key and mutates faster, giving up ordered
 * iteration (cds_ft_next / cds_ft_prev / cds_ft_for_each*) and the
 * node-pointer key/step helpers.
 *
 * On by default, so passing true is normally redundant (it re-affirms the
 * default).  Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for a NULL @attr.
 */
enum cds_ft_status cds_ft_group_attr_set_ordered_list(
		struct cds_ft_group_attr *attr, bool ordered_list);

/*
 * cds_ft_group_attr_set_rank_stats - Enable (@rank_stats true) or disable
 *   (@rank_stats false) maintenance of the per-node order-statistics key
 *   counts.  DISABLED is the default.
 *
 * The order-statistics counts (a per-node subtree total of distinct keys) back
 * the rank / select / count queries:
 *   - cds_ft_count_keys / cds_ft_count_keys_prefix
 *   - cds_ft_lookup_nth / cds_ft_lookup_nth_last
 *   - cds_ft_iter_skip_forward / cds_ft_iter_skip_reverse
 *
 * When ENABLED, every mutation propagates the count along the path to the root
 * so the queries above read a maintained aggregate: cds_ft_count_keys is O(1),
 * cds_ft_count_keys_prefix is O(prefix_len), and the select / skip queries are
 * O(depth).  This costs a root-ward count propagation on every insert / remove.
 *
 * Because every count-changing mutation updates the single root count, no two
 * such writers are ever disjoint, so order statistics imply a COARSE writer
 * strategy: a group that enables rank stats is silently coerced to
 * CDS_FT_WRITER_LOCK_COARSE from ANY other strategy (both the lock-free
 * CDS_FT_WRITER_LOCK_FINE parallelizes
 * disjoint writers, of which there are none here, and both would run the
 * root-ward count walk without the FT-wide-lock exclusion it requires).
 *
 * When DISABLED (the default), the library maintains no count and those queries
 * stay correct but fall back to enumeration / iteration: cds_ft_count_keys and
 * cds_ft_count_keys_prefix enumerate the (sub)tree in O(size); cds_ft_lookup_nth
 * / _last and cds_ft_iter_skip_forward / _reverse iterate n steps via
 * cds_ft_next / cds_ft_prev.  (With the ordered list off as well, each
 * iteration step is itself an O(depth) structural traversal, so select / skip
 * become O(n.depth) -- the two flags interact for that cost class.)
 *
 * Off by default, so passing false is normally redundant.  Returns
 * CDS_FT_STATUS_OK on success, CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for a NULL
 * @attr.
 */
enum cds_ft_status cds_ft_group_attr_set_rank_stats(
		struct cds_ft_group_attr *attr, bool rank_stats);

/*
 * cds_ft_group_attr_set_numa_policy - Select the trie group's NUMA
 *                                     placement policy for its internal
 *                                     allocator.
 * @attr: Fractal Trie group attributes.
 * @policy: One of enum cds_ft_numa_policy (DEFAULT, INTERLEAVE, LOCAL).
 *
 * Defaults to CDS_FT_NUMA_DEFAULT when the group attr is freshly
 * created: the library honors any explicit per-process NUMA policy but
 * interleaves its own arenas when none is set (see the enum doc for the
 * full rationale -- first-touch is a performance cliff for a shared,
 * random-access trie).  Applications can force a placement via INTERLEAVE
 * or LOCAL.
 *
 * Selection guide:
 *   - DEFAULT:    honor an explicit process policy (MPOL_BIND / PREFERRED /
 *                 LOCAL set via numactl / set_mempolicy), upgrade an
 *                 explicit MPOL_INTERLEAVE to 2 MiB granularity, and
 *                 interleave when no process policy is set.  The good
 *                 default for most callers; never overrides a stated
 *                 process intent.
 *   - INTERLEAVE: 2 MiB-granular round-robin across allowed NUMA nodes.
 *                 Best for multi-reader workloads with readers distributed
 *                 across NUMA nodes.  Single-threaded performance is within
 *                 noise of LOCAL on modern x86 (the L2/L3 prefetcher works
 *                 fine over 2 MiB chunks of contiguous physical memory).
 *                 Enables transparent hugepage collapse by keeping each
 *                 2 MiB chunk on a single node.
 *   - LOCAL:      explicit first-touch (skip mbind).  Forces first-touch
 *                 even when no process policy is set, whereas DEFAULT
 *                 interleaves in that case.  Use when you specifically
 *                 want the trie on the faulting thread's node
 *                 (single-threaded / single-node access).
 *
 * The CDS_FT_NUMA_INTERLEAVE=0 environment variable, if set, forces
 * the library to skip mbind regardless of the group's policy --
 * useful for debugging without recompiling.  Otherwise the group's
 * policy takes effect.
 *
 * No effect on caller-provided external node storage
 * (cds_ft_external_arena_create has no group context; it applies
 * CDS_FT_NUMA_DEFAULT -- honors an explicit process policy, interleaves
 * otherwise).
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @policy value.
 */
enum cds_ft_status cds_ft_group_attr_set_numa_policy(
		struct cds_ft_group_attr *attr,
		enum cds_ft_numa_policy policy);

/*
 * cds_ft_group_attr_set_optimize - Select the group's page-size policy.
 *
 * Sets the page-size policy for the group's internal + compressed node
 * arenas (see enum cds_ft_optimize); the default is
 * CDS_FT_OPTIMIZE_THROUGHPUT.  Returns CDS_FT_STATUS_OK, or
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @opt.
 */
enum cds_ft_status cds_ft_group_attr_set_optimize(
		struct cds_ft_group_attr *attr,
		enum cds_ft_optimize opt);

/*
 * The group's structural-writer concurrency strategy (MW lock-escalation
 * model, doc/design/mw-writer-lock-escalation-model.md).  Readers are wait-free
 * under every strategy; this selects only how concurrent structural WRITERS
 * coordinate.
 *
 * CDS_FT_WRITER_LOCK_COARSE: writers serialize under one FT-wide writer lock per
 *   trie (classic RCU single-writer).  This is the single-writer opt-in.
 * CDS_FT_WRITER_LOCK_FINE (the DEFAULT): writers coordinate through
 *   fine-grained per-node lock-sets, so writers on disjoint subtrees proceed in
 *   parallel and only structural collisions serialize -- no FT-wide writer
 *   mutex.  Note this is the mode under which a CROSS-TRIE graft / graft_swap /
 *   merge_at requires an EXCLUSIVE source (see those functions): being the
 *   default, that requirement applies unless a group opts out to COARSE.  A trie that
 *   also maintains order statistics is coerced to CDS_FT_WRITER_LOCK_COARSE
 *   (every count-changing writer updates the shared root, so there are no
 *   disjoint writers for fine locking to parallelize).
 */
enum cds_ft_writer_strategy {
	CDS_FT_WRITER_LOCK_COARSE = 1,
	CDS_FT_WRITER_LOCK_FINE = 2,
};

/*
 * cds_ft_group_attr_set_writer_strategy - Select the group's structural-writer
 *   concurrency strategy (enum cds_ft_writer_strategy); the default is
 *   CDS_FT_WRITER_LOCK_FINE.  Returns CDS_FT_STATUS_OK, or
 *   CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @strategy.
 *
 * Any strategy combined with order statistics
 * (cds_ft_group_attr_set_rank_stats) is coerced to CDS_FT_WRITER_LOCK_COARSE:
 * rank stats serialize every count-changing writer on the root count, so
 * fine-grained locking buys no parallelism there, and it would run the count
 * walk without the FT-wide-lock exclusion the walk requires.  (The optimistic
 * engine this sentence also weighed no longer exists.)
 */
enum cds_ft_status cds_ft_group_attr_set_writer_strategy(
		struct cds_ft_group_attr *attr,
		enum cds_ft_writer_strategy strategy);

/*
 * Granularity of the per-node lock-sets a CDS_FT_WRITER_LOCK_FINE trie takes
 * (doc/design/ft-dlm-lock-coarseness.md).  A writer normally locks each node it
 * mutates; spacing the locks lets it instead lock an ANCESTOR at a designated
 * key-byte depth, so lock-set members that share one such level collapse onto
 * one lock word.  Readers are unaffected under every setting; this trades
 * writer parallelism against the cost and width of an acquire.
 *
 * Ignored by CDS_FT_WRITER_LOCK_COARSE, which derives no lock-set at all.
 *
 * CDS_FT_LOCK_SPACING_PER_NODE (the DEFAULT): a writer locks exactly the nodes
 *   it mutates -- maximum writer parallelism, widest acquire.
 * CDS_FT_LOCK_SPACING_EXPONENTIAL: lock levels at key-byte depths 0, 1, 2, 4,
 *   8, ... -- dense near the root, where one lock covers a subtree that may be
 *   half the trie, and sparse deeper, where a subtree is small enough that a
 *   lock spanning many levels excludes little.
 * CDS_FT_LOCK_SPACING_ROOT_ONLY: the root is the only lock level, so every
 *   writer serializes on it.  This is the granularity axis meeting
 *   CDS_FT_WRITER_LOCK_COARSE from the other side: one lock per trie.
 */
enum cds_ft_lock_spacing {
	CDS_FT_LOCK_SPACING_PER_NODE = 1,
	CDS_FT_LOCK_SPACING_EXPONENTIAL = 2,
	CDS_FT_LOCK_SPACING_ROOT_ONLY = 3,
};

/*
 * cds_ft_group_attr_set_lock_spacing - Select the granularity of the group's
 *   per-node lock-sets (enum cds_ft_lock_spacing); the default is
 *   CDS_FT_LOCK_SPACING_PER_NODE.  Returns CDS_FT_STATUS_OK, or
 *   CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @spacing.
 *
 * Meaningful only under CDS_FT_WRITER_LOCK_FINE.  The best setting is
 * workload-shaped -- it depends on trie depth, key distribution and how
 * disjoint the writers are -- so it is a knob rather than a fixed schedule.
 *
 * Anchoring is ALL-OR-NOTHING: two ops mutating one node must acquire the SAME
 * word, so a spacing coarser than per-node is correct only once every acquire
 * site maps its members through the anchor.  Until then
 * CDS_FT_LOCK_SPACING_EXPONENTIAL and CDS_FT_LOCK_SPACING_ROOT_ONLY are
 * REFUSED with CDS_FT_STATUS_INVALID_ARGUMENT_ERROR, rather than offered as a
 * setting that silently excludes nothing.
 */
enum cds_ft_status cds_ft_group_attr_set_lock_spacing(
		struct cds_ft_group_attr *attr,
		enum cds_ft_lock_spacing spacing);

/*
 * cds_ft_attr_create - Create a per-instance Fractal Trie attribute
 *                      structure.
 * @result: Attribute output. Set to the newly created attribute
 *          structure on success, or NULL on error.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result);

/*
 * cds_ft_attr_destroy - Destroy a per-instance Fractal Trie attribute
 *                       structure.
 * @attr: Fractal Trie attributes.
 */
void cds_ft_attr_destroy(struct cds_ft_attr *attr);

/*
 * cds_ft_attr_set_exclusive - Set the exclusive access discipline
 *                             attribute.
 * @attr: Fractal Trie attributes.
 * @exclusive: true if the trie will be accessed under exclusive
 *             discipline (single-threaded or mutex-protected, no
 *             concurrent RCU readers); false if concurrent RCU
 *             readers are permitted.
 *
 * Default: false (concurrent RCU readers permitted).
 *
 * Returns CDS_FT_STATUS_OK on success.
 */
enum cds_ft_status cds_ft_attr_set_exclusive(struct cds_ft_attr *attr,
		bool exclusive);

/*
 * cds_ft_attr_set_speculative_keys - Per-trie opt-out of speculative leaf keys.
 * @attr: Fractal Trie attributes.
 * @enabled: true (default) -- this trie uses the group's speculative result-key
 *           capture (cds_ft_group_attr_set_speculative_key_offset), reading each
 *           matched leaf's stored key field on lookup.  false -- this trie
 *           behaves as if the group had NO speculative key offset: every lookup
 *           reconstructs the result key from the trie structure (the EAGER path)
 *           and NEVER reads the stored leaf key.
 *
 * Has no effect on a group that was not configured with a speculative key
 * offset (such a group is always EAGER).
 *
 * Use false for a trie whose leaves' stored keys do not match their positions:
 *   - a STAGING source built for a non-root graft, whose leaves are stamped
 *     with their FUTURE destination key (dst prefix + suffix) rather than their
 *     current staging position;
 *   - the DESTINATION of a graft_swap or a re-keying merge_at that you do not
 *     speculative-look-up;
 *   - (the result of a non-root cds_ft_detach is created this way automatically,
 *     since the detach strips the key prefix).
 *
 * The library only ever READS the speculative key field; it cannot rewrite it
 * across a re-keying move (it owns neither the field's layout nor its capacity),
 * so maintaining "stored key == position" across such moves is the caller's
 * responsibility.  This attribute is how a trie within a speculative group opts
 * out of that responsibility by foregoing the speculative read.
 *
 * Default: true (inherit the group).  Returns CDS_FT_STATUS_OK, or
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @attr is NULL.
 */
enum cds_ft_status cds_ft_attr_set_speculative_keys(struct cds_ft_attr *attr,
		bool enabled);

/*
 * IN-TRIE REKEY COHERENCE is automatic; there is no attribute for it.
 *
 * A rekey (cds_ft_rekey_graft / cds_ft_rekey_merge) moves a live subtree from one
 * key prefix to another WITHIN one trie, so a reader whose traversal races the
 * move could otherwise be torn onto a position the key no longer occupies.  Every
 * trie that can host such a move -- any trie whose group does not use speculative
 * stored keys, since the library cannot rewrite an application-stored key when a
 * leaf's key changes -- verifies its reads against a concurrent move.
 *
 * The cost is confined to the move itself.  A mover first publishes a per-trie
 * "a move is in flight" mode and waits one grace period, so that every reader has
 * observed the mode before any structure changes; readers check that mode once per
 * traversal.  With no move in flight, that check is a single load of a word nobody
 * writes and the read runs exactly as it always did.  While a move IS in flight,
 * readers on this trie verify each traversal and repeat it if the move restructured
 * their path -- so reads on that trie are lock-free rather than wait-free for the
 * duration of the move, and unaffected otherwise.
 *
 * A burst of concurrent moves pays about ONE grace period between them, not one
 * each.  A rekey therefore BLOCKS and must not be called from an RCU read-side
 * critical section (see the rekey functions' own documentation).
 */

/*
 * cds_ft_make_exclusive - Transition a Fractal Trie to exclusive
 *                         access discipline.
 * @ft: The Fractal Trie.
 *
 * Blocks until in-flight RCU readers have drained (one grace period),
 * then marks the trie as exclusive.  Subsequent graft / graft_swap
 * operations with this trie as source then need no grace-period drain
 * of their own.
 *
 * The caller asserts that no new RCU readers will enter the trie
 * after this call returns (e.g. single-threaded access, or
 * mutex-protected access without RCU readers) until a matching
 * cds_ft_make_concurrent() call, if any.
 *
 * No-op if the trie is already exclusive.
 */
void cds_ft_make_exclusive(struct cds_ft *ft);

/*
 * cds_ft_make_concurrent - Transition a Fractal Trie to concurrent
 *                          access discipline.
 * @ft: The Fractal Trie.
 *
 * Marks the trie as permitting concurrent RCU readers.  Cheap:
 * does not block.  Typically called before publishing the trie
 * handle to reader threads.
 *
 * After this call, graft / graft_swap operations with this trie as
 * source must drain readers (one grace period) before moving the
 * content elsewhere -- unlike an exclusive trie, which skips that
 * drain.
 */
void cds_ft_make_concurrent(struct cds_ft *ft);

/*
 * cds_ft_is_exclusive - Query the access discipline of a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns true if the trie is currently in exclusive discipline,
 * false if concurrent RCU readers are permitted.
 */
bool cds_ft_is_exclusive(const struct cds_ft *ft);

/*
 * cds_ft_excl_validate_enabled - Query whether the optional
 *                                access-discipline validator is
 *                                compiled into the library.
 *
 * Returns true if the library was built with -DFEATURE_FT_EXCL_VALIDATE,
 * in which case writer/writer conflicts (any mode) and writer/reader
 * conflicts (exclusive mode) are detected at the public API boundary
 * and abort() the process with a violation report.
 *
 * Returns false if the validator is compiled out (the default); the
 * validator helpers are then no-ops.
 *
 * Primarily intended for tests: a negative test that deliberately
 * violates the contract can query this accessor and SKIP itself when
 * the validator is absent.
 */
bool cds_ft_excl_validate_enabled(void);

/*
 * cds_ft_merge_enabled - Query whether the merge subsystem is compiled in.
 *
 * The merge family (cds_ft_merge, cds_ft_merge_at and the graft paths
 * that reuse them) can be compiled out with -DNO_FEATURE_FT_MERGE,
 * which drops ~20 KiB of .text for a deployment that never merges;
 * cds_ft_merge then returns CDS_FT_STATUS_NOT_SUPPORTED.
 *
 * Returns false in such a build, true otherwise.
 *
 * Primarily intended for tests, for the same reason
 * cds_ft_excl_validate_enabled exists: a test that exercises merge
 * directly can query this and SKIP itself rather than fail, so the
 * merge-less configuration can be tested at all instead of being
 * build-only.
 */
bool cds_ft_merge_enabled(void);

/*
 * cds_ft_verify_at_mutation_enabled - Query whether the optional
 *                                     verify-at-mutation feature is
 *                                     compiled into the library.
 *
 * Returns true if the library was built with
 * -DFEATURE_FT_VERIFY_AT_MUTATION, in which case the writer
 * scope-exit hook samples cds_ft_verify at a per-trie tunable period
 * (see cds_ft_verify_at_mutation_period_set).  Returns false if the
 * feature is compiled out (the default), in which case the period
 * setter / getter return CDS_FT_STATUS_NOT_SUPPORTED.
 *
 * Use as a build-time gate from tests so that a negative test that
 * relies on the verify cadence can SKIP itself rather than silently
 * pass on a non-VAM build.
 */
bool cds_ft_verify_at_mutation_enabled(void);

/*
 * cds_ft_verify_at_mutation_period_set - Set the verify-at-mutation
 *                                        sampling period for @ft.
 *
 * When the library is built with -DFEATURE_FT_VERIFY_AT_MUTATION,
 * cds_ft_verify is run automatically once every @period mutations:
 *
 *   period == 0 : disable the verify walk on this trie;
 *   period == 1 : verify every mutation (the historical
 *                 -DFEATURE_FT_VERIFY_AT_MUTATION cadence; this is
 *                 the default at cds_ft_create() time);
 *   period >  1 : verify every @period mutations -- useful on large
 *                 tries where O(N) per mutation is impractical.
 *
 * Setting a new period resets the cadence, so the next verify lands
 * @period mutations from now.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED when the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION -- the call surfaces the mismatch
 * loudly so a test that relies on the verify cadence cannot
 * accidentally run with verify-at-mutation compiled out.  Use
 * cds_ft_verify_at_mutation_enabled() to gate the call.
 *
 * Write-side only (mutex-held); not safe to call concurrently with
 * writers on the same trie.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_set(struct cds_ft *ft,
		unsigned long period);

/*
 * cds_ft_verify_at_mutation_period_get - Read the verify-at-mutation
 *                                        sampling period for @ft
 *                                        into *@period.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED when the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION -- distinguishing build-disabled
 * from a runtime period == 0.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_get(
		const struct cds_ft *ft, unsigned long *period);

/*
 * Iterator management
 *
 * It is recommended that the user keeps a per-thread pool of
 * iterators in a TLS variable to minimize memory allocation.
 */

/*
 * cds_ft_iter_create - Allocate and initialize an iterator.
 * @ft: The Fractal Trie this iterator is bound to.
 * @result_iter: Iterator output. Set to the newly created iterator
 *               on success, or NULL on error.
 *
 * The iterator lifetime is bound to the Trie. The Trie must not be
 * destroyed while iterators to that trie exist.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter);

/*
 * cds_ft_iter_destroy - Free an iterator.
 * @iter: The iterator to free.
 */
void cds_ft_iter_destroy(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_set_cache_mode - Set the iterator's cache mode.
 * @iter: The iterator.
 * @mode: Cache mode to set.
 *
 * Switching from CDS_FT_ITER_CACHED to CDS_FT_ITER_UNCACHED
 * immediately clears any cached position (snapshotting a referenced
 * key first, as cds_ft_iter_bind_key() does). Switching from UNCACHED
 * to CACHED is always safe since there is no stale position.
 *
 * May be called at any point in the iterator's lifetime.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_set_cache_mode(struct cds_ft_iter *iter,
		enum cds_ft_iter_cache_mode mode);

/*
 * cds_ft_iter_get_cache_mode - Get the iterator's cache mode.
 * @iter: The iterator.
 *
 * Returns the current cache mode.
 */
enum cds_ft_iter_cache_mode cds_ft_iter_get_cache_mode(
		const struct cds_ft_iter *iter);

/*
 * cds_ft_iter_get_key - Retrieve the current key from an iterator.
 * @iter: The iterator.
 * @result_key: Output buffer for the key.
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the key written (output).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the buffer is too small.
 */
enum cds_ft_status cds_ft_iter_get_key(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_iter_get_prefix - Retrieve the current prefix from an iterator.
 * @iter: The iterator.
 * @result_key: Output buffer for the prefix.
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the prefix written (output).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the buffer is too small.
 */
enum cds_ft_status cds_ft_iter_get_prefix(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_iter_set_key - Set the key for the next lookup operation.
 * @iter: The iterator.
 * @key: The key to set.
 * @key_len: Key length in bytes.
 *
 * Setting a key always invalidates the iterator's cached position; the
 * next lookup re-descends from the root by the new key.  This is the
 * required mechanism to safely reuse an iterator after the RCU
 * read-side lock was dropped since the last operation.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len);

/*
 * cds_ft_iter_set_prefix_len - Set the prefix length for iteration.
 * @iter: The iterator.
 * @prefix_len: Length of the key prefix for scoped traversal.
 *              Must be <= the current key length (set the key first).
 *              Set to 0 to iterate over the entire trie.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @prefix_len
 * exceeds the current key length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len);

/*
 * cds_ft_iter_reset - Reset an iterator to its initial state.
 * @iter: The iterator.
 *
 * Clears the key, prefix, cached position, status, and result node.
 * The iterator remains bound to its Fractal Trie.
 */
void cds_ft_iter_reset(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_bind_key - Snapshot the current key into the iterator and drop
 *                        the cached position (for cross-critical-section resume).
 * @iter: The iterator.
 *
 * Drops the RCU-protected position cached in the iterator and snapshots its
 * current key into the iterator's own storage: the next operation is forced to
 * perform a fresh top-down traversal from that key.  The key and prefix length
 * are preserved.
 *
 * For CDS_FT_ITER_CACHED iterators this must be called if the RCU read-side
 * lock is dropped between operations on the same iterator, both to prevent
 * use-after-free of the cached node AND -- in a speculative skip-compressed
 * group configured with a leaf-key offset
 * (cds_ft_group_attr_set_speculative_key_offset()) or the library-owned
 * ordered list (cds_ft_group_attr_set_ordered_list()) -- because the current
 * key after an ordered lookup (cds_ft_lookup_first / cds_ft_next / the
 * relational lookups) is held as a LIVE REFERENCE into the matched leaf, valid
 * only while that lock is held.  Bind copies it out WHILE STILL HOLDING the
 * lock, so the iterator no longer depends on the soon-to-be-reclaimable leaf.
 *
 * CDS_FT_ITER_UNCACHED iterators do not require this call -- the cached
 * position is discarded (and any referenced key snapshotted) automatically
 * after each operation.
 *
 * After bind the iterator resumes like an uncached one: cds_ft_iter_get_key()
 * still returns the bound key, and the next cds_ft_next() in a fresh critical
 * section re-descends from the root and yields the bound key's successor
 * (robust even if the bound key was removed in the meantime).  Note that bind
 * also drops the current result -- cds_ft_iter_node() returns NULL until that
 * re-descent -- whereas a plain uncached iterator keeps the last operation's
 * result node readable until its next operation.
 *
 * Piecewise iteration:
 *
 *   bool resuming = false;
 *   for (;;) {
 *           rcu_read_lock();
 *           if (resuming)
 *                   cds_ft_next(ft, iter);          // successor of the bound key
 *           else
 *                   cds_ft_lookup_first(ft, iter);
 *           while (cds_ft_iter_node(iter)) {
 *                   cds_ft_iter_get_key(iter, ...); // process this key
 *                   if (batch_full)
 *                           break;
 *                   cds_ft_next(ft, iter);
 *           }
 *           if (!cds_ft_iter_node(iter)) {          // reached the end
 *                   rcu_read_unlock();
 *                   break;
 *           }
 *           cds_ft_iter_bind_key(iter);             // snapshot before unlocking
 *           rcu_read_unlock();
 *           resuming = true;
 *           // ... work outside the read-side critical section ...
 *   }
 */
void cds_ft_iter_bind_key(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_copy - Copy an iterator's state.
 * @dst: Destination iterator.
 * @src: Source iterator.
 *
 * Both iterators must be bound to the same Fractal Trie.
 *
 * The copy carries @src's cached position (and, when @src's result key
 * is held as a live reference into a leaf, that dependency too), so @dst
 * is valid only while the RCU read-side lock that produced @src's
 * position is held continuously; reusing @dst's position then carries
 * the same stale-position hazard as @src's.  To carry @dst across a
 * critical section, cds_ft_iter_bind_key() it (or @src before copying)
 * while the lock is held.
 */
void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src);

/*
 * cds_ft_iter_status - Return the status of the last operation on an iterator.
 * @iter: The iterator.
 *
 * Returns the cds_ft_status set by the last lookup or iteration
 * step performed on this iterator. Check (ret < 0) to detect errors.
 */
enum cds_ft_status cds_ft_iter_status(const struct cds_ft_iter *iter);

/*
 * cds_ft_iter_node - Return the current node from an iterator.
 * @iter: The iterator.
 *
 * Returns the node found by the last lookup, or NULL if the last
 * lookup found nothing.
 */
struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter);

/*
 * Cell batch iteration
 */

/*
 * cds_ft_node_get_key - Materialize a key from a bare external node pointer.
 * @ft: The Fractal Trie @node belongs to.
 * @node: An external head node (e.g. from a point lookup, or cds_ft_cell_node()).
 * @result_key: Output buffer for the key.
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the key written (output).
 *
 * Reconstructs @node's key without an iterator object -- the same key
 * cds_ft_iter_get_key() would return.  The key comes from an in-leaf key when
 * the group declares a speculative key offset, otherwise from the trie
 * structure (ordered-list groups only).
 *
 * RCU CONTRACT: @node, and the trie structure its key is read from, are valid
 * only while the RCU read-side lock that produced @node is held CONTINUOUSLY.
 * Unlike an iterator -- which an UNCACHED caller can carry across a critical
 * section because it materializes the key into its own storage -- a bare node
 * pointer must NOT outlive its read-side critical section.  This is the
 * within-CS companion to the iterator, for a node from a point lookup; for a
 * batched ordered scan use the cell batch (cds_ft_cell_get_key()) instead.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @node is NULL.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the buffer is too small.
 * Returns CDS_FT_STATUS_NOT_FOUND if the group has neither an in-leaf key nor
 * an ordered list, so a key cannot be materialized from a node alone.
 */
enum cds_ft_status cds_ft_node_get_key(const struct cds_ft *ft,
		const struct cds_ft_node *node, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_cell_next_batch - Iterator-free ordered batched walk, cell handles.
 * @ft: The Fractal Trie.
 * @cursor: Cell to start AT (inclusive); NULL starts at the list minimum.
 * @buf: Output array of up to @cap opaque cell handles, in ascending key order.
 * @cap: Capacity of @buf.
 * @count: Output -- number of cells written to @buf (0 at the end of the walk).
 * @next_cursor: Output -- the cell to pass as @cursor next, or NULL at the end.
 *
 * Walks @ft's ordered cell list without an iterator object, emitting opaque CELL
 * handles a batch at a time so the per-call boundary is paid once per @cap cells.
 * From each handle, recover the node with cds_ft_cell_node() (off the
 * cached cds_ft_cell_node_offset()) and the key with
 * cds_ft_cell_get_key().  Stop when @next_cursor comes back
 * NULL (NOT when *@count is 0): a NULL cursor is BOTH the start sentinel and the
 * end signal, so terminate on the returned cursor, not the count:
 *
 *   const struct cds_ft_cell *cur = NULL;
 *   size_t off = cds_ft_cell_node_offset(), n, i;
 *   do {
 *           if (cds_ft_cell_next_batch(ft, cur, buf, N, &n, &cur) != CDS_FT_STATUS_OK)
 *                   break;       // list off: use cds_ft_for_each_rcu() instead
 *           for (i = 0; i < n; i++) {
 *                   struct cds_ft_node *node = cds_ft_cell_node(buf[i], off);
 *                   cds_ft_cell_get_key(ft, buf[i], k, sizeof k, &kl);
 *           }
 *   } while (cur);
 *
 * ORDERED-LIST ONLY: a list-off trie (cds_ft_group_attr_set_ordered_list(attr,
 * false)) has no cell list, so it returns CDS_FT_STATUS_NOT_SUPPORTED (*@count =
 * 0, *@next_cursor = NULL); use cds_ft_next()/cds_ft_for_each_rcu() (a stateful
 * iterator) there.  Query the mode up front with cds_ft_group_ordered_list().
 *
 * RCU CONTRACT: @cursor and every returned cell are valid only while the RCU
 * read-side lock that produced @cursor is held CONTINUOUSLY.  A cell handle (and
 * any node/key derived from it) must not outlive its critical section; for a
 * resumable scan that spans grace periods, use the iterator.
 *
 * Returns CDS_FT_STATUS_OK (batch in @buf/@count, possibly 0 at the end), or
 * CDS_FT_STATUS_NOT_SUPPORTED on a list-off trie.
 */
enum cds_ft_status cds_ft_cell_next_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor);

/*
 * cds_ft_cell_prev_batch - Reverse (descending-key-order) cds_ft_cell_next_batch.
 * @cursor NULL starts at the list maximum; otherwise identical, stepping down.
 */
enum cds_ft_status cds_ft_cell_prev_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor);

/*
 * cds_ft_cell_node_offset - Byte offset of the head-node pointer in a cell.
 *
 * Invariant for the process: fetch it once, cache it, and pass it to
 * cds_ft_cell_node() per element with no further library call.  It is a
 * runtime getter rather than a header constant so the cell stays opaque and
 * the offset stays ABI-stable: a caller that fetches it at runtime keeps
 * working if the layout ever changes.
 */
size_t cds_ft_cell_node_offset(void);

/*
 * cds_ft_cell_node - Recover the head node from an opaque cell handle, given a
 * cached @node_offset from cds_ft_cell_node_offset().  A MACRO (not an inline) so
 * rcu_dereference resolves in the CALLER's translation unit, where the RCU
 * flavor is included.  The cell's node pointer can change under concurrent
 * mutation, so this is an rcu_dereference snapshot -- valid old-or-new under a
 * continuously held read lock, like the iterator's node read.  @cell and
 * @node_offset are each evaluated once.
 */
#define cds_ft_cell_node(cell, node_offset)				\
	((struct cds_ft_node *) rcu_dereference(			\
		*(struct cds_ft_node *const *)				\
			((const char *) (cell) + (node_offset))))

/*
 * cds_ft_cell_get_key - Materialize a key from an opaque cell handle (the lazy
 * companion to the cell batch).  Same key sources as cds_ft_node_get_key.
 * Same RCU contract as cds_ft_cell_next_batch().
 *
 * Returns CDS_FT_STATUS_OK, CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @cell is
 * NULL, or CDS_FT_STATUS_OVERFLOW_ERROR if @result_key is too small.
 */
enum cds_ft_status cds_ft_cell_get_key(const struct cds_ft *ft,
		const struct cds_ft_cell *cell, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len);

/*
 * Key conversion helpers
 */

/*
 * cds_ft_key_to_u64 - Convert a Fractal Trie key to an unsigned 64-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..8]: fixed-length
 * tries with a configured length <= 8 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  Behaves as a big-endian conversion over that
 * many bytes.
 *
 * Returns 0 if the resolved @key_len exceeds 8 or is invalid for
 * this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u64_to_key - Convert an unsigned 64-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..8]: fixed-length
 * tries with a configured length <= 8 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  Truncates the most significant bits beyond the
 * key range.
 *
 * No-op if the resolved @key_len exceeds 8 or is invalid for this
 * trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a variable-length
 * trie).
 */
void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key, size_t key_len);

/*
 * cds_ft_key_to_u32 - Convert a Fractal Trie key to an unsigned 32-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..4]: fixed-length
 * tries with a configured length <= 4 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).
 *
 * Returns 0 if the resolved @key_len exceeds 4 or is invalid for
 * this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u32_to_key - Convert an unsigned 32-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..4]: fixed-length
 * tries with a configured length <= 4 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  Truncates the most significant bits beyond the
 * key range.
 *
 * No-op if the resolved @key_len exceeds 4 or is invalid for this
 * trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a variable-length
 * trie).
 */
void cds_ft_u32_to_key(const struct cds_ft *ft, uint32_t v, uint8_t *key, size_t key_len);

/*
 * Signed integer key helpers.
 *
 * These helpers convert between signed integers and Fractal Trie keys.
 * The sign bit is flipped so that the big-endian byte ordering used by
 * the trie preserves the natural signed integer ordering:
 *
 *   INT64_MIN  -> 0x0000000000000000   (sorts first)
 *   -1         -> 0x7FFFFFFFFFFFFFFF
 *    0         -> 0x8000000000000000
 *   INT64_MAX  -> 0xFFFFFFFFFFFFFFFF   (sorts last)
 *
 * The same principle applies to 32-bit signed integers.
 *
 * When the key is narrower than the integer type (e.g. a 2-byte key
 * representing a signed 16-bit range within a 64-bit integer), the
 * sign bit is at position (key_len * 8 - 1), not at the MSB of the
 * full integer type. The key-to-integer helpers sign-extend from the
 * key's MSB to fill the integer. The integer-to-key helpers flip the
 * sign bit at the key's MSB and truncate to the key width.
 */

/*
 * cds_ft_key_to_s64 - Convert a Fractal Trie key to a signed 64-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..8]: fixed-length
 * tries with a configured length <= 8 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  When the key is narrower than 8 bytes the
 * result is sign-extended to 64 bits.
 *
 * Returns 0 if the resolved @key_len is 0, exceeds 8, or is invalid
 * for this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
int64_t cds_ft_key_to_s64(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_s64_to_key - Convert a signed 64-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..8]: fixed-length
 * tries with a configured length <= 8 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  Truncates the most significant bits beyond the
 * key range.
 *
 * No-op if the resolved @key_len is 0, exceeds 8, or is invalid for
 * this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
void cds_ft_s64_to_key(const struct cds_ft *ft, int64_t v, uint8_t *key, size_t key_len);

/*
 * cds_ft_key_to_s32 - Convert a Fractal Trie key to a signed 32-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..4]: fixed-length
 * tries with a configured length <= 4 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).  When the key is narrower than 4 bytes the
 * result is sign-extended to 32 bits.
 *
 * Returns 0 if the resolved @key_len is 0, exceeds 4, or is invalid
 * for this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
int32_t cds_ft_key_to_s32(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_s32_to_key - Convert a signed 32-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Works whenever the resolved @key_len falls in [1..4]: fixed-length
 * tries with a configured length <= 4 (pass CDS_FT_LEN_DEFAULT or a
 * matching explicit length) and variable-length tries (pass an
 * explicit length).
 *
 * No-op if the resolved @key_len is 0, exceeds 4, or is invalid for
 * this trie's configuration (e.g. CDS_FT_LEN_DEFAULT on a
 * variable-length trie).
 */
void cds_ft_s32_to_key(const struct cds_ft *ft, int32_t v, uint8_t *key, size_t key_len);

/*
 * Diagnostics
 */

/*
 * cds_ft_verify - Verify integrity of the entire Fractal Trie.
 * @ft: The Fractal Trie.
 * @out: File stream for diagnostic output on failure (may be NULL
 *       to suppress output).
 *
 * Walks the whole trie checking its internal structural invariants
 * (child and key counts, parent back-pointers, compressed-node
 * invariants).
 *
 * Scoped to ONE trie: a node reachable from two tries of a group is
 * self-consistent in each and passes here.  Use cds_ft_verify_disjoint
 * to check a set of tries against each other.
 *
 * Must be called with mutual exclusion wrt updaters.
 *
 * Returns CDS_FT_STATUS_OK if the trie passes all checks, or
 * CDS_FT_STATUS_INTEGRITY_ERROR on integrity violation.
 */
enum cds_ft_status cds_ft_verify(const struct cds_ft *ft, FILE *out);

/*
 * cds_ft_verify_disjoint - Verify several tries, and that they share no node.
 * @fts: Array of @nr_fts tries; no NULL entries.
 * @nr_fts: Number of tries in @fts.
 * @out: File stream for diagnostic output on failure (may be NULL
 *       to suppress output).
 *
 * Runs the cds_ft_verify walk on each trie in turn, all against ONE
 * visited set, so a node reachable from more than one of them is
 * reported instead of passing as it does per-trie.
 *
 * The cross-trie operations (cds_ft_graft, cds_ft_graft_swap,
 * cds_ft_merge_at with src != dst) MOVE nodes between tries of a group;
 * none of them shares one.  An aliased node keeps every per-trie
 * invariant intact and only shows up later, as a mutation in one trie
 * corrupting another.
 *
 * Must be called with mutual exclusion wrt updaters of ALL @fts: an
 * in-flight cross-trie operation legitimately holds a node between two
 * tries.
 *
 * Returns CDS_FT_STATUS_OK if every trie passes and they are disjoint,
 * CDS_FT_STATUS_INTEGRITY_ERROR on integrity violation, or
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @fts has a NULL entry.
 */
enum cds_ft_status cds_ft_verify_disjoint(struct cds_ft *const *fts,
		size_t nr_fts, FILE *out);

/*
 * enum cds_ft_compact_status - Drive/result status for the compaction API.
 *
 * Returned by cds_ft_compact_step (DONE / MORE / OOM / BUSY) and
 * cds_ft_compact (DONE / OOM / BUSY, never MORE).
 *
 * OOM and BUSY are BOTH "stopped early, resume from the interrupted key", and
 * they are distinct because the caller's remedy differs: OOM asks it to free
 * memory, BUSY only to try again -- a relocation lost a lock-set to a peer and
 * nothing is wrong with memory.  Reporting contention as OOM would send a
 * caller freeing memory it does not need to free.
 */
enum cds_ft_compact_status {
	CDS_FT_COMPACT_DONE	= 0,	/* fully compacted (terminal success) */
	CDS_FT_COMPACT_MORE	= 1,	/* more work remains; call cds_ft_compact_step again */
	CDS_FT_COMPACT_OOM	= 2,	/* stopped on memory pressure; free memory and resume */
	CDS_FT_COMPACT_BUSY	= 3,	/* stopped on writer contention; just resume */
};

/*
 * cds_ft_compact - Defragment a trie's internal-node arenas in place.
 *
 * Recovers the descent locality and resident memory that churn or
 * graft-based population fragmentation cost a trie over time.
 *
 * One-shot convenience wrapper around cds_ft_compact_begin/step/end: the
 * caller must exclude concurrent writers on @ft for the whole call (the same
 * mutual-exclusion contract as the other mutators). Concurrent RCU readers are
 * permitted throughout, and other tries sharing the group keep mutating -- the
 * group stays online.
 *
 * Returns CDS_FT_COMPACT_DONE once the trie is fully compacted, or
 * CDS_FT_COMPACT_OOM if it stopped early under memory pressure: an allocation
 * failed, so the one-shot leaves the affected node in place and STOPS rather
 * than walking the rest of the trie into doomed allocations. The trie stays
 * valid and correct, only partially compacted. A caller that wants to free
 * memory and continue from where it stopped should drive the resumable
 * cds_ft_compact_begin/step/end API instead (cds_ft_compact does not resume).
 *
 * Memory reclaim is deferred (RCU grace period): resident memory does not
 * drop synchronously when this returns, but once a grace period has
 * elapsed.  A caller that needs to observe the lower RSS (e.g. before
 * measuring) must wait for a grace period itself, for example rcu_barrier().
 *
 * To defragment a whole group, call this on each trie the group contains.
 */
enum cds_ft_compact_status cds_ft_compact(struct cds_ft *ft);

/*
 * Resumable compaction (cds_ft_compact_begin / _step / _end).
 *
 * Lets a long compaction be interleaved with concurrent mutations of the
 * SAME trie: the caller drives it, holding its writer mutex only around each
 * cds_ft_compact_step and releasing it between steps so other writers (and
 * grace periods that drain reclaimed ranges) get a window. Each step
 * tolerates the structure changing between steps (a writer may mutate
 * the trie while the lock is dropped).
 *
 *   struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);
 *   enum cds_ft_compact_status s;
 *   do {
 *           writer_lock();
 *           s = cds_ft_compact_step(st, batch);
 *           writer_unlock();
 *           if (s == CDS_FT_COMPACT_OOM) {
 *                   ... free memory, then loop to resume from the same key ...
 *           }
 *   } while (s != CDS_FT_COMPACT_DONE);
 *   cds_ft_compact_end(st);
 */
struct cds_ft_compact_state;

/*
 * cds_ft_compact_begin - Start a resumable compaction of @ft.
 *
 * Returns an opaque state to drive with cds_ft_compact_step, or NULL on
 * allocation failure or if a compaction is already in progress on @ft
 * (only one may be in flight per trie at a time; end the current one with
 * cds_ft_compact_end first). The returned state must be released with
 * cds_ft_compact_end.
 */
struct cds_ft_compact_state *cds_ft_compact_begin(struct cds_ft *ft);

/*
 * cds_ft_compact_step - Relocate up to @batch internal nodes.
 * @st: State from cds_ft_compact_begin.
 * @batch: Maximum nodes to relocate this step (0 selects a default).
 *
 * The caller must hold its writer exclusion for @ft across this call. Returns:
 *   CDS_FT_COMPACT_MORE - more work remains; call again.
 *   CDS_FT_COMPACT_DONE - the trie is fully compacted.
 *   CDS_FT_COMPACT_OOM  - stopped early: a relocation hit an allocation failure.
 * The trie is valid at every step boundary. On CDS_FT_COMPACT_OOM the bound
 * cursor is left at the interrupted key, so after freeing memory the caller may
 * call this again to RESUME -- it re-attempts that key inclusively and loses no
 * key (an un-relocated node of any key pins its whole old arena range against
 * reclaim).
 */
enum cds_ft_compact_status cds_ft_compact_step(struct cds_ft_compact_state *st,
		size_t batch);

/*
 * cds_ft_compact_end - Finish a compaction and release its state.
 * @st: State from cds_ft_compact_begin.
 *
 * Must be called exactly once after cds_ft_compact_begin, whether the
 * compaction ran to completion or the caller stopped early. Stopping early is
 * fine: it merges the work done so far and leaves a valid, partially-compacted
 * trie. Releases all resources held by @st.
 */
void cds_ft_compact_end(struct cds_ft_compact_state *st);

/*
 * cds_ft_show_format - Output format for cds_ft_show().
 *
 * CDS_FT_SHOW_PRETTY: human-readable indented text, suitable for
 *   debugging printouts.  Emits level headers, node kinds, and
 *   per-edge key-byte values.
 * CDS_FT_SHOW_JSON:   machine-readable JSON tree.  The root document
 *   is an object with "ft" (pointer) and "root" (node).  Each node
 *   carries "ptr", "kind", "level", optional "nr_child", and a
 *   "children" array whose entries are {"key_byte": N, "child": <node>}.
 *   Compressed nodes additionally carry "path_len" and "child".  External
 *   nodes carry just "ptr" and "kind".  Intended for programmatic
 *   consumption (visualizers, test assertions).
 */
enum cds_ft_show_format {
	CDS_FT_SHOW_PRETTY,
	CDS_FT_SHOW_JSON,
};

/*
 * cds_ft_show - Print content of the Fractal Trie.
 * @ft: The Fractal Trie.
 * @out: File stream output.
 * @fmt: Output format (see enum cds_ft_show_format).
 */
void cds_ft_show(const struct cds_ft *ft, FILE *out,
		enum cds_ft_show_format fmt);

/*
 * cds_ft_show_stats - Print Fractal Trie statistics.
 * @ft: The Fractal Trie.
 * @out: File stream output.
 */
void cds_ft_show_stats(const struct cds_ft *ft, FILE *out);

/*
 * cds_ft_status_to_string - Convert a status value to string.
 * @status: The status value.
 *
 * Return a pointer to a const string representing the status.
 */
const char *cds_ft_status_to_string(enum cds_ft_status status);

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FRACTAL_TRIE_H */
