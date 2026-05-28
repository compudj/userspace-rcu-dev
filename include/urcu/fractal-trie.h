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
 * A concurrent, RCU-protected ordered trie mapping variable or
 * fixed-length byte keys to user-defined nodes. Keys are opaque
 * byte sequences with no reserved or sentinel values; unlike
 * tries that rely on NUL or other terminal characters, any byte
 * value may appear at any position in a key. Lookups and
 * traversals are wait-free under the RCU read-side lock and can
 * proceed concurrently with mutations. The internal structure is
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
 * Internal node configurations:
 *
 * Internal nodes self-adapt to the key population using several
 * configurations (popcount-bitmap, pigeon), each with a different
 * indexing strategy suited to its child density.  The appropriate
 * configuration is chosen automatically based on the number of
 * children.  Node sizes are powers of 2 between 32 bytes and 2048
 * bytes on 64-bit architectures (1024 bytes on 32-bit).  Mutations
 * use in-place updates when possible to minimize node recompaction,
 * and hysteresis at size thresholds prevents repeated recompaction
 * when the child count oscillates near a boundary.
 *
 * The popcount-bitmap configurations (cascaded 2-level bitmaps for
 * small/medium fan-out, single 256-bit byte bitmap for large
 * fan-out) record which key bytes are populated in a fixed-position
 * bitmap and dispatch matches through a popcount-derived rank.
 * This provides a range of intermediate node sizes up to the full
 * 256-entry pigeon configuration, allowing memory-efficient
 * representation of medium-density populations without requiring
 * the full pigeon footprint.  Alternative approaches such as Judy use a population
 * bitmap with a dense child array, which requires recompacting the
 * array on every insertion or removal -- incompatible with wait-free
 * RCU lookups because a reader could observe a partially recompacted
 * array.  The popcount-bitmap approach avoids that by using fixed
 * bit positions derived from key bytes, allowing children to be
 * added or removed with single-pointer updates visible atomically
 * to concurrent readers.
 *
 * Node type and configuration are encoded in the low bits of
 * child pointers (tagged pointers), so determining a node's
 * layout during lookup requires no extra memory access.
 *
 * Pigeon and the largest popcount-bitmap node both carry a 32-byte
 * presence bitmap that locates populated children via bit scanning,
 * reducing the number of cache-line accesses needed for ordered
 * traversal.
 *
 * Prefix compression (path compaction):
 *
 * Chains of single-child internal nodes are automatically
 * replaced with compressed path nodes that store the key bytes
 * inline with a direct child pointer. This is similar to the
 * path compaction used in Patricia tries and adaptive radix
 * trees (ART). A compressed node replaces N single-child
 * internal nodes with a single allocation storing N key bytes,
 * reducing both memory usage and traversal time for tries with
 * long shared prefixes.
 *
 * Compressed nodes are created transparently during insert when
 * a chain of two or more single-child levels would otherwise be
 * built. All mutation operations (insert, remove, graft, detach)
 * handle compressed nodes without decompressing them: inserts
 * split compressed paths at the divergence or key-endpoint,
 * removes traverse through them, and grafts split or traverse
 * as needed. Read-side operations (lookup, inequality, iteration,
 * skip, count) traverse compressed paths in O(1) per compressed
 * byte with no extra memory accesses beyond the single compressed
 * node.
 *
 * Prefix compression is enabled by default and can be disabled
 * at compile time with -DNO_FEATURE_FT_COMPRESS for a simpler
 * trie with no path compaction. The pointer tag encoding uses
 * bit 1 for the compressed flag (0b10), orthogonal to the
 * internal flag in bit 0 (0b01), and the external tag (0b00).
 *
 * Memory layout:
 *
 * Per-node metadata is stored in a separate page via a strided
 * allocator and reached by pointer offset from the node address,
 * keeping metadata out of the node's cache lines. This avoids
 * padding overhead within nodes and ensures that lookups and
 * traversals only touch the data they need. Internal nodes are
 * reclaimed via RCU (call_rcu), so readers never access freed
 * memory even for the trie's own internal structure.
 *
 * This provides memory efficiency comparable to adaptive radix
 * tree schemes without requiring user tuning or configuration.
 *
 * Graft, graft-swap, and detach:
 *
 * The graft, graft-swap, and detach operations move entire
 * sub-tries between trie instances within the same group. The
 * write-side cost is more than a pointer store — descent to the
 * graft/detach point, key-count propagation up the destination's
 * ancestors, allocation of the result-trie wrapper for detach,
 * metadata fix-up, and (in detach with concurrent readers
 * possible) one synchronize_rcu before the dependent reclamation.
 * What concurrent readers see, however, is published by a single
 * atomic pointer update: a reader observes either the complete
 * sub-trie or nothing, never a partial state.
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
 * pattern populates a staging trie offline — with no RCU or
 * locking overhead — and then grafts it into the live trie in
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
 * *validates* the discipline above and aborts on violation — it
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
 * The `cds_ft_iter` object can operate in two path modes, selected via
 * cds_ft_iter_set_path_mode():
 *
 * CDS_FT_ITER_PATH_CACHED (default):
 *
 * The iterator caches the traversal path (RCU protected pointers) to
 * accelerate subsequent sequential operations like cds_ft_next(),
 * cds_ft_prev(), or cds_ft_remove().
 *
 * Because of this caching, the RCU read-side lock must be held
 * CONTINUOUSLY between the operation that populates the iterator
 * (e.g., cds_ft_lookup) and the operation that consumes it. If the
 * RCU read-side lock is dropped, the cached path may point to freed
 * memory.
 *
 * If you need to drop the RCU read-side lock between operations, you
 * MUST invalidate the iterator's cached path before reusing it. This
 * is done by calling:
 * - cds_ft_iter_invalidate_path()
 *
 * Calling this function clears the internal path state while keeping
 * your key intact. The next operation (e.g., cds_ft_remove) will
 * automatically fall back to a safe, fresh top-down traversal.
 *
 * Example A (Continuous Lock - Fast):
 * rcu_read_lock();
 * cds_ft_lookup(ft, iter);
 * cds_ft_remove(ft, iter, node); // Uses cached path O(1)
 * rcu_read_unlock();
 *
 * Example B (Dropped Lock - CDS_FT_ITER_PATH_CACHED only):
 * rcu_read_lock();
 * cds_ft_lookup(ft, iter);
 * rcu_read_unlock();
 * // ... time passes, lock is dropped ...
 *
 * lock(&writer_mutex);
 * cds_ft_iter_invalidate_path(iter); // Flush the stale RCU cache
 * cds_ft_remove(ft, iter, node);     // Safe: Fresh traversal protected by writer lock
 * unlock(&writer_mutex);
 *
 * Note: CDS_FT_ITER_PATH_UNCACHED iterators do not require
 * cds_ft_iter_invalidate_path() — the path is discarded
 * automatically after each operation.
 *
 * Debug validation (URCU_FRACTAL_TRIE_DEBUG_PATH):
 *
 * Building with URCU_FRACTAL_TRIE_DEBUG_PATH defined enables run-time
 * detection of stale cached paths.  Each path population records an RCU
 * grace-period snapshot (via the flavor's
 * update_start_poll_synchronize_rcu); each path consumption polls it
 * (via update_poll_state_synchronize_rcu).  If a full grace period has
 * elapsed since the path was populated, the cached pointers may
 * reference freed memory — the program aborts with a diagnostic.
 * Since struct cds_ft_iter is opaque, this option does not affect the
 * application ABI — only the library needs to be rebuilt.
 *
 * CDS_FT_ITER_PATH_UNCACHED:
 *
 * The iterator automatically discards the traversal path after each
 * operation returns. Every subsequent operation performs a fresh
 * top-down traversal from the current key. The RCU read-side lock
 * only needs to be held during each individual operation and while
 * accessing the returned node — it may be dropped between operations.
 *
 * This mode is suited for iteration patterns where the caller needs
 * to drop the RCU read-side lock between steps (e.g. to perform
 * blocking work or acquire other locks).
 *
 * Example C (Uncached mode, per-step locking with refcount):
 * cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);
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
	CDS_FT_STATUS_NOT_SUPPORTED		= -7	/* Feature not compiled in. */
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
	 *
	 *   Optimize for cds_ft_lookup_candidate_key and
	 *   cds_ft_speculative_lookup_key.  Compressed-node bytes are
	 *   skipped during descent; the candidate match is returned
	 *   without per-step verification.
	 *
	 *   On architectures where the skip-compressed pointer
	 *   encoding is available (64-bit with sufficient free high
	 *   pointer bits) the descent also bypasses the compressed
	 *   node's cache-line load entirely.  On other architectures
	 *   the compressed cache line is still loaded but the byte
	 *   comparison is skipped — speculative descent still wins.
	 *
	 *   cds_ft_eager_lookup_key works on speculative-tuned tries;
	 *   on skip-compressed-capable archs the precise descent pays
	 *   a small extra cost (unwrap the skip-encoded pointer before
	 *   fetching the compressed node), but correctness is
	 *   unchanged.
	 */
	CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE = 0,

	/*
	 * CDS_FT_LOOKUP_OPTIMIZE_EAGER:
	 *
	 *   Optimize for cds_ft_eager_lookup_key.  Precise descent
	 *   reads compressed nodes via direct pointers (no
	 *   skip-encoded unwrap).  cds_ft_lookup_candidate_key and
	 *   cds_ft_speculative_lookup_key still work but lose the
	 *   skip-compressed encoding's bypass of compressed cache
	 *   lines on supported archs.
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
	 * CDS_FT_NUMA_DEFAULT: defer to the process's libnuma / mempolicy
	 * configuration.  The library applies no mbind() of its own — the
	 * kernel honors whatever policy the application set (via
	 * set_mempolicy(), numa_set_*(), or a numactl wrapper at launch),
	 * falling back to first-touch placement if no process policy is
	 * set.
	 *
	 * This is the default value of a freshly-created group attr.
	 * Applications that want explicit library-side placement should
	 * set INTERLEAVE or LOCAL.
	 */
	CDS_FT_NUMA_DEFAULT = 0,

	/*
	 * CDS_FT_NUMA_INTERLEAVE: round-robin allocator superblocks
	 * across the calling thread's allowed NUMA nodes in 2 MiB
	 * chunks (mbind(MPOL_BIND) per chunk).  Best for workloads
	 * with concurrent readers distributed across NUMA nodes: every
	 * reader sees the same balanced cross-node access pattern, so
	 * the worst-case cross-NUMA cost is bounded by (NR_NODES - 1) /
	 * NR_NODES of accesses.  Enables transparent hugepage collapse
	 * by keeping each 2 MiB chunk on a single node.
	 */
	CDS_FT_NUMA_INTERLEAVE,

	/*
	 * CDS_FT_NUMA_LOCAL: explicit first-touch placement — pages
	 * land on whichever NUMA node first faults them (typically the
	 * writer thread).  Mechanically the library skips mbind(); the
	 * distinction from DEFAULT is intent: LOCAL declares "I want
	 * first-touch", DEFAULT declares "I defer to process policy".
	 *
	 * Best for single-threaded workloads and for tries accessed
	 * exclusively by threads on the writer's node.  Multi-node-
	 * distributed readers pay full cross-NUMA latency for every
	 * access since all pages live on one node.
	 */
	CDS_FT_NUMA_LOCAL,
};

/*
 * Iterator path mode.
 *
 * Controls whether the iterator retains or discards its internal
 * traversal path between operations.
 */
enum cds_ft_iter_path_mode {
	/*
	 * CDS_FT_ITER_PATH_CACHED (default):
	 *   The iterator retains the traversal path between operations,
	 *   allowing O(1) next/prev/remove from the current position.
	 *   The RCU read-side lock must be held CONTINUOUSLY between
	 *   the operation that populates the iterator and any operation
	 *   that consumes the path.
	 */
	CDS_FT_ITER_PATH_CACHED = 0,

	/*
	 * CDS_FT_ITER_PATH_UNCACHED:
	 *   The iterator automatically discards the traversal path
	 *   after each operation returns. Every subsequent operation
	 *   performs a fresh top-down traversal from the current key.
	 *   The RCU read-side lock must be held only during each
	 *   individual operation and while accessing the returned
	 *   node — it may be dropped between operations.
	 *
	 *   This mode is suitable for iteration patterns where the
	 *   caller needs to drop the RCU read-side lock between steps
	 *   (e.g. to perform blocking work, acquire mutexes, or call
	 *   into subsystems that must not run in an RCU critical
	 *   section).
	 *
	 *   Example (uncached, per-step locking with refcount):
	 *
	 *     cds_ft_iter_set_path_mode(iter,
	 *                               CDS_FT_ITER_PATH_UNCACHED);
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
	CDS_FT_ITER_PATH_UNCACHED = 1,
};

/*
 * Duplicate nodes with the same key are chained into a doubly-linked
 * list. The last item of this list has a NULL next pointer.
 * The node needs to be zeroed or initialized with cds_ft_node_init
 * before being inserted into a Fractal Trie.
 *
 * The first node's prev pointer points to the parent internal node
 * (a flagged pointer distinguishable via ft_node_external() returning
 * false). Non-head nodes' prev pointers point to the preceding
 * cds_ft_node in the duplicate chain.
 *
 * The prev pointer is written by the mutation side (mutex-held) and
 * read by the write side and by the read side (exact lookup via
 * ft_skip_to_compressed, ordered-traversal parent-pointer walk via
 * ft_get_parent_rcu).  It is not accessed on the candidate lookup
 * fast path — RCU candidate readers only follow next pointers.
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
	/*
	 * prev pointer: for the head of the duplicate chain, this points
	 * to the parent internal node (flagged pointer, used by
	 * CDS_FT_FLAG_SKIP_COMPRESSED to recover the compressed node
	 * from a skip pointer, and by the ordered-traversal upward walk
	 * to reach the parent without consulting a cached iterator
	 * path). For non-head duplicates, this points to the preceding
	 * cds_ft_node. Written by the mutation side via
	 * rcu_assign_pointer; read on the read side via rcu_dereference.
	 * Not accessed on the candidate lookup fast path.
	 */
	void *prev;
	struct cds_ft_node *next;
};

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
 * The Fractal Trie keys most significant byte is first, and least
 * significant byte is last. This corresponds to a big endian integer.
 */

/*
 * External-node arena: bump-allocator with end-of-zone over-read
 * safety, suitable for embedding application leaf structures (those
 * containing a struct cds_ft_node) when the caller needs a trailing
 * readable pad past stored keys for SIMD-friendly comparison.
 *
 * Pattern: app embeds struct cds_ft_node in its own leaf struct,
 * allocates each leaf from this arena via cds_ft_external_arena_alloc,
 * and inserts into the trie.  The arena grows on demand as a linked
 * list of mmap'd ranges (default ~64 MiB each); each range reserves
 * a trailing guard page so the library's SIMD leaf compare can safely
 * over-read 32 bytes past any allocation's last byte without faulting,
 * regardless of where the allocation sits within its range.
 *
 * Lifetime: all-or-nothing.  cds_ft_external_arena_destroy frees
 * every allocation served by the arena.  The caller is responsible
 * for ensuring no RCU readers (and no remaining trie keys) reach
 * any allocation before destroy.
 *
 * Allocations are 8-byte aligned and zero-initialised.
 *
 * NUMA: pages lazy-fault onto the first-touching thread's local
 * node.  Callers that want a specific NUMA policy should pre-touch
 * the arena from the desired thread.
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
 * cds_ft_external_arena_attr_create - Allocate a default external-arena attr
 * (CDS_FT_OPTIMIZE_THROUGHPUT).  Destroy with
 * cds_ft_external_arena_attr_destroy.  Returns CDS_FT_STATUS_OK on success.
 */
enum cds_ft_status cds_ft_external_arena_attr_create(
		struct cds_ft_external_arena_attr **attr);

void cds_ft_external_arena_attr_destroy(struct cds_ft_external_arena_attr *attr);

/*
 * cds_ft_external_arena_attr_set_optimize - Set the arena's page-size policy
 * (see enum cds_ft_optimize).  Returns CDS_FT_STATUS_OK, or
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
 *         above 1 MiB are rejected with NULL.
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
 * The slot's size class is recovered from the containing range
 * header (no size argument needed).  Slots returned to the
 * freelist are reused by subsequent allocations of the same class.
 *
 * Caller must ensure no RCU reader still references @ptr (e.g.
 * via synchronize_rcu after removal from any published trie) — the
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
 * @key_readable_pad: Number of bytes guaranteed safely loadable PAST
 *                    the last byte of @key (i.e. starting at
 *                    key + key_len) without faulting.  0 means "no
 *                    over-read promised"; the library uses a
 *                    page-cross-safe fallback compare.  Values >= 32
 *                    let the library use unmasked SIMD loads on the
 *                    input side, avoiding page-cross checks on the
 *                    hot path.  A memory allocator that gives every
 *                    allocation a non-faulting trailing region (e.g.
 *                    cds_ft_external_arena's per-range guard page,
 *                    or a guard-page allocator) satisfies any value
 *                    up to that trailing-region size trivially.
 * @result_node: Candidate node output. Set to a node if a candidate is
 *               found, or NULL if not found or on error.
 *
 * The fastest lookup: the descent skips key comparison at compressed
 * nodes, so the returned node is a CANDIDATE that may not be an exact
 * match.  The caller MUST compare the returned node's key against the
 * lookup key to confirm; if the keys do not match, the lookup key is
 * not in the trie.  cds_ft_speculative_lookup_key wraps this with the
 * validating compare and is the recommended entry point for callers
 * that want an exact match.
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
 * Equivalent to cds_ft_lookup_candidate_key followed by a memcmp
 * against the candidate's stored key bytes.  Inlined into the caller
 * so the offset and length are visible to the compiler at the compare
 * site (typically allowing constant-folding when the caller knows
 * them).  The validation memcmp uses libc's optimized variant (e.g.
 * EVEX/AVX-512 on x86) resolved through the PLT at runtime.
 *
 * Validation is done caller-side (this memcmp) rather than in the
 * descent function, so the library pays no inline SIMD validation cost
 * on the hot path and the caller's memcmp inlines at the use site where
 * @key_offset and @key_len are typically constant.  cds_ft_eager_lookup_key
 * returns the same result with library-side validation instead.
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
	if (memcmp(key, (const uint8_t *) found + key_offset, key_len) != 0) {
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
 * the key during a precise descent (reading compressed-node bytes
 * directly), so no @key_offset is needed.  Strongest when the group is
 * tuned with CDS_FT_LOOKUP_OPTIMIZE_EAGER; on the default speculative-
 * tuned trie it still works but pays a small extra cost to unwrap
 * skip-encoded pointers before fetching compressed nodes.
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
 * result node, status, and backtracking state. Set the input key
 * with cds_ft_iter_set_key() before calling. On return, the iterator
 * holds the result key (cds_ft_iter_get_key()), result node
 * (cds_ft_iter_node()), status (cds_ft_iter_status()), and
 * backtracking path. The status is also returned by the function for
 * convenience.
 *
 * The RCU read-side lock must be held while calling these functions
 * and while accessing the returned node or reusing the iterator path.
 */

/*
 * cds_ft_lookup - Look up a node by key (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result node, status,
 *        and backtracking path.
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
 *        and backtracking path. The iterator's key length is set to
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
 *        CDS_FT_STATUS_INTERNAL_MATCH). The backtracking path is
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
 *        node, status, and backtracking path.
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
 *        node, status, and backtracking path.
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
 *        node, status, and backtracking path.
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
 *        node, status, and backtracking path.
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
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error. The
 * status is also stored in the iterator (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_last - Look up the node with the greatest key.
 * @ft: The Fractal Trie.
 * @iter: Iterator. On return, holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error. The
 * status is also stored in the iterator (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_next - Find the next node in lexicographical order.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at the current node.
 *        On return, advanced to the next node. The iterator holds
 *        the result key, key length, node, status, and backtracking
 *        path.
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
 *        the result key, key length, node, status, and backtracking
 *        path.
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
 * the returned node. For CDS_FT_ITER_PATH_CACHED iterators, this
 * means the RCU read-side critical section must span the entire
 * loop, since the cached path references internal nodes that could
 * be reclaimed after a grace period. For CDS_FT_ITER_PATH_UNCACHED
 * iterators, the lock may be dropped and reacquired within the loop
 * body, because the path is discarded after each operation:
 *
 *   cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);
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
 * cds_ft_for_each_rcu() for the locking rules for each path mode.
 */
#define cds_ft_for_each_entry_rcu(ft, iter, pos, member)			\
	for (cds_ft_lookup_first((ft), (iter));					\
			cds_ft_iter_node(iter) != NULL ?			\
				((pos) = cds_ft_entry(cds_ft_iter_node(iter),	\
					__typeof__(*(pos)), member), 1) : 0;	\
			cds_ft_next((ft), (iter)))

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
 * cds_ft_for_each_rcu() for the locking rules for each path mode.
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
 * cds_ft_for_each_rcu() for the locking rules for each path mode.
 */
#define cds_ft_for_each_entry_reverse_rcu(ft, iter, pos, member)		\
	for (cds_ft_lookup_last((ft), (iter));					\
			cds_ft_iter_node(iter) != NULL ?			\
				((pos) = cds_ft_entry(cds_ft_iter_node(iter),	\
					__typeof__(*(pos)), member), 1) : 0;	\
			cds_ft_prev((ft), (iter)))

/*
 * Mutation API
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
 * on error.
 *
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
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
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
 * Pointers to existing nodes returned by this function are only safe to
 * dereference as long as the writer mutual exclusion is held, or if the
 * caller wraps the operation in their own RCU read-side critical
 * section.
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
 * Returns a negative cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
 */
enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node);

/*
 * cds_ft_replace - Replace an existing node by a new node at the same key.
 * @ft: The Fractal Trie.
 * @iter: Iterator position at which @old_node is expected.
 *        If the iterator holds a valid path from a prior lookup,
 *        the replace operation may use it to avoid a full traversal.
 *        WARNING (CDS_FT_ITER_PATH_CACHED only): If the RCU read-side
 *        lock was dropped since the last iterator operation, you must
 *        call cds_ft_iter_invalidate_path() to invalidate the cached
 *        path before calling this function. CDS_FT_ITER_PATH_UNCACHED
 *        iterators handle this automatically and do not require
 *        cds_ft_iter_invalidate_path().
 * @old_node: Node to replace. Must be currently present in the trie.
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
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
 */
enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node);

/*
 * cds_ft_remove - Remove @node at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Iterator position at which @node is expected.
 *        If the iterator holds a valid path from a prior lookup,
 *        the remove operation may use it to avoid a full traversal.
 *        WARNING (CDS_FT_ITER_PATH_CACHED only): If the RCU read-side
 *        lock was dropped since the last iterator operation, you must
 *        call cds_ft_iter_invalidate_path() to invalidate the cached
 *        path before calling this function. CDS_FT_ITER_PATH_UNCACHED
 *        iterators handle this automatically and do not require
 *        cds_ft_iter_invalidate_path().
 * @node: Node to remove.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the node is not found, or a negative cds_ft_status on error.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming @node memory.
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
 */
enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node);

/*
 * cds_ft_remove_all - Remove the entire duplicate chain at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Iterator position identifying the key.
 *        If the iterator holds a valid path from a prior lookup,
 *        the remove operation may use it to avoid a full traversal.
 *        WARNING (CDS_FT_ITER_PATH_CACHED only): If the RCU read-side
 *        lock was dropped since the last iterator operation, you must
 *        call cds_ft_iter_invalidate_path() to invalidate the cached
 *        path before calling this function. CDS_FT_ITER_PATH_UNCACHED
 *        iterators handle this automatically and do not require
 *        cds_ft_iter_invalidate_path().
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
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
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
 * Concurrent RCU readers see the graft published by a single atomic
 * pointer update — either the pre-graft state or the post-graft state,
 * never a partial view — so the duration of writer mutual exclusion on
 * the main trie is short and bounded (independent of the number of
 * nodes being grafted, modulo the O(depth) descent and key-count
 * propagation). This pattern is well suited for batch loading,
 * sharding, and periodic bulk updates where minimizing the writer
 * critical section on the live trie is important.
 *
 *   // Phase 1: populate offline, no locking needed.
 *   cds_ft_create(group, NULL, &staging);
 *   for each (key, node) in batch:
 *       cds_ft_insert(staging, key, key_len, node);
 *
 *   // Phase 2: graft into the live trie, O(1) under lock.
 *   lock(&writer_mutex);
 *   cds_ft_graft(live_trie, prefix, prefix_len, staging);
 *   unlock(&writer_mutex);
 *   // staging is now empty but still valid; it can be reused
 *   // for the next batch or destroyed with cds_ft_destroy().
 *
 * Both source and destination tries must belong to the same group.
 * Mutual exclusion between writers on all affected tries is the
 * caller's responsibility. An RCU read-side lock must be held.
 * The source trie must not be the same object as the destination trie.
 *
 * Efficient bulk-removal pattern:
 *
 * Detach (or graft_swap) removes an entire sub-trie from the live
 * trie in a single O(1) operation. After a single grace period, no
 * concurrent reader can still hold a reference to any node in the
 * detached sub-trie. At that point, the detached trie is purely
 * local: the caller can iterate it and free all external nodes
 * directly, without observing a grace period for each individual
 * node. This reduces the cost of removing N nodes from N grace
 * periods (or N call_rcu callbacks) to a single grace period
 * followed by a local iteration.
 *
 *   // Phase 1: detach from the live trie, O(1) under lock.
 *   lock(&writer_mutex);
 *   cds_ft_detach(live_trie, prefix, prefix_len, &detached);
 *   unlock(&writer_mutex);
 *
 *   // Phase 2: wait for readers, then drain locally.
 *   synchronize_rcu();
 *   // detached is now purely local — no readers can access it.
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
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @src_ft: Source Fractal Trie. Must be in the same group as @dst_ft.
 *          On success, @src_ft becomes empty. The caller retains
 *          ownership of the (now empty) @src_ft object.
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
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_POPULATED_ERROR if the graft point is already
 * populated (destination has content at or below @key).
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the grafted keys would
 * exceed the group's maximum key length.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if the tries are not
 * in the same group, or if @src_ft is the same object as @dst_ft.
 * Returns a negative cds_ft_status on other errors.
 *
 * Mutual exclusion between writers on both @dst_ft and @src_ft is
 * the caller's responsibility.
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
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @swap_ft: Fractal Trie to exchange content with. Must be in the same
 *           group as @dst_ft. On entry, its content is grafted into
 *           @dst_ft at @key. On success, it receives the content that
 *           was previously at @key in @dst_ft, or is empty if the
 *           graft point had no content. The caller retains ownership.
 *
 * Exchanges the content at @key in @dst_ft with the content of
 * @swap_ft.  Concurrent RCU readers traversing @dst_ft observe
 * the swap published by a single atomic pointer update: they see
 * either the old content or the new content, never an empty
 * intermediate state.
 *
 * On success, the previous content at the graft point (if any) is
 * placed into @swap_ft. The caller can check cds_ft_empty(@swap_ft)
 * to determine whether there was pre-existing content. If @swap_ft
 * is non-empty, the "Efficient bulk-removal pattern" described above
 * applies: after a single grace period, the caller can drain
 * @swap_ft locally without per-node grace periods.
 *
 * The operation validates that @key_len plus the maximum used key
 * length of @swap_ft does not exceed the group's maximum key length.
 *
 * Root-level swap (@key_len 0) works with both fixed-length and
 * variable-length key groups. Non-root swap (@key_len > 0)
 * requires a variable-length key group (CDS_FT_LEN_VARIABLE).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the grafted keys would
 * exceed the group's maximum key length.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if the tries are not
 * in the same group, or if @swap_ft is the same object as @dst_ft.
 * Returns a negative cds_ft_status on other errors.
 *
 * Mutual exclusion between writers on both @dst_ft and @swap_ft is
 * the caller's responsibility.
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
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
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
 * inside it at return (the handle is freshly returned, and any
 * in-flight reader of the moved subtree has been drained by the
 * detach path itself).  A subsequent graft of the detached trie
 * therefore skips its internal synchronize_rcu(), coalescing
 * detach+graft into a single grace period.  Callers that publish
 * the detached trie to concurrent readers must call
 * cds_ft_make_concurrent() first.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_NOT_FOUND if nothing exists at @key.
 * Returns a negative cds_ft_status on error (including memory
 * allocation failure for the result trie).
 *
 * Mutual exclusion between writers on @ft is the caller's
 * responsibility.
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
 *          @dst_ft and must not equal @dst_ft.  May be in either
 *          exclusive or concurrent mode; concurrent readers on
 *          @src_ft are tolerated.
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
 * Algorithm (single call from the caller's POV):
 *
 *   1. Detach @src_ft at @key into a transient internal sub-trie.
 *      For a concurrent-mode @src_ft this incurs one grace period
 *      (the same one cds_ft_detach already needs to drain readers
 *      of the moved subtree); for an exclusive @src_ft no grace
 *      period is required.  After this step the transient is
 *      exclusive and @src_ft retains only the keys outside @key.
 *   2. Compute the longest common prefix (LCP) of the transient's
 *      stripped keys.  The full attach point in @dst_ft is
 *      @key concatenated with that LCP.
 *   3. If @dst_ft has no content under @key||LCP, move the
 *      transient as a single sub-trie:
 *        - LCP empty: graft the transient into @dst_ft at @key.
 *        - LCP non-empty: nested-detach the transient at LCP and
 *          graft the inner sub-trie into @dst_ft at @key||LCP.
 *      Works uniformly for variable-length and fixed-length
 *      groups; the intermediate stripped-key state is purely
 *      internal.
 *   4. Otherwise, drain the transient one duplicate chain at a
 *      time, re-prepending @key to each stripped key, and insert
 *      into @dst_ft.
 *
 * Concurrent RCU readers on @dst_ft observe a sequence of single
 * atomic publishes — either the fast-path graft or one per
 * per-entry insert — and never see a half-merged duplicate chain
 * at a given key.
 *
 * Returns CDS_FT_STATUS_OK on success (including the no-op case
 * where @src_ft has no content under @key).
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if either trie
 * pointer is NULL, if @dst_ft == @src_ft, if the tries are not in
 * the same group, or if @key_len exceeds the group's maximum key
 * length.
 * Returns a negative cds_ft_status on memory allocation failure.
 * On failure the function makes a best-effort rollback of the
 * initial detach by grafting the transient back into @src_ft at
 * @key; if the rollback itself fails, the affected externals are
 * leaked.  Both tries remain individually valid in any case.
 *
 * Mutual exclusion between writers on both @dst_ft and @src_ft is
 * the caller's responsibility.
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
 *          @dst_ft and must not equal @dst_ft.  May be in either
 *          exclusive or concurrent mode.
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
 * Same algorithm and same contract as cds_ft_merge, with @src_key
 * driving the initial detach and @dst_key driving the graft attach
 * point and per-entry key re-prepend.
 *
 * Returns the same statuses as cds_ft_merge.  In addition,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR is returned if either
 * @src_key_len or @dst_key_len exceeds the group's maximum key
 * length.
 */
enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len);

/*
 * Trie lifecycle
 */

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor);

/*
 * cds_ft_group_create - Create a Fractal Trie group.
 * @attr: Fractal Trie attributes.
 * @result_ft_group: Fractal Trie group output. Set to the newly created
 *                   trie group on success, or NULL on error.
 *
 * The @attr pointer is used to specify the Fractal Trie attributes. If
 * NULL, use default attribute values. The @attr can be destroyed
 * by the caller immediately after cds_ft_group_create() returns. The
 * caller keeps ownership of @attr. Default attributes select a variable
 * key length.
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
 * node. Duplicates at the same key are counted as one. The count is
 * maintained via per-node subtree counters propagated upward on each
 * mutation.
 *
 * This function has O(1) time complexity. The RCU read-side lock must
 * be held while calling this function. Concurrent updates may occur,
 * so the result is an approximation when updates are in progress.
 */
unsigned long cds_ft_count_keys(struct cds_ft *ft);

/*
 * cds_ft_count_keys_prefix - Return the number of unique keys under a prefix.
 * @ft: The Fractal Trie.
 * @prefix: The key prefix to count under.
 * @prefix_len: Length of the prefix in bytes. Use 0 to count all keys
 *              (equivalent to cds_ft_count_keys).
 *
 * Returns the number of distinct keys whose key starts with @prefix.
 * This descends through the trie following the prefix bytes, then reads
 * the subtree's propagated key counter.
 *
 * This function has O(prefix_len) time complexity. The RCU read-side
 * lock must be held while calling this function.
 */
unsigned long cds_ft_count_keys_prefix(struct cds_ft *ft,
		const uint8_t *prefix, size_t prefix_len);

/*
 * cds_ft_lookup_nth - Lookup the nth key in forward (smallest-first) order.
 * @ft: The Fractal Trie.
 * @iter: Iterator (must be created via cds_ft_iter_create).
 * @n: 0-indexed rank from the first (smallest) key.
 *
 * Finds the key at rank @n in the trie's sorted key order using
 * per-node key counters to skip entire subtrees. On success the
 * iterator points to the first external node of the nth key and the
 * result key is accessible via cds_ft_iter_get_key().
 *
 * Returns CDS_FT_STATUS_OK on success or CDS_FT_STATUS_NOT_FOUND if
 * @n >= the number of keys. O(depth) time complexity.
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
 * Descends from the right (largest children first) using per-node key
 * counters, so concurrent updates to the low end of the key space do
 * not affect the traversal. O(depth) time complexity.
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
 * Traverses locally from the current position: walks up the trie from
 * the current leaf counting rightward siblings using per-node key
 * counters, then descends into the target subtree. Only touches nodes
 * between the start and end positions, so concurrent mutations in
 * unrelated key ranges do not affect the result. O(depth) time
 * complexity. Returns CDS_FT_STATUS_NOT_FOUND if the target is out
 * of range. The RCU read-side lock must be held.
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
 * Traverses locally from the current position: walks up the trie from
 * the current leaf counting leftward siblings using per-node key
 * counters, then descends into the target subtree. Only touches nodes
 * between the start and end positions, so concurrent mutations in
 * unrelated key ranges do not affect the result. O(depth) time
 * complexity. Returns CDS_FT_STATUS_NOT_FOUND if @n exceeds the
 * number of preceding keys. The RCU read-side lock must be held.
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
 * cds_ft_key_len - Return the key length of a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the fixed key length if the trie is in fixed-length mode.
 * Returns CDS_FT_LEN_VARIABLE if the trie supports variable-length keys.
 */
size_t cds_ft_key_len(const struct cds_ft *ft);

/*
 * cds_ft_max_key_len - Return the maximum key length allowed by the trie.
 * @ft: The Fractal Trie.
 *
 * Returns the maximum length (in bytes) of any key that can be stored
 * in this trie instance.
 *
 * This value is intended for use by callers to allocate buffers for
 * output parameters (e.g., result_key). Even if the trie is configured
 * as "unlimited," this function returns a finite, implementation-defined
 * maximum.
 */
size_t cds_ft_max_key_len(const struct cds_ft *ft);

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
 * cds_ft_key_map - Return the key map of a Fractal Trie.
 * @ft: The Fractal Trie.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 *
 * Returns CDS_FT_STATUS_OK if there is a key mapping (output
 * parameters are populated). Returns CDS_FT_STATUS_NOT_FOUND if
 * the key map is the identity function.
 */
enum cds_ft_status cds_ft_key_map(const struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key);

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
 * Defaults to CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE when the group attr is
 * freshly created.  Callers that need strict eager descent (per-step
 * exact compare via cds_ft_eager_lookup_key, no skip-compressed
 * encoding) should select CDS_FT_LOOKUP_OPTIMIZE_EAGER explicitly.
 *
 * Speculative mode at the implementation level:
 *
 *   Speculative descent skips byte comparisons at compressed nodes.
 *   On architectures where the skip-compressed pointer encoding is
 *   available (64-bit with sufficient free high pointer bits), the
 *   compressed-node cache line is also bypassed entirely: the
 *   compressed path length and child pointer are encoded in the
 *   parent slot.  Compressed paths longer than the architecture
 *   limit fall back to traditional compressed node pointers
 *   transparently.  On architectures without the encoding, the
 *   compressed cache line is still loaded but no byte comparison
 *   runs — speculative descent still wins versus eager mode.
 *
 *   Caller requirement (skip-compressed-capable archs only):
 *   external node pointers (struct cds_ft_node *) stored in the
 *   trie must not carry metadata in their upper bits.  Pointer
 *   authentication (AArch64 PAC) or memory tagging (MTE)
 *   signatures must be stripped before the pointer is passed to
 *   the trie insertion API.
 *
 * Eager mode at the implementation level:
 *
 *   Precise descent reads compressed nodes via direct pointers; no
 *   skip-encoded unwrap is needed before fetching the node.
 *   cds_ft_lookup_candidate_key and cds_ft_speculative_lookup_key
 *   continue to work on eager-tuned tries but lose the
 *   skip-compressed encoding's CL bypass on supported archs.
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @opt value.
 * Never returns NOT_SUPPORTED: both modes work on every arch.  On
 * architectures without skip-compressed support, OPTIMIZE_SPECULATIVE
 * loses only the compressed-CL bypass; the per-step byte-compare
 * skip is unaffected.
 */
enum cds_ft_status cds_ft_group_attr_set_lookup_optimization(
		struct cds_ft_group_attr *attr,
		enum cds_ft_lookup_optimization opt);

/*
 * cds_ft_group_attr_set_numa_policy - Select the trie group's NUMA
 *                                     placement policy for its internal
 *                                     allocator.
 * @attr: Fractal Trie group attributes.
 * @policy: One of enum cds_ft_numa_policy
 *          (DEFAULT, INTERLEAVE, LOCAL).
 *
 * Defaults to CDS_FT_NUMA_DEFAULT when the group attr is freshly
 * created: the library applies no mbind() of its own, deferring to
 * whatever NUMA policy the process has configured (numactl, libnuma,
 * set_mempolicy()).  Applications wanting explicit library-side
 * placement should opt in via INTERLEAVE or LOCAL.
 *
 * Selection guide:
 *   - DEFAULT:    library imposes nothing; process / libnuma policy
 *                 takes effect (typically first-touch absent a
 *                 process-wide policy).  Use when running under
 *                 numactl or when the application configures
 *                 set_mempolicy() itself.
 *   - INTERLEAVE: 2 MiB-granular round-robin across allowed NUMA
 *                 nodes (mbind(MPOL_BIND) per chunk).  Best for
 *                 multi-reader workloads with readers distributed
 *                 across NUMA nodes.  Single-threaded performance is
 *                 within noise of LOCAL on modern x86 (the L2/L3
 *                 prefetcher works fine over 2 MiB chunks of
 *                 contiguous physical memory).  Enables transparent
 *                 hugepage collapse by keeping each 2 MiB chunk on
 *                 a single node.
 *   - LOCAL:      explicit first-touch (skip mbind, like DEFAULT).
 *                 Same syscall sequence as DEFAULT; differs in
 *                 intent: LOCAL means "I want first-touch on my
 *                 thread's node", DEFAULT means "I defer to whatever
 *                 the process / libnuma decides".
 *
 * The CDS_FT_NUMA_INTERLEAVE=0 environment variable, if set, forces
 * the library to skip mbind regardless of the group's policy —
 * useful for debugging without recompiling.  Otherwise the group's
 * policy takes effect.
 *
 * No effect on caller-provided external node storage
 * (cds_ft_external_arena_create has no group context; today it
 * applies CDS_FT_NUMA_DEFAULT — defers to process policy).
 *
 * Returns CDS_FT_STATUS_OK on success,
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @policy value.
 */
enum cds_ft_status cds_ft_group_attr_set_numa_policy(
		struct cds_ft_group_attr *attr,
		enum cds_ft_numa_policy policy);

/*
 * cds_ft_group_attr_set_optimize - Select the page-size policy for the group's
 * internal + compressed node arenas (see enum cds_ft_optimize).  Default:
 * CDS_FT_OPTIMIZE_THROUGHPUT.  Returns CDS_FT_STATUS_OK, or
 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR for an unknown @opt.
 */
enum cds_ft_status cds_ft_group_attr_set_optimize(
		struct cds_ft_group_attr *attr,
		enum cds_ft_optimize opt);

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
 * cds_ft_make_exclusive - Transition a Fractal Trie to exclusive
 *                         access discipline.
 * @ft: The Fractal Trie.
 *
 * Blocks in synchronize_rcu() to drain any in-flight RCU readers,
 * then marks the trie as exclusive.  Subsequent graft / graft_swap
 * operations with this trie as source skip their internal
 * synchronize_rcu().
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
 * source will internally synchronize_rcu() to drain readers before
 * re-parenting the content into another trie.
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
 * the writer scope-exit hook runs cds_ft_verify once every @period
 * mutations:
 *
 *   period == 0 : disable the verify walk on this trie (the
 *                 increment-and-compare in the hook still runs);
 *   period == 1 : verify every mutation (the historical
 *                 -DFEATURE_FT_VERIFY_AT_MUTATION cadence; this is
 *                 the default at cds_ft_create() time);
 *   period >  1 : verify every @period mutations — useful on large
 *                 tries where O(N) per mutation is impractical.
 *
 * The internal counter is reset to 0 on each period boundary, so it
 * never exceeds @period - 1 and there is no overflow / cadence-drift
 * concern on long-running workloads.  Setting a new period also
 * resets the counter, so the next verify lands @period mutations
 * from now.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED when the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION — the call surfaces the mismatch
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
 * FEATURE_FT_VERIFY_AT_MUTATION — distinguishing build-disabled
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
 * cds_ft_iter_set_path_mode - Set the iterator's path caching mode.
 * @iter: The iterator.
 * @mode: Path mode to set.
 *
 * Switching from CDS_FT_ITER_PATH_CACHED to CDS_FT_ITER_PATH_UNCACHED
 * immediately invalidates any cached path (equivalent to calling
 * cds_ft_iter_invalidate_path()). Switching from UNCACHED to CACHED
 * is always safe since there is no stale path.
 *
 * May be called at any point in the iterator's lifetime.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_set_path_mode(struct cds_ft_iter *iter,
		enum cds_ft_iter_path_mode mode);

/*
 * cds_ft_iter_get_path_mode - Get the iterator's path caching mode.
 * @iter: The iterator.
 *
 * Returns the current path mode.
 */
enum cds_ft_iter_path_mode cds_ft_iter_get_path_mode(
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
 * If the new key is a prefix of the current iterator key, the
 * internal backtracking path remains valid; otherwise it is
 * invalidated.
 *
 * Note: Calling this function safely invalidates the cached traversal
 * path (if the key differs). This is the required mechanism to safely
 * reuse an iterator if the RCU read-side lock was dropped since the
 * last operation.
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
 * Clears path, key, prefix, status, and node. The iterator remains
 * bound to its Fractal Trie.
 */
void cds_ft_iter_reset(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_invalidate_path - Invalidate the iterator's cached path.
 * @iter: The iterator.
 *
 * Invalidates the RCU-protected traversal path cached in the iterator.
 * The next operation on this iterator will be forced to perform a
 * fresh top-down traversal using the current key. The key and prefix
 * length are preserved.
 *
 * For CDS_FT_ITER_PATH_CACHED iterators, this function must be called
 * if the RCU read-side lock is dropped between operations on the same
 * iterator, to prevent use-after-free of the cached pointers.
 *
 * CDS_FT_ITER_PATH_UNCACHED iterators do not require this call — the
 * path is discarded automatically after each operation. Calling it on
 * an uncached iterator is harmless but unnecessary.
 */
void cds_ft_iter_invalidate_path(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_copy - Copy an iterator's state.
 * @dst: Destination iterator.
 * @src: Source iterator.
 *
 * Both iterators must be bound to the same Fractal Trie.
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
 * Recursively walks every internal and compressed node starting
 * from the root, checking that:
 * - nr_child matches the actual count of non-NULL child slots.
 * - nr_keys equals the sum of children's nr_keys plus the count
 *   of unique keys from local external node chains.
 * - Parent pointers of children point back to the correct parent.
 * - Compressed node invariants (len >= 1, no external_nodes on
 *   the compressed node itself).
 *
 * Must be called with mutual exclusion wrt updaters.
 *
 * Returns CDS_FT_STATUS_OK if the trie passes all checks, or
 * CDS_FT_STATUS_INTEGRITY_ERROR on integrity violation.
 */
enum cds_ft_status cds_ft_verify(const struct cds_ft *ft, FILE *out);

/*
 * cds_ft_compact - Defragment a trie's internal-node arenas in place.
 *
 * Relocates every internal node into freshly-allocated, densely-packed slots
 * in descent order; the now-empty source ranges drain and their memory is
 * reclaimed. Recovers the descent locality and resident memory that churn or
 * graft-based population fragmentation cost over time.
 *
 * One-shot convenience wrapper around cds_ft_compact_begin/step/end: the
 * caller must exclude concurrent writers on @ft for the whole call (the same
 * mutual-exclusion contract as the other mutators). Concurrent RCU readers are
 * permitted throughout, and other tries sharing the group keep mutating -- the
 * group stays online.
 *
 * Best-effort with respect to memory pressure: if an allocation fails while
 * walking, the affected node is left at its current address and the walk
 * continues. The trie remains valid and correct, only less fully compacted.
 *
 * Memory reclaim is deferred, like every other mutator: each relocated node's
 * old copy is freed through the RCU grace-period mechanism, and a drained
 * source range only releases its pages (MADV_DONTNEED) from inside that
 * deferred-free callback. So resident memory does NOT drop synchronously when
 * this returns -- it falls once a grace period elapses and the callbacks run.
 * A caller that wants the reclaim to have completed (e.g. to observe the lower
 * RSS, or before measuring) must wait for a grace period itself, for example
 * rcu_barrier(), ideally batched with the reclamation it already performs. The
 * library deliberately does not force this wait, to keep the caller in control
 * of grace-period timing (the resumable form below depends on that).
 *
 * To defragment a whole group, call this on each trie the group contains.
 */
void cds_ft_compact(struct cds_ft *ft);

/*
 * Resumable compaction (cds_ft_compact_begin / _step / _end).
 *
 * Lets a long compaction be interleaved with concurrent mutations of the
 * SAME trie: the caller drives it, holding its writer mutex only around each
 * cds_ft_compact_step and releasing it between steps so other writers (and
 * grace periods that drain reclaimed ranges) get a window. Each step uses the
 * leaf key reached so far as a resume cursor and re-descends from the root, so
 * it tolerates the structure changing between steps.
 *
 *   struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);
 *   do {
 *           writer_lock();
 *           more = cds_ft_compact_step(st, batch);
 *           writer_unlock();
 *   } while (more);
 *   cds_ft_compact_end(st);
 */
struct cds_ft_compact_state;

/*
 * cds_ft_compact_begin - Start a resumable compaction of @ft.
 *
 * Returns an opaque state to drive with cds_ft_compact_step, or NULL on
 * allocation failure. The returned state must be released with
 * cds_ft_compact_end.
 */
struct cds_ft_compact_state *cds_ft_compact_begin(struct cds_ft *ft);

/*
 * cds_ft_compact_step - Relocate up to @batch internal nodes.
 * @st: State from cds_ft_compact_begin.
 * @batch: Maximum nodes to relocate this step (0 selects a default).
 *
 * The caller must hold its writer exclusion for @ft across this call.
 * Returns true if more work remains (call again), false once the trie is
 * fully compacted. Either way the trie is valid at every step boundary.
 */
bool cds_ft_compact_step(struct cds_ft_compact_state *st, size_t batch);

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
 *   "children" array whose entries are
 *   {"key_byte": N, "child": <node>}.  Compressed nodes additionally
 *   carry "path_len" and "child".  External nodes carry just "ptr"
 *   and "kind".  Intended for programmatic consumption (visualizers,
 *   test assertions).
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
	for (; (pos) != NULL; (pos) = rcu_dereference((pos)->next))

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
			(node) = rcu_dereference((node)->next))

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
			((p) = rcu_dereference((pos)->next), 1) : 0;	\
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
			(p) = rcu_dereference((node)->next), 1) : 0;	\
			(node) = (p))

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FRACTAL_TRIE_H */
