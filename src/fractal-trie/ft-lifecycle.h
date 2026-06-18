// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-lifecycle.h
 *
 * Userspace RCU library - Fractal Trie: group / trie / attr create + destroy, group accessors, verify-at-mutation knobs.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-lifecycle.h is an implementation unit; #include it from fractal-trie.c only"
#endif

unsigned long cds_ft_count_entries(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	unsigned long count = 0;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK) {
		FT_TP(count_entries, (unsigned long) 0);
		return 0;
	}
	cds_ft_for_each_rcu(ft, iter) {
		struct cds_ft_node *node = cds_ft_iter_node(iter);

		cds_ft_for_each_duplicate_rcu(node)
			count++;
	}
	if (cds_ft_iter_status(iter) < 0)
		count = 0;
	cds_ft_iter_destroy(iter);
	FT_TP(count_entries, count);
	return count;
}

/*
 * Validate that the pointer bits used by the skip-compressed encoding
 * are outside the process's virtual address range.  Attempt to mmap a
 * page at the encoding boundary; if the mapping succeeds (or fails with
 * EEXIST) the bit is within the VA range and skip-compressed cannot be
 * used safely.  Only ENOMEM (address beyond TASK_SIZE) confirms the bit
 * is available, and only after a positive control rules out a spurious
 * ENOMEM from a global mmap failure (see below); any other failure
 * (EPERM, EINVAL, EAGAIN, seccomp, ...) is treated conservatively as
 * unavailable.
 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
static
bool ft_skip_compressed_validate(void)
{
	size_t page = urcu_get_page_len();
	void *p, *ctrl;

	p = mmap((void *)(1UL << FT_SKIP_LEN_SHIFT), page, PROT_NONE,
		 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	if (p != MAP_FAILED) {
		/* Mapping succeeded: the bit is within the VA range. */
		munmap(p, page);
		return false;
	}
	if (errno != ENOMEM)
		return false;	/* EEXIST / EPERM / EINVAL / seccomp / ... */
	/*
	 * ENOMEM is overloaded: besides "address beyond TASK_SIZE" (what we
	 * want to confirm) it is also returned for a global mmap failure --
	 * notably vm.max_map_count exhaustion -- regardless of the address.
	 * Cross-check with a positive control: map one page at a
	 * kernel-chosen address.  If the control succeeds, mmap is healthy
	 * and the boundary ENOMEM was a genuine out-of-range rejection, so
	 * the bit is available.  If the control also fails, mmap is failing
	 * globally and the boundary ENOMEM proves nothing -- stay
	 * conservative and leave skip-compressed disabled.
	 */
	ctrl = mmap(NULL, page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (ctrl == MAP_FAILED)
		return false;
	munmap(ctrl, page);
	return true;
}
#endif

enum cds_ft_status cds_ft_group_attr_create(struct cds_ft_group_attr **result)
{
	struct cds_ft_group_attr *attr = calloc(1, sizeof(struct cds_ft_group_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	attr->key_len = CDS_FT_LEN_DEFAULT;
	attr->max_key_len = FT_MAX_KEY_LEN;
	attr->key_map.identity = true;
	/*
	 * Default lookup optimization is SPECULATIVE: speculative
	 * descent with validate-and-retry, plus opportunistic
	 * skip-compressed pointer encoding on supported archs.
	 * Callers that need strict EAGER (per-step exact compare, no
	 * skip-compressed) must call
	 * cds_ft_group_attr_set_lookup_optimization(attr,
	 * CDS_FT_LOOKUP_OPTIMIZE_EAGER) explicitly.
	 */
	attr->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_skip_compressed_validate())
		attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
	/*
	 * Ordered sibling list ON by default: the library-owned cell is allocated
	 * per head regardless, so the key-ordered cds_ft_next / cds_ft_prev /
	 * cds_ft_for_each*_batched fast paths are available out of the box.  A
	 * group that never iterates in key order and wants to skip the per-mutation
	 * splice / unsplice opts out with cds_ft_group_attr_set_ordered_list(attr, false).
	 */
	attr->ordered_list_set = true;
	/*
	 * NUMA placement default: defer to process / libnuma policy.
	 * The library applies no mbind() of its own; the kernel honors
	 * numactl wrappers, set_mempolicy() calls, or falls back to
	 * first-touch when no process policy is set.  Applications that
	 * want explicit library-side placement should opt into
	 * CDS_FT_NUMA_INTERLEAVE (multi-reader workloads) or
	 * CDS_FT_NUMA_LOCAL (single-threaded / sharded workloads) via
	 * cds_ft_group_attr_set_numa_policy.
	 */
	attr->numa_policy = CDS_FT_NUMA_DEFAULT;
	attr->optimize = CDS_FT_OPTIMIZE_THROUGHPUT;
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_group_attr_destroy(struct cds_ft_group_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_group_attr_set_key_len(struct cds_ft_group_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_max_key_len(struct cds_ft_group_attr *attr, size_t max_key_len)
{
	if (max_key_len == CDS_FT_MAX_LEN_UNLIMITED) {
		attr->max_key_len = FT_MAX_KEY_LEN;
		return CDS_FT_STATUS_OK;
	}
	if (max_key_len > FT_MAX_KEY_LEN)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->max_key_len = max_key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_key_map(struct cds_ft_group_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key)
{
#ifndef FEATURE_FT_KEY_MAP
	/* Built without non-identity key-map support (byte-ordinal keys only). */
	(void) attr;
	(void) key_to_ordinal;
	(void) ordinal_to_key;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#else
	size_t i;

	if (!key_to_ordinal || !ordinal_to_key)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/*
	 * Verify the two maps are exact inverse permutations.  Checking
	 * ordinal_to_key[key_to_ordinal[i]] == i for every i is sufficient:
	 * it forces key_to_ordinal to be injective (hence a bijection over
	 * the byte values) and ordinal_to_key to be its inverse (hence also
	 * a bijection), so both maps are valid permutations.
	 */
	for (i = 0; i < sizeof(attr->key_map.key_to_ordinal); i++) {
		if (ordinal_to_key[key_to_ordinal[i]] != i)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
#endif /* FEATURE_FT_KEY_MAP */
}

enum cds_ft_status cds_ft_group_attr_set_lookup_optimization(
		struct cds_ft_group_attr *attr,
		enum cds_ft_lookup_optimization opt)
{
	switch (opt) {
	case CDS_FT_LOOKUP_OPTIMIZE_EAGER:
		attr->speculative = false;
		attr->flags &= ~CDS_FT_FLAG_SKIP_COMPRESSED;
		return CDS_FT_STATUS_OK;
	case CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE:
		/*
		 * Speculative descent always works.  Opportunistically
		 * enable the skip-compressed pointer encoding when the
		 * arch supports it; on archs without it the descent still
		 * skips per-step byte compares -- just without the extra
		 * compressed-CL bypass.
		 */
		attr->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_skip_compressed_validate())
			attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
}

enum cds_ft_status cds_ft_group_attr_set_speculative_key_offset(
		struct cds_ft_group_attr *attr,
		size_t key_offset)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->speculative_key_offset = key_offset;
	attr->speculative_key_offset_set = true;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_key_len_offset(
		struct cds_ft_group_attr *attr,
		size_t offset)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->key_len_offset = offset;
	attr->key_len_offset_set = true;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_ordered_list(
		struct cds_ft_group_attr *attr, bool ordered_list)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->ordered_list_set = ordered_list;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_numa_policy(
		struct cds_ft_group_attr *attr,
		enum cds_ft_numa_policy policy)
{
	switch (policy) {
	case CDS_FT_NUMA_DEFAULT:
	case CDS_FT_NUMA_INTERLEAVE:
	case CDS_FT_NUMA_LOCAL:
		attr->numa_policy = policy;
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
}

enum cds_ft_status cds_ft_group_attr_set_optimize(
		struct cds_ft_group_attr *attr,
		enum cds_ft_optimize opt)
{
	switch (opt) {
	case CDS_FT_OPTIMIZE_THROUGHPUT:
	case CDS_FT_OPTIMIZE_RSS:
		attr->optimize = opt;
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
}

enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result)
{
	struct cds_ft_attr *attr = calloc(1, sizeof(struct cds_ft_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * The per-instance attr currently carries only the exclusive-
	 * access flag (see cds_ft_attr_set_exclusive); calloc leaves it
	 * false (concurrent RCU readers permitted) as the default.
	 */
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_attr_destroy(struct cds_ft_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_attr_set_exclusive(struct cds_ft_attr *attr,
		bool exclusive)
{
	attr->exclusive = exclusive;
	return CDS_FT_STATUS_OK;
}

void cds_ft_make_exclusive(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	if (ft->exclusive)
		return;
	ft->group->flavor->update_synchronize_rcu();
	ft->exclusive = true;
}

void cds_ft_make_concurrent(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	ft->exclusive = false;
}

bool cds_ft_is_exclusive(const struct cds_ft *ft)
{
	return ft->exclusive;
}

bool cds_ft_excl_validate_enabled(void)
{
#ifdef FEATURE_FT_EXCL_VALIDATE
	return true;
#else
	return false;
#endif
}

bool cds_ft_verify_at_mutation_enabled(void)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	return true;
#else
	return false;
#endif
}

/*
 * Set the verify-at-mutation sampling period for @ft.  When the
 * library is built with -DFEATURE_FT_VERIFY_AT_MUTATION, the writer
 * scope-exit hook runs cds_ft_verify once every @period mutations.
 *
 *   period == 0 : disable the verify walk on this trie (the
 *                 increment-and-compare still runs in the hook).
 *   period == 1 : verify at every mutation (the historical
 *                 -DFEATURE_FT_VERIFY_AT_MUTATION cadence).
 *   period >  1 : verify every @period mutations -- useful on large
 *                 tries where O(N) per mutation is impractical.
 *
 * The counter is reset to 0 on each period boundary, so it never
 * exceeds @period - 1 and there is no overflow / cadence-drift
 * concern on long-running workloads.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED if the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION -- the call surfaces the mismatch
 * loudly rather than silently doing nothing, so a test that relies
 * on the verify cadence cannot accidentally run with verify-at-
 * mutation compiled out.  Use cds_ft_verify_at_mutation_enabled()
 * to gate the call.
 *
 * Write-side only (mutex-held); not safe to call concurrently
 * with writers on the same trie.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_set(struct cds_ft *ft,
		unsigned long period)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	ft->verify_at_mutation_period = period;
	ft->verify_at_mutation_counter = 0;
	return CDS_FT_STATUS_OK;
#else
	(void) ft;
	(void) period;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

/*
 * Read the verify-at-mutation sampling period for @ft into
 * *@period.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED if the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION -- distinguishing the
 * build-disabled case from a runtime period == 0.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_get(
		const struct cds_ft *ft, unsigned long *period)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	*period = ft->verify_at_mutation_period;
	return CDS_FT_STATUS_OK;
#else
	(void) ft;
	(void) period;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft_group *ft_group;
	size_t key_len = CDS_FT_LEN_DEFAULT,
	       max_key_len = FT_MAX_KEY_LEN;

	ft_specialized_scan_layout_assert();
	if (attr) {
		key_len = attr->key_len;
		max_key_len = attr->max_key_len;
	}
	/* max_tree_depth 0 is for pointer to root node */
	if (key_len != CDS_FT_LEN_VARIABLE && key_len > max_key_len) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	ft_group = calloc(1, sizeof(*ft_group));
	if (!ft_group) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_group->key_len = key_len;
	ft_group->max_key_len = max_key_len;
	ft_group->max_tree_depth = max_key_len + 1;
	assert(ft_group->max_tree_depth <= FT_MAX_DEPTH);
	ft_group->flavor = flavor;
	pthread_mutex_init(&ft_group->arena_lock, NULL);
	if (attr) {
		ft_group->key_map = attr->key_map;
		ft_group->flags = attr->flags;
		ft_group->speculative = attr->speculative;
		ft_group->speculative_key_offset = attr->speculative_key_offset;
		ft_group->speculative_key_offset_set = attr->speculative_key_offset_set;
		ft_group->key_len_offset = attr->key_len_offset;
		ft_group->key_len_offset_set = attr->key_len_offset_set;
		ft_group->ordered_list_set = attr->ordered_list_set;
		ft_group->numa_policy = attr->numa_policy;
		ft_group->optimize = attr->optimize;
	} else {
		/*
		 * NULL attr: mirror the defaults set by
		 * cds_ft_group_attr_create -- identity key map plus
		 * SPECULATIVE lookup optimization with opportunistic
		 * SKIP_COMPRESSED on supported archs, and the DEFAULT NUMA
		 * policy (interleave unless the process set an explicit
		 * preference -- see ft_apply_interleave).
		 */
		ft_group->key_map.identity = true;
		ft_group->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_skip_compressed_validate())
			ft_group->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
		ft_group->ordered_list_set = true;	/* on by default; see attr_create */
		ft_group->numa_policy = CDS_FT_NUMA_DEFAULT;
		ft_group->optimize = CDS_FT_OPTIMIZE_THROUGHPUT;
	}
	*result_ft_group = ft_group;
	FT_TP(group_create, (const void *) ft_group);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_destroy(struct cds_ft_group *ft_group)
{
	if (uatomic_load(&ft_group->nr_ft_instances, CMM_RELAXED) != 0)
		return CDS_FT_STATUS_BUSY_ERROR;
	FT_TP(group_destroy, (const void *) ft_group);
	/*
	 * All tries are destroyed (nr_ft_instances == 0) and each
	 * cds_ft_destroy drained its deferred frees, so every ordered-list
	 * cell and every internal/compressed node that was allocated should
	 * have had its free issued.  Both balances are group-scoped because
	 * the bulk ops migrate cells and nodes between the group's tries.
	 */
	if (ft_debug_counters() &&
	    ft_group->nr_cells_allocated != ft_group->nr_cells_freed) {
		fprintf(stderr,
			"[error] Fractal Trie leaked %ld ordered-list cells. Allocated: %lu, freed: %lu.\n",
			(long) (ft_group->nr_cells_allocated - ft_group->nr_cells_freed),
			ft_group->nr_cells_allocated, ft_group->nr_cells_freed);
	}
	if (ft_debug_counters() &&
	    ft_group->nr_nodes_allocated != ft_group->nr_nodes_freed) {
		fprintf(stderr, "[error] Fractal Trie leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) (ft_group->nr_nodes_allocated - ft_group->nr_nodes_freed),
			ft_group->nr_nodes_allocated, ft_group->nr_nodes_freed);
		fprintf(stderr, "  internal: alloc=%lu freed=%lu leaked=%ld\n",
			ft_group->nr_internal_alloc, ft_group->nr_internal_freed,
			(long) (ft_group->nr_internal_alloc - ft_group->nr_internal_freed));
		fprintf(stderr, "  compressed: alloc=%lu freed=%lu leaked=%ld\n",
			ft_group->nr_compressed_alloc, ft_group->nr_compressed_freed,
			(long) (ft_group->nr_compressed_alloc - ft_group->nr_compressed_freed));
	}
	cds_ft_free_all_arenas(ft_group);
	pthread_mutex_destroy(&ft_group->arena_lock);
	free(ft_group);
	return CDS_FT_STATUS_OK;
}

#ifdef DEBUG_COUNTERS
/*
 * DEBUG_COUNTERS-only cell leak introspection (no public header decl; tests
 * weak-reference it).  Reports the group's ordered-list cell alloc / free
 * balance.  Cells migrate between the group's tries via the bulk ops, so the
 * balance is meaningful only at group granularity; after every trie is drained
 * @allocated should equal @freed.  Caller must ensure no concurrent cell
 * mutation.
 */
void cds_ft_debug_cell_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed);
void cds_ft_debug_cell_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed)
{
	if (allocated)
		*allocated = group->nr_cells_allocated;
	if (freed)
		*freed = group->nr_cells_freed;
}

/*
 * DEBUG_COUNTERS-only node leak introspection (no public header decl; tests
 * weak-reference it).  Reports the group's internal/compressed node alloc /
 * free balance.  Nodes migrate between the group's tries via the bulk ops, so
 * the balance is meaningful only at group granularity; after every trie is
 * drained @allocated should equal @freed.  Caller must ensure no concurrent
 * node mutation.
 */
void cds_ft_debug_node_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed);
void cds_ft_debug_node_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed)
{
	if (allocated)
		*allocated = group->nr_nodes_allocated;
	if (freed)
		*freed = group->nr_nodes_freed;
}
#endif

enum cds_ft_status cds_ft_create(struct cds_ft_group *ft_group,
		const struct cds_ft_attr *attr,
		struct cds_ft **result_ft)
{
	struct cds_ft *ft;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *metadata;
	const struct cds_ft_type *type0 = &ft_types[0];

	ft = calloc(1, sizeof(*ft));
	if (!ft) {
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->group = ft_group;
	/* Cache the group's ordered-list mode for the read-side cell gate. */
	ft->ordered_list = ft_group->ordered_list_set;
	ft_install_lookup_ops(ft);
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * Default to verify-every-mutation cadence to preserve the
	 * historical -DFEATURE_FT_VERIFY_AT_MUTATION behavior; tests
	 * working with large tries can call
	 * cds_ft_set_verify_at_mutation_period() to dial it down.
	 * Counter is already zero from calloc.
	 */
	ft->verify_at_mutation_period = 1;
#endif
	if (attr)
		ft->exclusive = attr->exclusive;

	/*
	 * Allocate the root node (smallest popcount_2l type, initially empty).
	 *
	 * ft->root always points to an internal node, even when the
	 * trie has no entries (nr_child == 0).  This is the one place
	 * where a node with 0 children is allowed; all other internal
	 * nodes are pruned when their last child is removed.  The node
	 * itself may be replaced by graft or graft-swap, but the
	 * invariant on the slot is maintained across all operations.
	 *
	 * The root is a regular internal node whose metadata
	 * (nr_child, external_nodes) is accessed the same way as any
	 * other node's.  Its metadata carries the NIL-key entries, so
	 * transplanting a root node between tries is a single pointer
	 * swap with no metadata relocation.
	 */
	root_node = alloc_cds_ft_node(ft, type0, &metadata);
	if (!root_node) {
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->root = ft_node_flag(root_node, 0);
	FT_TP(root_publish, (const void *) ft, (const void *) ft->root);

	/*
	 * Ordinal-cell list enabled: eagerly allocate the writer-side scratch
	 * iterator used for cell predecessor discovery (ft_ord_cell_splice).
	 * ord_cell_head / ord_cell_tail are NULL from calloc.
	 */
	if (ft_group->ordered_list_set &&
	    cds_ft_iter_create(ft, &ft->ord_cell_scratch_iter) != CDS_FT_STATUS_OK) {
		free_cds_ft_node_unpublished(ft, root_node);
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	uatomic_inc(&ft_group->nr_ft_instances, CMM_RELAXED);
	*result_ft = ft;
	FT_TP(ft_create, (const void *) ft, (const void *) ft_group);
	return CDS_FT_STATUS_OK;
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the
 * caller).
 */
void cds_ft_destroy(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	/*
	 * A compaction the caller never ended would otherwise strand its
	 * private ranges (relocated nodes in arena-untracked ranges) and leak
	 * its state/iter.  Finalize it here: cds_ft_compact_end merges the
	 * private ranges back into the arenas and frees the state, leaving a
	 * consistent trie to tear down.
	 */
	if (caa_unlikely(ft->active_compact != NULL)) {
		fprintf(stderr,
			"cds_ft_destroy: trie %p destroyed with a compaction still in progress; finalizing (missing cds_ft_compact_end)\n",
			(void *) ft);
		cds_ft_compact_end(ft->active_compact);
	}
	FT_TP(ft_destroy, (const void *) ft);
	/* Free root node. No concurrent readers at this point. */
	free_cds_ft_node(ft, ft_node_ptr(ft->root));
	/*
	 * Wait for in-flight call_rcu free to complete, so every deferred
	 * node free this trie issued is counted (group-scoped) before the
	 * trie's instance count is dropped.  The group-level node-leak check
	 * runs in cds_ft_group_destroy, once all tries have drained.
	 */
	flavor->barrier();
	if (ft->ord_cell_scratch_iter)
		cds_ft_iter_destroy(ft->ord_cell_scratch_iter);
	uatomic_dec(&ft->group->nr_ft_instances, CMM_RELAXED);
	free(ft);
}
