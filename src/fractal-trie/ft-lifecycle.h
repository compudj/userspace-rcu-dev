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
	 * Order-statistics (per-node nr_keys) default OFF: calloc already left
	 * rank_stats_set false.  A group that needs O(1) cds_ft_count_keys /
	 * O(depth) cds_ft_lookup_nth / cds_ft_iter_skip_* opts in with
	 * cds_ft_group_attr_set_rank_stats(attr, true); otherwise those queries
	 * fall back to enumeration / iteration.
	 */
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

enum cds_ft_status cds_ft_group_attr_set_rank_stats(
		struct cds_ft_group_attr *attr, bool rank_stats)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->rank_stats_set = rank_stats;
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

enum cds_ft_status cds_ft_attr_set_speculative_keys(struct cds_ft_attr *attr,
		bool enabled)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->speculative_keys_disabled = !enabled;
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
		ft_group->rank_stats_set = attr->rank_stats_set;
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
		unsigned long *allocated, unsigned long *freed)
{
	if (allocated)
		*allocated = group->nr_nodes_allocated;
	if (freed)
		*freed = group->nr_nodes_freed;
}
#endif

/*
 * Root-is-internal probe (no public header decl; the inv suite weak-references
 * it).  Reads ft->root under the caller's RCU read lock and returns whether it
 * tags a plain internal node.  Used by inv_root_always_internal to sample the
 * root CONCURRENTLY with a re-rooting bulk op (graft_swap) -- a transient
 * compressed / skip-compressed root, which no mutator should ever publish,
 * would show up here even though a post-op probe (after canonicalization)
 * cannot see it.
 */
int cds_ft_debug_root_is_internal(struct cds_ft *ft);
int cds_ft_debug_root_is_internal(struct cds_ft *ft)
{
	/*
	 * Resolve a flip proxy: the empty-dst root-level graft fuses the root
	 * swap with the ordered-list head/tail transfer in one flip, parking a
	 * transient proxy here.  It resolves to an internal node in BOTH phases
	 * (old empty root XOR new src root), so the root-always-internal
	 * invariant this probe checks still holds through the flip.
	 */
	struct cds_ft_inode_flag *rf = ft_root_dereference(ft);

	return ft_node_internal(rf) ? 1 : 0;
}

/*
 * Install per-API lookup function pointers on @ft based on group
 * flags.  Called once at cds_ft_create.  Pointers are stable for
 * the trie's lifetime because the group flags (key_map.identity,
 * CDS_FT_FLAG_SKIP_COMPRESSED) are immutable after group creation.
 */
static
void ft_install_lookup_ops(struct cds_ft *ft)
{
	const struct cds_ft_group *group = ft->group;
	bool sc = ft_group_skip_compressed(group);
	/*
	 * Limit-none relational entries: resolve the immutable use_keycopy config
	 * once and install the matching specialization, so cds_ft_lookup_le/ge/lt/gt
	 * (and cds_ft_next/prev) tail-call through @ft with no per-call config
	 * branch.  Independent of key_map.identity (ft_speculative_keycopy handles
	 * non-identity via ft_key_to_ordinals), so installed before the early
	 * non-identity return below.
	 */
	bool kc = ft->speculative_key_offset_active && group->speculative &&
			(group->flags & CDS_FT_FLAG_SKIP_COMPRESSED);

	ft->lookup_le_fn = kc ? ft_ineq_le_keycopy : ft_ineq_le_eager;
	ft->lookup_ge_fn = kc ? ft_ineq_ge_keycopy : ft_ineq_ge_eager;
	ft->lookup_lt_fn = kc ? ft_ineq_lt_keycopy : ft_ineq_lt_eager;
	ft->lookup_gt_fn = kc ? ft_ineq_gt_keycopy : ft_ineq_gt_eager;

	/*
	 * Iter-form lookup_iter_fn: iter always carries ordinals-mapped
	 * key bytes (see cds_ft_iter_set_key), so the non-identity key_map
	 * case is handled at iter-set time and the lookup-time inner only
	 * needs (skip_compressed) specialization.
	 */
	ft->lookup_iter_fn = sc
		? ft_lookup_iter_precise_sc : ft_lookup_iter_precise_nosc;
	/*
	 * Partial-match (tracking=PARTIAL) is precise descent.  Iter form
	 * has no non-identity issue (iter holds ordinals already).
	 */
	ft->lookup_partial_iter_fn = sc
		? ft_lookup_partial_iter_sc : ft_lookup_partial_iter_nosc;
	/*
	 * Longest-match (tracking=LONGEST) is also always precise
	 * descent.  Same shape as partial.
	 */
	ft->lookup_longest_match_iter_fn = sc
		? ft_lookup_longest_match_iter_sc
		: ft_lookup_longest_match_iter_nosc;
#ifdef FEATURE_FT_KEY_MAP
	if (caa_unlikely(!group->key_map.identity)) {
		ft->lookup_key_fn = ft_lookup_key_nonidentity;
		ft->lookup_candidate_key_fn = ft_lookup_candidate_key_nonidentity;
		ft->lookup_partial_key_fn = ft_lookup_partial_key_nonidentity;
		ft->lookup_longest_match_key_fn = ft_lookup_longest_match_key_nonidentity;
		return;
	}
#endif
	ft->lookup_key_fn = sc
		? ft_lookup_precise_sc : ft_lookup_precise_nosc;
	ft->lookup_candidate_key_fn = sc
		? ft_lookup_cand_sc : ft_lookup_cand_nosc;
	ft->lookup_partial_key_fn = sc
		? ft_lookup_partial_key_sc : ft_lookup_partial_key_nosc;
	ft->lookup_longest_match_key_fn = sc
		? ft_lookup_longest_match_key_sc
		: ft_lookup_longest_match_key_nosc;
}

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
	/* Cache the group's order-statistics mode (per-node nr_keys upkeep). */
	ft->rank_stats = ft_group->rank_stats_set;
	/*
	 * A fresh trie's ordinal-cell list is empty: init the circular sentinel so
	 * it points at itself (calloc would leave it NULL, which is not a valid
	 * empty circular list).  Done unconditionally -- cheap, and every created
	 * trie (incl. the transient detach / merge tries) flows through here.
	 */
	urcu_txn_sw_list_init(&ft->ord_sentinel);
	/*
	 * Effective per-trie speculative-key state: the group is configured for
	 * speculative result-key capture AND this trie did not opt out via
	 * cds_ft_attr_set_speculative_keys(attr, false).  Set BEFORE
	 * ft_install_lookup_ops (which keys @kc off it) so an opted-out trie gets
	 * the EAGER lookup specializations.
	 */
	ft->speculative_key_offset_active = ft_group->speculative_key_offset_set &&
		(!attr || !attr->speculative_keys_disabled);
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
	 * iterator used for cell predecessor discovery
	 * (ft_ord_cell_find_pred_from_head).  The ordinal-cell list sentinel was
	 * already inited (empty) above.
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
	/*
	 * Free root node.  No concurrent readers at this point.  The root was a
	 * published (reader-visible) node throughout the trie's life, so it gets
	 * its freeze-on-free tombstone like any other retired node (doc §4.B);
	 * here it is a teardown no-op, but it keeps the mark-before-free
	 * invariant universal (see FT_DEBUG_TOMBSTONE_AUDIT).
	 */
	ft_meta_tombstone_set_flip(cds_ft_item_to_metadata(ft_node_ptr(ft->root)));
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

/*
 * Group / trie accessors: key lengths, key map, and emptiness query.
 */
size_t cds_ft_group_key_len(const struct cds_ft_group *group)
{
	return group->key_len;
}

size_t cds_ft_group_max_key_len(const struct cds_ft_group *group)
{
	return group->max_key_len;
}

size_t cds_ft_max_used_key_len(const struct cds_ft *ft)
{
	return uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
}

enum cds_ft_status cds_ft_group_key_map(const struct cds_ft_group *group, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key)
{
	if (group->key_map.identity)
		return CDS_FT_STATUS_NOT_FOUND;
	memcpy(key_to_ordinal, group->key_map.key_to_ordinal, sizeof(group->key_map.key_to_ordinal));
	memcpy(ordinal_to_key, group->key_map.ordinal_to_key, sizeof(group->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

bool cds_ft_empty(struct cds_ft *ft)
{
	struct cds_ft_inode_flag *root_flag;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *rmeta;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	root_flag = ft_root_dereference(ft);
	root_node = ft_node_ptr(root_flag);

	/*
	 * The root is always an internal node (invariant enforced by
	 * ft_make_root_internal_glue at every site that publishes
	 * ft->root), so no tag dispatch is needed before reading metadata.
	 */
	rmeta = cds_ft_item_to_metadata(root_node);

	/*
	 * Empty trie: the root has no children and no NIL-key entries.
	 * For popcount root, nr_child is derived from the bitmap and a
	 * freshly-allocated (calloc'd) root with bitmap == 0 correctly
	 * reports nr_child == 0.
	 */
	if (ft_meta_nr_child(rmeta) != 0)
		return false;
	return !uatomic_load(&rmeta->external_nodes, CMM_RELAXED);
}

/*
 * Trie statistics introspection: walk the trie collecting per-level,
 * per-node-type counts and child-count distributions, then render them.
 * Public debug API (cds_ft_recompute_stats / cds_ft_show_stats); off all hot
 * paths.  Lives here beside the other group-level introspection accessors.
 */
struct cds_ft_node_stats {
	uint64_t count;
	uint64_t distribution[257];
};

struct cds_ft_stats_level {
	uint64_t nr_external_nodes;
	uint64_t nr_metadata_external_nodes;
	uint64_t nr_duplicate_external_nodes;
	uint64_t nr_internal_nodes;
	uint64_t nr_compressed_nodes;
	struct cds_ft_node_stats node_stats[FT_TYPE_MAX_NR];
	bool has_nodes;
};

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
};

enum cds_ft_status cds_ft_recompute_stats(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	size_t max_len = 0;

	CDS_FT_SCOPED_WRITER(ft);
	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return status;
	cds_ft_for_each_rcu(ft, iter) {
		size_t klen = ft_iter_resolve_key_len(iter);

		if (klen > max_len)
			max_len = klen;
	}
	status = cds_ft_iter_status(iter);
	cds_ft_iter_destroy(iter);
	if (status < 0)
		return status;
	uatomic_store(&ft->max_used_key_len, max_len, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

static
void calc_stats_node(const struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag, struct cds_ft_stats *stats, int level)
{
	unsigned long node_type = ft_node_type(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	node_stats->count++;
	node_stats->distribution[ft_meta_nr_child(metadata)]++;
	stats->level[level].nr_internal_nodes++;
	stats->level[level].has_nodes = true;
}

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level);

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int key;

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			calc_stats_node(ft, child_node_flag, stats, level);
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			calc_stats_node_recursive(ft, child_node_flag, stats, level + 1);
		} else if (ft_node_compressed(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);
			int j;

			stats->level[level].nr_internal_nodes++;
			stats->level[level].nr_compressed_nodes++;
			stats->level[level].has_nodes = true;
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			for (j = 1; j < cn->len; j++) {
				stats->level[level + j].nr_internal_nodes++;
				stats->level[level + j].nr_compressed_nodes++;
				stats->level[level + j].has_nodes = true;
			}
			if (cn->child &&
			    !ft_node_external(cn->child))
				calc_stats_node_recursive(ft, cn->child, stats, level + cn->len);
			else if (cn->child) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = (struct cds_ft_node *) ft_node_ptr(cn->child);
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level + cn->len].nr_external_nodes++;
					else
						stats->level[level + cn->len].nr_duplicate_external_nodes++;
					stats->level[level + cn->len].has_nodes = true;
				}
			}
		} else {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child_node_flag);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[level].nr_external_nodes++;
				else
					stats->level[level].nr_duplicate_external_nodes++;
				stats->level[level].has_nodes = true;
			}
		}
	}
}

static
void do_show_stats(const struct cds_ft *ft, FILE *out, const struct cds_ft_stats *stats)
{
	int level;

	fprintf(out, "Fractal Trie (%p) Statistics\n", ft);
	fprintf(out, "---------------------------------------------------\n");
	for (level = 0; level < FT_MAX_DEPTH; level++) {
		const struct cds_ft_stats_level *stats_level = &stats->level[level];
		unsigned long type;

		if (!stats_level->has_nodes)
			break;
		fprintf(out, "Level: %d\n", level);
		if (stats_level->nr_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "External nodes: %" PRIu64 "\n", stats_level->nr_external_nodes);
		}
		if (stats_level->nr_metadata_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Metadata external nodes: %" PRIu64 "\n", stats_level->nr_metadata_external_nodes);
		}
		if (stats_level->nr_duplicate_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Duplicate external nodes: %" PRIu64 "\n", stats_level->nr_duplicate_external_nodes);
		}
		if (stats_level->nr_internal_nodes) {
			print_indent(out, 1);
			fprintf(out, "Internal nodes: %" PRIu64 "\n", stats_level->nr_internal_nodes);
		}
		if (stats_level->nr_compressed_nodes) {
			print_indent(out, 1);
			fprintf(out, "Compressed nodes: %" PRIu64 "\n", stats_level->nr_compressed_nodes);
		}
		for (type = 0; type < FT_TYPE_MAX_NR; type++) {
			const struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[type];
			uint64_t nr_nodes = node_stats->count;

			if (nr_nodes) {
				unsigned int i;
				bool first = true;

				print_indent(out, 2);
				fprintf(out, "Internal node type %lu: %" PRIu64 " (", type, nr_nodes);
				for (i = 0; i <= 256; i++) {
					if (node_stats->distribution[i]) {
						fprintf(out, "%s%u: %" PRIu64,
							(!first ? ", " : ""), i, node_stats->distribution[i]);
						first = false;
					}
				}
				fprintf(out, ")\n");
			}
		}
	}
	fprintf(out, "---------------------------------------------------\n");
}

void cds_ft_show_stats(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_stats *stats;
	int level = 0;

	/*
	 * struct cds_ft_stats is several MiB: a per-level node-type
	 * distribution histogram (257 buckets) for each of FT_MAX_DEPTH
	 * levels.  That is far too large for the stack on threads with a
	 * small stack (e.g. musl's 128 KiB default), so heap-allocate it.
	 */
	stats = calloc(1, sizeof(*stats));
	if (!stats) {
		fprintf(out, "Fractal Trie (%p) Statistics: out of memory\n", ft);
		return;
	}

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	calc_stats_node(ft, node_flag, stats, level);
	calc_stats_node_recursive(ft, node_flag, stats, level + 1);
	do_show_stats(ft, out, stats);
	free(stats);
}
