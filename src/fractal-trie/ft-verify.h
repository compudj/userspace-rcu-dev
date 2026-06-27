// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-verify.h
 *
 * Userspace RCU library - Fractal Trie: cds_ft_verify: visited-set + recursive structural invariant checks.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-verify.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Integrity verification.
 *
 * ft_verify_node_recursive: recursively verify structural invariants
 * starting at @node_flag (which may be internal or compressed).
 * Returns 0 on success, -1 on first detected error (with details
 * printed to @out).  Must be called with mutual exclusion wrt
 * updaters.
 *
 * Checks performed:
 * - nr_child matches the actual count of non-NULL child slots.
 * - nr_keys equals the sum of children's nr_keys plus the count
 *   of unique keys from external node chains attached to this node.
 * - Parent pointers of children point back to the correct parent.
 * - Compressed node invariants (len > 0, no external_nodes).
 *
 * @ft: the Fractal Trie (for group/flag access).
 * @out: file stream for diagnostic output (may be NULL to suppress).
 * @node_flag: tagged pointer to the node being verified.
 * @expected_parent: tagged pointer that the node's metadata->parent
 *                   should match (NULL for root).
 * @depth: current depth (used for diagnostics).
 * @out_nr_keys: output -- total nr_keys in the subtree rooted here
 *               (written on success for parent aggregation).
 */
/*
 * Visited-pointer set for cds_ft_verify subtree-uniqueness check.
 *
 * Linear-probing open-addressing hash table keyed by node allocation
 * address (low tag bits stripped via ft_node_ptr).  Only used while a
 * single verify walk is in progress; the entire table is freed at the
 * end of cds_ft_verify.  Catches accidental sharing of a subtree
 * between two parents (a rebase/recompact bug class) and detects
 * parent-pointer cycles before the upward adjacency walk in
 * ft_verify_node_compressed gets a chance to loop forever.
 */
struct ft_visited_set {
	void **slots;		/* NULL = empty bucket. */
	size_t cap;		/* Power of two. */
	size_t mask;		/* cap - 1. */
	size_t count;
};

static
size_t ft_visited_hash(void *p)
{
	/*
	 * Drop the low alignment bits (arena items are at least 16-byte
	 * aligned, so the low 4 bits are zero), then mix with the 64-bit
	 * golden-ratio multiplier.
	 */
	uintptr_t v = (uintptr_t) p >> 4;
	return (size_t) (v * 11400714819323198485ULL);
}

static
int ft_visited_init(struct ft_visited_set *vs)
{
	vs->cap = 64;
	vs->mask = vs->cap - 1;
	vs->count = 0;
	vs->slots = calloc(vs->cap, sizeof(void *));
	return vs->slots ? 0 : -1;
}

static
void ft_visited_destroy(struct ft_visited_set *vs)
{
	free(vs->slots);
	vs->slots = NULL;
}

static
int ft_visited_grow(struct ft_visited_set *vs)
{
	size_t new_cap = vs->cap * 2;
	size_t new_mask = new_cap - 1;
	void **new_slots = calloc(new_cap, sizeof(void *));
	size_t i;

	if (!new_slots)
		return -1;
	for (i = 0; i < vs->cap; i++) {
		void *key = vs->slots[i];
		size_t j;

		if (!key)
			continue;
		j = ft_visited_hash(key) & new_mask;
		while (new_slots[j] != NULL)
			j = (j + 1) & new_mask;
		new_slots[j] = key;
	}
	free(vs->slots);
	vs->slots = new_slots;
	vs->cap = new_cap;
	vs->mask = new_mask;
	return 0;
}

/*
 * Returns 1 if @key was newly inserted, 0 if @key was already present
 * (duplicate visit), -1 on allocation failure.  NULL keys are not
 * tracked (they are filtered out by callers anyway).
 */
static
int ft_visited_add(struct ft_visited_set *vs, void *key)
{
	size_t i;

	if (key == NULL)
		return 1;
	/* Keep load factor below 0.5 for fast linear probing. */
	if ((vs->count + 1) * 2 > vs->cap) {
		if (ft_visited_grow(vs))
			return -1;
	}
	i = ft_visited_hash(key) & vs->mask;
	while (vs->slots[i] != NULL) {
		if (vs->slots[i] == key)
			return 0;
		i = (i + 1) & vs->mask;
	}
	vs->slots[i] = key;
	vs->count++;
	return 1;
}

/*
 * Flip-proxy-at-rest check.  cds_ft_verify runs under writer exclusion
 * (FEATURE_FT_VERIFY_AT_MUTATION calls it after the writer's mutation
 * completes), and a urcu flip parks a type-7 proxy in a slot only
 * transiently -- between its park and its settle, inside a single mutator.
 * So once the writer is between operations, NO reachable slot may hold a
 * flip proxy: one found here is a fused flip that left a slot UNSETTLED
 * (the "frozen-stale" slot that dangles after its batch is reclaimed).
 * Reports the exact slot deterministically, single-threaded, with no
 * trace-window dependence -- the recommended way to localize a remove /
 * splice fusion bug.
 */
static
int ft_verify_no_proxy_at_rest(FILE *out, const char *what,
		struct cds_ft_inode_flag *val,
		struct cds_ft_inode_flag *node_flag, unsigned int depth)
{
	if (caa_unlikely(ft_node_flip_proxy(val))) {
		if (out)
			fprintf(out, "ft_verify: depth %u: %s at node %p holds a flip proxy AT REST (%p) -- a fused flip left this slot unsettled\n",
				depth, what, (void *) node_flag, (void *) val);
		return -1;
	}
	return 0;
}

/* Forward declaration so the per-kind helpers below can recurse. */
static
int ft_verify_node_recursive(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys);

/*
 * Verify the doubly-linked external-node duplicate chain anchored at
 * @head, owned by @owner_flag (the flagged pointer to the
 * internal/compressed node, or its slot's parent).
 *
 *   - Head's prev must equal @owner_flag (the parent flagged-pointer
 *     convention used by ft_metadata_set_external_nodes and by the
 *     slot-attached external publish in cds_ft_insert).
 *   - Each non-head node's prev must point to its predecessor.
 *   - No node may appear twice (cycle / aliasing across chains).  We
 *     reuse @visited so a node accidentally referenced from a second
 *     chain elsewhere in the trie is also caught.
 *   - @path is currently always NULL.  Historical end-to-end
 *     path/key consistency relied on a group-known stored-key
 *     offset, which is no longer tracked at group level -- callers
 *     now provide @key_offset per-call via
 *     cds_ft_speculative_lookup_key.
 *
 * Returns 0 on success, -1 on first violation.  No-op when @head is
 * NULL.
 */
/*
 * Speculative leaf-key correctness: when this trie reads leaf keys for result
 * capture (ft->speculative_key_offset_active), every external leaf's STORED key
 * must equal its STRUCTURAL position -- the ordinal path @path[0..@depth) to the
 * leaf.  The library only ever reads that app-owned field; it cannot rewrite it
 * across a re-keying move (graft / graft_swap / merge_at with src!=dst), so a
 * mis-stamped or un-restamped leaf would make a speculative lookup return the
 * wrong key.  Catching it here, in the verifier (and thus under
 * FEATURE_FT_VERIFY_AT_MUTATION after every mutation), turns that silent
 * corruption into a loud, located failure -- the verify-time check for the
 * staging-graft workflow where the app stamps each leaf with its destination
 * key.  EAGER tries (cds_ft_attr_set_speculative_keys false) never read the leaf
 * key, so they are exempt.
 */
static
int ft_verify_speculative_key(const struct cds_ft *ft, FILE *out,
		const struct cds_ft_node *node, const uint8_t *path,
		unsigned int depth)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *stored;
	size_t klen;
	uint8_t ord[FT_MAX_KEY_LEN];

	if (!ft->speculative_key_offset_active || !path)
		return 0;
	stored = (const uint8_t *) node + group->speculative_key_offset;
	if (group->key_len != CDS_FT_LEN_VARIABLE)
		klen = group->key_len;
	else if (group->key_len_offset_set)
		klen = *(const size_t *) ((const char *) node +
				group->key_len_offset);
	else
		klen = depth;	/* no length field: validate bytes only */
	if (klen != depth) {
		if (out)
			fprintf(out, "ft_verify: depth %u: leaf %p speculative key length %zu != structural depth %u (stale leaf key after a re-keying move? re-stamp it, or create the trie with cds_ft_attr_set_speculative_keys(false))\n",
				depth, (const void *) node, klen, depth);
		return -1;
	}
	ft_key_to_ordinals(ord, stored, depth, &group->key_map);
	if (depth && memcmp(ord, path, depth) != 0) {
		if (out)
			fprintf(out, "ft_verify: depth %u: leaf %p speculative key does not match its position (stale leaf key after a re-keying move? re-stamp it, or create the trie with cds_ft_attr_set_speculative_keys(false))\n",
				depth, (const void *) node);
		return -1;
	}
	return 0;
}

static
int ft_verify_external_chain(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		const uint8_t *path,
		struct cds_ft_inode_flag *owner_flag,
		struct cds_ft_node *head,
		unsigned int depth)
{
	struct cds_ft_node *node = head;
	struct cds_ft_node *prev = NULL;
	const struct cds_ft_group *group = ft->group;
	bool check_path = (path != NULL);

	/*
	 * Every external leaf reached at @depth represents a key of
	 * length @depth (NIL terminator at metadata depth, or full key
	 * at slot depth -- both produce the same external chain).  That
	 * length must respect the group's max_key_len bound.  When the
	 * group is configured with CDS_FT_MAX_LEN_UNLIMITED
	 * (max_key_len == SIZE_MAX) this is a no-op since @depth is at
	 * most FT_MAX_KEY_LEN.
	 */
	if (head && (size_t) depth > group->max_key_len) {
		if (out)
			fprintf(out, "ft_verify: depth %u: external chain head %p exceeds group max_key_len %zu\n",
				depth, head, group->max_key_len);
		return -1;
	}
	while (node) {
		void *expected_prev = (prev == NULL) ?
			(void *) owner_flag : (void *) prev;
		int added = ft_visited_add(visited, node);

		if (added < 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: visited-set allocation failed in external chain at %p\n",
					depth, node);
			return -1;
		}
		if (added == 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: external chain node %p reached twice (cycle or alias)\n",
					depth, node);
			return -1;
		}
		if (prev == NULL && ft->ordered_list) {
			/*
			 * Ordered-list head: prev is the head's cell (cell-tagged),
			 * whose ->parent is the owner and ->node is this head.
			 */
			struct ft_ord_cell *cell = ft_ord_cell_ptr(node->prev);

			if (ft_node_external((struct cds_ft_inode_flag *) node->prev) ||
			    (void *) cell->parent != (void *) owner_flag ||
			    cell->node != node) {
				if (out)
					fprintf(out, "ft_verify: depth %u: head %p cell %p {parent %p, node %p} != expected {owner %p, node %p}\n",
						depth, node, (void *) cell,
						(void *) (ft_node_external((struct cds_ft_inode_flag *) node->prev) ? NULL : cell->parent),
						(void *) (ft_node_external((struct cds_ft_inode_flag *) node->prev) ? NULL : cell->node),
						(void *) owner_flag, (void *) node);
				return -1;
			}
		} else if (prev == NULL) {
			/* List off: a head's prev is the owner (flagged parent) directly. */
			if ((void *) node->prev != (void *) owner_flag) {
				if (out)
					fprintf(out, "ft_verify: depth %u: head %p prev %p != owner %p\n",
						depth, node, node->prev, (void *) owner_flag);
				return -1;
			}
		} else if (node->prev != expected_prev) {
			if (out)
				fprintf(out, "ft_verify: depth %u: external chain node %p prev %p != predecessor %p\n",
					depth, node, node->prev, expected_prev);
			return -1;
		}
		(void) check_path;
		(void) group;
		/* Every leaf in the chain shares this position; validate each. */
		if (ft_verify_speculative_key(ft, out, node, path, depth))
			return -1;
		prev = node;
		node = ft_node_next(node);
	}
	return 0;
}

/*
 * If @slot_val is skip-encoded, verify the encoded slen equals
 * cn->len of the underlying compressed node.  Returns 0 when the
 * slot is not skip-encoded or the slen matches; -1 on mismatch
 * (with diagnostic to @out).  No-op on architectures without
 * FEATURE_FT_SKIP_COMPRESSED (ft_node_skip_compressed is constant
 * false and the body is dead-coded).
 *
 * Catches double-wrap of an already-skip-encoded child, stale skip
 * pointers left behind by a recompact that did not refresh the
 * encoded slen, and the parent_depth_span class of bug fixed by
 * ft_parent_depth_span match against skip-encoded child.
 */
static
int ft_verify_skip_encoding(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *slot_val,
		unsigned int depth)
{
	struct cds_ft_compressed_node *cn;
	unsigned int slen, cn_len;

	if (!ft_node_skip_compressed(slot_val))
		return 0;
	slen = ft_skip_len(slot_val);
	cn = ft_skip_to_compressed(ft, slot_val);
	cn_len = cn->len;
	if (slen != cn_len) {
		if (out)
			fprintf(out, "ft_verify: depth %u: skip-encoded slot %p slen %u != cn->len %u (cn %p)\n",
				depth, slot_val, slen, cn_len, cn);
		return -1;
	}
	return 0;
}

/*
 * Verify a compressed node's invariants (cn->len >= 1, no
 * external_nodes, nr_child <= 1, parent pointer matches), recurse
 * into its child, and check the stored nr_keys against the child's
 * subtree count plus any local end-of-path key.
 */
static
int ft_verify_node_compressed(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) cn);
	struct cds_ft_node *external_nodes = cn_meta->external_nodes;
	unsigned long child_nr_keys = 0;
	unsigned long local_keys = 0;
	unsigned long stored_nr_keys;

	/* Compressed path must have length >= 1. */
	if (cn->len < 1) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has len %u < 1\n",
				depth, node_flag, (unsigned int) cn->len);
		return -1;
	}
	/*
	 * key_bytes[] must fit in the allocated slot.  The arena
	 * allocation order encodes the slot size; subtract the fixed
	 * header (offsetof(..., key_bytes)) to get the capacity, then
	 * assert cn->len fits.  Catches a stale len byte after a
	 * size-class mismatch (e.g. a chain-compress that grew len
	 * without reallocating into a larger size class), which would
	 * otherwise silently overrun key_bytes[] on lookup.
	 */
	{
		size_t alloc_size = 1UL << cds_ft_item_order(cn);
		size_t header_size = offsetof(struct cds_ft_compressed_node,
				key_bytes);
		size_t key_bytes_capacity = alloc_size - header_size;

		if ((size_t) cn->len > key_bytes_capacity) {
			if (out)
				fprintf(out, "ft_verify: depth %u: compressed node %p len %u exceeds key_bytes capacity %zu (alloc size %zu)\n",
					depth, node_flag,
					(unsigned int) cn->len,
					key_bytes_capacity, alloc_size);
			return -1;
		}
	}
	/* Parent pointer check. */
	if (cn_meta->parent != expected_parent) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p parent mismatch: "
				"expected %p, got %p\n",
				depth, node_flag, expected_parent, cn_meta->parent);
		return -1;
	}
	/* nr_child must be 0 or 1. */
	if (ft_meta_nr_child(cn_meta) > 1) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u > 1\n",
				depth, node_flag, ft_meta_nr_child(cn_meta));
		return -1;
	}
	/*
	 * cn->child / nr_child bookkeeping must agree:
	 *   nr_child == 1 implies cn->child is non-NULL (the one child);
	 *   nr_child == 0 implies cn->child is NULL.
	 * A drift between the two is a publish/clear bug that the
	 * subtree-key recursion would not catch on its own -- the
	 * key-aggregation path simply skips a NULL cn->child and would
	 * accept a stored nr_child of 1 with cn->child = NULL as long
	 * as nr_keys also dropped to 0 in lockstep.
	 */
	if ((ft_meta_nr_child(cn_meta) == 1) != (ft_node_ptr(cn->child) != NULL)) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u does not match cn->child %p presence\n",
				depth, node_flag,
				ft_meta_nr_child(cn_meta), cn->child);
		return -1;
	}
	/* Compressed nodes must not carry external_nodes. */
	if (external_nodes) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has external_nodes %p (forbidden)\n",
				depth, node_flag, external_nodes);
		return -1;
	}
	/*
	 * Canonicalization: no two adjacent compressed nodes.  Check both
	 * directions of the adjacency at every visited compressed:
	 *
	 *  - ancestor-chain (works in both modes, including skip-compress):
	 *    walk metadata->parent upward; every consecutive compressed
	 *    ancestor is part of an adjacency run.  The chain walk is
	 *    necessary in skip-compress mode because a single skip pointer
	 *    can bypass two (or more) compresseds at once: the slot's
	 *    metadata-parent chase from the deepest underlying child only
	 *    surfaces the innermost compressed-being-skipped, so the
	 *    walker visits that one but not its compressed ancestor(s).
	 *    Walking the parent chain at the visited compressed re-exposes
	 *    every adjacency in the bypassed run.
	 *
	 *  - child-side (cheap and direct, primary in non-skip mode):
	 *    cn->child must not be a (raw or skip-encoded) compressed.  In
	 *    canonical post-fix tries cn->child is never skip-encoded; the
	 *    skip-compressed disjunct is defensive.
	 *
	 * Chain-compress is responsible for fusing adjacencies into a
	 * single compressed; a violation here means the canonicalization
	 * walk missed a case (and skip mode would silently double-wrap the
	 * grandparent's skip pointer).
	 */
	{
		struct cds_ft_inode_flag *child_in_chain = node_flag;
		struct cds_ft_inode_flag *anc = cn_meta->parent;
		bool adj_violation = false;

		while (anc && ft_node_compressed(anc)) {
			struct cds_ft_metadata *anc_meta =
				cds_ft_item_to_metadata(ft_node_ptr(anc));

			if (out)
				fprintf(out, "ft_verify: depth %u: compressed %p adjacent to compressed parent %p (no two adjacent compresseds)\n",
					depth, child_in_chain, anc);
			adj_violation = true;
			child_in_chain = anc;
			anc = anc_meta->parent;
		}
		if (adj_violation)
			return -1;
	}
	if (ft_node_skip_compressed(cn->child) ||
			(cn->child && ft_node_compressed(cn->child))) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has %s child %p (no two adjacent compresseds)\n",
				depth, node_flag,
				ft_node_skip_compressed(cn->child) ?
					"skip-encoded compressed" : "compressed",
				cn->child);
		return -1;
	}
	/*
	 * Parent-slot offset round-trip.  Each compressed node records a
	 * pointer-stride offset from its parent to the slot that holds the
	 * pointer to itself (skip-encoded under FEATURE_FT_SKIP_COMPRESSED,
	 * a raw compressed flag otherwise).  ft_publish_to_parent uses it to
	 * refresh the slot when cn->child is replaced, and the parent-pointer
	 * backtrack (e.g. position-based remove) uses it to recover the slot.
	 * Verify the offset still resolves to a slot whose contents encode
	 * this very compressed.  A stale offset left after a recompact /
	 * graft / relocate is surfaced at the mutation that introduced it
	 * rather than as a corrupted slot at lookup / remove time.  Checked
	 * on every build (the offset is no longer skip-specific).
	 *
	 * NULL slot means the offset was never set (offset == 0 with a
	 * non-NULL parent).  No round-trip to verify in that case.
	 * ft_get_parent_slot wants a non-const ft for the root case
	 * (parent == NULL); cast away const since we only read *slot.
	 */
	{
		struct cds_ft_inode_flag **skip_slot =
			ft_get_parent_slot(cn_meta, (struct cds_ft *) ft);

		if (skip_slot) {
			struct cds_ft_inode_flag *slot_val = *skip_slot;
			struct cds_ft_compressed_node *target_cn = NULL;

			if (ft_verify_no_proxy_at_rest(out, "compressed skip_slot",
					slot_val, node_flag, depth))
				return -1;
			if (ft_node_skip_compressed(slot_val))
				target_cn = ft_skip_to_compressed(ft, slot_val);
			else if (slot_val &&
				 ft_node_compressed(slot_val))
				target_cn = ft_compressed_node_ptr(slot_val);
			if (target_cn != cn) {
				if (out)
					fprintf(out, "ft_verify: depth %u: compressed %p parent_slot_offset round-trip mismatch: slot %p holds %p (resolves to cn %p, expected %p)\n",
						depth, node_flag, skip_slot,
						slot_val, target_cn, cn);
				return -1;
			}
		}
	}
	/*
	 * Path tracking: write the compressed key bytes into the path
	 * buffer at positions [depth..depth+cn->len-1].  Subsequent
	 * recursion / external-chain compares read these bytes back
	 * against leaf-stored keys.
	 */
	if (path)
		memcpy(path + depth, cn->key_bytes, cn->len);
	/* Recurse into the child. */
	if (ft_verify_no_proxy_at_rest(out, "compressed cn->child", cn->child,
			node_flag, depth))
		return -1;
	if (cn->child) {
		if (ft_node_external(cn->child)) {
			/* External leaf chain at end of compressed path. */
			if (ft_verify_external_chain(ft, out, visited, path,
					node_flag,
					(struct cds_ft_node *) ft_node_ptr(cn->child),
					depth + cn->len))
				return -1;
			local_keys = 1;	/* One unique key. */
		} else {
			/* Internal/compressed child. */
			if (ft_verify_node_recursive(ft, out, visited, path,
					cn->child,
					node_flag, depth + cn->len,
					&child_nr_keys))
				return -1;
		}
	}
	/* Verify nr_keys. */
	stored_nr_keys = ft_nr_keys_get(cn_meta);
	if (stored_nr_keys != child_nr_keys + local_keys) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_keys mismatch: "
				"stored %lu, computed %lu (children %lu + local %lu)\n",
				depth, node_flag, stored_nr_keys,
				child_nr_keys + local_keys,
				child_nr_keys, local_keys);
		return -1;
	}
	*out_nr_keys = stored_nr_keys;
	return 0;
}

static
int ft_verify_node_recursive(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys)
{
	/*
	 * Subtree-uniqueness / cycle check.  Every traversable node
	 * (compressed, internal) must be reached exactly once from
	 * the root.  A duplicate visit means either two
	 * parents share the same child subtree (rebase/recompact bug)
	 * or a parent-pointer cycle has been introduced -- bail out
	 * before recursing further so the upward parent walks in the
	 * adjacency check cannot loop forever.
	 */
	{
		void *node_addr = ft_node_ptr(node_flag);
		int added = ft_visited_add(visited, node_addr);
		struct cds_ft_metadata *m = cds_ft_item_to_metadata(node_addr);

		if (added < 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: visited-set allocation failed at node %p\n",
					depth, node_flag);
			return -1;
		}
		if (added == 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: node %p reached twice (shared subtree or parent-pointer cycle)\n",
					depth, node_flag);
			return -1;
		}
		/*
		 * alloc_index round-trip: cds_ft_metadata_to_item walks back
		 * from the metadata to the arena slot using m->alloc_index.
		 * It must land on this very node; a corrupted alloc_index
		 * would otherwise survive verify and only fault later inside
		 * the allocator on free or recompact.
		 */
		if (cds_ft_metadata_to_item(m) != node_addr) {
			if (out)
				fprintf(out, "ft_verify: depth %u: node %p alloc_index round-trip yields %p (mismatch)\n",
					depth, node_flag,
					cds_ft_metadata_to_item(m));
			return -1;
		}
	}
	if (ft_node_compressed(node_flag))
		return ft_verify_node_compressed(ft, out, visited, path,
			node_flag, expected_parent, depth, out_nr_keys);

	/* --- Internal node (popcount, pigeon) --- */
	{
		struct cds_ft_inode *node = ft_node_ptr(node_flag);
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);
		struct cds_ft_node *external_nodes = metadata->external_nodes;
		unsigned long total_child_keys = 0;
		unsigned long local_keys = 0;
		unsigned int counted_children = 0;
		unsigned int key;

		/* Parent pointer check (root has NULL parent). */
		if (metadata->parent != expected_parent) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p parent mismatch: "
					"expected %p, got %p\n",
					depth, node_flag, expected_parent,
					metadata->parent);
			return -1;
		}
		/*
		 * Slot-offset round-trip (non-root): the recorded
		 * parent_slot_offset must resolve, in the parent body, to the
		 * slot that holds this node.  Catches a stale/unset offset
		 * (e.g. a placement that passed a NULL slot to ft_set_parent)
		 * at the mutation that introduced it, rather than as a
		 * corrupted parent-pointer backtrack later.
		 */
		if (metadata->parent) {
			struct cds_ft_inode_flag **slot =
				ft_get_parent_slot(metadata,
						(struct cds_ft *) ft);

			if (!slot ||
			    ft_node_ptr(*slot) != ft_node_ptr(node_flag)) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p slot_offset round-trip mismatch: slot %p holds %p\n",
						depth, node_flag, (void *) slot,
						slot ? (void *) *slot : NULL);
				return -1;
			}
		}
#ifdef FT_IMMEDIATE_FREE
		/* Check parent target is not poisoned (freed). */
		if (metadata->parent) {
			unsigned char *p = (unsigned char *) ft_node_ptr(metadata->parent);
			if (*p == 0xfe) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p parent %p points to freed (poisoned) node\n",
						depth, node_flag, metadata->parent);
				return -1;
			}
		}
#endif
		/*
		 * Type / alloc_index sanity.  ft_node_type already asserts
		 * the tag bits decode within FT_TYPE_BITS, but it does not
		 * verify the entry is a real internal class, that the arena
		 * order matches the type's expected order, or that nr_child
		 * fits the type's capacity.  A corrupted tag/bitfield write
		 * would otherwise survive verify and only manifest later as
		 * a wrong-sized scan or a min_child assertion.
		 */
		{
			unsigned int type_index = ft_node_type(node_flag);
			const struct cds_ft_type *type = &ft_types[type_index];
			size_t actual_order = cds_ft_item_order(node);

			if (type->type_class != FT_POPCOUNT &&
			    type->type_class != FT_PIGEON) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p has non-internal type_class %d (type_index %u)\n",
						depth, node_flag,
						(int) type->type_class,
						type_index);
				return -1;
			}
			if (actual_order != type->order) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p alloc order %zu mismatches type %u expected order %u\n",
						depth, node_flag,
						actual_order, type_index,
						(unsigned int) type->order);
				return -1;
			}
			if (ft_meta_nr_child(metadata) > type->max_child) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p nr_child %u exceeds type %u max_child %u\n",
						depth, node_flag,
						ft_meta_nr_child(metadata), type_index,
						(unsigned int) type->max_child);
				return -1;
			}
		}
		/* Count external nodes attached to this node's metadata. */
		if (ft_verify_no_proxy_at_rest(out, "external_nodes",
				(struct cds_ft_inode_flag *) external_nodes,
				node_flag, depth))
			return -1;
		if (external_nodes) {
			if (ft_verify_external_chain(ft, out, visited, path,
					node_flag, external_nodes, depth))
				return -1;
			local_keys = 1;	/* One unique key position. */
		}
		/* Walk all 256 child slots. */
		for (key = 0; key < 256; key++) {
			struct cds_ft_inode_flag **slot = NULL;
			struct cds_ft_inode_flag *child_raw =
				ft_node_get_nth_skip(node_flag, &slot, (uint8_t) key, FT_PF_NONE);
			struct cds_ft_inode_flag *child;

			if (!child_raw)
				continue;
			if (ft_verify_no_proxy_at_rest(out, "child slot",
					child_raw, node_flag, depth))
				return -1;
			/*
			 * Raw slot may be skip-encoded; verify the encoded
			 * slen matches the underlying compressed's cn->len
			 * before resolving for the recursion.
			 */
			if (ft_verify_skip_encoding(ft, out, child_raw, depth + 1))
				return -1;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Slot-centric skip-slot invariant: a slot holding
			 * skip(cn) must be the very slot cn records as its
			 * skip_slot (the inverse of the cn-centric round-trip
			 * check above).  rec != slot names a dangling/aliased
			 * skip slot directly: a reparent left cn's skip_slot
			 * pointing elsewhere, or a placement never recorded it.
			 */
			if (ft_node_skip_compressed(child_raw) && slot) {
				struct cds_ft_compressed_node *scn =
					ft_skip_to_compressed(ft, child_raw);
				struct cds_ft_metadata *scnm =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) scn);
				struct cds_ft_inode_flag **rec =
					ft_get_parent_slot(scnm,
						(struct cds_ft *) ft);

				if (rec != slot) {
					if (out)
						fprintf(out, "ft_verify: depth %u: node %p key %u slot %p holds skip(cn %p) but cn->skip_slot names %p (dangling skip slot)\n",
							depth, node_flag, key,
							(void *) slot, (void *) scn,
							(void *) rec);
					return -1;
				}
			}
#endif
			child = ft_resolve_skip_compressed(ft, child_raw);
			counted_children++;
			/*
			 * Path tracking: this slot's byte at @depth is
			 * the one being consumed to reach @child.
			 */
			if (path)
				path[depth] = (uint8_t) key;
			if (ft_node_external(child)) {
				/* External leaf chain at this slot. */
				if (ft_verify_external_chain(ft, out, visited,
						path, node_flag,
						(struct cds_ft_node *) ft_node_ptr(child),
						depth + 1))
					return -1;
				total_child_keys += 1;
			} else {
				unsigned long sub_keys = 0;

				if (ft_verify_node_recursive(ft, out, visited,
						path, child,
						node_flag, depth + 1,
						&sub_keys))
					return -1;
				total_child_keys += sub_keys;
			}
		}
		/* Verify nr_child. */
		if (ft_meta_nr_child(metadata) != counted_children) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p nr_child mismatch: "
					"stored %u, counted %u\n",
					depth, node_flag, ft_meta_nr_child(metadata),
					counted_children);
			return -1;
		}
		/*
		 * Pigeon bitmap consistency: pigeon nodes maintain a
		 * 256-bit occupancy bitmap (allocated alongside the node
		 * via ft_alloc_item with type->bitmap=true) that the
		 * directional / bitmap-scan readers consult.  The bitmap is
		 * a HINT, with the pointer array as the sole truth: a
		 * non-NULL node->data[i] MUST have bit i set (else the
		 * directional scan would skip a live child), but a set bit
		 * over a NULL slot is allowed -- a sticky soft-delete hole
		 * the scan rescans past (ft_pigeon_node_get_direction) and
		 * a recompact rebuilds away.  So cross-check the weaker
		 * slot_set => bit_set, surfacing only a live child missing
		 * its bit (a desync that would lose directional results).
		 */
		{
			unsigned int t = ft_node_type(node_flag);
			const struct cds_ft_type *t_type = &ft_types[t];

			if (ft_type_is_pigeon(t_type->type_class)) {
				struct cds_ft_bitmap *bm =
					cds_ft_item_to_bitmap(node, t_type->order);
				unsigned int b;

				for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
					struct cds_ft_inode_flag *child =
						ft_pigeon_node_get_nth(NULL, node,
							NULL, (uint8_t) b,
							FT_PF_NONE);
					bool slot_set = ft_node_ptr(child) != NULL;
					bool bit_set = cds_test_bit(bm->bitmap, b);

					/* A live slot must have its bit set; a set
					 * bit over a NULL slot is a tolerated sticky
					 * soft-delete hole. */
					if (slot_set && !bit_set) {
						if (out)
							fprintf(out, "ft_verify: depth %u: pigeon node %p slot %u: data %s, bitmap bit %s\n",
								depth, node_flag, b,
								slot_set ? "set" : "NULL",
								bit_set ? "set" : "clear");
						return -1;
					}
				}
			}
		}
#ifdef FEATURE_FT_COMPRESS
		/*
		 * Canonicalization (skip-compressed mode, non-root): a
		 * single-child internal node with no external_nodes attached
		 * should have been replaced by a 1-byte compressed node -- in
		 * skip mode the compressed publishes as a skip-encoded
		 * pointer (zero read-side cost), strictly cheaper than the
		 * 1-child internal it stands in for.  The external_nodes
		 * carve-out is mandatory: compressed nodes cannot carry
		 * external_nodes, so an internal that hosts a NIL-key
		 * end-of-path and a single non-NIL branch must remain
		 * internal.
		 *
		 * The root is exempt: ft->root is read directly by the
		 * traversal entry, so the skip-encoded pointer's zero
		 * read-side cost has nowhere to attach (there is no parent
		 * slot to encode the slen into).  A 1-byte compressed at
		 * root costs the same CL as a 1-child internal, so the
		 * canonicalization policy is allowed to keep it internal.
		 *
		 * In non-skip mode, ft_build_ordinal_chain keeps a 1-byte
		 * compressed floor at len >= 2, so a 1-child internal at
		 * the head of a length-1 chain is canonical and must not
		 * trip this check; the runtime gate handles the distinction.
		 *
		 * Dual of the existing "no two adjacent compresseds" check.
		 */
		if (expected_parent != NULL &&
		    ft_group_skip_compressed(ft->group) &&
		    counted_children == 1 && !external_nodes) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p has 1 child and no external_nodes (should be a 1-byte compressed in skip mode)\n",
					depth, node_flag);
			return -1;
		}
#endif
		/* Verify nr_keys. */
		{
			unsigned long stored_nr_keys = ft_nr_keys_get(metadata);

			if (stored_nr_keys != total_child_keys + local_keys) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p nr_keys mismatch: "
						"stored %lu, computed %lu (children %lu + local %lu)\n",
						depth, node_flag, stored_nr_keys,
						total_child_keys + local_keys,
						total_child_keys, local_keys);
				return -1;
			}
			*out_nr_keys = stored_nr_keys;
		}
		return 0;
	}
}


/*
 * ft_verify_ord_cells: verify the ordinal-cell list against the trie.
 *
 * Walks the trie in key order via the relational descent and the cell list in
 * lockstep, asserting: the list visits exactly the trie's distinct-key heads
 * in the same order; each visited cell is the head's own cell (head->prev) and
 * cell->node points back at that head; the back-edge invariant
 * ord_next(c)->ord_prev == c holds; the minimum cell has ord_prev == NULL and
 * the cached ord_cell_head / ord_cell_tail equal the trie minimum / maximum.
 *
 * The oracle stays independent of the cell list (cache_valid cleared before
 * each step forces the full descent).  Runs under the caller's writer
 * exclusion.  Returns 0 on success, -1 on the first violation.
 */
static
int ft_verify_ord_cells(const struct cds_ft *cft, FILE *out)
{
	struct cds_ft *ft = (struct cds_ft *) cft;
	struct cds_ft_iter *iter;
	struct cds_ft_node *trie_head;
	struct ft_ord_cell *cell, *max_cell = NULL;
	int ret = 0;

	if (cds_ft_iter_create(ft, &iter) != CDS_FT_STATUS_OK) {
		if (out)
			fprintf(out, "ft_verify: ord-cell iter allocation failed\n");
		return -1;
	}
	iter->key_len = 0;
	iter->prefix_len = 0;
	iter->cache_valid = false;	/* force descent oracle */
	cds_ft_lookup_inequality_impl(ft, iter, FT_LOOKUP_GE,
			FT_LOOKUP_LIMIT_FIRST, false, false);
	trie_head = cds_ft_iter_node(iter);
	cell = ft->ord_cell_head;
	if (trie_head &&
	    ft_ord_cell_resolve_ord(&ft_ord_cell_ptr(trie_head->prev)->ord_prev)
		!= NULL) {
		if (out)
			fprintf(out, "ft_verify: ord-cell min head %p cell has ord_prev != NULL\n",
				(void *) trie_head);
		ret = -1;
		goto out;
	}
	while ((trie_head = cds_ft_iter_node(iter)) != NULL) {
		struct ft_ord_cell *head_cell =
			ft_ord_cell_ptr(rcu_dereference(trie_head->prev));
		struct ft_ord_cell *next_cell;

		if (cell != head_cell) {
			if (out)
				fprintf(out, "ft_verify: ord-cell order mismatch: list cell %p vs trie head %p cell %p\n",
					(void *) cell, (void *) trie_head,
					(void *) head_cell);
			ret = -1;
			goto out;
		}
		if (cell->node != trie_head) {
			if (out)
				fprintf(out, "ft_verify: ord-cell %p node %p != trie head %p\n",
					(void *) cell, (void *) cell->node,
					(void *) trie_head);
			ret = -1;
			goto out;
		}
		max_cell = cell;
		next_cell = ft_ord_cell_resolve_ord(&cell->ord_next);
		if (next_cell &&
		    ft_ord_cell_resolve_ord(&next_cell->ord_prev) != cell) {
			if (out)
				fprintf(out, "ft_verify: ord-cell back-edge broken at cell %p (ord_next %p whose ord_prev is %p)\n",
					(void *) cell, (void *) next_cell,
					(void *) ft_ord_cell_resolve_ord(&next_cell->ord_prev));
			ret = -1;
			goto out;
		}
		cell = next_cell;
		iter->cache_valid = false;	/* force descent oracle */
		cds_ft_lookup_inequality_impl(ft, iter, FT_LOOKUP_GT,
				FT_LOOKUP_LIMIT_NONE, false, false);
	}
	if (cell != NULL) {
		if (out)
			fprintf(out, "ft_verify: ord-cell list longer than trie (extra cell %p)\n",
				(void *) cell);
		ret = -1;
	}
	if (ret == 0 && ft->ord_cell_tail != max_cell) {
		if (out)
			fprintf(out, "ft_verify: ord-cell ord_cell_tail %p != trie maximum cell %p\n",
				(void *) ft->ord_cell_tail, (void *) max_cell);
		ret = -1;
	}
out:
	cds_ft_iter_destroy(iter);
	return ret;
}

/*
 * cds_ft_verify - Verify integrity of the entire Fractal Trie.
 *
 * Recursively walks every internal and compressed node starting
 * from the root, checking that nr_child, nr_keys, and parent
 * pointers are self-consistent.
 *
 * Must be called with mutual exclusion wrt updaters.
 *
 * @out: file stream for diagnostic output on failure (may be NULL
 *       to suppress output).
 *
 * Returns CDS_FT_STATUS_OK if the trie passes all checks, or
 * CDS_FT_STATUS_INTEGRITY_ERROR on integrity violation.
 */
enum cds_ft_status cds_ft_verify(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *root = ft->root;
	unsigned long root_nr_keys = 0;
	struct ft_visited_set visited;
	uint8_t path_buf[FT_MAX_KEY_LEN];
	/*
	 * Track the structural key down to each leaf ONLY when this trie reads
	 * leaf-stored keys (ft->speculative_key_offset_active): the external-chain
	 * walk then compares each leaf's stored speculative key against its
	 * position (ft_verify_speculative_key).  For an EAGER trie the leaf key is
	 * not consulted, so path tracking is left off and every path write/compare
	 * short-circuits, exactly as before this check existed.
	 */
	uint8_t *path = ft->speculative_key_offset_active ? path_buf : NULL;
	int ret;

	if (ft_verify_no_proxy_at_rest(out, "root", root, NULL, 0))
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	/*
	 * Root-is-internal invariant.  ft->root must ALWAYS tag a plain internal
	 * node -- never compressed / skip-compressed / external -- which the read
	 * path's hot descent relies on.  Every mutator that re-roots the trie
	 * (graft / graft_swap / detach / merge) materializes the new root through
	 * the build-invisible internal-root builders (ft_make_root_internal_glue /
	 * ft_build_extracted_root_glue), so no compressed root is ever published.
	 * Asserted here so any future mutator that violates it is caught at the
	 * next mutation point (and so the descent's defensive non-internal-root
	 * resolver can be retired once this is proven 0 across the bulk-op suite).
	 */
	if (!ft_node_internal(root)) {
		if (out)
			fprintf(out, "ft_verify: root %p is NOT internal (compressed=%d skip=%d external=%d)\n",
				(void *) root, (int) ft_node_compressed(root),
				(int) ft_node_skip_compressed(root),
				(int) ft_node_external(root));
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	}
	if (ft_visited_init(&visited)) {
		if (out)
			fprintf(out, "ft_verify: visited-set allocation failed\n");
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	}
	/*
	 * @path is non-NULL only for a speculative-key-active trie (above), in
	 * which case the leaf walk validates each stored key against its
	 * position; otherwise it is NULL and all path-tracking writes / compares
	 * short-circuit.
	 */
	ret = ft_verify_node_recursive(ft, out, &visited, path, root, NULL, 0,
			&root_nr_keys);
	ft_visited_destroy(&visited);
	if (ret)
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	if (ft->group->ordered_list_set && ft_verify_ord_cells(ft, out))
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	return CDS_FT_STATUS_OK;
}

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
/*
 * Hook called from CDS_FT_SCOPED_WRITER's scope-exit, before the
 * writer claim is released.  Sampled by the per-trie
 * @verify_at_mutation_period: the cds_ft_verify walk runs once every
 * @period mutations.  The counter is incremented and reset on the
 * boundary so it never exceeds @period - 1, avoiding any overflow /
 * cadence-drift issue on long-running workloads.  Period 0 disables
 * the walk entirely (only the increment-and-compare runs).  On
 * mismatch, abort with diagnostic.
 */
void ft_writer_scope_verify(struct cds_ft *ft)
{
	unsigned long period = ft->verify_at_mutation_period;

	if (period == 0)
		return;
	ft->verify_at_mutation_counter++;
	if (ft->verify_at_mutation_counter < period)
		return;
	ft->verify_at_mutation_counter = 0;

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "FT verify-at-mutation: invariant violation on ft=%p\n",
			(void *) ft);
		abort();
	}
}
#endif

