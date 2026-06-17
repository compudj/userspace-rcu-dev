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
		(void) path;
		(void) group;
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
	if (cn_meta->nr_child > 1) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u > 1\n",
				depth, node_flag, cn_meta->nr_child);
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
	if ((cn_meta->nr_child == 1) != (ft_node_ptr(cn->child) != NULL)) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u does not match cn->child %p presence\n",
				depth, node_flag,
				cn_meta->nr_child, cn->child);
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
			if (metadata->nr_child > type->max_child) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p nr_child %u exceeds type %u max_child %u\n",
						depth, node_flag,
						metadata->nr_child, type_index,
						(unsigned int) type->max_child);
				return -1;
			}
		}
		/* Count external nodes attached to this node's metadata. */
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
		if (metadata->nr_child != counted_children) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p nr_child mismatch: "
					"stored %u, counted %u\n",
					depth, node_flag, metadata->nr_child,
					counted_children);
			return -1;
		}
		/*
		 * Pigeon bitmap consistency: pigeon nodes maintain a
		 * 256-bit live-slot bitmap (allocated alongside the node
		 * via ft_alloc_item with type->bitmap=true) that the
		 * directional / bitmap-scan readers consult.  The bitmap
		 * must agree slot-for-slot with the actual pointer array:
		 * bit i set iff node->data[i] holds a non-NULL pointer.
		 * ft_pigeon_node_get_nth reads node->data[n] directly
		 * (bitmap-independent), so cross-checking the two surfaces
		 * a desynchronised set / clear at the mutation site rather
		 * than letting it produce wrong directional results later.
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

					if (slot_set != bit_set) {
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
	uint8_t *path = NULL;
	int ret;

	if (ft_visited_init(&visited)) {
		if (out)
			fprintf(out, "ft_verify: visited-set allocation failed\n");
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	}
	/*
	 * End-to-end path/key consistency (invariant 10) requires an
	 * addressable copy of the inserted key at a group-known offset
	 * on each leaf, which the library no longer tracks (the user
	 * provides @key_offset per-call via cds_ft_speculative_lookup_key).
	 * Pass path = NULL so all path-tracking writes / compares
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

/*
 * cds_ft_compact internals.
 *
 * Relocate every internal node of a trie into fresh, densely-packed
 * allocations in DFS descent order, so the node arenas defragment: the old
 * ranges drain as relocated nodes are freed, and the allocator reclaims the
 * ones that empty (see cds_ft_do_free_item).  The relative layout order is
 * not a measurable performance lever (DFS, BFS+DFS and density-DFS all tie);
 * the win is recovering locality lost to churn/graft, so we use plain DFS.
 *
 * Concurrency: the caller's writer exclusion keeps the trie quiescent w.r.t.
 * other writers for the whole walk; concurrent RCU readers are fine.  Each
 * node is republished and its old copy RCU-freed (ft_node_recompact +
 * cds_ft_free_item), so a reader observes the old or the new node, never a
 * freed one.  Compressed nodes are relocated too, via
 * ft_compact_relocate_compressed: a traditional compressed node through its
 * grandparent slot, a skip-compressed node recovered from its skip pointer
 * with ft_skip_to_compressed with no grandparent repoint (the slot holds a
 * skip pointer to the target, not to cn).  This covers a skip whose target is
 * an external leaf: cn is relocated, the leaf is not (leaves are
 * application-owned), and the descent ends at the loop's ft_node_external
 * check.  An internal skip target is then relocated through cn->child, where
 * ft_node_recompact's dual-pointer publish updates both cn->child and the skip
 * pointer.
 */
/* Relocate the internal node at *@holder into a fresh slot; RCU-free the old. */
static
void ft_compact_relocate_at(struct cds_ft *ft, struct cds_ft_inode_flag **holder)
{
	struct cds_ft_inode_flag *nf = *holder;
	unsigned int type_index = ft_node_type(nf);
	struct cds_ft_inode *node = ft_node_ptr(nf), *old_ret = NULL;
	struct cds_ft_metadata *meta = cds_ft_item_to_metadata(node);

	(void) ft_node_recompact(FT_RECOMPACT_RELOCATE, ft, type_index,
			&ft_types[type_index], node, meta, holder,
			0, NULL, NULL, &old_ret, holder == &ft->root, 0,
			false);
	/*
	 * The old node was just unpublished; concurrent readers may still
	 * hold it, so free it after a grace period.  Its range's nr_live
	 * decrements in the callback, so a fully-drained range self-reclaims.
	 * ALWAYS deferred, even on an exclusive trie: the step's walk keeps
	 * navigating relative to nodes it has just unpublished, and the
	 * exclusive-mode synchronous free threads the freelist link through
	 * the freed slot immediately.
	 */
	if (old_ret)
		cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old_ret));
}

/*
 * Relocate a compressed node @cn into a fresh slot, keeping the same length.
 * Because the length is unchanged, the skip pointer's encoded length still
 * matches the relocated node (cn2->len == skip_len, whether a reader resolves
 * the old or the new node via the skip child's back-pointer), and the
 * parent-pointer flip is exactly the transient that ft_skip_reanchor already
 * tolerates; the old node stays alive until its grace period.  A compressed
 * node never carries external_nodes (that would fork a path that must remain
 * skippable), so the only reference to redirect is its child's back-pointer.
 * The grandparent slot is repointed only for a traditional (non-skip)
 * compressed node -- a skip pointer addresses the target, not @cn, so it needs
 * no change.
 *
 * @gp_slot: grandparent slot holding the cn flag (traditional), or NULL (skip).
 * Returns the new compressed node (or @cn unchanged on allocation failure).
 */
static
struct cds_ft_compressed_node *ft_compact_relocate_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag **gp_slot)
{
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_metadata *cn2_meta;
	struct cds_ft_compressed_node *cn2;
	struct cds_ft_inode_flag *cn2_flag;
	uint8_t len = cn->len;

	cn2 = alloc_compressed_node(ft, len, &cn2_meta);
	if (!cn2)
		return cn;	/* OOM: leave in place (best-effort) */
	cn2->len = len;
	cn2->child = cn->child;
	memcpy(cn2->key_bytes, cn->key_bytes, len);
	cn2_meta->parent = cn_meta->parent;
	/*
	 * The relocated compressed node keeps the SAME slot in the SAME
	 * parent, so its parent-slot offset is identical.  Copy it on every
	 * build: the offset is no longer skip-specific (it backs the
	 * parent-pointer backtrack's O(1) slot recovery), and a position-
	 * based remove that climbs via ft_get_parent_slot would otherwise
	 * read a fresh-zeroed offset and resolve the wrong slot.
	 */
	cn2_meta->parent_slot_offset = cn_meta->parent_slot_offset;
	/* Same slot in the same parent => same incoming edge byte (up-walk source). */
	cn2_meta->incoming_byte = cn_meta->incoming_byte;
	cn2_meta->nr_child = cn_meta->nr_child;		/* == 1 for a compressed node */
	cn2_meta->external_nodes = NULL;		/* never set on a compressed node */
	ft_nr_keys_store(cn2_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
	cn2_flag = ft_compressed_node_flag(cn2);
	/* Redirect the child's back-reference (internal: parent; external: prev). */
	ft_set_parent(ft, cn2->child, cn2_flag, &cn2->child);
	if (gp_slot)
		rcu_assign_pointer(*gp_slot, cn2_flag);
	/* Always-deferred free: see ft_compact_relocate_at. */
	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(cn));
	cds_ft_free_item_deferred(ft, cn_meta);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
	return cn2;
}

/*
 * Relocate one ordinal cell into a fresh slot from the dedicated cell arena.
 * The active recompaction context routes the allocation into a private cell
 * range, so cells relocated in key-traversal order pack densely there -- the
 * dense ord-walk stride that makes ordered iteration a sequential scan rather
 * than a random pointer chase.  Returns the new cell, or @old unchanged on
 * allocation failure (best-effort: leave it in place).
 *
 * Atomicity reuses ft_ord_cell_swap for the two ordered-list edges
 * (pred->ord_next / succ->ord_prev flip together via the flip-latch, so a
 * bidirectional ordered reader never sees a half-relocated list; ord_cell_head
 * /tail follow).  The head's UPWARD reference (head->prev) is then re-pointed
 * with a PLAIN RCU store: an up-walk reader resolves the old or the new cell,
 * both carrying an IDENTICAL parent (compaction runs under writer exclusion, so
 * @old->parent is settled), and @old stays live until its grace period -- so no
 * flip is needed, and head->prev must never hold a flip-proxy (ft_resolve_head_
 * prev does not resolve one).  @old keeps its own links for parked ordered
 * readers and is RCU-freed (its general-arena range drains for reclaim).
 */
static
struct ft_ord_cell *ft_compact_relocate_cell(struct cds_ft *ft,
		struct ft_ord_cell *old)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct cds_ft_node *head = old->node;
	struct ft_ord_cell *new_cell;

	if (!meta)
		return old;		/* OOM: best-effort, leave in place */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	new_cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	new_cell->node = head;
	new_cell->parent = old->parent;
	/* Carry the head's edge byte across the relocation (up-walk key source). */
	meta->incoming_byte = cds_ft_item_to_metadata(old)->incoming_byte;
	/* ord_prev / ord_next are set from @old's neighbours by the swap. */
	ft_ord_cell_swap(ft, old, new_cell);
	rcu_assign_pointer(head->prev, ft_ord_cell_flag(new_cell));
	/* Always-deferred free: see ft_compact_relocate_at. */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old));
	return new_cell;
}

/*
 * Forward relocate-descent: walk from the root to the leaf for @key
 * (the user key), relocating every internal node on the path that has
 * not already been relocated this pass (recompact_private).  Mirrors the
 * lookup descent's ordinal mapping and skip/compressed advancement, but
 * tracks the holder at each step (which the read descent does not) so it
 * can republish.  A skip target is relocated through the compressed
 * node's cn->child slot, so ft_node_recompact republishes both cn->child
 * and the skip pointer.  Increments *@relocated per node moved.
 */
static
void ft_compact_descend(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, unsigned long *relocated)
{
	const struct cds_ft_key_map *km = &ft->group->key_map;
	struct cds_ft_inode_flag **holder = &ft->root;
	size_t depth = 0;

	for (;;) {
		struct cds_ft_inode_flag *nf = rcu_dereference(*holder);
		struct cds_ft_inode_flag **child_slot = NULL;
		struct cds_ft_inode_flag *raw;
		uint8_t ord;

		if (ft_node_external(nf))
			return;		/* reached a leaf */
		if (!cds_ft_metadata_in_recompact_private(
				cds_ft_item_to_metadata(ft_node_ptr(nf)))) {
			ft_compact_relocate_at(ft, holder);
			(*relocated)++;
			nf = rcu_dereference(*holder);	/* the relocated node */
		}
		if (depth >= key_len)
			return;		/* consumed the whole key */
		ord = key_to_ordinal(key[depth], km);
		raw = ft_node_get_nth_skip(nf, &child_slot, ord, FT_PF_NONE);
		if (!raw)
			return;		/* child absent (e.g. concurrent removal) */
		depth++;		/* child-index byte (matches iter_key = *key++) */
		if (ft_node_skip_compressed(raw)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, raw);

			depth += ft_skip_len(raw);
			/*
			 * Relocate the compressed node carrying the skip.
			 * ft_skip_to_compressed recovers it through the child's
			 * back-pointer, so a skip whose target is an external leaf is
			 * handled too (its parent is cell-indirect; compaction's
			 * writer exclusion keeps the back-pointer settled).  The
			 * grandparent slot is a skip pointer addressing the target,
			 * not cn, so it needs no repoint (NULL).  Continue at
			 * cn->child: an internal target is relocated next iteration,
			 * while an external leaf (not itself relocatable) ends the
			 * descent at the loop's ft_node_external check.
			 */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, NULL);
				(*relocated)++;
			}
			holder = &cn->child;
		} else if (ft_node_compressed(raw)) {
			struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(raw);

			depth += cn->len;
			/* Traditional: the grandparent slot (child_slot) holds the cn flag. */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, child_slot);
				(*relocated)++;
			}
			holder = &cn->child;
		} else {
			holder = child_slot;	/* plain internal or external child */
		}
	}
}

/* Default per-step relocation budget when cds_ft_compact_step(batch == 0). */
#define FT_COMPACT_BATCH_DEFAULT	64

/*
 * Resumable compaction state.  Heap-allocated by cds_ft_compact_begin so the
 * caller treats it as opaque.  The navigation iterator is left in its default
 * CACHED mode: within a step's read-lock window it advances incrementally on
 * its cached path (the relocated-away nodes it navigates stay alive until the
 * read-unlock, and relocation preserves key order), and it retains the cursor
 * key itself.  Between steps the path is invalidated, so the iterator's key is
 * the only resume token; the private-range context persists, merged at end.
 */
struct cds_ft_compact_state {
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_recompact_alloc_ctx ctx;
	bool started;
	bool done;
};

struct cds_ft_compact_state *cds_ft_compact_begin(struct cds_ft *ft)
{
	struct cds_ft_compact_state *st;

	if (caa_unlikely(ft->active_compact != NULL)) {
		/*
		 * A compaction is already in flight on this trie (a previous
		 * one was never ended, or two are being started).  Programmer
		 * error: assert in debug, and refuse in release rather than
		 * abandon the in-flight one.
		 */
		assert(!"cds_ft_compact_begin: a compaction is already in progress on this trie");
		return NULL;
	}
	st = calloc(1, sizeof(*st));
	if (!st)
		return NULL;
	if (cds_ft_iter_create(ft, &st->iter) != CDS_FT_STATUS_OK) {
		free(st);
		return NULL;
	}
	ft_recompact_alloc_init(&st->ctx);
	st->ft = ft;
	ft->active_compact = st;
	return st;
}

bool cds_ft_compact_step(struct cds_ft_compact_state *st, size_t batch)
{
	struct cds_ft *ft = st->ft;
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	unsigned long relocated = 0;

	if (st->done)
		return false;
	if (batch == 0)
		batch = FT_COMPACT_BATCH_DEFAULT;

	/*
	 * Route this step's relocations into private ranges, and hold the
	 * RCU read lock for the whole batch: it keeps the iterator's reads
	 * and our descents safe, and defers our own call_rcu node frees until
	 * the read-unlock between steps (where the drained ranges reclaim and
	 * concurrent mutations get their window).
	 */
	ft_recompact_alloc_set_active(&st->ctx);
	flavor->read_lock();
	while (relocated < batch) {
		uint8_t key[FT_MAX_KEY_LEN];
		size_t key_len;
		enum cds_ft_status s;

		/*
		 * Cached iter: lookup_gt advances incrementally on the cached
		 * path within this read-lock window.  The first lookup of each
		 * batch re-descends the current structure (the path was
		 * invalidated at the previous read-unlock) from the iterator's
		 * retained key.
		 */
		s = st->started ? cds_ft_lookup_gt(ft, st->iter)
				: cds_ft_lookup_first(ft, st->iter);
		st->started = true;
		if (s != CDS_FT_STATUS_OK) {	/* NOT_FOUND or error: finished */
			st->done = true;
			break;
		}
		if (cds_ft_iter_get_key(st->iter, key, sizeof(key),
				&key_len) != CDS_FT_STATUS_OK) {
			st->done = true;
			break;
		}
		ft_compact_descend(ft, key, key_len, &relocated);
		/*
		 * Relocate this key's cell into a dense private cell range, in the
		 * same key order the iterator visits -- so the ordered cell list
		 * becomes a near-sequential scan.  iter->node is the chain head;
		 * its cell is head->prev.  Skip a cell already moved this pass
		 * (its range is recompact_private), mirroring the node descent, so
		 * the pass is idempotent and re-visits do not re-allocate.
		 */
		if (ft->group->ordered_list_set && st->iter->node) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(st->iter->node->prev));

			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata(cell))) {
				ft_compact_relocate_cell(ft, cell);
				relocated++;
			}
		}
	}
	/*
	 * Drop the cached path before releasing the read lock: the nodes it
	 * references become eligible for the grace-period free once unlocked.
	 * Bind (not just invalidate) so the iterator's key is materialized into
	 * its own buffer -- on a reference-keycopy / ordinal-cell group the live
	 * key is a leaf reference that does NOT survive the unlock, and the next
	 * step re-descends from that key.
	 */
	cds_ft_iter_bind_key(st->iter);
	flavor->read_unlock();
	ft_recompact_alloc_set_active(NULL);
	return !st->done;
}

void cds_ft_compact_end(struct cds_ft_compact_state *st)
{
	st->ft->active_compact = NULL;
	ft_recompact_alloc_merge(&st->ctx);
	cds_ft_iter_destroy(st->iter);
	free(st);
}

