// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-mutation-node.h
 *
 * Userspace RCU library - Fractal Trie: node-structure WRITE operations -- the
 * high-level child-slot mutators built on the popcount / pigeon bitmap layout.
 * ft_node_set_nth / ft_node_replace_ptr and their per-class dispatchers
 * (ft_popcount_node_set_nth, ft_pigeon_node_set_nth, the *_replace_ptr pair),
 * plus ft_node_recompact which grows or shrinks a node to the next layout
 * class.  The read accessors and the per-class bitmap layout they all share --
 * including the per-class LOW-level set_nth -- live in ft-lookup-node.h.
 *
 * Split out of ft-lookup-node.h and #included after ft-mutation-helpers.h so
 * ft_set_parent_raw (the deferred parent back-pointer store) is already
 * defined: in ft-lookup-node.h this cluster sat ahead of that definition and
 * had to forward-declare it.
 *
 * Implementation unit: #included once into the fractal-trie.c translation unit
 * (preserves cross-module inlining).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-mutation-node.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Insert (or replace) child_node_flag at byte n within an FT_POPCOUNT
 * node.  Three outcome paths:
 *
 *   - is_init: caller-promised first set_nth on a freshly-allocated
 *     (unpublished) node; call the layout's helper directly with
 *     is_init=true to set up the bitmap and pointer array from
 *     scratch.
 *
 *   - non-init, key already present: in-place pointer replace
 *     (single-slot RCU-safe write).
 *
 *   - non-init, key not present, "no existing bit lies above n":
 *     safe-append.  Setting bit n does not shift any existing
 *     pointer's popcount-rank, so the new pointer can be written at
 *     the (current) end-of-array slot and the new bit then published.
 *     Reader sees either the old state, an "appendinflight" state
 *     that returns NULL (legitimate), or the fully-published new
 *     entry.  See the publish-stores below for the detailed ordering
 *     reasoning (release on the new pointer, relaxed on each bitmap
 *     publish; reader's address-dependency from bitmap-derived index
 *     to pointer load keeps the loads ordered on weakly-ordered
 *     architectures).
 *
 *   - otherwise: -ERANGE, forcing the caller to recompact.
 */
static
int ft_popcount_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr,
		bool is_init,
		bool defer_parent)
{
	assert(ft_type_is_popcount(type->type_class));

	/*
	 * Parent-first: wire the (fresh, not-yet-published) child's parent
	 * back-pointer before any in-place store below, so a reader that
	 * descends to it and walks back up never observes a NULL/stale parent.
	 * Skipped when @defer_parent: a recompact child-copy (new node is
	 * unpublished, reparented post-assembly) or a build-invisible cluster-
	 * leaf (mutator wires parents at publish).  skip_slot is still set by
	 * the wrapper's post-store ft_set_parent.
	 */
	if (!defer_parent)
		ft_set_parent_raw(ft, child_node_flag, node_flag);

	if (type->popcount_2l && type->max_child == 6) {
		/* Flat 5+3 layout (max_child=6 on 64-bit). */
		struct cds_ft_inode_flag **qp_pointers =
			ft_popcount_2l_pointers(node, type);
		unsigned int qp_hi = (unsigned int) n >> 3;
		unsigned int qp_lo = (unsigned int) n & 0x7U;
		uint32_t qp_root;
		uint64_t qp_bms;
		unsigned int qp_slot1, qp_p, qp_ptr_idx;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_root = *(const uint32_t *) &node->data[0];
		qp_bms  = *(const uint64_t *) &node->data[4];
		qp_slot1 = (unsigned int) __builtin_popcount(
				qp_root & ((1U << qp_hi) - 1U));
		qp_p = (qp_slot1 << 3) | qp_lo;

		if (qp_root >> qp_hi & 1U) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: safe-append within existing hi.
			 * Need: new lo is highest in slot, and no slot1' > slot1.
			 */
			{
				/* bits in current slot above qp_lo */
				unsigned int slot_byte_pos = qp_slot1 * 8;
				uint64_t in_byte_above = (qp_lo == 7) ? 0ULL :
					((qp_bms >> (slot_byte_pos + qp_lo + 1))
						& ((1ULL << (7 - qp_lo)) - 1ULL));
				/* bits in higher-slot bytes */
				uint64_t above_slot = (qp_slot1 == 7) ? 0ULL :
					(qp_bms >> ((qp_slot1 + 1) * 8));
				uint32_t root_above =
					(uint32_t) ((qp_root >> qp_hi) >> 1);
				if (in_byte_above != 0 || above_slot != 0
						|| root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
			if (qp_ptr_idx >= type->max_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
			uatomic_store((uint64_t *) &node->data[4],
				qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 8)),
				CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0)
			return -ERANGE;	/* Case 3. */
		qp_slot1 = (unsigned int) __builtin_popcount(qp_root);
		qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
		if (qp_ptr_idx >= type->max_child)
			return -ENOSPC;
		/*
		 * slot1 == popcount(root) since all set his are below qp_hi.
		 * The byte at slot1 position is guaranteed zero (alloc-zeroed
		 * or recompact-into; never written for an unpopulated slot).
		 */
		uatomic_store((uint64_t *) &node->data[4],
			qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 8)),
			CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store((uint32_t *) &node->data[0],
			qp_root | (1U << qp_hi), CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	if (type->popcount_2l && (type->max_child == 12
				|| type->max_child == 14
				|| type->max_child == 16)) {
		/* Flat 6+2 layout (max_child=14 on 64-bit, 12/16 on 32-bit). */
		struct cds_ft_inode_flag **qp_pointers =
			ft_popcount_2l_pointers(node, type);
		unsigned int qp_hi = (unsigned int) n >> 2;
		unsigned int qp_lo = (unsigned int) n & 0x3U;
		uint64_t qp_root, qp_bms;
		unsigned int qp_slot1, qp_p, qp_ptr_idx;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_root = *(const uint64_t *) &node->data[0];
		qp_bms  = *(const uint64_t *) &node->data[8];
		qp_slot1 = (unsigned int) __builtin_popcountll(
				qp_root & ((1ULL << qp_hi) - 1ULL));
		qp_p = (qp_slot1 << 2) | qp_lo;

		if ((qp_root >> qp_hi) & 1ULL) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: safe-append within existing hi.
			 * Need: new lo is highest in slot, no slot1' > slot1.
			 */
			{
				/* bits in current slot above qp_lo */
				unsigned int slot_nibble_pos = qp_slot1 * 4;
				uint64_t in_nibble_above = (qp_lo == 3) ? 0ULL :
					((qp_bms >> (slot_nibble_pos + qp_lo + 1))
						& ((1ULL << (3 - qp_lo)) - 1ULL));
				/* bits in higher-slot nibbles */
				uint64_t above_slot = (qp_slot1 == 13) ? 0ULL :
					(qp_bms >> ((qp_slot1 + 1) * 4));
				uint64_t root_above =
					(qp_root >> qp_hi) >> 1;
				if (in_nibble_above != 0 || above_slot != 0
						|| root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
			if (qp_ptr_idx >= type->max_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
			uatomic_store((uint64_t *) &node->data[8],
				qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 4)),
				CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0ULL)
			return -ERANGE;	/* Case 3. */
		qp_slot1 = (unsigned int) __builtin_popcountll(qp_root);
		qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
		if (qp_ptr_idx >= type->max_child)
			return -ENOSPC;
		/*
		 * slot1 == popcount(root) since all set his are below qp_hi.
		 * The nibble at slot1 position is guaranteed zero (alloc-
		 * zeroed or recompact-into; never written for an unpopulated
		 * slot).
		 */
		uatomic_store((uint64_t *) &node->data[8],
			qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 4)),
			CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store((uint64_t *) &node->data[0],
			qp_root | (1ULL << qp_hi), CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	if (type->popcount_2l) {
		struct cds_ft_inode_flag **qp_pointers;
		unsigned int qp_max_lc = type->max_child;
		unsigned int qp_hi, qp_lo, qp_slot1, qp_ptr_idx;
		uint16_t qp_root, qp_sub;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_pointers = ft_popcount_2l_pointers(node, type);
		qp_root = *ft_popcount_2l_root_bm_addr(node, qp_max_lc);
		qp_hi = (unsigned int) n >> 4;
		qp_lo = (unsigned int) n & 0xFU;
		qp_slot1 = (unsigned int) __builtin_popcount(
				qp_root & ((1U << qp_hi) - 1U));

		if ((qp_root >> qp_hi) & 1U) {
			qp_sub = *ft_popcount_2l_sub_bm_addr(node,
					qp_max_lc, qp_slot1);
			if ((qp_sub >> qp_lo) & 1U) {
				/* Case 1: key already present, in-place replace. */
				uint64_t subs_below = 0;
				unsigned int k;

				for (k = 0; k < qp_slot1; k++)
					subs_below += (uint64_t)
						__builtin_popcount(
							*ft_popcount_2l_sub_bm_addr(
								node, qp_max_lc, k));
				subs_below += (uint64_t) __builtin_popcount(
						(unsigned int) qp_sub
						& ((1U << qp_lo) - 1U));
				qp_ptr_idx = (unsigned int) subs_below;
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: try safe-append within existing hi.
			 * Need: no bit above lo in sub_bm[slot1], AND no bit
			 * above hi in root_bm (so sub_bm[k > slot1] is all 0).
			 */
			{
				uint16_t in_word_above = qp_lo == 15 ? 0
					: (uint16_t) (qp_sub >> (qp_lo + 1));
				uint16_t root_above = (uint16_t)
					((qp_root >> qp_hi) >> 1);

				if (in_word_above != 0 || root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int)
				ft_popcount_2l_node_get_nr_child(
					type, node);
			if (qp_ptr_idx >= type->max_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx],
					child_node_flag);
			uatomic_store(ft_popcount_2l_sub_bm_addr(node,
					qp_max_lc, qp_slot1),
					(uint16_t) (qp_sub | (1U << qp_lo)),
					CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Try safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0)
			return -ERANGE;	/* Case 3. */
		/*
		 * slot1 == popcount(root_bm) since all currently-set hi
		 * positions are below qp_hi.  This is the first unused
		 * sub_bm slot, guaranteed zero from alloc / recompact-into.
		 */
		qp_slot1 = (unsigned int) __builtin_popcount(qp_root);
		qp_ptr_idx = (unsigned int)
			ft_popcount_2l_node_get_nr_child(type, node);
		if (qp_ptr_idx >= type->max_child)
			return -ENOSPC;
		uatomic_store(ft_popcount_2l_sub_bm_addr(node,
				qp_max_lc, qp_slot1),
				(uint16_t) (1U << qp_lo), CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store(ft_popcount_2l_root_bm_addr(node, qp_max_lc),
				(uint16_t) (qp_root | (1U << qp_hi)),
				CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	assert(type->popcount_1l);
	if (is_init) {
		int ret;
		ret = ft_popcount_1l_node_set_nth(type, node,
				metadata, n, child_node_flag, true);
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return ret;
	}
	{
		uint64_t *bm = (uint64_t *) &node->data[0];
		struct cds_ft_inode_flag **bp_pointers =
			(struct cds_ft_inode_flag **)
				((uint8_t *) node + 32);
		unsigned int word_idx = (unsigned int) n >> 6;
		unsigned int bit_idx = (unsigned int) n & 63U;
		uint64_t word = bm[word_idx];
		uint64_t bit = 1ULL << bit_idx;
		unsigned int ptr_idx = 0;
		unsigned int k;

		if (word & bit) {
			/* Case 1: in-place pointer replace. */
			for (k = 0; k < word_idx; k++)
				ptr_idx += (unsigned int)
					__builtin_popcountll(bm[k]);
			ptr_idx += (unsigned int) __builtin_popcountll(
					word & (bit - 1ULL));
			if (bp_pointers[ptr_idx]) {
				if (_replace_old_ptr)
					*_replace_old_ptr = true;
			} else {
				if (_replace_old_ptr)
					*_replace_old_ptr = false;
				metadata->nr_child++;
			}
			rcu_assign_pointer(bp_pointers[ptr_idx], child_node_flag);
			return 0;
		}

		/*
		 * Case 2: try safe-append.  Condition is "no existing
		 * bit at a position > n" -- evaluated as "no bit
		 * above bit_idx in word_idx, AND no bit set in any
		 * higher word".
		 */
		{
			bool safe_append;
			uint64_t in_word_above = bit_idx == 63 ? 0
				: (word >> (bit_idx + 1));

			safe_append = (in_word_above == 0);
			for (k = word_idx + 1; safe_append && k < 4; k++)
				safe_append = (bm[k] == 0);
			if (!safe_append)
				return -ERANGE;	/* Case 3. */
		}

		/* Case 2: do the append. */
		ptr_idx = (unsigned int) __builtin_popcountll(bm[0])
			+ (unsigned int) __builtin_popcountll(bm[1])
			+ (unsigned int) __builtin_popcountll(bm[2])
			+ (unsigned int) __builtin_popcountll(bm[3]);
		if (ptr_idx >= type->max_child)
			return -ENOSPC;
		/*
		 * Pointer store carries a release: the next-level child
		 * node's contents (initialized by the caller before this
		 * set_nth) must be visible to readers that follow the
		 * pointer (matching acquire on the scanner's pointer load).
		 *
		 * Bitmap bit set is relaxed: it is only a reachability flag
		 * for this slot.  If a reader observes the bit set but the
		 * pointer-store is not yet visible (still NULL), the lookup
		 * returns NULL -- a legitimate not-found result for an
		 * append that hasn't fully propagated.  No reader can
		 * compute ptr_idx == this new slot without also seeing the
		 * bit set, so the slot is invisible until the bit is.
		 */
		rcu_assign_pointer(bp_pointers[ptr_idx], child_node_flag);
		uatomic_store(&bm[word_idx], word | bit, CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
}

static
int ft_pigeon_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool defer_parent)
{
	struct cds_ft_inode_flag **ptr;
	bool replace_old_ptr = false;

	assert(ft_type_is_pigeon(type->type_class));
	/* Parent-first (see ft_popcount_node_set_nth). */
	if (!defer_parent)
		ft_set_parent_raw(ft, child_node_flag, node_flag);
	ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	if (*ptr)
		replace_old_ptr = true;
	rcu_assign_pointer(*ptr, child_node_flag);
	if (!replace_old_ptr) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
		metadata->nr_child++;
	}
	return 0;
}

/*
 * _ft_node_set_nth: set nth item within a node. Return an error
 * (negative error value) if it is already there.
 *
 * @is_init: caller guarantees this is the first set_nth on a
 * freshly-allocated unpublished (sub)node.  Used by recompact to
 * adopt the first inserted byte as values[0].  Ignored for
 * FT_PIGEON (dense 256-slot array, no reserved slot).
 *
 * This helper does NOT set @child_node_flag's parent pointer.  The
 * caller is responsible for ft_set_parent once @node is in a state
 * where linking it from @child's parent pointer is safe:
 *
 *   - Regular in-place insert: ft_node_set_nth does ft_set_parent
 *     right after this returns, since @node is already published
 *     and fully valid.
 *   - Recompact child-copy: new_node is UNPUBLISHED and being built
 *     slot by slot; linking children's parent pointers to new_node
 *     mid-build would expose transient nr_child < min_child to
 *     parent-pointer readers.  Recompact's post-copy reparent loop
 *     performs ft_set_parent after the whole new_node is assembled.
 */
static
int _ft_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init,
		bool defer_parent)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, NULL, is_init, defer_parent);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, defer_parent);
		break;
	case FT_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}
	return ret;
}

/*
 * FT_POPCOUNT class replace_ptr: publishes via rcu_assign_pointer on
 * the slot, with min_child gating on delete and nr_child accounting
 * via the bitmap-popcount helper.
 */
static
int ft_popcount_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr,
		struct ft_remove_pub *pub)
{
	assert(ft_type_is_popcount(type->type_class));
	assert(ft_popcount_node_get_nr_child(type, node) <= type->max_child);

	if (!newptr) {
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("popcount replace ptr: node %p\n", node);
	assert(*node_flag_ptr != NULL);
	/*
	 * Fusion armed: DEFER the forward store into @pub so ft_detach_node
	 * commits it in one flip with the dead head cell's unsplice.  A delete
	 * (NULL newptr) stores NULL and decrements nr_child in place
	 * (reader-invisible for navigation); an external promote (non-NULL
	 * newptr) replaces the child with the external chain head, so it wires
	 * the promoted external's back-pointer first (parent-first) and leaves
	 * nr_child unchanged.
	 */
	if (pub) {
		if (newptr)
			ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
		pub->slot = node_flag_ptr;
		pub->old_val = *node_flag_ptr;
		pub->new_val = newptr;
		pub->armed = true;
		if (!newptr)
			metadata->nr_child--;
		return 0;
	}
	/*
	 * Parent-first: wire the replacement's back-pointer before the
	 * forward publish (a reader descending here then walking back up must
	 * not see a NULL/stale parent).  Placed past the -EFBIG check: the
	 * recompaction path re-parents via its rebuilt copy itself, so setting
	 * it here would re-parent through a copy recompaction frees.  (NULL
	 * newptr == delete: ft_set_parent is a no-op.)
	 */
	ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr)
		metadata->nr_child--;
	dbg_printf("popcount replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) ft_popcount_node_get_nr_child(type, node),
		(unsigned int) metadata->nr_child,
		node, newptr);
	return 0;
}

static
int ft_pigeon_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr,
		struct ft_remove_pub *pub)
{
	assert(ft_type_is_pigeon(type->type_class));

	if (!newptr) {
		/* We should try recompacting the node */
		if (metadata->nr_child <= type->min_child)
			return -EFBIG;
	}
	dbg_printf("ft_pigeon_node_replace_ptr: replace ptr: %p by %p\n", *node_flag_ptr, newptr);
	assert(*node_flag_ptr != NULL);
	/*
	 * Fusion armed: DEFER the forward store into @pub (committed in one flip
	 * with the dead head cell's unsplice).  A DELETE (NULL newptr) records
	 * the bitmap bit-clear too -- a reader channel that must settle AFTER the
	 * flip -- and decrements nr_child in place.  An external PROMOTE (non-NULL
	 * newptr) wires the promoted external's back-pointer first (parent-first),
	 * leaves the slot occupied (no bitmap change) and nr_child unchanged.  See
	 * the popcount variant.
	 */
	if (pub) {
		if (newptr)
			ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
		pub->slot = node_flag_ptr;
		pub->old_val = *node_flag_ptr;
		pub->new_val = newptr;
		pub->armed = true;
		if (!newptr) {
			pub->pigeon_bitmap = cds_ft_item_to_bitmap(node, type->order);
			pub->pigeon_bit = n;
			metadata->nr_child--;
		}
		return 0;
	}
	/* Parent-first: wire the back-pointer before the forward publish,
	 * past the -EFBIG recompaction check (see popcount variant). */
	ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
		metadata->nr_child--;
	}
	return 0;
}

/*
 * _ft_node_replace_ptr: replace ptr item within a node. Return an error
 * (negative error value) if it is not found (-ENOENT).
 */
static
int _ft_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n, struct cds_ft_inode_flag *newptr,
		struct ft_remove_pub *pub)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_replace_ptr(ft, type, node, node_flag, metadata, node_flag_ptr, newptr, pub);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_replace_ptr(ft, type, node, node_flag, metadata, node_flag_ptr, n, newptr, pub);
		break;
	case FT_NULL:
		return -ENOENT;
	default:
		assert(0);
		return -EINVAL;
	}
	/* Parent back-pointer is wired inside the per-class body, ahead of
	 * the forward store. */
	return ret;
}

static
unsigned int find_nearest_type_index(unsigned int type_index,
		unsigned int nr_nodes, bool is_root)
{
	const struct cds_ft_type *type;

	assert(type_index != NODE_INDEX_NULL);
	if (nr_nodes == 0) {
		/*
		 * The root node is kept alive with 0 children (smallest
		 * popcount type).  All other nodes are pruned.
		 */
		return is_root ? 0 : NODE_INDEX_NULL;
	}
	for (;;) {
		type = &ft_types[type_index];
		if (nr_nodes < type->min_child)
			type_index--;
		else if (nr_nodes > type->max_child)
			type_index++;
		else
			break;
	}
	return type_index;
}

/*
 * ft_node_recompact_add: recompact a node, adding a new child.
 * Return 0 on success or negative error value on error.
 *
 * @cluster_leaf: when true, the target node sits at the lower boundary of an
 * as-yet-unpublished cluster (a cluster-leaf): its children point at live
 * nodes from the old structure, and the mutator wires every one of those
 * back-pointers itself at publish time.  Children are still copied into the
 * new node's slots, but the re-parent loop is skipped entirely.  See
 * ft_node_set_nth and the rcu-mutation build-invisible pattern.
 */
static
int ft_node_recompact(enum ft_recompact mode,
		struct cds_ft *ft,
		unsigned int old_type_index,
		const struct cds_ft_type *old_type,
		struct cds_ft_inode *old_node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **old_node_flag_ptr, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode_flag **nullify_node_flag_ptr,
		struct cds_ft_inode **old_node_ret,
		bool is_root,
		unsigned int node_depth __attribute__((unused)),
		bool cluster_leaf)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	int ret;
	/*
	 * Track whether new_node has received its first child via
	 * is_init=true.  Popcount nodes use a single init-done flag
	 * (no per-subnode state).
	 */
	bool new_init_done = false;

	/*
	 * Need to find nearest type index even for ADD_SAME, so that
	 * recompaction can promote/demote across tier boundaries
	 * (e.g. a popcount node that no longer fits its current tier).
	 */
	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child + 1, false);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			new_type_index = find_nearest_type_index(old_type_index,
				metadata->nr_child + 1, false);
			dbg_printf("Recompact for node with %u children\n",
				metadata->nr_child + 1);
		}
		break;
	case FT_RECOMPACT_DEL:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child - 1, is_root);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child - 1);
		break;
	case FT_RECOMPACT_RELOCATE:
		new_type_index = old_type_index;	/* same type, pure relocation */
		break;
	default:
		assert(0);
	}

	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		new_node = alloc_cds_ft_node(ft, new_type, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		new_node_flag = ft_node_flag(new_node, new_type_index);

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata) {
			new_metadata->parent = metadata->parent;
			/* The retyped node keeps its own incoming edge byte. */
			new_metadata->incoming_byte = metadata->incoming_byte;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			new_metadata->parent_slot_offset = metadata->parent_slot_offset;
#endif
			/*
			 * Recompact: new_metadata->parent is already inherited
			 * above, so the back-channel prev = new_node_flag is
			 * safe to publish here (up-walkers reach a parent-wired
			 * node).  Split into the Phase-1 metadata write + the
			 * Phase-2 prev publish for consistency with the other
			 * external-nodes attach sites.
			 */
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, metadata->external_nodes);
			ft_publish_external_nodes_prev(ft, new_node_flag,
				metadata->external_nodes);
			ft_nr_keys_store(new_metadata,
				ft_nr_keys_get(metadata), CMM_RELAXED);
		}
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != FT_RECOMPACT_ADD_NEXT || old_type->type_class != FT_PIGEON);

	/*
	 * A DEL must never prune the node to NODE_INDEX_NULL: the remove
	 * paths route a node emptying its last child through ft_detach_node
	 * (which unlinks the whole branch) instead of a DEL recompact, so
	 * @new_node below is non-NULL whenever the copy/parent-inherit tail
	 * dereferences it.  That invariant is enforced several call layers
	 * away -- catch a regression here, at the dereference site.
	 */
	assert(mode != FT_RECOMPACT_DEL || new_type_index != NODE_INDEX_NULL);

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

/*
 * Derive is_init for a set_nth call into the freshly-allocated
 * new_node.  Updates init-done state so the next call returns false.
 */
#define RECOMPACT_IS_INIT(byte_value) ({				\
	bool __is_init = false;						\
	if (new_type->type_class == FT_POPCOUNT) {			\
		__is_init = !new_init_done;				\
		new_init_done = true;					\
	} /* FT_PIGEON, FT_NULL: is_init irrelevant */			\
	__is_init;							\
})

	switch (old_type->type_class) {
	case FT_POPCOUNT:
	{
		uint8_t nr_child =
			ft_popcount_node_get_nr_child(old_type, old_node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_popcount_node_get_ith_pos(old_type, old_node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			if (new_type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(new_type,
						new_node, new_metadata, v, iter,
						RECOMPACT_IS_INIT(v));
			else if (new_type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(new_type,
						new_node, new_metadata, v, iter,
						RECOMPACT_IS_INIT(v));
			else
			ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
					new_metadata, v, iter,
					RECOMPACT_IS_INIT(v), true);
			assert(!ret);
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL ||
			mode == FT_RECOMPACT_RELOCATE);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			if (new_type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(new_type,
						new_node, new_metadata, (uint8_t)i, iter,
						RECOMPACT_IS_INIT((uint8_t)i));
			else if (new_type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(new_type,
						new_node, new_metadata, (uint8_t)i, iter,
						RECOMPACT_IS_INIT((uint8_t)i));
			else
			ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
					new_metadata, i, iter,
					RECOMPACT_IS_INIT((uint8_t)i), true);
			assert(!ret);
		}
		break;
	}
	default:
		assert(0);
		ret = -EINVAL;
		goto end;
	}
skip_copy:

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		/* add node */
		if (new_type->popcount_2l)
			ret = ft_popcount_2l_node_set_nth(new_type,
					new_node, new_metadata, n, child_node_flag,
					RECOMPACT_IS_INIT(n));
		else if (new_type->popcount_1l)
			ret = ft_popcount_1l_node_set_nth(new_type,
					new_node, new_metadata, n, child_node_flag,
					RECOMPACT_IS_INIT(n));
		else
		ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag,
				RECOMPACT_IS_INIT(n), true);
		assert(!ret);
	}

#undef RECOMPACT_IS_INIT

	/*
	 * Inherit the old node's parent pointer so upward walks
	 * (density propagation, ft_skip_to_compressed) can find
	 * the parent from the new node.
	 *
	 * If the recompacted node was the child of a compressed
	 * node published as a skip pointer, update the skip
	 * pointer BEFORE updating cn->child.  This ensures
	 * candidate readers (which follow the skip pointer)
	 * see the new child before exact/inequality readers
	 * (which follow cn->child) do.  The old child remains
	 * alive until after a grace period.
	 */
	if (old_node) {
		struct cds_ft_metadata *old_meta =
			cds_ft_item_to_metadata(old_node);
		struct cds_ft_inode_flag *old_parent = old_meta->parent;

		new_metadata->parent = old_parent;
		/*
		 * The recompacted node replaces the old node at the SAME slot
		 * in the SAME parent, so its parent-slot offset is identical.
		 * Inherit it on every build (the offset is no longer
		 * skip-specific -- it backs the parent-pointer backtrack's O(1)
		 * slot recovery for plain-internal nodes too).
		 */
		new_metadata->parent_slot_offset = old_meta->parent_slot_offset;

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (old_parent && ft_node_compressed(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			struct cds_ft_inode_flag **skip_slot =
				ft_get_parent_slot(cn_meta, ft);

			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot))
				rcu_assign_pointer(*skip_slot,
					ft_skip_compressed_flag(
						new_node_flag, cn->len));
		}
#endif
	}
	/*
	 * Reparent children to the new node.
	 *
	 * Children were copied value-for-value from the old node.
	 * Their parent pointers and skip_slot (for skip pointer
	 * children) still reference the old node, which will be
	 * freed after a grace period.  ft_set_parent updates both
	 * parent and skip_slot in one call.
	 *
	 * Skip this entirely for a cluster-leaf node: it sits at the
	 * lower boundary of an as-yet-unpublished cluster, its children
	 * point at live nodes, and the mutator wires every one of those
	 * back-pointers itself at publish time.  Re-parenting any of them
	 * here would expose the unpublished cluster from below.
	 */
	if (!cluster_leaf) {
		switch (new_type->type_class) {
		case FT_POPCOUNT:
		{
			uint8_t nc = ft_popcount_node_get_nr_child(new_type,
					new_node);
			unsigned int i;

			for (i = 0; i < nc; i++) {
				struct cds_ft_inode_flag *iter;
				struct cds_ft_inode_flag **slot = NULL;
				uint8_t v;

				ft_popcount_node_get_ith_pos(new_type,
						new_node, i, &v, &iter);
				if (!iter)
					continue;
				ft_node_get_nth_skip(new_node_flag,
						&slot, v, FT_PF_NONE);
				ft_set_parent(ft, iter, new_node_flag, slot);
			}
			break;
		}
		case FT_PIGEON:
		{
			unsigned int i;

			for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
				struct cds_ft_inode_flag *iter;
				struct cds_ft_inode_flag **slot = NULL;

				iter = ft_pigeon_node_get_ith_pos(new_type,
						new_node, i);
				if (!iter)
					continue;
				ft_node_get_nth_skip(new_node_flag,
						&slot, i, FT_PF_NONE);
				ft_set_parent(ft, iter, new_node_flag, slot);
			}
			break;
		}
		default:
			break;
		}
	}

	FT_TP(node_recompact, (const void *) *old_node_flag_ptr,
		(const void *) new_node_flag, (int) new_type_index);

	/*
	 * Return the new recompacted node through old_node_flag_ptr.  For the
	 * ADD/SAME/DEL mutators this is a local out-param, re-published with
	 * rcu_assign_pointer by the caller; but ft_compact_relocate_at passes
	 * the LIVE slot (&ft->root, a parent child slot, &cn->child) and never
	 * re-publishes, so this store IS the reader-visible publication of the
	 * relocated node: use a release store so the node-body and metadata
	 * stores above are ordered before it (a plain store would let a
	 * weakly-ordered architecture expose an unwired copy).  Free for the
	 * local-out-param callers.
	 */
	rcu_assign_pointer(*old_node_flag_ptr, new_node_flag);
	if (old_node && old_node_ret)
		*old_node_ret = old_node;

	ret = 0;
end:
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 *
 * @cluster_leaf: when true, the target node is a cluster-leaf -- the lower
 * boundary of an as-yet-unpublished cluster (rcu-mutation build-invisible
 * pattern).  Its children are live nodes also still reachable through the old
 * structure, so writing their back-pointers to this unpublished node would
 * expose the cluster from below and, on a later allocation failure, leave a
 * dangling back-pointer.  The forward slot is still set, but NO child's
 * back-pointer is written here (neither in-place nor through recompaction);
 * the mutator wires every child of the node itself at publish time, using the
 * final node flag it tracks across recompactions.  Callers building such a
 * node must pass true for ALL of its set_nth calls (no per-child exception).
 * false for the ordinary published-node case.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,
		unsigned int node_depth,
		bool cluster_leaf)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_set_nth for n=%u, node %p\n", (unsigned int) n, ft_node_ptr(*node_flag));

	node = ft_node_ptr(*node_flag);
	type_index = ft_node_type(*node_flag);
	type = &ft_types[type_index];
	/*
	 * Top-level entry: target node is always a published internal
	 * node (descent end-point or compressed-split destination),
	 * never a freshly-allocated unpublished node.  Pass is_init =
	 * false; fresh-init cases funnel here via -ENOSPC / -ERANGE to
	 * ft_node_recompact, which uses is_init internally.
	 */
	ret = _ft_node_set_nth(ft, type, node, *node_flag, metadata, n,
			child_node_flag, false, cluster_leaf);
	switch (ret) {
	case 0:
	{
		/*
		 * In-place insert succeeded on the published target node.
		 * Safe to link child -> target via parent pointer now:
		 * target is already fully valid to readers.
		 *
		 * Fetch the parent's child slot for both SKIP_X and plain
		 * COMPRESSED children: ft_set_parent's compressed branches
		 * use it to record parent_slot_offset, which chain-merge
		 * canonicalization (ft_detach_node) recovers via
		 * ft_get_parent_slot to publish the replacement at the same
		 * slot.  Without it, plain-COMPRESSED children (the path
		 * taken when CDS_FT_FLAG_SKIP_COMPRESSED is unset on the
		 * group) leave parent_slot_offset == 0 -- a latent gap that
		 * trips chain-merge with parent-cn=plain-COMPRESSED.
		 */
		struct cds_ft_inode_flag **slot_ptr = NULL;

		if (cluster_leaf)
			break;	/* child back-pointers set by mutator at publish */
		/*
		 * Fetch the parent's child slot for every non-external child
		 * (internal, compressed, or skip): ft_set_parent records the
		 * slot offset so the parent-pointer backtrack can recover it.
		 * (Externals carry no metadata / offset.)  Test skip FIRST: a
		 * SKIP_X flag carries its external child's low tag bits, so
		 * ft_node_external() would misclassify it.
		 */
		if (ft_node_skip_compressed(child_node_flag) ||
		    !ft_node_external(child_node_flag))
			ft_node_get_nth_skip(*node_flag, &slot_ptr, n, FT_PF_NONE);
		ft_set_parent(ft, child_node_flag, *node_flag, slot_ptr);
		break;
	}
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth, cluster_leaf);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth, cluster_leaf);
		break;
	}
	if (ret == 0)
		FT_TP(tree_edge_set, (const void *) ft,
			(const void *) *node_flag,
			(unsigned int) node_depth, (uint8_t) n,
			(const void *) child_node_flag);
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_replace_ptr(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag_ptr,		/* Pointer to location to nullify */
		struct cds_ft_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,			/* of parent */
		uint8_t n,
		struct cds_ft_inode_flag *newptr,
		bool is_root,
		unsigned int node_depth,
		struct ft_remove_pub *pub)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_replace_ptr for node %p, target ptr %p\n",
		ft_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ft_node_ptr(*parent_node_flag_ptr);
	type_index = ft_node_type(*parent_node_flag_ptr);
	type = &ft_types[type_index];
	ret = _ft_node_replace_ptr(ft, type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr, pub);
	if (ret == -EFBIG) {
		assert(!newptr);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, old_node_ret, is_root, node_depth,
				false);
	}
	if (ret == 0)
		FT_TP(tree_edge_set, (const void *) ft,
			(const void *) *parent_node_flag_ptr,
			(unsigned int) node_depth, (uint8_t) n,
			(const void *) newptr);
	return ret;
}
