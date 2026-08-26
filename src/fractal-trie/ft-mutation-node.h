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
/*
 * Apply the nr_child++ that an OCCUPANCY-ADDING set_nth owes, either in place or
 * by handing it to the caller's commit.
 *
 * @deferred_count NULL (every build-path caller): increment in place.  On a
 * build-invisible node that is correct by construction -- the count is part of
 * the node's initial image and publishes wholesale with it.
 *
 * @deferred_count non-NULL: the caller opted in to PUBLISHING the increment
 * itself, as an edge in the same commit as the structural publish
 * (ft_flip_txn_record_nr_child_inc, which carries the full rationale).  Only a
 * LIVE-node store needs that -- a build-invisible one (@defer_parent) is rolled
 * back wholesale with its fresh node -- so the deferral is keyed on the node
 * being live, and the flag reports back that the caller now owes the edge.
 * Never cleared here: the caller inits it false, and an -ERANGE/-ENOSPC retry
 * through ft_node_recompact builds its count on the FRESH copy, leaving it false.
 */
static inline
void ft_node_count_add(struct cds_ft_metadata *metadata, bool defer_parent,
		bool *deferred_count)
{
	if (deferred_count && !defer_parent) {
		*deferred_count = true;
		return;
	}
	ft_meta_nr_child_inc(metadata);
}

static
int ft_popcount_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr,
		bool is_init,
		bool defer_parent,
		bool *deferred_count)
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

		/* Recompact-on-insert: a new bitmap occupancy must not be set in
		 * place on a LIVE node.  Report -ERANGE so the wrapper routes
		 * through ft_node_recompact(ADD_SAME), exactly as a non-tail
		 * (Case 3) insert already does; a replace at an already-occupied
		 * slot (Case 1) stays in place (no bitmap change).  @defer_parent
		 * means the target is an unpublished build node (recompact
		 * child-copy / cluster-leaf) -- build-invisible, so keep the
		 * in-place store.  See FEATURE_FT_INSERT_IN_PLACE. */
		if (!ft_in_place_ok(ft) && !defer_parent &&
		    !((qp_root >> qp_hi & 1U) && ((qp_bms >> qp_p) & 1ULL)))
			return -ERANGE;
		if (qp_root >> qp_hi & 1U) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					/*
					 * FENCED ADD, in-place mirror (see the
					 * same check in ft_node_recompact): a
					 * LIVE-node RESERVE (@child_node_flag
					 * NULL) whose byte is no longer free.  A
					 * peer COMMITTED a child at @n between
					 * this op's descent -- which read the byte
					 * as absent, or the caller would not be
					 * reserving it -- and this store.  The
					 * blind store below would not replace a
					 * pointer, it would CLEAR that live child
					 * and orphan its whole subtree.  Bail
					 * -EAGAIN so the op re-descends and dives
					 * into the peer's child instead; the
					 * insert's pre-commit conflict path
					 * already routes -EAGAIN to
					 * restart_attempt.  Never fires under a
					 * single writer (the descent's read holds).
					 */
					if (!defer_parent && !child_node_flag)
						return -EAGAIN;
					/*
					 * CONTRACT (audited 2026-07-27): a
					 * REPLACE at an already-occupied slot
					 * stores straight into the LIVE pointer
					 * array -- no node lock, no txn
					 * record.  That is sound only on a
					 * build-invisible node, which is the
					 * only way this arm is reachable today:
					 * every internal caller is
					 * ft_node_recompact building @new_node
					 * (@defer_parent true), and every
					 * external caller passes a NULL child
					 * for an UNOCCUPIED slot -- insert
					 * guards it with `if (!old_node_flag)`
					 * precisely because a NULL store here
					 * would drop a live external before the
					 * commit.  Assert it so the convention
					 * is a checked contract: a future caller
					 * reaching this arm on a reachable node
					 * would get an untransacted live-node
					 * store, the exact class
					 * recompact-on-insert exists to remove.
					 */
					assert(defer_parent);
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					/*
					 * Recompact-on-insert (default): refilling a
					 * soft-deleted hole (bit sticky-set, slot NULL)
					 * bumps this LIVE node's nr_child -- report
					 * -ERANGE so the wrapper routes through
					 * ft_node_recompact(ADD_SAME) (the fresh copy
					 * drops the NULL hole and re-adds byte n).
					 * @defer_parent = build-invisible node: keep in
					 * place (also avoids recompact-within-recompact).
					 */
					if (!ft_in_place_ok(ft) && !defer_parent)
						return -ERANGE;
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					ft_node_count_add(metadata, defer_parent,
							deferred_count);
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
			ft_node_count_add(metadata, defer_parent,
					deferred_count);
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
		ft_node_count_add(metadata, defer_parent,
				deferred_count);
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

		/* Recompact-on-insert (see FEATURE_FT_INSERT_IN_PLACE): a new
		 * bitmap occupancy on a live node (!defer_parent) routes to
		 * ADD_SAME via -ERANGE; a replace at an occupied slot, and a
		 * build-into-unpublished-node insert (defer_parent), stay in
		 * place. */
		if (!ft_in_place_ok(ft) && !defer_parent &&
		    !(((qp_root >> qp_hi) & 1ULL) && ((qp_bms >> qp_p) & 1ULL)))
			return -ERANGE;
		if ((qp_root >> qp_hi) & 1ULL) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					/*
					 * FENCED ADD, in-place mirror (see the
					 * same check in ft_node_recompact): a
					 * LIVE-node RESERVE (@child_node_flag
					 * NULL) whose byte is no longer free.  A
					 * peer COMMITTED a child at @n between
					 * this op's descent -- which read the byte
					 * as absent, or the caller would not be
					 * reserving it -- and this store.  The
					 * blind store below would not replace a
					 * pointer, it would CLEAR that live child
					 * and orphan its whole subtree.  Bail
					 * -EAGAIN so the op re-descends and dives
					 * into the peer's child instead; the
					 * insert's pre-commit conflict path
					 * already routes -EAGAIN to
					 * restart_attempt.  Never fires under a
					 * single writer (the descent's read holds).
					 */
					if (!defer_parent && !child_node_flag)
						return -EAGAIN;
					/*
					 * CONTRACT (audited 2026-07-27): a
					 * REPLACE at an already-occupied slot
					 * stores straight into the LIVE pointer
					 * array -- no node lock, no txn
					 * record.  That is sound only on a
					 * build-invisible node, which is the
					 * only way this arm is reachable today:
					 * every internal caller is
					 * ft_node_recompact building @new_node
					 * (@defer_parent true), and every
					 * external caller passes a NULL child
					 * for an UNOCCUPIED slot -- insert
					 * guards it with `if (!old_node_flag)`
					 * precisely because a NULL store here
					 * would drop a live external before the
					 * commit.  Assert it so the convention
					 * is a checked contract: a future caller
					 * reaching this arm on a reachable node
					 * would get an untransacted live-node
					 * store, the exact class
					 * recompact-on-insert exists to remove.
					 */
					assert(defer_parent);
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					/*
					 * Recompact-on-insert (default): refilling a
					 * soft-deleted hole (bit sticky-set, slot NULL)
					 * bumps this LIVE node's nr_child -- report
					 * -ERANGE so the wrapper routes through
					 * ft_node_recompact(ADD_SAME) (the fresh copy
					 * drops the NULL hole and re-adds byte n).
					 * @defer_parent = build-invisible node: keep in
					 * place (also avoids recompact-within-recompact).
					 */
					if (!ft_in_place_ok(ft) && !defer_parent)
						return -ERANGE;
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					ft_node_count_add(metadata, defer_parent,
							deferred_count);
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
			ft_node_count_add(metadata, defer_parent,
					deferred_count);
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
		ft_node_count_add(metadata, defer_parent,
				deferred_count);
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

		/* Recompact-on-insert (see FEATURE_FT_INSERT_IN_PLACE): a new
		 * bitmap occupancy on a live node (!defer_parent) routes to
		 * ADD_SAME via -ERANGE; a replace at an occupied slot, and a
		 * build-into-unpublished-node insert (defer_parent), stay in
		 * place. */
		if (!ft_in_place_ok(ft) && !defer_parent) {
			bool qp_present = false;

			if ((qp_root >> qp_hi) & 1U) {
				uint16_t qp_sub0 = *ft_popcount_2l_sub_bm_addr(
						node, qp_max_lc, qp_slot1);
				qp_present = ((qp_sub0 >> qp_lo) & 1U) != 0;
			}
			if (!qp_present)
				return -ERANGE;
		}

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
					/*
					 * FENCED ADD, in-place mirror (see the
					 * same check in ft_node_recompact): a
					 * LIVE-node RESERVE (@child_node_flag
					 * NULL) whose byte is no longer free.  A
					 * peer COMMITTED a child at @n between
					 * this op's descent -- which read the byte
					 * as absent, or the caller would not be
					 * reserving it -- and this store.  The
					 * blind store below would not replace a
					 * pointer, it would CLEAR that live child
					 * and orphan its whole subtree.  Bail
					 * -EAGAIN so the op re-descends and dives
					 * into the peer's child instead; the
					 * insert's pre-commit conflict path
					 * already routes -EAGAIN to
					 * restart_attempt.  Never fires under a
					 * single writer (the descent's read holds).
					 */
					if (!defer_parent && !child_node_flag)
						return -EAGAIN;
					/*
					 * CONTRACT (audited 2026-07-27): a
					 * REPLACE at an already-occupied slot
					 * stores straight into the LIVE pointer
					 * array -- no node lock, no txn
					 * record.  That is sound only on a
					 * build-invisible node, which is the
					 * only way this arm is reachable today:
					 * every internal caller is
					 * ft_node_recompact building @new_node
					 * (@defer_parent true), and every
					 * external caller passes a NULL child
					 * for an UNOCCUPIED slot -- insert
					 * guards it with `if (!old_node_flag)`
					 * precisely because a NULL store here
					 * would drop a live external before the
					 * commit.  Assert it so the convention
					 * is a checked contract: a future caller
					 * reaching this arm on a reachable node
					 * would get an untransacted live-node
					 * store, the exact class
					 * recompact-on-insert exists to remove.
					 */
					assert(defer_parent);
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					/*
					 * Recompact-on-insert (default): refilling a
					 * soft-deleted hole (bit sticky-set, slot NULL)
					 * bumps this LIVE node's nr_child -- report
					 * -ERANGE so the wrapper routes through
					 * ft_node_recompact(ADD_SAME) (the fresh copy
					 * drops the NULL hole and re-adds byte n).
					 * @defer_parent = build-invisible node: keep in
					 * place (also avoids recompact-within-recompact).
					 */
					if (!ft_in_place_ok(ft) && !defer_parent)
						return -ERANGE;
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					ft_node_count_add(metadata, defer_parent,
							deferred_count);
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
			ft_node_count_add(metadata, defer_parent,
					deferred_count);
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
		ft_node_count_add(metadata, defer_parent,
				deferred_count);
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

		/* Recompact-on-insert (see FEATURE_FT_INSERT_IN_PLACE): a new
		 * bitmap occupancy on a live node (!defer_parent) routes to
		 * ADD_SAME via -ERANGE; a replace at an occupied slot, and a
		 * build-into-unpublished-node insert (defer_parent), stay in
		 * place. */
		if (!ft_in_place_ok(ft) && !defer_parent && !(word & bit))
			return -ERANGE;
		if (word & bit) {
			/* Case 1: in-place pointer replace. */
			for (k = 0; k < word_idx; k++)
				ptr_idx += (unsigned int)
					__builtin_popcountll(bm[k]);
			ptr_idx += (unsigned int) __builtin_popcountll(
					word & (bit - 1ULL));
			if (bp_pointers[ptr_idx]) {
				/*
				 * FENCED ADD, in-place mirror: a LIVE-node RESERVE
				 * (NULL child) whose byte a peer filled between this
				 * op's descent and this store -- the blind store would
				 * CLEAR the peer's live child.  Bail -EAGAIN to
				 * re-descend.  See the long note on the 2L arms.
				 */
				if (!defer_parent && !child_node_flag)
					return -EAGAIN;
				/* Same checked contract as the other Case-1 arms: an
				 * occupied-slot replace stores into the LIVE pointer array
				 * with no lock and no txn, so it is sound only on a
				 * build-invisible node.  See the long note above. */
				assert(defer_parent);
				if (_replace_old_ptr)
					*_replace_old_ptr = true;
			} else {
				/*
				 * Recompact-on-insert (default): refilling a
				 * soft-deleted hole (bit sticky-set, slot NULL) bumps
				 * this LIVE node's nr_child -- report -ERANGE so the
				 * wrapper routes through ft_node_recompact(ADD_SAME)
				 * (the fresh copy drops the NULL hole and re-adds byte
				 * n).  @defer_parent = build-invisible node: keep in
				 * place (also avoids recompact-within-recompact).
				 */
				if (!ft_in_place_ok(ft) && !defer_parent)
					return -ERANGE;
				if (_replace_old_ptr)
					*_replace_old_ptr = false;
				ft_node_count_add(metadata, defer_parent,
						deferred_count);
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
		ft_node_count_add(metadata, defer_parent,
				deferred_count);
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
		bool defer_parent,
		bool *deferred_count)
{
	struct cds_ft_inode_flag **ptr;
	bool replace_old_ptr = false;

	assert(ft_type_is_pigeon(type->type_class));
	/* Parent-first (see ft_popcount_node_set_nth). */
	if (!defer_parent)
		ft_set_parent_raw(ft, child_node_flag, node_flag);
	ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	/*
	 * Recompact-on-insert (see FEATURE_FT_INSERT_IN_PLACE): retire the
	 * in-place occupancy-bitmap set on a LIVE pigeon by reporting -ERANGE
	 * for a new occupancy, so the wrapper rebuilds the node via
	 * ft_node_recompact(ADD_SAME) (uniform with popcount).  A replace at
	 * an occupied slot, and a build-into-unpublished-node insert
	 * (defer_parent), stay in place; pigeon has no reserved-byte state,
	 * so an empty slot is a genuinely new key.
	 *
	 * NOTE (future, not done -- kept simple for now): a pigeon slot is
	 * direct-indexed, so the pointer store is already a clean flip edge
	 * and the bitmap is only an occupancy HINT (point lookups read the
	 * slot; iteration rescans past a set bit whose slot is NULL).  So the
	 * whole-node recompact is avoidable for pigeon: make the bitmap a
	 * STICKY hint instead -- set with an atomic OR, never cleared in
	 * place (a delete leaves the bit; only a recompact rebuilds a clean
	 * bitmap), optionally triggering a cleanup recompact once stale bits
	 * (set bit over NULL slot = popcount(bitmap) - nr_child) get high.
	 * That keeps pigeon's O(1) insert/delete; deferred to avoid bundling
	 * too many changes here (it also needs the verify cross-check and the
	 * delete bit-clear relaxed for the sticky semantics).
	 */
	if (!ft_in_place_ok(ft) && !defer_parent && !*ptr)
		return -ERANGE;
	if (*ptr)
		replace_old_ptr = true;
	rcu_assign_pointer(*ptr, child_node_flag);
	if (!replace_old_ptr) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
		ft_node_count_add(metadata, defer_parent,
				deferred_count);
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
		bool defer_parent,
		bool *deferred_count)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, NULL, is_init, defer_parent, deferred_count);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, defer_parent, deferred_count);
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
		/*
		 * Recompact-on-remove (default, concurrent-safe): EVERY delete
		 * rebuilds the node fresh (FT_RECOMPACT_DEL, dropping child n)
		 * so a consistent (bitmap, nr_child) is published in ONE pointer
		 * swap.  The in-place alternative below stores NULL into the
		 * child slot and decrements nr_child IN PLACE on the LIVE node;
		 * a concurrent lookup reads that torn -- the bitmap and nr_child
		 * are separate words, so a reader that sampled nr_child before
		 * the decrement walks a child index i >= the new nr_child and
		 * trips ft_popcount_node_get_ith_pos's `i < nr_child` assert.
		 * Under FEATURE_FT_INSERT_IN_PLACE (single writer, no concurrent
		 * reader mid-mutation) the in-place fast path is retained; there
		 * we only recompact once the node would shrink below min_child.
		 */
		if (!ft_in_place_ok(ft) ||
		    ft_meta_nr_child_load(metadata) <= type->min_child)
			return -EFBIG;
	}
	dbg_printf("popcount replace ptr: node %p\n", node);
	assert(*node_flag_ptr != NULL);
	/*
	 * Fusion armed: DEFER the forward store into @pub so ft_detach_node
	 * commits it in one flip with the dead head cell's unsplice.  A delete
	 * (NULL newptr) stores NULL and decrements nr_child in place
	 * (reader-invisible for navigation); an external promote (non-NULL
	 * newptr) replaces the child with the external chain head -- its
	 * back-pointer re-parent is CAPTURED into @pub and committed with the
	 * forward flip (an arm-time store survived a commit ABORT torn, with
	 * the retrying remove leaving the still-chained head pointing at the
	 * holder).  nr_child unchanged for a promote.
	 */
	if (pub) {
		if (newptr) {
			struct cds_ft_node *en = (struct cds_ft_node *)
				ft_node_ptr(newptr);

			pub->head_parent_field = ft->ordered_list ?
				&ft_ord_cell_ptr(en->prev)->parent :
				(struct cds_ft_inode_flag **) &en->prev;
			pub->head_parent_old = *pub->head_parent_field;
			pub->head_parent_new = node_flag;
		}
		pub->slot = node_flag_ptr;
		/* @slot is a child slot of THIS node, promote or delete alike. */
		pub->slot_owner = metadata;
		pub->old_val = *node_flag_ptr;
		pub->new_val = newptr;
		pub->armed = true;
		if (!newptr)
			pub->state_meta = metadata;	/* nr_child-- fuses into the commit flip */
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
	ft_node_child_edge_flip(ft, node_flag_ptr, *node_flag_ptr, newptr);
	if (!newptr)
		ft_meta_nr_child_dec_flip(metadata);
	dbg_printf("popcount replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) ft_popcount_node_get_nr_child(type, node),
		(unsigned int) ft_meta_nr_child_load(metadata),
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
		/*
		 * Recompact-on-remove (default, concurrent-safe): rebuild the
		 * node fresh rather than soft-deleting the slot and decrementing
		 * nr_child IN PLACE on the LIVE node -- a concurrent lookup reads
		 * the nr_child change torn against the pointer/bitmap state.  See
		 * the popcount variant for the full rationale.
		 */
		if (!ft_in_place_ok(ft) ||
		    ft_meta_nr_child_load(metadata) <= type->min_child)
			return -EFBIG;
	}
	dbg_printf("ft_pigeon_node_replace_ptr: replace ptr: %p by %p\n", *node_flag_ptr, newptr);
	assert(*node_flag_ptr != NULL);
	/*
	 * Fusion armed: DEFER the forward store into @pub (committed in one flip
	 * with the dead head cell's unsplice).  A DELETE (NULL newptr) fuses its
	 * nr_child-- into the commit flip (via @state_meta) and leaves the
	 * occupancy bitmap bit SET -- a sticky soft-delete hint, never cleared in
	 * place: the pointer load is the source of truth (ft_pigeon_node_get_nth
	 * reads node->data[n] directly and the directional scan rescans past a set
	 * bit over a NULL slot), and a later recompact rebuilds a clean bitmap from
	 * the occupied slots.  An external PROMOTE (non-NULL newptr) captures the
	 * promoted external's back-pointer re-parent into @pub -- committed WITH
	 * the forward flip, not stored at arm time (see the popcount variant) --
	 * and leaves the slot occupied and nr_child unchanged.
	 */
	if (pub) {
		if (newptr) {
			struct cds_ft_node *en = (struct cds_ft_node *)
				ft_node_ptr(newptr);

			pub->head_parent_field = ft->ordered_list ?
				&ft_ord_cell_ptr(en->prev)->parent :
				(struct cds_ft_inode_flag **) &en->prev;
			pub->head_parent_old = *pub->head_parent_field;
			pub->head_parent_new = node_flag;
		}
		pub->slot = node_flag_ptr;
		/* @slot is a child slot of THIS node, promote or delete alike. */
		pub->slot_owner = metadata;
		pub->old_val = *node_flag_ptr;
		pub->new_val = newptr;
		pub->armed = true;
		if (!newptr)
			pub->state_meta = metadata;	/* nr_child-- fuses into the commit flip */
		return 0;
	}
	/* Parent-first: wire the back-pointer before the forward publish,
	 * past the -EFBIG recompaction check (see popcount variant). */
	ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
	ft_node_child_edge_flip(ft, node_flag_ptr, *node_flag_ptr, newptr);
	if (!newptr) {
		/*
		 * Soft-delete: the slot is NULLed (the flip above), nr_child
		 * decrements as a committed flip, and the occupancy bitmap bit
		 * is left SET -- a sticky hint, never cleared in place (the
		 * pointer load is the source of truth; recompact rebuilds it
		 * clean), exactly as the popcount layout already does.
		 */
		ft_meta_nr_child_dec_flip(metadata);
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
		/*
		 * Bound the search against ft_types[] (Phase 4.3 defense): a
		 * garbage @nr_nodes -- e.g. a raw read of a node whose state word a
		 * peer parked with a re-home proxy, before the ft_meta_nr_child_load
		 * callers -- would otherwise walk @type_index off the end of the
		 * table (unbounded ++), reading junk min/max_child and spinning
		 * (the FT_INV_MW hang) or faulting.  With the resolving loads this
		 * never fires; keep it so no future raw read can wedge the trie.
		 */
		assert(type_index < FT_TYPE_MAX_NR);
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
 *
 * @nullify_expected (FT_RECOMPACT_DEL only): the child the CALLER's plan drops,
 * sampled from @nullify_node_flag_ptr when that plan was built.  The DEL arm
 * identifies its victim by re-reading the slot under the F2 fence, so without
 * this expected-old it drops whatever the slot holds THEN -- which, after a peer
 * republished the slot between the plan and the fence, is the peer's freshly
 * published subtree, silently dropping keys the plan never covered.  Mismatch is
 * a stale plan: -EAGAIN, re-descend.
 *
 * @rec: the accumulator for reader-visible forward stores.
 *  - The forward publish into @old_node_flag_ptr is recorded into @rec ONLY for
 *    FT_RECOMPACT_RELOCATE, where @old_node_flag_ptr is the LIVE slot and the
 *    caller (ft_compact_relocate_at) commits @rec.  For the ADD/SAME/DEL
 *    mutators @old_node_flag_ptr is a LOCAL out-param re-published by the
 *    caller, so the forward stays a bare store into that local.
 *  - A compressed parent's SKIP_X dual (the live grandparent skip slot) is
 *    recorded into @rec whenever @rec is non-NULL: for RELOCATE, AND for an
 *    ADD/SAME relocation whose LIVE caller threads its own commit rec so the
 *    dual flips ATOMICALLY with that caller's forward publish (else the dual
 *    would be a bare store ordered BEFORE the deferred forward).  @rec == NULL
 *    keeps the bare SKIP_X store for build-invisible ADD recompacts.
 *  - FT_RECOMPACT_DEL is the exception: it reaches here with @rec == NULL but
 *    its node is LIVE, so a bare SKIP_X store would be the same premature
 *    non-atomic window.  Its caller (ft_detach_node) republishes the forward
 *    via _ft_publish_to_parent(parent_nf = the compressed parent), which RECORDS
 *    this SKIP_X dual itself, so DEL defers the dual entirely (records nothing
 *    here -- see the inline note at the skip slot).
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
		struct cds_ft_inode_flag *nullify_expected,
		struct cds_ft_inode **old_node_ret,
		bool is_root,
		unsigned int node_depth,
		bool cluster_leaf,
		struct ft_pub_rec *rec,
		struct ft_flip_txn *retire_txn,
		const struct ft_parent_hint *inh_hint,
		const struct ft_lock_ctx *ctx)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	/*
	 * old_node's flag (type tag encoded) for a live retire: lets the copy
	 * loop / reparent sweep address old_node's source child slots via
	 * ft_node_get_nth_skip, to resolve each to a definite child and freeze it
	 * as a COPY_SLOT.  Stays NULL on the build-invisible / cluster-leaf /
	 * no-txn arms, which copy nodes no peer publishes into (raw read, no
	 * freeze).  Assigned once the copy path is reached (old_node non-NULL).
	 */
	struct cds_ft_inode_flag *old_node_flag = NULL;
	int ret;
	/*
	 * Track whether new_node has received its first child via
	 * is_init=true.  Popcount nodes use a single init-done flag
	 * (no per-subnode state).
	 */
	bool new_init_done = false;
	/*
	 * ONE resolved snapshot of the old node's external-head word: the
	 * word itself is flip-managed (a key-disappearing remove parks its
	 * proxy there), so a raw multi-read could inherit a latch or tear
	 * across a peer's head swap.  Captured latch-checked below; every
	 * later use (Phase-1 metadata, back-channel derive, deferred plain
	 * publish) goes through this snapshot.  Non-NULL also flags the
	 * deferred plain-store arm (build-invisible / no-txn): its Phase-2
	 * prev publish moves to after the copy loops so the copied-slot
	 * latch bail point has no reader-visible effect; the retire_txn arm
	 * records the edge into the commit instead.
	 */
	struct cds_ft_node *ext_snapshot = NULL;
	bool bc_plain = false;
	/*
	 * DEL: the to-remove slot's value, captured ONCE (latch-checked at
	 * capture).  The copy loops compare child VALUES against this stable
	 * capture -- re-reading *nullify_node_flag_ptr per iteration could
	 * see a peer's latch park between reads and silently COPY the child
	 * being removed.  A child flag appears in exactly one slot, so the
	 * value compare is exact AGAINST THIS CAPTURE; it does NOT detect a
	 * peer's clean republish of the slot (old -> fresh copy) in the
	 * capture->copy window -- that stale-plan window is closed by the F2
	 * node lock (the mark precedes the capture), not here.
	 */
	struct cds_ft_inode_flag *nullify_val = NULL;
	/*
	 * F2 node lock state: set when this recompact retires a LIVE
	 * published node (@retire_txn arm).  @c_held names BOTH words a coarse
	 * spacing splits apart: the one the acquire CAS'd, and C's own clean
	 * pre-mark state word -- the ONE snapshot the whole copy plan (type
	 * sizing, tombstone expected-old) derives from.
	 */
	struct ft_held_anchor c_held = { 0 };
	bool fenced = false;
	/*
	 * §9.3 LOCK_FINE lock-set, RELEASE half: the members this recompact locks
	 * that SURVIVE the commit -- P (the parent whose slot the fresh copy is
	 * published into) and, when P is a compressed node whose SKIP_X dual this
	 * recompact re-encodes, GP (whose slot that dual writes).  C -- the node
	 * copied away -- is the RETIRE half (@fenced / @c_held above); the two
	 * halves differ only in the terminal they record at commit.
	 *
	 * Held only under FINE: COARSE derives no lock-set (§10.5, one FT-wide
	 * lock).  (The OPTIMISTIC arm this also described -- "keeps its §4.B
	 * guards, which the release record would poison" -- is gone with the
	 * strategy; see ft_flip_txn_record_release_lock.)
	 */
	struct ft_held_anchor rel_held[2];
	unsigned int nr_rel = 0, ri;

	/*
	 * F2 node lock, MARK (doc at ft_meta_lock_acquire): a live-retire
	 * locks the old node BEFORE any body/state read below -- the
	 * sizing nr_child loads, the (parent, offset) inherit, the external-
	 * head snapshot, and the copy loops all read under the fence, and the
	 * commit's state record {LOCK|s -> TOMBSTONE|s} pins the word so a
	 * peer publish that slips a state change under the fence aborts one of
	 * the two.  A dirty mark (peer proxy / concurrent copier / real
	 * retire) bails to the op's retry before anything is allocated.  The
	 * build-invisible (@cluster_leaf) and legacy no-txn arms copy nodes no
	 * peer publishes into (unpublished cluster / retained exclusion), so
	 * they stay unfenced.
	 */
	/*
	 * DLM Step 1 (see doc/design/mw-writer-lock-escalation-model.md):
	 * acquire the WHOLE lock-set
	 * {C, P, (GP)} in ONE all-or-none MCAS up front, replacing the incremental
	 * marks (C here, P at the inherit, GP at the skip-dual).  §9.3: P is resolved
	 * from C and validated -- the read-set guard C.parent==P (and P.parent==GP)
	 * rides the SAME commit, so a re-home between the racy plan read and the
	 * acquire aborts it -> re-plan.  Populates the same @fenced / @c_held /
	 * @rel_held / @nr_rel the incremental scheme does, so the build, the commit
	 * terminals (retire C via @c_held, release P/GP via @rel_held), and the
	 * abandon_fresh unwind are all unchanged below.  Only the
	 * LOCK_FINE retire arm hoists; the universal F2 lock (non-lock_fine /
	 * flag-off) keeps its single mark.
	 */
	if (ft->lock_fine && retire_txn && !cluster_leaf && metadata && old_node) {
		struct cds_ft_inode_flag *pf_p = NULL, *pf_gp = NULL;
		struct cds_ft_metadata *p_meta = NULL, *gp_meta = NULL;
		struct ft_dlm_member set[3];
		unsigned int p_depth = 0, gp_depth = 0;
		int dret;

		/* PLAN (read-only, racy): resolve P (+GP iff P compressed). */
		if (inh_hint)
			pf_p = ft_parent_node(inh_hint->parent);
		else
			(void) ft_resolve_parent_slot(metadata, ft, &pf_p);
		if (pf_p) {
			p_meta = ft_flag_to_metadata(ft, pf_p);
			if (ft_node_compressed(pf_p) ||
					ft_node_skip_compressed(pf_p)) {
				if (inh_hint)
					pf_gp = inh_hint->gp;
				else
					(void) ft_resolve_parent_slot(p_meta,
						ft, &pf_gp);
				if (pf_gp)
					gp_meta = ft_flag_to_metadata(ft, pf_gp);
			}
		}

		/*
		 * FOLD (parent_held): P is already LOCK-held by an earlier step of
		 * the SAME op (a same-trie rekey's graft locked the shared spine), so
		 * do NOT re-lock it (a second ft_dlm_lock would abort -EAGAIN) and do
		 * NOT add it to @rel_held below (the holder owns its release).  P stays
		 * the republish target; only C (and, if present, GP) are acquired here.
		 *
		 * The guard is NOT skipped with the lock: @pf_p is then the CALLER's
		 * held-node identity, and the reuse is only sound while C really hangs
		 * off it, so C.parent == P rides this same acquire commit.  It makes the
		 * fold VALIDATE at commit what it would otherwise assume from a
		 * descent-time relationship, rather than park an SW store into a word it
		 * does not hold.  HONEST SCOPE (measured: 0 mismatches in 5525 folds):
		 * under the fold's OWN precondition it cannot fire -- re-homing C means
		 * rewriting P's child set, which needs P's LOCK, which the holder has
		 * for the whole window, and a re-home that happened EARLIER tombstoned
		 * the old P so the holder's own ft_dlm_lock(P) aborted first.  Its value
		 * is the shapes the precondition does not cover (a climbing detach, where
		 * C is a higher ancestor than the hint's slot names) and any future
		 * relaxation of the driver's shape gate -- i.e. it keeps the invariant
		 * checked by the machine instead of by a comment.
		 */
		bool p_held = inh_hint && inh_hint->parent_held;

		/*
		 * A held P also SKIPS the GP lock below, but the SKIP_X dual re-encode
		 * asserts GP was acquired whenever P is compressed -- so the fold must
		 * never be entered with a compressed P.  The only producer (the same-trie
		 * rekey driver) rejects compressed nodes at every descent level, and
		 * FT_RECOMPACT_DEL defers the dual; fail the acquire rather than trust
		 * that if a future caller breaks it.
		 */
		if (p_held && gp_meta)
			return -EAGAIN;

		/*
		 * P and GP were reached through a back-pointer or a caller's
		 * hint, so neither carries a depth.  Each is dated from the node
		 * BELOW it -- C by @node_depth, P by C -- with the descent's
		 * window answering directly whenever it describes the member:
		 * the window holds the last four nodes the descent passed, and a
		 * three-ancestor set whose C already sits at the third slot runs
		 * off it.  A member neither source can date is one this plan
		 * cannot anchor: re-plan rather than borrow another node's depth.
		 */
		if ((p_meta && !ft_lock_ctx_depth_of_parent(ft, ctx, pf_p,
					node_depth, &p_depth)) ||
				(gp_meta && !ft_lock_ctx_depth_of_parent(ft, ctx,
					pf_gp, p_depth, &gp_depth)))
			return -EAGAIN;

		/*
		 * ACQUIRE {C, (P), (GP)} + read-set guards in one MCAS.
		 *
		 * C's guard validates the plan's racy read of its parent.  With
		 * a hint the parent identity is the CALLER's, and an ordinary
		 * hint user does NOT want it validated against C's lazily-updated
		 * back-pointer (see ft_parent_hint) -- only a caller that asks
		 * for it (@parent_guard: the rekey fold's non-held src junction,
		 * whose republish parks SW into @parent's slot) gets it, and the
		 * FOLD arm gets it unconditionally because its lock is skipped.
		 */
		set[0] = (struct ft_dlm_member){
			.nf = *old_node_flag_ptr, .node = metadata,
			.depth = node_depth,
			.guard_child = (p_meta && (p_held || !inh_hint ||
				inh_hint->parent_guard)) ? metadata : NULL,
			.guard_pf = pf_p };
		set[1] = (struct ft_dlm_member){
			.nf = (p_meta && !p_held) ? pf_p : NULL,
			.node = p_meta, .depth = p_depth };
		set[2] = (struct ft_dlm_member){
			.nf = (gp_meta && !p_held) ? pf_gp : NULL,
			.node = gp_meta, .depth = gp_depth,
			.guard_child = inh_hint ? NULL : p_meta,
			.guard_pf = pf_gp };
		if (ft_recompact_fault_refuse_acquire(mode))
			return -EAGAIN;		/* test-only; nothing acquired */
		dret = ft_dlm_acquire_set(ft, ctx, set, 3);
		if (dret)
			return dret == -ENOMEM ? -ENOMEM : -EAGAIN;

		/* Populate the lock-set state -- build/commit/unwind unchanged. */
		fenced = true;
		c_held = set[0].held;
		if (set[1].nf)
			rel_held[nr_rel++] = set[1].held;
		if (set[2].nf)
			rel_held[nr_rel++] = set[2].held;
	} else
	if (retire_txn && !cluster_leaf && metadata && old_node) {
		ret = ft_acquire_member(ft, ctx, *old_node_flag_ptr, metadata,
			node_depth, &c_held);
		if (ret)
			return ret;
		fenced = true;
	}

	/*
	 * Need to find nearest type index even for ADD_SAME, so that
	 * recompaction can promote/demote across tier boundaries
	 * (e.g. a popcount node that no longer fits its current tier).
	 */
	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		new_type_index = find_nearest_type_index(old_type_index,
			ft_meta_nr_child_load(metadata) + 1, false);
		dbg_printf("Recompact for node with %u children\n",
			ft_meta_nr_child_load(metadata) + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			new_type_index = find_nearest_type_index(old_type_index,
				ft_meta_nr_child_load(metadata) + 1, false);
			dbg_printf("Recompact for node with %u children\n",
				ft_meta_nr_child_load(metadata) + 1);
		}
		break;
	case FT_RECOMPACT_DEL:
		new_type_index = find_nearest_type_index(old_type_index,
			ft_meta_nr_child_load(metadata) - 1, is_root);
		dbg_printf("Recompact for node with %u children\n",
			ft_meta_nr_child_load(metadata) - 1);
		break;
	case FT_RECOMPACT_RELOCATE:
		new_type_index = old_type_index;	/* same type, pure relocation */
		break;
	default:
		assert(0);
	}

	/*
	 * SIZE -> COPY window (-DFT_DELAY_INJECT only).  The replacement node's
	 * type was just chosen from a live count read; the copy loop below then
	 * skips any source child a peer has removed in between.  Widen the gap
	 * so that skip actually happens under test.
	 */
#ifndef FT_DELAY_SITE_A_ONLY
	ft_delay_writer();
#endif
	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		/*
		 * Phase 4.3 atomic re-home: when this recompact retires a LIVE
		 * published node whose children stay reader-reachable (retire_txn
		 * set = the unlink/forward-publish commit; not a build-invisible
		 * cluster leaf), the copy loop records each surviving child slot as
		 * a COPY_SLOT (freeze old.child[b] at the resolved child, publish it
		 * into the fresh node's slot) and the reparent sweep records each
		 * child's (parent, offset) as a co-committed pair INTO @retire_txn,
		 * so the whole rebuild flips atomically with the publish -- not
		 * stores a peer reads torn.  Widen the txn for the <=3 edges per
		 * child up front (1 COPY_SLOT + <=2 reparent, <= old nr_child + 1),
		 * +1 for the external head's back-channel edge below; an OOM here
		 * aborts cleanly, nothing allocated yet -- a single-commit insert
		 * may fail its widen (unlike a graft/merge second commit, which
		 * pre-reserves).
		 */
		if (retire_txn && !cluster_leaf && metadata &&
				!ft_flip_txn_reserve_extra(retire_txn,
					3 * (ft_meta_nr_child_load(metadata) + 1)
					+ 1 + (ft->lock_fine ? 2 : 0))) {
			/* Release the WHOLE lock set, not just C: the up-front
			 * acquire took P (and GP) into @rel_held, and this bail is
			 * before the txn registry takes ownership of them, so they
			 * are still ours to clear.  Missing this left P/GP LOCK
			 * for good -- structurally invisible (the trie is
			 * byte-for-byte intact) and fatal to every later op that
			 * needs them. */
			ft_unlock_held(rel_held, nr_rel);
			if (fenced && !c_held.shared)
				ft_meta_lock_release(c_held.lock);
			return -ENOMEM;
		}
		new_node = alloc_cds_ft_node(ft, new_type, &new_metadata);
		if (!new_node) {
			ft_unlock_held(rel_held, nr_rel);
			if (fenced && !c_held.shared)
				ft_meta_lock_release(c_held.lock);
			return -ENOMEM;
		}

		new_node_flag = ft_node_flag(new_node, new_type_index);

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata) {
			struct cds_ft_inode_flag *inh_parent;
			struct cds_ft_inode_flag **inh_slot;

			if (inh_hint) {
				/*
				 * §11 cross-trie graft: the caller's reanchoring
				 * descent captured (parent, slot) coherently at the
				 * LIVE level (d->ppnf, d->pnfp).  Use it verbatim
				 * instead of ft_resolve_parent_slot(@metadata), which
				 * reads @metadata's own back-pointer -- stale, and
				 * once a shared-spine peer frees the old parent it
				 * dangles to a reclaimed node (Defect C).  A NULL
				 * hint->parent is a publish into &ft->root.
				 */
				inh_parent = ft_parent_node(inh_hint->parent);
				inh_slot = inh_hint->slot;
			} else {
				inh_slot = ft_resolve_parent_slot(metadata, ft,
					&inh_parent);
			}

			/*
			 * Inherit the retired node's (parent, offset) as ONE
			 * CONSISTENT snapshot (Phase 4.3 atomic re-home): a peer
			 * that re-homes @metadata's node commits its parent and
			 * state-word offset atomically, so copying them via two raw
			 * reads could tear -- the fresh copy would then invert its
			 * publish slot against the wrong parent and fault
			 * ft_slot_to_byte.  ft_resolve_parent_slot recovers the pair
			 * from a single MCAS status snapshot (a plain read when no
			 * re-home is in flight, so single-writer is unchanged).
			 */
			new_metadata->parent_word = ft_parent_word(ft, inh_parent);
			/* The retyped node keeps its own incoming edge byte. */
			new_metadata->incoming_byte = metadata->incoming_byte;
			/*
			 * Set the offset on EVERY build: it is no longer
			 * skip-specific -- it backs the parent-pointer backtrack's
			 * O(1) slot recovery for plain-internal nodes too.  It must
			 * come from @inh_slot (the coherent snapshot above), never
			 * from a second raw read of the state word, which would tear
			 * against @inh_parent.
			 */
			ft_meta_parent_slot_offset_set(new_metadata, inh_parent ?
				(unsigned int) ((char *) inh_slot -
					(char *) ft_node_ptr(inh_parent))
					/ sizeof(void *) : 0);
			/*
			 * §9.3 LOCK_FINE: acquire P, the second member of the
			 * lock-set.  P's slot is the one the fresh copy is
			 * published into, and it is a SAME-slot value swap (the
			 * copy inherits @incoming_byte, so P's bitmap and
			 * nr_child are untouched and P can never itself recompact
			 * -- no cascade, and no GP beyond the compressed-parent
			 * dual locked further down).  Acquired from the SAME
			 * coherent (parent, offset) snapshot the publish slot is
			 * derived from, so the lock and the slot cannot disagree
			 * about who P is.
			 *
			 * A NULL @inh_parent is a publish into &ft->root: no node
			 * to lock, auto-guarded by the root-slot CAS (identical
			 * to ft_flip_txn_guard_parent's NULL case).
			 *
			 * On failure P is held by a peer: drop C's lock and
			 * re-descend.  Nothing is published (the fresh copy is
			 * build-invisible until the commit), so the abort boundary
			 * is byte-for-byte clean.
			 */
			/* DLM: {C,P,(GP)} were acquired up front (see the block at
			 * function entry); P is already held here. */
			ext_snapshot = (struct cds_ft_node *)
				rcu_dereference(metadata->external_nodes);
			if (caa_unlikely(ft_node_flip_proxy(
					(struct cds_ft_inode_flag *)
						ext_snapshot))) {
				/*
				 * Peer flip parked on the head word itself
				 * (key-disappearing remove): nothing exposed
				 * yet -- bail and retry after it settles.
				 */
				free_cds_ft_node_unpublished(ft, new_node);
				ft_unlock_held(rel_held, nr_rel);
				if (fenced && !c_held.shared)
					ft_meta_lock_release(c_held.lock);
				return -EAGAIN;
			}
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, ext_snapshot);
			if (retire_txn && !cluster_leaf && ext_snapshot) {
				/*
				 * Live retire: the external head's back-channel
				 * (cell->parent / head->prev) re-points to the
				 * fresh node ATOMICALLY with the forward publish
				 * -- a RECORDED edge riding @retire_txn (+1
				 * reserved above), not an early plain store a
				 * peer observes against the not-yet-published
				 * copy, and discarded coherently on ABORT or on
				 * the copied-slot latch bail below.  A peer
				 * latch already parked on the back-channel word
				 * (concurrent head re-home) aborts this attempt
				 * up front.
				 */
				void **bc_slot;
				struct cds_ft_inode_flag *bc_old;

				if (ft->ordered_list)
					bc_slot = (void **) &ft_ord_cell_ptr(
						ext_snapshot->prev)->parent;
				else
					bc_slot = (void **)
						&ext_snapshot->prev;
				bc_old = (struct cds_ft_inode_flag *)
					rcu_dereference(*bc_slot);
				if (caa_unlikely(ft_node_flip_proxy(bc_old))) {
					free_cds_ft_node_unpublished(ft, new_node);
					ft_unlock_held(rel_held, nr_rel);
					if (fenced && !c_held.shared)
						ft_meta_lock_release(c_held.lock);
					return -EAGAIN;
				}
				ft_flip_txn_record_head_back_edge(retire_txn,
					bc_slot, bc_old, new_node_flag);
			} else {
				/*
				 * Build-invisible / legacy no-txn arm: the
				 * back-channel prev publish stays a plain store,
				 * DEFERRED to after the copy loops (the helper's
				 * contract only needs it at-or-just-before the
				 * forward publish, and new_metadata->parent is
				 * already inherited above) so the copied-slot
				 * latch bail point has zero reader-visible
				 * effects to undo.
				 */
				bc_plain = ext_snapshot != NULL;
			}
			ft_nr_keys_store(ft,new_metadata,
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

	if (mode == FT_RECOMPACT_DEL) {
		nullify_val = rcu_dereference(*nullify_node_flag_ptr);
		if (caa_unlikely(ft_node_flip_proxy(nullify_val))) {
			ret = -EAGAIN;
			goto abandon_fresh;
		}
		/*
		 * PLAN EXPECTED-OLD (@nullify_expected, header): the caller
		 * named a subtree to drop; this read only names the slot.  A
		 * peer that republished the slot between the two -- an insert
		 * splitting the compressed chain here, say, whose fresh junction
		 * carries the caller's target AND the peer's own key -- makes
		 * the two differ, and copying "every child except this slot"
		 * then drops the peer's subtree whole.  The F2 fence does not
		 * cover it: the peer's publish is serialized correctly, BEFORE
		 * the mark, so the mark's snapshot already contains it.  A stale
		 * plan cannot be repaired here (the caller's orphan set and
		 * count fold derive from the same sample), so bail to the op's
		 * re-descend.
		 */
		if (caa_unlikely(nullify_val != nullify_expected)) {
			ret = -EAGAIN;
			goto abandon_fresh;
		}
	}

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

	/* Live retire: enable COPY_SLOT freeze of old_node's source slots. */
	if (retire_txn && !cluster_leaf && old_node)
		old_node_flag = ft_node_flag(old_node, old_type_index);

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
			if (old_node_flag) {
				/*
				 * Live retire: RESOLVE this source slot to a
				 * definite child by aging priority (help a peer
				 * child-recompact forward, evict a lower-priority
				 * one) instead of copying its parked flip proxy --
				 * an embedded copy would dangle into the peer's
				 * reclaimed descriptor after its grace period (a
				 * permanent wild edge, and the reparent sweep would
				 * skip the proxy child, orphaning its back-pointer)
				 * -- and instead of bailing, which livelocks under
				 * a stream of peer latches.  The slot is frozen as a
				 * COPY_SLOT in the reparent sweep below (read-set
				 * src == resolved catches a later republish).  DEL
				 * skips the to-remove slot by IDENTITY: a resolved-
				 * value compare would miss a peer recompacting the
				 * target out from under us.
				 */
				struct cds_ft_inode_flag **src_slot;
				void *resolved;

				ft_node_get_nth_skip(old_node_flag, &src_slot, v,
						FT_PF_NONE);
				if (mode == FT_RECOMPACT_DEL &&
						src_slot == nullify_node_flag_ptr)
					continue;
				/*
				 * FOLD the op's own pending slot drop into the
				 * copy (see @pending_del_slot): this recompaction
				 * supersedes the node the drop targets, so the
				 * copy must be born WITHOUT the child.  Applied
				 * BY IDENTITY on the slot, like the nullify above
				 * and the publish below.
				 */
				if (retire_txn && retire_txn->pending_del_slot &&
						src_slot == retire_txn->pending_del_slot) {
					/*
					 * PLAN EXPECTED-OLD, for the reason
					 * @nullify_expected states: a peer that
					 * republished this slot since the plan was
					 * built holds a subtree "copy all but this
					 * slot" would drop whole.  A parked flip
					 * proxy fails the compare too.
					 */
					if (caa_unlikely(rcu_dereference(*src_slot) !=
							retire_txn->pending_del_expected)) {
						ret = -EAGAIN;
						goto abandon_fresh;
					}
					retire_txn->pending_del_folded = true;
					continue;
				}
				if (!ft_flip_txn_resolve_prio(retire_txn,
						(void **) src_slot, &resolved)) {
					ret = -EAGAIN;	/* CAP: retry higher-priority */
					goto abandon_fresh;
				}
				iter = (struct cds_ft_inode_flag *) resolved;
				if (!iter)
					continue;	/* peer removed the child */
				/*
				 * FOLD the op's own pending forward publish
				 * into the copy (see @pending_pub_slot): this
				 * recompaction is replacing the very node the
				 * publish targets, so the copy must be born
				 * holding the new child.  Applied BY IDENTITY
				 * on the slot, like the nullify above -- the
				 * resolve deliberately reads COMMITTED values,
				 * and must keep doing so for every other slot.
				 */
				if (retire_txn && retire_txn->pending_pub_slot &&
						src_slot == retire_txn->pending_pub_slot) {
					iter = retire_txn->pending_pub_val;
					retire_txn->pending_pub_folded = true;
				}
			} else if (caa_unlikely(ft_node_flip_proxy(iter))) {
				/*
				 * Build-invisible / no-txn arm: no peer publishes
				 * into this unpublished node, so a proxy is
				 * unexpected -- bail defensively (the live-retire
				 * arm above documents the copy-a-latch hazard).
				 */
				ret = -EAGAIN;
				goto abandon_fresh;
			}
			/*
			 * Fenced ADD: the descent chose @n because the old
			 * node had no child there -- a live child at @n now
			 * means a peer COMMITTED an insert at this byte
			 * between the descent's read and the fence mark (the
			 * one window the state pin cannot cover: the mark
			 * snapshot already includes the peer's count).  The
			 * blind set_nth below would overwrite the peer's
			 * whole subtree; re-descend and dive into it instead.
			 * Never fires single-writer (the descent's read
			 * holds).
			 */
			if (caa_unlikely(fenced &&
					(mode == FT_RECOMPACT_ADD_NEXT ||
					 mode == FT_RECOMPACT_ADD_SAME) &&
					v == n)) {
				ret = -EAGAIN;
				goto abandon_fresh;
			}
			if (mode == FT_RECOMPACT_DEL && nullify_val == iter)
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
					RECOMPACT_IS_INIT(v), true, NULL);
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

		/*
		 * Adding to a pigeon SOURCE happens only under recompact-on-
		 * insert (the DEFAULT; i.e. not -DFEATURE_FT_INSERT_IN_PLACE): a
		 * new key for a live pigeon routes here as ADD_SAME to retire the
		 * in-place bitmap set.  A new-key insert never fills the pigeon
		 * (an occupied byte is a replace, not an insert), so
		 * find_nearest_type_index stays within the pigeon tier.  In the
		 * opt-in in-place build a pigeon never reaches an ADD recompact
		 * (tighter assert below).
		 */
		assert(mode == FT_RECOMPACT_DEL ||
			mode == FT_RECOMPACT_RELOCATE
			|| (!ft_in_place_ok(ft) &&
			    mode == FT_RECOMPACT_ADD_SAME)
			);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (old_node_flag) {
				/*
				 * Live retire: resolve this source slot to a
				 * definite child by aging priority + freeze it as a
				 * COPY_SLOT below; DEL skips the to-remove slot by
				 * identity.  See the popcount loop for the rationale.
				 */
				struct cds_ft_inode_flag **src_slot;
				void *resolved;

				ft_node_get_nth_skip(old_node_flag, &src_slot,
						(uint8_t) i, FT_PF_NONE);
				if (mode == FT_RECOMPACT_DEL &&
						src_slot == nullify_node_flag_ptr)
					continue;
				/*
				 * FOLD the op's own pending slot drop into the
				 * copy (see @pending_del_slot): this recompaction
				 * supersedes the node the drop targets, so the
				 * copy must be born WITHOUT the child.  Applied
				 * BY IDENTITY on the slot, like the nullify above
				 * and the publish below.
				 */
				if (retire_txn && retire_txn->pending_del_slot &&
						src_slot == retire_txn->pending_del_slot) {
					/*
					 * PLAN EXPECTED-OLD, for the reason
					 * @nullify_expected states: a peer that
					 * republished this slot since the plan was
					 * built holds a subtree "copy all but this
					 * slot" would drop whole.  A parked flip
					 * proxy fails the compare too.
					 */
					if (caa_unlikely(rcu_dereference(*src_slot) !=
							retire_txn->pending_del_expected)) {
						ret = -EAGAIN;
						goto abandon_fresh;
					}
					retire_txn->pending_del_folded = true;
					continue;
				}
				if (!ft_flip_txn_resolve_prio(retire_txn,
						(void **) src_slot, &resolved)) {
					ret = -EAGAIN;
					goto abandon_fresh;
				}
				iter = (struct cds_ft_inode_flag *) resolved;
				if (!iter)
					continue;
				/*
				 * FOLD the op's own pending forward publish
				 * into the copy (see @pending_pub_slot): this
				 * recompaction is replacing the very node the
				 * publish targets, so the copy must be born
				 * holding the new child.  Applied BY IDENTITY
				 * on the slot, like the nullify above -- the
				 * resolve deliberately reads COMMITTED values,
				 * and must keep doing so for every other slot.
				 */
				if (retire_txn && retire_txn->pending_pub_slot &&
						src_slot == retire_txn->pending_pub_slot) {
					iter = retire_txn->pending_pub_val;
					retire_txn->pending_pub_folded = true;
				}
			} else if (caa_unlikely(ft_node_flip_proxy(iter))) {
				/* Copied-slot latch (unpublished arm): see popcount. */
				ret = -EAGAIN;
				goto abandon_fresh;
			}
			/* Fenced ADD occupied byte: see the popcount loop. */
			if (caa_unlikely(fenced &&
					(mode == FT_RECOMPACT_ADD_NEXT ||
					 mode == FT_RECOMPACT_ADD_SAME) &&
					(uint8_t) i == n)) {
				ret = -EAGAIN;
				goto abandon_fresh;
			}
			if (mode == FT_RECOMPACT_DEL && nullify_val == iter)
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
					RECOMPACT_IS_INIT((uint8_t)i), true, NULL);
			assert(!ret);
		}
		break;
	}
	default:
		/*
		 * Statically unreachable (every ft_types entry is POPCOUNT /
		 * PIGEON / NULL), but route through abandon_fresh anyway: it is
		 * the one failure arm past the lock acquire, and `goto end`
		 * here would leak both the fresh node and the fence if a new
		 * type class ever forgot to extend this switch.
		 */
		assert(0);
		ret = -EINVAL;
		goto abandon_fresh;
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
				RECOMPACT_IS_INIT(n), true, NULL);
		assert(!ret);
	}

#undef RECOMPACT_IS_INIT

	/*
	 * Deferred Phase-2 back-channel publish (plain-store arm only; the
	 * retire_txn arm recorded it above): past the copy loops the attempt
	 * can no longer bail, so this is the first moment the fresh node may
	 * become peer-visible.
	 */
	if (bc_plain)
		ft_publish_external_nodes_prev(ft, new_node_flag,
			ext_snapshot);

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
		/*
		 * F1 discipline on the BACK-EDGE: never clone a peer's parked flip
		 * proxy into the fresh copy's parent.  The peer's txn record names
		 * the OLD node's &meta->parent, so a cloned proxy at the fresh
		 * copy's meta->parent is owned by nobody and is NEVER settled: it
		 * outlives the owner's reclaim (its rcu_head is already queued),
		 * so ft_resolve_parent_slot() either spins forever in its
		 * unbounded for(;;) -- the "parent parked but offset settled" arm,
		 * observed as a 1-core MW livelock -- or later resolves through
		 * freed descriptor memory (the CORE_682870 dangling-proxy UAF).
		 *
		 * Resolving is exact, not just a fresher guess, because a
		 * parked @meta->parent here is always a DOOMED peer.  The only
		 * writer that parks it is ft_reparent_record_meta(), which ALWAYS
		 * records the &meta->state edge alongside it.  So either the peer
		 * parked before our ft_meta_lock_acquire(), which then observed
		 * FT_STATE_PROXY and bailed -EAGAIN (we never reach here), or it
		 * parks after, and its state-word install fails against the word
		 * the fence pinned to {LOCK|s -> TOMBSTONE|s}, aborting it.
		 * Either way ft_resolve_flip_proxy() returns old_ptr == the true
		 * current parent.  (A peer that COMMITTED before the mark leaves a
		 * plain, coherent new parent; the caller's ft_get_parent_slot()
		 * re-validate handles that orthogonal case.)
		 *
		 * Resolving also keeps ft_node_compressed() below from misreading
		 * the 0xF proxy tag (bit 1 set) as a compressed node.
		 *
		 * The unfenced arms (retire_txn == NULL / @cluster_leaf) copy only
		 * build-invisible nodes, into whose meta->parent no peer can park:
		 * there the resolve is the identity.
		 */
		/*
		 * §11 cross-trie graft (Defect C): the SKIP_X dual re-encode
		 * below rewrites the GRANDPARENT's skip pointer, so it must name
		 * the SAME coherent grandparent the forward publish uses -- the
		 * LIVE reanchored @inh_hint->parent (== d->ppnf) -- NOT
		 * old_meta->parent, which is the very stale/dangling back-pointer
		 * the hint exists to avoid.  Without this the SKIP_X store lands
		 * in a relocated (or freed/reclaimed) grandparent: the wild store
		 * merely moves from the forward slot to the dual slot.  @inh_hint
		 * is supplied only for a LIVE-node recompact (metadata == old_meta);
		 * the build-invisible arm parks no back-edge, so it keeps the
		 * identity read.
		 */
		struct cds_ft_inode_flag *old_parent = inh_hint ?
			inh_hint->parent :
			ft_resolve_flip_proxy((struct cds_ft_inode_flag *)
				rcu_dereference(old_meta->parent_word));

		/*
		 * Inherit (parent, offset) ONLY for a dest with no @metadata --
		 * a build-invisible node (fresh cluster/junction) that no peer can
		 * reach, so nothing parks its back-edge and the raw pair below is
		 * trivially coherent.
		 *
		 * A LIVE node (@metadata set) already got its (parent, offset)
		 * from the ONE coherent ft_resolve_parent_slot() snapshot above.
		 * Re-deriving them here from two raw reads would TEAR against that
		 * snapshot if a peer re-homed the node in between: the parent
		 * pointer and the state-word offset flip as a co-committed PAIR
		 * (ft_reparent_record_meta), and only ft_resolve_parent_slot's
		 * same-mcas + stability re-read recovers them atomically.  It is
		 * currently a same-value rewrite because the node lock keeps
		 * a peer from parking either edge -- but relying on the fence for
		 * COHERENCE (as opposed to for the resolve's exactness below) is
		 * an invariant this code should not have to know.
		 */
		assert(!metadata || metadata == old_meta);
		if (!metadata) {
			new_metadata->parent_word = old_parent;
			ft_meta_parent_slot_offset_set(new_metadata,
				ft_meta_parent_slot_offset(old_meta));
		}

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (old_parent && ft_node_compressed(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			/*
			 * §11 cross-trie graft (Defect C, SKIP_X level): the dual
			 * slot is a slot in cn's parent (the great-grandparent).
			 * ft_get_parent_slot(cn_meta) recovers it from cn's OWN raw
			 * back-pointer, which dangles to a freed great-grandparent
			 * once a peer relocates it (lazy reanchor).  When the caller
			 * supplied a descent hint, use its coherent slot (d->ppnfp),
			 * captured by navigating the CURRENT tree -- the live
			 * relocated great-grandparent, not cn's stale back-edge.
			 */
			struct cds_ft_inode_flag **skip_slot = inh_hint ?
				inh_hint->gp_slot :
				ft_get_parent_slot(cn_meta, ft);

			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot)) {
				/*
				 * §9.3 LOCK_FINE: P is compressed and carries a
				 * SKIP_X dual, so this recompact's publish also
				 * re-encodes @skip_slot -- a slot in GP.  GP is
				 * therefore the third lock-set member, in EVERY
				 * mode: DEL does not write the dual here, but it
				 * does not escape it either -- it defers the very
				 * same write to ft_detach_node's republish, which
				 * commits in THIS txn.  So the lock is taken here
				 * (the only place that knows P is compressed) and
				 * released by the commit either way.
				 *
				 * Like P, a same-slot value swap: SKIP_X(old, len)
				 * -> SKIP_X(new, len) leaves GP's shape untouched.
				 * A NULL GP means the dual lives in &ft->root.
				 */
				/*
				 * @new_node gates the lock so that "locked" implies
				 * "reserved": the +2 widen for these two release
				 * records lives in the new_type_index != NULL arm
				 * above, which is also the only arm that produces a
				 * fresh copy.  A DEL that shrinks its node away
				 * entirely (NULL type = prune) allocates no copy, so
				 * it re-encodes no dual here -- there is nothing to
				 * point GP's SKIP_X at -- and the prune's own edges
				 * belong to the caller's lock-set (remove, step 5),
				 * not to this atom.
				 */
				if (ft->lock_fine && fenced && new_node) {
					struct cds_ft_inode_flag *gp_parent;

					/*
					 * Same §11 coherence: the great-grandparent
					 * to node lock is cn's parent.  Prefer the
					 * hint's LIVE reanchored great-grandparent
					 * (d->pppnf) over ft_resolve_parent_slot(cn_meta),
					 * which reads cn's stale back-pointer and would
					 * false-succeed the lock on a reclaimed node.
					 * NULL @gp (cn at the root: dual lives in
					 * &ft->root) skips the lock, as the raw NULL did.
					 */
					if (inh_hint)
						gp_parent = inh_hint->gp;
					else
						(void) ft_resolve_parent_slot(cn_meta,
							ft, &gp_parent);
					/* DLM: GP was acquired up front (whenever
					 * P is compressed) -- already held. */
					(void) gp_parent;
				}
				/*
				 * FT_RECOMPACT_DEL relocates the rebuilt
				 * (smaller) node into a LOCAL out-param that
				 * ft_detach_node republishes via
				 * _ft_publish_to_parent(parent_nf = this very
				 * compressed parent), which RECORDS this SKIP_X
				 * dual into the op's commit rec so it flips
				 * ATOMICALLY with the forward cn->child store.
				 * Performing (or recording) it here too would be
				 * a premature store ordered BEFORE that deferred
				 * forward -- a non-atomic window where a candidate
				 * reader follows the skip to the new node while an
				 * exact reader following cn->child still sees the
				 * old.  Defer it entirely to the caller's publish.
				 * (DEL is the lone live-slot mutator reaching here
				 * with rec == NULL; the other rec == NULL cases are
				 * build-invisible ADD recompacts.)
				 */
				if (mode != FT_RECOMPACT_DEL) {
					struct cds_ft_inode_flag *skip_new =
						ft_skip_compressed_flag(
							new_node_flag, cn->len);
					/*
					 * @skip_slot is a slot in GP (see above),
					 * and §8.2 puts a node's body under its
					 * own lock -- so GP owns this edge.  Taken
					 * from the descent hint when there is one,
					 * for the same reason @skip_slot itself is:
					 * cn's back edge can be stale.
					 */
					struct cds_ft_inode_flag *skip_owner_nf =
						NULL;

					if (skip_slot != &ft->root) {
						if (inh_hint)
							skip_owner_nf =
								inh_hint->gp;
						else
							(void) ft_resolve_parent_slot(
								cn_meta, ft,
								&skip_owner_nf);
					}

					if (rec)
						/* SW compaction: *slot == plan old.
						 * A compressed ROOT's dual slot
						 * IS &ft->root. */
						ft_pub_rec_add(rec, skip_slot,
							*skip_slot, skip_new,
							skip_slot == &ft->root,
							skip_owner_nf ?
							ft_flag_to_metadata(ft,
								skip_owner_nf) :
							NULL);
					else
						*skip_slot = skip_new;
				}
			}
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
				if (retire_txn) {
					/*
					 * No per-slot COPY_SLOT freeze: the node-level
					 * node lock (ft_meta_lock_acquire, set before
					 * the copy loop) already froze every source slot
					 * of the retiring node against a peer republish --
					 * a peer's §4.B clean-LIVE guard fails against the
					 * LOCK bit -- and the child value was copied
					 * into @new_node above.  Only the live re-parent
					 * edge remains to record.
					 */
					/*
					 * The DLM acquire locks {C,P,(GP)}
					 * only -- C's CHILDREN are never in
					 * it -- so the §4.B guard must
					 * VALIDATE, not park.
					 *
					 * ☠ THAT IS TRUE OF THIS RECOMPACTION
					 * AND FALSE OF THE OP.  A second
					 * subsystem in the same op can hold one
					 * of C's children: an atomic rekey whose
					 * source junction is at depth 1 makes C
					 * the ROOT, and its merge arm holds the
					 * publish parent, which is a root child.
					 * The validate expects the word CLEAN,
					 * the word carries the op's OWN LOCK, so
					 * the install CAS can never match and
					 * every attempt aborts -- a livelock
					 * with no contention.
					 *
					 * So hand the recorder @ctx, the op's
					 * whole held set, and let it answer the
					 * question this site cannot see.  A
					 * child found there takes NO state edge:
					 * this sweep is never the acquirer, so
					 * the step that took the word still owes
					 * its release.
					 */
					ft_reparent_record(ft, retire_txn, iter,
							new_node_flag, slot,
							/*child_marked=*/ false,
							ctx);
				} else
					ft_set_parent(ft, iter, new_node_flag,
							slot);
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
				if (retire_txn) {
					/* No per-slot freeze: the node-level LOCK
					 * fence covers every source slot.  See the
					 * popcount sweep above. */
					/*
					 * @ctx for the reason the popcount sweep
					 * above spells out: "C's children are
					 * never in the DLM set" is true of this
					 * recompaction and false of the op.
					 */
					ft_reparent_record(ft, retire_txn, iter,
							new_node_flag, slot,
							/*child_marked=*/ false,
							ctx);
				} else
					ft_set_parent(ft, iter, new_node_flag,
							slot);
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
	 * ADD/SAME/DEL mutators this is a LOCAL out-param re-published by the
	 * caller, so the forward is a bare store into that local; only
	 * FT_RECOMPACT_RELOCATE passes the LIVE slot (&ft->root, a parent child
	 * slot, &cn->child) and never re-publishes, so its store IS the
	 * reader-visible publication and is RECORDED into @rec to commit through a
	 * flip descriptor together with the SKIP_X dual above (keyed on the mode,
	 * not on @rec being set -- an ADD/SAME/DEL relocation may now carry a @rec
	 * for the SKIP_X dual alone while its forward stays local).  For the
	 * local-out-param callers the bare release store orders the node-body and
	 * metadata stores above it (a plain store would let a weakly-ordered
	 * architecture expose an unwired copy).
	 */
	if (mode == FT_RECOMPACT_RELOCATE) {
		/* SW compaction: *slot == plan old.  Relocating the ROOT node
		 * publishes into &ft->root (ft_compact_descend starts its walk
		 * there), so the holder slot must be asked. */
		struct cds_ft_inode_flag *holder_nf = NULL;

		/*
		 * The holder slot lives in @metadata's PARENT (§8.2: a node's
		 * body is its own), so that is the word that owns this edge --
		 * resolved rather than assumed, because a root relocation has no
		 * owning node and takes the always-MW route through @root.
		 */
		if (old_node_flag_ptr != &ft->root)
			(void) ft_resolve_parent_slot(metadata, ft, &holder_nf);
		ft_pub_rec_add(rec, old_node_flag_ptr, *old_node_flag_ptr,
			new_node_flag, old_node_flag_ptr == &ft->root,
			holder_nf ? ft_flag_to_metadata(ft, holder_nf) : NULL);
	}
	else
		*old_node_flag_ptr = new_node_flag;
	if (old_node && old_node_ret)
		*old_node_ret = old_node;


	/*
	 * The old node is RETIRED: the fresh copy is fully wired above and the
	 * caller publishes it into @old_node's slot and frees @old_node after a
	 * grace period.  Mark it DEAD (§4.B freeze-on-free, @metadata is the old
	 * node's metadata): a no-op store under one writer (nothing reads the
	 * bit; the arena re-zeroes metadata on reallocation), the freeze mark a
	 * concurrent MCAS writer validates under multi-writer.  Covers every
	 * recompact retire -- recompact-on-insert, delete shrink (DEL), and
	 * cds_ft_compact relocation.
	 *
	 * @retire_txn set: record the tombstone INTO the caller's commit txn so it
	 * flips atomically with the publish that unlinks @old_node (atomic detach,
	 * §4.B) instead of an early standalone flip; NULL keeps the lone-edge flip
	 * (a fresh build-invisible recompaction that never publishes @old_node, or
	 * a caller not yet routing its retire through a txn).
	 */
	if (old_node && metadata) {
		if (fenced) {
			/*
			 * Fenced retire: the tombstone's expected old is the
			 * MARK snapshot -- the commit ratifies exactly the
			 * state the copy was planned against -- and the fence
			 * hands its outcome to @retire_txn (commit OK consumes
			 * it via the {LOCK|s -> TOMBSTONE|s} transition;
			 * every other terminal outcome clears it through the
			 * wrapper's registry).
			 */
			if (!c_held.shared) {
				ft_flip_txn_lock_register(retire_txn,
					c_held.lock, c_held.lock_snap);
				ft_flip_txn_record_anchor_release(retire_txn,
					&c_held, metadata);
			}
			ft_flip_txn_record_retire_anchored(retire_txn, ctx,
					&c_held, metadata);
		} else if (retire_txn)
			ft_flip_txn_record_tombstone(retire_txn, metadata);
		else
			ft_meta_tombstone_set_flip(metadata);
	}
	/*
	 * §9.3 LOCK_FINE, the RELEASE half of the lock-set: {P} (+ {GP} when P is
	 * compressed) are EDITED, not retired, so their locks resolve through the
	 * other terminal -- {LOCK|s -> s}, dropped atomically with the publish
	 * that flips their slots.  Expected old is each member's MARK snapshot, so
	 * a peer state change on a locked node between the mark and the commit
	 * aborts this recompact rather than committing a plan derived from a world
	 * that moved (the same contract the retire half carries).
	 *
	 * Registering them hands the unlock to @retire_txn exactly like the retire
	 * half: commit OK consumes each lock through its recorded transition, and
	 * every other terminal (ABORT / MEMORY_ERROR / a caller's pre-commit bail)
	 * CAS-clears it through the registry.  Past this point the local bails
	 * below must NOT unlock them -- the txn owns them.
	 */
	for (ri = 0; ri < nr_rel; ri++) {
		if (rel_held[ri].shared)
			continue;
		ft_flip_txn_lock_register(retire_txn, rel_held[ri].lock,
			rel_held[ri].lock_snap);
		ft_flip_txn_record_release_lock(retire_txn, rel_held[ri].lock,
				rel_held[ri].lock_snap);
	}

	ret = 0;
end:
	return ret;

abandon_fresh:
	/*
	 * Latch bail (copied slot, DEL-target capture, or a fenced ADD's
	 * occupied byte): the fresh body never became peer-visible -- nothing
	 * recorded into @rec, forward slot untouched, the retire_txn arm's
	 * back-channel edge is a RECORD discarded with the abandoned attempt,
	 * and the plain-store arm's prev publish is deferred past this point.
	 * Reclaim the never-escaped copy immediately and lift the whole lock-set
	 * -- the retire half (@fenced) and the release half (@rel_held), neither
	 * yet registered with @retire_txn (registration happens only on the
	 * success path above); -EAGAIN re-descends after the peer settles.
	 */
	free_cds_ft_node_unpublished(ft, new_node);
	ft_unlock_held(rel_held, nr_rel);
	if (fenced && !c_held.shared)
		ft_meta_lock_release(c_held.lock);
	return ret;
}

/*
 * The metadata whose STATE WORD carries a child's parent_slot_offset -- the word
 * an SW re-parent pso edge parks and that ft_meta_nr_child_inc CASes.  NULL for
 * an external head (its back-edge is a plain parent pointer with no state word)
 * and for a flip proxy (a resolved child is never one).  Mirrors
 * ft_reparent_record's child-kind dispatch so the mark set matches the edges.
 */
static
struct cds_ft_metadata *ft_child_state_meta(struct cds_ft *ft,
		struct cds_ft_inode_flag *child_nf)
{
	if (!child_nf || ft_node_flip_proxy(child_nf))
		return NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf))
		return cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_skip_to_compressed(ft, child_nf));
	if (ft_node_compressed(child_nf))
		return cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(child_nf));
#endif
	if (ft_node_external(child_nf))
		return NULL;
	return cds_ft_item_to_metadata(ft_node_ptr(child_nf));
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
int ft_node_set_nth_rec(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,
		unsigned int node_depth,
		bool cluster_leaf,
		struct ft_pub_rec *rec,
		struct ft_flip_txn *retire_txn,
		const struct ft_parent_hint *inh_hint,
		const struct ft_lock_ctx *ctx,
		bool *deferred_count)
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
			child_node_flag, false, cluster_leaf, deferred_count);
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
					NULL, old_node_ret, false, node_depth, cluster_leaf,
					rec, retire_txn, inh_hint, ctx);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					NULL, old_node_ret, false, node_depth, cluster_leaf,
					rec, retire_txn, inh_hint, ctx);
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
 * The ordinary published-node set_nth: a build-invisible or non-relocating
 * publish with no SKIP_X-dual to fuse, so it passes @rec == NULL.  The live
 * one-commit insert reserve (ft_attach_node) calls ft_node_set_nth_rec directly
 * with its commit rec so a recompact-relocation's compressed-parent SKIP_X dual
 * flips atomically with the forward publish.
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
	/*
	 * No @retire_txn, so no lock-set is derived and no anchor is needed:
	 * this wrapper builds into a node whose retire nobody records.
	 */
	return ft_node_set_nth_rec(ft, node_flag, n, child_node_flag,
			old_node_ret, metadata, node_depth, cluster_leaf, NULL,
			NULL, NULL, NULL, NULL);
}

/*
 * Return 0 on success or negative error value on error.
 *
 * @node_flag_expected: the child value @node_flag_ptr held when the CALLER built
 * its plan -- the subtree this replace drops (delete) or displaces (external
 * promote).  Threaded to ft_node_recompact's DEL arm as its plan expected-old
 * (see its header); a peer republish of the slot since the plan is -EAGAIN.
 */
static
int ft_node_replace_ptr(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag_ptr,		/* Pointer to location to nullify */
		struct cds_ft_inode_flag *node_flag_expected,
		struct cds_ft_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,			/* of parent */
		uint8_t n,
		struct cds_ft_inode_flag *newptr,
		bool is_root,
		unsigned int node_depth,
		struct ft_remove_pub *pub,
		struct ft_flip_txn *retire_txn,
		const struct ft_parent_hint *held_hint,
		const struct ft_lock_ctx *ctx)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	/*
	 * @pub is NOT optional here, and the external-PROMOTE arm below depends
	 * on it: with @pub the promote's single reader-visible forward store is
	 * DEFERRED into it and committed after the §4.B acquire of the holder
	 * (ft-remove.h, `!boundary_fused && pub && pub->armed`); without it that
	 * arm stores into the LIVE holder immediately, i.e. mutates before
	 * acquiring, which is what the acquire exists to prevent -- a peer that
	 * recompacts the holder through its grandparent slot leaves the value
	 * intact in the retired copy, so the CAS still matches and the promoted
	 * head is published into a reclaimed node.
	 *
	 * The invariant holds structurally, not by convention: ft_detach_node is
	 * this family's ONLY caller, and it substitutes its own @local_pub for a
	 * NULL argument (ft-remove.h) before reaching here -- so a list-off
	 * caller passing NULL still arrives with @pub set.  Asserted rather than
	 * left implicit because the failure is silent: the pub-less arm compiles,
	 * runs, and corrupts only under a concurrent recompaction.
	 */
	assert(pub != NULL);

	dbg_printf("ft_node_replace_ptr for node %p, target ptr %p\n",
		ft_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ft_node_ptr(*parent_node_flag_ptr);
	type_index = ft_node_type(*parent_node_flag_ptr);
	type = &ft_types[type_index];
	ret = _ft_node_replace_ptr(ft, type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr, pub);
	if (ret == -EFBIG) {
		/*
		 * FOLD (@held_hint): a same-trie rekey folds this delete-recompaction
		 * of the src junction into a commit the folded graft also records
		 * into.  The hint carries the CALLER's junction-parent IDENTITY (and
		 * the recompacted node's slot in it) -- NOT a fresh resolve of this
		 * node's current parent, which would be a racy read: if a peer
		 * re-homed the node since the caller's descent, its current parent is
		 * NOT the node the caller planned against, and the recompaction would
		 * park an SW store into a slot that no longer holds it.  Either way
		 * the identity is VALIDATED by a C.parent == @held_hint->parent
		 * read-set guard riding the recompaction's own acquire commit, so a
		 * re-home aborts -> re-descend.  Two shapes:
		 *  - @parent_held: the graft already holds the lock that parent (the
		 *    two junctions share it), so this recompaction must REUSE the held
		 *    lock -- a second ft_dlm_lock would abort -EAGAIN -- and must not
		 *    record a second release.
		 *  - @parent_guard: the junctions do NOT share a parent, so this
		 *    recompaction acquires (and releases) it itself, guarded.
		 * NULL for every ordinary recompaction (it acquires + releases the
		 * parent itself and resolves it from the back-pointer).
		 */
		assert(!newptr);
		assert(!held_hint || held_hint->parent_held ||
			held_hint->parent_guard);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, node_flag_expected,
				old_node_ret, is_root, node_depth,
				false, NULL, retire_txn, held_hint, ctx);
	}
	if (ret == 0)
		FT_TP(tree_edge_set, (const void *) ft,
			(const void *) *parent_node_flag_ptr,
			(unsigned int) node_depth, (uint8_t) n,
			(const void *) newptr);
	return ret;
}
