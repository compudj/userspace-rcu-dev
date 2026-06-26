// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-merge.h
 *
 * Userspace RCU library - Fractal Trie: merge / merge_at (spine-copy, same-trie rekey).
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-merge.h is an implementation unit; #include it from fractal-trie.c only"
#endif

#ifdef FEATURE_FT_MERGE

/*
 * Read-only locate of a merge point at the end of @key.  Reuses
 * ft_graft_swap_descend (the same three outcomes, publishes nothing) and
 * additionally reports the cursor's compressed offset and the subtree's key
 * count, both needed by cds_ft_merge_at:
 *
 *   FT_GRAFT_SWAP_EXACT:       @d->nf is the live subtree at @key; @off_ret 0;
 *                              @count_ret its nr_keys (1 for an external / dup
 *                              chain -- one unique key).
 *   FT_GRAFT_SWAP_KEY_SHORTER: @key ends inside compressed @d->nf; @off_ret =
 *                              key_len - d->depth (bytes consumed into the
 *                              node); @count_ret the node's subtree nr_keys.
 *   FT_GRAFT_SWAP_DELEGATE:    no content under @key (src side -> the merge is
 *                              a no-op; dst side -> the atomic fast-path graft).
 *
 * @off_ret / @count_ret are 0 for DELEGATE.
 */
static
enum ft_graft_swap_case ft_merge_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d,
		unsigned int *off_ret, unsigned long *count_ret)
{
	enum ft_graft_swap_case kase = ft_graft_swap_descend(ft, key, key_len, d);

	switch (kase) {
	case FT_GRAFT_SWAP_KEY_SHORTER:
	{
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);

		*off_ret = (unsigned int) (key_len - d->depth);
		*count_ret = ft_nr_keys_get(
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn));
		break;
	}
	case FT_GRAFT_SWAP_EXACT:
		*off_ret = 0;
		if (ft_node_external(d->nf))
			*count_ret = 1;	/* one key (possibly a dup chain) */
		else
			*count_ret = ft_nr_keys_get(ft_flag_to_metadata(ft, d->nf));
		break;
	default:	/* FT_GRAFT_SWAP_DELEGATE */
		*off_ret = 0;
		*count_ret = 0;
		break;
	}
	return kase;
}

/*
 * cds_ft_merge_at build-invisible spine-copy.  ft_merge_build recursively
 * COPIES the overlapping spine of two subtrees S (from src) and D (from dst),
 * both rooted at the same suffix position, and REFERENCES every disjoint
 * subtree by pointer (recorded as a deferred re-parent edge applied at commit).
 * All allocation happens here, build-invisibly; OOM frees the fresh copies and
 * leaves both tries pristine -- no rollback.
 *
 * Returns the PLAIN flag of the freshly-built merged node (or, when the merged
 * position is leaf-only, the surviving external head), or FT_MERGE_OOM on
 * allocation failure (the caller aborts both glues).  On success *nr_keys_ret
 * holds the merged subtree's unique-key count.
 */
#define FT_MERGE_OOM		((struct cds_ft_inode_flag *) (long) -ENOMEM)

struct ft_merge_ctx {
	struct cds_ft *dst_ft;		/* all fresh merged nodes live here */
	struct ft_glue *gd;	/* dst cluster: built/deferred/dst frees/splices */
	struct ft_glue *gs;	/* src side: src-overlap frees (+ prune later) */
};

/* Upper-bound counters for the read-only pre-pass that sizes the glues. */
struct ft_merge_counts {
	int nb;		/* fresh built nodes (-> gd) */
	int nd;		/* deferred re-parent edges (-> gd) */
	int nf_dst;	/* dst-overlap frees (-> gd) */
	int nf_src;	/* src-overlap frees (-> gs) */
	int ns;		/* duplicate-chain splices (-> gd) */
};

/* nr_keys of a (possibly skip-encoded) child subtree referenced by pointer. */
static
unsigned long ft_merge_child_count(struct cds_ft *ft, struct cds_ft_inode_flag *c)
{
	if (ft_node_external(ft_resolve_skip_compressed(ft, c)))
		return 1;
	return ft_nr_keys_get(ft_flag_to_metadata(ft, c));
}

static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned long *nr_keys_ret);

/*
 * Advance a compressed cursor one byte past @next_off - 1: while still inside
 * the run report the run's flag at @next_off, else fall through to the run's
 * (internal/external, never compressed) child at offset 0.
 */
static inline
void ft_merge_advance(struct cds_ft_compressed_node *cn, unsigned int next_off,
		struct cds_ft_inode_flag **flag_ret, unsigned int *off_ret)
{
	if (next_off < cn->len) {
		*flag_ret = ft_compressed_node_flag(cn);
		*off_ret = next_off;
	} else {
		*flag_ret = cn->child;
		*off_ret = 0;
	}
}

/*
 * Materialize, build-invisibly, the subtree a single-side compressed run
 * contributes to a fresh branch slot: the bytes cn->key_bytes[off+1 .. len)
 * down to cn's (live) child.  The byte cn->key_bytes[off] is consumed by the
 * parent branch's slot, so this builds the part below it:
 *
 *   suffix_len == 0:                  the bare live child (no fresh node; the
 *                                     parent frame's Pass 2 wires it).
 *   suffix_len == 1 (non-skip):       a 1-child internal node.
 *   suffix_len >= 2, or == 1 (skip):  a fresh compressed node, returned in its
 *                                     skip-published form.
 *
 * Mirrors the old-direction suffix of ft_split_compressed_graft_build.  cn's
 * (live) child back-pointer is deferred into @c->gd with @dst_origin (false
 * for a src run, true for a dst run); a fresh wrapper is tracked there too.
 * @child_depth is the depth of the materialized node itself (parent + 1).
 * Returns the flag to store in the parent slot, or FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_materialize_suffix(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn, unsigned int off,
		unsigned int child_depth, bool dst_origin,
		unsigned long *ck_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct ft_glue *g = c->gd;	/* all fresh merged nodes -> gd */
	unsigned int suffix_len = cn->len - off - 1;
	unsigned long child_keys = ft_merge_child_count(ft, cn->child);
	struct cds_ft_inode_flag **slot;
	int ret;

	*ck_ret = child_keys;
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;
		struct cds_ft_inode_flag *plain;

		sfx = alloc_compressed_node(ft, (uint8_t) suffix_len, &sfx_meta);
		if (!sfx)
			return FT_MERGE_OOM;
		sfx->child = cn->child;
		sfx->len = (uint8_t) suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[off + 1], suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, child_keys, CMM_RELAXED);
		plain = ft_compressed_node_flag(sfx);
		ft_glue_track(g, plain);
		ft_glue_defer_edge_origin(ft, g, cn->child, plain, &sfx->child,
				dst_origin);
		/*
		 * Return the PLAIN flag: a fresh compressed node's skip form
		 * cannot be resolved during the build (its child's back-pointer
		 * is deferred), so the parent stores the plain form -- which
		 * ft_glue_is_fresh matches by direct identity -- and the
		 * parent's Pass 2 re-encodes the slot to the skip form.
		 */
		return plain;
	} else if (suffix_len == 1) {
		/* 1-child internal suffix (non-skip-compressed). */
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[off + 1],
				cn->child, NULL, NULL, child_depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)),
				child_keys, CMM_RELAXED);
		ft_glue_track(g, dest);
		ft_node_get_nth_skip(dest, &slot, cn->key_bytes[off + 1],
				FT_PF_NONE);
		ft_glue_defer_edge_origin(ft, g, cn->child, dest, slot,
				dst_origin);
		return dest;
	}
	/* suffix_len == 0: the bare child; parent Pass 2 wires its edge. */
	return cn->child;
}

/*
 * Both overlap sides are compressed runs sharing a @p (>= 1) byte prefix from
 * their cursors.  Emit ONE fresh compressed run for the shared bytes and
 * recurse on what follows each cursor (the next byte, or the run's child once
 * a run is exhausted).  The shared run's child is internal/external/another
 * fresh branch -- never another compressed -- because a canonical compressed
 * node's child is never compressed, so no two adjacent compresseds result.
 * Returns the run's PLAIN flag (parent Pass 2 re-encodes the slot to skip), or
 * FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_build_run(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn_s, unsigned int off_s,
		struct cds_ft_compressed_node *cn_d, unsigned int off_d,
		unsigned int p, unsigned int depth, unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_compressed_node *run;
	struct cds_ft_metadata *run_meta;
	struct cds_ft_inode_flag *adv_s, *adv_d, *child, *plain;
	unsigned int aoff_s, aoff_d;
	unsigned long ck = 0;

	ft_merge_advance(cn_s, off_s + p, &adv_s, &aoff_s);
	ft_merge_advance(cn_d, off_d + p, &adv_d, &aoff_d);
	child = ft_merge_build(c, adv_s, aoff_s, adv_d, aoff_d, depth + p, &ck);
	if (child == FT_MERGE_OOM)
		return child;

#ifndef FEATURE_FT_SKIP_COMPRESSED
	if (p == 1) {
		/*
		 * Non-skip-compressed: a 1-byte run is a 1-child internal node,
		 * not a len-1 compressed node (matching insert/split canonical
		 * form).  Same node+edge budget as the compressed run.
		 */
		struct cds_ft_inode_flag *dest = NULL, **slot;
		int ret;

		ret = ft_node_set_nth(ft, &dest, cn_s->key_bytes[off_s], child,
				NULL, NULL, depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)), ck,
				CMM_RELAXED);
		ft_glue_track(c->gd, dest);
		ft_node_get_nth_skip(dest, &slot, cn_s->key_bytes[off_s],
				FT_PF_NONE);
		ft_glue_defer_edge_origin(ft, c->gd, child, dest, slot,
				/*dst_origin=*/ true);
		*nr_keys_ret = ck;
		return dest;
	}
#endif
	run = alloc_compressed_node(ft, (uint8_t) p, &run_meta);
	if (!run) {
		/*
		 * The recursion's cluster is already tracked in the glue; the
		 * caller's abort frees it.  Only this frame's run failed.
		 */
		return FT_MERGE_OOM;
	}
	run->child = child;
	run->len = (uint8_t) p;
	memcpy(run->key_bytes, &cn_s->key_bytes[off_s], p);
	run_meta->nr_child = 1;
	ft_nr_keys_store(run_meta, ck, CMM_RELAXED);
	plain = ft_compressed_node_flag(run);
	ft_glue_track(c->gd, plain);
	/*
	 * Wire run->child's back-pointer.  A fresh recursion result is stored
	 * immediately (is_fresh); a live result can only be the dst splice head
	 * the recursion returns, so dst_origin is safe either way.
	 */
	ft_glue_defer_edge_origin(ft, c->gd, child, plain, &run->child,
			/*dst_origin=*/ true);
	/*
	 * Return the PLAIN flag (like ft_merge_materialize_suffix): the parent
	 * frame's Pass 2 re-encodes the holding slot to the skip form once the
	 * run is wired, and is_fresh can match it by identity in the meantime.
	 */
	*nr_keys_ret = ck;
	return plain;
}

static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_node *S_leaf, *D_leaf, *M_ext;
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_inode_flag *M = NULL;
	struct cds_ft_metadata *Mmeta = NULL;
	unsigned long total_keys = 0;
	bool tracked = false;
	unsigned int b, s_fb = 0, d_fb = 0;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);

	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/*
	 * Record each compressed overlap node's reclaim exactly once, on first
	 * entry (off == 0): a run may be re-entered at a deeper cursor by the
	 * shared-run recursion, but the node is freed whole.  src -> gs,
	 * dst -> gd.  Internal overlap nodes are recorded at the tail instead.
	 */
	if (S_comp && off_s == 0)
		ft_glue_defer_free(c->gs, cn_s, true);
	if (D_comp && off_d == 0)
		ft_glue_defer_free(c->gd, cn_d, true);

	/*
	 * Both compressed and sharing a prefix from their cursors -> collapse
	 * the shared bytes into one fresh run and recurse past it.  A zero-byte
	 * common prefix means the two runs diverge at the cursor: fall through
	 * to build a 2-way branch.
	 */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1)
			return ft_merge_build_run(c, cn_s, off_s, cn_d, off_d,
					p, depth, nr_keys_ret);
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);

	/*
	 * Both leaf-only -> the SAME full key terminates on both sides.
	 * Splice src after dst and return the (leaf-only) dst head; the
	 * parent frame wires the slot and the dst head's back-pointer.
	 */
	if (S_ext && D_ext) {
		ft_glue_record_splice(c->gd, D_leaf, S_leaf);
		*nr_keys_ret = 1;
		return D;
	}

	/*
	 * Otherwise build a fresh internal branch M (a compressed side
	 * contributes its single forced byte; never external_nodes, which a
	 * compressed node may not carry).  Pass 1: add every union child's
	 * forward slot (cluster_leaf: no child back-pointer is written here --
	 * a later set_nth may reallocate M).  Recurse on shared bytes;
	 * reference (internal) or materialize (compressed) one-side bytes.
	 */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL, *child;
		struct cds_ft_inode *old = NULL;
		bool sc_present, dc_present;
		unsigned long ck = 0;
		int ret;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;

		if (sc_present && dc_present) {
			/*
			 * Both present -> recurse.  At most one side is
			 * compressed here (two divergent runs have different
			 * forced bytes), so the other side's target is its
			 * internal child at offset 0.
			 */
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			child = ft_merge_build(c, ts, os, td, od, depth + 1, &ck);
			if (child == FT_MERGE_OOM)
				return child;
		} else if (sc_present) {
			if (S_comp) {
				child = ft_merge_materialize_suffix(c, cn_s,
						off_s, depth + 1,
						/*dst_origin=*/ false, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = sc;	/* reference live src subtree */
				ck = ft_merge_child_count(ft, sc);
			}
		} else {
			if (D_comp) {
				child = ft_merge_materialize_suffix(c, cn_d,
						off_d, depth + 1,
						/*dst_origin=*/ true, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = dc;	/* reference live dst subtree */
				ck = ft_merge_child_count(ft, dc);
			}
		}
		ret = ft_node_set_nth(ft, &M, (uint8_t) b, child, &old, Mmeta,
				depth, /*cluster_leaf*/ true);
		if (ret)
			return FT_MERGE_OOM;
		if (old) {
			ft_glue_untrack(ft, c->gd, old);
			free_cds_ft_node_unpublished(ft, old);
		}
		if (!tracked) {
			ft_glue_track(c->gd, M);
			tracked = true;
			Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
			/*
			 * Clear the recycled allocation's stale parent before
			 * any later set_nth reallocation copies it forward.
			 */
			rcu_assign_pointer(Mmeta->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			Mmeta->parent_slot_offset = 0;
#endif
		} else if (old) {
			ft_glue_track(c->gd, M);
		}
		Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
		total_keys += ck;
	}

	/* Merged external_nodes (the key terminating at M itself). */
	if (S_leaf && D_leaf) {
		M_ext = D_leaf;
		ft_glue_record_splice(c->gd, D_leaf, S_leaf);
	} else if (D_leaf) {
		M_ext = D_leaf;
	} else {
		M_ext = S_leaf;		/* may be NULL */
	}
	if (M_ext) {
		ft_metadata_set_external_nodes(M, Mmeta, M_ext);
		total_keys += 1;
	}

	/*
	 * Pass 2: wire every child's back-pointer (deferred) plus M's
	 * external back-channel.  Fetch slots only now -- the Pass-1 set_nth
	 * reallocations may have moved them.  A fresh (recursed/materialized)
	 * child stores its parent immediately via defer_edge's is_fresh fast
	 * path; a referenced live child truly defers to commit.
	 */
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag **slot;
		struct cds_ft_inode_flag *child =
			ft_node_get_nth_skip(M, &slot, (uint8_t) b, FT_PF_NONE);
		bool dst_origin;

		if (!child)
			continue;
		/*
		 * A referenced one-side child is dst-origin iff D still holds a
		 * child at this byte (a shared byte was recursed/materialized
		 * into a fresh cluster node, applied immediately by defer_edge's
		 * is_fresh path, so its origin is irrelevant).  dst-origin
		 * back-pointers are switched by the flip-latch after the forward
		 * publish, not by apply_deferred.
		 */
		if (D_comp)
			dst_origin = (b == d_fb);
		else
			dst_origin = !D_ext && ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE) != NULL;
		ft_glue_defer_edge_origin(ft, c->gd, child, M, slot, dst_origin);
		/*
		 * A freshly-built compressed child (run / suffix) was stored as
		 * its PLAIN flag so is_fresh could match it by identity above;
		 * now that its edge is wired, re-encode the slot to the canonical
		 * skip form.  Referenced compressed children are already stored
		 * skip-encoded (a skip flag is not ft_node_compressed).
		 */
		if (ft_node_compressed(child)) {
			struct cds_ft_inode_flag *skip = ft_publish_compressed(ft,
				ft_compressed_node_ptr(child), child);

			if (skip != child)
				rcu_assign_pointer(*slot, skip);
		}
	}
	if (M_ext)
		ft_glue_defer_edge_origin(ft, c->gd,
			(struct cds_ft_inode_flag *) M_ext, M, NULL,
			/*dst_origin=*/ D_leaf != NULL);

	/*
	 * Reclaim the INTERNAL overlap nodes M copied (compressed ones were
	 * recorded on entry above).  src -> gs, dst -> gd.
	 */
	if (!S_ext && !S_comp)
		ft_glue_defer_free(c->gs, ft_node_ptr(S), false);
	if (!D_ext && !D_comp)
		ft_glue_defer_free(c->gd, ft_node_ptr(D), false);

	ft_nr_keys_store(Mmeta, total_keys, CMM_RELAXED);
	*nr_keys_ret = total_keys;
	return M;
}

/*
 * Read-only pre-pass mirroring ft_merge_build's control flow, accumulating
 * upper bounds for ft_glue_reserve so the build never asserts on a full
 * inline floor.  No allocation, no mutation.
 */
static
void ft_merge_count(struct cds_ft *ft, struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		struct ft_merge_counts *cnt)
{
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_node *S_leaf, *D_leaf;
	unsigned int b, s_fb = 0, d_fb = 0;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);
	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/* Compressed overlap nodes are freed whole, recorded on first entry. */
	if (S_comp && off_s == 0)
		cnt->nf_src++;
	if (D_comp && off_d == 0)
		cnt->nf_dst++;

	/* Shared run: one fresh run node + its child edge, then recurse past. */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1) {
			struct cds_ft_inode_flag *as, *ad;
			unsigned int aos, aod;

			cnt->nb++;
			cnt->nd++;
			ft_merge_advance(cn_s, off_s + p, &as, &aos);
			ft_merge_advance(cn_d, off_d + p, &ad, &aod);
			ft_merge_count(ft, as, aos, ad, aod, cnt);
			return;
		}
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);
	if (S_ext && D_ext) {
		cnt->ns++;
		return;
	}
	cnt->nb++;		/* fresh branch M */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL;
		bool sc_present, dc_present;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;
		cnt->nd++;		/* M -> child back-pointer edge */
		if (sc_present && dc_present) {
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			ft_merge_count(ft, ts, os, td, od, cnt);
		} else if ((sc_present && S_comp) || (dc_present && D_comp)) {
			/* Materialized suffix; a fresh wrapper adds a node+edge. */
			struct cds_ft_compressed_node *cn = sc_present ? cn_s : cn_d;
			unsigned int off = sc_present ? off_s : off_d;
			unsigned int suffix_len = cn->len - off - 1;

			if (suffix_len >= 1) {
				cnt->nb++;
				cnt->nd++;
			}
		}
	}
	if (S_leaf && D_leaf) {
		cnt->ns++;
		cnt->nd++;
	} else if (S_leaf || D_leaf) {
		cnt->nd++;
	}
	if (!S_ext && !S_comp)
		cnt->nf_src++;
	if (!D_ext && !D_comp)
		cnt->nf_dst++;
}

/*
 * A surviving source head captured for the spine-copy interleave: its cell plus
 * the key SUFFIX below the src merge point (@suffix_off into the caller's packed
 * pool, @suffix_len bytes).  Captured BEFORE the src unlink, while S is still
 * attached and up-walkable; the merge below compares these against the LIVE dst
 * suffixes.  See ft_merge_ord_interleave_collect.
 */
struct ft_merge_src_cap {
	struct ft_ord_cell *cell;
	size_t suffix_off;
	size_t suffix_len;
};

/*
 * Compare two ordinal key suffixes.  Matches the ordered-list key ordering: a
 * shorter key that is a prefix of a longer one sorts FIRST (the prefix-key rule
 * ft_subtree_minmax_head relies on).  Returns <0 / 0 / >0.
 */
static inline
int ft_merge_suffix_cmp(const uint8_t *a, size_t la, const uint8_t *b, size_t lb)
{
	size_t m = la < lb ? la : lb;
	int c = m ? memcmp(a, b, m) : 0;

	if (c != 0)
		return c;
	if (la == lb)
		return 0;
	return la < lb ? -1 : 1;
}

/*
 * COLLECT the ordered-list edges that interleave the surviving source cells into
 * @dst's ordered list for a cds_ft_merge_at spine-copy, by a two-pointer
 * KEY-ORDER MERGE of the two already-sorted LIVE cell runs:
 *   - the dst merge region: @dst_first .. (exclusive) @dst_succ, walked via
 *     ord_next.  Its heads are up-walked LIVE (ft_rebuild_key_upwalk over the
 *     intact D, no proxy installed) and their key SUFFIX below the dst merge
 *     point is suffix = full_key[@dst_key_len ..].
 *   - the surviving src run: @src_caps[0 .. @nsrc), each carrying its
 *     pre-captured suffix below the src merge point (captured by the caller
 *     BEFORE ft_merge_unlink_src_subtree, while S was attached -- the detach
 *     leaves S-root's parent stale, so a post-unlink src up-walk is unsafe).
 * Both runs share the merge-point prefix, so they merge by comparing those
 * suffixes; an equal-suffix step is a COLLISION (the dst head wins -- its dup
 * chain absorbs the src head via ft_glue_apply_splices -- so the src head is
 * dropped, a floating duplicate never reachable as a distinct head).
 *
 * Runs in the txn's PREPARE phase, with NO structural proxy installed: the
 * merged ORDER is reconstructed from the two live runs rather than by walking
 * the about-to-be-published merged structure, so the collect needs neither the
 * staged proxies nor a writer-side merged-view resolution -- and there is no
 * append-after-install (every edge is recorded before install).  Pre-sets each surviving
 * cell's own links with plain stores (the cell is not ord-reachable in @dst --
 * never was -- and the caller already unlinked it from src), and accumulates
 * ONLY the <= 2*merged_keys+2 reader-VISIBLE boundary edges -- a dst-original
 * cell's ord_next / ord_prev, or @dst's head / tail -- into @edges, RETURNED as a
 * count for the caller to record into the structural flip txn so structure +
 * interleave commit in ONE flip.  @prev_placed seeds at the region predecessor
 * (@dst_first's ord_prev).
 *
 * Identity key_map only (matches the rest of the ordered-list machinery).
 */
static
unsigned int ft_merge_ord_interleave_collect(struct cds_ft *dst,
		size_t dst_key_len, struct ft_ord_cell *dst_first,
		struct ft_ord_cell *dst_succ, struct ft_ord_cell *prev_placed,
		const struct ft_merge_src_cap *src_caps, unsigned long nsrc,
		const uint8_t *src_pool, struct ft_ord_cell_edge *edges)
{
	size_t max_len = dst->group->max_key_len;
	uint8_t dbuf[FT_MAX_KEY_LEN];
	struct ft_ord_cell *dcur = dst_first;
	struct ft_ord_cell *prev = prev_placed;
	bool prev_is_dst = (prev_placed != NULL);
	unsigned long si = 0;
	unsigned int n = 0;
	const uint8_t *dsuf = NULL;
	size_t dsuf_len = 0;
	bool dsuf_valid = false;

	/*
	 * Two-pointer key-order merge of the dst region run and the src survivor
	 * run.  Each step emits the smaller-suffix head: a dst-original head stays
	 * put (its back edge changes only when a survivor run precedes it); a
	 * surviving src head splices in after the last placed cell.  A tie is a
	 * collision -> emit the dst head, drop the src head.  Dst-original cells
	 * keep their relative order, so a dst<->dst step needs no edge; each
	 * survivor RUN costs at most two edges (one entering, one leaving), so
	 * 2 * merged_keys + 2 bounds @edges.
	 */
	while (dcur != dst_succ || si < nsrc) {
		bool take_dst;

		if (dcur == dst_succ) {
			take_dst = false;		/* dst run exhausted */
		} else if (si == nsrc) {
			take_dst = true;		/* src run exhausted */
		} else {
			int cmp;

			if (!dsuf_valid) {
				size_t dfl = ft_rebuild_key_upwalk(dst, dcur,
						dbuf, max_len);

				assert(dfl >= dst_key_len);
				dsuf = dbuf + (max_len - dfl) + dst_key_len;
				dsuf_len = dfl - dst_key_len;
				dsuf_valid = true;
			}
			cmp = ft_merge_suffix_cmp(dsuf, dsuf_len,
					src_pool + src_caps[si].suffix_off,
					src_caps[si].suffix_len);
			/* Tie: dst head wins, drop the colliding src head. */
			if (cmp == 0)
				si++;
			take_dst = (cmp <= 0);
		}

		if (take_dst) {
			struct ft_ord_cell *cell = dcur;

			/*
			 * Dst-original cell: stays put, already linked in key
			 * order.  Its back edge changes only when a survivor run
			 * was just placed before it.
			 */
			if (prev && !prev_is_dst) {
				prev->ord_next = cell;	/* survivor: invisible */
				edges[n].slot = &cell->ord_prev;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&cell->ord_prev);
				edges[n].new_target = prev;
				n++;
			}
			prev = cell;
			prev_is_dst = true;
			dcur = ft_ord_cell_resolve_ord(&dcur->ord_next);
			dsuf_valid = false;
		} else {
			struct ft_ord_cell *cell = src_caps[si].cell;

			/* Surviving src cell: pre-set its back link. */
			cell->ord_prev = prev;		/* invisible */
			if (!prev) {
				/* new list minimum: flip @dst head. */
				edges[n].slot = &dst->ord_cell_head;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&dst->ord_cell_head);
				edges[n].new_target = cell;
				n++;
			} else if (prev_is_dst) {
				/* dst -> survivor: flip the dst cell's fwd edge. */
				edges[n].slot = &prev->ord_next;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&prev->ord_next);
				edges[n].new_target = cell;
				n++;
			} else {
				prev->ord_next = cell;	/* survivor: invisible */
			}
			prev = cell;
			prev_is_dst = false;
			si++;
		}
	}
	/*
	 * Close the trailing edge: if the last placed cell is a survivor, link it
	 * to the region successor (@dst_succ; NULL at the list tail) and flip that
	 * neighbour's back edge -- or @dst's tail when there is none.
	 */
	if (prev && !prev_is_dst) {
		prev->ord_next = dst_succ;	/* survivor: invisible */
		if (!dst_succ) {
			edges[n].slot = &dst->ord_cell_tail;
			edges[n].old_target =
				ft_ord_cell_resolve_ord(&dst->ord_cell_tail);
			edges[n].new_target = prev;
			n++;
		} else {
			edges[n].slot = &dst_succ->ord_prev;
			edges[n].old_target =
				ft_ord_cell_resolve_ord(&dst_succ->ord_prev);
			edges[n].new_target = prev;
			n++;
		}
	}
	return n;
}

/*
 * Unlink the EXACT subtree at @src_key from @src_ft IN PLACE, preserving the
 * subtree node so the spine-copy merge can keep referencing it (it is
 * re-parented into the merged cluster at commit).  This is the non-root-src
 * analogue of the root-src ft->root swap: it removes the merge source from
 * @src_ft and prunes the now-empty single-child branch above it.
 *
 * The descent + ft_detach_node + chain reclaim mirror ft_detach_keylen's
 * non-root path, but with @free_detached_subtree = false and WITHOUT wrapping
 * the subtree in a transient trie (the merge owns it via @gd's referenced
 * edges; wrapping it would double-own it).  @detached_count is the subtree's
 * unique-key count (from ft_merge_descend), propagated out of the ancestors.
 *
 * The caller invokes this as the LAST fallible commit step: on -ENOMEM
 * (ft_detach_node recompaction failed) @src_ft is left pristine, so the caller
 * aborts the still-invisible build with both tries intact -- no rollback.  On
 * success the caller drains @src_ft and runs the failure-free commit tail.
 */
static
int ft_merge_unlink_src_subtree(struct cds_ft *src_ft,
		const uint8_t *_src_key, size_t src_key_len,
		unsigned long detached_count, struct ft_detach_run *run)
{
	/*
	 * @src_key is ALREADY ORDINAL (cds_ft_merge_at converts once at its
	 * entry); a second key-map application here would descend a different
	 * subtree than the one the spine build copied.
	 */
	const uint8_t *key = _src_key, *ik;
	struct ft_descent d;
	int ret;

	ik = key;

	/*
	 * Plain key-guided descent to the merge source's subtree root.  As in
	 * ft_detach_keylen, no branch-point snapshot is tracked: ft_detach_node
	 * is bootstrapped from the target's own slot and climbs parent pointers
	 * to the surviving ancestor.  A KEY_SHORTER source (the key ends inside
	 * a compressed node) overshoots that node -- @d.nf becomes its child --
	 * and the climb's free walk reclaims the whole compressed node while
	 * preserving @d.nf, matching the old explicit chain reclaim.
	 */
	ft_descent_init(&d, src_ft);
	for (; d.depth < src_key_len; ) {
		uint8_t kv;

		/* Caller already established EXACT, so the path is present. */
		if (ft_node_compressed(d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d.nf);

			ft_descent_traverse_compressed(&d, cn, &ik);
			continue;
		}
		kv = *(ik++);
		ft_descent_step(src_ft, &d, kv);
	}

	/* Propagate the removal through ancestors before touching freed slots. */
	ft_propagate_external_count_parent(src_ft, d.pnf,
			-(long) detached_count);

	/*
	 * Unlink the branch in place, preserving the move target (@d.nf, the
	 * subtree root the spine-copy merge keeps referencing).  Bootstrapped
	 * from the target's own slot, ft_detach_node climbs to the surviving
	 * ancestor and its free-walk phase 1 reclaims the intermediate single-
	 * child chain (compressed / skip-target nodes included) while phase 2 --
	 * which would free the target -- stays gated off for move-style.
	 */
	{
		/*
		 * @run (EXCISE-ONLY, into==NULL) fuses the structural unlink with the
		 * run's removal from src's ordered list in ONE flip -- but the fusion
		 * in ft_detach_node is gated on a non-NULL @pub (the structural-publish
		 * deferral it records and commits alongside the cell edges), so supply
		 * one here exactly as ft_detach's move-style unlink does.  @run NULL
		 * (list off) leaves both NULL = the unfused two-store unlink.
		 */
		struct ft_remove_pub pub = { .armed = false };
		struct ft_remove_pub *pubp = run ? &pub : NULL;

		ret = ft_detach_node(src_ft, d.nfp, d.pnfp, d.depth,
				/*free_detached_subtree=*/ false, NULL, pubp, run);
	}
	if (ret < 0) {
		/* Recompaction OOM: undo the propagation; src is pristine. */
		ft_propagate_external_count_parent(src_ft, d.pnf,
				(long) detached_count);
		return -ENOMEM;
	}
	return 0;
}

/*
 * KEY_SHORTER dst: the merge point sits @off bytes inside a compressed dst
 * node whose prefix bytes @key[@d_depth .. @key_len) lie ABOVE it.  Wrap the
 * freshly-built merged node @M (with @mk unique keys) under that prefix and
 * return the PLAIN flag to publish into the old compressed node's forward slot
 * (the caller skip-encodes it), or FT_MERGE_OOM.
 *
 * @M internal: ft_build_branch lays a compressed prefix over the fresh
 * internal M -- M's edge into the prefix applies immediately via defer_edge's
 * is_fresh fast path, so ft_build_branch's dst_origin=false leaf edge is
 * correct, and M's own children keep the dst_origin ft_merge_build gave them.
 * A 1-child-no-external M is first canonicalized to compressed (its lone child
 * is always a FRESH recursion result, so the fold is dst_origin-safe too).
 *
 * @M compressed: M is a fresh run produced by ft_merge_build_run (the sole
 * path returning a compressed top), whose child may be a LIVE dst splice head
 * carrying dst_origin=true (a duplicate-key merge through the compressed node).
 * ft_build_branch's chain-merge would re-defer that child as dst_origin=false
 * and demote a live re-parent out of the flip latch, so fuse the prefix into M
 * here by hand, carrying the run child's dst_origin (always true).  The fused
 * length @off + M->len <= cn_d->len, so it is a valid compressed length.
 */
static
struct cds_ft_inode_flag *ft_merge_wrap_prefix(struct ft_merge_ctx *c,
		const uint8_t *key, unsigned int d_depth, unsigned int key_len,
		struct cds_ft_inode_flag *M, unsigned long mk)
{
	struct cds_ft *ft = c->dst_ft;
	unsigned int prefix_len = key_len - d_depth;

	if (ft_node_compressed(M)) {
		struct cds_ft_compressed_node *mcn = ft_compressed_node_ptr(M);
		unsigned int merged_len = prefix_len + mcn->len;
		struct cds_ft_compressed_node *merged;
		struct cds_ft_metadata *merged_meta;
		struct cds_ft_inode_flag *mflag;

		merged = alloc_compressed_node(ft, (uint8_t) merged_len,
				&merged_meta);
		if (!merged)
			return FT_MERGE_OOM;
		memcpy(merged->key_bytes, &key[d_depth], prefix_len);
		memcpy(&merged->key_bytes[prefix_len], mcn->key_bytes, mcn->len);
		merged->len = (uint8_t) merged_len;
		merged->child = mcn->child;
		merged_meta->nr_child = 1;
		ft_nr_keys_store(merged_meta, mk, CMM_RELAXED);
		mflag = ft_compressed_node_flag(merged);
		ft_glue_track(c->gd, mflag);
		/*
		 * Carry the run child's dst_origin (ft_merge_build_run records
		 * it true): a fresh child applies via is_fresh, a live splice
		 * head flips with the forward slot.  The defer de-dup supersedes
		 * M's stale &mcn->child edge with this one (same @child).
		 */
		ft_glue_defer_edge_origin(ft, c->gd, mcn->child, mflag,
				&merged->child, /*dst_origin=*/ true);
		ft_glue_untrack(ft, c->gd, mcn);
		free_compressed_node_unpublished(ft, mcn);
		return mflag;	/* PLAIN; caller skip-encodes before publish */
	} else {
		struct cds_ft_inode_flag *canon, *top;

		canon = ft_compress_single_child_if_needed(ft, M, c->gd);
		if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return FT_MERGE_OOM;
		top = ft_build_branch(ft, key, d_depth, key_len, canon, mk,
				/*has_external_nodes=*/ false, c->gd);
		if (!top)
			return FT_MERGE_OOM;
		return top;	/* PLAIN compressed; caller skip-encodes */
	}
}

/*
 * Keys in ORDINAL form (cds_ft_merge_at converts the caller's application
 * keys once at its entry; every consumer below -- the spine build, the source
 * unlink, the ordered-list interleave -- expects ordinal bytes).
 *
 * Build-invisible spine-copy merge of @src_ft's subtree at @src_key into
 * @dst_ft at the live EXACT subtree @d_dst.  All allocation is in the build
 * phase (plus the single fallible src unlink at commit for a non-root src), so
 * any OOM frees the fresh copies and leaves both tries pristine -- no rollback.
 *
 * @src_ft merged at @src_key (@d_src its subtree, @cnt_src its key count) into
 * @d_dst, a non-empty dst subtree.  A root src (src_key_len == 0) unlinks via
 * the ft->root swap; a non-root src unlinks its branch in place at commit (the
 * only post-build fallible step).  @off_src selects the src merge-point shape:
 * 0 is an EXACT subtree at @d_src->nf; > 0 is KEY_SHORTER (the src key ends
 * @off_src bytes inside the compressed node @d_src->nf -- the build enters that
 * node at the cursor, and ft_merge_unlink_src_subtree reclaims the whole node
 * via its chain-reclaim, the move-style detach preserving cn_s->child).  Both
 * root and non-root dst merge points are handled: a non-root point flips the
 * interior forward slot through a type-7 proxy that the read-side descent
 * resolves at every child fetch.  Compressed overlaps are handled.  @off_dst
 * selects the dst merge-point shape: 0 is an EXACT subtree at @d_dst->nf; > 0
 * is KEY_SHORTER (the dst key ends @off_dst bytes inside the compressed node
 * @d_dst->nf, whose prefix bytes are wrapped around the merged cluster by
 * ft_merge_wrap_prefix).
 *
 * Every (EXACT | KEY_SHORTER) src x (EXACT | KEY_SHORTER) dst shape is handled.
 *
 * Returns OK on a committed merge, or MEMORY_ERROR on OOM (both tries pristine).
 */
static
enum cds_ft_status ft_merge_spine_copy(struct cds_ft *dst_ft,
		struct cds_ft *src_ft, struct ft_descent *d_src,
		const uint8_t *src_key, size_t src_key_len, unsigned long cnt_src,
		unsigned int off_src, struct ft_descent *d_dst,
		unsigned long cnt_dst, unsigned int off_dst,
		size_t dst_key_len,
		struct urcu_flip_txn **pre_txn)
{
	struct ft_glue gd, gs;
	struct ft_merge_ctx ctx = { .dst_ft = dst_ft, .gd = &gd, .gs = &gs };
	struct ft_merge_counts cnt = { 0, 0, 0, 0, 0 };
	bool root_src = (src_key_len == 0);
	bool ks_dst = (off_dst > 0);
	bool ed;
	struct cds_ft_inode_flag *S = d_src->nf;
	struct cds_ft_inode_flag *D = d_dst->nf;
	struct cds_ft_inode_flag *M, *M_slot = NULL, *pub, *D_old;
	struct cds_ft_inode_flag *pub_parent = d_dst->pnf, **pub_slot = d_dst->nfp;
	struct cds_ft_inode *fresh_root = NULL;
	struct cds_ft_metadata *fresh_meta;
	struct urcu_flip_txn *txn;
	bool gp_owed;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL, *ms_succ = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_merge_src_cap *ms_src_caps = NULL;	/* src survivor suffixes, pre-captured */
	uint8_t *ms_src_pool = NULL;			/* packed suffix byte pool */
	unsigned long ms_nsrc = 0;			/* src run cells captured */
	unsigned int ms_cap = 0;	/* interleave edge cap, set once merged_keys is known */
	unsigned int ms_n = 0;		/* interleave edges collected (staged pre-commit) */

	/*
	 * Every dst merge-point shape is handled.  The flip proxies the publish
	 * slot @pub_slot, the read-side descent resolves the type-7 proxy at every
	 * child fetch (ft_resolve_flip_proxy, before the skip handler), and the
	 * published node's parent is wired to @pub_parent by
	 * ft_glue_set_publish, so a descent and an up-walk see a coherent
	 * old-XOR-merged view across the flip.
	 *
	 * Edge D: the merge point's PARENT is a COMPRESSED node cn_p reached via a
	 * grandparent skip slot.  cn_p's own parent cannot be compressed ("no two
	 * adjacent compressed" invariant), so the grandparent slot d_dst->pnfp is a
	 * plain internal slot that merely *holds* skip(cn_p).  Handle it one level
	 * up: the merge is EXACT at d_dst->nf (off_dst == 0), M is wrapped under a
	 * fresh copy of the WHOLE cn_p, and that copy is published into d_dst->pnfp
	 * in place of skip(cn_p) -- identical machinery to a KEY_SHORTER dst wrap,
	 * just with cn_p as the wrapped node and the grandparent as the publish
	 * point.  ks_dst and ed are mutually exclusive (a KEY_SHORTER merge point
	 * sits inside cn_d, whose parent is internal).
	 */
	ed = (d_dst->pnf &&
	      ft_node_compressed(ft_resolve_skip_compressed(dst_ft, d_dst->pnf)));

	/* Size both glues from a read-only pre-pass (with headroom). */
	ft_merge_count(dst_ft, S, off_src, D, off_dst, &cnt);
	ft_glue_init(&gd);
	ft_glue_init(&gs);
	if (ft_glue_reserve(&gd, cnt.nb + 8, cnt.nd + 8,
				cnt.nf_dst + 8, cnt.ns + 8) ||
	    ft_glue_reserve(&gs, 0, 0, cnt.nf_src + 8, 0)) {
		ft_glue_fini(&gd);
		ft_glue_fini(&gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* Root src: pre-allocate the fresh empty root for the commit swap. */
	if (root_src) {
		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_glue_fini(&gd);
			ft_glue_fini(&gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);
	}

	/* Build the merged cluster invisibly (the only build-phase fallible step). */
	M = ft_merge_build(&ctx, S, off_src, D, off_dst, 0, &merged_keys);
	if (M == FT_MERGE_OOM) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * The ordered-list interleave is folded into the structural flip (one
	 * commit for structure + ordered list), so its <= 2*merged_keys+2 cell
	 * edges share the structural flip batch.  Each survivor run costs at most
	 * two visible edges; bound it now that @merged_keys is known.
	 */
	if (ms_ord)
		ms_cap = 2u * (unsigned int) merged_keys + 2u;

	/*
	 * Compute @pub (the value to publish), @pub_parent / @pub_slot (where) and,
	 * for the compressed shapes, reclaim the replaced node.
	 *
	 * KEY_SHORTER dst (off_dst > 0): the merge point sits off_dst bytes inside
	 * the compressed node cn_d = D, whose prefix bytes lie above it; wrap M
	 * under cn_d->key_bytes[0..off_dst) and replace cn_d at its own slot
	 * (d_dst->nfp).  Edge D (ed): the merge point's parent is the compressed
	 * node cn_p; wrap M under the WHOLE cn_p and replace cn_p at the grandparent
	 * slot d_dst->pnfp (pub_parent = d_dst->ppnf).  Either way reclaim the
	 * replaced compressed node -- ft_merge_build entered it (or, for Edge D, did
	 * not touch it) without recording the free.  ft_merge_wrap_prefix
	 * canonicalizes a single-child M and preserves the dst_origin of a
	 * compressed M's child.
	 *
	 * EXACT dst with an internal/absent parent (off_dst == 0, !ed): M itself
	 * replaces the subtree.  Canonicalize a non-root single-child internal merge
	 * top: when S and D contribute exactly one shared byte (a churned "ab"-prefix
	 * shape), ft_merge_build returns M as a 1-child internal with no
	 * external_nodes -- forbidden at a non-root position under skip mode
	 * (chain-compress invariant; cds_ft_verify catches it at the merge depth).
	 * Collapse it to a 1-byte compressed via the glue.  Only the TOP M can hit
	 * this; its lone child is always a FRESH recursion result, so the deferred
	 * child edge carries dst_origin=false correctly and disturbs no flip edge.
	 * A root dst merge point (d_dst->pnf == NULL) is exempt: a 1-child internal
	 * is canonical at the root.
	 */
	if (ks_dst || ed) {
		struct cds_ft_compressed_node *wrap_cn;
		uint8_t kbuf[FT_MAX_KEY_LEN];
		unsigned int wrap_depth, wrap_len;

		if (ed) {
			wrap_cn = ft_compressed_node_ptr(
				ft_resolve_skip_compressed(dst_ft, d_dst->pnf));
			wrap_len = wrap_cn->len;
			wrap_depth = (unsigned int) d_dst->depth - wrap_len;
			pub_parent = d_dst->ppnf;
			pub_slot = d_dst->pnfp;
		} else {	/* ks_dst */
			wrap_cn = ft_compressed_node_ptr(D);
			wrap_len = off_dst;
			wrap_depth = (unsigned int) d_dst->depth;
		}
		ft_glue_defer_free(&gd, wrap_cn, true);
		memcpy(&kbuf[wrap_depth], wrap_cn->key_bytes, wrap_len);
		pub = ft_merge_wrap_prefix(&ctx, kbuf, wrap_depth,
				wrap_depth + wrap_len, M, merged_keys);
		if (pub == FT_MERGE_OOM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (pub_parent) {
		pub = ft_compress_single_child_if_needed(dst_ft, M, &gd);
		if (pub == (struct cds_ft_inode_flag *) (long) -ENOMEM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else {
		pub = M;
	}

	/*
	 * Take or create the flip-txn: one latch per dst-origin re-parent edge,
	 * one for the merge-point forward slot, plus @ms_cap for the ordered-list
	 * interleave's boundary edges -- structure and ordered list commit in ONE
	 * flip, so they share this txn.  Reserve it up front to that bound so every
	 * post-drain record (the structural edges and the INSTALLED-state cell
	 * edges, which append into the reserved head chunk) is allocation-free.
	 * The rekey hands in a pre-reserved txn (cannot fail); otherwise create one
	 * here, where failure aborts the still-invisible build (both tries pristine).
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		txn = ft_flip_txn_take(pre_txn);
		if (!txn) {
			txn = ft_flip_txn_create();
			if (txn && !urcu_flip_txn_reserve(txn,
					nr_dst + 1 + ms_cap)) {
				urcu_flip_txn_destroy(txn);
				txn = NULL;
			}
		}
	}
	if (!txn) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_glue_set_publish(dst_ft, &gd, pub_parent, pub_slot, pub);

	/*
	 * Slot-canonical form of @pub for the @pub_slot stores (both the flip
	 * proxy's new target and the settle): a compressed @pub is published
	 * SKIP-ENCODED, exactly as a direct slot write would store it, so a reader
	 * resolving the proxy gets a value identical in encoding to a normal slot
	 * read and runs the same skip handling.  set_publish wired @pub's parent +
	 * skip_slot via the plain flag already; an internal @pub (EXACT only -- a
	 * compressed-wrap @pub is always compressed) needs no re-encode.
	 *
	 * @D_old is the flip proxy's old target -- the value @pub_slot currently
	 * holds.  Read it from the live slot: for the compressed shapes the descent
	 * resolved d->nf / d->pnf to the PLAIN flag while the slot holds the SKIP
	 * form, and *pub_slot is untouched until the flip (the build is invisible
	 * and apply_deferred wires back-pointers, not this forward slot).
	 */
	if (ft_node_compressed(pub))
		M_slot = ft_publish_compressed(dst_ft,
				ft_compressed_node_ptr(pub), pub);
	else
		M_slot = pub;
	D_old = *pub_slot;

	/*
	 * Ordered list: capture the dst merge subtree's min head (the cursor for
	 * the post-commit interleave walk) and the region predecessor, while D is
	 * still intact (the build only copied its spine; the flip below moves its
	 * leaves into M).  The surviving src cells are spliced in after the commit.
	 */
	if (ms_ord) {
		ms_cursor = ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, false)->prev));
		ms_prev = ft_ord_cell_resolve_ord(&ms_cursor->ord_prev);
		/*
		 * The dst region run is [@ms_cursor .. D's max head]; @ms_succ is
		 * the cell following it (NULL at the list tail), the stop boundary
		 * for the interleave walk and the trailing survivor's successor.
		 */
		ms_succ = ft_ord_cell_resolve_ord(&ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, true)->prev))->ord_next);
		/*
		 * Capture src's merged-subtree (S) run endpoints now, while S is
		 * still intact, but defer the actual run-unlink until AFTER the
		 * last fallible step (ft_merge_unlink_src_subtree).  The run
		 * unlink is an externally observable ordered-list mutation; doing
		 * it here would leave src's list inconsistent if the src subtree
		 * unlink below OOMs and we roll the whole merge back.  The unlink
		 * still happens before the drain, so sync drains src ord-readers
		 * of the run too.
		 */
		ms_s_first = ft_subtree_minmax_head(dst_ft, S, false);
		ms_s_last = ft_subtree_minmax_head(dst_ft, S, true);
		/*
		 * Pre-allocate the interleave's edge scratch NOW, while the build
		 * is still abortable.  The interleave is collected pre-commit and
		 * its proxies recorded into the (already-reserved) structural @txn, so
		 * the commit tail has no allocation left and cannot degrade to a
		 * non-atomic per-edge fallback.  Each survivor run costs at most
		 * two visible edges, so @ms_cap (2*merged_keys+2) bounds @ms_edges.
		 */
		ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
		if (!ms_edges) {
			urcu_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/*
		 * Capture the src run's key SUFFIXES (below the src merge point)
		 * NOW, while S is still attached and up-walkable.  The post-commit
		 * interleave merges them against the LIVE dst suffixes to rebuild
		 * the merged order WITHOUT walking the about-to-be-published merged
		 * structure -- so no proxy need be installed before the collect.
		 * Capturing them here (not at the collect) is mandatory:
		 * ft_merge_unlink_src_subtree below leaves S-root's parent stale,
		 * so a src up-walk after the unlink could run into freed / relocated
		 * structure.  The run length is bounded by @cnt_src (the src
		 * subtree's key count >= its distinct heads); the variable-length
		 * suffix bytes pack into a realloc-growable pool addressed by offset
		 * (a byte pointer would dangle across the realloc).  Fallible
		 * pre-pass before the last fallible step -> any OOM aborts the
		 * still-invisible build, both tries pristine.
		 */
		ms_src_caps = malloc((size_t) (cnt_src + 8) * sizeof(*ms_src_caps));
		if (ms_src_caps) {
			struct ft_ord_cell *sc = ft_ord_cell_ptr(
				rcu_dereference(ms_s_first->prev));
			struct ft_ord_cell *slast = ft_ord_cell_ptr(
				rcu_dereference(ms_s_last->prev));
			size_t s_max_len = dst_ft->group->max_key_len;
			size_t pool_cap = 0, pool_len = 0;
			uint8_t sbuf[FT_MAX_KEY_LEN];
			bool oom = false;

			for (;;) {
				size_t sfl = ft_rebuild_key_upwalk(dst_ft, sc,
						sbuf, s_max_len);
				size_t suf_len;

				assert(sfl >= src_key_len &&
					ms_nsrc < cnt_src + 8);
				suf_len = sfl - src_key_len;
				if (pool_len + suf_len > pool_cap) {
					size_t ncap = pool_cap ? pool_cap * 2 : 256;
					uint8_t *np;

					while (ncap < pool_len + suf_len)
						ncap *= 2;
					np = realloc(ms_src_pool, ncap);
					if (!np) {
						oom = true;
						break;
					}
					ms_src_pool = np;
					pool_cap = ncap;
				}
				memcpy(ms_src_pool + pool_len,
					sbuf + (s_max_len - sfl) + src_key_len,
					suf_len);
				ms_src_caps[ms_nsrc].cell = sc;
				ms_src_caps[ms_nsrc].suffix_off = pool_len;
				ms_src_caps[ms_nsrc].suffix_len = suf_len;
				pool_len += suf_len;
				ms_nsrc++;
				if (sc == slast)
					break;
				sc = ft_ord_cell_resolve_ord(&sc->ord_next);
			}
			if (oom) {
				free(ms_src_pool);
				ms_src_pool = NULL;
				free(ms_src_caps);
				ms_src_caps = NULL;
			}
		}
		if (!ms_src_caps) {
			free(ms_edges);
			urcu_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * 1. Unlink the merge source from src.  Root src: swap in the pre-
	 *    allocated empty root.  Non-root src: detach its branch in place --
	 *    the LAST fallible step.  On its OOM both tries are pristine (the
	 *    detach self-undoes, the cluster is still unpublished), so abort the
	 *    build; this preserves the no-rollback property.  Then drain src
	 *    readers of the moved content.
	 *
	 *    ===== Everything from the drain onward is failure-free. =====
	 */
	struct ft_detach_run sdrun = { .into = NULL };

	if (ms_ord && !root_src) {
		/*
		 * Non-root src: fuse the src run-unlink into ft_merge_unlink_src_
		 * subtree's structural unlink flip (EXCISE-ONLY, into == NULL -- the
		 * run cells disperse to dst via the post-commit interleave), exactly
		 * as the subpos src side does, closing the src-side disappear window.
		 */
		sdrun.rfirst = ms_s_first;
		sdrun.rlast = ms_s_last;
	}
	/*
	 * Pre-reserve the src-side ordered-list commit txn before the (still
	 * fallible) src unlink.  Whichever fires -- the root-src root+endpoint
	 * swap or the rare unfused non-root run-unlink -- is the src side's
	 * commit, un-abortable once the structural unlink is public, so it commits
	 * through this pre-reserved txn (ft_ord_cell_flip_into).  Sized to the
	 * larger bound (4-edge run-unlink >= 3-edge root swap).  OOM here aborts
	 * the still-invisible build (both tries pristine).  The lone-edge list-off
	 * paths (ft_root_edge_flip) need no txn, so reserve only when ms_ord.
	 */
	struct urcu_flip_txn *src_side_txn = NULL;

	if (ms_ord) {
		src_side_txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
		if (!src_side_txn) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			urcu_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}
	if (root_src) {
		/*
		 * A root src moves the WHOLE source, so its run is the whole src
		 * ordered list: fuse the src->root swap with the head/tail clear into
		 * ONE flip (ft_root_list_swap_publish), so a src reader never sees src
		 * structurally empty but its ordered-list front still populated.  The
		 * run cells keep their internal links (only the head/tail endpoints
		 * flip), so the post-commit interleave still re-homes them to dst.
		 */
		if (ms_ord) {
			ft_root_list_swap_publish(src_ft, src_side_txn,
				&src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0),
				src_ft->ord_cell_head, NULL,
				src_ft->ord_cell_tail, NULL);
			src_side_txn = NULL;	/* consumed */
		} else {
			/*
			 * No ordered list: src->root is the only reader-visible slot.
			 * Express the lone root edge as a single-edge flip descriptor
			 * (one release store, like a bare rcu_assign_pointer) so the
			 * swap is MCAS-expressible like the list-on path.
			 */
			ft_root_edge_flip(src_ft, &src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0));
		}
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src, ms_ord ? &sdrun : NULL) < 0) {
		/*
		 * OOM in the last fallible step: @src_ft is left pristine (the run was
		 * not yet applied -- it commits in ft_detach_node's flip, past the
		 * fallible alloc), so abort the still-invisible build.
		 */
		free(ms_src_pool);
		free(ms_src_caps);
		free(ms_edges);
		urcu_flip_txn_destroy(txn);
		if (src_side_txn)
			urcu_flip_txn_destroy(src_side_txn);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.  Skipped
	 * for a root src (fused into the root swap above) and for a non-root src
	 * whose unlink already fused the run (sdrun.armed).
	 */
	if (ms_ord && !root_src && !sdrun.armed) {
		ft_ord_cell_run_unlink(src_ft, src_side_txn, ms_s_first,
			ms_s_last);
		src_side_txn = NULL;	/* consumed */
	}
	/* Reserved but unused: a non-root run whose unlink already fused it. */
	if (src_side_txn)
		urcu_flip_txn_destroy(src_side_txn);
	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	/*
	 * 2. Re-parent the SRC-origin referenced subtrees directly: the src
	 *    drain above made them unreachable to readers, and the cluster is
	 *    not yet forward-published, so this is invisible.  dst-origin
	 *    edges are NOT applied here -- they go through the flip.
	 */
	ft_glue_apply_deferred(dst_ft, &gd);

	/*
	 * 3. Record the structural edges into @txn (still PREPARE): every
	 *    dst-origin child's parent re-parent and the merge-point forward slot.
	 *    ft_glue_record_back_edge mirrors ft_set_parent's child-kind dispatch
	 *    (reading the field's current value as the old target) and runs the
	 *    writer-only slot bookkeeping early; the commit's settle below writes
	 *    each field to its new parent, subsuming the old step-7 ft_set_parent.
	 *    Nothing is parked yet -- commit's auto-install stages every proxy
	 *    (selector 0, resolving to old) so up-walks and the root descent still
	 *    see the pre-merge dst until the flip.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			if (!gd.deferred[j].dst_origin)
				continue;
			ft_glue_record_back_edge(dst_ft, txn,
				gd.deferred[j].child, gd.deferred[j].parent,
				gd.deferred[j].slot);
		}
		/* Forward publish: old dst subtree -> merged cluster. */
		ft_flip_txn_record_reserved(txn, (void **) pub_slot,
			D_old, M_slot);
	}

	/*
	 * 3b. Ordered list: collect the interleave and record its cell edges into
	 *    the SAME txn, STILL IN PREPARE -- no install before the collect.
	 *    The merged order is reconstructed by a two-pointer merge of the two
	 *    LIVE cell runs (the dst region up-walked over the intact D, and the
	 *    src run's suffixes captured pre-unlink), so the collect does NOT need
	 *    the about-to-be-published merged structure -- the last
	 *    append-after-install is gone, and every edge is recorded before
	 *    install.  Structure + ordered list thus commit in ONE flip (commit
	 *    auto-installs every recorded proxy, then flips): a reader never sees
	 *    the merged structural minimum ahead of the ordered-list front.  The
	 *    collect pre-sets the surviving cells' own links invisibly (not yet
	 *    ord-reachable) and returns only the visible boundary edges.  It runs
	 *    before apply_splices (step 5), but collisions are invariant: a collided
	 *    src head is a floating duplicate (only in the splice record), never a
	 *    distinct reachable head, so the merge enumerates the same heads either way.
	 */
	if (ms_ord) {
		unsigned int i;

		ms_n = ft_merge_ord_interleave_collect(dst_ft, dst_key_len,
			ms_cursor, ms_succ, ms_prev, ms_src_caps, ms_nsrc,
			ms_src_pool, ms_edges);
		for (i = 0; i < ms_n; i++)
			ft_flip_txn_record_reserved(txn,
				(void **) ms_edges[i].slot,
				(void *) ms_edges[i].old_target,
				(void *) ms_edges[i].new_target);
	}

	/*
	 * 4. Commit: one selector flip switches every dst-origin parent, the
	 *    forward slot, AND every interleave cell edge from old to merged,
	 *    atomically, then settles each slot to its direct merged target.  (List
	 *    off with no dst-origin re-parent reduces to a single release store of
	 *    the forward slot -- no proxy, no grace period.)  Because the forward
	 *    slot flips with the back-pointers, a reader (descend then up-walk) only
	 *    progresses old->merged; and the ordered-list front advances in the same
	 *    instant the merged minimum becomes reachable.
	 */
	gp_owed = urcu_flip_txn_commit(txn);

	/* 5. Concatenate same-key duplicate chains (dst now reachable via M). */
	ft_glue_apply_splices(dst_ft, &gd);

	/*
	 * 6. Propagate the dst key-count delta through the ancestors, starting at
	 *    @pub_parent (the parent of the replaced node -- d_dst->pnf for an
	 *    EXACT / KEY_SHORTER point, the grandparent d_dst->ppnf for Edge D).
	 *    @pub is a fresh node already carrying merged_keys, so the walk must
	 *    begin one level up; @pub_parent is a plain internal flag (cn_p's
	 *    parent cannot be compressed), safe for ft_node_ptr.
	 */
	if (pub_parent && merged_keys != cnt_dst)
		ft_propagate_external_count_parent(dst_ft, pub_parent,
			(long) merged_keys - (long) cnt_dst);

	/*
	 * 7. Reclaim, all deferred: the flip-txn (commit settled every proxied slot
	 *    -- the dst-origin parents, the forward slot and the interleave cell
	 *    edges -- to its direct merged target, so the proxies are unreferenced;
	 *    @gp_owed is false only when the commit reduced to a single bare store)
	 *    and the old overlap spines (src-side to src, dst-side to dst).  No dst
	 *    synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_flip_txn_reclaim(dst_ft, txn, gp_owed);
	ft_glue_free_old(src_ft, &gs);
	ft_glue_free_old(dst_ft, &gd);

	/*
	 * 8. Free the collided (demoted) src heads' cells.  The interleave (folded
	 * into the flip above) already rewired the surviving cells' stale src-run
	 * links away from these cells, so they are unreachable to new readers, and
	 * the grace-period defer inside the free covers readers already holding such
	 * a link or parked on a demoted head.
	 */
	if (ms_ord) {
		free(ms_edges);
		free(ms_src_caps);
		free(ms_src_pool);
	}
	ft_glue_free_collided_cells(dst_ft, &gd);

	ft_glue_fini(&gd);
	ft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}

/*
 * Build-invisible graft of a SUB-position source subtree into a dst position
 * ABSENT at @dst_key (the diverged-dst case) WITHOUT re-rooting the payload --
 * the leak-free reorder for cds_ft_merge_at's residual shapes.  Modeled on
 * ft_graft_keylen's commit, but the source side is ft_merge_spine_copy's: the
 * payload stays @payload (= d_src->nf, referenced in place, not detached into a
 * fresh root), and ft_merge_unlink_src_subtree is the single last fallible step
 * (it self-undoes on OOM, preserving @payload).  Because the dst attach is built
 * invisibly BEFORE that unlink and published failure-free AFTER the drain,
 * nothing can strand the moved externals on an allocation failure -- no
 * rollback, hence no leak (contrast the legacy detach-then-graft fallback).
 *
 * Covers both diverged-dst shapes: a GLUE diverge (the dst key diverges inside a
 * compressed node, so ft_graft_build assembles the whole split cluster invisibly
 * -- no reserve) and a NOSPLIT point (an empty slot at @dst_key, or a built
 * branch / displaced external -- ft_store_at_graft_point grafts it post-drain
 * drawing from a pre-filled reserve so it cannot fail on the arena).  *handled is
 * always set true here, so the caller never falls through.
 *
 * @okey_dst / @okey_src are ORDINAL; @dst_key is the APPLICATION dst key (the
 * ordered-list splice-pos descent remaps it).  @payload is the source subtree to
 * move with @cnt_src unique keys: d_src->nf for an EXACT source, or, for a
 * KEY_SHORTER source, cn_s->child with @okey_dst / @dst_key / @dst_key_len
 * already EXTENDED by the residual cn_s bytes (the caller's reduction to EXACT).
 * @okey_src / @src_key_len remain the ORIGINAL source key (the unlink overshoots
 * cn_s on its own).
 */
static
enum cds_ft_status ft_merge_graft_subpos_inplace(struct cds_ft *dst_ft,
		struct cds_ft *src_ft,
		const uint8_t *okey_dst, const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *okey_src, size_t src_key_len,
		struct cds_ft_inode_flag *payload, unsigned long cnt_src,
		bool *handled)
{
	struct ft_glue glue;
	struct cds_ft_alloc_reserve reserve;
	struct ft_descent d;
	enum ft_graft_prep prep;
	struct cds_ft_inode_flag *attached_nf, *aparent;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *pred = NULL, *succ = NULL;
	struct ft_ord_cell *run_first = NULL, *run_last = NULL;
	struct cds_ft_node *s_first = NULL, *s_last = NULL;
	struct ft_graft_run mrun;
	struct ft_graft_run *run_arg = NULL;
	struct ft_detach_run srun = { .into = NULL, .armed = false };
	struct ft_detach_run *srunp = NULL;
	size_t src_max, nm, dm;

	*handled = false;

	/*
	 * Classify the dst graft point with a build-invisible prep.  A GLUE
	 * diverge builds the whole split cluster into @glue (referencing @payload
	 * by deferred edge), published failure-free after the drain.  A NOSPLIT
	 * point is grafted post-drain by ft_store_at_graft_point drawing from a
	 * pre-filled reserve, so it likewise cannot fail on an arena allocation.
	 * Either way the source unlink is the single last fallible step, so an OOM
	 * leaves both tries pristine -- no rollback, no leak.
	 */
	ft_glue_init(&glue);
	memset(&reserve, 0, sizeof(reserve));
	/*
	 * Every attach shape -- GLUE diverge and the NOSPLIT in-place store --
	 * commits through @glue.txn (exactly like cds_ft_graft): the build (below)
	 * tags its displaced old child dst_origin, and the NOSPLIT store reserves
	 * its slot proxy + run-splice edges, all out of this txn.  Create it BEFORE
	 * the build; destroyed on a POPULATED point (nothing to commit).
	 */
	glue.txn = ft_flip_txn_create();
	if (!glue.txn || !urcu_flip_txn_reserve(glue.txn,
			FT_GLUE_FLOOR_DEFERRED + 6)) {
		if (glue.txn)
			urcu_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	prep = ft_graft_build(dst_ft, okey_dst, dst_key_len, payload, cnt_src,
			&d, &glue);
	*handled = true;
	if (prep == FT_GRAFT_PREP_OOM) {
		ft_glue_abort(dst_ft, &glue);
		urcu_flip_txn_destroy(glue.txn);
		return CDS_FT_STATUS_MEMORY_ERROR;	/* both tries pristine */
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/* Defensive: cnt_dst == 0 should never yield an occupied point. */
		urcu_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		return CDS_FT_STATUS_POPULATED_ERROR;
	}
	if (prep == FT_GRAFT_PREP_NOSPLIT) {
		/*
		 * Pre-fill the store's node reserve while both tries are still
		 * pristine.  The store's slot proxy + run-splice edges draw from the
		 * already-reserved @glue.txn, so no separate flip batch is needed.
		 * Securing the reserve here -- before the source unlink -- makes the
		 * post-drain store wholly failure-free, so the unlink is the true last
		 * fallible step and nothing strands.
		 *
		 * Use the GENEROUS superset fill (as ft_graft_keylen's NOSPLIT attach
		 * does for the same ft_store_at_graft_point), not a hand-rolled exact
		 * manifest: predicting the store's node set is fragile (e.g. attaching
		 * a high byte forces a RANGE recompact of the attach node even within
		 * its child capacity -- a grow an exact count heuristic misses, which
		 * underflowed the reserve and aborted).  A bulk op already pays a grace
		 * period, so the throwaway pops/pushes are negligible.
		 */
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			urcu_flip_txn_destroy(glue.txn);
			ft_glue_fini(&glue);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}
	/* GLUE: cluster built invisibly.  NOSPLIT: reserve secured (+ glue.txn). */

	/*
	 * Ordered list: locate the dst splice neighbours while dst is still
	 * payload-free, and capture the source subtree's run endpoints + cells
	 * while the subtree is still intact in src.
	 */
	if (ms_ord) {
		ft_ord_cell_find_splice_pos(dst_ft, dst_key, dst_key_len,
			&pred, &succ);
		s_first = ft_subtree_minmax_head(src_ft, payload, false);
		s_last = ft_subtree_minmax_head(src_ft, payload, true);
		run_first = ft_ord_cell_ptr(rcu_dereference(s_first->prev));
		run_last = ft_ord_cell_ptr(rcu_dereference(s_last->prev));
		mrun.run_first = run_first;
		mrun.run_last = run_last;
		mrun.pred = pred;
		mrun.succ = succ;
		mrun.armed = false;
		run_arg = &mrun;
		/* Src side: EXCISE-ONLY run so the structural unlink and the run's
		 * removal from src's ordered list commit in ONE flip (the
		 * disappear-side cross-view fix). */
		srun.rfirst = s_first;
		srun.rlast = s_last;
		srunp = &srun;
	}

	/*
	 * Pre-reserve the standalone run-unlink txn before the fallible unlink:
	 * the rare unfused shape removes the src run from src's list AFTER the
	 * structural unlink is public (un-abortable).  OOM here aborts the still-
	 * invisible build (both tries pristine).  Reserve only when ms_ord.
	 */
	struct urcu_flip_txn *run_unlink_txn = NULL;
	/*
	 * Pre-reserve the standalone dst run-splice txn too: the rare unfused
	 * attach shape splices the moved run into dst's list AFTER the structural
	 * attach is public (un-abortable).  Reserved only when ms_ord; consumed by
	 * the standalone splice or freed-unused when fused (the common case).
	 */
	struct urcu_flip_txn *run_splice_txn = NULL;

	if (ms_ord) {
		run_unlink_txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
		run_splice_txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_RUN_SPLICE_MAX_EDGES);
		if (!run_unlink_txn || !run_splice_txn) {
			if (run_unlink_txn)
				urcu_flip_txn_destroy(run_unlink_txn);
			if (run_splice_txn)
				urcu_flip_txn_destroy(run_splice_txn);
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_abort(dst_ft, &glue);
			urcu_flip_txn_destroy(glue.txn);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * Last fallible step: unlink the source subtree in place, preserving
	 * @payload.  On OOM the unlink self-undoes (src pristine) and the still-
	 * invisible cluster / reserve is released (dst pristine) -- no rollback.
	 */
	if (ft_merge_unlink_src_subtree(src_ft, okey_src, src_key_len,
			cnt_src, srunp) < 0) {
		if (run_unlink_txn)
			urcu_flip_txn_destroy(run_unlink_txn);
		if (run_splice_txn)
			urcu_flip_txn_destroy(run_splice_txn);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_glue_abort(dst_ft, &glue);
		urcu_flip_txn_destroy(glue.txn);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* ===== Everything from here on is failure-free. ===== */

	/*
	 * Remove the source run from src's ordered list (the cells keep their
	 * internal links for the splice into dst below) -- BEFORE the drain, so sync
	 * drains src ord-readers of the run too.  The structural unlink above FUSED
	 * this into its flip (@srun.armed), so a src reader never sees the run gone
	 * from the structure but still in the list; the standalone two-commit unlink
	 * remains as a defensive fallback for any unfused unlink shape.
	 */
	if (ms_ord && !srun.armed) {
		ft_ord_cell_run_unlink(src_ft, run_unlink_txn, s_first, s_last);
		run_unlink_txn = NULL;	/* consumed */
	}
	/* Reserved but unused: the structural unlink already fused the run. */
	if (run_unlink_txn)
		urcu_flip_txn_destroy(run_unlink_txn);

	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	/*
	 * An EXTERNAL payload is re-parented directly under a new internal slot, so
	 * its edge byte changes (src_key's last byte -> dst_key's).  That byte lives
	 * in the head's CELL metadata (ft_rebuild_key_upwalk reads it for the
	 * ordered key rebuild) and ft_set_parent does NOT maintain it for externals.
	 * Stamp it HERE -- AFTER the run leaves src's list (run_unlink) and the sync
	 * drains any src reader mid-run, but BEFORE the store FUSES the run-splice
	 * and publishes the cell into dst: the cell is in NEITHER list and
	 * structurally invisible, so no reader rebuilds a key from it during the
	 * stamp.  Stamping it at capture (while still in src's list) would let a src
	 * reader rematerialize an out-of-namespace key -- a cross-view escape.  An
	 * internal / compressed payload keeps every leaf's edge byte (the subtree
	 * moves wholesale), so no per-leaf fix-up is needed.
	 */
	if (ms_ord && ft_node_external(payload))
		cds_ft_item_to_metadata(run_first)->incoming_byte =
			okey_dst[dst_key_len - 1];

	if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * Failure-free commit of the build-invisible diverge cluster: the
		 * displaced old child is LIVE (still reachable through the compressed
		 * node being split until the forward publish), so via @glue.txn it
		 * flips atomically with the forward publish and the ordered-list
		 * run-splice -- the whole attach observed old XOR new.  The payload's
		 * hidden back-pointers are wired immediately inside the commit.  Then
		 * reclaim the replaced compressed node.
		 */
		ft_glue_txn_commit(dst_ft, &glue, run_arg);
		attached_nf = glue.attached_nf;
		ft_glue_free_old(dst_ft, &glue);
		ft_glue_fini(&glue);
	} else {
		/*
		 * NOSPLIT: graft the in-place payload at @d.  Every node allocation
		 * draws from the reserve and the slot proxy + run-splice edges draw
		 * from the pre-reserved @glue.txn, so the store has no fallible step
		 * left -- it cannot fail.
		 */
		unsigned int adepth = 0;
		enum cds_ft_status st;

		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		st = ft_store_at_graft_point(dst_ft, okey_dst, dst_key_len, &d,
				payload, cnt_src, &attached_nf, &adepth, &glue,
				run_arg);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		assert(st == CDS_FT_STATUS_OK);
		(void) st;
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_glue_fini(&glue);
	}

	/*
	 * Propagate the moved key count up dst's ancestor chain, starting from
	 * @attached_nf's parent (its own nr_keys already carries @cnt_src).
	 * ft_get_parent_rcu handles every payload node type (external / compressed
	 * / internal) and resolves the merge-point flip proxy.
	 */
	aparent = ft_get_parent_rcu(dst_ft,
		ft_resolve_skip_compressed(dst_ft, attached_nf));
	if (aparent)
		ft_propagate_external_count_parent(dst_ft, aparent, (long) cnt_src);

	/*
	 * Ordered list: every attach shape (GLUE, displaced-external, in-place
	 * slot) FUSED the run-splice into its structural flip (@armed) -- the
	 * external edge-byte was stamped before that splice, above.  The standalone
	 * two-commit splice remains as a defensive fallback for any not-yet-fused
	 * shape (none today); without it an unfused shape would strand the moved
	 * run out of the ordered list.  It commits through the pre-reserved
	 * @run_splice_txn (un-abortable post-drain); the fused common case frees
	 * that txn unused.
	 */
	if (ms_ord && !mrun.armed) {
		ft_ord_cell_run_splice(dst_ft, run_splice_txn, run_first,
			run_last, pred, succ);
		run_splice_txn = NULL;	/* consumed */
	}
	if (run_splice_txn)
		urcu_flip_txn_destroy(run_splice_txn);	/* fused: unused */

	/* Raise dst's max_used_key_len for the moved keys (dst_key || suffix). */
	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	nm = src_max > src_key_len ? dst_key_len + (src_max - src_key_len) :
		dst_key_len;
	dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
	if (nm > dm)
		uatomic_store(&dst_ft->max_used_key_len, nm, CMM_RELAXED);

	return CDS_FT_STATUS_OK;
}

/*
 * @pre_txn carries a flip-txn the caller reserved before its own last fallible
 * step, so the spine-copy / graft commit below draws an unfailable txn instead
 * of allocating one.  NULL on the public path (cds_ft_merge_at), set only by the
 * same-trie rekey, which pre-reserves it (sized from the O(1) subtree key counts:
 * the structural re-parent + folded ordered-list interleave, or the graft
 * cluster floor) so its post-detach merge cannot fail -- no reader-observable
 * rollback.  The consume site NULLs the slot it takes, so the rekey frees the
 * txn only when a given merge shape left it unused.
 */
static enum cds_ft_status ft_merge_at_inner(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len,
		struct urcu_flip_txn **pre_txn)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;
	struct ft_descent d_src, d_dst;
	unsigned int off_src, off_dst;
	unsigned long cnt_src, cnt_dst;
	enum ft_graft_swap_case ks, kd;
	uint8_t okey_dst_buf[FT_MAX_KEY_LEN], okey_src_buf[FT_MAX_KEY_LEN];
	const uint8_t *okey_dst = dst_key, *okey_src = src_key;

	FT_TP(merge_enter, (const void *) dst_ft, (const void *) src_ft);

	if (!dst_ft || !src_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != src_ft->group) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_key_len > dst_ft->group->max_key_len ||
			src_key_len > dst_ft->group->max_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if ((dst_key_len > 0 && !dst_key) ||
			(src_key_len > 0 && !src_key)) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Fixed-length groups: each source key K has length fixed_len,
	 * the moved subtree's stripped keys have length
	 * (fixed_len - src_key_len), and the resulting destination key
	 * is dst_key || stripped, of length
	 * (dst_key_len + fixed_len - src_key_len).  For that result to
	 * equal fixed_len (the only key length the destination group
	 * accepts), src_key_len and dst_key_len must be equal.
	 */
	if (dst_ft->group->key_len != CDS_FT_LEN_VARIABLE
			&& dst_key_len != src_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Combined-length overflow validation (mirrors cds_ft_graft): a moved
	 * key K becomes dst_key || (K - src_key prefix), of length
	 * dst_key_len + len(K) - src_key_len.  Bound len(K) by the source's
	 * max_used_key_len; without this check a variable-length merge with
	 * dst_key_len > src_key_len could create keys exceeding the group's
	 * max_key_len, overflowing the fixed-size key buffers downstream
	 * (the spine's compressed-wrap kbuf, the iterator buffers).
	 */
	{
		size_t src_max = uatomic_load(&src_ft->max_used_key_len,
				CMM_RELAXED);

		if (src_max > src_key_len &&
				src_max - src_key_len >
				dst_ft->group->max_key_len - dst_key_len) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
			return CDS_FT_STATUS_OVERFLOW_ERROR;
		}
	}

	/*
	 * merge_at is a mutator; the application provides mutual exclusion
	 * between mutators.  Take the reentrant writer-validation scope (like
	 * cds_ft_graft_swap), NOT a flavor read lock -- the spine-copy commit
	 * calls update_synchronize_rcu, which would deadlock inside a
	 * read-side critical section.
	 */
	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	/*
	 * Convert both keys to ORDINAL form ONCE; every internal consumer
	 * (the merge-point descents, the spine build, the source-subtree
	 * unlink, the ordered-list interleave) takes the ordinal form.  The
	 * detach+graft fallback below instead receives the ORIGINAL
	 * application keys -- those entry points remap internally.
	 * Previously the descents consumed the application bytes raw while
	 * the unlink remapped: on a non-identity key map the build copied one
	 * subtree and the unlink targeted another.
	 */
	{
		const struct cds_ft_key_map *km = &dst_ft->group->key_map;

		if (caa_unlikely(!km->identity)) {
			ft_key_to_ordinals(okey_dst_buf, dst_key, dst_key_len,
				km);
			ft_key_to_ordinals(okey_src_buf, src_key, src_key_len,
				km);
			okey_dst = okey_dst_buf;
			okey_src = okey_src_buf;
		}
	}

	/*
	 * Same-trie "rekey" (src_ft == dst_ft): moving @src_key's subtree to
	 * @dst_key within one trie is allowed, but the two keys must be DISJOINT
	 * -- neither a prefix of the other.  A prefix relationship means one key
	 * lies inside the other's subtree, so the move would be circular (and
	 * @dst_key would not be absent), and equal keys are a degenerate self-
	 * move.  The check is byte-position-wise, identical in application and
	 * ordinal space (the key map is a per-position bijection).  Cross-trie
	 * subtrees are disjoint by construction, so the guard is same-trie only.
	 */
	if (src_ft == dst_ft) {
		size_t m = src_key_len < dst_key_len ? src_key_len : dst_key_len;

		if (m == 0 || memcmp(okey_src, okey_dst, m) == 0) {
			FT_TP(merge_exit,
				(int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	/*
	 * Locate both merge points read-only (a writer descends its own
	 * stable state).  No content under @src_key -> the merge is a no-op;
	 * cnt_src == 0 covers an empty @src_ft root (src_key_len == 0, where
	 * the descent still reports EXACT at the always-present root).
	 */
	ks = ft_merge_descend(src_ft, okey_src, src_key_len, &d_src,
			&off_src, &cnt_src);
	if (ks == FT_GRAFT_SWAP_DELEGATE || cnt_src == 0) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Same-trie "rekey": detach @src_key's subtree into a transient trie,
	 * then merge that whole trie back in at @dst_key.  The detach fully
	 * commits -- recompacting the shared common-prefix ancestor and draining
	 * -- BEFORE the placement reads it, so the move's two sides no longer
	 * alias the same node.  The in-place reorder CANNOT be used here: its
	 * source unlink would recompact that shared ancestor out from under the
	 * build-invisible destination placement (a stale-slot publish).  The
	 * cross-trie merge handles an occupied @dst_key by MERGING and an absent
	 * one by grafting; it is made UNFAILABLE (its nodes + flip batches are
	 * pre-reserved before the detach below), so it always commits the move --
	 * there is no failure path that would restore the content to @src_key, i.e.
	 * no reader-observable rollback.
	 *
	 * A KEY_SHORTER source (off_src > 0, @src_key ends inside the compressed
	 * node @d_src.nf) is reduced to the EXACT node boundary first, exactly as
	 * the cross-trie path does: detach @cn_s->child via the full-compressed
	 * key (src prefix ++ all of cn_s), and merge at @dst_key extended by the
	 * residual cn_s bytes -- ft_detach_keylen overshoots a compressed node, so
	 * it must be handed a boundary key, not an interior one.
	 */
	if (src_ft == dst_ft) {
		struct cds_ft *tmp = NULL;
		const uint8_t *det_key = src_key, *mrg_key = dst_key;
		size_t det_len = src_key_len, mrg_len = dst_key_len;
		uint8_t det_buf[FT_MAX_KEY_LEN], mrg_buf[FT_MAX_KEY_LEN];
		uint8_t omrg_buf[FT_MAX_KEY_LEN];
		const uint8_t *omrg = okey_dst;		/* ordinal mrg key */
		size_t omrg_len = dst_key_len;
		struct cds_ft_alloc_reserve reserve;
		struct ft_descent d_mrg;
		unsigned int off_mrg;
		unsigned long n = cnt_src, m;		/* moved / dst subtree counts */
		struct urcu_flip_txn *pf_txn = NULL;

		if (off_src > 0) {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;
			unsigned int base = (unsigned int) d_src.depth, j;

			/* det_key = src prefix to cn_s start ++ ALL of cn_s. */
			memcpy(det_buf, src_key, base);
			/* mrg_key = dst_key ++ the residual cn_s bytes. */
			memcpy(mrg_buf, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&det_buf[base], cn_s->key_bytes, cn_s->len);
				memcpy(&mrg_buf[dst_key_len],
					&cn_s->key_bytes[off_src],
					cn_s->len - off_src);
			} else {
				for (j = 0; j < cn_s->len; j++)
					det_buf[base + j] = km->ordinal_to_key[
						cn_s->key_bytes[j]];
				for (j = off_src; j < cn_s->len; j++)
					mrg_buf[dst_key_len + j - off_src] =
						km->ordinal_to_key[
						cn_s->key_bytes[j]];
			}
			det_key = det_buf;
			det_len = base + cn_s->len;
			mrg_key = mrg_buf;
			mrg_len = dst_key_len + (cn_s->len - off_src);
			/* Ordinal mrg key: dst prefix ++ residual cn_s (already ordinal). */
			memcpy(omrg_buf, okey_dst, dst_key_len);
			memcpy(&omrg_buf[dst_key_len], &cn_s->key_bytes[off_src],
				cn_s->len - off_src);
			omrg = omrg_buf;
			omrg_len = mrg_len;
		}

		/*
		 * Read-only count pass at @mrg_key (writer lock held, so the counts
		 * stay valid for the post-detach merge): @m, the dst subtree key
		 * count, bounds the structural re-parent flips (nr_dst <= m); @n
		 * (== cnt_src, the moved subtree) sizes the ordered interleave flips
		 * (2n+2).  Both are O(1) reads off the subtree-root metadata.  The
		 * detach preserves @n into @tmp and only reshapes the PATH to
		 * @mrg_key (never its subtree), so nr_dst <= m still holds after.
		 * m == 0 means @mrg_key is absent (the merge grafts).
		 */
		(void) ft_merge_descend(dst_ft, omrg, omrg_len, &d_mrg, &off_mrg,
			&m);

		/*
		 * Pre-reserve the merge's flip-txn HERE -- before the detach, where a
		 * malloc failure is harmless (nothing has moved).  The post-detach
		 * merge then has no fallible allocation left (every node draws from
		 * @reserve, every flip latch from this reserved txn), so it CANNOT fail
		 * and needs no reader-observable rollback.  @pf_txn feeds the structural
		 * re-parent or the graft commit; the consume site NULLs the slot it
		 * takes, so we free it below only if a given shape left it unused.
		 *
		 * Size per shape (the rekey recursion lands on spine-copy when @mrg_key
		 * is occupied, graft when absent):
		 *   - m > 0 (spine-copy): the structural re-parent (<= m+1) plus, for an
		 *     ordered merge, the folded interleave's <= 2n+2 cell edges.
		 *   - m == 0 (graft): the cluster floor FT_GLUE_FLOOR_DEFERRED + 6, the
		 *     bound ft_graft_keylen reserves its own txn to -- it covers a GLUE
		 *     diverge's back-edges + forward + run-splice AND the NOSPLIT store's
		 *     slot proxy + run edges (FT_GRAFT_RUN_FLIP_CAP), whichever the graft
		 *     point turns out to be.
		 */
		{
			unsigned int pf_cap;

			if (m == 0)
				pf_cap = FT_GLUE_FLOOR_DEFERRED + 6;
			else if (dst_ft->group->ordered_list_set)
				pf_cap = (unsigned int) (m + 1) +
					(unsigned int) (2 * n + 2);
			else
				pf_cap = (unsigned int) (m + 1);
			pf_txn = ft_flip_txn_create();
			if (pf_txn && !urcu_flip_txn_reserve(pf_txn, pf_cap)) {
				urcu_flip_txn_destroy(pf_txn);
				pf_txn = NULL;
			}
		}
		if (!pf_txn) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Generous node reserve, also before the detach, so the merge's node
		 * allocations cannot fail either.  With both reserves in hand the
		 * detach is the LAST fallible step (clean on its own failure).
		 */
		memset(&reserve, 0, sizeof(reserve));
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			urcu_flip_txn_destroy(pf_txn);
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		status = ft_detach_keylen(dst_ft, det_key, det_len, &tmp);
		if (status < 0) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			urcu_flip_txn_destroy(pf_txn);
			FT_TP(merge_exit, (int) (status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status));
			return status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status;	/* NOT_FOUND -> no-op */
		}
		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		cds_ft_alloc_reserve_activate(tmp, &reserve);
		/*
		 * Unfailable placement: every node draws from @reserve and every
		 * flip latch from @pf_txn, so the merge always commits the move.  No
		 * failure path can strand @tmp's content back at @det_key -- that
		 * re-graft was the reader-observable rollback we removed.
		 */
		status = ft_merge_at_inner(dst_ft, mrg_key, mrg_len, tmp, NULL, 0,
			&pf_txn);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		cds_ft_alloc_reserve_deactivate(tmp);
		assert(status == CDS_FT_STATUS_OK);
		/* Free the flip-txn this merge shape did not consume. */
		if (pf_txn)
			urcu_flip_txn_destroy(pf_txn);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		cds_ft_destroy(tmp);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	kd = ft_merge_descend(dst_ft, okey_dst, dst_key_len, &d_dst,
			&off_dst, &cnt_dst);

	/*
	 * Atomic build-invisible spine-copy of @src_ft's subtree at @src_key into
	 * @dst_ft's non-empty subtree at @dst_key.  Each side is EXACT (@off == 0,
	 * the subtree at @d->nf) or KEY_SHORTER (@off > 0, the key ends inside a
	 * compressed node -- the src node is reclaimed by the commit's unlink, the
	 * dst node is wrapped under its prefix).  ft_merge_spine_copy handles every
	 * such shape (internal, external, compressed and compressed-parent dst
	 * merge points); only a dst that is empty under @dst_key falls through to
	 * the detach-based graft below.
	 */
	if ((ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER)
			&& cnt_dst > 0
			&& (kd == FT_GRAFT_SWAP_EXACT
				|| kd == FT_GRAFT_SWAP_KEY_SHORTER)) {
		status = ft_merge_spine_copy(dst_ft, src_ft, &d_src,
				okey_src, src_key_len, cnt_src, off_src,
				&d_dst, cnt_dst, off_dst, dst_key_len,
				pre_txn);
		if (status == CDS_FT_STATUS_OK) {
			/*
			 * Raise dst's max_used_key_len for the moved keys
			 * (dst_key_len + the longest stripped suffix), as the
			 * graft paths do -- later graft overflow validations
			 * feed off it.
			 */
			size_t src_max = uatomic_load(&src_ft->max_used_key_len,
					CMM_RELAXED);
			size_t nm = src_max > src_key_len ?
				dst_key_len + (src_max - src_key_len) :
				dst_key_len;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
					CMM_RELAXED);

			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					CMM_RELAXED);
		}
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Whole-source move (src_key_len == 0): the entire @src_ft is the
	 * payload, so graft it directly -- no detach, no fallible rollback.
	 * ft_graft is itself leak-free: a diverge split is built invisibly and
	 * a NOSPLIT attach restores the saved old source root allocation-free
	 * on OOM, so an allocation failure leaves both tries pristine.  This
	 * covers an empty dst root and a diverged dst_key alike, and is the
	 * leak-free path for "rekey within a trie" (detach a sub-trie, then
	 * merge_at it at a new, currently-absent key in the same group).  A
	 * dst_key occupied by an empty-internal leftover yields POPULATED_ERROR
	 * here, exactly as the detach-then-graft path did.
	 */
	if (src_key_len == 0) {
		status = ft_graft_keylen(dst_ft, dst_key, dst_key_len, src_ft,
				pre_txn);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Empty dst ROOT (dst_key_len == 0, so the whole @dst_ft is empty at
	 * the merge point): move @src_ft@src_key in WITHOUT a fallible graft,
	 * so no rollback can strand the moved externals.  Pre-allocate the
	 * source's replacement root (fallible while @src_ft is still pristine),
	 * detach the source subtree (clean on its own failure), then SWAP it
	 * into @dst_ft's root.  The swap is a failure-free root-to-root move:
	 * no parent-pointer change and no "jump out" window (both ends are
	 * roots with parent NULL), and the detach already drained @src_ft's
	 * readers -- the same reasoning as ft_graft's root path.  Because
	 * nothing fallible follows the detach, this path has no rollback, so it
	 * cannot leak (contrast the diverged path below).
	 */
	if (dst_key_len == 0 && cnt_dst == 0) {
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode *old_dst_root;
		struct urcu_flip_txn *appear_txn = NULL;
		size_t sm;

		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);

		/*
		 * Pre-reserve the dst-appear root-swap txn BEFORE the detach: the
		 * swap is failure-free (post-detach) so it commits through this
		 * pre-reserved txn (ft_ord_cell_flip_into).  OOM here aborts while
		 * @src_ft is still pristine (no detach yet).  List off uses the
		 * lone-edge ft_root_edge_flip below (no txn).
		 */
		if (dst_ft->group->ordered_list_set) {
			appear_txn = ft_flip_txn_create_bounded(
				FT_ROOT_LIST_SWAP_MAX_EDGES);
			if (!appear_txn) {
				free_cds_ft_node_unpublished(src_ft, fresh_root);
				status = CDS_FT_STATUS_MEMORY_ERROR;
				goto out;
			}
		}

		status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
		if (status < 0) {
			/* NOT_FOUND impossible: @src_ft had content. */
			if (appear_txn)
				urcu_flip_txn_destroy(appear_txn);
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			goto out;
		}

		/*
		 * Failure-free swap.  @subtree->root carries the moved content
		 * with parent already NULL (set by the detach), so it drops
		 * straight into @dst_ft's empty root slot.  @subtree keeps the
		 * pre-allocated empty root so its destroy below frees nothing of
		 * the moved content.  @dst_ft was empty, so it adopts @subtree's
		 * whole ordered cell list wholesale (head/tail endpoints only).
		 */
		old_dst_root = ft_node_ptr(dst_ft->root);
		/*
		 * Fuse the structural root swap with the ordered-list head/tail
		 * transfer into ONE flip (ft_root_list_swap_publish), so a reader
		 * never sees the moved keys reachable in @dst_ft's structure but
		 * its ordered list still empty -- the dst-appear cross-view window,
		 * closed the same way as the graft empty-dst path.  Only @dst_ft
		 * has concurrent readers here: @subtree is the fresh EXCLUSIVE trie
		 * the detach above just produced, so its root reset + head/tail
		 * clear are plain stores (no src-side disappear window, unlike
		 * ft_graft's cross-trie empty-dst).
		 */
		if (dst_ft->group->ordered_list_set) {
			ft_root_list_swap_publish(dst_ft, appear_txn, &dst_ft->root,
				dst_ft->root, subtree->root,
				NULL, subtree->ord_cell_head,
				NULL, subtree->ord_cell_tail);
			subtree->ord_cell_head = NULL;
			subtree->ord_cell_tail = NULL;
		} else {
			/*
			 * No ordered list: dst's root is the only reader-visible
			 * slot.  Express the lone appear edge as a single-edge flip
			 * descriptor (one release store, like a bare
			 * rcu_assign_pointer) so it is MCAS-expressible like the
			 * list-on path.  (@subtree is the fresh EXCLUSIVE trie, so
			 * its root reset below stays a plain store.)
			 */
			ft_root_edge_flip(dst_ft, &dst_ft->root,
				dst_ft->root, subtree->root);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(subtree->root, ft_node_flag(fresh_root, 0));
		free_cds_ft_node(dst_ft, old_dst_root);

		/* dst_key_len == 0: dst keys equal the moved keys, same lengths. */
		sm = uatomic_load(&subtree->max_used_key_len, CMM_RELAXED);
		if (sm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, sm, CMM_RELAXED);

		cds_ft_destroy(subtree);
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Sub-position source into a dst absent at @dst_key: graft the source
	 * subtree IN PLACE (no detach, no re-root) so the build-invisible cluster
	 * references @d_src.nf directly and the source unlink is the last fallible
	 * step -- the leak-free reorder.  Handles every source-root shape (plain
	 * multi-child internal, an internal carrying external_nodes, external,
	 * compressed, 1-child) at both a GLUE diverge and a NOSPLIT (at-node /
	 * build-branch) dst point.
	 *
	 * A KEY_SHORTER source (the src key ends @off_src bytes inside the
	 * compressed node @d_src.nf) reduces to the EXACT shape: the moved subtree
	 * is @cn_s->child, and its keys gain the residual bytes cn_s->key_bytes
	 * [off_src .. len) as a prefix, so graft @cn_s->child at @dst_key extended
	 * by that residual.  ft_merge_unlink_src_subtree overshoots @cn_s (preserves
	 * @cn_s->child) so the source side is identical; @cn_s itself is reclaimed
	 * by that unlink.
	 */
	if (ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER) {
		bool handled;
		struct cds_ft_inode_flag *payload;
		const uint8_t *gokey = okey_dst, *gappkey = dst_key;
		size_t glen = dst_key_len;
		uint8_t ext_okey[FT_MAX_KEY_LEN], ext_app[FT_MAX_KEY_LEN];

		if (ks == FT_GRAFT_SWAP_EXACT) {
			payload = d_src.nf;	/* off_src == 0 by construction */
		} else {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			unsigned int rlen = cn_s->len - off_src, j;
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;

			payload = cn_s->child;
			memcpy(ext_okey, okey_dst, dst_key_len);
			memcpy(&ext_okey[dst_key_len], &cn_s->key_bytes[off_src],
				rlen);
			memcpy(ext_app, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&ext_app[dst_key_len],
					&cn_s->key_bytes[off_src], rlen);
			} else {
				for (j = 0; j < rlen; j++)
					ext_app[dst_key_len + j] =
						km->ordinal_to_key[
						cn_s->key_bytes[off_src + j]];
			}
			gokey = ext_okey;
			gappkey = ext_app;
			glen = dst_key_len + rlen;
		}

		status = ft_merge_graft_subpos_inplace(dst_ft, src_ft,
			gokey, gappkey, glen, okey_src, src_key_len,
			payload, cnt_src, &handled);
		if (handled) {
			FT_TP(merge_exit, (int) status);
			return status;
		}
	}

	/*
	 * Unreachable.  ft_merge_descend reports EXACT, KEY_SHORTER or DELEGATE;
	 * DELEGATE returned OK at the top (cnt_src == 0), and every EXACT /
	 * KEY_SHORTER source is handled above: a non-empty dst by the build-
	 * invisible spine copy, an empty dst root or whole-source move by their
	 * failure-free root swaps, and a diverged / absent dst point by
	 * ft_merge_graft_subpos_inplace (which always reports handled).  No shape
	 * falls through, so the old detach-then-graft fallback -- the last path
	 * that carried the double-OOM leak -- and its rollback are gone.
	 */
	assert(0 && "cds_ft_merge_at: unhandled merge shape");
	status = CDS_FT_STATUS_OK;
out:
	FT_TP(merge_exit, (int) status);
	return status;
}

#endif /* FEATURE_FT_MERGE */

enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len)
{
#ifdef FEATURE_FT_MERGE
	return ft_merge_at_inner(dst_ft, dst_key, dst_key_len, src_ft,
			src_key, src_key_len, NULL);
#else
	(void) dst_ft; (void) dst_key; (void) dst_key_len;
	(void) src_ft; (void) src_key; (void) src_key_len;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

enum cds_ft_status cds_ft_merge(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft)
{
	return cds_ft_merge_at(dst_ft, key, key_len, src_ft, key, key_len);
}
