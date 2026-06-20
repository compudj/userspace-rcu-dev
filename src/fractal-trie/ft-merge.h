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
 * Interleave the surviving source cells into @dst's ordered list after a
 * cds_ft_merge_at spine-copy commit.  Walks the merged subtree at @dst_key in
 * key order (@merged_keys distinct heads) via the structural inequality oracle,
 * two-pointering against @dst's ORIGINAL region cells: @ord_cursor steps through
 * those (captured before the commit, min head of the dst merge subtree), and any
 * walked head that is NOT the cursor cell is a surviving src head -> splice it
 * after the last placed cell.  Dst-original cells are left untouched (their
 * relative order is preserved by the merge); collided src heads were demoted to
 * duplicates + their cells freed, so the walk never sees them.  @prev_placed
 * starts at the region's predecessor (@ord_cursor's ord_prev).
 *
 * Identity key_map only (the seed uses @dst_key directly; matches the rest of
 * the ordered-list machinery).  Uses @dst's writer-exclusive scratch iterator.
 */
static
void ft_merge_ord_interleave(struct cds_ft *dst, const uint8_t *dst_key,
		size_t dst_key_len, unsigned long merged_keys,
		struct ft_ord_cell *ord_cursor, struct ft_ord_cell *prev_placed,
		struct ft_ord_cell_edge *edges, struct ft_flip_batch *flip_b)
{
	struct cds_ft_iter *it = dst->ord_cell_scratch_iter;
	unsigned long i;

	/*
	 * Seed at the merge region's minimum, via the descent oracle
	 * (cache_valid = false: the cell list is mid-update, so the cell fast
	 * path must not be used).  A root merge (@dst_key_len == 0) merges the
	 * whole trie -> seed at the GLOBAL minimum with LIMIT_FIRST, which ignores
	 * the search key (a zero-length key is invalid for cds_ft_iter_set_key /
	 * a relational GE on a FIXED-length group, so the relational path would
	 * bail and leave the src run unspliced).  A scoped merge (@dst_key_len >
	 * 0) needs the first key >= @dst_key, so it sets @dst_key and uses the
	 * relational GE (LIMIT_NONE), which honors @dst_key.
	 */
	it->node = NULL;
	it->cache_valid = false;
	if (dst_key_len == 0) {
		it->key_len = 0;
		it->prefix_len = 0;
		it->key_off = 0;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_FIRST, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	} else {
		const uint8_t *seed_key = dst_key;
		size_t seed_len = dst_key_len;
		size_t flen = dst->group->key_len;
		uint8_t padbuf[FT_MAX_KEY_LEN];

		/*
		 * FIXED-length group with a mid-key merge point: @dst_key is
		 * shorter than the group's key length, which the relational GE
		 * rejects -- pad it to the fixed length with the ordinal-space
		 * minimum so GE finds the merged region's first key (every key
		 * in the region extends @dst_key, hence sorts at or above the
		 * padded probe; everything below the region sorts under it).
		 * @dst_key is ALREADY ORDINAL here (cds_ft_merge_at converts
		 * once at entry), so pad with raw 0x00 and set the iterator key
		 * without the public set_key's key-map application.
		 */
		if (flen != CDS_FT_LEN_VARIABLE && dst_key_len != flen) {
			assert(dst_key_len < flen && flen <= FT_MAX_KEY_LEN);
			memcpy(padbuf, dst_key, dst_key_len);
			memset(padbuf + dst_key_len, 0x00,
				flen - dst_key_len);
			seed_key = padbuf;
			seed_len = flen;
		}
		ft_iter_set_key_ordinals(it, seed_key, seed_len);
		it->prefix_len = 0;
		it->node = NULL;
		it->cache_valid = false;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	}
	/*
	 * Single-flip re-weave.  Walk the merged region in key order; pre-set
	 * each surviving src cell's links with plain stores (the cell is not yet
	 * ord-reachable in @dst, so this is invisible) and accumulate ONLY the
	 * VISIBLE boundary edges -- a dst-original cell's ord_next / ord_prev, or
	 * @dst's head / tail -- into one batch.  A single flip then commits the
	 * entire interleave atomically, so a concurrent ordered reader never
	 * observes a partially re-woven list (the per-splice path was N
	 * independent flips).  Dst-original cells keep their relative order, so a
	 * dst<->dst step needs no edge; each survivor RUN costs at most two edges
	 * (one entering, one leaving), so 2 * merged_keys + 2 bounds the batch.
	 * @edges and @flip_b are pre-allocated by the caller in the merge's
	 * fallible BUILD phase (this runs in the failure-free commit tail, where
	 * an alloc failure would force a non-atomic fallback).
	 */
	{
		struct ft_ord_cell *prev = prev_placed;
		bool prev_is_dst = (prev_placed != NULL);
		unsigned int n = 0;

		for (i = 0; i < merged_keys; i++) {
			struct cds_ft_node *head = cds_ft_iter_node(it);
			struct ft_ord_cell *cell;

			if (!head)
				break;
			cell = ft_ord_cell_ptr(rcu_dereference(head->prev));
			if (cell == ord_cursor) {
				/*
				 * Dst-original cell: stays put, already linked in
				 * key order.  Its back edge changes only when a
				 * survivor run was just placed before it.
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
				ord_cursor = ft_ord_cell_resolve_ord(
					&ord_cursor->ord_next);
			} else {
				/* Surviving src cell: pre-set its back link. */
				cell->ord_prev = prev;	/* invisible */
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
			}
			it->cache_valid = false;	/* force the descent oracle */
			if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GT,
					FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
				break;
		}
		/*
		 * Close the trailing edge: if the last placed cell is a survivor,
		 * link it to the region successor (@ord_cursor, advanced past the
		 * last dst-original; NULL at the list tail) and flip that
		 * neighbour's back edge -- or @dst's tail when there is none.
		 */
		if (prev && !prev_is_dst) {
			prev->ord_next = ord_cursor;	/* survivor: invisible */
			if (!ord_cursor) {
				edges[n].slot = &dst->ord_cell_tail;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&dst->ord_cell_tail);
				edges[n].new_target = prev;
				n++;
			} else {
				edges[n].slot = &ord_cursor->ord_prev;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&ord_cursor->ord_prev);
				edges[n].new_target = prev;
				n++;
			}
		}
		ft_ord_cell_flip_prealloc(dst, edges, n, flip_b);
	}
	return;
unused:
	/* Seed failed (empty region): the pre-allocated batch goes unused. */
	ft_flip_batch_free_unpublished(flip_b);
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
		unsigned long detached_count)
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
	ret = ft_detach_node(src_ft, d.nfp, d.pnfp, d.depth,
			/*free_detached_subtree=*/ false, NULL, NULL);
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
		const uint8_t *dst_key, size_t dst_key_len,
		struct ft_flip_batch **pre_flip,
		struct ft_flip_batch **pre_ms_flip)
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
	struct ft_flip_batch *flip;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_flip_batch *ms_flip = NULL;

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
	 * Allocate the flip batch: one proxy per dst-origin re-parent edge,
	 * plus one for the merge-point forward slot.  Fallible -> abort the
	 * still-invisible build; both tries stay pristine.
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		flip = ft_flip_batch_take(dst_ft, nr_dst + 1, pre_flip);
	}
	if (!flip) {
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
		 * Pre-allocate the interleave's edge batch + flip batch NOW,
		 * while the build is still abortable.  The interleave runs in
		 * the failure-free commit tail; an alloc failure there would
		 * have to degrade to a non-atomic per-edge fallback, exposing a
		 * half-re-woven list to bidirectional ordered readers under
		 * memory pressure.  Each survivor run costs at most two visible
		 * edges, so 2 * merged_keys + 2 bounds both.
		 */
		{
			unsigned int ms_cap =
				2u * (unsigned int) merged_keys + 2u;

			ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
			if (ms_edges)
				ms_flip = ft_flip_batch_take(dst_ft, ms_cap,
						pre_ms_flip);
			if (!ms_edges || !ms_flip) {
				free(ms_edges);
				ft_flip_batch_free_unpublished(flip);
				if (fresh_root)
					free_cds_ft_node_unpublished(src_ft,
						fresh_root);
				ft_glue_abort(dst_ft, &gd);
				ft_glue_abort(src_ft, &gs);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
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
	if (root_src) {
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src) < 0) {
		free(ms_edges);
		if (ms_flip)
			ft_flip_batch_free_unpublished(ms_flip);
		ft_flip_batch_free_unpublished(flip);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.
	 */
	if (ms_ord)
		ft_ord_cell_run_unlink(src_ft, ms_s_first, ms_s_last);
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
	 * 3. Stage the flip: point every dst-origin child's parent, and the
	 *    merge-point forward slot, at a proxy that still resolves to the
	 *    OLD target (selector 0).  Transparent to readers: up-walks and
	 *    the root descent still see the pre-merge dst.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			struct cds_ft_inode_flag *child, *old_parent, *pf;

			if (!gd.deferred[j].dst_origin)
				continue;
			child = gd.deferred[j].child;
			/*
			 * Current parent = the old dst overlap node.  Resolve a
			 * skip-encoded picked child to its compressed node first:
			 * ft_get_parent_rcu reads node metadata and cannot take a
			 * skip pointer (ft_set_parent_raw below resolves itself).
			 */
			old_parent = ft_get_parent_rcu(dst_ft,
				ft_resolve_skip_compressed(dst_ft, child));
			pf = ft_flip_batch_add(flip, old_parent,
				gd.deferred[j].parent);
			ft_set_parent_raw(dst_ft, child, pf);
		}
		/* Publish slot: old dst subtree -> merged cluster. */
		rcu_assign_pointer(*pub_slot,
				ft_flip_batch_add(flip, D_old, M_slot));
	}

	/*
	 * 4. Flip: one release store switches every dst-origin parent AND the
	 *    forward slot from old to merged, atomically.  Because the forward
	 *    slot flips with the back-pointers, a reader (which descends then
	 *    walks up) only ever progresses old->merged, never regresses.
	 */
	urcu_flip_commit(&flip->group);

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
	 * 7. Settle: rewrite each proxied slot to its direct merged target
	 *    (idempotent for readers -- the proxy already resolves to merged),
	 *    so the proxies become unreferenced and reclaimable.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				ft_set_parent(dst_ft, gd.deferred[j].child,
					gd.deferred[j].parent,
					gd.deferred[j].slot);
		rcu_assign_pointer(*pub_slot, M_slot);
	}

	/*
	 * 8. Reclaim, all deferred: the flip batch (proxies now unreferenced)
	 *    and the old overlap spines (src-side to src, dst-side to dst).
	 *    No dst synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_flip_batch_reclaim(flip);
	ft_glue_free_old(src_ft, &gs);
	ft_glue_free_old(dst_ft, &gd);

	/*
	 * 9. Ordered list: splice the surviving src cells into dst's ordered
	 * list at their merged positions.  Done AFTER the settle (step 7) so the
	 * merged structure carries direct pointers -- the interleave's GT
	 * continuation backtracks via parent pointers, which are flip proxies
	 * until settled.  Collided src cells were demoted to duplicates by
	 * apply_splices (step 5), so the merged-region walk never sees them;
	 * dst-original cells keep their links.  The edge + flip batches were
	 * pre-allocated in the build phase, so this cannot fail.
	 *
	 * 10. Only now free the collided cells: the interleave rewired the
	 * surviving cells' stale src-run links away from them, and the
	 * grace-period defer inside the free covers readers already holding
	 * such a link.
	 */
	if (ms_ord) {
		ft_merge_ord_interleave(dst_ft, dst_key, dst_key_len, merged_keys,
			ms_cursor, ms_prev, ms_edges, ms_flip);
		free(ms_edges);
	}
	ft_glue_free_collided_cells(dst_ft, &gd);

	ft_glue_fini(&gd);
	ft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}

/*
 * Reserve one node mirroring a freshly-built graft-cluster node @nf, so the
 * real graft (which rebuilds the same cluster) can draw it instead of
 * allocating.  Compressed node -- the compressed kind/order for its byte
 * length; internal node -- its type's order and bitmap.  Used to AUTO-LEARN a
 * GLUE diverge's node manifest from a build-and-abort learn pass, without hand-
 * coding the split's per-node arithmetic.
 */
static
int ft_merge_reserve_add_built(struct cds_ft *ft, struct cds_ft_alloc_reserve *r,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_inode_flag *p = ft_resolve_skip_compressed(ft, nf);

	if (ft_node_compressed(p)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(p);
		enum cds_ft_alloc_kind kind = ft->group->speculative ?
			CDS_FT_ALLOC_KIND_COMPRESSED : CDS_FT_ALLOC_KIND_NODE;

		return cds_ft_alloc_reserve_add(ft, r, kind,
			ft_compressed_order(cn->len), false, 1);
	}
	{
		unsigned int ti = ft_node_type(p);

		return cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			ft_types[ti].order, ft_types[ti].bitmap, 1);
	}
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
/*
 * Reserve EXACTLY the nodes ft_store_at_graft_point will allocate for a NOSPLIT
 * in-place graft of @payload at the located point @d, so the post-drain store
 * draws them and cannot fail on an arena allocation.  @d is ft_graft_build's
 * NOSPLIT descent.  Two shapes:
 *  - at-node (@d->depth == @key_len, empty slot): the only commit node alloc is
 *    an optional grow-recompact of the attach node @d->pnf.
 *  - build-a-branch (@d->depth < @key_len): ft_build_branch's nodes, learned by
 *    a build-and-abort pass (it builds into a throwaway glue with no dst
 *    mutation), plus an optional grow when an empty slot ADDS a child (a
 *    displaced external is REPLACED at its slot, no grow).
 * The payload needs NO canonicalize allocation: ft_compress_single_child_if_
 * needed only allocates for a 1-child internal under skip mode, and a skip-mode
 * trie never holds a 1-child internal as a subtree root (it is canonicalized at
 * creation), so a re-rooted payload is never that shape.  The flip batch is a
 * malloc (not arena), so it is not reserved.  Returns 0, or -ENOMEM (learn build
 * or fill OOM; the caller drains the reserve).
 */
static
int ft_merge_nosplit_reserve(struct cds_ft *dst_ft, const uint8_t *okey_dst,
		size_t dst_key_len, struct cds_ft_inode_flag *payload,
		unsigned long cnt_src, struct ft_descent *d,
		struct cds_ft_alloc_reserve *reserve)
{
	struct cds_ft_inode_flag *displaced;
	struct ft_glue lg;
	struct cds_ft_inode_flag *branch;
	bool grow = false;
	int rret = 0, bi;

	if (d->depth == dst_key_len && !d->nf) {
		grow = true;			/* at-node: only the grow, if any */
	} else {
		displaced = (d->nf && ft_node_external(d->nf)) ? d->nf : NULL;
		ft_glue_init(&lg);
		branch = ft_build_branch(dst_ft, okey_dst, d->depth, dst_key_len,
			payload, cnt_src, displaced != NULL, &lg);
		if (!branch) {
			ft_glue_fini(&lg);
			return -ENOMEM;
		}
		for (bi = 0; bi < lg.nr_built && !rret; bi++)
			rret = ft_merge_reserve_add_built(dst_ft, reserve,
				lg.built[bi]);
		ft_glue_abort(dst_ft, &lg);	/* frees the learn nodes */
		if (rret)
			return rret;
		grow = (displaced == NULL);	/* an empty slot ADDS a child */
	}

	if (grow) {
		struct cds_ft_metadata *am =
			cds_ft_item_to_metadata(ft_node_ptr(d->pnf));
		unsigned int aidx = ft_node_type(d->pnf);

		if (am->nr_child + 1 > ft_types[aidx].max_child) {
			unsigned int gidx = find_nearest_type_index(aidx,
				am->nr_child + 1, am->parent == NULL);

			rret = cds_ft_alloc_reserve_add(dst_ft, reserve,
				CDS_FT_ALLOC_KIND_NODE, ft_types[gidx].order,
				ft_types[gidx].bitmap, 1);
		}
	}
	return rret;
}

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
	struct ft_flip_batch *pre_flip = NULL;
	struct ft_descent d;
	enum ft_graft_prep prep;
	struct cds_ft_inode_flag *attached_nf, *aparent;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *pred = NULL, *succ = NULL;
	struct ft_ord_cell *run_first = NULL, *run_last = NULL;
	struct cds_ft_node *s_first = NULL, *s_last = NULL;
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
	prep = ft_graft_build(dst_ft, okey_dst, dst_key_len, payload, cnt_src,
			&d, &glue);
	*handled = true;
	if (prep == FT_GRAFT_PREP_OOM) {
		ft_glue_abort(dst_ft, &glue);
		return CDS_FT_STATUS_MEMORY_ERROR;	/* both tries pristine */
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/* Defensive: cnt_dst == 0 should never yield an occupied point. */
		ft_glue_fini(&glue);
		return CDS_FT_STATUS_POPULATED_ERROR;
	}
	if (prep == FT_GRAFT_PREP_NOSPLIT) {
		/*
		 * Pre-fill the store's manifest while both tries are still pristine:
		 * the node allocations into the reserve, AND the one flip batch the
		 * store parks its proxy in (a malloc, so not reservable).  A displaced
		 * external is REPLACED via the back-channel with no flip batch; every
		 * other NOSPLIT shape (at-node empty slot, built non-displaced branch)
		 * needs exactly one.  Securing the flip batch here -- before the source
		 * unlink -- makes the post-drain store wholly failure-free, so the
		 * unlink is the true last fallible step and nothing strands.
		 */
		bool displaced = (d.depth < dst_key_len && d.nf
				&& ft_node_external(d.nf));

		if (ft_merge_nosplit_reserve(dst_ft, okey_dst, dst_key_len,
				payload, cnt_src, &d, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_fini(&glue);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (!displaced) {
			pre_flip = ft_flip_batch_alloc(dst_ft, 1);
			if (!pre_flip) {
				cds_ft_alloc_reserve_drain(dst_ft, &reserve);
				ft_glue_fini(&glue);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
	}
	/* GLUE: cluster built invisibly.  NOSPLIT: reserve + flip batch secured. */

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
	}

	/*
	 * Last fallible step: unlink the source subtree in place, preserving
	 * @payload.  On OOM the unlink self-undoes (src pristine) and the still-
	 * invisible cluster / reserve is released (dst pristine) -- no rollback.
	 */
	if (ft_merge_unlink_src_subtree(src_ft, okey_src, src_key_len,
			cnt_src) < 0) {
		if (pre_flip)
			ft_flip_batch_free_unpublished(pre_flip);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_glue_abort(dst_ft, &glue);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* ===== Everything from here on is failure-free. ===== */

	/*
	 * Remove the source run from src's ordered list now (the cells keep their
	 * internal links for the splice into dst below) -- before the drain, so
	 * sync drains src ord-readers of the run too.
	 */
	if (ms_ord)
		ft_ord_cell_run_unlink(src_ft, s_first, s_last);

	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * Failure-free commit of the build-invisible diverge cluster: wire
		 * the deferred live back-pointers (the payload, the displaced old
		 * child, the cluster top), splice the cluster in with the single
		 * forward publish, then reclaim the replaced compressed node.
		 */
		ft_glue_apply_deferred(dst_ft, &glue);
		ft_glue_publish(dst_ft, &glue);
		attached_nf = glue.attached_nf;
		ft_glue_free_old(dst_ft, &glue);
		ft_glue_fini(&glue);
	} else {
		/*
		 * NOSPLIT: graft the in-place payload at @d.  Every node allocation
		 * draws from the reserve and the proxy parks in the pre-secured flip
		 * batch, so the store has no fallible step left -- it cannot fail.
		 */
		unsigned int adepth = 0;
		enum cds_ft_status st;

		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		st = ft_store_at_graft_point(dst_ft, okey_dst, dst_key_len, &d,
				payload, cnt_src, &attached_nf, &adepth, &glue,
				&pre_flip);
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

	if (ms_ord) {
		/*
		 * An EXTERNAL payload is re-parented directly under a new internal
		 * slot, so its edge byte changes (src_key's last byte -> dst_key's).
		 * That byte lives in the head's CELL metadata (ft_rebuild_key_upwalk
		 * reads it for the ordered key rebuild) and ft_set_parent does NOT
		 * maintain it for externals, so refresh it here.  The moved external
		 * sits at exactly @dst_key (it was an EXACT leaf at @src_key, no
		 * suffix), so its new edge byte is the last byte of @dst_key.  An
		 * internal / compressed payload keeps every leaf's edge byte (the
		 * subtree moves wholesale), so no per-leaf fix-up is needed.
		 */
		if (ft_node_external(payload))
			cds_ft_item_to_metadata(run_first)->incoming_byte =
				okey_dst[dst_key_len - 1];
		/* Splice the moved run into dst at the located position. */
		ft_ord_cell_run_splice(dst_ft, run_first, run_last, pred, succ);
	}

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
 * @pre_flip / @pre_ms_flip carry flip batches the caller reserved before its
 * own last fallible step, so the spine-copy / graft commit below draws an
 * unfailable batch instead of allocating one.  Both NULL on the public path
 * (cds_ft_merge_at), set only by the same-trie rekey, which pre-allocates them
 * (sized from the O(1) subtree key counts) so its post-detach merge cannot fail
 * -- no reader-observable rollback.  The consume sites NULL the slot they take,
 * so the rekey frees exactly the batches a given merge shape left unused.
 */
static enum cds_ft_status ft_merge_at_inner(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len,
		struct ft_flip_batch **pre_flip,
		struct ft_flip_batch **pre_ms_flip)
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
		struct ft_flip_batch *pf_flip = NULL, *pf_ms = NULL;

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
		 * Pre-allocate the merge's flip batches HERE -- before the detach,
		 * where a malloc failure is harmless (nothing has moved).  The
		 * post-detach merge then has no fallible allocation left (every node
		 * draws from @reserve, every flip proxy from these batches), so it
		 * CANNOT fail and needs no reader-observable rollback.  @pf_flip
		 * (cap m+1) feeds the structural re-parent or the graft store;
		 * @pf_ms (cap 2n+2, only for an occupied ordered merge) feeds the
		 * ordered-list interleave.  Caps are upper bounds; the merge uses
		 * <= them and NULLs the slot it consumes, so we free the rest below.
		 */
		pf_flip = ft_flip_batch_alloc(dst_ft, (unsigned int) (m + 1));
		if (!pf_flip) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (m > 0 && dst_ft->group->ordered_list_set) {
			pf_ms = ft_flip_batch_alloc(dst_ft,
				(unsigned int) (2 * n + 2));
			if (!pf_ms) {
				ft_flip_batch_free_unpublished(pf_flip);
				FT_TP(merge_exit,
					(int) CDS_FT_STATUS_MEMORY_ERROR);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Generous node reserve, also before the detach, so the merge's node
		 * allocations cannot fail either.  With both reserves in hand the
		 * detach is the LAST fallible step (clean on its own failure).
		 */
		memset(&reserve, 0, sizeof(reserve));
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_flip_batch_free_unpublished(pf_flip);
			if (pf_ms)
				ft_flip_batch_free_unpublished(pf_ms);
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		status = ft_detach_keylen(dst_ft, det_key, det_len, &tmp);
		if (status < 0) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_flip_batch_free_unpublished(pf_flip);
			if (pf_ms)
				ft_flip_batch_free_unpublished(pf_ms);
			FT_TP(merge_exit, (int) (status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status));
			return status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status;	/* NOT_FOUND -> no-op */
		}
		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		cds_ft_alloc_reserve_activate(tmp, &reserve);
		/*
		 * Unfailable placement: every node draws from @reserve and every
		 * flip proxy from @pf_flip / @pf_ms, so the merge always commits the
		 * move.  No failure path can strand @tmp's content back at @det_key
		 * -- that re-graft was the reader-observable rollback we removed.
		 */
		status = ft_merge_at_inner(dst_ft, mrg_key, mrg_len, tmp, NULL, 0,
			&pf_flip, &pf_ms);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		cds_ft_alloc_reserve_deactivate(tmp);
		assert(status == CDS_FT_STATUS_OK);
		/* Free the flip batches this merge shape did not consume. */
		if (pf_flip)
			ft_flip_batch_free_unpublished(pf_flip);
		if (pf_ms)
			ft_flip_batch_free_unpublished(pf_ms);
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
				&d_dst, cnt_dst, off_dst, okey_dst, dst_key_len,
				pre_flip, pre_ms_flip);
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
				pre_flip);
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
		size_t sm;

		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);

		status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
		if (status < 0) {
			/* NOT_FOUND impossible: @src_ft had content. */
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
		rcu_assign_pointer(dst_ft->root, subtree->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		if (dst_ft->group->ordered_list_set) {
			rcu_assign_pointer(dst_ft->ord_cell_head,
				subtree->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				subtree->ord_cell_tail);
			subtree->ord_cell_head = NULL;
			subtree->ord_cell_tail = NULL;
		}
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
			src_key, src_key_len, NULL, NULL);
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
