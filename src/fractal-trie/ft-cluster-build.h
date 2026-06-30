// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-cluster-build.h
 *
 * Userspace RCU library - Fractal Trie: node-cluster builders.  The shared
 * primitives that assemble a fresh, unpublished subtrie cluster for the
 * insert / graft / merge attach paths, cooperating with the ft_glue
 * build-invisible transaction:
 *   - ft_build_branch                build an internal branch down to a leaf;
 *   - ft_free_branch_unpublished     reclaim a fresh branch on a later failure;
 *   - ft_compress_single_child_if_needed   collapse a 1-child internal node;
 *   - ft_try_compress_chain          compress a single-child run.
 * They are mutually recursive, so a small forward-declaration block precedes
 * the definitions.
 *
 * #included after ft-mutation-node.h (the builders call the node set_nth /
 * recompact ops) and before ft-insert.h (their first user); graft and merge
 * reach them from further down the translation unit.
 *
 * Implementation unit: #included once into the fractal-trie.c translation unit
 * (preserves cross-module inlining).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-cluster-build.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/* Mutually-recursive builders -- forward decls to close the cycle. */
static struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_glue *glue);
static void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf);
static struct cds_ft_inode_flag *ft_compress_single_child_if_needed(
		struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct ft_glue *glue);
static struct cds_ft_inode_flag *ft_try_compress_chain(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, unsigned int level,
		struct cds_ft_inode_flag *child,
		struct cds_ft_node *external_nodes,
		struct ft_glue *glue);

/*
 * Build a branch for key[start .. end-1] with @leaf at the bottom.
 * When the path is 2+ bytes, a single compressed node is used instead
 * of a chain of single-child internal nodes.  Returns the topmost
 * flagged node, or NULL on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), @leaf is LIVE
 * payload data: its back-pointer into the bottom branch node is deferred
 * to the post-sync commit, every fresh branch node is tracked in @glue,
 * and on OOM the function returns NULL WITHOUT freeing -- the caller's
 * ft_glue_abort reclaims the tracked nodes.  When NULL, the legacy
 * immediate path runs (leaf back-pointer set now, self-free on OOM).
 */
static
struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_glue *glue)
{
	/*
	 * When the caller will attach external_nodes to the top,
	 * the top must be an internal node (compressed nodes cannot
	 * carry metadata->external_nodes).  Bias compression to start
	 * one byte deeper so an internal node is created at `start`.
	 */
	unsigned int compress_start = has_external_nodes ? start + 1 : start;
	struct cds_ft_inode_flag *cur = leaf;
	int loop_top, i;

	if (start == end)
		return leaf;	/* path_len == 0. */

	/*
	 * Try compression over [compress_start, end).  Floor is len 1:
	 * a 1-byte compressed under FEATURE_FT_SKIP_COMPRESSED publishes
	 * as a SKIP_X-tagged slot pointer (free dispatch) and is the
	 * canonical replacement for what would otherwise be a non-root
	 * 1-child internal node.  In glue mode ft_try_compress_chain
	 * defers the live leaf's back-pointer (and absorbs a compressed
	 * canonicalization-wrapper @leaf into this compressed).
	 */
	if (end >= compress_start + 1) {
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, end,
			compress_start, leaf, NULL, glue);
		if (compressed == (void *) (long) -ENOMEM)
			return NULL;
		if (compressed) {
			struct cds_ft_metadata *m =
				ft_flag_to_metadata(ft, compressed);

			ft_nr_keys_store(ft,m,
				subtree_external_count, CMM_RELAXED);
			/*
			 * ft_try_compress_chain already tracked the compressed
			 * node by its PLAIN flag in glue mode (the @compressed
			 * return here is the skip form, unsafe to track).
			 */
			cur = compressed;
			if (!has_external_nodes)
				return cur;
			/*
			 * has_external_nodes: fall through to create
			 * an internal node at `start` wrapping the
			 * compressed chunk.
			 */
		}
	}

	/*
	 * Create internal nodes from loop_top down to start.
	 *   Compression succeeded: only need an internal at `start`
	 *     (compress_start == start + 1, loop_top == start).
	 *   No compression:        create internal nodes for each
	 *     byte in [start, end).
	 */
	loop_top = (cur != leaf) ? (int) compress_start - 1 : (int) end - 1;
	for (i = loop_top; i >= (int) start; i--) {
		struct cds_ft_inode_flag *dest = NULL;
		/*
		 * Bottom internal whose child is the live @leaf (no
		 * compression happened): defer @leaf's back-pointer
		 * (cluster_leaf) and record it for commit.  Higher internals
		 * and the compressed-wrapping case have only fresh children,
		 * whose back-pointers are safe to set during the build.
		 */
		bool leaf_edge = (glue != NULL) && (cur == leaf);
		int ret;

		ret = ft_node_set_nth(ft, &dest, key[i], cur,
			NULL, NULL, i, leaf_edge);
		if (ret) {
			if (glue)
				return NULL;	/* abort frees tracked nodes */
			/*
			 * Legacy: free the created internal chain and, if
			 * present, the compressed chunk at the bottom.
			 */
			while (cur != leaf) {
				if (ft_node_compressed(cur)) {
					free_compressed_node(ft,
						ft_compressed_node_ptr(cur));
					cur = leaf;
				} else {
					struct cds_ft_inode_flag *next;
					uint8_t kv = key[i + 1];

					next = ft_node_get_nth(ft, cur, NULL, kv, FT_PF_NONE);
					free_cds_ft_node(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
			}
			return NULL;
		}
		ft_nr_keys_store(ft,
			cds_ft_item_to_metadata(ft_node_ptr(dest)),
			subtree_external_count, CMM_RELAXED);
		if (glue) {
			ft_glue_track(glue, dest);
			if (leaf_edge) {
				struct cds_ft_inode_flag **slot = NULL;

				ft_node_get_nth_skip(dest, &slot, key[i],
					FT_PF_NONE);
				ft_glue_defer_edge(ft, glue, leaf, dest, slot);
			}
		}
		/*
		 * Initialize density: this node's child (cur) may be
		 * the graft payload with an existing subtree.
		 * Bottom-up order ensures child density is set before
		 * parent.
		 */
		cur = dest;
	}
	return cur;
}

/*
 * Free a FRESH, never-published single-path branch built by ft_build_branch
 * (legacy glue == NULL mode) after a LATER fallible step failed: walk the
 * single-child chain from @top down to -- but not including -- @leaf (the
 * caller's payload), freeing every fresh node.  Writer-private memory, so
 * immediate frees are safe.  A skip-encoded link resolves through the leaf's
 * back-pointer, which the build wired before returning.
 */
static
void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf)
{
	while (top && top != leaf) {
		struct cds_ft_inode_flag *next;

		if (ft_node_skip_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_external(top)) {
			/* Only @leaf may be external on a fresh branch. */
			assert(top == leaf);
			break;
		} else {
			uint8_t v;

			next = ft_node_get_direction(ft, top, -1, &v,
				FT_RIGHT, false);
			free_cds_ft_node_unpublished(ft, ft_node_ptr(top));
		}
		top = next;
	}
}

/*
 * ft_compress_single_child_if_needed: convert a 1-child internal node
 * (no external_nodes) to a compressed/skip-encoded node when the trie
 * group has skip-compressed enabled.  Used at graft sites where a
 * moved root (which was an internal at trie-root position) is being
 * placed at a non-root position: under skip mode, non-root 1-child
 * internals without external_nodes must be a compressed (chain-
 * compress invariant).
 *
 * Two cases on the single child:
 *  - non-compressed:    build a 1-byte cn(byte) -> single_child.
 *  - compressed (or
 *    skip-encoded):     chain-merge -- build a cn(byte ++
 *                       single_cn.key_bytes) -> single_cn.child,
 *                       free the absorbed cn.  Bounded by
 *                       FT_SKIP_LEN_MAX; on overflow leave @child
 *                       unchanged.
 *
 * Returns the converted compressed/skip-encoded flag on success.
 * Returns @child unchanged when conversion isn't applicable (and the
 * unchanged @child is itself canonical at the target position):
 *   - skip-compressed mode is disabled,
 *   - @child is not an internal node,
 *   - @child has more than one child,
 *   - @child has external_nodes attached,
 *   - chain-merge length would overflow FT_SKIP_LEN_MAX.
 * Returns (void *)(long)-ENOMEM on allocation failure: conversion WAS
 * required (a non-root 1-child internal) but could not be done, so the
 * caller must NOT publish @child non-canonically -- it must fail the
 * mutation.  @child is left untouched in that case (nothing freed), so
 * the caller can still roll it back.
 *
 * On successful conversion the old internal is freed.  Write-side
 * only (mutex held); the caller is the sole owner of @child.
 *
 * @glue: when non-NULL, build-invisible mode for the graft transaction.
 * The wrapper @cn is built but its re-parent of the live @cn->child, and
 * the frees of the old internal and any absorbed sub-cn, are DEFERRED to
 * the post-sync commit (recorded in @glue) rather than performed here.
 * The wrapper is tracked in @glue so an OOM elsewhere in the build frees
 * it.  When NULL, the legacy immediate path runs (caller has already
 * drained the source).
 */
static
struct cds_ft_inode_flag *ft_compress_single_child_if_needed(struct cds_ft *ft,
		struct cds_ft_inode_flag *child,
		struct ft_glue *glue)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	struct cds_ft_inode *node;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_metadata *meta;
	uint8_t byte;
	struct cds_ft_inode_flag *single_child;
	struct cds_ft_compressed_node *single_cn = NULL;
	unsigned int single_len = 0;
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *cflag;
	unsigned int cn_len;

	if (!ft_group_skip_compressed(ft->group))
		return child;
	if (!ft_node_internal(child))
		return child;
	node = ft_node_ptr(child);
	type_index = ft_node_type(child);
	type = &ft_types[type_index];
	meta = cds_ft_item_to_metadata_fast(node, type->order);
	if (ft_meta_nr_child(meta) != 1 || meta->external_nodes != NULL)
		return child;

	/*
	 * Find the single set byte.  ft_node_get_minmax returns the
	 * resolved child (skip-encoded -> compressed flag); but we
	 * need the raw slot value to preserve any skip encoding when
	 * placing it as cn->child.  Walk via the low-level get_ith.
	 */
	switch (type->type_class) {
	case FT_POPCOUNT:
		ft_popcount_node_get_ith_pos(type, node, 0, &byte, &single_child);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		single_child = NULL;
		byte = 0;
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *v =
				ft_pigeon_node_get_ith_pos(type, node, i);
			if (v) {
				byte = (uint8_t) i;
				single_child = v;
				break;
			}
		}
		break;
	}
	default:
		return child;
	}
	if (!single_child)
		return child;

	/*
	 * Chain-merge when the single child is itself a compressed
	 * (or skip-encoded) cn: absorb its path bytes so the result
	 * is one compressed spanning [byte ++ single_cn->key_bytes]
	 * -> single_cn->child.  Preserves the "no two adjacent
	 * compresseds" invariant.
	 */
	if (ft_node_skip_compressed(single_child))
		single_cn = ft_skip_to_compressed(ft, single_child);
	else if (ft_node_compressed(single_child))
		single_cn = ft_compressed_node_ptr(single_child);
	if (single_cn) {
		single_len = single_cn->len;
		if (1U + single_len > FT_SKIP_LEN_MAX) {
			/* Overflow: leave un-canonicalized. */
			return child;
		}
	}
	cn_len = 1U + single_len;

	cn = alloc_compressed_node(ft, cn_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	cn->len = (uint8_t) cn_len;
	cn->key_bytes[0] = byte;
	if (single_cn) {
		memcpy(&cn->key_bytes[1], single_cn->key_bytes, single_len);
		cn->child = single_cn->child;
	} else {
		cn->child = single_child;
	}
	ft_meta_nr_child_set(cn_meta, 1);
	ft_nr_keys_store(ft,cn_meta, ft_nr_keys_get(meta), CMM_RELAXED);
	cflag = ft_compressed_node_flag(cn);
	if (glue) {
		/*
		 * Build-invisible path (graft transaction).  @node and
		 * @single_cn are LIVE source data that must survive an OOM
		 * elsewhere in the build, and @cn->child's back-pointer must
		 * not be flipped until the post-sync commit.  Defer all three;
		 * track @cn so the abort path frees it.  The deferred edge
		 * records the PLAIN @cflag -- ft_set_parent recovers @cn
		 * directly and records its skip_slot at commit.
		 */
		ft_glue_track(glue, cflag);
		ft_glue_defer_edge(ft, glue, cn->child, cflag, &cn->child);
		ft_glue_defer_free(glue, node, false);
		if (single_cn)
			ft_glue_defer_free(glue, single_cn, true);
		/*
		 * Return the PLAIN flag, NOT the skip form: @cn->child's
		 * back-pointer is deferred, so a skip pointer would not yet
		 * resolve (ft_skip_to_compressed recovers @cn via that stale
		 * back-pointer).  The caller installs @cn directly (chain-merge
		 * recovers it via ft_compressed_node_ptr) and re-encodes the
		 * holding slot to the skip form before publish -- same dance as
		 * the split's suffix.
		 */
		return cflag;
	}
	ft_set_parent(ft, cn->child, cflag, &cn->child);
	/*
	 * Legacy immediate path: the caller has already unlinked @node
	 * from src and waited a grace period (cds_ft_graft / graft_swap
	 * protocol); @node has not been published to dst.  No reader can
	 * be inside it, so the immediate-free path is safe -- saves a grace
	 * period of deferred-free pressure.  Same applies to single_cn
	 * (the compressed sub-node we just absorbed): it was reachable
	 * only via @node, which is itself unpublished here.
	 */
	free_cds_ft_node_unpublished(ft, node);
	if (single_cn)
		free_compressed_node_unpublished(ft, single_cn);
	return ft_publish_compressed(ft, cn, cflag);
#else
	(void) ft;
	(void) glue;
	return child;
#endif
}

/*
 * Try to create a compressed path for a chain of single-child nodes.
 * Returns the compressed node flag on success, NULL if compression
 * is not applicable (path too short) or disabled, -ENOMEM cast to
 * pointer on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), the live child's
 * back-pointer is recorded as a deferred edge rather than set now, and an
 * absorbed compressed @child (a fresh canonicalization wrapper) is
 * untracked from the glue as it is freed during the merge.
 */
#ifdef FEATURE_FT_COMPRESS
static
struct cds_ft_inode_flag *ft_try_compress_chain(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, unsigned int level,
		struct cds_ft_inode_flag *child,
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_glue *glue)
{
	uint8_t path_len = (uint8_t)(key_len - level);
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_compressed_node *child_cn = NULL;
	unsigned int child_len = 0;
	uint8_t merged_len;
	int j;

	/*
	 * Length-1 compressed nodes are canonical under
	 * FEATURE_FT_SKIP_COMPRESSED: the publish wraps cn into a
	 * SKIP_X-tagged slot pointer, dispatching for free relative to
	 * the 1-child internal node it replaces.
	 */
	if (path_len < 1)
		return NULL;

	/*
	 * Chain-merge: if @child is already a compressed (or skip-
	 * compressed) node, wrapping it in another compressed prefix
	 * would violate the "no two adjacent compresseds" invariant.
	 * Absorb the child's path bytes into the outer cn so the
	 * result is a single compressed spanning
	 * (key[level..key_len-1] ++ child_cn->key_bytes) ->
	 * child_cn->child.  Bounded by FT_SKIP_LEN_MAX; on overflow,
	 * fall back to the un-merged form (rare; the residue may be
	 * cleaned up by a subsequent mutation).
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		child_cn = ft_skip_to_compressed(ft, child);
	else
#endif
	if (ft_node_compressed(child))
		child_cn = ft_compressed_node_ptr(child);
	if (child_cn) {
		child_len = child_cn->len;
		/*
		 * Cap the fused path at what one compressed node can hold.  Under
		 * skip-compressed the merged node must also stay skip-encodable, so
		 * the cap is FT_SKIP_LEN_MAX.  Without skip-compression (notably
		 * 32-bit, where FT_SKIP_LEN_MAX is 0) the node is a plain compressed
		 * and the only limit is its uint8_t len field -- cap at UINT8_MAX.
		 * Using FT_SKIP_LEN_MAX unconditionally would never fuse there and
		 * leave two adjacent compresseds, violating the invariant.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if ((unsigned int) path_len + child_len > FT_SKIP_LEN_MAX) {
#else
		if ((unsigned int) path_len + child_len > UINT8_MAX) {
#endif
			/* Overflow: leave adjacency in place. */
			child_cn = NULL;
			child_len = 0;
		}
	}
	merged_len = (uint8_t)(path_len + child_len);

	cn = alloc_compressed_node(ft, merged_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	if (child_cn)
		cn->child = child_cn->child;
	else
		cn->child = child;
	cn->len = merged_len;
	for (j = 0; j < path_len; j++)
		cn->key_bytes[j] = key[level + j];
	if (child_cn)
		memcpy(&cn->key_bytes[path_len],
			child_cn->key_bytes, child_len);
	ft_meta_nr_child_set(cn_meta, 1);
	ft_nr_keys_store(ft,cn_meta, 1, CMM_RELAXED);
	/* Compressed nodes must not carry external_nodes. */
	assert(!external_nodes);
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);

		if (glue) {
			/*
			 * Build-invisible (graft): cn->child is LIVE -- either
			 * the merged-away child_cn's grandchild or the leaf
			 * itself.  Record its back-pointer for the post-sync
			 * commit instead of flipping it now.
			 *
			 * The absorbed child_cn is freed one of two ways: a fresh
			 * canonicalization wrapper (tracked in @glue) is dropped
			 * from tracking and freed here, so the abort path cannot
			 * double-free it; a LIVE compressed leaf (a re-rooted-in-
			 * place merge source absorbed into the branch run, never
			 * tracked) is still reader-reachable until the commit, so
			 * its free is DEFERRED past the grace period instead.
			 *
			 * Track the PLAIN @cflag (not the skip form returned to
			 * the caller): the abort path resolves a tracked node
			 * via ft_compressed_node_ptr, so it must not depend on
			 * cn->child's still-deferred back-pointer (which is how
			 * a skip pointer recovers its compressed node).
			 */
			ft_glue_defer_edge(ft, glue, cn->child, cflag,
				&cn->child);
			if (child_cn) {
				if (ft_glue_is_fresh(ft, glue,
						ft_compressed_node_flag(child_cn))) {
					ft_glue_untrack(ft, glue, child_cn);
					free_compressed_node_unpublished(ft,
						child_cn);
				} else {
					ft_glue_defer_free(glue, child_cn,
						true);
				}
			}
			ft_glue_track(glue, cflag);
			/*
			 * Emit the creation trace but return the PLAIN flag:
			 * the caller installs @cn directly and resolves it via
			 * ft_compressed_node_ptr (cn->child's back-pointer is
			 * deferred, so the skip form would not yet resolve).
			 * The caller re-encodes the holding slot to skip before
			 * publish.
			 */
			(void) ft_publish_compressed(ft, cn, cflag);
			return cflag;
		}
		ft_set_parent(ft, cn->child, cflag, &cn->child);
		if (child_cn)
			free_compressed_node_unpublished(ft, child_cn);
		/* compressed_publish emitted by ft_publish_compressed. */
		return ft_publish_compressed(ft, cn, cflag);
	}
}
#else
static inline
struct cds_ft_inode_flag *ft_try_compress_chain(
		struct cds_ft *ft __attribute__((unused)),
		const uint8_t *key __attribute__((unused)),
		size_t key_len __attribute__((unused)),
		unsigned int level __attribute__((unused)),
		struct cds_ft_inode_flag *child __attribute__((unused)),
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_glue *glue __attribute__((unused)))
{
	return NULL;
}
#endif

/*
 * Root-cluster builders: assemble a fresh root internal node from a glue
 * transaction -- used by the graft_swap re-root and by ft-detach when a detach
 * re-roots the trie.  ft_make_root_internal_glue layers over
 * ft_build_extracted_root_glue, so the latter precedes it.
 */
/*
 * Build (invisibly) a swap_ft root node holding the extracted subtree
 *   @first_byte ++ @rest[0 .. @rest_len-1]  ->  @child
 * i.e. an internal root whose @first_byte slot leads (via a fresh compressed
 * suffix when @rest_len >= 1) to the LIVE @child.  Used (via
 * ft_make_root_internal_glue) by the extract side of cds_ft_graft_swap and by
 * ft_detach_keylen's pre-publish root materialization.
 *
 * @child is LIVE data relocated into swap_ft: its back-pointer is recorded as
 * a deferred edge in @glue rather than flipped now, and every fresh node is
 * tracked so an OOM elsewhere in the swap build frees the cluster (both tries
 * pristine, nothing published).  Nothing is freed here.
 *
 * @child is always a plain (internal / external) node -- a compressed node's
 * child is plain by the chain-merge invariant, and the suffix path is held by
 * the fresh compressed @rest -- so no chain-merge is needed.
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_build_extracted_root_glue(struct cds_ft *ft,
		struct ft_glue *glue,
		uint8_t first_byte, const uint8_t *rest, unsigned int rest_len,
		struct cds_ft_inode_flag *child, unsigned long subtree_count)
{
	struct cds_ft_inode_flag *slot_value;
	struct cds_ft_inode_flag *skip_value = NULL;
	struct cds_ft_compressed_node *new_cn = NULL;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *root_meta;
	struct cds_ft_inode_flag *dest;
	struct cds_ft_inode_flag **slot = NULL;
	int ret;

	if (rest_len == 0) {
		slot_value = child;	/* LIVE; back-pointer deferred below. */
	} else {
		struct cds_ft_metadata *new_cn_meta;

		new_cn = alloc_compressed_node(ft, rest_len, &new_cn_meta);
		if (!new_cn)
			return (struct cds_ft_inode_flag *) (long) -ENOMEM;
		new_cn->len = (uint8_t) rest_len;
		new_cn->child = child;
		memcpy(new_cn->key_bytes, rest, rest_len);
		ft_meta_nr_child_set(new_cn_meta, 1);
		ft_nr_keys_store(ft,new_cn_meta, subtree_count, CMM_RELAXED);
		slot_value = ft_compressed_node_flag(new_cn);	/* PLAIN */
		ft_glue_track(glue, slot_value);
		/* @child (live) -> new_cn, deferred to the post-sync commit. */
		ft_glue_defer_edge(ft, glue, child, slot_value, &new_cn->child);
		/* Skip form for the root slot (resolves once the edge applies). */
		skip_value = ft_publish_compressed(ft, new_cn, slot_value);
	}

	root_node = alloc_cds_ft_node(ft, &ft_types[0], &root_meta);
	if (!root_node)
		/* new_cn (if any) is tracked in @glue; the caller's abort frees it. */
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	dest = ft_node_flag(root_node, 0);
	/*
	 * rest_len == 0: @child is live, so defer its back-pointer (cluster_leaf).
	 * rest_len >= 1: the slot holds the fresh new_cn, whose own back-pointer
	 * into @dest is a fresh-to-fresh edge that is safe to set during the build.
	 */
	ret = ft_node_set_nth(ft, &dest, first_byte, slot_value,
			NULL, root_meta, 0, rest_len == 0 /* cluster_leaf */);
	if (ret)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	ft_glue_track(glue, dest);
	ft_nr_keys_store(ft,ft_flag_to_metadata(ft, dest), subtree_count, CMM_RELAXED);
	ft_node_get_nth_skip(dest, &slot, first_byte, FT_PF_NONE);
	if (rest_len == 0) {
		ft_glue_defer_edge(ft, glue, child, dest, slot);
	} else {
		/* Re-encode the root slot to the skip form (new_cn is compressed). */
		if (skip_value && skip_value != slot_value && slot)
			*slot = skip_value;
		ft_set_parent(ft, slot_value, dest, slot);
	}
	return dest;
}

/*
 * Build-invisible internal-root materialization, preserving the trie-wide
 * invariant that the root pointer always tags an internal node (never
 * compressed, never skip-compressed).  Used by cds_ft_graft_swap's extract
 * side and by ft_detach_keylen.  Materializes an internal-node root from the
 * LIVE displaced @old_child without publishing or mutating live data: fresh nodes are tracked in @glue, the moved grandchild's
 * back-pointer is deferred, and the peeled-away compressed node is recorded for
 * deferred free.  Nothing is freed here.
 *
 *   - @old_child internal:    returned unchanged (already a valid internal
 *                             root; the caller clears its parent at commit).
 *                             No glue node, no deferred edge.
 *   - @old_child compressed:  peel the first path byte into a fresh internal
 *                             root, the rest (if any) into a fresh compressed
 *                             node; defer the live grandchild's back-pointer;
 *                             record the old compressed node for deferred free.
 *
 * External @old_child must be filtered by the caller (externals attach as
 * external_nodes, not via this helper).
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_make_root_internal_glue(struct cds_ft *ft,
		struct ft_glue *glue, struct cds_ft_inode_flag *old_child)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *root;

	old_child = ft_resolve_skip_compressed(ft, old_child);
	if (caa_likely(!ft_node_compressed(old_child)))
		return old_child;	/* already internal */
	cn = ft_compressed_node_ptr(old_child);
	cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	root = ft_build_extracted_root_glue(ft, glue,
			cn->key_bytes[0], &cn->key_bytes[1], cn->len - 1,
			cn->child, ft_nr_keys_get(cn_meta));
	if (root == (struct cds_ft_inode_flag *) (long) -ENOMEM)
		return root;
	/* Reclaim the peeled-away compressed node after the commit. */
	ft_glue_defer_free(glue, cn, true);
	return root;
}
