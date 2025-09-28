// SPDX-FileCopyrightText: 2012-2025 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * rcuja/rcuja-hashtable.c
 *
 * Userspace RCU library - RCU Judy Array Shadow Node Hash Table
 */

#define _LGPL_SOURCE
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <urcu/rcuja.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu-pointer.h>

#include "rcuja-internal.h"
#include "murmurhash3-internal.h"

static unsigned long hash_seed;

static
unsigned long hash_pointer(const void *_key, unsigned long seed)
{
	unsigned long key = (unsigned long) _key;

	return murmurhash3_u32(&key, sizeof(key), seed);
}

static
int match_pointer(struct cds_lfht_node *node, const void *key)
{
	struct cds_ja_shadow_node *shadow =
		caa_container_of(node, struct cds_ja_shadow_node, ht_node);

	return (key == shadow->node_flag);
}

__attribute__((visibility("hidden")))
struct cds_ja_shadow_node *rcuja_shadow_lookup(struct cds_lfht *ht,
		struct cds_ja_inode_flag *node_flag)
{
	struct cds_lfht_iter iter;
	struct cds_lfht_node *lookup_node;
	struct cds_ja_shadow_node *shadow_node;
	const struct rcu_flavor_struct *flavor;

	flavor = cds_lfht_rcu_flavor(ht);
	flavor->read_lock();
	cds_lfht_lookup(ht, hash_pointer(node_flag, hash_seed),
			match_pointer, node_flag, &iter);

	lookup_node = cds_lfht_iter_get_node(&iter);
	if (!lookup_node) {
		shadow_node = NULL;
		goto rcu_unlock;
	}
	shadow_node = caa_container_of(lookup_node,
			struct cds_ja_shadow_node, ht_node);
	assert(!cds_lfht_is_node_deleted(lookup_node));
rcu_unlock:
	flavor->read_unlock();
	return shadow_node;
}

__attribute__((visibility("hidden")))
struct cds_ja_shadow_node *rcuja_shadow_set(struct cds_lfht *ht,
		struct cds_ja_inode_flag *new_node_flag,
		struct cds_ja_shadow_node *inherit_from,
		struct cds_ja *ja, int level)
{
	struct cds_ja_shadow_node *shadow_node;
	struct cds_lfht_node *ret_node;
	const struct rcu_flavor_struct *flavor;

	shadow_node = calloc(sizeof(*shadow_node), 1);
	if (!shadow_node)
		return NULL;

	shadow_node->node_flag = new_node_flag;
	shadow_node->ja = ja;
	if (inherit_from) {
		shadow_node->level = inherit_from->level;
	} else {
		shadow_node->level = level;
	}

	flavor = cds_lfht_rcu_flavor(ht);
	flavor->read_lock();
	ret_node = cds_lfht_add_unique(ht,
			hash_pointer(new_node_flag, hash_seed),
			match_pointer,
			new_node_flag,
			&shadow_node->ht_node);
	flavor->read_unlock();

	if (ret_node != &shadow_node->ht_node) {
		free(shadow_node);
		return NULL;
	}
	return shadow_node;
}

static
void free_shadow_node(struct rcu_head *head)
{
	struct cds_ja_shadow_node *shadow_node =
		caa_container_of(head, struct cds_ja_shadow_node, head);
	free(shadow_node);
}

static
void free_shadow_node_and_node(struct rcu_head *head)
{
	struct cds_ja_shadow_node *shadow_node =
		caa_container_of(head, struct cds_ja_shadow_node, head);
	assert(shadow_node->level);
	free_cds_ja_node(shadow_node->ja, ja_node_ptr(shadow_node->node_flag));
	free(shadow_node);
}

__attribute__((visibility("hidden")))
int rcuja_shadow_clear(struct cds_lfht *ht,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_shadow_node *shadow_node,
		unsigned int flags)
{
	struct cds_lfht_iter iter;
	struct cds_lfht_node *lookup_node;
	const struct rcu_flavor_struct *flavor;
	int ret;

	flavor = cds_lfht_rcu_flavor(ht);
	flavor->read_lock();

	cds_lfht_lookup(ht, hash_pointer(node_flag, hash_seed),
			match_pointer, node_flag, &iter);
	lookup_node = cds_lfht_iter_get_node(&iter);
	if (!lookup_node) {
		ret = -ENOENT;
		goto rcu_unlock;
	}

	if (!shadow_node) {
		shadow_node = caa_container_of(lookup_node,
				struct cds_ja_shadow_node, ht_node);
	}

	ret = cds_lfht_del(ht, lookup_node);
	if (ret)
		goto rcu_unlock;
	if ((flags & RCUJA_SHADOW_CLEAR_FREE_NODE)
			&& shadow_node->level) {
		flavor->update_call_rcu(&shadow_node->head,
			free_shadow_node_and_node);
	} else {
		flavor->update_call_rcu(&shadow_node->head,
			free_shadow_node);
	}
rcu_unlock:
	flavor->read_unlock();

	return ret;
}

/*
 * Delete all shadow nodes and nodes from hash table.
 */
__attribute__((visibility("hidden")))
void rcuja_shadow_prune(struct cds_lfht *ht, unsigned int flags)
{
	const struct rcu_flavor_struct *flavor;
	struct cds_ja_shadow_node *shadow_node;
	struct cds_lfht_iter iter;
	int ret;

	flavor = cds_lfht_rcu_flavor(ht);
	/*
	 * Read-side lock is needed to ensure hash table node existence
	 * vs concurrent resize.
	 */
	flavor->read_lock();
	cds_lfht_for_each_entry(ht, &iter, shadow_node, ht_node) {
		ret = cds_lfht_del(ht, &shadow_node->ht_node);
		if (ret)
			continue;
		if ((flags & RCUJA_SHADOW_CLEAR_FREE_NODE)
				&& shadow_node->level) {
			flavor->update_call_rcu(&shadow_node->head,
				free_shadow_node_and_node);
		} else {
			flavor->update_call_rcu(&shadow_node->head,
				free_shadow_node);
		}
	}
	flavor->read_unlock();
}

__attribute__((visibility("hidden")))
struct cds_lfht *rcuja_create_ht(const struct rcu_flavor_struct *flavor)
{
	return _cds_lfht_new(1, 1, 0,
		CDS_LFHT_AUTO_RESIZE | CDS_LFHT_ACCOUNTING,
		NULL, flavor, NULL);
}

__attribute__((visibility("hidden")))
int rcuja_delete_ht(struct cds_lfht *ht)
{
	return cds_lfht_destroy(ht, NULL);
}

__attribute__((constructor))
static void rcuja_ht_init(void)
{
	hash_seed = (unsigned long) time(NULL);
}
