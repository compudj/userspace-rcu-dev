#ifndef _URCU_RCUJA_INTERNAL_H
#define _URCU_RCUJA_INTERNAL_H

/*
 * rcuja/rcuja-internal.h
 *
 * Userspace RCU library - RCU Judy Array Internal Header
 *
 * Copyright 2012 - Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <pthread.h>
#include <urcu/rculfhash.h>

/* Never declared. Opaque type used to store flagged node pointers. */
struct rcu_ja_node_flag;

/*
 * Shadow node contains mutex and call_rcu head associated with a node.
 */
struct rcu_ja_shadow_node {
	pthread_mutex_t lock;	/* mutual exclusion on node */
	struct rcu_head head;	/* for deferred node and shadow node reclaim */
};

struct rcu_ja {
	struct rcu_ja_node_flag *root;
	/*
	 * We use a hash table to associate nodes to their respective
	 * shadow node. This helps reducing lookup hot path cache
	 * footprint, especially for very small nodes.
	 */
	struct cds_lfht *ht;
};

#endif /* _URCU_RCUJA_INTERNAL_H */
