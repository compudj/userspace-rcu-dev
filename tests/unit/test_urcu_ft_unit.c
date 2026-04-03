// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * test_urcu_ft_unit.c
 *
 * Userspace RCU library - Fractal Trie Unit Tests
 *
 * Self-contained, deterministic unit tests for the Fractal Trie public
 * API. Each test function creates its own trie, exercises one API
 * surface in isolation, validates invariants, tears down, and returns 0
 * on success or -1 on failure. Tests are registered in a table and
 * can be run individually by name or all at once.
 *
 * Build example (adapt include/library paths to your tree):
 *
 *   cc -O2 -g -DRCU_QSBR \
 *      -I/path/to/urcu/include \
 *      test_urcu_ft_unit.c \
 *      -lurcu-qsbr -lurcu-cds -lurcu-common -lpthread \
 *      -o test_urcu_ft_unit
 *
 * Run all:  ./test_urcu_ft_unit
 * Run one:  ./test_urcu_ft_unit test_lifecycle_defaults
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <urcu/compiler.h>
#include <urcu-qsbr.h>		/* Must precede fractal-trie.h */
#include <urcu/fractal-trie.h>
#include <urcu-call-rcu.h>

#include <assert.h>
#include <inttypes.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tap.h"

#define NR_TESTS 141

/* ------------------------------------------------------------------ */
/* Test-node infrastructure (mirrors test_urcu_ft.h).                 */
/* ------------------------------------------------------------------ */

struct ft_test_node {
	struct cds_ft_node node;
	struct rcu_head head;
	uint64_t key;		/* shadow copy for validation */
	uint64_t value;		/* optional payload */
};

static inline
void ft_test_node_init(struct ft_test_node *n, uint64_t key)
{
	cds_ft_node_init(&n->node);
	n->key = key;
	n->value = 0;
}

static inline
struct ft_test_node *to_test_node(struct cds_ft_node *n)
{
	return caa_container_of(n, struct ft_test_node, node);
}

static unsigned long nodes_allocated, nodes_freed;

static struct ft_test_node *node_alloc(uint64_t key)
{
	struct ft_test_node *n = (struct ft_test_node *) calloc(1, sizeof(*n));
	if (!n) {
		fprintf(stderr, "node_alloc: out of memory\n");
		abort();
	}
	ft_test_node_init(n, key);
	__atomic_add_fetch(&nodes_allocated, 1, __ATOMIC_RELAXED);
	return n;
}

static void node_free(struct ft_test_node *n)
{
	memset(n, 0xfe, sizeof(*n));	/* poison */
	free(n);
	__atomic_add_fetch(&nodes_freed, 1, __ATOMIC_RELAXED);
}

static void node_free_rcu_cb(struct rcu_head *head)
{
	node_free(caa_container_of(head, struct ft_test_node, head));
}

static void node_free_rcu(struct ft_test_node *n)
{
	call_rcu(&n->head, node_free_rcu_cb);
}

/* Free every node reachable through the trie, then destroy the trie. */
static int drain_and_destroy(struct cds_ft *ft, struct cds_ft_group *group)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = 0;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		fprintf(stderr, "drain_and_destroy: iter_create: %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			fprintf(stderr, "drain_and_destroy: remove_all: %s\n",
				cds_ft_status_to_string(s));
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
	}
	rcu_read_unlock();
	rcu_barrier();		/* wait for all node_free_rcu callbacks */
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/* Reset global leak counters. */
static void leak_reset(void)
{
	__atomic_store_n(&nodes_allocated, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&nodes_freed, 0, __ATOMIC_RELAXED);
}

static int leak_check(void)
{
	rcu_barrier();
	unsigned long na = __atomic_load_n(&nodes_allocated, __ATOMIC_RELAXED);
	unsigned long nf = __atomic_load_n(&nodes_freed, __ATOMIC_RELAXED);
	if (na != nf) {
		fprintf(stderr, "LEAK: allocated %lu, freed %lu (delta %ld)\n",
			na, nf, (long)(na - nf));
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

/*
 * Convenience: create a fixed-length trie with the given key_len.
 * Aborts on failure.
 */
static struct cds_ft *create_fixed_ft(size_t klen, struct cds_ft_group **group_out)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_attr_create(&attr) < 0)
		abort();
	if (cds_ft_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_attr_destroy(attr);
	if (cds_ft_create(group, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/* Create a variable-length trie (default attributes). */
static struct cds_ft *create_varlen_ft(struct cds_ft_group **group_out)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_create(NULL, &group) < 0)
		abort();
	if (cds_ft_create(group, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/* Insert helper for fixed-length integer keys. */
static enum cds_ft_status
insert_u64(struct cds_ft *ft, uint64_t v, struct ft_test_node *n)
{
	uint8_t k[8];

	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	return cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
}

/* Lookup helper for fixed-length integer keys. */
static enum cds_ft_status
lookup_u64(struct cds_ft *ft, uint64_t v, struct cds_ft_node **out)
{
	uint8_t k[8];

	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	return cds_ft_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, out);
}

/* ------------------------------------------------------------------ */
/* Macro: run a test, emit TAP ok/not-ok, skip if filtered out.       */
/* ------------------------------------------------------------------ */

#define RUN_TEST(fn)							\
	do {								\
		if (filter && strcmp(filter, #fn) != 0) {		\
			skip(1, "filtered out: " #fn);			\
			break;						\
		}							\
		leak_reset();						\
		rcu_quiescent_state();					\
		ok((fn)() == 0 && leak_check() == 0, "%s", #fn);	\
	} while (0)

static int drain_trie(struct cds_ft *ft);

/* ================================================================== */
/*                                                                    */
/*                 1. LIFECYCLE & ATTRIBUTE TESTS                     */
/*                                                                    */
/* ================================================================== */

/*
 * Create a trie with default (NULL) attributes → variable-length keys.
 * Verify properties and destroy.
 */
static int test_lifecycle_defaults(void)
{
	struct cds_ft_group *group;
	unsigned long ft_count;
	struct cds_ft *ft;

	if (cds_ft_group_create(NULL, &group) < 0)
		return -1;
	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_key_len(ft) != CDS_FT_LEN_VARIABLE) {
		fprintf(stderr, "expected variable key length\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "freshly created trie not empty\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 0) {
		fprintf(stderr, "freshly created trie count != 0\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Create fixed-length tries for each key width 1..8 and verify key_len.
 */
static int test_lifecycle_fixed_key_lengths(void)
{
	struct cds_ft_group *group;
	unsigned int klen;

	for (klen = 1; klen <= 8; klen++) {
		struct cds_ft *ft = create_fixed_ft(klen, &group);

		if (cds_ft_key_len(ft) != klen) {
			fprintf(stderr, "key_len mismatch for %u-byte trie\n", klen);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
		if (cds_ft_max_key_len(ft) < klen) {
			fprintf(stderr, "max_key_len < key_len for %u-byte trie\n", klen);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
	}
	return 0;
}

/*
 * Create a trie with a zero-length fixed key (NIL-only).
 */
static int test_lifecycle_nil_only_trie(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(0, &group);

	if (cds_ft_key_len(ft) != 0) {
		fprintf(stderr, "expected key_len == 0\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Exercise cds_ft_attr_set_max_key_len.
 */
static int test_lifecycle_max_key_len(void)
{
	struct cds_ft_group *group;
	struct cds_ft_attr *attr;
	struct cds_ft *ft;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	/* Variable-length keys with a 32-byte maximum. */
	if (cds_ft_attr_set_max_key_len(attr, 32) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);
	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	if (cds_ft_max_key_len(ft) != 32) {
		fprintf(stderr, "max_key_len not honoured\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Set a custom key map and retrieve it.
 */
static int test_lifecycle_key_map(void)
{
	struct cds_ft_group *group;
	struct cds_ft_attr *attr;
	struct cds_ft *ft;
	uint8_t k2o[CDS_FT_KEY_MAP_SIZE], o2k[CDS_FT_KEY_MAP_SIZE];
	uint8_t k2o_out[CDS_FT_KEY_MAP_SIZE], o2k_out[CDS_FT_KEY_MAP_SIZE];
	unsigned int i;
	enum cds_ft_status s;

	/* Build a simple reversed mapping. */
	for (i = 0; i < CDS_FT_KEY_MAP_SIZE; i++) {
		k2o[i] = (uint8_t)(255 - i);
		o2k[i] = (uint8_t)(255 - i);
	}

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, 1) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	s = cds_ft_attr_set_key_map(attr, k2o, o2k);
	if (s < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);
	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	s = cds_ft_key_map(ft, k2o_out, o2k_out);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "key_map returned unexpected status: %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (memcmp(k2o, k2o_out, sizeof(k2o)) != 0 ||
	    memcmp(o2k, o2k_out, sizeof(o2k)) != 0) {
		fprintf(stderr, "key map round-trip mismatch\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Verify cds_ft_status_to_string returns non-NULL for all known codes.
 */
static int test_status_to_string(void)
{
	const enum cds_ft_status codes[] = {
		CDS_FT_STATUS_OK,
		CDS_FT_STATUS_NOT_FOUND,
		CDS_FT_STATUS_DUPLICATE_FOUND,
		CDS_FT_STATUS_INTERNAL_MATCH,
		CDS_FT_STATUS_INVALID_ARGUMENT_ERROR,
		CDS_FT_STATUS_MEMORY_ERROR,
		CDS_FT_STATUS_OVERFLOW_ERROR,
		CDS_FT_STATUS_BUSY_ERROR,
		CDS_FT_STATUS_POPULATED_ERROR,
	};
	unsigned int i;

	for (i = 0; i < sizeof(codes) / sizeof(codes[0]); i++) {
		if (!cds_ft_status_to_string(codes[i])) {
			fprintf(stderr, "status_to_string(=%d) returned NULL\n",
				(int)codes[i]);
			return -1;
		}
	}
	return 0;
}

/* ================================================================== */
/*                                                                    */
/*                     2. INSERT VARIANT TESTS                        */
/*                                                                    */
/* ================================================================== */

/*
 * Basic insert on an empty trie and verify count/empty.
 */
static int test_insert_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n = node_alloc(42);
	unsigned long ft_count;
	enum cds_ft_status s;

	rcu_read_lock();
	s = insert_u64(ft, 42, n);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "insert failed: %s\n", cds_ft_status_to_string(s));
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_empty(ft)) {
		fprintf(stderr, "trie empty after insert\n");
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 1) {
		fprintf(stderr, "count != 1 after single insert\n");
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * insert_unique: first insert succeeds, second returns DUPLICATE_FOUND
 * and sets result_node to the existing node.
 */
static int test_insert_unique(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(7);
	struct ft_test_node *n2 = node_alloc(7);
	struct cds_ft_node *result;
	unsigned long ft_count;
	enum cds_ft_status s;
	uint8_t k[4];

	cds_ft_u64_to_key(ft, 7, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert_unique(ft, k, CDS_FT_LEN_DEFAULT, &n1->node, &result);
	if (s != CDS_FT_STATUS_OK || result != &n1->node) {
		fprintf(stderr, "first insert_unique failed: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		node_free(n1);
		node_free(n2);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	s = cds_ft_insert_unique(ft, k, CDS_FT_LEN_DEFAULT, &n2->node, &result);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_DUPLICATE_FOUND) {
		fprintf(stderr, "second insert_unique: expected DUPLICATE_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		node_free(n2);
		drain_and_destroy(ft, group);
		return -1;
	}
	if (result != &n1->node) {
		fprintf(stderr, "result_node does not point to existing node\n");
		node_free(n2);
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 1) {
		fprintf(stderr, "count should still be 1\n");
		node_free(n2);
		drain_and_destroy(ft, group);
		return -1;
	}
	node_free(n2);	/* was never inserted */
	return drain_and_destroy(ft, group);
}

/*
 * insert (non-unique) twice at the same key builds a duplicate chain
 * of length 2.
 */
static int test_insert_duplicate_chain(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct ft_test_node *n1 = node_alloc(99);
	struct ft_test_node *n2 = node_alloc(99);
	struct cds_ft_node *head;
	unsigned long ft_count;
	enum cds_ft_status s;
	int count = 0;

	rcu_read_lock();
	s = insert_u64(ft, 99, n1);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = insert_u64(ft, 99, n2);
	if (s != CDS_FT_STATUS_OK) goto fail;

	s = lookup_u64(ft, 99, &head);
	if (s != CDS_FT_STATUS_OK || !head) goto fail;

	cds_ft_for_each_duplicate_rcu(head)
		count++;
	rcu_read_unlock();

	if (count != 2) {
		fprintf(stderr, "duplicate chain length %d, expected 2\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 2) {
		fprintf(stderr, "trie count %lu, expected 2\n", ft_count);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "insert/lookup failed: %s\n", cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * insert_replace: insert a node, then replace the chain with a new node.
 * The old head is returned.
 */
static int test_insert_replace(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(10);
	struct ft_test_node *n2 = node_alloc(10);
	struct cds_ft_node *old_head = NULL;
	enum cds_ft_status s;
	uint8_t k[4];

	n1->value = 111;
	n2->value = 222;

	cds_ft_u64_to_key(ft, 10, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n1->node);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}

	s = cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT,
				  &n2->node, &old_head);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_DUPLICATE_FOUND) {
		fprintf(stderr, "insert_replace: expected DUPLICATE_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (old_head != &n1->node) {
		fprintf(stderr, "insert_replace: old_head != n1\n");
		goto fail;
	}

	/* Verify the trie now holds n2. */
	rcu_read_lock();
	{
		struct cds_ft_node *found;

		s = lookup_u64(ft, 10, &found);
		if (s != CDS_FT_STATUS_OK || found != &n2->node) {
			fprintf(stderr, "lookup after replace: wrong node\n");
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Free old node after grace period. */
	node_free_rcu(n1);
	/* n2 is still in the trie; drain_and_destroy frees it. */
	return drain_and_destroy(ft, group);

fail:
	/* Best-effort cleanup. */
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_entries tracks correctly across multiple inserts.
 */
static int test_count_tracking(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned long i, ft_count;
	int ret = -1;

	for (i = 0; i < 50; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "insert %lu failed\n", i);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 50) {
		fprintf(stderr, "count %lu, expected 50\n", ft_count);
		drain_and_destroy(ft, group);
		return -1;
	}
	ret = drain_and_destroy(ft, group);
	return ret;
}

/*
 * cds_ft_count_keys returns the number of distinct keys, not duplicates.
 * Insert several nodes at the same key, verify count_keys == 1 while
 * count_entries reflects the total number of nodes.
 */
static int test_count_keys_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(42);
	struct ft_test_node *n2 = node_alloc(42);
	struct ft_test_node *n3 = node_alloc(42);
	unsigned long keys, entries;
	enum cds_ft_status s;

	rcu_read_lock();
	s = insert_u64(ft, 42, n1);
	if (s != CDS_FT_STATUS_OK) goto fail;

	keys = cds_ft_count_keys(ft);
	entries = cds_ft_count_entries(ft);
	if (keys != 1 || entries != 1) {
		fprintf(stderr, "after 1 insert: keys %lu (exp 1), entries %lu (exp 1)\n",
			keys, entries);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	s = insert_u64(ft, 42, n2);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = insert_u64(ft, 42, n3);
	if (s != CDS_FT_STATUS_OK) goto fail;

	keys = cds_ft_count_keys(ft);
	entries = cds_ft_count_entries(ft);
	rcu_read_unlock();

	if (keys != 1) {
		fprintf(stderr, "count_keys_duplicates: keys %lu, expected 1\n", keys);
		drain_and_destroy(ft, group);
		return -1;
	}
	if (entries != 3) {
		fprintf(stderr, "count_keys_duplicates: entries %lu, expected 3\n", entries);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "count_keys_duplicates: insert failed: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys tracks distinct keys across inserts at different keys.
 */
static int test_count_keys_distinct(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned long i, keys;

	for (i = 0; i < 50; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "count_keys_distinct: insert %lu failed\n", i);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	keys = cds_ft_count_keys(ft);
	rcu_read_unlock();
	if (keys != 50) {
		fprintf(stderr, "count_keys_distinct: keys %lu, expected 50\n", keys);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_count_keys: removing a duplicate does not change the key count,
 * but removing the last duplicate at a key decrements it.
 */
static int test_count_keys_remove(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(10);
	struct ft_test_node *n2 = node_alloc(10);
	struct ft_test_node *n3 = node_alloc(20);
	struct cds_ft_iter *iter;
	struct cds_ft_node *found;
	unsigned long keys;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	insert_u64(ft, 10, n1);
	insert_u64(ft, 10, n2);
	insert_u64(ft, 20, n3);

	keys = cds_ft_count_keys(ft);
	if (keys != 2) {
		fprintf(stderr, "count_keys_remove: initial keys %lu, expected 2\n", keys);
		goto fail;
	}

	/* Remove one duplicate at key 10; key count should stay at 2. */
	s = lookup_u64(ft, 10, &found);
	if (s != CDS_FT_STATUS_OK || !found) goto fail;
	cds_ft_iter_set_key(iter,
		(const uint8_t *)"\0\0\0\0\0\0\0\0", CDS_FT_LEN_DEFAULT);
	{
		uint8_t k[8];

		cds_ft_u64_to_key(ft, 10, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	}
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_remove(ft, iter, &n1->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "count_keys_remove: remove n1: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	keys = cds_ft_count_keys(ft);
	if (keys != 2) {
		fprintf(stderr, "count_keys_remove: after dup remove, keys %lu, expected 2\n", keys);
		goto fail;
	}

	/* Remove last node at key 10; key count should drop to 1. */
	{
		uint8_t k[8];

		cds_ft_u64_to_key(ft, 10, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	}
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_remove(ft, iter, &n2->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "count_keys_remove: remove n2: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	keys = cds_ft_count_keys(ft);
	if (keys != 1) {
		fprintf(stderr, "count_keys_remove: after last remove, keys %lu, expected 1\n", keys);
		goto fail;
	}
	rcu_read_unlock();

	node_free_rcu(n1);
	node_free_rcu(n2);
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys: remove_all removes exactly one key regardless
 * of how many duplicates were at that key.
 */
static int test_count_keys_remove_all(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(5);
	struct ft_test_node *n2 = node_alloc(5);
	struct ft_test_node *n3 = node_alloc(5);
	struct ft_test_node *n4 = node_alloc(99);
	struct cds_ft_iter *iter;
	struct cds_ft_node *old_chain;
	unsigned long keys;
	enum cds_ft_status s;
	uint8_t k[8];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3); node_free(n4);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	insert_u64(ft, 5, n1);
	insert_u64(ft, 5, n2);
	insert_u64(ft, 5, n3);
	insert_u64(ft, 99, n4);

	keys = cds_ft_count_keys(ft);
	if (keys != 2) {
		fprintf(stderr, "count_keys_remove_all: initial keys %lu, expected 2\n", keys);
		goto fail;
	}

	cds_ft_u64_to_key(ft, 5, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_remove_all(ft, iter, &old_chain);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "count_keys_remove_all: remove_all: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	keys = cds_ft_count_keys(ft);
	if (keys != 1) {
		fprintf(stderr, "count_keys_remove_all: after remove_all, keys %lu, expected 1\n", keys);
		goto fail;
	}
	rcu_read_unlock();

	node_free_rcu(n1);
	node_free_rcu(n2);
	node_free_rcu(n3);
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys: insert_replace at an existing key does not change
 * the key count; insert_replace at a new key increments it.
 */
static int test_count_keys_replace(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1 = node_alloc(7);
	struct ft_test_node *n2 = node_alloc(7);
	struct ft_test_node *n3 = node_alloc(8);
	struct cds_ft_node *old_head = NULL;
	unsigned long keys;
	enum cds_ft_status s;
	uint8_t k[8];

	rcu_read_lock();
	s = insert_u64(ft, 7, n1);
	if (s != CDS_FT_STATUS_OK) goto fail;

	keys = cds_ft_count_keys(ft);
	if (keys != 1) {
		fprintf(stderr, "count_keys_replace: after insert, keys %lu, expected 1\n", keys);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Replace at existing key: count stays 1. */
	cds_ft_u64_to_key(ft, 7, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT, &n2->node, &old_head);
	if (s < 0) {
		fprintf(stderr, "count_keys_replace: insert_replace: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	keys = cds_ft_count_keys(ft);
	if (keys != 1) {
		fprintf(stderr, "count_keys_replace: after replace, keys %lu, expected 1\n", keys);
		rcu_read_unlock();
		node_free_rcu(n1);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* insert_replace at a new key: count becomes 2. */
	cds_ft_u64_to_key(ft, 8, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT, &n3->node, &old_head);
	if (s < 0) {
		fprintf(stderr, "count_keys_replace: insert_replace new key: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		node_free_rcu(n1);
		drain_and_destroy(ft, group);
		return -1;
	}

	keys = cds_ft_count_keys(ft);
	rcu_read_unlock();
	if (keys != 2) {
		fprintf(stderr, "count_keys_replace: after new key, keys %lu, expected 2\n", keys);
		node_free_rcu(n1);
		drain_and_destroy(ft, group);
		return -1;
	}

	node_free_rcu(n1);
	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "count_keys_replace: insert failed: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys: graft transfers source key count to destination,
 * detach removes keys from source and places them in the detached trie.
 */
static int test_count_keys_graft_detach(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging, *detached;
	enum cds_ft_status s;
	unsigned long keys;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate staging with 3 keys. */
	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);
		struct ft_test_node *c = node_alloc(0);

		s = cds_ft_insert(staging, (const uint8_t *)"aa", 2, &a->node);
		if (s < 0) goto fail;
		s = cds_ft_insert(staging, (const uint8_t *)"ab", 2, &b->node);
		if (s < 0) goto fail;
		s = cds_ft_insert(staging, (const uint8_t *)"ac", 2, &c->node);
		if (s < 0) goto fail;
	}

	rcu_read_lock();
	keys = cds_ft_count_keys(staging);
	rcu_read_unlock();
	if (keys != 3) {
		fprintf(stderr, "count_keys_graft_detach: staging keys %lu, expected 3\n", keys);
		goto fail;
	}

	/* Also add a key directly in live. */
	{
		struct ft_test_node *d = node_alloc(0);

		rcu_read_lock();
		s = cds_ft_insert(live, (const uint8_t *)"zz", 2, &d->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* Graft staging into live at prefix "p". */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"p", 1, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "count_keys_graft_detach: graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should have 4 keys: paa, pab, pac, zz. */
	rcu_read_lock();
	keys = cds_ft_count_keys(live);
	rcu_read_unlock();
	if (keys != 4) {
		fprintf(stderr, "count_keys_graft_detach: live after graft %lu, expected 4\n", keys);
		goto fail;
	}

	/* Staging should be empty. */
	rcu_read_lock();
	keys = cds_ft_count_keys(staging);
	rcu_read_unlock();
	if (keys != 0) {
		fprintf(stderr, "count_keys_graft_detach: staging after graft %lu, expected 0\n", keys);
		goto fail;
	}

	/* Detach the "p" subtree from live. */
	rcu_read_lock();
	s = cds_ft_detach(live, (const uint8_t *)"p", 1, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "count_keys_graft_detach: detach: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should have 1 key: zz. */
	rcu_read_lock();
	keys = cds_ft_count_keys(live);
	rcu_read_unlock();
	if (keys != 1) {
		fprintf(stderr, "count_keys_graft_detach: live after detach %lu, expected 1\n", keys);
		drain_trie(detached);
		rcu_barrier();
		cds_ft_destroy(detached);
		goto fail;
	}

	/* Detached should have 3 keys: aa, ab, ac. */
	rcu_read_lock();
	keys = cds_ft_count_keys(detached);
	rcu_read_unlock();
	if (keys != 3) {
		fprintf(stderr, "count_keys_graft_detach: detached %lu, expected 3\n", keys);
		drain_trie(detached);
		rcu_barrier();
		cds_ft_destroy(detached);
		goto fail;
	}

	drain_trie(detached);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(detached);
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * cds_ft_count_keys: empty trie has 0 keys, adding and draining returns to 0.
 */
static int test_count_keys_empty(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned long keys;

	rcu_read_lock();
	keys = cds_ft_count_keys(ft);
	rcu_read_unlock();
	if (keys != 0) {
		fprintf(stderr, "count_keys_empty: fresh trie keys %lu, expected 0\n", keys);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Insert and drain, verify count returns to 0. */
	{
		struct ft_test_node *n = node_alloc(1);

		rcu_read_lock();
		insert_u64(ft, 1, n);
		keys = cds_ft_count_keys(ft);
		rcu_read_unlock();
		if (keys != 1) {
			fprintf(stderr, "count_keys_empty: after insert keys %lu, expected 1\n", keys);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	/* drain_and_destroy removes all entries and checks leak. */
	{
		int ret = drain_and_destroy(ft, group);

		return ret;
	}
}

/*
 * cds_ft_count_keys_prefix: count keys under a prefix in a varlen trie.
 * Insert keys at "aa", "ab", "ac", "ba", verify prefix "a" returns 3,
 * prefix "b" returns 1, prefix "" returns 4, non-existent prefix returns 0.
 */
static int test_count_keys_prefix_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct ft_test_node *n1 = node_alloc(0);
	struct ft_test_node *n2 = node_alloc(0);
	struct ft_test_node *n3 = node_alloc(0);
	struct ft_test_node *n4 = node_alloc(0);
	unsigned long count;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"aa", 2, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"ab", 2, &n2->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"ac", 2, &n3->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"ba", 2, &n4->node);
	if (s < 0) goto fail;

	/* Empty prefix: all keys. */
	count = cds_ft_count_keys_prefix(ft, NULL, 0);
	if (count != 4) {
		fprintf(stderr, "prefix_basic: empty prefix count %lu, expected 4\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Prefix "a": keys aa, ab, ac. */
	count = cds_ft_count_keys_prefix(ft, (const uint8_t *)"a", 1);
	if (count != 3) {
		fprintf(stderr, "prefix_basic: prefix 'a' count %lu, expected 3\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Prefix "b": key ba. */
	count = cds_ft_count_keys_prefix(ft, (const uint8_t *)"b", 1);
	if (count != 1) {
		fprintf(stderr, "prefix_basic: prefix 'b' count %lu, expected 1\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Prefix "ab": exact key ab. */
	count = cds_ft_count_keys_prefix(ft, (const uint8_t *)"ab", 2);
	if (count != 1) {
		fprintf(stderr, "prefix_basic: prefix 'ab' count %lu, expected 1\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Non-existent prefix "c". */
	count = cds_ft_count_keys_prefix(ft, (const uint8_t *)"c", 1);
	if (count != 0) {
		fprintf(stderr, "prefix_basic: prefix 'c' count %lu, expected 0\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "prefix_basic: insert failed: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys_prefix with duplicates: duplicates at the same key
 * do not inflate the prefix key count.
 */
static int test_count_keys_prefix_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct ft_test_node *n1 = node_alloc(0);
	struct ft_test_node *n2 = node_alloc(0);
	struct ft_test_node *n3 = node_alloc(0);
	unsigned long count;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"xy", 2, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"xy", 2, &n2->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"xy", 2, &n3->node);
	if (s < 0) goto fail;

	count = cds_ft_count_keys_prefix(ft, (const uint8_t *)"x", 1);
	rcu_read_unlock();
	if (count != 1) {
		fprintf(stderr, "prefix_duplicates: prefix 'x' count %lu, expected 1\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}

	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "prefix_duplicates: insert failed: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_count_keys_prefix on a fixed-length trie using integer keys.
 */
static int test_count_keys_prefix_fixed(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned long i, count;

	/* Insert 256 keys: 0x00000000 .. 0x000000FF. */
	for (i = 0; i < 256; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "prefix_fixed: insert %lu failed\n", i);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	/* Empty prefix: all 256 keys. */
	count = cds_ft_count_keys_prefix(ft, NULL, 0);
	if (count != 256) {
		fprintf(stderr, "prefix_fixed: empty prefix %lu, expected 256\n", count);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/*
	 * 1-byte prefix: all 256 keys share the same first 3 bytes
	 * (big-endian 0x00000000..0x000000FF), so a 1-byte prefix
	 * of 0x00 should match all 256.
	 */
	{
		uint8_t prefix[1] = { 0x00 };

		count = cds_ft_count_keys_prefix(ft, prefix, 1);
		if (count != 256) {
			fprintf(stderr, "prefix_fixed: 1-byte 0x00 prefix %lu, expected 256\n", count);
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	/* A 1-byte prefix of 0x01 should match nothing. */
	{
		uint8_t prefix[1] = { 0x01 };

		count = cds_ft_count_keys_prefix(ft, prefix, 1);
		if (count != 0) {
			fprintf(stderr, "prefix_fixed: 1-byte 0x01 prefix %lu, expected 0\n", count);
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_lookup_nth: basic forward rank lookup on a fixed-length trie.
 * Insert keys 0..9, verify lookup_nth(i) returns the ith key in order.
 */
static int test_lookup_nth_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 10; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "lookup_nth_basic: insert %lu failed\n", i);
			goto fail;
		}
		rcu_read_unlock();
	}

	/* Verify each rank. */
	for (i = 0; i < 10; i++) {
		uint8_t result_key[4];
		size_t result_key_len;
		uint64_t val;

		rcu_read_lock();
		s = cds_ft_lookup_nth(ft, iter, i);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "lookup_nth_basic: nth(%lu): %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto fail;
		}
		s = cds_ft_iter_get_key(iter, result_key, sizeof(result_key),
				&result_key_len);
		if (s != CDS_FT_STATUS_OK || result_key_len != 4) {
			fprintf(stderr, "lookup_nth_basic: get_key(%lu): %s len %zu\n",
				i, cds_ft_status_to_string(s), result_key_len);
			rcu_read_unlock();
			goto fail;
		}
		val = cds_ft_key_to_u64(ft, result_key, 4);
		rcu_read_unlock();
		if (val != i) {
			fprintf(stderr, "lookup_nth_basic: nth(%lu) got key %lu\n",
				i, (unsigned long) val);
			goto fail;
		}
	}

	/* Out-of-range should return NOT_FOUND. */
	rcu_read_lock();
	s = cds_ft_lookup_nth(ft, iter, 10);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "lookup_nth_basic: nth(10) expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_lookup_nth_last: reverse rank lookup.
 * nth_last(0) is the largest key, nth_last(9) is the smallest.
 */
static int test_lookup_nth_last(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 10; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	for (i = 0; i < 10; i++) {
		uint8_t result_key[4];
		size_t result_key_len;
		uint64_t val;

		rcu_read_lock();
		s = cds_ft_lookup_nth_last(ft, iter, i);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "lookup_nth_last: nth_last(%lu): %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto fail;
		}
		cds_ft_iter_get_key(iter, result_key, sizeof(result_key),
				&result_key_len);
		val = cds_ft_key_to_u64(ft, result_key, 4);
		rcu_read_unlock();
		if (val != 9 - i) {
			fprintf(stderr, "lookup_nth_last: nth_last(%lu) got %lu, expected %lu\n",
				i, (unsigned long) val, 9 - i);
			goto fail;
		}
	}

	/* Out-of-range. */
	rcu_read_lock();
	s = cds_ft_lookup_nth_last(ft, iter, 10);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "lookup_nth_last: nth_last(10) expected NOT_FOUND\n");
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_lookup_nth with duplicates: duplicates at the same key share
 * a single rank. The nth lookup returns the head of the duplicate chain.
 */
static int test_lookup_nth_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(10);
	struct ft_test_node *n2 = node_alloc(10);
	struct ft_test_node *n3 = node_alloc(20);
	enum cds_ft_status s;
	uint8_t result_key[4];
	size_t result_key_len;
	uint64_t val;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	insert_u64(ft, 10, n1);
	insert_u64(ft, 10, n2);
	insert_u64(ft, 20, n3);

	/* 2 unique keys: rank 0 = key 10, rank 1 = key 20. */
	s = cds_ft_lookup_nth(ft, iter, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_nth_dup: nth(0): %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, result_key, sizeof(result_key), &result_key_len);
	val = cds_ft_key_to_u64(ft, result_key, 4);
	if (val != 10) {
		fprintf(stderr, "lookup_nth_dup: nth(0) got %lu, expected 10\n", (unsigned long) val);
		rcu_read_unlock();
		goto fail;
	}

	s = cds_ft_lookup_nth(ft, iter, 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_nth_dup: nth(1): %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, result_key, sizeof(result_key), &result_key_len);
	val = cds_ft_key_to_u64(ft, result_key, 4);
	if (val != 20) {
		fprintf(stderr, "lookup_nth_dup: nth(1) got %lu, expected 20\n", (unsigned long) val);
		rcu_read_unlock();
		goto fail;
	}

	/* Rank 2 should be NOT_FOUND (only 2 unique keys). */
	s = cds_ft_lookup_nth(ft, iter, 2);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "lookup_nth_dup: nth(2) expected NOT_FOUND\n");
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_lookup_nth on a variable-length trie with prefix keys.
 * Keys "a", "ab", "abc" test that prefix keys at internal nodes
 * are correctly ranked.
 */
static int test_lookup_nth_varlen(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(0);
	struct ft_test_node *n2 = node_alloc(0);
	struct ft_test_node *n3 = node_alloc(0);
	enum cds_ft_status s;
	uint8_t result_key[8];
	size_t result_key_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"ab", 2, &n2->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"a", 1, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n3->node);
	if (s < 0) goto fail;

	/* Order: "a" (rank 0), "ab" (rank 1), "abc" (rank 2). */
	s = cds_ft_lookup_nth(ft, iter, 0);
	if (s != CDS_FT_STATUS_OK) goto fail;
	cds_ft_iter_get_key(iter, result_key, sizeof(result_key), &result_key_len);
	if (result_key_len != 1 || memcmp(result_key, "a", 1) != 0) {
		fprintf(stderr, "lookup_nth_varlen: nth(0) key_len %zu\n", result_key_len);
		rcu_read_unlock();
		goto fail_nolock;
	}

	s = cds_ft_lookup_nth(ft, iter, 1);
	if (s != CDS_FT_STATUS_OK) goto fail;
	cds_ft_iter_get_key(iter, result_key, sizeof(result_key), &result_key_len);
	if (result_key_len != 2 || memcmp(result_key, "ab", 2) != 0) {
		fprintf(stderr, "lookup_nth_varlen: nth(1) key_len %zu\n", result_key_len);
		rcu_read_unlock();
		goto fail_nolock;
	}

	s = cds_ft_lookup_nth(ft, iter, 2);
	if (s != CDS_FT_STATUS_OK) goto fail;
	cds_ft_iter_get_key(iter, result_key, sizeof(result_key), &result_key_len);
	if (result_key_len != 3 || memcmp(result_key, "abc", 3) != 0) {
		fprintf(stderr, "lookup_nth_varlen: nth(2) key_len %zu\n", result_key_len);
		rcu_read_unlock();
		goto fail_nolock;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
fail_nolock:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_lookup_nth on empty trie returns NOT_FOUND.
 */
static int test_lookup_nth_empty(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	s = cds_ft_lookup_nth(ft, iter, 0);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "lookup_nth_empty: expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_iter_skip_forward: insert keys 0..9, position at key 3,
 * skip forward 4 → should land on key 7.
 */
static int test_iter_skip_forward(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;
	uint8_t rk[4];
	size_t rk_len;
	uint64_t val;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 10; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	/* Position at key 3. */
	rcu_read_lock();
	s = cds_ft_lookup_nth(ft, iter, 3);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_forward: lookup_nth(3): %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Skip forward 4 → key 7. */
	s = cds_ft_iter_skip_forward(ft, iter, 4);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_forward: skip(4): %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	val = cds_ft_key_to_u64(ft, rk, 4);
	rcu_read_unlock();
	if (val != 7) {
		fprintf(stderr, "skip_forward: got key %lu, expected 7\n",
			(unsigned long) val);
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_iter_skip_reverse: insert keys 0..9, position at key 7,
 * skip reverse 4 → should land on key 3.
 */
static int test_iter_skip_reverse(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;
	uint8_t rk[4];
	size_t rk_len;
	uint64_t val;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 10; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	/* Position at key 7. */
	rcu_read_lock();
	s = cds_ft_lookup_nth(ft, iter, 7);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_reverse: lookup_nth(7): %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Skip reverse 4 → key 3. */
	s = cds_ft_iter_skip_reverse(ft, iter, 4);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_reverse: skip(4): %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	val = cds_ft_key_to_u64(ft, rk, 4);
	rcu_read_unlock();
	if (val != 3) {
		fprintf(stderr, "skip_reverse: got key %lu, expected 3\n",
			(unsigned long) val);
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Skip out-of-range: skip forward beyond the last key returns NOT_FOUND,
 * skip reverse beyond the first key returns NOT_FOUND.
 */
static int test_iter_skip_boundary(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	/* Position at key 3, skip forward 5 → out of range (only 1 key after 3). */
	rcu_read_lock();
	s = cds_ft_lookup_nth(ft, iter, 3);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_iter_skip_forward(ft, iter, 5);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "skip_boundary: forward overflow expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Position at key 1, skip reverse 5 → out of range. */
	s = cds_ft_lookup_nth(ft, iter, 1);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_iter_skip_reverse(ft, iter, 5);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "skip_boundary: reverse overflow expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Skip with duplicates: duplicates share a rank, skip should jump
 * over entire duplicate groups.
 */
static int test_iter_skip_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(10);
	struct ft_test_node *n2 = node_alloc(10);
	struct ft_test_node *n3 = node_alloc(20);
	struct ft_test_node *n4 = node_alloc(30);
	enum cds_ft_status s;
	uint8_t rk[4];
	size_t rk_len;
	uint64_t val;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3); node_free(n4);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	insert_u64(ft, 10, n1);
	insert_u64(ft, 10, n2);	/* duplicate at key 10 */
	insert_u64(ft, 20, n3);
	insert_u64(ft, 30, n4);

	/* 3 unique keys: 10 (rank 0), 20 (rank 1), 30 (rank 2). */

	/* Position at key 10, skip forward 2 → key 30. */
	s = cds_ft_lookup_nth(ft, iter, 0);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_iter_skip_forward(ft, iter, 2);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_dup: forward skip: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	val = cds_ft_key_to_u64(ft, rk, 4);
	if (val != 30) {
		fprintf(stderr, "skip_dup: forward got %lu, expected 30\n",
			(unsigned long) val);
		rcu_read_unlock();
		goto fail;
	}

	/* Skip reverse 1 from key 30 → key 20. */
	s = cds_ft_iter_skip_reverse(ft, iter, 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_dup: reverse skip: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	val = cds_ft_key_to_u64(ft, rk, 4);
	rcu_read_unlock();
	if (val != 20) {
		fprintf(stderr, "skip_dup: reverse got %lu, expected 20\n",
			(unsigned long) val);
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Skip on a varlen trie with prefix keys: "a", "ab", "abc", "b".
 * Position at "a", skip forward 2 → "abc".
 */
static int test_iter_skip_varlen(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *na = node_alloc(0);
	struct ft_test_node *nab = node_alloc(0);
	struct ft_test_node *nabc = node_alloc(0);
	struct ft_test_node *nb = node_alloc(0);
	enum cds_ft_status s;
	uint8_t rk[8];
	size_t rk_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(na); node_free(nab); node_free(nabc); node_free(nb);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"a", 1, &na->node);
	cds_ft_insert(ft, (const uint8_t *)"ab", 2, &nab->node);
	cds_ft_insert(ft, (const uint8_t *)"abc", 3, &nabc->node);
	cds_ft_insert(ft, (const uint8_t *)"b", 1, &nb->node);

	/* Position at "a" (rank 0), skip forward 2 → "abc" (rank 2). */
	s = cds_ft_lookup_nth(ft, iter, 0);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_iter_skip_forward(ft, iter, 2);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_varlen: forward: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	if (rk_len != 3 || memcmp(rk, "abc", 3) != 0) {
		fprintf(stderr, "skip_varlen: forward got key_len %zu\n", rk_len);
		rcu_read_unlock();
		goto fail;
	}

	/* Skip reverse 3 from "abc" (rank 2) → out of range. */
	s = cds_ft_iter_skip_reverse(ft, iter, 3);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "skip_varlen: reverse overflow expected NOT_FOUND\n");
		rcu_read_unlock();
		goto fail;
	}

	/* Skip reverse 2 from "abc" → "a". */
	/* Re-position at "abc" first. */
	s = cds_ft_lookup_nth(ft, iter, 2);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_iter_skip_reverse(ft, iter, 2);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "skip_varlen: reverse: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	rcu_read_unlock();
	if (rk_len != 1 || rk[0] != 'a') {
		fprintf(stderr, "skip_varlen: reverse got key_len %zu\n", rk_len);
		goto fail;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/* ================================================================== */
/*                                                                    */
/*                     3. LOOKUP VARIANT TESTS                        */
/*                                                                    */
/* ================================================================== */

/*
 * Exact lookup: hit and miss on a 4-byte trie.
 */
static int test_lookup_exact_hit_miss(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n = node_alloc(100);
	struct cds_ft_node *found;
	enum cds_ft_status s;

	rcu_read_lock();
	insert_u64(ft, 100, n);

	s = lookup_u64(ft, 100, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "lookup hit failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	if (to_test_node(found)->key != 100) {
		fprintf(stderr, "lookup returned wrong node\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	s = lookup_u64(ft, 999, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND || found != NULL) {
		fprintf(stderr, "lookup miss: expected NOT_FOUND, got %s (node %p)\n",
			cds_ft_status_to_string(s), (void *)found);
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);
}

/*
 * Lookup on an empty trie returns NOT_FOUND.
 */
static int test_lookup_empty_trie(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_node *found;
	enum cds_ft_status s;

	rcu_read_lock();
	s = lookup_u64(ft, 0, &found);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_NOT_FOUND || found != NULL) {
		fprintf(stderr, "lookup on empty trie: expected NOT_FOUND\n");
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * lookup_partial_key: trie has "abc" and "abcde", querying "abcde"
 * with partial should find "abcde", querying "abcdef" should find
 * "abcde" as the closest ancestor.
 */
static int test_lookup_partial(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct ft_test_node *n1 = node_alloc(0);
	struct ft_test_node *n2 = node_alloc(0);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	size_t match_len;

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"abcde", 5, &n2->node);
	if (s < 0) goto fail;

	/* Exact partial match for "abcde". */
	s = cds_ft_lookup_partial_key(ft, (const uint8_t *)"abcde", 5,
				      &match_len, &found);
	if (s != CDS_FT_STATUS_OK || !found || match_len != 5) {
		fprintf(stderr, "partial lookup 'abcde': %s match_len=%zu\n",
			cds_ft_status_to_string(s), match_len);
		goto fail;
	}

	/* Query "abcdef" (6 bytes) — closest ancestor is "abcde" (5 bytes). */
	s = cds_ft_lookup_partial_key(ft, (const uint8_t *)"abcdef", 6,
				      &match_len, &found);
	if (s != CDS_FT_STATUS_OK || !found || match_len != 5) {
		fprintf(stderr, "partial lookup 'abcdef': %s match_len=%zu\n",
			cds_ft_status_to_string(s), match_len);
		goto fail;
	}

	/* Query "xyz" — no common prefix with any key. */
	s = cds_ft_lookup_partial_key(ft, (const uint8_t *)"xyz", 3,
				      &match_len, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "partial lookup 'xyz': expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * lookup_longest_match: when only an internal node exists at the
 * longest matching prefix, status should be INTERNAL_MATCH.
 */
static int test_lookup_longest_match(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct ft_test_node *n = node_alloc(0);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	size_t match_len;

	/* Insert "abcd" only — "ab" is an internal prefix with no node. */
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"abcd", 4, &n->node);
	if (s < 0) {
		rcu_read_unlock();
		goto fail;
	}

	/* Query "abxy" — shares prefix "ab" with "abcd", but "ab" has no
	 * external node. The longest match should report 2 bytes. */
	s = cds_ft_lookup_longest_match_key(ft,
		(const uint8_t *)"abxy", 4, &match_len, &found);
	if (s != CDS_FT_STATUS_INTERNAL_MATCH) {
		fprintf(stderr, "longest_match 'abxy': expected INTERNAL_MATCH, got %s\n",
			cds_ft_status_to_string(s));
		goto unlock_fail;
	}
	if (match_len != 2) {
		fprintf(stderr, "longest_match 'abxy': match_len=%zu, expected 2\n",
			match_len);
		goto unlock_fail;
	}
	if (found != NULL) {
		fprintf(stderr, "longest_match: expected NULL node for internal match\n");
		goto unlock_fail;
	}

	/* Query "abcd" — full match, should return the node. */
	s = cds_ft_lookup_longest_match_key(ft,
		(const uint8_t *)"abcd", 4, &match_len, &found);
	if (s != CDS_FT_STATUS_OK || match_len != 4 || found != &n->node) {
		fprintf(stderr, "longest_match 'abcd': %s len=%zu\n",
			cds_ft_status_to_string(s), match_len);
		goto unlock_fail;
	}

	rcu_read_unlock();
	return drain_and_destroy(ft, group);

unlock_fail:
	rcu_read_unlock();
fail:
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Relational lookups: le, ge, lt, gt on a sparse set of keys.
 */
static int test_lookup_relational(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t keys[] = { 100, 200, 300, 400 };
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 4; i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		if (insert_u64(ft, keys[i], n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "insert key %lu failed\n",
				(unsigned long)keys[i]);
			goto out;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	{
		uint8_t k[2], rk[2];
		size_t rk_len;
		struct cds_ft_node *node;
		uint64_t found_val;

		/* lookup_le(250) → should find 200. */
		cds_ft_u64_to_key(ft, 250, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_le(ft, iter);
		node = cds_ft_iter_node(iter);
		if (!node) {
			fprintf(stderr, "le(250): no result\n");
			goto unlock_out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		found_val = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (found_val != 200) {
			fprintf(stderr, "le(250): got %" PRIu64 ", expected 200\n", found_val);
			goto unlock_out;
		}

		/* lookup_ge(250) → should find 300. */
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_ge(ft, iter);
		node = cds_ft_iter_node(iter);
		if (!node) {
			fprintf(stderr, "ge(250): no result\n");
			goto unlock_out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		found_val = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (found_val != 300) {
			fprintf(stderr, "ge(250): got %" PRIu64 ", expected 300\n", found_val);
			goto unlock_out;
		}

		/* lookup_lt(200) → should find 100. */
		cds_ft_u64_to_key(ft, 200, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_lt(ft, iter);
		node = cds_ft_iter_node(iter);
		if (!node) {
			fprintf(stderr, "lt(200): no result\n");
			goto unlock_out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		found_val = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (found_val != 100) {
			fprintf(stderr, "lt(200): got %" PRIu64 ", expected 100\n", found_val);
			goto unlock_out;
		}

		/* lookup_gt(200) → should find 300. */
		cds_ft_u64_to_key(ft, 200, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_gt(ft, iter);
		node = cds_ft_iter_node(iter);
		if (!node) {
			fprintf(stderr, "gt(200): no result\n");
			goto unlock_out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		found_val = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (found_val != 300) {
			fprintf(stderr, "gt(200): got %" PRIu64 ", expected 300\n", found_val);
			goto unlock_out;
		}

		/* lookup_lt(100) → nothing below 100, should be NOT_FOUND. */
		cds_ft_u64_to_key(ft, 100, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_lt(ft, iter);
		node = cds_ft_iter_node(iter);
		if (node) {
			fprintf(stderr, "lt(100): should have no result\n");
			goto unlock_out;
		}

		/* lookup_gt(400) → nothing above 400, should be NOT_FOUND. */
		cds_ft_u64_to_key(ft, 400, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_gt(ft, iter);
		node = cds_ft_iter_node(iter);
		if (node) {
			fprintf(stderr, "gt(400): should have no result\n");
			goto unlock_out;
		}
	}
	rcu_read_unlock();
	ret = 0;
	goto out;

unlock_out:
	rcu_read_unlock();
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * lookup_first / lookup_last on a 2-byte trie.
 */
static int test_lookup_first_last(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t keys[] = { 500, 100, 900, 300 };
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < 4; i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	{
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_lookup_first(ft, iter);
		if (!cds_ft_iter_node(iter)) {
			fprintf(stderr, "lookup_first: empty\n");
			goto unlock;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (v != 100) {
			fprintf(stderr, "lookup_first: got %" PRIu64 ", expected 100\n", v);
			goto unlock;
		}

		cds_ft_lookup_last(ft, iter);
		if (!cds_ft_iter_node(iter)) {
			fprintf(stderr, "lookup_last: empty\n");
			goto unlock;
		}
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (v != 900) {
			fprintf(stderr, "lookup_last: got %" PRIu64 ", expected 900\n", v);
			goto unlock;
		}
	}
	ret = 0;
unlock:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                       4. ITERATION TESTS                           */
/*                                                                    */
/* ================================================================== */

/*
 * Forward iteration produces strictly ascending keys.
 */
static int test_iteration_forward_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	/* Insert in deliberately non-sorted order. */
	uint64_t keys[] = { 500, 100, 300, 900, 200, 700, 400 };
	unsigned int i, count = 0;
	uint64_t prev = 0;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			fprintf(stderr, "forward order violation: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;
	}
	rcu_read_unlock();
	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "iteration error: %s\n",
			cds_ft_status_to_string(cds_ft_iter_status(iter)));
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != sizeof(keys) / sizeof(keys[0])) {
		fprintf(stderr, "forward iteration: %u nodes, expected %zu\n",
			count, sizeof(keys) / sizeof(keys[0]));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Reverse iteration produces strictly descending keys.
 */
static int test_iteration_reverse_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t keys[] = { 500, 100, 300, 900, 200, 700, 400 };
	unsigned int i, count = 0;
	uint64_t prev = UINT64_MAX;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	cds_ft_for_each_reverse_rcu(ft, iter) {
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v >= prev) {
			fprintf(stderr, "reverse order violation: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;
	}
	rcu_read_unlock();
	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "reverse iteration error\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != sizeof(keys) / sizeof(keys[0])) {
		fprintf(stderr, "reverse iteration: %u nodes, expected %zu\n",
			count, sizeof(keys) / sizeof(keys[0]));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Prefix-scoped iteration: insert keys with different prefixes,
 * then iterate only within one prefix.
 */
static int test_iteration_prefix_scoped(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = {
		"apple", "apply", "apt",
		"banana", "band",
		"cat",
	};
	unsigned int i, count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)words[i],
			      strlen(words[i]), &n->node);
		rcu_read_unlock();
	}

	/* Iterate only keys with prefix "ap" (should yield apple, apply, apt). */
	cds_ft_iter_set_key(iter, (const uint8_t *)"ap", 2);
	cds_ft_iter_set_prefix_len(iter, 2);

	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		count++;
	}
	rcu_read_unlock();
	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "prefix iteration error\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != 3) {
		fprintf(stderr, "prefix 'ap': %u keys, expected 3\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * for_each_entry_rcu: verify the typed-entry macro works.
 */
static int test_iteration_for_each_entry(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i, count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(i * 10);

		rcu_read_lock();
		insert_u64(ft, i * 10, n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	{
		struct ft_test_node *entry;

		cds_ft_for_each_entry_rcu(ft, iter, entry, node) {
			/* Just verify the entry pointer is usable. */
			(void)entry->key;
			count++;
		}
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);

	if (count != 5) {
		fprintf(stderr, "for_each_entry: %u nodes, expected 5\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * iter_copy: copying an iterator produces an independent snapshot.
 */
static int test_iter_copy(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter_a, *iter_b;
	unsigned int i;

	if (cds_ft_iter_create(ft, &iter_a) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_create(ft, &iter_b) < 0) {
		cds_ft_iter_destroy(iter_a);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(i * 100);

		rcu_read_lock();
		insert_u64(ft, i * 100, n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	/* Position iter_a at the first node. */
	cds_ft_lookup_first(ft, iter_a);
	if (!cds_ft_iter_node(iter_a)) {
		fprintf(stderr, "iter_copy: trie unexpectedly empty\n");
		rcu_read_unlock();
		goto fail;
	}

	/* Copy and advance original. */
	cds_ft_iter_copy(iter_b, iter_a);
	cds_ft_next(ft, iter_a);

	/* iter_b should still be at the first node. */
	{
		uint8_t rk_a[2], rk_b[2];
		size_t len_a, len_b;

		cds_ft_iter_get_key(iter_a, rk_a, sizeof(rk_a), &len_a);
		cds_ft_iter_get_key(iter_b, rk_b, sizeof(rk_b), &len_b);

		uint64_t va = cds_ft_key_to_u64(ft, rk_a, CDS_FT_LEN_DEFAULT);
		uint64_t vb = cds_ft_key_to_u64(ft, rk_b, CDS_FT_LEN_DEFAULT);

		if (va == vb) {
			fprintf(stderr, "iter_copy: advancing original moved copy\n");
			rcu_read_unlock();
			goto fail;
		}
		/* iter_b should be at the smallest key (0). */
		if (vb != 0) {
			fprintf(stderr, "iter_copy: copy not at first (%" PRIu64 ")\n", vb);
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter_a);
	cds_ft_iter_destroy(iter_b);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter_a);
	cds_ft_iter_destroy(iter_b);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * iter_reset clears the iterator state.
 */
static int test_iter_reset(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n = node_alloc(55);

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	insert_u64(ft, 55, n);

	/* Position the iterator. */
	cds_ft_lookup_first(ft, iter);
	if (!cds_ft_iter_node(iter)) {
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Reset — node should be NULL. */
	cds_ft_iter_reset(iter);
	if (cds_ft_iter_node(iter) != NULL) {
		fprintf(stderr, "iter_reset: node not NULL after reset\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*                   5. REPLACE & REMOVE_ALL TESTS                    */
/*                                                                    */
/* ================================================================== */

/*
 * cds_ft_replace: swap a node in-place within a duplicate chain.
 */
static int test_replace_node(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n_old = node_alloc(50);
	struct ft_test_node *n_new = node_alloc(50);
	enum cds_ft_status s;
	uint8_t k[4];

	n_old->value = 111;
	n_new->value = 222;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n_old);
		node_free(n_new);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 50, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n_old->node);
	if (s < 0) goto fail;

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;

	s = cds_ft_replace(ft, iter, &n_old->node, &n_new->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "replace failed: %s\n", cds_ft_status_to_string(s));
		goto fail;
	}

	/* Verify the new node is now findable. */
	{
		struct cds_ft_node *found;

		s = lookup_u64(ft, 50, &found);
		if (s != CDS_FT_STATUS_OK || found != &n_new->node) {
			fprintf(stderr, "lookup after replace: wrong node\n");
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Old node must be freed after a grace period. */
	node_free_rcu(n_old);

	cds_ft_iter_destroy(iter);
	/* n_new is still in the trie. */
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_remove_all: remove a 3-node duplicate chain in one call.
 */
static int test_remove_all(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(77);
	struct ft_test_node *n2 = node_alloc(77);
	struct ft_test_node *n3 = node_alloc(77);
	struct cds_ft_node *old_chain;
	unsigned long ft_count;
	enum cds_ft_status s;
	uint8_t k[4];
	int chain_len = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2); node_free(n3);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 77, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n1->node);
	cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n2->node);
	cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n3->node);

	ft_count = cds_ft_count_entries(ft);
	if (ft_count != 3) {
		fprintf(stderr, "count before remove_all: %lu\n", ft_count);
		goto fail;
	}

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;

	s = cds_ft_remove_all(ft, iter, &old_chain);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "remove_all failed: %s\n", cds_ft_status_to_string(s));
		goto fail;
	}

	ft_count = cds_ft_count_entries(ft);
	if (ft_count != 0) {
		fprintf(stderr, "count after remove_all: %lu\n", ft_count);
		goto fail;
	}

	/* Walk the removed chain. */
	{
		struct cds_ft_node *pos = old_chain;

		cds_ft_for_each_duplicate_rcu(pos)
			chain_len++;
	}
	if (chain_len != 3) {
		fprintf(stderr, "removed chain length: %d, expected 3\n", chain_len);
		goto fail;
	}

	/* Verify the key is truly gone. */
	{
		struct cds_ft_node *found;

		s = lookup_u64(ft, 77, &found);
		if (found != NULL) {
			fprintf(stderr, "key still present after remove_all\n");
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Free the removed chain after a grace period. */
	node_free_rcu(n1);
	node_free_rcu(n2);
	node_free_rcu(n3);

	cds_ft_iter_destroy(iter);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Remove a single node from the middle of a duplicate chain, verify
 * the remaining chain is intact.
 */
static int test_remove_middle_of_chain(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *nodes[3];
	enum cds_ft_status s;
	uint8_t k[4];
	unsigned int i;
	int count;

	for (i = 0; i < 3; i++)
		nodes[i] = node_alloc(88);

	if (cds_ft_iter_create(ft, &iter) < 0) {
		for (i = 0; i < 3; i++) node_free(nodes[i]);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 88, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	for (i = 0; i < 3; i++) {
		s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &nodes[i]->node);
		if (s < 0) goto fail;
	}

	/* Remove the second node inserted (nodes[1]). */
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;

	s = cds_ft_remove(ft, iter, &nodes[1]->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "remove middle node: %s\n", cds_ft_status_to_string(s));
		goto fail;
	}

	/* Count remaining duplicates. */
	{
		struct cds_ft_node *head;

		s = lookup_u64(ft, 88, &head);
		if (s != CDS_FT_STATUS_OK || !head) {
			fprintf(stderr, "key gone after removing one duplicate\n");
			goto fail;
		}
		count = 0;
		cds_ft_for_each_duplicate_rcu(head)
			count++;
		if (count != 2) {
			fprintf(stderr, "remaining chain: %d, expected 2\n", count);
			goto fail;
		}
	}
	rcu_read_unlock();

	node_free_rcu(nodes[1]);

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/* ================================================================== */
/*                                                                    */
/*               6. KEY CONVERSION HELPER TESTS                       */
/*                                                                    */
/* ================================================================== */

/*
 * Round-trip u64 → key → u64 for every key width 1–8.
 */
static int test_key_u64_roundtrip(void)
{
	struct cds_ft_group *group;
	unsigned int klen;

	for (klen = 1; klen <= 8; klen++) {
		struct cds_ft *ft = create_fixed_ft(klen, &group);
		uint64_t max_val = (klen == 8) ? UINT64_MAX
			: (1ULL << (klen * 8)) - 1;
		/* Test several values including boundaries. */
		uint64_t vals[] = { 0, 1, max_val / 2, max_val - 1, max_val };
		unsigned int i;

		for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
			uint8_t k[8];
			uint64_t back;

			if (vals[i] > max_val)
				continue;
			cds_ft_u64_to_key(ft, vals[i], k, CDS_FT_LEN_DEFAULT);
			back = cds_ft_key_to_u64(ft, k, CDS_FT_LEN_DEFAULT);
			if (back != vals[i]) {
				fprintf(stderr, "u64 roundtrip klen=%u: %" PRIu64 " → %" PRIu64 "\n",
					klen, vals[i], back);
				cds_ft_destroy(ft);
				cds_ft_group_destroy(group);
				return -1;
			}
		}
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
	}
	return 0;
}

/*
 * Round-trip u32 → key → u32 for key widths 1–4.
 */
static int test_key_u32_roundtrip(void)
{
	struct cds_ft_group *group;
	unsigned int klen;

	for (klen = 1; klen <= 4; klen++) {
		struct cds_ft *ft = create_fixed_ft(klen, &group);
		uint32_t max_val = (klen == 4) ? UINT32_MAX
			: (uint32_t)((1ULL << (klen * 8)) - 1);
		uint32_t vals[] = { 0, 1, max_val / 2, max_val - 1, max_val };
		unsigned int i;

		for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
			uint8_t k[4];
			uint32_t back;

			if (vals[i] > max_val)
				continue;
			cds_ft_u32_to_key(ft, vals[i], k, CDS_FT_LEN_DEFAULT);
			back = cds_ft_key_to_u32(ft, k, CDS_FT_LEN_DEFAULT);
			if (back != vals[i]) {
				fprintf(stderr, "u32 roundtrip klen=%u: %" PRIu32 " → %" PRIu32 "\n",
					klen, vals[i], back);
				cds_ft_destroy(ft);
				cds_ft_group_destroy(group);
				return -1;
			}
		}
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
	}
	return 0;
}

/*
 * Signed s64 round-trip including negatives and extremes.
 */
static int test_key_s64_roundtrip(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(8, &group);
	int64_t vals[] = { INT64_MIN, INT64_MIN + 1, -1, 0, 1,
			   INT64_MAX - 1, INT64_MAX };
	unsigned int i;

	for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
		uint8_t k[8];
		int64_t back;

		cds_ft_s64_to_key(ft, vals[i], k, CDS_FT_LEN_DEFAULT);
		back = cds_ft_key_to_s64(ft, k, CDS_FT_LEN_DEFAULT);
		if (back != vals[i]) {
			fprintf(stderr, "s64 roundtrip: %" PRId64 " → %" PRId64 "\n",
				vals[i], back);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Signed s32 round-trip.
 */
static int test_key_s32_roundtrip(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	int32_t vals[] = { INT32_MIN, INT32_MIN + 1, -1, 0, 1,
			   INT32_MAX - 1, INT32_MAX };
	unsigned int i;

	for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
		uint8_t k[4];
		int32_t back;

		cds_ft_s32_to_key(ft, vals[i], k, CDS_FT_LEN_DEFAULT);
		back = cds_ft_key_to_s32(ft, k, CDS_FT_LEN_DEFAULT);
		if (back != vals[i]) {
			fprintf(stderr, "s32 roundtrip: %" PRId32 " → %" PRId32 "\n",
				vals[i], back);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Verify that the signed-key encoding preserves sort order:
 * INT64_MIN sorts first, INT64_MAX sorts last.
 */
static int test_key_signed_sort_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(8, &group);
	struct cds_ft_iter *iter;
	int64_t vals[] = { 0, -1, INT64_MAX, INT64_MIN, 1, -100, 100 };
	unsigned int i, count = 0;
	int64_t prev = 0;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
		struct ft_test_node *n = node_alloc((uint64_t)vals[i]);
		uint8_t k[8];

		cds_ft_s64_to_key(ft, vals[i], k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
		rcu_read_unlock();
	}

	/* Forward iteration should produce signed ascending order. */
	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[8];
		size_t rk_len;
		int64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_s64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			fprintf(stderr, "signed sort violation: %" PRId64 " after %" PRId64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);

	if (count != sizeof(vals) / sizeof(vals[0])) {
		fprintf(stderr, "signed sort: %u nodes, expected %zu\n",
			count, sizeof(vals) / sizeof(vals[0]));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*              7. NIL KEY (ZERO-LENGTH) TESTS                        */
/*                                                                    */
/* ================================================================== */

/*
 * Insert, lookup, iterate, and remove a NIL key on a variable-length trie.
 */
static int test_nil_key_varlen(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n_nil = node_alloc(0);
	struct ft_test_node *n_abc = node_alloc(0);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned int count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n_nil); node_free(n_abc);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	/* Insert NIL key (length 0). */
	s = cds_ft_insert(ft, NULL, 0, &n_nil->node);
	if (s < 0) {
		fprintf(stderr, "insert NIL key: %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	/* Insert a normal key alongside. */
	s = cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n_abc->node);
	if (s < 0) {
		fprintf(stderr, "insert 'abc': %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Lookup NIL key. */
	s = cds_ft_lookup_key(ft, NULL, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "lookup NIL: %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Iterate — should see 2 nodes. */
	cds_ft_for_each_rcu(ft, iter)
		count++;

	if (count != 2) {
		fprintf(stderr, "iteration with NIL key: %u nodes, expected 2\n", count);
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/* ================================================================== */
/*                                                                    */
/*                    8. BOUNDARY / ERROR TESTS                       */
/*                                                                    */
/* ================================================================== */

/*
 * Insert all 256 one-byte keys, verify count and sorted iteration,
 * then remove all and verify empty.
 */
static int test_1byte_exhaustive(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(1, &group);
	struct cds_ft_iter *iter;
	unsigned int i, count;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) {
		fprintf(stderr, "1byte exhaustive: count %lu\n", ft_count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Verify ascending order. */
	count = 0;
	rcu_read_lock();
	{
		uint64_t prev = 0;
		int first = 1;

		cds_ft_for_each_rcu(ft, iter) {
			uint8_t rk[1];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
			if (!first && v <= prev) {
				fprintf(stderr, "1byte order: %" PRIu64 " after %" PRIu64 "\n", v, prev);
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
			prev = v;
			first = 0;
			count++;
		}
	}
	rcu_read_unlock();

	if (count != 256) {
		fprintf(stderr, "1byte iteration: %u nodes\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Common-prefix split: insert "abc", then "abd", remove "abc",
 * verify "abd" is still reachable.
 */
static int test_prefix_split(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(0);
	struct ft_test_node *n2 = node_alloc(0);
	struct cds_ft_node *found;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1); node_free(n2);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n1->node);
	cds_ft_insert(ft, (const uint8_t *)"abd", 3, &n2->node);

	/* Remove "abc". */
	cds_ft_iter_set_key(iter, (const uint8_t *)"abc", 3);
	cds_ft_lookup(ft, iter);
	s = cds_ft_remove(ft, iter, &n1->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "remove 'abc': %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	node_free_rcu(n1);

	/* "abd" should still be there. */
	s = cds_ft_lookup_key(ft, (const uint8_t *)"abd", 3, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "'abd' unreachable after removing 'abc'\n");
		rcu_read_unlock();
		goto fail;
	}
	/* "abc" should be gone. */
	s = cds_ft_lookup_key(ft, (const uint8_t *)"abc", 3, &found);
	if (found != NULL) {
		fprintf(stderr, "'abc' still present after removal\n");
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * iter_get_key with a too-small buffer returns OVERFLOW_ERROR.
 */
static int test_iter_get_key_overflow(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n = node_alloc(0);
	enum cds_ft_status s;
	uint8_t tiny[1];
	size_t out_len;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"longkey", 7, &n->node);
	cds_ft_lookup_first(ft, iter);
	if (!cds_ft_iter_node(iter)) {
		rcu_read_unlock();
		goto fail;
	}

	/* Buffer too small (1 byte for a 7-byte key). */
	s = cds_ft_iter_get_key(iter, tiny, sizeof(tiny), &out_len);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_OVERFLOW_ERROR) {
		fprintf(stderr, "expected OVERFLOW_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * iter_set_prefix_len with prefix > key length returns error.
 */
static int test_iter_prefix_len_invalid(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Set a 3-byte key, then try prefix_len=5. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"abc", 3);
	s = cds_ft_iter_set_prefix_len(iter, 5);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "expected INVALID_ARGUMENT_ERROR for prefix_len > key_len, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Remove a node that was already removed returns NOT_FOUND.
 */
static int test_double_remove(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n = node_alloc(33);
	enum cds_ft_status s;
	uint8_t k[4];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 33, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);

	s = cds_ft_remove(ft, iter, &n->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "first remove: %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Re-lookup — the key should be gone now. */
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	s = cds_ft_remove(ft, iter, &n->node);
	rcu_read_unlock();

	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "second remove: should NOT succeed\n");
		goto fail;
	}

	node_free_rcu(n);
	cds_ft_iter_destroy(iter);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Ordered iteration is correct after inserting a new key in the
 * middle of an existing population.
 */
static int test_order_after_mid_insert(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t initial[] = { 100, 300, 500, 700 };
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 4; i++) {
		struct ft_test_node *n = node_alloc(initial[i]);

		rcu_read_lock();
		insert_u64(ft, initial[i], n);
		rcu_read_unlock();
	}

	/* Insert 400 in the gap between 300 and 500. */
	{
		struct ft_test_node *n = node_alloc(400);

		rcu_read_lock();
		insert_u64(ft, 400, n);
		rcu_read_unlock();
	}

	/* Verify full order: 100, 300, 400, 500, 700. */
	{
		uint64_t expected[] = { 100, 300, 400, 500, 700 };
		unsigned int idx = 0;

		rcu_read_lock();
		cds_ft_for_each_rcu(ft, iter) {
			uint8_t rk[2];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
			if (idx >= 5) {
				fprintf(stderr, "too many nodes in iteration\n");
				rcu_read_unlock();
				goto out;
			}
			if (v != expected[idx]) {
				fprintf(stderr, "order[%u]: got %" PRIu64 ", expected %" PRIu64 "\n",
					idx, v, expected[idx]);
				rcu_read_unlock();
				goto out;
			}
			idx++;
		}
		rcu_read_unlock();
		if (cds_ft_iter_status(iter) < 0) {
			fprintf(stderr, "iteration error\n");
			goto out;
		}
		if (idx != 5) {
			fprintf(stderr, "only %u nodes, expected 5\n", idx);
			goto out;
		}
	}
	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Variable-length string keys: insert, lookup, sorted iteration,
 * removal.
 */
static int test_varlen_string_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	/* Strings chosen to share prefixes and exercise internal splits. */
	const char *words[] = {
		"apple", "application", "app",
		"banana", "band", "ban",
		"z",
	};
	unsigned int i, count = 0;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert. */
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *)words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}

	/* Lookup each. */
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct cds_ft_node *found;

		rcu_read_lock();
		if (cds_ft_lookup_key(ft, (const uint8_t *)words[i],
				      strlen(words[i]), &found) != CDS_FT_STATUS_OK
		    || !found) {
			fprintf(stderr, "lookup '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}

	/* Sorted iteration: verify ascending lexicographic order. */
	rcu_read_lock();
	{
		char prev[256] = "";

		cds_ft_for_each_rcu(ft, iter) {
			uint8_t rk[256];
			size_t rk_len;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			rk[rk_len] = '\0';
			if (prev[0] && strcmp((char *)rk, prev) <= 0) {
				fprintf(stderr, "string order: '%s' after '%s'\n",
					(char *)rk, prev);
				rcu_read_unlock();
				goto out;
			}
			memcpy(prev, rk, rk_len + 1);
			count++;
		}
	}
	rcu_read_unlock();
	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "iteration error\n");
		goto out;
	}

	if (count != sizeof(words) / sizeof(words[0])) {
		fprintf(stderr, "string iteration: %u words, expected %zu\n",
			count, sizeof(words) / sizeof(words[0]));
		goto out;
	}

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                9. GRAFT, GRAFT_SWAP & DETACH TESTS                 */
/*                                                                    */
/* ================================================================== */

/*
 * Helper: drain every node from @ft without destroying the group.
 * Returns 0 on success, -1 on error.
 */
static int drain_trie(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = 0;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0)
		return -1;

	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return ret;
}

/*
 * Graft basic: populate a staging trie offline, graft it into a live
 * trie at a prefix, verify all grafted keys are reachable and that
 * the staging trie is empty afterward.
 */
static int test_graft_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Phase 1: populate staging offline (no lock needed). */
	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		s = cds_ft_insert(staging, (const uint8_t *)"lo", 2, &n1->node);
		if (s < 0) goto fail;
		s = cds_ft_insert(staging, (const uint8_t *)"lp", 2, &n2->node);
		if (s < 0) goto fail;
	}

	/* Phase 2: graft staging into live at prefix "he". */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"he", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_basic: graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Staging should be empty. */
	if (!cds_ft_empty(staging)) {
		fprintf(stderr, "graft_basic: staging not empty after graft\n");
		goto fail;
	}

	/* Live trie should contain "helo" and "help". */
	rcu_read_lock();
	s = cds_ft_lookup_key(live, (const uint8_t *)"helo", 4, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_basic: lookup 'helo': %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_lookup_key(live, (const uint8_t *)"help", 4, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_basic: lookup 'help': %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	count = cds_ft_count_entries(live);
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "graft_basic: live count %lu, expected 2\n", count);
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft at root: graft an entire staging trie at the root (key_len=0).
 */
static int test_graft_at_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	enum cds_ft_status s;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	{
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(staging, (const uint8_t *)"abc", 3, &n->node);
		if (s < 0) goto fail;
	}

	rcu_read_lock();
	s = cds_ft_graft(live, NULL, 0, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_at_root: graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Key should be "abc" — no prefix prepended. */
	rcu_read_lock();
	s = cds_ft_lookup_key(live, (const uint8_t *)"abc", 3, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_at_root: lookup 'abc': %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft into a populated destination returns POPULATED_ERROR.
 */
static int test_graft_populated_error(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live at key "ab". */
	{
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		s = cds_ft_insert(live, (const uint8_t *)"ab", 2, &n->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* Populate staging with something to graft. */
	{
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(staging, (const uint8_t *)"cd", 2, &n->node);
		if (s < 0) goto fail;
	}

	/* Graft staging at "ab" should fail — "ab" is already populated. */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"ab", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_POPULATED_ERROR) {
		fprintf(stderr, "graft_populated: expected POPULATED_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Staging should still hold its content (graft failed). */
	if (cds_ft_empty(staging)) {
		fprintf(stderr, "graft_populated: staging should not be empty after failed graft\n");
		goto fail;
	}

	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft with tries from different groups returns INVALID_ARGUMENT_ERROR.
 */
static int test_graft_different_group_error(void)
{
	struct cds_ft_group *group1, *group2;
	struct cds_ft *ft1, *ft2;
	enum cds_ft_status s;

	ft1 = create_varlen_ft(&group1);
	ft2 = create_varlen_ft(&group2);

	rcu_read_lock();
	s = cds_ft_graft(ft1, NULL, 0, ft2);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_different_group: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft2);
		cds_ft_group_destroy(group2);
		cds_ft_destroy(ft1);
		cds_ft_group_destroy(group1);
		return -1;
	}

	cds_ft_destroy(ft2);
	cds_ft_group_destroy(group2);
	cds_ft_destroy(ft1);
	cds_ft_group_destroy(group1);
	return 0;
}

/*
 * Graft a trie into itself returns INVALID_ARGUMENT_ERROR.
 */
static int test_graft_self_error(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	rcu_read_lock();
	s = cds_ft_graft(ft, NULL, 0, ft);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_self: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Graft that would exceed the group's max key length returns
 * OVERFLOW_ERROR.
 */
static int test_graft_overflow_error(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;

	/* Create a group with max_key_len = 4. */
	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert a 3-byte key into staging. */
	{
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(staging, (const uint8_t *)"abc", 3, &n->node);
		if (s < 0) {
			node_free(n);
			goto cleanup;
		}
	}

	/*
	 * Graft staging at a 2-byte prefix: combined length = 2 + 3 = 5,
	 * which exceeds max_key_len = 4.
	 */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"XY", 2, staging);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_OVERFLOW_ERROR) {
		fprintf(stderr, "graft_overflow: expected OVERFLOW_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		drain_trie(staging);
		drain_trie(live);
		rcu_barrier();
		cds_ft_destroy(staging);
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Staging should still own its content. */
	if (cds_ft_empty(staging)) {
		fprintf(stderr, "graft_overflow: staging empty after failed graft\n");
		drain_trie(live);
		drain_trie(staging);
		rcu_barrier();
		cds_ft_destroy(staging);
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	drain_trie(staging);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

cleanup:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * graft_swap basic: the swap trie's content replaces whatever is at
 * the graft point, and the old content moves into the swap trie.
 */
static int test_graft_swap_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live with keys under prefix "ab": "abX" and "abY". */
	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		n1->value = 1;
		n2->value = 2;
		rcu_read_lock();
		s = cds_ft_insert(live, (const uint8_t *)"abX", 3, &n1->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		s = cds_ft_insert(live, (const uint8_t *)"abY", 3, &n2->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		rcu_read_unlock();
	}

	/* Populate swap with keys that will replace the "ab" subtree. */
	{
		struct ft_test_node *n = node_alloc(0);

		n->value = 99;
		s = cds_ft_insert(swap, (const uint8_t *)"Z", 1, &n->node);
		if (s < 0) goto fail;
	}

	/* Swap at prefix "ab". */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, (const uint8_t *)"ab", 2, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_swap_basic: swap: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should now have "abZ" (value 99), not "abX"/"abY". */
	rcu_read_lock();
	s = cds_ft_lookup_key(live, (const uint8_t *)"abZ", 3, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_swap_basic: lookup 'abZ' failed: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 99) {
		fprintf(stderr, "graft_swap_basic: 'abZ' has wrong value\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_lookup_key(live, (const uint8_t *)"abX", 3, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "graft_swap_basic: 'abX' should be gone\n");
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	/* Swap trie should now contain the old content: "X" and "Y"
	 * (prefix "ab" stripped). */
	if (cds_ft_empty(swap)) {
		fprintf(stderr, "graft_swap_basic: swap trie is empty, expected old content\n");
		goto fail;
	}
	rcu_read_lock();
	count = cds_ft_count_entries(swap);
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "graft_swap_basic: swap count %lu, expected 2\n", count);
		goto fail;
	}

	/* After a grace period, drain the old content from swap. */
	synchronize_rcu();
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * graft_swap into an empty position: the swap trie's content is
 * grafted and the swap trie becomes empty (nothing was at that key).
 */
static int test_graft_swap_into_empty(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Swap trie has one node. */
	{
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(swap, (const uint8_t *)"cd", 2, &n->node);
		if (s < 0) goto fail;
	}

	/* Swap at prefix "ab" where live is empty. */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, (const uint8_t *)"ab", 2, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_swap_into_empty: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Swap should be empty — nothing was at "ab" before. */
	if (!cds_ft_empty(swap)) {
		fprintf(stderr, "graft_swap_into_empty: swap trie not empty\n");
		goto fail;
	}

	/* Live should contain "abcd". */
	rcu_read_lock();
	s = cds_ft_lookup_key(live, (const uint8_t *)"abcd", 4, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_swap_into_empty: lookup 'abcd': %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * graft_swap at root: exchange the entire trie content.
 */
static int test_graft_swap_at_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long live_count, swap_count;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live with "aa" and "bb". */
	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(live, (const uint8_t *)"aa", 2, &n1->node);
		cds_ft_insert(live, (const uint8_t *)"bb", 2, &n2->node);
		rcu_read_unlock();
	}

	/* Populate swap with "xx". */
	{
		struct ft_test_node *n = node_alloc(0);

		cds_ft_insert(swap, (const uint8_t *)"xx", 2, &n->node);
	}

	/* Swap at root. */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, NULL, 0, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_swap_at_root: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should now have "xx", swap should have "aa" and "bb". */
	rcu_read_lock();
	live_count = cds_ft_count_entries(live);
	swap_count = cds_ft_count_entries(swap);
	s = cds_ft_lookup_key(live, (const uint8_t *)"xx", 2, &found);
	rcu_read_unlock();

	if (live_count != 1 || s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_swap_at_root: live should have 'xx' only (count=%lu)\n",
			live_count);
		goto fail;
	}
	if (swap_count != 2) {
		fprintf(stderr, "graft_swap_at_root: swap count %lu, expected 2\n",
			swap_count);
		goto fail;
	}

	synchronize_rcu();
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * graft_swap a trie with itself returns INVALID_ARGUMENT_ERROR.
 */
static int test_graft_swap_self_error(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	rcu_read_lock();
	s = cds_ft_graft_swap(ft, NULL, 0, ft);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_swap_self: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * graft_swap with tries from different groups returns
 * INVALID_ARGUMENT_ERROR.
 */
static int test_graft_swap_different_group_error(void)
{
	struct cds_ft_group *group1, *group2;
	struct cds_ft *ft1, *ft2;
	enum cds_ft_status s;

	ft1 = create_varlen_ft(&group1);
	ft2 = create_varlen_ft(&group2);

	rcu_read_lock();
	s = cds_ft_graft_swap(ft1, NULL, 0, ft2);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_swap_different_group: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft2);
		cds_ft_group_destroy(group2);
		cds_ft_destroy(ft1);
		cds_ft_group_destroy(group1);
		return -1;
	}

	cds_ft_destroy(ft2);
	cds_ft_group_destroy(group2);
	cds_ft_destroy(ft1);
	cds_ft_group_destroy(group1);
	return 0;
}

/*
 * Detach basic: insert nodes sharing a prefix, detach that prefix,
 * verify the detached trie contains the nodes (with stripped keys)
 * and the original trie no longer has them.
 */
static int test_detach_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;

	ft = create_varlen_ft(&group);

	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);
		struct ft_test_node *n3 = node_alloc(0);

		n1->value = 10;
		n2->value = 20;
		n3->value = 30;

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)"abX", 3, &n1->node);
		cds_ft_insert(ft, (const uint8_t *)"abY", 3, &n2->node);
		/* A key outside the detach prefix. */
		cds_ft_insert(ft, (const uint8_t *)"cd",  2, &n3->node);
		rcu_read_unlock();
	}

	/* Detach everything under "ab". */
	rcu_read_lock();
	s = cds_ft_detach(ft, (const uint8_t *)"ab", 2, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "detach_basic: detach: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Original trie should only have "cd". */
	rcu_read_lock();
	count = cds_ft_count_entries(ft);
	s = cds_ft_lookup_key(ft, (const uint8_t *)"abX", 3, &found);
	rcu_read_unlock();
	if (count != 1) {
		fprintf(stderr, "detach_basic: original count %lu, expected 1\n", count);
		goto fail;
	}
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "detach_basic: 'abX' still in original after detach\n");
		goto fail;
	}

	/* Detached trie should have "X" and "Y" (prefix "ab" stripped). */
	rcu_read_lock();
	count = cds_ft_count_entries(detached);
	s = cds_ft_lookup_key(detached, (const uint8_t *)"X", 1, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "detach_basic: lookup 'X' in detached: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 10) {
		fprintf(stderr, "detach_basic: 'X' wrong value\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_lookup_key(detached, (const uint8_t *)"Y", 1, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "detach_basic: lookup 'Y' in detached: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (count != 2) {
		fprintf(stderr, "detach_basic: detached count %lu, expected 2\n", count);
		goto fail;
	}

	/* Bulk-removal pattern: grace period, then drain locally. */
	synchronize_rcu();
	drain_trie(detached);
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(detached);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Detach at root: detach everything from the trie.
 */
static int test_detach_at_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	enum cds_ft_status s;
	unsigned long count;

	ft = create_varlen_ft(&group);

	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)"foo", 3, &n1->node);
		cds_ft_insert(ft, (const uint8_t *)"bar", 3, &n2->node);
		rcu_read_unlock();
	}

	rcu_read_lock();
	s = cds_ft_detach(ft, NULL, 0, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "detach_at_root: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Original should be empty. */
	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "detach_at_root: original not empty\n");
		goto fail;
	}

	/* Detached should have both nodes with same keys. */
	rcu_read_lock();
	count = cds_ft_count_entries(detached);
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "detach_at_root: detached count %lu, expected 2\n", count);
		goto fail;
	}

	synchronize_rcu();
	drain_trie(detached);
	rcu_barrier();
	cds_ft_destroy(detached);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Detach at a key with no content returns NOT_FOUND.
 */
static int test_detach_not_found(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	/* Insert at "foo", detach at "bar" — nothing there. */
	{
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)"foo", 3, &n->node);
		rcu_read_unlock();
	}

	rcu_read_lock();
	s = cds_ft_detach(ft, (const uint8_t *)"bar", 3, &detached);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "detach_not_found: expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		if (detached) {
			drain_trie(detached);
			cds_ft_destroy(detached);
		}
		drain_trie(ft);
		rcu_barrier();
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Original trie should still have "foo". */
	rcu_read_lock();
	{
		struct cds_ft_node *found;

		s = cds_ft_lookup_key(ft, (const uint8_t *)"foo", 3, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "detach_not_found: 'foo' missing after failed detach\n");
			rcu_read_unlock();
			drain_trie(ft);
			rcu_barrier();
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
	}
	rcu_read_unlock();

	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Detach, then graft the detached trie back at a different prefix.
 */
static int test_detach_then_graft(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;

	ft = create_varlen_ft(&group);

	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		n1->value = 1;
		n2->value = 2;

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)"abX", 3, &n1->node);
		cds_ft_insert(ft, (const uint8_t *)"abY", 3, &n2->node);
		rcu_read_unlock();
	}

	/* Detach "ab" subtree. */
	rcu_read_lock();
	s = cds_ft_detach(ft, (const uint8_t *)"ab", 2, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "detach_then_graft: detach: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Graft detached content at new prefix "zz". */
	rcu_read_lock();
	s = cds_ft_graft(ft, (const uint8_t *)"zz", 2, detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "detach_then_graft: graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Original "ab*" keys should be gone; "zz*" should exist. */
	rcu_read_lock();
	s = cds_ft_lookup_key(ft, (const uint8_t *)"abX", 3, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "detach_then_graft: 'abX' still present\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_lookup_key(ft, (const uint8_t *)"zzX", 3, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "detach_then_graft: lookup 'zzX': %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 1) {
		fprintf(stderr, "detach_then_graft: 'zzX' wrong value\n");
		rcu_read_unlock();
		goto fail;
	}
	count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "detach_then_graft: count %lu, expected 2\n", count);
		goto fail;
	}

	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(detached);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Detach on an empty trie returns NOT_FOUND.
 */
static int test_detach_empty_trie(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);

	rcu_read_lock();
	s = cds_ft_detach(ft, NULL, 0, &detached);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "detach_empty_trie: expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		if (detached) {
			cds_ft_destroy(detached);
		}
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * graft_swap with integer keys encoded as 4-byte big-endian values:
 * verify the root-level swap (bulk-load) pattern works.
 *
 * A variable-length trie is used because fixed-length tries require
 * the key_len parameter to match the configured length exactly; they
 * do not accept key_len=0 for root-level operations. The key
 * conversion helpers are called with an explicit length of 4.
 */
static int test_graft_swap_fixed_key(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	/*
	 * Variable-length trie with max_key_len = 4 so that
	 * root-level swap (key_len = 0) is accepted while key
	 * values match the 4-byte integer encoding.
	 */
	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_max_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live with keys 100, 200. */
	{
		struct ft_test_node *n1 = node_alloc(100);
		struct ft_test_node *n2 = node_alloc(200);

		cds_ft_u64_to_key(live, 100, k, klen);
		rcu_read_lock();
		s = cds_ft_insert(live, k, klen, &n1->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		cds_ft_u64_to_key(live, 200, k, klen);
		s = cds_ft_insert(live, k, klen, &n2->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		rcu_read_unlock();
	}

	/* Populate swap with keys 300, 400, 500. */
	{
		unsigned int i;
		uint64_t vals[] = { 300, 400, 500 };

		for (i = 0; i < 3; i++) {
			struct ft_test_node *n = node_alloc(vals[i]);

			cds_ft_u64_to_key(swap, vals[i], k, klen);
			s = cds_ft_insert(swap, k, klen, &n->node);
			if (s < 0) goto fail;
		}
	}

	/* Swap at root — exchange everything. */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, NULL, 0, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_swap_fixed_key: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should have 300, 400, 500. */
	rcu_read_lock();
	count = cds_ft_count_entries(live);
	cds_ft_u64_to_key(live, 300, k, klen);
	s = cds_ft_lookup_key(live, k, klen, &found);
	rcu_read_unlock();
	if (count != 3 || s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_swap_fixed_key: live count %lu, lookup 300: %s\n",
			count, cds_ft_status_to_string(s));
		goto fail;
	}

	/* Swap should have 100, 200. */
	rcu_read_lock();
	count = cds_ft_count_entries(swap);
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "graft_swap_fixed_key: swap count %lu, expected 2\n", count);
		goto fail;
	}

	synchronize_rcu();
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft at root on a fixed-length trie: populate a staging trie
 * with integer keys, graft it into an empty live trie at root
 * (key_len=0), and verify all nodes are reachable in the live trie.
 */
static int test_fixed_graft_at_root(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate staging with keys 10, 20, 30. */
	{
		unsigned int i;
		uint64_t vals[] = { 10, 20, 30 };

		for (i = 0; i < 3; i++) {
			struct ft_test_node *n = node_alloc(vals[i]);

			cds_ft_u64_to_key(staging, vals[i], k, klen);
			s = cds_ft_insert(staging, k, klen, &n->node);
			if (s < 0) goto fail;
		}
	}

	/* Graft staging into live at root. */
	rcu_read_lock();
	s = cds_ft_graft(live, NULL, 0, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "fixed_graft_at_root: graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Staging should be empty. */
	if (!cds_ft_empty(staging)) {
		fprintf(stderr, "fixed_graft_at_root: staging not empty after graft\n");
		goto fail;
	}

	/* Live should contain all three keys. */
	rcu_read_lock();
	count = cds_ft_count_entries(live);
	cds_ft_u64_to_key(live, 10, k, klen);
	s = cds_ft_lookup_key(live, k, klen, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_graft_at_root: lookup 10: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_u64_to_key(live, 30, k, klen);
	s = cds_ft_lookup_key(live, k, klen, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_graft_at_root: lookup 30: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (count != 3) {
		fprintf(stderr, "fixed_graft_at_root: count %lu, expected 3\n", count);
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Non-root graft on a fixed-length trie returns INVALID_ARGUMENT_ERROR.
 * Fixed-length groups require key_len to match the configured length
 * exactly; a shorter prefix cannot address an interior graft point.
 */
static int test_fixed_graft_nonroot_error(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Put one node into staging so it is non-empty. */
	{
		struct ft_test_node *n = node_alloc(42);

		cds_ft_u64_to_key(staging, 42, k, klen);
		s = cds_ft_insert(staging, k, klen, &n->node);
		if (s < 0) goto fail;
	}

	/* Attempt a non-root graft (key_len=2 < fixed klen=4). */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"\x00\x01", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "fixed_graft_nonroot: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Staging should still hold its content (graft failed). */
	if (cds_ft_empty(staging)) {
		fprintf(stderr, "fixed_graft_nonroot: staging should not be empty\n");
		goto fail;
	}

	drain_trie(staging);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * graft_swap at root on a fixed-length trie: exchange the entire
 * content of two fixed-length tries using a root-level swap.
 */
static int test_fixed_graft_swap_at_root(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live with keys 100, 200. */
	{
		struct ft_test_node *n1 = node_alloc(100);
		struct ft_test_node *n2 = node_alloc(200);

		n1->value = 1;
		n2->value = 2;
		cds_ft_u64_to_key(live, 100, k, klen);
		rcu_read_lock();
		s = cds_ft_insert(live, k, klen, &n1->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		cds_ft_u64_to_key(live, 200, k, klen);
		s = cds_ft_insert(live, k, klen, &n2->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		rcu_read_unlock();
	}

	/* Populate swap with keys 300, 400, 500. */
	{
		unsigned int i;
		uint64_t vals[] = { 300, 400, 500 };

		for (i = 0; i < 3; i++) {
			struct ft_test_node *n = node_alloc(vals[i]);

			n->value = vals[i];
			cds_ft_u64_to_key(swap, vals[i], k, klen);
			s = cds_ft_insert(swap, k, klen, &n->node);
			if (s < 0) goto fail;
		}
	}

	/* Root-level swap: exchange everything. */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, NULL, 0, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "fixed_graft_swap_at_root: swap: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Live should now have 300, 400, 500. */
	rcu_read_lock();
	count = cds_ft_count_entries(live);
	cds_ft_u64_to_key(live, 300, k, klen);
	s = cds_ft_lookup_key(live, k, klen, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_graft_swap_at_root: lookup 300: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 300) {
		fprintf(stderr, "fixed_graft_swap_at_root: 300 wrong value\n");
		rcu_read_unlock();
		goto fail;
	}
	/* Key 100 should no longer be in live. */
	cds_ft_u64_to_key(live, 100, k, klen);
	s = cds_ft_lookup_key(live, k, klen, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "fixed_graft_swap_at_root: key 100 still in live\n");
		goto fail;
	}
	if (count != 3) {
		fprintf(stderr, "fixed_graft_swap_at_root: live count %lu, expected 3\n",
			count);
		goto fail;
	}

	/* Swap should now have 100, 200 (the old live content). */
	rcu_read_lock();
	count = cds_ft_count_entries(swap);
	cds_ft_u64_to_key(swap, 100, k, klen);
	s = cds_ft_lookup_key(swap, k, klen, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_graft_swap_at_root: lookup 100 in swap: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 1) {
		fprintf(stderr, "fixed_graft_swap_at_root: 100 wrong value in swap\n");
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "fixed_graft_swap_at_root: swap count %lu, expected 2\n",
			count);
		goto fail;
	}

	synchronize_rcu();
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Non-root graft_swap on a fixed-length trie returns
 * INVALID_ARGUMENT_ERROR.
 */
static int test_fixed_graft_swap_nonroot_error(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live so the swap point is non-trivial. */
	{
		struct ft_test_node *n = node_alloc(0x01020304);

		cds_ft_u64_to_key(live, 0x01020304, k, klen);
		rcu_read_lock();
		s = cds_ft_insert(live, k, klen, &n->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* Attempt non-root graft_swap (key_len=2 < fixed klen=4). */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, (const uint8_t *)"\x01\x02", 2, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "fixed_graft_swap_nonroot: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Detach at root on a fixed-length trie: populate a trie, detach
 * everything at root, verify the original is empty and the detached
 * trie contains all nodes.
 */
static int test_fixed_detach_at_root(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate with keys 5, 10, 15. */
	{
		unsigned int i;
		uint64_t vals[] = { 5, 10, 15 };

		for (i = 0; i < 3; i++) {
			struct ft_test_node *n = node_alloc(vals[i]);

			n->value = vals[i];
			cds_ft_u64_to_key(ft, vals[i], k, klen);
			rcu_read_lock();
			s = cds_ft_insert(ft, k, klen, &n->node);
			rcu_read_unlock();
			if (s < 0) goto fail;
		}
	}

	/* Detach everything at root. */
	rcu_read_lock();
	s = cds_ft_detach(ft, NULL, 0, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "fixed_detach_at_root: detach: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Original should be empty. */
	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "fixed_detach_at_root: original not empty\n");
		goto fail;
	}

	/* Detached should have all three nodes with same keys. */
	rcu_read_lock();
	count = cds_ft_count_entries(detached);
	cds_ft_u64_to_key(detached, 5, k, klen);
	s = cds_ft_lookup_key(detached, k, klen, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_detach_at_root: lookup 5 in detached: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 5) {
		fprintf(stderr, "fixed_detach_at_root: key 5 wrong value\n");
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_u64_to_key(detached, 15, k, klen);
	s = cds_ft_lookup_key(detached, k, klen, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_detach_at_root: lookup 15 in detached: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (count != 3) {
		fprintf(stderr, "fixed_detach_at_root: detached count %lu, expected 3\n",
			count);
		goto fail;
	}

	synchronize_rcu();
	drain_trie(detached);
	rcu_barrier();
	cds_ft_destroy(detached);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Non-root detach on a fixed-length trie returns
 * INVALID_ARGUMENT_ERROR.
 */
static int test_fixed_detach_nonroot_error(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate so there is content to detach. */
	{
		struct ft_test_node *n = node_alloc(0xAABBCCDD);

		cds_ft_u64_to_key(ft, 0xAABBCCDD, k, klen);
		rcu_read_lock();
		s = cds_ft_insert(ft, k, klen, &n->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* Attempt non-root detach (key_len=2 < fixed klen=4). */
	rcu_read_lock();
	s = cds_ft_detach(ft, (const uint8_t *)"\xAA\xBB", 2, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "fixed_detach_nonroot: expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Original trie should still have its node. */
	if (cds_ft_empty(ft)) {
		fprintf(stderr, "fixed_detach_nonroot: original should not be empty\n");
		goto fail;
	}

	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/* ================================================================== */
/*                                                                    */
/*            10. CORNER-CASE & COVERAGE-GAP TESTS                    */
/*                                                                    */
/* ================================================================== */

/*
 * cds_ft_max_used_key_len increases on insert. After removing the
 * longest key and calling cds_ft_recompute_stats, the value drops.
 */
static int test_recompute_stats(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n1, *n2, *n3;
	enum cds_ft_status s;
	size_t used;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	n1 = node_alloc(0);
	n2 = node_alloc(0);
	n3 = node_alloc(0);

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"a", 1, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"bc", 2, &n2->node);
	if (s < 0) goto fail;

	used = cds_ft_max_used_key_len(ft);
	if (used != 2) {
		fprintf(stderr, "recompute_stats: max_used after 2 inserts: %zu, expected 2\n", used);
		goto fail;
	}

	s = cds_ft_insert(ft, (const uint8_t *)"def", 3, &n3->node);
	if (s < 0) goto fail;

	used = cds_ft_max_used_key_len(ft);
	if (used != 3) {
		fprintf(stderr, "recompute_stats: max_used after 3 inserts: %zu, expected 3\n", used);
		goto fail;
	}

	/* Remove the longest key "def". */
	cds_ft_iter_set_key(iter, (const uint8_t *)"def", 3);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_remove(ft, iter, &n3->node);
	if (s != CDS_FT_STATUS_OK) goto fail;

	/* max_used_key_len is conservative — should still be 3. */
	used = cds_ft_max_used_key_len(ft);
	if (used != 3) {
		fprintf(stderr, "recompute_stats: max_used after remove: %zu, expected 3\n", used);
		goto fail;
	}

	/* Recompute: should drop to 2. */
	s = cds_ft_recompute_stats(ft);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "recompute_stats: %s\n", cds_ft_status_to_string(s));
		goto fail;
	}
	rcu_read_unlock();

	used = cds_ft_max_used_key_len(ft);
	if (used != 2) {
		fprintf(stderr, "recompute_stats: after recompute: %zu, expected 2\n", used);
		node_free_rcu(n3);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	node_free_rcu(n3);
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_iter_get_prefix returns the prefix portion of the iterator
 * key after a prefix-scoped lookup.
 */
static int test_iter_get_prefix(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n1, *n2;
	enum cds_ft_status s;
	uint8_t prefix_buf[16];
	size_t prefix_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	n1 = node_alloc(0);
	n2 = node_alloc(0);

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"abX", 3, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"abY", 3, &n2->node);
	if (s < 0) goto fail;

	/* Set up prefix-scoped iteration under "ab". */
	cds_ft_iter_set_key(iter, (const uint8_t *)"ab", 2);
	cds_ft_iter_set_prefix_len(iter, 2);
	s = cds_ft_lookup_ge(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_get_prefix: lookup_ge: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Retrieve the prefix. */
	s = cds_ft_iter_get_prefix(iter, prefix_buf, sizeof(prefix_buf), &prefix_len);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_get_prefix: get_prefix: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (prefix_len != 2 || memcmp(prefix_buf, "ab", 2) != 0) {
		fprintf(stderr, "iter_get_prefix: prefix_len %zu, expected 2\n", prefix_len);
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Iterator-based cds_ft_lookup_partial: insert "hello", look up
 * "helloworld" → should find "hello" with match_len 5.
 */
static int test_lookup_partial_iter(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n;
	enum cds_ft_status s;
	uint8_t rk[32];
	size_t rk_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	n = node_alloc(0);
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"hello", 5, &n->node);
	if (s < 0) goto fail;

	cds_ft_iter_set_key(iter, (const uint8_t *)"helloworld", 10);
	s = cds_ft_lookup_partial(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_partial_iter: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (!cds_ft_iter_node(iter)) {
		fprintf(stderr, "lookup_partial_iter: node is NULL\n");
		goto fail;
	}

	s = cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	if (s != CDS_FT_STATUS_OK) goto fail;
	if (rk_len != 5 || memcmp(rk, "hello", 5) != 0) {
		fprintf(stderr, "lookup_partial_iter: match_len %zu, expected 5\n", rk_len);
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Iterator-based cds_ft_lookup_longest_match: insert "ab", look up
 * "abcdef" → longest match at 2 bytes with external node.
 */
static int test_lookup_longest_match_iter(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n;
	enum cds_ft_status s;
	uint8_t rk[32];
	size_t rk_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	n = node_alloc(0);
	n->value = 42;
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"ab", 2, &n->node);
	if (s < 0) goto fail;

	cds_ft_iter_set_key(iter, (const uint8_t *)"abcdef", 6);
	s = cds_ft_lookup_longest_match(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_longest_match_iter: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (!cds_ft_iter_node(iter)) {
		fprintf(stderr, "lookup_longest_match_iter: node is NULL\n");
		goto fail;
	}
	if (to_test_node(cds_ft_iter_node(iter))->value != 42) {
		fprintf(stderr, "lookup_longest_match_iter: wrong node\n");
		goto fail;
	}

	s = cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	if (s != CDS_FT_STATUS_OK) goto fail;
	if (rk_len != 2) {
		fprintf(stderr, "lookup_longest_match_iter: match_len %zu, expected 2\n", rk_len);
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_for_each_entry_reverse_rcu iterates entries in descending key order.
 */
static int test_for_each_entry_reverse(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *entry;
	enum cds_ft_status s;
	uint64_t vals[] = { 100, 200, 300 };
	uint64_t prev_key;
	unsigned int i, count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 3; i++) {
		struct ft_test_node *n = node_alloc(vals[i]);

		rcu_read_lock();
		s = insert_u64(ft, vals[i], n);
		rcu_read_unlock();
		if (s < 0) {
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	count = 0;
	prev_key = UINT64_MAX;
	rcu_read_lock();
	cds_ft_for_each_entry_reverse_rcu(ft, iter, entry, node) {
		uint64_t k = entry->key;

		if (k >= prev_key) {
			fprintf(stderr, "for_each_entry_reverse: not descending at %lu\n",
				(unsigned long)k);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev_key = k;
		count++;
	}
	rcu_read_unlock();

	if (count != 3) {
		fprintf(stderr, "for_each_entry_reverse: count %u, expected 3\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_for_each_duplicate_entry_rcu: walk a duplicate chain with
 * typed entries.
 */
static int test_for_each_duplicate_entry(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n1, *n2, *n3;
	struct cds_ft_node *head;
	struct ft_test_node *entry;
	enum cds_ft_status s;
	uint64_t sum;
	int count;

	n1 = node_alloc(50); n1->value = 10;
	n2 = node_alloc(50); n2->value = 20;
	n3 = node_alloc(50); n3->value = 30;

	rcu_read_lock();
	s = insert_u64(ft, 50, n1);
	if (s < 0) goto fail;
	s = insert_u64(ft, 50, n2);
	if (s < 0) goto fail;
	s = insert_u64(ft, 50, n3);
	if (s < 0) goto fail;

	s = lookup_u64(ft, 50, &head);
	if (s != CDS_FT_STATUS_OK || !head) goto fail;

	count = 0;
	sum = 0;
	cds_ft_for_each_duplicate_entry_rcu(entry, head, node) {
		sum += entry->value;
		count++;
	}
	rcu_read_unlock();

	if (count != 3) {
		fprintf(stderr, "for_each_duplicate_entry: count %d, expected 3\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	if (sum != 60) {
		fprintf(stderr, "for_each_duplicate_entry: sum %lu, expected 60\n",
			(unsigned long)sum);
		drain_and_destroy(ft, group);
		return -1;
	}

	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "for_each_duplicate_entry: insert/lookup: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_for_each_duplicate_entry_safe_rcu: walk and remove from a
 * duplicate chain using the safe typed-entry macro.
 */
static int test_for_each_duplicate_entry_safe(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1, *n2;
	struct cds_ft_node *head, *tmp;
	struct ft_test_node *entry;
	enum cds_ft_status s;
	int count;

	n1 = node_alloc(60); n1->value = 1;
	n2 = node_alloc(60); n2->value = 2;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n1);
		node_free(n2);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	s = insert_u64(ft, 60, n1);
	if (s < 0) goto fail;
	s = insert_u64(ft, 60, n2);
	if (s < 0) goto fail;

	/* remove_all to get the chain, then walk it with the safe macro. */
	{
		uint8_t k[4];

		cds_ft_u64_to_key(ft, 60, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ft, iter);
	}
	s = cds_ft_remove_all(ft, iter, &head);
	if (s != CDS_FT_STATUS_OK || !head) goto fail;
	rcu_read_unlock();

	/* Walk the removed chain with the typed safe macro, freeing nodes. */
	count = 0;
	rcu_read_lock();
	cds_ft_for_each_duplicate_entry_safe_rcu(entry, head, tmp, node) {
		count++;
		node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();

	if (count != 2) {
		fprintf(stderr, "for_each_duplicate_entry_safe: count %d, expected 2\n", count);
		cds_ft_iter_destroy(iter);
		rcu_barrier();
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	fprintf(stderr, "for_each_duplicate_entry_safe: %s\n",
		cds_ft_status_to_string(s));
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_group_destroy returns BUSY_ERROR when tries still exist.
 */
static int test_group_destroy_busy_error(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	enum cds_ft_status s;

	if (cds_ft_group_create(NULL, &group) < 0)
		return -1;
	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Try to destroy the group while a trie still exists. */
	s = cds_ft_group_destroy(group);
	if (s != CDS_FT_STATUS_BUSY_ERROR) {
		fprintf(stderr, "group_destroy_busy: expected BUSY_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Now destroy properly. */
	cds_ft_destroy(ft);
	s = cds_ft_group_destroy(group);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "group_destroy_busy: final destroy: %s\n",
			cds_ft_status_to_string(s));
		return -1;
	}
	return 0;
}

/*
 * graft_swap that would exceed the group's max key length returns
 * OVERFLOW_ERROR (mirrors test_graft_overflow_error but for graft_swap).
 */
static int test_graft_swap_overflow_error(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	enum cds_ft_status s;

	/* max_key_len = 4. */
	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert a 3-byte key into swap. */
	{
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(swap, (const uint8_t *)"abc", 3, &n->node);
		if (s < 0) goto fail;
	}

	/*
	 * Swap at a 2-byte prefix: combined length = 2 + 3 = 5,
	 * exceeding max_key_len = 4.
	 */
	rcu_read_lock();
	s = cds_ft_graft_swap(live, (const uint8_t *)"XY", 2, swap);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_OVERFLOW_ERROR) {
		fprintf(stderr, "graft_swap_overflow: expected OVERFLOW_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Swap trie should still own its content. */
	if (cds_ft_empty(swap)) {
		fprintf(stderr, "graft_swap_overflow: swap empty after failed graft_swap\n");
		goto fail;
	}

	drain_trie(swap);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * cds_ft_replace with a node not present at the iterator position
 * returns NOT_FOUND.
 */
static int test_replace_not_found(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n_in, *n_fake, *n_new;
	enum cds_ft_status s;
	uint8_t k[4];

	n_in = node_alloc(77);
	n_fake = node_alloc(77);
	n_new = node_alloc(77);

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n_in);
		node_free(n_fake);
		node_free(n_new);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 77, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n_in->node);
	if (s < 0) goto fail;

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) goto fail;

	/*
	 * Try to replace n_fake (never inserted) — should fail.
	 */
	s = cds_ft_replace(ft, iter, &n_fake->node, &n_new->node);
	rcu_read_unlock();

	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "replace_not_found: replace of absent node succeeded\n");
		node_free(n_fake);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	node_free(n_fake);
	node_free(n_new);
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	node_free(n_fake);
	node_free(n_new);
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_remove_all at a key with no nodes returns NOT_FOUND.
 */
static int test_remove_all_not_found(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct cds_ft_node *head = (struct cds_ft_node *)1; /* sentinel */
	enum cds_ft_status s;
	uint8_t k[4];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 999, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	cds_ft_lookup(ft, iter);
	s = cds_ft_remove_all(ft, iter, &head);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "remove_all_not_found: expected NOT_FOUND, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * cds_ft_insert_replace on a key with no prior node returns
 * CDS_FT_STATUS_OK with *result_node == NULL.
 */
static int test_insert_replace_no_existing(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n = node_alloc(55);
	struct cds_ft_node *old_head = (struct cds_ft_node *)1; /* sentinel */
	enum cds_ft_status s;
	uint8_t k[4];

	n->value = 555;
	cds_ft_u64_to_key(ft, 55, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT,
				  &n->node, &old_head);
	rcu_read_unlock();

	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "insert_replace_no_existing: expected OK, got %s\n",
			cds_ft_status_to_string(s));
		drain_and_destroy(ft, group);
		return -1;
	}
	if (old_head != NULL) {
		fprintf(stderr, "insert_replace_no_existing: old_head should be NULL\n");
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Verify the node is findable. */
	rcu_read_lock();
	{
		struct cds_ft_node *found;

		s = lookup_u64(ft, 55, &found);
		if (s != CDS_FT_STATUS_OK || found != &n->node) {
			fprintf(stderr, "insert_replace_no_existing: lookup failed\n");
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);
}

/*
 * Inserting a key longer than the group's max_key_len returns an error.
 */
static int test_insert_exceeds_max_key_len(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct ft_test_node *n;
	enum cds_ft_status s;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	n = node_alloc(0);

	/* 5-byte key exceeds max_key_len of 4. */
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"ABCDE", 5, &n->node);
	rcu_read_unlock();

	if (s >= 0) {
		fprintf(stderr, "insert_exceeds_max: should have failed, got %s\n",
			cds_ft_status_to_string(s));
		drain_and_destroy(ft, group);
		return -1;
	}

	/* n was never inserted. */
	node_free(n);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Graft and detach with CDS_FT_LEN_DEFAULT on a fixed-length trie.
 * CDS_FT_LEN_DEFAULT resolves to the fixed key length (non-zero),
 * which is a non-root operation, so both must return
 * INVALID_ARGUMENT_ERROR on a fixed-length group.
 */
static int test_graft_detach_len_default(void)
{
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *other, *detached = NULL;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_attr_set_key_len(attr, klen) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		return -1;
	}
	cds_ft_attr_destroy(attr);

	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, &other) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate ft so there is content at the key. */
	{
		struct ft_test_node *n = node_alloc(12345678);

		cds_ft_u64_to_key(ft, 12345678, k, klen);
		rcu_read_lock();
		s = cds_ft_insert(ft, k, klen, &n->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* graft with CDS_FT_LEN_DEFAULT → non-root → INVALID_ARGUMENT_ERROR. */
	rcu_read_lock();
	s = cds_ft_graft(ft, k, CDS_FT_LEN_DEFAULT, other);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_detach_len_default: graft expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* detach with CDS_FT_LEN_DEFAULT → same. */
	rcu_read_lock();
	s = cds_ft_detach(ft, k, CDS_FT_LEN_DEFAULT, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_detach_len_default: detach expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* graft_swap with CDS_FT_LEN_DEFAULT → same. */
	rcu_read_lock();
	s = cds_ft_graft_swap(ft, k, CDS_FT_LEN_DEFAULT, other);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "graft_detach_len_default: graft_swap expected INVALID_ARGUMENT_ERROR, got %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	drain_trie(ft);
	rcu_barrier();
	if (detached) cds_ft_destroy(detached);
	cds_ft_destroy(other);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(ft);
	drain_trie(other);
	rcu_barrier();
	if (detached) cds_ft_destroy(detached);
	cds_ft_destroy(other);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft an empty source trie: the operation should succeed as a no-op
 * and the destination should remain unchanged.
 */
static int test_graft_empty_source(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *empty_src;
	enum cds_ft_status s;
	unsigned long count;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &empty_src) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate live with one node. */
	{
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		s = cds_ft_insert(live, (const uint8_t *)"xyz", 3, &n->node);
		rcu_read_unlock();
		if (s < 0) goto fail;
	}

	/* Graft empty_src at "aa" (destination is empty there). */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"aa", 2, empty_src);
	rcu_read_unlock();

	/*
	 * Accept either OK (no-op graft) or NOT_FOUND (nothing to graft).
	 * The key invariant is: live trie is not corrupted.
	 */
	if (s < 0 && s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "graft_empty_source: unexpected error: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Verify live still has exactly one node. */
	rcu_read_lock();
	count = cds_ft_count_entries(live);
	rcu_read_unlock();
	if (count != 1) {
		fprintf(stderr, "graft_empty_source: live count %lu, expected 1\n", count);
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(empty_src);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(empty_src);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Graft at a prefix, drain the grafted content, then re-graft new
 * content at the same prefix (verifies the graft point is reusable).
 */
static int test_graft_reuse_after_drain(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	struct cds_ft *detached = NULL;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* First graft: populate staging, graft at "ab". */
	{
		struct ft_test_node *n = node_alloc(0);

		n->value = 1;
		s = cds_ft_insert(staging, (const uint8_t *)"X", 1, &n->node);
		if (s < 0) goto fail;
	}
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"ab", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_reuse: first graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Detach the "ab" subtree to free the graft point. */
	rcu_read_lock();
	s = cds_ft_detach(live, (const uint8_t *)"ab", 2, &detached);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "graft_reuse: detach: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	synchronize_rcu();
	drain_trie(detached);
	rcu_barrier();
	cds_ft_destroy(detached);
	detached = NULL;

	/* Second graft: new content at the same prefix "ab". */
	{
		struct ft_test_node *n = node_alloc(0);

		n->value = 2;
		s = cds_ft_insert(staging, (const uint8_t *)"Y", 1, &n->node);
		if (s < 0) goto fail;
	}
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"ab", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_reuse: second graft: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Verify live has "abY" with value 2. */
	rcu_read_lock();
	s = cds_ft_lookup_key(live, (const uint8_t *)"abY", 3, &found);
	count = cds_ft_count_entries(live);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_reuse: lookup 'abY': %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	if (to_test_node(found)->value != 2) {
		fprintf(stderr, "graft_reuse: 'abY' wrong value\n");
		goto fail;
	}
	if (count != 1) {
		fprintf(stderr, "graft_reuse: count %lu, expected 1\n", count);
		goto fail;
	}

	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	if (detached) {
		drain_trie(detached);
		cds_ft_destroy(detached);
	}
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Two independent tries in the same group: insert/lookup/remove
 * on each trie operates independently.
 */
static int test_multiple_tries_same_group(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft1, *ft2;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long c1, c2;

	ft1 = create_varlen_ft(&group);
	if (cds_ft_create(group, &ft2) < 0) {
		cds_ft_destroy(ft1);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert different keys into each trie. */
	{
		struct ft_test_node *n1 = node_alloc(0);
		struct ft_test_node *n2 = node_alloc(0);

		n1->value = 100;
		n2->value = 200;

		rcu_read_lock();
		s = cds_ft_insert(ft1, (const uint8_t *)"AAA", 3, &n1->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		s = cds_ft_insert(ft2, (const uint8_t *)"BBB", 3, &n2->node);
		if (s < 0) { rcu_read_unlock(); goto fail; }
		rcu_read_unlock();
	}

	/* Verify isolation: "AAA" not in ft2, "BBB" not in ft1. */
	rcu_read_lock();
	s = cds_ft_lookup_key(ft1, (const uint8_t *)"BBB", 3, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "multiple_tries: 'BBB' found in ft1\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_lookup_key(ft2, (const uint8_t *)"AAA", 3, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "multiple_tries: 'AAA' found in ft2\n");
		rcu_read_unlock();
		goto fail;
	}

	/* Verify each has exactly one node. */
	c1 = cds_ft_count_entries(ft1);
	c2 = cds_ft_count_entries(ft2);
	rcu_read_unlock();
	if (c1 != 1 || c2 != 1) {
		fprintf(stderr, "multiple_tries: counts %lu, %lu, expected 1, 1\n", c1, c2);
		goto fail;
	}

	drain_trie(ft1);
	drain_trie(ft2);
	rcu_barrier();
	cds_ft_destroy(ft1);
	cds_ft_destroy(ft2);
	cds_ft_group_destroy(group);
	return 0;

fail:
	drain_trie(ft1);
	drain_trie(ft2);
	rcu_barrier();
	cds_ft_destroy(ft1);
	cds_ft_destroy(ft2);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Fixed key_len=0 trie: insert, lookup, iterate, and remove a NIL key.
 */
static int test_nil_key_fixed_zero_len_trie(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(0, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;

	n = node_alloc(0);
	n->value = 77;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert NIL key. */
	rcu_read_lock();
	s = cds_ft_insert(ft, NULL, 0, &n->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nil_fixed: insert: %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* Lookup NIL key. */
	s = cds_ft_lookup_key(ft, NULL, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "nil_fixed: lookup: %s\n", cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	if (to_test_node(found)->value != 77) {
		fprintf(stderr, "nil_fixed: wrong value\n");
		rcu_read_unlock();
		goto fail;
	}

	/* Iterate: should find exactly one node. */
	count = cds_ft_count_entries(ft);
	if (count != 1) {
		fprintf(stderr, "nil_fixed: count %lu, expected 1\n", count);
		rcu_read_unlock();
		goto fail;
	}

	/* Remove the NIL key. */
	cds_ft_iter_set_key(iter, NULL, 0);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nil_fixed: lookup for remove: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_remove(ft, iter, &n->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nil_fixed: remove: %s\n", cds_ft_status_to_string(s));
		goto fail;
	}

	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "nil_fixed: trie not empty after remove\n");
		goto fail;
	}

	node_free_rcu(n);
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * NIL key with insert_unique and insert_replace on a variable-length trie.
 */
static int test_nil_key_unique_and_replace(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct ft_test_node *n1, *n2, *n3;
	struct cds_ft_node *result;
	struct cds_ft_node *old_head;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);
	n1 = node_alloc(0); n1->value = 1;
	n2 = node_alloc(0); n2->value = 2;
	n3 = node_alloc(0); n3->value = 3;

	/* insert_unique: first should succeed. */
	rcu_read_lock();
	s = cds_ft_insert_unique(ft, NULL, 0, &n1->node, &result);
	if (s != CDS_FT_STATUS_OK || result != &n1->node) {
		fprintf(stderr, "nil_unique: first insert: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* insert_unique: second should return DUPLICATE_FOUND. */
	s = cds_ft_insert_unique(ft, NULL, 0, &n2->node, &result);
	if (s != CDS_FT_STATUS_DUPLICATE_FOUND || result != &n1->node) {
		fprintf(stderr, "nil_unique: second insert: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}

	/* insert_replace: should replace n1 with n3. */
	s = cds_ft_insert_replace(ft, NULL, 0, &n3->node, &old_head);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_DUPLICATE_FOUND || old_head != &n1->node) {
		fprintf(stderr, "nil_replace: %s, old_head %s n1\n",
			cds_ft_status_to_string(s),
			old_head == &n1->node ? "==" : "!=");
		goto fail;
	}

	/* Verify trie holds n3. */
	rcu_read_lock();
	{
		struct cds_ft_node *found;

		s = cds_ft_lookup_key(ft, NULL, 0, &found);
		if (s != CDS_FT_STATUS_OK || found != &n3->node) {
			fprintf(stderr, "nil_replace: lookup after replace failed\n");
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	node_free_rcu(n1);
	node_free(n2);		/* was never inserted */
	return drain_and_destroy(ft, group);

fail:
	node_free(n2);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_for_each_entry_rcu on a trie with duplicate chains: the macro
 * should visit each key position once (the chain head), not each
 * duplicate individually.
 */
static int test_for_each_entry_with_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *entry;
	enum cds_ft_status s;
	unsigned int count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert 3 nodes at key 10, 2 nodes at key 20. */
	{
		unsigned int i;
		uint64_t keys[] = { 10, 10, 10, 20, 20 };

		for (i = 0; i < 5; i++) {
			struct ft_test_node *n = node_alloc(keys[i]);

			rcu_read_lock();
			s = insert_u64(ft, keys[i], n);
			rcu_read_unlock();
			if (s < 0) {
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
		}
	}

	/*
	 * for_each_entry_rcu visits each key position once,
	 * so we should see exactly 2 iterations (key 10, key 20).
	 */
	count = 0;
	rcu_read_lock();
	cds_ft_for_each_entry_rcu(ft, iter, entry, node) {
		count++;
	}
	rcu_read_unlock();

	if (count != 2) {
		fprintf(stderr, "for_each_entry_dup: count %u, expected 2\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_iter_set_key with a non-prefix key invalidates the
 * backtracking path. A subsequent lookup must still succeed.
 */
static int test_iter_set_key_path_invalidation(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_test_node *n1, *n2;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	uint8_t rk[32];
	size_t rk_len;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	n1 = node_alloc(0); n1->value = 1;
	n2 = node_alloc(0); n2->value = 2;

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n1->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(ft, (const uint8_t *)"xyz", 3, &n2->node);
	if (s < 0) goto fail;

	/* First lookup: position at "abc". */
	cds_ft_iter_set_key(iter, (const uint8_t *)"abc", 3);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "set_key_invalidation: lookup 'abc': %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->value != 1) {
		fprintf(stderr, "set_key_invalidation: wrong node for 'abc'\n");
		goto fail;
	}

	/* Set key to "xyz" — not a prefix of "abc", invalidates path. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"xyz", 3);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "set_key_invalidation: lookup 'xyz': %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->value != 2) {
		fprintf(stderr, "set_key_invalidation: wrong node for 'xyz'\n");
		goto fail;
	}

	/* Verify the key in the iterator is correct. */
	s = cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
	if (s != CDS_FT_STATUS_OK || rk_len != 3 ||
	    memcmp(rk, "xyz", 3) != 0) {
		fprintf(stderr, "set_key_invalidation: iter key mismatch\n");
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Smoke test for cds_ft_show and cds_ft_show_stats: call them on a
 * non-empty trie and verify no crash. Output goes to /dev/null.
 */
static int test_show_smoke(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n;
	enum cds_ft_status s;
	FILE *devnull;

	n = node_alloc(42);

	rcu_read_lock();
	s = insert_u64(ft, 42, n);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	devnull = fopen("/dev/null", "w");
	if (!devnull) {
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_show(ft, devnull);
	cds_ft_show_stats(ft, devnull);
	fclose(devnull);

	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*     11. ADVERSARIAL KEY & PER-NODE DISTRIBUTION TESTS              */
/*                                                                    */
/* ================================================================== */

/*
 * Adversarial key patterns and per-node child distributions.
 *
 * These tests are designed to stress internal node configurations
 * (linear, 1D pool, 2D pool, pigeon) by constructing key populations
 * that force transitions through every node type, trigger pool
 * fallback paths, exercise hysteresis boundaries, and verify
 * correctness under pathological key distributions.
 *
 * The node configuration thresholds on 64-bit are:
 *   Type 0 LINEAR:  1 child       (16 B)
 *   Type 1 LINEAR:  1-3 children  (32 B)
 *   Type 2 LINEAR:  3-7 children  (64 B)
 *   Type 3 LINEAR:  5-14 children (128 B)
 *   Type 4 LINEAR:  10-28 children(256 B)
 *   Type 5 POOL 1D: 22-54 children(512 B)  pool uses 1 bit
 *   Type 6 POOL 2D: 51-104 children(1024 B) pool uses 2 bits
 *   Type 7 PIGEON:  95-256 children(2048 B)
 *
 * On 32-bit the thresholds differ; the tests use counts that cover
 * both architectures by targeting the wider 64-bit thresholds.
 */

/*
 * Helper: insert a raw byte key of given length into a variable-length trie.
 */
static enum cds_ft_status
insert_raw(struct cds_ft *ft, const uint8_t *key, size_t key_len,
	   struct ft_test_node *n)
{
	return cds_ft_insert(ft, key, key_len, &n->node);
}

/*
 * Helper: remove a node by raw key from a variable-length trie.
 */
static enum cds_ft_status
remove_raw(struct cds_ft *ft, struct cds_ft_iter *iter,
	   const uint8_t *key, size_t key_len, struct ft_test_node *n)
{
	enum cds_ft_status s;

	cds_ft_iter_set_key(iter, key, key_len);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK)
		return s;
	return cds_ft_remove(ft, iter, &n->node);
}

/*
 * Helper: verify forward iteration yields exactly @expected_count
 * nodes in strictly ascending key order for a 2-byte fixed trie.
 */
static int
verify_2byte_order(struct cds_ft *ft, struct cds_ft_iter *iter,
		   unsigned int expected_count)
{
	unsigned int count = 0;
	uint64_t prev = 0;
	int first = 1;

	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			fprintf(stderr, "order violation: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			return -1;
		}
		prev = v;
		first = 0;
		count++;
	}
	rcu_read_unlock();

	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "iteration error: %s\n",
			cds_ft_status_to_string(cds_ft_iter_status(iter)));
		return -1;
	}
	if (count != expected_count) {
		fprintf(stderr, "count mismatch: %u, expected %u\n",
			count, expected_count);
		return -1;
	}
	return 0;
}

/*
 * Helper: verify forward iteration for variable-length trie yields
 * @expected_count nodes in non-descending lexicographic order.
 */
static int
verify_varlen_order(struct cds_ft *ft, struct cds_ft_iter *iter,
		    unsigned int expected_count)
{
	size_t max_klen = cds_ft_max_key_len(ft);
	uint8_t *prev_key, *rk;
	unsigned int count = 0;
	size_t prev_len = 0;
	int first = 1;
	int ret = -1;

	prev_key = (uint8_t *) malloc(max_klen);
	rk = (uint8_t *) malloc(max_klen);
	if (!prev_key || !rk) {
		fprintf(stderr, "verify_varlen_order: malloc failed\n");
		free(prev_key);
		free(rk);
		return -1;
	}

	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		size_t rk_len;

		cds_ft_iter_get_key(iter, rk, max_klen, &rk_len);
		if (!first) {
			size_t cmp_len = prev_len < rk_len ? prev_len : rk_len;
			int cmp = memcmp(prev_key, rk, cmp_len);

			if (cmp > 0 || (cmp == 0 && prev_len >= rk_len)) {
				fprintf(stderr, "varlen order violation at position %u\n", count);
				rcu_read_unlock();
				goto out;
			}
		}
		memcpy(prev_key, rk, rk_len);
		prev_len = rk_len;
		first = 0;
		count++;
	}
	rcu_read_unlock();

	if (cds_ft_iter_status(iter) < 0)
		goto out;
	if (count != expected_count) {
		fprintf(stderr, "varlen count: %u, expected %u\n",
			count, expected_count);
		goto out;
	}
	ret = 0;
out:
	free(prev_key);
	free(rk);
	return ret;
}

/*
 * Insert keys sharing a common first byte, all 256 second-byte values.
 * Forces a single internal node through every configuration: linear
 * (types 0..4), 1D pool (type 5), 2D pool (type 6), and pigeon
 * (type 7). Verifies count and sorted iteration.
 */
static int test_adversarial_ramp_all_configs(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		struct ft_test_node *n = node_alloc((0xAA << 8) | i);

		n->value = i;
		rcu_read_lock();
		if (insert_u64(ft, (0xAA << 8) | i, n) < 0) {
			fprintf(stderr, "ramp: insert %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) {
		fprintf(stderr, "ramp: count %lu, expected 256\n", ft_count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, 256) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Keys whose discriminating byte values all have bit 7 set
 * (values 0x80..0xB5). The 1D pool partitions children based on a
 * selected bit position; if all children share the same bit value,
 * one sub-pool is empty and the other overflows, triggering fallback.
 */
static int test_adversarial_single_bit_cluster(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned int nr_keys = 54;	/* 1D pool max_child on 64-bit */
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < nr_keys; i++) {
		struct ft_test_node *n = node_alloc((0x42 << 8) | (0x80 + i));

		rcu_read_lock();
		if (insert_u64(ft, (0x42 << 8) | (0x80 + i), n) < 0) {
			fprintf(stderr, "single_bit_cluster: insert %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != nr_keys) {
		fprintf(stderr, "single_bit_cluster: count %lu, expected %u\n",
			ft_count, nr_keys);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, nr_keys) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Keys whose discriminating byte values all have bit 0 set
 * (i*2+1 for i in [0..103]), adversarial for any pool split on bit 0.
 * 104 children reach the 2D pool max_child on 64-bit.
 */
static int test_adversarial_same_nibble_cluster(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned int nr_keys = 104;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < nr_keys; i++) {
		uint64_t v = (0x77ULL << 8) | (uint64_t)(i * 2 + 1);
		struct ft_test_node *n = node_alloc(v);

		rcu_read_lock();
		if (insert_u64(ft, v, n) < 0) {
			fprintf(stderr, "same_nibble: insert %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != nr_keys) {
		fprintf(stderr, "same_nibble: count %lu, expected %u\n",
			ft_count, nr_keys);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, nr_keys) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Keys where bits 0 and 1 are always clear (values i*4 for i in
 * [0..63]). Any 2D pool split using bits 0 and 1 places all children
 * in one quadrant.
 */
static int test_adversarial_two_bit_cluster(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned int nr_keys = 64;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < nr_keys; i++) {
		uint64_t v = (0x33ULL << 8) | (uint64_t)(i * 4);
		struct ft_test_node *n = node_alloc(v);

		rcu_read_lock();
		if (insert_u64(ft, v, n) < 0) {
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != nr_keys) {
		fprintf(stderr, "two_bit_cluster: count %lu, expected %u\n",
			ft_count, nr_keys);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, nr_keys) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Repeatedly insert and remove keys near the linear-to-pool
 * transition. Insert 30 children (above type 4 max on 64-bit),
 * remove 10 (below type 5 min), re-insert 10. Repeat 3 cycles.
 * Verifies that hysteresis-driven recompaction preserves order.
 */
static int test_adversarial_transition_oscillation(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *nodes[30];
	unsigned int i, cycle;
	unsigned long ft_count;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 30; i++) {
		nodes[i] = node_alloc((0x50 << 8) | i);
		rcu_read_lock();
		if (insert_u64(ft, (0x50 << 8) | i, nodes[i]) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}

	if (verify_2byte_order(ft, iter, 30) < 0)
		goto out;

	for (cycle = 0; cycle < 3; cycle++) {
		for (i = 20; i < 30; i++) {
			uint8_t k[8];

			cds_ft_u64_to_key(ft, (0x50 << 8) | i, k, CDS_FT_LEN_DEFAULT);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
			cds_ft_lookup(ft, iter);
			if (cds_ft_remove(ft, iter, &nodes[i]->node) < 0) {
				rcu_read_unlock();
				goto out;
			}
			rcu_read_unlock();
			node_free_rcu(nodes[i]);
		}
		rcu_barrier();

		rcu_read_lock();
		ft_count = cds_ft_count_entries(ft);
		rcu_read_unlock();
		if (ft_count != 20) {
			fprintf(stderr, "oscillation: count %lu after removal cycle %u\n",
				ft_count, cycle);
			goto out;
		}

		if (verify_2byte_order(ft, iter, 20) < 0)
			goto out;

		for (i = 20; i < 30; i++) {
			nodes[i] = node_alloc((0x50 << 8) | i);
			rcu_read_lock();
			if (insert_u64(ft, (0x50 << 8) | i, nodes[i]) < 0) {
				rcu_read_unlock();
				goto out;
			}
			rcu_read_unlock();
		}

		if (verify_2byte_order(ft, iter, 30) < 0)
			goto out;
	}

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Fill a node to 256 children (pigeon), remove every other child
 * (128 removals), then continue removing until only 1 remains.
 * Verifies iteration order at each phase, exercising the full
 * pigeon-to-linear shrink path.
 */
static int test_adversarial_sparse_removal(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *nodes[256];
	unsigned int i, remaining;
	unsigned long ft_count;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		nodes[i] = node_alloc((0xCC << 8) | i);
		rcu_read_lock();
		if (insert_u64(ft, (0xCC << 8) | i, nodes[i]) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}

	/* Remove even-indexed children. */
	for (i = 0; i < 256; i += 2) {
		uint8_t k[8];

		cds_ft_u64_to_key(ft, (0xCC << 8) | i, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
		cds_ft_lookup(ft, iter);
		if (cds_ft_remove(ft, iter, &nodes[i]->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		node_free_rcu(nodes[i]);
		nodes[i] = NULL;
	}
	rcu_barrier();

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 128) {
		fprintf(stderr, "sparse_removal: count %lu, expected 128\n", ft_count);
		goto out;
	}

	if (verify_2byte_order(ft, iter, 128) < 0)
		goto out;

	/* Continue removing odd children until 1 remains. */
	remaining = 128;
	for (i = 1; remaining > 1; i += 2) {
		uint8_t k[8];

		if (i >= 256)
			break;
		if (!nodes[i])
			continue;

		cds_ft_u64_to_key(ft, (0xCC << 8) | i, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
		cds_ft_lookup(ft, iter);
		if (cds_ft_remove(ft, iter, &nodes[i]->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		node_free_rcu(nodes[i]);
		nodes[i] = NULL;
		remaining--;
	}
	rcu_barrier();

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 1) {
		fprintf(stderr, "sparse_removal: count %lu at end, expected 1\n", ft_count);
		goto out;
	}

	if (verify_2byte_order(ft, iter, 1) < 0)
		goto out;

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Insert variable-length keys composed entirely of 0x00 and 0xFF
 * bytes at lengths 1..16. These extreme boundary values test the
 * trie's handling of minimum/maximum byte values at every depth.
 */
static int test_adversarial_boundary_bytes(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned int i;
	unsigned int nr_keys = 0;
	int ret = -1;
	uint8_t zero_key[16];
	uint8_t ff_key[16];

	memset(zero_key, 0x00, sizeof(zero_key));
	memset(ff_key, 0xFF, sizeof(ff_key));

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 1; i <= 16; i++) {
		struct ft_test_node *nz = node_alloc(i);
		struct ft_test_node *nf = node_alloc(0xFF00 + i);

		rcu_read_lock();
		s = insert_raw(ft, zero_key, i, nz);
		if (s < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = insert_raw(ft, ff_key, i, nf);
		if (s < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		nr_keys += 2;
	}

	rcu_read_lock();
	for (i = 1; i <= 16; i++) {
		s = cds_ft_lookup_key(ft, zero_key, i, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "boundary: lookup zero len %u failed\n", i);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_lookup_key(ft, ff_key, i, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "boundary: lookup ff len %u failed\n", i);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	if (verify_varlen_order(ft, iter, nr_keys) < 0)
		goto out;

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Insert keys that are strict prefixes of each other: {0xAB} (len 1),
 * {0xAB,0xAB} (len 2), ... up to depth 32. Each key shares a prefix
 * with all shorter keys, stress-testing variable-length handling,
 * partial-match logic, and the internal/external node distinction.
 */
static int test_adversarial_prefix_nesting(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	unsigned int i;
	unsigned int depth = 32;
	uint8_t key[32];
	int ret = -1;

	memset(key, 0xAB, sizeof(key));

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 1; i <= depth; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		s = insert_raw(ft, key, i, n);
		rcu_read_unlock();
		if (s < 0) {
			fprintf(stderr, "prefix_nesting: insert len %u: %s\n",
				i, cds_ft_status_to_string(s));
			goto out;
		}
	}

	rcu_read_lock();
	for (i = 1; i <= depth; i++) {
		struct cds_ft_node *found;

		s = cds_ft_lookup_key(ft, key, i, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "prefix_nesting: lookup len %u: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	/* Partial lookup from longest key should match at full depth. */
	rcu_read_lock();
	{
		struct cds_ft_node *found;
		size_t match_len;

		s = cds_ft_lookup_partial_key(ft, key, depth, &match_len, &found);
		if (s != CDS_FT_STATUS_OK || !found || match_len != depth) {
			fprintf(stderr, "prefix_nesting: partial match_len %zu, expected %u\n",
				match_len, depth);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	if (verify_varlen_order(ft, iter, depth) < 0)
		goto out;

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Insert 256 nodes at the same key to build a very long duplicate
 * chain. Verify chain length, then remove all with remove_all.
 */
static int test_adversarial_mass_duplicates(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	struct cds_ft_node *head, *tmp;
	enum cds_ft_status s;
	unsigned int i, count;
	unsigned long ft_count;
	uint8_t k[8];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 0x1234, k, CDS_FT_LEN_DEFAULT);

	for (i = 0; i < 256; i++) {
		struct ft_test_node *n = node_alloc(0x1234);

		n->value = i;
		rcu_read_lock();
		s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
		rcu_read_unlock();
		if (s < 0) {
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) {
		fprintf(stderr, "mass_dup: count %lu\n", ft_count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	rcu_read_lock();
	s = cds_ft_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, &head);
	if (s != CDS_FT_STATUS_OK || !head) {
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	count = 0;
	cds_ft_for_each_duplicate_rcu(head)
		count++;
	rcu_read_unlock();

	if (count != 256) {
		fprintf(stderr, "mass_dup: chain length %u\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	rcu_read_lock();
	cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
	cds_ft_lookup(ft, iter);
	s = cds_ft_remove_all(ft, iter, &head);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
		node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();

	rcu_barrier();

	if (!cds_ft_empty(ft)) {
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Insert keys with complementary bit patterns 0xAA and 0x55 at every
 * first-byte prefix (128 prefixes x 2 children each = 256 nodes).
 * These bit-complement values maximally stress pool bit-selection.
 */
static int test_adversarial_alternating_bits(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int prefix, nr_keys = 0;
	unsigned long ft_count;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (prefix = 0; prefix < 128; prefix++) {
		struct ft_test_node *na = node_alloc((prefix << 8) | 0xAA);
		struct ft_test_node *nb = node_alloc((prefix << 8) | 0x55);

		rcu_read_lock();
		if (insert_u64(ft, (prefix << 8) | 0xAA, na) < 0 ||
		    insert_u64(ft, (prefix << 8) | 0x55, nb) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		nr_keys += 2;
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != nr_keys) {
		fprintf(stderr, "alt_bits: count %lu, expected %u\n",
			ft_count, nr_keys);
		goto out;
	}

	if (verify_2byte_order(ft, iter, nr_keys) < 0)
		goto out;

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Insert 3 keys at the implementation's maximum key length: all-zero,
 * all-0xFF, and alternating 0xAA/0x55 bytes. Verifies lookup and
 * sorted iteration at maximum trie depth.
 *
 * The maximum key length is not hardcoded; it is discovered at
 * runtime by creating a default variable-length trie and querying
 * cds_ft_max_key_len().
 */
static int test_adversarial_max_depth(void)
{
	struct cds_ft_group *probe_group, *group;
	struct cds_ft_attr *attr;
	struct cds_ft *probe_ft, *ft;
	struct cds_ft_iter *iter;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	size_t max_klen;
	unsigned int i;
	int ret = -1;
	uint8_t *key_zero = NULL, *key_ff = NULL, *key_alt = NULL;
	struct ft_test_node *n1, *n2, *n3;

	/*
	 * Discover the implementation's maximum key length from a
	 * default variable-length trie, then tear down the probe.
	 */
	if (cds_ft_group_create(NULL, &probe_group) < 0)
		return -1;
	if (cds_ft_create(probe_group, &probe_ft) < 0) {
		cds_ft_group_destroy(probe_group);
		return -1;
	}
	max_klen = cds_ft_max_key_len(probe_ft);
	cds_ft_destroy(probe_ft);
	cds_ft_group_destroy(probe_group);

	if (max_klen == 0) {
		fprintf(stderr, "max_depth: cds_ft_max_key_len returned 0\n");
		return -1;
	}

	key_zero = (uint8_t *) calloc(max_klen, 1);
	key_ff = (uint8_t *) malloc(max_klen);
	key_alt = (uint8_t *) malloc(max_klen);
	if (!key_zero || !key_ff || !key_alt)
		goto out_free_keys;

	memset(key_ff, 0xFF, max_klen);
	for (i = 0; i < max_klen; i++)
		key_alt[i] = (i & 1) ? 0x55 : 0xAA;

	if (cds_ft_attr_create(&attr) < 0)
		goto out_free_keys;
	if (cds_ft_attr_set_key_len(attr, max_klen) < 0) {
		cds_ft_attr_destroy(attr);
		goto out_free_keys;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_attr_destroy(attr);
		goto out_free_keys;
	}
	cds_ft_attr_destroy(attr);
	if (cds_ft_create(group, &ft) < 0) {
		cds_ft_group_destroy(group);
		goto out_free_keys;
	}
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		goto out_free_keys;
	}

	n1 = node_alloc(0); n1->value = 1;
	n2 = node_alloc(0); n2->value = 2;
	n3 = node_alloc(0); n3->value = 3;

	rcu_read_lock();
	s = cds_ft_insert(ft, key_zero, max_klen, &n1->node);
	if (s < 0) { rcu_read_unlock(); goto out; }
	s = cds_ft_insert(ft, key_ff, max_klen, &n2->node);
	if (s < 0) { rcu_read_unlock(); goto out; }
	s = cds_ft_insert(ft, key_alt, max_klen, &n3->node);
	if (s < 0) { rcu_read_unlock(); goto out; }

	s = cds_ft_lookup_key(ft, key_zero, max_klen, &found);
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->value != 1) {
		rcu_read_unlock(); goto out;
	}
	s = cds_ft_lookup_key(ft, key_ff, max_klen, &found);
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->value != 2) {
		rcu_read_unlock(); goto out;
	}
	s = cds_ft_lookup_key(ft, key_alt, max_klen, &found);
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->value != 3) {
		rcu_read_unlock(); goto out;
	}
	rcu_read_unlock();

	/* Verify sorted order: zero < alt < ff. */
	{
		unsigned int count = 0;
		uint64_t expected_vals[] = { 1, 3, 2 };

		rcu_read_lock();
		cds_ft_for_each_rcu(ft, iter) {
			struct cds_ft_node *node = cds_ft_iter_node(iter);

			if (count >= 3) { rcu_read_unlock(); goto out; }
			if (to_test_node(node)->value != expected_vals[count]) {
				fprintf(stderr, "max_depth: order[%u] value %" PRIu64 "\n",
					count, to_test_node(node)->value);
				rcu_read_unlock();
				goto out;
			}
			count++;
		}
		rcu_read_unlock();
		if (count != 3) goto out;
	}

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
out_free_keys:
	free(key_zero);
	free(key_ff);
	free(key_alt);
	return ret;
}

/*
 * Insert 256 children in strictly descending order (255..0).
 * Exercises insertion ordering assumptions in node compaction.
 */
static int test_adversarial_reverse_insert_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	int i;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 255; i >= 0; i--) {
		struct ft_test_node *n = node_alloc((0xDD << 8) | i);

		rcu_read_lock();
		if (insert_u64(ft, (0xDD << 8) | i, n) < 0) {
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, 256) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Insert 256 children in pseudorandom order (XOR-scrambled with 0xA7).
 * Non-monotonic insertion pattern exercises recompaction under
 * irregular growth.
 */
static int test_adversarial_xor_scramble_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		uint64_t child_byte = i ^ 0xA7;
		struct ft_test_node *n = node_alloc((0xEE << 8) | child_byte);

		rcu_read_lock();
		if (insert_u64(ft, (0xEE << 8) | child_byte, n) < 0) {
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, 256) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Interleaved insert/remove: insert 2, remove 1, repeat 128 times.
 * Net result is 128 surviving nodes. Each cycle causes recompaction
 * as the node grows one child at a time.
 */
static int test_adversarial_interleaved_grow(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int insert_idx = 0, remove_idx = 0;
	unsigned int cycle;
	unsigned long ft_count;
	struct ft_test_node *remove_nodes[128];
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (cycle = 0; cycle < 128; cycle++) {
		struct ft_test_node *n1, *n2;
		uint8_t k[8];

		n1 = node_alloc((0xBB << 8) | insert_idx);
		rcu_read_lock();
		if (insert_u64(ft, (0xBB << 8) | insert_idx, n1) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		remove_nodes[cycle] = n1;
		insert_idx++;

		n2 = node_alloc((0xBB << 8) | insert_idx);
		rcu_read_lock();
		if (insert_u64(ft, (0xBB << 8) | insert_idx, n2) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		insert_idx++;

		cds_ft_u64_to_key(ft, (0xBB << 8) | remove_idx, k,
				  CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
		cds_ft_lookup(ft, iter);
		if (cds_ft_remove(ft, iter, &remove_nodes[cycle]->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		node_free_rcu(remove_nodes[cycle]);
		remove_idx += 2;
	}
	rcu_barrier();

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 128) {
		fprintf(stderr, "interleaved_grow: count %lu, expected 128\n", ft_count);
		goto out;
	}

	if (verify_2byte_order(ft, iter, 128) < 0)
		goto out;

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Two clusters with a wide gap: 0x00..0x0F and 0xF0..0xFF.
 * Relational lookups (le, ge, lt, gt) targeting the gap exercise
 * the iterator backtracking logic over large unpopulated regions.
 */
static int test_adversarial_relational_gap(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(1, &group);
	struct cds_ft_iter *iter;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned int i;
	uint8_t k[1];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i <= 0x0F; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}
	for (i = 0xF0; i <= 0xFF; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	rcu_read_lock();

	/* le(0x80) -> 0x0F */
	cds_ft_u64_to_key(ft, 0x80, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
	s = cds_ft_lookup_le(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0x0F) {
		fprintf(stderr, "relational_gap: le(0x80) key %" PRIu64 "\n",
			found ? to_test_node(found)->key : (uint64_t)-1);
		rcu_read_unlock();
		goto fail;
	}

	/* ge(0x80) -> 0xF0 */
	cds_ft_u64_to_key(ft, 0x80, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
	s = cds_ft_lookup_ge(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0xF0) {
		rcu_read_unlock();
		goto fail;
	}

	/* gt(0x0F) -> 0xF0 */
	cds_ft_u64_to_key(ft, 0x0F, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
	s = cds_ft_lookup_gt(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0xF0) {
		rcu_read_unlock();
		goto fail;
	}

	/* lt(0xF0) -> 0x0F */
	cds_ft_u64_to_key(ft, 0xF0, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_key_len(ft));
	s = cds_ft_lookup_lt(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0x0F) {
		rcu_read_unlock();
		goto fail;
	}

	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Insert single-bit-set keys (1,2,4,...,128) then fill remaining
 * values. The initial sparse bit-position coverage creates the
 * maximally spread population for pool bit-selection, then
 * gradual fill forces transitions under non-sequential growth.
 */
static int test_adversarial_power_of_two_stride(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned int nr_keys = 0;
	unsigned long ft_count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 8; i++) {
		struct ft_test_node *n = node_alloc((0x11 << 8) | (1 << i));

		rcu_read_lock();
		if (insert_u64(ft, (0x11 << 8) | (1 << i), n) < 0) {
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
		nr_keys++;
	}

	for (i = 0; i < 256; i++) {
		if (i != 0 && (i & (i - 1)) == 0)
			continue;
		{
			struct ft_test_node *n = node_alloc((0x11 << 8) | i);

			rcu_read_lock();
			if (insert_u64(ft, (0x11 << 8) | i, n) < 0) {
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
			rcu_read_unlock();
			nr_keys++;
		}
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != nr_keys) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	if (verify_2byte_order(ft, iter, nr_keys) < 0) {
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * 256 variable-length keys sharing a common suffix {0xFF,0xDE,0xAD}
 * but differing at byte 0. Since the trie indexes MSB-first, shared
 * suffixes cause early divergence, creating a wide, shallow trie
 * (256 children at the root node).
 */
static int test_adversarial_shared_suffix(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	unsigned int i;
	unsigned long ft_count;
	int ret = -1;

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		uint8_t key[4] = { (uint8_t)i, 0xFF, 0xDE, 0xAD };
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_raw(ft, key, 4, n) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 256) goto out;

	if (verify_varlen_order(ft, iter, 256) < 0) goto out;

	rcu_read_lock();
	{
		uint8_t key[4] = { 0x42, 0xFF, 0xDE, 0xAD };
		struct cds_ft_node *found;
		enum cds_ft_status s;

		s = cds_ft_lookup_key(ft, key, 4, &found);
		if (s != CDS_FT_STATUS_OK || !found ||
		    to_test_node(found)->key != 0x42) {
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Deep prefix chain (lengths 1..16, all bytes 0xCC) with intermediate
 * nodes (lengths 5..10) removed. Tests longest-match lookup across
 * gaps in the external node population.
 */
static int test_adversarial_longest_match_gaps(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	unsigned int i;
	uint8_t key[16];
	struct ft_test_node *nodes[16];
	int ret = -1;

	memset(key, 0xCC, sizeof(key));

	ft = create_varlen_ft(&group);
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 16; i++) {
		nodes[i] = node_alloc(i + 1);
		nodes[i]->value = i + 1;
		rcu_read_lock();
		s = insert_raw(ft, key, i + 1, nodes[i]);
		rcu_read_unlock();
		if (s < 0) goto out;
	}

	/* Remove keys of length 5..10 (indices 4..9). */
	for (i = 4; i <= 9; i++) {
		rcu_read_lock();
		s = remove_raw(ft, iter, key, i + 1, nodes[i]);
		rcu_read_unlock();
		if (s < 0) goto out;
		node_free_rcu(nodes[i]);
		nodes[i] = NULL;
	}
	rcu_barrier();

	/* longest_match(key, 16) should find at length 16. */
	rcu_read_lock();
	{
		struct cds_ft_node *found;
		size_t match_len;

		s = cds_ft_lookup_longest_match_key(ft, key, 16, &match_len, &found);
		if (s != CDS_FT_STATUS_OK || match_len != 16 || !found) {
			fprintf(stderr, "longest_match_gaps: full: status=%s, len=%zu\n",
				cds_ft_status_to_string(s), match_len);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	/*
	 * longest_match(key, 7): external nodes at 5-7 removed, so
	 * the result should either be INTERNAL_MATCH (structure exists
	 * but no external node) or OK at the nearest ancestor (len <= 4).
	 */
	rcu_read_lock();
	{
		struct cds_ft_node *found;
		size_t match_len;

		s = cds_ft_lookup_longest_match_key(ft, key, 7, &match_len, &found);
		if (s < 0) {
			rcu_read_unlock();
			goto out;
		}
		if (s == CDS_FT_STATUS_OK && match_len > 4) {
			fprintf(stderr, "longest_match_gaps: len 7: found at %zu\n",
				match_len);
			rcu_read_unlock();
			goto out;
		}
		/* INTERNAL_MATCH at 5..7 is also acceptable. */
	}
	rcu_read_unlock();

	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Use insert_replace to overwrite the same key 128 times. Each
 * replacement returns the previous node. Verifies the atomic
 * replacement path under rapid churn.
 */
static int test_adversarial_replace_churn(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_node *old_node;
	enum cds_ft_status s;
	unsigned int i;
	uint8_t k[8];
	unsigned long ft_count;

	cds_ft_u64_to_key(ft, 0xBEEF, k, CDS_FT_LEN_DEFAULT);

	{
		struct ft_test_node *n = node_alloc(0xBEEF);

		n->value = 0;
		rcu_read_lock();
		s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
		rcu_read_unlock();
		if (s < 0) {
			node_free(n);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
	}

	for (i = 1; i <= 128; i++) {
		struct ft_test_node *n = node_alloc(0xBEEF);

		n->value = i;
		rcu_read_lock();
		s = cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT,
					  &n->node, &old_node);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_DUPLICATE_FOUND || !old_node) {
			drain_and_destroy(ft, group);
			return -1;
		}
		node_free_rcu(to_test_node(old_node));
	}
	rcu_barrier();

	rcu_read_lock();
	ft_count = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (ft_count != 1) {
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_lock();
	{
		struct cds_ft_node *found;

		s = cds_ft_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, &found);
		if (s != CDS_FT_STATUS_OK || !found ||
		    to_test_node(found)->value != 128) {
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*            12. UNCACHED ITERATOR PATH MODE TESTS                   */
/*                                                                    */
/* ================================================================== */

/*
 * Default path mode is CACHED. get/set roundtrip works. Invalid mode
 * returns INVALID_ARGUMENT_ERROR.
 */
static int test_iter_path_mode_default(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	if (cds_ft_iter_get_path_mode(iter) != CDS_FT_ITER_PATH_CACHED) {
		fprintf(stderr, "path_mode_default: expected CACHED\n");
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	s = cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "path_mode_default: set UNCACHED: %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_get_path_mode(iter) != CDS_FT_ITER_PATH_UNCACHED) {
		fprintf(stderr, "path_mode_default: get after set UNCACHED\n");
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Invalid mode value. */
	s = cds_ft_iter_set_path_mode(iter, (enum cds_ft_iter_path_mode) 99);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "path_mode_default: expected error for mode 99, got %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Forward iteration in UNCACHED mode, dropping and reacquiring the RCU
 * read-side lock between each step. Verifies that the iterator produces
 * the same ascending sequence as CACHED mode.
 */
static int test_iter_uncached_forward(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t keys[] = { 500, 100, 300, 900, 200, 700, 400 };
	unsigned int i, count = 0;
	uint64_t prev = 0;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);

	/* Iterate with explicit per-step RCU lock/unlock. */
	rcu_read_lock();
	for (cds_ft_lookup_first(ft, iter);
	     cds_ft_iter_node(iter);
	     cds_ft_next(ft, iter)) {
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			fprintf(stderr, "uncached forward: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;

		/* Drop and reacquire the lock between steps. */
		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "uncached forward: error: %s\n",
			cds_ft_status_to_string(cds_ft_iter_status(iter)));
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != sizeof(keys) / sizeof(keys[0])) {
		fprintf(stderr, "uncached forward: %u nodes, expected %zu\n",
			count, sizeof(keys) / sizeof(keys[0]));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Reverse iteration in UNCACHED mode with per-step lock dropping.
 */
static int test_iter_uncached_reverse(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	uint64_t keys[] = { 500, 100, 300, 900, 200, 700, 400 };
	unsigned int i, count = 0;
	uint64_t prev = UINT64_MAX;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);

	rcu_read_lock();
	for (cds_ft_lookup_last(ft, iter);
	     cds_ft_iter_node(iter);
	     cds_ft_prev(ft, iter)) {
		uint8_t rk[2];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v >= prev) {
			fprintf(stderr, "uncached reverse: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;

		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "uncached reverse: error\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != sizeof(keys) / sizeof(keys[0])) {
		fprintf(stderr, "uncached reverse: %u nodes, expected %zu\n",
			count, sizeof(keys) / sizeof(keys[0]));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Uncached lookup followed by remove, with the RCU lock dropped
 * between the two operations. This is the pattern that would be
 * unsafe in CACHED mode without cds_ft_iter_invalidate_path().
 */
static int test_iter_uncached_lookup_remove(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n = node_alloc(42);
	enum cds_ft_status s;
	uint8_t k[4];

	if (cds_ft_iter_create(ft, &iter) < 0) {
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_u64_to_key(ft, 42, k, CDS_FT_LEN_DEFAULT);

	rcu_read_lock();
	s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
	rcu_read_unlock();
	if (s < 0) {
		node_free(n);
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);

	/* Lookup under RCU. */
	rcu_read_lock();
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	s = cds_ft_lookup(ft, iter);
	if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
		fprintf(stderr, "uncached lookup_remove: lookup: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	/* Grace period passes — in CACHED mode, the path would be stale. */
	rcu_quiescent_state();

	/*
	 * Remove under the writer mutex (simulated). The iterator's
	 * key is preserved; the path was auto-invalidated by UNCACHED
	 * mode, so cds_ft_remove will do a fresh top-down traversal.
	 */
	rcu_read_lock();
	s = cds_ft_remove(ft, iter, &n->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "uncached lookup_remove: remove: %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	node_free_rcu(n);
	rcu_barrier();

	/* Trie should be empty. */
	rcu_read_lock();
	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "uncached lookup_remove: trie not empty\n");
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Prefix-scoped iteration in UNCACHED mode.
 */
static int test_iter_uncached_prefix_scoped(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = {
		"apple", "apply", "apt",
		"banana", "band",
		"cat",
	};
	unsigned int i, count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(ft, (const uint8_t *)words[i],
			      strlen(words[i]), &n->node);
		rcu_read_unlock();
	}

	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);
	cds_ft_iter_set_key(iter, (const uint8_t *)"ap", 2);
	cds_ft_iter_set_prefix_len(iter, 2);

	rcu_read_lock();
	for (cds_ft_lookup_first(ft, iter);
	     cds_ft_iter_node(iter);
	     cds_ft_next(ft, iter)) {
		count++;
		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	if (cds_ft_iter_status(iter) < 0) {
		fprintf(stderr, "uncached prefix: error\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	cds_ft_iter_destroy(iter);

	if (count != 3) {
		fprintf(stderr, "uncached prefix 'ap': %u keys, expected 3\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Switching from CACHED to UNCACHED invalidates the path. Switching
 * back to CACHED is safe and re-enables path caching.
 */
static int test_iter_path_mode_switch(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	uint64_t keys[] = { 10, 20, 30, 40, 50 };
	unsigned int count;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct ft_test_node *n = node_alloc(keys[i]);

		rcu_read_lock();
		insert_u64(ft, keys[i], n);
		rcu_read_unlock();
	}

	/* Phase 1: iterate in UNCACHED mode with per-step lock dropping. */
	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);
	count = 0;
	rcu_read_lock();
	for (cds_ft_lookup_first(ft, iter);
	     cds_ft_iter_node(iter);
	     cds_ft_next(ft, iter)) {
		count++;
		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	if (count != 5) {
		fprintf(stderr, "mode_switch uncached phase: %u nodes, expected 5\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Phase 2: switch back to CACHED, iterate normally. */
	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_CACHED);
	if (cds_ft_iter_get_path_mode(iter) != CDS_FT_ITER_PATH_CACHED) {
		fprintf(stderr, "mode_switch: mode not CACHED after switch\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	count = 0;
	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		count++;
	}
	rcu_read_unlock();

	if (count != 5) {
		fprintf(stderr, "mode_switch cached phase: %u nodes, expected 5\n", count);
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * iter_copy preserves the path mode of the source iterator.
 */
static int test_iter_uncached_copy(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter_a, *iter_b;
	unsigned int count;

	if (cds_ft_iter_create(ft, &iter_a) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_create(ft, &iter_b) < 0) {
		cds_ft_iter_destroy(iter_a);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	{
		struct ft_test_node *n;
		unsigned int i;

		for (i = 0; i < 5; i++) {
			n = node_alloc(i * 10);
			rcu_read_lock();
			insert_u64(ft, i * 10, n);
			rcu_read_unlock();
		}
	}

	/* Set iter_a to UNCACHED and position it. */
	cds_ft_iter_set_path_mode(iter_a, CDS_FT_ITER_PATH_UNCACHED);

	rcu_read_lock();
	cds_ft_lookup_first(ft, iter_a);
	rcu_read_unlock();

	/* Copy iter_a → iter_b. iter_b should inherit UNCACHED mode. */
	cds_ft_iter_copy(iter_b, iter_a);

	if (cds_ft_iter_get_path_mode(iter_b) != CDS_FT_ITER_PATH_UNCACHED) {
		fprintf(stderr, "uncached_copy: copy did not inherit path mode\n");
		cds_ft_iter_destroy(iter_a);
		cds_ft_iter_destroy(iter_b);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Continue iterating from iter_b with per-step lock dropping. */
	count = 0;
	rcu_read_lock();
	while (cds_ft_iter_node(iter_b)) {
		count++;
		cds_ft_next(ft, iter_b);
		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	if (count != 5) {
		fprintf(stderr, "uncached_copy: %u nodes from copy, expected 5\n", count);
		cds_ft_iter_destroy(iter_a);
		cds_ft_iter_destroy(iter_b);
		drain_and_destroy(ft, group);
		return -1;
	}

	cds_ft_iter_destroy(iter_a);
	cds_ft_iter_destroy(iter_b);
	return drain_and_destroy(ft, group);
}

/*
 * Stress test: iterate 256 1-byte keys in UNCACHED mode, dropping
 * the lock at every step. Exercises all internal node configurations
 * (linear, pool, pigeon) under the uncached path.
 */
static int test_iter_uncached_all_configs(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(1, &group);
	struct cds_ft_iter *iter;
	unsigned int i, count;
	uint64_t prev = 0;
	int first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 256; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		insert_u64(ft, i, n);
		rcu_read_unlock();
	}

	cds_ft_iter_set_path_mode(iter, CDS_FT_ITER_PATH_UNCACHED);

	count = 0;
	rcu_read_lock();
	for (cds_ft_lookup_first(ft, iter);
	     cds_ft_iter_node(iter);
	     cds_ft_next(ft, iter)) {
		uint8_t rk[1];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			fprintf(stderr, "uncached_all_configs: %" PRIu64 " after %" PRIu64 "\n",
				v, prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		prev = v;
		first = 0;
		count++;

		rcu_read_unlock();
		rcu_quiescent_state();
		rcu_read_lock();
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);

	if (count != 256) {
		fprintf(stderr, "uncached_all_configs: %u nodes, expected 256\n", count);
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*  13. Compressed node corner case tests                             */
/*                                                                    */
/* ================================================================== */

/*
 * Long shared prefix: insert keys that share a long common prefix,
 * creating a compressed path, then verify lookup, count, and drain.
 * Uses 4-byte fixed keys where keys 0..9 share a 3-byte prefix.
 */
static int test_compress_long_prefix(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned long i, count;

	rcu_read_lock();
	for (i = 0; i < 10; i++) {
		if (insert_u64(ft, i, node_alloc(i)) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "compress_long_prefix: insert %lu failed\n", i);
			rcu_read_unlock();
			return drain_and_destroy(ft, group) | -1;
		}
	}
	count = cds_ft_count_keys(ft);
	rcu_read_unlock();
	if (count != 10) {
		fprintf(stderr, "compress_long_prefix: count %lu != 10\n", count);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Verify all keys are findable. */
	rcu_read_lock();
	for (i = 0; i < 10; i++) {
		struct cds_ft_node *found;

		if (lookup_u64(ft, i, &found) != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "compress_long_prefix: lookup %lu failed\n", i);
			rcu_read_unlock();
			return drain_and_destroy(ft, group) | -1;
		}
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);
}

/*
 * Split at early divergence: two keys that share no prefix bytes.
 * The first insert creates a compressed path; the second diverges
 * at position 0, splitting the compressed node with no prefix.
 */
static int test_compress_split_diverge_early(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_node *found;

	rcu_read_lock();
	if (insert_u64(ft, 0, node_alloc(0)) != CDS_FT_STATUS_OK ||
	    insert_u64(ft, 0x01000000ULL, node_alloc(1)) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_split_diverge_early: insert failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	if (lookup_u64(ft, 0, &found) != CDS_FT_STATUS_OK || !found ||
	    lookup_u64(ft, 0x01000000ULL, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_split_diverge_early: lookup failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	if (cds_ft_count_keys(ft) != 2) {
		fprintf(stderr, "compress_split_diverge_early: count != 2\n");
		return drain_and_destroy(ft, group) | -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Split at late divergence: two 4-byte keys that share the first 3
 * bytes.  The split happens at the last byte of the compressed path.
 */
static int test_compress_split_diverge_late(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_node *found;

	rcu_read_lock();
	if (insert_u64(ft, 0, node_alloc(0)) != CDS_FT_STATUS_OK ||
	    insert_u64(ft, 1, node_alloc(1)) != CDS_FT_STATUS_OK ||
	    insert_u64(ft, 2, node_alloc(2)) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_split_diverge_late: insert failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	/* All three should be findable. */
	if (lookup_u64(ft, 0, &found) != CDS_FT_STATUS_OK || !found ||
	    lookup_u64(ft, 1, &found) != CDS_FT_STATUS_OK || !found ||
	    lookup_u64(ft, 2, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_split_diverge_late: lookup failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);
}

/*
 * Remove through compressed path: insert two keys sharing a prefix,
 * remove one, verify the other remains.  The compressed path between
 * root and the branch point should survive the remove.
 */
static int test_compress_remove_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n0 = node_alloc(0), *n1 = node_alloc(1);
	struct cds_ft_node *found;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	insert_u64(ft, 0, n0);
	insert_u64(ft, 1, n1);

	/* Remove key 0, verify key 1 still present. */
	{
		uint8_t k[4];

		cds_ft_u64_to_key(ft, 0, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ft, iter);
	}
	if (cds_ft_remove(ft, iter, &n0->node) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_remove_through: remove failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	node_free_rcu(n0);
	if (lookup_u64(ft, 1, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_remove_through: lookup 1 failed after remove 0\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	if (lookup_u64(ft, 0, &found) == CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_remove_through: key 0 still present\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	if (cds_ft_count_keys(ft) != 1) {
		fprintf(stderr, "compress_remove_through: count != 1\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Remove causing upward pruning through compressed node:
 * insert a single key (creates compressed path), then remove it.
 * ft_detach_node should prune through the compressed node.
 */
static int test_compress_remove_prune_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n = node_alloc(42);

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	insert_u64(ft, 42, n);
	{
		uint8_t k[4];

		cds_ft_u64_to_key(ft, 42, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ft, iter);
	}
	if (cds_ft_remove(ft, iter, &n->node) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_remove_prune: remove failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	node_free_rcu(n);
	if (!cds_ft_empty(ft)) {
		fprintf(stderr, "compress_remove_prune: trie not empty\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Variable-length keys with external_nodes on compressed node:
 * insert "abc" and "abcdef".  "abc" is a prefix of "abcdef", so
 * "abc" should be stored as external_nodes at the depth where the
 * compressed path begins.  Verify lookup, remove of the short key,
 * and that the long key survives.
 */
static int test_compress_varlen_external_nodes(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(1), *n2 = node_alloc(2);
	struct cds_ft_node *found;
	const uint8_t *k1 = (const uint8_t *)"abc";
	const uint8_t *k2 = (const uint8_t *)"abcdef";

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	if (cds_ft_insert(ft, k1, 3, &n1->node) != CDS_FT_STATUS_OK ||
	    cds_ft_insert(ft, k2, 6, &n2->node) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_varlen_ext: insert failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Both should be findable. */
	if (cds_ft_lookup_key(ft, k1, 3, &found) != CDS_FT_STATUS_OK || !found ||
	    cds_ft_lookup_key(ft, k2, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_varlen_ext: lookup failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	if (cds_ft_count_keys(ft) != 2) {
		fprintf(stderr, "compress_varlen_ext: count != 2\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Remove short key, verify long key survives. */
	cds_ft_iter_set_key(iter, k1, 3);
	cds_ft_lookup(ft, iter);
	if (cds_ft_remove(ft, iter, &n1->node) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_varlen_ext: remove short failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	node_free_rcu(n1);
	if (cds_ft_lookup_key(ft, k2, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_varlen_ext: long key gone after short remove\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	if (cds_ft_count_keys(ft) != 1) {
		fprintf(stderr, "compress_varlen_ext: count != 1 after remove\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Graft into a position that diverges within a compressed path.
 * Insert "abcdef" (creates compressed path), then graft a subtrie
 * at "abx" (diverges at third byte of compressed path).
 * Variable-length keys required for non-root graft.
 */
static int test_compress_graft_diverge(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft *src;
	struct cds_ft_node *found;
	const uint8_t *k1 = (const uint8_t *)"abcdef";
	const uint8_t *k_graft = (const uint8_t *)"abx";
	const uint8_t *k_src = (const uint8_t *)"QR";
	enum cds_ft_status s;

	if (cds_ft_create(group, &src) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	/* Insert into main trie. */
	cds_ft_insert(ft, k1, 6, &node_alloc(0)->node);

	/* Insert into source trie (key relative to graft point). */
	cds_ft_insert(src, k_src, 2, &node_alloc(1)->node);

	/* Graft source at "abx" — diverges at byte 2 of "abcdef" path. */
	s = cds_ft_graft(ft, k_graft, 3, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_graft_diverge: graft failed: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		cds_ft_destroy(src);
		return drain_and_destroy(ft, group) | -1;
	}

	/* Original key should still exist. */
	if (cds_ft_lookup_key(ft, k1, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_graft_diverge: lookup original failed\n");
		rcu_read_unlock();
		cds_ft_destroy(src);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Grafted key "abxQR" should exist. */
	{
		const uint8_t *k_full = (const uint8_t *)"abxQR";

		if (cds_ft_lookup_key(ft, k_full, 5, &found) != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "compress_graft_diverge: lookup grafted key failed\n");
			rcu_read_unlock();
			cds_ft_destroy(src);
			return drain_and_destroy(ft, group) | -1;
		}
	}
	rcu_read_unlock();
	cds_ft_destroy(src);
	return drain_and_destroy(ft, group);
}

/*
 * Detach through compressed path: insert two keys sharing a prefix,
 * detach one, verify the other remains and the detached subtrie
 * contains the removed content.
 */
static int test_compress_detach_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft *detached = NULL;
	struct cds_ft_node *found;
	struct ft_test_node *n1 = node_alloc(1), *n2 = node_alloc(2);
	const uint8_t *k1 = (const uint8_t *)"abcXYZ";
	const uint8_t *k2 = (const uint8_t *)"abcDEF";
	const uint8_t *prefix = (const uint8_t *)"abcX";

	rcu_read_lock();
	cds_ft_insert(ft, k1, 6, &n1->node);
	cds_ft_insert(ft, k2, 6, &n2->node);

	if (cds_ft_detach(ft, prefix, 4, &detached) != CDS_FT_STATUS_OK || !detached) {
		fprintf(stderr, "compress_detach_through: detach failed\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	/* k2 should remain, k1 should be in detached (under relative key). */
	if (cds_ft_lookup_key(ft, k2, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_detach_through: k2 missing after detach\n");
		rcu_read_unlock();
		goto fail_detach;
	}
	if (cds_ft_lookup_key(ft, k1, 6, &found) == CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_detach_through: k1 still in trie\n");
		rcu_read_unlock();
		goto fail_detach;
	}
	rcu_read_unlock();
	/* Drain detached trie. */
	{
		struct cds_ft_iter *dit;

		cds_ft_iter_create(detached, &dit);
		rcu_read_lock();
		while (cds_ft_lookup_first(detached, dit) == CDS_FT_STATUS_OK) {
			struct cds_ft_node *head, *tmp;

			cds_ft_remove_all(detached, dit, &head);
			cds_ft_for_each_duplicate_safe_rcu(head, tmp)
				node_free_rcu(to_test_node(head));
		}
		rcu_read_unlock();
		rcu_barrier();
		cds_ft_iter_destroy(dit);
	}
	cds_ft_destroy(detached);
	return drain_and_destroy(ft, group);
fail_detach:
	rcu_barrier();
	cds_ft_destroy(detached);
	return drain_and_destroy(ft, group) | -1;
}

/*
 * Iteration order through compressed paths: insert 10 keys with
 * 4-byte fixed length (0..9), verify forward iteration is sorted.
 */
static int test_compress_iteration_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i, prev = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	for (i = 0; i < 10; i++)
		insert_u64(ft, i, node_alloc(i));
	i = 0;
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t k[4];
		size_t klen;
		uint64_t val;

		cds_ft_iter_get_key(iter, k, 4, &klen);
		val = cds_ft_key_to_u64(ft, k, 4);
		if (i > 0 && val <= prev) {
			fprintf(stderr, "compress_iteration: order violation at %lu: %lu <= %lu\n",
				i, (unsigned long)val, (unsigned long)prev);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			return drain_and_destroy(ft, group) | -1;
		}
		prev = val;
		i++;
	}
	rcu_read_unlock();
	if (i != 10) {
		fprintf(stderr, "compress_iteration: iterated %lu, expected 10\n", i);
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * lookup_nth through compressed paths: insert 10 keys, verify
 * lookup_nth(0) through lookup_nth(9) and that lookup_nth(10)
 * returns NOT_FOUND.
 */
static int test_compress_lookup_nth_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	for (i = 0; i < 10; i++)
		insert_u64(ft, i, node_alloc(i));

	for (i = 0; i < 10; i++) {
		if (cds_ft_lookup_nth(ft, iter, i) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "compress_nth: nth(%lu) failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			return drain_and_destroy(ft, group) | -1;
		}
	}
	if (cds_ft_lookup_nth(ft, iter, 10) != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "compress_nth: nth(10) should be NOT_FOUND\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Inequality lookups through compressed paths: insert string keys
 * "apple", "banana", "cherry", verify GE("band") returns "banana",
 * LE("cat") returns "cherry" ... actually "banana" (cat < cherry).
 * Uses variable-length keys so compressed paths are exercised
 * within the key itself, not just in the shared prefix.
 */
static int test_compress_inequality_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct cds_ft_node *node;
	uint8_t rk[32];
	size_t rklen;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"apple", 5, &node_alloc(0)->node);
	cds_ft_insert(ft, (const uint8_t *)"banana", 6, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *)"cherry", 6, &node_alloc(2)->node);

	/* GE("band") should return "banana". */
	cds_ft_iter_set_key(iter, (const uint8_t *)"band", 4);
	if (cds_ft_lookup_ge(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_ineq: ge(band) failed\n");
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "compress_ineq: ge(band) got len=%zu\n", rklen);
		rcu_read_unlock();
		goto fail;
	}

	/* LE("cat") should return "banana" (banana < cat < cherry). */
	cds_ft_iter_set_key(iter, (const uint8_t *)"cat", 3);
	if (cds_ft_lookup_le(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_ineq: le(cat) failed\n");
		rcu_read_unlock();
		goto fail;
	}
	node = cds_ft_iter_node(iter);
	if (!node) {
		fprintf(stderr, "compress_ineq: le(cat) no node\n");
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "compress_ineq: le(cat) got len=%zu key=%.6s\n",
			rklen, (char *)rk);
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
fail:
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group) | -1;
}

/*
 * Nested compressed nodes: use variable-length keys to create a trie
 * where one compressed path leads to another.  Insert "abcdef" and
 * "abcxyz" (share "abc" prefix → compressed, then separate compressed
 * suffixes "def" and "xyz").  Verify lookup, iteration, and removal.
 */
static int test_compress_nested(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(1), *n2 = node_alloc(2);
	struct cds_ft_node *found;
	const uint8_t *k1 = (const uint8_t *)"abcdef";
	const uint8_t *k2 = (const uint8_t *)"abcxyz";
	unsigned long count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	cds_ft_insert(ft, k1, 6, &n1->node);
	cds_ft_insert(ft, k2, 6, &n2->node);

	if (cds_ft_lookup_key(ft, k1, 6, &found) != CDS_FT_STATUS_OK || !found ||
	    cds_ft_lookup_key(ft, k2, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_nested: lookup failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Iterate and count. */
	cds_ft_for_each_rcu(ft, iter)
		count++;
	if (count != 2) {
		fprintf(stderr, "compress_nested: iterated %lu, expected 2\n", count);
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Remove one, verify other survives. */
	cds_ft_iter_set_key(iter, k1, 6);
	cds_ft_lookup(ft, iter);
	cds_ft_remove(ft, iter, &n1->node);
	node_free_rcu(n1);
	if (cds_ft_lookup_key(ft, k2, 6, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_nested: k2 missing after k1 remove\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*                           MAIN                                     */
/*                                                                    */
/* ================================================================== */

int main(int argc, char **argv)
{
	const char *filter = (argc >= 2) ? argv[1] : NULL;
	int err;

	err = create_all_cpu_call_rcu_data(0);
	if (err)
		diag("Per-CPU call_rcu() workers unavailable, using default.");

	rcu_register_thread();

	plan_tests(NR_TESTS);

	/* 1. Lifecycle & attributes */
	diag("Lifecycle & attribute tests");
	RUN_TEST(test_lifecycle_defaults);
	RUN_TEST(test_lifecycle_fixed_key_lengths);
	RUN_TEST(test_lifecycle_nil_only_trie);
	RUN_TEST(test_lifecycle_max_key_len);
	RUN_TEST(test_lifecycle_key_map);
	RUN_TEST(test_status_to_string);

	/* 2. Insert variants */
	diag("Insert variant tests");
	RUN_TEST(test_insert_basic);
	RUN_TEST(test_insert_unique);
	RUN_TEST(test_insert_duplicate_chain);
	RUN_TEST(test_insert_replace);
	RUN_TEST(test_count_tracking);

	/* Key count (cds_ft_count_keys) tests */
	diag("Key count tests");
	RUN_TEST(test_count_keys_empty);
	RUN_TEST(test_count_keys_duplicates);
	RUN_TEST(test_count_keys_distinct);
	RUN_TEST(test_count_keys_remove);
	RUN_TEST(test_count_keys_remove_all);
	RUN_TEST(test_count_keys_replace);
	RUN_TEST(test_count_keys_graft_detach);
	RUN_TEST(test_count_keys_prefix_basic);
	RUN_TEST(test_count_keys_prefix_duplicates);
	RUN_TEST(test_count_keys_prefix_fixed);

	/* Rank-based lookup (cds_ft_lookup_nth) tests */
	diag("Rank-based lookup tests");
	RUN_TEST(test_lookup_nth_empty);
	RUN_TEST(test_lookup_nth_basic);
	RUN_TEST(test_lookup_nth_last);
	RUN_TEST(test_lookup_nth_duplicates);
	RUN_TEST(test_lookup_nth_varlen);

	/* Iterator skip tests */
	diag("Iterator skip tests");
	RUN_TEST(test_iter_skip_forward);
	RUN_TEST(test_iter_skip_reverse);
	RUN_TEST(test_iter_skip_boundary);
	RUN_TEST(test_iter_skip_duplicates);
	RUN_TEST(test_iter_skip_varlen);

	/* 3. Lookup variants */
	diag("Lookup variant tests");
	RUN_TEST(test_lookup_exact_hit_miss);
	RUN_TEST(test_lookup_empty_trie);
	RUN_TEST(test_lookup_partial);
	RUN_TEST(test_lookup_longest_match);
	RUN_TEST(test_lookup_relational);
	RUN_TEST(test_lookup_first_last);

	/* 4. Iteration */
	diag("Iteration tests");
	RUN_TEST(test_iteration_forward_order);
	RUN_TEST(test_iteration_reverse_order);
	RUN_TEST(test_iteration_prefix_scoped);
	RUN_TEST(test_iteration_for_each_entry);
	RUN_TEST(test_iter_copy);
	RUN_TEST(test_iter_reset);

	/* 5. Replace & remove_all */
	diag("Replace & remove_all tests");
	RUN_TEST(test_replace_node);
	RUN_TEST(test_remove_all);
	RUN_TEST(test_remove_middle_of_chain);

	/* 6. Key conversion */
	diag("Key conversion tests");
	RUN_TEST(test_key_u64_roundtrip);
	RUN_TEST(test_key_u32_roundtrip);
	RUN_TEST(test_key_s64_roundtrip);
	RUN_TEST(test_key_s32_roundtrip);
	RUN_TEST(test_key_signed_sort_order);

	/* 7. NIL key */
	diag("NIL key tests");
	RUN_TEST(test_nil_key_varlen);

	/* 8. Boundary / error paths */
	diag("Boundary / error path tests");
	RUN_TEST(test_1byte_exhaustive);
	RUN_TEST(test_prefix_split);
	RUN_TEST(test_iter_get_key_overflow);
	RUN_TEST(test_iter_prefix_len_invalid);
	RUN_TEST(test_double_remove);
	RUN_TEST(test_order_after_mid_insert);
	RUN_TEST(test_varlen_string_basic);

	/* 9. Graft, graft_swap & detach */
	diag("Graft, graft_swap & detach tests");
	RUN_TEST(test_graft_basic);
	RUN_TEST(test_graft_at_root);
	RUN_TEST(test_graft_populated_error);
	RUN_TEST(test_graft_different_group_error);
	RUN_TEST(test_graft_self_error);
	RUN_TEST(test_graft_overflow_error);
	RUN_TEST(test_graft_swap_basic);
	RUN_TEST(test_graft_swap_into_empty);
	RUN_TEST(test_graft_swap_at_root);
	RUN_TEST(test_graft_swap_self_error);
	RUN_TEST(test_graft_swap_different_group_error);
	RUN_TEST(test_graft_swap_fixed_key);
	RUN_TEST(test_fixed_graft_at_root);
	RUN_TEST(test_fixed_graft_nonroot_error);
	RUN_TEST(test_fixed_graft_swap_at_root);
	RUN_TEST(test_fixed_graft_swap_nonroot_error);
	RUN_TEST(test_fixed_detach_at_root);
	RUN_TEST(test_fixed_detach_nonroot_error);
	RUN_TEST(test_detach_basic);
	RUN_TEST(test_detach_at_root);
	RUN_TEST(test_detach_not_found);
	RUN_TEST(test_detach_empty_trie);
	RUN_TEST(test_detach_then_graft);

	/* 10. Corner-case & coverage-gap tests */
	diag("Corner-case & coverage-gap tests");
	RUN_TEST(test_recompute_stats);
	RUN_TEST(test_iter_get_prefix);
	RUN_TEST(test_lookup_partial_iter);
	RUN_TEST(test_lookup_longest_match_iter);
	RUN_TEST(test_for_each_entry_reverse);
	RUN_TEST(test_for_each_duplicate_entry);
	RUN_TEST(test_for_each_duplicate_entry_safe);
	RUN_TEST(test_group_destroy_busy_error);
	RUN_TEST(test_graft_swap_overflow_error);
	RUN_TEST(test_replace_not_found);
	RUN_TEST(test_remove_all_not_found);
	RUN_TEST(test_insert_replace_no_existing);
	RUN_TEST(test_insert_exceeds_max_key_len);
	RUN_TEST(test_graft_detach_len_default);
	RUN_TEST(test_graft_empty_source);
	RUN_TEST(test_graft_reuse_after_drain);
	RUN_TEST(test_multiple_tries_same_group);
	RUN_TEST(test_nil_key_fixed_zero_len_trie);
	RUN_TEST(test_nil_key_unique_and_replace);
	RUN_TEST(test_for_each_entry_with_duplicates);
	RUN_TEST(test_iter_set_key_path_invalidation);
	RUN_TEST(test_show_smoke);

	/* 11. Adversarial key & per-node distribution tests */
	diag("Adversarial key & per-node distribution tests");
	RUN_TEST(test_adversarial_ramp_all_configs);
	RUN_TEST(test_adversarial_single_bit_cluster);
	RUN_TEST(test_adversarial_same_nibble_cluster);
	RUN_TEST(test_adversarial_two_bit_cluster);
	RUN_TEST(test_adversarial_transition_oscillation);
	RUN_TEST(test_adversarial_sparse_removal);
	RUN_TEST(test_adversarial_boundary_bytes);
	RUN_TEST(test_adversarial_prefix_nesting);
	RUN_TEST(test_adversarial_mass_duplicates);
	RUN_TEST(test_adversarial_alternating_bits);
	RUN_TEST(test_adversarial_max_depth);
	RUN_TEST(test_adversarial_reverse_insert_order);
	RUN_TEST(test_adversarial_xor_scramble_order);
	RUN_TEST(test_adversarial_interleaved_grow);
	RUN_TEST(test_adversarial_relational_gap);
	RUN_TEST(test_adversarial_power_of_two_stride);
	RUN_TEST(test_adversarial_shared_suffix);
	RUN_TEST(test_adversarial_longest_match_gaps);
	RUN_TEST(test_adversarial_replace_churn);

	/* 12. Uncached iterator path mode tests */
	diag("Uncached iterator path mode tests");
	RUN_TEST(test_iter_path_mode_default);
	RUN_TEST(test_iter_uncached_forward);
	RUN_TEST(test_iter_uncached_reverse);
	RUN_TEST(test_iter_uncached_lookup_remove);
	RUN_TEST(test_iter_uncached_prefix_scoped);
	RUN_TEST(test_iter_path_mode_switch);
	RUN_TEST(test_iter_uncached_copy);
	RUN_TEST(test_iter_uncached_all_configs);

	/* 13. Compressed node corner case tests */
	diag("Compressed node corner case tests");
	RUN_TEST(test_compress_long_prefix);
	RUN_TEST(test_compress_split_diverge_early);
	RUN_TEST(test_compress_split_diverge_late);
	RUN_TEST(test_compress_remove_through);
	RUN_TEST(test_compress_remove_prune_through);
	RUN_TEST(test_compress_varlen_external_nodes);
	RUN_TEST(test_compress_graft_diverge);
	RUN_TEST(test_compress_iteration_order);
	RUN_TEST(test_compress_lookup_nth_through);
	RUN_TEST(test_compress_nested);

	rcu_barrier();
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();

	return exit_status();
}
