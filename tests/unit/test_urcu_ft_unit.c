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

#define NR_TESTS 56

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
	ft_count = cds_ft_count(ft);
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
	ft_count = cds_ft_count(ft);
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
	ft_count = cds_ft_count(ft);
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
	ft_count = cds_ft_count(ft);
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
 * cds_ft_count tracks correctly across multiple inserts.
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
	ft_count = cds_ft_count(ft);
	rcu_read_unlock();
	if (ft_count != 50) {
		fprintf(stderr, "count %lu, expected 50\n", ft_count);
		drain_and_destroy(ft, group);
		return -1;
	}
	ret = drain_and_destroy(ft, group);
	return ret;
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

	ft_count = cds_ft_count(ft);
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

	ft_count = cds_ft_count(ft);
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
	int64_t prev;
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
	ft_count = cds_ft_count(ft);
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
	count = cds_ft_count(live);
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
	count = cds_ft_count(swap);
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
	live_count = cds_ft_count(live);
	swap_count = cds_ft_count(swap);
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
	count = cds_ft_count(ft);
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
	count = cds_ft_count(detached);
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
	count = cds_ft_count(detached);
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
	count = cds_ft_count(ft);
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
	count = cds_ft_count(live);
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
	count = cds_ft_count(swap);
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
	RUN_TEST(test_detach_basic);
	RUN_TEST(test_detach_at_root);
	RUN_TEST(test_detach_not_found);
	RUN_TEST(test_detach_empty_trie);
	RUN_TEST(test_detach_then_graft);

	rcu_barrier();
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();

	return exit_status();
}
