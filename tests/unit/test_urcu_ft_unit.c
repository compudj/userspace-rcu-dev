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
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "tap.h"

#ifdef FEATURE_FT_FAULT_INJECT
#define NR_TESTS 284
#else
#define NR_TESTS 252
#endif

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
#ifdef FT_IMMEDIATE_FREE
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "drain_and_destroy: verify failed\n");
			cds_ft_show(ft, stderr, CDS_FT_SHOW_PRETTY);
			ret = -1;
			break;
		}
#endif
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
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
	if (cds_ft_create(group, NULL, &ft) < 0)
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
	return cds_ft_eager_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, 0, out);
}

/* Exact-key presence check for a NUL-terminated string key. */
static int ft_test_has_key(struct cds_ft *ft, const char *k)
{
	struct cds_ft_node *out = NULL;
	size_t len = strlen(k);

	return cds_ft_eager_lookup_key(ft, (const uint8_t *) k, len, 0,
			&out) == CDS_FT_STATUS_OK;
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
		if (exclude && strstr(exclude, #fn)) {			\
			skip(1, "excluded: " #fn);			\
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
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_group_key_len(group) != CDS_FT_LEN_VARIABLE) {
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
 * cds_ft_group_create_flavor: the explicit-flavor creator (the form
 * required when including <urcu/urcu-*.h> modern flavor headers, which
 * clear the URCU API mapping) binds the group to the passed flavor.
 * This test builds with the legacy QSBR include, so urcu_qsbr_flavor
 * is the same flavor the mapped cds_ft_group_create() would pick.
 */
static int test_lifecycle_group_create_flavor(void)
{
	struct cds_ft_group *group;
	struct ft_test_node *n;
	struct cds_ft *ft;
	enum cds_ft_status s;

	if (cds_ft_group_create_flavor(NULL, &group, &urcu_qsbr_flavor) != CDS_FT_STATUS_OK)
		return -1;
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	n = node_alloc(42);
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *) "flv", 3, &n->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "insert failed: %s\n", cds_ft_status_to_string(s));
		node_free(n);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	return drain_and_destroy(ft, group);
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

		if (cds_ft_group_key_len(group) != klen) {
			fprintf(stderr, "key_len mismatch for %u-byte trie\n", klen);
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
		if (cds_ft_group_max_key_len(group) < klen) {
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

	if (cds_ft_group_key_len(group) != 0) {
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
 * Exercise cds_ft_group_attr_set_max_key_len.
 */
static int test_lifecycle_max_key_len(void)
{
	struct cds_ft_group *group;
	struct cds_ft_group_attr *attr;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	/* Variable-length keys with a 32-byte maximum. */
	if (cds_ft_group_attr_set_max_key_len(attr, 32) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	if (cds_ft_group_max_key_len(group) != 32) {
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
	struct cds_ft_group_attr *attr;
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

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, 1) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	s = cds_ft_group_attr_set_key_map(attr, k2o, o2k);
	if (s == CDS_FT_STATUS_NOT_SUPPORTED) {
		/* Built with NO_FEATURE_FT_KEY_MAP -- skip the non-identity case. */
		cds_ft_group_attr_destroy(attr);
		return 0;
	}
	if (s < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	s = cds_ft_group_key_map(group, k2o_out, o2k_out);
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
 * Head-promotion in a duplicate chain: after removing the head, the new
 * head's prev must point to the parent (was the old head, a cds_ft_node).
 * This verifies the transfer-of-prev logic in ft_unchain_node.
 *
 * Also exercises successive head removals and a non-head removal, ensuring
 * prev remains consistent across the doubly-linked chain.
 */
static int test_dup_chain_head_promotion(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct ft_test_node *n[4];
	struct cds_ft_node *found;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	uint8_t k[4];
	void *parent;
	int i;

	for (i = 0; i < 4; i++)
		n[i] = node_alloc(42);

	cds_ft_u64_to_key(ft, 42, k, CDS_FT_LEN_DEFAULT);

	if (cds_ft_iter_create(ft, &iter) < 0)
		goto fail;

	rcu_read_lock();
	for (i = 0; i < 4; i++) {
		s = cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr,
				"dup_chain_head_promotion: insert %d: %s\n",
				i, cds_ft_status_to_string(s));
			goto fail_rcu;
		}
	}

	/* Snapshot the parent pointer from the current head's prev. */
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (found != &n[0]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: expected n[0] as head, got %p\n",
			found);
		goto fail_rcu;
	}
	parent = n[0]->node.prev;
	if (!parent) {
		fprintf(stderr,
			"dup_chain_head_promotion: head prev is NULL\n");
		goto fail_rcu;
	}

	/*
	 * Remove the head (n[0]).  n[1] should become the new head with
	 * prev == parent (was n[0], a cds_ft_node, before promotion).
	 */
	s = cds_ft_remove(ft, iter, &n[0]->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr,
			"dup_chain_head_promotion: remove n[0]: %s\n",
			cds_ft_status_to_string(s));
		goto fail_rcu;
	}

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (found != &n[1]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: expected n[1] as new head, got %p\n",
			found);
		goto fail_rcu;
	}
	if (n[1]->node.prev != parent) {
		fprintf(stderr,
			"dup_chain_head_promotion: n[1] prev %p != parent %p\n",
			n[1]->node.prev, parent);
		goto fail_rcu;
	}
	/* n[2] and n[3] are non-head; their prev should point back into the chain. */
	if (n[2]->node.prev != &n[1]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: n[2] prev %p != n[1] %p\n",
			n[2]->node.prev, &n[1]->node);
		goto fail_rcu;
	}
	if (n[3]->node.prev != &n[2]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: n[3] prev %p != n[2] %p\n",
			n[3]->node.prev, &n[2]->node);
		goto fail_rcu;
	}

	/* Remove a non-head in the middle (n[2]). */
	s = cds_ft_remove(ft, iter, &n[2]->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr,
			"dup_chain_head_promotion: remove n[2]: %s\n",
			cds_ft_status_to_string(s));
		goto fail_rcu;
	}
	/* n[3]'s prev should now point to n[1] (its new predecessor). */
	if (n[3]->node.prev != &n[1]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: n[3] prev after mid-remove %p != n[1] %p\n",
			n[3]->node.prev, &n[1]->node);
		goto fail_rcu;
	}

	/* Remove the new head (n[1]).  n[3] should promote. */
	s = cds_ft_remove(ft, iter, &n[1]->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr,
			"dup_chain_head_promotion: remove n[1]: %s\n",
			cds_ft_status_to_string(s));
		goto fail_rcu;
	}
	if (n[3]->node.prev != parent) {
		fprintf(stderr,
			"dup_chain_head_promotion: n[3] prev after second promotion %p != parent %p\n",
			n[3]->node.prev, parent);
		goto fail_rcu;
	}

	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (found != &n[3]->node) {
		fprintf(stderr,
			"dup_chain_head_promotion: expected n[3] as final head, got %p\n",
			found);
		goto fail_rcu;
	}

	/* Remove the last remaining node (n[3]). */
	s = cds_ft_remove(ft, iter, &n[3]->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr,
			"dup_chain_head_promotion: remove n[3]: %s\n",
			cds_ft_status_to_string(s));
		goto fail_rcu;
	}
	rcu_read_unlock();

	cds_ft_iter_destroy(iter);
	for (i = 0; i < 4; i++)
		call_rcu(&n[i]->head, node_free_rcu_cb);
	return drain_and_destroy(ft, group);

fail_rcu:
	rcu_read_unlock();
fail:
	if (iter)
		cds_ft_iter_destroy(iter);
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
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
 * cds_ft_node_get_key: materialize a key from a bare external node pointer,
 * without an iterator.  Drive a batched cell-walk to gather node pointers, then
 * reconstruct each one's key via cds_ft_node_get_key and check it matches both
 * the iterator's own get_key and the expected in-order value.  Exercises the
 * EAGER parent up-walk source (a fixed-length ordered group has no in-leaf key).
 */
static int test_node_get_key(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	const struct cds_ft_cell *buf[16];
	const struct cds_ft_cell *cur;
	size_t off = cds_ft_cell_node_offset();
	unsigned long i, seen = 0;
	size_t got, b;

	for (i = 0; i < 10; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			goto fail;
		}
		rcu_read_unlock();
	}

	/*
	 * One continuous read-side critical section: cell handles are valid.
	 * Gather cells with the iterator-free cell batch (NO iterator), recover
	 * each node with cds_ft_cell_node, and materialize its key with
	 * cds_ft_node_get_key.
	 */
	rcu_read_lock();
	cur = NULL;
	do {
		enum cds_ft_status bs = cds_ft_cell_next_batch(ft, cur, buf, 16,
				&got, &cur);

		if (bs == CDS_FT_STATUS_NOT_SUPPORTED) {
			/* No ordered list (and no in-leaf key): N/A -- pass. */
			rcu_read_unlock();
			return drain_and_destroy(ft, group);
		}
		if (bs != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "node_get_key: batch status %d\n", (int) bs);
			goto fail;
		}
		for (b = 0; b < got; b++) {
			struct cds_ft_node *node = cds_ft_cell_node(buf[b], off);
			uint8_t k[4];
			size_t klen;
			uint64_t v;

			if (cds_ft_node_get_key(ft, node, k, sizeof(k), &klen)
					!= CDS_FT_STATUS_OK || klen != 4) {
				rcu_read_unlock();
				fprintf(stderr, "node_get_key: bad result at %lu\n",
					seen);
				goto fail;
			}
			v = cds_ft_key_to_u64(ft, k, 4);
			if (v != seen) {
				rcu_read_unlock();
				fprintf(stderr, "node_get_key: got %lu expected %lu\n",
					(unsigned long) v, seen);
				goto fail;
			}
			seen++;
		}
	} while (cur);
	rcu_read_unlock();
	if (seen != 10) {
		fprintf(stderr, "node_get_key: saw %lu of 10 keys\n", seen);
		goto fail;
	}

	return drain_and_destroy(ft, group);
fail:
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_cell_next_batch / cds_ft_cell_prev_batch: the fully iterator-free
 * ordered scan.  Walk all keys forward (cursor seeded NULL = list minimum) and
 * reverse (NULL = maximum) with a small batch buffer (forcing several batches),
 * resolving each cell's key via cds_ft_cell_get_key, and check the order.
 * A build with no ordered cell list returns NOT_SUPPORTED -- treated as N/A.
 */
static int test_node_batch(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	const struct cds_ft_cell *buf[4];
	const struct cds_ft_cell *cur;
	size_t off = cds_ft_cell_node_offset();
	unsigned long i, seen;
	size_t n, b;

	for (i = 0; i < 10; i++) {
		struct ft_test_node *nd = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, nd) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			goto fail;
		}
		rcu_read_unlock();
	}

	/* Forward: cap 4 over 10 keys forces multiple batches. */
	rcu_read_lock();
	cur = NULL;
	seen = 0;
	do {
		enum cds_ft_status bs = cds_ft_cell_next_batch(ft, cur, buf, 4,
				&n, &cur);

		if (bs == CDS_FT_STATUS_NOT_SUPPORTED) {
			/* List-off trie: the cell-cursor walk is N/A. */
			rcu_read_unlock();
			return drain_and_destroy(ft, group);
		}
		if (bs != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "node_batch fwd: status %d\n", (int) bs);
			goto fail;
		}
		for (b = 0; b < n; b++) {
			uint8_t k[4];
			size_t kl;

			if (cds_ft_cell_get_key(ft, buf[b], k, sizeof k, &kl)
					!= CDS_FT_STATUS_OK ||
					cds_ft_key_to_u64(ft, k, 4) != seen) {
				rcu_read_unlock();
				fprintf(stderr, "node_batch fwd: mismatch at %lu\n",
					seen);
				goto fail;
			}
			seen++;
		}
	} while (cur);
	rcu_read_unlock();
	if (seen != 10) {
		fprintf(stderr, "node_batch fwd: saw %lu of 10\n", seen);
		goto fail;
	}

	/* Reverse: NULL cursor starts at the maximum, stepping down. */
	rcu_read_lock();
	cur = NULL;
	seen = 0;
	do {
		if (cds_ft_cell_prev_batch(ft, cur, buf, 4, &n, &cur)
				!= CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "node_batch rev: bad status\n");
			goto fail;
		}
		for (b = 0; b < n; b++) {
			uint8_t k[4];
			size_t kl;

			if (cds_ft_cell_get_key(ft, buf[b], k, sizeof k, &kl)
					!= CDS_FT_STATUS_OK ||
					cds_ft_key_to_u64(ft, k, 4) != 9 - seen) {
				rcu_read_unlock();
				fprintf(stderr, "node_batch rev: mismatch at %lu\n",
					seen);
				goto fail;
			}
			seen++;
		}
	} while (cur);
	rcu_read_unlock();
	if (seen != 10) {
		fprintf(stderr, "node_batch rev: saw %lu of 10\n", seen);
		goto fail;
	}

	/*
	 * Exercise the cell-cursor batched MACROS (no iterator in scope).  A tiny
	 * @cap forces multiple internal batches; cross-check the key order.
	 */
	{
		const struct cds_ft_cell *cell, *mbuf[3];
		unsigned long fwd = 0, rev = 0;

		rcu_read_lock();
		cds_ft_for_each_batched_rcu(ft, cell, mbuf, 3) {
			if (to_test_node(cds_ft_cell_node(cell, off))->key != fwd) {
				rcu_read_unlock();
				fprintf(stderr, "node_batch macro fwd: [%lu]=%llu\n", fwd,
					(unsigned long long) to_test_node(cds_ft_cell_node(cell, off))->key);
				goto fail;
			}
			fwd++;
		}
		cds_ft_for_each_reverse_batched_rcu(ft, cell, mbuf, 3) {
			if (to_test_node(cds_ft_cell_node(cell, off))->key != 9 - rev) {
				rcu_read_unlock();
				fprintf(stderr, "node_batch macro rev: [%lu]=%llu\n", rev,
					(unsigned long long) to_test_node(cds_ft_cell_node(cell, off))->key);
				goto fail;
			}
			rev++;
		}
		rcu_read_unlock();
		if (fwd != 10 || rev != 10) {
			fprintf(stderr, "node_batch macro: fwd %lu rev %lu\n", fwd, rev);
			goto fail;
		}
	}

	return drain_and_destroy(ft, group);
fail:
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * `break` inside cds_ft_for_each_batched_rcu / _reverse_ terminates the
 * whole traversal.  The macros used to expand to a refill loop nested
 * around a batch loop, so a user break only exited the current batch
 * and iteration silently resumed with the next refill (2026-06 review
 * finding 5.4).  With cap 4 over 10 keys, breaking at the 6th visit
 * crosses a batch boundary: the buggy shape resumed and visited all 10.
 */
static int test_batched_break(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	const struct cds_ft_cell *cell, *mbuf[4];
	size_t off = cds_ft_cell_node_offset();
	unsigned long seen;
	uint64_t i;

	rcu_read_lock();
	for (i = 0; i < 10; i++) {
		if (insert_u64(ft, i, node_alloc(i)) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "batched_break: insert %llu failed\n",
				(unsigned long long) i);
			goto fail;
		}
	}

	seen = 0;
	cds_ft_for_each_batched_rcu(ft, cell, mbuf, 4) {
		if (to_test_node(cds_ft_cell_node(cell, off))->key != seen) {
			rcu_read_unlock();
			fprintf(stderr, "batched_break fwd: order mismatch\n");
			goto fail;
		}
		if (++seen == 6)
			break;
	}
	if (seen != 6) {
		rcu_read_unlock();
		fprintf(stderr, "batched_break fwd: visited %lu, want 6\n", seen);
		goto fail;
	}

	seen = 0;
	cds_ft_for_each_reverse_batched_rcu(ft, cell, mbuf, 4) {
		if (to_test_node(cds_ft_cell_node(cell, off))->key != 9 - seen) {
			rcu_read_unlock();
			fprintf(stderr, "batched_break rev: order mismatch\n");
			goto fail;
		}
		if (++seen == 6)
			break;
	}
	if (seen != 6) {
		rcu_read_unlock();
		fprintf(stderr, "batched_break rev: visited %lu, want 6\n", seen);
		goto fail;
	}

	/* `continue` skips to the next cell, batch boundaries included. */
	seen = 0;
	cds_ft_for_each_batched_rcu(ft, cell, mbuf, 4) {
		if (to_test_node(cds_ft_cell_node(cell, off))->key & 1)
			continue;
		seen++;
	}
	if (seen != 5) {
		rcu_read_unlock();
		fprintf(stderr, "batched_break continue: %lu even, want 5\n", seen);
		goto fail;
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);
fail:
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * cds_ft_lookup_first/last on a miss must leave the iterator's key
 * length unchanged.  Both overwrite iter->key_len with the descent's
 * working length (prefix_len for first, max_key_len for last) and used
 * to restore the caller's length only on error (< 0): a NOT_FOUND left
 * the working length behind (2026-06 review finding 5.4).
 */
static int test_first_last_keylen_on_miss(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter = NULL;
	struct ft_test_node *n;
	enum cds_ft_status s;
	uint8_t rk[64];
	size_t rl;
	int ret = -1;

	if (cds_ft_group_create(NULL, &group) < 0)
		return -1;
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	n = node_alloc(0);
	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *) "zz", 2, &n->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "keylen_on_miss: insert failed\n");
		node_free(n);
		goto out;
	}
	if (cds_ft_iter_create(ft, &iter) < 0)
		goto out;

	rcu_read_lock();
	/* Scope to 'a': nothing matches, both endpoint lookups miss. */
	cds_ft_iter_set_key(iter, (const uint8_t *) "abc", 3);
	cds_ft_iter_set_prefix_len(iter, 1);
	s = cds_ft_lookup_first(ft, iter);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		rcu_read_unlock();
		fprintf(stderr, "keylen_on_miss: first status %d\n", s);
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
	if (rl != 3 || rk[0] != 'a') {
		rcu_read_unlock();
		fprintf(stderr, "keylen_on_miss: first left len %zu (want 3)\n", rl);
		goto out;
	}
	s = cds_ft_lookup_last(ft, iter);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		rcu_read_unlock();
		fprintf(stderr, "keylen_on_miss: last status %d\n", s);
		goto out;
	}
	if (cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) != CDS_FT_STATUS_OK ||
			rl != 3 || rk[0] != 'a') {
		rcu_read_unlock();
		fprintf(stderr, "keylen_on_miss: last left len %zu (want 3)\n", rl);
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	if (drain_and_destroy(ft, group))
		return -1;
	return ret;
}

/*
 * cds_ft_node_get_key with the ordered LIST disabled at runtime
 * (cds_ft_group_attr_set_ordered_list(attr, false)).  With the list off the trie
 * allocates NO ordinal cells (runtime cell-optional), so this no-in-leaf-key
 * group has no node-alone key source: node_get_key returns NOT_FOUND -- treated
 * as N/A here, same as a non-cell build.  (A group WITH an in-leaf key would
 * still resolve via the leaf even with the list off.)  The iterator-free
 * stepper, which IS the ord-list walk, returns nothing.
 */
static int test_node_get_key_no_list(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	const struct cds_ft_cell *buf[4];
	const struct cds_ft_cell *cur;
	unsigned long i;
	size_t n;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, 4) < 0 ||
			cds_ft_group_attr_set_ordered_list(attr, false) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	for (i = 0; i < 10; i++) {
		struct ft_test_node *nd = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, nd) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			goto fail;
		}
		rcu_read_unlock();
	}

	for (i = 0; i < 10; i++) {
		uint8_t kb[4], k[4];
		struct cds_ft_node *found = NULL;
		size_t kl;
		enum cds_ft_status s;

		cds_ft_u64_to_key(ft, i, kb, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		if (cds_ft_eager_lookup_key(ft, kb, 4, 0, &found)
				!= CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			goto fail;
		}
		s = cds_ft_node_get_key(ft, found, k, sizeof k, &kl);
		rcu_read_unlock();
		if (s == CDS_FT_STATUS_NOT_FOUND)
			break;	/* non-cell build: N/A for the rest */
		if (s != CDS_FT_STATUS_OK || cds_ft_key_to_u64(ft, k, 4) != i) {
			fprintf(stderr, "node no-list get_key: bad at %lu\n", i);
			goto fail;
		}
	}

	/* The iterator-free stepper needs the list -> NOT_SUPPORTED, not a silent
	 * empty: cds_ft_group_ordered_list() must agree the list is off. */
	{
		enum cds_ft_status bs;

		if (cds_ft_group_ordered_list(group)) {
			fprintf(stderr, "node no-list: cds_ft_group_ordered_list true on no-list trie\n");
			goto fail;
		}
		rcu_read_lock();
		bs = cds_ft_cell_next_batch(ft, NULL, buf, 4, &n, &cur);
		rcu_read_unlock();
		if (bs != CDS_FT_STATUS_NOT_SUPPORTED || n != 0 || cur != NULL) {
			fprintf(stderr, "node no-list next_batch: status %d n %zu cur %p\n",
				(int) bs, n, (const void *) cur);
			goto fail;
		}
	}

	return drain_and_destroy(ft, group);
fail:
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Full point-operation battery on a runtime ordered-list-OFF trie
 * (cds_ft_group_attr_set_ordered_list(attr, false)): with the list off the library
 * allocates NO ordinal cells, so a head's prev IS its flagged parent directly
 * and every op runs the no-cell branch (insert / point lookup / ordered
 * iteration via descent / remove / duplicate-head promotion / remove_all).
 * Structure is checked by FEATURE_FT_VERIFY_AT_MUTATION on every mutation;
 * results are checked against the nodes' shadow keys (a no-in-leaf trie cannot
 * materialize a key from a node alone with the list off, so iteration reads the
 * node returned by the iterator, not cds_ft_node_get_key).
 */
static int test_list_off_ops(void)
{
	static const uint64_t in[] = { 30, 10, 40, 20, 50, 90, 25, 60 };
	static const uint64_t sorted[] = { 10, 20, 25, 30, 40, 50, 60, 90 };
	const unsigned int N = 8;
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct cds_ft_node *found, *head, *tmp;
	struct ft_test_node *dup;
	unsigned int i, seen;
	uint64_t prev;
	enum cds_ft_status s;
	uint8_t k[8];

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, 8) < 0 ||
			cds_ft_group_attr_set_ordered_list(attr, false) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert the distinct keys (no cell: list off). */
	for (i = 0; i < N; i++) {
		rcu_read_lock();
		s = insert_u64(ft, in[i], node_alloc(in[i]));
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "list_off: insert %lu\n", (unsigned long) in[i]);
			goto fail;
		}
	}
	/* A duplicate at key 10 -> a dup chain (head-removal promotion below). */
	dup = node_alloc(10);
	rcu_read_lock();
	s = insert_u64(ft, 10, dup);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK)
		goto fail;

	/* Point lookups: every distinct key present + shadow match; a phantom is absent. */
	for (i = 0; i < N; i++) {
		rcu_read_lock();
		s = lookup_u64(ft, in[i], &found);
		if (s != CDS_FT_STATUS_OK || !found ||
				to_test_node(found)->key != in[i]) {
			rcu_read_unlock();
			fprintf(stderr, "list_off: lookup %lu\n", (unsigned long) in[i]);
			goto fail;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	s = lookup_u64(ft, 999, &found);
	rcu_read_unlock();
	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "list_off: phantom hit\n");
		goto fail;
	}

	/* Forward ordered iteration via DESCENT (no cell list): lookup_first + next,
	 * reading each head node's shadow key -> must be ascending == sorted. */
	rcu_read_lock();
	seen = 0;
	prev = 0;
	for (s = cds_ft_lookup_first(ft, iter); s == CDS_FT_STATUS_OK;
			s = cds_ft_next(ft, iter)) {
		uint64_t v = to_test_node(cds_ft_iter_node(iter))->key;

		if (seen >= N || v != sorted[seen] || (seen && v <= prev)) {
			rcu_read_unlock();
			fprintf(stderr, "list_off: fwd[%u]=%lu\n", seen,
				(unsigned long) v);
			goto fail;
		}
		prev = v;
		seen++;
	}
	rcu_read_unlock();
	if (seen != N) {
		fprintf(stderr, "list_off: fwd saw %u/%u\n", seen, N);
		goto fail;
	}

	/* Reverse iteration: lookup_last + prev -> descending. */
	rcu_read_lock();
	seen = 0;
	for (s = cds_ft_lookup_last(ft, iter); s == CDS_FT_STATUS_OK;
			s = cds_ft_prev(ft, iter)) {
		uint64_t v = to_test_node(cds_ft_iter_node(iter))->key;

		if (seen >= N || v != sorted[N - 1 - seen]) {
			rcu_read_unlock();
			fprintf(stderr, "list_off: rev[%u]=%lu\n", seen,
				(unsigned long) v);
			goto fail;
		}
		seen++;
	}
	rcu_read_unlock();
	if (seen != N)
		goto fail;

	/*
	 * The cell-cursor batched MACRO is ordered-list-only: on this list-off
	 * trie it must iterate NOTHING (cds_ft_cell_next_batch -> NOT_SUPPORTED),
	 * not silently skip a populated trie.  Ordered iteration here goes through
	 * the iterator (cds_ft_for_each_rcu / the lookup_first+next loop above).
	 */
	{
		const struct cds_ft_cell *cell, *mbuf[4];
		unsigned int cnt = 0;

		rcu_read_lock();
		cds_ft_for_each_batched_rcu(ft, cell, mbuf, 4) {
			(void) cell;
			cnt++;
		}
		rcu_read_unlock();
		if (cnt != 0) {
			fprintf(stderr, "list_off: batched macro iterated %u\n", cnt);
			goto fail;
		}
	}

	/* Remove a distinct key (40): absent afterwards. */
	rcu_read_lock();
	cds_ft_u64_to_key(ft, 40, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (!found || cds_ft_remove(ft, iter, found) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "list_off: remove 40\n");
		goto fail;
	}
	node_free_rcu(to_test_node(found));
	rcu_read_unlock();
	rcu_read_lock();
	s = lookup_u64(ft, 40, &found);
	rcu_read_unlock();
	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "list_off: 40 survived remove\n");
		goto fail;
	}

	/* Remove the dup key's chain HEAD: promotion keeps key 10 present. */
	rcu_read_lock();
	cds_ft_u64_to_key(ft, 10, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (!found || cds_ft_remove(ft, iter, found) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	node_free_rcu(to_test_node(found));
	rcu_read_unlock();
	rcu_read_lock();
	s = lookup_u64(ft, 10, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->key != 10) {
		fprintf(stderr, "list_off: 10 lost on head-promotion\n");
		goto fail;
	}

	/* remove_all the remaining 10: key disappears. */
	rcu_read_lock();
	cds_ft_u64_to_key(ft, 10, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	if (cds_ft_remove_all(ft, iter, &head) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_for_each_duplicate_safe_rcu(head, tmp)
		node_free_rcu(to_test_node(head));
	rcu_read_unlock();
	rcu_read_lock();
	s = lookup_u64(ft, 10, &found);
	rcu_read_unlock();
	if (s == CDS_FT_STATUS_OK)
		goto fail;

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
fail:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
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
 * Regression: ordered iteration must not skip a prefix key that terminates
 * at an internal node reached as an immediate sibling.
 *
 * With keys "y" < "z" < "zz1" < "zzz", "z" terminates at an internal node
 * (the 'z' subtree root) that also has the children "zz1"/"zzz".  next("y")
 * backtracks to the root, finds 'z' as the immediate right sibling, and must
 * descend to the smallest key in that subtree, which is the prefix key "z" —
 * not jump straight to the leftmost leaf "zz1".  The bug: the inequality
 * minmax descent set skip_eq_external_nodes whenever the sibling was found
 * without climbing a level (going_up still false), wrongly skipping "z".
 * "a"/"ab"/"abc" cover the analogous case where the prefix node is the min.
 */
static int test_iteration_prefix_key(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	/* Expected key order (sorted). */
	static const char * const sorted[] = {
		"a", "ab", "abc", "y", "z", "zz1", "zzz",
	};
	/* Insert deliberately out of order. */
	static const char * const insert_order[] = {
		"zzz", "abc", "z", "a", "zz1", "y", "ab",
	};
	const unsigned int nr = (unsigned int) (sizeof(sorted) / sizeof(sorted[0]));
	unsigned int i, count = 0;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < nr; i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) insert_order[i],
				strlen(insert_order[i]), &n->node)
				!= CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "prefix_key: insert '%s' failed\n",
				insert_order[i]);
			node_free(n);
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		rcu_read_unlock();
	}

	/* Forward iteration must visit every key, in sorted order. */
	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[16];
		size_t rk_len = 0;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		if (count >= nr || rk_len != strlen(sorted[count]) ||
				memcmp(rk, sorted[count], rk_len) != 0) {
			fprintf(stderr,
				"prefix_key: step %u got '%.*s' (len %zu), expected '%s'\n",
				count, (int) rk_len, (const char *) rk, rk_len,
				count < nr ? sorted[count] : "<end>");
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		count++;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);

	if (count != nr) {
		fprintf(stderr,
			"prefix_key: visited %u keys, expected %u (prefix key skipped?)\n",
			count, nr);
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
	s = cds_ft_eager_lookup_key(ft, NULL, 0, 0, &found);
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
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"abd", 3, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "'abd' unreachable after removing 'abc'\n");
		rcu_read_unlock();
		goto fail;
	}
	/* "abc" should be gone. */
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"abc", 3, 0, &found);
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
		if (cds_ft_eager_lookup_key(ft, (const uint8_t *)words[i],
				      strlen(words[i]), 0, &found) != CDS_FT_STATUS_OK
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

/*
 * Inequality lookups on an exact prefix-key match reached through compressed
 * nodes.  Regression for cds_ft_lookup_lt / cds_ft_lookup_le returning a
 * GREATER key (the compressed subtree's max) instead of the strict predecessor
 * / equal match.  "app" is a prefix of "apple" / "application", inserted after
 * them so it lands as a prefix key on the compressed node spanning "app".
 */
static int test_inequality_prefix_key(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = { "apple", "application", "app" };
	struct ineq_case {
		const char *q;
		enum cds_ft_status (*fn)(struct cds_ft *, struct cds_ft_iter *);
		const char *expect;	/* NULL = expect NOT_FOUND */
	} cases[] = {
		{ "app",   cds_ft_lookup_lt, NULL },	/* nothing < "app" */
		{ "app",   cds_ft_lookup_le, "app" },	/* equal match */
		{ "app",   cds_ft_lookup_gt, "apple" },
		{ "app",   cds_ft_lookup_ge, "app" },
		{ "apple", cds_ft_lookup_lt, "app" },
	};
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		enum cds_ft_status s;

		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) cases[i].q,
			strlen(cases[i].q));
		s = cases[i].fn(ft, iter);
		if (cases[i].expect == NULL) {
			if (s == CDS_FT_STATUS_OK && cds_ft_iter_node(iter)) {
				uint8_t rk[64]; size_t rl;

				cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
				rk[rl] = '\0';
				fprintf(stderr, "ineq('%s'): got '%s', expected NOT_FOUND\n",
					cases[i].q, (char *) rk);
				rcu_read_unlock();
				goto out;
			}
		} else {
			uint8_t rk[64]; size_t rl;

			if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
				fprintf(stderr, "ineq('%s'): not found, expected '%s'\n",
					cases[i].q, cases[i].expect);
				rcu_read_unlock();
				goto out;
			}
			cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
			rk[rl] = '\0';
			if (strcmp((char *) rk, cases[i].expect) != 0) {
				fprintf(stderr, "ineq('%s'): got '%s', expected '%s'\n",
					cases[i].q, (char *) rk, cases[i].expect);
				rcu_read_unlock();
				goto out;
			}
		}
		rcu_read_unlock();
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
 * Inequality lookups where the search key EXTENDS an existing leaf key (the
 * leaf is a proper prefix of the search key).  Regression for cds_ft_lookup_lt
 * / cds_ft_lookup_le returning NOT_FOUND instead of that leaf -- a leaf has no
 * extensions, so it is the largest key <= the search key on its path, but the
 * going-up backtrack skips external leaves.  "ab" is a leaf; "abcd" extends it.
 */
static int test_inequality_extends_prefix(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = { "ab", "m" };
	struct ext_case {
		const char *q;
		enum cds_ft_status (*fn)(struct cds_ft *, struct cds_ft_iter *);
		const char *expect;	/* NULL = expect NOT_FOUND */
	} cases[] = {
		{ "abcd", cds_ft_lookup_lt, "ab" },	/* ab < abcd, leaf predecessor */
		{ "abcd", cds_ft_lookup_le, "ab" },
		{ "abcd", cds_ft_lookup_gt, "m" },
		{ "abcd", cds_ft_lookup_ge, "m" },
		{ "a",    cds_ft_lookup_lt, NULL },	/* nothing < "a" */
	};
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		enum cds_ft_status s;

		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) cases[i].q,
			strlen(cases[i].q));
		s = cases[i].fn(ft, iter);
		if (cases[i].expect == NULL) {
			if (s == CDS_FT_STATUS_OK && cds_ft_iter_node(iter)) {
				fprintf(stderr, "ext('%s'): expected NOT_FOUND\n",
					cases[i].q);
				rcu_read_unlock();
				goto out;
			}
		} else {
			uint8_t rk[64]; size_t rl;

			if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
				fprintf(stderr, "ext('%s'): not found, expected '%s'\n",
					cases[i].q, cases[i].expect);
				rcu_read_unlock();
				goto out;
			}
			cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
			rk[rl] = '\0';
			if (strcmp((char *) rk, cases[i].expect) != 0) {
				fprintf(stderr, "ext('%s'): got '%s', expected '%s'\n",
					cases[i].q, (char *) rk, cases[i].expect);
				rcu_read_unlock();
				goto out;
			}
		}
		rcu_read_unlock();
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
 * Inequality lookups whose descent dead-ends on an EMPTY SLOT more than one
 * byte before the key end ("mmm" between "aaa" and "zzz" dead-ends at the
 * root).  Regression for cds_ft_lookup_le / cds_ft_lookup_lt returning
 * CDS_FT_STATUS_OK with a NULL node: the proper-prefix-leaf early return
 * tested ft_node_external(node_flag), which also matches NULL, instead of
 * falling through to the going-up backtrack (2026-06 review, 1.1).  This also
 * silently terminated cds_ft_prev / reverse iteration early.
 */
static int test_inequality_deadend_empty_slot(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = { "aaa", "zzz" };
	struct dead_case {
		const char *q;
		enum cds_ft_status (*fn)(struct cds_ft *, struct cds_ft_iter *);
		const char *expect;	/* NULL = expect NOT_FOUND */
	} cases[] = {
		{ "mmm", cds_ft_lookup_le, "aaa" },
		{ "mmm", cds_ft_lookup_lt, "aaa" },
		{ "mmm", cds_ft_lookup_ge, "zzz" },
		{ "mmm", cds_ft_lookup_gt, "zzz" },
		/* Dead-end below an existing first byte. */
		{ "amm", cds_ft_lookup_le, "aaa" },
		{ "amm", cds_ft_lookup_ge, "zzz" },
	};
	unsigned int i;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		enum cds_ft_status s;
		uint8_t rk[64]; size_t rl;

		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) cases[i].q,
			strlen(cases[i].q));
		s = cases[i].fn(ft, iter);
		if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
			fprintf(stderr, "deadend('%s'): status %d node %p, expected '%s'\n",
				cases[i].q, s, (void *) cds_ft_iter_node(iter),
				cases[i].expect);
			rcu_read_unlock();
			goto out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
		rk[rl] = '\0';
		if (strcmp((char *) rk, cases[i].expect) != 0) {
			fprintf(stderr, "deadend('%s'): got '%s', expected '%s'\n",
				cases[i].q, (char *) rk, cases[i].expect);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
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
 * Key read back after an ordered-cell-walk landing (key materialized by the
 * parent up-walk at the iter buffer's TAIL, iter->key_off > 0) followed by a
 * SCOPED step (descent path, result key written at the buffer FRONT).
 * Regression for the descent's terminal result stores not resetting
 * iter->key_off: ft_iter_read_key returned the previous position's stale
 * tail bytes for the new node (2026-06 review, 1.3).
 */
static int test_iter_key_off_cell_to_descent(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = { "aaa", "aab", "zzz" };
	unsigned int i;
	int ret = -1;
	uint8_t rk[64]; size_t rl;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	/* Cell-walk landing: key_len LAZY, then up-walk tail-fills the key. */
	s = cds_ft_lookup_first(ft, iter);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_first: status %d\n", s);
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
	if (rl != 3 || memcmp(rk, "aaa", 3) != 0) {
		fprintf(stderr, "first: got len %zu, expected 'aaa'\n", rl);
		rcu_read_unlock();
		goto out;
	}
	/* Scoping disables the cell fast path: next steps via the descent. */
	cds_ft_iter_set_prefix_len(iter, 1);
	s = cds_ft_next(ft, iter);
	if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
		fprintf(stderr, "next(scoped): status %d\n", s);
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
	if (rl != 3 || memcmp(rk, "aab", 3) != 0) {
		rk[rl < sizeof rk ? rl : sizeof rk - 1] = '\0';
		fprintf(stderr, "next(scoped): got '%s' (len %zu), expected 'aab'\n",
			(char *) rk, rl);
		rcu_read_unlock();
		goto out;
	}
	/* "zzz" is outside the 1-byte scope: the walk must end here. */
	s = cds_ft_next(ft, iter);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "next(scoped) past end: status %d, expected NOT_FOUND\n", s);
		rcu_read_unlock();
		goto out;
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
 * cds_ft_iter_copy of ordered-cell-walk positions on an EAGER identity
 * variable-length group (the default config).  Regression for two bugs
 * (2026-06 review, 1.2):
 *  - copying a fresh cell landing, whose key length is the deferred LAZY
 *    sentinel ((size_t)-1), did memcpy(..., SIZE_MAX);
 *  - iter->key_off was never copied, so copying an up-walk-materialized
 *    position (key at the buffer TAIL) copied the wrong byte range and
 *    left dst reading garbage at the front.
 */
static int test_iter_copy_cell_positions(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *src = NULL, *dst = NULL, *dst2 = NULL;
	const char *words[] = { "aaa", "aab", "zzz" };
	unsigned int i;
	int ret = -1;
	uint8_t rk[64]; size_t rl;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &src) < 0 ||
			cds_ft_iter_create(ft, &dst) < 0 ||
			cds_ft_iter_create(ft, &dst2) < 0)
		goto out;
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	/* Fresh cell landing: key_len is the LAZY sentinel, nothing materialized. */
	s = cds_ft_lookup_first(ft, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "lookup_first: status %d\n", s);
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_copy(dst, src);
	cds_ft_iter_get_key(dst, rk, sizeof rk, &rl);
	if (rl != 3 || memcmp(rk, "aaa", 3) != 0) {
		fprintf(stderr, "copy(lazy): got len %zu, expected 'aaa'\n", rl);
		rcu_read_unlock();
		goto out;
	}
	s = cds_ft_next(ft, dst);
	cds_ft_iter_get_key(dst, rk, sizeof rk, &rl);
	if (s != CDS_FT_STATUS_OK || rl != 3 || memcmp(rk, "aab", 3) != 0) {
		fprintf(stderr, "copy(lazy)+next: status %d len %zu, expected 'aab'\n",
			s, rl);
		rcu_read_unlock();
		goto out;
	}
	/* Materialize src's key (up-walk fills the buffer TAIL, key_off > 0). */
	cds_ft_iter_get_key(src, rk, sizeof rk, &rl);
	if (rl != 3 || memcmp(rk, "aaa", 3) != 0) {
		fprintf(stderr, "src materialize: got len %zu, expected 'aaa'\n", rl);
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_copy(dst2, src);
	cds_ft_iter_get_key(dst2, rk, sizeof rk, &rl);
	if (rl != 3 || memcmp(rk, "aaa", 3) != 0) {
		fprintf(stderr, "copy(tail): got len %zu, expected 'aaa'\n", rl);
		rcu_read_unlock();
		goto out;
	}
	s = cds_ft_next(ft, dst2);
	cds_ft_iter_get_key(dst2, rk, sizeof rk, &rl);
	if (s != CDS_FT_STATUS_OK || rl != 3 || memcmp(rk, "aab", 3) != 0) {
		fprintf(stderr, "copy(tail)+next: status %d len %zu, expected 'aab'\n",
			s, rl);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (src)
		cds_ft_iter_destroy(src);
	if (dst)
		cds_ft_iter_destroy(dst);
	if (dst2)
		cds_ft_iter_destroy(dst2);
	if (ret == 0)
		ret = drain_and_destroy(ft, group);
	else
		drain_and_destroy(ft, group);
	return ret;
}

/*
 * Ordered iteration key read-back on an EAGER identity variable-length group
 * WITH an in-leaf key length (key_len_offset set, no speculative_key_offset).
 * Regression for ft_iter_resolve_key_len taking the leaf-length shortcut on
 * such a group: it cleared the LAZY sentinel -- which doubles as "iter key
 * buffer filled" for ft_iter_read_key -- without filling the buffer, so
 * cds_ft_iter_get_key returned the resolved length with garbage key bytes.
 * The shortcut is only valid when the key itself is leaf-referenced
 * (speculative_key_offset set); an EAGER group must take the up-walk, which
 * fills the buffer in the same walk (2026-06 review follow-up to 1.2).
 */
struct klen_test_node {
	struct cds_ft_node node;
	size_t klen;
};

static int test_eager_key_len_offset_iter_key(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter = NULL;
	const char *words[] = { "aaa", "aab", "zzz" };
	struct klen_test_node *nodes[3] = { NULL, NULL, NULL };
	unsigned int i;
	int ret = -1;
	uint8_t rk[64]; size_t rl;
	enum cds_ft_status s;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len_offset(attr,
			offsetof(struct klen_test_node, klen) -
			offsetof(struct klen_test_node, node)) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();

	if (cds_ft_iter_create(ft, &iter) < 0)
		goto out;
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct klen_test_node *n = (struct klen_test_node *)
			calloc(1, sizeof(*n));

		if (!n)
			abort();
		cds_ft_node_init(&n->node);
		n->klen = strlen(words[i]);
		nodes[i] = n;
		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			fprintf(stderr, "insert '%s' failed\n", words[i]);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	s = cds_ft_lookup_first(ft, iter);
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
			fprintf(stderr, "step %u: status %d\n", i, s);
			rcu_read_unlock();
			goto out;
		}
		cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
		if (rl != strlen(words[i]) ||
				memcmp(rk, words[i], rl) != 0) {
			rk[rl < sizeof rk ? rl : sizeof rk - 1] = '\0';
			fprintf(stderr, "step %u: got '%s' (len %zu), expected '%s'\n",
				i, (char *) rk, rl, words[i]);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_next(ft, iter);
	}
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "past end: status %d, expected NOT_FOUND\n", s);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	rcu_read_lock();
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		if (!nodes[i])
			continue;
		if (cds_ft_iter_set_key(iter, (const uint8_t *) words[i],
				strlen(words[i])) == CDS_FT_STATUS_OK &&
				cds_ft_remove(ft, iter, &nodes[i]->node) ==
					CDS_FT_STATUS_OK) {
			/* freed below after a grace period */
		} else if (ret == 0) {
			fprintf(stderr, "cleanup remove '%s' failed\n", words[i]);
			ret = -1;
		}
	}
	rcu_read_unlock();
	rcu_barrier();
	synchronize_rcu();
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++)
		free(nodes[i]);
	if (iter)
		cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Ordered iteration on a NON-IDENTITY key map group with NO in-leaf key
 * (no speculative_key_offset), fixed and variable length.  Regression for
 * the ordinal-cell fast path materializing garbage keys on such a group:
 * ft_ord_cell_iter_land remapped from the unconfigured leaf-key offset (0),
 * copying cds_ft_node header bytes through the key map (2026-06 review,
 * 1.4).  Fixed by serving any key map from the structural up-walk (ordinal
 * bytes, remapped on copy-out).  The reversed byte map makes ordinal order
 * the REVERSE of byte order, so a stale identity assumption also shows as a
 * wrong iteration order.
 */
static int test_nonidentity_ordered_iteration(void)
{
	uint8_t k2o[256], o2k[256];
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter = NULL;
	/* Ordinal (iteration) order under the reversed map: w < m < a. */
	const char *fixed_words_ord[] = { "wxyz", "mmmm", "abcd" };
	const char *var_words_ord[] = { "wxy", "ab" };
	unsigned int i;
	int part, ret = -1;

	for (i = 0; i < 256; i++) {
		k2o[i] = 255 - i;
		o2k[255 - i] = i;
	}
	for (part = 0; part < 2; part++) {
		const char **words = part ? var_words_ord : fixed_words_ord;
		unsigned int nr_words = part ? 2 : 3;
		enum cds_ft_status s;
		uint8_t rk[64]; size_t rl;

		if (cds_ft_group_attr_create(&attr) < 0)
			abort();
		if (cds_ft_group_attr_set_key_len(attr,
				part ? CDS_FT_LEN_VARIABLE : 4) < 0)
			abort();
		s = cds_ft_group_attr_set_key_map(attr, k2o, o2k);
		if (s == CDS_FT_STATUS_NOT_SUPPORTED) {
			/* Built with NO_FEATURE_FT_KEY_MAP -- skip. */
			cds_ft_group_attr_destroy(attr);
			return 0;
		}
		if (s < 0)
			abort();
		if (cds_ft_group_create(attr, &group) < 0)
			abort();
		cds_ft_group_attr_destroy(attr);
		if (cds_ft_create(group, NULL, &ft) < 0)
			abort();
		if (cds_ft_iter_create(ft, &iter) < 0) {
			cds_ft_destroy(ft);
			cds_ft_group_destroy(group);
			return -1;
		}
		for (i = 0; i < nr_words; i++) {
			struct ft_test_node *n = node_alloc(0);

			rcu_read_lock();
			if (cds_ft_insert(ft, (const uint8_t *) words[i],
					  strlen(words[i]), &n->node) < 0) {
				fprintf(stderr, "part %d: insert '%s' failed\n",
					part, words[i]);
				rcu_read_unlock();
				goto out;
			}
			rcu_read_unlock();
		}
		/* Forward walk: ordinal order. */
		rcu_read_lock();
		s = cds_ft_lookup_first(ft, iter);
		for (i = 0; i < nr_words; i++) {
			if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
				fprintf(stderr, "part %d fwd step %u: status %d\n",
					part, i, s);
				rcu_read_unlock();
				goto out;
			}
			cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
			if (rl != strlen(words[i]) ||
					memcmp(rk, words[i], rl) != 0) {
				rk[rl < sizeof rk ? rl : sizeof rk - 1] = '\0';
				fprintf(stderr, "part %d fwd step %u: got '%s' (len %zu), expected '%s'\n",
					part, i, (char *) rk, rl, words[i]);
				rcu_read_unlock();
				goto out;
			}
			s = cds_ft_next(ft, iter);
		}
		if (s != CDS_FT_STATUS_NOT_FOUND) {
			fprintf(stderr, "part %d fwd past end: status %d\n", part, s);
			rcu_read_unlock();
			goto out;
		}
		/* Reverse walk. */
		s = cds_ft_lookup_last(ft, iter);
		for (i = nr_words; i-- > 0; ) {
			if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
				fprintf(stderr, "part %d rev step %u: status %d\n",
					part, i, s);
				rcu_read_unlock();
				goto out;
			}
			cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
			if (rl != strlen(words[i]) ||
					memcmp(rk, words[i], rl) != 0) {
				rk[rl < sizeof rk ? rl : sizeof rk - 1] = '\0';
				fprintf(stderr, "part %d rev step %u: got '%s' (len %zu), expected '%s'\n",
					part, i, (char *) rk, rl, words[i]);
				rcu_read_unlock();
				goto out;
			}
			s = cds_ft_prev(ft, iter);
		}
		if (s != CDS_FT_STATUS_NOT_FOUND) {
			fprintf(stderr, "part %d rev past end: status %d\n", part, s);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		iter = NULL;
		if (drain_and_destroy(ft, group) < 0)
			return -1;
		ft = NULL;
	}
	return 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return ret;
}

/*
 * Inequality lookups of the EMPTY (NIL) key.  Regression for cds_ft_lookup_lt
 * / cds_ft_lookup_le returning the global max instead of NOT_FOUND: the empty
 * key is the global minimum, so nothing is < "" (LT) and the only <= match is
 * the "" key itself (LE).  The going-up backtrack does not run for a
 * zero-length key, so the search wrongly fell through to a max descent.
 */
static int test_inequality_empty_key(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	const char *words[] = { "a", "ab", "abc" };
	unsigned int i;
	int ret = -1;
	uint8_t rk[64]; size_t rl;
	struct ft_test_node *niln;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		if (cds_ft_insert(ft, (const uint8_t *) words[i],
				  strlen(words[i]), &n->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
	}
	rcu_read_lock();
	/* No "" key yet: LT/LE of "" are NOT_FOUND; GT/GE of "" are "a". */
	cds_ft_iter_set_key(iter, (const uint8_t *) "", 0);
	s = cds_ft_lookup_lt(ft, iter);
	if (s == CDS_FT_STATUS_OK && cds_ft_iter_node(iter)) {
		fprintf(stderr, "LT(\"\"): expected NOT_FOUND\n");
		rcu_read_unlock(); goto out;
	}
	cds_ft_iter_set_key(iter, (const uint8_t *) "", 0);
	s = cds_ft_lookup_le(ft, iter);
	if (s == CDS_FT_STATUS_OK && cds_ft_iter_node(iter)) {
		fprintf(stderr, "LE(\"\"): expected NOT_FOUND (no \"\" key)\n");
		rcu_read_unlock(); goto out;
	}
	cds_ft_iter_set_key(iter, (const uint8_t *) "", 0);
	if (cds_ft_lookup_ge(ft, iter) != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
		fprintf(stderr, "GE(\"\"): expected \"a\"\n");
		rcu_read_unlock(); goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof rk, &rl); rk[rl] = '\0';
	if (rl != 1 || rk[0] != 'a') {
		fprintf(stderr, "GE(\"\"): got '%s', expected 'a'\n", (char *) rk);
		rcu_read_unlock(); goto out;
	}
	rcu_read_unlock();
	/* Now insert "" : LE("") returns "", LT("") still NOT_FOUND. */
	niln = node_alloc(0);
	rcu_read_lock();
	if (cds_ft_insert(ft, NULL, 0, &niln->node) < 0) { rcu_read_unlock(); goto out; }
	cds_ft_iter_set_key(iter, (const uint8_t *) "", 0);
	if (cds_ft_lookup_le(ft, iter) != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
		fprintf(stderr, "LE(\"\") with \"\" present: expected the \"\" key\n");
		rcu_read_unlock(); goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
	if (rl != 0) {
		fprintf(stderr, "LE(\"\"): expected zero-length key, got len %zu\n", rl);
		rcu_read_unlock(); goto out;
	}
	cds_ft_iter_set_key(iter, (const uint8_t *) "", 0);
	s = cds_ft_lookup_lt(ft, iter);
	if (s == CDS_FT_STATUS_OK && cds_ft_iter_node(iter)) {
		fprintf(stderr, "LT(\"\") with \"\" present: expected NOT_FOUND\n");
		rcu_read_unlock(); goto out;
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
 * Ordered cell list maintained across a fixed-length root merge.  Regression
 * for ft_merge_ord_interleave seeding the merge-region walk with a zero-length
 * key: cds_ft_iter_set_key rejects a zero-length key on a fixed-length group,
 * so the interleave bailed and the source run was never spliced into the dst
 * cell list -- leaving the merged-in heads out of the list.
 */
static int test_merge_ordered_fixed_root(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	unsigned int i;
	int ret = -1;
	uint8_t k[4];

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_key_len(attr, 4);
	cds_ft_group_attr_set_ordered_list(attr, true);
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &src) < 0) {
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		return -1;
	}
	/* dst in the 0x000000.. range; src in the disjoint 0xff0000.. range. */
	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(i);

		cds_ft_u64_to_key(dst, i, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		s = cds_ft_insert(dst, k, 4, &n->node);
		rcu_read_unlock();
		if (s < 0) goto out;
	}
	for (i = 0; i < 5; i++) {
		uint64_t v = 0xff000000ull | i;
		struct ft_test_node *n = node_alloc(v);

		cds_ft_u64_to_key(src, v, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		s = cds_ft_insert(src, k, 4, &n->node);
		rcu_read_unlock();
		if (s < 0) goto out;
	}
	rcu_read_lock();
	s = cds_ft_merge(dst, NULL, 0, src);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_ordered_fixed_root: merge: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	/* The cell list must hold all 10 merged heads (structural verify). */
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_ordered_fixed_root: verify failed\n");
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Ordered-list correctness across a RE-ROOTED source, GLUE diverge merge_at:
 * exercises ft_merge_graft_subpos_inplace's ordered-cell run capture / unlink /
 * splice.  The OOM unit tests run with the list OFF, so this is the
 * deterministic ordered check.  src@"a" (an EXTERNAL leaf, @shape 0) or src@"x"
 * (a COMPRESSED "ab" run, @shape 1) merges at "mb", which diverges inside dst's
 * compressed "mango"; the moved run must land in dst's cell list in key order
 * ("mango" < "mb..."), with src's list left consistent.
 */
static int merge_rerooted_glue_ordered(int shape)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst = NULL, *src = NULL;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	int ret = -1;
	const char *exp_dst[3];
	unsigned int nexp_dst, i;
	struct ft_test_node *d1 = node_alloc(1);
	struct ft_test_node *s1 = node_alloc(3);
	struct ft_test_node *s2 = node_alloc(4);

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_ordered_list(attr, true);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0 ||
	    cds_ft_create(group, NULL, &src) < 0 ||
	    cds_ft_iter_create(dst, &iter) < 0)
		abort();

	rcu_read_lock();
	if (cds_ft_insert(dst, (const uint8_t *) "mango", 5, &d1->node) < 0) {
		rcu_read_unlock();
		goto out;
	}
	if (shape == 0) {
		if (cds_ft_insert(src, (const uint8_t *) "a", 1, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "b", 1, &s2->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_merge_at(dst, (const uint8_t *) "mb", 2, src,
				(const uint8_t *) "a", 1);
	} else if (shape == 1) {
		if (cds_ft_insert(src, (const uint8_t *) "xabc", 4, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "xabd", 4, &s2->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_merge_at(dst, (const uint8_t *) "mb", 2, src,
				(const uint8_t *) "x", 1);
	} else {
		/* KEY_SHORTER src: "ca" ends inside compressed "cab" over {e,f}. */
		if (cds_ft_insert(src, (const uint8_t *) "cabe", 4, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "cabf", 4, &s2->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_merge_at(dst, (const uint8_t *) "mb", 2, src,
				(const uint8_t *) "ca", 2);
	}
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_glue_ord[%d]: merge: %s\n", shape,
			cds_ft_status_to_string(s));
		goto out;
	}

	if (shape == 0) {
		exp_dst[0] = "mango"; exp_dst[1] = "mb"; nexp_dst = 2;
	} else if (shape == 1) {
		exp_dst[0] = "mango"; exp_dst[1] = "mbabc"; exp_dst[2] = "mbabd";
		nexp_dst = 3;
	} else {
		exp_dst[0] = "mango"; exp_dst[1] = "mbbe"; exp_dst[2] = "mbbf";
		nexp_dst = 3;
	}

	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_glue_ord[%d]: verify failed\n", shape);
		rcu_read_unlock();
		goto out;
	}
	/* Ordered walk over dst must yield exactly exp_dst, in key order. */
	s = cds_ft_lookup_first(dst, iter);
	for (i = 0; i < nexp_dst; i++) {
		uint8_t rk[64];
		size_t rl;

		if (s != CDS_FT_STATUS_OK ||
		    cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) !=
				CDS_FT_STATUS_OK ||
		    rl != strlen(exp_dst[i]) ||
		    memcmp(rk, exp_dst[i], rl) != 0) {
			fprintf(stderr,
				"rerooted_glue_ord[%d]: dst pos %u mismatch (status=%d got '%.*s' want '%s')\n",
				shape, i, (int) s, (int) rl, (const char *) rk,
				exp_dst[i]);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_next(dst, iter);
	}
	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_glue_ord[%d]: dst has extra keys\n",
			shape);
		rcu_read_unlock();
		goto out;
	}
	/* src keeps a consistent, non-empty (shape 0) or empty (shape 1) list. */
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

static int test_merge_rerooted_glue_ordered_ext(void)
{
	return merge_rerooted_glue_ordered(0);
}

static int test_merge_rerooted_glue_ordered_compressed(void)
{
	return merge_rerooted_glue_ordered(1);
}

static int test_merge_rerooted_glue_ordered_key_shorter(void)
{
	return merge_rerooted_glue_ordered(2);
}

/*
 * Node reserve for the NOSPLIT branch-build graft (cds_ft_merge_at into a dst
 * ABSENT at @dst_key -- ft_merge_graft_subpos_inplace).  Attaching the new
 * top-level branch byte to the dst root forces a RANGE recompact of the attach
 * node (a grow even within its child capacity), which the old exact-manifest
 * reserve did not account for -- it underflowed the pre-filled reserve and
 * aborted ("ft alloc reserve underflow").  Deterministic single-threaded repro:
 * a dst whose only branch is {0xFF,0xFF,*} (root holds one high child), merge a
 * 2-key src subtree at the absent high prefix {0xFE,0xFF}.  The merge must
 * commit and the moved keys land as dst's new minimum, in order.
 */
static int test_merge_subpos_branch_reserve(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst = NULL, *src = NULL;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	int ret = -1;
	unsigned int i;
	const uint8_t dk[2] = { 0xFE, 0xFF };	/* absent high dst prefix */
	const uint8_t sk[1] = { 0x53 };		/* "S": the src sub-position */
	const uint8_t want0[3] = { 0xFE, 0xFF, 0x00 };
	const uint8_t want1[3] = { 0xFE, 0xFF, 0x01 };
	uint8_t rk[64];
	size_t rl;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_ordered_list(attr, true);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0 ||
	    cds_ft_create(group, NULL, &src) < 0 ||
	    cds_ft_iter_create(dst, &iter) < 0)
		abort();

	rcu_read_lock();
	for (i = 0; i < 16; i++) {
		uint8_t k[3] = { 0xFF, 0xFF, (uint8_t) i };

		if (cds_ft_insert(dst, k, 3, &node_alloc(0x100 + i)->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
	}
	{
		uint8_t s0[2] = { 0x53, 0x00 }, s1[2] = { 0x53, 0x01 };

		if (cds_ft_insert(src, s0, 2, &node_alloc(1)->node) < 0 ||
		    cds_ft_insert(src, s1, 2, &node_alloc(2)->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
	}
	/* Aborted here before the reserve fix (underflow on the range recompact). */
	s = cds_ft_merge_at(dst, dk, 2, src, sk, 1);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "subpos_branch_reserve: merge: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "subpos_branch_reserve: verify failed\n");
		rcu_read_unlock();
		goto out;
	}
	/* The two moved keys are dst's new minimum {0xFF,0xFE,0/1}, in order. */
	s = cds_ft_lookup_first(dst, iter);
	if (s != CDS_FT_STATUS_OK ||
	    cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) != CDS_FT_STATUS_OK ||
	    rl != 3 || memcmp(rk, want0, 3) != 0) {
		fprintf(stderr, "subpos_branch_reserve: dst min mismatch\n");
		rcu_read_unlock();
		goto out;
	}
	s = cds_ft_next(dst, iter);
	if (s != CDS_FT_STATUS_OK ||
	    cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) != CDS_FT_STATUS_OK ||
	    rl != 3 || memcmp(rk, want1, 3) != 0) {
		fprintf(stderr, "subpos_branch_reserve: dst 2nd mismatch\n");
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Ordered-list correctness across a RE-ROOTED source, NOSPLIT dst merge_at: the
 * companion of the GLUE check above for the in-place graft's other dst shape.
 * @shape 0 grafts an EXTERNAL source at an empty slot of an existing dst node
 * (dst {za,zb}, key "zc" -> exercises the external cell edge-byte refresh on the
 * at-node path); @shape 1 grafts a COMPRESSED source via a built branch that
 * chain-merges the live compressed run (dst {m}, key "mxyz").  The moved run
 * must land in dst's cell list in key order.
 */
static int merge_rerooted_nosplit_ordered(int shape)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst = NULL, *src = NULL;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	int ret = -1;
	const char *exp_dst[3];
	unsigned int nexp_dst, i;
	struct ft_test_node *b1 = node_alloc(1);
	struct ft_test_node *b2 = node_alloc(2);
	struct ft_test_node *s1 = node_alloc(3);
	struct ft_test_node *s2 = node_alloc(4);

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_ordered_list(attr, true);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0 ||
	    cds_ft_create(group, NULL, &src) < 0 ||
	    cds_ft_iter_create(dst, &iter) < 0)
		abort();

	rcu_read_lock();
	if (shape == 0) {
		/* at-node: dst {za,zb}; ext src "a" -> "zc". */
		if (cds_ft_insert(dst, (const uint8_t *) "za", 2, &b1->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *) "zb", 2, &b2->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "a", 1, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "b", 1, &s2->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_merge_at(dst, (const uint8_t *) "zc", 2, src,
				(const uint8_t *) "a", 1);
		exp_dst[0] = "za"; exp_dst[1] = "zb"; exp_dst[2] = "zc";
		nexp_dst = 3;
	} else {
		/* build-branch: dst {m}; compressed src "ab" -> "mxyz{ab}{c,d}". */
		node_free(b2);	/* unused */
		if (cds_ft_insert(dst, (const uint8_t *) "m", 1, &b1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "xabc", 4, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "xabd", 4, &s2->node) < 0) {
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_merge_at(dst, (const uint8_t *) "mxyz", 4, src,
				(const uint8_t *) "x", 1);
		exp_dst[0] = "m"; exp_dst[1] = "mxyzabc"; exp_dst[2] = "mxyzabd";
		nexp_dst = 3;
	}
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_nosplit_ord[%d]: merge: %s\n", shape,
			cds_ft_status_to_string(s));
		goto out;
	}

	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_nosplit_ord[%d]: verify failed\n", shape);
		rcu_read_unlock();
		goto out;
	}
	s = cds_ft_lookup_first(dst, iter);
	for (i = 0; i < nexp_dst; i++) {
		uint8_t rk[64] = { 0 };
		size_t rl = 0;

		if (s != CDS_FT_STATUS_OK ||
		    cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) !=
				CDS_FT_STATUS_OK ||
		    rl != strlen(exp_dst[i]) ||
		    memcmp(rk, exp_dst[i], rl) != 0) {
			fprintf(stderr,
				"rerooted_nosplit_ord[%d]: dst pos %u mismatch (status=%d got '%.*s' want '%s')\n",
				shape, i, (int) s, (int) rl,
				(const char *) rk, exp_dst[i]);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_next(dst, iter);
	}
	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "rerooted_nosplit_ord[%d]: dst has extra keys\n",
			shape);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

static int test_merge_rerooted_nosplit_ordered_atnode(void)
{
	return merge_rerooted_nosplit_ordered(0);
}

static int test_merge_rerooted_nosplit_ordered_branch(void)
{
	return merge_rerooted_nosplit_ordered(1);
}

/* Rekey helper: cds_ft_merge_at(ft, new, ft, old) within one trie. */
static enum cds_ft_status ft_rekey(struct cds_ft *ft, const char *nw,
		const char *old)
{
	return cds_ft_merge_at(ft, (const uint8_t *) nw, strlen(nw),
			ft, (const uint8_t *) old, strlen(old));
}

/*
 * Same-trie "rekey" (src_ft == dst_ft, old_key -> new_key): move a subtree to a
 * new, disjoint key within ONE trie.  Covers a deep shared ancestor that
 * canonicalizes after the move (a/x,y -> the unlink collapses 'a' to compressed
 * "ay"), a root-level divergence, a KEY_SHORTER source (the old key ends inside
 * a compressed run), an external source, an OCCUPIED destination (which MERGES),
 * and the non-overlap guard (a prefix relationship / equal keys are rejected).
 */
static int test_merge_rekey_same_trie(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	int ret = -1;
	enum cds_ft_status s;

	ft = create_varlen_ft(&group);
	rcu_read_lock();

	/* Deep shared ancestor 'a' canonicalizes when "ax" leaves. */
	cds_ft_insert(ft, (const uint8_t *) "axm", 3, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *) "axn", 3, &node_alloc(2)->node);
	cds_ft_insert(ft, (const uint8_t *) "ayp", 3, &node_alloc(3)->node);
	s = ft_rekey(ft, "az", "ax");
	if (s != CDS_FT_STATUS_OK ||
	    cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK ||
	    !ft_test_has_key(ft, "azm") || !ft_test_has_key(ft, "azn") ||
	    !ft_test_has_key(ft, "ayp") ||
	    ft_test_has_key(ft, "axm") || ft_test_has_key(ft, "axn")) {
		fprintf(stderr, "rekey: deep-canon failed (%s)\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	/* KEY_SHORTER source: "he" ends inside compressed "hello". */
	cds_ft_insert(ft, (const uint8_t *) "hello", 5, &node_alloc(4)->node);
	s = ft_rekey(ft, "we", "he");
	if (s != CDS_FT_STATUS_OK ||
	    cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK ||
	    !ft_test_has_key(ft, "wello") || ft_test_has_key(ft, "hello")) {
		fprintf(stderr, "rekey: key-shorter failed (%s)\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	/* External source; occupied destination MERGES. */
	cds_ft_insert(ft, (const uint8_t *) "q", 1, &node_alloc(5)->node);
	cds_ft_insert(ft, (const uint8_t *) "azq", 3, &node_alloc(6)->node);
	s = ft_rekey(ft, "az", "q");	/* "q" -> "az"; "az" already a subtree */
	if (s != CDS_FT_STATUS_OK ||
	    cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK ||
	    !ft_test_has_key(ft, "az") ||	/* the moved "q" lands at "az" */
	    !ft_test_has_key(ft, "azq") ||	/* pre-existing subtree kept */
	    !ft_test_has_key(ft, "azm") || ft_test_has_key(ft, "q")) {
		fprintf(stderr, "rekey: occupied-merge failed (%s)\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	/* Non-overlap guard: prefix relationships + equal keys are rejected. */
	if (ft_rekey(ft, "a", "az") != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR ||
	    ft_rekey(ft, "azz", "az") != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR ||
	    ft_rekey(ft, "az", "az") != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "rekey: non-overlap guard not enforced\n");
		goto out;
	}
	ret = 0;
out:
	rcu_read_unlock();
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Same-trie ORDERED rekey into an OCCUPIED destination: drives the spine-copy
 * merge's ordered-list interleave through the SAME-TRIE pre-reserved flip batch
 * (the combined @pf_flip path -- structural re-parent + folded interleave in one
 * batch and one flip).  "az" is a pre-occupied subtree {azm, azq}; rekeying the
 * "q" subtree {qm, qx, qy} onto "az" merges (occupied dst => spine_copy), with
 * "qm" -> "azm" COLLIDING (demoted to a duplicate, never an ordered head -- the
 * collect runs before apply_splices, so this also checks that invariance).  The
 * ordered walk must surface exactly the distinct merged heads, in key order.
 */
static int test_merge_rekey_same_trie_ordered(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft = NULL;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	const char *exp[] = { "azm", "azq", "azx", "azy" };
	unsigned int i;
	int ret = -1;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_ordered_list(attr, true);
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0 ||
	    cds_ft_iter_create(ft, &iter) < 0)
		abort();

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *) "azm", 3, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *) "azq", 3, &node_alloc(2)->node);
	cds_ft_insert(ft, (const uint8_t *) "qm", 2, &node_alloc(3)->node);
	cds_ft_insert(ft, (const uint8_t *) "qx", 2, &node_alloc(4)->node);
	cds_ft_insert(ft, (const uint8_t *) "qy", 2, &node_alloc(5)->node);
	/* "q" subtree {qm,qx,qy} merges onto the occupied "az" {azm,azq}. */
	s = cds_ft_merge_at(ft, (const uint8_t *) "az", 2,
			ft, (const uint8_t *) "q", 1);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey_ord: merge: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey_ord: verify failed\n");
		rcu_read_unlock();
		goto out;
	}
	/* Ordered walk must yield exactly exp, in key order (collided qm hidden). */
	s = cds_ft_lookup_first(ft, iter);
	for (i = 0; i < 4; i++) {
		uint8_t rk[64];
		size_t rl;

		if (s != CDS_FT_STATUS_OK ||
		    cds_ft_iter_get_key(iter, rk, sizeof rk, &rl) !=
				CDS_FT_STATUS_OK ||
		    rl != strlen(exp[i]) || memcmp(rk, exp[i], rl) != 0) {
			fprintf(stderr,
				"rekey_ord: pos %u mismatch (status=%d got '%.*s' want '%s')\n",
				i, (int) s, (int) rl, (const char *) rk, exp[i]);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_next(ft, iter);
	}
	if (s == CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey_ord: extra keys after merge\n");
		rcu_read_unlock();
		goto out;
	}
	if (!ft_test_has_key(ft, "azm") || !ft_test_has_key(ft, "azq") ||
	    !ft_test_has_key(ft, "azx") || !ft_test_has_key(ft, "azy") ||
	    ft_test_has_key(ft, "qm") || ft_test_has_key(ft, "qx")) {
		fprintf(stderr, "rekey_ord: key membership wrong after merge\n");
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Bulk ops (merge_at spine, graft_swap DELEGATE) on a NON-IDENTITY key map.
 * Regression for the 2026-06 review's finding 2.10: the merge-point descents
 * consumed the caller's application bytes raw while the source unlink
 * remapped them -- on a non-identity map the spine build copied one subtree
 * and the unlink targeted another; and graft_swap's DELEGATE fallback passed
 * its already-remapped key to cds_ft_graft, which remapped it AGAIN (wrong
 * graft point, wrong splice position).  The keys are now converted once at
 * the public entries and threaded in ordinal form.
 */
static int test_nonidentity_bulk_ops(void)
{
	uint8_t k2o[256], o2k[256];
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst = NULL, *src = NULL, *swap = NULL;
	struct cds_ft_iter *iter = NULL;
	const char *dst_keys[] = { "xyz" };
	const char *src_keys[] = { "abc", "abd" };
	const char *swap_keys[] = { "pqr", "pqs" };
	/* dst after merge_at(dst@"xy" <- src@"ab"): xyc xyd xyz. */
	const char *exp_merge[] = { "xyc", "xyd", "xyz" };
	unsigned int i;
	int ret = -1;
	enum cds_ft_status s;

	for (i = 0; i < 256; i++) {
		k2o[i] = 255 - i;
		o2k[255 - i] = i;
	}
	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	s = cds_ft_group_attr_set_key_map(attr, k2o, o2k);
	if (s == CDS_FT_STATUS_NOT_SUPPORTED) {
		/* Built with NO_FEATURE_FT_KEY_MAP -- skip. */
		cds_ft_group_attr_destroy(attr);
		return 0;
	}
	if (s < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0 ||
	    cds_ft_create(group, NULL, &src) < 0 ||
	    cds_ft_create(group, NULL, &swap) < 0 ||
	    cds_ft_iter_create(dst, &iter) < 0)
		abort();
	for (i = 0; i < 1; i++) {
		struct ft_test_node *n = node_alloc(0);

		rcu_read_lock();
		s = cds_ft_insert(dst, (const uint8_t *) dst_keys[i], 3, &n->node);
		rcu_read_unlock();
		if (s < 0) goto out;
	}
	for (i = 0; i < 2; i++) {
		struct ft_test_node *n = node_alloc(0);
		struct ft_test_node *m = node_alloc(0);

		rcu_read_lock();
		s = cds_ft_insert(src, (const uint8_t *) src_keys[i], 3, &n->node);
		if (s == CDS_FT_STATUS_OK)
			s = cds_ft_insert(swap, (const uint8_t *) swap_keys[i],
					3, &m->node);
		rcu_read_unlock();
		if (s < 0) goto out;
	}

	/* merge_at through the spine path (dst non-empty under "xy"). */
	rcu_read_lock();
	s = cds_ft_merge_at(dst, (const uint8_t *) "xy", 2,
			src, (const uint8_t *) "ab", 2);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nonid_bulk: merge_at: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nonid_bulk: post-merge verify failed\n");
		rcu_read_unlock();
		goto out;
	}
	for (i = 0; i < 3; i++) {
		if (!ft_test_has_key(dst, exp_merge[i])) {
			fprintf(stderr, "nonid_bulk: '%s' missing after merge\n",
				exp_merge[i]);
			rcu_read_unlock();
			goto out;
		}
	}
	/* Ordered walk over dst: reversed map => xyz < xyd < xyc?  No --
	 * iteration is in APPLICATION key order per the map; just count. */
	s = cds_ft_lookup_first(dst, iter);
	for (i = 0; s == CDS_FT_STATUS_OK && i < 16; i++)
		s = cds_ft_next(dst, iter);
	if (i != 3) {
		fprintf(stderr, "nonid_bulk: ordered walk found %u keys, expected 3\n", i);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	/* graft_swap DELEGATE: no content at "pq" in dst -> reduces to graft. */
	rcu_read_lock();
	s = cds_ft_graft_swap(dst, (const uint8_t *) "pq", 2, swap);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nonid_bulk: graft_swap: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(swap, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "nonid_bulk: post-swap verify failed\n");
		rcu_read_unlock();
		goto out;
	}
	if (!ft_test_has_key(dst, "pqpqr") ||
	    !ft_test_has_key(dst, "pqpqs")) {
		fprintf(stderr, "nonid_bulk: swapped keys missing\n");
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(dst);
	drain_trie(src);
	drain_trie(swap);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * cds_ft_merge_at combined-length validation (2026-06 review, 2.12): a
 * variable-length merge with dst_key_len > src_key_len must reject moved keys
 * whose resulting length exceeds the group's max_key_len (previously
 * unchecked: the spine's fixed-size key buffers overflowed downstream), and a
 * successful spine merge must raise dst's max_used_key_len (previously left
 * stale, under-feeding later graft validations).
 */
static int test_merge_at_overflow(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *dst = NULL, *src = NULL;
	struct ft_test_node *a = node_alloc(1);
	struct ft_test_node *b = node_alloc(2);
	struct ft_test_node *c = node_alloc(3);
	enum cds_ft_status s;
	int ret = -1;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_max_key_len(attr, 8);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &dst) < 0 ||
	    cds_ft_create(group, NULL, &src) < 0)
		abort();
	rcu_read_lock();
	/* src: "abcde" at prefix "a" -> stripped suffix "bcde" (4 bytes). */
	s = cds_ft_insert(src, (const uint8_t *) "abcde", 5, &a->node);
	/* dst max_used stays 6: the max_used raise below must come from
	 * the merge itself, not from these inserts. */
	if (s == CDS_FT_STATUS_OK)
		s = cds_ft_insert(dst, (const uint8_t *) "zzzzzz", 6, &b->node);
	if (s == CDS_FT_STATUS_OK)
		s = cds_ft_insert(dst, (const uint8_t *) "zzz1", 4, &c->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_overflow: build failed\n");
		goto out;
	}

	/* 6 + (5-1) = 10 > max 8: must be rejected up front. */
	rcu_read_lock();
	s = cds_ft_merge_at(dst, (const uint8_t *) "zzzzzz", 6,
			src, (const uint8_t *) "a", 1);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OVERFLOW_ERROR) {
		fprintf(stderr, "merge_overflow: expected OVERFLOW, got %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!ft_test_has_key(src, "abcde")) {
		fprintf(stderr, "merge_overflow: src key lost on rejection\n");
		goto out;
	}

	/* 3 + (5-1) = 7 <= 8: succeeds and raises dst's max_used_key_len. */
	rcu_read_lock();
	s = cds_ft_merge_at(dst, (const uint8_t *) "zzz", 3,
			src, (const uint8_t *) "a", 1);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK ||
	    !ft_test_has_key(dst, "zzzbcde")) {
		fprintf(stderr, "merge_overflow: valid merge failed (%s)\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (cds_ft_max_used_key_len(dst) < 7) {
		fprintf(stderr,
			"merge_overflow: max_used_key_len %zu, expected >= 7\n",
			cds_ft_max_used_key_len(dst));
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_overflow: verify failed\n");
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Mid-key cds_ft_merge_at on a FIXED-length group: the merged keys must be
 * spliced into dst's ordered cell list.  Regression for the splice-position
 * probes passing the merge-point PREFIX (shorter than the group's fixed key
 * length) to the relational lookups, which reject it: the grafted run got
 * pred == succ == NULL and ft_ord_cell_run_splice emitted zero edges on a
 * non-empty dst list -- merged keys visible to point lookups but permanently
 * invisible to ordered iteration and first/last (2026-06 review, 2.11).  The
 * probes now pad the prefix to the fixed length with the ordinal extremes.
 * Part 0 exercises the detach+graft path (dst empty under the merge point),
 * part 1 the spine-copy path's interleave seed (dst non-empty there).
 */
static int test_merge_at_fixed_ordered_splice(void)
{
	int part, ret = -1;

	for (part = 0; part < 2; part++) {
		struct cds_ft_group_attr *attr;
		struct cds_ft_group *group;
		struct cds_ft *dst = NULL, *src = NULL;
		struct cds_ft_iter *iter = NULL;
		enum cds_ft_status s;
		const char *dst_keys[] = { "aaaaaaaa", "abmmmmmm", "zzzzzzzz" };
		const char *src_keys[] = { "abcdefgh", "abzzzzzz" };
		/* Expected ordered walk: union, sorted. */
		const char *exp_fallback[] =
			{ "aaaaaaaa", "abcdefgh", "abzzzzzz", "zzzzzzzz" };
		const char *exp_spine[] =
			{ "aaaaaaaa", "abcdefgh", "abmmmmmm", "abzzzzzz", "zzzzzzzz" };
		const char **exp = part ? exp_spine : exp_fallback;
		/* part 0: dst empty under "ab" -> detach+graft path. */
		unsigned int nr_dst = part ? 3 : 2;
		unsigned int nr_exp = part ? 5 : 4;
		unsigned int i;

		if (cds_ft_group_attr_create(&attr) < 0)
			return -1;
		cds_ft_group_attr_set_key_len(attr, 8);
		if (cds_ft_group_create(attr, &group) < 0) {
			cds_ft_group_attr_destroy(attr);
			return -1;
		}
		cds_ft_group_attr_destroy(attr);
		if (cds_ft_create(group, NULL, &dst) < 0 ||
		    cds_ft_create(group, NULL, &src) < 0 ||
		    cds_ft_iter_create(dst, &iter) < 0)
			abort();
		for (i = 0; i < nr_dst; i++) {
			const char *k = part ? dst_keys[i] :
				(i == 0 ? dst_keys[0] : dst_keys[2]);
			struct ft_test_node *n = node_alloc(0);

			rcu_read_lock();
			s = cds_ft_insert(dst, (const uint8_t *) k, 8, &n->node);
			rcu_read_unlock();
			if (s < 0) goto out;
		}
		for (i = 0; i < 2; i++) {
			struct ft_test_node *n = node_alloc(0);

			rcu_read_lock();
			s = cds_ft_insert(src, (const uint8_t *) src_keys[i], 8,
					&n->node);
			rcu_read_unlock();
			if (s < 0) goto out;
		}
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "ab", 2,
				src, (const uint8_t *) "ab", 2);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "merge_at_fixed[%d]: merge: %s\n",
				part, cds_ft_status_to_string(s));
			goto out;
		}
		rcu_read_lock();
		if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
		    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "merge_at_fixed[%d]: verify failed\n", part);
			rcu_read_unlock();
			goto out;
		}
		/* The ordered walk must surface every merged key, in order. */
		s = cds_ft_lookup_first(dst, iter);
		for (i = 0; i < nr_exp; i++) {
			uint8_t rk[16]; size_t rl;

			if (s != CDS_FT_STATUS_OK || !cds_ft_iter_node(iter)) {
				fprintf(stderr,
					"merge_at_fixed[%d]: walk ended at %u/%u (status %d)\n",
					part, i, nr_exp, s);
				rcu_read_unlock();
				goto out;
			}
			cds_ft_iter_get_key(iter, rk, sizeof rk, &rl);
			if (rl != 8 || memcmp(rk, exp[i], 8) != 0) {
				rk[rl < sizeof rk ? rl : sizeof rk - 1] = '\0';
				fprintf(stderr,
					"merge_at_fixed[%d]: step %u got '%s', expected '%s'\n",
					part, i, (char *) rk, exp[i]);
				rcu_read_unlock();
				goto out;
			}
			s = cds_ft_next(dst, iter);
		}
		if (s != CDS_FT_STATUS_NOT_FOUND) {
			fprintf(stderr, "merge_at_fixed[%d]: walk past end: %d\n",
				part, s);
			rcu_read_unlock();
			goto out;
		}
		rcu_read_unlock();
		ret = 0;
out:
		if (iter)
			cds_ft_iter_destroy(iter);
		drain_trie(dst);
		drain_trie(src);
		rcu_barrier();
		cds_ft_destroy(src);
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		if (ret < 0)
			return -1;
		ret = -1;	/* re-arm for part 1 */
	}
	return 0;
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
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"helo", 4, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "graft_basic: lookup 'helo': %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"help", 4, 0, &found);
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
 * Reproducer: graft where the descent stops at an external leaf at depth <
 * key_len, with path_len = key_len - depth >= 2 so ft_build_branch may
 * return a compressed node.  With a displaced external, the caller then
 * calls ft_metadata_set_external_nodes on a compressed node, which aborts.
 */
static int test_graft_displaced_external_compressed(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/*
	 * Insert "ab" into live so that at depth 2 we have an external
	 * node.  Descent for a longer "ab..." key will stop at this
	 * external.
	 */
	{
		struct ft_test_node *n1 = node_alloc(0);
		s = cds_ft_insert(live, (const uint8_t *)"ab", 2, &n1->node);
		if (s < 0) goto fail;
	}

	/* Populate staging with a single entry. */
	{
		struct ft_test_node *n2 = node_alloc(0);
		s = cds_ft_insert(staging, (const uint8_t *)"x", 1, &n2->node);
		if (s < 0) goto fail;
	}

	/*
	 * Graft staging into live at "abcd" (key_len=4).  Descent stops
	 * at depth 2 (external "ab"), path_len = 2 — triggers the
	 * compressed-branch path.
	 */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"abcd", 4, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr,
			"graft_displaced_external_compressed: graft: %s\n",
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
 * Reproducer for nr_keys propagation through a compressed parent
 * under SKIP_COMPRESSED:
 *
 * After ft_store_at_graft_point publishes the new branch into a
 * compressed parent's child slot, ft_publish_to_parent updates the
 * grandparent slot via the cn's skip_slot mechanism, encoding
 * cn->len|new_child.  *d.pnfp consequently points at the new branch
 * (with skip_len bits set), not at the old parent cn.
 *
 * ft_propagate_external_count_parent starting from *d.pnfp then
 * walks from @branch (which already has the correct nr_keys set by
 * ft_store_at_graft_point) instead of from @branch.parent.  The
 * extra +delta at @branch over-counts its subtree, surfaced by
 * cds_ft_verify as a nr_keys mismatch.
 *
 * Setup: live trie holds a single key whose path goes through a
 * compressed node ("hello"); staging holds two keys ("y0", "z0") so
 * its old root is multi-child (no chain-compress canonicalization
 * required, isolating the propagation bug).
 *
 * Forces SPECULATIVE so the compressed publish materializes as a
 * SKIP-X encoded slot pointer.
 */
static int test_graft_propagate_through_compressed(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;
	int ret = -1;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(attr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Live: single multi-byte key creates a compressed path. */
	rcu_read_lock();
	s = cds_ft_insert(live, (const uint8_t *)"hello", 5,
			&node_alloc(0)->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) goto out;

	/* Staging: two keys so old root is multi-child (no
	 * chain-compress canonicalization needed for the swap root). */
	s = cds_ft_insert(staging, (const uint8_t *)"y0", 2,
			&node_alloc(1)->node);
	if (s != CDS_FT_STATUS_OK) goto out;
	s = cds_ft_insert(staging, (const uint8_t *)"z0", 2,
			&node_alloc(2)->node);
	if (s != CDS_FT_STATUS_OK) goto out;

	/* Graft staging into live at "helloX" (descent stops at the
	 * existing external "hello", d.pnf is the cn for "ello" with
	 * the displaced external). */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"helloX", 6, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_propagate: graft failed: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	if (cds_ft_verify(live, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_propagate: live verify failed\n");
		goto out;
	}

	ret = 0;
out:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Reproducer for the chain-compress invariant in cds_ft_graft when
 * the descent stops at d.depth < key_len:
 *
 * ft_store_at_graft_point only canonicalizes graft_payload via
 * ft_compress_single_child_if_needed when d.depth == key_len.  The
 * d.depth < key_len branch hands graft_payload to ft_build_branch
 * unchanged; if graft_payload is a 1-child internal at its source
 * root (permitted at root, forbidden at non-root under
 * SKIP_COMPRESSED), the resulting structure has a 1-child internal
 * at a non-root position.
 *
 * Setup: empty live; staging holds "lo" + "lp" so staging.root has
 * exactly one outgoing edge ('l').  Graft at "he" so the descent
 * breaks at d.depth=1 (empty slot) with d.depth < key_len=2.
 *
 * Forces SPECULATIVE so the verify walk's chain-compress check
 * actually fires.
 */
static int test_graft_canonicalize_at_intermediate_depth(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;
	int ret = -1;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(attr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Staging: two keys sharing prefix "l" so staging.root has a
	 * single outgoing edge at 'l' → internal with 'o'/'p'. */
	s = cds_ft_insert(staging, (const uint8_t *)"lo", 2,
			&node_alloc(0)->node);
	if (s != CDS_FT_STATUS_OK) goto out;
	s = cds_ft_insert(staging, (const uint8_t *)"lp", 2,
			&node_alloc(1)->node);
	if (s != CDS_FT_STATUS_OK) goto out;

	/* Graft staging at "he" (descent stops at empty d.nf, depth 1
	 * < key_len 2). */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"he", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_canonicalize: graft failed: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	if (cds_ft_verify(live, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_canonicalize: live verify failed\n");
		goto out;
	}

	ret = 0;
out:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Diverge graft with the ordered list OFF -- exercises the flip-txn fold
 * (ft_glue_txn_commit): the diverge split's live re-parents + forward publish
 * commit as ONE atomic flip instead of the deferred-edge ordering protocol.
 *
 * live holds "hello" (a compressed "ello" path under 'h').  Grafting staging at
 * "help" diverges INSIDE that compressed node (h-e-l match, then the compressed
 * 'l' vs the key 'p') -> FT_GRAFT_PREP_GLUE.  With the list off (and no caller
 * pre_flip) cds_ft_graft drives the GLUE commit through urcu_flip_txn.
 * SPECULATIVE so the verify walk's chain-compress + skip-compressed checks fire.
 *
 * Asserts: the structure verifies, and the pre-existing key plus both grafted
 * keys ("help"+"o", "help"+"p") are reachable afterwards.
 */
static int test_graft_diverge_no_list(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_iter *iter = NULL;
	const char *miss = NULL;
	enum cds_ft_status s;
	int ret = -1;
	size_t i;
	static const char * const want[] = { "hello", "helpo", "helpp" };

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(attr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Live: single multi-byte key -> a compressed "ello" path under 'h'. */
	rcu_read_lock();
	s = cds_ft_insert(live, (const uint8_t *)"hello", 5,
			&node_alloc(0)->node);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) goto out;

	/* Staging: two single-byte keys so the source root is multi-child. */
	s = cds_ft_insert(staging, (const uint8_t *)"o", 1,
			&node_alloc(1)->node);
	if (s != CDS_FT_STATUS_OK) goto out;
	s = cds_ft_insert(staging, (const uint8_t *)"p", 1,
			&node_alloc(2)->node);
	if (s != CDS_FT_STATUS_OK) goto out;

	/* Graft at "help": diverges inside the compressed "ello" -> GLUE. */
	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"help", 4, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_diverge_no_list: graft failed: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	if (cds_ft_verify(live, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "graft_diverge_no_list: live verify failed\n");
		goto out;
	}

	if (cds_ft_iter_create(live, &iter) < 0)
		goto out;
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(want); i++) {
		cds_ft_iter_set_key(iter, (const uint8_t *)want[i],
			strlen(want[i]));
		cds_ft_lookup(live, iter);
		if (!cds_ft_iter_node(iter)) {
			miss = want[i];
			break;
		}
	}
	rcu_read_unlock();
	if (miss) {
		fprintf(stderr, "graft_diverge_no_list: key \"%s\" missing\n",
			miss);
		goto out;
	}

	ret = 0;
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return ret;
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
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"abc", 3, 0, &found);
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
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;

	/* Create a group with max_key_len = 4. */
	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"abZ", 3, 0, &found);
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"abX", 3, 0, &found);
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
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"abcd", 4, 0, &found);
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
/*
 * Regression: graft_swap that extracts a subtree into an EMPTY swap, where the
 * graft point is the SOLE child of a COMPRESSED node, must PRUNE that parent --
 * not leave it childless.  Before the fix, the empty-swap remove published NULL
 * into the compressed parent's child slot, leaving a childless compressed node;
 * the next relational descent (find_splice_pos in a later graft_swap, or any
 * lookup_first with the ordered list on) then dereferenced its NULL child and
 * aborted ("compressed node always has a live child").  Needs a long shared
 * prefix (so the parent is compressed) + the ordered list (so a relational
 * descent runs).  Round-trips extract + graft-back a few times.
 */
static int test_graft_swap_extract_empty_compressed_parent(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = -1, it;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_ordered_list(attr, true) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_create(live, &iter) < 0)
		abort();

	/* Two keys sharing a long prefix -> compressed("ABCDEF") -> {x,y}. */
	{
		struct ft_test_node *a = node_alloc(0), *b = node_alloc(0);

		rcu_read_lock();
		cds_ft_insert(live, (const uint8_t *) "ABCDEFx", 7, &a->node);
		cds_ft_insert(live, (const uint8_t *) "ABCDEFy", 7, &b->node);
		rcu_read_unlock();
	}

	for (it = 0; it < 4; it++) {
		/* Extract the whole "ABCDEF" subtree into the empty swap: the graft
		 * point is the compressed node's sole child. */
		rcu_read_lock();
		s = cds_ft_graft_swap(live, (const uint8_t *) "ABCDEF", 6, swap);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "graft_swap_extract_empty: extract %s\n",
				cds_ft_status_to_string(s));
			goto out;
		}
		/* Relational descent on @live -- aborted before the fix. */
		rcu_read_lock();
		cds_ft_lookup_first(live, iter);
		rcu_read_unlock();
		/* Graft it back. */
		rcu_read_lock();
		s = cds_ft_graft_swap(live, (const uint8_t *) "ABCDEF", 6, swap);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "graft_swap_extract_empty: graft-back %s\n",
				cds_ft_status_to_string(s));
			goto out;
		}
	}
	rcu_read_lock();
	if (cds_ft_count_entries(live) != 2) {
		rcu_read_unlock();
		fprintf(stderr, "graft_swap_extract_empty: live count %lu != 2\n",
			cds_ft_count_entries(live));
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	cds_ft_iter_destroy(iter);
	drain_trie(live);
	drain_trie(swap);
	rcu_barrier();
	cds_ft_destroy(live);
	cds_ft_destroy(swap);
	cds_ft_group_destroy(group);
	return ret;
}

static int test_graft_swap_at_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long live_count, swap_count;

	live = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"xx", 2, 0, &found);
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
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"abX", 3, 0, &found);
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
	s = cds_ft_eager_lookup_key(detached, (const uint8_t *)"X", 1, 0, &found);
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
	s = cds_ft_eager_lookup_key(detached, (const uint8_t *)"Y", 1, 0, &found);
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

		s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"foo", 3, 0, &found);
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
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"abX", 3, 0, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "detach_then_graft: 'abX' still present\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"zzX", 3, 0, &found);
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
	struct cds_ft_group_attr *attr;
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
	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	s = cds_ft_eager_lookup_key(live, k, klen, 0, &found);
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	s = cds_ft_eager_lookup_key(live, k, klen, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "fixed_graft_at_root: lookup 10: %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto fail;
	}
	cds_ft_u64_to_key(live, 30, k, klen);
	s = cds_ft_eager_lookup_key(live, k, klen, 0, &found);
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *staging;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	s = cds_ft_eager_lookup_key(live, k, klen, 0, &found);
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
	s = cds_ft_eager_lookup_key(live, k, klen, 0, &found);
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
	s = cds_ft_eager_lookup_key(swap, k, klen, 0, &found);
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	unsigned long count;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &ft) < 0) {
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
	s = cds_ft_eager_lookup_key(detached, k, klen, 0, &found);
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
	s = cds_ft_eager_lookup_key(detached, k, klen, 0, &found);
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *detached = NULL;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &ft) < 0) {
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
	if (cds_ft_create(group, NULL, &ft) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	enum cds_ft_status s;

	/* max_key_len = 4. */
	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct ft_test_node *n;
	enum cds_ft_status s;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &ft) < 0) {
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
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft, *other, *detached = NULL;
	enum cds_ft_status s;
	uint8_t k[4];
	const size_t klen = 4;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);

	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &other) < 0) {
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
	if (cds_ft_create(group, NULL, &empty_src) < 0) {
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
	if (cds_ft_create(group, NULL, &staging) < 0) {
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
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"abY", 3, 0, &found);
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
	if (cds_ft_create(group, NULL, &ft2) < 0) {
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
	s = cds_ft_eager_lookup_key(ft1, (const uint8_t *)"BBB", 3, 0, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "multiple_tries: 'BBB' found in ft1\n");
		rcu_read_unlock();
		goto fail;
	}
	s = cds_ft_eager_lookup_key(ft2, (const uint8_t *)"AAA", 3, 0, &found);
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
	s = cds_ft_eager_lookup_key(ft, NULL, 0, 0, &found);
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

		s = cds_ft_eager_lookup_key(ft, NULL, 0, 0, &found);
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

	cds_ft_show(ft, devnull, CDS_FT_SHOW_PRETTY);
	cds_ft_show(ft, devnull, CDS_FT_SHOW_JSON);
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
 * (popcount-bitmap, pigeon) by constructing key populations that
 * force transitions through every node type, exercise hysteresis
 * boundaries, and verify correctness under pathological key
 * distributions.
 *
 * The node configuration thresholds on 64-bit are:
 *   Type 0 popcount_2l:  1-3 children    (32 B)
 *   Type 1 popcount_2l:  3-6 children    (64 B)
 *   Type 2 popcount_2l:  5-14 children   (128 B)
 *   Type 3 popcount_1l:  10-28 children  (256 B)
 *   Type 4 popcount_1l:  22-60 children  (512 B)
 *   Type 5 popcount_1l:  51-124 children (1024 B)
 *   Type 6 PIGEON:       95-256 children (2048 B)
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
verify_varlen_order(struct cds_ft_group *group, struct cds_ft *ft,
		    struct cds_ft_iter *iter, unsigned int expected_count)
{
	size_t max_klen = cds_ft_group_max_key_len(group);
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
 * Forces a single internal node through every configuration: popcount_2l
 * (types 0..2), popcount_1l (types 3..5), and pigeon (type 6).  Verifies
 * count and sorted iteration.
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
 * Repeatedly insert and remove keys near a popcount_2l/popcount_1l
 * tier transition.  Insert 30 children (above type 3 max on 64-bit),
 * remove 10 (below type 4 min), re-insert 10. Repeat 3 cycles.
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
			cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
 * pigeon-to-popcount_2l shrink path.
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
		cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
		cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
		s = cds_ft_eager_lookup_key(ft, zero_key, i, 0, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "boundary: lookup zero len %u failed\n", i);
			rcu_read_unlock();
			goto out;
		}
		s = cds_ft_eager_lookup_key(ft, ff_key, i, 0, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "boundary: lookup ff len %u failed\n", i);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();

	if (verify_varlen_order(group, ft, iter, nr_keys) < 0)
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

		s = cds_ft_eager_lookup_key(ft, key, i, 0, &found);
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

	if (verify_varlen_order(group, ft, iter, depth) < 0)
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
	s = cds_ft_eager_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, 0, &head);
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
	cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
 * cds_ft_group_max_key_len().
 */
static int test_adversarial_max_depth(void)
{
	struct cds_ft_group *probe_group, *group;
	struct cds_ft_group_attr *attr;
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
	if (cds_ft_create(probe_group, NULL, &probe_ft) < 0) {
		cds_ft_group_destroy(probe_group);
		return -1;
	}
	max_klen = cds_ft_group_max_key_len(probe_group);
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

	if (cds_ft_group_attr_create(&attr) < 0)
		goto out_free_keys;
	if (cds_ft_group_attr_set_key_len(attr, max_klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		goto out_free_keys;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		goto out_free_keys;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0) {
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

	s = cds_ft_eager_lookup_key(ft, key_zero, max_klen, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->value != 1) {
		rcu_read_unlock(); goto out;
	}
	s = cds_ft_eager_lookup_key(ft, key_ff, max_klen, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found || to_test_node(found)->value != 2) {
		rcu_read_unlock(); goto out;
	}
	s = cds_ft_eager_lookup_key(ft, key_alt, max_klen, 0, &found);
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
		cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
	cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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
	cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
	s = cds_ft_lookup_ge(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0xF0) {
		rcu_read_unlock();
		goto fail;
	}

	/* gt(0x0F) -> 0xF0 */
	cds_ft_u64_to_key(ft, 0x0F, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
	s = cds_ft_lookup_gt(ft, iter);
	if (s != CDS_FT_STATUS_OK) { rcu_read_unlock(); goto fail; }
	found = cds_ft_iter_node(iter);
	if (!found || to_test_node(found)->key != 0xF0) {
		rcu_read_unlock();
		goto fail;
	}

	/* lt(0xF0) -> 0x0F */
	cds_ft_u64_to_key(ft, 0xF0, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
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

	if (verify_varlen_order(group, ft, iter, 256) < 0) goto out;

	rcu_read_lock();
	{
		uint8_t key[4] = { 0x42, 0xFF, 0xDE, 0xAD };
		struct cds_ft_node *found;
		enum cds_ft_status s;

		s = cds_ft_eager_lookup_key(ft, key, 4, 0, &found);
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

		s = cds_ft_eager_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, 0, &found);
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
static int test_iter_cache_mode_default(void)
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

	if (cds_ft_iter_get_cache_mode(iter) != CDS_FT_ITER_CACHED) {
		fprintf(stderr, "cache_mode_default: expected CACHED\n");
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	s = cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "cache_mode_default: set UNCACHED: %s\n",
			cds_ft_status_to_string(s));
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_iter_get_cache_mode(iter) != CDS_FT_ITER_UNCACHED) {
		fprintf(stderr, "cache_mode_default: get after set UNCACHED\n");
		cds_ft_iter_destroy(iter);
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Invalid mode value. */
	s = cds_ft_iter_set_cache_mode(iter, (enum cds_ft_iter_cache_mode) 99);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "cache_mode_default: expected error for mode 99, got %s\n",
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

	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);

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

	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);

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
 * unsafe in CACHED mode without cds_ft_iter_bind_key().
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

	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);

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

	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);
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
static int test_iter_cache_mode_switch(void)
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
	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);
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
	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_CACHED);
	if (cds_ft_iter_get_cache_mode(iter) != CDS_FT_ITER_CACHED) {
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
	cds_ft_iter_set_cache_mode(iter_a, CDS_FT_ITER_UNCACHED);

	rcu_read_lock();
	cds_ft_lookup_first(ft, iter_a);
	rcu_read_unlock();

	/* Copy iter_a → iter_b. iter_b should inherit UNCACHED mode. */
	cds_ft_iter_copy(iter_b, iter_a);

	if (cds_ft_iter_get_cache_mode(iter_b) != CDS_FT_ITER_UNCACHED) {
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
 * (popcount_2l, popcount_1l, pigeon) under the uncached path.
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

	cds_ft_iter_set_cache_mode(iter, CDS_FT_ITER_UNCACHED);

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
	if (cds_ft_eager_lookup_key(ft, k1, 3, 0, &found) != CDS_FT_STATUS_OK || !found ||
	    cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
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
	if (cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
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

	if (cds_ft_create(group, NULL, &src) < 0) {
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
	if (cds_ft_eager_lookup_key(ft, k1, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_graft_diverge: lookup original failed\n");
		rcu_read_unlock();
		cds_ft_destroy(src);
		return drain_and_destroy(ft, group) | -1;
	}
	/* Grafted key "abxQR" should exist. */
	{
		const uint8_t *k_full = (const uint8_t *)"abxQR";

		if (cds_ft_eager_lookup_key(ft, k_full, 5, 0, &found) != CDS_FT_STATUS_OK || !found) {
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
	if (cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_detach_through: k2 missing after detach\n");
		rcu_read_unlock();
		goto fail_detach;
	}
	if (cds_ft_eager_lookup_key(ft, k1, 6, 0, &found) == CDS_FT_STATUS_OK) {
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
/*
 * Helper: single inequality lookup with a fresh iterator to avoid
 * cached-path interactions between tests.
 */
static int check_ineq(struct cds_ft *ft, const char *label,
		const uint8_t *key, size_t key_len,
		enum cds_ft_status (*fn)(struct cds_ft *, struct cds_ft_iter *),
		const uint8_t *expect, size_t expect_len)
{
	struct cds_ft_iter *iter;
	uint8_t rk[32];
	size_t rklen;

	if (cds_ft_iter_create(ft, &iter) < 0)
		return -1;
	cds_ft_iter_set_key(iter, key, key_len);
	if (fn(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_ineq: %s: NOT_FOUND\n", label);
		cds_ft_iter_destroy(iter);
		return -1;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	cds_ft_iter_destroy(iter);
	if (rklen != expect_len || memcmp(rk, expect, expect_len) != 0) {
		fprintf(stderr, "compress_ineq: %s: got len=%zu key=%.*s, "
			"expected len=%zu key=%.*s\n", label,
			rklen, (int)rklen, rk,
			expect_len, (int)expect_len, expect);
		return -1;
	}
	return 0;
}

static int test_compress_inequality_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"apple", 5, &node_alloc(0)->node);
	cds_ft_insert(ft, (const uint8_t *)"banana", 6, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *)"cherry", 6, &node_alloc(2)->node);

	/* GE("band") → "cherry" (band > banana). */
	if (check_ineq(ft, "ge(band)",
		    (const uint8_t *)"band", 4, cds_ft_lookup_ge,
		    (const uint8_t *)"cherry", 6))
		goto fail;

	/* LE("band") → "banana" (banana < band). */
	if (check_ineq(ft, "le(band)",
		    (const uint8_t *)"band", 4, cds_ft_lookup_le,
		    (const uint8_t *)"banana", 6))
		goto fail;

	/* GE("banan") → "banana" (banan < banana). */
	if (check_ineq(ft, "ge(banan)",
		    (const uint8_t *)"banan", 5, cds_ft_lookup_ge,
		    (const uint8_t *)"banana", 6))
		goto fail;

	/* GE("banana") exact match. */
	if (check_ineq(ft, "ge(banana)",
		    (const uint8_t *)"banana", 6, cds_ft_lookup_ge,
		    (const uint8_t *)"banana", 6))
		goto fail;

	/* LE("banana") exact match. */
	if (check_ineq(ft, "le(banana)",
		    (const uint8_t *)"banana", 6, cds_ft_lookup_le,
		    (const uint8_t *)"banana", 6))
		goto fail;

	rcu_read_unlock();
	return drain_and_destroy(ft, group);
fail:
	rcu_read_unlock();
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

	if (cds_ft_eager_lookup_key(ft, k1, 6, 0, &found) != CDS_FT_STATUS_OK || !found ||
	    cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
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
	if (cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_nested: k2 missing after k1 remove\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}

/*
 * Replace through compressed paths: insert two keys that create
 * compressed nodes, then replace a node via cds_ft_replace.
 * Exercises the replace descent through multiple compressed nodes.
 */
static int test_compress_replace_through(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	struct ft_test_node *n1 = node_alloc(1), *n2 = node_alloc(2);
	struct ft_test_node *repl = node_alloc(99);
	struct cds_ft_node *found;
	const uint8_t *k1 = (const uint8_t *)"abcdef";
	const uint8_t *k2 = (const uint8_t *)"abcxyz";

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	cds_ft_insert(ft, k1, 6, &n1->node);
	cds_ft_insert(ft, k2, 6, &n2->node);

	/* Replace n2 at "abcxyz" through compressed paths. */
	cds_ft_iter_set_key(iter, k2, 6);
	if (cds_ft_lookup(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_replace: lookup failed\n");
		rcu_read_unlock();
		goto fail;
	}
	if (cds_ft_replace(ft, iter, &n2->node, &repl->node) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_replace: replace failed\n");
		rcu_read_unlock();
		goto fail;
	}
	node_free_rcu(n2);
	/* Verify replacement. */
	if (cds_ft_eager_lookup_key(ft, k2, 6, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &repl->node) {
		fprintf(stderr, "compress_replace: wrong node after replace\n");
		rcu_read_unlock();
		goto fail;
	}
	/* Other key should be unaffected. */
	if (cds_ft_eager_lookup_key(ft, k1, 6, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &n1->node) {
		fprintf(stderr, "compress_replace: other key damaged\n");
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
 * External_nodes surviving recompaction: insert a long key (creates
 * compressed path), then a short prefix key (stored as
 * external_nodes after decompression), then a diverging key that
 * triggers recompaction of the node holding external_nodes.
 * Verifies external_nodes are preserved across recompaction.
 */
static int test_compress_recompact_external_nodes(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct ft_test_node *n1 = node_alloc(1);
	struct ft_test_node *n2 = node_alloc(2);
	struct ft_test_node *n3 = node_alloc(3);
	struct cds_ft_node *found;

	rcu_read_lock();
	/* Long key creates compressed path. */
	cds_ft_insert(ft, (const uint8_t *)"abcdef", 6, &n1->node);
	/* Short prefix key: triggers decompression, stored as external_nodes. */
	cds_ft_insert(ft, (const uint8_t *)"abc", 3, &n2->node);

	if (cds_ft_eager_lookup_key(ft, (const uint8_t *)"abc", 3, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &n2->node) {
		fprintf(stderr, "compress_recompact_ext: abc missing before diverge\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}

	/* Diverging key: adds second child to the node holding
	 * external_nodes, potentially triggering recompaction. */
	cds_ft_insert(ft, (const uint8_t *)"abcxyz", 6, &n3->node);

	if (cds_ft_count_keys(ft) != 3) {
		fprintf(stderr, "compress_recompact_ext: count != 3\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	/* Verify all three keys survive. */
	if (cds_ft_eager_lookup_key(ft, (const uint8_t *)"abc", 3, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &n2->node) {
		fprintf(stderr, "compress_recompact_ext: abc lost after diverge\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	if (cds_ft_eager_lookup_key(ft, (const uint8_t *)"abcdef", 6, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &n1->node) {
		fprintf(stderr, "compress_recompact_ext: abcdef lost\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	if (cds_ft_eager_lookup_key(ft, (const uint8_t *)"abcxyz", 6, 0, &found) != CDS_FT_STATUS_OK ||
	    found != &n3->node) {
		fprintf(stderr, "compress_recompact_ext: abcxyz lost\n");
		rcu_read_unlock();
		return drain_and_destroy(ft, group) | -1;
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);
}

/*
 * Cached iterator reuse through compressed paths: exercise the
 * inequality fast-path with compressed entries in iter_path.
 *
 * The fast-path reads iter_path[key_depth - 1] from a prior
 * lookup's cached path.  When that entry is a compressed node,
 * the fast-path must fall back to the slow path.  This test
 * verifies correctness across several reuse patterns:
 *
 * 1. GE → LE with same key (mode switch)
 * 2. GE → GE with shorter prefix (subset optimization)
 * 3. Exact lookup → GE with shorter key
 * 4. GE → cds_ft_next (iteration from inequality result)
 * 5. Repeated identical lookups (idempotent reuse)
 *
 * All using a SINGLE shared iterator to exercise path caching.
 */
static int test_compress_iter_reuse(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft_iter *iter;
	uint8_t rk[32];
	size_t rklen;
	int ret = -1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"apple", 5, &node_alloc(0)->node);
	cds_ft_insert(ft, (const uint8_t *)"banana", 6, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *)"cherry", 6, &node_alloc(2)->node);

	/* 1. GE("banana") → LE("banana") with same iterator. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"banana", 6);
	if (cds_ft_lookup_ge(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_reuse: ge(banana) failed\n");
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "iter_reuse: ge(banana) = %.*s\n", (int)rklen, rk);
		goto out;
	}
	/* LE with same key reuses cached path. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"banana", 6);
	if (cds_ft_lookup_le(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_reuse: le(banana) after ge failed\n");
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "iter_reuse: le(banana) = %.*s\n", (int)rklen, rk);
		goto out;
	}

	/* 2. GE("banana") → GE("banan") — shorter prefix, subset. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"banana", 6);
	cds_ft_lookup_ge(ft, iter);
	cds_ft_iter_set_key(iter, (const uint8_t *)"banan", 5);
	if (cds_ft_lookup_ge(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_reuse: ge(banan) after ge(banana) failed\n");
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "iter_reuse: ge(banan) = %.*s (expect banana)\n",
			(int)rklen, rk);
		goto out;
	}

	/* 3. Exact lookup → GE with shorter key. */
	cds_ft_iter_set_key(iter, (const uint8_t *)"banana", 6);
	cds_ft_lookup(ft, iter);
	cds_ft_iter_set_key(iter, (const uint8_t *)"ban", 3);
	if (cds_ft_lookup_ge(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_reuse: ge(ban) after lookup(banana) failed\n");
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "banana", 6) != 0) {
		fprintf(stderr, "iter_reuse: ge(ban) = %.*s (expect banana)\n",
			(int)rklen, rk);
		goto out;
	}

	/* 4. GE("banana") → cds_ft_next (iterate to next key). */
	cds_ft_iter_set_key(iter, (const uint8_t *)"banana", 6);
	cds_ft_lookup_ge(ft, iter);
	if (cds_ft_next(ft, iter) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "iter_reuse: next after ge(banana) failed\n");
		goto out;
	}
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "cherry", 6) != 0) {
		fprintf(stderr, "iter_reuse: next = %.*s (expect cherry)\n",
			(int)rklen, rk);
		goto out;
	}

	/* 5. Repeated identical GE (idempotent). */
	cds_ft_iter_set_key(iter, (const uint8_t *)"band", 4);
	cds_ft_lookup_ge(ft, iter);
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "cherry", 6) != 0) {
		fprintf(stderr, "iter_reuse: ge(band) first = %.*s\n",
			(int)rklen, rk);
		goto out;
	}
	cds_ft_iter_set_key(iter, (const uint8_t *)"band", 4);
	cds_ft_lookup_ge(ft, iter);
	cds_ft_iter_get_key(iter, rk, sizeof(rk), &rklen);
	if (rklen != 6 || memcmp(rk, "cherry", 6) != 0) {
		fprintf(stderr, "iter_reuse: ge(band) second = %.*s\n",
			(int)rklen, rk);
		goto out;
	}

	ret = 0;
out:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group) | ret;
}

/*
 * Graft_swap with key shorter than compressed path: insert "abcdef"
 * (creates compressed path), then graft_swap at "ab" which is
 * shorter than the compressed path.  Verifies the compressed path
 * is split into prefix + suffix, the suffix is swapped into the
 * source trie, and the original key is accessible in the swap trie
 * (exercises compressed root lookup in the swapped trie).
 */
static int test_compress_graft_swap_key_shorter(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	struct cds_ft *swap;
	struct cds_ft_node *found;
	const uint8_t *k_src = (const uint8_t *)"QR";

	if (cds_ft_create(group, NULL, &swap) < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *)"abcdef", 6, &node_alloc(1)->node);
	cds_ft_insert(swap, k_src, 2, &node_alloc(2)->node);

	/* Graft_swap at "ab" — shorter than compressed path "bcdef". */
	if (cds_ft_graft_swap(ft, (const uint8_t *)"ab", 2, swap) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compress_graft_swap_shorter: swap failed\n");
		rcu_read_unlock();
		goto fail;
	}
	/* "abQR" should exist in ft (from swap source). */
	if (cds_ft_eager_lookup_key(ft, (const uint8_t *)"abQR", 4, 0, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_graft_swap_shorter: abQR not found\n");
		rcu_read_unlock();
		goto fail;
	}
	/* "cdef" should exist in swap trie (relative key). */
	if (cds_ft_eager_lookup_key(swap, (const uint8_t *)"cdef", 4, 0, &found) != CDS_FT_STATUS_OK || !found) {
		fprintf(stderr, "compress_graft_swap_shorter: cdef not in swap\n");
		rcu_read_unlock();
		goto fail;
	}
	if (cds_ft_count_keys(ft) != 1 || cds_ft_count_keys(swap) != 1) {
		fprintf(stderr, "compress_graft_swap_shorter: counts wrong ft=%lu swap=%lu\n",
			cds_ft_count_keys(ft), cds_ft_count_keys(swap));
		rcu_read_unlock();
		goto fail;
	}
	rcu_read_unlock();
	{
		int r = drain_and_destroy(swap, group);
		return drain_and_destroy(ft, group) | r;
	}
fail:
	rcu_read_unlock();
	{
		int r = drain_and_destroy(swap, group);
		return drain_and_destroy(ft, group) | r | -1;
	}
}

/* ================================================================== */
/*                                                                    */
/*               14. SKIP-COMPRESSED UNIT TESTS                       */
/*                                                                    */
/* ================================================================== */

/* Create a variable-length trie with skip-compressed pointer encoding. */
static struct cds_ft *create_skip_compressed_ft(struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_lookup_optimization(attr, CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Unit test for skip-compressed mode.
 *
 * Exercises: insert (with splits), exact lookup, candidate lookup,
 * inequality lookup, remove, iteration, and count — all through
 * skip-compressed paths.
 */
static int test_skip_compressed_unit(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_skip_compressed_ft(&group);
	struct cds_ft_node *found;
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = -1;

	static const char *keys[] = {
		"hello", "help", "world", "wonder",
		"abcdefghij", "abcdefghik",
	};
	enum { NKEYS = sizeof(keys) / sizeof(keys[0]) };
	struct ft_test_node *nodes[NKEYS];
	unsigned int i;

	for (i = 0; i < NKEYS; i++)
		nodes[i] = node_alloc(i);

	/* Insert all keys. */
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_insert(ft, (const uint8_t *)keys[i],
			strlen(keys[i]), &nodes[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "skip_compressed_unit: insert '%s': %s\n",
				keys[i], cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Exact lookup. */
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_eager_lookup_key(ft, (const uint8_t *)keys[i],
			strlen(keys[i]), 0, &found);
		if (s != CDS_FT_STATUS_OK || found != &nodes[i]->node) {
			fprintf(stderr, "skip_compressed_unit: exact lookup '%s' failed\n",
				keys[i]);
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Candidate lookup. */
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_lookup_candidate_key(ft, (const uint8_t *)keys[i],
			strlen(keys[i]), 0, &found);
		if (s != CDS_FT_STATUS_OK || found != &nodes[i]->node) {
			fprintf(stderr, "skip_compressed_unit: candidate lookup '%s' failed\n",
				keys[i]);
			rcu_read_unlock();
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Non-existent key: exact lookup should return NOT_FOUND. */
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *)"helloX", 6, 0, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "skip_compressed_unit: non-existent key returned %s\n",
			cds_ft_status_to_string(s));
		goto fail;
	}

	/* Inequality lookup (next >= "help"). */
	{
		s = cds_ft_iter_create(ft, &iter);
		if (s < 0) goto fail;
		cds_ft_iter_set_key(iter, (const uint8_t *)"help", 4);
		rcu_read_lock();
		s = cds_ft_lookup_ge(ft, iter);
		found = cds_ft_iter_node(iter);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK || found != &nodes[1]->node) {
			fprintf(stderr, "skip_compressed_unit: lookup_ge 'help' failed: %s\n",
				cds_ft_status_to_string(s));
			cds_ft_iter_destroy(iter);
			goto fail;
		}
		cds_ft_iter_destroy(iter);
	}

	/* Count entries. */
	{
		unsigned long count;

		rcu_read_lock();
		count = cds_ft_count_entries(ft);
		rcu_read_unlock();
		if (count != NKEYS) {
			fprintf(stderr, "skip_compressed_unit: count %lu != %u\n",
				count, NKEYS);
			goto fail;
		}
	}

	/* Forward iteration: should visit all keys. */
	{
		unsigned int count = 0;

		s = cds_ft_iter_create(ft, &iter);
		if (s < 0) goto fail;
		rcu_read_lock();
		while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
			s = cds_ft_remove(ft, iter, cds_ft_iter_node(iter));
			if (s != CDS_FT_STATUS_OK) {
				fprintf(stderr, "skip_compressed_unit: remove during drain: %s\n",
					cds_ft_status_to_string(s));
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				goto fail;
			}
			count++;
			rcu_read_unlock();
			rcu_quiescent_state();
			rcu_read_lock();
		}
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		if (count != NKEYS) {
			fprintf(stderr, "skip_compressed_unit: drain count %u != %u\n",
				count, NKEYS);
			goto fail;
		}
	}

	ret = 0;
fail:
	/* Nodes were removed from the trie; free after grace period. */
	rcu_barrier();
	for (i = 0; i < NKEYS; i++)
		node_free(nodes[i]);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*       14b. SPECULATIVE-VALIDATED LOOKUP UNIT TESTS                 */
/*                                                                    */
/* ================================================================== */

/*
 * Speculative-validated lookup runs cds_ft_eager_lookup_key in cand-mode
 * descent and validates the candidate leaf against the user-stored
 * key bytes via the library's inline comparator.  The struct layout
 * below positions the key (and optional key_len) at known offsets
 * from the embedded cds_ft_node so the library can locate them from
 * a stored leaf pointer.
 *
 * Regression: the descent loop advances the local @key cursor, so
 * the post-loop validation must compare the original caller-supplied
 * key against the stored key.  A bug that compared the post-descent
 * cursor caused valid hits to return NOT_FOUND.
 */

struct ft_specv_node {
	struct cds_ft_node node;
	struct rcu_head head;
	size_t key_len;
	uint8_t key[64];
};

#define SPECV_KEY_OFFSET \
	(offsetof(struct ft_specv_node, key) - \
	 offsetof(struct ft_specv_node, node))
#define SPECV_KEY_LEN_OFFSET \
	(offsetof(struct ft_specv_node, key_len) - \
	 offsetof(struct ft_specv_node, node))

static struct ft_specv_node *specv_node_alloc(const uint8_t *key, size_t klen)
{
	struct ft_specv_node *n =
		(struct ft_specv_node *) calloc(1, sizeof(*n));
	if (!n)
		abort();
	cds_ft_node_init(&n->node);
	if (klen > sizeof(n->key))
		abort();
	memcpy(n->key, key, klen);
	n->key_len = klen;
	__atomic_add_fetch(&nodes_allocated, 1, __ATOMIC_RELAXED);
	return n;
}

static void specv_node_free(struct ft_specv_node *n)
{
	memset(n, 0xfe, sizeof(*n));
	free(n);
	__atomic_add_fetch(&nodes_freed, 1, __ATOMIC_RELAXED);
}

/*
 * Drain the trie by removing each entry via lookup_first iter and
 * freeing the ft_specv_node behind a grace period.
 */
static int specv_drain_and_destroy(struct cds_ft *ft, struct cds_ft_group *group)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = 0;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head;
		struct cds_ft_node *node = cds_ft_iter_node(iter);
		struct ft_specv_node *sn =
			caa_container_of(node, struct ft_specv_node, node);

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			ret = -1;
			break;
		}
		(void) head;
		rcu_read_unlock();
		rcu_barrier();
		specv_node_free(sn);
		rcu_read_lock();
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Build a variable-length speculative-validated trie.  Returns NULL
 * on architectures where speculative_validated isn't supported (the
 * caller should skip the test in that case).
 */
static struct cds_ft *create_specv_varlen_ft(struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	enum cds_ft_status s;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	s = cds_ft_group_attr_set_lookup_optimization(attr, CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (s != CDS_FT_STATUS_OK) {
		cds_ft_group_attr_destroy(attr);
		return NULL;
	}
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Build a fixed-length speculative-validated trie.  Returns NULL on
 * architectures where speculative_validated isn't supported.
 */
static struct cds_ft *create_specv_fixed_ft(size_t klen,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	enum cds_ft_status s;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0) {
		cds_ft_group_attr_destroy(attr);
		abort();
	}
	s = cds_ft_group_attr_set_lookup_optimization(attr, CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (s != CDS_FT_STATUS_OK) {
		cds_ft_group_attr_destroy(attr);
		return NULL;
	}
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Insert a handful of fixed-length keys, look each one up via
 * cds_ft_eager_lookup_key, expect every hit to validate.  Without the
 * orig_key fix the validation comparator runs on garbage bytes and
 * every hit is rejected as NOT_FOUND.
 */
static int test_specv_fixed_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_specv_fixed_ft(8, &group);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	static const uint64_t keys[] = {
		0x0011223344556677ULL,
		0xaabbccddeeff0011ULL,
		0xdeadbeefcafef00dULL,
		0x0123456789abcdefULL,
	};
	enum { NKEYS = sizeof(keys) / sizeof(keys[0]) };
	struct ft_specv_node *nodes[NKEYS];
	unsigned int i;
	int ret = -1;

	if (!ft) {
		skip(1, "speculative_validated unsupported on this build/host");
		return 0;
	}

	for (i = 0; i < NKEYS; i++) {
		uint8_t k[8];

		cds_ft_u64_to_key(ft, keys[i], k, CDS_FT_LEN_DEFAULT);
		nodes[i] = specv_node_alloc(k, 8);
	}
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_insert(ft, nodes[i]->key, CDS_FT_LEN_DEFAULT,
			&nodes[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "specv_fixed_basic: insert %u: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto out;
		}
	}
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_eager_lookup_key(ft, nodes[i]->key, CDS_FT_LEN_DEFAULT, 0,
			&found);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "specv_fixed_basic: lookup %u: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto out;
		}
		if (found != &nodes[i]->node) {
			fprintf(stderr, "specv_fixed_basic: lookup %u wrong node\n",
				i);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();
	ret = 0;
out:
	return specv_drain_and_destroy(ft, group) | ret;
}

/*
 * Variable-length keys with a length offset: validation must check
 * stored_len against _key_len before the byte compare, and then
 * compare against the original caller key.
 */
static int test_specv_varlen_basic(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_specv_varlen_ft(&group);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	static const char *keys[] = {
		"alpha", "beta", "gamma",
		"prefix-shared-key-one",
		"prefix-shared-key-two",
		"prefix-shared-key-three",
	};
	enum { NKEYS = sizeof(keys) / sizeof(keys[0]) };
	struct ft_specv_node *nodes[NKEYS];
	unsigned int i;
	int ret = -1;

	if (!ft) {
		skip(1, "speculative_validated unsupported on this build/host");
		return 0;
	}

	for (i = 0; i < NKEYS; i++)
		nodes[i] = specv_node_alloc((const uint8_t *) keys[i],
			strlen(keys[i]));
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_insert(ft, nodes[i]->key, nodes[i]->key_len,
			&nodes[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "specv_varlen_basic: insert '%s': %s\n",
				keys[i], cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto out;
		}
	}
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_eager_lookup_key(ft, nodes[i]->key, nodes[i]->key_len, 0,
			&found);
		if (s != CDS_FT_STATUS_OK || found != &nodes[i]->node) {
			fprintf(stderr, "specv_varlen_basic: lookup '%s' status %s found %p (expected %p)\n",
				keys[i], cds_ft_status_to_string(s),
				(void *) found, (void *) &nodes[i]->node);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();
	ret = 0;
out:
	return specv_drain_and_destroy(ft, group) | ret;
}

/*
 * Long shared prefix exercises the cand-mode descent through long
 * compressed paths (and skip-compressed encoding).  Without the
 * orig_key fix this path was the most reliable repro: the descent
 * advances the cursor past the shared prefix, so the post-loop
 * validation comparator sees only the diverging-suffix tail and
 * rejects every match.
 */
static int test_specv_long_compressed_prefix(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_specv_varlen_ft(&group);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	static const char *prefix = "the-quick-brown-fox-jumps-over-the-lazy-dog";
	enum { NSUFFIX = 16, NKEYS = NSUFFIX };
	struct ft_specv_node *nodes[NKEYS];
	uint8_t kbuf[64];
	size_t plen;
	unsigned int i;
	int ret = -1;

	if (!ft) {
		skip(1, "speculative_validated unsupported on this build/host");
		return 0;
	}
	plen = strlen(prefix);
	for (i = 0; i < NKEYS; i++) {
		size_t klen;

		memcpy(kbuf, prefix, plen);
		kbuf[plen]     = '/';
		kbuf[plen + 1] = (uint8_t) ('a' + i);
		klen = plen + 2;
		nodes[i] = specv_node_alloc(kbuf, klen);
	}
	rcu_read_lock();
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_insert(ft, nodes[i]->key, nodes[i]->key_len,
			&nodes[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "specv_long_compressed_prefix: insert %u: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			goto out;
		}
	}
	for (i = 0; i < NKEYS; i++) {
		s = cds_ft_eager_lookup_key(ft, nodes[i]->key, nodes[i]->key_len, 0,
			&found);
		if (s != CDS_FT_STATUS_OK || found != &nodes[i]->node) {
			fprintf(stderr, "specv_long_compressed_prefix: lookup %u status %s found %p\n",
				i, cds_ft_status_to_string(s),
				(void *) found);
			rcu_read_unlock();
			goto out;
		}
	}
	rcu_read_unlock();
	ret = 0;
out:
	return specv_drain_and_destroy(ft, group) | ret;
}

/*
 * Negative case: a key absent from the trie must return NOT_FOUND
 * after validation rejects whichever candidate the cand-mode descent
 * returned.  Validates that the validation step actually rejects
 * non-matching candidates rather than reporting OK on whatever leaf
 * the descent landed on.
 */
static int test_specv_mismatch_rejected(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_specv_varlen_ft(&group);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	struct ft_specv_node *nodes[3];
	int ret = -1;
	static const char *inserted[] = {
		"prefix-shared-key-aaaaa",
		"prefix-shared-key-bbbbb",
		"prefix-shared-key-ccccc",
	};
	static const char *missing = "prefix-shared-key-zzzzz";
	unsigned int i;

	if (!ft) {
		skip(1, "speculative_validated unsupported on this build/host");
		return 0;
	}
	for (i = 0; i < 3; i++)
		nodes[i] = specv_node_alloc((const uint8_t *) inserted[i],
			strlen(inserted[i]));
	rcu_read_lock();
	for (i = 0; i < 3; i++) {
		s = cds_ft_insert(ft, nodes[i]->key, nodes[i]->key_len,
			&nodes[i]->node);
		if (s != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			goto out;
		}
	}
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *) missing,
		strlen(missing), 0, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "specv_mismatch_rejected: missing key returned %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	ret = 0;
out:
	return specv_drain_and_destroy(ft, group) | ret;
}

/*
 * Mixed-length keys at the same position: a short key that is a
 * prefix of an existing longer key must be rejected as NOT_FOUND
 * (when not actually inserted), even though the cand-mode descent
 * can land on the longer key's leaf along the shared path.  The
 * validation step rejects via the stored_len check before the byte
 * compare even runs, but the ordering matters — confirm the path.
 */
static int test_specv_prefix_key_mismatch(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_specv_varlen_ft(&group);
	struct cds_ft_node *found;
	enum cds_ft_status s;
	int ret = -1;
	static const char *long_key  = "shared-prefix-then-suffix";
	static const char *short_key = "shared-prefix";
	struct ft_specv_node *node;

	if (!ft) {
		skip(1, "speculative_validated unsupported on this build/host");
		return 0;
	}
	node = specv_node_alloc((const uint8_t *) long_key,
		strlen(long_key));
	rcu_read_lock();
	s = cds_ft_insert(ft, node->key, node->key_len, &node->node);
	if (s != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto out;
	}
	/* Short key not inserted: must NOT_FOUND. */
	s = cds_ft_eager_lookup_key(ft, (const uint8_t *) short_key,
		strlen(short_key), 0, &found);
	if (s != CDS_FT_STATUS_NOT_FOUND) {
		fprintf(stderr, "specv_prefix_key_mismatch: short key returned %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto out;
	}
	/* Long key inserted: must hit. */
	s = cds_ft_eager_lookup_key(ft, node->key, node->key_len, 0, &found);
	if (s != CDS_FT_STATUS_OK || found != &node->node) {
		fprintf(stderr, "specv_prefix_key_mismatch: long key status %s\n",
			cds_ft_status_to_string(s));
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	return specv_drain_and_destroy(ft, group) | ret;
}

/* ================================================================== */
/*                                                                    */
/*  15. Integrity verification tests                                  */
/*                                                                    */
/* ================================================================== */

/*
 * Verify empty trie passes integrity check.
 */
static int test_verify_empty(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	enum cds_ft_status s;

	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_empty: verify failed: %s\n",
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
 * Insert keys one at a time into a single trie level, growing through
 * all internal node configurations (popcount_2l, popcount_1l, pigeon).
 * Verify integrity after every insert.
 */
static int test_verify_recompact_grow(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	unsigned int i;

	rcu_read_lock();
	for (i = 0; i < 256; i++) {
		enum cds_ft_status s;

		s = insert_u64(ft, (0xAA << 8) | i, node_alloc((0xAA << 8) | i));
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_recompact_grow: insert %u failed: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_recompact_grow: verify failed after insert %u: %s\n",
				i, cds_ft_status_to_string(s));
			rcu_read_unlock();
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();
	return drain_and_destroy(ft, group);
}

/*
 * Fill a node to 256 children (pigeon), then remove keys one at a
 * time, shrinking through pigeon -> popcount_1l -> popcount_2l.
 * Verify integrity after every removal.
 */
static int test_verify_recompact_shrink(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int i;
	enum cds_ft_status s;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Fill to pigeon. */
	rcu_read_lock();
	for (i = 0; i < 256; i++) {
		s = insert_u64(ft, (0xBB << 8) | i,
			       node_alloc((0xBB << 8) | i));
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_recompact_shrink: insert %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();

	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_recompact_shrink: verify after fill failed\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Remove one at a time, verifying after each. */
	rcu_read_lock();
	for (i = 0; i < 256; i++) {
		uint8_t k[8];
		struct cds_ft_node *removed;

		cds_ft_u64_to_key(ft, (0xBB << 8) | i, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
		s = cds_ft_lookup(ft, iter);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_recompact_shrink: lookup %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		removed = cds_ft_iter_node(iter);
		s = cds_ft_remove(ft, iter, removed);
		if (s < 0) {
			fprintf(stderr, "verify_recompact_shrink: remove %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		node_free_rcu(to_test_node(removed));

		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_recompact_shrink: verify failed after remove %u\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Insert keys that share a long common prefix, creating compressed
 * path nodes.  Verify integrity after each insert.  Then remove
 * them one at a time (triggering compress merge-back) and verify
 * after each removal.
 */
static int test_verify_compress_split(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct cds_ft_iter *iter;
	unsigned long i;
	enum cds_ft_status s;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/*
	 * Insert keys 0..9 — on a 4-byte fixed trie these share a
	 * 3-byte prefix of zeros, producing compressed path nodes.
	 */
	rcu_read_lock();
	for (i = 0; i < 10; i++) {
		s = insert_u64(ft, i, node_alloc(i));
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_compress_split: insert %lu failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_compress_split: verify after insert %lu failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	/*
	 * Insert a key that diverges at byte 0 — splits the compressed
	 * path at the beginning.
	 */
	s = insert_u64(ft, 0x01000000ULL, node_alloc(0x01000000ULL));
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_compress_split: divergent insert failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_compress_split: verify after divergent insert failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/*
	 * Insert a key that diverges at byte 2 — splits within the
	 * compressed path.
	 */
	s = insert_u64(ft, 0x00000100ULL, node_alloc(0x00000100ULL));
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_compress_split: mid-split insert failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_compress_split: verify after mid-split insert failed\n");
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Now remove all keys, verifying after each. */
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			fprintf(stderr, "verify_compress_split: remove failed\n");
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_compress_split: verify after remove failed\n");
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Variable-length keys with prefix-terminating external nodes on
 * internal nodes.  Insert "a", "ab", "abc" — "a" terminates at an
 * internal node's external_nodes list, testing nr_keys counting for
 * the local_keys path.
 */
static int test_verify_varlen_prefix_keys(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ft(&group);
	enum cds_ft_status s;

	rcu_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *)"a", 1,
			  &node_alloc(0x61)->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: insert 'a' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: verify after 'a' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	s = cds_ft_insert(ft, (const uint8_t *)"ab", 2,
			  &node_alloc(0x6162)->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: insert 'ab' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: verify after 'ab' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	s = cds_ft_insert(ft, (const uint8_t *)"abc", 3,
			  &node_alloc(0x616263)->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: insert 'abc' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: verify after 'abc' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Add a duplicate at "a" — should not change nr_keys. */
	s = cds_ft_insert(ft, (const uint8_t *)"a", 1,
			  &node_alloc(0x61)->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: insert dup 'a' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: verify after dup 'a' failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}

	/* NIL key (zero-length). */
	s = cds_ft_insert(ft, NULL, 0, &node_alloc(0)->node);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: insert NIL failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_varlen: verify after NIL failed\n");
		rcu_read_unlock();
		drain_and_destroy(ft, group);
		return -1;
	}
	rcu_read_unlock();

	return drain_and_destroy(ft, group);
}

/*
 * Graft and detach: build a subtrie, graft it, verify, detach it,
 * verify both the source and the detached trie.
 */
static int test_verify_graft_detach(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live, *staging, *detached;
	enum cds_ft_status s;
	unsigned int i;

	if (cds_ft_group_create(NULL, &group) < 0)
		return -1;
	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &staging) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate staging with keys "xa", "xb", "xc". */
	rcu_read_lock();
	s = cds_ft_insert(staging, (const uint8_t *)"a", 1,
			  &node_alloc(0x61)->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(staging, (const uint8_t *)"b", 1,
			  &node_alloc(0x62)->node);
	if (s < 0) goto fail;
	s = cds_ft_insert(staging, (const uint8_t *)"c", 1,
			  &node_alloc(0x63)->node);
	if (s < 0) goto fail;
	rcu_read_unlock();

	s = cds_ft_verify(staging, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: staging verify failed\n");
		goto fail_nounlock;
	}

	/* Put some keys into live. */
	rcu_read_lock();
	for (i = 0; i < 5; i++) {
		uint8_t k[2] = { 'y', (uint8_t)('a' + i) };

		s = cds_ft_insert(live, k, 2, &node_alloc(0x7961 + i)->node);
		if (s < 0) goto fail;
	}
	rcu_read_unlock();

	s = cds_ft_verify(live, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: live pre-graft verify failed\n");
		goto fail_nounlock;
	}

	/* Graft staging into live at prefix "x". */
	s = cds_ft_graft(live, (const uint8_t *)"x", 1, staging);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: graft failed: %s\n",
			cds_ft_status_to_string(s));
		goto fail_nounlock;
	}

	s = cds_ft_verify(live, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: live post-graft verify failed\n");
		goto fail_nounlock;
	}

	/* Detach the "x" subtrie. */
	s = cds_ft_detach(live, (const uint8_t *)"x", 1, &detached);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: detach failed: %s\n",
			cds_ft_status_to_string(s));
		goto fail_nounlock;
	}

	s = cds_ft_verify(live, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: live post-detach verify failed\n");
		drain_and_destroy(detached, group);
		goto fail_nounlock;
	}
	s = cds_ft_verify(detached, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_graft_detach: detached verify failed\n");
		drain_and_destroy(detached, group);
		goto fail_nounlock;
	}

	if (drain_and_destroy(detached, group))
		goto fail_nounlock;
	cds_ft_destroy(staging);	/* empty after graft */
	return drain_and_destroy(live, group);

fail:
	rcu_read_unlock();
fail_nounlock:
	cds_ft_destroy(staging);
	drain_and_destroy(live, group);
	return -1;
}

/*
 * Compressed nested: two layers of compressed paths (8-byte keys where
 * keys share long prefixes but diverge at multiple levels).
 * Exercises nested compressed node creation/destruction with verify
 * at each step.
 */
static int test_verify_compress_nested(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(8, &group);
	enum cds_ft_status s;

	rcu_read_lock();
	/* First key: 7 bytes of zeros + 0x01 — creates compressed path. */
	s = insert_u64(ft, 0x01ULL, node_alloc(0x01));
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) goto fail;

	/* Second key: diverges at byte 6 — splits deep in the path. */
	s = insert_u64(ft, 0x0101ULL, node_alloc(0x0101));
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) goto fail;

	/* Third key: diverges at byte 0 — splits at the top. */
	s = insert_u64(ft, 0x0100000000000000ULL,
		       node_alloc(0x0100000000000000ULL));
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) goto fail;

	/* Fourth key: shares prefix with first two but diverges mid-path. */
	s = insert_u64(ft, 0x0001ULL, node_alloc(0x0001));
	if (s != CDS_FT_STATUS_OK) goto fail;
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) goto fail;

	rcu_read_unlock();
	return drain_and_destroy(ft, group);

fail:
	fprintf(stderr, "verify_compress_nested: failed\n");
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return -1;
}

/*
 * Oscillation: repeatedly insert and remove keys at a node type
 * boundary, triggering repeated recompaction across the hysteresis
 * threshold.  Verify integrity at each step.
 */
static int test_verify_oscillation(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(2, &group);
	struct cds_ft_iter *iter;
	unsigned int round, i;
	enum cds_ft_status s;
	/*
	 * On 64-bit: type boundary at max_child=14 (popcount_2l) with
	 * min_child=10 for the next type.  Insert 14, remove down to
	 * 10, repeat.
	 */
	unsigned int hi = 14, lo = 10;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Initial fill to hi. */
	rcu_read_lock();
	for (i = 0; i < hi; i++) {
		s = insert_u64(ft, (0xCC << 8) | i,
			       node_alloc((0xCC << 8) | i));
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_oscillation: initial insert %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();

	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "verify_oscillation: initial verify failed\n");
		cds_ft_iter_destroy(iter);
		drain_and_destroy(ft, group);
		return -1;
	}

	for (round = 0; round < 5; round++) {
		/* Remove from hi down to lo. */
		rcu_read_lock();
		for (i = hi; i > lo; i--) {
			uint8_t k[8];
			struct cds_ft_node *removed;

			cds_ft_u64_to_key(ft, (0xCC << 8) | (i - 1), k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
			s = cds_ft_lookup(ft, iter);
			if (s != CDS_FT_STATUS_OK) {
				fprintf(stderr, "verify_oscillation: lookup %u failed round %u\n",
					i - 1, round);
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
			removed = cds_ft_iter_node(iter);
			s = cds_ft_remove(ft, iter, removed);
			if (s < 0) {
				fprintf(stderr, "verify_oscillation: remove %u failed round %u\n",
					i - 1, round);
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
			node_free_rcu(to_test_node(removed));
		}
		rcu_read_unlock();

		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_oscillation: verify after shrink round %u failed\n",
				round);
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}

		/* Re-insert from lo back to hi. */
		rcu_read_lock();
		for (i = lo; i < hi; i++) {
			s = insert_u64(ft, (0xCC << 8) | i,
				       node_alloc((0xCC << 8) | i));
			if (s != CDS_FT_STATUS_OK) {
				fprintf(stderr, "verify_oscillation: re-insert %u failed round %u\n",
					i, round);
				rcu_read_unlock();
				cds_ft_iter_destroy(iter);
				drain_and_destroy(ft, group);
				return -1;
			}
		}
		rcu_read_unlock();

		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "verify_oscillation: verify after grow round %u failed\n",
				round);
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	cds_ft_iter_destroy(iter);
	return drain_and_destroy(ft, group);
}


/*
 * Density counter verification: remove through compressed paths.
 *
 * Build a trie with compressed paths (long shared prefixes), split
 * them by inserting divergent keys, then remove keys one at a time.
 * Removals traverse through compressed nodes and cause subtrees
 * under compressed paths to shrink, testing density propagation
 * when compressed children lose subtree content.
 *
 * Uses 8-byte fixed keys for deep compressed paths.
 */
static int test_density_remove_through_compress(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(8, &group);
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	unsigned int i;

	/*
	 * Keys share a 6-byte prefix (bytes 0-5 = 0), diverge at
	 * byte 6, creating a compressed path of length 6 from the
	 * root's child to the divergence point.
	 */
	uint64_t keys[] = {
		0x0000000000000100ULL,	/* diverge at byte 6 = 0x01, byte 7 = 0x00 */
		0x0000000000000200ULL,	/* diverge at byte 6 = 0x02 */
		0x0000000000000300ULL,	/* diverge at byte 6 = 0x03 */
		0x0000000000000101ULL,	/* same byte 6 as [0], diverge at byte 7 */
		0x0000000000000102ULL,	/* same byte 6 as [0], diverge at byte 7 */
		0x0000000000000201ULL,	/* same byte 6 as [1], diverge at byte 7 */
		/* A key that diverges early (byte 0) to trigger root recompact. */
		0x0100000000000000ULL,
		/* Another key diverging at byte 3 (mid-compressed-path split). */
		0x0000000100000000ULL,
	};
	unsigned int nkeys = sizeof(keys) / sizeof(keys[0]);

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Insert all keys, verifying after each. */
	rcu_read_lock();
	for (i = 0; i < nkeys; i++) {
		s = insert_u64(ft, keys[i], node_alloc(keys[i]));
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "density_rm_compress: insert %u (0x%016" PRIx64 ") failed: %s\n",
				i, keys[i], cds_ft_status_to_string(s));
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "density_rm_compress: structural verify failed after insert %u\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}

	/* Remove keys one at a time, verifying after each. */
	for (i = 0; i < nkeys; i++) {
		uint8_t k[8];
		struct cds_ft_node *removed;

		cds_ft_u64_to_key(ft, keys[i], k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, cds_ft_group_key_len(group));
		s = cds_ft_lookup(ft, iter);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "density_rm_compress: lookup key %u for remove failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		removed = cds_ft_iter_node(iter);
		s = cds_ft_remove(ft, iter, removed);
		if (s < 0) {
			fprintf(stderr, "density_rm_compress: remove key %u failed\n", i);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
		node_free_rcu(to_test_node(removed));

		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "density_rm_compress: structural verify failed after remove %u\n", i);
			cds_ft_show(ft, stderr, CDS_FT_SHOW_PRETTY);
			rcu_read_unlock();
			cds_ft_iter_destroy(iter);
			drain_and_destroy(ft, group);
			return -1;
		}
	}
	rcu_read_unlock();
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * Density counter verification: graft-swap.
 *
 * Exercises cds_ft_graft_swap with density verification on both
 * the destination and swap tries before and after each swap.
 *
 * Phase 1: swap into empty destination slot.
 * Phase 2: swap replacing existing content.
 * Phase 3: swap at root level.
 */
static int test_density_graft_swap(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live, *swap;
	enum cds_ft_status s;
	unsigned int i;

	/*
	 * Force SPECULATIVE so the test exercises the chain-compress
	 * invariant on the graft_swap path (under EAGER the verify
	 * walk's SC-only checks are no-ops).
	 */
	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(attr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &swap) < 0) {
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	/* Populate live trie with keys under prefix "a". */
	for (i = 0; i < 5; i++) {
		uint8_t k[2] = { 'a', (uint8_t)('0' + i) };

		s = cds_ft_insert(live, k, 2, &node_alloc(0xa030 + i)->node);
		if (s < 0) goto fail;
	}

	/* Populate swap trie with keys (become "x" + key after swap). */
	for (i = 0; i < 3; i++) {
		uint8_t k[2] = { 'b', (uint8_t)('0' + i) };

		s = cds_ft_insert(swap, k, 2, &node_alloc(0xb030 + i)->node);
		if (s < 0) goto fail;
	}

	/*
	 * Phase 1: graft swap content into live at prefix "x".
	 * Use cds_ft_graft (not graft_swap) for the initial graft
	 * into an empty slot, because graft_swap assumes the slot
	 * already exists in the parent node.
	 */
	s = cds_ft_graft(live, (const uint8_t *)"x", 1, swap);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "density_graft_swap: phase 1 swap failed: %s\n",
			cds_ft_status_to_string(s));
		goto fail_nounlock;
	}

	s = cds_ft_verify(live, stderr);
	if (s < 0) {
		fprintf(stderr, "density_graft_swap: live phase 1 structural failed\n");
		goto fail_nounlock;
	}

	/*
	 * Phase 2: populate swap with new content, then swap again at "x".
	 * This replaces live's "x" subtree with the new swap content;
	 * the old "x" content moves into swap.
	 */
	rcu_read_lock();
	for (i = 0; i < 4; i++) {
		uint8_t k[2] = { 'c', (uint8_t)('0' + i) };

		s = cds_ft_insert(swap, k, 2, &node_alloc(0xc030 + i)->node);
		if (s < 0) goto fail;
	}
	rcu_read_unlock();

	s = cds_ft_graft_swap(live, (const uint8_t *)"x", 1, swap);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "density_graft_swap: phase 2 swap failed: %s\n",
			cds_ft_status_to_string(s));
		goto fail_nounlock;
	}

	s = cds_ft_verify(live, stderr);
	if (s < 0) {
		fprintf(stderr, "density_graft_swap: live phase 2 structural failed\n");
		goto fail_nounlock;
	}

	/*
	 * Phase 3: root-level graft-swap.  Exchange entire live trie
	 * content with swap trie content.
	 */
	s = cds_ft_graft_swap(live, NULL, 0, swap);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "density_graft_swap: phase 3 root swap failed: %s\n",
			cds_ft_status_to_string(s));
		goto fail_nounlock;
	}

	s = cds_ft_verify(live, stderr);
	if (s < 0) {
		fprintf(stderr, "density_graft_swap: live phase 3 structural failed\n");
		goto fail_nounlock;
	}

	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;

fail:
	rcu_read_unlock();
fail_nounlock:
	drain_trie(swap);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(swap);
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Stress test: random insert/remove with integrity verification
 * after every mutation.
 *
 * Uses a deterministic PRNG (xorshift32) for reproducibility.
 * Variable-length keys exercise compress, split, and recompact paths.
 * Keys are 1-6 bytes, drawn from a small alphabet to maximize prefix
 * sharing and structural transitions.
 */

static uint32_t xorshift32(uint32_t *state)
{
	uint32_t x = *state;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

#define STRESS_MAX_KEYS		200
#define STRESS_KEY_MAX_LEN	6
#define STRESS_OPS		600

static int test_density_stress(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	uint32_t rng = 0xdeadbeef;	/* fixed seed */
	unsigned int op;

	/*
	 * Track inserted keys so we can remove them.
	 * Each slot: key bytes + length + node pointer.
	 */
	struct {
		uint8_t key[STRESS_KEY_MAX_LEN];
		size_t len;
		struct ft_test_node *node;
		bool live;
	} keys[STRESS_MAX_KEYS];
	unsigned int nr_live = 0;
	unsigned int nr_keys = 0;

	memset(keys, 0, sizeof(keys));

	ft = create_varlen_ft(&group);
	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		drain_and_destroy(ft, group);
		return -1;
	}

	rcu_read_lock();
	for (op = 0; op < STRESS_OPS; op++) {
		uint32_t r = xorshift32(&rng);
		/*
		 * Bias: insert when few keys, remove when many,
		 * 50/50 in the middle.
		 */
		bool do_insert = (nr_live < 10) ||
			(nr_live < STRESS_MAX_KEYS && (r & 1));

		if (do_insert && nr_keys < STRESS_MAX_KEYS) {
			/* Generate a random key: 1-6 bytes from {0..5}. */
			unsigned int slot = nr_keys;
			unsigned int klen = 1 + (xorshift32(&rng) % STRESS_KEY_MAX_LEN);
			unsigned int k;

			for (k = 0; k < klen; k++)
				keys[slot].key[k] = (uint8_t)(xorshift32(&rng) % 6);
			keys[slot].len = klen;
			keys[slot].node = node_alloc(slot);
			keys[slot].live = true;

			s = cds_ft_insert(ft, keys[slot].key, klen,
					  &keys[slot].node->node);
			if (s != CDS_FT_STATUS_OK) {
				fprintf(stderr, "stress: insert op %u slot %u failed: %s\n",
					op, slot, cds_ft_status_to_string(s));
				goto fail;
			}
			nr_keys++;
			nr_live++;
		} else if (nr_live > 0) {
			/* Remove a random live key. */
			unsigned int pick = xorshift32(&rng) % nr_keys;
			unsigned int i;
			struct cds_ft_node *removed;

			/* Find next live key from pick. */
			for (i = 0; i < nr_keys; i++) {
				unsigned int idx = (pick + i) % nr_keys;

				if (!keys[idx].live)
					continue;
				cds_ft_iter_set_key(iter, keys[idx].key,
						    keys[idx].len);
				s = cds_ft_lookup(ft, iter);
				if (s != CDS_FT_STATUS_OK) {
					fprintf(stderr, "stress: lookup for remove op %u slot %u failed\n",
						op, idx);
					goto fail;
				}
				removed = cds_ft_iter_node(iter);
				s = cds_ft_remove(ft, iter, removed);
				if (s < 0) {
					fprintf(stderr, "stress: remove op %u slot %u failed\n",
						op, idx);
					goto fail;
				}
				node_free_rcu(keys[idx].node);
				keys[idx].live = false;
				nr_live--;
				break;
			}
		}

		/* Verify after every mutation. */
		s = cds_ft_verify(ft, stderr);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "stress: structural verify failed at op %u "
				"(nr_live=%u, seed=0xdeadbeef)\n", op, nr_live);
			cds_ft_show(ft, stderr, CDS_FT_SHOW_PRETTY);
			goto fail;
		}
	}
	rcu_read_unlock();

	/* Drain remaining keys. */
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			fprintf(stderr, "stress: final drain failed\n");
			goto fail;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
	}
	rcu_read_unlock();

	/* Verify empty trie. */
	s = cds_ft_verify(ft, stderr);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "stress: structural verify after drain failed\n");
		goto fail_nounlock;
	}
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return 0;

fail:
	rcu_read_unlock();
fail_nounlock:
	cds_ft_iter_destroy(iter);
	drain_and_destroy(ft, group);
	return -1;
}

/* ================================================================== */
/*                                                                    */
/*         17. EXCLUSIVE ACCESS DISCIPLINE TESTS                      */
/*                                                                    */
/* ================================================================== */

/*
 * Default access discipline is concurrent (exclusive = false) when
 * attr is NULL.
 */
static int test_exclusive_default_is_concurrent(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	int ret = 0;

	ft = create_varlen_ft(&group);
	if (cds_ft_is_exclusive(ft)) {
		fprintf(stderr, "exclusive_default_is_concurrent: expected concurrent\n");
		ret = -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * cds_ft_attr_set_exclusive(true) + cds_ft_create with that attr
 * produces an exclusive trie.
 */
static int test_exclusive_attr_set_true(void)
{
	struct cds_ft_group *group;
	struct cds_ft_attr *attr;
	struct cds_ft *ft;
	int ret = 0;

	if (cds_ft_group_create(NULL, &group) < 0)
		return -1;
	if (cds_ft_attr_create(&attr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_attr_set_exclusive(attr, true) < 0) {
		cds_ft_attr_destroy(attr);
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, attr, &ft) < 0) {
		cds_ft_attr_destroy(attr);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_attr_destroy(attr);
	if (!cds_ft_is_exclusive(ft)) {
		fprintf(stderr, "exclusive_attr_set_true: expected exclusive\n");
		ret = -1;
	}
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * cds_ft_make_exclusive / cds_ft_make_concurrent round-trips; also
 * exercises the idempotent exclusive → exclusive case (no-op).
 */
static int test_exclusive_make_transitions(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	int ret = 0;

	ft = create_varlen_ft(&group);
	if (cds_ft_is_exclusive(ft)) {
		ret = -1;
		goto end;
	}
	cds_ft_make_exclusive(ft);
	if (!cds_ft_is_exclusive(ft)) {
		ret = -1;
		goto end;
	}
	/* Idempotent: already exclusive, should stay exclusive without syncing. */
	cds_ft_make_exclusive(ft);
	if (!cds_ft_is_exclusive(ft)) {
		ret = -1;
		goto end;
	}
	cds_ft_make_concurrent(ft);
	if (cds_ft_is_exclusive(ft)) {
		ret = -1;
		goto end;
	}
	cds_ft_make_exclusive(ft);
	if (!cds_ft_is_exclusive(ft)) {
		ret = -1;
		goto end;
	}
end:
	if (ret)
		fprintf(stderr, "exclusive_make_transitions: wrong state\n");
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * cds_ft_detach returns the detached trie in exclusive mode
 * regardless of the source's access discipline: no external handle
 * to the detached trie exists at return, and the detach path has
 * already drained any in-flight readers of the moved subtree, so
 * no concurrent reader can be inside it.
 */
static int test_exclusive_detach_always_exclusive(void)
{
	struct cds_ft_group *group;
	struct cds_ft *excl_src, *conc_src, *det_from_excl, *det_from_conc;
	struct ft_test_node *n;
	enum cds_ft_status s;
	int ret = -1;

	/* Build two sources with different disciplines. */
	excl_src = create_varlen_ft(&group);
	cds_ft_make_exclusive(excl_src);
	if (cds_ft_create(group, NULL, &conc_src) < 0)
		goto out_excl;

	/* Populate both so root-level detach has content to move. */
	n = node_alloc(0);
	s = cds_ft_insert(excl_src, (const uint8_t *)"a", 1, &n->node);
	if (s < 0) { node_free(n); goto out_both; }
	n = node_alloc(0);
	s = cds_ft_insert(conc_src, (const uint8_t *)"a", 1, &n->node);
	if (s < 0) { node_free(n); goto out_both; }

	s = cds_ft_detach(excl_src, NULL, 0, &det_from_excl);
	if (s != CDS_FT_STATUS_OK)
		goto out_both;
	if (!cds_ft_is_exclusive(det_from_excl)) {
		fprintf(stderr, "detach_always_exclusive: exclusive source → detached not exclusive\n");
		goto out_det_excl;
	}

	s = cds_ft_detach(conc_src, NULL, 0, &det_from_conc);
	if (s != CDS_FT_STATUS_OK)
		goto out_det_excl;
	if (!cds_ft_is_exclusive(det_from_conc)) {
		fprintf(stderr, "detach_always_exclusive: concurrent source → detached not exclusive\n");
		drain_trie(det_from_conc);
		rcu_barrier();
		cds_ft_destroy(det_from_conc);
		goto out_det_excl;
	}

	ret = 0;
	drain_trie(det_from_conc);
	rcu_barrier();
	cds_ft_destroy(det_from_conc);
out_det_excl:
	drain_trie(det_from_excl);
	rcu_barrier();
	cds_ft_destroy(det_from_excl);
out_both:
	drain_trie(conc_src);
	rcu_barrier();
	cds_ft_destroy(conc_src);
out_excl:
	drain_trie(excl_src);
	rcu_barrier();
	cds_ft_destroy(excl_src);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Populate-then-graft workflow with an exclusive staging trie.
 * Staging stays exclusive (no internal sync on graft); content
 * correctly migrates into the concurrent live trie.
 */
static int test_exclusive_graft_from_exclusive(void)
{
	struct cds_ft_group *group;
	struct cds_ft_attr *attr;
	struct cds_ft *live, *staging;
	struct cds_ft_node *found;
	struct ft_test_node *n1, *n2;
	enum cds_ft_status s;
	int ret = -1;

	live = create_varlen_ft(&group);
	if (cds_ft_attr_create(&attr) < 0)
		goto out_live;
	cds_ft_attr_set_exclusive(attr, true);
	if (cds_ft_create(group, attr, &staging) < 0) {
		cds_ft_attr_destroy(attr);
		goto out_live;
	}
	cds_ft_attr_destroy(attr);

	if (!cds_ft_is_exclusive(staging) || cds_ft_is_exclusive(live)) {
		fprintf(stderr, "graft_from_exclusive: wrong initial state\n");
		goto out_both;
	}

	n1 = node_alloc(0);
	s = cds_ft_insert(staging, (const uint8_t *)"lo", 2, &n1->node);
	if (s < 0) { node_free(n1); goto out_both; }
	n2 = node_alloc(0);
	s = cds_ft_insert(staging, (const uint8_t *)"lp", 2, &n2->node);
	if (s < 0) { node_free(n2); goto out_both; }

	rcu_read_lock();
	s = cds_ft_graft(live, (const uint8_t *)"he", 2, staging);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK)
		goto out_both;
	if (!cds_ft_empty(staging))
		goto out_both;
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"helo", 4, 0, &found);
	if (s != CDS_FT_STATUS_OK || !found) {
		rcu_read_unlock();
		goto out_both;
	}
	s = cds_ft_eager_lookup_key(live, (const uint8_t *)"help", 4, 0, &found);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK || !found)
		goto out_both;

	/* Discipline flags are unchanged by graft. */
	if (!cds_ft_is_exclusive(staging) || cds_ft_is_exclusive(live)) {
		fprintf(stderr, "graft_from_exclusive: post-graft state changed\n");
		goto out_both;
	}
	ret = 0;
out_both:
	drain_trie(staging);
	drain_trie(live);
	rcu_barrier();
	cds_ft_destroy(staging);
out_live:
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Root-level graft_swap: swap inherits dst's pre-swap exclusive
 * state; dst keeps its own discipline.  Exercises both combinations
 * (exclusive dst, concurrent swap) and vice versa.
 */
static int test_exclusive_graft_swap_inherit_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *swp;
	enum cds_ft_status s;
	int ret = -1;

	/* Case 1: dst exclusive, swp concurrent — swp becomes exclusive. */
	dst = create_varlen_ft(&group);
	cds_ft_make_exclusive(dst);
	if (cds_ft_create(group, NULL, &swp) < 0)
		goto out_case1_dst_only;
	/* Populate dst so swap has content to exchange. */
	{
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *)"x", 1, &n->node);
		if (s < 0) { node_free(n); goto out_case1; }
	}
	rcu_read_lock();
	s = cds_ft_graft_swap(dst, NULL, 0, swp);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK)
		goto out_case1;
	if (!cds_ft_is_exclusive(dst) || !cds_ft_is_exclusive(swp)) {
		fprintf(stderr, "graft_swap_inherit_root: case 1 bad state\n");
		goto out_case1;
	}
	drain_trie(dst);
	drain_trie(swp);
	rcu_barrier();
	cds_ft_destroy(swp);
out_case1_dst_only:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);

	/* Case 2: dst concurrent, swp exclusive — swp becomes concurrent. */
	dst = create_varlen_ft(&group);
	{
		struct cds_ft_attr *attr;
		if (cds_ft_attr_create(&attr) < 0)
			goto out_case2_dst_only;
		cds_ft_attr_set_exclusive(attr, true);
		if (cds_ft_create(group, attr, &swp) < 0) {
			cds_ft_attr_destroy(attr);
			goto out_case2_dst_only;
		}
		cds_ft_attr_destroy(attr);
	}
	{
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *)"y", 1, &n->node);
		if (s < 0) { node_free(n); goto out_case2; }
	}
	rcu_read_lock();
	s = cds_ft_graft_swap(dst, NULL, 0, swp);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK)
		goto out_case2;
	if (cds_ft_is_exclusive(dst) || cds_ft_is_exclusive(swp)) {
		fprintf(stderr, "graft_swap_inherit_root: case 2 bad state\n");
		goto out_case2;
	}
	ret = 0;
out_case2:
	drain_trie(dst);
	drain_trie(swp);
	rcu_barrier();
	cds_ft_destroy(swp);
out_case2_dst_only:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;

out_case1:
	drain_trie(dst);
	drain_trie(swp);
	rcu_barrier();
	cds_ft_destroy(swp);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return -1;
}

/*
 * Non-root graft_swap at a prefix: swap inherits dst's exclusive
 * state (the displaced subtree came from dst).
 */
static int test_exclusive_graft_swap_inherit_non_root(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *swp;
	struct ft_test_node *n;
	enum cds_ft_status s;
	int ret = -1;

	/* dst exclusive with content at "he*"; swp concurrent with content "*". */
	dst = create_varlen_ft(&group);
	cds_ft_make_exclusive(dst);
	n = node_alloc(0);
	s = cds_ft_insert(dst, (const uint8_t *)"helo", 4, &n->node);
	if (s < 0) { node_free(n); goto out_dst; }

	if (cds_ft_create(group, NULL, &swp) < 0)
		goto out_dst;
	n = node_alloc(0);
	s = cds_ft_insert(swp, (const uint8_t *)"lx", 2, &n->node);
	if (s < 0) { node_free(n); goto out_both; }

	rcu_read_lock();
	s = cds_ft_graft_swap(dst, (const uint8_t *)"he", 2, swp);
	rcu_read_unlock();
	if (s != CDS_FT_STATUS_OK)
		goto out_both;

	/* dst still exclusive; swp inherited dst's pre-swap exclusive = true. */
	if (!cds_ft_is_exclusive(dst) || !cds_ft_is_exclusive(swp)) {
		fprintf(stderr, "graft_swap_inherit_non_root: bad state\n");
		goto out_both;
	}
	ret = 0;
out_both:
	drain_trie(swp);
	rcu_barrier();
	cds_ft_destroy(swp);
out_dst:
	drain_trie(dst);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Negative tests for the FEATURE_FT_EXCL_VALIDATE validator.         */
/*                                                                    */
/* Each provocation runs in a forked child where two threads          */
/* deliberately violate the access-discipline contract on the same    */
/* trie.  When the validator is compiled into the library the child   */
/* aborts with SIGABRT; when it is absent the test is a no-op that    */
/* diag's the skip reason and passes.                                 */
/* ------------------------------------------------------------------ */

#define FT_EXCL_NEG_ITERATIONS		(200 * 1000)

struct excl_neg_ctx {
	struct cds_ft *ft;
	pthread_barrier_t *start;
	unsigned int seed_bump;
};

static void *excl_neg_writer(void *arg)
{
	struct excl_neg_ctx *ctx = (struct excl_neg_ctx *) arg;
	unsigned int i;

	rcu_register_thread();
	pthread_barrier_wait(ctx->start);
	for (i = 0; i < FT_EXCL_NEG_ITERATIONS; i++) {
		struct ft_test_node *n = node_alloc(0);
		uint8_t key[4];
		unsigned int k = i ^ ctx->seed_bump;

		key[0] = (uint8_t) k;
		key[1] = (uint8_t) (k >> 8);
		key[2] = (uint8_t) ctx->seed_bump;
		key[3] = 0;
		if (cds_ft_insert(ctx->ft, key, 4, &n->node) != CDS_FT_STATUS_OK)
			node_free(n);
	}
	rcu_unregister_thread();
	return NULL;
}

static void *excl_neg_reader(void *arg)
{
	struct excl_neg_ctx *ctx = (struct excl_neg_ctx *) arg;
	unsigned int i;

	rcu_register_thread();
	pthread_barrier_wait(ctx->start);
	for (i = 0; i < FT_EXCL_NEG_ITERATIONS; i++) {
		struct cds_ft_node *found;
		uint8_t key[4] = { 0 };

		rcu_read_lock();
		(void) cds_ft_eager_lookup_key(ctx->ft, key, 4, 0, &found);
		rcu_read_unlock();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * In the forked child: two writers racing on the same trie trip the
 * writer/writer CAS.  Works in both concurrent and exclusive mode.
 */
__attribute__((noreturn))
static void excl_neg_writer_writer_child(void)
{
	struct cds_ft_group *group;
	struct excl_neg_ctx ctx_a, ctx_b;
	pthread_barrier_t start;
	pthread_t t_a, t_b;

	/* Don't clutter the test harness with the validator's abort msg. */
	(void) freopen("/dev/null", "w", stderr);
	alarm(30);
	ctx_a.ft = ctx_b.ft = create_varlen_ft(&group);
	pthread_barrier_init(&start, NULL, 2);
	ctx_a.start = ctx_b.start = &start;
	ctx_a.seed_bump = 0x1111;
	ctx_b.seed_bump = 0x2222;
	pthread_create(&t_a, NULL, excl_neg_writer, &ctx_a);
	pthread_create(&t_b, NULL, excl_neg_writer, &ctx_b);
	pthread_join(t_a, NULL);
	pthread_join(t_b, NULL);
	/* Should not reach here — the validator should have abort()'d. */
	_exit(42);
}

/*
 * Concurrent-mode reader that does NOT hold the RCU read-side lock
 * across the trie API call.  In QSBR a registered thread is online
 * by default and read_ongoing() returns true; calling
 * rcu_thread_offline() before the loop forces read_ongoing() to
 * report false so the validator's mutex-claim path is exercised.
 *
 * No actual UAF risk: the writer thread stays online and never
 * reaches a quiescent state inside the FT API, so call_rcu
 * callbacks are deferred and no node is freed before the validator
 * aborts the process on the first reader/writer overlap.
 */
static void *excl_neg_reader_no_rcu(void *arg)
{
	struct excl_neg_ctx *ctx = (struct excl_neg_ctx *) arg;
	unsigned int i;

	rcu_register_thread();
	rcu_thread_offline();
	pthread_barrier_wait(ctx->start);
	for (i = 0; i < FT_EXCL_NEG_ITERATIONS; i++) {
		struct cds_ft_node *found;
		uint8_t key[4] = { 0 };

		(void) cds_ft_eager_lookup_key(ctx->ft, key, 4, 0, &found);
	}
	rcu_thread_online();
	rcu_unregister_thread();
	return NULL;
}

/*
 * In the forked child on a concurrent-mode trie: a reader that did
 * not take the RCU read-side lock racing with a writer trips the
 * generalised reader/writer check.
 */
__attribute__((noreturn))
static void excl_neg_concurrent_reader_writer_child(void)
{
	struct cds_ft_group *group;
	struct excl_neg_ctx ctx_r, ctx_w;
	pthread_barrier_t start;
	pthread_t t_r, t_w;

	(void) freopen("/dev/null", "w", stderr);
	alarm(30);
	ctx_r.ft = ctx_w.ft = create_varlen_ft(&group);
	pthread_barrier_init(&start, NULL, 2);
	ctx_r.start = ctx_w.start = &start;
	ctx_r.seed_bump = 0;
	ctx_w.seed_bump = 0x4444;
	pthread_create(&t_r, NULL, excl_neg_reader_no_rcu, &ctx_r);
	pthread_create(&t_w, NULL, excl_neg_writer, &ctx_w);
	pthread_join(t_r, NULL);
	pthread_join(t_w, NULL);
	_exit(42);
}

/*
 * In the forked child on an exclusive-mode trie: a reader and a
 * writer racing on the same trie trip the exclusive-mode
 * reader/writer check.
 */
__attribute__((noreturn))
static void excl_neg_excl_reader_writer_child(void)
{
	struct cds_ft_group *group;
	struct excl_neg_ctx ctx_r, ctx_w;
	pthread_barrier_t start;
	pthread_t t_r, t_w;

	(void) freopen("/dev/null", "w", stderr);
	alarm(30);
	ctx_r.ft = ctx_w.ft = create_varlen_ft(&group);
	cds_ft_make_exclusive(ctx_r.ft);
	pthread_barrier_init(&start, NULL, 2);
	ctx_r.start = ctx_w.start = &start;
	ctx_r.seed_bump = 0;
	ctx_w.seed_bump = 0x3333;
	pthread_create(&t_r, NULL, excl_neg_reader, &ctx_r);
	pthread_create(&t_w, NULL, excl_neg_writer, &ctx_w);
	pthread_join(t_r, NULL);
	pthread_join(t_w, NULL);
	_exit(42);
}

static int excl_neg_expect_sigabrt(void (*child_fn)(void))
{
	pid_t pid;
	int status;

	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "excl_neg: fork failed\n");
		return -1;
	}
	if (pid == 0) {
		child_fn();
		_exit(42);	/* unreachable */
	}
	if (waitpid(pid, &status, 0) != pid) {
		fprintf(stderr, "excl_neg: waitpid failed\n");
		return -1;
	}
	if (!WIFSIGNALED(status)) {
		fprintf(stderr,
			"excl_neg: child exited normally (code %d), "
			"expected SIGABRT\n",
			WIFEXITED(status) ? WEXITSTATUS(status) : -1);
		return -1;
	}
	if (WTERMSIG(status) != SIGABRT) {
		fprintf(stderr,
			"excl_neg: child killed by signal %d, expected SIGABRT\n",
			WTERMSIG(status));
		return -1;
	}
	return 0;
}

static int test_excl_validate_writer_writer(void)
{
	if (!cds_ft_excl_validate_enabled()) {
		diag("FEATURE_FT_EXCL_VALIDATE not compiled in; "
			"writer/writer provocation is a no-op");
		return 0;
	}
	return excl_neg_expect_sigabrt(excl_neg_writer_writer_child);
}

static int test_excl_validate_excl_reader_writer(void)
{
	if (!cds_ft_excl_validate_enabled()) {
		diag("FEATURE_FT_EXCL_VALIDATE not compiled in; "
			"exclusive reader/writer provocation is a no-op");
		return 0;
	}
	return excl_neg_expect_sigabrt(excl_neg_excl_reader_writer_child);
}

static int test_excl_validate_concurrent_reader_writer_no_rcu(void)
{
	if (!cds_ft_excl_validate_enabled()) {
		diag("FEATURE_FT_EXCL_VALIDATE not compiled in; "
			"concurrent reader/writer (no RCU read lock) "
			"provocation is a no-op");
		return 0;
	}
	return excl_neg_expect_sigabrt(excl_neg_concurrent_reader_writer_child);
}

/* ================================================================== */
/*                                                                    */
/*                  19. cds_ft_merge tests                            */
/*                                                                    */
/* ================================================================== */

/*
 * Helper used by merge tests: collect every (key, node) pair reachable
 * in @ft and verify they exactly match an expected list of keys.
 */
struct merge_expected {
	const char *key;
	size_t key_len;
};

static int verify_keys_present(struct cds_ft *ft,
		const struct merge_expected *exp, unsigned int nr_exp)
{
	unsigned int i;
	int ret = 0;

	rcu_read_lock();
	if (cds_ft_count_entries(ft) != nr_exp) {
		fprintf(stderr, "verify_keys_present: count %lu != expected %u\n",
			cds_ft_count_entries(ft), nr_exp);
		ret = -1;
		goto out;
	}
	for (i = 0; i < nr_exp; i++) {
		struct cds_ft_node *found = NULL;
		enum cds_ft_status s;

		s = cds_ft_eager_lookup_key(ft, (const uint8_t *) exp[i].key,
				exp[i].key_len, 0, &found);
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "verify_keys_present: missing '%.*s'\n",
				(int) exp[i].key_len, exp[i].key);
			ret = -1;
			goto out;
		}
	}
out:
	rcu_read_unlock();
	return ret;
}

/*
 * Merge with a disjoint LCP: src has all keys under prefix "z", dst
 * has none.  Triggers the detach+graft fast path.  After merge, dst
 * must contain its original keys plus src's keys, byte-identical to
 * what was inserted.
 */
static int test_merge_disjoint_prefix_fast_path(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected expected[] = {
		{ "alpha", 5 }, { "beta", 4 },
		{ "zaa", 3 }, { "zab", 3 }, { "zac", 3 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 2; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 2; i < 5; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_disjoint_prefix: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_disjoint_prefix: src not empty\n");
		goto out;
	}
	if (verify_keys_present(dst, expected, 5) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge where dst has content under src's LCP: forces the per-entry
 * fallback.  Verifies that all keys end up in dst with original bytes.
 */
static int test_merge_overlapping_per_entry(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected expected[] = {
		{ "k01", 3 }, { "k03", 3 }, { "k05", 3 },	/* dst */
		{ "k02", 3 }, { "k04", 3 }, { "k06", 3 },	/* src */
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 3; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 3; i < 6; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_overlapping: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_overlapping: src not empty\n");
		goto out;
	}
	if (verify_keys_present(dst, expected, 6) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge where the overlap between src and dst contains COMPRESSED nodes:
 * shared multi-byte runs, mid-run divergence (suffix == bare child, and
 * suffix == a fresh compressed wrapper), a compressed run meeting an
 * internal node that carries external_nodes, same-key splices, and a
 * disjoint reference.  Exercises ft_merge_build's compressed cases (the
 * build-invisible spine-copy when src is merged at its root into a
 * non-empty dst), falling back to the per-entry path only for shapes the
 * builder still delegates.  Verifies the full key union, the spliced
 * duplicate count, structural integrity, and an emptied src.
 *
 *   Group A (identical run + branch + splice):
 *       dst {aaaa1, aaaa2}            src {aaaa1, aaaa3}
 *   Group B (diverge mid-run, suffix == bare leaf child):
 *       dst {bbbbbX}                  src {bbbbbY}
 *   Group C (diverge mid-run, suffix == fresh compressed wrapper):
 *       dst {ccccPQR}                 src {ccccXYZ}
 *   Group D (compressed run meets internal node with external_nodes):
 *       dst {dd, ddmn}                src {ddpq}
 *   Group E (disjoint reference):
 *       dst {zzzz}                    src {wwww}
 */
static int test_merge_compressed_overlap(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dst_keys[] = {
		"aaaa1", "aaaa2", "bbbbbX", "ccccPQR", "dd", "ddmn", "zzzz",
	};
	static const char *const src_keys[] = {
		"aaaa1", "aaaa3", "bbbbbY", "ccccXYZ", "ddpq", "wwww",
	};
	const struct merge_expected expected[] = {
		{ "aaaa1", 5 }, { "aaaa2", 5 }, { "aaaa3", 5 },
		{ "bbbbbX", 6 }, { "bbbbbY", 6 },
		{ "ccccPQR", 7 }, { "ccccXYZ", 7 },
		{ "dd", 2 }, { "ddmn", 4 }, { "ddpq", 4 },
		{ "zzzz", 4 }, { "wwww", 4 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dst_keys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dst_keys[i],
				strlen(dst_keys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(src_keys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) src_keys[i],
				strlen(src_keys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_compressed_overlap: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_compressed_overlap: src not empty\n");
		goto out;
	}
	/*
	 * Every distinct key survives, and the entry total equals the sum of
	 * both inputs (the shared key "aaaa1" becomes a 2-entry chain, so the
	 * per-key presence check is used directly rather than
	 * verify_keys_present, which assumes count == distinct keys).
	 */
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(expected); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) expected[i].key,
				expected[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_compressed_overlap: missing '%s'\n",
				expected[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != CAA_ARRAY_SIZE(dst_keys) + CAA_ARRAY_SIZE(src_keys)) {
		fprintf(stderr, "merge_compressed_overlap: %lu entries, expected %zu\n",
			total, CAA_ARRAY_SIZE(dst_keys) + CAA_ARRAY_SIZE(src_keys));
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_compressed_overlap: dst verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge with an empty src: no-op, dst unchanged.
 */
static int test_merge_empty_source(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct ft_test_node *n;
	enum cds_ft_status s;
	int ret = -1;
	unsigned long count_before, count_after;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	n = node_alloc(0);
	s = cds_ft_insert(dst, (const uint8_t *)"keep", 4, &n->node);
	if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }

	rcu_read_lock();
	count_before = cds_ft_count_entries(dst);
	rcu_read_unlock();

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_empty_source: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}

	rcu_read_lock();
	count_after = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (count_before != count_after) {
		fprintf(stderr, "merge_empty_source: count changed %lu→%lu\n",
			count_before, count_after);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge with LCP=0 + dst empty: root-level graft fast path.
 */
static int test_merge_at_root_empty_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected expected[] = {
		{ "alpha", 5 }, { "beta", 4 }, { "gamma", 5 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 3; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_root_empty_dst: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_root_empty_dst: src not empty\n");
		goto out;
	}
	if (verify_keys_present(dst, expected, 3) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge with duplicate chains in src: every duplicate must end up in
 * dst at the same key.
 */
static int test_merge_duplicate_chains(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct cds_ft_iter *iter;
	struct cds_ft_node *found;
	enum cds_ft_status s;
	int ret = -1;
	unsigned int i, count;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	/* 5 duplicates at key "dup" in src. */
	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(0);
		n->value = i;
		s = cds_ft_insert(src, (const uint8_t *)"dup", 3, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_duplicate_chains: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_duplicate_chains: src not empty\n");
		goto out;
	}

	if (cds_ft_iter_create(dst, &iter) != CDS_FT_STATUS_OK)
		goto out;
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(dst, (const uint8_t *)"dup", 3, 0, &found);
	count = 0;
	if (s == CDS_FT_STATUS_OK) {
		struct cds_ft_node *p;
		cds_ft_for_each_duplicate_safe_rcu(found, p)
			count++;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	if (count != 5) {
		fprintf(stderr, "merge_duplicate_chains: %u dups, expected 5\n",
			count);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Argument validation: NULL, dst == src, mismatched groups,
 * out-of-range key_len all return INVALID_ARGUMENT_ERROR.
 *
 * Non-exclusive src is now accepted (the implementation detaches
 * @key first, which yields an exclusive transient regardless of
 * src's mode), so it is no longer a rejected case.
 */
static int test_merge_invalid_arguments(void)
{
	struct cds_ft_group *group1, *group2;
	struct cds_ft *ft1a, *ft1b, *ft2;
	enum cds_ft_status s;
	int ret = -1;

	ft1a = create_varlen_ft(&group1);
	if (cds_ft_create(group1, NULL, &ft1b) < 0)
		goto out_ft1a;
	ft2 = create_varlen_ft(&group2);

	s = cds_ft_merge(NULL, NULL, 0, ft1b);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) goto out;
	s = cds_ft_merge(ft1a, NULL, 0, NULL);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) goto out;
	s = cds_ft_merge(ft1a, NULL, 0, ft1a);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) goto out;
	s = cds_ft_merge(ft1a, NULL, 0, ft2);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) goto out;

	ret = 0;
out:
	cds_ft_destroy(ft2);
	cds_ft_group_destroy(group2);
	cds_ft_destroy(ft1b);
out_ft1a:
	cds_ft_destroy(ft1a);
	cds_ft_group_destroy(group1);
	return ret;
}

/*
 * Merge from a concurrent (non-exclusive) src trie.  The new
 * detach-first design accepts this because the initial detach
 * drains src's RCU readers and produces an exclusive transient
 * for the rest of the merge.
 */
static int test_merge_concurrent_source(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected expected[] = {
		{ "k1", 2 }, { "k2", 2 }, { "k3", 2 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	/* src stays in concurrent (default) mode — no make_exclusive. */
	for (i = 0; i < 3; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) expected[i].key,
				expected[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	if (cds_ft_is_exclusive(src)) {
		fprintf(stderr, "merge_concurrent_source: src expected concurrent\n");
		goto out;
	}

	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_concurrent_source: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_concurrent_source: src not empty\n");
		goto out;
	}
	if (verify_keys_present(dst, expected, 3) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Prefix-scoped merge: src has keys both inside and outside the
 * @key prefix.  Only keys under @key are moved to dst.  src keys
 * outside @key remain untouched.
 */
static int test_merge_prefix_subtree(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected src_keys[] = {
		{ "z01", 3 }, { "z02", 3 }, { "z03", 3 },	/* moved */
		{ "alpha", 5 }, { "beta", 4 },			/* stay  */
	};
	const struct merge_expected dst_after[] = {
		{ "z01", 3 }, { "z02", 3 }, { "z03", 3 },
	};
	const struct merge_expected src_after[] = {
		{ "alpha", 5 }, { "beta", 4 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) src_keys[i].key,
				src_keys[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, (const uint8_t *)"z", 1, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_prefix_subtree: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (verify_keys_present(dst, dst_after, 3) < 0)
		goto out;
	if (verify_keys_present(src, src_after, 2) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Fixed-length merge fast path: src has 4-byte integer keys all
 * sharing a leading byte, dst has none.  The internal detach+graft
 * helpers (which skip the public API's fixed-length-vs-non-root
 * rejection) move src as a sub-trie at the LCP.
 */
static int test_merge_fixed_length_fast_path(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	uint64_t i;
	unsigned long count;

	dst = create_fixed_ft(4, &group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	/* Insert dst keys in the 0x00.. range. */
	for (i = 0; i < 5; i++) {
		struct ft_test_node *n = node_alloc(i);
		uint8_t k[4];
		cds_ft_u64_to_key(dst, i, k, CDS_FT_LEN_DEFAULT);
		s = cds_ft_insert(dst, k, 4, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	/* Insert src keys in the 0xff.. range — disjoint from dst. */
	for (i = 0; i < 5; i++) {
		uint64_t v = 0xff000000ull | i;
		struct ft_test_node *n = node_alloc(v);
		uint8_t k[4];
		cds_ft_u64_to_key(src, v, k, CDS_FT_LEN_DEFAULT);
		s = cds_ft_insert(src, k, 4, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, NULL, 0, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_fixed_length_fast_path: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_fixed_length_fast_path: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	count = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (count != 10) {
		fprintf(stderr, "merge_fixed_length_fast_path: count=%lu, expected 10\n",
			count);
		goto out;
	}
	/* Verify each src key landed in dst with original byte value. */
	for (i = 0; i < 5; i++) {
		uint64_t v = 0xff000000ull | i;
		struct cds_ft_node *found = NULL;

		rcu_read_lock();
		s = lookup_u64(dst, v, &found);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "merge_fixed_length_fast_path: missing 0x%lx\n",
				(unsigned long) v);
			goto out;
		}
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge-at-key with overlap: src has keys under @key, dst already has
 * overlapping keys under the same @key.  Forces the per-entry
 * fallback (scoped to @key).  src keys outside @key remain.
 */
static int test_merge_prefix_overlap_per_entry(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	unsigned int i;
	const struct merge_expected dst_after[] = {
		{ "p01", 3 }, { "p02", 3 }, { "p03", 3 },
		{ "p04", 3 }, { "p05", 3 }, { "p06", 3 },
	};
	const struct merge_expected src_after[] = {
		{ "outside", 7 },
	};

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	/* dst gets p01, p03, p05; src gets p02, p04, p06 + "outside". */
	for (i = 0; i < 6; i += 2) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 1; i < 6; i += 2) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	{
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *)"outside", 7, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	cds_ft_make_exclusive(src);
	s = cds_ft_merge(dst, (const uint8_t *)"p", 1, src);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_prefix_overlap: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (verify_keys_present(dst, dst_after, 6) < 0)
		goto out;
	if (verify_keys_present(src, src_after, 1) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Cross-key merge on a variable-length group: src has keys under
 * "src/", merged into dst under "dst/".  Each src key K = "src/X"
 * lands in dst as "dst/X".  src keys outside "src/" are left.
 */
static int test_merge_at_varlen_rekey(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected src_keys[] = {
		{ "src/a", 5 }, { "src/b", 5 }, { "src/c", 5 },
		{ "elsewhere", 9 },
	};
	const struct merge_expected dst_after[] = {
		{ "dst/a", 5 }, { "dst/b", 5 }, { "dst/c", 5 },
	};
	const struct merge_expected src_after[] = {
		{ "elsewhere", 9 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 4; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) src_keys[i].key,
				src_keys[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst,
			(const uint8_t *)"dst/", 4,
			src,
			(const uint8_t *)"src/", 4);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_varlen_rekey: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (verify_keys_present(dst, dst_after, 3) < 0)
		goto out;
	if (verify_keys_present(src, src_after, 1) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Cross-key merge on a fixed-length group: 4-byte keys, src_key and
 * dst_key both 1 byte (same length, as required for fixed-length).
 * Src has keys under leading byte 0x10, merged into dst under
 * leading byte 0x20.  Verifies that fixed-length groups can do
 * cross-key merges (which the public detach+graft pair cannot).
 */
static int test_merge_at_fixed_rekey(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	uint8_t src_prefix = 0x10, dst_prefix = 0x20;
	uint64_t i;

	dst = create_fixed_ft(4, &group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 5; i++) {
		uint64_t v = ((uint64_t) src_prefix << 24) | i;
		struct ft_test_node *n = node_alloc(v);
		uint8_t k[4];
		cds_ft_u64_to_key(src, v, k, CDS_FT_LEN_DEFAULT);
		s = cds_ft_insert(src, k, 4, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst,
			&dst_prefix, 1,
			src,
			&src_prefix, 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_fixed_rekey: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_fixed_rekey: src not empty\n");
		goto out;
	}
	for (i = 0; i < 5; i++) {
		uint64_t v = ((uint64_t) dst_prefix << 24) | i;
		struct cds_ft_node *found = NULL;

		rcu_read_lock();
		s = lookup_u64(dst, v, &found);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK || !found) {
			fprintf(stderr, "merge_at_fixed_rekey: missing dst 0x%lx\n",
				(unsigned long) v);
			goto out;
		}
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Cross-key merge with overlap: dst already has content under
 * dst_key.  Per-entry fallback engages.
 */
static int test_merge_at_overlap(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected dst_pre[] = {
		{ "dst/x", 5 },
	};
	const struct merge_expected src_pre[] = {
		{ "src/a", 5 }, { "src/b", 5 },
	};
	const struct merge_expected dst_after[] = {
		{ "dst/x", 5 }, { "dst/a", 5 }, { "dst/b", 5 },
	};
	unsigned int i;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < 1; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(dst, (const uint8_t *) dst_pre[i].key,
				dst_pre[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < 2; i++) {
		struct ft_test_node *n = node_alloc(0);
		s = cds_ft_insert(src, (const uint8_t *) src_pre[i].key,
				src_pre[i].key_len, &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst,
			(const uint8_t *)"dst/", 4,
			src,
			(const uint8_t *)"src/", 4);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_overlap: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_overlap: src not empty\n");
		goto out;
	}
	if (verify_keys_present(dst, dst_after, 3) < 0)
		goto out;
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Non-root SRC into a ROOT dst: exercises the build-invisible spine-copy's
 * in-place src-branch detach (the non-root-src unlink that replaces the
 * root-src ft->root swap).  src holds an "S" subtree (the merge source) AND a
 * disjoint "T" subtree that must survive; the merge detaches only the "S"
 * branch.  The overlap is compressed ("aaa" run shared with dst), so this also
 * rides the compressed spine-copy.  dst is at its root and non-empty, so the
 * flip publishes through ft->root (no non-root-dst delegation).
 *
 *   dst {aaaa, aaab}                 src {Saaaa, Saaac, Tzz}
 *   merge_at(dst, "", src, "S") -> dst {aaaa x2, aaab, aaac}, src {Tzz}
 */
static int test_merge_at_nonroot_src(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "aaaa", "aaab" };
	static const char *const skeys[] = { "Saaaa", "Saaac", "Tzz" };
	const struct merge_expected dst_after[] = {
		{ "aaaa", 4 }, { "aaab", 4 }, { "aaac", 4 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, NULL, 0, src, (const uint8_t *) "S", 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_nonroot_src: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_nonroot_src: dst missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != 4) {	/* aaaa is a 2-entry chain. */
		fprintf(stderr, "merge_at_nonroot_src: dst %lu entries, expected 4\n",
			total);
		goto out;
	}
	/* The disjoint "T" subtree must remain in src; "S" content is gone. */
	{
		struct cds_ft_node *tn = NULL, *sn = NULL;
		unsigned long src_total;

		rcu_read_lock();
		(void) cds_ft_eager_lookup_key(src, (const uint8_t *) "Tzz", 3, 0, &tn);
		(void) cds_ft_eager_lookup_key(src, (const uint8_t *) "Saaaa", 5, 0, &sn);
		src_total = cds_ft_count_entries(src);
		rcu_read_unlock();
		if (!tn || sn || src_total != 1) {
			fprintf(stderr, "merge_at_nonroot_src: src residue wrong (Tzz=%p Saaaa=%p n=%lu)\n",
				(void *) tn, (void *) sn, src_total);
			goto out;
		}
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK ||
	    cds_ft_verify(src, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_nonroot_src: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge into a NON-ROOT dst merge point: the merged cluster publishes through
 * an interior forward slot, so the flip installs a type-7 proxy there and
 * readers descending past it resolve it at child dispatch.  dst_key "P" lands
 * at a depth-1 INTERNAL subtree (its parent is the root, so d_dst->pnf != NULL);
 * single-byte children keep that merge point a plain internal branch (a
 * compressed / external merge point still delegates).  A disjoint "Z" subtree
 * confirms the rest of dst is intact.
 *
 *   dst {Pa, Pb, Z}                  src {a, c}  (root src)
 *   merge_at(dst, "P", src, "") -> dst {Pa x2, Pb, Pc, Z}, src empty
 */
static int test_merge_at_nonroot_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "Pa", "Pb", "Z" };
	static const char *const skeys[] = { "a", "c" };
	const struct merge_expected dst_after[] = {
		{ "Pa", 2 }, { "Pb", 2 }, { "Pc", 2 }, { "Z", 1 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_nonroot_dst: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_nonroot_dst: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_nonroot_dst: dst missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != 5) {	/* Pa is a 2-entry chain. */
		fprintf(stderr, "merge_at_nonroot_dst: dst %lu entries, expected 5\n",
			total);
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_nonroot_dst: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge into an EXTERNAL non-root dst merge point: dst_key is a single leaf
 * (no subtree below it) whose parent is a plain internal node.  Merging a
 * src subtree under it builds a fresh INTERNAL M that carries the original
 * leaf as M's external_nodes (the key terminating at the merge point) plus
 * src's re-keyed children.  M is internal, so it publishes through the
 * interior forward slot exactly like the all-internal case.
 *
 *   dst {P, Z}                       src {a, c}  (root src)
 *   merge_at(dst, "P", src, "") -> dst {P, Pa, Pc, Z}, src empty
 *
 * "P" survives as a key (now a prefix of Pa/Pc) and a disjoint "Z" confirms
 * the rest of dst is intact.
 */
static int test_merge_at_external_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "P", "Z" };
	static const char *const skeys[] = { "a", "c" };
	const struct merge_expected dst_after[] = {
		{ "P", 1 }, { "Pa", 2 }, { "Pc", 2 }, { "Z", 1 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_external_dst: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_external_dst: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_external_dst: dst missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != 4) {
		fprintf(stderr, "merge_at_external_dst: dst %lu entries, expected 4\n",
			total);
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_external_dst: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge a single-leaf non-root SRC into an EXTERNAL non-root dst merge point:
 * the SAME full key terminates on both sides, so the merge is a pure
 * duplicate-chain splice (M is the surviving dst external head, non-internal).
 *
 *   dst {P, Z}   src {Sx} (x has the same suffix as P? no -- exact key match)
 * To make the keys coincide, both sides hold key "P" at the merge points:
 *   dst {P, Z}, src {QP}  merge_at(dst, "P", src, "Q")
 * src@"Q" is the single leaf "P"-suffix... simpler: merge_at(dst,"P",src,"Q")
 * where src={Q} so src@Q is a leaf; result re-keys Q's (empty) suffix under P
 * -> dst "P" becomes a 2-entry chain.
 */
static int test_merge_at_external_dst_splice(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct cds_ft_node *out_node = NULL;
	struct cds_ft_node *tmp;
	enum cds_ft_status s;
	int ret = -1;
	unsigned int dups = 0;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);
		struct ft_test_node *c = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "P", 1, &a->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *) "Z", 1, &b->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "Q", 1, &c->node) < 0)
			goto out;
	}

	/* src@"Q" (single leaf) re-keyed under dst@"P" -> "P" gets a 2nd entry. */
	s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1,
			src, (const uint8_t *) "Q", 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_external_dst_splice: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_external_dst_splice: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(dst, (const uint8_t *) "P", 1, 0, &out_node);
	if (s == CDS_FT_STATUS_OK && out_node)
		cds_ft_for_each_duplicate_safe_rcu(out_node, tmp)
			dups++;
	rcu_read_unlock();
	if (dups != 2) {
		fprintf(stderr, "merge_at_external_dst_splice: P has %u dups, expected 2\n",
			dups);
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_external_dst_splice: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge into a COMPRESSED non-root dst merge point where the result M is a
 * fresh INTERNAL branch: dst_key "P" lands exactly at a compressed node
 * ("Pabc" the only key under it), root src diverges at the first byte -> M
 * branches.
 *
 *   dst {Pabc, Z}   src {x, y}  ->  dst {Pabc, Px, Py, Z}, src empty
 */
static int test_merge_at_compressed_dst_internal(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "Pabc", "Z" };
	static const char *const skeys[] = { "x", "y" };
	const struct merge_expected dst_after[] = {
		{ "Pabc", 4 }, { "Px", 2 }, { "Py", 2 }, { "Z", 1 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_compressed_dst_internal: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_compressed_dst_internal: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_compressed_dst_internal: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != 4) {
		fprintf(stderr, "merge_at_compressed_dst_internal: %lu entries, expected 4\n",
			total);
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_compressed_dst_internal: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Merge into a COMPRESSED non-root dst merge point where the result M is
 * itself COMPRESSED (both sides compressed runs sharing a prefix), published
 * into the interior slot SKIP-ENCODED (the M_slot path).
 *
 *   dst {Pxyz, Z}   src {Qxyw}  ->  dst {Pxyz, Pxyw, Z}, src empty
 */
static int test_merge_at_compressed_dst_compressed(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected dst_after[] = {
		{ "Pxyz", 4 }, { "Pxyw", 4 }, { "Z", 1 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);
		struct ft_test_node *c = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "Pxyz", 4, &a->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *) "Z", 1, &b->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "Qxyw", 4, &c->node) < 0)
			goto out;
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1,
			src, (const uint8_t *) "Q", 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_compressed_dst_compressed: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_compressed_dst_compressed: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_compressed_dst_compressed: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	rcu_read_unlock();
	if (total != 3) {
		fprintf(stderr, "merge_at_compressed_dst_compressed: %lu entries, expected 3\n",
			total);
		goto out;
	}
	rcu_read_lock();
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_compressed_dst_compressed: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * KEY_SHORTER dst: @dst_key ends strictly INSIDE a compressed dst node, so
 * the merge point sits mid-edge and the merged cluster is wrapped under the
 * node's prefix bytes.  Here the merged top M is INTERNAL (src contributes
 * disjoint bytes), wrapped under a fresh compressed prefix.
 *
 *   dst {abcd}  src {P, Q} at ""  ->  merge_at "ab"
 *   ("bcd" splits at offset 1: prefix "b", suffix "cd"->leaf)
 *   ->  dst {abcd, abP, abQ}, src empty
 */
static int test_merge_at_key_shorter_dst_internal(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const skeys[] = { "P", "Q" };
	const struct merge_expected dst_after[] = {
		{ "abcd", 4 }, { "abP", 3 }, { "abQ", 3 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *n = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "abcd", 4, &n->node) < 0) {
			node_free(n);
			goto out;
		}
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "ab", 2, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_key_shorter_dst_internal: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_key_shorter_dst_internal: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_key_shorter_dst_internal: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_key_shorter_dst_internal: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (total != 3) {
		fprintf(stderr, "merge_at_key_shorter_dst_internal: %lu entries, expected 3\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * KEY_SHORTER dst where the re-keyed src content is a DUPLICATE of an existing
 * dst key reached mid-compressed-node.  The merged top M is a COMPRESSED run
 * whose child is the LIVE dst leaf (a splice), which must be carried as a
 * dst_origin re-parent through the flip latch -- ft_merge_wrap_prefix fuses
 * the prefix into the run while preserving that dst_origin.
 *
 *   dst {PQRS}  src {XRS} merged at src_key "X" into dst_key "PQ"
 *   ("QRS" splits at offset 1: prefix "Q", suffix "RS"->leaf; src "RS"
 *   collides on the same suffix) ->  dst {PQRS x2}, src empty
 */
static int test_merge_at_key_shorter_dst_splice(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct cds_ft_node *found, *p;
	enum cds_ft_status s;
	int ret = -1;
	unsigned int count;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "PQRS", 4, &a->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "XRS", 3, &b->node) < 0)
			goto out;
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "PQ", 2,
			src, (const uint8_t *) "X", 1);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_key_shorter_dst_splice: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_key_shorter_dst_splice: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(dst, (const uint8_t *) "PQRS", 4, 0, &found);
	count = 0;
	if (s == CDS_FT_STATUS_OK) {
		cds_ft_for_each_duplicate_safe_rcu(found, p)
			count++;
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_key_shorter_dst_splice: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "merge_at_key_shorter_dst_splice: %u dups at PQRS, expected 2\n",
			count);
		goto out;
	}
	if (total != 2) {
		fprintf(stderr, "merge_at_key_shorter_dst_splice: %lu entries, expected 2\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * KEY_SHORTER src: @src_key ends strictly INSIDE a compressed src node.  The
 * build enters that node at the cursor (referencing its child as a src-origin
 * edge applied after the source drain), and the commit's unlink reclaims the
 * whole compressed node.  Here the dst merge point is EXACT.
 *
 *   src {XYZ}  ("YZ" under 'X'),  dst {Qa}  ("a" under 'Q')
 *   merge_at src_key "XY" into dst_key "Q"  ("YZ" splits at offset 1: the
 *   "Z"->leaf suffix re-keys to "QZ")  ->  dst {Qa, QZ}, src empty
 */
static int test_merge_at_key_shorter_src(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected dst_after[] = {
		{ "Qa", 2 }, { "QZ", 2 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "Qa", 2, &a->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "XYZ", 3, &b->node) < 0)
			goto out;
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "Q", 1,
			src, (const uint8_t *) "XY", 2);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_key_shorter_src: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_key_shorter_src: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_key_shorter_src: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_key_shorter_src: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (total != 2) {
		fprintf(stderr, "merge_at_key_shorter_src: %lu entries, expected 2\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Both sides KEY_SHORTER: @src_key ends inside a compressed src node AND
 * @dst_key ends inside a compressed dst node.  Exercises the off_src + off_dst
 * cursors together with the prefix wrap at publish.
 *
 *   src {XYZ},  dst {Qab}  merge_at src_key "XY" into dst_key "Qa"
 *   ("YZ" suffix "Z"->leaf re-keys to "QaZ"; "ab" splits at prefix "a")
 *   ->  dst {Qab, QaZ}, src empty
 */
static int test_merge_at_key_shorter_src_both(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	const struct merge_expected dst_after[] = {
		{ "Qab", 3 }, { "QaZ", 3 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	{
		struct ft_test_node *a = node_alloc(0);
		struct ft_test_node *b = node_alloc(0);

		if (cds_ft_insert(dst, (const uint8_t *) "Qab", 3, &a->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "XYZ", 3, &b->node) < 0)
			goto out;
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "Qa", 2,
			src, (const uint8_t *) "XY", 2);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_key_shorter_src_both: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_key_shorter_src_both: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_key_shorter_src_both: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_key_shorter_src_both: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (total != 2) {
		fprintf(stderr, "merge_at_key_shorter_src_both: %lu entries, expected 2\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Edge D: the dst merge point's PARENT is a compressed node, reached via a
 * grandparent skip slot.  The merge is EXACT at the merge point; M is wrapped
 * under a fresh copy of the whole compressed parent and published into the
 * grandparent slot.
 *
 *   dst {aXYc, aXYd}  ("XY" under 'a' -> internal{c,d}),  src {P, Q} at ""
 *   merge_at "aXY"  ->  dst {aXYc, aXYd, aXYP, aXYQ}, src empty
 */
static int test_merge_at_compressed_parent_internal(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "aXYc", "aXYd" };
	static const char *const skeys[] = { "P", "Q" };
	const struct merge_expected dst_after[] = {
		{ "aXYc", 4 }, { "aXYd", 4 }, { "aXYP", 4 }, { "aXYQ", 4 },
	};
	unsigned int i;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(src, (const uint8_t *) skeys[i],
				strlen(skeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "aXY", 3, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_compressed_parent_internal: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_compressed_parent_internal: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < CAA_ARRAY_SIZE(dst_after); i++) {
		struct cds_ft_node *found = NULL;

		if (cds_ft_eager_lookup_key(dst,
				(const uint8_t *) dst_after[i].key,
				dst_after[i].key_len, 0,
				&found) != CDS_FT_STATUS_OK || !found) {
			rcu_read_unlock();
			fprintf(stderr, "merge_at_compressed_parent_internal: missing '%s'\n",
				dst_after[i].key);
			goto out;
		}
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_compressed_parent_internal: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (total != 4) {
		fprintf(stderr, "merge_at_compressed_parent_internal: %lu entries, expected 4\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Edge D with a duplicate-producing merge: the re-keyed src content collides
 * with an existing dst key under the compressed parent, so a dst leaf is
 * re-parented AND spliced through the flip while its grandparent skip slot is
 * re-encoded to the fresh compressed-parent copy.
 *
 *   dst {aXYc, aXYd},  src {c}  merge_at "aXY"  ->  "aXYc" x2, plus aXYd
 */
static int test_merge_at_compressed_parent_splice(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct cds_ft_node *found, *p;
	enum cds_ft_status s;
	int ret = -1;
	static const char *const dkeys[] = { "aXYc", "aXYd" };
	unsigned int i, count;
	unsigned long total;

	dst = create_varlen_ft(&group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
		struct ft_test_node *n = node_alloc(0);

		s = cds_ft_insert(dst, (const uint8_t *) dkeys[i],
				strlen(dkeys[i]), &n->node);
		if (s != CDS_FT_STATUS_OK) { node_free(n); goto out; }
	}
	{
		struct ft_test_node *n = node_alloc(0);

		if (cds_ft_insert(src, (const uint8_t *) "c", 1, &n->node) < 0) {
			node_free(n);
			goto out;
		}
	}

	s = cds_ft_merge_at(dst, (const uint8_t *) "aXY", 3, src, NULL, 0);
	if (s != CDS_FT_STATUS_OK) {
		fprintf(stderr, "merge_at_compressed_parent_splice: %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	if (!cds_ft_empty(src)) {
		fprintf(stderr, "merge_at_compressed_parent_splice: src not empty\n");
		goto out;
	}
	rcu_read_lock();
	s = cds_ft_eager_lookup_key(dst, (const uint8_t *) "aXYc", 4, 0, &found);
	count = 0;
	if (s == CDS_FT_STATUS_OK) {
		cds_ft_for_each_duplicate_safe_rcu(found, p)
			count++;
	}
	total = cds_ft_count_entries(dst);
	if (cds_ft_verify(dst, stderr) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		fprintf(stderr, "merge_at_compressed_parent_splice: verify failed\n");
		goto out;
	}
	rcu_read_unlock();
	if (count != 2) {
		fprintf(stderr, "merge_at_compressed_parent_splice: %u dups at aXYc, expected 2\n",
			count);
		goto out;
	}
	if (total != 3) {	/* aXYc x2 + aXYd */
		fprintf(stderr, "merge_at_compressed_parent_splice: %lu entries, expected 3\n",
			total);
		goto out;
	}
	ret = 0;
out:
	drain_trie(dst);
	drain_trie(src);
	rcu_barrier();
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Fixed-length group rejects cross-key merge with mismatched key
 * lengths.
 */
static int test_merge_at_fixed_unequal_keylen(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	enum cds_ft_status s;
	uint8_t a = 0x10, b[2] = { 0x20, 0x21 };
	int ret = -1;

	dst = create_fixed_ft(4, &group);
	if (cds_ft_create(group, NULL, &src) < 0)
		goto out_dst;

	s = cds_ft_merge_at(dst, b, 2, src, &a, 1);
	if (s != CDS_FT_STATUS_INVALID_ARGUMENT_ERROR) {
		fprintf(stderr, "merge_at_fixed_unequal_keylen: expected INVALID, got %s\n",
			cds_ft_status_to_string(s));
		goto out;
	}
	ret = 0;
out:
	cds_ft_destroy(src);
out_dst:
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                  EXTERNAL-NODE ARENA TESTS                         */
/*                                                                    */
/* ================================================================== */

/*
 * Basic create / alloc / destroy.
 */
static int test_external_arena_basic(void)
{
	struct cds_ft_external_arena *a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	void *p = cds_ft_external_arena_alloc(a, 16);
	if (!p) {
		cds_ft_external_arena_destroy(a);
		return -1;
	}
	/* Allocation must be at least 16-byte aligned (MIN_ORDER). */
	if (((uintptr_t) p) & 0xF) {
		cds_ft_external_arena_destroy(a);
		return -1;
	}
	/* Zero-initialised. */
	uint8_t buf[16];
	memcpy(buf, p, 16);
	for (int i = 0; i < 16; i++)
		if (buf[i] != 0) {
			cds_ft_external_arena_destroy(a);
			return -1;
		}
	cds_ft_external_arena_destroy(a);
	return 0;
}

/*
 * Return 1 if the /proc/self/smaps VMA containing @addr lists VmFlags @flag,
 * 0 if VmFlags was found without it, -1 if undeterminable (no smaps, no
 * VmFlags line for that VMA).  Used to confirm the cds_ft_optimize hint
 * reaches madvise: MADV_NOHUGEPAGE sets the "nh" flag regardless of whether
 * THP formation is enabled (so it is reliable even in a THP-disabled sandbox).
 */
static int smaps_vmflag(const void *addr, const char *flag)
{
	unsigned long a = (unsigned long) addr;
	char line[1024];
	int in_vma = 0, ret = -1;
	FILE *f = fopen("/proc/self/smaps", "r");

	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f)) {
		unsigned long s, e;

		if (sscanf(line, "%lx-%lx", &s, &e) == 2) {
			in_vma = (a >= s && a < e);
		} else if (in_vma && strncmp(line, "VmFlags:", 8) == 0) {
			char *tok = strtok(line + 8, " \t\n");

			ret = 0;
			for (; tok != NULL; tok = strtok(NULL, " \t\n"))
				if (strcmp(tok, flag) == 0) {
					ret = 1;
					break;
				}
			break;
		}
	}
	fclose(f);
	return ret;
}

/*
 * cds_ft_optimize attribute API: group + external-arena setters accept the
 * two valid values and reject anything else; create works with an attr and
 * with NULL (default).
 */
static int test_optimize_attr_api(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_external_arena_attr *eattr;
	struct cds_ft_external_arena *a;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) != CDS_FT_STATUS_OK)
		return -1;
	if (cds_ft_group_attr_set_optimize(gattr, CDS_FT_OPTIMIZE_THROUGHPUT) != CDS_FT_STATUS_OK ||
	    cds_ft_group_attr_set_optimize(gattr, CDS_FT_OPTIMIZE_RSS) != CDS_FT_STATUS_OK ||
	    cds_ft_group_attr_set_optimize(gattr, (enum cds_ft_optimize) 99) !=
			CDS_FT_STATUS_INVALID_ARGUMENT_ERROR)
		ret = -1;
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_external_arena_attr_create(&eattr) != CDS_FT_STATUS_OK)
		return -1;
	if (cds_ft_external_arena_attr_set_optimize(eattr, CDS_FT_OPTIMIZE_RSS) != CDS_FT_STATUS_OK ||
	    cds_ft_external_arena_attr_set_optimize(eattr, (enum cds_ft_optimize) 99) !=
			CDS_FT_STATUS_INVALID_ARGUMENT_ERROR)
		ret = -1;
	a = cds_ft_external_arena_create(eattr);
	cds_ft_external_arena_attr_destroy(eattr);
	if (!a)
		return -1;
	if (!cds_ft_external_arena_alloc(a, 32))
		ret = -1;
	cds_ft_external_arena_destroy(a);

	a = cds_ft_external_arena_create(NULL);	/* default */
	if (!a)
		return -1;
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * The external-arena hint must reach madvise: an RSS arena's range VMA carries
 * the "nh" (MADV_NOHUGEPAGE) flag; a THROUGHPUT arena's does not.  Skips
 * gracefully if smaps is unreadable.
 */
static int test_optimize_external_thp(void)
{
	struct cds_ft_external_arena_attr *attr;
	struct cds_ft_external_arena *a;
	void *p;
	int nh, ret = 0;

	if (cds_ft_external_arena_attr_create(&attr) != CDS_FT_STATUS_OK)
		return -1;
	cds_ft_external_arena_attr_set_optimize(attr, CDS_FT_OPTIMIZE_THROUGHPUT);
	a = cds_ft_external_arena_create(attr);
	cds_ft_external_arena_attr_destroy(attr);
	if (!a)
		return -1;
	p = cds_ft_external_arena_alloc(a, 64);
	nh = p ? smaps_vmflag(p, "nh") : -1;
	cds_ft_external_arena_destroy(a);
	if (!p)
		return -1;
	if (nh == 1) {		/* THROUGHPUT arena wrongly advised NOHUGEPAGE */
		fprintf(stderr, "optimize_external_thp: THROUGHPUT arena marked nh\n");
		ret = -1;
	}

	if (cds_ft_external_arena_attr_create(&attr) != CDS_FT_STATUS_OK)
		return -1;
	cds_ft_external_arena_attr_set_optimize(attr, CDS_FT_OPTIMIZE_RSS);
	a = cds_ft_external_arena_create(attr);
	cds_ft_external_arena_attr_destroy(attr);
	if (!a)
		return -1;
	p = cds_ft_external_arena_alloc(a, 64);
	nh = p ? smaps_vmflag(p, "nh") : -1;
	cds_ft_external_arena_destroy(a);
	if (!p)
		return -1;
	if (nh == 0) {		/* RSS arena not advised NOHUGEPAGE */
		fprintf(stderr, "optimize_external_thp: RSS arena not marked nh\n");
		ret = -1;
	}
	return ret;
}

/*
 * The group hint must be semantically inert: a trie built under each
 * cds_ft_optimize value (which drives the internal + compressed arena THP
 * policy) verifies and looks up identically.
 */
static int test_optimize_group_functional(void)
{
	enum cds_ft_optimize opts[2] = {
		CDS_FT_OPTIMIZE_THROUGHPUT, CDS_FT_OPTIMIZE_RSS
	};
	const unsigned int N = 20000;
	int ret = 0, k;

	for (k = 0; k < 2; k++) {
		struct cds_ft_group_attr *attr;
		struct cds_ft_group *group;
		struct cds_ft *ft;
		unsigned int i;

		if (cds_ft_group_attr_create(&attr) < 0)
			return -1;
		cds_ft_group_attr_set_key_len(attr, 3);
		cds_ft_group_attr_set_optimize(attr, opts[k]);
		if (cds_ft_group_create(attr, &group) < 0) {
			cds_ft_group_attr_destroy(attr);
			return -1;
		}
		cds_ft_group_attr_destroy(attr);
		if (cds_ft_create(group, NULL, &ft) < 0) {
			cds_ft_group_destroy(group);
			return -1;
		}
		for (i = 0; i < N; i++) {
			struct ft_test_node *n = node_alloc(i);

			rcu_read_lock();
			if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				node_free(n);
				ret = -1;
				break;
			}
			rcu_read_unlock();
			if ((i & 8191) == 8191)
				rcu_quiescent_state();
		}
		if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK)
			ret = -1;
		rcu_read_lock();
		for (i = 0; i < N; i += 313) {
			struct cds_ft_node *out_node = NULL;

			if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
					to_test_node(out_node)->key != i) {
				ret = -1;
				break;
			}
		}
		rcu_read_unlock();
		drain_and_destroy(ft, group);
	}
	return ret;
}

/*
 * Alloc/free of many slots; verify each pointer is unique and
 * the freelist correctly recycles slots after a free round.
 */
static int test_external_arena_alloc_free_recycle(void)
{
	enum { N = 4096 };
	struct cds_ft_external_arena *a;
	void *p[N];
	void *q[N];
	int ret = -1;

	a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	for (int i = 0; i < N; i++) {
		p[i] = cds_ft_external_arena_alloc(a, 64);
		if (!p[i])
			goto out;
	}
	for (int i = 0; i < N; i++)
		for (int j = i + 1; j < N; j++)
			if (p[i] == p[j])
				goto out;
	for (int i = N - 1; i >= 0; i--)
		cds_ft_external_arena_free(a, p[i]);
	for (int i = 0; i < N; i++) {
		q[i] = cds_ft_external_arena_alloc(a, 64);
		if (!q[i])
			goto out;
	}
	for (int i = 0; i < N; i++)
		for (int j = i + 1; j < N; j++)
			if (q[i] == q[j])
				goto out;
	ret = 0;
out:
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * Cross-class buddy split: free no order-O slots, only one order-O+K
 * slot.  alloc(O) should split the larger free block down and return
 * an order-O slot.  Verify the returned pointer falls inside the
 * larger block's range.
 */
static int test_external_arena_split(void)
{
	struct cds_ft_external_arena *a;
	void *p256_a, *p256_b, *p128;
	uintptr_t base;
	int ret = -1;

	a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	p256_a = cds_ft_external_arena_alloc(a, 256);
	p256_b = cds_ft_external_arena_alloc(a, 256);
	if (!p256_a || !p256_b)
		goto out;
	cds_ft_external_arena_free(a, p256_a);
	cds_ft_external_arena_free(a, p256_b);
	p128 = cds_ft_external_arena_alloc(a, 128);
	if (!p128)
		goto out;
	base = ((uintptr_t) p256_a < (uintptr_t) p256_b)
		? (uintptr_t) p256_a : (uintptr_t) p256_b;
	if ((uintptr_t) p128 < base || (uintptr_t) p128 >= base + 512)
		goto out;
	ret = 0;
out:
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * Cross-class buddy merge: alloc two buddies at order O, free both,
 * then alloc at order O+1 — must return the merged block.
 */
static int test_external_arena_merge(void)
{
	struct cds_ft_external_arena *a;
	void *p128_a, *p64_a, *p64_b, *p128_b;
	int ret = -1;

	a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	/*
	 * Bump-allocated 64 B slots are 64-aligned but not necessarily
	 * 128-aligned, so adjacent 64 B bump allocations may NOT be
	 * buddies at order 7 (their order-7 buddy may land in the
	 * range's header region).  Force a 128-aligned starting offset
	 * by first allocating + freeing a 128 B block: that puts a
	 * 128-aligned 128 B block on freelist[7].  Subsequent 64 B
	 * allocations then come from splitting that block, producing
	 * a true buddy pair at order 6.
	 */
	p128_a = cds_ft_external_arena_alloc(a, 128);
	if (!p128_a)
		goto out;
	cds_ft_external_arena_free(a, p128_a);
	p64_a = cds_ft_external_arena_alloc(a, 64);
	p64_b = cds_ft_external_arena_alloc(a, 64);
	if (!p64_a || !p64_b)
		goto out;
	/* Confirm they are order-6 buddies. */
	if (((uintptr_t) p64_a ^ (uintptr_t) p64_b) != 64)
		goto out;
	cds_ft_external_arena_free(a, p64_a);
	cds_ft_external_arena_free(a, p64_b);
	/* alloc(128) must return the merged block at the original
	 * 128 B address. */
	p128_b = cds_ft_external_arena_alloc(a, 128);
	if (p128_b != p128_a)
		goto out;
	ret = 0;
out:
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * Cross-range allocation: allocate enough to span multiple ranges,
 * then verify pointers are distinct and free returns work across
 * range boundaries.
 */
static int test_external_arena_multi_range(void)
{
	struct cds_ft_external_arena *a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	/* Each range fits ~64 1 MiB slots (after header + guard).
	 * Allocate 200 of them so we span at least 3 ranges. */
	enum { N = 200 };
	void *p[N];
	int ret = -1;

	for (int i = 0; i < N; i++) {
		p[i] = cds_ft_external_arena_alloc(a, 1UL << 20);
		if (!p[i])
			goto out;
	}
	/* All distinct. */
	for (int i = 0; i < N; i++)
		for (int j = i + 1; j < N; j++)
			if (p[i] == p[j])
				goto out;
	/* Free + re-alloc all. */
	for (int i = 0; i < N; i++)
		cds_ft_external_arena_free(a, p[i]);
	for (int i = 0; i < N; i++) {
		p[i] = cds_ft_external_arena_alloc(a, 1UL << 20);
		if (!p[i])
			goto out;
	}
	ret = 0;
out:
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * Pointer alignment matches size class.
 */
static int test_external_arena_alignment(void)
{
	struct cds_ft_external_arena *a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	int ret = -1;
	for (size_t sz = 16; sz <= 4096; sz *= 2) {
		void *p = cds_ft_external_arena_alloc(a, sz);
		if (!p)
			goto out;
		if (((uintptr_t) p) & (sz - 1))
			goto out;
	}
	ret = 0;
out:
	cds_ft_external_arena_destroy(a);
	return ret;
}

/*
 * Allocations exceeding the MAX_ORDER cap (4 MiB) must return NULL.
 */
static int test_external_arena_oversize_reject(void)
{
	struct cds_ft_external_arena *a = cds_ft_external_arena_create(NULL);
	if (!a)
		return -1;
	void *p = cds_ft_external_arena_alloc(a, (1UL << 22) + 1);
	int ret = (p == NULL) ? 0 : -1;
	if (p)
		cds_ft_external_arena_free(a, p);
	cds_ft_external_arena_destroy(a);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                         COMPACTION                                 */
/*                                                                    */
/* ================================================================== */

/*
 * Build a trie, cds_ft_compact() it, and verify the trie stays correct:
 * structural integrity (cds_ft_verify) plus every key still found at its
 * expected value.  Exercises node relocation (including skip targets) and
 * the private-range allocation + merge path.
 */
/*
 * Leak introspection, present only in DEBUG_COUNTERS lib builds (no public
 * header decl).  Weak-referenced so this test links against any lib build:
 * when absent (default build) the compressed-leak assertion is skipped and
 * cds_ft_verify is the structural check; when present (debug-counters) the
 * compressed-node leak is asserted directly.
 */
extern void cds_ft_debug_arena_resident(const struct cds_ft *ft,
		size_t *live_ranges, size_t *reclaimed_ranges,
		size_t *internal_items, size_t *compressed_items,
		size_t *range_bytes) __attribute__((weak));

/*
 * Build a dense 3-byte keyspace into @ft (~N/256 full 256-child nodes).
 * Returns 0 on success.
 */
static int dense_populate(struct cds_ft *ft, unsigned int N)
{
	unsigned int i;

	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "dense_populate: insert %u failed\n", i);
			node_free(n);
			return -1;
		}
		rcu_read_unlock();
		if ((i & 8191) == 8191)
			rcu_quiescent_state();
	}
	return 0;
}

/* Remove every key from @ft (freeing leaves) but keep the trie alive. */
static int dense_drain(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	int ret = 0;

	if (cds_ft_iter_create(ft, &iter) < 0)
		return -1;
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		if (cds_ft_remove_all(ft, iter, &head) < 0) {
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();
	rcu_barrier();		/* drain node_free_rcu + internal-node reclaim */
	cds_ft_iter_destroy(iter);
	return ret;
}

/*
 * Regression: compacting a dense trie whose deepest level forms full 256-child
 * nodes must not corrupt nr_child.  ft_node_recompact rebuilds nr_child from
 * per-child increments, so it must start at 0; a node carved from a recycled
 * range carries a stale nr_child (the far-metadata region is not re-zeroed on
 * range reuse: reclaim only MADV_DONTNEEDs the node body), and a relocated full
 * node would wrap the 9-bit field (stale 256 + 256 copied = 512 -> 0).
 *
 * The trigger needs a *recycled* range, so populate-then-drain a dense keyspace
 * first to leave the group arena holding ranges with stale far-metadata, then
 * rebuild + compact so the relocations reuse them.  A single fresh build allocs
 * only from mmap-zeroed ranges and does not reproduce.  cds_ft_verify catches
 * the nr_child mismatch.
 */
static int test_compact_dense_full_node(void)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * This regression drives ~900k mutations (populate + drain + repopulate
	 * of a dense 300k keyspace).  Under per-mutation verification every one
	 * of them triggers a full-trie cds_ft_verify, turning the run into hours
	 * of O(keys^2) work -- while detecting nothing the test's own end-state
	 * cds_ft_verify (below) does not.  Gate it behind FT_TEST_SLOW so the
	 * VERIFY_AT_MUTATION config is not forced to pay it on every smoke run;
	 * the fast configs always exercise the regression.
	 */
	if (!getenv("FT_TEST_SLOW")) {
		diag("test_compact_dense_full_node: skipped under FEATURE_FT_VERIFY_AT_MUTATION (set FT_TEST_SLOW=1 to force; O(keys^2) per-mutation verify)");
		return 0;
	}
#endif
	/*
	 * N must fill and fully drain at least one far range of order-11
	 * nodes (~1000 nodes, i.e. ~256k dense 3-byte keys) so phase 1 leaves
	 * the arena holding recycled ranges; phase 2 then reuses them.  300k
	 * clears that floor in every node-layout config tested.
	 */
	const char *nenv = getenv("FT_TEST_N");
	const unsigned int N = nenv ? (unsigned int) atoi(nenv) : 300000;
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(3, &group);
	unsigned int i;
	int ret = 0;

	/* Phase 1: populate then drain to recycle stale-metadata ranges. */
	if (dense_populate(ft, N) < 0 || dense_drain(ft) < 0)
		return -1;
	/* Phase 2: rebuild into the recycled ranges, then compact. */
	if (dense_populate(ft, N) < 0)
		return -1;
	/*
	 * On corruption, a wrong nr_child makes iteration loop forever, so
	 * report and bail before any walk (drain_and_destroy included): the
	 * leaked trie is harmless because the test has already failed.
	 */
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact_dense: post-rebuild verify failed (nr_child corruption)\n");
		return -1;
	}
	cds_ft_compact(ft);
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact_dense: post-compaction verify failed (nr_child corruption)\n");
		return -1;
	}
	/* Spot-check that every key still resolves to its node. */
	rcu_read_lock();
	for (i = 0; i < N; i += 997) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != i) {
			fprintf(stderr, "compact_dense: key %u lost after compaction\n", i);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
	drain_and_destroy(ft, group);
	return ret;
}

/*
 * Regression: removing every key must free the compressed (path-edge) nodes,
 * not just the internal branch nodes.  Keys (i << 32) leave a 4-byte zero
 * suffix per leaf, so the build creates ~N compressed nodes.  After removing
 * all keys via cds_ft_remove the trie is empty, so the compressed node arena
 * must drain to zero -- a nonzero residual is the detach-path compressed leak.
 */
static int test_remove_compressed_no_leak(void)
{
	const char *nenv = getenv("FT_TEST_N");
	const unsigned int N = nenv ? (unsigned int) atoi(nenv) : 4096;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_iter *iter = NULL;
	struct ft_test_node **nodes;
	size_t lr = 0, rr = 0, ii = 0, ci_build = 0, ci_after = 0, rb = 0;
	unsigned int i;
	int ret = 0;

	nodes = (struct ft_test_node **) calloc(N, sizeof(*nodes));
	if (!nodes)
		abort();
	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		nodes[i] = node_alloc((uint64_t) i << 32);
		rcu_read_lock();
		if (insert_u64(ft, (uint64_t) i << 32, nodes[i]) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			fprintf(stderr, "remove_compressed: insert %u failed\n", i);
			node_free(nodes[i]);
			ret = -1;
			goto out;
		}
		rcu_read_unlock();
	}
	if (cds_ft_debug_arena_resident)
		cds_ft_debug_arena_resident(ft, &lr, &rr, &ii, &ci_build, &rb);
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "remove_compressed: verify failed after build\n");
		ret = -1;
		goto out;
	}

	if (cds_ft_iter_create(ft, &iter) != CDS_FT_STATUS_OK) {
		ret = -1;
		goto out;
	}
	for (i = 0; i < N; i++) {
		uint8_t k[8];

		cds_ft_u64_to_key(ft, (uint64_t) i << 32, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		if (cds_ft_lookup(ft, iter) == CDS_FT_STATUS_OK)
			(void) cds_ft_remove(ft, iter, &nodes[i]->node);
		rcu_read_unlock();
		node_free_rcu(nodes[i]);
		/* Periodically re-verify the shrinking trie stays well-formed. */
		if ((i & 1023) == 1023 && cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "remove_compressed: verify failed mid-removal at %u\n", i);
			ret = -1;
			goto out;
		}
	}
	rcu_barrier();		/* complete deferred frees */

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "remove_compressed: verify failed on empty trie\n");
		ret = -1;
		goto out;
	}
	if (cds_ft_debug_arena_resident) {
		cds_ft_debug_arena_resident(ft, &lr, &rr, &ii, &ci_after, &rb);
		fprintf(stderr, "remove_compressed: compressed build=%zu after_remove_all=%zu (internal residual=%zu)\n",
			ci_build, ci_after, ii);
		if (ci_after != 0) {
			fprintf(stderr, "remove_compressed: LEAK %zu compressed nodes survived full deletion\n",
				ci_after);
			ret = -1;
		}
	} else {
		fprintf(stderr, "remove_compressed: arena introspection unavailable (build without DEBUG_COUNTERS); cds_ft_verify-only\n");
	}
out:
	if (iter)
		cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	free(nodes);
	return ret;
}

static int test_compact_integrity(void)
{
	const unsigned int N = 4096;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	unsigned int i;
	int ret = 0;

	/* Fixed 8-byte keys so insert_u64/lookup_u64 (CDS_FT_LEN_DEFAULT) apply. */
	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc(i);

		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "compact: insert %u failed\n", i);
			node_free(n);
			ret = -1;
			goto out;
		}
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact: pre-compaction verify failed\n");
		ret = -1;
		goto out;
	}

	cds_ft_compact(ft);

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact: post-compaction verify failed\n");
		ret = -1;
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < N; i++) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != i) {
			fprintf(stderr, "compact: key %u not found after compaction\n",
				i);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
out:
	drain_trie(ft);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Compaction over skip-compressed-over-external-leaf shapes.  Keys of the form
 * (i << 32) share a four-byte zero suffix, so each key's unique tail compresses
 * into a skip-compressed node whose single child is the external leaf.
 * cds_ft_compact must relocate those compressed nodes too -- recovered from the
 * skip pointer via ft_skip_to_compressed, whose external-target back-pointer is
 * cell-indirect -- and keep every key reachable.  A DEBUG_COUNTERS build
 * additionally confirms the extra relocations leave no compressed-node leak
 * once the trie drains (checked at destroy / group destroy).
 */
static int test_compact_skip_over_leaf(void)
{
	const unsigned int N = 1024;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	unsigned int i;
	int ret = 0;

	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc((uint64_t) i << 32);

		if (insert_u64(ft, (uint64_t) i << 32, n) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "compact_skip_over_leaf: insert %u failed\n", i);
			node_free(n);
			ret = -1;
			goto out;
		}
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact_skip_over_leaf: pre-compaction verify failed\n");
		ret = -1;
		goto out;
	}

	cds_ft_compact(ft);

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "compact_skip_over_leaf: post-compaction verify failed\n");
		ret = -1;
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < N; i++) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, (uint64_t) i << 32, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != ((uint64_t) i << 32)) {
			fprintf(stderr, "compact_skip_over_leaf: key %u not found after compaction\n",
				i);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
out:
	drain_trie(ft);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Resumable compaction interleaved with mutations: drive begin/step/end with a
 * tiny batch and insert a fresh key between each step.  Exercises the headline
 * capability -- the cached iterator re-descending by key onto a structure that
 * changed while the read lock was dropped -- then verifies integrity and that
 * every key (original + inserted) survives.
 */
static int test_compact_concurrent_mutation(void)
{
	const unsigned int N = 2000;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_compact_state *st;
	unsigned int i, inserted = 0;
	int ret = 0, more;

	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc(i);

		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			node_free(n);
			ret = -1;
			goto out;
		}
	}

	st = cds_ft_compact_begin(ft);
	if (!st) {
		ret = -1;
		goto out;
	}
	do {
		more = cds_ft_compact_step(st, 1);	/* batch 1 -> many steps */
		/* Mutate between steps (the read lock was just dropped). */
		if (inserted < N) {
			struct ft_test_node *n = node_alloc(N + inserted);

			if (insert_u64(ft, N + inserted, n) == CDS_FT_STATUS_OK)
				inserted++;
			else
				node_free(n);
		}
	} while (more);
	cds_ft_compact_end(st);

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "concurrent compact: verify failed\n");
		ret = -1;
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < N + inserted; i++) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != i) {
			fprintf(stderr, "concurrent compact: key %u missing\n", i);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
out:
	drain_trie(ft);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Compaction of an EXCLUSIVE trie.  Regression for the 2026-06 review's
 * finding 2.14a: the relocation passes freed every old copy through
 * cds_ft_free_item, whose exclusive-mode arm frees SYNCHRONOUSLY -- threading
 * the freelist link through the just-unpublished slot while the step's own
 * walk may still navigate relative to it.  The compactor now defers every
 * old-copy free past a grace period regardless of the trie mode.  Also the
 * first coverage of the exclusive + compact combination.
 */
static int test_compact_exclusive(void)
{
	const unsigned int N = 2000;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_compact_state *st;
	unsigned int i;
	int ret = 0, more;

	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc(i);

		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			node_free(n);
			ret = -1;
			goto out;
		}
	}
	cds_ft_make_exclusive(ft);

	st = cds_ft_compact_begin(ft);
	if (!st) {
		ret = -1;
		goto out_concurrent;
	}
	do {
		more = cds_ft_compact_step(st, 64);
	} while (more);
	cds_ft_compact_end(st);

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "exclusive compact: verify failed\n");
		ret = -1;
		goto out_concurrent;
	}
	/* Mutate post-compaction: exclusive-mode frees must stay coherent. */
	for (i = 0; i < 64; i++) {
		struct ft_test_node *n = node_alloc(N + i);

		if (insert_u64(ft, N + i, n) != CDS_FT_STATUS_OK) {
			node_free(n);
			ret = -1;
			goto out_concurrent;
		}
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "exclusive compact: post-mutation verify failed\n");
		ret = -1;
		goto out_concurrent;
	}
	for (i = 0; i < N + 64; i++) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != i) {
			fprintf(stderr, "exclusive compact: key %u missing\n", i);
			ret = -1;
			break;
		}
	}
out_concurrent:
	cds_ft_make_concurrent(ft);
	rcu_barrier();	/* flush the compactor's deferred old-copy frees */
out:
	drain_trie(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * Forgotten cds_ft_compact_end: start a compaction, run a couple of steps
 * (partial), then never call _end.  cds_ft_destroy must finalize the abandoned
 * compaction (merge its private ranges, free its state) -- no crash, no leak.
 * The "compaction still in progress" warning on stderr at destroy is expected.
 */
static int test_compact_forgotten_end(void)
{
	const unsigned int N = 2000;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct cds_ft_compact_state *st;
	unsigned int i;
	int ret = 0;

	ft = create_fixed_ft(8, &group);
	for (i = 0; i < N; i++) {
		struct ft_test_node *n = node_alloc(i);

		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			node_free(n);
			ret = -1;
			goto out;
		}
	}
	st = cds_ft_compact_begin(ft);
	if (!st) {
		ret = -1;
		goto out;
	}
	(void) cds_ft_compact_step(st, 8);	/* partial: do not run to completion */
	(void) cds_ft_compact_step(st, 8);
	/* Intentionally NO cds_ft_compact_end(st) -- destroy must finalize it. */
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "forgotten-end: verify failed mid-compaction\n");
		ret = -1;
		goto out;
	}
	rcu_read_lock();
	for (i = 0; i < N; i++) {
		struct cds_ft_node *out_node = NULL;

		if (lookup_u64(ft, i, &out_node) != CDS_FT_STATUS_OK ||
				to_test_node(out_node)->key != i) {
			fprintf(stderr, "forgotten-end: key %u missing\n", i);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
out:
	drain_trie(ft);
	cds_ft_destroy(ft);	/* finalizes the un-ended compaction */
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                           MAIN                                     */
/*                                                                    */
/* ================================================================== */

#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_alloc_countdown;

/*
 * Drive a compressed-split insert through each of its allocation-failure
 * points while a compressed node is live, and assert the trie stays
 * structurally consistent after every failed split (cds_ft_verify checks
 * each node's parent back-pointer).  Catches the bug where the split
 * publishes a live node's back-pointer mid-build and, on a later OOM, frees
 * the transiently-observable new node without restoring it: no back-pointer
 * into the cluster may be written until the failure-free commit tail.
 *
 * The base trie is compressed("aaaaaaa") -> internal {a, b}.  @dkey selects
 * which split path the insert exercises:
 *   - diverge inside the path (suffix_len >= 1): a new suffix node wraps the
 *     old child; the cluster-leaf is that suffix node.
 *   - diverge at the last path byte (suffix_len == 0): the branch itself is
 *     the cluster-leaf (its old direction points straight at the live child).
 */
static int run_split_oom_insert(const char *label,
		const uint8_t *dkey, size_t dlen)
{
	int n, rc = 0;

	for (n = 0; n < 16; n++) {
		struct cds_ft_group *group;
		struct cds_ft *ft = create_varlen_ft(&group);
		struct ft_test_node *a = node_alloc(1);
		struct ft_test_node *b = node_alloc(2);
		struct ft_test_node *c = node_alloc(3);
		enum cds_ft_status s;
		int verified;

		/* Build compressed("aaaaaaa") -> internal child {a, b}. */
		if (cds_ft_insert(ft, (const uint8_t *)"aaaaaaaa", 8, &a->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *)"aaaaaaab", 8, &b->node) < 0) {
			fprintf(stderr, "split_oom[%s]: build failed\n", label);
			rc = -1;
		}

		/* Fail the (n+1)-th allocation performed by the split. */
		cds_ft_fault_alloc_countdown = n;
		s = cds_ft_insert(ft, dkey, dlen, &c->node);
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"split_oom[%s]: verify FAILED after fault n=%d (insert=%s)\n",
				label, n, cds_ft_status_to_string(s));
			rc = -1;
			/*
			 * The trie is corrupt; draining it would follow the
			 * dangling back-pointer and livelock.  Abandon (leak)
			 * this iteration's trie -- the test has already failed.
			 */
			continue;
		}

		if (s != CDS_FT_STATUS_OK)
			node_free(c);	/* OOM: never inserted, never observable */
		if (drain_and_destroy(ft, group) < 0)
			rc = -1;
	}
	return rc;
}

/*
 * As run_split_oom_insert, but the compressed split is driven by cds_ft_graft:
 * grafting a staging trie at @gkey, which diverges inside the live trie's
 * compressed path, runs the build-invisible diverge split + inline payload
 * attach (ft_split_compressed_graft_build).  The ENTIRE attach cluster — the
 * split rearrangement AND the grafted payload — is built from fresh nodes
 * before the source root is unlinked, so on any allocation failure both tries
 * must still pass cds_ft_verify (the cluster is freed, nothing published,
 * nothing rolled back).  @nr_faults sweeps the whole allocation window
 * (fresh-root + every node in the cluster), exercising the graft transaction's
 * atomicity end to end — not just the split's own allocations.
 */
static int run_split_oom_graft(const char *label,
		const uint8_t *gkey, size_t glen, int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *live = create_varlen_ft(&group);
		struct cds_ft *staging;
		struct ft_test_node *a = node_alloc(1);
		struct ft_test_node *b = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		enum cds_ft_status s;
		int verified;

		if (cds_ft_create(group, NULL, &staging) < 0) {
			fprintf(stderr, "split_oom[%s]: staging create failed\n", label);
			return -1;
		}

		/* live: compressed("aaaaaaa") -> internal {a, b}. */
		if (cds_ft_insert(live, (const uint8_t *)"aaaaaaaa", 8, &a->node) < 0 ||
		    cds_ft_insert(live, (const uint8_t *)"aaaaaaab", 8, &b->node) < 0)
			rc = -1;
		/* staging: one key, becomes the payload under the graft point. */
		if (cds_ft_insert(staging, (const uint8_t *)"z", 1, &s1->node) < 0)
			rc = -1;

		/* Fail the (n+1)-th allocation performed by the graft. */
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_graft(live, gkey, glen, staging);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		/*
		 * Verify BOTH tries: a failed (OOM) graft must leave the
		 * source pristine too, not just the destination.
		 */
		rcu_read_lock();
		verified = (cds_ft_verify(live, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(staging, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"split_oom[%s]: verify FAILED after fault n=%d (graft=%s)\n",
				label, n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}

		/*
		 * On success the payload moved to live; on OOM the graft rolled
		 * back and staging still owns it.  Drain both either way.
		 */
		if (drain_trie(live) < 0 || drain_trie(staging) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(live);
		cds_ft_destroy(staging);
		rcu_barrier();	/* flush destroy's deferred frees before the leak check */
		cds_ft_group_destroy(group);
	}
	return rc;
}

/* Exact-key presence check for a NUL-terminated string key. */
static int graft_swap_oom_has_key(struct cds_ft *ft, const char *k)
{
	struct cds_ft_node *out = NULL;
	size_t len = strlen(k);

	return cds_ft_eager_lookup_key(ft, (const uint8_t *) k, len, 0,
			&out) == CDS_FT_STATUS_OK;
}

/*
 * cds_ft_graft into a NOSPLIT graft point (a free sibling slot under an existing
 * internal -- here "az" beside "ax"/"ay") under allocation fault injection.
 * This is the path that SELF-SECURES its commit: a generous node reserve + a
 * flip batch are drawn BEFORE the source-root swap, so the post-swap store
 * cannot fail and there is NO source-root rollback.  A fault therefore can only
 * hit the pre-swap fresh-root / reserve-fill / flip allocation, leaving BOTH
 * tries PRISTINE (staging still owns the payload); once those succeed the store
 * always commits ("az" appears in live, staging empties).  Pre-fix this path
 * re-published the source root on a store OOM -- a flicker (source empty, then
 * full again) a concurrent reader could observe; it can no longer occur, and the
 * in-code assert(status == OK) would fire here if the reserve under-counted.
 * The reserve fill is alloc-heavy, so the sweep is wide enough to reach the
 * committing case too.
 */
static int run_graft_oom_nosplit(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *live = create_varlen_ft(&group);
		struct cds_ft *staging;
		struct ft_test_node *a = node_alloc(1);
		struct ft_test_node *b = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &staging) < 0) {
			fprintf(stderr, "graft_oom_nosplit: staging create failed\n");
			return -1;
		}
		if (cds_ft_insert(live, (const uint8_t *)"ax", 2, &a->node) < 0 ||
		    cds_ft_insert(live, (const uint8_t *)"ay", 2, &b->node) < 0 ||
		    cds_ft_insert(staging, (const uint8_t *)"z", 1, &s1->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_graft(live, (const uint8_t *)"az", 2, staging);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(live, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(staging, stderr) == CDS_FT_STATUS_OK);
		/*
		 * "ax"/"ay" always survive in live.  Graft prepends prefix "az" to
		 * staging's key "z", so a successful move yields "azz" in live.
		 */
		keys_ok = graft_swap_oom_has_key(live, "ax") &&
			graft_swap_oom_has_key(live, "ay");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(live, "azz") &&
				!graft_swap_oom_has_key(staging, "z");
		} else {
			/* Pristine: payload still in staging, "azz" absent from live. */
			keys_ok = keys_ok &&
				!graft_swap_oom_has_key(live, "azz") &&
				graft_swap_oom_has_key(staging, "z");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"graft_oom_nosplit: %s after fault n=%d (graft=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(live) < 0 || drain_trie(staging) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(live);
		cds_ft_destroy(staging);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

/*
 * As run_split_oom_graft, but for cds_ft_graft_swap.  A swap is two coupled
 * clusters: (A) swap_ft's content, canonicalized, inserted at the graft point
 * in place of the displaced dst old-child; and (B) that displaced old-child
 * materialized as swap_ft's new root (ft_make_root_internal_glue).  Both must be
 * built from fresh nodes BEFORE swap_ft's root is unlinked and a grace period
 * taken, so any allocation failure leaves BOTH tries pristine (MEMORY_ERROR,
 * nothing published, nothing lost).
 *
 * @live_keys / @nr_live build the dst trie; swap_ft always holds the single
 * key "Z" (a 1-child internal root, exercising the insert-side
 * canonicalization).  @gkey / @glen select the graft point: an exact existing
 * node, or a point strictly inside a compressed path (key-shorter split).
 *
 * The fault sweep covers the whole allocation window.  Pre-fix it exposes two
 * hazards: (a) an insert-cluster OOM AFTER the swap unlink+sync leaves swap
 * emptied and the dst content orphaned (DATA LOSS — caught by the key-presence
 * check below, which structural verify alone would miss), and (b)
 * ft_make_root_internal OOM calls abort() (SIGABRT).  Post-fix every OOM
 * returns MEMORY_ERROR with both tries verifiable and pristine.
 */
static int run_split_oom_graft_swap(const char *label,
		const char *const *live_keys, int nr_live,
		const uint8_t *gkey, size_t glen, int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *live = create_varlen_ft(&group);
		struct cds_ft *swap;
		struct ft_test_node *zn;
		enum cds_ft_status s;
		int i, verified, pristine_ok = 1;

		if (cds_ft_create(group, NULL, &swap) < 0) {
			fprintf(stderr, "graft_swap_oom[%s]: swap create failed\n", label);
			return -1;
		}

		for (i = 0; i < nr_live; i++) {
			struct ft_test_node *nd = node_alloc((uint64_t) i);

			if (cds_ft_insert(live, (const uint8_t *) live_keys[i],
					strlen(live_keys[i]), &nd->node) < 0) {
				node_free(nd);
				rc = -1;
			}
		}
		zn = node_alloc(1000);
		if (cds_ft_insert(swap, (const uint8_t *) "Z", 1, &zn->node) < 0) {
			node_free(zn);
			rc = -1;
		}

		/* Fail the (n+1)-th allocation performed by the swap. */
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_graft_swap(live, gkey, glen, swap);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(live, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(swap, stderr) == CDS_FT_STATUS_OK);
		/*
		 * On OOM the swap is a no-op: live keeps every key and swap
		 * still owns "Z".  A vanished key here is the data-loss hazard
		 * (swap emptied, dst content orphaned) that structural verify
		 * does not catch.  Only walk the trie when it is structurally
		 * sound — a failed verify means the structure is corrupt and a
		 * lookup would chase a dangling pointer.
		 */
		if (verified && s != CDS_FT_STATUS_OK) {
			for (i = 0; i < nr_live; i++)
				if (!graft_swap_oom_has_key(live, live_keys[i]))
					pristine_ok = 0;
			if (!graft_swap_oom_has_key(swap, "Z"))
				pristine_ok = 0;
		}
		rcu_read_unlock();
		if (!verified || !pristine_ok) {
			fprintf(stderr,
				"graft_swap_oom[%s]: %s after fault n=%d (graft_swap=%s)\n",
				label,
				!verified ? "verify FAILED" : "DATA LOST (key vanished)",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}

		if (drain_trie(live) < 0 || drain_trie(swap) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(live);
		cds_ft_destroy(swap);
		rcu_barrier();	/* flush destroy's deferred frees before the leak check */
		cds_ft_group_destroy(group);
	}
	return rc;
}

/*
 * Fault-injection regression for the compressed-split create-cluster-then-
 * publish discipline.  Requires the alloc fault hook (FEATURE_FT_FAULT_INJECT).
 */
static int test_split_oom_backpointer(void)
{
	int rc = 0;

	/* Diverge inside the compressed path: suffix_len >= 1. */
	if (run_split_oom_insert("insert-suffix>=1",
			(const uint8_t *)"aaaXaaaa", 8) < 0)
		rc = -1;
	/*
	 * Diverge at the last path byte ('a' at index 6 of "aaaaaaa" vs 'X'):
	 * suffix_len == 0, so the branch is the cluster-leaf with the live old
	 * child on its old direction and a fresh "mas" -> leaf on the new one.
	 */
	if (run_split_oom_insert("insert-suffix==0",
			(const uint8_t *)"aaaaaaXmas", 10) < 0)
		rc = -1;
	/*
	 * Key shorter than the compressed path: ft_split_compressed_key_shorter.
	 * "aaa" terminates at depth 3 (suffix_len == 3: a new compressed suffix
	 * wraps the old child); "aaaaaa" at depth 6 (suffix_len == 0: the
	 * junction itself is the cluster-leaf, holding the live old child).
	 */
	if (run_split_oom_insert("key_shorter-suffix>=1",
			(const uint8_t *)"aaa", 3) < 0)
		rc = -1;
	/*
	 * "aaaaa" terminates at depth 5 (suffix_len == 1): under SKIP_COMPRESSED
	 * a 1-byte compressed suffix; under non-SC a 1-child internal suffix that
	 * is itself the cluster-leaf holding the live old child.
	 */
	if (run_split_oom_insert("key_shorter-suffix==1",
			(const uint8_t *)"aaaaa", 5) < 0)
		rc = -1;
	if (run_split_oom_insert("key_shorter-suffix==0",
			(const uint8_t *)"aaaaaa", 6) < 0)
		rc = -1;
	/*
	 * Graft diverging inside the compressed path: the build-invisible
	 * diverge split + inline payload attach.  "aaaXmas" diverges at index 3
	 * (suffix_len == 3: compressed suffix wraps the old child, payload sits
	 * under a "mas" path); "aaaaaaX" at index 6 (suffix_len == 0: the branch
	 * is the cluster-leaf holding the live old child, payload attaches
	 * directly).  The fault sweep covers the WHOLE allocation window: the
	 * graft transaction is atomic, so every OOM leaves both tries verifiable.
	 */
	if (run_split_oom_graft("graft-suffix>=1",
			(const uint8_t *)"aaaXmas", 7, 16) < 0)
		rc = -1;
	if (run_split_oom_graft("graft-suffix==0",
			(const uint8_t *)"aaaaaaX", 7, 16) < 0)
		rc = -1;
	/*
	 * Graft into a NOSPLIT graft point (free sibling slot): the self-secured
	 * commit path.  Sweep wide enough to cross the reserve fill (alloc-heavy)
	 * into the committing case -- every fault leaves both tries pristine or
	 * fully grafted, never a re-published (flickered) source root.
	 */
	if (run_graft_oom_nosplit(120) < 0)
		rc = -1;
	/*
	 * graft_swap, key-shorter graft point: "aaX" lands strictly inside the
	 * compressed path "aaXcdef" (cn -> internal {g, h}).  The graft point is
	 * the prefix/suffix boundary; the displaced old-child is the compressed
	 * suffix "cdef", so the insert side canonicalizes + chain-merges the swap
	 * content into the prefix AND the extract side runs ft_make_root_internal
	 * on a compressed old-child.  Pre-fix: insert-cluster OOM loses data, then
	 * ft_make_root_internal OOM aborts.
	 */
	{
		static const char *const ks_keys[] = {
			"aaXcdefg", "aaXcdefh",
		};

		if (run_split_oom_graft_swap("graft_swap-key_shorter",
				ks_keys, 2, (const uint8_t *)"aaX", 3, 16) < 0)
			rc = -1;
	}
	/*
	 * graft_swap, exact graft point: "aaX" reaches an existing node (the X
	 * child of the branch under "aa"), whose subtree is the compressed
	 * "cdef" -> internal {g, h}.  The displaced old-child is that compressed
	 * node, so the extract side again runs ft_make_root_internal; the insert
	 * side lands at a (non-compressed) branch slot, so no chain-merge.
	 */
	{
		static const char *const ex_keys[] = {
			"aaXcdefg", "aaXcdefh", "aaWxyz",
		};

		if (run_split_oom_graft_swap("graft_swap-exact",
				ex_keys, 3, (const uint8_t *)"aaX", 3, 16) < 0)
			rc = -1;
	}
	/*
	 * graft_swap, exact graft point UNDER a compressed parent: "aaX" reaches
	 * the multi-child internal {p, q} whose parent is the compressed node
	 * "aaX".  With a 1-child swap content ("Z" canonicalizes to a compressed),
	 * the insert side fuses the compressed parent with the compressed swap
	 * content into one node (the build-invisible merged_cn path under
	 * SKIP_COMPRESSED) -- a fallible allocation that, pre-fix, ran after the
	 * swap unlink+sync and lost data on OOM.  Under non-SC / nocompress this
	 * degrades to a plain replace (still a valid scenario).
	 */
	{
		static const char *const merge_keys[] = {
			"aaXp", "aaXq",
		};

		if (run_split_oom_graft_swap("graft_swap-exact-merge",
				merge_keys, 2, (const uint8_t *)"aaX", 3, 16) < 0)
			rc = -1;
	}
	return rc;
}

/*
 * OOM coverage for cds_ft_merge_at's build-invisible spine-copy.  All of src
 * is merged at its root into a non-empty dst whose top-level byte ('b') is
 * DISJOINT from src's ('a'), so ft_merge_build copies only the merge-point
 * root and REFERENCES both sides' subtrees: the spine-copy path is taken under
 * every build (no compressed overlap forces a per-entry delegation), and the
 * pristine guarantee below holds uniformly.
 *
 * The fault sweep covers the whole build window.  The merge is build-invisible,
 * so every OOM must leave BOTH tries pristine -- dst keeps exactly its own keys,
 * src keeps exactly its own keys (no detach, no rollback, no leak).  On success
 * dst holds the union and src is empty.
 */
static int run_merge_oom(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *d2 = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom: src create failed\n");
			return -1;
		}
		/* dst: "ax","ay"; src: "bx","by" -- disjoint top byte. */
		if (cds_ft_insert(dst, (const uint8_t *)"ax", 2, &d1->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *)"ay", 2, &d2->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"bx", 2, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"by", 2, &s2->node) < 0)
			rc = -1;

		/* Fail the (n+1)-th allocation performed by the merge. */
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge(dst, NULL, 0, src);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "ax") &&
				graft_swap_oom_has_key(dst, "ay") &&
				graft_swap_oom_has_key(dst, "bx") &&
				graft_swap_oom_has_key(dst, "by") &&
				!graft_swap_oom_has_key(src, "bx") &&
				!graft_swap_oom_has_key(src, "by");
		} else {
			/* OOM: both tries pristine. */
			keys_ok = graft_swap_oom_has_key(dst, "ax") &&
				graft_swap_oom_has_key(dst, "ay") &&
				!graft_swap_oom_has_key(dst, "bx") &&
				!graft_swap_oom_has_key(dst, "by") &&
				graft_swap_oom_has_key(src, "bx") &&
				graft_swap_oom_has_key(src, "by");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom(void)
{
	return run_merge_oom(12);
}

/*
 * OOM coverage for the EMPTY-dst-ROOT merge path (cds_ft_merge_at into a @dst
 * that is empty at the merge point).  Unlike the spine-copy path, this moves
 * the source subtree in via a detach plus a FAILURE-FREE root swap, with the
 * source's replacement root pre-allocated up front, so nothing fallible follows
 * the detach and there is no rollback to strand the moved externals.  Every OOM
 * must therefore leave BOTH tries pristine -- @dst stays empty, @src keeps its
 * keys -- with no leak (RUN_TEST's leak_check catches a stranded external); on
 * success @dst holds src's keys and @src is empty.  The sweep covers both a
 * whole-trie src (src_key_len 0, EXACT at root) and a sub-prefix src
 * (src_key_len > 0, exercising the detach re-rooting that feeds the swap).
 */
static int run_merge_oom_empty_dst(int nr_faults)
{
	int n, rc = 0;
	unsigned int shape;

	/* shape 0: whole src ("bx","by", src_key NIL); 1: sub-prefix ("p"). */
	for (shape = 0; shape < 2; shape++) {
		const char *k1 = shape ? "px" : "bx";
		const char *k2 = shape ? "py" : "by";
		const uint8_t *src_key = shape ? (const uint8_t *) "p" : NULL;
		size_t src_key_len = shape ? 1 : 0;
		/* moved keys as they land in the (empty, NIL-prefix) dst. */
		const char *m1 = shape ? "x" : "bx";
		const char *m2 = shape ? "y" : "by";

		for (n = 0; n < nr_faults; n++) {
			struct cds_ft_group *group;
			struct cds_ft *dst = create_varlen_ft(&group); /* empty */
			struct cds_ft *src;
			struct ft_test_node *s1 = node_alloc(1);
			struct ft_test_node *s2 = node_alloc(2);
			enum cds_ft_status s;
			int verified, keys_ok;

			if (cds_ft_create(group, NULL, &src) < 0) {
				fprintf(stderr,
					"merge_oom_empty_dst: src create failed\n");
				return -1;
			}
			if (cds_ft_insert(src, (const uint8_t *) k1, 2,
					&s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *) k2, 2,
					&s2->node) < 0)
				rc = -1;

			/* Fail the (n+1)-th allocation performed by the merge. */
			cds_ft_fault_alloc_countdown = n;
			rcu_read_lock();
			s = cds_ft_merge_at(dst, NULL, 0, src, src_key,
					src_key_len);
			rcu_read_unlock();
			cds_ft_fault_alloc_countdown = -1;

			rcu_read_lock();
			verified = (cds_ft_verify(dst, stderr) ==
					CDS_FT_STATUS_OK) &&
				(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
			if (s == CDS_FT_STATUS_OK) {
				keys_ok = graft_swap_oom_has_key(dst, m1) &&
					graft_swap_oom_has_key(dst, m2) &&
					!graft_swap_oom_has_key(src, k1) &&
					!graft_swap_oom_has_key(src, k2);
			} else {
				/* OOM: both pristine -- dst empty, src intact. */
				keys_ok = !graft_swap_oom_has_key(dst, m1) &&
					!graft_swap_oom_has_key(dst, m2) &&
					graft_swap_oom_has_key(src, k1) &&
					graft_swap_oom_has_key(src, k2);
			}
			rcu_read_unlock();
			if (!verified || !keys_ok) {
				fprintf(stderr,
					"merge_oom_empty_dst: %s after fault n=%d shape=%u (merge=%s)\n",
					!verified ? "verify FAILED" :
						"KEY SET WRONG",
					n, shape, cds_ft_status_to_string(s));
				rc = -1;
				continue;	/* corrupt: abandon this iteration */
			}

			if (drain_trie(dst) < 0 || drain_trie(src) < 0)
				rc = -1;
			rcu_barrier();
			cds_ft_destroy(dst);
			cds_ft_destroy(src);
			rcu_barrier();
			cds_ft_group_destroy(group);
		}
	}
	return rc;
}

static int test_merge_oom_empty_dst(void)
{
	return run_merge_oom_empty_dst(8);
}

/*
 * OOM coverage for the diverged-dst WHOLE-SOURCE merge (src_key_len == 0 into a
 * @dst that is populated but has no key with @dst_key as a prefix -- the
 * "rekey to a fresh prefix" shape).  This takes the direct-graft path (the
 * source trie is the payload; ft_graft is leak-free on its own), so every OOM
 * must leave both tries pristine with no stranded external (RUN_TEST's
 * leak_check).  Two dst layouts exercise a NOSPLIT diverge (an empty slot on a
 * multi-child root) and a GLUE diverge (inside a compressed run).
 */
static int run_merge_oom_diverged_dst(int nr_faults)
{
	static const struct {
		const char *dkeys[2];
		unsigned int ndk;
		const char *mkey;
	} layouts[] = {
		{ { "ax", "zx" }, 2, "m" },	/* 'm' is an empty root slot */
		{ { "max", NULL }, 1, "mb" },	/* 'mb' diverges inside "max" */
	};
	unsigned int L;
	int rc = 0;

	for (L = 0; L < 2; L++) {
		const char *mkey = layouts[L].mkey;
		size_t mklen = strlen(mkey);
		char m1[16], m2[16];
		unsigned int j;
		int n;

		snprintf(m1, sizeof m1, "%sbx", mkey);
		snprintf(m2, sizeof m2, "%sby", mkey);

		for (n = 0; n < nr_faults; n++) {
			struct cds_ft_group *group;
			struct cds_ft *dst = create_varlen_ft(&group);
			struct cds_ft *src;
			struct ft_test_node *s1 = node_alloc(1);
			struct ft_test_node *s2 = node_alloc(2);
			enum cds_ft_status s;
			int verified, keys_ok = 1;

			if (cds_ft_create(group, NULL, &src) < 0) {
				fprintf(stderr,
					"merge_oom_diverged_dst: src create\n");
				return -1;
			}
			for (j = 0; j < layouts[L].ndk; j++) {
				struct ft_test_node *dn = node_alloc(100 + j);
				if (cds_ft_insert(dst,
						(const uint8_t *) layouts[L].dkeys[j],
						strlen(layouts[L].dkeys[j]),
						&dn->node) < 0) {
					node_free(dn);
					rc = -1;
				}
			}
			if (cds_ft_insert(src, (const uint8_t *) "bx", 2,
					&s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *) "by", 2,
					&s2->node) < 0)
				rc = -1;

			cds_ft_fault_alloc_countdown = n;
			rcu_read_lock();
			s = cds_ft_merge_at(dst, (const uint8_t *) mkey, mklen,
					src, NULL, 0);
			rcu_read_unlock();
			cds_ft_fault_alloc_countdown = -1;

			rcu_read_lock();
			verified = (cds_ft_verify(dst, stderr) ==
					CDS_FT_STATUS_OK) &&
				(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
			/* dst keeps its own keys either way. */
			for (j = 0; j < layouts[L].ndk; j++)
				keys_ok = keys_ok &&
					graft_swap_oom_has_key(dst,
						layouts[L].dkeys[j]);
			if (s == CDS_FT_STATUS_OK) {
				keys_ok = keys_ok &&
					graft_swap_oom_has_key(dst, m1) &&
					graft_swap_oom_has_key(dst, m2) &&
					!graft_swap_oom_has_key(src, "bx") &&
					!graft_swap_oom_has_key(src, "by");
			} else {
				/* OOM: src intact, dst gained nothing. */
				keys_ok = keys_ok &&
					!graft_swap_oom_has_key(dst, m1) &&
					!graft_swap_oom_has_key(dst, m2) &&
					graft_swap_oom_has_key(src, "bx") &&
					graft_swap_oom_has_key(src, "by");
			}
			rcu_read_unlock();
			if (!verified || !keys_ok) {
				fprintf(stderr,
					"merge_oom_diverged_dst: %s after fault n=%d L=%u (merge=%s)\n",
					!verified ? "verify FAILED" :
						"KEY SET WRONG",
					n, L, cds_ft_status_to_string(s));
				rc = -1;
				continue;
			}

			if (drain_trie(dst) < 0 || drain_trie(src) < 0)
				rc = -1;
			rcu_barrier();
			cds_ft_destroy(dst);
			cds_ft_destroy(src);
			rcu_barrier();
			cds_ft_group_destroy(group);
		}
	}
	return rc;
}

static int test_merge_oom_diverged_dst(void)
{
	return run_merge_oom_diverged_dst(10);
}

/*
 * OOM coverage for the SUB-position residual reserve fast path: a multi-child
 * internal source subtree (src@"ca", holding "cax"/"cay") moved to an empty
 * slot of an existing dst node (dst_key "zc", dst holding "za"/"zb").  This is
 * the src_key_len > 0 + at-node-empty dst shape, which cds_ft_merge_at handles
 * by pre-reserving graft's node allocations and drawing them, so graft cannot
 * fail on an arena allocation.  Consequence under single-shot fault injection:
 * a fault during the reserve fill or the detach makes merge_at fail with BOTH
 * tries pristine; a fault armed past those (where the legacy path would fail
 * mid-graft and roll back) instead never fires inside graft (its allocations
 * draw from the reserve, above the fault hook), so the move SUCCEEDS.  Either
 * way leak_check sees no stranded external, and the reserve's completeness
 * assert (abort on a drawn-but-empty bucket) proves the manifest is complete.
 */
static int run_merge_oom_subpos_residual(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *d2 = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_subpos: src create failed\n");
			return -1;
		}
		/*
		 * dst: "za","zb" (root -> "z" -> {a,b}); src: "cax","cay"
		 * (src@"ca" is a 2-child internal).  merge_at at "zc" lands on
		 * the empty 'c' slot of dst's {a,b} node.
		 */
		if (cds_ft_insert(dst, (const uint8_t *)"za", 2, &d1->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *)"zb", 2, &d2->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cax", 3, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cay", 3, &s2->node) < 0)
			rc = -1;

		/* Fail the (n+1)-th allocation performed by the merge. */
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)"zc", 2,
				src, (const uint8_t *)"ca", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		/* dst keeps its own keys either way. */
		keys_ok = graft_swap_oom_has_key(dst, "za") &&
			graft_swap_oom_has_key(dst, "zb");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(dst, "zcx") &&
				graft_swap_oom_has_key(dst, "zcy") &&
				!graft_swap_oom_has_key(src, "cax") &&
				!graft_swap_oom_has_key(src, "cay");
		} else {
			/* OOM during fill/detach: both tries pristine. */
			keys_ok = keys_ok &&
				!graft_swap_oom_has_key(dst, "zcx") &&
				!graft_swap_oom_has_key(dst, "zcy") &&
				graft_swap_oom_has_key(src, "cax") &&
				graft_swap_oom_has_key(src, "cay");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_subpos: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_subpos_residual(void)
{
	return run_merge_oom_subpos_residual(12);
}

/*
 * OOM coverage for the sub-position residual GLUE shape: dst_key "mb" diverges
 * INSIDE dst's compressed "mango" run, so graft builds a split cluster.
 * cds_ft_merge_at AUTO-LEARNS that cluster's node manifest (a build-and-abort
 * pass over graft's invisible prep, mirroring its built[] into the reserve),
 * so the real graft draws every cluster node and cannot fail on an arena
 * allocation.  As for the at-node case, every injected OOM hits the learn
 * build, reserve fill, or detach -- all leaving BOTH tries pristine -- or is
 * armed past them and the move succeeds.  The completeness assert (abort on a
 * drawn-but-empty bucket) proves the auto-learned manifest is complete.
 */
static int run_merge_oom_subpos_glue(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_subpos_glue: src create failed\n");
			return -1;
		}
		/*
		 * dst: single key "mango" (root -> compressed "mango"); src:
		 * "cax","cay" (src@"ca" a 2-child internal).  merge_at at "mb"
		 * diverges inside "mango" at offset 1 -> GLUE split.
		 */
		if (cds_ft_insert(dst, (const uint8_t *)"mango", 5, &d1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cax", 3, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cay", 3, &s2->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)"mb", 2,
				src, (const uint8_t *)"ca", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		keys_ok = graft_swap_oom_has_key(dst, "mango");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(dst, "mbx") &&
				graft_swap_oom_has_key(dst, "mby") &&
				!graft_swap_oom_has_key(src, "cax") &&
				!graft_swap_oom_has_key(src, "cay");
		} else {
			keys_ok = keys_ok &&
				!graft_swap_oom_has_key(dst, "mbx") &&
				!graft_swap_oom_has_key(dst, "mby") &&
				graft_swap_oom_has_key(src, "cax") &&
				graft_swap_oom_has_key(src, "cay");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_subpos_glue: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_subpos_glue(void)
{
	return run_merge_oom_subpos_glue(16);
}

/*
 * OOM coverage for the sub-position residual BUILD-A-BRANCH shape: dst_key
 * "mxyz" extends past dst's structure (dst holds only "m"), so the descent
 * stops short and graft builds an intermediate branch that also absorbs the
 * displaced "m" external.  cds_ft_merge_at auto-learns the branch's node
 * manifest by running ft_build_branch as a learn pass (it builds into the glue
 * with no dst mutation) and mirroring its built[] into the reserve, so the real
 * graft draws every branch node.  As before, every injected OOM hits the learn
 * build / reserve fill / detach (BOTH tries pristine) or is armed past them and
 * the move succeeds; the completeness assert proves the manifest is complete.
 */
static int run_merge_oom_subpos_branch(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_subpos_branch: src create failed\n");
			return -1;
		}
		/*
		 * dst: single key "m"; src: "cax","cay" (src@"ca" a 2-child
		 * internal).  merge_at at "mxyz" stops short at depth 1 on the
		 * displaced "m" external -> build-a-branch.
		 */
		if (cds_ft_insert(dst, (const uint8_t *)"m", 1, &d1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cax", 3, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cay", 3, &s2->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)"mxyz", 4,
				src, (const uint8_t *)"ca", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		keys_ok = graft_swap_oom_has_key(dst, "m");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(dst, "mxyzx") &&
				graft_swap_oom_has_key(dst, "mxyzy") &&
				!graft_swap_oom_has_key(src, "cax") &&
				!graft_swap_oom_has_key(src, "cay");
		} else {
			keys_ok = keys_ok &&
				!graft_swap_oom_has_key(dst, "mxyzx") &&
				!graft_swap_oom_has_key(dst, "mxyzy") &&
				graft_swap_oom_has_key(src, "cax") &&
				graft_swap_oom_has_key(src, "cay");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_subpos_branch: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_subpos_branch(void)
{
	return run_merge_oom_subpos_branch(16);
}

/*
 * OOM coverage for the RE-ROOTED source, GLUE diverge dst: a sub-position
 * source whose subtree root is EXTERNAL (@shape 0: src@"a", a single key) or
 * COMPRESSED (@shape 1: src@"x", a compressed "ab" run), merged at "mb" which
 * diverges inside dst's compressed "mango".  These shapes used to fall to the
 * leaky detach-then-graft path because detach RE-ROOTS the payload; instead
 * cds_ft_merge_at now grafts the subtree IN PLACE (no re-root) so the GLUE
 * cluster is built invisibly referencing it and the source unlink is the last
 * fallible step.  Every injected OOM hits the cluster build or the unlink (both
 * leaving the tries pristine), or is armed past them and the move succeeds; the
 * external shape additionally exercises the external-payload count propagation
 * (attached_nf is the external itself).  RUN_TEST's leak_check catches any
 * stranded external on the no-rollback property.
 */
static int run_merge_oom_rerooted_glue(int nr_faults, int shape)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_rerooted_glue: src create failed\n");
			return -1;
		}
		if (cds_ft_insert(dst, (const uint8_t *)"mango", 5, &d1->node) < 0)
			rc = -1;
		if (shape == 0) {
			/* src@"a" is an EXTERNAL leaf; "b" stays behind. */
			if (cds_ft_insert(src, (const uint8_t *)"a", 1, &s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *)"b", 1, &s2->node) < 0)
				rc = -1;
		} else {
			/* src@"x" is a COMPRESSED "ab" run over {c,d}. */
			if (cds_ft_insert(src, (const uint8_t *)"xabc", 4, &s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *)"xabd", 4, &s2->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		if (shape == 0)
			s = cds_ft_merge_at(dst, (const uint8_t *)"mb", 2,
					src, (const uint8_t *)"a", 1);
		else
			s = cds_ft_merge_at(dst, (const uint8_t *)"mb", 2,
					src, (const uint8_t *)"x", 1);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		keys_ok = graft_swap_oom_has_key(dst, "mango");
		if (shape == 0) {
			keys_ok = keys_ok && graft_swap_oom_has_key(src, "b");
			if (s == CDS_FT_STATUS_OK)
				keys_ok = keys_ok &&
					graft_swap_oom_has_key(dst, "mb") &&
					!graft_swap_oom_has_key(src, "a");
			else
				keys_ok = keys_ok &&
					!graft_swap_oom_has_key(dst, "mb") &&
					graft_swap_oom_has_key(src, "a");
		} else {
			if (s == CDS_FT_STATUS_OK)
				keys_ok = keys_ok &&
					graft_swap_oom_has_key(dst, "mbabc") &&
					graft_swap_oom_has_key(dst, "mbabd") &&
					!graft_swap_oom_has_key(src, "xabc") &&
					!graft_swap_oom_has_key(src, "xabd");
			else
				keys_ok = keys_ok &&
					!graft_swap_oom_has_key(dst, "mbabc") &&
					!graft_swap_oom_has_key(dst, "mbabd") &&
					graft_swap_oom_has_key(src, "xabc") &&
					graft_swap_oom_has_key(src, "xabd");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_rerooted_glue[shape=%d]: %s after fault n=%d (merge=%s)\n",
				shape, !verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_rerooted_glue_ext(void)
{
	return run_merge_oom_rerooted_glue(12, 0);
}

static int test_merge_oom_rerooted_glue_compressed(void)
{
	return run_merge_oom_rerooted_glue(16, 1);
}

/*
 * OOM coverage for the RE-ROOTED source, NOSPLIT dst: a sub-position source
 * whose subtree root is EXTERNAL (@src_shape 0) or COMPRESSED (@src_shape 1)
 * grafted in place at a NOSPLIT dst point -- an empty slot on an existing dst
 * node (@dst_shape 0: dst {za,zb}, key "zc") or a built branch absorbing a
 * displaced external (@dst_shape 1: dst {m}, key "mxyz").  cds_ft_merge_at
 * pre-reserves ft_store_at_graft_point's node allocations (grow + build-branch,
 * the re-rooted payload needing no canonicalize) and draws them, so the store
 * cannot fail on an arena allocation: every injected OOM hits the reserve fill
 * or the source unlink (both leaving the tries pristine), or is armed past them
 * and the move succeeds.  RUN_TEST's leak_check catches a stranded external on
 * the no-rollback property; the completeness assert proves the manifest.
 */
static int run_merge_oom_rerooted_nosplit(int nr_faults, int src_shape,
		int dst_shape)
{
	const char *dst_key = dst_shape == 0 ? "zc" : "mxyz";
	size_t dst_key_len = strlen(dst_key);
	const char *src_key = src_shape == 0 ? "a" : "x";
	char m0[16], m1[16];	/* moved keys */
	int n, rc = 0;

	if (src_shape == 0)
		snprintf(m0, sizeof(m0), "%s", dst_key);		/* ext */
	else {
		snprintf(m0, sizeof(m0), "%sabc", dst_key);		/* compressed */
		snprintf(m1, sizeof(m1), "%sabd", dst_key);
	}

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *b1 = node_alloc(1);
		struct ft_test_node *b2 = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_rerooted_nosplit: src create failed\n");
			return -1;
		}
		if (dst_shape == 0) {
			if (cds_ft_insert(dst, (const uint8_t *)"za", 2, &b1->node) < 0 ||
			    cds_ft_insert(dst, (const uint8_t *)"zb", 2, &b2->node) < 0)
				rc = -1;
		} else {
			if (cds_ft_insert(dst, (const uint8_t *)"m", 1, &b1->node) < 0)
				rc = -1;
			node_free(b2);	/* unused for the branch shape */
		}
		if (src_shape == 0) {
			if (cds_ft_insert(src, (const uint8_t *)"a", 1, &s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *)"b", 1, &s2->node) < 0)
				rc = -1;
		} else {
			if (cds_ft_insert(src, (const uint8_t *)"xabc", 4, &s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *)"xabd", 4, &s2->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)dst_key, dst_key_len,
				src, (const uint8_t *)src_key, strlen(src_key));
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		/* dst keeps its base keys either way. */
		keys_ok = dst_shape == 0
			? (graft_swap_oom_has_key(dst, "za") &&
			   graft_swap_oom_has_key(dst, "zb"))
			: graft_swap_oom_has_key(dst, "m");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok && graft_swap_oom_has_key(dst, m0);
			if (src_shape == 1)
				keys_ok = keys_ok && graft_swap_oom_has_key(dst, m1);
			keys_ok = keys_ok && !graft_swap_oom_has_key(src,
				src_shape == 0 ? "a" : "xabc");
		} else {
			keys_ok = keys_ok && !graft_swap_oom_has_key(dst, m0);
			keys_ok = keys_ok && graft_swap_oom_has_key(src,
				src_shape == 0 ? "a" : "xabc");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_rerooted_nosplit[src=%d,dst=%d]: %s after fault n=%d (merge=%s)\n",
				src_shape, dst_shape,
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_rerooted_nosplit_ext_atnode(void)
{
	return run_merge_oom_rerooted_nosplit(12, 0, 0);
}

static int test_merge_oom_rerooted_nosplit_ext_branch(void)
{
	return run_merge_oom_rerooted_nosplit(16, 0, 1);
}

static int test_merge_oom_rerooted_nosplit_compressed_atnode(void)
{
	return run_merge_oom_rerooted_nosplit(16, 1, 0);
}

static int test_merge_oom_rerooted_nosplit_compressed_branch(void)
{
	return run_merge_oom_rerooted_nosplit(20, 1, 1);
}

/*
 * OOM coverage for a KEY_SHORTER source into a diverged dst: the source key ends
 * INSIDE a compressed node, so cds_ft_merge_at reduces it to the EXACT in-place
 * graft of cn_s->child at @dst_key EXTENDED by the residual cn_s bytes.
 * @pshape 0: src {XYZ} merged at "X" -> payload is the EXTERNAL leaf, residual
 * "YZ".  @pshape 1: src {cabe,cabf} merged at "ca" -> payload is the INTERNAL
 * {e,f}, residual "b".  @dshape 0 lands the residual at a GLUE diverge (dst
 * {mango}, key "mb"); @dshape 1 at a NOSPLIT empty slot (dst {za,zb}, key "zc").
 * As for the EXACT shapes, every injected OOM leaves both tries pristine or the
 * move succeeds; RUN_TEST's leak_check covers the no-rollback property.
 */
static int run_merge_oom_key_shorter_diverged(int nr_faults, int pshape,
		int dshape)
{
	const char *dst_key = dshape == 0 ? "mb" : "zc";
	const char *src_key = pshape == 0 ? "X" : "ca";
	char m0[16], m1[16];
	int n, rc = 0;

	/* Moved keys: dst_key ++ residual ++ (payload-leaf suffix). */
	if (pshape == 0) {
		snprintf(m0, sizeof(m0), "%sYZ", dst_key);	/* external */
	} else {
		snprintf(m0, sizeof(m0), "%sbe", dst_key);	/* internal {e,f} */
		snprintf(m1, sizeof(m1), "%sbf", dst_key);
	}

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *b1 = node_alloc(1);
		struct ft_test_node *b2 = node_alloc(2);
		struct ft_test_node *s1 = node_alloc(3);
		struct ft_test_node *s2 = node_alloc(4);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_key_shorter_diverged: src create failed\n");
			return -1;
		}
		if (dshape == 0) {
			if (cds_ft_insert(dst, (const uint8_t *)"mango", 5, &b1->node) < 0)
				rc = -1;
			node_free(b2);
		} else {
			if (cds_ft_insert(dst, (const uint8_t *)"za", 2, &b1->node) < 0 ||
			    cds_ft_insert(dst, (const uint8_t *)"zb", 2, &b2->node) < 0)
				rc = -1;
		}
		if (pshape == 0) {
			if (cds_ft_insert(src, (const uint8_t *)"XYZ", 3, &s1->node) < 0)
				rc = -1;
			node_free(s2);
		} else {
			if (cds_ft_insert(src, (const uint8_t *)"cabe", 4, &s1->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *)"cabf", 4, &s2->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)dst_key, strlen(dst_key),
				src, (const uint8_t *)src_key, strlen(src_key));
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		keys_ok = dshape == 0
			? graft_swap_oom_has_key(dst, "mango")
			: (graft_swap_oom_has_key(dst, "za") &&
			   graft_swap_oom_has_key(dst, "zb"));
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok && graft_swap_oom_has_key(dst, m0);
			if (pshape == 1)
				keys_ok = keys_ok && graft_swap_oom_has_key(dst, m1);
			keys_ok = keys_ok && !graft_swap_oom_has_key(src,
				pshape == 0 ? "XYZ" : "cabe");
		} else {
			keys_ok = keys_ok && !graft_swap_oom_has_key(dst, m0);
			keys_ok = keys_ok && graft_swap_oom_has_key(src,
				pshape == 0 ? "XYZ" : "cabe");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_key_shorter_diverged[p=%d,d=%d]: %s after fault n=%d (merge=%s)\n",
				pshape, dshape,
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_key_shorter_diverged_ext_glue(void)
{
	return run_merge_oom_key_shorter_diverged(16, 0, 0);
}

static int test_merge_oom_key_shorter_diverged_ext_atnode(void)
{
	return run_merge_oom_key_shorter_diverged(16, 0, 1);
}

static int test_merge_oom_key_shorter_diverged_internal_glue(void)
{
	return run_merge_oom_key_shorter_diverged(20, 1, 0);
}

static int test_merge_oom_key_shorter_diverged_internal_atnode(void)
{
	return run_merge_oom_key_shorter_diverged(20, 1, 1);
}

/*
 * OOM coverage for a source root that is a multi-child internal ALSO carrying
 * external_nodes (a NIL key exactly at @src_key): src {ca, cax, cay} merged at
 * "zc" into a NOSPLIT empty slot (dst {za,zb}).  The moved subtree carries the
 * "ca" key as its at-graft-point entry ("zc") plus the x/y children ("zcx",
 * "zcy").  The in-place graft must place the whole internal -- external_nodes
 * and all -- with no fallible step after the source unlink; every OOM leaves
 * both tries pristine or the move succeeds (leak_check + completeness assert).
 */
static int run_merge_oom_subpos_external_nodes(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *d1 = node_alloc(1);
		struct ft_test_node *d2 = node_alloc(2);
		struct ft_test_node *s0 = node_alloc(3);
		struct ft_test_node *s1 = node_alloc(4);
		struct ft_test_node *s2 = node_alloc(5);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_subpos_external_nodes: src create failed\n");
			return -1;
		}
		if (cds_ft_insert(dst, (const uint8_t *)"za", 2, &d1->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *)"zb", 2, &d2->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"ca", 2, &s0->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cax", 3, &s1->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *)"cay", 3, &s2->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *)"zc", 2,
				src, (const uint8_t *)"ca", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		keys_ok = graft_swap_oom_has_key(dst, "za") &&
			graft_swap_oom_has_key(dst, "zb");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(dst, "zc") &&
				graft_swap_oom_has_key(dst, "zcx") &&
				graft_swap_oom_has_key(dst, "zcy") &&
				!graft_swap_oom_has_key(src, "ca");
		} else {
			keys_ok = keys_ok &&
				!graft_swap_oom_has_key(dst, "zc") &&
				graft_swap_oom_has_key(src, "ca") &&
				graft_swap_oom_has_key(src, "cax");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_subpos_external_nodes: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_subpos_external_nodes(void)
{
	return run_merge_oom_subpos_external_nodes(16);
}

/*
 * OOM coverage for the SAME-TRIE rekey: move "ax" -> "az" within one trie (the
 * deep shared ancestor 'a' canonicalizes on the move).  The implementation
 * pre-reserves every node + flip batch BEFORE the detach, so the detach is the
 * last fallible step and the post-detach placement merge cannot fail.  On a
 * single injected fault the move therefore either leaves the trie PRISTINE
 * (the fault hit during the reserve fill or the detach, before anything moved
 * -- content still at "ax") or fully succeeds (content at "az").  There is no
 * reader-observable rollback / restore, and leak_check never sees a stranded
 * external.
 */
static int run_merge_oom_rekey(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *ft = create_varlen_ft(&group);
		struct ft_test_node *a1 = node_alloc(1);
		struct ft_test_node *a2 = node_alloc(2);
		struct ft_test_node *a3 = node_alloc(3);
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_insert(ft, (const uint8_t *)"axm", 3, &a1->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *)"axn", 3, &a2->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *)"ayp", 3, &a3->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(ft, (const uint8_t *)"az", 2,
				ft, (const uint8_t *)"ax", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		keys_ok = graft_swap_oom_has_key(ft, "ayp");
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(ft, "azm") &&
				graft_swap_oom_has_key(ft, "azn") &&
				!graft_swap_oom_has_key(ft, "axm");
		} else {
			/* Fault before the detach committed: pristine, content at "ax". */
			keys_ok = keys_ok &&
				graft_swap_oom_has_key(ft, "axm") &&
				graft_swap_oom_has_key(ft, "axn") &&
				!graft_swap_oom_has_key(ft, "azm");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_rekey: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(ft) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(ft);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_rekey(void)
{
	return run_merge_oom_rekey(20);
}

/*
 * OOM coverage for the PIECEWISE merge: dst already has nodes that overlap
 * src's, so ft_merge_build must recurse INTO the shared spine -- copying the
 * shared branch nodes, splicing the same full keys ("aa", "ba"), and
 * referencing only the divergent children ("ab"/"bb" in dst, "ac"/"bc" in
 * src).  The shared path runs through multi-child branches (root -> {a,b};
 * each -> {a,...}), never a single-child run, so no compressed node appears on
 * it and the spine-copy is taken under every build.  Because per-entry
 * delegation would leave dst with a partial key set on a mid-loop OOM, this
 * test passing the pristine assertion is itself proof the spine-copy path ran.
 *
 * Every OOM across the (larger) build window must leave BOTH tries pristine;
 * success yields the union in dst (with "aa"/"ba" as 2-deep duplicate chains)
 * and an empty src.
 */
static int run_merge_oom_overlap(int nr_faults)
{
	static const char *const dkeys[] = { "aa", "ab", "ba", "bb" };
	static const char *const skeys[] = { "aa", "ac", "ba", "bc" };
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		enum cds_ft_status s;
		int verified, keys_ok;
		unsigned int i;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_overlap: src create failed\n");
			return -1;
		}
		for (i = 0; i < 4; i++) {
			struct ft_test_node *dn = node_alloc(100 + i);
			struct ft_test_node *sn = node_alloc(200 + i);

			if (cds_ft_insert(dst, (const uint8_t *) dkeys[i], 2,
					&dn->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *) skeys[i], 2,
					&sn->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge(dst, NULL, 0, src);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			/* dst holds the union; src is empty. */
			keys_ok = graft_swap_oom_has_key(dst, "aa") &&
				graft_swap_oom_has_key(dst, "ab") &&
				graft_swap_oom_has_key(dst, "ac") &&
				graft_swap_oom_has_key(dst, "ba") &&
				graft_swap_oom_has_key(dst, "bb") &&
				graft_swap_oom_has_key(dst, "bc") &&
				!graft_swap_oom_has_key(src, "aa") &&
				!graft_swap_oom_has_key(src, "ac");
		} else {
			/* OOM: both tries pristine (no key moved, none lost). */
			keys_ok = graft_swap_oom_has_key(dst, "aa") &&
				graft_swap_oom_has_key(dst, "ab") &&
				!graft_swap_oom_has_key(dst, "ac") &&
				graft_swap_oom_has_key(dst, "bb") &&
				!graft_swap_oom_has_key(dst, "bc") &&
				graft_swap_oom_has_key(src, "aa") &&
				graft_swap_oom_has_key(src, "ac") &&
				graft_swap_oom_has_key(src, "bc");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_overlap: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_overlap(void)
{
	return run_merge_oom_overlap(16);
}

/*
 * As run_merge_oom_overlap, but the src/dst overlap contains COMPRESSED
 * nodes, so the spine-copy builder allocates fresh compressed runs and
 * suffix wrappers in addition to internal branch nodes:
 *
 *   dst {aaaa1, aaaa2, ccccPQR}      src {aaaa1, aaaa3, ccccXYZ}
 *
 * "aaaa" is a shared run resolving to a {1,2,3} branch (with "aaaa1" a
 * spliced 2-entry chain); "cccc" is a shared run diverging at P/X into a
 * branch whose two children are freshly-built "QR"/"YZ" compressed
 * wrappers.  Every OOM across the build window must leave BOTH tries
 * pristine -- the property the per-entry fallback (which moves keys one at
 * a time) cannot provide and which the build-invisible spine copy exists
 * to restore.  On success dst holds the 6-entry union and src is empty.
 */
static int run_merge_oom_compressed(int nr_faults)
{
	static const char *const dkeys[] = { "aaaa1", "aaaa2", "ccccPQR" };
	static const char *const skeys[] = { "aaaa1", "aaaa3", "ccccXYZ" };
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		enum cds_ft_status s;
		int verified, keys_ok;
		unsigned int i;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_compressed: src create failed\n");
			return -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
			struct ft_test_node *dn = node_alloc(100 + i);
			struct ft_test_node *sn = node_alloc(200 + i);

			if (cds_ft_insert(dst, (const uint8_t *) dkeys[i],
					strlen(dkeys[i]), &dn->node) < 0 ||
			    cds_ft_insert(src, (const uint8_t *) skeys[i],
					strlen(skeys[i]), &sn->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge(dst, NULL, 0, src);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			/* dst holds the union (aaaa1 a 2-chain); src is empty. */
			keys_ok = graft_swap_oom_has_key(dst, "aaaa1") &&
				graft_swap_oom_has_key(dst, "aaaa2") &&
				graft_swap_oom_has_key(dst, "aaaa3") &&
				graft_swap_oom_has_key(dst, "ccccPQR") &&
				graft_swap_oom_has_key(dst, "ccccXYZ") &&
				cds_ft_count_entries(dst) == 6 &&
				cds_ft_empty(src);
		} else {
			/* OOM: both tries pristine (no key moved, none lost). */
			keys_ok = graft_swap_oom_has_key(dst, "aaaa1") &&
				graft_swap_oom_has_key(dst, "aaaa2") &&
				!graft_swap_oom_has_key(dst, "aaaa3") &&
				graft_swap_oom_has_key(dst, "ccccPQR") &&
				!graft_swap_oom_has_key(dst, "ccccXYZ") &&
				graft_swap_oom_has_key(src, "aaaa1") &&
				graft_swap_oom_has_key(src, "aaaa3") &&
				graft_swap_oom_has_key(src, "ccccXYZ");
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_compressed: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_compressed(void)
{
	return run_merge_oom_compressed(20);
}

/*
 * As run_merge_oom_compressed, but a NON-ROOT src merged into a ROOT dst, so
 * the spine-copy's commit performs the in-place src-branch detach -- the one
 * post-build fallible step (ft_detach_node recompaction).  src holds a "T"
 * subtree that must survive every OOM untouched.  This sweep therefore covers
 * the build window AND the detach: on every allocation failure both tries must
 * stay pristine (dst unchanged, src keeps all of "S" and "T").  On success dst
 * holds the merged union and src keeps only "T".
 */
static int run_merge_oom_nonroot_src(int nr_faults)
{
	static const char *const dkeys[] = { "aaaa", "aaab" };
	static const char *const skeys[] = { "Saaaa", "Saaac", "Tzz" };
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		enum cds_ft_status s;
		int verified, keys_ok;
		unsigned int i;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_nonroot_src: src create failed\n");
			return -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
			struct ft_test_node *dn = node_alloc(100 + i);

			if (cds_ft_insert(dst, (const uint8_t *) dkeys[i],
					strlen(dkeys[i]), &dn->node) < 0)
				rc = -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
			struct ft_test_node *sn = node_alloc(200 + i);

			if (cds_ft_insert(src, (const uint8_t *) skeys[i],
					strlen(skeys[i]), &sn->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, NULL, 0, src, (const uint8_t *) "S", 1);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "aaaa") &&
				graft_swap_oom_has_key(dst, "aaab") &&
				graft_swap_oom_has_key(dst, "aaac") &&
				cds_ft_count_entries(dst) == 4 &&
				graft_swap_oom_has_key(src, "Tzz") &&
				cds_ft_count_entries(src) == 1;
		} else {
			/* OOM: both tries pristine. */
			keys_ok = graft_swap_oom_has_key(dst, "aaaa") &&
				graft_swap_oom_has_key(dst, "aaab") &&
				!graft_swap_oom_has_key(dst, "aaac") &&
				cds_ft_count_entries(dst) == 2 &&
				graft_swap_oom_has_key(src, "Saaaa") &&
				graft_swap_oom_has_key(src, "Saaac") &&
				graft_swap_oom_has_key(src, "Tzz") &&
				cds_ft_count_entries(src) == 3;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_nonroot_src: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_nonroot_src(void)
{
	return run_merge_oom_nonroot_src(24);
}

/*
 * As run_merge_oom_compressed, but into a NON-ROOT dst merge point (the merged
 * cluster publishes through an interior slot via the flip's type-7 proxy).  A
 * disjoint "Q" subtree in dst must stay intact on every OOM.  On success dst
 * holds the merged union under "P" plus "Q"; on OOM both tries are pristine.
 */
static int run_merge_oom_nonroot_dst(int nr_faults)
{
	static const char *const dkeys[] = { "Pa", "Pb", "Z" };
	static const char *const skeys[] = { "a", "c" };
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		enum cds_ft_status s;
		int verified, keys_ok;
		unsigned int i;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_nonroot_dst: src create failed\n");
			return -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
			struct ft_test_node *dn = node_alloc(100 + i);

			if (cds_ft_insert(dst, (const uint8_t *) dkeys[i],
					strlen(dkeys[i]), &dn->node) < 0)
				rc = -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
			struct ft_test_node *sn = node_alloc(200 + i);

			if (cds_ft_insert(src, (const uint8_t *) skeys[i],
					strlen(skeys[i]), &sn->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1, src, NULL, 0);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "Pa") &&
				graft_swap_oom_has_key(dst, "Pb") &&
				graft_swap_oom_has_key(dst, "Pc") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 5 &&
				cds_ft_empty(src);
		} else {
			/* OOM: both tries pristine. */
			keys_ok = graft_swap_oom_has_key(dst, "Pa") &&
				graft_swap_oom_has_key(dst, "Pb") &&
				!graft_swap_oom_has_key(dst, "Pc") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 3 &&
				graft_swap_oom_has_key(src, "a") &&
				graft_swap_oom_has_key(src, "c") &&
				cds_ft_count_entries(src) == 2;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_nonroot_dst: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_nonroot_dst(void)
{
	return run_merge_oom_nonroot_dst(20);
}

/*
 * As run_merge_oom_nonroot_dst, but the dst merge point is EXTERNAL (dst_key
 * "P" is a single leaf, parent internal): merging a src subtree under it
 * builds a fresh internal M carrying the leaf as external_nodes.  Every OOM
 * across the build window must leave both tries pristine; a disjoint "Z"
 * confirms the rest of dst is untouched.
 *
 *   dst {P, Z}   src {a, c}  ->  dst {P, Pa, Pc, Z}, src empty
 */
static int run_merge_oom_external_dst(int nr_faults)
{
	static const char *const dkeys[] = { "P", "Z" };
	static const char *const skeys[] = { "a", "c" };
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		enum cds_ft_status s;
		int verified, keys_ok;
		unsigned int i;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_external_dst: src create failed\n");
			return -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(dkeys); i++) {
			struct ft_test_node *dn = node_alloc(100 + i);

			if (cds_ft_insert(dst, (const uint8_t *) dkeys[i],
					strlen(dkeys[i]), &dn->node) < 0)
				rc = -1;
		}
		for (i = 0; i < CAA_ARRAY_SIZE(skeys); i++) {
			struct ft_test_node *sn = node_alloc(200 + i);

			if (cds_ft_insert(src, (const uint8_t *) skeys[i],
					strlen(skeys[i]), &sn->node) < 0)
				rc = -1;
		}

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1, src, NULL, 0);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "P") &&
				graft_swap_oom_has_key(dst, "Pa") &&
				graft_swap_oom_has_key(dst, "Pc") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 4 &&
				cds_ft_empty(src);
		} else {
			keys_ok = graft_swap_oom_has_key(dst, "P") &&
				!graft_swap_oom_has_key(dst, "Pa") &&
				!graft_swap_oom_has_key(dst, "Pc") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 2 &&
				graft_swap_oom_has_key(src, "a") &&
				graft_swap_oom_has_key(src, "c") &&
				cds_ft_count_entries(src) == 2;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_external_dst: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_external_dst(void)
{
	return run_merge_oom_external_dst(20);
}

/*
 * As run_merge_oom_external_dst, but the dst merge point is COMPRESSED and the
 * merged result M is itself a fresh compressed run published skip-encoded (the
 * M_slot path).  Every OOM across the build window must leave both tries
 * pristine.
 *
 *   dst {Pxyz, Z}   src {Qxyw}  ->  dst {Pxyz, Pxyw, Z}, src empty
 */
static int run_merge_oom_compressed_dst(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *a, *b, *c;
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_compressed_dst: src create failed\n");
			return -1;
		}
		a = node_alloc(100);
		b = node_alloc(101);
		c = node_alloc(200);
		if (cds_ft_insert(dst, (const uint8_t *) "Pxyz", 4, &a->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *) "Z", 1, &b->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "Qxyw", 4, &c->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "P", 1,
				src, (const uint8_t *) "Q", 1);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "Pxyz") &&
				graft_swap_oom_has_key(dst, "Pxyw") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 3 &&
				cds_ft_empty(src);
		} else {
			keys_ok = graft_swap_oom_has_key(dst, "Pxyz") &&
				!graft_swap_oom_has_key(dst, "Pxyw") &&
				graft_swap_oom_has_key(dst, "Z") &&
				cds_ft_count_entries(dst) == 2 &&
				graft_swap_oom_has_key(src, "Qxyw") &&
				cds_ft_count_entries(src) == 1;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_compressed_dst: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_compressed_dst(void)
{
	return run_merge_oom_compressed_dst(24);
}

/*
 * OOM atomicity for a KEY_SHORTER dst merge point (@dst_key ends inside a
 * compressed node).  Every fault in the build window -- including the prefix-
 * wrap allocation -- must leave dst unchanged ({abcd}) and src intact ({P, Q}),
 * with both tries verifying clean; only a fault-free run commits the full
 * merged set {abcd, abP, abQ}.
 */
static int run_merge_oom_key_shorter_dst(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *a, *b, *c;
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_key_shorter_dst: src create failed\n");
			return -1;
		}
		a = node_alloc(100);
		b = node_alloc(101);
		c = node_alloc(102);
		if (cds_ft_insert(dst, (const uint8_t *) "abcd", 4, &a->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "P", 1, &b->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "Q", 1, &c->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "ab", 2, src, NULL, 0);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "abcd") &&
				graft_swap_oom_has_key(dst, "abP") &&
				graft_swap_oom_has_key(dst, "abQ") &&
				cds_ft_count_entries(dst) == 3 &&
				cds_ft_empty(src);
		} else {
			keys_ok = graft_swap_oom_has_key(dst, "abcd") &&
				!graft_swap_oom_has_key(dst, "abP") &&
				!graft_swap_oom_has_key(dst, "abQ") &&
				cds_ft_count_entries(dst) == 1 &&
				graft_swap_oom_has_key(src, "P") &&
				graft_swap_oom_has_key(src, "Q") &&
				cds_ft_count_entries(src) == 2;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_key_shorter_dst: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_key_shorter_dst(void)
{
	return run_merge_oom_key_shorter_dst(24);
}

/*
 * OOM atomicity for a KEY_SHORTER src merge point (@src_key ends inside a
 * compressed src node).  Every fault in the build window must leave dst
 * unchanged ({Qa}) and src intact ({XYZ}), with both tries verifying clean;
 * only a fault-free run commits the full merged set {Qa, QZ} and empties src.
 */
static int run_merge_oom_key_shorter_src(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *a, *b;
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_key_shorter_src: src create failed\n");
			return -1;
		}
		a = node_alloc(100);
		b = node_alloc(101);
		if (cds_ft_insert(dst, (const uint8_t *) "Qa", 2, &a->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "XYZ", 3, &b->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "Q", 1,
				src, (const uint8_t *) "XY", 2);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "Qa") &&
				graft_swap_oom_has_key(dst, "QZ") &&
				cds_ft_count_entries(dst) == 2 &&
				cds_ft_empty(src);
		} else {
			keys_ok = graft_swap_oom_has_key(dst, "Qa") &&
				!graft_swap_oom_has_key(dst, "QZ") &&
				cds_ft_count_entries(dst) == 1 &&
				graft_swap_oom_has_key(src, "XYZ") &&
				cds_ft_count_entries(src) == 1;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_key_shorter_src: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_key_shorter_src(void)
{
	return run_merge_oom_key_shorter_src(24);
}

/*
 * OOM atomicity for Edge D (the dst merge point's PARENT is a compressed node).
 * Every fault in the build window -- including the compressed-parent copy --
 * must leave dst unchanged ({aXYc, aXYd}) and src intact ({P, Q}), both tries
 * verifying clean; only a fault-free run commits {aXYc, aXYd, aXYP, aXYQ}.
 */
static int run_merge_oom_compressed_parent_dst(int nr_faults)
{
	int n, rc = 0;

	for (n = 0; n < nr_faults; n++) {
		struct cds_ft_group *group;
		struct cds_ft *dst = create_varlen_ft(&group);
		struct cds_ft *src;
		struct ft_test_node *a, *b, *c, *d;
		enum cds_ft_status s;
		int verified, keys_ok;

		if (cds_ft_create(group, NULL, &src) < 0) {
			fprintf(stderr, "merge_oom_compressed_parent_dst: src create failed\n");
			return -1;
		}
		a = node_alloc(100);
		b = node_alloc(101);
		c = node_alloc(102);
		d = node_alloc(103);
		if (cds_ft_insert(dst, (const uint8_t *) "aXYc", 4, &a->node) < 0 ||
		    cds_ft_insert(dst, (const uint8_t *) "aXYd", 4, &b->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "P", 1, &c->node) < 0 ||
		    cds_ft_insert(src, (const uint8_t *) "Q", 1, &d->node) < 0)
			rc = -1;

		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_merge_at(dst, (const uint8_t *) "aXY", 3, src, NULL, 0);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(dst, stderr) == CDS_FT_STATUS_OK) &&
			(cds_ft_verify(src, stderr) == CDS_FT_STATUS_OK);
		if (s == CDS_FT_STATUS_OK) {
			keys_ok = graft_swap_oom_has_key(dst, "aXYc") &&
				graft_swap_oom_has_key(dst, "aXYd") &&
				graft_swap_oom_has_key(dst, "aXYP") &&
				graft_swap_oom_has_key(dst, "aXYQ") &&
				cds_ft_count_entries(dst) == 4 &&
				cds_ft_empty(src);
		} else {
			keys_ok = graft_swap_oom_has_key(dst, "aXYc") &&
				graft_swap_oom_has_key(dst, "aXYd") &&
				!graft_swap_oom_has_key(dst, "aXYP") &&
				cds_ft_count_entries(dst) == 2 &&
				graft_swap_oom_has_key(src, "P") &&
				graft_swap_oom_has_key(src, "Q") &&
				cds_ft_count_entries(src) == 2;
		}
		rcu_read_unlock();
		if (!verified || !keys_ok) {
			fprintf(stderr,
				"merge_oom_compressed_parent_dst: %s after fault n=%d (merge=%s)\n",
				!verified ? "verify FAILED" : "KEY SET WRONG",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}

		if (drain_trie(dst) < 0 || drain_trie(src) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_merge_oom_compressed_parent_dst(void)
{
	return run_merge_oom_compressed_parent_dst(24);
}

/*
 * Drive a fresh-key ATTACH with a compressed tail through each allocation-
 * failure point.  Regression for the 2026-06 review's finding 2.1: the attach
 * tracked ft_try_compress_chain's SKIP-ENCODED return in created_nodes[]; the
 * ENOMEM unwind dispatched only on ft_node_compressed, so the skip flag fell
 * into the plain-node arm and ran arena arithmetic on the APPLICATION'S
 * pointer (the skip encoding carries the child's address), poisoning an arena
 * freelist.  Also covers 2.5 for ordered tries: after a failed insert the
 * node must be retryable (node->prev reset).
 *
 * The trie has root children 'm' and 'z'; inserting "a<tail>" dispatches at
 * the root with a compressible 7-byte tail, and the root append of byte 'a'
 * below the existing maximum forces a rank-preserving recompact -- the
 * fallible step AFTER the compressed node was built and tracked.
 */
static int run_attach_oom(const char *label, bool ordered)
{
	int n, rc = 0;

	for (n = 0; n < 8; n++) {
		struct cds_ft_group_attr *attr;
		struct cds_ft_group *group;
		struct cds_ft *ft;
		struct ft_test_node *m = node_alloc(1);
		struct ft_test_node *z = node_alloc(2);
		struct ft_test_node *a = node_alloc(3);
		enum cds_ft_status s;
		int verified;

		if (cds_ft_group_attr_create(&attr) < 0)
			abort();
		cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
		cds_ft_group_attr_set_ordered_list(attr, ordered);
		if (cds_ft_group_create(attr, &group) < 0)
			abort();
		cds_ft_group_attr_destroy(attr);
		if (cds_ft_create(group, NULL, &ft) < 0)
			abort();

		if (cds_ft_insert(ft, (const uint8_t *) "mmmmmmmm", 8, &m->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "zzzzzzzz", 8, &z->node) < 0) {
			fprintf(stderr, "attach_oom[%s]: build failed\n", label);
			rc = -1;
		}

		/* Fail the (n+1)-th allocation performed by the insert. */
		cds_ft_fault_alloc_countdown = n;
		s = cds_ft_insert(ft, (const uint8_t *) "aaaaaaaa", 8, &a->node);
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"attach_oom[%s]: verify FAILED after fault n=%d (insert=%s)\n",
				label, n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}
		if (s != CDS_FT_STATUS_OK) {
			/* 2.5: the failed insert must leave @a retryable. */
			s = cds_ft_insert(ft, (const uint8_t *) "aaaaaaaa", 8,
					&a->node);
			if (s != CDS_FT_STATUS_OK) {
				fprintf(stderr,
					"attach_oom[%s]: retry after fault n=%d failed: %s\n",
					label, n, cds_ft_status_to_string(s));
				rc = -1;
			}
		}
		if (!graft_swap_oom_has_key(ft, "aaaaaaaa") ||
		    !graft_swap_oom_has_key(ft, "mmmmmmmm") ||
		    !graft_swap_oom_has_key(ft, "zzzzzzzz")) {
			fprintf(stderr, "attach_oom[%s]: key missing (n=%d)\n",
				label, n);
			rc = -1;
		}
		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"attach_oom[%s]: verify FAILED after retry (n=%d)\n",
				label, n);
			rc = -1;
			continue;
		}
		if (drain_trie(ft) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(ft);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

static int test_attach_oom_skip_unwind(void)
{
	if (run_attach_oom("ordered", true))
		return -1;
	return run_attach_oom("list_off", false);
}

/*
 * Drive cds_ft_remove_all through allocation-failure points and assert its
 * error contract (2026-06 review, 2.4):
 *  - leaf key, detach ENOMEM: the removal rolls back cleanly -> status
 *    CDS_FT_STATUS_MEMORY_ERROR (was NOT_FOUND), *result_node == NULL (was
 *    the live chain -- inviting caller-side reclamation of reachable data),
 *    key still present, counts consistent;
 *  - prefix key whose emptied holder fails to prune: the removal itself
 *    COMMITTED -> status OK, key gone, no count re-add (the old +1 undo left
 *    a permanent ancestor overcount that cds_ft_verify flags), empty holder
 *    tolerated.
 */
static int test_remove_all_oom_contract(void)
{
	int n, rc = 0;

	for (n = 0; n < 6; n++) {
		struct cds_ft_group *group;
		struct cds_ft *ft = create_varlen_ft(&group);
		struct cds_ft_iter *iter;
		struct ft_test_node *ab = node_alloc(1);
		struct ft_test_node *abc = node_alloc(2);
		struct cds_ft_node *res = (struct cds_ft_node *) (long) -1;
		enum cds_ft_status s;
		int verified;

		if (cds_ft_iter_create(ft, &iter) < 0)
			abort();
		/*
		 * "ab" + "abc", then remove "abc": leaves "ab" as a prefix
		 * key on an internal holder with nr_child == 0 (the holder
		 * cannot collapse while it carries external_nodes).
		 */
		if (cds_ft_insert(ft, (const uint8_t *) "ab", 2, &ab->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "abc", 3, &abc->node) < 0) {
			fprintf(stderr, "remove_all_oom: build failed\n");
			rc = -1;
		}
		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) "abc", 3);
		if (cds_ft_lookup(ft, iter) != CDS_FT_STATUS_OK ||
		    cds_ft_remove_all(ft, iter, &res) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "remove_all_oom: abc removal failed\n");
			rc = -1;
		}
		rcu_read_unlock();
		node_free_rcu(abc);

		/* Prefix-key removal under an allocation fault. */
		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) "ab", 2);
		s = cds_ft_lookup(ft, iter);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "remove_all_oom: ab lookup failed\n");
			rc = -1;
		}
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_remove_all(ft, iter, &res);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"remove_all_oom: verify FAILED after fault n=%d (%s)\n",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;
		}
		if (s == CDS_FT_STATUS_OK) {
			if (res != &ab->node ||
			    graft_swap_oom_has_key(ft, "ab")) {
				fprintf(stderr,
					"remove_all_oom: OK but inconsistent (n=%d)\n", n);
				rc = -1;
			}
			node_free_rcu(ab);
		} else if (s == CDS_FT_STATUS_MEMORY_ERROR) {
			if (res != NULL || !graft_swap_oom_has_key(ft, "ab")) {
				fprintf(stderr,
					"remove_all_oom: MEMORY_ERROR contract broken (n=%d, res=%p)\n",
					n, (void *) res);
				rc = -1;
			}
		} else {
			fprintf(stderr,
				"remove_all_oom: unexpected status %s for existing key (n=%d)\n",
				cds_ft_status_to_string(s), n);
			rc = -1;
		}
		cds_ft_iter_destroy(iter);
		if (drain_trie(ft) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(ft);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

/* Collect the trie's keys in ordered forward (or reverse) traversal into
 * @out (@max slots of 8 bytes), with lengths in @lens.  Returns -1 on a
 * runaway (more than @max keys -- suspected loop or corruption) or a get_key
 * failure, else 0 with the count in *@count.  Caller holds the RCU read lock. */
static int remove_oom_collect_ordered(struct cds_ft *ft, struct cds_ft_iter *iter,
		char out[][8], size_t *lens, int max, int reverse, int *count)
{
	enum cds_ft_status s;
	int n = 0;

	s = reverse ? cds_ft_lookup_last(ft, iter) : cds_ft_lookup_first(ft, iter);
	while (s == CDS_FT_STATUS_OK) {
		size_t kl = 0;

		if (n >= max)
			return -1;	/* runaway: suspected loop / empty-holder corruption */
		if (cds_ft_iter_get_key(iter, (uint8_t *) out[n], 8, &kl) != CDS_FT_STATUS_OK)
			return -1;
		lens[n++] = kl;
		s = reverse ? cds_ft_prev(ft, iter) : cds_ft_next(ft, iter);
	}
	*count = n;
	return 0;
}

/* True if the (buf, len) key equals the NUL-terminated literal @lit. */
static int remove_oom_keq(const char *buf, size_t len, const char *lit)
{
	return len == strlen(lit) && memcmp(buf, lit, len) == 0;
}

/*
 * #F regression (companion to test_remove_all_oom_contract): when a prefix-key
 * remove_all's emptied-holder prune fails with -ENOMEM, the holder is left as a
 * reachable nr_child==0 internal node (the key removal was already published, so
 * remove_all returns OK).  Ordered traversal must step PAST that empty holder,
 * not dead-end on it or loop.
 *
 * Shape: "a0", "ab", "abc", "az" share the byte-'a' internal node.  Removing
 * "abc" leaves "ab" as a prefix key on internal holder N (nr_child==0).
 * Removing "ab" under an allocation fault can fail the detach/prune of N,
 * leaving N empty between its siblings "a0" and "az".  Forward AND reverse
 * ordered traversals must then visit "a0" and "az" (and "ab" iff the remove
 * OOM'd, in which case ab's external is still present so N is not empty), in
 * order, mutually consistent, and terminate.
 */
static int test_remove_emptied_holder_traversal_oom(void)
{
	int n, rc = 0;

	for (n = 0; n < 10; n++) {
		struct cds_ft_group *group;
		struct cds_ft *ft = create_varlen_ft(&group);
		struct cds_ft_iter *iter;
		struct ft_test_node *a0 = node_alloc(1);
		struct ft_test_node *ab = node_alloc(2);
		struct ft_test_node *abc = node_alloc(3);
		struct ft_test_node *az = node_alloc(4);
		struct cds_ft_node *res = NULL;
		enum cds_ft_status s;
		int verified, removed_ab, ok = 1, i, fc = 0, rvc = 0;
		char fwd[8][8], rev[8][8];
		size_t fwl[8], rvl[8];

		if (cds_ft_iter_create(ft, &iter) < 0)
			abort();
		if (cds_ft_insert(ft, (const uint8_t *) "a0", 2, &a0->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "ab", 2, &ab->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "abc", 3, &abc->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "az", 2, &az->node) < 0) {
			fprintf(stderr, "emptied_holder: build failed\n");
			rc = -1;
		}

		/* Remove "abc": "ab" becomes a prefix key on an nr_child==0 holder. */
		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) "abc", 3);
		if (cds_ft_lookup(ft, iter) != CDS_FT_STATUS_OK ||
		    cds_ft_remove_all(ft, iter, &res) != CDS_FT_STATUS_OK)
			rc = -1;
		rcu_read_unlock();
		node_free_rcu(abc);

		/* Remove the prefix key "ab" under the (n+1)-th allocation fault. */
		rcu_read_lock();
		cds_ft_iter_set_key(iter, (const uint8_t *) "ab", 2);
		s = cds_ft_lookup(ft, iter);
		rcu_read_unlock();
		if (s != CDS_FT_STATUS_OK)
			rc = -1;
		res = NULL;
		cds_ft_fault_alloc_countdown = n;
		rcu_read_lock();
		s = cds_ft_remove_all(ft, iter, &res);
		rcu_read_unlock();
		cds_ft_fault_alloc_countdown = -1;
		removed_ab = (s == CDS_FT_STATUS_OK);

		/* Verify + ordered traversal over the (possibly empty-holder) trie. */
		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		if (verified &&
		    (remove_oom_collect_ordered(ft, iter, fwd, fwl, 8, 0, &fc) < 0 ||
		     remove_oom_collect_ordered(ft, iter, rev, rvl, 8, 1, &rvc) < 0)) {
			fprintf(stderr,
				"emptied_holder: traversal runaway/failure (n=%d)\n", n);
			ok = 0;
		}
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"emptied_holder: verify FAILED after fault n=%d (%s)\n",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}
		if (ok) {
			int expect = removed_ab ? 2 : 3;	/* {a0,az} or {a0,ab,az} */

			/* Forward and reverse must agree (reverse == forward reversed). */
			if (fc != expect || rvc != fc) {
				fprintf(stderr,
					"emptied_holder: count fwd=%d rev=%d expect=%d (n=%d, removed_ab=%d)\n",
					fc, rvc, expect, n, removed_ab);
				ok = 0;
			}
			for (i = 0; ok && i < fc; i++) {
				if (fwl[i] != rvl[fc - 1 - i] ||
				    memcmp(fwd[i], rev[fc - 1 - i], fwl[i]) != 0) {
					fprintf(stderr,
						"emptied_holder: fwd/rev mismatch at %d (n=%d)\n", i, n);
					ok = 0;
				}
			}
			/* Strictly increasing forward, and the siblings are present. */
			if (ok && (!remove_oom_keq(fwd[0], fwl[0], "a0") ||
				   !remove_oom_keq(fwd[fc - 1], fwl[fc - 1], "az"))) {
				fprintf(stderr,
					"emptied_holder: siblings a0/az not at the ends (n=%d)\n", n);
				ok = 0;
			}
			for (i = 1; ok && i < fc; i++) {
				size_t m = fwl[i - 1] < fwl[i] ? fwl[i - 1] : fwl[i];
				int c = memcmp(fwd[i - 1], fwd[i], m);

				if (c > 0 || (c == 0 && fwl[i - 1] >= fwl[i])) {
					fprintf(stderr,
						"emptied_holder: not strictly increasing at %d (n=%d)\n", i, n);
					ok = 0;
				}
			}
			/* "ab" present iff the removal OOM'd (then N is not empty). */
			if (ok) {
				int has_ab = 0;

				for (i = 0; i < fc; i++)
					if (remove_oom_keq(fwd[i], fwl[i], "ab"))
						has_ab = 1;
				if (has_ab == removed_ab) {
					fprintf(stderr,
						"emptied_holder: ab presence=%d but removed_ab=%d (n=%d)\n",
						has_ab, removed_ab, n);
					ok = 0;
				}
			}
		}
		if (!ok)
			rc = -1;
		if (removed_ab) {
			if (res != &ab->node)
				rc = -1;
			node_free_rcu(ab);	/* removed: drain won't see it */
		}
		cds_ft_iter_destroy(iter);
		if (drain_trie(ft) < 0)	/* frees a0, az (+ ab if it stayed) */
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(ft);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}

/*
 * Drive cds_ft_detach at a key whose child is a COMPRESSED node (so the
 * detached trie's root must be materialized as an internal node) through every
 * allocation-failure point, and assert atomicity: on MEMORY_ERROR the source
 * trie must still pass cds_ft_verify AND still contain every key.  Regression
 * for the 2026-06 review's finding 2.9: the fallible root materialization ran
 * AFTER the detach had published the unlink and taken a grace period, and the
 * "re-attach the subtree" rollback did not exist -- an OOM there silently lost
 * every detached key from BOTH tries (plus an ancestor count corruption and a
 * UAF count undo).  The materialization now runs build-invisibly BEFORE any
 * published mutation.
 */
static int test_detach_oom_atomicity(void)
{
	int n, rc = 0;

	for (n = 0; n < 10; n++) {
		struct cds_ft_group *group;
		struct cds_ft *ft = create_varlen_ft(&group);
		struct cds_ft *det = NULL;
		struct ft_test_node *a = node_alloc(1);
		struct ft_test_node *b = node_alloc(2);
		enum cds_ft_status s;
		int verified;

		/* root slot 'a' -> compressed("bcdefg") -> internal {1, 2}. */
		if (cds_ft_insert(ft, (const uint8_t *) "abcdefg1", 8, &a->node) < 0 ||
		    cds_ft_insert(ft, (const uint8_t *) "abcdefg2", 8, &b->node) < 0) {
			fprintf(stderr, "detach_oom: build failed\n");
			rc = -1;
		}

		/* Fail the (n+1)-th allocation performed by the detach. */
		cds_ft_fault_alloc_countdown = n;
		s = cds_ft_detach(ft, (const uint8_t *) "a", 1, &det);
		cds_ft_fault_alloc_countdown = -1;

		rcu_read_lock();
		verified = (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK);
		if (det)
			verified = verified &&
				(cds_ft_verify(det, stderr) == CDS_FT_STATUS_OK);
		rcu_read_unlock();
		if (!verified) {
			fprintf(stderr,
				"detach_oom: verify FAILED after fault n=%d (detach=%s)\n",
				n, cds_ft_status_to_string(s));
			rc = -1;
			continue;	/* corrupt: abandon (leak) this iteration */
		}

		if (s == CDS_FT_STATUS_OK) {
			/* Keys moved: present in @det under the stripped prefix. */
			if (!det ||
			    !graft_swap_oom_has_key(det, "bcdefg1") ||
			    !graft_swap_oom_has_key(det, "bcdefg2")) {
				fprintf(stderr,
					"detach_oom: keys missing in detached trie (n=%d)\n", n);
				rc = -1;
			}
		} else {
			/* OOM: the source must be UNCHANGED (no silent loss). */
			if (s != CDS_FT_STATUS_MEMORY_ERROR || det) {
				fprintf(stderr,
					"detach_oom: unexpected status %s (n=%d)\n",
					cds_ft_status_to_string(s), n);
				rc = -1;
			}
			if (!graft_swap_oom_has_key(ft, "abcdefg1") ||
			    !graft_swap_oom_has_key(ft, "abcdefg2")) {
				fprintf(stderr,
					"detach_oom: keys LOST from source after OOM (n=%d)\n", n);
				rc = -1;
			}
		}

		if (det) {
			if (drain_trie(det) < 0)
				rc = -1;
			rcu_barrier();
			cds_ft_destroy(det);
		}
		if (drain_trie(ft) < 0)
			rc = -1;
		rcu_barrier();
		cds_ft_destroy(ft);
		rcu_barrier();
		cds_ft_group_destroy(group);
	}
	return rc;
}
#endif /* FEATURE_FT_FAULT_INJECT */

int main(int argc, char **argv)
{
	const char *filter = (argc >= 2) ? argv[1] : NULL;
	const char *exclude = getenv("FT_TEST_EXCLUDE");
	int err;

	err = create_all_cpu_call_rcu_data(0);
	if (err)
		diag("Per-CPU call_rcu() workers unavailable, using default.");

	rcu_register_thread();

	plan_tests(NR_TESTS);

	/* 1. Lifecycle & attributes */
	diag("Lifecycle & attribute tests");
	RUN_TEST(test_lifecycle_defaults);
	RUN_TEST(test_lifecycle_group_create_flavor);
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
	RUN_TEST(test_dup_chain_head_promotion);
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
	RUN_TEST(test_node_get_key);
	RUN_TEST(test_node_batch);
	RUN_TEST(test_batched_break);
	RUN_TEST(test_first_last_keylen_on_miss);
	RUN_TEST(test_node_get_key_no_list);
	RUN_TEST(test_list_off_ops);
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
	RUN_TEST(test_iteration_prefix_key);
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
	RUN_TEST(test_inequality_prefix_key);
	RUN_TEST(test_inequality_extends_prefix);
	RUN_TEST(test_inequality_deadend_empty_slot);
	RUN_TEST(test_iter_key_off_cell_to_descent);
	RUN_TEST(test_iter_copy_cell_positions);
	RUN_TEST(test_eager_key_len_offset_iter_key);
	RUN_TEST(test_nonidentity_ordered_iteration);
	RUN_TEST(test_inequality_empty_key);
	RUN_TEST(test_merge_ordered_fixed_root);
	RUN_TEST(test_merge_at_fixed_ordered_splice);
	RUN_TEST(test_merge_rerooted_glue_ordered_ext);
	RUN_TEST(test_merge_rerooted_glue_ordered_compressed);
	RUN_TEST(test_merge_rerooted_glue_ordered_key_shorter);
	RUN_TEST(test_merge_subpos_branch_reserve);
	RUN_TEST(test_merge_rerooted_nosplit_ordered_atnode);
	RUN_TEST(test_merge_rerooted_nosplit_ordered_branch);
	RUN_TEST(test_merge_rekey_same_trie);
	RUN_TEST(test_merge_rekey_same_trie_ordered);
	RUN_TEST(test_nonidentity_bulk_ops);
	RUN_TEST(test_merge_at_overflow);

	/* 9. Graft, graft_swap & detach */
	diag("Graft, graft_swap & detach tests");
	RUN_TEST(test_graft_basic);
	RUN_TEST(test_graft_displaced_external_compressed);
	RUN_TEST(test_graft_propagate_through_compressed);
	RUN_TEST(test_graft_canonicalize_at_intermediate_depth);
	RUN_TEST(test_graft_diverge_no_list);
	RUN_TEST(test_graft_at_root);
	RUN_TEST(test_graft_populated_error);
	RUN_TEST(test_graft_different_group_error);
	RUN_TEST(test_graft_self_error);
	RUN_TEST(test_graft_overflow_error);
	RUN_TEST(test_graft_swap_basic);
	RUN_TEST(test_graft_swap_into_empty);
	RUN_TEST(test_graft_swap_extract_empty_compressed_parent);
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
	RUN_TEST(test_iter_cache_mode_default);
	RUN_TEST(test_iter_uncached_forward);
	RUN_TEST(test_iter_uncached_reverse);
	RUN_TEST(test_iter_uncached_lookup_remove);
	RUN_TEST(test_iter_uncached_prefix_scoped);
	RUN_TEST(test_iter_cache_mode_switch);
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
	RUN_TEST(test_compress_detach_through);
	RUN_TEST(test_compress_iteration_order);
	RUN_TEST(test_compress_lookup_nth_through);
	RUN_TEST(test_compress_inequality_through);
	RUN_TEST(test_compress_nested);
	RUN_TEST(test_compress_replace_through);
	RUN_TEST(test_compress_recompact_external_nodes);
	RUN_TEST(test_compress_iter_reuse);
	RUN_TEST(test_compress_graft_swap_key_shorter);

	/* 14. Skip-compressed unit tests */
	diag("Skip-compressed unit tests");
	RUN_TEST(test_skip_compressed_unit);

	/* 14b. Speculative-validated lookup unit tests */
	diag("Speculative-validated lookup unit tests");
	RUN_TEST(test_specv_fixed_basic);
	RUN_TEST(test_specv_varlen_basic);
	RUN_TEST(test_specv_long_compressed_prefix);
	RUN_TEST(test_specv_mismatch_rejected);
	RUN_TEST(test_specv_prefix_key_mismatch);

	/* 15. Integrity verification tests */
	diag("Integrity verification tests");
	RUN_TEST(test_verify_empty);
	RUN_TEST(test_verify_recompact_grow);
	RUN_TEST(test_verify_recompact_shrink);
	RUN_TEST(test_verify_compress_split);
	RUN_TEST(test_verify_varlen_prefix_keys);
	RUN_TEST(test_verify_graft_detach);
	RUN_TEST(test_verify_compress_nested);
	RUN_TEST(test_verify_oscillation);

	/* 16. Compress/graft/remove integrity tests */
	diag("Compress/graft/remove integrity tests");
	RUN_TEST(test_density_remove_through_compress);
	RUN_TEST(test_density_graft_swap);
	RUN_TEST(test_density_stress);

	/* 17. Exclusive access discipline tests */
	diag("Exclusive access discipline tests");
	RUN_TEST(test_exclusive_default_is_concurrent);
	RUN_TEST(test_exclusive_attr_set_true);
	RUN_TEST(test_exclusive_make_transitions);
	RUN_TEST(test_exclusive_detach_always_exclusive);
	RUN_TEST(test_exclusive_graft_from_exclusive);
	RUN_TEST(test_exclusive_graft_swap_inherit_root);
	RUN_TEST(test_exclusive_graft_swap_inherit_non_root);

	/* 18. FEATURE_FT_EXCL_VALIDATE negative tests (SKIP if absent) */
	diag("Exclusive-access validator tests");
	RUN_TEST(test_excl_validate_writer_writer);
	RUN_TEST(test_excl_validate_excl_reader_writer);
	RUN_TEST(test_excl_validate_concurrent_reader_writer_no_rcu);

	/* 19. cds_ft_merge tests */
	diag("cds_ft_merge tests");
	RUN_TEST(test_merge_disjoint_prefix_fast_path);
	RUN_TEST(test_merge_overlapping_per_entry);
	RUN_TEST(test_merge_compressed_overlap);
	RUN_TEST(test_merge_empty_source);
	RUN_TEST(test_merge_at_root_empty_dst);
	RUN_TEST(test_merge_duplicate_chains);
	RUN_TEST(test_merge_invalid_arguments);
	RUN_TEST(test_merge_concurrent_source);
	RUN_TEST(test_merge_prefix_subtree);
	RUN_TEST(test_merge_fixed_length_fast_path);
	RUN_TEST(test_merge_prefix_overlap_per_entry);
	RUN_TEST(test_merge_at_varlen_rekey);
	RUN_TEST(test_merge_at_fixed_rekey);
	RUN_TEST(test_merge_at_overlap);
	RUN_TEST(test_merge_at_nonroot_src);
	RUN_TEST(test_merge_at_nonroot_dst);
	RUN_TEST(test_merge_at_external_dst);
	RUN_TEST(test_merge_at_external_dst_splice);
	RUN_TEST(test_merge_at_compressed_dst_internal);
	RUN_TEST(test_merge_at_compressed_dst_compressed);
	RUN_TEST(test_merge_at_key_shorter_dst_internal);
	RUN_TEST(test_merge_at_key_shorter_dst_splice);
	RUN_TEST(test_merge_at_key_shorter_src);
	RUN_TEST(test_merge_at_key_shorter_src_both);
	RUN_TEST(test_merge_at_compressed_parent_internal);
	RUN_TEST(test_merge_at_compressed_parent_splice);
	RUN_TEST(test_merge_at_fixed_unequal_keylen);

	diag("External-node arena allocator tests");
	RUN_TEST(test_external_arena_basic);
	RUN_TEST(test_optimize_attr_api);
	RUN_TEST(test_optimize_external_thp);
	RUN_TEST(test_optimize_group_functional);
	RUN_TEST(test_external_arena_alloc_free_recycle);
	RUN_TEST(test_external_arena_split);
	RUN_TEST(test_external_arena_merge);
	RUN_TEST(test_external_arena_multi_range);
	RUN_TEST(test_external_arena_alignment);
	RUN_TEST(test_external_arena_oversize_reject);

	/* Compaction */
	diag("Compaction tests");
	RUN_TEST(test_compact_integrity);
	RUN_TEST(test_compact_skip_over_leaf);
	RUN_TEST(test_compact_concurrent_mutation);
	RUN_TEST(test_compact_forgotten_end);
	RUN_TEST(test_compact_exclusive);
	RUN_TEST(test_remove_compressed_no_leak);
	RUN_TEST(test_compact_dense_full_node);
#ifdef FEATURE_FT_FAULT_INJECT
	RUN_TEST(test_split_oom_backpointer);
	RUN_TEST(test_merge_oom);
	RUN_TEST(test_merge_oom_empty_dst);
	RUN_TEST(test_merge_oom_diverged_dst);
	RUN_TEST(test_merge_oom_subpos_residual);
	RUN_TEST(test_merge_oom_subpos_glue);
	RUN_TEST(test_merge_oom_subpos_branch);
	RUN_TEST(test_merge_oom_rerooted_glue_ext);
	RUN_TEST(test_merge_oom_rerooted_glue_compressed);
	RUN_TEST(test_merge_oom_rerooted_nosplit_ext_atnode);
	RUN_TEST(test_merge_oom_rerooted_nosplit_ext_branch);
	RUN_TEST(test_merge_oom_rerooted_nosplit_compressed_atnode);
	RUN_TEST(test_merge_oom_rerooted_nosplit_compressed_branch);
	RUN_TEST(test_merge_oom_key_shorter_diverged_ext_glue);
	RUN_TEST(test_merge_oom_key_shorter_diverged_ext_atnode);
	RUN_TEST(test_merge_oom_key_shorter_diverged_internal_glue);
	RUN_TEST(test_merge_oom_key_shorter_diverged_internal_atnode);
	RUN_TEST(test_merge_oom_subpos_external_nodes);
	RUN_TEST(test_merge_oom_rekey);
	RUN_TEST(test_merge_oom_overlap);
	RUN_TEST(test_merge_oom_compressed);
	RUN_TEST(test_merge_oom_nonroot_src);
	RUN_TEST(test_merge_oom_nonroot_dst);
	RUN_TEST(test_merge_oom_external_dst);
	RUN_TEST(test_merge_oom_compressed_dst);
	RUN_TEST(test_merge_oom_key_shorter_dst);
	RUN_TEST(test_merge_oom_key_shorter_src);
	RUN_TEST(test_merge_oom_compressed_parent_dst);
	RUN_TEST(test_detach_oom_atomicity);
	RUN_TEST(test_attach_oom_skip_unwind);
	RUN_TEST(test_remove_all_oom_contract);
	RUN_TEST(test_remove_emptied_holder_traversal_oom);
#endif

	rcu_barrier();
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();

	return exit_status();
}
