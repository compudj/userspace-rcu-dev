// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FRACTAL_TRIE_H
#define _URCU_FRACTAL_TRIE_H

/*
 * urcu/fractal-trie.h
 *
 * Userspace RCU library - Fractal Trie
 *
 * Include this file _after_ including your URCU flavor.
 */

#include <urcu/compiler.h>
#include <urcu-call-rcu.h>
#include <urcu-flavor.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque types forward declarations. */
struct cds_ft;
struct cds_ft_attr;
struct cds_ft_iter;

/* Fractal Trie lookup and mutation constants. */
#define CDS_FT_LEN_DEFAULT		SIZE_MAX

/* Fractal Trie attribute constants. */
#define CDS_FT_LEN_VARIABLE		SIZE_MAX
#define CDS_FT_MAX_LEN_UNLIMITED	SIZE_MAX
#define CDS_FT_KEY_MAP_SIZE		256

/*
 * Status codes returned by Fractal Trie operations.
 *
 * Success codes are >= 0. Error codes are < 0. Callers can test
 * (ret < 0) to catch all errors.
 */
enum cds_ft_status {
	/* Success return codes (>= 0). */
	CDS_FT_STATUS_OK			= 0,	/* Operation completed successfully. */
	CDS_FT_STATUS_NOT_FOUND			= 1,	/* No node found. */
	CDS_FT_STATUS_DUPLICATE_FOUND		= 2,	/* Duplicate node exists. */

	/* Error return codes (< 0). */
	CDS_FT_STATUS_INVALID_ARGUMENT_ERROR	= -1,	/* Invalid argument. */
	CDS_FT_STATUS_MEMORY_ERROR		= -2,	/* Memory allocation failure. */
	CDS_FT_STATUS_OVERFLOW_ERROR		= -3,	/* Buffer too small for key length. */
};

/*
 * Duplicate nodes with the same key are chained into a singly-linked
 * list. The last item of this list has a NULL next pointer.
 * The node needs to be zeroed or initialized with cds_ft_node_init
 * before being inserted into a Fractal Trie.
 *
 * Note that removal from a Fractal Trie does _not_ reset node->next,
 * because it can still be accessed by concurrent RCU readers. After
 * removal of a node, the user needs to re-initialize the node after
 * a grace period (e.g. via call_rcu() or synchronize_rcu()) before
 * being allowed to re-insert it.
 *
 * This structure is required to be naturally aligned.
 */
struct cds_ft_node {
	struct cds_ft_node *next;
};

/*
 * cds_ft_node_init - Initialize Fractal Trie node.
 * @node: The node.
 */
static inline
void cds_ft_node_init(struct cds_ft_node *node)
{
	node->next = NULL;
}

/*
 * The Fractal Trie keys most significant byte is first, and least
 * significant byte is last. This corresponds to a big endian integer.
 */

/*
 * Key-based lookup API
 */

/*
 * cds_ft_lookup_key - Look up a node by key.
 * @ft: The Fractal Trie.
 * @key: Pointer to the key (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes.
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @result_node: Node output. Set to the first node of the duplicate chain
 *               if a match is found, or NULL if not found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_partial_key - Look up by key, find closest partial match.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @match_len: Length of the matching sub-key (output).
 *             The returned node's key is the first @match_len
 *             bytes of @key.
 * @result_node: Node output. Set to the first node of a duplicate chain if
 *               a match is found. If no node matches the full key, set to
 *               the closest ancestor (partial match). Set to NULL if no
 *               match is found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *match_len,
		struct cds_ft_node **result_node);

/*
 * Iterator-based lookup API
 *
 * These functions use a cds_ft_iter to hold input key, output key,
 * result node, status, and backtracking state. Set the input key
 * with cds_ft_iter_set_key() before calling. On return, the iterator
 * holds the result key (cds_ft_iter_get_key()), result node
 * (cds_ft_iter_node()), status (cds_ft_iter_status()), and
 * backtracking path. The status is also returned by the function for
 * convenience.
 *
 * The RCU read-side lock must be held while calling these functions
 * and while accessing the returned node or reusing the iterator path.
 */

/*
 * cds_ft_lookup - Look up a node by key (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result node, status,
 *        and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_partial - Look up by key, find closest partial match
 *                         (iterator-based).
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result node, status,
 *        and backtracking path. The iterator's key length is set to
 *        the matching sub-key length, and the result node is the
 *        closest ancestor with external nodes.
 *
 * Returns CDS_FT_STATUS_OK on success (match found),
 * CDS_FT_STATUS_NOT_FOUND if no match, or a negative cds_ft_status
 * on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_le - Look up first node with key <= iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key <= the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_ge - Look up first node with key >= iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key >= the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_lt - Look up first node with key < iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key < the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_gt - Look up first node with key > iterator key.
 * @ft: The Fractal Trie.
 * @iter: Iterator with key set via cds_ft_iter_set_key().
 *        On return, the iterator holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key > the iterator key exists, or a negative
 * cds_ft_status on error. The status is also stored in the iterator
 * (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_first - Look up the node with the lowest key.
 * @ft: The Fractal Trie.
 * @iter: Iterator. On return, holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error. The
 * status is also stored in the iterator (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_lookup_last - Look up the node with the greatest key.
 * @ft: The Fractal Trie.
 * @iter: Iterator. On return, holds the result key, key length,
 *        node, status, and backtracking path.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error. The
 * status is also stored in the iterator (cds_ft_iter_status()).
 */
enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter);

/*
 * cds_ft_next - Find the next node in lexicographical order.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at the current node.
 *        On return, advanced to the next node. The iterator holds
 *        the result key, key length, node, status, and backtracking
 *        path.
 *
 * Equivalent to cds_ft_lookup_gt().
 */
static inline
enum cds_ft_status cds_ft_next(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return cds_ft_lookup_gt(ft, iter);
}

/*
 * cds_ft_prev - Find the previous node in lexicographical order.
 * @ft: The Fractal Trie.
 * @iter: Iterator positioned at the current node.
 *        On return, moved to the previous node. The iterator holds
 *        the result key, key length, node, status, and backtracking
 *        path.
 *
 * Equivalent to cds_ft_lookup_lt().
 */
static inline
enum cds_ft_status cds_ft_prev(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return cds_ft_lookup_lt(ft, iter);
}

/*
 * cds_ft_for_each - Iterate through all (or prefix-scoped) nodes in key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 *
 * The iterator holds the current key (cds_ft_iter_get_key()),
 * node (cds_ft_iter_node()), and status (cds_ft_iter_status()) at
 * each step. Check (cds_ft_iter_status(iter) < 0) after the loop
 * to detect errors.
 *
 * Use cds_ft_iter_set_key and cds_ft_iter_set_prefix_len on @iter to
 * perform prefix-scoped iteration.
 *
 * An RCU read-side lock must be held while using this macro.
 */
#define cds_ft_for_each(ft, iter)					\
	for (cds_ft_lookup_first((ft), (iter));				\
			cds_ft_iter_node(iter);				\
			cds_ft_next((ft), (iter)))

/*
 * cds_ft_for_each_reverse - Iterate through all (or prefix-scoped) nodes in reverse key order.
 * @ft: The Fractal Trie (struct cds_ft *).
 * @iter: Iterator (struct cds_ft_iter *), used as loop cursor.
 *
 * The iterator holds the current key (cds_ft_iter_get_key()),
 * node (cds_ft_iter_node()), and status (cds_ft_iter_status()) at
 * each step. Check (cds_ft_iter_status(iter) < 0) after the loop
 * to detect errors.
 *
 * Use cds_ft_iter_set_key and cds_ft_iter_set_prefix_len on @iter to
 * perform prefix-scoped iteration.
 *
 * An RCU read-side lock must be held while using this macro.
 */
#define cds_ft_for_each_reverse(ft, iter)				\
	for (cds_ft_lookup_last((ft), (iter));				\
			cds_ft_iter_node(iter);				\
			cds_ft_prev((ft), (iter)))

/*
 * Mutation API
 */

/*
 * cds_ft_insert - Insert @node at @key, allowing duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be inserted (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 *
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

/*
 * cds_ft_insert_unique - Insert @node at @key, without duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 * @result_node: Node output. Set to @node if successfully inserted. Set to
 *               the existing node if a duplicate exists. Set to NULL on
 *               error.
 *
 * Returns CDS_FT_STATUS_OK on success (node inserted, *@result_node
 * is @node). Returns CDS_FT_STATUS_DUPLICATE_FOUND if a duplicate
 * exists (*@result_node is the existing node). Returns a negative
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node);

/*
 * cds_ft_insert_replace - Insert @node at @key, replacing the existing
 *                         duplicate chain.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be inserted (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to insert.
 * @result_node: Node output. Set to the head of the previously existing
 *               duplicate chain if a key match existed, or NULL if no
 *               previous node existed at this key. Set to NULL on error.
 *
 * Atomically inserts @node at @key, replacing any existing duplicate
 * chain. If a chain was replaced, *@result_node points to the head
 * of the old chain. A grace period must be observed (e.g.,
 * synchronize_rcu, call_rcu) after success before reclaiming the old
 * chain's memory.
 *
 * Returns CDS_FT_STATUS_OK on success (node inserted, no prior node
 * existed). Returns CDS_FT_STATUS_DUPLICATE_FOUND on success when a
 * prior duplicate chain was replaced (*@result_node is the old head).
 * Returns a negative cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node);

/*
 * cds_ft_replace - Replace an existing node by a new node at the same key.
 * @ft: The Fractal Trie.
 * @iter: Iterator position at which @old_node is expected.
 *        If the iterator holds a valid path from a prior lookup,
 *        the replace operation may use it to avoid a full traversal.
 * @old_node: Node to replace. Must be currently present in the trie.
 * @new_node: Node to insert in place of @old_node. Must be
 *            initialized with cds_ft_node_init() before this call.
 *
 * Atomically replaces @old_node with @new_node in the duplicate
 * chain at the iterator's key position. The @new_node inherits the
 * position of @old_node in the chain. A grace period must be
 * observed (e.g., synchronize_rcu, call_rcu) after success before
 * reclaiming @old_node memory.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * @old_node is not found at the iterator position, or a negative
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node);

/*
 * cds_ft_remove - Remove @node at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Iterator position at which @node is expected.
 *        If the iterator holds a valid path from a prior lookup,
 *        the remove operation may use it to avoid a full traversal.
 * @node: Node to remove.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the node is not found, or a negative cds_ft_status on error.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming @node memory.
 * Mutual exclusion between updates (insert, insert_unique, remove) is
 * the user's responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node);

/*
 * cds_ft_remove_all - Remove the entire duplicate chain at @iter position.
 * @ft: The Fractal Trie.
 * @iter: Iterator position identifying the key.
 *        If the iterator holds a valid path from a prior lookup,
 *        the remove operation may use it to avoid a full traversal.
 * @result_node: Node output. Set to the head of the removed duplicate
 *               chain on success, or NULL if no node is found or on
 *               error.
 *
 * Removes the key and all associated duplicate nodes from the trie.
 * On success, *@result_node points to the head of the removed chain;
 * the caller can traverse it with cds_ft_for_each_duplicate_rcu()
 * under rcu_read_lock, or reclaim all nodes after a grace period.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming memory of any nodes in the chain.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node is found at the iterator position, or a negative
 * cds_ft_status on error.
 *
 * Mutual exclusion between updates (insert, insert_unique,
 * insert_replace, replace, remove, remove_all) is the user's
 * responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node);

/*
 * Trie lifecycle
 */

enum cds_ft_status _cds_ft_create(const struct cds_ft_attr *attr,
		struct cds_ft **result_ft,
		const struct rcu_flavor_struct *flavor);

/*
 * cds_ft_create - Create a Fractal Trie.
 * @attr: Fractal Trie attributes.
 * @result_ft: Fractal Trie output. Set to the newly created trie on
 *          success, or NULL on error.
 *
 * The @attr pointer is used to specify the Fractal Trie attributes. If
 * NULL, use default attribute values. The @attr can be destroyed
 * by the caller immediately after cds_ft_create() returns. The caller
 * keeps ownership of @attr. Default attributes select a variable key
 * length.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
static inline
enum cds_ft_status cds_ft_create(const struct cds_ft_attr *attr,
		struct cds_ft **result_ft)
{
	return _cds_ft_create(attr, result_ft, &rcu_flavor);
}

/*
 * cds_ft_destroy - Destroy a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * There should be no more concurrent insert, delete, nor look-up
 * performed on the Fractal Trie while it is being destroyed (ensured
 * by the caller).
 */
void cds_ft_destroy(struct cds_ft *ft);

/*
 * Trie properties
 */

/*
 * cds_ft_empty - Test whether a Fractal Trie contains any nodes.
 * @ft: The Fractal Trie.
 *
 * Returns true if the trie contains no nodes, false otherwise.
 *
 * This function uses a relaxed atomic load and does not require
 * the RCU read-side lock to be held. The result is a snapshot:
 * concurrent updates may change the emptiness state at any time.
 */
bool cds_ft_empty(struct cds_ft *ft);

/*
 * cds_ft_count - Return the number of external nodes in a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the total number of external nodes (user-visible nodes)
 * currently stored in the trie, including duplicates. Each
 * successful insert increments this count by one; each successful
 * remove decrements it by one; remove_all decrements it by the
 * number of nodes in the removed chain.
 *
 * This function uses a relaxed atomic load and does not require
 * the RCU read-side lock to be held. The result is a snapshot:
 * concurrent updates may change the count at any time.
 */
unsigned long cds_ft_count(const struct cds_ft *ft);

/*
 * cds_ft_key_len - Return the key length of a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the fixed key length if the trie is in fixed-length mode.
 * Returns CDS_FT_LEN_VARIABLE if the trie supports variable-length keys.
 */
size_t cds_ft_key_len(const struct cds_ft *ft);

/*
 * cds_ft_max_key_len - Return the maximum key length allowed by the trie.
 * @ft: The Fractal Trie.
 *
 * Returns the maximum length (in bytes) of any key that can be stored
 * in this trie instance.
 *
 * This value is intended for use by callers to allocate buffers for
 * output parameters (e.g., result_key). Even if the trie is configured
 * as "unlimited," this function returns a finite, implementation-defined
 * maximum.
 */
size_t cds_ft_max_key_len(const struct cds_ft *ft);

/*
 * cds_ft_key_map - Return the key map of a Fractal Trie.
 * @ft: The Fractal Trie.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (output, caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 *
 * Returns CDS_FT_STATUS_OK if there is a key mapping (output
 * parameters are populated). Returns CDS_FT_STATUS_NOT_FOUND if
 * the key map is the identity function.
 */
enum cds_ft_status cds_ft_key_map(const struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key);

/*
 * Attributes
 */

/*
 * cds_ft_attr_create - Create a Fractal Trie attribute structure.
 * @result: Attribute output. Set to the newly created attribute
 *          structure on success, or NULL on error.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result);

/*
 * cds_ft_attr_destroy - Destroy a Fractal Trie attribute structure.
 * @attr: Fractal Trie attributes.
 */
void cds_ft_attr_destroy(struct cds_ft_attr *attr);

/*
 * cds_ft_attr_set_key_len - Set Fractal Trie key length attribute.
 * @attr: Fractal Trie attributes.
 * @key_len: Key length.
 * - CDS_FT_LEN_VARIABLE for variable length keys.
 * - > 0 for fixed length keys.
 * - 0 for a trie fixed to NIL keys only.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len);

/*
 * cds_ft_attr_set_max_key_len - Set the maximum key length attribute.
 * @attr: Fractal Trie attributes.
 * @max_key_len: Maximum key length in bytes:
 * - n > 0: Limits keys to a maximum of n bytes.
 * - 0: Limits keys to length 0 (only NIL keys allowed).
 * - CDS_FT_MAX_LEN_UNLIMITED: No user-defined limit.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @max_key_len
 * exceeds implementation-defined limits.
 */
enum cds_ft_status cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len);

/*
 * cds_ft_attr_set_key_map - Set Fractal Trie key map attribute.
 * @attr: Fractal Trie attributes.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_attr_set_key_map(struct cds_ft_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key);

/*
 * Iterator management
 *
 * It is recommended that the user keeps a per-thread pool of
 * iterators in a TLS variable to minimize memory allocation.
 */

/*
 * cds_ft_iter_create - Allocate and initialize an iterator.
 * @ft: The Fractal Trie this iterator is bound to.
 * @result_iter: Iterator output. Set to the newly created iterator
 *               on success, or NULL on error.
 *
 * The iterator lifetime is bound to the Trie. The Trie must not be
 * destroyed while iterators to that trie exist.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter);

/*
 * cds_ft_iter_destroy - Free an iterator.
 * @iter: The iterator to free.
 */
void cds_ft_iter_destroy(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_get_key - Retrieve the current key from an iterator.
 * @iter: The iterator.
 * @result_key: Output buffer for the key.
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the key written (output).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the buffer is too small.
 */
enum cds_ft_status cds_ft_iter_get_key(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_iter_get_prefix - Retrieve the current prefix from an iterator.
 * @iter: The iterator.
 * @result_key: Output buffer for the prefix.
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the prefix written (output).
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_OVERFLOW_ERROR if the buffer is too small.
 */
enum cds_ft_status cds_ft_iter_get_prefix(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_iter_set_key - Set the key for the next lookup operation.
 * @iter: The iterator.
 * @key: The key to set.
 * @key_len: Key length in bytes.
 *
 * If the new key is a prefix of the current iterator key, the
 * internal backtracking path remains valid; otherwise it is
 * invalidated.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len);

/*
 * cds_ft_iter_set_prefix_len - Set the prefix length for iteration.
 * @iter: The iterator.
 * @prefix_len: Length of the key prefix for scoped traversal.
 *              Must be <= the current key length (set the key first).
 *              Set to 0 to iterate over the entire trie.
 *
 * Returns CDS_FT_STATUS_OK on success.
 * Returns CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if @prefix_len
 * exceeds the current key length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len);

/*
 * cds_ft_iter_reset - Reset an iterator to its initial state.
 * @iter: The iterator.
 *
 * Clears path, key, prefix, status, and node. The iterator remains
 * bound to its Fractal Trie.
 */
void cds_ft_iter_reset(struct cds_ft_iter *iter);

/*
 * cds_ft_iter_copy - Copy an iterator's state.
 * @dst: Destination iterator.
 * @src: Source iterator.
 *
 * Both iterators must be bound to the same Fractal Trie.
 */
void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src);

/*
 * cds_ft_iter_status - Return the status of the last operation on an iterator.
 * @iter: The iterator.
 *
 * Returns the cds_ft_status set by the last lookup or iteration
 * step performed on this iterator. Check (ret < 0) to detect errors.
 */
enum cds_ft_status cds_ft_iter_status(const struct cds_ft_iter *iter);

/*
 * cds_ft_iter_node - Return the current node from an iterator.
 * @iter: The iterator.
 *
 * Returns the node found by the last lookup, or NULL if the last
 * lookup found nothing.
 */
struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter);

/*
 * Key conversion helpers
 */

/*
 * cds_ft_key_to_u64 - Convert a Fractal Trie key to an unsigned 64-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 8.
 * For fixed-length tries, it behaves as a big-endian conversion of the
 * configured number of bytes.
 *
 * Returns 0 if @key_len exceeds 8 or is invalid for this trie's configuration.
 */
uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u64_to_key - Convert an unsigned 64-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 8.
 * It truncates the most significant bits beyond the Fractal Trie key range.
 *
 * No-op if @key_len exceeds 8 or is invalid for this trie's configuration.
 */
void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key, size_t key_len);

/*
 * cds_ft_key_to_u32 - Convert a Fractal Trie key to an unsigned 32-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input). May be NULL if @key_len is 0.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (returns 0).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 4.
 *
 * Returns 0 if @key_len exceeds 4 or is invalid for this trie's configuration.
 */
uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u32_to_key - Convert an unsigned 32-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to (output). Should provide enough space for the
 * trie's max key length.
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length.
 * - 0: NIL key (no-op).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 4.
 *
 * No-op if @key_len exceeds 4 or is invalid for this trie's configuration.
 */
void cds_ft_u32_to_key(const struct cds_ft *ft, uint32_t v, uint8_t *key, size_t key_len);

/*
 * Diagnostics
 */

/*
 * cds_ft_show - Print content of the Fractal Trie.
 * @ft: The Fractal Trie.
 * @out: File stream output.
 */
void cds_ft_show(const struct cds_ft *ft, FILE *out);

/*
 * cds_ft_show_stats - Print Fractal Trie statistics.
 * @ft: The Fractal Trie.
 * @out: File stream output.
 */
void cds_ft_show_stats(const struct cds_ft *ft, FILE *out);

/*
 * cds_ft_status_to_string - Convert a status value to string.
 * @status: The status value.
 *
 * Return a pointer to a const string representing the status.
 */
const char *cds_ft_status_to_string(enum cds_ft_status status);

/*
 * Duplicate traversal macros
 */

/*
 * cds_ft_for_each_duplicate_rcu - Iterate through duplicates.
 * @pos: struct cds_ft_node *, start of duplicate list and loop cursor.
 *
 * Iterate through duplicates returned by cds_ft_lookup*()
 * This must be done while rcu_read_lock() is held.
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 * _NOT_ safe against node removal within iteration.
 */
#define cds_ft_for_each_duplicate_rcu(pos)				\
	for (; (pos) != NULL; (pos) = rcu_dereference((pos)->next))

/*
 * cds_ft_for_each_duplicate_safe_rcu - Iterate through duplicates.
 * @pos: struct cds_ft_node *, start of duplicate list and loop cursor.
 * @p: struct cds_ft_node *, temporary pointer to next.
 *
 * Iterate through duplicates returned by cds_ft_lookup*().
 * Safe against node removal within iteration.
 * This must be done while rcu_read_lock() is held.
 */
#define cds_ft_for_each_duplicate_safe_rcu(pos, p)			\
	for (; (pos) != NULL ?						\
			((p) = rcu_dereference((pos)->next), 1) : 0;	\
			(pos) = (p))

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FRACTAL_TRIE_H */
