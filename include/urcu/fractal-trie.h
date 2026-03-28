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

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque types forward declarations. */
struct cds_ft;
struct cds_ft_attr;

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
 * Output pointer convention
 * -------------------------
 * All functions write to their output pointers unconditionally, even
 * on error. On error or not-found, node output pointers are set to
 * NULL. This guarantees that callers can rely on the output pointer
 * value without checking the return status first, which is
 * particularly useful for loop constructs.
 */

/*
 * cds_ft_lookup - Look up a node by key.
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
enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_partial - Look up by key, find closest partial match.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @prefix_len: Length of the matching prefix key (output).
 *              The returned node's key is the first @prefix_len
 *              bytes of @key.
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
enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *prefix_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_lower_equal - Look up first node with key <= @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the found key (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if found, or NULL if not found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key <= @key exists, or a negative cds_ft_status on
 * error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_lower_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_greater_equal - Look up first node with key >= @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the found key (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if found, or NULL if not found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key >= @key exists, or a negative cds_ft_status on
 * error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_greater_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_lower_than - Look up first node with key < @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the found key (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if found, or NULL if not found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key < @key exists, or a negative cds_ft_status on
 * error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_lower_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_greater_than - Look up first node with key > @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Length of the found key (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *          if found, or NULL if not found or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * no node with key > @key exists, or a negative cds_ft_status on
 * error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_greater_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_next - Find the next lexicographical node.
 * Uses the provided key/len as the current position.
 */
static inline
enum cds_ft_status cds_ft_next(struct cds_ft *ft,
		uint8_t *key, size_t key_max_len, size_t *key_len,
		struct cds_ft_node **result_node)
{
	return cds_ft_lookup_greater_than(ft, key, *key_len, key,
			key_max_len, key_len, result_node);
}

/*
 * cds_ft_prev - Find the previous lexicographical node.
 * Uses the provided key/len as the current position.
 */
static inline
enum cds_ft_status cds_ft_prev(struct cds_ft *ft,
		uint8_t *key, size_t key_max_len, size_t *key_len,
		struct cds_ft_node **result_node)
{
	return cds_ft_lookup_lower_than(ft, key, *key_len, key,
			key_max_len, key_len, result_node);
}

/*
 * cds_ft_lookup_first - Look up node with lowest key.
 * @ft: The Fractal Trie.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Result key length (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if found, or NULL if the trie is empty or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_lookup_last - Look up node with greatest key.
 * @ft: The Fractal Trie.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Result key length (output).
 * @result_node: Node output. Set to the first node of a duplicate chain
 *               if found, or NULL if the trie is empty or on error.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the trie is empty, or a negative cds_ft_status on error.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len,
		struct cds_ft_node **result_node);

/*
 * cds_ft_for_each - Iterate through all nodes in key order.
 * @ft: The Fractal Trie.
 * @node: struct cds_ft_node *, loop cursor.
 * @key: uint8_t[], key buffer used across iterations.
 * @key_max_len: Size of the @key buffer.
 * @key_len: size_t, tracks the current key length across iterations.
 * @status: enum cds_ft_status, set by each iteration step.
 *          Check (status < 0) after the loop to detect errors.
 *
 * An RCU read-side lock must be held while using this macro.
 */
#define cds_ft_for_each(ft, node, key, key_max_len, key_len, status)	\
	for ((status) = cds_ft_lookup_first((ft), (key), (key_max_len),	\
				&(key_len), &(node));			\
			(node);						\
			(status) = cds_ft_next((ft), (key),		\
				(key_max_len), &(key_len), &(node)))

/*
 * cds_ft_for_each_reverse - Iterate through all nodes in reverse key order.
 * @ft: The Fractal Trie.
 * @node: struct cds_ft_node *, loop cursor.
 * @key: uint8_t[], key buffer used across iterations.
 * @key_max_len: Size of the @key buffer.
 * @key_len: size_t, tracks the current key length across iterations.
 * @status: enum cds_ft_status, set by each iteration step.
 *          Check (status < 0) after the loop to detect errors.
 *
 * An RCU read-side lock must be held while using this macro.
 */
#define cds_ft_for_each_reverse(ft, node, key, key_max_len, key_len, status) \
	for ((status) = cds_ft_lookup_last((ft), (key), (key_max_len),	\
				&(key_len), &(node));			\
			(node);						\
			(status) = cds_ft_prev((ft), (key),		\
				(key_max_len), &(key_len), &(node)))

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
 * cds_ft_remove - Remove @node at @key.
 * @ft: The Fractal Trie.
 * @key: Key at which @node is expected (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
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
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

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
