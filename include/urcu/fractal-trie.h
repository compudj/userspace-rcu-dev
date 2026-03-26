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
 * Duplicate nodes with the same key are chained into a singly-linked
 * list. The last item of this list has a NULL next pointer.
 */
struct cds_ft_node {
	struct cds_ft_node *next;
};

/*
 * The Fractal Trie keys most significant byte is first, and least
 * significant byte is last. This corresponds to a big endian integer.
 */

/*
 * cds_ft_lookup - Look up a node by key.
 * @ft: The Fractal Trie.
 * @key: Pointer to the key (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes.
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 *
 * Returns the first node of the duplicate chain if a match is found.
 * Returns NULL if no match is found, or if @key_len is invalid for
 * this trie's configuration.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup(struct cds_ft *ft, const uint8_t *key, size_t key_len);

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
 *
 * Returns the first node of a duplicate chain if a match is found. If no node
 * matches the full key, the closest ancestor (partial match) is returned.
 * Returns NULL if the trie is empty, or if the key_len is invalid for
 * the trie's configuration.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_partial(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *prefix_len);

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
 *
 * Returns the first node of a duplicate chain if a node exists with a key
 * lower than or equal to @key. Returns NULL if no such node is found
 * or if @key_len is invalid.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_lower_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

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
 *
 * Returns the first node of a duplicate chain if a node exists with a key
 * greater than or equal to @key. Returns NULL if no such node is found
 * or if @key_len is invalid.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_greater_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

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
 *
 * Returns the first node of a duplicate chain if a node exists with a key
 * lower than @key. Returns NULL if no such node is found or if @key_len
 * is invalid.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_lower_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

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
 *
 * Returns the first node of a duplicate chain if a node exists with a key
 * greater than @key. Returns NULL if no such node is found or if @key_len
 * is invalid.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_greater_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_next - Find the next lexicographical node.
 * Uses the provided key/len as the current position.
 */
static inline
struct cds_ft_node *cds_ft_next(struct cds_ft *ft,
		uint8_t *key, size_t key_max_len, size_t *key_len)
{
	return cds_ft_lookup_greater_than(ft, key, *key_len, key, key_max_len, key_len);
}

/*
 * cds_ft_prev - Find the previous lexicographical node.
 * Uses the provided key/len as the current position.
 */
static inline
struct cds_ft_node *cds_ft_prev(struct cds_ft *ft,
		uint8_t *key, size_t key_max_len, size_t *key_len)
{
	return cds_ft_lookup_lower_than(ft, key, *key_len, key, key_max_len, key_len);
}

/*
 * cds_ft_lookup_first - Look up node with lowest key.
 * @ft: The Fractal Trie.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Result key length (output).
 *
 * Returns the first node of a duplicate chain if a node exists with the
 * lowest key within the trie. Returns NULL if no such node is found.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_first(struct cds_ft *ft,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_lookup_last - Look up node with greatest key.
 * @ft: The Fractal Trie.
 * @result_key: Found key (output buffer).
 * @result_key_max_len: Size of the @result_key buffer.
 * @result_key_len: Result key length (output).
 *
 * Returns the first node of a duplicate chain if a node exists with the
 * greatest key within the trie. Returns NULL if no such node is found.
 *
 * An RCU read-side lock must be held while calling this function and
 * while accessing the returned node.
 */
struct cds_ft_node *cds_ft_lookup_last(struct cds_ft *ft,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len);

/*
 * cds_ft_insert - Insert @node at @key, allowing duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to add.
 *
 * Returns 0 on success, negative error value on error (e.g., -EINVAL if
 * @key_len is invalid).
 * Mutual exclusion between updates (add, add_unique, del) is the user's
 * responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
int cds_ft_insert(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

/*
 * cds_ft_insert_unique - Insert @node at @key, without duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added (may be NULL if @key_len is 0).
 * @key_len: Key length in bytes:
 * - > 0: Explicit key length (must not exceed trie's max length).
 * - 0: NIL key (zero-length).
 * - CDS_FT_LEN_DEFAULT: Use the trie's configured fixed length.
 * @node: Node to add.
 *
 * Returns @node if successfully added. If a duplicate exists, returns the
 * existing node. Returns NULL if @key_len is invalid.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 * An RCU read-side lock must be held while calling this function.
 */
struct cds_ft_node *cds_ft_insert_unique(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct cds_ft_node *node);

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
 * Returns 0 on success, negative error value on error.
 * A grace period must be observed (e.g., synchronize_rcu, call_rcu)
 * after success before reclaiming @node memory.
 * An RCU read-side lock must be held while calling this function.
 */
int cds_ft_remove(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

struct cds_ft *_cds_ft_create(const struct cds_ft_attr *attr,
		const struct rcu_flavor_struct *flavor);

/*
 * cds_ft_create - Create a Fractal Trie.
 * @attr: Fractal Trie attributes.
 *
 * The @attr pointer is used to specify the Fractal Trie attributes. If
 * NULL, use default attribute values. The @attr can be destroyed
 * by the caller immediately after cds_ft_create() returns. The caller
 * keeps ownership of @attr. Default attributes select a variable key
 * length.
 *
 * Returns non-NULL pointer on success, else NULL on error.
 */
static inline
struct cds_ft *cds_ft_create(const struct cds_ft_attr *attr)
{
	return _cds_ft_create(attr, &rcu_flavor);
}

/*
 * cds_ft_destroy - Destroy a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns 0 on success, negative error value on error.
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the caller).
 */
int cds_ft_destroy(struct cds_ft *ft);

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
 * Returns -ENOENT if key map is identity function. Populate the output
 * parameters and return 0 if there is a key mapping.
 */
int cds_ft_key_map(const struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key);

/*
 * cds_ft_attr_create - Create a Fractal Trie attribute structure.
 */
struct cds_ft_attr *cds_ft_attr_create(void);

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
 */
int cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len);

/*
 * cds_ft_attr_set_max_key_len - Set the maximum key length attribute.
 * @attr: Fractal Trie attributes.
 * @max_key_len: Maximum key length in bytes:
 * - n > 0: Limits keys to a maximum of n bytes.
 * - 0: Limits keys to length 0 (only NIL keys allowed).
 * - CDS_FT_MAX_LEN_UNLIMITED: No user-defined limit.
 *
 * Returns 0 on success.
 * Returns -EINVAL if @max_key_len exceeds implementation-defined limits.
 */
int cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len);

/*
 * cds_ft_attr_set_key_map - Set Fractal Trie key map attribute.
 * @attr: Fractal Trie attributes.
 * @key_to_ordinal: Mapping from external key to ordered values.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 * @ordinal_to_key: Mapping from ordered values to external key.
 *                  (caller-provided array of CDS_FT_KEY_MAP_SIZE elements)
 */
int cds_ft_attr_set_key_map(struct cds_ft_attr *attr,
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
 * cds_ft_for_each_duplicate_rcu - Iterate through duplicates.
 * @pos: struct cds_ft_node *, start of duplicate list and loop cursor.
 *
 * Iterate through duplicates returned by cds_ft_lookup*()
 * This must be done while rcu_read_lock() is held.
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 * _NOT_ safe against node removal within iteration.
 */
#define cds_ft_for_each_duplicate_rcu(pos)					\
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
