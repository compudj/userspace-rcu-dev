// SPDX-FileCopyrightText: 2012-2025 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
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
 * cds_ft_lookup - Look up by key.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 *
 * Returns the first node of a duplicate chain if a match is found, else
 * returns NULL.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup(struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_lookup_partial - Look up by key, find closest partial match.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @match_len: Length of (partial) match.
 *
 * Returns the first node of a duplicate chain if a match is found, else
 * returns NULL. If no node it found to completely match the key, the
 * closest ancestor (partial match) is returned.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_partial(struct cds_ft *ft, const uint8_t *key, size_t key_len, size_t *match_len);

/*
 * cds_ft_lookup_lower_equal - Look up first node with key <= @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key lower or equal to @key, else returns NULL.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_lower_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_lookup_greater_equal - Look up first node with key >= @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key greater or equal to @key, else returns NULL.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_greater_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_lookup_lower_than - Look up first node with key < @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key lower than @key, else returns NULL.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_lower_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_lookup_greater_than - Look up first node with key > @key.
 * @ft: The Fractal Trie.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key greater than @key, else returns NULL.
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_greater_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_lookup_first - Look up node with lowest key.
 * @ft: The Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has the lowest key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_first(struct cds_ft *ft,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_lookup_last - Look up node with greatest key.
 * @ft: The Fractal Trie.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has the greatest key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ft_node *cds_ft_lookup_last(struct cds_ft *ft,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ft_add - Add @node at @key, allowing duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @node: Node to add.
 *
 * Returns 0 on success, negative error value on error.
 * A RCU read-side lock should be held across call to this function.
 * Return -EINVAL if the key_len is larger than the Fractal Trie max key length.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 */
int cds_ft_add(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct cds_ft_node *node);

/*
 * cds_ft_add_unique - Add @node at @key, without duplicates.
 * @ft: The Fractal Trie.
 * @key: Key at which @node should be added.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @node: Node to add.
 *
 * Returns @node if successfully added, else returns the already
 * existing node (acts as a RCU lookup).
 * Return NULL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 */
struct cds_ft_node *cds_ft_add_unique(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct cds_ft_node *node);

/*
 * cds_ft_del - Remove @node at @key.
 * @ft: The Fractal Trie.
 * @key: Key at which @node is expected.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Fractal Trie.
 * @node: Node to remove.
 *
 * Returns 0 on success, negative error value on error.
 * Return -EINVAL if the key_len is larger than the Fractal Trie max key length.
 * A RCU read-side lock should be held across call to this function.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 * The caller needs to wait for a grace period (synchronize_rcu or
 * call_rcu) after a successful cds_ft_del before reclaiming the memory
 * used by @node.
 */
int cds_ft_del(struct cds_ft *ft, const uint8_t *key, size_t key_len,
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
 * keeps ownership of @attr. Default attributes select a 4 bytes key
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
 * Returns 0 if the Fractal Trie uses variable length keys, > 0
 * otherwise.
 */
size_t cds_ft_key_len(const struct cds_ft *ft);

/*
 * cds_ft_max_key_len - Return the maximum key length of a Fractal Trie.
 * @ft: The Fractal Trie.
 *
 * Returns the Fractal Trie maximum key length limit.
 */
size_t cds_ft_max_key_len(const struct cds_ft *ft);

/*
 * cds_ft_key_map - Return the key map of a Fractal Trie.
 * @ft: The Fractal Trie.
 * @key_to_ordinal: Mapping from external key to ordered values. (output)
 * @ordinal_to_key: Mapping from ordered values to external key. (output)
 *
 * Returns -ENOENT if key map is identity function. Populate the output
 * parameters and return 0 if there is a key mapping.
 */
int cds_ft_key_map(struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key);

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
 * @key_len: Key length. Set to 0 for variable length keys, > 0
 *           otherwise.
 */
int cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len);

/*
 * cds_ft_attr_set_max_key_len - Set Fractal Trie max key length attribute.
 * @attr: Fractal Trie attributes.
 * @max_key_len: Maximum key length. 0 means no limit.
 * Returns 0 if the limit is set successfully, -EINVAL if the requested
 * limit is larger than the maximum limit.
 */
int cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len);

/*
 * cds_ft_attr_set_key_map - Set Fractal Trie key map attribute.
 * @attr: Fractal Trie attributes.
 * @key_to_ordinal: Mapping from external key to ordered values.
 * @ordinal_to_key: Mapping from ordered values to external key.
 */
int cds_ft_attr_set_key_map(struct cds_ft_attr *attr, const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key);

/*
 * cds_ft_key_to_u64 - Convert a Fractal Trie key to an unsigned 64-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input).
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Fractal Trie.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 8.
 */
uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u64_to_key - Convert an unsigned 64-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to. Should provide enough space to store "key length" bytes.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Fractal Trie.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 8.
 * It truncates the most significant bits beyond the Fractal Trie key range.
 */
void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key, size_t key_len);

/*
 * cds_ft_key_to_u32 - Convert a Fractal Trie key to an unsigned 32-bit integer.
 * @ft: The Fractal Trie.
 * @key: Key to convert from (input).
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Fractal Trie.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 4.
 */
uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key, size_t key_len);

/*
 * cds_ft_u32_to_key - Convert an unsigned 32-bit integer to a Fractal Trie key.
 * @ft: The Fractal Trie.
 * @v: Value to convert from.
 * @key: Key to convert to. Should be at least as large as the Fractal Trie key.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Fractal Trie.
 *
 * This helper function expects a Fractal Trie with a fixed key length <= 4.
 * It truncates the most significant bits beyond the Fractal Trie key range.
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
void cds_ft_show_stats(const struct cds_ft *ft, FILE *out);;;;

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
 * cds_ft_for_each_duplicate_safe - Iterate through duplicates.
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
