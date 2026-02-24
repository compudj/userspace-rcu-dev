// SPDX-FileCopyrightText: 2012-2025 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_RCUJA_H
#define _URCU_RCUJA_H

/*
 * urcu/rcuja.h
 *
 * Userspace RCU library - RCU Judy Array
 *
 * Include this file _after_ including your URCU flavor.
 */

#include <urcu/compiler.h>
#include <urcu-call-rcu.h>
#include <urcu-flavor.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque types forward declarations. */
struct cds_ja;
struct cds_ja_attr;

/*
 * Duplicate nodes with the same key are chained into a singly-linked
 * list. The last item of this list has a NULL next pointer.
 */
struct cds_ja_node {
	struct cds_ja_node *next;
};

/*
 * The Judy array keys most significant byte is first, and least
 * significant byte is last. This corresponds to a big endian integer.
 */

/*
 * cds_ja_lookup - Look up by key.
 * @ja: The Judy array.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 *
 * Returns the first node of a duplicate chain if a match is found, else
 * returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ja_node *cds_ja_lookup(struct cds_ja *ja, const uint8_t *key, size_t key_len);

/*
 * cds_ja_lookup_lower_equal - Look up first node with key <= @key.
 * @ja: The Judy array.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key lower or equal to @key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ja_node *cds_ja_lookup_lower_equal(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ja_lookup_greater_equal - Look up first node with key >= @key.
 * @ja: The Judy array.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key greater or equal to @key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ja_node *cds_ja_lookup_greater_equal(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ja_lookup_lower_than - Look up first node with key < @key.
 * @ja: The Judy array.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key lower than @key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ja_node *cds_ja_lookup_lower_than(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ja_lookup_greater_than - Look up first node with key > @key.
 * @ja: The Judy array.
 * @key: Key to look up.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @result_key: Key found.
 * @result_key_len: Result key length.
 *
 * Returns the first node of a duplicate chain if a node is present in
 * the tree which has a key greater than @key, else returns NULL.
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 */
struct cds_ja_node *cds_ja_lookup_greater_than(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len);

/*
 * cds_ja_add - Add @node at @key, allowing duplicates.
 * @ja: The Judy array.
 * @key: Key at which @node should be added.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @node: Node to add.
 *
 * Returns 0 on success, negative error value on error.
 * A RCU read-side lock should be held across call to this function.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 */
int cds_ja_add(struct cds_ja *ja, const uint8_t *key, size_t key_len,
		struct cds_ja_node *node);

/*
 * cds_ja_add_unique - Add @node at @key, without duplicates.
 * @ja: The Judy array.
 * @key: Key at which @node should be added.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @node: Node to add.
 *
 * Returns @node if successfully added, else returns the already
 * existing node (acts as a RCU lookup).
 * A RCU read-side lock should be held across call to this function and
 * use of its return value.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 */
struct cds_ja_node *cds_ja_add_unique(struct cds_ja *ja, const uint8_t *key,
		size_t key_len, struct cds_ja_node *node);

/*
 * cds_ja_del - Remove @node at @key.
 * @ja: The Judy array.
 * @key: Key at which @node is expected.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 use the key length of the Judy array.
 * @node: Node to remove.
 *
 * Returns 0 on success, negative error value on error.
 * A RCU read-side lock should be held across call to this function.
 * Mutual exclusion between updates (add, add_unique, del) is the user
 * responsibility.
 */
int cds_ja_del(struct cds_ja *ja, const uint8_t *key, size_t key_len,
		struct cds_ja_node *node);

struct cds_ja *_cds_ja_create(const struct cds_ja_attr *attr,
		const struct rcu_flavor_struct *flavor);

/*
 * cds_ja_create - Create a Judy array.
 * @attr: Judy array attributes.
 *
 * The @attr pointer is used to specify the Judy array attributes. If
 * NULL, use default attribute values. The @attr can be destroyed
 * by the caller immediately after cds_ja_create() returns. The caller
 * keeps ownership of @attr. Default attributes select a 4 bytes key
 * length.
 *
 * Returns non-NULL pointer on success, else NULL on error.
 */
static inline
struct cds_ja *cds_ja_create(const struct cds_ja_attr *attr)
{
	return _cds_ja_create(attr, &rcu_flavor);
}

/*
 * cds_ja_destroy - Destroy a Judy array.
 * @ja: The Judy array.
 *
 * Returns 0 on success, negative error value on error.
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Judy array while it is being destroyed (ensured by the caller).
 */
int cds_ja_destroy(struct cds_ja *ja);

/*
 * cds_ja_key_len - Return the key length of a Judy array.
 * @ja: The Judy array.
 */
size_t cds_ja_key_len(const struct cds_ja *ja);

/*
 * cds_ja_max_key_len - Return the maximum key length of a Judy array.
 * @ja: The Judy array.
 *
 * Returns 0 if the Judy array has no maximum key length limit, > 0
 * otherwise.
 */
size_t cds_ja_max_key_len(const struct cds_ja *ja);


/*
 * cds_ja_attr_create - Create a Judy array attribute structure.
 */
struct cds_ja_attr *cds_ja_attr_create(void);

/*
 * cds_ja_attr_destroy - Destroy a Judy array attribute structure.
 * @attr: Judy array attributes.
 */
void cds_ja_attr_destroy(struct cds_ja_attr *attr);

/*
 * cds_ja_attr_set_key_len - Set Judy array key length attribute.
 * @attr: Judy array attributes.
 * @key_len: Key length.
 */
int cds_ja_attr_set_key_len(struct cds_ja_attr *attr, size_t key_len);

/*
 * cds_ja_attr_set_max_key_len - Set Judy array max key length attribute.
 * @attr: Judy array attributes.
 * @max_key_len: Maximum key length. 0 means no limit.
 */
int cds_ja_attr_set_max_key_len(struct cds_ja_attr *attr, size_t max_key_len);

/*
 * cds_ja_key_to_u64 - Convert a Judy array key to an unsigned 64-bit integer.
 * @ja: The Judy array.
 * @key: Key to convert from (input).
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Judy array.
 *
 * This helper function expects a Judy array with a fixed key length <= 8.
 */
uint64_t cds_ja_key_to_u64(const struct cds_ja *ja, const uint8_t *key, size_t key_len);

/*
 * cds_ja_u64_to_key - Convert an unsigned 64-bit integer to a Judy array key.
 * @ja: The Judy array.
 * @v: Value to convert from.
 * @key: Key to convert to. Should provide enough space to store "key length" bytes.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Judy array.
 *
 * This helper function expects a Judy array with a fixed key length <= 8.
 * It truncates the most significant bits beyond the Judy array key range.
 */
void cds_ja_u64_to_key(const struct cds_ja *ja, uint64_t v, uint8_t *key, size_t key_len);

/*
 * cds_ja_key_to_u32 - Convert a Judy array key to an unsigned 32-bit integer.
 * @ja: The Judy array.
 * @key: Key to convert from (input).
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Judy array.
 *
 * This helper function expects a Judy array with a fixed key length <= 4.
 */
uint32_t cds_ja_key_to_u32(const struct cds_ja *ja, const uint8_t *key, size_t key_len);

/*
 * cds_ja_u32_to_key - Convert an unsigned 32-bit integer to a Judy array key.
 * @ja: The Judy array.
 * @v: Value to convert from.
 * @key: Key to convert to. Should be at least as large as the Judy array key.
 * @key_len: Key length.
 *           key_len > 0 is an explicit key length.
 *           key_len == 0 implicitly uses the key length of the Judy array.
 *
 * This helper function expects a Judy array with a fixed key length <= 4.
 * It truncates the most significant bits beyond the Judy array key range.
 */
void cds_ja_u32_to_key(const struct cds_ja *ja, uint32_t v, uint8_t *key, size_t key_len);

/*
 * cds_ja_for_each_duplicate_rcu - Iterate through duplicates.
 * @pos: struct cds_ja_node *, start of duplicate list and loop cursor.
 *
 * Iterate through duplicates returned by cds_ja_lookup*()
 * This must be done while rcu_read_lock() is held.
 * Receives a struct cds_ja_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 * _NOT_ safe against node removal within iteration.
 */
#define cds_ja_for_each_duplicate_rcu(pos)					\
	for (; (pos) != NULL; (pos) = rcu_dereference((pos)->next))

/*
 * cds_ja_for_each_duplicate_safe - Iterate through duplicates.
 * @pos: struct cds_ja_node *, start of duplicate list and loop cursor.
 * @p: struct cds_ja_node *, temporary pointer to next.
 *
 * Iterate through duplicates returned by cds_ja_lookup*().
 * Safe against node removal within iteration.
 * This must be done while rcu_read_lock() is held.
 */
#define cds_ja_for_each_duplicate_safe_rcu(pos, p)			\
	for (; (pos) != NULL ?						\
			((p) = rcu_dereference((pos)->next), 1) : 0;	\
			(pos) = (p))

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCUJA_H */
