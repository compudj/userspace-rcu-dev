// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
// SPDX-License-Identifier: MIT

/*
 * LTTng-UST tracepoint provider for Fractal Trie debugging.
 *
 * This header is only active when the library is built with
 * -DFT_ENABLE_TRACING.  Without that define, cds_ft_tp.c is empty and
 * fractal-trie.c's FT_TP() macro expands to a no-op, so the library
 * has no lttng-ust dependency in default builds.
 *
 * To enable: rebuild with CFLAGS including -DFT_ENABLE_TRACING and
 * link with -llttng-ust -llttng-ust-common.
 *
 * Keys appear in tracepoint payloads as byte sequences
 * (lttng_ust_field_sequence_hex) — the trie is byte-oriented and
 * keys may have arbitrary length.
 *
 * Inequality-lookup mode, operation status, and compressed-handler
 * action are exposed as LTTng-UST enumerations so trace viewers
 * display the symbolic name rather than a raw integer.
 */

#ifdef FT_ENABLE_TRACING

/*
 * Helpers (struct/enum/function forward decls) are guarded separately
 * so they are only processed once, even though tracepoint-event.h
 * re-reads this header multiple times.  File-scope C declarations
 * are not legal in all re-read contexts.
 *
 * The ft_tp_node_kind enum lives in fractal-trie-internal.h so the
 * C-side return values and the LTTng enum values below cannot drift
 * apart.  ft_tp_node_kind() is defined in fractal-trie.c where
 * ft_types[] and the tag-bit helpers are visible; callers pass node
 * pointers and the kind field is computed in the tracepoint field
 * expressions below, not by the caller.
 */
#ifndef _FT_TP_H_HELPERS
#define _FT_TP_H_HELPERS
#include "fractal-trie-internal.h"
struct cds_ft_inode_flag;
uint16_t ft_tp_node_kind(struct cds_ft_inode_flag *nf);
uint16_t ft_tp_node_skip_len(struct cds_ft_inode_flag *nf);
#endif

#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER cds_ft

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "./cds_ft_tp.h"

#if !defined(_FT_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define _FT_TP_H

#include <lttng/tracepoint.h>
#include <stdint.h>

/*
 * Enumerations for human-readable trace output.
 */

LTTNG_UST_TRACEPOINT_ENUM(cds_ft, ft_lookup_mode,
	LTTNG_UST_TP_ENUM_VALUES(
		lttng_ust_field_enum_value("FT_LOOKUP_GE", 0)
		lttng_ust_field_enum_value("FT_LOOKUP_LE", 1)
		lttng_ust_field_enum_value("FT_LOOKUP_GT", 2)
		lttng_ust_field_enum_value("FT_LOOKUP_LT", 3)
	)
)

LTTNG_UST_TRACEPOINT_ENUM(cds_ft, ft_status,
	LTTNG_UST_TP_ENUM_VALUES(
		lttng_ust_field_enum_value("OK",				 0)
		lttng_ust_field_enum_value("NOT_FOUND",				 1)
		lttng_ust_field_enum_value("DUPLICATE_FOUND",			 2)
		lttng_ust_field_enum_value("INTERNAL_MATCH",			 3)
		lttng_ust_field_enum_value("INVALID_ARGUMENT_ERROR",		-1)
		lttng_ust_field_enum_value("MEMORY_ERROR",			-2)
		lttng_ust_field_enum_value("OVERFLOW_ERROR",			-3)
		lttng_ust_field_enum_value("BUSY_ERROR",			-4)
		lttng_ust_field_enum_value("POPULATED_ERROR",			-5)
	)
)

LTTNG_UST_TRACEPOINT_ENUM(cds_ft, ft_compressed_action,
	LTTNG_UST_TP_ENUM_VALUES(
		lttng_ust_field_enum_value("CONTINUE",		0)
		lttng_ust_field_enum_value("BREAK",		1)
		lttng_ust_field_enum_value("END",		2)
		lttng_ust_field_enum_value("GOING_UP",		3)
		lttng_ust_field_enum_value("DESCEND_CHILDREN",	4)
	)
)

/*
 * Node-kind enum: maps a tagged cds_ft_inode_flag pointer to a
 * symbolic label that encodes both the structural class and the
 * specific size variant.  See ft_tp_node_kind() in fractal-trie.c
 * for the mapping.  Values must stay in sync with the C-side enum
 * defined alongside the helper.
 *
 * Internal labels are <class>_<size_in_bytes>; size = 2^order
 * where order is the ft_types[i].order field.  Collapsed labels
 * encode the scan-size variant.  The exact labels realized in a
 * trace depend on build-time configuration (32-bit vs 64-bit); the
 * full superset is listed here so the same provider metadata works
 * on any build.
 */
LTTNG_UST_TRACEPOINT_ENUM(cds_ft, ft_tp_node_kind,
	LTTNG_UST_TP_ENUM_VALUES(
		lttng_ust_field_enum_value("NULL",		FT_TP_NODE_NULL)
		lttng_ust_field_enum_value("EXTERNAL",		FT_TP_NODE_EXTERNAL)
		lttng_ust_field_enum_value("COMPRESSED",	FT_TP_NODE_COMPRESSED)
		lttng_ust_field_enum_value("P1L_128",		FT_TP_NODE_P1L_128)
		lttng_ust_field_enum_value("P2L_32",		FT_TP_NODE_P2L_32)
		lttng_ust_field_enum_value("P2L_64",		FT_TP_NODE_P2L_64)
		lttng_ust_field_enum_value("P2L_128",		FT_TP_NODE_P2L_128)
		lttng_ust_field_enum_value("P1L_256",		FT_TP_NODE_P1L_256)
		lttng_ust_field_enum_value("P1L_512",		FT_TP_NODE_P1L_512)
		lttng_ust_field_enum_value("P1L_1024",		FT_TP_NODE_P1L_1024)
		lttng_ust_field_enum_value("PIGEON_1024",	FT_TP_NODE_PIGEON_1024)
		lttng_ust_field_enum_value("PIGEON_2048",	FT_TP_NODE_PIGEON_2048)
		lttng_ust_field_enum_value("UNKNOWN",		FT_TP_NODE_UNKNOWN)
	)
)

/*
 * Event classes for common signatures to avoid boilerplate.
 */

LTTNG_UST_TRACEPOINT_EVENT_CLASS(cds_ft, ft_key_event_class,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const uint8_t *, key,
		size_t, key_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_sequence_hex(uint8_t, key, key, size_t, key_len)
	)
)

/*
 * Iter-keyed class for public APIs that act through an iterator.
 * ft is emitted alongside iter even though iter is bound to ft at
 * creation time: flight-recorder snapshots are finite and the
 * iter_create event linking iter -> ft may have scrolled out of the
 * ring buffer by the time the snapshot is taken, so emitting ft
 * in each event keeps every event self-contained.
 */
LTTNG_UST_TRACEPOINT_EVENT_CLASS(cds_ft, ft_iter_key_event_class,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, iter,
		const uint8_t *, key,
		size_t, key_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, iter, (uintptr_t) iter)
		lttng_ust_field_sequence_hex(uint8_t, key, key, size_t, key_len)
	)
)

LTTNG_UST_TRACEPOINT_EVENT_CLASS(cds_ft, ft_status_event_class,
	LTTNG_UST_TP_ARGS(
		int, status
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_status, int, status, status)
	)
)

LTTNG_UST_TRACEPOINT_EVENT_CLASS(cds_ft, ft_node_event_class,
	LTTNG_UST_TP_ARGS(
		const void *, node
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, node, (uintptr_t) node)
	)
)

/*
 * Iter lifecycle/state class: every event carries both ft and iter
 * so that flight-recorder snapshots remain self-contained even when
 * the iter_create event has rotated out of the ring buffer.
 */
LTTNG_UST_TRACEPOINT_EVENT_CLASS(cds_ft, ft_iter_event_class,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, iter
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, iter, (uintptr_t) iter)
	)
)

/*
 * Compressed-node mutation events.
 */

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, compressed_publish,
	LTTNG_UST_TP_ARGS(
		const void *, cn,
		unsigned int, len,
		const uint8_t *, key_bytes,
		const void *, child,
		const void *, parent
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, cn, (uintptr_t) cn)
		lttng_ust_field_integer(unsigned int, len, len)
		lttng_ust_field_sequence_hex(uint8_t, key_bytes, key_bytes,
			unsigned int, len)
		lttng_ust_field_integer_hex(uintptr_t, child, (uintptr_t) child)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, child_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) child))
		lttng_ust_field_integer_hex(uintptr_t, parent, (uintptr_t) parent)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, compressed_split,
	LTTNG_UST_TP_ARGS(
		const char *, kind,
		const void *, old_cn,
		unsigned int, old_len,
		const void *, new_top,
		unsigned int, diverge_pos
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_string(kind, kind)
		lttng_ust_field_integer_hex(uintptr_t, old_cn, (uintptr_t) old_cn)
		lttng_ust_field_integer(unsigned int, old_len, old_len)
		lttng_ust_field_integer_hex(uintptr_t, new_top, (uintptr_t) new_top)
		lttng_ust_field_integer(unsigned int, diverge_pos, diverge_pos)
	)
)

LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_node_event_class, cds_ft,
	compressed_free,
	LTTNG_UST_TP_ARGS(const void *, node))

/*
 * Lifecycle events that bind ft to its group.  A trace consumer uses
 * group_create + ft_create to build an ft -> group map, then filters
 * the reconstruction by group (all cross-ft subtree moves via graft /
 * graft_swap are bounded to one group, so the group is the natural
 * reconstruction scope).
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, group_create,
	LTTNG_UST_TP_ARGS(
		const void *, ft_group
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft_group, (uintptr_t) ft_group)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, group_destroy,
	LTTNG_UST_TP_ARGS(
		const void *, ft_group
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft_group, (uintptr_t) ft_group)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ft_create,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, ft_group
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, ft_group, (uintptr_t) ft_group)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ft_destroy,
	LTTNG_UST_TP_ARGS(
		const void *, ft
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
	)
)

/*
 * root_publish: the trie's ft->root slot has been updated.
 *
 * Emitted at every site that writes ft->root, including the
 * initial cds_ft_alloc() assignment, all graft/swap/detach paths
 * that rcu_assign_pointer(ft->root, ...), and implicitly from
 * ft_publish_to_parent() when parent_nf is NULL (indicating a
 * root-slot publish, which has no structural parent node).
 *
 * The root_kind field lets a consumer distinguish the kind of
 * the new root (internal, compressed, collapsed, external) in
 * the same enum as tree_edge_set's parent_kind/child_kind, so
 * the consumer can tag the root node correctly without having
 * to observe a separate event.
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, root_publish,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, root
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, root, (uintptr_t) root)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, root_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) root))
	)
)

/*
 * Collapsed-node entry (re)publish.
 *
 * A collapsed node stores up to 256 entries, each with a variable-
 * length byte suffix and a child pointer.  Rather than emit the
 * entire entry table in a single event (which cannot easily
 * accommodate the nested variable-length suffixes), a separate
 * event per entry is emitted whenever a slot is installed or its
 * child pointer changes.  Consumers accumulate the state keyed by
 * (col, entry_idx).
 *
 * `dead` is 0 for a live entry (install / update) and non-zero
 * for an entry being logically removed.
 */
/*
 * Structural edge change: a parent node's child slot at ordinal
 * byte `key_byte` is now `child`.  Fired after ft_node_set_nth()
 * and ft_node_replace_ptr() succeed — these are the functions
 * where the real (parent, key_byte, child, parent_level) is
 * unambiguously known.  child == NULL means the slot was cleared
 * (detach / removal).  This is the tracepoint a consumer should
 * consume to reconstruct the trie topology; publish_to_parent is
 * a lower-level event about the underlying publish primitive and
 * its parent_nf is tied to skip-pointer bookkeeping, not the
 * structural parent.
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, tree_edge_set,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, parent,
		unsigned int, parent_level,
		uint8_t, key_byte,
		const void *, child
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, parent, (uintptr_t) parent)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, parent_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) parent))
		lttng_ust_field_integer(unsigned int, parent_level, parent_level)
		lttng_ust_field_integer(uint8_t, key_byte, key_byte)
		lttng_ust_field_integer_hex(uintptr_t, child, (uintptr_t) child)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, child_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) child))
		lttng_ust_field_integer(uint16_t, child_skip_len,
			ft_tp_node_skip_len((struct cds_ft_inode_flag *) child))
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, publish_to_parent,
	LTTNG_UST_TP_ARGS(
		const void *, parent_nf,
		const void *, slot,
		const void *, old_child,
		const void *, new_child
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, parent_nf, (uintptr_t) parent_nf)
		lttng_ust_field_integer_hex(uintptr_t, slot, (uintptr_t) slot)
		lttng_ust_field_integer_hex(uintptr_t, old_child, (uintptr_t) old_child)
		lttng_ust_field_integer_hex(uintptr_t, new_child, (uintptr_t) new_child)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, split_compressed_insert_enter,
	LTTNG_UST_TP_ARGS(
		const void *, cn,
		unsigned int, len,
		unsigned int, diverge_pos
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, cn, (uintptr_t) cn)
		lttng_ust_field_integer(unsigned int, len, len)
		lttng_ust_field_integer(unsigned int, diverge_pos, diverge_pos)
	)
)

/*
 * Inequality lookup events.
 */

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ineq_enter,
	LTTNG_UST_TP_ARGS(
		int, mode,
		const uint8_t *, key,
		size_t, key_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
		lttng_ust_field_sequence_hex(uint8_t, key, key, size_t, key_len)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ineq_compressed_enter,
	LTTNG_UST_TP_ARGS(
		const void *, cn,
		int, level,
		int, mode
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, cn, (uintptr_t) cn)
		lttng_ust_field_integer(int, level, level)
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ineq_compressed,
	LTTNG_UST_TP_ARGS(
		const void *, cn,
		int, level,
		int, cmp_result,
		unsigned int, mpos,
		int, action
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, cn, (uintptr_t) cn)
		lttng_ust_field_integer(int, level, level)
		lttng_ust_field_integer(int, cmp_result, cmp_result)
		lttng_ust_field_integer(unsigned int, mpos, mpos)
		lttng_ust_field_enum(cds_ft, ft_compressed_action, int, action, action)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ineq_going_up_step,
	LTTNG_UST_TP_ARGS(
		int, level,
		const void *, path_entry,
		int, found_sibling,
		uint8_t, ord_key
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(int, level, level)
		lttng_ust_field_integer_hex(uintptr_t, path_entry, (uintptr_t) path_entry)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, path_entry_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) path_entry))
		lttng_ust_field_integer(uint16_t, path_entry_skip_len,
			ft_tp_node_skip_len((struct cds_ft_inode_flag *) path_entry))
		lttng_ust_field_integer(int, found_sibling, found_sibling)
		lttng_ust_field_integer(uint8_t, ord_key, ord_key)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, ineq_result,
	LTTNG_UST_TP_ARGS(
		int, mode,
		const uint8_t *, query,
		size_t, query_len,
		const uint8_t *, result,
		size_t, result_len,
		int, status
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
		lttng_ust_field_sequence_hex(uint8_t, query, query, size_t, query_len)
		lttng_ust_field_sequence_hex(uint8_t, result, result, size_t, result_len)
		lttng_ust_field_enum(cds_ft, ft_status, int, status, status)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, fastpath_enter,
	LTTNG_UST_TP_ARGS(
		int, mode,
		const void *, node_flag,
		int, level_after
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
		lttng_ust_field_integer_hex(uintptr_t, node_flag, (uintptr_t) node_flag)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, node_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) node_flag))
		lttng_ust_field_integer(uint16_t, skip_len,
			ft_tp_node_skip_len((struct cds_ft_inode_flag *) node_flag))
		lttng_ust_field_integer(int, level_after, level_after)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, post_traversal,
	LTTNG_UST_TP_ARGS(
		int, mode,
		int, level,
		const void *, node_flag
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
		lttng_ust_field_integer(int, level, level)
		lttng_ust_field_integer_hex(uintptr_t, node_flag, (uintptr_t) node_flag)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, node_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) node_flag))
		lttng_ust_field_integer(uint16_t, skip_len,
			ft_tp_node_skip_len((struct cds_ft_inode_flag *) node_flag))
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, slowpath_enter,
	LTTNG_UST_TP_ARGS(
		int, mode,
		int, path_valid,
		int, path_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_enum(cds_ft, ft_lookup_mode, int, mode, mode)
		lttng_ust_field_integer(int, path_valid, path_valid)
		lttng_ust_field_integer(int, path_len, path_len)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, slowpath_step,
	LTTNG_UST_TP_ARGS(
		int, level,
		uint8_t, key_value,
		const void *, node_flag,
		int, broke
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(int, level, level)
		lttng_ust_field_integer(uint8_t, key_value, key_value)
		lttng_ust_field_integer_hex(uintptr_t, node_flag, (uintptr_t) node_flag)
		lttng_ust_field_enum(cds_ft, ft_tp_node_kind, uint16_t, node_kind,
			ft_tp_node_kind((struct cds_ft_inode_flag *) node_flag))
		lttng_ust_field_integer(uint16_t, skip_len,
			ft_tp_node_skip_len((struct cds_ft_inode_flag *) node_flag))
		lttng_ust_field_integer(int, broke, broke)
	)
)

/*
 * Test violation event.  query/returned are user-facing u64 values
 * already decoded by cds_ft_key_to_u64() in the test; not raw key
 * byte sequences.
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, violation,
	LTTNG_UST_TP_ARGS(
		uint64_t, query,
		uint64_t, returned
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(uint64_t, query, query)
		lttng_ust_field_integer(uint64_t, returned, returned)
	)
)

/*
 * Public API entries with key byte-sequence.
 */
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	insert_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	insert_unique_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	insert_replace_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	lookup_key_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	graft_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	graft_swap_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_key_event_class, cds_ft,
	detach_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const uint8_t *, key, size_t, key_len))

/*
 * Merge: keyless entry, status exit (defined below in the status-only block).
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, merge_enter,
	LTTNG_UST_TP_ARGS(const void *, dst_ft, const void *, src_ft),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, dst_ft, (uintptr_t) dst_ft)
		lttng_ust_field_integer_hex(uintptr_t, src_ft, (uintptr_t) src_ft)
	)
)

/* Iter-keyed public APIs: emit both ft and iter for snapshot safety. */
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_key_event_class, cds_ft,
	remove_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter,
		const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_key_event_class, cds_ft,
	replace_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter,
		const uint8_t *, key, size_t, key_len))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_key_event_class, cds_ft,
	lookup_enter,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter,
		const uint8_t *, key, size_t, key_len))

/* Public API exits (status enum). */
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	insert_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	insert_unique_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	insert_replace_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	remove_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	replace_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	graft_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	graft_swap_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	detach_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	merge_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	lookup_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	lookup_key_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	lookup_nth_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	lookup_nth_last_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	iter_skip_forward_exit,
	LTTNG_UST_TP_ARGS(int, status))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	iter_skip_reverse_exit,
	LTTNG_UST_TP_ARGS(int, status))

/*
 * Rank/skip ops — the "n" parameter is a numeric count, not a key.
 */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, lookup_nth_enter,
	LTTNG_UST_TP_ARGS(unsigned long, nth),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, nth, nth)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, lookup_nth_last_enter,
	LTTNG_UST_TP_ARGS(unsigned long, nth),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, nth, nth)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, iter_skip_forward_enter,
	LTTNG_UST_TP_ARGS(unsigned long, skip),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, skip, skip)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, iter_skip_reverse_enter,
	LTTNG_UST_TP_ARGS(unsigned long, skip),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, skip, skip)
	)
)

/* Counting ops. */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, count_entries,
	LTTNG_UST_TP_ARGS(unsigned long, count),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, count, count)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, count_prefix,
	LTTNG_UST_TP_ARGS(unsigned long, count),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer(unsigned long, count, count)
	)
)

/* Iterator lifecycle / state events: (ft, iter) pair on every event. */
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_event_class, cds_ft,
	iter_create,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_event_class, cds_ft,
	iter_destroy,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_event_class, cds_ft,
	iter_invalidate_path,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter))
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_iter_event_class, cds_ft,
	iter_reset,
	LTTNG_UST_TP_ARGS(const void *, ft, const void *, iter))

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, iter_set_key_enter,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, iter,
		const uint8_t *, key,
		size_t, key_len,
		int, prev_key_len,
		int, prev_path_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, iter, (uintptr_t) iter)
		lttng_ust_field_sequence_hex(uint8_t, key, key, size_t, key_len)
		lttng_ust_field_integer(int, prev_key_len, prev_key_len)
		lttng_ust_field_integer(int, prev_path_len, prev_path_len)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, iter_set_key_exit,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, iter,
		int, subset,
		int, path_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, iter, (uintptr_t) iter)
		lttng_ust_field_integer(int, subset, subset)
		lttng_ust_field_integer(int, path_len, path_len)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, iter_set_prefix_len,
	LTTNG_UST_TP_ARGS(
		const void *, ft,
		const void *, iter,
		int, prefix_len
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, ft, (uintptr_t) ft)
		lttng_ust_field_integer_hex(uintptr_t, iter, (uintptr_t) iter)
		lttng_ust_field_integer(int, prefix_len, prefix_len)
	)
)

/* Internal mutation primitives. */
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, attach_node_enter,
	LTTNG_UST_TP_ARGS(
		const void *, attach_node,
		const void *, old_node,
		unsigned int, level
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, attach_node, (uintptr_t) attach_node)
		lttng_ust_field_integer_hex(uintptr_t, old_node, (uintptr_t) old_node)
		lttng_ust_field_integer(unsigned int, level, level)
	)
)
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	attach_node_exit,
	LTTNG_UST_TP_ARGS(int, status))

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, detach_node_enter,
	LTTNG_UST_TP_ARGS(
		const void *, detach_node,
		unsigned int, depth
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, detach_node, (uintptr_t) detach_node)
		lttng_ust_field_integer(unsigned int, depth, depth)
	)
)
LTTNG_UST_TRACEPOINT_EVENT_INSTANCE(cds_ft, ft_status_event_class, cds_ft,
	detach_node_exit,
	LTTNG_UST_TP_ARGS(int, status))

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, node_recompact,
	LTTNG_UST_TP_ARGS(
		const void *, old_node,
		const void *, new_node,
		int, new_type
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, old_node, (uintptr_t) old_node)
		lttng_ust_field_integer_hex(uintptr_t, new_node, (uintptr_t) new_node)
		lttng_ust_field_integer(int, new_type, new_type)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, chain_node,
	LTTNG_UST_TP_ARGS(
		const void *, last_node,
		const void *, node
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, last_node, (uintptr_t) last_node)
		lttng_ust_field_integer_hex(uintptr_t, node, (uintptr_t) node)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, unchain_node,
	LTTNG_UST_TP_ARGS(
		const void *, head_slot,
		const void *, node,
		int, was_head
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, head_slot, (uintptr_t) head_slot)
		lttng_ust_field_integer_hex(uintptr_t, node, (uintptr_t) node)
		lttng_ust_field_integer(int, was_head, was_head)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, set_parent,
	LTTNG_UST_TP_ARGS(
		const void *, child,
		const void *, parent
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, child, (uintptr_t) child)
		lttng_ust_field_integer_hex(uintptr_t, parent, (uintptr_t) parent)
	)
)
LTTNG_UST_TRACEPOINT_EVENT(cds_ft, metadata_set_external_nodes,
	LTTNG_UST_TP_ARGS(
		const void *, node,
		const void *, external_nodes
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, node, (uintptr_t) node)
		lttng_ust_field_integer_hex(uintptr_t, external_nodes, (uintptr_t) external_nodes)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, inv_violation,
	LTTNG_UST_TP_ARGS(
		const char *, test_name,
		const char *, message
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_string(test, test_name)
		lttng_ust_field_string(msg, message)
	)
)

LTTNG_UST_TRACEPOINT_EVENT(cds_ft, collapsed_suffix_len_bad,
	LTTNG_UST_TP_ARGS(
		const void *, col,
		unsigned int, i,
		unsigned int, nr_entries,
		unsigned int, start,
		unsigned int, end,
		uint8_t, data_i,
		uint8_t, data_prev
	),
	LTTNG_UST_TP_FIELDS(
		lttng_ust_field_integer_hex(uintptr_t, col, (uintptr_t) col)
		lttng_ust_field_integer(unsigned int, i, i)
		lttng_ust_field_integer(unsigned int, nr_entries, nr_entries)
		lttng_ust_field_integer(unsigned int, start, start)
		lttng_ust_field_integer(unsigned int, end, end)
		lttng_ust_field_integer(uint8_t, data_i, data_i)
		lttng_ust_field_integer(uint8_t, data_prev, data_prev)
	)
)

#endif /* _FT_TP_H */

#include <lttng/tracepoint-event.h>

#endif /* FT_ENABLE_TRACING */
