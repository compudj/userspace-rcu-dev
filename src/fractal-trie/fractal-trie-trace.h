// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FT_TRACE_H
#define _URCU_FT_TRACE_H

/*
 * src/fractal-trie-trace.h
 *
 * Userspace RCU library - Fractal Trie tracing macros
 *
 * FT_TP() and friends expand to LTTng-UST tracepoints when FT_ENABLE_TRACING
 * is defined, and to no-ops otherwise (arguments discarded, no code emitted).
 * The key-payload variants resolve the key length before the LTTng sequence
 * field serializes it.  These are macros, so the symbols they reference
 * (ft_key_len(), iter_key(), iter fields, the CDS_FT_LEN_* sentinels) only
 * need to be in scope at each FT_TP* use site in fractal-trie.c, not here.
 *
 * Included by fractal-trie.c AFTER fractal-trie-internal.h.
 */

#ifdef FT_ENABLE_TRACING
#include <stdio.h>
#include <stdlib.h>
#include "cds_ft_tp.h"

/*
 * FAST STOP.  `lttng stop` is a round trip to the session daemon and
 * `lttng snapshot record` a fork+exec -- MILLISECONDS.  On a workload emitting
 * millions of events per second a 64 KiB ring wraps many times over in that
 * window, so a violation site that calls them directly dumps a snapshot from
 * which its own violation event has already been evicted.  (Measured: 6
 * snapshots, 0 violation events in any of them.)
 *
 * So freeze emission in-process FIRST, with a single relaxed store, and only
 * then pay for the slow calls.  Every emitter checks the flag, so the ring stops
 * advancing within a store-buffer drain of the violation instead of within a
 * process spawn.
 *
 * ORDER AT THE SITE: emit the violation, THEN freeze -- the violation must be
 * the last event IN the buffer, not the first one dropped by its own flag.
 *
 * SINGLE-PROCESS ONLY, and that is exactly the case here: the flag is
 * process-local, so it silences this process's emitters and no others.  A
 * multi-process session still needs `lttng stop` to freeze its peers, which is
 * why the slow path stays.
 *
 * RELAXED is deliberate.  This is not ordering data -- it is a "stop shouting"
 * hint whose only requirement is to become visible fast; on x86 a relaxed store
 * lands in tens of nanoseconds, and a straggler event or two from a thread that
 * has not yet observed it is harmless next to a full ring wrap.
 */
extern int ft_trace_frozen;

#define FT_TRACE_FREEZE()	uatomic_store(&ft_trace_frozen, 1, CMM_RELAXED)

/*
 * ONE SESSION PER PROCESS, over PER-UID buffers.
 *
 * per-UID rings are owned by the session daemon, so they SURVIVE the traced
 * process -- which per-process rings do not: a process that vanishes before the
 * snapshot is issued takes its trace with it.  Isolation instead comes from
 * giving each process its OWN session (`lttng track -u --pid`), so peers cannot
 * evict each other's window and a process-local freeze corresponds exactly to
 * one session's buffers.
 *
 * The session name arrives in FT_TRACE_SESSION from the launcher that created
 * and PID-tracked it; "ftrd" keeps the single-process case working unchanged.
 */
static inline
const char *ft_trace_session(void)
{
	static const char *name;

	if (caa_unlikely(!name)) {
		const char *e = getenv("FT_TRACE_SESSION");

		name = (e && *e) ? e : "ftrd";
	}
	return name;
}

/*
 * Freeze, then stop and dump THIS process's session.  Order is load-bearing:
 * the caller must already have emitted its violation event, because the freeze
 * silences every emitter in this process.
 */
static inline
void ft_trace_capture(void)
{
	char cmd[256];

	FT_TRACE_FREEZE();
	snprintf(cmd, sizeof(cmd), "lttng stop %s 1>&2", ft_trace_session());
	(void) system(cmd);
	snprintf(cmd, sizeof(cmd), "lttng snapshot record -s %s 1>&2",
		ft_trace_session());
	(void) system(cmd);
}

#define FT_TP(name, ...)						\
	do {								\
		if (caa_likely(!uatomic_load(&ft_trace_frozen,		\
				CMM_RELAXED)))				\
			lttng_ust_tracepoint(cds_ft, name, ##__VA_ARGS__); \
	} while (0)
/*
 * Emit a tracepoint with a key byte-sequence payload (LTTng's
 * sequence_hex field).  When tracing is disabled, the arguments are
 * discarded by the FT_TP no-op expansion, so no code is emitted.
 *
 * FT_TP_KEY_RESOLVED: caller has already resolved keylen to a real
 *   byte count (e.g. from ft_key_len() or iter->key_len).
 *
 * FT_TP_KEY: user-facing sentinel values (CDS_FT_LEN_DEFAULT ==
 *   SIZE_MAX, etc.) must be resolved via ft_key_len(ft_, keylen)
 *   before the sequence field reads keylen bytes from keybuf, or
 *   the sequence reader would attempt to serialize SIZE_MAX bytes.
 *   On resolution failure (LEN_ERROR), the key sequence is empty.
 */
#define FT_TP_KEY_RESOLVED(name, ft_, keybuf, keylen)			\
	FT_TP(name, (const void *) (ft_), (keybuf), (keylen))
#define FT_TP_KEY(name, ft_, keybuf, keylen)				\
	do {								\
		size_t _tp_klen = ft_key_len((ft_), (keylen));		\
		FT_TP_KEY_RESOLVED(name, (ft_), (keybuf),		\
			_tp_klen == CDS_FT_LEN_ERROR ? 0 : _tp_klen);	\
	} while (0)
/*
 * FT_TP_ITER_KEY: emit an iter-keyed key event.  Uses the resolved
 * iter->key_len; ft is carried alongside iter for snapshot safety
 * (iter_create may have already scrolled out of a flight recorder).
 */
#define FT_TP_ITER_KEY(name, iter)					\
	FT_TP(name, (const void *) (iter)->ft, (const void *) (iter),	\
		iter_key(iter), (iter)->key_len)
/*
 * enum ft_tp_node_kind (node-kind identifiers) lives in
 * fractal-trie-internal.h so the C side and the LTTng enum in
 * src/cds_ft_tp.h reference a single definition.  ft_tp_node_kind()
 * in fractal-trie.c maps a tagged cds_ft_inode_flag pointer to one of
 * those values.
 */
#else
#define FT_TP(name, ...)			do {} while (0)
#define FT_TRACE_FREEZE()			do {} while (0)
#define ft_trace_capture()			do {} while (0)
#define FT_TP_KEY_RESOLVED(name, ft_, keybuf, keylen)	do {} while (0)
#define FT_TP_KEY(name, ft_, keybuf, keylen)	do {} while (0)
#define FT_TP_ITER_KEY(name, iter)		do {} while (0)
#endif

#endif /* _URCU_FT_TRACE_H */
