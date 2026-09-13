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

/*
 * THE PER-OP RETRY CAP: one counter, every mutation entry, one loud failure.
 *
 * ☠ WHY A COUNTER IS A CORRECTNESS INSTRUMENT AND NOT A PERFORMANCE ONE.
 * -EAGAIN in this tree carries a PROMISE: "retrying can help, because a PEER is
 * responsible for this refusal".  A retry count that runs away therefore does
 * not mean the machine is busy -- it means the op is refusing ITSELF, on a
 * condition no re-descent can change.  Single-threaded that reading is certain;
 * with peers it is a hypothesis the event's @nr_writers lets the reader judge.
 *
 * The rekey writer already had a private version of this (FT_DEBUG_REKEY_RETRY_
 * CAP), and it found three self-refusals by hand.  This is that detector made
 * GENERIC -- insert, remove, replace and rekey share it -- and made TRACEABLE:
 * instead of only printing, it fires an LTTng violation event, records the
 * flight-recorder snapshot, and aborts, so the window that led to the spin is
 * on disk instead of being overwritten by the spin itself.
 *
 * ☞ IT IS PER OP, NOT PER THREAD.  Reset at every entry: a thread that does a
 * million ordinary mutations must not accumulate its way into a false positive,
 * and the question is always "did THIS call converge".
 *
 * The cap is deliberately far above real contention -- a saturated multi-writer
 * arm re-attempts in single digits, so five figures cannot be a peer.
 */
#ifdef FT_DEBUG_OP_RETRY_CAP
# include <stdio.h>
# include <stdlib.h>
# ifndef FT_OP_RETRY_CAP
#  define FT_OP_RETRY_CAP	50000
# endif

enum ft_op_kind {
	FT_OP_INSERT = 1,
	FT_OP_REMOVE,
	FT_OP_REPLACE,
	FT_OP_REKEY,
};

/*
 * ★ WHICH RETRY SITE.  The cap says an op spun; it does not say WHERE, and an
 * op with four distinct "publishes nothing, retry" exits has four different
 * diagnoses.  Mirrors ft_dbg_rm_site (ft-helpers.h): each site stamps its own
 * __LINE__, and the livelock report names the line that spun, how many times
 * consecutively, and the last OTHER line seen -- so a single spinning exit is
 * distinguishable from a cycle between two.
 */
static __thread unsigned int ft_dbg_retry_line;
static __thread unsigned long ft_dbg_retry_line_nr;
static __thread unsigned int ft_dbg_retry_line_other;

static inline
void ft_dbg_retry_stamp(unsigned int line)
{
	if (line == ft_dbg_retry_line) {
		ft_dbg_retry_line_nr++;
		return;
	}
	if (ft_dbg_retry_line)
		ft_dbg_retry_line_other = ft_dbg_retry_line;
	ft_dbg_retry_line = line;
	ft_dbg_retry_line_nr = 1;
}
# define FT_DBG_RETRY_SITE()	ft_dbg_retry_stamp(__LINE__)

/*
 * ★ WHICH -EAGAIN.  The retry-exit stamp above names the EXIT; on an op whose
 * body has forty-odd -EAGAIN producers that is still a whole file.  This stamps
 * the PRODUCER, as an expression so it is safe in a braceless `if` and in both
 * `return -EAGAIN;` and `ret = -EAGAIN;` forms.
 *
 * NOT applied at rest: wrapping fifty-odd returns costs more readability in a
 * shipping header than it is worth once the ACQUIRE stamp below exists, and the
 * two ends of the path usually pin the middle.  Re-apply mechanically when they
 * do not -- sed 's/return -EAGAIN;/return FT_DBG_EAGAIN(-EAGAIN);/' over the op's
 * file, plus the same for `<var> = -EAGAIN;` -- which is how ft-remove.h's
 * line 5622 (the holder acquire) was named.
 */
static __thread unsigned int ft_dbg_eagain_line;
static __thread unsigned long ft_dbg_eagain_line_nr;
static __thread unsigned int ft_dbg_eagain_line_other;

static inline
int ft_dbg_eagain_stamp(int v, unsigned int line)
{
	if (line == ft_dbg_eagain_line) {
		ft_dbg_eagain_line_nr++;
		return v;
	}
	if (ft_dbg_eagain_line)
		ft_dbg_eagain_line_other = ft_dbg_eagain_line;
	ft_dbg_eagain_line = line;
	ft_dbg_eagain_line_nr = 1;
	return v;
}
# define FT_DBG_EAGAIN(v)	ft_dbg_eagain_stamp((v), __LINE__)

/*
 * ★ AND WHICH REFUSAL INSIDE THE ACQUIRE.  One -EAGAIN producer in the caller
 * can be four in ft_dlm_acquire_set_at, which funnels every refusal through one
 * `eagain:` label -- so the label erases the reason.  Stamp at the GOTO, in a
 * do/while so it stays one statement inside a braceless `if`.
 */
static __thread unsigned int ft_dbg_acq_line;
static __thread unsigned long ft_dbg_acq_line_nr;
static __thread unsigned int ft_dbg_acq_line_other;

static inline
void ft_dbg_acq_stamp(unsigned int line)
{
	if (line == ft_dbg_acq_line) {
		ft_dbg_acq_line_nr++;
		return;
	}
	if (ft_dbg_acq_line)
		ft_dbg_acq_line_other = ft_dbg_acq_line;
	ft_dbg_acq_line = line;
	ft_dbg_acq_line_nr = 1;
}
# define FT_DBG_ACQ_SITE()	ft_dbg_acq_stamp(__LINE__)

/*
 * ★ AND WHICH BIT.  ft_dlm_lock refuses on PROXY | TOMBSTONE | LOCK as one
 * test, and the three have different cures: LOCK is contention (it clears when
 * the holder commits), PROXY is a parked flip that settles, but TOMBSTONE is
 * PERMANENT -- a retired word whose expected-old can never recur, which is the
 * immortal-old livelock, not contention.  Counted apart, with the last refused
 * word kept so the report can name the state.
 */
static __thread unsigned long ft_dbg_lock_refuse_lock;
static __thread unsigned long ft_dbg_lock_refuse_proxy;
static __thread unsigned long ft_dbg_lock_refuse_tomb;
static __thread unsigned long ft_dbg_lock_refuse_state;
/*
 * ★ LEAKED OR CHURNING.  A LOCK refusal that is always the SAME word is a lock
 * nobody will release (a peer's bail path lost it); one that moves is ordinary
 * contention the op is losing.  Same cure-splitting question as the bit above.
 */
static __thread const void *ft_dbg_lock_refuse_meta;
static __thread unsigned long ft_dbg_lock_refuse_streak;
static __thread unsigned long ft_dbg_lock_refuse_switches;

struct ft_op_retry {
	unsigned int attempts;
	unsigned int op;
	const uint8_t *key;
	size_t key_len;
};

/*
 * @nr_writers is what separates "certain" from "hypothesis" at the event, and
 * it is sampled rather than tracked: the trie knows how many writer scopes are
 * open.  Zero when the build cannot answer, which reads as "unknown", never as
 * "single".
 */
static inline
unsigned int ft_op_retry_nr_writers(const struct cds_ft *ft)
{
	(void) ft;
	return 0;
}

static inline
void ft_op_retry_init(struct ft_op_retry *r, unsigned int op,
		const uint8_t *key, size_t key_len)
{
	r->attempts = 0;
	r->op = op;
	r->key = key;
	r->key_len = key_len;
}

/*
 * Called once per attempt.  Emits the per-retry step event (low value until the
 * violation has named a site, high rate -- enable by name), and on the cap
 * fires the violation, records the snapshot and aborts.
 *
 * ☠ THE SNAPSHOT MUST BE TAKEN BEFORE THE ABORT AND AFTER THE VIOLATION: the
 * ring is overwritten by the spin itself, so the event that explains the window
 * has to be IN the window it explains.
 */
static inline
void ft_op_retry_tick(const struct cds_ft *ft, struct ft_op_retry *r, int last_ret)
{
	FT_TP(op_retry_step, (const void *) ft, r->op, r->attempts, last_ret);
	if (caa_likely(++r->attempts <= FT_OP_RETRY_CAP))
		return;
	FT_TP(op_retry_violation, (const void *) ft, r->op, r->attempts,
		last_ret, ft_op_retry_nr_writers(ft),
		r->key, r->key ? r->key_len : 0);
	fprintf(stderr,
		"FT OP RETRY LIVELOCK: op=%u attempts=%u last_ret=%d -- an "
		"-EAGAIN no re-descent can clear.  Single-threaded this is "
		"certain; under peers it is the leading hypothesis.\n",
		r->op, r->attempts, last_ret);
	fprintf(stderr,
		"FT OP RETRY SITE: line=%u consecutive=%lu other_line=%u\n",
		ft_dbg_retry_line, ft_dbg_retry_line_nr,
		ft_dbg_retry_line_other);
	fprintf(stderr,
		"FT OP EAGAIN SITE: line=%u consecutive=%lu other_line=%u\n",
		ft_dbg_eagain_line, ft_dbg_eagain_line_nr,
		ft_dbg_eagain_line_other);
	fprintf(stderr,
		"FT OP ACQUIRE SITE: line=%u consecutive=%lu other_line=%u\n",
		ft_dbg_acq_line, ft_dbg_acq_line_nr, ft_dbg_acq_line_other);
	fprintf(stderr,
		"FT OP LOCK REFUSE: lock=%lu proxy=%lu tombstone=%lu "
		"last_state=0x%lx\n",
		ft_dbg_lock_refuse_lock, ft_dbg_lock_refuse_proxy,
		ft_dbg_lock_refuse_tomb, ft_dbg_lock_refuse_state);
	fprintf(stderr,
		"FT OP LOCK REFUSE WORD: meta=%p streak=%lu switches=%lu\n",
		ft_dbg_lock_refuse_meta, ft_dbg_lock_refuse_streak,
		ft_dbg_lock_refuse_switches);
	if (system("lttng snapshot record 1>&2") == -1)
		fprintf(stderr, "FT OP RETRY: snapshot record failed\n");
	abort();
}
#else
struct ft_op_retry { int unused; };
# define ft_op_retry_init(r, op, key, key_len)	do { (void) (r); } while (0)
# define ft_op_retry_tick(ft, r, last_ret)	do { (void) (r); } while (0)
# define FT_DBG_RETRY_SITE()			do { } while (0)
# define FT_DBG_EAGAIN(v)			(v)
# define FT_DBG_ACQ_SITE()			do { } while (0)
#endif	/* FT_DEBUG_OP_RETRY_CAP */

#endif /* _URCU_FT_TRACE_H */
