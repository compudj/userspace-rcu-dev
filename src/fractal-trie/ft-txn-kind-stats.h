// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_KIND_STATS_H
#define _FT_TXN_KIND_STATS_H

/*
 * ft-txn-kind-stats: WHICH RECORD KIND does each flip-txn plant, and what does
 * its commit return -- counted per TXN CREATION SITE.  Built only under
 * -DFT_DEBUG_TXN_KIND; byte-neutral otherwise (the struct fields, the
 * increments and this whole unit compile out).
 *
 * WHAT IT IS FOR.  The engine takes SW-kind and MW-kind records in one commit
 * (doc/design/mw-writer-lock-escalation-model.md).  An MW record is a validated
 * CAS: it arbitrates, and it can ABORT the whole commit.  An SW record is a
 * plain locked park: it cannot fail, and it is legal exactly where the op
 * EXCLUDES every peer writer of that slot.  Most FT ops record all-MW today not
 * because they must, but because they never opted in -- all-MW is stricter and
 * always sound.  Converting those CONSERVATIVE MW records to SW buys FAIRNESS
 * and LATENCY, never correctness, so the only honest acceptance criterion is a
 * measured ABORT / RETRY number.  This is that instrument, and it is meant to
 * be read BEFORE and AFTER a conversion:
 *
 *   BEFORE  MW_STRUCT is the conversion SURFACE: structural edges that took the
 *           MW branch only because their txn never armed @structural_sw.
 *   AFTER   MW_STRUCT must have MOVED to SW at the converted sites, and the
 *           site's ABORT count must have dropped.  A conversion that leaves
 *           MW_STRUCT where it was did not run
 *           ([[feedback_verify_the_mechanism_ran_before_believing_a_zero]]).
 *
 * IT ALSO CLASSIFIES THE 70 CREATION SITES EMPIRICALLY.  The acquire lane must
 * stay all-MW forever -- the lock take {clean -> LOCK|s} IS the arbitration
 * point, and an SW take hands BOTH racing ops the node -- so an acquire txn is
 * one whose site records MW_LOCK and VALIDATE and nothing else.  A content txn
 * records structural edges.  The table says which is which by measurement,
 * against which a source-level split can check itself.
 *
 * THE CLASSES, and what each one is allowed to become:
 *
 *   SW          a structural edge parked SW (the txn armed @structural_sw and
 *               the slot is not the &ft->root exemption).  Already converted.
 *   MW_STRUCT   a structural edge recorded MW because the txn is not armed.
 *               THE CONVERSION SURFACE.
 *   MW_ALWAYS   ft_flip_txn_record_tag_mw: the genuinely-unlocked slots --
 *               ordered-cell interleave, duplicate-chain splices, rank-count
 *               propagation.  Stays MW by design.
 *   MW_LOCK     the DLM lock take.  MUST stay MW; ft_dlm_lock asserts it.
 *   VALIDATE    a read-set guard (back-edge / anchor state).  Not a store; it
 *               can abort a commit, which is why it is worth seeing beside the
 *               abort column.
 *
 * The cell / hlist stores that bypass these helpers and record straight onto
 * the engine handle (ft-txn-hlist.h, ft_txn_list_insert_between_prepare) hold
 * no back-pointer to their flip-txn, so they are counted GLOBALLY rather than
 * per site.  They are MW on purpose and out of the conversion's scope; the
 * global count exists so that "MW records still standing after the conversion"
 * has no unexplained remainder.
 *
 * COUNTERS ARE PER THREAD, summed at the dump.  A shared counter on the hot
 * path would be a contended cacheline in the middle of the very fairness this
 * measures.  Each thread's block is registered on a global list and is never
 * freed, so a thread that exits before the dump still reports.
 */

#ifndef FRACTAL_TRIE_IMPL
#error "ft-txn-kind-stats.h is an implementation unit; #include it from fractal-trie.c only"
#endif

#ifdef FT_DEBUG_TXN_KIND

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

enum ft_tk_rec_class {
	FT_TK_SW = 0,
	FT_TK_MW_STRUCT,
	FT_TK_MW_ALWAYS,
	FT_TK_MW_LOCK,
	FT_TK_VALIDATE,
	/*
	 * THE PHASE B READINESS PAIR, and it is NOT a kind: they SPLIT
	 * MW_STRUCT -- the conversion surface -- by whether the op's lock
	 * registry HOLDS the word's owner (ft_flip_txn_owns).  OWN_HELD +
	 * OWN_MISS == MW_STRUCT is an invariant of the table, and a useful
	 * self-check on it.
	 *
	 * WHY A COUNTER AND NOT ONLY THE ASSERT.  The assert
	 * (FT_OWNER_ASSERT_OWNED) is armed only for a txn that armed PER-OP,
	 * so it has no coverage until the first Phase B site arms -- and an
	 * assert with no coverage is not coverage.  These two run the same
	 * predicate on every SW-capable record in EVERY mode, so a FINE
	 * unarmed run says, per site, whether the arm would be legal BEFORE
	 * the arm is written:
	 *
	 *   OWN_MISS == 0   the site's structural records are all owner-held;
	 *                   the per-op arm is the only thing missing.
	 *   OWN_MISS > 0    the site names a word its lock-set does not own.
	 *                   That is the exclusion gap to close, and its size
	 *                   is the number here -- not an argument to be had.
	 *
	 * ☠ A MISS IS NOT AUTOMATICALLY A DEFECT TODAY.  Unarmed, the record
	 * is MW and MW is always sound; and a site may hold the word through
	 * a frame the txn registry cannot see (ft_flip_txn_owns says why that
	 * is still worth reporting).  It is a Phase B PRECONDITION, read per
	 * site, never a bug count.
	 */
	FT_TK_OWN_HELD,
	/*
	 * The registry does NOT name the word's owner but this THREAD holds it
	 * (ft_hold_trace_holds).  A REGISTRY gap, not an exclusion gap: the op
	 * really does exclude every peer from that word, it just filed the hold
	 * where a record helper cannot see it.  Separating the two is what makes
	 * OWN_MISS readable as a Phase B obligation instead of a mix of the two.
	 *
	 * ☠ ZERO WITHOUT -DFEATURE_FT_HOLD_TRACE, where the query is a constant
	 * false and every such record falls into OWN_MISS -- exactly the older
	 * two-way split.  A zero column in that build is NOT evidence.
	 */
	FT_TK_OWN_LEDGER,
	FT_TK_OWN_MISS,
	FT_TK_REC_NR,
};

/*
 * MW_ALWAYS IS NOT ONE POPULATION, and the G4 decision reads it as if it were.
 * The column mixes slots that can NEVER convert (a trie root has no node to
 * lock) with slots that are unheld only because THIS op's lock-set does not
 * reach them (a SKIP_X dual in a grandparent) -- and af22756b's per-edge
 * @owner_held moved the second family INTO this column from MW_STRUCT, which is
 * why the post-Phase-B re-measure shows MW_ALWAYS nearly doubling while the
 * conversion surface falls 65%.  Part of that rise is BOOKKEEPING, and no
 * argument settles which part: this splits it.
 *
 * ONE CLASS PER RECORD BRANCH, declared by the branch itself.  There is no
 * default: the class is a required argument of ft_flip_txn_record_tag_mw in the
 * instrumented build, so a new always-MW branch cannot be added without saying
 * which population it joins (the same compiler-enforced discipline @owner_held
 * uses).  The classes are:
 *
 *   ROOT         &ft->root.  A root lives in no node, so there is no lock word
 *                to make a park legal -- ft_flip_txn_record_root.  NEVER
 *                converts, whatever G4 decides.
 *   HEAD_BACK    an external head's back channel (cell->parent / en->prev).
 *                Neither an external node nor its cell carries a state word,
 *                so this is that predicate's permanent false arm.  NEVER
 *                converts (kind settled at f79438e7).
 *   DUAL         a STRUCTURAL trie edge whose owner the op does not hold: the
 *                SKIP_X dual landing in a grandparent the op never acquired.
 *                THE RECLASSIFIED POPULATION -- it was MW_STRUCT (or, before
 *                af22756b, an unsound SW park) and it is convertible in
 *                principle, by WIDENING THE LOCK-SET, not by arming.
 *   CELL         an ordered-cell / duplicate-chain edge (a non-structural tag).
 *                THE G4 LANE.  Convertible only if cells grow a state word and
 *                join lock-sets -- the separately-planned workstream.
 *   RANK         nr_keys propagated up UNLOCKED ancestors.  Convertible only
 *                where the whole path is covered (root-only spacing), which is
 *                Phase E's fold, not G4's.
 *   PARENT_WORD  a child's parent_word written by the recompaction re-parent
 *                sweep, whose acquire takes {C,P,(GP)} and never C's children.
 *                Same shape as DUAL: a lock-set reach question.
 *   PSO          the same child's parent_slot_offset, third word of the same
 *                node.  §8.3's layout split is what retires it.
 *   STATE        the recompaction re-parent sweep's own child state word,
 *                recorded {live -> live}.  A VALIDATE in everything but the
 *                engine's bookkeeping, and it must stay MW: a park validates
 *                nothing.  MARKING those children instead was implemented in
 *                full and does not live (a contended child fails the acquire,
 *                and escalation cannot rescue it).
 *   GUARD        the §4.B guard on a node a forward edge INSTALLS as a VALUE,
 *                which no lock can ever cover (the acquire takes {C,P,(GP)},
 *                never C's children).  Same shape as STATE, different source,
 *                and separated here because one number for both is what this
 *                decomposition exists to stop.
 *
 * ☞ CELL here counts only the cell edges that ride a flip-txn's edge replay.
 * The ones recorded straight on the engine handle are the separate cell/hlist
 * line -- see the header comment.  Both are the same lane for G4.
 */
enum ft_tk_mwa_class {
	FT_TK_MWA_ROOT = 0,
	FT_TK_MWA_HEAD_BACK,
	FT_TK_MWA_DUAL,
	FT_TK_MWA_CELL,
	FT_TK_MWA_RANK,
	FT_TK_MWA_PARENT_WORD,
	FT_TK_MWA_PSO,
	FT_TK_MWA_STATE,
	FT_TK_MWA_GUARD,
	FT_TK_MWA_NR,
};

#ifdef FT_ABORT_ATTRIB
/*
 * ft-txn-rec-dbg.h reserves its tail for these nine and is parsed BEFORE this
 * header (it defines hooks the engine's inlines need), so it cannot use the
 * enum itself.  Fail the BUILD if the two ever disagree rather than silently
 * folding a tenth population into the last row.
 */
urcu_static_assert(FT_AB_CLS_NR - FT_AB_MWA_BASE == FT_TK_MWA_NR,
	"ft-txn-rec-dbg.h reserves a different number of always-MW classes",
	ft_ab_mwa_class_count);
#endif

enum ft_tk_end_class {
	FT_TK_OK = 0,		/* urcu_txn_commit_flavor -> OK */
	FT_TK_ABORT,		/* a peer won an MW record or a guard */
	FT_TK_MEMERR,		/* sticky ENOMEM: nothing was parked */
	FT_TK_MISS,		/* acquire_miss: discarded before the engine */
	FT_TK_BAILED,		/* ft_flip_txn_destroy: no commit attempted */
	FT_TK_END_NR,
};

/*
 * The last id is the OVERFLOW bucket, so a site that finds the table full
 * still counts (into a row named as such) instead of counting nowhere.
 */
#define FT_TK_MAX_SITES		256
#define FT_TK_OVERFLOW_ID	(FT_TK_MAX_SITES - 1)

/*
 * One txn creation site.  Lives as a function-static at the site itself
 * (FT_TK_SITE_HERE), so the identity is the source line rather than anything
 * the site has to be told.
 */
struct ft_tk_site {
	const char *file;
	int line;
	const char *what;	/* which constructor */
	int id;			/* -1 until registered */
};

#ifdef FT_WINNER_DBG
/* Raw-note sites (FT_WIN_NOTE_RAW): the non-engine live-slot writers. */
#define FT_WIN_RAW_NR	2
#endif

struct ft_tk_tls {
	struct ft_tk_tls *next;
	unsigned long rec[FT_TK_MAX_SITES][FT_TK_REC_NR];
	unsigned long end[FT_TK_MAX_SITES][FT_TK_END_NR];
	unsigned long created[FT_TK_MAX_SITES];
	unsigned long armed[FT_TK_MAX_SITES];
	unsigned long cell_mw;	/* not site-attributed, see the header comment */
	unsigned long mwa[FT_TK_MWA_NR];
#ifdef FT_ABORT_ATTRIB
	/*
	 * WHICH CLASS OF RECORD LOST, per txn creation site.  Per site because a
	 * class that only ever loses at one site is a different problem from one
	 * that loses everywhere, and the site is the unit every other row here is
	 * keyed by.  The OWNERSHIP witness is kept GLOBALLY beside it: it splits
	 * one class (the structural surface), and a full site x class x witness
	 * cube would be 4x this for a column that is zero almost everywhere.
	 */
	unsigned long ab[FT_TK_MAX_SITES][FT_AB_CLS_NR];
	unsigned long ab_own[FT_AB_CLS_NR][FT_AB_OWN_NR];
	unsigned long ab_mixed;		/* two classes chained onto the losing word */
	unsigned long ab_unattrib;	/* an abort no hook could name (poisoned) */
	unsigned long ab_alarm[FT_TK_MAX_SITES][2];	/* [0] state word, [1] pointer slot */
	/*
	 * ft_slot_in_node(g->publish_parent, g->publish_slot) at the glue publish:
	 * is the slot this commit writes actually one of the named parent's own
	 * child slots?  The record names that parent as the slot's OWNER, and the
	 * record-time owner check asks only whether the op HOLDS it -- never
	 * whether it OWNS the word.  [0] no, [1] yes.
	 */
	unsigned long pub_pair[2];
#ifdef FT_WINNER_DBG
	/*
	 * WHO WON the word an alarmed record lost.  The PRIMARY witness is the
	 * value the losing CAS OBSERVED (URCU_TXN_REC_LOST) -- race-free.  A
	 * proxy there decodes to the winning record exactly (win_beater_proxy);
	 * a plain value is matched against the winner ledger for CORROBORATION
	 * only (win_beater_ledger) -- under ABA a value-match can name an older
	 * write of the same value, so it is a population claim, never proof of
	 * the one write.  win_raw names the non-engine release-store lane by
	 * its note site.  win_owner compares the owner the (ledger-corroborated)
	 * winner named against the loser's -- [0] same, [1] ☠ DIFFERENT (two
	 * metadata each believed to own the word), [2] unknown.  win_owner_tomb
	 * and win_owner_unparented are LOSER-side probes of candidate 2: the
	 * registered owner found TOMBSTONED, or mid re-home (parent_word NULL),
	 * at the alarm itself.
	 */
	unsigned long win_beater[FT_AB_CLS_NR][FT_AB_OWN_NR];
	unsigned long win_beater_proxy;
	unsigned long win_beater_ledger;
	unsigned long win_raw[FT_WIN_RAW_NR];
	unsigned long win_ledger_mismatch;	/* plain beater; ledger has the slot, another value */
	unsigned long win_ledger_absent;	/* plain beater; no ledger entry (eviction, or an unhooked writer) */
	unsigned long win_seen_none;	/* the loss exit had no observed value in hand */
	unsigned long win_drift;	/* the slot changed again between the loss and this lookup */
	unsigned long win_owner[3];
	unsigned long win_tid_same;	/* the corroborated winner was THIS thread's own write */
	unsigned long win_owner_tomb;
	unsigned long win_owner_unparented;
	unsigned long win_torn;		/* the ledger entry would not settle under 4 seq retries */
	unsigned long win_note;		/* reach: ledger writes by this thread */
	unsigned long win_note_skip;	/* ledger writes skipped on entry contention */
#endif
#endif
};

static pthread_mutex_t ft_tk_lock = PTHREAD_MUTEX_INITIALIZER;
static struct ft_tk_site *ft_tk_sites[FT_TK_MAX_SITES];
static int ft_tk_nr_sites;
static struct ft_tk_tls *ft_tk_threads;
static __thread struct ft_tk_tls *ft_tk_self;

static
struct ft_tk_site ft_tk_overflow_site = { "<sites overflowed>", 0, "-", FT_TK_OVERFLOW_ID };

/*
 * Assign @s a stable id.  Two threads reaching a cold site both take the lock;
 * the second finds the id already published and uses it.
 */
static
int ft_tk_site_register(struct ft_tk_site *s)
{
	int id;

	pthread_mutex_lock(&ft_tk_lock);
	if (s->id >= 0) {
		id = s->id;
		goto unlock;
	}
	if (ft_tk_nr_sites >= FT_TK_OVERFLOW_ID) {
		ft_tk_sites[FT_TK_OVERFLOW_ID] = &ft_tk_overflow_site;
		id = FT_TK_OVERFLOW_ID;
	} else {
		id = ft_tk_nr_sites++;
		ft_tk_sites[id] = s;
	}
	CMM_STORE_SHARED(s->id, id);
unlock:
	pthread_mutex_unlock(&ft_tk_lock);
	return id;
}

static inline
int ft_tk_site_id(struct ft_tk_site *s)
{
	int id = CMM_LOAD_SHARED(s->id);

	if (caa_likely(id >= 0))
		return id;
	return ft_tk_site_register(s);
}

static
struct ft_tk_tls *ft_tk_tls_create(void)
{
	struct ft_tk_tls *tls = (struct ft_tk_tls *) calloc(1, sizeof(*tls));

	if (!tls)
		abort();	/* a debug build that cannot count is a lie */
	pthread_mutex_lock(&ft_tk_lock);
	tls->next = ft_tk_threads;
	ft_tk_threads = tls;
	pthread_mutex_unlock(&ft_tk_lock);
	ft_tk_self = tls;
	return tls;
}

static inline
struct ft_tk_tls *ft_tk_tls_get(void)
{
	struct ft_tk_tls *tls = ft_tk_self;

	if (caa_likely(tls != NULL))
		return tls;
	return ft_tk_tls_create();
}

/*
 * @site NULL means the txn was built by a path that predates this instrument
 * (there is none today) -- count it in the overflow row rather than nowhere.
 */
static inline
void ft_tk_count_rec(struct ft_tk_site *site, enum ft_tk_rec_class c)
{
	ft_tk_tls_get()->rec[site ? ft_tk_site_id(site) : FT_TK_OVERFLOW_ID][c]++;
}

static inline
void ft_tk_count_end(struct ft_tk_site *site, enum ft_tk_end_class c)
{
	ft_tk_tls_get()->end[site ? ft_tk_site_id(site) : FT_TK_OVERFLOW_ID][c]++;
}

static inline
void ft_tk_count_created(struct ft_tk_site *site)
{
	ft_tk_tls_get()->created[site ? ft_tk_site_id(site) : FT_TK_OVERFLOW_ID]++;
}

static inline
void ft_tk_count_armed(struct ft_tk_site *site)
{
	ft_tk_tls_get()->armed[site ? ft_tk_site_id(site) : FT_TK_OVERFLOW_ID]++;
}

static inline
void ft_tk_count_cell_mw(void)
{
	ft_tk_tls_get()->cell_mw++;
}

/*
 * Which always-MW POPULATION this record joins.  Global rather than per site:
 * the class is a property of the record BRANCH (its producer), and every branch
 * is reached from many txn sites.
 */
static inline
void ft_tk_count_mwa(enum ft_tk_mwa_class c)
{
	ft_tk_tls_get()->mwa[c]++;
}

#ifdef FT_ABORT_ATTRIB
static const char * const ft_ab_names[FT_AB_CLS_NR] = {
	"UNATTRIB", "SW", "MW_STRUCT", "MW_LOCK", "VALIDATE", "CELL_HANDLE",
	"mwa:ROOT", "mwa:HEAD_BACK", "mwa:DUAL", "mwa:CELL",
	"mwa:RANK", "mwa:PARENT_WORD", "mwa:PSO", "mwa:STATE", "mwa:GUARD",
};

static
const char *ft_ab_cls_name(unsigned int cls)
{
	return cls < FT_AB_CLS_NR ? ft_ab_names[cls] : "?";
}

/*
 * The engine's abort hook, declared in ft-txn-rec-dbg.h and defined here where
 * struct urcu_txn_record is complete.  It only STASHES: the class is counted in
 * ft_flip_txn_commit, which still has the txn's creation site in hand.
 */
static const char *ft_ab_cls_name(unsigned int cls);
static int ft_ab_is_alarm(unsigned int cls, unsigned int own);

#ifdef FT_WINNER_DBG
/*
 * THE WINNER LEDGER.  A global slot-keyed table of the LAST ENGINE WRITE to
 * each slot -- who (label, named owner, record call site, tid) and what value.
 * Filled from URCU_TXN_REC_WROTE on the writer's own thread (which is what
 * lets it read the writer's ring for the owner it named); read by an aborting
 * peer whose alarmed record just lost that slot.
 *
 * ☠ THE LEDGER IS A CLAIM ABOUT A POPULATION, NOT ABOUT ONE WRITE.  The entry
 * the loser reads is only "the last stamped write" -- under ABA (the alarmed
 * slot's value is known to cycle among three pointers) a value-match may name
 * an OLDER write of the same value, and a racing stamp is skipped outright.
 * What makes it usable anyway: every stamped writer of the slot IS a writer of
 * that slot, so the HISTOGRAM over tens of thousands of alarms describes the
 * slot's writer population, which is the question ("who writes a child slot of
 * P without holding P").  The value-match (win_attr vs win_stale) and the
 * reach counters bound the noise instead of hiding it.
 *
 * Entries are seq-versioned (even = stable, odd = writer active).  A writer
 * TRY-claims with one CAS and SKIPS on contention -- the ledger must never
 * add a wait to the engine's install path -- and the skip is counted.
 */
#define FT_WIN_BITS	19
#define FT_WIN_SIZE	(1UL << FT_WIN_BITS)
#define FT_WIN_MASK	(FT_WIN_SIZE - 1)
#define FT_WIN_WAYS	4

struct ft_win_ent {
	unsigned long seq;
	void **slot;
	void *val;
	const void *owner;	/* the owner the WINNER named (its ring), or NULL */
	const void *ra;		/* the winner's record call site, or NULL */
	unsigned long tid;
	unsigned int code;	/* the winner's dbg_embedder label */
};

static struct ft_win_ent ft_win_tbl[FT_WIN_SIZE];

static inline
unsigned long ft_win_hash(void **slot)
{
	uintptr_t a = (uintptr_t) slot >> 3;

	a ^= a >> 33;
	a *= 0xff51afd7ed558ccdULL;
	a ^= a >> 29;
	return (unsigned long) a & FT_WIN_MASK;
}

static inline
unsigned long ft_win_tid(void)
{
	return (unsigned long) pthread_self();
}

/* Declared in ft-txn-rec-dbg.h; the engine's URCU_TXN_REC_WROTE lands here. */
static
void ft_win_note(void **slot, void *val, unsigned int code)
{
	struct ft_tk_tls *tls = ft_tk_tls_get();
	unsigned long h = ft_win_hash(slot);
	struct ft_win_ent *e = NULL;
	unsigned long s;
	unsigned int i, cls = ft_ab_code_cls(code);

	for (i = 0; i < FT_WIN_WAYS; i++) {
		struct ft_win_ent *c = &ft_win_tbl[(h + i) & FT_WIN_MASK];

		if (CMM_LOAD_SHARED(c->slot) == slot) {
			e = c;
			break;
		}
		if (!e && !CMM_LOAD_SHARED(c->slot))
			e = c;
	}
	if (!e)
		e = &ft_win_tbl[h];	/* all ways foreign: evict the primary */
	s = uatomic_load(&e->seq, CMM_RELAXED);
	if ((s & 1) || uatomic_cmpxchg(&e->seq, s, s + 1) != s) {
		tls->win_note_skip++;
		return;
	}
	CMM_STORE_SHARED(e->slot, slot);
	CMM_STORE_SHARED(e->val, val);
	e->code = code;
	/*
	 * The ring notes owners only where FT_AB_NOTE_OWNER runs -- the
	 * lock-coverable classes.  Asking it for any other class would return
	 * a STALE note from an earlier record on this thread that happened to
	 * share the slot.
	 */
	if (cls == FT_AB_MW_STRUCT || cls == FT_AB_MW_LOCK) {
		e->owner = ft_ab_owner_of(slot);
		e->ra = ft_ab_ra_of(slot);
	} else if (cls >= FT_AB_CLS_NR) {
		/* A raw note: the return address names the storing lane
		 * (addr2line -i walks the inline chain to the caller). */
		e->owner = NULL;
		e->ra = __builtin_return_address(0);
	} else {
		e->owner = NULL;
		e->ra = NULL;
	}
	e->tid = ft_win_tid();
	uatomic_store(&e->seq, s + 2, CMM_RELEASE);
	tls->win_note++;
}

enum ft_win_state {
	FT_WIN_NONE = 0,	/* no observed value at the loss exit */
	FT_WIN_ATTR,	/* plain beater, ledger-corroborated (population claim) */
	FT_WIN_PROXY,	/* ★ the observed value IS the winning record (exact) */
	FT_WIN_STALE,	/* ledger names the slot, another value: lag or an unhooked writer */
	FT_WIN_MISS,	/* no ledger entry: eviction, or an unhooked writer */
	FT_WIN_TORN,	/* the entry would not settle */
	FT_WIN_RAW,	/* ☠ the beater was a NON-ENGINE release store, by note site */
};

/* The observed value of the last lost CAS (URCU_TXN_REC_LOST). */
static __thread struct {
	const struct urcu_txn_record *rec;
	void *seen;
	int valid;
} ft_win_seen_tls;

static
void ft_win_lost(const struct urcu_txn_record *rec, void *seen)
{
	ft_win_seen_tls.rec = rec;
	ft_win_seen_tls.seen = seen;
	ft_win_seen_tls.valid = 1;
}

/* The last lookup's answer, for the -DFT_ABORT_CLAIM dump. */
static __thread struct {
	enum ft_win_state state;
	unsigned int code;
	const void *owner;
	const void *ra;
	unsigned long tid;
	void *now;
	void *seen;
	void *val;
	int have;
	int owner_tomb;
	int owner_unparented;
} ft_win_last;

static
void ft_win_lookup(const struct urcu_txn_record *r)
{
	struct ft_tk_tls *tls = ft_tk_tls_get();
	int have = ft_win_seen_tls.valid && ft_win_seen_tls.rec == r;
	void *seen = have ? ft_win_seen_tls.seen : NULL;
	void *now = CMM_LOAD_SHARED(*r->slot);
	unsigned long h;
	struct ft_win_ent snap;
	int found = 0, torn = 0;
	unsigned int i;

	ft_win_seen_tls.valid = 0;
	memset(&ft_win_last, 0, sizeof(ft_win_last));
	ft_win_last.now = now;
	ft_win_last.seen = seen;
	ft_win_last.have = have;
	/*
	 * LOSER-SIDE probes of the re-home candidate, no winner cooperation
	 * needed: is the owner this op registered (and still holds) already
	 * TOMBSTONED, or mid re-home (the parent_word NULL transient belongs
	 * to detach / graft_swap)?  Either found true at an alarm says the
	 * word left the registered owner's coverage while the lock was held.
	 */
	{
		const struct cds_ft_metadata *own =
			(const struct cds_ft_metadata *)
				ft_ab_owner_of(r->slot);

		if (own) {
			if (CMM_LOAD_SHARED(own->state) & FT_STATE_TOMBSTONE) {
				tls->win_owner_tomb++;
				ft_win_last.owner_tomb = 1;
			}
			if (!CMM_LOAD_SHARED(own->parent_word)) {
				tls->win_owner_unparented++;
				ft_win_last.owner_unparented = 1;
			}
		}
	}
	if (!have || !seen) {
		/* No observed value (or a genuine NULL beater): unattributable. */
		tls->win_seen_none++;
		ft_win_last.state = FT_WIN_NONE;
		return;
	}
	if (now != seen)
		tls->win_drift++;	/* the window a re-read design would misread */
	/*
	 * ★ THE EXACT ARM.  A proxy observed AT the losing CAS is the winning
	 * record itself -- RCU keeps its descriptor alive for this reader --
	 * and its label needs no ledger.  The winner's ring is another
	 * thread's, so its named owner stays unknown here.
	 */
	if (urcu_txn_is_proxy(seen, r->proxy_tag)) {
		const struct urcu_txn_record *wr =
			(const struct urcu_txn_record *)
				urcu_txn_untag(seen, r->proxy_tag);
		unsigned int wcode = CMM_LOAD_SHARED(wr->dbg_embedder);
		unsigned int wcls = ft_ab_code_cls(wcode);

		if (wcls >= FT_AB_CLS_NR)
			wcls = FT_AB_UNSET;
		tls->win_beater[wcls][ft_ab_code_own(wcode)]++;
		tls->win_beater_proxy++;
		ft_win_last.state = FT_WIN_PROXY;
		ft_win_last.code = wcode;
		return;
	}
	/* A plain beater: the ledger is corroboration, never proof. */
	h = ft_win_hash(r->slot);
	for (i = 0; i < FT_WIN_WAYS && !found && !torn; i++) {
		struct ft_win_ent *e = &ft_win_tbl[(h + i) & FT_WIN_MASK];
		unsigned int tries;

		if (CMM_LOAD_SHARED(e->slot) != r->slot)
			continue;
		torn = 1;
		for (tries = 0; tries < 4; tries++) {
			unsigned long s0 = uatomic_load(&e->seq, CMM_ACQUIRE);

			if (s0 & 1) {
				caa_cpu_relax();
				continue;
			}
			snap = *e;
			cmm_smp_rmb();
			if (uatomic_load(&e->seq, CMM_RELAXED) == s0) {
				found = snap.slot == r->slot;
				torn = 0;
				break;
			}
		}
	}
	if (torn) {
		tls->win_torn++;
		ft_win_last.state = FT_WIN_TORN;
		return;
	}
	if (found && snap.val == seen) {
		/*
		 * ☠ Compare the CLASS FIELD, not the code: the code carries the
		 * ownership witness at bit 8, so an engine MW_STRUCT/held code
		 * (0x102) is numerically past FT_AB_CLS_NR.  A full-code compare
		 * misfiled every such winner as raw -- caught by ONE claim
		 * sample printing the fields the counter had already binned.
		 */
		if (ft_ab_code_cls(snap.code) >= FT_AB_CLS_NR) {
			unsigned int k = ft_ab_code_cls(snap.code) - FT_AB_CLS_NR;

			tls->win_raw[k < FT_WIN_RAW_NR ? k : 0]++;
			ft_win_last.state = FT_WIN_RAW;
		} else {
			unsigned int wcls = ft_ab_code_cls(snap.code);
			const void *lown = ft_ab_owner_of(r->slot);

			if (wcls >= FT_AB_CLS_NR)
				wcls = FT_AB_UNSET;
			tls->win_beater[wcls][ft_ab_code_own(snap.code)]++;
			tls->win_beater_ledger++;
			if (!snap.owner || !lown)
				tls->win_owner[2]++;
			else if (snap.owner == lown)
				tls->win_owner[0]++;
			else
				tls->win_owner[1]++;
			if (snap.tid == ft_win_tid())
				tls->win_tid_same++;
			ft_win_last.state = FT_WIN_ATTR;
		}
		ft_win_last.code = snap.code;
		ft_win_last.owner = snap.owner;
		ft_win_last.ra = snap.ra;
		ft_win_last.tid = snap.tid;
		ft_win_last.val = snap.val;
		return;
	}
	if (found) {
		tls->win_ledger_mismatch++;
		ft_win_last.state = FT_WIN_STALE;
		ft_win_last.code = snap.code;
		ft_win_last.owner = snap.owner;
		ft_win_last.ra = snap.ra;
		ft_win_last.tid = snap.tid;
		ft_win_last.val = snap.val;
		return;
	}
	tls->win_ledger_absent++;
	ft_win_last.state = FT_WIN_MISS;
}
#endif	/* FT_WINNER_DBG */

static
void ft_ab_note_lost(const struct urcu_txn_desc *t,
		const struct urcu_txn_record *r)
{
	ft_ab_lost = r ? r->dbg_embedder :
		ft_ab_code(FT_AB_UNSET, FT_AB_OWN_NA);
	ft_ab_lost_tag = r ? r->proxy_tag : 0;
	ft_ab_lost_valid = r != NULL;
#ifdef FT_WINNER_DBG
	/*
	 * The winner lookup runs HERE, at the first instant after the lost
	 * CAS: every microsecond of delay lets more writes land on the slot
	 * and turns ATTRIBUTED into STALE.
	 */
	ft_win_last.state = FT_WIN_NONE;
	if (r && ft_ab_is_alarm(ft_ab_code_cls(ft_ab_lost),
			ft_ab_code_own(ft_ab_lost)))
		ft_win_lookup(r);
	/*
	 * Consume the stash on EVERY abort: records live in a reused slab, so
	 * a stale {rec, seen} pair could otherwise match a LATER alarm's
	 * record by address and attribute last week's beater to it.
	 */
	ft_win_seen_tls.valid = 0;
#endif
#ifdef FT_ABORT_CLAIM
	/*
	 * THE DESCRIPTOR IS STILL INTACT HERE and is freed on the way out, so
	 * this is the only place the losing record can be read against its
	 * SIBLINGS -- which is how it is read: "this CAS lost" is not a
	 * diagnosis, "these two records disagree about whether the op holds
	 * that word" is.  `now=` is what turns a mismatch into one: it says
	 * which of the two is lying.
	 */
	if (r && ft_ab_is_alarm(ft_ab_code_cls(ft_ab_lost),
			ft_ab_code_own(ft_ab_lost))) {
		unsigned int i;

		fprintf(stderr,
"\n[FT_ABORT_CLAIM] a record that CANNOT lose, lost.  nr=%u nr_mw=%u retry=%u\n"
"    ra base: cds_ft_create=%p  (file offset of ra = nm(cds_ft_create) + ra - this)\n",
			t->nr, t->nr_mw, t->retry, (void *) &cds_ft_create);
		for (i = 0; i < t->nr; i++) {
			const struct urcu_txn_record *q = &t->recs[i];

			fprintf(stderr,
"    rec[%2u] %-14s %s%s slot=%p old=%p new=%p now=%p tag=0x%lx owner=%p ra=%p%s\n",
				i, ft_ab_cls_name(ft_ab_code_cls(q->dbg_embedder)),
				q->kind == URCU_TXN_KIND_SW ? "SW" : "MW",
				ft_ab_code_own(q->dbg_embedder) == FT_AB_OWN_HELD ?
					"/held" : (ft_ab_code_own(q->dbg_embedder) ==
						FT_AB_OWN_MISS ? "/MISS " : "     "),
				(void *) q->slot, q->old_ptr, q->new_ptr,
				(void *) CMM_LOAD_SHARED(*q->slot),
				(unsigned long) q->proxy_tag,
				ft_ab_owner_of(q->slot), ft_ab_ra_of(q->slot),
				q == r ? "   <== LOST" : "");
		}
#ifdef FT_WINNER_DBG
		{
			static const char * const wst[] = {
				"NONE (no observed value)",
				"ledger-corroborated",
				"★ PROXY-DECODED (exact)",
				"ledger STALE",
				"no ledger entry",
				"ledger TORN",
				"☠ RAW-LANE store",
			};

			fprintf(stderr,
"    beater: %s  seen=%p now=%p  cls=%s own=%u owner=%p ra=%p tid=0x%lx%s%s\n",
				wst[ft_win_last.state],
				ft_win_last.seen, ft_win_last.now,
				ft_ab_cls_name(ft_ab_code_cls(ft_win_last.code)),
				ft_ab_code_own(ft_win_last.code),
				ft_win_last.owner, ft_win_last.ra,
				ft_win_last.tid,
				ft_win_last.owner_tomb ?
					"  ☠ REGISTERED OWNER TOMBSTONED" : "",
				ft_win_last.owner_unparented ?
					"  ☠ REGISTERED OWNER MID RE-HOME (parent_word NULL)" : "");
		}
#endif
	}
#endif
}

/*
 * ★ THE ALARM.  An SW park cannot lose a CAS -- it does not do one -- and a
 * STRUCTURAL MW record whose owner the op HOLDS should not either, because the
 * DLM lock over the slot's owner is precisely what excludes every peer writer
 * of that word.  Either sighting says the exclusion is not what it claims, and
 * that is the opposite of a conversion opportunity: arming such a site would
 * turn a contested word into an unarbitrated plain store.
 *
 * ☠ NOT AN ASSERT, YET.  It is counted first, because a detector that has never
 * been shown to reach its subject is not coverage
 * ([[feedback_an_assert_config_that_never_fires_is_not_coverage]]).
 * -DFT_ABORT_CLAIM turns it into an abort at the first sighting.
 */
static
int ft_ab_is_alarm(unsigned int cls, unsigned int own)
{
	if (cls == FT_AB_SW)
		return 1;
	return cls == FT_AB_MW_STRUCT && own == FT_AB_OWN_HELD;
}

static inline
void ft_ab_count_pub_pair(int in_node)
{
	ft_tk_tls_get()->pub_pair[!!in_node]++;
}

static inline
void ft_ab_count_lost(struct ft_tk_site *site)
{
	struct ft_tk_tls *tls = ft_tk_tls_get();
	int id = site ? ft_tk_site_id(site) : FT_TK_OVERFLOW_ID;
	unsigned int cls, own;

	if (!ft_ab_lost_valid) {
		tls->ab_unattrib++;
		return;
	}
	cls = ft_ab_code_cls(ft_ab_lost);
	own = ft_ab_code_own(ft_ab_lost);
	if (cls >= FT_AB_CLS_NR)
		cls = FT_AB_UNSET;	/* a stamp this build does not know */
	tls->ab[id][cls]++;
	tls->ab_own[cls][own]++;
	if (ft_ab_lost & FT_AB_MIXED)
		tls->ab_mixed++;
	if (ft_ab_is_alarm(cls, own)) {
		int ptr = ft_ab_lost_tag == FT_FLIP_PROXY_TAG;

		tls->ab_alarm[id][ptr]++;
#ifdef FT_ABORT_CLAIM
		fprintf(stderr,
"    at %s:%d %s -- class=%s witness=%u word=%s%s\n",
			site ? site->file : "?", site ? site->line : 0,
			site ? site->what : "?", ft_ab_cls_name(cls), own,
			ptr ? "CHILD POINTER SLOT" : "packed state word",
			(ft_ab_lost & FT_AB_MIXED) ? " MIXED" : "");
		ft_ab_claim_armed = 1;	/* the caller prints its registry, then aborts */
#endif
	}
}
#endif /* FT_ABORT_ATTRIB */

/*
 * The site object is a function-static: one per expansion of the constructor
 * macro, which is one per source line that creates a txn.
 */
#define FT_TK_SITE_HERE(what_)						\
	__extension__ ({						\
		static struct ft_tk_site __ft_tk_site = {		\
			__FILE__, __LINE__, (what_), -1,		\
		};							\
		&__ft_tk_site;						\
	})

/*
 * THE SITE PARAMETER EXISTS ONLY IN THE INSTRUMENTED BUILD, spelled into the
 * constructors' signatures rather than added unconditionally, so that a build
 * without the knob is not merely "cheap" but IDENTICAL: an extra always-NULL
 * argument is free at runtime yet still moves the compiler's inlining and
 * layout decisions, and this instrument must not perturb the very code paths
 * whose latency it is used to compare.
 */
#define FT_TK_SITE_PARAM	struct ft_tk_site *dbg_site,
#define FT_TK_SITE_FWD		dbg_site,

struct ft_tk_row {
	const struct ft_tk_site *site;
	unsigned long rec[FT_TK_REC_NR];
	unsigned long end[FT_TK_END_NR];
	unsigned long created;
	unsigned long armed;
};

static
int ft_tk_row_cmp(const void *a, const void *b)
{
	const struct ft_tk_row *ra = (const struct ft_tk_row *) a;
	const struct ft_tk_row *rb = (const struct ft_tk_row *) b;

	/* The conversion surface first: that is what the table is read for. */
	if (ra->rec[FT_TK_MW_STRUCT] != rb->rec[FT_TK_MW_STRUCT])
		return ra->rec[FT_TK_MW_STRUCT] > rb->rec[FT_TK_MW_STRUCT] ? -1 : 1;
	if (ra->end[FT_TK_ABORT] != rb->end[FT_TK_ABORT])
		return ra->end[FT_TK_ABORT] > rb->end[FT_TK_ABORT] ? -1 : 1;
	return strcmp(ra->site->file, rb->site->file);
}

static
void ft_tk_dump(void)
{
	struct ft_tk_row *rows;
	struct ft_tk_tls *tls;
	unsigned long cell_mw = 0;
	unsigned long mwa[FT_TK_MWA_NR];
	unsigned long mwa_tot = 0;
#ifdef FT_ABORT_ATTRIB
	unsigned long ab[FT_AB_CLS_NR], ab_own[FT_AB_CLS_NR][FT_AB_OWN_NR];
	unsigned long ab_alarm_site[FT_TK_MAX_SITES][2];
	unsigned long ab_tot = 0, ab_alarm[2] = { 0, 0 }, pub_pair[2] = { 0, 0 };
	unsigned long ab_mixed = 0, ab_unattrib = 0;
#ifdef FT_WINNER_DBG
	unsigned long win_beater[FT_AB_CLS_NR][FT_AB_OWN_NR];
	unsigned long win_raw[FT_WIN_RAW_NR];
	unsigned long win_owner[3] = { 0, 0, 0 };
	unsigned long win_beater_proxy = 0, win_beater_ledger = 0;
	unsigned long win_ledger_mismatch = 0, win_ledger_absent = 0;
	unsigned long win_seen_none = 0, win_drift = 0;
	unsigned long win_tid_same = 0, win_torn = 0;
	unsigned long win_owner_tomb = 0, win_owner_unparented = 0;
	unsigned long win_note = 0, win_note_skip = 0;
#endif
#endif
	struct ft_tk_row tot;
	int nr, i, threads = 0;
	char name[96];

	pthread_mutex_lock(&ft_tk_lock);
	nr = ft_tk_nr_sites;
	if (ft_tk_sites[FT_TK_OVERFLOW_ID])
		nr = FT_TK_MAX_SITES;
	memset(mwa, 0, sizeof(mwa));
#ifdef FT_ABORT_ATTRIB
	memset(ab, 0, sizeof(ab));
	memset(ab_own, 0, sizeof(ab_own));
	memset(ab_alarm_site, 0, sizeof(ab_alarm_site));
#ifdef FT_WINNER_DBG
	memset(win_beater, 0, sizeof(win_beater));
	memset(win_raw, 0, sizeof(win_raw));
#endif
#endif
	rows = (struct ft_tk_row *) calloc(nr ? nr : 1, sizeof(*rows));
	if (!rows) {
		pthread_mutex_unlock(&ft_tk_lock);
		return;
	}
	for (i = 0; i < nr; i++)
		rows[i].site = ft_tk_sites[i];
	for (tls = ft_tk_threads; tls; tls = tls->next) {
		int c;

		threads++;
		cell_mw += tls->cell_mw;
		for (i = 0; i < FT_TK_MWA_NR; i++)
			mwa[i] += tls->mwa[i];
#ifdef FT_ABORT_ATTRIB
		{
			int c2, o2;

			for (c2 = 0; c2 < FT_AB_CLS_NR; c2++) {
				for (i = 0; i < nr; i++)
					ab[c2] += tls->ab[i][c2];
				for (o2 = 0; o2 < FT_AB_OWN_NR; o2++)
					ab_own[c2][o2] += tls->ab_own[c2][o2];
			}
			for (i = 0; i < nr; i++) {
				for (o2 = 0; o2 < 2; o2++) {
					ab_alarm[o2] += tls->ab_alarm[i][o2];
					ab_alarm_site[i][o2] += tls->ab_alarm[i][o2];
				}
			}
			ab_mixed += tls->ab_mixed;
			ab_unattrib += tls->ab_unattrib;
			pub_pair[0] += tls->pub_pair[0];
			pub_pair[1] += tls->pub_pair[1];
#ifdef FT_WINNER_DBG
			for (c2 = 0; c2 < FT_AB_CLS_NR; c2++)
				for (o2 = 0; o2 < FT_AB_OWN_NR; o2++)
					win_beater[c2][o2] += tls->win_beater[c2][o2];
			for (o2 = 0; o2 < FT_WIN_RAW_NR; o2++)
				win_raw[o2] += tls->win_raw[o2];
			for (o2 = 0; o2 < 3; o2++)
				win_owner[o2] += tls->win_owner[o2];
			win_beater_proxy += tls->win_beater_proxy;
			win_beater_ledger += tls->win_beater_ledger;
			win_ledger_mismatch += tls->win_ledger_mismatch;
			win_ledger_absent += tls->win_ledger_absent;
			win_seen_none += tls->win_seen_none;
			win_drift += tls->win_drift;
			win_tid_same += tls->win_tid_same;
			win_torn += tls->win_torn;
			win_owner_tomb += tls->win_owner_tomb;
			win_owner_unparented += tls->win_owner_unparented;
			win_note += tls->win_note;
			win_note_skip += tls->win_note_skip;
#endif
		}
#endif
		for (i = 0; i < nr; i++) {
			if (!rows[i].site)
				continue;
			for (c = 0; c < FT_TK_REC_NR; c++)
				rows[i].rec[c] += tls->rec[i][c];
			for (c = 0; c < FT_TK_END_NR; c++)
				rows[i].end[c] += tls->end[i][c];
			rows[i].created += tls->created[i];
			rows[i].armed += tls->armed[i];
		}
	}
	pthread_mutex_unlock(&ft_tk_lock);

	/* Drop the rows that never registered (a full table has holes). */
	for (i = 0; i < nr; ) {
		if (rows[i].site) {
			i++;
			continue;
		}
		rows[i] = rows[--nr];
	}
	qsort(rows, nr, sizeof(*rows), ft_tk_row_cmp);

	memset(&tot, 0, sizeof(tot));
	fprintf(stderr,
"\n=== FT_DEBUG_TXN_KIND: record kind + commit outcome, per txn creation site ===\n"
"    threads=%d sites=%d\n"
"%-44s %9s %7s %10s %10s %10s %10s %8s %10s %10s %9s %10s %9s %8s %7s %8s\n",
		threads, nr,
		"site", "created", "armSW",
		"SW", "MW_STRUCT", "MW_ALWAYS", "MW_LOCK", "VALID",
		"OWN_HELD", "OWN_LEDGER", "OWN_MISS",
		"OK", "ABORT", "MEMERR", "MISS", "BAILED");
	for (i = 0; i < nr; i++) {
		int c;

		if (rows[i].site->line)
			snprintf(name, sizeof(name), "%s:%d %s",
				rows[i].site->file, rows[i].site->line,
				rows[i].site->what);
		else
			snprintf(name, sizeof(name), "%s", rows[i].site->file);
		fprintf(stderr,
"%-44s %9lu %7lu %10lu %10lu %10lu %10lu %8lu %10lu %10lu %9lu %10lu %9lu %8lu %7lu %8lu\n",
			name, rows[i].created, rows[i].armed,
			rows[i].rec[FT_TK_SW], rows[i].rec[FT_TK_MW_STRUCT],
			rows[i].rec[FT_TK_MW_ALWAYS], rows[i].rec[FT_TK_MW_LOCK],
			rows[i].rec[FT_TK_VALIDATE],
			rows[i].rec[FT_TK_OWN_HELD], rows[i].rec[FT_TK_OWN_LEDGER],
			rows[i].rec[FT_TK_OWN_MISS],
			rows[i].end[FT_TK_OK], rows[i].end[FT_TK_ABORT],
			rows[i].end[FT_TK_MEMERR], rows[i].end[FT_TK_MISS],
			rows[i].end[FT_TK_BAILED]);
		for (c = 0; c < FT_TK_REC_NR; c++)
			tot.rec[c] += rows[i].rec[c];
		for (c = 0; c < FT_TK_END_NR; c++)
			tot.end[c] += rows[i].end[c];
		tot.created += rows[i].created;
		tot.armed += rows[i].armed;
	}
	fprintf(stderr,
"%-44s %9lu %7lu %10lu %10lu %10lu %10lu %8lu %10lu %10lu %9lu %10lu %9lu %8lu %7lu %8lu\n",
		"TOTAL", tot.created, tot.armed,
		tot.rec[FT_TK_SW], tot.rec[FT_TK_MW_STRUCT],
		tot.rec[FT_TK_MW_ALWAYS], tot.rec[FT_TK_MW_LOCK],
		tot.rec[FT_TK_VALIDATE],
		tot.rec[FT_TK_OWN_HELD], tot.rec[FT_TK_OWN_LEDGER],
		tot.rec[FT_TK_OWN_MISS],
		tot.end[FT_TK_OK], tot.end[FT_TK_ABORT],
		tot.end[FT_TK_MEMERR], tot.end[FT_TK_MISS],
		tot.end[FT_TK_BAILED]);
	fprintf(stderr,
"    cell/hlist MW stores (recorded straight on the engine handle, not site-attributed): %lu\n",
		cell_mw);

	/*
	 * MW_ALWAYS, SPLIT BY THE BRANCH THAT RECORDED IT.  The classes sum to
	 * the MW_ALWAYS total above -- that equality is the table's self-check,
	 * and it is printed rather than asserted because a mismatch means a
	 * record branch was added without a class, which is a fact about the
	 * table, not a reason to kill the run.
	 */
	{
		static const char * const mwa_name[FT_TK_MWA_NR] = {
			"ROOT", "HEAD_BACK", "DUAL", "CELL",
			"RANK", "PARENT_WORD", "PSO", "STATE",
			"GUARD",
		};
		static const char * const mwa_what[FT_TK_MWA_NR] = {
			"&ft->root -- no node to lock, NEVER converts",
			"external head back channel -- no state word, NEVER converts",
			"structural edge, owner NOT held (SKIP_X dual) -- RECLASSIFIED by af22756b; needs a wider lock-set",
			"ordered-cell / dup-chain edge -- THE G4 LANE",
			"nr_keys up unlocked ancestors -- Phase E (root-only spacing)",
			"child parent_word, child not held (reparent sweep) -- lock-set reach",
			"child parent_slot_offset -- §8.3 layout split retires it",
			"the reparent sweep's own child state {live->live} -- a validate; must stay MW",
			"§4.B guard on an INSTALLED value node -- a validate; must stay MW",
		};

		for (i = 0; i < FT_TK_MWA_NR; i++)
			mwa_tot += mwa[i];
		fprintf(stderr,
"\n    MW_ALWAYS split by RECORD BRANCH (the G4 input -- one number was four populations):\n");
		for (i = 0; i < FT_TK_MWA_NR; i++)
			fprintf(stderr,
"      %-12s %14lu  %5.1f%%  %s\n",
				mwa_name[i], mwa[i],
				mwa_tot ? 100.0 * (double) mwa[i] /
					(double) mwa_tot : 0.0,
				mwa_what[i]);
		fprintf(stderr,
"      %-12s %14lu  (MW_ALWAYS column %lu%s)\n",
			"sum", mwa_tot, tot.rec[FT_TK_MW_ALWAYS],
			mwa_tot == tot.rec[FT_TK_MW_ALWAYS] ? "" :
				" -- ☠ MISMATCH: an always-MW branch records no class");
	}

#ifdef FT_ABORT_ATTRIB
	/*
	 * WHICH RECORD LOST, over every aborted commit.  The rows sum to the
	 * ABORT column above -- the same self-check the MW_ALWAYS table uses,
	 * and here it also proves the engine hooks cover every abort EXIT (they
	 * are three, and the lone-MW-edge one builds no descriptor at all).
	 */
	{
		for (i = 0; i < FT_AB_CLS_NR; i++)
			ab_tot += ab[i];
		fprintf(stderr,
"\n    ABORT attributed to the LOSING RECORD's class (%lu aborted commits):\n",
			ab_tot);
		for (i = 0; i < FT_AB_CLS_NR; i++) {
			if (!ab[i])
				continue;
			fprintf(stderr,
"      %-16s %12lu  %5.1f%%   held %lu / ledger %lu / miss %lu / n-a %lu\n",
				ft_ab_names[i], ab[i],
				ab_tot ? 100.0 * (double) ab[i] / (double) ab_tot : 0.0,
				ab_own[i][FT_AB_OWN_HELD], ab_own[i][FT_AB_OWN_LEDGER],
				ab_own[i][FT_AB_OWN_MISS], ab_own[i][FT_AB_OWN_NA]);
		}
		fprintf(stderr,
"      %-16s %12lu  (ABORT column %lu%s)\n"
"      mixed-class losing words: %lu   aborts no hook could name: %lu\n",
			"sum", ab_tot, tot.end[FT_TK_ABORT],
			ab_tot + ab_unattrib == tot.end[FT_TK_ABORT] ? "" :
				" -- ☠ MISMATCH: an abort EXIT is unhooked",
			ab_mixed, ab_unattrib);
		/*
		 * ☠ SPLIT BY THE WORD, because the two halves are not the same
		 * finding.  A child POINTER slot may only be written by a holder
		 * of the owning node's lock, so an owner-held record losing one
		 * is an exclusion that did not exclude.  The packed STATE word
		 * has lock-free writers BY DESIGN (the re-parent sweep's guard,
		 * the §4.B validates -- the MW_ALWAYS populations above), so
		 * losing one says only that a non-holder wrote another field of
		 * the same word.  Pooling them would manufacture a defect.
		 */
		fprintf(stderr,
"    glue publish PAIRING (ft_slot_in_node(publish_parent, publish_slot)):"
" in %lu / ☠ NOT-IN %lu\n",
			pub_pair[1], pub_pair[0]);
		fprintf(stderr,
"    ★ ALARM (a record that cannot lose, lost): pointer-slot %lu / state-word %lu\n"
"      %s\n",
			ab_alarm[1], ab_alarm[0],
			ab_alarm[1] ?
				"☠ POINTER SLOT: an OWNER-HELD structural edge lost a CAS on a word only a holder may write" :
				(ab_alarm[0] ?
					"state word only: a non-holder wrote another field of the same packed word (by design)" :
					"(none -- the exclusion held everywhere it was claimed)"));
		/*
		 * ☠ BY SITE ID, not by row index: the rows were sorted by
		 * MW_STRUCT above, so row i is no longer site i.
		 */
		for (i = 0; (ab_alarm[0] || ab_alarm[1]) && i < nr; i++) {
			int id = rows[i].site->id;

			if (id >= 0 && id < FT_TK_MAX_SITES &&
					(ab_alarm_site[id][0] || ab_alarm_site[id][1]))
				fprintf(stderr,
					"        %s:%d %s  ptr %lu / state %lu\n",
					rows[i].site->file, rows[i].site->line,
					rows[i].site->what,
					ab_alarm_site[id][1], ab_alarm_site[id][0]);
		}
#ifdef FT_WINNER_DBG
		/*
		 * WHO BEAT the alarmed word.  The primary witness is the value
		 * the losing CAS OBSERVED (URCU_TXN_REC_LOST): a proxy there
		 * decodes to the winning record EXACTLY; a plain value is only
		 * ledger-CORROBORATED (a population claim -- ABA can name an
		 * older write of the same value).  The rows are the WINNER's
		 * label; the loser is always the owner-held structural record
		 * above.  The reach line is the proof the stamps ran at all
		 * ([[feedback_verify_the_mechanism_ran_before_believing_a_zero]]).
		 */
		{
			static const char * const raw_names[FT_WIN_RAW_NR] = {
				"lone-edge flip (ft_ord_cell_flip_one)",
				"remove head-promote back store",
			};
			unsigned long win_tot = 0, raw_tot = 0;
			int c3, o3;

			for (c3 = 0; c3 < FT_AB_CLS_NR; c3++)
				for (o3 = 0; o3 < FT_AB_OWN_NR; o3++)
					win_tot += win_beater[c3][o3];
			for (o3 = 0; o3 < FT_WIN_RAW_NR; o3++)
				raw_tot += win_raw[o3];
			fprintf(stderr,
"\n    ★ BEATER of the alarmed word (-DFT_WINNER_DBG):\n"
"      ledger reach: %lu writes stamped / %lu skipped on entry contention\n"
"      engine winner %lu = ★ proxy-decoded (exact) %lu + ledger-corroborated %lu (self-tid %lu)\n"
"      ☠ raw-lane winner %lu / plain unmatched: ledger-mismatch %lu + no-entry %lu / torn %lu\n"
"      no observed value at the loss exit: %lu    drift (slot moved again before lookup): %lu\n"
"      corroborated winner's named owner vs the loser's: same %lu / ☠ DIFFERENT %lu / unknown %lu\n"
"      loser's registered owner AT the alarm: ☠ TOMBSTONED %lu / ☠ MID RE-HOME (parent_word NULL) %lu\n",
				win_note, win_note_skip,
				win_tot, win_beater_proxy, win_beater_ledger,
				win_tid_same,
				raw_tot, win_ledger_mismatch, win_ledger_absent,
				win_torn, win_seen_none, win_drift,
				win_owner[0], win_owner[1], win_owner[2],
				win_owner_tomb, win_owner_unparented);
			for (c3 = 0; c3 < FT_AB_CLS_NR; c3++) {
				for (o3 = 0; o3 < FT_AB_OWN_NR; o3++) {
					if (!win_beater[c3][o3])
						continue;
					fprintf(stderr,
"        %-16s witness=%s %12lu  %5.1f%%\n",
						ft_ab_cls_name(c3),
						o3 == FT_AB_OWN_HELD ? "held" :
						(o3 == FT_AB_OWN_LEDGER ? "ledger" :
						(o3 == FT_AB_OWN_MISS ? "MISS" : "n/a")),
						win_beater[c3][o3],
						win_tot ? 100.0 *
							(double) win_beater[c3][o3] /
							(double) win_tot : 0.0);
				}
			}
			for (o3 = 0; o3 < FT_WIN_RAW_NR; o3++)
				if (win_raw[o3])
					fprintf(stderr,
"        ☠ raw: %-40s %12lu\n",
						raw_names[o3], win_raw[o3]);
		}
#endif
	}
#endif
	fprintf(stderr,
"    MW_STRUCT is the conversion surface; MW_ALWAYS + MW_LOCK + the cell/hlist line stay MW by design.\n"
"    OWN_HELD/OWN_LEDGER/OWN_MISS split MW_STRUCT (the surface) by whether the op holds the word's owner; they sum to it:\n"
"      OWN_HELD   the txn registry names it.  OWN_LEDGER  only this thread's hold ledger does (a REGISTRY gap;\n"
"      needs -DFEATURE_FT_HOLD_TRACE or it reads 0).  OWN_MISS  neither: a real exclusion gap.\n"
"    a site with OWN_MISS == 0 is ready for the Phase B per-op arm; OWN_MISS is the size of its exclusion gap.\n\n");
	free(rows);
}

static __attribute__((destructor))
void ft_tk_dump_at_exit(void)
{
	ft_tk_dump();
}

/*
 * What a struct ft_flip_txn carries for this instrument, and how the record
 * helpers reach it.  @dbg_lock_take is the one piece the dispatch cannot infer:
 * the DLM lock take goes through the SAME structural record helper as every
 * content edge, and only its caller knows it is the arbitration point.
 */
#define FT_TK_TXN_FIELDS						\
	struct ft_tk_site *dbg_site;					\
	bool dbg_lock_take;						\
	bool dbg_ended;

/*
 * ☠ FIRST, before ANY other counting call on @t.  Every other FT_TK_* macro
 * reads @t->dbg_site to find the row it credits, and a constructor's @t comes
 * from malloc -- so a count taken before this one dereferences uninitialised
 * memory as a struct ft_tk_site *.  It does not read as an instrumentation bug
 * when it lands, either: the fault is inside the trie's own hot path, on a
 * thread doing ordinary work.  FT_TK_COUNT_ARMED is the one that will find it,
 * because arming is decided inside the constructor itself.
 */
#define FT_TK_TXN_INIT(t, site)						\
	do {								\
		(t)->dbg_site = (site);					\
		(t)->dbg_lock_take = false;				\
		(t)->dbg_ended = false;					\
		ft_tk_count_created(site);				\
	} while (0)

#define FT_TK_TXN_SITE(t)		((t)->dbg_site)
#define FT_TK_TXN_IS_TAKE(t)		((t)->dbg_lock_take)
#define FT_TK_TXN_SET_TAKE(t, v)	do { (t)->dbg_lock_take = (v); } while (0)
#define FT_TK_COUNT_REC(t, c)		ft_tk_count_rec((t)->dbg_site, (c))
/*
 * THE UNION IS THE HELD SET, and the three-way split says which witness saw it.
 * Asked registry-first because that is the cheap one and the one a shipped
 * (non-test) build can also answer.
 */
#define FT_TK_COUNT_OWN(t, owner)					\
	ft_tk_count_rec((t)->dbg_site,					\
		ft_flip_txn_owns((t), (owner)) ? FT_TK_OWN_HELD :	\
		(ft_hold_trace_holds((owner)) ? FT_TK_OWN_LEDGER :	\
			FT_TK_OWN_MISS))
#define FT_TK_COUNT_ARMED(t)		ft_tk_count_armed((t)->dbg_site)
#define FT_TK_COUNT_CELL_MW()		ft_tk_count_cell_mw()
/*
 * THE CLASS ARGUMENT EXISTS ONLY IN THE INSTRUMENTED BUILD, for the reason the
 * site parameter does (see FT_TK_SITE_PARAM): an extra always-constant argument
 * is free at runtime and still moves the compiler's inlining decisions, and
 * this instrument must not perturb the paths it measures.  Written as a
 * TRAILING pair so a call site reads
 * ft_flip_txn_record_tag_mw(t, slot, o, n, tag FT_TK_MWA(FT_TK_MWA_ROOT)).
 */
#define FT_TK_MWA_PARAM			, enum ft_tk_mwa_class dbg_mwa
#define FT_TK_MWA(c)			, (c)
#define FT_TK_COUNT_MWA(c)		ft_tk_count_mwa(c)
#ifdef FT_ABORT_ATTRIB
#define FT_AB_LOST_RESET()		do { ft_ab_lost_valid = 0; } while (0)
#define FT_AB_COUNT_LOST(t)		ft_ab_count_lost((t)->dbg_site)
#define FT_AB_COUNT_PUB_PAIR(ok)	ft_ab_count_pub_pair(ok)
#ifdef FT_ABORT_CLAIM
/*
 * THE OTHER HALF OF THE CLAIM, and it has to be a macro expanded at the call
 * site: the op's LOCK REGISTRY is what the alarm is an accusation about, and
 * struct ft_flip_txn is not defined yet where ft_ab_count_lost is.  The engine
 * printed the descriptor; this prints WHO THIS OP THOUGHT IT HELD, which is the
 * pair the diagnosis lives in.
 */
#define FT_AB_CLAIM_REPORT(t_)						\
	do {								\
		unsigned int i_;					\
									\
		if (!ft_ab_claim_armed)					\
			break;						\
		fprintf(stderr,						\
			"    registry: nr_locks=%u sw=%d miss=%d\n",	\
			(t_)->nr_locks, (int) (t_)->structural_sw,	\
			(int) (t_)->acquire_miss);			\
		for (i_ = 0; i_ < (t_)->nr_locks; i_++)			\
			fprintf(stderr, "      held[%u] meta=%p snap=0x%lx\n", \
				i_, (void *) (t_)->locks[i_].meta,	\
				(unsigned long) (t_)->locks[i_].snap);	\
		abort();						\
	} while (0)
#else
#define FT_AB_CLAIM_REPORT(t_)		do { } while (0)
#endif
#else
#define FT_AB_LOST_RESET()		do { } while (0)
#define FT_AB_COUNT_LOST(t)		do { } while (0)
#define FT_AB_CLAIM_REPORT(t)		do { } while (0)
#define FT_AB_COUNT_PUB_PAIR(ok)	do { } while (0)
#endif
/*
 * An outcome is recorded ONCE per txn: ft_flip_txn_commit's acquire-miss arm
 * reports MISS and then destroys the handle, and the destroy must not also
 * report BAILED for it.
 */
#define FT_TK_COUNT_END(t, c)						\
	do {								\
		if (!(t)->dbg_ended) {					\
			(t)->dbg_ended = true;				\
			ft_tk_count_end((t)->dbg_site, (c));		\
		}							\
	} while (0)

#else	/* !FT_DEBUG_TXN_KIND */

struct ft_tk_site;	/* incomplete: the NULL the constructors take */

#define FT_TK_SITE_PARAM
#define FT_TK_SITE_FWD
#define FT_TK_TXN_FIELDS
#define FT_TK_TXN_INIT(t, site)		do { } while (0)
#define FT_TK_TXN_IS_TAKE(t)		0
#define FT_TK_TXN_SET_TAKE(t, v)	do { } while (0)
#define FT_TK_COUNT_REC(t, c)		do { } while (0)
#define FT_TK_COUNT_OWN(t, held)	do { } while (0)
#define FT_TK_COUNT_END(t, c)		do { } while (0)
#define FT_TK_COUNT_ARMED(t)		do { } while (0)
#define FT_TK_COUNT_CELL_MW()		do { } while (0)
#define FT_TK_MWA_PARAM
#define FT_TK_MWA(c)
#define FT_TK_COUNT_MWA(c)		do { } while (0)
#define FT_AB_LOST_RESET()		do { } while (0)
#define FT_AB_COUNT_LOST(t)		do { } while (0)
#define FT_AB_CLAIM_REPORT(t)		do { } while (0)
#define FT_AB_COUNT_PUB_PAIR(ok)	do { } while (0)

#endif	/* FT_DEBUG_TXN_KIND */

#endif /* _FT_TXN_KIND_STATS_H */
