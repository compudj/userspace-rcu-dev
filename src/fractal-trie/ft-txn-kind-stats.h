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
	FT_TK_REC_NR,
};

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

struct ft_tk_tls {
	struct ft_tk_tls *next;
	unsigned long rec[FT_TK_MAX_SITES][FT_TK_REC_NR];
	unsigned long end[FT_TK_MAX_SITES][FT_TK_END_NR];
	unsigned long created[FT_TK_MAX_SITES];
	unsigned long armed[FT_TK_MAX_SITES];
	unsigned long cell_mw;	/* not site-attributed, see the header comment */
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
	struct ft_tk_row tot;
	int nr, i, threads = 0;
	char name[96];

	pthread_mutex_lock(&ft_tk_lock);
	nr = ft_tk_nr_sites;
	if (ft_tk_sites[FT_TK_OVERFLOW_ID])
		nr = FT_TK_MAX_SITES;
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
"%-44s %9s %7s %10s %10s %10s %10s %8s %10s %9s %8s %7s %8s\n",
		threads, nr,
		"site", "created", "armSW",
		"SW", "MW_STRUCT", "MW_ALWAYS", "MW_LOCK", "VALID",
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
"%-44s %9lu %7lu %10lu %10lu %10lu %10lu %8lu %10lu %9lu %8lu %7lu %8lu\n",
			name, rows[i].created, rows[i].armed,
			rows[i].rec[FT_TK_SW], rows[i].rec[FT_TK_MW_STRUCT],
			rows[i].rec[FT_TK_MW_ALWAYS], rows[i].rec[FT_TK_MW_LOCK],
			rows[i].rec[FT_TK_VALIDATE],
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
"%-44s %9lu %7lu %10lu %10lu %10lu %10lu %8lu %10lu %9lu %8lu %7lu %8lu\n",
		"TOTAL", tot.created, tot.armed,
		tot.rec[FT_TK_SW], tot.rec[FT_TK_MW_STRUCT],
		tot.rec[FT_TK_MW_ALWAYS], tot.rec[FT_TK_MW_LOCK],
		tot.rec[FT_TK_VALIDATE],
		tot.end[FT_TK_OK], tot.end[FT_TK_ABORT],
		tot.end[FT_TK_MEMERR], tot.end[FT_TK_MISS],
		tot.end[FT_TK_BAILED]);
	fprintf(stderr,
"    cell/hlist MW stores (recorded straight on the engine handle, not site-attributed): %lu\n",
		cell_mw);
	fprintf(stderr,
"    MW_STRUCT is the conversion surface; MW_ALWAYS + MW_LOCK + the cell/hlist line stay MW by design.\n\n");
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
#define FT_TK_COUNT_ARMED(t)		ft_tk_count_armed((t)->dbg_site)
#define FT_TK_COUNT_CELL_MW()		ft_tk_count_cell_mw()
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
#define FT_TK_COUNT_END(t, c)		do { } while (0)
#define FT_TK_COUNT_ARMED(t)		do { } while (0)
#define FT_TK_COUNT_CELL_MW()		do { } while (0)

#endif	/* FT_DEBUG_TXN_KIND */

#endif /* _FT_TXN_KIND_STATS_H */
