/*
 * ft-stale-disposal-rig.c -- the CONCURRENT reproducer for the two disposal
 * defects described in doc/design/ft-stale-disposal-predicate.md.
 *
 * NOT BUILT.  It is not in any _SOURCES and the build system never sees it.
 * It is kept in the tree because §5 of that document reports MEASURED counts,
 * and a measured table whose workload is not in the tree cannot be re-derived.
 * ☠ THE LIBRARY MUST BE BUILT WITH -DFT_ENABLE_TRACING for CHK=1 to mean
 * anything.  The violation hook, the ft_dbg_* counters and the rm-site stamp
 * are one subsystem behind that flag; without it the weak reference below is
 * NULL and every violation is recorded SILENTLY as nothing.  An LTTng session
 * is NOT required -- the classifier prints to stderr -- only the build flag is.
 * The rig refuses to start in that configuration rather than hand back a wrong
 * zero, but the refusal only exists because a 120-seed sweep already produced
 * one: 42 oracles fired, 0 discriminator lines.
 *
 * Build it by hand against a configured tree -- never against an installed
 * liburcu, which would silently shadow the tree you meant to test:
 *
 *   gcc -O0 -g -I<top>/include -I<build>/include -I<top>/src -I<top>  *       doc/design/ft-stale-disposal-rig.c -o /tmp/ftrig  *       -L<build>/src/.libs -lurcu-qsbr -lurcu-cds -lurcu-common  *       -Wl,-rpath,<build>/src/.libs
 *   ldd /tmp/ftrig | grep urcu     # PROVE it resolved into <build>, not /usr
 *
 * SHAPE.  A set of STABLE keys is inserted once and never touched.  W writers
 * churn a DISJOINT set of keys (insert then remove, forever).  The churn set is
 * PARTITIONED across the writers, so each key's bookkeeping has exactly one
 * owner and none of the writer oracles below can race on it.  R readers pick a
 * random stable key and do an exact eager lookup.  Keys are drawn from a small
 * alphabet at mixed lengths so PREFIX heads -- a key ending AT an internal node
 * -- are dense on the paths the readers descend.
 *
 * THE READER ORACLES (always on).  A reader miss is retried once IN THE SAME
 * read section, which is what separates the two:
 *   transient  first attempt failed, retry succeeded   -- a reachability gap
 *   hard       both attempts failed                    -- a key was LOST
 *   wrongid    found, but the node's tag is another key's
 *
 * THE WRITER ORACLES (CHK=1).  These are the ones the disposal defects trip.
 * Each aborts on the spot, after calling cds_ft_debug_ext_violation(), which
 * emits the cds_ft:ext_violation tracepoint and prints an EXTVIOL line whose
 * anctomb/ancroot fields say how far up a TOMBSTONED ancestor sits -- the
 * discriminator that separates the two defects:
 *   kind 0  RM-LOOKUP-MISS  a key we hold present is not found to remove
 *   kind 1  RM-FAIL         cds_ft_remove REFUSED a key it just looked up
 *   kind 2  STALE-FOUND     a key removed with status OK is still findable
 *   kind 3  RM-WRONG-NODE   the lookup returned a node we did not insert
 *   kind 4  DUP-CHAINED     an insert reported OK for a key already present
 *   kind 5  PRESENT-MISSING a key we hold present is not found at all
 *
 * env: NSTABLE NCHURN ALPHA MAXLEN SEED SECS READERS WRITERS LIST=0 ORD=1
 *      CHK=1   arm the writer oracles above (off by default)
 *      NOFREE=1  leak retired nodes instead of recycling them.  Set this for
 *                any pointer-level analysis: a reused malloc chunk makes a live
 *                node and a long-dead one read identically.
 *                ☠ RUN BOTH ARMS.  It is also a BLIND SPOT: no address is ever
 *                reused, so any defect whose mechanism needs a RECLAIMED node
 *                cannot occur under it.  MEASURED: 160 seeds at NOFREE=1
 *                produced ZERO kind-3 RM-WRONG-NODE; the same 160 at NOFREE=0
 *                produced two.  It also suppresses the memory cost of a lane
 *                that spins inside a read-side bracket, since leaking means
 *                call_rcu never has a backlog to defer (memcg kills 2 -> 8).
 *      WRITER=coarse  serialise all structural writers on the FT-wide lock
 *                (default fine).  ☞ The coarser lock SPACINGS are NOT settable
 *                here: cds_ft_group_attr_set_lock_spacing REFUSES exponential
 *                and root-only by design, so they need CDS_FT_LOCK_SPACING=...
 *                AND a library built with -DFEATURE_FT_LOCK_SPACING_ENV.  The
 *                rig echoes an "ARM writer=... spacing_env=..." line so a
 *                mislabelled arm is visible instead of silently per-node.
 *      NOREM=1   insert only, never remove
 *      PFX=1     every churn key is a PROPER PREFIX of a stable key, so each
 *                insert/remove creates and destroys a PREFIX HEAD directly on
 *                a path the readers descend
 * exit 0 ok | 11 alarm | 20 hard miss | 21 wrong identity | 22 walk broke
 *      134 (SIGABRT) a CHK writer oracle fired -- read the EXTVIOL line
 */
#define _GNU_SOURCE
#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu/fractal-trie.h>
#include <urcu-call-rcu.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>

#define MAXK 16
struct key { uint8_t b[MAXK]; unsigned len; uint64_t tag; int present; struct cds_ft_node *last_rm; struct cds_ft_node *cur; };
struct tnode { struct cds_ft_node node; uint64_t tag; struct rcu_head rh; };
static void tn_free(struct rcu_head *h){ free(caa_container_of(h,struct tnode,rh)); }

static struct cds_ft *ft;
static volatile int stop;
static struct key *stable, *churn;
static int norem = 0, chk = 0, nofree = 0;
static int nstable = 200, nchurn = 200, alpha = 3, maxlen = 6, nreaders = 6, nwriters = 1, secs = 10, ord_walk = 1;
static unsigned long transient, hardmiss, wrongid, reads, churn_ops, walks;
static void alrm(int s){(void)s; _exit(11);}

static uint64_t rnd_state;
static uint64_t rnd(uint64_t *s){ *s ^= *s<<13; *s ^= *s>>7; *s ^= *s<<17; return *s; }

static struct tnode *na(uint64_t t){ struct tnode *n = calloc(1,sizeof(*n));
	if(!n)_exit(5); cds_ft_node_init(&n->node); n->tag = t; return n; }

static uint64_t ktag(const struct key *k){ uint64_t h = 1469598103934665603ULL; unsigned i;
	for(i=0;i<k->len;i++){ h ^= k->b[i]; h *= 1099511628211ULL; }
	return h ? h : 1; }

/* build @n distinct keys not already in @avoid[0..navoid) */
static void mkkeys(struct key *dst, int n, struct key *avoid, int navoid, uint64_t *s)
{
	int i = 0, guard = 0;
	while (i < n && guard++ < n*10000) {
		struct key k = {{0}}; unsigned j; int dup = 0;
		k.len = 1 + (unsigned)(rnd(s) % (uint64_t)maxlen);
		for (j = 0; j < k.len; j++) k.b[j] = (uint8_t)('a' + (rnd(s) % (uint64_t)alpha));
		for (j = 0; j < (unsigned)i; j++)
			if (dst[j].len==k.len && !memcmp(dst[j].b,k.b,k.len)) { dup=1; break; }
		for (j = 0; !dup && j < (unsigned)navoid; j++)
			if (avoid[j].len==k.len && !memcmp(avoid[j].b,k.b,k.len)) { dup=1; break; }
		if (dup) continue;
		k.tag = ktag(&k); dst[i++] = k;
	}
	if (i < n) { fprintf(stderr,"mkkeys: only %d/%d\n", i, n); _exit(5); }
}

static int lookup_tag(const struct key *k, uint64_t *tag_out)
{
	struct cds_ft_node *o = NULL;
	if (cds_ft_eager_lookup_key(ft, k->b, k->len, 0, &o) != CDS_FT_STATUS_OK)
		return 0;
	*tag_out = caa_container_of(o, struct tnode, node)->tag;
	return 1;
}

static void *reader(void *a)
{
	uint64_t s = (uint64_t)(uintptr_t)a * 2654435761ULL + 12345;
	struct cds_ft_iter *it = NULL;
	rcu_register_thread();
	if (ord_walk && cds_ft_iter_create(ft, &it) != CDS_FT_STATUS_OK) _exit(5);
	while (!stop) {
		const struct key *k = &stable[rnd(&s) % (uint64_t)nstable];
		uint64_t tag = 0;
		rcu_read_lock();
		if (!lookup_tag(k, &tag)) {
			if (lookup_tag(k, &tag)) {
				__atomic_fetch_add(&transient,1,__ATOMIC_RELAXED);
			} else {
				__atomic_fetch_add(&hardmiss,1,__ATOMIC_RELAXED);
				fprintf(stderr,"HARD miss len=%u %.*s\n", k->len,(int)k->len,k->b);
				rcu_read_unlock(); _exit(20);
			}
		}
		if (tag != k->tag) {
			__atomic_fetch_add(&wrongid,1,__ATOMIC_RELAXED);
			fprintf(stderr,"WRONG identity for %.*s\n",(int)k->len,k->b);
			rcu_read_unlock(); _exit(21);
		}
		if (ord_walk && (rnd(&s) & 15) == 0) {
			int n = 0; enum cds_ft_status st = cds_ft_lookup_first(ft, it);
			while (st == CDS_FT_STATUS_OK && n < 4*(nstable+nchurn)+16) { n++; st = cds_ft_next(ft,it); }
			if (n >= 4*(nstable+nchurn)+16) { fprintf(stderr,"walk did not terminate\n"); rcu_read_unlock(); _exit(22); }
			__atomic_fetch_add(&walks,1,__ATOMIC_RELAXED);
		}
		rcu_read_unlock();
		__atomic_fetch_add(&reads,1,__ATOMIC_RELAXED);
		rcu_quiescent_state();
	}
	if (it) cds_ft_iter_destroy(it);
	rcu_unregister_thread();
	return NULL;
}


/*
 * TWO-WRITER EXTERNAL-HEAD CAMPAIGN (2026-09-04): route every oracle violation
 * through the library's tracing hook, which emits cds_ft:ext_violation and THEN
 * runs the in-process FAST STOP before `lttng stop` / `lttng snapshot record`.
 * Emitting from here with a bare system() would let the ring wrap past the
 * window during the fork+exec (see fractal-trie-trace.h).  The hook only exists
 * in an FT_ENABLE_TRACING build; VIOL is a weak-symbol call so the same source
 * still builds against a shipping library.
 */
extern void cds_ft_debug_ext_violation(unsigned int kind, struct cds_ft *ft,
		struct cds_ft_node *node, uint64_t key0)
	__attribute__((weak));

static uint64_t viol_key0(const unsigned char *b, size_t len)
{
	uint64_t v = 0;
	size_t i;

	for (i = 0; i < 8; i++)
		v = (v << 8) | (i < len ? b[i] : 0);
	return v;
}

#define VIOL(kind, node, aux, kb, klen)	do {				\
	(void) (aux);							\
	if (cds_ft_debug_ext_violation)					\
		cds_ft_debug_ext_violation((kind), ft,			\
			(struct cds_ft_node *) (node),			\
			viol_key0((const unsigned char *) (kb), (klen)));\
} while (0)

static void *writer(void *a)
{
	uint64_t s = (uint64_t)(uintptr_t)a * 88172645463325252ULL + 7;
	struct cds_ft_iter *it;
	int id = (int)(uintptr_t)a, lo, hi, i;
	rcu_register_thread();
	if (cds_ft_iter_create(ft,&it) != CDS_FT_STATUS_OK) _exit(5);
	lo = id * (nchurn/nwriters); hi = (id==nwriters-1) ? nchurn : lo + nchurn/nwriters;
	while (!stop) {
		for (i = lo; i < hi && !stop; i++) {
			struct key *k = &churn[i];
			enum cds_ft_status st;
			rcu_read_lock();
			if (chk) {
				struct cds_ft_node *o = NULL;
				enum cds_ft_status ls = cds_ft_eager_lookup_key(ft, k->b, k->len, 0, &o);
				if (ls == CDS_FT_STATUS_OK && !k->present) {
					fprintf(stderr,"STALE-FOUND writer%d key=%.*s node=%p last_rm=%p same=%d next=%p\n",
						id,(int)k->len,k->b,(void*)o,(void*)k->last_rm,o==k->last_rm,(void*)o->next);
					VIOL(2, o, k->last_rm, k->b, k->len);
					abort();
				}
				if (ls != CDS_FT_STATUS_OK && k->present) {
					fprintf(stderr,"PRESENT-MISSING writer%d key=%.*s cur=%p\n",
						id,(int)k->len,k->b,(void*)k->cur);
					VIOL(5, k->cur, NULL, k->b, k->len);
					abort();
				}
			}
			{ struct tnode *tn = na(k->tag);
			  st = cds_ft_insert(ft, k->b, k->len, &tn->node);
			  if (st != CDS_FT_STATUS_OK) free(tn);
			  else { if (chk && k->present) { fprintf(stderr,"DUP-CHAINED writer%d key=%.*s\n",id,(int)k->len,k->b); VIOL(4, k->cur, NULL, k->b, k->len); abort(); }
			         k->present = 1; k->cur = &tn->node; } }
			rcu_read_unlock();
			rcu_quiescent_state();
			if (st != CDS_FT_STATUS_OK && st != CDS_FT_STATUS_DUPLICATE_FOUND) {
				fprintf(stderr,"insert %.*s -> %s\n",(int)k->len,k->b,cds_ft_status_to_string(st)); _exit(6); }
			__atomic_fetch_add(&churn_ops,1,__ATOMIC_RELAXED);
			(void) rnd(&s);
		}
		for (i = lo; i < hi && !stop && !norem; i++) {
			struct key *k = &churn[i];
			rcu_read_lock();
			if (cds_ft_iter_set_key(it,k->b,k->len) == CDS_FT_STATUS_OK &&
			    cds_ft_lookup(ft,it) == CDS_FT_STATUS_OK) {
				struct cds_ft_node *nd = cds_ft_iter_node(it);
				enum cds_ft_status rs;

				if (chk && nd != k->cur) { fprintf(stderr,"RM-WRONG-NODE writer%d key=%.*s nd=%p cur=%p\n",id,(int)k->len,k->b,(void*)nd,(void*)k->cur); VIOL(3, nd, k->cur, k->b, k->len); abort(); }
				rs = cds_ft_remove(ft,it,nd);
				if (rs == CDS_FT_STATUS_OK) {
					k->present = 0; k->last_rm = nd; k->cur = NULL;
					/*
					 * NOFREE: leak instead of recycling.  A
					 * reused malloc chunk makes a traced
					 * pointer ambiguous -- the address of a
					 * live node and of a long-dead one read
					 * identically -- and the whole analysis
					 * is "what happened to THIS node".
					 */
					if (!nofree)
						call_rcu(&caa_container_of(nd,struct tnode,node)->rh,
							tn_free);
				} else if (chk) { fprintf(stderr,"RM-FAIL writer%d key=%.*s st=%s\n",id,(int)k->len,k->b,cds_ft_status_to_string(rs)); VIOL(1, nd, k->cur, k->b, k->len); abort(); }
			} else if (chk && k->present) { fprintf(stderr,"RM-LOOKUP-MISS writer%d key=%.*s cur=%p\n",id,(int)k->len,k->b,(void*)k->cur); VIOL(0, k->cur, NULL, k->b, k->len); abort(); }
			rcu_read_unlock();
			rcu_quiescent_state();
			__atomic_fetch_add(&churn_ops,1,__ATOMIC_RELAXED);
		}
	}
	cds_ft_iter_destroy(it);
	rcu_unregister_thread();
	return NULL;
}

static int geti(const char*e,int d){const char*v=getenv(e);return v?atoi(v):d;}

int main(void)
{
	struct cds_ft_group *g; struct cds_ft_group_attr *at = NULL;
	pthread_t *rt, *wt; int i;
	uint64_t s;

	/*
	 * ☠ THE HOOK IS WEAK, SO ITS ABSENCE IS SILENT.  cds_ft_debug_ext_violation
	 * lives behind FT_ENABLE_TRACING together with the ft_dbg_* counters and
	 * the rm-site stamp it prints; against a library built without that flag
	 * the weak reference resolves to NULL and every VIOL() below is skipped
	 * with no diagnostic at all.  MEASURED: a 120-seed sweep against a default
	 * build fired 42 writer oracles and produced ZERO discriminator lines.
	 * The oracles still abort, so the run LOOKS like it worked -- you just get
	 * no anctomb/ancroot, which is the only thing that separates the two
	 * defects.  Refuse to start rather than hand back a wrong zero.
	 */
	if (geti("CHK",0) && !cds_ft_debug_ext_violation) {
		fprintf(stderr,
			"CHK=1 but cds_ft_debug_ext_violation is absent: this library "
			"was built WITHOUT -DFT_ENABLE_TRACING, so no violation would "
			"be classified.  Rebuild the library with it (an LTTng session "
			"is NOT required -- the classifier prints to stderr).\n");
		_exit(7);
	}

	nstable=geti("NSTABLE",200); nchurn=geti("NCHURN",200); alpha=geti("ALPHA",3);
	maxlen=geti("MAXLEN",6); nreaders=geti("READERS",6); nwriters=geti("WRITERS",1);
	secs=geti("SECS",10); ord_walk=geti("ORD",1); norem=geti("NOREM",0); chk=geti("CHK",0); nofree=geti("NOFREE",0);
	if (maxlen > MAXK) maxlen = MAXK;
	s = (uint64_t)geti("SEED",1) * 6364136223846793005ULL + 1442695040888963407ULL;
	rnd_state = s;
	signal(SIGALRM,alrm); alarm((unsigned)secs + 120);
	rcu_register_thread();
	if (cds_ft_group_attr_create(&at) != CDS_FT_STATUS_OK) _exit(5);
	if (geti("LIST",1)==0 && cds_ft_group_attr_set_ordered_list(at,false)!=CDS_FT_STATUS_OK) _exit(5);
	if (geti("LIST",1)==0) ord_walk = 0;
	/*
	 * WRITER=coarse serialises every structural writer on the one FT-wide
	 * lock.  It is a SUPPORTED attribute, unlike the coarser lock SPACINGS
	 * (exponential / root-only), which cds_ft_group_attr_set_lock_spacing
	 * REFUSES outright until every acquire site maps its members through the
	 * anchor -- those are reachable only through the CDS_FT_LOCK_SPACING env
	 * back door, and ONLY against a library built with
	 * -DFEATURE_FT_LOCK_SPACING_ENV.  Without that flag the variable is never
	 * read and the run is PER-NODE under whatever label you gave it, so the
	 * arm is echoed below rather than assumed.
	 */
	{
		const char *w = getenv("WRITER"), *sp = getenv("CDS_FT_LOCK_SPACING");

		if (w && !strcmp(w, "coarse")) {
			if (cds_ft_group_attr_set_writer_strategy(at,
					CDS_FT_WRITER_LOCK_COARSE) != CDS_FT_STATUS_OK)
				_exit(5);
		} else if (w && strcmp(w, "fine")) {
			fprintf(stderr, "WRITER must be fine or coarse\n"); _exit(5);
		}
		fprintf(stderr, "ARM writer=%s spacing_env=%s\n",
			(w && !strcmp(w, "coarse")) ? "coarse" : "fine",
			sp ? sp : "unset(per-node)");
	}
	if (cds_ft_group_create(at,&g) < 0) _exit(5);
	cds_ft_group_attr_destroy(at);
	if (cds_ft_create(g,NULL,&ft) < 0) _exit(5);

	stable = calloc((size_t)nstable,sizeof(*stable));
	churn  = calloc((size_t)nchurn,sizeof(*churn));
	mkkeys(stable,nstable,NULL,0,&s);
	if (geti("PFX",0)) {
		/*
		 * PREFIX mode: every churn key is a PROPER PREFIX of a stable
		 * key, so each insert/remove creates and destroys a PREFIX HEAD
		 * (a key that ends AT an internal node) directly on the path a
		 * reader descends to reach the stable key.
		 */
		int n = 0, tries = 0;
		while (n < nchurn && tries++ < nchurn * 20000) {
			const struct key *b = &stable[rnd(&s) % (uint64_t)nstable];
			struct key k = {{0}}; int j, dup = 0;

			if (b->len < 2) continue;
			k.len = 1 + (unsigned)(rnd(&s) % (uint64_t)(b->len - 1));
			memcpy(k.b, b->b, k.len);
			for (j = 0; j < nstable; j++)
				if (stable[j].len==k.len && !memcmp(stable[j].b,k.b,k.len)) { dup=1; break; }
			for (j = 0; !dup && j < n; j++)
				if (churn[j].len==k.len && !memcmp(churn[j].b,k.b,k.len)) { dup=1; break; }
			if (dup) continue;
			k.tag = ktag(&k); churn[n++] = k;
		}
		if (n < nchurn) { fprintf(stderr,"PFX: only %d/%d churn keys\n", n, nchurn); nchurn = n; }
		if (!nchurn) { fprintf(stderr,"PFX: no churn keys\n"); _exit(5); }
	} else {
		mkkeys(churn,nchurn,stable,nstable,&s);
	}
	rcu_read_lock();
	for (i=0;i<nstable;i++)
		if (cds_ft_insert(ft,stable[i].b,stable[i].len,&na(stable[i].tag)->node)!=CDS_FT_STATUS_OK) _exit(5);
	rcu_read_unlock();
	rcu_quiescent_state();

	rt = calloc((size_t)nreaders,sizeof(*rt)); wt = calloc((size_t)nwriters,sizeof(*wt));
	for (i=0;i<nreaders;i++) pthread_create(&rt[i],NULL,reader,(void*)(uintptr_t)i);
	for (i=0;i<nwriters;i++) pthread_create(&wt[i],NULL,writer,(void*)(uintptr_t)i);
	sleep((unsigned)secs);
	stop = 1;
	for (i=0;i<nreaders;i++) pthread_join(rt[i],NULL);
	for (i=0;i<nwriters;i++) pthread_join(wt[i],NULL);
	printf("transient=%lu hard=%lu wrongid=%lu reads=%lu walks=%lu churn_ops=%lu\n",
		transient,hardmiss,wrongid,reads,walks,churn_ops);
	return transient || hardmiss || wrongid ? 1 : 0;
}
