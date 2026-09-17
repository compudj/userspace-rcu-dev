/*
 * ft-anchor-move-rig.c -- doc/design/ft-lockset-inventory.md §5, settled by test.
 *
 * QUESTION: does holding the OLD anchor during a restructure exclude an op
 * that PLANNED before the restructure and ACQUIRES the old anchor after it?
 * ANSWER (measured): it did not; the acquire now validates the COVERER the
 * anchor was derived from -- see the doc's table.
 *
 * Shape, exponential spacing (levels 0,1,2,4,8,16,...):
 *   root [0,1) --'a'--> cn [1,9) "bcdefghi" --> A internal @9 --'J'--> X internal @10
 *   X has children '1'..'8', each an external leaf DIRECTLY in X's slot (key
 *   length 11): no orphan chain, so the victim holds nothing before its hoist.
 *   anchor(X): depth 10, L=8, first boundary >= 8 is A @9  ==> A.
 * Split: insert "abcdefghQzz" (diverges at byte 8) ==> a new boundary B @8,
 *   anchor(X) becomes B.
 *
 * VICTIM P (thread): remove "abcdefghiJ8" -- the in-place leaf delete, whose
 *   hoist takes anchor(X) from its descent.  The library's interleave hook
 *   (FT_IL_LEAF_HOIST_PRE / _POST) parks P before and after that acquire.
 * MAIN: while P is at PRE, (mode split) insert the splitting key.  Release P.
 *   While P is at POST (holding what it took), start PEER Q and wait up to
 *   RIG_WAIT (default 2) seconds for it:
 *     default          insert "abcdefghiJ0" -- ☠ a FALSE negative: growing X
 *                      makes X's parent A a coarsened member, and A's own word
 *                      carries P's lock, so Q waits on A as a member.
 *     RIG_PEER=delete  remove "abcdefghiJ7" -- lock set {X} alone.
 *   Keys are chosen so no insert is an ORDERED-LIST neighbour of the victim:
 *   an insert beside a cell the parked victim is deleting spins on that
 *   half-deleted cell, which hangs the rig, not the trie.
 *
 * READING:
 *   nosplit (CONTROL): P and Q agree on A, so Q must NOT complete while P holds
 *                      A.  Q completing here means the rig cannot see exclusion.
 *   split:             Q completing while P holds its lock means P holds a word
 *                      that no longer excludes X's writers: the hole is real.
 *   splitheld:         the OTHER order -- the split runs while P already HOLDS
 *                      A.  Neither the split nor Q may complete until P lets go.
 *   splitlate:         as split, but the split is parked INSIDE its commit's
 *                      settle, before the lock words it hands back: A is
 *                      takeable while cn's committed tombstone still reads as a
 *                      proxy.  Reads like split.
 *
 * Build against a tree configured with -DFT_DEBUG_INTERLEAVE (add
 * -DFT_DEBUG_STRUCT_ANCHOR -DFT_DEBUG_TXN_KIND for probe A's mismatch line):
 *   gcc -O1 -g -I<top>/include -I<build>/include ft-anchor-move-rig.c -o rig \
 *     -L<build>/src/.libs -lurcu-memb -lurcu-cds -lurcu-common -lpthread \
 *     -Wl,-rpath,<build>/src/.libs
 *   ./rig nosplit; ./rig split; RIG_PEER=delete ./rig nosplit;
 *   RIG_PEER=delete ./rig split; RIG_PEER=delete ./rig splitheld;
 *   RIG_PEER=delete ./rig splitlate
 */
#define _GNU_SOURCE
#include <urcu/urcu-memb.h>
#include <urcu/fractal-trie.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

extern void (*cds_ft_dbg_interleave_hook)(int point);
#define FT_IL_LEAF_HOIST_PRE	1
#define FT_IL_LEAF_HOIST_POST	2
#define FT_IL_SETTLE_LOCKS_LAST	3

struct tnode {
	struct cds_ft_node node;
};

static struct cds_ft *ft;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static int stage;
static __thread int armed;
static int pre_hits, post_hits;

/* splitlate: the splitter parks in its commit's settle, before its lock words. */
static __thread int late_armed;
static int late_paused, late_release;

static void hook(int point)
{
	if (point == FT_IL_SETTLE_LOCKS_LAST) {
		struct timespec dl;

		if (!late_armed)
			return;
		late_armed = 0;
		/*
		 * Bounded: a victim whose acquire is refused re-plans, and the
		 * re-plan may need this settle to finish before it can hold.
		 */
		clock_gettime(CLOCK_REALTIME, &dl);
		dl.tv_sec += 2;
		pthread_mutex_lock(&mu);
		late_paused = 1;
		pthread_cond_broadcast(&cv);
		while (!late_release)
			if (pthread_cond_timedwait(&cv, &mu, &dl) == ETIMEDOUT)
				break;
		pthread_mutex_unlock(&mu);
		return;
	}
	if (!armed)
		return;
	pthread_mutex_lock(&mu);
	if (point == FT_IL_LEAF_HOIST_PRE) {
		pre_hits++;
		if (stage == 0) {
			stage = 1;
			pthread_cond_broadcast(&cv);
			while (stage < 2)
				pthread_cond_wait(&cv, &mu);
		}
	} else if (point == FT_IL_LEAF_HOIST_POST) {
		post_hits++;
		if (stage == 2) {
			stage = 3;
			pthread_cond_broadcast(&cv);
			while (stage < 4)
				pthread_cond_wait(&cv, &mu);
		}
	}
	pthread_mutex_unlock(&mu);
}

static void wait_stage(int s)
{
	pthread_mutex_lock(&mu);
	while (stage < s)
		pthread_cond_wait(&cv, &mu);
	pthread_mutex_unlock(&mu);
}

static void set_stage(int s)
{
	pthread_mutex_lock(&mu);
	stage = s;
	pthread_cond_broadcast(&cv);
	pthread_mutex_unlock(&mu);
}

static struct tnode *last_inserted;

static enum cds_ft_status insert_key(const char *k)
{
	struct tnode *n = calloc(1, sizeof(*n));
	enum cds_ft_status s;

	cds_ft_node_init(&n->node);
	urcu_memb_read_lock();
	s = cds_ft_insert(ft, (const uint8_t *) k, strlen(k), &n->node);
	urcu_memb_read_unlock();
	last_inserted = n;
	return s;
}

static int lookup_key(const char *k)
{
	struct cds_ft_iter *it;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &it) != CDS_FT_STATUS_OK)
		abort();
	urcu_memb_read_lock();
	cds_ft_iter_set_key(it, (const uint8_t *) k, strlen(k));
	s = cds_ft_lookup(ft, it);
	urcu_memb_read_unlock();
	cds_ft_iter_destroy(it);
	return s == CDS_FT_STATUS_OK;
}

static enum cds_ft_status victim_status = -1;
static struct tnode *victim_node;

static void *victim(void *arg)
{
	const char *k = "abcdefghiJ8";
	struct cds_ft_iter *it;

	(void) arg;
	urcu_memb_register_thread();
	if (cds_ft_iter_create(ft, &it) != CDS_FT_STATUS_OK)
		abort();
	armed = 1;
	urcu_memb_read_lock();
	cds_ft_iter_set_key(it, (const uint8_t *) k, strlen(k));
	if (cds_ft_lookup(ft, it) == CDS_FT_STATUS_OK)
		victim_status = cds_ft_remove(ft, it, &victim_node->node);
	urcu_memb_read_unlock();
	armed = 0;
	cds_ft_iter_destroy(it);
	urcu_memb_unregister_thread();
	return NULL;
}

static int peer_done;
static enum cds_ft_status peer_status = -1;
static struct tnode *peer_del_node;	/* RIG_PEER=delete: "abcdefghiJ7" */

/*
 * RIG_PEER=delete: the peer is ANOTHER in-place leaf delete under X, whose lock
 * set is {X} alone.  The insert peer's set also names X's parent A (its
 * recompaction's P, its in-place arm's grandparent), and a coarsened member's
 * own word is sampled -- so it waits on the victim's lock on A's OWN word
 * whatever anchor it computes, which is not the question.
 */
static void *peer(void *arg)
{
	(void) arg;
	urcu_memb_register_thread();
	if (peer_del_node) {
		struct cds_ft_iter *it;

		if (cds_ft_iter_create(ft, &it) != CDS_FT_STATUS_OK)
			abort();
		urcu_memb_read_lock();
		cds_ft_iter_set_key(it, (const uint8_t *) "abcdefghiJ7", 11);
		if (cds_ft_lookup(ft, it) == CDS_FT_STATUS_OK)
			peer_status = cds_ft_remove(ft, it,
				&peer_del_node->node);
		urcu_memb_read_unlock();
		cds_ft_iter_destroy(it);
	} else
		peer_status = insert_key("abcdefghiJ0");
	pthread_mutex_lock(&mu);
	peer_done = 1;
	pthread_cond_broadcast(&cv);
	pthread_mutex_unlock(&mu);
	urcu_memb_unregister_thread();
	return NULL;
}

static int split_done;

static void *splitter(void *arg)
{
	late_armed = arg != NULL;
	urcu_memb_register_thread();
	if (insert_key("abcdefghQzz") != CDS_FT_STATUS_OK)
		fprintf(stderr, "split insert failed\n");
	pthread_mutex_lock(&mu);
	split_done = 1;
	pthread_cond_broadcast(&cv);
	pthread_mutex_unlock(&mu);
	urcu_memb_unregister_thread();
	return NULL;
}

/* Wait up to @secs for *@flag; returns its value. */
static int wait_flag(int *flag, int secs)
{
	struct timespec dl;
	int v;

	clock_gettime(CLOCK_REALTIME, &dl);
	dl.tv_sec += secs;
	pthread_mutex_lock(&mu);
	while (!*flag)
		if (pthread_cond_timedwait(&cv, &mu, &dl) == ETIMEDOUT)
			break;
	v = *flag;
	pthread_mutex_unlock(&mu);
	return v;
}

int main(int argc, char **argv)
{
	int split = argc > 1 && !strcmp(argv[1], "split");
	/*
	 * splitheld: the OTHER order -- the victim already HOLDS its anchor
	 * when the split runs.  The split must not complete while it is held
	 * (it re-homes A under a guard on A's clean state word).
	 */
	int splitheld = argc > 1 && !strcmp(argv[1], "splitheld");
	/*
	 * splitlate: the split COMMITS before the victim's acquire but is parked
	 * in its settle after its guard on A (plain, pass 2) and before its lock
	 * words (last) -- so A is takeable while cn's committed tombstone is still
	 * a proxy.  Reads like split.
	 */
	int splitlate = argc > 1 && !strcmp(argv[1], "splitlate");
	int split_during_hold = -1;
	pthread_t ts;
	struct cds_ft_group *group;
	pthread_t tp, tq;
	struct timespec dl;
	int q_during_hold;
	char k[16];
	int i;

	alarm(30);	/* a grace period waiting on the paused victim would hang */
	setenv("CDS_FT_LOCK_SPACING", "exponential", 1);
	urcu_memb_register_thread();
	if (cds_ft_group_create_flavor(NULL, &group, &urcu_memb_flavor) !=
			CDS_FT_STATUS_OK || cds_ft_create(group, NULL, &ft) < 0) {
		fprintf(stderr, "create failed\n");
		return 2;
	}
	cds_ft_dbg_interleave_hook = hook;

	insert_key("Zzzzzzzzzzz");
	insert_key("abcdefghiK1");
	for (i = 1; i <= 8; i++) {
		snprintf(k, sizeof(k), "abcdefghiJ%d", i);
		insert_key(k);
		if (i == 8)
			victim_node = last_inserted;
		if (i == 7 && getenv("RIG_PEER") &&
				!strcmp(getenv("RIG_PEER"), "delete"))
			peer_del_node = last_inserted;
	}

	pthread_create(&tp, NULL, victim, NULL);
	wait_stage(1);			/* P planned, before its acquire */
	if (split && insert_key("abcdefghQzz") != CDS_FT_STATUS_OK)
		fprintf(stderr, "split insert failed\n");
	if (splitlate) {
		pthread_create(&ts, NULL, splitter, (void *) 1);
		if (!wait_flag(&late_paused, 5))
			fprintf(stderr, "splitlate: the split never reached its settle\n");
	}
	set_stage(2);
	wait_stage(3);			/* P holds its lock */
	if (splitlate) {
		pthread_mutex_lock(&mu);
		late_release = 1;
		pthread_cond_broadcast(&cv);
		pthread_mutex_unlock(&mu);
		pthread_join(ts, NULL);	/* settled before the peer plans */
	}
	if (splitheld) {
		pthread_create(&ts, NULL, splitter, NULL);
		split_during_hold = wait_flag(&split_done,
			getenv("RIG_WAIT") ? atoi(getenv("RIG_WAIT")) : 2);
	}
	pthread_create(&tq, NULL, peer, NULL);
	clock_gettime(CLOCK_REALTIME, &dl);
	dl.tv_sec += getenv("RIG_WAIT") ? atoi(getenv("RIG_WAIT")) : 2;
	pthread_mutex_lock(&mu);
	while (!peer_done)
		if (pthread_cond_timedwait(&cv, &mu, &dl) == ETIMEDOUT)
			break;
	q_during_hold = peer_done;
	pthread_mutex_unlock(&mu);
	set_stage(4);			/* release P */
	pthread_join(tp, NULL);
	pthread_join(tq, NULL);
	if (splitheld)
		pthread_join(ts, NULL);

	printf("mode=%s pre_hits=%d post_hits=%d victim=%d peer=%d "
		"PEER_COMPLETED_WHILE_VICTIM_HELD=%d | J8=%d J0=%d J7=%d Q=%d K1=%d peer=%s\n",
		splitheld ? "splitheld" : splitlate ? "splitlate" :
			split ? "split" : "nosplit", pre_hits, post_hits,
		(int) victim_status, (int) peer_status, q_during_hold,
		lookup_key("abcdefghiJ8"), lookup_key("abcdefghiJ0"),
		lookup_key("abcdefghiJ7"),
		lookup_key("abcdefghQzz"), lookup_key("abcdefghiK1"),
		peer_del_node ? "delete J7" : "insert J0");
	if (!pre_hits || !post_hits)
		printf("RIG INVALID: the victim never reached the hoist\n");
	else if (splitheld)
		printf("SPLITHELD: split %s while the victim held its anchor; "
			"peer %s\n", split_during_hold ?
				"COMPLETED -- the split does not wait on the old anchor" :
				"waited",
			q_during_hold ? "COMPLETED while held -- HOLE" :
				"waited");
	else if (!split && !splitlate)
		printf("CONTROL %s\n", q_during_hold ?
			"FAILED: the peer was not excluded by an AGREEING anchor" :
			"OK: the peer waited on the agreeing anchor");
	else
		printf("QUESTION: %s\n", q_during_hold ?
			"HOLE -- a pre-split planner holding the OLD anchor did not exclude a post-split writer of the same node" :
			"no hole observed -- the post-split writer waited");
	return 0;
}
