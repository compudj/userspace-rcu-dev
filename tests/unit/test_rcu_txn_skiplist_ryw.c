// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Composing several skiplist _prepare forms that touch the SAME skiplist within
 * one transaction, under read-your-own-writes + chained same-slot stores
 * (urcu_txn_enable_ryw).
 *
 * Without RYW this composition is unsound: each _prepare searches the COMMITTED
 * structure, blind to the transaction's other pending edits, so a later edit
 * lands on a predecessor an earlier edit has already displaced and both stores
 * name the same pred->next[L] slot.  The olds then agree (precisely because both
 * reads were stale), the engine's one-record-per-slot upgrade overwrites new_ptr,
 * and one edge is destroyed.  Test 9 pins that down as a negative control.
 *
 * With RYW the descent observes the transaction's own edits, so:
 *   - a later edit whose true predecessor is a node this txn allocated retargets
 *     its store into that PRIVATE node -- the slot collision never forms;
 *   - where the fused slot belongs to a PUBLISHED node, the two edits chain into
 *     a single record {committed_old -> final_new}.
 * Tests 3 and 4 are white-box: they inspect the descriptor before commit and
 * prove each mechanism directly, rather than inferring it from the outcome.
 *
 * The remaining tests drive the shapes that actually occur in the 3-skiplist
 * rotation benchmark: insert-then-delete and delete-then-insert on one skiplist
 * (movesper=2), adjacent deletes and adjacent inserts, a delete and an insert
 * sharing a published predecessor (movesper=3), then whole rotations, single-
 * threaded and concurrent.
 */

#define _GNU_SOURCE
#define _LGPL_SOURCE
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn-skiplist.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	12

/* Bound every retry loop so a livelock FAILS the test instead of hanging it. */
#define SPIN_LIMIT	100000

/* Concurrent rotation test. */
#ifndef NR_ROTATORS
#define NR_ROTATORS	4
#endif
#ifndef ROTATIONS
#define ROTATIONS	3000
#endif
#define KEYS_PER_THREAD	30		/* multiple of 3 */
#define KEY_STRIDE	64		/* disjoint per-thread key ranges */

struct node {
	unsigned long key;
	struct rcu_head rh;
	struct urcu_txn_skiplist_node sl;	/* last: flexible next[] */
};

static struct urcu_txn_domain g_dom;

static int node_cmp(struct urcu_txn_skiplist_node *n, void *key)
{
	unsigned long a = caa_container_of(n, struct node, sl)->key;
	unsigned long b = *(unsigned long *) key;

	return (a > b) - (a < b);
}

static unsigned long key_of(struct urcu_txn_skiplist_node *n)
{
	return caa_container_of(n, struct node, sl)->key;
}

static void node_free_cb(struct rcu_head *h)
{
	free(caa_container_of(h, struct node, rh));
}

/*
 * Clamp a requested tower height to the configured maximum, so the fixed heights
 * these tests pick stay valid when the header is built with a smaller
 * URCU_TXN_SKIPLIST_MAX_LEVELS -- notably 1, the minimal Harris/Michael list,
 * where every collision is a level-0 collision.
 */
static unsigned int tl(unsigned int want)
{
	return want < URCU_TXN_SKIPLIST_MAX_LEVELS ?
			want : URCU_TXN_SKIPLIST_MAX_LEVELS - 1;
}

static struct node *node_alloc(unsigned long key, unsigned int toplevel)
{
	struct node *e;

	toplevel = tl(toplevel);
	e = (struct node *) malloc(sizeof(*e)
			+ (toplevel + 1) * sizeof(struct urcu_txn_skiplist_node *));
	if (!e)
		abort();
	e->key = key;
	urcu_txn_skiplist_node_init(&e->sl, toplevel);
	return e;
}

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

/* --------------------------------------------------------------------- */
/* Structural checks.                                                     */
/* --------------------------------------------------------------------- */

/* Every level-L link lands on a level-0-downstream node tall enough for it. */
static void fsck(struct urcu_txn_skiplist *sl)
{
	struct urcu_txn_skiplist_node *first;

	for (first = sl->head; first != NULL;
			first = urcu_txn_skiplist_next_rcu(first, 0)) {
		unsigned int i;
		struct urcu_txn_skiplist_node *s =
				urcu_txn_skiplist_next_rcu(first, 0);

		for (i = 1; i <= first->toplevel; i++) {
			struct urcu_txn_skiplist_node *tgt =
					urcu_txn_skiplist_next_rcu(first, i);
			while (s != tgt) {
				assert(s != NULL);
				s = urcu_txn_skiplist_next_rcu(s, 0);
			}
			assert(s == NULL || s->toplevel >= i);
		}
	}
}

/*
 * No node REACHABLE at level 0 may carry a tombstone on any of its levels.  This
 * is what catches the delete-first corruption: there the victim ends up MARK-ed
 * yet still linked, so the delete silently did not happen and the node is
 * reclaimed under live readers.  fsck() cannot see it -- it resolves marks.
 */
static void assert_no_live_marks(struct urcu_txn_skiplist *sl)
{
	struct urcu_txn_skiplist_node *n;

	for (n = urcu_txn_skiplist_next_rcu(sl->head, 0); n != NULL;
			n = urcu_txn_skiplist_next_rcu(n, 0)) {
		unsigned int i;

		for (i = 0; i <= n->toplevel; i++) {
			void *v = urcu_mcas_read((void **) &n->next[i],
					URCU_TXN_SKIPLIST_TAG);

			assert(!urcu_txn_skiplist_is_marked(v));
		}
	}
}

/* Collect the level-0 keys; asserts strictly ascending. Returns the count. */
static int collect(struct urcu_txn_skiplist *sl, unsigned long *out, int max)
{
	struct urcu_txn_skiplist_node *n;
	unsigned long prev = 0;
	int have_prev = 0, count = 0;

	for (n = urcu_txn_skiplist_next_rcu(sl->head, 0); n != NULL;
			n = urcu_txn_skiplist_next_rcu(n, 0)) {
		unsigned long k = key_of(n);

		if (have_prev)
			assert(k > prev);
		prev = k;
		have_prev = 1;
		assert(count < max);
		out[count++] = k;
	}
	return count;
}

static int keys_equal(struct urcu_txn_skiplist *sl,
		const unsigned long *want, int nwant)
{
	unsigned long got[64];
	int n = collect(sl, got, 64), i;

	fsck(sl);
	assert_no_live_marks(sl);
	if (n != nwant)
		return 0;
	for (i = 0; i < n; i++) {
		if (got[i] != want[i])
			return 0;
	}
	return 1;
}

/* --------------------------------------------------------------------- */
/* Batch driver: N _prepare forms composed into ONE transaction.          */
/* --------------------------------------------------------------------- */

enum op_kind { OP_INS, OP_DEL };

struct op {
	enum op_kind kind;
	struct urcu_txn_skiplist *sl;
	unsigned long key;			/* DEL: the key to remove */
	struct node *newn;			/* INS: caller-allocated node */
	struct urcu_txn_skiplist_node *removed;	/* DEL: set on success */
};

/*
 * Run @ops as one transaction.  @ryw opts the handle into read-your-own-writes.
 * Returns 0, a negative errno from a _prepare, or -ETIMEDOUT if the attempt loop
 * exceeded SPIN_LIMIT (a livelock -- which is exactly what composing on one
 * skiplist without RYW deterministically produces).
 */
static int batch_commit(struct op *ops, int n, int ryw)
{
	struct urcu_mcas_txn txn;
	long spins = 0;
	int i, prep;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, &g_dom);
	if (ryw)
		urcu_txn_enable_ryw(&txn);
	for (;;) {
		int retry = 0, err = 0;

		if (++spins > SPIN_LIMIT)
			return -ETIMEDOUT;
		urcu_txn_begin(&txn);
		for (i = 0; i < n; i++) {
			if (ops[i].kind == OP_INS)
				prep = urcu_txn_skiplist_insert_prepare(&txn,
						ops[i].sl, &ops[i].newn->sl,
						&ops[i].newn->key);
			else
				prep = urcu_txn_skiplist_del_prepare(&txn,
						ops[i].sl, &ops[i].key,
						&ops[i].removed);
			if (prep == -EAGAIN) { retry = 1; break; }
			if (prep < 0) { err = prep; break; }
		}
		if (err) {
			urcu_txn_end(&txn);
			return err;
		}
		if (retry) {
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_OK)
			return 0;
		if (st == URCU_TXN_STATUS_ABORT)
			continue;
		return -ENOMEM;
	}
}

/* Single-op helpers built on the same driver (used to seed a skiplist). */
static void seed_insert(struct urcu_txn_skiplist *sl, unsigned long key,
		unsigned int toplevel)
{
	struct op op = { .kind = OP_INS, .sl = sl, .newn = node_alloc(key, toplevel) };

	if (batch_commit(&op, 1, 0))
		abort();
}

static void sl_seed(struct urcu_txn_skiplist *sl, const unsigned long *keys,
		int n, unsigned int toplevel)
{
	int i;

	if (urcu_txn_skiplist_init(sl, node_cmp))
		abort();
	for (i = 0; i < n; i++)
		seed_insert(sl, keys[i], toplevel);
}

/* Drain a skiplist synchronously (single-threaded, quiescent). */
static void sl_drain(struct urcu_txn_skiplist *sl)
{
	struct urcu_txn_skiplist_node *n;

	while ((n = urcu_txn_skiplist_next_rcu(sl->head, 0)) != NULL) {
		struct node *e = caa_container_of(n, struct node, sl);
		struct urcu_txn_skiplist_node *rem = NULL;

		if (urcu_txn_skiplist_del_rcu(sl, &e->key, &g_dom, &rem) != 1)
			abort();		/* 1 == this call removed it */
		free(e);
	}
	urcu_txn_skiplist_destroy(sl);
}

/* --------------------------------------------------------------------- */
/* 1-2. RYW: insert-then-delete on ONE skiplist (the movesper=2 shape).   */
/* --------------------------------------------------------------------- */

static void test_insert_then_delete(void)
{
	static const unsigned long seed[] = { 1, 4, 7, 10, 13 };
	static const unsigned long want[] = { 0, 4, 7, 10, 13 };
	struct urcu_txn_skiplist sl;
	struct op ops[2];

	sl_seed(&sl, seed, 5, 2);		/* towers taller than the new node */

	ops[0] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(0, 3) };
	ops[1] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 1 };

	ok(batch_commit(ops, 2, 1) == 0, "ryw: insert(0)+del(1) on one skiplist commits");
	ok(keys_equal(&sl, want, 5),
			"ryw: insert-then-delete leaves {0,4,7,10,13}, tower intact");

	free(caa_container_of(ops[1].removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 3. White-box: delete-first fuses into ONE chained record.              */
/* --------------------------------------------------------------------- */

static void test_whitebox_chain(void)
{
	static const unsigned long seed[] = { 1, 4, 7 };
	struct urcu_txn_skiplist sl;
	struct urcu_mcas_txn txn;
	struct urcu_mcas_record *r;
	struct node *new0, *node1;
	unsigned long k1 = 1;
	struct urcu_txn_skiplist_node *removed = NULL;
	int pass;

	sl_seed(&sl, seed, 3, 0);		/* flat: every tower is level 0 */
	node1 = caa_container_of(urcu_txn_skiplist_next_rcu(sl.head, 0),
			struct node, sl);
	new0 = node_alloc(0, 0);

	urcu_txn_init(&txn, &g_dom);
	urcu_txn_enable_ryw(&txn);
	urcu_txn_begin(&txn);
	/* Delete first, then insert immediately before the deleted node. */
	if (urcu_txn_skiplist_del_prepare(&txn, &sl, &k1, &removed))
		abort();
	if (urcu_txn_skiplist_insert_prepare(&txn, &sl, &new0->sl, &new0->key))
		abort();

	/*
	 * Both edits legitimately rewrite head->next[0]: the delete unlinks
	 * node(1), the insert splices new0 in its place.  They must have FUSED
	 * into one record whose old is the committed value (node1) and whose new
	 * is the final value (new0) -- not two records, and not a lost edge.
	 */
	r = urcu_mcas_find(txn.mcas, (void **) &sl.head->next[0]);
	pass = r != NULL &&
		r->old_ptr == (void *) &node1->sl &&
		r->new_ptr == (void *) &new0->sl;
	ok(pass, "ryw whitebox: del+insert on one published slot chain to {node(1) -> new(0)}");

	if (urcu_txn_commit(&txn) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&txn);

	free(caa_container_of(removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 4. White-box: insert-first RETARGETS the delete onto the private node. */
/* --------------------------------------------------------------------- */

static void test_whitebox_retarget(void)
{
	static const unsigned long seed[] = { 1, 4, 7 };
	struct urcu_txn_skiplist sl;
	struct urcu_mcas_txn txn;
	struct urcu_mcas_record *rhead, *rnew;
	struct node *new0, *node1, *node4;
	unsigned long k1 = 1;
	struct urcu_txn_skiplist_node *removed = NULL;
	int pass;

	sl_seed(&sl, seed, 3, 0);
	node1 = caa_container_of(urcu_txn_skiplist_next_rcu(sl.head, 0),
			struct node, sl);
	node4 = caa_container_of(urcu_txn_skiplist_next_rcu(&node1->sl, 0),
			struct node, sl);
	new0 = node_alloc(0, 0);

	urcu_txn_init(&txn, &g_dom);
	urcu_txn_enable_ryw(&txn);
	urcu_txn_begin(&txn);
	if (urcu_txn_skiplist_insert_prepare(&txn, &sl, &new0->sl, &new0->key))
		abort();
	if (urcu_txn_skiplist_del_prepare(&txn, &sl, &k1, &removed))
		abort();

	/*
	 * The delete's RYW descent sees new0 already spliced in, so key 1's
	 * predecessor is new0 -- a node this txn allocated and has not published.
	 * Its unlink therefore retargets to &new0->next[0]; head->next[0] keeps
	 * ONLY the insert's edge.  That is the collision dissolving rather than
	 * being reconciled.
	 */
	rhead = urcu_mcas_find(txn.mcas, (void **) &sl.head->next[0]);
	rnew = urcu_mcas_find(txn.mcas, (void **) &new0->sl.next[0]);
	pass = rhead != NULL && rhead->new_ptr == (void *) &new0->sl &&
		rnew != NULL && rnew->new_ptr == (void *) &node4->sl;
	ok(pass, "ryw whitebox: insert-first retargets the unlink onto the private node");

	if (urcu_txn_commit(&txn) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&txn);

	free(caa_container_of(removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 5. RYW: delete-then-insert (mirrored order).                           */
/* --------------------------------------------------------------------- */

static void test_delete_then_insert(void)
{
	static const unsigned long seed[] = { 1, 4, 7, 10, 13 };
	static const unsigned long want[] = { 0, 4, 7, 10, 13 };
	struct urcu_txn_skiplist sl;
	struct op ops[2];

	sl_seed(&sl, seed, 5, 2);

	ops[0] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 1 };
	ops[1] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(0, 3) };

	if (batch_commit(ops, 2, 1))
		abort();
	ok(keys_equal(&sl, want, 5),
			"ryw: delete-then-insert leaves {0,4,7,10,13}, victim not left linked");

	free(caa_container_of(ops[0].removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 6-7. RYW: adjacent deletes, adjacent inserts.                          */
/* --------------------------------------------------------------------- */

static void test_adjacent_deletes(void)
{
	static const unsigned long seed[] = { 1, 4, 7, 10 };
	static const unsigned long want[] = { 1, 10 };
	struct urcu_txn_skiplist sl;
	struct op ops[2];

	sl_seed(&sl, seed, 4, 1);

	ops[0] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 4 };
	ops[1] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 7 };

	if (batch_commit(ops, 2, 1))
		abort();
	ok(keys_equal(&sl, want, 2),
			"ryw: two adjacent deletes in one txn chain on the shared pred slot");

	free(caa_container_of(ops[0].removed, struct node, sl));
	free(caa_container_of(ops[1].removed, struct node, sl));
	sl_drain(&sl);
}

static void test_adjacent_inserts(void)
{
	static const unsigned long seed[] = { 1, 4 };
	static const unsigned long want[] = { 1, 2, 3, 4 };
	struct urcu_txn_skiplist sl;
	struct op ops[2];

	sl_seed(&sl, seed, 2, 2);

	ops[0] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(2, 1) };
	ops[1] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(3, 0) };

	if (batch_commit(ops, 2, 1))
		abort();
	ok(keys_equal(&sl, want, 4),
			"ryw: two adjacent inserts in one txn both land, in order");
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 8. RYW: del(j) + ins(j+2) sharing a PUBLISHED pred (movesper=3 shape).  */
/* --------------------------------------------------------------------- */

static void test_del_plus_insert_published_pred(void)
{
	static const unsigned long seed[] = { 0, 3, 6 };
	static const unsigned long want[] = { 2, 3, 6 };
	struct urcu_txn_skiplist sl;
	struct op ops[2];

	sl_seed(&sl, seed, 3, 1);

	ops[0] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 0 };
	ops[1] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(2, 2) };

	if (batch_commit(ops, 2, 1))
		abort();
	ok(keys_equal(&sl, want, 3),
			"ryw: del(0)+ins(2) sharing the head slot chain to {2,3,6}");

	free(caa_container_of(ops[0].removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 9. Negative control: WITHOUT ryw the same batch silently loses a key.   */
/* --------------------------------------------------------------------- */

static void test_negative_no_ryw(void)
{
	static const unsigned long seed[] = { 1, 4, 7, 10, 13 };
	static const unsigned long lost[] = { 4, 7, 10, 13 };	/* key 0 destroyed */
	struct urcu_txn_skiplist sl;
	struct op ops[2];
	int rc, silent_loss;

	/*
	 * Flat towers, and a level-0 new node: the coalesce then destroys exactly
	 * the insert's single edge, leaving a consistent-but-wrong structure we
	 * can safely inspect.  (With a taller new node the same coalesce tears the
	 * tower and leaves next[0] aimed at a reclaimed node -- correct to expect,
	 * unsafe to walk, so this control does not provoke it.)
	 */
	sl_seed(&sl, seed, 5, 0);

	ops[0] = (struct op){ .kind = OP_INS, .sl = &sl, .newn = node_alloc(0, 0) };
	ops[1] = (struct op){ .kind = OP_DEL, .sl = &sl, .key = 1 };

	rc = batch_commit(ops, 2, 0);		/* RYW OFF */
	silent_loss = (rc == 0) && keys_equal(&sl, lost, 4);
	ok(silent_loss,
			"no-ryw control: the same batch commits OK yet silently drops key 0");

	free(ops[0].newn);			/* never linked: the destroyed edge */
	free(caa_container_of(ops[1].removed, struct node, sl));
	sl_drain(&sl);
}

/* --------------------------------------------------------------------- */
/* 10-11. Whole 3-skiplist rotations, batched (the bench workload).        */
/* --------------------------------------------------------------------- */

/*
 * K keys, key j starting in sl[j%3]; a rotation moves every key t -> (t+1)%3 in
 * chunks of @movesper composed moves.  Consecutive keys in a chunk always share
 * a skiplist (key j's destination is key j+1's source) and are adjacent there --
 * the exact aliasing this scheme must handle.
 */
static int rotate(struct urcu_txn_skiplist *sls, struct node **cur, int *curtab,
		int K, int movesper, int rotations, unsigned int *seedp)
{
	struct op ops[8];
	int r, j;

	assert(movesper <= 4);
	for (r = 0; r < rotations; r++) {
		for (j = 0; j < K; ) {
			int n = 0, base = j, k;

			while (n < movesper && j < K) {
				unsigned int lvl;

				*seedp = xs(*seedp);
				lvl = urcu_txn_skiplist_random_level(*seedp);
				ops[2 * n] = (struct op){ .kind = OP_DEL,
					.sl = &sls[curtab[j]], .key = cur[j]->key };
				ops[2 * n + 1] = (struct op){ .kind = OP_INS,
					.sl = &sls[(curtab[j] + 1) % 3],
					.newn = node_alloc(cur[j]->key, lvl) };
				n++; j++;
			}
			if (batch_commit(ops, 2 * n, 1))
				return -1;
			for (k = 0; k < n; k++) {
				int idx = base + k;

				call_rcu(&caa_container_of(ops[2 * k].removed,
						struct node, sl)->rh, node_free_cb);
				cur[idx] = ops[2 * k + 1].newn;
				curtab[idx] = (curtab[idx] + 1) % 3;
			}
		}
		rcu_quiescent_state();
	}
	return 0;
}

/* Every key present exactly once across the three skiplists, in curtab[j]. */
static int conserved(struct urcu_txn_skiplist *sls, struct node **cur,
		int *curtab, int K)
{
	int t, j, total = 0;

	for (t = 0; t < 3; t++) {
		unsigned long got[256];

		total += collect(&sls[t], got, 256);
	}
	if (total != K)
		return 0;
	for (j = 0; j < K; j++) {
		if (!urcu_txn_skiplist_lookup_rcu(&sls[curtab[j]], &cur[j]->key))
			return 0;
	}
	return 1;
}

static void test_rotation(int movesper)
{
	struct urcu_txn_skiplist sls[3];
	struct node *cur[30];
	int curtab[30];
	unsigned int seed = 0x1234u + (unsigned int) movesper;
	const int K = 30;
	int t, j, rc;

	for (t = 0; t < 3; t++) {
		if (urcu_txn_skiplist_init(&sls[t], node_cmp))
			abort();
	}
	for (j = 0; j < K; j++) {
		curtab[j] = j % 3;
		cur[j] = node_alloc((unsigned long) j, 1);
		if (urcu_txn_skiplist_add_rcu(&sls[curtab[j]], &cur[j]->sl,
					&cur[j]->key, &g_dom))
			abort();
	}

	rc = rotate(sls, cur, curtab, K, movesper, 200, &seed);

	for (t = 0; t < 3; t++) {
		fsck(&sls[t]);
		assert_no_live_marks(&sls[t]);
	}
	ok(rc == 0 && conserved(sls, cur, curtab, K),
			"ryw: 200 rotations at movesper=%d conserve all %d keys",
			movesper, K);

	for (t = 0; t < 3; t++)
		sl_drain(&sls[t]);
}

/* --------------------------------------------------------------------- */
/* 12. Concurrent batched rotations over disjoint key ranges.             */
/* --------------------------------------------------------------------- */

static struct urcu_txn_skiplist g_sls[3];

struct rot_arg {
	int id;
	int rc;
};

static void *rotator(void *a)
{
	struct rot_arg *me = (struct rot_arg *) a;
	struct node *cur[KEYS_PER_THREAD];
	int curtab[KEYS_PER_THREAD];
	unsigned int seed = 0xbeef ^ (unsigned int) (me->id + 1);
	unsigned long first = (unsigned long) me->id * KEY_STRIDE;
	int j;

	rcu_register_thread();
	for (j = 0; j < KEYS_PER_THREAD; j++) {
		curtab[j] = j % 3;
		cur[j] = node_alloc(first + (unsigned long) j, 1);
		if (urcu_txn_skiplist_add_rcu(&g_sls[curtab[j]], &cur[j]->sl,
					&cur[j]->key, &g_dom))
			abort();
	}
	me->rc = rotate(g_sls, cur, curtab, KEYS_PER_THREAD, 3, ROTATIONS, &seed);

	/* Drain this thread's keys so the final audit sees an empty structure. */
	for (j = 0; j < KEYS_PER_THREAD && !me->rc; j++) {
		struct urcu_txn_skiplist_node *rem = NULL;

		if (urcu_txn_skiplist_del_rcu(&g_sls[curtab[j]], &cur[j]->key,
					&g_dom, &rem) != 1)
			abort();		/* 1 == this call removed it */
		call_rcu(&caa_container_of(rem, struct node, sl)->rh, node_free_cb);
	}
	rcu_quiescent_state();
	rcu_unregister_thread();
	return NULL;
}

static void test_concurrent_rotation(void)
{
	pthread_t th[NR_ROTATORS];
	struct rot_arg args[NR_ROTATORS];
	int i, bad = 0, t;

	for (t = 0; t < 3; t++) {
		if (urcu_txn_skiplist_init(&g_sls[t], node_cmp))
			abort();
	}
	for (i = 0; i < NR_ROTATORS; i++) {
		args[i].id = i;
		args[i].rc = 0;
		if (pthread_create(&th[i], NULL, rotator, &args[i]))
			abort();
	}
	rcu_thread_offline();
	for (i = 0; i < NR_ROTATORS; i++)
		pthread_join(th[i], NULL);
	rcu_thread_online();

	for (i = 0; i < NR_ROTATORS; i++)
		bad += (args[i].rc != 0);
	for (t = 0; t < 3; t++) {
		fsck(&g_sls[t]);
		assert_no_live_marks(&g_sls[t]);
		bad += (urcu_txn_skiplist_next_rcu(g_sls[t].head, 0) != NULL);
	}
	ok(bad == 0,
			"ryw: %d threads x %d batched rotations (movesper=3) stay consistent",
			NR_ROTATORS, ROTATIONS);

	for (t = 0; t < 3; t++)
		urcu_txn_skiplist_destroy(&g_sls[t]);
}

int main(void)
{
	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_dom);

	test_insert_then_delete();		/* 1, 2 */
	test_whitebox_chain();			/* 3 */
	test_whitebox_retarget();		/* 4 */
	test_delete_then_insert();		/* 5 */
	test_adjacent_deletes();		/* 6 */
	test_adjacent_inserts();		/* 7 */
	test_del_plus_insert_published_pred();	/* 8 */
	test_negative_no_ryw();			/* 9 */
	test_rotation(2);			/* 10 */
	test_rotation(3);			/* 11 */
	test_concurrent_rotation();		/* 12 */

	/*
	 * Drain the deferred reclaim queue before exiting, so a leak report is
	 * about the test rather than about callbacks that simply had not run yet.
	 * Go offline across the wait: a registered QSBR thread that blocks without
	 * reporting a quiescent state stalls the grace period it is waiting on.
	 */
	rcu_thread_offline();
	rcu_barrier();
	rcu_thread_online();

	rcu_unregister_thread();
	return exit_status();
}
