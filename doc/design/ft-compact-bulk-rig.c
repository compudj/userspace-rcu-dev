/*
 * compact_bulk_rig.c -- compaction running CONCURRENTLY with bulk ops on a FINE
 * trie, no application lock: the library's own exclusion must hold.
 *
 *   thread COMPACT: cds_ft_compact_begin / step(16) until DONE / end, forever
 *   thread BULK:    detach "z"+b (b cycling), graft it straight back
 *   thread READ:    RCU lookups of the "p" keys, which nothing removes
 *
 * At the end: every "p" and "z" key present, cds_ft_verify OK.  A reader miss
 * on a "p" key during the run is reported immediately (key loss is permanent).
 *
 * Env: RIG_SECS (default 20), CDS_FT_LOCK_SPACING (library env knob).
 *
 * doc/design/ft-lockset-inventory.md §9.  ☠ While bulk ops still take DLM
 * locks this rig CANNOT fail on a missing compaction flip: those locks arbitrate
 * with compaction's own.  It becomes the red harness once bulk ops stop taking
 * them (queued as an optimization).
 *
 * Build against a tree configured with -DFEATURE_FT_LOCK_SPACING_ENV:
 *   gcc -O1 -g -I<top>/include -I<build>/include ft-compact-bulk-rig.c -o rig \
 *     -L<build>/src/.libs -lurcu-memb -lurcu-cds -lurcu-common -lpthread \
 *     -Wl,-rpath,<build>/src/.libs
 */
#define _GNU_SOURCE
#include <urcu/urcu-memb.h>
#include <urcu/fractal-trie.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NR_P		20000
#define NR_ZB		16
#define NR_Z		400	/* keys per z-subtree */

struct tnode {
	struct cds_ft_node node;
};

static struct cds_ft *ft;
static atomic_int stop;
static atomic_ulong compact_passes, compact_steps, compact_busy, bulk_cycles,
	bulk_fail, reads, read_miss;

static void pkey(unsigned int i, uint8_t *k, size_t *len)
{
	k[0] = 'p';
	k[1] = (uint8_t) (i >> 12);
	k[2] = (uint8_t) (i >> 6) & 0x3f;
	k[3] = (uint8_t) i & 0x3f;
	k[4] = (uint8_t) (i * 7);
	*len = 5;
}

static void zkey(unsigned int b, unsigned int i, uint8_t *k, size_t *len)
{
	k[0] = 'z';
	k[1] = (uint8_t) b;
	k[2] = (uint8_t) (i >> 6);
	k[3] = (uint8_t) i & 0x3f;
	*len = 4;
}

static int present(const uint8_t *k, size_t len)
{
	struct cds_ft_iter *it;
	enum cds_ft_status s;

	if (cds_ft_iter_create(ft, &it) != CDS_FT_STATUS_OK)
		abort();
	urcu_memb_read_lock();
	cds_ft_iter_set_key(it, k, len);
	s = cds_ft_lookup(ft, it);
	urcu_memb_read_unlock();
	cds_ft_iter_destroy(it);
	return s == CDS_FT_STATUS_OK;
}

static void *compact_thr(void *arg)
{
	(void) arg;
	urcu_memb_register_thread();
	while (!atomic_load(&stop)) {
		struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);
		enum cds_ft_compact_status s;

		if (!st) {
			usleep(1000);
			continue;
		}
		do {
			s = cds_ft_compact_step(st, 16);
			atomic_fetch_add(&compact_steps, 1);
			if (s == CDS_FT_COMPACT_BUSY)
				atomic_fetch_add(&compact_busy, 1);
		} while (s != CDS_FT_COMPACT_DONE && !atomic_load(&stop));
		cds_ft_compact_end(st);
		atomic_fetch_add(&compact_passes, 1);
	}
	urcu_memb_unregister_thread();
	return NULL;
}

static void *bulk_thr(void *arg)
{
	unsigned int b = 0;

	(void) arg;
	urcu_memb_register_thread();
	while (!atomic_load(&stop)) {
		uint8_t k[2] = { 'z', (uint8_t) (b++ % NR_ZB) };
		struct cds_ft *d = NULL;

		if (cds_ft_detach(ft, k, 2, &d) != CDS_FT_STATUS_OK || !d) {
			atomic_fetch_add(&bulk_fail, 1);
			continue;
		}
		if (cds_ft_graft(ft, k, 2, d) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "RIG: graft back of z%u FAILED\n", k[1]);
			atomic_fetch_add(&bulk_fail, 1);
			abort();
		}
		cds_ft_destroy(d);
		atomic_fetch_add(&bulk_cycles, 1);
	}
	urcu_memb_unregister_thread();
	return NULL;
}

static void *read_thr(void *arg)
{
	unsigned int i = 0;

	(void) arg;
	urcu_memb_register_thread();
	while (!atomic_load(&stop)) {
		uint8_t k[8];
		size_t len;

		pkey(i++ % NR_P, k, &len);
		atomic_fetch_add(&reads, 1);
		if (!present(k, len)) {
			atomic_fetch_add(&read_miss, 1);
			fprintf(stderr, "RIG: reader MISSED p-key %u\n",
				(i - 1) % NR_P);
		}
	}
	urcu_memb_unregister_thread();
	return NULL;
}

int main(void)
{
	struct cds_ft_group *group;
	pthread_t tc, tb, tr;
	unsigned int i, b, missing = 0;
	int secs = getenv("RIG_SECS") ? atoi(getenv("RIG_SECS")) : 20;
	enum cds_ft_status vs;

	urcu_memb_register_thread();
	if (cds_ft_group_create_flavor(NULL, &group, &urcu_memb_flavor) !=
			CDS_FT_STATUS_OK || cds_ft_create(group, NULL, &ft) < 0)
		return 2;
	for (i = 0; i < NR_P; i++) {
		struct tnode *n = calloc(1, sizeof(*n));
		uint8_t k[8];
		size_t len;

		pkey(i, k, &len);
		cds_ft_node_init(&n->node);
		urcu_memb_read_lock();
		if (cds_ft_insert(ft, k, len, &n->node) != CDS_FT_STATUS_OK)
			return 3;
		urcu_memb_read_unlock();
	}
	for (b = 0; b < NR_ZB; b++)
		for (i = 0; i < NR_Z; i++) {
			struct tnode *n = calloc(1, sizeof(*n));
			uint8_t k[8];
			size_t len;

			zkey(b, i, k, &len);
			cds_ft_node_init(&n->node);
			urcu_memb_read_lock();
			if (cds_ft_insert(ft, k, len, &n->node) !=
					CDS_FT_STATUS_OK)
				return 3;
			urcu_memb_read_unlock();
		}
	alarm((unsigned) secs + 120);
	pthread_create(&tc, NULL, compact_thr, NULL);
	pthread_create(&tb, NULL, bulk_thr, NULL);
	pthread_create(&tr, NULL, read_thr, NULL);
	sleep((unsigned) secs);
	atomic_store(&stop, 1);
	pthread_join(tc, NULL);
	pthread_join(tb, NULL);
	pthread_join(tr, NULL);

	for (i = 0; i < NR_P; i++) {
		uint8_t k[8];
		size_t len;

		pkey(i, k, &len);
		missing += !present(k, len);
	}
	for (b = 0; b < NR_ZB; b++)
		for (i = 0; i < NR_Z; i++) {
			uint8_t k[8];
			size_t len;

			zkey(b, i, k, &len);
			missing += !present(k, len);
		}
	vs = cds_ft_verify(ft, stderr);
	printf("spacing=%s secs=%d compact passes=%lu steps=%lu busy=%lu | bulk "
		"cycles=%lu fail=%lu | reads=%lu read_miss=%lu | final missing=%u "
		"verify=%s\n",
		getenv("CDS_FT_LOCK_SPACING") ? getenv("CDS_FT_LOCK_SPACING") :
			"default", secs,
		atomic_load(&compact_passes), atomic_load(&compact_steps),
		atomic_load(&compact_busy), atomic_load(&bulk_cycles),
		atomic_load(&bulk_fail), atomic_load(&reads),
		atomic_load(&read_miss), missing,
		vs == CDS_FT_STATUS_OK ? "OK" : "FAIL");
	return (missing || vs != CDS_FT_STATUS_OK ||
		atomic_load(&read_miss)) ? 1 : 0;
}
