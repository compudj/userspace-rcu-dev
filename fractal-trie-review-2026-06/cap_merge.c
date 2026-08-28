/*
 * ITEM 2 / shape A: does the lock_fine MERGE overlap recursion register more
 * than FT_FLIP_TXN_MAX_LOCKS (257) words in one txn?
 *
 * ft_merge_build fences EVERY internal/compressed overlap node on each side,
 * ft_glue_defer_free_fenced records it, and ft_glue_tombstone_free_list
 * registers each !shared fenced holder into the ONE txn.  So the registry load
 * tracks the OVERLAP NODE COUNT, which the caller controls.
 *
 * Build N structurally identical internal nodes in both tries under a common
 * parent: for each 2-byte prefix, two children, so the prefix node is a real
 * internal node with nr_child=2 rather than a compressed leaf.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <urcu-qsbr.h>
#include <urcu/fractal-trie.h>

struct tn { struct cds_ft_node node; int v; };
static struct cds_ft_node *na(int v){struct tn*n=calloc(1,sizeof *n);n->v=v;return &n->node;}

static void fill(struct cds_ft *ft, int n)
{
	int i, k;
	for (i = 0; i < n; i++) {
		for (k = 1; k <= 2; k++) {
			uint8_t key[4];
			key[0] = 'Z';
			key[1] = (uint8_t) ('a' + (i / 150));
			key[2] = (uint8_t) (i % 150);
			key[3] = (uint8_t) k;
			cds_ft_insert(ft, key, 4, na(i * 4 + k));
		}
	}
}

int main(int argc, char **argv)
{
	struct cds_ft_group_attr *attr; struct cds_ft_group *group;
	struct cds_ft *dst, *src; enum cds_ft_status s;
	int n = argc > 1 ? atoi(argv[1]) : 300;

	rcu_register_thread();
	cds_ft_group_attr_create(&attr);
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_create(attr, &group);
	cds_ft_group_attr_destroy(attr);
	cds_ft_create(group, NULL, &dst);
	cds_ft_create(group, NULL, &src);

	rcu_read_lock();
	fill(dst, n);
	fill(src, n);
	rcu_read_unlock();
	printf("overlap prefixes=%d  dst_keys=%lu src_keys=%lu\n", n,
		cds_ft_count_keys(dst), cds_ft_count_keys(src));
	fflush(stdout);

	cds_ft_make_exclusive(src);
	s = cds_ft_merge_at(dst, (const uint8_t *) "Z", 1,
			src, (const uint8_t *) "Z", 1);
	printf("merge_at -> %s\n", cds_ft_status_to_string(s));
	rcu_read_lock();
	printf("verify dst: %s  count=%lu\n",
		cds_ft_status_to_string(cds_ft_verify(dst, stderr)),
		cds_ft_count_keys(dst));
	rcu_read_unlock();
	rcu_unregister_thread();
	return 0;
}
