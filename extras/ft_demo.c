/*
 * SPDX-FileCopyrightText: 2026 EfficiOS Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * ft_demo.c — populate two Fractal Trie instances in the same
 * process so a single LTTng capture illustrates the full
 * structural vocabulary that cds_ft_ emits.  Dumps
 * cds_ft_show(JSON) for each so the trace-reconstructed snapshot
 * (extras/ft_visualize.py) can be diffed against an authoritative
 * snapshot.
 *
 *   cc -I<srcdir>/include                                             \
 *      -L<builddir>/src/.libs                                         \
 *      extras/ft_demo.c -lurcu-qsbr -lurcu-cds -ldl                   \
 *      -Wl,-rpath,<builddir>/src/.libs -o ft_demo
 *
 * Session setup (see extras/ft_visualize.py module docstring for
 * the full recipe; the vpid context is mandatory for
 * ft_visualize.py to scope its (group, ft) tuples):
 *
 *   lttng create
 *   lttng enable-event --userspace 'cds_ft:*'
 *   lttng add-context --userspace --type vpid
 *   lttng start
 *   ./ft_demo
 *   lttng stop
 *
 * Tries:
 *
 *   dns trie — skip-compressed group, DNS-reverse-ish keys,
 *              exercises LINEAR / LINEAR_WIDE / COMPRESSED /
 *              EXTERNAL with skip-compressed fast-path pointers.
 *
 *   col trie — non-skip group, "NNNN<branch><3-byte-suffix>"
 *              keys.  Skip-compressed mode inhibits collapse (its
 *              compact layout already beats the collapsed cost
 *              model), so the collapsed showcase lives in its
 *              own group.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <urcu-qsbr.h>
#include <urcu/rculist.h>
#include <urcu/fractal-trie.h>

struct demo_node {
	struct cds_ft_node node;
	unsigned int val;
};

/* DNS-reverse-ish keys: exercise compressed paths + wide/linear
 * splits in skip-compressed mode.  Collapse is intentionally not
 * sought here (see col_keys below). */
static const char *dns_keys[] = {
	"arpa",
	"arpa.in-addr",
	"arpa.in-addr.10",
	"arpa.in-addr.10.0.0.1",
	"arpa.in-addr.10.0.0.2",
	"arpa.in-addr.10.0.0.3",
	"arpa.in-addr.192",
	"arpa.in-addr.192.168",
	"arpa.in-addr.192.168.1.1",
	"arpa.in-addr.192.168.1.2",
	"arpa.in-addr.192.168.1.3",
	"arpa.in-addr.172.16.0.1",
	"arpa.in-addr.172.16.0.5",
	"arpa.ip6.fe80",
	"arpa.ip6.fe80.1",
	"arpa.ip6.fe80.2",
	"com.example",
	"com.example.www",
	"com.example.api.v1",
	"com.example.api.v2",
};

/* Collapse showcase: 4-way branch under NNNN prefix, each branch
 * carries three 3-byte suffixes.  The 3-byte suffix satisfies
 * ft_try_collapse_at_node's per-config min_slen and
 * FT_COLLAPSE_SUFFIX_MIN.  Two short keys + one different-branch
 * trigger key round the shape out to reproduce the pattern that
 * test_density_collapse_explode uses, lifted to printable bytes. */
static const char *col_keys[] = {
	"NNNNa123", "NNNNa456", "NNNNa789",
	"NNNNb123", "NNNNb456", "NNNNb789",
	"NNNNc123", "NNNNc456", "NNNNc789",
	"NNNNd123", "NNNNd456", "NNNNd789",
	"NNNNa", "NNNNb",
	"M1",
};

#define NR_DNS_KEYS ((int)(sizeof(dns_keys) / sizeof(dns_keys[0])))
#define NR_COL_KEYS ((int)(sizeof(col_keys) / sizeof(col_keys[0])))

static int make_group(struct cds_ft_group **group_out, unsigned int flags)
{
	struct cds_ft_group_attr *attr;

	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	if (flags)
		cds_ft_group_attr_set_flags(attr, flags);
	if (cds_ft_group_create(attr, group_out) != CDS_FT_STATUS_OK) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	return 0;
}

static int populate(struct cds_ft *ft, const char **keys, int nkeys,
		struct demo_node *nodes, const char *label)
{
	int i;

	for (i = 0; i < nkeys; i++) {
		size_t klen = strlen(keys[i]);
		nodes[i].val = i;
		if (cds_ft_insert(ft, (const uint8_t *) keys[i], klen,
				&nodes[i].node) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "%s: insert %s failed\n",
				label, keys[i]);
			return -1;
		}
	}
	return 0;
}

static void dump_json(struct cds_ft *ft, const char *path)
{
	FILE *j = fopen(path, "w");
	if (j) {
		cds_ft_show(ft, j, CDS_FT_SHOW_JSON);
		fclose(j);
	}
}

static void drain(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) != CDS_FT_STATUS_OK)
		return;
	{
		struct cds_ft_node *head;
		while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK &&
		       cds_ft_remove_all(ft, iter, &head) == CDS_FT_STATUS_OK) {
			(void) head;
		}
	}
	cds_ft_iter_destroy(iter);
}

int main(void)
{
	struct cds_ft_group *dns_group, *col_group;
	struct cds_ft *dns_ft, *col_ft;
	struct demo_node *dns_nodes, *col_nodes;

	rcu_register_thread();
	rcu_read_lock();

	if (make_group(&dns_group, CDS_FT_FLAG_SKIP_COMPRESSED) < 0)
		return 1;
	if (make_group(&col_group, 0) < 0)
		return 1;
	if (cds_ft_create(dns_group, NULL, &dns_ft) != CDS_FT_STATUS_OK)
		return 1;
	if (cds_ft_create(col_group, NULL, &col_ft) != CDS_FT_STATUS_OK)
		return 1;

	dns_nodes = calloc(NR_DNS_KEYS, sizeof(*dns_nodes));
	col_nodes = calloc(NR_COL_KEYS, sizeof(*col_nodes));

	if (populate(dns_ft, dns_keys, NR_DNS_KEYS, dns_nodes, "dns") < 0)
		return 1;
	if (populate(col_ft, col_keys, NR_COL_KEYS, col_nodes, "col") < 0)
		return 1;

	dump_json(dns_ft, "/tmp/ft_dns_demo.json");
	dump_json(col_ft, "/tmp/ft_col_demo.json");

	fprintf(stderr,
		"dns group=%p ft=%p nr_keys=%d\n"
		"col group=%p ft=%p nr_keys=%d\n",
		(void *) dns_group, (void *) dns_ft, NR_DNS_KEYS,
		(void *) col_group, (void *) col_ft, NR_COL_KEYS);

	rcu_read_unlock();

	/* Keep the tries alive a moment — gives the LTTng consumer
	 * some breathing room before the teardown events fire.
	 * The tries are at their post-insert peak during this
	 * window, which is what ft_visualize.py's --end should
	 * target when rendering the authoritative snapshot. */
	usleep(50000);

	rcu_read_lock();
	drain(dns_ft);
	drain(col_ft);
	rcu_read_unlock();

	cds_ft_destroy(dns_ft);
	cds_ft_destroy(col_ft);
	cds_ft_group_destroy(dns_group);
	cds_ft_group_destroy(col_group);
	free(dns_nodes);
	free(col_nodes);
	rcu_unregister_thread();
	return 0;
}
