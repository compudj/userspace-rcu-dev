/*
 * Linker script fragment for FT lookup-API cluster placement.
 *
 * Each public-API entry point and its specialized inners live in
 * sub-sections under .text.hot.cds_ft_<api>.<0_dispatch | 1_fast |
 * 2_slow>.  This script gathers each cluster into one page-aligned
 * output section, sub-sections sorted alphabetically:
 *
 *   ALIGN(4096) ┐
 *               │  .text.hot.cds_ft_<api>.0_dispatch  — public stub
 *               │  .text.hot.cds_ft_<api>.1_fast      — primary hot
 *               │                                      inner
 *               │  .text.hot.cds_ft_<api>.2_slow     — slow paths:
 *               │                                      *_nosc, precise_*,
 *               │                                      *_nonidentity
 *
 * The bench's hot path (call → stub → primary) lands at the
 * page-aligned cluster start and the stub's indirect jmp forwards
 * into the primary on the same page.  Slow paths trail later in
 * the cluster.  The whole cluster shares iTLB/icache locality;
 * when the fn-ptr happens to point to a slow path on a rare
 * config the jmp lands on a nearby page rather than at the other
 * end of .text.
 *
 * Pass to ld via -Wl,-T,ft-lookup-layout.ld (injected by
 * src/Makefile.am via liburcu_cds_la_LDFLAGS).
 */

SECTIONS
{
	.text.cluster_cds_ft_lookup_key : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_key.*)))
	}
	.text.cluster_cds_ft_lookup_candidate_key : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_candidate_key.*)))
	}
	.text.cluster_cds_ft_lookup : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup.*)))
	}
	.text.cluster_cds_ft_lookup_partial_key : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_partial_key.*)))
	}
	.text.cluster_cds_ft_lookup_partial : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_partial.*)))
	}
	.text.cluster_cds_ft_lookup_longest_match_key : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_longest_match_key.*)))
	}
	.text.cluster_cds_ft_lookup_longest_match : ALIGN(4096) {
		KEEP(*(SORT_BY_NAME(.text.hot.cds_ft_lookup_longest_match.*)))
	}
} INSERT BEFORE .text;
