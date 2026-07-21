# SPDX-FileCopyrightText: 2023 EfficiOS Inc.
#
# SPDX-License-Identifier: MIT

AM_CPPFLAGS += -I$(top_srcdir)/src -I$(top_srcdir)/tests/utils -I$(top_srcdir)/tests/common

LOG_DRIVER_FLAGS = --merge --comments
LOG_DRIVER = env AM_TAP_AWK='$(AWK)' \
	URCU_TESTS_SRCDIR='$(abs_top_srcdir)/tests' \
	URCU_TESTS_BUILDDIR='$(abs_top_builddir)/tests' \
	$(SHELL) $(top_srcdir)/tests/utils/tap-driver.sh

noinst_PROGRAMS = \
	get_cpu_mask_from_sysfs \
	get_cpu_mask_from_sysfs_cxx \
	get_max_cpuid_from_sysfs \
	get_max_cpuid_from_sysfs_cxx \
	test_arch \
	test_arch_cxx \
	test_get_max_cpuid_from_mask \
	test_get_max_cpuid_from_mask_cxx \
	test_get_possible_cpus_array_len \
	test_get_possible_cpus_array_len_cxx \
	test_uatomic \
	test_uatomic_cxx \
	test_rcu_txn_sw \
	test_rcu_mcas \
	test_rcu_txn_sw_mw \
	test_rcu_txn \
	test_rcu_txn_grow \
	test_rcu_txn_fallback \
	test_rcu_txn_fallback_policy \
	test_rcu_txn_fallback_publish \
	test_rcu_txn_validate \
	test_rcu_txn_sw_list \
	test_rcu_txn_sw_list_concurrent \
	test_rcu_txn_sw_bitmap \
	test_rcu_txn_sw_hlist \
	test_rcu_txn_list \
	test_rcu_txn_list_guarded \
	test_rcu_txn_list_compose \
	test_rcu_txn_list_replace \
	test_rcu_txn_hlist \
	test_rcu_txn_hlist_ryw \
	test_rcu_txn_bitmap \
	test_rcu_txn_skiplist \
	test_rcu_txn_skiplist_ryw \
	test_rcu_txn_slab \
	test_rcu_txn_read_policy \
	test_rcu_txn_beginless \
	test_rcu_txn_guards \
	test_rcu_txn_sw_excl \
	test_fair_mutex \
	test_urcu_multiflavor \
	test_urcu_multiflavor_cxx \
	test_urcu_multiflavor_dynlink \
	test_urcu_multiflavor_dynlink_cxx \
	test_urcu_multiflavor_single_unit \
	test_urcu_multiflavor_single_unit_cxx \
	test_urcu_multiflavor_single_unit_dynlink \
	test_urcu_multiflavor_single_unit_dynlink_cxx \
	test_build \
	test_build_cxx \
	test_build_dynlink \
	test_build_dynlink_cxx

dist_noinst_SCRIPTS = \
	test_get_cpu_mask_from_sysfs \
	test_get_cpu_mask_from_sysfs_cxx \
	test_get_max_cpuid_from_sysfs \
	test_get_max_cpuid_from_sysfs_cxx

TESTS = \
	test_arch \
	test_arch_cxx \
	test_get_cpu_mask_from_sysfs \
	test_get_cpu_mask_from_sysfs_cxx \
	test_get_max_cpuid_from_mask \
	test_get_max_cpuid_from_mask_cxx \
	test_get_max_cpuid_from_sysfs \
	test_get_max_cpuid_from_sysfs_cxx \
	test_get_possible_cpus_array_len \
	test_get_possible_cpus_array_len_cxx \
	test_uatomic \
	test_uatomic_cxx \
	test_rcu_txn_sw \
	test_rcu_mcas \
	test_rcu_txn_sw_mw \
	test_rcu_txn \
	test_rcu_txn_grow \
	test_rcu_txn_fallback \
	test_rcu_txn_fallback_policy \
	test_rcu_txn_fallback_publish \
	test_rcu_txn_validate \
	test_rcu_txn_sw_list \
	test_rcu_txn_sw_list_concurrent \
	test_rcu_txn_sw_bitmap \
	test_rcu_txn_sw_hlist \
	test_rcu_txn_list \
	test_rcu_txn_list_guarded \
	test_rcu_txn_list_compose \
	test_rcu_txn_list_replace \
	test_rcu_txn_hlist \
	test_rcu_txn_hlist_ryw \
	test_rcu_txn_bitmap \
	test_rcu_txn_skiplist \
	test_rcu_txn_skiplist_ryw \
	test_rcu_txn_slab \
	test_rcu_txn_read_policy \
	test_rcu_txn_beginless \
	test_rcu_txn_guards \
	test_rcu_txn_sw_excl \
	test_fair_mutex \
	test_urcu_multiflavor \
	test_urcu_multiflavor_cxx \
	test_urcu_multiflavor_dynlink \
	test_urcu_multiflavor_dynlink_cxx \
	test_urcu_multiflavor_single_unit \
	test_urcu_multiflavor_single_unit_cxx \
	test_urcu_multiflavor_single_unit_dynlink \
	test_urcu_multiflavor_single_unit_dynlink_cxx \
	test_build \
	test_build_cxx \
	test_build_dynlink \
	test_build_dynlink_cxx

noinst_HEADERS = test_urcu_multiflavor.h

URCU_COMMON_LIB=$(top_builddir)/src/liburcu-common.la
URCU_LIB=$(top_builddir)/src/liburcu.la
URCU_QSBR_LIB=$(top_builddir)/src/liburcu-qsbr.la
URCU_MB_LIB=$(top_builddir)/src/liburcu-mb.la
URCU_BP_LIB=$(top_builddir)/src/liburcu-bp.la
URCU_CDS_LIB=$(top_builddir)/src/liburcu-cds.la
TAP_LIB=$(top_builddir)/tests/utils/libtap.a

get_cpu_mask_from_sysfs_cxx_SOURCES = get_cpu_mask_from_sysfs_cxx.cpp
get_cpu_mask_from_sysfs_SOURCES = get_cpu_mask_from_sysfs.c
get_max_cpuid_from_sysfs_cxx_SOURCES = get_max_cpuid_from_sysfs_cxx.cpp
get_max_cpuid_from_sysfs_SOURCES = get_max_cpuid_from_sysfs.c

test_arch_SOURCES = test_arch.c
test_arch_LDADD = $(URCU_COMMON_LIB) $(TAP_LIB)

test_arch_cxx_SOURCES = test_arch_cxx.cpp
test_arch_cxx_LDADD = $(URCU_COMMO_LIB) $(TAP_LIB)

test_get_max_cpuid_from_mask_SOURCES = test_get_max_cpuid_from_mask.c
test_get_max_cpuid_from_mask_LDADD = $(TAP_LIB)

test_get_max_cpuid_from_mask_cxx_SOURCES = test_get_max_cpuid_from_mask_cxx.cpp
test_get_max_cpuid_from_mask_cxx_LDADD = $(TAP_LIB)

test_get_possible_cpus_array_len_SOURCES = test_get_possible_cpus_array_len.c
test_get_possible_cpus_array_len_LDADD = $(TAP_LIB)

test_get_possible_cpus_array_len_cxx_SOURCES = test_get_possible_cpus_array_len_cxx.cpp
test_get_possible_cpus_array_len_cxx_LDADD = $(TAP_LIB)

test_uatomic_SOURCES = test_uatomic.c
test_uatomic_LDADD = $(URCU_COMMON_LIB) $(TAP_LIB)

test_uatomic_cxx_SOURCES = test_uatomic_cxx.cpp
test_uatomic_cxx_LDADD = $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_SOURCES = test_rcu_txn_sw.c
test_rcu_txn_sw_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_mcas_SOURCES = test_rcu_mcas.c
test_rcu_mcas_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_mw_SOURCES = test_rcu_txn_sw_mw.c
test_rcu_txn_sw_mw_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_SOURCES = test_rcu_txn.c
test_rcu_txn_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_grow_SOURCES = test_rcu_txn_grow.c
test_rcu_txn_grow_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_fallback_SOURCES = test_rcu_txn_fallback.c
test_rcu_txn_fallback_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_fallback_policy_SOURCES = test_rcu_txn_fallback_policy.c
test_rcu_txn_fallback_policy_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_fallback_publish_SOURCES = test_rcu_txn_fallback_publish.c
test_rcu_txn_fallback_publish_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_validate_SOURCES = test_rcu_txn_validate.c
test_rcu_txn_validate_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_list_SOURCES = test_rcu_txn_sw_list.c
test_rcu_txn_sw_list_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_list_concurrent_SOURCES = test_rcu_txn_sw_list_concurrent.c
test_rcu_txn_sw_list_concurrent_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_list_SOURCES = test_rcu_txn_list.c
test_rcu_txn_list_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_list_guarded_SOURCES = test_rcu_txn_list_guarded.c
test_rcu_txn_list_guarded_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_list_compose_SOURCES = test_rcu_txn_list_compose.c
test_rcu_txn_list_compose_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_list_replace_SOURCES = test_rcu_txn_list_replace.c
test_rcu_txn_list_replace_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_hlist_SOURCES = test_rcu_txn_hlist.c
test_rcu_txn_hlist_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_hlist_ryw_SOURCES = test_rcu_txn_hlist_ryw.c
test_rcu_txn_hlist_ryw_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_bitmap_SOURCES = test_rcu_txn_bitmap.c
test_rcu_txn_bitmap_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_bitmap_SOURCES = test_rcu_txn_sw_bitmap.c
test_rcu_txn_sw_bitmap_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_sw_hlist_SOURCES = test_rcu_txn_sw_hlist.c
test_rcu_txn_sw_hlist_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_skiplist_SOURCES = test_rcu_txn_skiplist.c
test_rcu_txn_skiplist_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_skiplist_ryw_SOURCES = test_rcu_txn_skiplist_ryw.c
test_rcu_txn_skiplist_ryw_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_slab_SOURCES = test_rcu_txn_slab.c
test_rcu_txn_slab_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)
# Compiled with the slab's stats counters live so the test can assert on
# reuse/carve directly (the counters are unconditional struct fields; the flag
# only enables the per-TU increments).  Harmless for a correctness test.
test_rcu_txn_slab_CPPFLAGS = $(AM_CPPFLAGS) -DURCU_TXN_CACHE_STATS

test_rcu_txn_read_policy_SOURCES = test_rcu_txn_read_policy.c
test_rcu_txn_read_policy_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)
# Built in read-policy SOFT-counting mode so the test can assert on
# urcu_txn_read_policy_violations() instead of aborting on the first violation
# (the negative-control case deliberately provokes one).  In a default build the
# read-policy machinery is compiled out entirely, so this flag is what makes the
# check exist at all -- see the header block above URCU_TXN_ENOMEM in rcu-txn.h.
test_rcu_txn_read_policy_CPPFLAGS = $(AM_CPPFLAGS) \
	-DURCU_TXN_DEBUG_READ_POLICY -DURCU_TXN_DEBUG_READ_POLICY_SOFT

test_rcu_txn_beginless_SOURCES = test_rcu_txn_beginless.c
test_rcu_txn_beginless_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)

test_rcu_txn_guards_SOURCES = test_rcu_txn_guards.c
test_rcu_txn_guards_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)
# The contract guards this test provokes are urcu_assert_debug(), which exists
# only under DEBUG_RCU.  Build the test with it so the guards are there whatever
# the tree's build flags -- the test forks a child per case and asserts it dies
# by SIGABRT, so a compiled-out guard would silently pass otherwise.  (The sw
# cases are urcu_posix_assert: present unless NDEBUG, under which the test skips
# itself entirely.)
test_rcu_txn_guards_CPPFLAGS = $(AM_CPPFLAGS) -DDEBUG_RCU

test_rcu_txn_sw_excl_SOURCES = test_rcu_txn_sw_excl.c
test_rcu_txn_sw_excl_LDADD = $(URCU_QSBR_LIB) $(URCU_COMMON_LIB) $(TAP_LIB)
# The sw single-updater validator is opt-in (zero overhead by default), so the
# test that provokes it has to turn it on.  It forks a child per case and
# asserts the child dies by SIGABRT.
test_rcu_txn_sw_excl_CPPFLAGS = $(AM_CPPFLAGS) -DURCU_TXN_SW_EXCL_VALIDATE

test_fair_mutex_SOURCES = test_fair_mutex.c
test_fair_mutex_LDADD = $(URCU_COMMON_LIB) $(TAP_LIB)

test_urcu_multiflavor_SOURCES = test_urcu_multiflavor.c \
	test_urcu_multiflavor-memb.c \
	test_urcu_multiflavor-mb.c \
	test_urcu_multiflavor-qsbr.c \
	test_urcu_multiflavor-bp.c
test_urcu_multiflavor_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_cxx_SOURCES = test_urcu_multiflavor_cxx.cpp \
	test_urcu_multiflavor-memb_cxx.cpp \
	test_urcu_multiflavor-mb_cxx.cpp \
	test_urcu_multiflavor-qsbr_cxx.cpp \
	test_urcu_multiflavor-bp_cxx.cpp
test_urcu_multiflavor_cxx_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_dynlink_SOURCES = test_urcu_multiflavor.c \
	test_urcu_multiflavor-memb.c \
	test_urcu_multiflavor-mb.c \
	test_urcu_multiflavor-qsbr.c \
	test_urcu_multiflavor-bp.c
test_urcu_multiflavor_dynlink_CFLAGS = -DDYNAMIC_LINK_TEST $(AM_CFLAGS)
test_urcu_multiflavor_dynlink_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_dynlink_cxx_SOURCES = test_urcu_multiflavor_cxx.cpp \
	test_urcu_multiflavor-memb_cxx.cpp \
	test_urcu_multiflavor-mb_cxx.cpp \
	test_urcu_multiflavor-qsbr_cxx.cpp \
	test_urcu_multiflavor-bp_cxx.cpp
test_urcu_multiflavor_dynlink_cxx_CXXFLAGS = -DDYNAMIC_LINK_TEST $(AM_CXXFLAGS)
test_urcu_multiflavor_dynlink_cxx_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_single_unit_SOURCES = test_urcu_multiflavor_single_unit.c
test_urcu_multiflavor_single_unit_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_single_unit_cxx_SOURCES = test_urcu_multiflavor_single_unit_cxx.cpp
test_urcu_multiflavor_single_unit_cxx_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_single_unit_dynlink_SOURCES = test_urcu_multiflavor_single_unit.c
test_urcu_multiflavor_single_unit_dynlink_CFLAGS = -DDYNAMIC_LINK_TEST $(AM_CFLAGS)
test_urcu_multiflavor_single_unit_dynlink_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_urcu_multiflavor_single_unit_dynlink_cxx_SOURCES = test_urcu_multiflavor_single_unit_cxx.cpp
test_urcu_multiflavor_single_unit_dynlink_cxx_CXXFLAGS = -DDYNAMIC_LINK_TEST $(AM_CXXFLAGS)
test_urcu_multiflavor_single_unit_dynlink_cxx_LDADD = $(URCU_LIB) $(URCU_MB_LIB) \
	$(URCU_QSBR_LIB) $(URCU_BP_LIB) $(TAP_LIB)

test_build_SOURCES = \
	test_build.c
test_build_LDADD = $(URCU_COMMON_LIB) $(URCU_CDS_LIB) $(TAP_LIB)

test_build_cxx_SOURCES = \
	test_build_cxx.cpp
test_build_cxx_LDADD = $(URCU_COMMON_LIB) $(URCU_CDS_LIB) $(TAP_LIB)

test_build_dynlink_SOURCES = \
	test_build.c
test_build_dynlink_CFLAGS = -DDYNAMIC_LINK_TEST $(AM_CFLAGS)
test_build_dynlink_LDADD = $(URCU_LIB) $(URCU_COMMON_LIB) $(URCU_CDS_LIB) $(TAP_LIB)

test_build_dynlink_cxx_SOURCES = \
	test_build_cxx.cpp
test_build_dynlink_cxx_CXXFLAGS = -DDYNAMIC_LINK_TEST $(AM_CXXFLAGS)
test_build_dynlink_cxx_LDADD = $(URCU_LIB) $(URCU_COMMON_LIB) $(URCU_CDS_LIB) $(TAP_LIB)
