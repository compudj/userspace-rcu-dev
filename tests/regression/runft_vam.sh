#!/bin/sh
#
# Verify-at-mutation variant of runft.sh.  Same set of invocations as
# runft.sh, but each one runs against a library built with
# -DFEATURE_FT_VERIFY_AT_MUTATION (see fractal-trie-internal.h) and
# passes --verify-at-mutation-period N to test_urcu_ft so the writer
# scope-exit hook samples cds_ft_verify at a tractable cadence instead
# of the every-mutation default.
#
# Verify is O(N) per call, so verifying every mutation on a large
# trie collapses throughput (16M-pool runs were minutes per
# mutation).  This script picks a period that scales roughly
# inversely with pool size, keeping wall time bounded while still
# giving high structural coverage.
#
# Usage: build the library with
#   make CFLAGS="-O2 -DFEATURE_FT_VERIFY_AT_MUTATION"
# then ./runft_vam.sh from this directory.  Without VAM compiled in,
# test_urcu_ft refuses to start (--verify-at-mutation-period returns
# CDS_FT_STATUS_NOT_SUPPORTED, the helper aborts with a clear error).

# Reduced from runft.sh's 30s default: VAM increases per-mutation
# cost noticeably even at large periods, and 5s already exercises
# every code path the longer run does.
TIME_UNITS=5

TESTPROG=./test_urcu_ft

# thread multiplier
THREAD_MUL=1

EXTRA_PARAMS=--verbose

# Map a pool size to a verify-at-mutation period.  The mapping is a
# loose 1/N: each verify is O(N), and the period absorbs the per-
# mutation cost so that wall time stays roughly proportional to
# mutation count rather than to N * mutation count.
period_for_pool() {
	pool=$1
	if   [ "$pool" -le 255 ];      then echo 1
	elif [ "$pool" -le 1000 ];     then echo 10
	elif [ "$pool" -le 10000 ];    then echo 100
	elif [ "$pool" -le 65535 ];    then echo 1000
	elif [ "$pool" -le 1000000 ];  then echo 10000
	else                                echo 100000
	fi
}

# Extract the lookup-pool-size value from a test_urcu_ft argv.
# Returns DEFAULT_RAND_POOL (1000000, as defined in test_urcu_ft.h)
# when --lookup-pool-size is not given on the command line — that is
# what test_urcu_ft itself uses for the unspecified case.
pool_from_args() {
	prev=
	for a in "$@"; do
		if [ "$prev" = "--lookup-pool-size" ]; then
			echo "$a"
			return
		fi
		prev=$a
	done
	echo 1000000
}

# Run a single test_urcu_ft invocation with an auto-computed verify
# period.  Sanity-test invocations are always tiny and deterministic
# so we keep period=1 to maximise coverage on the structural tests
# that exercise the most corner cases (varlen, varlen-string).
run_test() {
	case "$*" in
	*--sanity-test*)
		period=1
		pool=N/A
		;;
	*)
		pool=$(pool_from_args "$@")
		period=$(period_for_pool "$pool")
		;;
	esac
	echo ">>> verify period=$period (pool=$pool): $TESTPROG $* --verify-at-mutation-period $period $EXTRA_PARAMS"
	$TESTPROG "$@" --verify-at-mutation-period "$period" $EXTRA_PARAMS || exit 1
}

# ---- sanity tests ----
run_test 0 $((4*THREAD_MUL)) ${TIME_UNITS} --sanity-test
run_test 0 $((4*THREAD_MUL)) ${TIME_UNITS} --sanity-test-varlen
run_test 0 $((4*THREAD_MUL)) ${TIME_UNITS} --sanity-test-varlen-string

# ---- single-key add/remove ----
run_test 0 $((4*THREAD_MUL)) ${TIME_UNITS} --lookup-pool-size 1 --write-pool-size 1 --init-pool-size 1
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --lookup-pool-size 1 --write-pool-size 1 --init-pool-size 1

# ---- add with duplicates ----
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 1 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 1 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 1 --lookup-pool-size 255 --write-pool-size 255 --init-pool-size 255

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 2 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 2 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 2 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 3 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 3 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 3 --lookup-pool-size 16777215 --write-pool-size 16777215 --init-pool-size 16777215

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 4 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 4 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 4 --lookup-pool-size 1000000 --write-pool-size 1000000 --init-pool-size 1000000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 4
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --key-len 8

# with node leak detection
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --leak-detection --key-len 4

# ---- add unique ----
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 255 --write-pool-size 255 --init-pool-size 255

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 16777215 --write-pool-size 16777215 --init-pool-size 16777215

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 1000000 --write-pool-size 1000000 --init-pool-size 1000000
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 4
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --key-len 8

# with node leak detection
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4

# removal (0% add), leak detection
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 0

# vary add ratio, leak detection
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 5
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 95

# validate lookup of init values
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000

# vary key multiplication factor
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --key-mul 17 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --key-mul 17 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000

run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 8 --key-mul 1717 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100
run_test $((2*THREAD_MUL)) $((2*THREAD_MUL)) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 8 --key-mul 1717 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000

echo "All runft_vam.sh invocations completed successfully."
