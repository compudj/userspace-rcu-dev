#!/bin/sh

# TODO: missing tests:
# - send kill signals during tests to change the behavior between
#   add/remove/random
# - validate that "nr_leaked" is always 0 in SUMMARY for all tests

# 30 seconds per test
TIME_UNITS=30

TESTPROG=./test_urcu_ft

#thread multiplier
THREAD_MUL=1

EXTRA_PARAMS=--verbose

# ** test update coherency with single-value table

# sanity test
${TESTPROG} 0 $((4*${THREAD_MUL})) ${TIME_UNITS} --sanity-test ${EXTRA_PARAMS} || exit 1
${TESTPROG} 0 $((4*${THREAD_MUL})) ${TIME_UNITS} --sanity-test-varlen ${EXTRA_PARAMS} || exit 1
${TESTPROG} 0 $((4*${THREAD_MUL})) ${TIME_UNITS} --sanity-test-varlen-string ${EXTRA_PARAMS} || exit 1

# rw test, single key, add and del randomly, 4 threads
# key range: init, lookup, and update: 0 to 0
${TESTPROG} 0 $((4*${THREAD_MUL})) ${TIME_UNITS} --lookup-pool-size 1 --write-pool-size 1 --init-pool-size 1 ${EXTRA_PARAMS} || exit 1

# rw test, single key, add and del randomly, 2 lookup threads, 2 update threads
# key range: init, lookup, and update: 0 to 0
${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --lookup-pool-size 1 --write-pool-size 1 --init-pool-size 1 ${EXTRA_PARAMS} || exit 1

# add with duplicates

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 1 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 1 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 1 --lookup-pool-size 255 --write-pool-size 255 --init-pool-size 255 ${EXTRA_PARAMS} || exit 1

#expected fail (TODO)
#${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 1 --lookup-pool-size 256 --write-pool-size 256 --init-pool-size 256 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 2 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 2 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 2 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535 ${EXTRA_PARAMS} || exit 1


${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 3 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 3 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 3 --lookup-pool-size 16777215 --write-pool-size 16777215 --init-pool-size 16777215 ${EXTRA_PARAMS} || exit 1


${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 4 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 4 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 4 --lookup-pool-size 1000000 --write-pool-size 1000000 --init-pool-size 1000000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 4 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --key-len 8 ${EXTRA_PARAMS} || exit 1

# with node leak detection
${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --leak-detection --key-len 4 ${EXTRA_PARAMS} || exit 1


# add unique

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 255 --write-pool-size 255 --init-pool-size 255 ${EXTRA_PARAMS} || exit 1

#expected fail (TODO)
#${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 1 --lookup-pool-size 256 --write-pool-size 256 --init-pool-size 256 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 2 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535 ${EXTRA_PARAMS} || exit 1


${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 65535 --write-pool-size 65535 --init-pool-size 65535 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 3 --lookup-pool-size 16777215 --write-pool-size 16777215 --init-pool-size 16777215 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 10 --write-pool-size 10 --init-pool-size 10 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 1000 --write-pool-size 1000 --init-pool-size 1000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 4 --lookup-pool-size 1000000 --write-pool-size 1000000 --init-pool-size 1000000 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 4 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --key-len 8 ${EXTRA_PARAMS} || exit 1

# with node leak detection
${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 ${EXTRA_PARAMS} || exit 1

# removal (0% add), leak detection

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 0 ${EXTRA_PARAMS} || exit 1

# vary add ratio, leak detection

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 5 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --insert-unique --leak-detection --key-len 4 --populate --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 --insert-ratio 95 ${EXTRA_PARAMS} || exit 1


# validate lookup of init values

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 ${EXTRA_PARAMS} || exit 1

# vary key multiplication factor

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --key-mul 17 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 4 --key-mul 17 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 ${EXTRA_PARAMS} || exit 1


${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 8 --key-mul 1717 --populate --write-pool-offset 100 --lookup-pool-size 100 --write-pool-size 100 --init-pool-size 100 ${EXTRA_PARAMS} || exit 1

${TESTPROG} $((2*${THREAD_MUL})) $((2*${THREAD_MUL})) ${TIME_UNITS} --validate-lookup --insert-unique --key-len 8 --key-mul 1717 --populate --write-pool-offset 10000 --lookup-pool-size 10000 --write-pool-size 10000 --init-pool-size 10000 ${EXTRA_PARAMS} || exit 1
