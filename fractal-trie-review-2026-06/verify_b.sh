#!/bin/bash
# Finding-B fix verification.  Two independent bars:
#   (1) the ORACLE bar   -- root-only MW must stop aborting (N runs, was ~1 in 2)
#   (2) the TRACE bar    -- stale.py's suspect count must go to ZERO
# plus the standing green legs.  Usage: verify_b.sh [runs]
set -u
SP="$(dirname "$0")"; RUNS="${1:-6}"; W=/mnt/data/efficios/git/userspace-rcu
fail=0

echo "=== (1) ORACLE BAR: root-only MW x $RUNS (build-e3) ==="
cd $W/build-e3/tests/regression || exit 1
for r in $(seq 1 $RUNS); do
  (ulimit -v 60000000; FT_INV_MW=1 CDS_FT_LOCK_SPACING=root-only timeout 900 \
     ./test_urcu_ft_inv >$SP/vb-ro$r.log 2>&1)
  v=$(grep -c "claimed at" $SP/vb-ro$r.log)
  echo "  run$r ok=$(grep -c '^ok' $SP/vb-ro$r.log) viol=$v"
  [ "$v" -gt 0 ] && fail=1
done

echo "=== (2) TRACE BAR: stale-dedupe census, root-only (build-btrace2) ==="
SESS=ftvb; rm -rf $SP/snapvb; mkdir -p $SP/snapvb
lttng destroy $SESS >/dev/null 2>&1
lttng create $SESS --snapshot --output $SP/snapvb >/dev/null 2>&1
lttng enable-channel -u --overwrite --subbuf-size 64K --num-subbuf 4 fb >/dev/null 2>&1
lttng enable-event -u -c fb 'cds_ft:stamp_*' >/dev/null 2>&1
lttng add-context -u -c fb -t vpid -t vtid >/dev/null 2>&1
lttng start $SESS >/dev/null 2>&1
cd $W/build-btrace2/tests/regression || exit 1
(ulimit -v 60000000; FT_TRACE_SESSION=$SESS FT_INV_MW=1 CDS_FT_LOCK_SPACING=root-only \
   timeout 900 ./test_urcu_ft_inv >$SP/vb-trace.log 2>&1) &
p=$!; sleep 120; lttng snapshot record -s $SESS >/dev/null 2>&1; wait $p
babeltrace2 "$(ls -d $SP/snapvb/snapshot-1-* | tail -1)" > $SP/vb-trace.txt 2>/dev/null
python3 $SP/stale.py $SP/vb-trace.txt | tee $SP/vb-census.txt
grep -q "FOREIGN take : 0 " $SP/vb-census.txt || { echo "  !! suspects remain"; fail=1; }
lttng destroy $SESS >/dev/null 2>&1

echo "=== (3) GREEN LEGS (build-e3): inv+unit x 3 spacings, MW per-node ==="
cd $W/build-e3/tests/regression || exit 1
for sp in per-node exponential root-only; do
  (ulimit -v 60000000; CDS_FT_LOCK_SPACING=$sp timeout 1200 ./test_urcu_ft_inv \
     >$SP/vb-inv-$sp.log 2>&1); echo "  inv $sp ok=$(grep -c '^ok' $SP/vb-inv-$sp.log)"
done
(ulimit -v 60000000; FT_INV_MW=1 CDS_FT_LOCK_SPACING=per-node timeout 1200 \
   ./test_urcu_ft_inv >$SP/vb-mwpn.log 2>&1); echo "  inv MW per-node ok=$(grep -c '^ok' $SP/vb-mwpn.log)"
cd $W/build-e3/tests/unit || exit 1
for sp in per-node exponential root-only; do
  (ulimit -v 60000000; CDS_FT_LOCK_SPACING=$sp timeout 1800 ./test_urcu_ft_unit \
     >$SP/vb-unit-$sp.log 2>&1)
  echo "  unit $sp ok=$(grep -c '^ok' $SP/vb-unit-$sp.log) notok=$(grep -c '^not ok' $SP/vb-unit-$sp.log)"
done
echo "=== RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL) ==="
echo "Then: red control (build-e2red, expect abort ~0.6s) + full gate per-leg diff vs gate-fa.log"
