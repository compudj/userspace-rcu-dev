#!/bin/bash
# Finding-B fix verification.  Three independent bars:
#   (1) the ORACLE bar    -- root-only MW must stop aborting (N runs, was ~1 in 2)
#   (2) the DETECTOR bar  -- the two in-code report-only detectors, "FT EXTRAS
#                            STALE" (frame extras) and "FT GLUE STALE" (the glue
#                            hand-offs), must be SILENT.
#   (3) the CONTROL bar   -- the SAME legs on build-ctlB (-DFT_SCRUB_OFF) must
#                            FAIL.  A silent detector proves nothing until the
#                            build with the mechanism removed makes it speak.
# plus the standing green legs, and stale.py as a COMPARATIVE census only (it
# has verified false positives -- see its header).
# Usage: verify_b.sh [runs]
set -u
SP="$(dirname "$0")"; RUNS="${1:-6}"; W=/mnt/data/efficios/git/userspace-rcu
fail=0

# ☠ (0) THE MECHANISM GATE.  Every bar below reads a GREP COUNT, and a grep for
# a string the binary does not contain returns 0 -- indistinguishable from a
# clean run.  A stale build directory therefore turns the whole harness
# vacuously green.  Prove the strings are IN the library, in BOTH builds,
# before any zero is allowed to mean anything.
echo "=== (0) MECHANISM GATE: the detectors must be IN both libraries ==="
for b in build-e3 build-ctlB; do
  lib=$W/$b/src/.libs/liburcu-cds.so.8.2.0
  for sym in "FT EXCLUSION VIOLATION" "FT EXTRAS STALE" "FT GLUE STALE"; do
    if strings "$lib" 2>/dev/null | grep -qF "$sym"; then
      echo "  $b: $sym present"
    else
      echo "  !! $b: $sym ABSENT -- every grep for it is a FALSE ZERO"; fail=1
    fi
  done
done

echo "=== (1) ORACLE BAR: root-only MW x $RUNS (build-e3) ==="
cd $W/build-e3/tests/regression || exit 1
for r in $(seq 1 $RUNS); do
  (ulimit -v 60000000; FT_INV_MW=1 CDS_FT_LOCK_SPACING=root-only timeout 900 \
     ./test_urcu_ft_inv >$SP/vb-ro$r.log 2>&1)
  v=$(grep -c "claimed at" $SP/vb-ro$r.log)
  g=$(grep -c "GLUE STALE" $SP/vb-ro$r.log)
  x=$(grep -c "EXTRAS STALE" $SP/vb-ro$r.log)
  n=$(grep -c '^ok' $SP/vb-ro$r.log)
  cap=$(grep -c 'report cap reached' $SP/vb-ro$r.log)
  echo "  run$r ok=$n viol=$v gluestale=$g extrastale=$x capped_threads=$cap"
  # ☠ COMPLETION IS PART OF THE BAR.  An over-silencing scrub does not print a
  # violation -- it makes the op RE-ACQUIRE a word it holds and hard-refuse, so
  # the run dies EARLY and every grep above reads a clean 0.
  [ "$n" = 119 ] || { echo "    !! did not complete (ok=$n, want 119)"; fail=1; }
  [ "$v" -gt 0 -o "$g" -gt 0 -o "$x" -gt 0 ] && fail=1
  # ☠ AND THE DETECTOR ZEROS ARE ONLY READABLE UNDER AN UNSPENT BUDGET.  The
  # per-thread report cap is SHARED with the routine FT REFUSED contention
  # line: measured, ~52k of those silence ~270 threads per root-only MW run,
  # after which "extrastale=0" says nothing at all.
  [ "$cap" -gt 0 ] && echo "    ~ $cap threads hit the report cap: the STALE" \
    "zeros above are NOT evidence for those threads"
done

echo "=== (1b) CONTROL BAR: the same legs with the scrub REMOVED (build-ctlB) ==="
echo "    a green arm is only evidence if THIS one is red."
cd $W/build-ctlB/tests/regression || exit 1
ctlhit=0
for r in $(seq 1 $RUNS); do
  (ulimit -v 60000000; FT_INV_MW=1 CDS_FT_LOCK_SPACING=root-only timeout 900 \
     ./test_urcu_ft_inv >$SP/vb-ctl$r.log 2>&1)
  v=$(grep -c "claimed at" $SP/vb-ctl$r.log)
  x=$(grep -c "EXTRAS STALE" $SP/vb-ctl$r.log)
  echo "  ctl$r ok=$(grep -c '^ok' $SP/vb-ctl$r.log) viol=$v extrastale=$x"
  [ "$v" -gt 0 -o "$x" -gt 0 ] && ctlhit=$((ctlhit + 1))
done
echo "  control runs that reproduced: $ctlhit / $RUNS (0 means the bar is BLIND)"
[ "$ctlhit" = 0 ] && fail=1

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
n=$(grep -c "EXTRAS STALE" $SP/vb-trace.log)
echo "  in-code detector hits: EXTRAS STALE $n (must be 0)"
[ "$n" = 0 ] || fail=1
# ☠ NO GLUE STALE GREP HERE.  build-btrace2's library predates that detector
# and does not contain the string, so the grep could only ever return a FALSE
# ZERO.  Re-add it only once (0) above covers build-btrace2 too.
lttng destroy $SESS >/dev/null 2>&1

echo "=== (3) GREEN LEGS (build-e3): inv+unit x 3 spacings, MW per-node ==="
cd $W/build-e3/tests/regression || exit 1
for sp in per-node exponential root-only; do
  (ulimit -v 60000000; CDS_FT_LOCK_SPACING=$sp timeout 1200 ./test_urcu_ft_inv \
     >$SP/vb-inv-$sp.log 2>&1)
  echo "  inv $sp ok=$(grep -c '^ok' $SP/vb-inv-$sp.log)" \
       "gluestale=$(grep -c 'GLUE STALE' $SP/vb-inv-$sp.log)" \
       "extrastale=$(grep -c 'EXTRAS STALE' $SP/vb-inv-$sp.log)"
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
