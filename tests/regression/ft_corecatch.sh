#!/bin/bash
#
# ft_corecatch.sh -- run one fractal-trie TAP suite in a loop until it
# CRASHES, and KEEP THE CORE.
#
# Why this is a separate tool from ft_parallel_gate.sh:
#   The gate COUNTS aborts (abrt=N in its per-leg line) and then throws the
#   evidence away.  A count tells you a config is red; it does not tell you
#   WHICH slot, which writer, or which arm -- and the two instances of the
#   raw-read-of-a-parked-slot class closed so far were both root-caused from
#   a CORE, after gdb failed to reproduce either one under a debugger (the
#   window closes when you single-step it).  So the gate answers "is it
#   red", and this answers "why".
#
# ★ IT POSITIVE-CONTROLS ITSELF BEFORE IT HUNTS.
#   "0 cores in 96 runs" is a claim about the HARNESS before it is a claim
#   about the code, and every ingredient below can silently make it
#   unfalsifiable: RLIMIT_CORE defaults to 0 on most shells, and a
#   core_pattern that pipes to a collector (systemd-coredump, apport) writes
#   NOTHING into the cwd no matter what the limit says.  A hunt that cannot
#   catch a core reports the same clean sweep as a codebase with no bug, so
#   this refuses to start until a deliberate SIGABRT has produced a core file
#   under the very conditions the iterations will run in.  Override with -F
#   only if you have decided you want the red-rate count alone.
#
# Traps this already handles -- a reimplementation will hit them again:
#   * core_uses_pid=1 (the common default) names the file core.PID, not
#     "core".  A catcher globbing exactly "core" reports EVERY run clean.
#   * one cwd PER ITERATION: core_pattern here is a bare relative "core", so
#     concurrent iterations sharing a directory overwrite each other's
#     evidence -- and with -p >1 that is the normal case, not the corner one.
#   * run the real ELF out of .libs.  tests/*/test_urcu_ft_* are libtool
#     WRAPPER SCRIPTS; the core they leave behind belongs to the wrapper's
#     shell, and gdb then has no symbols for the crash.
#   * ulimit -c unlimited must be set INSIDE the iteration subshell.
#
# Usage:
#   tests/regression/ft_corecatch.sh [-b BUILD] [-t SUITE] [-s SPACING]
#                                    [-f FILTER] [-n N] [-p P] [-o OUT]
#                                    [-T SECS] [-k] [-F]
#
#   -b BUILD   build directory (default: the repo root -- an in-tree build).
#              A gate per-config tree works as-is and is the usual target:
#              -b ${TMPDIR:-/tmp}/ft-parallel-gate-$UID/anchorval
#   -t SUITE   inv (default) | unit
#   -s SPACING per-node (default) | exponential | root-only
#   -f FILTER  test-name filter, passed as argv[1] to the suite (both
#              suites take one); default runs everything
#   -n N       iterations (default 48)
#   -p P       concurrent iterations (default nproc/6 -- see below)
#   -o OUT     directory for the kept iterations (default ./ft-corecatch-OUT)
#   -T SECS    per-iteration timeout (default 1800)
#   -k         keep GREEN iteration dirs too (default: delete them, so a
#              1000-iteration hunt does not fill the disk with passing logs)
#   -F         hunt even if the self-test says a core cannot be caught
#
# Env: FT_INV_MW and FT_INV_NO_ORDERED_LIST are passed through to the suite.
#
# ★ Sizing -p: ft_inv creates one call_rcu worker per CPU, but those workers
# run co-located with the workload thread that enqueued the free, so they are
# not independent CPU demand.  The real footprint is the WORKLOAD threads.  Do
# NOT size from the ~nproc total thread count -- that reads as a 7x overcommit
# that is not there.
#   default oracles   4 readers + 2 writers  ~6   -> nproc/6
#   FT_INV_MW=1       MW_NR_WRITERS == 16    ~20  -> nproc/20
# so the default -p halves itself when FT_INV_MW is set; an explicit -p wins.
#
# Exit status: 0 if no iteration went red, 1 if any did, 2 on a usage or
# self-test error.
#
# SPDX-License-Identifier: LGPL-2.1-or-later

set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
NCPU=$( (nproc 2>/dev/null || echo 8) )

BUILD=$ROOT
SUITE=inv
SPACING=per-node
FILTER=
N=48
# Per-workload-thread sizing; see the -p note above.  An explicit -p overrides.
if [ -n "${FT_INV_MW:-}" ] && [ "${FT_INV_MW:-0}" != 0 ]; then
	P=$(( NCPU/20 > 1 ? NCPU/20 : 1 ))
else
	P=$(( NCPU/6 > 1 ? NCPU/6 : 1 ))
fi
OUT=
TMO=1800
KEEP_GREEN=0
FORCE=0

usage() { sed -n '/^# Usage:/,/^# Exit status/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while getopts 'b:t:s:f:n:p:o:T:kFh' o; do
	case $o in
	b) BUILD=$OPTARG ;;
	t) SUITE=$OPTARG ;;
	s) SPACING=$OPTARG ;;
	f) FILTER=$OPTARG ;;
	n) N=$OPTARG ;;
	p) P=$OPTARG ;;
	o) OUT=$OPTARG ;;
	T) TMO=$OPTARG ;;
	k) KEEP_GREEN=1 ;;
	F) FORCE=1 ;;
	h) usage; exit 0 ;;
	*) usage >&2; exit 2 ;;
	esac
done

case $SUITE in
inv)  REL=tests/regression/.libs/test_urcu_ft_inv ;;
unit) REL=tests/unit/.libs/test_urcu_ft_unit ;;
*) echo "ft_corecatch: unknown suite '$SUITE' (want inv or unit)" >&2; exit 2 ;;
esac
case $SPACING in
per-node|exponential|root-only) ;;
*) echo "ft_corecatch: unknown spacing '$SPACING'" >&2; exit 2 ;;
esac

BIN=$(readlink -f "$BUILD/$REL" 2>/dev/null)
LIBS=$(readlink -f "$BUILD/src/.libs" 2>/dev/null)
[ -n "$BIN" ] && [ -x "$BIN" ] || { echo "ft_corecatch: no test binary at $BUILD/$REL (build it first)" >&2; exit 2; }
[ -n "$LIBS" ] && [ -d "$LIBS" ] || { echo "ft_corecatch: no library dir at $BUILD/src/.libs" >&2; exit 2; }

# ★ A SPACING THE BUILD CANNOT SELECT IS A SWEEP THAT DOES NOT HAPPEN.
# CDS_FT_LOCK_SPACING is only read when FEATURE_FT_LOCK_SPACING_ENV is
# compiled in (FEATURE_FT_ANCHOR_VALIDATE / FEATURE_FT_HOLD_TRACE imply it),
# and without it every iteration silently runs per-node while the output says
# otherwise -- which is how an exponential-only defect scored 0 in the
# configs that "swept" for it.  Ask the ARTIFACT, not the configure line: the
# getenv() argument is a string literal, so it exists in the built library if
# and only if the arm that reads it was compiled.
# Look in the shared library AND in the test binary: a --disable-shared tree
# has no liburcu-cds.so at all, and there the literal lives in the statically
# linked binary instead.  Refusing THAT build would be the same false negative
# in the other direction.
if [ "$SPACING" != per-node ]; then
	if ! command -v strings >/dev/null 2>&1; then
		echo "ft_corecatch: no 'strings' (binutils) -- cannot verify that this" >&2
		echo "  build reads CDS_FT_LOCK_SPACING.  Proceeding UNVERIFIED: if it" >&2
		echo "  does not, every iteration runs per-node under an '$SPACING' label." >&2
	elif ! strings "$LIBS"/liburcu-cds.so* "$BIN" 2>/dev/null \
			| grep -qx 'CDS_FT_LOCK_SPACING'; then
		echo "ft_corecatch: $BUILD cannot select '$SPACING' -- neither its" >&2
		echo "  liburcu-cds nor its test binary reads CDS_FT_LOCK_SPACING, so" >&2
		echo "  every iteration would run per-node under an '$SPACING' label." >&2
		echo "  Rebuild that tree with -DFEATURE_FT_ANCHOR_VALIDATE." >&2
		exit 2
	fi
fi

[ -n "$OUT" ] || OUT=./ft-corecatch-$SUITE-$SPACING-$$
mkdir -p "$OUT" || exit 2
OUT=$(readlink -f "$OUT")

# ---------------------------------------------------------------- self-test
# Deliberately dump a core under the iterations' own conditions, and refuse
# the hunt if none lands.  bash SIGABRTing itself needs no compiler and no
# fixture, and it exercises exactly the three things that break: the limit,
# the pattern, and the cwd.
CORE_PATTERN=$(cat /proc/sys/kernel/core_pattern 2>/dev/null || echo '?')
selftest() {
	local d=$OUT/.selftest rc
	rm -rf "$d"; mkdir -p "$d" || return 1
	# The trailing ':' matters.  Without it bash tail-EXECs the last command
	# of the subshell, so the process that dumps core is a direct child of
	# THIS script and this script prints "Aborted (core dumped)" -- to its
	# own stderr, which the redirection below cannot reach.  Keeping the
	# subshell alive makes the subshell the reporting parent, and its stderr
	# is /dev/null.
	( cd "$d" && ulimit -c unlimited 2>/dev/null; bash -c 'kill -ABRT $$'; : ) >/dev/null 2>&1
	if compgen -G "$d/core*" > /dev/null; then rc=0; else rc=1; fi
	rm -rf "$d"
	return $rc
}
if selftest; then
	echo "ft_corecatch: self-test ok -- a core lands in the iteration cwd."
else
	echo "ft_corecatch: SELF-TEST FAILED -- a deliberate SIGABRT produced no core." >&2
	echo "  core_pattern = $CORE_PATTERN" >&2
	case $CORE_PATTERN in
	'|'*)	echo "  That pattern PIPES cores to a collector, so nothing is ever" >&2
		echo "  written into the cwd.  Either query the collector instead" >&2
		echo "  (coredumpctl) or, as root:" >&2
		echo "    sysctl -w kernel.core_pattern=core kernel.core_uses_pid=1" >&2 ;;
	*)	echo "  Check the hard core limit ('ulimit -Hc'; a hard 0 cannot be" >&2
		echo "  raised by this script) and that $OUT is writable." >&2 ;;
	esac
	echo "  Refusing to hunt: without a core, a clean sweep here is a claim" >&2
	echo "  about this harness, not about the code.  Pass -F to count reds" >&2
	echo "  anyway." >&2
	[ "$FORCE" -eq 1 ] || exit 2
	echo "ft_corecatch: -F given, continuing WITHOUT core capture." >&2
fi

# ---------------------------------------------------------------- iteration
# Written out as a script rather than inlined into xargs so that a red
# iteration can be replayed by hand, unchanged: "$OUT/one.sh 7".
cat > "$OUT/one.sh" <<EOF
#!/bin/bash
# One ft_corecatch iteration.  Replay by hand: $OUT/one.sh <n>
i=\$1
d=$OUT/it-\$i
mkdir -p "\$d" || exit 1
cd "\$d" || exit 1
ulimit -c unlimited 2>/dev/null
export LD_LIBRARY_PATH="$LIBS\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export CDS_FT_LOCK_SPACING=$SPACING
export FT_INV_MW=${FT_INV_MW:-0}
export FT_INV_NO_ORDERED_LIST=${FT_INV_NO_ORDERED_LIST:-0}
# Park this shell's stderr while the suite runs.  A crash is the EXPECTED
# outcome here, and bash announces every one of them as "Aborted (core
# dumped)" on the harness's own stderr -- thousands of lines in a long hunt,
# each of which reads like the harness itself failing.  The suite's stderr
# still goes to the log; only the job-status line is dropped.
exec 3>&2 2>/dev/null
timeout $TMO "$BIN" $FILTER > log 2>&1
rc=\$?
exec 2>&3 3>&-
# A red is any of: a core, a non-zero exit (124 == the timeout killed a
# hang), or a TAP failure.  Report the first line that says why, so the
# hits file is triageable without opening every log.
if compgen -G "core*" > /dev/null; then
	echo "\$i CORE rc=\$rc \$(grep -m1 -E 'Assertion|Segmentation|not ok ' log)"
elif [ \$rc -ne 0 ] || grep -q '^not ok ' log; then
	echo "\$i FAIL rc=\$rc \$(grep -m1 -E 'Assertion|Segmentation|^not ok ' log)"
elif [ $KEEP_GREEN -eq 0 ]; then
	cd /; rm -rf "\$d"
fi
EOF
chmod +x "$OUT/one.sh"

echo "ft_corecatch: $SUITE ${FILTER:-<whole suite>} @ $SPACING, ${N} runs x${P}"
echo "  build $BUILD"
echo "  out   $OUT"
seq 1 "$N" | xargs -P "$P" -n1 "$OUT/one.sh" > "$OUT/hits.txt"

nred=$(grep -c . "$OUT/hits.txt")
ncore=$(grep -c ' CORE ' "$OUT/hits.txt")
printf 'CORECATCH %s %s %s: %d/%d red (%d with a core)\n' \
	"$(basename "$BUILD")" "$SPACING" "${FILTER:--}" "$nred" "$N" "$ncore" \
	| tee "$OUT/verdict.txt"
if [ "$nred" -gt 0 ]; then
	head -20 "$OUT/hits.txt"
	if [ "$ncore" -gt 0 ]; then
		c=$(ls "$OUT"/it-*/core* 2>/dev/null | head -1)
		echo "open it:  LD_LIBRARY_PATH=$LIBS gdb $BIN $c"
	fi
	exit 1
fi
exit 0
