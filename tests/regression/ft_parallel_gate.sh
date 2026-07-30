#!/bin/bash
#
# ft_parallel_gate.sh -- build + run the fractal-trie TAP suites
# (test_urcu_ft_unit and test_urcu_ft_inv) across every feature-flag
# configuration CONCURRENTLY, as a pre-commit gate.
#
# Why a separate tree per config:
#   The feature flags (verify-at-mutation, skip-compress off, compress
#   off, fault-inject + tombstone-audit, insert-in-place) are compile
#   time -D defines.  A single in-tree build can only hold one set at a
#   time (and an out-of-tree build is refused while the source is
#   configured), so a sequential "clean; rebuild; test" per config
#   serializes the whole matrix behind the slowest leg (VAM).  Instead
#   we keep one CONFIGURED copy of the source tree per config under
#   $FT_GATE_DIR and build+test them all at once.
#
#   The flags are applied at CONFIGURE time (CPPFLAGS=... ./configure),
#   not via a make-time `make CPPFLAGS=` override: the override reaches
#   the library but NOT the test-side #ifdefs (e.g. VAM's skip of the
#   O(keys^2) test_compact_dense_full_node), which would silently build
#   the tests as default and mis-report coverage.
#
# Usage:
#   tests/regression/ft_parallel_gate.sh [config ...]
#     no args   -- run the full matrix
#     config... -- run only the named configs (default fault-audit vam
#                  noskip nocompress in-place)
#
# Env:
#   FT_GATE_DIR  per-config tree copies live here   (default:
#                ${TMPDIR:-/tmp}/ft-parallel-gate-$UID -- OUTSIDE the
#                repo so the GB of copies never pollute git status)
#   FT_GATE_J    make -j per config                 (default: cores/12,
#                so nconfigs*J stays near the core count)
#
# Requires a bootstrapped source (./configure present -- run
# ./bootstrap first on a fresh clone).  Exit status is non-zero if any
# config fails to build or reports a failing/aborted test.
#
# SPDX-License-Identifier: LGPL-2.1-or-later

set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
GATE=${FT_GATE_DIR:-${TMPDIR:-/tmp}/ft-parallel-gate-$(id -u)}
NCPU=$( (nproc 2>/dev/null || echo 8) )
J=${FT_GATE_J:-$(( NCPU/12 > 3 ? NCPU/12 : 4 ))}
mkdir -p "$GATE"

if [ ! -x "$ROOT/configure" ]; then
	echo "ft_parallel_gate: $ROOT/configure not found -- run ./bootstrap first." >&2
	exit 2
fi

# name | configure-time CPPFLAGS | tests
#   u=ft_unit  ion=ft_inv-on  ioff=ft_inv-off  imw=ft_inv-on + FT_INV_MW=1
ALL_CONFIGS=(
	# DLM (per-node lock-sets) is no longer a build flag: it is the ONLY
	# multi-writer implementation, selected at RUNTIME by ft->lock_fine
	# (the writer strategy).  The former "dlm" / "dlm-fault" configs folded
	# into "default" / "fault-audit" -- every config now exercises the one
	# MW implementation, so a failure no longer has to be attributed to a
	# build mode first.  "default" keeps imw because the concurrent-writer
	# and rekey oracles gate on FT_INV_MW at RUNTIME: without it they
	# compile in and then skip, which reads as coverage and is not.
	"default||u ion ioff imw"
	# Fault injection drives the acquire bail + re-descend paths: those
	# acquires never miss single-threaded, and the concurrent oracles merge
	# DISJOINT key sets, so injection is the only thing that reaches them.
	"fault-audit|-DFEATURE_FT_FAULT_INJECT -DFT_DEBUG_TOMBSTONE_AUDIT|u ioff"
	"audit|-DFT_DEBUG_TOMBSTONE_AUDIT|u ion ioff"
	"vam|-DFEATURE_FT_VERIFY_AT_MUTATION|u"
	"noskip|-DNO_FEATURE_FT_SKIP_COMPRESSED|u ioff"
	"nocompress|-DNO_FEATURE_FT_COMPRESS|u ioff"
	# Concurrent legs are SAFE here since the in-place tier became runtime
	# gated on ft->exclusive: on a shared trie every one of these builds
	# takes the recompact path, so this config now checks that the opt-in
	# flag changes nothing a concurrent trie can observe.  Before that gate
	# it failed 303 of 480 saturated runs of inv_sibling_split_compress
	# ("ft_attach_node: Assertion `slot_ptr'"), because the flag alone
	# decided and an in-place store on a LIVE node races a peer's rebuild.
	#
	# What these legs do NOT cover is the in-place path itself: with the
	# gate, ft_unit takes it 2255 times against 204M refusals, because most
	# tries in the suite are not exclusive.  Exercising the fast path
	# meaningfully needs exclusive-trie mutation coverage, which is its own
	# piece of work.
	"in-place|-DFEATURE_FT_INSERT_IN_PLACE|u ion ioff"
	# The byte-key-only build (~25 KiB less .text): compiles out the
	# non-identity key-map lookup specializations, after which
	# cds_ft_group_attr_set_key_map returns NOT_SUPPORTED.  It had no gate
	# config, so that state was untested by construction.
	"nokeymap|-DNO_FEATURE_FT_KEY_MAP|u ion ioff"
	# The access-discipline validator.  Its writer/writer check is now
	# MODE-AWARE, so it runs against the concurrent oracles too: a FINE trie
	# counts writers instead of claiming a single owner, because disjoint
	# writers running in parallel is the design there, not a violation.
	"excl|-DFEATURE_FT_EXCL_VALIDATE|u ion ioff imw"
	# -DNO_FEATURE_FT_MERGE compiles out the merge subsystem (~20 KiB .text;
	# cds_ft_merge then returns NOT_SUPPORTED).  It had no gate config and had
	# ROTTED: the rekey fold's occupied-dst arm -- which IS a merge -- was not
	# guarded, so the library did not compile at all (9 errors).
	#
	# It was BUILD-ONLY until the 48 ft_unit tests that exercise merge
	# directly could tell "compiled out" from "broken": they now ask
	# cds_ft_merge_enabled() and skip, so the config runs the suite (295,
	# 48 of them skips) instead of only proving it links.  Compiling is a
	# weak claim about a configuration -- the rot above got in while this
	# config compiled fine.
	"nomerge|-DNO_FEATURE_FT_MERGE|u"
)

# Optional positional filter: run only the named configs.
CONFIGS=()
if [ "$#" -gt 0 ]; then
	for want in "$@"; do
		for c in "${ALL_CONFIGS[@]}"; do
			[ "${c%%|*}" = "$want" ] && CONFIGS+=("$c")
		done
	done
	[ "${#CONFIGS[@]}" -eq 0 ] && { echo "no matching configs in: $*" >&2; exit 2; }
else
	CONFIGS=("${ALL_CONFIGS[@]}")
fi

setup_tree() {	# $1=name $2=cppflags -- one-time: copy source + configure WITH flags
	local dir=$GATE/$1 flags=$2
	if [ -f "$dir/config.status" ] && \
	   grep -qxF "CPPFLAGS=$flags" "$dir/.gate_flags" 2>/dev/null; then
		return 0	# already configured with these exact flags
	fi
	rm -rf "$dir"; mkdir -p "$dir"
	rsync -a --exclude='.git' --exclude='build-*' --exclude='ft-parallel-gate-*' \
		"$ROOT/"  "$dir/" 2>/dev/null
	( cd "$dir" && make distclean >/dev/null 2>&1
	  CPPFLAGS="$flags" ./configure --quiet >/dev/null 2>&1 )
	printf 'CPPFLAGS=%s\n' "$flags" > "$dir/.gate_flags"
	[ -f "$dir/config.status" ] && echo "  setup $1 ok" || echo "  setup $1 FAILED"
}

sync_src() {	# $1=name -- refresh live sources into the (already-configured) tree
	local dir=$GATE/$1
	rsync -a --delete "$ROOT/src/fractal-trie/" "$dir/src/fractal-trie/" 2>/dev/null
	rsync -a "$ROOT/src/"     "$dir/src/"     --exclude='.libs' --exclude='*.o' --exclude='*.lo' 2>/dev/null
	rsync -a "$ROOT/tests/"   "$dir/tests/"   --exclude='.libs' --exclude='*.o' --exclude='*.lo' 2>/dev/null
	rsync -a "$ROOT/include/" "$dir/include/" 2>/dev/null
}

run_one() {	# $1=name $2=tests -- build lib+tests, run the TAP suites
	local name=$1 tests=$2
	local dir=$GATE/$name out=$GATE/$name.result
	local LIB=$dir/src/.libs U=$dir/tests/unit/.libs/test_urcu_ft_unit
	local I=$dir/tests/regression/.libs/test_urcu_ft_inv
	: > "$out"
	if ! make -C "$dir/src" -j"$J" >"$GATE/$name.build" 2>&1; then
		echo "$name: BUILD FAIL (see $GATE/$name.build)" >> "$out"; return
	fi
	# libtap et al must exist first, else the test binaries fail to link
	# silently and every suite reports ok=0 (a false PASS -- see the ok=0
	# gate below).
	make -C "$dir/tests/utils"      -j"$J" >>"$GATE/$name.build" 2>&1
	# The TEST binaries' build status is checked, not just the library's: a
	# test source that fails to compile ONLY in this config (an #ifdef'd
	# region the default build never sees) would otherwise leave the previous
	# run's binary in place, and the suite would report that STALE binary's
	# results as if they were this config's -- a false PASS, or a false FAIL
	# attributed to the wrong change.  Both happened before this check.
	if ! make -C "$dir/tests/unit" test_urcu_ft_unit -j"$J" \
			>>"$GATE/$name.build" 2>&1; then
		echo "$name: TEST BUILD FAIL (ft_unit; see $GATE/$name.build)" >> "$out"
		return
	fi
	if ! make -C "$dir/tests/regression" test_urcu_ft_inv -j"$J" \
			>>"$GATE/$name.build" 2>&1; then
		echo "$name: TEST BUILD FAIL (ft_inv; see $GATE/$name.build)" >> "$out"
		return
	fi
	echo "$name: build ok" >> "$out"
	local t o ok notok abrt lbl rc plan ran
	for t in $tests; do
		case $t in
		u)    o=$(LD_LIBRARY_PATH=$LIB timeout 900 "$U" 2>&1); rc=$?
		      lbl="ft_unit   ";;
		ion)  o=$(LD_LIBRARY_PATH=$LIB timeout 300 "$I" 2>&1); rc=$?
		      lbl="ft_inv on ";;
		ioff) o=$(LD_LIBRARY_PATH=$LIB FT_INV_NO_ORDERED_LIST=1 timeout 300 "$I" 2>&1); rc=$?
		      lbl="ft_inv off";;
		# The concurrent-writer oracles (MW writers, coherent rekey) all
		# gate on FT_INV_MW at runtime and are otherwise skipped, so ion
		# / ioff never exercise them.  They are the long leg -- hence the
		# larger timeout.
		imw)  o=$(LD_LIBRARY_PATH=$LIB FT_INV_MW=1 timeout 1800 "$I" 2>&1); rc=$?
		      lbl="ft_inv mw ";;
		esac
		ok=$(printf '%s' "$o" | grep -c '^ok ')
		notok=$(printf '%s' "$o" | grep -c '^not ok ')
		abrt=$(printf '%s' "$o" | grep -c -i 'assert')
		printf '  [%-11s] %s ok=%s notok=%s abrt=%s\n' "$name" "$lbl" "$ok" "$notok" "$abrt" >> "$out"
		# NAME the failures, do not just count them.  An INTERMITTENT red is
		# unattributable from a count alone: you cannot tell a known flake
		# from a new regression without re-running and hoping it recurs.
		# (Cost real time once: a fault-audit red that never reproduced.)
		if [ "$notok" -gt 0 ]; then
			printf '%s' "$o" | grep '^not ok ' | sed "s/^/      [$name] $lbl /" >> "$out"
		fi
		# A HUNG suite used to score GREEN.  The run is killed, its PARTIAL
		# ok-count is reported, and with no 'not ok ' line and no abort the
		# config passed -- so a single-threaded DLM self-deadlock rode in on
		# a "GATE PASS" while dlm's ft_unit silently fell from 290 tests to
		# 36.  The count drop was the only signal and nothing checked it.
		# Two independent checks now, because each catches what the other
		# misses: a non-zero exit (124 == timeout kill, or a crash after the
		# last emitted line), and a run that never reached its TAP plan
		# (self-maintaining -- the plan is the suite's own test count, so
		# adding tests needs no gate update).
		# Most-specific diagnosis first: a timeout and a plan shortfall have
		# very different causes (hang vs retired-but-still-planned test), and
		# libtap's exit_status() is planned-minus-run, so a bare "exit 3"
		# would report the shortfall in its least legible form.
		plan=$(printf '%s' "$o" | sed -n 's/^1\.\.\([0-9]\{1,\}\)$/\1/p' | head -1)
		ran=$((ok + notok))
		if [ "$rc" -eq 124 ]; then
			echo "$name: $lbl INCOMPLETE (TIMEOUT/hang after $ran tests)" >> "$out"
		elif [ "$rc" -gt 128 ]; then
			echo "$name: $lbl INCOMPLETE (killed by signal $((rc - 128)) after $ran tests)" >> "$out"
		elif [ -z "$plan" ]; then
			echo "$name: $lbl NO TAP PLAN (completeness unverifiable)" >> "$out"
		elif [ "$ran" -ne "$plan" ]; then
			echo "$name: $lbl INCOMPLETE (ran $ran of $plan planned)" >> "$out"
		elif [ "$rc" -ne 0 ]; then
			echo "$name: $lbl NONZERO EXIT ($rc) with a complete run" >> "$out"
		fi
	done
}

echo "gate dir : $GATE   (-j$J x ${#CONFIGS[@]} configs on $NCPU cores)"
echo "=== [1/3] configure per-config trees (parallel; one-time) ==="
for c in "${CONFIGS[@]}"; do IFS='|' read -r n f _ <<< "$c"; setup_tree "$n" "$f" & done; wait
echo "=== [2/3] sync live sources (parallel) ==="
for c in "${CONFIGS[@]}"; do sync_src "${c%%|*}" & done; wait
echo "=== [3/3] build + test (parallel) ==="
for c in "${CONFIGS[@]}"; do IFS='|' read -r n _ t <<< "$c"; run_one "$n" "$t" & done; wait

echo "=== RESULTS ==="
rc=0
for c in "${CONFIGS[@]}"; do
	n=${c%%|*}
	if [ ! -s "$GATE/$n.result" ]; then
		echo "  [$n] NO RESULT (harness error -- see $GATE/$n.build)"; rc=1; continue
	fi
	cat "$GATE/$n.result"
	# Fail on a bad build, any failing/aborted test, or a missing 'build ok'.
	# Also fail on ok=0: a test suite that produced zero 'ok' lines built
	# but never ran (missing/unlinked binary, e.g. tests/utils not built) --
	# notok/abrt alone would let that pass as a false GREEN.
	if ! grep -q 'build ok' "$GATE/$n.result" || \
	   grep -qE 'BUILD FAIL|notok=[1-9]|abrt=[1-9]|ok=0 notok=|INCOMPLETE|NO TAP PLAN' "$GATE/$n.result"; then
		rc=1
	fi
done
if [ "$rc" -eq 0 ]; then echo "=== GATE PASS ==="; else echo "=== GATE FAIL ==="; fi
exit "$rc"
