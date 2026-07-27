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
	"default||u ion ioff"
	"fault-audit|-DFEATURE_FT_FAULT_INJECT -DFT_DEBUG_TOMBSTONE_AUDIT|u ioff"
	"audit|-DFT_DEBUG_TOMBSTONE_AUDIT|u ion ioff"
	"vam|-DFEATURE_FT_VERIFY_AT_MUTATION|u"
	"noskip|-DNO_FEATURE_FT_SKIP_COMPRESSED|u ioff"
	"nocompress|-DNO_FEATURE_FT_COMPRESS|u ioff"
	"in-place|-DFEATURE_FT_INSERT_IN_PLACE|u"
	# The DLM (per-node lock-set) build.  Until this config existed, NOTHING
	# in the gate defined FEATURE_FT_MW_DLM_ACQUIRE -- unlike
	# FEATURE_FT_MW_LOCK_FINE_DROP it does not self-enable -- so the whole
	# mixed sw/mw subsystem (structural_sw, ft_flip_txn_record_tag's SW
	# branch, ft_flip_txn_record_tag_mw) and the six rekey-graft oracles
	# gated on it in ft_inv's main() were compiled out of every gate run.
	# It carries imw because those oracles ALSO gate on FT_INV_MW at
	# runtime: without it they would compile in and then skip, which reads
	# as coverage and is not.
	"dlm|-DFEATURE_FT_MW_DLM_ACQUIRE|u ion ioff imw"
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
	local t o ok notok abrt lbl
	for t in $tests; do
		case $t in
		u)    o=$(LD_LIBRARY_PATH=$LIB timeout 900 "$U" 2>&1)
		      lbl="ft_unit   ";;
		ion)  o=$(LD_LIBRARY_PATH=$LIB timeout 300 "$I" 2>&1)
		      lbl="ft_inv on ";;
		ioff) o=$(LD_LIBRARY_PATH=$LIB FT_INV_NO_ORDERED_LIST=1 timeout 300 "$I" 2>&1)
		      lbl="ft_inv off";;
		# The concurrent-writer oracles (MW writers, coherent rekey) all
		# gate on FT_INV_MW at runtime and are otherwise skipped, so ion
		# / ioff never exercise them.  They are the long leg -- hence the
		# larger timeout.
		imw)  o=$(LD_LIBRARY_PATH=$LIB FT_INV_MW=1 timeout 1800 "$I" 2>&1)
		      lbl="ft_inv mw ";;
		esac
		ok=$(printf '%s' "$o" | grep -c '^ok ')
		notok=$(printf '%s' "$o" | grep -c '^not ok ')
		abrt=$(printf '%s' "$o" | grep -c -i 'assert')
		printf '  [%-11s] %s ok=%s notok=%s abrt=%s\n' "$name" "$lbl" "$ok" "$notok" "$abrt" >> "$out"
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
	   grep -qE 'BUILD FAIL|notok=[1-9]|abrt=[1-9]|ok=0 notok=' "$GATE/$n.result"; then
		rc=1
	fi
done
if [ "$rc" -eq 0 ]; then echo "=== GATE PASS ==="; else echo "=== GATE FAIL ==="; fi
exit "$rc"
