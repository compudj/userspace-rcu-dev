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
#   FT_GATE_SPACINGS  lock spacings to run every config at (default
#                per-node).  The two DETECTOR configs (txndbg,
#                proxyassert) carry their own 3-spacing sweep in the
#                matrix and ignore this default -- see the comment there
#                for why a defect can hide in the middle of that axis.
#   FT_GATE_REPEAT    run each leg N times (default 1).  For hunting an
#                INTERMITTENT: the two measured instances of the
#                raw-read-of-a-parked-slot class fired at ~10% and ~8%,
#                so one green leg is not evidence.  Multiplies the whole
#                matrix, hence opt-in.
#
#                ★ WHAT IT COSTS AND BUYS.  Measured 2026-08-11 on a
#                384-thread box, per-config trees already configured, with
#                the spacings running CONCURRENTLY (see run_one):
#                  * The full 14-config matrix is 383 s at N=1, and anchorval
#                    -- the only config that has ever reproduced that class --
#                    is 365 s of it, so the gate is still exactly as long as
#                    its longest swept config.
#                  * Repeats are SEQUENTIAL within a spacing while the three
#                    spacings are not, so each +1 adds one spacing's legs
#                    (~330 s), not three.  N=1 383 s, N=3 ~17 min,
#                    N=5 ~28 min.
#                  * The per-leg cost is flat across the swept axis (per-node
#                    328 s, exponential 325, root-only 328); the inv legs are
#                    83% of it (unit 55 s, ion 89, ioff 88, imw 95).
#                  * Detection per gate run: 3 inv invocations land on the
#                    ONE spacing that reproduces, so an 8%-per-run defect is
#                    seen 1-0.92^3 = 23% of the time at N=1 -- and 54% at
#                    N=3, 73% at N=5.
#                The standing default is 1: 23% per commit still surfaces a
#                NEWLY introduced defect within a few commits, and it now
#                keeps the CORE when it does (see run_one), which is what
#                makes a hit actionable rather than merely alarming.
#                ★ N=3 is now affordable in a way it was not before the
#                spacings were parallelised -- it costs ~17 min, which is
#                what N=1 cost when they ran one after another.  Raise it if
#                you would rather spend that; nothing else needs to change.
#                For a deliberate hunt prefer tests/regression/ft_corecatch.sh,
#                which repeats ONE suite with the evidence kept and no
#                build-matrix tax.
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
	"default||u ion ioff imw imwx"
	# Fault injection drives the acquire bail + re-descend paths: those
	# acquires never miss single-threaded, and the concurrent oracles merge
	# DISJOINT key sets, so injection is the only thing that reaches them.
	"fault-audit|-DFEATURE_FT_FAULT_INJECT -DFT_DEBUG_TOMBSTONE_AUDIT|u ioff"
	# FT_DEBUG_REKEY_RETRY_CAP rides here rather than buying a config of its
	# own: it costs one increment and one compare per rekey attempt, and this
	# config already runs the unit suite and both single-writer inv legs, which
	# is where every rekey shape the tests can reach gets driven.  A detector
	# compiled into no configuration is not coverage.
	"audit|-DFT_DEBUG_TOMBSTONE_AUDIT -DFT_DEBUG_REKEY_RETRY_CAP|u ion ioff"
	"vam|-DFEATURE_FT_VERIFY_AT_MUTATION|u"
	# The TRANSACTION ENGINE's own debug features.  Every other config
	# compiles them out, so an engine-contract violation the FT commits is
	# invisible to the whole matrix:
	#   DEBUG_RCU                 -> urcu_assert_debug.  A record value that
	#                                is itself a proxy (the embedder read the
	#                                slot raw instead of through
	#                                urcu_txn_load), two records on one slot,
	#                                an MW record on the sw-only commit, a
	#                                same-slot kind conflict.
	#   URCU_TXN_DEBUG_READ_POLICY-> a slot that enters the read/write set
	#                                after an OPTIMISTIC load.
	# The two are complementary: the read-policy table only marks slots
	# loaded through the engine's API, so a RAW C read of a transacted word
	# leaves no mark and only the assert catches it.
	#
	# imw matters MOST here, for the same reason it does on "default": the
	# writers that do the heaviest transacted-slot work -- the concurrent-writer
	# and rekey oracles -- gate on FT_INV_MW at RUNTIME, so without it the one
	# config that carries the engine's assertions checks only the SINGLE-writer
	# subset.  A proxy can only be another writer's, and an expected-old only
	# arbitrates against a peer, so the raw-read class this config exists to
	# detect is the class the missing leg hid.  The leg costs 82 s, alongside
	# ioff's 76 s.
	#
	# ★ SWEPT ACROSS LOCK SPACINGS (the 4th field), because the class this
	# config exists to detect HIDES IN ONE.  The gate ran every leg at the
	# default per-node and found nothing for months; the same suite under
	# CDS_FT_LOCK_SPACING=exponential produced a raw-read abort in 4 runs of
	# 48 (ft_chain_compress_fused's forward slot, @f8b1640e), and 0 of 48
	# under per-node AND root-only.  Per-node holds the slot's own word so no
	# peer can park in it; root-only serialises every op on one word; only
	# coarsening-to-an-ancestor leaves the slot open.  A defect can live in
	# the MIDDLE of this axis, so testing its two ends proves nothing about it.
	"txndbg|-DDEBUG_RCU -DURCU_TXN_DEBUG_READ_POLICY -DURCU_TXN_DEBUG_SETTLE -DFEATURE_FT_ANCHOR_VALIDATE|u ion ioff imw sp|per-node exponential root-only"
	# The FT's own resolved-pointer assertion (ft_assert_resolved): a parked
	# flip proxy handed to an accessor that requires a resolved flag.  It is
	# the embedder-side counterpart to txndbg's engine-side DEBUG_RCU, and it
	# had NO config at all -- the flag appeared nowhere but its own definition,
	# so every one of the accessors carrying it was asserting into a build
	# nobody made.
	#
	# imw for the same reason txndbg needs it, only more so: a proxy is BY
	# CONSTRUCTION another writer's, so without the MW leg this config asserts
	# over a workload that cannot produce the thing it detects.
	#
	# noskip is in the CFLAGS deliberately.  The class this catches is the raw
	# child-slot read, and skip-compression collapses the chains those reads
	# walk -- the free-walk defect that motivated the config reproduced 10 of
	# 10 without skip-compression and never with it.
	# Swept for the same reason txndbg is: this is the embedder-side detector
	# for the same class, so it is blind to the same spacings.
	"proxyassert|-DFT_DEBUG_PROXY_ASSERT -DNO_FEATURE_FT_SKIP_COMPRESSED -DFEATURE_FT_ANCHOR_VALIDATE|u ion ioff imw|per-node exponential root-only"
	# Phase E.3's certification config: the self-collision ledger
	# (FEATURE_FT_HOLD_TRACE) armed across the spacing sweep.  A collision
	# aborts (ft_hold_trace_refused), so a red here is a leg abort, not a
	# grep.  DEBUG_RCU rides along so the engine's own asserts stay armed at
	# the same spacings this config certifies.
	# ★ imw AT ALL THREE SPACINGS IS THE POINT OF THIS CONFIG.  The E.2
	# owner-stamp oracle only has two writers to arbitrate under MW, and
	# the exclusion defects it exists for live at the COARSE spacings: a
	# concurrent-writer leg at per-node alone cannot reach them, because
	# per-node anchors every member on itself and no two members of one op
	# ever collapse onto one word.  Its abort IS the failure signal -- a
	# violation kills the leg rather than printing a line a grep must
	# find.
	"holdtrace|-DDEBUG_RCU -DFEATURE_FT_HOLD_TRACE -DFEATURE_FT_ANCHOR_VALIDATE|u ion ioff imw|per-node exponential root-only"
	# ★ THE CONFIG THAT ACTUALLY CATCHES THE RAW-READ CLASS.
	#
	# txndbg above arms the same engine assert and NEVER FIRES IT: with
	# ft_chain_compress_fused's raw-read defect (@f8b1640e) deliberately put
	# back, txndbg scored 0 of 144 runs -- at -O2 AND at -O1, with and
	# without the MW leg -- while this combination reproduced it.  The
	# ingredient is FEATURE_FT_ANCHOR_VALIDATE, bisected against the same
	# reverted defect on the same machine:
	#
	#   --enable-rcu-debug + ANCHOR_VALIDATE     6 / 144   (the repro)
	#   --enable-rcu-debug, no ANCHOR_VALIDATE   0 /  96
	#   -DDEBUG_RCU alone                        0 /  48
	#   txndbg (DEBUG_RCU + READ_POLICY)         0 / 144
	#   these exact CPPFLAGS                     1 /  96
	#
	# ANCHOR_VALIDATE is not an extra assert here -- it is what makes a COARSE
	# SPACING SELECTABLE AT ALL.  Without it FEATURE_FT_LOCK_SPACING_ENV is not
	# compiled in, so CDS_FT_LOCK_SPACING is never read and every leg runs
	# per-node (and cds_ft_group_attr_set_lock_spacing refuses coarse outright).
	# That, not a widened window, is why the table above reads the way it does:
	# every 0-scoring row was running per-node against an EXPONENTIAL-only
	# defect.  Hence the guard below -- a swept config without the flag is a
	# sweep that silently does not happen.
	#
	# ★ The rate is ~1-4%, so ONE run of this config proves nothing -- it is
	# here to be run with FT_GATE_REPEAT when hunting, and the 3-spacing sweep
	# is mandatory because the defect it was built from is exponential-only.
	"anchorval|-DDEBUG_RCU -DFEATURE_FT_ANCHOR_VALIDATE|u ion ioff imw|per-node exponential root-only"
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
	# ☠ A FAILED SETUP MUST NOT REPORT OK, AND MUST NOT BE CACHED.
	#
	# Both halves were wrong and they compounded.  rsync's status was
	# discarded, so a copy that died part way (a full disk: see the excludes
	# above) left a tree missing arbitrary files; success was then inferred
	# from config.status EXISTING, which a configure that failed on a missing
	# Makefile.in still leaves behind; and .gate_flags was written regardless,
	# so the NEXT run took the early-return above and reused the broken tree.
	# The visible result was three consecutive runs reporting "setup ok"
	# followed by an autotools error from a disk-space failure.
	#
	# So: check both commands, keep their output for the report, prove the
	# configure by the file the build actually needs (src/Makefile -- not
	# config.status), and stamp .gate_flags ONLY on success so a failure
	# retries from scratch instead of sticking.
	if ! rsync -aS --exclude='.git' --exclude='build-*' --exclude='ft-parallel-gate-*' \
			--exclude='ft-hunt-*' --exclude='ft-segv-*' \
			--exclude='core' --exclude='core.*' --exclude='vgcore.*' --exclude='*.core' \
			"$ROOT/"  "$dir/" >"$GATE/$1.setup" 2>&1; then
		echo "  setup $1 FAILED (copy -- see $GATE/$1.setup)"
		echo "$1: CONFIG ERROR (source copy failed; see $GATE/$1.setup)" >> "$GATE/$1.result"
		return
	fi
	if ! ( cd "$dir" && make distclean >/dev/null 2>&1
	       CPPFLAGS="$flags" ./configure --quiet >>"$GATE/$1.setup" 2>&1 ); then
		echo "  setup $1 FAILED (configure -- see $GATE/$1.setup)"
		echo "$1: CONFIG ERROR (configure failed; see $GATE/$1.setup)" >> "$GATE/$1.result"
		return
	fi
	if [ ! -f "$dir/src/Makefile" ]; then
		echo "  setup $1 FAILED (no src/Makefile -- see $GATE/$1.setup)"
		echo "$1: CONFIG ERROR (configure left no src/Makefile; see $GATE/$1.setup)" >> "$GATE/$1.result"
		return
	fi
	printf 'CPPFLAGS=%s\n' "$flags" > "$dir/.gate_flags"
	echo "  setup $1 ok"
}

sync_src() {	# $1=name -- refresh live sources into the (already-configured) tree
	local dir=$GATE/$1
	rsync -a --delete "$ROOT/src/fractal-trie/" "$dir/src/fractal-trie/" 2>/dev/null
	rsync -a "$ROOT/src/"     "$dir/src/"     --exclude='.libs' --exclude='*.o' --exclude='*.lo' 2>/dev/null
	rsync -aS "$ROOT/tests/"  "$dir/tests/"   --exclude='.libs' --exclude='*.o' --exclude='*.lo' \
		--exclude='core' --exclude='core.*' --exclude='vgcore.*' --exclude='*.core' 2>/dev/null
	rsync -a "$ROOT/include/" "$dir/include/" 2>/dev/null
}

run_leg() {	# $1=cwd $2=timeout-secs ; $3.. = the command -- run it, echo its output
	local d=$1 tmo=$2; shift 2
	( cd "$d" || exit 99
	  # A crash is an EXPECTED outcome for this harness, so let it dump...
	  ulimit -c unlimited 2>/dev/null
	  # ...and drop bash's own "Aborted (core dumped)" job line, which would
	  # otherwise land on the gate's stderr once per red leg and read like
	  # the harness itself failing.  The command's own stderr is folded into
	  # stdout below and is not affected by this.
	  exec 2>/dev/null
	  timeout "$tmo" "$@" 2>&1 )
}

run_leg_multi() {	# $1=name $2=cdir $3=spacing $4=bin $5=lib $6=outfile
	# ★ THE MULTI-PROCESS ARM.  Every other leg runs ONE process, so the gate
	# is structurally blind to the load-sensitive class: test_urcu_ft_inv is
	# clean alone and has been recorded failing (`not ok inv_remove_cross_view`,
	# aborts around the graft_swap solo family) only with several copies in
	# flight.  A patch that "passes the gate" has therefore not been tested
	# under load at all.
	#
	# Copies are SEPARATE PROCESSES on purpose, not more threads inside one:
	# what the class needs is independent allocators, independent call_rcu
	# worker sets and independent RCU grace-period domains competing for the
	# same cores -- none of which raising a thread count inside one process
	# reproduces.
	#
	# Reported per COPY, never merged: concatenating N TAP streams multiplies
	# the ok-count and breaks the plan check that every other leg relies on to
	# catch a hang, and an aggregate "notok=3" cannot say whether one copy
	# failed three times or three copies failed once -- which is exactly the
	# distinction this leg exists to make.
	local name=$1 cdir=$2 sp=$3 bin=$4 lib=$5 out=$6
	local n=${FT_GATE_COPIES:-4}
	local i o rc ok notok plan ran clean=0 red=0 c
	local -a pids=() dirs=()
	for i in $(seq 1 "$n"); do
		mkdir -p "$cdir/copy-$i"
		dirs+=("$cdir/copy-$i")
		( run_leg "$cdir/copy-$i" 1800 env LD_LIBRARY_PATH="$lib" \
			CDS_FT_LOCK_SPACING="$sp" FT_INV_MW=1 "$bin" \
			> "$cdir/copy-$i.out" 2>&1; echo $? > "$cdir/copy-$i.rc" ) &
		pids+=($!)
	done
	wait "${pids[@]}" 2>/dev/null
	for i in $(seq 1 "$n"); do
		o=$(cat "$cdir/copy-$i.out" 2>/dev/null)
		rc=$(cat "$cdir/copy-$i.rc" 2>/dev/null); rc=${rc:-99}
		ok=$(printf '%s' "$o" | grep -c '^ok ')
		notok=$(printf '%s' "$o" | grep -c '^not ok ')
		plan=$(printf '%s' "$o" | sed -n 's/^1\.\.\([0-9]\{1,\}\)$/\1/p' | head -1)
		ran=$((ok + notok))
		# Same completeness checks as the single-process leg, and for the
		# same reason: a HUNG copy emits no `not ok` and would otherwise
		# score green on a partial run.
		if [ "$notok" -gt 0 ] || [ "$rc" -ne 0 ] || [ -z "$plan" ] \
				|| [ "$ran" -ne "$plan" ]; then
			red=$((red + 1))
			printf '%s' "$o" | grep '^not ok ' \
				| sed "s/^/      [$name] ft_inv mwx $sp copy$i /" >> "$out"
			if [ "$rc" -eq 124 ]; then
				echo "$name: ft_inv mwx $sp copy$i INCOMPLETE (TIMEOUT/hang after $ran tests)" >> "$out"
			elif [ "$rc" -gt 128 ]; then
				echo "$name: ft_inv mwx $sp copy$i INCOMPLETE (killed by signal $((rc - 128)) after $ran tests)" >> "$out"
			elif [ -z "$plan" ]; then
				echo "$name: ft_inv mwx $sp copy$i NO TAP PLAN (completeness unverifiable)" >> "$out"
			elif [ "$ran" -ne "$plan" ]; then
				echo "$name: ft_inv mwx $sp copy$i INCOMPLETE (ran $ran of $plan planned)" >> "$out"
			elif [ "$rc" -ne 0 ]; then
				echo "$name: ft_inv mwx $sp copy$i NONZERO EXIT ($rc) with a complete run" >> "$out"
			fi
			c=$(ls "$cdir/copy-$i"/core* 2>/dev/null | head -1)
			[ -n "$c" ] && echo "$name: ft_inv mwx $sp copy$i CORE kept -- LD_LIBRARY_PATH=$lib gdb $bin $c" >> "$out"
		else
			clean=$((clean + 1))
			rm -rf "$cdir/copy-$i" "$cdir/copy-$i.out" "$cdir/copy-$i.rc"
		fi
	done
	printf '  [%-11s] ft_inv mwx  %s copies=%s clean=%s red=%s\n' \
		"$name" "$sp" "$n" "$clean" "$red" >> "$out"
	[ "$red" -eq 0 ] && rm -rf "$cdir"
	return 0
}

run_one() {	# $1=name $2=tests $3=spacings $4=cppflags -- build lib+tests, run TAP
	local name=$1 tests=$2 spacings=${3:-} flags=${4:-}
	[ -n "$spacings" ] || spacings=${FT_GATE_SPACINGS:-per-node}
	# ★ A SWEEP THE BUILD CANNOT HONOUR IS WORSE THAN NO SWEEP: it relabels
	# three identical per-node runs as three spacings.  CDS_FT_LOCK_SPACING is
	# only read when FEATURE_FT_LOCK_SPACING_ENV is compiled in, which
	# FEATURE_FT_ANCHOR_VALIDATE / FEATURE_FT_HOLD_TRACE imply -- and without
	# it the library refuses a coarse spacing anyway.  Refuse to pretend.
	case " $spacings " in
	*" exponential "*|*" root-only "*)
		case "$flags" in
		*FEATURE_FT_ANCHOR_VALIDATE*|*FEATURE_FT_HOLD_TRACE*|*FEATURE_FT_LOCK_SPACING_ENV*) ;;
		*)
			echo "$name: CONFIG ERROR (sweeps [$spacings] but its flags cannot select one -- add -DFEATURE_FT_ANCHOR_VALIDATE)" >> "$GATE/$name.result"
			return ;;
		esac ;;
	esac
	local dir=$GATE/$name out=$GATE/$name.result
	local LIB=$dir/src/.libs U=$dir/tests/unit/.libs/test_urcu_ft_unit
	local I=$dir/tests/regression/.libs/test_urcu_ft_inv
	local SP=$dir/tests/unit/.libs/test_rcu_txn_settle_premise
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
	# Only the configs that ASK for it (tests list contains sp) build the
	# settle-premise validator, because it is only meaningful where
	# -DURCU_TXN_DEBUG_SETTLE is defined -- elsewhere it compiles to a skip.
	case " $tests " in
	*" sp "*)
		if ! make -C "$dir/tests/unit" test_rcu_txn_settle_premise -j"$J" \
				>>"$GATE/$name.build" 2>&1; then
			echo "$name: TEST BUILD FAIL (txn settle; see $GATE/$name.build)" >> "$out"
			return
		fi ;;
	esac
	echo "$name: build ok" >> "$out"
	local t o ok notok abrt lbl rc plan ran sp rep cdir bin tmo c
	local env_x
	# ★ KEEP THE CORE.  Counting aborts names the CONFIG; it does not name
	# the slot, the writer or the arm -- and both instances of the
	# raw-read-of-a-parked-slot class closed so far were root-caused from a
	# CORE, after gdb had failed to reproduce either one under a debugger
	# (single-stepping closes the window).  So every leg below runs in its
	# OWN directory with the core limit raised, and that directory SURVIVES
	# iff the leg went red: a green matrix leaves nothing behind, a red one
	# leaves the evidence next to the line that names it.
	# The per-leg cwd is not tidiness.  core_pattern is a bare RELATIVE
	# "core" on the usual configuration, so without it the cores land
	# wherever the gate was invoked from -- the repo, polluting git status --
	# and concurrent legs overwrite each other's.  core_uses_pid then names
	# the file core.PID, so glob core*, never exactly "core".
	local cores=$GATE/$name.cores
	rm -rf "$cores"; mkdir -p "$cores"
	# ★ THE SPACINGS RUN CONCURRENTLY, and this is what makes a swept config
	# affordable.  Serialised, each three-spacing config took 981 s and was
	# the whole gate's critical path -- the 14-config matrix measured 1053 s,
	# i.e. one swept config plus noise, with eleven others idling alongside
	# it.  The axis is embarrassingly parallel (separate processes, separate
	# cwds, one shared read-only build), so sweeping it now costs ONE
	# spacing rather than three: anchorval 1017 s -> 365 s, the whole matrix
	# 1053 s -> 383 s, same 61 legs, same verdict.
	# Peak concurrency rises from ~14 legs to ~19; measured load stayed near
	# 54 on this 192-core box, so the axis fits without a throttle.  Add one
	# here if a future matrix makes (configs x spacings) outgrow the machine.
	#
	# Each spacing writes its OWN result file.  They are concatenated below in
	# the DECLARED order, never in completion order: appending concurrently to
	# one file interleaves the lines of a red with the lines of a green, and an
	# unattributable red is worse than a slow gate.  It also keeps two runs of
	# the gate diffable, which completion order would not.
	local sp
	for sp in $spacings; do
		run_spacing "$name" "$tests" "$sp" "$GATE/$name.result.$sp" &
	done
	wait
	for sp in $spacings; do
		cat "$GATE/$name.result.$sp" >> "$out" 2>/dev/null
		rm -f "$GATE/$name.result.$sp"
	done
	# Every leg was green, so nothing was kept: drop the empty spine too.
	rmdir "$cores" 2>/dev/null
}

run_spacing() {	# $1=name $2=tests $3=spacing $4=outfile -- every leg at ONE spacing
	local name=$1 tests=$2 sp=$3 out=$4
	local dir=$GATE/$name
	local LIB=$dir/src/.libs U=$dir/tests/unit/.libs/test_urcu_ft_unit
	local I=$dir/tests/regression/.libs/test_urcu_ft_inv
	local SP=$dir/tests/unit/.libs/test_rcu_txn_settle_premise
	local cores=$GATE/$name.cores
	local t o ok notok abrt lbl rc plan ran rep cdir bin tmo c
	local env_x
	: > "$out"
	# REPEAT: an intermittent defect is a coin flip at one run.  The two
	# measured instances of the raw-read class fired at ~10% and ~8%, so a
	# single green leg is not evidence.  Opt-in (default 1) because it
	# multiplies the whole matrix; use it when hunting, not on every commit.
	for rep in $(seq 1 "${FT_GATE_REPEAT:-1}"); do
	for t in $tests; do
		cdir=$cores/$sp-$t-$rep
		mkdir -p "$cdir"
		# The multi-process arm reports per copy and keeps its own cores, so
		# it bypasses the single-process parsing below entirely.
		if [ "$t" = imwx ]; then
			run_leg_multi "$name" "$cdir" "$sp" "$I" "$LIB" "$out"
			continue
		fi
		case $t in
		u)    bin=$U; tmo=900;  lbl="ft_unit   "; env_x=() ;;
		ion)  bin=$I; tmo=300;  lbl="ft_inv on "; env_x=() ;;
		ioff) bin=$I; tmo=300;  lbl="ft_inv off"; env_x=(FT_INV_NO_ORDERED_LIST=1) ;;
		# The concurrent-writer oracles (MW writers, coherent rekey) all
		# gate on FT_INV_MW at runtime and are otherwise skipped, so ion
		# / ioff never exercise them.  They are the long leg -- hence the
		# larger timeout.
		imw)  bin=$I; tmo=1800; lbl="ft_inv mw "; env_x=(FT_INV_MW=1) ;;
		# The settle-premise DETECTOR's own validation.  It belongs to a
		# config that DEFINES -DURCU_TXN_DEBUG_SETTLE and nowhere else:
		# without the flag it skips, and a detector whose self-test only
		# ever skips is indistinguishable from one that is blind.  It
		# constructs its violation rather than waiting for one, so it is
		# ~20 ms and carries no spacing dependence.
		sp)   bin=$SP; tmo=120; lbl="txn settle"; env_x=() ;;
		esac
		o=$(run_leg "$cdir" "$tmo" env LD_LIBRARY_PATH="$LIB" \
			CDS_FT_LOCK_SPACING="$sp" \
			${env_x[@]+"${env_x[@]}"} "$bin"); rc=$?
		# The SPACING rides every label below: a red that names only the
		# suite is unattributable when three spacings run the same leg.
		lbl="$lbl $sp"
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
		# Keep this leg's directory iff it went red, and say where the core
		# is (see the KEEP THE CORE note above).  "Red" is deliberately
		# WIDER here than the RESULTS grep below: a leg killed by a timeout
		# or a signal leaves a core too, and that core is the entire point
		# of having kept the directory.
		if [ "$notok" -gt 0 ] || [ "$abrt" -gt 0 ] || [ "$rc" -ne 0 ] \
				|| [ -z "$plan" ] || [ "$ran" -ne "$plan" ]; then
			c=$(ls "$cdir"/core* 2>/dev/null | head -1)
			if [ -n "$c" ]; then
				echo "$name: $lbl CORE kept -- LD_LIBRARY_PATH=$LIB gdb $bin $c" >> "$out"
			fi
		else
			rm -rf "$cdir"
		fi
	done
	done
}

echo "gate dir : $GATE   (-j$J x ${#CONFIGS[@]} configs on $NCPU cores)"
echo "=== [1/3] configure per-config trees (parallel; one-time) ==="
for c in "${CONFIGS[@]}"; do IFS='|' read -r n f _ <<< "$c"; setup_tree "$n" "$f" & done; wait
echo "=== [2/3] sync live sources (parallel) ==="
for c in "${CONFIGS[@]}"; do sync_src "${c%%|*}" & done; wait
echo "=== [3/3] build + test (parallel) ==="
for c in "${CONFIGS[@]}"; do IFS='|' read -r n f t sp <<< "$c"; run_one "$n" "$t" "$sp" "$f" & done; wait

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
	# NONZERO EXIT belongs in this list: run_one PRINTS it and nothing acted
	# on it, so a leg that completed its plan with no failing TAP line and a
	# non-zero status -- a crash in teardown after the last test, an explicit
	# exit(1) -- reported its own red line under a "GATE PASS".
	   grep -qE 'BUILD FAIL|CONFIG ERROR|notok=[1-9]|abrt=[1-9]|ok=0 notok=|INCOMPLETE|NO TAP PLAN|NONZERO EXIT' "$GATE/$n.result"; then
		rc=1
	fi
done
if [ "$rc" -eq 0 ]; then echo "=== GATE PASS ==="; else echo "=== GATE FAIL ==="; fi
exit "$rc"
