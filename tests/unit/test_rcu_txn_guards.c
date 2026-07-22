// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test the ENGINE CONTRACT GUARDS of <urcu/rcu-txn-mcas.h> and <urcu/rcu-txn-sw.h>:
 * the debug assertions that turn a violated precondition into an immediate,
 * localized abort instead of silent corruption far away.
 *
 * Every case below is an embedder BUG, so the guard's job is to die.  Each is
 * therefore run in a forked child and the parent asserts the child was killed by
 * SIGABRT; a control case (a well-formed transaction) checks the same guards do
 * not fire spuriously.  The child drops its core limit to 0 so a passing run
 * leaves no core files behind.
 *
 * The guards, and what each one is standing in front of:
 *
 *   1. Duplicate slot in one commit.  The install protocol's "own proxy"
 *      short-circuit assumes records target pairwise-distinct slots; a duplicate
 *      makes the commit install both records and settle them in record order,
 *      silently dropping the earlier edit while reporting OK.  The check must be
 *      PAIRWISE, not adjacent-only: an age-0 commit installs flat and UNSORTED,
 *      so its duplicates sit anywhere.  This test's duplicate is deliberately
 *      NON-ADJACENT (a, b, a) -- the adjacency scan this replaced walked
 *      straight past it.
 *   2/3. Tag contract.  No value an embedder stores in a transacted slot may
 *      carry all of that slot's tag bits, or the engine mistakes a live value
 *      for one of its parked records and a resolver fabricates a record pointer
 *      out of it.  Caught at the store that introduces it -- on the new value
 *      and on the old -- rather than as a wild dereference in a later resolve.
 *   4. sw install of a CALLER-storage (init_inline) handle carrying records.
 *      Such a handle owns no block to hang a group off, and install's "no
 *      record" branch would repoint its latch array at a fresh block --
 *      discarding the caller's records -- then park proxies through that block's
 *      UNINITIALIZED slot pointers: wild stores to garbage addresses.
 *   5. sw reserve() after install().  install() is a documented public entry, so
 *      a white-box caller can reach reserve() with proxies already parked; the
 *      grow path would then free the block those live slots still point into --
 *      a reader use-after-free.  The record set is frozen once installed.
 *
 * Cases 1-3 are urcu_assert_debug (DEBUG_RCU); 4-5 are urcu_posix_assert
 * (NDEBUG).  This test is compiled with -DDEBUG_RCU (see Makefile.am) so the
 * first group exists whatever the tree's build flags; under NDEBUG every guard
 * compiles out and the whole set is skipped.
 *
 * As regression tests: 1, 2, 3 and 5 all FAIL against the pre-guard engine.
 * Case 4 does not, and it is worth being precise about why -- that engine also
 * died, but incidentally: its debug duplicate-scan ran on the fresh block's
 * still-zeroed latch array, read the all-NULL slot pointers as duplicates, and
 * aborted there.  With the scan compiled out it instead parked proxies through
 * those NULL/garbage pointers.  Case 4 therefore pins the CONTRACT (reject the
 * handle, before parking anything, with a diagnostic that names the real bug)
 * rather than discriminating old from new.
 *
 * QSBR flavor (the commit defers descriptor reclaim through call_rcu).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-sw.h>

#include "tap.h"

#define NR_TESTS	6

/* Opaque, bit-0-clear slot values (the engine owns bit 0 as its proxy tag). */
#define V0	((void *) 0x100)
#define V1	((void *) 0x200)
#define V2	((void *) 0x300)
/* Bit 0 SET: indistinguishable from a parked proxy under URCU_TXN_TAG. */
#define VTAGGED	((void *) 0x101)

static void *g_a, *g_b;

/*
 * A checkpoint the child sets at a point it must NOT reach.  Shared with the
 * parent (the child dies, so it cannot report anything else), which is what lets
 * a case assert WHERE the guard fired, not merely THAT the process died: see
 * body_sw_inline_install(), whose whole point is that the abort must precede the
 * wild stores rather than follow them.
 */
static int *g_reached;

/*
 * Run @body in a child.  Returns 1 if the child died by SIGABRT (the guard
 * fired), 0 otherwise.  @body either aborts or returns.  Clears the checkpoint
 * first, so *g_reached afterwards says whether @body ran past it.
 */
static int aborts_in_child(void (*body)(void))
{
	struct rlimit nocore = { .rlim_cur = 0, .rlim_max = 0 };
	pid_t pid;
	int status;

	*g_reached = 0;
	pid = fork();
	if (pid < 0) {
		diag("fork: failed");
		return 0;
	}
	if (pid == 0) {
		(void) setrlimit(RLIMIT_CORE, &nocore);	/* a fired guard dumps no core */
		body();
		_exit(0);			/* the guard did NOT fire */
	}
	if (waitpid(pid, &status, 0) < 0) {
		diag("waitpid: failed");
		return 0;
	}
	return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

/* 1. A disjoint handle records the same slot twice, NON-adjacently. */
static void body_dup_slot(void)
{
	struct urcu_txn txn;

	urcu_txn_init(&txn, NULL);
	urcu_txn_declare_disjoint(&txn);	/* the lie: the slots are NOT distinct */
	urcu_txn_begin(&txn);
	urcu_txn_store_mw(&txn, &g_a, NULL, V0, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_b, NULL, V1, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_a, NULL, V2, URCU_TXN_TAG);	/* duplicate of record 0 */
	(void) urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
}

/* 2. A stored NEW value carries the slot's proxy tag bits. */
static void body_tagged_new(void)
{
	struct urcu_txn txn;

	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	urcu_txn_store_mw(&txn, &g_a, NULL, VTAGGED, URCU_TXN_TAG);
	(void) urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
}

/* 3. A stored OLD value carries the slot's proxy tag bits. */
static void body_tagged_old(void)
{
	struct urcu_txn txn;

	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	urcu_txn_store_mw(&txn, &g_a, VTAGGED, V0, URCU_TXN_TAG);
	(void) urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
}

/* 4. sw: an inline (caller-storage) handle carrying records reaches install. */
static void body_sw_inline_install(void)
{
	struct urcu_txn_sw_latch buf[4] __attribute__((aligned(16)));
	struct urcu_txn_sw_txn t;

	urcu_txn_sw_init_inline(&t, buf, 4);
	(void) urcu_txn_sw_record(&t, &g_a, NULL, V0, URCU_TXN_TAG);
	(void) urcu_txn_sw_record(&t, &g_b, NULL, V1, URCU_TXN_TAG);
	urcu_txn_sw_install(&t);		/* nr >= 2 on inline storage: illegal */
	/*
	 * Unreachable: install() must REJECT the handle, not fix it up.  Reaching
	 * here means it ran the park loop -- i.e. it already release-stored
	 * proxies through the fresh block's UNINITIALIZED slot pointers.  The
	 * checkpoint is what pins "aborts BEFORE parking"; SIGABRT alone would not
	 * (commit() has a late latches_inline assert that fires after the damage).
	 */
	*g_reached = 1;
	(void) urcu_txn_sw_commit(&t);
}

/* 5. sw: reserve() after install() -- would free a block with parked proxies. */
static void body_sw_reserve_after_install(void)
{
	struct urcu_txn_sw_txn t;

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_record(&t, &g_a, NULL, V0, URCU_TXN_TAG);
	(void) urcu_txn_sw_record(&t, &g_b, NULL, V1, URCU_TXN_TAG);
	urcu_txn_sw_install(&t);		/* proxies now parked in g_a, g_b */
	(void) urcu_txn_sw_reserve(&t, 64);	/* the record set is frozen: illegal */
	(void) urcu_txn_sw_commit(&t);
}

/* Control: a well-formed disjoint transaction must NOT trip any guard. */
static void body_control(void)
{
	struct urcu_txn txn;

	urcu_txn_init(&txn, NULL);
	urcu_txn_declare_disjoint(&txn);	/* true here: g_a != g_b */
	urcu_txn_begin(&txn);
	urcu_txn_store_mw(&txn, &g_a, NULL, V0, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_b, NULL, V1, URCU_TXN_TAG);
	if (urcu_txn_commit(&txn) != URCU_TXN_STATUS_OK)
		abort();		/* fail the control: an honest txn must commit */
	urcu_txn_end(&txn);
}

int main(void)
{
	plan_tests(NR_TESTS);

#ifdef NDEBUG
	skip(NR_TESTS, "NDEBUG: every contract guard is compiled out");
#else
	g_reached = mmap(NULL, sizeof(*g_reached), PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (g_reached == MAP_FAILED) {
		diag("mmap: failed");
		return EXIT_FAILURE;
	}
	rcu_register_thread();

	ok(aborts_in_child(body_dup_slot),
		"duplicate slot in one commit aborts (pairwise: the dup is NON-adjacent, at age 0)");
	ok(aborts_in_child(body_tagged_new),
		"a stored NEW value carrying the slot's tag bits aborts at the store");
	ok(aborts_in_child(body_tagged_old),
		"a stored OLD value carrying the slot's tag bits aborts at the store");
	/*
	 * Note the second half: it must abort BEFORE the park loop.  A late
	 * assert (as the pre-fix engine had, in commit()) also kills the process,
	 * so SIGABRT alone would pass while memory was already scribbled on.
	 */
	ok(aborts_in_child(body_sw_inline_install) && !*g_reached,
		"sw: installing an inline-storage handle that holds records aborts BEFORE parking anything");
	ok(aborts_in_child(body_sw_reserve_after_install),
		"sw: reserve() after install() aborts (would free a block with parked proxies)");
	ok(!aborts_in_child(body_control),
		"control: a well-formed disjoint commit trips no guard");

	rcu_barrier();			/* drain deferred descriptor frees */
	rcu_unregister_thread();
#endif
	return exit_status();
}
