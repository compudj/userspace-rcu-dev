// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Unit test for urcu_flip_txn: the multi-edge transaction layer of the
 * flip-latch, focusing on the commit() shortcuts -- the single-edge fast path
 * (no proxy at all) and auto-install -- and the explicit-install caveat.
 */

#include <stdbool.h>

#include "tap.h"
#include "urcu-flip-latch.h"

#define NR_TESTS 11

/* Embedder tag hook: mark bit 0 of the proxy pointer (latches are 16B-aligned). */
static void *test_tag(struct urcu_flip_proxy *p)
{
	return (void *) ((unsigned long) p | 1UL);
}

static int is_proxy(void *v)
{
	return (unsigned long) v & 1UL;
}

static void *resolve(void *v)
{
	if (is_proxy(v))
		return urcu_flip_proxy_get(
			(struct urcu_flip_proxy *) ((unsigned long) v & ~1UL));
	return v;
}

int main(void)
{
	plan_tests(NR_TESTS);

	/*
	 * 1. Single-edge fast path: a commit reached in PREPARE with exactly one
	 * recorded edge stores the new target directly -- no proxy is ever
	 * installed, so no grace period is owed.
	 */
	{
		void *slot = (void *) 0x100;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		bool gp;

		ok(t && urcu_flip_txn_reserve(t, 4), "create + reserve");
		urcu_flip_txn_record(t, &slot, (void *) 0x100, (void *) 0x200);
		gp = urcu_flip_txn_commit(t);		/* no explicit install */
		ok(!gp, "single-edge commit owes no grace period");
		ok(slot == (void *) 0x200, "single-edge slot holds the new target directly");
		ok(!is_proxy(slot), "single-edge never parks a proxy");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 2. Multi-edge auto-install: a commit reached in PREPARE with two or more
	 * edges installs every proxy first, then flips the group; all slots settle
	 * to new and a grace period is owed.
	 */
	{
		void *s1 = (void *) 0x10, *s2 = (void *) 0x20, *s3 = (void *) 0x30;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		bool gp;

		urcu_flip_txn_reserve(t, 4);
		urcu_flip_txn_record(t, &s1, (void *) 0x10, (void *) 0x11);
		urcu_flip_txn_record(t, &s2, (void *) 0x20, (void *) 0x21);
		urcu_flip_txn_record(t, &s3, (void *) 0x30, (void *) 0x31);
		gp = urcu_flip_txn_commit(t);		/* no explicit install */
		ok(gp, "multi-edge commit owes a grace period");
		ok(s1 == (void *) 0x11 && s2 == (void *) 0x21 &&
			s3 == (void *) 0x31, "multi-edge slots all settled to new");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 3. Explicit install + single edge: once install() parks the proxy a
	 * reader may already hold it, so commit MUST flip the group (the fast path
	 * is unavailable) -- a grace period is owed.
	 */
	{
		void *slot = (void *) 0x100;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		bool gp;

		urcu_flip_txn_reserve(t, 4);
		urcu_flip_txn_record(t, &slot, (void *) 0x100, (void *) 0x200);
		urcu_flip_txn_install(t);
		ok(is_proxy(slot) && resolve(slot) == (void *) 0x100,
			"explicit install parks a proxy resolving to old");
		gp = urcu_flip_txn_commit(t);
		ok(gp, "explicitly-installed single edge still owes a grace period");
		ok(slot == (void *) 0x200, "explicitly-installed slot settles to new");
		urcu_flip_txn_destroy(t);
	}

	/* 4. Abort in PREPARE: nothing installed, slot untouched, no grace period. */
	{
		void *slot = (void *) 0x100;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		bool gp;

		urcu_flip_txn_record(t, &slot, (void *) 0x100, (void *) 0x200);
		gp = urcu_flip_txn_abort(t);
		ok(!gp && slot == (void *) 0x100,
			"abort in PREPARE leaves the slot untouched, owes no GP");
		urcu_flip_txn_destroy(t);
	}

	/* 5. Abort after install: every slot restored to old, grace period owed. */
	{
		void *slot = (void *) 0x100;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		bool gp;

		urcu_flip_txn_record(t, &slot, (void *) 0x100, (void *) 0x200);
		urcu_flip_txn_install(t);
		gp = urcu_flip_txn_abort(t);
		ok(gp && slot == (void *) 0x100,
			"abort after install restores old, owes a GP");
		urcu_flip_txn_destroy(t);
	}

	return exit_status();
}
