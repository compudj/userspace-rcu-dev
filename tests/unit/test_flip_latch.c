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

#define NR_TESTS 22

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

	/*
	 * 6. reserve_slot: a proxy needed BEFORE its slot exists.  The embedder
	 * places the returned tagged proxy itself, binds the slot, then commits.
	 * Because the placed proxy is already live, commit MUST flip the group (the
	 * single-edge bare-store fast path would not switch a held proxy), so it
	 * owes a grace period and a reader holding the proxy resolves to new.
	 */
	{
		void *slot;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		struct urcu_flip_latch *l;
		void *pf;
		bool gp;

		urcu_flip_txn_reserve(t, 4);
		pf = urcu_flip_txn_reserve_slot(t, (void *) 0x100,
			(void *) 0x200, &l);
		slot = pf;			/* embedder places the proxy */
		ok(is_proxy(slot) && resolve(slot) == (void *) 0x100,
			"reserve_slot proxy is live and resolves to old");
		urcu_flip_txn_bind_slot(l, &slot);
		gp = urcu_flip_txn_commit(t);
		ok(gp, "reserve_slot commit owes a grace period (proxy is live)");
		ok(slot == (void *) 0x200, "reserve_slot slot settles to new");
		ok(resolve(pf) == (void *) 0x200,
			"a reader holding the placed proxy now resolves to new");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 7. reserve_slot fused with ordinary record()s: the placed slot proxy and
	 * the recorded edges all flip together in one commit.
	 */
	{
		void *s_slot, *r1 = (void *) 0x10, *r2 = (void *) 0x20;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		struct urcu_flip_latch *l;
		void *pf;
		bool gp;

		urcu_flip_txn_reserve(t, 4);
		pf = urcu_flip_txn_reserve_slot(t, (void *) 0x100,
			(void *) 0x200, &l);
		s_slot = pf;
		urcu_flip_txn_bind_slot(l, &s_slot);
		urcu_flip_txn_record(t, &r1, (void *) 0x10, (void *) 0x11);
		urcu_flip_txn_record(t, &r2, (void *) 0x20, (void *) 0x21);
		gp = urcu_flip_txn_commit(t);
		ok(gp, "fused reserve_slot + records owes a grace period");
		ok(s_slot == (void *) 0x200 && r1 == (void *) 0x11 &&
			r2 == (void *) 0x21,
			"fused commit settles the placed slot and the records");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 8. Abort after a reserve_slot placement: the live proxy is restored to
	 * old and a grace period is owed, even though the txn never left PREPARE.
	 */
	{
		void *slot;
		struct urcu_flip_txn *t = urcu_flip_txn_create(test_tag);
		struct urcu_flip_latch *l;
		void *pf;
		bool gp;

		urcu_flip_txn_reserve(t, 4);
		pf = urcu_flip_txn_reserve_slot(t, (void *) 0x100,
			(void *) 0x200, &l);
		slot = pf;
		urcu_flip_txn_bind_slot(l, &slot);
		gp = urcu_flip_txn_abort(t);
		ok(gp && slot == (void *) 0x100,
			"abort after reserve_slot restores old and owes a GP");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 9. Single-allocation bounded txn: header + inline head chunk in ONE
	 * malloc.  Behaves like an ordinary multi-edge commit; the inline chunk is
	 * freed with the header (ASAN would catch a double-free or leak).
	 */
	{
		void *s1 = (void *) 0x10, *s2 = (void *) 0x20;
		struct urcu_flip_txn *t = urcu_flip_txn_create_bounded(test_tag, 4);
		bool gp;

		ok(t != NULL, "create_bounded one-alloc");
		urcu_flip_txn_record(t, &s1, (void *) 0x10, (void *) 0x11);
		urcu_flip_txn_record(t, &s2, (void *) 0x20, (void *) 0x21);
		gp = urcu_flip_txn_commit(t);
		ok(gp && s1 == (void *) 0x11 && s2 == (void *) 0x21,
			"bounded multi-edge commit settles to new, owes a GP");
		urcu_flip_txn_destroy(t);
	}

	/*
	 * 10. Bounded txn + reserve_slot: the insert/point-store shape -- an
	 * embedder-placed slot proxy fused with a recorded edge, single allocation.
	 */
	{
		void *s_slot, *r1 = (void *) 0x40;
		struct urcu_flip_txn *t = urcu_flip_txn_create_bounded(test_tag, 4);
		struct urcu_flip_latch *l;
		void *pf;
		bool gp;

		pf = urcu_flip_txn_reserve_slot(t, (void *) 0x100,
			(void *) 0x200, &l);
		s_slot = pf;
		urcu_flip_txn_bind_slot(l, &s_slot);
		urcu_flip_txn_record(t, &r1, (void *) 0x40, (void *) 0x41);
		gp = urcu_flip_txn_commit(t);
		ok(gp, "bounded reserve_slot + record owes a GP");
		ok(s_slot == (void *) 0x200 && r1 == (void *) 0x41,
			"bounded reserve_slot commit settles placed slot + record");
		urcu_flip_txn_destroy(t);
	}

	return exit_status();
}
