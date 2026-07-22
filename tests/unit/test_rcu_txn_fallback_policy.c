// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/* Exercise the scaled fallback budget at its largest accepted numerator. */

#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <limits.h>

#define MAX_EXACT_SCALE			(UINT64_MAX / UINT_MAX)
#define URCU_TXN_FALLBACK_PER_COST_NUM	MAX_EXACT_SCALE
#define URCU_TXN_FALLBACK_PER_COST_DEN	MAX_EXACT_SCALE

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

int main(void)
{
	struct urcu_txn txn;

	plan_tests(2);

	urcu_txn_init(&txn, NULL);
	txn.last_cost = 2;
	ok(urcu_txn__fallback_at(&txn) == URCU_TXN_FALLBACK_MIN,
		"the largest exact scale is divided before applying the floor");

	txn.last_cost = UINT_MAX;
	ok(urcu_txn__fallback_at(&txn) == URCU_TXN_FALLBACK_MAX,
		"the exact wide quotient is capped at the configured maximum");

	return exit_status();
}
