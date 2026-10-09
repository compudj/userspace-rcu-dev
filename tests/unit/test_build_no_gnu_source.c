// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Check that the fair mutex and transaction headers build in a translation
 * unit that does not define _GNU_SOURCE.  config.h defines it for every source
 * file of this tree, so undefine it here, before the first system header reads
 * it.
 */

#undef _GNU_SOURCE

#define _LGPL_SOURCE

#include <urcu-qsbr.h>
#include <urcu/fair-mutex.h>
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-bitmap.h>
#include <urcu/rcu-txn-deque.h>
#include <urcu/rcu-txn-hlist.h>
#include <urcu/rcu-txn-list.h>
#include <urcu/rcu-txn-skiplist.h>
#include <urcu/rcu-txn-sw.h>
#include <urcu/rcu-txn-sw-bitmap.h>
#include <urcu/rcu-txn-sw-hlist.h>
#include <urcu/rcu-txn-sw-list.h>

#include "tap.h"

int main(void)
{
	plan_tests(1);
	ok(1, "the fair mutex and transaction headers build without _GNU_SOURCE");
	return exit_status();
}
