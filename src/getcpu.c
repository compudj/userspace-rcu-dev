// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * The part of <urcu/getcpu.h> that is not inline.
 */

#include <urcu/getcpu.h>

#include "compat-getcpu.h"

int urcu_getcpu_fallback(void)
{
	return urcu_sched_getcpu();
}
