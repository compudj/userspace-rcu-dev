// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
// SPDX-License-Identifier: MIT

/*
 * LTTng-UST tracepoint probe provider for Fractal Trie.
 *
 * Only instantiated when the library is built with -DFT_ENABLE_TRACING.
 * Otherwise this compilation unit is empty and the library has no
 * lttng-ust link dependency.
 *
 * When enabled, callers must link with -llttng-ust -llttng-ust-common.
 */

#ifdef FT_ENABLE_TRACING
#define LTTNG_UST_TRACEPOINT_CREATE_PROBES
#define LTTNG_UST_TRACEPOINT_DEFINE
#include "cds_ft_tp.h"
#endif
