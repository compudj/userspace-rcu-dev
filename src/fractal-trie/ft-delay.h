// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-delay.h
 *
 * Userspace RCU library - Fractal Trie: optional race-window delay injection.
 *
 * Manual debugging aid, compiled out unless -DFT_DELAY_INJECT is defined.  It
 * widens reader/writer race windows by inserting env-gated usleeps at the
 * ft_delay_writer() / ft_delay_reader() hooks (declared in
 * fractal-trie-internal.h, called at the critical descent / publish points).
 * This unit holds the runtime state and the env-var (FT_DELAY_MODE /
 * FT_DELAY_US) constructor.
 *
 * Implementation unit: #included once into the fractal-trie.c translation unit.
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-delay.h is an implementation unit; #include it from fractal-trie.c only"
#endif

#ifdef FT_DELAY_INJECT
#include <unistd.h>
#include <stdlib.h>
enum ft_delay_mode ft_delay_mode = FT_DELAY_NONE;
unsigned int ft_delay_us = 1;

static void __attribute__((constructor))
ft_delay_init(void)
{
	const char *mode = getenv("FT_DELAY_MODE");
	const char *us = getenv("FT_DELAY_US");

	if (mode) {
		if (!strcmp(mode, "writer"))
			ft_delay_mode = FT_DELAY_WRITER;
		else if (!strcmp(mode, "reader"))
			ft_delay_mode = FT_DELAY_READER;
		else if (!strcmp(mode, "both"))
			ft_delay_mode = FT_DELAY_BOTH;
		else if (!strcmp(mode, "random"))
			ft_delay_mode = FT_DELAY_RANDOM;
	}
	if (us)
		ft_delay_us = (unsigned int) atoi(us);
}
#endif
