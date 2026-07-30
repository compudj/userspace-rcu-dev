// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * LD_PRELOAD shim that makes one cpu "go away" mid-run, without root.
 *
 * Hot-unplugging a cpu needs privileges; so does cpuset(7) unless the user
 * slice was configured with Delegate=cpuset.  But the library does not observe
 * hotplug -- it observes two syscalls: sched_getcpu() telling it it is no
 * longer on its cpu, and sched_setaffinity() answering EINVAL when it tries to
 * get back.  Interpose exactly those two and the code under test cannot tell
 * the difference, which is the point: this exercises the real code path rather
 * than a special case compiled in for testing.
 *
 * Sequence, for the worker pinned to URCU_TEST_FAULT_CPU:
 *
 *   1. its FIRST pin succeeds, so the worker starts normally and is genuinely
 *      running on that cpu -- the state a real departure starts from;
 *   2. from then on sched_getcpu() reports a different cpu to any thread
 *      actually running there, so the worker believes it was migrated off;
 *   3. its attempt to re-pin gets EINVAL, which is what a departed cpu gives.
 *
 * Step 1 is what makes this cover the RUNTIME transition rather than the
 * already-gone-at-startup case: no reasonable amount of unprivileged
 * cpu-juggling gets you a cpu that works and then stops.
 *
 * The lie is deliberately narrow -- only a single-cpu mask naming the target is
 * failed, and only a thread genuinely ON the target is misinformed -- so every
 * other thread in the process behaves normally.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <sched.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

static int (*real_setaffinity)(pid_t, size_t, const cpu_set_t *);
static int (*real_getcpu)(void);
static int fault_cpu = -1;
static int ncpu;
static volatile int armed;	/* set once the target has been pinned for real */

__attribute__((constructor))
static void affinity_fault_init(void)
{
	const char *e = getenv("URCU_TEST_FAULT_CPU");

	real_setaffinity = dlsym(RTLD_NEXT, "sched_setaffinity");
	real_getcpu = dlsym(RTLD_NEXT, "sched_getcpu");
	ncpu = (int) sysconf(_SC_NPROCESSORS_CONF);
	if (e)
		fault_cpu = atoi(e);
}

static int targets_fault_cpu(size_t cpusetsize, const cpu_set_t *mask)
{
	return fault_cpu >= 0 && mask &&
		CPU_ISSET_S(fault_cpu, cpusetsize, mask) &&
		CPU_COUNT_S(cpusetsize, mask) == 1;
}

int sched_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t *mask)
{
	if (targets_fault_cpu(cpusetsize, mask)) {
		if (!armed) {
			int ret = real_setaffinity(pid, cpusetsize, mask);

			if (!ret)
				armed = 1;	/* the cpu worked once */
			return ret;
		}
		errno = EINVAL;			/* ... and now it is gone */
		return -1;
	}
	return real_setaffinity(pid, cpusetsize, mask);
}

int sched_getcpu(void)
{
	int cpu = real_getcpu();

	if (armed && fault_cpu >= 0 && cpu == fault_cpu && ncpu > 1)
		return (fault_cpu + 1) % ncpu;	/* "you were migrated off" */
	return cpu;
}
