// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * call_rcu affinity-change notifiers.
 *
 * The notifier exists so that per-cpu state owned by something OUTSIDE the
 * call_rcu machinery -- a per-cpu allocator's rseq-only freelists, say -- can be
 * reclaimed when its cpu goes away.  The worker is the only thing that both
 * knows which cpu it stands for and finds out the cpu is unreachable, via
 * sched_setaffinity() returning EINVAL.
 *
 * Making a cpu genuinely go away needs root.  Three ways to approach it without,
 * in descending order of fidelity; this test takes whichever is available:
 *
 *   1. cpuset(7).  A real "possible but unavailable" cpu, which is one of the
 *      two causes the library names.  Needs the user slice to be delegated the
 *      cpuset controller (systemd Delegate=cpuset), so it is probed, not
 *      assumed.
 *   2. LD_PRELOAD fault injection.  The library does not observe hotplug, it
 *      observes sched_getcpu() and sched_setaffinity(); interposing those
 *      reproduces the RUNTIME transition -- worker starts happily on its cpu,
 *      then loses it -- which neither of the others can reach unprivileged.
 *      Run in a forked child so the preload applies to a fresh process.
 *   3. Nothing special: assert that a cpu the machine cannot have is REFUSED at
 *      creation, which needs no privilege at all.
 *
 * Self-restriction is deliberately NOT used: a thread that narrows its own mask
 * with sched_setaffinity() can widen it again, so no EINVAL ever appears
 * (measured).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>

#include <urcu/urcu-memb.h>
#include <urcu/call-rcu.h>
#include <urcu/uatomic.h>
#include <urcu/compiler.h>

#include "tap.h"

static unsigned long fired;
static int fired_cpu = -1;

static void on_affinity_lost(int cpu, void *priv)
{
	uatomic_inc(&fired);
	fired_cpu = cpu;
	*(int *) priv = 1;
}

static unsigned long cb_ran;

static void noop_cb(struct rcu_head *head)
{
	free(head);
	uatomic_inc(&cb_ran);
}

/*
 * CHILD ROLE, under LD_PRELOAD.  The shim lets the first pin through, so the
 * worker starts on the target cpu for real and only then loses it.  Drive
 * enough callbacks to cross SET_AFFINITY_CHECK_PERIOD (256) so the work loop
 * re-checks its affinity, and report through the exit status.
 */
static int run_fault_child(int cpu)
{
	struct call_rcu_data *crdp;
	struct urcu_affinity_notifier n;
	int seen = 0, i;

	urcu_memb_register_thread();
	crdp = urcu_memb_create_call_rcu_data(0, cpu);
	if (!crdp) {
		fprintf(stderr, "child: create_call_rcu_data(%d) failed\n", cpu);
		return 1;
	}
	memset(&n, 0, sizeof(n));
	n.fct = on_affinity_lost;
	n.priv = &seen;
	if (urcu_memb_call_rcu_affinity_notifier_register(crdp, &n)) {
		fprintf(stderr, "child: notifier register failed\n");
		return 1;
	}
	urcu_memb_set_thread_call_rcu_data(crdp);
	/*
	 * The affinity re-check is rate-limited to once per
	 * SET_AFFINITY_CHECK_PERIOD (256) iterations of the WORK LOOP, and the
	 * worker splices the whole queue per iteration -- so a bulk enqueue of
	 * thousands of callbacks costs only a handful of iterations and never
	 * reaches the check.  Force one iteration per callback instead: enqueue
	 * one, wait for it to run, repeat.
	 */
	for (i = 0; i < 600 && !uatomic_load(&fired, CMM_RELAXED); i++) {
		struct rcu_head *h = malloc(sizeof(*h));
		unsigned long before;

		if (!h)
			break;
		before = uatomic_load(&cb_ran, CMM_RELAXED);
		urcu_memb_call_rcu(h, noop_cb);
		while (uatomic_load(&cb_ran, CMM_RELAXED) == before)
			caa_cpu_relax();
	}
	urcu_memb_barrier();
	urcu_memb_set_thread_call_rcu_data(NULL);
	(void) urcu_memb_call_rcu_affinity_notifier_unregister(crdp, &n);
	urcu_memb_unregister_thread();

	if (!uatomic_load(&fired, CMM_RELAXED)) {
		fprintf(stderr, "child: notifier never fired\n");
		return 2;
	}
	if (fired_cpu != cpu) {
		fprintf(stderr, "child: fired for cpu %d, want %d\n",
			fired_cpu, cpu);
		return 3;
	}
	return 0;
}

/* Is the cpuset controller actually delegated to us? */
static int cpuset_available(void)
{
	char path[512], line[512];
	FILE *f;
	char *p;

	f = fopen("/proc/self/cgroup", "r");
	if (!f)
		return 0;
	if (!fgets(line, sizeof(line), f)) {
		fclose(f);
		return 0;
	}
	fclose(f);
	p = strchr(line, ':');
	if (!p || !(p = strchr(p + 1, ':')))
		return 0;
	p++;
	p[strcspn(p, "\n")] = '\0';
	snprintf(path, sizeof(path), "/sys/fs/cgroup%s/cgroup.controllers", p);
	f = fopen(path, "r");
	if (!f)
		return 0;
	line[0] = '\0';
	if (!fgets(line, sizeof(line), f))
		line[0] = '\0';
	fclose(f);
	return strstr(line, "cpuset") != NULL;
}

int main(void)
{
	struct call_rcu_data *crdp;
	struct urcu_affinity_notifier n;
	int seen = 0, ret;
	long conf = sysconf(_SC_NPROCESSORS_CONF);
	const char *child_cpu = getenv("URCU_TEST_FAULT_CPU");
	char selfpath[512];
	ssize_t len;

	/* child role: no TAP, status only */
	if (child_cpu)
		return run_fault_child(atoi(child_cpu));

	plan_no_plan();

	if (conf < 1 || conf >= CPU_SETSIZE) {
		skip(4, "no usable cpu range");
		return exit_status();
	}

	/* 1. a cpu the machine cannot have is refused outright */
	crdp = urcu_memb_create_call_rcu_data(0, (int) conf);
	ok(crdp == NULL && errno == EINVAL,
		"create_call_rcu_data() REFUSES an impossible cpu (%ld) "
		"instead of starting a worker that can never pin",
		conf);
	if (crdp)
		urcu_memb_call_rcu_data_free(crdp);

	/* 2. register/unregister on a normal per-cpu worker */
	urcu_memb_register_thread();
	crdp = urcu_memb_create_call_rcu_data(0, 0);
	ok(crdp != NULL, "create_call_rcu_data() accepts a real cpu");
	if (crdp) {
		memset(&n, 0, sizeof(n));
		n.fct = on_affinity_lost;
		n.priv = &seen;
		ret = urcu_memb_call_rcu_affinity_notifier_register(crdp, &n);
		ok(ret == 0 && !uatomic_load(&fired, CMM_RELAXED),
			"notifier registers and does NOT fire for a healthy cpu");
		ret = urcu_memb_call_rcu_affinity_notifier_unregister(crdp, &n);
		ok(ret == 0, "notifier unregisters");
	} else {
		skip(2, "no worker to register on");
	}
	urcu_memb_unregister_thread();

	/* 3. the runtime transition, via fault injection in a fresh process */
	len = readlink("/proc/self/exe", selfpath, sizeof(selfpath) - 1);
	if (len <= 0) {
		skip(1, "cannot find own path to re-exec under LD_PRELOAD");
		return exit_status();
	}
	selfpath[len] = '\0';
	{
		char preload[640], *slash;
		pid_t pid;
		int status = -1;

		snprintf(preload, sizeof(preload), "%s", selfpath);
		slash = strrchr(preload, '/');
		if (slash)
			snprintf(slash + 1, sizeof(preload) - (size_t) (slash + 1 - preload),
				"libaffinity_fault.so");
		if (access(preload, R_OK) != 0) {
			skip(1, "fault-injection shim not built (%s)", preload);
			diag("cpuset delegated: %s",
				cpuset_available() ? "yes" : "no");
			return exit_status();
		}
		pid = fork();
		if (pid == 0) {
			setenv("LD_PRELOAD", preload, 1);
			setenv("URCU_TEST_FAULT_CPU", "0", 1);
			execl(selfpath, selfpath, (char *) NULL);
			_exit(127);
		}
		if (pid < 0 || waitpid(pid, &status, 0) < 0) {
			skip(1, "fork/waitpid failed");
			return exit_status();
		}
		ok(WIFEXITED(status) && WEXITSTATUS(status) == 0,
			"notifier fires on the RUNTIME transition: worker starts "
			"on its cpu, then cannot re-pin (exit %d)",
			WIFEXITED(status) ? WEXITSTATUS(status) : -1);
	}
	diag("cpuset delegated: %s (would give a real excluded cpu)",
		cpuset_available() ? "yes" : "no");
	return exit_status();
}
