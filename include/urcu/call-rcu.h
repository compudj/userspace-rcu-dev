// SPDX-FileCopyrightText: 2009 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
// SPDX-FileCopyrightText: 2009 Paul E. McKenney, IBM Corporation.
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_CALL_RCU_H
#define _URCU_CALL_RCU_H

/*
 * Userspace RCU header - batch memory reclamation with kernel API
 *
 * This header is meant to be included indirectly through a liburcu
 * flavor header.
 */

#include <stdlib.h>
#include <pthread.h>

#include <urcu/wfcqueue.h>
#include <urcu/list.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Note that struct call_rcu_data is opaque to callers. */

struct call_rcu_data;

/* Flag values. */

#define URCU_CALL_RCU_RT	(1U << 0)
#define URCU_CALL_RCU_RUNNING	(1U << 1)
#define URCU_CALL_RCU_STOP	(1U << 2)
#define URCU_CALL_RCU_STOPPED	(1U << 3)
#define URCU_CALL_RCU_PAUSE	(1U << 4)
#define URCU_CALL_RCU_PAUSED	(1U << 5)

/*
 * The rcu_head data structure is placed in the structure to be freed
 * via call_rcu().
 */

struct rcu_head {
	struct cds_wfcq_node next;
	void (*func)(struct rcu_head *head);
};

/*
 * Exported functions
 *
 * Important: see rcu-api.md in userspace-rcu documentation for
 * call_rcu family of functions usage detail, including the surrounding
 * RCU usage required when using these primitives.
 */

void call_rcu(struct rcu_head *head,
	      void (*func)(struct rcu_head *head));

struct call_rcu_data *create_call_rcu_data(unsigned long flags,
					   int cpu_affinity);
void call_rcu_data_free(struct call_rcu_data *crdp);

struct call_rcu_data *get_default_call_rcu_data(void);
struct call_rcu_data *get_cpu_call_rcu_data(int cpu);
struct call_rcu_data *get_thread_call_rcu_data(void);
struct call_rcu_data *get_call_rcu_data(void);
pthread_t get_call_rcu_thread(struct call_rcu_data *crdp);

void set_thread_call_rcu_data(struct call_rcu_data *crdp);
int set_cpu_call_rcu_data(int cpu, struct call_rcu_data *crdp);

int create_all_cpu_call_rcu_data(unsigned long flags);
void free_all_cpu_call_rcu_data(void);

/*
 * Affinity-change notification.
 *
 * A per-cpu call_rcu worker re-pins itself to its cpu periodically, and
 * sched_setaffinity() reports EINVAL when that cpu can no longer be honoured --
 * hot-unplugged, or removed from the process by cpuset(7).  The worker is the
 * only thing in the library that both knows which cpu it stands for and finds
 * out that the cpu is gone, so it is where such news has to come from.
 *
 * Registered notifiers are invoked, once per departure, on the worker thread of
 * @crdp -- which by then is necessarily running on some OTHER cpu, so a
 * notifier may safely touch per-cpu state belonging to the departed cpu.  If
 * the cpu later returns and the re-pin succeeds, the notifier arms again.
 *
 * If the cpu is ALREADY known to be gone when a notifier registers, it is
 * invoked immediately, on the registering thread.  The worker pins itself once
 * at startup, so a crdp created for an unavailable cpu records the loss before
 * any caller could have registered; reporting it late beats never.  The caller
 * cannot itself be on the departed cpu -- the cpu is unavailable to the process
 * -- so notifiers see the same guarantee either way.
 *
 * A notifier runs with the crdp's notifier lock held: it MUST NOT call back
 * into the call_rcu API, and MUST NOT unregister itself.  Keep it to reclaiming
 * whatever the departed cpu owned.
 *
 * CAVEAT.  The check lives in the worker's callback loop, so it is only reached
 * while that worker has work.  In practice a departing cpu's queue still holds
 * callbacks from before the change, and the notification fires as they drain --
 * exactly when the per-cpu state was orphaned.  But a cpu that goes away with a
 * completely empty queue will never wake its worker, and no notification comes.
 * Treat this as a best-effort hint, not a guarantee.
 */
struct urcu_affinity_notifier {
	struct cds_list_head node;	/* internal: owned by the crdp */
	void (*fct)(int cpu, void *priv);
	void *priv;
};

int call_rcu_affinity_notifier_register(struct call_rcu_data *crdp,
				struct urcu_affinity_notifier *notifier);
int call_rcu_affinity_notifier_unregister(struct call_rcu_data *crdp,
				struct urcu_affinity_notifier *notifier);

void call_rcu_before_fork(void);
void call_rcu_after_fork_parent(void);
void call_rcu_after_fork_child(void);

void rcu_barrier(void);

#ifdef __cplusplus
}
#endif

#endif /* _URCU_CALL_RCU_H */
