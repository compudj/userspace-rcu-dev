// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_INTERNAL_H
#define _URCU_RCU_TXN_SW_INTERNAL_H

/*
 * The part of <urcu/rcu-txn-sw.h> that is compiled into liburcu-common: the
 * steps of a commit that installs proxies.  See urcu_txn_sw__commit_desc() in
 * urcu-txn.c.
 *
 * Not installed.  The unit tests include it as well, to stage a transaction
 * between two steps.
 */

#include <urcu/rcu-txn-sw.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Commit the flip group: switch every record in the transaction from old to new
 * with a single store-release.  Must be called once every record is installed:
 * the store-release orders the installs before the flip.
 */
static inline
void urcu_txn_sw__group_commit(struct urcu_txn_sw_group *group)
{
	/*
	 * Store-release (B) pairs with the load-acquire (A) of
	 * urcu_txn_sw__proxy_resolve().
	 */
	uatomic_store(&group->selector, 1, CMM_RELEASE);
}

/*
 * Install record @r's tagged proxy into its slot, bound to flip group @group.
 * Readers resolve it to old until the group commits.  The installed value is
 * @r's address OR'd with the record's tag.
 */
static inline
void urcu_txn_sw__record_install(struct urcu_txn_sw_group *group,
		struct urcu_txn_sw_record *r)
{
	/*
	 * Installing requires a non-zero tag within URCU_TXN_SW_TAG_MASK, none
	 * of whose bits is set in the record's address.  Under a zero tag
	 * every plain value passes urcu_txn_sw_is_proxy(), and readers would
	 * resolve live values as proxies.  With a tag bit already set in the
	 * address, the OR changes nothing and urcu_txn_sw__untag() subtracts to
	 * a wrong address.
	 */
	urcu_assert_debug(r->tag && !(r->tag & ~URCU_TXN_SW_TAG_MASK));
	urcu_assert_debug(!((uintptr_t) r & r->tag));
	r->group = group;
	/* Store-release of slot pairs with rcu_dereference(). */
	uatomic_store(r->slot,
			(void *) ((uintptr_t) r | r->tag), CMM_RELEASE);
}

/*
 * The install and settle checks of URCU_TXN_SW_EXCL_VALIDATE: see
 * <urcu/rcu-txn-sw.h>.
 */
#ifdef URCU_TXN_SW_EXCL_VALIDATE

/* At settle, @r's slot must still hold the proxy this writer installed. */
static inline
void urcu_txn_sw__excl_slot_ours(struct urcu_txn_sw_record *r)
{
	void *want = (void *) ((uintptr_t) r | r->tag);
	void *v = uatomic_load(r->slot, CMM_RELAXED);

	if (v != want)
		urcu_txn_sw__excl_abort("slot=%p holds %p at settle, expected our installed proxy %p: a concurrent writer overwrote it\n",
			(void *) r->slot, v, want);
}

/*
 * At the multi-edge install, @r's slot must hold no proxy and must still hold
 * the recorded old.
 *
 * The value check catches a racing writer that ran a complete transaction on
 * the slot -- install, flip, settle -- between this transaction's record() and
 * its install.  That writer leaves a plain value behind, so the slot looks
 * free; installing over it would publish a stale old to readers, then settle
 * over the other writer's committed value.
 *
 * A correct single-writer program never trips it: this transaction has not
 * written the slot yet, the writer's earlier transactions settled before
 * returning, and chaining keeps the committed old in ptr[0].
 */
static inline
void urcu_txn_sw__excl_slot_installable(struct urcu_txn_sw_record *r)
{
	urcu_txn_sw__excl_slot_free(r->slot, r->tag, "install");
	urcu_txn_sw__excl_slot_unchanged(r, "install");
}

#else	/* !URCU_TXN_SW_EXCL_VALIDATE */

static inline
void urcu_txn_sw__excl_slot_ours(
		struct urcu_txn_sw_record *r __attribute__((unused)))
{
}

static inline
void urcu_txn_sw__excl_slot_installable(
		struct urcu_txn_sw_record *r __attribute__((unused)))
{
}

#endif	/* URCU_TXN_SW_EXCL_VALIDATE */

/*
 * Records must target pairwise-distinct slots: see urcu_txn_sw_record().  They
 * are in record order, hence the pairwise scan.  A debug build only.
 */
static inline
void urcu_txn_sw__assert_distinct_slots(
		const struct urcu_txn_sw_record *records, unsigned int nr)
{
	unsigned int i, j;

	for (i = 1; i < nr; i++)
		for (j = 0; j < i; j++)
			urcu_assert_debug(records[i].slot != records[j].slot);
}

/*
 * call_rcu callback: free a committed transaction's descriptor once the grace
 * period guarantees no reader still holds one of its proxies.
 */
static inline
void urcu_txn_sw__free_rcu(struct rcu_head *head)
{
	struct urcu_txn_sw_desc *desc = caa_container_of(head,
			struct urcu_txn_sw_desc, rcu_head);

	urcu_txn_sw__desc_free(desc);
}

/*
 * Retire the descriptor of a committed transaction.  A reader may still hold
 * one of its proxies, so it must outlive a grace period.  By default the slab
 * retires it in a batch, with one call_rcu per batch;
 * -DURCU_TXN_SLAB_NO_BATCH keeps one call_rcu per descriptor.  Only a slab
 * descriptor, under @flavor's own deferral, may take the batch route: see
 * urcu_txn_retire() in <urcu/rcu-txn-mcas.h>.
 */
static inline
void urcu_txn_sw__retire(struct urcu_txn_sw_desc *desc,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)),
		const struct rcu_flavor_struct *flavor)
{
#ifndef URCU_TXN_SLAB_NO_BATCH
	if (caa_likely(desc->slab && flavor &&
			flavor->update_call_rcu == call_rcu_fn) &&
			urcu_slab_free_pending(desc, flavor))
		return;
#else
	(void) flavor;
#endif
	call_rcu_fn(&desc->rcu_head, urcu_txn_sw__free_rcu);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SW_INTERNAL_H */
