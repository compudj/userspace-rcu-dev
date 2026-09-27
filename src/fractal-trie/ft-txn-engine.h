// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_ENGINE_H
#define _FT_TXN_ENGINE_H

/*
 * THE COMMIT ENGINE AS FT SEES IT: the parked-slot format.
 *
 * While a commit is between install and settle, each of its slots holds a
 * PROXY: a tagged pointer to a small engine object that resolves to the
 * slot's old or new value, depending on ONE decision shared by every proxy
 * of that commit.  Readers resolve through that decision, and a reader that
 * needs two slots to agree compares the two proxies' decisions.
 *
 * FT reads parked slots only through this file, so that an engine swap
 * re-implements it and nothing else.  Here it is the MCAS engine
 * (<urcu/rcu-txn.h>): a proxy is (struct urcu_txn_record * | tag), and its
 * decision is the record's descriptor status word.  In the SW engine
 * (<urcu/rcu-txn-sw.h>) a proxy is a latch's proxy and its decision is the
 * flip group's selector.  Both are 16-byte aligned, which FT's 0xF tag
 * needs (FT_FLIP_PROXY_TAG).
 */

#define FT_TXN_INLINE	static inline __attribute__((always_inline))

/* A parked slot's proxy, with its tag stripped.  Opaque outside this file. */
struct ft_txn_parked;

/* The decision every proxy of one commit resolves through. */
struct ft_txn_decision;

/* Does @v, loaded from a slot parked under @tag, hold a proxy? */
FT_TXN_INLINE
int ft_txn_is_proxy(const void *v, uintptr_t tag)
{
	return urcu_txn_is_proxy((void *) (uintptr_t) v, tag);
}

/* The proxy @v holds; @v must be one (ft_txn_is_proxy). */
FT_TXN_INLINE
struct ft_txn_parked *ft_txn_parked_of(const void *v, uintptr_t tag)
{
	return (struct ft_txn_parked *) urcu_txn_untag((void *) (uintptr_t) v,
			tag);
}

FT_TXN_INLINE
const struct ft_txn_decision *ft_txn_parked_decision(
		const struct ft_txn_parked *p)
{
	return (const struct ft_txn_decision *)
		((const struct urcu_txn_record *) p)->desc;
}

/*
 * Has the commit behind @d taken effect?  Acquire: a true answer makes the
 * new values' targets visible.  False while the commit is still in flight,
 * and after it failed.
 */
FT_TXN_INLINE
bool ft_txn_decision_committed(const struct ft_txn_decision *d)
{
	return urcu_txn_desc_status((const struct urcu_txn_desc *) d) ==
		URCU_TXN_DESC_SUCCEEDED;
}

/* The value @p stands for under a decision the caller already read. */
FT_TXN_INLINE
void *ft_txn_parked_value(const struct ft_txn_parked *p, bool committed)
{
	const struct urcu_txn_record *r = (const struct urcu_txn_record *) p;

	return committed ? r->new_ptr : r->old_ptr;
}

/* The value @p stands for now. */
FT_TXN_INLINE
void *ft_txn_parked_resolve(const struct ft_txn_parked *p)
{
	return urcu_txn_resolve_record((struct urcu_txn_record *)
			(uintptr_t) p);
}

/* A value loaded from a slot parked under @tag, proxy resolved. */
FT_TXN_INLINE
void *ft_txn_resolve(const void *v, uintptr_t tag)
{
	return urcu_txn_resolve((void *) (uintptr_t) v, tag);
}

/* Acquire-load @slot, parked under @tag, and resolve it. */
FT_TXN_INLINE
void *ft_txn_read(void **slot, uintptr_t tag)
{
	return urcu_txn_read(slot, tag);
}

#endif /* _FT_TXN_ENGINE_H */
