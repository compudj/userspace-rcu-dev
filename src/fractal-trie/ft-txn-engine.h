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

/*
 * LOADS INSIDE A TXN.  ft_txn_load() returns @slot as the txn behind @h will
 * leave it: its own pending value when it recorded @slot, else the slot's
 * committed value.  ft_txn_load_committed() skips the pending value.
 *
 * A slot this txn did not record may hold ANOTHER commit's proxy, between
 * that commit's install and its settle.  The MCAS engine resolves it (and
 * waits while the other commit is undecided).  The SW engine's load assumes
 * a single updater and asserts that no such proxy exists.  So
 * -DFT_DEBUG_FOREIGN_PARKED counts, per call site and tag, the loads that
 * found one.
 */
#ifdef FT_DEBUG_FOREIGN_PARKED
# include <stdio.h>

struct ft_fp_site {
	uint64_t key;		/* 0: free; claimed once by cmpxchg */
	const char *file;
	int line;
	uintptr_t tag;
	unsigned long n;
};

# define FT_FP_SITES	128
static struct ft_fp_site ft_fp_sites[FT_FP_SITES];
static unsigned long ft_fp_loads, ft_fp_parked, ft_fp_dropped;

static inline
void ft_fp_note(struct urcu_txn *h, void **slot, uintptr_t tag,
		const char *file, int line)
{
	void *raw = uatomic_load(slot, CMM_RELAXED);
	uint64_t key;
	unsigned int i, at;

	uatomic_inc(&ft_fp_loads);
	if (!urcu_txn_is_proxy(raw, tag))
		return;
	if (h->desc && h->desc != URCU_TXN_ENOMEM &&
			urcu_txn_find(h->desc, slot))
		return;			/* recorded: the load returns our own value */
	uatomic_inc(&ft_fp_parked);
	key = ((uint64_t) (uintptr_t) file << 12) ^ ((uint64_t) line << 44) ^
		(uint64_t) tag ^ 1;
	at = (unsigned int) ((key ^ (key >> 29)) % FT_FP_SITES);
	for (i = 0; i < FT_FP_SITES; i++) {
		struct ft_fp_site *s = &ft_fp_sites[(at + i) % FT_FP_SITES];
		uint64_t k = uatomic_load(&s->key, CMM_ACQUIRE);

		if (!k && uatomic_cmpxchg(&s->key, 0, key) == 0) {
			s->file = file;
			s->line = line;
			s->tag = tag;
			k = key;
		}
		if (k == key) {
			uatomic_inc(&s->n);
			return;
		}
	}
	uatomic_inc(&ft_fp_dropped);
}

static void ft_fp_report(void) __attribute__((destructor));
static void ft_fp_report(void)
{
	unsigned int i;

	if (!ft_fp_loads)
		return;
	fprintf(stderr, "FT FOREIGN PARKED: %lu txn loads, %lu found another "
		"commit's proxy on a slot they did not record (%lu unsited)\n",
		ft_fp_loads, ft_fp_parked, ft_fp_dropped);
	for (i = 0; i < FT_FP_SITES; i++)
		if (ft_fp_sites[i].n)
			fprintf(stderr, "  %12lu  tag 0x%lx  %s:%d\n",
				ft_fp_sites[i].n,
				(unsigned long) ft_fp_sites[i].tag,
				ft_fp_sites[i].file, ft_fp_sites[i].line);
}

FT_TXN_INLINE
void *ft_txn_load_at(struct urcu_txn *h, void **slot, uintptr_t tag,
		bool committed, const char *file, int line)
{
	ft_fp_note(h, slot, tag, file, line);
	return committed ? urcu_txn_load_committed(h, slot, tag) :
		urcu_txn_load(h, slot, tag);
}
# define ft_txn_load(h, slot, tag)					\
	ft_txn_load_at((h), (slot), (tag), false, __FILE__, __LINE__)
# define ft_txn_load_committed(h, slot, tag)				\
	ft_txn_load_at((h), (slot), (tag), true, __FILE__, __LINE__)
#else
FT_TXN_INLINE
void *ft_txn_load(struct urcu_txn *h, void **slot, uintptr_t tag)
{
	return urcu_txn_load(h, slot, tag);
}

FT_TXN_INLINE
void *ft_txn_load_committed(struct urcu_txn *h, void **slot, uintptr_t tag)
{
	return urcu_txn_load_committed(h, slot, tag);
}
#endif

#endif /* _FT_TXN_ENGINE_H */
