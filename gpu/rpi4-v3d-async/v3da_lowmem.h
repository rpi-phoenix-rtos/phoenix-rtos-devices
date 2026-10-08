/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - scan-out
 * placement below 1 GiB (V3DA_BO_LOWMEM, protocol 5)
 *
 * The firmware display plane fetches nothing at or above 1 GiB, and the kernel's
 * page allocator (vm/page.c, a buddy allocator) takes no address constraint: a
 * MAP_CONTIGUOUS block lands wherever the first free block of its order is. So a
 * BO that may be scanned out takes, in this order:
 *   1. a pooled block of its size and memory type that lies below the limit;
 *   2. fresh blocks, up to V3DA_LOWMEM_TRIES of them, until one lies below the
 *      limit (rpi4-kms reserves its scan-out pool the same way, kms_pool_init).
 *      The rejects are held while trying, so that each try is a different block,
 *      and handed back to the kernel afterwards (safe since the E1 section 6
 *      object-tree fix, kernel d0fb0ca9 / build 8). If none lands low, the last
 *      one is kept: the BO is created anyway, and rpi4-kms refuses it at ADDFB2.
 * within a budget of low memory held by such BOs (the VideoCore, rpi4-kms's pool
 * and every other DMA user need the low GiB too). An ordinary BO takes a pooled
 * block ABOVE the limit when the pool has both, so pooled low blocks are kept for
 * the next scan-out BO.
 *
 * Pure functions over a pool array and an allocator callback (no Phoenix headers,
 * no server state), so tools/gpu-lane/v3d-async/hosttest/run.sh checks exactly the
 * policy the server applies.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3DA_LOWMEM_H_
#define _V3DA_LOWMEM_H_

#include <stddef.h>
#include <stdint.h>

#include "v3da_proto.h"   /* V3DA_LOWMEM_LIMIT */


#define V3DA_LOWMEM_TRIES      16u   /* fresh blocks tried per BO (kms_pool_init: 16) */
#define V3DA_LOWMEM_BUDGET_MIB 64u   /* default -L: eight 1080p XRGB buffers (one 8 MiB buddy block each) */


/* A block the server owns and has not handed out (v3da_bo.c block_put). */
typedef struct {
	void *cpu;
	uintptr_t pa;
	uint32_t pages;
	uint32_t cached;
	uint64_t freed_us;   /* when it entered the pool (the trim's age, v3da_pool.h) */
} v3da_pool_block_t;


/* What a `pages`-page MAP_CONTIGUOUS block really occupies: vm_objectContiguous
 * takes one buddy block, the next power of two (a 2026-page 1080p buffer: 8 MiB). */
static inline uint64_t v3da_lowmem_footprint(uint32_t pages, uint32_t page_size)
{
	uint64_t want = (uint64_t)pages * page_size, b = page_size;

	while (b < want) {
		b <<= 1;
	}
	return b;
}


/* May a LOWMEM BO of footprint `foot` be placed, with `live` bytes already held
 * against `budget`? (Overflow-safe; budget 0 = never.) */
static inline int v3da_lowmem_admit(uint64_t live, uint64_t foot, uint64_t budget)
{
	return (foot <= budget) && (live <= budget - foot);
}


/* Does [pa, pa + bytes) lie below the display limit? */
static inline int v3da_lowmem_is_low(uint64_t pa, uint64_t bytes)
{
	return (bytes != 0u) && (pa + bytes >= pa) && (pa + bytes <= V3DA_LOWMEM_LIMIT);
}


/* The pooled block a new BO takes, or -1. Exact size and memory type always.
 * want_low: only a block below the limit. Otherwise any, preferring one above the
 * limit - the low ones stay for the next scan-out BO. */
static inline int v3da_pool_pick(const v3da_pool_block_t *pool, uint32_t n, uint32_t pages, uint32_t cached,
	int want_low, uint32_t page_size)
{
	int any = -1;
	uint32_t i;

	for (i = 0u; i < n; i++) {
		int low;

		if ((pool[i].pages != pages) || (pool[i].cached != cached)) {
			continue;
		}
		low = v3da_lowmem_is_low((uint64_t)pool[i].pa, (uint64_t)pages * page_size);
		if (want_low != 0) {
			if (low != 0) {
				return (int)i;
			}
		}
		else if (low == 0) {
			return (int)i;
		}
		else if (any < 0) {
			any = (int)i;
		}
	}
	return (want_low != 0) ? -1 : any;
}


/* The allocator the placement loop drives: one fresh MAP_CONTIGUOUS block of
 * `bytes` (NULL = none), with the physical addresses of its first and last page
 * (the loop checks contiguity), and its release. */
typedef struct {
	void *(*map)(void *ctx, size_t bytes, int cached, uint64_t *pa_first, uint64_t *pa_last);
	void (*unmap)(void *ctx, void *cpu, size_t bytes);
	void *ctx;
} v3da_lowmem_ops_t;

typedef struct {
	uint32_t tries;      /* fresh blocks taken */
	uint32_t rejected;   /* ... handed back (above the limit, or not contiguous) */
	int low;             /* 1: the kept block lies below the limit */
} v3da_lowmem_result_t;


/* Fresh blocks until one lies below the limit (at most `tries`, clamped to
 * V3DA_LOWMEM_TRIES). Returns the kept block - the low one, else the last
 * contiguous one tried (res->low = 0) - with its physical address in *pa, or NULL
 * when the allocator gave nothing contiguous. Every other block is unmapped before
 * the return. */
static inline void *v3da_lowmem_map(const v3da_lowmem_ops_t *ops, size_t bytes, int cached, uint32_t tries,
	uint32_t page_size, uint64_t *pa, v3da_lowmem_result_t *res)
{
	void *held[V3DA_LOWMEM_TRIES];
	uint64_t held_pa[V3DA_LOWMEM_TRIES];
	int held_ok[V3DA_LOWMEM_TRIES];
	uint32_t n = 0u, i;
	int keep = -1;

	res->tries = 0u;
	res->rejected = 0u;
	res->low = 0;
	if (tries > V3DA_LOWMEM_TRIES) {
		tries = V3DA_LOWMEM_TRIES;
	}
	if ((bytes == 0u) || (page_size == 0u) || ((bytes % page_size) != 0u)) {
		return NULL;
	}
	while (n < tries) {
		uint64_t first = 0u, last = 0u;
		void *cpu = ops->map(ops->ctx, bytes, cached, &first, &last);

		if (cpu == NULL) {
			break;   /* out of contiguous memory of this order: stop, keep what we have */
		}
		res->tries++;
		held[n] = cpu;
		held_pa[n] = first;
		/* never hand a device a torn block, nor one va2pa could not resolve ((addr_t)-1) */
		held_ok[n] = ((first != UINT64_MAX) && (last == first + (uint64_t)bytes - page_size)) ? 1 : 0;
		if (held_ok[n] != 0) {
			keep = (int)n;   /* the last contiguous one: the fallback */
			if (v3da_lowmem_is_low(first, (uint64_t)bytes) != 0) {
				res->low = 1;
				n++;
				break;
			}
		}
		n++;
	}
	for (i = 0u; i < n; i++) {
		if ((int)i != keep) {
			ops->unmap(ops->ctx, held[i], bytes);
			res->rejected++;
		}
	}
	if (keep < 0) {
		return NULL;
	}
	*pa = held_pa[keep];
	return held[keep];
}

#endif /* _V3DA_LOWMEM_H_ */
