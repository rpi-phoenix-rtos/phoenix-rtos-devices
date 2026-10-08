/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - BO block pool trim
 *
 * Freed BO blocks wait in the server's pool (v3da_bo.c block_put) for a BO of the
 * same size and memory type. Without a bound that pool kept every block a client
 * ever freed (up to V3DA_MAX_POOL blocks), invisible to per-process accounting: a
 * WebKit session left ~400 MB held after the browser exited (C17). The pool is now
 * bounded two ways:
 *   - a byte cap on its footprint (what the kernel really holds: one buddy block,
 *     the next power of two, per MAP_CONTIGUOUS block - v3da_lowmem_footprint);
 *   - an idle limit: a block pooled that long without being reused goes back.
 * Over the cap the OLDEST blocks go first, blocks above the display limit before
 * those below it (a low block costs up to V3DA_LOWMEM_TRIES fresh blocks to find).
 *
 * Pure functions over a pool array and a release callback (no Phoenix headers, no
 * server state), so a host test checks exactly the policy the server applies.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3DA_POOL_H_
#define _V3DA_POOL_H_

#include <stddef.h>
#include <stdint.h>

#include "v3da_lowmem.h"   /* v3da_pool_block_t, v3da_lowmem_footprint, v3da_lowmem_is_low */


#define V3DA_POOL_CAP_MIB 64u   /* default -P: pooled footprint the server keeps */
#define V3DA_POOL_IDLE_S  10u   /* default -T: a block unused this long goes back */


/* Gives one block back (munmap in the server). */
typedef void (*v3da_pool_release_t)(void *ctx, const v3da_pool_block_t *b);

typedef struct {
	uint64_t bytes;      /* footprint released */
	uint32_t blocks;
	uint32_t idle;       /* ... of the blocks: past the idle limit */
	uint32_t cap;        /* ... over the byte cap */
} v3da_pool_freed_t;


static inline uint64_t v3da_pool_block_bytes(const v3da_pool_block_t *b, uint32_t page_size)
{
	return v3da_lowmem_footprint(b->pages, page_size);
}


/* The block to give back first when over the cap: the oldest above the display
 * limit, else the oldest. -1 = the pool is empty. */
static inline int v3da_pool_victim(const v3da_pool_block_t *pool, uint32_t n, uint32_t page_size)
{
	int best = -1, best_low = 1;
	uint32_t i;

	for (i = 0u; i < n; i++) {
		int low = v3da_lowmem_is_low((uint64_t)pool[i].pa, (uint64_t)pool[i].pages * page_size);

		if ((best < 0) || (low < best_low) || ((low == best_low) && (pool[i].freed_us < pool[best].freed_us))) {
			best = (int)i;
			best_low = low;
		}
	}
	return best;
}


static inline void v3da_pool_remove(v3da_pool_block_t *pool, uint32_t *n, uint64_t *bytes, uint32_t i,
	uint32_t page_size, v3da_pool_release_t release, void *ctx, v3da_pool_freed_t *freed)
{
	uint64_t foot = v3da_pool_block_bytes(&pool[i], page_size);

	release(ctx, &pool[i]);
	*bytes = (*bytes >= foot) ? (*bytes - foot) : 0u;
	freed->bytes += foot;
	freed->blocks++;
	pool[i] = pool[--(*n)];
}


/* Give back (1) every block pooled at least idle_us ago (idle_us 0: none), then
 * (2) blocks in v3da_pool_victim order until the footprint *bytes is at most cap.
 * *n and *bytes are updated; *freed says what went (zeroed first). */
static inline void v3da_pool_trim(v3da_pool_block_t *pool, uint32_t *n, uint64_t *bytes, uint64_t cap, uint64_t now_us,
	uint64_t idle_us, uint32_t page_size, v3da_pool_release_t release, void *ctx, v3da_pool_freed_t *freed)
{
	uint32_t i = 0u, before;
	int v;

	freed->bytes = 0u;
	freed->blocks = 0u;
	freed->idle = 0u;
	freed->cap = 0u;
	if (idle_us != 0u) {
		while (i < *n) {
			if ((now_us >= pool[i].freed_us) && ((now_us - pool[i].freed_us) >= idle_us)) {
				v3da_pool_remove(pool, n, bytes, i, page_size, release, ctx, freed);   /* slot i now holds another block */
				freed->idle++;
			}
			else {
				i++;
			}
		}
	}
	before = freed->blocks;
	while ((*bytes > cap) && ((v = v3da_pool_victim(pool, *n, page_size)) >= 0)) {
		v3da_pool_remove(pool, n, bytes, (uint32_t)v, page_size, release, ctx, freed);
	}
	freed->cap = freed->blocks - before;
}

#endif /* _V3DA_POOL_H_ */
