/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server - GPU-owning + BO-management core
 *
 * Server-side GPU logic for the rpi4-v3d daemon. The implementation (v3d_gpu.c)
 * is copied essentially verbatim from the in-process winsys backend
 * (tools/v3d-driver-port/v3d_phoenix_winsys.c) + its power-on
 * (v3d_phoenix_power.c): the same state struct, BCM2711 power-on, apply_core_regs
 * (single global MMU_PT_PA_BASE + fault config), flat MMU page table, GPU-VA
 * bump/hole allocator, BO table and the CREATE_BO / GET_BO_OFFSET / MMAP_BO /
 * GEM_CLOSE ioctl bodies. All of that logic is process-local static state in the
 * winsys, so lifting it into the sole GPU-owning daemon is a copy, not a rewrite.
 *
 * The in-process winsys stays UNTOUCHED (the linked-in path keeps working); this
 * is a separate, byte-independent copy owned by the server.
 *
 * SCOPE (M1 step 2a): GPU ownership + BO lifecycle only. The submit path
 * (SUBMIT_CL/TFU/CSD) is NOT here yet - step 2b lifts ioc_submit_* and the
 * reset/overflow servicing.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3D_GPU_H_
#define _V3D_GPU_H_

#include <stdint.h>


/* Result of a CREATE_BO: what the server hands back to the client so the BO can
 * be shared by physical address (client mmap(MAP_PHYSMEM, pa)) and referenced in
 * GPU-VA space. Mirrors the meaningful fields of v3d_rpc_resp_t. */
typedef struct {
	uint32_t handle; /* assigned GEM handle (nonzero) */
	uint64_t pa;     /* BO physical address (may be > 4 GiB on an 8 GB Pi 4) */
	uint32_t size;   /* BO byte size (page-rounded) */
	uint32_t gpuva;  /* assigned GPU VA (== the DRM offset) */
} v3d_gpu_bo_t;


/*
 * Take exclusive ownership of the GPU: BCM2711 power-on, map HUB/CORE0, install
 * the single flat MMU page table + fault config, arm the GPU-VA allocator and BO
 * table (matching the winsys winsys_init ordering exactly). Idempotent. Returns
 * 0 on success or a negative errno; a nonzero return means the GPU is only
 * half-owned and the daemon must NOT start serving clients.
 */
int v3d_gpu_init(void);

/* CREATE_BO: allocate a page-rounded, physically-contiguous BO, map it into the
 * one GPU page table at a freshly-allocated GPU VA, and return {handle, pa, size,
 * gpuva}. `flags` are the DRM_V3D create flags (bit0 CACHEABLE, bit1 SCANOUT).
 * Returns 0 or a negative errno. */
int v3d_gpu_createBo(uint32_t size, uint32_t flags, v3d_gpu_bo_t *out);

/* GET_BO_OFFSET: return the BO's assigned GPU VA. -EINVAL if the handle is
 * unknown. */
int v3d_gpu_getBoOffset(uint32_t handle, uint32_t *gpuva);

/* MMAP_BO: return the BO's physical address + size so the client can
 * mmap(MAP_PHYSMEM, pa) it for CPU access (the server owns no per-client CPU
 * mapping). -EINVAL if the handle is unknown. */
int v3d_gpu_mmapBo(uint32_t handle, uint64_t *pa, uint32_t *size);

/* GEM_CLOSE: free the BO's slot + reclaim its GPU-VA range. Returns 0 (also for
 * an already-freed / never-ours handle, matching the winsys). */
int v3d_gpu_closeBo(uint32_t handle);

/* SUBMIT_CSD: run one synchronous compute-shader dispatch from the 7 CSD config
 * words (cfg[0..6] == CSD_QUEUED_CFG0..6; the CFG0 write kicks the job). Blocks
 * on INT_CSDDONE, then cleans the compute's dirty caches to DRAM so the output
 * BO is CPU-visible. The dispatch consumes only cfg[]; it never dereferences the
 * BO handles (Phoenix submit is synchronous, no async fencing). Returns 0. */
int v3d_gpu_submitCsd(const uint32_t cfg[7]);


#endif /* _V3D_GPU_H_ */
