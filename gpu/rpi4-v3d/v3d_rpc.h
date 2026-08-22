/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server - client/server RPC ABI
 *
 * The V3D GPU can be driven by exactly ONE process at a time: it has a single
 * global MMU page-table base register (MMU_PT_PA_BASE), a single set of
 * submit/CT registers and one power/reset domain, none of which are arbitrated
 * in hardware. Two processes touching the GPU concurrently silently corrupt
 * each other's results and time out (M0 HW-confirmed 2026-08-22, see
 * docs/inprogress/2026-08-22-concurrent-gpu-v3d-server-feasibility.md).
 *
 * The rpi4-v3d server (/dev/v3d-srv) is the sole owner of GPU power, the MMU
 * page table, the GPU-VA allocator and the BO table. GPU clients link the thin
 * libv3d-client, which re-implements the same phoenix_v3d_ioctl() entry point
 * Mesa's libdrm shim already calls, but routes the MMIO-touching ioctls to the
 * server as mtDevCtl messages. Because a Phoenix server processes one message
 * at a time, every submit is serialized for free - no two clients ever race the
 * GPU registers (mirrors the rpi4-vcmbox mailbox-serialization pattern).
 *
 * This header is the shared wire contract between rpi4-v3d.c (server) and
 * libv3d-client.c (client). It is deliberately minimal.
 *
 * ---------------------------------------------------------------------------
 * Wire format
 * ---------------------------------------------------------------------------
 *
 * A forwarded ioctl is one mtDevCtl message:
 *   - the fixed-size request descriptor (v3d_rpc_req_t) travels in msg.i.raw
 *     (the 64-byte inline payload);
 *   - the response descriptor (v3d_rpc_resp_t) travels in msg.o.raw;
 *   - for SUBMIT_* ops, the (larger) drm_v3d_submit_* descriptor plus the
 *     referenced BO-handle array travel in msg.i.data (the kernel copies that
 *     buffer across the address-space boundary). See "Submit marshaling" below.
 *
 * BOs are NOT copied over IPC. A BO is physically-contiguous DRAM; CREATE_BO
 * returns its PHYSICAL ADDRESS and the client maps the same physical page with
 * mmap(MAP_PHYSMEM) for CPU access. Command lists, vertex/texture data, etc.
 * all live inside BOs, so only the tiny submit descriptor ever crosses IPC -
 * this is why the per-submit overhead is a single message round-trip, not a
 * data copy. (Phoenix has no anonymous shared memory; PA + MAP_PHYSMEM is the
 * sharing mechanism used throughout the RPi4 driver set.)
 *
 * ---------------------------------------------------------------------------
 * RPC boundary (which ioctls are forwarded)
 * ---------------------------------------------------------------------------
 *
 *   GET_PARAM, WAIT_BO         -> handled CLIENT-LOCAL (constants / synchronous
 *                                 no-op; touch no MMIO), never forwarded.
 *   CREATE_BO                  -> forwarded; server does va_alloc + PT map and
 *                                 returns {handle, PA, size, GPU-VA}.
 *   GET_BO_OFFSET, MMAP_BO,
 *   GEM_CLOSE                  -> forwarded; server owns the BO table + VA list.
 *   SUBMIT_CL/TFU/CSD          -> forwarded; server executes synchronously and
 *                                 responds on completion.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3D_RPC_H_
#define _V3D_RPC_H_

#include <stdint.h>


/* devfs node the server registers and the client resolves. Chosen distinct
 * from any node the in-process winsys uses (it registers NONE - the in-process
 * path is a linked-in library, not a device server), and from a potential
 * future real DRM node under /dev/dri). */
#define V3D_RPC_DEV_NAME "v3d-srv"


/* Forwarded opcodes. GET_PARAM / WAIT_BO are intentionally absent: they are
 * served client-local and never reach the server. Values are the server's own
 * ABI (NOT the DRM_V3D_* command numbers) - the client translates from the DRM
 * ioctl NR to one of these before packing the request. */
enum v3d_rpc_op {
	V3D_RPC_CREATE_BO = 0,
	V3D_RPC_GET_BO_OFFSET,
	V3D_RPC_MMAP_BO,
	V3D_RPC_GEM_CLOSE,
	V3D_RPC_SUBMIT_CL,
	V3D_RPC_SUBMIT_TFU,
	V3D_RPC_SUBMIT_CSD,
	V3D_RPC_OP_COUNT
};


/*
 * Request descriptor - lives in msg.i.raw (64 B). Which fields are meaningful
 * depends on `op`:
 *   CREATE_BO      : size, flags
 *   GET_BO_OFFSET  : handle
 *   MMAP_BO        : handle
 *   GEM_CLOSE      : handle
 *   SUBMIT_CSD     : cfg[0..6] (the 7 CSD dispatch words; NO i.data - the CSD
 *                    dispatch is synchronous and consumes only cfg[], it never
 *                    dereferences the BO handles, so nothing rides i.data)
 *   SUBMIT_CL/TFU  : desc_size, bo_handle_count (payload rides msg.i.data)
 */
typedef struct {
	uint32_t op;               /* enum v3d_rpc_op */
	uint32_t handle;           /* GET_BO_OFFSET / MMAP_BO / GEM_CLOSE */
	uint32_t size;             /* CREATE_BO: requested byte size */
	uint32_t flags;            /* CREATE_BO: DRM create flags (e.g. SCANOUT bit) */
	uint32_t desc_size;        /* SUBMIT_CL/TFU: sizeof(drm_v3d_submit_*) at head of i.data */
	uint32_t bo_handle_count;  /* SUBMIT_CL/TFU: number of u32 handles appended in i.data */
	uint32_t cfg[7];           /* SUBMIT_CSD: the 7 CSD_QUEUED_CFG0..6 dispatch words */
} v3d_rpc_req_t;


/*
 * Response descriptor - lives in msg.o.raw (64 B). `err` is 0 or a negative
 * errno. Other fields are valid only when err == 0 and only for the ops noted:
 *   CREATE_BO      : handle, pa, size, gpuva
 *   GET_BO_OFFSET  : gpuva
 *   MMAP_BO        : pa, size   (client mmap(MAP_PHYSMEM, pa) for CPU access)
 *   GEM_CLOSE      : (err only)
 *   SUBMIT_*       : (err only) - submit is synchronous; err reflects completion
 */
typedef struct {
	int32_t  err;      /* 0 on success, negative errno on failure */
	uint32_t handle;   /* CREATE_BO: assigned GEM handle */
	uint64_t pa;       /* CREATE_BO / MMAP_BO: BO physical address (MAP_PHYSMEM).
	                    * 64-bit: a MAP_CONTIGUOUS BO may land above 4 GiB on an
	                    * 8 GB Pi 4. */
	uint32_t size;     /* CREATE_BO / MMAP_BO: BO byte size (page-rounded) */
	uint32_t gpuva;    /* CREATE_BO / GET_BO_OFFSET: assigned GPU VA (== drm offset) */
} v3d_rpc_resp_t;


/*
 * Submit marshaling (SUBMIT_CL / SUBMIT_TFU / SUBMIT_CSD).
 *
 * msg.i.raw carries v3d_rpc_req_t {op, desc_size, bo_handle_count}. msg.i.data
 * carries a flat buffer laid out as:
 *
 *     [ struct drm_v3d_submit_* descriptor ]  (desc_size bytes)
 *     [ uint32_t bo_handles[bo_handle_count] ] (bo_handle_count * 4 bytes)
 *
 * The descriptor's own __u64 pointer fields (drm_v3d_submit_cl.bo_handles,
 * .extensions, and the CSD/TFU equivalents) hold CLIENT virtual addresses and
 * are MEANINGLESS across the process boundary - the server MUST ignore them and
 * use the appended bo_handles[] array instead. All other descriptor fields are
 * GPU virtual addresses / immediates (bcl_start/end, rcl_start/end, qma/qms/qts,
 * the CSD cfg[]/coef[] words, the TFU register images) which are valid in the
 * server's single shared GPU-VA space - the server owns that space and every BO
 * is mapped in its one page table, so a client-supplied GPU VA resolves the same
 * way in the server. Sync-object fields are ignored (submit is synchronous).
 *
 * V3D_RPC_MAX_BO_HANDLES bounds the appended array so the marshaled buffer stays
 * within one message's i.data budget and the server can validate the count.
 */
#define V3D_RPC_MAX_BO_HANDLES 256u


/* Compile-enforced: both descriptors MUST fit the 64-byte msg.i.raw / msg.o.raw
 * inline payload. This is the header's central invariant. */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(v3d_rpc_req_t) <= 64, "v3d_rpc_req_t must fit msg.i.raw[64]");
_Static_assert(sizeof(v3d_rpc_resp_t) <= 64, "v3d_rpc_resp_t must fit msg.o.raw[64]");
#endif


#endif /* _V3D_RPC_H_ */
