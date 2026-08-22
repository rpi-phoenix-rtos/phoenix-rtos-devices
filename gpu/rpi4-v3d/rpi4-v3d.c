/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server (/dev/v3d-srv)
 *
 * The V3D GPU has a single global MMU page-table base, one set of submit/CT
 * registers and one power/reset domain, none arbitrated in hardware. Two
 * processes driving it concurrently silently corrupt each other's results and
 * time out (M0 HW-confirmed, see the concurrent-gpu feasibility doc). This
 * daemon is the SOLE owner of the GPU: it will power the block on once, map
 * HUB/CORE0, own the single MMU page table + GPU-VA allocator + BO table, and
 * be the only code that writes V3D registers. Clients link libv3d-client and
 * route the MMIO-touching ioctls here as mtDevCtl messages; because a Phoenix
 * server handles one message at a time, every submit is serialized for free
 * (exactly how rpi4-vcmbox de-races the single VideoCore mailbox FIFO).
 *
 * ===========================================================================
 * M1 step 2a+2b+2c - GPU OWNERSHIP + BO LIFECYCLE + FULL SUBMIT (CSD/CL/TFU).
 * ===========================================================================
 * main() takes exclusive ownership of the GPU before serving (v3d_gpu_init:
 * power-on once, map HUB/CORE0, install the single flat MMU page table + VA
 * allocator + BO table). The GPU-owning + BO + submit logic lives in v3d_gpu.c,
 * copied essentially verbatim from the in-process winsys (state struct W,
 * power-on/reset, apply_core_regs, va_alloc, BO table, ioc_create_bo/close, and
 * ioc_submit_csd/cl/tfu + their cache-flush + binner-overflow + wedge-reset
 * helpers); this file is the RPC/message-loop half only. CREATE_BO /
 * GET_BO_OFFSET / MMAP_BO / GEM_CLOSE hand back the BO physical address so a
 * client can share the BO via mmap(MAP_PHYSMEM). SUBMIT_CSD forwards the 7
 * dispatch words in msg.i.raw; SUBMIT_CL/TFU forward the (larger) drm_v3d_submit_*
 * descriptor in msg.i.data - no bo_handle array (the synchronous submit consumes
 * only the descriptor's baked GPU-VA scalars, never the handles).
 *
 * The in-process winsys stays UNTOUCHED and keeps working; this daemon is a
 * separate, opt-in component.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/types.h>
#include <posix/utils.h>

#include <string.h>

#include "v3d_drm.h"   /* vendored UAPI: drm_v3d_submit_cl / drm_v3d_submit_tfu */
#include "v3d_rpc.h"
#include "v3d_gpu.h"


/* ------------------------------------------------------------------------- */
/* BO-management opcode handlers (M1 step 2a).                               */
/*                                                                           */
/* Each decodes the request from msg.i.raw (v3d_rpc_req_t), calls into the   */
/* GPU-owning core (v3d_gpu.c) and fills the response in msg.o.raw           */
/* (v3d_rpc_resp_t). resp->err is 0 or a negative errno. SUBMIT_* remain     */
/* stubs; step 2b lifts the submit path.                                     */
/* ------------------------------------------------------------------------- */

static void v3d_srv_createBo(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	v3d_gpu_bo_t bo;
	resp->err = v3d_gpu_createBo(req->size, req->flags, &bo);
	if (resp->err == 0) {
		resp->handle = bo.handle;
		resp->pa = bo.pa;
		resp->size = bo.size;
		resp->gpuva = bo.gpuva;
	}
}


static void v3d_srv_getBoOffset(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	resp->err = v3d_gpu_getBoOffset(req->handle, &resp->gpuva);
}


static void v3d_srv_mmapBo(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	resp->err = v3d_gpu_mmapBo(req->handle, &resp->pa, &resp->size);
}


static void v3d_srv_gemClose(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	resp->err = v3d_gpu_closeBo(req->handle);
}


/* SUBMIT_CSD: the 7 dispatch words ride in req->cfg (no i.data). The dispatch is
 * synchronous, so resp->err reflects completion. */
static void v3d_srv_submitCsd(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	resp->err = v3d_gpu_submitCsd(req->cfg);
}


/* SUBMIT_CL: the drm_v3d_submit_cl descriptor rides in msg.i.data (too big for
 * i.raw). No bo_handle array is appended - the synchronous submit consumes only
 * the descriptor's baked GPU-VA scalars, never the handles. */
static void v3d_srv_submitCl(const v3d_rpc_req_t *req, const msg_t *msg, v3d_rpc_resp_t *resp)
{
	struct drm_v3d_submit_cl cl;

	if (msg->i.data == NULL || req->desc_size != sizeof(cl) || msg->i.size < sizeof(cl)) {
		resp->err = -EINVAL;
		return;
	}
	memcpy(&cl, msg->i.data, sizeof(cl));
	resp->err = v3d_gpu_submitCl(&cl);
}


/* SUBMIT_TFU: the drm_v3d_submit_tfu descriptor rides in msg.i.data. */
static void v3d_srv_submitTfu(const v3d_rpc_req_t *req, const msg_t *msg, v3d_rpc_resp_t *resp)
{
	struct drm_v3d_submit_tfu tfu;

	if (msg->i.data == NULL || req->desc_size != sizeof(tfu) || msg->i.size < sizeof(tfu)) {
		resp->err = -EINVAL;
		return;
	}
	memcpy(&tfu, msg->i.data, sizeof(tfu));
	resp->err = v3d_gpu_submitTfu(&tfu);
}


/* Decode one forwarded ioctl (mtDevCtl) and dispatch to the matching stub. */
static void v3d_srv_handleMsg(const msg_t *msg, v3d_rpc_resp_t *resp)
{
	const v3d_rpc_req_t *req = (const v3d_rpc_req_t *)msg->i.raw;

	resp->err = -ENOSYS;
	resp->handle = 0;
	resp->pa = 0;
	resp->size = 0;
	resp->gpuva = 0;

	switch (req->op) {
		case V3D_RPC_CREATE_BO:
			v3d_srv_createBo(req, resp);
			break;
		case V3D_RPC_GET_BO_OFFSET:
			v3d_srv_getBoOffset(req, resp);
			break;
		case V3D_RPC_MMAP_BO:
			v3d_srv_mmapBo(req, resp);
			break;
		case V3D_RPC_GEM_CLOSE:
			v3d_srv_gemClose(req, resp);
			break;
		case V3D_RPC_SUBMIT_CL:
			v3d_srv_submitCl(req, msg, resp);
			break;
		case V3D_RPC_SUBMIT_TFU:
			v3d_srv_submitTfu(req, msg, resp);
			break;
		case V3D_RPC_SUBMIT_CSD:
			v3d_srv_submitCsd(req, resp);
			break;
		default:
			printf("rpi4-v3d: unknown op %u\n", req->op);
			resp->err = -EINVAL;
			break;
	}
}


static void v3d_srv_thread(uint32_t port)
{
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			break; /* invalid/closed port or OOM - fatal */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtRead:
			case mtWrite:
				/* Control node: driven via mtDevCtl only. */
				msg.o.err = -EINVAL;
				break;

			case mtDevCtl:
				/* o.err is the IPC-layer status; the per-op status lives in the
				 * v3d_rpc_resp_t in o.raw. EOK = the message was handled. */
				v3d_srv_handleMsg(&msg, (v3d_rpc_resp_t *)msg.o.raw);
				msg.o.err = EOK;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


int main(int argc, char **argv)
{
	uint32_t port;
	oid_t dev;
	int rc;

	(void)argc;
	(void)argv;

	/* Claim the /dev/v3d-srv node FIRST: create_dev is the single-owner guard. A
	 * second server instance must fail here BEFORE it powers on the GPU / installs
	 * its own MMU page table - otherwise it would clobber the single global
	 * MMU_PT_PA_BASE (exactly the conflict this daemon exists to prevent) before the
	 * duplicate-node error fired. Clients that msgSend before the serve loop starts
	 * simply block until we enter msgRecv. */
	if (portCreate(&port) != EOK) {
		printf("rpi4-v3d: portCreate failed\n");
		return 1;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, V3D_RPC_DEV_NAME) < 0) {
		printf("rpi4-v3d: could not create /dev/%s (already owned?)\n", V3D_RPC_DEV_NAME);
		return 2;
	}

	/* Now take exclusive ownership of the GPU: power-on once, map HUB/CORE0, install
	 * the single MMU flat page table, arm the GPU-VA allocator and BO table. Fail
	 * loud - a half-owned GPU must not start accepting clients. */
	rc = v3d_gpu_init();
	if (rc != 0) {
		printf("rpi4-v3d: GPU init failed (%d); refusing to serve\n", rc);
		return 3;
	}

	printf("rpi4-v3d: registered /dev/%s (GPU owned; BO + CSD/CL/TFU submit live)\n",
		V3D_RPC_DEV_NAME);

	v3d_srv_thread(port);

	return 0;
}
