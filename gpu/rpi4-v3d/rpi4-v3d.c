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
 * M1 step 2a - GPU OWNERSHIP + BO LIFECYCLE (submit still stubbed).
 * ===========================================================================
 * main() now takes exclusive ownership of the GPU before serving (v3d_gpu_init:
 * power-on once, map HUB/CORE0, install the single flat MMU page table + VA
 * allocator + BO table). The GPU-owning + BO logic lives in v3d_gpu.c, copied
 * essentially verbatim from the in-process winsys (state struct W, power-on,
 * apply_core_regs, va_alloc, BO table, ioc_create_bo/ioc_close_bo); this file is
 * the RPC/message-loop half only. The CREATE_BO / GET_BO_OFFSET / MMAP_BO /
 * GEM_CLOSE handlers call into v3d_gpu and hand back the BO physical address so
 * a client can share the BO via mmap(MAP_PHYSMEM).
 *
 * SUBMIT_CL/TFU/CSD stay stubs (return -ENOSYS): step 2b lifts ioc_submit_* and
 * the wedge reset/overflow servicing. The in-process winsys stays UNTOUCHED and
 * keeps working; this daemon is a separate, opt-in component.
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


static void v3d_srv_submit(const v3d_rpc_req_t *req, const msg_t *msg, v3d_rpc_resp_t *resp,
	const char *what)
{
	printf("rpi4-v3d: %s desc_size=%u bo_handle_count=%u data_size=%u (stub)\n",
		what, req->desc_size, req->bo_handle_count, (unsigned)msg->i.size);
	/* TODO(v3d-srv-gpu): validate bo_handle_count <= V3D_RPC_MAX_BO_HANDLES;
	 * read [descriptor][bo_handles[]] from msg->i.data; rebind the descriptor's
	 * bo_handles pointer to the appended array (client VAs are meaningless here);
	 * run the existing synchronous ioc_submit_* and report completion in err. */
	resp->err = -ENOSYS;
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
			v3d_srv_submit(req, msg, resp, "SUBMIT_CL");
			break;
		case V3D_RPC_SUBMIT_TFU:
			v3d_srv_submit(req, msg, resp, "SUBMIT_TFU");
			break;
		case V3D_RPC_SUBMIT_CSD:
			v3d_srv_submit(req, msg, resp, "SUBMIT_CSD");
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

	/* Take exclusive ownership of the GPU BEFORE serving: power-on once, map
	 * HUB/CORE0, install the single MMU flat page table, arm the GPU-VA allocator
	 * and BO table. Fail loud - a half-owned GPU must not start accepting clients. */
	rc = v3d_gpu_init();
	if (rc != 0) {
		printf("rpi4-v3d: GPU init failed (%d); refusing to serve\n", rc);
		return 3;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-v3d: portCreate failed\n");
		return 1;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, V3D_RPC_DEV_NAME) < 0) {
		printf("rpi4-v3d: could not create /dev/%s\n", V3D_RPC_DEV_NAME);
		return 2;
	}

	printf("rpi4-v3d: registered /dev/%s (GPU owned; BO management live, submit stubbed)\n",
		V3D_RPC_DEV_NAME);

	v3d_srv_thread(port);

	return 0;
}
