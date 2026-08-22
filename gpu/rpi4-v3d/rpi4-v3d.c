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
 * SCAFFOLD INCREMENT (M1 step 1) - IPC + RPC ONLY, NO GPU LOGIC YET.
 * ===========================================================================
 * This file currently implements the message-port server skeleton and the RPC
 * decode/dispatch only. Every opcode handler is a STUB that logs and returns
 * -ENOSYS. NONE of the GPU bring-up exists yet: no v3d_phoenix_powerOn(), no
 * MMIO map, no MMU page table, no VA allocator, no BO table, no real submit.
 *
 * The NEXT increment must (see TODO(v3d-srv-gpu) markers below):
 *   - move the winsys state struct `W`, winsys_init()/apply_core_regs()/power,
 *     va_alloc, the BO table and the ioc_* bodies (ioc_create_bo,
 *     ioc_close_bo, ioc_submit_cl/tfu/csd) from tools/v3d-driver-port/
 *     v3d_phoenix_winsys.c into this server essentially verbatim - they already
 *     ARE the server logic (all state is process-local static today);
 *   - wire the BO physical-address hand-off: CREATE_BO returns pa+gpuva+size,
 *     the client maps the PA with MAP_PHYSMEM;
 *   - decode the submit payload from msg.i.data ([descriptor][bo_handles[]]),
 *     rebind the descriptor's bo_handles pointer to the appended array, and run
 *     the existing synchronous submit.
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

#include "v3d_rpc.h"


/* ------------------------------------------------------------------------- */
/* Opcode handlers - STUBS for this increment.                               */
/*                                                                           */
/* Each logs the decoded request and returns -ENOSYS. The response `err` is  */
/* set here; the o.raw payload fields (pa/gpuva/handle/size) stay zero until  */
/* the real GPU logic lands. TODO(v3d-srv-gpu): replace each body with the   */
/* corresponding winsys ioc_* implementation.                               */
/* ------------------------------------------------------------------------- */

static void v3d_srv_createBo(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	printf("rpi4-v3d: CREATE_BO size=%u flags=0x%x (stub)\n", req->size, req->flags);
	/* TODO(v3d-srv-gpu): va_alloc + PT map; return {handle, pa, size, gpuva}. */
	resp->err = -ENOSYS;
}


static void v3d_srv_getBoOffset(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	printf("rpi4-v3d: GET_BO_OFFSET handle=%u (stub)\n", req->handle);
	/* TODO(v3d-srv-gpu): look up the BO; return its gpuva. */
	resp->err = -ENOSYS;
}


static void v3d_srv_mmapBo(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	printf("rpi4-v3d: MMAP_BO handle=%u (stub)\n", req->handle);
	/* TODO(v3d-srv-gpu): look up the BO; return its pa + size for MAP_PHYSMEM. */
	resp->err = -ENOSYS;
}


static void v3d_srv_gemClose(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp)
{
	printf("rpi4-v3d: GEM_CLOSE handle=%u (stub)\n", req->handle);
	/* TODO(v3d-srv-gpu): free the BO slot + reclaim its GPU-VA range. */
	resp->err = -ENOSYS;
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

	(void)argc;
	(void)argv;

	/* TODO(v3d-srv-gpu): before serving, take exclusive ownership of the GPU:
	 * v3d_phoenix_powerOn() once, map HUB/CORE0, allocate + install the single
	 * MMU flat page table, init the GPU-VA allocator and BO table. Fail loud if
	 * any step fails - a half-owned GPU must not start accepting clients. */

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

	printf("rpi4-v3d: registered /dev/%s (RPC scaffold; GPU logic not yet wired)\n",
		V3D_RPC_DEV_NAME);

	v3d_srv_thread(port);

	return 0;
}
