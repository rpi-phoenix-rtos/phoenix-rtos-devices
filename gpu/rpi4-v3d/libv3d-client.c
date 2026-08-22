/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU client library
 *
 * Thin IPC veneer over the rpi4-v3d server (/dev/v3d-srv). Re-implements the
 * winsys phoenix_v3d_ioctl() entry point Mesa's libdrm shim calls: GET_PARAM /
 * WAIT_BO are served locally (constants / synchronous no-op, no MMIO), and the
 * MMIO-touching ioctls (CREATE_BO, GET_BO_OFFSET, MMAP_BO, GEM_CLOSE,
 * SUBMIT_CL/TFU/CSD) are packed into an mtDevCtl message and forwarded to the
 * daemon. All GPU state (power, MMU page table, GPU-VA allocator, BO table)
 * lives in the server; a client sees only this API. Patterned on libvcmbox.c.
 *
 * ===========================================================================
 * SCAFFOLD INCREMENT (M1 step 1) - RPC PLUMBING ONLY.
 * ===========================================================================
 * The message pack/send/unpack path is real and build-verified. The GPU-facing
 * halves that depend on a server that actually allocates BOs are marked
 * TODO(v3d-cli-gpu): the client-side handle table and the CREATE_BO/MMAP_BO
 * mmap(MAP_PHYSMEM) of the server-returned physical address are declared but not
 * wired, because the server returns -ENOSYS for every forwarded op this
 * increment (no runtime path exists to exercise them yet). The in-process
 * winsys (tools/v3d-driver-port/v3d_phoenix_winsys.c) is UNTOUCHED and remains
 * the working default; this library is an opt-in alternative backend.
 *
 * The include of the DRM UAPI headers (v3d_drm.h -> drm.h) mirrors the winsys:
 * the integrated GPU-app build already has drm-uapi/ on its include path, and
 * the standalone build-verify adds -I to tools/v3d-driver-port (+ its
 * shim-include for sys/ioccom.h).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <sys/msg.h>
#include <sys/types.h>

#include "v3d_drm.h"        /* DRM_V3D_* commands + drm_v3d_* arg structs (UAPI) */
#include "v3d_rpc.h"        /* shared client/server wire contract */
#include "libv3d-client.h"


/* Cached server node. Resolved once, then reused for the process lifetime. */
static struct {
	oid_t oid;
	int resolved;
} v3d_cli;


/*
 * TODO(v3d-cli-gpu): client-side BO table. When the server starts returning
 * real BOs, CREATE_BO must record {handle, pa, size, gpuva} here and
 * mmap(MAP_PHYSMEM, pa) the physical page for CPU access; MMAP_BO returns that
 * CPU VA; GEM_CLOSE munmaps + drops the entry. Not wired this increment (the
 * server returns -ENOSYS), so it is only declared, to fix the shape.
 *
 * struct v3d_cli_bo { uint32_t handle; uint64_t pa; void *cpu; uint32_t gpuva, size; };
 */


/* Resolve the server node directly through the `devfs` named port (pre-`bind
 * devfs /dev` fallback), mirroring libvcmbox_resolveViaDevfs. */
static int v3d_cli_resolveViaDevfs(oid_t *out)
{
	oid_t devfs;
	msg_t msg;

	if (lookup("devfs", NULL, &devfs) < 0) {
		return -ENOENT;
	}

	memset(&msg, 0, sizeof(msg));
	msg.type = mtLookup;
	msg.oid = devfs;
	msg.i.data = (void *)V3D_RPC_DEV_NAME;
	msg.i.size = sizeof(V3D_RPC_DEV_NAME);

	if (msgSend(devfs.port, &msg) < 0) {
		return -EIO;
	}
	if (msg.o.err < 0) {
		return msg.o.err;
	}

	*out = msg.o.lookup.dev;
	return 0;
}


/* Resolve /dev/v3d-srv with a bounded retry budget + pre-bind fallback. */
static int v3d_cli_resolve(void)
{
	int tries;

	if (v3d_cli.resolved != 0) {
		return 0;
	}

	for (tries = 0; tries < 50; tries++) {
		if (lookup("/dev/" V3D_RPC_DEV_NAME, NULL, &v3d_cli.oid) == 0) {
			v3d_cli.resolved = 1;
			return 0;
		}
		if (v3d_cli_resolveViaDevfs(&v3d_cli.oid) == 0) {
			v3d_cli.resolved = 1;
			return 0;
		}
		usleep(100 * 1000);
	}

	return -ETIMEDOUT;
}


/*
 * Send one control-only forwarded op (CREATE_BO / GET_BO_OFFSET / MMAP_BO /
 * GEM_CLOSE): request in i.raw, no i.data. Returns the server's per-op err and
 * copies the response descriptor out via *resp_out (when non-NULL).
 */
static int v3d_cli_call(const v3d_rpc_req_t *req, v3d_rpc_resp_t *resp_out)
{
	msg_t msg;
	const v3d_rpc_resp_t *resp = (const v3d_rpc_resp_t *)msg.o.raw;
	int err;

	if (v3d_cli_resolve() != 0) {
		return -EIO;
	}

	memset(&msg, 0, sizeof(msg));
	msg.type = mtDevCtl;
	msg.oid = v3d_cli.oid;
	memcpy(msg.i.raw, req, sizeof(*req));

	err = msgSend(v3d_cli.oid.port, &msg);
	if (err < 0) {
		return err;
	}
	if (resp_out != NULL) {
		memcpy(resp_out, resp, sizeof(*resp_out));
	}
	return resp->err;
}


/*
 * Send a SUBMIT_* op: request in i.raw, [descriptor][bo_handles[]] in i.data.
 * `desc`/`desc_size` is the drm_v3d_submit_* struct; `handles`/`count` is the
 * BO-handle array pulled out of the descriptor (client VAs in the descriptor's
 * own bo_handles pointer are not usable server-side - see v3d_rpc.h).
 */
static int v3d_cli_submit(uint32_t op, const void *desc, uint32_t desc_size,
	const uint32_t *handles, uint32_t count)
{
	msg_t msg;
	const v3d_rpc_resp_t *resp = (const v3d_rpc_resp_t *)msg.o.raw;
	v3d_rpc_req_t req;
	/* Flat marshaling buffer: descriptor + appended handle array. Sized for the
	 * largest of ALL THREE submit descriptors (CL/TFU/CSD all flow through here)
	 * plus the handle cap - so the desc_size guard below can never wrongly reject
	 * a valid TFU submit. */
#define V3D_MAX2(a, b) ((a) > (b) ? (a) : (b))
	uint8_t payload[V3D_MAX2(sizeof(struct drm_v3d_submit_cl),
		V3D_MAX2(sizeof(struct drm_v3d_submit_tfu), sizeof(struct drm_v3d_submit_csd)))];
#undef V3D_MAX2
	uint8_t buf[sizeof(payload) + V3D_RPC_MAX_BO_HANDLES * sizeof(uint32_t)];
	uint32_t total;
	int err;

	if (count > V3D_RPC_MAX_BO_HANDLES) {
		return -EINVAL;
	}
	if (desc_size > sizeof(payload)) {
		return -EINVAL;
	}
	if (v3d_cli_resolve() != 0) {
		return -EIO;
	}

	memcpy(buf, desc, desc_size);
	if ((count > 0) && (handles != NULL)) {
		memcpy(buf + desc_size, handles, count * sizeof(uint32_t));
	}
	total = desc_size + count * (uint32_t)sizeof(uint32_t);

	memset(&req, 0, sizeof(req));
	req.op = op;
	req.desc_size = desc_size;
	req.bo_handle_count = count;

	memset(&msg, 0, sizeof(msg));
	msg.type = mtDevCtl;
	msg.oid = v3d_cli.oid;
	memcpy(msg.i.raw, &req, sizeof(req));
	msg.i.data = buf;
	msg.i.size = total;

	err = msgSend(v3d_cli.oid.port, &msg);
	if (err < 0) {
		return err;
	}
	return resp->err;
}


/* ------------------------------------------------------------------------- */
/* GET_PARAM: served client-local (device-info constants, no MMIO). Copied    */
/* verbatim from the winsys ioc_get_param so a client that only creates a     */
/* screen (GET_PARAM-only) needs no server round-trip. Keep in sync with      */
/* tools/v3d-driver-port/v3d_phoenix_winsys.c:ioc_get_param.                  */
/* ------------------------------------------------------------------------- */
static int v3d_cli_getParam(struct drm_v3d_get_param *gp)
{
	switch (gp->param) {
		case DRM_V3D_PARAM_V3D_UIFCFG:             gp->value = 0x00000045; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT1:         gp->value = 0x000e1124; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT2:         gp->value = 0x00000100; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT3:         gp->value = 0x00000e00; return 0;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT0:       gp->value = 0x04443356; return 0; /* "V3D" */
		case DRM_V3D_PARAM_V3D_CORE0_IDENT1:       gp->value = 0x81001422; return 0;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT2:       gp->value = 0x40078121; return 0;
		case DRM_V3D_PARAM_SUPPORTS_TFU:           gp->value = 1; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CSD:           gp->value = 1; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CACHE_FLUSH:   gp->value = 1; return 0;
		case DRM_V3D_PARAM_SUPPORTS_MULTISYNC_EXT: gp->value = 1; return 0;
		case DRM_V3D_PARAM_SUPPORTS_PERFMON:       gp->value = 0; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CPU_QUEUE:     gp->value = 1; return 0;
		default:                                   gp->value = 0; return 0;
	}
}


int phoenix_v3d_ioctl(int fd, unsigned long request, void *arg)
{
	/* Mesa builds requests as DRM_IOWR(DRM_COMMAND_BASE + DRM_V3D_*, ...); strip
	 * the base to recover the DRM_V3D_* command (same as the winsys entry). */
	unsigned cmd = _IOC_NR(request) - DRM_COMMAND_BASE;

	(void)fd;

	/* Client-local: constants / synchronous no-op, never forwarded. */
	switch (cmd) {
		case DRM_V3D_GET_PARAM:
			return v3d_cli_getParam(arg);
		case DRM_V3D_WAIT_BO:
			return 0; /* submit is synchronous; the BO is already done */
	}

	/* DRM core GEM_CLOSE (NR 0x09, below DRM_COMMAND_BASE so not a DRM_V3D_*
	 * cmd): Mesa's bufmgr issues it to free a BO. Forward to the server, which
	 * owns the BO table. */
	if (_IOC_NR(request) == _IOC_NR(DRM_IOCTL_GEM_CLOSE)) {
		struct drm_gem_close *gc = arg;
		v3d_rpc_req_t req;
		memset(&req, 0, sizeof(req));
		req.op = V3D_RPC_GEM_CLOSE;
		req.handle = gc->handle;
		return v3d_cli_call(&req, NULL);
	}

	switch (cmd) {
		case DRM_V3D_CREATE_BO: {
			struct drm_v3d_create_bo *c = arg;
			v3d_rpc_req_t req;
			v3d_rpc_resp_t resp;
			int rc;
			memset(&req, 0, sizeof(req));
			req.op = V3D_RPC_CREATE_BO;
			req.size = c->size;
			req.flags = c->flags;
			rc = v3d_cli_call(&req, &resp);
			if (rc == 0) {
				c->handle = resp.handle;
				c->offset = resp.gpuva;
				/* TODO(v3d-cli-gpu): record {handle, resp.pa, resp.size, gpuva} in
				 * the client BO table and mmap(MAP_PHYSMEM, resp.pa) for CPU access
				 * so MMAP_BO can return the CPU VA. */
			}
			return rc;
		}
		case DRM_V3D_GET_BO_OFFSET: {
			struct drm_v3d_get_bo_offset *g = arg;
			v3d_rpc_req_t req;
			v3d_rpc_resp_t resp;
			int rc;
			memset(&req, 0, sizeof(req));
			req.op = V3D_RPC_GET_BO_OFFSET;
			req.handle = g->handle;
			rc = v3d_cli_call(&req, &resp);
			if (rc == 0) {
				g->offset = resp.gpuva;
			}
			return rc;
		}
		case DRM_V3D_MMAP_BO: {
			struct drm_v3d_mmap_bo *m = arg;
			v3d_rpc_req_t req;
			v3d_rpc_resp_t resp;
			int rc;
			memset(&req, 0, sizeof(req));
			req.op = V3D_RPC_MMAP_BO;
			req.handle = m->handle;
			rc = v3d_cli_call(&req, &resp);
			if (rc == 0) {
				/* TODO(v3d-cli-gpu): the winsys returns a CPU VA as the mmap offset;
				 * here that VA comes from the client-side MAP_PHYSMEM(resp.pa) set up
				 * in CREATE_BO. Placeholder: echo the (unusable) offset 0. */
				m->offset = 0;
			}
			return rc;
		}
		case DRM_V3D_SUBMIT_CL: {
			struct drm_v3d_submit_cl *s = arg;
			return v3d_cli_submit(V3D_RPC_SUBMIT_CL, s, sizeof(*s),
				(const uint32_t *)(uintptr_t)s->bo_handles, s->bo_handle_count);
		}
		case DRM_V3D_SUBMIT_TFU: {
			/* TFU handles are an inline bo_handles[4] in the descriptor itself, so
			 * no separate handle array is appended (count = 0); the server reads
			 * them from its copy of the descriptor. */
			struct drm_v3d_submit_tfu *t = arg;
			return v3d_cli_submit(V3D_RPC_SUBMIT_TFU, t, sizeof(*t), NULL, 0);
		}
		case DRM_V3D_SUBMIT_CSD: {
			struct drm_v3d_submit_csd *s = arg;
			return v3d_cli_submit(V3D_RPC_SUBMIT_CSD, s, sizeof(*s),
				(const uint32_t *)(uintptr_t)s->bo_handles, s->bo_handle_count);
		}
		default:
			return 0; /* perfmon etc.: no-op, matches the winsys */
	}
}
