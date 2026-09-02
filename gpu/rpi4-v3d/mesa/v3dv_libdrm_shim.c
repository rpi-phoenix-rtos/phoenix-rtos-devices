/*
 * v3dv_libdrm_shim.c — the EXTRA libdrm surface V3DV (Vulkan) needs beyond what
 * v3d_libdrm_shim.c (the GL gallium port) provides.
 *
 * Two groups, both consequences of V3DV's SYNCHRONOUS, single-client winsys:
 *
 * 1. drmSyncobj* completion surface. The Vulkan runtime reaches the kernel sync layer
 *    NOT via raw DRM_IOCTL_SYNCOBJ_* through drmIoctl, but via the libdrm C wrappers
 *    called from src/util/u_sync_provider.c (vk_drm_syncobj -> util_sync_provider_drm
 *    -> drmSyncobj{Create,Signal,Reset,Query2,Wait,Timeline*,Transfer,...}). Because
 *    SUBMIT_CL blocks on FLDONE/FRDONE inside the winsys, by the time any sync object
 *    is queried the GPU is idle and all results are in RAM -> every op reports
 *    "already signaled / passed". The GL shim already defines Create/Destroy/Wait/
 *    Import/Export; this file adds the rest of the provider vtable.
 *
 * 2. Device-enumeration surface (drmGetDevices2 / drmGetVersion / drmFree*). Phoenix
 *    has no /dev/dri node and no real libdrm enumeration. enumerate_devices is patched
 *    (patches/mesa/phoenix-rpi4-v3d.patch) to bypass the drmGetDevices2 scan and call
 *    create_physical_device(instance, -1, FAKE_FD, -1) directly, but the symbols must
 *    still resolve for the link, and drmGetVersion is also called from
 *    create_physical_device itself -> return a tiny static "v3d" version.
 *
 * NOTE: the drmSyncobjCreate/Destroy/Wait/ImportSyncFile/ExportSyncFile + drmPrime* +
 * drmGetCap + drmGetRenderDeviceNameFromFd symbols are defined in v3d_libdrm_shim.c
 * (reused from the GL port) and are NOT redefined here.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "xf86drm.h"
#include "xf86drmMode.h"

/* ---- drmSyncobj* completion surface (synchronous submit => always signaled) ---- */

int drmSyncobjSignal(int fd, const uint32_t *handles, uint32_t handle_count)
{
	(void)fd; (void)handles; (void)handle_count;
	return 0;
}

int drmSyncobjReset(int fd, const uint32_t *handles, uint32_t handle_count)
{
	(void)fd; (void)handles; (void)handle_count;
	return 0;
}

/* Report each queried syncobj as having reached a monotonically-passed point.
 * The runtime only compares the returned point against the one it waits for, and we
 * always satisfy waits, so any large constant is fine. */
int drmSyncobjQuery2(int fd, const uint32_t *handles, uint64_t *points,
                     uint32_t handle_count, uint32_t flags)
{
	(void)fd; (void)handles; (void)flags;
	if (points) {
		for (uint32_t i = 0; i < handle_count; i++)
			points[i] = ~(uint64_t)0;   /* already passed every requested point */
	}
	return 0;
}

int drmSyncobjQuery(int fd, const uint32_t *handles, uint64_t *points,
                    uint32_t handle_count)
{
	return drmSyncobjQuery2(fd, handles, points, handle_count, 0);
}

/* Timeline syncobjs: work is already done -> signaling is a no-op, waiting returns
 * immediately. */
int drmSyncobjTimelineSignal(int fd, const uint32_t *handles,
                             uint64_t *points, uint32_t handle_count)
{
	(void)fd; (void)handles; (void)points; (void)handle_count;
	return 0;
}

int drmSyncobjTimelineWait(int fd, uint32_t *handles, uint64_t *points,
                           unsigned num_handles, int64_t timeout_nsec,
                           unsigned flags, uint32_t *first_signaled)
{
	(void)fd; (void)handles; (void)points; (void)timeout_nsec; (void)flags;
	if (first_signaled && num_handles)
		*first_signaled = 0;
	return 0;   /* all points already reached */
}

int drmSyncobjTransfer(int fd, uint32_t dst_handle, uint64_t dst_point,
                       uint32_t src_handle, uint64_t src_point, uint32_t flags)
{
	(void)fd; (void)dst_handle; (void)dst_point;
	(void)src_handle; (void)src_point; (void)flags;
	return 0;
}

/* No cross-process/cross-device sharing on Phoenix (single client) -> no FD export. */
int drmSyncobjHandleToFD(int fd, uint32_t handle, int *obj_fd)
{
	(void)fd; (void)handle;
	if (obj_fd)
		*obj_fd = -1;
	return -1;
}

int drmSyncobjFDToHandle(int fd, int obj_fd, uint32_t *handle)
{
	(void)fd; (void)obj_fd; (void)handle;
	return -1;
}

/* ---- device enumeration surface ---- */

/* enumerate_devices is patched to skip the real scan, but drmGetDevices2 must still
 * link. Report zero devices (the patch never consults the array). */
int drmGetDevices2(uint32_t flags, drmDevicePtr *devices, int max_devices)
{
	(void)flags; (void)devices; (void)max_devices;
	return 0;
}

void drmFreeDevices(drmDevicePtr *devices, int count)
{
	(void)devices; (void)count;
}

/* Called from create_physical_device on the (fake) render fd. Return a small static
 * "v3d" version so the is_shim==false path is taken. */
static char v3dv_drm_name[] = "v3d";
static char v3dv_drm_date[] = "20180419";
static char v3dv_drm_desc[] = "Broadcom V3D (Phoenix)";

drmVersionPtr drmGetVersion(int fd)
{
	(void)fd;
	static drmVersion v;
	memset(&v, 0, sizeof(v));
	v.version_major = 1;
	v.version_minor = 0;
	v.name = v3dv_drm_name;
	v.name_len = (int)(sizeof(v3dv_drm_name) - 1);
	v.date = v3dv_drm_date;
	v.date_len = (int)(sizeof(v3dv_drm_date) - 1);
	v.desc = v3dv_drm_desc;
	v.desc_len = (int)(sizeof(v3dv_drm_desc) - 1);
	return &v;
}

void drmFreeVersion(drmVersionPtr v)
{
	(void)v;   /* static storage; nothing to free */
}

/* ---- KMS display-probe surface (bypassed VK_KHR_display path) ---- */

/* Report "no display resources" so try_display_device fails cleanly; the Phoenix
 * present path is the custom /dev/fb0 swapchain (Tier 4), not KMS. */
drmModeResPtr drmModeGetResources(int fd)
{
	(void)fd;
	return NULL;
}

void drmModeFreeResources(drmModeResPtr ptr)
{
	(void)ptr;
}

drmModeConnectorPtr drmModeGetConnector(int fd, uint32_t connectorId)
{
	(void)fd; (void)connectorId;
	return NULL;
}

void drmModeFreeConnector(drmModeConnectorPtr ptr)
{
	(void)ptr;
}
