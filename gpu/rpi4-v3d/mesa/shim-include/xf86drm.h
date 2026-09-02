/* Minimal xf86drm.h shim for the v3d gallium driver on Phoenix (GLQuake Path C).
 * Declares only the libdrm surface the v3d driver uses. drmIoctl() dispatches into
 * the Phoenix winsys backend (phoenix_v3d_ioctl); syncobj/prime are stubbed
 * (synchronous submit, no BO sharing). Impls live in v3d_libdrm_shim.c (port task).
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#ifndef PHX_XF86DRM_H
#define PHX_XF86DRM_H
#include <stdint.h>

int phoenix_v3d_ioctl(int fd, unsigned long request, void *arg);
static inline int drmIoctl(int fd, unsigned long request, void *arg)
{ return phoenix_v3d_ioctl(fd, request, arg); }

/* DRM syncobj (fences) — stubbed for synchronous submit. Guarded: V3DV TUs also pull
 * in the real drm-uapi/drm.h which defines these. */
#ifndef DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL          (1u<<0)
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT   (1u<<1)
#endif
int drmSyncobjCreate(int fd, uint32_t flags, uint32_t *handle);
int drmSyncobjDestroy(int fd, uint32_t handle);
int drmSyncobjWait(int fd, uint32_t *handles, unsigned num_handles,
                   int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled);
int drmSyncobjImportSyncFile(int fd, uint32_t handle, int sync_file_fd);
int drmSyncobjExportSyncFile(int fd, uint32_t handle, int *sync_file_fd);
int drmPrimeHandleToFD(int fd, uint32_t handle, uint32_t flags, int *prime_fd);
int drmPrimeFDToHandle(int fd, int prime_fd, uint32_t *handle);
int drmGetCap(int fd, uint64_t capability, uint64_t *value);
char *drmGetRenderDeviceNameFromFd(int fd);
int drmIsKMS(int fd);

/* DRM caps queried by u_sync_provider.c (timeline-syncobj support probe). */
#ifndef DRM_CAP_SYNCOBJ
#define DRM_CAP_SYNCOBJ          0x13
#define DRM_CAP_SYNCOBJ_TIMELINE 0x14
#endif

/* ---- Vulkan (V3DV) additions ---- */

/* drmSyncobj* completion surface beyond the GL subset (impls in v3dv_libdrm_shim.c).
 * All synchronous-submit no-ops: created fences are immediately satisfiable. */
int drmSyncobjSignal(int fd, const uint32_t *handles, uint32_t handle_count);
int drmSyncobjReset(int fd, const uint32_t *handles, uint32_t handle_count);
int drmSyncobjQuery(int fd, const uint32_t *handles, uint64_t *points, uint32_t handle_count);
int drmSyncobjQuery2(int fd, const uint32_t *handles, uint64_t *points, uint32_t handle_count, uint32_t flags);
int drmSyncobjTimelineSignal(int fd, const uint32_t *handles, uint64_t *points, uint32_t handle_count);
int drmSyncobjTimelineWait(int fd, uint32_t *handles, uint64_t *points, unsigned num_handles,
                           int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled);
int drmSyncobjTransfer(int fd, uint32_t dst_handle, uint64_t dst_point,
                       uint32_t src_handle, uint64_t src_point, uint32_t flags);
int drmSyncobjHandleToFD(int fd, uint32_t handle, int *obj_fd);
int drmSyncobjFDToHandle(int fd, int obj_fd, uint32_t *handle);

#ifndef DRM_SYNCOBJ_WAIT_FLAGS_WAIT_DEADLINE
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_AVAILABLE (1u<<2)
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_DEADLINE  (1u<<3)
#endif

/* drmVersion (drmGetVersion) — minimal libdrm-compatible layout. */
typedef struct _drmVersion {
	int     version_major;
	int     version_minor;
	int     version_patchlevel;
	int     name_len;
	char   *name;
	int     date_len;
	char   *date;
	int     desc_len;
	char   *desc;
} drmVersion, *drmVersionPtr;

drmVersionPtr drmGetVersion(int fd);
void          drmFreeVersion(drmVersionPtr v);

/* drmDevice enumeration — node/bus enums + a minimal device struct. The real scan is
 * bypassed (patches/mesa/phoenix-rpi4-v3d.patch patches enumerate_devices), so these only need to
 * link; drmGetDevices2 returns 0 devices. */
#define DRM_NODE_PRIMARY 0
#define DRM_NODE_CONTROL 1
#define DRM_NODE_RENDER  2
#define DRM_NODE_MAX     3
#define DRM_BUS_PCI      0
#define DRM_BUS_USB      1
#define DRM_BUS_PLATFORM 2
#define DRM_BUS_HOST1X   3

typedef struct _drmDevice {
	char     **nodes;
	int        available_nodes;
	int        bustype;
	void      *businfo;
	void      *deviceinfo;
} drmDevice, *drmDevicePtr;

int  drmGetDevices2(uint32_t flags, drmDevicePtr *devices, int max_devices);
void drmFreeDevices(drmDevicePtr *devices, int count);
#endif
