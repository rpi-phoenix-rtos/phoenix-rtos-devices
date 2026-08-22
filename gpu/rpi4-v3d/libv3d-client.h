/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU client library - API
 *
 * Thin IPC veneer over the rpi4-v3d server (/dev/v3d-srv). It exposes the exact
 * entry point Mesa's libdrm shim already calls - phoenix_v3d_ioctl(fd, request,
 * arg) - so a GPU app links this INSTEAD of the in-process winsys backend to
 * become a client of the single GPU-owning daemon. GET_PARAM / WAIT_BO are
 * served locally (constants / synchronous no-op); the MMIO-touching ioctls are
 * marshaled to the server (see v3d_rpc.h for the wire contract).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBV3D_CLIENT_H_
#define _LIBV3D_CLIENT_H_

/*
 * Drop-in replacement for the winsys phoenix_v3d_ioctl(): same signature, same
 * DRM_IOCTL_V3D_* / GEM_CLOSE request encoding in `request`, same `arg` structs
 * (drm_v3d_*). Returns 0 on success or a negative errno. `fd` is ignored (there
 * is no real DRM fd; the server node is resolved internally on first use).
 */
int phoenix_v3d_ioctl(int fd, unsigned long request, void *arg);

#endif /* _LIBV3D_CLIENT_H_ */
