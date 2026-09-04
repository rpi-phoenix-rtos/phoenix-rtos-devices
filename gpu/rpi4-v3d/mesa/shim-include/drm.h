/*
 * Phoenix-RTOS RPi4 V3D port: <drm.h> for the cross build.
 *
 * Mesa's v3dv_queue.c does `#include "drm.h"`, expecting the copy a Linux host
 * gets from libdrm-dev. Cross-compiling for Phoenix there is no system libdrm,
 * and the host's copy was reaching the CROSS compiler only because
 * scripts/build-showcase-apps.sh exports C_INCLUDE_PATH=/usr/include/libdrm for
 * the whole gpu phase (it is needed by the HOST meson/ninja codegen steps). So a
 * host x86 header was being compiled into the Phoenix driver, invisibly: it
 * appears in no compile command, and building the script standalone -- without
 * that variable -- fails with "drm.h: No such file or directory".
 *
 * Mesa vendors the same UAPI header at include/drm-uapi/drm.h, which is already
 * on the include path. Forward to it, so the cross build takes its DRM ABI from
 * the Mesa tree like every other Mesa header. Both copies are identical for the
 * !__linux__ branch we compile (stdint.h + sys/types.h + sys/ioccom.h, the last
 * supplied by this same shim directory).
 *
 * Copyright 2026 Phoenix Systems
 * SPDX-License-Identifier: MIT
 */
#ifndef PHOENIX_V3D_SHIM_DRM_H
#define PHOENIX_V3D_SHIM_DRM_H

#include "drm-uapi/drm.h"

#endif /* PHOENIX_V3D_SHIM_DRM_H */
