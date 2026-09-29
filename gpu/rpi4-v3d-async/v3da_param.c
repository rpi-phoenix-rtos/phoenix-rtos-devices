/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - GET_PARAM
 *
 * Kept in its own translation unit: the vendored DRM uapi (gpu/rpi4-v3d/uapi,
 * MIT) brings a <sys/ioccom.h> shim whose _IO* macros clash with libphoenix's
 * <sys/ioctl.h>, which the dispatcher needs for ioctl_unpack().
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>

#include "v3d_drm.h"
#include "v3da.h"


/* Identity from the LIVE registers (the old client hard-codes them,
 * gpu/rpi4-v3d/libv3d-client.c:233-239); capabilities as the old lane reports
 * them - what M1 will serve (v3dv refuses a device without TFU, CSD,
 * CACHE_FLUSH, MULTISYNC_EXT and CPU_QUEUE, v3dv_device.c:861-868). */
int v3da_get_param(uint32_t param, uint64_t *value)
{
	switch (param) {
		case DRM_V3D_PARAM_V3D_CORE0_IDENT0: *value = srv.hw.ident[0]; return 0;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT1: *value = srv.hw.ident[1]; return 0;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT2: *value = srv.hw.ident[2]; return 0;
		case DRM_V3D_PARAM_V3D_UIFCFG:       *value = srv.hw.ident[3]; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT1:   *value = srv.hw.ident[4]; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT2:   *value = srv.hw.ident[5]; return 0;
		case DRM_V3D_PARAM_V3D_HUB_IDENT3:   *value = srv.hw.ident[6]; return 0;
		case DRM_V3D_PARAM_SUPPORTS_TFU:           *value = 1u; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CSD:           *value = 1u; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CACHE_FLUSH:   *value = 1u; return 0;
		case DRM_V3D_PARAM_SUPPORTS_MULTISYNC_EXT: *value = 1u; return 0;
		case DRM_V3D_PARAM_SUPPORTS_CPU_QUEUE:     *value = 1u; return 0;
		case DRM_V3D_PARAM_SUPPORTS_PERFMON:       *value = 0u; return 0;
		default:                                   *value = 0u; return 0;   /* old lane: unknown = 0 */
	}
}
