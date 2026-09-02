/* xf86drmMode.h shim — V3DV references the KMS display-probe surface in
 * try_display_device (the VK_KHR_display path), which Phoenix bypasses (no KMS / no
 * /dev/dri display node). Declare the minimal struct/enum/function surface so the TUs
 * compile + link; the stubs in v3dv_libdrm_shim.c report "no resources" so the probe
 * fails cleanly and the bypassed path is never taken on HW.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#ifndef PHX_XF86DRMMODE_H
#define PHX_XF86DRMMODE_H
#include <stdint.h>

typedef enum {
	DRM_MODE_CONNECTED         = 1,
	DRM_MODE_DISCONNECTED      = 2,
	DRM_MODE_UNKNOWNCONNECTION = 3,
} drmModeConnection;

typedef struct _drmModeRes {
	int count_fbs;          uint32_t *fbs;
	int count_crtcs;        uint32_t *crtcs;
	int count_connectors;   uint32_t *connectors;
	int count_encoders;     uint32_t *encoders;
	uint32_t min_width, max_width;
	uint32_t min_height, max_height;
} drmModeRes, *drmModeResPtr;

typedef struct _drmModeConnector {
	uint32_t connector_id;
	uint32_t encoder_id;
	uint32_t connector_type;
	uint32_t connector_type_id;
	drmModeConnection connection;
	uint32_t mmWidth, mmHeight;
	int count_modes;
	void *modes;
	int count_props;        uint32_t *props;
	uint64_t *prop_values;
	int count_encoders;     uint32_t *encoders;
} drmModeConnector, *drmModeConnectorPtr;

drmModeResPtr       drmModeGetResources(int fd);
void                drmModeFreeResources(drmModeResPtr ptr);
drmModeConnectorPtr drmModeGetConnector(int fd, uint32_t connectorId);
void                drmModeFreeConnector(drmModeConnectorPtr ptr);
#endif
