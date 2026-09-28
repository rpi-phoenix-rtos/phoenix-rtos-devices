/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - plane backends
 *
 * A backend turns "plane p shows framebuffer f" into firmware calls. Two exist,
 * selected at start (-b); E3 (2026-09-26, build 9) proved the plane API, so
 * "plane" is the default and "pan" the fallback:
 *
 *   pan   - the mechanism the old lane has shipped for months: the firmware
 *           framebuffer is 3 screens tall (plo, max_framebuffer_height) and a
 *           flip is SET_VIRTUAL_OFFSET to slot k. Only a primary plane; only
 *           full-screen buffers that ARE firmware-fb slots 1..n-1 (slot 0 is
 *           where fbcon and rpi4-fb draw). Fallback (-b pan).
 *   plane - the firmware plane API (SET_PLANE, 60-byte value buffer, the "fake
 *           KMS" interface) for buffers from our own contiguous pool: primary,
 *           cursor, optional overlays, hardware scaling, per-plane alpha,
 *           rotate-180/reflect. Needs the /dev/vcmbox large-buffer call. Default.
 *           E3: 1201 flips in 20 s at 60/s, 0 missed, SET_PLANE p50 100 us,
 *           overlay p50 75 us, our pool buffers scanned below 1 GiB.
 *
 * SET_PLANE layout (vc4_firmware_kms.c:64-96, facts only; see E3 section 1.2):
 *   w0 display u8 | plane_id u8 | vc_image_type u8 | layer s8
 *   w1 width u16 | height u16        w2 pitch u16 | vpitch u16
 *   w3..w6 src_x/y/w/h 16.16         w7 dst_x s16 | dst_y s16
 *   w8 dst_w u16 | dst_h u16         w9 alpha u8 | num_planes u8 | is_vu u8 | color_encoding u8
 *   w10..w13 planes[4] bus addresses  w14 transform
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

#include "kms.h"
#include "libvcmbox.h"


/* ========================================================================= */
/* pan                                                                        */
/* ========================================================================= */

static int pan_init(kms_crtc_state_t *c)
{
	if ((srv.fb_pa == 0u) || (srv.fb_pitch == 0u)) {
		KMS_LOG("backend=pan FAIL no firmware framebuffer (graphmode)");
		return -ENODEV;
	}
	if (srv.fb_slots < 2u) {
		KMS_LOG("backend=pan FAIL virt_h=%u gives %u slot(s): nothing to flip to", srv.fb_virt_h, srv.fb_slots);
		return -ENODEV;
	}
	c->plane_mask = 1u;   /* primary only */
	return 0;
}


static uint32_t pan_formats(uint32_t plane, uint32_t *out, uint32_t max)
{
	if ((plane != 0u) || (max == 0u)) {
		return 0u;
	}
	out[0] = srv.fb_format;
	return 1u;
}


static int pan_check(const kms_crtc_state_t *c, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
	const kms_bo_t *bo)
{
	(void)c;
	if (p != 0u) {
		return -EINVAL;
	}
	if (fb == NULL) {
		return 0;   /* disable = show the console slot */
	}
	if ((bo->kind != KMS_BOK_SLOT) || (fb->offset != 0u) || (fb->w != srv.fb_w) || (fb->h != srv.fb_h) ||
			(fb->pitch != srv.fb_pitch) || (fb->format != srv.fb_format)) {
		return -EINVAL;   /* not a firmware-fb slot: this backend cannot scan it */
	}
	if ((st->crtc_x != 0) || (st->crtc_y != 0) || (st->crtc_w != fb->w) || (st->crtc_h != fb->h) ||
			(st->src_x != 0u) || (st->src_y != 0u) || (st->src_w != (fb->w << 16)) || (st->src_h != (fb->h << 16))) {
		return -ERANGE;   /* no scaling, no offsets: panning shows a whole slot */
	}
	if ((st->rotation != 0u) && (st->rotation != 1u)) {
		return -EINVAL;
	}
	return 0;
}


static int pan_apply(kms_crtc_state_t *c, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
	const kms_bo_t *bo, uint32_t *lat_us)
{
	uint32_t want = (fb == NULL) ? srv.fb_yoff0 : bo->slot * srv.fb_h, got = 0u;
	uint64_t t0 = kms_cnt();
	int rc;

	(void)c;
	(void)p;
	(void)st;
	rc = kms_fw_pan(want, &got);
	if (lat_us != NULL) {
		*lat_us = (uint32_t)kms_cnt_us(kms_cnt() - t0);
	}
	if ((rc == 0) && (got != want)) {
		KMS_LOG("pan clamped want=%u got=%u", want, got);
		rc = -ERANGE;
	}
	return rc;
}


static void pan_restore(kms_crtc_state_t *c)
{
	(void)pan_apply(c, 0u, NULL, NULL, NULL, NULL);
}


const kms_backend_t kms_backend_pan = {
	.name = "pan",
	.id = KMS_BACKEND_PAN,
	.init = pan_init,
	.formats = pan_formats,
	.check = pan_check,
	.apply = pan_apply,
	.restore = pan_restore,
};


/* ========================================================================= */
/* plane (firmware SET_PLANE)                                                 */
/* ========================================================================= */

#define TAG_SET_PLANE 0x00048015u

/* VC_IMAGE_TYPE_T (raspberrypi/userland vc_image_types.h, BSD-3) */
#define VC_IMAGE_MIN      0u
#define VC_IMAGE_ARGB8888 43u
#define VC_IMAGE_XRGB8888 44u

#define FW_XFORM_ROT180 (1u << 1)
#define FW_XFORM_HFLIP  (1u << 16)
#define FW_XFORM_VFLIP  (1u << 17)

#define DRM_ROTATE_0    (1u << 0)
#define DRM_ROTATE_180  (1u << 2)
#define DRM_REFLECT_X   (1u << 4)
#define DRM_REFLECT_Y   (1u << 5)

typedef struct {
	uint8_t display;
	uint8_t plane_id;
	uint8_t vc_image_type;
	int8_t layer;
	uint16_t width, height;
	uint16_t pitch, vpitch;
	uint32_t src_x, src_y, src_w, src_h;
	int16_t dst_x, dst_y;
	uint16_t dst_w, dst_h;
	uint8_t alpha, num_planes, is_vu, color_encoding;
	uint32_t planes[4];
	uint32_t transform;
} fw_plane_t;

_Static_assert(sizeof(fw_plane_t) == 60, "SET_PLANE value buffer must be 60 bytes");

static uint32_t plane_touched;   /* bit p: plane p of display 0 was set by us */
uint32_t kms_plane_overlays;     /* -o: overlays exposed (0..6) */


static int plane_init(kms_crtc_state_t *c)
{
	uint32_t i;

	if (srv.xl == 0) {
		KMS_LOG("backend=plane FAIL /dev/vcmbox has no large-buffer call (SET_PLANE is 60 B)");
		return -ENOTSUP;
	}
	if (srv.pool_ok == 0) {
		KMS_LOG("backend=plane FAIL no scan-out pool");
		return -ENOMEM;
	}
	c->plane_mask = 1u | (1u << 7);   /* primary + cursor */
	for (i = 1u; (i <= kms_plane_overlays) && (i <= 6u); i++) {
		c->plane_mask |= 1u << i;
	}
	return 0;
}


static uint32_t plane_formats(uint32_t plane, uint32_t *out, uint32_t max)
{
	uint32_t n = 0u;

	(void)plane;
	if (n < max) {
		out[n++] = KMS_FMT_XRGB8888;
	}
	if (n < max) {
		out[n++] = KMS_FMT_ARGB8888;
	}
	return n;
}


static uint8_t vc_type(uint32_t fmt)
{
	return (fmt == KMS_FMT_ARGB8888) ? VC_IMAGE_ARGB8888 : VC_IMAGE_XRGB8888;
}


static int plane_check(const kms_crtc_state_t *c, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
	const kms_bo_t *bo)
{
	uint32_t bus, sw, sh;

	if ((p >= KMS_PLANES_PER_CRTC) || ((c->plane_mask & (1u << p)) == 0u)) {
		return -EINVAL;
	}
	if (fb == NULL) {
		return 0;
	}
	if ((fb->format != KMS_FMT_XRGB8888) && (fb->format != KMS_FMT_ARGB8888)) {
		return -EINVAL;
	}
	if ((fb->w > 0xffffu) || (fb->h > 0xffffu) || (fb->pitch > 0xffffu)) {
		return -ERANGE;
	}
	if (kms_bus_addr(bo->pa + fb->offset, (size_t)fb->pitch * fb->h, &bus) != 0) {
		return -ERANGE;   /* outside what the chosen bus convention reaches */
	}
	sw = st->src_w >> 16;
	sh = st->src_h >> 16;
	if ((sw == 0u) || (sh == 0u) || ((st->src_x >> 16) + sw > fb->w) || ((st->src_y >> 16) + sh > fb->h)) {
		return -ERANGE;
	}
	if ((st->crtc_w == 0u) || (st->crtc_h == 0u) || (st->crtc_w > 0xffffu) || (st->crtc_h > 0xffffu) ||
			(st->crtc_x < -32768) || (st->crtc_x > 32767) || (st->crtc_y < -32768) || (st->crtc_y > 32767)) {
		return -ERANGE;
	}
	if ((st->rotation & ~(DRM_ROTATE_0 | DRM_ROTATE_180 | DRM_REFLECT_X | DRM_REFLECT_Y)) != 0u) {
		return -EINVAL;
	}
	return 0;
}


/* Layer for the primary: above the firmware fb so it covers the console (the
 * fb answers GET_LAYER -127 on the bench; E3 showed layer 0 on top of it).
 * Overlays and the cursor stack above the primary by zpos. */
static int8_t plane_layer(uint32_t p, int32_t zpos)
{
	int base = 0;

	/* E3: fb at layer -127 (GET_LAYER), primary at layer 0 covered it. */
	if (srv.have_fb_layer && (srv.fb_layer >= 0) && (srv.fb_layer < 100)) {
		base = srv.fb_layer + 1;
	}
	if (p == 7u) {
		return (int8_t)(base + 8);
	}
	if (p == 0u) {
		return (int8_t)base;
	}
	if (zpos < 1) {
		zpos = (int32_t)p;
	}
	if (zpos > 7) {
		zpos = 7;
	}
	return (int8_t)(base + zpos);
}


static int plane_apply(kms_crtc_state_t *c, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
	const kms_bo_t *bo, uint32_t *lat_us)
{
	fw_plane_t fp;
	uint32_t bus = 0u, o[15];
	uint64_t t0;
	int rc;

	memset(&fp, 0, sizeof(fp));
	fp.display = (uint8_t)c->fw_display_id;
	fp.plane_id = (uint8_t)(p + (uint32_t)c->idx * KMS_PLANES_PER_CRTC);
	if (fb != NULL) {
		rc = kms_bus_addr(bo->pa + fb->offset, (size_t)fb->pitch * fb->h, &bus);
		if (rc != 0) {
			return rc;
		}
		fp.vc_image_type = vc_type(fb->format);
		fp.layer = plane_layer(p, st->zpos);
		fp.width = (uint16_t)fb->w;
		fp.height = (uint16_t)fb->h;
		fp.pitch = (uint16_t)fb->pitch;
		fp.vpitch = (uint16_t)fb->h;
		fp.src_x = st->src_x;
		fp.src_y = st->src_y;
		fp.src_w = st->src_w;
		fp.src_h = st->src_h;
		fp.dst_x = (int16_t)st->crtc_x;
		fp.dst_y = (int16_t)st->crtc_y;
		fp.dst_w = (uint16_t)st->crtc_w;
		fp.dst_h = (uint16_t)st->crtc_h;
		fp.alpha = (uint8_t)((st->alpha > 0xffffu ? 0xffffu : st->alpha) >> 8);
		fp.num_planes = 1u;
		fp.planes[0] = bus;
		if ((st->rotation & DRM_ROTATE_180) != 0u) {
			fp.transform |= FW_XFORM_ROT180;
		}
		if ((st->rotation & DRM_REFLECT_X) != 0u) {
			fp.transform |= FW_XFORM_HFLIP;
		}
		if ((st->rotation & DRM_REFLECT_Y) != 0u) {
			fp.transform |= FW_XFORM_VFLIP;
		}
	}
	/* else: unset = display + plane_id only (vc4_plane_set_blank) */

	__asm__ volatile("dsb sy" ::: "memory");   /* CPU pixel stores reach DRAM before the plane latches */
	t0 = kms_cnt();
	rc = vcmbox_callXL(TAG_SET_PLANE, &fp, sizeof(fp), o);
	if (lat_us != NULL) {
		*lat_us = (uint32_t)kms_cnt_us(kms_cnt() - t0);
	}
	/* Record even on error: the firmware may have acted, and unsetting a plane
	 * that was never set is harmless. */
	if (fb != NULL) {
		plane_touched |= 1u << p;
	}
	else if (rc == 0) {
		plane_touched &= ~(1u << p);
	}
	return rc;
}


static void plane_restore(kms_crtc_state_t *c)
{
	uint32_t p;

	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		if ((plane_touched & (1u << p)) != 0u) {
			int rc = plane_apply(c, p, NULL, NULL, NULL, NULL);
			KMS_LOG("restore unset plane=%u rc=%d", p, rc);
		}
	}
}


const kms_backend_t kms_backend_plane = {
	.name = "plane",
	.id = KMS_BACKEND_PLANE,
	.init = plane_init,
	.formats = plane_formats,
	.check = plane_check,
	.apply = plane_apply,
	.restore = plane_restore,
};
