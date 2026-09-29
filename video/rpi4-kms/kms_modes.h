/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - scaled display modes (M9)
 *
 * The connector lists lower modes beside the native one; the HDMI link never
 * leaves the native timing. A client that sets a lower mode draws in that mode's
 * coordinate space and the HVS scales every plane to the screen, aspect preserved
 * and centred (black bars where the aspect differs): what a Linux panel fitter
 * (i915/amdgpu "scaling mode" Full aspect) does, and what vc4-fkms does for
 * overscan margins (vc4_firmware_kms.c vc4_fkms_margins_adj: dst rescaled, src
 * untouched). The firmware SET_PLANE value carries src and dst sizes separately
 * (kms_backend.c), so the scaler is free.
 *
 * Pure functions (no Phoenix headers, no server state), like kms_scanout.h, so the
 * host test tools/gpu-lane/kms/hosttest/ checks exactly the arithmetic the server
 * applies.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _KMS_MODES_H_
#define _KMS_MODES_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "kms_proto.h"


/* The lower modes offered, largest first (DRM lists modes that way; SDL re-sorts).
 * 16:9 ones fill a 16:9 screen; 4:3 ones are pillarboxed. A size is offered only
 * when it is smaller than the native mode in both directions. */
static const struct {
	uint16_t w, h;
} kms_scaled_sizes[] = {
	{ 1600u, 900u }, { 1440u, 1080u }, { 1280u, 720u }, { 1024u, 768u }, { 960u, 540u }, { 800u, 600u },
	{ 640u, 480u },
};

#define KMS_NSCALED   (sizeof(kms_scaled_sizes) / sizeof(kms_scaled_sizes[0]))
#define KMS_MAX_MODES (1u + KMS_NSCALED)


static inline int kms_mode_offered(const kms_modeinfo_t *native, uint32_t w, uint32_t h)
{
	uint32_t i;

	for (i = 0u; i < KMS_NSCALED; i++) {
		if ((kms_scaled_sizes[i].w == w) && (kms_scaled_sizes[i].h == h)) {
			return ((w <= native->hdisplay) && (h <= native->vdisplay) &&
				((w != native->hdisplay) || (h != native->vdisplay))) ? 1 : 0;
		}
	}
	return 0;
}


/* A lower mode's modeinfo. Its timings are nominal (the link keeps the native
 * ones): mode_fallback's ~10 %/4 % blanking and a pixel clock that gives the
 * native refresh, so a client that derives the rate from clock/htotal/vtotal
 * (Xorg) and one that matches vrefresh exactly (quakespasm's fullscreen mode
 * lookup) both see the native rate. */
static inline void kms_mode_scaled(const kms_modeinfo_t *native, uint32_t refresh_mhz, uint32_t w, uint32_t h,
	kms_modeinfo_t *m)
{
	memset(m, 0, sizeof(*m));
	m->hdisplay = (uint16_t)w;
	m->vdisplay = (uint16_t)h;
	m->hsync_start = (uint16_t)(w + w / 40u);
	m->hsync_end = (uint16_t)(m->hsync_start + w / 40u);
	m->htotal = (uint16_t)(w + w / 10u);
	m->vsync_start = (uint16_t)(h + 3u);
	m->vsync_end = (uint16_t)(m->vsync_start + 5u);
	m->vtotal = (uint16_t)(h + h / 25u + 8u);
	m->vrefresh = native->vrefresh;
	if (refresh_mhz == 0u) {
		refresh_mhz = native->vrefresh * 1000u;
	}
	m->clock = (uint32_t)(((uint64_t)m->htotal * m->vtotal * refresh_mhz) / 1000000ULL);   /* kHz */
	m->flags = KMS_MODE_FLAG_NHSYNC | KMS_MODE_FLAG_NVSYNC;
	m->type = KMS_MODE_TYPE_DRIVER;
	snprintf(m->name, sizeof(m->name), "%ux%u", w, h);
}


/* The connector's mode list: the native mode first, then (scaled != 0) every
 * offered lower mode. Writes min(total, max) entries; returns the total. */
static inline uint32_t kms_mode_list(const kms_modeinfo_t *native, uint32_t refresh_mhz, int scaled,
	kms_modeinfo_t *out, uint32_t max)
{
	uint32_t i, n = 0u;

	if ((out != NULL) && (n < max)) {
		out[n] = *native;
	}
	n++;
	for (i = 0u; (scaled != 0) && (i < KMS_NSCALED); i++) {
		if (!kms_mode_offered(native, kms_scaled_sizes[i].w, kms_scaled_sizes[i].h)) {
			continue;
		}
		if ((out != NULL) && (n < max)) {
			kms_mode_scaled(native, refresh_mhz, kms_scaled_sizes[i].w, kms_scaled_sizes[i].h, &out[n]);
		}
		n++;
	}
	return n;
}


/* Mode space -> screen: uniform scale num/den, centred. num == 0 is the identity
 * (a zero-initialised CRTC, and every CRTC in the native mode). */
typedef struct {
	uint32_t num, den;
	int32_t ox, oy;              /* top-left of the image on the screen */
	uint32_t fw, fh;             /* image size on the screen */
	uint32_t sw, sh;             /* screen (native mode) size */
} kms_fit_t;


static inline int64_t kms_fit_div_round(int64_t a, int64_t b)
{
	/* round half up, also for negative a (a plane partly off the left edge) */
	int64_t q = (2 * a + b) / (2 * b);

	if (((2 * a + b) % (2 * b) != 0) && ((2 * a + b) < 0)) {
		q--;
	}
	return q;
}


static inline kms_fit_t kms_fit(uint32_t sw, uint32_t sh, uint32_t mw, uint32_t mh)
{
	kms_fit_t f;

	memset(&f, 0, sizeof(f));
	f.sw = sw;
	f.sh = sh;
	if ((mw == 0u) || (mh == 0u) || (sw == 0u) || (sh == 0u) || ((mw == sw) && (mh == sh))) {
		f.fw = sw;
		f.fh = sh;
		return f;   /* identity */
	}
	/* the smaller of sw/mw and sh/mh: the image fits both ways */
	if ((uint64_t)sw * mh <= (uint64_t)sh * mw) {
		f.num = sw;
		f.den = mw;
	}
	else {
		f.num = sh;
		f.den = mh;
	}
	f.fw = (uint32_t)kms_fit_div_round((int64_t)mw * f.num, f.den);
	f.fh = (uint32_t)kms_fit_div_round((int64_t)mh * f.num, f.den);
	if (f.fw > sw) {
		f.fw = sw;
	}
	if (f.fh > sh) {
		f.fh = sh;
	}
	f.ox = (int32_t)((sw - f.fw) / 2u);
	f.oy = (int32_t)((sh - f.fh) / 2u);
	return f;
}


static inline int kms_fit_is_identity(const kms_fit_t *f)
{
	return (f->num == 0u) ? 1 : 0;
}


/* Parts of the screen the image does not cover (the letterbox/pillarbox bars). */
static inline int kms_fit_bars(const kms_fit_t *f)
{
	return (!kms_fit_is_identity(f) && ((f->fw < f->sw) || (f->fh < f->sh))) ? 1 : 0;
}


/* A plane's destination rectangle, mode space -> screen. Both edges are mapped and
 * the size is their difference, so adjacent planes stay adjacent. */
static inline void kms_fit_rect(const kms_fit_t *f, int32_t x, int32_t y, uint32_t w, uint32_t h, int32_t *dx,
	int32_t *dy, uint32_t *dw, uint32_t *dh)
{
	int64_t x0, y0, x1, y1;

	if (kms_fit_is_identity(f)) {
		*dx = x;
		*dy = y;
		*dw = w;
		*dh = h;
		return;
	}
	x0 = kms_fit_div_round((int64_t)x * f->num, f->den);
	y0 = kms_fit_div_round((int64_t)y * f->num, f->den);
	x1 = kms_fit_div_round(((int64_t)x + w) * f->num, f->den);
	y1 = kms_fit_div_round(((int64_t)y + h) * f->num, f->den);
	*dx = (int32_t)(f->ox + x0);
	*dy = (int32_t)(f->oy + y0);
	*dw = (uint32_t)(x1 - x0);
	*dh = (uint32_t)(y1 - y0);
}

#endif /* _KMS_MODES_H_ */
