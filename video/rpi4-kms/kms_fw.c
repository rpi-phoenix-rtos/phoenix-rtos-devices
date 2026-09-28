/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - firmware interface
 *
 * Everything the server asks the VideoCore firmware goes through /dev/vcmbox
 * (libvcmbox): display count/ids, timings, EDID, the framebuffer geometry,
 * panning. The large-buffer call (vcmbox_callXL) carries EDID (136 B) and
 * SET_PLANE (60 B, kms_backend.c). Tag numbers and value layouts were read from
 * the Raspberry Pi Linux fork (raspberrypi-firmware.h, vc4_firmware_kms.c);
 * facts only, no code (docs/gpu-new-lane/E3-firmware-planes-vblank.md).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/platform.h>

#include <phoenix/arch/aarch64/generic/generic.h>
#include <phoenix/fbcon.h>

#include "kms.h"
#include "libvcmbox.h"


#define TAG_GET_FIRMWARE_REV   0x00000001u
#define TAG_GET_EDID_BLOCK_DISP 0x00030023u   /* value: block, display, 128 B */
#define TAG_FB_BLANK           0x00040002u
#define TAG_FB_GET_VIRT_WH     0x00040004u
#define TAG_FB_GET_PIXEL_ORDER 0x00040006u
#define TAG_FB_GET_VIRT_OFFSET 0x00040009u
#define TAG_FB_GET_LAYER       0x0004000cu
#define TAG_FB_GET_NUM_DISP    0x00040013u
#define TAG_FB_GET_DISPLAY_ID  0x00040016u
#define TAG_FB_SET_VIRT_OFFSET 0x00048009u
#define TAG_GET_DISPLAY_TIMING 0x00040017u   /* value: struct set_timings, 36 B */

/* set_timings.flags (vc4_firmware_kms.c TIMINGS_FLAGS_*) */
#define FW_TIMING_HSYNC_POS  (1u << 0)
#define FW_TIMING_VSYNC_POS  (1u << 1)
#define FW_TIMING_INTERLACE  (1u << 2)


int kms_fw_prop(uint32_t tag, uint32_t valWords, const uint32_t *in, uint32_t nIn, uint32_t *out)
{
	return vcmbox_call(tag, valWords * 4u, in, nIn, out, (out != NULL) ? valWords : 0u);
}


static int prop1(uint32_t tag, uint32_t in, uint32_t *out)
{
	uint32_t o[1] = { 0u };
	int rc = kms_fw_prop(tag, 1u, &in, 1u, o);

	if ((rc == 0) && (out != NULL)) {
		*out = o[0];
	}
	return rc;
}


/* SET_VIRTUAL_OFFSET (x = 0, y = yoff). The firmware answers with the offset it
 * actually applied; a clamped answer means the slot is outside the granted
 * virtual height. */
int kms_fw_pan(uint32_t yoff, uint32_t *got)
{
	uint32_t in[2] = { 0u, yoff }, o[2] = { 0u, 0u };
	int rc = kms_fw_prop(TAG_FB_SET_VIRT_OFFSET, 2u, in, 2u, o);

	if (got != NULL) {
		*got = o[1];
	}
	return rc;
}


/* FRAMEBUFFER_BLANK of the firmware fb (-B): E3 `stack` steps 3/4 showed it
 * leaves the list and comes back. With an opaque full-screen primary above it
 * the fb element is still fetched by the HVS every frame (1080p60 x 4 B ~ 0.5 GB/s
 * of DRAM reads [inferred]); blanking it removes that, as Linux fkms does. */
int kms_fw_blank(int on)
{
	return prop1(TAG_FB_BLANK, (on != 0) ? 1u : 0u, NULL);
}


/* Console handover (-C): the pl011-tty fbcon stops drawing into the firmware fb
 * (it keeps an off-screen shadow and blits it back on re-enable). Side effect
 * measured in source: pl011-tty also releases /dev/kbd0 while disabled. */
int kms_fw_console(int enable)
{
	int mode = (enable != 0) ? FBCON_ENABLED : FBCON_DISABLED;
	int rc;

	if (srv.tty_fd < 0) {
		srv.tty_fd = open("/dev/tty0", O_RDWR);
		if (srv.tty_fd < 0) {
			srv.tty_fd = open("/dev/console", O_RDWR);
		}
	}
	if (srv.tty_fd < 0) {
		return -ENODEV;
	}
	rc = ioctl(srv.tty_fd, FBCONSETMODE, mode);
	return (rc < 0) ? -errno : 0;
}


/* The 32-bit address handed to the firmware for a buffer. Never truncates: a
 * PA that does not fit is refused (the project has lost weeks to silent
 * 64->32-bit and missing-alias bugs). E3/E6 (2026-09-26, build 9) measured both
 * conventions on the bench:
 *   raw PA (Linux fkms parity, default): the firmware ORs in bus alias 0x8
 *     (dlist word 0xac000000 for PA 0x2c000000) and the image is correct;
 *   -c, 0xC0000000|PA: kept as is (alias 0xC, the alias of the firmware's own
 *     fb, 0xfd3b2000) and correct too;
 *   raw PA 0xf8000000 (above 1 GiB): the firmware keeps the low 30 bits (dlist
 *     0xf8000000 = 0x38000000, the VideoCore memory base) and the screen shows
 *     other memory. So BOTH conventions reach the low 1 GiB only. */
int kms_bus_addr(uint64_t pa, size_t len, uint32_t *bus)
{
	uint64_t end = pa + len;

	if (end > KMS_GIB) {
		return -ERANGE;
	}
	*bus = (srv.bus_c0 != 0) ? (0xc0000000u | (uint32_t)pa) : (uint32_t)pa;
	return 0;
}


/* GET_DISPLAY_TIMING answered all zeros on the bench firmware (E3 `info timing`,
 * 2026-09-26), so the mode is SYNTHESIZED: the fb geometry, CEA-861 VIC 16
 * blanking (2200x1125, 148.5 MHz) for 1920x1080, else ~10 %/4 % blanking, and a
 * pixel clock consistent with the refresh (60 Hz until the vblank thread has
 * measured the real rate, then recomputed: kms_mode_set_refresh). Clients that
 * derive refresh from clock/htotal/vtotal (Xorg) then agree with vrefresh.
 * The blanking values are not what the HDMI link uses [inferred: only the
 * firmware knows them]; Stage A never programs them. */
void kms_mode_set_refresh(kms_crtc_state_t *c, uint32_t mhz)
{
	kms_modeinfo_t *m = &c->mode;

	if (c->mode_from_fw || (mhz == 0u)) {
		return;
	}
	c->refresh_mhz = mhz;
	m->vrefresh = (mhz + 500u) / 1000u;
	m->clock = (uint32_t)(((uint64_t)m->htotal * m->vtotal * mhz) / 1000000ULL);   /* kHz */
	/* Keep MODE_ID's blob identical to the connector's mode: Weston/Xorg compare
	 * modes structurally, and a stale blob would read as "mode not in the list". */
	{
		kms_srvblob_t *b = kms_blob_get(c->mode_blob);
		if ((b != NULL) && (b->len == sizeof(*m))) {
			memcpy(b->data, m, sizeof(*m));
		}
	}
}


static void mode_fallback(kms_crtc_state_t *c)
{
	kms_modeinfo_t *m = &c->mode;

	memset(m, 0, sizeof(*m));
	m->hdisplay = (uint16_t)srv.fb_w;
	m->vdisplay = (uint16_t)srv.fb_h;
	if ((srv.fb_w == 1920u) && (srv.fb_h == 1080u)) {
		m->hsync_start = 2008u;
		m->hsync_end = 2052u;
		m->htotal = 2200u;
		m->vsync_start = 1084u;
		m->vsync_end = 1089u;
		m->vtotal = 1125u;
		m->flags = KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC;
	}
	else {
		m->hsync_start = (uint16_t)(m->hdisplay + m->hdisplay / 40u);
		m->hsync_end = (uint16_t)(m->hsync_start + m->hdisplay / 40u);
		m->htotal = (uint16_t)(m->hdisplay + m->hdisplay / 10u);
		m->vsync_start = (uint16_t)(m->vdisplay + 3u);
		m->vsync_end = (uint16_t)(m->vsync_start + 5u);
		m->vtotal = (uint16_t)(m->vdisplay + m->vdisplay / 25u + 8u);
		m->flags = KMS_MODE_FLAG_NHSYNC | KMS_MODE_FLAG_NVSYNC;
	}
	m->type = KMS_MODE_TYPE_PREFERRED | KMS_MODE_TYPE_DRIVER;
	snprintf(m->name, sizeof(m->name), "%ux%u", srv.fb_w, srv.fb_h);
	c->mode_from_fw = 0;
	kms_mode_set_refresh(c, 60000u);
}


/* GET_DISPLAY_TIMING: word 0 = display | vic << 16, 1 = clock kHz, 2 = hdisplay |
 * hsync_start << 16, 3 = hsync_end | htotal << 16, 4 = hskew | vdisplay << 16,
 * 5 = vsync_start | vsync_end << 16, 6 = vtotal | vscan << 16, 7 = vrefresh,
 * 8 = flags (struct set_timings, vc4_firmware_kms.c:129-174). */
static void mode_query(kms_crtc_state_t *c)
{
	uint32_t o[9], in[1];
	kms_modeinfo_t *m = &c->mode;

	memset(o, 0, sizeof(o));
	in[0] = c->fw_display_id & 0xffu;
	if ((kms_fw_prop(TAG_GET_DISPLAY_TIMING, 9u, in, 1u, o) != 0) || (o[1] == 0u) || ((o[2] & 0xffffu) == 0u)) {
		mode_fallback(c);
		return;
	}
	memset(m, 0, sizeof(*m));
	m->clock = o[1];
	m->hdisplay = (uint16_t)(o[2] & 0xffffu);
	m->hsync_start = (uint16_t)(o[2] >> 16);
	m->hsync_end = (uint16_t)(o[3] & 0xffffu);
	m->htotal = (uint16_t)(o[3] >> 16);
	m->hskew = (uint16_t)(o[4] & 0xffffu);
	m->vdisplay = (uint16_t)(o[4] >> 16);
	m->vsync_start = (uint16_t)(o[5] & 0xffffu);
	m->vsync_end = (uint16_t)(o[5] >> 16);
	m->vtotal = (uint16_t)(o[6] & 0xffffu);
	m->vscan = (uint16_t)(o[6] >> 16);
	m->vrefresh = o[7] & 0xffffu;
	m->flags = ((o[8] & FW_TIMING_HSYNC_POS) ? KMS_MODE_FLAG_PHSYNC : KMS_MODE_FLAG_NHSYNC) |
		((o[8] & FW_TIMING_VSYNC_POS) ? KMS_MODE_FLAG_PVSYNC : KMS_MODE_FLAG_NVSYNC) |
		((o[8] & FW_TIMING_INTERLACE) ? KMS_MODE_FLAG_INTERLACE : 0u);
	m->type = KMS_MODE_TYPE_PREFERRED | KMS_MODE_TYPE_DRIVER;
	snprintf(m->name, sizeof(m->name), "%ux%u", m->hdisplay, m->vdisplay);
	c->mode_from_fw = 1;
	if ((m->htotal != 0u) && (m->vtotal != 0u)) {
		c->refresh_mhz = (uint32_t)(((uint64_t)m->clock * 1000000ULL) / ((uint64_t)m->htotal * m->vtotal));
	}
	else {
		c->refresh_mhz = m->vrefresh * 1000u;
	}
}


/* EDID block 0 through the large-buffer call: value = block, display, 128 B. */
static void edid_query(kms_crtc_state_t *c)
{
	static const uint8_t hdr[8] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
	uint32_t buf[2 + 32], ans[2 + 32];
	const uint8_t *e = (const uint8_t *)&ans[2];

	if (srv.xl == 0) {
		return;
	}
	memset(buf, 0, sizeof(buf));
	buf[0] = 0u;
	buf[1] = (uint32_t)c->idx;
	memset(ans, 0, sizeof(ans));
	if (vcmbox_callXL(TAG_GET_EDID_BLOCK_DISP, buf, sizeof(buf), ans) != 0) {
		return;
	}
	if (memcmp(e, hdr, sizeof(hdr)) != 0) {
		return;
	}
	c->edid_blob = kms_blob_create(0u, e, 128u);
	c->mm_w = (uint32_t)e[21] * 10u;   /* EDID 1.x: max image size in cm */
	c->mm_h = (uint32_t)e[22] * 10u;
}


/* Firmware framebuffer: base + geometry from plo (platformctl graphmode), the
 * granted virtual height, pan offset, pixel order and layer from the firmware. */
static void fb_query(void)
{
	platformctl_t pctl;
	uint32_t o[2];

	memset(&pctl, 0, sizeof(pctl));
	pctl.action = pctl_get;
	pctl.type = pctl_graphmode;
	if ((platformctl(&pctl) == 0) && (pctl.task.graphmode.framebuffer != 0u) && (pctl.task.graphmode.bpp == 32u)) {
		srv.fb_pa = pctl.task.graphmode.framebuffer;
		srv.fb_w = pctl.task.graphmode.width;
		srv.fb_h = pctl.task.graphmode.height;
		srv.fb_pitch = pctl.task.graphmode.pitch;
	}
	memset(o, 0, sizeof(o));
	if (kms_fw_prop(TAG_FB_GET_VIRT_WH, 2u, o, 2u, o) == 0) {
		srv.fb_virt_h = o[1];
	}
	memset(o, 0, sizeof(o));
	if (kms_fw_prop(TAG_FB_GET_VIRT_OFFSET, 2u, o, 2u, o) == 0) {
		srv.fb_yoff0 = o[1];
	}
	o[0] = 0u;
	if (prop1(TAG_FB_GET_PIXEL_ORDER, 0u, &o[0]) == 0) {
		srv.fb_pixel_order = o[0];
	}
	o[0] = 0u;
	if (prop1(TAG_FB_GET_LAYER, 0u, &o[0]) == 0) {
		srv.fb_layer = (int32_t)o[0];
		srv.have_fb_layer = 1;
	}
	/* plo asks for pixel order 1 ("RGB"); fbcon found that this puts R in the LOW
	 * byte of each u32 (pl011-tty.c palette comment), i.e. DRM XBGR8888 [inferred
	 * mapping; E3's snapshots check it]. */
	srv.fb_format = (srv.fb_pixel_order == 1u) ? KMS_FMT_XBGR8888 : KMS_FMT_XRGB8888;
	srv.fb_slots = ((srv.fb_h != 0u) && (srv.fb_virt_h >= srv.fb_h)) ? srv.fb_virt_h / srv.fb_h : 1u;
	if (srv.fb_slots > KMS_FB_SLOTS) {
		srv.fb_slots = KMS_FB_SLOTS;
	}
}


int kms_fw_init(void)
{
	uint32_t n = 1u, id, i, rev = 0u, v[1] = { 0u }, o[1] = { 0u };
	int rc;

	/* /dev/vcmbox must be there: the display server never drives the FIFO itself. */
	rc = prop1(TAG_GET_FIRMWARE_REV, 0u, &rev);
	if (rc != 0) {
		KMS_LOG("srv FAIL vcmbox rc=%d - refusing to drive the mailbox FIFO directly", rc);
		return rc;
	}
	/* Large-buffer extension (rpi4-vcmbox 79a4212): a server without it answers -EINVAL
	 * before touching the FIFO. */
	srv.xl = ((vcmbox_callXL(TAG_GET_FIRMWARE_REV, v, 4u, o) == 0) && (o[0] != 0u)) ? 1 : 0;

	fb_query();

	if (prop1(TAG_FB_GET_NUM_DISP, 0u, &n) != 0) {
		n = 1u;
	}
	if (n == 0u) {
		n = 1u;
	}
	srv.ncrtc = (n > KMS_MAX_CRTCS) ? KMS_MAX_CRTCS : n;
	/* Stage A drives display 0 only (HDMI0 on this bench); a second display is
	 * reported (connector + CRTC) but its planes are not exposed. */
	srv.ncrtc = 1u;
	for (i = 0u; i < srv.ncrtc; i++) {
		kms_crtc_state_t *c = &srv.crtc[i];

		memset(c, 0, sizeof(*c));
		c->idx = (int)i;
		id = i;
		(void)prop1(TAG_FB_GET_DISPLAY_ID, i, &id);
		c->fw_display_id = id;
		mode_query(c);
		c->connection = KMS_CONNECTED;   /* the firmware set a mode on it at boot */
		c->mode_blob = kms_blob_create(0u, &c->mode, sizeof(c->mode));
		edid_query(c);
	}
	KMS_LOG("fw rev=0x%08x xl=%d displays=%u fb_pa=0x%llx fb=%ux%u pitch=%u virt_h=%u slots=%u yoff=%u "
		"pixel_order=%u fmt=%.4s fb_layer=%d(%s)",
		rev, srv.xl, n, (unsigned long long)srv.fb_pa, srv.fb_w, srv.fb_h, srv.fb_pitch, srv.fb_virt_h, srv.fb_slots,
		srv.fb_yoff0, srv.fb_pixel_order, (const char *)&srv.fb_format, (int)srv.fb_layer,
		srv.have_fb_layer ? "fw" : "unknown");
	for (i = 0u; i < srv.ncrtc; i++) {
		const kms_crtc_state_t *c = &srv.crtc[i];
		const kms_modeinfo_t *m = &c->mode;

		KMS_LOG("mode crtc=%u fw_id=%u %s clock_khz=%u h=%u/%u/%u/%u v=%u/%u/%u/%u vrefresh=%u refresh_mhz=%u "
			"from_fw=%d edid=%s mm=%ux%u",
			i, c->fw_display_id, m->name, m->clock, m->hdisplay, m->hsync_start, m->hsync_end, m->htotal, m->vdisplay,
			m->vsync_start, m->vsync_end, m->vtotal, m->vrefresh, c->refresh_mhz, c->mode_from_fw,
			(c->edid_blob != 0u) ? "yes" : "no", c->mm_w, c->mm_h);
	}
	return 0;
}
