/*
 * Phoenix-RTOS
 *
 * USB HID mouse: report-descriptor parser and report-protocol translation
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#ifndef _USBMOUSE_HIDMOUSE_H_
#define _USBMOUSE_HIDMOUSE_H_

#include <stddef.h>
#include <stdint.h>


/* The /dev/mouseN packet: [0] buttons (bit n = button n+1), [1] dX int8,
 * [2] dY int8, [3] wheel int8 (positive = away from the user). */
#define HIDMOUSE_PKT_SIZE 4u

/* Buttons that fit the packet's button byte. */
#define HIDMOUSE_MAX_BUTTONS 8u

/* A report whose X/Y delta exceeds the int8 range of one packet is split into
 * several packets so the motion is not lost. 17 packets carry +/-2159 counts:
 * the full range of the common 12-bit fields; a larger 16-bit delta (far beyond
 * one poll interval of real motion) is clamped there, so one report can never
 * flood the 63-packet rx fifo. */
#define HIDMOUSE_MAX_SPLIT 17u


typedef struct {
	uint16_t off;     /* bit offset, counted from the first byte after the Report ID */
	uint8_t size;     /* bits; 0 = field absent */
	uint8_t id;       /* Report ID carrying the field (0 when the device uses none) */
	uint8_t isSigned; /* Logical Minimum < 0 */
} hidmouse_field_t;


typedef struct {
	uint8_t useIds;        /* every input report starts with a Report ID byte */
	uint8_t nbuttons;      /* distinct Button-page usages declared (informational) */
	uint16_t maxReportLen; /* longest input report in bytes, Report ID included */
	hidmouse_field_t button[HIDMOUSE_MAX_BUTTONS];
	hidmouse_field_t x;
	hidmouse_field_t y;
	hidmouse_field_t wheel;
	hidmouse_field_t hwheel; /* AC Pan; parsed for the log, no packet byte for it */
} hidmouse_layout_t;


/* Walks a configuration descriptor for interface `iface` (alternate setting 0):
 * the report-descriptor length from its HID class descriptor and the
 * wMaxPacketSize of its interrupt-IN endpoint. Returns 0 when both are found,
 * -ENOENT otherwise. */
int hidmouse_findInterface(const uint8_t *conf, size_t len, unsigned int iface, size_t *reportDescLen, size_t *maxPacket);


/* Parses a HID report descriptor. Returns 0 and fills `layout` when it
 * describes a relative X/Y pointer; otherwise returns a negative errno and, if
 * `reason` is non-NULL, points it at a short static description. */
int hidmouse_parse(const uint8_t *desc, size_t len, hidmouse_layout_t *layout, const char **reason);


/* Translates one input report into /dev/mouseN packets written to `out`
 * (`outsz` bytes). `buttons` holds the latched button state across reports (a
 * device may carry buttons and motion in different reports). Returns the number
 * of bytes written, a multiple of HIDMOUSE_PKT_SIZE; 0 when the report carries
 * no mouse field (another Report ID, or too short to hold any). */
size_t hidmouse_translate(const hidmouse_layout_t *layout, uint8_t *buttons, const uint8_t *report, size_t len, uint8_t *out, size_t outsz);


/* Formats the layout for the insertion log, e.g.
 * "id=none buttons=3 len=4 x=8@8 y=8@16 wheel=8@24 hwheel=none"; with Report
 * IDs each field also names its ID ("x=12@16#2"). */
void hidmouse_describe(const hidmouse_layout_t *layout, char *buf, size_t size);


#endif
