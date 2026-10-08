/*
 * Phoenix-RTOS
 *
 * USB HID mouse: report-descriptor parser and report-protocol translation
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

/*
 * A HID boot mouse reports 3 bytes (buttons, dX, dY) in boot protocol; the
 * wheel exists only in report protocol, where the layout is whatever the
 * device's report descriptor says (12/16-bit deltas, Report IDs, buttons and
 * motion in different reports). This file finds the fields a mouse needs in
 * that descriptor and turns each input report back into the driver's 4-byte
 * /dev/mouseN packet.
 *
 * It is deliberately small and self-contained (no Phoenix headers) so the same
 * source can be exercised on the build host against real descriptors. Every
 * read of the descriptor and of a report is bounds-checked: the descriptor
 * comes from the device and must not be trusted.
 *
 * References: Device Class Definition for HID 1.11, sections 6.2.2 (report
 * descriptor items) and 8.4 (report format); HID Usage Tables 1.4.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "hidmouse.h"


/* Usage pages and usages (HUT 1.4) */
#define HID_PAGE_GENERIC_DESKTOP 0x01u
#define HID_PAGE_BUTTON          0x09u
#define HID_PAGE_CONSUMER        0x0cu

#define HID_USAGE(page, id) (((uint32_t)(page) << 16) | (uint32_t)(id))

#define HID_GD_X      HID_USAGE(HID_PAGE_GENERIC_DESKTOP, 0x30u)
#define HID_GD_Y      HID_USAGE(HID_PAGE_GENERIC_DESKTOP, 0x31u)
#define HID_GD_WHEEL  HID_USAGE(HID_PAGE_GENERIC_DESKTOP, 0x38u)
#define HID_CON_ACPAN HID_USAGE(HID_PAGE_CONSUMER, 0x238u)

/* Item types (HID 1.11, 6.2.2.2) */
enum { hid_typeMain = 0, hid_typeGlobal = 1, hid_typeLocal = 2 };

/* Main item tags */
enum { hid_mainInput = 0x8, hid_mainOutput = 0x9, hid_mainCollection = 0xa, hid_mainFeature = 0xb, hid_mainEndCollection = 0xc };

/* Global item tags */
enum {
	hid_globalUsagePage = 0x0,
	hid_globalLogicalMin = 0x1,
	hid_globalReportSize = 0x7,
	hid_globalReportId = 0x8,
	hid_globalReportCount = 0x9,
	hid_globalPush = 0xa,
	hid_globalPop = 0xb
};

/* Local item tags */
enum { hid_localUsage = 0x0, hid_localUsageMin = 0x1, hid_localUsageMax = 0x2 };

/* Input item flags */
#define HID_INPUT_CONSTANT 0x1u
#define HID_INPUT_VARIABLE 0x2u
#define HID_INPUT_RELATIVE 0x4u

#define HID_LONG_ITEM 0xfeu

/* Parser limits: generous for any mouse, small enough to bound the work. */
#define HID_MAX_USAGES      16u   /* local Usage items per main item */
#define HID_MAX_STACK       4u    /* Push depth */
#define HID_MAX_DEPTH       16u   /* collection nesting */
#define HID_MAX_IDS         16u   /* distinct input Report IDs */
#define HID_MAX_REPORT_BITS 8192u /* per input report */
#define HID_MAX_FIELD_BITS  32u   /* largest field extracted */


typedef struct {
	uint32_t usagePage;
	int32_t logicalMin;
	uint32_t reportSize;
	uint32_t reportCount;
	uint32_t reportId;
} hid_globals_t;


typedef struct {
	hid_globals_t g;
	hid_globals_t stack[HID_MAX_STACK];
	unsigned int sp;

	/* locals, cleared after every main item */
	uint32_t usages[HID_MAX_USAGES];
	unsigned int nusages;
	int usagesOverflow;
	uint32_t usageMin;
	uint32_t usageMax;
	int haveMin;
	int haveMax;

	unsigned int depth;

	/* running input bit position per Report ID */
	struct {
		uint8_t id;
		uint32_t bits;
	} ids[HID_MAX_IDS];
	unsigned int nids;
	int sawIdlessInput;
	int sawIdInput;

	uint32_t buttonsSeen; /* Button usages 1..32 declared */
	int absXY;            /* an absolute X/Y was seen (tablet-like) */
} hid_parser_t;


static uint32_t hid_itemUnsigned(const uint8_t *data, unsigned int size)
{
	uint32_t v = 0;
	unsigned int i;

	for (i = size; i > 0u; --i) {
		v = (v << 8) | data[i - 1u];
	}

	return v;
}


static int32_t hid_itemSigned(const uint8_t *data, unsigned int size)
{
	uint32_t v = hid_itemUnsigned(data, size);

	if ((size > 0u) && (size < 4u) && ((v & (1u << (size * 8u - 1u))) != 0u)) {
		v |= ~((1u << (size * 8u)) - 1u);
	}

	return (int32_t)v;
}


/* A Usage of 1 or 2 bytes selects an ID on the current Usage Page; a 4-byte
 * Usage carries its own page in the high half (HID 1.11, 6.2.2.8). */
static uint32_t hid_extendedUsage(const hid_parser_t *p, uint32_t v, unsigned int size)
{
	return (size == 4u) ? v : HID_USAGE(p->g.usagePage, v & 0xffffu);
}


static void hid_clearLocals(hid_parser_t *p)
{
	p->nusages = 0;
	p->usagesOverflow = 0;
	p->haveMin = 0;
	p->haveMax = 0;
}


static uint32_t *hid_idBits(hid_parser_t *p, uint8_t id)
{
	unsigned int i;

	for (i = 0; i < p->nids; ++i) {
		if (p->ids[i].id == id) {
			return &p->ids[i].bits;
		}
	}

	if (p->nids >= HID_MAX_IDS) {
		return NULL;
	}

	p->ids[p->nids].id = id;
	p->ids[p->nids].bits = 0;
	return &p->ids[p->nids++].bits;
}


/* Usage of the i-th field of the current main item; 0 = none. */
static uint32_t hid_fieldUsage(const hid_parser_t *p, uint32_t i)
{
	if (p->nusages > 0u) {
		if (i < p->nusages) {
			return p->usages[i];
		}
		/* HID 1.11, 6.2.2.8: the last usage applies to the remaining controls --
		 * unless some usages were dropped for lack of room */
		return (p->usagesOverflow != 0) ? 0u : p->usages[p->nusages - 1u];
	}

	if ((p->haveMin != 0) && (p->haveMax != 0) && (p->usageMin <= p->usageMax) && (i <= p->usageMax - p->usageMin)) {
		return p->usageMin + i;
	}

	return 0;
}


static void hid_setField(hidmouse_field_t *f, const hid_parser_t *p, uint32_t off, uint32_t size)
{
	f->off = (uint16_t)off;
	f->size = (uint8_t)size;
	f->id = (uint8_t)p->g.reportId;
	f->isSigned = (p->g.logicalMin < 0) ? 1u : 0u;
}


static int hid_input(hid_parser_t *p, hidmouse_layout_t *l, uint32_t flags, const char **reason)
{
	uint32_t *bits;
	uint32_t size = p->g.reportSize;
	uint32_t count = p->g.reportCount;
	uint32_t i;
	uint32_t usage;
	uint32_t off;

	if (p->g.reportId == 0u) {
		p->sawIdlessInput = 1;
	}
	else {
		p->sawIdInput = 1;
	}
	if ((p->sawIdlessInput != 0) && (p->sawIdInput != 0)) {
		/* With Report IDs, every report carries one (HID 1.11, 6.2.2.7). */
		*reason = "mixed report ids";
		return -EINVAL;
	}

	bits = hid_idBits(p, (uint8_t)p->g.reportId);
	if (bits == NULL) {
		*reason = "too many report ids";
		return -EINVAL;
	}

	if ((size > HID_MAX_REPORT_BITS) || (count > HID_MAX_REPORT_BITS) || (size * count > HID_MAX_REPORT_BITS - *bits)) {
		*reason = "report too long";
		return -EINVAL;
	}

	off = *bits;
	*bits += size * count;

	if (((flags & HID_INPUT_CONSTANT) != 0u) || ((flags & HID_INPUT_VARIABLE) == 0u) || (size == 0u) || (size > HID_MAX_FIELD_BITS)) {
		/* padding, array fields and oversized fields carry nothing we map */
		return 0;
	}

	for (i = 0; i < count; ++i, off += size) {
		usage = hid_fieldUsage(p, i);

		if ((usage >> 16) == HID_PAGE_BUTTON) {
			usage &= 0xffffu;
			if ((usage >= 1u) && (usage <= 32u)) {
				p->buttonsSeen |= 1uL << (usage - 1u);
			}
			if ((usage >= 1u) && (usage <= HIDMOUSE_MAX_BUTTONS) && (l->button[usage - 1u].size == 0u)) {
				hid_setField(&l->button[usage - 1u], p, off, size);
			}
			continue;
		}

		if ((usage == HID_GD_X) || (usage == HID_GD_Y)) {
			if ((flags & HID_INPUT_RELATIVE) == 0u) {
				p->absXY = 1;
			}
			else if (size >= 2u) {
				hidmouse_field_t *f = (usage == HID_GD_X) ? &l->x : &l->y;
				if (f->size == 0u) {
					hid_setField(f, p, off, size);
				}
			}
		}
		else if ((usage == HID_GD_WHEEL) && ((flags & HID_INPUT_RELATIVE) != 0u) && (size >= 2u) && (l->wheel.size == 0u)) {
			hid_setField(&l->wheel, p, off, size);
		}
		else if ((usage == HID_CON_ACPAN) && ((flags & HID_INPUT_RELATIVE) != 0u) && (size >= 2u) && (l->hwheel.size == 0u)) {
			hid_setField(&l->hwheel, p, off, size);
		}
	}

	return 0;
}


static int hid_main(hid_parser_t *p, hidmouse_layout_t *l, unsigned int tag, uint32_t v, const char **reason)
{
	int err = 0;

	switch (tag) {
		case hid_mainInput:
			err = hid_input(p, l, v, reason);
			break;

		case hid_mainOutput:
		case hid_mainFeature:
			/* separate report spaces: they don't move input offsets */
			break;

		case hid_mainCollection:
			if (p->depth >= HID_MAX_DEPTH) {
				*reason = "collections too deep";
				err = -EINVAL;
			}
			else {
				p->depth++;
			}
			break;

		case hid_mainEndCollection:
			if (p->depth == 0u) {
				*reason = "unbalanced collection";
				err = -EINVAL;
			}
			else {
				p->depth--;
			}
			break;

		default:
			*reason = "bad main item";
			err = -EINVAL;
			break;
	}

	hid_clearLocals(p);
	return err;
}


static int hid_global(hid_parser_t *p, unsigned int tag, const uint8_t *data, unsigned int size, const char **reason)
{
	uint32_t v = hid_itemUnsigned(data, size);

	switch (tag) {
		case hid_globalUsagePage:
			p->g.usagePage = v & 0xffffu;
			break;

		case hid_globalLogicalMin:
			p->g.logicalMin = hid_itemSigned(data, size);
			break;

		case hid_globalReportSize:
			p->g.reportSize = v;
			break;

		case hid_globalReportCount:
			p->g.reportCount = v;
			break;

		case hid_globalReportId:
			if ((v == 0u) || (v > 0xffu)) {
				*reason = "bad report id";
				return -EINVAL;
			}
			p->g.reportId = v;
			break;

		case hid_globalPush:
			if (p->sp >= HID_MAX_STACK) {
				*reason = "push overflow";
				return -EINVAL;
			}
			p->stack[p->sp++] = p->g;
			break;

		case hid_globalPop:
			if (p->sp == 0u) {
				*reason = "pop underflow";
				return -EINVAL;
			}
			p->g = p->stack[--p->sp];
			break;

		default:
			/* Logical Max, Physical Min/Max, Unit, Unit Exponent: unused */
			break;
	}

	return 0;
}


static void hid_local(hid_parser_t *p, unsigned int tag, uint32_t v, unsigned int size)
{
	switch (tag) {
		case hid_localUsage:
			if (p->nusages < HID_MAX_USAGES) {
				p->usages[p->nusages++] = hid_extendedUsage(p, v, size);
			}
			else {
				p->usagesOverflow = 1;
			}
			break;

		case hid_localUsageMin:
			p->usageMin = hid_extendedUsage(p, v, size);
			p->haveMin = 1;
			break;

		case hid_localUsageMax:
			p->usageMax = hid_extendedUsage(p, v, size);
			p->haveMax = 1;
			break;

		default:
			/* designators, strings, delimiters: unused */
			break;
	}
}


int hidmouse_parse(const uint8_t *desc, size_t len, hidmouse_layout_t *layout, const char **reason)
{
	static const unsigned int dataSize[4] = { 0, 1, 2, 4 };
	hid_parser_t p;
	const char *why = NULL;
	size_t pos = 0;
	unsigned int i;
	uint32_t maxBits = 0;
	int err = 0;

	memset(&p, 0, sizeof(p));
	memset(layout, 0, sizeof(*layout));

	if ((desc == NULL) || (len == 0u)) {
		why = "empty descriptor";
		err = -EINVAL;
	}

	while ((err == 0) && (pos < len)) {
		uint8_t prefix = desc[pos];
		unsigned int size;
		unsigned int type;
		unsigned int tag;

		if (prefix == HID_LONG_ITEM) {
			/* long items carry no data defined by HID 1.11: skip them */
			if (len - pos < 3u) {
				why = "truncated long item";
				err = -EINVAL;
				break;
			}
			if ((size_t)desc[pos + 1u] > len - pos - 3u) {
				why = "truncated long item";
				err = -EINVAL;
				break;
			}
			pos += 3u + desc[pos + 1u];
			continue;
		}

		size = dataSize[prefix & 0x3u];
		type = (prefix >> 2) & 0x3u;
		tag = prefix >> 4;

		if ((size_t)size > len - pos - 1u) {
			why = "truncated item";
			err = -EINVAL;
			break;
		}

		switch (type) {
			case hid_typeMain:
				err = hid_main(&p, layout, tag, hid_itemUnsigned(&desc[pos + 1u], size), &why);
				break;

			case hid_typeGlobal:
				err = hid_global(&p, tag, &desc[pos + 1u], size, &why);
				break;

			case hid_typeLocal:
				hid_local(&p, tag, hid_itemUnsigned(&desc[pos + 1u], size), size);
				break;

			default:
				why = "reserved item";
				err = -EINVAL;
				break;
		}

		pos += 1u + size;
	}

	if ((err == 0) && (p.depth != 0u)) {
		why = "unbalanced collection";
		err = -EINVAL;
	}

	if ((err == 0) && ((layout->x.size == 0u) || (layout->y.size == 0u))) {
		why = (p.absXY != 0) ? "absolute X/Y" : "no relative X/Y";
		err = -ENOENT;
	}

	if (err == 0) {
		for (i = 0; i < p.nids; ++i) {
			if (p.ids[i].bits > maxBits) {
				maxBits = p.ids[i].bits;
			}
		}
		layout->useIds = (p.sawIdlessInput != 0) ? 0u : 1u;
		layout->maxReportLen = (uint16_t)((maxBits + 7u) / 8u + layout->useIds);
		for (i = 0; i < 32u; ++i) {
			if ((p.buttonsSeen & (1uL << i)) != 0u) {
				layout->nbuttons++;
			}
		}
	}
	else {
		memset(layout, 0, sizeof(*layout));
	}

	if (reason != NULL) {
		*reason = why;
	}

	return err;
}


int hidmouse_findInterface(const uint8_t *conf, size_t len, unsigned int iface, size_t *reportDescLen, size_t *maxPacket)
{
	/* USB 2.0, 9.4.3 / 9.6.5-9.6.6 and HID 1.11, 6.2.1 */
	enum { descInterface = 0x04, descEndpoint = 0x05, descHid = 0x21, descReport = 0x22 };
	size_t pos = 0;
	size_t dlen;
	size_t i;
	int inIface = 0;

	*reportDescLen = 0;
	*maxPacket = 0;

	while ((pos + 2u <= len) && (conf[pos] >= 2u) && (conf[pos] <= len - pos)) {
		dlen = conf[pos];

		switch (conf[pos + 1u]) {
			case descInterface:
				/* bInterfaceNumber, bAlternateSetting */
				inIface = (dlen >= 9u) && (conf[pos + 2u] == iface) && (conf[pos + 3u] == 0u);
				break;

			case descHid:
				/* bNumDescriptors at 5, then (bDescriptorType, wDescriptorLength) */
				if ((inIface != 0) && (dlen >= 6u)) {
					for (i = 0; (i < conf[pos + 5u]) && (6u + 3u * i + 3u <= dlen); ++i) {
						if (conf[pos + 6u + 3u * i] == descReport) {
							*reportDescLen = conf[pos + 7u + 3u * i] | ((size_t)conf[pos + 8u + 3u * i] << 8);
							break;
						}
					}
				}
				break;

			case descEndpoint:
				/* the first interrupt-IN endpoint: bEndpointAddress bit 7, bmAttributes 3 */
				if ((inIface != 0) && (dlen >= 7u) && (*maxPacket == 0u) &&
						((conf[pos + 2u] & 0x80u) != 0u) && ((conf[pos + 3u] & 0x3u) == 0x3u)) {
					*maxPacket = (conf[pos + 4u] | ((size_t)conf[pos + 5u] << 8)) & 0x7ffu;
				}
				break;

			default:
				break;
		}

		pos += dlen;
	}

	return ((*reportDescLen != 0u) && (*maxPacket != 0u)) ? 0 : -ENOENT;
}


/* Little-endian bit field of `size` (1..32) bits at bit `off` (HID 1.11, 8.4). */
static int32_t hid_extract(const hidmouse_field_t *f, const uint8_t *data)
{
	unsigned int first = f->off / 8u;
	unsigned int last = (f->off + f->size - 1u) / 8u;
	unsigned int i;
	uint64_t v = 0;

	for (i = last + 1u; i > first; --i) {
		v = (v << 8) | data[i - 1u];
	}
	v >>= f->off % 8u;
	v &= (1uLL << f->size) - 1u;

	if ((f->isSigned != 0u) && ((v & (1uLL << (f->size - 1u))) != 0u)) {
		return (int32_t)((int64_t)v - (int64_t)(1uLL << f->size));
	}

	return (int32_t)v;
}


/* The field is carried by this report and fits in its `plen` payload bytes. A
 * field past the end of a short report reads as absent rather than dropping the
 * report: losing a wheel is better than losing the pointer. */
static int hid_fieldIn(const hidmouse_field_t *f, uint8_t id, size_t plen)
{
	return (f->size != 0u) && (f->id == id) && ((size_t)f->off + f->size <= plen * 8u);
}


static int32_t hid_clamp(int32_t v, int32_t lim)
{
	return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}


size_t hidmouse_translate(const hidmouse_layout_t *layout, uint8_t *buttons, const uint8_t *report, size_t len, uint8_t *out, size_t outsz)
{
	const uint8_t *payload = report;
	size_t plen = len;
	uint8_t id = 0;
	int32_t dx = 0;
	int32_t dy = 0;
	int32_t wheel = 0;
	int32_t sx;
	int32_t sy;
	int found = 0;
	unsigned int i;
	size_t n = 0;

	if (layout->useIds != 0u) {
		if (len < 1u) {
			return 0;
		}
		id = report[0];
		payload = report + 1;
		plen = len - 1u;
	}

	for (i = 0; i < HIDMOUSE_MAX_BUTTONS; ++i) {
		if (hid_fieldIn(&layout->button[i], id, plen) != 0) {
			found = 1;
			if (hid_extract(&layout->button[i], payload) != 0) {
				*buttons |= (uint8_t)(1u << i);
			}
			else {
				*buttons &= (uint8_t)~(1u << i);
			}
		}
	}
	if (hid_fieldIn(&layout->x, id, plen) != 0) {
		found = 1;
		dx = hid_extract(&layout->x, payload);
	}
	if (hid_fieldIn(&layout->y, id, plen) != 0) {
		found = 1;
		dy = hid_extract(&layout->y, payload);
	}
	if (hid_fieldIn(&layout->wheel, id, plen) != 0) {
		found = 1;
		wheel = hid_clamp(hid_extract(&layout->wheel, payload), 127);
	}

	/* another report of the device (consumer keys, vendor) */
	if (found == 0) {
		return 0;
	}

	/* One packet, plus more while the motion exceeds the int8 range. The wheel
	 * and the button state ride in the first; the rest are pure motion. */
	do {
		if (outsz - n < HIDMOUSE_PKT_SIZE) {
			break;
		}
		sx = hid_clamp(dx, 127);
		sy = hid_clamp(dy, 127);
		out[n] = *buttons;
		out[n + 1u] = (uint8_t)(int8_t)sx;
		out[n + 2u] = (uint8_t)(int8_t)sy;
		out[n + 3u] = (uint8_t)(int8_t)((n == 0u) ? wheel : 0);
		dx -= sx;
		dy -= sy;
		n += HIDMOUSE_PKT_SIZE;
	} while (((dx != 0) || (dy != 0)) && (n < HIDMOUSE_MAX_SPLIT * HIDMOUSE_PKT_SIZE));

	return n;
}


static void hid_describeField(char *buf, size_t size, const char *name, const hidmouse_field_t *f, int useIds)
{
	if (f->size == 0u) {
		(void)snprintf(buf, size, " %s=none", name);
	}
	else if (useIds != 0) {
		(void)snprintf(buf, size, " %s=%u@%u#%u", name, (unsigned int)f->size, (unsigned int)f->off, (unsigned int)f->id);
	}
	else {
		(void)snprintf(buf, size, " %s=%u@%u", name, (unsigned int)f->size, (unsigned int)f->off);
	}
}


void hidmouse_describe(const hidmouse_layout_t *layout, char *buf, size_t size)
{
	char tmp[32];
	size_t used;

	if (size == 0u) {
		return;
	}

	if (layout->useIds != 0u) {
		(void)snprintf(buf, size, "id=%u buttons=%u len=%u", (unsigned int)layout->x.id, (unsigned int)layout->nbuttons, (unsigned int)layout->maxReportLen);
	}
	else {
		(void)snprintf(buf, size, "id=none buttons=%u len=%u", (unsigned int)layout->nbuttons, (unsigned int)layout->maxReportLen);
	}

	hid_describeField(tmp, sizeof(tmp), "x", &layout->x, layout->useIds);
	used = strlen(buf);
	(void)snprintf(buf + used, size - used, "%s", tmp);
	hid_describeField(tmp, sizeof(tmp), "y", &layout->y, layout->useIds);
	used = strlen(buf);
	(void)snprintf(buf + used, size - used, "%s", tmp);
	hid_describeField(tmp, sizeof(tmp), "wheel", &layout->wheel, layout->useIds);
	used = strlen(buf);
	(void)snprintf(buf + used, size - used, "%s", tmp);
	hid_describeField(tmp, sizeof(tmp), "hwheel", &layout->hwheel, layout->useIds);
	used = strlen(buf);
	(void)snprintf(buf + used, size - used, "%s", tmp);
}
