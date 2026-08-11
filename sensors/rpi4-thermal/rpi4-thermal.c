/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) SoC thermal / throttle sensor
 *
 * Exposes two read-only pseudo-files backed by the VideoCore firmware
 * mailbox property interface:
 *   /dev/thermal    - SoC temperature, milli-degrees Celsius ("<mC>\n")
 *   /dev/throttled  - under-voltage / throttle bitfield ("0x%08x\n")
 *
 * The BCM2711 has no software-readable thermal register block; temperature
 * is owned by the VideoCore firmware and read over the property mailbox at
 * 0xfe00b880 (channel 8). The firmware also enforces the hard thermal trip
 * (~85 C max, reported via GET_MAX_TEMPERATURE), so this driver is telemetry
 * only - no trip configuration or throttling governor.
 *
 * The mailbox FIFO is a single, hardware-arbitration-free shared peripheral, so
 * this driver does NOT drive it directly: every property call is routed through
 * the rpi4-vcmbox server (/dev/vcmbox) via libvcmbox, which owns the FIFO and
 * serializes all callers (see misc/rpi4-vcmbox/). That kills the cross-process
 * race that used to surface as transient mailbox read failures.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/types.h>
#include <posix/utils.h>

#include <libvcmbox.h>


/* VideoCore property tags this driver reads (BCM2711 firmware mailbox). */
#define VC_PROP_GET_TEMPERATURE 0x00030006u
#define VC_PROP_GET_MAX_TEMP    0x0003000au
#define VC_PROP_GET_THROTTLED   0x00030046u

/* Device node ids (one port, two nodes). */
#define DEV_THERMAL_ID          0
#define DEV_THROTTLED_ID        1


/* SoC temperature, milli-Celsius (sensor 0). Negative errno on failure. */
static int thermal_temp(uint32_t *out)
{
	return vcmbox_prop(VC_PROP_GET_TEMPERATURE, 0, out);
}


/* Firmware-enforced max temperature, milli-Celsius (sensor 0). */
static int thermal_max(uint32_t *out)
{
	return vcmbox_prop(VC_PROP_GET_MAX_TEMP, 0, out);
}


/* Throttle / under-voltage bitfield (sticky bits 16-19 = "since boot"). */
static int thermal_throttled(uint32_t *out)
{
	return vcmbox_prop(VC_PROP_GET_THROTTLED, 0, out);
}


/* Render the requested node's current value into buf. Returns the byte count,
 * or -EIO if the mailbox query failed (so read() reports an error rather than
 * formatting a bogus value). */
static int thermal_format(id_t id, char *buf, size_t size)
{
	uint32_t v;
	int rc = (id == DEV_THROTTLED_ID) ? thermal_throttled(&v) : thermal_temp(&v);

	if (rc != 0) {
		return -EIO;
	}
	if (id == DEV_THROTTLED_ID) {
		return snprintf(buf, size, "0x%08x\n", v);
	}
	return snprintf(buf, size, "%u\n", v);
}


/* read(): offset-aware slice of the rendered value (mirrors rpi4-gpio's gpio_read).
 * Renders into a local buffer first, then clamps to the caller's size — snprintf
 * returns the length it WOULD have written, which can exceed the client buffer;
 * handing that back as the read count would over-report and desync the reader. */
static int thermal_read(id_t id, off_t offs, char *dst, size_t size)
{
	char scratch[32];
	int len = thermal_format(id, scratch, sizeof(scratch));

	if (len < 0) {
		return len; /* -EIO from a failed mailbox query */
	}
	if ((offs < 0) || (offs >= len)) {
		return 0;
	}
	if (size > (size_t)(len - offs)) {
		size = (size_t)(len - offs);
	}
	memcpy(dst, scratch + offs, size);
	return (int)size;
}


static void thermal_thread(void *arg)
{
	uint32_t port = (uint32_t)(uintptr_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			break; /* invalid/closed port or OOM - fatal */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtWrite:
				msg.o.err = -EINVAL;
				break;

			case mtRead:
				msg.o.err = thermal_read(msg.oid.id, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


int main(int argc, char **argv)
{
	uint32_t port;
	oid_t dev;
	uint32_t temp = 0, max = 0, throttle = 0;
	int haveTemp, haveMax, haveThrottle;

	(void)argc;
	(void)argv;

	/* Read the startup banner values through the serializing mailbox server.
	 * A failure here is NOT fatal: the server retries internally, and on-demand
	 * read()s go back through it, so we register the device nodes regardless and
	 * just note which banner values were unavailable. */
	haveTemp = (thermal_temp(&temp) == 0);
	haveMax = (thermal_max(&max) == 0);
	haveThrottle = (thermal_throttled(&throttle) == 0);

	if (portCreate(&port) != EOK) {
		printf("rpi4-thermal: portCreate failed\n");
		return 2;
	}

	dev.port = port;

	dev.id = DEV_THERMAL_ID;
	if (create_dev(&dev, "thermal") < 0) {
		printf("rpi4-thermal: could not create /dev/thermal\n");
		return 3;
	}

	dev.id = DEV_THROTTLED_ID;
	if (create_dev(&dev, "throttled") < 0) {
		printf("rpi4-thermal: could not create /dev/throttled\n");
		return 3;
	}

	if (haveTemp && haveMax && haveThrottle) {
		printf("rpi4-thermal: T=%u mC max=%u mC throttle=0x%08x; registered /dev/thermal /dev/throttled\n",
			temp, max, throttle);
	}
	else {
		printf("rpi4-thermal: registered /dev/thermal /dev/throttled (startup banner read incomplete: "
			"temp=%d max=%d throttle=%d; on-demand reads retry via /dev/vcmbox)\n",
			haveTemp, haveMax, haveThrottle);
	}

	thermal_thread((void *)(uintptr_t)port);

	return 0;
}
