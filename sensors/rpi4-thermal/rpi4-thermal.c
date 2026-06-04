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
#include <sys/mman.h>
#include <sys/types.h>
#include <posix/utils.h>


/* VideoCore property mailbox (see BCM2711 ARM peripherals + plo video.c). */
#define RPI_MAILBOX_BASE        0xfe00b880u

#define VC_MBOX_STATUS          0x18u
#define VC_MBOX_WRITE           0x20u
#define VC_MBOX_READ            0x00u
#define VC_MBOX_STATUS_FULL     0x80000000u
#define VC_MBOX_STATUS_EMPTY    0x40000000u
#define VC_MBOX_RESP_OK         0x80000000u
#define VC_MBOX_PROP_CHANNEL    8u

#define VC_PROP_GET_TEMPERATURE 0x00030006u
#define VC_PROP_GET_MAX_TEMP    0x0003000au
#define VC_PROP_GET_THROTTLED   0x00030046u

#define MBOX_FAIL               0xffffffffu

/* Device node ids (one port, two nodes). */
#define DEV_THERMAL_ID          0
#define DEV_THROTTLED_ID        1


/*
 * Single-u32-in / single-u32-out property call. The protocol and cache
 * discipline (device-mapped uncached mailbox window + uncached, contiguous,
 * physically-pinned property buffer) are carried over verbatim from the
 * validated diag-udp scout (phoenix-rtos-lwip/port/diag-udp.c). The buffer
 * is mapped uncached, so no explicit clean/invalidate is needed around the
 * firmware round-trip. Returns the firmware response word, or MBOX_FAIL.
 */
static uint32_t rpi4_mboxProp1in1out(uint32_t tag, uint32_t arg_in)
{
	addr_t pa_base = (addr_t)RPI_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request;
	uint32_t result = MBOX_FAIL;
	void *mbox_page;
	void *msg_page;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, pa_base);
	if (mbox_page == MAP_FAILED) {
		return MBOX_FAIL;
	}
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return MBOX_FAIL;
	}
	msg = msg_page;

	/* tag layout: [size, REQUEST, tag, valbuf_size=8, req=0, arg_in, out, END]. */
	msg[0] = 32;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = 8;
	msg[4] = 0;
	msg[5] = arg_in;
	msg[6] = 0;
	msg[7] = 0;

	msg_pa = (uintptr_t)va2pa(msg);
	if (msg_pa == (uintptr_t)-1) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return MBOX_FAIL;
	}
	request = ((uint32_t)msg_pa & ~0xFu) | VC_MBOX_PROP_CHANNEL;

	while ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_FULL) != 0u) {
	}
	mbox[VC_MBOX_WRITE / 4] = request;

	for (;;) {
		while ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) != 0u) {
		}
		if (mbox[VC_MBOX_READ / 4] == request) {
			break;
		}
	}

	if (msg[1] == VC_MBOX_RESP_OK) {
		result = msg[6];
	}

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}


/* SoC temperature, milli-Celsius (sensor 0). MBOX_FAIL on error. */
static uint32_t thermal_temp(void)
{
	return rpi4_mboxProp1in1out(VC_PROP_GET_TEMPERATURE, 0);
}


/* Firmware-enforced max temperature, milli-Celsius (sensor 0). */
static uint32_t thermal_max(void)
{
	return rpi4_mboxProp1in1out(VC_PROP_GET_MAX_TEMP, 0);
}


/* Throttle / under-voltage bitfield (sticky bits 16-19 = "since boot"). */
static uint32_t thermal_throttled(void)
{
	return rpi4_mboxProp1in1out(VC_PROP_GET_THROTTLED, 0);
}


/* Render the requested node's current value into buf. Returns length. */
static int thermal_format(id_t id, char *buf, size_t size)
{
	if (id == DEV_THROTTLED_ID) {
		return snprintf(buf, size, "0x%08x\n", thermal_throttled());
	}
	return snprintf(buf, size, "%u\n", thermal_temp());
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
				/* No partial reads: any non-zero offset signals EOF. */
				if (msg.i.io.offs > 0) {
					msg.o.err = 0;
				}
				else {
					msg.o.err = thermal_format(msg.oid.id, msg.o.data, msg.o.size);
				}
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
	uint32_t temp, max, throttle;

	(void)argc;
	(void)argv;

	temp = thermal_temp();
	if (temp == MBOX_FAIL) {
		printf("rpi4-thermal: mailbox temperature read failed\n");
		return 1;
	}
	max = thermal_max();
	throttle = thermal_throttled();

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

	printf("rpi4-thermal: T=%u mC max=%u mC throttle=0x%08x; registered /dev/thermal /dev/throttled\n",
		temp, max, throttle);

	thermal_thread((void *)(uintptr_t)port);

	return 0;
}
