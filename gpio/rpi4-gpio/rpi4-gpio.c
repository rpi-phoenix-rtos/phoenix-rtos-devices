/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) GPIO observer device
 *
 * Exposes /dev/gpio, a read-only view of the BCM2711 GPIO block at 0xfe200000:
 *   read()  - human-readable register snapshot (GPFSEL0..5, GPLEV0/1,
 *             PUP_PDN0..3), e.g. `cat /dev/gpio`
 *   RPI4GPIO_GETPIN devctl - per-pin {fsel, level, pull} query
 *
 * Read-only by design: driving outputs (GPSET/GPCLR, function-select changes)
 * needs a bench rig to validate safely and is left as an attended follow-up
 * (pi4-hardware-support-matrix: GPIO full driver is marked attended). This
 * device productionizes the #45 snapshot helpers into a real /dev node.
 *
 * Register layout per docs/knowledge/gpio-pinctrl.md. The node is named /dev/gpio
 * (singular) to stay distinct from the per-bank /dev/gpioN convention the lwIP
 * gpiosrv uses (the Pi 4 genet config uses irq:MAC, so no /dev/gpioN exists).
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
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <posix/utils.h>

#include "rpi4-gpio.h"


/* BCM2711 GPIO block (bus 0x7e200000 -> ARM low-peripheral 0xfe200000). */
#define GPIO_BASE      0xfe200000u

#define GPIO_GPFSEL0   0x00u /* +4*n for GPFSEL1..5; 3 bits/pin, 10 pins/reg */
#define GPIO_GPLEV0    0x34u /* +4 for GPLEV1; 1 bit/pin */
#define GPIO_PUP_PDN0  0xe4u /* +4*n for REG1..3; 2 bits/pin, 16 pins/reg */


static volatile uint8_t *gpio_base;


static inline uint32_t gpio_rd(unsigned off)
{
	return *(volatile uint32_t *)(gpio_base + off);
}


static unsigned gpio_fsel(unsigned pin)
{
	unsigned bank = pin / 10u;
	unsigned shift = (pin % 10u) * 3u;
	return (gpio_rd(GPIO_GPFSEL0 + bank * 4u) >> shift) & 0x7u;
}


static unsigned gpio_level(unsigned pin)
{
	unsigned bank = pin / 32u;
	return (gpio_rd(GPIO_GPLEV0 + bank * 4u) >> (pin % 32u)) & 0x1u;
}


static unsigned gpio_pull(unsigned pin)
{
	unsigned reg = pin / 16u;
	unsigned shift = (pin % 16u) * 2u;
	return (gpio_rd(GPIO_PUP_PDN0 + reg * 4u) >> shift) & 0x3u;
}


/* Render the full register snapshot into buf. Returns the byte count. */
static int gpio_snapshot(char *buf, size_t size)
{
	int len = snprintf(buf, size,
		"PHX-RPI4-GPIO\n"
		"GPFSEL0..5:  %08x %08x %08x %08x %08x %08x\n"
		"GPLEV0/1:    %08x %08x\n"
		"PUP_PDN0..3: %08x %08x %08x %08x\n",
		gpio_rd(GPIO_GPFSEL0 + 0u), gpio_rd(GPIO_GPFSEL0 + 4u), gpio_rd(GPIO_GPFSEL0 + 8u),
		gpio_rd(GPIO_GPFSEL0 + 12u), gpio_rd(GPIO_GPFSEL0 + 16u), gpio_rd(GPIO_GPFSEL0 + 20u),
		gpio_rd(GPIO_GPLEV0 + 0u), gpio_rd(GPIO_GPLEV0 + 4u),
		gpio_rd(GPIO_PUP_PDN0 + 0u), gpio_rd(GPIO_PUP_PDN0 + 4u),
		gpio_rd(GPIO_PUP_PDN0 + 8u), gpio_rd(GPIO_PUP_PDN0 + 12u));
	/* snprintf returns the would-have-written length; clamp it so gpio_read can
	 * never derive an out-of-bounds slice from an inflated len should the format
	 * ever outgrow buf (currently ~161 B, well within callers' buffers). */
	if ((len >= 0) && ((size_t)len >= size)) {
		len = (int)size - 1;
	}
	return len;
}


/* read(): offset-aware slice of the rendered snapshot. */
static int gpio_read(off_t offs, char *dst, size_t size)
{
	char snap[256];
	int len = gpio_snapshot(snap, sizeof(snap));

	if ((len < 0) || (offs < 0) || (offs >= len)) {
		return 0;
	}
	if (size > (size_t)(len - offs)) {
		size = (size_t)(len - offs);
	}
	memcpy(dst, snap + offs, size);
	return (int)size;
}


static void gpio_devctl(msg_t *msg)
{
	unsigned long req;
	id_t id;
	const void *idata;

	idata = ioctl_unpack(msg, &req, &id);

	if ((req == RPI4GPIO_GETPIN) && (idata != NULL)) {
		rpi4gpio_pin_t pin = *(const rpi4gpio_pin_t *)idata;

		if (pin.pin >= RPI4_GPIO_NPINS) {
			ioctl_setResponse(msg, req, -EINVAL, NULL);
			return;
		}
		pin.fsel = (uint8_t)gpio_fsel(pin.pin);
		pin.level = (uint8_t)gpio_level(pin.pin);
		pin.pull = (uint8_t)gpio_pull(pin.pin);
		ioctl_setResponse(msg, req, EOK, &pin);
	}
	else {
		ioctl_setResponse(msg, req, -EINVAL, NULL);
	}
}


static void gpio_thread(void *arg)
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
			break;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = gpio_read(msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				/* Read-only device: outputs are an attended follow-up. */
				msg.o.err = -EROFS;
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0444;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;

			case mtDevCtl:
				gpio_devctl(&msg);
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

	(void)argc;
	(void)argv;

	gpio_base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, (off_t)GPIO_BASE);
	if (gpio_base == MAP_FAILED) {
		printf("rpi4-gpio: mmap of GPIO block failed\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-gpio: portCreate failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "gpio") < 0) {
		printf("rpi4-gpio: could not create /dev/gpio\n");
		return 3;
	}

	/* Self-test / acceptance line: read two registers through the mapping. */
	printf("rpi4-gpio: bcm2711 gpio @ 0x%08x; GPFSEL0=0x%08x GPLEV0=0x%08x; registered /dev/gpio\n",
		GPIO_BASE, gpio_rd(GPIO_GPFSEL0), gpio_rd(GPIO_GPLEV0));

	gpio_thread((void *)(uintptr_t)port);

	return 0;
}
