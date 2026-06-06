/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) VideoCore framebuffer device
 *
 * Exposes /dev/fb0 backed by the firmware-allocated HDMI framebuffer. The
 * framebuffer base + geometry come from the plo->kernel syspage and are read
 * with platformctl(pctl_get, pctl_graphmode); the surface is mapped uncached
 * by physical address (the only zero-copy path to scanout DRAM on this port -
 * see pl011-tty fbcon, which uses the same mapping for the boot console).
 *
 * Device semantics (provisional, minimal):
 *   read()       - copy framebuffer bytes at the file offset
 *   write()      - copy bytes into the framebuffer at the file offset
 *   RPI4FB_GETMODE devctl - report {width,height,bpp,pitch,smemlen,framebuffer}
 *   getattr      - atSize = framebuffer byte size, atMode = S_IFCHR | 0666
 *
 * Deliberately NOT provided (attended upstreaming decisions, documented in
 * docs/inprogress/2026-06-05-fb0-attended-decisions.md):
 *   - a Linux-fbdev FBIOGET_* veneer (one-way ABI door),
 *   - mmap(fd, 0) returning the framebuffer (needs new kernel VM work; the
 *     in-tree device-fd mmap path demand-pages a private copy, not the FB),
 *   - any drawing at startup or display-ownership arbitration with the
 *     pl011-tty boot console (both write the same surface).
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
#include <sys/platform.h>
#include <sys/ioctl.h>
#include <posix/utils.h>

#include <phoenix/arch/aarch64/generic/generic.h>

#include "rpi4-fb.h"


static struct {
	volatile uint8_t *fb; /* uncached mapping of the framebuffer */
	size_t fbsize;        /* usable bytes (pitch * height) */
	size_t mapsize;       /* page-aligned mapped length */
	rpi4fb_mode_t mode;   /* geometry reported to clients */
} fb_common;


/* Query the firmware framebuffer geometry and map it uncached by physical
 * address. Returns 0 on success, -1 if no usable 32-bpp framebuffer exists. */
static int fb_acquire(void)
{
	platformctl_t pctl = { .action = pctl_get, .type = pctl_graphmode };

	if (platformctl(&pctl) < 0) {
		return -1;
	}

	if ((pctl.task.graphmode.bpp != 32u) || (pctl.task.graphmode.framebuffer == 0u) || (pctl.task.graphmode.pitch == 0u)) {
		return -1;
	}

	fb_common.fbsize = (size_t)pctl.task.graphmode.pitch * pctl.task.graphmode.height;
	fb_common.mapsize = (fb_common.fbsize + _PAGE_SIZE - 1u) & ~((size_t)_PAGE_SIZE - 1u);

	fb_common.fb = mmap(NULL, fb_common.mapsize, PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_UNCACHED | MAP_ANONYMOUS | MAP_PHYSMEM,
		-1, (off_t)pctl.task.graphmode.framebuffer);
	if (fb_common.fb == MAP_FAILED) {
		return -1;
	}

	fb_common.mode.width = pctl.task.graphmode.width;
	fb_common.mode.height = pctl.task.graphmode.height;
	fb_common.mode.bpp = pctl.task.graphmode.bpp;
	fb_common.mode.pitch = pctl.task.graphmode.pitch;
	fb_common.mode.smemlen = fb_common.fbsize;
	fb_common.mode.framebuffer = pctl.task.graphmode.framebuffer;

	return 0;
}


/* read(): copy framebuffer bytes at the file offset. Reads past the end
 * report EOF (0 bytes). */
static int fb_read(off_t offs, void *dst, size_t size)
{
	if ((offs < 0) || ((size_t)offs >= fb_common.fbsize)) {
		return 0;
	}
	if (size > fb_common.fbsize - (size_t)offs) {
		size = fb_common.fbsize - (size_t)offs;
	}
	memcpy(dst, (const void *)(fb_common.fb + offs), size);
	return (int)size;
}


/* write(): copy bytes into the framebuffer at the file offset. Writes past
 * the end are rejected (a short write would mask a client bug). */
static int fb_write(off_t offs, const void *src, size_t size)
{
	if ((offs < 0) || ((size_t)offs >= fb_common.fbsize)) {
		return -ENOSPC;
	}
	if (size > fb_common.fbsize - (size_t)offs) {
		size = fb_common.fbsize - (size_t)offs;
	}
	memcpy((void *)(fb_common.fb + offs), src, size);
	return (int)size;
}


static void fb_devctl(msg_t *msg)
{
	unsigned long req;
	id_t id;

	(void)ioctl_unpack(msg, &req, &id);

	if (req == RPI4FB_GETMODE) {
		ioctl_setResponse(msg, req, EOK, &fb_common.mode);
	}
	else {
		ioctl_setResponse(msg, req, -EINVAL, NULL);
	}
}


static void fb_thread(void *arg)
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
				msg.o.err = fb_read(msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = fb_write(msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				switch (msg.i.attr.type) {
					case atSize:
						msg.o.attr.val = (long long)fb_common.fbsize;
						msg.o.err = EOK;
						break;

					case atMode:
						msg.o.attr.val = S_IFCHR | 0666;
						msg.o.err = EOK;
						break;

					default:
						msg.o.err = -EINVAL;
						break;
				}
				break;

			case mtDevCtl:
				fb_devctl(&msg);
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
	uint32_t firstpx;

	(void)argc;
	(void)argv;

	if (fb_acquire() != 0) {
		printf("rpi4-fb: no usable 32-bpp framebuffer (graphmode) -- not registering\n");
		return 1;
	}

	/* Read-only self-test: prove this process can reach scanout DRAM through
	 * the uncached mapping. Routed through fb_read() (not a raw pointer deref)
	 * so the read path's bounds + memcpy run on real hardware too, the way the
	 * sibling thermal/hwrng canaries exercise their read() path. No write --
	 * drawing would race the pl011-tty boot console, which owns the surface. */
	firstpx = 0;
	if (fb_read(0, &firstpx, sizeof(firstpx)) != (int)sizeof(firstpx)) {
		printf("rpi4-fb: framebuffer read self-test failed -- not registering\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-fb: portCreate failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "fb0") < 0) {
		printf("rpi4-fb: could not create /dev/fb0\n");
		return 3;
	}

	printf("rpi4-fb: pa=0x%08llx %ux%u bpp=%u pitch=%u size=%u first_px=0x%08x; registered /dev/fb0\n",
		(unsigned long long)fb_common.mode.framebuffer, fb_common.mode.width, fb_common.mode.height,
		fb_common.mode.bpp, fb_common.mode.pitch, (unsigned int)fb_common.fbsize, firstpx);

	fb_thread((void *)(uintptr_t)port);

	return 0;
}
