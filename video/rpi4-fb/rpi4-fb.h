/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) VideoCore framebuffer - client ABI
 *
 * Provisional Phoenix-native geometry query for /dev/fb0. A Linux-fbdev
 * (FBIOGET_VSCREENINFO / FBIOGET_FSCREENINFO) veneer is deliberately NOT
 * provided yet: that ABI choice (and a true mmap(fd, 0) backing, which needs
 * new kernel VM work) is an attended upstreaming decision. Clients that only
 * need geometry can equivalently call platformctl(pctl_get, pctl_graphmode)
 * directly - this devctl just makes /dev/fb0 self-describing.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RPI4_FB_H_
#define _RPI4_FB_H_

#include <stdint.h>
#include <sys/ioctl.h>


typedef struct {
	uint16_t width;       /* visible width in pixels */
	uint16_t height;      /* visible height in pixels */
	uint16_t bpp;         /* bits per pixel (32 on Pi 4) */
	uint16_t pitch;       /* bytes per scanline */
	uint64_t smemlen;     /* framebuffer size in bytes (pitch * height) */
	uint64_t framebuffer; /* physical base address (for MAP_PHYSMEM clients) */
} rpi4fb_mode_t;


/* Read the current framebuffer geometry. */
#define RPI4FB_GETMODE _IOR('g', 1, rpi4fb_mode_t)


#endif
