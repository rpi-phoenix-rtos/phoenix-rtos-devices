/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - scan-out rules for imported
 * buffers (gap G7)
 *
 * Pure functions (no Phoenix headers, no server state), so the host test
 * tools/gpu-lane/kms/hosttest/run.sh checks exactly the rules the server applies.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _KMS_SCANOUT_H_
#define _KMS_SCANOUT_H_

#include <stddef.h>
#include <stdint.h>

#include "kms_proto.h"


/* The firmware plane fetches nothing at or above 1 GiB (E3/E6 `range hi`). */
#define KMS_SCANOUT_LIMIT (1ULL << 30)

/* Row pitch and start offset of an imported framebuffer: what the pool's own BOs
 * get (64-byte rows, kms_bo.c bo_from_pool) [inferred: safe for the HVS]. */
#define KMS_IMPORT_ALIGN 64u


/* Can the plane fetch an imported buffer of `size` bytes at physical `pa`?
 * NULL = yes; else the reason, as it appears in the tagged log lines. */
static inline const char *kms_import_why(uint64_t pa, uint64_t size, int contiguous)
{
	if (contiguous == 0) {
		return "noncontig";   /* the plane scans one linear physical range */
	}
	if ((size == 0u) || (pa + size > KMS_SCANOUT_LIMIT) || (pa + size < pa)) {
		return "above_1g";
	}
	return NULL;
}


/* ADDFB2 of an imported buffer: the layout rules on top of the generic ones
 * (kms_fb_add: format, width/height, pitch >= width * 4, offset + pitch * height
 * within the buffer). `why` is the buffer's own verdict (kms_import_why). NULL = ok. */
static inline const char *kms_import_fb_why(const kms_addfb2_req_t *rq, uint64_t bo_size, const char *why)
{
	if (why != NULL) {
		return why;
	}
	if (rq->modifier != KMS_MOD_LINEAR) {
		return "modifier";   /* no Broadcom UIF/SAND/T-tiled scan-out on this path */
	}
	if (((rq->pitch % KMS_IMPORT_ALIGN) != 0u) || ((rq->offset % KMS_IMPORT_ALIGN) != 0u)) {
		return "align";
	}
	if ((rq->width == 0u) || (rq->height == 0u) || (rq->pitch < rq->width * 4u) ||
			((uint64_t)rq->offset + (uint64_t)rq->pitch * rq->height > bo_size)) {
		return "size";
	}
	return NULL;
}

#endif /* _KMS_SCANOUT_H_ */
