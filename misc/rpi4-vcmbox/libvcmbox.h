/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) VideoCore property-mailbox client - ABI + helpers
 *
 * The BCM2711 has a single VideoCore property mailbox (one hardware FIFO at
 * 0xfe00b880, channel 8). It is the only way to read SoC temperature, the board
 * MAC, throttle state, and to power/clock the V3D, HVS, and USB blocks. The FIFO
 * has no hardware arbitration: if two processes drive it concurrently, one read
 * loop pops and discards the other's response, destroying it. The rpi4-vcmbox
 * server (/dev/vcmbox) owns the FIFO and serializes every transaction (a Phoenix
 * server handles one msg at a time), so callers MUST route mailbox traffic
 * through this client rather than open the FIFO themselves.
 *
 * Wire format: a property call is sent as an mtDevCtl message whose request and
 * response live entirely in msg.i.raw / msg.o.raw (no msg.i.data buffer), so the
 * helper works in low-level driver processes that run before posixsrv / "/" and
 * cannot rely on the libc open()+ioctl() path.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBVCMBOX_H_
#define _LIBVCMBOX_H_

#include <stdint.h>


/* Property-call request/response carried in msg.i.raw / msg.o.raw (64 B each).
 * Designed multi-word from the start: the 1-in/1-out case (thermal, throttle,
 * V3D power) is a thin wrapper, but GET_BOARD_MAC (tag 0x10003) returns TWO
 * words, so the general form is required for the other clients. */

#define VCMBOX_MAX_WORDS 12u /* fits comfortably in the 64-byte raw region */

typedef struct {
	uint32_t tag;                    /* VideoCore property tag */
	uint32_t valBufSize;            /* firmware value-buffer size, bytes (e.g. 8) */
	uint32_t nIn;                    /* number of valid input words in `in` */
	uint32_t in[VCMBOX_MAX_WORDS];   /* request words written into the value buffer */
} vcmbox_req_t;

typedef struct {
	int err;                         /* 0 on success, -EIO on failure */
	uint32_t nOut;                   /* number of valid response words in `out` */
	uint32_t out[VCMBOX_MAX_WORDS];  /* firmware response words read back */
} vcmbox_resp_t;


/*
 * Run one property transaction through the /dev/vcmbox server. On first use it
 * blocks until the server has registered the node (standard Phoenix retry-lookup
 * pattern), then sends one mtDevCtl msg. `nOut` response words are copied into
 * `out` (caller-provided, at least `nOut` words). Returns 0 on success, or a
 * negative errno (the IPC error, or the server's -EIO if the firmware
 * transaction ultimately failed). `out` may be NULL when the caller expects no
 * response words.
 */
int vcmbox_call(uint32_t tag, uint32_t valBufSize, const uint32_t *in, uint32_t nIn,
	uint32_t *out, uint32_t nOut);

/*
 * Convenience wrapper for the common single-u32-in / single-u32-out property
 * call (temperature, throttle, V3D power-on, ...). Returns 0 and stores the
 * firmware response word in *out, or a negative errno on failure.
 */
int vcmbox_prop(uint32_t tag, uint32_t arg_in, uint32_t *out);


#endif
