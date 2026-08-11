/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) hardware RNG (iproc RNG200)
 *
 * Exposes /dev/hwrng: each read() returns fresh hardware entropy bytes drawn
 * from the BCM2711 RNG200 block (mainline Linux "brcm,bcm2711-rng200",
 * driver drivers/char/hw_random/iproc-rng200.c) at 0xfe104000.
 *
 * Stream device: the read offset is ignored; every read yields new entropy.
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


/* BCM2711 RNG200 MMIO base (bus 0x7e104000 -> ARM low-peripheral 0xfe104000). */
#define RNG200_BASE 0xfe104000u

/* Register offsets (32-bit words), from iproc-rng200.c. */
#define RNG_CTRL        (0x00u / 4u)
#define RNG_SOFT_RESET  (0x04u / 4u)
#define RBG_SOFT_RESET  (0x08u / 4u)
#define RNG_INT_STATUS  (0x18u / 4u)
#define RNG_FIFO_DATA   (0x20u / 4u)
#define RNG_FIFO_COUNT  (0x24u / 4u)

#define RNG_CTRL_RBGEN_MASK     0x00001fffu
#define RNG_CTRL_RBGEN_ENABLE   0x00000001u
#define RNG_FIFO_COUNT_MASK     0x000000ffu
#define RNG_SOFT_RESET_BIT      0x00000001u
#define RBG_SOFT_RESET_BIT      0x00000001u

/* Bounded FIFO-wait spin cap: a read can never hang the process/boot. After
 * enable the FIFO fills in well under this; exhausting it means a dead RNG. */
#define RNG_FIFO_SPINS 2000000u


static volatile uint32_t *rng_base;


static void rng_enableSet(int enable)
{
	uint32_t v = rng_base[RNG_CTRL];
	v &= ~RNG_CTRL_RBGEN_MASK;
	if (enable != 0) {
		v |= RNG_CTRL_RBGEN_ENABLE;
	}
	rng_base[RNG_CTRL] = v;
}


/* RNG200 bring-up: disable, clear interrupt status, soft-reset RBG and RNG,
 * deassert resets, re-enable. Mirrors iproc_rng200_restart(). */
static void rng_init(void)
{
	uint32_t v;

	rng_enableSet(0);

	rng_base[RNG_INT_STATUS] = 0xffffffffu; /* clear all */

	v = rng_base[RBG_SOFT_RESET] | RBG_SOFT_RESET_BIT;
	rng_base[RBG_SOFT_RESET] = v;
	v = rng_base[RNG_SOFT_RESET] | RNG_SOFT_RESET_BIT;
	rng_base[RNG_SOFT_RESET] = v;

	v = rng_base[RNG_SOFT_RESET] & ~RNG_SOFT_RESET_BIT;
	rng_base[RNG_SOFT_RESET] = v;
	v = rng_base[RBG_SOFT_RESET] & ~RBG_SOFT_RESET_BIT;
	rng_base[RBG_SOFT_RESET] = v;

	rng_enableSet(1);
}


/* Fetch one 32-bit random word. Returns 0 on success, -1 if the FIFO never
 * produced data within the spin bound. */
static int rng_readWord(uint32_t *out)
{
	uint32_t spins;

	for (spins = 0; spins < RNG_FIFO_SPINS; spins++) {
		if ((rng_base[RNG_FIFO_COUNT] & RNG_FIFO_COUNT_MASK) != 0u) {
			*out = rng_base[RNG_FIFO_DATA];
			return 0;
		}
	}
	return -1;
}


static void rng_thread(void *arg)
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

			case mtWrite:
				msg.o.err = -EINVAL;
				break;

			case mtRead: {
				/* Stream device: offset ignored, every read is fresh
				 * entropy. Fill the caller's buffer word-by-word. */
				char *dst = msg.o.data;
				size_t want = msg.o.size;
				size_t got = 0;
				while (got < want) {
					uint32_t w;
					size_t n;
					if (rng_readWord(&w) != 0) {
						break;
					}
					n = want - got;
					if (n > sizeof(w)) {
						n = sizeof(w);
					}
					memcpy(dst + got, &w, n);
					got += n;
				}
				/* POSIX: a zero-length read returns 0, not an error; -EIO only
				 * when a non-empty request could not draw any entropy. */
				msg.o.err = (got > 0) ? (int)got : ((want == 0u) ? 0 : -EIO);
				break;
			}

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
	uint32_t s0 = 0, s1 = 0;

	(void)argc;
	(void)argv;

	rng_base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, (off_t)RNG200_BASE);
	if (rng_base == MAP_FAILED) {
		printf("rpi4-hwrng: mmap of RNG200 failed\n");
		return 1;
	}

	rng_init();

	/* Canary: two words must come out and not be identically zero (a dead or
	 * mis-initialized RNG returns 0/stuck). This line is the acceptance test. */
	if ((rng_readWord(&s0) != 0) || (rng_readWord(&s1) != 0)) {
		printf("rpi4-hwrng: no entropy (FIFO empty after enable) -- not registering\n");
		return 1;
	}
	if ((s0 == 0u) && (s1 == 0u)) {
		printf("rpi4-hwrng: suspect entropy (both samples zero) -- not registering\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-hwrng: portCreate failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "hwrng") < 0) {
		printf("rpi4-hwrng: could not create /dev/hwrng\n");
		return 3;
	}

	printf("rpi4-hwrng: enabled (bcm2711-rng200 @ 0x%08x); sample=0x%08x 0x%08x; registered /dev/hwrng\n",
		RNG200_BASE, s0, s1);

	rng_thread((void *)(uintptr_t)port);

	return 0;
}
