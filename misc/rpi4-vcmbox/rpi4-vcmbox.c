/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) VideoCore property-mailbox server (/dev/vcmbox)
 *
 * The BCM2711 exposes a single VideoCore property mailbox: one hardware FIFO at
 * 0xfe00b880, channel 8, with NO hardware arbitration. It is the only path to
 * SoC temperature, the board MAC, throttle state, and to power/clock the V3D,
 * HVS, and USB blocks - so several independent Phoenix processes need it at boot
 * (thermal, genet, the usb daemon's VL805 bring-up, sdio, v3d power-on). When
 * two of them drive the FIFO concurrently, one read loop pops and discards the
 * other's response word, destroying it -> transient mailbox failures.
 *
 * This server owns the FIFO. Clients send property calls as mtDevCtl messages
 * (see libvcmbox); because a Phoenix server processes one message at a time,
 * every transaction is naturally serialized - no two callers ever drive the FIFO
 * at once. The server also:
 *   - mmaps the mailbox MMIO ONCE and reuses a SINGLE uncached-contiguous bounce
 *     buffer allocated early at startup (deterministic, low PA -> stays within
 *     the VideoCore-addressable range even on 4/8 GB boots);
 *   - retries internally on a failed transaction (the FIFO may have been
 *     momentarily disturbed by leftover pre-server traffic);
 *   - on ultimate failure logs a one-line diagnostic that distinguishes the
 *     three intermittency-consistent causes (see vcmbox_transact).
 *
 * The mailbox protocol + cache discipline (device-mapped uncached MMIO window,
 * uncached/contiguous/physically-pinned message buffer, va2pa, channel 8, the
 * spin caps, the RESP_OK check) are carried over verbatim from the validated
 * rpi4-thermal driver, which is now a client of this server.
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
#include <unistd.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <posix/utils.h>

#include "libvcmbox.h"


/* VideoCore property mailbox (BCM2711 ARM peripherals + plo video.c). */
#define RPI_MAILBOX_BASE     0xfe00b880u

#define VC_MBOX_READ         0x00u
#define VC_MBOX_STATUS       0x18u
#define VC_MBOX_WRITE        0x20u
#define VC_MBOX_STATUS_FULL  0x80000000u
#define VC_MBOX_STATUS_EMPTY 0x40000000u
#define VC_MBOX_RESP_OK      0x80000000u
#define VC_MBOX_PROP_CHANNEL 8u

/* Bounded mailbox-wait spin cap. The happy path completes in microseconds, far
 * under this; the cap only protects against a response that was raced away (now
 * that this server serializes the FIFO, that can only come from leftover
 * pre-server traffic) so we never hang forever. */
#define MBOX_SPINS  4000000u

/* Internal retry budget on a failed transaction. */
#define MBOX_RETRIES  8u
#define MBOX_RETRY_US 2000u

/* Property message header: [bufsize, REQUEST, tag, valBufSize, req=0, ...vals,
 * END(0)]. Header words + value words + END, all u32. */
#define MBOX_MSG_HDR_WORDS 5u /* bufsize, code, tag, valBufSize, reqresp */


/* Outcome of one FIFO round-trip, used to pick the failure diagnostic. */
typedef enum {
	MBOX_OK = 0,        /* matched our request, RESP_OK */
	MBOX_EMPTY,         /* FIFO never went non-empty -> firmware never replied (#2) */
	MBOX_WRFULL,        /* write FIFO stayed full -> request could not be posted */
	MBOX_RACED,         /* consumed a non-matching entry -> another client's resp (#1) */
	MBOX_BADRESP        /* matched, but firmware code != RESP_OK (#3) */
} mbox_outcome_t;


static struct {
	volatile uint32_t *mbox; /* mapped mailbox MMIO (offset-adjusted) */
	uint32_t *buf;           /* single reusable uncached-contiguous bounce buffer */
	uint32_t buf_pa;         /* its physical address (channel-packed below) */
	uint32_t lastRacedPa;    /* last non-matching FIFO word seen (diag) */
} vcmbox;


/*
 * One FIFO round-trip with the message already built in vcmbox.buf. Reports the
 * outcome so the caller can both retry and, on final failure, log the cause.
 * msg[1] (the firmware response code) is left in place for the BADRESP diag.
 */
static mbox_outcome_t vcmbox_fifoRoundtrip(void)
{
	volatile uint32_t *mbox = vcmbox.mbox;
	uint32_t request = (vcmbox.buf_pa & ~0xFu) | VC_MBOX_PROP_CHANNEL;
	uint32_t spins;
	int sawNonEmpty = 0;

	/* Wait for write space. */
	for (spins = MBOX_SPINS; (mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_FULL) != 0u; spins--) {
		if (spins == 0u) {
			return MBOX_WRFULL;
		}
	}
	/* Ensure the Normal-NC bounce-buffer stores (the message the caller built)
	 * complete before the Device doorbell store below: ARM permits reordering
	 * Normal-NC vs Device accesses to different addresses, so without this DSB the
	 * firmware could observe the doorbell before the message lands. */
	__asm__ volatile("dsb sy" ::: "memory");
	mbox[VC_MBOX_WRITE / 4] = request;

	/* Drain until our own response surfaces. Reading MBOX0 consumes the entry;
	 * only ours matches `request`. A non-matching RESP_OK entry is leftover
	 * traffic from a pre-server client (recorded for the race diagnostic). */
	for (spins = MBOX_SPINS; spins != 0u; spins--) {
		if ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) == 0u) {
			uint32_t word = mbox[VC_MBOX_READ / 4];
			sawNonEmpty = 1;
			if (word == request) {
				break;
			}
			vcmbox.lastRacedPa = word;
		}
	}
	if (spins == 0u) {
		return (sawNonEmpty != 0) ? MBOX_RACED : MBOX_EMPTY;
	}

	/* The doorbell response surfaced; DSB before reading the firmware's Normal-NC
	 * buffer writes so they are observed in order after the Device FIFO read. */
	__asm__ volatile("dsb sy" ::: "memory");
	if (vcmbox.buf[1] != VC_MBOX_RESP_OK) {
		return MBOX_BADRESP;
	}
	return MBOX_OK;
}


/*
 * Run a full property transaction: build the message in the bounce buffer, drive
 * the FIFO (with internal retry), and copy the firmware value words back. On
 * ultimate failure, log a one-line diagnostic distinguishing the three causes.
 * Returns 0 and fills resp on success, or sets resp->err = -EIO on failure.
 */
static void vcmbox_transact(const vcmbox_req_t *req, vcmbox_resp_t *resp)
{
	uint32_t valWords = req->valBufSize / 4u;
	uint32_t totalWords;
	uint32_t i;
	uint32_t attempt;
	mbox_outcome_t outcome = MBOX_EMPTY;

	resp->err = -EIO;
	resp->nOut = 0;

	if (valWords > VCMBOX_MAX_WORDS) {
		valWords = VCMBOX_MAX_WORDS;
	}

	/* [bufsize, REQUEST(0), tag, valBufSize, reqresp(0), val0.., END(0)]. */
	totalWords = MBOX_MSG_HDR_WORDS + valWords + 1u;

	for (attempt = 0; attempt < MBOX_RETRIES; attempt++) {
		memset(vcmbox.buf, 0, totalWords * 4u);
		vcmbox.buf[0] = totalWords * 4u;
		vcmbox.buf[1] = 0u; /* request code */
		vcmbox.buf[2] = req->tag;
		vcmbox.buf[3] = req->valBufSize;
		vcmbox.buf[4] = 0u; /* request indicator + response length */
		for (i = 0; (i < req->nIn) && (i < valWords); i++) {
			vcmbox.buf[MBOX_MSG_HDR_WORDS + i] = req->in[i];
		}
		/* END tag is already 0 from the memset. */

		vcmbox.lastRacedPa = 0u;
		outcome = vcmbox_fifoRoundtrip();
		if (outcome == MBOX_OK) {
			for (i = 0; (i < valWords) && (i < VCMBOX_MAX_WORDS); i++) {
				resp->out[i] = vcmbox.buf[MBOX_MSG_HDR_WORDS + i];
			}
			resp->nOut = valWords;
			resp->err = 0;
			return;
		}

		usleep(MBOX_RETRY_US);
	}

	/* All retries exhausted - report the discriminating cause once. */
	switch (outcome) {
		case MBOX_EMPTY:
			printf("rpi4-vcmbox: tag 0x%08x FAILED - FIFO stayed empty (firmware never replied; "
				"buf_pa=0x%08x may be above VC-addressable range)\n",
				req->tag, vcmbox.buf_pa);
			break;
		case MBOX_WRFULL:
			printf("rpi4-vcmbox: tag 0x%08x FAILED - write FIFO stayed full (request not posted)\n",
				req->tag);
			break;
		case MBOX_RACED:
			printf("rpi4-vcmbox: tag 0x%08x FAILED - consumed a non-matching FIFO entry "
				"(cross-process race; other buf_pa+chan=0x%08x)\n",
				req->tag, vcmbox.lastRacedPa);
			break;
		case MBOX_BADRESP:
			printf("rpi4-vcmbox: tag 0x%08x FAILED - firmware returned code 0x%08x (not RESP_OK)\n",
				req->tag, vcmbox.buf[1]);
			break;
		default:
			printf("rpi4-vcmbox: tag 0x%08x FAILED - unknown outcome\n", req->tag);
			break;
	}
}


static void vcmbox_handleMsg(msg_t *msg)
{
	const vcmbox_req_t *req = (const vcmbox_req_t *)msg->i.raw;
	vcmbox_resp_t *resp = (vcmbox_resp_t *)msg->o.raw;

	if (req->nIn > VCMBOX_MAX_WORDS) {
		resp->err = -EINVAL;
		resp->nOut = 0;
		return;
	}

	vcmbox_transact(req, resp);
}


static void vcmbox_thread(void *arg)
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

			case mtRead:
			case mtWrite:
				/* /dev/vcmbox is a control node, driven via mtDevCtl only. */
				msg.o.err = -EINVAL;
				break;

			case mtDevCtl:
				vcmbox_handleMsg(&msg);
				/* o.err is the IPC-layer status; the property-call status lives
				 * in the response struct in o.raw. EOK = the msg was handled. */
				msg.o.err = EOK;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


/* Map the mailbox MMIO once and allocate the single reusable bounce buffer.
 * Returns 0 on success. */
static int vcmbox_init(void)
{
	addr_t pa_base = (addr_t)RPI_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	void *mbox_page;
	void *buf_page;
	uintptr_t buf_pa;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, pa_base);
	if (mbox_page == MAP_FAILED) {
		printf("rpi4-vcmbox: mmap of mailbox MMIO failed\n");
		return -1;
	}
	vcmbox.mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	/* One reusable uncached-contiguous bounce buffer, allocated early so it
	 * lands low / VideoCore-addressable. Uncached => no clean/invalidate needed
	 * around the firmware round-trip. */
	buf_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (buf_page == MAP_FAILED) {
		printf("rpi4-vcmbox: mmap of bounce buffer failed\n");
		munmap(mbox_page, _PAGE_SIZE);
		return -1;
	}
	vcmbox.buf = buf_page;

	buf_pa = (uintptr_t)va2pa(vcmbox.buf);
	/* The VideoCore mailbox request is a 32-bit bus address. Reject a bounce buffer
	 * that va2pa couldn't resolve (-1) OR that landed above 4 GiB (possible for a
	 * MAP_CONTIGUOUS page on a 4/8 GB Pi 4): truncating it to uint32_t would silently
	 * point the firmware at the wrong physical page while the transaction still
	 * "matches" (same PA key) and reports success. Fail init loudly instead. */
	if ((buf_pa == (uintptr_t)-1) || ((uint64_t)buf_pa > 0xffffffffULL)) {
		printf("rpi4-vcmbox: bounce-buffer PA 0x%llx not VC-addressable (need < 4 GiB)\n",
			(unsigned long long)buf_pa);
		munmap(buf_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return -1;
	}
	vcmbox.buf_pa = (uint32_t)buf_pa;

	return 0;
}


int main(int argc, char **argv)
{
	uint32_t port;
	oid_t dev;

	(void)argc;
	(void)argv;

	if (vcmbox_init() != 0) {
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-vcmbox: portCreate failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "vcmbox") < 0) {
		printf("rpi4-vcmbox: could not create /dev/vcmbox\n");
		return 3;
	}

	/* The startup banner logs the bounce-buffer PA: a high address on a 4/8 GB
	 * boot is the visible signature of failure mode #2 (firmware never replies). */
	printf("rpi4-vcmbox: mailbox @ 0x%08x, bounce buf_pa=0x%08x; registered /dev/vcmbox (serialized)\n",
		RPI_MAILBOX_BASE, vcmbox.buf_pa);

	vcmbox_thread((void *)(uintptr_t)port);

	return 0;
}
