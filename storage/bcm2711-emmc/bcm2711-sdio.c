/*
 * Phoenix-RTOS
 *
 * BCM2711 (Raspberry Pi 4) EMMC2 SD Host Controller platform layer
 *
 * Provides sdio_platformConfigure() for the generic SDHCI core (sdcard.c):
 * the EMMC2 register-bank address + IRQ are fixed by the SoC, and the
 * reference clock is obtained from the VideoCore firmware over the mailbox
 * property channel (the Pi 4 has no kernel device-clock platformctl, unlike
 * Zynq). The EMMC2 controller is hard-wired to the SD-card slot on the Pi 4,
 * so no GPIO pin-mux is required here.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "bcm2711-sdio.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>


/* BCM2711 EMMC2 (Arasan SDHCI) — SD-card slot. Physical (ARM) addresses;
 * the VideoCore bus addresses are 0x7eXXXXXX. From the Pi 4 device tree
 * (/emmc2bus/mmc@7e340000, compatible "brcm,bcm2711-emmc2"). */
#define BCM2711_EMMC2_BASE 0xfe340000u
#define BCM2711_EMMC2_SIZE 0x100u
/* interrupts = <GIC_SPI 126 IRQ_TYPE_LEVEL_HIGH>; GIC SPIs start at 32. */
#define BCM2711_EMMC2_IRQ  (32 + 126)

/* VideoCore mailbox (Pi 4). Mirrors the proven sequence in
 * phoenix-rtos-lwip/port/diag-udp.c. */
#define RPI_PI4_MAILBOX_BASE  0xfe00b880u
#define VC_MBOX_READ          0x00u
#define VC_MBOX_STATUS        0x18u
#define VC_MBOX_WRITE         0x20u
#define VC_MBOX_STATUS_FULL   0x80000000u
#define VC_MBOX_STATUS_EMPTY  0x40000000u
#define VC_MBOX_RESP_OK       0x80000000u
#define VC_MBOX_PROP_CHANNEL  8u

#define VC_PROP_GET_CLOCK_RATE 0x00030002u
#define VC_PROP_SET_CLOCK_RATE 0x00038002u

#define VC_CLOCK_EMMC2 12u

/* Fallback if the firmware reports no rate (it normally has EMMC2 running,
 * since the board boots from the SD card). */
#define BCM2711_EMMC2_DEFAULT_HZ (100u * 1000u * 1000u)


/* One mailbox property transaction carrying a single tag. payload[] holds the
 * tag value buffer (in/out); payloadWords is its u32 count. Returns 0 on an
 * OK response, -EIO otherwise. */
static int sdio_mboxProperty(uint32_t tag, uint32_t *payload, unsigned int payloadWords)
{
	addr_t pa_base = (addr_t)RPI_PI4_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_PI4_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request;
	unsigned int i;
	int ret = -EIO;
	void *mbox_page;
	void *msg_page;
	/* header(5) + payload + end-tag(1), rounded to 16 bytes. */
	unsigned int hdrWords = 5;
	unsigned int totalWords = hdrWords + payloadWords + 1;

	if (((totalWords * sizeof(uint32_t)) > _PAGE_SIZE) || (payload == NULL)) {
		return -EINVAL;
	}

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, pa_base);
	if (mbox_page == MAP_FAILED) {
		return -ENOMEM;
	}
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return -ENOMEM;
	}
	msg = msg_page;

	msg[0] = totalWords * sizeof(uint32_t); /* total size in bytes */
	msg[1] = 0;                             /* request */
	msg[2] = tag;
	msg[3] = payloadWords * sizeof(uint32_t); /* value buffer size */
	msg[4] = 0;                             /* tag request code */
	for (i = 0; i < payloadWords; i++) {
		msg[hdrWords + i] = payload[i];
	}
	msg[hdrWords + payloadWords] = 0; /* end tag */

	msg_pa = (uintptr_t)va2pa(msg);
	if (msg_pa == (uintptr_t)-1) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return -EIO;
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
		for (i = 0; i < payloadWords; i++) {
			payload[i] = msg[hdrWords + i];
		}
		ret = 0;
	}

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return ret;
}


static uint32_t sdio_emmc2ClockHz(void)
{
	uint32_t payload[2];

	/* GET_CLOCK_RATE: [clock_id] -> [clock_id, rate_hz]. */
	payload[0] = VC_CLOCK_EMMC2;
	payload[1] = 0;
	if ((sdio_mboxProperty(VC_PROP_GET_CLOCK_RATE, payload, 2) == 0) && (payload[1] != 0u)) {
		return payload[1];
	}

	/* Firmware reports nothing usable — ask it to set a sane rate, then re-read. */
	payload[0] = VC_CLOCK_EMMC2;
	payload[1] = BCM2711_EMMC2_DEFAULT_HZ;
	(void)sdio_mboxProperty(VC_PROP_SET_CLOCK_RATE, payload, 2);

	payload[0] = VC_CLOCK_EMMC2;
	payload[1] = 0;
	if ((sdio_mboxProperty(VC_PROP_GET_CLOCK_RATE, payload, 2) == 0) && (payload[1] != 0u)) {
		return payload[1];
	}

	return BCM2711_EMMC2_DEFAULT_HZ;
}


int sdio_platformConfigure(unsigned int slot, sdio_platformInfo_t *infoOut)
{
	uint32_t refclk;

	if (slot >= PLATFORM_SDIO_N_HOSTS) {
		return -ENOENT;
	}

	/* EMMC2 is hard-wired to the SD-card slot on the Pi 4 (no GPIO mux), and
	 * the SD power domain is already up because the board boots from the card.
	 * So the only platform step is to learn the reference clock. */
	refclk = sdio_emmc2ClockHz();

	infoOut->refclkFrequency = refclk;
	infoOut->regBankPhys = BCM2711_EMMC2_BASE;
	infoOut->interruptNum = BCM2711_EMMC2_IRQ;
	/* The Pi 4 SD slot has no software-readable WP switch; card detect is
	 * reported by the SDHCI Present-State register, not a separate GPIO. */
	infoOut->isCDPinSupported = false;
	infoOut->isWPPinSupported = false;

	printf("bcm2711-emmc: EMMC2 @0x%08x irq=%d refclk=%u Hz\n",
		(unsigned)BCM2711_EMMC2_BASE, BCM2711_EMMC2_IRQ, (unsigned)refclk);

	return 0;
}
