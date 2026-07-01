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

#include <libvcmbox.h>

#define VC_PROP_GET_CLOCK_RATE 0x00030002u
#define VC_PROP_SET_CLOCK_RATE 0x00038002u

#define VC_CLOCK_EMMC2 12u

/* VideoCore firmware GPIO expander (expgpio): pin 4 drives sd_io_1v8_reg, the SD
 * I/O signaling rail. Firmware GPIO numbering bases the expander at 128, so
 * expgpio 4 = 132. State 1 = 1.8V (UHS), state 0 = 3.3V (default/High-Speed). */
#define VC_PROP_SET_GPIO_STATE 0x00038041u
#define VC_GPIO_SD_IO_1V8      132u

/* Fallback if the firmware reports no rate (it normally has EMMC2 running,
 * since the board boots from the SD card). */
#define BCM2711_EMMC2_DEFAULT_HZ (100u * 1000u * 1000u)


/* One mailbox property transaction carrying a single tag, routed through the
 * serializing rpi4-vcmbox server (/dev/vcmbox) — the BCM2711 mailbox FIFO has
 * no hardware arbitration, so every user must go through the server rather than
 * drive the FIFO directly. payload[] holds the tag value buffer (in/out);
 * payloadWords is its u32 count. Returns 0 on an OK response, negative errno
 * otherwise.
 *
 * Pre-bind hazard: in the sd variant the SD driver runs BEFORE `bind devfs
 * /dev`, so a plain `/dev/vcmbox` path lookup would not resolve. libvcmbox
 * handles this internally — it falls back to resolving the node through the
 * `devfs` named port, and its lookup is bounded, so if the server is somehow
 * absent this returns an error (the caller then uses the default clock rate)
 * rather than hanging the sd-boot path. */
static int sdio_mboxProperty(uint32_t tag, uint32_t *payload, unsigned int payloadWords)
{
	if (payload == NULL) {
		return -EINVAL;
	}

	return vcmbox_call(tag, payloadWords * (uint32_t)sizeof(uint32_t),
		payload, payloadWords, payload, payloadWords);
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


int sdio_setSdIoVoltage18(bool enable)
{
	uint32_t payload[2];

	/* SET_GPIO_STATE: [gpio, state] -> [gpio, state]. Drives expgpio 4 (fw GPIO
	 * 132) which selects the SD I/O rail: 1 = 1.8V (UHS), 0 = 3.3V. */
	payload[0] = VC_GPIO_SD_IO_1V8;
	payload[1] = enable ? 1u : 0u;
	return sdio_mboxProperty(VC_PROP_SET_GPIO_STATE, payload, 2);
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
