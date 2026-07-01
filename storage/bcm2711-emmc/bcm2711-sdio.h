/*
 * Phoenix-RTOS
 *
 * BCM2711 (Raspberry Pi 4) EMMC2 SD Host Controller platform layer
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _BCM2711_SDIO_H_
#define _BCM2711_SDIO_H_

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

/* The Pi 4 SD card slot is wired to the BCM2711 EMMC2 (Arasan SDHCI) controller.
 * We expose a single host/slot. (The legacy EMMC @0xfe300000 + SDHOST @0xfe202000
 * are separate controllers used for WiFi etc. and are not handled here.) */
#define PLATFORM_SDIO_N_HOSTS 1

/* Structure with platform-specific information that the SD Host Controller may need.
 * Mirrors the zynq7000-sdcard contract so the generic SDHCI core (sdcard.c) is shared. */
typedef struct {
	/* Frequency of the reference clock feeding the SDHCI block (Hz). */
	uint32_t refclkFrequency;
	/* Physical address of the SD Host Controller register bank. */
	addr_t regBankPhys;
	/* Number of the interrupt that will be used by this controller. */
	int interruptNum;
	/* Does this slot have a physical Card Detect switch connected. */
	bool isCDPinSupported;
	/* Does this slot have a physical Write Protect switch connected. */
	bool isWPPinSupported;
} sdio_platformInfo_t;


static inline void sdio_dataBarrier(void)
{
	/* Full-system data memory barrier — orders the uncached DMA-buffer and
	 * MMIO accesses on the non-coherent A72 fabric (same hazard class as the
	 * Pi 4 USB/PCIe DMA work). */
	__asm__ volatile("dmb sy" ::: "memory");
}


int sdio_platformConfigure(unsigned int slot, sdio_platformInfo_t *infoOut);


/* Switch the Pi 4 SD I/O signaling rail via the VideoCore mailbox. The rail
 * (sd_io_1v8_reg in the Linux DT) is driven by firmware GPIO 132 (expgpio pin 4):
 * enable=true selects 1.8V (required for UHS-I DDR50/SDR50), false selects 3.3V
 * (default / High-Speed). Returns 0 on success, negative errno otherwise. The
 * caller must observe the regulator settling time (~5 ms) before relying on the
 * new level. There is no Linux-style regulator framework on Phoenix, so this
 * mailbox SET_GPIO_STATE call is the direct equivalent. */
int sdio_setSdIoVoltage18(bool enable);

#endif /* _BCM2711_SDIO_H_ */
