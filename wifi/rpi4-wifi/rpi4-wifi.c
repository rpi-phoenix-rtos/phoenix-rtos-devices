/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM43455 SDIO) WiFi device
 *
 * Brings up the BCM43455 WiFi controller on the Pi 4's Arasan SDHCI /
 * SDIO bus and exposes it as /dev/wifi, a text-oriented scan interface:
 *   write("scan") - trigger an active escan of the air
 *   read()        - the discovered access points as text, one per line:
 *                     SSID  BSSID(xx:xx:..)  RSSI(dBm)  ch<N>
 *
 * At startup it runs the full firmware bring-up ONCE (power-cycle WL_ON
 * via the VideoCore mailbox, SDIO enumeration, 643 KB firmware download
 * into the CR4 TCM, NVRAM + CLM regulatory blob, ARM-CR4 reset release,
 * SDPCM function-2 enable), then serves /dev/wifi.
 *
 * PROVENANCE
 * ----------
 * The SDIO/SDHCI/GPIO/mailbox helpers (`diag_*`) and the firmware-release
 * sequence were lifted VERBATIM from the proven WiFi bring-up probe
 * (tools/wifi-probe/wifi-probe.c) — the code path that first drove the
 * 43455 entirely from Phoenix and scanned 16 real APs. That probe ran the
 * whole sequence once from main() as `diag_format_sdio_fwrelease`; this
 * driver splits that single orchestrator into `wifi_bringup()` (power-on
 * through SDPCM-F2 enable, run once at startup) and `wifi_scan()` (the
 * escan, run per client "scan" request), keeping the exact ordering and
 * timing the probe established. The bring-up telemetry the probe printed
 * is preserved as startup logging.
 *
 * This productionizes tools/wifi-probe into a resident driver, mirroring
 * the sibling rpi4-hci Bluetooth device (T-WIFI-BT: make WiFi a
 * first-class Phoenix citizen). See
 * docs/inprogress/2026-08-10-wifi-bt-first-class-design.md.
 *
 * MMIO / GPIO TOUCHED (all via userspace mmap of physical pages, the
 * same MAP_PHYSMEM|MAP_DEVICE|MAP_UNCACHED pattern the ported
 * thermal/hwrng/vcmbox drivers use):
 *   - SDHCI (Arasan) @ 0xfe300000     — the controller the 43455 sits on
 *   - BCM2711 GPIO   @ 0xfe200000     — routes GPIO 34..39 to ALT3 (SDIO)
 *   - VideoCore mbox @ 0xfe00b880     — SET_GPIO_STATE(WL_ON) power cycle
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include "wifi-fw-43455.h"
#include "wifi-nvram-43455.h"
#include "clm-43455.h"

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/* BCM2711 GPIO block (function-select for the SDIO alt-function). */

#define BCM2711_GPIO_BASE   0xfe200000u
#define GPIO_GPFSEL0        0x00u   /* +4*n for GPFSEL1..5 */

/* Set pin function-select (3 bits). pin: 0..53, fn: 0..7. Read-
 * modify-write of GPFSEL(pin/10). Routes GPIO 34..39 to ALT3 for SDIO. */
static void diag_gpioSetFsel(volatile uint8_t *base, unsigned pin, unsigned fn)
{
	unsigned bank = pin / 10u;
	unsigned shift = (pin % 10u) * 3u;
	volatile uint32_t *reg = (volatile uint32_t *)(base + GPIO_GPFSEL0 + bank * 4u);
	uint32_t v = *reg;
	v &= ~(0x7u << shift);
	v |= ((fn & 0x7u) << shift);
	*reg = v;
}

/* ------------------------------------------------------------------ */
/* VideoCore mailbox (property channel). Used only for the WL_ON expander
 * GPIO power cycle. Pi 4 mailbox base hardcoded (the port has no
 * board_config.h include path). */

#define RPI_PI4_MAILBOX_BASE  0xfe00b880u

#define VC_MBOX_READ          0x00u
#define VC_MBOX_STATUS        0x18u
#define VC_MBOX_WRITE         0x20u
#define VC_MBOX_STATUS_FULL   0x80000000u
#define VC_MBOX_STATUS_EMPTY  0x40000000u
#define VC_MBOX_RESP_OK       0x80000000u
#define VC_MBOX_PROP_CHANNEL  8u

#define VC_PROP_SET_GPIO_STATE  0x00038041u

#define EXPGPIO_WL_ON           129u  /* expgpio[1] = "WL_ON" per Pi 4 DT */

/* Get / set VideoCore device power state (here: an expander GPIO via
 * SET_GPIO_STATE). Returns the resulting state on success, 0xFFFFFFFF on
 * failure. */
static uint32_t diag_mboxPower(uint32_t tag, uint32_t device_id, uint32_t state)
{
	addr_t pa_base = (addr_t)RPI_PI4_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_PI4_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request;
	uint32_t result = 0xFFFFFFFFu;
	uint32_t deadline;
	void *mbox_page;
	void *msg_page;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, pa_base);
	if (mbox_page == MAP_FAILED) {
		return 0xFFFFFFFFu;
	}
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}
	msg = msg_page;

	/* GET takes (device_id) and returns (device_id, state).
	 * SET takes (device_id, state) and returns (device_id, state). */
	msg[0] = 32;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = 8;
	msg[4] = 0;
	msg[5] = device_id;
	msg[6] = state;
	msg[7] = 0;

	msg_pa = (uintptr_t)va2pa(msg);
	/* The mailbox request carries a 32-bit bus address; reject an unmapped page
	 * (-1) or one whose PA doesn't fit in 32 bits (a >4 GiB contiguous alloc on a
	 * large Pi 4) rather than silently truncating it and pointing VideoCore at the
	 * wrong physical page. */
	if ((msg_pa == (uintptr_t)-1) || ((uint64_t)msg_pa > 0xffffffffULL)) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}
	request = ((uint32_t)msg_pa & ~0xFu) | VC_MBOX_PROP_CHANNEL;

	/* Bound every mailbox spin (cf. the SDHCI helpers' 100000-iteration caps): a
	 * wedged/unresponsive VideoCore must fail bring-up, not hang the WiFi thread
	 * forever (which would leave /dev/wifi unregistered and the parent blocked). */
	for (deadline = 100000u; ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_FULL) != 0u) && (deadline > 0u); --deadline) {
	}
	if (deadline == 0u) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}
	mbox[VC_MBOX_WRITE / 4] = request;

	for (deadline = 100000u; deadline > 0u; --deadline) {
		if ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) != 0u) {
			continue; /* no response yet */
		}
		if (mbox[VC_MBOX_READ / 4] == request) {
			break; /* our response */
		}
		/* else: a response for a different request — drain and keep waiting */
	}
	if (deadline == 0u) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}

	if (msg[1] == VC_MBOX_RESP_OK) {
		result = msg[6];  /* returned state */
	}

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}

/* Cold-power-cycle the BCM43455 WiFi chip via its WL_REG_ON line (a Pi 4
 * expander GPIO driven through the VideoCore mailbox): drop it, wait,
 * re-assert, settle. NB: a 20x-longer power-down was tested and did NOT
 * make the 43455 firmware execute (the fw-exec gate is not a reset-timing
 * issue); 50/150 ms is the established, enumeration-tested baseline. */
static void diag_wifiPowerCycle(void)
{
	(void)diag_mboxPower(VC_PROP_SET_GPIO_STATE, EXPGPIO_WL_ON, 0u);
	usleep(50 * 1000);
	(void)diag_mboxPower(VC_PROP_SET_GPIO_STATE, EXPGPIO_WL_ON, 1u);
	usleep(150 * 1000);
}

/* ------------------------------------------------------------------ */
/* SDHCI 3.0 controller (Arasan @ 0xfe300000). Register offsets and
 * command/response encodings per the SD Host Controller Simplified
 * Specification 3.0. */

#define SDHCI_ARGUMENT_1   0x08u
#define SDHCI_TRANS_CMD    0x0Cu
#define SDHCI_RESPONSE_0   0x10u
#define SDHCI_PRES_STATE   0x24u
#define SDHCI_INT_STATUS   0x30u

#define SDHCI_PRES_CMD_INHIBIT  0x00000001u
#define SDHCI_INT_CMD_COMPLETE  0x00000001u
#define SDHCI_INT_ERR_ANY       0x00008000u  /* ERR_INT bits live in the upper 16 */

/* SOFT_RESET_* live in bits 24..26 of the 32-bit dword at offset 0x2C
 * (CLOCK_CTL + TIMEOUT_CTL + SOFT_RESET). Write 1 to start the reset;
 * the bit clears when done. */
#define SDHCI_CLK_TIMEOUT_RESET 0x2Cu
#define SDHCI_SOFT_RESET_ALL    (1u << 24)
#define SDHCI_SOFT_RESET_CMD    (1u << 25)
#define SDHCI_SOFT_RESET_DAT    (1u << 26)

/* Command-register RESPONSE_TYPE + check-bit encodings (bits 0..5 of the
 * COMMAND half of the TRANS_CMD dword):
 *   R0  (no resp)  = 0x00
 *   R1             = 0x1a  (resp=2, CRC, index)
 *   R1b            = 0x1b
 *   R3  (CMD41)    = 0x02  (resp=2, no CRC, no index)
 *   R4  (CMD5)     = 0x02
 *   R5  (CMD52,53) = 0x1a
 *   R6  (CMD3)     = 0x1a */
#define SDHCI_RESP_R0   0x00u
#define SDHCI_RESP_R1   0x1au
#define SDHCI_RESP_R1b  0x1bu
#define SDHCI_RESP_R3   0x02u
#define SDHCI_RESP_R4   0x02u
#define SDHCI_RESP_R5   0x1au
#define SDHCI_RESP_R6   0x1au

#define SDHCI_BLOCK_SIZE_CNT  0x04u  /* BLOCK_SIZE (low 16) + BLOCK_COUNT (high 16) */
#define SDHCI_DATA_PORT       0x20u  /* PIO FIFO */
#define SDHCI_INT_XFER_COMPLETE  0x00000002u
#define SDHCI_INT_BUF_RD_READY   0x00000020u
#define SDHCI_INT_BUF_WR_READY   0x00000010u

/* Program SDHCI to a target SD-bus clock by dividing the 250 MHz base.
 * Per SDHCI 3.0 §2.2.13: divisor is 10-bit, output_hz = base / (2*N). */
static int diag_sdhciSetClockKHz(volatile uint8_t *base, unsigned target_khz)
{
	uint32_t base_hz = 250000000u;
	uint32_t target_hz = (uint32_t)target_khz * 1000u;
	uint32_t divisor;
	uint32_t clkctl;
	uint32_t i;

	if (target_hz == 0u || target_hz > base_hz) {
		return -1;
	}
	divisor = (base_hz + (2u * target_hz) - 1u) / (2u * target_hz);
	if (divisor > 0x3FFu) {
		divisor = 0x3FFu;
	}

	/* Disable SD clock first. RMW the low 16 (CLOCK_CTL) only. */
	clkctl = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET);
	clkctl &= 0xFFFF0000u;
	*(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET) = clkctl;

	/* Build new CLOCK_CTL: INTERNAL_CLOCK_EN=1, SD_CLOCK_EN=0 for now,
	 * divisor high bits [9:8] at [7:6], low bits [7:0] at [15:8]. */
	{
		uint16_t cctl = (uint16_t)(
			(uint16_t)(divisor & 0xFFu) << 8 |
			(uint16_t)((divisor >> 8) & 0x3u) << 6 |
			(1u << 0));
		uint32_t hi = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET) &
			0xFFFF0000u;
		*(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET) =
			hi | (uint32_t)cctl;
	}

	/* Wait for INTERNAL_CLOCK_STABLE (bit 1). */
	for (i = 0; i < 100000u; ++i) {
		uint32_t v = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET);
		if ((v & (1u << 1)) != 0u) {
			break;
		}
	}
	if (i == 100000u) {
		return -2;
	}

	/* Enable SD_CLOCK (bit 2). */
	{
		uint32_t v = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET);
		v |= (1u << 2);
		*(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET) = v;
	}

	return 0;
}

/* Soft-reset the CMD and DAT lines without disturbing CLOCK_CTL /
 * TIMEOUT_CTL (which firmware has already set up). 32-bit RMW. */
static int diag_sdhciResetCmdDat(volatile uint8_t *base)
{
	uint32_t orig = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET);
	uint32_t deadline = 100000u;
	uint32_t i;

	*(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET) =
		(orig & 0x00FFFFFFu) | SDHCI_SOFT_RESET_CMD | SDHCI_SOFT_RESET_DAT;

	for (i = 0; i < deadline; ++i) {
		uint32_t v = *(volatile uint32_t *)(base + SDHCI_CLK_TIMEOUT_RESET);
		if ((v & (SDHCI_SOFT_RESET_CMD | SDHCI_SOFT_RESET_DAT)) == 0u) {
			return 0;
		}
	}
	return -1;
}

/* Issue an SDHCI command. Returns 0 on success, negative on error. On
 * success, response_out[0..3] is filled from RESPONSE_0..3 (caller must
 * allocate a 4-element array). */
static int diag_sdhciCmd(volatile uint8_t *base, uint8_t cmd_index,
	uint32_t arg, uint16_t resp_type, uint32_t response_out[4])
{
	uint32_t deadline = 100000u;
	uint32_t i;

	/* Clear stale INT_STATUS bits (W1C). */
	*(volatile uint32_t *)(base + SDHCI_INT_STATUS) = 0xFFFFFFFFu;

	/* Wait for CMD_INHIBIT clear. */
	for (i = 0; i < deadline; ++i) {
		if ((*(volatile uint32_t *)(base + SDHCI_PRES_STATE) &
				SDHCI_PRES_CMD_INHIBIT) == 0u) {
			break;
		}
	}
	if (i == deadline) {
		return -1;  /* CMD_INHIBIT stuck */
	}

	/* Program ARGUMENT then COMMAND. 32-bit write to TRANS_CMD (offset
	 * 0x0C): low 16 = TRANSFER_MODE = 0 (no data), high 16 = COMMAND.
	 * The Arasan controller requires the combined 32-bit write. COMMAND
	 * layout in the upper dword: CMD_NUMBER at 31:24, RESPONSE_TYPE +
	 * check bits at 21:16. */
	*(volatile uint32_t *)(base + SDHCI_ARGUMENT_1) = arg;
	{
		uint32_t cmd_word =
			((uint32_t)resp_type << 16) |
			((uint32_t)cmd_index << 24);
		*(volatile uint32_t *)(base + SDHCI_TRANS_CMD) = cmd_word;
	}

	/* Wait for CMD_COMPLETE (or any error bit). */
	for (i = 0; i < deadline; ++i) {
		uint32_t st = *(volatile uint32_t *)(base + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -2;  /* error reported */
		}
		if ((st & SDHCI_INT_CMD_COMPLETE) != 0u) {
			break;
		}
	}
	if (i == deadline) {
		return -3;  /* cmd_complete didn't assert */
	}

	if (response_out != NULL) {
		response_out[0] = *(volatile uint32_t *)(base + SDHCI_RESPONSE_0 + 0x0);
		response_out[1] = *(volatile uint32_t *)(base + SDHCI_RESPONSE_0 + 0x4);
		response_out[2] = *(volatile uint32_t *)(base + SDHCI_RESPONSE_0 + 0x8);
		response_out[3] = *(volatile uint32_t *)(base + SDHCI_RESPONSE_0 + 0xC);
	}

	/* W1C the CMD_COMPLETE bit. */
	*(volatile uint32_t *)(base + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	return 0;
}

/* CMD52 (IO_RW_DIRECT). arg layout: bit31 R/W, bits30:28 FN, bits25:9
 * 17-bit REG, bits7:0 DATA. resp_out must be a 4-element uint32_t array
 * (diag_sdhciCmd unconditionally dumps all four response slots). */
static int diag_sdioCmd52(volatile uint8_t *sdhci, int write, int fn,
	uint32_t reg, uint8_t data, uint32_t *resp_out)
{
	uint32_t arg = 0;

	arg |= (write ? 1u : 0u) << 31;
	arg |= ((uint32_t)fn & 7u) << 28;
	arg |= ((uint32_t)reg & 0x1ffffu) << 9;
	if (write) {
		arg |= (uint32_t)data;
	}
	return diag_sdhciCmd(sdhci, 52u, arg, SDHCI_RESP_R5, resp_out);
}

/* Switch SDIO to High-Speed (25 MHz) on a 4-bit data bus. Call after
 * CMD0/5/3/7 + F1 enable + IORDY. Sequence per BCM43455c0 / SDIO 2.0:
 * CCCR 0x13 SHS check + EHS set, CCCR 0x07 4-bit width, SDHCI HCTL1
 * 4BIT+HIGH_SPEED, reprogram clock to 25 MHz. */
static int diag_sdioGoHighSpeed(volatile uint8_t *sdhci)
{
	uint32_t hs_resp[4] = {0};
	uint32_t bic_resp[4] = {0};
	int rc;

	rc = diag_sdioCmd52(sdhci, 0, 0, 0x13u, 0u, hs_resp);
	if (rc != 0) {
		return -1;
	}
	if ((hs_resp[0] & 0x01u) == 0u) {
		return -2;  /* SHS not set */
	}

	rc = diag_sdioCmd52(sdhci, 1, 0, 0x13u,
		(uint8_t)((hs_resp[0] | 0x02u) & 0xffu), NULL);
	if (rc != 0) {
		return -3;
	}

	rc = diag_sdioCmd52(sdhci, 0, 0, 0x07u, 0u, bic_resp);
	if (rc != 0) {
		return -4;
	}
	rc = diag_sdioCmd52(sdhci, 1, 0, 0x07u,
		(uint8_t)((bic_resp[0] & 0xFCu) | 0x02u), NULL);
	if (rc != 0) {
		return -5;
	}

	{
		uint32_t hctl = *(volatile uint32_t *)(sdhci + 0x28u);
		hctl &= 0xFFFFFF00u;
		hctl |= (1u << 1) | (1u << 2);
		*(volatile uint32_t *)(sdhci + 0x28u) = hctl;
	}

	rc = diag_sdhciSetClockKHz(sdhci, 25000u);
	if (rc != 0) {
		return -6;
	}
	return 0;
}

/* CMD53 (IO_RW_EXTENDED) block-mode READ via SDHCI PIO. buf must point
 * to a 4-byte-aligned destination of at least block_count*block_size
 * bytes. */
static int diag_sdioCmd53Read(volatile uint8_t *sdhci, int fn,
	int incr_addr, uint32_t reg_addr,
	uint32_t block_count, uint32_t block_size,
	uint8_t *buf)
{
	uint32_t arg, cmd_word;
	uint32_t st;
	uint32_t bytes_total = block_count * block_size;
	uint32_t words_total = bytes_total / 4u;
	uint32_t bytes_in_block = 0;
	uint32_t i;
	int deadline;

	/* Wait for CMD line idle. */
	for (deadline = 100000; deadline > 0; --deadline) {
		if ((*(volatile uint32_t *)(sdhci + SDHCI_PRES_STATE) &
			SDHCI_PRES_CMD_INHIBIT) == 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -1;
	}

	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;

	*(volatile uint32_t *)(sdhci + SDHCI_BLOCK_SIZE_CNT) =
		(block_count << 16) | (block_size & 0xFFFu);

	arg = (0u << 31) |
		((uint32_t)(fn & 7u) << 28) |
		(1u << 27) |  /* block_mode */
		((incr_addr ? 1u : 0u) << 26) |
		((reg_addr & 0x1FFFFu) << 9) |
		(block_count & 0x1FFu);
	*(volatile uint32_t *)(sdhci + SDHCI_ARGUMENT_1) = arg;

	/* TRANSFER_MODE + COMMAND dword at 0x0C: BLOCK_COUNT_EN, DAT_XFER_DIR
	 * = read, MULTI_BLK if >1, R5 resp + CRC/index, DATA_PRESENT, CMD53. */
	cmd_word =
		(1u << 1) |
		(1u << 4) |
		((block_count > 1u ? 1u : 0u) << 5) |
		((uint32_t)0x3Au << 16) |
		((uint32_t)53u << 24);
	*(volatile uint32_t *)(sdhci + SDHCI_TRANS_CMD) = cmd_word;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -2;
		}
		if ((st & SDHCI_INT_CMD_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	/* PIO read loop: drain DATA_PORT one word at a time; clear
	 * BUFFER_READ_READY after each block-worth. */
	for (i = 0; i < words_total; ++i) {
		for (deadline = 100000; deadline > 0; --deadline) {
			st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
			if ((st & SDHCI_INT_ERR_ANY) != 0u) {
				return -4;
			}
			if ((st & SDHCI_INT_BUF_RD_READY) != 0u) {
				break;
			}
		}
		if (deadline == 0) {
			return -5;
		}

		{
			uint32_t data = *(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT);
			if (buf != NULL) {
				buf[i * 4 + 0] = (uint8_t)(data & 0xffu);
				buf[i * 4 + 1] = (uint8_t)((data >> 8) & 0xffu);
				buf[i * 4 + 2] = (uint8_t)((data >> 16) & 0xffu);
				buf[i * 4 + 3] = (uint8_t)((data >> 24) & 0xffu);
			}
		}

		bytes_in_block += 4u;
		if (bytes_in_block >= block_size) {
			bytes_in_block = 0u;
			*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_RD_READY;
		}
	}

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -6;
		}
		if ((st & SDHCI_INT_XFER_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -7;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	return 0;
}

/* CMD53 (IO_RW_EXTENDED) BYTE-mode transfers: a single transaction of `nbytes`
 * (<=512), no SDIO block-count. brcmf/MMC use byte mode for sub-block control
 * frames (a block-mode CMD53 whose size mismatches the function's configured
 * block size stalls the data phase). Differs from the block helpers only in:
 * arg bit27(block_mode)=0 + byte count in arg[8:0]; TRANSFER_MODE has no
 * BLOCK_COUNT_EN / MULTI_BLK. nbytes is rounded up to 4 for the PIO word loop. */
static int diag_sdioCmd53ReadByteMode(volatile uint8_t *sdhci, int fn,
	int incr_addr, uint32_t reg_addr, uint32_t nbytes, uint8_t *buf)
{
	uint32_t arg, cmd_word, st, data;
	uint32_t words_total = (nbytes + 3u) / 4u;
	uint32_t i;
	int deadline;

	for (deadline = 100000; deadline > 0; --deadline) {
		if ((*(volatile uint32_t *)(sdhci + SDHCI_PRES_STATE) & SDHCI_PRES_CMD_INHIBIT) == 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -1;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	*(volatile uint32_t *)(sdhci + SDHCI_BLOCK_SIZE_CNT) = (1u << 16) | (nbytes & 0xFFFu);
	arg = (0u << 31) | ((uint32_t)(fn & 7u) << 28) | /* block_mode bit27 = 0 */
		((incr_addr ? 1u : 0u) << 26) |
		((reg_addr & 0x1FFFFu) << 9) | (nbytes & 0x1FFu);
	*(volatile uint32_t *)(sdhci + SDHCI_ARGUMENT_1) = arg;
	cmd_word = (1u << 4) | ((uint32_t)0x3Au << 16) | ((uint32_t)53u << 24); /* read dir, no BLK_CNT_EN */
	*(volatile uint32_t *)(sdhci + SDHCI_TRANS_CMD) = cmd_word;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -2;
		}
		if ((st & SDHCI_INT_CMD_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -4;
		}
		if ((st & SDHCI_INT_BUF_RD_READY) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -5;
	}
	for (i = 0; i < words_total; ++i) {
		data = *(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT);
		if (buf != NULL) {
			buf[i * 4 + 0] = (uint8_t)(data & 0xffu);
			buf[i * 4 + 1] = (uint8_t)((data >> 8) & 0xffu);
			buf[i * 4 + 2] = (uint8_t)((data >> 16) & 0xffu);
			buf[i * 4 + 3] = (uint8_t)((data >> 24) & 0xffu);
		}
	}
	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -6;
		}
		if ((st & SDHCI_INT_XFER_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -7;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	return 0;
}

static int diag_sdioCmd53WriteByteMode(volatile uint8_t *sdhci, int fn,
	int incr_addr, uint32_t reg_addr, uint32_t nbytes, const uint8_t *buf)
{
	uint32_t arg, cmd_word, st, data;
	uint32_t words_total = (nbytes + 3u) / 4u;
	uint32_t i;
	int deadline;

	for (deadline = 100000; deadline > 0; --deadline) {
		if ((*(volatile uint32_t *)(sdhci + SDHCI_PRES_STATE) & SDHCI_PRES_CMD_INHIBIT) == 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -1;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	*(volatile uint32_t *)(sdhci + SDHCI_BLOCK_SIZE_CNT) = (1u << 16) | (nbytes & 0xFFFu);
	arg = (1u << 31) | ((uint32_t)(fn & 7u) << 28) | /* write; block_mode bit27 = 0 */
		((incr_addr ? 1u : 0u) << 26) |
		((reg_addr & 0x1FFFFu) << 9) | (nbytes & 0x1FFu);
	*(volatile uint32_t *)(sdhci + SDHCI_ARGUMENT_1) = arg;
	cmd_word = ((uint32_t)0x3Au << 16) | ((uint32_t)53u << 24); /* write dir, no BLK_CNT_EN */
	*(volatile uint32_t *)(sdhci + SDHCI_TRANS_CMD) = cmd_word;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -2;
		}
		if ((st & SDHCI_INT_CMD_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -4;
		}
		if ((st & SDHCI_INT_BUF_WR_READY) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -5;
	}
	for (i = 0; i < words_total; ++i) {
		data = (uint32_t)buf[i * 4 + 0] | ((uint32_t)buf[i * 4 + 1] << 8) |
			((uint32_t)buf[i * 4 + 2] << 16) | ((uint32_t)buf[i * 4 + 3] << 24);
		*(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT) = data;
	}
	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -6;
		}
		if ((st & SDHCI_INT_XFER_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -7;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	return 0;
}

/* CMD53 (IO_RW_EXTENDED) block-mode WRITE via SDHCI PIO. Mirror of the
 * read: arg bit31 = 1, TRANSFER_MODE bit4 = 0, polls BUFFER_WRITE_READY,
 * writes DATA_PORT. Source is a little-endian byte buffer of at least
 * block_count*block_size bytes. */
static int diag_sdioCmd53Write(volatile uint8_t *sdhci, int fn,
	int incr_addr, uint32_t reg_addr,
	uint32_t block_count, uint32_t block_size,
	const uint8_t *buf)
{
	uint32_t arg, cmd_word;
	uint32_t st;
	uint32_t bytes_total = block_count * block_size;
	uint32_t words_total = bytes_total / 4u;
	uint32_t bytes_in_block = 0;
	uint32_t i;
	int deadline;

	for (deadline = 100000; deadline > 0; --deadline) {
		if ((*(volatile uint32_t *)(sdhci + SDHCI_PRES_STATE) &
			SDHCI_PRES_CMD_INHIBIT) == 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -1;
	}

	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	*(volatile uint32_t *)(sdhci + SDHCI_BLOCK_SIZE_CNT) =
		(block_count << 16) | (block_size & 0xFFFu);

	arg = (1u << 31) |
		((uint32_t)(fn & 7u) << 28) |
		(1u << 27) |
		((incr_addr ? 1u : 0u) << 26) |
		((reg_addr & 0x1FFFFu) << 9) |
		(block_count & 0x1FFu);
	*(volatile uint32_t *)(sdhci + SDHCI_ARGUMENT_1) = arg;

	cmd_word =
		(1u << 1) |
		((block_count > 1u ? 1u : 0u) << 5) |
		((uint32_t)0x3Au << 16) |
		((uint32_t)53u << 24);
	*(volatile uint32_t *)(sdhci + SDHCI_TRANS_CMD) = cmd_word;

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -2;
		}
		if ((st & SDHCI_INT_CMD_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	for (i = 0; i < words_total; ++i) {
		for (deadline = 100000; deadline > 0; --deadline) {
			st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
			if ((st & SDHCI_INT_ERR_ANY) != 0u) {
				return -4;
			}
			if ((st & SDHCI_INT_BUF_WR_READY) != 0u) {
				break;
			}
		}
		if (deadline == 0) {
			return -5;
		}

		{
			uint32_t data = (uint32_t)buf[i * 4 + 0] |
				((uint32_t)buf[i * 4 + 1] << 8) |
				((uint32_t)buf[i * 4 + 2] << 16) |
				((uint32_t)buf[i * 4 + 3] << 24);
			*(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT) = data;
		}

		bytes_in_block += 4u;
		if (bytes_in_block >= block_size) {
			bytes_in_block = 0u;
			*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_WR_READY;
		}
	}

	for (deadline = 100000; deadline > 0; --deadline) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -6;
		}
		if ((st & SDHCI_INT_XFER_COMPLETE) != 0u) {
			break;
		}
	}
	if (deadline == 0) {
		return -7;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	return 0;
}

/* ------------------------------------------------------------------ */
/* ---- #91 EROM (DMP) walk -------------------------------------------------
 * Replicates brcmfmac's brcmf_chip_dmp_erom_scan (external/linux .../chip.c)
 * to enumerate the chip's cores over the SDIO backplane, replacing the probe's
 * remaining HARDCODED core-address hypotheses (CR4 wrapper 0x18102000, SDIOD
 * mailbox 0x18005000, ram-top 0x238000) with the chip's own EROM answers.
 * Read-only. The bases it reports feed the fw-precondition bursts: the CR4
 * CORE base (=> ARMCR4_CAP/BANKINFO ramsize) and the SDIO-DEV core base
 * (=> the intstatus clear brcmf_sdio_buscore_activate does + the true HMB
 * mailbox). */
#define SI_ENUM_BASE_43455 0x18000000u
#define CC_EROMPTR_OFF 0x000000fcu
#define DMP_DESC_TYPE_MSK 0x0000000Fu
#define DMP_DESC_EMPTY 0x00000000u
#define DMP_DESC_VALID 0x00000001u
#define DMP_DESC_COMPONENT 0x00000001u
#define DMP_DESC_MASTER_PORT 0x00000003u
#define DMP_DESC_ADDRESS 0x00000005u
#define DMP_DESC_ADDRSIZE_GT32 0x00000008u
#define DMP_DESC_EOT 0x0000000Fu
#define DMP_COMP_PARTNUM 0x000FFF00u
#define DMP_COMP_PARTNUM_S 8
#define DMP_COMP_REVISION 0xFF000000u
#define DMP_COMP_REVISION_S 24
#define DMP_COMP_NUM_SWRAP 0x00F80000u
#define DMP_COMP_NUM_SWRAP_S 19
#define DMP_COMP_NUM_MWRAP 0x0007C000u
#define DMP_COMP_NUM_MWRAP_S 14
#define DMP_SLAVE_ADDR_BASE 0xFFFFF000u
#define DMP_SLAVE_TYPE 0x000000C0u
#define DMP_SLAVE_TYPE_S 6
#define DMP_SLAVE_TYPE_SLAVE 0u
#define DMP_SLAVE_TYPE_SWRAP 2u
#define DMP_SLAVE_TYPE_MWRAP 3u
#define DMP_SLAVE_SIZE_TYPE 0x00000030u
#define DMP_SLAVE_SIZE_TYPE_S 4
#define DMP_SLAVE_SIZE_4K 0u
#define DMP_SLAVE_SIZE_8K 1u
#define DMP_SLAVE_SIZE_DESC 3u
#define BCMA_ID_PMU 0x827u
#define BCMA_ID_GCI 0x840u
#define BCMA_ID_ARM_CR4 0x83Eu
#define BCMA_ID_SDIO_DEV 0x829u
#define BCMA_ID_INTERNAL_MEM 0x80Eu
#define BCMA_ID_CHIPCOMMON 0x800u

#define EROM_MAX_CORES 40

static int g_erom_ncores = -1; /* -1 = walk not run/failed */
static uint16_t g_erom_id[EROM_MAX_CORES];
static uint8_t g_erom_rev[EROM_MAX_CORES];
static uint32_t g_erom_base[EROM_MAX_CORES];
static uint32_t g_erom_wrap[EROM_MAX_CORES];
static uint32_t g_erom_ptr = 0u; /* the eromptr value we read */

/* Read one backplane byte at chip-internal `addr`, windowing per-byte so a
 * 32-bit read that straddles a 32 KiB SBADDR window boundary is still correct. */
static uint8_t diag_bpRead8(volatile uint8_t *sdhci, uint32_t addr)
{
	uint32_t resp[4] = {0};
	uint8_t lo = (uint8_t)(((addr >> 15) & 1u) ? 0x80u : 0x00u);
	uint8_t mid = (uint8_t)((addr >> 16) & 0xffu);
	uint8_t hi = (uint8_t)((addr >> 24) & 0xffu);
	uint32_t f1 = addr & 0x7FFFu;
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, lo, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, mid, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, hi, NULL);
	(void)diag_sdioCmd52(sdhci, 0, 1, f1, 0u, resp);
	return (uint8_t)(resp[0] & 0xffu);
}

static uint32_t diag_bpRead32(volatile uint8_t *sdhci, uint32_t addr)
{
	return (uint32_t)diag_bpRead8(sdhci, addr) |
		((uint32_t)diag_bpRead8(sdhci, addr + 1u) << 8) |
		((uint32_t)diag_bpRead8(sdhci, addr + 2u) << 16) |
		((uint32_t)diag_bpRead8(sdhci, addr + 3u) << 24);
}

/* Write one backplane byte at chip-internal `addr` (per-byte windowing). */
static void diag_bpWrite8(volatile uint8_t *sdhci, uint32_t addr, uint8_t v)
{
	uint8_t lo = (uint8_t)(((addr >> 15) & 1u) ? 0x80u : 0x00u);
	uint8_t mid = (uint8_t)((addr >> 16) & 0xffu);
	uint8_t hi = (uint8_t)((addr >> 24) & 0xffu);
	uint32_t f1 = addr & 0x7FFFu;
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, lo, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, mid, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, hi, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, f1, v, NULL);
}

static void diag_bpWrite32(volatile uint8_t *sdhci, uint32_t addr, uint32_t v)
{
	diag_bpWrite8(sdhci, addr, (uint8_t)(v & 0xffu));
	diag_bpWrite8(sdhci, addr + 1u, (uint8_t)((v >> 8) & 0xffu));
	diag_bpWrite8(sdhci, addr + 2u, (uint8_t)((v >> 16) & 0xffu));
	diag_bpWrite8(sdhci, addr + 3u, (uint8_t)((v >> 24) & 0xffu));
}

/* Compute the ARMCR4 TCM RAM size from bankinfo (brcmf_chip_tcm_ramsize).
 * cr4_core = the CR4 CORE base (NOT the wrapper). Returns bytes, 0 on failure. */
#define ARMCR4_CAP_OFF 0x04u
#define ARMCR4_BANKIDX_OFF 0x40u
#define ARMCR4_BANKINFO_OFF 0x44u
#define ARMCR4_TCBANB_MASK 0x0000000Fu
#define ARMCR4_TCBBNB_MASK 0x000000F0u
#define ARMCR4_TCBBNB_SHIFT 4
#define ARMCR4_BSZ_MASK 0x0000007Fu
#define ARMCR4_BSZ_MULT 8192u
#define ARMCR4_BLK_1K_MASK 0x00000200u
static uint32_t diag_cr4RamSize(volatile uint8_t *sdhci, uint32_t cr4_core)
{
	uint32_t corecap, memsize = 0u, blksize, bxinfo;
	uint32_t nab, nbb, totb, idx;

	if (cr4_core == 0u) {
		return 0u;
	}
	corecap = diag_bpRead32(sdhci, cr4_core + ARMCR4_CAP_OFF);
	nab = (corecap & ARMCR4_TCBANB_MASK);
	nbb = (corecap & ARMCR4_TCBBNB_MASK) >> ARMCR4_TCBBNB_SHIFT;
	totb = nab + nbb;
	for (idx = 0u; idx < totb && idx < 64u; ++idx) {
		diag_bpWrite32(sdhci, cr4_core + ARMCR4_BANKIDX_OFF, idx);
		bxinfo = diag_bpRead32(sdhci, cr4_core + ARMCR4_BANKINFO_OFF);
		blksize = ARMCR4_BSZ_MULT;
		if (bxinfo & ARMCR4_BLK_1K_MASK) {
			blksize >>= 3; /* 1024 */
		}
		memsize += ((bxinfo & ARMCR4_BSZ_MASK) + 1u) * blksize;
	}
	return memsize;
}

/* get one EROM descriptor, advancing the cursor; classify ADDRESS variants. */
static uint32_t diag_dmpGetDesc(volatile uint8_t *sdhci, uint32_t *ea, uint8_t *type)
{
	uint32_t val = diag_bpRead32(sdhci, *ea);
	*ea += 4u;
	if (type != NULL) {
		*type = (uint8_t)(val & DMP_DESC_TYPE_MSK);
		if ((uint32_t)(*type & ~DMP_DESC_ADDRSIZE_GT32) == DMP_DESC_ADDRESS) {
			*type = (uint8_t)DMP_DESC_ADDRESS;
		}
	}
	return val;
}

/* obtain the (slave) regbase + wrapper base for the current component. Mirrors
 * brcmf_chip_dmp_get_regaddr. */
static int diag_dmpGetRegaddr(volatile uint8_t *sdhci, uint32_t *ea,
	uint32_t *regbase, uint32_t *wrapbase)
{
	uint8_t desc, stype, sztype, wraptype;
	uint32_t val, szdesc;

	*regbase = 0u;
	*wrapbase = 0u;

	val = diag_dmpGetDesc(sdhci, ea, &desc);
	if (desc == (uint8_t)DMP_DESC_MASTER_PORT) {
		wraptype = (uint8_t)DMP_SLAVE_TYPE_MWRAP;
	}
	else if (desc == (uint8_t)DMP_DESC_ADDRESS) {
		*ea -= 4u; /* revert */
		wraptype = (uint8_t)DMP_SLAVE_TYPE_SWRAP;
	}
	else {
		*ea -= 4u;
		return -1;
	}

	do {
		do {
			val = diag_dmpGetDesc(sdhci, ea, &desc);
			if (desc == (uint8_t)DMP_DESC_EOT) {
				*ea -= 4u;
				return -2;
			}
		} while (desc != (uint8_t)DMP_DESC_ADDRESS &&
			desc != (uint8_t)DMP_DESC_COMPONENT);

		if (desc == (uint8_t)DMP_DESC_COMPONENT) {
			*ea -= 4u;
			return 0;
		}

		if (val & DMP_DESC_ADDRSIZE_GT32) {
			(void)diag_dmpGetDesc(sdhci, ea, NULL);
		}

		sztype = (uint8_t)((val & DMP_SLAVE_SIZE_TYPE) >> DMP_SLAVE_SIZE_TYPE_S);
		if (sztype == (uint8_t)DMP_SLAVE_SIZE_DESC) {
			szdesc = diag_dmpGetDesc(sdhci, ea, NULL);
			if (szdesc & DMP_DESC_ADDRSIZE_GT32) {
				(void)diag_dmpGetDesc(sdhci, ea, NULL);
			}
		}

		if (sztype != (uint8_t)DMP_SLAVE_SIZE_4K &&
			sztype != (uint8_t)DMP_SLAVE_SIZE_8K) {
			continue;
		}

		stype = (uint8_t)((val & DMP_SLAVE_TYPE) >> DMP_SLAVE_TYPE_S);
		if (*regbase == 0u && stype == (uint8_t)DMP_SLAVE_TYPE_SLAVE) {
			*regbase = val & DMP_SLAVE_ADDR_BASE;
		}
		if (*wrapbase == 0u && stype == wraptype) {
			*wrapbase = val & DMP_SLAVE_ADDR_BASE;
		}
	} while (*regbase == 0u || *wrapbase == 0u);

	return 0;
}

/* Walk the EROM, filling g_erom_*. Returns core count (>=0) or <0 on error. */
static int diag_eromWalk(volatile uint8_t *sdhci)
{
	uint32_t eromaddr, val;
	uint8_t desc_type = 0u;
	uint16_t id;
	uint8_t nmw, nsw, rev;
	uint32_t base, wrap;
	int n = 0;
	int guard = 0;

	g_erom_ptr = diag_bpRead32(sdhci, SI_ENUM_BASE_43455 + CC_EROMPTR_OFF);
	eromaddr = g_erom_ptr;
	if (eromaddr == 0u || eromaddr == 0xFFFFFFFFu) {
		return -1;
	}

	while (desc_type != (uint8_t)DMP_DESC_EOT && n < EROM_MAX_CORES && guard < 4096) {
		guard++;
		val = diag_dmpGetDesc(sdhci, &eromaddr, &desc_type);
		if (!(val & DMP_DESC_VALID)) {
			continue;
		}
		if (desc_type == (uint8_t)DMP_DESC_EMPTY) {
			continue;
		}
		if (desc_type != (uint8_t)DMP_DESC_COMPONENT) {
			continue;
		}

		id = (uint16_t)((val & DMP_COMP_PARTNUM) >> DMP_COMP_PARTNUM_S);

		val = diag_dmpGetDesc(sdhci, &eromaddr, &desc_type);
		if ((val & DMP_DESC_TYPE_MSK) != DMP_DESC_COMPONENT) {
			return (n > 0) ? n : -2; /* malformed */
		}

		nmw = (uint8_t)((val & DMP_COMP_NUM_MWRAP) >> DMP_COMP_NUM_MWRAP_S);
		nsw = (uint8_t)((val & DMP_COMP_NUM_SWRAP) >> DMP_COMP_NUM_SWRAP_S);
		rev = (uint8_t)((val & DMP_COMP_REVISION) >> DMP_COMP_REVISION_S);

		if ((nmw + nsw) == 0 && id != BCMA_ID_PMU && id != BCMA_ID_GCI) {
			continue;
		}

		if (diag_dmpGetRegaddr(sdhci, &eromaddr, &base, &wrap) != 0) {
			continue;
		}

		g_erom_id[n] = id;
		g_erom_rev[n] = rev;
		g_erom_base[n] = base;
		g_erom_wrap[n] = wrap;
		n++;
	}

	return n;
}

/* Look up a core base (or wrap) by id from the walk results; 0 if not found. */
static uint32_t diag_eromCoreBase(uint16_t id)
{
	int i;
	for (i = 0; i < g_erom_ncores; ++i) {
		if (g_erom_id[i] == id) {
			return g_erom_base[i];
		}
	}
	return 0u;
}

static uint32_t diag_eromCoreWrap(uint16_t id)
{
	int i;
	for (i = 0; i < g_erom_ncores; ++i) {
		if (g_erom_id[i] == id) {
			return g_erom_wrap[i];
		}
	}
	return 0u;
}

/* ---- #91 sdpcm_shared + firmware console ---------------------------------
 * Port of brcmf_sdio_readshared (sdio.c): the fw, once booted, overwrites the
 * word at ram_top-4 (where the NVRAM length-magic token was) with a pointer to
 * its sdpcm_shared struct. From there console_addr -> rte_console gives the fw
 * console ring buffer -- letting us SEE what the firmware prints instead of
 * poking blind. On-dongle (32-bit) offsets: sdpcm_shared { flags@0, trap@4,
 * assert_exp@8, assert_file@12, assert_line@16, console_addr@20 }; rte_console
 * { ... log_le@8 { buf@0, buf_size@4, idx@8 } } => log_buf@console+8,
 * buf_size@console+12, idx@console+16. */
#define FWCON_MAX 1536
static int g_shared_valid = -1; /* -1 not attempted, 0 invalid, 1 valid */
static uint32_t g_sh_word = 0u, g_sh_addr = 0u, g_sh_flags = 0u, g_trap_addr = 0u;
static uint32_t g_console_addr = 0u, g_log_buf = 0u, g_log_bufsize = 0u, g_log_idx = 0u;
static char g_console[FWCON_MAX];
static int g_console_len = 0;

static void diag_readShared(volatile uint8_t *sdhci, uint32_t ram_size)
{
	uint32_t shaddr, a, n, i;

	g_shared_valid = 0;
	g_console_len = 0;
	if (ram_size == 0u) {
		return;
	}
	shaddr = 0x198000u + ram_size - 4u;
	a = diag_bpRead32(sdhci, shaddr);
	g_sh_word = a;
	/* brcmf_sdio_valid_shared_address: the NVRAM-token pattern (~x<<16)|x is
	 * INVALID -> means the fw never overwrote it -> not booted. */
	if (a == 0u || (((~a >> 16) & 0xffffu) == (a & 0xffffu))) {
		return;
	}
	g_shared_valid = 1;
	g_sh_addr = a;
	g_sh_flags = diag_bpRead32(sdhci, a + 0u);
	g_trap_addr = diag_bpRead32(sdhci, a + 4u);
	g_console_addr = diag_bpRead32(sdhci, a + 20u);
	if (g_console_addr != 0u && g_console_addr != 0xffffffffu) {
		g_log_buf = diag_bpRead32(sdhci, g_console_addr + 8u);
		g_log_bufsize = diag_bpRead32(sdhci, g_console_addr + 12u);
		g_log_idx = diag_bpRead32(sdhci, g_console_addr + 16u);
		if (g_log_buf != 0u && g_log_buf != 0xffffffffu) {
			n = g_log_idx;
			if (n > (uint32_t)(FWCON_MAX - 1)) {
				n = (uint32_t)(FWCON_MAX - 1);
			}
			if (g_log_bufsize != 0u && n > g_log_bufsize) {
				n = g_log_bufsize;
			}
			for (i = 0u; i < n; ++i) {
				g_console[i] = (char)diag_bpRead8(sdhci, g_log_buf + i);
			}
			g_console_len = (int)n;
		}
	}
}

/* Software-reset the SDHCI CMD + DAT lines (reg 0x2C, bits 25/26) to recover a
 * wedged data transfer, without a full controller reset. */
static void diag_sdhciResetDatCmd(volatile uint8_t *sdhci)
{
	uint32_t v = *(volatile uint32_t *)(sdhci + SDHCI_CLK_TIMEOUT_RESET);
	int d;
	*(volatile uint32_t *)(sdhci + SDHCI_CLK_TIMEOUT_RESET) =
		v | SDHCI_SOFT_RESET_CMD | SDHCI_SOFT_RESET_DAT;
	for (d = 0; d < 100000; ++d) {
		if ((*(volatile uint32_t *)(sdhci + SDHCI_CLK_TIMEOUT_RESET) &
			(SDHCI_SOFT_RESET_CMD | SDHCI_SOFT_RESET_DAT)) == 0u) {
			break;
		}
	}
}

/* ---- #91 BCDC control ioctl round-trip over F2 ---------------------------
 * First real driver protocol: send one BCDC GET (WLC_GET_VERSION=1) wrapped in
 * an SDPCM control frame over SDIO function 2, poll the SDIO-core intstatus for
 * I_HMB_FRAME_IND, read the reply back from the F2 FIFO, strip SDPCM+BCDC, and
 * report the returned u32 version. Spec derived byte-for-byte from brcmfmac
 * (bcdc.c/sdio.c/bcmsdh.c). F2 frame addressing: backplane window 0x18000000,
 * CMD53 addr 0x8000; write=incrementing, read=fixed FIFO. Small frame padded to
 * a 64-byte block (F2 blocksize set to 64 via CCCR FBR to reuse block-mode). */
#define IOCTL_F2_ADDR 0x8000u
#define F2_FRAME_MAX 512u    /* per-frame F2 read size (byte-mode cap; card pads short frames) */
static int g_evt_seen = 0;             /* chan-1 (event) frames demuxed past */
static int g_ctrl_seen = 0;            /* chan-0 (control) frames read */
static uint16_t g_last_evt_len = 0u;
static uint8_t g_last_evt[32];         /* head of the last event frame seen */

static uint32_t diag_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
		((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void diag_setWindow18(volatile uint8_t *sdhci)
{
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x00u, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x00u, NULL);
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x18u, NULL);
}

/* Read one SDPCM frame from the F2 FIFO into buf (>= F2_FRAME_MAX). Reads a
 * fixed F2_FRAME_MAX bytes: a short read (< frame length) CRCs the SDIO data
 * phase, and the card pads a frame shorter than the request. Parses the HW
 * header: *outlen = SDPCM frame length, *outchan = SDPCM channel. Returns 0 on
 * a valid frame, 1 if none is ready (len|chk==0), <0 on a transport error (and
 * resets DAT/CMD to clear a wedge). */
static int diag_f2RecvFrame(volatile uint8_t *sdhci, uint8_t *buf,
	uint16_t *outlen, uint8_t *outchan)
{
	uint16_t len, chk;
	int rc;

	*outlen = 0u;
	*outchan = 0xffu;
	diag_setWindow18(sdhci);
	rc = diag_sdioCmd53ReadByteMode(sdhci, 2, /*incr=*/0, IOCTL_F2_ADDR,
		F2_FRAME_MAX, buf);
	if (rc != 0) {
		diag_sdhciResetDatCmd(sdhci);
		return -30;
	}
	len = (uint16_t)(buf[0] | (buf[1] << 8));
	chk = (uint16_t)(buf[2] | (buf[3] << 8));
	if (len == 0u && chk == 0u) {
		return 1; /* no frame ready */
	}
	if ((uint16_t)(~(len ^ chk)) != 0u || len < 12u) {
		diag_sdhciResetDatCmd(sdhci);
		return -31;
	}
	/* Clamp the fw-claimed length to what we actually read (512): `len` is a
	 * fw-controlled 16-bit field, and every downstream offset check (event
	 * stack: ehdr = sdoff + 4 + 4*data_offset, bss = ehdr+84, ...) is bounded
	 * against *outlen -- an unclamped len would let a malformed large frame
	 * drive those indices past g_rxf[512]. We only ever read one 512B frame. */
	if (len > (uint16_t)F2_FRAME_MAX) {
		len = (uint16_t)F2_FRAME_MAX;
	}
	*outlen = len;
	*outchan = (uint8_t)(buf[5] & 0x0fu);
	return 0;
}

/* Send a BCDC dcmd (GET if is_set==0, SET if 1) carrying txlen payload bytes,
 * then read F2 frames demuxing SDPCM channels until the CONTROL reply whose
 * BCDC id matches reqid; copy up to rxcap payload bytes to rxbuf. EVENT (chan 1)
 * frames seen meanwhile are counted (g_evt_seen) and the last stashed
 * (g_last_evt) -- escan results arrive as events. Returns the BCDC status
 * (>=0, 0=ok) on a matched reply, or a negative transport error. */
static uint8_t g_txf[F2_FRAME_MAX];
static uint8_t g_rxf[F2_FRAME_MAX];
static int diag_bcdcCmd(volatile uint8_t *sdhci, uint32_t sdio_core, int is_set,
	uint32_t cmd, const uint8_t *txdata, uint32_t txlen,
	uint8_t *rxbuf, uint32_t rxcap, uint32_t *rxlen,
	uint32_t reqid, uint8_t seq)
{
	uint32_t total = 12u + 16u + txlen; /* SDPCM + BCDC + payload */
	uint32_t flags = (reqid << 16) | (is_set ? 0x02u : 0x00u);
	uint16_t frlen = (uint16_t)total;
	uint32_t i, st, wlen;
	int rc, tries;

	if (rxlen != NULL) {
		*rxlen = 0u;
	}
	if (total > F2_FRAME_MAX) {
		return -1040; /* transport errors use <= -1000 so they can't be mistaken for a fw BCME_* status */
	}
	for (i = 0; i < F2_FRAME_MAX; ++i) {
		g_txf[i] = 0u;
	}
	g_txf[0] = (uint8_t)(frlen & 0xffu);
	g_txf[1] = (uint8_t)((frlen >> 8) & 0xffu);
	g_txf[2] = (uint8_t)((~frlen) & 0xffu);
	g_txf[3] = (uint8_t)(((~frlen) >> 8) & 0xffu);
	g_txf[4] = seq;
	g_txf[7] = 12u; /* data_offset */
	g_txf[12] = (uint8_t)(cmd & 0xffu);
	g_txf[13] = (uint8_t)((cmd >> 8) & 0xffu);
	g_txf[14] = (uint8_t)((cmd >> 16) & 0xffu);
	g_txf[15] = (uint8_t)((cmd >> 24) & 0xffu);
	g_txf[16] = (uint8_t)(txlen & 0xffu);
	g_txf[17] = (uint8_t)((txlen >> 8) & 0xffu);
	g_txf[18] = (uint8_t)((txlen >> 16) & 0xffu);
	g_txf[19] = (uint8_t)((txlen >> 24) & 0xffu);
	g_txf[20] = (uint8_t)(flags & 0xffu);
	g_txf[21] = (uint8_t)((flags >> 8) & 0xffu);
	g_txf[22] = (uint8_t)((flags >> 16) & 0xffu);
	g_txf[23] = (uint8_t)((flags >> 24) & 0xffu);
	/* status @24 = 0; payload @28.. */
	for (i = 0; i < txlen; ++i) {
		g_txf[28u + i] = (txdata != NULL) ? txdata[i] : 0u;
	}

	diag_setWindow18(sdhci);
	wlen = (total + 3u) & ~3u; /* pad to 4 */
	rc = diag_sdioCmd53WriteByteMode(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR, wlen, g_txf);
	if (rc != 0) {
		diag_sdhciResetDatCmd(sdhci);
		return -1041; /* transport error range (<= -1000), distinct from fw BCME_* */
	}

	/* Drain the F2 RX FIFO directly rather than one-frame-per-interrupt: the fw
	 * asserts I_HMB_FRAME_IND once for "frames available", so after reading the
	 * queued event the reply would be missed if we waited for a fresh IND. Read
	 * frames until the matching reply arrives or the FIFO stays empty. */
	for (tries = 0; tries < 600; ++tries) {
		uint16_t len;
		uint8_t chan;
		int fr;
		fr = diag_f2RecvFrame(sdhci, g_rxf, &len, &chan);
		if (fr == 1) {
			usleep(2000); /* FIFO empty -- wait for the reply to land */
			continue;
		}
		if (fr < 0) {
			usleep(1000); /* transient transport hiccup */
			continue;
		}
		/* clear the frame-ready indication as we drain */
		st = diag_bpRead32(sdhci, sdio_core + 0x20u);
		if (st != 0u && st != 0xffffffffu) {
			diag_bpWrite32(sdhci, sdio_core + 0x20u, st);
		}
		if (chan == 1u) {
			g_evt_seen++;
			g_last_evt_len = len;
			for (i = 0u; i < 32u && i < len; ++i) {
				g_last_evt[i] = g_rxf[i];
			}
			continue;
		}
		if (chan == 0u) {
			uint8_t doff = g_rxf[7];
			g_ctrl_seen++;
			if ((uint32_t)doff + 16u <= len) {
				uint32_t rflags = diag_le32(g_rxf + doff + 8);
				uint32_t rstat = diag_le32(g_rxf + doff + 12);
				if ((rflags >> 16) == reqid) {
					uint32_t plen = (uint32_t)len - doff - 16u;
					if (rxbuf != NULL) {
						for (i = 0u; i < plen && i < rxcap; ++i) {
							rxbuf[i] = g_rxf[doff + 16u + i];
						}
					}
					if (rxlen != NULL) {
						*rxlen = plen;
					}
					return (int)rstat;
				}
			}
			continue; /* control frame, wrong id -- keep looking */
		}
		/* data channel -- ignore */
	}
	return -1042; /* no matching reply (transport error range, distinct from fw BCME_*) */
}

/* ---- #91 WiFi scan (escan) over the BCDC ioctl API ------------------------
 * Prelude (event_msgs bit69 -> WLC_UP -> mpc0) then SET_VAR "escan" (V1 108B
 * broadcast active), then read WLC_E_ESCAN_RESULT (type 69) events off SDPCM
 * channel 1 and extract each AP. See tools/wifi-probe/SCAN-SPEC.md. */
#define WLC_UP_CMD 2u
#define BRCMF_C_SET_INFRA 20u
#define SET_VAR_CMD 263u
#define GET_VAR_CMD 262u
#define SCAN_MAX_APS 16

static int g_scan_ran = 0;
static uint32_t g_ram_size = 0u; /* set in the main flow; used to re-read the fw console after scan */
static int g_scan_em_rc = -100, g_scan_infra_rc = -100, g_scan_up_rc = -100, g_scan_mpc_rc = -100, g_scan_escan_rc = -100;
static int g_scan_escan_tries = 0;
static int g_clm_chunks = 0, g_clm_last_rc = -100;
static uint32_t g_chanspecs_count = 0xffffffffu; /* channels the fw reports usable after UP */
static int g_mac_rc = -100, g_mac_valid = 0;
static uint8_t g_mac[6] = { 0 };
static int g_scan_ap_count = 0, g_scan_evt_total = 0, g_scan_escan_events = 0;
static int g_scan_done_status = -1;
static struct {
	uint8_t bssid[6];
	uint8_t ssid_len;
	char ssid[33];
	int16_t rssi;
	uint8_t chan;
} g_scan_aps[SCAN_MAX_APS];

static uint16_t diag_be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
static uint32_t diag_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* Issue an iovar (SET if is_set, else GET): payload = "name\0" + data. */
static uint8_t g_iov[512];
static int diag_iovar(volatile uint8_t *sdhci, uint32_t sdio_core, int is_set,
	const char *name, const uint8_t *data, uint32_t dlen,
	uint8_t *rx, uint32_t rxcap, uint32_t *rxlen, uint32_t reqid, uint8_t seq)
{
	uint32_t nl = 0u, i;
	while (name[nl] != '\0') {
		nl++;
	}
	nl++; /* include the NUL */
	if (nl + dlen > sizeof(g_iov)) {
		return -50;
	}
	for (i = 0u; i < nl; ++i) {
		g_iov[i] = (uint8_t)name[i];
	}
	for (i = 0u; i < dlen; ++i) {
		g_iov[nl + i] = (data != NULL) ? data[i] : 0u;
	}
	return diag_bcdcCmd(sdhci, sdio_core, is_set,
		is_set ? SET_VAR_CMD : GET_VAR_CMD, g_iov, nl + dlen,
		rx, rxcap, rxlen, reqid, seq);
}

/* Download the CLM (regulatory/channel) blob via the "clmload" iovar BEFORE
 * WLC_UP -- on the 43455 the channel set lives here; without it WLC_UP returns
 * OK but the radio has no channels and escan is refused NOTUP. Format (brcmf
 * common.c): payload = brcmf_dload_data_le { le16 flag; le16 dload_type=2(CLM);
 * le32 len; le32 crc=0 } + chunk. flag = 0x1000(ver) | 0x2(DL_BEGIN first) |
 * 0x4(DL_END last). brcmf uses 1400B chunks but byte-mode CMD53 caps at 512, so
 * chunk at 384B (fits SDPCM+BCDC+"clmload\0"+hdr+data in one 512B F2 frame). */
#define CLM_CHUNK 384u
static int diag_clmLoad(volatile uint8_t *sdhci, uint32_t sdio_core,
	uint32_t *reqid, uint8_t *seq)
{
	static uint8_t clmbuf[12 + CLM_CHUNK];
	uint32_t off = 0u, chunk, i;
	int rc = 0;

	g_clm_chunks = 0;
	while (off < clm_43455_len) {
		uint16_t flag = 0x1000u; /* DLOAD_HANDLER_VER<<12 */
		chunk = clm_43455_len - off;
		if (chunk > CLM_CHUNK) {
			chunk = CLM_CHUNK;
		}
		if (off == 0u) {
			flag |= 0x0002u; /* DL_BEGIN */
		}
		if (off + chunk >= clm_43455_len) {
			flag |= 0x0004u; /* DL_END */
		}
		clmbuf[0] = (uint8_t)(flag & 0xffu);
		clmbuf[1] = (uint8_t)((flag >> 8) & 0xffu);
		clmbuf[2] = 2u; /* dload_type = DL_TYPE_CLM */
		clmbuf[3] = 0u;
		clmbuf[4] = (uint8_t)(chunk & 0xffu);
		clmbuf[5] = (uint8_t)((chunk >> 8) & 0xffu);
		clmbuf[6] = 0u;
		clmbuf[7] = 0u;
		clmbuf[8] = 0u; clmbuf[9] = 0u; clmbuf[10] = 0u; clmbuf[11] = 0u; /* crc=0 */
		for (i = 0u; i < chunk; ++i) {
			clmbuf[12 + i] = clm_43455[off + i];
		}
		rc = diag_iovar(sdhci, sdio_core, 1, "clmload", clmbuf, 12u + chunk,
			NULL, 0u, NULL, (*reqid)++, (*seq)++);
		g_clm_last_rc = rc;
		g_clm_chunks++;
		if (rc != 0) {
			break;
		}
		off += chunk;
	}
	return rc;
}

static void diag_wifiScan(volatile uint8_t *sdhci, uint32_t sdio_core)
{
	uint8_t emask[16];
	uint8_t up[4] = { 0, 0, 0, 0 };
	uint8_t mpc[4] = { 0, 0, 0, 0 };
	uint8_t escan[108];
	uint32_t reqid = 1u;
	uint8_t seq = 0u;
	int i, t, done = 0;

	g_scan_ran = 1;
	diag_sdhciResetDatCmd(sdhci);

	/* event_msgs: enable WLC_E_ESCAN_RESULT (69): mask[8] |= 0x20 */
	for (i = 0; i < 16; ++i) {
		emask[i] = 0u;
	}
	emask[8] = 0x20u;
	g_scan_em_rc = diag_iovar(sdhci, sdio_core, 1, "event_msgs", emask, 16u,
		NULL, 0u, NULL, reqid++, seq++);

	/* CLM (regulatory/channel) blob BEFORE UP -- the missing precondition:
	 * without it WLC_UP returns OK but the radio has no channels => escan
	 * NOTUP. */
	(void)diag_clmLoad(sdhci, sdio_core, &reqid, &seq);

	/* SET_INFRA 1 (STA / infrastructure mode). */
	{
		uint8_t infra[4] = { 1, 0, 0, 0 };
		g_scan_infra_rc = diag_bcdcCmd(sdhci, sdio_core, 1, BRCMF_C_SET_INFRA,
			infra, 4u, NULL, 0u, NULL, reqid++, seq++);
	}

	/* WLC_UP (value ignored by fw; brcmf passes 0 on the STA path). */
	up[0] = 1u;
	g_scan_up_rc = diag_bcdcCmd(sdhci, sdio_core, 1, WLC_UP_CMD, up, 4u,
		NULL, 0u, NULL, reqid++, seq++);

	/* Validate the GET_VAR reply path with cur_etheraddr (must return the Pi's
	 * 6-byte MAC) -- join/assoc status reads lean on GET_VAR. */
	{
		uint8_t mac[8] = { 0 };
		uint32_t ml = 0u;
		g_mac_rc = diag_iovar(sdhci, sdio_core, 0, "cur_etheraddr", NULL, 6u,
			mac, sizeof(mac), &ml, reqid++, seq++);
		if (g_mac_rc >= 0 && ml >= 6u) {
			int k;
			for (k = 0; k < 6; ++k) {
				g_mac[k] = mac[k];
			}
			g_mac_valid = 1;
		}
	}

	/* GET "chanspecs": count>0 confirms usable channels. The reply is the full
	 * chanspec list, so the OUTPUT buffer (BCDC len) must be sized large -- a
	 * too-small GET returns BCME_BUFTOOSHORT (the earlier chanspecs=-1). We only
	 * need the leading le32 count. */
	{
		uint8_t cs[8] = { 0 };
		uint32_t cl = 0u;
		int crc = diag_iovar(sdhci, sdio_core, 0, "chanspecs", NULL, 256u,
			cs, sizeof(cs), &cl, reqid++, seq++);
		if (crc >= 0 && cl >= 4u) {
			g_chanspecs_count = diag_le32(cs);
		}
	}

	/* mpc = 0 (keep radio awake on SDIO parts) */
	g_scan_mpc_rc = diag_iovar(sdhci, sdio_core, 1, "mpc", mpc, 4u,
		NULL, 0u, NULL, reqid++, seq++);

	/* escan params (V1, broadcast active, all channels) */
	for (i = 0; i < 108; ++i) {
		escan[i] = 0u;
	}
	escan[0] = 1u;                 /* version = 1 */
	escan[4] = 1u;                 /* action = WL_ESCAN_ACTION_START */
	escan[6] = 0x34u; escan[7] = 0x12u; /* sync_id = 0x1234 */
	/* ssid_len @8 = 0; ssid[32] @12 = 0 */
	for (i = 44; i < 50; ++i) {
		escan[i] = 0xffu;          /* bssid = broadcast */
	}
	escan[50] = 2u;                /* bss_type = ANY */
	escan[51] = 0u;                /* scan_type = ACTIVE */
	for (i = 52; i < 68; ++i) {
		escan[i] = 0xffu;          /* nprobes/active/passive/home = -1 (default) */
	}
	escan[70] = 1u;                /* channel_num = 0x00010000: n_channels=0(all), n_ssids=1 */
	/* ssid_le[0] @72 = 36 zero bytes = one wildcard SSID (active broadcast) */

	/* WLC_UP acks before the interface finishes coming up (PHY init), so a
	 * too-soon escan gets BCME_NOTUP(-4) ("can not scan while driver is down",
	 * per the fw console). Wait, then retry the escan until it is accepted. */
	for (i = 0; i < 6; ++i) {
		usleep(400 * 1000);
		g_scan_escan_rc = diag_iovar(sdhci, sdio_core, 1, "escan", escan, 108u,
			NULL, 0u, NULL, reqid++, seq++);
		g_scan_escan_tries = i + 1;
		if (g_scan_escan_rc != -4) {
			break; /* accepted (0) or a different error */
		}
	}

	/* Read WLC_E_ESCAN_RESULT events off channel 1 until a non-PARTIAL status. */
	for (t = 0; t < 2000 && !done; ++t) {
		uint16_t len;
		uint8_t chan;
		int fr;
		uint32_t sdoff, ehdr, etype, status, bss;

		fr = diag_f2RecvFrame(sdhci, g_rxf, &len, &chan);
		if (fr == 1) {
			usleep(3000);
			continue;
		}
		if (fr < 0) {
			usleep(2000);
			continue;
		}
		{
			uint32_t st = diag_bpRead32(sdhci, sdio_core + 0x20u);
			if (st != 0u && st != 0xffffffffu) {
				diag_bpWrite32(sdhci, sdio_core + 0x20u, st);
			}
		}
		if (chan != 1u) {
			continue;
		}
		g_scan_evt_total++;

		sdoff = g_rxf[7];
		if (sdoff + 4u > len) {
			continue;
		}
		ehdr = sdoff + 4u + 4u * (uint32_t)g_rxf[sdoff + 3u]; /* skip BDC hdr */
		if (ehdr + 48u > (uint32_t)len) {
			continue;
		}
		/* event_type==69 (below) is the effective filter for escan results; the
		 * brcm_ethhdr OUI (00:10:18) check from SCAN-SPEC is intentionally
		 * omitted (marginal, and a wrong offset would drop valid events). */
		if (diag_be16(g_rxf + ehdr + 12u) != 0x886Cu) {
			continue; /* not an event (h_proto != ETH_P_LINK_CTL) */
		}
		etype = diag_be32(g_rxf + ehdr + 28u);
		if (etype != 69u) {
			continue; /* not WLC_E_ESCAN_RESULT */
		}
		g_scan_escan_events++;
		status = diag_be32(g_rxf + ehdr + 32u);
		if (status != 8u) {          /* not PARTIAL => scan done (SUCCESS/ABORT) */
			g_scan_done_status = (int)status;
			done = 1;
			continue;
		}
		bss = ehdr + 84u;            /* brcmf_bss_info_le */
		if (bss + 90u > (uint32_t)len) {
			continue;
		}
		if (g_scan_ap_count < SCAN_MAX_APS) {
			int k;
			uint8_t sl;
			for (k = 0; k < 6; ++k) {
				g_scan_aps[g_scan_ap_count].bssid[k] = g_rxf[bss + 8u + k];
			}
			sl = g_rxf[bss + 18u];
			if (sl > 32u) {
				sl = 32u;
			}
			g_scan_aps[g_scan_ap_count].ssid_len = sl;
			for (k = 0; k < (int)sl; ++k) {
				g_scan_aps[g_scan_ap_count].ssid[k] = (char)g_rxf[bss + 19u + k];
			}
			g_scan_aps[g_scan_ap_count].ssid[sl] = '\0';
			g_scan_aps[g_scan_ap_count].rssi =
				(int16_t)(g_rxf[bss + 78u] | (g_rxf[bss + 79u] << 8));
			g_scan_aps[g_scan_ap_count].chan = g_rxf[bss + 72u]; /* chanspec low byte */
			g_scan_ap_count++;
		}
	}

	/* Re-read the fw console: any escan rejection is logged there by the fw. */
	diag_readShared(sdhci, g_ram_size);
}

/* ------------------------------------------------------------------ */
/* Resident-driver state. wifi_bringup() runs the firmware bring-up once at
 * startup and leaves the SDIO controller mapped here so wifi_scan() can drive
 * escan transactions against it per client "scan" request. */
static volatile uint8_t *g_sdhci = NULL;   /* SDHCI (Arasan) mapping, kept live after bring-up */
static uint32_t g_sdio_core = 0x18004000u; /* EROM-derived SDIO-DEV core base (set by bring-up) */

#define WIFI_RESP_CAP (8u * 1024u)
static char g_resp[WIFI_RESP_CAP]; /* most recent scan result text, served over mtRead */
static int g_resp_len = 0;

/* The escan chain (diag_wifiScan -> diag_iovar -> diag_bcdcCmd -> diag_f2Recv
 * -> diag_sdioCmd53*) runs on the message thread, so give it a generous stack
 * (cf. MEMORY #152 pool-thread stack-overflow history). */
static char g_msgStack[16 * 1024] __attribute__((aligned(8)));

/* WiFi P3 final: full-firmware load + release ARM-CR4 + look for fw boot.
 *
 * Load pipeline: enum (CMD0/5/3/7) -> F1 enable -> KSO -> HS-mode ->
 * ALP-only backplane clock -> walk 643 KB firmware into SOCRAM at
 * chip-internal 0x198000 -> load NVRAM at 0x238000-len, then:
 *
 *   1. Write the firmware reset vector (first word) to chip-internal 0.
 *   2. Re-window to ARM-CR4 wrapper (0x18100000) and do the brcmfmac AXI
 *      resetcore toggle to release the CR4 (BCMA_IOCTL/RESET_CTL pokes).
 *   3. Enable Function 2 (SDPCM data channel) and wait for F2-ready.
 *   4. Sleep, then read back SOCRAM head + several scan points, HT_AVAIL
 *      (CHIPCLKCSR), SDHCI CARD_INTR, the SOCRAM NVRAM trailer, and the
 *      SDIOD tohostmailboxdata HMB_DATA_FWREADY word.
 *
 * "fw_alive" = HT_AVAIL asserted OR CARD_INTR asserted. See the inline
 * comments (kept verbatim) for the brcmfmac references behind each step. */
static int wifi_bringup(void)
{
	static char logbuf[16u * 1024u];
	char *buf = logbuf;
	size_t cap = sizeof(logbuf);
	/* The driver never runs the probe's trivial-counter self-test, so fold that
	 * mode out to a compile-time constant. The file-scope g_ioctl_mode likewise
	 * stays 0: its gated call keeps diag_bcdcGetVersion linked but never fires,
	 * leaving the proven real-firmware sequence unchanged. */
	const int g_trivial_mode = 0;
	static uint8_t pre_buf[64];
	static uint8_t post_buf[64];
	int off = 0, r;
	void *gpio_page, *sdhci_page;
	uint32_t ocr_resp[4] = {0}, claim_resp[4] = {0};
	uint32_t rca_resp[4] = {0}, sel_resp[4] = {0};
	uint32_t ioen_pre_resp[4] = {0}, iordy_resp[4] = {0};
	uint32_t rc_pre_resp[4] = {0}, rc_post_resp[4] = {0};
	int rc_ocr = -1, rc_claim = -1, rc_sel = -1, rc_iordy = -1;
	int rc_hs = -100;
	int ready_iters = 0, rdy_iters = 0;
	uint16_t rca = 0;
	int rc_w, rc_r_pre = -100, rc_r_post = -100;
	int rc_nvram_w = -100;
	int rc_tail = -100;
	uint8_t chipclk_samples[8] = {0};
	uint8_t socram_tail[16] = {0};
	uint8_t scan_buf[64];
	int scan_rc[6] = {0};
	int scan_diff[6] = {0};
	int scan_changed_pts = -1;
	uint8_t ht_clk_csr = 0u;
	uint8_t f2_ready = 0u;
	int f2_ready_iters = -1;
	uint8_t rstvec_rb[4] = {0};
	uint32_t hmb_data = 0u;
	unsigned card_intr = 0u;
	int worst_rc_w = 0;
	int i, pre_match, post_match, diff_count;
	uint32_t bytes_written = 0u;
	int window_idx = 0;
	size_t fw_offset = 0u;
	size_t fw_target_bytes;
	const uint32_t window_bytes = 32u * 1024u;
	const uint32_t blk_size = 64u;
	const uint32_t blk_count = 64u;
	/* #91 trivial-program test extras (baseline path ignores these). */
	const uint8_t *fw_img = wifi_fw_43455;
	const size_t fw_img_len = (size_t)wifi_fw_43455_len;
	uint8_t cnt_pre[4] = { 0 }, cnt_post[4] = { 0 }, cnt_post2[4] = { 0 };
	int rc_cnt_pre = -100, rc_cnt_post = -100, rc_cnt_post2 = -100;
	uint32_t ioctl_w2 = 0u, ioctl_w3 = 0u; /* dual ARM-wrapper CR4-identity cross-check */
	uint32_t cr4_core = 0u, sdio_core = 0x18004000u, ram_size = 0u; /* EROM-derived bases */

	for (i = 0; i < (int)sizeof(pre_buf); ++i) {
		pre_buf[i] = 0;
		post_buf[i] = 0;
	}

	r = snprintf(buf + off, cap - off, "PHX-DIAG/1 sdio-fwrelease\n");
	if (r < 0 || (size_t)r >= cap - off) {
		return -1;
	}
	off += r;

	if (fw_img_len == 0u) {
		r = snprintf(buf + off, cap - off,
			"error: firmware blob not staged\n.\n");
		off += (r > 0) ? r : 0;
		printf("%.*s", off, buf);
		return 1;
	}
	/* Round down to the 64-byte block: the CR4 image is loaded verbatim to
	 * rambase with no end-of-image trailer, so dropping the <64-byte tail
	 * (643651 % 64 = 3) is benign and the fw boots+scans. (NVRAM IS 64-aligned
	 * = 27*64, so its ram-top magic token is transferred in full.) */
	fw_target_bytes = (fw_img_len / blk_size) * blk_size;

	gpio_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, BCM2711_GPIO_BASE);
	sdhci_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1, 0xfe300000u);

	if (gpio_page == MAP_FAILED || sdhci_page == MAP_FAILED) {
		r = snprintf(buf + off, cap - off, "error: mmap failed\n.\n");
		if (gpio_page != MAP_FAILED) {
			munmap(gpio_page, _PAGE_SIZE);
		}
		if (sdhci_page != MAP_FAILED) {
			munmap(sdhci_page, _PAGE_SIZE);
		}
		off += (r > 0) ? r : 0;
		printf("%.*s", off, buf);
		return 2;
	}

	{
		volatile uint8_t *gpio = (volatile uint8_t *)gpio_page;
		volatile uint8_t *sdhci = (volatile uint8_t *)sdhci_page;

		for (i = 34; i <= 39; ++i) {
			diag_gpioSetFsel(gpio, (unsigned)i, 7u);
		}
		diag_wifiPowerCycle();
		(void)diag_sdhciSetClockKHz(sdhci, 400u);
		(void)diag_sdhciResetCmdDat(sdhci);

		(void)diag_sdhciCmd(sdhci, 0u, 0u, SDHCI_RESP_R0, NULL);
		usleep(1000);
		rc_ocr = diag_sdhciCmd(sdhci, 5u, 0u, SDHCI_RESP_R4, ocr_resp);
		for (ready_iters = 0; ready_iters < 50; ++ready_iters) {
			rc_claim = diag_sdhciCmd(sdhci, 5u, ocr_resp[0] & 0x00ffffffu,
				SDHCI_RESP_R4, claim_resp);
			if (rc_claim != 0) {
				break;
			}
			if ((claim_resp[0] & 0x80000000u) != 0u) {
				ready_iters++;
				break;
			}
			usleep(1000);
		}
		(void)diag_sdhciCmd(sdhci, 3u, 0u, SDHCI_RESP_R6, rca_resp);
		rca = (uint16_t)((rca_resp[0] >> 16) & 0xFFFFu);
		rc_sel = diag_sdhciCmd(sdhci, 7u, (uint32_t)rca << 16, SDHCI_RESP_R1, sel_resp);

		(void)diag_sdioCmd52(sdhci, 0, 0, 0x02u, 0u, ioen_pre_resp);
		(void)diag_sdioCmd52(sdhci, 1, 0, 0x02u,
			(uint8_t)((ioen_pre_resp[0] | 0x02u) & 0xffu), NULL);
		for (rdy_iters = 0; rdy_iters < 50; ++rdy_iters) {
			rc_iordy = diag_sdioCmd52(sdhci, 0, 0, 0x03u, 0u, iordy_resp);
			if (rc_iordy != 0) {
				break;
			}
			if ((iordy_resp[0] & 0x02u) != 0u) {
				rdy_iters++;
				break;
			}
			usleep(1000);
		}

		/* KSO (Keep-SDIO-On) enable. SDIO core rev >= 12 (43455 qualifies)
		 * gates the backplane clock on KSO; without it the device can
		 * drop the clock and HT_AVAIL never latches. SLEEPCSR (F1
		 * 0x1001F) bit 0 = KSO_EN. RMW. */
		{
			uint32_t kso[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1001Fu, 0u, kso);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1001Fu,
				(uint8_t)((kso[0] | 0x01u) & 0xffu), NULL);
		}

		rc_hs = diag_sdioGoHighSpeed(sdhci);

		/* Backplane clock bring-up before CR4 release: ALP ONLY.
		 * Per brcmfmac brcmf_sdio_load_firmware(), the host sets
		 * alp_only=true for the whole firmware-download + CR4-release
		 * window and brings the backplane up on ALP only
		 * (SBSDIO_ALP_AVAIL_REQ 0x08; wait SBSDIO_ALP_AVAIL 0x40). The
		 * firmware running on the CR4 brings HT up itself once executing;
		 * forcing HT here cannot work (the CR4 has no HT clock until fw
		 * requests it). HT_AVAIL is polled AFTER release below as the
		 * firmware-alive tell. */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Eu, 0x08u, NULL);
		for (i = 0; i < 250; ++i) {
			uint32_t cc[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1000Eu, 0u, cc);
			ht_clk_csr = (uint8_t)(cc[0] & 0xffu);
			if ((ht_clk_csr & 0x40u) != 0u) {
				break;
			}
			usleep(2000);
		}

		(void)diag_sdioCmd52(sdhci, 1, 0, 0x110u, 0x40u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 0, 0x111u, 0x00u, NULL);

		/* #91: enumerate cores over the backplane (read-only) now that the
		 * ALP clock is up, so the report can replace the hardcoded core-
		 * address hypotheses with the chip's own EROM answers. Done before
		 * the fw download; it only sets/reads SBADDR windows, which the
		 * download loop re-sets on its first iteration. */
		g_erom_ncores = diag_eromWalk(sdhci);
		cr4_core = diag_eromCoreBase(BCMA_ID_ARM_CR4);
		if (cr4_core == 0u) {
			cr4_core = 0x18002000u; /* EROM-confirmed fallback */
		}
		{
			uint32_t s = diag_eromCoreBase(BCMA_ID_SDIO_DEV);
			if (s != 0u) {
				sdio_core = s;
			}
		}
		/* True TCM ramsize from CR4 bankinfo (fw is halted here — safe). */
		ram_size = diag_cr4RamSize(sdhci, cr4_core);
		g_ram_size = ram_size;

		while (fw_offset < fw_target_bytes && rc_hs == 0) {
			uint32_t addr = 0x00198000u + (uint32_t)window_idx * 0x8000u;
			uint8_t  lo  = (uint8_t)(((addr >> 15) & 1u) ? 0x80u : 0x00u);
			uint8_t  mid = (uint8_t)((addr >> 16) & 0xffu);
			uint8_t  hi  = (uint8_t)((addr >> 24) & 0xffu);
			size_t   remaining = fw_target_bytes - fw_offset;
			size_t   this_window = (remaining > window_bytes) ? window_bytes : remaining;
			uint32_t bytes_per_cmd = blk_count * blk_size;
			uint32_t chunks = (uint32_t)(this_window / bytes_per_cmd);
			uint32_t leftover_blocks = (uint32_t)((this_window % bytes_per_cmd) / blk_size);
			uint32_t ci;

			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, lo,  NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, mid, NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, hi,  NULL);

			for (ci = 0; ci < chunks; ++ci) {
				rc_w = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1,
					/*reg_addr=*/ci * bytes_per_cmd,
					/*block_count=*/blk_count,
					/*block_size=*/blk_size,
					fw_img + fw_offset + ci * bytes_per_cmd);
				if (rc_w != 0) {
					if (worst_rc_w == 0) worst_rc_w = rc_w;
					break;
				}
				bytes_written += bytes_per_cmd;
			}
			if (rc_w != 0) break;

			if (leftover_blocks > 0) {
				rc_w = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1,
					/*reg_addr=*/chunks * bytes_per_cmd,
					/*block_count=*/leftover_blocks,
					/*block_size=*/blk_size,
					fw_img + fw_offset + chunks * bytes_per_cmd);
				if (rc_w != 0) {
					if (worst_rc_w == 0) worst_rc_w = rc_w;
					break;
				}
				bytes_written += leftover_blocks * blk_size;
			}

			fw_offset += this_window;
			window_idx++;
		}

		/* NVRAM load: chip-ready blob goes at chip-internal
		 * (rambase + ramsize - wifi_nvram_43455_len) = 0x238000 - len,
		 * inside SBADDR window 19, padded to a 64-byte boundary so it
		 * lands as a single CMD53 multi-block write. Skipped in the
		 * trivial-program test: the counter needs no NVRAM, and skipping
		 * it removes NVRAM as a variable from a dead-counter result. */
		if (!g_trivial_mode) {
			/* Place NVRAM at the TRUE ram-top from CR4 bankinfo, not the old
			 * hardcoded 0x238000. The bootloader reads the length-magic token
			 * at ram_top-4; a wrong ram-top => fw never finds NVRAM. */
			uint32_t nv_ramtop = (ram_size != 0u) ? (0x198000u + ram_size) : 0x238000u;
			uint32_t nv_start = nv_ramtop - (uint32_t)wifi_nvram_43455_len;
			uint8_t  nv_lo  = (uint8_t)(((nv_start >> 15) & 1u) ? 0x80u : 0x00u);
			uint8_t  nv_mid = (uint8_t)((nv_start >> 16) & 0xffu);
			uint8_t  nv_hi  = (uint8_t)((nv_start >> 24) & 0xffu);
			uint32_t nv_f1_offset = nv_start & 0x7FFFu;
			uint32_t nv_blocks = (uint32_t)(wifi_nvram_43455_len / 64u);

			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, nv_lo,  NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, nv_mid, NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, nv_hi,  NULL);

			rc_nvram_w = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1,
				/*reg_addr=*/nv_f1_offset,
				/*block_count=*/nv_blocks,
				/*block_size=*/64u, wifi_nvram_43455);
		}

		/* Snapshot SOCRAM[0..63] BEFORE release — should match source
		 * firmware byte-identically. */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x80u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x19u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x00u, NULL);
		rc_r_pre = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1,
			/*reg_addr=*/0u, /*block_count=*/1u, /*block_size=*/64u, pre_buf);

		/* #91 trivial test: counter pre-state at CR4TINY_COUNTER_ADDR
		 * (0x199000 = blob offset 0x1000, which is 0 => expect 0). Same
		 * 0x198000 window as the SOCRAM snapshot; F1 offset 0x1000. */
		{
			uint32_t c0[4] = {0}, c1[4] = {0}, c2[4] = {0}, c3[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1000u, 0u, c0);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1001u, 0u, c1);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1002u, 0u, c2);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1003u, 0u, c3);
			cnt_pre[0] = (uint8_t)(c0[0] & 0xffu);
			cnt_pre[1] = (uint8_t)(c1[0] & 0xffu);
			cnt_pre[2] = (uint8_t)(c2[0] & 0xffu);
			cnt_pre[3] = (uint8_t)(c3[0] & 0xffu);
			rc_cnt_pre = 0;
		}

		/* brcmf_sdio_buscore_activate step 0 (was MISSING — suspect 3b):
		 * clear the SDIO-DEV core intstatus (write 0xFFFFFFFF) BEFORE the
		 * reset vector, exactly as brcmfmac does. Uses the EROM SDIO_DEV
		 * base (0x18004000) + intstatus@0x20, NOT the old 0x18005000 guess. */
		diag_bpWrite32(sdhci, sdio_core + 0x20u, 0xFFFFFFFFu);

		/* brcmfmac CR4 activation, step 1: write the firmware reset
		 * vector (first word of the blob) to chip-internal address 0.
		 * The low 32 bytes of address 0 are a writable vector-table
		 * overlay; the CR4 fetches its reset vector from here when it
		 * leaves reset. */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x00u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x00u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x00u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x0u, fw_img[0], NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1u, fw_img[1], NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2u, fw_img[2], NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x3u, fw_img[3], NULL);

		/* Read addr 0 back to VERIFY the rstvec landed at TRUE backplane
		 * address 0. A mismatch means the addr-0 write is landing in
		 * TCM/0x198000 (SBADDR window / address-mask bug) and the CR4
		 * fetches a garbage reset vector. */
		{
			uint32_t v0[4] = {0}, v1[4] = {0}, v2[4] = {0}, v3[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x0u, 0u, v0);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1u, 0u, v1);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x2u, 0u, v2);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x3u, 0u, v3);
			rstvec_rb[0] = (uint8_t)(v0[0] & 0xffu);
			rstvec_rb[1] = (uint8_t)(v1[0] & 0xffu);
			rstvec_rb[2] = (uint8_t)(v2[0] & 0xffu);
			rstvec_rb[3] = (uint8_t)(v3[0] & 0xffu);
		}

		/* Re-window to ARM-CR4 wrapper window 0x18100000:
		 *   F1 0x2408 = chip-internal 0x18102408 = BCMA_IOCTL
		 *   F1 0x2800 = chip-internal 0x18102800 = BCMA_RESET_CTL */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x00u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x10u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x18u, NULL);

		/* Read IOCTL pre (POR observed 0x21 = CPUHALT|CLK). */
		(void)diag_sdioCmd52(sdhci, 0, 1, 0x2408u, 0u, rc_pre_resp);

		/* CR4-identity cross-check: read IOCTL at BOTH candidate ARM-wrapper
		 * windows (0x18102408 = the one we release, 0x18103408 = the other)
		 * so a dead-counter result can be attributed to the right half of
		 * the tree. The true CR4 exposes the CPUHALT bit (0x20). */
		{
			uint32_t w2[4] = {0}, w3[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x2408u, 0u, w2);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x3408u, 0u, w3);
			ioctl_w2 = w2[0] & 0xffu;
			ioctl_w3 = w3[0] & 0xffu;
		}

		/* brcmfmac CR4 activation, step 2: full AXI resetcore toggle,
		 * resetcore(core, prereset=CPUHALT(0x20), reset=0, postreset=0):
		 *   coredisable: IOCTL=0x23; RESET_CTL=0x01; IOCTL=0x03
		 *   deassert:    RESET_CTL=0 (poll until clear)
		 *   finalize:    IOCTL=0x01 (CLK only, CPU runs) */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2408u, 0x23u, NULL);   /* IOCTL CPUHALT|FGC|CLK */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2800u, 0x01u, NULL);   /* RESET_CTL assert */
		(void)diag_sdioCmd52(sdhci, 0, 1, 0x2800u, 0u, NULL);      /* readback settle */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2408u, 0x03u, NULL);   /* IOCTL FGC|CLK (reset=0) */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2800u, 0x00u, NULL);   /* RESET_CTL deassert */
		for (i = 0; i < 50; ++i) {
			uint32_t rcv[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x2800u, 0u, rcv);
			if ((rcv[0] & 0x01u) == 0u) {
				break;
			}
			usleep(1000);
		}
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x2408u, 0x01u, NULL);   /* IOCTL CLK (CPU runs) */

		/* Post-release SDIO handshake (brcmf_sdio_bus_init): once the CR4
		 * is running, enable Function 2 (SDPCM data channel) via CCCR
		 * IOEN bit 2 (0x04) and wait for F2-ready in CCCR IOR bit 2. */
		{
			uint32_t ioen_resp[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 0, 0x02u, 0u, ioen_resp);
			(void)diag_sdioCmd52(sdhci, 1, 0, 0x02u,
				(uint8_t)((ioen_resp[0] | 0x04u) & 0xffu), NULL);  /* IOEN F2 */
			for (i = 0; i < 500; ++i) {
				uint32_t ior_resp[4] = {0};
				(void)diag_sdioCmd52(sdhci, 0, 0, 0x03u, 0u, ior_resp);
				f2_ready = (uint8_t)(ior_resp[0] & 0xffu);
				if ((f2_ready & 0x04u) != 0u) {
					f2_ready_iters = i;
					break;
				}
				usleep(2000);
			}
		}

		usleep(300 * 1000);  /* firmware init: NVRAM parse + chip-self-test */

		/* Read IOCTL post (expect 0x01 = CLK only, CPU running). */
		(void)diag_sdioCmd52(sdhci, 0, 1, 0x2408u, 0u, rc_post_resp);

		/* Re-window to SOCRAM and capture post-release snapshot. */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x80u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x19u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x00u, NULL);
		rc_r_post = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1,
			/*reg_addr=*/0u, /*block_count=*/1u, /*block_size=*/64u, post_buf);

		/* #91 trivial test: counter POST-release at CR4TINY_COUNTER_ADDR.
		 * Same 0x198000 window; F1 offset 0x1000. The counter free-runs at
		 * ~MHz, so we do NOT expect the exact seed magic back -- we expect a
		 * value that (a) differs from the known-zero pre-state and (b) keeps
		 * CLIMBING between two reads a short delay apart. read2 >> read1 is
		 * unambiguous live execution (kills any static-artifact hypothesis in
		 * one boot). The 0xC0/0xC1 top byte corroborates our seed. */
		{
			uint32_t c0[4] = {0}, c1[4] = {0}, c2[4] = {0}, c3[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1000u, 0u, c0);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1001u, 0u, c1);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1002u, 0u, c2);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1003u, 0u, c3);
			cnt_post[0] = (uint8_t)(c0[0] & 0xffu);
			cnt_post[1] = (uint8_t)(c1[0] & 0xffu);
			cnt_post[2] = (uint8_t)(c2[0] & 0xffu);
			cnt_post[3] = (uint8_t)(c3[0] & 0xffu);
			rc_cnt_post = 0;

			usleep(50 * 1000); /* let the free-running counter advance */

			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1000u, 0u, c0);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1001u, 0u, c1);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1002u, 0u, c2);
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1003u, 0u, c3);
			cnt_post2[0] = (uint8_t)(c0[0] & 0xffu);
			cnt_post2[1] = (uint8_t)(c1[0] & 0xffu);
			cnt_post2[2] = (uint8_t)(c2[0] & 0xffu);
			cnt_post2[3] = (uint8_t)(c3[0] & 0xffu);
			rc_cnt_post2 = 0;
		}

		/* fw-execution disambiguation (#91): SOCRAM[0..63] is entry/vector
		 * code a running fw need not modify, so it is a weak "alive" tell.
		 * Scan several points spread across the loaded image and compare
		 * the post-release on-chip bytes to the source blob. ANY changed
		 * point => the CR4 IS executing; zero change everywhere => fw
		 * genuinely not running. Skipped in trivial mode: the scan offsets
		 * exceed the small trivial blob (the counter readback is the tell). */
		if (!g_trivial_mode) {
			static const uint32_t scan_off[6] = {
				0x02000u, 0x10000u, 0x30000u, 0x60000u, 0x90000u, 0x9C000u
			};
			unsigned s;
			int k;
			scan_changed_pts = 0;
			for (s = 0u; s < 6u; ++s) {
				uint32_t a = 0x198000u + scan_off[s];
				uint8_t lo = (uint8_t)(((a >> 15) & 1u) ? 0x80u : 0x00u);
				uint8_t mid = (uint8_t)((a >> 16) & 0xffu);
				uint8_t hi = (uint8_t)((a >> 24) & 0xffu);
				uint32_t f1 = a & 0x7FFFu;
				int d = 0;
				(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, lo, NULL);
				(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, mid, NULL);
				(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, hi, NULL);
				scan_rc[s] = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1,
					/*reg_addr=*/f1, /*block_count=*/1u, /*block_size=*/64u,
					scan_buf);
				if (scan_rc[s] == 0) {
					for (k = 0; k < 64; ++k) {
						if (scan_buf[k] != wifi_fw_43455[scan_off[s] + (uint32_t)k]) {
							++d;
						}
					}
					scan_diff[s] = d;
					if (d > 0) {
						++scan_changed_pts;
					}
				}
				else {
					scan_diff[s] = -1;
				}
			}
		}

		/* Firmware-running probes:
		 * 1. CHIPCLKCSR (F1 0x1000E): HT_AVAIL (bit 7, 0x80) goes high
		 *    once the booted firmware requests the HT backplane clock.
		 * 2. SDHCI CARD_INTR (INT_STATUS bit 8): the chip asserts its SDIO
		 *    interrupt line when firmware has a mailbox message.
		 * 3. SOCRAM trailer at chip-internal 0x237FFC (the NVRAM
		 *    length-magic word): firmware overwrites this after parsing
		 *    NVRAM. */
		for (i = 0; i < 8; ++i) {
			uint32_t ccsr[4] = {0};
			(void)diag_sdioCmd52(sdhci, 0, 1, 0x1000Eu, 0u, ccsr);
			chipclk_samples[i] = (uint8_t)(ccsr[0] & 0xffu);
			usleep(30 * 1000);
		}

		card_intr = (*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS)
			>> 8) & 1u;

		/* SOCRAM tail trailer: window 19 (0x230000), F1 offset 0x7FF0
		 * = chip-internal 0x237FF0. Read 16 bytes ending at 0x237FFF. */
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, 0x00u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, 0x23u, NULL);
		(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, 0x00u, NULL);
		rc_tail = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1,
			/*reg_addr=*/0x7FF0u, /*block_count=*/1u, /*block_size=*/16u,
			socram_tail);

		/* DEFINITIVE fw-ready probe: read the SDIO-DEV core's
		 * tohostmailboxdata (core base + 0x4C). brcmfmac/WHD treat
		 * HMB_DATA_FWREADY (0x0008) here as THE "firmware booted" signal.
		 * FIXED: use the EROM-enumerated SDIO_DEV base (0x18004000), not the
		 * old 0x18005000 guess (off by 0x1000 -> was reading 0x1800504C). */
		hmb_data = diag_bpRead32(sdhci, sdio_core + 0x4Cu);

		/* #91: read sdpcm_shared @ ram_top-4 (fw overwrites the NVRAM token
		 * with it once booted) -> the fw console ring buffer. Real fw only. */
		if (!g_trivial_mode) {
			diag_readShared(sdhci, ram_size);
		}

		/* The escan itself is deferred to wifi_scan(), run per client request
		 * against the SDIO controller left mapped in g_sdhci below. This is the
		 * split point: everything above is one-shot bring-up; the scan is not. */
	}

	/* Keep the SDIO controller mapped so wifi_scan() can drive it later; the
	 * GPIO routing is finished, so that page can be released. */
	g_sdhci = (volatile uint8_t *)sdhci_page;
	g_sdio_core = sdio_core;
	munmap(gpio_page, _PAGE_SIZE);

	r = snprintf(buf + off, cap - off,
		"enum: CMD5=%d/%d C=%d RCA=0x%04x CMD7=%d IORDY=0x%02x rdy=%d\n",
		rc_ocr, rc_claim,
		(int)((claim_resp[0] >> 31) & 1u),
		(unsigned)rca, rc_sel,
		(unsigned)(iordy_resp[0] & 0xff), rdy_iters);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}
	(void)rc_iordy;

	r = snprintf(buf + off, cap - off,
		"fw_load: staged %u bytes across %d windows  HS=%d  worst rc_w=%d\n",
		bytes_written, window_idx, rc_hs, worst_rc_w);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"nvram: %zu bytes -> chip 0x%06x (ram-top 0x%06x from bankinfo)  rc_nvram_w=%d  HT_clk_csr=0x%02x (HT_AVAIL=0x80)\n",
		wifi_nvram_43455_len,
		(unsigned)(((ram_size != 0u) ? (0x198000u + ram_size) : 0x238000u) - (uint32_t)wifi_nvram_43455_len),
		(unsigned)((ram_size != 0u) ? (0x198000u + ram_size) : 0x238000u),
		rc_nvram_w, (unsigned)ht_clk_csr);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"ARMCR4 IoCtrl pre=0x%02x  post=0x%02x  (expect pre=0x21 CPUHALT+clk, post=0x01 clk-only)\n",
		(unsigned)(rc_pre_resp[0] & 0xff),
		(unsigned)(rc_post_resp[0] & 0xff));
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"rstvec@addr0 readback: %02x %02x %02x %02x  vs fw[0..3]: %02x %02x %02x %02x  -> %s\n",
		rstvec_rb[0], rstvec_rb[1], rstvec_rb[2], rstvec_rb[3],
		fw_img[0], fw_img[1], fw_img[2], fw_img[3],
		(rstvec_rb[0] == fw_img[0] && rstvec_rb[1] == fw_img[1] &&
			rstvec_rb[2] == fw_img[2] && rstvec_rb[3] == fw_img[3])
			? "MATCH (vector placed at true backplane 0)"
			: "MISMATCH (addr-0 write landed elsewhere -- CR4 fetches garbage!)");
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	if (rc_r_pre == 0 && rc_r_post == 0) {
		pre_match = 0;
		post_match = 0;
		diff_count = 0;
		for (i = 0; i < (int)sizeof(pre_buf); ++i) {
			if (pre_buf[i] == fw_img[i]) ++pre_match;
			if (post_buf[i] == fw_img[i]) ++post_match;
			if (pre_buf[i] != post_buf[i]) ++diff_count;
		}
		r = snprintf(buf + off, cap - off,
			"SOCRAM[0..63] pre vs fw: %d/64 match (load check)\n",
			pre_match);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		r = snprintf(buf + off, cap - off,
			"SOCRAM[0..63] post vs fw: %d/64 match  pre-vs-post diff: %d/64 bytes\n",
			post_match, diff_count);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		r = snprintf(buf + off, cap - off,
			"  fw[0..7]   %02x %02x %02x %02x %02x %02x %02x %02x\n"
			"  pre[0..7]  %02x %02x %02x %02x %02x %02x %02x %02x\n"
			"  post[0..7] %02x %02x %02x %02x %02x %02x %02x %02x\n",
			fw_img[0], fw_img[1], fw_img[2], fw_img[3],
			fw_img[4], fw_img[5], fw_img[6], fw_img[7],
			pre_buf[0], pre_buf[1], pre_buf[2], pre_buf[3],
			pre_buf[4], pre_buf[5], pre_buf[6], pre_buf[7],
			post_buf[0], post_buf[1], post_buf[2], post_buf[3],
			post_buf[4], post_buf[5], post_buf[6], post_buf[7]);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		if (diff_count > 0) {
			r = snprintf(buf + off, cap - off,
				"  -> SOCRAM CHANGED after release: firmware appears to be running\n");
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
		}
		else {
			r = snprintf(buf + off, cap - off,
				"  -> SOCRAM unchanged: firmware may not have started (need NVRAM?)\n");
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
		}
	}

	if (scan_changed_pts >= 0) {
		r = snprintf(buf + off, cap - off,
			"image-scan post vs fw (changed bytes/64 @ +off): "
			"+0x02000=%d +0x10000=%d +0x30000=%d +0x60000=%d +0x90000=%d +0x9C000=%d\n",
			scan_diff[0], scan_diff[1], scan_diff[2], scan_diff[3], scan_diff[4], scan_diff[5]);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		r = snprintf(buf + off, cap - off,
			"  -> %d/6 points changed => %s\n",
			scan_changed_pts,
			(scan_changed_pts > 0)
				? "CR4 IS EXECUTING (writing memory) -- gate is observability/early-stall"
				: "no memory writes anywhere -- fw genuinely not running (chase rstvec/activate)");
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
	}

	r = snprintf(buf + off, cap - off,
		"F2 enable: IOR=0x%02x ready=%s @iter=%d (F2_RDY=bit2 0x04)\n",
		f2_ready, ((f2_ready & 0x04u) != 0u) ? "YES" : "no", f2_ready_iters);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"SDIOD tohostmailboxdata@0x%08x=0x%08x -> %s (HMB_DATA_FWREADY=0x0008; SDIOD base from EROM)\n",
		(unsigned)(sdio_core + 0x4Cu), hmb_data,
		((hmb_data & 0x0008u) != 0u) ? "FWREADY set -- FIRMWARE BOOTED!"
			: ((hmb_data == 0xffffffffu || hmb_data == 0u) ? "0/0xff (no fw signal, or wrong SDIOD base)"
				: "nonzero but no FWREADY bit"));
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"CHIPCLKCSR poll: %02x %02x %02x %02x %02x %02x %02x %02x (HT_AVAIL=bit7 0x80)\n",
		chipclk_samples[0], chipclk_samples[1], chipclk_samples[2],
		chipclk_samples[3], chipclk_samples[4], chipclk_samples[5],
		chipclk_samples[6], chipclk_samples[7]);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	r = snprintf(buf + off, cap - off,
		"SDHCI CARD_INTR=%u  SOCRAM-tail rc=%d  trailer[12..15]=%02x %02x %02x %02x (blob trailer=%02x %02x %02x %02x)\n",
		card_intr, rc_tail,
		socram_tail[12], socram_tail[13], socram_tail[14], socram_tail[15],
		wifi_nvram_43455[wifi_nvram_43455_len - 4], wifi_nvram_43455[wifi_nvram_43455_len - 3],
		wifi_nvram_43455[wifi_nvram_43455_len - 2], wifi_nvram_43455[wifi_nvram_43455_len - 1]);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	{
		int fw_alive = 0;
		for (i = 0; i < 8; ++i) {
			if ((chipclk_samples[i] & 0x80u) != 0u) {
				fw_alive = 1;
			}
		}
		if (card_intr != 0u) {
			fw_alive = 1;
		}
		r = snprintf(buf + off, cap - off,
			"  -> fw_alive=%d %s\n", fw_alive,
			fw_alive ? "(HT_AVAIL or CARD_INTR asserted -- firmware booted!)"
				: "(no HT_AVAIL / no CARD_INTR -- firmware not confirmed running)");
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
	}

	/* #91 EROM core enumeration: the chip's own answer for every core base /
	 * wrapper, replacing the hardcoded hypotheses. */
	if (g_erom_ncores > 0) {
		uint32_t cr4b = diag_eromCoreBase(BCMA_ID_ARM_CR4);
		uint32_t cr4w = diag_eromCoreWrap(BCMA_ID_ARM_CR4);
		uint32_t sdiob = diag_eromCoreBase(BCMA_ID_SDIO_DEV);
		uint32_t socb = diag_eromCoreBase(BCMA_ID_INTERNAL_MEM);
		int ci;
		r = snprintf(buf + off, cap - off,
			"EROM: eromptr=0x%08x  cores=%d\n"
			"  ARM_CR4(0x83E): core=0x%08x wrap=0x%08x (release-wrap hyp was 0x18102000 -> %s)\n"
			"  SDIO_DEV(0x829): core=0x%08x (mailbox hyp was 0x18005000 -> %s)\n"
			"  INTERNAL_MEM/SOCRAM(0x80E): core=0x%08x (0=absent: 43455 RAM is CR4 TCM)\n"
			"  CR4 TCM ramsize=0x%08x -> ram-top=0x%08x (hardcoded NVRAM top was 0x238000 -> %s)\n",
			(unsigned)g_erom_ptr, g_erom_ncores,
			(unsigned)cr4b, (unsigned)cr4w,
			(cr4w == 0x18102000u) ? "MATCH" : "DIFFERS",
			(unsigned)sdiob,
			(sdiob == 0x18005000u) ? "MATCH" : "DIFFERS(fixed)",
			(unsigned)socb,
			(unsigned)ram_size, (unsigned)(0x198000u + ram_size),
			((0x198000u + ram_size) == 0x238000u) ? "MATCH" : "DIFFERS");
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		for (ci = 0; ci < g_erom_ncores; ++ci) {
			r = snprintf(buf + off, cap - off,
				"  core[%d] id=0x%03x rev=%u base=0x%08x wrap=0x%08x\n",
				ci, (unsigned)g_erom_id[ci], (unsigned)g_erom_rev[ci],
				(unsigned)g_erom_base[ci], (unsigned)g_erom_wrap[ci]);
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
		}
	}
	else {
		r = snprintf(buf + off, cap - off,
			"EROM: walk failed/skipped (ncores=%d, eromptr=0x%08x)\n",
			g_erom_ncores, (unsigned)g_erom_ptr);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
	}

	/* #91 CR4-identity cross-check + (when active) the trivial-program test. */
	r = snprintf(buf + off, cap - off,
		"CR4-identity: IOCTL@0x18102408=0x%02x IOCTL@0x18103408=0x%02x "
		"(CPUHALT=0x20; we release 0x18102000)\n",
		(unsigned)ioctl_w2, (unsigned)ioctl_w3);
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	/* The probe's trivial-counter self-test (argv "trivial") is dropped from the
	 * resident driver. The counter reads above are kept verbatim — they are part
	 * of the proven SDIO command sequence — but their values are unused here. */
	(void)cnt_pre;
	(void)cnt_post;
	(void)cnt_post2;
	(void)rc_cnt_pre;
	(void)rc_cnt_post;
	(void)rc_cnt_post2;

	/* #91 sdpcm_shared + firmware console (real fw only). */
	if (!g_trivial_mode && g_shared_valid >= 0) {
		if (g_shared_valid == 1) {
			r = snprintf(buf + off, cap - off,
				"sdpcm_shared @0x%08x VALID (word@ram_top-4=0x%08x, fw booted+overwrote NVRAM token)\n"
				"  flags=0x%08x (ver=%u trap=%s assert_built=%s assert=%s) trap_addr=0x%08x\n"
				"  console_addr=0x%08x log_buf=0x%08x bufsize=%u idx=%u  (console %d bytes below)\n",
				(unsigned)g_sh_addr, (unsigned)g_sh_word,
				(unsigned)g_sh_flags, (unsigned)(g_sh_flags & 0xffu),
				(g_sh_flags & 0x0400u) ? "YES" : "no",
				(g_sh_flags & 0x0100u) ? "yes" : "no",
				(g_sh_flags & 0x0200u) ? "FIRED" : "no",
				(unsigned)g_trap_addr,
				(unsigned)g_console_addr, (unsigned)g_log_buf,
				(unsigned)g_log_bufsize, (unsigned)g_log_idx, g_console_len);
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
			if (g_console_len > 0) {
				int ci;
				r = snprintf(buf + off, cap - off, "----- FW CONSOLE -----\n");
				if (r > 0 && (size_t)r < cap - off) {
					off += r;
				}
				for (ci = 0; ci < g_console_len && (size_t)(off + 2) < cap; ++ci) {
					char c = g_console[ci];
					if (c == '\n' || (c >= 0x20 && c < 0x7f)) {
						buf[off++] = c;
					}
					else if (c != '\0') {
						buf[off++] = '.';
					}
				}
				if ((size_t)(off + 24) < cap) {
					r = snprintf(buf + off, cap - off, "\n----- END CONSOLE -----\n");
					if (r > 0 && (size_t)r < cap - off) {
						off += r;
					}
				}
			}
		}
		else {
			r = snprintf(buf + off, cap - off,
				"sdpcm_shared: word@ram_top-4=0x%08x INVALID (NVRAM-token pattern => fw not booted / no shared)\n",
				(unsigned)g_sh_word);
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
		}
	}

	/* #91 WiFi scan report. */
	if (g_scan_ran) {
		int ap;
		r = snprintf(buf + off, cap - off,
			"WiFi SCAN: event_msgs rc=%d  clmload(%d chunks, last rc=%d)  infra rc=%d  UP rc=%d  chanspecs=%d  mpc rc=%d  escan rc=%d (tries=%d)\n"
			"  GET_VAR cur_etheraddr rc=%d valid=%d MAC=%02x:%02x:%02x:%02x:%02x:%02x\n"
			"  chan1 frames=%d  escan-events(type69)=%d  APs=%d  done_status=%d\n",
			g_scan_em_rc, g_clm_chunks, g_clm_last_rc, g_scan_infra_rc, g_scan_up_rc,
			(int)g_chanspecs_count, g_scan_mpc_rc, g_scan_escan_rc, g_scan_escan_tries,
			g_mac_rc, g_mac_valid,
			g_mac[0], g_mac[1], g_mac[2], g_mac[3], g_mac[4], g_mac[5],
			g_scan_evt_total, g_scan_escan_events, g_scan_ap_count, g_scan_done_status);
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
		for (ap = 0; ap < g_scan_ap_count; ++ap) {
			r = snprintf(buf + off, cap - off,
				"  AP[%d] %02x:%02x:%02x:%02x:%02x:%02x  ch=%u  rssi=%d dBm  ssid(%u)=\"%s\"\n",
				ap,
				g_scan_aps[ap].bssid[0], g_scan_aps[ap].bssid[1], g_scan_aps[ap].bssid[2],
				g_scan_aps[ap].bssid[3], g_scan_aps[ap].bssid[4], g_scan_aps[ap].bssid[5],
				(unsigned)g_scan_aps[ap].chan, (int)g_scan_aps[ap].rssi,
				(unsigned)g_scan_aps[ap].ssid_len, g_scan_aps[ap].ssid);
			if (r > 0 && (size_t)r < cap - off) {
				off += r;
			}
		}
		r = snprintf(buf + off, cap - off,
			"  -> %s\n",
			(g_scan_ap_count > 0)
				? "SCAN FOUND APs -- the radio works! (SSID/RSSI/channel above)"
				: "no APs parsed (see rc/event counts)");
		if (r > 0 && (size_t)r < cap - off) {
			off += r;
		}
	}

	r = snprintf(buf + off, cap - off, ".\n");
	if (r > 0 && (size_t)r < cap - off) {
		off += r;
	}

	/* The probe emitted this telemetry as its whole reason to exist; here it is
	 * one-shot bring-up logging on the console. */
	printf("%.*s", off, buf);
	fflush(stdout);
	return 0;
}


/* ---- WiFi scan --------------------------------------------------------- */
/* Run one active escan against the controller brought up by wifi_bringup(), and
 * render the discovered APs into `out` as text, one line per AP:
 *   SSID  BSSID(xx:xx:..)  RSSI(dBm)  ch<N>
 * Returns the number of bytes written. diag_wifiScan() is reused verbatim: it
 * fills the g_scan_aps[] globals (it does not print), so this wrapper resets the
 * result accumulators, runs it, then formats the globals. */
static int wifi_scan(char *out, int cap)
{
	int off = 0, ap, r;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}

	/* Reset the scan-result accumulators so repeated scans do not accrete
	 * (diag_wifiScan appends into g_scan_aps[] and caps at SCAN_MAX_APS). */
	g_scan_ap_count = 0;
	g_scan_evt_total = 0;
	g_scan_escan_events = 0;
	g_scan_done_status = -1;

	diag_wifiScan(g_sdhci, g_sdio_core);

	for (ap = 0; ap < g_scan_ap_count && off < cap; ++ap) {
		const char *ssid = (g_scan_aps[ap].ssid_len > 0u) ? g_scan_aps[ap].ssid : "<hidden>";
		r = snprintf(out + off, (size_t)(cap - off),
			"%s  %02x:%02x:%02x:%02x:%02x:%02x  %ddBm  ch%u\n",
			ssid,
			g_scan_aps[ap].bssid[0], g_scan_aps[ap].bssid[1], g_scan_aps[ap].bssid[2],
			g_scan_aps[ap].bssid[3], g_scan_aps[ap].bssid[4], g_scan_aps[ap].bssid[5],
			(int)g_scan_aps[ap].rssi, (unsigned)g_scan_aps[ap].chan);
		if (r > 0 && r < cap - off) {
			off += r;
		}
	}

	if (g_scan_ap_count == 0 && off < cap) {
		r = snprintf(out + off, (size_t)(cap - off),
			"(no APs found; done_status=%d escan_rc=%d events=%d)\n",
			g_scan_done_status, g_scan_escan_rc, g_scan_escan_events);
		if (r > 0 && r < cap - off) {
			off += r;
		}
	}

	return off;
}


/* WLC ioctl command numbers (brcmu; == BRCMF_C_*). */
#define WLC_SET_INFRA 20u
#define WLC_SET_SSID  26u

/* Exercise the join (association) CONTROL PATH against `ssid`: enable the join
 * events, bring the radio up, set infrastructure mode, WLC_SET_SSID, then drain
 * the WLC_E_* association events reporting each type+status. Run after a scan
 * (which loads the CLM channel data). Against a non-existent test SSID this
 * reports SET_SSID + association-failure events, proving the join machinery
 * end-to-end. NOTE: this is an OPEN-network join; WPA2 key setup (wsec /
 * wpa_auth / wsec_pmk) and a real-network association with a real PSK are the
 * documented owner-triggered follow-on (needs a real AP for strong validation).
 * Event framing + WLC numbers verified vs the brcmfmac primary source. */
static int wifi_join(const char *ssid, char *out, int cap)
{
	uint32_t reqid = 0x4000u, i, rxlen, slen = 0u;
	uint8_t seq = 0x40u, emask[16], ssidbuf[36], infra[4];
	int off = 0, t, r, saw_set_ssid = 0, saw_assoc = 0, saw_link = 0;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}

	/* Enable the association events (keep escan bit 69): SET_SSID(0), JOIN(1),
	 * AUTH(3), ASSOC(7), LINK(16). */
	for (i = 0u; i < 16u; ++i) {
		emask[i] = 0u;
	}
	emask[0] = (uint8_t)((1u << 0) | (1u << 1) | (1u << 3) | (1u << 7));
	emask[2] = (uint8_t)(1u << 0); /* event 16 -> byte 2 bit 0 */
	emask[8] = 0x20u;              /* event 69 (escan) */
	(void)diag_iovar(g_sdhci, g_sdio_core, 1, "event_msgs", emask, 16u, NULL, 0u, &rxlen, reqid++, seq++);

	(void)diag_bcdcCmd(g_sdhci, g_sdio_core, 1, WLC_UP_CMD, NULL, 0u, NULL, 0u, &rxlen, reqid++, seq++);
	infra[0] = 1u; infra[1] = 0u; infra[2] = 0u; infra[3] = 0u;
	(void)diag_bcdcCmd(g_sdhci, g_sdio_core, 1, WLC_SET_INFRA, infra, 4u, NULL, 0u, &rxlen, reqid++, seq++);

	/* WLC_SET_SSID: wlc_ssid_t { le32 SSID_len; u8 SSID[32] } -> triggers the join. */
	while (ssid[slen] != '\0' && slen < 32u) {
		slen++;
	}
	for (i = 0u; i < sizeof(ssidbuf); ++i) {
		ssidbuf[i] = 0u;
	}
	ssidbuf[0] = (uint8_t)slen;
	for (i = 0u; i < slen; ++i) {
		ssidbuf[4u + i] = (uint8_t)ssid[i];
	}
	(void)diag_bcdcCmd(g_sdhci, g_sdio_core, 1, WLC_SET_SSID, ssidbuf, 36u, NULL, 0u, &rxlen, reqid++, seq++);

	off += snprintf(out + off, (size_t)(cap - off), "join \"%.*s\": association events:\n", (int)slen, ssid);

	/* Drain WLC_E_* association events for ~6 s (same channel-1 event framing as
	 * the escan reader: sdoff=buf[7], ehdr=sdoff+4+4*buf[sdoff+3], h_proto@+12,
	 * event_type@+28, status@+32, all big-endian). */
	for (t = 0; t < 600 && off < cap; ++t) {
		uint16_t len;
		uint8_t chan;
		int fr;
		uint32_t sdoff, ehdr, etype, status;

		fr = diag_f2RecvFrame(g_sdhci, g_rxf, &len, &chan);
		if (fr == 1) {
			usleep(10000);
			continue;
		}
		if (fr < 0) {
			usleep(5000);
			continue;
		}
		{
			uint32_t st = diag_bpRead32(g_sdhci, g_sdio_core + 0x20u);
			if (st != 0u && st != 0xffffffffu) {
				diag_bpWrite32(g_sdhci, g_sdio_core + 0x20u, st);
			}
		}
		if (chan != 1u) {
			continue;
		}
		sdoff = g_rxf[7];
		if (sdoff + 4u > len) {
			continue;
		}
		ehdr = sdoff + 4u + 4u * (uint32_t)g_rxf[sdoff + 3u];
		if (ehdr + 40u > (uint32_t)len) {
			continue;
		}
		if (diag_be16(g_rxf + ehdr + 12u) != 0x886Cu) {
			continue; /* not an event (h_proto != ETH_P_LINK_CTL) */
		}
		etype = diag_be32(g_rxf + ehdr + 28u);
		status = diag_be32(g_rxf + ehdr + 32u);
		if (etype == 0u || etype == 1u || etype == 3u || etype == 5u ||
			etype == 6u || etype == 7u || etype == 16u) {
			const char *nm = (etype == 0u) ? "SET_SSID" : (etype == 1u) ? "JOIN" :
				(etype == 3u) ? "AUTH" : (etype == 5u) ? "DEAUTH" :
				(etype == 6u) ? "DEAUTH_IND" : (etype == 7u) ? "ASSOC" : "LINK";
			r = snprintf(out + off, (size_t)(cap - off),
				"  WLC_E_%s (type=%u) status=%u\n", nm, (unsigned)etype, (unsigned)status);
			if (r > 0 && r < cap - off) {
				off += r;
			}
			if (etype == 0u) { saw_set_ssid = 1; }
			if (etype == 7u) { saw_assoc = 1; }
			if (etype == 16u) {
				saw_link = 1;
				if (status == 0u) {
					break; /* link up */
				}
			}
		}
	}
	if (off < cap) {
		r = snprintf(out + off, (size_t)(cap - off),
			"join machinery ran: SET_SSID=%d ASSOC=%d LINK=%d "
			"(test SSID -> failure events expected; proves the control path)\n",
			saw_set_ssid, saw_assoc, saw_link);
		if (r > 0 && r < cap - off) {
			off += r;
		}
	}
	return off;
}


/* ---- /dev/wifi message loop -------------------------------------------- */
/* Offset-aware slice of the most recent scan result (cf. rpi4-gpio's read). */
static int wifi_readResp(off_t offs, char *dst, size_t size)
{
	if ((g_resp_len <= 0) || (offs < 0) || (offs >= (off_t)g_resp_len)) {
		return 0;
	}
	if (size > (size_t)(g_resp_len - offs)) {
		size = (size_t)(g_resp_len - offs);
	}
	memcpy(dst, g_resp + offs, size);
	return (int)size;
}


static void wifi_thread(void *arg)
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
				msg.o.err = wifi_readResp(msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				/* write("scan") triggers a fresh escan into g_resp; write("join
				 * <ssid>") exercises the association control path (run after a
				 * scan, which loads the CLM channel data). A client read()s the
				 * result. Any other payload is accepted but ignored. */
				if (msg.i.size >= 4 && memcmp(msg.i.data, "scan", 4) == 0) {
					g_resp_len = wifi_scan(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 6 && memcmp(msg.i.data, "join ", 5) == 0) {
					char ssid[33];
					int n = (int)msg.i.size - 5;
					if (n > 32) {
						n = 32;
					}
					memcpy(ssid, (const char *)msg.i.data + 5, (size_t)n);
					ssid[n] = '\0';
					while (n > 0 && (ssid[n - 1] == '\n' || ssid[n - 1] == '\r' || ssid[n - 1] == ' ')) {
						ssid[--n] = '\0';
					}
					g_resp_len = wifi_join(ssid, g_resp, (int)sizeof(g_resp));
				}
				msg.o.err = (int)msg.i.size;
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0600;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void wifi_sigExit(int sig)
{
	(void)sig;
	_exit(0);
}


int main(int argc, char **argv)
{
	int selftest = 0, jointest = 0, ai, rc;
	uint32_t port;
	oid_t dev;
	pid_t pid;

	for (ai = 1; ai < argc; ++ai) {
		if (strcmp(argv[ai], "selftest") == 0) {
			selftest = 1;
		}
		else if (strcmp(argv[ai], "jointest") == 0) {
			jointest = 1;
		}
	}

	if (selftest != 0 || jointest != 0) {
		/* Single-process acceptance harness: bring up, scan once (also loads the
		 * CLM channel data + brings the radio up), print. For jointest, then
		 * exercise the association control path against a non-existent test SSID
		 * (failure events are expected; it proves the join machinery). */
		rc = wifi_bringup();
		if (rc == 0) {
			g_resp_len = wifi_scan(g_resp, (int)sizeof(g_resp));
			printf("rpi4-wifi selftest: scan result (%d bytes):\n%.*s",
				g_resp_len, g_resp_len, g_resp);
			if (jointest != 0) {
				g_resp_len = wifi_join("PHX-JOIN-TEST-NOAP", g_resp, (int)sizeof(g_resp));
				printf("rpi4-wifi jointest:\n%.*s", g_resp_len, g_resp);
			}
		}
		usleep(100 * 1000); /* let the log flush */
		return rc;
	}

	/* Resident daemon: fork so the shell returns once /dev/wifi is up while the
	 * child keeps serving (canonical Phoenix pattern, cf. rpi4-hci). */
	signal(SIGUSR1, wifi_sigExit);
	pid = fork();
	if (pid < 0) {
		printf("rpi4-wifi: fork failed\n");
		return 1;
	}
	if (pid > 0) {
		/* Wait to be signalled by the child once /dev/wifi is up. Generous
		 * fallback: WiFi bring-up (643 KB SDIO fw download + CLM + settle) is
		 * much slower than BT; only reached if the child fails to come up. */
		(void)sleep(120);
		return 1;
	}

	/* child: bring up the radio, register /dev/wifi, then serve forever */
	signal(SIGUSR1, wifi_sigExit);
	(void)setsid();
	rc = wifi_bringup();
	if (rc != 0) {
		printf("rpi4-wifi: bring-up failed (rc=%d)\n", rc);
		return rc;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-wifi: portCreate failed\n");
		return 3;
	}
	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "wifi") < 0) {
		printf("rpi4-wifi: could not create /dev/wifi\n");
		return 4;
	}
	printf("rpi4-wifi: registered /dev/wifi (write \"scan\", then read the AP list)\n");
	fflush(stdout);

	if (beginthread(wifi_thread, 3, g_msgStack, sizeof(g_msgStack),
			(void *)(uintptr_t)port) != EOK) {
		printf("rpi4-wifi: msg thread failed\n");
		return 5;
	}

	kill(getppid(), SIGUSR1);
	for (;;) {
		usleep(1000 * 1000); /* the message thread does the work */
	}
	return 0;
}
