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
#include <time.h>
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
	/* CMD53 byte mode encodes the count in a 9-bit field (below), so 512 is
	 * the hard maximum. A larger request used to wrap that field silently
	 * while BLOCK_SIZE_CNT got the full length -- a host/card length
	 * mismatch that corrupted the transfer with no error reported anywhere.
	 * Refuse instead; callers needing more use block mode (diag_f2Read). */
	if (nbytes > 512u) {
		return -1050;
	}
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
	/* See diag_sdioCmd53ReadByteMode: 9-bit count field, 512 byte maximum. */
	if (nbytes > 512u) {
		return -1051;
	}
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
/* Max F2 frame we can hold: SDPCM(12) + BDC(4) + full-MTU eth(1514) + slack.
 * Was 512, which silently TRUNCATED anything larger. */
#define F2_FRAME_MAX 2048u
#define F2_HDR_LEN   12u     /* SDPCM HW(4) + SW(8): enough to learn the length */
/* SDIO function-2 block size, programmed into FBR 0x210 during bring-up.
 * Block mode is the ONLY way to move more than 512 bytes (see the guards in
 * the byte-mode helpers), so every MTU-sized data frame goes through it.
 * 64 is what brcmfmac uses for this chip. */
#define F2_BLKSZ     64u
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

/* ---- F2 data transfers of ANY length -------------------------------------
 *
 * The byte-mode helpers cap at 512 bytes, which is fine for control frames and
 * for the DHCP exchange but cannot carry a 1514-byte ethernet frame -- the
 * prerequisite for an lwip netif. These wrappers pick the addressing mode by
 * length: byte mode below the cap (the HW-proven path, untouched), block mode
 * above it. Block mode moves whole blocks, so the length is rounded up to
 * F2_BLKSZ; the SDPCM header still tells both sides where the real frame ends,
 * and the TX tail padding is zeroed so nothing stale goes on the air.
 */
static int diag_f2Write(volatile uint8_t *sdhci, const uint8_t *buf, uint32_t len)
{
	uint32_t wlen = (len + 3u) & ~3u; /* pad to 4 for the PIO word loop */

	if (wlen <= 512u) {
		return diag_sdioCmd53WriteByteMode(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR, wlen, buf);
	}
	return diag_sdioCmd53Write(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR,
		(len + F2_BLKSZ - 1u) / F2_BLKSZ, F2_BLKSZ, buf);
}


/* RX transfer mode is a measured throughput decision.
 *
 * Byte mode caps at 512 bytes, so a 1514-byte frame needs three chunk transfers
 * on top of the header read -- four commands per frame, each spinning on SDHCI
 * status in PIO. Block mode moves the whole remainder in ONE command.
 *
 * So one command per frame should be faster. It is not, and the decisive
 * evidence is a COUNTER, not a rate: switching to one block-mode command took
 * rx_err from 1 to 433 in the same test. Block mode must move whole blocks, so
 * it pops ceil(len/64)*64 bytes, reads past the end of the frame and
 * desynchronises the stream -- those errors are that desync. (Throughput also
 * looked worse, but see the WIFI_RX_* note: run-to-run throughput on this link
 * varies 2.6x, so rates alone prove nothing here.)
 *
 * Byte mode pops exactly what is asked for, so a frame is consumed precisely.
 * Keep it. F2_RX_ONE_CMD stays only so the comparison can be re-run. */
#define F2_RX_ONE_CMD 0

static int diag_f2Read(volatile uint8_t *sdhci, uint8_t *buf, uint32_t len)
{
#if !F2_RX_ONE_CMD
	uint32_t done = 0u;
#endif

	if (len <= 512u) {
		return diag_sdioCmd53ReadByteMode(sdhci, 2, /*incr=*/0, IOCTL_F2_ADDR,
			(len + 3u) & ~3u, buf);
	}
#if F2_RX_ONE_CMD
	return diag_sdioCmd53Read(sdhci, 2, /*incr=*/0, IOCTL_F2_ADDR,
		(len + F2_BLKSZ - 1u) / F2_BLKSZ, F2_BLKSZ, buf);
#else
	while (done < len) {
		uint32_t chunk = len - done;
		int rc;

		if (chunk > 512u) {
			chunk = 512u;
		}
		chunk = (chunk + 3u) & ~3u;
		rc = diag_sdioCmd53ReadByteMode(sdhci, 2, /*incr=*/0, IOCTL_F2_ADDR,
			chunk, buf + done);
		if (rc != 0) {
			return rc;
		}
		done += chunk;
	}
	return 0;
#endif
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
	/* phase 1: the SDPCM header only (see brcmf_sdio_readframes) */
	rc = diag_sdioCmd53ReadByteMode(sdhci, 2, /*incr=*/0, IOCTL_F2_ADDR,
		F2_HDR_LEN, buf);
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
	 * drive those indices past the end of the caller's F2_FRAME_MAX buffer. */
	if (len > (uint16_t)F2_FRAME_MAX) {
		diag_sdhciResetCmdDat(sdhci);
		return -32;
	}
	/* phase 2: exactly the rest of this frame (4-byte aligned request) */
	if (len > (uint16_t)F2_HDR_LEN) {
		uint32_t rest = ((uint32_t)len - F2_HDR_LEN + 3u) & ~3u;
		/* diag_f2Read, not the byte-mode helper: a frame over 524 bytes
		 * total needs block mode. Requesting it in byte mode is what
		 * silently corrupted every larger RX frame before. */
		rc = diag_f2Read(sdhci, buf + F2_HDR_LEN, rest);
		if (rc != 0) {
			diag_sdhciResetCmdDat(sdhci);
			return -33;
		}
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
#define WLC_SET_SSID_CMD 26u       /* BRCMF_C_SET_SSID: brcmf_ssid_le (broadcast WPA2 join) */
#define WLC_SET_WSEC_PMK_CMD 268u  /* BRCMF_C_SET_WSEC_PMK: brcmf_wsec_pmk_le (passphrase) */
#define BRCMF_C_GET_PKTCNTS 137u   /* brcmf_pktcnt_le { rx_good, rx_bad, tx_good, tx_bad, rx_ocast } */
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


/* ---- #91 WPA2-PSK join + DHCP over the SDPCM data plane ------------------
 * Ported from the hardware-proven tools/wifi-probe `jointxcnt` flow (join a
 * WPA2 AP, then DISCOVER -> OFFER -> REQUEST -> ACK to BIND a real lease). The
 * BCM43455 is fullmac with an in-dongle supplicant (FWSUP): we set the security
 * params, enable sup_wpa, hand the firmware the ASCII passphrase (it derives the
 * PMK + runs the 4-way handshake itself), issue a broadcast WLC_SET_SSID join,
 * then watch WLC_E_SET_SSID(0)/status0 + WLC_E_PSK_SUP(46)/status6 for success.
 * Mirrors diag_wifiScan's prelude + event demux. Spec:
 * docs/inprogress/2026-08-12-wifi-join-design.md (from Linux brcmfmac).
 *
 * The ordering below was established over many hardware cycles; it is a
 * faithful copy of the probe's proven sequence and must not be reordered. */
static int g_join_ran = 0;
static int g_join_em_rc = -100, g_join_infra_rc = -100, g_join_up_rc = -100;
static int g_join_wsec_rc = -100, g_join_wpaauth_rc = -100, g_join_sup_rc = -100;
static int g_join_pmk_rc = -100, g_join_ssid_rc = -100;
static int g_join_attempts = 0;          /* SET_SSID attempts (retry on no-network) */
/* Set by the `joinwpa` command: associate + 4-way-key only, and leave IP to the
 * caller. An lwip netif must own DHCP itself, so the daemon's built-in exchange
 * has to be skippable. */
static int g_join_skip_dhcp = 0;
static int g_join_setssid_status = -100; /* WLC_E_SET_SSID status (0 = assoc ok) */
static int g_join_psksup_status = -100;  /* WLC_E_PSK_SUP status (6 = 4-way keyed) */
static int g_join_link_up = 0;           /* last WLC_E_LINK flags&0x01 */
static int g_join_evt_total = 0;
static char g_join_ssid[33] = { 0 };     /* credentials, filled by wifi_netup() */
static char g_join_psk[64] = { 0 };

static int g_tx_ran = 0;
static int g_tx_mac_rc = -100;
static int g_tx_rc = -100;
static int g_tx_len = 0;
/* STA MAC read in non-glom mode before any data TX (rxglom breaks our
 * single-frame RX reader, so the MAC must be fetched first). */
static uint8_t g_txmac[6] = { 0 };
static int g_txmac_valid = 0;

/* fw pktcnt: GET the firmware's packet counters (BRCMF_C_GET_PKTCNTS = 137 ->
 * brcmf_pktcnt_le { rx_good, rx_bad, tx_good, tx_bad, rx_ocast }, all le32).
 * Snapshotting them before the DHCP burst localizes where a frame dies:
 * tx_good climbs => the fw TX'd it; tx_bad climbs => the fw TX path failed;
 * both flat => the fw never queued the frame (host->fw ingest gate). Uses the
 * proven BCDC GET-ioctl path, NON-glom so the single-frame control-reply RX
 * stays intact. */
static int g_pktcnt_pre_rc = -100;
static uint32_t g_pktcnt_pre[5] = { 0 };
static int g_dtx_burst = 0;            /* # of data frames actually TX'd */

static int diag_wifiPktcnt(volatile uint8_t *sdhci, uint32_t sdio_core,
	uint32_t out[5], uint32_t reqid, uint8_t seq)
{
	uint8_t buf[32];
	uint32_t rxlen = 0u;
	int rc, i;

	for (i = 0; i < 32; ++i) {
		buf[i] = 0u;
	}
	rc = diag_bcdcCmd(sdhci, sdio_core, /*is_set=*/0, BRCMF_C_GET_PKTCNTS,
		NULL, 20u, buf, sizeof(buf), &rxlen, reqid, seq);
	if (rc >= 0 && rxlen >= 20u) {
		for (i = 0; i < 5; ++i) {
			out[i] = diag_le32(buf + 4 * i);
		}
		return 0;
	}
	return (rc < 0) ? rc : -1;
}

/* RX data-plane: read SDPCM channel-2 DATA frames from the F2 FIFO and parse a
 * BOOTP/DHCP reply out of one. Frame layout (RX):
 *   SDPCM HW[0-3] + SW[4-11] (data_offset = buf[7]) then BDC[buf[7]..] whose
 *   byte 3 = data_offset in 4-byte words (brcmf_proto_bcdc_hdrpull: <<2); the
 *   802.3 frame follows at buf[7] + 4 + (bdc[3]<<2). Then eth(14)+IP(20)+UDP(8)+
 *   BOOTP: op@0(=2 reply), xid@4, yiaddr@16. Non-glom only (single frame RX;
 *   glom RX is a separate de-glom reader we deliberately avoid). */
static int g_rx_offer_seen = 0;        /* a ch2 DHCP reply (op=2) was parsed */
static int g_rx_ch2_frames = 0;        /* ch2 data frames observed */
static uint8_t g_rx_offer_yiaddr[4] = { 0 };
static uint8_t g_rx_offer_msgtype = 0; /* DHCP opt53 (2=OFFER, 5=ACK) */
static uint32_t g_rx_offer_xid = 0u;
static uint8_t g_dhcp_serverid[4] = { 0 }; /* opt54, needed for the REQUEST */
static uint8_t g_rx_want = 0u;         /* required DHCP msgtype (0 = any) */

/* Poll SDPCM ch2 for a BOOTP/DHCP reply whose opt53 == want (want==0 => any).
 * Records msgtype/xid/yiaddr and captures opt54 (server-id). Sets
 * g_rx_offer_seen on a matching reply. */
static void diag_wifiRxDhcp(volatile uint8_t *sdhci, uint32_t sdio_core)
{
	int iter;

	(void)sdio_core;
	g_rx_offer_seen = 0;
	for (iter = 0; iter < 400 && g_rx_offer_seen == 0; ++iter) {
		uint16_t flen = 0u;
		uint8_t chan = 0xffu;
		int rc = diag_f2RecvFrame(sdhci, g_rxf, &flen, &chan);
		if (rc < 0) {
			continue; /* transient transport wedge; diag_f2RecvFrame reset DAT/CMD */
		}
		if (rc == 1) {
			usleep(5000); /* FIFO empty: pace the poll so the ~400-iter window
			               * spans ~2s (the DHCP round-trip + fw->host delivery),
			               * instead of burning through in milliseconds. */
			continue;
		}
		if (chan != 2u) {
			continue; /* control (0) / event (1): not our data path */
		}
		g_rx_ch2_frames++;
		{
			uint32_t sdoff = g_rxf[7];
			uint32_t eth, ip, udp, bootp, opt, end;
			if (sdoff < 12u || sdoff + 4u > flen) {
				continue;
			}
			eth = sdoff + 4u + ((uint32_t)g_rxf[sdoff + 3u] << 2);
			/* diagnostic: dump the first few ch2 frames so we can identify what
			 * the fw forwarded (ARP vs the IPv4 DHCP OFFER) and verify offsets. */
			if (g_rx_ch2_frames <= 3) {
				uint32_t d, dn = (eth + 42u <= flen) ? (eth + 42u) : flen;
				printf("wifi: RX-CH2[%d] flen=%u sdoff=%u bdc=%02x %02x %02x %02x eth@%u etype=%02x%02x bytes[%u..]=",
					g_rx_ch2_frames, flen, sdoff,
					g_rxf[sdoff], g_rxf[sdoff + 1u], g_rxf[sdoff + 2u], g_rxf[sdoff + 3u],
					eth, (eth + 13u < flen) ? g_rxf[eth + 12u] : 0u, (eth + 13u < flen) ? g_rxf[eth + 13u] : 0u, sdoff);
				for (d = sdoff; d < dn; ++d) {
					printf("%02x ", g_rxf[d]);
				}
				printf("\n");
				fflush(stdout);
			}
			/* need eth(14)+IP(20)+UDP(8)+BOOTP(240 min incl magic) in-frame */
			if (eth + 14u + 20u + 8u + 240u > flen) {
				continue;
			}
			/* ethertype IPv4, IP proto UDP, UDP dst port 68 (BOOTP client) */
			if (g_rxf[eth + 12u] != 0x08u || g_rxf[eth + 13u] != 0x00u) {
				continue;
			}
			ip = eth + 14u;
			if (g_rxf[ip + 9u] != 0x11u) {
				continue; /* not UDP */
			}
			udp = ip + 20u;
			if (g_rxf[udp + 2u] != 0x00u || g_rxf[udp + 3u] != 0x44u) {
				continue; /* not ->68 */
			}
			bootp = udp + 8u;
			if (g_rxf[bootp] != 0x02u) {
				continue; /* not a BOOTP reply */
			}
			g_rx_offer_xid = diag_le32(&g_rxf[bootp + 4u]);
			g_rx_offer_yiaddr[0] = g_rxf[bootp + 16u];
			g_rx_offer_yiaddr[1] = g_rxf[bootp + 17u];
			g_rx_offer_yiaddr[2] = g_rxf[bootp + 18u];
			g_rx_offer_yiaddr[3] = g_rxf[bootp + 19u];
			/* DHCP options: magic cookie @ bootp+236, then TLV; find opt53
			 * (msg type) + opt54 (server-id, needed for the REQUEST). */
			g_rx_offer_msgtype = 0u;
			opt = bootp + 236u + 4u; /* skip the 4-byte magic cookie */
			end = flen;
			while (opt + 1u < end) {
				uint8_t t = g_rxf[opt];
				uint8_t l;
				if (t == 0xffu) {
					break; /* end option */
				}
				if (t == 0x00u) {
					opt++; /* pad */
					continue;
				}
				l = g_rxf[opt + 1u];
				if (t == 53u && l >= 1u && opt + 2u < end) {
					g_rx_offer_msgtype = g_rxf[opt + 2u];
				}
				if (t == 54u && l >= 4u && opt + 5u < end) {
					g_dhcp_serverid[0] = g_rxf[opt + 2u];
					g_dhcp_serverid[1] = g_rxf[opt + 3u];
					g_dhcp_serverid[2] = g_rxf[opt + 4u];
					g_dhcp_serverid[3] = g_rxf[opt + 5u];
				}
				opt += 2u + l;
			}
			/* only a reply of the wanted type ends the poll (a late OFFER while
			 * we wait for the ACK must not stop us). */
			if (g_rx_want == 0u || g_rx_offer_msgtype == g_rx_want) {
				g_rx_offer_seen = 1;
			}
		}
	}
	printf("wifi: RX-DHCP want=%u seen=%d msgtype=%u xid=0x%08x yiaddr=%u.%u.%u.%u serverid=%u.%u.%u.%u ch2=%d\n",
		(unsigned)g_rx_want, g_rx_offer_seen, (unsigned)g_rx_offer_msgtype, (unsigned)g_rx_offer_xid,
		g_rx_offer_yiaddr[0], g_rx_offer_yiaddr[1], g_rx_offer_yiaddr[2], g_rx_offer_yiaddr[3],
		g_dhcp_serverid[0], g_dhcp_serverid[1], g_dhcp_serverid[2], g_dhcp_serverid[3], g_rx_ch2_frames);
	fflush(stdout);
}

/* IPv4 header checksum (16-bit ones-complement over the 20-byte header, cksum
 * field pre-zeroed). Needed because the DHCP REQUEST's option block differs in
 * length from the DISCOVER, changing the IP total-length. */
static uint16_t diag_ipcksum(const uint8_t *p, int len)
{
	uint32_t sum = 0u;
	int i;
	for (i = 0; i + 1 < len; i += 2) {
		sum += ((uint32_t)p[i] << 8) | (uint32_t)p[i + 1];
	}
	if ((len & 1) != 0) {
		sum += (uint32_t)p[len - 1] << 8;
	}
	while ((sum >> 16) != 0u) {
		sum = (sum & 0xffffu) + (sum >> 16);
	}
	return (uint16_t)(~sum);
}

/* DHCP message type this frame carries: 1 = DISCOVER (default), 3 = REQUEST
 * (adds opt50 requested-IP = g_dhcp_reqip + opt54 server-id). */
static int g_tx_dhcp_type = 1;
static uint8_t g_dhcp_reqip[4] = { 0 };  /* offered IP, snapshotted for the REQUEST's opt50 */
static int g_dhcp_ack_seen = 0;
static int g_dhcp_offered = 0;           /* an OFFER (opt53=2) was accepted */
static uint8_t g_dhcp_bound[4] = { 0 };  /* IP confirmed by the ACK */

/* TX a DHCP DISCOVER/REQUEST 802.3 frame as an SDPCM channel-2 DATA frame
 * (4-byte BDC header), driving the data-plane TX path end-to-end. Mirrors
 * diag_bcdcCmd's F2 write but channel=2 and the BDC header instead of the
 * 16-byte BCDC dcmd. The eth frame is built directly into g_txf at +16 (after
 * SDPCM[12]+BDC[4]). Design: docs/inprogress/2026-08-13-wifi-dataplane-design.md.
 *
 * Bare NON-glom SDPCM data frame (the fw's DEFAULT mode form, before any
 * bus:rxglom): HW[0-3] + SW[4-11] (seq/chan2/nextlen0/doff12) + BDC[12-15] +
 * eth@16. This is the format every non-scatter-gather brcmfmac host uses for
 * data, and the one the proven DHCP exchange used. (The probe also carried a
 * HWEXT txglom variant; that hypothesis was disproved -- the glom matrix was
 * flat -- so the driver only ever builds the non-glom form.) */
static void diag_wifiDataTx(volatile uint8_t *sdhci, uint32_t sdio_core, uint8_t seq)
{
	uint8_t mac[8];
	uint32_t ml = 0u;
	int i, elen;
	uint32_t total, wlen;
	int rc;

	g_tx_ran = 1;
	for (i = 0; i < 8; ++i) {
		mac[i] = 0u;
	}
	if (g_txmac_valid) {
		/* use the MAC read earlier in non-glom mode (see the DHCP block) */
		for (i = 0; i < 6; ++i) {
			mac[i] = g_txmac[i];
		}
		g_tx_mac_rc = 0;
	} else {
		g_tx_mac_rc = diag_iovar(sdhci, sdio_core, 0, "cur_etheraddr", NULL, 6u,
			mac, sizeof(mac), &ml, 200u, seq);
	}

	for (i = 0; i < (int)F2_FRAME_MAX; ++i) {
		g_txf[i] = 0u;
	}
	/* --- 802.3 Ethernet header @16 --- */
	for (i = 0; i < 6; ++i) {
		g_txf[16 + i] = 0xffu; /* dst broadcast */
	}
	for (i = 0; i < 6; ++i) {
		g_txf[22 + i] = mac[i]; /* src = Pi wifi MAC */
	}
	g_txf[28] = 0x08u; g_txf[29] = 0x00u; /* ethertype IPv4 */
	/* --- IP header @30 (20B), src 0.0.0.0 dst 255.255.255.255 --- */
	g_txf[30] = 0x45u; g_txf[31] = 0x00u; /* ver/ihl, tos */
	g_txf[32] = 0x01u; g_txf[33] = 0x13u; /* total length 275 */
	g_txf[38] = 0x40u;                    /* ttl 64 */
	g_txf[39] = 0x11u;                    /* proto UDP */
	g_txf[40] = 0x79u; g_txf[41] = 0xdbu; /* IP header checksum */
	g_txf[46] = 0xffu; g_txf[47] = 0xffu; g_txf[48] = 0xffu; g_txf[49] = 0xffu; /* dst */
	/* --- UDP header @50 (8B), 68->67 --- */
	g_txf[50] = 0x00u; g_txf[51] = 0x44u; /* src port 68 */
	g_txf[52] = 0x00u; g_txf[53] = 0x43u; /* dst port 67 */
	g_txf[54] = 0x00u; g_txf[55] = 0xffu; /* udp length 255 (checksum 0) */
	/* --- DHCP/BOOTP @58 --- */
	g_txf[58] = 0x01u; g_txf[59] = 0x01u; g_txf[60] = 0x06u; /* op/htype/hlen */
	g_txf[62] = 0x12u; g_txf[63] = 0x34u; g_txf[64] = 0x56u; g_txf[65] = 0x78u; /* xid */
	g_txf[68] = 0x80u; g_txf[69] = 0x00u; /* flags: broadcast */
	for (i = 0; i < 6; ++i) {
		g_txf[86 + i] = mac[i]; /* chaddr = MAC */
	}
	g_txf[294] = 0x63u; g_txf[295] = 0x82u; g_txf[296] = 0x53u; g_txf[297] = 0x63u; /* DHCP magic */
	{
		/* DHCP options (cursor p = absolute g_txf offset), then patch the IP
		 * total-length / UDP length / IP checksum from the actual options end so
		 * DISCOVER and the longer REQUEST are both well-formed. */
		uint32_t p = 298u;
		uint32_t udp_len, ip_total;
		uint16_t cks;
		g_txf[p++] = 53u; g_txf[p++] = 1u; g_txf[p++] = (uint8_t)g_tx_dhcp_type; /* opt53 msg type */
		if (g_tx_dhcp_type == 3) {
			g_txf[p++] = 50u; g_txf[p++] = 4u;                 /* opt50 requested IP = offered yiaddr */
			g_txf[p++] = g_dhcp_reqip[0]; g_txf[p++] = g_dhcp_reqip[1];
			g_txf[p++] = g_dhcp_reqip[2]; g_txf[p++] = g_dhcp_reqip[3];
			g_txf[p++] = 54u; g_txf[p++] = 4u;                 /* opt54 server identifier */
			g_txf[p++] = g_dhcp_serverid[0]; g_txf[p++] = g_dhcp_serverid[1];
			g_txf[p++] = g_dhcp_serverid[2]; g_txf[p++] = g_dhcp_serverid[3];
		}
		g_txf[p++] = 55u; g_txf[p++] = 1u; g_txf[p++] = 1u;    /* opt55 param req (subnet) */
		g_txf[p++] = 0xffu;                                    /* end */
		udp_len = p - 50u;   /* UDP hdr(8) + BOOTP(236)+magic(4)+options */
		ip_total = p - 30u;  /* IP hdr(20) + UDP */
		g_txf[54] = (uint8_t)((udp_len >> 8) & 0xffu); g_txf[55] = (uint8_t)(udp_len & 0xffu);
		g_txf[32] = (uint8_t)((ip_total >> 8) & 0xffu); g_txf[33] = (uint8_t)(ip_total & 0xffu);
		g_txf[40] = 0u; g_txf[41] = 0u;
		cks = diag_ipcksum(&g_txf[30], 20);
		g_txf[40] = (uint8_t)((cks >> 8) & 0xffu); g_txf[41] = (uint8_t)(cks & 0xffu);
		elen = (int)(p - 16u); /* eth payload length */
	}
	g_tx_len = elen;

	{
		uint32_t ng_total = 16u + (uint32_t)elen;
		g_txf[0] = (uint8_t)(ng_total & 0xffu);
		g_txf[1] = (uint8_t)((ng_total >> 8) & 0xffu);
		g_txf[2] = (uint8_t)((~ng_total) & 0xffu);
		g_txf[3] = (uint8_t)(((~ng_total) >> 8) & 0xffu);
		g_txf[4] = seq;
		g_txf[5] = 0x02u; /* channel = DATA */
		g_txf[6] = 0u;    /* nextlen */
		g_txf[7] = 12u;   /* data_offset = SDPCM_HWHDR+SWHDR = 12 (no HWEXT) */
		g_txf[8] = 0u; g_txf[9] = 0u; g_txf[10] = 0u; g_txf[11] = 0u;
		/* BDC header [12-15]: flags = BCDC proto ver 2 << 4; prio/flags2/doff = 0 */
		g_txf[12] = 0x20u;
		g_txf[13] = 0u; g_txf[14] = 0u; g_txf[15] = 0u;
		total = ng_total;
	}

	diag_setWindow18(sdhci);
	wlen = (total + 3u) & ~3u;
	/* dump the on-wire TX frame header so it can be diffed byte-for-byte
	 * against Linux's captured data frame (measure, don't infer). */
	printf("wifi: TXFRAME wlen=%u total=%u hdr=", (unsigned)wlen, (unsigned)total);
	for (i = 0; i < 28; ++i) {
		printf("%02x ", (unsigned)g_txf[i]);
	}
	printf("\n");
	fflush(stdout);
	rc = diag_sdioCmd53WriteByteMode(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR, wlen, g_txf);
	if (rc != 0) {
		diag_sdhciResetDatCmd(sdhci);
	}
	g_tx_rc = rc;
}


/* ---- generic data path (the lwip netif TX/RX primitives) ------------------
 *
 * diag_wifiDataTx above can only synthesize a DHCP packet. A netif needs
 * "send THIS frame" / "give me the next frame", so these two are the generic
 * form. They are deliberately free of printf: this is a data plane.
 */
/* SDPCM sequence for the generic data path. NOT independent: it is the same
 * per-bus stream the control/event paths advance, and the join+DHCP flow seeds
 * it (see g_data_seq = seq there). An out-of-sequence data frame is dropped by
 * the firmware without any error surfacing to the host. */
static uint8_t g_data_seq = 0;
static uint32_t g_frame_tx_ok = 0, g_frame_tx_err = 0;
static uint32_t g_frame_rx_ok = 0, g_frame_rx_err = 0, g_frame_rx_garbage = 0;


/* Transmit one 802.3 frame. Returns 0 on success, <0 on a transport error. */
static int diag_wifiFrameTx(volatile uint8_t *sdhci, const uint8_t *eth, uint32_t elen)
{
	uint32_t total, padded, i;
	int rc;

	if ((sdhci == NULL) || (eth == NULL) || (elen < 14u) || ((elen + 16u) > F2_FRAME_MAX)) {
		return -1060;
	}
	total = 16u + elen;
	padded = ((total + F2_BLKSZ - 1u) / F2_BLKSZ) * F2_BLKSZ;
	if (padded > F2_FRAME_MAX) {
		padded = F2_FRAME_MAX;
	}

	for (i = 0; i < 16u; ++i) {
		g_txf[i] = 0u;
	}
	for (i = 0; i < elen; ++i) {
		g_txf[16u + i] = eth[i];
	}
	/* zero the block-mode tail padding rather than shipping stale bytes */
	for (i = total; i < padded; ++i) {
		g_txf[i] = 0u;
	}

	/* SDPCM HW header: length + its one's complement as the check word. */
	g_txf[0] = (uint8_t)(total & 0xffu);
	g_txf[1] = (uint8_t)((total >> 8) & 0xffu);
	g_txf[2] = (uint8_t)((~total) & 0xffu);
	g_txf[3] = (uint8_t)(((~total) >> 8) & 0xffu);
	/* SDPCM SW header: seq, channel 2 (DATA), no nextlen, data_offset 12. */
	g_txf[4] = g_data_seq++;
	g_txf[5] = 0x02u;
	g_txf[6] = 0u;
	g_txf[7] = 12u;
	/* BDC header: BCDC proto ver 2 << 4; prio/flags2/doff = 0 (eth at +16). */
	g_txf[12] = 0x20u;

	diag_setWindow18(sdhci);
	rc = diag_f2Write(sdhci, g_txf, total);
	if (rc != 0) {
		diag_sdhciResetDatCmd(sdhci);
		g_frame_tx_err++;
		return rc;
	}
	g_frame_tx_ok++;
	return 0;
}


/* Receive one 802.3 frame. Returns 0 with *elen set, 1 if nothing is ready (or
 * the frame was not DATA), <0 on error.
 *
 * NOTE for the netif work: a non-channel-2 frame read here is DISCARDED, and
 * the control/event paths drain the same FIFO into the same g_rxf. That is
 * survivable for a one-shot command but NOT for a continuous RX thread, which
 * would steal control replies. The netif attach needs one central demux
 * (ch0 -> control waiter, ch1 -> events, ch2 -> lwip input) plus a bus mutex. */
static int diag_wifiFrameRx(volatile uint8_t *sdhci, uint8_t *eth, uint32_t cap,
	uint32_t *elen)
{
	uint16_t flen = 0u;
	uint8_t chan = 0u;
	uint32_t sdoff, ethoff, n, i;
	int rc;

	rc = diag_f2RecvFrame(sdhci, g_rxf, &flen, &chan);
	if (rc != 0) {
		/* -31 is an inconsistent SDPCM header, i.e. the FIFO handed back
		 * something that is not a frame boundary. Count it apart from real
		 * transport failures: conflating the two hid which was happening. */
		if (rc == -31) {
			g_frame_rx_garbage++;
			return 1;
		}
		if (rc < 0) {
			g_frame_rx_err++;
		}
		return rc;
	}
	if (chan != 2u) {
		return 1;
	}
	/* eth offset = SDPCM data_offset + BDC(4) + BDC.doff words (<<2), exactly
	 * as brcmf_proto_bcdc_hdrpull computes it. Never assume a fixed 16. */
	sdoff = g_rxf[7];
	if ((sdoff < 12u) || ((sdoff + 4u) > (uint32_t)flen)) {
		g_frame_rx_err++;
		return -1061;
	}
	ethoff = sdoff + 4u + ((uint32_t)g_rxf[sdoff + 3u] << 2);
	if ((ethoff + 14u) > (uint32_t)flen) {
		g_frame_rx_err++;
		return -1062;
	}
	n = (uint32_t)flen - ethoff;
	if (n > cap) {
		g_frame_rx_err++;
		return -1063;
	}
	if (eth != NULL) {
		for (i = 0; i < n; ++i) {
			eth[i] = g_rxf[ethoff + i];
		}
	}
	if (elen != NULL) {
		*elen = n;
	}
	g_frame_rx_ok++;
	return 0;
}


/* UDP checksum over the IPv4 pseudo-header + UDP header + payload (RFC 768).
 * Worth computing: it makes the HOST verify payload integrity for us, since
 * tcpdump only prints "udp sum ok" when every byte survived the air. */
static uint16_t diag_udpcksum(const uint8_t *ip, const uint8_t *udp, uint32_t udplen)
{
	uint32_t sum = 0u, i;

	for (i = 0; i < 4u; i += 2u) { /* src + dst addresses */
		sum += ((uint32_t)ip[12 + i] << 8) | (uint32_t)ip[13 + i];
		sum += ((uint32_t)ip[16 + i] << 8) | (uint32_t)ip[17 + i];
	}
	sum += 17u;     /* pseudo-header protocol */
	sum += udplen;  /* pseudo-header UDP length */
	for (i = 0; (i + 1u) < udplen; i += 2u) {
		sum += ((uint32_t)udp[i] << 8) | (uint32_t)udp[i + 1u];
	}
	if ((udplen & 1u) != 0u) {
		sum += (uint32_t)udp[udplen - 1u] << 8;
	}
	while ((sum >> 16) != 0u) {
		sum = (sum & 0xffffu) + (sum >> 16);
	}
	sum = (~sum) & 0xffffu;
	return (uint16_t)((sum == 0u) ? 0xffffu : sum); /* 0 means "no checksum" */
}


/* Build and send one UDP broadcast carrying `plen` pattern bytes, so a
 * host-side capture can confirm both the length and (via the checksum) the
 * integrity of a full-MTU frame. Broadcast on purpose: no ARP needed. */
static uint8_t g_udpf[F2_FRAME_MAX]; /* last frame built by diag_wifiUdpTx */

static int diag_wifiUdpTx(volatile uint8_t *sdhci, uint32_t plen, uint16_t dport)
{
	uint32_t i, iptot, udplen, elen;
	uint16_t cks;

	if ((42u + plen) > 1514u) {
		return -1064;
	}
	for (i = 0; i < (42u + plen); ++i) {
		g_udpf[i] = 0u;
	}
	for (i = 0; i < 6u; ++i) {
		g_udpf[i] = 0xffu;          /* dst: broadcast */
		g_udpf[6u + i] = g_txmac[i]; /* src: our WiFi MAC */
	}
	g_udpf[12] = 0x08u;
	g_udpf[13] = 0x00u;             /* ethertype IPv4 */

	udplen = 8u + plen;
	iptot = 20u + udplen;
	g_udpf[14] = 0x45u;             /* IPv4, ihl 5 */
	g_udpf[16] = (uint8_t)((iptot >> 8) & 0xffu);
	g_udpf[17] = (uint8_t)(iptot & 0xffu);
	g_udpf[22] = 64u;               /* ttl */
	g_udpf[23] = 17u;               /* proto UDP */
	for (i = 0; i < 4u; ++i) {
		g_udpf[26u + i] = g_dhcp_bound[i];  /* src: our lease */
		g_udpf[30u + i] = 0xffu;            /* dst: 255.255.255.255 */
	}
	cks = diag_ipcksum(&g_udpf[14], 20);
	g_udpf[24] = (uint8_t)((cks >> 8) & 0xffu);
	g_udpf[25] = (uint8_t)(cks & 0xffu);

	g_udpf[34] = 0x27u;             /* src port 9999 */
	g_udpf[35] = 0x0fu;
	g_udpf[36] = (uint8_t)((dport >> 8) & 0xffu);
	g_udpf[37] = (uint8_t)(dport & 0xffu);
	g_udpf[38] = (uint8_t)((udplen >> 8) & 0xffu);
	g_udpf[39] = (uint8_t)(udplen & 0xffu);
	for (i = 0; i < plen; ++i) {
		g_udpf[42u + i] = (uint8_t)(i ^ 0x5au); /* the pattern the host checks */
	}
	cks = diag_udpcksum(&g_udpf[14], &g_udpf[34], udplen);
	g_udpf[40] = (uint8_t)((cks >> 8) & 0xffu);
	g_udpf[41] = (uint8_t)(cks & 0xffu);

	elen = 42u + plen;
	return diag_wifiFrameTx(sdhci, g_udpf, elen);
}

/* WPA2-PSK join of g_join_ssid/g_join_psk, then (on success) the full DHCP
 * exchange over SDPCM channel 2. Faithful copy of the probe's proven
 * diag_wifiJoin + its `jointxcnt` DHCP block; results land in the g_join_* /
 * g_dhcp_* globals, which wifi_netup() renders. */
static void diag_wifiJoinWpa2(volatile uint8_t *sdhci, uint32_t sdio_core)
{
	uint8_t emask[16];
	uint8_t val4[4];
	uint8_t pmk[132];
	uint8_t ssidbuf[36];
	uint32_t reqid = 1u;
	uint8_t seq = 0u;
	int i, t, slen, plen;
	int got_setssid = 0, got_psksup = 0;
	int attempt = 0;

	g_join_ran = 1;
	printf("wifi: JOIN-START (ssid=%s)\n", g_join_ssid);
	fflush(stdout);
	diag_sdhciResetDatCmd(sdhci);

	/* event_msgs: enable join events 0(SET_SSID),5,6,7(ASSOC),11,12,16(LINK),
	 * 46(PSK_SUP) + keep 69(escan, harmless). mask[i/8] |= 1<<(i%8). */
	for (i = 0; i < 16; ++i) {
		emask[i] = 0u;
	}
	emask[0] = (1u << 0) | (1u << 5) | (1u << 6) | (1u << 7); /* 0,5,6,7 */
	emask[1] = (1u << 3) | (1u << 4);                         /* 11,12 */
	emask[2] = (1u << 0);                                     /* 16 */
	emask[5] = (1u << 6);                                     /* 46 */
	emask[8] = 0x20u;                                        /* 69 (escan) */
	g_join_em_rc = diag_iovar(sdhci, sdio_core, 1, "event_msgs", emask, 16u,
		NULL, 0u, NULL, reqid++, seq++);

	/* CLM (regulatory) before UP so the radio has channels (same as scan). */
	(void)diag_clmLoad(sdhci, sdio_core, &reqid, &seq);

	/* infra=1 then WLC_UP */
	val4[0] = 1u; val4[1] = 0u; val4[2] = 0u; val4[3] = 0u;
	g_join_infra_rc = diag_bcdcCmd(sdhci, sdio_core, 1, BRCMF_C_SET_INFRA,
		val4, 4u, NULL, 0u, NULL, reqid++, seq++);
	g_join_up_rc = diag_bcdcCmd(sdhci, sdio_core, 1, WLC_UP_CMD,
		val4, 4u, NULL, 0u, NULL, reqid++, seq++);
	usleep(500 * 1000); /* let PHY finish coming up before security/join */

	/* wsec = 4 (AES/CCMP) */
	val4[0] = 4u; val4[1] = 0u; val4[2] = 0u; val4[3] = 0u;
	g_join_wsec_rc = diag_iovar(sdhci, sdio_core, 1, "wsec", val4, 4u,
		NULL, 0u, NULL, reqid++, seq++);
	/* wpa_auth = 0x80 (WPA2_AUTH_PSK) */
	val4[0] = 0x80u;
	g_join_wpaauth_rc = diag_iovar(sdhci, sdio_core, 1, "wpa_auth", val4, 4u,
		NULL, 0u, NULL, reqid++, seq++);
	/* sup_wpa = 1 (enable firmware supplicant) -- MUST precede WSEC_PMK */
	val4[0] = 1u;
	g_join_sup_rc = diag_iovar(sdhci, sdio_core, 1, "sup_wpa", val4, 4u,
		NULL, 0u, NULL, reqid++, seq++);

	/* WLC_SET_WSEC_PMK (268): brcmf_wsec_pmk_le { le16 key_len; le16 flags;
	 * u8 key[128] } = 132 bytes. Passphrase path: flags=0x0001, key=ASCII. */
	for (i = 0; i < 132; ++i) {
		pmk[i] = 0u;
	}
	plen = 0;
	while (g_join_psk[plen] != '\0' && plen < 63) {
		plen++;
	}
	pmk[0] = (uint8_t)(plen & 0xff);
	pmk[1] = (uint8_t)((plen >> 8) & 0xff);
	pmk[2] = 0x01u; /* BRCMF_WSEC_PASSPHRASE */
	pmk[3] = 0x00u;
	for (i = 0; i < plen; ++i) {
		pmk[4 + i] = (uint8_t)g_join_psk[i];
	}
	g_join_pmk_rc = diag_bcdcCmd(sdhci, sdio_core, 1, WLC_SET_WSEC_PMK_CMD,
		pmk, 132u, NULL, 0u, NULL, reqid++, seq++);

	/* WLC_SET_SSID (26): brcmf_ssid_le { le32 SSID_len; u8 SSID[32] } = 36 B
	 * broadcast join -> fw associates + runs the handshake. */
	for (i = 0; i < 36; ++i) {
		ssidbuf[i] = 0u;
	}
	slen = 0;
	while (g_join_ssid[slen] != '\0' && slen < 32) {
		slen++;
	}
	ssidbuf[0] = (uint8_t)(slen & 0xff);
	ssidbuf[1] = (uint8_t)((slen >> 8) & 0xff);
	for (i = 0; i < slen; ++i) {
		ssidbuf[4 + i] = (uint8_t)g_join_ssid[i];
	}
	/* Retry the join: at good RSSI a broadcast WLC_SET_SSID can still intermittently
	 * miss the AP in the fw's join-scan (WLC_E_SET_SSID status=3 NO_NETWORKS); a
	 * few retries reliably associate. Stop as soon as connected. */
	for (attempt = 0; attempt < 5; ++attempt) {
	got_setssid = 0;
	got_psksup = 0;
	g_join_setssid_status = -100;
	g_join_psksup_status = -100;
	g_join_ssid_rc = diag_bcdcCmd(sdhci, sdio_core, 1, WLC_SET_SSID_CMD,
		ssidbuf, 36u, NULL, 0u, NULL, reqid++, seq++);

	/* Watch events off SDPCM channel 1 (same demux as escan): WLC_E_SET_SSID
	 * (type 0) status, WLC_E_PSK_SUP (type 46) status, WLC_E_LINK (16) flags.
	 * event_msg fields relative to ehdr (ethhdr@0 + 10B bcmeth + event_msg@24):
	 * flags be16 @ehdr+26, event_type be32 @ehdr+28, status be32 @ehdr+32. */
	for (t = 0; t < 3000 && !(got_setssid && got_psksup); ++t) {
		uint16_t len;
		uint8_t chan;
		int fr;
		uint32_t sdoff, ehdr, etype, status, flags;

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
		g_join_evt_total++;
		sdoff = g_rxf[7];
		if (sdoff + 4u > len) {
			continue;
		}
		ehdr = sdoff + 4u + 4u * (uint32_t)g_rxf[sdoff + 3u];
		if (ehdr + 48u > (uint32_t)len) {
			continue;
		}
		if (diag_be16(g_rxf + ehdr + 12u) != 0x886Cu) {
			continue; /* not ETH_P_LINK_CTL (event) */
		}
		flags = diag_be16(g_rxf + ehdr + 26u);
		etype = diag_be32(g_rxf + ehdr + 28u);
		status = diag_be32(g_rxf + ehdr + 32u);
		if (etype == 0u) { /* WLC_E_SET_SSID */
			g_join_setssid_status = (int)status;
			got_setssid = 1;
			if (status != 0u) {
				break; /* association failed */
			}
		}
		else if (etype == 46u) { /* WLC_E_PSK_SUP */
			g_join_psksup_status = (int)status;
			got_psksup = 1;
		}
		else if (etype == 16u) { /* WLC_E_LINK */
			g_join_link_up = (flags & 0x01u) ? 1 : 0;
		}
	}

	g_join_attempts = attempt + 1;
	if (g_join_setssid_status == 0 && g_join_psksup_status == 6) {
		break; /* connected -- stop retrying */
	}
	if (attempt < 4) {
		usleep(700 * 1000); /* brief settle before the next join attempt */
	}
	}
	printf("wifi: JOIN-DONE attempts=%d setssid=%d psksup=%d link=%d\n",
		g_join_attempts, g_join_setssid_status, g_join_psksup_status, g_join_link_up);
	fflush(stdout);

	/* Only an associated + 4-way-keyed STA can carry data frames, so skip the
	 * DHCP exchange on a failed join (the probe's one-shot run always fell
	 * through here; the resident driver classifies it as JOIN-FAILED instead). */
	if (!(g_join_setssid_status == 0 && g_join_psksup_status == 6)) {
		return;
	}

	/* NON-glom data TX with an fw pktcnt snapshot (localizes where a frame dies
	 * if the exchange stalls). Stays non-glom so the pktcnt GET's control-reply
	 * RX is unperturbed. */
	{
		uint8_t macbuf[8] = { 0 };
		uint32_t maclen = 0u;
		int k;
		g_tx_mac_rc = diag_iovar(sdhci, sdio_core, 0, "cur_etheraddr", NULL, 6u,
			macbuf, sizeof(macbuf), &maclen, reqid++, seq++);
		for (k = 0; k < 6; ++k) {
			g_txmac[k] = macbuf[k];
		}
		g_txmac_valid = 1;

		g_pktcnt_pre_rc = diag_wifiPktcnt(sdhci, sdio_core, g_pktcnt_pre, reqid++, seq++);
		printf("wifi: PKTCNT-PRE rc=%d rx_good=%u rx_bad=%u tx_good=%u tx_bad=%u\n",
			g_pktcnt_pre_rc, g_pktcnt_pre[0], g_pktcnt_pre[1], g_pktcnt_pre[2], g_pktcnt_pre[3]);
		fflush(stdout);

		/* Full DHCP over Wi-Fi (SELECTING): DISCOVER->OFFER->REQUEST->ACK, in
		 * DHCP-client-like rounds (send a few, then a paced ch2 poll -- the
		 * fw->host RX delivery + round-trip is timing-sensitive).
		 * Skipped for `joinwpa`, where lwip runs DHCP over the frame seam. */
		if (g_join_skip_dhcp == 0) {
			int round, offered = 0;

			/* Phase A: DISCOVER -> OFFER (opt53=2); capture yiaddr + server-id. */
			g_tx_dhcp_type = 1;
			g_rx_want = 2u;
			for (round = 0; round < 4 && g_rx_offer_seen == 0; ++round) {
				for (k = 0; k < 4; ++k) {
					diag_wifiDataTx(sdhci, sdio_core, (uint8_t)(seq + k));
					if (g_tx_rc == 0) {
						g_dtx_burst++;
					}
				}
				seq = (uint8_t)(seq + 4);
				printf("wifi: DHCP-DISCOVER round=%d frames=%d last_rc=%d\n", round, g_dtx_burst, g_tx_rc);
				fflush(stdout);
				diag_wifiRxDhcp(sdhci, sdio_core);
			}

			if (g_rx_offer_seen != 0 && g_rx_offer_msgtype == 2u) {
				offered = 1;
				g_dhcp_reqip[0] = g_rx_offer_yiaddr[0]; g_dhcp_reqip[1] = g_rx_offer_yiaddr[1];
				g_dhcp_reqip[2] = g_rx_offer_yiaddr[2]; g_dhcp_reqip[3] = g_rx_offer_yiaddr[3];
				printf("wifi: DHCP-OFFER accepted yiaddr=%u.%u.%u.%u serverid=%u.%u.%u.%u\n",
					g_dhcp_reqip[0], g_dhcp_reqip[1], g_dhcp_reqip[2], g_dhcp_reqip[3],
					g_dhcp_serverid[0], g_dhcp_serverid[1], g_dhcp_serverid[2], g_dhcp_serverid[3]);
				fflush(stdout);

				/* Phase B: REQUEST (opt50 reqip + opt54 serverid) -> ACK (opt53=5). */
				g_tx_dhcp_type = 3;
				g_rx_want = 5u;
				for (round = 0; round < 5 && g_dhcp_ack_seen == 0; ++round) {
					for (k = 0; k < 4; ++k) {
						diag_wifiDataTx(sdhci, sdio_core, (uint8_t)(seq + k));
						if (g_tx_rc == 0) {
							g_dtx_burst++;
						}
					}
					seq = (uint8_t)(seq + 4);
					printf("wifi: DHCP-REQUEST round=%d reqip=%u.%u.%u.%u\n", round,
						g_dhcp_reqip[0], g_dhcp_reqip[1], g_dhcp_reqip[2], g_dhcp_reqip[3]);
					fflush(stdout);
					diag_wifiRxDhcp(sdhci, sdio_core);
					if (g_rx_offer_seen != 0 && g_rx_offer_msgtype == 5u) {
						g_dhcp_ack_seen = 1;
						g_dhcp_bound[0] = g_rx_offer_yiaddr[0]; g_dhcp_bound[1] = g_rx_offer_yiaddr[1];
						g_dhcp_bound[2] = g_rx_offer_yiaddr[2]; g_dhcp_bound[3] = g_rx_offer_yiaddr[3];
					}
				}
			}

			printf("wifi: DHCP-RESULT offer=%d ack=%d bound_ip=%u.%u.%u.%u => %s\n",
				offered, g_dhcp_ack_seen,
				g_dhcp_bound[0], g_dhcp_bound[1], g_dhcp_bound[2], g_dhcp_bound[3],
				g_dhcp_ack_seen ? "BOUND (full DHCP lease over WiFi)" :
					(offered ? "OFFER-ONLY (REQUEST/ACK incomplete)" : "NO-OFFER"));
			fflush(stdout);
			g_dhcp_offered = offered;
		}

		/* Hand the bus sequence over to the generic data path, on BOTH paths
		 * (join-only included -- the netif transmits from there). SDPCM seq is
		 * ONE per-bus stream shared by control, event and data frames, so a
		 * later data TX must CONTINUE it. Starting a second counter at 0 makes
		 * the fw silently drop every frame while the SDIO write still returns
		 * 0, so it looks like a working TX. */
		g_data_seq = seq;
	}
}

/* ------------------------------------------------------------------ */
/* Resident-driver state. wifi_bringup() runs the firmware bring-up once at
 * startup and leaves the SDIO controller mapped here so wifi_scan() can drive
 * escan transactions against it per client "scan" request. */
static volatile uint8_t *g_sdhci = NULL;   /* SDHCI (Arasan) mapping, kept live after bring-up */
static uint32_t g_sdio_core = 0x18004000u; /* EROM-derived SDIO-DEV core base (set by bring-up) */

#define WIFI_RESP_CAP (8u * 1024u)
/* Two device ids on one port: 0 = /dev/wifi (text commands + text result),
 * 1 = /dev/wifidata (raw 802.3 frames, the lwip netif seam). */
/* How long a /dev/wifidata read waits for a frame before reporting "none", and
 * how often it probes the FIFO while waiting. The wait keeps the caller's RX
 * thread off the message bus; the probe interval is what actually bounds
 * receive latency. */
/* READ THIS BEFORE TUNING THESE CONSTANTS.
 *
 * A waiting read (WAIT>0), a spin before sleeping, and a bus mutex with a
 * multi-threaded message loop were all tried on hardware. None of them showed a
 * benefit that survives the measurement noise: the SAME code measured 1.73 and
 * 0.66 MB/s TX on two runs, a 2.6x spread, so single-run A/B of throughput on
 * this link proves nothing. They were reverted for adding complexity without
 * demonstrated gain, and WAIT=0 keeps the simplest behaviour: probe once,
 * return.
 *
 * What IS solid, because it averages thousands of samples inside one run:
 *   transmit  132 us per frame
 *   receive   178 us per frame
 *   empty probe 22 us, and there were 1.1 MILLION of them (24.9 s of bus time)
 *
 * So the radio and the SDIO transfers are not the limit -- per-frame overhead
 * and the empty polling are. The fix is an interrupt-driven RX (SDIO CARD_INTR)
 * plus frame batching, which removes the per-frame wakeup instead of re-timing
 * it. Re-measure with n>=3 runs per config. */
#define WIFI_RX_WAIT_US  0u
#define WIFI_RX_PROBE_US 150u
#define WIFI_RX_SPIN_US  0u

#define WIFI_DEV_TEXT_ID 0
#define WIFI_DEV_DATA_ID 1

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

		/* Function 2 (WLAN data) block size. F1's was programmed just above;
		 * F2's never was, because nothing used block mode on the data path --
		 * which is precisely why data frames were stuck under the 512-byte
		 * byte-mode cap. FBR2's block size lives at 0x210/0x211. */
		(void)diag_sdioCmd52(sdhci, 1, 0, 0x210u, (uint8_t)(F2_BLKSZ & 0xffu), NULL);
		(void)diag_sdioCmd52(sdhci, 1, 0, 0x211u, (uint8_t)((F2_BLKSZ >> 8) & 0xffu), NULL);

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


/* ---- WiFi netup (WPA2 join + DHCP lease) -------------------------------- */
/* Bring the interface all the way up on a WPA2-PSK network: run the proven
 * firmware-supplicant join, then the DISCOVER/OFFER/REQUEST/ACK exchange over
 * the SDPCM data plane, and render the outcome into `out`. Wraps
 * diag_wifiJoinWpa2() the same way wifi_scan() wraps diag_wifiScan(): the diag
 * function only fills globals, this formats them. Returns bytes written.
 *
 * The probe this was ported from ran once per boot, so every result global has
 * to be reset here for the command to be repeatable (a stale g_dhcp_serverid
 * would otherwise be baked into the next REQUEST). */
/* `wifi mtu`: prove the data path carries FULL-MTU frames, which is the real
 * prerequisite for an lwip netif. The proven DHCP exchange only ever moved
 * ~340-byte frames -- comfortably under the 512-byte byte-mode ceiling -- so
 * it said nothing about MTU. Sends one UDP broadcast per size with correct IP
 * AND UDP checksums (so a host capture verifies integrity, not just arrival),
 * then drains channel 2 for a few seconds and reports the largest frame seen
 * plus a pattern check for a datagram the host sends to port 9997.
 *
 * Run `wifi netup <ssid> <psk>` first: this deliberately does not re-join,
 * since a second join against already-associated firmware is untested. */
static int wifi_mtu(char *out, int cap)
{
	static const uint32_t sizes[3] = { 400u, 1000u, 1472u };
	static uint8_t rxeth[F2_FRAME_MAX];
	int rc[3];
	int n = 0, i, tries;
	uint32_t rx_frames = 0u, rx_max = 0u, rx_pat_len = 0u, elen = 0u;
	uint32_t pre[5] = { 0 }, post[5] = { 0 };
	int rx_pat_ok = -1; /* -1 = no tagged frame seen, 1 = match, 0 = mismatch */

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	if (!((g_join_setssid_status == 0) && (g_join_psksup_status == 6))) {
		return snprintf(out, (size_t)cap,
			"WiFi MTU: NOT-JOINED (setssid=%d psksup=%d)\n"
			"  run `wifi netup <ssid> <psk>` first -- only an associated, 4-way-keyed\n"
			"  STA may carry data frames.\n",
			g_join_setssid_status, g_join_psksup_status);
	}
	if (!g_txmac_valid) {
		uint8_t mac[8];
		uint32_t ml = 0u;
		if (diag_iovar(g_sdhci, g_sdio_core, 0, "cur_etheraddr", NULL, 6u,
				mac, sizeof(mac), &ml, 200u, g_data_seq++) == 0) {
			for (i = 0; i < 6; ++i) {
				g_txmac[i] = mac[i];
			}
			g_txmac_valid = 1;
		}
	}

	/* --- TX: one broadcast per size, smallest first ---
	 * Bracketed by the firmware's OWN packet counters: a 0 rc from the SDIO
	 * write only proves the bytes reached the chip, not that the radio sent
	 * them. tx_good tells us which of the two happened. */
	(void)diag_wifiPktcnt(g_sdhci, g_sdio_core, pre, 300u, g_data_seq++);
	for (i = 0; i < 3; ++i) {
		rc[i] = diag_wifiUdpTx(g_sdhci, sizes[i], 9998u);
		usleep(20000);
	}
	usleep(200000);
	(void)diag_wifiPktcnt(g_sdhci, g_sdio_core, post, 301u, g_data_seq++);

	/* Dump the head of the last frame built, so it can be diffed byte-for-byte
	 * against the DHCP frame the AP demonstrably accepts. */
	printf("wifi: MTUFRAME eth=");
	for (i = 0; i < 48; ++i) {
		printf("%02x ", (unsigned)g_udpf[i]);
	}
	printf("\n");
	fflush(stdout);

	/* --- RX: drain channel 2 for ~3 s --- */
	for (tries = 0; tries < 600; ++tries) {
		int r = diag_wifiFrameRx(g_sdhci, rxeth, sizeof(rxeth), &elen);
		if (r == 1) {
			usleep(5000);
			continue;
		}
		if (r != 0) {
			continue; /* transport error already counted; keep draining */
		}
		rx_frames++;
		if (elen > rx_max) {
			rx_max = elen;
		}
		/* Is this the host's tagged probe? IPv4 + UDP + dport 9997, and if so
		 * does the payload still carry the pattern byte-for-byte? */
		if ((elen > 42u) && (rxeth[12] == 0x08u) && (rxeth[13] == 0x00u) &&
				((rxeth[14] & 0xf0u) == 0x40u) && (rxeth[23] == 17u)) {
			uint32_t ihl = (uint32_t)(rxeth[14] & 0x0fu) * 4u;
			uint32_t uoff = 14u + ihl;
			if ((uoff + 8u) < elen) {
				uint32_t dport = ((uint32_t)rxeth[uoff + 2u] << 8) | (uint32_t)rxeth[uoff + 3u];
				if (dport == 9997u) {
					uint32_t poff = uoff + 8u;
					uint32_t plen = elen - poff;
					uint32_t k;
					rx_pat_len = plen;
					rx_pat_ok = 1;
					for (k = 0; k < plen; ++k) {
						if (rxeth[poff + k] != (uint8_t)(k ^ 0x5au)) {
							rx_pat_ok = 0;
							break;
						}
					}
				}
			}
		}
	}

	n += snprintf(out + n, (size_t)(cap - n),
		"WiFi MTU test (bound %u.%u.%u.%u, F2 block size %u)\n",
		g_dhcp_bound[0], g_dhcp_bound[1], g_dhcp_bound[2], g_dhcp_bound[3],
		(unsigned)F2_BLKSZ);
	for (i = 0; i < 3; ++i) {
		n += snprintf(out + n, (size_t)(cap - n),
			"  TX udp payload=%4u eth=%4u frame=%4u mode=%-5s rc=%d %s\n",
			(unsigned)sizes[i], (unsigned)(42u + sizes[i]),
			(unsigned)(58u + sizes[i]),
			((58u + sizes[i] + 3u) & ~3u) <= 512u ? "byte" : "block",
			rc[i], (rc[i] == 0) ? "OK" : "FAIL");
	}
	n += snprintf(out + n, (size_t)(cap - n),
		"  RX ch2 frames=%u max_eth_len=%u\n"
		"  RX tagged(:9997) len=%u pattern=%s\n"
		"  fw pktcnt tx_good %u -> %u (delta %d), tx_bad %u -> %u\n"
		"  counters: tx_ok=%u tx_err=%u rx_ok=%u rx_err=%u rx_garbage=%u\n"
		"RESULT %s\n",
		(unsigned)rx_frames, (unsigned)rx_max,
		(unsigned)rx_pat_len,
		(rx_pat_ok < 0) ? "none-seen" : ((rx_pat_ok == 1) ? "OK" : "MISMATCH"),
		(unsigned)pre[2], (unsigned)post[2], (int)(post[2] - pre[2]),
		(unsigned)pre[3], (unsigned)post[3],
		(unsigned)g_frame_tx_ok, (unsigned)g_frame_tx_err,
		(unsigned)g_frame_rx_ok, (unsigned)g_frame_rx_err, (unsigned)g_frame_rx_garbage,
		((rc[0] == 0) && (rc[1] == 0) && (rc[2] == 0)) ? "TX-ALL-SIZES-SENT" : "TX-FAILED");
	return n;
}


/* Shared by `netup` and `joinwpa`: validate, reset every result global (the
 * probe this came from ran once per boot, so a repeatable command has to clear
 * them), then run the WPA2 join -- with or without the built-in DHCP.
 * Returns 0, or -1 if the arguments are unusable. */
static int wifi_joinRun(const char *ssid, const char *psk, int do_dhcp)
{
	size_t k;

	if ((ssid[0] == '\0') || (psk[0] == '\0')) {
		return -1;
	}
	g_join_skip_dhcp = (do_dhcp != 0) ? 0 : 1;

	for (k = 0; k + 1 < sizeof(g_join_ssid) && ssid[k] != '\0'; ++k) {
		g_join_ssid[k] = ssid[k];
	}
	g_join_ssid[k] = '\0';
	for (k = 0; k + 1 < sizeof(g_join_psk) && psk[k] != '\0'; ++k) {
		g_join_psk[k] = psk[k];
	}
	g_join_psk[k] = '\0';

	/* join accumulators */
	g_join_attempts = 0;
	g_join_setssid_status = -100;
	g_join_psksup_status = -100;
	g_join_link_up = 0;
	g_join_evt_total = 0;
	/* data-plane + DHCP accumulators */
	g_txmac_valid = 0;
	g_dtx_burst = 0;
	g_tx_dhcp_type = 1;
	g_rx_want = 0u;
	g_rx_offer_seen = 0;
	g_rx_ch2_frames = 0;
	g_rx_offer_msgtype = 0u;
	g_rx_offer_xid = 0u;
	for (k = 0; k < 4u; ++k) {
		g_rx_offer_yiaddr[k] = 0u;
		g_dhcp_serverid[k] = 0u;
		g_dhcp_reqip[k] = 0u;
		g_dhcp_bound[k] = 0u;
	}
	g_dhcp_ack_seen = 0;
	g_dhcp_offered = 0;

	diag_wifiJoinWpa2(g_sdhci, g_sdio_core);
	g_join_skip_dhcp = 0;
	return 0;
}


/* `joinwpa <ssid> <psk>`: associate + 4-way key ONLY, no DHCP -- the form an
 * lwip netif needs, because lwip runs DHCP itself over /dev/wifidata. Reports
 * the MAC too, so the netif can set its hwaddr without a second command. */
static int wifi_joinwpa(const char *ssid, const char *psk, char *out, int cap)
{
	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	if (wifi_joinRun(ssid, psk, 0) != 0) {
		return snprintf(out, (size_t)cap, "joinwpa: usage: joinwpa <ssid> <psk>\n");
	}
	return snprintf(out, (size_t)cap,
		"JOINWPA %s setssid=%d psksup=%d link=%d\n"
		"MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
		((g_join_setssid_status == 0) && (g_join_psksup_status == 6)) ? "ok" : "fail",
		g_join_setssid_status, g_join_psksup_status, g_join_link_up,
		g_txmac[0], g_txmac[1], g_txmac[2], g_txmac[3], g_txmac[4], g_txmac[5]);
}


/* Where the per-frame time actually goes. RX measured ~10 ms/frame end to end,
 * which is far more than the poll interval explains, so the read path is timed
 * here and split into HIT (a frame came back) vs MISS (empty FIFO): the two have
 * very different costs and averaging them together hides which one dominates. */
static uint64_t g_rx_hit_us = 0, g_rx_miss_us = 0, g_tx_us = 0;
static uint32_t g_rx_hits = 0, g_rx_misses = 0, g_tx_calls = 0;


/* Timing is OFF by default: two clock_gettime calls per probe are not free at
 * ~5000 probes/s, and switching them on measurably cost TX throughput
 * (1.73 -> 1.10 MB/s). Build with -DWIFI_STATS_TIMING=1 to attribute per-frame
 * cost again; the counters below are plain increments and always on. */
#ifndef WIFI_STATS_TIMING
#define WIFI_STATS_TIMING 0
#endif

#if WIFI_STATS_TIMING
static uint64_t wifi_nowUs(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		return 0u;
	}
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}
#define WIFI_T0(v)       uint64_t v = wifi_nowUs()
#define WIFI_ACC(acc, v) ((acc) += wifi_nowUs() - (v))
#else
#define WIFI_T0(v)       ((void)0)
#define WIFI_ACC(acc, v) ((void)0)
#endif


/* `stats`: per-frame timing of the data path, so a throughput number can be
 * attributed instead of guessed. */
static int wifi_stats(char *out, int cap)
{
	return snprintf(out, (size_t)cap,
		"WIFISTATS tx_calls=%u tx_us_total=%llu tx_us_avg=%llu\n"
		"WIFISTATS rx_hits=%u rx_hit_us_total=%llu rx_hit_us_avg=%llu\n"
		"WIFISTATS rx_misses=%u rx_miss_us_total=%llu rx_miss_us_avg=%llu\n"
		"WIFISTATS frames tx_ok=%u tx_err=%u rx_ok=%u rx_err=%u rx_garbage=%u\n",
		g_tx_calls, (unsigned long long)g_tx_us,
		(unsigned long long)(g_tx_calls ? g_tx_us / g_tx_calls : 0u),
		g_rx_hits, (unsigned long long)g_rx_hit_us,
		(unsigned long long)(g_rx_hits ? g_rx_hit_us / g_rx_hits : 0u),
		g_rx_misses, (unsigned long long)g_rx_miss_us,
		(unsigned long long)(g_rx_misses ? g_rx_miss_us / g_rx_misses : 0u),
		g_frame_tx_ok, g_frame_tx_err, g_frame_rx_ok, g_frame_rx_err, g_frame_rx_garbage);
}


/* `mac`: the station MAC, for a netif that has not joined yet. */
static int wifi_mac(char *out, int cap)
{
	uint8_t mac[8];
	uint32_t ml = 0u;
	int i;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	if (!g_txmac_valid) {
		if (diag_iovar(g_sdhci, g_sdio_core, 0, "cur_etheraddr", NULL, 6u,
				mac, sizeof(mac), &ml, 210u, g_data_seq++) == 0) {
			for (i = 0; i < 6; ++i) {
				g_txmac[i] = mac[i];
			}
			g_txmac_valid = 1;
		}
		else {
			return snprintf(out, (size_t)cap, "MAC unavailable\n");
		}
	}
	return snprintf(out, (size_t)cap, "MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_txmac[0], g_txmac[1], g_txmac[2], g_txmac[3], g_txmac[4], g_txmac[5]);
}


static int wifi_netup(const char *ssid, const char *psk, char *out, int cap)
{
	int off = 0, r;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	if (wifi_joinRun(ssid, psk, 1) != 0) {
		return snprintf(out, (size_t)cap, "netup: usage: netup <ssid> <psk>\n");
	}

	r = snprintf(out + off, (size_t)(cap - off),
		"netup: join ssid=%s setssid=%d psksup=%d link=%d\n",
		g_join_ssid, g_join_setssid_status, g_join_psksup_status, g_join_link_up);
	if (r > 0 && r < cap - off) {
		off += r;
	}
	r = snprintf(out + off, (size_t)(cap - off),
		"netup: dhcp offer=%d ack=%d bound_ip=%u.%u.%u.%u serverid=%u.%u.%u.%u\n",
		g_dhcp_offered, g_dhcp_ack_seen,
		g_dhcp_bound[0], g_dhcp_bound[1], g_dhcp_bound[2], g_dhcp_bound[3],
		g_dhcp_serverid[0], g_dhcp_serverid[1], g_dhcp_serverid[2], g_dhcp_serverid[3]);
	if (r > 0 && r < cap - off) {
		off += r;
	}
	r = snprintf(out + off, (size_t)(cap - off), "netup: RESULT %s\n",
		(g_join_setssid_status != 0 || g_join_psksup_status != 6) ? "JOIN-FAILED" :
			(g_dhcp_ack_seen ? "BOUND" : (g_dhcp_offered ? "OFFER-ONLY" : "NO-OFFER")));
	if (r > 0 && r < cap - off) {
		off += r;
	}
	return off;
}


/* Parse "<cmd> <ssid> <psk>" out of a write payload that is NOT NUL-terminated:
 * the ssid is the first space-separated token, the psk is the rest of the line
 * (a WPA2 passphrase may contain spaces). An over-long ssid tail is discarded. */
static void wifi_parseSsidPsk(const void *data, size_t size, size_t skip,
	char *ssid, size_t ssid_cap, char *psk, size_t psk_cap)
{
	char line[160];
	size_t n, i = 0, sn = 0, pn = 0;

	ssid[0] = '\0';
	psk[0] = '\0';
	if (size <= skip) {
		return;
	}
	n = size - skip;
	if (n > (sizeof(line) - 1)) {
		n = sizeof(line) - 1;
	}
	memcpy(line, (const char *)data + skip, n);
	line[n] = '\0';
	while ((n > 0) && ((line[n - 1] == '\n') || (line[n - 1] == '\r') || (line[n - 1] == ' '))) {
		line[--n] = '\0';
	}
	while (line[i] == ' ') {
		i++;
	}
	while ((line[i] != '\0') && (line[i] != ' ') && (sn < (ssid_cap - 1))) {
		ssid[sn++] = line[i++];
	}
	ssid[sn] = '\0';
	while ((line[i] != '\0') && (line[i] != ' ')) {
		i++; /* discard an over-long ssid tail */
	}
	while (line[i] == ' ') {
		i++;
	}
	while ((line[i] != '\0') && (pn < (psk_cap - 1))) {
		psk[pn++] = line[i++];
	}
	psk[pn] = '\0';
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


/* ---- /dev/wifidata: the raw-frame seam for an lwip netif ----------------
 *
 * /dev/wifi (id 0) is unchanged -- scan/join/netup plus a text result. This
 * second device carries frames:
 *
 *   write(fd, eth_frame, len)  -> transmit it            (netif->linkoutput)
 *   read(fd, buf, cap)         -> next RX frame, 0 if none queued
 *
 * Why a device instead of moving the driver into the lwip process: the whole
 * SDIO/SDPCM stack lives here and ONE process must own the bus, because
 * control, event and data frames share a single F2 FIFO -- two drainers would
 * steal each other's frames. This keeps the bus owner intact and hands lwip a
 * frame pipe; the WiFi ceiling (a few MB/s over SDIO) leaves ample room for one
 * message per frame.
 *
 * read() MUST NOT block: this daemon has a single message thread, so blocking
 * in a read would stall every other request, TX included.
 */
static int wifi_frameWrite(const void *data, size_t len)
{
	if (g_sdhci == NULL) {
		return -EIO;
	}
	if ((len < 14u) || (len > (size_t)(F2_FRAME_MAX - 16u))) {
		return -EINVAL;
	}
	{
		WIFI_T0(t0);
		int rc;

		rc = diag_wifiFrameTx(g_sdhci, (const uint8_t *)data, (uint32_t)len);
		WIFI_ACC(g_tx_us, t0);
		g_tx_calls++;
		if (rc != 0) {
			return -EIO;
		}
	}
	return (int)len;
}


static int wifi_frameRead(void *dst, size_t cap)
{
	static uint8_t frame[F2_FRAME_MAX];
	uint32_t elen = 0u;

	if (g_sdhci == NULL) {
		return -EIO;
	}
	/* One probe, no lock, no wait: measured fastest. See the WIFI_RX_* block
	 * for the full comparison -- a waiting read, a spin, and a bus mutex were
	 * all tried on hardware and every one of them cost more than it saved. */
	{
		WIFI_T0(t0);
		int rc = diag_wifiFrameRx(g_sdhci, frame, sizeof(frame), &elen);
		if (rc != 0) {
			WIFI_ACC(g_rx_miss_us, t0);
			g_rx_misses++;
			return 0; /* nothing queued, or a non-data frame was drained */
		}
		WIFI_ACC(g_rx_hit_us, t0);
		g_rx_hits++;
	}

	if ((size_t)elen > cap) {
		return -EMSGSIZE;
	}
	memcpy(dst, frame, elen);
	return (int)elen;
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

		/* Text commands drive the same bus (and the same g_txf/g_rxf), so they
		 * take the lock for their whole duration. A join legitimately holds it
		 * for tens of seconds; that was equally true single-threaded. */
		if (msg.oid.id == WIFI_DEV_DATA_ID) {
			switch (msg.type) {
				case mtOpen:
				case mtClose:
					msg.o.err = EOK;
					break;

				case mtWrite:
					msg.o.err = wifi_frameWrite(msg.i.data, msg.i.size);
					break;

				case mtRead:
					msg.o.err = wifi_frameRead(msg.o.data, msg.o.size);
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
			continue;
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
				 * scan, which loads the CLM channel data); write("netup <ssid>
				 * <psk>") runs the WPA2-PSK join plus the full DHCP exchange;
				 * write("mtu") proves the data path at full MTU after that. A
				 * client read()s the result. Any other payload is accepted but
				 * ignored. */
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
				else if (msg.i.size >= 7 && memcmp(msg.i.data, "netup ", 6) == 0) {
					char ssid[33], psk[64];
					wifi_parseSsidPsk(msg.i.data, msg.i.size, 6, ssid, sizeof(ssid), psk, sizeof(psk));
					g_resp_len = wifi_netup(ssid, psk, g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 9 && memcmp(msg.i.data, "joinwpa ", 8) == 0) {
					/* join + 4-way key only; the caller (an lwip netif) runs DHCP */
					char ssid[33], psk[64];
					wifi_parseSsidPsk(msg.i.data, msg.i.size, 8, ssid, sizeof(ssid), psk, sizeof(psk));
					g_resp_len = wifi_joinwpa(ssid, psk, g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 5 && memcmp(msg.i.data, "stats", 5) == 0) {
					g_resp_len = wifi_stats(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 3 && memcmp(msg.i.data, "mac", 3) == 0) {
					g_resp_len = wifi_mac(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 3 && memcmp(msg.i.data, "mtu", 3) == 0) {
					/* full-MTU data-path proof; needs a prior successful netup */
					g_resp_len = wifi_mtu(g_resp, (int)sizeof(g_resp));
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
	dev.id = WIFI_DEV_TEXT_ID;
	if (create_dev(&dev, "wifi") < 0) {
		printf("rpi4-wifi: could not create /dev/wifi\n");
		return 4;
	}
	dev.id = WIFI_DEV_DATA_ID;
	if (create_dev(&dev, "wifidata") < 0) {
		/* Not fatal: the text device still works, only the netif seam is gone. */
		printf("rpi4-wifi: WARNING could not create /dev/wifidata (frame seam unavailable)\n");
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
