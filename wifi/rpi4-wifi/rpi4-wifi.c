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
 * SDPCM function-2 enable), then serves /dev/wifi and /dev/wifidata plus
 * /dev/wifibatch (the frame seam of the lwip wifi43455 netif, one frame or a
 * batch of frames per message; the netif joins the network named in
 * /etc/wifi.conf). The firmware, NVRAM and CLM come from /lib/firmware/brcm/
 * (see WIFI_FW_*). The image starts it at boot with -f; started from the shell
 * it forks and returns once /dev/wifi is up.
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
 * The VideoCore mailbox is not touched directly: the WL_REG_ON power cycle
 * (SET/GET_GPIO_STATE) and the SDHCI reference clock rate (GET_CLOCK_RATE,
 * EMMC clock) go through the rpi4-vcmbox server (/dev/vcmbox).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include "libvcmbox.h"
#include "wifibatch.h"

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <ctype.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/* Firmware files. The BCM43455 needs three vendor files, read from the root
 * file system at start-up under the names and in the layout Linux brcmfmac and
 * linux-firmware use. They are not part of this source tree (the firmware and
 * the CLM blob are under the Cypress licence, /lib/firmware/LICENSES/); the
 * image build fetches them, pinned by commit and sha256. */

#define WIFI_FW_DIR       "/lib/firmware/brcm/"
#define WIFI_FW_BIN       WIFI_FW_DIR "brcmfmac43455-sdio.bin"
#define WIFI_FW_CLM       WIFI_FW_DIR "brcmfmac43455-sdio.clm_blob"
/* NVRAM: the board-specific file first, then the generic name, as brcmfmac does. */
#define WIFI_FW_NVRAM     WIFI_FW_DIR "brcmfmac43455-sdio.raspberrypi,4-model-b.txt"
#define WIFI_FW_NVRAM_ALT WIFI_FW_DIR "brcmfmac43455-sdio.txt"

#define WIFI_FW_BIN_MAX   (1024u * 1024u) /* 643651 B in the pinned release */
#define WIFI_FW_CLM_MAX   (64u * 1024u)   /* 4733 B */
#define WIFI_FW_NVRAM_MAX (16u * 1024u)   /* 1883 B of text */

/* With -f (the boot launch) the daemon can start before the root file system
 * holding /lib/firmware is mounted -- on nfsroot it is taken over after lwip
 * brings the network up -- so it waits this long for the files to appear. */
#define WIFI_FW_WAIT_S    60

typedef struct {
	uint8_t *data;
	size_t len;
} wifi_blob_t;

static wifi_blob_t g_fw;    /* the CR4 firmware image, loaded verbatim into SOCRAM */
static wifi_blob_t g_nvram; /* the NVRAM in the chip's format (wifi_nvramPack) */
static wifi_blob_t g_clm;   /* the CLM regulatory blob, sent with the "clmload" iovar */


/* Read a whole file of at most `max` bytes into a new buffer. */
static int wifi_readFile(const char *path, size_t max, wifi_blob_t *out)
{
	struct stat st;
	uint8_t *data;
	size_t len = 0u;
	ssize_t n;
	int fd, err = 0;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		return -errno;
	}
	if (fstat(fd, &st) != 0) {
		err = -errno;
	}
	else if ((st.st_size <= 0) || ((size_t)st.st_size > max)) {
		err = -EFBIG;
	}
	if (err != 0) {
		close(fd);
		return err;
	}

	data = malloc((size_t)st.st_size);
	if (data == NULL) {
		close(fd);
		return -ENOMEM;
	}
	while (len < (size_t)st.st_size) {
		n = read(fd, data + len, (size_t)st.st_size - len);
		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}
			err = -errno;
			break;
		}
		if (n == 0) {
			err = -EIO; /* shorter than fstat() said */
			break;
		}
		len += (size_t)n;
	}
	close(fd);
	if (err != 0) {
		free(data);
		return err;
	}

	out->data = data;
	out->len = len;
	return 0;
}


/* Convert the NVRAM text (key=value lines, '#' comments) into the image the
 * chip's boot loader expects at the top of its RAM, as brcmfmac does:
 *   - each non-blank, non-comment line, whitespace-trimmed, NUL-terminated;
 *   - one more NUL, then zeros to a 4-byte boundary;
 *   - more zeros so that the image INCLUDING the trailer is a multiple of 64
 *     bytes, which lets the loader write it as whole 64-byte CMD53 blocks (the
 *     zeros land between the variables and the trailer; the firmware accepts
 *     that);
 *   - a 4-byte little-endian trailer (~words << 16) | words, where words is
 *     the length before the trailer in 32-bit words.
 * Byte-identical to scripts/gen-wifi-nvram-py.py, which produced the image the
 * driver shipped with before it read the file at run time. */
static int wifi_nvramPack(const uint8_t *text, size_t len, wifi_blob_t *out)
{
	uint8_t *img;
	size_t pos = 0u, n = 0u, b, e;
	uint32_t words, token;

	/* Worst case: every byte kept, plus the two NULs, the 4-byte and the
	 * 64-byte padding and the trailer. */
	img = calloc(1u, len + 2u + 3u + 63u + 4u);
	if (img == NULL) {
		return -ENOMEM;
	}

	while (pos < len) {
		b = pos;
		while ((pos < len) && (text[pos] != '\n')) {
			pos++;
		}
		e = pos;
		pos++; /* past the '\n' */

		while ((b < e) && (isspace(text[b]) != 0)) {
			b++;
		}
		while ((e > b) && (isspace(text[e - 1u]) != 0)) {
			e--;
		}
		if ((b == e) || (text[b] == '#')) {
			continue;
		}
		memcpy(img + n, text + b, e - b);
		n += e - b;
		img[n++] = 0u;
	}
	img[n++] = 0u;                             /* the list terminator */
	n = (n + 3u) & ~(size_t)3u;                /* 4-byte boundary */
	n = ((n + 4u + 63u) & ~(size_t)63u) - 4u;  /* image + trailer = k * 64 */

	words = (uint32_t)(n / 4u);
	token = ((~words & 0xffffu) << 16) | (words & 0xffffu);
	img[n++] = (uint8_t)(token & 0xffu);
	img[n++] = (uint8_t)((token >> 8) & 0xffu);
	img[n++] = (uint8_t)((token >> 16) & 0xffu);
	img[n++] = (uint8_t)((token >> 24) & 0xffu);

	out->data = img;
	out->len = n;
	return 0;
}


/* Load the three firmware files, once: every bring-up (and each retry) reuses
 * them. Returns 0, or a negative errno with the file that failed in *failed. */
static int wifi_fwLoad(const char **failed)
{
	static int loaded = 0;
	wifi_blob_t text = { NULL, 0u };
	int err;

	if (loaded != 0) {
		return 0;
	}

	*failed = WIFI_FW_BIN;
	err = wifi_readFile(WIFI_FW_BIN, WIFI_FW_BIN_MAX, &g_fw);
	if (err == 0) {
		*failed = WIFI_FW_NVRAM;
		err = wifi_readFile(WIFI_FW_NVRAM, WIFI_FW_NVRAM_MAX, &text);
		if (err == -ENOENT) {
			*failed = WIFI_FW_NVRAM_ALT;
			err = wifi_readFile(WIFI_FW_NVRAM_ALT, WIFI_FW_NVRAM_MAX, &text);
		}
	}
	if (err == 0) {
		err = wifi_nvramPack(text.data, text.len, &g_nvram);
		free(text.data);
	}
	if (err == 0) {
		*failed = WIFI_FW_CLM;
		err = wifi_readFile(WIFI_FW_CLM, WIFI_FW_CLM_MAX, &g_clm);
	}
	if (err != 0) {
		free(g_fw.data);
		free(g_nvram.data);
		g_fw.data = NULL;
		g_nvram.data = NULL;
		return err;
	}

	printf("rpi4-wifi: loaded %s (%zu B), clm_blob (%zu B), nvram (%zu B chip image)\n",
		WIFI_FW_BIN, g_fw.len, g_clm.len, g_nvram.len);
	loaded = 1;
	return 0;
}

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
/* WL_REG_ON, the chip's power line, is a Pi 4 expander GPIO owned by the
 * VideoCore firmware, set and read with property calls. They go through the
 * rpi4-vcmbox server like every other mailbox call: the FIFO has no hardware
 * arbitration, and this daemon starts at boot, while other mailbox clients
 * (thermal, the display and GPU servers) are busy. */

#define VC_PROP_SET_GPIO_STATE  0x00038041u
#define VC_PROP_GET_GPIO_STATE  0x00030041u

#define EXPGPIO_WL_ON           129u  /* expgpio[1] = "WL_ON" per Pi 4 DT */

/* Set (SET_GPIO_STATE) or read (GET_GPIO_STATE) an expander GPIO. Returns the
 * state the firmware reports, or 0xFFFFFFFF on failure. */
static uint32_t diag_expGpio(uint32_t tag, uint32_t gpio, uint32_t state)
{
	uint32_t in[2] = { gpio, state };
	uint32_t out[2] = { 0u, 0u };

	if (vcmbox_call(tag, sizeof(in), in, 2u, out, 2u) != 0) {
		return 0xFFFFFFFFu;
	}
	return out[1];
}

/* Cold-power-cycle the BCM43455 WiFi chip via its WL_REG_ON line (a Pi 4
 * expander GPIO driven through the VideoCore mailbox): drop it, wait,
 * re-assert, settle. NB: a 20x-longer power-down was tested and did NOT
 * make the 43455 firmware execute (the fw-exec gate is not a reset-timing
 * issue); 50/150 ms is the established, enumeration-tested baseline. */
static void diag_sbwinForget(void);

static void diag_wifiPowerCycle(void)
{
	diag_sbwinForget(); /* the chip comes back with its reset window */

	/* Report a failed toggle instead of discarding it. diag_expGpio() returns
	 * 0xFFFFFFFF when the mailbox call fails, and dropping that made the
	 * consequence appear far downstream as a chip that
	 * "does not respond" -- the shape of a hardware-marginality story rather than
	 * of a diagnosable failure. Read the line back afterwards, as rpi4-hci does,
	 * so the log says whether the radio is actually powered. */
	if (diag_expGpio(VC_PROP_SET_GPIO_STATE, EXPGPIO_WL_ON, 0u) == 0xFFFFFFFFu) {
		printf("rpi4-wifi: WL_REG_ON off failed (mailbox); chip may not reset\n");
	}
	usleep(50 * 1000);
	if (diag_expGpio(VC_PROP_SET_GPIO_STATE, EXPGPIO_WL_ON, 1u) == 0xFFFFFFFFu) {
		printf("rpi4-wifi: WL_REG_ON on failed (mailbox); chip will not power up\n");
	}
	usleep(150 * 1000);

	printf("rpi4-wifi: WL_REG_ON readback=%d (expect 1)\n",
		(int)diag_expGpio(VC_PROP_GET_GPIO_STATE, EXPGPIO_WL_ON, 0u));
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

#define SDHCI_INT_STATUS_EN 0x34u        /* Normal + Error Interrupt Status Enable */
#define SDHCI_SIGNAL_EN     0x38u        /* Normal + Error Interrupt Signal Enable */
#define SDHCI_INT_CARD      0x00000100u  /* Card Interrupt: the SDIO card's DAT1 line, read-only */

/* The Arasan controller is clocked by the EMMC clock (firmware clock id 1;
 * Linux DT: BCM2835_CLOCK_EMMC), a CPRMAN peripheral clock the firmware may
 * derive from the same PLL as the core clock -- so ask for its rate rather
 * than assume one. 250 MHz is the rate observed with core_freq=250 and is
 * only the fallback when the mailbox query fails. */
#define VC_PROP_GET_CLOCK_RATE          0x00030002u
#define VC_PROP_GET_CLOCK_RATE_MEASURED 0x00030047u
#define VC_CLOCK_EMMC          1u
#define SDHCI_BASE_HZ_DEFAULT  250000000u

/* The EMMC clock as the firmware MEASURES it (0 if the query failed). The rate
 * GET_CLOCK_RATE reports is the one the firmware set; the measured one is what
 * the CPRMAN counter sees, so a PLL that moved underneath the setting (e.g. with
 * core_freq) would show up as a difference between the two. */
static uint32_t g_sdhci_measured_hz;

/* The SD bus clock as last programmed: the requested target and what the
 * divider actually gives. */
static unsigned g_sdclk_khz;
static uint32_t g_sdhci_sd_hz;

/* Reference clock of the SDHCI divider, queried once through /dev/vcmbox. */
static uint32_t diag_sdhciBaseHz(void)
{
	static uint32_t base_hz;
	uint32_t in[2] = { VC_CLOCK_EMMC, 0u };
	uint32_t out[2] = { 0u, 0u };

	if (base_hz != 0u) {
		return base_hz;
	}

	if ((vcmbox_call(VC_PROP_GET_CLOCK_RATE, sizeof(in), in, 2u, out, 2u) == 0) && (out[1] != 0u)) {
		base_hz = out[1];
		printf("rpi4-wifi: SDHCI base clock (EMMC) = %u Hz\n", (unsigned)base_hz);
	}
	else {
		base_hz = SDHCI_BASE_HZ_DEFAULT;
		printf("rpi4-wifi: GET_CLOCK_RATE(EMMC) failed; assuming %u Hz\n", (unsigned)base_hz);
	}

	out[0] = 0u;
	out[1] = 0u;
	if (vcmbox_call(VC_PROP_GET_CLOCK_RATE_MEASURED, sizeof(in), in, 2u, out, 2u) == 0) {
		g_sdhci_measured_hz = out[1];
	}

	return base_hz;
}

/* Program SDHCI to a target SD-bus clock by dividing the EMMC base clock.
 * Per SDHCI 3.0 §2.2.13: divisor is 10-bit, output_hz = base / (2*N). */
static int diag_sdhciSetClockKHz(volatile uint8_t *base, unsigned target_khz)
{
	uint32_t base_hz = diag_sdhciBaseHz();
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

	g_sdclk_khz = target_khz;
	g_sdhci_sd_hz = base_hz / (2u * divisor);

	/* The whole derivation on one line, so a boot log shows the bus clock the
	 * card actually got rather than the one that was asked for. */
	printf("rpi4-wifi: SDIO-CLK target=%u kHz base=%u Hz measured=%u Hz div=%u sd=%u Hz\n",
		target_khz, (unsigned)base_hz, (unsigned)g_sdhci_measured_hz, (unsigned)divisor,
		(unsigned)g_sdhci_sd_hz);

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

/* The chip's backplane window (F1 SBADDR low/mid/high, 0x1000A..0x1000C) as
 * last written. Every F2 frame, and every empty RX probe, used to rewrite all
 * three bytes first -- three CMD52s that usually rewrote the value already
 * there. brcmfmac keeps the same shadow (sdiodev->sbwad in bcmsdh.c) and writes
 * the window only when it changes. Kept up to date by every CMD52 write, so the
 * many places that set the window by hand stay correct; forgotten when the chip
 * is power-cycled or a write to it failed. */
#define SBSDIO_FUNC1_SBADDRLOW 0x1000Au
static uint8_t g_sbwin[3];
static uint8_t g_sbwin_valid[3];
static uint32_t g_sbwin_writes = 0u, g_sbwin_skips = 0u;

static void diag_sbwinForget(void)
{
	g_sbwin_valid[0] = 0u;
	g_sbwin_valid[1] = 0u;
	g_sbwin_valid[2] = 0u;
}

/* CMD52 (IO_RW_DIRECT). arg layout: bit31 R/W, bits30:28 FN, bits25:9
 * 17-bit REG, bits7:0 DATA. resp_out is NULL or a 4-element uint32_t array
 * (diag_sdhciCmd fills all four response slots). */
static int diag_sdioCmd52(volatile uint8_t *sdhci, int write, int fn,
	uint32_t reg, uint8_t data, uint32_t *resp_out)
{
	uint32_t arg = 0;
	int rc;

	arg |= (write ? 1u : 0u) << 31;
	arg |= ((uint32_t)fn & 7u) << 28;
	arg |= ((uint32_t)reg & 0x1ffffu) << 9;
	if (write) {
		arg |= (uint32_t)data;
	}
	rc = diag_sdhciCmd(sdhci, 52u, arg, SDHCI_RESP_R5, resp_out);
	if ((write != 0) && (fn == 1) && (reg >= SBSDIO_FUNC1_SBADDRLOW) && (reg < (SBSDIO_FUNC1_SBADDRLOW + 3u))) {
		g_sbwin[reg - SBSDIO_FUNC1_SBADDRLOW] = data;
		g_sbwin_valid[reg - SBSDIO_FUNC1_SBADDRLOW] = (rc == 0) ? 1u : 0u;
	}
	return rc;
}

/* Switch the card to high-speed timing on a 4-bit data bus, at 25 MHz. Call
 * after CMD0/5/3/7 + F1 enable + IORDY. Sequence per BCM43455c0 / SDIO 2.0:
 * CCCR 0x13 SHS check + EHS set, CCCR 0x07 4-bit width, SDHCI HCTL1
 * 4BIT+HIGH_SPEED, reprogram clock to 25 MHz. diag_sdioClockUp() raises the
 * clock later, once the backplane can hold its test pattern. */
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

/* ---- Waiting on the controller -------------------------------------------
 *
 * Every wait in this file is a fixed count of 100000 register reads. That is a
 * budget in bus cycles, not in time: an uncached read of this controller
 * crosses the VideoCore peripheral bus, so the same count may last only about
 * half as long at core_freq=500 as it did at 250. SDHCI-POLL below prints what
 * it lasts on the running board. The block-mode CMD53 helpers now fall back to
 * a wall-clock bound when the spin runs out.
 *
 * Their PIO loops also had an ordering hazard. They waited on the LATCHED
 * buffer-ready bit in INT_STATUS before every word and cleared it after each
 * block. If the controller raises "ready" for the next block before that clear
 * lands -- the thread is interrupted or preempted between the last word and the
 * clear while the previous block drains -- the clear erases it, nothing sets it
 * again, and the loop times out with no error bit. That is rc -5 from
 * diag_sdioCmd53Write, the signature of all three natural firmware-download
 * failures in the log archive; whether it is also their cause is what
 * `fwloadbench` measures. Linux's driver for this controller (bcm2835-mmc.c,
 * bcm2835_mmc_transfer_pio) acknowledges first and then tests the LEVEL bits in
 * PRESENT_STATE, which a lost edge cannot hide. That is the default here now.
 * The old loop is kept, selectable at run time, only so `fwloadbench` can
 * compare the two in one boot; remove it once that comparison is graded. */
#define SDHCI_PRES_SPACE_AVAILABLE 0x00000400u /* Buffer Write Enable (level) */
#define SDHCI_PRES_DATA_AVAILABLE  0x00000800u /* Buffer Read Enable (level) */
#define SDHCI_SPIN_ITERS    100000u /* the historical budget, in register reads */
#define SDHCI_SLOW_WAIT_US  100000u /* wall-clock grace once the spin runs out */

static int g_pio_legacy = 0;          /* 1 = the old latched-bit loop, spin budget only */
static uint32_t g_pio_slow_waits = 0; /* waits that outlasted the spin and then succeeded */
static uint32_t g_pio_slow_max_us = 0;
static uint32_t g_pio_timeouts = 0;
static uint32_t g_pio_to_pres = 0, g_pio_to_int = 0; /* registers at the last timeout */

static uint64_t diag_monoUs(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		return 0u;
	}
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

/* 0 when (reg & mask) != 0, -1 when INT_STATUS reports an error first, -2 on
 * timeout: the spin budget, then (unless legacy) up to SDHCI_SLOW_WAIT_US of
 * wall-clock time, checking once more after the deadline so that being
 * preempted across it cannot fake a timeout. */
static int diag_sdhciWaitSet(volatile uint8_t *sdhci, uint32_t reg, uint32_t mask)
{
	uint64_t t0, dt;
	uint32_t i, st;
	int last;

	/* One read per iteration when waiting on INT_STATUS itself, exactly as the
	 * loops this replaces did, so the legacy mode keeps their timing. */
	for (i = 0; i < SDHCI_SPIN_ITERS; ++i) {
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -1;
		}
		if ((((reg == SDHCI_INT_STATUS) ? st : *(volatile uint32_t *)(sdhci + reg)) & mask) != 0u) {
			return 0;
		}
	}
	if (g_pio_legacy != 0) {
		return -2;
	}

	t0 = diag_monoUs();
	do {
		dt = diag_monoUs() - t0;
		last = (dt >= SDHCI_SLOW_WAIT_US);
		st = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
		if ((st & SDHCI_INT_ERR_ANY) != 0u) {
			return -1;
		}
		if ((((reg == SDHCI_INT_STATUS) ? st : *(volatile uint32_t *)(sdhci + reg)) & mask) != 0u) {
			g_pio_slow_waits++;
			if (dt > g_pio_slow_max_us) {
				g_pio_slow_max_us = (uint32_t)dt;
			}
			return 0;
		}
	} while (last == 0);
	return -2;
}

/* Record, and report the first few of, the PIO waits that timed out. SPACE/DATA
 * are the level bits, WR/RD the latched ones: level=1 with latched=0 is a lost
 * edge; level=0 means the controller really had no room (or no data). */
static void diag_pioNoteTimeout(volatile uint8_t *sdhci, const char *dir, uint32_t blk, uint32_t nblk)
{
	g_pio_to_pres = *(volatile uint32_t *)(sdhci + SDHCI_PRES_STATE);
	g_pio_to_int = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
	if (g_pio_timeouts++ < 8u) {
		printf("rpi4-wifi: SDHCI-TIMEOUT dir=%s mode=%s blk=%u/%u pres=0x%08x int=0x%08x "
			"space=%u data=%u wr_rdy=%u rd_rdy=%u\n",
			dir, (g_pio_legacy != 0) ? "legacy" : "level", (unsigned)blk, (unsigned)nblk,
			(unsigned)g_pio_to_pres, (unsigned)g_pio_to_int,
			(unsigned)((g_pio_to_pres >> 10) & 1u), (unsigned)((g_pio_to_pres >> 11) & 1u),
			(unsigned)((g_pio_to_int >> 4) & 1u), (unsigned)((g_pio_to_int >> 5) & 1u));
	}
}

/* Time the spin budget once: how long SDHCI_SPIN_ITERS reads of INT_STATUS
 * take on this board, at this core clock. */
static void diag_sdhciPollCalibrate(volatile uint8_t *sdhci)
{
	static int done = 0;
	uint64_t t0, dt;
	uint32_t i;

	if (done != 0) {
		return;
	}
	done = 1;
	t0 = diag_monoUs();
	for (i = 0; i < SDHCI_SPIN_ITERS; ++i) {
		(void)*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS);
	}
	dt = diag_monoUs() - t0;
	printf("rpi4-wifi: SDHCI-POLL spin=%u reads took %u us (%u ns/read)\n",
		(unsigned)SDHCI_SPIN_ITERS, (unsigned)dt,
		(unsigned)((dt * 1000u) / SDHCI_SPIN_ITERS));
}

/* One DATA_PORT word to/from a little-endian byte buffer (NULL = discard). */
static void diag_pioReadWord(volatile uint8_t *sdhci, uint8_t *buf, uint32_t i)
{
	uint32_t data = *(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT);

	if (buf != NULL) {
		buf[i * 4 + 0] = (uint8_t)(data & 0xffu);
		buf[i * 4 + 1] = (uint8_t)((data >> 8) & 0xffu);
		buf[i * 4 + 2] = (uint8_t)((data >> 16) & 0xffu);
		buf[i * 4 + 3] = (uint8_t)((data >> 24) & 0xffu);
	}
}

static void diag_pioWriteWord(volatile uint8_t *sdhci, const uint8_t *buf, uint32_t i)
{
	*(volatile uint32_t *)(sdhci + SDHCI_DATA_PORT) = (uint32_t)buf[i * 4 + 0] |
		((uint32_t)buf[i * 4 + 1] << 8) |
		((uint32_t)buf[i * 4 + 2] << 16) |
		((uint32_t)buf[i * 4 + 3] << 24);
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
	uint32_t bytes_total = block_count * block_size;
	uint32_t words_total = bytes_total / 4u;
	uint32_t wpb = block_size / 4u; /* words per block */
	uint32_t bytes_in_block = 0;
	uint32_t i, blk;
	int deadline, rc;

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

	rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_CMD_COMPLETE);
	if (rc != 0) {
		return (rc == -1) ? -2 : -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	if (g_pio_legacy != 0) {
		/* The old loop: latched ready bit before every word, cleared after each
		 * block (see "Waiting on the controller"). */
		for (i = 0; i < words_total; ++i) {
			rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_BUF_RD_READY);
			if (rc != 0) {
				if (rc == -2) {
					diag_pioNoteTimeout(sdhci, "rd", i / wpb, block_count);
				}
				return (rc == -1) ? -4 : -5;
			}
			diag_pioReadWord(sdhci, buf, i);
			bytes_in_block += 4u;
			if (bytes_in_block >= block_size) {
				bytes_in_block = 0u;
				*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_RD_READY;
			}
		}
	}
	else {
		/* Acknowledge, then wait for the LEVEL bit, then drain one block. */
		for (blk = 0; blk < block_count; ++blk) {
			*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_RD_READY;
			rc = diag_sdhciWaitSet(sdhci, SDHCI_PRES_STATE, SDHCI_PRES_DATA_AVAILABLE);
			if (rc != 0) {
				if (rc == -2) {
					diag_pioNoteTimeout(sdhci, "rd", blk, block_count);
				}
				return (rc == -1) ? -4 : -5;
			}
			for (i = blk * wpb; i < (blk + 1u) * wpb; ++i) {
				diag_pioReadWord(sdhci, buf, i);
			}
		}
	}

	rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_XFER_COMPLETE);
	if (rc != 0) {
		return (rc == -1) ? -6 : -7;
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
 * read: arg bit31 = 1, TRANSFER_MODE bit4 = 0, waits for buffer space (see
 * "Waiting on the controller"), writes DATA_PORT. Source is a little-endian
 * byte buffer of at least block_count*block_size bytes. */
static int diag_sdioCmd53Write(volatile uint8_t *sdhci, int fn,
	int incr_addr, uint32_t reg_addr,
	uint32_t block_count, uint32_t block_size,
	const uint8_t *buf)
{
	uint32_t arg, cmd_word;
	uint32_t bytes_total = block_count * block_size;
	uint32_t words_total = bytes_total / 4u;
	uint32_t wpb = block_size / 4u; /* words per block */
	uint32_t bytes_in_block = 0;
	uint32_t i, blk;
	int deadline, rc;

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

	rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_CMD_COMPLETE);
	if (rc != 0) {
		return (rc == -1) ? -2 : -3;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_CMD_COMPLETE;

	if (g_pio_legacy != 0) {
		/* The old loop: latched ready bit before every word, cleared after each
		 * block (see "Waiting on the controller"). */
		for (i = 0; i < words_total; ++i) {
			rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_BUF_WR_READY);
			if (rc != 0) {
				if (rc == -2) {
					diag_pioNoteTimeout(sdhci, "wr", i / wpb, block_count);
				}
				return (rc == -1) ? -4 : -5;
			}
			diag_pioWriteWord(sdhci, buf, i);
			bytes_in_block += 4u;
			if (bytes_in_block >= block_size) {
				bytes_in_block = 0u;
				*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_WR_READY;
			}
		}
	}
	else {
		/* Acknowledge, then wait for the LEVEL bit, then fill one block. */
		for (blk = 0; blk < block_count; ++blk) {
			*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = SDHCI_INT_BUF_WR_READY;
			rc = diag_sdhciWaitSet(sdhci, SDHCI_PRES_STATE, SDHCI_PRES_SPACE_AVAILABLE);
			if (rc != 0) {
				if (rc == -2) {
					diag_pioNoteTimeout(sdhci, "wr", blk, block_count);
				}
				return (rc == -1) ? -4 : -5;
			}
			for (i = blk * wpb; i < (blk + 1u) * wpb; ++i) {
				diag_pioWriteWord(sdhci, buf, i);
			}
		}
	}

	rc = diag_sdhciWaitSet(sdhci, SDHCI_INT_STATUS, SDHCI_INT_XFER_COMPLETE);
	if (rc != 0) {
		return (rc == -1) ? -6 : -7;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) = 0xFFFFFFFFu;
	return 0;
}

/* ---- SDIO high speed: a data clock above 25 MHz ----------------------------
 *
 * diag_sdioGoHighSpeed() selects high-speed timing on the card (CCCR EHS), which
 * allows up to 50 MHz, but the bus always ran at 25 MHz, the default-speed limit.
 * Every byte of firmware and every frame crosses this bus, and at 25 MHz the
 * transfers are close to wire-bound: the 643 648-byte download takes 67 ms against
 * 51.5 ms of pure wire time, and a 4 KB CMD53 429 us against 328 us.
 *
 * This divider only gives base / (2 N), so from the 250 MHz EMMC clock the fastest
 * rate within 50 MHz is N = 3, 41.67 MHz. That is the rate Linux runs this chip at
 * on the Pi (bcm2835_mmc_set_clock with brcm,overclock-50 = <0>, bcm270x.dtsi).
 * Above 25 MHz this driver also programs the host the way Linux does:
 *   - HOST_CONTROL's HISPD bit stays clear: Linux never sets it on this controller
 *     (SDHCI_QUIRK_NO_HISPD_BIT in sdhci-iproc.c; bcm2835_mmc_set_ios);
 *   - the data timeout is the maximum, 0xE, as bcm2835_mmc_prepare_data writes it
 *     before every data command. It counts SD clocks here
 *     (SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK), so a faster clock shortens it.
 * At 25 MHz nothing changes from before (HISPD set, the timeout the firmware left),
 * so `sdclk=25000` gives exactly the bus of older builds.
 *
 * The new clock is proven before the firmware goes over it. A 512-byte pattern is
 * written to SOCRAM and read back at 25 MHz, which checks the test itself. It is
 * read back again at the new clock, then its complement is written and read back
 * there. Every block carries a CRC both ways, so a marginal bus shows up as a
 * command error or a mismatch. Either one returns the bus to 25 MHz and the line
 * says why. The download that follows overwrites the pattern. */
#define SDIO_CLK_DS_KHZ    25000u  /* default speed: the bus of every earlier build */
#define SDIO_CLK_HS_KHZ    50000u  /* high speed: 250 MHz / 6 = 41.67 MHz here */
#define SDHCI_HOST_CTL     0x28u   /* HOST_CONTROL (low byte of the dword) */
#define SDHCI_HCTL_HISPD   0x04u
#define SDHCI_TIMEOUT_MAX  0xEu    /* TIMEOUT_CONTROL, byte 0x2E: TMCLK x 2^27 */
#define SDIO_CCCR_BUS_SPEED 0x13u
#define SDIO_CCCR_EHS      0x02u
#define SDIO_HS_TEST_LEN   512u
#define SOCRAM_BASE_43455  0x198000u

static unsigned g_sdclk_want_khz = SDIO_CLK_HS_KHZ; /* `sdclk=<kHz>` */
static unsigned g_sdclk_try_khz = SDIO_CLK_HS_KHZ;  /* this bring-up's target; a retry uses 25 MHz */
static int g_hs_hispd = 0;                /* `hispd=1`: keep HISPD set above 25 MHz as well */
static uint32_t g_hs_timeout_boot = 0xffu; /* TIMEOUT_CONTROL as found before the first raise */
static uint32_t g_hs_ups = 0u, g_hs_fallbacks = 0u;
static char g_hs_why[96];                 /* why the last raise fell back ("" = it did not) */
static uint8_t g_hs_pat[SDIO_HS_TEST_LEN] __attribute__((aligned(4)));
static uint8_t g_hs_rb[SDIO_HS_TEST_LEN] __attribute__((aligned(4)));

/* Program the host for a data clock of `khz`: HISPD, the data timeout, the
 * divider. The card side needs nothing: EHS, set at bring-up, allows any clock
 * up to 50 MHz. Between commands only. */
static int diag_sdhciDataClock(volatile uint8_t *sdhci, unsigned khz)
{
	volatile uint32_t *hctl = (volatile uint32_t *)(sdhci + SDHCI_HOST_CTL);
	volatile uint32_t *ctr = (volatile uint32_t *)(sdhci + SDHCI_CLK_TIMEOUT_RESET);
	uint32_t v;

	v = *hctl;
	if ((khz <= SDIO_CLK_DS_KHZ) || (g_hs_hispd != 0)) {
		v |= SDHCI_HCTL_HISPD;
	}
	else {
		v &= ~SDHCI_HCTL_HISPD;
	}
	*hctl = v;

	if (khz > SDIO_CLK_DS_KHZ) {
		/* The 32-bit write keeps CLOCK_CTL and leaves the reset bits at 0. */
		v = *ctr;
		if (g_hs_timeout_boot == 0xffu) {
			g_hs_timeout_boot = (v >> 16) & 0xfu;
		}
		*ctr = (v & 0x0000ffffu) | (SDHCI_TIMEOUT_MAX << 16);
	}
	return diag_sdhciSetClockKHz(sdhci, khz);
}

/* Fill g_hs_pat (xorshift32: every DAT line toggles, no period within 512 bytes),
 * complemented when `invert` is set. */
static void diag_hsPattern(int invert)
{
	uint32_t x = 0x2545f491u, i;

	for (i = 0; i < SDIO_HS_TEST_LEN; ++i) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		g_hs_pat[i] = (uint8_t)(((invert != 0) ? ~x : x) & 0xffu);
	}
}

/* Optionally write g_hs_pat to SOCRAM, then read it back and compare. 0 when the
 * pattern came back intact; otherwise -1 and the reason in g_hs_why. */
static int diag_hsRoundTrip(volatile uint8_t *sdhci, int write, const char *phase)
{
	const uint32_t nblk = SDIO_HS_TEST_LEN / 64u;
	uint32_t i;
	int rc;

	/* The window as the download sets it for its first 32 KB (0x198000). */
	rc = diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW, 0x80u, NULL);
	rc |= diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW + 1u, (uint8_t)((SOCRAM_BASE_43455 >> 16) & 0xffu), NULL);
	rc |= diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW + 2u, (uint8_t)((SOCRAM_BASE_43455 >> 24) & 0xffu), NULL);
	if (rc != 0) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: window CMD52 failed", phase);
		return -1;
	}
	if (write != 0) {
		rc = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1, SOCRAM_BASE_43455 & 0x7fffu, nblk, 64u, g_hs_pat);
		if (rc != 0) {
			(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: write rc=%d", phase, rc);
			return -1;
		}
	}
	memset(g_hs_rb, 0, sizeof(g_hs_rb));
	rc = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1, SOCRAM_BASE_43455 & 0x7fffu, nblk, 64u, g_hs_rb);
	if (rc != 0) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: read rc=%d", phase, rc);
		return -1;
	}
	for (i = 0; i < SDIO_HS_TEST_LEN; ++i) {
		if (g_hs_rb[i] != g_hs_pat[i]) {
			(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: byte %u read 0x%02x, wrote 0x%02x",
				phase, (unsigned)i, (unsigned)g_hs_rb[i], (unsigned)g_hs_pat[i]);
			return -1;
		}
	}
	return 0;
}

/* Raise the data clock from 25 MHz to `khz` at bring-up: the backplane clock is
 * up and the CR4 is halted, so SOCRAM is free until the download. Returns 0 at
 * the new clock; on any failure the bus is back at 25 MHz and -1 is returned.
 * Never fatal: the caller downloads the firmware either way. */
static int diag_sdioClockUp(volatile uint8_t *sdhci, unsigned khz)
{
	uint32_t cccr[4] = { 0 };
	int rc;

	g_hs_why[0] = '\0';
	if ((diag_sdioCmd52(sdhci, 0, 0, SDIO_CCCR_BUS_SPEED, 0u, cccr) != 0) ||
		((cccr[0] & SDIO_CCCR_EHS) == 0u)) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "CCCR 0x13 reads 0x%02x, EHS not set",
			(unsigned)(cccr[0] & 0xffu));
		rc = -2; /* nothing was changed */
	}
	else {
		diag_hsPattern(0);
		rc = diag_hsRoundTrip(sdhci, 1, "25MHz write+read");
		if (rc == 0) {
			rc = diag_sdhciDataClock(sdhci, khz);
			if (rc != 0) {
				(void)snprintf(g_hs_why, sizeof(g_hs_why), "clock rc=%d", rc);
			}
		}
		if (rc == 0) {
			rc = diag_hsRoundTrip(sdhci, 0, "fast read");
		}
		if (rc == 0) {
			diag_hsPattern(1);
			rc = diag_hsRoundTrip(sdhci, 1, "fast write+read");
		}
	}

	if (rc == 0) {
		g_hs_ups++;
		printf("rpi4-wifi: SDIO-HS on: sd=%u Hz hctl=0x%02x timeout=0x%x (was 0x%x) cccr13=0x%02x "
			"check=pass (%u B written+read at 25 MHz and at the new clock)\n",
			(unsigned)g_sdhci_sd_hz,
			(unsigned)(*(volatile uint32_t *)(sdhci + SDHCI_HOST_CTL) & 0xffu),
			(unsigned)((*(volatile uint32_t *)(sdhci + SDHCI_CLK_TIMEOUT_RESET) >> 16) & 0xfu),
			(unsigned)g_hs_timeout_boot, (unsigned)(cccr[0] & 0xffu), (unsigned)SDIO_HS_TEST_LEN);
		return 0;
	}

	g_hs_fallbacks++;
	if (rc != -2) {
		/* A transfer may have died mid-data: clear both ends before 25 MHz. */
		(void)diag_sdhciResetCmdDat(sdhci);
		(void)diag_sdhciDataClock(sdhci, SDIO_CLK_DS_KHZ);
		(void)diag_sdioCmd52(sdhci, 1, 0, 0x06u, 0x01u, NULL); /* CCCR abort, function 1 */
		(void)diag_sdhciResetCmdDat(sdhci);
	}
	printf("rpi4-wifi: SDIO-HS fallback: %s; the bus stays at %u kHz\n", g_hs_why,
		(unsigned)g_sdclk_khz);
	return -1;
}

/* The time each bus transfer takes, read from the generic timer (CNTVCT_EL0,
 * enabled for EL0 by the kernel): two register reads per frame, against the
 * clock_gettime pair the per-request timing once took. Full-size frames (> 1024
 * bytes) are what bulk TCP moves, so they are counted apart; an empty probe is
 * the 12-byte header read that finds nothing queued. */
static uint64_t g_bt_freq;
static uint64_t g_bt_tx_ticks, g_bt_rx_ticks, g_bt_empty_ticks;
static uint32_t g_bt_tx_n, g_bt_rx_n, g_bt_empty_n;
/* Glom superframes (SDPCM channel 3, not the descriptor): nearly all bulk RX
 * arrives this way, so rx_big alone says nothing about RX bus time. */
static uint64_t g_bt_glom_ticks, g_bt_glom_bytes;
static uint32_t g_bt_glom_n;

static inline uint64_t diag_ticks(void)
{
	uint64_t v;

	__asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
	return v;
}

static void diag_busTimeReset(void)
{
	g_bt_tx_ticks = 0u;
	g_bt_rx_ticks = 0u;
	g_bt_empty_ticks = 0u;
	g_bt_tx_n = 0u;
	g_bt_rx_n = 0u;
	g_bt_empty_n = 0u;
	g_bt_glom_ticks = 0u;
	g_bt_glom_bytes = 0u;
	g_bt_glom_n = 0u;
}

/* Mean of `ticks` over `n`, in tenths of a microsecond. */
static unsigned diag_busTimeAvg(uint64_t ticks, uint32_t n)
{
	if (g_bt_freq == 0u) {
		__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(g_bt_freq));
	}
	if ((n == 0u) || (g_bt_freq == 0u)) {
		return 0u;
	}
	return (unsigned)(((ticks / n) * 10000000u) / g_bt_freq);
}

static int diag_busTimeLine(char *out, int cap)
{
	unsigned tx = diag_busTimeAvg(g_bt_tx_ticks, g_bt_tx_n);
	unsigned rx = diag_busTimeAvg(g_bt_rx_ticks, g_bt_rx_n);
	unsigned em = diag_busTimeAvg(g_bt_empty_ticks, g_bt_empty_n);
	unsigned gl = diag_busTimeAvg(g_bt_glom_ticks, g_bt_glom_n);

	return snprintf(out, (size_t)cap,
		"WIFISTATS bustime sd=%u Hz tx_big=%u avg_us=%u.%u rx_big=%u avg_us=%u.%u empty=%u avg_us=%u.%u "
		"glom=%u avg_us=%u.%u avg_bytes=%u\n",
		(unsigned)g_sdhci_sd_hz, (unsigned)g_bt_tx_n, tx / 10u, tx % 10u,
		(unsigned)g_bt_rx_n, rx / 10u, rx % 10u, (unsigned)g_bt_empty_n, em / 10u, em % 10u,
		(unsigned)g_bt_glom_n, gl / 10u, gl % 10u,
		(g_bt_glom_n != 0u) ? (unsigned)(g_bt_glom_bytes / g_bt_glom_n) : 0u);
}

/* A read-only check of the bus while the firmware runs (SOCRAM may not be
 * written): CCCR 0x13 still selects high speed, and the image's first block in
 * SOCRAM -- the reset vectors, unchanged after release in every boot that logged
 * them -- reads back as the image. 0, or -1 with the reason in g_hs_why. */
static int diag_hsCheckLive(volatile uint8_t *sdhci, const char *phase)
{
	uint32_t cccr[4] = { 0 };
	uint32_t i;
	int rc;

	if ((g_fw.len < 64u) || (g_fw.data == NULL)) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: no firmware image to compare", phase);
		return -1;
	}
	if ((diag_sdioCmd52(sdhci, 0, 0, SDIO_CCCR_BUS_SPEED, 0u, cccr) != 0) ||
		((cccr[0] & SDIO_CCCR_EHS) == 0u)) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: CCCR 0x13 reads 0x%02x", phase,
			(unsigned)(cccr[0] & 0xffu));
		return -1;
	}
	rc = diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW, 0x80u, NULL);
	rc |= diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW + 1u, (uint8_t)((SOCRAM_BASE_43455 >> 16) & 0xffu), NULL);
	rc |= diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW + 2u, (uint8_t)((SOCRAM_BASE_43455 >> 24) & 0xffu), NULL);
	if (rc == 0) {
		memset(g_hs_rb, 0, 64u);
		rc = diag_sdioCmd53Read(sdhci, 1, /*incr=*/1, SOCRAM_BASE_43455 & 0x7fffu, 1u, 64u, g_hs_rb);
	}
	if (rc != 0) {
		(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: SOCRAM read rc=%d", phase, rc);
		return -1;
	}
	for (i = 0; i < 64u; ++i) {
		if (g_hs_rb[i] != g_fw.data[i]) {
			(void)snprintf(g_hs_why, sizeof(g_hs_why), "%s: SOCRAM byte %u reads 0x%02x, image 0x%02x",
				phase, (unsigned)i, (unsigned)g_hs_rb[i], (unsigned)g_fw.data[i]);
			return -1;
		}
	}
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

/* F1 registers of the chip's clock and sleep control (brcmfmac sdio.h). */
#define SBSDIO_FUNC1_CHIPCLKCSR 0x1000Eu
#define SBSDIO_HT_AVAIL_REQ     0x10u
#define SBSDIO_HT_AVAIL         0x80u
#define SBSDIO_FUNC1_SLEEPCSR   0x1001Fu
#define SBSDIO_SLEEPCSR_KSO     0x01u
#define SBSDIO_SLEEPCSR_DEVON   0x02u

static unsigned int g_ctrl_wake_retries = 0u; /* control frames re-sent after a wake */

/* Make sure the chip can take an F2 transfer: awake (KSO) and on its HT
 * backplane clock. brcmfmac does this before every control frame
 * (brcmf_sdio_bus_txctl -> brcmf_sdio_clkctl(CLK_AVAIL)), because the chip may
 * drop HT, or doze, while the host is idle -- and the first CMD53 to a chip in
 * that state fails. Here that surfaced as a join whose first command
 * (event_msgs) failed at the transport while every later command succeeded, so
 * the firmware was never told to report join events (JOIN-RC events=0,
 * em=-1041, 2 of 2 failures; em=0 in both successes).
 *
 * The first write to a dozing chip may itself be NACKed, so KSO is written until
 * it reads back with DEVON set. The HT request stays asserted: the radio is
 * already kept awake (mpc=0), so there is no power saving to lose.
 * Returns the CHIPCLKCSR value found on entry (for the join report), or -1 if
 * it could not be read. *ok is set to 1 when HT is available on return. */
static int diag_chipWake(volatile uint8_t *sdhci, int *ok)
{
	uint32_t r[4] = { 0 };
	int entry, i;

	*ok = 0;
	entry = (diag_sdioCmd52(sdhci, 0, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0u, r) == 0) ? (int)(r[0] & 0xffu) : -1;
	if ((entry >= 0) && ((entry & SBSDIO_HT_AVAIL) != 0)) {
		*ok = 1;
		return entry;
	}

	for (i = 0; i < 64; ++i) {
		(void)diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SLEEPCSR, SBSDIO_SLEEPCSR_KSO, NULL);
		r[0] = 0u;
		if ((diag_sdioCmd52(sdhci, 0, 1, SBSDIO_FUNC1_SLEEPCSR, 0u, r) == 0) &&
			((r[0] & (SBSDIO_SLEEPCSR_KSO | SBSDIO_SLEEPCSR_DEVON)) == (SBSDIO_SLEEPCSR_KSO | SBSDIO_SLEEPCSR_DEVON))) {
			break;
		}
		usleep(300);
	}

	(void)diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_CHIPCLKCSR, SBSDIO_HT_AVAIL_REQ, NULL);
	for (i = 0; i < 50; ++i) {
		r[0] = 0u;
		if ((diag_sdioCmd52(sdhci, 0, 1, SBSDIO_FUNC1_CHIPCLKCSR, 0u, r) == 0) && ((r[0] & SBSDIO_HT_AVAIL) != 0u)) {
			*ok = 1;
			break;
		}
		usleep(1000);
	}
	return entry;
}
/* Max F2 frame we can hold: SDPCM(12) + BDC(4) + full-MTU eth(1514) + slack.
 *
 * Do not raise this without reading the following. Instrumenting the RX errors
 * showed that ALL of them were this cap rejecting frames the firmware had
 * announced (-32), up to 9248 bytes -- 184 of 1016 receives, ~18% of inbound
 * frames dropped. That looks exactly like an undersized buffer, so the obvious
 * fix is a bigger one. Measured on hardware, 16384:
 *
 *   cap  2048: rx_err 184, rx_garbage 876, RX 0.10-0.29 MB/s
 *   cap 16384: rx_err   0, rx_garbage   0, RX 0.03 MB/s in three runs running
 *              -- and max_announced_len 0, i.e. NO oversize frame ever arrived
 *
 * The 9248-byte headers simply stopped occurring once the cap changed, so they
 * were never real aggregation: they are a symptom of the receive stream losing
 * alignment, and the -32 path's controller reset was what recovered it. A
 * bigger buffer removes the symptom, the recovery with it, and RX gets an order
 * of magnitude worse.
 *
 * So this stays at 2048 (the better-measured behaviour) and the real fix is the
 * stream itself: SDPCM glom handling or a proper resync strategy, not a wider
 * buffer. The rxxfer/rxbig counters exist to re-run this comparison. */
#define F2_FRAME_MAX 2048u
#define F2_HDR_LEN   12u     /* SDPCM HW(4) + SW(8): enough to learn the length */
/* SDIO function-2 block size, programmed into FBR 0x210 during bring-up.
 * Block mode is the ONLY way to move more than 512 bytes (see the guards in
 * the byte-mode helpers), so every MTU-sized data frame goes through it.
 *
 * 64 is what this driver has always used. brcmfmac uses 512 for this chip
 * (SDIO_FUNC2_BLOCKSIZE; only the 4329, 4354/4356/4359 and 4373 get less, in
 * brcmf_sdiod_probe), and the block size costs time: every block carries its
 * own CRC, CRC-status token and busy phase. The firmware download measures that
 * at ~1.6 us per 64-byte block at 25 MHz (a 4 KB CMD53 takes 429 us against 328
 * us of data), so a 1536-byte TX frame spends ~38 us of its ~160 us on 24 block
 * trailers; at 512 it has 3.
 *
 * Only TX is affected on this side: the receive path reads F2 in byte mode (see
 * F2_RX_ONE_CMD). What the firmware does with the value is not known here --
 * it may align what it sends us to it -- so 512 is opt-in until measured:
 * `f2blk=512` sets it at bring-up (Linux's order), `wifi f2blk <n>` at run time.
 * The glom/garbage/resync counters in `stats` are the ones to watch. */
#define F2_BLKSZ_DEFAULT 64u
static uint32_t g_f2_blksz = F2_BLKSZ_DEFAULT;      /* what FBR 0x210 holds */
static uint32_t g_f2_blksz_want = F2_BLKSZ_DEFAULT; /* `f2blk=<n>` */
static int g_evt_seen = 0;             /* chan-1 (event) frames demuxed past */
static int g_ctrl_seen = 0;            /* chan-0 (control) frames read */
static uint16_t g_last_evt_len = 0u;
static uint8_t g_last_evt[32];         /* head of the last event frame seen */

static uint32_t diag_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
		((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Point the backplane window at 0x18000000 (the F2 FIFO and the SDIO core),
 * writing only the bytes that differ from what the chip already holds. */
static void diag_setWindow18(volatile uint8_t *sdhci)
{
	static const uint8_t want[3] = { 0x00u, 0x00u, 0x18u };
	uint32_t k;

	for (k = 0; k < 3u; ++k) {
		if ((g_sbwin_valid[k] != 0u) && (g_sbwin[k] == want[k])) {
			g_sbwin_skips++;
			continue;
		}
		(void)diag_sdioCmd52(sdhci, 1, 1, SBSDIO_FUNC1_SBADDRLOW + k, want[k], NULL);
		g_sbwin_writes++;
	}
}

/* SDPCM flow control. The firmware advertises, in EVERY frame it sends us, how
 * far our transmit sequence may advance -- a credit window. It lives in the
 * SECOND software-header dword: byte 0 (buf[8]) is the flow-control mask and
 * byte 1 (buf[9]) is the maximum transmit sequence (brcmfmac: SDPCM_FCMASK_MASK
 * and SDPCM_WINDOW_MASK >> SDPCM_WINDOW_SHIFT). We used to write those bytes as
 * zero and ignore them on receive, i.e. transmit with no regard for credits at
 * all. The firmware then silently drops what it cannot take, TCP retransmits,
 * and throughput decays the longer a session runs -- measured as TX falling
 * 1.02 -> 0.45 -> 0.26 MB/s across three back-to-back transfers in one boot. */
static uint8_t g_tx_max = 0;      /* highest sequence the fw will accept */
static uint8_t g_fc_mask = 0;     /* fw flow-control mask (non-zero = throttled) */
static uint32_t g_tx_blocked = 0; /* transmits refused because the window was shut */
static uint32_t g_rx_badhdr_run = 0; /* consecutive malformed headers */
/* rx_err lumped six different failures together, which is useless for deciding
 * what to fix: -30/-32/-33 are transport (the transfer itself failed), while
 * -1061/-1062/-1063 mean the frame arrived but its SDPCM/BDC offsets did not
 * make sense. Count them apart. */
static uint32_t g_rxe_xfer = 0;   /* any of the three below */
static uint32_t g_rxe_hdr = 0;    /* -30 phase-1 header read failed */
static uint32_t g_rxe_big = 0;    /* -32 fw announced a frame larger than F2_FRAME_MAX */
static int g_join_rxglom_rc = -100; /* result of asking the fw to stop glomming */
static uint32_t g_rxe_body = 0;   /* -33 phase-2 body read failed */
static uint16_t g_rxe_big_len = 0;/* the largest such announced length, to size the buffer */
static uint8_t g_rxe_big_chan = 0xff; /* and which SDPCM channel it came in on */
static uint32_t g_rx_big_ok = 0;      /* frames > 2048 that were read successfully */
static uint16_t g_rx_big_ok_len = 0;
static uint32_t g_rxe_sdoff = 0;  /* -1061 implausible SDPCM data_offset */
static uint32_t g_rxe_ethoff = 0; /* -1062 ethernet offset past the frame end */
static uint32_t g_rxe_toobig = 0; /* -1063 frame larger than the caller's buffer */
#define WIFI_RX_RESYNC_AFTER 64u
static uint32_t g_fc_updates = 0;

static uint8_t g_data_seq = 0;


/* Resynchronise the F2 read path after the stream has gone out of step,
 * mirroring brcmf_sdio_rxfail: abort function 2 through CCCR, terminate the
 * read frame via FRAMECTRL, then wait for the device's pending read byte count
 * to drain. Without any recovery we keep reading from wherever the stream
 * desynchronised, so every following header is garbage too.
 *
 * Only call this after SEVERAL consecutive bad headers. A single one is also
 * what an empty FIFO looks like (it returns bytes that fail the header check),
 * and aborting F2 on every idle poll would be far worse than the problem. */
static uint32_t g_rx_resyncs = 0;

static void diag_f2RxFail(volatile uint8_t *sdhci)
{
	uint32_t r[4], last = 0xffffu;
	int tries;

	(void)diag_sdioCmd52(sdhci, 1, 0, 0x06u, 0x02u, NULL);    /* CCCR abort, function 2 */
	(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Du, 0x01u, NULL); /* FRAMECTRL: SFC_RF_TERM */

	/* Wait for the read frame byte count to drain or stop moving. */
	for (tries = 0; tries < 50; ++tries) {
		uint32_t hi, lo, bc;

		if (diag_sdioCmd52(sdhci, 0, 1, 0x1001Cu, 0u, r) != 0) {
			break;
		}
		hi = r[0] & 0xffu;
		if (diag_sdioCmd52(sdhci, 0, 1, 0x1001Bu, 0u, r) != 0) {
			break;
		}
		lo = r[0] & 0xffu;
		bc = (hi << 8) | lo;
		if ((bc == 0u) || (bc == last)) {
			break;
		}
		last = bc;
	}
	diag_sdhciResetCmdDat(sdhci);
	g_rx_resyncs++;
}


/* ---- F2 data transfers of ANY length -------------------------------------
 *
 * The byte-mode helpers cap at 512 bytes, which is fine for control frames and
 * for the DHCP exchange but cannot carry a 1514-byte ethernet frame -- the
 * prerequisite for an lwip netif. These wrappers pick the addressing mode by
 * length: byte mode below the cap (the HW-proven path, untouched), block mode
 * above it. Block mode moves whole blocks, so the length is rounded up to
 * the F2 block size; the SDPCM header still tells both sides where the real frame ends,
 * and the TX tail padding is zeroed so nothing stale goes on the air.
 */
/* Program the function-2 block size (FBR2 0x210/0x211) and read it back. 0 when
 * the card holds `sz`; g_f2_blksz follows what was written either way, so a TX
 * never uses a block size the card was not given. */
static int diag_f2SetBlockSize(volatile uint8_t *sdhci, uint32_t sz)
{
	uint32_t lo[4] = { 0 }, hi[4] = { 0 };
	int rc;

	rc = diag_sdioCmd52(sdhci, 1, 0, 0x210u, (uint8_t)(sz & 0xffu), NULL);
	rc |= diag_sdioCmd52(sdhci, 1, 0, 0x211u, (uint8_t)((sz >> 8) & 0xffu), NULL);
	g_f2_blksz = sz;
	rc |= diag_sdioCmd52(sdhci, 0, 0, 0x210u, 0u, lo);
	rc |= diag_sdioCmd52(sdhci, 0, 0, 0x211u, 0u, hi);
	if (rc != 0) {
		return -1;
	}
	return ((((hi[0] & 0xffu) << 8) | (lo[0] & 0xffu)) == sz) ? 0 : -2;
}

static int diag_f2Write(volatile uint8_t *sdhci, const uint8_t *buf, uint32_t len)
{
	uint32_t wlen = (len + 3u) & ~3u; /* pad to 4 for the PIO word loop */

	if (wlen <= 512u) {
		return diag_sdioCmd53WriteByteMode(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR, wlen, buf);
	}
	return diag_sdioCmd53Write(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR,
		(len + g_f2_blksz - 1u) / g_f2_blksz, g_f2_blksz, buf);
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
		(len + g_f2_blksz - 1u) / g_f2_blksz, g_f2_blksz, buf);
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
static int diag_f2RecvFrame(volatile uint8_t *sdhci, uint8_t *buf, uint32_t cap,
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
	if ((uint32_t)len > cap) {
		/* Remember how big the firmware actually wanted to hand us: if this is
		 * the dominant RX failure then the buffer, not the bus, is the problem. */
		if (len > g_rxe_big_len) {
			g_rxe_big_len = len;
		}
		g_rxe_big_chan = (uint8_t)(buf[5] & 0x0fu);
		diag_sdhciResetCmdDat(sdhci);
		return -32;
	}
	/* phase 2: the rest of this frame.
	 *
	 * Normal frames are word-aligned, but a GLOM frame (channel 3) is padded up
	 * to the SDIO block size -- brcmf_sdio_hdparse treats
	 * roundup(len, blocksize) != read_len as an error. Reading only to a word
	 * boundary leaves the pad in the FIFO, and every following header read then
	 * starts mid-pad: the stream desynchronises. */
	if (len > (uint16_t)F2_HDR_LEN) {
		/* Word alignment for every channel, including glom.
		 *
		 * brcmf_sdio_hdparse expects a superframe transfer of
		 * roundup(len, blocksize), so padding to the F2 block size looks right --
		 * and measured on hardware it is WRONG here: with it, receive dies within
		 * a handful of frames (rx_ok 5, no glom frame ever seen). That padding
		 * belongs to block-mode transfers; this driver reads the FIFO in byte
		 * mode, which pops exactly what is asked for, so rounding up to 64 reads
		 * past the frame and desynchronises the stream. */
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
	/* Credits ride along on every frame, whatever its channel. */
	{
		uint8_t fc = buf[8];
		uint8_t tx_max = buf[9];

		/* brcmf_sdio_hdparse's sanity clamp: a window more than 0x40 ahead of
		 * our sequence is a garbled header, not a generous firmware. */
		if ((uint8_t)(tx_max - g_data_seq) > 0x40u) {
			tx_max = (uint8_t)(g_data_seq + 2u);
		}
		g_tx_max = tx_max;
		g_fc_mask = fc;
		g_fc_updates++;
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
		/* Most likely the chip dozed or dropped HT while idle (see
		 * diag_chipWake): wake it and send the frame once more. */
		int awake;

		diag_sdhciResetDatCmd(sdhci);
		(void)diag_chipWake(sdhci, &awake);
		diag_setWindow18(sdhci);
		rc = diag_sdioCmd53WriteByteMode(sdhci, 2, /*incr=*/1, IOCTL_F2_ADDR, wlen, g_txf);
		g_ctrl_wake_retries++;
	}
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
		fr = diag_f2RecvFrame(sdhci, g_rxf, F2_FRAME_MAX, &len, &chan);
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
#define WLC_DISASSOC_CMD 52u       /* BRCMF_C_DISASSOC: no payload, as brcmf_link_down() */
#define WLC_SET_WSEC_PMK_CMD 268u  /* BRCMF_C_SET_WSEC_PMK: brcmf_wsec_pmk_le (passphrase) */
#define BRCMF_C_GET_PKTCNTS 137u   /* brcmf_pktcnt_le { rx_good, rx_bad, tx_good, tx_bad, rx_ocast } */
#define BRCMF_C_GET_RATE 12u       /* le32: current TX rate, in 500 kbit/s units (fwil.h) */
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
static uint8_t g_iov[1536]; /* `dump` text replies need more than the 512 the setup iovars do */
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
	while (off < (uint32_t)g_clm.len) {
		uint16_t flag = 0x1000u; /* DLOAD_HANDLER_VER<<12 */
		chunk = (uint32_t)g_clm.len - off;
		if (chunk > CLM_CHUNK) {
			chunk = CLM_CHUNK;
		}
		if (off == 0u) {
			flag |= 0x0002u; /* DL_BEGIN */
		}
		if (off + chunk >= (uint32_t)g_clm.len) {
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
			clmbuf[12 + i] = g_clm.data[off + i];
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

		fr = diag_f2RecvFrame(sdhci, g_rxf, F2_FRAME_MAX, &len, &chan);
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
static int g_join_clk_entry = -1, g_join_clk_ok = 0; /* diag_chipWake() at join start */
static unsigned int g_join_wake_retries0 = 0u;
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
		int rc = diag_f2RecvFrame(sdhci, g_rxf, F2_FRAME_MAX, &flen, &chan);
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
static uint32_t g_frame_tx_ok = 0, g_frame_tx_err = 0;
static uint32_t g_frame_rx_ok = 0, g_frame_rx_err = 0, g_frame_rx_garbage = 0;


/* Transmit one 802.3 frame. Returns 0 on success, <0 on a transport error. */
static int diag_wifiFrameTx(volatile uint8_t *sdhci, const uint8_t *eth, uint32_t elen)
{
	uint32_t total, padded, i;
	uint64_t t0;
	int rc;

	if ((sdhci == NULL) || (eth == NULL) || (elen < 14u) || ((elen + 16u) > F2_FRAME_MAX)) {
		return -1060;
	}
	/* Respect the credit window: brcmfmac's test is that (tx_max - tx_seq) is
	 * non-zero and has not wrapped past 0x80. Refusing here is much better than
	 * handing the firmware a frame it will drop -- the caller (TCP) retransmits,
	 * and the window reopens as soon as the fw sends us anything. g_fc_updates
	 * stays 0 until the first frame arrives, so do not gate before that or a
	 * fresh link could never send its first packet. */
	if (g_fc_updates != 0u) {
		uint8_t avail = (uint8_t)(g_tx_max - g_data_seq);

		if ((avail == 0u) || ((avail & 0x80u) != 0u)) {
			g_tx_blocked++;
			return -1070;
		}
	}

	total = 16u + elen;
	padded = ((total + g_f2_blksz - 1u) / g_f2_blksz) * g_f2_blksz;
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
	t0 = diag_ticks();
	rc = diag_f2Write(sdhci, g_txf, total);
	if (rc != 0) {
		diag_sdhciResetDatCmd(sdhci);
		g_frame_tx_err++;
		return rc;
	}
	if (total > 1024u) {
		g_bt_tx_ticks += diag_ticks() - t0;
		g_bt_tx_n++;
	}
	g_frame_tx_ok++;
	return 0;
}


/* ---- SDPCM glom (aggregated receive) ------------------------------------
 *
 * The firmware bundles several received frames into ONE superframe on channel 3
 * -- measured here at up to 23072 bytes. This driver used to reject those as
 * oversize and drop them, losing ~18% of all inbound frames; asking the firmware
 * to stop (bus:rxglom = 0, accepted with rc 0) does not stop it. So de-aggregate.
 *
 * A superframe is its own SDPCM header followed, from its data_offset, by
 * back-to-back subframes that each carry a full SDPCM header of their own. We
 * read the superframe once and then hand out its channel-2 subframes one read()
 * at a time, which also amortises the per-frame cost that limits this driver:
 * one bus transfer now feeds several frames instead of one. */
#define F2_GLOM_MAX 32768u

static uint8_t g_glomf[F2_GLOM_MAX];
static uint32_t g_glom_off = 0, g_glom_end = 0;
static uint32_t g_glom_supers = 0, g_glom_subs = 0, g_glom_bad = 0;
static uint32_t g_glom_descs = 0;
/* Subframe strides from the most recent glom descriptor. */
#define WIFI_GLOM_MAX_SUBS 64u
static uint16_t g_glom_lens[WIFI_GLOM_MAX_SUBS];
static uint32_t g_glom_nlens = 0, g_glom_idx = 0, g_glom_slotbase = 0;


/* Association state after the join. The join loop reads its own events; after
 * it returns, event frames (SDPCM channel 1) reach only the data path below, which
 * used to drop them -- so a deauthentication, or the AP simply going away, left
 * `status` reporting a join that no longer existed. These record the loss. */
static int wifi_isJoined(void);
static uint32_t g_link_losses = 0u;
static uint32_t g_link_loss_event = 0u; /* WLC_E_* type of the most recent loss */


/* Look at one channel-1 frame (SDPCM header at f[0]) from the data path and note a
 * lost association: WLC_E_LINK with the link bit clear, or a deauth/disassoc in
 * either direction. Same event_msg offsets as the join loop. */
static void wifi_noteEvent(const uint8_t *f, uint32_t len)
{
	uint32_t sdoff, ehdr, etype;
	uint16_t flags;

	sdoff = f[7];
	if ((sdoff + 4u) > len) {
		return;
	}
	ehdr = sdoff + 4u + 4u * (uint32_t)f[sdoff + 3u];
	if (((ehdr + 48u) > len) || (diag_be16(f + ehdr + 12u) != 0x886Cu)) {
		return; /* not an ETH_P_LINK_CTL event */
	}
	flags = diag_be16(f + ehdr + 26u);
	etype = diag_be32(f + ehdr + 28u);

	/* 5/6 = WLC_E_DEAUTH(_IND), 11/12 = WLC_E_DISASSOC(_IND), 16 = WLC_E_LINK */
	if ((etype == 5u) || (etype == 6u) || (etype == 11u) || (etype == 12u) ||
		((etype == 16u) && ((flags & 0x01u) == 0u))) {
		if (wifi_isJoined()) {
			g_link_losses++;
			g_link_loss_event = etype;
			printf("wifi: association LOST (event %u)\n", (unsigned)etype);
		}
		g_join_link_up = 0;
		g_join_psksup_status = -101; /* no longer keyed: wifi_isJoined() reads 0 */
	}
}


/* Next channel-2 subframe of the superframe in hand. 0 with *elen set, or 1 when
 * it is exhausted. Consumes every subframe it walks past, data or not. */
static int diag_glomNext(uint8_t *eth, uint32_t cap, uint32_t *elen)
{
	while (g_glom_idx < g_glom_nlens) {
		uint32_t off = g_glom_off;
		uint8_t *q = g_glomf + off;
		uint16_t slen, schk;
		uint32_t sdoff, ethoff, n;
		uint8_t schan;

		/* Step to the next SLOT first, so every exit path below leaves the walk
		 * correctly positioned. */
		g_glom_slotbase += (uint32_t)g_glom_lens[g_glom_idx];
		g_glom_idx++;
		g_glom_off = g_glom_slotbase;

		if ((off + 12u) > g_glom_end) {
			break;
		}
		slen = (uint16_t)(q[0] | (q[1] << 8));
		schk = (uint16_t)(q[2] | (q[3] << 8));
		if (((uint16_t)(~(slen ^ schk)) != 0u) || (slen < 12u) ||
				((off + (uint32_t)slen) > g_glom_end)) {
			g_glom_bad++;
			continue;
		}
		schan = (uint8_t)(q[5] & 0x0fu);
		sdoff = q[7];
		if (schan != 2u) {
			if (schan == 1u) {
				wifi_noteEvent(q, (uint32_t)slen);
			}
			continue; /* control/event subframe */
		}
		if ((sdoff < 12u) || ((sdoff + 4u) > (uint32_t)slen)) {
			g_glom_bad++;
			continue;
		}
		ethoff = sdoff + 4u + ((uint32_t)q[sdoff + 3u] << 2);
		if ((ethoff + 14u) > (uint32_t)slen) {
			g_glom_bad++;
			continue;
		}
		n = (uint32_t)slen - ethoff;
		if (n > cap) {
			g_glom_bad++;
			continue;
		}
		memcpy(eth, q + ethoff, n);
		*elen = n;
		g_glom_subs++;
		return 0;
	}
	return 1;
}


/* What diag_wifiFrameRx returns when it has no 802.3 frame to hand out. The
 * difference matters to a reader that sleeps until the next card interrupt:
 * after WIFI_RX_NODATA more may be queued behind the frame it consumed. */
#define WIFI_RX_NONE   1 /* nothing queued */
#define WIFI_RX_NODATA 2 /* a frame was consumed, but it carried no 802.3 data */

/* Receive one 802.3 frame. Returns 0 with *elen set, WIFI_RX_NONE if nothing is
 * ready, WIFI_RX_NODATA if the frame read was not DATA, <0 on error.
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
	uint64_t t0, dt;
	int rc;

	/* Hand out what is already in the superframe before touching the bus. */
	if (g_glom_off < g_glom_end) {
		if (diag_glomNext(eth, cap, elen) == 0) {
			g_frame_rx_ok++;
			return 0;
		}
	}

	t0 = diag_ticks();
	rc = diag_f2RecvFrame(sdhci, g_glomf, F2_GLOM_MAX, &flen, &chan);
	dt = diag_ticks() - t0;
	if (rc == 1) {
		g_bt_empty_ticks += dt;
		g_bt_empty_n++;
	}
	else if ((rc == 0) && (chan == 2u) && (flen > 1024u)) {
		g_bt_rx_ticks += dt;
		g_bt_rx_n++;
	}
	else if ((rc == 0) && (chan == 3u) && ((g_glomf[5] & 0x80u) == 0u)) {
		g_bt_glom_ticks += dt;
		g_bt_glom_bytes += flen;
		g_bt_glom_n++;
	}
	if (rc != 0) {
		/* -31 is an inconsistent SDPCM header: either an empty FIFO (the common,
		 * harmless case) or a stream that has lost frame alignment. They look
		 * identical one at a time, so distinguish by persistence -- a long run of
		 * them is desynchronisation, and only then is a resync worth its cost. */
		if (rc == -31) {
			g_frame_rx_garbage++;
			if (++g_rx_badhdr_run >= WIFI_RX_RESYNC_AFTER) {
				g_rx_badhdr_run = 0;
				diag_f2RxFail(sdhci);
			}
			return WIFI_RX_NONE;
		}
		if (rc < 0) {
			g_frame_rx_err++;
			g_rxe_xfer++;
			if (rc == -30) {
				g_rxe_hdr++;
			}
			else if (rc == -32) {
				g_rxe_big++;
			}
			else if (rc == -33) {
				g_rxe_body++;
			}
		}
		return rc;
	}
	if ((chan == 3u) && ((g_glomf[5] & 0x80u) != 0u)) {
		/* Glom DESCRIPTOR (brcmfmac SDPCM_GLOMDESC): its payload is a list of
		 * u16 lengths for the subframes of the superframe that follows, not the
		 * data itself. Walking it as data was the bug -- a 22-byte descriptor
		 * listing 5 x 96 bytes was being parsed as if it held frames. Nothing to
		 * deliver here; the superframe arrives as the next channel-3 frame. */
		uint32_t d = g_glomf[7];

		g_glom_nlens = 0u;
		while (((d + 1u) < (uint32_t)flen) && (g_glom_nlens < WIFI_GLOM_MAX_SUBS)) {
			uint16_t sl = (uint16_t)(g_glomf[d] | (g_glomf[d + 1u] << 8));

			if (sl == 0u) {
				break;
			}
			g_glom_lens[g_glom_nlens++] = sl;
			d += 2u;
		}
		g_glom_descs++;
		return WIFI_RX_NODATA;
	}
	if (chan == 3u) {
		/* Glom superframe: start walking it, and return its first data subframe. */
		uint32_t soff = g_glomf[7];

		if ((soff < 12u) || (soff > (uint32_t)flen)) {
			g_glom_bad++;
			return WIFI_RX_NODATA;
		}
		/* Subframes live in the descriptor's SLOTS, measured from the START of
		 * the superframe: slot k spans [sum(lens[0..k-1]), +lens[k]). Slot 0 also
		 * carries the superframe's own 12-byte header, so its subframe begins at
		 * data_offset while every later slot begins with its subframe.
		 *
		 * Decoded from a hardware dump rather than assumed: a 384-byte
		 * superframe had valid subframe headers at 12, 96, 192 and 288 -- gaps
		 * of 84, 96, 96 -- and its descriptor listed 96-byte slots summing to
		 * exactly 384. Walking by each subframe's own length (72) instead landed
		 * mid-header after the first one, every time. */
		g_glom_off = soff;
		g_glom_end = (uint32_t)flen;
		g_glom_slotbase = 0u;
		g_glom_idx = 0u;
		g_glom_supers++;
		if (diag_glomNext(eth, cap, elen) == 0) {
			g_frame_rx_ok++;
			return 0;
		}
		return WIFI_RX_NODATA;
	}
	if (chan != 2u) {
		if (chan == 1u) {
			wifi_noteEvent(g_glomf, (uint32_t)flen);
		}
		return WIFI_RX_NODATA;
	}
	/* eth offset = SDPCM data_offset + BDC(4) + BDC.doff words (<<2), exactly
	 * as brcmf_proto_bcdc_hdrpull computes it. Never assume a fixed 16. */
	sdoff = g_glomf[7];
	if ((sdoff < 12u) || ((sdoff + 4u) > (uint32_t)flen)) {
		g_frame_rx_err++;
		g_rxe_sdoff++;
		return -1061;
	}
	ethoff = sdoff + 4u + ((uint32_t)g_glomf[sdoff + 3u] << 2);
	if ((ethoff + 14u) > (uint32_t)flen) {
		g_frame_rx_err++;
		g_rxe_ethoff++;
		return -1062;
	}
	g_rx_badhdr_run = 0; /* a good frame means the stream is in step again */
	if (flen > 2048u) {
		g_rx_big_ok++;
		if (flen > g_rx_big_ok_len) {
			g_rx_big_ok_len = flen;
		}
	}
	n = (uint32_t)flen - ethoff;
	if (n > cap) {
		g_frame_rx_err++;
		g_rxe_toobig++;
		return -1063;
	}
	if (eth != NULL) {
		for (i = 0; i < n; ++i) {
			eth[i] = g_glomf[ethoff + i];
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


/* ---- Radio settings applied at every join ---------------------------------
 *
 * Throughput on this link is bounded by the air as much as by the bus: a 1x1
 * HT20 association runs at 72 Mbit/s at best, and the chip can do 150 (HT40) or
 * 433 (5 GHz, VHT80). Some of what decides that can only be set while the radio
 * is DOWN -- brcmfmac sets `bw_cap` that way, before its first WLC_UP -- so these
 * are queued and applied by the join, between the CLM download and WLC_UP:
 *
 *   country   ISO 3166 code, revision 0 (what brcmfmac sends for this chip,
 *             brmcf_use_iso3166_ccode_fallback). Without one the firmware runs
 *             its built-in default locale, which may leave 5 GHz out or passive.
 *   iovars    any integer iovar, as little-endian words: `bw_cap` {band, cap},
 *             `ampdu_ba_wsize`, `ampdu_mpdu`, ...
 *
 * With nothing queued the join sends exactly what it always did (no WLC_DOWN).
 * The join runs on every `joinwpa`, so a queued change takes effect at the next
 * association: `leave` makes the netif rejoin within one status poll. */
#define WIFI_ATJOIN_MAX   8u
#define WIFI_ATJOIN_WORDS 4u
#define WIFI_IOVAR_NAME   32u

/* bw_cap's band numbers (WLC_BAND_*, brcmu_wifi.h) and its bit sets */
#define WLC_BAND_5G       1u
#define WLC_BAND_2G       2u
#define WLC_BW_CAP_20MHZ  0x1u
#define WLC_BW_CAP_40MHZ  0x3u
#define WLC_BW_CAP_80MHZ  0x7u

#define WLC_DOWN_CMD      3u
#define BRCMF_C_GET_PM    85u
#define BRCMF_C_SET_PM    86u

typedef struct {
	char name[WIFI_IOVAR_NAME];
	uint32_t nwords;
	uint32_t w[WIFI_ATJOIN_WORDS];
	int rc; /* result at the last join */
} wifi_atjoin_t;

static wifi_atjoin_t g_atjoin[WIFI_ATJOIN_MAX];
static uint32_t g_atjoin_n = 0u;
static char g_country[3] = { 0 };   /* "" = the firmware's default locale */
static int g_country_rc = -100;     /* at the last join, or the last `country` */
static int g_join_down_rc = -100;   /* WLC_DOWN at the last join; -100 = not sent */


static void wifi_putLe32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xffu);
	p[1] = (uint8_t)((v >> 8) & 0xffu);
	p[2] = (uint8_t)((v >> 16) & 0xffu);
	p[3] = (uint8_t)((v >> 24) & 0xffu);
}


/* brcmf_fil_country_le: country_abbrev[4], le32 rev, ccode[4]. */
static void wifi_countryPayload(uint8_t p[12], const char *cc)
{
	memset(p, 0, 12u);
	p[0] = (uint8_t)cc[0];
	p[1] = (uint8_t)cc[1];
	p[8] = (uint8_t)cc[0];
	p[9] = (uint8_t)cc[1];
}


/* Queue (or replace) an iovar for every following join. An entry is replaced by
 * name, except `bw_cap`, which is per band: there the band (first word) is part
 * of the key, so 2g and 5g settings can both be queued. 0, or -1 when the list
 * is full or the name does not fit. */
static int wifi_atjoinAdd(const char *name, const uint32_t *w, uint32_t n)
{
	uint32_t i, k;
	int perband = (strcmp(name, "bw_cap") == 0);

	if ((strlen(name) >= WIFI_IOVAR_NAME) || (n == 0u) || (n > WIFI_ATJOIN_WORDS)) {
		return -1;
	}
	for (i = 0u; i < g_atjoin_n; ++i) {
		if ((strcmp(g_atjoin[i].name, name) == 0) && (!perband || (g_atjoin[i].w[0] == w[0]))) {
			break;
		}
	}
	if (i == g_atjoin_n) {
		if (g_atjoin_n >= WIFI_ATJOIN_MAX) {
			return -1;
		}
		g_atjoin_n++;
	}
	strcpy(g_atjoin[i].name, name);
	g_atjoin[i].nwords = n;
	for (k = 0u; k < n; ++k) {
		g_atjoin[i].w[k] = w[k];
	}
	g_atjoin[i].rc = -100;
	return 0;
}


/* `bw_cap` for one band: band is WLC_BAND_2G/5G, mhz 20, 40 or 80. */
static int wifi_atjoinBw(uint32_t band, int mhz)
{
	uint32_t w[2];

	w[0] = band;
	if (mhz == 20) {
		w[1] = WLC_BW_CAP_20MHZ;
	}
	else if (mhz == 40) {
		w[1] = WLC_BW_CAP_40MHZ;
	}
	else if ((mhz == 80) && (band == WLC_BAND_5G)) {
		w[1] = WLC_BW_CAP_80MHZ;
	}
	else {
		return -1;
	}
	return wifi_atjoinAdd("bw_cap", w, 2u);
}


/* The join's hook: radio down, country, queued iovars. Nothing at all when
 * nothing is queued. */
static void diag_joinSettings(volatile uint8_t *sdhci, uint32_t sdio_core, uint32_t *reqid, uint8_t *seq)
{
	uint8_t buf[4u * WIFI_ATJOIN_WORDS];
	uint8_t one[4] = { 1u, 0u, 0u, 0u };
	uint32_t i, k;

	if ((g_atjoin_n == 0u) && (g_country[0] == '\0')) {
		g_join_down_rc = -100;
		return;
	}
	g_join_down_rc = diag_bcdcCmd(sdhci, sdio_core, 1, WLC_DOWN_CMD, one, 4u, NULL, 0u, NULL, (*reqid)++, (*seq)++);
	if (g_country[0] != '\0') {
		uint8_t cc[12];

		wifi_countryPayload(cc, g_country);
		g_country_rc = diag_iovar(sdhci, sdio_core, 1, "country", cc, 12u, NULL, 0u, NULL, (*reqid)++, (*seq)++);
	}
	for (i = 0u; i < g_atjoin_n; ++i) {
		for (k = 0u; k < g_atjoin[i].nwords; ++k) {
			wifi_putLe32(buf + 4u * k, g_atjoin[i].w[k]);
		}
		g_atjoin[i].rc = diag_iovar(sdhci, sdio_core, 1, g_atjoin[i].name, buf, 4u * g_atjoin[i].nwords,
			NULL, 0u, NULL, (*reqid)++, (*seq)++);
	}

	printf("wifi: JOIN-SETTINGS down=%d country=%s rc=%d", g_join_down_rc,
		(g_country[0] != '\0') ? g_country : "-", (g_country[0] != '\0') ? g_country_rc : 0);
	for (i = 0u; i < g_atjoin_n; ++i) {
		printf(" %s=%d", g_atjoin[i].name, g_atjoin[i].rc);
	}
	printf("\n");
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
	g_join_clk_entry = diag_chipWake(sdhci, &g_join_clk_ok);
	g_join_wake_retries0 = g_ctrl_wake_retries;

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

	/* Turn RX glomming OFF before anything can arrive.
	 *
	 * The firmware aggregates several received frames into one superframe on
	 * SDPCM channel 3 -- measured here at up to 9248 bytes -- and this driver
	 * has no de-aggregation: it either rejected them as oversize or, with a
	 * bigger buffer, read them and dropped them for not being channel 2. Either
	 * way ~18% of all inbound frames were being thrown away and left for TCP to
	 * retransmit. brcmfmac sets bus:rxglom to 1 to ENABLE this and only when it
	 * has scatter-gather; we never set it at all, so the chip's default was in
	 * charge. Ask for 0 and every frame arrives on channel 2, which the RX path
	 * already handles correctly.
	 *
	 * Glom is worth having eventually -- it amortises exactly the per-frame
	 * overhead that limits this driver -- but only once the superframe is
	 * actually split. The iovar is allowed to fail (brcmfmac treats it the same
	 * way); rx of channel-3 frames is still counted so a regression is visible. */
	{
		uint8_t glom[4] = { 0u, 0u, 0u, 0u };

		g_join_rxglom_rc = diag_iovar(sdhci, sdio_core, 1, "bus:rxglom", glom, 4u,
			NULL, 0u, NULL, reqid++, seq++);
		/* A transport error (<= -1000) is the bus, not the firmware refusing;
		 * the first attempt failed that way when this was the very first command
		 * of the join, so retry once before believing it. */
		if (g_join_rxglom_rc <= -1000) {
			usleep(20000);
			g_join_rxglom_rc = diag_iovar(sdhci, sdio_core, 1, "bus:rxglom", glom, 4u,
				NULL, 0u, NULL, reqid++, seq++);
		}
	}


	/* CLM (regulatory) before UP so the radio has channels (same as scan). */
	(void)diag_clmLoad(sdhci, sdio_core, &reqid, &seq);

	/* Queued radio settings (country, bw_cap, ...): these need the radio down. */
	diag_joinSettings(sdhci, sdio_core, &reqid, &seq);

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

		fr = diag_f2RecvFrame(sdhci, g_rxf, F2_FRAME_MAX, &len, &chan);
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
	/* The setup commands' own return codes and the event count: a join that fails
	 * with no SET_SSID event at all (setssid=-100, ~2 runs in 5) is otherwise
	 * undiagnosable -- this says whether the firmware refused a command or simply
	 * never answered. */
	printf("wifi: JOIN-RC events=%d em=%d glom=%d infra=%d up=%d wsec=%d wpa=%d sup=%d pmk=%d ssid=%d\n",
		g_join_evt_total, g_join_em_rc, g_join_rxglom_rc, g_join_infra_rc, g_join_up_rc,
		g_join_wsec_rc, g_join_wpaauth_rc, g_join_sup_rc, g_join_pmk_rc, g_join_ssid_rc);
	/* The chip's clock state when the join began (HT_AVAIL = 0x80) and how many
	 * control frames needed a wake-and-resend during it. */
	printf("wifi: JOIN-WAKE clkcsr=0x%02x ht=%d resent=%u\n",
		(unsigned int)(g_join_clk_entry & 0xff), g_join_clk_ok, g_ctrl_wake_retries - g_join_wake_retries0);
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
static int g_fw_alive = 0;                 /* the last bring-up saw the firmware start (HT_AVAIL or CARD_INTR) */
static int g_fw_retry_test = 0;            /* `fwretrytest`: treat the first bring-up as failed, to exercise the retry */
static int g_bringup_quiet = 0;            /* the daemon, `fwloadbench`: print the bring-up report only when it failed */
static uint32_t g_fw_bytes = 0u;           /* firmware bytes the last bring-up wrote */
static int g_fw_rc_w = 0, g_fw_rc_nvram = 0; /* its first firmware / NVRAM write error */
/* How long the firmware download took, and its slowest CMD53 (4 KB) and where.
 * The image is read into RAM before the first bring-up (wifi_fwLoad), so the
 * transfer no longer pages it in from the root file system as it did while it
 * was compiled into the binary; a slow CMD53 at the offset of a failure still
 * says the host stalled. */
static uint32_t g_fw_dl_us = 0u, g_fw_cmd53_max_us = 0u, g_fw_cmd53_max_off = 0u, g_fw_cmd53_ge2ms = 0u;
static uint32_t g_sdio_core = 0x18004000u; /* EROM-derived SDIO-DEV core base (set by bring-up) */

#define WIFI_RESP_CAP (8u * 1024u)
/* Three device ids, on two ports: 0 = /dev/wifi (text commands + text result),
 * 1 = /dev/wifidata (raw 802.3 frames, the lwip netif seam), both served by the
 * one message thread that owns the SDIO bus; 2 = /dev/wifiirq, served by its own
 * thread on its own port, which waits for the chip's RX interrupt and never
 * touches the bus (see "RX interrupt" below).
 *
 * A /dev/wifidata read probes the F2 FIFO once and returns (0 = nothing
 * queued); it never blocks, because a blocked read would stall every other
 * request on the message thread, TX included. The client decides when to read:
 * the lwip netif sleeps in a /dev/wifiirq read until the chip interrupts, and
 * falls back to a paced poll (fast while frames flow, 10 ms on an idle link)
 * when that device is absent or misbehaves (drivers/wifi43455.c). A waiting
 * data read, a spin before sleeping, and a bus mutex with a multi-threaded
 * message loop were all tried here on hardware and none showed a benefit that
 * survives the measurement noise (the SAME code measured 1.73 and 0.66 MB/s TX
 * on two runs), so they were reverted.
 *
 * What IS solid, because it averages thousands of samples inside one run:
 *   transmit  132 us per frame
 *   receive   178 us per frame
 *   empty probe 22 us, and there were 1.1 MILLION of them (24.9 s of bus time)
 * -- those empty probes are what the interrupt path removes.
 *
 * Id 3, /dev/wifibatch, carries the same frames several per message (format in
 * wifibatch.h), on the same message thread. It is always served; whether the
 * netif uses it is the `batch` setting, which `status` reports (see "Frame
 * batches" below). */

#define WIFI_DEV_TEXT_ID  0
#define WIFI_DEV_DATA_ID  1
#define WIFI_DEV_IRQ_ID   2
#define WIFI_DEV_BATCH_ID 3

/* Frame batches. `want` is only advice to the netif: `status` reports it, the
 * netif follows it within its status poll (~3 s), and both devices keep working
 * either way. 0 by default until the batch path is measured on hardware. The
 * counters are the message thread's. */
static struct {
	int want; /* `batch=1` argument, `batch 0|1` command */
	uint32_t tx_msgs, tx_frames, tx_max, tx_partial, tx_bad;
	uint32_t rx_msgs, rx_empty, rx_frames, rx_max, rx_capped, rx_drained, rx_skipped;
	uint64_t tx_ticks, rx_ticks;
} g_bat;

static char g_resp[WIFI_RESP_CAP]; /* most recent scan result text, served over mtRead */
static int g_resp_len = 0;

/* The escan chain (diag_wifiScan -> diag_iovar -> diag_bcdcCmd -> diag_f2Recv
 * -> diag_sdioCmd53*) runs on the message thread, so give it a generous stack
 * (cf. MEMORY #152 pool-thread stack-overflow history). */
static char g_msgStack[16 * 1024] __attribute__((aligned(8)));
/* The /dev/wifiirq waiter only takes a lock and waits on a cond. */
static char g_irqStack[4 * 1024] __attribute__((aligned(8)));

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
	const uint8_t *fw_img = g_fw.data;
	const size_t fw_img_len = g_fw.len;
	uint8_t cnt_pre[4] = { 0 }, cnt_post[4] = { 0 }, cnt_post2[4] = { 0 };
	int rc_cnt_pre = -100, rc_cnt_post = -100, rc_cnt_post2 = -100;
	uint32_t ioctl_w2 = 0u, ioctl_w3 = 0u; /* dual ARM-wrapper CR4-identity cross-check */
	uint32_t cr4_core = 0u, sdio_core = 0x18004000u, ram_size = 0u; /* EROM-derived bases */
	uint64_t dl_t0;

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
			"error: firmware not loaded (wifi_fwLoad)\n.\n");
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

	g_pio_slow_waits = 0u;
	g_pio_slow_max_us = 0u;
	g_pio_timeouts = 0u;
	g_pio_to_pres = 0u;
	g_pio_to_int = 0u;
	g_fw_dl_us = 0u;

	{
		volatile uint8_t *gpio = (volatile uint8_t *)gpio_page;
		volatile uint8_t *sdhci = (volatile uint8_t *)sdhci_page;

		diag_sdhciPollCalibrate(sdhci);
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
		if ((diag_f2SetBlockSize(sdhci, g_f2_blksz_want) != 0) && (g_f2_blksz_want != F2_BLKSZ_DEFAULT)) {
			printf("rpi4-wifi: F2 block size %u did not read back; using %u\n",
				(unsigned)g_f2_blksz_want, (unsigned)F2_BLKSZ_DEFAULT);
			(void)diag_f2SetBlockSize(sdhci, F2_BLKSZ_DEFAULT);
		}

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

		/* The download and everything after it at the high-speed clock. A check
		 * that fails leaves the bus at 25 MHz; rc_hs stays 0 either way. */
		if ((rc_hs == 0) && (g_sdclk_try_khz > SDIO_CLK_DS_KHZ)) {
			(void)diag_sdioClockUp(sdhci, g_sdclk_try_khz);
		}

		g_fw_cmd53_max_us = 0u;
		g_fw_cmd53_max_off = 0u;
		g_fw_cmd53_ge2ms = 0u;
		dl_t0 = diag_monoUs();
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
				uint64_t c0 = diag_monoUs(), cdt;

				rc_w = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1,
					/*reg_addr=*/ci * bytes_per_cmd,
					/*block_count=*/blk_count,
					/*block_size=*/blk_size,
					fw_img + fw_offset + ci * bytes_per_cmd);
				cdt = diag_monoUs() - c0;
				if (cdt > g_fw_cmd53_max_us) {
					g_fw_cmd53_max_us = (uint32_t)cdt;
					g_fw_cmd53_max_off = (uint32_t)(fw_offset + ci * bytes_per_cmd);
				}
				if (cdt >= 2000u) {
					g_fw_cmd53_ge2ms++;
				}
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
		g_fw_dl_us = (uint32_t)(diag_monoUs() - dl_t0);

		/* NVRAM load: chip-ready blob goes at chip-internal
		 * (rambase + ramsize - g_nvram.len) = 0x238000 - len,
		 * inside SBADDR window 19, padded to a 64-byte boundary so it
		 * lands as a single CMD53 multi-block write. Skipped in the
		 * trivial-program test: the counter needs no NVRAM, and skipping
		 * it removes NVRAM as a variable from a dead-counter result. */
		if (!g_trivial_mode) {
			/* Place NVRAM at the TRUE ram-top from CR4 bankinfo, not the old
			 * hardcoded 0x238000. The bootloader reads the length-magic token
			 * at ram_top-4; a wrong ram-top => fw never finds NVRAM. */
			uint32_t nv_ramtop = (ram_size != 0u) ? (0x198000u + ram_size) : 0x238000u;
			uint32_t nv_start = nv_ramtop - (uint32_t)g_nvram.len;
			uint8_t  nv_lo  = (uint8_t)(((nv_start >> 15) & 1u) ? 0x80u : 0x00u);
			uint8_t  nv_mid = (uint8_t)((nv_start >> 16) & 0xffu);
			uint8_t  nv_hi  = (uint8_t)((nv_start >> 24) & 0xffu);
			uint32_t nv_f1_offset = nv_start & 0x7FFFu;
			uint32_t nv_blocks = (uint32_t)(g_nvram.len / 64u);

			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Au, nv_lo,  NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Bu, nv_mid, NULL);
			(void)diag_sdioCmd52(sdhci, 1, 1, 0x1000Cu, nv_hi,  NULL);

			rc_nvram_w = diag_sdioCmd53Write(sdhci, 1, /*incr=*/1,
				/*reg_addr=*/nv_f1_offset,
				/*block_count=*/nv_blocks,
				/*block_size=*/64u, g_nvram.data);
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
						if (scan_buf[k] != fw_img[scan_off[s] + (uint32_t)k]) {
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
		g_nvram.len,
		(unsigned)(((ram_size != 0u) ? (0x198000u + ram_size) : 0x238000u) - (uint32_t)g_nvram.len),
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
		g_nvram.data[g_nvram.len - 4], g_nvram.data[g_nvram.len - 3],
		g_nvram.data[g_nvram.len - 2], g_nvram.data[g_nvram.len - 1]);
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
		g_fw_alive = fw_alive;
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
	if ((g_bringup_quiet == 0) || (g_fw_alive == 0)) {
		printf("%.*s", off, buf);
	}
	g_fw_bytes = bytes_written;
	g_fw_rc_w = worst_rc_w;
	g_fw_rc_nvram = rc_nvram_w;
	printf("rpi4-wifi: SDHCI-PIO mode=%s fw_bytes=%u rc_w=%d rc_nvram=%d slow_waits=%u slow_max_us=%u timeouts=%u "
		"dl_ms=%u cmd53_max_us=%u@0x%x cmd53_ge2ms=%u sd=%u Hz\n",
		(g_pio_legacy != 0) ? "legacy" : "level", (unsigned)bytes_written, worst_rc_w, rc_nvram_w,
		(unsigned)g_pio_slow_waits, (unsigned)g_pio_slow_max_us, (unsigned)g_pio_timeouts,
		(unsigned)(g_fw_dl_us / 1000u), (unsigned)g_fw_cmd53_max_us, (unsigned)g_fw_cmd53_max_off,
		(unsigned)g_fw_cmd53_ge2ms, (unsigned)g_sdhci_sd_hz);
	fflush(stdout);
	return (g_fw_alive != 0) ? 0 : 6;
}


/* `fwloadbench N`: load the firmware N times in one boot (each load starts with
 * the WL_REG_ON power cycle wifi_bringup() always does), alternating the level
 * and the legacy PIO wait, one FWLOAD-BENCH line per load. Comparing the two
 * inside one boot and one binary is the only A/B here that is not confounded by
 * build or boot differences; the natural failure rate (2 of 13 loads at
 * core_freq=500, ~1 in 100 before) is far too low to grade across boots.
 * The natural failures were all first loads (while the image was compiled in,
 * load 1 was also the one that paged it in): `legacypio` before `fwloadbench`
 * makes load 1 legacy, and `legacypio` on the daemon compares first loads
 * across boots. */
static int wifi_fwLoadBench(int n)
{
	/* `legacypio fwloadbench N` starts with the legacy wait, so the first
	 * (cold) load can be given to either mode. */
	int start = g_pio_legacy;
	int i, rc, ok[2] = { 0, 0 }, runs[2] = { 0, 0 };

	g_bringup_quiet = 1;
	for (i = 0; i < n; ++i) {
		if (g_sdhci != NULL) {
			(void)munmap((void *)g_sdhci, _PAGE_SIZE);
			g_sdhci = NULL;
		}
		g_fw_alive = 0;
		g_pio_legacy = (i + start) & 1;
		rc = wifi_bringup();
		runs[g_pio_legacy]++;
		if (rc == 0) {
			ok[g_pio_legacy]++;
		}
		printf("rpi4-wifi: FWLOAD-BENCH load=%d/%d mode=%s rc=%d fw_alive=%d fw_bytes=%u rc_w=%d "
			"rc_nvram=%d slow_waits=%u slow_max_us=%u timeouts=%u to_pres=0x%08x to_int=0x%08x "
			"dl_ms=%u cmd53_max_us=%u\n",
			i + 1, n, (g_pio_legacy != 0) ? "legacy" : "level", rc, g_fw_alive,
			(unsigned)g_fw_bytes, g_fw_rc_w, g_fw_rc_nvram,
			(unsigned)g_pio_slow_waits, (unsigned)g_pio_slow_max_us, (unsigned)g_pio_timeouts,
			(unsigned)g_pio_to_pres, (unsigned)g_pio_to_int,
			(unsigned)(g_fw_dl_us / 1000u), (unsigned)g_fw_cmd53_max_us);
		fflush(stdout);
	}
	g_pio_legacy = start;
	g_bringup_quiet = 0;
	printf("rpi4-wifi: FWLOAD-BENCH-SUMMARY level=%d/%d legacy=%d/%d (loads whose firmware started)\n",
		ok[0], runs[0], ok[1], runs[1]);
	fflush(stdout);
	return 0;
}


/* Bring the chip up, retrying when the firmware did not start. A firmware
 * download can fail part-way at the SDIO transport (CMD53 data-phase error,
 * `worst rc_w=-5`; 2 of ~112 loads in the log archive). The daemon then served
 * a chip with no firmware and every join failed until the next reboot. Each
 * wifi_bringup() begins with a WL_REG_ON power cycle, so a retry starts from a
 * cold chip; the previous attempt's SDHCI mapping is released first. */
static int wifi_bringupRetry(void)
{
	int attempt, rc = 6;

	g_sdclk_try_khz = g_sdclk_want_khz;
	for (attempt = 1; attempt <= 3; ++attempt) {
		if (g_sdhci != NULL) {
			(void)munmap((void *)g_sdhci, _PAGE_SIZE);
			g_sdhci = NULL;
		}
		g_fw_alive = 0;
		rc = wifi_bringup();
		if ((rc == 0) && (attempt == 1) && (g_fw_retry_test != 0)) {
			printf("rpi4-wifi: fwretrytest: treating bring-up 1 as failed\n");
			rc = 6;
		}
		if (rc != 6) {
			break;
		}
		printf("rpi4-wifi: firmware did not start (bring-up %d of 3)%s\n", attempt,
			(attempt < 3) ? "; power-cycling the chip and loading it again" : "");
		/* Whatever the cause, the retries run on the bus every earlier build used. */
		if ((attempt < 3) && (g_sdclk_try_khz > SDIO_CLK_DS_KHZ)) {
			g_sdclk_try_khz = SDIO_CLK_DS_KHZ;
			printf("rpi4-wifi: SDIO-HS off for the retry: it runs at %u kHz\n", (unsigned)SDIO_CLK_DS_KHZ);
		}
		fflush(stdout);
	}
	if (rc == 0) {
		printf("rpi4-wifi: firmware running after bring-up %d\n", attempt);
	}
	return rc;
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

		fr = diag_f2RecvFrame(g_sdhci, g_rxf, F2_FRAME_MAX, &len, &chan);
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
		if ((r == WIFI_RX_NONE) || (r == WIFI_RX_NODATA)) {
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
		(unsigned)g_f2_blksz);
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


/* The last join succeeded, and neither a `leave` nor a lost association (see
 * wifi_noteEvent) has happened since. */
static int wifi_isJoined(void)
{
	return (g_join_setssid_status == 0) && (g_join_psksup_status == 6);
}


/* `leave`: disassociate, the way brcmf_link_down() does, and forget the join so
 * `status` and the next `joinwpa` start clean. An lwip netif sends this before
 * joining a different network -- WLC_SET_SSID on top of a live association is
 * untested territory. */
static int wifi_leave(char *out, int cap)
{
	int rc;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	rc = diag_bcdcCmd(g_sdhci, g_sdio_core, 1, WLC_DISASSOC_CMD, NULL, 0u,
		NULL, 0u, NULL, 211u, g_data_seq++);
	g_join_setssid_status = -100;
	g_join_psksup_status = -100;
	g_join_link_up = 0;
	g_join_ssid[0] = '\0';
	g_join_psk[0] = '\0';

	return snprintf(out, (size_t)cap, "LEAVE %s rc=%d\n", (rc == 0) ? "ok" : "fail", rc);
}


/* `status`: one line a client can parse, plus the MAC line when it is known. */
static int wifi_status(char *out, int cap)
{
	int n;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "STATUS controller=down\n");
	}
	/* batch= is read by the netif (drivers/wifi43455.c in phoenix-rtos-lwip). */
	n = snprintf(out, (size_t)cap, "STATUS joined=%d ssid=%s losses=%u batch=%d\n",
		wifi_isJoined(), wifi_isJoined() ? g_join_ssid : "-", (unsigned)g_link_losses, g_bat.want);
	if ((n > 0) && (n < cap) && g_txmac_valid) {
		int m = snprintf(out + n, (size_t)(cap - n), "MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
			g_txmac[0], g_txmac[1], g_txmac[2], g_txmac[3], g_txmac[4], g_txmac[5]);
		if ((m > 0) && (m < cap - n)) {
			n += m;
		}
	}
	return n;
}


/* ---- RX interrupt: the SDIO card interrupt, served on /dev/wifiirq --------
 *
 * The chip drives the SDIO card interrupt (DAT1, which the controller shows as
 * INT_STATUS bit 8, CARD_INT) while its SDIO core's intstatus & hostintmask is
 * non-zero. hostintmask is set to I_HMB_FRAME_IND alone, so the line means "the
 * firmware queued a frame" and nothing else. The pieces, after Linux sdhci +
 * brcmfmac:
 *
 *   handler  IRQ 158, which this controller SHARES with the SD card's EMMC2
 *            (bcm2711.dtsi: &sdhci and emmc2 are both GIC SPI 126, level).
 *            Returns -1 unless INT_STATUS & SIGNAL_EN has CARD_INT, else masks
 *            SIGNAL_EN and wakes the waiter. It cannot ack the source: the card
 *            holds the level until intstatus is cleared, which takes an SDIO
 *            command, i.e. the bus.
 *   waiter   a /dev/wifiirq read. Restarts card-interrupt detection, arms
 *            SIGNAL_EN, sleeps until the handler fires or WIFI_IRQ_WAIT_US
 *            passes, disarms. Returns 1 byte for an interrupt, 0 for a timeout.
 *            Runs on its own thread and port and never issues an SDIO command.
 *   ack      the next /dev/wifidata read (the bus owner) write-1-clears
 *            intstatus BEFORE it drains the FIFO, so a frame queued during the
 *            drain raises the line again -- brcmf_sdio_dpc's order.
 *
 * SIGNAL_EN is armed only while a waiter sleeps. Without a /dev/wifiirq client
 * (a netif that polls, `rxpoll`, the `pollrx` argument) this controller never
 * drives the shared line, as before this path existed. The timeout is the idle
 * poll interval the netif used without it, so a lost interrupt costs one idle
 * poll, never a hang. */
#define WIFI_IRQ          (32u + 126u)
#define WIFI_IRQ_WAIT_US  10000u
#define WIFI_RX_DRAIN_MAX 16u     /* non-data frames a data read may skip past */

#define SDIOD_INTSTATUS   0x20u   /* SDIO-DEV core registers (brcmfmac sdpcmd_regs) */
#define SDIOD_HOSTINTMASK 0x24u
#define I_HMB_FRAME_IND   0x40u   /* I_HMB_SW2: frame indication */

#define SDIO_CCCR_IEN     0x04u   /* function 0: Int Enable */
#define SDIO_CCCR_IEN_ALL 0x07u   /* IENM (master) + function 1 + function 2, as sdio_claim_irq */

/* The only state the handler touches, besides the controller's registers. */
typedef struct {
	volatile uint8_t *sdhci;
	volatile int fired;          /* set by the handler, cleared before each arm */
	volatile uint32_t claimed;   /* dispatches that were this controller's */
	volatile uint32_t declined;  /* dispatches of the shared line that were not */
} wifi_irqHandlerState_t;

static struct {
	wifi_irqHandlerState_t h;
	handle_t lock;
	handle_t cond;
	handle_t inth;
	int ready;    /* set up, /dev/wifiirq served; written before any thread starts */
	int disabled; /* `rxpoll`: waits fail, so the netif returns to polling (under lock) */
	int need_ack; /* an interrupt was handed out; the next data read acks the chip */
	uint32_t boot_status_en, boot_signal_en; /* as the boot firmware left them */
	uint32_t waits, wakes, level, timeouts;  /* waiter (under lock) */
	uint32_t acks, acks_noframe, ack_errs, drained; /* data path (message thread) */
	char why[48]; /* why the path is off, for `stats` */
} g_irq;


static int wifi_irqHandler(unsigned int n, void *arg)
{
	wifi_irqHandlerState_t *h = arg;
	uint32_t st = *(volatile uint32_t *)(h->sdhci + SDHCI_INT_STATUS);
	uint32_t en = *(volatile uint32_t *)(h->sdhci + SDHCI_SIGNAL_EN);

	(void)n;
	/* SIGNAL_EN never holds anything but CARD_INT (wifi_irqSetup clears it), so
	 * this is exactly "this controller is driving the line". */
	if ((st & en & SDHCI_INT_CARD) == 0u) {
		h->declined++;
		return -1;
	}
	*(volatile uint32_t *)(h->sdhci + SDHCI_SIGNAL_EN) = 0u;
	h->fired = 1;
	h->claimed++;
	return 0;
}


static void wifi_irqUndo(volatile uint8_t *sdhci)
{
	diag_bpWrite32(sdhci, g_sdio_core + SDIOD_HOSTINTMASK, 0u);
	(void)diag_sdioCmd52(sdhci, 1, 0, SDIO_CCCR_IEN, 0u, NULL);
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS_EN) = g_irq.boot_status_en;
}


/* Enable the chip's frame interrupt and register the handler. Runs once, after
 * bring-up and before the message thread starts, so it may use the bus. With
 * want == 0 it only silences the controller. Returns 0 when /dev/wifiirq can be
 * served; otherwise g_irq.why says why not and nothing is left enabled. */
static int wifi_irqSetup(int want)
{
	volatile uint8_t *sdhci = g_sdhci;
	uint32_t r[4] = { 0 };
	uint32_t mask;

	g_irq.boot_status_en = *(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS_EN);
	g_irq.boot_signal_en = *(volatile uint32_t *)(sdhci + SDHCI_SIGNAL_EN);
	/* Nothing on this controller may drive the shared line before a handler
	 * that can take it is in place. Nothing here used its interrupts so far. */
	*(volatile uint32_t *)(sdhci + SDHCI_SIGNAL_EN) = 0u;

	if (want == 0) {
		(void)snprintf(g_irq.why, sizeof(g_irq.why), "pollrx");
		return -1;
	}
	/* The ack is two CMD52s through the window diag_setWindow18 keeps. */
	if ((g_sdio_core & ~0x7fffu) != 0x18000000u) {
		(void)snprintf(g_irq.why, sizeof(g_irq.why), "SDIO core 0x%08x outside window",
			(unsigned)g_sdio_core);
		return -1;
	}

	diag_bpWrite32(sdhci, g_sdio_core + SDIOD_HOSTINTMASK, I_HMB_FRAME_IND);
	mask = diag_bpRead32(sdhci, g_sdio_core + SDIOD_HOSTINTMASK);
	(void)diag_sdioCmd52(sdhci, 1, 0, SDIO_CCCR_IEN, SDIO_CCCR_IEN_ALL, NULL);
	if ((diag_sdioCmd52(sdhci, 0, 0, SDIO_CCCR_IEN, 0u, r) != 0) ||
		((r[0] & 0xffu) != SDIO_CCCR_IEN_ALL) || (mask != I_HMB_FRAME_IND)) {
		(void)snprintf(g_irq.why, sizeof(g_irq.why), "readback hostintmask=0x%x ien=0x%02x",
			(unsigned)mask, (unsigned)(r[0] & 0xffu));
		wifi_irqUndo(sdhci);
		return -1;
	}
	*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS_EN) = g_irq.boot_status_en | SDHCI_INT_CARD;

	g_irq.h.sdhci = sdhci;
	/* Run the handler once from thread context (SIGNAL_EN is 0, so it only
	 * declines) to have its code resident before it runs in interrupt context. */
	(void)wifi_irqHandler(WIFI_IRQ, &g_irq.h);
	g_irq.h.declined = 0u;

	if ((mutexCreate(&g_irq.lock) != EOK) || (condCreate(&g_irq.cond) != EOK) ||
		(interrupt(WIFI_IRQ, wifi_irqHandler, &g_irq.h, g_irq.cond, &g_irq.inth) < 0)) {
		(void)snprintf(g_irq.why, sizeof(g_irq.why), "interrupt() registration failed");
		wifi_irqUndo(sdhci);
		return -1;
	}
	g_irq.ready = 1;
	return 0;
}


/* A /dev/wifiirq read: wait for the next frame interrupt. 1 = interrupt (one
 * byte written), 0 = timeout, -EIO once `rxpoll` has switched the path off. */
static int wifi_irqWait(void *dst, size_t size)
{
	volatile uint8_t *sdhci = g_irq.h.sdhci;
	volatile uint32_t *signal_en = (volatile uint32_t *)(sdhci + SDHCI_SIGNAL_EN);
	volatile uint32_t *status_en = (volatile uint32_t *)(sdhci + SDHCI_INT_STATUS_EN);
	uint32_t en;
	int got;

	if (size == 0u) {
		return 0;
	}
	mutexLock(g_irq.lock);
	if (g_irq.disabled != 0) {
		mutexUnlock(g_irq.lock);
		return -EIO;
	}
	g_irq.waits++;

	/* Restart card-interrupt detection before sampling it. SDHCI 2.2.18 has the
	 * host clear Card Interrupt Status Enable while it services the card and set
	 * it again afterwards; sdhci_enable_sdio_irq does the same. Only this thread
	 * writes STATUS_EN after setup, and SIGNAL_EN is 0 here, so the handler
	 * cannot run in between. */
	en = *status_en;
	*status_en = en & ~SDHCI_INT_CARD;
	*status_en = en | SDHCI_INT_CARD;

	if ((*(volatile uint32_t *)(sdhci + SDHCI_INT_STATUS) & SDHCI_INT_CARD) != 0u) {
		/* Still asserted: a frame was indicated after the last ack. */
		g_irq.level++;
		got = 1;
	}
	else {
		/* An interrupt between the arm and condWait is not lost: a broadcast
		 * with nobody waiting leaves the cond pending, and condWait returns at
		 * once. A stale pending wakeup returns with fired == 0 -- a timeout. */
		__atomic_store_n(&g_irq.h.fired, 0, __ATOMIC_SEQ_CST);
		*signal_en = SDHCI_INT_CARD;
		(void)condWait(g_irq.cond, g_irq.lock, WIFI_IRQ_WAIT_US);
		*signal_en = 0u;
		got = (__atomic_load_n(&g_irq.h.fired, __ATOMIC_SEQ_CST) != 0) ? 1 : 0;
		if (got != 0) {
			g_irq.wakes++;
		}
		else {
			g_irq.timeouts++;
		}
	}
	if (got != 0) {
		__atomic_store_n(&g_irq.need_ack, 1, __ATOMIC_SEQ_CST);
		*(uint8_t *)dst = (uint8_t)'i';
	}
	mutexUnlock(g_irq.lock);
	return got;
}


/* Clear the frame indication that raised the interrupt: read intstatus' low
 * byte (I_HMB_SW*) and write the bits back (write-1-to-clear). Message thread
 * only: it owns the bus. */
static void wifi_irqAck(volatile uint8_t *sdhci)
{
	uint32_t reg = (g_sdio_core + SDIOD_INTSTATUS) & 0x7fffu;
	uint32_t r[4] = { 0 };
	uint8_t st;

	diag_setWindow18(sdhci);
	if (diag_sdioCmd52(sdhci, 0, 1, reg, 0u, r) != 0) {
		g_irq.ack_errs++;
		return;
	}
	st = (uint8_t)(r[0] & 0xffu);
	if ((st != 0u) && (diag_sdioCmd52(sdhci, 1, 1, reg, st, NULL) != 0)) {
		g_irq.ack_errs++;
		return;
	}
	if ((st & I_HMB_FRAME_IND) != 0u) {
		g_irq.acks++;
	}
	else {
		g_irq.acks_noframe++;
	}
}


/* `sdclk <kHz>`: change the data clock while the daemon runs, for an A/B of the
 * two clocks in one boot. Runs on the message thread, which owns the bus, so no
 * transfer is in flight. The bus is checked before (nothing changes if that
 * fails) and after (a failure restores the old clock). The reply carries the
 * bus times of the clock being left, and they start again from zero. */
static int wifi_sdclk(const char *arg, char *out, int cap)
{
	unsigned old_khz = g_sdclk_khz;
	int khz = atoi(arg), n, m;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "SDCLK error: controller not initialized\n");
	}
	if ((khz < 1000) || (khz > (int)SDIO_CLK_HS_KHZ)) {
		return snprintf(out, (size_t)cap, "SDCLK error: %d kHz outside 1000..%u (now %u kHz, %u Hz)\n",
			khz, (unsigned)SDIO_CLK_HS_KHZ, old_khz, (unsigned)g_sdhci_sd_hz);
	}
	n = snprintf(out, (size_t)cap, "SDCLK leaving %u kHz: ", old_khz);
	if ((n < 0) || (n >= cap)) {
		return n;
	}
	m = diag_busTimeLine(out + n, cap - n);
	if ((m < 0) || (m >= (cap - n))) {
		return n;
	}
	n += m;

	if (diag_hsCheckLive(g_sdhci, "before") != 0) {
		m = snprintf(out + n, (size_t)(cap - n), "SDCLK refused: %s; the bus stays at %u kHz\n", g_hs_why, old_khz);
		return ((m > 0) && (m < (cap - n))) ? (n + m) : n;
	}
	if ((diag_sdhciDataClock(g_sdhci, (unsigned)khz) != 0) || (diag_hsCheckLive(g_sdhci, "after") != 0)) {
		g_hs_fallbacks++;
		(void)diag_sdhciResetCmdDat(g_sdhci);
		(void)diag_sdhciDataClock(g_sdhci, old_khz);
		(void)diag_sdioCmd52(g_sdhci, 1, 0, 0x06u, 0x01u, NULL); /* CCCR abort, function 1 */
		(void)diag_sdhciResetCmdDat(g_sdhci);
		m = snprintf(out + n, (size_t)(cap - n), "SDCLK fallback: %s; the bus is back at %u kHz (%u Hz)\n",
			g_hs_why, (unsigned)g_sdclk_khz, (unsigned)g_sdhci_sd_hz);
	}
	else {
		if ((unsigned)khz > SDIO_CLK_DS_KHZ) {
			g_hs_ups++;
		}
		m = snprintf(out + n, (size_t)(cap - n), "SDCLK ok: %u kHz -> %u kHz, sd=%u Hz hctl=0x%02x (bus times reset)\n",
			old_khz, (unsigned)khz, (unsigned)g_sdhci_sd_hz,
			(unsigned)(*(volatile uint32_t *)(g_sdhci + SDHCI_HOST_CTL) & 0xffu));
	}
	diag_busTimeReset();
	return ((m > 0) && (m < (cap - n))) ? (n + m) : n;
}

/* Block sizes `f2blk` accepts: powers of two the card's FBR takes, up to the
 * 512 bytes brcmfmac uses for this chip. */
static int wifi_f2blkValid(int sz)
{
	return (sz == 64) || (sz == 128) || (sz == 256) || (sz == 512);
}

/* `f2blk <n>`: change the function-2 block size while the daemon runs, for an
 * A/B of TX block sizes in one boot (see F2_BLKSZ_DEFAULT). Message thread, so
 * no transfer is in flight. Linux sets it once, before the firmware starts;
 * `f2blk=<n>` on the command line does that. The reply carries the bus times of
 * the size being left, and they start again from zero. */
static int wifi_f2blk(const char *arg, char *out, int cap)
{
	uint32_t old = g_f2_blksz;
	int sz = atoi(arg), n, m, rc;

	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "F2BLK error: controller not initialized\n");
	}
	if (wifi_f2blkValid(sz) == 0) {
		return snprintf(out, (size_t)cap, "F2BLK error: %d is not 64, 128, 256 or 512 (now %u)\n",
			sz, (unsigned)old);
	}
	n = snprintf(out, (size_t)cap, "F2BLK leaving %u: ", (unsigned)old);
	if ((n < 0) || (n >= cap)) {
		return n;
	}
	m = diag_busTimeLine(out + n, cap - n);
	if ((m < 0) || (m >= (cap - n))) {
		return n;
	}
	n += m;
	rc = diag_f2SetBlockSize(g_sdhci, (uint32_t)sz);
	if (rc != 0) {
		(void)diag_f2SetBlockSize(g_sdhci, old);
		m = snprintf(out + n, (size_t)(cap - n), "F2BLK failed: rc=%d; back at %u\n", rc, (unsigned)g_f2_blksz);
	}
	else {
		m = snprintf(out + n, (size_t)(cap - n), "F2BLK ok: %u -> %u (read back; bus times reset)\n",
			(unsigned)old, (unsigned)g_f2_blksz);
	}
	diag_busTimeReset();
	return ((m > 0) && (m < (cap - n))) ? (n + m) : n;
}

/* `rxpoll`: switch the interrupt path off for the rest of this boot. The netif
 * sees its next /dev/wifiirq read fail and goes back to polling -- the same
 * binary and the same boot measured both ways. */
static int wifi_rxPoll(char *out, int cap)
{
	if (g_irq.ready == 0) {
		return snprintf(out, (size_t)cap, "RXPOLL already polling (%s)\n", g_irq.why);
	}
	mutexLock(g_irq.lock);
	g_irq.disabled = 1;
	mutexUnlock(g_irq.lock);
	(void)snprintf(g_irq.why, sizeof(g_irq.why), "rxpoll");
	return snprintf(out, (size_t)cap, "RXPOLL ok: /dev/wifiirq reads now fail; the netif polls\n");
}


/* ---- Radio probes and settings (`iovar`, `ioctl`, `atjoin`, `country`, `bw`,
 * `chanspecs`, `dump`) ------------------------------------------------------
 *
 * What the radio negotiated and how the firmware aggregates bound throughput
 * before any host-side change does, so these read and set it at run time, in
 * the boot being measured. Like every control command they run on the message
 * thread (the bus owner), and the reply wait drains the shared F2 FIFO, dropping
 * the data frames it meets: use them between transfers, not during one. */
#define WIFI_CMD_TOKS  20
#define WIFI_CMD_WORDS 16u

static uint32_t g_cmd_reqid = 0u;
static uint8_t g_cmd_rx[1536];

/* BCDC request ids for these commands: 0x400..0x7ff, clear of the join's small
 * counters and the fixed ids of `stats`/`leave`/`mac`. */
static uint32_t wifi_cmdReqid(void)
{
	return 0x400u + ((g_cmd_reqid++) & 0x3ffu);
}


/* Split a command written to /dev/wifi (not NUL-terminated) into blank-separated
 * tokens inside `line`. Returns the token count. */
static int wifi_cmdTokens(const void *data, size_t size, char *line, size_t cap, char **tok, int max)
{
	size_t n = (size < (cap - 1u)) ? size : (cap - 1u);
	char *p = line;
	int nt = 0;

	memcpy(line, data, n);
	line[n] = '\0';
	while ((*p != '\0') && (nt < max)) {
		while ((*p == ' ') || (*p == '\t') || (*p == '\n') || (*p == '\r')) {
			*p++ = '\0';
		}
		if (*p == '\0') {
			break;
		}
		tok[nt++] = p;
		while ((*p != '\0') && (*p != ' ') && (*p != '\t') && (*p != '\n') && (*p != '\r')) {
			p++;
		}
	}
	return nt;
}


/* One 32-bit word: decimal, 0x hex, or negative. 0, or -1 if it is not a number. */
static int wifi_cmdWord(const char *s, uint32_t *w)
{
	char *end;
	long long v = strtoll(s, &end, 0);

	if ((end == s) || (*end != '\0') || (v < -2147483648LL) || (v > 0xffffffffLL)) {
		return -1;
	}
	*w = (uint32_t)v;
	return 0;
}


/* Words of tok[from..nt) into w[] (at most WIFI_CMD_WORDS). Count, or -1. */
static int wifi_cmdWords(char **tok, int from, int nt, uint32_t *w)
{
	int i;

	if ((nt - from) > (int)WIFI_CMD_WORDS) {
		return -1;
	}
	for (i = from; i < nt; ++i) {
		if (wifi_cmdWord(tok[i], &w[i - from]) != 0) {
			return -1;
		}
	}
	return (nt > from) ? (nt - from) : 0;
}


/* A reply as little-endian words, up to the last non-zero one (at least one). */
static int wifi_fmtWords(char *out, int cap, const uint8_t *b, uint32_t len)
{
	uint32_t nw = len / 4u, last = 0u, i;
	int n = 0, m;

	if (nw > WIFI_CMD_WORDS) {
		nw = WIFI_CMD_WORDS;
	}
	for (i = 0u; i < nw; ++i) {
		if (diag_le32(b + 4u * i) != 0u) {
			last = i + 1u;
		}
	}
	if ((last == 0u) && (nw != 0u)) {
		last = 1u;
	}
	for (i = 0u; i < last; ++i) {
		uint32_t v = diag_le32(b + 4u * i);

		m = snprintf(out + n, (size_t)(cap - n), " %d(0x%x)", (int)(int32_t)v, (unsigned)v);
		if ((m < 0) || (m >= (cap - n))) {
			break;
		}
		n += m;
	}
	return n;
}


/* Chanspec, d11ac encoding (brcmu_d11.h): channel 7:0, bandwidth 13:11, band 15:14. */
static unsigned wifi_chspecMhz(uint32_t cs)
{
	switch (cs & 0x3800u) {
		case 0x1000u: return 20u;
		case 0x1800u: return 40u;
		case 0x2000u: return 80u;
		case 0x2800u: return 160u;
		default: return 0u;
	}
}

static const char *wifi_chspecBand(uint32_t cs)
{
	switch (cs & 0xc000u) {
		case 0x0000u: return "2g";
		case 0xc000u: return "5g";
		default: return "?";
	}
}


/* `iovar get <name> [in-words]` / `iovar set <name> <words>`. A GET may need
 * input words (`bw_cap` takes the band). */
static int wifi_iovarCmd(char **tok, int nt, char *out, int cap)
{
	uint8_t in[4u * WIFI_CMD_WORDS];
	uint32_t w[WIFI_CMD_WORDS], rlen = 0u, dlen;
	int is_set, nw, rc, n, k;

	if ((nt < 3) || ((strcmp(tok[1], "get") != 0) && (strcmp(tok[1], "set") != 0)) ||
			(strlen(tok[2]) >= WIFI_IOVAR_NAME)) {
		return snprintf(out, (size_t)cap, "IOVAR usage: iovar get <name> [words] | iovar set <name> <words>\n");
	}
	is_set = (tok[1][0] == 's');
	nw = wifi_cmdWords(tok, 3, nt, w);
	if ((nw < 0) || (is_set && (nw == 0))) {
		return snprintf(out, (size_t)cap, "IOVAR error: words are 32-bit numbers (%u at most)%s\n",
			WIFI_CMD_WORDS, is_set ? ", and a set needs one" : "");
	}
	memset(in, 0, sizeof(in));
	for (k = 0; k < nw; ++k) {
		wifi_putLe32(in + 4 * k, w[k]);
	}
	/* A GET's buffer is also where the answer goes: leave room for one. */
	dlen = is_set ? (4u * (uint32_t)nw) : sizeof(in);
	rc = diag_iovar(g_sdhci, g_sdio_core, is_set, tok[2], in, dlen,
		g_cmd_rx, sizeof(g_cmd_rx), &rlen, wifi_cmdReqid(), g_data_seq++);
	n = snprintf(out, (size_t)cap, "IOVAR %s %s rc=%d", tok[1], tok[2], rc);
	if ((n > 0) && (n < cap) && !is_set && (rc == 0)) {
		n += snprintf(out + n, (size_t)(cap - n), " len=%u:", (unsigned)rlen);
		if (n < cap) {
			n += wifi_fmtWords(out + n, cap - n, g_cmd_rx, (rlen < sizeof(g_cmd_rx)) ? rlen : sizeof(g_cmd_rx));
		}
	}
	if ((n > 0) && (n < (cap - 1))) {
		out[n++] = '\n';
		out[n] = '\0';
	}
	return n;
}


/* `ioctl get <cmd> [in-words]` / `ioctl set <cmd> <words>`: a raw WLC command
 * (BRCMF_C_*, fwil.h), e.g. 85/86 = GET/SET_PM. No list of allowed commands:
 * this is a probe, and 2/3 (UP/DOWN) will take the link with them. */
static int wifi_ioctlCmd(char **tok, int nt, char *out, int cap)
{
	uint8_t in[4u * WIFI_CMD_WORDS];
	uint32_t w[WIFI_CMD_WORDS], cmd, rlen = 0u, dlen;
	int is_set, nw, rc, n, k;

	if ((nt < 3) || ((strcmp(tok[1], "get") != 0) && (strcmp(tok[1], "set") != 0)) ||
			(wifi_cmdWord(tok[2], &cmd) != 0)) {
		return snprintf(out, (size_t)cap, "IOCTL usage: ioctl get <cmd> [words] | ioctl set <cmd> <words>\n");
	}
	is_set = (tok[1][0] == 's');
	nw = wifi_cmdWords(tok, 3, nt, w);
	if ((nw < 0) || (is_set && (nw == 0))) {
		return snprintf(out, (size_t)cap, "IOCTL error: words are 32-bit numbers (%u at most)%s\n",
			WIFI_CMD_WORDS, is_set ? ", and a set needs one" : "");
	}
	memset(in, 0, sizeof(in));
	for (k = 0; k < nw; ++k) {
		wifi_putLe32(in + 4 * k, w[k]);
	}
	dlen = (nw > 0) ? (4u * (uint32_t)nw) : 4u;
	if (!is_set && (dlen < 64u)) {
		dlen = 64u;
	}
	rc = diag_bcdcCmd(g_sdhci, g_sdio_core, is_set, cmd, in, dlen,
		g_cmd_rx, sizeof(g_cmd_rx), &rlen, wifi_cmdReqid(), g_data_seq++);
	n = snprintf(out, (size_t)cap, "IOCTL %s %u rc=%d", tok[1], (unsigned)cmd, rc);
	if ((n > 0) && (n < cap) && !is_set && (rc == 0)) {
		n += snprintf(out + n, (size_t)(cap - n), " len=%u:", (unsigned)rlen);
		if (n < cap) {
			n += wifi_fmtWords(out + n, cap - n, g_cmd_rx, (rlen < sizeof(g_cmd_rx)) ? rlen : sizeof(g_cmd_rx));
		}
	}
	if ((n > 0) && (n < (cap - 1))) {
		out[n++] = '\n';
		out[n] = '\0';
	}
	return n;
}


/* The queued join settings, one line. */
static int wifi_atjoinList(char *out, int cap)
{
	uint32_t i, k;
	int n, m;

	n = snprintf(out, (size_t)cap, "ATJOIN country=%s (rc %d)", (g_country[0] != '\0') ? g_country : "-", g_country_rc);
	for (i = 0u; (i < g_atjoin_n) && (n > 0) && (n < cap); ++i) {
		m = snprintf(out + n, (size_t)(cap - n), " %s", g_atjoin[i].name);
		for (k = 0u; (k < g_atjoin[i].nwords) && (m > 0) && ((n + m) < cap); ++k) {
			m += snprintf(out + n + m, (size_t)(cap - n - m), "%c%d", (k == 0u) ? '=' : ',', (int)(int32_t)g_atjoin[i].w[k]);
		}
		if ((m > 0) && ((n + m) < cap)) {
			m += snprintf(out + n + m, (size_t)(cap - n - m), " (rc %d)", g_atjoin[i].rc);
		}
		if ((m < 0) || ((n + m) >= cap)) {
			break;
		}
		n += m;
	}
	if ((n > 0) && (n < cap)) {
		m = snprintf(out + n, (size_t)(cap - n), "; last join WLC_DOWN rc=%d (-100: not sent)\n", g_join_down_rc);
		if ((m > 0) && ((n + m) < cap)) {
			n += m;
		}
	}
	return n;
}


/* `atjoin` lists, `atjoin clear` empties (country included), `atjoin <name>
 * <words>` queues an iovar for every following join. */
static int wifi_atjoinCmd(char **tok, int nt, char *out, int cap)
{
	uint32_t w[WIFI_CMD_WORDS];
	int nw, n;

	if (nt == 1) {
		return wifi_atjoinList(out, cap);
	}
	if ((nt == 2) && (strcmp(tok[1], "clear") == 0)) {
		g_atjoin_n = 0u;
		g_country[0] = '\0';
		n = snprintf(out, (size_t)cap, "ATJOIN cleared: the next join sends no WLC_DOWN. ");
		return ((n > 0) && (n < cap)) ? (n + wifi_atjoinList(out + n, cap - n)) : n;
	}
	nw = wifi_cmdWords(tok, 2, nt, w);
	if ((nw <= 0) || (wifi_atjoinAdd(tok[1], w, (uint32_t)nw) != 0)) {
		return snprintf(out, (size_t)cap, "ATJOIN error: atjoin <name> <1..%u words> (%u entries at most)\n",
			WIFI_ATJOIN_WORDS, WIFI_ATJOIN_MAX);
	}
	n = snprintf(out, (size_t)cap, "ATJOIN queued for the next join (`wifi leave` makes the netif rejoin). ");
	return ((n > 0) && (n < cap)) ? (n + wifi_atjoinList(out + n, cap - n)) : n;
}


/* `country` reads it, `country <CC>` sets it now and at every join, `country -`
 * stops setting it (the firmware keeps the last one until it is reloaded). */
static int wifi_countryCmd(char **tok, int nt, char *out, int cap)
{
	uint8_t cc[12];
	uint32_t rlen = 0u;
	int rc;

	if (nt == 1) {
		memset(cc, 0, sizeof(cc));
		rc = diag_iovar(g_sdhci, g_sdio_core, 0, "country", cc, sizeof(cc),
			g_cmd_rx, sizeof(g_cmd_rx), &rlen, wifi_cmdReqid(), g_data_seq++);
		if ((rc != 0) || (rlen < 12u)) {
			return snprintf(out, (size_t)cap, "COUNTRY get rc=%d len=%u\n", rc, (unsigned)rlen);
		}
		return snprintf(out, (size_t)cap, "COUNTRY abbrev=%.4s rev=%d ccode=%.4s (queued for joins: %s)\n",
			(const char *)g_cmd_rx, (int)(int32_t)diag_le32(g_cmd_rx + 4), (const char *)(g_cmd_rx + 8),
			(g_country[0] != '\0') ? g_country : "-");
	}
	if (strcmp(tok[1], "-") == 0) {
		g_country[0] = '\0';
		return snprintf(out, (size_t)cap, "COUNTRY no longer set at joins\n");
	}
	if ((strlen(tok[1]) != 2u) || (tok[1][0] < 'A') || (tok[1][0] > 'Z') || (tok[1][1] < 'A') || (tok[1][1] > 'Z')) {
		return snprintf(out, (size_t)cap, "COUNTRY error: a two-letter upper-case ISO 3166 code, e.g. PL\n");
	}
	g_country[0] = tok[1][0];
	g_country[1] = tok[1][1];
	g_country[2] = '\0';
	wifi_countryPayload(cc, g_country);
	g_country_rc = diag_iovar(g_sdhci, g_sdio_core, 1, "country", cc, sizeof(cc),
		NULL, 0u, NULL, wifi_cmdReqid(), g_data_seq++);
	return snprintf(out, (size_t)cap, "COUNTRY set %s rev 0 rc=%d; also set at every join\n", g_country, g_country_rc);
}


/* `bw <2g|5g> <20|40|80>`: queue `bw_cap` for that band (80 on 5g only). */
static int wifi_bwCmd(char **tok, int nt, char *out, int cap)
{
	uint32_t band;
	int n;

	if ((nt == 3) && ((strcmp(tok[1], "2g") == 0) || (strcmp(tok[1], "5g") == 0))) {
		band = (tok[1][0] == '2') ? WLC_BAND_2G : WLC_BAND_5G;
		if (wifi_atjoinBw(band, atoi(tok[2])) == 0) {
			n = snprintf(out, (size_t)cap, "BW queued %s %s MHz for the next join (`wifi leave` makes the netif rejoin). ",
				tok[1], tok[2]);
			return ((n > 0) && (n < cap)) ? (n + wifi_atjoinList(out + n, cap - n)) : n;
		}
	}
	return snprintf(out, (size_t)cap, "BW usage: bw 2g <20|40> | bw 5g <20|40|80>\n");
}


/* `chanspecs`: what the firmware will use under the current locale, by band
 * and width (channel = centre channel for 40/80). */
static int wifi_chanspecsCmd(char *out, int cap)
{
	static const char *const bands[2] = { "2g", "5g" };
	static const unsigned widths[3] = { 20u, 40u, 80u };
	uint8_t z[496];
	uint32_t rlen = 0u, cnt, i, b, wi;
	int rc, n, m;

	memset(z, 0, sizeof(z));
	rc = diag_iovar(g_sdhci, g_sdio_core, 0, "chanspecs", z, sizeof(z),
		g_cmd_rx, sizeof(g_cmd_rx), &rlen, wifi_cmdReqid(), g_data_seq++);
	if ((rc != 0) || (rlen < 4u)) {
		return snprintf(out, (size_t)cap, "CHANSPECS rc=%d len=%u\n", rc, (unsigned)rlen);
	}
	cnt = diag_le32(g_cmd_rx);
	if (cnt > ((rlen - 4u) / 4u)) {
		cnt = (rlen - 4u) / 4u; /* what fits in the reply */
	}
	n = snprintf(out, (size_t)cap, "CHANSPECS count=%u (listed %u)\n", (unsigned)diag_le32(g_cmd_rx), (unsigned)cnt);
	for (b = 0u; b < 2u; ++b) {
		for (wi = 0u; wi < 3u; ++wi) {
			int any = 0;

			for (i = 0u; (i < cnt) && (n > 0) && (n < cap); ++i) {
				uint32_t cs = diag_le32(g_cmd_rx + 4u + 4u * i);

				if ((strcmp(wifi_chspecBand(cs), bands[b]) != 0) || (wifi_chspecMhz(cs) != widths[wi])) {
					continue;
				}
				if (any) {
					m = snprintf(out + n, (size_t)(cap - n), " %u", (unsigned)(cs & 0xffu));
				}
				else {
					m = snprintf(out + n, (size_t)(cap - n), "CHANSPECS %s/%u: %u", bands[b], widths[wi], (unsigned)(cs & 0xffu));
				}
				if ((m < 0) || (m >= (cap - n))) {
					break;
				}
				n += m;
				any = 1;
			}
			if (any && (n > 0) && (n < (cap - 1))) {
				out[n++] = '\n';
				out[n] = '\0';
			}
		}
	}
	return n;
}


/* `dump <name>`: the firmware's text dump, e.g. `dump ampdu` (A-MPDU sizes and
 * block-ack state, if this firmware build carries it). */
static int wifi_dumpCmd(char **tok, int nt, char *out, int cap)
{
	uint8_t req[1280];
	uint32_t rlen = 0u, len;
	int rc, n;

	if ((nt != 2) || (strlen(tok[1]) >= 32u)) {
		return snprintf(out, (size_t)cap, "DUMP usage: dump <name>, e.g. dump ampdu\n");
	}
	memset(req, 0, sizeof(req));
	memcpy(req, tok[1], strlen(tok[1]));
	rc = diag_iovar(g_sdhci, g_sdio_core, 0, "dump", req, sizeof(req),
		g_cmd_rx, sizeof(g_cmd_rx) - 1u, &rlen, wifi_cmdReqid(), g_data_seq++);
	if (rc != 0) {
		return snprintf(out, (size_t)cap, "DUMP %s rc=%d (not in this firmware, or not here)\n", tok[1], rc);
	}
	len = (rlen < (sizeof(g_cmd_rx) - 1u)) ? rlen : (sizeof(g_cmd_rx) - 1u);
	g_cmd_rx[len] = 0u;
	n = snprintf(out, (size_t)cap, "DUMP %s (%u bytes):\n%s\n", tok[1], (unsigned)rlen, (const char *)g_cmd_rx);
	return (n < cap) ? n : (cap - 1);
}


/* `batch [0|1]`: show or set whether the netif should move frames through
 * /dev/wifibatch. Takes effect at the netif's next status poll (~3 s); the
 * `WIFISTATS ipc` line of `stats` shows which path the frames took. */
static int wifi_batchCmd(char **tok, int nt, char *out, int cap)
{
	if (nt >= 2) {
		if ((strcmp(tok[1], "1") == 0) || (strcmp(tok[1], "on") == 0)) {
			g_bat.want = 1;
		}
		else if ((strcmp(tok[1], "0") == 0) || (strcmp(tok[1], "off") == 0)) {
			g_bat.want = 0;
		}
		else {
			return snprintf(out, (size_t)cap, "BATCH usage: batch [0|1]\n");
		}
	}
	return snprintf(out, (size_t)cap, "BATCH batch=%d (the netif follows within ~3 s)\n", g_bat.want);
}


/* The `/dev/wifi` entry for all of the above. Returns the reply length, or -1
 * when the command is not one of them. */
static int wifi_radioCmd(const void *data, size_t size, char *out, int cap)
{
	char line[192];
	char *tok[WIFI_CMD_TOKS];
	int nt = wifi_cmdTokens(data, size, line, sizeof(line), tok, WIFI_CMD_TOKS);

	if (nt == 0) {
		return -1;
	}
	if (strcmp(tok[0], "batch") == 0) {
		return wifi_batchCmd(tok, nt, out, cap); /* a flag: no bus needed */
	}
	if (strcmp(tok[0], "atjoin") == 0) {
		return wifi_atjoinCmd(tok, nt, out, cap); /* only queues: no bus needed */
	}
	if (strcmp(tok[0], "bw") == 0) {
		return wifi_bwCmd(tok, nt, out, cap);
	}
	if ((strcmp(tok[0], "iovar") != 0) && (strcmp(tok[0], "ioctl") != 0) && (strcmp(tok[0], "country") != 0) &&
			(strcmp(tok[0], "chanspecs") != 0) && (strcmp(tok[0], "dump") != 0)) {
		return -1;
	}
	if (g_sdhci == NULL) {
		return snprintf(out, (size_t)cap, "wifi: controller not initialized\n");
	}
	if (strcmp(tok[0], "iovar") == 0) {
		return wifi_iovarCmd(tok, nt, out, cap);
	}
	if (strcmp(tok[0], "ioctl") == 0) {
		return wifi_ioctlCmd(tok, nt, out, cap);
	}
	if (strcmp(tok[0], "country") == 0) {
		return wifi_countryCmd(tok, nt, out, cap);
	}
	if (strcmp(tok[0], "chanspecs") == 0) {
		return wifi_chanspecsCmd(out, cap);
	}
	return wifi_dumpCmd(tok, nt, out, cap);
}


static void wifi_irqThread(void *arg)
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
				msg.o.err = wifi_irqWait(msg.o.data, msg.o.size);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0400;
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


/* Where the per-frame time goes inside this daemon. Each /dev/wifidata request
 * is timed from the generic timer: two register reads, no syscall. (It used to
 * take a clock_gettime pair, which cost TX 1.73 -> 1.10 MB/s, so it was off by
 * default and the tx/rx_us fields of `stats` read 0.) Read HITs (a frame came
 * back) and MISSes (empty FIFO) are kept apart: their costs differ by an order
 * of magnitude.
 *
 * The busy window says how much of the wall time the single message thread
 * spends serving requests at all. Near 100 % during a transfer, this thread is
 * the bottleneck; well below it, the time per frame goes elsewhere -- the IPC
 * itself, lwip, the application. `stats` reports the window and restarts it. */
static uint64_t g_rx_hit_ticks = 0, g_rx_miss_ticks = 0, g_tx_ticks = 0;
static uint32_t g_rx_hits = 0, g_rx_misses = 0, g_tx_calls = 0;
static uint64_t g_busy_ticks = 0, g_busy_since = 0;
static uint32_t g_busy_msgs = 0;

#define WIFI_T0(v)       uint64_t v = diag_ticks()
#define WIFI_ACC(acc, v) ((acc) += diag_ticks() - (v))

static uint64_t diag_ticksToUs(uint64_t t)
{
	if (g_bt_freq == 0u) {
		__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(g_bt_freq));
	}
	if (g_bt_freq == 0u) {
		return 0u;
	}
	return (t / g_bt_freq) * 1000000u + ((t % g_bt_freq) * 1000000u) / g_bt_freq;
}


/* x/y with two decimals, for a "frames per request" ratio. */
static int wifi_fmtRatio(char *out, size_t cap, uint32_t x, uint32_t y)
{
	uint64_t r = (y != 0u) ? (((uint64_t)x * 100u + (y / 2u)) / y) : 0u;

	return snprintf(out, cap, "%llu.%02llu", (unsigned long long)(r / 100u), (unsigned long long)(r % 100u));
}


/* The `batch` and `ipc` lines of `stats`. `ipc` is the A/B number: frames per
 * frame-device request in each direction, over both /dev/wifidata (one frame
 * per request, an empty read counts as a request) and /dev/wifibatch, since the
 * previous `stats` -- so `wifi stats`, a transfer, `wifi stats` gives the
 * transfer alone. /dev/wifiirq waits are not data requests and are not counted
 * (`rxirq` has them). */
static int wifi_statsBatch(char *out, int cap)
{
	static uint32_t prev_txr = 0u, prev_txf = 0u, prev_rxr = 0u, prev_rxf = 0u;
	uint32_t txr = g_tx_calls + g_bat.tx_msgs;
	uint32_t txf = g_tx_calls + g_bat.tx_frames;
	uint32_t rxr = g_rx_hits + g_rx_misses + g_bat.rx_msgs;
	uint32_t rxf = g_rx_hits + g_bat.rx_frames;
	char tx_fpi[24], rx_fpi[24];
	int n;

	(void)wifi_fmtRatio(tx_fpi, sizeof(tx_fpi), txf - prev_txf, txr - prev_txr);
	(void)wifi_fmtRatio(rx_fpi, sizeof(rx_fpi), rxf - prev_rxf, rxr - prev_rxr);
	n = snprintf(out, (size_t)cap,
		"WIFISTATS batch want=%d tx_msgs=%u tx_frames=%u tx_max=%u tx_partial=%u tx_bad=%u tx_us_avg=%llu "
		"rx_msgs=%u rx_empty=%u rx_frames=%u rx_max=%u rx_capped=%u rx_drained=%u rx_skipped=%u rx_us_avg=%llu\n"
		"WIFISTATS ipc tx req=%u frames=%u fpi=%s rx req=%u frames=%u fpi=%s (since the previous stats)\n",
		g_bat.want, (unsigned)g_bat.tx_msgs, (unsigned)g_bat.tx_frames, (unsigned)g_bat.tx_max,
		(unsigned)g_bat.tx_partial, (unsigned)g_bat.tx_bad,
		(unsigned long long)(g_bat.tx_msgs ? diag_ticksToUs(g_bat.tx_ticks / g_bat.tx_msgs) : 0u),
		(unsigned)g_bat.rx_msgs, (unsigned)g_bat.rx_empty, (unsigned)g_bat.rx_frames, (unsigned)g_bat.rx_max,
		(unsigned)g_bat.rx_capped, (unsigned)g_bat.rx_drained, (unsigned)g_bat.rx_skipped,
		(unsigned long long)(g_bat.rx_msgs ? diag_ticksToUs(g_bat.rx_ticks / g_bat.rx_msgs) : 0u),
		(unsigned)(txr - prev_txr), (unsigned)(txf - prev_txf), tx_fpi,
		(unsigned)(rxr - prev_rxr), (unsigned)(rxf - prev_rxf), rx_fpi);
	prev_txr = txr;
	prev_txf = txf;
	prev_rxr = rxr;
	prev_rxf = rxf;
	return n;
}


/* `stats`: per-frame timing of the data path, so a throughput number can be
 * attributed instead of guessed. */
static int wifi_stats(char *out, int cap)
{
	/* The negotiated PHY rate bounds what any SDIO or host change can buy, so it
	 * is reported next to the throughput counters. Only here, on demand: like any
	 * control command, the GET drains the shared F2 FIFO and drops the data
	 * frames it meets, so it must not go into the periodically polled `status`. */
	uint8_t rbuf[8] = { 0 };
	uint8_t z[4] = { 0 };
	uint32_t rlen = 0u, rate = 0u, cs = 0u, pm = 0xffffffffu;
	int rate_rc = -1, cs_rc = -1, pm_rc = -1, n, m;

	if ((g_sdhci != NULL) && wifi_isJoined()) {
		rate_rc = diag_bcdcCmd(g_sdhci, g_sdio_core, 0, BRCMF_C_GET_RATE, NULL, 4u,
			rbuf, sizeof(rbuf), &rlen, 211u, g_data_seq++);
		if ((rate_rc == 0) && (rlen >= 4u)) {
			rate = diag_le32(rbuf);
		}
		/* The channel and width the association runs on, and the power-save
		 * mode: with the rate, the three things the air ceiling depends on. */
		cs_rc = diag_iovar(g_sdhci, g_sdio_core, 0, "chanspec", z, 4u,
			rbuf, sizeof(rbuf), &rlen, 212u, g_data_seq++);
		if ((cs_rc == 0) && (rlen >= 4u)) {
			cs = diag_le32(rbuf);
		}
		pm_rc = diag_bcdcCmd(g_sdhci, g_sdio_core, 0, BRCMF_C_GET_PM, z, 4u,
			rbuf, sizeof(rbuf), &rlen, 213u, g_data_seq++);
		if ((pm_rc == 0) && (rlen >= 4u)) {
			pm = diag_le32(rbuf);
		}
	}

	n = snprintf(out, (size_t)cap, "WIFISTATS phy rate=%u.%u Mbit/s rc=%d\n"
		"WIFISTATS radio chanspec=0x%04x ch=%u bw=%u band=%s rc=%d pm=%d rc=%d atjoin=%u country=%s down_rc=%d\n",
		(unsigned)(rate / 2u), (unsigned)((rate & 1u) * 5u), rate_rc,
		(unsigned)(cs & 0xffffu), (unsigned)(cs & 0xffu), wifi_chspecMhz(cs), wifi_chspecBand(cs), cs_rc,
		(int)(int32_t)pm, pm_rc, (unsigned)g_atjoin_n, (g_country[0] != '\0') ? g_country : "-", g_join_down_rc);
	if ((n < 0) || (n >= cap)) {
		return n;
	}
	m = snprintf(out + n, (size_t)(cap - n),
		"WIFISTATS tx_calls=%u tx_us_total=%llu tx_us_avg=%llu\n"
		"WIFISTATS rx_hits=%u rx_hit_us_total=%llu rx_hit_us_avg=%llu\n"
		"WIFISTATS rx_misses=%u rx_miss_us_total=%llu rx_miss_us_avg=%llu\n"
		"WIFISTATS frames tx_ok=%u tx_err=%u rx_ok=%u rx_err=%u rx_garbage=%u\n"
		"WIFISTATS flowctl tx_seq=%u tx_max=%u avail=%u fc_mask=0x%02x blocked=%u updates=%u resyncs=%u\n"
		"WIFISTATS rxerr xfer=%u sdoff=%u ethoff=%u toobig=%u\n"
		"WIFISTATS rxxfer hdr=%u oversize=%u body=%u max_announced_len=%u chan=%u (cap %u)\n"
		"WIFISTATS rxbig ok=%u max_len=%u rxglom_off_rc=%d\n"
		"WIFISTATS glom descs=%u supers=%u subframes=%u bad=%u\n"
		"WIFISTATS pio mode=%s slow_waits=%u slow_max_us=%u timeouts=%u\n"
		"WIFISTATS sbwin writes=%u skips=%u\n"
		"WIFISTATS sdio target=%u kHz sd=%u Hz hctl=0x%02x timeout=0x%x f2blk=%u hs_ups=%u hs_fallbacks=%u%s%s\n"
		"WIFISTATS rxirq mode=%s%s%s waits=%u wakes=%u level=%u timeouts=%u\n"
		"WIFISTATS rxirq isr claimed=%u declined=%u acks=%u acks_noframe=%u ack_errs=%u drained=%u\n",
		g_tx_calls, (unsigned long long)diag_ticksToUs(g_tx_ticks),
		(unsigned long long)(g_tx_calls ? diag_ticksToUs(g_tx_ticks / g_tx_calls) : 0u),
		g_rx_hits, (unsigned long long)diag_ticksToUs(g_rx_hit_ticks),
		(unsigned long long)(g_rx_hits ? diag_ticksToUs(g_rx_hit_ticks / g_rx_hits) : 0u),
		g_rx_misses, (unsigned long long)diag_ticksToUs(g_rx_miss_ticks),
		(unsigned long long)(g_rx_misses ? diag_ticksToUs(g_rx_miss_ticks / g_rx_misses) : 0u),
		g_frame_tx_ok, g_frame_tx_err, g_frame_rx_ok, g_frame_rx_err, g_frame_rx_garbage,
		g_data_seq, g_tx_max, (unsigned)(uint8_t)(g_tx_max - g_data_seq), g_fc_mask,
		g_tx_blocked, g_fc_updates, g_rx_resyncs,
		g_rxe_xfer, g_rxe_sdoff, g_rxe_ethoff, g_rxe_toobig,
		g_rxe_hdr, g_rxe_big, g_rxe_body, g_rxe_big_len, g_rxe_big_chan, (unsigned)F2_FRAME_MAX,
		g_rx_big_ok, g_rx_big_ok_len, g_join_rxglom_rc,
		g_glom_descs, g_glom_supers, g_glom_subs, g_glom_bad,
		(g_pio_legacy != 0) ? "legacy" : "level", (unsigned)g_pio_slow_waits,
		(unsigned)g_pio_slow_max_us, (unsigned)g_pio_timeouts,
		(unsigned)g_sbwin_writes, (unsigned)g_sbwin_skips,
		g_sdclk_khz, (unsigned)g_sdhci_sd_hz,
		(g_sdhci != NULL) ? (unsigned)(*(volatile uint32_t *)(g_sdhci + SDHCI_HOST_CTL) & 0xffu) : 0u,
		(g_sdhci != NULL) ? (unsigned)((*(volatile uint32_t *)(g_sdhci + SDHCI_CLK_TIMEOUT_RESET) >> 16) & 0xfu) : 0u,
		(unsigned)g_f2_blksz, (unsigned)g_hs_ups, (unsigned)g_hs_fallbacks,
		(g_hs_why[0] != '\0') ? " last_fail=" : "", g_hs_why,
		((g_irq.ready != 0) && (g_irq.disabled == 0)) ? "irq" : "poll",
		(g_irq.why[0] != '\0') ? " off=" : "", g_irq.why,
		(unsigned)g_irq.waits, (unsigned)g_irq.wakes, (unsigned)g_irq.level, (unsigned)g_irq.timeouts,
		(unsigned)g_irq.h.claimed, (unsigned)g_irq.h.declined, (unsigned)g_irq.acks,
		(unsigned)g_irq.acks_noframe, (unsigned)g_irq.ack_errs, (unsigned)g_irq.drained);
	if (m < 0) {
		return m;
	}
	if ((n + m) >= cap) {
		return cap - 1;
	}
	n += m;
	m = diag_busTimeLine(out + n, cap - n);
	if (m < 0) {
		return n;
	}
	if ((n + m) >= cap) {
		return cap - 1;
	}
	n += m;

	{
		uint64_t now = diag_ticks();
		uint64_t wall = (g_busy_since != 0u) ? (now - g_busy_since) : 0u;
		unsigned permille = (wall != 0u) ? (unsigned)((g_busy_ticks * 1000u) / wall) : 0u;

		m = snprintf(out + n, (size_t)(cap - n),
			"WIFISTATS daemon busy=%u.%u%% of %llu ms, %u requests (since the previous stats)\n",
			permille / 10u, permille % 10u, (unsigned long long)(diag_ticksToUs(wall) / 1000u), (unsigned)g_busy_msgs);
		g_busy_ticks = 0u;
		g_busy_msgs = 0u;
		g_busy_since = now;
	}
	if (m < 0) {
		return n;
	}
	if ((n + m) >= cap) {
		return cap - 1;
	}
	n += m;

	m = wifi_statsBatch(out + n, cap - n);
	if (m < 0) {
		return n;
	}
	return ((n + m) < cap) ? (n + m) : (cap - 1);
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
		WIFI_ACC(g_tx_ticks, t0);
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
	/* The first read after an interrupt clears the chip's frame indication,
	 * BEFORE draining, so a frame queued meanwhile raises the line again. */
	if (__atomic_exchange_n(&g_irq.need_ack, 0, __ATOMIC_SEQ_CST) != 0) {
		wifi_irqAck(g_sdhci);
	}
	/* One probe, no lock, no wait: measured fastest. The caller paces the
	 * probes, or waits on /dev/wifiirq between them; see the notes above
	 * WIFI_DEV_TEXT_ID. */
	{
		WIFI_T0(t0);
		unsigned int skipped = 0u;
		int rc;

		for (;;) {
			rc = diag_wifiFrameRx(g_sdhci, frame, sizeof(frame), &elen);
			/* An interrupt client sleeps after a 0, until the line rises again,
			 * so 0 must mean "FIFO empty" -- not "an event frame was drained",
			 * which may have a data frame queued right behind it. */
			if ((rc != WIFI_RX_NODATA) || (g_irq.ready == 0) || (g_irq.disabled != 0) ||
				(skipped >= WIFI_RX_DRAIN_MAX)) {
				break;
			}
			skipped++;
			g_irq.drained++;
		}
		if (rc != 0) {
			WIFI_ACC(g_rx_miss_ticks, t0);
			g_rx_misses++;
			return 0; /* nothing queued, or a non-data frame was drained */
		}
		WIFI_ACC(g_rx_hit_ticks, t0);
		g_rx_hits++;
	}

	if ((size_t)elen > cap) {
		return -EMSGSIZE;
	}
	memcpy(dst, frame, elen);
	return (int)elen;
}


/* ---- Frame batches: /dev/wifibatch ------------------------------------------
 *
 * The per-frame cost of the path above is mostly not the bus: it is one IPC
 * round trip per frame through this single message thread, on top of the bus
 * time (~290 us per frame in all at 41.67 MHz, of which ~80-100 us is the bus).
 * On receive the firmware already packs ~6 frames into one glom superframe,
 * which costs one bus transfer, yet each frame still crossed to the netif in
 * its own read(), and every drain ended with an empty read. /dev/wifibatch
 * moves the same frames several per message, in the format of wifibatch.h,
 * through the same per-frame functions: the credit check, the SDPCM framing,
 * the glom walk and the event demux are exactly those of /dev/wifidata.
 *
 * A batch read holds this thread while it fills, and transmits wait behind it;
 * WIFI_BATCH_RX_FRAMES and the reader's buffer bound that. */
#define WIFI_BATCH_RX_FRAMES 32u


/* Transmit the frames of one batch, in order, each through diag_wifiFrameTx.
 * Stops at the first frame the credit window refuses: that frame and the rest
 * stay with the netif, which sends them again once the window opens (a refused
 * single write is retried by TCP the same way). A frame that fails on the bus,
 * or has an impossible length, is consumed and lost, as a failed single write
 * is. Returns the number of frames taken from the front, or -EINVAL for a
 * malformed batch. */
static int wifi_batchWrite(const void *data, size_t size)
{
	wifibatch_rd_t r;
	const uint8_t *f;
	uint32_t flen;
	int count, taken = 0, rc;

	if (g_sdhci == NULL) {
		return -EIO;
	}
	count = wifibatch_open(&r, data, size);
	if (count < 0) {
		return -EINVAL;
	}
	{
		WIFI_T0(t0);

		while ((f = wifibatch_next(&r, &flen)) != NULL) {
			if ((flen < 14u) || (flen > (F2_FRAME_MAX - 16u))) {
				g_bat.tx_bad++;
				taken++;
				continue;
			}
			rc = diag_wifiFrameTx(g_sdhci, f, flen);
			if (rc == -1070) {
				break; /* window shut: g_tx_blocked counted it */
			}
			taken++;
		}
		WIFI_ACC(g_bat.tx_ticks, t0);
	}
	g_bat.tx_msgs++;
	g_bat.tx_frames += (uint32_t)taken;
	if ((uint32_t)taken > g_bat.tx_max) {
		g_bat.tx_max = (uint32_t)taken;
	}
	if (taken < count) {
		g_bat.tx_partial++;
	}
	return taken;
}


/* Fill the reader's buffer with received frames: the rest of the superframe in
 * hand, then whatever else the FIFO holds, until it reads empty, the buffer
 * cannot take a frame of F2_FRAME_MAX bytes, or WIFI_BATCH_RX_FRAMES. Each frame
 * is written straight into the reader's buffer (one copy fewer than a
 * /dev/wifidata read). Non-data frames are read past, at most WIFI_RX_DRAIN_MAX
 * of them, in either RX mode. Returns the batch length, 0 if no frame came, or
 * <0 on error. As for /dev/wifidata, 0 means the FIFO read empty (or failed),
 * so an interrupt client may sleep on it. */
static int wifi_batchRead(void *dst, size_t cap)
{
	wifibatch_t b;
	uint8_t *slot;
	uint32_t elen = 0u, flags = 0u;
	unsigned int skipped = 0u;
	int rc;

	if (g_sdhci == NULL) {
		return -EIO;
	}
	if ((cap < (WIFIBATCH_HDR + wifibatch_recSize(F2_FRAME_MAX))) || (cap > 0x7fffffffu)) {
		return -EMSGSIZE;
	}
	/* As wifi_frameRead: ack the interrupt BEFORE draining. */
	if (__atomic_exchange_n(&g_irq.need_ack, 0, __ATOMIC_SEQ_CST) != 0) {
		wifi_irqAck(g_sdhci);
	}

	wifibatch_init(&b, dst, (uint32_t)cap);
	{
		WIFI_T0(t0);

		for (;;) {
			slot = wifibatch_slot(&b, F2_FRAME_MAX);
			if ((slot == NULL) || (b.count >= WIFI_BATCH_RX_FRAMES)) {
				g_bat.rx_capped++;
				break;
			}
			rc = diag_wifiFrameRx(g_sdhci, slot, F2_FRAME_MAX, &elen);
			if (rc == 0) {
				wifibatch_commit(&b, elen);
				continue;
			}
			if ((rc == WIFI_RX_NODATA) && (skipped < WIFI_RX_DRAIN_MAX)) {
				skipped++;
				g_bat.rx_skipped++;
				continue;
			}
			/* Empty, failed, or too many non-data frames in a row: what makes a
			 * /dev/wifidata read return 0. */
			flags = WIFIBATCH_F_DRAINED;
			g_bat.rx_drained++;
			break;
		}
		WIFI_ACC(g_bat.rx_ticks, t0);
	}

	g_bat.rx_msgs++;
	if (b.count == 0u) {
		g_bat.rx_empty++;
		return 0;
	}
	g_bat.rx_frames += b.count;
	if (b.count > g_bat.rx_max) {
		g_bat.rx_max = b.count;
	}
	return (int)wifibatch_finish(&b, flags);
}


static void wifi_thread(void *arg)
{
	uint32_t port = (uint32_t)(uintptr_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err, radio_len;
	uint64_t t_msg;

	g_busy_since = diag_ticks();
	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			break;
		}
		t_msg = diag_ticks();

		/* Text commands drive the same bus (and the same g_txf/g_rxf), so they
		 * take the lock for their whole duration. A join legitimately holds it
		 * for tens of seconds; that was equally true single-threaded. */
		if ((msg.oid.id == WIFI_DEV_DATA_ID) || (msg.oid.id == WIFI_DEV_BATCH_ID)) {
			switch (msg.type) {
				case mtOpen:
				case mtClose:
					msg.o.err = EOK;
					break;

				case mtWrite:
					msg.o.err = (msg.oid.id == WIFI_DEV_BATCH_ID) ?
						wifi_batchWrite(msg.i.data, msg.i.size) :
						wifi_frameWrite(msg.i.data, msg.i.size);
					break;

				case mtRead:
					msg.o.err = (msg.oid.id == WIFI_DEV_BATCH_ID) ?
						wifi_batchRead(msg.o.data, msg.o.size) :
						wifi_frameRead(msg.o.data, msg.o.size);
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
			g_busy_ticks += diag_ticks() - t_msg;
			g_busy_msgs++;
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
				if ((radio_len = wifi_radioCmd(msg.i.data, msg.i.size, g_resp, (int)sizeof(g_resp))) >= 0) {
					g_resp_len = (radio_len < (int)sizeof(g_resp)) ? radio_len : ((int)sizeof(g_resp) - 1);
				}
				else if (msg.i.size >= 4 && memcmp(msg.i.data, "scan", 4) == 0) {
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
				else if (msg.i.size >= 5 && memcmp(msg.i.data, "leave", 5) == 0) {
					g_resp_len = wifi_leave(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 6 && memcmp(msg.i.data, "status", 6) == 0) {
					g_resp_len = wifi_status(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 5 && memcmp(msg.i.data, "stats", 5) == 0) {
					g_resp_len = wifi_stats(g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 7 && memcmp(msg.i.data, "sdclk ", 6) == 0) {
					char arg[16];
					size_t an = msg.i.size - 6u;

					if (an >= sizeof(arg)) {
						an = sizeof(arg) - 1u;
					}
					memcpy(arg, (const char *)msg.i.data + 6, an);
					arg[an] = '\0';
					g_resp_len = wifi_sdclk(arg, g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 7 && memcmp(msg.i.data, "f2blk ", 6) == 0) {
					char arg[16];
					size_t an = msg.i.size - 6u;

					if (an >= sizeof(arg)) {
						an = sizeof(arg) - 1u;
					}
					memcpy(arg, (const char *)msg.i.data + 6, an);
					arg[an] = '\0';
					g_resp_len = wifi_f2blk(arg, g_resp, (int)sizeof(g_resp));
				}
				else if (msg.i.size >= 6 && memcmp(msg.i.data, "rxpoll", 6) == 0) {
					g_resp_len = wifi_rxPoll(g_resp, (int)sizeof(g_resp));
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

		g_busy_ticks += diag_ticks() - t_msg;
		g_busy_msgs++;
		msgRespond(port, &msg, rid);
	}
}


static void wifi_sigExit(int sig)
{
	(void)sig;
	_exit(0);
}


/* Load the firmware files. At boot (`wait` set) the root file system holding
 * them may not be mounted yet -- or be in the middle of the NFS takeover of
 * "/" -- so any failure is retried for up to WIFI_FW_WAIT_S. A failure that
 * outlasts that disables WiFi with one line saying which file. */
static int wifi_fwLoadOrWait(int wait)
{
	const char *failed = WIFI_FW_BIN;
	int err, waited = 0;

	for (;;) {
		err = wifi_fwLoad(&failed);
		if ((err == 0) || (wait == 0) || (waited >= WIFI_FW_WAIT_S)) {
			break;
		}
		sleep(1);
		waited++;
	}
	if (err != 0) {
		printf("rpi4-wifi: WiFi disabled: cannot read %s (%s)%s\n", failed, strerror(-err),
			(err == -ENOENT) ? " -- this image was built without the WiFi firmware" : "");
	}
	return err;
}


int main(int argc, char **argv)
{
	int selftest = 0, jointest = 0, fwbench = 0, foreground = 0, verbose = 0, rxirq = 1, ai, rc;
	uint32_t port, irqport;
	oid_t dev;
	pid_t pid;

	for (ai = 1; ai < argc; ++ai) {
		if (strcmp(argv[ai], "-f") == 0) {
			/* Stay in the foreground: the boot launch (user.plo.yaml), where
			 * nothing waits for the daemon to detach. */
			foreground = 1;
		}
		else if (strcmp(argv[ai], "verbose") == 0) {
			/* the full bring-up report even when the firmware starts */
			verbose = 1;
		}
		else if (strcmp(argv[ai], "selftest") == 0) {
			selftest = 1;
		}
		else if (strcmp(argv[ai], "jointest") == 0) {
			jointest = 1;
		}
		else if (strcmp(argv[ai], "fwretrytest") == 0) {
			g_fw_retry_test = 1;
		}
		else if (strcmp(argv[ai], "pollrx") == 0) {
			/* no RX interrupt: /dev/wifiirq is not created, the netif polls */
			rxirq = 0;
		}
		else if (strncmp(argv[ai], "sdclk=", 6) == 0) {
			/* the SDIO data clock: 50000 (default, 41.67 MHz) or 25000 (every
			 * earlier build); anything from 1000 to 50000 kHz is accepted */
			int khz = atoi(argv[ai] + 6);

			if ((khz >= 1000) && (khz <= (int)SDIO_CLK_HS_KHZ)) {
				g_sdclk_want_khz = (unsigned)khz;
			}
			else {
				printf("rpi4-wifi: ignoring %s (1000..%u kHz)\n", argv[ai], (unsigned)SDIO_CLK_HS_KHZ);
			}
		}
		else if (strncmp(argv[ai], "f2blk=", 6) == 0) {
			/* the function-2 block size set at bring-up: 64 (default) .. 512 */
			int sz = atoi(argv[ai] + 6);

			if (wifi_f2blkValid(sz) != 0) {
				g_f2_blksz_want = (uint32_t)sz;
			}
			else {
				printf("rpi4-wifi: ignoring %s (64, 128, 256 or 512)\n", argv[ai]);
			}
		}
		else if ((strncmp(argv[ai], "country=", 8) == 0) && (strlen(argv[ai]) == 10u) &&
				(argv[ai][8] >= 'A') && (argv[ai][8] <= 'Z') && (argv[ai][9] >= 'A') && (argv[ai][9] <= 'Z')) {
			/* set at every join (see diag_joinSettings), as `wifi country` does */
			g_country[0] = argv[ai][8];
			g_country[1] = argv[ai][9];
			g_country[2] = '\0';
		}
		else if ((strncmp(argv[ai], "bw2g=", 5) == 0) || (strncmp(argv[ai], "bw5g=", 5) == 0)) {
			/* bw_cap at every join: bw2g=20|40, bw5g=20|40|80, as `wifi bw` does */
			if (wifi_atjoinBw((argv[ai][2] == '2') ? WLC_BAND_2G : WLC_BAND_5G, atoi(argv[ai] + 5)) != 0) {
				printf("rpi4-wifi: ignoring %s (bw2g=20|40, bw5g=20|40|80)\n", argv[ai]);
			}
		}
		else if ((strcmp(argv[ai], "batch=0") == 0) || (strcmp(argv[ai], "batch=1") == 0)) {
			/* the netif's starting mode: several frames per message (/dev/wifibatch)
			 * or one (/dev/wifidata, the default); `wifi batch 0|1` at run time */
			g_bat.want = argv[ai][6] - '0';
		}
		else if (strcmp(argv[ai], "hispd=1") == 0) {
			/* keep HOST_CONTROL HISPD set above 25 MHz too (Linux clears it) */
			g_hs_hispd = 1;
		}
		else if (strcmp(argv[ai], "legacypio") == 0) {
			/* the old PIO wait, for a first-load A/B across boots */
			g_pio_legacy = 1;
		}
		else if (strcmp(argv[ai], "fwloadbench") == 0) {
			fwbench = ((ai + 1) < argc) ? atoi(argv[ai + 1]) : 20;
			if (fwbench <= 0) {
				fwbench = 20;
			}
		}
	}

	g_sdclk_try_khz = g_sdclk_want_khz; /* fwloadbench uses it as given */

	/* One owner of the SDIO bus: the daemon started at boot answers on
	 * /dev/wifi, and a second instance (or a one-shot mode) would drive the
	 * same chip underneath it. */
	rc = open("/dev/wifi", O_RDWR);
	if (rc >= 0) {
		close(rc);
		printf("rpi4-wifi: already running (/dev/wifi is served); nothing to do\n");
		return 0;
	}

	if (wifi_fwLoadOrWait(foreground) != 0) {
		return 1;
	}

	if (fwbench > 0) {
		/* One-shot, like selftest: the chip is left powered with the last load's
		 * firmware, and nothing is registered. */
		rc = wifi_fwLoadBench(fwbench);
		usleep(100 * 1000); /* let the log flush */
		return rc;
	}

	if (selftest != 0 || jointest != 0) {
		/* Single-process acceptance harness: bring up, scan once (also loads the
		 * CLM channel data + brings the radio up), print. For jointest, then
		 * exercise the association control path against a non-existent test SSID
		 * (failure events are expected; it proves the join machinery). */
		rc = wifi_bringupRetry();
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

	/* The resident daemon prints its ~16 KB bring-up report only when the
	 * firmware did not start (or with `verbose`); the one-line summary and the
	 * "firmware running" line are always printed. */
	g_bringup_quiet = (verbose == 0) ? 1 : 0;

	if (foreground == 0) {
		/* Started from the shell: fork so the shell returns once /dev/wifi is up
		 * while the child keeps serving (canonical Phoenix pattern, cf. rpi4-hci). */
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
		signal(SIGUSR1, wifi_sigExit);
		(void)setsid();
	}

	/* The daemon (or the forked child): bring up the radio, register /dev/wifi,
	 * then serve forever. */
	rc = wifi_bringupRetry();
	if (rc != 0) {
		printf("rpi4-wifi: bring-up failed (rc=%d)\n", rc);
		return rc;
	}

	/* Before the message thread exists: the setup uses the bus. */
	if (wifi_irqSetup(rxirq) == 0) {
		printf("rpi4-wifi: RX interrupt on IRQ %u (SDIO card interrupt, hostintmask=0x%02x); "
			"controller enables at boot: status=0x%08x signal=0x%08x\n",
			WIFI_IRQ, (unsigned)I_HMB_FRAME_IND,
			(unsigned)g_irq.boot_status_en, (unsigned)g_irq.boot_signal_en);
	}
	else {
		printf("rpi4-wifi: RX interrupt off (%s); the netif polls. "
			"Controller enables at boot: status=0x%08x signal=0x%08x\n",
			g_irq.why, (unsigned)g_irq.boot_status_en, (unsigned)g_irq.boot_signal_en);
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
	/* /dev/wifiirq BEFORE /dev/wifidata: the netif opens the pair as soon as
	 * /dev/wifidata appears and looks for this one only then. On any failure
	 * here it finds no device and polls; with no waiter the controller never
	 * signals, so nothing needs undoing. */
	if (g_irq.ready != 0) {
		rc = -1;
		if (portCreate(&irqport) == EOK) {
			dev.port = irqport;
			dev.id = WIFI_DEV_IRQ_ID;
			if (beginthread(wifi_irqThread, 3, g_irqStack, sizeof(g_irqStack),
					(void *)(uintptr_t)irqport) == EOK) {
				rc = create_dev(&dev, "wifiirq");
			}
		}
		if (rc < 0) {
			g_irq.disabled = 1;
			(void)snprintf(g_irq.why, sizeof(g_irq.why), "no /dev/wifiirq");
			printf("rpi4-wifi: WARNING could not serve /dev/wifiirq; the netif polls\n");
		}
	}
	dev.port = port;
	dev.id = WIFI_DEV_DATA_ID;
	if (create_dev(&dev, "wifidata") < 0) {
		/* Not fatal: the text device still works, only the netif seam is gone. */
		printf("rpi4-wifi: WARNING could not create /dev/wifidata (frame seam unavailable)\n");
	}
	dev.id = WIFI_DEV_BATCH_ID;
	if (create_dev(&dev, "wifibatch") < 0) {
		/* Not fatal either: the netif stays on /dev/wifidata. */
		printf("rpi4-wifi: WARNING could not create /dev/wifibatch (one frame per message only)\n");
	}
	printf("rpi4-wifi: registered /dev/wifi (write \"scan\", then read the AP list)\n");
	fflush(stdout);

	if (beginthread(wifi_thread, 3, g_msgStack, sizeof(g_msgStack),
			(void *)(uintptr_t)port) != EOK) {
		printf("rpi4-wifi: msg thread failed\n");
		return 5;
	}

	if (foreground == 0) {
		kill(getppid(), SIGUSR1);
	}
	for (;;) {
		usleep(1000 * 1000); /* the message thread does the work */
	}
	return 0;
}
