/*
 * Phoenix-RTOS
 *
 * SD memory card driver
 * Compatible with SD Specifications Part A2: SD Host Controller Simplified Specification Version 2.00
 *
 * Copyright 2023 Phoenix Systems
 * Author: Ziemowit Leszczynski, Artur Miller, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "sdcard.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "sdhost_defs.h"
/* TODO: sdcard.c/sdhost_defs.h/sdstorage_*.c are copied verbatim from
 * storage/zynq7000-sdcard (generic SDHCI core). De-duplicate into a shared lib
 * (neutralize this platform include) before upstreaming. */
#include "bcm2711-sdio.h"

#define LOG_TAG "sdcard"
/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, LOG_TAG " error: " str "\n", ##__VA_ARGS__); } while (0)
#define TRACE(str, ...)     do { if (0) fprintf(stderr, LOG_TAG " trace: " str "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */

/* TODO(#154 diag): one-shot write/read self-test at end of card bring-up.
 * Root cause is now FOUND (writes land but Transfer-Complete IRQ never fires for
 * PIO writes; see docs/inprogress/2026-06-07-sd-write-completion-rootcause.md).
 * The diagnostic code below is gated on this macro being DEFINED (#ifdef). It is
 * intentionally left UNDEFINED so the diag compiles out (clean netboot/other
 * builds); re-add `#define SDCARD_DIAG_CLOCKSWEEP 1` to validate the CMD13-poll
 * write-completion fix. The reset-on-timeout fixes below are NOT gated and stay
 * active. The self-test (write+readback + large consecutive read) was HW-validated
 * 2026-06-30: writeRc=16/16, large read 2048/2048 (no EIO on a good card). Re-add
 * `#define SDCARD_DIAG_CLOCKSWEEP 1` to re-run it (e.g. to triage a marginal card). */
#define SDCARD_ENABLE_DMA 1   /* SDMA read data path (HW-validated; reads only — writes use PIO, see the write path) */
#define SDCARD_ENABLE_DDR50 1 /* UHS-I DDR50 1.8V (HW-validated ~1.6x read on netboot + SD-boot; HS50 fallback) */


/* #154: bound on the CMD13 SEND_STATUS busy-poll that detects write completion
 * (mirrors Linux MMC_BLK_TIMEOUT_MS). The card normally returns to TRAN in well
 * under a millisecond on these small single-block writes. */
#define SDCARD_WRITE_BUSY_TIMEOUT_MS 1000u

#define SDHOST_ERROR_REASONS ( \
	SDHOST_INTR_CMD_ERRORS | \
	SDHOST_INTR_DAT_ERRORS | \
	SDHOST_INTR_OVERCURRENT_ERROR | \
	SDHOST_INTR_AUTO_CMD12_ERROR | \
	SDHOST_INTR_ADMA_ERROR | \
	SDHOST_INTR_DMA_ERROR)

#define SDHOST_STATUS_MASK ( \
	SDHOST_INTR_CMD_STATUS | \
	SDHOST_INTR_CARD_IN | \
	SDHOST_INTR_CARD_OUT | \
	SDHOST_ERROR_REASONS)

#define AWAITABLE_INTRS ( \
	SDHOST_INTR_CMD_DONE | \
	SDHOST_INTR_TRANSFER_DONE | \
	SDHOST_INTR_BLOCK_GAP | \
	SDHOST_INTR_CARD_OUT | \
	SDHOST_ERROR_REASONS)

/* configuration values */
#define THREAD_STACK_SIZE 1024
#define SDHOST_RETRIES    10

/* Number of blocks to erase in a single transaction
 * Number is limited to not trigger timeouts
 */
#define ERASE_N_BLOCKS 128

#define SD_FREQ_INITIAL 400000   /* 400 kHz clock used for card initialization */
#define SD_FREQ_25M     25000000 /* 25 MHz clock usable when card is initialized */
#define SD_FREQ_50M     50000000 /* 50 MHz clock usable when card is initialized and supports high speed */

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

typedef struct {
	/* Relative Card Address of the card. Needed for selecting the card so it can use the data bus. */
	uint32_t rca;
	/* Size of SD card in blocks */
	uint32_t sizeBlocks;
	/* Size (in blocks) of an erase sector */
	uint32_t eraseSizeBlocks;
	/* Number of command timeouts since last card initialization */
	uint32_t commandTimeouts;
	/* Whether the card supports SDHC protocol (requires slightly different handling) */
	bool highCapacity;
	/* Whether the card+host completed the 1.8V UHS-I signaling switch at init (enables DDR50). */
	bool uhs;
} sdcard_cardMetadata_t;

typedef struct {
	/* Base address (in virtual memory) of SD Host Controller register set */
	volatile uint32_t *base;
	sdcard_cardMetadata_t card;
	/* Reference clock frequency in Hz */
	uint32_t refclkFrequency;

	/* Address of DMA buffer in virtual memory (for access by the CPU) */
	void *dmaBuffer;
	/* Address of DMA buffer in physical memory (for access by the SD Host Controller) */
	addr_t dmaBufferPhys;
	/* True once the staging buffer is confirmed DMA-reachable (< 1 GiB CPU-phys, the
	 * BCM2711 emmc2bus dma-ranges window): enables the SDMA data path instead of the
	 * CPU PIO FIFO loop. Set in sdhost_allocDMA. */
	bool useDma;

	bool sdioInitialized;
	bool isCDPinSupported;
	bool isWPPinSupported;

	handle_t cmdLock;
	handle_t isrHandle;
	handle_t eventLock;
	handle_t eventCond;
} sdcard_hostData_t;

static sdcard_hostData_t sdio_hosts[PLATFORM_SDIO_N_HOSTS] = { 0 };
static unsigned int initializedHosts = 0;

static struct {
	handle_t lock;
	handle_t cond;
	bool initialized;
} presenceEvents = { .initialized = false };

static inline sdcard_hostData_t *sdcard_getHostForSlot(unsigned int slot)
{
	return (slot >= PLATFORM_SDIO_N_HOSTS) ? NULL : &sdio_hosts[slot];
}


static int sdcard_configClockAndPower(sdcard_hostData_t *host, uint32_t freq);
static int sdcard_wideAndFast(sdcard_hostData_t *host);


/* Resets parts or all of the host according to the given reset type.
 * Argument must be one of CLOCK_CONTROL_RESET_*
 */
static int sdhost_reset(sdcard_hostData_t *host, uint32_t resetType)
{
	*(host->base + SDHOST_REG_CLOCK_CONTROL) |= resetType;
	sdio_dataBarrier();
	for (int i = 0; i < SDHOST_RETRIES; ++i) {
		usleep(10);
		if ((*(host->base + SDHOST_REG_CLOCK_CONTROL) & resetType) == 0) {
			return 0;
		}
	}

	return -1;
}


static inline bool sdcard_isWriteProtected(sdcard_hostData_t *host)
{
	if (host->isWPPinSupported) {
		if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_WRITE_PROT_PIN) == 0) {
			return true;
		}
	}

	return false;
}


static int sdhost_allocDMA(sdcard_hostData_t *host)
{
	/* The staging buffer must be physically contiguous for the SDMA_ADDRESS
	 * register (the SDMA read path programs it directly). Cap at 128 KiB
	 * (256 blocks) so multi-block CMD18/CMD25 transfers can be large. */
	if (SDCARD_MAX_TRANSFER > (32 * _PAGE_SIZE)) {
		return -ENOMEM;
	}

	void *p = mmap(NULL, SDCARD_MAX_TRANSFER, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_UNCACHED | MAP_CONTIGUOUS, -1, 0);
	if (p == MAP_FAILED) {
		return -ENOMEM;
	}

	host->dmaBuffer = p;
	if (host->dmaBuffer == NULL) {
		return -ENOMEM;
	}

	host->dmaBufferPhys = va2pa(host->dmaBuffer);
	/* The EMMC2 DMA master can only reach the low 1 GiB of CPU-physical memory (Pi 4
	 * emmc2bus dma-ranges: bus 0xC0000000.. -> CPU 0x0..0x3FFFFFFF). Enable the SDMA
	 * data path only when the staging buffer actually landed there; otherwise fall
	 * back to PIO (which has no addressing limit). The MAP_CONTIGUOUS allocation for
	 * this small early-boot buffer reliably lands low (observed ~0x03780000).
	 *
	 * SDCARD_ENABLE_DMA is defined: SDMA is the read data path (reads only — writes
	 * still use PIO, see the write path). useDma additionally requires the staging
	 * buffer to be DMA-reachable (< 1 GiB), else this read falls back to PIO. */
#ifdef SDCARD_ENABLE_DMA
	host->useDma = (host->dmaBufferPhys < 0x40000000ul);
#else
	host->useDma = false;
#endif
	return 0;
}


static int sdhost_isr(unsigned int n, void *arg)
{
	/* SD Host controller interrupts are triggered by level, not by edge.
	 * This means that ISR has to remove the reason for interrupt (zero out bit in STATUS register)
	 * or disable the interrupt (zero out bit in SIGNAL_ENABLE register) - otherwise we get stuck
	 * in infinite interrupt loop.
	 */
	sdcard_hostData_t *host = (sdcard_hostData_t *)arg;
	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
	return 0;
}


static int _sdio_cmdExecutionWait(sdcard_hostData_t *host, uint32_t flags, time_t wait_us)
{
	int ret = -ETIME;
	bool doResetCmd = false, doResetDat = false;

	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = AWAITABLE_INTRS;
	/* Lost-wakeup guard (#119): the command is written to SDHOST_REG_CMD before
	 * this wait runs, so its completion interrupt can fire and latch its bit in
	 * INTR_STATUS before we arm the cond here. Empirically (CMD7 R1b: INTR_STATUS
	 * read 0x3 = CMD_DONE|TRANSFER_DONE at the poll point) re-enabling
	 * SIGNAL_ENABLE does NOT re-trigger on an already-high level on this
	 * controller, so condWait would block to timeout despite the result being
	 * ready. Check the latched status first and skip the sleep if the awaited
	 * flags (or an error) are already present. */
	int waitret = 0;
	uint32_t pending = *(host->base + SDHOST_REG_INTR_STATUS);
	if (((pending & flags) != flags) && ((pending & SDHOST_ERROR_REASONS) == 0u)) {
		/* If there is a pending interrupt from flagsWithErrors set, it will fire now */
		waitret = condWait(host->eventCond, host->eventLock, wait_us);
	}
	if (waitret == -ETIME) {
		/* It's a bit odd, but card ejection doesn't cause any errors - only timeouts
		 * We may get a SDHOST_INTR_CARD_OUT, but only if physical card detect is available
		 */
		*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
		host->card.commandTimeouts++;
	}

	uint32_t val = *(host->base + SDHOST_REG_INTR_STATUS);
	if ((val & SDHOST_ERROR_REASONS) != 0) {
		uint32_t errBits = val & SDHOST_ERROR_REASONS;
		/* A pure command/data TIMEOUT means the card did not respond in time —
		 * expected when probing an absent or not-yet-ready card (e.g. ACMD41 on
		 * an empty slot, which the EMMC2 slot can't detect electrically). Report
		 * it as -ETIME and keep it quiet; reserve the loud error log + -EIO for
		 * genuine bus errors (CRC / end-bit / index / data corruption). */
		if ((errBits & ~(uint32_t)(SDHOST_INTR_CMD_TIMEOUT | SDHOST_INTR_DATA_TIMEOUT)) == 0u) {
			TRACE("cmd timeout: intr_status=0x%08x", (unsigned)val);
			ret = -ETIME;
		}
		else {
			LOG_ERROR("cmd error: intr_status=0x%08x err=0x%08x caps=0x%08x", (unsigned)val,
				(unsigned)errBits, (unsigned)*(host->base + SDHOST_REG_CAPABILITIES));
			ret = -EIO;
		}
		doResetCmd = (val & SDHOST_INTR_CMD_ERRORS) != 0;
		doResetDat = (val & SDHOST_INTR_DAT_ERRORS) != 0;
		/* TODO(#120): a genuine bus error (CRC / end-bit / index / data) can wedge
		 * BOTH the command and data lines; reset both so the next command (e.g. a
		 * single-block read-retry) is not rejected with -EBUSY by a stuck inhibit. */
		if (ret == -EIO) {
			doResetCmd = true;
			doResetDat = true;
		}
		*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
		/* Under QEMU it is important to zero out this register if an incomplete transfer happens */
		*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
	}
	else if ((val & flags) == flags) {
		*(host->base + SDHOST_REG_INTR_STATUS) = flags;
		ret = EOK;
	}

	if ((val & SDHOST_INTR_BLOCK_GAP) != 0) {
		/* Not strictly an error, but should not happen in the current implementation */
		*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_BLOCK_GAP;
		doResetDat = true;
		ret = -EIO;
	}

	if (doResetCmd || (ret == -ETIME)) {
		sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
	}

	if (doResetDat || (ret == -ETIME)) {
		sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
	}

	if ((val & SDHOST_INTR_CARD_OUT) != 0) {
		condSignal(presenceEvents.cond);
	}

	return ret;
}


/* #154: issue a single raw CMD13 SEND_STATUS and return the card's R1 in *st.
 *
 * Must run with host->eventLock HELD (it uses _sdio_cmdExecutionWait, which
 * condWaits on eventLock). It pokes the command registers directly rather than
 * recursing through sdio_cmdSend()/_sdio_cmdSend() because:
 *  - the caller already holds host->cmdLock (sdcard_transferBlocks) AND
 *    host->eventLock, so a re-entrant high-level send would self-deadlock; and
 *  - the high-level path runs the pre-command PRES_STATE_BUSY_FLAGS busy poll,
 *    which would bail -EBUSY while the card holds DAT0 busy after a write —
 *    exactly when we need to interrogate it. CMD13 is CMD-line-only and is legal
 *    while the card is programming, so it bypasses that poll here.
 * Returns 0 (R1 in *st) or <0 if the CMD13 itself failed (timeout / bus error). */
static int _sdio_rawSendStatus(sdcard_hostData_t *host, uint32_t *st)
{
	sdhost_command_reg_t cmdFrame;
	cmdFrame.raw = 0;
	cmdFrame.commandIdx = SDIO_CMD13_SEND_STATUS;
	cmdFrame.commandMeta = sdCmdMetadata[SDIO_CMD13_SEND_STATUS].bitsWhenSending;

	/* No data phase for CMD13. */
	*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
	/* Drop any stale completion bits so the upcoming wait sees only this CMD. */
	*(host->base + SDHOST_REG_INTR_STATUS) = (SDHOST_INTR_TRANSFER_DONE | SDHOST_INTR_CMD_DONE);
	*(host->base + SDHOST_REG_CMD_ARGUMENT) = host->card.rca;
	sdio_dataBarrier();
	/* This register write starts command execution and must be done last. */
	*(host->base + SDHOST_REG_CMD) = cmdFrame.raw;

	int ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_CMD_DONE, 50 * 1000);
	if (ret < 0) {
		return ret;
	}

	if (st != NULL) {
		*st = *(host->base + SDHOST_REG_RESPONSE_0);
	}

	return 0;
}


/* Issue a single CMD23-SET_BLOCK_COUNT (R1, no data, no busy) ahead of a
 * multi-block CMD18/CMD25 — the Auto-CMD23 alternative to Auto-CMD12.
 *
 * On the BCM2711/Arasan EMMC2 controller the Transfer-Complete IRQ never latches
 * for any data transfer that uses Auto-CMD12 (the CMD12 STOP) — proven on HW for
 * multi-block reads, which sit idle in TRAN with no busy phase yet INTR_STATUS
 * stays 0x0. CMD23 bounds the transfer up front, so the data command needs no
 * trailing Auto-CMD12 STOP at all; the multi-block transfer then looks like a
 * plain CMD17/CMD24 to the completion logic. (`SDHCI_QUIRK2_ACMD23_BROKEN` is NOT
 * set for brcm,bcm2711-emmc2 in Linux, i.e. CMD23 is considered usable here.)
 *
 * The host command register exposes only `autoCmd12Enable` (bit 2), not the
 * full 2-bit SDHCI Auto-CMD-Enable field, so hardware Auto-CMD23 is not cleanly
 * available — we issue CMD23 manually (also the right choice on the PIO path,
 * where hardware Auto-CMD23 would be DMA-coupled).
 *
 * MUST be issued BEFORE the data command's CMD_ARGUMENT (LBA) and TRANSFER_BLOCK
 * are programmed: this helper writes those very registers, so calling it after
 * them would clobber the data command's setup. Runs with host->cmdLock held (the
 * caller holds it) and takes host->eventLock itself for the duration of the
 * CMD_DONE wait. `count` is the block COUNT (not bytes). Returns 0 / <0. */
static int _sdio_setBlockCount(sdcard_hostData_t *host, uint16_t count)
{
	sdhost_command_reg_t cmdFrame;
	cmdFrame.raw = 0;
	cmdFrame.commandIdx = SDIO_CMD23_SET_BLOCK_COUNT;
	cmdFrame.commandMeta = sdCmdMetadata[SDIO_CMD23_SET_BLOCK_COUNT].bitsWhenSending;

	/* No data phase for CMD23. */
	*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
	/* Drop any stale completion bits so the upcoming wait sees only this CMD. */
	*(host->base + SDHOST_REG_INTR_STATUS) = (SDHOST_INTR_TRANSFER_DONE | SDHOST_INTR_CMD_DONE);
	*(host->base + SDHOST_REG_CMD_ARGUMENT) = (uint32_t)count;
	sdio_dataBarrier();
	mutexLock(host->eventLock);
	/* This register write starts command execution and must be done last. */
	*(host->base + SDHOST_REG_CMD) = cmdFrame.raw;

	int ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_CMD_DONE, 50 * 1000);
	mutexUnlock(host->eventLock);

#ifdef SDCARD_DIAG_CLOCKSWEEP
	/* One-shot marker so the multi-block validation log shows the Auto-CMD23 path
	 * is live (and whether the first CMD23 itself succeeded). */
	static bool sbcLogged = false;
	if (!sbcLogged) {
		sbcLogged = true;
		printf("SDDIAG-MB: CMD23-SET_BLOCK_COUNT active (Auto-CMD12 disabled), first count=%u ret=%d\n",
			(unsigned)count, ret);
	}
#endif

	return ret;
}


/* #154: poll CMD13 SEND_STATUS until the card reports it is back in the transfer
 * state and ready for data (READY_FOR_DATA & CURRENT_STATE==TRAN) — the
 * authoritative write-completion check Linux's MMC core performs for writes
 * (mmc_blk_card_busy -> __mmc_poll_for_busy -> mmc_ready_for_data). We use it
 * because this controller's Transfer-Complete IRQ never latches for PIO writes
 * and its present-state busy bits are internally inconsistent on writes.
 *
 * Must run with host->eventLock HELD. Returns 0 when ready, -EIO if the card R1
 * flags an error, -ETIME on timeout, or the CMD13 send error. Uses the same
 * exponential backoff as the Linux poll. */
static int _sdio_pollCardReady(sdcard_hostData_t *host, unsigned int timeoutMs)
{
	unsigned int us = 32u, total = 0u;
	const unsigned int totalMax = timeoutMs * 1000u;

	for (;;) {
		uint32_t st = 0;
		int ret = _sdio_rawSendStatus(host, &st);
		if (ret < 0) {
			return ret;
		}

		if ((st & CARD_STATUS_ERRORS) != 0u) {
			LOG_ERROR("write completion: card R1 error 0x%08x", (unsigned)st);
			return -EIO;
		}

		if (((st & CARD_STATUS_READY_FOR_DATA) != 0u) &&
				(CARD_STATUS_CURRENT_STATE(st) == CARD_STATUS_CURRENT_STATE_TRAN)) {
			return 0;
		}

		if (total >= totalMax) {
			return -ETIME;
		}

		usleep(us);
		total += us;
		if (us < 32768u) {
			us *= 2u;
		}
	}
}


/* #154 large-read EIO diagnostic. On a read data-phase failure, the host
 * INTR_STATUS data-end-bit and the card's own CMD13 R1 state are independent
 * facts that classify the failure: if the card is in TRAN + READY_FOR_DATA the
 * data left the card cleanly and the host mis-sampled it (HS50 margin → the fix
 * is to drop the clock); if the card is stuck off-TRAN it is wedged (→ re-init).
 * This is the discriminator §4.1 of the research doc needs from HW. Default-on
 * (NOT gated by SDCARD_DIAG_CLOCKSWEEP) but capped at the first few occurrences
 * so a sustained read failure can't flood the console. */
static unsigned int sdcard_readDiagCount = 0;
enum { SDCARD_READDIAG_MAX = 3u };

static void sdcard_readDiagPrint(uint8_t cmd, uint32_t intr, int cmd13rc, uint32_t r1)
{
	printf("SDREADDIAG: cmd=%u intr=0x%08x cmd13rc=%d r1=0x%08x state=%u\n",
		(unsigned)cmd, (unsigned)intr, cmd13rc, (unsigned)r1,
		(unsigned)CARD_STATUS_CURRENT_STATE(r1));
}


/* Host-level read failure (_sdio_cmdSend returned <0): issue a fresh raw CMD13
 * to learn the card's current state. Must be called with host->eventLock HELD
 * (the raw CMD13 condWaits on eventLock). Does not change read behavior. */
static void sdcard_readDiag(sdcard_hostData_t *host, uint8_t cmd, uint32_t intr)
{
	if (sdcard_readDiagCount >= SDCARD_READDIAG_MAX) {
		return;
	}
	sdcard_readDiagCount++;

	uint32_t st = 0;
	int cmd13rc = _sdio_rawSendStatus(host, &st);
	sdcard_readDiagPrint(cmd, intr, cmd13rc, st);
}


/* Card-flagged read failure (_sdio_cmdSend returned >=0 but the read command's
 * R1 carries error bits — this is the field "transfer error 00400900" path).
 * The R1 is already in hand, so no fresh CMD13 is issued (cmdLock is held by the
 * caller; a high-level CMD13 here would self-deadlock). Logs the same line with
 * cmd13rc=0 to mark "R1 from the failing command itself". */
static void sdcard_readDiagR1(uint8_t cmd, uint32_t intr, uint32_t r1)
{
	if (sdcard_readDiagCount >= SDCARD_READDIAG_MAX) {
		return;
	}
	sdcard_readDiagCount++;

	sdcard_readDiagPrint(cmd, intr, 0, r1);
}


/* res is single u32 if isLongResponse == 0, array of 4 u32 otherwise */
/* Completion path for R1b (response-with-busy, no data) commands such as CMD7
 * SELECT_CARD / CMD38 ERASE. On the BCM2711/Arasan EMMC2 controller the
 * Command-Complete interrupt is NOT reliably delivered for a check-busy command
 * (observed #119: CMD7 completes — Command-Inhibit-CMD clears — yet CMD_DONE
 * never fires, so the IRQ-driven wait times out). The Present State register
 * reflects the truth, so poll it instead: wait for the response (Command-
 * Inhibit-CMD clear) then for busy release (DAT0 / Command-Inhibit-DAT clear),
 * watching the error-status bits. Data-transfer commands keep the IRQ path. */
static int _sdio_pollBusyCmd(sdcard_hostData_t *host, uint8_t cmd)
{
	enum { cmdPollUs = 10u, cmdPollMax = 2000u, busyPollMax = 100000u }; /* ~20 ms cmd, ~1 s busy */
	unsigned int i;
	uint32_t st;

	/* No interrupts during the poll: completion bits stay latched in
	 * INTR_STATUS regardless of SIGNAL_ENABLE, and we read PRES_STATE directly. */
	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;

	for (i = 0; i < cmdPollMax; i++) {
		st = *(host->base + SDHOST_REG_INTR_STATUS);
		if ((st & SDHOST_ERROR_REASONS) != 0) {
			LOG_ERROR("R1b cmd %u error (response phase): intr=0x%08x", (unsigned)cmd, (unsigned)st);
			*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
			sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
			sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
			return -EIO;
		}
		if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_CMD_BUSY) == 0u) {
			break;
		}
		usleep(cmdPollUs);
	}
	if (i == cmdPollMax) {
		TRACE("R1b cmd %u: command-inhibit stuck (pres=0x%08x)", (unsigned)cmd,
			(unsigned)*(host->base + SDHOST_REG_PRES_STATE));
		sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
		return -ETIME;
	}

	/* Diagnostic (#119): did the controller raise CMD_DONE / TRANSFER_DONE for
	 * this R1b after the response latched? */
	TRACE("R1b cmd %u response done: intr=0x%08x", (unsigned)cmd,
		(unsigned)*(host->base + SDHOST_REG_INTR_STATUS));

	for (i = 0; i < busyPollMax; i++) {
		st = *(host->base + SDHOST_REG_INTR_STATUS);
		if ((st & SDHOST_ERROR_REASONS) != 0) {
			LOG_ERROR("R1b cmd %u error (busy phase): intr=0x%08x", (unsigned)cmd, (unsigned)st);
			*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
			sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
			return -EIO;
		}
		if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_DAT_BUSY) == 0u) {
			break;
		}
		usleep(cmdPollUs);
	}
	if (i == busyPollMax) {
		TRACE("R1b cmd %u: DAT0 busy stuck (pres=0x%08x)", (unsigned)cmd,
			(unsigned)*(host->base + SDHOST_REG_PRES_STATE));
		sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
		return -ETIME;
	}

	/* Drop any latched completion bits so they don't bleed into the next
	 * command's IRQ-driven wait. */
	*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_CMD_DONE | SDHOST_INTR_TRANSFER_DONE;
	return 0;
}


/* dataBuf: the buffer the PIO data phase reads from (write) or writes to (read).
 * Callers on the throughput path pass their own (cached) buffer so the FIFO loop
 * moves data directly with no intermediate copy; passing NULL falls back to the
 * uncached staging buffer (used only by the small register-read path). */
static int _sdio_cmdSend(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res, uint16_t blockCount, bool isLongResponse, void *dataBuf, bool useDma)
{
	sdhost_command_reg_t cmdFrame;

	if (cmd >= MAX_SD_COMMANDS) {
		return -EINVAL;
	}

	sdhost_command_data_t dataType = sdCmdMetadata[cmd].dataType;
	if (dataType == CMD_INVALID) {
		return -EINVAL;
	}

	/* Wait (bounded, ~100 ms) for the controller/card to leave the busy state
	 * before issuing a new command. This previously returned -EBUSY immediately,
	 * which defeated single-block read-retry after a data error (#120): the
	 * post-error card holds DAT0 busy briefly, so every retry insta-failed with
	 * -EBUSY. The fast path (not busy) takes the first iteration with no sleep.
	 *
	 * #154: CMD13 SEND_STATUS is explicitly legal while the card is programming
	 * (DAT0 busy) and is exactly how a caller interrogates a still-busy card, so
	 * it must skip this busy-wait — otherwise a status poll during a write's PRG
	 * phase would bail -EBUSY before it could ever report completion. */
	if (cmd != SDIO_CMD13_SEND_STATUS) {
		unsigned int i;
		for (i = 0; i < 1000u; i++) {
			if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_BUSY_FLAGS) == 0u) {
				break;
			}
			usleep(100);
		}
		uint32_t val = *(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_BUSY_FLAGS;
		if (val != 0) {
			TRACE("busy %x", val);
			return -EBUSY;
		}
	}

	/* Clear any stale command/transfer-complete bits left by a prior op rather
	 * than bailing with -EBUSY -- a leftover DONE bit must not block the new
	 * command (the upcoming wait consumes the fresh completion). */
	*(host->base + SDHOST_REG_INTR_STATUS) = (SDHOST_INTR_TRANSFER_DONE | SDHOST_INTR_CMD_DONE);

	cmdFrame.commandIdx = cmd;
	cmdFrame.commandMeta = sdCmdMetadata[cmd].bitsWhenSending;

	if ((dataType == CMD_NO_DATA) || (dataType == CMD_NO_DATA_WAIT)) {
		*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
	}
	else {
		/* Multi-block (CMD18/CMD25): set the block count with CMD23
		 * SET_BLOCK_COUNT up front instead of arming Auto-CMD12. The Auto-CMD12
		 * STOP suppresses this controller's Transfer-Complete IRQ; CMD23 removes
		 * it so the multi-block transfer completes like a single block. CMD23
		 * MUST be issued here, before the data command's TRANSFER_BLOCK /
		 * CMD_ARGUMENT below — _sdio_setBlockCount writes those same registers,
		 * which are then re-populated for the data command. */
		if (((dataType == CMD_READ_MULTI) || (dataType == CMD_WRITE_MULTI)) && (blockCount != 0)) {
			int sbcRet = _sdio_setBlockCount(host, blockCount);
			if (sbcRet < 0) {
				TRACE("error %d on CMD23 SET_BLOCK_COUNT (cmd %d)", sbcRet, cmd);
				return sbcRet;
			}
		}

		if (blockCount != 0) {
			uint32_t blockLength;
			switch (dataType) {
				case CMD_READ8:
					blockLength = 8;
					break;

				case CMD_READ64:
					blockLength = 64;
					break;

				default:
					blockLength = SDCARD_BLOCKLEN;
					break;
			}

			*(host->base + SDHOST_REG_SDMA_ADDRESS) = host->dmaBufferPhys;
			sdio_dataBarrier();
			/* SDMA boundary = 512K (the max): our staging buffer is at most
			 * SDCARD_MAX_TRANSFER (64K), so the transfer never crosses a boundary
			 * and the engine never raises the DMA-boundary interrupt (which the
			 * driver does not service). A too-small boundary (the old 4K) is what
			 * stalled SDMA after the first page — misdiagnosed as a reach limit in
			 * #120; the buffer is in fact DMA-reachable (see sdhost_allocDMA). */
			*(host->base + SDHOST_REG_TRANSFER_BLOCK) =
				((uint32_t)blockCount << 16) |
				TRANSFER_BLOCK_SDMA_BOUNDARY_512K |
				blockLength;
		}

		cmdFrame.dataPresent = 1;
		/* SDMA when the staging buffer is DMA-reachable (useDma), else PIO over the
		 * BUFFER_DATA FIFO. SDMA offloads the byte movement from the CPU; the data
		 * lands in the (uncached, DMA-coherent) staging buffer. HOST_CONTROL DMA
		 * select defaults to SDMA (00b), so dmaEnable alone selects the SDMA path. */
		cmdFrame.dmaEnable = useDma ? 1 : 0;
		cmdFrame.blockCountEnable = 1;
		if ((dataType == CMD_READ) || (dataType == CMD_READ_MULTI) || (dataType == CMD_READ8) || (dataType == CMD_READ64)) {
			cmdFrame.directionRead = 1;
		}

		if ((dataType == CMD_READ_MULTI) || (dataType == CMD_WRITE_MULTI)) {
			cmdFrame.multiBlock = 1;
			/* Auto-CMD12 OFF: block count is bounded by the CMD23 above, so the
			 * controller must NOT append a CMD12 STOP. The trailing R1b-busy CMD12
			 * STOP is what suppresses the Transfer-Complete IRQ on this controller. */
			cmdFrame.autoCmd12Enable = 0;
		}
	}

	*(host->base + SDHOST_REG_CMD_ARGUMENT) = arg;
	sdio_dataBarrier();
	mutexLock(host->eventLock);
	/* This register write starts command execution and must be done last */
	*(host->base + SDHOST_REG_CMD) = cmdFrame.raw;

	int ret;
	if (dataType == CMD_NO_DATA_WAIT) {
		/* R1b (response-with-busy, no data): poll Present State for completion
		 * — the Command-Complete IRQ is not reliably raised for check-busy
		 * commands on this controller (#119). See _sdio_pollBusyCmd. */
		ret = _sdio_pollBusyCmd(host, cmd);
		if (ret < 0) {
			TRACE("error %d on cmd %d (R1b poll)", ret, cmd);
			mutexUnlock(host->eventLock);
			return ret;
		}
	}
	else {
		/* wait 1 ms max for the command response */
		ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_CMD_DONE, 1000);
		if (ret < 0) {
			TRACE("error %d on cmd %d (cmd_done wait, pres=0x%08x)", ret, cmd,
				(unsigned)*(host->base + SDHOST_REG_PRES_STATE));
#ifdef SDCARD_DIAG_CLOCKSWEEP
			printf("SDDIAG: cmd_done-timeout cmd=%u ret=%d pres=0x%08x intr=0x%08x\n", (unsigned)cmd, ret,
				(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#endif
			mutexUnlock(host->eventLock);
			/* #154: a timed-out command leaves the CMD/DAT engine wedged; without
			 * recovery every subsequent command also times out (a cascade that
			 * e.g. turns one failed write into a failed MBR read). Mirror the
			 * PIO-error recovery below. */
			sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
			sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
			return ret;
		}

		if (dataType != CMD_NO_DATA) {
			/* PIO data transfer over the BUFFER_DATA FIFO (SDMA is unusable here —
			 * see the dmaEnable=0 note above). pioBuf is the caller's own buffer on
			 * the throughput path (no staging copy), or the uncached staging buffer
			 * for the small register-read path (dataBuf == NULL).
			 *
			 * Gate each block's drain on the PRES_STATE Buffer-Read/Write-Enable
			 * LEVEL bit (the Linux sdhci_transfer_pio pattern), NOT the latched
			 * RW_READ_READY interrupt + per-block write-1-clear. The latched-bit
			 * approach raced the FIFO at 50 MHz high-speed and produced a
			 * Data-CRC / End-Bit error on ~every block (#120). The level bit
			 * reflects the true buffer state, so we only touch the FIFO when the
			 * controller says a block is actually available. */
			bool pioRead = (dataType == CMD_READ) || (dataType == CMD_READ_MULTI) || (dataType == CMD_READ8) || (dataType == CMD_READ64);

			/* PIO byte-movement only when DMA is not in use. With useDma the SDMA
			 * engine has already moved (read) / will move (write) the data to/from
			 * the staging buffer; we skip straight to the completion wait below. */
			if (!useDma) {
			uint32_t pioBlockLen = (dataType == CMD_READ8) ? 8u : ((dataType == CMD_READ64) ? 64u : SDCARD_BLOCKLEN);
			uint32_t pioWords = pioBlockLen / 4u;
			uint32_t pioReadyLevel = pioRead ? PRES_STATE_BUFFER_READ_ENABLE : PRES_STATE_BUFFER_WRITE_ENABLE;
			volatile uint32_t *pioFifo = host->base + SDHOST_REG_BUFFER_DATA;
			uint8_t *pioBuf = (uint8_t *)((dataBuf != NULL) ? dataBuf : host->dmaBuffer);
			int pioErr = 0;
			uint32_t pioErrIntr = 0; /* INTR_STATUS at the offending block (#154 read diag) */

			for (uint32_t pioBlk = 0; pioBlk < (uint32_t)blockCount; pioBlk++) {
				uint32_t st = 0;
				long spin;
				for (spin = 0; spin < 2000000; spin++) {
					st = *(host->base + SDHOST_REG_INTR_STATUS);
					if ((st & SDHOST_ERROR_REASONS) != 0u) {
						break;
					}
					if ((*(host->base + SDHOST_REG_PRES_STATE) & pioReadyLevel) != 0u) {
						break;
					}
				}
				if ((st & SDHOST_ERROR_REASONS) != 0u) {
					LOG_ERROR("pio xfer error on cmd %d: intr=0x%08x", cmd, (unsigned)st);
					pioErrIntr = st;
					*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
					pioErr = -EIO;
					break;
				}
				if ((*(host->base + SDHOST_REG_PRES_STATE) & pioReadyLevel) == 0u) {
#ifdef SDCARD_DIAG_CLOCKSWEEP
					printf("SDDIAG: pio-timeout cmd=%u blk=%u dir=%s readyLvl=0x%08x pres=0x%08x intr=0x%08x\n",
						(unsigned)cmd, (unsigned)pioBlk, pioRead ? "rd" : "wr", (unsigned)pioReadyLevel,
						(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#endif
					pioErr = -ETIME;
					break;
				}
				uint32_t *pioW = (uint32_t *)(pioBuf + (size_t)pioBlk * pioBlockLen);
				if (pioRead) {
					for (uint32_t i = 0; i < pioWords; i++) {
						pioW[i] = *pioFifo;
					}
				}
				else {
					for (uint32_t i = 0; i < pioWords; i++) {
						*pioFifo = pioW[i];
					}
				}
			}
			if (pioErr != 0) {
				/* #154 read-bug diagnostic: on a read data-phase PIO failure log the
				 * offending host INTR_STATUS together with the card's own CMD13 R1
				 * state (issued raw with eventLock still held, BEFORE the reset).
				 * See sdcard_readDiag. */
				if (pioRead) {
					sdcard_readDiag(host, cmd, pioErrIntr);
				}
				mutexUnlock(host->eventLock);
				sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
				sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
				return pioErr;
			}
			} /* end if (!useDma): PIO byte-movement */

			if (useDma) {
				/* DMA completion: the Transfer-Complete IRQ is unreliable for DMA on
				 * this controller (it does not latch — observed intr=0x0 with the bus
				 * already idle), so DO NOT wait on TRANSFER_DONE. Poll the data lines
				 * idle instead — the SDMA analogue of the PIO per-block PRES_STATE gate:
				 * spin until DAT_BUSY|DAT_LINE_ACTIVE clear, or an error latches. */
				uint32_t dmaActive = PRES_STATE_DAT_BUSY | PRES_STATE_DAT_LINE_ACTIVE;
				uint32_t dst = 0;
				long dspin;
				for (dspin = 0; dspin < 2000000; dspin++) {
					dst = *(host->base + SDHOST_REG_INTR_STATUS);
					if ((dst & SDHOST_ERROR_REASONS) != 0u) {
						break;
					}
					if ((*(host->base + SDHOST_REG_PRES_STATE) & dmaActive) == 0u) {
						break;
					}
				}
				if (((dst & SDHOST_ERROR_REASONS) != 0u) || ((*(host->base + SDHOST_REG_PRES_STATE) & dmaActive) != 0u)) {
#ifdef SDCARD_DIAG_CLOCKSWEEP
					printf("SDDIAG: dma-complete-fail cmd=%u dir=%s pres=0x%08x intr=0x%08x\n", (unsigned)cmd,
						pioRead ? "rd" : "wr", (unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)dst);
#endif
					if (pioRead) {
						sdcard_readDiag(host, cmd, dst);
					}
					*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
					mutexUnlock(host->eventLock);
					sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
					sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
					return -EIO;
				}
				/* Clear any (possibly-latched) Transfer-Complete so it cannot go stale
				 * and prematurely satisfy a later command's wait. */
				*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_TRANSFER_DONE;
				/* Reads: data is now in the (uncached) staging buffer, nothing more to
				 * do. Writes: the SDMA data phase is done; poll the card back to TRAN
				 * (programming complete), exactly as the PIO write path does. */
				if (!pioRead) {
					ret = _sdio_pollCardReady(host, SDCARD_WRITE_BUSY_TIMEOUT_MS);
					if (ret < 0) {
						TRACE("error %d on cmd %d (DMA write CMD13 completion)", ret, cmd);
						mutexUnlock(host->eventLock);
						sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
						sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
						return ret;
					}
				}
			}
			else if (pioRead) {
				/* Reads have NO terminal busy phase (the card stays idle in TRAN),
				 * so the Transfer-Complete IRQ is the proper completion signal —
				 * provided no Auto-CMD12 STOP is appended. Single-block CMD17 never
				 * used Auto-CMD12 and latches TC reliably; multi-block CMD18 now
				 * bounds its length with CMD23 (above) with Auto-CMD12 OFF, so it
				 * should likewise latch TC. Wait on TRANSFER_DONE for both. (If TC
				 * still does not latch for CMD18, the SDDIAG read-completion-timeout
				 * line below prints intr=0x0 — that is the HW experimental result.) */
				ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_TRANSFER_DONE, 1000 * 1000);
				if (ret < 0) {
					TRACE("error %d on cmd %d (read completion, pres=0x%08x intr=0x%08x)", ret, cmd,
						(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#ifdef SDCARD_DIAG_CLOCKSWEEP
					printf("SDDIAG: read-completion-timeout cmd=%u ret=%d pres=0x%08x intr=0x%08x\n", (unsigned)cmd, ret,
						(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#endif
					/* #154 read-bug diagnostic (default-on, first few only): the
					 * host data-end-bit interrupt vs. the card's own CMD13 R1 state
					 * are different facts; log BOTH so a read EIO can be classed as
					 * host sampling margin (card in TRAN+READY) vs. a card wedge. CMD13
					 * is issued raw with eventLock still held (do NOT unlock first —
					 * the raw send condWaits on eventLock). */
					sdcard_readDiag(host, cmd, *(host->base + SDHOST_REG_INTR_STATUS));
					mutexUnlock(host->eventLock);
					/* #154: recover the DAT/CMD engine so a single timed-out data
					 * phase can't wedge every following command. */
					sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
					sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
					return ret;
				}
			}
			else {
				/* Writes (#154): the Transfer-Complete IRQ never latches and the
				 * present-state busy bits are inconsistent on this controller, so
				 * do NOT wait on TRANSFER_DONE. Instead poll the card's own status
				 * (CMD13) until it is back in TRAN + READY_FOR_DATA — exactly the
				 * write-completion check the Linux MMC core does (mmc_blk_card_busy).
				 * The PIO push already succeeded (pioErr == 0) when we reach here.
				 * eventLock stays HELD across the poll (the raw CMD13 condWaits on
				 * it); on success we fall through to the single tail unlock + R1
				 * read below — that reads the final CMD13's R1 (TRAN+READY, no
				 * error bits), which the caller's CARD_STATUS_ERRORS check accepts.
				 * The poll returns <0 on any card R1 error, so an error status can
				 * never fall through as success. For a DMA write the SDMA data phase
				 * already completed via the poll-idle check above, so this is the same
				 * card-side programming wait. */
				ret = _sdio_pollCardReady(host, SDCARD_WRITE_BUSY_TIMEOUT_MS);
				if (ret < 0) {
					TRACE("error %d on cmd %d (write CMD13 completion poll, pres=0x%08x intr=0x%08x)", ret, cmd,
						(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#ifdef SDCARD_DIAG_CLOCKSWEEP
					printf("SDDIAG: write-completion-fail cmd=%u ret=%d pres=0x%08x intr=0x%08x\n", (unsigned)cmd, ret,
						(unsigned)*(host->base + SDHOST_REG_PRES_STATE), (unsigned)*(host->base + SDHOST_REG_INTR_STATUS));
#endif
					mutexUnlock(host->eventLock);
					sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
					sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
					return ret;
				}
			}
		}
	}

	mutexUnlock(host->eventLock);
	if (res != NULL) {
		int responseLen = isLongResponse ? 4 : 1;
		for (int i = 0; i < responseLen; i++) {
			res[i] = *(host->base + SDHOST_REG_RESPONSE_0 + i);
		}
	}

	return 0;
}


/* res is single u32 if isLongResponse == 0, array of 4 u32 otherwise */
static int sdio_cmdSendEx(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res, bool isLongResponse, uint8_t *data)
{
	int ret;

	mutexLock(host->cmdLock);
	if ((cmd & SDIO_ACMD_BIT) != 0) {
		uint32_t resAcmd;
		ret = _sdio_cmdSend(host, SDIO_CMD55_APP_CMD, host->card.rca, &resAcmd, 0, false, NULL, false);
		if (ret < 0) {
			mutexUnlock(host->cmdLock);
			return ret;
		}

		if ((resAcmd & CARD_STATUS_APP_CMD) == 0) {
			LOG_ERROR("app cmd not accepted");
			mutexUnlock(host->cmdLock);
			return -EOPNOTSUPP;
		}
	}

	if (cmd >= MAX_SD_COMMANDS) {
		mutexUnlock(host->cmdLock);
		return -EINVAL;
	}

	size_t dataSize;
	switch (sdCmdMetadata[cmd].dataType) {
		case CMD_READ8:
			dataSize = 8;
			break;

		case CMD_READ64:
			dataSize = 64;
			break;

		default:
			dataSize = 0;
			break;
	}

	uint16_t blockCount = (dataSize > 0) ? 1 : 0;
	/* Register reads (SCR/SD_STATUS/SWITCH) go through the uncached staging buffer
	 * (dataBuf == NULL): they are small, rare (init only), and the caller's target
	 * may be an unaligned stack array, so the aligned FIFO loop can't target it. */
	ret = _sdio_cmdSend(host, cmd, arg, res, blockCount, isLongResponse, NULL, false);
	if (data != NULL) {
		memcpy(data, host->dmaBuffer, dataSize);
	}

	mutexUnlock(host->cmdLock);

	return ret;
}

/* Send common command with no data transfer */
static inline int sdio_cmdSend(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res)
{
	return sdio_cmdSendEx(host, cmd, arg, res, false, NULL);
}

/* Send common command with no data transfer and a long response */
static inline int sdio_cmdSendWithLongResponse(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t res[4])
{
	return sdio_cmdSendEx(host, cmd, arg, res, true, NULL);
}


static int sdcard_getCardSize(sdcard_hostData_t *host)
{
	uint32_t resp[4];
	uint32_t blockNr;
	uint32_t eraseSectorSize;
	sdcard_cardMetadata_t *card = &host->card;
	if (sdio_cmdSendWithLongResponse(host, SDIO_CMD9_SEND_CSD, card->rca, resp) < 0) {
		return -EIO;
	}

	uint32_t csdVersion = CSD_VERSION(resp);
	if (csdVersion == 0) {
		uint32_t cSize = CSDV1_C_SIZE(resp);
		uint32_t cSizeMultiplier = CSDV1_C_SIZE_MULT(resp);
		uint32_t mult = 4 << cSizeMultiplier;
		uint32_t readBlLen = CSDV1_READ_BL_LEN(resp);
		uint32_t writeBlLen = CSDV1_WRITE_BL_LEN(resp);
		blockNr = (cSize + 1) * mult;
		if (readBlLen >= 9) {
			blockNr <<= readBlLen - 9;
		}
		else {
			blockNr >>= 9 - readBlLen;
		}

		/* In cards with CSD v1 if ERASE_BLK_EN == 0 the card may erase more blocks
		 * than we selected - rounding on both sides of the range up to eraseSectorSize.
		 */
		eraseSectorSize = CSDV1_ERASE_SECTOR_SIZE(resp);
		if (writeBlLen >= 9) {
			eraseSectorSize <<= writeBlLen - 9;
		}
		else {
			uint32_t divider = 1 << (9 - writeBlLen);
			/* Round up - it's better to erase more than necessary than try to erase less
			 * and end up erasing more than intended.
			 */
			eraseSectorSize = (eraseSectorSize + divider - 1) / divider;
		}
	}
	else if (csdVersion == 1) {
		uint32_t cSize = CSDV2_C_SIZE(resp);
		blockNr = cSize << 10;
		/* In CSD v2 this field is not used and card can always erase blocks one by one */
		eraseSectorSize = 1;
	}
	else {
		/* Unknown CSD structure version; cannot compute size. */
		return -EOPNOTSUPP;
	}

	TRACE("Memory card size: %u blocks", blockNr);
	card->sizeBlocks = blockNr;
	card->eraseSizeBlocks = eraseSectorSize;
	return 0;
}


#ifdef SDCARD_DIAG_CLOCKSWEEP
/* Partial reset of the CMD/DAT engines (clock + card selection preserved) so
 * one failed attempt can't wedge the next — lets the sweep measure each clock
 * independently instead of testing a controller already dead from mode 0. */
static void sdcard_diagReset(sdcard_hostData_t *host)
{
	sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
	sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
}


static void sdcard_diagClockSweep(sdcard_hostData_t *host, unsigned int slot)
{
	const uint32_t scratchLba = 100; /* unused MBR..p1 gap */
	const uint32_t readLba = 0;      /* MBR — known to hold data */
	const uint32_t dat0Level = (1u << 20); /* PRES_STATE DAT0 line level; low = card busy */
	const int trials = 16;
	uint8_t wbuf[SDCARD_BLOCKLEN];
	uint8_t rbuf[SDCARD_BLOCKLEN];

	uint32_t caps = *(host->base + SDHOST_REG_CAPABILITIES);
	uint32_t cc = *(host->base + SDHOST_REG_CLOCK_CONTROL);
	uint32_t hc = *(host->base + SDHOST_REG_HOST_CONTROL);
	printf("SDDIAG: CAPS=0x%08x baseClk[15:8]=%uMHz CTRL1=0x%08x HOSTCTRL=0x%08x refclkMbox=%uHz (as-left N1/50MHz, no clock switching)\n",
		caps, (unsigned)((caps >> 8) & 0xff), cc, hc, (unsigned)host->refclkFrequency);
	/* Check 1 (ADMA/SDMA go/no-go): the EMMC2 DMA master reaches only CPU-phys
	 * 0x0..0x3FFFFFFF (Pi 4 emmc2bus dma-ranges, low 1 GiB). Report where our
	 * MAP_CONTIGUOUS staging buffer physically landed — DMA is viable iff < 0x40000000. */
	printf("SDDIAG: dmaBufferPhys=0x%08llx (%s for EMMC2 DMA: needs < 0x40000000)\n",
		(unsigned long long)host->dmaBufferPhys,
		(host->dmaBufferPhys < 0x40000000ull) ? "REACHABLE" : "OUT-OF-REACH");
	/* DDR50 state: HC2 lives in the upper 16 bits of the 0x3C word. */
	uint32_t hc2word = *(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS);
	printf("SDDIAG: HC2=0x%04x uhsMode=%u vdd180=%u cardUhs=%d (uhsMode 4=DDR50)\n",
		(unsigned)(hc2word >> 16), (unsigned)((hc2word >> 16) & 0x7), (unsigned)((hc2word >> 19) & 1),
		(int)host->card.uhs);

	/* Reads at the operational 50 MHz config (reset between each attempt). */
	int rdOk = 0;
	for (int t = 0; t < trials; t++) {
		sdcard_diagReset(host);
		if (sdcard_transferBlocks(slot, sdio_read, readLba, rbuf, SDCARD_BLOCKLEN) == 0) {
			rdOk++;
		}
	}
	printf("SDDIAG: read LBA%u x%d: readOk=%d/%d\n", (unsigned)readLba, trials, rdOk, trials);

	/* Large CONSECUTIVE single-block read to reproduce the >256 KB sustained-
	 * read EIO (#154/largeread): progressing LBAs, NO reset between blocks —
	 * exactly how exec / a big file (pak0.pak) reads. The SDREADDIAG probe
	 * fires inside the read path at the first error and prints the card CMD13
	 * state (state==4 TRAN ⇒ host HS50 sampling margin; state!=4 ⇒ card wedge),
	 * which selects the read fix. Reads are non-destructive (any LBA is safe). */
	{
		const uint32_t bigBlocks = 2048; /* ~1 MiB; EIO expected near block ~512 (256 KB) */
		uint32_t bigOk = 0;
		int bigFirstErr = -1;
		sdcard_diagReset(host);
		for (uint32_t i = 0; i < bigBlocks; i++) {
			if (sdcard_transferBlocks(slot, sdio_read, i, rbuf, SDCARD_BLOCKLEN) == 0) {
				bigOk++;
			}
			else if (bigFirstErr < 0) {
				bigFirstErr = (int)i;
			}
		}
		printf("SDDIAG: large consecutive read LBA0..%u: readOk=%u/%u firstErrBlk=%d\n",
			(unsigned)bigBlocks, (unsigned)bigOk, (unsigned)bigBlocks, bigFirstErr);
	}

	/* Isolated single-block WRITES at the SAME 50 MHz config — no clock
	 * switching (sdcard_diagSetDivisor was found to wedge the controller).
	 * Reset between each attempt; verify by read-back. Decisive test of
	 * whether writes work at the operational clock once recovery is correct. */
	int wrOk = 0;
	int matchOk = 0;
	for (int t = 0; t < trials; t++) {
		for (size_t b = 0; b < SDCARD_BLOCKLEN; b++) {
			wbuf[b] = (uint8_t)((t * 31) + (b * 7) + 1);
			rbuf[b] = 0;
		}
		sdcard_diagReset(host);
		int wr = sdcard_transferBlocks(slot, sdio_write, scratchLba, wbuf, SDCARD_BLOCKLEN);
		if (wr == 0) {
			wrOk++;
		}

		/* On a non-OK write, OBSERVE the card (no behaviour change to the real
		 * path): poll DAT0 busy-release and read CMD13 state. Distinguishes
		 * "data landed but Transfer-Complete IRQ missed" (DAT0 releases, card
		 * back to TRAN) from "card stuck mid-transfer" (DAT0 never releases). */
		if (wr != 0) {
			int relMs = -1;
			for (int p = 0; p < 500; p++) {
				if ((*(host->base + SDHOST_REG_PRES_STATE) & dat0Level) != 0u) {
					relMs = p;
					break;
				}
				usleep(1000);
			}
			uint32_t st = 0;
			int cs = sdio_cmdSend(host, SDIO_CMD13_SEND_STATUS, host->card.rca, &st);
			printf("SDDIAG:   t=%d wr=%d DAT0release=%dms cmd13rc=%d cardState=%u\n",
				t, wr, relMs, cs, (unsigned)CARD_STATUS_CURRENT_STATE(st));
		}

		/* Readback + content compare on EVERY attempt, regardless of wr — the
		 * key question is whether the bytes actually reached the card. */
		sdcard_diagReset(host);
		int rd = sdcard_transferBlocks(slot, sdio_read, scratchLba, rbuf, SDCARD_BLOCKLEN);
		int match = ((rd == 0) && (memcmp(wbuf, rbuf, SDCARD_BLOCKLEN) == 0));
		if (match) {
			matchOk++;
		}
		printf("SDDIAG:   t=%d wr=%d rd=%d data=%s w[0..3]=%02x%02x%02x%02x r[0..3]=%02x%02x%02x%02x\n",
			t, wr, rd, match ? "MATCH" : "NOMATCH",
			wbuf[0], wbuf[1], wbuf[2], wbuf[3], rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
	}
	printf("SDDIAG: write+readback LBA%u x%d: writeRc=%d/%d dataMatch=%d/%d\n",
		(unsigned)scratchLba, trials, wrOk, trials, matchOk, trials);

	/* No clock switching done — controller left in the operational config so
	 * the real MBR read + rootfs mount proceed normally. */
}


/* Multi-block (CMD18/CMD25) proof + throughput sweep (#154-perf). The storage
 * layer currently forces single-block I/O (a #120 sidestep), so multi-block has
 * never run on this controller. For each block count this: (1) reads the same
 * region single-block-in-a-loop and once as multi-block and byte-compares them
 * (proves CMD18 reads correct data), (2) multi-block-writes a pattern to the
 * scratch gap and reads it back to verify (proves CMD25 + the CMD13-poll write
 * completion across blocks), and times each so the boot reports MB/s. Reads are
 * non-destructive; writes hit the unused LBA100+ MBR..p1 gap. */
static uint64_t sdcard_diagElapsedUs(struct timespec *a, struct timespec *b)
{
	return (uint64_t)(b->tv_sec - a->tv_sec) * 1000000ULL + (uint64_t)(b->tv_nsec - a->tv_nsec) / 1000ULL;
}


static unsigned long sdcard_diagKBps(size_t bytes, uint64_t us)
{
	if (us == 0) {
		return 0;
	}
	return (unsigned long)(((uint64_t)bytes * 1000000ULL) / 1024ULL / us);
}


/* First 512-byte block where a and b differ, or -1 if identical. */
static int sdcard_diagFirstDiffBlk(const uint8_t *a, const uint8_t *b, uint32_t nb)
{
	for (uint32_t i = 0; i < nb; i++) {
		if (memcmp(a + (size_t)i * SDCARD_BLOCKLEN, b + (size_t)i * SDCARD_BLOCKLEN, SDCARD_BLOCKLEN) != 0) {
			return (int)i;
		}
	}
	return -1;
}


static void sdcard_diagMultiBlock(sdcard_hostData_t *host, unsigned int slot)
{
	const uint32_t maxBlk = SDCARD_MAX_TRANSFER / SDCARD_BLOCKLEN;
	const uint32_t readLba = 0;       /* MBR + p1 region: real data, non-destructive read */
	const uint32_t scratchLba = 100;  /* unused MBR..p1 gap (writes safe up to LBA ~2047) */
	const uint32_t sweep[] = { 8, 32, 128, maxBlk };
	const int trials = 10;

	uint8_t *bufA = mmap(NULL, SDCARD_MAX_TRANSFER, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	uint8_t *bufB = mmap(NULL, SDCARD_MAX_TRANSFER, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if ((bufA == MAP_FAILED) || (bufB == MAP_FAILED)) {
		printf("SDDIAG-MB: test buffer alloc failed\n");
		return;
	}

	/* The single-block path is the proven-reliable oracle (2048/2048, 16/16). Use
	 * it to VERIFY multi-block transfers so a silent multi-block fault (rc==0 but
	 * wrong data — the only unrecoverable failure, since retry can't see it) is
	 * caught distinctly from a detected/retryable error. No per-op reset (the real
	 * storage path doesn't reset between transfers); repeated to separate a
	 * transient from a real bug. */
	printf("SDDIAG-MB: oracle=single-block, trials=%d, no per-op reset (detected-fail is retryable; SILENT is the showstopper)\n", trials);

	for (size_t s = 0; s < sizeof(sweep) / sizeof(sweep[0]); s++) {
		uint32_t nb = sweep[s];
		if ((nb == 0) || (nb > maxBlk) || ((s > 0) && (nb == sweep[s - 1]))) {
			continue;
		}
		size_t len = (size_t)nb * SDCARD_BLOCKLEN;
		struct timespec t0, t1;

		/* READ A/B (the decisive diagnostic): is a per-transfer CMD+DAT re-arm the
		 * thing that makes multi-block reads work? Each trial issues the SAME
		 * multi-block read twice — once with NO reset before it, once with a
		 * sdcard_diagReset (CLOCK_CONTROL_RESET_CMD+DAT) before it. The reset is the
		 * variable that flipped reads from working (sdmb: reset-before-each) to
		 * failing (sdmb2/sds3: no reset). Also capture the first non-zero return code
		 * (rc) — every gated error probe came back empty, so we need to see WHAT the
		 * read returns (-ETIME=-37 completion, -EIO=-5 R1/CRC, etc.). */
		int rdFailNo = 0, rdFailReset = 0, rdFirstRc = 0;
		uint64_t rdUsBest = ~0ULL;
		for (int t = 0; t < trials; t++) {
			/* no reset before */
			clock_gettime(CLOCK_MONOTONIC, &t0);
			int mb = sdcard_transferBlocks(slot, sdio_read, readLba, bufA, len);
			clock_gettime(CLOCK_MONOTONIC, &t1);
			if (mb != 0) {
				rdFailNo++;
				if (rdFirstRc == 0) {
					rdFirstRc = mb;
				}
			}
			else {
				uint64_t us = sdcard_diagElapsedUs(&t0, &t1);
				if (us < rdUsBest) {
					rdUsBest = us;
				}
			}
			/* CMD+DAT re-arm before the same read */
			sdcard_diagReset(host);
			int mbR = sdcard_transferBlocks(slot, sdio_read, readLba, bufB, len);
			if (mbR != 0) {
				rdFailReset++;
				if (rdFirstRc == 0) {
					rdFirstRc = mbR;
				}
			}
		}
		printf("SDDIAG-MB: READ  nb=%u noReset-fail=%d/%d withReset-fail=%d/%d firstRc=%d bestKBps=%lu\n",
			(unsigned)nb, rdFailNo, trials, rdFailReset, trials, rdFirstRc,
			(rdUsBest == ~0ULL) ? 0UL : sdcard_diagKBps(len, rdUsBest));

		/* WRITE: multi-block write to scratch, VERIFY with single-block reads, x trials.
		 * SILENT-CORRUPT>0 means a write returned success but the bytes on the card
		 * are wrong — the unrecoverable case that blocks enabling multi-block. */
		int wrFail = 0, wrSilent = 0, wrBadBlk = -1;
		uint64_t wrUsBest = ~0ULL;
		for (int t = 0; t < trials; t++) {
			for (size_t i = 0; i < len; i++) {
				bufA[i] = (uint8_t)((i * 7) + (t * 13) + nb + 1);
			}
			clock_gettime(CLOCK_MONOTONIC, &t0);
			int wr = sdcard_transferBlocks(slot, sdio_write, scratchLba, bufA, len);
			clock_gettime(CLOCK_MONOTONIC, &t1);
			uint64_t us = sdcard_diagElapsedUs(&t0, &t1);
			memset(bufB, 0, len);
			int orErr = 0;
			for (uint32_t i = 0; i < nb; i++) {
				if (sdcard_transferBlocks(slot, sdio_read, scratchLba + i, bufB + (size_t)i * SDCARD_BLOCKLEN, SDCARD_BLOCKLEN) != 0) {
					orErr++;
				}
			}
			if (wr != 0) {
				wrFail++;
			}
			else {
				if (us < wrUsBest) {
					wrUsBest = us;
				}
				if ((orErr == 0) && (memcmp(bufA, bufB, len) != 0)) {
					wrSilent++;
					if (wrBadBlk < 0) {
						wrBadBlk = sdcard_diagFirstDiffBlk(bufA, bufB, nb);
					}
				}
			}
		}
		printf("SDDIAG-MB: WRITE nb=%u detected-fail=%d/%d SILENT-CORRUPT=%d/%d firstBadBlk=%d bestKBps=%lu\n",
			(unsigned)nb, wrFail, trials, wrSilent, trials, wrBadBlk,
			(wrUsBest == ~0ULL) ? 0UL : sdcard_diagKBps(len, wrUsBest));
	}

	munmap(bufA, SDCARD_MAX_TRANSFER);
	munmap(bufB, SDCARD_MAX_TRANSFER);
}
#endif


#ifdef SDCARD_ENABLE_DDR50
/* PRES_STATE DAT[3:0] signal levels (bits 20-23): the card holds them low during
 * the CMD11 voltage-switch handshake and releases them high once the switch to
 * 1.8V signaling is complete. */
#define PRES_STATE_DAT30_LEVEL (0xfUL << 20)

/* Perform the SD UHS-I 1.8V signal-voltage switch (CMD11 handshake). Mirrors the
 * Linux sdhci sequence: CMD11 (card drives DAT[3:0] low) -> gate the SD clock ->
 * switch the I/O rail to 1.8V (firmware GPIO) + enable HC2 1.8V signaling ->
 * restart the clock -> confirm the card released DAT[3:0] high. Returns 0 on a
 * completed switch; on any failure reverts the rail to 3.3V and returns <0 (the
 * caller then continues at 3.3V/High-Speed). Must run with the init clock active. */
static int sdcard_switchTo18V(sdcard_hostData_t *host)
{
	uint32_t resp;
	if (_sdio_cmdSend(host, SDIO_CMD11_VOLTAGE_SWITCH, 0, &resp, 0, false, NULL, false) < 0) {
		return -EIO;
	}
	if ((resp & CARD_STATUS_ERRORS) != 0) {
		return -EIO;
	}

	/* Gate the SD clock while switching (SD spec). */
	*(host->base + SDHOST_REG_CLOCK_CONTROL) &= ~CLOCK_CONTROL_START_SD_CLOCK;
	sdio_dataBarrier();
	usleep(2000);

	/* Rail to 1.8V first, then enable 1.8V signaling in the host controller. */
	if (sdio_setSdIoVoltage18(true) < 0) {
		return -EIO;
	}
	usleep(5000); /* regulator settling (DT settling-time-us = 5000) */

	uint32_t hc2 = *(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS);
	hc2 |= HOST_CONTROL2_VDD_180;
	*(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS) = hc2;
	sdio_dataBarrier();
	usleep(5000);

	if ((*(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS) & HOST_CONTROL2_VDD_180) == 0) {
		/* Host rejected 1.8V signaling — revert. */
		(void)sdio_setSdIoVoltage18(false);
		return -EIO;
	}

	/* Restart the SD clock; the card should release DAT[3:0] high within ~1 ms. */
	*(host->base + SDHOST_REG_CLOCK_CONTROL) |= CLOCK_CONTROL_START_SD_CLOCK;
	sdio_dataBarrier();
	usleep(1000);

	for (int i = 0; i < 100; i++) {
		if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_DAT30_LEVEL) == PRES_STATE_DAT30_LEVEL) {
			return 0;
		}
		usleep(1000);
	}

	/* Card never released the data lines — the switch failed; revert the rail so a
	 * re-init can proceed at 3.3V. */
	hc2 = *(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS);
	hc2 &= ~HOST_CONTROL2_VDD_180;
	*(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS) = hc2;
	(void)sdio_setSdIoVoltage18(false);
	return -EIO;
}
#endif /* SDCARD_ENABLE_DDR50 */


int sdcard_initCard(unsigned int slot, bool fallbackMode)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	host->card.commandTimeouts = 0;
	host->card.uhs = false;
	/* NOTE: a full RESET_ALL here was tried to make the SD-boot 1.8V/DDR50 switch
	 * reliable, but it BREAKS card init on SD-boot (the firmware pre-inited EMMC2 and
	 * RESET_ALL clears clock/power state this re-init does not fully restore -> the
	 * card never comes ready, "root device not found"). So do NOT full-reset here;
	 * the DDR50 switch is reliable on netboot and best-effort on SD-boot with a safe
	 * HS50 fallback. */
	/* Switch off 4-bit mode, because card will be in 1-bit mode after CMD0 */
	*(host->base + SDHOST_REG_HOST_CONTROL) &= ~HOST_CONTROL_4_BIT_MODE;
#ifdef SDCARD_ENABLE_DDR50
	/* Start every init from a known 3.3V state: clear HC2 1.8V-signaling and drive
	 * the I/O rail back to 3.3V, so CMD0/identification runs at 3.3V even after a
	 * previous (possibly failed) 1.8V switch. */
	*(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS) &= ~(HOST_CONTROL2_VDD_180 | HOST_CONTROL2_UHS_MASK);
	(void)sdio_setSdIoVoltage18(false);
#endif
	sdcard_configClockAndPower(host, SD_FREQ_INITIAL);
	/* Before we know the card's RCA, set to 0 to send to all cards */
	host->card.rca = 0;

	sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
	sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
	if (sdio_cmdSend(host, SDIO_CMD0_GO_IDLE_STATE, 0, NULL) < 0) {
		/* This will only fail if something is wrong with host, succeeds even with no card inserted */
		LOG_ERROR("go idle fail");
		return -EIO;
	}

	bool trySdhc = true;
	uint32_t cmd8Response;
	if (sdio_cmdSend(host, SDIO_CMD8_SEND_IF_COND, (IF_COND_ECHO_PATTERN | IF_COND_3V3_SUPPORTED), &cmd8Response) < 0) {
		/* This command is not supported on SD v1 cards so it must be treated as Standard Capacity*/
		trySdhc = false;
	}
	else {
		if ((cmd8Response & 0xff) != IF_COND_ECHO_PATTERN) {
			LOG_ERROR("connection error");
			return -EIO;
		}

		if (((cmd8Response >> 8) & 0xf) != 0b0001) {
			LOG_ERROR("incompatible voltage");
			return -EOPNOTSUPP;
		}
	}

	uint32_t acmd41Response;
	uint32_t acmd41Arg = trySdhc ? (1 << 30) : 0;
#ifdef SDCARD_ENABLE_DDR50
	/* S18R (bit 24): request 1.8V signaling support so the card advertises S18A in
	 * its OCR. Only meaningful on v2/SDHC cards (trySdhc). */
	if (trySdhc) {
		acmd41Arg |= (1u << 24);
	}
#endif
	int acmd41Ret = sdio_cmdSend(host, SDIO_ACMD41_SD_SEND_OP_COND, acmd41Arg, &acmd41Response);
	if (acmd41Ret < 0) {
		/* The Pi 4 EMMC2 SD slot has no software-readable card-detect, so the
		 * driver spoofs card-present and only finds out a card is absent when it
		 * fails to respond. A pure timeout (-ETIME) on ACMD41 — the OCR query
		 * every real SD card (v1 and v2) answers — means no card is present;
		 * report it as -ENODEV without logging an error so a card-absent boot
		 * (e.g. netboot with the slot empty) stays quiet. A non-timeout error is
		 * a real failure. */
		if (acmd41Ret == -ETIME) {
			return -ENODEV;
		}
		LOG_ERROR("op cond fail");
		return -EIO;
	}

	/* NOTE: on some hosts the host voltage could be changed at this point */
	if ((acmd41Response & ((1 << 20) | (1 << 21))) == 0) {
		LOG_ERROR("3.3V not supported");
		return -EOPNOTSUPP;
	}

	acmd41Arg |= acmd41Response & 0xffffff;
	/* According to docs timeout value for initialization process == 1 sec */
	for (int i = 0; i < 1000; i++) {
		if (sdio_cmdSend(host, SDIO_ACMD41_SD_SEND_OP_COND, acmd41Arg, &acmd41Response) < 0) {
			LOG_ERROR("waiting for card init failed");
			return -EIO;
		}

		if ((acmd41Response & IF_COND_READY) != 0) {
			break;
		}

		/* We can wait up to 50 ms before reissuing ACMD41 */
		usleep(1000);
	}

	if ((acmd41Response & IF_COND_READY) == 0) {
		return -ETIME;
	}

	host->card.highCapacity = trySdhc && (((acmd41Response >> 30) & 1) != 0);

#ifdef SDCARD_ENABLE_DDR50
	/* S18A (OCR bit 24): the card accepted 1.8V signaling. Switch now, while the
	 * card is in the ready state and before CMD2 identification. On any failure the
	 * rail is reverted to 3.3V inside sdcard_switchTo18V and we continue at 3.3V/HS. */
	if ((acmd41Response & (1u << 24)) != 0) {
		if (sdcard_switchTo18V(host) == 0) {
			host->card.uhs = true;
		}
		else {
			LOG_ERROR("1.8V UHS switch failed; continuing at 3.3V");
		}
	}
#endif

	/* Not sure what that is for, but it's in the documentation that we should do this */
	if (sdio_cmdSend(host, SDIO_CMD2_ALL_SEND_CID, 0, NULL) < 0) {
		return -EIO;
	}

	uint32_t cardRCA;
	if (sdio_cmdSend(host, SDIO_CMD3_RELATIVE_ADDR, 0, &cardRCA) < 0) {
		return -EIO;
	}

	host->card.rca = cardRCA & 0xffff0000;
	if (sdcard_getCardSize(host) < 0) {
		LOG_ERROR("cannot determine size");
		host->card.sizeBlocks = 0;
		host->card.eraseSizeBlocks = 1;
	}

	if (sdio_cmdSend(host, SDIO_CMD7_SELECT_CARD, host->card.rca, NULL) < 0) {
		return -EIO;
	}

	if (!host->card.highCapacity) {
		if (sdio_cmdSend(host, SDIO_CMD16_SET_BLOCKLEN, SDCARD_BLOCKLEN, NULL) < 0) {
			TRACE("set blocklen fail");
			return -EIO;
		}
	}

	uint32_t finalStatus;
	if (sdio_cmdSend(host, SDIO_CMD13_SEND_STATUS, host->card.rca, &finalStatus) < 0) {
		return -EIO;
	}

	if ((finalStatus & CARD_STATUS_ERRORS) != 0) {
		return -EIO;
	}

	if (CARD_STATUS_CURRENT_STATE(finalStatus) != CARD_STATUS_CURRENT_STATE_TRAN) {
		/* Something unexpected must have happened because card is not in a state to transfer data */
		return -EIO;
	}

	if (fallbackMode) {
		return 0;
	}

	int rc = sdcard_wideAndFast(host);
#ifdef SDCARD_DIAG_CLOCKSWEEP
	if (rc == 0) {
		sdcard_diagClockSweep(host, slot);
		sdcard_diagMultiBlock(host, slot);
	}
#endif
	return rc;
}


/* Extract information about supported functions in a given function group from the function register */
static inline uint16_t sdcard_extractFunctionGroupInfo(uint8_t fnRegister[64], uint8_t fnGroup)
{
	if ((fnGroup > 6) || (fnGroup < 1)) {
		return 0;
	}

	uint8_t fnGroupIndex = (6 - fnGroup) * 2 + 2;
	return ((uint16_t)fnRegister[fnGroupIndex] << 8) | fnRegister[fnGroupIndex + 1];
}


/* Extract status of function from a given function group from the function register */
static inline uint8_t sdcard_extractFunctionSwitchResult(uint8_t fnRegister[64], uint8_t fnGroup)
{
	if ((fnGroup > 6) || (fnGroup < 1)) {
		return 0;
	}

	fnGroup = (6 - fnGroup);
	uint8_t fnGroupIndex = fnGroup / 2 + 14;
	uint8_t fnGroupBits = ((fnGroup % 2) == 0) ? 4 : 0;
	return (fnRegister[fnGroupIndex] >> fnGroupBits) & 0xf;
}


static bool sdcard_hasHighSpeedFunction(sdcard_hostData_t *host, uint8_t tmpReg[64])
{
	uint32_t getFunctionArg = SDIO_SWITCH_FUNC_GET | SDIO_SWITCH_FUNC_HIGH_SPEED;
	if (sdio_cmdSendEx(host, SDIO_CMD6_SWITCH_FUNC, getFunctionArg, NULL, false, tmpReg) < 0) {
		return false;
	}

	uint16_t accessModeFunctions = sdcard_extractFunctionGroupInfo(tmpReg, SDCARD_FUNCTION_GROUP_ACCESS_MODE);
	return (accessModeFunctions & SDCARD_FUNCTION_GROUP_ACCESS_MODE_HIGH_SPEED) != 0;
}


/* Switches card to maximum width and speed supported */
static int sdcard_wideAndFast(sdcard_hostData_t *host)
{
	uint8_t bigRegs[64];
	/* Get SD card configuration register with some useful info about the card */
	if (sdio_cmdSendEx(host, SDIO_ACMD51_SEND_SCR, 0, NULL, false, bigRegs) < 0) {
		return -EIO;
	}

	bool cmd6Supported = SCR_SD_SPEC(bigRegs) >= SCR_SD_SPEC_V1_10;
	/* In theory all SD cards should support 4-bit, but make sure */
	if ((SCR_BUS_WIDTHS(bigRegs) & SCR_BUS_WIDTHS_4_BIT) != 0) {
		if (sdio_cmdSend(host, SDIO_ACMD6_SET_BUS_WIDTH, 2, NULL) < 0) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}

		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_4_BIT_MODE;
		usleep(10);
		if (sdio_cmdSendEx(host, SDIO_ACMD13_SD_STATUS, 0, NULL, false, bigRegs) < 0) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}

		if (SD_STATUS_DAT_BUS_WIDTH(bigRegs) != SD_STATUS_DAT_BUS_WIDTH_4_BIT) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}
	}

#ifdef SDCARD_ENABLE_DDR50
	/* UHS-I DDR50: only after a successful 1.8V switch. CMD6 SET access-mode group
	 * -> DDR50 (function 4), then run the 50 MHz clock with Host Control 2 in DDR50
	 * mode (double-data-rate sampling = ~2x the SDR bus rate at the same clock). */
	if (host->card.uhs && cmd6Supported) {
		uint32_t ddrArg = SDIO_SWITCH_FUNC_SET | SDIO_SWITCH_FUNC_DDR50;
		if ((sdio_cmdSendEx(host, SDIO_CMD6_SWITCH_FUNC, ddrArg, NULL, false, bigRegs) == 0) &&
				(sdcard_extractFunctionSwitchResult(bigRegs, SDCARD_FUNCTION_GROUP_ACCESS_MODE) == 4)) {
			if (sdcard_configClockAndPower(host, SD_FREQ_50M) < 0) {
				return -EIO;
			}
			/* Select DDR50 in Host Control 2 (upper half of the 0x3C word). */
			uint32_t hc2 = *(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS);
			hc2 = (hc2 & ~HOST_CONTROL2_UHS_MASK) | HOST_CONTROL2_UHS_DDR50;
			*(host->base + SDHOST_REG_AUTOCMD12_ERROR_STATUS) = hc2;
			sdio_dataBarrier();
			printf("sdcard: UHS-I DDR50 @ 50 MHz DDR (1.8V)\n");
			usleep(10);
			if (sdio_cmdSendEx(host, SDIO_ACMD13_SD_STATUS, 0, NULL, false, NULL) < 0) {
				LOG_ERROR("DDR50 verify failed");
				return -EIO;
			}
			return 0;
		}
		TRACE("DDR50 CMD6 not accepted; falling back");
	}
#endif

	bool isHighSpeedSupported = false;
	if (cmd6Supported && sdcard_hasHighSpeedFunction(host, bigRegs)) {
		uint32_t switchFunctionArg = SDIO_SWITCH_FUNC_SET | SDIO_SWITCH_FUNC_HIGH_SPEED;
		if (sdio_cmdSendEx(host, SDIO_CMD6_SWITCH_FUNC, switchFunctionArg, NULL, false, bigRegs) == 0) {
			if (sdcard_extractFunctionSwitchResult(bigRegs, SDCARD_FUNCTION_GROUP_ACCESS_MODE) == 1) {
				isHighSpeedSupported = true;
			}
		}
	}

	if (isHighSpeedSupported) {
		printf("sdcard: High-Speed @ 50 MHz (3.3V)\n");
		if (sdcard_configClockAndPower(host, SD_FREQ_50M) < 0) {
			return -EIO;
		}
	}
	else {
		printf("sdcard: default speed @ 25 MHz\n");
		if (sdcard_configClockAndPower(host, SD_FREQ_25M) < 0) {
			return -EIO;
		}
	}

	usleep(10);

	/* Perform a final transaction to check if data transfer is working (we only care about retcode this time) */
	if (sdio_cmdSendEx(host, SDIO_ACMD13_SD_STATUS, 0, NULL, false, NULL) < 0) {
		LOG_ERROR("bus speed change failed");
		return -EIO;
	}

	return 0;
}


static int sdcard_startEventISR(sdcard_hostData_t *host, int interruptNum)
{
	if (mutexCreate(&host->eventLock) < 0) {
		return -ENOMEM;
	}

	if (condCreate(&host->eventCond) < 0) {
		resourceDestroy(host->eventLock);
		return -ENOMEM;
	}

	interrupt(interruptNum, sdhost_isr, host, host->eventCond, &host->isrHandle);

	return 0;
}


static void _sdcard_free(sdcard_hostData_t *host)
{
	TRACE("freeing resources");
	sdhost_reset(host, CLOCK_CONTROL_RESET_ALL);
	host->sdioInitialized = false;

	if (host->isrHandle != 0) {
		resourceDestroy(host->isrHandle);
	}

	if (host->cmdLock != 0) {
		resourceDestroy(host->cmdLock);
	}

	if (host->eventCond != 0) {
		resourceDestroy(host->eventCond);
	}

	if (host->eventLock != 0) {
		resourceDestroy(host->eventLock);
	}

	if (host->dmaBuffer != NULL) {
		munmap(host->dmaBuffer, SDCARD_MAX_TRANSFER);
		host->dmaBuffer = NULL;
		host->dmaBufferPhys = (addr_t)NULL;
	}

	*(host->base + SDHOST_REG_INTR_STATUS_ENABLE) = 0;
	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
	*(host->base + SDHOST_REG_CLOCK_CONTROL) = 0;
}


void sdcard_free(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return;
	}

	if (!host->sdioInitialized) {
		return;
	}

	mutexLock(host->eventLock);
	_sdcard_free(host);
	mutexUnlock(host->eventLock);
	resourceDestroy(host->eventLock);
	host->eventLock = 0;
}


int sdcard_initHost(unsigned int slot)
{
	if (!presenceEvents.initialized) {
		if (mutexCreate(&presenceEvents.lock) < 0) {
			return -ENOMEM;
		}

		if (condCreate(&presenceEvents.cond) < 0) {
			resourceDestroy(presenceEvents.lock);
			return -ENOMEM;
		}

		presenceEvents.initialized = true;
	}

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (host->sdioInitialized) {
		return (initializedHosts < PLATFORM_SDIO_N_HOSTS) ?
			(PLATFORM_SDIO_N_HOSTS - initializedHosts) :
			0;
	}

	host->cmdLock = 0;
	host->eventCond = 0;
	host->eventLock = 0;
	host->isrHandle = 0;

	/* Perform platform-specific configuration */
	sdio_platformInfo_t info;
	if (sdio_platformConfigure(slot, &info) < 0) {
		return -EIO;
	}

	host->refclkFrequency = info.refclkFrequency;
	host->isCDPinSupported = info.isCDPinSupported;
	host->isWPPinSupported = info.isWPPinSupported;

	/* Map register bank into our virtual memory */
	void *ptr = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info.regBankPhys);
	if (ptr == MAP_FAILED) {
		_sdcard_free(host);
		return -EIO;
	}

	host->base = ptr;
	if (sdhost_allocDMA(host) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	if (mutexCreate(&host->cmdLock) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	if (sdhost_reset(host, CLOCK_CONTROL_RESET_ALL) < 0) {
		_sdcard_free(host);
		return -EIO;
	}

	if (sdcard_configClockAndPower(host, SD_FREQ_INITIAL) < 0) {
		_sdcard_free(host);
		return -EIO;
	}

	if (!host->isCDPinSupported) {
		/* Spoof card state so that it always appears inserted */
		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_CARD_DET_TEST | HOST_CONTROL_CARD_DET_TEST_ENABLE;
	}

	/* Enable the buffer read/write-ready bits in the status register too: the PIO
	 * data path (see _sdio_cmdSend) polls them in INTR_STATUS, and a bit only
	 * latches there if it is set in STATUS_ENABLE. Without this the PIO poll never
	 * sees RW_READ_READY and every data command times out (#120). */
	*(host->base + SDHOST_REG_INTR_STATUS_ENABLE) =
		SDHOST_STATUS_MASK | SDHOST_INTR_RW_READ_READY | SDHOST_INTR_RW_WRITE_READY;
	if (sdcard_startEventISR(host, info.interruptNum) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	host->sdioInitialized = true;
	initializedHosts++;
	return (initializedHosts < PLATFORM_SDIO_N_HOSTS) ?
		(PLATFORM_SDIO_N_HOSTS - initializedHosts) :
		0;
}


/* Calculate the divisor value so the output frequency is no larger than `freq` */
static int sdcard_calculateDivisor(uint32_t refclk, uint32_t freq, uint32_t *divisor)
{
	if (freq >= refclk) {
		*divisor = CLOCK_CONTROL_DIV_1;
		return 0;
	}

	for (uint32_t val = CLOCK_CONTROL_DIV_2; val <= CLOCK_CONTROL_DIV_256; val <<= 1) {
		refclk /= 2;
		if (freq >= refclk) {
			*divisor = val;
			return 0;
		}
	}

	return -1;
}


static int sdcard_configClockAndPower(sdcard_hostData_t *host, uint32_t freq)
{
	if ((freq != SD_FREQ_INITIAL) && (freq != SD_FREQ_25M) && (freq != SD_FREQ_50M)) {
		return -EINVAL;
	}

	/* NOTE: In theory, the Capabilities register can hold the reference clock frequency,
	 * but this doesn't have to be implemented. For this reason, we get the frequency
	 * from sdio_platformConfigure().
	 */
	uint32_t divRegValue;
	if (sdcard_calculateDivisor(host->refclkFrequency, freq, &divRegValue) < 0) {
		return -EINVAL;
	}

	*(host->base + SDHOST_REG_CLOCK_CONTROL) &= ~CLOCK_CONTROL_START_SD_CLOCK;

	if (freq == SD_FREQ_50M) {
		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_HIGH_SPEED;
	}
	else {
		*(host->base + SDHOST_REG_HOST_CONTROL) &= ~HOST_CONTROL_HIGH_SPEED;
	}

	/* This looks weird because we may set the "divisor" to 0, but this is intended */
	*(host->base + SDHOST_REG_CLOCK_CONTROL) = divRegValue | CLOCK_CONTROL_START_INTERNAL_CLOCK;
	sdio_dataBarrier();
	for (int i = 0; i < SDHOST_RETRIES; i++) {
		uint32_t val = *(host->base + SDHOST_REG_CLOCK_CONTROL) & CLOCK_CONTROL_INTERNAL_CLOCK_STABLE;
		if (val != 0) {
			*(host->base + SDHOST_REG_CLOCK_CONTROL) |= CLOCK_CONTROL_START_SD_CLOCK | CLOCK_CONTROL_DATA_TIMEOUT_VALUE(0b1110UL);
			*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_BUS_VOLTAGE_3V3 | HOST_CONTROL_BUS_POWER;
			return 0;
		}

		usleep(10);
	}

	return -ETIME;
}


static int _sdcard_transferBlocks(sdcard_hostData_t *host, sdio_dir_t dir, uint32_t blockOffset, void *data, size_t len)
{
	uint8_t cmd;

	/* Two data paths:
	 *  - useDma: the SDMA engine moves the data to/from the low DMA-reachable
	 *    staging buffer (uncached, so DMA-coherent); we copy across to the caller.
	 *    The copy is cheap relative to the transfer and was measured not to bound
	 *    throughput; it also lifts any alignment/physical-reach constraint on the
	 *    caller's buffer.
	 *  - PIO (fallback): move the FIFO directly to/from the caller's (cacheable)
	 *    buffer when 4-byte aligned (Linux sg_miter style, no staging copy), else
	 *    via the staging buffer.
	 * len is bounded by SDCARD_MAX_TRANSFER upstream (fits the staging buffer). */
	/* DMA READS ONLY. DMA reads are validated correct (0 silent-corrupt vs the PIO
	 * oracle, clean large-read) and DDR50-fast. DMA *writes* show intermittent
	 * first-block silent corruption that survived both a poll-idle completion and a
	 * `dsb` drain barrier — a write-DMA quirk on this controller — so writes stay on
	 * the trusted PIO path (100% correct; ~13 MB/s at the DDR50 clock). Reads are the
	 * headline win; correct DMA writes are a separate investigation. */
	bool useDma = host->useDma && (dir == sdio_read);
	void *xferBuf;
	bool bounce;
	if (useDma) {
		xferBuf = host->dmaBuffer;
		bounce = true;
		if (dir == sdio_write) {
			memcpy(host->dmaBuffer, data, len);
		}
	}
	else {
		xferBuf = data;
		bounce = (((uintptr_t)data & 0x3u) != 0u);
		if (bounce) {
			xferBuf = host->dmaBuffer;
			if (dir == sdio_write) {
				memcpy(host->dmaBuffer, data, len);
			}
		}
	}

	uint16_t blockCount = len / SDCARD_BLOCKLEN;
	if (dir == sdio_read) {
		if (blockCount > 1) {
			cmd = SDIO_CMD18_READ_MULTIPLE_BLOCK;
		}
		else {
			cmd = SDIO_CMD17_READ_SINGLE_BLOCK;
		}
	}
	else {
		if (blockCount > 1) {
			cmd = SDIO_CMD25_WRITE_MULTIPLE_BLOCK;
		}
		else {
			cmd = SDIO_CMD24_WRITE_SINGLE_BLOCK;
		}
	}

	/* The unit of “data address” in argument is byte for Standard Capacity SD Memory Card
	 * and block (512 bytes) for High Capacity SD Memory Card.
	 */
	uint32_t arg = host->card.highCapacity ? blockOffset : (blockOffset * SDCARD_BLOCKLEN);
	uint32_t resp;
	int ret = _sdio_cmdSend(host, cmd, arg, &resp, blockCount, false, xferBuf, useDma);

	if (ret < 0) {
		return ret;
	}

	if (bounce && (dir == sdio_read)) {
		memcpy(data, host->dmaBuffer, len);
	}

	if ((resp & CARD_STATUS_ERRORS) != 0) {
		LOG_ERROR("transfer error %08x", resp);
		/* #154 read-bug diagnostic (the field "transfer error 00400900" path):
		 * the command succeeded at the host level but the card's R1 carries error
		 * bits. resp IS the card R1, so report it directly (no fresh CMD13 — this
		 * runs with cmdLock held). state==TRAN+READY ⇒ host HS50 sampling margin
		 * (fix = drop clock); off-TRAN ⇒ card wedge. */
		if (dir == sdio_read) {
			sdcard_readDiagR1(cmd, *(host->base + SDHOST_REG_INTR_STATUS), resp);
		}
		return -EIO;
	}

	return 0;
}


int sdcard_transferBlocks(unsigned int slot, sdio_dir_t dir, uint32_t blockOffset, void *data, size_t len)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if ((len % SDCARD_BLOCKLEN != 0) || (len > SDCARD_MAX_TRANSFER)) {
		return -EINVAL;
	}

	if ((dir == sdio_write) && sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	int ret = _sdcard_transferBlocks(host, dir, blockOffset, data, len);
	mutexUnlock(host->cmdLock);

	return ret;
}


static int _sdcard_eraseBlocks(sdcard_hostData_t *host, uint32_t start, uint32_t end)
{
	uint32_t resp;
	int ret;

	ret = _sdio_cmdSend(host, SDIO_CMD32_ERASE_WR_BLK_START, start, &resp, 0, false, NULL, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("erase start %d %08x", ret, resp);
		return -EIO;
	}

	ret = _sdio_cmdSend(host, SDIO_CMD33_ERASE_WR_BLK_END, end, &resp, 0, false, NULL, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("erase end %d %08x", ret, resp);
		return -EIO;
	}

	ret = _sdio_cmdSend(host, SDIO_CMD38_ERASE, 0, &resp, 0, false, NULL, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("do erase %d %08x", ret, resp);
		return -EIO;
	}

	return ret;
}


int sdcard_eraseBlocks(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks)
{
	int ret = EOK;

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	if ((blockOffset % host->card.eraseSizeBlocks != 0) || (nBlocks % host->card.eraseSizeBlocks != 0)) {
		mutexUnlock(host->cmdLock);
		return -EINVAL;
	}

	uint32_t erasePerIteration = (host->card.eraseSizeBlocks > ERASE_N_BLOCKS) ?
		host->card.eraseSizeBlocks :
		ERASE_N_BLOCKS;
	while (nBlocks > 0) {
		if (nBlocks < erasePerIteration) {
			erasePerIteration = nBlocks;
		}

		uint32_t start = blockOffset;
		uint32_t end = blockOffset + erasePerIteration - 1;
		if (!host->card.highCapacity) {
			start *= SDCARD_BLOCKLEN;
			end *= SDCARD_BLOCKLEN;
		}

		ret = _sdcard_eraseBlocks(host, start, end);
		if (ret < 0) {
			break;
		}

		blockOffset += erasePerIteration;
		nBlocks -= erasePerIteration;
	}

	mutexUnlock(host->cmdLock);

	return ret;
}


int sdcard_writeFF(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks)
{
	int ret = EOK;

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	uint32_t erasePerIteration = SDCARD_MAX_TRANSFER / SDCARD_BLOCKLEN;
	memset(host->dmaBuffer, 0xff, SDCARD_MAX_TRANSFER);
	while (nBlocks > 0) {
		if (nBlocks < erasePerIteration) {
			erasePerIteration = nBlocks;
		}

		uint8_t cmd = (erasePerIteration > 1) ? SDIO_CMD25_WRITE_MULTIPLE_BLOCK : SDIO_CMD24_WRITE_SINGLE_BLOCK;
		uint32_t arg = host->card.highCapacity ? blockOffset : (blockOffset * SDCARD_BLOCKLEN);
		uint32_t resp;
		ret = _sdio_cmdSend(host, cmd, arg, &resp, erasePerIteration, false, host->dmaBuffer, false);
		if (ret < 0) {
			break;
		}

		if ((resp & CARD_STATUS_ERRORS) != 0) {
			LOG_ERROR("transfer error %08x", resp);
			ret = -EIO;
			break;
		}

		blockOffset += erasePerIteration;
		nBlocks -= erasePerIteration;
	}

	mutexUnlock(host->cmdLock);

	return ret;
}


uint32_t sdcard_getSizeBlocks(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	return host->card.sizeBlocks;
}


uint32_t sdcard_getEraseSizeBlocks(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	return host->card.eraseSizeBlocks;
}


sdcard_insertion_t sdcard_isInserted(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	uint32_t val = *(host->base + SDHOST_REG_PRES_STATE);
	if ((val & PRES_STATE_CARD_DET_STABLE) == 0) {
		return SDCARD_INSERTION_UNSTABLE;
	}

	return ((val & PRES_STATE_CARD_INSERTED) != 0) ? SDCARD_INSERTION_IN : SDCARD_INSERTION_OUT;
}


void sdcard_handlePresence(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove)
{
	int ret;
	for (unsigned int i = 0; i < PLATFORM_SDIO_N_HOSTS; i++) {
		sdcard_insertion_t state = sdcard_isInserted(i);
		if (state != SDCARD_INSERTION_UNSTABLE) {
			*(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_CARD_OUT | SDHOST_INTR_CARD_IN;
		}

		if (state == SDCARD_INSERTION_OUT) {
			if (onRemove != NULL) {
				ret = onRemove(i);
				if (ret < 0) {
					TRACE("Card eject callback returned %d", ret);
				}
			}
		}
		else if (state == SDCARD_INSERTION_IN) {
			if (onInsert != NULL) {
				ret = onInsert(i);
				if (ret < 0) {
					TRACE("Card insert callback returned %d", ret);
				}
			}
		}
	}
}


void sdcard_presenceThread(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove)
{
	int ret;
	if (!presenceEvents.initialized) {
		return;
	}

	for (;;) {
		mutexLock(presenceEvents.lock);
		for (unsigned int i = 0; i < PLATFORM_SDIO_N_HOSTS; i++) {
			uint32_t val = *(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS);
			sdio_dataBarrier();
			*(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_CARD_OUT | SDHOST_INTR_CARD_IN;
			if ((val & SDHOST_INTR_CARD_OUT) != 0) {
				if (onRemove != NULL) {
					ret = onRemove(i);
					if (ret < 0) {
						TRACE("Card eject callback returned %d", ret);
					}
				}

				TRACE("Card ejected");
			}

			if ((val & SDHOST_INTR_CARD_IN) != 0) {
				if (onInsert != NULL) {
					ret = onInsert(i);
					if (ret < 0) {
						TRACE("Card insert callback returned %d", ret);
					}
				}

				TRACE("Card inserted");
			}
		}

		condWait(presenceEvents.cond, presenceEvents.lock, 1 * 1000 * 1000);
		mutexUnlock(presenceEvents.lock);
	}
}
