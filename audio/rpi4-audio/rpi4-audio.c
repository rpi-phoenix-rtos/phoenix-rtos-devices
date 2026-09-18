/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) PWM audio output (/dev/audio0)
 *
 * First-light PWM audio driver for the on-board 3.5 mm headphone jack, driven by
 * the dedicated PWM1 engine on GPIO 40/41 (ALT0). The clock/GPIO/PWM bring-up
 * sequence follows the BCM2711 peripheral datasheet (the rpi4os.com part9-sound
 * bare-metal tutorial documents the same sequence and was a useful reference);
 * the code here is an original Phoenix userspace driver, not derived from it.
 * Brings up the PWM clock (CPRMAN -> ~44.1 kHz sample class), muxes the jack
 * GPIOs, enables PWM1 channels 1+2 in mark/space + FIFO mode, and exposes a
 * streaming char device:
 *
 *   write()  - 16-bit signed mono/stereo PCM, converted to PWM duty words and fed
 *              to the jack by a continuous, self-chained DMA ring (DREQ-paced to
 *              PWM1): audio_write() fills the ring ahead of the live DMA read
 *              cursor (SOURCE_AD) and applies backpressure so the caller blocks at
 *              playback rate. Falls back to PIO (poll STA.FULL) if the DMA ring
 *              cannot be brought up.
 *   RPI4AUDIO_GETSTATE devctl - {clock busy, PWEN, STA, underruns} for the scout.
 *
 * Userspace MMIO driver in the rpi4-thermal/rpi4-gpio idiom (mmap MAP_PHYSMEM
 * uncached, portCreate + create_dev, msg loop). The *audible* sign-off needs
 * headphones on the jack (attended); the clock/GPIO/PWM bring-up + FIFO data
 * path here are self-verified via the boot self-log + the GETSTATE devctl.
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
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <posix/utils.h>

#include "rpi4-audio.h"

/* BCM2711 blocks (bus 0x7e... -> ARM low-peripheral 0xfe...). MAP_PHYSMEM needs a
 * page-aligned offset, so map the containing page and index in. PWM1 sits at
 * +0x800 inside its page; CPRMAN and GPIO are already page-aligned. */
#define PWM1_BASE    0xfe20c800u   /* PWM1 engine (drives the analog jack) */
#define PWM1_PAGE    (PWM1_BASE & ~0xfffu)
#define PWM1_OFFS    ((PWM1_BASE & 0xfffu) / 4u)   /* word offset into the page */
#define CPRMAN_BASE  0xfe101000u   /* clock manager */
#define GPIO_BASE    0xfe200000u   /* GPIO (jack pins 40/41) */

/* PWM register offsets (word index). */
enum {
	PWM_CTL  = 0x00 / 4,
	PWM_STA  = 0x04 / 4,
	PWM_DMAC = 0x08 / 4,
	PWM_RNG1 = 0x10 / 4,
	PWM_DAT1 = 0x14 / 4,
	PWM_FIF1 = 0x18 / 4,
	PWM_RNG2 = 0x20 / 4,
	PWM_DAT2 = 0x24 / 4,
};

/* PWM_CTL bits. */
#define PWEN1 (1u << 0)
#define MODE1 (1u << 1)
#define RPTL1 (1u << 2)
#define USEF1 (1u << 5)
#define CLRF1 (1u << 6)
#define MSEN1 (1u << 7)
#define PWEN2 (1u << 8)
#define USEF2 (1u << 13)
#define MSEN2 (1u << 15)

/* PWM_STA bits. */
#define STA_BERR  (1u << 8)   /* bus error: a register write did not take (NOT STA1, which is bit 9) */
#define STA_FULL1 (1u << 0)
#define STA_EMPT1 (1u << 1)
#define STA_WERR1 (1u << 2)

/* CPRMAN PWM clock. */
enum {
	CM_PWMCTL = 0xa0 / 4,
	CM_PWMDIV = 0xa4 / 4,
};
#define CM_PASSWD   0x5a000000u
#define CM_CTL_ENAB (1u << 4)
#define CM_CTL_KILL (1u << 5)
/* Bit 6 is not in the public BCM2835 peripherals doc. Linux names it CM_GATE and
 * sets it on EVERY clock enable (drivers/clk/bcm/clk-bcm2835.c, bcm2835_clock_on:
 * `ctl | CM_ENABLE | CM_GATE`), which is the only known-good driver for this block,
 * so this one follows it. Our captures read CM_PWMCTL=0x91 -- bit 6 clear -- i.e. we
 * were the only driver in the world not setting it.
 * ⓘ Measured 2026-09-18 after writing it: the bit does NOT read back. CM_PWMCTL is
 * 0x91 at the ready line on 6/6 gate boots with CM_CTL_GATE in the enable write, so
 * on the BCM2711 PWM clock this is either write-only or not implemented. The write
 * is kept because it matches the known-good driver and costs nothing; do not quote
 * it as "we now set CM_GATE" -- the hardware does not agree. */
#define CM_CTL_GATE (1u << 6)
#define CM_CTL_BUSY (1u << 7)
/* Bit 9 enables the MASH fractional divider. Linux clears it whenever the
 * fractional part of the divider is zero (clk-bcm2835.c: `ctl &= ~CM_FRAC; ctl |=
 * (div & CM_DIV_FRAC_MASK) ? CM_FRAC : 0`). Ours is an exact integer divide
 * (DIVI=2, DIVF=0), so it must be CLEAR — and the firmware leaves it SET
 * (entry CM_PWMCTL=0x200, measured 2026-09-18), so a read-modify-write that does
 * not mask it would run a fractional divider with nothing fractional to do.
 * Mask the whole MASH field [10:9], not just the bit this firmware happened to
 * leave set -- a firmware that leaves 0x400 would otherwise reintroduce it. */
#define CM_CTL_MASH (3u << 9)
#define CM_SRC_MASK 0xfu
#define CM_SRC_OSC  1u            /* BCM2711 crystal oscillator (54 MHz) */

/* GPIO GPFSEL4 covers pins 40..49; GPIO40 = bits[2:0], GPIO41 = bits[5:3]. */
#define GPFSEL4    (0x10 / 4)
#define GPIO_ALT0  4u            /* PWM0_0 / PWM0_1 on pins 40/41 */

/* Clock + range. Oscillator 54 MHz, DIVI=2 -> 27 MHz PWM clock; RNG=612 ->
 * ~44.1 kHz sample rate (27e6/612). Mark/space mode: duty = DAT/RNG. The exact
 * rate is tuned when the audible path is validated; bring-up only needs a stable
 * running clock + a draining FIFO. */
#define PWM_CLK_DIVI  2u
#define PWM_RANGE     612u
#define AUDIO_RATE    (27000000u / PWM_RANGE)  /* ~44117 Hz */

#define SPIN_MAX 1000000u

/* BCM2711 legacy DMA controller. 15 channels, 0x100 apart, from DMA_BASE. We use
 * one channel to pace the PWM FIFO from a DRAM tone buffer via the PWM DREQ (no CPU
 * spin). Bus addresses: peripherals at 0x7e... (PWM_FIF1 = 0x7e20c818); DRAM via the
 * 0xC0000000 legacy uncached alias for the low 1 GB; audio_dmaStart falls back to
 * PIO if a DMA buffer lands at/above 1 GB (the alias can't reach it). */
#define DMA_BASE        0xfe007000u
#define DMA_CHAN        5u             /* avoid VPU-reserved 0..4; revisit via the firmware mask */
#define DMA_CS          (0x00u / 4u)
#define DMA_CONBLK_AD   (0x04u / 4u)
#define DMA_SOURCE_AD   (0x0cu / 4u)   /* live source address (the ring read cursor) */
#define DMA_DEST_AD     (0x10u / 4u)   /* live destination address (the PWM FIFO) */
#define DMA_TXFR_LEN_R  (0x14u / 4u)   /* live remaining length (read-only copy) */
#define DMA_NEXTCONBK   (0x1cu / 4u)   /* next control block the engine will load */
#define DMA_DEBUG       (0x20u / 4u)
/* DMA_DEBUG bits (BCM2835 legacy DMA; the three error bits are write-1-to-clear).
 * Layout taken from the BCM2835 peripherals doc and cross-checked against Linux
 * drivers/dma/bcm2835-dma.c in external/ -- this file has already paid once for a
 * bit decoded from memory (PWM_STA's BERR/STA1 swap), so it is not repeated here. */
#define DMA_DBG_LAST_NOT_SET  (1u << 0)
#define DMA_DBG_FIFO_ERR      (1u << 1)
#define DMA_DBG_READ_ERR      (1u << 2)
#define DMA_DBG_ERRORS        (DMA_DBG_LAST_NOT_SET | DMA_DBG_FIFO_ERR | DMA_DBG_READ_ERR)
#define DMA_CS_ACTIVE   (1u << 0)
#define DMA_CS_END      (1u << 1)
#define DMA_CS_ERROR    (1u << 8)
#define DMA_CS_RESET    (1u << 31)
/* Transfer-info (CB word 0). */
#define TI_WAIT_RESP    (1u << 3)
#define TI_DEST_DREQ    (1u << 6)
#define TI_SRC_INC      (1u << 8)
#define TI_PERMAP_PWM   (1u << 16)     /* BCM2711 DREQ 1 = PWM1 (DREQ 5 is the legacy PWM0) */
#define PWM_FIF1_BUS    0x7e20c818u    /* PWM1 FIFO, peripheral bus address */
#define DRAM_BUS(pa)    (0xc0000000u | ((uint32_t)(pa) & 0x3fffffffu))
/* PWM_DMAC (0x08): ENAB(31) | PANIC[15:8] | DREQ[7:0] thresholds. */
#define PWM_DMAC_ENAB   (1u << 31)

/* Continuous-streaming DMA ring: a self-chained CB plays this ring of duty words
 * to the PWM FIFO forever (DREQ-paced); audio_write() fills it ahead of the live
 * read cursor (SOURCE_AD) with backpressure. ~0.19 s of stereo @44.1 kHz. */
#define RING_WORDS      16384u
#define RING_BYTES      (RING_WORDS * 4u)

/* A started channel is one that makes PROGRESS. The ring plays free-running silence,
 * so across the 20 ms arm settle a healthy channel walks SOURCE_AD 1784-2352 words
 * (one FIFO word per channel per range period) while the stall signature — ACTIVE,
 * DREQ_STOPPED, FIFO fed, PWM not transmitting — moves at most the FIFO depth (16).
 * Measured on this hardware: 1784-2352 words across 16 healthy boots (10-boot bench
 * + 6-app gate). The threshold sits ~28x below the slowest of those and 4x above the
 * FIFO, so it cannot fire on a merely slow boot. */
#define DMA_START_MIN_WORDS  64u
#define DMA_ARM_TRIES        3u

/* DMA control block (32 bytes, 256-bit aligned) — the legacy-DMA descriptor. */
typedef struct {
	uint32_t ti;
	uint32_t source_ad;
	uint32_t dest_ad;
	uint32_t txfr_len;
	uint32_t stride;
	uint32_t nextconbk;
	uint32_t pad[2];
} dma_cb_t;

static struct {
	volatile uint32_t *pwm;
	volatile uint32_t *cprman;
	volatile uint32_t *gpio;
	volatile uint32_t *dma;        /* DMA channel DMA_CHAN registers */
	volatile uint32_t *ring;       /* persistent duty-word ring the DMA plays */
	uintptr_t ring_pa;             /* ring physical base (for the read-cursor math) */
	uint32_t write_idx;            /* next ring word audio_write() will fill */
	int dma_active;                /* streaming DMA running -> ring path; else PIO */
	uint32_t underruns;
	uint32_t start_words;          /* ring words the DMA consumed during the start settle */
	uintptr_t cb_pa;               /* control-block physical base (re-arm needs it) */
	int null_sink;                 /* engine will not stream -> accept+drop at playback rate */
	uint32_t stalls;               /* write paths that timed out on a non-draining engine */
	uint32_t recoveries;           /* successful re-arms */
} ad;


/* Mux GPIO 40 + 41 to ALT0 (PWM). */
static void audio_gpioAlt0(void)
{
	uint32_t v = ad.gpio[GPFSEL4];
	v &= ~((7u << 0) | (7u << 3));            /* clear FSEL40, FSEL41 */
	v |= (GPIO_ALT0 << 0) | (GPIO_ALT0 << 3); /* ALT0 on both */
	ad.gpio[GPFSEL4] = v;
}


/* Bring up the CPRMAN PWM clock at DIVI from the oscillator. Returns 0 if the clock
 * reports BUSY (running) within the spin bound.
 *
 * ⚠ The ORDER here is Linux's, not the obvious one, and the difference is a race.
 * clk-bcm2835.c stops the generator, then writes CTL (source, still DISABLED), then
 * DIV, and only afterwards sets ENAB in a SEPARATE read-modify-write — with the
 * comment "we have to pause clock generation while updating the control and div regs"
 * (bcm2835_clock_set_rate / bcm2835_clock_on). This driver used to change the SOURCE
 * MUX and the ENABLE in ONE write, and write DIV before CTL: the generator's source
 * and its enable then change on the same bus cycle, and what comes out depends on
 * where that lands relative to the clock-domain crossing.
 *
 * ⓘ That is a candidate for the ~7% "enabled, clocked, FIFO fed, never transmits"
 * stall (2026-09-18 captures) — all registers read back correct on a boot that fails,
 * which is what a start-up race looks like — but it is NOT claimed as its cure. It is
 * landed because conforming to the only known-good driver for this block is right
 * either way, and because the rate of that stall is not settleable by short runs. */
static int audio_clockInit(void)
{
	uint32_t spin, ctl;

	/* Stop the generator before touching SRC or DIV (keep KILL low), wait for !BUSY. */
	ctl = ad.cprman[CM_PWMCTL] & ~(CM_PASSWD | CM_CTL_ENAB);
	ad.cprman[CM_PWMCTL] = CM_PASSWD | ctl;
	for (spin = SPIN_MAX; spin && (ad.cprman[CM_PWMCTL] & CM_CTL_BUSY); spin--) {
	}
	if ((ad.cprman[CM_PWMCTL] & CM_CTL_BUSY) != 0) {
		/* Reprogramming a generator that never stopped is the one case Linux's comment
		 * warns about, so say so rather than doing it silently. */
		printf("rpi4-audio: PWM clock still BUSY after disable (CM_PWMCTL=0x%08x) — "
			"reprogramming a running generator\n", ad.cprman[CM_PWMCTL]);
	}

	/* Source while disabled, then the divider, then enable — three separate writes. */
	ctl = (ad.cprman[CM_PWMCTL] & ~(CM_PASSWD | CM_CTL_ENAB | CM_CTL_MASH | CM_SRC_MASK))
		| CM_SRC_OSC;
	ad.cprman[CM_PWMCTL] = CM_PASSWD | ctl;
	ad.cprman[CM_PWMDIV] = CM_PASSWD | (PWM_CLK_DIVI << 12);
	ad.cprman[CM_PWMCTL] = CM_PASSWD | ctl | CM_CTL_ENAB | CM_CTL_GATE;

	for (spin = SPIN_MAX; spin && !(ad.cprman[CM_PWMCTL] & CM_CTL_BUSY); spin--) {
	}
	return (ad.cprman[CM_PWMCTL] & CM_CTL_BUSY) ? 0 : -1;
}


/* Enable PWM1 channels 1+2 in mark/space + FIFO mode at PWM_RANGE. */
static void audio_pwmInit(void)
{
	/* ⚠ PACED, and it is not cosmetic. The BCM2835 peripherals doc sets PWM_STA's
	 * BERR bit (8) when the bus "tries to write successive cycles to the same set
	 * of registers", and this function used to write PWM_CTL three times back to
	 * back (0, CLRF1, enable). Measured 2026-09-18: PWM_STA reads 0x2 (EMPT1, BERR
	 * CLEAR) at driver entry and 0x102 (EMPT1|BERR) at ready — so the driver
	 * latched a bus error on EVERY boot, and nothing decoded bit 8 to say so.
	 * A read-back between writes forces the previous one to retire; the short
	 * delay gives the PWM's own ~9.6 MHz domain time to accept it.
	 *
	 * ⓘ What this does NOT claim: that it cures the ~7% DMA stall. That stall is
	 * timing-sensitive (its rate moved from 3-in-44 to 0-in-85 across a relink
	 * with no functional change), so no short run can settle it. What is
	 * verifiable here is narrow and exact: BERR must be CLEAR on the ready line
	 * after this change, on every boot. */
	ad.pwm[PWM_CTL] = 0; /* disable while configuring */
	(void)ad.pwm[PWM_CTL];
	usleep(10);

	ad.pwm[PWM_RNG1] = PWM_RANGE;
	(void)ad.pwm[PWM_RNG1];
	ad.pwm[PWM_RNG2] = PWM_RANGE;
	(void)ad.pwm[PWM_RNG2];
	usleep(10);

	ad.pwm[PWM_CTL] = CLRF1; /* clear FIFO */
	(void)ad.pwm[PWM_CTL];
	usleep(10);

	/* Both channels: FIFO-fed (USEF), mark/space (MSEN), enabled (PWEN). */
	ad.pwm[PWM_CTL] = USEF1 | MSEN1 | PWEN1 | USEF2 | MSEN2 | PWEN2;
	(void)ad.pwm[PWM_CTL];
	usleep(10);

	/* Clear any bus error we (or the firmware) latched, so the ready line's STA
	 * reports THIS init's outcome rather than history. BERR is write-1-to-clear. */
	ad.pwm[PWM_STA] = STA_BERR;

	/* ...and say so if the period or the enable did not take. Both are read back
	 * above, so this costs nothing on a healthy boot and names the failure on a
	 * bad one instead of leaving a silently dead channel. */
	if (ad.pwm[PWM_RNG1] != PWM_RANGE) {
		printf("rpi4-audio: PWM RNG1 did not take (reads %u, wanted %u) — audio will be "
			"silent and the DMA will park\n", ad.pwm[PWM_RNG1], PWM_RANGE);
	}
	if ((ad.pwm[PWM_CTL] & (PWEN1 | USEF1)) != (PWEN1 | USEF1)) {
		printf("rpi4-audio: PWM CTL did not take (reads 0x%08x) — channel 1 not enabled in "
			"FIFO mode\n", ad.pwm[PWM_CTL]);
	}
}


/* Push one sample pair (already converted to 0..RANGE duty) into the FIFO,
 * spin-bounded on STA.FULL. Returns 0 on success, -1 on FIFO-full timeout
 * (underrun-the-other-way: producer faster than the clock drains). */
static int audio_fifoPush(uint32_t duty)
{
	uint32_t spin;
	for (spin = SPIN_MAX; spin && (ad.pwm[PWM_STA] & STA_FULL1); spin--) {
	}
	if (ad.pwm[PWM_STA] & STA_FULL1) {
		return -1;
	}
	ad.pwm[PWM_FIF1] = duty;
	return 0;
}


/* The DMA's live read cursor as a RAW ring word index -- no clamping. Used for the
 * start-progress measurement, which must not confuse "cursor outside the ring" with
 * "cursor at word 0"; audio_ringReadIdx() below clamps to 0 and would read a parked
 * channel and a healthy one alike. */
static uint32_t audio_ringWordRaw(void)
{
	uint32_t src = ad.dma[DMA_SOURCE_AD] & 0x3fffffffu;
	uint32_t base = (uint32_t)ad.ring_pa & 0x3fffffffu;
	return ((src - base) / 4u) % RING_WORDS;
}


/* The DMA's live read cursor as a ring word index (SOURCE_AD walks the ring). */
static uint32_t audio_ringReadIdx(void)
{
	uint32_t src = ad.dma[DMA_SOURCE_AD] & 0x3fffffffu;
	uint32_t base = (uint32_t)ad.ring_pa & 0x3fffffffu;
	uint32_t idx = (src - base) / 4u;
	return (idx < RING_WORDS) ? idx : 0u;
}

/* Convert signed 16-bit PCM to PWM duty (0..RANGE). Stereo interleaved input feeds
 * both channels (the shared FIFO alternates ch1/ch2 with both USEF set).
 *
 * Streaming-DMA path (preferred): copy each duty word into the free-running ring
 * ahead of the DMA read cursor. When the ring is full of not-yet-played audio we
 * spin on SOURCE_AD until the DMA drains a slot — i.e. write() blocks at the real
 * playback rate (backpressure), no CPU FIFO-spin. PIO path (fallback if the DMA
 * never started): the old per-sample FIFO push, which spins on STA.FULL.
 *
 * Either way we return the bytes *actually consumed* (len on success, a short count
 * only if the engine is genuinely stuck), so a userspace feeder (the Quakespasm
 * SNDDMA backend) advances its play cursor by exactly what was queued. */
static int audio_dmaArm(void);


static ssize_t audio_write(const void *buf, size_t len)
{
	const int16_t *s = buf;
	size_t n = len / 2;   /* int16 samples */
	size_t i;

	/* Degraded mode: the engine would not stream and the driver said so at init. Accept
	 * the data and drop it, but PACE the acceptance at the real playback rate — a sink
	 * that returns instantly turns the caller's audio thread into a spin loop that
	 * starves the render threads, which on this port means a game that runs worse with
	 * broken audio than with none. Silent audio is survivable; a blocked open() is the
	 * defect this replaces (KNOWN-ISSUES q2-sdl-openaudio-hang). */
	if (ad.null_sink != 0) {
		usleep((unsigned int)(((uint64_t)n * 500000u) / AUDIO_RATE));
		return (ssize_t)len;
	}

	for (i = 0; i < n; i++) {
		uint32_t duty = (uint32_t)(((int32_t)s[i] + 32768) * (int32_t)PWM_RANGE / 65536);

		if (ad.dma_active != 0) {
			uint32_t waits = 0;
			/* Wait for room ahead of the read cursor (keep 1 word of headroom). Yield
			 * (~0.5 ms) instead of busy-spinning so the driver doesn't burn a core
			 * competing with the game's render threads — the DMA drains ~44 words per
			 * 0.5 ms, so each wake bursts in a chunk; the loop self-paces to playback. */
			while (((ad.write_idx - audio_ringReadIdx() + RING_WORDS) % RING_WORDS) >= (RING_WORDS - 1u)) {
				usleep(500);
				if (++waits > 20000u) {   /* ~10 s with no drain -> DMA stuck */
					ad.underruns++;
					ad.stalls++;
					/* Mid-stream stall. Never observed (all three 2026-09-18 captures were
					 * at 0 samples, i.e. at init), so this does exactly what the init path
					 * does and no more: one re-arm, then degrade. */
					printf("rpi4-audio: write STALLED ~10s with no drain — CS=0x%08x "
						"DEBUG=0x%08x STA=0x%08x CTL=0x%08x RNG1=%u CM_PWMCTL=0x%08x; "
						"re-arming\n",
						ad.dma[DMA_CS], ad.dma[DMA_DEBUG], ad.pwm[PWM_STA],
						ad.pwm[PWM_CTL], ad.pwm[PWM_RNG1], ad.cprman[CM_PWMCTL]);
					ad.dma[DMA_CS] = 0;
					audio_pwmInit();
					if (audio_dmaArm() == 0) {
						ad.recoveries++;
						printf("rpi4-audio: engine RECOVERED after the stall (advanced %u "
							"words in 20ms) — the parked channel is re-armable\n",
							ad.start_words);
					}
					else {
						ad.dma_active = 0;
						ad.null_sink = 1;
						printf("rpi4-audio: engine did NOT recover (advanced %u words) — "
							"/dev/audio0 degrades to a paced null sink; a PWM re-init does "
							"not reach whatever state this is\n", ad.start_words);
					}
					return (ssize_t)(i * 2);
				}
			}
			ad.ring[ad.write_idx] = duty;
			ad.write_idx = (ad.write_idx + 1u) % RING_WORDS;
		}
		else if (audio_fifoPush(duty) != 0) {
			ad.underruns++;
			break;
		}
	}
	return (ssize_t)(i * 2);
}


static void audio_devctl(msg_t *msg)
{
	unsigned long req;
	id_t id;
	rpi4audio_state_t st;

	(void)ioctl_unpack(msg, &req, &id);
	if (req == RPI4AUDIO_GETSTATE) {
		st.rate = AUDIO_RATE;
		st.range = PWM_RANGE;
		st.cm_pwmctl = ad.cprman[CM_PWMCTL];
		st.pwm_ctl = ad.pwm[PWM_CTL];
		st.pwm_sta = ad.pwm[PWM_STA];
		st.underruns = ad.underruns;
		ioctl_setResponse(msg, req, EOK, &st);
	}
	else {
		ioctl_setResponse(msg, req, -EINVAL, NULL);
	}
}


static void audio_thread(void *arg)
{
	uint32_t port = (uint32_t)(uintptr_t)arg;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		int err = msgRecv(port, &msg, &rid);
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
				msg.o.err = (int)audio_write(msg.i.data, msg.i.size);
				break;
			case mtRead:
				msg.o.err = 0; /* not a capture device */
				break;
			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0222; /* write-only audio sink */
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;
			case mtDevCtl:
				audio_devctl(&msg);
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}
		msgRespond(port, &msg, rid);
	}
}


/* Start the free-running streaming DMA: a duty-word ring played to the PWM FIFO by
 * a self-chained control block (NEXTCONBK -> itself), DREQ-paced, forever. The ring
 * is pre-filled with mid-scale (silence). audio_write() then fills it ahead of the
 * read cursor. On success sets ad.dma_active so the write path uses the ring; on any
 * failure leaves it 0 so audio_write() falls back to the PIO FIFO push. */
/* Arm — or RE-arm — the streaming DMA on the already-allocated ring and control block,
 * and answer whether the channel is actually STREAMING rather than merely ACTIVE.
 * Allocates nothing and remaps nothing, so it is safe to call again after a failure. */
static int audio_dmaArm(void)
{
	uint32_t spins, start_word;

	ad.pwm[PWM_DMAC] = PWM_DMAC_ENAB | (8u << 8) | (4u << 0);

	ad.dma[DMA_CS] = DMA_CS_RESET;
	for (spins = 10000u; spins && (ad.dma[DMA_CS] & DMA_CS_RESET); spins--) {
	}
	ad.dma[DMA_DEBUG] = DMA_DBG_ERRORS;   /* W1C anything the engine had latched */
	ad.dma[DMA_CONBLK_AD] = DRAM_BUS(ad.cb_pa);
	ad.write_idx = 0;
	ad.dma[DMA_CS] = DMA_CS_ACTIVE;

	/* ⚠ ACTIVE with no ERROR is NOT a started channel: DMA_CS=0x21 is
	 * ACTIVE|DREQ_STOPPED, exactly the signature of the stall captured three times on
	 * 2026-09-18 — a channel parked waiting for a DREQ the PWM never raises. So measure
	 * PROGRESS instead, and sample it on EVERY boot (not only the ~7% that stall) so the
	 * figure is a continuous measurement rather than a once-in-fourteen-boots capture.
	 * usleep gives a real settle delay — a bare empty spin loop can be optimized away. */
	start_word = audio_ringWordRaw();
	usleep(20000);
	ad.start_words = (audio_ringWordRaw() - start_word + RING_WORDS) % RING_WORDS;

	return (ad.start_words >= DMA_START_MIN_WORDS) ? 0 : -1;
}


static void audio_dmaStart(void)
{
	uint32_t i, attempt;
	dma_cb_t *cb;
	uintptr_t cb_pa;

	/* CB lives at the top of a dedicated page; the ring is its own contiguous block. */
	cb = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_CONTIGUOUS | MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	ad.ring = mmap(NULL, (RING_BYTES + _PAGE_SIZE - 1u) & ~((uint32_t)_PAGE_SIZE - 1u),
		PROT_READ | PROT_WRITE, MAP_CONTIGUOUS | MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if ((cb == MAP_FAILED) || (ad.ring == MAP_FAILED)) {
		printf("rpi4-audio: dma-stream mmap failed (PIO fallback)\n");
		/* munmap whichever mapping succeeded so a partial failure doesn't leak it. */
		if (cb != MAP_FAILED) {
			munmap(cb, _PAGE_SIZE);
		}
		if (ad.ring != MAP_FAILED) {
			munmap((void *)ad.ring, (RING_BYTES + _PAGE_SIZE - 1u) & ~((uint32_t)_PAGE_SIZE - 1u));
		}
		ad.ring = NULL;
		return;
	}

	for (i = 0; i < RING_WORDS; i++)
		ad.ring[i] = PWM_RANGE / 2u;   /* mid-scale = silence */
	ad.write_idx = 0;
	ad.ring_pa = (uintptr_t)va2pa((void *)ad.ring);

	cb_pa = (uintptr_t)va2pa(cb);
	ad.cb_pa = cb_pa;

	/* The 0xC0000000 legacy DMA alias only reaches the low 1 GB. If either buffer
	 * landed at/above 1 GB (possible on a 2/4/8 GB Pi 4), DRAM_BUS() would truncate
	 * the address and the engine would fetch an unrelated DRAM region -> garbage to
	 * the PWM FIFO. Check each buffer's LAST byte (base + length - 1), not just its
	 * base: a 64 KB ring whose base is just under 1 GB can still straddle the boundary
	 * and DMA its tail from the low alias. Fall back to PIO instead of driving a bad DMA. */
	if (((((ad.ring_pa + RING_BYTES - 1u) | (cb_pa + _PAGE_SIZE - 1u)) >> 30) != 0)) {
		printf("rpi4-audio: DMA buffer PA >= 1GB (ring=0x%08x cb=0x%08x) - PIO fallback\n",
			(uint32_t)ad.ring_pa, (uint32_t)cb_pa);
		munmap((void *)ad.ring, (RING_BYTES + _PAGE_SIZE - 1u) & ~((uint32_t)_PAGE_SIZE - 1u));
		munmap(cb, _PAGE_SIZE);
		ad.ring = NULL;
		return;
	}

	cb->ti = TI_WAIT_RESP | TI_DEST_DREQ | TI_SRC_INC | TI_PERMAP_PWM;
	cb->source_ad = DRAM_BUS(ad.ring_pa);
	cb->dest_ad = PWM_FIF1_BUS;
	cb->txfr_len = RING_BYTES;
	cb->stride = 0;
	cb->nextconbk = DRAM_BUS(cb_pa);   /* self-chain -> loop the ring forever */
	cb->pad[0] = cb->pad[1] = 0;

	/* Arm, and if the channel parks instead of streaming, re-arm the PWM side and try
	 * again. Whether a re-arm un-sticks it is itself the measurement the capture record
	 * asked for: a channel that starts on attempt 2 was in a state a paced CTL/CLRF1
	 * re-init can reach, which is a different defect from one that never starts. */
	for (attempt = 1u; attempt <= DMA_ARM_TRIES; attempt++) {
		if (audio_dmaArm() == 0) {
			ad.dma_active = 1;
			break;
		}
		printf("rpi4-audio: DMA NOT STREAMING on arm %u/%u — advanced %u ring words in 20ms "
			"(want >= %u). CS=0x%08x DEBUG=0x%08x CONBLK=0x%08x SRC=0x%08x DEST=0x%08x "
			"LEN=%u STA=0x%08x CTL=0x%08x DMAC=0x%08x RNG1=%u CM_PWMCTL=0x%08x; re-arming "
			"the PWM\n",
			attempt, DMA_ARM_TRIES, ad.start_words, DMA_START_MIN_WORDS,
			ad.dma[DMA_CS], ad.dma[DMA_DEBUG], ad.dma[DMA_CONBLK_AD], ad.dma[DMA_SOURCE_AD],
			ad.dma[DMA_DEST_AD], ad.dma[DMA_TXFR_LEN_R], ad.pwm[PWM_STA], ad.pwm[PWM_CTL],
			ad.pwm[PWM_DMAC], ad.pwm[PWM_RNG1], ad.cprman[CM_PWMCTL]);
		ad.dma[DMA_CS] = 0;
		audio_pwmInit();
	}

	if (ad.dma_active == 0) {
		/* Three arms and it still will not stream. Do NOT fall back to PIO: that path
		 * spins on STA.FULL1, and a FIFO nothing drains is exactly what is wrong here, so
		 * PIO would trade a 10 s block for a slower one. Degrade to a paced null sink —
		 * the app opens /dev/audio0, runs at full speed and is silent. */
		ad.null_sink = 1;
		printf("rpi4-audio: engine would not stream in %u arms — /dev/audio0 degrades to a "
			"PACED NULL SINK (writes accepted at the playback rate and dropped). Audio is "
			"silent this boot; nothing blocks\n", DMA_ARM_TRIES);
	}
	else {
		if (attempt > 1u) {
			ad.recoveries++;
		}
		printf("rpi4-audio: dma streaming on arm %u — advanced %u ring words in 20ms, "
			"CS=0x%08x DEBUG=0x%08x\n",
			attempt, ad.start_words, ad.dma[DMA_CS], ad.dma[DMA_DEBUG]);
	}
}


int main(int argc, char **argv)
{
	uint32_t port;
	oid_t dev;
	int clkok;

	(void)argc;
	(void)argv;

	{
		volatile uint32_t *pwmpage = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)PWM1_PAGE);
		ad.pwm = (pwmpage == MAP_FAILED) ? MAP_FAILED : (pwmpage + PWM1_OFFS);
	}
	ad.cprman = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)CPRMAN_BASE);
	ad.gpio = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)GPIO_BASE);
	{
		volatile uint32_t *dmapage = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)DMA_BASE);
		ad.dma = (dmapage == MAP_FAILED) ? MAP_FAILED : (dmapage + (DMA_CHAN * 0x100u) / 4u);
	}
	if ((ad.pwm == MAP_FAILED) || (ad.cprman == MAP_FAILED) || (ad.gpio == MAP_FAILED)) {
		printf("rpi4-audio: mmap of PWM/CPRMAN/GPIO failed\n");
		return 1;
	}

	/* What the block looks like BEFORE we touch it. The ready line below reports
	 * STA=0x102 on every boot, and bit 8 of PWM_STA is BERR (a bus error latched
	 * when a register write does not take) -- bit 9 is STA1, which is the mistake
	 * to avoid here. Printing the entry value settles whether that BERR is OURS
	 * or was already set by the firmware/bootloader: 21 000 write trials on the
	 * unused PWM0 instance raised BERR exactly 0 times, so our write PATTERN is
	 * not what sets it. */
	printf("rpi4-audio: entry PWM_STA=0x%08x PWM_CTL=0x%08x RNG1=%u CM_PWMCTL=0x%08x "
		"CM_PWMDIV=0x%08x (before any write of ours)\n",
		ad.pwm[PWM_STA], ad.pwm[PWM_CTL], ad.pwm[PWM_RNG1], ad.cprman[CM_PWMCTL],
		ad.cprman[CM_PWMDIV]);

	/* P1 bring-up: GPIO ALT0, PWM clock, PWM engine. */
	audio_gpioAlt0();
	clkok = audio_clockInit();
	audio_pwmInit();

	if (portCreate(&port) != EOK) {
		printf("rpi4-audio: portCreate failed\n");
		return 2;
	}
	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "audio0") < 0) {
		printf("rpi4-audio: could not create /dev/audio0\n");
		return 3;
	}

	/* RNG1 is on this line deliberately: a channel whose PERIOD never took is
	 * enabled, clocked and fed while consuming nothing, which is the one
	 * explanation still standing for the stall captured 3x on 2026-09-18. Printing
	 * it here samples it on EVERY boot instead of only on the ~7% that stall. */
	printf("rpi4-audio: PWM1 jack @ 0x%08x, clk %s (CM_PWMCTL=0x%08x, ~%u Hz), PWM_CTL=0x%08x STA=0x%08x RNG1=%u (want %u); /dev/audio0 ready\n",
		PWM1_BASE, clkok == 0 ? "BUSY" : "FAILED", ad.cprman[CM_PWMCTL], AUDIO_RATE,
		ad.pwm[PWM_CTL], ad.pwm[PWM_STA], ad.pwm[PWM_RNG1], PWM_RANGE);

	/* Start the free-running streaming DMA (preferred write path). Falls back to PIO
	 * inside audio_write() if the DMA didn't come up. */
	if ((clkok == 0) && (ad.dma != MAP_FAILED)) {
		audio_dmaStart();
	}
	else if (ad.dma == MAP_FAILED) {
		printf("rpi4-audio: dma channel mmap failed (PIO fallback)\n");
	}

	/* Boot self-test: feed a short 440 Hz square wave through the s16->duty write path
	 * end-to-end. With the streaming DMA up this fills the ring (paced by the ring
	 * backpressure to the ~44.1 kHz drain rate); underruns==0 confirms the path keeps
	 * up. Audible as a brief blip on the jack (headphones = the attended sign-off); the
	 * self-log is the autonomous verification. No libm: square wave via integer phase. */
	if (clkok == 0) {
		int16_t tone[256];
		uint32_t total = AUDIO_RATE / 5u;            /* ~0.2 s */
		uint32_t half = AUDIO_RATE / (440u * 2u);    /* samples per half-period (~50) */
		uint32_t phase = 0, fed = 0, c, i;
		ad.underruns = 0;
		for (c = 0; c < total; c += 256u) {
			for (i = 0; i < 256u; i++, phase++)
				tone[i] = ((phase / half) & 1u) ? (int16_t)8000 : (int16_t)-8000;
			/* ⚠ STOP AT THE FIRST STALLED WRITE. This loop runs BEFORE audio_thread()
			 * and therefore before ANY message is served, so every second spent here is a
			 * second in which open("/dev/audio0") blocks on a node that already advertises
			 * itself as ready. audio_write() gives up on a stuck DMA after ~10 s and
			 * returns short; 35 chunks x 10 s is ~350 s of a device that exists and
			 * answers nothing. That is not hypothetical — it stalled a Quake II startup
			 * past a 300 s capture window (KNOWN-ISSUES q2-sdl-openaudio-hang, 2 in 136
			 * runs): the "ready" line was printed, this self-test line never was, and
			 * SDL_OpenAudio never returned. One timeout is all the evidence the self-test
			 * needs, so bail out and let the message loop start. */
			if (audio_write(tone, sizeof(tone)) != (ssize_t)sizeof(tone)) {
				/* Print everything needed to decide WHICH half stalled, because this
				 * fires at most once in ~100 boots and nobody will be watching when it
				 * does (KNOWN-ISSUES q2-sdl-openaudio-hang):
				 *   PWM_STA  FULL1 set  -> FIFO full, the PWM is not consuming (clock/DREQ)
				 *            EMPT1 set  -> FIFO empty, the DMA is not feeding (ring/CB)
				 *   DMA_CS   ACTIVE clear -> the channel stopped; ERROR set -> bus error
				 *   ring     write==read+1 (mod RING_WORDS) -> producer blocked on a full
				 *            ring, i.e. the read cursor is not advancing
				 * Healthy reference: STA=0x102 at "ready", 0x700 after a good self-test. */
				if (ad.dma_active != 0) {
					/* ⊕ 2026-09-18: this fired twice and the two registers above already
					 * named the side — DMA_CS=0x21 is ACTIVE|DREQ_STOPPED (the channel is
					 * alive, waiting for a DREQ that never comes) while PWM STA=0x100 has
					 * neither FULL1 nor EMPT1 and claims STA1, i.e. the FIFO holds data the
					 * PWM is not clocking out. That points at the PWM CLOCK, so print it:
					 * CM_PWMCTL's BUSY bit says whether the clock generator is actually
					 * running, and DIV says at what rate. Healthy: CTL has ENAB|BUSY|SRC_OSC,
					 * DIV = PWM_CLK_DIVI << 12. A stopped or unconfigured clock explains
					 * every other register in one stroke.
					 *
					 * ↩ 2026-09-18, third capture: that hypothesis is DEAD. CM_PWMCTL reads
					 * 0x91 (ENAB|BUSY|SRC_OSC) at the abort -- the generator IS running -- and
					 * CM_PWMDIV, PWM_CTL and PWM_DMAC all read back exactly what we
					 * programmed, with the ring FULL (w=15 r=16) and the FIFO non-empty. So
					 * everything configured is configured correctly and the PWM still will not
					 * drain. The one value no print has shown yet is the PERIOD: RNG1=0 would
					 * leave the channel enabled, clocked and fed while consuming nothing, which
					 * explains every other register at once. So print RNG1/DAT1 -- plus STA
					 * re-read 2 ms later, which separates a FIFO that is merely SLOW from one
					 * that is frozen. */
					printf("rpi4-audio: self-test ABORTED after %u samples — the write path "
						"stalled (DMA not draining); /dev/audio0 is still served, but audio "
						"may be silent. STA=0x%08x DMA_CS=0x%08x ring w=%u r=%u "
						"CM_PWMCTL=0x%08x CM_PWMDIV=0x%08x PWM_CTL=0x%08x PWM_DMAC=0x%08x "
						"RNG1=%u DAT1=%u STA+2ms=0x%08x DEBUG=0x%08x CONBLK=0x%08x "
						"SRC=0x%08x DEST=0x%08x LEN=%u NEXT=0x%08x start_words=%u\n",
						fed, ad.pwm[PWM_STA], ad.dma[DMA_CS], ad.write_idx,
						audio_ringReadIdx(), ad.cprman[CM_PWMCTL], ad.cprman[CM_PWMDIV],
						ad.pwm[PWM_CTL], ad.pwm[PWM_DMAC], ad.pwm[PWM_RNG1],
						ad.pwm[PWM_DAT1], (usleep(2000), ad.pwm[PWM_STA]),
						ad.dma[DMA_DEBUG], ad.dma[DMA_CONBLK_AD], ad.dma[DMA_SOURCE_AD],
						ad.dma[DMA_DEST_AD], ad.dma[DMA_TXFR_LEN_R], ad.dma[DMA_NEXTCONBK],
						ad.start_words);
				}
				else {
					printf("rpi4-audio: self-test ABORTED after %u samples — the PIO write "
						"path stalled (FIFO never drained); /dev/audio0 is still served, but "
						"audio may be silent. STA=0x%08x\n", fed, ad.pwm[PWM_STA]);
				}
				fed = 0;
				break;
			}
			fed += 256u;
		}
		if (fed != 0u) {
			printf("rpi4-audio: self-test fed %u samples (~0.2s 440Hz tone), underruns=%u, path=%s, STA=0x%08x\n",
				fed, ad.underruns, ad.dma_active ? "DMA" : "PIO", ad.pwm[PWM_STA]);
		}
	}

	audio_thread((void *)(uintptr_t)port);
	return 0;
}
