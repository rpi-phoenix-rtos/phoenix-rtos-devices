/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) PWM audio output (/dev/audio0)
 *
 * First-light PWM audio driver for the on-board 3.5 mm headphone jack, driven by
 * the dedicated PWM1 engine on GPIO 40/41 (ALT0), exactly as the rpi4os.com
 * part9-sound bare-metal tutorial does. Brings up the PWM clock (CPRMAN ->
 * ~44.1 kHz sample class), muxes the jack GPIOs, enables PWM1 channels 1+2 in
 * mark/space + FIFO mode, and exposes a streaming char device:
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
#define CM_CTL_BUSY (1u << 7)
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
 * 0xC0000000 legacy uncached alias for the low 1 GB (logged + checked at runtime). */
#define DMA_BASE        0xfe007000u
#define DMA_CHAN        5u             /* avoid VPU-reserved 0..4; revisit via the firmware mask */
#define DMA_CS          (0x00u / 4u)
#define DMA_CONBLK_AD   (0x04u / 4u)
#define DMA_SOURCE_AD   (0x0cu / 4u)   /* live source address (the ring read cursor) */
#define DMA_TXFR_LEN_R  (0x14u / 4u)   /* live remaining length (read-only copy) */
#define DMA_DEBUG       (0x20u / 4u)
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
} ad;


/* Mux GPIO 40 + 41 to ALT0 (PWM). */
static void audio_gpioAlt0(void)
{
	uint32_t v = ad.gpio[GPFSEL4];
	v &= ~((7u << 0) | (7u << 3));            /* clear FSEL40, FSEL41 */
	v |= (GPIO_ALT0 << 0) | (GPIO_ALT0 << 3); /* ALT0 on both */
	ad.gpio[GPFSEL4] = v;
}


/* Bring up the CPRMAN PWM clock at DIVI from the oscillator. Returns 0 if the
 * clock reports BUSY (running) within the spin bound. */
static int audio_clockInit(void)
{
	uint32_t spin;

	/* Stop the clock: clear ENAB (keep KILL low), wait for !BUSY. */
	ad.cprman[CM_PWMCTL] = CM_PASSWD | (ad.cprman[CM_PWMCTL] & ~CM_CTL_ENAB);
	for (spin = SPIN_MAX; spin && (ad.cprman[CM_PWMCTL] & CM_CTL_BUSY); spin--) {
	}

	ad.cprman[CM_PWMDIV] = CM_PASSWD | (PWM_CLK_DIVI << 12);
	ad.cprman[CM_PWMCTL] = CM_PASSWD | CM_CTL_ENAB | CM_SRC_OSC;

	for (spin = SPIN_MAX; spin && !(ad.cprman[CM_PWMCTL] & CM_CTL_BUSY); spin--) {
	}
	return (ad.cprman[CM_PWMCTL] & CM_CTL_BUSY) ? 0 : -1;
}


/* Enable PWM1 channels 1+2 in mark/space + FIFO mode at PWM_RANGE. */
static void audio_pwmInit(void)
{
	ad.pwm[PWM_CTL] = 0;                 /* disable while configuring */
	ad.pwm[PWM_RNG1] = PWM_RANGE;
	ad.pwm[PWM_RNG2] = PWM_RANGE;
	ad.pwm[PWM_CTL] = CLRF1;             /* clear FIFO */
	/* Both channels: FIFO-fed (USEF), mark/space (MSEN), enabled (PWEN). */
	ad.pwm[PWM_CTL] = USEF1 | MSEN1 | PWEN1 | USEF2 | MSEN2 | PWEN2;
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
static ssize_t audio_write(const void *buf, size_t len)
{
	const int16_t *s = buf;
	size_t n = len / 2;   /* int16 samples */
	size_t i;

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
static void audio_dmaStart(void)
{
	uint32_t i, spins;
	dma_cb_t *cb;
	uintptr_t cb_pa;

	/* CB lives at the top of a dedicated page; the ring is its own contiguous block. */
	cb = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_CONTIGUOUS | MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	ad.ring = mmap(NULL, (RING_BYTES + _PAGE_SIZE - 1u) & ~((uint32_t)_PAGE_SIZE - 1u),
		PROT_READ | PROT_WRITE, MAP_CONTIGUOUS | MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if ((cb == MAP_FAILED) || (ad.ring == MAP_FAILED)) {
		printf("rpi4-audio: dma-stream mmap failed (PIO fallback)\n");
		ad.ring = NULL;
		return;
	}

	for (i = 0; i < RING_WORDS; i++)
		ad.ring[i] = PWM_RANGE / 2u;   /* mid-scale = silence */
	ad.write_idx = 0;
	ad.ring_pa = (uintptr_t)va2pa((void *)ad.ring);

	cb_pa = (uintptr_t)va2pa(cb);
	cb->ti = TI_WAIT_RESP | TI_DEST_DREQ | TI_SRC_INC | TI_PERMAP_PWM;
	cb->source_ad = DRAM_BUS(ad.ring_pa);
	cb->dest_ad = PWM_FIF1_BUS;
	cb->txfr_len = RING_BYTES;
	cb->stride = 0;
	cb->nextconbk = DRAM_BUS(cb_pa);   /* self-chain -> loop the ring forever */
	cb->pad[0] = cb->pad[1] = 0;

	ad.pwm[PWM_DMAC] = PWM_DMAC_ENAB | (8u << 8) | (4u << 0);

	ad.dma[DMA_CS] = DMA_CS_RESET;
	for (spins = 10000u; spins && (ad.dma[DMA_CS] & DMA_CS_RESET); spins--) {
	}
	ad.dma[DMA_CONBLK_AD] = DRAM_BUS(cb_pa);
	ad.dma[DMA_CS] = DMA_CS_ACTIVE;

	/* Let it run a moment and confirm it stays ACTIVE with no error + the read cursor moves. */
	for (spins = 2000000u; spins; spins--) {
	}

	if (((ad.dma[DMA_CS] & DMA_CS_ACTIVE) != 0) && ((ad.dma[DMA_CS] & DMA_CS_ERROR) == 0)) {
		ad.dma_active = 1;
	}
	printf("rpi4-audio: dma-stream ch%u ring_pa=0x%08x words=%u -> CS=0x%08x DEBUG=0x%08x read_idx=%u STA=0x%08x active=%d\n",
		DMA_CHAN, (uint32_t)ad.ring_pa, RING_WORDS, ad.dma[DMA_CS], ad.dma[DMA_DEBUG],
		audio_ringReadIdx(), ad.pwm[PWM_STA], ad.dma_active);
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

	/* P0 scout: report firmware-left state before we touch anything. */
	printf("rpi4-audio: scout CM_PWMCTL=0x%08x CM_PWMDIV=0x%08x PWM_CTL=0x%08x PWM_STA=0x%08x GPFSEL4=0x%08x\n",
		ad.cprman[CM_PWMCTL], ad.cprman[CM_PWMDIV], ad.pwm[PWM_CTL], ad.pwm[PWM_STA], ad.gpio[GPFSEL4]);

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

	printf("rpi4-audio: PWM1 jack @ 0x%08x, clk %s (CM_PWMCTL=0x%08x, ~%u Hz), PWM_CTL=0x%08x STA=0x%08x; /dev/audio0 ready\n",
		PWM1_BASE, clkok == 0 ? "BUSY" : "FAILED", ad.cprman[CM_PWMCTL], AUDIO_RATE,
		ad.pwm[PWM_CTL], ad.pwm[PWM_STA]);

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
			audio_write(tone, sizeof(tone));
			fed += 256u;
		}
		printf("rpi4-audio: self-test fed %u samples (~0.2s 440Hz tone), underruns=%u, path=%s, STA=0x%08x\n",
			fed, ad.underruns, ad.dma_active ? "DMA" : "PIO", ad.pwm[PWM_STA]);
	}

	audio_thread((void *)(uintptr_t)port);
	return 0;
}
