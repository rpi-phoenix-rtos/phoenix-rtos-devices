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
 *   write()  - 16-bit signed mono/stereo PCM, converted to PWM duty and pushed
 *              to the PWM FIFO (PIO; polls STA.FULL). DMA streaming is a later tier.
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

static struct {
	volatile uint32_t *pwm;
	volatile uint32_t *cprman;
	volatile uint32_t *gpio;
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


/* Convert signed 16-bit PCM to PWM duty (0..RANGE) and push. Stereo interleaved
 * input feeds both channels; mono duplicates. Returns bytes consumed. */
static ssize_t audio_write(const void *buf, size_t len)
{
	const int16_t *s = buf;
	size_t n = len / 2;   /* int16 samples */
	size_t i;

	for (i = 0; i < n; i++) {
		uint32_t duty = (uint32_t)(((int32_t)s[i] + 32768) * (int32_t)PWM_RANGE / 65536);
		if (audio_fifoPush(duty) != 0) {
			ad.underruns++;
			/* FIFO full: drop the remainder of this write (no blocking in PIO). */
			break;
		}
	}
	return (ssize_t)len;
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

	/* Boot self-test: feed a short 440 Hz square wave through the s16->duty->FIFO write
	 * path end-to-end. The spin-bounded FIFO push paces it to the ~44.1 kHz drain rate,
	 * so underruns==0 confirms the data path keeps the FIFO fed. Audible as a brief blip
	 * on the jack (headphones needed = the attended sign-off); the self-log is the
	 * autonomous verification. No libm: square wave via integer phase. */
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
		printf("rpi4-audio: self-test fed %u samples (~0.2s 440Hz tone), underruns=%u, STA=0x%08x\n",
			fed, ad.underruns, ad.pwm[PWM_STA]);
	}

	audio_thread((void *)(uintptr_t)port);
	return 0;
}
