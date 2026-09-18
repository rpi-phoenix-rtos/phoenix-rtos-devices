/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) PWM audio - client ABI
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RPI4_AUDIO_H_
#define _RPI4_AUDIO_H_

#include <stdint.h>
#include <sys/ioctl.h>


typedef struct {
	uint32_t rate;      /* sample rate in Hz */
	uint32_t range;     /* PWM range (full-scale duty) */
	uint32_t cm_pwmctl; /* clock-manager PWM control (bit7 = BUSY) */
	uint32_t pwm_ctl;   /* PWM control (PWEN1/PWEN2 = running) */
	uint32_t pwm_sta;   /* PWM status (FULL/EMPT/WERR) */
	uint32_t underruns; /* FIFO-full drops since start */
} rpi4audio_state_t;


/* Repeat the driver's own DMA arm sequence `trials` times and report how many came
 * up PARKED (enabled, clocked, FIFO fed, not transmitting). This exists because the
 * stall behind KNOWN-ISSUES q2-sdl-openaudio-hang fires on ~7% of BOOTS, i.e. about
 * seven samples per hundred boots at 2.5 minutes each, while the userspace probes
 * that sample the same shape thousands of times per boot (tools/pwm-write-probe,
 * tools/pwm-dma-probe) necessarily run on PWM0 / DREQ 5 / channel 6 -- never on the
 * instance that actually fails. This is the only way to sample PWM1 / DREQ 1 /
 * channel 5 at that rate.
 *
 * ⚠ It blocks the message loop for ~20 ms per trial and resets the streaming engine
 * on every one, so /dev/audio0 plays nothing while it runs. Diagnostic only. */
typedef struct {
	uint32_t trials;      /* in:  arm cycles to run; out: the count after the driver's cap */
	uint32_t ran;         /* out: cycles actually run */
	uint32_t parked;      /* out: cycles that did not stream */
	uint32_t minWords;    /* out: smallest advance seen, in ring words */
	uint32_t maxWords;    /* out: largest advance seen */
	uint32_t firstSta;    /* out: PWM_STA at the first parked cycle (0 if none) */
	uint32_t firstCs;     /* out: DMA_CS at the first parked cycle */
	uint32_t firstDebug;  /* out: DMA_DEBUG at the first parked cycle */
} rpi4audio_armtrials_t;


/* Report clock/PWM/FIFO state (scout + health). */
#define RPI4AUDIO_GETSTATE _IOR('A', 1, rpi4audio_state_t)

/* Run the arm-trial loop above. Diagnostic; see the struct comment. */
#define RPI4AUDIO_ARMTRIALS _IOWR('A', 2, rpi4audio_armtrials_t)


#endif
