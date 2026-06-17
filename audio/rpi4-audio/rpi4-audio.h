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


/* Report clock/PWM/FIFO state (scout + health). */
#define RPI4AUDIO_GETSTATE _IOR('A', 1, rpi4audio_state_t)


#endif
