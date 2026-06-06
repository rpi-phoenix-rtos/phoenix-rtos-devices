/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) GPIO - client ABI
 *
 * Read-only query interface for /dev/gpio. Setting GPIO outputs is deliberately
 * NOT exposed yet: driving real pins needs a bench rig (scope / known wiring) to
 * validate, which an autonomous netboot session cannot provide. The set path is
 * an attended follow-up; until then this device only observes pin state.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RPI4_GPIO_H_
#define _RPI4_GPIO_H_

#include <stdint.h>
#include <sys/ioctl.h>


#define RPI4_GPIO_NPINS 54u

/* Function-select (GPFSELn, 3 bits/pin). ALT order matches the BCM2711 datasheet. */
enum {
	RPI4_GPIO_FN_INPUT = 0,
	RPI4_GPIO_FN_OUTPUT = 1,
	RPI4_GPIO_FN_ALT5 = 2,
	RPI4_GPIO_FN_ALT4 = 3,
	RPI4_GPIO_FN_ALT0 = 4,
	RPI4_GPIO_FN_ALT1 = 5,
	RPI4_GPIO_FN_ALT2 = 6,
	RPI4_GPIO_FN_ALT3 = 7
};

/* Pull state (GPIO_PUP_PDN_CNTRL_REGn, 2 bits/pin; BCM2711 replaces the legacy
 * GPPUD/GPPUDCLK clock-sequence with these readable registers). */
enum {
	RPI4_GPIO_PULL_NONE = 0,
	RPI4_GPIO_PULL_UP = 1,
	RPI4_GPIO_PULL_DOWN = 2
};


typedef struct {
	uint8_t pin;   /* in:  pin index 0..53 */
	uint8_t fsel;  /* out: function-select (RPI4_GPIO_FN_*) */
	uint8_t level; /* out: current pin level (0 or 1) */
	uint8_t pull;  /* out: pull state (RPI4_GPIO_PULL_*) */
} rpi4gpio_pin_t;


/* Query one pin's function-select, level, and pull. */
#define RPI4GPIO_GETPIN _IOWR('G', 1, rpi4gpio_pin_t)


#endif
