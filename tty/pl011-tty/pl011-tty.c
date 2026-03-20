/*
 * Phoenix-RTOS
 *
 * PL011 tty driver
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <paths.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <board_config.h>
#include <posix/utils.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libklog.h>
#include <libtty.h>


#define KMSG_CTRL_ID 100

#ifndef PL011_TTY_BASE
#define PL011_TTY_BASE 0u
#endif

#ifndef PL011_TTY_CLOCK
#define PL011_TTY_CLOCK 24000000u
#endif

#ifndef PL011_TTY_BAUDRATE
#define PL011_TTY_BAUDRATE TTYDEF_SPEED
#endif

#ifndef PL011_TTY_POLL_US
#define PL011_TTY_POLL_US 1000
#endif


enum { dr = 0x00, fr = 0x18, ibrd = 0x24, fbrd = 0x28, lcrh = 0x2c, cr = 0x30, imsc = 0x38, icr = 0x44 };

enum { fr_rxfe = 1 << 4, fr_txff = 1 << 5 };

enum {
	lcrh_fen = 1 << 4,
	lcrh_pen = 1 << 1,
	lcrh_eps = 1 << 2,
	lcrh_stp2 = 1 << 3,
	lcrh_wlen_shift = 5,
	lcrh_wlen_mask = 0x3 << lcrh_wlen_shift
};

enum { cr_uarten = 1 << 0, cr_txe = 1 << 8, cr_rxe = 1 << 9 };


typedef struct {
	volatile uint32_t *base;
	oid_t oid;
	libtty_common_t tty;
	int speed;
	tcflag_t cflag;
	char stack[1024] __attribute__((aligned(8)));
} pl011_t;


static struct {
	pl011_t uart;
	char stack[2048] __attribute__((aligned(8)));
} pl011_common;


static inline uint32_t pl011_read(pl011_t *uart, unsigned int reg)
{
	return *(uart->base + reg / sizeof(*uart->base));
}


static inline void pl011_write(pl011_t *uart, unsigned int reg, uint32_t val)
{
	*(uart->base + reg / sizeof(*uart->base)) = val;
}


static void pl011_writeRaw(pl011_t *uart, const char *s)
{
	while (*s != '\0') {
		while ((pl011_read(uart, fr) & fr_txff) != 0) {
		}

		pl011_write(uart, dr, (unsigned char)*s++);
	}
}


static uint32_t pl011_lcrh(tcflag_t cflag)
{
	uint32_t val = lcrh_fen;

	switch (cflag & CSIZE) {
		case CS5:
			break;

		case CS6:
			val |= 0x1 << lcrh_wlen_shift;
			break;

		case CS7:
			val |= 0x2 << lcrh_wlen_shift;
			break;

		case CS8:
		default:
			val |= 0x3 << lcrh_wlen_shift;
			break;
	}

	if ((cflag & CSTOPB) != 0) {
		val |= lcrh_stp2;
	}

	if ((cflag & PARENB) != 0) {
		val |= lcrh_pen;
		if ((cflag & PARODD) == 0) {
			val |= lcrh_eps;
		}
	}

	return val;
}


static void pl011_configure(pl011_t *uart)
{
	uint32_t divisor;
	uint32_t fraction;

	if (uart->speed <= 0) {
		return;
	}

	divisor = (uint32_t)(PL011_TTY_CLOCK / (16u * (unsigned int)uart->speed));
	fraction = (uint32_t)((((uint64_t)PL011_TTY_CLOCK * 4u) / (unsigned int)uart->speed) - (divisor * 64u) + 1u) / 2u;

	pl011_write(uart, cr, 0);
	pl011_write(uart, imsc, 0);
	pl011_write(uart, icr, 0x7ffu);
	pl011_write(uart, ibrd, divisor);
	pl011_write(uart, fbrd, fraction);
	pl011_write(uart, lcrh, pl011_lcrh(uart->cflag));
	pl011_write(uart, cr, cr_uarten | cr_txe | cr_rxe);
}


static void set_baudrate(void *arg, int baud)
{
	pl011_t *uart = (pl011_t *)arg;

	if (baud <= 0) {
		return;
	}

	uart->speed = baud;
	pl011_configure(uart);
}


static void set_cflag(void *arg, tcflag_t *cflag)
{
	pl011_t *uart = (pl011_t *)arg;

	uart->cflag = *cflag;
	pl011_configure(uart);
}


static void signal_txready(void *arg)
{
	(void)arg;
}


static void pl011_klogClbk(const char *data, size_t size)
{
	libtty_write(&pl011_common.uart.tty, data, size, 0);
}


static int pl011_init(pl011_t *uart, unsigned int port)
{
	libtty_callbacks_t callbacks;

	if (PL011_TTY_BASE == 0u) {
		return -ENODEV;
	}

	uart->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)PL011_TTY_BASE);
	if (uart->base == MAP_FAILED) {
		return -ENOMEM;
	}

	callbacks.arg = uart;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, PL011_TTY_BAUDRATE) < 0) {
		return -ENOMEM;
	}

	uart->speed = uart->tty.term.c_ospeed;
	uart->cflag = uart->tty.term.c_cflag;
	uart->oid.port = port;
	uart->oid.id = 0;

	pl011_configure(uart);
	pl011_writeRaw(uart, "pl011-tty: started\r\n");

	return EOK;
}


static void pl011_ioctl(unsigned int port, msg_t *msg)
{
	const void *idata, *odata = NULL;
	oid_t oid = { .port = port };
	unsigned long req;
	int err;

	idata = ioctl_unpack(msg, &req, &oid.id);

	if (req == KIOEN) {
		libklog_enable((int)(intptr_t)idata);
		err = EOK;
	}
	else {
		err = libtty_ioctl(&pl011_common.uart.tty, ioctl_getSenderPid(msg), req, idata, &odata);
	}

	ioctl_setResponse(msg, req, err, odata);
}


static void poolthr(void *arg)
{
	unsigned int port = (uintptr_t)arg;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0) {
			continue;
		}

		if (libklog_ctrlHandle(port, &msg, rid) == 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = libtty_read(&pl011_common.uart.tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				msg.o.err = libtty_write(&pl011_common.uart.tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if (msg.i.attr.type != atPollStatus) {
					msg.o.err = -EINVAL;
					break;
				}

				msg.o.attr.val = libtty_poll_status(&pl011_common.uart.tty);
				msg.o.err = EOK;
				break;

			case mtDevCtl:
				pl011_ioctl(port, &msg);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void pl011_thr(void *arg)
{
	pl011_t *uart = (pl011_t *)arg;

	for (;;) {
		int wake_reader = 0;
		int wake_writer = 0;

		while ((pl011_read(uart, fr) & fr_rxfe) == 0) {
			libtty_putchar(&uart->tty, (unsigned char)pl011_read(uart, dr), &wake_reader);
		}

		while ((libtty_txready(&uart->tty) != 0) && ((pl011_read(uart, fr) & fr_txff) == 0)) {
			pl011_write(uart, dr, libtty_popchar(&uart->tty));
			wake_writer = 1;
		}

		if (wake_reader != 0) {
			libtty_wake_reader(&uart->tty);
		}

		if (wake_writer != 0) {
			libtty_wake_writer(&uart->tty);
		}

		usleep(PL011_TTY_POLL_US);
	}
}


int main(void)
{
	uint32_t port;

	if (portCreate(&port) < 0) {
		return EXIT_FAILURE;
	}

	if (pl011_init(&pl011_common.uart, port) < 0) {
		fprintf(stderr, "pl011-tty: failed to initialize PL011 console\n");
		return EXIT_FAILURE;
	}

	if (create_dev(&pl011_common.uart.oid, "/dev/tty0") < 0) {
		fprintf(stderr, "pl011-tty: failed to register /dev/tty0\n");
		return EXIT_FAILURE;
	}
	pl011_writeRaw(&pl011_common.uart, "pl011-tty: tty0 ready\r\n");

	if (create_dev(&pl011_common.uart.oid, _PATH_CONSOLE) < 0) {
		fprintf(stderr, "pl011-tty: failed to register %s\n", _PATH_CONSOLE);
		return EXIT_FAILURE;
	}
	pl011_writeRaw(&pl011_common.uart, "pl011-tty: console ready\r\n");

	libklog_init(pl011_klogClbk);
	oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
	libklog_ctrlRegister(&kmsgctrl);

	beginthread(pl011_thr, 4, pl011_common.uart.stack, sizeof(pl011_common.uart.stack), &pl011_common.uart);
	beginthread(poolthr, 4, pl011_common.stack, sizeof(pl011_common.stack), (void *)(uintptr_t)port);
	poolthr((void *)(uintptr_t)port);

	return EOK;
}
