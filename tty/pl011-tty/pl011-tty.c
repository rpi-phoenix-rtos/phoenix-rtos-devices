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
#include <fcntl.h>
#include <paths.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <board_config.h>
#include <posix/utils.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libklog.h>
#include <libtty.h>

#if defined(__CPU_GENERIC)
#include <phoenix/arch/aarch64/generic/generic.h>
#include "../pc-tty/ttypc_fbfont.h"
#endif


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

#ifndef PL011_TTY_KBD_PATH
#define PL011_TTY_KBD_PATH ((const char *)NULL)
#endif

#ifndef PL011_TTY_KBD_RETRY_US
#define PL011_TTY_KBD_RETRY_US 500000
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

#define PL011_FBCON_BG 0xff000000u
#define PL011_FBCON_FG 0xffffffffu


typedef struct {
	volatile uint32_t *base;
	oid_t oid;
	libtty_common_t tty;
	int speed;
	tcflag_t cflag;
	handle_t fbLock;
	volatile uint32_t *fbaddr;
	uint32_t fbmemsz;
	uint16_t fbcols;
	uint16_t fbrows;
	uint16_t fbcol;
	uint16_t fbrow;
	uint16_t fbpitch;
	char stack[4096] __attribute__((aligned(8)));
	char kbdstack[4096] __attribute__((aligned(8)));
} pl011_t;


static struct {
	pl011_t uart;
	char stack[4096] __attribute__((aligned(8)));
} pl011_common;


static inline uint32_t pl011_read(pl011_t *uart, unsigned int reg)
{
	return *(uart->base + reg / sizeof(*uart->base));
}


static inline void pl011_write(pl011_t *uart, unsigned int reg, uint32_t val)
{
	*(uart->base + reg / sizeof(*uart->base)) = val;
}


#if defined(__CPU_GENERIC)
static inline void pl011_fbcon_drawPixel(pl011_t *uart, uint16_t x, uint16_t y, uint32_t color)
{
	*(volatile uint32_t *)((char *)uart->fbaddr + y * uart->fbpitch + x * sizeof(uint32_t)) = color;
}


static void pl011_fbcon_drawChar(pl011_t *uart, uint16_t col, uint16_t row, unsigned char c)
{
	uint16_t x = col * TTYPC_FBFONT_W;
	uint16_t y = row * TTYPC_FBFONT_H;
	uint8_t *data = ttypc_fbcon_fbfont + (TTYPC_FBFONT_BYTES_PER_GLYPH * c);
	uint16_t charPixY;
	size_t i;

	for (charPixY = y; charPixY < (y + TTYPC_FBFONT_H); ++charPixY) {
		for (i = 0u; i < 8u; ++i) {
			pl011_fbcon_drawPixel(uart, x + (7u - i), charPixY, ((*data & (1u << i)) != 0u) ? PL011_FBCON_FG : PL011_FBCON_BG);
		}

		data += TTYPC_FBFONT_W_BYTES;
	}
}


static void pl011_fbcon_clearRow(pl011_t *uart, uint16_t row)
{
	uint16_t y;
	uint16_t x;

	for (y = row * TTYPC_FBFONT_H; y < (row + 1u) * TTYPC_FBFONT_H; ++y) {
		for (x = 0u; x < uart->fbcols * TTYPC_FBFONT_W; ++x) {
			pl011_fbcon_drawPixel(uart, x, y, PL011_FBCON_BG);
		}
	}
}


static void pl011_fbcon_scroll(pl011_t *uart)
{
	size_t rowsz = uart->fbpitch * TTYPC_FBFONT_H;
	size_t visible = uart->fbpitch * uart->fbrows * TTYPC_FBFONT_H;

	memmove((void *)uart->fbaddr, (const char *)uart->fbaddr + rowsz, visible - rowsz);
	uart->fbrow = uart->fbrows - 1u;
	pl011_fbcon_clearRow(uart, uart->fbrow);
}


static void pl011_fbcon_putc(pl011_t *uart, unsigned char c)
{
	if (uart->fbaddr == NULL) {
		return;
	}

	switch (c) {
		case '\r':
			uart->fbcol = 0u;
			return;

		case '\n':
			uart->fbcol = 0u;
			uart->fbrow++;
			break;

		case '\b':
			if (uart->fbcol > 0u) {
				uart->fbcol--;
				pl011_fbcon_drawChar(uart, uart->fbcol, uart->fbrow, ' ');
			}
			return;

		default:
			if ((c < ' ') || (c > '~')) {
				c = '?';
			}

			pl011_fbcon_drawChar(uart, uart->fbcol, uart->fbrow, c);
			uart->fbcol++;
			if (uart->fbcol >= uart->fbcols) {
				uart->fbcol = 0u;
				uart->fbrow++;
			}
			break;
	}

	if (uart->fbrow >= uart->fbrows) {
		pl011_fbcon_scroll(uart);
	}
}


static void pl011_fbcon_write(pl011_t *uart, const char *data, size_t size)
{
	size_t i;

	if (uart->fbaddr == NULL) {
		return;
	}

	mutexLock(uart->fbLock);
	for (i = 0u; i < size; ++i) {
		pl011_fbcon_putc(uart, (unsigned char)data[i]);
	}
	mutexUnlock(uart->fbLock);
}


static int pl011_fbcon_init(pl011_t *uart)
{
	platformctl_t pctl = { .action = pctl_get, .type = pctl_graphmode };
	uint16_t row;
	int err;

	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	if ((pctl.task.graphmode.bpp != 32u) || (pctl.task.graphmode.framebuffer == 0u) || (pctl.task.graphmode.pitch == 0u)) {
		return -ENOSYS;
	}

	uart->fbmemsz = (pctl.task.graphmode.pitch * pctl.task.graphmode.height + _PAGE_SIZE - 1u) & ~(_PAGE_SIZE - 1u);
	uart->fbaddr = mmap(NULL, uart->fbmemsz, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_SHARED | MAP_UNCACHED | MAP_ANONYMOUS | MAP_PHYSMEM, -1, pctl.task.graphmode.framebuffer);
	if (uart->fbaddr == MAP_FAILED) {
		uart->fbaddr = NULL;
		return -ENOMEM;
	}

	uart->fbpitch = pctl.task.graphmode.pitch;
	uart->fbcols = pctl.task.graphmode.width / TTYPC_FBFONT_W;
	uart->fbrows = pctl.task.graphmode.height / TTYPC_FBFONT_H;
	uart->fbcol = 0u;
	uart->fbrow = 6u;
	if (uart->fbrow >= uart->fbrows) {
		uart->fbrow = 0u;
	}

	for (row = 0u; row < uart->fbrows; ++row) {
		pl011_fbcon_clearRow(uart, row);
	}

	pl011_fbcon_write(uart, "Phoenix-RTOS HDMI console\r\n", sizeof("Phoenix-RTOS HDMI console\r\n") - 1u);

	return EOK;
}
#else
static void pl011_fbcon_write(pl011_t *uart, const char *data, size_t size)
{
	(void)uart;
	(void)data;
	(void)size;
}


static int pl011_fbcon_init(pl011_t *uart)
{
	(void)uart;
	return -ENOSYS;
}
#endif


static void pl011_writeRaw(pl011_t *uart, const char *s)
{
	if (uart->fbaddr != NULL) {
		pl011_fbcon_write(uart, s, strlen(s));
	}

	while (*s != '\0') {
		while ((pl011_read(uart, fr) & fr_txff) != 0) {
		}

		pl011_write(uart, dr, (unsigned char)*s++);
	}
}


static int pl011_createTty0(pl011_t *uart)
{
	static const char name[] = "tty0";
	oid_t odev;
	msg_t msg = { 0 };
	int err;
	unsigned int i;

	pl011_writeRaw(uart, "pl011-tty: tty0 lookup\r\n");
	err = -ENODEV;
	for (i = 0; i < 50; ++i) {
		err = lookup("devfs", NULL, &odev);
		if (err >= 0) {
			break;
		}

		if ((i == 0U) || (((i + 1U) % 10U) == 0U)) {
			pl011_writeRaw(uart, "pl011-tty: tty0 lookup retry\r\n");
		}

		usleep(100000);
		if (i == 0U) {
			pl011_writeRaw(uart, "pl011-tty: tty0 wake\r\n");
		}
	}

	if (err < 0) {
		pl011_writeRaw(uart, "pl011-tty: tty0 lookup failed\r\n");
		return err;
	}

	pl011_writeRaw(uart, "pl011-tty: tty0 lookup ok\r\n");

	msg.type = mtCreate;
	msg.oid = odev;
	msg.i.create.dev = uart->oid;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0666;
	msg.i.data = name;
	msg.i.size = sizeof(name);

	pl011_writeRaw(uart, "pl011-tty: tty0 send\r\n");
	if (msgSend(odev.port, &msg) != EOK) {
		pl011_writeRaw(uart, "pl011-tty: tty0 send failed\r\n");
		return -ENOMEM;
	}

	pl011_writeRaw(uart, "pl011-tty: tty0 send done\r\n");

	return msg.o.err;
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

	if (mutexCreate(&uart->fbLock) < 0) {
		return -ENOMEM;
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
	(void)pl011_fbcon_init(uart);
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
			unsigned char c = libtty_popchar(&uart->tty);

			pl011_write(uart, dr, c);
			pl011_fbcon_write(uart, (const char *)&c, 1u);
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


static void pl011_kbdthr(void *arg)
{
	pl011_t *uart = (pl011_t *)arg;
	const char *path = PL011_TTY_KBD_PATH;
	char buf[64];
	ssize_t len;
	int fd;

	if (path == NULL) {
		endthread();
	}

	for (;;) {
		fd = open(path, O_RDONLY);
		if (fd < 0) {
			usleep(PL011_TTY_KBD_RETRY_US);
			continue;
		}

		for (;;) {
			int wake_reader = 0;
			size_t i;

			len = read(fd, buf, sizeof(buf));
			if (len == 0) {
				break;
			}
			if (len < 0) {
				if (errno == EINTR) {
					continue;
				}
				break;
			}

			libtty_putchar_lock(&uart->tty);
			for (i = 0u; i < (size_t)len; ++i) {
				int woke = 0;

				(void)libtty_putchar_unlocked(&uart->tty, (unsigned char)buf[i], &wake_reader);
				wake_reader |= woke;
			}
			libtty_putchar_unlock(&uart->tty);

			if (wake_reader != 0) {
				libtty_wake_reader(&uart->tty);
			}
		}

		close(fd);
		usleep(PL011_TTY_KBD_RETRY_US);
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

	pl011_writeRaw(&pl011_common.uart, "pl011-tty: register tty0\r\n");
	if (pl011_createTty0(&pl011_common.uart) < 0) {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: tty0 failed\r\n");
		fprintf(stderr, "pl011-tty: failed to register /dev/tty0\n");
		return EXIT_FAILURE;
	}
	pl011_writeRaw(&pl011_common.uart, "pl011-tty: tty0 ready\r\n");

	pl011_writeRaw(&pl011_common.uart, "pl011-tty: register console\r\n");
	if (create_dev(&pl011_common.uart.oid, _PATH_CONSOLE) < 0) {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: console failed\r\n");
		fprintf(stderr, "pl011-tty: failed to register %s\n", _PATH_CONSOLE);
		return EXIT_FAILURE;
	}
	pl011_writeRaw(&pl011_common.uart, "pl011-tty: console ready\r\n");

	libklog_init(pl011_klogClbk);
	oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
	libklog_ctrlRegister(&kmsgctrl);

	beginthread(pl011_thr, 4, pl011_common.uart.stack, sizeof(pl011_common.uart.stack), &pl011_common.uart);
	if (PL011_TTY_KBD_PATH != NULL) {
		beginthread(pl011_kbdthr, 4, pl011_common.uart.kbdstack, sizeof(pl011_common.uart.kbdstack), &pl011_common.uart);
	}
	beginthread(poolthr, 4, pl011_common.stack, sizeof(pl011_common.stack), (void *)(uintptr_t)port);
	poolthr((void *)(uintptr_t)port);

	return EOK;
}
