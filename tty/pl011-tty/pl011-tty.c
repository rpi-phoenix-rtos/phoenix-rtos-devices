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


/* TD-15 Stage 4: minimal VT100/ANSI parser state for the HDMI fbcon
 * path. Without this the literal escape bytes from psh / klog
 * (e.g. "\x1b[0J", "\x1b[2J", "\x1b[H", "\x1b[r;cH") show up on
 * screen as garbage characters ("?[0J(psh)%" in the user-captured
 * HDMI screenshot). PL011_FBCON_ESC_MAX_PARAMS bounds the parameter
 * list (max 8 params is well above the longest sequence psh
 * realistically emits). */
#define PL011_FBCON_ESC_MAX_PARAMS 8u

typedef enum {
	pl011_fbcon_esc_normal = 0,
	pl011_fbcon_esc_gotEsc,    /* received \x1b, waiting for the next char */
	pl011_fbcon_esc_inCsi,     /* in a CSI (Control Sequence Introducer) sequence \x1b[ */
} pl011_fbcon_escState_t;


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

	/* TD-15 Stage 4: VT100/ANSI parser state. */
	pl011_fbcon_escState_t fbescState;
	uint16_t fbescParams[PL011_FBCON_ESC_MAX_PARAMS];
	uint8_t fbescNumParams;
	uint8_t fbescPad;

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


/* TD-15 Stage 4: 64-bit BG fill primitive. With kernel D-cache disabled
 * (Stage 1 parked, see TD-16-cache-enable), every framebuffer write
 * goes straight to DDR and pays a per-store penalty. Switching from
 * per-pixel uint32_t stores to 64-bit stores roughly halves the
 * instruction count and lets the compiler emit STP pairs for inner
 * loops. The full-screen clear is still observably slow on HDMI (the
 * line-by-line wipe the user reported is the cache-off cost), but
 * this halves it. */
static inline void pl011_fbcon_fill64(volatile uint32_t *base, size_t bytes, uint32_t color)
{
	volatile uint64_t *p = (volatile uint64_t *)base;
	size_t qwords = bytes / sizeof(uint64_t);
	uint64_t pair = ((uint64_t)color << 32) | (uint64_t)color;
	size_t i;

	for (i = 0u; i < qwords; ++i) {
		p[i] = pair;
	}

	/* Tail (a single u32 left over) — extremely unlikely with sane
	 * pitch values, but cover it correctly anyway. */
	if ((bytes & (sizeof(uint64_t) - 1u)) != 0u) {
		base[bytes / sizeof(uint32_t) - 1u] = color;
	}
}


static void pl011_fbcon_clearRow(pl011_t *uart, uint16_t row)
{
	uint16_t y;
	size_t rowBytes = (size_t)uart->fbcols * (size_t)TTYPC_FBFONT_W * sizeof(uint32_t);

	for (y = row * TTYPC_FBFONT_H; y < (row + 1u) * TTYPC_FBFONT_H; ++y) {
		volatile uint32_t *line = (volatile uint32_t *)((char *)uart->fbaddr + y * uart->fbpitch);
		pl011_fbcon_fill64(line, rowBytes, PL011_FBCON_BG);
	}
}


/* TD-15 Stage 4: clear a half-open row range. */
static void pl011_fbcon_clearRowRange(pl011_t *uart, uint16_t firstRow, uint16_t lastRowExcl)
{
	uint16_t r;
	for (r = firstRow; r < lastRowExcl; ++r) {
		pl011_fbcon_clearRow(uart, r);
	}
}


/* TD-15 Stage 4: clear the fbmemsz-bounded area to BG. Uses the
 * 64-bit fill primitive to halve instruction count vs the previous
 * per-pixel clearRow loop. */
static void pl011_fbcon_clearAll(pl011_t *uart)
{
	pl011_fbcon_fill64(uart->fbaddr, uart->fbmemsz, PL011_FBCON_BG);
	uart->fbcol = 0u;
	uart->fbrow = 0u;
}


static void pl011_fbcon_scroll(pl011_t *uart)
{
	size_t rowsz = uart->fbpitch * TTYPC_FBFONT_H;
	size_t visible = uart->fbpitch * uart->fbrows * TTYPC_FBFONT_H;

	memmove((void *)uart->fbaddr, (const char *)uart->fbaddr + rowsz, visible - rowsz);
	uart->fbrow = uart->fbrows - 1u;
	pl011_fbcon_clearRow(uart, uart->fbrow);
}


/* TD-15 Stage 4: dispatch a CSI sequence (ESC [ params final).
 *
 * We support the small set of sequences psh and klog actually emit:
 *   J  - Erase in display.   0/missing: cursor to end. 2: entire screen.
 *   K  - Erase in line.      0/missing: cursor to EOL. 2: entire line.
 *   H  - Cursor position.    Optional row;col, defaulting to 1;1.
 *   f  - Cursor position.    Same as H.
 *   A  - Cursor up by N (default 1).
 *   B  - Cursor down by N.
 *   C  - Cursor forward by N.
 *   D  - Cursor back by N.
 *   m  - SGR (color/attr).   Consumed silently — fbcon is BG/FG only.
 *
 * Anything else is consumed silently. The cursor is row-clamped to
 * [0, fbrows) and column-clamped to [0, fbcols).
 */
static void pl011_fbcon_dispatchCsi(pl011_t *uart, unsigned char final)
{
	uint16_t r;
	uint16_t param0 = (uart->fbescNumParams > 0u) ? uart->fbescParams[0] : 0u;
	uint16_t param1 = (uart->fbescNumParams > 1u) ? uart->fbescParams[1] : 0u;
	uint16_t row;
	uint16_t col;
	uint16_t step;

	switch (final) {
		case 'J':
			if (param0 == 0u) {
				/* Cursor to end of screen */
				for (r = uart->fbcol; r < uart->fbcols; ++r) {
					pl011_fbcon_drawChar(uart, r, uart->fbrow, ' ');
				}
				pl011_fbcon_clearRowRange(uart, uart->fbrow + 1u, uart->fbrows);
			}
			else if (param0 == 2u) {
				pl011_fbcon_clearRowRange(uart, 0u, uart->fbrows);
			}
			break;

		case 'K':
			if (param0 == 0u) {
				/* Cursor to EOL */
				for (r = uart->fbcol; r < uart->fbcols; ++r) {
					pl011_fbcon_drawChar(uart, r, uart->fbrow, ' ');
				}
			}
			else if (param0 == 2u) {
				pl011_fbcon_clearRow(uart, uart->fbrow);
			}
			break;

		case 'H':
		case 'f':
			row = (param0 > 0u) ? (param0 - 1u) : 0u;
			col = (param1 > 0u) ? (param1 - 1u) : 0u;
			if (row >= uart->fbrows) {
				row = uart->fbrows - 1u;
			}
			if (col >= uart->fbcols) {
				col = uart->fbcols - 1u;
			}
			uart->fbrow = row;
			uart->fbcol = col;
			break;

		case 'A':
			step = (param0 > 0u) ? param0 : 1u;
			uart->fbrow = (uart->fbrow > step) ? (uart->fbrow - step) : 0u;
			break;

		case 'B':
			step = (param0 > 0u) ? param0 : 1u;
			uart->fbrow += step;
			if (uart->fbrow >= uart->fbrows) {
				uart->fbrow = uart->fbrows - 1u;
			}
			break;

		case 'C':
			step = (param0 > 0u) ? param0 : 1u;
			uart->fbcol += step;
			if (uart->fbcol >= uart->fbcols) {
				uart->fbcol = uart->fbcols - 1u;
			}
			break;

		case 'D':
			step = (param0 > 0u) ? param0 : 1u;
			uart->fbcol = (uart->fbcol > step) ? (uart->fbcol - step) : 0u;
			break;

		case 'm':
			/* SGR — ignored; fbcon is BG/FG only. */
			break;

		default:
			/* Unsupported final char — drop silently. */
			break;
	}
}


static void pl011_fbcon_putc(pl011_t *uart, unsigned char c)
{
	if (uart->fbaddr == NULL) {
		return;
	}

	/* TD-15 Stage 4: VT100/ANSI ESC parser. Without this the raw
	 * escape bytes from psh / klog are rendered literally on HDMI. */
	switch (uart->fbescState) {
		case pl011_fbcon_esc_gotEsc:
			if (c == '[') {
				uart->fbescState = pl011_fbcon_esc_inCsi;
				uart->fbescNumParams = 0u;
				uart->fbescParams[0] = 0u;
			}
			else if (c == 'c') {
				/* RIS — reset to initial state. */
				pl011_fbcon_clearAll(uart);
				uart->fbescState = pl011_fbcon_esc_normal;
			}
			else {
				/* Two-byte non-CSI sequence we don't model — drop. */
				uart->fbescState = pl011_fbcon_esc_normal;
			}
			return;

		case pl011_fbcon_esc_inCsi:
			if ((c >= '0') && (c <= '9')) {
				if (uart->fbescNumParams == 0u) {
					uart->fbescNumParams = 1u;
				}
				uart->fbescParams[uart->fbescNumParams - 1u] =
					(uart->fbescParams[uart->fbescNumParams - 1u] * 10u) + (c - '0');
				return;
			}
			else if (c == ';') {
				if (uart->fbescNumParams < PL011_FBCON_ESC_MAX_PARAMS) {
					uart->fbescNumParams++;
					uart->fbescParams[uart->fbescNumParams - 1u] = 0u;
				}
				return;
			}
			else if ((c == '?') || (c == '>')) {
				/* Private CSI introducer — consume but treat the rest
				 * as standard. */
				return;
			}
			else if ((c >= 0x40u) && (c <= 0x7eu)) {
				pl011_fbcon_dispatchCsi(uart, c);
				uart->fbescState = pl011_fbcon_esc_normal;
				return;
			}
			else {
				/* Out-of-spec byte inside CSI — abort the sequence. */
				uart->fbescState = pl011_fbcon_esc_normal;
				return;
			}
			/* unreachable */

		case pl011_fbcon_esc_normal:
		default:
			if (c == 0x1bu) {
				uart->fbescState = pl011_fbcon_esc_gotEsc;
				return;
			}
			break;
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

	/* TD-15 Stage 4: reset VT100 parser and clear the entire mapped
	 * framebuffer (raw 32-bit fill of fbmemsz bytes — covers any
	 * pitch padding / partial bottom row that the per-glyph clearRow
	 * loop would skip). The previous boot left the firmware splash
	 * visible in the lower half of the HDMI screen because the per-
	 * glyph loop only covered fbrows*FONT_H rows. */
	uart->fbescState = pl011_fbcon_esc_normal;
	uart->fbescNumParams = 0u;
	pl011_fbcon_clearAll(uart);

	(void)row;

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
	/* TODO(TD-14-pl011-retry): originally 50 retries (5 s wall), bumped
	 * to 500 (50 s) for slow Pi 4 IPC, then back DOWN to 30 (3 s) once
	 * we made the caller TD-14-tty0-nonfatal. /dev/tty0 is now optional;
	 * if devfs lookup is slow we'd rather give up quickly here and let
	 * the create_dev() fallback path handle /dev/console registration
	 * directly via portRegister. Restore to 50 after IPC slowness is
	 * rooted out and the non-fatal hack is reverted. */
	for (i = 0; i < 30; ++i) {
		err = lookup("devfs", NULL, &odev);
		if (err >= 0) {
			break;
		}

		if ((i == 0U) || (((i + 1U) % 10U) == 0U)) {
			pl011_writeRaw(uart, "pl011-tty: tty0 lookup retry\r\n");
		}

		usleep(100000);
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


static void pl011_attrAll(struct _attrAll *attrs)
{
	memset(attrs, 0, sizeof(*attrs));
	attrs->mode.val = S_IFCHR | 0666;
	attrs->links.val = 1;
	attrs->ioblock.val = 1;
	attrs->pollStatus.val = libtty_poll_status(&pl011_common.uart.tty);
}


static void pl011_stat(struct stat *st, const oid_t *oid)
{
	memset(st, 0, sizeof(*st));
	st->st_dev = oid->port;
	st->st_ino = oid->id;
	st->st_rdev = oid->port;
	st->st_mode = S_IFCHR | 0666;
	st->st_nlink = 1;
	st->st_blksize = 1;
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
				switch (msg.i.attr.type) {
					case atMode:
						msg.o.attr.val = S_IFCHR | 0666;
						msg.o.err = EOK;
						break;

					case atPollStatus:
						msg.o.attr.val = libtty_poll_status(&pl011_common.uart.tty);
						msg.o.err = EOK;
						break;

					default:
						msg.o.err = -EINVAL;
						break;
				}
				break;

			case mtGetAttrAll:
				if ((msg.o.data == NULL) || (msg.o.size < sizeof(struct _attrAll))) {
					msg.o.err = -EINVAL;
					break;
				}

				pl011_attrAll(msg.o.data);
				msg.o.err = EOK;
				break;

			case mtStat:
				if ((msg.o.data == NULL) || (msg.o.size < sizeof(struct stat))) {
					msg.o.err = -EINVAL;
					break;
				}

				pl011_stat(msg.o.data, &msg.oid);
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
			if (uart->fbaddr != NULL) {
				pl011_fbcon_write(uart, (const char *)&c, 1u);
			}
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
	/* TODO(TD-14-tty0-nonfatal): pl011_createTty0() depends on a fast
	 * lookup("devfs") that is intermittently slow on real Pi 4 (TD-04-
	 * class IPC fragility). It runs the same lookup as create_dev()
	 * below — but lacks the latter's fallback, so it can hang for tens
	 * of seconds and block the entire pl011-tty bring-up. Treat its
	 * failure as non-fatal: skip /dev/tty0 if it doesn't register
	 * cleanly and proceed to register /dev/console (whose libphoenix
	 * helper has its own portRegister fallback). /dev/tty0 is not
	 * strictly required for psh's shell prompt to come up. Restore
	 * the fatal path once the underlying IPC slowness is rooted out. */
	if (pl011_createTty0(&pl011_common.uart) < 0) {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: tty0 failed (non-fatal)\r\n");
		fprintf(stderr, "pl011-tty: tty0 register failed (non-fatal, continuing)\n");
	}
	else {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: tty0 ready\r\n");
	}

	pl011_writeRaw(&pl011_common.uart, "pl011-tty: register console\r\n");
	if (create_dev(&pl011_common.uart.oid, _PATH_CONSOLE) < 0) {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: console failed\r\n");
		fprintf(stderr, "pl011-tty: failed to register %s\n", _PATH_CONSOLE);
		return EXIT_FAILURE;
	}

	/* TODO(TD-14-console-alias): keep the direct kernel namespace alias
	 * until Pi 4 bind/devfs lookup latency is fixed. create_dev() registers
	 * the node in devfs; this alias preserves the fast /dev/console path used
	 * by early shell startup and mirrors create_dev()'s fallback behavior. */
	if (portRegister(port, _PATH_CONSOLE, &pl011_common.uart.oid) < 0) {
		pl011_writeRaw(&pl011_common.uart, "pl011-tty: console alias skipped\r\n");
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

	{
		/* TD-15 / Stage 4 phase 1: instrument fbcon init result so we
		 * can tell on real Pi whether (a) graphmode was never populated
		 * (returns -ENOSYS — plo's mailbox path didn't fill it in), or
		 * (b) framebuffer mmap failed (returns -ENOMEM), or (c) it
		 * actually succeeded. Direct UART write via pl011_writeRaw to
		 * avoid debug() IPC dependency. */
		int fbres = pl011_fbcon_init(&pl011_common.uart);
		if (fbres == EOK) {
			pl011_writeRaw(&pl011_common.uart, "fbcon: ok\r\n");
		}
		else if (fbres == -ENOSYS) {
			pl011_writeRaw(&pl011_common.uart, "fbcon: skip (graphmode not populated)\r\n");
		}
		else if (fbres == -ENOMEM) {
			pl011_writeRaw(&pl011_common.uart, "fbcon: skip (framebuffer mmap failed)\r\n");
		}
		else {
			pl011_writeRaw(&pl011_common.uart, "fbcon: skip (other error)\r\n");
		}
	}

	poolthr((void *)(uintptr_t)port);

	return EOK;
}
