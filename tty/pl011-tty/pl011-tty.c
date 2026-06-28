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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <board_config.h>
#include <phoenix/fbcon.h>
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

/* Poll cadence for the non-blocking kbd bridge read: small enough that typing feels
 * instant, large enough to stay off the CPU when idle, and bounds how fast the bridge
 * reacts to kbdReleased (release/reacquire of /dev/kbd0). */
#ifndef PL011_TTY_KBD_POLL_US
#define PL011_TTY_KBD_POLL_US 8000
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

/* 16-colour ANSI palette (SGR, #49), ARGB8888 with opaque alpha. Indices 0-7 are
 * the normal colours (SGR 30-37 / 40-47), 8-15 the bright ones (SGR 90-97 /
 * 100-107, or 0-7 with bold). */
static const uint32_t pl011_fbcon_palette[16] = {
	0xff000000u, 0xffaa0000u, 0xff00aa00u, 0xffaa5500u, /* blk red grn yel  */
	0xff0000aau, 0xffaa00aau, 0xff00aaaau, 0xffaaaaaau, /* blu mag cyn wht  */
	0xff555555u, 0xffff5555u, 0xff55ff55u, 0xffffff55u, /* bright black-yel */
	0xff5555ffu, 0xffff55ffu, 0xff55ffffu, 0xffffffffu, /* bright blu-white */
};


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
	/* TD-15 Stage 4 phase 1c: the POINTER itself is volatile (not just
	 * the pointed-to data). pl011_thr starts before pl011_fbcon_init
	 * (line 995 vs the fbcon_init call ~15 lines later) and reads
	 * uart->fbaddr in its inner TX-drain loop to decide whether to
	 * mirror tty bytes to HDMI. Without `* volatile` on the pointer
	 * field the compiler is free to cache the initial NULL value in
	 * a register and never re-read it after fbcon_init writes the
	 * mmap result, which is exactly what we observed on real Pi 4
	 * (banner + `fbcon: ok` show on HDMI but no klog or psh content
	 * even though UART receives the full stream). */
	volatile uint32_t *volatile fbaddr;
	uint32_t fbmemsz;
	uint16_t fbcols;
	uint16_t fbrows;
	uint16_t fbcol;
	uint16_t fbrow;
	uint16_t fbpitch;

	/* DOS-style "text mode <-> graphics mode" switch for full-screen apps
	 * (FBCONSETMODE/FBCONGETMODE ioctls). When DISABLED, a full-screen GPU app
	 * (e.g. rpi4-quake) owns the real framebuffer; the console keeps drawing into
	 * an off-screen shadow so klog/psh text is never lost, and on re-ENABLE the
	 * shadow is blitted back so the user sees the console with everything that was
	 * printed while it was hidden. fbdraw is the live draw target: fbaddr when
	 * enabled, fbshadow when disabled. */
	int fbmode;                          /* FBCON_ENABLED / FBCON_DISABLED / FBCON_UNSUPPORTED */
	volatile uint32_t *volatile fbdraw;  /* current draw target (fbaddr or fbshadow) */
	uint32_t *fbshadow;                  /* off-screen console image (lazily allocated) */

	/* When a full-screen app takes the HDMI console into graphics mode
	 * (FBCONSETMODE(FBCON_DISABLED)) it also owns the USB keyboard, so the kbd
	 * bridge releases /dev/kbd0 (single-opener usbkbd) for the app to open. Set on
	 * DISABLE, cleared on ENABLE; the kbd bridge thread polls it. */
	volatile int kbdReleased;

	/* TD-15 Stage 4: VT100/ANSI parser state. */
	pl011_fbcon_escState_t fbescState;
	uint16_t fbescParams[PL011_FBCON_ESC_MAX_PARAMS];
	uint8_t fbescNumParams;
	uint8_t fbescPad;

	/* ANSI SGR colour state (#49) + text cursor (#50). */
	uint32_t fbfg;      /* current foreground colour */
	uint32_t fbbg;      /* current background colour */
	uint8_t fbbold;     /* SGR 1 (bold) -> brighten the 30-37 foreground */
	uint8_t fbcurShown; /* an XOR cursor is currently applied at fbcurCol/Row */
	uint16_t fbcurCol;  /* where that cursor was drawn */
	uint16_t fbcurRow;

	char stack[4096] __attribute__((aligned(8)));
	char kbdstack[4096] __attribute__((aligned(8)));
	char klogstack[4096] __attribute__((aligned(8)));
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
	*(volatile uint32_t *)((char *)uart->fbdraw + y * uart->fbpitch + x * sizeof(uint32_t)) = color;
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
			pl011_fbcon_drawPixel(uart, x + (7u - i), charPixY, ((*data & (1u << i)) != 0u) ? uart->fbfg : uart->fbbg);
		}

		data += TTYPC_FBFONT_W_BYTES;
	}
}


/* Text cursor (#50): invert (XOR) every pixel of one character cell. XOR is its
 * own inverse, so the same call both draws and erases the cursor and the glyph
 * underneath survives (shown in reverse video) — an empty cell becomes a solid
 * block. Drawn after each console write at the live position and erased before
 * the next, so it tracks the prompt without leaving artefacts. */
static void pl011_fbcon_xorCursor(pl011_t *uart, uint16_t col, uint16_t row)
{
	uint16_t x0 = col * TTYPC_FBFONT_W;
	uint16_t y0 = row * TTYPC_FBFONT_H;
	uint16_t x, y;

	for (y = y0; y < (y0 + TTYPC_FBFONT_H); ++y) {
		volatile uint32_t *line = (volatile uint32_t *)((char *)uart->fbdraw + y * uart->fbpitch);
		for (x = x0; x < (x0 + TTYPC_FBFONT_W); ++x) {
			line[x] ^= 0x00ffffffu;
		}
	}
}


/* 64-bit BG fill primitive. The framebuffer is mapped Normal
 * Non-Cacheable (it is VC4 scanout DRAM, not cacheable memory), so
 * every store goes straight to DDR. Using 64-bit stores instead of
 * per-pixel uint32_t stores halves the instruction count and lets the
 * compiler emit STP pairs for the inner loop, doubling fill
 * throughput on the non-cacheable mapping. */
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
		volatile uint32_t *line = (volatile uint32_t *)((char *)uart->fbdraw + y * uart->fbpitch);
		pl011_fbcon_fill64(line, rowBytes, PL011_FBCON_BG);
	}
}


/* TD-15 Stage 4: clear the fbmemsz-bounded area to BG. Uses the
 * 64-bit fill primitive to halve instruction count vs the previous
 * per-pixel clearRow loop. */
static void pl011_fbcon_clearAll(pl011_t *uart)
{
	pl011_fbcon_fill64(uart->fbdraw, uart->fbmemsz, PL011_FBCON_BG);
	uart->fbcol = 0u;
	uart->fbrow = 0u;
}


static void pl011_fbcon_scroll(pl011_t *uart)
{
	size_t rowsz = uart->fbpitch * TTYPC_FBFONT_H;
	size_t visible = uart->fbpitch * uart->fbrows * TTYPC_FBFONT_H;

	memmove((void *)uart->fbdraw, (const char *)uart->fbdraw + rowsz, visible - rowsz);
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
	uint16_t param0 = (uart->fbescNumParams > 0u) ? uart->fbescParams[0] : 0u;
	uint16_t param1 = (uart->fbescNumParams > 1u) ? uart->fbescParams[1] : 0u;
	uint16_t row;
	uint16_t col;
	uint16_t step;

	switch (final) {
		case 'J':
			/* 2026-05-17: restored real clear now caches are on.
			 * param0=0 (default) = cursor-to-end-of-display;
			 * param0=2 = entire screen. */
			if (param0 == 2u) {
				pl011_fbcon_clearAll(uart);
			}
			else {
				/* Cursor-to-end: clear current row from cursor to
				 * EOL, then all rows below. */
				uint16_t r;
				for (r = uart->fbrow; r < uart->fbrows; ++r) {
					pl011_fbcon_clearRow(uart, r);
				}
			}
			break;

		case 'K':
			/* 2026-05-17: restored real line clear now caches are on.
			 * For simplicity we always clear the entire current row;
			 * psh's typical \x1b[K means "to end of line" but the
			 * cells past the cursor are already empty in practice. */
			pl011_fbcon_clearRow(uart, uart->fbrow);
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

		case 'm': {
			/* SGR (#49): set foreground/background colour + bold. ESC[m with
			 * no params == reset. Iterate every parameter so compound
			 * sequences like ESC[1;32m (bold + green) apply in order. */
			uint8_t np = (uart->fbescNumParams > 0u) ? uart->fbescNumParams : 1u;
			uint8_t k;
			for (k = 0u; k < np; ++k) {
				uint16_t p = (k < uart->fbescNumParams) ? uart->fbescParams[k] : 0u;
				if (p == 0u) { /* reset */
					uart->fbfg = PL011_FBCON_FG;
					uart->fbbg = PL011_FBCON_BG;
					uart->fbbold = 0u;
				}
				else if (p == 1u) { /* bold -> brighten fg */
					uart->fbbold = 1u;
				}
				else if (p == 22u) { /* normal intensity */
					uart->fbbold = 0u;
				}
				else if (p == 7u) { /* reverse video: swap fg/bg */
					uint32_t t = uart->fbfg;
					uart->fbfg = uart->fbbg;
					uart->fbbg = t;
				}
				else if ((p >= 30u) && (p <= 37u)) {
					uart->fbfg = pl011_fbcon_palette[(p - 30u) + (uart->fbbold ? 8u : 0u)];
				}
				else if (p == 39u) { /* default fg */
					uart->fbfg = PL011_FBCON_FG;
				}
				else if ((p >= 40u) && (p <= 47u)) {
					uart->fbbg = pl011_fbcon_palette[p - 40u];
				}
				else if (p == 49u) { /* default bg */
					uart->fbbg = PL011_FBCON_BG;
				}
				else if ((p >= 90u) && (p <= 97u)) {
					uart->fbfg = pl011_fbcon_palette[(p - 90u) + 8u];
				}
				else if ((p >= 100u) && (p <= 107u)) {
					uart->fbbg = pl011_fbcon_palette[(p - 100u) + 8u];
				}
				/* 38/48 (256-colour / truecolour) + others: consumed. */
			}
			break;
		}

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
	/* Erase the old cursor before drawing so output never fights the XOR (#50). */
	if (uart->fbcurShown != 0u) {
		pl011_fbcon_xorCursor(uart, uart->fbcurCol, uart->fbcurRow);
		uart->fbcurShown = 0u;
	}
	for (i = 0u; i < size; ++i) {
		pl011_fbcon_putc(uart, (unsigned char)data[i]);
	}
	/* Redraw the cursor at the new live position. */
	uart->fbcurCol = uart->fbcol;
	uart->fbcurRow = uart->fbrow;
	pl011_fbcon_xorCursor(uart, uart->fbcurCol, uart->fbcurRow);
	uart->fbcurShown = 1u;
	mutexUnlock(uart->fbLock);
}


/* FBCONSETMODE: switch the HDMI console between text mode (FBCON_ENABLED, drawing to the
 * real framebuffer) and "graphics mode" (FBCON_DISABLED, where a full-screen app owns the
 * framebuffer and console output is diverted to an off-screen shadow). The DOS-style
 * round-trip: on DISABLE we snapshot the current screen into the shadow and keep rendering
 * klog/psh text there (nothing is lost); on ENABLE we blit the shadow back, so the user
 * returns to the text console with all output that arrived while it was hidden. */
static int pl011_fbcon_setmode(pl011_t *uart, int mode)
{
	if (uart->fbaddr == NULL) {
		return -ENODEV;
	}
	if ((mode != FBCON_ENABLED) && (mode != FBCON_DISABLED)) {
		return -EINVAL;
	}

	mutexLock(uart->fbLock);

	if (mode == uart->fbmode) {
		mutexUnlock(uart->fbLock);
		return EOK;
	}

	if (mode == FBCON_DISABLED) {
		if (uart->fbshadow == NULL) {
			uart->fbshadow = malloc(uart->fbmemsz);
			if (uart->fbshadow == NULL) {
				mutexUnlock(uart->fbLock);
				return -ENOMEM;
			}
		}
		/* Seed the shadow with what is currently on screen so continued output
		 * appends to the visible console rather than to a blank surface. */
		memcpy(uart->fbshadow, (const void *)uart->fbaddr, uart->fbmemsz);
		uart->fbdraw = (volatile uint32_t *volatile)uart->fbshadow;
		uart->fbmode = FBCON_DISABLED;
	}
	else {
		/* Restore the accumulated console image to the real framebuffer and resume
		 * drawing directly to it. */
		if (uart->fbshadow != NULL) {
			memcpy((void *)uart->fbaddr, uart->fbshadow, uart->fbmemsz);
		}
		uart->fbdraw = uart->fbaddr;
		uart->fbmode = FBCON_ENABLED;
	}

	/* Hand the USB keyboard to the full-screen app on DISABLE; take it back on
	 * ENABLE. The kbd bridge thread (pl011_kbdthr) observes this and closes/reopens
	 * /dev/kbd0 so the single-opener usbkbd device can be claimed by the app. */
	uart->kbdReleased = (uart->fbmode == FBCON_DISABLED) ? 1 : 0;

	mutexUnlock(uart->fbLock);
	return EOK;
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
	/* Map the framebuffer Normal Non-Cacheable (MAP_UNCACHED /
	 * MAIR_IDX_NONCACHED) rather than Device/Strongly-Ordered: it is
	 * plain DRAM scanned out by the VC4 engine, not a peripheral
	 * register window, so Normal NC is the architecturally correct
	 * attribute. Normal NC also permits write combining, letting
	 * adjacent stores merge into burst writes for fast sequential
	 * fills, whereas a Device/S-Ordered mapping would serialise every
	 * store. */
	uart->fbaddr = mmap(NULL, uart->fbmemsz, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_UNCACHED | MAP_ANONYMOUS | MAP_PHYSMEM, -1, pctl.task.graphmode.framebuffer);
	if (uart->fbaddr == MAP_FAILED) {
		uart->fbaddr = NULL;
		return -ENOMEM;
	}

	uart->fbpitch = pctl.task.graphmode.pitch;
	uart->fbcols = pctl.task.graphmode.width / TTYPC_FBFONT_W;
	uart->fbrows = pctl.task.graphmode.height / TTYPC_FBFONT_H;

	/* Console starts in text mode, drawing directly to the real framebuffer. */
	uart->fbmode = FBCON_ENABLED;
	uart->fbdraw = uart->fbaddr;
	uart->fbshadow = NULL;

	/* 2026-05-17: restored full-framebuffer clear at init now that
	 * caches are operational (armstub fix 1319367 + L2CTLR_EL1).
	 * With caches on the per-pixel store cost is negligible vs the
	 * cache-off era when 786K stores took >>10 minutes. Clearing
	 * here matches the original Phoenix-RTOS pattern on other
	 * platforms and removes the firmware splash background before
	 * text rendering begins. */
	uart->fbescState = pl011_fbcon_esc_normal;
	uart->fbescNumParams = 0u;
	/* Default colours + no cursor yet (the struct is zero-initialised, so the
	 * fg/bg MUST be set here or drawChar would render black-on-black). */
	uart->fbfg = PL011_FBCON_FG;
	uart->fbbg = PL011_FBCON_BG;
	uart->fbbold = 0u;
	uart->fbcurShown = 0u;
	uart->fbcol = 0u;
	uart->fbrow = 0u;
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

	err = -ENODEV;
	/* TODO(TD-14-pl011-retry): originally 50 retries (5 s wall), bumped
	 * to 500 (50 s) for slow Pi 4 IPC, then 30 (3 s) once the caller
	 * was made TD-14-tty0-nonfatal, then 5 here (~500 ms nominal but
	 * each lookup IPC itself can stretch 1 ms to 43 s per TD-14, so the
	 * fewer retries the better when /dev/tty0 is optional anyway).
	 * Restore to 50 after IPC slowness is rooted out and the non-fatal
	 * hack is reverted. */
	for (i = 0; i < 5; ++i) {
		err = lookup("devfs", NULL, &odev);
		if (err >= 0) {
			break;
		}

		usleep(100000);
	}

	if (err < 0) {
		pl011_writeRaw(uart, "pl011-tty: tty0 lookup failed\r\n");
		return err;
	}

	msg.type = mtCreate;
	msg.oid = odev;
	msg.i.create.dev = uart->oid;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0666;
	msg.i.data = name;
	msg.i.size = sizeof(name);

	if (msgSend(odev.port, &msg) != EOK) {
		pl011_writeRaw(uart, "pl011-tty: tty0 send failed\r\n");
		return -ENOMEM;
	}

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


/* TD-14/#127 split-sink klog->fbcon drain.
 *
 * The kernel klog ring is served by the well-known kernel port (port 0, id 0 —
 * created during kernel init before any userspace; this is the same oid the
 * kernel mirror and libklog target). We attach to it DIRECTLY by message rather
 * than resolving "/dev/kmsg" through devfs, whose bind/lookup is slow and
 * fragile on Pi 4 (TD-14) — that path left the klog drain permanently
 * unattached, so the kernel boot log never reached the HDMI fbcon.
 *
 * Sink split: the kernel UART already carries the complete klog via its
 * permanent mirror (kernel log.c). This drain therefore writes the ring's bytes
 * to the HDMI fbcon ONLY (pl011_fbcon_write, never libtty/UART), so the mirror
 * and this drain never double up on the UART. On attach (mtOpen) the kernel
 * sets our reader's ridx to the ring head, so we replay the whole pre-attach
 * backlog to fbcon — i.e. HDMI gets the full early-boot log too. */
#define KLOG_OID_PORT 0u
#define KLOG_OID_ID   0u

/* Task #31 — logging build mode (defined in board_config.h; 0 if the board does
 * not opt in). When 1 (USER mode) the klog drain below keeps reading the ring
 * but does NOT paint it to the HDMI fbcon, so the verbose kernel log no longer
 * floods the screen (rpi4-klogd captures it to /var/log/messages instead). At 0
 * (DEBUG, default) the fbcon shows the full klog exactly as today. */
#ifndef RPI4_LOG_TO_FILE
#define RPI4_LOG_TO_FILE 0
#endif

static void pl011_klogthr(void *arg)
{
	pl011_t *uart = (pl011_t *)arg;
	oid_t klog = { .port = KLOG_OID_PORT, .id = KLOG_OID_ID };
	char buf[256];
	msg_t msg;
	int rc;

	/* Register as a blocking klog reader: mtOpen(O_RDONLY) -> log_readerAdd. */
	memset(&msg, 0, sizeof(msg));
	msg.type = mtOpen;
	msg.oid = klog;
	msg.i.openclose.flags = O_RDONLY;
	rc = msgSend(klog.port, &msg);
	fprintf(stderr, "pl011-tty: klog attach rc=%d err=%d\n", rc, msg.o.err);
	if ((rc < 0) || (msg.o.err < 0)) {
		endthread();
	}

	for (;;) {
		memset(&msg, 0, sizeof(msg));
		msg.type = mtRead;
		msg.oid = klog;
		msg.o.data = buf;
		msg.o.size = sizeof(buf);

		rc = msgSend(klog.port, &msg);
		if (rc < 0) {
			usleep(PL011_TTY_KBD_RETRY_US);
			continue;
		}

		if (msg.o.err > 0) {
#if !RPI4_LOG_TO_FILE
			/* fbcon-only: UART is covered by the kernel mirror. In USER mode
			 * (RPI4_LOG_TO_FILE) the klog is captured to /var/log/messages by
			 * rpi4-klogd and is not painted to the HDMI console; we still drain
			 * the ring (the read above) so the kernel reader does not back up. */
			pl011_fbcon_write(uart, buf, (size_t)msg.o.err);
#else
			(void)uart;
#endif
		}
		else if (msg.o.err < 0) {
			/* -EPIPE: fell behind, ring wrapped; kernel reset us to head. */
			usleep(PL011_TTY_KBD_RETRY_US);
		}
		/* msg.o.err == 0: a blocking read returned no data; just loop. */
	}
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

	/* No HDMI console until fbcon_init succeeds (it sets FBCON_ENABLED). */
	uart->fbmode = FBCON_UNSUPPORTED;
	uart->fbdraw = NULL;
	uart->fbshadow = NULL;

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
#if defined(__CPU_GENERIC)
	else if (req == FBCONSETMODE) {
		err = pl011_fbcon_setmode(&pl011_common.uart, (int)(intptr_t)idata);
	}
	else if (req == FBCONGETMODE) {
		odata = (const void *)&pl011_common.uart.fbmode;
		err = EOK;
	}
#endif
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

		/* TD-12 boot-speed fix (2026-05-17, refined 2026-05-18):
		 * pop up to 64 bytes from libtty TX into a local buffer, write
		 * them all to PL011 DR, then call pl011_fbcon_write once for
		 * the whole batch. Batching the fbcon mirror amortises the
		 * mutexLock/Unlock pair across many bytes; per-byte pacing
		 * here was 10 µs/byte and back-pressured the libtty queue
		 * under usb-daemon's printf flood.
		 *
		 * Critically, this routine pops only ONE batch per outer
		 * pl011_thr iteration, then yields back to the for(;;) loop
		 * so the RX-FIFO drain at the top runs again. An earlier
		 * variant drained the entire libtty TX queue before yielding,
		 * which meant up to ~2 s of TX work (klog backlog right after
		 * boot) before RX was serviced — and at 16-byte PL011 RX FIFO
		 * vs slow servicing, typed characters from the serial console
		 * trickled in seconds late. With one-batch-per-iter the worst
		 * case RX-to-libtty latency is one batch of UART writes
		 * (~64 × 87 µs = 6 ms) plus one fbcon batch. The bottom
		 * `wake_writer / wake_reader / idle-sleep` block still gates
		 * the usleep on libtty_txready, so we keep tight-polling
		 * while TX has more data — i.e. we drain the same total
		 * volume, just with RX servicing interleaved. */
		{
			char batch[64];
			size_t n = 0u;

			while ((n < sizeof(batch)) &&
				(libtty_txready(&uart->tty) != 0) &&
				((pl011_read(uart, fr) & fr_txff) == 0)) {
				batch[n] = (char)libtty_popchar(&uart->tty);
				pl011_write(uart, dr, (unsigned char)batch[n]);
				++n;
			}
			if (n != 0u) {
				if (uart->fbaddr != NULL) {
					pl011_fbcon_write(uart, batch, n);
				}
				wake_writer = 1;
			}
		}

		if (wake_reader != 0) {
			libtty_wake_reader(&uart->tty);
		}

		if (wake_writer != 0) {
			libtty_wake_writer(&uart->tty);
		}

		/* TD-15 Stage 4 phase 1h: only sleep when there was nothing
		 * to do AND there is nothing pending. With caches disabled
		 * (Stage 1 parked) usleep can stretch to many seconds (TD-14
		 * timing variance documents `proc_send` round trips of 1 ms
		 * to 43 s on the same hardware), so a fixed-period sleep
		 * after every drain causes pl011_thr to miss whole bursts of
		 * libtty output. Tight-polling while work exists ensures
		 * psh prompts and klog content flow to UART + fbcon without
		 * being held in the libtty buffer. The inner loops both
		 * already drain to completion, so once we exit them we
		 * really are idle. */
		if ((wake_reader == 0) && (wake_writer == 0) &&
			(libtty_txready(&uart->tty) == 0) &&
			((pl011_read(uart, fr) & fr_rxfe) != 0)) {
			usleep(PL011_TTY_POLL_US);
		}
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

	fd = -1;
	for (;;) {
		int wake_reader = 0;
		size_t i;

		/* A full-screen app owns the keyboard: release /dev/kbd0 (single-opener
		 * usbkbd) so it can open it, and stop reopening until the console returns. */
		if (uart->kbdReleased != 0) {
			if (fd >= 0) {
				close(fd);
				fd = -1;
			}
			usleep(PL011_TTY_KBD_POLL_US);
			continue;
		}

		if (fd < 0) {
			/* O_NONBLOCK so this thread stays responsive to kbdReleased rather than
			 * blocking forever in read() (which a Phoenix msg-read can't be
			 * interrupted out of). */
			fd = open(path, O_RDONLY | O_NONBLOCK);
			if (fd < 0) {
				usleep(PL011_TTY_KBD_RETRY_US);
				continue;
			}
			/* TODO(#127): bring-up observability — the open succeeding both starts
			 * the keyboard's URB polling (usbkbd opens on first client) and marks
			 * when the USB keyboard became usable relative to boot. */
			fprintf(stderr, "pl011-tty: kbd bridge opened %s\n", path);
		}

		len = read(fd, buf, sizeof(buf));
		if (len < 0) {
			if (errno == EWOULDBLOCK || errno == EAGAIN || errno == EINTR) {
				usleep(PL011_TTY_KBD_POLL_US);   /* no data this tick */
				continue;
			}
			close(fd);   /* real error (e.g. device gone) — reopen */
			fd = -1;
			usleep(PL011_TTY_KBD_RETRY_US);
			continue;
		}
		if (len == 0) {
			usleep(PL011_TTY_KBD_POLL_US);
			continue;
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
		fprintf(stderr, "pl011-tty: tty0 register failed (non-fatal, continuing)\n");
	}

	if (create_dev(&pl011_common.uart.oid, _PATH_CONSOLE) < 0) {
		fprintf(stderr, "pl011-tty: failed to register %s\n", _PATH_CONSOLE);
		return EXIT_FAILURE;
	}

	/* TODO(TD-14-console-alias): keep the direct kernel namespace alias
	 * until Pi 4 bind/devfs lookup latency is fixed. create_dev() registers
	 * the node in devfs; this alias preserves the fast /dev/console path used
	 * by early shell startup and mirrors create_dev()'s fallback behavior. */
	(void)portRegister(port, _PATH_CONSOLE, &pl011_common.uart.oid);

	/* TD-14/#127: initialize the HDMI fbcon (which sets uart->fbaddr) BEFORE
	 * registering the klog callback and BEFORE starting pl011_thr, so every
	 * console byte (kernel klog + psh output) is mirrored to HDMI from the very
	 * first drained batch. Previously fbcon_init ran AFTER pl011_thr started, so
	 * bytes drained during the init window reached UART but not fbcon — a
	 * non-deterministic "fbcon shows only the banner/psh prompt, no klog"
	 * depending on how much console output drained before fbaddr was set. */
	{
		int fbres = pl011_fbcon_init(&pl011_common.uart);
		if (fbres == EOK) {
			pl011_writeRaw(&pl011_common.uart, "fbcon: ok\r\n");
		}
		else {
			fprintf(stderr, "pl011-tty: fbcon init failed: %d\n", fbres);
		}
	}

	/* klog -> HDMI fbcon. Direct-attach to the kernel log port (see
	 * pl011_klogthr); replaces libklog's devfs-path drain, which never
	 * attached on Pi 4. The kernel UART mirror covers the UART, so this is
	 * fbcon-only. */
	beginthread(pl011_klogthr, 4, pl011_common.uart.klogstack, sizeof(pl011_common.uart.klogstack), &pl011_common.uart);

	beginthread(pl011_thr, 4, pl011_common.uart.stack, sizeof(pl011_common.uart.stack), &pl011_common.uart);
	if (PL011_TTY_KBD_PATH != NULL) {
		beginthread(pl011_kbdthr, 4, pl011_common.uart.kbdstack, sizeof(pl011_common.uart.kbdstack), &pl011_common.uart);
	}
	beginthread(poolthr, 4, pl011_common.stack, sizeof(pl011_common.stack), (void *)(uintptr_t)port);

	poolthr((void *)(uintptr_t)port);

	return EOK;
}
