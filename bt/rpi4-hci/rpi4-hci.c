/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM43455) Bluetooth HCI device
 *
 * Brings up the BCM43455 Bluetooth controller and exposes it as /dev/hci0, a
 * raw H4 HCI byte stream:
 *   write() - an H4 HCI packet (type byte + payload) is sent to the controller
 *   read()  - the next available H4 bytes from the controller (events/ACL)
 *
 * The BCM43455 BT side is a UART HCI controller, but the VideoCore firmware
 * leaves its UART pins unrouted, so this driver routes BT to the AUX mini-UART
 * itself (PL011 stays the debug console): GPIO30-33 -> ALT5, mini-UART up at
 * 115200 (baud divisor from the CORE clock via mailbox), BT_REG_ON raised via
 * the expander (mailbox GPIO 128). At startup it resets the controller and
 * uploads the patch-RAM firmware (.hcd, loaded from a file at runtime so the
 * Cypress-EULA blob stays out of the tree), then serves /dev/hci0.
 *
 * A background RX thread continuously drains the 8-byte mini-UART FIFO into a
 * ring buffer, so HCI events are not lost between a client's discrete read()s.
 *
 * This productionizes tools/bt-probe into a resident driver (T-WIFI-BT: make
 * Bluetooth a first-class Phoenix citizen). See
 * docs/inprogress/2026-08-10-wifi-bt-first-class-design.md.
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
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <posix/utils.h>


/* ---- VideoCore mailbox (property channel) ------------------------------- */
#define RPI_MAILBOX_BASE       0xfe00b880u
#define VC_MBOX_READ           0x00u
#define VC_MBOX_STATUS         0x18u
#define VC_MBOX_WRITE          0x20u
#define VC_MBOX_STATUS_FULL    0x80000000u
#define VC_MBOX_STATUS_EMPTY   0x40000000u
#define VC_MBOX_RESP_OK        0x80000000u
#define VC_MBOX_PROP_CHANNEL   8u
#define VC_PROP_SET_GPIO_STATE 0x00038041u
#define VC_PROP_GET_GPIO_STATE 0x00030041u
#define VC_PROP_GET_CLOCK_RATE 0x00030002u
#define VC_CLK_CORE            4u
#define EXPGPIO_BT_ON          128u

/* ---- BCM2711 GPIO + AUX mini-UART --------------------------------------- */
#define GPIO_BASE     0xfe200000u
#define GPIO_FN_ALT5  2u

#define AUX_BASE      0xfe215000u
#define AUX_ENABLES   0x04u
#define AUX_MU_IO     0x40u
#define AUX_MU_IER    0x44u
#define AUX_MU_IIR    0x48u
#define AUX_MU_LCR    0x4Cu
#define AUX_MU_MCR    0x50u
#define AUX_MU_LSR    0x54u
#define AUX_MU_CNTL   0x60u
#define AUX_MU_BAUD   0x68u
#define LSR_RX_RDY    (1u << 0)
#define LSR_TX_EMPTY  (1u << 5)

#define HCI_RING_SZ   8192u
#define DEFAULT_HCD   "/etc/bluetooth/BCM4345C0.hcd"


static struct {
	volatile uint8_t *gpio;
	volatile uint8_t *aux;
	handle_t uartLock; /* serializes mini-UART register access (TX vs RX drain) */
	handle_t ringLock; /* guards the RX ring */
	uint8_t ring[HCI_RING_SZ];
	unsigned rhead; /* producer (RX thread) */
	unsigned rtail; /* consumer (mtRead) */
	char rxStack[4096] __attribute__((aligned(8)));
	char msgStack[4096] __attribute__((aligned(8)));
} hci_common;


/* ---- mailbox ------------------------------------------------------------ */
static uint32_t hci_mbox(uint32_t tag, uint32_t arg0, uint32_t arg1)
{
	addr_t pa_base = (addr_t)RPI_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request, result = 0xFFFFFFFFu;
	void *mbox_page, *msg_page;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, pa_base);
	if (mbox_page == MAP_FAILED) {
		return 0xFFFFFFFFu;
	}
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}
	msg = msg_page;
	msg[0] = 32; msg[1] = 0; msg[2] = tag; msg[3] = 8; msg[4] = 0;
	msg[5] = arg0; msg[6] = arg1; msg[7] = 0;

	msg_pa = (uintptr_t)va2pa(msg);
	if (msg_pa == (uintptr_t)-1) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return 0xFFFFFFFFu;
	}
	request = ((uint32_t)msg_pa & ~0xFu) | VC_MBOX_PROP_CHANNEL;

	while ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_FULL) != 0u) {
	}
	mbox[VC_MBOX_WRITE / 4] = request;
	for (;;) {
		while ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) != 0u) {
		}
		if (mbox[VC_MBOX_READ / 4] == request) {
			break;
		}
	}
	if (msg[1] == VC_MBOX_RESP_OK) {
		result = msg[6];
	}
	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}


/* ---- GPIO + mini-UART --------------------------------------------------- */
static void gpio_fsel(unsigned pin, unsigned fn)
{
	volatile uint32_t *reg = (volatile uint32_t *)(hci_common.gpio + (pin / 10u) * 4u);
	unsigned shift = (pin % 10u) * 3u;
	uint32_t v = *reg;
	v &= ~(0x7u << shift);
	v |= ((fn & 0x7u) << shift);
	*reg = v;
}


static unsigned gpio_getfsel(unsigned pin)
{
	uint32_t v = *(volatile uint32_t *)(hci_common.gpio + (pin / 10u) * 4u);
	return (v >> ((pin % 10u) * 3u)) & 0x7u;
}


static void aux_init(uint32_t baud_reg)
{
	volatile uint8_t *a = hci_common.aux;
	*(volatile uint32_t *)(a + AUX_ENABLES) |= 1u;
	*(volatile uint32_t *)(a + AUX_MU_CNTL) = 0u;
	*(volatile uint32_t *)(a + AUX_MU_IER) = 0u;
	*(volatile uint32_t *)(a + AUX_MU_LCR) = 3u;   /* 8-bit (BCM erratum: 0x3) */
	*(volatile uint32_t *)(a + AUX_MU_MCR) = 2u;   /* assert RTS (active-low) */
	*(volatile uint32_t *)(a + AUX_MU_IIR) = 0xC6u;
	*(volatile uint32_t *)(a + AUX_MU_BAUD) = baud_reg;
	*(volatile uint32_t *)(a + AUX_MU_CNTL) = 3u;  /* enable TX+RX */
}


static int aux_putc(uint8_t c)
{
	volatile uint8_t *a = hci_common.aux;
	int d;
	for (d = 0; d < 2000000; ++d) {
		if ((*(volatile uint32_t *)(a + AUX_MU_LSR) & LSR_TX_EMPTY) != 0u) {
			*(volatile uint32_t *)(a + AUX_MU_IO) = c;
			return 0;
		}
	}
	return -1;
}


/* blocking single-char read with a timeout, for the synchronous bring-up phase */
static int aux_getc(int timeout_ms)
{
	volatile uint8_t *a = hci_common.aux;
	int t;
	for (t = 0; t < timeout_ms * 20; ++t) {
		if ((*(volatile uint32_t *)(a + AUX_MU_LSR) & LSR_RX_RDY) != 0u) {
			return (int)(*(volatile uint32_t *)(a + AUX_MU_IO) & 0xffu);
		}
		usleep(50);
	}
	return -1;
}


/* ---- synchronous HCI, used only during startup bring-up ----------------- */
static int hci_cmd(uint16_t opcode, const uint8_t *params, uint8_t plen,
	uint8_t *resp, int cap)
{
	int i, b, n = 0;
	if (aux_putc(0x01u) != 0) {
		return -1;
	}
	(void)aux_putc((uint8_t)(opcode & 0xffu));
	(void)aux_putc((uint8_t)((opcode >> 8) & 0xffu));
	(void)aux_putc(plen);
	for (i = 0; i < (int)plen; ++i) {
		(void)aux_putc(params[i]);
	}
	b = aux_getc(500);
	while (b >= 0 && n < cap) {
		resp[n++] = (uint8_t)b;
		b = aux_getc(40);
	}
	return n;
}


/* Broadcom patch-RAM upload (mirrors Linux btbcm_patchram). Reads the .hcd from
 * `path`; returns records acked, or -1 if the file is unavailable. */
static int hci_patchram(const char *path, int *out_total)
{
	uint8_t resp[16];
	uint8_t *hcd;
	long sz;
	uint32_t off = 0u;
	int ok = 0, total = 0, fd;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		return -1;
	}
	sz = lseek(fd, 0, SEEK_END);
	(void)lseek(fd, 0, SEEK_SET);
	if (sz <= 0) {
		close(fd);
		return -1;
	}
	hcd = malloc((size_t)sz);
	if (hcd == NULL || read(fd, hcd, (size_t)sz) != (ssize_t)sz) {
		free(hcd);
		close(fd);
		return -1;
	}
	close(fd);

	(void)hci_cmd(0xfc2eu, NULL, 0u, resp, sizeof(resp)); /* Download_Minidriver */
	usleep(50 * 1000);

	while (off + 3u <= (uint32_t)sz) {
		uint16_t opcode = (uint16_t)(hcd[off] | (hcd[off + 1] << 8));
		uint8_t plen = hcd[off + 2];
		int n;
		if (off + 3u + (uint32_t)plen > (uint32_t)sz) {
			break;
		}
		n = hci_cmd(opcode, &hcd[off + 3], plen, resp, sizeof(resp));
		total++;
		if (n >= 7 && resp[0] == 0x04u && resp[1] == 0x0eu && resp[6] == 0x00u) {
			ok++;
		}
		off += 3u + (uint32_t)plen;
	}
	usleep(250 * 1000);
	free(hcd);
	if (out_total != NULL) {
		*out_total = total;
	}
	return ok;
}


/* ---- RX ring ------------------------------------------------------------ */
static void ring_push(const uint8_t *buf, int n)
{
	int i;
	mutexLock(hci_common.ringLock);
	for (i = 0; i < n; ++i) {
		unsigned next = (hci_common.rhead + 1u) % HCI_RING_SZ;
		if (next == hci_common.rtail) {
			break; /* full: drop (a stalled reader shouldn't wedge RX) */
		}
		hci_common.ring[hci_common.rhead] = buf[i];
		hci_common.rhead = next;
	}
	mutexUnlock(hci_common.ringLock);
}


static int ring_pop(uint8_t *dst, int cap)
{
	int n = 0;
	mutexLock(hci_common.ringLock);
	while (n < cap && hci_common.rtail != hci_common.rhead) {
		dst[n++] = hci_common.ring[hci_common.rtail];
		hci_common.rtail = (hci_common.rtail + 1u) % HCI_RING_SZ;
	}
	mutexUnlock(hci_common.ringLock);
	return n;
}


/* Continuously drain the mini-UART FIFO into the ring (8-byte HW FIFO would
 * otherwise overflow between a client's discrete read()s). */
static void hci_rxThread(void *arg)
{
	volatile uint8_t *a = hci_common.aux;
	uint8_t tmp[64];

	(void)arg;
	for (;;) {
		int n = 0;
		mutexLock(hci_common.uartLock);
		while (n < (int)sizeof(tmp) &&
			(*(volatile uint32_t *)(a + AUX_MU_LSR) & LSR_RX_RDY) != 0u) {
			tmp[n++] = (uint8_t)(*(volatile uint32_t *)(a + AUX_MU_IO) & 0xffu);
		}
		mutexUnlock(hci_common.uartLock);
		if (n > 0) {
			ring_push(tmp, n);
		}
		else {
			usleep(1000); /* idle poll */
		}
	}
}


/* ---- /dev/hci0 message loop --------------------------------------------- */
static int hci_write(const uint8_t *src, int n)
{
	int i;
	mutexLock(hci_common.uartLock);
	for (i = 0; i < n; ++i) {
		if (aux_putc(src[i]) != 0) {
			mutexUnlock(hci_common.uartLock);
			return (i > 0) ? i : -EIO;
		}
	}
	mutexUnlock(hci_common.uartLock);
	return n;
}


static void hci_thread(void *arg)
{
	uint32_t port = (uint32_t)(uintptr_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err, n;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
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

			case mtRead:
				n = ring_pop((uint8_t *)msg.o.data, (int)msg.o.size);
				if (n == 0) {
					usleep(2000); /* no event pending: throttle the poll */
				}
				msg.o.err = n; /* non-blocking: 0 == "no event yet, retry" */
				break;

			case mtWrite:
				msg.o.err = hci_write((const uint8_t *)msg.i.data, (int)msg.i.size);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0600;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


/* Acceptance self-test: act as a /dev/hci0 CLIENT (open/write/read) to prove
 * the device relays HCI both directions. First a deterministic HCI_RESET
 * round-trip (write cmd -> read its Command Complete), then an Inquiry scan for
 * demonstration. This is a temporary in-driver harness for the pre-boot-
 * integration increment; the real client is btctl. */
static int hci_selftest(void)
{
	static const uint8_t reset[] = { 0x01, 0x03, 0x0c, 0x00 };
	static const uint8_t inquiry[] = { 0x01, 0x01, 0x04, 0x05, 0x33, 0x8b, 0x9e, 0x08, 0x00 };
	uint8_t buf[256];
	int fd, i, ticks, got_cc = 0, devs = 0;
	int st = 0, code = 0, plen = 0, pi = 0;
	uint8_t pp[256];

	fd = open("/dev/hci0", O_RDWR);
	if (fd < 0) {
		printf("rpi4-hci selftest: open(/dev/hci0) failed\n");
		return -1;
	}

	/* 1. HCI_RESET round-trip through the device node. */
	(void)write(fd, reset, sizeof(reset));
	for (ticks = 0; ticks < 100 && got_cc == 0; ++ticks) {
		int n = (int)read(fd, buf, sizeof(buf));
		if (n <= 0) {
			usleep(10 * 1000);
			continue;
		}
		for (i = 0; i + 6 < n; ++i) {
			/* Command Complete for HCI_RESET: 04 0e .. .. 03 0c 00 */
			if (buf[i] == 0x04 && buf[i + 1] == 0x0e && buf[i + 4] == 0x03 &&
				buf[i + 5] == 0x0c && buf[i + 6] == 0x00) {
				got_cc = 1;
				break;
			}
		}
	}
	printf("rpi4-hci selftest: HCI_RESET via /dev/hci0 -> %s\n",
		got_cc ? "Command Complete received (device relays HCI both ways)" : "NO reply");

	/* 2. Inquiry scan (demonstration; 0 devices is fine if none discoverable). */
	(void)write(fd, inquiry, sizeof(inquiry));
	st = 0;
	for (ticks = 0; ticks < 1200; ++ticks) {
		int n = (int)read(fd, buf, sizeof(buf));
		if (n <= 0) {
			usleep(10 * 1000);
			continue;
		}
		for (i = 0; i < n; ++i) {
			uint8_t c = buf[i];
			switch (st) {
				case 0: st = (c == 0x04) ? 1 : 0; break;
				case 1: code = c; st = 2; break;
				case 2: plen = c; pi = 0; st = (plen > 0) ? 3 : 0; break;
				case 3:
					pp[pi++] = c;
					if (pi >= plen) {
						if (code == 0x01) {
							printf("  Inquiry Complete (status=0x%02x)\n", pp[0]);
							ticks = 1200; /* done */
						}
						else if ((code == 0x02 || code == 0x22) && plen > 0) {
							int num = pp[0], d;
							for (d = 0; d < num && (1 + d * 6 + 6) <= plen; ++d) {
								devs++;
							}
						}
						st = 0;
					}
					break;
				default: st = 0; break;
			}
		}
	}
	printf("rpi4-hci selftest: Inquiry saw %d device sighting(s)\n", devs);
	close(fd);
	printf("rpi4-hci selftest: %s\n", got_cc ? "PASS" : "FAIL");
	return got_cc ? 0 : 1;
}


static void hci_sigExit(int sig)
{
	(void)sig;
	_exit(0);
}


/* Full bring-up: power + route + reset + patchram, then register /dev/hci0 and
 * start the RX + message threads. Returns 0 on success (the device is live and
 * served by threads), or a nonzero code on failure. */
static int hci_bringup(const char *hcd)
{
	uint32_t port, core_hz, baud_reg;
	oid_t dev;
	uint8_t resp[64];
	int n, total = 0, ok;

	printf("rpi4-hci: BCM43455 Bluetooth HCI bring-up (hcd=%s)\n", hcd);

	hci_common.gpio = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, GPIO_BASE);
	hci_common.aux = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, AUX_BASE);
	if (hci_common.gpio == MAP_FAILED || hci_common.aux == MAP_FAILED) {
		printf("rpi4-hci: mmap failed\n");
		return 1;
	}

	core_hz = hci_mbox(VC_PROP_GET_CLOCK_RATE, VC_CLK_CORE, 0u);
	if (core_hz == 0u || core_hz == 0xFFFFFFFFu) {
		core_hz = 500000000u;
		printf("rpi4-hci: GET_CLOCK_RATE(core) failed; assuming %u Hz\n", core_hz);
	}
	baud_reg = core_hz / (8u * 115200u);
	if (baud_reg > 0u) {
		baud_reg -= 1u;
	}
	printf("rpi4-hci: core_clk=%u Hz -> AUX_MU_BAUD=%u\n", core_hz, baud_reg);

	/* Power the BT core on, route its UART to the mini-UART, bring it up. */
	(void)hci_mbox(VC_PROP_SET_GPIO_STATE, EXPGPIO_BT_ON, 0u);
	usleep(50 * 1000);
	(void)hci_mbox(VC_PROP_SET_GPIO_STATE, EXPGPIO_BT_ON, 1u);
	usleep(500 * 1000);
	printf("rpi4-hci: BT_REG_ON readback=%d (expect 1)\n",
		(int)hci_mbox(VC_PROP_GET_GPIO_STATE, EXPGPIO_BT_ON, 0u));

	gpio_fsel(30, GPIO_FN_ALT5);
	gpio_fsel(31, GPIO_FN_ALT5);
	gpio_fsel(32, GPIO_FN_ALT5);
	gpio_fsel(33, GPIO_FN_ALT5);
	aux_init(baud_reg);
	printf("rpi4-hci: routing GPIO30-33 fsel = %u %u %u %u (2=ALT5)\n",
		gpio_getfsel(30), gpio_getfsel(31), gpio_getfsel(32), gpio_getfsel(33));
	usleep(50 * 1000); /* let the mini-UART + chip settle before first HCI */
	while (aux_getc(5) >= 0) {
	}

	n = hci_cmd(0x0c03u, NULL, 0u, resp, sizeof(resp)); /* HCI_RESET */
	printf("rpi4-hci: HCI_RESET reply (%d):", n);
	{
		int _i;
		for (_i = 0; _i < n && _i < 12; ++_i) {
			printf(" %02x", resp[_i]);
		}
		printf("\n");
	}
	if (n < 7 || resp[0] != 0x04u || resp[6] != 0x00u) {
		printf("rpi4-hci: HCI_RESET failed -- aborting\n");
		return 2;
	}
	printf("rpi4-hci: controller alive (HCI_RESET ok)\n");

	ok = hci_patchram(hcd, &total);
	if (ok < 0) {
		printf("rpi4-hci: WARNING no .hcd at %s -- running unpatched ROM\n", hcd);
	}
	else {
		printf("rpi4-hci: patchram %d/%d records acked\n", ok, total);
		(void)hci_cmd(0x0c03u, NULL, 0u, resp, sizeof(resp)); /* post-patch reset */
		usleep(100 * 1000);
	}

	n = hci_cmd(0x1009u, NULL, 0u, resp, sizeof(resp)); /* READ_BD_ADDR */
	if (n >= 13 && resp[0] == 0x04u && resp[6] == 0x00u) {
		const uint8_t *b = &resp[7];
		printf("rpi4-hci: BD_ADDR %02x:%02x:%02x:%02x:%02x:%02x\n",
			b[5], b[4], b[3], b[2], b[1], b[0]);
	}

	/* Drain any bring-up leftovers before the RX thread + clients take over. */
	while (aux_getc(5) >= 0) {
	}

	if (mutexCreate(&hci_common.uartLock) != EOK || mutexCreate(&hci_common.ringLock) != EOK) {
		printf("rpi4-hci: mutexCreate failed\n");
		return 3;
	}
	if (beginthread(hci_rxThread, 3, hci_common.rxStack, sizeof(hci_common.rxStack), NULL) != EOK) {
		printf("rpi4-hci: RX thread failed\n");
		return 4;
	}

	if (portCreate(&port) != EOK) {
		printf("rpi4-hci: portCreate failed\n");
		return 5;
	}
	dev.port = port;
	dev.id = 0;
	if (create_dev(&dev, "hci0") < 0) {
		printf("rpi4-hci: could not create /dev/hci0\n");
		return 6;
	}
	printf("rpi4-hci: registered /dev/hci0 (raw H4 HCI byte stream)\n");

	/* Serve /dev/hci0 in a thread. */
	if (beginthread(hci_thread, 3, hci_common.msgStack, sizeof(hci_common.msgStack),
			(void *)(uintptr_t)port) != EOK) {
		printf("rpi4-hci: msg thread failed\n");
		return 7;
	}

	return 0;
}


int main(int argc, char **argv)
{
	const char *hcd = DEFAULT_HCD;
	int selftest = 0, ai, rc;
	pid_t pid;

	/* args: [hcd-path] [selftest]  (order-independent) */
	for (ai = 1; ai < argc; ++ai) {
		if (strcmp(argv[ai], "selftest") == 0) {
			selftest = 1;
		}
		else {
			hcd = argv[ai];
		}
	}

	if (selftest != 0) {
		/* Single-process acceptance harness: bring up + be our own client. */
		rc = hci_bringup(hcd);
		if (rc == 0) {
			rc = hci_selftest();
		}
		usleep(100 * 1000); /* let the log flush */
		return rc;
	}

	/* Resident daemon: fork so the shell returns once /dev/hci0 is up while the
	 * child keeps serving (canonical Phoenix pattern, cf. flashsrv.c). */
	signal(SIGUSR1, hci_sigExit);
	pid = fork();
	if (pid < 0) {
		printf("rpi4-hci: fork failed\n");
		return 1;
	}
	if (pid > 0) {
		(void)sleep(10); /* wait to be signalled by the child, then give up */
		return 1;
	}

	/* child: bring up, tell the parent, then serve forever */
	signal(SIGUSR1, hci_sigExit);
	(void)setsid();
	rc = hci_bringup(hcd);
	if (rc != 0) {
		return rc;
	}
	kill(getppid(), SIGUSR1);
	for (;;) {
		usleep(1000 * 1000); /* the RX + msg threads do the work */
	}
	return 0;
}
