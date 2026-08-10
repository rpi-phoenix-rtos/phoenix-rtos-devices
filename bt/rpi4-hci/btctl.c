/*
 * Phoenix-RTOS — btctl: Bluetooth control client for /dev/hci0.
 *
 * Talks raw H4 HCI to the rpi4-hci driver. `btctl scan` sends an HCI Inquiry
 * and prints the discovered devices + Inquiry Complete — the acceptance test
 * that the /dev/hci0 driver relays HCI both directions.
 *
 * First cut: a standalone /bin tool. It will become a psh applet
 * (phoenix-rtos-utils/psh/btctl) once the driver is boot-integrated.
 *
 * Copyright 2026 Phoenix Systems
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define HCI_DEV "/dev/hci0"

/* Reassemble an H4 event stream: 0x04 <code> <plen> <plen bytes>. */
static void handle_event(uint8_t code, const uint8_t *p, int plen, int *found)
{
	if (code == 0x01) { /* Inquiry Complete */
		printf("  Inquiry Complete (status=0x%02x)\n", plen > 0 ? p[0] : 0xff);
	}
	else if (code == 0x02 || code == 0x22) { /* Inquiry Result [+RSSI] */
		int num = plen > 0 ? p[0] : 0, d;
		for (d = 0; d < num && (1 + d * 6 + 6) <= plen; ++d) {
			const uint8_t *b = &p[1 + d * 6];
			printf("  BT device: %02x:%02x:%02x:%02x:%02x:%02x\n",
				b[5], b[4], b[3], b[2], b[1], b[0]);
			(*found)++;
		}
	}
}

static int cmd_scan(int fd)
{
	/* H4: 01(cmd) 0401(Inquiry) 05(plen) 9e8b33(GIAC LAP, LE) 08(len~10s) 00(unlimited) */
	static const uint8_t inquiry[] = { 0x01, 0x01, 0x04, 0x05, 0x33, 0x8b, 0x9e, 0x08, 0x00 };
	uint8_t buf[256];
	int found = 0, ticks;
	/* event reassembly state */
	int st = 0, code = 0, plen = 0, pi = 0;
	uint8_t params[256];

	if (write(fd, inquiry, sizeof(inquiry)) != (ssize_t)sizeof(inquiry)) {
		printf("btctl: write(inquiry) failed\n");
		return 1;
	}
	printf("btctl: HCI Inquiry sent, scanning ~12s...\n");

	for (ticks = 0; ticks < 1200; ++ticks) {
		int n = (int)read(fd, buf, sizeof(buf)), i;
		if (n <= 0) {
			usleep(10 * 1000);
			continue;
		}
		for (i = 0; i < n; ++i) {
			uint8_t c = buf[i];
			switch (st) {
				case 0: st = (c == 0x04) ? 1 : 0; break;       /* H4 event type */
				case 1: code = c; st = 2; break;               /* event code */
				case 2: plen = c; pi = 0; st = (plen > 0) ? 3 : 0;
					if (plen == 0) { handle_event(code, params, 0, &found); }
					break;
				case 3:
					params[pi++] = c;
					if (pi >= plen) {
						handle_event(code, params, plen, &found);
						st = 0;
					}
					break;
				default: st = 0; break;
			}
		}
	}
	printf("btctl: scan done, %d device sighting(s)\n", found);
	return 0;
}

int main(int argc, char **argv)
{
	int fd, rc;

	if (argc < 2 || strcmp(argv[1], "scan") != 0) {
		printf("usage: btctl scan\n");
		return 2;
	}
	fd = open(HCI_DEV, O_RDWR);
	if (fd < 0) {
		printf("btctl: cannot open %s (is rpi4-hci running?)\n", HCI_DEV);
		return 1;
	}
	rc = cmd_scan(fd);
	close(fd);
	return rc;
}
