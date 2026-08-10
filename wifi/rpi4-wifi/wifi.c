/*
 * Phoenix-RTOS — wifi: WiFi control client for /dev/wifi.
 *
 * `wifi scan` asks the rpi4-wifi driver to scan the air and prints the
 * discovered access points — the acceptance test that the /dev/wifi driver
 * drives the BCM43455 radio. It writes the literal "scan" to /dev/wifi (which
 * triggers an active escan in the driver) and then reads back the AP list text.
 *
 * First cut: a standalone /bin tool. It will become a psh applet
 * (phoenix-rtos-utils/psh/wifi) once the driver is boot-integrated.
 *
 * Copyright 2026 Phoenix Systems
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define WIFI_DEV "/dev/wifi"

static int cmd_scan(int fd)
{
	static const char scan[] = "scan";
	char buf[512];
	int total = 0;
	ssize_t n;

	printf("wifi: scanning (this takes ~10-20s while the radio sweeps channels)...\n");

	/* The write triggers the escan in the driver and returns once it finishes;
	 * the AP list is then available to read back. */
	if (write(fd, scan, sizeof(scan) - 1) != (ssize_t)(sizeof(scan) - 1)) {
		printf("wifi: write(scan) failed\n");
		return 1;
	}

	/* The write advanced the fd offset by 4; reset to 0 so the AP-list read
	 * starts at the beginning (else the first 4 chars are skipped). */
	(void)lseek(fd, 0, SEEK_SET);

	for (;;) {
		n = read(fd, buf, sizeof(buf));
		if (n <= 0) {
			break;
		}
		(void)fwrite(buf, 1, (size_t)n, stdout);
		total += (int)n;
	}
	fflush(stdout);

	if (total == 0) {
		printf("wifi: scan returned no data\n");
	}
	return 0;
}

int main(int argc, char **argv)
{
	int fd, rc;

	if (argc < 2 || strcmp(argv[1], "scan") != 0) {
		printf("usage: wifi scan\n");
		return 2;
	}
	fd = open(WIFI_DEV, O_RDWR);
	if (fd < 0) {
		printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
		return 1;
	}
	rc = cmd_scan(fd);
	close(fd);
	return rc;
}
