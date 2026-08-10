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

/* Write a command to /dev/wifi (which runs it synchronously) then read back +
 * print the text result. */
static int cmd_run(int fd, const char *cmd, int cmdlen)
{
	char buf[512];
	int total = 0;
	ssize_t n;

	if (write(fd, cmd, (size_t)cmdlen) != (ssize_t)cmdlen) {
		printf("wifi: write failed\n");
		return 1;
	}

	/* The write advanced the fd offset; reset to 0 so the read starts at the
	 * beginning (else the first cmdlen chars are skipped). */
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
		printf("wifi: no data returned\n");
	}
	return 0;
}

int main(int argc, char **argv)
{
	int fd, rc;
	char cmd[64];

	if (argc >= 2 && strcmp(argv[1], "scan") == 0) {
		printf("wifi: scanning (~10-20s while the radio sweeps channels)...\n");
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, "scan", 4);
		close(fd);
		return rc;
	}
	if (argc >= 3 && strcmp(argv[1], "join") == 0) {
		int n = snprintf(cmd, sizeof(cmd), "join %s", argv[2]);
		if (n < 0 || n >= (int)sizeof(cmd)) {
			printf("wifi: ssid too long\n");
			return 2;
		}
		printf("wifi: joining \"%s\" (~6s; open-network control path)...\n", argv[2]);
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, cmd, n);
		close(fd);
		return rc;
	}
	printf("usage: wifi scan | wifi join <ssid>\n");
	return 2;
}
