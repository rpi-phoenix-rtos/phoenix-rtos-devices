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
#define WIFI_CONF "/etc/wifi.conf"

/* Read the `ssid=` value from /etc/wifi.conf into out (INI-lite: key=value per
 * line, '#' comments, surrounding whitespace trimmed). Returns 0 on success, -1
 * if the file or key is absent. Forward-compatible: unknown keys (e.g. a future
 * psk=) are ignored, so a config can carry psk= before the driver supports it. */
static int conf_get_ssid(char *out, int outsz)
{
	FILE *f = fopen(WIFI_CONF, "r");
	char line[160];
	int got = -1;

	if (f == NULL) {
		return -1;
	}
	while (fgets(line, sizeof(line), f) != NULL) {
		char *p = line, *eq, *v, *e;
		while (*p == ' ' || *p == '\t') {
			p++;
		}
		if (*p == '#' || *p == '\0' || *p == '\n') {
			continue;
		}
		eq = strchr(p, '=');
		if (eq == NULL) {
			continue;
		}
		*eq = '\0';
		/* trim trailing space off the key */
		for (e = eq - 1; e >= p && (*e == ' ' || *e == '\t'); e--) {
			*e = '\0';
		}
		if (strcmp(p, "ssid") != 0) {
			continue;
		}
		v = eq + 1;
		while (*v == ' ' || *v == '\t') {
			v++;
		}
		/* strip trailing whitespace/newline off the value */
		for (e = v + strlen(v) - 1; e >= v && (*e == '\n' || *e == '\r' || *e == ' ' || *e == '\t'); e--) {
			*e = '\0';
		}
		if (*v != '\0') {
			int n = snprintf(out, (size_t)outsz, "%s", v);
			got = (n > 0 && n < outsz) ? 0 : -1;
		}
		break;
	}
	fclose(f);
	return got;
}

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
	if (argc >= 2 && strcmp(argv[1], "up") == 0) {
		char ssid[40];
		int n;
		if (conf_get_ssid(ssid, (int)sizeof(ssid)) != 0) {
			printf("wifi: no ssid in %s (add a line: ssid=<name>)\n", WIFI_CONF);
			return 2;
		}
		n = snprintf(cmd, sizeof(cmd), "join %s", ssid);
		if (n < 0 || n >= (int)sizeof(cmd)) {
			printf("wifi: ssid too long\n");
			return 2;
		}
		printf("wifi: bringing up \"%s\" from %s (open-network control path)...\n", ssid, WIFI_CONF);
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, cmd, n);
		close(fd);
		return rc;
	}
	printf("usage: wifi scan | wifi join <ssid> | wifi up\n");
	printf("  up: join the ssid configured in %s\n", WIFI_CONF);
	return 2;
}
