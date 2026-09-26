/*
 * Phoenix-RTOS — wifi: WiFi control client.
 *
 * Everyday use goes through the lwip `wl` netif, whose join thread keeps the
 * association in line with /etc/wifi.conf:
 *
 *   wifi scan                   list access points (asks the rpi4-wifi daemon)
 *   wifi connect <ssid> <psk>   write /etc/wifi.conf, wait for the DHCP lease
 *   wifi disconnect             remove /etc/wifi.conf; the netif leaves
 *   wifi status                 wanted network, daemon state, netif address
 *
 * The remaining commands talk to the daemon directly and bypass lwip. They are
 * bring-up diagnostics: `netup` and `join` run their own association (and
 * `netup` its own DHCP client) inside the daemon, which fights the netif if it
 * is active.
 *
 * Copyright 2026 Phoenix Systems
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define WIFI_DEV       "/dev/wifi"
#define WIFI_CONF      "/etc/wifi.conf"
#define WIFI_CONF_TMP  "/etc/wifi.conf.new"
#define WIFI_WAIT_SECS 120 /* join (20-40 s) + DHCP, plus a failed first attempt */

/* Read the `<key>=` value from /etc/wifi.conf into out (INI-lite: key=value per
 * line, '#' comments, surrounding whitespace trimmed) -- the same tolerance as
 * the netif's parser. Returns 0 on success, -1 if the file or key is absent. */
static int conf_get(const char *key, char *out, int outsz)
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
		if (strcmp(p, key) != 0) {
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


/* IPv4 address of the WiFi netif (the lwip `wl` interface), or 0 if it has
 * none yet. The netif's number depends on registration order, so match the
 * name prefix. */
static uint32_t wl_addr(char *name, size_t namesz)
{
	struct ifaddrs *list, *ifa;
	uint32_t addr = 0;

	if (getifaddrs(&list) != 0) {
		return 0;
	}
	for (ifa = list; ifa != NULL; ifa = ifa->ifa_next) {
		if ((ifa->ifa_name == NULL) || (strncmp(ifa->ifa_name, "wl", 2) != 0) ||
				(ifa->ifa_addr == NULL) || (ifa->ifa_addr->sa_family != AF_INET)) {
			continue;
		}
		addr = ((struct sockaddr_in *)ifa->ifa_addr)->sin_addr.s_addr;
		if (name != NULL) {
			(void)snprintf(name, namesz, "%s", ifa->ifa_name);
		}
		if (addr != 0) {
			break;
		}
	}
	freeifaddrs(list);
	return addr;
}


static void print_addr(const char *label, uint32_t addr, const char *ifname)
{
	char buf[INET_ADDRSTRLEN];
	struct in_addr in = { .s_addr = addr };

	printf("%s%s (%s)\n", label,
		(inet_ntop(AF_INET, &in, buf, sizeof(buf)) != NULL) ? buf : "?", ifname);
}


/* Replace /etc/wifi.conf in one step: write a new file beside it, then rename
 * over the old one, so the netif (which re-reads it every few seconds) never
 * sees half a file. */
static int conf_write(const char *ssid, const char *psk)
{
	FILE *f = fopen(WIFI_CONF_TMP, "w");

	if (f == NULL) {
		printf("wifi: cannot create %s: %s\n", WIFI_CONF_TMP, strerror(errno));
		return -1;
	}
	fprintf(f, "# Written by `wifi connect`. The WiFi netif re-reads this file;\n"
		"# `wifi disconnect` removes it.\n"
		"ssid=%s\npsk=%s\n", ssid, psk);
	if (fclose(f) != 0) {
		printf("wifi: cannot write %s: %s\n", WIFI_CONF_TMP, strerror(errno));
		(void)unlink(WIFI_CONF_TMP);
		return -1;
	}
	if (rename(WIFI_CONF_TMP, WIFI_CONF) != 0) {
		printf("wifi: cannot replace %s: %s\n", WIFI_CONF, strerror(errno));
		(void)unlink(WIFI_CONF_TMP);
		return -1;
	}
	return 0;
}


/* A WPA2 passphrase is 8..63 printable characters; 64 characters is a raw hex
 * key. Anything else the firmware would reject 30 s later, so say so now. */
static int creds_valid(const char *ssid, const char *psk)
{
	size_t sl = strlen(ssid), pl = strlen(psk);

	if ((sl == 0) || (sl > 32)) {
		printf("wifi: ssid must be 1-32 characters\n");
		return 0;
	}
	if ((pl < 8) || (pl > 64)) {
		printf("wifi: WPA2 passphrase must be 8-63 characters (or 64 hex digits)\n");
		return 0;
	}
	if ((strchr(ssid, '\n') != NULL) || (strchr(psk, '\n') != NULL)) {
		printf("wifi: ssid/passphrase may not contain a newline\n");
		return 0;
	}
	return 1;
}


static int do_connect(const char *ssid, const char *psk)
{
	char ifname[16] = "wl", old_ssid[40], old_psk[72];
	uint32_t addr;
	int t, same;

	if (!creds_valid(ssid, psk)) {
		return 2;
	}
	same = (conf_get("ssid", old_ssid, (int)sizeof(old_ssid)) == 0) &&
		(conf_get("psk", old_psk, (int)sizeof(old_psk)) == 0) &&
		(strcmp(old_ssid, ssid) == 0) && (strcmp(old_psk, psk) == 0);
	addr = wl_addr(ifname, sizeof(ifname));
	if (same && (addr != 0)) {
		print_addr("wifi: already connected, address ", addr, ifname);
		return 0;
	}

	if (conf_write(ssid, psk) != 0) {
		return 1;
	}
	printf("wifi: connecting to \"%s\" (join 20-40 s, then DHCP)...\n", ssid);

	/* A lease from the PREVIOUS network stays on the netif until its join
	 * thread notices the new file (within a few seconds), so wait for that one
	 * to go before accepting an address as the new lease. */
	for (t = 0; (t < 15) && (addr != 0); ++t) {
		sleep(1);
		addr = wl_addr(ifname, sizeof(ifname));
	}
	if (addr != 0) {
		print_addr("wifi: the netif did not leave its network, still ", addr, ifname);
		printf("      (a network named in the boot config overrides %s)\n", WIFI_CONF);
		return 1;
	}
	for (; t < WIFI_WAIT_SECS; ++t) {
		addr = wl_addr(ifname, sizeof(ifname));
		if (addr != 0) {
			print_addr("wifi: connected, address ", addr, ifname);
			return 0;
		}
		sleep(1);
	}
	printf("wifi: no address after %d s -- the netif keeps trying in the background.\n"
		"      Check the passphrase and the AP; `wifi status` shows the state and the\n"
		"      lwip log has the join result (lwip: wifi43455: ...).\n", WIFI_WAIT_SECS);
	return 1;
}


static int do_disconnect(void)
{
	char ifname[16] = "wl";
	int t;

	if ((unlink(WIFI_CONF) != 0) && (errno != ENOENT)) {
		printf("wifi: cannot remove %s: %s\n", WIFI_CONF, strerror(errno));
		return 1;
	}
	for (t = 0; t < 15; ++t) {
		if (wl_addr(ifname, sizeof(ifname)) == 0) {
			printf("wifi: disconnected\n");
			return 0;
		}
		sleep(1);
	}
	printf("wifi: %s removed, but the netif still has an address after 15 s\n", WIFI_CONF);
	return 1;
}


static int cmd_run(int fd, const char *cmd, int cmdlen);


static int do_status(void)
{
	char ssid[40], ifname[16] = "wl";
	uint32_t addr;
	int fd;

	if (conf_get("ssid", ssid, (int)sizeof(ssid)) == 0) {
		printf("wanted:  \"%s\" (%s)\n", ssid, WIFI_CONF);
	}
	else {
		printf("wanted:  none (no %s; the boot config may still name a network)\n", WIFI_CONF);
	}

	fd = open(WIFI_DEV, O_RDWR);
	if (fd < 0) {
		printf("daemon:  not running (%s absent -- start rpi4-wifi)\n", WIFI_DEV);
	}
	else {
		printf("daemon:  ");
		fflush(stdout);
		(void)cmd_run(fd, "status", 6);
		close(fd);
	}

	addr = wl_addr(ifname, sizeof(ifname));
	if (addr != 0) {
		print_addr("address: ", addr, ifname);
	}
	else {
		printf("address: none\n");
	}
	return 0;
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
	char cmd[160]; /* "netup <ssid(32)> <psk(63)>" needs more than 64 */

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
	if (argc >= 2 && strcmp(argv[1], "stats") == 0) {
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, "stats", 5);
		close(fd);
		return rc;
	}
	if (argc >= 2 && strcmp(argv[1], "mac") == 0) {
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, "mac", 3);
		close(fd);
		return rc;
	}
	if (argc >= 4 && strcmp(argv[1], "joinwpa") == 0) {
		/* Associate + 4-way key only, no DHCP -- what an lwip netif wants,
		 * since lwip runs DHCP itself over /dev/wifidata. */
		int n = snprintf(cmd, sizeof(cmd), "joinwpa %s %s", argv[2], argv[3]);
		if (n < 0 || n >= (int)sizeof(cmd)) {
			printf("wifi: ssid/psk too long\n");
			return 2;
		}
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		printf("wifi: WPA2 join (no DHCP) on \"%s\" (~20-40s)...\n", argv[2]);
		rc = cmd_run(fd, cmd, n);
		close(fd);
		return rc;
	}
	if (argc >= 2 && strcmp(argv[1], "mtu") == 0) {
		/* Full-MTU data-path proof. Requires a prior successful `netup`, and
		 * takes ~3 s while it drains the RX FIFO. */
		printf("wifi: full-MTU data-path test (~3s)...\n");
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, "mtu", 3);
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
	if (argc >= 4 && strcmp(argv[1], "connect") == 0) {
		return do_connect(argv[2], argv[3]);
	}
	if (argc >= 2 && strcmp(argv[1], "disconnect") == 0) {
		return do_disconnect();
	}
	if (argc >= 2 && strcmp(argv[1], "status") == 0) {
		return do_status();
	}
	if (argc >= 2 && strcmp(argv[1], "leave") == 0) {
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, "leave", 5);
		close(fd);
		return rc;
	}
	if (argc >= 4 && strcmp(argv[1], "netup") == 0) {
		/* WPA2-PSK join + full DHCP exchange, entirely inside the resident
		 * driver. Takes ~20-40s: firmware join + 4-way handshake, then the
		 * DHCP rounds (DISCOVER/OFFER, REQUEST/ACK). */
		int n = snprintf(cmd, sizeof(cmd), "netup %s %s", argv[2], argv[3]);
		if (n < 0 || n >= (int)sizeof(cmd)) {
			printf("wifi: ssid/psk too long\n");
			return 2;
		}
		printf("wifi: WPA2 join + DHCP on \"%s\" (~20-40s)...\n", argv[2]);
		fd = open(WIFI_DEV, O_RDWR);
		if (fd < 0) {
			printf("wifi: cannot open %s (is rpi4-wifi running?)\n", WIFI_DEV);
			return 1;
		}
		rc = cmd_run(fd, cmd, n);
		close(fd);
		return rc;
	}
	printf("usage: wifi scan | wifi connect <ssid> <psk> | wifi disconnect | wifi status\n"
	       "  connect:    save the network to %s and wait for the DHCP lease;\n"
	       "              the WiFi netif (wl) joins it and rejoins after a reboot\n"
	       "  disconnect: forget the network; the netif releases its lease and leaves\n"
	       "diagnostics (talk to the daemon directly, bypassing the netif -- do not use\n"
	       "while it is connected):\n"
	       "  wifi mac | wifi stats | wifi joinwpa <ssid> <psk> | wifi leave |\n"
	       "  wifi netup <ssid> <psk> | wifi join <ssid> | wifi mtu\n", WIFI_CONF);
	return 2;
}
