/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) AF_UNIX socket readiness probe (rpi4-ipcprobe)
 *
 * The X11 (tinyx/kdrive) port gates on AF_UNIX SOCK_STREAM working on this
 * target: every X client connects to the server over /tmp/.X11-unix/X0. The
 * kernel implements AF_UNIX (posix/unix.c), but the X11 plan's Phase-1 gate is
 * a *runtime* confirmation on aarch64-rpi4b. This boot probe is that gate: it
 * exercises (1) socketpair() — the pure AF_UNIX stream data path — and (2) the
 * full named bind/listen/accept/connect/send/recv dance an X server uses, and
 * self-logs PASS/FAIL per step so a regression localizes without a debugger.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/random.h>

#define SOCK_PATH "/tmp/.x11probe"

static const char *cli_result = "client did not run";

static void *probe_client(void *arg)
{
	struct sockaddr_un sa;
	char buf[8];
	int fd;
	ssize_t n;
	(void)arg;

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		cli_result = "client socket() failed";
		return NULL;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sun_family = AF_UNIX;
	strncpy(sa.sun_path, SOCK_PATH, sizeof(sa.sun_path) - 1);

	/* The listener is bound+listening before this thread is spawned. */
	if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		cli_result = "client connect() failed";
		close(fd);
		return NULL;
	}
	printf("rpi4-ipcprobe:   client connect ok\n");

	if (write(fd, "PING", 4) != 4) {
		cli_result = "client write(PING) failed";
		close(fd);
		return NULL;
	}

	n = read(fd, buf, sizeof(buf));
	if (n == 4 && memcmp(buf, "PONG", 4) == 0) {
		cli_result = "PASS";
	}
	else {
		cli_result = "client did not receive PONG";
	}
	close(fd);
	return NULL;
}

/* socketpair(): the AF_UNIX stream data path without any name/bind. */
static int probe_socketpair(void)
{
	int sv[2];
	char buf[8];

	if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) {
		printf("rpi4-ipcprobe: socketpair FAIL: socketpair() errno=%d\n", errno);
		return -1;
	}
	if (write(sv[0], "PING", 4) != 4) {
		printf("rpi4-ipcprobe: socketpair FAIL: write errno=%d\n", errno);
		close(sv[0]);
		close(sv[1]);
		return -1;
	}
	if (read(sv[1], buf, sizeof(buf)) != 4 || memcmp(buf, "PING", 4) != 0) {
		printf("rpi4-ipcprobe: socketpair FAIL: read mismatch errno=%d\n", errno);
		close(sv[0]);
		close(sv[1]);
		return -1;
	}
	close(sv[0]);
	close(sv[1]);
	printf("rpi4-ipcprobe: socketpair PASS (AF_UNIX SOCK_STREAM data path ok)\n");
	return 0;
}

/* Full named-socket path: bind + listen + accept + connect + send + recv. */
static int probe_named(void)
{
	struct sockaddr_un sa;
	pthread_t cli;
	char buf[8];
	int lfd, cfd;
	ssize_t n;

	mkdir("/tmp", 0777);          /* X needs /tmp anyway; ignore EEXIST */
	unlink(SOCK_PATH);

	lfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (lfd < 0) {
		printf("rpi4-ipcprobe: named FAIL: socket() errno=%d\n", errno);
		return -1;
	}

	memset(&sa, 0, sizeof(sa));
	sa.sun_family = AF_UNIX;
	strncpy(sa.sun_path, SOCK_PATH, sizeof(sa.sun_path) - 1);

	if (bind(lfd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		printf("rpi4-ipcprobe: named FAIL: bind(%s) errno=%d\n", SOCK_PATH, errno);
		close(lfd);
		return -1;
	}
	printf("rpi4-ipcprobe:   bind ok\n");

	if (listen(lfd, 1) < 0) {
		printf("rpi4-ipcprobe: named FAIL: listen() errno=%d\n", errno);
		close(lfd);
		return -1;
	}
	printf("rpi4-ipcprobe:   listen ok\n");

	if (pthread_create(&cli, NULL, probe_client, NULL) != 0) {
		printf("rpi4-ipcprobe: named FAIL: pthread_create errno=%d\n", errno);
		close(lfd);
		return -1;
	}

	cfd = accept(lfd, NULL, NULL);
	if (cfd < 0) {
		printf("rpi4-ipcprobe: named FAIL: accept() errno=%d\n", errno);
		pthread_join(cli, NULL);
		close(lfd);
		return -1;
	}
	printf("rpi4-ipcprobe:   accept ok\n");

	n = read(cfd, buf, sizeof(buf));
	if (n == 4 && memcmp(buf, "PING", 4) == 0) {
		if (write(cfd, "PONG", 4) != 4) {
			printf("rpi4-ipcprobe: named FAIL: server write PONG errno=%d\n", errno);
		}
	}
	else {
		printf("rpi4-ipcprobe: named FAIL: server read got %d bytes errno=%d\n", (int)n, errno);
	}

	pthread_join(cli, NULL);
	close(cfd);
	close(lfd);
	unlink(SOCK_PATH);

	if (strcmp(cli_result, "PASS") == 0) {
		printf("rpi4-ipcprobe: named PASS (bind/listen/accept/connect/send/recv ok)\n");
		return 0;
	}
	printf("rpi4-ipcprobe: named FAIL: %s\n", cli_result);
	return -1;
}

/* getentropy/getrandom: confirm the libc entropy API fills a buffer (backed by
 * /dev/urandom -> the hardware RNG). Two independent draws should differ in most
 * bytes; all-equal would mean a stuck/zero source. */
static int probe_entropy(void)
{
	unsigned char a[32], b[32];
	int i, diff = 0;

	if (getentropy(a, sizeof(a)) != 0) {
		printf("rpi4-ipcprobe: entropy FAIL: getentropy errno=%d\n", errno);
		return -1;
	}
	if (getrandom(b, sizeof(b), 0) != (ssize_t)sizeof(b)) {
		printf("rpi4-ipcprobe: entropy FAIL: getrandom errno=%d\n", errno);
		return -1;
	}
	for (i = 0; i < (int)sizeof(a); i++) {
		if (a[i] != b[i]) {
			diff++;
		}
	}
	printf("rpi4-ipcprobe: entropy %s (getentropy+getrandom ok; %d/32 bytes differ between draws)\n",
		(diff > 0) ? "PASS" : "FAIL", diff);
	return (diff > 0) ? 0 : -1;
}

int main(int argc, char **argv)
{
	int sp, nm, en;
	(void)argc;
	(void)argv;

	printf("rpi4-ipcprobe: userspace-API readiness probe (AF_UNIX + libc entropy)\n");
	sp = probe_socketpair();
	nm = probe_named();
	en = probe_entropy();

	printf("rpi4-ipcprobe: VERDICT socketpair=%s named=%s entropy=%s -> AF_UNIX %s for X11; getrandom/getentropy %s\n",
		(sp == 0) ? "PASS" : "FAIL", (nm == 0) ? "PASS" : "FAIL", (en == 0) ? "PASS" : "FAIL",
		(sp == 0 && nm == 0) ? "READY" : "NOT-READY", (en == 0) ? "READY" : "NOT-READY");

	/* One-shot probe: done. */
	return 0;
}
