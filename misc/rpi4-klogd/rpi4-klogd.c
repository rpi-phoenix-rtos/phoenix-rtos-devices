/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 kernel-log capture daemon (klogd) — task #31
 *
 * Reads the kernel log ring (klog) and appends it to /var/log/messages, giving
 * the port Linux-like persistent logging. It is the file-capture half of the
 * USER logging build mode (RPI4_LOG_TO_FILE in board_config.h): in that mode
 * the verbose klog is suppressed on the UART + HDMI console sinks and captured
 * here instead, so the interactive console stays quiet while the full log is
 * preserved in a file (viewable with `logread`).
 *
 * This is a klog->file pump, NOT a syslogd: there is no /dev/log socket server
 * and it does not capture application stdout/stderr (which stays on the console
 * so psh remains interactive).
 *
 * klog read path: we attach DIRECTLY to the kernel log port (the well-known oid
 * {port:0,id:0}, created during kernel init before any userspace) via mtOpen +
 * mtRead, exactly as pl011-tty's klog->fbcon drain does. We do NOT open
 * "/dev/kmsg": on Pi4 nothing registers that devfs node (pl011-tty replaced
 * libklog's devfs drain with this same direct attach, and never calls the
 * libklog registrar — see tty/pl011-tty/pl011-tty.c and libklog/libklog.c), so
 * the /dev/kmsg path that `dmesg` uses does not resolve here. On mtOpen the
 * kernel sets our reader index to the ring head, so the whole pre-attach boot
 * backlog is replayed into the file too.
 *
 * It is harmless to run in DEBUG mode (it just mirrors the console log into the
 * file) but it is only LAUNCHED in USER mode (user.plo.yaml gates it on the
 * RPI4_LOG_TO_FILE env the rebuild wrapper derives from the board macro).
 *
 * /var/log writability per boot variant:
 *   netboot - dummyfs RAM root: writable, but the file is lost on power-cycle.
 *   sd      - ext2 on the SD card: writable + persistent.
 *   nfsroot - NFS export: writable + persistent (on the NFS server).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/types.h>


#define LOG_DIR  "/var/log"
#define LOG_FILE "/var/log/messages"

/* Well-known kernel log port (oid {0,0}); see the file header. */
#define KLOG_OID_PORT 0u
#define KLOG_OID_ID   0u

/* Backoff when the klog read returns no data / a transient error, so the loop
 * cannot spin hot. mtRead on the kernel log port blocks for a blocking reader,
 * so this is only hit on the rare 0/-EPIPE return. */
#define KLOG_RETRY_US 100000 /* 100 ms */

/* Read chunk; matches the size pl011-tty/dmesg use against the same ring. */
#define BUF_SIZE 256


int main(int argc, char **argv)
{
	oid_t klog = { .port = KLOG_OID_PORT, .id = KLOG_OID_ID };
	int fileFd;
	msg_t msg;
	int rc;

	(void)argc;
	(void)argv;

	/* Create /var and /var/log ourselves so we do not need a second `-x mkdir`
	 * launch in the plo render (a duplicate mkdir alias bricks the boot). An
	 * already-existing directory (EEXIST) is fine. */
	(void)mkdir("/var", 0755);
	(void)mkdir(LOG_DIR, 0755);

	fileFd = open(LOG_FILE, O_WRONLY | O_CREAT | O_APPEND, 0644);
	if (fileFd < 0) {
		fprintf(stderr, "rpi4-klogd: cannot open %s: %s\n", LOG_FILE, strerror(errno));
		return 1;
	}

	/* Register as a blocking klog reader: mtOpen(O_RDONLY) -> log_readerAdd.
	 * The kernel sets our read index to the ring head, so the first reads
	 * replay the whole pre-attach boot backlog. */
	memset(&msg, 0, sizeof(msg));
	msg.type = mtOpen;
	msg.oid = klog;
	msg.i.openclose.flags = O_RDONLY;
	rc = msgSend(klog.port, &msg);
	if ((rc < 0) || (msg.o.err < 0)) {
		fprintf(stderr, "rpi4-klogd: klog attach failed (rc=%d err=%d)\n", rc, msg.o.err);
		close(fileFd);
		return 1;
	}

	/* One acceptance line so USER mode still has a positive console signal that
	 * logging came up (driver banner printfs are not gated). */
	printf("rpi4-klogd: capturing klog -> %s\n", LOG_FILE);

	/* Pump loop: blocking mtRead from the ring then append to the file. The
	 * kernel returns err>0 (bytes read), err==0 (spurious wake, just loop), or
	 * err<0 (-EPIPE: fell behind a ring wrap; kernel reset us to head — no data
	 * lost beyond what wrapped, so just continue). */
	for (;;) {
		char buf[BUF_SIZE];

		memset(&msg, 0, sizeof(msg));
		msg.type = mtRead;
		msg.oid = klog;
		msg.o.data = buf;
		msg.o.size = sizeof(buf);

		rc = msgSend(klog.port, &msg);
		if (rc < 0) {
			usleep(KLOG_RETRY_US);
			continue;
		}

		if (msg.o.err > 0) {
			size_t n = (size_t)msg.o.err;
			if (write(fileFd, buf, n) != (ssize_t)n) {
				/* A write error (e.g. RAM root full) must not spin the loop. */
				fprintf(stderr, "rpi4-klogd: write to %s failed: %s\n", LOG_FILE, strerror(errno));
				usleep(KLOG_RETRY_US);
			}
		}
		else if (msg.o.err < 0) {
			usleep(KLOG_RETRY_US);
		}
		/* msg.o.err == 0: blocking read returned no data; just loop. */
	}

	/* not reached */
}
