/*
 * Phoenix-RTOS
 *
 * Phoenix-RTOS klog driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/debug.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <paths.h>

#include <board_config.h>
#include <posix/utils.h>

#include "libklog.h"

extern int sys_open(const char *filename, int oflag, ...);
extern void ioctl_setResponse(msg_t *msg, unsigned long req, int err, const void *data);

#define KMSG_CTRL_ID 100

#define ERROR_MSG "libklog: Fatal error, exiting\n"

static struct {
	char __attribute__((aligned(8))) stack[2048];
	libklog_write_t ttywrite;
	struct __errno_t e;
	volatile int enabled;
	int createDevs;
	int port;
} libklog_common;


static void pumpthr(void *arg)
{
	char buf[256];
	int fd, ret;
	oid_t dev;
	char *name;

	dev.id = 0;
	dev.port = 0;

	_errno_new(&libklog_common.e);
	fd = open(_PATH_KLOG, O_RDONLY);
	if (fd < 0) {
		strcpy(buf, "devfs");
		name = strrchr(_PATH_KLOG, '/');
		if (name == NULL) {
			_errno_remove(&libklog_common.e);
			endthread();
		}

		strcat(buf, name);

		if (libklog_common.createDevs == 0) {
			/* Wait for KLOG device */
			do {
				fd = open(_PATH_KLOG, O_RDONLY);
				if (fd < 0) {
					fd = sys_open(buf, O_RDONLY, 0);
				}
				if (fd < 0) {
					usleep(10000);
				}
			} while (fd < 0);
		}
		else {
			/* On some architectures devFS might not be bound
			 * to /dev directory yet, which makes /dev/kmsg path not resolvable.
			 * To make devfs/kmsg resolvable, we need to register it first.
			 */

			if (portRegister(0, buf, &dev) != 0) {
				_errno_remove(&libklog_common.e);
				endthread();
			}

			/* open() treats paths not starting with '/' slash as local */
			fd = sys_open(buf, O_RDONLY, 0);
			if (fd < 0) {
				_errno_remove(&libklog_common.e);
				endthread();
			}
		}
	}

	while (1) {
		ret = read(fd, buf, sizeof(buf));
		if (ret <= 0) {
			if ((ret == 0) || (errno == EINTR) || (errno == EPIPE)) {
				continue;
			}
			else {
				_errno_remove(&libklog_common.e);
				endthread();
			}
		}

		if (libklog_common.enabled != 0) {
			libklog_common.ttywrite(buf, ret);
		}
	}
}


int libklog_ctrlHandle(unsigned int port, msg_t *msg, msg_rid_t rid)
{
	if (msg->type != mtDevCtl) {
		return -1;
	}

	if (msg->i.data == NULL) {
		return -1;
	}

	if (msg->oid.id != KMSG_CTRL_ID) {
		return -1;
	}

	unsigned long req;
	const void *idata;
	int err = 0;

	idata = ioctl_unpack(msg, &req, NULL);
	if (req == KIOEN) {
		libklog_common.enabled = (int)(intptr_t)idata;
	}
	else {
		err = -ENOSYS;
	}

	ioctl_setResponse(msg, req, err, NULL);
	msgRespond(port, msg, rid);

	return 0;
}


int libklog_ctrlRegister(oid_t *oid)
{
	msg_t msg = { 0 };
	oid_t odev;

	if (lookup("devfs", NULL, &odev) < 0) {
		return -1;
	}

	msg.type = mtCreate;
	msg.oid = odev;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0666;
	msg.i.create.dev = *oid;
	msg.i.data = "kmsgctrl";
	msg.i.size = sizeof("kmsgctrl");

	if (msgSend(odev.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}


void libklog_enable(int val)
{
	libklog_common.enabled = val;
}


static int libklog_initHelper(libklog_write_t clbk, int createDevs)
{
	libklog_common.ttywrite = clbk;
	libklog_common.enabled = 1;
	libklog_common.createDevs = createDevs;

	beginthread(pumpthr, 4, libklog_common.stack, sizeof(libklog_common.stack), NULL);

	return 0;
}


int libklog_initNoDev(libklog_write_t clbk)
{
	return libklog_initHelper(clbk, 0);
}


int libklog_init(libklog_write_t clbk)
{
	return libklog_initHelper(clbk, 1);
}
