/*
 * Phoenix-RTOS
 *
 * USB HID boot mouse driver
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

/*
 * TODO: usbmouse.c and usbkbd.c share most of their scaffolding — driver
 * registration, idtree device management, control + interrupt-IN pipe open,
 * URB alloc/submit/completion + re-arm, the rx fifo, and the msgport/read/poll
 * loop. The device-specific part is tiny (the keyboard translates HID usages to
 * ASCII; the mouse forwards raw boot-report packets). A future cleanup should
 * factor the common part into a shared libhidboot core. Kept separate for now
 * so this lands as a small mirror of the accepted usbkbd driver without
 * regressing the working keyboard path.
 */


#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <posix/idtree.h>
#include <posix/utils.h>
#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <usb.h>
#include <usbdriver.h>

#include "../libtty/fifo.h"


#ifndef USBMOUSE_N_MSG_THREADS
#define USBMOUSE_N_MSG_THREADS 2
#endif

#ifndef USBMOUSE_MSG_PRIO
#define USBMOUSE_MSG_PRIO 3
#endif

#ifndef USBMOUSE_RX_FIFO_SIZE
/* 64 four-byte boot-mouse packets. */
#define USBMOUSE_RX_FIFO_SIZE 256
#endif

#ifndef USBMOUSE_N_URBS
/* MUST stay 1 with the current Pi 4 xHCI HCD — the per-slot interrupt pipe
 * tracks a single in-flight transfer (see usbkbd.c for the full rationale and
 * task #124). Bumping it crashes the HCD. */
#define USBMOUSE_N_URBS 1
#endif


enum { usbmouse_bootSubclass = 0x1, usbmouse_mouseProtocol = 0x2 };

/* HID boot-mouse report: [0]=buttons (bit0 L, bit1 R, bit2 M), [1]=X int8,
 * [2]=Y int8, [3]=wheel int8. wMaxPacketSize on the Pixart 093a:2510 is 4. */
enum { usbmouse_reportSize = 4 };

enum { usbmouse_rxStopped = 0, usbmouse_rxRunning = 1, usbmouse_rxDisconnected = -1 };

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) printf("usbmouse: " fmt "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */


typedef struct {
	idnode_t node;

	usb_devinfo_t instance;
	usb_driver_t *drv;

	int pipeCtrl;
	int pipeIntIn;
	int urbIntIn[USBMOUSE_N_URBS];
	uint8_t report[USBMOUSE_N_URBS][usbmouse_reportSize];

	char path[32];

	int flags;
	pid_t clientpid;
	volatile int rfcnt;

	fifo_t *fifo;
	handle_t lock;
	handle_t cond;
	int rxState;
} usbmouse_dev_t;


static struct {
	/* 8 KB (#121): the msg thread submits URBs through the deep
	 * usblibdrv_urbTransferAsync -> xhci_cmdExec chain; the old 1 KB stack could
	 * overflow into adjacent .bss (the usbkbd twin's 1 KB stack overflowed into
	 * hub_common.events — same pattern, root-caused via the Route-A watchpoint,
	 * see docs/inprogress/2026-06-09-usb-hid-attach-abort-localized.md). No guard
	 * page, so keep generous margin. */
	char msgstack[USBMOUSE_N_MSG_THREADS][8192] __attribute__((aligned(8)));
	idtree_t devices;
	unsigned int msgport;
	handle_t lock;
} usbmouse_common;


static const usb_device_id_t filters[] = {
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_HID, usbmouse_bootSubclass, usbmouse_mouseProtocol },
};


static usbmouse_dev_t *usbmouse_get(int id)
{
	usbmouse_dev_t *dev;

	mutexLock(usbmouse_common.lock);
	dev = lib_treeof(usbmouse_dev_t, node, idtree_find(&usbmouse_common.devices, id));
	if (dev != NULL) {
		dev->rfcnt++;
	}
	mutexUnlock(usbmouse_common.lock);

	return dev;
}


static usbmouse_dev_t *usbmouse_getByPipe(int pipe)
{
	usbmouse_dev_t *tmp, *dev = NULL;
	rbnode_t *node;

	mutexLock(usbmouse_common.lock);
	for (node = lib_rbMinimum(usbmouse_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbmouse_dev_t, node, lib_treeof(idnode_t, linkage, node));
		if (tmp->pipeIntIn == pipe) {
			dev = tmp;
			dev->rfcnt++;
			break;
		}
	}
	mutexUnlock(usbmouse_common.lock);

	return dev;
}


static int _usbmouse_put(usbmouse_dev_t *dev)
{
	if (--dev->rfcnt == 0) {
		idtree_remove(&usbmouse_common.devices, &dev->node);
	}

	return dev->rfcnt;
}


static void usbmouse_free(usbmouse_dev_t *dev)
{
	remove(dev->path);
	free(dev->fifo);
	resourceDestroy(dev->cond);
	resourceDestroy(dev->lock);
	free(dev);
}


static void usbmouse_put(usbmouse_dev_t *dev)
{
	int rfcnt;

	mutexLock(usbmouse_common.lock);
	rfcnt = _usbmouse_put(dev);
	mutexUnlock(usbmouse_common.lock);

	if (rfcnt == 0) {
		usbmouse_free(dev);
	}
}


/* Forward whole boot-mouse report packets to the rx fifo, preserving packet
 * alignment: if there isn't room for a full packet, drop the oldest whole
 * packet(s) rather than a single byte (which would desync the 4-byte framing). */
static void usbmouse_fifoPush(usbmouse_dev_t *dev, const uint8_t *data, size_t len)
{
	size_t i;
	size_t k;

	mutexLock(dev->lock);
	while ((fifo_freespace(dev->fifo) < len) && !fifo_is_empty(dev->fifo)) {
		for (k = 0u; (k < len) && !fifo_is_empty(dev->fifo); ++k) {
			(void)fifo_pop_back(dev->fifo);
		}
	}

	for (i = 0u; i < len; ++i) {
		fifo_push(dev->fifo, data[i]);
	}
	condSignal(dev->cond);
	mutexUnlock(dev->lock);
}


static void usbmouse_handleReport(usbmouse_dev_t *dev, const uint8_t *report, size_t len)
{
	if (len < usbmouse_reportSize) {
		return;
	}

	/* Raw passthrough: a boot mouse with SET_IDLE(0) reports only on change, so
	 * every report is a meaningful event (movement and/or button transition).
	 * Userspace consumes /dev/mouseN as a stream of 4-byte packets. */
	usbmouse_fifoPush(dev, report, usbmouse_reportSize);
}


static int usbmouse_setProtocol(usbmouse_dev_t *dev)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = CLASS_REQ_SET_PROTOCOL,
		.wValue = 0,
		.wIndex = dev->instance.interface,
		.wLength = 0
	};

	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, NULL, 0, usb_dir_out);
}


static int usbmouse_setIdle(usbmouse_dev_t *dev)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = CLASS_REQ_SET_IDLE,
		.wValue = 0,
		.wIndex = dev->instance.interface,
		.wLength = 0
	};

	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, NULL, 0, usb_dir_out);
}


static int _usbmouse_start(usbmouse_dev_t *dev)
{
	int i;
	int ret;

	for (i = 0; i < USBMOUSE_N_URBS; ++i) {
		ret = usb_transferAsync(dev->drv, dev->pipeIntIn, dev->urbIntIn[i], usbmouse_reportSize, NULL);
		if (ret < 0) {
			return -EIO;
		}
	}

	dev->rxState = usbmouse_rxRunning;
	return EOK;
}


static int _usbmouse_urbsAlloc(usbmouse_dev_t *dev)
{
	int i;
	int j;

	for (i = 0; i < USBMOUSE_N_URBS; ++i) {
		dev->urbIntIn[i] = usb_urbAlloc(dev->drv, dev->pipeIntIn, dev->report[i], usb_dir_in, usbmouse_reportSize, usb_transfer_interrupt);
		if (dev->urbIntIn[i] < 0) {
			for (j = i - 1; j >= 0; --j) {
				usb_urbFree(dev->drv, dev->pipeIntIn, dev->urbIntIn[j]);
			}
			return -ENOMEM;
		}
	}

	return EOK;
}


static void _usbmouse_close(usbmouse_dev_t *dev)
{
	int i;

	for (i = 0; i < USBMOUSE_N_URBS; ++i) {
		if (dev->urbIntIn[i] >= 0) {
			usb_urbFree(dev->drv, dev->pipeIntIn, dev->urbIntIn[i]);
			dev->urbIntIn[i] = -1;
		}
	}

	fifo_remove_all(dev->fifo);
	dev->flags = 0;
	dev->clientpid = 0;
	dev->rxState = usbmouse_rxStopped;
}


static int _usbmouse_open(usbmouse_dev_t *dev, int flags, pid_t pid)
{
	if (dev->flags != 0) {
		return -EBUSY;
	}

	if ((flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}

	if (_usbmouse_urbsAlloc(dev) < 0) {
		return -ENOMEM;
	}

	if (_usbmouse_start(dev) < 0) {
		_usbmouse_close(dev);
		return -EIO;
	}

	dev->flags = flags;
	dev->clientpid = pid;

	return EOK;
}


/* Return whole 4-byte packets only, so a reader never sees a torn report. */
static int usbmouse_read(usbmouse_dev_t *dev, char *data, size_t len, unsigned int mode)
{
	size_t cnt;
	size_t avail;
	size_t want;

	if ((dev->flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}

	mutexLock(dev->lock);
	while ((dev->rxState != usbmouse_rxDisconnected) && fifo_is_empty(dev->fifo)) {
		if ((mode & O_NONBLOCK) != 0u) {
			mutexUnlock(dev->lock);
			return -EWOULDBLOCK;
		}

		condWait(dev->cond, dev->lock, 0);
	}

	if ((dev->rxState == usbmouse_rxDisconnected) && fifo_is_empty(dev->fifo)) {
		mutexUnlock(dev->lock);
		return -ENODEV;
	}

	avail = fifo_count(dev->fifo);
	want = (len < avail) ? len : avail;
	want -= (want % (size_t)usbmouse_reportSize);

	for (cnt = 0u; cnt < want; ++cnt) {
		data[cnt] = (char)fifo_pop_back(dev->fifo);
	}
	mutexUnlock(dev->lock);

	return (int)cnt;
}


static void usbmouse_msgthr(void *arg)
{
	usbmouse_dev_t *dev;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(usbmouse_common.msgport, &msg, &rid) < 0) {
			fprintf(stderr, "usbmouse: msgRecv returned with err\n");
			break;
		}

		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(usbmouse_common.msgport, &msg, rid);
			continue;
		}

		dev = usbmouse_get(msg.oid.id);
		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(usbmouse_common.msgport, &msg, rid);
			continue;
		}

		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid)) {
			usbmouse_put(dev);
			msg.o.err = -EBUSY;
			msgRespond(usbmouse_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbmouse_common.lock);
				msg.o.err = _usbmouse_open(dev, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbmouse_common.lock);
				break;

			case mtClose:
				mutexLock(usbmouse_common.lock);
				_usbmouse_close(dev);
				mutexUnlock(usbmouse_common.lock);
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = usbmouse_read(dev, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				msg.o.err = -ENOSYS;
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = POLLOUT;
					mutexLock(dev->lock);
					if (!fifo_is_empty(dev->fifo)) {
						msg.o.attr.val |= POLLIN;
					}
					mutexUnlock(dev->lock);
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

		usbmouse_put(dev);
		msgRespond(usbmouse_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbmouse_handleCompletion(usb_driver_t *drv, usb_completion_t *c, const char *data, size_t len)
{
	usbmouse_dev_t *dev;

	dev = usbmouse_getByPipe(c->pipeid);
	if (dev == NULL) {
		return -ENOENT;
	}

	if (c->err != 0) {
		mutexLock(dev->lock);
		dev->rxState = usbmouse_rxStopped;
		condSignal(dev->cond);
		mutexUnlock(dev->lock);
		usbmouse_put(dev);
		return -EIO;
	}

	usbmouse_handleReport(dev, (const uint8_t *)data, len);

	if (dev->rxState == usbmouse_rxRunning) {
		usb_transferAsync(drv, dev->pipeIntIn, c->urbid, usbmouse_reportSize, NULL);
	}

	usbmouse_put(dev);
	return EOK;
}


static usbmouse_dev_t *_usbmouse_devAlloc(void)
{
	usbmouse_dev_t *dev;
	int i;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		return NULL;
	}

	if (idtree_alloc(&usbmouse_common.devices, &dev->node) < 0) {
		free(dev);
		return NULL;
	}

	dev->fifo = malloc(sizeof(fifo_t) + USBMOUSE_RX_FIFO_SIZE);
	if (dev->fifo == NULL) {
		idtree_remove(&usbmouse_common.devices, &dev->node);
		free(dev);
		return NULL;
	}

	fifo_init(dev->fifo, USBMOUSE_RX_FIFO_SIZE);

	if (mutexCreate(&dev->lock) != 0) {
		idtree_remove(&usbmouse_common.devices, &dev->node);
		free(dev->fifo);
		free(dev);
		return NULL;
	}

	if (condCreate(&dev->cond) != 0) {
		idtree_remove(&usbmouse_common.devices, &dev->node);
		resourceDestroy(dev->lock);
		free(dev->fifo);
		free(dev);
		return NULL;
	}

	for (i = 0; i < USBMOUSE_N_URBS; ++i) {
		dev->urbIntIn[i] = -1;
	}

	dev->rfcnt = 1;
	return dev;
}


static int usbmouse_handleInsertion(usb_driver_t *drv, usb_devinfo_t *insertion, usb_event_insertion_t *event)
{
	usbmouse_dev_t *dev;
	oid_t oid;
	int err;

	debug("usbmouse: handleInsertion fired\n");

	mutexLock(usbmouse_common.lock);

	dev = _usbmouse_devAlloc();
	if (dev == NULL) {
		mutexUnlock(usbmouse_common.lock);
		return -ENOMEM;
	}

	dev->instance = *insertion;
	dev->drv = drv;

	do {
		dev->pipeCtrl = usb_open(drv, insertion, usb_transfer_control, 0);
		if (dev->pipeCtrl < 0) {
			err = -EINVAL;
			break;
		}

		err = usb_setConfiguration(drv, dev->pipeCtrl, 1);
		if (err != 0) {
			err = -EINVAL;
			break;
		}

		dev->pipeIntIn = usb_open(drv, insertion, usb_transfer_interrupt, usb_dir_in);
		if (dev->pipeIntIn < 0) {
			err = -EINVAL;
			break;
		}

		err = usbmouse_setProtocol(dev);
		if (err < 0) {
			fprintf(stderr, "usbmouse: failed to switch to boot protocol: %d\n", err);
			break;
		}

		err = usbmouse_setIdle(dev);
		if (err < 0) {
			TRACE("set idle unsupported: %d", err);
		}

		oid.port = usbmouse_common.msgport;
		oid.id = idtree_id(&dev->node);

		snprintf(dev->path, sizeof(dev->path), "/dev/mouse%d", idtree_id(&dev->node));
		err = create_dev(&oid, dev->path);
		if (err != 0) {
			err = -EINVAL;
			break;
		}
	} while (0);

	if (err < 0) {
		_usbmouse_put(dev);
	}

	mutexUnlock(usbmouse_common.lock);

	if (err < 0) {
		free(dev);
		return err;
	}

	fprintf(stdout, "usbmouse: New device: %s\n", dev->path);
	debug("usbmouse: New /dev/mouse device created\n");

	event->deviceCreated = true;
	event->dev = oid;
	strncpy(event->devPath, dev->path, sizeof(event->devPath));

	return EOK;
}


static int usbmouse_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	rbnode_t *node;
	rbnode_t *next;
	usbmouse_dev_t *dev;

	mutexLock(usbmouse_common.lock);

	node = lib_rbMinimum(usbmouse_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);
		dev = lib_treeof(usbmouse_dev_t, node, lib_treeof(idnode_t, linkage, node));

		if ((dev->instance.bus == del->bus) && (dev->instance.dev == del->dev) &&
				(dev->instance.interface == del->interface)) {
			mutexLock(dev->lock);
			dev->rxState = usbmouse_rxDisconnected;
			condSignal(dev->cond);
			mutexUnlock(dev->lock);

			if (_usbmouse_put(dev) == 0) {
				usbmouse_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbmouse_common.lock);

	return EOK;
}


static int usbmouse_init(usb_driver_t *drv, void *args)
{
	int err;
	int i;

	if (portCreate(&usbmouse_common.msgport) != 0) {
		return -ENOMEM;
	}

	if (mutexCreate(&usbmouse_common.lock) != 0) {
		return -ENOMEM;
	}

	idtree_init(&usbmouse_common.devices);

	for (i = 0; i < USBMOUSE_N_MSG_THREADS; ++i) {
		err = beginthread(usbmouse_msgthr, USBMOUSE_MSG_PRIO, usbmouse_common.msgstack[i], sizeof(usbmouse_common.msgstack[i]), NULL);
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


static int usbmouse_destroy(usb_driver_t *drv)
{
	return EOK;
}


static usb_driver_t usbmouse_driver = {
	.name = "usbmouse",
	.handlers = {
		.insertion = usbmouse_handleInsertion,
		.deletion = usbmouse_handleDeletion,
		.completion = usbmouse_handleCompletion,
	},
	.ops = {
		.init = usbmouse_init,
		.destroy = usbmouse_destroy,
	},
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = (void *)&usbmouse_common,
};


__attribute__((constructor)) static void usbmouse_register(void)
{
	usb_driverRegister(&usbmouse_driver);
}
