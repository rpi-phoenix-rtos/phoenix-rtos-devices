/*
 * Phoenix-RTOS
 *
 * USB HID boot keyboard driver
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
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


#ifndef USBKBD_N_MSG_THREADS
#define USBKBD_N_MSG_THREADS 2
#endif

#ifndef USBKBD_MSG_PRIO
#define USBKBD_MSG_PRIO 3
#endif

#ifndef USBKBD_RX_FIFO_SIZE
#define USBKBD_RX_FIFO_SIZE 256
#endif

#ifndef USBKBD_N_URBS
/* Number of interrupt-IN URBs queued on the HID endpoint.
 *
 * MUST stay 1 with the current Pi 4 xHCI HCD: queuing multiple URBs on a single
 * interrupt endpoint corrupted xHCI controller state and faulted (Data Abort in
 * xhci_enterRunState->writeReg with a wild register base) once the keyboard's
 * interrupt pipe was driven. The xHCI per-slot interrupt-pipe support tracks a
 * single priv per slot (see xhci_initInterruptInPipe / the roothub poll), so it
 * does not yet handle a ring of in-flight URBs per pipe.
 *
 * Consequence: a single URB leaves a re-arm gap between a completion and its
 * resubmit (Phoenix delivers completions over an async message round-trip), so
 * fast keypresses can be dropped (observed: 3 presses, 1 echoed). Fixing that
 * reliably needs HCD-level work (multi-URB-per-interrupt-pipe support, or a
 * faster in-completion resubmit) — tracked separately — NOT a bump of this
 * constant, which crashes. */
#define USBKBD_N_URBS 1
#endif


enum { usbkbd_bootSubclass = 0x1, usbkbd_keyboardProtocol = 0x1 };

enum { usbkbd_reportSize = 8 };

enum { usbkbd_rxStopped = 0, usbkbd_rxRunning = 1, usbkbd_rxDisconnected = -1 };

enum {
	usbkbd_modLCtrl = 1 << 0,
	usbkbd_modLShift = 1 << 1,
	usbkbd_modRShift = 1 << 5,
	usbkbd_modRCtrl = 1 << 4
};

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) printf("usbkbd: " fmt "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */


typedef struct {
	idnode_t node;

	usb_devinfo_t instance;
	usb_driver_t *drv;

	int pipeCtrl;
	int pipeIntIn;
	int urbIntIn[USBKBD_N_URBS];
	uint8_t report[USBKBD_N_URBS][usbkbd_reportSize];
	uint8_t prevReport[usbkbd_reportSize];

	char path[32];

	int flags;
	pid_t clientpid;
	volatile int rfcnt;

	fifo_t *fifo;
	handle_t lock;
	handle_t cond;
	int rxState;

	unsigned int capsLock;
} usbkbd_dev_t;


static struct {
	/* 8 KB (#121): the msg thread submits URBs for the interrupt-IN pipe, which
	 * descends usblibdrv_urbTransferAsync -> xhci_cmdExec -> xhci_enterRunState
	 * (a deep, ~1 KB+ frame chain). The old 1 KB stack overflowed ~64 B past its
	 * base into the adjacent .bss (hub_common.events), writing a saved return
	 * address there and surfacing later as a bogus lib_listRemove crash in the
	 * USB daemon (the intermittent HID-attach Data Abort). Caught with the Route-A
	 * watchpoint; see docs/inprogress/2026-06-09-usb-hid-attach-abort-localized.md.
	 * These stacks have no guard page, so keep generous margin. */
	char msgstack[USBKBD_N_MSG_THREADS][8192] __attribute__((aligned(8)));
	idtree_t devices;
	unsigned int msgport;
	handle_t lock;
} usbkbd_common;


static const usb_device_id_t filters[] = {
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_HID, usbkbd_bootSubclass, usbkbd_keyboardProtocol },
};


static const char usbkbd_letters[] = "abcdefghijklmnopqrstuvwxyz";
static const char usbkbd_lettersShift[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const char usbkbd_digits[] = "1234567890";
static const char usbkbd_digitsShift[] = "!@#$%^&*()";
/* HID Keyboard/Keypad usages 0x2d..0x38, indexed by (usage - 0x2d). The range
 * is contiguous and INCLUDES 0x32 (Non-US # and ~) between 0x31 (\ |) and
 * 0x33 (; :); a US keyboard never emits 0x32 but the slot must be present or
 * every entry from 0x33 onward shifts down by one (which previously made / ?
 * at 0x38 read the string terminator -> dead key). Keep both tables 12 long. */
static const char usbkbd_symbols[] = "-=[]\\#;'`,./";
static const char usbkbd_symbolsShift[] = "_+{}|~:\"~<>?";


static usbkbd_dev_t *usbkbd_get(int id)
{
	usbkbd_dev_t *dev;

	mutexLock(usbkbd_common.lock);
	dev = lib_treeof(usbkbd_dev_t, node, idtree_find(&usbkbd_common.devices, id));
	if (dev != NULL) {
		dev->rfcnt++;
	}
	mutexUnlock(usbkbd_common.lock);

	return dev;
}


static usbkbd_dev_t *usbkbd_getByPipe(int pipe)
{
	usbkbd_dev_t *tmp, *dev = NULL;
	rbnode_t *node;

	mutexLock(usbkbd_common.lock);
	for (node = lib_rbMinimum(usbkbd_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbkbd_dev_t, node, lib_treeof(idnode_t, linkage, node));
		if (tmp->pipeIntIn == pipe) {
			dev = tmp;
			dev->rfcnt++;
			break;
		}
	}
	mutexUnlock(usbkbd_common.lock);

	return dev;
}


static int _usbkbd_put(usbkbd_dev_t *dev)
{
	if (--dev->rfcnt == 0) {
		idtree_remove(&usbkbd_common.devices, &dev->node);
	}

	return dev->rfcnt;
}


static void usbkbd_free(usbkbd_dev_t *dev)
{
	remove(dev->path);
	free(dev->fifo);
	resourceDestroy(dev->cond);
	resourceDestroy(dev->lock);
	free(dev);
}


static void usbkbd_put(usbkbd_dev_t *dev)
{
	int rfcnt;

	mutexLock(usbkbd_common.lock);
	rfcnt = _usbkbd_put(dev);
	mutexUnlock(usbkbd_common.lock);

	if (rfcnt == 0) {
		usbkbd_free(dev);
	}
}


static void usbkbd_fifoPush(usbkbd_dev_t *dev, const char *data, size_t len)
{
	size_t i;

	mutexLock(dev->lock);
	for (i = 0; i < len; ++i) {
		if (fifo_is_full(dev->fifo)) {
			(void)fifo_pop_back(dev->fifo);
		}

		fifo_push(dev->fifo, (uint8_t)data[i]);
	}
	condSignal(dev->cond);
	mutexUnlock(dev->lock);
}


static int usbkbd_isPressed(const uint8_t *report, uint8_t usage)
{
	size_t i;

	for (i = 2; i < usbkbd_reportSize; ++i) {
		if (report[i] == usage) {
			return 1;
		}
	}

	return 0;
}


static size_t usbkbd_translateUsage(usbkbd_dev_t *dev, uint8_t modifiers, uint8_t usage, char *out, size_t outsz)
{
	unsigned int shift;
	unsigned int ctrl;
	unsigned int upper;

	if (outsz == 0u) {
		return 0u;
	}

	shift = ((modifiers & usbkbd_modLShift) != 0u) || ((modifiers & usbkbd_modRShift) != 0u);
	ctrl = ((modifiers & usbkbd_modLCtrl) != 0u) || ((modifiers & usbkbd_modRCtrl) != 0u);

	if ((usage >= 0x04u) && (usage <= 0x1du)) {
		upper = shift ^ (dev->capsLock != 0u);
		out[0] = upper ? usbkbd_lettersShift[usage - 0x04u] : usbkbd_letters[usage - 0x04u];
		if (ctrl != 0u) {
			out[0] = (char)(usbkbd_letters[usage - 0x04u] - 'a' + 1);
		}
		return 1u;
	}

	if ((usage >= 0x1eu) && (usage <= 0x27u)) {
		out[0] = (shift != 0u) ? usbkbd_digitsShift[usage - 0x1eu] : usbkbd_digits[usage - 0x1eu];
		return 1u;
	}

	if ((usage >= 0x2du) && (usage <= 0x38u)) {
		out[0] = (shift != 0u) ? usbkbd_symbolsShift[usage - 0x2du] : usbkbd_symbols[usage - 0x2du];
		return 1u;
	}

	switch (usage) {
		case 0x28u:
			out[0] = '\n';
			return 1u;

		case 0x29u:
			out[0] = '\033';
			return 1u;

		case 0x2au:
			out[0] = '\b';
			return 1u;

		case 0x2bu:
			out[0] = '\t';
			return 1u;

		case 0x2cu:
			out[0] = ' ';
			return 1u;

		case 0x39u:
			dev->capsLock ^= 1u;
			return 0u;

		case 0x4au:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[H", 3);
			return 3u;

		case 0x4bu:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[F", 3);
			return 3u;

		case 0x4cu:
			if (outsz < 5u) {
				return 0u;
			}
			memcpy(out, "\033[3~", 4);
			return 4u;

		case 0x4du:
			if (outsz < 5u) {
				return 0u;
			}
			memcpy(out, "\033[4~", 4);
			return 4u;

		case 0x4eu:
			if (outsz < 5u) {
				return 0u;
			}
			memcpy(out, "\033[6~", 4);
			return 4u;

		case 0x4fu:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[C", 3);
			return 3u;

		case 0x50u:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[D", 3);
			return 3u;

		case 0x51u:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[B", 3);
			return 3u;

		case 0x52u:
			if (outsz < 4u) {
				return 0u;
			}
			memcpy(out, "\033[A", 3);
			return 3u;

		default:
			return 0u;
	}
}


static void usbkbd_handleReport(usbkbd_dev_t *dev, const uint8_t *report, size_t len)
{
	uint8_t modifiers;
	char translated[8];
	size_t translatedLen;
	size_t i;

	if (len < usbkbd_reportSize) {
		return;
	}

	modifiers = report[0];
	for (i = 2; i < usbkbd_reportSize; ++i) {
		if ((report[i] == 0u) || (usbkbd_isPressed(dev->prevReport, report[i]) != 0)) {
			continue;
		}

		translatedLen = usbkbd_translateUsage(dev, modifiers, report[i], translated, sizeof(translated));
		if (translatedLen != 0u) {
			usbkbd_fifoPush(dev, translated, translatedLen);
		}
	}

	memcpy(dev->prevReport, report, usbkbd_reportSize);
}


static int usbkbd_setProtocol(usbkbd_dev_t *dev)
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


static int usbkbd_setIdle(usbkbd_dev_t *dev)
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


static int _usbkbd_start(usbkbd_dev_t *dev)
{
	int i;
	int ret;

	for (i = 0; i < USBKBD_N_URBS; ++i) {
		ret = usb_transferAsync(dev->drv, dev->pipeIntIn, dev->urbIntIn[i], usbkbd_reportSize, NULL);
		if (ret < 0) {
			return -EIO;
		}
	}

	dev->rxState = usbkbd_rxRunning;
	return EOK;
}


static int _usbkbd_urbsAlloc(usbkbd_dev_t *dev)
{
	int i;
	int j;

	for (i = 0; i < USBKBD_N_URBS; ++i) {
		dev->urbIntIn[i] = usb_urbAlloc(dev->drv, dev->pipeIntIn, dev->report[i], usb_dir_in, usbkbd_reportSize, usb_transfer_interrupt);
		if (dev->urbIntIn[i] < 0) {
			for (j = i - 1; j >= 0; --j) {
				usb_urbFree(dev->drv, dev->pipeIntIn, dev->urbIntIn[j]);
			}
			return -ENOMEM;
		}
	}

	return EOK;
}


static void _usbkbd_close(usbkbd_dev_t *dev)
{
	int i;

	for (i = 0; i < USBKBD_N_URBS; ++i) {
		if (dev->urbIntIn[i] >= 0) {
			usb_urbFree(dev->drv, dev->pipeIntIn, dev->urbIntIn[i]);
			dev->urbIntIn[i] = -1;
		}
	}

	fifo_remove_all(dev->fifo);
	memset(dev->prevReport, 0, sizeof(dev->prevReport));
	dev->capsLock = 0u;
	dev->flags = 0;
	dev->clientpid = 0;
	dev->rxState = usbkbd_rxStopped;
}


static int _usbkbd_open(usbkbd_dev_t *dev, int flags, pid_t pid)
{
	if (dev->flags != 0) {
		return -EBUSY;
	}

	if ((flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}

	if (_usbkbd_urbsAlloc(dev) < 0) {
		return -ENOMEM;
	}

	if (_usbkbd_start(dev) < 0) {
		_usbkbd_close(dev);
		return -EIO;
	}

	dev->flags = flags;
	dev->clientpid = pid;

	return EOK;
}


static int usbkbd_read(usbkbd_dev_t *dev, char *data, size_t len, unsigned int mode)
{
	size_t cnt;

	if ((dev->flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}

	mutexLock(dev->lock);
	while ((dev->rxState != usbkbd_rxDisconnected) && fifo_is_empty(dev->fifo)) {
		if ((mode & O_NONBLOCK) != 0u) {
			mutexUnlock(dev->lock);
			return -EWOULDBLOCK;
		}

		condWait(dev->cond, dev->lock, 0);
	}

	if ((dev->rxState == usbkbd_rxDisconnected) && fifo_is_empty(dev->fifo)) {
		mutexUnlock(dev->lock);
		return -ENODEV;
	}

	for (cnt = 0u; (cnt < len) && !fifo_is_empty(dev->fifo); ++cnt) {
		data[cnt] = (char)fifo_pop_back(dev->fifo);
	}
	mutexUnlock(dev->lock);

	return (int)cnt;
}


static void usbkbd_msgthr(void *arg)
{
	usbkbd_dev_t *dev;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(usbkbd_common.msgport, &msg, &rid) < 0) {
			fprintf(stderr, "usbkbd: msgRecv returned with err\n");
			break;
		}

		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(usbkbd_common.msgport, &msg, rid);
			continue;
		}

		dev = usbkbd_get(msg.oid.id);
		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(usbkbd_common.msgport, &msg, rid);
			continue;
		}

		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid)) {
			usbkbd_put(dev);
			msg.o.err = -EBUSY;
			msgRespond(usbkbd_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbkbd_common.lock);
				msg.o.err = _usbkbd_open(dev, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbkbd_common.lock);
				break;

			case mtClose:
				mutexLock(usbkbd_common.lock);
				_usbkbd_close(dev);
				mutexUnlock(usbkbd_common.lock);
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = usbkbd_read(dev, msg.o.data, msg.o.size, msg.i.io.mode);
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

		usbkbd_put(dev);
		msgRespond(usbkbd_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbkbd_handleCompletion(usb_driver_t *drv, usb_completion_t *c, const char *data, size_t len)
{
	usbkbd_dev_t *dev;

	dev = usbkbd_getByPipe(c->pipeid);
	if (dev == NULL) {
		return -ENOENT;
	}

	if (c->err != 0) {
		mutexLock(dev->lock);
		dev->rxState = usbkbd_rxStopped;
		condSignal(dev->cond);
		mutexUnlock(dev->lock);
		usbkbd_put(dev);
		return -EIO;
	}

	usbkbd_handleReport(dev, (const uint8_t *)data, len);

	if (dev->rxState == usbkbd_rxRunning) {
		usb_transferAsync(drv, dev->pipeIntIn, c->urbid, usbkbd_reportSize, NULL);
	}

	usbkbd_put(dev);
	return EOK;
}


static usbkbd_dev_t *_usbkbd_devAlloc(void)
{
	usbkbd_dev_t *dev;
	int i;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		return NULL;
	}

	if (idtree_alloc(&usbkbd_common.devices, &dev->node) < 0) {
		free(dev);
		return NULL;
	}

	dev->fifo = malloc(sizeof(fifo_t) + USBKBD_RX_FIFO_SIZE);
	if (dev->fifo == NULL) {
		idtree_remove(&usbkbd_common.devices, &dev->node);
		free(dev);
		return NULL;
	}

	fifo_init(dev->fifo, USBKBD_RX_FIFO_SIZE);

	if (mutexCreate(&dev->lock) != 0) {
		idtree_remove(&usbkbd_common.devices, &dev->node);
		free(dev->fifo);
		free(dev);
		return NULL;
	}

	if (condCreate(&dev->cond) != 0) {
		idtree_remove(&usbkbd_common.devices, &dev->node);
		resourceDestroy(dev->lock);
		free(dev->fifo);
		free(dev);
		return NULL;
	}

	for (i = 0; i < USBKBD_N_URBS; ++i) {
		dev->urbIntIn[i] = -1;
	}

	dev->rfcnt = 1;
	return dev;
}


static int usbkbd_handleInsertion(usb_driver_t *drv, usb_devinfo_t *insertion, usb_event_insertion_t *event)
{
	usbkbd_dev_t *dev;
	oid_t oid;
	int err;

	debug("usbkbd: handleInsertion fired\n");

	mutexLock(usbkbd_common.lock);

	dev = _usbkbd_devAlloc();
	if (dev == NULL) {
		mutexUnlock(usbkbd_common.lock);
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

		err = usbkbd_setProtocol(dev);
		if (err < 0) {
			fprintf(stderr, "usbkbd: failed to switch to boot protocol: %d\n", err);
			break;
		}

		err = usbkbd_setIdle(dev);
		if (err < 0) {
			TRACE("set idle unsupported: %d", err);
		}

		oid.port = usbkbd_common.msgport;
		oid.id = idtree_id(&dev->node);

		snprintf(dev->path, sizeof(dev->path), "/dev/kbd%d", idtree_id(&dev->node));
		err = create_dev(&oid, dev->path);
		if (err != 0) {
			err = -EINVAL;
			break;
		}
	} while (0);

	if (err < 0) {
		_usbkbd_put(dev);
	}

	mutexUnlock(usbkbd_common.lock);

	if (err < 0) {
		free(dev);
		return err;
	}

	fprintf(stdout, "usbkbd: New device: %s\n", dev->path);
	debug("usbkbd: New /dev/kbd0 device created\n");

	event->deviceCreated = true;
	event->dev = oid;
	strncpy(event->devPath, dev->path, sizeof(event->devPath));

	return EOK;
}


static int usbkbd_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	rbnode_t *node;
	rbnode_t *next;
	usbkbd_dev_t *dev;

	mutexLock(usbkbd_common.lock);

	node = lib_rbMinimum(usbkbd_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);
		dev = lib_treeof(usbkbd_dev_t, node, lib_treeof(idnode_t, linkage, node));

		if ((dev->instance.bus == del->bus) && (dev->instance.dev == del->dev) &&
				(dev->instance.interface == del->interface)) {
			mutexLock(dev->lock);
			dev->rxState = usbkbd_rxDisconnected;
			condSignal(dev->cond);
			mutexUnlock(dev->lock);

			if (_usbkbd_put(dev) == 0) {
				usbkbd_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbkbd_common.lock);

	return EOK;
}


static int usbkbd_init(usb_driver_t *drv, void *args)
{
	int err;
	int i;

	if (portCreate(&usbkbd_common.msgport) != 0) {
		return -ENOMEM;
	}

	if (mutexCreate(&usbkbd_common.lock) != 0) {
		return -ENOMEM;
	}

	idtree_init(&usbkbd_common.devices);

	for (i = 0; i < USBKBD_N_MSG_THREADS; ++i) {
		err = beginthread(usbkbd_msgthr, USBKBD_MSG_PRIO, usbkbd_common.msgstack[i], sizeof(usbkbd_common.msgstack[i]), NULL);
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


static int usbkbd_destroy(usb_driver_t *drv)
{
	return EOK;
}


static usb_driver_t usbkbd_driver = {
	.name = "usbkbd",
	.handlers = {
		.insertion = usbkbd_handleInsertion,
		.deletion = usbkbd_handleDeletion,
		.completion = usbkbd_handleCompletion,
	},
	.ops = {
		.init = usbkbd_init,
		.destroy = usbkbd_destroy,
	},
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = (void *)&usbkbd_common,
};


__attribute__((constructor)) static void usbkbd_register(void)
{
	usb_driverRegister(&usbkbd_driver);
}
