/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Copyright 2021, 2024 Phoenix Systems
 * Author: Maciej Purski, Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <arpa/inet.h>
#include <errno.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <stdint.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <posix/idtree.h>

#include <board_config.h>

#define UMASS_DEBUG 0

#ifdef UMASS_MOUNT_EXT2
#include <libext2.h>
#endif

#include <usb.h>
#include <usbdriver.h>

#include "../pc-ata/mbr.h"
#include "umass.h"
#include "scsi.h"

#ifndef UMASS_N_MSG_THREADS
#define UMASS_N_MSG_THREADS 2
#endif

#ifndef UMASS_N_POOL_THREADS
#define UMASS_N_POOL_THREADS 2
#endif

#ifndef UMASS_MSG_PRIO
#define UMASS_MSG_PRIO 4
#endif

#ifndef UMASS_POOL_PRIO
#define UMASS_POOL_PRIO 4
#endif

#define UMASS_TRANSMIT_RETRIES 3
#define UMASS_INIT_RETRIES     10

#define UMASS_SECTOR_SIZE 512
/* Largest single SCSI READ(10)/WRITE(10) this driver issues. Anything bigger is
 * split across commands by umass_{read,write}ToDev. */
#define UMASS_MAX_IO (64u * 1024u)

#define UMASS_WRITE 0
#define UMASS_READ  0x80

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355

#define USB_SUBCLASS_SCSI 0x06
#define USB_PROTOCOL_BULK 0x50

/* clang-format off */
#define LOG(str_, ...) do { printf("umass: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG("error: " str_, ##__VA_ARGS__)
#define TRACE(str_, ...) do { if (0) LOG("trace (%d): " str_, __LINE__, ##__VA_ARGS__); } while (0)
#define DEBUG(str_, ...) do { if (UMASS_DEBUG) LOG("debug: " str_, ##__VA_ARGS__); } while (0)
/* clang-format on */


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dlen;
	uint8_t flags;
	uint8_t lun;
	uint8_t clen;
	uint8_t cmd[16];
} __attribute__((packed)) umass_cbw_t;


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dr;
	uint8_t status;
} __attribute__((packed)) umass_csw_t;


/* Device access callbacks */
static ssize_t umass_read(id_t id, off_t offs, char *buf, size_t len);
static ssize_t umass_write(id_t id, off_t offs, const char *buf, size_t len);


/* Thread types */
typedef struct umass_dev umass_dev_t;
static int _umass_partUnmount(umass_dev_t *dev);

static void umass_fsthr(void *arg);
static void umass_poolthr(void *arg);
static void umass_msgthr(void *arg);


/* Filesystem callbacks types */
typedef int (*fs_handler_t)(void *, msg_t *);
typedef int (*fs_unmount_t)(void *);
typedef int (*fs_mount_t)(oid_t *, unsigned int, typeof(umass_read) *, typeof(umass_write) *, void **);


typedef struct {
	rbnode_t node;      /* RBTree node */
	char name[16];      /* Filesystem name */
	uint8_t type;       /* Compatible partition type */
	fs_handler_t handler; /* Message handler callback */
	fs_unmount_t unmount; /* Unmount callback */
	fs_mount_t mount;     /* Mount callback */
} umass_fs_t;


typedef struct _umass_part_t {
	/* Partition data */
	unsigned int idx;  /* Partition index */
	unsigned int port; /* Partition port */
	uint32_t start;    /* Partition start */
	uint32_t sectors;  /* Number of sectors */

	/* Filesystem data */
	umass_fs_t *fs; /* Mounted filesystem */
	void *fdata;    /* Mounted filesystem data */

	oid_t mnt; /* Mountpoint this partition is spliced onto (for mtMountPoint) */

	/* Teardown handshake with umass_fsthr. The thread runs on fsstack below,
	 * i.e. INSIDE this struct, so the struct must not be freed until it has
	 * left -- see _umass_devFree. */
	volatile int fsthrRunning;
	volatile int fsthrExit;

	/* Partition filesystem thread stack */
	char fsstack[4 * _PAGE_SIZE] __attribute__((aligned(8)));
} umass_part_t;


typedef struct _umass_req_t {
	msg_t msg;                        /* Request msg */
	msg_rid_t rid;                    /* Request receiving context */
	umass_part_t *part;               /* Request receiver partition */
	struct _umass_req_t *prev, *next; /* Doubly linked list */
} umass_req_t;


typedef struct umass_dev {
	idnode_t node; /* Device ID */

	char buffer[8 * UMASS_SECTOR_SIZE];
	usb_devinfo_t instance;
	char path[32];
	int pipeCtrl;
	int pipeIn;
	int pipeOut;
	int fileId;
	int tag;
	unsigned port;
	handle_t lock;

	umass_part_t part; /* TODO extend for more partitions */

	usb_driver_t *drv;
} umass_dev_t;


static struct {
	idtree_t devices;
	umass_dev_t device;

	unsigned msgport;
	handle_t lock;
	rbtree_t fss; /* Registered filesystems */

	umass_req_t *rqueue;   /* Requests FIFO queue */
	handle_t rlock, rcond; /* Requests synchronization */

	bool mount_root;

	/* Message threads stacks */
	char mstacks[UMASS_N_MSG_THREADS][2 * _PAGE_SIZE] __attribute__((aligned(8)));

	/* Pool threads stacks */
	char pstacks[UMASS_N_POOL_THREADS][4 * _PAGE_SIZE] __attribute__((aligned(8)));
} umass_common;


static const usb_device_id_t filters[] = {
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_MASS_STORAGE, USB_SUBCLASS_SCSI, USB_PROTOCOL_BULK },
};


#ifdef UMASS_MOUNT_EXT2
static int umass_registerfs(const char *name, uint8_t type, fs_mount_t mount, fs_unmount_t unmount, fs_handler_t handler)
{
	umass_fs_t *fs = malloc(sizeof(umass_fs_t));
	if (fs == NULL) {
		return -ENOMEM;
	}

	strncpy(fs->name, name, sizeof(fs->name));
	fs->name[sizeof(fs->name) - 1] = '\0';
	fs->type = type;
	fs->mount = mount;
	fs->unmount = unmount;
	fs->handler = handler;
	lib_rbInsert(&umass_common.fss, &fs->node);

	return EOK;
}
#endif


static int umass_cmpfs(rbnode_t *node1, rbnode_t *node2)
{
	umass_fs_t *fs1 = lib_treeof(umass_fs_t, node, node1);
	umass_fs_t *fs2 = lib_treeof(umass_fs_t, node, node2);

	return strcmp(fs1->name, fs2->name);
}


static int umass_scsiRequestSense(umass_dev_t *dev, char *odata);


static int _umass_transmit(umass_dev_t *dev, void *cmd, size_t clen, char *data, size_t dlen, int dir)
{
	scsi_sense_t *sense;
	umass_cbw_t cbw = { 0 };
	umass_csw_t csw = { 0 };
	int ret = -1, bytes = 0;
	int dataPipe;
	int i;

	if (clen > 16)
		return -1;

	for (i = 0; ret < 0 && i < UMASS_TRANSMIT_RETRIES; i++) {
		dataPipe = (dir == usb_dir_out) ? dev->pipeOut : dev->pipeIn;
		cbw.sig = CBW_SIG;
		cbw.tag = dev->tag++;
		cbw.dlen = dlen;
		cbw.flags = (dir == usb_dir_out) ? UMASS_WRITE : UMASS_READ;
		cbw.lun = 0;
		cbw.clen = clen;
		memcpy(cbw.cmd, cmd, clen);

		ret = usb_transferBulk(dev->drv, dev->pipeOut, &cbw, sizeof(cbw), usb_dir_out);
		if (ret != sizeof(cbw)) {
			fprintf(stderr, "umass_transmit: usb_transferBulk OUT failed\n");
			return -EIO;
		}

		/* Optional data transfer */
		if (dlen > 0) {
			ret = usb_transferBulk(dev->drv, dataPipe, data, dlen, dir);
			if (ret < 0) {
				fprintf(stderr, "umass_transmit: umass_transmit data transfer failed\n");
				return ret;
			}
			bytes = ret;
		}

		ret = usb_transferBulk(dev->drv, dev->pipeIn, &csw, sizeof(csw), usb_dir_in);
		if (ret != sizeof(csw)) {
			fprintf(stderr, "umass_transmit: usb_transferBulk IN transfer failed\n");
			return -EIO;
		}

		/* Transfer finished, check transfer correctness */
		if (csw.sig != CSW_SIG || csw.tag != cbw.tag || csw.status != 0) {
			if (csw.status == 1) {
				ret = umass_scsiRequestSense(dev, dev->buffer);
				if (ret < 0) {
					DEBUG("REQUEST SENSE failed: %d", ret);
				}
				else {
					sense = (scsi_sense_t *)dev->buffer;
					DEBUG("REQUEST SENSE code=0x%x, sense key code=0x%x", sense->errorcode, (sense->misc0_sensekey & 0xf));
				}

				/* Retry transfer */
				ret = -1;
				bytes = 0;
				continue;
			}
			else {
				DEBUG("transfer incorrect.\n csw.sig=0x%x, csw.tag=0x%x, cbw.tag=0x%x, csw.status=%d\n", csw.sig, csw.tag,
						cbw.tag, csw.status);
				return -EIO;
			}
		}
	}

	return bytes;
}


/* Left commented out: can be useful when handling devices of other types than direct-access
 * (non-zero "Peripheral Device Type" in INQUIRY) */
#if 0
static int umass_getMaxLUN(umass_dev_t *dev, int ifaceNum, int *maxlun)
{
	usb_setup_packet_t setup = (usb_setup_packet_t) {
		.bmRequestType = REQUEST_DIR_DEV2HOST | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 0xFE, /* Get Max LUN */
		.wValue = 0,
		.wIndex = ifaceNum,
		.wLength = 1,
	};
	int ret;

	ret = usb_transferControl(dev->drv, dev->pipeCtrl, &setup, maxlun, 1, usb_dir_in);
	if (ret < 0) {
		LOG("get max LUN failed");
		dev->maxlun = 0;
		return -EIO;
	}

	return 0;
}
#endif


static int umass_scsiTest(umass_dev_t *dev)
{
	char testCmd[6] = { 0 };
	int ret;

	ret = _umass_transmit(dev, testCmd, sizeof(testCmd), NULL, 0, usb_dir_in);

	return ret == 0 ? 0 : -EIO;
}


static int umass_scsiRequestSense(umass_dev_t *dev, char *odata)
{
	uint8_t len = sizeof(scsi_sense_t);
	int ret;
	scsi_cdb6_t requestSenseCmd = {
		.opcode = SCSI_REQUEST_SENSE,
		.length = htons(len),
	};

	ret = _umass_transmit(dev, &requestSenseCmd, sizeof(requestSenseCmd), (char *)odata, len, usb_dir_in);

	return ret == sizeof(scsi_sense_t) ? 0 : -EIO;
}


static int umass_scsiInquiry(umass_dev_t *dev, char *odata)
{
	uint8_t len = sizeof(scsi_inquiry_t);
	int ret;
	scsi_cdb6_t inquiryCmd = {
		.opcode = SCSI_INQUIRY,
		.length = htons(len),
	};

	ret = _umass_transmit(dev, &inquiryCmd, sizeof(inquiryCmd), odata, len, usb_dir_in);

	return ret == sizeof(scsi_inquiry_t) ? 0 : -EIO;
}


static int _umass_scsiInit(umass_dev_t *dev)
{
	int ret;
	int i, ok;
	scsi_inquiry_t *inquiry;

	for (i = 0, ok = 0; ok < 2 && i < UMASS_INIT_RETRIES; i++) {
		ok = 0;

		ret = umass_scsiTest(dev);
		if (ret == 0) {
			DEBUG("TEST UNIT READY succeeded");
			ok++;
		}

		/* Some real world are said to not work without doing INQUIRY first */
		ret = umass_scsiInquiry(dev, dev->buffer);
		if (ret == 0) {
			inquiry = (scsi_inquiry_t *)dev->buffer;

			DEBUG("INQUIRY succeeded\n"
				  "  device type=%x vendorid=%s\n"
				  "  productid=%s",
					(inquiry->qualifier_devicetype & 0x1f), inquiry->vendorid, inquiry->productid);

			ok++;
		}
		else {
			DEBUG("INQUIRY failed, ret=%d", ret);
		}
	}

	return ok == 2 ? 0 : -1;
}


static int _umass_check(umass_dev_t *dev)
{
	scsi_cdb10_t readcmd = {
		.opcode = 0x28,
		.length = htons(0x1)
	};
	mbr_t *mbr;
	int ret;

	/* Read MBR */
	ret = _umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, UMASS_SECTOR_SIZE, usb_dir_in);
	if (ret < 0) {
		LOG_ERROR("reading MBR failed");
		return -1;
	}

	mbr = (mbr_t *)dev->buffer;
	if (mbr->magic != MBR_MAGIC) {
		LOG_ERROR("bad MBR magic: 0x%x", mbr->magic);
		return -1;
	}

	/* Read only the first partition */
	dev->part.start = mbr->pent[0].start;
	dev->part.sectors = mbr->pent[0].sectors;
	dev->part.fs = NULL;
	dev->part.fdata = NULL;
	dev->part.idx = 0;

	DEBUG("part.start=0x%x, part.sectors=0x%x", dev->part.start, dev->part.sectors);

	return 0;
}


static umass_fs_t *umass_getfs(const char *name)
{
	umass_fs_t fs;

	strncpy(fs.name, name, sizeof(fs.name));
	fs.name[sizeof(fs.name) - 1] = '\0';

	return lib_treeof(umass_fs_t, node, lib_rbFind(&umass_common.fss, &fs.node));
}


/* TODO: handle mounting other filesystems than ext2 */
#if 0
static umass_fs_t *umass_findfs(uint8_t type)
{
	rbnode_t *node;
	umass_fs_t *fs;

	for (node = lib_rbMinimum(umass_common.fss.root); node != NULL; node = lib_rbNext(node)) {
		fs = lib_treeof(umass_fs_t, node, node);

		if (fs->type == type) {
			return fs;
		}
	}

	return NULL;
}
#endif


/* One SCSI READ(10)/WRITE(10) can only move UMASS_MAX_IO bytes, so a larger
 * request must be issued as several commands.
 *
 * Both of these used to `len = min(len, sizeof(dev->buffer))` and return the
 * truncated count as if it were the whole transfer. Nothing reported an error,
 * so the caller simply got less data than it asked for: a 512 B or 4 KiB read
 * was exact, an 8 KiB read silently returned 4 KiB. libext2 reads more than
 * 4 KiB while mounting, saw a short read, and failed the mount with -EIO --
 * which pointed at the device rather than at the driver that shortened it.
 * Measured on hardware: requested 512 -> 512, 4096 -> 4096, 8192 -> 4096,
 * 16384 -> 4096. */
static int umass_readFromDev(umass_dev_t *dev, off_t offs, char *buf, size_t len)
{
	size_t done = 0;

	if (((offs % UMASS_SECTOR_SIZE) != 0) || ((len % UMASS_SECTOR_SIZE) != 0)) {
		return -EINVAL;
	}

	if ((offs + (off_t)len) > (off_t)((off_t)dev->part.sectors * UMASS_SECTOR_SIZE)) {
		return -EINVAL;
	}

	while (done < len) {
		scsi_cdb10_t readcmd = { .opcode = 0x28 };
		size_t chunk = len - done;
		int ret;

		if (chunk > UMASS_MAX_IO) {
			chunk = UMASS_MAX_IO;
		}

		readcmd.lba = htonl((uint32_t)((offs + (off_t)done) / UMASS_SECTOR_SIZE) + dev->part.start);
		readcmd.length = htons((uint16_t)(chunk / UMASS_SECTOR_SIZE));

		mutexLock(dev->lock);
		ret = _umass_transmit(dev, &readcmd, sizeof(readcmd), buf + done, chunk, usb_dir_in);
		mutexUnlock(dev->lock);

		if (ret <= 0) {
			fprintf(stderr, "umass: read failed at offs %jd (+%zu of %zu): %d\n",
				(intmax_t)offs, done, len, ret);
			/* Report what was actually transferred rather than claiming the
			 * whole request: a partial result the caller can see is far better
			 * than a short one it cannot. */
			return (done > 0u) ? (int)done : ret;
		}

		done += (size_t)ret;

		/* A short-but-successful transfer means the device ended the command
		 * early; stop rather than spin. */
		if ((size_t)ret < chunk) {
			break;
		}
	}

	return (int)done;
}


static int umass_writeToDev(umass_dev_t *dev, off_t offs, const char *buf, size_t len)
{
	size_t done = 0;

	if (((offs % UMASS_SECTOR_SIZE) != 0) || ((len % UMASS_SECTOR_SIZE) != 0)) {
		return -EINVAL;
	}

	if ((offs + (off_t)len) > (off_t)((off_t)dev->part.sectors * UMASS_SECTOR_SIZE)) {
		return -EINVAL;
	}

	while (done < len) {
		scsi_cdb10_t writecmd = { .opcode = 0x2a };
		size_t chunk = len - done;
		int ret;

		if (chunk > UMASS_MAX_IO) {
			chunk = UMASS_MAX_IO;
		}

		writecmd.lba = htonl((uint32_t)((offs + (off_t)done) / UMASS_SECTOR_SIZE) + dev->part.start);
		writecmd.length = htons((uint16_t)(chunk / UMASS_SECTOR_SIZE));

		mutexLock(dev->lock);
		ret = _umass_transmit(dev, &writecmd, sizeof(writecmd), (char *)buf + done, chunk, usb_dir_out);
		mutexUnlock(dev->lock);

		if (ret <= 0) {
			fprintf(stderr, "umass: write failed at offs %jd (+%zu of %zu): %d\n",
				(intmax_t)offs, done, len, ret);
			return (done > 0u) ? (int)done : ret;
		}

		done += (size_t)ret;

		if ((size_t)ret < chunk) {
			break;
		}
	}

	return (int)done;
}


static int umass_getattr(umass_dev_t *dev, int type, long long int *attr)
{
	switch (type) {
		case atSize:
			*attr = (long long int)dev->part.sectors * UMASS_SECTOR_SIZE;
			break;

		case atMode:
			/* umount() asks for this FIRST, to tell a mounted device from a
			 * mountpoint (libphoenix sys/mount.c). Answering -EINVAL made
			 * `umount /dev/umass0` fail with EINVAL on a perfectly good mount,
			 * and there is no other way to release one -- umount() by
			 * mountpoint is still an unimplemented TODO there. */
			*attr = (long long int)(S_IFBLK | 0660);
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


/* stat() on the block device. Without this, mtStat fell through to the message
 * loop's default -ENOSYS and userspace saw
 * `cannot fstat '/dev/umass0': Function not implemented` -- so every tool that
 * stats its input before reading it (coreutils dd, cat, cp, ...) failed on a
 * perfectly healthy stick. Same gap, same fix as bcm2711-emmc's sdstorage_srv.c. */
static void umass_stat(umass_dev_t *dev, struct stat *st, const oid_t *oid)
{
	memset(st, 0, sizeof(*st));
	st->st_dev = oid->port;
	st->st_ino = (ino_t)oid->id;
	st->st_rdev = oid->port;
	st->st_mode = S_IFBLK | 0660;
	st->st_nlink = 1;
	st->st_blksize = UMASS_SECTOR_SIZE;
	st->st_size = (off_t)dev->part.sectors * UMASS_SECTOR_SIZE;
	st->st_blocks = (blkcnt_t)dev->part.sectors;
}


static umass_dev_t *_umass_devFind(id_t id)
{
	return lib_treeof(umass_dev_t, node, idtree_find(&umass_common.devices, id));
}


static ssize_t umass_read(id_t id, off_t offs, char *buf, size_t len)
{
	mutexLock(umass_common.lock);
	umass_dev_t *dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);
	if (dev == NULL) {
		return -ENODEV;
	}
	return umass_readFromDev(dev, offs, buf, len);
}


static ssize_t umass_write(id_t id, off_t offs, const char *buf, size_t len)
{
	mutexLock(umass_common.lock);
	umass_dev_t *dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);
	if (dev == NULL) {
		return -ENODEV;
	}
	return umass_writeToDev(dev, offs, buf, len);
}


static int umass_mountFromDev(umass_dev_t *dev, const char *name, oid_t *oid)
{
	umass_fs_t *fs;
	int err;

	if (dev == NULL) {
		return -ENODEV;
	}

	if (dev->part.fs != NULL) {
		return -EEXIST;
	}

	fs = umass_getfs(name);
	if (fs == NULL) {
		return -ENOENT;
	}
	dev->part.fs = fs;

	err = portCreate(&dev->part.port);
	if (err != 0) {
		fprintf(stderr, "umass: Can't create partition port!\n");
		dev->part.fs = NULL;
		return -EIO;
	}

	oid->port = dev->part.port;
	oid->id = dev->fileId;

	err = fs->mount(oid, UMASS_SECTOR_SIZE, umass_read, umass_write, &dev->part.fdata);
	if (err < 0) {
		/* Clearing part.fs here is the point: it is the "already mounted" flag
		 * this function tests on entry, so leaving it set after a FAILED mount
		 * made the device permanently unmountable -- one failed attempt and
		 * every later mount returned -EEXIST, which reads like the device is
		 * busy rather than like the first attempt went wrong. */
		dev->part.fs = NULL;
		dev->part.fdata = NULL;
		portDestroy(dev->part.port);
		return err;
	}
	oid->id = err;

	dev->part.fsthrExit = 0;
	dev->part.fsthrRunning = 1;
	err = beginthread(umass_fsthr, 4, dev->part.fsstack, sizeof(dev->part.fsstack), &dev->part);
	if (err < 0) {
		dev->part.fsthrRunning = 0;
		dev->part.fs->unmount(dev->part.fdata);
		dev->part.fs = NULL;
		dev->part.fdata = NULL;
		portDestroy(dev->part.port);
		return err;
	}

	return EOK;
}


static int umass_mount(id_t id, const char *name, oid_t *oid)
{
	umass_dev_t *dev;
	int ret;

	mutexLock(umass_common.lock);
	dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);

	/* Name which check failed. Both "no such device id" and "no such filesystem"
	 * return -ENOENT, and mount(2) surfaces either as a bare
	 * "No such file or directory" -- indistinguishable from the path not
	 * existing, which is where this was misread twice. */
	if (dev == NULL) {
		fprintf(stderr, "umass: mount: no device with id %llu\n", (unsigned long long)id);
		return -ENOENT;
	}

	if (umass_getfs(name) == NULL) {
		fprintf(stderr, "umass: mount: filesystem '%s' is not registered\n", name);
		return -ENOENT;
	}

	ret = umass_mountFromDev(dev, name, oid);
	if (ret < 0) {
		fprintf(stderr, "umass: mount: %s on id %llu failed: %s (%d)\n",
			name, (unsigned long long)id, strerror(-ret), ret);
	}

	return ret;
}


static void umass_fsthr(void *arg)
{
	umass_part_t *part = (umass_part_t *)arg;
	umass_req_t *req;
	int umount = 0, ret;

	for (;;) {
		/* TODO: use static requests buffer? */
		req = malloc(sizeof(umass_req_t));
		if (req == NULL) {
			continue;
		}

		req->part = part;

		ret = -1;
		while (ret < 0) {
			if (part->fsthrExit != 0) {
				/* Torn down underneath us (device unplugged). The port is gone,
				 * so msgRecv will keep failing -- leave instead of spinning. */
				free(req);
				part->fsthrRunning = 0;
				endthread();
			}
			ret = msgRecv(req->part->port, &req->msg, &req->rid);
		}

		if (req->msg.type == mtUmount) {
			umount = 1;
		}

		mutexLock(umass_common.rlock);

		LIST_ADD(&umass_common.rqueue, req);

		condSignal(umass_common.rcond);
		mutexUnlock(umass_common.rlock);

		if (umount != 0) {
			part->fsthrRunning = 0;
			endthread();
		}
	}
}


static void umass_poolthr(void *arg)
{
	umass_req_t *req;

	for (;;) {
		mutexLock(umass_common.rlock);

		while (umass_common.rqueue == NULL) {
			condWait(umass_common.rcond, umass_common.rlock, 0);
		}

		/*
		 * FIFO - assumes LIST_ADD/REMOVE set the queue pointer to the
		 * element added first
		 */
		req = umass_common.rqueue;
		LIST_REMOVE(&umass_common.rqueue, req);

		mutexUnlock(umass_common.rlock);

		if (req->msg.type == mtUmount) {
			req->part->fs->unmount(req->part->fdata);
			req->part->fs = NULL;
			req->part->fdata = NULL;
		}
		else {
			req->part->fs->handler(req->part->fdata, &req->msg);
		}

		msgRespond(req->part->port, &req->msg, req->rid);
		free(req);
	}
}


static void umass_msgthr(void *arg)
{
	umass_dev_t *dev;
	msg_rid_t rid;
	msg_t msg;
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	for (;;) {
		if (msgRecv(umass_common.msgport, &msg, &rid) < 0)
			break;

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		mutexLock(umass_common.lock);
		dev = _umass_devFind(msg.oid.id);
		mutexUnlock(umass_common.lock);

		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtMount:
				imnt = (mount_i_msg_t *)msg.i.raw;
				omnt = (mount_o_msg_t *)msg.o.raw;
				msg.o.err = umass_mount(msg.oid.id, imnt->fstype, &omnt->oid);
				if (msg.o.err >= 0) {
					/* mount() tells us which directory it spliced us onto. It
					 * was being thrown away, and umount() needs it back via
					 * mtMountPoint -- without it there is no way to release a
					 * mounted device at all. */
					dev->part.mnt = imnt->mnt;
				}
				break;

			case mtMountPoint:
				omnt = (mount_o_msg_t *)msg.o.raw;
				if (dev->part.fs == NULL) {
					msg.o.err = -ENOENT;
				}
				else {
					omnt->oid = dev->part.mnt;
					msg.o.err = EOK;
				}
				break;

			case mtUmount:
				/* umount() sends this to the DEVICE port; umass_fsthr expects
				 * one on the PARTITION port. Bridge them -- WITHOUT holding
				 * umass_common.lock, see _umass_partUnmount. */
				msg.o.err = _umass_partUnmount(dev);
				break;

			case mtRead:
				msg.o.err = umass_readFromDev(dev, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = umass_writeToDev(dev, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.err = umass_getattr(dev, msg.i.attr.type, &msg.o.attr.val);
				break;

			case mtStat:
				if ((msg.o.data == NULL) || (msg.o.size < sizeof(struct stat))) {
					msg.o.err = -EINVAL;
				}
				else {
					umass_stat(dev, msg.o.data, &msg.oid);
					msg.o.err = EOK;
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(umass_common.msgport, &msg, rid);
	}

	endthread();
}


/* Stop the filesystem thread and release the mount.
 *
 * ⛔ MUST NOT be called with umass_common.lock held. That was the bug behind the
 * umount hang: this sends a message whose completion path runs
 * fs->unmount() -> libext2 flush -> umass_write(), and umass_write() takes
 * umass_common.lock. Holding it across the synchronous round-trip deadlocks the
 * driver against itself. Nothing else in the message loop holds it either --
 * mtRead/mtWrite drop it before doing any work.
 *
 * ⛔ And do NOT portDestroy() to break umass_fsthr out of msgRecv: it is blocked
 * INSIDE msgRecv, not at the loop test, so the port vanishing neither wakes it
 * nor lets it re-check the exit flag. Tried on hardware; it hung.
 *
 * umass_fsthr already knows how to leave -- it exits on an mtUmount arriving on
 * its OWN port, and umass_poolthr does the actual fs->unmount() and clears
 * fs/fdata when it processes that message. So ask, wait, then take the port. */
static int _umass_partUnmount(umass_dev_t *dev)
{
	unsigned tries;

	if (dev->part.fs == NULL) {
		return 0;
	}

	dev->part.fsthrExit = 1;

	if (dev->part.fsthrRunning != 0) {
		msg_t msg = { 0 };

		msg.type = mtUmount;
		(void)msgSend(dev->part.port, &msg);

		for (tries = 0u; (tries < 1000u) && (dev->part.fsthrRunning != 0); ++tries) {
			usleep(1000);
		}

		if (dev->part.fsthrRunning != 0) {
			return -EBUSY;
		}

		/* umass_poolthr has already unmounted and cleared fs/fdata -- that is
		 * what it does with an mtUmount. Unmounting again here would free
		 * libext2's state twice. Only the port is left. */
		portDestroy(dev->part.port);

		return 0;
	}

	/* Thread never started (a mount that failed part-way): nothing has
	 * unmounted anything, so do it here. */
	if (dev->part.fs->unmount != NULL) {
		(void)dev->part.fs->unmount(dev->part.fdata);
	}

	portDestroy(dev->part.port);
	dev->part.fs = NULL;
	dev->part.fdata = NULL;

	return 0;
}


static void _umass_devFree(umass_dev_t *dev)
{
	/* A mounted device being freed is the hot-UNPLUG path, and it used to be a
	 * use-after-free: umass_fsthr runs on dev->part.fsstack, which lives INSIDE
	 * this struct, and nothing stopped it. The filesystem was never unmounted
	 * and the partition port was never destroyed either, so any client still
	 * holding the mount kept messaging a dead port.
	 *
	 * The thread only ever left on an mtUmount message and retried msgRecv
	 * forever on error, so destroying the port alone would have spun it. Hence
	 * the explicit flag: set it, destroy the port to break the thread out of
	 * msgRecv, then wait for it to acknowledge before touching the memory. */
	/* idtree_remove is idempotent enough to call twice here: the unplug path
	 * unlinks before dropping the lock, the alloc-failure paths have not. */
	if (_umass_partUnmount(dev) < 0) {
		/* Never observed; say so rather than free memory a live thread is
		 * running on. Leaking one device struct beats corrupting the heap. */
		fprintf(stderr, "umass: fs thread for %s did not exit; leaking its state\n", dev->path);
		idtree_remove(&umass_common.devices, &dev->node);
		return;
	}

	idtree_remove(&umass_common.devices, &dev->node);
	resourceDestroy(dev->lock);
	free(dev);
}


static umass_dev_t *_umass_devAlloc(void)
{
	umass_dev_t *dev;
	int rv;

	dev = malloc(sizeof(umass_dev_t));
	if (dev == NULL) {
		fprintf(stderr, "umass: Not enough memory\n");
		return NULL;
	}

	rv = mutexCreate(&dev->lock);
	if (rv < 0) {
		free(dev);
		return NULL;
	}

	idtree_alloc(&umass_common.devices, &dev->node);
	dev->fileId = idtree_id(&dev->node);

	rv = snprintf(dev->path, sizeof(dev->path), "/dev/umass%d", dev->fileId);
	if (rv < 0 || rv >= sizeof(dev->path)) {
		_umass_devFree(dev);
		return NULL;
	}

	return dev;
}


static int umass_mountRoot(umass_dev_t *dev)
{
#ifdef UMASS_MOUNT_EXT2
	int err;
	oid_t roid;

	err = umass_mountFromDev(dev, LIBEXT2_NAME, &roid);
	if (err < 0) {
		return err;
	}

	err = portRegister(roid.port, "/", &roid);
	if (err < 0) {
		return err;
	}

	return EOK;
#else
	return -ENOSYS;
#endif
}


static int umass_handleInsertion(usb_driver_t *drv, usb_devinfo_t *insertion, usb_event_insertion_t *event)
{
	int err;
	umass_dev_t *dev;
	oid_t oid;

	mutexLock(umass_common.lock);

	do {
		dev = _umass_devAlloc();
		if (dev == NULL) {
			fprintf(stderr, "umass: devAlloc failed\n");
			err = -ENOMEM;
			break;
		}

		dev->drv = drv;
		dev->instance = *insertion;
		dev->pipeCtrl = usb_open(drv, insertion, usb_transfer_control, 0);
		if (dev->pipeCtrl < 0) {
			fprintf(stderr, "umass: usb_open failed\n");
			_umass_devFree(dev);
			err = -EINVAL;
			break;
		}

		err = usb_setConfiguration(drv, dev->pipeCtrl, 1);
		if (err != 0) {
			fprintf(stderr, "umass: setConfiguration failed\n");
			_umass_devFree(dev);
			break;
		}

		dev->pipeIn = usb_open(drv, insertion, usb_transfer_bulk, usb_dir_in);
		if (dev->pipeIn < 0) {
			fprintf(stderr, "umass: pipe open failed \n");
			_umass_devFree(dev);
			err = -EINVAL;
			break;
		}

		dev->pipeOut = usb_open(drv, insertion, usb_transfer_bulk, usb_dir_out);
		if (dev->pipeOut < 0) {
			fprintf(stderr, "umass: pipe open failed\n");
			_umass_devFree(dev);
			err = -EINVAL;
			break;
		}
		dev->tag = 0;

		err = _umass_scsiInit(dev);
		if (err < 0) {
			fprintf(stderr, "umass: device didn't initialize properly after scsi init sequence\n");
			_umass_devFree(dev);
			break;
		}

		err = _umass_check(dev);
		if (err < 0) {
			fprintf(stderr, "umass: umass_check failed\n");
			_umass_devFree(dev);
			break;
		}

		oid.port = umass_common.msgport;
		oid.id = dev->fileId;
		err = create_dev(&oid, dev->path);
		if (err != 0) {
			fprintf(stderr, "usb: Can't create dev!\n");
			_umass_devFree(dev);
			break;
		}

		printf("umass: New USB Mass Storage device: %s sectors: %d\n", dev->path, dev->part.sectors);

		event->deviceCreated = true;
		event->dev = oid;
		strncpy(event->devPath, dev->path, sizeof(event->devPath));
		event->devPath[sizeof(event->devPath) - 1] = '\0';
	} while (0);

	mutexUnlock(umass_common.lock);

	if (err == 0 && umass_common.mount_root) {
		err = umass_mountRoot(dev);
		if (err < 0) {
			fprintf(stderr, "umass: failed to mount root partition: %s (%d)\n", strerror(-err), err);
			return err;
		}
		umass_common.mount_root = false; /* don't try to mount root again */
	}

	return err;
}


static int umass_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	umass_dev_t *dev;
	rbnode_t *node, *next;

	/* Tear each match down OUTSIDE umass_common.lock.
	 *
	 * _umass_devFree -> _umass_partUnmount sends a message whose completion path
	 * runs fs->unmount() -> libext2 flush -> umass_write(), and umass_write()
	 * takes this very lock. Holding it here would deadlock the driver against
	 * itself on every unplug of a MOUNTED stick -- the same self-deadlock that
	 * hung umount until it was root-caused.
	 *
	 * So: find one match and unlink it from the tree under the lock (which makes
	 * it unreachable to anyone else), then drop the lock and tear it down.
	 * Repeat until no matches remain. */
	for (;;) {
		umass_dev_t *victim = NULL;

		mutexLock(umass_common.lock);

		node = lib_rbMinimum(umass_common.devices.root);
		while (node != NULL) {
			next = lib_rbNext(node);

			dev = lib_treeof(umass_dev_t, node, lib_treeof(idnode_t, linkage, node));
			if ((dev->instance.bus == del->bus) && (dev->instance.dev == del->dev) &&
					(dev->instance.interface == del->interface)) {
				idtree_remove(&umass_common.devices, &dev->node);
				victim = dev;
				break;
			}

			node = next;
		}

		mutexUnlock(umass_common.lock);

		if (victim == NULL) {
			break;
		}

		remove(victim->path);
		fprintf(stderr, "umass: Device removed: %s\n", victim->path);
		_umass_devFree(victim);
	}

	return 0;
}


static int umass_handleCompletion(usb_driver_t *drv, usb_completion_t *c, const char *data, size_t len)
{
	return EOK;
}


static int umass_init(usb_driver_t *drv, void *args)
{
	umass_args_t *umass_args = (umass_args_t *)args;
	int ret, i;

	/* No args means hosted inside the usb daemon (USB_HOSTDRV_LIBS), which calls
	 * ops.init(drv, NULL). That must NOT imply "mount the first stick somebody
	 * plugs in as the system root": on a Pi 4 the root is already NFS or the SD
	 * card, the attempt fails, and -- see umass_mountFromDev -- the failure used
	 * to leave dev->part.fs set, so every later explicit mount of that device
	 * returned -EEXIST. Root-on-USB stays opt-in via the standalone server's -r,
	 * which is what umass_args_t already defaults to (srv.c: .mount_root = false);
	 * the NULL-args path was contradicting the program's own default. */
	umass_common.mount_root = (umass_args != NULL) ? umass_args->mount_root : false;

	do {
		ret = mutexCreate(&umass_common.rlock);
		if (ret < 0) {
			fprintf(stderr, "umass: failed to create server requests mutex\n");
			break;
		}

		ret = condCreate(&umass_common.rcond);
		if (ret < 0) {
			fprintf(stderr, "umass: failed to create server requests condition variable\n");
			break;
		}

		umass_common.rqueue = NULL;
		idtree_init(&umass_common.devices);
		lib_rbInit(&umass_common.fss, umass_cmpfs, NULL);

#ifdef UMASS_MOUNT_EXT2
		/* Register filesystems */
		ret = umass_registerfs(LIBEXT2_NAME, LIBEXT2_TYPE, LIBEXT2_MOUNT, LIBEXT2_UNMOUNT, LIBEXT2_HANDLER);
		if (ret < 0) {
			fprintf(stderr, "umass: failed to register ext2 filesystem\n");
			break;
		}
#endif

		/* Port for communication with driver clients */
		ret = portCreate(&umass_common.msgport);
		if (ret != 0) {
			fprintf(stderr, "umass: Can't create message port!\n");
			break;
		}

		ret = mutexCreate(&umass_common.lock);
		if (ret < 0) {
			fprintf(stderr, "umass: Can't create mutex!\n");
			break;
		}

		/* Run message threads */
		for (i = 0; i < sizeof(umass_common.mstacks) / sizeof(umass_common.mstacks[0]); i++) {
			ret = beginthread(umass_msgthr, UMASS_MSG_PRIO, umass_common.mstacks[i], sizeof(umass_common.mstacks[i]), NULL);
			if (ret != 0) {
				fprintf(stderr, "umass: fail to beginthread ret: %d\n", ret);
				break;
			}
		}

		/* Run pool threads */
		for (i = 0; i < sizeof(umass_common.pstacks) / sizeof(umass_common.pstacks[0]); i++) {
			ret = beginthread(umass_poolthr, UMASS_POOL_PRIO, umass_common.pstacks[i], sizeof(umass_common.pstacks[i]), NULL);
			if (ret < 0) {
				fprintf(stderr, "umass: failed to start pool thread %d\n", i);
				break;
			}
		}

		ret = EOK;
	} while (0);

	return ret;
}


static int umass_destroy(usb_driver_t *drv)
{
	return EOK;
}


static usb_driver_t umass_driver = {
	.name = "umass",
	.handlers = {
		.insertion = umass_handleInsertion,
		.deletion = umass_handleDeletion,
		.completion = umass_handleCompletion,
	},
	.ops = { .init = umass_init, .destroy = umass_destroy },
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = (void *)&umass_common,
};


__attribute__((constructor)) static void umass_register(void)
{
	usb_driverRegister(&umass_driver);
}
