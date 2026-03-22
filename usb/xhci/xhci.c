/*
 * Phoenix-RTOS
 *
 * xHCI USB Host Controller skeleton
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/minmax.h>
#include <sys/threads.h>
#include <unistd.h>

#include <hcd.h>
#include <hub.h>


#define XHCI_MAP_SIZE            0x10000u
#define XHCI_REG_CAP_CAPLENGTH   0x00u
#define XHCI_REG_CAP_HCIVERSION  0x02u
#define XHCI_REG_CAP_HCSPARAMS1  0x04u
#define XHCI_REG_CAP_HCSPARAMS2  0x08u
#define XHCI_REG_CAP_HCCPARAMS1  0x10u
#define XHCI_REG_CAP_DBOFF       0x14u
#define XHCI_REG_CAP_RTSOFF      0x18u
#define XHCI_REG_CAP_HCSPARAMS1_MAX_SLOTS__MASK 0xffu
#define XHCI_REG_CAP_HCSPARAMS1_MAX_INTRS__SHIFT 8u
#define XHCI_REG_CAP_HCSPARAMS1_MAX_INTRS__MASK (0x7ffu << XHCI_REG_CAP_HCSPARAMS1_MAX_INTRS__SHIFT)
#define XHCI_REG_CAP_HCSPARAMS1_MAX_PORTS__SHIFT 24u
#define XHCI_REG_CAP_HCSPARAMS1_MAX_PORTS__MASK  (0xffu << XHCI_REG_CAP_HCSPARAMS1_MAX_PORTS__SHIFT)
#define XHCI_REG_CAP_HCSPARAMS2_IST__MASK 0xfu
#define XHCI_REG_CAP_HCSPARAMS2_ERST_MAX__SHIFT 4u
#define XHCI_REG_CAP_HCSPARAMS2_ERST_MAX__MASK (0xfu << XHCI_REG_CAP_HCSPARAMS2_ERST_MAX__SHIFT)
#define XHCI_REG_CAP_HCSPARAMS2_SPR (1u << 26)
#define XHCI_REG_CAP_HCSPARAMS2_MAX_SCRATCHPAD_BUFS__SHIFT 27u
#define XHCI_REG_CAP_HCSPARAMS2_MAX_SCRATCHPAD_BUFS__MASK  (0x1fu << XHCI_REG_CAP_HCSPARAMS2_MAX_SCRATCHPAD_BUFS__SHIFT)
#define XHCI_REG_CAP_HCCPARAMS1_AC64 (1u << 0)
#define XHCI_REG_CAP_HCCPARAMS1_CSZ (1u << 2)
#define XHCI_REG_CAP_HCCPARAMS1_MAX_PSA_SIZE__SHIFT 12u
#define XHCI_REG_CAP_HCCPARAMS1_MAX_PSA_SIZE__MASK (0xfu << XHCI_REG_CAP_HCCPARAMS1_MAX_PSA_SIZE__SHIFT)
#define XHCI_REG_CAP_DBOFF__MASK 0xfffffffcu
#define XHCI_REG_CAP_RTSOFF__MASK 0xffffffe0u
#define XHCI_REG_OP_USBCMD       0x00u
#define XHCI_REG_OP_USBSTS       0x04u
#define XHCI_REG_OP_PAGESIZE     0x08u
#define XHCI_REG_OP_CRCR         0x18u
#define XHCI_REG_OP_CRCR_HI      0x1cu
#define XHCI_REG_OP_DCBAAP       0x30u
#define XHCI_REG_OP_DCBAAP_HI    0x34u
#define XHCI_REG_OP_CONFIG       0x38u
#define XHCI_REG_OP_PORTS_BASE   0x400u
#define XHCI_REG_OP_PORT__SIZE   0x10u
#define XHCI_REG_OP_PORT_PORTSC  0x00u
#define XHCI_REG_RT_IR0          0x20u
#define XHCI_REG_RT_IR_IMAN      0x00u
#define XHCI_REG_RT_IR_ERSTSZ    0x08u
#define XHCI_REG_RT_IR_ERSTBA_LO 0x10u
#define XHCI_REG_RT_IR_ERSTBA_HI 0x14u
#define XHCI_REG_RT_IR_ERDP_LO   0x18u
#define XHCI_REG_RT_IR_ERDP_HI   0x1cu
#define XHCI_REG_RT_IR__SIZE     0x20u
#define XHCI_SUPPORTED_VERSION   0x0100u
#define XHCI_REG_OP_USBCMD_RS    (1u << 0)
#define XHCI_REG_OP_USBCMD_HCRST (1u << 1)
#define XHCI_REG_OP_USBSTS_HCH   (1u << 0)
#define XHCI_REG_OP_USBSTS_HSE   (1u << 2)
#define XHCI_REG_OP_USBSTS_CNR   (1u << 11)
#define XHCI_REG_OP_USBSTS_HCE   (1u << 12)
#define XHCI_REG_OP_PAGESIZE_4K  (1u << 0)
#define XHCI_REG_OP_CRCR_RCS     (1u << 0)
#define XHCI_REG_OP_CRCR_CS      (1u << 1)
#define XHCI_REG_OP_CRCR_CA      (1u << 2)
#define XHCI_REG_OP_CRCR_CRR     (1u << 3)
#define XHCI_REG_OP_CRCR_CR_PTR_LO__MASK 0xffffffc0u
#define XHCI_REG_OP_DCBAAP__MASK 0xffffffc0u
#define XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK 0xffu
#define XHCI_REG_OP_PORT_PORTSC_CCS (1u << 0)
#define XHCI_REG_OP_PORT_PORTSC_PED (1u << 1)
#define XHCI_REG_OP_PORT_PORTSC_OCA (1u << 3)
#define XHCI_REG_OP_PORT_PORTSC_PR  (1u << 4)
#define XHCI_REG_OP_PORT_PORTSC_PLS__SHIFT 5u
#define XHCI_REG_OP_PORT_PORTSC_PLS__MASK  (0xfu << XHCI_REG_OP_PORT_PORTSC_PLS__SHIFT)
#define XHCI_REG_OP_PORT_PORTSC_PLS_U0     0u
#define XHCI_REG_OP_PORT_PORTSC_PP  (1u << 9)
#define XHCI_REG_OP_PORT_PORTSC_PORT_SPEED__SHIFT 10u
#define XHCI_REG_OP_PORT_PORTSC_PORT_SPEED__MASK  (0xfu << XHCI_REG_OP_PORT_PORTSC_PORT_SPEED__SHIFT)
#define XHCI_REG_OP_PORT_PORTSC_CSC (1u << 17)
#define XHCI_REG_OP_PORT_PORTSC_PEC (1u << 18)
#define XHCI_REG_OP_PORT_PORTSC_OCC (1u << 20)
#define XHCI_REG_OP_PORT_PORTSC_PRC (1u << 21)
#define XHCI_REG_OP_PORT_PORTSC_RW1C (XHCI_REG_OP_PORT_PORTSC_CSC | XHCI_REG_OP_PORT_PORTSC_PEC | \
	XHCI_REG_OP_PORT_PORTSC_OCC | XHCI_REG_OP_PORT_PORTSC_PRC)
#define XHCI_REG_RT_IR_ERSTSZ__MASK 0xffffu
#define XHCI_REG_RT_IR_ERSTBA_LO__MASK 0xffffffc0u
#define XHCI_REG_RT_IR_ERDP_LO_EHB (1u << 3)
#define XHCI_REG_RT_IR_ERDP_LO__MASK 0xfffffff0u
#define XHCI_DCBAA_ALIGN         0x1000u
#define XHCI_DCBAA_SIZE          0x1000u
#define XHCI_CMD_RING_ALIGN      0x10000u
#define XHCI_CMD_RING_SIZE       0x10000u
#define XHCI_EVENT_RING_ALIGN    0x1000u
#define XHCI_EVENT_RING_SIZE     0x1000u
#define XHCI_ERST_ALIGN          0x40u
#define XHCI_ERST_SIZE           0x40u
#define XHCI_ERST_ENTRY_COUNT    1u
#define XHCI_TRB_SIZE            16u
#define XHCI_CNR_TIMEOUT_MS      100u
#define XHCI_HCRST_TIMEOUT_MS    20u
#define XHCI_RUNSTOP_TIMEOUT_MS  20u
#define XHCI_TRB_CONTROL_C       (1u << 0)
#define XHCI_TRB_CONTROL_TRB_TYPE__SHIFT 10u
#define XHCI_TRB_TYPE_LINK       6u
#define XHCI_TRB_TYPE_CMD_ENABLE_SLOT 9u
#define XHCI_TRB_TYPE_CMD_NO_OP  23u
#define XHCI_TRB_TYPE_EVENT_CMD_COMPLETION 33u
#define XHCI_LINK_TRB_CONTROL_TC (1u << 1)
#define XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT 24u
#define XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK (0xffu << XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT)
#define XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT 24u
#define XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__MASK (0xffu << XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT)
#define XHCI_TRB_COMPLETION_CODE_SUCCESS 1u
#define XHCI_CMD_TIMEOUT_MS      100u
#define XHCI_PORT_RESET_TIMEOUT_MS 100u
#define XHCI_PORT_POWER_GOOD_DELAY_US 20000u


static const struct {
	usb_device_desc_t dev;
	usb_configuration_desc_t cfg;
	usb_interface_desc_t iface;
	usb_endpoint_desc_t ep;
	const char langID[2];
	const char product[16];
	const char manufacturer[16];
} __attribute__((packed)) xhci_descs = {
	{
		.bLength = sizeof(xhci_descs.dev),
		.bcdUSB = 0x0300,
		.bDeviceClass = USB_CLASS_HUB,
		.bDeviceSubClass = 0,
		.bDeviceProtocol = USB_HUB_PROTO_ROOT,
		.bMaxPacketSize0 = 64,
		.idVendor = 0x0,
		.idProduct = 0x0,
		.bcdDevice = 0x0,
		.iManufacturer = 2,
		.iProduct = 1,
		.iSerialNumber = 0,
		.bNumConfigurations = 1,
	},
	{
		.bLength = sizeof(xhci_descs.cfg),
		.bDescriptorType = USB_DESC_CONFIG,
		.wTotalLength = sizeof(xhci_descs.cfg) + sizeof(xhci_descs.iface) + sizeof(xhci_descs.ep),
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0,
		.bMaxPower = 0,
	},
	{
		.bLength = sizeof(xhci_descs.iface),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_HUB,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	{
		.bLength = sizeof(xhci_descs.ep),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = USB_ENDPT_DIR_IN | (1 << 7),
		.bmAttributes = USB_ENDPT_TYPE_INTR,
		.wMaxPacketSize = 4,
		.bInterval = 0xff,
	},
	{ 0x04, 0x09 },
	"3.0 root hub",
	"Phoenix Systems"
};


typedef struct {
	uint64_t parameter;
	uint32_t status;
	uint32_t control;
} __attribute__((packed)) xhci_trb_t;


typedef struct {
	uint64_t ringSegmentBase;
	uint32_t ringSegmentSize;
	uint32_t reserved;
} __attribute__((packed)) xhci_erst_entry_t;


typedef struct {
	void *mmio;
	size_t mapSz;
	char statusStack[2048] __attribute__((aligned(8)));
	uint8_t caplength;
	uint16_t version;
	uint32_t hcsparams1;
	uint32_t hcsparams2;
	uint32_t hccparams1;
	uint32_t dboff;
	uint32_t rtsoff;
	uint32_t pagesize;
	uint32_t nintrs;
	uint32_t nslots;
	uint32_t nports;
	uint32_t erstMax;
	uint32_t ist;
	uint32_t nscratchpad;
	uint32_t maxPsaSize;
	uint32_t contextSize;
	uint32_t crcrLo;
	uint32_t crcrHi;
	uint32_t dcbaapLo;
	uint32_t dcbaapHi;
	uint32_t config;
	uint64_t crcr;
	uint64_t dcbaap;
	void *dcbaa;
	void *cmdRing;
	xhci_trb_t *cmdRingTrbs;
	size_t dcbaaSize;
	size_t cmdRingSize;
	uint64_t dcbaaPhys;
	uint64_t cmdRingPhys;
	void *eventRing;
	void *erst;
	size_t eventRingSize;
	size_t erstSize;
	uint64_t eventRingPhys;
	uint64_t erstPhys;
	uint32_t cmdRingCount;
	uint32_t cmdCycleState;
	uint32_t eventRingTrbs;
	uint32_t eventCycleState;
	uint8_t slotId;
	uint32_t erstsz;
	uint32_t erstbaLo;
	uint32_t erstbaHi;
	uint32_t erdpLo;
	uint32_t erdpHi;
	uint64_t erstba;
	uint64_t erdp;
	unsigned ac64 : 1;
	unsigned spr : 1;
} xhci_t;


static uint32_t xhci_getHubStatus(usb_dev_t *hub);
static int xhci_cmdExec(xhci_t *xhci, uint64_t parameter, uint32_t status, uint32_t control, uint8_t *slotId);


static inline uint8_t xhci_read8(xhci_t *xhci, uintptr_t off)
{
	volatile uint8_t *base = (volatile uint8_t *)xhci->mmio;

	return *(base + off);
}


static inline uint16_t xhci_read16(xhci_t *xhci, uintptr_t off)
{
	volatile uint16_t *reg = (volatile uint16_t *)((volatile uint8_t *)xhci->mmio + off);

	return *reg;
}


static inline uint32_t xhci_read32(xhci_t *xhci, uintptr_t off)
{
	volatile uint32_t *reg = (volatile uint32_t *)((volatile uint8_t *)xhci->mmio + off);

	return *reg;
}


static inline uint32_t xhci_opRead32(xhci_t *xhci, uintptr_t off)
{
	return xhci_read32(xhci, xhci->caplength + off);
}


static inline void xhci_opWrite32(xhci_t *xhci, uintptr_t off, uint32_t val)
{
	volatile uint32_t *reg = (volatile uint32_t *)((volatile uint8_t *)xhci->mmio + xhci->caplength + off);

	*reg = val;
}


static inline uint32_t xhci_rtRead32(xhci_t *xhci, uintptr_t off)
{
	return xhci_read32(xhci, xhci->rtsoff + XHCI_REG_RT_IR0 + off);
}


static inline void xhci_rtWrite32(xhci_t *xhci, uintptr_t off, uint32_t val)
{
	volatile uint32_t *reg = (volatile uint32_t *)((volatile uint8_t *)xhci->mmio + xhci->rtsoff + XHCI_REG_RT_IR0 + off);

	*reg = val;
}


static inline void xhci_dbWrite32(xhci_t *xhci, uintptr_t off, uint32_t val)
{
	volatile uint32_t *reg = (volatile uint32_t *)((volatile uint8_t *)xhci->mmio + xhci->dboff + off);

	*reg = val;
}


static inline uint32_t xhci_portRead32(xhci_t *xhci, unsigned int port, uintptr_t off)
{
	return xhci_opRead32(xhci, XHCI_REG_OP_PORTS_BASE + ((port - 1u) * XHCI_REG_OP_PORT__SIZE) + off);
}


static inline void xhci_portWrite32(xhci_t *xhci, unsigned int port, uintptr_t off, uint32_t val)
{
	xhci_opWrite32(xhci, XHCI_REG_OP_PORTS_BASE + ((port - 1u) * XHCI_REG_OP_PORT__SIZE) + off, val);
}


static int xhci_portWaitBits(xhci_t *xhci, unsigned int port, uint32_t mask, uint32_t value, unsigned int timeoutMs)
{
	uint32_t reg;

	for (; timeoutMs > 0u; --timeoutMs) {
		reg = xhci_portRead32(xhci, port, XHCI_REG_OP_PORT_PORTSC);
		if ((reg & mask) == value) {
			return EOK;
		}

		usleep(1000);
	}

	return -ETIMEDOUT;
}


static void xhci_destroy(xhci_t *xhci)
{
	if (xhci == NULL) {
		return;
	}

	if (xhci->mmio != MAP_FAILED) {
		munmap(xhci->mmio, xhci->mapSz);
	}

	if (xhci->dcbaa != NULL) {
		usb_freeAligned(xhci->dcbaa, xhci->dcbaaSize);
	}

	if (xhci->cmdRing != NULL) {
		usb_freeAligned(xhci->cmdRing, xhci->cmdRingSize);
	}

	if (xhci->eventRing != NULL) {
		usb_freeAligned(xhci->eventRing, xhci->eventRingSize);
	}

	if (xhci->erst != NULL) {
		usb_freeAligned(xhci->erst, xhci->erstSize);
	}

	free(xhci);
}


static void xhci_roothubStatusThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	usb_dev_t *hub;
	uint32_t status;

	for (;;) {
		hub = hcd->roothub;
		if ((hub != NULL) && (hub->statusTransfer != NULL) && !usb_transferCheck(hub->statusTransfer)) {
			status = xhci_getHubStatus(hub);
			if (status != 0u) {
				memcpy(hub->statusTransfer->buffer, &status, sizeof(status));
				usb_transferFinished(hub->statusTransfer, hub->statusTransfer->size);
			}
		}

		usleep(100000);
	}
}


static int xhci_map(hcd_t *hcd, xhci_t **xhcip)
{
	xhci_t *xhci;
	off_t offs;

	xhci = calloc(1, sizeof(*xhci));
	if (xhci == NULL) {
		return -ENOMEM;
	}

	xhci->mmio = MAP_FAILED;
	xhci->mapSz = XHCI_MAP_SIZE;

	offs = hcd->info->hcdaddr % _PAGE_SIZE;
	xhci->mmio = mmap(NULL, xhci->mapSz, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
	if (xhci->mmio == MAP_FAILED) {
		xhci_destroy(xhci);
		return -ENOMEM;
	}

	hcd->base = (volatile uint32_t *)((uintptr_t)xhci->mmio + offs);
	hcd->priv = xhci;
	*xhcip = xhci;

	return EOK;
}


static int xhci_capProbe(hcd_t *hcd, xhci_t *xhci)
{
	(void)hcd;

	xhci->caplength = xhci_read8(xhci, XHCI_REG_CAP_CAPLENGTH);
	xhci->version = xhci_read16(xhci, XHCI_REG_CAP_HCIVERSION);
	xhci->hcsparams1 = xhci_read32(xhci, XHCI_REG_CAP_HCSPARAMS1);
	xhci->hcsparams2 = xhci_read32(xhci, XHCI_REG_CAP_HCSPARAMS2);
	xhci->hccparams1 = xhci_read32(xhci, XHCI_REG_CAP_HCCPARAMS1);
	xhci->dboff = xhci_read32(xhci, XHCI_REG_CAP_DBOFF) & XHCI_REG_CAP_DBOFF__MASK;
	xhci->rtsoff = xhci_read32(xhci, XHCI_REG_CAP_RTSOFF) & XHCI_REG_CAP_RTSOFF__MASK;

	if ((xhci->caplength < 0x20u) || (xhci->caplength > 0xffu)) {
		fprintf(stderr, "xhci: invalid caplength 0x%02x\n", xhci->caplength);
		return -ENODEV;
	}

	if (xhci->version != XHCI_SUPPORTED_VERSION) {
		fprintf(stderr, "xhci: unsupported version 0x%04x\n", xhci->version);
		return -ENODEV;
	}

	return -ENOSYS;
}


static int xhci_waitOpBits(xhci_t *xhci, uintptr_t off, uint32_t mask, uint32_t value, unsigned timeoutMs)
{
	uint32_t reg;

	for (; timeoutMs > 0u; --timeoutMs) {
		reg = xhci_opRead32(xhci, off);
		if ((reg & mask) == value) {
			return EOK;
		}

		usleep(1000);
	}

	return -ETIMEDOUT;
}


static int xhci_reset(xhci_t *xhci)
{
	uint32_t usbcmd;
	int err;

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_CNR, 0u, XHCI_CNR_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: controller not ready before reset\n");
		return err;
	}

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd | XHCI_REG_OP_USBCMD_HCRST);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBCMD, XHCI_REG_OP_USBCMD_HCRST, 0u, XHCI_HCRST_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: reset timeout\n");
		return err;
	}

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_CNR, 0u, XHCI_CNR_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: controller not ready after reset\n");
		return err;
	}

	return EOK;
}


static int xhci_validateRuntime(xhci_t *xhci)
{
	xhci->pagesize = xhci_opRead32(xhci, XHCI_REG_OP_PAGESIZE);
	xhci->nintrs = (xhci->hcsparams1 & XHCI_REG_CAP_HCSPARAMS1_MAX_INTRS__MASK) >> XHCI_REG_CAP_HCSPARAMS1_MAX_INTRS__SHIFT;
	xhci->nslots = xhci->hcsparams1 & XHCI_REG_CAP_HCSPARAMS1_MAX_SLOTS__MASK;
	xhci->nports = (xhci->hcsparams1 & XHCI_REG_CAP_HCSPARAMS1_MAX_PORTS__MASK) >> XHCI_REG_CAP_HCSPARAMS1_MAX_PORTS__SHIFT;
	xhci->ist = xhci->hcsparams2 & XHCI_REG_CAP_HCSPARAMS2_IST__MASK;
	xhci->erstMax = (xhci->hcsparams2 & XHCI_REG_CAP_HCSPARAMS2_ERST_MAX__MASK) >> XHCI_REG_CAP_HCSPARAMS2_ERST_MAX__SHIFT;
	xhci->spr = ((xhci->hcsparams2 & XHCI_REG_CAP_HCSPARAMS2_SPR) != 0u) ? 1u : 0u;
	xhci->nscratchpad = (xhci->hcsparams2 & XHCI_REG_CAP_HCSPARAMS2_MAX_SCRATCHPAD_BUFS__MASK) >> XHCI_REG_CAP_HCSPARAMS2_MAX_SCRATCHPAD_BUFS__SHIFT;
	xhci->ac64 = ((xhci->hccparams1 & XHCI_REG_CAP_HCCPARAMS1_AC64) != 0u) ? 1u : 0u;
	xhci->contextSize = ((xhci->hccparams1 & XHCI_REG_CAP_HCCPARAMS1_CSZ) != 0u) ? 64u : 32u;
	xhci->maxPsaSize = (xhci->hccparams1 & XHCI_REG_CAP_HCCPARAMS1_MAX_PSA_SIZE__MASK) >> XHCI_REG_CAP_HCCPARAMS1_MAX_PSA_SIZE__SHIFT;
	xhci->crcrLo = xhci_opRead32(xhci, XHCI_REG_OP_CRCR);
	xhci->crcrHi = xhci_opRead32(xhci, XHCI_REG_OP_CRCR_HI);
	xhci->dcbaapLo = xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP);
	xhci->dcbaapHi = xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP_HI);
	xhci->crcr = ((uint64_t)xhci->crcrHi << 32) | (xhci->crcrLo & XHCI_REG_OP_CRCR_CR_PTR_LO__MASK);
	xhci->dcbaap = ((uint64_t)xhci->dcbaapHi << 32) | (xhci->dcbaapLo & XHCI_REG_OP_DCBAAP__MASK);

	if ((xhci->pagesize & XHCI_REG_OP_PAGESIZE_4K) == 0u) {
		fprintf(stderr, "xhci: 4k page size unsupported\n");
		return -ENODEV;
	}

	if (xhci->nslots == 0u) {
		fprintf(stderr, "xhci: no slots reported\n");
		return -ENODEV;
	}

	if (xhci->nintrs == 0u) {
		fprintf(stderr, "xhci: no interrupters reported\n");
		return -ENODEV;
	}

	if (xhci->nports == 0u) {
		fprintf(stderr, "xhci: no ports reported\n");
		return -ENODEV;
	}

	if (xhci->contextSize != 32u) {
		fprintf(stderr, "xhci: unsupported context size %u\n", xhci->contextSize);
		return -ENODEV;
	}

	if ((xhci->dboff == 0u) || (xhci->dboff >= xhci->mapSz)) {
		fprintf(stderr, "xhci: invalid doorbell offset 0x%08x\n", xhci->dboff);
		return -ENODEV;
	}

	if ((xhci->rtsoff == 0u) || (xhci->rtsoff >= xhci->mapSz)) {
		fprintf(stderr, "xhci: invalid runtime offset 0x%08x\n", xhci->rtsoff);
		return -ENODEV;
	}

	if ((!xhci->ac64) && ((xhci->crcrHi != 0u) || (xhci->dcbaapHi != 0u))) {
		fprintf(stderr, "xhci: unexpected high register state without ac64 support\n");
		return -ENODEV;
	}

	if (((xhci->crcrLo & ~(XHCI_REG_OP_CRCR_CR_PTR_LO__MASK | XHCI_REG_OP_CRCR_RCS | XHCI_REG_OP_CRCR_CS |
			XHCI_REG_OP_CRCR_CA | XHCI_REG_OP_CRCR_CRR)) != 0u) ||
		((xhci->dcbaapLo & ~XHCI_REG_OP_DCBAAP__MASK) != 0u)) {
		fprintf(stderr, "xhci: invalid operational layout register state\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_allocCommandSpace(xhci_t *xhci)
{
	xhci->dcbaaSize = XHCI_DCBAA_SIZE;
	xhci->cmdRingSize = XHCI_CMD_RING_SIZE;

	xhci->dcbaa = usb_allocAligned(xhci->dcbaaSize, XHCI_DCBAA_ALIGN);
	if (xhci->dcbaa == NULL) {
		fprintf(stderr, "xhci: failed to allocate dcbaa\n");
		return -ENOMEM;
	}

	xhci->cmdRing = usb_allocAligned(xhci->cmdRingSize, XHCI_CMD_RING_ALIGN);
	if (xhci->cmdRing == NULL) {
		fprintf(stderr, "xhci: failed to allocate command ring\n");
		return -ENOMEM;
	}

	memset(xhci->dcbaa, 0, xhci->dcbaaSize);
	memset(xhci->cmdRing, 0, xhci->cmdRingSize);

	xhci->dcbaaPhys = va2pa(xhci->dcbaa);
	xhci->cmdRingPhys = va2pa(xhci->cmdRing);

	if (((xhci->dcbaaPhys & (XHCI_DCBAA_ALIGN - 1u)) != 0u) ||
		((xhci->cmdRingPhys & (XHCI_CMD_RING_ALIGN - 1u)) != 0u)) {
		fprintf(stderr, "xhci: invalid command space alignment\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_initCommandRing(xhci_t *xhci)
{
	xhci_trb_t *link;

	xhci->cmdRingCount = xhci->cmdRingSize / XHCI_TRB_SIZE;
	if (xhci->cmdRingCount <= 1u) {
		fprintf(stderr, "xhci: command ring too small\n");
		return -ENODEV;
	}

	xhci->cmdRingTrbs = (xhci_trb_t *)xhci->cmdRing;
	xhci->cmdCycleState = 1u;

	link = &xhci->cmdRingTrbs[xhci->cmdRingCount - 1u];
	link->parameter = xhci->cmdRingPhys;
	link->status = 0u;
	link->control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	if ((link->parameter != xhci->cmdRingPhys) ||
		(((link->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu) != XHCI_TRB_TYPE_LINK) ||
		((link->control & (XHCI_TRB_CONTROL_C | XHCI_LINK_TRB_CONTROL_TC)) !=
			(XHCI_TRB_CONTROL_C | XHCI_LINK_TRB_CONTROL_TC))) {
		fprintf(stderr, "xhci: invalid command ring link trb\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_programCommandSpace(xhci_t *xhci)
{
	uint32_t config;
	uint32_t crcrLo;
	uint32_t crcrHi;
	uint32_t dcbaapLo;
	uint32_t dcbaapHi;

	dcbaapLo = (uint32_t)(xhci->dcbaaPhys & XHCI_REG_OP_DCBAAP__MASK);
	dcbaapHi = (uint32_t)(xhci->dcbaaPhys >> 32);
	if ((!xhci->ac64) && (dcbaapHi != 0u)) {
		fprintf(stderr, "xhci: dcbaa above 32-bit address space\n");
		return -ENODEV;
	}

	crcrLo = (uint32_t)(xhci->cmdRingPhys & XHCI_REG_OP_CRCR_CR_PTR_LO__MASK) | XHCI_REG_OP_CRCR_RCS;
	crcrHi = (uint32_t)(xhci->cmdRingPhys >> 32);
	if ((!xhci->ac64) && (crcrHi != 0u)) {
		fprintf(stderr, "xhci: command ring above 32-bit address space\n");
		return -ENODEV;
	}

	config = xhci_opRead32(xhci, XHCI_REG_OP_CONFIG);
	config &= ~XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK;
	config |= xhci->nslots & XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK;

	xhci_opWrite32(xhci, XHCI_REG_OP_DCBAAP_HI, dcbaapHi);
	xhci_opWrite32(xhci, XHCI_REG_OP_DCBAAP, dcbaapLo);
	xhci_opWrite32(xhci, XHCI_REG_OP_CRCR_HI, crcrHi);
	xhci_opWrite32(xhci, XHCI_REG_OP_CRCR, crcrLo);
	xhci_opWrite32(xhci, XHCI_REG_OP_CONFIG, config);

	xhci->dcbaapLo = xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP);
	xhci->dcbaapHi = xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP_HI);
	xhci->crcrLo = xhci_opRead32(xhci, XHCI_REG_OP_CRCR);
	xhci->crcrHi = xhci_opRead32(xhci, XHCI_REG_OP_CRCR_HI);
	xhci->config = xhci_opRead32(xhci, XHCI_REG_OP_CONFIG);
	xhci->dcbaap = ((uint64_t)xhci->dcbaapHi << 32) | (xhci->dcbaapLo & XHCI_REG_OP_DCBAAP__MASK);
	xhci->crcr = ((uint64_t)xhci->crcrHi << 32) | (xhci->crcrLo & XHCI_REG_OP_CRCR_CR_PTR_LO__MASK);

	if ((xhci->dcbaap != xhci->dcbaaPhys) || (xhci->crcr != xhci->cmdRingPhys) ||
		((xhci->crcrLo & XHCI_REG_OP_CRCR_RCS) == 0u) ||
		((xhci->config & XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK) != (xhci->nslots & XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK))) {
		fprintf(stderr, "xhci: command space register program mismatch\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_runStateSelftest(xhci_t *xhci)
{
	uint32_t usbcmd;
	uint32_t usbsts;
	int err;

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state before run\n");
		return -ENODEV;
	}

	if ((usbsts & XHCI_REG_OP_USBSTS_HCH) == 0u) {
		fprintf(stderr, "xhci: controller not halted before run\n");
		return -ENODEV;
	}

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd | XHCI_REG_OP_USBCMD_RS);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, 0u, XHCI_RUNSTOP_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: run transition timeout\n");
		return err;
	}

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state after run\n");
		return -ENODEV;
	}

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd & ~XHCI_REG_OP_USBCMD_RS);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, XHCI_REG_OP_USBSTS_HCH, XHCI_RUNSTOP_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: halt transition timeout\n");
		return err;
	}

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state after halt\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_enterRunState(xhci_t *xhci)
{
	uint32_t usbcmd;
	uint32_t usbsts;
	int err;

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state before run\n");
		return -ENODEV;
	}

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd | XHCI_REG_OP_USBCMD_RS);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, 0u, XHCI_RUNSTOP_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: run transition timeout\n");
		return err;
	}

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state after run\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_enterHaltedState(xhci_t *xhci)
{
	uint32_t usbcmd;
	uint32_t usbsts;
	int err;

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd & ~XHCI_REG_OP_USBCMD_RS);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, XHCI_REG_OP_USBSTS_HCH, XHCI_RUNSTOP_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: halt transition timeout\n");
		return err;
	}

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state after halt\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_allocEventRing(xhci_t *xhci)
{
	xhci_erst_entry_t *erst;

	xhci->eventRingSize = XHCI_EVENT_RING_SIZE;
	xhci->erstSize = XHCI_ERST_SIZE;

	xhci->eventRing = usb_allocAligned(xhci->eventRingSize, XHCI_EVENT_RING_ALIGN);
	if (xhci->eventRing == NULL) {
		fprintf(stderr, "xhci: failed to allocate event ring\n");
		return -ENOMEM;
	}

	xhci->erst = usb_allocAligned(xhci->erstSize, XHCI_ERST_ALIGN);
	if (xhci->erst == NULL) {
		fprintf(stderr, "xhci: failed to allocate erst\n");
		return -ENOMEM;
	}

	memset(xhci->eventRing, 0, xhci->eventRingSize);
	memset(xhci->erst, 0, xhci->erstSize);

	xhci->eventRingPhys = va2pa(xhci->eventRing);
	xhci->erstPhys = va2pa(xhci->erst);
	xhci->eventRingTrbs = xhci->eventRingSize / XHCI_TRB_SIZE;
	xhci->eventCycleState = 1u;

	if (((xhci->eventRingPhys & (XHCI_EVENT_RING_ALIGN - 1u)) != 0u) ||
		((xhci->erstPhys & (XHCI_ERST_ALIGN - 1u)) != 0u) ||
		(xhci->eventRingTrbs == 0u)) {
		fprintf(stderr, "xhci: invalid event ring allocation layout\n");
		return -ENODEV;
	}

	erst = (xhci_erst_entry_t *)xhci->erst;
	erst[0].ringSegmentBase = xhci->eventRingPhys;
	erst[0].ringSegmentSize = xhci->eventRingTrbs;

	if ((erst[0].ringSegmentBase != xhci->eventRingPhys) ||
		(erst[0].ringSegmentSize != xhci->eventRingTrbs)) {
		fprintf(stderr, "xhci: failed to populate erst entry\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_cmdNoopSelftest(xhci_t *xhci)
{
	return xhci_cmdExec(xhci, 0u, 0u, XHCI_TRB_TYPE_CMD_NO_OP << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT, NULL);
}


static int xhci_cmdExec(xhci_t *xhci, uint64_t parameter, uint32_t status, uint32_t control, uint8_t *slotId)
{
	xhci_trb_t *cmd;
	xhci_trb_t *event;
	uint64_t cmdPhys;
	uint32_t type;
	uint32_t completion;
	unsigned timeoutMs;
	int err;

	cmd = &xhci->cmdRingTrbs[0];
	memset(cmd, 0, sizeof(*cmd));
	cmd->parameter = parameter;
	cmd->status = status;
	cmd->control = (control & ~XHCI_TRB_CONTROL_C) | (xhci->cmdCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u);
	cmdPhys = xhci->cmdRingPhys;

	event = (xhci_trb_t *)xhci->eventRing;
	memset(event, 0, sizeof(*event));

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		memset(cmd, 0, sizeof(*cmd));
		return err;
	}

	xhci_dbWrite32(xhci, 0u, 0u);

	for (timeoutMs = XHCI_CMD_TIMEOUT_MS; timeoutMs > 0u; --timeoutMs) {
		if ((event->control & XHCI_TRB_CONTROL_C) == (xhci->eventCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u)) {
			break;
		}

		usleep(1000);
	}

	if (timeoutMs == 0u) {
		fprintf(stderr, "xhci: command completion timeout\n");
		(void)xhci_enterHaltedState(xhci);
		memset(cmd, 0, sizeof(*cmd));
		return -ETIMEDOUT;
	}

	type = (event->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;
	completion = (event->status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
	if ((type != XHCI_TRB_TYPE_EVENT_CMD_COMPLETION) || (event->parameter != cmdPhys)) {
		fprintf(stderr, "xhci: invalid command completion event\n");
		(void)xhci_enterHaltedState(xhci);
		memset(cmd, 0, sizeof(*cmd));
		return -ENODEV;
	}

	if (slotId != NULL) {
		*slotId = (event->control & XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__MASK) >>
			XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT;
	}

	err = xhci_enterHaltedState(xhci);
	if (err < 0) {
		memset(cmd, 0, sizeof(*cmd));
		return err;
	}

	memset(cmd, 0, sizeof(*cmd));

	if (completion != XHCI_TRB_COMPLETION_CODE_SUCCESS) {
		fprintf(stderr, "xhci: command completion code %u\n", completion);
		return -ENODEV;
	}

	return EOK;
}


static int xhci_cmdEnableSlot(xhci_t *xhci, uint8_t *slotId)
{
	uint8_t cmdSlotId = 0u;
	int err;

	err = xhci_cmdExec(xhci, 0u, 0u, XHCI_TRB_TYPE_CMD_ENABLE_SLOT << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT, &cmdSlotId);
	if (err < 0) {
		return err;
	}

	if ((cmdSlotId == 0u) || (cmdSlotId > xhci->nslots)) {
		fprintf(stderr, "xhci: invalid enable-slot id %u\n", cmdSlotId);
		return -ENODEV;
	}

	if (slotId != NULL) {
		*slotId = cmdSlotId;
	}

	return EOK;
}


static int xhci_programEventRing(xhci_t *xhci)
{
	uint32_t erstbaLo;
	uint32_t erstbaHi;
	uint32_t erdpLo;
	uint32_t erdpHi;

	erstbaLo = (uint32_t)(xhci->erstPhys & XHCI_REG_RT_IR_ERSTBA_LO__MASK);
	erstbaHi = (uint32_t)(xhci->erstPhys >> 32);
	erdpLo = (uint32_t)(xhci->eventRingPhys & XHCI_REG_RT_IR_ERDP_LO__MASK);
	erdpHi = (uint32_t)(xhci->eventRingPhys >> 32);

	if ((!xhci->ac64) && ((erstbaHi != 0u) || (erdpHi != 0u))) {
		fprintf(stderr, "xhci: event ring state above 32-bit address space\n");
		return -ENODEV;
	}

	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTSZ, XHCI_ERST_ENTRY_COUNT & XHCI_REG_RT_IR_ERSTSZ__MASK);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTBA_HI, erstbaHi);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTBA_LO, erstbaLo);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, erdpHi);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO, erdpLo);

	xhci->erstsz = xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERSTSZ);
	xhci->erstbaLo = xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERSTBA_LO);
	xhci->erstbaHi = xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERSTBA_HI);
	xhci->erdpLo = xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERDP_LO);
	xhci->erdpHi = xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERDP_HI);
	xhci->erstba = ((uint64_t)xhci->erstbaHi << 32) | (xhci->erstbaLo & XHCI_REG_RT_IR_ERSTBA_LO__MASK);
	xhci->erdp = ((uint64_t)xhci->erdpHi << 32) | (xhci->erdpLo & XHCI_REG_RT_IR_ERDP_LO__MASK);

	if (((xhci->erstsz & XHCI_REG_RT_IR_ERSTSZ__MASK) != XHCI_ERST_ENTRY_COUNT) ||
		(xhci->erstba != xhci->erstPhys) ||
		(xhci->erdp != xhci->eventRingPhys) ||
		((xhci->erdpLo & XHCI_REG_RT_IR_ERDP_LO_EHB) != 0u)) {
		fprintf(stderr, "xhci: event ring register program mismatch\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_init(hcd_t *hcd)
{
	xhci_t *xhci;
	int err;

	err = xhci_map(hcd, &xhci);
	if (err < 0) {
		return err;
	}

	err = xhci_capProbe(hcd, xhci);
	if (err == -ENOSYS) {
		err = xhci_reset(xhci);
		if (err == 0) {
			err = xhci_validateRuntime(xhci);
			if (err == 0) {
				err = xhci_allocCommandSpace(xhci);
				if (err == 0) {
					err = xhci_initCommandRing(xhci);
					if (err == 0) {
						err = xhci_programCommandSpace(xhci);
						if (err == 0) {
							err = xhci_runStateSelftest(xhci);
							if (err == 0) {
								err = xhci_allocEventRing(xhci);
							if (err == 0) {
								err = xhci_programEventRing(xhci);
								if (err == 0) {
									err = xhci_cmdNoopSelftest(xhci);
									if (err == 0) {
										err = xhci_cmdEnableSlot(xhci, &xhci->slotId);
									}
								}
							}
						}
					}
					}
				}
			}
		}
	}

	if (err != 0) {
		xhci_destroy(xhci);
		hcd->priv = NULL;
		hcd->base = NULL;
	}
	else if (beginthread(xhci_roothubStatusThread, 4, xhci->statusStack, sizeof(xhci->statusStack), hcd) != 0) {
		xhci_destroy(xhci);
		hcd->priv = NULL;
		hcd->base = NULL;
		err = -ENOMEM;
	}

	return err;
}


static int xhci_getStringDesc(int index, char *buf, size_t size)
{
	usb_string_desc_t *desc;
	size_t len = 0u;
	const char *src;
	int i;

	switch (index) {
		case 0:
			len = 4u;
			src = xhci_descs.langID;
			break;
		case 1:
			len = strlen(xhci_descs.product) * 2u + 3u;
			src = xhci_descs.product;
			break;
		case 2:
			len = strlen(xhci_descs.manufacturer) * 2u + 3u;
			src = xhci_descs.manufacturer;
			break;
		default:
			return 0;
	}

	if (size < len) {
		return -1;
	}

	desc = (usb_string_desc_t *)buf;
	desc->bDescriptorType = USB_DESC_STRING;
	desc->bLength = len;

	if (index == 0) {
		memcpy(buf, src, len - 2u);
	}
	else {
		memset(desc->wData, 0, len - 2u);

		for (i = 0; src[i] != '\0'; ++i) {
			desc->wData[i * 2] = src[i];
		}
	}

	return len;
}


static int xhci_getDesc(usb_dev_t *hub, int type, int index, char *buf, size_t size)
{
	hcd_t *hcd = hub->hcd;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	usb_hub_desc_t *hdesc;
	int bytes = 0;

	switch (type) {
		case USB_DESC_DEVICE:
			bytes = min(size, xhci_descs.dev.bLength);
			memcpy(buf, &xhci_descs.dev, bytes);
			break;

		case USB_DESC_CONFIG:
			bytes = min(size, xhci_descs.cfg.wTotalLength);
			memcpy(buf, &xhci_descs.cfg, bytes);
			break;

		case USB_DESC_STRING:
			bytes = xhci_getStringDesc(index, buf, size);
			break;

		case USB_DESC_TYPE_HUB:
			if (size < sizeof(usb_hub_desc_t) + 2u) {
				break;
			}

			hdesc = (usb_hub_desc_t *)buf;
			hdesc->bDescLength = 9;
			hdesc->bDescriptorType = USB_DESC_TYPE_HUB;
			hdesc->wHubCharacteristics = 0x1;
			hdesc->bPwrOn2PwrGood = 10;
			hdesc->bHubContrCurrent = 0;
			hdesc->bNbrPorts = min((unsigned int)USB_HUB_MAX_PORTS, xhci->nports);
			hdesc->variable[0] = 0;
			hdesc->variable[1] = 0xff;
			bytes = hdesc->bDescLength;
			break;
	}

	return bytes;
}


static int xhci_getPortStatus(usb_dev_t *hub, int port, usb_port_status_t *status)
{
	hcd_t *hcd = hub->hcd;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	uint32_t portsc;
	uint32_t speed;

	if ((port <= 0) || (port > hub->nports)) {
		return -1;
	}

	memset(status, 0, sizeof(*status));

	portsc = xhci_portRead32(xhci, port, XHCI_REG_OP_PORT_PORTSC);

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_CCS) != 0u) {
		status->wPortStatus |= USB_PORT_STAT_CONNECTION;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_PED) != 0u) {
		status->wPortStatus |= USB_PORT_STAT_ENABLE;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_OCA) != 0u) {
		status->wPortStatus |= USB_PORT_STAT_OVERCURRENT;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_PR) != 0u) {
		status->wPortStatus |= USB_PORT_STAT_RESET;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_PP) != 0u) {
		status->wPortStatus |= USB_PORT_STAT_POWER;
	}

	speed = (portsc & XHCI_REG_OP_PORT_PORTSC_PORT_SPEED__MASK) >> XHCI_REG_OP_PORT_PORTSC_PORT_SPEED__SHIFT;
	if (speed == 2u) {
		status->wPortStatus |= USB_PORT_STAT_LOW_SPEED;
	}
	else if (speed == 3u) {
		status->wPortStatus |= USB_PORT_STAT_HIGH_SPEED;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_CSC) != 0u) {
		status->wPortChange |= USB_PORT_STAT_C_CONNECTION;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_PEC) != 0u) {
		status->wPortChange |= USB_PORT_STAT_C_ENABLE;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_OCC) != 0u) {
		status->wPortChange |= USB_PORT_STAT_C_OVERCURRENT;
	}

	if ((portsc & XHCI_REG_OP_PORT_PORTSC_PRC) != 0u) {
		status->wPortChange |= USB_PORT_STAT_C_RESET;
	}

	return 0;
}


static int xhci_setPortFeature(usb_dev_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	uint32_t portsc;
	int err;

	if ((port <= 0) || (port > hub->nports)) {
		return -1;
	}

	portsc = xhci_portRead32(xhci, port, XHCI_REG_OP_PORT_PORTSC);

	switch (wValue) {
		case USB_PORT_FEAT_RESET:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, (portsc | XHCI_REG_OP_PORT_PORTSC_PR) & ~XHCI_REG_OP_PORT_PORTSC_PED);
			err = xhci_portWaitBits(xhci, port, XHCI_REG_OP_PORT_PORTSC_PR, 0u, XHCI_PORT_RESET_TIMEOUT_MS);
			if (err < 0) {
				return err;
			}
			err = xhci_portWaitBits(xhci, port, XHCI_REG_OP_PORT_PORTSC_PLS__MASK,
				XHCI_REG_OP_PORT_PORTSC_PLS_U0 << XHCI_REG_OP_PORT_PORTSC_PLS__SHIFT, XHCI_PORT_RESET_TIMEOUT_MS);
			if (err < 0) {
				return err;
			}
			break;

		case USB_PORT_FEAT_POWER:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, portsc | XHCI_REG_OP_PORT_PORTSC_PP);
			usleep(XHCI_PORT_POWER_GOOD_DELAY_US);
			break;

		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_TEST:
		case USB_PORT_FEAT_INDICATOR:
			break;

		default:
			return -1;
	}

	return 0;
}


static int xhci_clearPortFeature(usb_dev_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	uint32_t portsc;

	if ((port <= 0) || (port > hub->nports)) {
		return -1;
	}

	portsc = xhci_portRead32(xhci, port, XHCI_REG_OP_PORT_PORTSC);

	switch (wValue) {
		case USB_PORT_FEAT_C_CONNECTION:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, (portsc & ~XHCI_REG_OP_PORT_PORTSC_PED) | XHCI_REG_OP_PORT_PORTSC_CSC);
			break;

		case USB_PORT_FEAT_C_ENABLE:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, (portsc & ~XHCI_REG_OP_PORT_PORTSC_PED) | XHCI_REG_OP_PORT_PORTSC_PEC);
			break;

		case USB_PORT_FEAT_C_OVER_CURRENT:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, (portsc & ~XHCI_REG_OP_PORT_PORTSC_PED) | XHCI_REG_OP_PORT_PORTSC_OCC);
			break;

		case USB_PORT_FEAT_C_RESET:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, (portsc & ~XHCI_REG_OP_PORT_PORTSC_PED) | XHCI_REG_OP_PORT_PORTSC_PRC);
			break;

		case USB_PORT_FEAT_ENABLE:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, portsc & ~(XHCI_REG_OP_PORT_PORTSC_PED | XHCI_REG_OP_PORT_PORTSC_RW1C));
			break;

		case USB_PORT_FEAT_POWER:
			xhci_portWrite32(xhci, port, XHCI_REG_OP_PORT_PORTSC, portsc & ~(XHCI_REG_OP_PORT_PORTSC_PP | XHCI_REG_OP_PORT_PORTSC_RW1C));
			break;

		case USB_PORT_FEAT_INDICATOR:
		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_C_SUSPEND:
			break;

		default:
			return -1;
	}

	return 0;
}


static int xhci_roothubReq(usb_dev_t *hub, usb_transfer_t *t)
{
	usb_setup_packet_t *setup = t->setup;
	int ret;

	if (t->type == usb_transfer_interrupt) {
		return 0;
	}

	switch (setup->bRequest) {
		case REQ_GET_STATUS:
			ret = xhci_getPortStatus(hub, setup->wIndex, (usb_port_status_t *)t->buffer);
			break;

		case REQ_SET_FEATURE:
			ret = xhci_setPortFeature(hub, setup->wIndex, setup->wValue);
			break;

		case REQ_CLEAR_FEATURE:
			ret = xhci_clearPortFeature(hub, setup->wIndex, setup->wValue);
			break;

		case REQ_GET_DESCRIPTOR:
			ret = xhci_getDesc(hub, setup->wValue >> 8, setup->wValue & 0xff, t->buffer, t->size);
			break;

		case REQ_SET_ADDRESS:
		case REQ_SET_CONFIGURATION:
			ret = 0;
			break;

		default:
			ret = -1;
	}

	usb_transferFinished(t, ret);

	return 0;
}


static int xhci_transferEnqueue(hcd_t *hcd, usb_transfer_t *t, usb_pipe_t *pipe)
{
	if (usb_isRoothub(pipe->dev) != 0) {
		return xhci_roothubReq(pipe->dev, t);
	}

	return -ENOSYS;
}


static void xhci_transferDequeue(hcd_t *hcd, usb_transfer_t *t)
{
	(void)hcd;
	(void)t;
}


static void xhci_pipeDestroy(hcd_t *hcd, usb_pipe_t *pipe)
{
	(void)hcd;
	(void)pipe;
}


static uint32_t xhci_getHubStatus(usb_dev_t *hub)
{
	hcd_t *hcd = hub->hcd;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	uint32_t status = 0u;
	uint32_t portsc;
	int i;

	for (i = 0; i < hub->nports; ++i) {
		portsc = xhci_portRead32(xhci, i + 1, XHCI_REG_OP_PORT_PORTSC);
		if ((portsc & XHCI_REG_OP_PORT_PORTSC_RW1C) != 0u) {
			status |= 1u << (i + 1);
		}
	}

	return status;
}


static const hcd_ops_t xhci_ops = {
	.type = "xhci",
	.init = xhci_init,
	.transferEnqueue = xhci_transferEnqueue,
	.transferDequeue = xhci_transferDequeue,
	.pipeDestroy = xhci_pipeDestroy,
	.getRoothubStatus = xhci_getHubStatus
};


__attribute__((constructor)) static void xhci_register(void)
{
	hcd_register(&xhci_ops);
}
