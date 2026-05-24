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

#if defined(__TARGET_AARCH64A72) && defined(PCI_EXPRESS_BCM2711_INDEXED_CFG)
#include "bcm2711-pcie.h"
#else
/* Other boards' xhci PHYs don't need BCM2711 bring-up; provide stubs. */
static inline int bcm2711_pcie_initVL805(void) { return 0; }
static inline volatile void *bcm2711_pcie_getXhciMmio(void) { return NULL; }
static inline uint64_t bcm2711_pcie_getXhciMmioSize(void) { return 0; }
static inline int bcm2711_pcie_resettleOutboundWindow(void) { return 0; }
#endif


/* VL805 BAR0 is 4 KiB on the Pi 4 (verified via cross-OS reference:
 * FreeBSD bcm2838_xhci, Circle USBStandardHub, Raspberry Pi linux-rpi
 * "xhci_pci_setup" probe). Mapping 64 KiB (the old XHCI_MAP_SIZE)
 * spills past the BAR into unmapped PCIe outbound window territory;
 * the BCM2711 root complex returns 0xdeaddead poison for those reads
 * and the BCM2711 SError absorbs the abort, surfacing as our long-
 * standing usb-hcd: ops->init fail rc=-19. Whole-region typical xHCI
 * usage (cap + op + runtime + doorbells) fits comfortably in 4 KiB
 * for a 4-port / 16-slot controller. See
 * docs/notes/2026-05-22-usb-research-synthesis.md. */
#define XHCI_MAP_SIZE            0x1000u
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
#define XHCI_REG_RT_IR_IMOD      0x04u
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
#define XHCI_REG_RT_IR_IMAN_IP    (1u << 0)
#define XHCI_REG_RT_IR_IMAN_IE    (1u << 1)
#define XHCI_REG_RT_IR_ERDP_LO__MASK 0xfffffff0u
#define XHCI_DCBAA_ALIGN         0x1000u
#define XHCI_DCBAA_SIZE          0x1000u
#define XHCI_SCRATCHPAD_PAGE_SIZE   0x1000u
#define XHCI_SCRATCHPAD_PAGE_ALIGN  0x1000u
#define XHCI_SCRATCHPAD_ARRAY_ALIGN 0x1000u
#define XHCI_SCRATCHPAD_ENTRY_SIZE  sizeof(uint64_t)
/*
 * Command Ring sizing rationale:
 *
 * xHCI 1.0 §6.1: "The Command Ring shall be aligned to a 64 byte boundary
 * and shall not cross a 64K Byte boundary." Phoenix's USB allocator
 * (usb_allocAligned) only guarantees VA alignment up to a page (4 KB) and
 * the corresponding va2pa() does not preserve larger alignments because
 * pages are backed independently. Choosing a 4 KB ring satisfies both
 * "64-byte aligned" and "no cross-64K" automatically (the entire ring fits
 * within one page), without requiring deeper allocator changes. 4 KB
 * holds 256 TRBs which is plenty for the command queue depth Phoenix
 * issues during enumeration and steady-state.
 */
#define XHCI_CMD_RING_ALIGN      0x40u
#define XHCI_CMD_RING_SIZE       0x1000u
#define XHCI_EVENT_RING_ALIGN    0x1000u
#define XHCI_EVENT_RING_SIZE     0x1000u
#define XHCI_ERST_ALIGN          0x40u
#define XHCI_ERST_SIZE           0x40u
#define XHCI_ERST_ENTRY_COUNT    1u
#define XHCI_CONTEXT_ALIGN       0x40u
#define XHCI_TRANSFER_RING_ALIGN 0x40u
#define XHCI_TRANSFER_RING_SIZE  0x1000u
#define XHCI_MAX_ENDPOINTS       31u
#define XHCI_CONTEXT_INPUT       1u
#define XHCI_DCBAA_ENTRY_SIZE    sizeof(uint64_t)
#define XHCI_TRB_SIZE            16u
#define XHCI_CNR_TIMEOUT_MS      100u
/* xHCI 5.4.1 says HCRST may take an indeterminate amount of time.
 * Linux xhci uses XHCI_RESET_SHORT_USEC (250 ms) for the normal
 * reset path; FreeBSD xhci_halt() polls up to 500 ms. Phoenix's
 * earlier 20 ms was too tight — empirical Pi 4 / VL805 reset
 * times out at 20 ms (rc=-110 seen in 2026-05-22 boots). Match
 * Linux's short reset budget. */
#define XHCI_HCRST_TIMEOUT_MS    250u
#define XHCI_RUNSTOP_TIMEOUT_MS  250u
#define XHCI_TRB_CONTROL_C       (1u << 0)
#define XHCI_TRB_CONTROL_TRB_TYPE__SHIFT 10u
#define XHCI_TRANSFER_TRB_CONTROL_ISP (1u << 2)
#define XHCI_TRANSFER_TRB_CONTROL_IOC (1u << 5)
#define XHCI_TRANSFER_TRB_CONTROL_IDT (1u << 6)
#define XHCI_TRANSFER_TRB_CONTROL_DIR_IN (1u << 16)
#define XHCI_TRANSFER_TRB_CONTROL_TRT__SHIFT 16u
#define XHCI_TRANSFER_TRB_CONTROL_TRT_NONE 0u
#define XHCI_TRANSFER_TRB_CONTROL_TRT_IN 3u
#define XHCI_TRB_TYPE_LINK       6u
#define XHCI_TRB_TYPE_NORMAL     1u
#define XHCI_TRB_TYPE_SETUP_STAGE 2u
#define XHCI_TRB_TYPE_DATA_STAGE 3u
#define XHCI_TRB_TYPE_STATUS_STAGE 4u
#define XHCI_TRB_TYPE_CMD_ENABLE_SLOT 9u
#define XHCI_TRB_TYPE_CMD_ADDRESS_DEVICE 11u
#define XHCI_TRB_TYPE_CMD_CONFIGURE_ENDPOINT 12u
#define XHCI_TRB_TYPE_CMD_NO_OP  23u
#define XHCI_TRB_TYPE_EVENT_TRANSFER 32u
#define XHCI_TRB_TYPE_EVENT_CMD_COMPLETION 33u
#define XHCI_LINK_TRB_CONTROL_TC (1u << 1)
#define XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT 24u
#define XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK (0xffu << XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT)
#define XHCI_TRANSFER_EVENT_TRB_STATUS_TRB_TRANSFER_LENGTH__MASK 0xffffffu
#define XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT 16u
#define XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK (0x1fu << XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT)
#define XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT 24u
#define XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK (0xffu << XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT)
#define XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT 24u
#define XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__MASK (0xffu << XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT)
#define XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_BSR (1u << 9)
#define XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_SLOTID__SHIFT 24u
#define XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_DC (1u << 9)
#define XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_SLOTID__SHIFT 24u
#define XHCI_TRB_COMPLETION_CODE_SUCCESS 1u
#define XHCI_TRB_COMPLETION_CODE_SHORT_PACKET 13u
#define XHCI_PORT_SPEED_FULL     1u
#define XHCI_PORT_SPEED_LOW      2u
#define XHCI_PORT_SPEED_HIGH     3u
#define XHCI_SLOT_CTX_SPEED__SHIFT 20u
#define XHCI_SLOT_CTX_CONTEXT_ENTRIES__SHIFT 27u
#define XHCI_SLOT_CTX_ROOT_HUB_PORT__SHIFT 16u
#define XHCI_EP_CTX_CERR__SHIFT  1u
#define XHCI_EP_CTX_TYPE__SHIFT  3u
#define XHCI_EP_CTX_TYPE_CONTROL 4u
#define XHCI_EP_CTX_TYPE_INTERRUPT_IN 7u
#define XHCI_EP_CTX_INTERVAL__SHIFT 16u
#define XHCI_EP_CTX_MAX_BURST__SHIFT 8u
#define XHCI_EP_CTX_MAX_PACKET__SHIFT 16u
#define XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS (1u << 0)
#define XHCI_EP_CTX_MAX_ESIT_PAYLOAD__SHIFT 16u
#define XHCI_INPUT_CTRL_CTX_ADD_A0_A1 0x3u
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
	uint32_t routeString_speed_mtt_hub_entries;
	uint32_t maxExitLatency_rootHubPort_ports;
	uint32_t ttHubSlot_ttPort_ttt_intrTarget;
	uint32_t usbAddr_slotState;
	uint32_t reserved[4];
} __attribute__((packed)) xhci_slot_ctx_t;


typedef struct {
	uint32_t epState_mult_streams_interval;
	uint32_t cerr_type_burst_packet;
	uint64_t trDequeuePtr;
	uint32_t averageTrbLen_maxEsitPayload;
	uint32_t reserved[3];
} __attribute__((packed)) xhci_ep_ctx_t;


typedef struct {
	xhci_slot_ctx_t slot;
	xhci_ep_ctx_t ep[XHCI_MAX_ENDPOINTS];
} __attribute__((packed)) xhci_dev_ctx_t;


typedef struct {
	uint32_t dropContextFlags;
	uint32_t addContextFlags;
	uint32_t reserved[6];
} __attribute__((packed)) xhci_input_ctrl_ctx_t;


typedef struct {
	xhci_input_ctrl_ctx_t control;
	xhci_dev_ctx_t device;
} __attribute__((packed)) xhci_input_ctx_t;


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
	void *scratchpadArray;
	void **scratchpadBufs;
	size_t scratchpadArraySize;
	uint64_t scratchpadArrayPhys;
	void *eventRing;
	void *erst;
	void *devCtx;
	void *inputCtx;
	void *ep0Ring;
	size_t eventRingSize;
	size_t erstSize;
	size_t devCtxSize;
	size_t inputCtxSize;
	size_t ep0RingSize;
	uint64_t eventRingPhys;
	uint64_t erstPhys;
	uint64_t devCtxPhys;
	uint64_t inputCtxPhys;
	uint64_t ep0RingPhys;
	uint32_t cmdRingCount;
	uint32_t cmdCycleState;
	uint32_t ep0RingCount;
	uint32_t ep0CycleState;
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
	struct xhci_pipePriv *interruptPriv;
	unsigned ac64 : 1;
	unsigned spr : 1;
} xhci_t;


typedef struct xhci_pipePriv {
	void *ring;
	size_t ringSize;
	uint64_t ringPhys;
	uint32_t ringCount;
	uint32_t cycleState;
	uint8_t endpointId;
	uint8_t endpointType;
	usb_transfer_t *pendingTransfer;
	uint64_t pendingTrbPhys;
} xhci_pipePriv_t;


static uint32_t xhci_getHubStatus(usb_dev_t *hub);
static int xhci_cmdExec(xhci_t *xhci, uint64_t parameter, uint32_t status, uint32_t control, uint8_t *slotId);
static int xhci_enterHaltedState(xhci_t *xhci);


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
	unsigned i;

	/* Tight ~1ms burst at 50us catches the common fast transitions
	 * (port-reset clear, PLS->U0) without paying a full 1ms per check. */
	for (i = 0u; i < 20u; ++i) {
		reg = xhci_portRead32(xhci, port, XHCI_REG_OP_PORT_PORTSC);
		if ((reg & mask) == value) {
			return EOK;
		}
		usleep(50);
	}

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

	if (xhci->scratchpadBufs != NULL) {
		uint32_t i;
		for (i = 0u; i < xhci->nscratchpad; ++i) {
			if (xhci->scratchpadBufs[i] != NULL) {
				usb_freeAligned(xhci->scratchpadBufs[i], XHCI_SCRATCHPAD_PAGE_SIZE);
			}
		}
		free(xhci->scratchpadBufs);
	}

	if (xhci->scratchpadArray != NULL) {
		usb_freeAligned(xhci->scratchpadArray, xhci->scratchpadArraySize);
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

	if (xhci->devCtx != NULL) {
		usb_freeAligned(xhci->devCtx, xhci->devCtxSize);
	}

	if (xhci->inputCtx != NULL) {
		usb_freeAligned(xhci->inputCtx, xhci->inputCtxSize);
	}

	if (xhci->ep0Ring != NULL) {
		usb_freeAligned(xhci->ep0Ring, xhci->ep0RingSize);
	}

	free(xhci);
}


static void xhci_roothubStatusThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	usb_dev_t *hub;
	xhci_pipePriv_t *priv;
	xhci_trb_t *event;
	usb_transfer_t *t;
	uint32_t status;
	uint32_t type;
	uint32_t completion;
	uint32_t endpointId;
	uint32_t slotId;
	uint32_t residual;
	unsigned sleepUs;
	int ret;

	for (;;) {
		sleepUs = 100000u;
		hub = hcd->roothub;
		if ((hub != NULL) && (hub->statusTransfer != NULL) && !usb_transferCheck(hub->statusTransfer)) {
			status = xhci_getHubStatus(hub);
			if (status != 0u) {
				memcpy(hub->statusTransfer->buffer, &status, sizeof(status));
				usb_transferFinished(hub->statusTransfer, hub->statusTransfer->size);
			}
		}

		priv = xhci->interruptPriv;
		if ((priv != NULL) && (priv->pendingTransfer != NULL)) {
			sleepUs = 1000u;
			event = (xhci_trb_t *)xhci->eventRing;
			if ((event->control & XHCI_TRB_CONTROL_C) == (xhci->eventCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u)) {
				t = priv->pendingTransfer;
				type = (event->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;
				completion = (event->status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >>
					XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
				endpointId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >>
					XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
				slotId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >>
					XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;
				residual = event->status & XHCI_TRANSFER_EVENT_TRB_STATUS_TRB_TRANSFER_LENGTH__MASK;

				if ((type == XHCI_TRB_TYPE_EVENT_TRANSFER) && (event->parameter == priv->pendingTrbPhys) &&
					(endpointId == priv->endpointId) && (slotId == xhci->slotId) &&
					(residual <= t->size) &&
					((completion == XHCI_TRB_COMPLETION_CODE_SUCCESS) ||
					(completion == XHCI_TRB_COMPLETION_CODE_SHORT_PACKET))) {
					ret = (int)(t->size - residual);
				}
				else {
					ret = -ENODEV;
				}

				memset(event, 0, sizeof(*event));
				xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, (uint32_t)(xhci->eventRingPhys >> 32));
				xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO,
					(uint32_t)(xhci->eventRingPhys & XHCI_REG_RT_IR_ERDP_LO__MASK) | XHCI_REG_RT_IR_ERDP_LO_EHB);
				(void)xhci_enterHaltedState(xhci);

				priv->pendingTransfer = NULL;
				usb_transferFinished(t, ret);
			}
		}

		usleep(sleepUs);
	}
}


static int xhci_map(hcd_t *hcd, xhci_t **xhcip)
{
	xhci_t *xhci;
	off_t offs;
	volatile void *preMapped;

	xhci = calloc(1, sizeof(*xhci));
	if (xhci == NULL) {
		return -ENOMEM;
	}

	xhci->mmio = MAP_FAILED;
	xhci->mapSz = XHCI_MAP_SIZE;
	offs = hcd->info->hcdaddr % _PAGE_SIZE;

	/* On BCM2711 the bridge invalidates its outbound translation as
	 * soon as a new MAP_DEVICE mapping of the outbound CPU PA happens
	 * outside the pcie scan-probe callback (see TD-USB-pmap notes in
	 * bcm2711-pcie.c). Prefer the pre-created mapping handed to us
	 * from pcie_scanProbe; only fall back to a fresh mmap on non-
	 * BCM2711 boards (which use a generic ECAM path). */
	preMapped = bcm2711_pcie_getXhciMmio();
	if (preMapped != NULL) {
		xhci->mmio = (void *)preMapped;
		xhci->mapSz = bcm2711_pcie_getXhciMmioSize();
	}
	else {
		xhci->mmio = mmap(NULL, xhci->mapSz, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
		if (xhci->mmio == MAP_FAILED) {
			xhci_destroy(xhci);
			return -ENOMEM;
		}
	}

	hcd->base = (volatile uint32_t *)((uintptr_t)xhci->mmio + offs);
	hcd->priv = xhci;
	*xhcip = xhci;

	return EOK;
}


static int xhci_capProbe(hcd_t *hcd, xhci_t *xhci)
{
	unsigned int attempt;
	(void)hcd;

	/* Poll cap-space up to N times with a settling delay between
	 * attempts. The BCM2711 PCIe outbound translation can be in a
	 * transient post-mailbox-notify state where reads return
	 * 0xdeaddead (caplength=0xad, version=0xdead). A short delay +
	 * re-read usually catches the bridge after it settles —
	 * empirically the recovery window is ~50–100 ms. We try 6×100 ms
	 * for a 600 ms total worst-case, well under any meaningful boot
	 * deadline but more than enough to clear the transient.
	 *
	 * Each attempt's caplength + HCIVERSION is logged so post-mortem
	 * we can tell whether MMIO became readable mid-loop, was always
	 * poisoned, or flipped between states. */
	for (attempt = 0u; attempt < 6u; ++attempt) {
		xhci->caplength = xhci_read8(xhci, XHCI_REG_CAP_CAPLENGTH);
		xhci->version = xhci_read16(xhci, XHCI_REG_CAP_HCIVERSION);
		xhci->hcsparams1 = xhci_read32(xhci, XHCI_REG_CAP_HCSPARAMS1);
		xhci->hcsparams2 = xhci_read32(xhci, XHCI_REG_CAP_HCSPARAMS2);
		xhci->hccparams1 = xhci_read32(xhci, XHCI_REG_CAP_HCCPARAMS1);
		xhci->dboff = xhci_read32(xhci, XHCI_REG_CAP_DBOFF) & XHCI_REG_CAP_DBOFF__MASK;
		xhci->rtsoff = xhci_read32(xhci, XHCI_REG_CAP_RTSOFF) & XHCI_REG_CAP_RTSOFF__MASK;

		fprintf(stderr, "xhci: capProbe[%u] caplength=0x%02x HCIVERSION=0x%04x\n",
			attempt, xhci->caplength, xhci->version);

		/* Valid xHCI cap space: caplength is at least 0x20, version is
		 * exactly XHCI_SUPPORTED_VERSION (0x0100). The "poison" pattern
		 * (caplength=0xad, version=0xdead) fails both checks below. */
		if ((xhci->caplength >= 0x20u) && (xhci->caplength <= 0xffu) &&
			(xhci->version == XHCI_SUPPORTED_VERSION)) {
			return -ENOSYS;
		}

		/* Wait for the bridge to settle and retry. The wait grows
		 * slightly with each attempt; on every Pi 4 boot we've seen
		 * the recovery — when it happens at all — within the first
		 * one or two retries. */
		usleep(100000u);
	}

	if ((xhci->caplength < 0x20u) || (xhci->caplength > 0xffu)) {
		fprintf(stderr, "xhci: invalid caplength 0x%02x (HCIVERSION 0x%04x) after %u retries\n",
			xhci->caplength, xhci->version, attempt);
		return -ENODEV;
	}

	fprintf(stderr, "xhci: unsupported version 0x%04x (caplength 0x%02x) after %u retries\n",
		xhci->version, xhci->caplength, attempt);
	return -ENODEV;
}


static int xhci_waitOpBits(xhci_t *xhci, uintptr_t off, uint32_t mask, uint32_t value, unsigned timeoutMs)
{
	uint32_t reg;
	unsigned i;

	/* Tight ~1ms burst at 50us catches the common case where the
	 * controller flips the bit in tens of microseconds; then fall back
	 * to 1ms granularity for the remainder of timeoutMs. */
	for (i = 0u; i < 20u; ++i) {
		reg = xhci_opRead32(xhci, off);
		if ((reg & mask) == value) {
			return EOK;
		}
		usleep(50);
	}

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
	uint32_t usbsts;
	int err;

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_CNR, 0u, XHCI_CNR_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: controller not ready before reset\n");
		return err;
	}

	/* xHCI spec 5.4.1: writing HCRST while HCHalted (HCH) is 0 has
	 * UNDEFINED behaviour. On the Pi 4 the VL805 firmware-load may
	 * leave the controller running (HCH=0); empirically writing HCRST
	 * in that state leaves the bit set indefinitely and our wait below
	 * times out. Match Linux's xhci_reset(): if the controller isn't
	 * already halted, clear USBCMD.R/S and wait for HCH=1 before
	 * proceeding. (Linux just aborts in this case with a warning; we
	 * stop first because the boot path can't recover from a no-USB
	 * outcome and stop-then-reset is the canonical sequence in every
	 * other xHCI driver we surveyed — FreeBSD usb/controller/xhci.c
	 * `xhci_halt`, NetBSD xhci.c, Circle USBControllerXHCI.) */
	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & XHCI_REG_OP_USBSTS_HCH) == 0u) {
		usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
		xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd & ~XHCI_REG_OP_USBCMD_RS);

		err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH,
			XHCI_REG_OP_USBSTS_HCH, XHCI_RUNSTOP_TIMEOUT_MS);
		if (err < 0) {
			fprintf(stderr, "xhci: failed to halt before reset (usbsts=0x%08x)\n", usbsts);
			return err;
		}
	}

	usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
	xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd | XHCI_REG_OP_USBCMD_HCRST);

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBCMD, XHCI_REG_OP_USBCMD_HCRST, 0u, XHCI_HCRST_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: reset timeout (usbsts=0x%08x)\n", xhci_opRead32(xhci, XHCI_REG_OP_USBSTS));
		return err;
	}

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_CNR, 0u, XHCI_CNR_TIMEOUT_MS);
	if (err < 0) {
		fprintf(stderr, "xhci: controller not ready after reset\n");
		return err;
	}

	/* (2026-05-24) Removed the post-HCRST bcm2711_pcie_resettleOutboundWindow
	 * call. Empirically the multiple bridge re-programs (initVL805 once
	 * after mailbox + here after HCRST + once more before R/S=1) gave no
	 * deterministic improvement and may have been churning bridge state.
	 * The post-mailbox re-program in bcm2711_pcie_initVL805 is the only
	 * one kept. */

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


/*
 * xhci_allocScratchpads
 *
 * Per xHCI 1.0 §4.20: when HCSPARAMS2.Max_Scratchpad_Buffers > 0 the host
 * controller requires the driver to provide an array of PAGESIZE-aligned
 * scratchpad buffers for its private use. The 64-bit physical address of
 * the pointer array is written into DCBAA[0] (slot 0 is reserved for this
 * purpose). The buffers and the array must be in place before R/S is set,
 * otherwise the run transition silently times out.
 *
 * VL805 reports HCSPARAMS2 = 0xfc000031 -> Max Scratchpad Buffers = 31,
 * so without this we hang at "xhci: run transition timeout".
 *
 * NOTE: xhci_validateRuntime currently extracts only the "Lo" field of the
 * scratchpad count (HCSPARAMS2[31:27]). Controllers reporting a non-zero
 * "Hi" field (HCSPARAMS2[25:21]) would need a wider count; VL805 reports
 * Hi=0 so this is sufficient for Pi 4 bring-up.
 */
static int xhci_allocScratchpads(xhci_t *xhci)
{
	uint64_t *array;
	uint64_t bufPhys;
	uint64_t *dcbaa;
	uint32_t i;
	void *buf;

	if (xhci->nscratchpad == 0u) {
		return EOK;
	}

	if ((xhci->pagesize & XHCI_REG_OP_PAGESIZE_4K) == 0u) {
		fprintf(stderr, "xhci: scratchpad requires 4k page support\n");
		return -ENODEV;
	}

	xhci->scratchpadArraySize = (size_t)xhci->nscratchpad * XHCI_SCRATCHPAD_ENTRY_SIZE;
	if (xhci->scratchpadArraySize < XHCI_SCRATCHPAD_ARRAY_ALIGN) {
		xhci->scratchpadArraySize = XHCI_SCRATCHPAD_ARRAY_ALIGN;
	}

	xhci->scratchpadArray = usb_allocAligned(xhci->scratchpadArraySize, XHCI_SCRATCHPAD_ARRAY_ALIGN);
	if (xhci->scratchpadArray == NULL) {
		fprintf(stderr, "xhci: failed to allocate scratchpad array\n");
		return -ENOMEM;
	}
	memset(xhci->scratchpadArray, 0, xhci->scratchpadArraySize);

	xhci->scratchpadArrayPhys = va2pa(xhci->scratchpadArray);
	if ((xhci->scratchpadArrayPhys & (XHCI_SCRATCHPAD_ARRAY_ALIGN - 1u)) != 0u) {
		fprintf(stderr, "xhci: scratchpad array misaligned\n");
		return -ENODEV;
	}
	if ((!xhci->ac64) && ((xhci->scratchpadArrayPhys >> 32) != 0u)) {
		fprintf(stderr, "xhci: scratchpad array above 32-bit address space\n");
		return -ENODEV;
	}

	xhci->scratchpadBufs = calloc(xhci->nscratchpad, sizeof(void *));
	if (xhci->scratchpadBufs == NULL) {
		fprintf(stderr, "xhci: failed to allocate scratchpad bookkeeping\n");
		return -ENOMEM;
	}

	array = (uint64_t *)xhci->scratchpadArray;
	for (i = 0u; i < xhci->nscratchpad; ++i) {
		buf = usb_allocAligned(XHCI_SCRATCHPAD_PAGE_SIZE, XHCI_SCRATCHPAD_PAGE_ALIGN);
		if (buf == NULL) {
			fprintf(stderr, "xhci: failed to allocate scratchpad buffer %u\n", (unsigned)i);
			return -ENOMEM;
		}
		memset(buf, 0, XHCI_SCRATCHPAD_PAGE_SIZE);
		xhci->scratchpadBufs[i] = buf;

		bufPhys = va2pa(buf);
		if ((bufPhys & (XHCI_SCRATCHPAD_PAGE_ALIGN - 1u)) != 0u) {
			fprintf(stderr, "xhci: scratchpad buffer %u misaligned\n", (unsigned)i);
			return -ENODEV;
		}
		if ((!xhci->ac64) && ((bufPhys >> 32) != 0u)) {
			fprintf(stderr, "xhci: scratchpad buffer %u above 32-bit address space\n", (unsigned)i);
			return -ENODEV;
		}
		array[i] = bufPhys;
	}

	dcbaa = (uint64_t *)xhci->dcbaa;
	dcbaa[0] = xhci->scratchpadArrayPhys;

	if (dcbaa[0] != xhci->scratchpadArrayPhys) {
		fprintf(stderr, "xhci: dcbaa[0] write-back mismatch\n");
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

	/* DCBAAP and CONFIG read back as written and are validated here.
	 * CRCR is special per xHCI 1.0 §5.4.5: the Command Ring Pointer
	 * (bits 63:6) and the write-only control bits (RCS/CS/CA, bits
	 * 0:2) ALWAYS read back as zero. Only CRR (bit 3, Command Ring
	 * Running) is readable. Comparing CRCR readback against the
	 * written pointer is therefore a spec-required mismatch and
	 * must NOT cause a programCommandSpace failure. The previous
	 * check broke xhci init on VL805 / BCM2711 after the outbound
	 * window started returning real device data.
	 */
	if ((xhci->dcbaap != xhci->dcbaaPhys) ||
		((xhci->config & XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK) != (xhci->nslots & XHCI_REG_OP_CONFIG_MAX_SLOTS_EN__MASK))) {
		fprintf(stderr, "xhci: command space register program mismatch (dcbaap or config)\n");
		return -ENODEV;
	}

	return EOK;
}


__attribute__((unused))
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
		fprintf(stderr, "xhci: controller error state after run (USBSTS=0x%08x ERDP_LO=0x%08x ERSTBA_LO=0x%08x DCBAAP_LO=0x%08x)\n",
			usbsts,
			xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERDP_LO),
			xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERSTBA_LO),
			xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP));
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
		fprintf(stderr, "xhci: controller error state after halt (USBSTS=0x%08x)\n", usbsts);
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

	/* Drain pending CPU writes to event ring / command ring / DCBAA
	 * before the controller starts DMA-ing from them on R/S=1.
	 * The xhci_*Write32 helpers use volatile MMIO writes (compiler
	 * ordering only) — on AArch64 the CPU store buffer can hold the
	 * regular-memory writes well past the MMIO write that triggers
	 * DMA, so the controller's first read can see uninitialized or
	 * stale data → USBSTS.HSE on the transient R/S=1.
	 * DSB SY ensures all preceding loads and stores are observed by
	 * all observers (including the PCIe DMA agent) before the next
	 * memory access. */
	__asm__ volatile("dsb sy" ::: "memory");

	/* HSE-on-R/S=1 soft retry (2026-05-24): on bridge state where the
	 * controller's first DMA fetch fails (USBSTS.HSE = 1, controller
	 * self-halts), clear HSE (write-1-to-clear), drop R/S, wait for
	 * HCH=1, and try R/S=1 again. Up to 3 attempts. If 2nd or 3rd
	 * attempt succeeds we recover from the intermittent HSE without
	 * the entire xhci_init re-running. Per xHCI 1.2 §5.4.2, HSE is
	 * RW1C in USBSTS. */
	{
		unsigned attempt;
		for (attempt = 0u; attempt < 3u; ++attempt) {
			usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
			xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd | XHCI_REG_OP_USBCMD_RS);

			err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, 0u, XHCI_RUNSTOP_TIMEOUT_MS);
			if (err < 0) {
				fprintf(stderr, "xhci: run transition timeout (attempt %u)\n", attempt);
				return err;
			}

			usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
			if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) == 0u) {
				if (attempt > 0u) {
					fprintf(stderr, "xhci: enterRunState recovered on attempt %u\n", attempt);
				}
				return EOK;
			}

			fprintf(stderr, "xhci: enterRunState HSE attempt %u (USBSTS=0x%08x)\n", attempt, usbsts);

			/* Clear HSE (W1C), drop R/S, wait for halt, retry. */
			xhci_opWrite32(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE);
			usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
			xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd & ~XHCI_REG_OP_USBCMD_RS);
			(void)xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, XHCI_REG_OP_USBSTS_HCH, XHCI_RUNSTOP_TIMEOUT_MS);

			__asm__ volatile("dsb sy" ::: "memory");
		}

		fprintf(stderr, "xhci: enterRunState gave up after 3 HSE attempts (USBSTS=0x%08x ERDP_LO=0x%08x ERSTBA_LO=0x%08x DCBAAP_LO=0x%08x)\n",
			usbsts,
			xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERDP_LO),
			xhci_rtRead32(xhci, XHCI_REG_RT_IR_ERSTBA_LO),
			xhci_opRead32(xhci, XHCI_REG_OP_DCBAAP));
	}
	return -ENODEV;
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


static unsigned int xhci_usbSpeedToPsi(enum usb_speed speed)
{
	switch (speed) {
		case usb_full_speed:
			return XHCI_PORT_SPEED_FULL;

		case usb_low_speed:
			return XHCI_PORT_SPEED_LOW;

		case usb_high_speed:
			return XHCI_PORT_SPEED_HIGH;

		default:
			return 0u;
	}
}


static uint16_t xhci_ep0MaxPacket(enum usb_speed speed)
{
	switch (speed) {
		case usb_low_speed:
		case usb_full_speed:
			return 8u;

		case usb_high_speed:
			return 64u;

		default:
			return 0u;
	}
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

	/* Drain any pending writes (cmd TRB cycle bit, event memset)
	 * before triggering the controller via doorbell. enterRunState
	 * already issued a DSB SY but a fresh barrier here makes the
	 * cmd ring -> doorbell handoff explicit (cmd TRB visibility is
	 * a precondition for the doorbell-triggered DMA read). */
	__asm__ volatile("dsb sy" ::: "memory");

	xhci_dbWrite32(xhci, 0u, 0u);

	for (timeoutMs = XHCI_CMD_TIMEOUT_MS; timeoutMs > 0u; --timeoutMs) {
		if ((event->control & XHCI_TRB_CONTROL_C) == (xhci->eventCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u)) {
			break;
		}

		usleep(1000);

		/* (2026-05-24) periodic doorbell re-ring. The first doorbell
		 * after R/S=1 may race the controller's RUN-state transition
		 * and get dropped. Re-ringing every 10 ms is cheap and
		 * spec-allowed (doorbell writes are idempotent). */
		if ((timeoutMs % 10u) == 0u) {
			__asm__ volatile("dsb sy" ::: "memory");
			xhci_dbWrite32(xhci, 0u, 0u);
		}
	}

	if (timeoutMs == 0u) {
		fprintf(stderr, "xhci: command completion timeout (USBSTS=0x%08x CRCR_LO=0x%08x event[0]=ctrl=0x%08x parm_lo=0x%08x cmd_phys=0x%08llx cmd_ctrl=0x%08x dboff=0x%08x USBCMD=0x%08x)\n",
			xhci_opRead32(xhci, XHCI_REG_OP_USBSTS),
			xhci_opRead32(xhci, XHCI_REG_OP_CRCR),
			event->control,
			(uint32_t)event->parameter,
			(unsigned long long)cmdPhys,
			cmd->control,
			xhci->dboff,
			xhci_opRead32(xhci, XHCI_REG_OP_USBCMD));
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


static int xhci_cmdAddressDevice(xhci_t *xhci, int setAddress)
{
	uint32_t control;

	if ((xhci->slotId == 0u) || (xhci->inputCtxPhys == 0u)) {
		return -EINVAL;
	}

	if ((!xhci->ac64) && ((xhci->inputCtxPhys >> 32) != 0u)) {
		fprintf(stderr, "xhci: input context above 32-bit address space\n");
		return -ENODEV;
	}

	control = (XHCI_TRB_TYPE_CMD_ADDRESS_DEVICE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		((uint32_t)xhci->slotId << XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_SLOTID__SHIFT);
	if (setAddress == 0) {
		control |= XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_BSR;
	}

	return xhci_cmdExec(xhci, xhci->inputCtxPhys, 0u, control, NULL);
}


static int xhci_cmdConfigureEndpoint(xhci_t *xhci, int deconfigure)
{
	uint32_t control;

	if ((xhci->slotId == 0u) || (xhci->inputCtxPhys == 0u)) {
		return -EINVAL;
	}

	control = (XHCI_TRB_TYPE_CMD_CONFIGURE_ENDPOINT << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		((uint32_t)xhci->slotId << XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_SLOTID__SHIFT);
	if (deconfigure != 0) {
		control |= XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_DC;
	}

	return xhci_cmdExec(xhci, xhci->inputCtxPhys, 0u, control, NULL);
}


static int xhci_allocSlotSpace(xhci_t *xhci)
{
	uint64_t *dcbaa;
	size_t dcbaaOffs;

	if ((xhci->slotId == 0u) || (xhci->slotId > xhci->nslots)) {
		fprintf(stderr, "xhci: invalid slot id for slot space %u\n", xhci->slotId);
		return -EINVAL;
	}

	xhci->devCtxSize = xhci->contextSize * (1u + XHCI_MAX_ENDPOINTS);
	xhci->inputCtxSize = xhci->contextSize * (XHCI_CONTEXT_INPUT + 1u + XHCI_MAX_ENDPOINTS);
	xhci->ep0RingSize = XHCI_TRANSFER_RING_SIZE;

	xhci->devCtx = usb_allocAligned(xhci->devCtxSize, XHCI_CONTEXT_ALIGN);
	if (xhci->devCtx == NULL) {
		fprintf(stderr, "xhci: failed to allocate device context\n");
		return -ENOMEM;
	}

	xhci->inputCtx = usb_allocAligned(xhci->inputCtxSize, XHCI_CONTEXT_ALIGN);
	if (xhci->inputCtx == NULL) {
		fprintf(stderr, "xhci: failed to allocate input context\n");
		return -ENOMEM;
	}

	xhci->ep0Ring = usb_allocAligned(xhci->ep0RingSize, XHCI_TRANSFER_RING_ALIGN);
	if (xhci->ep0Ring == NULL) {
		fprintf(stderr, "xhci: failed to allocate ep0 ring\n");
		return -ENOMEM;
	}

	memset(xhci->devCtx, 0, xhci->devCtxSize);
	memset(xhci->inputCtx, 0, xhci->inputCtxSize);
	memset(xhci->ep0Ring, 0, xhci->ep0RingSize);

	xhci->devCtxPhys = va2pa(xhci->devCtx);
	xhci->inputCtxPhys = va2pa(xhci->inputCtx);
	xhci->ep0RingPhys = va2pa(xhci->ep0Ring);

	if (((xhci->devCtxPhys & (XHCI_CONTEXT_ALIGN - 1u)) != 0u) ||
		((xhci->inputCtxPhys & (XHCI_CONTEXT_ALIGN - 1u)) != 0u) ||
		((xhci->ep0RingPhys & (XHCI_TRANSFER_RING_ALIGN - 1u)) != 0u)) {
		fprintf(stderr, "xhci: invalid slot-space alignment\n");
		return -ENODEV;
	}

	dcbaaOffs = xhci->slotId * XHCI_DCBAA_ENTRY_SIZE;
	if ((dcbaaOffs + XHCI_DCBAA_ENTRY_SIZE) > xhci->dcbaaSize) {
		fprintf(stderr, "xhci: slot id %u exceeds dcbaa size\n", xhci->slotId);
		return -ENODEV;
	}

	dcbaa = (uint64_t *)xhci->dcbaa;
	dcbaa[xhci->slotId] = xhci->devCtxPhys;

	if (dcbaa[xhci->slotId] != xhci->devCtxPhys) {
		fprintf(stderr, "xhci: failed to bind dcbaa slot entry\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_initEp0Ring(xhci_t *xhci)
{
	xhci_trb_t *ring;
	xhci_trb_t *link;

	xhci->ep0RingCount = xhci->ep0RingSize / XHCI_TRB_SIZE;
	if (xhci->ep0RingCount <= 1u) {
		fprintf(stderr, "xhci: ep0 ring too small\n");
		return -ENODEV;
	}

	memset(xhci->ep0Ring, 0, xhci->ep0RingSize);
	xhci->ep0CycleState = 1u;

	ring = (xhci_trb_t *)xhci->ep0Ring;
	link = &ring[xhci->ep0RingCount - 1u];
	link->parameter = xhci->ep0RingPhys;
	link->status = 0u;
	link->control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	if ((link->parameter != xhci->ep0RingPhys) ||
		(((link->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu) != XHCI_TRB_TYPE_LINK) ||
		((link->control & (XHCI_TRB_CONTROL_C | XHCI_LINK_TRB_CONTROL_TC)) !=
			(XHCI_TRB_CONTROL_C | XHCI_LINK_TRB_CONTROL_TC))) {
		fprintf(stderr, "xhci: invalid ep0 ring link trb\n");
		return -ENODEV;
	}

	return EOK;
}


static uint8_t xhci_pipeEndpointId(const usb_pipe_t *pipe)
{
	uint8_t endpointId;

	if ((pipe == NULL) || (pipe->num <= 0) || (pipe->num > 15)) {
		return 0u;
	}

	endpointId = (uint8_t)(pipe->num * 2);
	if (pipe->dir == usb_dir_in) {
		endpointId++;
	}

	return endpointId;
}


static uint8_t xhci_convertInterval(const usb_pipe_t *pipe)
{
	unsigned interval;
	unsigned i;

	if ((pipe == NULL) || (pipe->interval <= 0)) {
		return 0u;
	}

	interval = (unsigned)pipe->interval;
	if (pipe->dev->speed == usb_high_speed) {
		if (interval < 1u) {
			interval = 1u;
		}
		if (interval > 16u) {
			interval = 16u;
		}

		return (uint8_t)(interval - 1u);
	}

	for (i = 3u; i < 11u; ++i) {
		if ((125u * (1u << i)) >= (1000u * interval)) {
			break;
		}
	}

	return (uint8_t)i;
}


static int xhci_prepareAddressContext(xhci_t *xhci, usb_dev_t *dev)
{
	xhci_input_ctx_t *input;
	xhci_slot_ctx_t *slot;
	xhci_ep_ctx_t *ep0;
	unsigned int psi;
	uint16_t maxPacket;

	if ((dev == NULL) || (dev->hub == NULL) || (dev->hub->hub != NULL)) {
		return -ENOSYS;
	}

	psi = xhci_usbSpeedToPsi(dev->speed);
	maxPacket = xhci_ep0MaxPacket(dev->speed);
	if ((psi == 0u) || (maxPacket == 0u)) {
		fprintf(stderr, "xhci: unsupported device speed for address context\n");
		return -ENOSYS;
	}

	memset(xhci->inputCtx, 0, xhci->inputCtxSize);

	input = (xhci_input_ctx_t *)xhci->inputCtx;
	slot = &input->device.slot;
	ep0 = &input->device.ep[0];

	input->control.addContextFlags = XHCI_INPUT_CTRL_CTX_ADD_A0_A1;

	slot->routeString_speed_mtt_hub_entries =
		(psi << XHCI_SLOT_CTX_SPEED__SHIFT) |
		(1u << XHCI_SLOT_CTX_CONTEXT_ENTRIES__SHIFT);
	slot->maxExitLatency_rootHubPort_ports = ((uint32_t)dev->port & 0xffu) << XHCI_SLOT_CTX_ROOT_HUB_PORT__SHIFT;

	ep0->cerr_type_burst_packet =
		(3u << XHCI_EP_CTX_CERR__SHIFT) |
		(XHCI_EP_CTX_TYPE_CONTROL << XHCI_EP_CTX_TYPE__SHIFT) |
		((uint32_t)maxPacket << XHCI_EP_CTX_MAX_PACKET__SHIFT);
	ep0->trDequeuePtr = xhci->ep0RingPhys | XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS;
	ep0->averageTrbLen_maxEsitPayload = 8u;

	if ((input->control.addContextFlags != XHCI_INPUT_CTRL_CTX_ADD_A0_A1) ||
		(slot->maxExitLatency_rootHubPort_ports != (((uint32_t)dev->port & 0xffu) << XHCI_SLOT_CTX_ROOT_HUB_PORT__SHIFT)) ||
		(ep0->trDequeuePtr != (xhci->ep0RingPhys | XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS))) {
		fprintf(stderr, "xhci: invalid address context preparation\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_initInterruptInPipe(xhci_t *xhci, usb_pipe_t *pipe)
{
	xhci_input_ctx_t *input;
	xhci_ep_ctx_t *epctx;
	xhci_trb_t *ring;
	xhci_trb_t *link;
	xhci_pipePriv_t *priv;
	uint8_t endpointId;
	uint32_t interval;
	int err;

	if ((xhci == NULL) || (pipe == NULL)) {
		return -EINVAL;
	}

	if (pipe->hcdpriv != NULL) {
		return 0;
	}

	if ((pipe->type != usb_transfer_interrupt) || (pipe->dir != usb_dir_in) ||
		(pipe->dev->hub == NULL) || (pipe->dev->hub->hub != NULL) ||
		(pipe->dev->address != (int)xhci->slotId)) {
		return -EINVAL;
	}

	endpointId = xhci_pipeEndpointId(pipe);
	if (endpointId == 0u) {
		return -EINVAL;
	}

	priv = calloc(1, sizeof(*priv));
	if (priv == NULL) {
		return -ENOMEM;
	}

	priv->ringSize = XHCI_TRANSFER_RING_SIZE;
	priv->endpointId = endpointId;
	priv->endpointType = XHCI_EP_CTX_TYPE_INTERRUPT_IN;
	priv->ring = usb_allocAligned(priv->ringSize, XHCI_TRANSFER_RING_ALIGN);
	if (priv->ring == NULL) {
		free(priv);
		return -ENOMEM;
	}

	memset(priv->ring, 0, priv->ringSize);
	priv->ringPhys = va2pa(priv->ring);
	priv->ringCount = priv->ringSize / XHCI_TRB_SIZE;
	priv->cycleState = 1u;
	if (((priv->ringPhys & (XHCI_TRANSFER_RING_ALIGN - 1u)) != 0u) || (priv->ringCount <= 1u)) {
		usb_freeAligned(priv->ring, priv->ringSize);
		free(priv);
		return -EINVAL;
	}

	ring = (xhci_trb_t *)priv->ring;
	link = &ring[priv->ringCount - 1u];
	link->parameter = priv->ringPhys;
	link->status = 0u;
	link->control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	memset(xhci->inputCtx, 0, xhci->inputCtxSize);
	input = (xhci_input_ctx_t *)xhci->inputCtx;
	input->control.addContextFlags = 1u << endpointId;
	input->control.dropContextFlags = 1u << endpointId;

	epctx = &input->device.ep[endpointId - 1u];
	interval = xhci_convertInterval(pipe);
	epctx->epState_mult_streams_interval = interval << XHCI_EP_CTX_INTERVAL__SHIFT;
	epctx->cerr_type_burst_packet =
		(3u << XHCI_EP_CTX_CERR__SHIFT) |
		((uint32_t)priv->endpointType << XHCI_EP_CTX_TYPE__SHIFT) |
		(0u << XHCI_EP_CTX_MAX_BURST__SHIFT) |
		((uint32_t)pipe->maxPacketLen << XHCI_EP_CTX_MAX_PACKET__SHIFT);
	epctx->trDequeuePtr = priv->ringPhys | XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS;
	epctx->averageTrbLen_maxEsitPayload = 16u |
		((uint32_t)pipe->maxPacketLen << XHCI_EP_CTX_MAX_ESIT_PAYLOAD__SHIFT);

	err = xhci_cmdConfigureEndpoint(xhci, 0);
	if (err < 0) {
		usb_freeAligned(priv->ring, priv->ringSize);
		free(priv);
		return err;
	}

	pipe->hcdpriv = priv;
	xhci->interruptPriv = priv;
	return 0;
}


static int xhci_submitInterruptIn(xhci_t *xhci, usb_transfer_t *t, usb_pipe_t *pipe)
{
	xhci_pipePriv_t *priv;
	xhci_trb_t *ring;
	xhci_trb_t *event;
	xhci_trb_t *link;
	int err;

	if ((xhci == NULL) || (t == NULL) || (pipe == NULL) || (t->buffer == NULL) || (t->size == 0u)) {
		return -EINVAL;
	}

	priv = (xhci_pipePriv_t *)pipe->hcdpriv;
	if ((priv == NULL) || (priv->endpointId == 0u) || (priv->ring == NULL)) {
		return -EINVAL;
	}

	if (priv->pendingTransfer != NULL) {
		return -EBUSY;
	}

	ring = (xhci_trb_t *)priv->ring;
	memset(ring, 0, priv->ringSize);
	ring[0].parameter = va2pa(t->buffer);
	ring[0].status = (uint32_t)t->size;
	ring[0].control = XHCI_TRB_CONTROL_C |
		XHCI_TRANSFER_TRB_CONTROL_IOC |
		XHCI_TRANSFER_TRB_CONTROL_ISP |
		(XHCI_TRB_TYPE_NORMAL << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	link = &ring[priv->ringCount - 1u];
	link->parameter = priv->ringPhys;
	link->status = 0u;
	link->control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	event = (xhci_trb_t *)xhci->eventRing;
	memset(event, 0, sizeof(*event));
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, (uint32_t)(xhci->eventRingPhys >> 32));
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO,
		(uint32_t)(xhci->eventRingPhys & XHCI_REG_RT_IR_ERDP_LO__MASK) | XHCI_REG_RT_IR_ERDP_LO_EHB);

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	priv->pendingTransfer = t;
	priv->pendingTrbPhys = priv->ringPhys;
	xhci_dbWrite32(xhci, xhci->slotId * sizeof(uint32_t), priv->endpointId);

	return 0;
}


static int xhci_ep0ControlRead(xhci_t *xhci, usb_transfer_t *t)
{
	xhci_trb_t *ring;
	xhci_trb_t *event;
	uint64_t statusTrbPhys;
	uint32_t type;
	uint32_t completion;
	uint32_t endpointId;
	uint32_t slotId;
	uint32_t residual;
	unsigned timeoutMs;
	int err;

	if ((xhci == NULL) || (t == NULL) || (t->setup == NULL) || (t->buffer == NULL) || (t->size == 0u)) {
		return -EINVAL;
	}

	ring = (xhci_trb_t *)xhci->ep0Ring;
	memset(ring, 0, xhci->ep0RingSize);

	ring[0].parameter =
		(uint64_t)t->setup->bmRequestType |
		((uint64_t)t->setup->bRequest << 8) |
		((uint64_t)t->setup->wValue << 16) |
		((uint64_t)t->setup->wIndex << 32) |
		((uint64_t)t->setup->wLength << 48);
	ring[0].status = sizeof(usb_setup_packet_t);
	ring[0].control = XHCI_TRB_CONTROL_C |
		(XHCI_TRB_TYPE_SETUP_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		(XHCI_TRANSFER_TRB_CONTROL_TRT_IN << XHCI_TRANSFER_TRB_CONTROL_TRT__SHIFT) |
		XHCI_TRANSFER_TRB_CONTROL_IDT;

	ring[1].parameter = va2pa(t->buffer);
	ring[1].status = (uint32_t)t->size;
	ring[1].control = XHCI_TRB_CONTROL_C |
		(XHCI_TRB_TYPE_DATA_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		XHCI_TRANSFER_TRB_CONTROL_DIR_IN |
		XHCI_TRANSFER_TRB_CONTROL_ISP;

	ring[2].parameter = 0u;
	ring[2].status = 0u;
	ring[2].control = XHCI_TRB_CONTROL_C |
		(XHCI_TRB_TYPE_STATUS_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		XHCI_TRANSFER_TRB_CONTROL_IOC;

	ring[xhci->ep0RingCount - 1u].parameter = xhci->ep0RingPhys;
	ring[xhci->ep0RingCount - 1u].status = 0u;
	ring[xhci->ep0RingCount - 1u].control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	statusTrbPhys = xhci->ep0RingPhys + (2u * XHCI_TRB_SIZE);
	event = (xhci_trb_t *)xhci->eventRing;
	memset(event, 0, sizeof(*event));

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	xhci_dbWrite32(xhci, xhci->slotId * sizeof(uint32_t), 1u);

	for (timeoutMs = XHCI_CMD_TIMEOUT_MS; timeoutMs > 0u; --timeoutMs) {
		if ((event->control & XHCI_TRB_CONTROL_C) == (xhci->eventCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u)) {
			break;
		}

		usleep(1000);
	}

	if (timeoutMs == 0u) {
		fprintf(stderr, "xhci: transfer completion timeout\n");
		(void)xhci_enterHaltedState(xhci);
		return -ETIMEDOUT;
	}

	type = (event->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;
	completion = (event->status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
	endpointId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
	slotId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;
	residual = event->status & XHCI_TRANSFER_EVENT_TRB_STATUS_TRB_TRANSFER_LENGTH__MASK;

	if ((type != XHCI_TRB_TYPE_EVENT_TRANSFER) || (event->parameter != statusTrbPhys) ||
		(endpointId != 1u) || (slotId != xhci->slotId)) {
		fprintf(stderr, "xhci: invalid transfer completion event\n");
		(void)xhci_enterHaltedState(xhci);
		return -ENODEV;
	}

	err = xhci_enterHaltedState(xhci);
	if (err < 0) {
		return err;
	}

	if ((completion != XHCI_TRB_COMPLETION_CODE_SUCCESS) && (completion != XHCI_TRB_COMPLETION_CODE_SHORT_PACKET)) {
		fprintf(stderr, "xhci: transfer completion code %u\n", completion);
		return -ENODEV;
	}

	if (residual > t->size) {
		fprintf(stderr, "xhci: invalid transfer residual %u\n", residual);
		return -ENODEV;
	}

	return (int)(t->size - residual);
}


static int xhci_ep0ControlWriteNoData(xhci_t *xhci, usb_transfer_t *t)
{
	xhci_trb_t *ring;
	xhci_trb_t *event;
	uint64_t statusTrbPhys;
	uint32_t type;
	uint32_t completion;
	uint32_t endpointId;
	uint32_t slotId;
	unsigned timeoutMs;
	int err;

	if ((xhci == NULL) || (t == NULL) || (t->setup == NULL) || (t->size != 0u) || (t->setup->wLength != 0u)) {
		return -EINVAL;
	}

	ring = (xhci_trb_t *)xhci->ep0Ring;
	memset(ring, 0, xhci->ep0RingSize);

	ring[0].parameter =
		(uint64_t)t->setup->bmRequestType |
		((uint64_t)t->setup->bRequest << 8) |
		((uint64_t)t->setup->wValue << 16) |
		((uint64_t)t->setup->wIndex << 32) |
		((uint64_t)t->setup->wLength << 48);
	ring[0].status = sizeof(usb_setup_packet_t);
	ring[0].control = XHCI_TRB_CONTROL_C |
		(XHCI_TRB_TYPE_SETUP_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		(XHCI_TRANSFER_TRB_CONTROL_TRT_NONE << XHCI_TRANSFER_TRB_CONTROL_TRT__SHIFT) |
		XHCI_TRANSFER_TRB_CONTROL_IDT;

	ring[1].parameter = 0u;
	ring[1].status = 0u;
	ring[1].control = XHCI_TRB_CONTROL_C |
		(XHCI_TRB_TYPE_STATUS_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		XHCI_TRANSFER_TRB_CONTROL_DIR_IN |
		XHCI_TRANSFER_TRB_CONTROL_IOC;

	ring[xhci->ep0RingCount - 1u].parameter = xhci->ep0RingPhys;
	ring[xhci->ep0RingCount - 1u].status = 0u;
	ring[xhci->ep0RingCount - 1u].control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	statusTrbPhys = xhci->ep0RingPhys + XHCI_TRB_SIZE;
	event = (xhci_trb_t *)xhci->eventRing;
	memset(event, 0, sizeof(*event));

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	xhci_dbWrite32(xhci, xhci->slotId * sizeof(uint32_t), 1u);

	for (timeoutMs = XHCI_CMD_TIMEOUT_MS; timeoutMs > 0u; --timeoutMs) {
		if ((event->control & XHCI_TRB_CONTROL_C) == (xhci->eventCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u)) {
			break;
		}

		usleep(1000);
	}

	if (timeoutMs == 0u) {
		fprintf(stderr, "xhci: transfer completion timeout\n");
		(void)xhci_enterHaltedState(xhci);
		return -ETIMEDOUT;
	}

	type = (event->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;
	completion = (event->status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
	endpointId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
	slotId = (event->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;

	if ((type != XHCI_TRB_TYPE_EVENT_TRANSFER) || (event->parameter != statusTrbPhys) ||
		(endpointId != 1u) || (slotId != xhci->slotId)) {
		fprintf(stderr, "xhci: invalid transfer completion event\n");
		(void)xhci_enterHaltedState(xhci);
		return -ENODEV;
	}

	err = xhci_enterHaltedState(xhci);
	if (err < 0) {
		return err;
	}

	if (completion != XHCI_TRB_COMPLETION_CODE_SUCCESS) {
		fprintf(stderr, "xhci: transfer completion code %u\n", completion);
		return -ENODEV;
	}

	return 0;
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

	/* Enable Interrupter 0 (IE=1). Phoenix polls the event ring rather
	 * than relying on the IRQ, so the IE bit shouldn't be strictly
	 * required for event delivery. However Linux's xhci_run_finished
	 * enables IMAN.IE before USBCMD.RUN per xHCI 4.2/5.5.2 — some
	 * controllers may require this for the interrupter to be considered
	 * "armed" and for the controller's internal R/S=1 state machine to
	 * succeed. Set IMAN.IE; clear IP (write-1-to-clear) at the same time
	 * so any stale pending bit doesn't immediately latch IRQ. */
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_IMOD, 0u);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_IMAN, XHCI_REG_RT_IR_IMAN_IE | XHCI_REG_RT_IR_IMAN_IP);

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

	/* Single-process merged design: this xhci HCD owns the BCM2711 PCIe
	 * bridge bring-up. Call it here BEFORE mmap so that VL805 BAR0 is
	 * programmed and the outbound window is set up against THIS process'
	 * page table. Previously pcie was a separate fire-and-exit daemon
	 * that brought up the bridge in its own address space; xhci's mmap
	 * of the outbound window from a different process hit a per-mmap
	 * bridge translation state and returned 0xdead patterns. Following
	 * the canonical Phoenix-RTOS pattern (imx6ull, imxrt106x/117x, ia32)
	 * where the USB process owns both bus init and HCD init eliminates
	 * the cross-process race.
	 *
	 * NB: a previous Phase-D experiment (commit 011c27a... reverted)
	 * tried wrapping this in a 3× outer retry that re-invoked
	 * bcm2711_pcie_initVL805 on cap-probe poison. Result was 4/4
	 * rc=-19 — re-running the bridge bring-up DESTABILISES the bridge
	 * (the host-bridge config-space mapping is leaked on purpose by
	 * design, and re-running pcie_cfgInitBcm2711 either takes a
	 * second mmap onto the already-mapped region or churns the
	 * outbound translation in a way that's worse than not retrying).
	 * Keep the bridge bring-up to a single attempt; recovery has to
	 * be at finer granularity (e.g. just re-program the outbound
	 * window, not the whole bridge). */
	err = bcm2711_pcie_initVL805();
	if (err != 0) {
		return err;
	}

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
						err = xhci_allocScratchpads(xhci);
					}
					if (err == 0) {
						err = xhci_programCommandSpace(xhci);
						/* xHCI 1.2 §4.5: software MUST have programmed
						 * Max Slots, DCBAAP, Command Ring, AND Event
						 * Ring 0 (ERSTBA) before setting R/S=1. Toggling
						 * R/S without a valid event ring leaves the
						 * controller's event handling undefined; on VL805
						 * we see USBSTS.HCE/HSE set on transition.
						 * Allocate + program the event ring FIRST, then
						 * run the R/S selftest. */
						if (err == 0) {
							err = xhci_allocEventRing(xhci);
							if (err == 0) {
								err = xhci_programEventRing(xhci);
								/* xhci_runStateSelftest (toggle R/S then back
								 * to verify the transition) was a Phoenix
								 * addition not done by Linux/FreeBSD/Circle.
								 * On VL805 the controller spends real time
								 * processing the brief R/S=1 (port scan,
								 * device discovery) and the subsequent
								 * R/S=0 halt-transition can't meet our 250ms
								 * timeout. Skip the selftest; cmdNoopSelftest
								 * already enters the run state via cmdExec,
								 * which is the canonical "controller alive"
								 * check. */
								if (err == 0) {
									err = xhci_cmdNoopSelftest(xhci);
									if (err == 0) {
										err = xhci_cmdEnableSlot(xhci, &xhci->slotId);
										if (err == 0) {
											err = xhci_allocSlotSpace(xhci);
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
		/* The chain above calls fprintf(stderr, ...) at each helper's
		 * failure site, but the usb daemon switches stderr to 4 KiB
		 * fully-buffered mode at boot (TD-12) — so without an explicit
		 * flush these diagnostics never reach UART when xhci_init bails
		 * before the buffer fills. Flush here so the operator can see
		 * which xhci_* helper actually failed. */
		fflush(stderr);
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
	xhci_t *xhci = (xhci_t *)hcd->priv;
	usb_setup_packet_t *setup = t->setup;
	int err;

	if (usb_isRoothub(pipe->dev) != 0) {
		return xhci_roothubReq(pipe->dev, t);
	}

	if ((pipe->dev->address == 0) && (xhci != NULL)) {
		err = xhci_initEp0Ring(xhci);
		if (err < 0) {
			return err;
		}

		err = xhci_prepareAddressContext(xhci, pipe->dev);
		if (err < 0) {
			return err;
		}

		if ((setup != NULL) && (setup->bRequest == REQ_SET_ADDRESS)) {
			if ((uint8_t)setup->wValue != xhci->slotId) {
				fprintf(stderr, "xhci: requested address %u mismatches slot %u\n",
					(uint8_t)setup->wValue, xhci->slotId);
				return -ENOSYS;
			}

			err = xhci_cmdAddressDevice(xhci, 1);
			if (err < 0) {
				return err;
			}

			usb_transferFinished(t, 0);
			return 0;
		}
	}

	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub == NULL) &&
		(pipe->dev->address == (int)xhci->slotId) &&
		(setup->bRequest == REQ_GET_DESCRIPTOR) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_in) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_DEV2HOST) &&
		(EXTRACT_REQ_TYPE(setup->bmRequestType) == REQUEST_TYPE_STANDARD) &&
		((setup->bmRequestType & 0x1f) == REQUEST_RECIPIENT_DEVICE)) {
		err = xhci_initEp0Ring(xhci);
		if (err < 0) {
			return err;
		}

		err = xhci_ep0ControlRead(xhci, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, err);
		return 0;
	}

	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub == NULL) &&
		(pipe->dev->address == (int)xhci->slotId) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_out) &&
		(t->size == 0u) &&
		(setup->wLength == 0u) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_HOST2DEV) &&
		((((EXTRACT_REQ_TYPE(setup->bmRequestType) == REQUEST_TYPE_STANDARD) &&
			((setup->bmRequestType & 0x1f) == REQUEST_RECIPIENT_DEVICE) &&
			(setup->bRequest == REQ_SET_CONFIGURATION)) ||
			((EXTRACT_REQ_TYPE(setup->bmRequestType) == REQUEST_TYPE_CLASS) &&
			((setup->bmRequestType & 0x1f) == REQUEST_RECIPIENT_INTERFACE) &&
			((setup->bRequest == CLASS_REQ_SET_PROTOCOL) || (setup->bRequest == CLASS_REQ_SET_IDLE)))))) {
		err = xhci_initEp0Ring(xhci);
		if (err < 0) {
			return err;
		}

		err = xhci_ep0ControlWriteNoData(xhci, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, 0);
		return 0;
	}

	if ((xhci != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub == NULL) &&
		(pipe->dev->address == (int)xhci->slotId) &&
		(t->type == usb_transfer_interrupt) &&
		(t->direction == usb_dir_in)) {
		err = xhci_initInterruptInPipe(xhci, pipe);
		if (err < 0) {
			return err;
		}

		return xhci_submitInterruptIn(xhci, t, pipe);
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
	xhci_pipePriv_t *priv;
	xhci_t *xhci = (xhci_t *)hcd->priv;

	if ((pipe == NULL) || (pipe->hcdpriv == NULL)) {
		return;
	}

	priv = (xhci_pipePriv_t *)pipe->hcdpriv;
	pipe->hcdpriv = NULL;
	if (xhci->interruptPriv == priv) {
		xhci->interruptPriv = NULL;
	}

	if (priv->ring != NULL) {
		usb_freeAligned(priv->ring, priv->ringSize);
	}

	free(priv);
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
