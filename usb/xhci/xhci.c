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
#include <sys/debug.h>
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
#define XHCI_REG_OP_USBCMD_INTE  (1u << 2)
#define XHCI_REG_OP_USBCMD_HSEE  (1u << 3)
#define XHCI_REG_OP_USBSTS_HCH   (1u << 0)
#define XHCI_REG_OP_USBSTS_HSE   (1u << 2)
#define XHCI_REG_OP_USBSTS_EINT  (1u << 3)
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
/* xHCI 1.2 §6.2.2 Slot Context bit layout (word indices match the struct
 * fields: word0 = routeString_speed_mtt_hub_entries, word1 =
 * maxExitLatency_rootHubPort_ports, word2 = ttHubSlot_ttPort_ttt_intrTarget). */
#define XHCI_SLOT_CTX_ROUTE_STRING__MASK 0xfffffu /* word0 bits 0-19 (no shift) */
#define XHCI_SLOT_CTX_MTT  (1u << 25)             /* word0 bit 25: Multi-TT */
#define XHCI_SLOT_CTX_HUB  (1u << 26)             /* word0 bit 26: device is a hub */
#define XHCI_SLOT_CTX_NUMBER_OF_PORTS__SHIFT 24u  /* word1 bits 24-31 */
#define XHCI_SLOT_CTX_TT_HUB_SLOT_ID__SHIFT 0u    /* word2 bits 0-7 */
#define XHCI_SLOT_CTX_TT_PORT_NUMBER__SHIFT 8u    /* word2 bits 8-15 */
#define XHCI_SLOT_CTX_TT_THINK_TIME__SHIFT 16u    /* word2 bits 16-17 */
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
#define XHCI_CMD_TIMEOUT_MS      1000u  /* xHCI commands (enable-slot, address-device) can need ~hundreds of ms; 1000 ms budget. */
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


/* Depth of the cross-consumer event stash (see xhci_eventAwait). Bounds the
 * number of in-flight events that belong to the other consumer; enumeration is
 * sequential so only a handful are ever live at once. */
#define XHCI_EVENT_STASH_MAX 16u


/* Per-device slot state. The controller drives one Device Slot per attached
 * device; everything keyed by slot id (device context, ep0 transfer ring,
 * addressed flag) lives here. The shared input context and the various size
 * constants stay on xhci_t — commands are serialised, so a single input
 * context is reused across slots. For now only slots[0] (the primary/rig slot,
 * slotId 1) is ever populated; the table generalises the single-slot path. */
typedef struct {
	uint8_t slotId;
	uint8_t addressed;        /* this slot has had Address Device issued */
	uint8_t hubFixedUp;       /* this slot's (hub) ctx has had Hub=1/NumPorts/TTT applied */
	uint16_t maxPacket;
	void *dev;                /* framework usb_dev_t this slot was allocated for (NULL = unused/primary) */
	void *devCtx;
	uint64_t devCtxPhys;
	void *ep0Ring;
	uint64_t ep0RingPhys;
	uint32_t ep0RingCount;
	uint32_t ep0CycleState;   /* producer cycle state for the ep0 transfer ring */
	uint32_t ep0Enqueue;      /* persistent ep0 producer index (xhci_ep0Push) */
	struct xhci_pipePriv *interruptPriv; /* this slot's interrupt-IN pipe (NULL = none) */
} xhci_slot_t;


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
	/* Per-port (bit = port number) "connect announced" latch. A device
	 * already attached before this controller's bring-up shows CCS=1 with
	 * no fresh CSC change bit, so the change-bit-driven hub poll never
	 * enumerates it. We synthesize a one-shot C_CONNECTION for such ports;
	 * this latch records the ack (ClearPortFeature C_CONNECTION) so we stop
	 * re-synthesizing, and is cleared on disconnect to allow re-announce. */
	uint32_t portConnAnnounced;
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
	void *inputCtx;
	size_t eventRingSize;
	size_t erstSize;
	size_t devCtxSize;
	size_t inputCtxSize;
	size_t ep0RingSize;
	uint64_t eventRingPhys;
	uint64_t erstPhys;
	uint64_t inputCtxPhys;
	uint32_t cmdRingCount;
	uint32_t cmdCycleState;
	uint32_t cmdEnqueue;      /* persistent command-ring producer index (xhci_cmdExec) */
	uint32_t eventRingTrbs;
	uint32_t eventCycleState; /* consumer cycle state (CCS) for the shared event-ring dequeue */
	uint32_t eventDeq;        /* shared event-ring dequeue index (xhci_eventAwait) */
	handle_t eventLock;       /* serialises shared event-ring consumption between the
	                             enumeration path (cmd/ep0) and the roothub status thread */
	xhci_trb_t eventStash[XHCI_EVENT_STASH_MAX]; /* events consumed for the *other* consumer,
	                             parked here so a dispatch pass never discards them */
	uint32_t eventStashCount;
	xhci_slot_t slots[8];     /* per-device slot state; slots[0] is the primary/rig slot */
	xhci_slot_t *cur;         /* slot currently driven by the ep0/addressing path */
	uint32_t erstsz;
	uint32_t erstbaLo;
	uint32_t erstbaHi;
	uint32_t erdpLo;
	uint32_t erdpHi;
	uint64_t erstba;
	uint64_t erdp;
	uint8_t crcrPublished;    /* CRCR re-published once before the first command */
	uint8_t keepRunning;      /* run the controller continuously (rig-handoff path) — never R/S=0 between ops */
	uint8_t running;          /* controller is in R/S=1: enterRunState is then a no-op (skip per-command re-settle) */
	unsigned ac64 : 1;
	unsigned spr : 1;
} xhci_t;


typedef struct xhci_pipePriv {
	void *ring;
	size_t ringSize;
	uint64_t ringPhys;
	uint32_t ringCount;
	uint32_t cycleState;      /* producer cycle bit (toggles on each Link-TRB wrap) */
	uint32_t enqueue;         /* producer index into ring[0..ringCount-2] */
	uint8_t slotId;           /* xHCI slot this interrupt pipe belongs to */
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

	/* Ring writes (TRBs + cycle bits) live in Normal-NonCacheable DMA memory;
	 * the doorbell is Device memory. ARM does NOT order a Normal-NC store before
	 * a later Device store without an explicit barrier, so the controller can act
	 * on the doorbell and DMA-read a ring whose newest TRB/cycle-bit has not yet
	 * reached DRAM — it then finds no work and the transfer/command never
	 * completes (a silent, per-transaction, timing-dependent failure). The
	 * command ring path open-coded this barrier (xhci_cmdExec); centralising it
	 * here gives the ep0/transfer and interrupt-IN doorbells the same guarantee.
	 * Mirrors Linux xhci_ring_*_doorbell(), which wmb() before the DB write. */
	__asm__ volatile("dsb sy" ::: "memory");
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

	{
		unsigned s;
		for (s = 0u; s < (sizeof(xhci->slots) / sizeof(xhci->slots[0])); ++s) {
			if (xhci->slots[s].devCtx != NULL) {
				usb_freeAligned(xhci->slots[s].devCtx, xhci->devCtxSize);
			}
			if (xhci->slots[s].ep0Ring != NULL) {
				usb_freeAligned(xhci->slots[s].ep0Ring, xhci->ep0RingSize);
			}
		}
	}

	if (xhci->inputCtx != NULL) {
		usb_freeAligned(xhci->inputCtx, xhci->inputCtxSize);
	}

	free(xhci);
}


static int xhci_eventAwait(xhci_t *xhci, uint32_t wantType, uint64_t wantParam, uint8_t wantSlot, uint32_t wantEp, unsigned timeoutMs, xhci_trb_t *out);


static void xhci_roothubStatusThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	xhci_t *xhci = (xhci_t *)hcd->priv;
	usb_dev_t *hub;
	xhci_pipePriv_t *priv;
	xhci_trb_t ev;
	usb_transfer_t *t;
	uint32_t status;
	uint32_t completion;
	uint32_t residual;
	unsigned sleepUs;
	unsigned s;
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

		/* Poll every active interrupt-IN pipe once per pass. Each slot owns its
		 * own pipe (slots[0] is the external hub's status-change ep; a device
		 * behind a non-root hub uses its own slot). The shared event dispatcher
		 * keys completions by (slot, endpoint), so each pipe must be awaited with
		 * its own slotId — NOT a hardcoded slots[0]. */
		for (s = 0u; s < (sizeof(xhci->slots) / sizeof(xhci->slots[0])); ++s) {
			priv = xhci->slots[s].interruptPriv;
			if ((priv == NULL) || (priv->pendingTransfer == NULL)) {
				continue;
			}

			sleepUs = 1000u;
			/* Single non-blocking poll of the shared event ring (locked,
			 * stash-aware) for this interrupt endpoint's completion. The
			 * dispatcher owns the dequeue/ERDP/cycle and the controller keeps
			 * running (keepRunning) — so no ERDP-reset or R/S=0 here. */
			if (xhci_eventAwait(xhci, XHCI_TRB_TYPE_EVENT_TRANSFER, priv->pendingTrbPhys,
					priv->slotId, priv->endpointId, 1u, &ev) == EOK) {
				t = priv->pendingTransfer;
				completion = (ev.status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >>
					XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
				residual = ev.status & XHCI_TRANSFER_EVENT_TRB_STATUS_TRB_TRANSFER_LENGTH__MASK;

				if ((residual <= t->size) &&
					((completion == XHCI_TRB_COMPLETION_CODE_SUCCESS) ||
					(completion == XHCI_TRB_COMPLETION_CODE_SHORT_PACKET))) {
					ret = (int)(t->size - residual);
				}
				else {
					ret = -ENODEV;
				}

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

	/* The ep0/addressing path drives one slot at a time via xhci->cur. For the
	 * single-slot flow that is always the primary slot, slots[0]. */
	xhci->cur = &xhci->slots[0];

	/* Serialises the shared event-ring dispatcher (xhci_eventAwait) between the
	 * enumeration path and the roothub status thread. Created before any command
	 * is issued (the rig handoff's EnableSlot is the first event consumer). */
	if (mutexCreate(&xhci->eventLock) != 0) {
		free(xhci);
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
		/* MAP_UNCACHED (Device-nGnRnE, no early write-ack) matches the
		 * working lwip 'X' mapping; see USB-FIX-22 in bcm2711-pcie.c. */
		xhci->mmio = mmap(NULL, xhci->mapSz, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
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
		debug("xhci_reset: CNR did not clear before reset\n");
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
		return err;
	}

	err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_CNR, 0u, XHCI_CNR_TIMEOUT_MS);
	if (err < 0) {
		debug("xhci_reset: CNR did not clear after reset\n");
		return err;
	}

	/* 100 ms settling window after HCRST. Empirically the BCM2711
	 * bridge and VL805 internal state can be in a transient mode
	 * even after CNR clears: ERSTBA / DCBAAP writes that immediately
	 * follow sometimes don't reach the controller, manifesting as
	 * HSE later when the controller tries to DMA. Linux's xhci-pci
	 * has a similar post-reset delay (xhci_handshake + udelay). */
	usleep(100000);

	/* HCRST may churn BCM2711 bridge state analogous to the
	 * mailbox-notify path. Re-program the outbound window AND inbound
	 * BAR1/BAR2 here so the subsequent DCBAAP/CRCR/ERSTBA writes from
	 * xhci_programCommandSpace + xhci_programEventRing land on a stable
	 * bridge translation and any inbound DMA the controller issues at
	 * R/S=1 still reaches DRAM. */
	{
		int re = bcm2711_pcie_resettleOutboundWindow();
		if (re != EOK && re != -ENODEV) {
			fprintf(stderr, "xhci: bridge re-settle after HCRST returned %d\n", re);
		}
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
	xhci->cmdEnqueue = 0u;

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

	/* 2026-05-28: match the 'X' diag rig's register-write order
	 * exactly (port/diag-udp.c):
	 *   1) CONFIG.MaxSlotsEn  — set BEFORE pointers, not after
	 *   2) DCBAAP_LO          — LO first (LO commits the 64-bit write)
	 *   3) DCBAAP_HI
	 *   4) CRCR_LO            — same LO-first ordering
	 *   5) CRCR_HI
	 * The rig has known-good behavior; xhci_init had a different order
	 * (HI first, CONFIG last) which xHCI spec says shouldn't matter but
	 * VL805 firmware quirks may not match the spec. */
	xhci_opWrite32(xhci, XHCI_REG_OP_CONFIG, config);
	xhci_opWrite32(xhci, XHCI_REG_OP_DCBAAP, dcbaapLo);
	xhci_opWrite32(xhci, XHCI_REG_OP_DCBAAP_HI, dcbaapHi);
	xhci_opWrite32(xhci, XHCI_REG_OP_CRCR, crcrLo);
	xhci_opWrite32(xhci, XHCI_REG_OP_CRCR_HI, crcrHi);

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


static int xhci_enterRunState(xhci_t *xhci)
{
	uint32_t usbcmd;
	uint32_t usbsts;
	int err;

	/* Idempotent when already running. cmdExec (and the addressing /
	 * configure-endpoint helpers) call this before every command; on the
	 * keepRunning path the controller never halts between commands, so
	 * re-asserting R/S here is pointless and actively harmful: it re-runs
	 * FIX-19's bridge outbound-window re-settle on every command (which the
	 * FIX-19 comment itself warns churns the inbound-DMA translation), adds a
	 * 10 ms settle per command, and floods the back-pressured UART with the
	 * re-settle's register dump — enough to stall the host lwip process so
	 * DHCP never completes. The `running` flag is set on the first successful
	 * R/S and cleared by xhci_enterHaltedState, so keepRunning==0 boards
	 * (which halt per command) still re-enter the run state each time. */
	if (xhci->running != 0u) {
		return EOK;
	}

	usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
	if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) != 0u) {
		fprintf(stderr, "xhci: controller error state before run\n");
		return -ENODEV;
	}

	/* Re-settle the BCM2711 inbound DMA window (RC_BAR2 / UBUS_BAR2)
	 * one more time right before R/S=1. The re-settle after HCRST
	 * covers reset, but between HCRST and here the scratchpad +
	 * event-ring DMA mmaps can re-churn the bridge inbound translation
	 * (this process also holds the bridge MMIO mapping). Re-asserting
	 * RC_BAR2 here is idempotent and keeps the inbound DMA path valid
	 * after the last mmap. */
	{
		int re = bcm2711_pcie_resettleOutboundWindow();
		if (re != EOK && re != -ENODEV) {
			fprintf(stderr, "xhci: bridge re-settle before R/S returned %d\n", re);
		}
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

	/* Pre-R/S settle: sleep 10 ms after the DSB and before the first
	 * R/S=1, giving the controller time to settle after the burst of
	 * ring-pointer register writes. */
	usleep(10000);

	/* HSE-on-R/S=1 soft retry (2026-05-24): on bridge state where the
	 * controller's first DMA fetch fails (USBSTS.HSE = 1, controller
	 * self-halts), clear HSE (write-1-to-clear), drop R/S, wait for
	 * HCH=1, and try R/S=1 again. Up to 3 attempts. If 2nd or 3rd
	 * attempt succeeds we recover from the intermittent HSE without
	 * the entire xhci_init re-running. Per xHCI 1.2 §5.4.2, HSE is
	 * RW1C in USBSTS. */
	{
		unsigned attempt;
		for (attempt = 0u; attempt < 10u; ++attempt) {
			usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
			/* Set R/S together with INTE and HSEE in a single USBCMD write,
			 * matching Linux xhci_run(). INTE/HSEE gate only the interrupt
			 * line (usb-hcd polls the event ring), so this is not required
			 * for correctness, but it keeps the run sequence aligned with
			 * the reference stacks. */
			xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD,
				usbcmd | XHCI_REG_OP_USBCMD_RS | XHCI_REG_OP_USBCMD_INTE | XHCI_REG_OP_USBCMD_HSEE);

			err = xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, 0u, XHCI_RUNSTOP_TIMEOUT_MS);
			if (err < 0) {
				fprintf(stderr, "xhci: run transition timeout (attempt %u)\n", attempt);
				return err;
			}

			usbsts = xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);
			if ((usbsts & (XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE)) == 0u) {
				xhci->running = 1u;
				return EOK;
			}

			/* Clear HSE (W1C), drop R/S, wait for halt, retry. */
			xhci_opWrite32(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HSE | XHCI_REG_OP_USBSTS_HCE);
			usbcmd = xhci_opRead32(xhci, XHCI_REG_OP_USBCMD);
			xhci_opWrite32(xhci, XHCI_REG_OP_USBCMD, usbcmd & ~XHCI_REG_OP_USBCMD_RS);
			(void)xhci_waitOpBits(xhci, XHCI_REG_OP_USBSTS, XHCI_REG_OP_USBSTS_HCH, XHCI_REG_OP_USBSTS_HCH, XHCI_RUNSTOP_TIMEOUT_MS);

			__asm__ volatile("dsb sy" ::: "memory");
		}
	}
	return -ENODEV;
}


static int xhci_enterHaltedState(xhci_t *xhci)
{
	uint32_t usbcmd;
	uint32_t usbsts;
	int err;

	/* Under keepRunning the controller must NEVER be halted (it runs continuously
	 * across all ops). Several error/timeout paths (cmdExec timeout, the ep0/
	 * address helpers) call this unconditionally; on the VL805 the halt never
	 * completes (HCH won't set) so it would spin XHCI_RUNSTOP_TIMEOUT_MS and, with
	 * the next command re-entering the run state, churn into an unbounded
	 * "halt transition timeout" loop (observed 106k lines = system hang). On a
	 * command timeout we want to leave the controller running and just return the
	 * error to the caller. keepRunning==0 boards keep the legacy halt behaviour. */
	if (xhci->keepRunning != 0u) {
		return EOK;
	}

	xhci->running = 0u; /* see xhci_enterRunState: re-enter on the next command */

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
	xhci->eventCycleState = 1u; /* consumer cycle state (CCS) */
	xhci->eventDeq = 0u;        /* shared dequeue starts at segment base */

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


/* Does event `ev` satisfy a waiter looking for (wantType, wantParam, wantEp)?
 * Command/port-status events match by parameter (the cmd-ring TRB PA). Transfer
 * events match by (slot, endpoint): a control transfer reports completion on
 * whichever stage TRB carries it — SUCCESS on the IOC'd Status TRB, an ERROR on
 * the failing Setup/Data TRB — so matching the exact Status-TRB PA would MISS
 * error completions. Slot+ep catches any completion for the addressed pipe and
 * surfaces its code; it is also the routing key multi-slot/HID needs. */
static int xhci_eventMatch(xhci_t *xhci, const xhci_trb_t *ev, uint32_t wantType, uint64_t wantParam, uint8_t wantSlot, uint32_t wantEp)
{
	uint32_t type = (ev->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;

	(void)xhci;
	if (type != wantType) {
		return 0;
	}
	if (wantType == XHCI_TRB_TYPE_EVENT_TRANSFER) {
		uint32_t evSlot = (ev->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >>
			XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;
		uint32_t evEp = (ev->control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >>
			XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
		(void)wantParam;
		return ((evSlot == (uint32_t)wantSlot) && (evEp == wantEp)) ? 1 : 0;
	}
	return (ev->parameter == wantParam) ? 1 : 0;
}


/* Is `ev` a completion that some waiter will be looking for (so it must be
 * parked in the stash rather than discarded)? Command + transfer completions
 * belong to a synchronous waiter; port-status changes are level state polled
 * elsewhere and can be safely dropped from the dequeue. */
static int xhci_eventStashable(const xhci_trb_t *ev)
{
	uint32_t type = (ev->control >> XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) & 0x3fu;

	return ((type == XHCI_TRB_TYPE_EVENT_CMD_COMPLETION) || (type == XHCI_TRB_TYPE_EVENT_TRANSFER)) ? 1 : 0;
}


/* Shared event-ring dispatcher. Two threads consume this single ring: the
 * synchronous enumeration path (xhci_cmdExec / ep0 control transfers) and the
 * roothub status thread (interrupt-IN completions, once an interrupt pipe
 * exists). They must NOT each walk the ring independently — that races the
 * shared dequeue/ERDP and lets one thread discard the other's completion.
 *
 * Instead every consumer calls here under xhci->eventLock. A pass first checks
 * the stash (events previously pulled off the ring for *this* waiter by the
 * other consumer), then walks the hardware ring from the shared dequeue: each
 * event is either returned to this caller (match), parked in the stash for the
 * other consumer (non-matching completion), or dropped (port-status). Every
 * consumed event advances the dequeue, writes ERDP (with EHB), and flips the
 * consumer cycle on wrap — exactly once, by whichever thread holds the lock.
 *
 * Match key: (wantType, wantParam, wantEp) — see xhci_eventMatch. Returns EOK
 * with *out filled, or -ETIMEDOUT if no matching event lands within timeoutMs.
 * timeoutMs==1 makes this a single non-blocking poll pass (used by the roothub
 * thread, which has its own poll loop). */
static int xhci_eventAwait(xhci_t *xhci, uint32_t wantType, uint64_t wantParam, uint8_t wantSlot, uint32_t wantEp, unsigned timeoutMs, xhci_trb_t *out)
{
	xhci_trb_t *ring = (xhci_trb_t *)xhci->eventRing;
	unsigned t;
	uint32_t i;

	for (t = timeoutMs; t > 0u; --t) {
		mutexLock(xhci->eventLock);

		/* The other consumer may already have pulled our event off the ring. */
		for (i = 0u; i < xhci->eventStashCount; ++i) {
			if (xhci_eventMatch(xhci, &xhci->eventStash[i], wantType, wantParam, wantSlot, wantEp) != 0) {
				uint32_t j;
				if (out != NULL) {
					*out = xhci->eventStash[i];
				}
				for (j = i + 1u; j < xhci->eventStashCount; ++j) {
					xhci->eventStash[j - 1u] = xhci->eventStash[j];
				}
				xhci->eventStashCount--;
				mutexUnlock(xhci->eventLock);
				return EOK;
			}
		}

		for (;;) {
			xhci_trb_t *cur = &ring[xhci->eventDeq];
			uint32_t ctrl = cur->control;
			uint32_t cbit = ((ctrl & XHCI_TRB_CONTROL_C) != 0u) ? 1u : 0u;
			int matched;

			if (cbit != ((xhci->eventCycleState != 0u) ? 1u : 0u)) {
				break; /* dequeue caught up to the producer */
			}

			matched = xhci_eventMatch(xhci, cur, wantType, wantParam, wantSlot, wantEp);
			if ((matched == 0) && (xhci_eventStashable(cur) != 0)) {
				if (xhci->eventStashCount < XHCI_EVENT_STASH_MAX) {
					/* completion for the other consumer — park, don't discard */
					xhci->eventStash[xhci->eventStashCount++] = *cur;
				}
				else {
					/* Unreachable in normal operation (the cross-consumer window
					 * is ~1-2 ms); if it prints, a waiter has stopped draining and
					 * its completions are accumulating — a real bug, not noise. */
					debug("xhci: event stash full, dropping a completion\n");
				}
			}
			if ((matched != 0) && (out != NULL)) {
				*out = *cur;
			}

			/* Advance the dequeue, wrap + flip consumer cycle, publish ERDP. */
			xhci->eventDeq++;
			if (xhci->eventDeq >= xhci->eventRingTrbs) {
				xhci->eventDeq = 0u;
				xhci->eventCycleState ^= 1u;
			}
			{
				uint64_t deqPhys = xhci->eventRingPhys + ((uint64_t)xhci->eventDeq * (uint64_t)sizeof(xhci_trb_t));
				xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, (uint32_t)(deqPhys >> 32));
				xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO,
					((uint32_t)(deqPhys & 0xFFFFFFFFu) & XHCI_REG_RT_IR_ERDP_LO__MASK) | XHCI_REG_RT_IR_ERDP_LO_EHB);
			}

			if (matched != 0) {
				mutexUnlock(xhci->eventLock);
				return EOK;
			}
			/* non-matching event consumed (stashed or dropped); keep draining */
		}

		mutexUnlock(xhci->eventLock);
		usleep(1000);
	}

	return -ETIMEDOUT;
}


/* Recover a wedged command ring after a command timed out.
 *
 * A command that the controller dequeues but never completes (observed on the
 * Pi4 VL805: AddressDevice/ep0 ops intermittently produce no completion event
 * while the port stays healthy) parks the controller's command dequeue on that
 * TRB. The producer (cmdEnqueue) then runs ahead, so naive "write another TRB +
 * ring the doorbell" retries are never reached — the controller is still stuck
 * on the original. xHCI 1.2 §4.6.1.2 / §5.4.5 give the escape: write CRCR.CA to
 * abort the running command, wait for CRR (Command Ring Running) to clear, then
 * the ring is Stopped and can be re-initialised. After this the caller may
 * re-issue the command and the controller will actually run it.
 *
 * Steps: (1) abort (CA=1, keep the ring pointer + RCS); (2) wait CRR=0;
 * (3) drain the event ring of the abort/stop completion so it can't be
 * mis-matched later; (4) re-init the command-ring producer (cmdEnqueue/cycle +
 * Link TRB) and clear crcrPublished so the next cmdExec re-publishes CRCR=base,
 * which resets the controller's dequeue to index 0 in lock-step. */
static int xhci_cmdRingRecover(xhci_t *xhci)
{
	uint32_t crcr;
	int err;

	crcr = xhci_opRead32(xhci, XHCI_REG_OP_CRCR);
	if ((crcr & XHCI_REG_OP_CRCR_CRR) != 0u) {
		xhci_opWrite32(xhci, XHCI_REG_OP_CRCR_HI, (uint32_t)(xhci->cmdRingPhys >> 32));
		xhci_opWrite32(xhci, XHCI_REG_OP_CRCR,
			((uint32_t)(xhci->cmdRingPhys & XHCI_REG_OP_CRCR_CR_PTR_LO__MASK)) |
				XHCI_REG_OP_CRCR_RCS | XHCI_REG_OP_CRCR_CA);
		(void)xhci_opRead32(xhci, XHCI_REG_OP_USBSTS); /* flush the posted write */
		err = xhci_waitOpBits(xhci, XHCI_REG_OP_CRCR, XHCI_REG_OP_CRCR_CRR, 0u, 100u);
		if (err < 0) {
			debug("xhci_cmdRingRecover: CRR did not clear after abort\n");
		}
	}

	/* Drain every event the controller has posted (incl. the abort/stop
	 * completion) so a stale completion can't satisfy the next waiter. Walk
	 * the hardware ring from the shared dequeue, discarding all valid events. */
	{
		xhci_trb_t *ring = (xhci_trb_t *)xhci->eventRing;
		unsigned guard = 0u;
		mutexLock(xhci->eventLock);
		for (; guard < xhci->eventRingTrbs; ++guard) {
			xhci_trb_t *cur = &ring[xhci->eventDeq];
			uint32_t cbit = ((cur->control & XHCI_TRB_CONTROL_C) != 0u) ? 1u : 0u;
			uint64_t deqPhys;
			if (cbit != ((xhci->eventCycleState != 0u) ? 1u : 0u)) {
				break; /* caught up to the producer */
			}
			xhci->eventDeq++;
			if (xhci->eventDeq >= xhci->eventRingTrbs) {
				xhci->eventDeq = 0u;
				xhci->eventCycleState ^= 1u;
			}
			deqPhys = xhci->eventRingPhys + ((uint64_t)xhci->eventDeq * (uint64_t)sizeof(xhci_trb_t));
			xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, (uint32_t)(deqPhys >> 32));
			xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO,
				((uint32_t)(deqPhys & 0xFFFFFFFFu) & XHCI_REG_RT_IR_ERDP_LO__MASK) | XHCI_REG_RT_IR_ERDP_LO_EHB);
		}
		xhci->eventStashCount = 0u; /* drop any cross-consumer stash too */
		mutexUnlock(xhci->eventLock);
	}

	(void)xhci_initCommandRing(xhci);
	xhci->crcrPublished = 0u;
	return EOK;
}


static int xhci_cmdExec(xhci_t *xhci, uint64_t parameter, uint32_t status, uint32_t control, uint8_t *slotId)
{
	xhci_trb_t *cmd;
	xhci_trb_t ev;
	uint64_t cmdPhys;
	uint32_t completion;
	int err;

	/* Persistent command-ring producer. The controller's internal command
	 * dequeue advances past each consumed command and is NOT rewound by a
	 * CRCR rewrite once the ring is running (the VL805 ignores it — see the
	 * one-shot republish below), so each command must be enqueued at the
	 * running producer position with the running cycle bit. Writing every
	 * command at index 0 (the prior behaviour) was only seen for the first
	 * command after init; the second hung (cmd-completion never landed).
	 * Wrap + toggle the cycle at the trailing Link TRB. */
	if (xhci->cmdEnqueue >= (xhci->cmdRingCount - 1u)) {
		xhci_trb_t *link = &xhci->cmdRingTrbs[xhci->cmdRingCount - 1u];
		link->control = (link->control & ~XHCI_TRB_CONTROL_C) |
			(xhci->cmdCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u);
		xhci->cmdEnqueue = 0u;
		xhci->cmdCycleState ^= 1u;
	}

	cmd = &xhci->cmdRingTrbs[xhci->cmdEnqueue];
	memset(cmd, 0, sizeof(*cmd));
	cmd->parameter = parameter;
	cmd->status = status;
	cmd->control = (control & ~XHCI_TRB_CONTROL_C) | (xhci->cmdCycleState != 0u ? XHCI_TRB_CONTROL_C : 0u);
	cmdPhys = xhci->cmdRingPhys + ((uint64_t)xhci->cmdEnqueue * (uint64_t)XHCI_TRB_SIZE);
	xhci->cmdEnqueue++;

	/* Completions are consumed via xhci_eventAwait() below (shared
	 * persistent dequeue, cycle-bit detection). The old idx 0-3 "sentinel
	 * paint" was removed in Stage 2a — it clobbered events the controller
	 * had already posted at idx 0, hiding the real completion. */

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

	/* Re-publish CRCR pointer ONCE, right before the first command's
	 * doorbell. Spec § 5.4.5 allows CRCR writes only when the Command Ring
	 * is Stopped (CRR=0), which holds before the first command. Earlier
	 * diagnostics showed the controller losing the cmd-ring pointer between
	 * programCommandSpace and the first doorbell (bridge-side MMIO churn
	 * during init), so re-establishing the base pointer here lets the first
	 * command run.
	 *
	 * It MUST be one-shot: once the ring is running the command producer is
	 * persistent (xhci->cmdEnqueue), so a later CRCR=base rewrite would —
	 * if honoured — rewind the controller's dequeue to index 0 and desync it
	 * from the producer mid-stream. (Empirically the VL805 ignores the
	 * rewrite once running anyway — that was bug #3 — but relying on that
	 * would be fragile.) */
	if (xhci->crcrPublished == 0u) {
		uint64_t cmdRingPhys = xhci->cmdRingPhys;
		uint32_t crcrLo = (uint32_t)(cmdRingPhys & XHCI_REG_OP_CRCR_CR_PTR_LO__MASK) | XHCI_REG_OP_CRCR_RCS;
		uint32_t crcrHi = (uint32_t)(cmdRingPhys >> 32);
		xhci_opWrite32(xhci, XHCI_REG_OP_CRCR_HI, crcrHi);
		xhci_opWrite32(xhci, XHCI_REG_OP_CRCR, crcrLo);
		(void)xhci_opRead32(xhci, XHCI_REG_OP_USBSTS); /* flush */
		xhci->crcrPublished = 1u;
	}

	xhci_dbWrite32(xhci, 0u, 0u);
	/* Flush the posted doorbell write: PCIe spec allows MMIO writes
	 * to be posted (queued in the bridge's write buffer). A subsequent
	 * MMIO read to the SAME device cannot bypass an earlier posted
	 * write, so reading USBSTS forces the doorbell write to complete
	 * its journey to the controller. Without this the doorbell may
	 * sit in the bridge buffer for some indeterminate time, during
	 * which the controller doesn't fetch from the cmd ring. */
	(void)xhci_opRead32(xhci, XHCI_REG_OP_USBSTS);

	/* Consume the event ring via the shared dequeue until our command
	 * completion (type-33, parameter == cmdPhys) lands, draining any
	 * port-status or stale events queued ahead of it. */
	/* CMD completions are matched by parameter (cmdPhys); the slot field is
	 * ignored for this event type, so wantSlot is irrelevant — pass 0. */
	err = xhci_eventAwait(xhci, XHCI_TRB_TYPE_EVENT_CMD_COMPLETION, cmdPhys, 0u, 0u, XHCI_CMD_TIMEOUT_MS, &ev);

	if (err < 0) {
		(void)xhci_enterHaltedState(xhci);
		memset(cmd, 0, sizeof(*cmd));
		/* The controller dequeued this command but never completed it, wedging the
		 * command ring (its dequeue is parked on this TRB; the producer runs ahead,
		 * so a plain re-enqueue is never reached). Abort + re-init the ring so the
		 * caller's retry (HUB_ENUM_RETRIES) actually executes on the controller.
		 * This abort-and-recover reliably rescues the wedged ring in practice. */
		(void)xhci_cmdRingRecover(xhci);
		return -ETIMEDOUT;
	}

	/* TRB type + parameter were already matched by xhci_eventAwait. */
	completion = (ev.status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;

	if (slotId != NULL) {
		*slotId = (ev.control & XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__MASK) >>
			XHCI_CMD_COMPLETION_EVENT_TRB_CONTROL_SLOTID__SHIFT;
	}

	/* Leave the controller running between commands (keepRunning). The rig
	 * hands off a RUNNING controller and keeps R/S=1 across EnableSlot ->
	 * AddressDevice; our former halt-per-command pattern dropped R/S after
	 * each command and re-ran it before the next, and a just-unhalted VL805
	 * fails the next USB transaction — AddressDevice(BSR=0) returned a
	 * Context State Error. Only the legacy reset-based init path (keepRunning
	 * == 0) still halts here. */
	if (xhci->keepRunning == 0u) {
		err = xhci_enterHaltedState(xhci);
		if (err < 0) {
			memset(cmd, 0, sizeof(*cmd));
			return err;
		}
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


static int xhci_cmdAddressDevice(xhci_t *xhci, xhci_slot_t *slot, int setAddress)
{
	uint32_t control;

	if ((slot->slotId == 0u) || (xhci->inputCtxPhys == 0u)) {
		return -EINVAL;
	}

	if ((!xhci->ac64) && ((xhci->inputCtxPhys >> 32) != 0u)) {
		fprintf(stderr, "xhci: input context above 32-bit address space\n");
		return -ENODEV;
	}

	control = (XHCI_TRB_TYPE_CMD_ADDRESS_DEVICE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		((uint32_t)slot->slotId << XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_SLOTID__SHIFT);
	if (setAddress == 0) {
		control |= XHCI_CMD_TRB_ADDRESS_DEVICE_CONTROL_BSR;
	}

	return xhci_cmdExec(xhci, xhci->inputCtxPhys, 0u, control, NULL);
}


static int xhci_cmdConfigureEndpoint(xhci_t *xhci, xhci_slot_t *slot, int deconfigure)
{
	uint32_t control;

	if ((slot->slotId == 0u) || (xhci->inputCtxPhys == 0u)) {
		return -EINVAL;
	}

	control = (XHCI_TRB_TYPE_CMD_CONFIGURE_ENDPOINT << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
		((uint32_t)slot->slotId << XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_SLOTID__SHIFT);
	if (deconfigure != 0) {
		control |= XHCI_CMD_TRB_CONFIGURE_ENDPOINT_CONTROL_DC;
	}

	return xhci_cmdExec(xhci, xhci->inputCtxPhys, 0u, control, NULL);
}


static int xhci_allocSlotSpace(xhci_t *xhci, xhci_slot_t *slot)
{
	uint64_t *dcbaa;
	size_t dcbaaOffs;

	if ((slot->slotId == 0u) || (slot->slotId > xhci->nslots)) {
		fprintf(stderr, "xhci: invalid slot id for slot space %u\n", slot->slotId);
		return -EINVAL;
	}

	xhci->devCtxSize = xhci->contextSize * (1u + XHCI_MAX_ENDPOINTS);
	xhci->inputCtxSize = xhci->contextSize * (XHCI_CONTEXT_INPUT + 1u + XHCI_MAX_ENDPOINTS);
	xhci->ep0RingSize = XHCI_TRANSFER_RING_SIZE;

	slot->devCtx = usb_allocAligned(xhci->devCtxSize, XHCI_CONTEXT_ALIGN);
	if (slot->devCtx == NULL) {
		fprintf(stderr, "xhci: failed to allocate device context\n");
		return -ENOMEM;
	}

	xhci->inputCtx = usb_allocAligned(xhci->inputCtxSize, XHCI_CONTEXT_ALIGN);
	if (xhci->inputCtx == NULL) {
		fprintf(stderr, "xhci: failed to allocate input context\n");
		return -ENOMEM;
	}

	slot->ep0Ring = usb_allocAligned(xhci->ep0RingSize, XHCI_TRANSFER_RING_ALIGN);
	if (slot->ep0Ring == NULL) {
		fprintf(stderr, "xhci: failed to allocate ep0 ring\n");
		return -ENOMEM;
	}

	memset(slot->devCtx, 0, xhci->devCtxSize);
	memset(xhci->inputCtx, 0, xhci->inputCtxSize);
	memset(slot->ep0Ring, 0, xhci->ep0RingSize);

	slot->devCtxPhys = va2pa(slot->devCtx);
	xhci->inputCtxPhys = va2pa(xhci->inputCtx);
	slot->ep0RingPhys = va2pa(slot->ep0Ring);

	if (((slot->devCtxPhys & (XHCI_CONTEXT_ALIGN - 1u)) != 0u) ||
		((xhci->inputCtxPhys & (XHCI_CONTEXT_ALIGN - 1u)) != 0u) ||
		((slot->ep0RingPhys & (XHCI_TRANSFER_RING_ALIGN - 1u)) != 0u)) {
		fprintf(stderr, "xhci: invalid slot-space alignment\n");
		return -ENODEV;
	}

	dcbaaOffs = slot->slotId * XHCI_DCBAA_ENTRY_SIZE;
	if ((dcbaaOffs + XHCI_DCBAA_ENTRY_SIZE) > xhci->dcbaaSize) {
		fprintf(stderr, "xhci: slot id %u exceeds dcbaa size\n", slot->slotId);
		return -ENODEV;
	}

	dcbaa = (uint64_t *)xhci->dcbaa;
	dcbaa[slot->slotId] = slot->devCtxPhys;

	if (dcbaa[slot->slotId] != slot->devCtxPhys) {
		fprintf(stderr, "xhci: failed to bind dcbaa slot entry\n");
		return -ENODEV;
	}

	return EOK;
}


static int xhci_initEp0Ring(xhci_t *xhci, xhci_slot_t *slot)
{
	xhci_trb_t *ring;
	xhci_trb_t *link;

	slot->ep0RingCount = xhci->ep0RingSize / XHCI_TRB_SIZE;
	if (slot->ep0RingCount <= 1u) {
		fprintf(stderr, "xhci: ep0 ring too small\n");
		return -ENODEV;
	}

	memset(slot->ep0Ring, 0, xhci->ep0RingSize);
	slot->ep0CycleState = 1u;
	slot->ep0Enqueue = 0u;

	ring = (xhci_trb_t *)slot->ep0Ring;
	link = &ring[slot->ep0RingCount - 1u];
	link->parameter = slot->ep0RingPhys;
	link->status = 0u;
	link->control = XHCI_TRB_CONTROL_C |
		XHCI_LINK_TRB_CONTROL_TC |
		(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);

	if ((link->parameter != slot->ep0RingPhys) ||
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


static int xhci_prepareAddressContext(xhci_t *xhci, xhci_slot_t *target, usb_dev_t *dev)
{
	xhci_input_ctx_t *input;
	xhci_slot_ctx_t *slot;
	xhci_ep_ctx_t *ep0;
	unsigned int psi;
	uint16_t maxPacket;
	uint32_t word0;
	uint32_t word1;
	uint32_t rootHubPort;
	int behindHub;

	/* Two topologies are handled:
	 *   - a device on a root-hub port (dev->hub is the root hub, dev->hub->hub
	 *     == NULL): root hub port = dev->port, no route string, no TT.
	 *   - a device behind a non-root hub (dev->hub->hub != NULL), e.g. the
	 *     low-speed keyboard behind the external VIA hub: it needs a route
	 *     string + TT (split transactions) and the ROOT-hub port of the chain. */
	if ((dev == NULL) || (dev->hub == NULL)) {
		return -ENOSYS;
	}
	behindHub = (dev->hub->hub != NULL) ? 1 : 0;

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

	word0 = (psi << XHCI_SLOT_CTX_SPEED__SHIFT) |
		(1u << XHCI_SLOT_CTX_CONTEXT_ENTRIES__SHIFT);

	if (behindHub == 0) {
		/* Root-port device (the external hub). Root hub port = dev->port; no
		 * route string, no TT. Behaviour identical to the pre-Step-2 path. */
		rootHubPort = (uint32_t)dev->port & 0xffu;
		word1 = rootHubPort << XHCI_SLOT_CTX_ROOT_HUB_PORT__SHIFT;
		slot->ttHubSlot_ttPort_ttt_intrTarget = 0u;
	}
	else {
		/* Device behind a non-root hub. Route string is the downstream-port
		 * path below the root hub; the root hub port lives in its own field
		 * (word1) and is NOT part of the route string.
		 *
		 * TODO(multi-tier): for the single-tier case (keyboard one level below
		 * the root hub) the route string is a single nibble = the device's port
		 * on its parent hub (dev->port). A deeper topology would need the full
		 * per-tier nibble path; that is (dev->locationID >> 8) for tier>=2, but
		 * only the single-tier VIA-hub case is exercised today. */
		uint32_t routeString = (uint32_t)dev->port & 0xfu;

		/* Root hub port of the whole chain = the parent hub's root-hub port.
		 * dev->hub is the VIA hub, which enumerated on a root port; its own
		 * ->port is that root port (1). NOT dev->port (the keyboard's port on
		 * the hub). */
		rootHubPort = (uint32_t)dev->hub->port & 0xffu;

		word0 |= routeString & XHCI_SLOT_CTX_ROUTE_STRING__MASK;
		/* MTT=0: the VIA hub (2109:3431) is single-TT. */
		word1 = rootHubPort << XHCI_SLOT_CTX_ROOT_HUB_PORT__SHIFT;

		/* TT for a low/full-speed device behind a high-speed hub: the TT hub is
		 * the parent hub's xHCI slot. TODO(hub-slot-map): the only non-root hub
		 * today is the primary slot (slots[0], slotId 1); map dev->hub -> its
		 * slot generally once a second hub can appear. TT Port = device's port
		 * on the hub; TT Think Time = 0. */
		slot->ttHubSlot_ttPort_ttt_intrTarget =
			((uint32_t)xhci->slots[0].slotId << XHCI_SLOT_CTX_TT_HUB_SLOT_ID__SHIFT) |
			(((uint32_t)dev->port & 0xffu) << XHCI_SLOT_CTX_TT_PORT_NUMBER__SHIFT) |
			(0u << XHCI_SLOT_CTX_TT_THINK_TIME__SHIFT);
	}

	slot->routeString_speed_mtt_hub_entries = word0;
	slot->maxExitLatency_rootHubPort_ports = word1;

	ep0->cerr_type_burst_packet =
		(3u << XHCI_EP_CTX_CERR__SHIFT) |
		(XHCI_EP_CTX_TYPE_CONTROL << XHCI_EP_CTX_TYPE__SHIFT) |
		((uint32_t)maxPacket << XHCI_EP_CTX_MAX_PACKET__SHIFT);
	ep0->trDequeuePtr = target->ep0RingPhys | XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS;
	ep0->averageTrbLen_maxEsitPayload = 8u;

	target->maxPacket = maxPacket;

	if ((input->control.addContextFlags != XHCI_INPUT_CTRL_CTX_ADD_A0_A1) ||
		(slot->maxExitLatency_rootHubPort_ports != word1) ||
		(ep0->trDequeuePtr != (target->ep0RingPhys | XHCI_EP_CTX_TR_DEQUEUE_PTR_DCS))) {
		fprintf(stderr, "xhci: invalid address context preparation\n");
		return -ENODEV;
	}

	return EOK;
}


/* PITFALL (xHCI 1.2 §4.6.6): for the controller to route split transactions to
 * a low/full-speed device behind a high-speed hub, the HUB's OWN slot context
 * must declare Hub=1, Number of Ports and TT Think Time. The hub was
 * Address-Device'd before its descriptor (port count) was known, so these are
 * unset. They are evaluated only by Configure Endpoint, NOT Evaluate Context
 * (Evaluate Context touches only Max Exit Latency / Interrupter Target / ep0
 * Max Packet Size). We copy the hub's current slot context out of its device
 * context, set only the A0 (slot) add flag, OR in the hub fields, and issue
 * Configure Endpoint so the existing Speed/RootHubPort/ContextEntries survive.
 * One-shot per hub slot (slot->hubFixedUp). */
static int xhci_cmdHubSlotFixup(xhci_t *xhci, xhci_slot_t *hubSlot, usb_dev_t *hubDev)
{
	xhci_input_ctx_t *input;
	xhci_dev_ctx_t *devCtx;
	xhci_slot_ctx_t *slot;
	uint32_t nports;
	int err;

	if (hubSlot->hubFixedUp != 0u) {
		return EOK;
	}

	nports = (uint32_t)hubDev->nports;
	if (nports == 0u) {
		/* Hub descriptor not parsed yet; nothing useful to program. */
		return -EAGAIN;
	}

	memset(xhci->inputCtx, 0, xhci->inputCtxSize);
	input = (xhci_input_ctx_t *)xhci->inputCtx;
	slot = &input->device.slot;

	/* A0 (slot context) only — Configure Endpoint with no endpoint add/drop. */
	input->control.addContextFlags = 0x1u;

	/* Preserve the hub's live slot context (Speed/RootHubPort/ContextEntries). */
	devCtx = (xhci_dev_ctx_t *)hubSlot->devCtx;
	slot->routeString_speed_mtt_hub_entries = devCtx->slot.routeString_speed_mtt_hub_entries;
	slot->maxExitLatency_rootHubPort_ports = devCtx->slot.maxExitLatency_rootHubPort_ports;
	slot->ttHubSlot_ttPort_ttt_intrTarget = devCtx->slot.ttHubSlot_ttPort_ttt_intrTarget;

	/* Hub=1, Number of Ports; TT Think Time = 0, MTT = 0 (VIA hub is single-TT). */
	slot->routeString_speed_mtt_hub_entries |= XHCI_SLOT_CTX_HUB;
	slot->maxExitLatency_rootHubPort_ports =
		(slot->maxExitLatency_rootHubPort_ports & ~(0xffu << XHCI_SLOT_CTX_NUMBER_OF_PORTS__SHIFT)) |
		((nports & 0xffu) << XHCI_SLOT_CTX_NUMBER_OF_PORTS__SHIFT);

	err = xhci_cmdConfigureEndpoint(xhci, hubSlot, 0);
	if (err < 0) {
		fprintf(stderr, "xhci: hub slot-ctx fixup (Configure Endpoint) failed rc=%d\n", err);
		return err;
	}

	hubSlot->hubFixedUp = 1u;
	return EOK;
}


/* Find the slot table entry already bound to `dev`, or NULL. The primary slot
 * (slots[0]) is reserved for the root-port device (the external hub) and is
 * matched by topology in xhci_slotForDev, not by this exact-pointer lookup. */
static xhci_slot_t *xhci_findSlotForDev(xhci_t *xhci, usb_dev_t *dev)
{
	unsigned i;

	for (i = 1u; i < (sizeof(xhci->slots) / sizeof(xhci->slots[0])); ++i) {
		if ((xhci->slots[i].slotId != 0u) && (xhci->slots[i].dev == dev)) {
			return &xhci->slots[i];
		}
	}

	return NULL;
}


/* Allocate, enable and Address-Device a fresh xHCI slot for a device sitting
 * behind a non-root hub (the low-speed keyboard behind the VIA hub). Returns
 * the new slot, or NULL on failure (errno-style rc via *err). The framework's
 * subsequent SET_ADDRESS is acknowledged without re-issuing Address Device,
 * exactly as the primary (slots[0]) path does. */
static xhci_slot_t *xhci_allocSlotForDev(xhci_t *xhci, usb_dev_t *dev, int *err)
{
	xhci_slot_t *slot = NULL;
	uint8_t slotId = 0u;
	unsigned i;

	*err = EOK;

	for (i = 1u; i < (sizeof(xhci->slots) / sizeof(xhci->slots[0])); ++i) {
		if (xhci->slots[i].slotId == 0u) {
			slot = &xhci->slots[i];
			break;
		}
	}

	if (slot == NULL) {
		fprintf(stderr, "xhci: no free slot for device behind hub\n");
		*err = -ENOMEM;
		return NULL;
	}

	*err = xhci_cmdEnableSlot(xhci, &slotId);
	if (*err < 0) {
		return NULL;
	}

	slot->slotId = slotId;
	slot->dev = dev;

	*err = xhci_allocSlotSpace(xhci, slot);
	if (*err < 0) {
		return NULL;
	}

	*err = xhci_initEp0Ring(xhci, slot);
	if (*err < 0) {
		return NULL;
	}

	/* Ensure the parent hub's slot ctx declares Hub=1/NumPorts/TTT before the
	 * keyboard slot is addressed, so the controller can route split transactions
	 * down to it. The framework drives the hub's class-descriptor read through
	 * this HCD, so nports is known by the time the keyboard first appears. */
	*err = xhci_cmdHubSlotFixup(xhci, &xhci->slots[0], dev->hub);
	if (*err < 0) {
		return NULL;
	}

	*err = xhci_prepareAddressContext(xhci, slot, dev);
	if (*err < 0) {
		return NULL;
	}

	/* Two-step addressing (BSR=1 context-only, then BSR=0 assign) — see the
	 * root-port path in xhci_handlePipeTransfer for the rationale (#129). */
	*err = xhci_cmdAddressDevice(xhci, slot, 0);
	if (*err < 0) {
		return NULL;
	}

	*err = xhci_cmdAddressDevice(xhci, slot, 1);
	if (*err < 0) {
		return NULL;
	}

	slot->addressed = 1u;
	return slot;
}


/* Map a device to the slot that drives it. Root-port devices (the external hub:
 * dev->hub->hub == NULL) use the primary slot, slots[0]. Devices behind a
 * non-root hub use their dedicated slot (allocated lazily by
 * xhci_allocSlotForDev). Returns slots[0] as the safe default so the existing
 * single-slot path is unchanged when no per-dev slot exists yet. */
static xhci_slot_t *xhci_slotForDev(xhci_t *xhci, usb_dev_t *dev)
{
	xhci_slot_t *slot;

	if ((dev->hub != NULL) && (dev->hub->hub != NULL)) {
		slot = xhci_findSlotForDev(xhci, dev);
		if (slot != NULL) {
			return slot;
		}
	}

	return &xhci->slots[0];
}


static int xhci_initInterruptInPipe(xhci_t *xhci, usb_pipe_t *pipe)
{
	xhci_input_ctx_t *input;
	xhci_ep_ctx_t *epctx;
	xhci_trb_t *ring;
	xhci_trb_t *link;
	xhci_pipePriv_t *priv;
	xhci_slot_t *slot;
	uint8_t endpointId;
	uint32_t interval;
	int err;

	if ((xhci == NULL) || (pipe == NULL)) {
		return -EINVAL;
	}

	if (pipe->hcdpriv != NULL) {
		return 0;
	}

	/* Require an ADDRESSED device that hangs off a hub, but do NOT require
	 * dev->address == slotId (the framework address is its own bookkeeping; xHCI
	 * routes by slot via the doorbell) NOR that the parent hub be the root hub.
	 * A device behind a non-root hub (the low-speed keyboard behind the external
	 * hub) has its own slot (xhci_slotForDev) with route string + TT already set
	 * by Address Device, so its interrupt-IN endpoint is configured on that slot
	 * the same way the hub's status endpoint is on slots[0]. */
	if ((pipe->type != usb_transfer_interrupt) || (pipe->dir != usb_dir_in) ||
		(pipe->dev->hub == NULL) ||
		(pipe->dev->address == 0)) {
		return -EINVAL;
	}

	endpointId = xhci_pipeEndpointId(pipe);
	if (endpointId == 0u) {
		return -EINVAL;
	}

	/* The owning slot drives the Configure Endpoint command and the doorbell,
	 * NOT a hardcoded slots[0]: the external hub maps to slots[0] (slotId 1),
	 * a device behind a non-root hub to its own slot. Per-slot interruptPriv
	 * tracking lets a second interrupt pipe on a different slot coexist with
	 * the hub's without clobbering it. */
	slot = xhci_slotForDev(xhci, pipe->dev);

	priv = calloc(1, sizeof(*priv));
	if (priv == NULL) {
		return -ENOMEM;
	}

	priv->ringSize = XHCI_TRANSFER_RING_SIZE;
	priv->slotId = slot->slotId;
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
	priv->enqueue = 0u;
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

	err = xhci_cmdConfigureEndpoint(xhci, slot, 0);
	if (err < 0) {
		usb_freeAligned(priv->ring, priv->ringSize);
		free(priv);
		return err;
	}

	pipe->hcdpriv = priv;
	slot->interruptPriv = priv;
	fprintf(stderr, "xhci: interrupt-IN pipe ready slot=%u ep=%u maxpkt=%u\n",
		(unsigned)slot->slotId, (unsigned)endpointId, (unsigned)pipe->maxPacketLen);
	return 0;
}


static int xhci_submitInterruptIn(xhci_t *xhci, usb_transfer_t *t, usb_pipe_t *pipe)
{
	xhci_pipePriv_t *priv;
	xhci_trb_t *ring;
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

	/* Circular producer. The previous code memset the whole ring and rewrote
	 * ring[0] with a FIXED cycle bit on every submit, leaving pendingTrbPhys at
	 * the ring base — so only the FIRST interrupt report was ever delivered:
	 * after the controller consumed ring[0] and followed the Link TRB (TC) its
	 * dequeue cycle toggled to 0, but each resubmit re-armed ring[0] with cycle 1,
	 * which the controller no longer owns, so it stopped. Instead, place the
	 * Normal TRB at the running enqueue index with the producer cycle, advance,
	 * and at the trailing Link TRB wrap to 0 and toggle the cycle (TC toggles the
	 * controller's dequeue cycle in step). Only one transfer is ever in flight
	 * (pendingTransfer gates it), so producer and consumer advance in lock-step. */
	{
		uint32_t idx = priv->enqueue;
		uint32_t pcs = (priv->cycleState != 0u) ? XHCI_TRB_CONTROL_C : 0u;

		ring[idx].parameter = va2pa(t->buffer);
		ring[idx].status = (uint32_t)t->size;
		ring[idx].control = pcs |
			XHCI_TRANSFER_TRB_CONTROL_IOC |
			XHCI_TRANSFER_TRB_CONTROL_ISP |
			(XHCI_TRB_TYPE_NORMAL << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);
		priv->pendingTrbPhys = priv->ringPhys + (uint64_t)idx * (uint64_t)XHCI_TRB_SIZE;

		priv->enqueue++;
		if (priv->enqueue >= (priv->ringCount - 1u)) {
			link = &ring[priv->ringCount - 1u];
			link->parameter = priv->ringPhys;
			link->status = 0u;
			link->control = pcs |
				XHCI_LINK_TRB_CONTROL_TC |
				(XHCI_TRB_TYPE_LINK << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT);
			priv->enqueue = 0u;
			priv->cycleState ^= 1u;
		}
	}

	/* Do NOT touch the event ring / ERDP here: the shared dispatcher
	 * (xhci_eventAwait, consumed by the roothub status thread) owns the event
	 * dequeue and ERDP. The old code zeroed event-ring slot 0 and reset ERDP to
	 * the ring base on every interrupt submit, desyncing the persistent dequeue
	 * and corrupting in-flight control completions on the shared ep. This submit
	 * only queues the Normal TRB on the interrupt ring and rings the doorbell;
	 * the roothub thread reaps the completion via the dispatcher. */
	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	priv->pendingTransfer = t;

	xhci_dbWrite32(xhci, (uintptr_t)priv->slotId * sizeof(uint32_t), priv->endpointId);

	return 0;
}


/* Reserve `count` contiguous TRBs ahead of the trailing Link TRB so a single
 * control transfer's stages never straddle the ring wrap. If they would,
 * publish the Link TRB now (with the running producer cycle) and wrap the
 * producer to the segment base, toggling the producer cycle. */
static void xhci_ep0Reserve(xhci_t *xhci, xhci_slot_t *slot, uint32_t count)
{
	xhci_trb_t *ring = (xhci_trb_t *)slot->ep0Ring;
	xhci_trb_t *link;

	(void)xhci;
	if ((slot->ep0Enqueue + count) <= (slot->ep0RingCount - 1u)) {
		return;
	}

	link = &ring[slot->ep0RingCount - 1u];
	link->control = (link->control & ~XHCI_TRB_CONTROL_C) |
		(slot->ep0CycleState != 0u ? XHCI_TRB_CONTROL_C : 0u);
	slot->ep0Enqueue = 0u;
	slot->ep0CycleState ^= 1u;
}


/* Enqueue one TRB at the current ep0 producer position with the running
 * producer cycle bit and return its physical address (for event matching).
 *
 * The controller's internal ep0 dequeue pointer is rewound to the ring base
 * only by Address Device (which reloads the EP0 context trDequeuePtr). Between
 * transfers it advances past each consumed TRB and parks there, so the driver
 * must enqueue at the SAME running position rather than rewriting index 0 each
 * time — otherwise the controller, parked past the previous transfer, never
 * sees the new work. Callers reserve room via xhci_ep0Reserve so a transfer
 * cannot straddle the trailing Link TRB; this advances only within the
 * segment. */
static uint64_t xhci_ep0Push(xhci_t *xhci, xhci_slot_t *slot, uint64_t parameter, uint32_t status, uint32_t control)
{
	xhci_trb_t *ring = (xhci_trb_t *)slot->ep0Ring;
	uint32_t idx = slot->ep0Enqueue;
	uint64_t trbPhys = slot->ep0RingPhys + ((uint64_t)idx * (uint64_t)XHCI_TRB_SIZE);

	(void)xhci;
	ring[idx].parameter = parameter;
	ring[idx].status = status;
	ring[idx].control = (control & ~XHCI_TRB_CONTROL_C) |
		(slot->ep0CycleState != 0u ? XHCI_TRB_CONTROL_C : 0u);

	slot->ep0Enqueue = idx + 1u;
	return trbPhys;
}


static int xhci_ep0ControlRead(xhci_t *xhci, xhci_slot_t *slot, usb_transfer_t *t)
{
	xhci_trb_t ev;
	uint64_t statusTrbPhys;
	uint64_t dataTrbPhys;
	uint32_t completion;
	uint32_t endpointId;
	uint32_t slotId;
	uint32_t residual;
	int transferred;
	int err;

	if ((xhci == NULL) || (t == NULL) || (t->setup == NULL) || (t->buffer == NULL) || (t->size == 0u)) {
		return -EINVAL;
	}

	/* Setup + Data(IN) + Status(OUT) at the persistent producer position. */
	xhci_ep0Reserve(xhci, slot, 3u);

	(void)xhci_ep0Push(xhci, slot,
		(uint64_t)t->setup->bmRequestType |
			((uint64_t)t->setup->bRequest << 8) |
			((uint64_t)t->setup->wValue << 16) |
			((uint64_t)t->setup->wIndex << 32) |
			((uint64_t)t->setup->wLength << 48),
		sizeof(usb_setup_packet_t),
		(XHCI_TRB_TYPE_SETUP_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
			(XHCI_TRANSFER_TRB_CONTROL_TRT_IN << XHCI_TRANSFER_TRB_CONTROL_TRT__SHIFT) |
			XHCI_TRANSFER_TRB_CONTROL_IDT);

	dataTrbPhys = xhci_ep0Push(xhci, slot, va2pa(t->buffer), (uint32_t)t->size,
		(XHCI_TRB_TYPE_DATA_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
			XHCI_TRANSFER_TRB_CONTROL_DIR_IN |
			XHCI_TRANSFER_TRB_CONTROL_ISP);

	statusTrbPhys = xhci_ep0Push(xhci, slot, 0u, 0u,
		(XHCI_TRB_TYPE_STATUS_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
			XHCI_TRANSFER_TRB_CONTROL_IOC);

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	xhci_dbWrite32(xhci, slot->slotId * sizeof(uint32_t), 1u);

	/* Consume EVERY event this TD produces, in ring order, until the Status-TRB
	 * event (end of TD) or an error. A short Data stage emits a SHORT_PACKET
	 * event (ISP) IN ADDITION to the Status-stage IOC event, so a control read
	 * can yield two events; consuming only one per call leaves the other to be
	 * mis-matched by the NEXT transfer (an off-by-one "drift" that returns a
	 * stale completion before the current data lands). Looping until our own
	 * Status TRB drains both, and also drains any stale events a prior transfer
	 * left behind. The data-stage residual gives the transferred length. */
	transferred = (int)t->size;
	for (;;) {
		err = xhci_eventAwait(xhci, XHCI_TRB_TYPE_EVENT_TRANSFER, statusTrbPhys, slot->slotId, 1u, XHCI_CMD_TIMEOUT_MS, &ev);
		if (err < 0) {
			/* The controller intermittently never completes a transfer to a
			 * freshly-reset device (the same inbound-side non-completion as the
			 * command path — see project_usb_addressdevice_wall_129 / the masked
			 * PCIe SError lead). An upstream caller can re-issue the failing
			 * transfer many times; logging every timeout floods the UART and the
			 * back-pressure reset-loops the box. Rate-limit the report so the
			 * system stays up and networked even when enumeration fails this boot
			 * (matches the command-ring treatment in xhci_cmdExec). */
			{
				static unsigned xferTimeoutLogged = 0u;
				if (xferTimeoutLogged < 12u) {
					fprintf(stderr, "xhci: transfer completion timeout\n");
					if (++xferTimeoutLogged == 12u) {
						fprintf(stderr, "xhci: transfer completion timeout: suppressing further reports\n");
					}
				}
			}
			(void)xhci_enterHaltedState(xhci);
			return -ETIMEDOUT;
		}

		completion = (ev.status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
		endpointId = (ev.control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
		slotId = (ev.control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;
		residual = ev.status & XHCI_TRANSFER_EVENT_TRB_STATUS_TRB_TRANSFER_LENGTH__MASK;

		if ((endpointId != 1u) || (slotId != slot->slotId)) {
			fprintf(stderr, "xhci: transfer completion ep/slot mismatch (ep=%u slot=%u)\n", endpointId, slotId);
			(void)xhci_enterHaltedState(xhci);
			return -ENODEV;
		}

		/* An error completes on the failing Setup/Data TRB and aborts the TD
		 * (no Status event follows) — stop here. */
		if ((completion != XHCI_TRB_COMPLETION_CODE_SUCCESS) && (completion != XHCI_TRB_COMPLETION_CODE_SHORT_PACKET)) {
			fprintf(stderr, "xhci: transfer completion code %u\n", completion);
			if (xhci->keepRunning == 0u) {
				(void)xhci_enterHaltedState(xhci);
			}
			return -ENODEV;
		}

		/* Data-stage event carries the transferred length (size - residual). */
		if (ev.parameter == dataTrbPhys) {
			if (residual > t->size) {
				fprintf(stderr, "xhci: invalid transfer residual %u\n", residual);
				return -ENODEV;
			}
			transferred = (int)(t->size - residual);
		}

		/* Status-stage event marks the end of this TD. */
		if (ev.parameter == statusTrbPhys) {
			break;
		}
		/* Otherwise a stale event from a prior transfer — skip, keep draining. */
	}

	/* Keep the controller running across successful control transfers (like
	 * the rig). Halting after each transfer and re-running before the next
	 * makes the following transfer's USB transaction fail on a just-unhalted
	 * controller (the halt-per-command failure, here on ep0). */
	if (xhci->keepRunning == 0u) {
		err = xhci_enterHaltedState(xhci);
		if (err < 0) {
			return err;
		}
	}

	return transferred;
}


static int xhci_ep0ControlWriteNoData(xhci_t *xhci, xhci_slot_t *slot, usb_transfer_t *t)
{
	xhci_trb_t ev;
	uint64_t statusTrbPhys;
	uint32_t completion;
	uint32_t endpointId;
	uint32_t slotId;
	int err;

	if ((xhci == NULL) || (t == NULL) || (t->setup == NULL) || (t->size != 0u) || (t->setup->wLength != 0u)) {
		return -EINVAL;
	}

	/* Setup + Status(IN), no Data stage, at the persistent producer position. */
	xhci_ep0Reserve(xhci, slot, 2u);

	(void)xhci_ep0Push(xhci, slot,
		(uint64_t)t->setup->bmRequestType |
			((uint64_t)t->setup->bRequest << 8) |
			((uint64_t)t->setup->wValue << 16) |
			((uint64_t)t->setup->wIndex << 32) |
			((uint64_t)t->setup->wLength << 48),
		sizeof(usb_setup_packet_t),
		(XHCI_TRB_TYPE_SETUP_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
			(XHCI_TRANSFER_TRB_CONTROL_TRT_NONE << XHCI_TRANSFER_TRB_CONTROL_TRT__SHIFT) |
			XHCI_TRANSFER_TRB_CONTROL_IDT);

	statusTrbPhys = xhci_ep0Push(xhci, slot, 0u, 0u,
		(XHCI_TRB_TYPE_STATUS_STAGE << XHCI_TRB_CONTROL_TRB_TYPE__SHIFT) |
			XHCI_TRANSFER_TRB_CONTROL_DIR_IN |
			XHCI_TRANSFER_TRB_CONTROL_IOC);

	err = xhci_enterRunState(xhci);
	if (err < 0) {
		return err;
	}

	xhci_dbWrite32(xhci, slot->slotId * sizeof(uint32_t), 1u);

	/* Consume events until this TD's Status-TRB event (or an error), draining
	 * any stale events a prior transfer left behind — see the loop in
	 * xhci_ep0ControlRead for the drift rationale. No Data stage here, so a
	 * single Status event is the norm. */
	for (;;) {
		err = xhci_eventAwait(xhci, XHCI_TRB_TYPE_EVENT_TRANSFER, statusTrbPhys, slot->slotId, 1u, XHCI_CMD_TIMEOUT_MS, &ev);
		if (err < 0) {
			/* The controller intermittently never completes a transfer to a
			 * freshly-reset device (the same inbound-side non-completion as the
			 * command path — see project_usb_addressdevice_wall_129 / the masked
			 * PCIe SError lead). An upstream caller can re-issue the failing
			 * transfer many times; logging every timeout floods the UART and the
			 * back-pressure reset-loops the box. Rate-limit the report so the
			 * system stays up and networked even when enumeration fails this boot
			 * (matches the command-ring treatment in xhci_cmdExec). */
			{
				static unsigned xferTimeoutLogged = 0u;
				if (xferTimeoutLogged < 12u) {
					fprintf(stderr, "xhci: transfer completion timeout\n");
					if (++xferTimeoutLogged == 12u) {
						fprintf(stderr, "xhci: transfer completion timeout: suppressing further reports\n");
					}
				}
			}
			(void)xhci_enterHaltedState(xhci);
			return -ETIMEDOUT;
		}

		completion = (ev.status & XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__MASK) >> XHCI_EVENT_TRB_STATUS_COMPLETION_CODE__SHIFT;
		endpointId = (ev.control & XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_ENDPOINTID__SHIFT;
		slotId = (ev.control & XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__MASK) >> XHCI_TRANSFER_EVENT_TRB_CONTROL_SLOTID__SHIFT;

		if ((endpointId != 1u) || (slotId != slot->slotId)) {
			fprintf(stderr, "xhci: transfer completion ep/slot mismatch (ep=%u slot=%u)\n", endpointId, slotId);
			(void)xhci_enterHaltedState(xhci);
			return -ENODEV;
		}

		/* A no-data control transfer's status stage is an IN that the device
		 * answers with a ZLP — the controller reports that as SHORT_PACKET (13),
		 * which is success here (matches ep0ControlRead). SET_CONFIGURATION on the
		 * VIA hub returned 13; treating it as an error wrongly failed hub config. */
		if ((completion != XHCI_TRB_COMPLETION_CODE_SUCCESS) && (completion != XHCI_TRB_COMPLETION_CODE_SHORT_PACKET)) {
			fprintf(stderr, "xhci: transfer completion code %u\n", completion);
			if (xhci->keepRunning == 0u) {
				(void)xhci_enterHaltedState(xhci);
			}
			return -ENODEV;
		}

		if (ev.parameter == statusTrbPhys) {
			break;
		}
		/* Otherwise a stale event from a prior transfer — skip, keep draining. */
	}

	/* Keep the controller running across successful control transfers (like
	 * the rig). Halting after each transfer and re-running before the next
	 * makes the following transfer's USB transaction fail on a just-unhalted
	 * controller (the halt-per-command failure, here on ep0). */
	if (xhci->keepRunning == 0u) {
		err = xhci_enterHaltedState(xhci);
		if (err < 0) {
			return err;
		}
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

	/* Order matters: ERSTSZ first, then ERDP, then ERSTBA LAST. Writing
	 * ERSTBA is what makes the xHC (re)read the ERST and latch its
	 * internal event-ring enqueue/dequeue (xHCI 1.2 §4.9.4 / §5.5.2.3).
	 * Within each 64-bit register pair, write LO then HI. (For the
	 * low-memory rings the HI halves are 0, so the half-order is inert,
	 * but LO-then-HI matches the known-good lwip-port bring-up
	 * sequence.) */
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTSZ, XHCI_ERST_ENTRY_COUNT & XHCI_REG_RT_IR_ERSTSZ__MASK);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_LO, erdpLo);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERDP_HI, erdpHi);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTBA_LO, erstbaLo);
	xhci_rtWrite32(xhci, XHCI_REG_RT_IR_ERSTBA_HI, erstbaHi);

	/* Deliberately NOT setting IMAN.IE / IMOD here: leave both at their
	 * post-HCRST default (IE=0, IMOD=0). IE only gates the IRQ line, not
	 * the controller's DMA of Command Completion / Port Status events to
	 * the ring, so a polled event-ring driver (as here) works with the
	 * interrupter masked. */

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

	{
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
				/* ALLOCATE EVERYTHING FIRST (2026-05-28 experiment):
				 * the 'X' diag rig allocates dcbaa + cmdRing + evtRing
				 * + erst + scratchpadArray + scratchpadBufs ALL in a
				 * single pre-MMIO batch before any controller register
				 * writes. xhci_init previously interleaved allocations
				 * with MMIO writes (allocCommandSpace, programCommand,
				 * allocEventRing, programEventRing). Restructure to
				 * mirror the rig's allocate-then-program pattern in
				 * case the kernel allocator's behavior with MAP_CONTIGUOUS
				 * depends on intervening MMIO writes (which would
				 * affect bridge-side state on BCM2711). */
				err = xhci_allocCommandSpace(xhci);
				if (err == 0) {
					err = xhci_initCommandRing(xhci);
				}
				if (err == 0) {
					err = xhci_allocScratchpads(xhci);
				}
				if (err == 0) {
					err = xhci_allocEventRing(xhci);
				}
				if (err == 0) {
					/* Now do all the MMIO programming, no allocations
					 * in between. */
					err = xhci_programCommandSpace(xhci);
					if (err == 0) {
						err = xhci_programEventRing(xhci);
					}
					if (err == 0) {
						/* No R/S self-test: xhci_cmdNoopSelftest (which enters
						 * the run state via cmdExec) is the liveness check. */
#if defined(__TARGET_AARCH64A72) && defined(PCI_EXPRESS_BCM2711_INDEXED_CFG)
						/* VL805 must run continuously: the halt-per-command
						 * pattern (keepRunning==0) drops R/S after each command
						 * and a just-unhalted VL805 fails the NEXT USB
						 * transaction (AddressDevice -> Context State Error) and,
						 * worse, leaves the controller halted when xhci_init
						 * returns so it posts no Port-Status-Change events and
						 * the roothub enumeration never starts. The rig path set
						 * this after its handoff (#129); set it here too so the
						 * framework's own bring-up keeps R/S=1 across the whole
						 * No-Op -> EnableSlot -> enumeration sequence. Pi4-only:
						 * imx6ull/ia32 et al. shipped on halt-per-command. */
						xhci->keepRunning = 1u;
#endif
						err = xhci_cmdNoopSelftest(xhci);
						if (err == 0) {
							err = xhci_cmdEnableSlot(xhci, &xhci->slots[0].slotId);
							if (err == 0) {
								err = xhci_allocSlotSpace(xhci, &xhci->slots[0]);
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

	/* Synthesize a connect change for a device attached before this
	 * controller's bring-up (CCS=1 with no fresh CSC) so the hub driver
	 * enumerates it; latched off by ClearPortFeature(C_CONNECTION). */
	if (((portsc & XHCI_REG_OP_PORT_PORTSC_CCS) != 0u) &&
		((xhci->portConnAnnounced & (1u << (unsigned)port)) == 0u)) {
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
			/* Hub driver acked the connect — latch it so getHubStatus /
			 * getPortStatus stop synthesizing C_CONNECTION for a device
			 * that was already attached before bring-up. */
			xhci->portConnAnnounced |= (1u << (unsigned)port);
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

	/* Route every transfer to the slot that drives this device: the external
	 * hub on a root port -> slots[0]; a device behind a non-root hub (the
	 * keyboard) -> its own slot. Defaults to slots[0] (the single-slot path) so
	 * root-port behaviour is unchanged. The behind-hub slot is created lazily in
	 * the address==0 block below; until then this returns slots[0], but the
	 * address==0 branch overrides xhci->cur after allocation. */
	if (xhci != NULL) {
		xhci->cur = xhci_slotForDev(xhci, pipe->dev);
	}

	if ((pipe->dev->address == 0) && (xhci != NULL)) {
		/* First contact with a freshly-connected device. xHCI requires the
		 * slot to be addressed before ANY ep0 transfer, so set up the ep0
		 * ring, program the slot/ep0 input context, and issue Address Device
		 * once. We use the non-BSR form (assign the USB address = slotId
		 * immediately): the framework still believes the device sits at the
		 * default address and issues SET_ADDRESS next, which we acknowledge
		 * without re-issuing the command — a second Address Device would
		 * reload trDequeuePtr and rewind the now-persistent ep0 ring
		 * mid-enumeration. The maxPacketSize0 fix-up the spec permits between
		 * the first descriptor read and SET_ADDRESS is a no-op for high- and
		 * super-speed (fixed 64/512) and is a TODO for low/full-speed devices
		 * once those are reachable behind a hub. */
		if ((pipe->dev->hub != NULL) && (pipe->dev->hub->hub != NULL)) {
			/* Device behind a non-root hub (the low-speed keyboard behind the
			 * VIA hub). It needs its OWN xHCI slot with a route string + TT;
			 * slots[0] (the hub) cannot be reused. Allocate + address it once,
			 * then drive its descriptor reads on that slot. ADDITIVE: the
			 * root-port (hub/slots[0]) path below is untouched. */
			xhci_slot_t *kbdSlot = xhci_findSlotForDev(xhci, pipe->dev);
			if (kbdSlot == NULL) {
				kbdSlot = xhci_allocSlotForDev(xhci, pipe->dev, &err);
				if (kbdSlot == NULL) {
					return err;
				}
			}
			xhci->cur = kbdSlot;
		}
		else if (xhci->cur->addressed == 0u) {
			err = xhci_initEp0Ring(xhci, xhci->cur);
			if (err < 0) {
				return err;
			}

			err = xhci_prepareAddressContext(xhci, xhci->cur, pipe->dev);
			if (err < 0) {
				return err;
			}

			/* Two-step addressing (Linux xhci_setup_device scheme, #129). BSR=1
			 * (setAddress==0) sets up the slot + ep0 context WITHOUT issuing
			 * SET_ADDRESS on the wire — it reads the input context only. BSR=0
			 * (setAddress==1) then assigns the address. The single-step BSR=0 form
			 * intermittently never completes on the Pi4 VL805 (~3/4 cold boots);
			 * the controller dequeues EnableSlot fine but the BSR=0 AddressDevice
			 * produces no completion at all. Splitting it isolates the wire step
			 * from the context read and matches what Linux does to be deterministic.
			 * Safe here: the ep0 ring was just initialised and no ep0 transfer has
			 * happened yet, so the BSR=0 step's trDequeuePtr (re)load is a no-op. */
			err = xhci_cmdAddressDevice(xhci, xhci->cur, 0);
			if (err < 0) {
				return err;
			}

			err = xhci_cmdAddressDevice(xhci, xhci->cur, 1);
			if (err < 0) {
				return err;
			}

			xhci->cur->addressed = 1u;
		}

		if ((setup != NULL) && (setup->bRequest == REQ_SET_ADDRESS)) {
			/* The device is ALREADY addressed in hardware (Address Device,
			 * BSR=0, assigned the slot's USB address during first contact). The
			 * framework's USB address is its own bookkeeping (here 2 — the root
			 * hub took 1) and need NOT equal xhci->slotId: xHCI routes by slot
			 * via the doorbell, not by the address in the SETUP packet. So
			 * acknowledge whatever address the framework assigns; transfers for
			 * this device are then routed by dev->address != 0 below. */
			usb_transferFinished(t, 0);
			return 0;
		}

		/* Default-address device-descriptor read (the framework's first probe,
		 * before SET_ADDRESS). The slot is addressed above, so route by request
		 * rather than by dev->address (still 0 at this point). */
		if ((setup != NULL) && (setup->bRequest == REQ_GET_DESCRIPTOR) &&
			(t->type == usb_transfer_control) && (t->direction == usb_dir_in)) {
			err = xhci_ep0ControlRead(xhci, xhci->cur, t);
			if (err < 0) {
				return err;
			}

			usb_transferFinished(t, err);
			return 0;
		}

		return -ENOSYS;
	}

	/* Any control-IN (DEV2HOST, has data) on the addressed device's ep0 — the
	 * HCD just runs the control transfer; the device interprets the request.
	 * Covers standard GetDescriptor as well as class requests the hub driver
	 * issues (GetHubDescriptor, GetPortStatus, GetStatus, ...). */
	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub == NULL) &&
		(pipe->dev->address != 0) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_in) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_DEV2HOST)) {
		err = xhci_ep0ControlRead(xhci, xhci->cur, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, err);
		return 0;
	}

	/* Any no-data control-OUT (HOST2DEV, wLength==0) on ep0 — SetConfiguration,
	 * SetPortFeature/ClearPortFeature (class, recipient OTHER), SetProtocol,
	 * SetIdle, etc. */
	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub == NULL) &&
		(pipe->dev->address != 0) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_out) &&
		(t->size == 0u) &&
		(setup->wLength == 0u) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_HOST2DEV)) {
		err = xhci_ep0ControlWriteNoData(xhci, xhci->cur, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, 0);
		return 0;
	}

	/* Interrupt-IN endpoint on any addressed device hanging off a hub: the hub's
	 * own status-change endpoint (slots[0]) AND a device behind a non-root hub
	 * (the keyboard's HID endpoint, on its own slot). Per-slot interrupt pipes
	 * (slot->interruptPriv) let both coexist; xhci_initInterruptInPipe derives
	 * the owning slot from pipe->dev and the roothub thread polls all slots. */
	if ((xhci != NULL) &&
		(pipe->dev->hub != NULL) &&
		(pipe->dev->address != 0) &&
		(t->type == usb_transfer_interrupt) &&
		(t->direction == usb_dir_in)) {
		err = xhci_initInterruptInPipe(xhci, pipe);
		if (err < 0) {
			return err;
		}

		return xhci_submitInterruptIn(xhci, t, pipe);
	}

	/* Addressed device BEHIND a non-root hub (the keyboard on its own slot):
	 * control transfers. xhci->cur was set to that slot at the top. */
	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub != NULL) &&
		(pipe->dev->address != 0) &&
		(xhci_findSlotForDev(xhci, pipe->dev) != NULL) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_in) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_DEV2HOST)) {
		err = xhci_ep0ControlRead(xhci, xhci->cur, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, err);
		return 0;
	}

	if ((xhci != NULL) && (setup != NULL) &&
		(pipe->dev->hub != NULL) && (pipe->dev->hub->hub != NULL) &&
		(pipe->dev->address != 0) &&
		(xhci_findSlotForDev(xhci, pipe->dev) != NULL) &&
		(t->type == usb_transfer_control) &&
		(t->direction == usb_dir_out) &&
		(t->size == 0u) &&
		(setup->wLength == 0u) &&
		((setup->bmRequestType & REQUEST_DIR_MASK) == REQUEST_DIR_HOST2DEV)) {
		err = xhci_ep0ControlWriteNoData(xhci, xhci->cur, t);
		if (err < 0) {
			return err;
		}

		usb_transferFinished(t, 0);
		return 0;
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
	/* Clear the owning slot's interrupt pipe by pointer match (slots[] is indexed
	 * by array position, not slotId, so don't index by priv->slotId here). */
	{
		unsigned s;
		for (s = 0u; s < (sizeof(xhci->slots) / sizeof(xhci->slots[0])); ++s) {
			if (xhci->slots[s].interruptPriv == priv) {
				xhci->slots[s].interruptPriv = NULL;
			}
		}
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
		uint32_t bit = 1u << (i + 1);
		portsc = xhci_portRead32(xhci, i + 1, XHCI_REG_OP_PORT_PORTSC);
		if ((portsc & XHCI_REG_OP_PORT_PORTSC_CCS) == 0u) {
			/* Disconnected: drop the latch so a future re-attach
			 * re-announces. */
			xhci->portConnAnnounced &= ~bit;
		}
		/* Report a port to the hub driver on a real RW1C change OR when a
		 * device is attached (CCS=1) that we have not yet announced — the
		 * latter covers a device already attached before bring-up (no
		 * fresh CSC), which the change-bit-only logic otherwise ignored. */
		if (((portsc & XHCI_REG_OP_PORT_PORTSC_RW1C) != 0u) ||
			(((portsc & XHCI_REG_OP_PORT_PORTSC_CCS) != 0u) && ((xhci->portConnAnnounced & bit) == 0u))) {
			status |= bit;
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
