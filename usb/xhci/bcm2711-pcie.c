/*
 * Phoenix-RTOS
 *
 * PCI Express driver server
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/debug.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <board_config.h>

#include "bcm2711-pcie.h"


#ifdef PCI_EXPRESS_INIT_TEBF0808_PHY
#include <tebf0808-pcie-refclk.h>
#include <tebf0808-ps-gtr-phy.h>
#endif


#ifdef PCI_EXPRESS_XILINX_NWL
#include <pcie-xilinx-nwl.h>
#endif


#ifdef PCI_EXPRESS_XILINX_AXI
#include <pcie-xilinx-axi.h>
#endif


/* ECAM commands */
#define PCI_CMD_IO_ENABLE         0x01
#define PCI_CMD_MEM_ENABLE        0x02
#define PCI_CMD_MASTER_ENABLE     0x04
#define PCI_CMD_PARITY_ERR_ENABLE 0x40
#define PCI_CMD_SERR_ERR_ENABLE   0x100


/* ECAM header offsets */
#define PCI_VENDOR_ID       0x00
#define PCI_DEVICE_ID       0x02
#define PCI_COMMAND         0x04
#define PCI_STATUS          0x06
#define PCI_CLASSCODE       0x08
#define PCI_CACHE_LINE_SIZE 0x0c
#define PCI_HEADER_TYPE     0x0e
#define PCI_BAR0            0x10
#define PCI_PRIMARY_BUS     0x18
#define PCI_SECONDARY_BUS   0x19
#define PCI_SUBORDINATE_BUS 0x1a
#define PCI_MEMORY_BASE     0x20
#define PCI_MEMORY_LIMIT    0x22
#define PCI_CAP_PTR         0x34
#define PCI_BRIDGE_CONTROL  0x3e


#define PCI_HT_MULTI_FUNC 0x80

#define PCI_BRIDGE_CTL_PARITY 0x01

#define PCI_CAP_LIST_ID    0x00
#define PCI_CAP_ID_EXP     0x10
#define PCI_EXP_RTCTL      0x1c
#define PCI_EXP_RTCTL_CRSSVE 0x0010


/* ECAM addressing */
#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12


typedef struct {
	void *ctx;
	uint32_t (*read32)(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
	void (*write32)(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val);
	void (*destroy)(void *ctx);
} pcie_cfgio_t;


typedef struct {
	uint32_t *base;
} pcie_ecam_ctx_t;


typedef struct {
	uint32_t *base;
	bool linkUp;
	bool rcMode;
} pcie_bcm2711_ctx_t;


static void pcie_scanBus(pcie_cfgio_t *cfgio, uint8_t bus);


/* Pre-created xhci MMIO mapping, see pcie_scanProbe's TD-USB-pmap workaround. */
static volatile void *bcm2711_pcie_xhciMmio = NULL;


/* Persistent BCM2711 PCIe context. Set by bcm2711_pcie_initVL805 (the
 * one-shot bridge bring-up at boot) and kept alive so callers can
 * re-program the outbound window after the bridge translation gets
 * invalidated by a downstream xHCI operation (e.g. HCRST). The
 * underlying host-bridge config-space mapping is leaked-by-design (see
 * the comment block at the bottom of bcm2711_pcie_initVL805), so the
 * pointer remains valid for the lifetime of the usb daemon. */
static pcie_bcm2711_ctx_t *bcm2711_pcie_lastCtx = NULL;


/* Same size as xhci.c's XHCI_MAP_SIZE. The VL805's BAR0 is 4 KiB on the
 * Pi 4 (verified against cross-OS reference implementations); mapping a
 * larger region spilled past the BAR into unmapped PCIe space and the
 * BCM2711 root complex returned 0xdeaddead. Hard-coded here to keep
 * bcm2711-pcie.c decoupled from xhci.c internals. */
#define BCM2711_PCIE_XHCI_MMIO_SIZE 0x1000u


volatile void *bcm2711_pcie_getXhciMmio(void)
{
	return bcm2711_pcie_xhciMmio;
}


uint64_t bcm2711_pcie_getXhciMmioSize(void)
{
	return BCM2711_PCIE_XHCI_MMIO_SIZE;
}


static inline uint16_t pcie_cfgRead16(pcie_cfgio_t *cfgio, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = cfgio->read32(cfgio->ctx, bus, dev, fn, off);
	if (off & 2) {
		return value_u32 >> 16;
	}
	else {
		return value_u32 & 0xffff;
	}
}


static inline uint8_t pcie_cfgRead8(pcie_cfgio_t *cfgio, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = cfgio->read32(cfgio->ctx, bus, dev, fn, off);
	return (value_u32 >> ((off & 3) * 8)) & 0xff;
}


#ifndef PCI_EXPRESS_BCM2711_INDEXED_CFG

static inline volatile uint32_t *ecamRegPtr(pcie_ecam_ctx_t *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfff);

	return (volatile uint32_t *)((uintptr_t)ecam->base + cfg_space_offset);
}


static uint32_t ecamRead32(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	pcie_ecam_ctx_t *ecam = (pcie_ecam_ctx_t *)ctx;

	return *ecamRegPtr(ecam, bus, dev, fn, off & ~0x3);
}


static void ecamWrite32(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	pcie_ecam_ctx_t *ecam = (pcie_ecam_ctx_t *)ctx;

	*ecamRegPtr(ecam, bus, dev, fn, off & ~0x3) = val;
}


static void pcie_cfgDestroyEcam(void *ctx)
{
	pcie_ecam_ctx_t *ecam = (pcie_ecam_ctx_t *)ctx;

	if (ecam == NULL) {
		return;
	}

	if (ecam->base != MAP_FAILED) {
		munmap((void *)ecam->base, ECAM_SIZE);
	}

	free(ecam);
}


static int pcie_cfgInitEcam(pcie_cfgio_t *cfgio)
{
	pcie_ecam_ctx_t *ecam;

	ecam = calloc(1, sizeof(*ecam));
	if (ecam == NULL) {
		return -ENOMEM;
	}

	ecam->base = mmap(NULL, ECAM_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, ECAM_ADDRESS);
	if (ecam->base == MAP_FAILED) {
		free(ecam);
		return -ENOMEM;
	}

	cfgio->ctx = ecam;
	cfgio->read32 = ecamRead32;
	cfgio->write32 = ecamWrite32;
	cfgio->destroy = pcie_cfgDestroyEcam;

	return EOK;
}

#endif


#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG

#if defined(RPI_MAILBOX_BASE_ADDRESS) && defined(XHCI_BCM2711_PCIE_BUS) && defined(XHCI_BCM2711_PCIE_SLOT) && defined(XHCI_BCM2711_PCIE_FUNC) && defined(XHCI_BCM2711_PCI_CLASS_CODE)

#define RPI_MBOX_READ          0x00u
#define RPI_MBOX_STATUS        0x18u
#define RPI_MBOX_WRITE         0x20u
#define RPI_MBOX_RESPONSE      0x80000000u
#define RPI_MBOX_FULL          0x80000000u
#define RPI_MBOX_EMPTY         0x40000000u
#define RPI_MBOX_PROP_CHANNEL  8u
#define RPI_PROP_REQUEST       0u
#define RPI_PROP_END           0u
#define RPI_PROP_NOTIFY_XHCI_RESET 0x00030058u
#define RPI_PROP_NOTIFY_MSG_WORDS 7u

static int bcm2711NotifyXhciReset(uint8_t bus, uint8_t dev, uint8_t fun)
{
	volatile uint8_t *mailbox_page;
	volatile uint32_t *mailbox;
	uint32_t *msgbuf;
	uint32_t msg;
	uintptr_t msgaddr;
	uintptr_t mailbox_pa_base;
	uintptr_t mailbox_pa_offs;
	int ret;

	/* The BCM2711 mailbox MMIO sits at PA 0xfe00b880 — NOT page-aligned.
	 * Phoenix's mmap rejects a non-page-aligned `off` argument with
	 * MAP_FAILED (vm_pageAlloc for a contiguous device mapping refuses
	 * to map a partial page). Round the PA down to a page boundary and
	 * fix up the offset within the page after mmap returns. */
	mailbox_pa_base = (uintptr_t)RPI_MAILBOX_BASE_ADDRESS & ~(uintptr_t)(_PAGE_SIZE - 1U);
	mailbox_pa_offs = (uintptr_t)RPI_MAILBOX_BASE_ADDRESS & (uintptr_t)(_PAGE_SIZE - 1U);

	mailbox_page = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, mailbox_pa_base);
	if (mailbox_page == MAP_FAILED) {
		debug("pcie: notifyXhciReset mailbox mmap FAILED\n");
		return -ENOMEM;
	}
	mailbox = (volatile uint32_t *)(mailbox_page + mailbox_pa_offs);

	msgbuf = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msgbuf == MAP_FAILED) {
		debug("pcie: notifyXhciReset MAP_CONTIGUOUS mmap FAILED\n");
		munmap((void *)mailbox_page, _PAGE_SIZE);
		return -ENOMEM;
	}

	msgbuf[0] = RPI_PROP_NOTIFY_MSG_WORDS * sizeof(uint32_t);
	msgbuf[1] = RPI_PROP_REQUEST;
	msgbuf[2] = RPI_PROP_NOTIFY_XHCI_RESET;
	msgbuf[3] = sizeof(uint32_t);
	msgbuf[4] = sizeof(uint32_t);
	msgbuf[5] = ((uint32_t)bus << 20) | ((uint32_t)dev << 15) | ((uint32_t)fun << 12);
	msgbuf[6] = RPI_PROP_END;

	msgaddr = va2pa(msgbuf);
	if (msgaddr == (uintptr_t)-1) {
		munmap(msgbuf, _PAGE_SIZE);
		munmap((void *)mailbox_page, _PAGE_SIZE);
		return -EFAULT;
	}
	msg = ((uint32_t)msgaddr & ~0xfu) | RPI_MBOX_PROP_CHANNEL;
	while ((*(mailbox + (RPI_MBOX_STATUS / sizeof(uint32_t))) & RPI_MBOX_FULL) != 0u) {
	}

	*(mailbox + (RPI_MBOX_WRITE / sizeof(uint32_t))) = msg;

	for (;;) {
		while ((*(mailbox + (RPI_MBOX_STATUS / sizeof(uint32_t))) & RPI_MBOX_EMPTY) != 0u) {
		}

		if (*(mailbox + (RPI_MBOX_READ / sizeof(uint32_t))) == msg) {
			break;
		}
	}

	ret = (msgbuf[1] == RPI_MBOX_RESPONSE) ? EOK : -EIO;
	if (ret != EOK) {
		fprintf(stderr, "pcie: notifyXhciReset failed: ret=%d resp=%08x\n", ret, msgbuf[1]);
	}

	munmap(msgbuf, _PAGE_SIZE);
	munmap((void *)mailbox_page, _PAGE_SIZE);

	return ret;
}

#endif

#define BCM2711_PCIE_EXT_CFG_DATA  0x8000u
#define BCM2711_PCIE_EXT_CFG_INDEX 0x9000u

#define BCM2711_PCIE_BUSNUM_SHIFT 20
#define BCM2711_PCIE_SLOT_SHIFT   15
#define BCM2711_PCIE_FUNC_SHIFT   12

#define BCM2711_PCIE_MISC_CTRL            0x4008u
#define BCM2711_PCIE_CAP_REGS             0x00acu
#define BCM2711_PCIE_MEM_WIN0_LO          0x400cu
#define BCM2711_PCIE_MEM_WIN0_HI          0x4010u
#define BCM2711_PCIE_RC_CFG_PRIV1_ID_VAL3 0x043cu
#define BCM2711_PCIE_RC_BAR1_CONFIG_LO    0x402cu
#define BCM2711_PCIE_RC_BAR2_CONFIG_LO    0x4034u
/* USB-FIX-16 (2026-05-26): RC_BAR3 is the PCIe -> SCB peripheral
 * window. Pi 4 firmware sometimes leaves it with a non-zero default
 * SIZE mapping that shadows the BAR2 path; inbound TLPs from PCIe
 * devices get silently routed to the SCB aperture instead of DRAM.
 * Linux's pcie-brcmstb.c::brcm_pcie_setup() always clears the SIZE
 * field unconditionally. */
#define BCM2711_PCIE_RC_BAR3_CONFIG_LO    0x403cu
#define BCM2711_PCIE_RC_BAR2_CONFIG_HI    0x4038u
/* PCIE_MISC_UBUS_BAR2_CONFIG_REMAP: CPU-side enable for inbound BAR2
 * window. On BCM2711 reset, bit 0 (ACCESS_ENABLE) is 0, gating all
 * inbound DMA from PCIe devices at the UBUS interconnect. Linux's
 * pcie-brcmstb.c::brcm_pcie_setup() sets it; without it, VL805's
 * DMA reads/writes silently disappear at the bridge -- exactly the
 * symptom we've been chasing all day. */
#define BCM2711_PCIE_UBUS_BAR2_REMAP_LO   0x40b4u
#define BCM2711_PCIE_UBUS_BAR2_REMAP_HI   0x40b8u
#define BCM2711_PCIE_UBUS_BAR2_ACCESS_EN  0x1u
#define BCM2711_PCIE_MISC_STATUS          0x4068u
#define BCM2711_PCIE_MISC_REVISION        0x406cu
#define BCM2711_PCIE_MEM_WIN0_BASE_LIMIT  0x4070u
#define BCM2711_PCIE_MEM_WIN0_BASE_HI     0x4080u
#define BCM2711_PCIE_MEM_WIN0_LIMIT_HI    0x4084u
#define BCM2711_PCIE_HARD_DEBUG           0x4204u
#define BCM2711_PCIE_RGR1_SW_INIT_1       0x9210u

#define BCM2711_PCIE_RGR1_PERST_MASK        0x1u
#define BCM2711_PCIE_RGR1_INIT_GENERIC_MASK 0x2u

#define BCM2711_PCIE_RC_CLASS_CODE_MASK      0x00ffffffu
#define BCM2711_PCIE_RC_BRIDGE_CLASS_CODE    0x00060400u
#define BCM2711_PCIE_RC_BAR2_SIZE_MASK       0x1fu

#define BCM2711_PCIE_MISC_CTRL_SCB_ACCESS_EN   0x1000u
#define BCM2711_PCIE_MISC_CTRL_CFG_READ_UR_MODE 0x2000u
#define BCM2711_PCIE_MISC_CTRL_MAX_BURST_MASK  0x300000u
/* Vendor-specific register at config offset 0x0188 controls the PCIe-to-SCB
 * endian mode for the inbound BAR2 window. Bits 2:3 = ENDIAN_MODE_BAR2,
 * with 0x0 = little-endian (matches ARM64 LE). Without writing this, the
 * bridge can byte-swap inbound DMA data → controller sees garbage TRBs →
 * USBSTS.HSE on the first R/S=1 fetch. Linux sets this in brcm_pcie_setup. */
#define BCM2711_PCIE_RC_CFG_VENDOR_VSR1        0x0188u
#define BCM2711_PCIE_RC_CFG_VSR1_ENDIAN_BAR2   0xcu     /* bits 2:3 */
/* From Linux pcie-brcmstb.c — required for BCM2711 inbound DMA. */
#define BCM2711_PCIE_MISC_CTRL_RCB_MPS_MODE    0x400u    /* bit 10 */
#define BCM2711_PCIE_MISC_CTRL_RCB_64B_MODE    0x80u     /* bit 7  */
#define BCM2711_PCIE_MISC_CTRL_SCB0_SIZE_MASK  0xf8000000u
/* log2(4 GiB) - 15 = 32 - 15 = 17, placed in bits 31:27 = 0x88000000. */
#define BCM2711_PCIE_MISC_CTRL_SCB0_SIZE_4G    (17u << 27)

#define BCM2711_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK 0x08000000u

#define BCM2711_PCIE_MISC_STATUS_PCIE_PORT_MASK       0x80u
#define BCM2711_PCIE_MISC_STATUS_PCIE_DL_ACTIVE_MASK  0x20u
#define BCM2711_PCIE_MISC_STATUS_PCIE_PHYLINKUP_MASK  0x10u

#define BCM2711_PCIE_MEM_WIN0_BASE_MASK           0x0000fff0u
#define BCM2711_PCIE_MEM_WIN0_LIMIT_MASK          0xfff00000u
#define BCM2711_PCIE_MEM_WIN0_BASE_HI_MASK        0x000000ffu
#define BCM2711_PCIE_MEM_WIN0_LIMIT_HI_MASK       0x000000ffu
#define BCM2711_PCIE_MEM_WIN0_BASE_LIMIT_SHIFT    20u
#define BCM2711_PCIE_MEM_WIN0_BASE_SHIFT          4u
#define BCM2711_PCIE_MEM_WIN0_NUM_MASK_BITS       12u


static inline int bcm2711_cfgIndex(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return ((dev & 0x1f) << BCM2711_PCIE_SLOT_SHIFT) |
		((fn & 0x07) << BCM2711_PCIE_FUNC_SHIFT) |
		((int)bus << BCM2711_PCIE_BUSNUM_SHIFT) |
		(off & ~0x3);
}


static inline volatile uint32_t *bcm2711RootCfgPtr(pcie_bcm2711_ctx_t *ctx, uint16_t off)
{
	return (volatile uint32_t *)((uintptr_t)ctx->base + (off & ~0x3));
}


static uint32_t bcm2711RootRead32(pcie_bcm2711_ctx_t *ctx, uint16_t off)
{
	return *bcm2711RootCfgPtr(ctx, off);
}


static void bcm2711RootWrite32(pcie_bcm2711_ctx_t *ctx, uint16_t off, uint32_t val)
{
	*bcm2711RootCfgPtr(ctx, off) = val;
}


static uint16_t bcm2711RootRead16(pcie_bcm2711_ctx_t *ctx, uint16_t off)
{
	uint32_t value = bcm2711RootRead32(ctx, off);

	return ((off & 2u) != 0u) ? (uint16_t)(value >> 16) : (uint16_t)(value & 0xffffu);
}


static uint8_t bcm2711RootRead8(pcie_bcm2711_ctx_t *ctx, uint16_t off)
{
	uint32_t value = bcm2711RootRead32(ctx, off);

	return (uint8_t)((value >> ((off & 3u) * 8u)) & 0xffu);
}


static void bcm2711RootWrite16(pcie_bcm2711_ctx_t *ctx, uint16_t off, uint16_t val)
{
	uint32_t value = bcm2711RootRead32(ctx, off);

	if ((off & 2u) != 0u) {
		value &= 0x0000ffffu;
		value |= (uint32_t)val << 16;
	}
	else {
		value &= 0xffff0000u;
		value |= val;
	}

	bcm2711RootWrite32(ctx, off, value);
}


static void bcm2711RootWrite8(pcie_bcm2711_ctx_t *ctx, uint16_t off, uint8_t val)
{
	uint32_t value = bcm2711RootRead32(ctx, off);
	unsigned shift = (off & 3u) * 8u;

	value &= ~(0xffu << shift);
	value |= (uint32_t)val << shift;
	bcm2711RootWrite32(ctx, off, value);
}


static void bcm2711BridgeSwInitSet(pcie_bcm2711_ctx_t *ctx, uint32_t val)
{
	writeRegMsk(ctx->base, BCM2711_PCIE_RGR1_SW_INIT_1,
		BCM2711_PCIE_RGR1_INIT_GENERIC_MASK,
		val ? BCM2711_PCIE_RGR1_INIT_GENERIC_MASK : 0u);
}


static void bcm2711PerstSet(pcie_bcm2711_ctx_t *ctx, uint32_t val)
{
	writeRegMsk(ctx->base, BCM2711_PCIE_RGR1_SW_INIT_1,
		BCM2711_PCIE_RGR1_PERST_MASK,
		val ? BCM2711_PCIE_RGR1_PERST_MASK : 0u);
}


static void bcm2711PrepareHostBridge(pcie_bcm2711_ctx_t *ctx)
{
	uint32_t misc;

	bcm2711BridgeSwInitSet(ctx, 1u);
	bcm2711PerstSet(ctx, 1u);
	/* PCIe CEM spec 2.0 §2.6.2: PERST# MUST be asserted for at least
	 * 100 ms with stable power before deassert. Phoenix's prior 200 us
	 * was orders of magnitude short of spec — bridge / VL805 internal
	 * state from start4.elf wasn't fully cleared, leaving the inbound
	 * DMA path in an intermittent broken state. Linux brcm_pcie holds
	 * PERST for the full 100 ms before deassert in start_link. */
	usleep(100000);

	bcm2711BridgeSwInitSet(ctx, 0u);
	writeRegMsk(ctx->base, BCM2711_PCIE_HARD_DEBUG,
		BCM2711_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK, 0u);
	usleep(200);

	(void)readReg(ctx->base, BCM2711_PCIE_MISC_REVISION);

	misc = readReg(ctx->base, BCM2711_PCIE_MISC_CTRL);
	misc |= BCM2711_PCIE_MISC_CTRL_SCB_ACCESS_EN;
	misc |= BCM2711_PCIE_MISC_CTRL_CFG_READ_UR_MODE;
	misc &= ~BCM2711_PCIE_MISC_CTRL_MAX_BURST_MASK; /* 0 = 128-byte burst per Linux for BCM2711 */
	/* Linux brcm_pcie_setup also sets these MISC_CTRL bits for BCM2711;
	 * without them the inbound BAR2 window doesn't reliably serve VL805
	 * DMA reads (symptom: USBSTS.HSE on first xhci R/S=1 transition,
	 * USBSTS=0x15 = HCH|HSE|PCD). */
	misc |= BCM2711_PCIE_MISC_CTRL_RCB_MPS_MODE;
	misc |= BCM2711_PCIE_MISC_CTRL_RCB_64B_MODE;
	misc &= ~BCM2711_PCIE_MISC_CTRL_SCB0_SIZE_MASK;
	misc |= BCM2711_PCIE_MISC_CTRL_SCB0_SIZE_4G;
	writeReg(ctx->base, BCM2711_PCIE_MISC_CTRL, misc);

	/* Force the bridge's PCIe-to-SCB inbound endian-mode for BAR2
	 * to little-endian (matches ARM64 LE memory layout). Without this
	 * the bridge's default mode can byte-swap inbound DMA data,
	 * making the controller's reads of cmd ring / event ring /
	 * DCBAA see garbage TRBs and set USBSTS.HSE on R/S=1. */
	{
		uint32_t vsr1 = readReg(ctx->base, BCM2711_PCIE_RC_CFG_VENDOR_VSR1);
		vsr1 &= ~BCM2711_PCIE_RC_CFG_VSR1_ENDIAN_BAR2;
		writeReg(ctx->base, BCM2711_PCIE_RC_CFG_VENDOR_VSR1, vsr1);
	}
}


static bool bcm2711LinkUp(pcie_bcm2711_ctx_t *ctx)
{
	uint32_t status = readReg(ctx->base, BCM2711_PCIE_MISC_STATUS);

	return ((status & BCM2711_PCIE_MISC_STATUS_PCIE_DL_ACTIVE_MASK) != 0u) &&
		((status & BCM2711_PCIE_MISC_STATUS_PCIE_PHYLINKUP_MASK) != 0u);
}


static bool bcm2711RcMode(pcie_bcm2711_ctx_t *ctx)
{
	uint32_t status = readReg(ctx->base, BCM2711_PCIE_MISC_STATUS);

	return (status & BCM2711_PCIE_MISC_STATUS_PCIE_PORT_MASK) != 0u;
}


static void bcm2711PrepareLinkState(pcie_bcm2711_ctx_t *ctx)
{
	int polls;

	bcm2711PerstSet(ctx, 0u);

	/* PCIe CEM spec 2.0 §2.2 / PCIe r5.0 §6.6.1: wait 100 ms after
	 * PERST# deassertion before initiating configuration cycles.
	 * The downstream device needs this window to come out of reset
	 * cleanly. Linux msleep(100) here. Without this Phoenix's earlier
	 * config-space accesses race the device's init and can leave
	 * VL805 in a partial state that surfaces later as inbound DMA
	 * failures (USBSTS.HSE on R/S=1 or cmd ring processing). */
	usleep(100000);

	/* Poll for link up; on a healthy Pi 4 the link comes up within
	 * 5–30 ms after PERST# deassert (we've already waited 100 ms).
	 * The 100 ms ceiling here matches Linux's brcmstb-pcie driver
	 * worst-case. */
	for (polls = 0; polls < 50; polls++) {
		usleep(2000);
		if (bcm2711LinkUp(ctx)) {
			break;
		}
	}

	ctx->linkUp = bcm2711LinkUp(ctx);
	ctx->rcMode = bcm2711RcMode(ctx);
	if (ctx->linkUp == 0) {
		fprintf(stderr, "pcie: link did not come up after %d ms (rcMode=%d)\n",
			polls * 2, ctx->rcMode);
	}
}


static uint32_t bcm2711EncodeBar2Size(uint64_t size)
{
	/* BCM2711 PCIe RC_BAR2 size field encoding (per Linux
	 * pcie-brcmstb.c::brcm_pcie_get_dma_ranges_property):
	 *
	 *   field = log2(size_bytes) - 15
	 *
	 * Valid range: 64 KB (field=1) through 4 GB (field=17). The
	 * field is 5 bits wide so the max representable value is 31
	 * (= 2^46 = 64 TB, plenty of headroom).
	 *
	 * USB-FIX-12 (2026-05-26): the previous implementation started
	 * `shift = 20` and counted right-shifts; for 4 GB input
	 * (size = 2^32) the loop ran 32 times producing shift=52 and
	 * return value 37. With BCM2711_PCIE_RC_BAR2_SIZE_MASK = 0x1F
	 * (5 bits), 37 silently truncated to 5 -- which the bridge
	 * interprets as a 1 MB window. The VL805 cmd-ring DMA target
	 * (cmd_phys = 0x33060000 = 854 MB) fell OUTSIDE the actual
	 * 1 MB window, so VL805's inbound TLP had no valid PCIe-side
	 * destination, the bridge dropped it, the controller saw a
	 * completion timeout, and USBSTS.HSE went high. This was the
	 * root cause of the rc=-110 wedge that survived every other
	 * hypothesis test today. */
	unsigned shift;
	uint64_t value = size;

	if (size == 0u) {
		return 0u;
	}

	/* Find log2(size). */
	shift = 0u;
	while ((value & 1u) == 0u) {
		value >>= 1;
		shift++;
	}

	if (value != 1u || shift < 15u || shift > 46u) {
		return 0u;
	}

	return shift - 15u;
}


static void bcm2711SetOutboundWindow0(pcie_bcm2711_ctx_t *ctx, uint64_t cpuAddr, uint64_t pcieAddr, uint64_t size)
{
	uint64_t cpuAddrMb = cpuAddr >> 20;
	uint64_t limitAddrMb = (cpuAddr + size - 1u) >> 20;
	uint32_t baseLimit;
	uint32_t value;

	writeReg(ctx->base, BCM2711_PCIE_MEM_WIN0_LO, LOWER_32_BITS(pcieAddr));
	writeReg(ctx->base, BCM2711_PCIE_MEM_WIN0_HI, UPPER_32_BITS(pcieAddr));

	baseLimit = readReg(ctx->base, BCM2711_PCIE_MEM_WIN0_BASE_LIMIT);
	baseLimit &= ~(BCM2711_PCIE_MEM_WIN0_BASE_MASK | BCM2711_PCIE_MEM_WIN0_LIMIT_MASK);
	baseLimit |= ((uint32_t)cpuAddrMb << BCM2711_PCIE_MEM_WIN0_BASE_SHIFT) & BCM2711_PCIE_MEM_WIN0_BASE_MASK;
	baseLimit |= ((uint32_t)limitAddrMb << BCM2711_PCIE_MEM_WIN0_BASE_LIMIT_SHIFT) & BCM2711_PCIE_MEM_WIN0_LIMIT_MASK;
	writeReg(ctx->base, BCM2711_PCIE_MEM_WIN0_BASE_LIMIT, baseLimit);

	value = readReg(ctx->base, BCM2711_PCIE_MEM_WIN0_BASE_HI);
	value &= ~BCM2711_PCIE_MEM_WIN0_BASE_HI_MASK;
	value |= (uint32_t)(cpuAddrMb >> BCM2711_PCIE_MEM_WIN0_NUM_MASK_BITS) & BCM2711_PCIE_MEM_WIN0_BASE_HI_MASK;
	writeReg(ctx->base, BCM2711_PCIE_MEM_WIN0_BASE_HI, value);

	value = readReg(ctx->base, BCM2711_PCIE_MEM_WIN0_LIMIT_HI);
	value &= ~BCM2711_PCIE_MEM_WIN0_LIMIT_HI_MASK;
	value |= (uint32_t)(limitAddrMb >> BCM2711_PCIE_MEM_WIN0_NUM_MASK_BITS) & BCM2711_PCIE_MEM_WIN0_LIMIT_HI_MASK;
	writeReg(ctx->base, BCM2711_PCIE_MEM_WIN0_LIMIT_HI, value);
}


static void bcm2711SetRcBar2(pcie_bcm2711_ctx_t *ctx, uint64_t pcieAddr, uint64_t size)
{
	uint32_t value = readReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_LO);

	value &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
	value |= bcm2711EncodeBar2Size(size) & BCM2711_PCIE_RC_BAR2_SIZE_MASK;
	/* USB-FIX-12b (2026-05-26): address portion is bits 12:31 (PCIe
	 * addresses are 4 KB aligned, and the register layout reserves
	 * bits 5:11 as 0). The previous mask 0xfffffff0u clears bits
	 * 4:31, which overlapped with bit 4 of the size field and
	 * silently clobbered bit 4 every time the address portion was
	 * written. Concretely: with size_field = 17 (= 0b10001 for
	 * 4 GiB), the subsequent address-write cleared bit 4 leaving 1
	 * = 1 MiB encoding. Use the proper bits-12:31 mask. */
	value &= ~0xfffff000u;
	value |= LOWER_32_BITS(pcieAddr) & 0xfffff000u;
	writeReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_LO, value);
	writeReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_HI, UPPER_32_BITS(pcieAddr));

	/* USB-FIX-15 (2026-05-26): enable UBUS-side BAR2 access. Without
	 * this bit set, the BCM2711 UBUS interconnect drops every inbound
	 * DMA TLP from PCIe devices targeting the BAR2 window -- even
	 * though RC_BAR2_CONFIG_LO/HI advertise a correctly sized window
	 * on the PCIe side. Linux's brcm_pcie_setup() always sets this
	 * after programming the PCIe-side decoder. On BCM2711 (unlike
	 * BCM2712) the REMAP register has no address-base field -- it is
	 * a pure enable, and the inbound PCIe address goes to the same
	 * CPU PA via the SCB. */
	{
		uint32_t remap = readReg(ctx->base, BCM2711_PCIE_UBUS_BAR2_REMAP_LO);
		remap |= BCM2711_PCIE_UBUS_BAR2_ACCESS_EN;
		writeReg(ctx->base, BCM2711_PCIE_UBUS_BAR2_REMAP_LO, remap);
	}

	/* USB-FIX-11 (2026-05-26): read RC_BAR2 back and report via
	 * debug() so we can confirm the inbound DMA window is actually
	 * programmed. If the readback doesn't match what we wrote, the
	 * bridge isn't accepting the configuration and VL805's inbound
	 * DMA reads fail at the bridge level. */
	{
		uint32_t lo_rb = readReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_LO);
		uint32_t hi_rb = readReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_HI);
		uint32_t remap_lo = readReg(ctx->base, BCM2711_PCIE_UBUS_BAR2_REMAP_LO);
		char dbgbuf[160];
		snprintf(dbgbuf, sizeof(dbgbuf),
			"pcie: RC_BAR2 LO=0x%08x HI=0x%08x sz=0x%x  UBUS_REMAP=0x%08x (EN=%u)\n",
			lo_rb, hi_rb,
			(unsigned)(lo_rb & BCM2711_PCIE_RC_BAR2_SIZE_MASK),
			remap_lo,
			(unsigned)(remap_lo & BCM2711_PCIE_UBUS_BAR2_ACCESS_EN));
		debug(dbgbuf);
	}
}


static void bcm2711ShapeRootBridge(pcie_bcm2711_ctx_t *ctx)
{
	uint32_t value = readReg(ctx->base, BCM2711_PCIE_RC_CFG_PRIV1_ID_VAL3);

	value &= ~BCM2711_PCIE_RC_CLASS_CODE_MASK;
	value |= BCM2711_PCIE_RC_BRIDGE_CLASS_CODE;
	writeReg(ctx->base, BCM2711_PCIE_RC_CFG_PRIV1_ID_VAL3, value);
}


static void bcm2711ExposeDownstreamBridge(pcie_bcm2711_ctx_t *ctx)
{
	uint32_t buses = bcm2711RootRead32(ctx, PCI_PRIMARY_BUS);
	uint16_t command = bcm2711RootRead16(ctx, PCI_COMMAND);
	uint16_t bridgeControl = bcm2711RootRead16(ctx, PCI_BRIDGE_CONTROL);
	uint16_t rootControl;

	bcm2711RootWrite8(ctx, PCI_CACHE_LINE_SIZE, 64u / 4u);

	buses &= 0xff000000u;
	buses |= 0x00010100u;
	bcm2711RootWrite32(ctx, PCI_PRIMARY_BUS, buses);

	bcm2711RootWrite16(ctx, PCI_MEMORY_BASE, (uint16_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE >> 16));
	bcm2711RootWrite16(ctx, PCI_MEMORY_LIMIT, (uint16_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE >> 16));

	bridgeControl |= PCI_BRIDGE_CTL_PARITY;
	bcm2711RootWrite16(ctx, PCI_BRIDGE_CONTROL, bridgeControl);

	if (bcm2711RootRead8(ctx, BCM2711_PCIE_CAP_REGS + PCI_CAP_LIST_ID) == PCI_CAP_ID_EXP) {
		rootControl = bcm2711RootRead16(ctx, BCM2711_PCIE_CAP_REGS + PCI_EXP_RTCTL);
		rootControl |= PCI_EXP_RTCTL_CRSSVE;
		bcm2711RootWrite16(ctx, BCM2711_PCIE_CAP_REGS + PCI_EXP_RTCTL, rootControl);

		/* USB-FIX-8 instrumentation: print RC PCIe Cap MPS/MRRS. */
		{
			uint32_t devcap = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x04);
			uint32_t devctl = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x08);
			char dbgbuf[128];
			snprintf(dbgbuf, sizeof(dbgbuf),
				"pcie: RC PCIe Cap @0xAC DCAP=0x%08x DCTL=0x%08x\n",
				devcap, devctl);
			debug(dbgbuf);

			/* USB-FIX-9 (2026-05-26): clear NO_SNOOP_EN (bit 11) in
			 * RC's DCTL. BCM2711 PCIe is not cache-coherent by
			 * default; NoSnoop TLPs bypass CPU caches, so an
			 * inbound DMA fetch can read stale DRAM if the most
			 * recent CPU write is still in a dirty cache line. */
			bcm2711RootWrite16(ctx, BCM2711_PCIE_CAP_REGS + 0x08,
				(uint16_t)((devctl & 0xFFFFu) & ~0x0800u));

			/* USB-FIX-10: clear RC Device Status sticky bits
			 * (bits 0..4: CED, NFED, FED, URD, AUX_PWR_DET). They
			 * are RW1C; write the mask to clear. */
			bcm2711RootWrite16(ctx, BCM2711_PCIE_CAP_REGS + 0x0A, 0x001Fu);

			{
				uint32_t devctl_post = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x08);
				snprintf(dbgbuf, sizeof(dbgbuf),
					"  RC DCTL post-fix=0x%08x (NoSnoop+sticky cleared)\n",
					devctl_post);
				debug(dbgbuf);
			}

			/* USB-FIX-11: read Link Control (PCIe Cap offset 0x10),
			 * print current ASPM state, then disable ASPM (bits[1:0]
			 * = 00). If VL805's link is in L1 when we issue R/S=1,
			 * coming out of L1 takes time and the first DMA can
			 * timeout. Linux disables ASPM on VL805 via PCI quirk. */
			{
				uint32_t lnkcap = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x0C);
				uint32_t lnkctl = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x10);
				snprintf(dbgbuf, sizeof(dbgbuf),
					"  RC LNKCAP=0x%08x LNKCTL=0x%08x (ASPM=%u)\n",
					lnkcap, lnkctl, (unsigned)(lnkctl & 0x3u));
				debug(dbgbuf);
				bcm2711RootWrite16(ctx, BCM2711_PCIE_CAP_REGS + 0x10,
					(uint16_t)((lnkctl & 0xFFFFu) & ~0x0003u));
				{
					uint32_t lnkctl_post = bcm2711RootRead32(ctx, BCM2711_PCIE_CAP_REGS + 0x10);
					snprintf(dbgbuf, sizeof(dbgbuf),
						"  RC LNKCTL post-fix=0x%08x (ASPM disabled)\n",
						lnkctl_post);
					debug(dbgbuf);
				}
			}
		}
	}

	command |= PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE |
		PCI_CMD_PARITY_ERR_ENABLE | PCI_CMD_SERR_ERR_ENABLE;
	bcm2711RootWrite16(ctx, PCI_COMMAND, command);
}


static uint32_t bcm2711Read32(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	pcie_bcm2711_ctx_t *bcm = (pcie_bcm2711_ctx_t *)ctx;

	if (bus == 0) {
		if ((dev != 0) || (fn != 0)) {
			return UINT32_MAX;
		}

		return *bcm2711RootCfgPtr(bcm, off);
	}

	if (!bcm->linkUp || !bcm->rcMode) {
		return UINT32_MAX;
	}

	writeReg(bcm->base, BCM2711_PCIE_EXT_CFG_INDEX, bcm2711_cfgIndex(bus, dev, fn, off));

	return readReg(bcm->base, BCM2711_PCIE_EXT_CFG_DATA + (off & ~0x3));
}


static void bcm2711Write32(void *ctx, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	pcie_bcm2711_ctx_t *bcm = (pcie_bcm2711_ctx_t *)ctx;

	if (bus == 0) {
		if ((dev != 0) || (fn != 0)) {
			return;
		}

		*bcm2711RootCfgPtr(bcm, off) = val;
		return;
	}

	if (!bcm->linkUp || !bcm->rcMode) {
		return;
	}

	writeReg(bcm->base, BCM2711_PCIE_EXT_CFG_INDEX, bcm2711_cfgIndex(bus, dev, fn, off));
	writeReg(bcm->base, BCM2711_PCIE_EXT_CFG_DATA + (off & ~0x3), val);
}


static void pcie_cfgDestroyBcm2711(void *ctx)
{
	pcie_bcm2711_ctx_t *bcm = (pcie_bcm2711_ctx_t *)ctx;

	if (bcm == NULL) {
		return;
	}

	if (bcm->base != MAP_FAILED) {
		munmap((void *)bcm->base, PCIE_BCM2711_HOST_SIZE);
	}

	free(bcm);
}


static int pcie_cfgInitBcm2711(pcie_cfgio_t *cfgio)
{
	pcie_bcm2711_ctx_t *bcm;

	bcm = calloc(1, sizeof(*bcm));
	if (bcm == NULL) {
		return -ENOMEM;
	}

	bcm->base = mmap(NULL, PCIE_BCM2711_HOST_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, PCIE_BCM2711_HOST_BASE);
	if (bcm->base == MAP_FAILED) {
		free(bcm);
		return -ENOMEM;
	}

	cfgio->ctx = bcm;
	cfgio->read32 = bcm2711Read32;
	cfgio->write32 = bcm2711Write32;
	cfgio->destroy = pcie_cfgDestroyBcm2711;

	bcm2711PrepareHostBridge(bcm);
	bcm2711PrepareLinkState(bcm);
	if (bcm->linkUp && bcm->rcMode) {
		bcm2711SetOutboundWindow0(bcm, PCIE_BCM2711_OUTBOUND_CPU_BASE,
			PCIE_BCM2711_OUTBOUND_PCIE_BASE, PCIE_BCM2711_OUTBOUND_SIZE);
		/* Disable inbound BAR1 explicitly before programming BAR2.
		 * U-Boot's brcm_pcie_probe does this — without it a stale
		 * BAR1 from start4.elf firmware can intercept VL805's DMA
		 * to system memory, producing HSE on the first R/S transition.
		 * Clearing the size-mask bits (low 5 bits) disables the BAR. */
		{
			uint32_t bar1 = readReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO);
			bar1 &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
			writeReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO, bar1);
			/* USB-FIX-16: also disable BAR3 (PCIe->SCB peripheral
			 * shadow). See header comment for rationale. */
			{
				uint32_t bar3 = readReg(bcm->base, BCM2711_PCIE_RC_BAR3_CONFIG_LO);
				bar3 &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
				writeReg(bcm->base, BCM2711_PCIE_RC_BAR3_CONFIG_LO, bar3);
			}
		}
		bcm2711SetRcBar2(bcm, 0u, 0x100000000ull);
		bcm2711ShapeRootBridge(bcm);
		bcm2711ExposeDownstreamBridge(bcm);
	}

	return EOK;
}

#endif


static void print_bars(pcie_cfgio_t *cfgio, uint8_t bus, uint8_t dev, uint8_t fn, uint8_t hdr)
{
	/* Choose number of bars depending on config type */
	const int bar_count = (hdr == 0x00) ? 6 : 2;

	/* Read all bars */
	for (int i = 0; i < bar_count; ++i) {

		uint32_t bar_low = cfgio->read32(cfgio->ctx, bus, dev, fn, PCI_BAR0 + i * 4);
		if (bar_low == 0) {
			continue;
		}

		if ((bar_low & 0x1) == 0x1) {
			printf("pcie: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3);
		}
		else {
			bool is_64_bit = (bar_low & 0x4) == 0x4;
			uint64_t addr = bar_low & ~0xf;

			if (is_64_bit) {
				uint32_t bar_high = cfgio->read32(cfgio->ctx, bus, dev, fn, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}
			printf("pcie: BAR%d MEM 0x%016llx (%s)\n",
					i, (unsigned long long)addr, is_64_bit ? "64-bit" : "32-bit");
		}
	}
}


static void print_capabilities(pcie_cfgio_t *cfgio, uint8_t bus, uint8_t dev, uint8_t fn)
{
	/* Check if there is capabilities list */
	uint16_t status = pcie_cfgRead16(cfgio, bus, dev, fn, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = pcie_cfgRead8(cfgio, bus, dev, fn, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = pcie_cfgRead8(cfgio, bus, dev, fn, ptr);
		uint8_t next = pcie_cfgRead8(cfgio, bus, dev, fn, ptr + 1);
		printf("pcie: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);
		if (next == 0) {
			break;
		}
		ptr = next;
	}
}


static void scanFunc(pcie_cfgio_t *cfgio, uint8_t bus, uint8_t *next_bus, uint8_t dev, uint8_t fun)
{
	/* Read information about device */
	uint16_t vendor = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_VENDOR_ID);
	uint16_t device = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_DEVICE_ID);
	uint32_t class24 = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_CLASSCODE);
	uint8_t classBase = class24 >> 24;
	uint8_t classSub = (class24 >> 16) & 0xff;
	uint8_t progIF = (class24 >> 8) & 0xff;
	uint8_t hdr = pcie_cfgRead8(cfgio, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f;

	printf("pcie: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vendor, device,
			classBase, classSub, progIF,
			hdr);

#if defined(PCI_EXPRESS_BCM2711_INDEXED_CFG) && defined(RPI_MAILBOX_BASE_ADDRESS) && defined(XHCI_BCM2711_PCIE_BUS) && defined(XHCI_BCM2711_PCIE_SLOT) && defined(XHCI_BCM2711_PCIE_FUNC) && defined(XHCI_BCM2711_PCI_CLASS_CODE)
	if ((bus == XHCI_BCM2711_PCIE_BUS) && (dev == XHCI_BCM2711_PCIE_SLOT) &&
		(fun == XHCI_BCM2711_PCIE_FUNC) && ((class24 >> 8) == XHCI_BCM2711_PCI_CLASS_CODE)) {
		/*
		 * USB-FIX-1 (2026-05-26): split PCI_COMMAND write into two
		 * stages around the firmware mailbox notify.
		 *
		 * Background: the previous form enabled both MEM_ENABLE
		 * (outbound CPU->VL805) and BUS_MASTER (inbound VL805->DRAM
		 * DMA) BEFORE the mailbox notify. That ordering disagrees
		 * with every reference (Linux pci-quirks.c::quirk_usb_early_handoff,
		 * Circle's bcmpciehostbridge.cpp::EnableDevice, NetBSD bwfm):
		 * they all enable MEM only pre-notify and defer BUS_MASTER
		 * until after BAR0 program + post-notify settle.
		 *
		 * Why the divergence matters: the mailbox handler
		 * (RPI_FIRMWARE_NOTIFY_XHCI_RESET) churns bridge-side
		 * registers. If BUS_MASTER is on during that churn, VL805
		 * can issue inbound DMA reads that complete against stale
		 * translations, leaving its internal DMA TLB inconsistent.
		 * The classic symptom is HSE on the first R/S=1 (mode B in
		 * our 10-cycle experiment) or "can't setup: -110"
		 * (raspberrypi/firmware#1617 — same error code).
		 *
		 * The original justification for setting BUS_MASTER first
		 * (BME-after-mailbox -> 0xdead capability reads) was a
		 * misdiagnosis: 0xdead came from the bridge-translation
		 * invalidation bug, not from BUS_MASTER ordering. That bug
		 * is fixed separately via the dev-0-only PCIe scan +
		 * keep-alive mmap.
		 *
		 * Stage 1 (here): MEM_ENABLE only; explicitly clear
		 * BUS_MASTER so any stale value can't bias the mailbox.
		 */
		{
			uint16_t cmd = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
			uint16_t want = (cmd | PCI_CMD_MEM_ENABLE) & ~(uint16_t)PCI_CMD_MASTER_ENABLE;
			cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_COMMAND, want);
			uint16_t rb = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
			char dbgbuf[80];
			snprintf(dbgbuf, sizeof(dbgbuf),
				"pcie: VL805 pre-mailbox CMD=0x%04x (MEM only, MASTER deferred)\n", rb);
			debug(dbgbuf);
			if ((rb & PCI_CMD_MEM_ENABLE) == 0) {
				debug("pcie: VL805 MEM_ENABLE did not stick\n");
			}
		}

		/*
		 * USB-FIX-2 (2026-05-26): check VL805 firmware version
		 * (PCI config offset 0x50). If non-zero, the EEPROM has
		 * already loaded firmware -- skip the mailbox to avoid
		 * raspberrypi/firmware#1617 (repeated XHCI_RESET ->
		 * "can't setup: -110"). Linux's rpi_firmware_init_vl805()
		 * does this exact check.
		 */
		uint32_t fw_ver_pre = cfgio->read32(cfgio->ctx, bus, dev, fun, 0x50);
		int skip_mailbox = (fw_ver_pre != 0u);
		{
			char dbgbuf[96];
			snprintf(dbgbuf, sizeof(dbgbuf),
				"pcie: VL805 fw_ver @0x50 = 0x%08x  %s\n",
				fw_ver_pre,
				skip_mailbox ? "(loaded, skip mailbox)" : "(zero, will notify)");
			debug(dbgbuf);
		}

		/* USB-FIX-8 (2026-05-26): read VL805's PCIe Capability MPS/MRRS
		 * fields. VL805 PCIe Cap is at config offset 0xC4 per the boot-time
		 * "pcie: CAP id 0x10 address 0xc4" print. */
		{
			uint32_t devcap = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x04);
			uint32_t devctl = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x08);
			char dbgbuf[128];
			snprintf(dbgbuf, sizeof(dbgbuf),
				"pcie: VL805 PCIe Cap @0xC4 DCAP=0x%08x DCTL=0x%08x\n",
				devcap, devctl);
			debug(dbgbuf);

			/* USB-FIX-9: clear NO_SNOOP_EN in VL805 DCTL (bit 11). */
			uint32_t devctl_new = (devctl & 0xFFFFu) & ~0x0800u;
			cfgio->write32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x08, devctl_new);

			/* USB-FIX-10: clear Device Status sticky bits (RW1C). */
			cfgio->write32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x08,
				devctl_new | (0x001Fu << 16));

			{
				uint32_t devctl_post = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x08);
				snprintf(dbgbuf, sizeof(dbgbuf),
					"  VL805 DCTL post-fix=0x%08x\n", devctl_post);
				debug(dbgbuf);
			}

			/* USB-FIX-11: VL805 ASPM disable via Link Control. */
			{
				uint32_t lnkcap = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x0C);
				uint32_t lnkctl = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x10);
				snprintf(dbgbuf, sizeof(dbgbuf),
					"  VL805 LNKCAP=0x%08x LNKCTL=0x%08x (ASPM=%u)\n",
					lnkcap, lnkctl, (unsigned)(lnkctl & 0x3u));
				debug(dbgbuf);
				cfgio->write32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x10,
					(lnkctl & 0xFFFFu) & ~0x0003u);
				{
					uint32_t lnkctl_post = cfgio->read32(cfgio->ctx, bus, dev, fun, 0xc4 + 0x10);
					snprintf(dbgbuf, sizeof(dbgbuf),
						"  VL805 LNKCTL post-fix=0x%08x\n", lnkctl_post);
					debug(dbgbuf);
				}
			}
		}

		int err = 0;
		if (!skip_mailbox) {
			err = bcm2711NotifyXhciReset(bus, dev, fun);
			if (err < 0) {
				fprintf(stderr, "pcie: xhci firmware notify failed: %d\n", err);
			}
		}
		/* TD-USB: VL805 firmware load is async after the mailbox reset
		 * call returns. Without an explicit wait, the next config-space
		 * writes and (especially) MMIO reads to BAR0 race the VL805
		 * boot ROM → firmware handoff and the BCM2711 PCIe bridge
		 * returns 0xdead-pattern for any register read until firmware
		 * is up. Linux's xhci-pci driver polls for the device to come
		 * out of CRS via PCIe Vendor ID readback. Mirror that pattern:
		 * after the mailbox reset, the bridge briefly returns CRS
		 * (vendor=0xffff) or stale 0xdead until VL805 firmware is up;
		 * the vendor ID stabilises at 0x1106 once it's ready.
		 *
		 * Empirically the firmware load takes ~30–80 ms on a cold
		 * boot. The previous fixed 200 ms wait wasted at least half
		 * of that. Cap at 300 ms so a broken VL805 still bails out. */
		{
			int polls;
			uint16_t v = 0;
			for (polls = 0; polls < 60; polls++) {
				usleep(5000);
				v = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_VENDOR_ID);
				if (v == 0x1106) {
					break;
				}
			}
			if (v != 0x1106) {
				fprintf(stderr,
					"pcie: VL805 firmware did not come up after %d ms (vendor=0x%04x)\n",
					polls * 5, v);
			}
		}
		/* TD-15 Stage 4 phase 2: program VL805 BAR0 to the outbound
		 * window's PCIe base address. The bcm2711NotifyXhciReset
		 * mailbox call resets the VL805 and reloads its firmware, but
		 * does NOT program BAR0 — that's the OS's job. Linux's
		 * brcmstb pcie driver does standard PCI BAR allocation; here
		 * we hardcode the assignment to PCIE_BCM2711_OUTBOUND_PCIE_BASE
		 * (a single device fits in the outbound window so no
		 * allocator is needed). The CPU-side mapping at
		 * XHCI_BCM2711_MMIO_BASE = PCIE_BCM2711_OUTBOUND_CPU_BASE
		 * already routes through the bridge to this PCIe-side
		 * address.
		 *
		 * VL805 has a single 64-bit MEM BAR (BAR0). Write the low and
		 * high words; the device preserves the type-bit (BAR0[2]=1
		 * for 64-bit) on read-back. */
		cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_BAR0,
			(uint32_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE & 0xfffffff0u));
		cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_BAR0 + 4,
			(uint32_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE >> 32));
		{
			uint32_t bar_lo = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_BAR0);
			uint32_t bar_hi = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_BAR0 + 4);
			uint32_t want_lo = (uint32_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE & 0xfffffff0u);
			uint32_t want_hi = (uint32_t)(PCIE_BCM2711_OUTBOUND_PCIE_BASE >> 32);
			if (((bar_lo & 0xfffffff0u) != want_lo) || (bar_hi != want_hi)) {
				fprintf(stderr, "pcie: VL805 BAR0 programming failed: got lo=%08x hi=%08x wanted lo=%08x hi=%08x\n",
					bar_lo, bar_hi, want_lo, want_hi);
			}
		}
		/* TD-USB diag 2026-05-16: read xhci CAPLENGTH + HCIVERSION
		 * directly through the outbound window. Confirms the path
		 * CPU PA 0x600000000 -> PCIe bus 0xf8000000 -> VL805 BAR0
		 * works.
		 *
		 * KEPT-ALIVE 2026-05-17: do NOT munmap. The BCM2711 PCIe
		 * bridge appears to invalidate its outbound translation
		 * entry when the kernel removes the user mapping. Leaving
		 * this mmap in place keeps the bridge translation warm so
		 * xhci's subsequent mmap of the same PA reads valid
		 * registers (otherwise the second mmap sees 0xdead).
		 */
		{
			/* Hand off the xhci MMIO mapping to xhci_init via the
			 * bcm2711_pcie_getXhciMmio() accessor. Historically this was
			 * a "keepalive" against a bridge-translation invalidation
			 * bug (where xhci's own later mmap of the outbound CPU PA
			 * read 0xdead); that turned out to be a side effect of
			 * pcie_scanBus probing dev 1..31 on bus 1 and is fixed by
			 * the dev-0-only sweep there. Pre-creating the mapping
			 * here is still useful because it avoids a redundant
			 * MAP_DEVICE mmap inside xhci_init. */
			/* Map the VL805 MMIO with MAP_UNCACHED in addition to
			 * MAP_DEVICE so the page uses Device-nGnRnE (strongly ordered,
			 * MAIR_IDX_S_ORDERED) rather than Device-nGnRE. nGnRnE is the
			 * correct, strongest ordering for PCIe device registers and
			 * matches the known-good lwip-port 'X' mapping of the same BAR.
			 * (Tested as a candidate for the usb-hcd "event writes never
			 * land" gap; it is NOT the fix, but the alignment is kept as a
			 * correctness improvement.) */
			volatile uint8_t *xhci_mmio = mmap(NULL,
				bcm2711_pcie_getXhciMmioSize(),
				PROT_READ | PROT_WRITE,
				MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS,
				-1, PCIE_BCM2711_OUTBOUND_CPU_BASE);
			if (xhci_mmio == MAP_FAILED) {
				fprintf(stderr, "pcie: xhci mmio mmap failed in scan callback\n");
			}
			else {
				bcm2711_pcie_xhciMmio = xhci_mmio;
			}
		}
	}
#endif

	/*
	 * Enable Memory Space and Bus Master.
	 *
	 * The previous form `if (!(cmd & (MSE|MASTER)))` only enabled the bits
	 * when BOTH were already clear; if exactly one was set, the other was
	 * never enabled. On the BCM2711 root + VL805, this manifested as MMIO
	 * writes to operational xHCI registers (DCBAAP, CRCR, CONFIG) being
	 * silently dropped after HCRST while capability reads still worked.
	 * Always set both bits and only write back if anything changed.
	 */
	{
		uint16_t cmd = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
		uint16_t want = cmd | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE;
		if (want != cmd) {
			cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_COMMAND, want);
		}
		uint16_t rb = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
		{
			char dbgbuf[96];
			snprintf(dbgbuf, sizeof(dbgbuf),
				"pcie: %02x:%02x.%u final CMD=0x%04x (MEM=%u MASTER=%u)\n",
				bus, dev, fun, rb,
				(unsigned)(!!(rb & PCI_CMD_MEM_ENABLE)),
				(unsigned)(!!(rb & PCI_CMD_MASTER_ENABLE)));
			debug(dbgbuf);
		}
	}

	/* Print some info about this device */
	print_bars(cfgio, bus, dev, fun, hdr);
	print_capabilities(cfgio, bus, dev, fun);

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		uint8_t sec = pcie_cfgRead8(cfgio, bus, dev, fun, PCI_SECONDARY_BUS);
		if (sec == 0) {
			/* Bridge not yet configured, assign fresh numbers */
			sec = (*next_bus)++;
			uint32_t cur = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff000000;
			uint32_t val = cur |
					((uint32_t)0xFF << 16) |
					((uint32_t)sec << 8) |
					bus;
			cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_PRIMARY_BUS, val);
		}

		uint8_t sub = pcie_cfgRead8(cfgio, bus, dev, fun, PCI_SUBORDINATE_BUS);

		printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
				bus, sec, sub);

		pcie_scanBus(cfgio, sec);

		/* After recursion write the real highest bus number reached */
		uint32_t cur = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff00ffff;
		uint32_t val = cur | ((uint32_t)((*next_bus) - 1) << 16);
		cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_PRIMARY_BUS, val);
	}
}


static void pcie_scanBus(pcie_cfgio_t *cfgio, uint8_t bus)
{
	uint8_t next_bus = 1;

	/*
	 * On the BCM2711 root complex any non-root bus is downstream of a
	 * PCIe Express bridge, which is point-to-point: device 0 is the only
	 * legal device ID. Worse, on real Pi 4 reading PCI_VENDOR_ID at
	 * bus=1, dev=1, fn=0 silently TEARS DOWN the bridge's outbound
	 * window translation for the duration of the process (observed:
	 * BAR0[0..3] reads as 0xdeaddead from that point on, blocking
	 * xhci_capProbe). Cap the per-bus device sweep at 1 on non-root
	 * buses to avoid that bridge-state corruption.
	 *
	 * The original code already capped the root bus (bus == 0) at one
	 * device because the BCM2711 root complex only exposes itself there;
	 * we extend the same rule to non-root buses for the reason above.
	 */
	uint8_t max_dev = 1;
	for (uint8_t dev = 0; dev < max_dev; ++dev) {
		/**
		 * In case there is no device under certain identifier the bridge
		 * returns all "ones" (0xffff) per PCIe spec, but the BCM2711
		 * root complex returns all-zeros (0x0000) instead on empty
		 * slots. Treat both as "no device" so we don't proceed into
		 * scanFunc() for non-existent devices — doing so writes the
		 * Command register and reads BARs against the bridge's
		 * unmapped slot decoder, which in turn (observed on real Pi 4)
		 * pushes the bridge into a state where subsequent VL805 reads
		 * also return zero, breaking xhci_capProbe.
		 */
		uint16_t vendor_id = pcie_cfgRead16(cfgio, bus, dev, 0, PCI_VENDOR_ID);
		if (vendor_id == 0xffff || vendor_id == 0x0000) {
			continue;
		}

		/* Scan first function of device */
		scanFunc(cfgio, bus, &next_bus, dev, 0);

		/* Check if this is multi function device and scan them */
		bool multi = pcie_cfgRead8(cfgio, bus, dev, 0, PCI_HEADER_TYPE) & PCI_HT_MULTI_FUNC;
		if (multi) {
			printf("pcie: multiple func device %u\n", dev);
			for (uint8_t fn = 1; fn < 8; fn++) {
				vendor_id = pcie_cfgRead16(cfgio, bus, dev, fn, PCI_VENDOR_ID);
				if (vendor_id == 0xffff || vendor_id == 0x0000) {
					continue;
				}
				scanFunc(cfgio, bus, &next_bus, dev, fn);
			}
		}
	}
}


/*
 * bcm2711_pcie_initVL805 — BCM2711 PCIe bridge + VL805 USB controller bring-up.
 *
 * Refactored from the standalone `pcie` daemon's `main()`. Now called from
 * inside the `usb` daemon's xhci PHY init so that the entire USB bring-up
 * (bridge config + BAR0 programming + xHCI register access) happens in a
 * single process, matching the canonical Phoenix-RTOS pattern used on every
 * other supported board (imx6ull, imxrt106x/117x, ia32) where one process
 * owns both the bus-side init and the host-controller driver.
 *
 * Two-process design (with `pcie` as a separate fire-and-exit daemon) failed
 * because the BCM2711 PCIe bridge's outbound-window translation is anchored
 * per-process / per-mmap; `xhci` in a separate user process never sees the
 * valid xhci-capability registers that `pcie`'s own mmap could read. Merging
 * the two eliminates the cross-process race entirely.
 *
 * Returns EOK on success, negative errno on failure. After this function
 * returns, the caller can mmap `XHCI_BCM2711_MMIO_BASE` and read xhci CAP
 * space (CAPLENGTH = 0x20, HCIVERSION = 0x0100, …) reliably.
 */
int bcm2711_pcie_initVL805(void)
{
	pcie_cfgio_t cfgio = { 0 };
	int ret = 0;

#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG
	/* Pi 4 PoC drive-only path (2026-05-28): when this process is hosting
	 * the USB host stack inside lwip-port (which sets USB_HCD_PCIE_DRIVE_ONLY
	 * before calling usb_init), an earlier boot-time `usb` daemon instance
	 * already performed the one-shot BCM2711 PCIe bridge bring-up — that
	 * state persists in bridge HW after the boot-time process exits. Skip
	 * the bring-up here so this drive-only instance never PERSTs the bridge
	 * or does the in-process bridge config that empirically (on BCM2711)
	 * leaves THIS process's subsequent xHCI inbound DMA writes silently
	 * lost. bcm2711_pcie_getXhciMmio() stays NULL on this path, so xhci_map
	 * falls back to a fresh MAP_PHYSMEM mapping of the controller MMIO —
	 * exactly the known-good drive-only path the diag-udp 'X' rig uses.
	 * The probe mapping is intentionally held (not unmapped) on the skip
	 * path: holding the bridge mapping is harmless and avoids the
	 * munmap-invalidates-translation hazard noted at the bottom of this
	 * function. */
	if (getenv("USB_HCD_PCIE_DRIVE_ONLY") != NULL) {
		volatile uint8_t *probe = mmap(NULL, PCIE_BCM2711_HOST_SIZE,
			PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1, PCIE_BCM2711_HOST_BASE);
		if (probe != MAP_FAILED) {
			uint32_t linkmask = BCM2711_PCIE_MISC_STATUS_PCIE_DL_ACTIVE_MASK |
				BCM2711_PCIE_MISC_STATUS_PCIE_PHYLINKUP_MASK;
			uint32_t st = 0u;
			int i;
			for (i = 0; i < 1000; i++) { /* wait up to ~10 s for the initializer */
				st = *(volatile uint32_t *)(probe + BCM2711_PCIE_MISC_STATUS);
				if ((st & linkmask) == linkmask) {
					break;
				}
				usleep(10000);
			}
			if ((st & linkmask) == linkmask) {
				char dbgbuf[128];
				snprintf(dbgbuf, sizeof(dbgbuf),
					"xhci-pcie: drive-only; link UP (MISC_STATUS=0x%08x) after %d iters\n",
					(unsigned)st, i);
				debug(dbgbuf);
				/* probe mapping intentionally held (leaked) — see comment
				 * above. */
				return EOK;
			}
			/* Link didn't come up within 10 s. Known Pi 4 silicon
			 * variability per Linux issue #5060 / rpi forums #380969:
			 * some VL805 instances fail PCIe link bring-up unless an
			 * RC delay between 3.3V and nPONRST is in place. This is
			 * hardware-level — software can only report and bail. */
			{
				char dbgbuf[128];
				snprintf(dbgbuf, sizeof(dbgbuf),
					"xhci-pcie: drive-only LINK DOWN after 10s (MISC_STATUS=0x%08x); aborting\n",
					(unsigned)st);
				debug(dbgbuf);
			}
			munmap((void *)probe, PCIE_BCM2711_HOST_SIZE);
			return -ENODEV;
		}
		/* Probe map failed; fall through to full init so a lone instance
		 * still has a chance to come up. */
	}
	ret = pcie_cfgInitBcm2711(&cfgio);
#else
	/* Non-BCM2711 boards reach this function only if their xhci PHY
	 * also opts in to merged bus init. Wire up ECAM fallback so the
	 * code path is portable. */
	ret = pcie_cfgInitEcam(&cfgio);
#endif
	if (ret != EOK) {
		fprintf(stderr, "xhci-pcie: fail to initialize config-space backend\n");
		return ret;
	}

	pcie_scanBus(&cfgio, 0);

#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG
	{
		/* The mailbox-driven xhci-reset issued during scanBus' VL805
		 * probe invalidates the bridge's outbound window translation
		 * we programmed in pcie_cfgInitBcm2711(). Re-program it now so
		 * xhci's later reads of the outbound CPU PA return real BAR0
		 * data instead of 0xdead poison. */
		pcie_bcm2711_ctx_t *bcm = (pcie_bcm2711_ctx_t *)cfgio.ctx;
		if ((bcm != NULL) && (bcm->base != MAP_FAILED) && bcm->linkUp && bcm->rcMode) {
			bcm2711SetOutboundWindow0(bcm, PCIE_BCM2711_OUTBOUND_CPU_BASE,
				PCIE_BCM2711_OUTBOUND_PCIE_BASE, PCIE_BCM2711_OUTBOUND_SIZE);
			/* 2026-05-24: also re-disable BAR1 and re-program BAR2.
			 * The NOTIFY_XHCI_RESET mailbox can churn bridge-side
			 * registers; without this re-program, inbound DMA may
			 * fail (USBSTS.HSE on first R/S=1). */
			{
				uint32_t bar1 = readReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO);
				bar1 &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
				writeReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO, bar1);
			}
			bcm2711SetRcBar2(bcm, 0u, 0x100000000ull);
			/* Stash the context so bcm2711_pcie_resettleOutboundWindow
			 * can replay this same write after the controller's HCRST
			 * has invalidated the bridge translation a second time. */
			bcm2711_pcie_lastCtx = bcm;
		}

		/* Post-re-program settling window. Empirical hardware sweep
		 * across cold boots of the BCM2711 bridge after mailbox-notify:
		 *
		 *   wait | rc=-19 (poison) | rc=-110 (reset timeout)
		 *   -----|-----------------|------------------------
		 *     0  | 1/3             | 2/3
		 *    50  | 1/3             | 2/3
		 *   200  | 3/3             | 0/3  (worse — bridge drifted)
		 *   500  | (anticipated) similarly bad
		 *
		 * 50 ms is the sweet spot: the bridge translation has time to
		 * propagate but doesn't have enough idle time to be invalidated
		 * by something else (the start4.elf firmware periodically
		 * touches PCIe; we see "PCI0 reset" again at firmware-time
		 * ~41.8 s on every boot). Keep at 50 ms. */
		usleep(50000);
	}
#endif

	/* AXI ordering: make sure every config-space and bridge-register
	 * write issued from scanBus + scanFunc is globally visible BEFORE
	 * the caller (xhci PHY init / xhci_init) starts reading from the
	 * outbound window. BCM2711 peripheral access guidance (Pi
	 * peripherals datasheet §1.3) recommends an explicit barrier
	 * between distinct peripherals. */
	__asm__ volatile("dsb sy" ::: "memory");
	__asm__ volatile("isb" ::: "memory");

	/* INTENTIONALLY DO NOT DESTROY: on real Pi 4, calling
	 * cfgio.destroy(cfgio.ctx) — which munmaps PCIE_BCM2711_HOST_BASE
	 * (the bridge config-register window) — invalidates the bridge's
	 * outbound window translation that bcm2711SetOutboundWindow0()
	 * just programmed. xhci_init's subsequent reads through the
	 * outbound window then return 0xdead poison. Leaking ~64 KiB of
	 * VA + the host-bridge mapping is the pragmatic workaround until
	 * the kernel pmap can refcount MAP_DEVICE mappings of the bridge
	 * registers across our process's mapping churn. The merged
	 * usb+pcie process lives until shutdown anyway, so the leak is
	 * bounded. */
	(void)cfgio;

	return ret;
}


/* Re-program the BCM2711 PCIe root complex's outbound window 0 with the
 * same parameters bcm2711_pcie_initVL805 used at boot. Idempotent and
 * cheap (a handful of MMIO writes to the host bridge). Callable any
 * time after the one-shot bridge bring-up; intended for the xHCI driver
 * to invoke between writing HCRST and waiting for the bit to clear,
 * because the controller's reset sequence appears to invalidate the
 * bridge translation the same way the firmware-load mailbox notify
 * does. Returns EOK on success, -ENODEV if the bridge context was
 * never set (i.e. boot-time bring-up didn't reach the IF block where
 * lastCtx is stashed — happens on non-BCM2711 builds or when the
 * bridge link never came up). */
int bcm2711_pcie_resettleOutboundWindow(void)
{
#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG
	pcie_bcm2711_ctx_t *bcm = bcm2711_pcie_lastCtx;
	uint32_t bar1;
	if ((bcm == NULL) || (bcm->base == MAP_FAILED) || !bcm->linkUp || !bcm->rcMode) {
		return -ENODEV;
	}
	bcm2711SetOutboundWindow0(bcm, PCIE_BCM2711_OUTBOUND_CPU_BASE,
		PCIE_BCM2711_OUTBOUND_PCIE_BASE, PCIE_BCM2711_OUTBOUND_SIZE);
	/* Inbound DMA window can also be churned by intermediate bridge-
	 * affecting events (xhci HCRST, mailbox notify). Re-disable BAR1
	 * and re-program BAR2 to 4 GiB to ensure VL805 DMA reads still
	 * reach system memory after this call returns. */
	bar1 = readReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO);
	bar1 &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
	writeReg(bcm->base, BCM2711_PCIE_RC_BAR1_CONFIG_LO, bar1);
	/* USB-FIX-16: also disable BAR3. */
	{
		uint32_t bar3 = readReg(bcm->base, BCM2711_PCIE_RC_BAR3_CONFIG_LO);
		bar3 &= ~BCM2711_PCIE_RC_BAR2_SIZE_MASK;
		writeReg(bcm->base, BCM2711_PCIE_RC_BAR3_CONFIG_LO, bar3);
	}
	bcm2711SetRcBar2(bcm, 0u, 0x100000000ull);
	__asm__ volatile("dsb sy" ::: "memory");
	__asm__ volatile("isb" ::: "memory");
	return EOK;
#else
	return -ENODEV;
#endif
}
