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
	volatile uint32_t *mailbox;
	uint32_t *msgbuf;
	uint32_t msg;
	uintptr_t msgaddr;
	int ret;
	{
		extern void debug(const char *s);
		debug("pcie: bcm2711NotifyXhciReset enter\n");
	}

	mailbox = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, RPI_MAILBOX_BASE_ADDRESS);
	if (mailbox == MAP_FAILED) {
		return -ENOMEM;
	}

	msgbuf = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msgbuf == MAP_FAILED) {
	        munmap((void *)mailbox, _PAGE_SIZE);
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
	        munmap((void *)mailbox, _PAGE_SIZE);
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
	{
		extern void debug(const char *s);
		char m[80];
		snprintf(m, sizeof(m), "pcie: notifyXhciReset ret=%d resp=%08x\n", ret, msgbuf[1]);
		debug(m);
	}

	munmap(msgbuf, _PAGE_SIZE);
	munmap((void *)mailbox, _PAGE_SIZE);

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
#define BCM2711_PCIE_RC_BAR2_CONFIG_LO    0x4034u
#define BCM2711_PCIE_RC_BAR2_CONFIG_HI    0x4038u
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
	usleep(200);

	bcm2711BridgeSwInitSet(ctx, 0u);
	writeRegMsk(ctx->base, BCM2711_PCIE_HARD_DEBUG,
		BCM2711_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK, 0u);
	usleep(200);

	(void)readReg(ctx->base, BCM2711_PCIE_MISC_REVISION);

	misc = readReg(ctx->base, BCM2711_PCIE_MISC_CTRL);
	misc |= BCM2711_PCIE_MISC_CTRL_SCB_ACCESS_EN;
	misc |= BCM2711_PCIE_MISC_CTRL_CFG_READ_UR_MODE;
	misc &= ~BCM2711_PCIE_MISC_CTRL_MAX_BURST_MASK;
	writeReg(ctx->base, BCM2711_PCIE_MISC_CTRL, misc);
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
	bcm2711PerstSet(ctx, 0u);
	usleep(100000);

	ctx->linkUp = bcm2711LinkUp(ctx);
	ctx->rcMode = bcm2711RcMode(ctx);
	{
		extern void debug(const char *s);
		char m[64];
		snprintf(m, sizeof(m), "pcie: linkUp=%d rcMode=%d\n", ctx->linkUp, ctx->rcMode);
		debug(m);
	}
}


static uint32_t bcm2711EncodeBar2Size(uint64_t size)
{
	unsigned shift = 20;
	uint64_t value = size;

	while ((value > 1u) && ((value & 1u) == 0u)) {
		value >>= 1;
		shift++;
	}

	if ((size == 0u) || (value != 1u) || (shift < 15u)) {
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
	value &= ~0xfffffff0u;
	value |= LOWER_32_BITS(pcieAddr) & 0xfffffff0u;
	writeReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_LO, value);
	writeReg(ctx->base, BCM2711_PCIE_RC_BAR2_CONFIG_HI, UPPER_32_BITS(pcieAddr));
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
		{
			extern void debug(const char *s);
			char m[80];
			snprintf(m, sizeof(m), "pcie: BAR%d raw=%08x\n", i, bar_low);
			debug(m);
		}
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
	{
		extern void debug(const char *s);
		char m[120];
		snprintf(m, sizeof(m), "pcie: %02x:%02x.%u ven=%04x dev=%04x cls=%02x%02x%02x hdr=%02x\n",
			bus, dev, fun, vendor, device, classBase, classSub, progIF, hdr);
		debug(m);
	}

#if defined(PCI_EXPRESS_BCM2711_INDEXED_CFG) && defined(RPI_MAILBOX_BASE_ADDRESS) && defined(XHCI_BCM2711_PCIE_BUS) && defined(XHCI_BCM2711_PCIE_SLOT) && defined(XHCI_BCM2711_PCIE_FUNC) && defined(XHCI_BCM2711_PCI_CLASS_CODE)
	if ((bus == XHCI_BCM2711_PCIE_BUS) && (dev == XHCI_BCM2711_PCIE_SLOT) &&
		(fun == XHCI_BCM2711_PCIE_FUNC) && ((class24 >> 8) == XHCI_BCM2711_PCI_CLASS_CODE)) {
		/*
		 * Enable Memory Space and Bus Master BEFORE the firmware mailbox
		 * notify and BAR programming. Empirically, enabling BME *after*
		 * the firmware reset leaves VL805 in a state where capability
		 * reads return 0xdead (no completion) even though the BAR is
		 * programmed and the cmd register read-back shows 0x0006.
		 */
		{
			extern void debug(const char *s);
			char m[80];
			uint16_t cmd = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
			uint16_t want = cmd | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE;
			cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_COMMAND, want);
			uint16_t rb = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
			snprintf(m, sizeof(m), "pcie: VL805 cmd %04x->%04x rb=%04x\n", cmd, want, rb);
			debug(m);
		}
		int err = bcm2711NotifyXhciReset(bus, dev, fun);
		if (err < 0) {
			fprintf(stderr, "pcie: xhci firmware notify failed: %d\n", err);
		}
		/* TD-USB: VL805 firmware load is async after the mailbox
		 * reset call returns. Without an explicit wait, the next
		 * config-space writes and (especially) MMIO reads to BAR0
		 * race the VL805 boot ROM → firmware handoff and the
		 * BCM2711 PCIe bridge returns 0xdead-pattern for any
		 * register read until firmware is up. Empirically a 200 ms
		 * settle is enough to make xhci_capProbe see valid
		 * caplen / version values on the first try.
		 *
		 * Reference: Linux's xhci-pci driver waits for the device
		 * to come out of CRS (Configuration Retry Status) via the
		 * PCIe Vendor ID polling pattern; we don't have CRS-aware
		 * helpers in this codebase yet so a simple usleep is the
		 * pragmatic fix. Future cleanup should poll for stable
		 * Vendor ID / caplen reads instead of a fixed delay.
		 */
		usleep(200000);
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
			extern void debug(const char *s);
			char m[80];
			uint32_t bar_lo = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_BAR0);
			uint32_t bar_hi = cfgio->read32(cfgio->ctx, bus, dev, fun, PCI_BAR0 + 4);
			snprintf(m, sizeof(m), "pcie: VL805 BAR0 programmed lo=%08x hi=%08x\n", bar_lo, bar_hi);
			debug(m);
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
			extern void debug(const char *s);
			static volatile uint8_t *vl805_mmio_keepalive;
			vl805_mmio_keepalive = mmap(NULL, _PAGE_SIZE,
				PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
				-1, PCIE_BCM2711_OUTBOUND_CPU_BASE);
			if (vl805_mmio_keepalive == MAP_FAILED) {
				debug("pcie: diag-mmap of outbound window FAILED\n");
			}
			else {
				char m[120];
				uint8_t cl = *vl805_mmio_keepalive;
				uint16_t ver = *(volatile uint16_t *)(vl805_mmio_keepalive + 2);
				uint32_t hcsp1 = *(volatile uint32_t *)(vl805_mmio_keepalive + 4);
				snprintf(m, sizeof(m),
					"pcie: diag-outbound caplen=%02x ver=%04x hcsparams1=%08x (KEPT)\n",
					cl, ver, hcsp1);
				debug(m);
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
			printf("pcie: enable memory space and bus master (cmd %04x->%04x)\n", cmd, want);
			cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_COMMAND, want);
			{
				extern void debug(const char *s);
				char m[80];
				uint16_t rb = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
				snprintf(m, sizeof(m), "pcie: cmd readback %04x (wanted %04x)\n", rb, want);
				debug(m);
			}
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

	{
		char m[48];
		extern void debug(const char *s);
		snprintf(m, sizeof(m), "pcie-scanBus: bus=%u enter\n", bus);
		debug(m);
	}

	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < (bus ? 32 : 1); ++dev) {
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

	{
		char m[48];
		extern void debug(const char *s);
		snprintf(m, sizeof(m), "pcie-scanBus: bus=%u exit\n", bus);
		debug(m);
	}
}


/* TD-15 Stage 4 phase 2 DIAGNOSTIC: pcie daemon doesn't print to UART
 * by default (uses fprintf which is buffered). Use debug() for direct
 * kernel klog → UART output so we can see what's happening on real
 * Pi 4. Remove once VL805 BAR-programming is fixed. */
#include <sys/debug.h>

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
	debug("xhci-pcie: bcm2711_pcie_initVL805 enter\n");

#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG
	debug("xhci-pcie: pre-cfgInitBcm2711\n");
	ret = pcie_cfgInitBcm2711(&cfgio);
	debug("xhci-pcie: post-cfgInitBcm2711\n");
#else
	/* Non-BCM2711 boards reach this function only if their xhci PHY
	 * also opts in to merged bus init. Wire up ECAM fallback so the
	 * code path is portable. */
	ret = pcie_cfgInitEcam(&cfgio);
#endif
	if (ret != EOK) {
		fprintf(stderr, "xhci-pcie: fail to initialize config-space backend\n");
		debug("xhci-pcie: cfgInit FAIL\n");
		return ret;
	}

	debug("xhci-pcie: pre-scanBus\n");
	pcie_scanBus(&cfgio, 0);
	debug("xhci-pcie: post-scanBus\n");

	/* AXI ordering: make sure every config-space and bridge-register
	 * write issued from scanBus + scanFunc is globally visible BEFORE
	 * the caller (xhci PHY init / xhci_init) starts reading from the
	 * outbound window. BCM2711 peripheral access guidance (Pi
	 * peripherals datasheet §1.3) recommends an explicit barrier
	 * between distinct peripherals. */
	__asm__ volatile("dsb sy" ::: "memory");
	__asm__ volatile("isb" ::: "memory");
	debug("xhci-pcie: post-barrier\n");

	cfgio.destroy(cfgio.ctx);
	debug("xhci-pcie: post-cfgio.destroy\n");
	debug("xhci-pcie: bcm2711_pcie_initVL805 done\n");

	return ret;
}
