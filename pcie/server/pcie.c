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

#include <pcie.h>


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
		int err = bcm2711NotifyXhciReset(bus, dev, fun);
		if (err < 0) {
			fprintf(stderr, "pcie: xhci firmware notify failed: %d\n", err);
		}
	}
#endif

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = pcie_cfgRead16(cfgio, bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE))) {
		printf("pcie: enable memory space and bus master\n");
		cfgio->write32(cfgio->ctx, bus, dev, fun, PCI_COMMAND, cmd | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE);
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

	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < (bus ? 32 : 1); ++dev) {
		/**
		 * In case there is no device under certain identifier the bridge
		 * returns all "ones" on read
		 */
		uint16_t vendor_id = pcie_cfgRead16(cfgio, bus, dev, 0, PCI_VENDOR_ID);
		if (vendor_id == 0xffff) {
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
				if (vendor_id == 0xffff) {
					continue;
				}
				scanFunc(cfgio, bus, &next_bus, dev, fn);
			}
		}
	}
}


int main(int argc, char **argv)
{
	pcie_cfgio_t cfgio = { 0 };
	int ret = 0;

#ifdef PCI_EXPRESS_INIT_TEBF0808_PHY
	ret = tebf0808_pcieRefClkInit();
	if (ret != 0) {
		return ret;
	}

	ret = tebf0808_pciePsGtrPhyInit();
	if (ret != 0) {
		return ret;
	}
#endif

#ifdef PCI_EXPRESS_XILINX_NWL
	ret = pcie_xilinx_nwl_init();
	if (ret != 0) {
		return ret;
	}
#endif

#ifdef PCI_EXPRESS_XILINX_AXI
	ret = pcie_xilinx_axi_init();
	if (ret != 0) {
		return ret;
	}
#endif

#ifdef PCI_EXPRESS_BCM2711_INDEXED_CFG
	ret = pcie_cfgInitBcm2711(&cfgio);
#else
	ret = pcie_cfgInitEcam(&cfgio);
#endif
	if (ret != EOK) {
		fprintf(stderr, "pcie: fail to initialize config-space backend\n");
		return ret;
	}

	pcie_scanBus(&cfgio, 0);
	cfgio.destroy(cfgio.ctx);

	return ret;
}
