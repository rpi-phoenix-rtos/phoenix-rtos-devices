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
#include <sys/mman.h>
#include <unistd.h>

#include <hcd.h>


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
#define XHCI_SUPPORTED_VERSION   0x0100u
#define XHCI_REG_OP_USBCMD_HCRST (1u << 1)
#define XHCI_REG_OP_USBSTS_CNR   (1u << 11)
#define XHCI_REG_OP_PAGESIZE_4K  (1u << 0)
#define XHCI_REG_OP_CRCR_RCS     (1u << 0)
#define XHCI_REG_OP_CRCR_CS      (1u << 1)
#define XHCI_REG_OP_CRCR_CA      (1u << 2)
#define XHCI_REG_OP_CRCR_CRR     (1u << 3)
#define XHCI_REG_OP_CRCR_CR_PTR_LO__MASK 0xffffffc0u
#define XHCI_REG_OP_DCBAAP__MASK 0xffffffc0u
#define XHCI_CNR_TIMEOUT_MS      100u
#define XHCI_HCRST_TIMEOUT_MS    20u


typedef struct {
	void *mmio;
	size_t mapSz;
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
	uint64_t crcr;
	uint64_t dcbaap;
	unsigned ac64 : 1;
	unsigned spr : 1;
} xhci_t;


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


static void xhci_destroy(xhci_t *xhci)
{
	if (xhci == NULL) {
		return;
	}

	if (xhci->mmio != MAP_FAILED) {
		munmap(xhci->mmio, xhci->mapSz);
	}

	free(xhci);
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
				err = -ENOSYS;
			}
		}
	}

	if (err != 0) {
		xhci_destroy(xhci);
		hcd->priv = NULL;
		hcd->base = NULL;
	}

	return err;
}


static int xhci_transferEnqueue(hcd_t *hcd, usb_transfer_t *t, usb_pipe_t *pipe)
{
	(void)hcd;
	(void)t;
	(void)pipe;

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
	(void)hub;

	return 0u;
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
