/*
 * Phoenix-RTOS
 *
 * BCM2711 PCIe bridge bring-up for VL805 USB (xhci PHY)
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#ifndef BCM2711_PCIE_H_
#define BCM2711_PCIE_H_

#include <stdint.h>


#define UPPER_32_BITS(n) ((uint32_t)(((n) >> 16) >> 16))
#define LOWER_32_BITS(n) ((uint32_t)((n) & 0xffffffff))


/* ECAM commands */
#define PCI_CMD_IO_ENABLE         0x01
#define PCI_CMD_MEM_ENABLE        0x02
#define PCI_CMD_MASTER_ENABLE     0x04
#define PCI_CMD_PARITY_ERR_ENABLE 0x40
#define PCI_CMD_SERR_ERR_ENABLE   0x100


static inline uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}


static inline void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}


static inline void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}


/*
 * Bring up the BCM2711 PCIe root complex, scan PCIe bus, locate VL805 USB
 * controller at bus 1 dev 0 fun 0, program its BAR0 to the outbound window's
 * PCIe-side base address, and (via firmware mailbox) reload the VL805
 * firmware. After this returns EOK, the xhci HCD can mmap the outbound
 * window CPU base (XHCI_BCM2711_MMIO_BASE) and read xhci capability
 * registers reliably.
 *
 * Called from the xhci PHY's phy_init() in a single-process design — the
 * USB daemon now does its own bus init in-process, matching the canonical
 * Phoenix-RTOS pattern.
 */
int bcm2711_pcie_initVL805(void);

#endif
