/*
 * Phoenix-RTOS
 *
 * xHCI Pi 4 discovery stub
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */


#include <hcd.h>

#include <board_config.h>


static const hcd_info_t xhci_info[] = {
	{
		.type = "xhci",
		.hcdaddr = XHCI_BCM2711_MMIO_BASE,
		.irq = 0,
		.pci_devId = {
			.bus = XHCI_BCM2711_PCIE_BUS,
			.dev = XHCI_BCM2711_PCIE_SLOT,
			.func = XHCI_BCM2711_PCIE_FUNC
		}
	}
};


int hcd_getInfo(const hcd_info_t **info)
{
	*info = xhci_info;

	return sizeof(xhci_info) / sizeof(xhci_info[0]);
}
