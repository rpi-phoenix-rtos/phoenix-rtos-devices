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

#include <hcd.h>


static int xhci_init(hcd_t *hcd)
{
	(void)hcd;

	fprintf(stderr, "xhci: init not implemented\n");

	return -ENOSYS;
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
