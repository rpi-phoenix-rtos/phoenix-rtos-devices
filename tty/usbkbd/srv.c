/*
 * Phoenix-RTOS
 *
 * USB HID keyboard driver
 *
 * Device driver server
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */


#include <stdio.h>

#include <usbdriver.h>
#include <usbprocdriver.h>


#ifndef USBKBD_UMSG_PRIO
#define USBKBD_UMSG_PRIO 4
#endif

#ifndef USBKBD_N_UMSG_THREADS
#define USBKBD_N_UMSG_THREADS 1
#endif


int main(int argc, char *argv[])
{
	usb_driver_t *driver = usb_registeredDriverPop();

	if (driver == NULL) {
		fprintf(stderr, "usbkbd: no driver registered!\n");
		return 1;
	}

	usb_driverProcRun(driver, USBKBD_UMSG_PRIO, USBKBD_N_UMSG_THREADS, NULL);
}
