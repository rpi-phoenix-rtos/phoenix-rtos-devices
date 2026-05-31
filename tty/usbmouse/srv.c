/*
 * Phoenix-RTOS
 *
 * USB HID mouse driver
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


#ifndef USBMOUSE_UMSG_PRIO
#define USBMOUSE_UMSG_PRIO 4
#endif

#ifndef USBMOUSE_N_UMSG_THREADS
#define USBMOUSE_N_UMSG_THREADS 1
#endif


int main(int argc, char *argv[])
{
	usb_driver_t *driver = usb_registeredDriverPop();

	if (driver == NULL) {
		fprintf(stderr, "usbmouse: no driver registered!\n");
		return 1;
	}

	usb_driverProcRun(driver, USBMOUSE_UMSG_PRIO, USBMOUSE_N_UMSG_THREADS, NULL);
}
