#
# Makefile for Phoenix-RTOS USB HID keyboard driver
#
# Copyright 2026 Phoenix Systems
#
# %LICENSE%
#

NAME := libusbdrv-usbkbd
LOCAL_SRCS := usbkbd.c
include $(static-lib.mk)

NAME := usbkbd
LOCAL_SRCS := usbkbd.c srv.c
LIBS := libusb
include $(binary.mk)
