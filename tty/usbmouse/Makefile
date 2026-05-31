#
# Makefile for Phoenix-RTOS USB HID mouse driver
#
# Copyright 2026 Phoenix Systems
#
# %LICENSE%
#

NAME := libusbdrv-usbmouse
LOCAL_SRCS := usbmouse.c
include $(static-lib.mk)

NAME := usbmouse
LOCAL_SRCS := usbmouse.c srv.c
LIBS := libusb
include $(binary.mk)
