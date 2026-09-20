#
# Makefile for Phoenix-RTOS USB Mass Storage driver
#
# Copyright 2020, 2024 Phoenix Systems
#

# ext2-on-USB-stick support. Was ia32-only; aarch64 (Pi 4) needs it too -- a USB
# stick you cannot mount is not much use, and libext2 is already built for this
# target (it backs the SD card's ext2 root).
ifneq ($(filter ia32 aarch64,$(TARGET_FAMILY)),)
	UMASS_LIBS := libext2
	UMASS_CFLAGS := -DUMASS_MOUNT_EXT2
endif

NAME := libusbdrv-umass
LOCAL_SRCS := umass.c
LOCAL_CFLAGS += $(UMASS_CFLAGS)
include $(static-lib.mk)

NAME := umass
LOCAL_SRCS := umass.c srv.c
LIBS := libusb $(UMASS_LIBS)
LOCAL_CFLAGS += $(UMASS_CFLAGS)
include $(binary.mk)
