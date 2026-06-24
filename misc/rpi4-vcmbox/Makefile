#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) VideoCore property-mailbox server + client library
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

# Client library: the IPC veneer over /dev/vcmbox. Linked into the server itself
# (for the shared ABI struct) and into every mailbox client (thermal, and later
# genet/usb/sdio/v3d) via DEP_LIBS.
NAME := libvcmbox
LOCAL_SRCS := libvcmbox.c
LOCAL_HEADERS := libvcmbox.h
include $(static-lib.mk)


# The serializing mailbox server (/dev/vcmbox).
NAME := rpi4-vcmbox
LOCAL_SRCS := rpi4-vcmbox.c
DEPS := libvcmbox
DEP_LIBS := libvcmbox
include $(binary.mk)
