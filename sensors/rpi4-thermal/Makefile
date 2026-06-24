#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) SoC thermal / throttle sensor
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-thermal
LOCAL_SRCS := rpi4-thermal.c
# Mailbox access is routed through the serializing rpi4-vcmbox server via its
# client lib; DEPS makes its header install first, DEP_LIBS links the archive.
DEPS := libvcmbox
DEP_LIBS := libvcmbox

include $(binary.mk)
