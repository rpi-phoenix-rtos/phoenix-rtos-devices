#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) PWM audio output device (/dev/audio0)
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-audio
LOCAL_SRCS := rpi4-audio.c
# The firmware's free DMA channel mask comes through the serializing mailbox server.
DEPS := libvcmbox
DEP_LIBS := libvcmbox

include $(binary.mk)
