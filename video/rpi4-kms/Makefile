#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) display server (/dev/kms, /dev/dri/card0, /kmsbuf)
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

KMS_DIR := $(call my-dir)

NAME := rpi4-kms
LOCAL_SRCS := kms_main.c kms_fw.c kms_backend.c kms_bo.c kms_vblank.c
# v3da_proto.h: the render server's fence page and buffer namespace (-G in-fences,
# imports of its buffers). KMS_POLL_NOTIFY: wake poll() on a card descriptor at the
# event (the kernel's pollNotify()).
LOCAL_CFLAGS := -I$(KMS_DIR)../../gpu/rpi4-v3d-async -DKMS_POLL_NOTIFY
# The firmware display calls (modes, planes, vblank) go through the mailbox server.
DEPS := libvcmbox
DEP_LIBS := libvcmbox

include $(binary.mk)
