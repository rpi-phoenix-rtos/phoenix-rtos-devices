#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) V3D 4.2 render server (/dev/v3d-async, /dev/dri/renderD128)
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

V3DA_DIR := $(call my-dir)

NAME := rpi4-v3d-async
LOCAL_SRCS := v3da_main.c v3da_hw.c v3da_sched.c v3da_jobs.c v3da_bo.c v3da_param.c
# uapi/: the vendored MIT v3d DRM uapi (DRM_V3D_PARAM_*), the ABI the server answers.
LOCAL_CFLAGS := -I$(V3DA_DIR)uapi
# V3D power and clock go through the serializing mailbox server.
DEPS := libvcmbox
DEP_LIBS := libvcmbox

include $(binary.mk)
