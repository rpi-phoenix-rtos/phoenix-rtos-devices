#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server + client library
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

# Absolute path of THIS makefile's dir, so the vendored DRM UAPI include path is
# self-contained (must be captured before including binary/static-lib.mk, which
# compute their own LOCAL_DIR). See uapi/ for the vendored v3d_drm.h -> drm.h ->
# drm_mode.h + the sys/ioccom.h shim (Phoenix sysroot has no <sys/ioccom.h>).
# This removes the M1-step1 dependency on a coord-repo tools/ -I path.
V3D_DIR := $(call my-dir)
V3D_UAPI_CFLAGS := -I$(V3D_DIR)uapi


# Client library: the IPC veneer over /dev/v3d-srv, linked into each GPU app
# (instead of the in-process winsys) via DEP_LIBS. Needs the vendored DRM UAPI
# headers (v3d_drm.h -> drm.h) on the include path.
NAME := libv3d-client
LOCAL_SRCS := libv3d-client.c
LOCAL_HEADERS := libv3d-client.h v3d_rpc.h
LOCAL_CFLAGS := $(V3D_UAPI_CFLAGS)
include $(static-lib.mk)


# The serializing GPU server (/dev/v3d-srv).
#
# M1 step 2a: takes exclusive GPU ownership (v3d_gpu.c: power-on / MMIO map / MMU
# page table / VA allocator / BO table, copied verbatim from the in-process
# winsys) and serves the BO-management RPCs. SUBMIT_* still stubbed (step 2b).
# Not yet in DEFAULT_COMPONENTS for the rpi4 target.
NAME := rpi4-v3d
LOCAL_SRCS := rpi4-v3d.c v3d_gpu.c
LOCAL_HEADERS := v3d_gpu.h
LOCAL_CFLAGS := $(V3D_UAPI_CFLAGS)
include $(binary.mk)
