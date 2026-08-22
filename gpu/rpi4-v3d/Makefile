#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server + client library
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

# Client library: the IPC veneer over /dev/v3d-srv, linked into each GPU app
# (instead of the in-process winsys) via DEP_LIBS. Needs the DRM UAPI headers
# (v3d_drm.h -> drm.h) on the include path; the GPU-app build already provides
# drm-uapi/, so this component only ships the RPC glue + the shared v3d_rpc.h.
NAME := libv3d-client
LOCAL_SRCS := libv3d-client.c
LOCAL_HEADERS := libv3d-client.h v3d_rpc.h
include $(static-lib.mk)


# The serializing GPU server (/dev/v3d-srv).
#
# SCAFFOLD: RPC skeleton only, no GPU logic yet (no power-on / MMIO / MMU / real
# submit). Not yet added to DEFAULT_COMPONENTS for the rpi4 target - the next
# increment wires the winsys GPU logic in and enables it in the image.
NAME := rpi4-v3d
LOCAL_SRCS := rpi4-v3d.c
include $(binary.mk)
