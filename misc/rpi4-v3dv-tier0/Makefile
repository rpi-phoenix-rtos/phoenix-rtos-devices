#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) Vulkan V3DV Tier-1 boot harness (rpi4-v3dv-tier0)
#
# Drives the ported Mesa V3DV (Vulkan) ICD through vkCreateInstance ->
# vkEnumeratePhysicalDevices -> vkCreateDevice on real HW. Links the Vulkan
# front-end+runtime (libv3dv-phoenix.a) + the reused v3d gallium back-end
# (libv3d-phoenix.a). The winsys self-powers the V3D lazily on first MMIO.
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-v3dv-tier0
LOCAL_SRCS := rpi4-v3dv-tier0.c

V3D_COORD := /home/houp/phoenix-rpi
V3DV_LIB := /tmp/libv3dv-phoenix.a
V3D_LIB := /tmp/libv3d-phoenix.a

# The harness is a clean loader-less Vulkan client: it only needs <vulkan/vulkan.h>.
LOCAL_CFLAGS := -Wno-error -I$(V3D_COORD)/external/mesa/include

# libv3dv FIRST (binds V3DV's copy of any symbol shared with the back-end), grouped
# for the cross-archive refs; -lstdc++ for the Mesa compiler C++ TUs; -lm for math.
# --allow-multiple-definition: a few V3D-4.2 version-specific helpers (v3d42_*) are
# compiled into BOTH the V3DV front-end and the gallium back-end; with libv3dv first
# the linker binds V3DV's copy (correct), matching the build-v3dv-phoenix.py link.
LOCAL_LDLIBS := -Wl,--allow-multiple-definition -Wl,--start-group $(V3DV_LIB) $(V3D_LIB) -Wl,--end-group -lstdc++ -lm

include $(binary.mk)
