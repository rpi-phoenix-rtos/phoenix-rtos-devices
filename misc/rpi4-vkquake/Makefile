#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) vkQuake port (Vulkan QuakeSpasm capstone).
#
# The engine + Phoenix platform shims (incl. main(), in pl_phoenix_main.o) are
# pre-built into /tmp/libvkquake.a by tools/vkquake-port/build-vkquake-phoenix.py
# (--link), with real SPIR-V shaders (glslang+spirv-tools). This program links
# that archive against the ported Mesa V3DV Vulkan ICD (libv3dv-phoenix.a) + the
# reused v3d gallium back-end (libv3d-phoenix.a) and is bundled into loader.disk
# (boot-launched), since a ~19 MB binary cannot be NFS-exec'd by the read-based
# process loader.
#
# This is the Vulkan counterpart to rpi4-quake (GL flagship). loader.disk fits
# only one large GL/VK binary per boot, so the two SWAP: rpi4-quake is the
# bundled/launched default; rpi4-vkquake is built default-off (see
# _targets/Makefile.aarch64a72-generic + user.plo.yaml).
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-vkquake
LOCAL_SRCS := rpi4-vkquake-stub.c

VKQUAKE_LIB := /tmp/libvkquake.a
V3DV_LIB := /tmp/libv3dv-phoenix.a
V3D_LIB := /tmp/libv3d-phoenix.a

# -z stack-size sets PT_GNU_STACK p_memsz -> the kernel (process.c) uses it as the
# MAIN-thread stack size. Like Quakespasm, vkQuake runs Host_Init + the Host_Frame
# loop on the main thread (pl_phoenix_main.c) and needs a large stack (big frames +
# recursive renderers): 32 MB.
#
# --build-id: V3DV's init_uuids() walks the ELF for a build-id note (pipeline-cache
# UUID); without it create_physical_device fails "Failed to find build-id".
# --allow-multiple-definition: a few V3D-4.2 version-specific helpers (v3d42_*) are
# compiled into BOTH the V3DV front-end and the gallium back-end; with libv3dv first
# in the group the linker binds V3DV's copy (correct), matching build-v3dv-phoenix.py.
# libvkquake first so crt0's `main` ref pulls pl_phoenix_main.o -> the engine TUs;
# the group resolves the cross-archive refs. -lstdc++ for the Mesa compiler C++ TUs.
# --whole-archive over libvkquake + V3DV: the Mesa Vulkan-runtime layer's vk_common_*
# fallback entrypoints (vkGetBufferMemoryRequirements, vkQueueSubmit, vkCreateFence,
# vkCreateCommandPool, ...) are reachable ONLY via WEAK relocs in the generated dispatch
# table; weak refs don't pull archive members, so a plain group link silently drops them
# -> NULL dispatch slots -> pc=0 Instruction Abort at runtime (0-undefined link masks it).
# Whole-archive forces those objects in. Mirrors build-vkquake-phoenix.py's standalone link.
LOCAL_LDLIBS := -Wl,-z,stack-size=33554432 -Wl,--build-id -Wl,--allow-multiple-definition -Wl,--whole-archive $(VKQUAKE_LIB) $(V3DV_LIB) -Wl,--no-whole-archive -Wl,--start-group $(V3DV_LIB) $(V3D_LIB) -Wl,--end-group -lstdc++ -lm

include $(binary.mk)
