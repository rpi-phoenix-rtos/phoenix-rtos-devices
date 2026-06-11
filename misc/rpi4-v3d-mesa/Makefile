#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) Mesa v3d driver harness (GLQuake Path C boot-launch)
#
# Links the ported Mesa v3d gallium driver (libv3d-phoenix.a, built by
# tools/v3d-driver-port/build-v3d-phoenix.py with the matching aarch64a72 ABI) and
# runs v3d_screen_create on HW. The Mesa headers live in the coordination repo's
# external/mesa clone + the host Mesa build's generated headers; referenced by
# absolute path (this is a bring-up harness, not an upstreamable component).
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-v3d-mesa
LOCAL_SRCS := rpi4-v3d-mesa.c

# Coordination-repo paths (current dev host); see CLAUDE.md.
V3D_COORD := /home/houp/phoenix-rpi
V3D_MESA := $(V3D_COORD)/external/mesa
V3D_GEN := /tmp/mesa-v3d-build/src
V3D_LIB := /tmp/libv3d-phoenix.a
V3D_COMPAT := $(V3D_COORD)/tools/v3d-driver-port/phoenix_mesa_compat.h

# Force-include the libc/libm compat shim (lrintf/etc.) like every Mesa TU.
# -Wno-error: Mesa headers use undefined-macro-as-0 (#if DETECT_ARCH_X86 ...) and
# other idioms the project's -Werror=undef/etc. reject; this is external code.
LOCAL_CFLAGS := -include $(V3D_COMPAT) -Wno-error -Wno-undef \
	-I$(V3D_MESA)/src -I$(V3D_MESA)/include \
	-I$(V3D_MESA)/src/gallium/include -I$(V3D_MESA)/src/gallium/auxiliary \
	-I$(V3D_MESA)/src/util -I$(V3D_GEN) \
	-DUTIL_ARCH_LITTLE_ENDIAN=1 -DUTIL_ARCH_BIG_ENDIAN=0

# Use LOCAL_LDLIBS (placed AFTER the objects in the link line) so the archive
# resolves main.o's references; LOCAL_LDFLAGS lands before the objects and would
# be discarded. Group the lib (internal cross-refs). -lm for the compiler's C99
# math. binary.mk already adds --gc-sections.
LOCAL_LDLIBS := -Wl,--start-group $(V3D_LIB) -Wl,--end-group -lm

include $(binary.mk)
