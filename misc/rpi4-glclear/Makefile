#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) Mesa OpenGL glClear harness (GLQuake Path C boot-launch)
#
# Links the ported Mesa GL frontend (libGL-phoenix.a) + v3d gallium driver
# (libv3d-phoenix.a) and drives a glClear through the OpenGL API on real HW.
# The Mesa headers live in the coordination repo's external/mesa clone + the host
# Mesa build's generated headers; referenced by absolute path (bring-up harness).
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-glclear
LOCAL_SRCS := rpi4-glclear.c

# Coordination-repo paths (current dev host); see CLAUDE.md.
V3D_COORD := /home/houp/phoenix-rpi
V3D_MESA := $(V3D_COORD)/external/mesa
V3D_GEN := /tmp/mesa-v3d-build/src
V3D_GL_LIB := /tmp/libGL-phoenix.a
V3D_LIB := /tmp/libv3d-phoenix.a
V3D_COMPAT := $(V3D_COORD)/tools/v3d-driver-port/phoenix_mesa_compat.h

# Force-include the libc/libm compat shim like every Mesa TU. The GL frontend adds
# src/mesa (state_tracker/, main/, program/) + src/mapi + src/compiler to the v3d
# include set. -Wno-error: external Mesa code uses idioms the project -Werror rejects.
LOCAL_CFLAGS := -include $(V3D_COMPAT) -Wno-error -Wno-undef \
	-I$(V3D_MESA)/src -I$(V3D_MESA)/include \
	-I$(V3D_MESA)/src/mesa -I$(V3D_MESA)/src/mapi -I$(V3D_MESA)/src/compiler \
	-I$(V3D_MESA)/src/gallium/include -I$(V3D_MESA)/src/gallium/auxiliary \
	-I$(V3D_MESA)/src/util -I$(V3D_GEN) \
	-DUTIL_ARCH_LITTLE_ENDIAN=1 -DUTIL_ARCH_BIG_ENDIAN=0 -DHAVE_STRUCT_TIMESPEC

# libGL first (it references the driver's v3d_screen_create); grouped for the
# internal cross-refs. -lstdc++ for the GLSL compiler's C++ TUs; -lm for C99 math.
# binary.mk already adds --gc-sections.
LOCAL_LDLIBS := -Wl,--start-group $(V3D_GL_LIB) $(V3D_LIB) -Wl,--end-group -lstdc++ -lm

include $(binary.mk)
