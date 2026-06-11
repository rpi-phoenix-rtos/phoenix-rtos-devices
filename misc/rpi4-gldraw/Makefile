#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) Mesa OpenGL textured-triangle harness (GLQuake Path C)
#
# Links the ported Mesa GL frontend (libGL-phoenix.a) + v3d gallium driver
# (libv3d-phoenix.a) and draws a textured, depth-tested triangle through the
# OpenGL API on real HW -- the rung between glClear and glgears.
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-gldraw
LOCAL_SRCS := rpi4-gldraw.c

V3D_COORD := /home/houp/phoenix-rpi
V3D_MESA := $(V3D_COORD)/external/mesa
V3D_GEN := /tmp/mesa-v3d-build/src
V3D_GL_LIB := /tmp/libGL-phoenix.a
V3D_LIB := /tmp/libv3d-phoenix.a
V3D_COMPAT := $(V3D_COORD)/tools/v3d-driver-port/phoenix_mesa_compat.h

LOCAL_CFLAGS := -include $(V3D_COMPAT) -Wno-error -Wno-undef \
	-I$(V3D_MESA)/src -I$(V3D_MESA)/include \
	-I$(V3D_MESA)/src/mesa -I$(V3D_MESA)/src/mapi -I$(V3D_MESA)/src/compiler \
	-I$(V3D_MESA)/src/gallium/include -I$(V3D_MESA)/src/gallium/auxiliary \
	-I$(V3D_MESA)/src/util -I$(V3D_GEN) \
	-DUTIL_ARCH_LITTLE_ENDIAN=1 -DUTIL_ARCH_BIG_ENDIAN=0 -DHAVE_STRUCT_TIMESPEC

LOCAL_LDLIBS := -Wl,--start-group $(V3D_GL_LIB) $(V3D_LIB) -Wl,--end-group -lstdc++ -lm

include $(binary.mk)
