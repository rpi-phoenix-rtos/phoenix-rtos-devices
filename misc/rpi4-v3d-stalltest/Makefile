#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) V3D render-stall repro harness.
#
# Renders a 1920x1088 frame (clear + triangle) in a loop and counts CT1 render
# timeouts (winsys v3d_phoenix_render_timeouts) to get many stall samples per boot,
# escaping one-boot-per-sample roulette. Links the ported Mesa GL frontend
# (libGL-phoenix.a) + v3d gallium driver (libv3d-phoenix.a). Same recipe as
# rpi4-glclear. Bring-up harness, not an upstreamable component.
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-v3d-stalltest
LOCAL_SRCS := rpi4-v3d-stalltest.c

# Coordination-repo paths (current dev host); see CLAUDE.md.
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
