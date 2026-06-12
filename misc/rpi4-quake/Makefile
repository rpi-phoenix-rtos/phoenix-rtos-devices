#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM2711) Quakespasm port (GLQuake capstone).
#
# The engine + Phoenix platform shims are pre-built into /tmp/libquakespasm.a by
# tools/quakespasm-port/build-quakespasm-phoenix.py (two flag sets: Quake-side TUs
# and the Mesa-side GL-context shim). This program links that archive against the
# ported Mesa GL frontend (libGL-phoenix.a) + v3d gallium driver (libv3d-phoenix.a)
# and is bundled into loader.disk (boot-launched), since a ~19 MB binary cannot be
# NFS-exec'd by the read-based process loader.
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := rpi4-quake
LOCAL_SRCS := rpi4-quake-stub.c

QS_LIB := /tmp/libquakespasm.a
V3D_GL_LIB := /tmp/libGL-phoenix.a
V3D_LIB := /tmp/libv3d-phoenix.a

LOCAL_LDLIBS := -Wl,--start-group $(QS_LIB) $(V3D_GL_LIB) $(V3D_LIB) -Wl,--end-group -lstdc++ -lm

include $(binary.mk)
