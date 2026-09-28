#
# Phoenix-RTOS
#
# Anonymous shared-memory server (/shm): memfd_create() backing for Wayland wl_shm
# pools and the X11 DRI3 xshmfence pages
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

NAME := shmsrv
LOCAL_SRCS := shmsrv.c
# /bin, not /sbin: clients run `shmsrv -s` (statistics) and `shmsrv -q` from scripts.
LOCAL_INSTALL_PATH := /bin

include $(binary.mk)
