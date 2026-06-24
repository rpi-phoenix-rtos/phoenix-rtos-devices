#
# Phoenix-RTOS
#
# BCM2711 (Raspberry Pi 4) EMMC2 SD card driver
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#
# TODO: sdcard.c / sdhost_defs.h / sdcard.h / sdstorage_*.c are the
# generic SDHCI core, currently copied from storage/zynq7000-sdcard. Only
# bcm2711-sdio.c is BCM2711-specific. De-duplicate into a shared lib before
# upstreaming.
#

NAME := libsdcard-bcm2711
LOCAL_SRCS := sdcard.c bcm2711-sdio.c
LOCAL_HEADERS := sdcard.h
# bcm2711-sdio.c reads the EMMC2 reference clock via the rpi4-vcmbox client lib;
# DEPS makes libvcmbox's header install before this lib compiles. The archive
# itself is linked at the bcm2711-emmc binary below (LIBS), since a static lib
# cannot itself link another.
DEPS := libvcmbox

include $(static-lib.mk)

NAME := bcm2711-emmc
LOCAL_SRCS := sdstorage_dev.c sdstorage_srv.c
DEP_LIBS := libsdcard-bcm2711
# Matches zynq7000-sdcard: the copied sdstorage_srv.c registers both jffs2 and
# ext2 and uses the MTD shim, so libmtd/libjffs2 are needed to link. We only
# use ext2 on the Pi 4; trimming the jffs2/mtd paths is a later cleanup.
# libvcmbox resolves bcm2711-sdio.c's vcmbox_call() (in libsdcard-bcm2711); it
# must follow that archive on the link line.
LIBS := libstorage libcache libmbr libmtd libjffs2 libext2 libvcmbox

include $(binary.mk)
