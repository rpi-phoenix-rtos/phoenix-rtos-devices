#
# Phoenix-RTOS
#
# Raspberry Pi 4 (BCM43455 over SDIO) WiFi daemon (/dev/wifi, /dev/wifidata) and
# its control client `wifi`
#
# Copyright 2026 Phoenix Systems
# Author: Witold Bołt
#

# The daemon reads the BCM43455 firmware, NVRAM and CLM blob from
# /lib/firmware/brcm/ at start-up; none of them is part of this build.
NAME := rpi4-wifi
LOCAL_SRCS := rpi4-wifi.c
# WL_REG_ON and the SDHCI base clock go through the mailbox server.
DEPS := libvcmbox
DEP_LIBS := libvcmbox

include $(binary.mk)


# /bin, not /sbin: `wifi connect|disconnect|status|scan` is the user interface.
NAME := wifi
LOCAL_SRCS := wifi.c
LOCAL_INSTALL_PATH := /bin

include $(binary.mk)
