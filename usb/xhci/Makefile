#
# Makefile for Phoenix-RTOS xHCI
#
# Copyright 2026 Phoenix Systems
#

NAME := libusbxhci
LOCAL_SRCS := xhci.c phy-$(TARGET_FAMILY)-$(TARGET_SUBFAMILY).c

# Merged-design Pi 4 bus init: the USB daemon owns BCM2711 PCIe bridge
# bring-up + VL805 BAR0 programming in-process, eliminating the
# cross-process bridge translation race that the previous standalone
# `pcie` daemon hit. The standalone `pcie` daemon is no longer spawned
# in user.plo.yaml for Pi 4. Other BCM2711-board variants (none yet
# in tree) would opt in the same way.
ifeq ($(PCI_EXPRESS_BCM2711_INDEXED_CFG),y)
  LOCAL_SRCS += bcm2711-pcie.c
  CFLAGS += -DPCI_EXPRESS_BCM2711_INDEXED_CFG
endif

include $(static-lib.mk)
