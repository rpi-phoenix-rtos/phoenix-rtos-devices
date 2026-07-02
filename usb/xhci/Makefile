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
  # bcm2711-pcie.c does `#include <libvcmbox.h>`, a header installed into the
  # shared per-target include prefix by the rpi4-vcmbox component (its libvcmbox
  # static lib) in this same devices pass. There is no file-level dependency
  # edge, so under parallel make (-j) on a cold sysroot the xhci objects could
  # compile before that header is installed. DEPS adds an order-only prerequisite
  # (static-lib.mk: `$(OBJS): | $(DEPS)`) on the libvcmbox component, whose
  # `libvcmbox-headers` sub-target installs the header, forcing correct ordering.
  DEPS += libvcmbox
endif

include $(static-lib.mk)
