#!/usr/bin/env bash
#
# Standalone build + stage of the rpi4-hci /dev/hci0 driver and the btctl test
# client, for netboot HW testing BEFORE boot/plo integration (de-risked first
# increment of T-WIFI-BT). Compiles with the aarch64-phoenix toolchain (like
# tools/bt-probe), stages the binaries into the NFS root /bin and the .hcd
# firmware into /etc/bluetooth so the driver can patchram at runtime.
#
# Copyright 2026 Phoenix Systems
# SPDX-License-Identifier: BSD-3-Clause
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../../.." && pwd)"
GCC="${GCC:-$REPO_ROOT/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-gcc}"
NM="${NM:-$REPO_ROOT/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-nm}"
NFSROOT="${NFSROOT:-/srv/phoenix-rpi4-nfs}"
HCD_SRC="${HCD_SRC:-$REPO_ROOT/artifacts/linux-netboot/rootfs/usr/lib/firmware/brcm/BCM4345C0.raspberrypi,4-model-b.hcd}"
CFLAGS="-O2 -Wall -Wextra -std=gnu11"

echo "rpi4-hci: building driver + btctl"
"$GCC" $CFLAGS -o "$HERE/rpi4-hci" "$HERE/rpi4-hci.c"
"$GCC" $CFLAGS -o "$HERE/btctl" "$HERE/btctl.c"

echo "rpi4-hci: undefined-symbol check (expect none):"
for b in rpi4-hci btctl; do
	u=$("$NM" -u "$HERE/$b" 2>/dev/null | grep -vE 'GLIBC|^\s*$' | wc -l || true)
	echo "  $b: $u undefined"
done
file "$HERE/rpi4-hci" "$HERE/btctl"

if [ -d "$NFSROOT" ]; then
	echo "rpi4-hci: staging into $NFSROOT"
	sudo mkdir -p "$NFSROOT/bin" "$NFSROOT/etc/bluetooth"
	sudo cp "$HERE/rpi4-hci" "$NFSROOT/bin/rpi4-hci"
	sudo cp "$HERE/btctl" "$NFSROOT/bin/btctl"
	if [ -f "$HCD_SRC" ]; then
		sudo cp "$HCD_SRC" "$NFSROOT/etc/bluetooth/BCM4345C0.hcd"
		echo "rpi4-hci: staged .hcd ($(stat -L -c%s "$HCD_SRC") bytes)"
	else
		echo "rpi4-hci: WARNING .hcd not found at $HCD_SRC (driver will run unpatched)"
	fi
	echo "rpi4-hci: staged. On the Pi:  rpi4-hci &   then   btctl scan"
else
	echo "rpi4-hci: NFS root $NFSROOT absent; skipped staging."
fi
echo "rpi4-hci: done."
