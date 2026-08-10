#!/usr/bin/env bash
#
# Standalone build + stage of the rpi4-wifi /dev/wifi driver and the wifi test
# client, for netboot HW testing BEFORE boot/plo integration (de-risked first
# increment of T-WIFI-BT, mirroring bt/rpi4-hci/build-standalone.sh). Compiles
# with the aarch64-phoenix toolchain (like tools/wifi-probe) and stages the two
# binaries into the NFS root /bin.
#
# The BCM43455 firmware / NVRAM / CLM regulatory blobs are COMPILED-IN C arrays
# (no runtime firmware files), pulled in exactly as tools/wifi-probe/build.sh
# does:
#   - firmware + NVRAM: the generated C arrays wifi-fw-43455.c / wifi-nvram-43455.c
#     in the lwip-port dir (produced by scripts/gen-wifi-fw-c.sh from the
#     gitignored .firmware/ blobs); compiled at -O0 (pure data) and linked in.
#   - CLM: the generated header clm-43455.h in tools/wifi-probe (gitignored,
#     Cypress EULA) — a `static const` array included directly by rpi4-wifi.c
#     via the -I include path (NOT copied into this dir).
#
# Copyright 2026 Phoenix Systems
# SPDX-License-Identifier: BSD-3-Clause
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../../.." && pwd)"
GCC="${GCC:-$REPO_ROOT/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-gcc}"
NM="${NM:-$REPO_ROOT/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-nm}"
NFSROOT="${NFSROOT:-/srv/phoenix-rpi4-nfs}"

# The firmware / NVRAM C-arrays live in the lwip-port dir; the CLM header lives
# in tools/wifi-probe. Override if they live elsewhere.
LWIP_PORT="${LWIP_PORT:-$REPO_ROOT/sources/phoenix-rtos-lwip/port}"
WIFI_PROBE="${WIFI_PROBE:-$REPO_ROOT/tools/wifi-probe}"

FW_C="$LWIP_PORT/wifi-fw-43455.c"
NVRAM_C="$LWIP_PORT/wifi-nvram-43455.c"

CFLAGS="-O2 -Wall -Wextra -std=gnu11 -I$LWIP_PORT -I$WIFI_PROBE"
# The 3.9 MB firmware array is pure data — compile it at -O0 (high opt is slow +
# memory-heavy for zero codegen benefit), same as tools/wifi-probe/build.sh.
CFLAGS_DATA="-O0 -I$LWIP_PORT"

for f in "$GCC" "$FW_C" "$NVRAM_C" \
	"$LWIP_PORT/wifi-fw-43455.h" "$LWIP_PORT/wifi-nvram-43455.h" \
	"$WIFI_PROBE/clm-43455.h"; do
	if [ ! -e "$f" ]; then
		echo "rpi4-wifi/build-standalone.sh: missing required input: $f" >&2
		echo "  (firmware/nvram arrays: scripts/gen-wifi-fw-c.sh; clm: tools/wifi-probe/gen-clm.py)" >&2
		exit 1
	fi
done

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "rpi4-wifi: compiling driver logic (-O2)"
"$GCC" $CFLAGS -c "$HERE/rpi4-wifi.c" -o "$TMP/rpi4-wifi.o"

echo "rpi4-wifi: compiling firmware array (-O0, ~3.9 MB)"
"$GCC" $CFLAGS_DATA -c "$FW_C" -o "$TMP/wifi-fw-43455.o"

echo "rpi4-wifi: compiling nvram array (-O0)"
"$GCC" $CFLAGS_DATA -c "$NVRAM_C" -o "$TMP/wifi-nvram-43455.o"

echo "rpi4-wifi: linking driver"
"$GCC" -O2 \
	"$TMP/rpi4-wifi.o" \
	"$TMP/wifi-fw-43455.o" \
	"$TMP/wifi-nvram-43455.o" \
	-o "$HERE/rpi4-wifi"

echo "rpi4-wifi: building wifi client"
"$GCC" -O2 -Wall -Wextra -std=gnu11 -o "$HERE/wifi" "$HERE/wifi.c"

echo "rpi4-wifi: undefined-symbol check (expect none):"
for b in rpi4-wifi wifi; do
	if "$NM" -u "$HERE/$b" | grep -q .; then
		echo "  !! $b: UNDEFINED SYMBOLS PRESENT:" >&2
		"$NM" -u "$HERE/$b" >&2
		exit 1
	fi
	echo "  $b: 0 undefined symbols."
done
file "$HERE/rpi4-wifi" "$HERE/wifi"

if [ -d "$NFSROOT" ]; then
	echo "rpi4-wifi: staging into $NFSROOT"
	sudo mkdir -p "$NFSROOT/bin"
	sudo cp "$HERE/rpi4-wifi" "$NFSROOT/bin/rpi4-wifi"
	sudo cp "$HERE/wifi" "$NFSROOT/bin/wifi"
	echo "rpi4-wifi: staged. On the Pi:  rpi4-wifi &   then   wifi scan"
else
	echo "rpi4-wifi: NFS root $NFSROOT absent; skipped staging."
fi
echo "rpi4-wifi: done."
