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
# Stage into the export the Pi actually mounts: the one carrying fsid=0, not a
# hardcoded name. Hardcoding is how these two drivers came to be built, "staged",
# and tested against /srv/phoenix-rpi4-nfs while the Pi mounted
# /srv/phoenix-rpi4-nfs-gcc16 -- the driver was simply absent from the live root,
# so every run exercised whatever was there before. Scan /etc/exports.d/*.exports
# as well: the canonical entry lives there (declaring it in both files makes
# `exportfs -ra` fail), and a detector reading only /etc/exports finds nothing and
# falls back to the wrong directory. An explicit NFSROOT still wins.
_fsid0="$(awk '$0 ~ /fsid=0/ && $1 ~ /^\// { print $1; exit }' /etc/exports /etc/exports.d/*.exports 2>/dev/null || true)"
NFSROOT="${NFSROOT:-${_fsid0:-/srv/phoenix-rpi4-nfs}}"

# The firmware / NVRAM C-arrays live in the lwip-port dir; the CLM header lives
# in tools/wifi-probe. Override if they live elsewhere.
LWIP_PORT="${LWIP_PORT:-$REPO_ROOT/sources/phoenix-rtos-lwip/port}"
WIFI_PROBE="${WIFI_PROBE:-$REPO_ROOT/tools/wifi-probe}"

FW_C="$LWIP_PORT/wifi-fw-43455.c"
NVRAM_C="$LWIP_PORT/wifi-nvram-43455.c"

# Link against the TREE's libphoenix, not the toolchain's. Plain `$GCC` resolves
# libc from the toolchain's own sysroot, which is only refreshed when the
# toolchain is rebuilt -- so a driver built that way silently carries whatever
# libphoenix the toolchain was last synced with, not the one the rest of the
# image was built from (the "standalone tools link the TOOLCHAIN's libphoenix.a"
# trap, which has bitten this project on a syscall renumber). Prefer the
# buildroot sysroot when it exists; an explicit SYSROOT still wins, and
# SYSROOT=none keeps the toolchain default.
_br_sysroot="$REPO_ROOT/.buildroot/_build/aarch64a72-generic-rpi4b/sysroot"
SYSROOT="${SYSROOT:-}"
if [ -z "$SYSROOT" ] && [ -d "$_br_sysroot/lib" ]; then
	SYSROOT="$_br_sysroot"
fi
SYSROOT_OPTS=""
if [ -n "$SYSROOT" ] && [ "$SYSROOT" != "none" ]; then
	SYSROOT_OPTS="--sysroot=$SYSROOT/ -B$SYSROOT/lib/"
	echo "rpi4-wifi: linking against the tree's libphoenix ($SYSROOT)"
else
	echo "rpi4-wifi: linking against the TOOLCHAIN's libphoenix (no buildroot sysroot found)"
fi

CFLAGS="-O2 -Wall -Wextra -std=gnu11 $SYSROOT_OPTS -I$LWIP_PORT -I$WIFI_PROBE"
# The 3.9 MB firmware array is pure data — compile it at -O0 (high opt is slow +
# memory-heavy for zero codegen benefit), same as tools/wifi-probe/build.sh.
CFLAGS_DATA="-O0 $SYSROOT_OPTS -I$LWIP_PORT"

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
"$GCC" -O2 $SYSROOT_OPTS \
	"$TMP/rpi4-wifi.o" \
	"$TMP/wifi-fw-43455.o" \
	"$TMP/wifi-nvram-43455.o" \
	-o "$HERE/rpi4-wifi"

echo "rpi4-wifi: building wifi client"
"$GCC" -O2 -Wall -Wextra -std=gnu11 $SYSROOT_OPTS -o "$HERE/wifi" "$HERE/wifi.c"

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
