#!/usr/bin/env python3
"""Build Mesa's v3d gallium driver as a Phoenix static lib + link-drive a harness.

Path-C Phase 2. Cross-compiles for aarch64a72-phoenix by reusing the HOST Mesa
build's per-file compile flags (HOSTBUILD/compile_commands.json), transformed:
  - swap the compiler -> the Phoenix toolchain gcc
  - drop x86 -m* flags, the host /usr/include/libdrm + valgrind includes
  - force-include phoenix_mesa_compat.h (C99 math / posix_memalign / qsort_r / ...)
  - prepend the shim-include dir (xf86drm.h/sys/ioccom.h/dlfcn.h/libsync.h) + port dir

Compiles are run with cwd=HOSTBUILD so the generated-header `-Isrc/...` relative
includes resolve. Driver-file set = every .c the host built under
gallium/drivers/v3d (exact -DV3D_VERSION variants included). The winsys
(v3d_phoenix_winsys.c) + the screen-create harness are compiled with flags
synthesized from a template driver entry.

LINK-DRIVE: link the harness against the prebuilt core lib + an aux archive built
from /tmp/v3dphx-aux.txt (one mesa-relative .c path per line). The linker pulls
only the members it needs; undefined refs are written to /tmp/v3dphx-undef.txt.
Add the aux files that define those symbols to aux.txt and re-run. Converges in a
few passes (the real closure, not a speculative bulk compile).

This script is NOT part of the phoenix-rtos-devices Makefile build: the recipe runs
meson/ninja on external/mesa, which the framework Makefiles cannot do. It is a
standalone build, on the same model as wifi/rpi4-wifi/build-standalone.sh.

Usage:  python3 build-v3d-phoenix.py [--compile-only]

Copyright 2026 Phoenix Systems
SPDX-License-Identifier: BSD-3-Clause
"""
import json, os, shlex, subprocess, sys
import concurrent.futures
from pathlib import Path

# PORT = this script's own dir (the Phoenix glue: winsys/power/stubs/shims + the
# committed Mesa source manifests). ROOT = the coordination-repo checkout that holds
# external/mesa and tools/.gpu-libs; env-first (mirroring
# wifi/rpi4-wifi/build-standalone.sh), else derived from PORT's position in the
# sibling-repo layout <root>/sources/phoenix-rtos-devices/gpu/rpi4-v3d/mesa.
# Deriving PORT from __file__ (not from ROOT) keeps the two sibling scripts that
# `exec` this prelude working wherever this directory is checked out.
PORT      = str(Path(__file__).resolve().parent)
ROOT      = os.environ.get("PHOENIX_RPI_ROOT") or str(Path(PORT).parents[4])
MESA      = f"{ROOT}/external/mesa"
HOSTBUILD = "/tmp/mesa-v3d-build"
TC        = f"{ROOT}/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-gcc"
AR        = f"{ROOT}/.toolchain/aarch64-phoenix/bin/aarch64-phoenix-gcc-ar"
SHIM      = f"{PORT}/shim-include"
COMPAT    = f"{PORT}/phoenix_mesa_compat.h"
# The CANONICAL libvcmbox client (serialized /dev/vcmbox access), owned by the
# rpi4-vcmbox component in this same repo — not a copy. v3d_phoenix_power.c does
# `#include "libvcmbox.h"` and links libvcmbox.c into libv3d-phoenix.a.
VCMBOX    = str(Path(PORT).parents[2] / "misc" / "rpi4-vcmbox")
# Harnesses for the link-drive symbol-closure loop. They are diagnostic tools, not
# part of the shipped driver, so they stay in the coordination repo's tools/ tree
# (their outputs live in /tmp and are never staged). Override to relocate them.
HARNESS_DIR = os.environ.get("V3D_HARNESS_DIR") or f"{ROOT}/tools/v3d-driver-port"
OBJDIR    = "/tmp/v3dphx-obj"          # shared with the core-lib build
DRVOBJ    = "/tmp/v3dphx-drvobj"
AUXOBJ    = "/tmp/v3dphx-auxobj"
COREA     = "/tmp/libv3d-phoenix-core.a"
AUXLIST   = "/tmp/v3dphx-aux.txt"      # mesa-relative .c paths, one per line
CORELIST  = f"{PORT}/v3d-core-sources.txt"   # committed core manifest
COREOBJ   = "/tmp/v3dphx-coreobj"
UNDEF     = "/tmp/v3dphx-undef.txt"
HARNESS_BIN = "/tmp/v3dphx-harness"
GPU_LIBS  = f"{ROOT}/tools/.gpu-libs"  # stable home for the prebuilt engine archives (was /tmp)
FULL_LIB  = f"{GPU_LIBS}/libv3d-phoenix.a"  # one archive: core+aux+driver+winsys+shim+stubs

# --- SYSROOT: compile against what the build produced, not the toolchain bundle ---
#
# Everything else in this project compiles against
# .buildroot/_build/<target>/sysroot; these Mesa builds passed no --sysroot, so
# ~400 objects resolved libc out of .toolchain/aarch64-phoenix/aarch64-phoenix/
# usr/include -- a HAND-MAINTAINED bundle. Measured 2026-09-04
# (docs/misc/2026-09-04-toolchain-header-skew.md): that snapshot was dated
# 2026-07-23 and five macro VALUES disagreed with live libphoenix, with
# PTHREAD_PROCESS_PRIVATE/SHARED and PTHREAD_PRIO_NONE/INHERIT *swapped* -- code
# built there asking for priority inheritance would silently get NOINHERIT. Same
# class as the recorded _SC_NPROCESSORS incident: a value skew warns about nothing.
#
# The rule is not new: scripts/build-rootfs-helpers.sh states it for the launcher
# helpers and fails hard when the sysroot is absent. Do the same here, and do NOT
# fall back to the bundle -- a silent fallback is exactly the trap being closed.
# The three flags mirror phoenix-rtos-build/makes/setup-sysroot.mk:17-23 so the
# include/link search order matches every other Phoenix component.
TARGET    = os.environ.get("TARGET") or "aarch64a72-generic-rpi4b"
BUILDROOT = os.environ.get("RPI4B_BUILDROOT") or f"{ROOT}/.buildroot"
SYSROOT   = os.environ.get("PHOENIX_SYSROOT") or f"{BUILDROOT}/_build/{TARGET}/sysroot"
if not os.path.isfile(f"{SYSROOT}/lib/libphoenix.a"):
    sys.exit(f"no sysroot at {SYSROOT} (no lib/libphoenix.a).\n"
             "The GPU archives must compile against the sysroot the CORE stage produces:\n"
             "  ./scripts/rebuild-rpi4b-fast.sh --scope core   (or pass PHOENIX_SYSROOT=)\n"
             "Refusing to fall back to the .toolchain bundle -- it is hand-maintained and\n"
             "goes stale silently (docs/misc/2026-09-04-toolchain-header-skew.md).")
SYSROOT_OPTS = [f"--sysroot={SYSROOT}/", f"-B{SYSROOT}/lib/", "-iprefix", f"{SYSROOT}/"]

# Correct on-device ABI (matches sources/phoenix-rtos-build/target/aarch64.mk) +
# section flags so the final --gc-sections strips code unreachable from main().
ABI_FLAGS = ["-mcpu=cortex-a72", "-mtune=cortex-a72", "-mstrict-align",
             "-mno-outline-atomics", "-fomit-frame-pointer",
             "-ffunction-sections", "-fdata-sections"]

# task #29 striping discriminator: set VKQ_CPU_TILE=1 in the environment to build the
# winsys CPU-tile path (CPU-tiles UIF textures into the dest image BO + skips the HW TFU
# kick) instead of the hardware TFU upload. Default OFF -> GLQuake/normal path unchanged.
# The macro only appears in v3d_phoenix_winsys.c, so passing it to every TU is harmless.
EXTRA_DEFINES = (["-DVKQ_CPU_TILE=1"]
                 if os.environ.get("VKQ_CPU_TILE", "0") not in ("", "0")
                 else [])

db = json.load(open(f"{HOSTBUILD}/compile_commands.json"))
by_file = {e["file"]: e for e in db}


def abssrc(f):
    """compile_commands 'file' is relative to HOSTBUILD -> absolute, normalized."""
    return os.path.normpath(f if os.path.isabs(f) else os.path.join(HOSTBUILD, f))


by_abs = {abssrc(e["file"]): e for e in db}     # robust source lookup (any path form)


def transform(entry, src, out):
    """Host compile command -> Phoenix cross compile command for `src`->`out`."""
    toks = shlex.split(entry.get("command") or " ".join(entry["arguments"]))[1:]
    keep, i = [], 0
    while i < len(toks):
        t = toks[i]
        if t == "-c":                                   i += 1; continue
        if t == "-o":                                   i += 2; continue
        if t in ("-MD", "-MMD", "-MP"):                 i += 1; continue
        if t in ("-MF", "-MQ", "-MT"):                  i += 2; continue
        if t.startswith("-m"):                          i += 1; continue
        if t == "-pthread":                             i += 1; continue
        if t.startswith("-DHAVE_VALGRIND"):             i += 1; continue
        if t.startswith("-Werror"):                     i += 1; continue   # warnings ok for the port
        if t.startswith("-I/usr/include/libdrm"):       i += 1; continue
        if t.startswith("-I/usr/include/valgrind"):     i += 1; continue
        if t == entry["file"]:                          i += 1; continue
        keep.append(t); i += 1
    # GCC-14 promotes implicit-function-declaration/implicit-int to hard errors by
    # default; downgrade to warnings for the port (Phoenix libc misses some decls).
    # ABI = the project's aarch64.mk flags (correct on-device); section flags let the
    # final link --gc-sections drop everything unreachable from main (shrinks the ELF).
    return [TC, "-c", src, "-o", out, f"-I{SHIM}", f"-I{PORT}", f"-I{VCMBOX}"] + SYSROOT_OPTS + ABI_FLAGS + EXTRA_DEFINES + [
            "-Wno-error=implicit-function-declaration", "-Wno-error=implicit-int",
            "-Wno-error=int-conversion",
            "-include", COMPAT] + keep


def compile_one(entry, src, out):
    cmd = transform(entry, src, out)
    r = subprocess.run(cmd, cwd=HOSTBUILD, capture_output=True, text=True)
    return r.returncode, r.stderr, cmd


def driver_entries():
    return [e for e in db if "/gallium/drivers/v3d/" in e["file"] and e["file"].endswith(".c")]


def template_entry():
    for e in db:
        if e["file"].endswith("gallium/drivers/v3d/v3d_screen.c"):
            return e
    raise SystemExit("no template entry (v3d_screen.c) in compile_commands")


def build_objs(entries, srcs_outs, objdir, label):
    """entries: list of (entry, src, out). Returns (ok_objs, fails).

    INCR=1 in the environment enables an mtime-based skip: a cached .o that is at least as
    new as its .c source is reused instead of recompiled. This turns a ~9-min full front-end
    rebuild into a ~30-s recompile of just the edited TUs — the intended workflow when
    iterating on one or two .c files (e.g. the v3dv copy-path). CAVEAT: mtime tracks the .c
    only, NOT its headers, so after editing a shared .h do a full (INCR-unset) rebuild."""
    os.makedirs(objdir, exist_ok=True)
    incr = os.environ.get("INCR") == "1"
    ok, fails, skipped = [], [], 0
    cached, todo = set(), []
    for entry, src, out in srcs_outs:
        # INCR skip: reuse a cached .o at least as new as its .c. `src` may be a relative path
        # (gallium compile_commands entries) that doesn't resolve from this cwd; resolve it against
        # the mesa build dir, and if it still can't be stat'd, fall through to recompile (never skip
        # on a stat error). HOSTBUILD is the ninja build dir compile_one runs in.
        if incr and os.path.exists(out):
            try:
                s = src if os.path.exists(src) else os.path.join(HOSTBUILD, src)
                if os.path.getmtime(out) >= os.path.getmtime(s):
                    cached.add(out)
                    skipped += 1
                    continue
            except OSError:
                pass
        todo.append((entry, src, out))
    # Parallel compile: each compile_one() is an independent gcc subprocess writing a distinct .o,
    # so a thread pool of cpu_count() workers uses ALL cores (the GIL doesn't block subprocess I/O).
    # Previously this loop ran one gcc at a time — the slow part of a ~400-object libv3d build.
    results = {}
    if todo:
        workers = max(1, os.cpu_count() or 1)
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
            futs = {ex.submit(compile_one, e, s, o): o for (e, s, o) in todo}
            for fut in concurrent.futures.as_completed(futs):
                results[futs[fut]] = fut.result()   # (rc, err, cmd)
    # Assemble ok/fails in the ORIGINAL srcs_outs order — deterministic archive member order
    # (bit-reproducible builds) regardless of parallel completion order.
    for entry, src, out in srcs_outs:
        if out in cached:
            ok.append(out)
            continue
        rc, err, cmd = results[out]
        if rc == 0:
            ok.append(out)
        else:
            fails.append((src, err, cmd))
    if incr:
        print(f"[{label}] INCR: {skipped} cached, {len(ok) - skipped} recompiled")
    print(f"[{label}] OK={len(ok)} FAIL={len(fails)}")
    for src, err, cmd in fails:
        print(f"  FAIL {os.path.relpath(src, MESA)}")
        for l in err.strip().splitlines()[-3:]:
            print(f"       {l}")
    # ...then the exact command + the COMPLETE compiler output for each failure.
    # The 3-line tail above is a useful digest, but on its own it hid the real
    # cause whenever the diagnostic that mattered was further up the log.
    if fails:
        print(f"[{label}] --- full compiler output per failure ---")
        for src, err, cmd in fails:
            print(f"===== {os.path.relpath(src, MESA)} =====")
            print("$ " + " ".join(cmd))
            print(err.strip() or "(no stderr)")
            print()
        sys.stdout.flush()
    return ok, fails


# Generated sources the aux/core lists consume but that a host `ninja` does NOT
# materialize on its own: the Phoenix gallium patch adds an aarch64 `dc civac`
# cache-flush asm to v3d_resource.c, which the HOST assembler can't encode, so the
# default host build aborts on that TU *before* emitting these custom-target outputs
# (builtin_types.c defines the glsl_type_builtin_* globals; the nir_* files define the
# opcode/intrinsic tables). They are independent of the failing TU, so we ninja them
# explicitly. Harmless if already present (ninja no-ops). See the noop-job rootcause doc.
GENERATED_SRCS = [
    "src/compiler/builtin_types.c",
    "src/broadcom/compiler/v3d_nir_lower_algebraic.c",
    "src/compiler/nir/nir_constant_expressions.c",
    "src/compiler/nir/nir_intrinsics.c",
    "src/compiler/nir/nir_opcodes.c",
    "src/compiler/nir/nir_opt_algebraic.c",
    "src/util/format/u_format_table.c",
]


def ensure_generated_sources():
    """ninja the custom-target sources the aux list needs (see GENERATED_SRCS).
    No-op when they already exist; warns (does not fail) if ninja isn't reachable."""
    missing = [s for s in GENERATED_SRCS
               if not os.path.exists(os.path.join(HOSTBUILD, s))]
    if not missing:
        return
    ninja = "ninja" if not os.path.exists("/tmp/mesa-pyenv/bin/ninja") \
        else "/tmp/mesa-pyenv/bin/ninja"
    env = dict(os.environ, PATH="/tmp/mesa-pyenv/bin:" + os.environ.get("PATH", ""))
    cmd = [ninja] + missing
    r = subprocess.run(cmd, cwd=HOSTBUILD, env=env,
                       capture_output=True, text=True)
    if r.returncode != 0:
        # COMPLETE output, BOTH streams: ninja relays the failing compiler's
        # diagnostics on stdout, so the old stderr[-400:] tail could hide the
        # entire cause of a generated-source failure.
        print(f"[gen] WARNING: could not generate {len(missing)} sources via ninja "
              f"(aux build may report them missing):")
        print("$ " + " ".join(cmd))
        if r.stdout.strip():
            print("--- ninja stdout ---\n" + r.stdout.strip())
        print("--- ninja stderr ---\n" + (r.stderr.strip() or "(no stderr)"))
        sys.stdout.flush()
    else:
        print(f"[gen] materialized {len(missing)} generated sources in {HOSTBUILD}")


def main():
    compile_only = "--compile-only" in sys.argv
    ensure_generated_sources()
    tmpl = template_entry()

    # 1. driver set (exact host-built variants)
    drv = driver_entries()
    so = []
    for e in drv:
        base = os.path.basename(e["file"])
        # v3dx files compiled per-version; key the obj by file+any -DV3D_VERSION
        ver = ""
        cmd = e.get("command") or " ".join(e["arguments"])
        if "V3D_VERSION=42" in cmd: ver = "_v42"
        elif "V3D_VERSION=33" in cmd: ver = "_v33"
        elif "V3D_VERSION=71" in cmd: ver = "_v71"
        out = f"{DRVOBJ}/{base}{ver}.o"
        so.append((e, e["file"], out))
    drv_ok, _ = build_objs(drv, so, DRVOBJ, "driver")

    # 2. winsys + libdrm-shim + power + harness (synth flags from the template entry)
    winsys_c  = f"{PORT}/v3d_phoenix_winsys.c"
    libdrm_c  = f"{PORT}/v3d_libdrm_shim.c"
    power_c   = f"{PORT}/v3d_phoenix_power.c"
    harness_c = f"{HARNESS_DIR}/harness_screen_create.c"
    winsys_o  = f"{DRVOBJ}/v3d_phoenix_winsys.o"
    libdrm_o  = f"{DRVOBJ}/v3d_libdrm_shim.o"
    power_o   = f"{DRVOBJ}/v3d_phoenix_power.o"
    harness_o = f"{DRVOBJ}/harness_screen_create.o"
    build_objs([tmpl] * 4,
        [(tmpl, winsys_c, winsys_o), (tmpl, libdrm_c, libdrm_o),
         (tmpl, power_c, power_o), (tmpl, harness_c, harness_o)], DRVOBJ, "winsys+shim+power+harness")

    # 2b. peripheral stubs (generic signatures -> compile warnings-off)
    stubs_c = f"{PORT}/v3d_phoenix_stubs.c"
    stubs_o = f"{DRVOBJ}/v3d_phoenix_stubs.o"
    stubs_cmd = [TC, "-c", stubs_c, "-o", stubs_o, "-std=gnu11", "-w",
                 "-include", COMPAT] + SYSROOT_OPTS
    rc = subprocess.run(stubs_cmd, capture_output=True, text=True)
    print(f"[stubs] rc={rc.returncode}")
    if rc.returncode != 0:
        # Exact command + COMPLETE output (both streams) -- a failing compile has
        # to show why, and stdout was previously dropped entirely.
        print("$ " + " ".join(stubs_cmd))
        print(rc.stderr.strip() or "(no stderr)")
        if rc.stdout.strip():
            print("--- compiler stdout ---\n" + rc.stdout.strip())
        sys.stdout.flush()

    # 2c. libvcmbox client: serialized /dev/vcmbox access so v3d_phoenix_power's
    #     GET_VIRTUAL_WH goes through the single-FIFO arbitration server (race-free)
    #     instead of driving the property mailbox directly. Plain Phoenix IPC — compile
    #     without the mesa template flags (like the stubs), only -I{VCMBOX} for its header.
    #     Source = the CANONICAL rpi4-vcmbox component in this repo (see VCMBOX above);
    #     there is deliberately no second copy of libvcmbox.c beside this script.
    vcmbox_c = f"{VCMBOX}/libvcmbox.c"
    vcmbox_o = f"{DRVOBJ}/libvcmbox.o"
    vcmbox_cmd = [TC, "-c", vcmbox_c, "-o", vcmbox_o, "-std=gnu11", "-w", f"-I{VCMBOX}"] + SYSROOT_OPTS
    rc = subprocess.run(vcmbox_cmd, capture_output=True, text=True)
    print(f"[vcmbox] rc={rc.returncode}")
    if rc.returncode != 0:
        # Exact command + COMPLETE output (both streams), as for [stubs] above.
        print("$ " + " ".join(vcmbox_cmd))
        print(rc.stderr.strip() or "(no stderr)")
        if rc.stdout.strip():
            print("--- compiler stdout ---\n" + rc.stdout.strip())
        sys.stdout.flush()

    # 3. aux objs from the link-drive list (seed from the committed manifest if the
    #    /tmp working copy is absent, so a fresh checkout builds without re-resolving)
    committed = f"{PORT}/v3d-aux-sources.txt"
    if not os.path.exists(AUXLIST) and os.path.exists(committed):
        rels = [l.strip() for l in open(committed)
                if l.strip() and not l.startswith("#")]
        open(AUXLIST, "w").write("\n".join(rels) + "\n")
        print(f"[aux] seeded {AUXLIST} from committed manifest ({len(rels)} files)")
    aux_objs = []
    if os.path.exists(AUXLIST):
        rels = [l.strip() for l in open(AUXLIST) if l.strip() and not l.startswith("#")]
        so = []
        for rel in rels:
            src = os.path.normpath(os.path.join(MESA, rel))
            e = by_abs.get(src)
            if e is None:
                print(f"  [aux] NO compile_commands entry for {rel} -- skipping")
                continue
            out = f"{AUXOBJ}/{os.path.basename(rel)}.o"
            so.append((e, src, out))
        aux_objs, _ = build_objs([x[0] for x in so], so, AUXOBJ, "aux")

    # 3a2. version-specific broadcom file(s): v3dx_counter.c @ V3D_VERSION=42 provides
    #      the REAL v3d42_perfcounters_num/get (v3d_perfcntrs_init NULLs without them,
    #      which fails v3d_screen_create at line 806). Pick the v42 compile entry.
    ver_objs = []
    for e in db:
        if e["file"].endswith("broadcom/perfcntrs/v3dx_counter.c"):
            cmd = e.get("command") or " ".join(e["arguments"])
            if "V3D_VERSION=42" in cmd:
                out = f"{DRVOBJ}/v3dx_counter_v42.o"
                ok, _ = build_objs([e], [(e, abssrc(e["file"]), out)], DRVOBJ, "perfcntr-v42")
                ver_objs += ok
                break

    # 3b. core set from the committed manifest (rebuilt here with the ABI/section flags
    #     so it gc-sections and matches the on-device ABI; replaces the old /tmp core.a)
    core_objs = []
    if os.path.exists(CORELIST):
        rels = [l.strip() for l in open(CORELIST) if l.strip() and not l.startswith("#")]
        so = []
        for rel in rels:
            src = os.path.normpath(os.path.join(MESA, rel))
            e = by_abs.get(src)
            if e is None:
                print(f"  [core] NO compile_commands entry for {rel} -- skipping")
                continue
            so.append((e, src, f"{COREOBJ}/{os.path.basename(rel)}.o"))
        core_objs, _ = build_objs([x[0] for x in so], so, COREOBJ, "core")

    if compile_only:
        print("compile-only: skipping archive + link")
        return

    # 4. one combined archive: core + aux + driver + winsys + shim + stubs
    if os.path.exists(FULL_LIB): os.remove(FULL_LIB)
    members = core_objs + aux_objs + drv_ok + ver_objs + [winsys_o, libdrm_o, power_o, stubs_o, vcmbox_o]
    subprocess.run([AR, "rcs", FULL_LIB] + members, check=True)
    sz = os.path.getsize(FULL_LIB)
    print(f"[archive] {FULL_LIB} ({len(members)} objs, {sz//1024} KiB)")

    # 5. link the harness against the combined lib with --gc-sections (drops everything
    #    unreachable from main -> measures the boot-bundle-able size).
    link = [TC, "-o", HARNESS_BIN, harness_o] + SYSROOT_OPTS + [
            "-Wl,--gc-sections", "-Wl,--start-group", FULL_LIB,
            "-Wl,--end-group", "-lm"]
    r = subprocess.run(link, capture_output=True, text=True)
    print(f"[link] rc={r.returncode}")
    # collect undefined references
    undef = set()
    for line in r.stderr.splitlines():
        if "undefined reference to" in line:
            sym = line.split("undefined reference to")[1].strip().strip("`'\"")
            undef.add(sym)
    open(UNDEF, "w").write("\n".join(sorted(undef)) + "\n")
    if r.returncode == 0:
        print(f"[link] PASS -> {HARNESS_BIN} ({os.path.getsize(HARNESS_BIN)//1024} KiB,"
              f" was ~12 MiB pre-gc-sections)")
    else:
        print(f"[link] {len(undef)} undefined symbols -> {UNDEF}")
        for s in sorted(undef)[:40]:
            print(f"    {s}")
        if len(undef) > 40:
            print(f"    ... +{len(undef)-40} more")
        # Then the exact command + the COMPLETE linker output. The undefined-symbol
        # digest above is the point of the link-drive loop, but on its own it says
        # nothing when the link died of something else (a bare "collect2: error: ld
        # returned 1 exit status" was all one Docker-on-macOS build ever printed).
        print("--- link command ---")
        print("$ " + " ".join(link))
        print("--- complete linker output ---")
        print(r.stderr.strip() or "(no stderr)")
        if r.stdout.strip():
            print("--- linker stdout ---")
            print(r.stdout.strip())
        sys.stdout.flush()


if __name__ == "__main__":
    main()
