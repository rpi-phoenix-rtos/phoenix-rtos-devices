#!/usr/bin/env python3
"""Build Mesa's V3DV (Vulkan for Broadcom V3D) driver + Vulkan runtime as a Phoenix
static lib + link-drive a vkCreateInstance/vkCreateDevice harness (Vulkan Tier 0).

Mirrors build-v3d-phoenix.py / build-gl-phoenix.py: the HOST Mesa build supplies
per-file compile flags via compile_commands.json; we re-emit each compile with the
Phoenix toolchain (force-include phoenix_mesa_compat.h, prepend shim-include/), then
link-drive to discover the real symbol closure (undef -> add the defining .c to the
aux list -> re-run, converges in a few passes).

CRUCIAL difference from the GL build: the host build dir is a SEPARATE Vulkan-enabled
meson configuration (HOSTBUILD below = /tmp/mesa-v3dv-build, configured with
-Dvulkan-drivers=broadcom). The gallium GL build dir (/tmp/mesa-v3d-build) is left
untouched. The broadcom back-end (v3d_compile/CLE/QPU/perfcntrs/nir) is reused AS-IS
from tools/.gpu-libs/libv3d-phoenix.a — it is front-end-agnostic and already cross-built; we only
add the Vulkan front-end + runtime objects on top (plan section 4.2).

Bulk-compiled (genuinely new, no overlap with libv3d-phoenix.a):
  - src/broadcom/vulkan/*.c            (V3D_VERSION=42 only for v3dvx_*; v3dv_wsi.c
                                        EXCLUDED -> its WSI entrypoints are stubbed)
  - src/vulkan/runtime/*.c             (vk_device/queue/cmd_buffer/sync/... + generated)
  - src/vulkan/util/*.c                (+ generated vk_enum_to_str / vk_dispatch_table)
  - generated v3dv_entrypoints.c

Link-driven via the aux list (linker pulls only referenced members):
  - src/compiler/spirv/*               (spirv_to_nir — vkQuake ships SPIR-V; GL excluded
                                        this, V3DV needs it)
  - any nir/util delta not already in libv3d-phoenix.a

Output: tools/.gpu-libs/libv3dv-phoenix.a, link-driven against libv3d-phoenix.a + the Phoenix
shims to a vk_icd harness ELF with (ideally) 0 undefined symbols.

PREREQUISITE — the Vulkan-enabled host Mesa build (one-time, do NOT disturb the GL
build dir /tmp/mesa-v3d-build):

    # meson on this host needs a python with mako+pyyaml; system python3.14 lacks mako.
    uv venv /tmp/mesa-pyenv --python 3.14
    uv pip install --python /tmp/mesa-pyenv/bin/python mako pyyaml packaging meson ninja
    cd external/mesa
    PATH=/tmp/mesa-pyenv/bin:$PATH /tmp/mesa-pyenv/bin/meson setup /tmp/mesa-v3dv-build \\
        -Dgallium-drivers=v3d -Dvulkan-drivers=broadcom -Dplatforms= \\
        -Dglx=disabled -Degl=disabled -Dgbm=disabled -Dvideo-codecs= -Dbuildtype=release
    # materialize the generated sources (entrypoints, dispatch tables, spirv_info.c, ...):
    cd /tmp/mesa-v3dv-build && PATH=/tmp/mesa-pyenv/bin:$PATH ninja \\
        src/broadcom/vulkan/libvulkan_broadcom.so

This script is NOT part of the phoenix-rtos-devices Makefile build (see
build-v3d-phoenix.py): it drives meson/ninja on external/mesa, so it is standalone,
on the wifi/rpi4-wifi/build-standalone.sh model.

Usage:  python3 build-v3dv-phoenix.py [--compile-only]

Copyright 2026 Phoenix Systems
SPDX-License-Identifier: BSD-3-Clause
"""
import json, os, subprocess, sys

# Reuse transform/abssrc/by_abs/TC/AR/MESA/COMPAT/SHIM/PORT/ABI_FLAGS from the gallium
# build script, but OVERRIDE HOSTBUILD to the Vulkan-enabled meson dir. We exec the
# pre-main() prelude of build-v3d-phoenix.py, then rebind HOSTBUILD + reload the db.
_pre = open(os.path.join(os.path.dirname(__file__), "build-v3d-phoenix.py")).read().split("def main")[0]
exec(_pre)

# --- Vulkan-specific config (overrides the gallium build's globals) ---
HOSTBUILD = "/tmp/mesa-v3dv-build"           # Vulkan-enabled host meson build
db        = json.load(open(f"{HOSTBUILD}/compile_commands.json"))
by_abs    = {abssrc(e["file"]): e for e in db}

V3D_LIB   = f"{GPU_LIBS}/libv3d-phoenix.a"   # reused broadcom back-end (built by build-v3d-phoenix.py); GPU_LIBS from prelude
V3DV_OBJ  = "/tmp/v3dvphx-obj"
V3DV_AUXOBJ = "/tmp/v3dvphx-auxobj"
V3DV_LIB  = f"{GPU_LIBS}/libv3dv-phoenix.a"  # stable home (was /tmp)
V3DV_AUXLIST = "/tmp/v3dvphx-aux.txt"
V3DV_AUX_COMMITTED = f"{PORT}/v3dv-aux-sources.txt"
V3DV_UNDEF = "/tmp/v3dvphx-undef.txt"
V3DV_HARNESS_BIN = "/tmp/v3dvphx-harness"

# Front-end source dirs to bulk-compile (these are NOT in libv3d-phoenix.a).
V3DV_DIRS = ["/src/broadcom/vulkan/", "/src/vulkan/runtime/", "/src/vulkan/util/"]
# v3dv_wsi.c drags in src/vulkan/wsi -> wsi_common_drm -> drmPrime/display deps the
# plan bypasses (section 3). Exclude it; the swapchain/surface entrypoints it would
# define are stubbed in vk_icd_link.c for Tier 0.
V3DV_EXCLUDE = ["v3dv_wsi.c", "vk_android.c", "vk_wsi", "wsi_common"]


def v3dv_entries():
    """Front-end TUs to bulk-compile. v3dvx_* per-version: V3D_VERSION=42 only.
    Match against the ABSOLUTE source path: generated files (vk_common_entrypoints.c,
    vk_enum_to_str.c, ...) have *relative* 'file' fields (src/vulkan/...) that lack
    the leading '/src/' the dir patterns expect — abssrc() normalizes both forms."""
    out = []
    for e in db:
        f = e["file"]
        if not f.endswith(".c"):
            continue
        af = abssrc(f)
        if not any(d in af for d in V3DV_DIRS):
            continue
        if any(x in af for x in V3DV_EXCLUDE):
            continue
        base = os.path.basename(f)
        cmd = e.get("command") or " ".join(e["arguments"])
        if base.startswith("v3dvx_"):
            # per-version file: take V3D_VERSION=42 only (drop 71)
            if "V3D_VERSION=42" not in cmd:
                continue
        out.append(e)
    return out


def gen_entrypoints_entry():
    """The host-generated v3dv_entrypoints.c (relative path in compile_commands)."""
    for e in db:
        if e["file"].endswith("broadcom/vulkan/v3dv_entrypoints.c"):
            return e
    return None


def dump_failure(label, cmd, stderr, stdout=""):
    """Print the exact command + the COMPLETE stderr (and stdout if non-empty) of a
    failed compile.

    Every failure path below used to print only a 4-8 line tail of stderr (or its
    last 400-600 chars), which can cut off the diagnostic that actually explains
    the failure -- a build that fails has to show why."""
    print(label)
    print("$ " + " ".join(cmd))
    print(stderr.strip() or "(no stderr)")
    if stdout.strip():
        print("--- compiler stdout ---")
        print(stdout.strip())
    sys.stdout.flush()


def build_set(entries, objdir, label):
    os.makedirs(objdir, exist_ok=True)
    so = []
    for e in entries:
        src = abssrc(e["file"])
        base = os.path.basename(src)
        ver = "_v42" if "V3D_VERSION=42" in (e.get("command") or " ".join(e["arguments"])) and base.startswith("v3dvx_") else ""
        out = f"{objdir}/{base}{ver}.o"
        so.append((e, src, out))
    return build_objs([x[0] for x in so], so, objdir, label)


def main():
    compile_only = "--compile-only" in sys.argv

    # 1. front-end bulk compile (broadcom/vulkan + vulkan/runtime + vulkan/util)
    fe = v3dv_entries()
    fe_ok, fe_fail = build_set(fe, V3DV_OBJ, "v3dv-frontend")

    # 1b. generated entrypoint dispatch (v3dv_entrypoints.c)
    gen = gen_entrypoints_entry()
    gen_ok = []
    if gen:
        src = abssrc(gen["file"])
        out = f"{V3DV_OBJ}/v3dv_entrypoints.c.o"
        rc, err, cmd = compile_one(gen, src, out)
        if rc == 0:
            gen_ok.append(out)
            print("[v3dv-gen] v3dv_entrypoints.c OK")
        else:
            dump_failure("[v3dv-gen] v3dv_entrypoints.c FAIL", cmd, err)
    else:
        print("[v3dv-gen] no v3dv_entrypoints.c in compile_commands")

    # 2. aux objs from the link-drive list (spirv_to_nir + nir/util deltas). Seed
    #    from the committed manifest if the /tmp working copy is absent.
    if not os.path.exists(V3DV_AUXLIST) and os.path.exists(V3DV_AUX_COMMITTED):
        rels = [l.strip() for l in open(V3DV_AUX_COMMITTED)
                if l.strip() and not l.startswith("#")]
        open(V3DV_AUXLIST, "w").write("\n".join(rels) + "\n")
        print(f"[v3dv-aux] seeded {V3DV_AUXLIST} from committed manifest ({len(rels)} files)")
    aux_objs = []
    if os.path.exists(V3DV_AUXLIST):
        rels = [l.strip() for l in open(V3DV_AUXLIST) if l.strip() and not l.startswith("#")]
        so = []
        for rel in rels:
            # source path first, then the HOSTBUILD path (generated files like
            # spirv_info.c live under the build dir, not the mesa source tree)
            src = os.path.normpath(os.path.join(MESA, rel))
            e = by_abs.get(src) or by_abs.get(os.path.normpath(os.path.join(HOSTBUILD, rel)))
            if e is None:
                print(f"  [v3dv-aux] NO compile_commands entry for {rel} -- skipping")
                continue
            out = f"{V3DV_AUXOBJ}/{os.path.basename(rel)}.o"
            so.append((e, abssrc(e["file"]), out))
        if so:
            aux_objs, _ = build_objs([x[0] for x in so], so, V3DV_AUXOBJ, "v3dv-aux")

    # 3. Phoenix Vulkan shims. v3dv_libdrm_shim.c is plain C (drmSyncobj* surface) and
    #    compiles with the compat force-include; vk_icd_link.c + the harness need the
    #    Vulkan headers, so they're compiled with a v3dv_device.c template's flags below.
    tmpl = by_abs.get(os.path.normpath(f"{MESA}/src/broadcom/vulkan/v3dv_device.c"))
    shim_objs = []
    out = f"{V3DV_OBJ}/v3dv_libdrm_shim.c.o"
    cmd = ([TC, "-c", f"{PORT}/v3dv_libdrm_shim.c", "-o", out, f"-I{SHIM}",
            f"-I{PORT}", "-std=gnu11", "-include", COMPAT] + ABI_FLAGS + ["-w"])
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode == 0:
        shim_objs.append(out); print("[v3dv-shim] v3dv_libdrm_shim.c OK")
    else:
        dump_failure("[v3dv-shim] v3dv_libdrm_shim.c FAIL", cmd, r.stderr, r.stdout)

    # vk_icd_link.c needs <vulkan/vulkan.h> -> compile with the v3dv_device template.
    if tmpl:
        out = f"{V3DV_OBJ}/vk_icd_link.c.o"
        rc, err, cmd = compile_one(tmpl, f"{PORT}/vk_icd_link.c", out)
        if rc == 0:
            shim_objs.append(out); print("[v3dv-shim] vk_icd_link.c OK")
        else:
            dump_failure("[v3dv-shim] vk_icd_link.c FAIL", cmd, err)

    # v71 trap-stubs (dead V3D-7.1 dispatch branch) + libc/libdrm gap stubs. Plain C,
    # no Mesa headers -> warnings-off, no compat force-include needed.
    for shim in ("v3dv_v71_stubs.c", "v3dv_gap_stubs.c"):
        out = f"{V3DV_OBJ}/{shim}.o"
        cmd = ([TC, "-c", f"{PORT}/{shim}", "-o", out, f"-I{SHIM}", f"-I{PORT}",
                "-std=gnu11", "-w"] + ABI_FLAGS)
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            shim_objs.append(out); print(f"[v3dv-shim] {shim} OK")
        else:
            dump_failure(f"[v3dv-shim] {shim} FAIL", cmd, r.stderr, r.stdout)

    # The REAL Phoenix V3D winsys (phoenix_v3d_ioctl + ioc_create_bo + lazy winsys_init that
    # powers V3D and builds the flat MMU) — the SAME backend the GL gallium port uses. Without
    # it, V3DV's v3dv_bo_alloc (-> v3d_ioctl DRM_V3D_CREATE_BO -> drmIoctl -> phoenix_v3d_ioctl)
    # has no allocator, so device-create's noop-job CL BO is never allocated and binning_prolog
    # faults on a NULL bcl. These need Mesa flags (drm_v3d_* structs, broadcom headers) so they
    # are compiled with the v3dv_device.c template, like the harness/aux — not the plain-C path.
    # See docs/inprogress/2026-06-18-vulkan-v3dv-noop-job-rootcause.md.
    if tmpl:
        for wsrc in ("v3d_phoenix_winsys.c", "v3d_phoenix_power.c"):
            out = f"{V3DV_OBJ}/{wsrc}.o"
            rc, err, cmd = compile_one(tmpl, f"{PORT}/{wsrc}", out)
            if rc == 0:
                shim_objs.append(out); print(f"[v3dv-winsys] {wsrc} OK")
            else:
                dump_failure(f"[v3dv-winsys] {wsrc} FAIL", cmd, err)
    else:
        print("[v3dv-winsys] no template -> winsys NOT built (BO alloc will fail)")

    # harness compiled with a v3dv_device.c template's flags (Vulkan headers on -I path)
    harness_o = f"{V3DV_OBJ}/v3dv_harness.o"
    if tmpl:
        # The harness is a link-drive diagnostic and stays in the coordination repo's
        # tools/ tree (HARNESS_DIR, from the prelude) -- see build-v3d-phoenix.py.
        rc, err, cmd = compile_one(tmpl, f"{HARNESS_DIR}/v3dv_harness.c", harness_o)
        if rc == 0:
            print("[v3dv-harness] OK")
        else:
            dump_failure("[v3dv-harness] FAIL", cmd, err)
            harness_o = None
    else:
        print("[v3dv-harness] no v3dv_device.c template entry"); harness_o = None

    print(f"\n[v3dv] frontend OK={len(fe_ok)} FAIL={len(fe_fail)}  gen={len(gen_ok)}  "
          f"aux={len(aux_objs)}  shims={len(shim_objs)}")

    if compile_only:
        print("compile-only: skipping archive + link")
        return

    # 4. archive the Vulkan front-end + runtime + shims (NOT the back-end; that stays
    #    in libv3d-phoenix.a and is added to the link group).
    if os.path.exists(V3DV_LIB):
        os.remove(V3DV_LIB)
    members = fe_ok + gen_ok + aux_objs + shim_objs
    subprocess.run([AR, "rcs", V3DV_LIB] + members, check=True)
    print(f"[archive] {V3DV_LIB} ({len(members)} objs, {os.path.getsize(V3DV_LIB)//1024} KiB)")

    if not harness_o:
        print("[link] skipped (harness did not compile)")
        return

    # 5. link-drive: harness + libv3dv-phoenix.a + libv3d-phoenix.a (back-end) + libm.
    #    --start-group/--end-group because the two archives reference each other.
    #
    #    --allow-multiple-definition: V3DV and the gallium back-end each carry their OWN
    #    copy of 3 per-version helpers (v3d42_job_emit_enable_double_buffer,
    #    v3d42_get_internal_type_bpp_for_output_format, v3d42_tfu_supports_tex_format) —
    #    V3DV from src/broadcom/vulkan/v3dvx_{cmd_buffer,formats}.c, the back-end from
    #    src/gallium/drivers/v3d/v3dx_{job,format_table}.c. Upstream these two drivers
    #    are NEVER in one link; our port links them together, surfacing the collision.
    #    With link order libv3dv FIRST, the linker binds V3DV's copy (correct: V3DV must
    #    use its own front-end helpers, not gallium's). CAVEAT for later tiers: this is
    #    tolerated ONLY for these 3 enumerated dups; any NEW multiple-definition during
    #    Tier 1-5 must be investigated, not absorbed. See the Tier-0 progress doc.
    link = [TC, "-o", V3DV_HARNESS_BIN, harness_o,
            "-Wl,--gc-sections", "-Wl,--allow-multiple-definition",
            "-Wl,--start-group", V3DV_LIB, V3D_LIB,
            "-Wl,--end-group", "-lm"]
    r = subprocess.run(link, capture_output=True, text=True)
    print(f"[link] rc={r.returncode}")
    undef = set()
    for line in r.stderr.splitlines():
        if "undefined reference to" in line:
            sym = line.split("undefined reference to")[1].strip().strip("`'\"")
            undef.add(sym)
    open(V3DV_UNDEF, "w").write("\n".join(sorted(undef)) + "\n")
    if r.returncode == 0:
        print(f"[link] PASS -> {V3DV_HARNESS_BIN} ({os.path.getsize(V3DV_HARNESS_BIN)//1024} KiB)")
    else:
        print(f"[link] {len(undef)} undefined symbols -> {V3DV_UNDEF}")
        for s in sorted(undef)[:60]:
            print(f"    {s}")
        if len(undef) > 60:
            print(f"    ... +{len(undef)-60} more")
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
