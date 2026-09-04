#!/usr/bin/env python3
"""Build Mesa's OpenGL frontend (st/mesa + main + program + glapi + GLSL compiler)
as a Phoenix static lib (GLQuake Path C, Phase 4), atop libv3d-phoenix.a.

Reuses build-v3d-phoenix.py's transform/abssrc/by_abs (host per-file flags ->
aarch64-phoenix). C files use aarch64-phoenix-gcc; C++ files (the GLSL compiler,
54 .cpp) use aarch64-phoenix-g++. The host build configured these files but never
ran their codegen, so we first `ninja` the generated GL headers in HOSTBUILD.

Output: tools/.gpu-libs/libGL-phoenix.a (the GL frontend; links with libv3d-phoenix.a + a
frontend/harness). Excludes x86-only sse_minmax.c (portable fallback exists).

This script is NOT part of the phoenix-rtos-devices Makefile build (see
build-v3d-phoenix.py): it drives meson/ninja on external/mesa, so it is standalone,
on the wifi/rpi4-wifi/build-standalone.sh model.

Usage: python3 build-gl-phoenix.py

Copyright 2026 Phoenix Systems
SPDX-License-Identifier: BSD-3-Clause
"""
import os, json, subprocess, glob, sys
import concurrent.futures

# pull in transform/abssrc/by_abs/TC/MESA/HOSTBUILD/COMPAT/SHIM/PORT/AR
_pre = open(os.path.join(os.path.dirname(__file__), "build-v3d-phoenix.py")).read().split("def main")[0]
exec(_pre)

TCXX = TC.rsplit("-", 1)[0] + "-g++"          # aarch64-phoenix-g++
GLOBJ = "/tmp/globj"
GL_LIB = f"{GPU_LIBS}/libGL-phoenix.a"  # stable home (was /tmp); GPU_LIBS from the v3d prelude

# GL frontend source dirs (in compile_commands), and files to exclude.
GL_DIRS = ["/src/mesa/main/", "/mesa/state_tracker/", "/mesa/program/",
           "/src/mapi/", "/compiler/glsl/",
           "/src/mesa/math/", "/src/mesa/vbo/", "/src/mesa/glapi/shared-glapi/",
           "/gallium/auxiliary/cso_cache/",
           "/gallium/auxiliary/util/u_vbuf.", "/gallium/auxiliary/util/u_threaded_context.",
           "/gallium/auxiliary/util/u_index_modify.", "/gallium/auxiliary/util/u_sampler.",
           "/c11/impl/threads_posix.", "/src/util/u_thread.", "/src/util/log.",
           "/gallium/auxiliary/indices/", "/gallium/auxiliary/util/u_gen_mipmap.",
           "/gallium/auxiliary/util/u_draw.", "/gallium/auxiliary/util/u_draw_quad.", "/gallium/auxiliary/tgsi/tgsi_from_mesa.",
           "/src/util/crc32.", "/src/util/u_cpu_detect.", "/src/util/process.",
           "/compiler/nir/nir_opt_varyings.", "/mesa/glapi/glapi/libgl_public."]
# NOTE: gallium/auxiliary/draw/ (sw vertex pipeline) + compiler/spirv/ are
# deliberately NOT included — they drag in the whole sw-render subtree (tgsi_exec,
# translate, nir_to_tgsi) that v3d (HW vertex) never uses at runtime. st references
# a handful of draw_*/spirv_to_nir symbols which gl_stubs.c stubs instead.
GL_EXCLUDE = ["sse_minmax.c", "spirv2nir.c", "/util/crc32.c"]                  # x86 SSE intrinsics; portable path exists

# generated GL entrypoint .c (not in compile_commands; compiled with a mesa/main
# template's flags). Provide glClear/_mesa_marshal_*/_mesa_enum_to_string/dispatch.
GEN_C = ["src/mesa/format_fallback.c", "src/mesa/glapi/glapi/gen/unmarshal_table.c", "src/gallium/auxiliary/u_indices_gen.c", "src/gallium/auxiliary/u_unfilled_gen.c", "src/mesa/glapi/glapi/gen/api_exec_init.c",
         "src/mesa/glapi/glapi/gen/enums.c"] + \
        [f"src/mesa/glapi/glapi/gen/marshal_generated{i}.c" for i in range(8)]

# generated GL headers to ninja-build in the host tree before compiling.
GEN_HEADERS = [
    "src/mesa/glapi/glapi/gen/dispatch.h", "src/mesa/glapi/glapi/gen/api_exec_decl.h",
    "src/mesa/glapi/glapi/gen/api_save_init.h", "src/mesa/glapi/glapi/gen/api_save.h",
    "src/mesa/glapi/glapi/gen/api_beginend_init.h", "src/mesa/glapi/glapi/gen/api_hw_select_init.h",
    "src/mesa/glapi/glapi/gen/marshal_generated.h", "src/mesa/glapi/glapi/gen/indirect.h",
    "src/mesa/glapi/glapi/gen/indirect_size.h", "src/mesa/glapi/glapi/gen/glapi_mapi_tmp.h",
    "src/mesa/glapi/shared-glapi/shared_glapi_mapi_tmp.h",
    "src/compiler/glsl/glsl_parser.h", "src/compiler/glsl/glcpp/glcpp-lex.c",
    "src/compiler/glsl/glcpp/glcpp-parse.h",
    "src/compiler/spirv/spirv_info.h", "src/compiler/spirv/vtn_generator_ids.h", "src/util/format/u_format_pack.h",
    # generated util/format sources (clean-build exposed these as MISSING -> the GL aux
    # sweep dropped cso_cache/format_fallback/... objs and build-quakespasm's pl_phoenix_glctx
    # hard-failed on util/format/u_format_gen.h; they share this HOSTBUILD, gl runs first).
    "src/util/format/u_format_gen.h", "src/util/format_srgb.c",
    # generated GLSL builtin-type sources (clean-build: MISSING -> the whole mesa
    # state_tracker st_*.c sweep dropped out of libGL, so the final GLQuake link
    # failed on st_create_context / _mesa_make_current / _mesa_noop_entrypoint).
    "src/compiler/builtin_types.h", "src/compiler/builtin_types.c",
    "src/compiler/glsl/astc_glsl.h", "src/compiler/glsl/bc1_glsl.h",
    "src/compiler/glsl/bc4_glsl.h", "src/compiler/glsl/etc2_rgba_stitch_glsl.h",
    "src/compiler/glsl/cross_platform_settings_piece_all.h",
    "src/mesa/format_info.h", "src/mesa/get_hash.h",
    "src/mesa/program/program_parse.tab.h", "src/mesa/program/lex.yy.c",
    "src/mesa/format_fallback.c", "src/mesa/glapi/glapi/gen/unmarshal_table.c",
    "src/mesa/glapi/glapi/gen/api_exec_init.c", "src/mesa/glapi/glapi/gen/enums.c",
    # GLSL-compiler custom-target sources/headers. The host GL build aborts on
    # v3d_resource.c's aarch64 `dc civac` asm (can't assemble on x86) BEFORE meson
    # emits these, so without ninja-ing them explicitly the GL aux sweep fails on
    # glsl_lexer.cpp (MISSING) + glsl_to_nir.cpp (float64_glsl.h) +
    # ir_constant_expression.cpp (ir_expression_operation_constant.h) + ir.cpp
    # (ir_expression_operation_strings.h). ir_expression_operation.h is their shared
    # dep. glsl_lexer.cpp lives under GL_DIRS (/compiler/glsl/) so once generated it
    # is picked up by the compile_commands sweep; the rest are headers. Mirrors the
    # ensure_generated_sources() fix in build-v3d-phoenix.py.
    "src/compiler/ir_expression_operation.h",
    "src/compiler/glsl/ir_expression_operation_constant.h",
    "src/compiler/glsl/ir_expression_operation_strings.h",
    "src/compiler/glsl/float64_glsl.h",
    "src/compiler/glsl/glsl_lexer.cpp",
    # gallium index-buffer codegen the GEN_C list compiles (were "[gen-c] MISSING").
    "src/gallium/auxiliary/u_indices_gen.c", "src/gallium/auxiliary/u_unfilled_gen.c",
] + [f"src/mesa/glapi/glapi/gen/marshal_generated{i}.c" for i in range(8)]


def gen_headers():
    cmd = ["ninja"] + GEN_HEADERS
    r = subprocess.run(cmd, cwd=HOSTBUILD, capture_output=True, text=True)
    print(f"[gen-headers] ninja rc={r.returncode}")
    if r.returncode != 0:
        # Exact command + COMPLETE output, BOTH streams: ninja relays the failing
        # compiler's diagnostics on stdout, so the old stderr[-400:] tail could
        # hide the entire cause of a generated-header failure.
        print("$ " + " ".join(cmd))
        if r.stdout.strip():
            print("--- ninja stdout ---\n" + r.stdout.strip())
        print("--- ninja stderr ---\n" + (r.stderr.strip() or "(no stderr)"))
        sys.stdout.flush()


def fail_detail(cmd, r):
    """The exact command + the COMPLETE stderr (and stdout if any) of a failed
    compile, as one printable block. Never a filtered or truncated subset: a
    build that fails has to show why."""
    detail = "$ " + " ".join(cmd) + "\n" + (r.stderr.strip() or "(no stderr)")
    if r.stdout.strip():
        detail += "\n--- compiler stdout ---\n" + r.stdout.strip()
    return detail


def main():
    gen_headers()
    db = json.load(open(f"{HOSTBUILD}/compile_commands.json"))
    files = [e for e in db
             if any(d in e["file"] for d in GL_DIRS)
             and not any(x in e["file"] for x in GL_EXCLUDE)]
    os.makedirs(GLOBJ, exist_ok=True)
    objs, fails = [], []
    # Parallel compile across all cores: each cmd is an independent gcc subprocess writing a distinct
    # .o. Build the work list first, then run it in a cpu_count() ThreadPool (the GIL doesn't block
    # subprocess I/O). ex.map preserves order -> deterministic archive. Was a serial one-at-a-time
    # loop (the single-core bottleneck in the showcase/GPU build phase).
    def _run(cmd, out, label):
        r = subprocess.run(cmd, cwd=HOSTBUILD, capture_output=True, text=True)
        if r.returncode == 0:
            return (out, None)
        # Carry the exact command + the COMPLETE compiler output. This used to keep
        # only the FIRST `error:` line, which threw away the diagnostic that
        # actually mattered whenever that line was not the informative one.
        return (out, (label, fail_detail(cmd, r)))

    work = []
    for e in files:
        src = abssrc(e["file"])
        out = f"{GLOBJ}/{os.path.basename(src)}.o"
        cmd = transform(e, src, out)
        if src.endswith(".cpp"):
            cmd[0] = TCXX
        work.append((cmd, out, os.path.relpath(src, MESA)))
    # generated GL entrypoint .c — compile with a mesa/main template's flags.
    tmpl = next(e for e in db if e["file"].endswith("src/mesa/main/context.c"))
    for rel in GEN_C:
        src = os.path.normpath(os.path.join(HOSTBUILD, rel))
        if not os.path.exists(src):
            print(f"  [gen-c] MISSING {rel}"); continue
        out = f"{GLOBJ}/{os.path.basename(rel)}.o"
        work.append((transform(tmpl, src, out), out, rel))
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, os.cpu_count() or 1)) as _ex:
        for out, fail in _ex.map(lambda w: _run(*w), work):
            (fails.append(fail) if fail else objs.append(out))

    # Phoenix shims, folded into libGL so a boot program links just the two libs:
    # mathshim (real float-math libm lacks; compiled WITHOUT the compat shim) and
    # gl_stubs (sw-path stubs + libc gaps). Neither conflicts with libGL's own
    # symbols (they supply what libGL/libv3d reference but lack).
    for shim, warn in (("v3d_phoenix_mathshim.c", []), ("gl_stubs.c", ["-w"])):
        out = f"{GLOBJ}/{shim}.o"
        cmd = [TC, "-O2", "-c"] + SYSROOT_OPTS + warn + [f"{PORT}/{shim}", "-o", out]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            objs.append(out)
        else:
            fails.append((shim, fail_detail(cmd, r)))

    print(f"[gl] OK={len(objs)} FAIL={len(fails)} (of {len(files)+len(GEN_C)})")
    for f, detail in fails:
        first = next((l for l in detail.splitlines() if "error:" in l), "?")
        print(f"  FAIL {f} :: " + first.split('error:')[-1].strip()[:70])
    # ...then the COMPLETE output for each failure. The one-line-per-TU table above
    # is a useful digest but hid the real cause whenever the first `error:` line was
    # not the informative one (or there was none at all).
    if fails:
        print("--- full compiler output per failure ---")
        for f, detail in fails:
            print(f"===== {f} =====\n{detail}\n")
        sys.stdout.flush()
    if objs:
        if os.path.exists(GL_LIB):
            os.remove(GL_LIB)
        subprocess.run([AR, "rcs", GL_LIB] + objs, check=True)
        print(f"[archive] {GL_LIB} ({len(objs)} objs, {os.path.getsize(GL_LIB)//1024} KiB)")


if __name__ == "__main__":
    main()
