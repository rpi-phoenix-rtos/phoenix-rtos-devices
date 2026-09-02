#!/usr/bin/env python3
"""Map undefined symbols (from a link-drive pass) to their Mesa source files.

The HOST Mesa build already compiled every needed source to a .o under HOSTBUILD.
We nm those objects to learn which .o DEFINES each symbol, then map .o -> source
via the host compile_commands.json (each entry's -o gives the obj path). This turns
/tmp/v3dphx-undef.txt into a list of mesa-relative .c paths to add to the aarch64
cross build (/tmp/v3dphx-aux.txt) — and flags symbols no host object defines
(=> need a Phoenix stub/port).

Sources under STUB_PREFIXES (disk cache / clif dump / driconf / build_id / memstream)
are NOT added to aux.txt — they go to the stub file instead. Symbols ending up in
the "STUB (no defining object)" bucket are libdrm/libc/peripheral and also stubbed.

Usage: python3 resolve-syms.py   (reads UNDEF, writes/updates AUXLIST, prints report)

Copyright 2026 Phoenix Systems
SPDX-License-Identifier: BSD-3-Clause
"""
import json, os, subprocess, glob
from pathlib import Path

# Repo root derived from this script's own location (portable across checkouts).
# Coordination-repo checkout that holds external/mesa; env-first, else derived from
# this script's position in <root>/sources/phoenix-rtos-devices/gpu/rpi4-v3d/mesa
# (same derivation as build-v3d-phoenix.py).
ROOT      = os.environ.get("PHOENIX_RPI_ROOT") or str(Path(__file__).resolve().parents[5])
MESA      = f"{ROOT}/external/mesa"
HOSTBUILD = "/tmp/mesa-v3d-build"
NM        = "nm"
UNDEF     = "/tmp/v3dphx-undef.txt"
AUXLIST   = "/tmp/v3dphx-aux.txt"

# sources we deliberately do NOT vendor — provide a Phoenix stub instead
# (peripheral: disk cache, CL debug dumper, driconf, build-id, memstream)
STUB_PREFIXES = (
    "src/util/disk_cache", "src/util/disk_cache_os",
    "src/util/u_memstream", "src/util/memstream", "src/util/build_id",
    "src/util/driconf", "src/util/xmlconfig", "src/util/log.c",
    "src/broadcom/clif/", "src/c11/impl/threads_win32",
    "src/util/os_misc", "src/util/os_time",            # Phoenix-unrecognized OS -> stub
    "src/gallium/auxiliary/renderonly/",               # KMS dumb-buffer -> stub (ro=NULL)
    "src/c11/impl/threads_posix",                      # needs pthread_mutex_timedlock -> stub mtx_/call_once
    "src/util/strtod",                                 # locale -> stub to strtod/strtof
)

# files needed wholesale whose symbols are data tables (not matched by ^SYM( ):
# the tgsi parser/info/strings (pulled by tgsi_to_nir) + util simple shaders.
SEED_SOURCES = [
    "src/gallium/auxiliary/tgsi/tgsi_strings.c",
    "src/gallium/auxiliary/tgsi/tgsi_info.c",
    "src/gallium/auxiliary/tgsi/tgsi_parse.c",
    "src/gallium/auxiliary/tgsi/tgsi_scan.c",
    "src/gallium/auxiliary/tgsi/tgsi_iterate.c",
    "src/gallium/auxiliary/tgsi/tgsi_ureg.c",
    "src/gallium/auxiliary/tgsi/tgsi_text.c",
    "src/gallium/auxiliary/tgsi/tgsi_util.c",
    "src/gallium/auxiliary/tgsi/tgsi_build.c",
    "src/gallium/auxiliary/tgsi/tgsi_sanity.c",
    "src/gallium/auxiliary/util/u_simple_shaders.c",
    "src/gallium/auxiliary/util/u_bitmask.c",
    "src/gallium/auxiliary/util/u_texture.c",
    "src/gallium/auxiliary/cso_cache/cso_hash.c",
]

# symbols whose definition the col-0 regex misses (return type on same line, etc.)
KNOWN_SOURCE = {
    "u_default_buffer_subdata":       "src/gallium/auxiliary/util/u_transfer.c",
    "u_default_texture_subdata":      "src/gallium/auxiliary/util/u_transfer.c",
    "u_default_transfer_flush_region":"src/gallium/auxiliary/util/u_transfer.c",
}


def abssrc(f):
    """compile_commands 'file' is relative to HOSTBUILD -> absolute, normalized."""
    return os.path.normpath(f if os.path.isabs(f) else os.path.join(HOSTBUILD, f))


def obj_to_source():
    """objpath(abs, normalized) -> source file, from compile_commands."""
    db = json.load(open(f"{HOSTBUILD}/compile_commands.json"))
    m = {}
    for e in db:
        cmd = e.get("command") or " ".join(e["arguments"])
        toks = cmd.split()
        for i, t in enumerate(toks):
            if t == "-o" and i + 1 < len(toks):
                obj = os.path.normpath(os.path.join(HOSTBUILD, toks[i + 1]))
                m[obj] = abssrc(e["file"])
    return m


def base_to_source():
    """object-basename (e.g. u_upload_mgr.c.o) -> source file, from compile_commands."""
    db = json.load(open(f"{HOSTBUILD}/compile_commands.json"))
    m = {}
    for e in db:
        cmd = e.get("command") or " ".join(e["arguments"])
        toks = cmd.split()
        for i, t in enumerate(toks):
            if t == "-o" and i + 1 < len(toks):
                m[os.path.basename(toks[i + 1])] = abssrc(e["file"])
    return m


def build_symmap():
    """defined-symbol -> objpath (loose .o) or 'ar:<member>.o' (archive member)."""
    symmap = {}
    DEFK = ("T", "t", "D", "d", "R", "r", "B", "b", "W", "w")
    # loose objects
    for obj in glob.glob(f"{HOSTBUILD}/**/*.o", recursive=True):
        try:
            out = subprocess.run([NM, "-g", "--defined-only", obj],
                                 capture_output=True, text=True).stdout
        except Exception:
            continue
        for line in out.splitlines():
            p = line.split()
            if len(p) >= 3 and p[1] in DEFK:
                symmap.setdefault(p[2], obj)
    # archives: nm prints "member.o:" header lines, then symbols
    narch = 0
    for arch in glob.glob(f"{HOSTBUILD}/**/*.a", recursive=True):
        narch += 1
        try:
            out = subprocess.run([NM, "-g", "--defined-only", arch],
                                 capture_output=True, text=True).stdout
        except Exception:
            continue
        member = None
        for line in out.splitlines():
            line = line.rstrip()
            if line.endswith(":") and (".o" in line or ".obj" in line):
                member = os.path.basename(line[:-1])
                continue
            p = line.split()
            if len(p) >= 3 and p[1] in DEFK and member:
                symmap.setdefault(p[2], f"ar:{member}")
    return symmap, narch


# symbols that are genuine Phoenix stubs/ports even if a definition exists in-tree
# (libdrm syncobj/prime, C lib gaps). NOTE: blake3_hash_many_neon was dropped —
# the Phoenix build forces BLAKE3_USE_NEON=0 (blake3_impl.h) so the dispatch uses
# the real portable backend and the NEON symbol is no longer referenced at all.
FORCE_STUB = {
    "drmSyncobjCreate", "drmSyncobjDestroy", "drmSyncobjWait",
    "drmSyncobjImportSyncFile", "drmSyncobjExportSyncFile",
    "drmPrimeFDToHandle", "drmPrimeHandleToFD",
    "qsort_r",
    "driCheckOption", "driParseConfigFiles", "driQueryOptionb", "driQueryOptionf",
}

# source dirs to grep for a definition when no host object provides the symbol
GREP_DIRS = ["src/gallium/auxiliary", "src/broadcom", "src/compiler",
             "src/util", "src/c11"]


def grep_define(sym):
    """Find the mesa .c that DEFINES sym (Mesa col-0 function-name style)."""
    for d in GREP_DIRS:
        base = f"{MESA}/{d}"
        if not os.path.isdir(base):
            continue
        r = subprocess.run(["grep", "-rlE", rf"^{sym}\(", base,
                            "--include=*.c"], capture_output=True, text=True)
        files = [l for l in r.stdout.splitlines() if l.strip()]
        if files:
            return os.path.relpath(files[0], MESA)
    return None


def main():
    undef = [l.strip() for l in open(UNDEF) if l.strip()]
    o2s = obj_to_source()
    b2s = base_to_source()
    symmap, narch = build_symmap()
    print(f"indexed {len(symmap)} defined symbols (loose .o + {narch} archives)")

    add_sources, stub_syms, stub_sources = set(), [], set()
    add_sources.update(SEED_SOURCES)
    for sym in undef:
        if sym in FORCE_STUB:
            stub_syms.append(sym)
            continue
        if sym in KNOWN_SOURCE:
            add_sources.add(KNOWN_SOURCE[sym]); continue
        obj = symmap.get(sym)
        src = None
        if obj is not None:
            src = b2s.get(obj[3:]) if obj.startswith("ar:") else o2s.get(obj)
        if src is not None:
            rel = os.path.relpath(src, MESA)
        else:
            rel = grep_define(sym)           # source-grep fallback (unbuilt host files)
        if rel is None:
            stub_syms.append(sym)            # truly undefined -> stub/port
            continue
        if any(rel.startswith(p) for p in STUB_PREFIXES):
            stub_sources.add(rel); stub_syms.append(sym)
            continue
        add_sources.add(rel)

    # merge with any existing aux list
    existing = set()
    if os.path.exists(AUXLIST):
        existing = {l.strip() for l in open(AUXLIST) if l.strip() and not l.startswith("#")}
    merged = existing | add_sources
    # prune anything now classified as a stub (e.g. os_misc/os_time/renderonly that
    # an earlier pass added before we decided to stub them)
    def is_stub_path(rel):
        norm = os.path.normpath(os.path.join(MESA, rel))
        r = os.path.relpath(norm, MESA)
        return any(r.startswith(p) for p in STUB_PREFIXES)
    allsrc = sorted(s for s in merged if not is_stub_path(s))
    open(AUXLIST, "w").write("\n".join(allsrc) + "\n")

    print(f"\n=== {len(add_sources)} new aux sources (now {len(allsrc)} total in {AUXLIST}) ===")
    for s in sorted(add_sources):
        print(f"  + {s}")
    print(f"\n=== {len(stub_syms)} symbols needing a stub/port ===")
    for s in stub_syms:
        tag = ""
        for src in stub_sources:
            pass
        print(f"  ? {s}")


if __name__ == "__main__":
    main()
