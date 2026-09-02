/*
 * v3dv_gap_stubs.c — libc / libdrm gap stubs surfaced by the V3DV Tier-0 link-drive.
 *
 * These are symbols the Vulkan runtime/util references that Phoenix's libc or our
 * libdrm shim does not provide. Each is either trivially derivable (the C23 time
 * helpers from clock_gettime) or inert on a single-client, no-KMS Phoenix target.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

/* os_get_option_secure (src/util/os_misc.c): getenv() unless in a secure-execution
 * context. We don't pull os_misc.c (it collides with v3d_phoenix_stubs.c's os_* set);
 * provide just this one symbol. No setuid distinction on Phoenix -> plain getenv. */
extern char *getenv(const char *name);
const char *os_get_option_secure(const char *name)
{
	return getenv(name);
}

/* libdrm: "is this fd a KMS (modesetting) node?" — Phoenix has no KMS node; the
 * V3D render path never needs one. Always 0. */
int drmIsKMS(int fd)
{
	(void)fd;
	return 0;
}

/* C23 <threads.h>/<time.h> helper: timespec_get(ts, TIME_UTC)-equivalent the c11
 * threads_posix.c uses for cnd_timedwait deadlines. Phoenix libc lacks the C23 name;
 * derive from clock_gettime(CLOCK_REALTIME). Returns base on success (C23 semantics). */
#ifndef TIME_UTC
#define TIME_UTC 1
#endif
int c23_timespec_get(struct timespec *ts, int base)
{
	if (base != TIME_UTC)
		return 0;
	if (clock_gettime(CLOCK_REALTIME, ts) != 0)
		return 0;
	return base;
}

/* POSIX clock_getres — Phoenix libc does not export it; report 1 ns resolution
 * (the runtime only uses it informationally for timeout granularity). */
#ifndef PHX_HAVE_CLOCK_GETRES
int clock_getres(clockid_t clk_id, struct timespec *res)
{
	(void)clk_id;
	if (res) {
		res->tv_sec = 0;
		res->tv_nsec = 1;
	}
	return 0;
}
#endif

/* ---- build-id (ELF .note.gnu.build-id) ----
 * src/util/build_id.c walks the ELF program headers (Elf_Nhdr / dl_phdr_info.dlpi_phdr)
 * to find the build-id note; Phoenix's libc/dl headers lack those fields, so build_id.c
 * does not cross-compile. v3dv init_uuids() (v3dv_device.c:842) uses the build-id only
 * to seed the pipeline-cache UUID — a CONSTANT UUID is fine for first-light (it only
 * affects detecting a STALE on-disk pipeline cache, which Phoenix has none of). It does,
 * however, HARD-FAIL device creation if build_id_find_nhdr_for_addr returns NULL or the
 * length is < 20 (a SHA). libv3d-phoenix.a already defines build_id_find_nhdr_for_addr +
 * build_id_data (the GL build); we supply the two missing leaves with a fixed 20-byte
 * synthetic id so init_uuids takes its success path.
 *
 * TIER-1 RISK (boot-verify): the libv3d-phoenix.a build_id_find_nhdr_for_addr does the
 * real ELF walk and may return NULL on the Phoenix ELF -> init_uuids fails BEFORE these
 * leaves are reached. If vkCreateDevice fails at init_uuids, override
 * build_id_find_nhdr_for_addr here too (weak) to return a fixed note. See the Tier-0
 * progress doc.
 */
struct build_id_note;   /* opaque to us; only its address is passed around */
static const uint8_t v3dv_fixed_build_id[20] = {
	0x70, 0x68, 0x6f, 0x65, 0x6e, 0x69, 0x78, 0x76, 0x33, 0x64,  /* "phoenixv3d" */
	0x76, 0x74, 0x69, 0x65, 0x72, 0x30, 0x00, 0x01, 0x02, 0x03,  /* "vtier0"+pad */
};

__attribute__((weak))
unsigned build_id_length(const struct build_id_note *note)
{
	(void)note;
	return (unsigned)sizeof(v3dv_fixed_build_id);   /* 20 == SHA length */
}

__attribute__((weak))
void copy_build_id_to_sha1(uint8_t sha1[20], const struct build_id_note *note)
{
	(void)note;
	memcpy(sha1, v3dv_fixed_build_id, sizeof(v3dv_fixed_build_id));
}

/* libv3d-phoenix.a's real build_id_find_nhdr_for_addr does a runtime ELF program-header
 * walk for the .note.gnu.build-id, which does not resolve on Phoenix's statically-loaded
 * ELF -> returns NULL -> init_uuids() hard-fails create_physical_device. Override it
 * (linked first, --allow-multiple-definition) to return a non-NULL sentinel; the length/
 * copy stubs above ignore the note pointer and use the fixed 20-byte id. */
const struct build_id_note *build_id_find_nhdr_for_addr(const void *addr);
const struct build_id_note *build_id_find_nhdr_for_addr(const void *addr)
{
	(void)addr;
	return (const struct build_id_note *)v3dv_fixed_build_id;
}

/* ---- driconf (xmlconfig) ----
 * src/util/xmlconfig.c needs libexpat to parse drirc XML; Phoenix has none. driconf is
 * purely an OPTIONAL per-app option-OVERRIDE layer — with an empty option cache every
 * option takes its compiled-in default, which is the default behavior anyway. Stub the
 * three referenced entrypoints to leave the cache empty/zeroed. */
typedef struct driOptionCache driOptionCache;
struct driOptionDescription;

__attribute__((weak))
void driParseOptionInfo(driOptionCache *info,
                        const struct driOptionDescription *configOptions,
                        unsigned numOptions)
{
	/* No-op: callers pass a vk_zalloc'd (zeroed) driOptionCache, so leaving it
	 * untouched yields an empty cache -> driQueryOption* return compiled-in defaults.
	 * TIER-1 RISK: if a caller relies on driParseOptionInfo populating the table,
	 * driQueryOption* would read an empty cache. Verify no required option on HW. */
	(void)info; (void)configOptions; (void)numOptions;
}

__attribute__((weak))
void driDestroyOptionInfo(driOptionCache *info)
{
	(void)info;
}

__attribute__((weak))
void driDestroyOptionCache(driOptionCache *cache)
{
	(void)cache;
}

/* Newly referenced after the upstream mesa merge (v3dv reads a driconf STRING option at
 * device create). With the empty cache above there is no override, so return "" -> the
 * caller falls back to its compiled-in default. Non-NULL so string callers never crash. */
__attribute__((weak))
char *driQueryOptionstr(const driOptionCache *cache, const char *name)
{
	(void)cache; (void)name;
	return (char *)"";
}
