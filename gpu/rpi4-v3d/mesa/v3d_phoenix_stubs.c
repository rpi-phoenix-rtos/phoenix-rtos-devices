/*
 * v3d_phoenix_stubs.c — Phoenix glue for Mesa peripherals excluded from the Phoenix
 * v3d driver subset (GLQuake Path C). Most are no-ops off the CL/shader-generation
 * path (CL debug dumper, build-id, logging, memstream, driconf). The disk shader
 * cache, however, is a REAL minimal implementation here: it persists compiled QPU
 * shaders to a directory so GL apps skip V3D recompilation on subsequent launches.
 * Plus a real qsort_r (Phoenix libc has only plain qsort).
 *
 * Linkage is by symbol name only, so generic signatures here resolve the driver's
 * references compiled against the real prototypes. Compiled with warnings off.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

/* --- disk shader cache: minimal persistent on-disk cache ------------------
 *
 * Mesa's v3d gallium driver (compiled with -DENABLE_SHADER_CACHE) computes a
 * cache_key (a uint8_t[32] BLAKE3 digest, CACHE_KEY_SIZE == BLAKE3_KEY_LEN)
 * from each shader and calls put()/get() keyed by it. We own BOTH ends, so we
 * do not need Mesa's multi-file on-disk format — one file per key, named by the
 * key's 64-char hex, is a correct self-consistent scheme.
 *
 * Cache directory: MESA_SHADER_CACHE_DIR, else MESA_GLSL_CACHE_DIR, else the
 * cwd-relative "./.mesa-shader-cache" (on Phoenix netboot the cwd is the NFS
 * export root, which is persistent across boots — mirrors how STK's cwd-relative
 * texture cache persists). A "/v<N>" version segment is appended so the on-disk
 * format / driver ABI can be invalidated by bumping V3D_PHX_CACHE_VERSION.
 *
 * NB Phoenix has no ELF build-id (build_id_* below are NULL stubs), so unlike
 * upstream Mesa this cache is NOT automatically invalidated when libv3d is
 * rebuilt. After any driver change that alters QPU codegen or the v3d_prog_data
 * blob layout, wipe the cache dir (rm -rf) or bump V3D_PHX_CACHE_VERSION, or a
 * stale entry could feed an incompatible QPU blob back to the GPU.
 */
#define V3D_PHX_CACHE_VERSION 1
#define V3D_CACHE_KEY_SIZE    32              /* == BLAKE3_KEY_LEN */
#define V3D_CACHE_DIR_MAX     1024

struct disk_cache {
	char dir[V3D_CACHE_DIR_MAX];             /* cache directory, no trailing '/' */
};

/* One-shot BLAKE3 from the in-tree util (already linked into this archive). The
 * stubs TU is compiled without Mesa's -I paths, so declare the prototype here. */
extern void _mesa_blake3_compute(const void *data, size_t size,
                                 uint8_t result[V3D_CACHE_KEY_SIZE]);

/* mkdir -p: create `path` and any missing parents. EEXIST is success. */
static int v3d_cache_mkdir_p(const char *path)
{
	char tmp[V3D_CACHE_DIR_MAX];
	size_t len = strlen(path);
	if (len == 0 || len >= sizeof(tmp))
		return -1;
	memcpy(tmp, path, len + 1);
	if (tmp[len - 1] == '/')
		tmp[len - 1] = '\0';
	for (char *p = tmp + 1; *p != '\0'; p++) {
		if (*p == '/') {
			*p = '\0';
			if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
				return -1;
			*p = '/';
		}
	}
	if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
		return -1;
	return 0;
}

/* <dir>/<64-hex-of-key> into `out`. */
static void v3d_cache_key_path(const struct disk_cache *c,
                               const unsigned char *key, char *out, size_t outsz)
{
	static const char hexd[] = "0123456789abcdef";
	char hex[V3D_CACHE_KEY_SIZE * 2 + 1];
	for (int i = 0; i < V3D_CACHE_KEY_SIZE; i++) {
		hex[2 * i]     = hexd[(key[i] >> 4) & 0xf];
		hex[2 * i + 1] = hexd[key[i] & 0xf];
	}
	hex[V3D_CACHE_KEY_SIZE * 2] = '\0';
	snprintf(out, outsz, "%s/%s", c->dir, hex);
}

void *disk_cache_create(const char *name, const char *renderer, uint64_t driver_flags)
{
	(void)name;
	(void)renderer;
	(void)driver_flags;

	const char *base = getenv("MESA_SHADER_CACHE_DIR");
	if (base == NULL || base[0] == '\0')
		base = getenv("MESA_GLSL_CACHE_DIR");
	if (base == NULL || base[0] == '\0')
		base = "./.mesa-shader-cache";

	struct disk_cache *c = calloc(1, sizeof(*c));
	if (c == NULL)
		return NULL;

	/* base + "/v" + version -> the actual cache dir (version-segmented) */
	if ((size_t)snprintf(c->dir, sizeof(c->dir), "%s/v%d", base,
	                     V3D_PHX_CACHE_VERSION) >= sizeof(c->dir)) {
		free(c);
		return NULL;                     /* path too long */
	}

	/* If the dir can't be created, disable caching (Mesa treats NULL as "off"). */
	if (v3d_cache_mkdir_p(c->dir) != 0) {
		free(c);
		return NULL;
	}
	return c;
}

void disk_cache_destroy(void *cache)
{
	free(cache);
}

void disk_cache_compute_key(void *c, const void *data, size_t size,
                            unsigned char *key)
{
	(void)c;
	_mesa_blake3_compute(data, size, key);
}

void *disk_cache_get(void *cache, const unsigned char *key, size_t *size)
{
	if (size != NULL)
		*size = 0;
	struct disk_cache *c = cache;
	if (c == NULL)
		return NULL;

	char path[V3D_CACHE_DIR_MAX + 80];
	v3d_cache_key_path(c, key, path, sizeof(path));

	FILE *f = fopen(path, "rb");            /* Phoenix libc rejects "rt" — use "rb" */
	if (f == NULL)
		return NULL;                        /* miss */

	void *buf = NULL;
	long len;
	if (fseek(f, 0, SEEK_END) != 0 || (len = ftell(f)) < 0 ||
	    fseek(f, 0, SEEK_SET) != 0)
		goto out;

	buf = malloc((size_t)len != 0 ? (size_t)len : 1);
	if (buf == NULL)
		goto out;

	if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
		free(buf);
		buf = NULL;
		goto out;
	}
	if (size != NULL)
		*size = (size_t)len;

out:
	fclose(f);
	return buf;                             /* caller owns + free()s the blob */
}

void disk_cache_put(void *cache, const unsigned char *key, const void *data,
                    size_t size, void *cache_item_metadata)
{
	(void)cache_item_metadata;
	struct disk_cache *c = cache;
	if (c == NULL)
		return;

	char path[V3D_CACHE_DIR_MAX + 80];
	v3d_cache_key_path(c, key, path, sizeof(path));

	/* Write to a unique temp name then rename() for an atomic publish. pid +
	 * a monotonic counter disambiguate concurrent writers (e.g. glamor-X). */
	static unsigned counter;              /* benign racy increment; only affects the temp name */
	char tmp[V3D_CACHE_DIR_MAX + 112];
	snprintf(tmp, sizeof(tmp), "%s.tmp.%d.%u", path, (int)getpid(), counter++);

	FILE *f = fopen(tmp, "wb");             /* Phoenix libc rejects "wt" — use "wb" */
	if (f == NULL)
		return;                             /* non-fatal: just don't cache */

	size_t wrote = fwrite(data, 1, size, f);
	if (fclose(f) != 0 || wrote != size) {
		remove(tmp);
		return;
	}
	if (rename(tmp, path) != 0)
		remove(tmp);
}

/* --- CL debug dumper (V3D_DEBUG=cl): disabled --- */
void *clif_dump_init(const void *devinfo, void *out, int pretty, int nocolor) { return NULL; }
void  clif_dump_destroy(void *clif) { }
void  clif_dump_add_bo(void *clif, const char *name, uint32_t offset, uint32_t size, void *vaddr) { }
int   clif_dump(void *clif) { return 0; }

/* --- build-id (Phoenix ELFs carry none): NULL. Under NDEBUG the asserts in
 * v3d_disk_cache_init() that check the note are compiled out, so returning NULL
 * is safe; the disk cache above keys purely off the shader BLAKE3, not build-id. --- */
const void *build_id_data(const void *note) { return NULL; }
void       *build_id_find_nhdr_for_addr(const void *addr) { return NULL; }

/* --- logging: route to nowhere (the driver only logs debug/perf) --- */
__attribute__((weak)) void  mesa_log(int level, const char *tag, const char *fmt, ...) { }
__attribute__((weak)) void  _mesa_log_multiline(int level, const char *tag, const char *lines) { }
__attribute__((weak)) void *_mesa_log_stream_create(int level, char *tag) { return NULL; }
__attribute__((weak)) void  mesa_log_stream_destroy(void *stream) { }
__attribute__((weak)) void  mesa_log_stream_printf(void *stream, const char *fmt, ...) { }

/* sha1 hex formatter: only used to format the (empty) build-id timestamp string
 * in v3d_disk_cache_init(); the disk cache does not consume its output. */
void _mesa_sha1_format(char *buf, const unsigned char *sha1) { if (buf) buf[0] = '\0'; }

/* --- memstream: not used by our disk cache; kept as link-safe no-ops --- */
int u_memstream_open(void *mem, char **bufp, size_t *sizep) { return 0; }
int u_memstream_close(void *mem) { return 0; }

/* --- driconf option lookup: always defaults --- */
void  driParseConfigFiles(void *opt, const void *info, int sc, const char *d,
                          const char *e, const char *a, const char *b, unsigned n, void *v) { }
unsigned char driCheckOption(const void *opt, const char *name, int type) { return 0; }
unsigned char driQueryOptionb(const void *opt, const char *name) { return 0; }
float driQueryOptionf(const void *opt, const char *name) { return 0.0f; }

/* --- NEON blake3: REMOVED. The previous do-nothing stub silently mis-hashed any
 * multi-chunk (>4 KiB) input — vkCreateShaderModule hashes the SPIR-V blob, so
 * large shaders (md5_vert 5228 B, screen_effects 8844 B) routed through it and
 * crashed/mis-hashed. The Phoenix aarch64 build now forces BLAKE3_USE_NEON=0
 * (external/mesa src/util/blake3/blake3_impl.h), so blake3_dispatch.c calls the
 * real portable blake3_hash_many_portable and never references this symbol. --- */

/* --- os abstraction: Phoenix isn't a Mesa-recognized OS, so os_misc.c/os_time.c
 * don't build. Provide minimal Phoenix-appropriate impls. --- */
#include <time.h>
int os_get_page_size(uint64_t *size) { if (size) *size = 4096; return 1; }
int os_get_total_physical_memory(uint64_t *size) { if (size) *size = 1ull << 30; return 1; }
int os_get_available_system_memory(uint64_t *size) { if (size) *size = 1ull << 30; return 1; }
const char *os_get_option(const char *name) { (void)name; return NULL; }
const char *os_get_option_cached(const char *name) { (void)name; return NULL; }
void os_log_message(const char *message) { (void)message; }

int64_t os_time_get_nano(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

int64_t os_time_get_absolute_timeout(uint64_t timeout)
{
	return os_time_get_nano() + (int64_t)timeout;
}

/* --- renderonly: KMS dumb-buffer import. We always pass ro=NULL, so these are
 * never reached; stub to satisfy the link. --- */
void *renderonly_create_gpu_import_for_resource(void *rsc, void *ro, void *out_handle)
{ return NULL; }
void renderonly_scanout_destroy(void *scanout, void *ro) { }

/* --- C11 threads: Mesa's threads_posix.c needs pthread_mutex_timedlock (absent on
 * Phoenix). The driver only uses call_once + mtx_init/lock/unlock/destroy, so back
 * those directly with pthread (mtx_t == pthread_mutex_t, once_flag == pthread_once_t
 * by Mesa's c11 typedefs, so the void* casts are size-correct). --- */
#include <pthread.h>
__attribute__((weak)) void call_once(void *flag, void (*func)(void)) { pthread_once((pthread_once_t *)flag, func); }
__attribute__((weak)) int  mtx_init(void *mtx, int type) { (void)type; return pthread_mutex_init((pthread_mutex_t *)mtx, NULL) ? 1 : 0; }
__attribute__((weak)) void mtx_destroy(void *mtx) { pthread_mutex_destroy((pthread_mutex_t *)mtx); }
__attribute__((weak)) int  mtx_lock(void *mtx) { return pthread_mutex_lock((pthread_mutex_t *)mtx) ? 1 : 0; }
__attribute__((weak)) int  mtx_unlock(void *mtx) { return pthread_mutex_unlock((pthread_mutex_t *)mtx) ? 1 : 0; }

/* --- locale-aware strtod/strtof (Mesa's _mesa_strtod uses newlocale): plain libc --- */
double _mesa_strtod(const char *s, char **end) { return strtod(s, end); }
float  _mesa_strtof(const char *s, char **end) { return strtof(s, end); }

/* v3d42_perfcounters_get/num are now provided by the REAL v3dx_counter.c@V3D_VERSION=42
 * (v3d_perfcntrs_init returns NULL with an empty set, failing v3d_screen_create). */

/* --- GNU qsort_r (Phoenix libc has only plain qsort). Simple insertion-sort
 * impl: correct, no global state, fine for Mesa's small sorts. --- */
void qsort_r(void *base, size_t nmemb, size_t size,
             int (*compar)(const void *, const void *, void *), void *arg)
{
	char *a = (char *)base;
	char tmp[256];
	if (size > sizeof(tmp))
		return;                 /* Mesa's qsort_r elements are tiny */
	for (size_t i = 1; i < nmemb; i++) {
		size_t j = i;
		while (j > 0 && compar(a + (j - 1) * size, a + j * size, arg) > 0) {
			__builtin_memcpy(tmp, a + (j - 1) * size, size);
			__builtin_memcpy(a + (j - 1) * size, a + j * size, size);
			__builtin_memcpy(a + j * size, tmp, size);
			j--;
		}
	}
}
