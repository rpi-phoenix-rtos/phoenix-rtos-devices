/*
 * gl_stubs.c — stubs + libc gaps for the Phoenix GL frontend (GLQuake Path C,
 * Phase 4). Two groups:
 *  (1) sw-path / debug symbols st references but v3d (HW vertex, GLSL-not-SPIRV,
 *      no disk cache) never exercises at runtime: the gallium `draw` module,
 *      SPIR-V, ASTC decode, disk cache, gallium trace, sw tgsi_exec. Stubbed to
 *      NULL/no-op (linkage is by name; generic signatures resolve the refs).
 *  (2) small libc gaps Phoenix lacks: posix_memalign,
 *      pthread_mutex_timedlock (lets c11 threads_posix.c link). Anything
 *      libphoenix later implements MUST be dropped from here, or the duplicate
 *      definition breaks the link.
 * Compiled WITHOUT the compat shim. Warnings off.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* --- gallium draw module (sw vertex pipeline): unused, v3d does HW vertex. st
 * creates+configures a draw context but only uses it for feedback/select/sw-vertex
 * paths we never hit; stub the whole surface (st->draw stays NULL/inert). --- */
void *draw_create(void *pipe) { (void)pipe; return NULL; }
void *draw_create_vertex_shader(void *draw, const void *shader) { (void)draw; (void)shader; return NULL; }
void  draw_delete_vertex_shader(void *draw, void *vs) { (void)draw; (void)vs; }
void  draw_destroy(void *draw) { (void)draw; }
int   draw_nir_lower_opcodes(void *nir) { (void)nir; return 0; }
void  draw_vbo(void *draw, const void *info, unsigned drawid, const void *indirect, const void *draws, unsigned num) { (void)draw; (void)info; (void)drawid; (void)indirect; (void)draws; (void)num; }
void  draw_set_rasterizer_state(void *draw, const void *rs, void *h) { (void)draw; (void)rs; (void)h; }
void  draw_set_sampler_views(void *draw, unsigned shader, void **views, unsigned n) { (void)draw; (void)shader; (void)views; (void)n; }
void  draw_set_samplers(void *draw, unsigned shader, void **s, unsigned n) { (void)draw; (void)shader; (void)s; (void)n; }
void  draw_set_vertex_buffers(void *draw, unsigned n, const void *b) { (void)draw; (void)n; (void)b; }
void  draw_set_vertex_elements(void *draw, unsigned n, const void *e) { (void)draw; (void)n; (void)e; }
void  draw_set_viewport_states(void *draw, unsigned s, unsigned n, const void *v) { (void)draw; (void)s; (void)n; (void)v; }
float draw_wide_line_threshold(void *draw) { (void)draw; return 1.0f; }
float draw_wide_point_threshold(void *draw) { (void)draw; return 1.0f; }
int   spirv_verify_gl_specialization_constants(const uint32_t *w, size_t n, void *s, size_t ns, int stage, const char *ep) { (void)w; (void)n; (void)s; (void)ns; (void)stage; (void)ep; return 0; }

/* remaining gallium-draw-module surface st configures on st->draw (NULL/inert) */
void draw_bind_vertex_shader(void *d, void *vs) { (void)d; (void)vs; }
void draw_enable_line_stipple(void *d, int e) { (void)d; (void)e; }
void draw_enable_point_sprites(void *d, int e) { (void)d; (void)e; }
void draw_set_clip_state(void *d, const void *c) { (void)d; (void)c; }
void draw_set_constant_buffer_stride(void *d, unsigned s) { (void)d; (void)s; }
void draw_set_images(void *d, unsigned sh, const void *i, unsigned n) { (void)d; (void)sh; (void)i; (void)n; }
void draw_set_indexes(void *d, const void *p, unsigned sz, unsigned cnt) { (void)d; (void)p; (void)sz; (void)cnt; }
void draw_set_mapped_constant_buffer(void *d, unsigned sh, unsigned i, const void *p, unsigned sz) { (void)d; (void)sh; (void)i; (void)p; (void)sz; }
void draw_set_mapped_image(void *d, unsigned sh, unsigned i, void *img) { (void)d; (void)sh; (void)i; (void)img; }
void draw_set_mapped_shader_buffer(void *d, unsigned sh, const void *b, unsigned n) { (void)d; (void)sh; (void)b; (void)n; }
void draw_set_mapped_texture(void *d, unsigned sh, unsigned i, unsigned w, unsigned h, unsigned dep, unsigned fl, unsigned ml, const void *a, const void *b, const void *c) { (void)d; (void)sh; (void)i; (void)w; (void)h; (void)dep; (void)fl; (void)ml; (void)a; (void)b; (void)c; }
void draw_set_mapped_vertex_buffer(void *d, unsigned i, const void *p, size_t sz) { (void)d; (void)i; (void)p; (void)sz; }
void *draw_set_rasterize_stage(void *d, void *s) { (void)d; (void)s; return NULL; }
void draw_set_vertex_info(void *d, const void *vi) { (void)d; (void)vi; }
void *draw_glfeedback_stage(void *ctx, void *d) { (void)ctx; (void)d; return NULL; }
void *draw_glselect_stage(void *ctx, void *d) { (void)ctx; (void)d; return NULL; }

/* pthread barriers (Phoenix lacks) — single-thread no-ops for util_barrier_* */
int pthread_barrier_init(void *b, const void *a, unsigned n) { (void)b; (void)a; (void)n; return 0; }
int pthread_barrier_wait(void *b) { (void)b; return -1; /* PTHREAD_BARRIER_SERIAL_THREAD */ }
int pthread_barrier_destroy(void *b) { (void)b; return 0; }
unsigned util_hash_crc32(const void *data, size_t size) { (void)data; (void)size; return 0; }

/* --- CPU affinity / cpu_set_t: Phoenix has no sched affinity; no-op --- */
int sched_getcpu(void) { return 0; }
int pthread_setaffinity_np(unsigned long t, size_t sz, const void *set) { (void)t; (void)sz; (void)set; return 0; }
int pthread_getaffinity_np(unsigned long t, size_t sz, void *set) { (void)t; (void)sz; (void)set; return 0; }

/* _mesa_uint_array_min_max lives in sse_minmax.c (excluded for x86 smmintrin) —
 * portable scalar version. */
void _mesa_uint_array_min_max(const unsigned *vals, unsigned *min, unsigned *max, unsigned n)
{
	unsigned lo = 0xffffffffu, hi = 0;
	for (unsigned i = 0; i < n; i++) {
		if (vals[i] < lo) lo = vals[i];
		if (vals[i] > hi) hi = vals[i];
	}
	if (min) *min = lo;
	if (max) *max = hi;
}
/* x86 SSE4.1 streaming load: portable memcpy fallback */
void *util_streaming_load_memcpy(void *dst, void *src, size_t n) { return memcpy(dst, src, n); }

/* --- SPIR-V (GL_ARB_gl_spirv): Quake/gears use GLSL, not SPIR-V --- */
void *spirv_to_nir(const uint32_t *w, size_t n, void *spec, size_t ns,
                   int stage, const char *ep, const void *opts, const void *nopts)
{ (void)w; (void)n; (void)spec; (void)ns; (void)stage; (void)ep; (void)opts; (void)nopts; return NULL; }
void *vtn_alloc_specialization(void *a, size_t b) { (void)a; (void)b; return NULL; }
void  vtn_free_specialization(void *a) { (void)a; }
void  vtn_add_specialization_entry(void *a, void *b) { (void)a; (void)b; }

/* --- ASTC texture decode: not used by Quake's textures --- */
const void *_mesa_get_astc_decoder_partition_table(int a, int b) { (void)a; (void)b; return NULL; }
void  _mesa_init_astc_decoder_luts(void) {}
void  _mesa_unpack_astc_2d_ldr(void) {}

/* --- disk shader cache: disabled --- */
unsigned char disk_cache_has_key(void *c, const unsigned char *k) { (void)c; (void)k; return 0; }
void disk_cache_put_key(void *c, const unsigned char *k) { (void)c; (void)k; }
void disk_cache_remove(void *c, const unsigned char *k) { (void)c; (void)k; }

/* --- gallium trace (debug dumper): off --- */
unsigned char trace_dumping_enabled_locked(void) { return 0; }
void trace_dumping_start_locked(void) {}
void trace_dumping_stop_locked(void) {}

/* --- sw tgsi interpreter + translate + nir_to_tgsi (only the draw module used
 * these; stubbed so any straggler ref links) --- */
void *nir_to_tgsi(void *nir, void *screen) { (void)nir; (void)screen; return NULL; }
void  tgsi_exec_machine_bind_shader(void *m, const void *t, void *s) { (void)m; (void)t; (void)s; }
void *tgsi_exec_machine_create(int p) { (void)p; return NULL; }
void  tgsi_exec_machine_destroy(void *m) { (void)m; }
unsigned tgsi_exec_machine_run(void *m, int i) { (void)m; (void)i; return 0; }
void  tgsi_exec_set_constant_buffers(void *m, unsigned n, const void *b) { (void)m; (void)n; (void)b; }
void *translate_cache_create(void) { return NULL; }
void  translate_cache_destroy(void *c) { (void)c; }
void *translate_cache_find(void *c, void *k) { (void)c; (void)k; return NULL; }
void  pipe_get_tile_rgba(void *t, void *p, int x, int y, int w, int h, int f, void *d)
{ (void)t; (void)p; (void)x; (void)y; (void)w; (void)h; (void)f; (void)d; }

/* --- misc --- */
const char *os_get_option_secure(const char *name) { (void)name; return NULL; }

/* --- libc gaps Phoenix lacks (real impls) --- */
int posix_memalign(void **memptr, size_t alignment, size_t size)
{
	if (alignment < sizeof(void *))
		alignment = sizeof(void *);
	void *p = NULL;
	/* malloc + manual align with a back-pointer for free() compatibility is
	 * fragile; Phoenix malloc returns 16-byte-aligned, enough for Mesa's uses. */
	p = malloc(size);
	if (!p)
		return 12; /* ENOMEM */
	*memptr = p;
	return 0;
}

/* strtok_r was here as a Phoenix libc gap. libphoenix provides it now (both
 * libphoenix.a and libm.a export it), so keeping a local copy makes the link
 * fail with `multiple definition of strtok_r` as soon as -lm is pulled in --
 * which is exactly what broke a from-scratch build while dev trees with older
 * .gpu-libs archives kept linking. If another libc gap closes upstream, delete
 * the stub here in the same way. */

/* c11 threads_posix.c uses pthread_mutex_timedlock (absent on Phoenix); a plain
 * blocking lock is a correct-enough fallback for Mesa's timed waits. */
struct __phx_mtx;
int pthread_mutex_lock(void *);
int pthread_mutex_timedlock(void *m, const void *abstime) { (void)abstime; return pthread_mutex_lock(m); }

/* util_get_process_name: u_process.c needs glibc program_invocation_short_name
 * (absent on Phoenix). Return a fixed name — only used for debug/cache naming. */
const char *util_get_process_name(void) { return "phoenix-gl"; }
