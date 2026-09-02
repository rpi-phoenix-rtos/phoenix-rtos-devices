/*
 * Copyright 2026 Phoenix Systems  %LICENSE%
 *
 * phoenix_mesa_compat.h — libc/libm gap shim for cross-compiling Mesa's v3d
 * driver subset to aarch64-phoenix (GLQuake Path C). Phoenix's libm lacks several
 * C99 math functions Mesa uses; provide self-contained inline implementations.
 * Force-include this (-include) ahead of Mesa sources. Grows as the cross-compile
 * gap-enumeration surfaces more missing symbols.
 */
#ifndef PHOENIX_MESA_COMPAT_H
#define PHOENIX_MESA_COMPAT_H
#include <stdint.h>

/* Phoenix has POSIX threads (pthread_mutex/cond/once/key); tell Mesa's
 * c11/threads.h to take the pthread path instead of #error-ing out. (Barriers
 * are the one gap — shimmed below.) */
#ifndef HAVE_PTHREAD
#define HAVE_PTHREAD 1
#endif

/* --- C99 math gaps (Phoenix math.h does not declare these) ---
 * C++ TUs (the GLSL compiler is C++) get these from <cmath>, and our `extern
 * double exp(double)` etc. conflict with libstdc++'s declarations — so the whole
 * math-gap block is C-only. */
#ifndef __cplusplus
static inline float  pmc_rintf(float f){ return (float)(f<0.0f?-(long long)(0.5f-f):(long long)(f+0.5f)); }
static inline float  rintf(float f){ return (float)(f<0.0f?-(long long)(0.5f-f):(long long)(f+0.5f)); }
static inline double rint(double d){ return (double)(d<0.0?-(long long)(0.5-d):(long long)(d+0.5)); }
static inline long   lrintf(float f){ return (long)(f<0.0f?-(long long)(0.5f-f):(long long)(f+0.5f)); }
static inline long   lrint(double d){ return (long)(d<0.0?-(long long)(0.5-d):(long long)(d+0.5)); }
static inline long long llroundf(float f){ return (long long)(f<0.0f?f-0.5f:f+0.5f); }
static inline long long llround(double d){ return (long long)(d<0.0?d-0.5:d+0.5); }
static inline long long llrint(double d){ return (long long)(d<0.0?-(long long)(0.5-d):(long long)(d+0.5)); }
static inline long      lround(double d){ return (long)(d<0.0?d-0.5:d+0.5); }
static inline float  roundf(float f){ return (float)(f<0.0f?(long long)(f-0.5f):(long long)(f+0.5f)); }
static inline double round(double d){ return (double)(d<0.0?(long long)(d-0.5):(long long)(d+0.5)); }
static inline float  fmaxf(float a,float b){ return a>b?a:b; }
static inline float  fminf(float a,float b){ return a<b?a:b; }
static inline double fmax(double a,double b){ return a>b?a:b; }
static inline double fmin(double a,double b){ return a<b?a:b; }
static inline float  truncf(float f){ return (float)(long long)f; }
/* float trig/transcendental + helpers Phoenix libm lacks — wrap its doubles. */
extern double sin(double), cos(double), tan(double), asin(double), acos(double);
extern double atan(double), atan2(double,double), pow(double,double), sqrt(double);
extern double floor(double), ceil(double), fabs(double), fmod(double,double), hypot(double,double);
static inline float sinf(float x){ return (float)sin((double)x); }
static inline float cosf(float x){ return (float)cos((double)x); }
static inline float tanf(float x){ return (float)tan((double)x); }
static inline float asinf(float x){ return (float)asin((double)x); }
static inline float acosf(float x){ return (float)acos((double)x); }
static inline float atanf(float x){ return (float)atan((double)x); }
static inline float atan2f(float y,float x){ return (float)atan2((double)y,(double)x); }
static inline float powf(float a,float b){ return (float)pow((double)a,(double)b); }
static inline float sqrtf(float x){ return (float)sqrt((double)x); }
static inline float floorf(float x){ return (float)floor((double)x); }
static inline float ceilf(float x){ return (float)ceil((double)x); }
static inline float fabsf(float x){ return x<0.0f?-x:x; }
static inline float fmodf(float a,float b){ return (float)fmod((double)a,(double)b); }
static inline float hypotf(float a,float b){ return (float)hypot((double)a,(double)b); }
static inline float  copysignf(float x,float y){ return (y<0.0f)?-( x<0.0f?-x:x ):( x<0.0f?-x:x ); }
static inline double copysign(double x,double y){ return (y<0.0)?-( x<0.0?-x:x ):( x<0.0?-x:x ); }
/* Phoenix libm has only the double exp/log/log2/pow; wrap the float variants. */
extern double exp(double); extern double log(double); extern double log2(double);
static inline float  expf(float x){ return (float)exp((double)x); }
static inline float  logf(float x){ return (float)log((double)x); }
static inline float  exp2f(float x){ return (float)exp((double)x*0.6931471805599453); }
static inline float  log2f(float x){ return (float)log2((double)x); }
#else /* __cplusplus: <cmath> provides exp/log/log2/round but Phoenix's lacks the
       * rint/lrint/fmax/fmin family — define just those (no extern conflicts). */
#include <cmath>
/* These fallbacks were needed when Phoenix's C++ <cmath>/math.h lacked the
 * rint/lrint/fmax/fmin/copysign/exp2f/log2f family. After the libmcs libm merge,
 * libphoenix's math.h (guard macro LIBMCS_MATH_H, pulled in by <cmath> above)
 * PROVIDES all of them as `extern`, so re-declaring them here `static inline`
 * is a hard C++ error ("'float rintf(float)' was declared 'extern' and later
 * 'static'") that drops the whole state_tracker/glsl .cpp sweep out of libGL.
 * Skip the fallbacks when libmcs is present; keep them for a pre-libmcs toolchain. */
#ifndef LIBMCS_MATH_H
static inline float  rintf(float f){ return (float)(f<0.0f?-(long long)(0.5f-f):(long long)(f+0.5f)); }
static inline double rint(double d){ return (double)(d<0.0?-(long long)(0.5-d):(long long)(d+0.5)); }
static inline long   lrintf(float f){ return (long)rintf(f); }
static inline long   lrint(double d){ return (long)rint(d); }
static inline float  fmaxf(float a,float b){ return a>b?a:b; }
static inline float  fminf(float a,float b){ return a<b?a:b; }
static inline double fmax(double a,double b){ return a>b?a:b; }
static inline double fmin(double a,double b){ return a<b?a:b; }
static inline float  copysignf(float x,float y){ return (y<0.0f)?-( x<0.0f?-x:x ):( x<0.0f?-x:x ); }
static inline double copysign(double x,double y){ return (y<0.0)?-( x<0.0?-x:x ):( x<0.0?-x:x ); }
/* exp2f/log2f: Phoenix C++ <cmath> declares expf/logf but NOT these two -> add only
 * these (adding expf/logf would conflict with the existing declarations). */
static inline float  exp2f(float x){ return (float)exp((double)x*0.6931471805599453); }
static inline float  log2f(float x){ return (float)log2((double)x); }
#endif /* !LIBMCS_MATH_H */
#endif /* !__cplusplus */


/* --- libc/POSIX gaps --- */
#include <stddef.h>
#include <string.h>
/* CPU affinity set — Phoenix lacks <sched.h> cpu_set_t; util/u_thread.c needs the
 * type + macros (the affinity calls sched_getcpu/pthread_setaffinity_np are stubbed
 * in gl_stubs.c). */
#ifndef CPU_SETSIZE
#define CPU_SETSIZE 128
typedef struct { unsigned long __bits[CPU_SETSIZE / (8 * sizeof(unsigned long))]; } cpu_set_t;
#define CPU_ZERO(s)     memset((s), 0, sizeof(*(s)))
#define CPU_SET(c, s)   ((s)->__bits[(c) / (8 * sizeof(unsigned long))] |= (1UL << ((c) % (8 * sizeof(unsigned long)))))
#define CPU_ISSET(c, s) ((((s)->__bits[(c) / (8 * sizeof(unsigned long))]) >> ((c) % (8 * sizeof(unsigned long)))) & 1UL)
#define CPU_COUNT(s)    (0)
#endif
/* sysconf() name Phoenix lacks: os_misc.c queries _SC_PHYS_PAGES for total RAM.
 * Define the name so it compiles; Phoenix sysconf returns -1 for unknown names and
 * os_get_total_physical_memory falls back gracefully. */
#ifndef _SC_PHYS_PAGES
#define _SC_PHYS_PAGES 85
#endif
/* inttypes.h pointer scanf-format macros missing on Phoenix */
#ifndef SCNxPTR
#define SCNxPTR "lx"
#endif
#ifndef SCNuPTR
#define SCNuPTR "lu"
#endif
#ifndef __cplusplus
/* C++ has static_assert as a keyword; defining it as a macro corrupts libstdc++. */
#ifndef static_assert
#define static_assert _Static_assert
#endif
#endif
#ifdef __cplusplus
extern "C" {
#endif
int posix_memalign(void **memptr, size_t alignment, size_t size);
/* GNU qsort_r (Phoenix has only plain qsort). Real impl is a port task. */
void qsort_r(void *base, size_t nmemb, size_t size,
             int (*compar)(const void *, const void *, void *), void *arg);
/* Phoenix pthread lacks barriers; provide types + decls (impls stubbed in gl_stubs.c). */
typedef struct { int __dummy; } pthread_barrier_t;
typedef struct { int __dummy; } pthread_barrierattr_t;
int pthread_barrier_init(pthread_barrier_t *, const pthread_barrierattr_t *, unsigned);
int pthread_barrier_wait(pthread_barrier_t *);
int pthread_barrier_destroy(pthread_barrier_t *);
#ifndef PTHREAD_BARRIER_SERIAL_THREAD
#define PTHREAD_BARRIER_SERIAL_THREAD (-1)
#endif
#ifdef __cplusplus
}
#endif

#endif /* PHOENIX_MESA_COMPAT_H */
