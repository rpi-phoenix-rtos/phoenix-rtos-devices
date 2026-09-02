/*
 * v3d_phoenix_mathshim.c — REAL (out-of-line) definitions of the C99 float-math
 * functions Phoenix libm lacks, for the Mesa GL frontend (GLQuake Path C, Phase 4).
 *
 * phoenix_mesa_compat.h provides these as `static inline` for in-TU use, but
 * cross-TU external references survive: objects in the prebuilt libv3d-phoenix.a,
 * and C++ TUs whose <cmath> DECLARES expf/logf (so the inline is suppressed) yet
 * Phoenix libm has no symbol. This file provides the global symbols so any such
 * reference links. Compiled WITHOUT the compat shim (so these are definitions, not
 * conflicting inlines). Wraps Phoenix libm's double routines.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
extern double sin(double), cos(double), tan(double), asin(double), acos(double);
extern double atan(double), atan2(double, double), exp(double), log(double), log2(double);
extern double pow(double, double), sqrt(double), floor(double), ceil(double);
extern double fmod(double, double), ldexp(double, int);
extern double rint(double);
/* NOTE: hypot (double) and hypotf (float) are NOT defined here — libphoenix's libm now
 * provides them. Defining them here too produced a duplicate-symbol link clash (previously
 * masked by -Wl,--allow-multiple-definition in rpi4-quake; that workaround is now removed).
 * If a future libphoenix drops them, restore the wrappers here. */

/* All definitions are WEAK: these are FALLBACKS for a libphoenix libm that lacks the C99
 * float-math functions. Since the RTOS-1132 libm overhaul (libmcs) upstream libphoenix now
 * PROVIDES most of them (tanf/acosf/asinf/atanf/atan2f/powf/logf/expf/fmodf/ldexpf, ...); a
 * strong libphoenix symbol overrides the weak shim (no duplicate-definition link clash), and
 * the weak shim still resolves the reference on an older libphoenix that lacks it. This
 * replaces the old manual "remove each fn as libphoenix adds it" tracking (see hypot note). */
#define WK __attribute__((weak))
WK float sinf(float x) { return (float)sin((double)x); }
WK float cosf(float x) { return (float)cos((double)x); }
WK float tanf(float x) { return (float)tan((double)x); }
WK float asinf(float x) { return (float)asin((double)x); }
WK float acosf(float x) { return (float)acos((double)x); }
WK float atanf(float x) { return (float)atan((double)x); }
WK float atan2f(float y, float x) { return (float)atan2((double)y, (double)x); }
WK float expf(float x) { return (float)exp((double)x); }
WK float logf(float x) { return (float)log((double)x); }
WK float exp2f(float x) { return (float)exp((double)x * 0.6931471805599453); }
WK float log2f(float x) { return (float)log2((double)x); }
WK float powf(float a, float b) { return (float)pow((double)a, (double)b); }
WK float fmodf(float a, float b) { return (float)fmod((double)a, (double)b); }
WK float ldexpf(float x, int e) { return (float)ldexp((double)x, e); }
WK float rintf(float x) { return (float)rint((double)x); }
WK long  lrintf(float x) { return (long)rintf(x); }
WK float fmaxf(float a, float b) { return a > b ? a : b; }
WK float fminf(float a, float b) { return a < b ? a : b; }
WK float copysignf(float x, float y) { return (y < 0.0f) ? -(x < 0.0f ? -x : x) : (x < 0.0f ? -x : x); }
/* NOTE: sqrtf/floorf/ceilf/fabsf/truncf/roundf/lroundf are present in Phoenix
 * libm — do NOT redefine them here (multiple-definition at link). */
