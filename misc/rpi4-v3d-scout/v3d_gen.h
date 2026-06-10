/*
 * Minimal "gen" shim for Mesa's generated V3D control-list packers.
 *
 * Mesa's gen_pack_header.py emits packet structs + `V3D<ver>_<PACKET>_pack()`
 * functions that reference a small set of driver-provided helpers (normally from
 * cle/v3d_packet_helpers.h + util/bitpack_helpers.h + a driver's __gen_* macros).
 * Porting those headers wholesale pulls in the Mesa build tree; instead this shim
 * provides exactly the symbols the generated header uses, so we can emit CLE
 * packets through Mesa's authoritative packers (no hand-encoded bytes).
 *
 * Addresses: this scout uses absolute GPU virtual addresses (the V3D MMU maps
 * them), so an "address" is just a u32 and relocation is a no-op.
 *
 * Copyright 2026 Phoenix Systems
 * %LICENSE%
 */

#ifndef V3D_GEN_SHIM_H
#define V3D_GEN_SHIM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* gen_pack_header.py guards asserts behind NDEBUG; define it so the generated
 * _pack functions don't pull in <assert.h> range checks. */
#ifndef NDEBUG
#define NDEBUG 1
#endif
#include <assert.h>

/* float<->u32 bit-casts (mesa util/u_math.h). */
static inline uint32_t fui(float f)
{
	union { float f; uint32_t ui; } u;
	u.f = f;
	return u.ui;
}

static inline float uif(uint32_t ui)
{
	union { float f; uint32_t ui; } u;
	u.ui = ui;
	return u.f;
}

/* --- util_bitpack_* (from mesa util/bitpack_helpers.h; NDEBUG-trimmed) --- */
static inline uint64_t util_bitpack_uint(uint64_t v, uint32_t start, uint32_t end)
{
	(void)end;
	return v << start;
}

static inline uint64_t util_bitpack_uint_nonzero(uint64_t v, uint32_t start, uint32_t end)
{
	return util_bitpack_uint(v, start, end);
}

static inline uint64_t util_bitpack_sint(int64_t v, uint32_t start, uint32_t end)
{
	const int bits = (int)(end - start + 1u);
	const uint64_t mask = (bits >= 64) ? ~0ull : ((1ull << bits) - 1ull);
	return ((uint64_t)v & mask) << start;
}

/* Manual round-to-nearest avoids a libm (llroundf) dependency. These are only
 * reached for packets with fixed-point fields, which this scout does not emit. */
static inline uint64_t util_bitpack_ufixed(float v, uint32_t start, uint32_t end, uint32_t fract_bits)
{
	const float factor = (float)(1u << fract_bits);
	return util_bitpack_uint((uint64_t)(v * factor + 0.5f), start, end);
}

static inline uint64_t util_bitpack_sfixed(float v, uint32_t start, uint32_t end, uint32_t fract_bits)
{
	const float factor = (float)(1u << fract_bits);
	const float r = v * factor + (v >= 0.0f ? 0.5f : -0.5f);
	return util_bitpack_sint((int64_t)r, start, end);
}

/* --- driver __gen_* layer: absolute-VA addresses, no relocations --- */
struct v3d_gen_address {
	uint32_t offset; /* absolute GPU virtual address */
};

#define __gen_address_type struct v3d_gen_address
#define __gen_user_data void

static inline uint64_t __gen_address_offset(const struct v3d_gen_address *addr)
{
	return addr->offset;
}

static inline void __gen_emit_reloc(void *data, const struct v3d_gen_address *addr)
{
	(void)data;
	(void)addr;
}

/* --- unpack stubs: only present so the generated _unpack functions compile;
 * this scout never calls them (it only packs). --- */
static inline uint64_t __gen_unpack_uint(const uint8_t *cl, uint32_t start, uint32_t end)
{
	(void)cl; (void)start; (void)end; return 0;
}
static inline int64_t __gen_unpack_sint(const uint8_t *cl, uint32_t start, uint32_t end)
{
	(void)cl; (void)start; (void)end; return 0;
}
static inline float __gen_unpack_ufixed(const uint8_t *cl, uint32_t start, uint32_t end, uint32_t fb)
{
	(void)cl; (void)start; (void)end; (void)fb; return 0.0f;
}
static inline float __gen_unpack_sfixed(const uint8_t *cl, uint32_t start, uint32_t end, uint32_t fb)
{
	(void)cl; (void)start; (void)end; (void)fb; return 0.0f;
}
static inline float __gen_unpack_float(const uint8_t *cl, uint32_t start, uint32_t end)
{
	(void)cl; (void)start; (void)end; return 0.0f;
}
static inline float __gen_unpack_f(const uint8_t *cl, uint32_t start, uint32_t end)
{
	(void)cl; (void)start; (void)end; return 0.0f;
}
static inline uint64_t __gen_unpack_address(const uint8_t *cl, uint32_t start, uint32_t end)
{
	(void)cl; (void)start; (void)end; return 0;
}

#endif /* V3D_GEN_SHIM_H */
