/*
 * rpi4-v3d-mesa.c — boot-launched harness that runs Mesa's ported v3d gallium
 * driver on the Pi4 (GLQuake Path C). First milestone: create a pipe_screen via
 * the real driver (v3d_screen_create) and report over UART — proving the entire
 * ported Mesa driver INITIALIZES on Phoenix HW (devinfo decode from our GET_PARAM,
 * caps, compiler/format-table setup), not just links.
 *
 * Launched at boot via user.plo.yaml (netboot variant) like rpi4-v3d-scout, so its
 * stdout reaches UART reliably and it runs BEFORE any NFS takeover — sidestepping
 * the intermittent NFS-as-root exec issues. screen_create is MMIO-free (GET_PARAM
 * only, no BO, no power-on), so this needs no GPU state and no filesystem.
 *
 * The driver talks to the "kernel" only through drmIoctl -> phoenix_v3d_ioctl (our
 * winsys, linked from libv3d-phoenix.a). Build links that lib; see the Makefile.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "pipe/p_screen.h"
#include "pipe/p_context.h"
#include "pipe/p_state.h"
#include "pipe/p_defines.h"
#include "pipe/p_shader_tokens.h"
#include "util/box.h"
#include "util/u_simple_shaders.h"
#include "util/u_draw.h"

/* Forward-declare rather than #include "v3d_screen.h" (drags c11/time.h timespec
 * clashes); the prototype is stable. */
struct pipe_screen_config;
struct renderonly;
struct pipe_screen *v3d_screen_create(int fd,
                                      const struct pipe_screen_config *config,
                                      struct renderonly *ro);

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("rpi4-v3d-mesa: entering v3d_screen_create\n");
	/* Pass a zeroed pipe_screen_config (NOT NULL): v3d_screen_create dereferences
	 * config->options_info for driParseConfigFiles; NULL config faults at far=0x8.
	 * Zeroed -> options/options_info NULL, which our driconf stubs accept. */
	struct pipe_screen_config cfg = { 0 };
	struct pipe_screen *pscreen = v3d_screen_create(0, &cfg, NULL);
	if (pscreen == NULL) {
		printf("rpi4-v3d-mesa: v3d_screen_create returned NULL\n");
		return 1;
	}
	const char *name = pscreen->get_name ? pscreen->get_name(pscreen) : "(no get_name)";
	const char *vendor = pscreen->get_vendor ? pscreen->get_vendor(pscreen) : "(no vendor)";
	printf("rpi4-v3d-mesa: pipe_screen OK name=%s vendor=%s\n", name, vendor);
	printf("rpi4-v3d-mesa: SCREEN-CREATE PASS (Mesa v3d driver initialized on HW)\n");

	/* Increment 2: create a pipe_context. Exercises v3d_context_create — the
	 * blitter (util_blitter_create), u_upload_mgr, CSO cache, fence init, and our
	 * mtx_/syncobj stubs. No GPU submit yet (that's the clear/draw increment). */
	printf("rpi4-v3d-mesa: entering context_create\n");
	struct pipe_context *pctx = pscreen->context_create(pscreen, NULL, 0);
	if (pctx == NULL) {
		printf("rpi4-v3d-mesa: context_create returned NULL\n");
		pscreen->destroy(pscreen);
		return 1;
	}
	printf("rpi4-v3d-mesa: CONTEXT-CREATE PASS (pipe_context up)\n");

	/* Increment 3: clear a small RT to green via the real Mesa driver. This is the
	 * FIRST GPU submit through the port — clear builds the TLB/RCL job, flush issues
	 * SUBMIT_CL -> the winsys CT0/CT1 path -> the V3D. Then read the RT back. Small
	 * RT (256x256) since the winsys MMU PT is one page (4 MiB GPU VA). */
	struct pipe_resource templ = { 0 };
	templ.target = PIPE_TEXTURE_2D;
	templ.format = PIPE_FORMAT_R8G8B8A8_UNORM;
	templ.width0 = 256; templ.height0 = 256; templ.depth0 = 1; templ.array_size = 1;
	templ.bind = PIPE_BIND_RENDER_TARGET | PIPE_BIND_SAMPLER_VIEW;
	struct pipe_resource *rt = pscreen->resource_create(pscreen, &templ);
	if (rt == NULL) {
		printf("rpi4-v3d-mesa: resource_create NULL\n");
		goto out;
	}
	printf("rpi4-v3d-mesa: RT 256x256 created\n");

	struct pipe_framebuffer_state fb = { 0 };
	fb.width = 256; fb.height = 256; fb.nr_cbufs = 1;
	fb.cbufs[0].texture = rt;
	fb.cbufs[0].format = templ.format;
	pctx->set_framebuffer_state(pctx, &fb);
	printf("rpi4-v3d-mesa: framebuffer set\n");

	union pipe_color_union color;
	color.f[0] = 0.0f; color.f[1] = 0.0f; color.f[2] = 0.0f; color.f[3] = 1.0f; /* black bg */
	pctx->clear(pctx, PIPE_CLEAR_COLOR0, 0, 0, NULL, &color, 0.0, 0);

	/* Increment 4: draw a colored triangle via the real Mesa driver. This runs
	 * v3d_compile (NIR->QPU) on the passthrough shaders at draw time + builds the
	 * geometry bin/render CLs -> the FIRST Mesa-generated SHADER + geometry render
	 * on the V3D. Markers bisect each step on HW. */
	enum tgsi_semantic vs_sem[2] = { TGSI_SEMANTIC_POSITION, TGSI_SEMANTIC_COLOR };
	unsigned vs_idx[2] = { 0, 0 };
	void *vs = util_make_vertex_passthrough_shader(pctx, 2, vs_sem, vs_idx, false);
	void *fs = util_make_fragment_passthrough_shader(pctx, TGSI_SEMANTIC_COLOR,
	                                                 TGSI_INTERPOLATE_PERSPECTIVE, true);
	if (!vs || !fs) { printf("rpi4-v3d-mesa: shader create NULL\n"); goto out; }
	pctx->bind_vs_state(pctx, vs);
	pctx->bind_fs_state(pctx, fs);
	printf("rpi4-v3d-mesa: shaders bound\n");

	struct pipe_blend_state blend = { 0 };
	blend.rt[0].colormask = PIPE_MASK_RGBA;
	void *cso_blend = pctx->create_blend_state(pctx, &blend);
	pctx->bind_blend_state(pctx, cso_blend);

	struct pipe_rasterizer_state rs = { 0 };
	rs.half_pixel_center = 1; rs.bottom_edge_rule = 1; rs.depth_clip_near = 1;
	rs.depth_clip_far = 1; rs.flatshade = 0;
	void *cso_rs = pctx->create_rasterizer_state(pctx, &rs);
	pctx->bind_rasterizer_state(pctx, cso_rs);

	/* depth-stencil-alpha: default (all disabled). v3d_emit_state derefs v3d->zsa
	 * (v3dx_emit.c:350) — leaving it unbound faults NULL+8 at draw. */
	struct pipe_depth_stencil_alpha_state dsa = { 0 };
	void *cso_dsa = pctx->create_depth_stencil_alpha_state(pctx, &dsa);
	pctx->bind_depth_stencil_alpha_state(pctx, cso_dsa);

	struct pipe_viewport_state vp = { 0 };
	vp.scale[0] = 128.0f; vp.scale[1] = 128.0f; vp.scale[2] = 0.5f;
	vp.translate[0] = 128.0f; vp.translate[1] = 128.0f; vp.translate[2] = 0.5f;
	pctx->set_viewport_states(pctx, 0, 1, &vp);
	printf("rpi4-v3d-mesa: state set\n");

	/* 3 verts: pos(xyzw) + color(rgba), interleaved (8 floats/vert, 32-byte stride) */
	static const float verts[3 * 8] = {
		-0.5f, -0.5f, 0.0f, 1.0f,   1.0f, 0.0f, 0.0f, 1.0f,  /* red   */
		 0.5f, -0.5f, 0.0f, 1.0f,   0.0f, 1.0f, 0.0f, 1.0f,  /* green */
		 0.0f,  0.5f, 0.0f, 1.0f,   0.0f, 0.0f, 1.0f, 1.0f,  /* blue  */
	};
	struct pipe_resource vbtempl = { 0 };
	vbtempl.target = PIPE_BUFFER; vbtempl.format = PIPE_FORMAT_R8_UNORM;
	vbtempl.width0 = sizeof(verts); vbtempl.height0 = 1; vbtempl.depth0 = 1;
	vbtempl.array_size = 1; vbtempl.bind = PIPE_BIND_VERTEX_BUFFER;
	struct pipe_resource *vbuf = pscreen->resource_create(pscreen, &vbtempl);
	if (!vbuf) { printf("rpi4-v3d-mesa: vbuf NULL\n"); goto out; }
	struct pipe_box vbox = { 0 };
	vbox.width = sizeof(verts); vbox.height = 1; vbox.depth = 1;
	struct pipe_transfer *vxfer = NULL;
	void *vmap = pctx->buffer_map(pctx, vbuf, 0, PIPE_MAP_WRITE, &vbox, &vxfer);
	if (!vmap) { printf("rpi4-v3d-mesa: vbuf map NULL\n"); goto out; }
	memcpy(vmap, verts, sizeof(verts));
	pctx->buffer_unmap(pctx, vxfer);

	struct pipe_vertex_buffer vb = { 0 };
	vb.buffer_offset = 0; vb.buffer.resource = vbuf;
	pctx->set_vertex_buffers(pctx, 1, &vb);

	struct pipe_vertex_element ve[2] = { 0 };
	ve[0].src_offset = 0;  ve[0].src_format = PIPE_FORMAT_R32G32B32A32_FLOAT; ve[0].src_stride = 32;
	ve[1].src_offset = 16; ve[1].src_format = PIPE_FORMAT_R32G32B32A32_FLOAT; ve[1].src_stride = 32;
	void *cso_ve = pctx->create_vertex_elements_state(pctx, 2, ve);
	pctx->bind_vertex_elements_state(pctx, cso_ve);
	printf("rpi4-v3d-mesa: vertex buffer + elements set; drawing\n");

	util_draw_arrays(pctx, MESA_PRIM_TRIANGLES, 0, 3);
	printf("rpi4-v3d-mesa: draw_vbo issued\n");
	pctx->flush(pctx, NULL, 0);
	printf("rpi4-v3d-mesa: flush done (triangle SUBMIT_CL)\n");

	struct pipe_box box = { 0 };
	box.width = 256; box.height = 256; box.depth = 1;
	struct pipe_transfer *xfer = NULL;
	void *map = pctx->texture_map(pctx, rt, 0, PIPE_MAP_READ, &box, &xfer);
	if (map != NULL) {
		uint32_t *px = map;
		printf("rpi4-v3d-mesa: TRI readback center=0x%08x topleft=0x%08x "
		       "midbottom=0x%08x (center should be the triangle, corner black)\n",
		       px[128 * 256 + 128], px[4 * 256 + 4], px[200 * 256 + 128]);

		/* Blit the 256x256 RT to /dev/fb0 (1024x768x32, pitch 4096) at top-left so
		 * the Mesa-rendered triangle is VISIBLE on HDMI. The pl011-tty console shares
		 * the surface and redraws over us, so loop ~25 s to land in an auto-snapshot. */
		int fbfd = open("/dev/fb0", O_WRONLY);
		if (fbfd >= 0) {
			for (int rep = 0; rep < 35; rep++) {
				for (int row = 0; row < 256; row++) {
					lseek(fbfd, (off_t)row * 4096, SEEK_SET);
					write(fbfd, (const uint8_t *)map + (size_t)row * 256 * 4, 256 * 4);
				}
				usleep(700000);
			}
			close(fbfd);
			printf("rpi4-v3d-mesa: blitted triangle to /dev/fb0 (HDMI top-left)\n");
		} else {
			printf("rpi4-v3d-mesa: open /dev/fb0 failed\n");
		}
		pctx->texture_unmap(pctx, xfer);
	} else {
		printf("rpi4-v3d-mesa: texture_map NULL\n");
	}
	printf("rpi4-v3d-mesa: TRIANGLE-DONE (Mesa-generated shaders+geometry on V3D)\n");

out:
	pctx->destroy(pctx);
	pscreen->destroy(pscreen);
	printf("rpi4-v3d-mesa: done\n");
	return 0;
}
