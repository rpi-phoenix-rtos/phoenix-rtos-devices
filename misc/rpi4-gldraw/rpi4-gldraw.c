/*
 * rpi4-gldraw.c — GLQuake Path-C Phase-4: a TEXTURED, DEPTH-TESTED triangle drawn
 * through the Mesa OpenGL frontend on the real V3D (boot-launched). This is the
 * rung between glClear and glgears: it exercises the GL frontend's vertex-submission
 * + fixed-function shader generation + v3d_compile-at-draw path (the earlier triangle
 * was hand-built control lists straight through the driver, NOT through the GL API).
 *
 *   st_create_context -> _mesa_make_current(NULL,NULL) (surfaceless)
 *   -> FBO with a color renderbuffer (RGBA8) + a depth renderbuffer (DEPTH24)
 *   -> a 2x2 RGBA texture, GL_TEXTURE_2D enabled (fixed-function texturing)
 *   -> client vertex+texcoord arrays -> glDrawArrays(GL_TRIANGLES,0,3) with depth test
 *   -> glReadPixels: center == a texture color (triangle drew), a corner == clear color.
 *
 * Textured + depth-tested is deliberately Quake-relevant (GLQuake is fixed-function,
 * textured, depth-tested; lighting is orthogonal). Links libGL-phoenix.a + libv3d-phoenix.a.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "pipe/p_screen.h"
#include "pipe/p_context.h"
#include "pipe/p_state.h"
#include "main/menums.h"
#include "frontend/api.h"
#include "main/mtypes.h"
#include "state_tracker/st_context.h"
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include "GL/gl.h"
#include "GL/glext.h"

struct pipe_screen_config;
struct renderonly;
struct pipe_screen *v3d_screen_create(int fd, const struct pipe_screen_config *config, struct renderonly *ro);
extern unsigned char _mesa_make_current(struct gl_context *ctx,
                                        struct gl_framebuffer *drawFb,
                                        struct gl_framebuffer *readFb);

/* Render at the proven 256x256 target (a 1024x768 render target currently comes back
 * all-zero -- a V3D tile-state/overflow sizing limit in the winsys, TODO for fullscreen
 * Quake). For HDMI we upscale the 256x256 result 3x to a centered 768x768 region on a
 * grey 1024x768 framebuffer (pitch 4096 == FB_W*4, contiguous). */
#define W 256
#define H 256
#define FB_W 1024
#define FB_H 768
#define SCALE 3
#define OFF_X ((FB_W - W * SCALE) / 2)   /* (1024-768)/2 = 128 */
#define OFF_Y ((FB_H - H * SCALE) / 2)   /* (768-768)/2 = 0   */

static void chk(const char *where)
{
	GLenum e = glGetError();
	if (e != GL_NO_ERROR)
		printf("gldraw: GL error 0x%x after %s\n", e, where);
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("gldraw: START (textured depth-tested triangle via the GL frontend)\n");

	struct pipe_screen_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	struct pipe_screen *pscreen = v3d_screen_create(0, &cfg, NULL);
	if (!pscreen) { printf("gldraw: pipe_screen NULL\n"); return 1; }
	struct pipe_context *pipe = pscreen->context_create(pscreen, NULL, 0);
	if (!pipe) { printf("gldraw: pipe_context NULL\n"); return 1; }

	struct gl_config visual;
	struct st_config_options opts;
	memset(&visual, 0, sizeof(visual));
	memset(&opts, 0, sizeof(opts));
	struct st_context *st = st_create_context(API_OPENGL_COMPAT, pipe, &visual,
	                                          NULL, &opts, 0, 0);
	if (!st) { printf("gldraw: st_create_context NULL\n"); return 1; }
	_mesa_make_current(st->ctx, NULL, NULL);
	printf("gldraw: GL up; %s / %s\n",
	       (const char *)glGetString(GL_VERSION), (const char *)glGetString(GL_RENDERER));

	/* FBO: color + depth renderbuffers. */
	GLuint fbo = 0, rbColor = 0, rbDepth = 0;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glGenRenderbuffers(1, &rbColor);
	glBindRenderbuffer(GL_RENDERBUFFER, rbColor);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, W, H);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbColor);
	glGenRenderbuffers(1, &rbDepth);
	glBindRenderbuffer(GL_RENDERBUFFER, rbDepth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, W, H);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbDepth);
	chk("FBO setup");
	GLenum fbs = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	printf("gldraw: FBO status=0x%x (complete=0x%x)\n", fbs, GL_FRAMEBUFFER_COMPLETE);

	/* 2x2 RGBA texture, solid RED. Deliberately non-white + non-clear so the readback
	 * is unambiguous: a textured fragment reads red (0xff0000ff in RGBA byte order);
	 * the FF default vertex color (white) would read 0xffffffff -> exposes "texturing
	 * silently not applied"; the clear color is 0xff1a1a1a. (FF default TEXENV is
	 * GL_MODULATE; primary color defaults to white, identity for the modulate.) */
	static const uint8_t texels[2 * 2 * 4] = {
		255, 0, 0, 255,   255, 0, 0, 255,
		255, 0, 0, 255,   255, 0, 0, 255,
	};
	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, texels);
	glEnable(GL_TEXTURE_2D);
	chk("texture");

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glViewport(0, 0, W, H);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  /* dark grey */
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* fixed-function client arrays: a triangle in clip space (identity matrices). */
	static const float verts[3 * 2] = {
		-0.7f, -0.7f,   0.7f, -0.7f,   0.0f, 0.7f,
	};
	static const float texc[3 * 2] = {
		0.0f, 0.0f,   1.0f, 0.0f,   0.5f, 1.0f,
	};
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texc);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	chk("DrawArrays");
	glFinish();
	printf("gldraw: glDrawArrays+glFinish done (FF vertex+texture shader gen at draw)\n");

	uint32_t *px = malloc((size_t)W * H * 4);
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
	chk("ReadPixels");
	uint32_t center = px[(H / 2) * W + (W / 2)];
	uint32_t corner = px[2 * W + 2];   /* top-left-ish: outside the triangle */
	printf("gldraw: readback center=0x%08x corner=0x%08x (textured-red=0xff0000ff, white=untextured, clear=0xff1a1a1a)\n",
	       center, corner);
	const char *verdict =
	    (center == 0xff0000ff) ? "TEXTURED (center=red texel -> FF texture sampling works)" :
	    (center == 0xffffffff) ? "DREW-BUT-UNTEXTURED (center=white -> default vertex color, texturing NOT applied)" :
	    (center != corner)     ? "DREW (center differs from clear, but unexpected color)" :
	                             "NO-DRAW (center==clear)";
	printf("gldraw: %s\n", verdict);
	printf("gldraw: GLDRAW-DONE\n");

	/* Build the fullscreen framebuffer image: grey background + the 256x256 render
	 * upscaled SCALE x (nearest) into a centered region. */
	uint32_t *fbimg = malloc((size_t)FB_W * FB_H * 4);
	for (size_t i = 0; i < (size_t)FB_W * FB_H; i++)
		fbimg[i] = 0xff1a1a1a;                 /* grey */
	for (int sy = 0; sy < H; sy++) {
		for (int sx = 0; sx < W; sx++) {
			uint32_t c = px[sy * W + sx];
			for (int dy = 0; dy < SCALE; dy++) {
				int fy = OFF_Y + sy * SCALE + dy;
				uint32_t *row = fbimg + (size_t)fy * FB_W + OFF_X + sx * SCALE;
				for (int dx = 0; dx < SCALE; dx++)
					row[dx] = c;
			}
		}
	}

	/* Hold it on HDMI: fbcon mirrors klog to the same /dev/fb0, so a one-shot blit
	 * gets overwritten by console text (our own prints + other daemons' logs). Re-blit
	 * continuously so the image wins and stays visible. pitch == FB_W*4 (contiguous). */
	int fb = open("/dev/fb0", O_WRONLY);
	if (fb < 0) { printf("gldraw: /dev/fb0 open failed\n"); for (;;) sleep(5); }
	printf("gldraw: holding %dx%d (256x256 render upscaled %dx, centered) on /dev/fb0\n", FB_W, FB_H, SCALE);
	for (;;) {
		lseek(fb, 0, SEEK_SET);
		(void)write(fb, fbimg, (size_t)FB_W * FB_H * 4);
		usleep(300000);  /* ~3 Hz refresh: beats sporadic console writes */
	}
	close(fb);
	free(fbimg);
	free(px);
	return 0;
}
