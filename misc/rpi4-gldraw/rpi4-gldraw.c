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

/* Focused GL conformance micro-tests (run on the already-current context + bound FBO).
 * Each draws a fullscreen quad into the FBO, reads back, and prints PASS/FAIL. They
 * isolate the features the Quake world rendering needs but gldraw's solid-red triangle
 * never exercised: VARYING-texcoord sampling and MULTITEXTURE combine. */
static void run_gltests(void)
{
	uint32_t *px = malloc((size_t)W * H * 4);
	/* full-FBO quad in clip space + texcoords 0..1 */
	static const float qv[8] = { -1.f,-1.f,  1.f,-1.f,  1.f,1.f,  -1.f,1.f };
	static const float qt[8] = {  0.f, 0.f,  1.f, 0.f,  1.f,1.f,   0.f,1.f };
	/* 2x2 texel grid, 4 DISTINCT colors. byte {R,G,B,A}; readback uint32 = A<<24|B<<16|G<<8|R.
	 * bottom row (t=0): red 0xff0000ff, green 0xff00ff00; top row (t=1): blue 0xffff0000, yellow 0xff00ffff */
	static const uint8_t t4[2*2*4] = {
		255,0,0,255,    0,255,0,255,
		0,0,255,255,    255,255,0,255,
	};
	uint32_t bl, br, tl, tr;

	printf("GLTEST: === begin ===\n");
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glViewport(0, 0, W, H);
	glMatrixMode(GL_PROJECTION); glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);  glLoadIdentity();
	glColor4f(1.f, 1.f, 1.f, 1.f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 0, qv);

	/* ---- T0: clear + readback baseline ---- */
	glClearColor(0.25f, 0.5f, 0.75f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT);
	glFinish();
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
	bl = px[(H/2)*W + W/2];
	printf("GLTEST %s T0-clear: got=0x%08x want=0xffbf7f40\n", (bl==0xffbf7f40)?"PASS":"FAIL", bl);

	/* ---- T1: single texture, 4 distinct texels, NEAREST, quad maps full 0..1 ---- */
	{
		GLuint tx = 0;
		glGenTextures(1, &tx);
		glBindTexture(GL_TEXTURE_2D, tx);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, t4);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glClientActiveTexture(GL_TEXTURE0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, qt);
		glClearColor(0.f, 0.f, 0.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
		chk("T1 draw");
		glFinish();
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
		bl = px[(H/4)*W + W/4];      br = px[(H/4)*W + 3*W/4];
		tl = px[(3*H/4)*W + W/4];    tr = px[(3*H/4)*W + 3*W/4];
		printf("GLTEST T1 quadrants: bl=0x%08x br=0x%08x tl=0x%08x tr=0x%08x (want red,green,blue,yellow)\n",
		       bl, br, tl, tr);
		int varying = !(bl == br && bl == tl && bl == tr);
		int exact = (bl == 0xff0000ff && br == 0xff00ff00 && tl == 0xffff0000 && tr == 0xff00ffff);
		printf("GLTEST %s T1-varying-texcoord-sampling (%s)\n",
		       varying ? "PASS" : "FAIL",
		       varying ? (exact ? "all 4 texels exact" : "quadrants differ but not exact") :
		                 "ALL QUADRANTS SAME = constant texcoord/sampling BUG");
		glDeleteTextures(1, &tx);
	}

	/* ---- T2: multitexture combine. TMU0 = 4-color, TMU1 = uniform 0.5, GL_MODULATE ---- */
	{
		static const uint8_t gray[2*2*4] = {
			128,128,128,255, 128,128,128,255, 128,128,128,255, 128,128,128,255,
		};
		GLuint tx0 = 0, tx1 = 0;
		glGenTextures(1, &tx0); glGenTextures(1, &tx1);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, tx0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, t4);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);  /* TMU0 = texel */
		glClientActiveTexture(GL_TEXTURE0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, qt);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, tx1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, gray);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);  /* TMU1: prev * 0.5 */
		glClientActiveTexture(GL_TEXTURE1);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, qt);
		glClearColor(0.f, 0.f, 0.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
		chk("T2 draw");
		glFinish();
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
		bl = px[(H/4)*W + W/4];   tr = px[(3*H/4)*W + 3*W/4];
		/* expect bl = red*0.5 ~ (128,0,0) = 0xff000080; tr = yellow*0.5 ~ (128,128,0)=0xff008080 */
		printf("GLTEST T2 multitex: bl=0x%08x tr=0x%08x (want ~0xff000080, ~0xff008080)\n", bl, tr);
		int rb = (bl & 0xff), rr = (tr & 0xff);
		int ok = (bl != tr) && (rb > 80 && rb < 176) && (rr > 80 && rr < 176);
		printf("GLTEST %s T2-multitexture-combine (%s)\n", ok ? "PASS" : "FAIL",
		       (bl == tr) ? "uniform = combiner not sampling TMU0 per-fragment" :
		       ok ? "modulated + varying" : "varying but unexpected magnitude");
		glActiveTexture(GL_TEXTURE1); glDisable(GL_TEXTURE_2D);
		glClientActiveTexture(GL_TEXTURE1); glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glActiveTexture(GL_TEXTURE0); glClientActiveTexture(GL_TEXTURE0);
		glDeleteTextures(1, &tx0); glDeleteTextures(1, &tx1);
	}

	/* ---- T3: IMMEDIATE-MODE multitexture — Quake's exact brush path (glBegin +
	 * glMultiTexCoord2f per vertex for TMU0 texture + TMU1 lightmap). T2 proved
	 * client-array multitexture works; if THIS is uniform/wrong, immediate-mode
	 * glMultiTexCoord2f is the broken path = the flat-gray world bug. ---- */
	{
		static const uint8_t gray[2*2*4] = {
			128,128,128,255, 128,128,128,255, 128,128,128,255, 128,128,128,255,
		};
		GLuint tx0 = 0, tx1 = 0;
		glGenTextures(1, &tx0); glGenTextures(1, &tx1);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, tx0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, t4);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, tx1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, gray);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glDisableClientState(GL_VERTEX_ARRAY);          /* immediate mode */
		glClientActiveTexture(GL_TEXTURE0); glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE1); glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glClearColor(0.f, 0.f, 0.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		glBegin(GL_TRIANGLE_FAN);
		for (int i = 0; i < 4; i++) {
			glMultiTexCoord2f(GL_TEXTURE0, qt[i*2], qt[i*2+1]);
			glMultiTexCoord2f(GL_TEXTURE1, qt[i*2], qt[i*2+1]);
			glVertex2f(qv[i*2], qv[i*2+1]);
		}
		glEnd();
		chk("T3 draw");
		glFinish();
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
		bl = px[(H/4)*W + W/4];   tr = px[(3*H/4)*W + 3*W/4];
		printf("GLTEST T3 immediate-multitex: bl=0x%08x tr=0x%08x (want ~0xff000080, ~0xff008080)\n", bl, tr);
		int rb = (bl & 0xff), rr = (tr & 0xff);
		int ok = (bl != tr) && (rb > 80 && rb < 176) && (rr > 80 && rr < 176);
		printf("GLTEST %s T3-immediate-mode-multitexture (%s)\n", ok ? "PASS" : "FAIL",
		       (bl == tr) ? "UNIFORM = immediate-mode glMultiTexCoord2f BROKEN (the brush-gray bug)" :
		       ok ? "modulated + varying" : "varying but unexpected magnitude");
		glActiveTexture(GL_TEXTURE1); glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glDeleteTextures(1, &tx0); glDeleteTextures(1, &tx1);
	}

	/* ---- T4: MIPMAPPED texture completeness + sampling (Quake's world/model textures are
	 * mipmapped with a LINEAR_MIPMAP_* min filter; my other tests used NEAREST). Upload a
	 * complete mip chain (2x2 4-color base + 1x1 level1) and draw magnified (-> level 0).
	 * If the result is the 4 colors -> mipmapped textures are complete + sample. If BLACK ->
	 * the texture is mipmap-INCOMPLETE (mip upload broken) -> GL samples black = brush bug. ---- */
	{
		static const uint8_t lvl1[1*1*4] = { 255, 255, 255, 255 };   /* 1x1 white */
		GLuint tx = 0;
		glGenTextures(1, &tx);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, tx);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, t4);
		glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, lvl1);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(2, GL_FLOAT, 0, qv);
		glClientActiveTexture(GL_TEXTURE0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY); glTexCoordPointer(2, GL_FLOAT, 0, qt);
		glClearColor(0.f, 0.f, 0.f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
		chk("T4 draw");
		glFinish();
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
		bl = px[(H/4)*W + W/4];   tr = px[(3*H/4)*W + 3*W/4];
		printf("GLTEST T4 mipmap: bl=0x%08x tr=0x%08x (LINEAR_MIPMAP filter; magnified -> level0 4-color)\n", bl, tr);
		int black = (bl == 0xff000000 && tr == 0xff000000);
		int varying = (bl != tr);
		printf("GLTEST %s T4-mipmapped-texture (%s)\n",
		       (varying && !black) ? "PASS" : "FAIL",
		       black ? "BLACK = mipmap-incomplete texture samples black (the brush bug!)" :
		       varying ? "samples level0 (mips complete)" : "uniform (unexpected)");
		glDeleteTextures(1, &tx);
	}

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);
	printf("GLTEST: === end ===\n");
	free(px);
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

	/* Focused GL micro-tests (varying-texcoord sampling + multitexture) before the demo
	 * triangle — these pinpoint the world-render bug. Results print as GLTEST PASS/FAIL. */
	run_gltests();

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
