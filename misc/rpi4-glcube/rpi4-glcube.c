/*
 * rpi4-glcube.c — GLQuake Path-C Phase-4: an ANIMATED spinning, depth-tested,
 * perspective-projected colored cube drawn through the Mesa OpenGL frontend on the
 * real V3D (boot-launched), held + animated on the HDMI screen. This is the
 * "glgears-or-similar" rung: it demonstrates basic OpenGL animation working,
 * GPU-accelerated, on hardware.
 *
 *   st_create_context -> _mesa_make_current(NULL,NULL) (surfaceless)
 *   -> FBO (RGBA8 color + DEPTH24) -> glFrustum perspective + GL_DEPTH_TEST
 *   -> per frame: clear, glRotatef(angle++), glBegin(GL_QUADS) 6 colored faces glEnd,
 *      glReadPixels -> upscale 3x (y-flipped to screen orientation) -> blit /dev/fb0.
 *
 * Renders at 256x256 (a 1024x768 RT currently comes back all-zero -- a V3D tile-state
 * sizing limit in the winsys, TODO for fullscreen) and upscales to a centered 768x768
 * region on a grey 1024x768 framebuffer. Continuous per-frame re-blit so the animation
 * wins against fbcon's klog mirror on the shared /dev/fb0.
 *
 * Links libGL-phoenix.a + libv3d-phoenix.a.
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

#define W 256
#define H 256
#define FB_W 1024
#define FB_H 768
#define SCALE 3
#define OFF_X ((FB_W - W * SCALE) / 2)
#define OFF_Y ((FB_H - H * SCALE) / 2)

static void chk(const char *where)
{
	GLenum e = glGetError();
	if (e != GL_NO_ERROR)
		printf("glcube: GL error 0x%x after %s\n", e, where);
}

/* one cube face: 4 vertices (a quad). */
static void face(const float n[3], const float v0[3], const float v1[3],
                 const float v2[3], const float v3[3], float r, float g, float b)
{
	(void)n;
	glColor3f(r, g, b);
	glVertex3fv(v0); glVertex3fv(v1); glVertex3fv(v2); glVertex3fv(v3);
}

static void draw_cube(void)
{
	/* 8 corners of a unit cube centered at origin. */
	static const float p[8][3] = {
		{-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
		{-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1},
	};
	static const float nz[3] = {0,0,0};
	glBegin(GL_QUADS);
	face(nz, p[4], p[5], p[6], p[7], 1.0f, 0.0f, 0.0f);  /* +Z red    */
	face(nz, p[1], p[0], p[3], p[2], 0.0f, 1.0f, 0.0f);  /* -Z green  */
	face(nz, p[5], p[1], p[2], p[6], 0.0f, 0.0f, 1.0f);  /* +X blue   */
	face(nz, p[0], p[4], p[7], p[3], 1.0f, 1.0f, 0.0f);  /* -X yellow */
	face(nz, p[7], p[6], p[2], p[3], 1.0f, 0.0f, 1.0f);  /* +Y magenta*/
	face(nz, p[0], p[1], p[5], p[4], 0.0f, 1.0f, 1.0f);  /* -Y cyan   */
	glEnd();
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("glcube: START (animated spinning cube via the GL frontend)\n");

	struct pipe_screen_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	struct pipe_screen *pscreen = v3d_screen_create(0, &cfg, NULL);
	if (!pscreen) { printf("glcube: pipe_screen NULL\n"); return 1; }
	struct pipe_context *pipe = pscreen->context_create(pscreen, NULL, 0);
	if (!pipe) { printf("glcube: pipe_context NULL\n"); return 1; }

	struct gl_config visual;
	struct st_config_options opts;
	memset(&visual, 0, sizeof(visual));
	memset(&opts, 0, sizeof(opts));
	struct st_context *st = st_create_context(API_OPENGL_COMPAT, pipe, &visual,
	                                          NULL, &opts, 0, 0);
	if (!st) { printf("glcube: st_create_context NULL\n"); return 1; }
	_mesa_make_current(st->ctx, NULL, NULL);
	printf("glcube: GL up; %s / %s\n",
	       (const char *)glGetString(GL_VERSION), (const char *)glGetString(GL_RENDERER));

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
	GLenum fbs = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	printf("glcube: FBO status=0x%x (complete=0x%x)\n", fbs, GL_FRAMEBUFFER_COMPLETE);

	glEnable(GL_DEPTH_TEST);
	glViewport(0, 0, W, H);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1.0, 1.0, -1.0, 1.0, 2.0, 20.0);  /* fovy ~53 deg, aspect 1 */
	glMatrixMode(GL_MODELVIEW);
	chk("setup");

	uint32_t *px = malloc((size_t)W * H * 4);
	uint32_t *fbimg = malloc((size_t)FB_W * FB_H * 4);
	int fb = open("/dev/fb0", O_WRONLY);
	if (fb < 0) printf("glcube: /dev/fb0 open failed (will still render)\n");
	printf("glcube: entering animation loop (per-frame render -> upscale -> /dev/fb0)\n");

	unsigned long frame = 0;
	for (;;) {
		/* DIAG (25aa): test whether PER-FRAME matrix updates reach the shader at all,
		 * using an animated glTranslatef with PURE LINEAR math (no sin/cos -- so this
		 * isolates "matrix re-upload dead" from "rotation/sincos broken"). If the cube
		 * slides horizontally across frames, per-frame translate works => the earlier
		 * frozen rotation is rotation/sincos-specific. If it stays put => the whole
		 * NOTE: a glRotatef-based spin currently renders garbage + hangs the render
		 * (rotation-specific, distinct from this proven-clean translate path; the
		 * SLCACTL uniform-cache fix made per-frame transforms work -- see UPDATE 25ak/25al).
		 * Using an animated translate (slide) as the working demo until the rotation
		 * render path is debugged. */
		/* full multi-axis tumble (depth-tested) -- works now that early-Z is disabled
		 * in the v3d driver (the EZ-on path hung on tilted depth-tested polys, 25aw). */
		float ay = (float)(frame % 360);
		float ax = (float)((frame * 7 / 10) % 360);
		glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		glTranslatef(0.0f, 0.0f, -5.0f);
		glRotatef(ay, 0.0f, 1.0f, 0.0f);
		glRotatef(ax, 1.0f, 0.0f, 0.0f);
		draw_cube();
		glFinish();

		glReadBuffer(GL_COLOR_ATTACHMENT0);
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);

		if (frame == 0) {
			chk("first frame");
			printf("glcube: frame0 center=0x%08x (cube face color, not clear 0xff1f1a1a)\n",
			       px[(H / 2) * W + (W / 2)]);
		}

		/* Correctness diagnostic (25ay): the scene is flat-colored (no lighting/blend/
		 * texture). Earlier exact-32-bit bucketing reported 100% "other" — but the bytes
		 * decode (LE RGBA = R|G<<8|B<<16|A<<24) to a DIM red +Z face (R~0x84 G=0 B~0x11,
		 * broken alpha) at frame 0, i.e. geometry is CORRECT and the defect is color
		 * intensity + alpha. So: (a) dump a raw 5x5 grid of pixel values, and (b) bucket
		 * with alpha MASKED and an RGB tolerance, classifying each pixel by nearest of
		 * {clear, 6 faces} within +-0x40 per channel. coverage% = pixels within tolerance
		 * of SOME palette entry; that should be ~100% if geometry+rough-color are right. */
		if ((frame % 120) == 0) {
			static const uint32_t pal[7] = {
				0x001f1a1au,  /* clear (rgb only) */
				0x000000ffu, 0x0000ff00u, 0x00ff0000u,
				0x0000ffffu, 0x00ff00ffu, 0x00ffff00u
			};
			size_t cover = 0; size_t tot = (size_t)W * H;
			for (size_t i = 0; i < tot; i++) {
				uint32_t c = px[i] & 0x00ffffffu;
				int r = c & 0xff, g = (c >> 8) & 0xff, b = (c >> 16) & 0xff;
				for (int k = 0; k < 7; k++) {
					int pr = pal[k] & 0xff, pg = (pal[k] >> 8) & 0xff, pb = (pal[k] >> 16) & 0xff;
					int dr = r - pr, dg = g - pg, db = b - pb;
					if (dr < 0) dr = -dr; if (dg < 0) dg = -dg; if (db < 0) db = -db;
					if (dr <= 0x40 && dg <= 0x40 && db <= 0x40) { cover++; break; }
				}
			}
			printf("glcube: COVER frame=%lu within-tol=%zu%% (of %zu) center=0x%08x\n",
			       frame, cover * 100 / tot, tot, px[(H / 2) * W + (W / 2)]);
			printf("glcube: GRID5x5 (raw RGBA):\n");
			for (int gy = 0; gy < 5; gy++) {
				int yy = (H - 1) * gy / 4;
				printf("glcube:  %08x %08x %08x %08x %08x\n",
				       px[yy * W + (W - 1) * 0 / 4], px[yy * W + (W - 1) * 1 / 4],
				       px[yy * W + (W - 1) * 2 / 4], px[yy * W + (W - 1) * 3 / 4],
				       px[yy * W + (W - 1) * 4 / 4]);
			}
		}

		/* upscale to centered region on grey, y-flipped (GL y-up -> screen y-down). */
		for (size_t i = 0; i < (size_t)FB_W * FB_H; i++)
			fbimg[i] = 0xff1f1a1a;
		for (int sy = 0; sy < H; sy++) {
			const uint32_t *srow = px + (size_t)(H - 1 - sy) * W;  /* flip */
			for (int sx = 0; sx < W; sx++) {
				uint32_t c = srow[sx];
				for (int dy = 0; dy < SCALE; dy++) {
					uint32_t *drow = fbimg + (size_t)(OFF_Y + sy * SCALE + dy) * FB_W + OFF_X + sx * SCALE;
					for (int dx = 0; dx < SCALE; dx++)
						drow[dx] = c;
				}
			}
		}
		if (fb >= 0) {
			lseek(fb, 0, SEEK_SET);
			(void)write(fb, fbimg, (size_t)FB_W * FB_H * 4);
		}
		if (frame == 0)
			printf("glcube: ANIMATING (spinning cube on /dev/fb0)\n");
		if ((frame % 120) == 0)
			printf("glcube: frame=%lu rot=%d center=0x%08x\n",
			       frame, (int)ay, px[(H / 2) * W + (W / 2)]);

		frame++;
		usleep(33000);  /* ~30 fps target */
	}
	close(fb);
	free(fbimg);
	free(px);
	return 0;
}
