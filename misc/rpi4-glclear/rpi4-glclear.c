/*
 * rpi4-glclear.c — GLQuake Path-C Phase-4: a Mesa OpenGL glClear on the V3D, run
 * on real hardware (boot-launched). glClear driven entirely through the OpenGL
 * API, rendered to an offscreen FBO and read back with glReadPixels:
 *
 *   v3d_screen_create -> pipe_context -> st_create_context (Mesa GL frontend)
 *   -> _mesa_make_current(ctx, NULL, NULL) (surfaceless, FBO-only)
 *   -> renderbuffer (GL_RGBA8) attached to an FBO color attachment
 *   -> glClearColor(green)+glClear -> glReadPixels -> expect 0xff00ff00
 *   -> blit to /dev/fb0.
 *
 * Surfaceless + FBO avoids the winsys-framebuffer/drawable machinery: the GL
 * context has no default framebuffer; we render to a user FBO. The renderbuffer
 * is the canonical offscreen render target (st allocates the backing pipe_resource)
 * and glReadPixels exercises the GL read path too.
 *
 * Links libGL-phoenix.a (the ported Mesa GL frontend) + libv3d-phoenix.a (the
 * ported v3d gallium driver). Bring-up harness, not an upstreamable component.
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
#include "main/menums.h"                 /* gl_api */
#include "frontend/api.h"                /* st_config_options */
#include "main/mtypes.h"                 /* gl_config, gl_context */
#include "state_tracker/st_context.h"    /* st_create_context */
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

static void chk(const char *where)
{
	GLenum e = glGetError();
	if (e != GL_NO_ERROR)
		printf("glclear: GL error 0x%x after %s\n", e, where);
}

static void blit_fb0(const uint32_t *src)
{
	int fd = open("/dev/fb0", O_WRONLY);
	if (fd < 0) { printf("glclear: /dev/fb0 open failed\n"); return; }
	/* fb0 is 1024x768x32, pitch 4096; place our W x H tile at the top-left. */
	const int pitch = 4096;
	for (int y = 0; y < H; y++) {
		lseek(fd, (off_t)y * pitch, SEEK_SET);
		write(fd, src + (size_t)y * W, W * 4);
	}
	close(fd);
	printf("glclear: blitted %dx%d to /dev/fb0\n", W, H);
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("glclear: START (Mesa OpenGL glClear on the V3D)\n");

	struct pipe_screen_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	struct pipe_screen *pscreen = v3d_screen_create(0, &cfg, NULL);
	if (!pscreen) { printf("glclear: pipe_screen NULL\n"); return 1; }
	struct pipe_context *pipe = pscreen->context_create(pscreen, NULL, 0);
	if (!pipe) { printf("glclear: pipe_context NULL\n"); return 1; }

	struct gl_config visual;
	struct st_config_options opts;
	memset(&visual, 0, sizeof(visual));
	memset(&opts, 0, sizeof(opts));
	struct st_context *st = st_create_context(API_OPENGL_COMPAT, pipe, &visual,
	                                          NULL, &opts, 0, 0);
	if (!st) { printf("glclear: st_create_context NULL\n"); return 1; }
	printf("glclear: GL context created\n");

	_mesa_make_current(st->ctx, NULL, NULL);
	printf("glclear: made current; GL_VERSION=%s GL_RENDERER=%s\n",
	       (const char *)glGetString(GL_VERSION), (const char *)glGetString(GL_RENDERER));

	/* renderbuffer-backed FBO: the canonical offscreen color target. */
	GLuint rb = 0, fbo = 0;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, W, H);
	chk("RenderbufferStorage");
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
	chk("FramebufferRenderbuffer");
	GLenum fbs = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	printf("glclear: FBO status=0x%x (complete=0x%x)\n", fbs, GL_FRAMEBUFFER_COMPLETE);

	glViewport(0, 0, W, H);
	glClearColor(0.0f, 1.0f, 0.0f, 1.0f);  /* green */
	glClear(GL_COLOR_BUFFER_BIT);
	glFinish();
	chk("Clear");
	printf("glclear: glClear+glFinish done\n");

	/* read back via the GL read path (reads from the bound FBO). */
	uint32_t *px = malloc((size_t)W * H * 4);
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
	chk("ReadPixels");
	printf("glclear: GLCLEAR readback center=0x%08x [0]=0x%08x (expect green: R=00 G=ff B=00 A=ff)\n",
	       px[(H / 2) * W + (W / 2)], px[0]);
	blit_fb0(px);
	printf("glclear: GLCLEAR-DONE\n");
	for (;;)
		sleep(5);
	free(px);
	return 0;
}
