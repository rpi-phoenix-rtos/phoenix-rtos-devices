/*
 * rpi4-v3d-stalltest.c — deterministic repro harness for the intermittent V3D render
 * stall (task: residual render stall). Boot-roulette (one Quake demo per boot = one
 * sample) cannot validate a ~30-50% intermittent stall: ~15-20 spaced boots are needed
 * per data point and GPU heat-soak pollutes the signal. This harness instead does MANY
 * render submits within a SINGLE boot and counts how many hit the CT1 render timeout
 * (via the winsys-exported v3d_phoenix_render_timeouts counter), turning 1 sample/6min
 * into hundreds/boot.
 *
 * It renders a 1920x1088 frame (clear + an immediate-mode triangle) in a loop — matching
 * the conditions under which Quake stalled: a large RT (~510 tiles, the binner-overflow
 * path) plus QPU fragment-shader execution (the triangle), via the same v3d gallium driver
 * + winsys submit path the flagship uses.
 *
 * STAGED EXPERIMENT (this first version, no GPU reset between iterations):
 *   - If stalls appear ONLY on the first submit(s) and never later in the loop, the failure
 *     is first-submit/cold-state-only -> the next harness revision must reset/re-power the
 *     V3D between iterations to recreate the cold condition per trial.
 *   - If stalls are scattered throughout the loop, it is a per-submit race and THIS loop is
 *     already a fast statistical signal for testing fixes.
 * Either outcome decides the next step deterministically.
 *
 * Links libGL-phoenix.a + libv3d-phoenix.a (like rpi4-glclear). Bring-up harness, not an
 * upstreamable component.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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

/* winsys-exported render-timeout counter (incremented on each CT1 spin-timeout). */
extern volatile unsigned v3d_phoenix_render_timeouts;

#define FBW 1920
#define FBH 1088          /* tile-aligned 1080 (the size Quake render-to-scanout used) */
#define ITERS 300

/* Render one frame: clear + a single triangle (exercises binner tiling + QPU shaders). */
static void render_frame(int i)
{
	glViewport(0, 0, FBW, FBH);
	/* vary the clear colour per iteration so nothing can be optimised to a no-op */
	glClearColor((float)(i & 7) / 7.0f, 1.0f, 0.25f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	/* immediate-mode triangle -> Mesa compiles a fixed-function shader -> QPU execution */
	glBegin(GL_TRIANGLES);
	glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(-0.8f, -0.8f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f); glVertex3f( 0.8f, -0.8f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f); glVertex3f( 0.0f,  0.8f, 0.0f);
	glEnd();

	glFinish();   /* synchronously submit + render -> winsys ioc_submit_cl (CT0 bin, CT1 render) */
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("stalltest: START (V3D render-stall repro, %dx%d x %d iters)\n", FBW, FBH, ITERS);

	struct pipe_screen_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	struct pipe_screen *pscreen = v3d_screen_create(0, &cfg, NULL);
	if (!pscreen) { printf("stalltest: pipe_screen NULL\n"); return 1; }
	struct pipe_context *pipe = pscreen->context_create(pscreen, NULL, 0);
	if (!pipe) { printf("stalltest: pipe_context NULL\n"); return 1; }

	struct gl_config visual;
	struct st_config_options opts;
	memset(&visual, 0, sizeof(visual));
	memset(&opts, 0, sizeof(opts));
	struct st_context *st = st_create_context(API_OPENGL_COMPAT, pipe, &visual, NULL, &opts, 0, 0);
	if (!st) { printf("stalltest: st_create_context NULL\n"); return 1; }
	_mesa_make_current(st->ctx, NULL, NULL);
	printf("stalltest: GL up; %s / %s\n",
	       (const char *)glGetString(GL_VERSION), (const char *)glGetString(GL_RENDERER));

	GLuint rb = 0, rbd = 0, fbo = 0;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, FBW, FBH);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
	glGenRenderbuffers(1, &rbd);
	glBindRenderbuffer(GL_RENDERBUFFER, rbd);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, FBW, FBH);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbd);
	printf("stalltest: FBO %dx%d status=0x%x\n", FBW, FBH, glCheckFramebufferStatus(GL_FRAMEBUFFER));

	unsigned first_stall_iter = 0;   /* 0 = none */
	unsigned prev = v3d_phoenix_render_timeouts;
	for (int i = 0; i < ITERS; i++) {
		render_frame(i);
		unsigned now = v3d_phoenix_render_timeouts;
		if (now != prev) {
			if (first_stall_iter == 0)
				first_stall_iter = (unsigned)(i + 1);
			printf("stalltest: STALL at iter %d (total timeouts=%u)\n", i + 1, now);
			prev = now;
		}
		if (((i + 1) % 25) == 0)
			printf("stalltest: %d/%d done, stalls so far=%u\n", i + 1, ITERS, v3d_phoenix_render_timeouts);
	}

	printf("stalltest: RESULT iters=%d total_render_timeouts=%u first_stall_iter=%u\n",
	       ITERS, v3d_phoenix_render_timeouts, first_stall_iter);
	printf("stalltest: DONE\n");
	for (;;)
		sleep(5);
	return 0;
}
