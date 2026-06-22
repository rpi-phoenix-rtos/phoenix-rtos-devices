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

/* winsys-exported counters. render_timeouts increments on each CT1 render spin-timeout;
 * render_recoveries increments on EVERY wedge (bin or render) handled by the drop-frame
 * mitigation, so it is the total-wedge count. */
extern volatile unsigned v3d_phoenix_render_timeouts;
extern volatile unsigned v3d_phoenix_render_recoveries;
/* winsys-exported TRUE reset: hold-in-reset + power-on + re-apply core regs over the
 * surviving page table. Re-creates the cold first-frame-after-power-on condition per call,
 * so each loop iteration becomes an independent trial of the intermittent render stall. */
extern void v3d_phoenix_harness_reset(void);

#define FBW 1920
#define FBH 1088          /* tile-aligned 1080 (the size Quake render-to-scanout used) */
#define FRAMES 3000       /* continuous frames (NO reset between) — mimics sustained Quake play */

/* Render one COMPLEX, depth-tested, perspective frame: a dense field of steeply-tilted triangles
 * with heavy overdraw and per-frame rotation. This reproduces the conditions under which Quake
 * wedges (the Phoenix EZ note: "steeply-tilted polygons / steep screen-space Z gradient hang"),
 * exercising the binner + the fragment/DEPTH-output pipeline that fdbgs localised the stall to.
 * Crucially this renders to the harness FBO (a tiled/RASTER render target in normal DRAM), NOT
 * the scanout HDMI framebuffer — so if it wedges, the stall is reproducible WITHOUT
 * render-to-scanout (the rework would not help); if it never wedges across many complex frames,
 * the scanout-fb path is required to trigger it (the rework is justified). */
static void render_frame(int i)
{
	glViewport(0, 0, FBW, FBH);
	glClearColor((float)(i & 7) / 7.0f, 0.2f, 0.4f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1.0, 1.0, -0.5625, 0.5625, 1.0, 60.0);   /* perspective -> steep screen-space Z */

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -8.0f);
	float a = (float)i * 1.7f;
	glRotatef(a,        1.0f, 0.0f, 0.0f);   /* tilt toward edge-on (steep Z gradient) */
	glRotatef(a * 0.7f, 0.0f, 1.0f, 0.0f);
	glRotatef(a * 0.3f, 0.0f, 0.0f, 1.0f);

	/* Dense grid of depth-tested tilted triangles across depth (12*12*8 = 1152 tris/frame). */
	for (int gz = 0; gz < 12; gz++) {
		float z = -2.0f - (float)gz * 0.4f;
		for (int gx = -6; gx < 6; gx++) {
			for (int gy = -4; gy < 4; gy++) {
				float x = (float)gx * 0.5f, y = (float)gy * 0.5f;
				glBegin(GL_TRIANGLES);
				glColor3f((float)(gx + 6) / 12.0f, (float)(gy + 4) / 8.0f, (float)gz / 12.0f);
				glVertex3f(x - 0.3f, y - 0.3f, z);
				glVertex3f(x + 0.3f, y - 0.3f, z + 0.5f);   /* tilted in Z */
				glVertex3f(x,        y + 0.3f, z - 0.3f);
				glEnd();
			}
		}
	}

	glFinish();   /* synchronously submit + render -> winsys ioc_submit_cl (CT0 bin, CT1 render) */
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("stalltest: START (V3D render-stall repro, %dx%d x %d continuous frames)\n", FBW, FBH, FRAMES);

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
	/* Early-Z is now enabled by default in the v3d driver (the EZ hang was the since-fixed L2T
	 * flush race), so this harness's tilted depth-tested geometry exercises the EZ path directly. */

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

	/* Warm-up frame: forces winsys_init (power-on, MMU, L2C) before the measured loop. */
	render_frame(0);
	printf("stalltest: warm-up frame done (wedges=%u); starting continuous render loop\n",
	       v3d_phoenix_render_recoveries);

	/* Render FRAMES complex depth-tested frames CONTINUOUSLY (no reset between) — the workload
	 * profile under which Quake wedges. A frame "wedged" if the total-wedge counter advanced
	 * during it (the drop-frame mitigation then resets+drops it; we count it regardless). */
	unsigned wedged_frames = 0;
	for (int i = 0; i < FRAMES; i++) {
		unsigned before = v3d_phoenix_render_recoveries;
		render_frame(i + 1);
		unsigned delta = v3d_phoenix_render_recoveries - before;
		if (delta != 0) {
			wedged_frames++;
			printf("stalltest: frame %d WEDGED (+%u; wedged_frames=%u/%d)\n",
			       i + 1, delta, wedged_frames, i + 1);
		}
		if (((i + 1) % 200) == 0)
			printf("stalltest: %d/%d frames, wedged_frames=%u total_wedges=%u render_timeouts=%u\n",
			       i + 1, FRAMES, wedged_frames, v3d_phoenix_render_recoveries,
			       v3d_phoenix_render_timeouts);
	}

	printf("stalltest: RESULT frames=%d wedged_frames=%u total_wedges=%u render_timeouts=%u\n",
	       FRAMES, wedged_frames, v3d_phoenix_render_recoveries, v3d_phoenix_render_timeouts);
	printf("stalltest: (wedged_frames>0 => complex depth geometry on a tiled/RASTER NON-scanout RT "
	       "reproduces the stall -> render-to-scanout NOT required; ==0 over %d frames => the "
	       "scanout-fb path is required to trigger it -> the rework is justified)\n", FRAMES);
	printf("stalltest: DONE\n");
	for (;;)
		sleep(5);
	return 0;
}
