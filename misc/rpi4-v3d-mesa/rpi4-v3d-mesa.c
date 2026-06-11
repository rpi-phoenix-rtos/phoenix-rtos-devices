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
#include "pipe/p_screen.h"
#include "pipe/p_context.h"
#include "pipe/p_defines.h"

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
	pctx->destroy(pctx);

	pscreen->destroy(pscreen);
	printf("rpi4-v3d-mesa: done\n");
	return 0;
}
