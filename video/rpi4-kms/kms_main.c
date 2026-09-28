/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms (/dev/kms, /kmsbuf)
 *
 * NEW GPU LANE, M2 Stage A (docs/gpu-new-lane/M2-kms-server.md). Owns the HDMI
 * display while it runs: modes and connectors from the firmware, a KMS object
 * model (one connector/encoder/CRTC, backend-defined planes), dumb BOs from one
 * contiguous exported pool, framebuffers, atomic commits and legacy page flips
 * with vblank-timed completion events read() from the card descriptor, and
 * in-fences checked against rpi4-v3d-async's fence page with no IPC.
 *
 * It must not run next to an old-lane full-screen app (any game, SDL2, the
 * glamor X server): the in-process winsys pans the same firmware framebuffer
 * through the raw mailbox FIFO. rpi4-fb (/dev/fb0) and fbcon may keep running:
 * they only touch slot 0, which this server never hands out.
 *
 * Usage: rpi4-kms [-f] [-b plane|pan] [-o overlays] [-p pool_mib] [-m <max_end>] [-c]
 *                 [-V irq|hvs|fwvsync|timer] [-g gate_us] [-K] [-L guard_us] [-G] [-B] [-C] [-F] [-v] [&]
 *        rpi4-kms -R        restore the display (unset planes, unblank, pan to 0, fbcon on) and exit
 *   (detaches itself: psh has no job control; a stray "&" argument is ignored)
 *   -f          stay in the foreground
 *   -b backend  plane (default: firmware SET_PLANE, proven by E3) or pan (firmware-fb
 *               panning, the old lane's mechanism)
 *   -o n        plane backend: expose n overlay planes (0..6, default 0)
 *   -p MiB      scan-out pool size (default 32 for plane, 4 for pan)
 *   -m max_end  the pool must end at or below this PA (default 1 GiB = all the
 *               firmware can scan, E6; lower only for experiments)
 *   -c          hand the firmware 0xC0000000|PA instead of the raw PA (both work, E6)
 *   -V src      force the vblank source (default: auto irq > hvs > fwvsync > timer)
 *   -g us       fence poll period while a commit waits for its in-fence (default 500)
 *   -K          A/B control only: a fence-deferred commit does not wake the vblank
 *               thread, so its fence is first checked at the next vblank (the
 *               behaviour before 2026-09-27; every GPU-rendered flip takes 2 vblanks)
 *   -L us       firmware latch guard (default 2000; E3: the list is written ~1.6 ms
 *               before the vblank)
 *   -G          connect to rpi4-v3d-async and map its fence page (in-fences)
 *   -B          blank the firmware fb while the primary plane shows (E3 stack 3/4)
 *   -C          console handover: FBCONSETMODE(DISABLED) while a plane is shown
 *               (pl011-tty then also releases /dev/kbd0)
 *   -F          start even if the firmware fb is not panned to 0 (someone else flips)
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <posix/utils.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <sys/types.h>

#include "kms.h"
#include "v3da_proto.h"

#ifdef KMS_POLL_NOTIFY
/* The kernel's poll() readiness wake-up (docs/gpu-new-lane/poll-wake.md). Declared
 * here too, so this builds against a libphoenix whose <sys/msg.h> predates it;
 * build.sh --poll-notify then links the syscall stub from tools/gpu-lane/pollwake. */
extern int pollNotify(const oid_t *oid);
#endif


_Static_assert(sizeof(kms_fence_t) == sizeof(v3da_fence_t), "kms_fence_t mirrors v3da_fence_t");
_Static_assert(sizeof(kms_memref_t) == sizeof(v3da_memref_t), "kms_memref_t mirrors v3da_memref_t");

kms_srv_t srv;
uint64_t kms_cnt_hz;
extern uint32_t kms_plane_overlays;

static struct {
	uint64_t cnt0;
	int64_t mono0_ns;
	uint8_t vblank_stack[16384] __attribute__((aligned(16)));
	uint8_t bufns_stack[8192] __attribute__((aligned(16)));
	uint32_t in_formats_blob[KMS_PLANES_PER_CRTC];
	uint32_t pid_notes;
	uint32_t attr_notes;
	int dri_name;                /* /dev/dri/card0 registered by this server */
	int poll_notify;             /* the kernel has pollNotify (KMS_POLL_NOTIFY builds only) */
} m;


void kms_log(const char *fmt, ...)
{
	va_list ap;
	char line[320];
	int n;

	n = snprintf(line, sizeof(line), "KMS ");
	va_start(ap, fmt);
	(void)vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
	va_end(ap);
	puts(line);   /* one write per line: stdout is line-buffered */
}


int64_t kms_cnt_to_mono_ns(uint64_t cnt)
{
	int64_t d = (int64_t)(cnt - m.cnt0);

	return m.mono0_ns + (d / (int64_t)kms_cnt_hz) * 1000000000LL + ((d % (int64_t)kms_cnt_hz) * 1000000000LL) /
		(int64_t)kms_cnt_hz;
}


/* ========================================================================= */
/* Clients and events                                                         */
/* ========================================================================= */

static kms_client_t *client_get(id_t id)
{
	if ((id < 1u) || (id > KMS_MAX_CLIENTS)) {
		return NULL;
	}
	return srv.clients[id - 1u].used ? &srv.clients[id - 1u] : NULL;
}


static int client_open(int pid)
{
	uint32_t i;

	for (i = 0u; i < KMS_MAX_CLIENTS; i++) {
		kms_client_t *c = &srv.clients[i];
		if (!c->used) {
			memset(c, 0, sizeof(*c));
			c->used = 1;
			c->id = i + 1u;
			c->pid = pid;
			return (int)c->id;
		}
	}
	return -ENFILE;
}


static void ev_push(uint32_t client, const void *ev)
{
	kms_client_t *c = client_get(client);

	if (c == NULL) {
		return;
	}
	if ((c->evtail - c->evhead) >= KMS_EVQ_LEN) {
		c->dropped++;
		srv.st.events_dropped++;
		return;
	}
	memcpy(c->evq[c->evtail % KMS_EVQ_LEN], ev, 32u);
	c->evtail++;
	srv.st.events_queued++;
#ifdef KMS_POLL_NOTIFY
	/* The queue the atPollStatus answer reads is updated above; now wake any
	 * poll()/select() on this client's descriptor (its oid.id is the client id). */
	if (m.poll_notify) {
		oid_t oid;
		oid.port = srv.port;
		oid.id = c->id;
		(void)pollNotify(&oid);
	}
#endif
}


static void ev_vblank(uint32_t client, uint32_t type, uint32_t crtc_id, uint64_t user, uint64_t seq, uint64_t cnt)
{
	int64_t ns = kms_cnt_to_mono_ns(cnt);

	if (type == KMS_DRM_EVENT_CRTC_SEQUENCE) {
		kms_drm_event_crtc_sequence_t e;
		e.base.type = type;
		e.base.length = sizeof(e);
		e.user_data = user;
		e.time_ns = ns;
		e.sequence = seq;
		ev_push(client, &e);
	}
	else {
		kms_drm_event_vblank_t e;
		e.base.type = type;
		e.base.length = sizeof(e);
		e.user_data = user;
		e.tv_sec = (uint32_t)(ns / 1000000000LL);
		e.tv_usec = (uint32_t)((ns % 1000000000LL) / 1000LL);
		e.sequence = (uint32_t)seq;
		e.crtc_id = crtc_id;
		ev_push(client, &e);
	}
}


/* Copy as many whole events as fit; returns bytes. */
static int ev_fill(kms_client_t *c, void *dst, size_t size)
{
	size_t done = 0u;

	while ((c->evhead != c->evtail) && (done + 32u <= size)) {
		memcpy((uint8_t *)dst + done, c->evq[c->evhead % KMS_EVQ_LEN], 32u);
		c->evhead++;
		done += 32u;
	}
	return (int)done;
}


/* A read() of at most 64 bytes arrives PACKED: the kernel points o.data at the
 * message's own o.raw (proc/msg.c msg_opack) and copies o.raw back at respond.
 * So o.data points into whichever msg_t the server received into; a COPY of the
 * message (a parked read) must be re-pointed at its own o.raw, or the fill lands
 * in the dispatch loop's stale stack copy and the client gets o.raw's zeros (the
 * m2-kms-a "event_bad type=0 len=0" defect). Same for i.data. */
static void msg_rebase(msg_t *dst, const msg_t *src)
{
	if ((src->o.data != NULL) && ((const uint8_t *)src->o.data >= src->o.raw) &&
			((const uint8_t *)src->o.data < src->o.raw + sizeof(src->o.raw))) {
		dst->o.data = dst->o.raw + ((const uint8_t *)src->o.data - src->o.raw);
	}
	if ((src->i.data != NULL) && ((const uint8_t *)src->i.data >= src->i.raw) &&
			((const uint8_t *)src->i.data < src->i.raw + sizeof(src->i.raw))) {
		dst->i.data = dst->i.raw + ((const uint8_t *)src->i.data - src->i.raw);
	}
}


/* The first reads served get one tagged line each: the proof, on the next Pi run,
 * that the bytes the client receives are the event (rate-limited, lock held). */
static void read_dump(const char *path, const msg_t *msg, int bytes)
{
	static uint32_t ndump;
	const uint32_t *w = (const uint32_t *)msg->o.data;
	int packed = ((msg->o.data != NULL) && ((const uint8_t *)msg->o.data >= msg->o.raw) &&
		((const uint8_t *)msg->o.data < msg->o.raw + sizeof(msg->o.raw))) ? 1 : 0;

	if ((ndump >= 3u) || (bytes < 16) || (w == NULL)) {
		return;
	}
	ndump++;
	KMS_LOG("srv read_dump n=%u path=%s o.size=%zu o.data=%p packed=%d bytes=%d first16=%08x %08x %08x %08x", ndump,
		path, msg->o.size, msg->o.data, packed, bytes, w[0], w[1], w[2], w[3]);
}


static int park(int kind, uint32_t client, uint32_t crtc, uint64_t target, const msg_t *msg, msg_rid_t rid)
{
	uint32_t i;

	for (i = 0u; i < KMS_MAX_PARKED; i++) {
		kms_parked_t *p = &srv.parked[i];
		if (!p->used) {
			p->used = 1;
			p->kind = kind;
			p->client = client;
			p->crtc = crtc;
			p->target_seq = target;
			p->deadline_cnt = kms_cnt() + kms_us_cnt((uint64_t)KMS_READ_MAX_MS * 1000u);
			p->msg = *msg;
			msg_rebase(&p->msg, msg);   /* a packed read's o.data must point at THIS copy's o.raw */
			p->rid = rid;
			srv.nparked++;
			if (srv.nparked > srv.st.reads_parked_max) {
				srv.st.reads_parked_max = srv.nparked;
			}
			return 0;
		}
	}
	return -ENOSPC;
}


/* Claim parked entry i: it leaves the table and goes to the answer list. */
static void claim(uint32_t i, kms_parked_t *answer, uint32_t *n)
{
	answer[*n] = srv.parked[i];
	msg_rebase(&answer[*n].msg, &srv.parked[i].msg);   /* keep a packed o.data on the copy that is responded */
	(*n)++;
	srv.parked[i].used = 0;
	srv.nparked--;
}


static void set_vblank_reply(kms_parked_t *p, int err, uint64_t seq, uint64_t cnt)
{
	kms_resp_t *r = (kms_resp_t *)p->msg.o.raw;

	memset(r, 0, sizeof(*r));
	r->op = KMS_OP_WAIT_VBLANK;
	r->err = err;
	r->u.vblank.sequence = seq;
	r->u.vblank.time_ns = kms_cnt_to_mono_ns(cnt);
	p->msg.o.err = EOK;
}


/* Serve parked reads whose queue has something (lock held). */
static void serve_reads(kms_parked_t *answer, uint32_t *n)
{
	uint32_t i;

	for (i = 0u; i < KMS_MAX_PARKED; i++) {
		kms_parked_t *p = &srv.parked[i];
		kms_client_t *c;

		if (!p->used || (p->kind != KMS_PARK_READ)) {
			continue;
		}
		c = client_get(p->client);
		if ((c != NULL) && (c->evhead != c->evtail)) {
			p->msg.o.err = ev_fill(c, p->msg.o.data, p->msg.o.size);
			read_dump("parked", &p->msg, p->msg.o.err);
			claim(i, answer, n);
		}
	}
}


void kms_expire_parked(uint64_t now, kms_parked_t *answer, uint32_t *n)
{
	uint32_t i;

	for (i = 0u; i < KMS_MAX_PARKED; i++) {
		kms_parked_t *p = &srv.parked[i];
		if (!p->used || (now < p->deadline_cnt)) {
			continue;
		}
		if (p->kind == KMS_PARK_READ) {
			p->msg.o.err = -EAGAIN;   /* bounded: a parked client cannot be interrupted (E5) */
		}
		else {
			set_vblank_reply(p, -EAGAIN, srv.crtc[0].seq, srv.crtc[0].last_vbl_cnt);
		}
		claim(i, answer, n);
	}
}


void kms_answer(kms_parked_t *list, uint32_t n)
{
	uint32_t i;

	for (i = 0u; i < n; i++) {
		(void)msgRespond(srv.port, &list[i].msg, list[i].rid);
	}
}


/* ========================================================================= */
/* Render-server fences (optional, -G)                                        */
/* ========================================================================= */

static void v3d_connect(void)
{
	v3da_hello_t h;
	void *p;
	int fd;

	fd = open("/dev/" V3DA_DEV_NAME, O_RDONLY);
	if (fd < 0) {
		KMS_LOG("v3d connect=0 why=no_/dev/%s (in-fences refused)", V3DA_DEV_NAME);
		return;
	}
	memset(&h, 0, sizeof(h));
	h.proto = V3DA_PROTO_BASE;   /* fences only: works with every render server since M1 part 2 */
	if ((ioctl(fd, V3DA_IOC_HELLO, &h) < 0) || (h.fence_page.kind != V3DA_MEM_PHYS)) {
		KMS_LOG("v3d connect=0 why=hello errno=%d", errno);
		close(fd);
		return;
	}
	/* Cached, read-only: every mapping of the page must be cached (E5 section 1(d)). */
	p = mmap(NULL, V3DA_FENCE_PAGE_SIZE, PROT_READ, MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)h.fence_page.addr);
	if (p == MAP_FAILED) {
		KMS_LOG("v3d connect=0 why=map errno=%d", errno);
		close(fd);
		return;
	}
	srv.v3d_fp = p;   /* keep fd open: closing it frees our client slot in the render server */
	KMS_LOG("v3d connect=1 fence_pa=0x%llx client=%u server_pid=%u", (unsigned long long)h.fence_page.addr,
		h.client_id, h.server_pid);
}


int kms_fence_signaled(const kms_fence_t *f)
{
	const v3da_fence_page_t *fp = (const v3da_fence_page_t *)srv.v3d_fp;
	uint64_t gen, done;

	if ((f->seqno == 0u) || (fp == NULL)) {
		return 1;
	}
	if ((fp->hdr.flags & V3DA_FP_EXITED) != 0u) {
		return 1;   /* render server gone: never wedge the display on it */
	}
	if ((f->slot >= V3DA_FENCE_NSLOTS) || (f->queue >= V3DA_Q_COUNT)) {
		return 1;
	}
	gen = __atomic_load_n(&fp->slot[f->slot].gen, __ATOMIC_ACQUIRE);
	if ((uint32_t)gen != f->gen) {
		return 1;   /* the slot was reassigned: the submitting client is gone */
	}
	done = __atomic_load_n(&fp->slot[f->slot].completed[f->queue], __ATOMIC_ACQUIRE);
	return (done >= f->seqno) ? 1 : 0;
}


/* ========================================================================= */
/* Commits                                                                    */
/* ========================================================================= */

static kms_crtc_state_t *crtc_by_id(uint32_t id)
{
	uint32_t i;

	for (i = 0u; i < srv.ncrtc; i++) {
		if (KMS_ID_CRTC(i) == id) {
			return &srv.crtc[i];
		}
	}
	return NULL;
}


static int plane_idx(const kms_crtc_state_t *c, uint32_t plane_id, uint32_t *p)
{
	uint32_t i;

	for (i = 0u; i < KMS_PLANES_PER_CRTC; i++) {
		if (((c->plane_mask & (1u << i)) != 0u) && (KMS_ID_PLANE((uint32_t)c->idx, i) == plane_id)) {
			*p = i;
			return 0;
		}
	}
	return -EINVAL;
}


static uint32_t plane_type(uint32_t p)
{
	return (p == 0u) ? KMS_PLANE_TYPE_PRIMARY : ((p == 7u) ? KMS_PLANE_TYPE_CURSOR : KMS_PLANE_TYPE_OVERLAY);
}


static void state_fullscreen(const kms_crtc_state_t *c, const kms_fb_t *fb, uint32_t p, kms_atomic_plane_t *st)
{
	uint32_t w = (srv.be->id == KMS_BACKEND_PAN) ? srv.fb_w : c->mode.hdisplay;
	uint32_t h = (srv.be->id == KMS_BACKEND_PAN) ? srv.fb_h : c->mode.vdisplay;

	memset(st, 0, sizeof(*st));
	st->plane_id = KMS_ID_PLANE((uint32_t)c->idx, p);
	st->crtc_id = KMS_ID_CRTC((uint32_t)c->idx);
	st->fb_id = fb->id;
	st->crtc_w = w;
	st->crtc_h = h;
	st->src_w = fb->w << 16;
	st->src_h = fb->h << 16;
	st->alpha = 0xffffu;
	st->rotation = 1u;
	st->zpos = (int32_t)p;
}


static int validate(uint32_t client, const kms_crtc_state_t *c, uint32_t p, const kms_atomic_plane_t *st)
{
	kms_fb_t *fb = NULL;
	kms_bo_t *bo = NULL;

	if (st->fb_id != 0u) {
		fb = kms_fb_lookup(st->fb_id);
		if ((fb == NULL) || !fb->user_ref) {
			return -ENOENT;
		}
		if (fb->owner != client) {
			return -EACCES;   /* Stage A: a client flips its own framebuffers */
		}
		bo = &srv.bos[fb->bo];
		if ((st->crtc_id != 0u) && (st->crtc_id != KMS_ID_CRTC((uint32_t)c->idx))) {
			return -EINVAL;
		}
		if ((st->in_fence.seqno != 0u) && (srv.v3d_fp == NULL)) {
			return -ENODEV;   /* an in-fence needs the render server's fence page (-G) */
		}
	}
	return srv.be->check(c, p, st, fb, bo);
}


static void console_update(void)
{
	uint32_t p, on = 0u;
	const kms_crtc_state_t *c = &srv.crtc[0];

	if (srv.blank_fb && (srv.be->id == KMS_BACKEND_PLANE)) {
		int prim = (c->cur[0].fb_id != 0u) || ((c->pend != KMS_PEND_NONE) && ((c->pmask & 1u) != 0u) &&
			(c->pst[0].fb_id != 0u));
		if (prim != srv.fb_blanked) {
			int rc = kms_fw_blank(prim);
			if (rc == 0) {
				srv.fb_blanked = prim;
			}
			if (srv.verbose || (rc != 0)) {
				KMS_LOG("fb blank=%d rc=%d", prim, rc);
			}
		}
	}
	if (!srv.console_off) {
		return;
	}
	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		if ((c->cur[p].fb_id != 0u) || ((c->pend != KMS_PEND_NONE) && ((c->pmask & (1u << p)) != 0u) &&
				(c->pst[p].fb_id != 0u))) {
			on = 1u;
		}
	}
	if (on && !srv.console_disabled) {
		KMS_LOG("console handover disable rc=%d", kms_fw_console(0));
		srv.console_disabled = 1;
	}
	else if (!on && srv.console_disabled) {
		KMS_LOG("console handover enable rc=%d", kms_fw_console(1));
		srv.console_disabled = 0;
	}
}


/* Hand the pending commit to the firmware if every in-fence has signalled.
 * Returns 1 when the commit is (now or already) armed. Lock held. */
int kms_try_apply(kms_crtc_state_t *c)
{
	uint32_t p, lat = 0u, maxlat = 0u;
	int rc;

	if (c->pend != KMS_PEND_FENCE) {
		return (c->pend == KMS_PEND_ARMED) ? 1 : 0;
	}
	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		if (((c->pmask & (1u << p)) != 0u) && !kms_fence_signaled(&c->pst[p].in_fence)) {
			return 0;
		}
	}
	console_update();
	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		kms_fb_t *fb;

		if ((c->pmask & (1u << p)) == 0u) {
			continue;
		}
		fb = kms_fb_lookup(c->pst[p].fb_id);
		rc = srv.be->apply(c, p, &c->pst[p], fb, (fb != NULL) ? &srv.bos[fb->bo] : NULL, &lat);
		if ((rc == 0) && (fb != NULL) &&
				((srv.bos[fb->bo].kind == KMS_BOK_IMPORT) || (srv.bos[fb->bo].kind == KMS_BOK_ALIAS))) {
			kms_import_shown(&srv.bos[fb->bo], fb->id);   /* G7: a foreign buffer went to the firmware */
		}
		if (rc != 0) {
			/* The commit was accepted: DRM has no error channel from here on. The event
			 * still fires so no client waits forever; the first failures are logged. */
			if (srv.st.apply_errors++ < 5u) {
				KMS_LOG("apply FAIL crtc=%d plane=%u fb=%u rc=%d (#%u)", c->idx, p, c->pst[p].fb_id, rc,
					srv.st.apply_errors);
			}
		}
		maxlat = (lat > maxlat) ? lat : maxlat;
	}
	c->parmed_cnt = kms_cnt();
	c->pend = KMS_PEND_ARMED;
	/* E3 `plane latch`: a SET_PLANE issued right after a vblank IRQ appears in
	 * the HVS list 15.04 ms later (p50 15041 us, max 15051), i.e. the firmware
	 * writes pending plane state at a fixed point ~1.6 ms before the next
	 * vblank, and the HVS scans it from that vblank. So a commit armed more than
	 * (frame - guard) after the last vblank has missed that point and is scanned
	 * one vblank later. last_vbl_cnt/seq may lag an ISR vblank the thread has not
	 * processed yet; the rule then errs late (never early). Pan uses the same rule
	 * [inferred: its latch point is not measured]. */
	{
		uint32_t period = (c->refresh_mhz != 0u) ? (uint32_t)(1000000000ULL / c->refresh_mhz) : 16667u;
		uint64_t since = (c->last_vbl_cnt != 0u) ? kms_cnt_us(c->parmed_cnt - c->last_vbl_cnt) : period;
		c->ptarget_seq = c->seq + (((since + srv.latch_guard_us) < period) ? 1u : 2u);
	}
	{
		uint32_t q2a = (uint32_t)kms_cnt_us(c->parmed_cnt - c->pqueued_cnt);
		srv.fst.q2a_us_sum += q2a;
		srv.fst.q2a_us_max = (q2a > srv.fst.q2a_us_max) ? q2a : srv.fst.q2a_us_max;
		if (c->ptarget_seq > c->seq + 1u) {
			srv.fst.late_target++;
		}
	}
	srv.st.flips_applied++;
	if (maxlat > srv.st.apply_us_max) {
		srv.st.apply_us_max = maxlat;
	}
	return 1;
}


/* The armed commit is on screen from this vblank on: swap state, release the
 * outgoing framebuffers, deliver the event. */
static void complete(kms_crtc_state_t *c, uint64_t cnt)
{
	uint32_t p;

	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		kms_fb_t *old;
		if ((c->pmask & (1u << p)) == 0u) {
			continue;
		}
		old = kms_fb_lookup(c->cur[p].fb_id);
		c->cur[p] = c->pst[p];   /* the pending reference moves to "on screen" */
		kms_fb_unref(old);
	}
	if (c->pevent) {
		ev_vblank(c->pclient, KMS_DRM_EVENT_FLIP_COMPLETE, KMS_ID_CRTC((uint32_t)c->idx), c->puser, c->seq, cnt);
	}
	c->pend = KMS_PEND_NONE;
	c->pmask = 0u;
	srv.st.flips_completed++;
	{
		uint64_t k = c->seq - c->pqueued_seq;
		srv.fst.flips++;
		if (k <= 1u) {
			srv.fst.vbl1++;
		}
		else if (k == 2u) {
			srv.fst.vbl2++;
		}
		else {
			srv.fst.vbl3p++;
		}
	}
	console_update();
}


void kms_on_vblank(kms_crtc_state_t *c, uint64_t cnt, uint32_t nvbl, kms_parked_t *answer, uint32_t *n)
{
	uint32_t i;

	c->seq += nvbl;
	c->last_vbl_cnt = cnt;

	/* The armed commit is on screen from its target vblank (kms_try_apply). */
	if ((c->pend == KMS_PEND_ARMED) && (c->seq >= c->ptarget_seq)) {
		complete(c, cnt);
	}
	else if (c->pend == KMS_PEND_FENCE) {
		if (kms_try_apply(c)) {
			srv.fst.applied_vblank++;
		}
	}

	for (i = 0u; i < KMS_MAX_VBL_EVENTS; i++) {
		kms_vblev_t *e = &srv.vblev[i];
		if (e->used && (e->crtc == KMS_ID_CRTC((uint32_t)c->idx)) && (e->target_seq <= c->seq)) {
			ev_vblank(e->client, e->type, e->crtc, e->user_data, c->seq, cnt);
			e->used = 0;
		}
	}
	for (i = 0u; i < KMS_MAX_PARKED; i++) {
		kms_parked_t *p = &srv.parked[i];
		if (p->used && (p->kind == KMS_PARK_VBLANK) && (p->crtc == KMS_ID_CRTC((uint32_t)c->idx)) &&
				(p->target_seq <= c->seq)) {
			set_vblank_reply(p, 0, c->seq, cnt);
			claim(i, answer, n);
		}
	}
	serve_reads(answer, n);
}


/* Accept a commit (lock held). Returns 0 (queued/applied) or -errno. */
static int commit(uint32_t client, kms_crtc_state_t *c, const kms_atomic_plane_t *sts, uint32_t nst, uint32_t flags,
	uint64_t user, kms_flip_resp_t *out)
{
	uint32_t i, p, mask = 0u, idx[KMS_ATOMIC_MAX_PLANES];
	int rc;

	if (nst > KMS_ATOMIC_MAX_PLANES) {
		return -E2BIG;
	}
	for (i = 0u; i < nst; i++) {
		rc = plane_idx(c, sts[i].plane_id, &p);
		if (rc != 0) {
			return rc;
		}
		if ((mask & (1u << p)) != 0u) {
			return -EINVAL;   /* one state per plane */
		}
		mask |= 1u << p;
		idx[i] = p;
		rc = validate(client, c, p, &sts[i]);
		if (rc != 0) {
			return rc;
		}
	}
	if ((flags & KMS_ATOMIC_TEST_ONLY) != 0u) {
		return 0;
	}
	if (c->pend != KMS_PEND_NONE) {
		return -EBUSY;   /* one commit in flight per CRTC, as DRM */
	}
	for (i = 0u; i < nst; i++) {
		p = idx[i];
		c->pst[p] = sts[i];
		c->pst[p].crtc_id = (sts[i].fb_id != 0u) ? KMS_ID_CRTC((uint32_t)c->idx) : 0u;
		kms_fb_ref(kms_fb_lookup(sts[i].fb_id));
	}
	c->pmask = mask;
	c->pclient = client;
	c->puser = user;
	c->pevent = ((flags & KMS_PAGE_FLIP_EVENT) != 0u) ? 1 : 0;
	c->pqueued_cnt = kms_cnt();
	c->pqueued_seq = c->seq;
	c->pend = KMS_PEND_FENCE;
	if (out != NULL) {
		out->sequence = c->seq;
	}
	if (kms_try_apply(c)) {
		if (out != NULL) {
			out->applied = 1u;
			out->apply_us = (uint32_t)kms_cnt_us(c->parmed_cnt - c->pqueued_cnt);
		}
	}
	else {
		srv.st.flips_fence_deferred++;
		srv.fst.deferred++;
		/* The vblank thread chose its wait when no commit was pending, so it would
		 * look at this fence only at the next vblank: arm there, scan out one vblank
		 * later, i.e. every GPU-rendered flip takes two frames (30 fps at 60 Hz for
		 * a client that renders after each flip event, the pollwake-kmscube result).
		 * Wake it now; from its next iteration it polls the fence every gate_us. */
		if (!srv.no_kick) {
			(void)__atomic_add_fetch(&srv.gate_kick, 1u, __ATOMIC_RELEASE);
			(void)condSignal(srv.vbl_cond);
		}
	}
	return 0;
}


/* Take planes showing a framebuffer of `client` (or fb_id, if nonzero) off the
 * screen at once (RMFB and client death). Returns the number of planes. */
static uint32_t planes_off(kms_crtc_state_t *c, uint32_t client, uint32_t fb_id)
{
	uint32_t p, n = 0u;

	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		kms_fb_t *fb = kms_fb_lookup(c->cur[p].fb_id);
		if ((fb == NULL) || ((fb_id != 0u) ? (fb->id != fb_id) : (fb->owner != client))) {
			continue;
		}
		(void)srv.be->apply(c, p, NULL, NULL, NULL, NULL);
		memset(&c->cur[p], 0, sizeof(c->cur[p]));
		kms_fb_unref(fb);
		n++;
	}
	console_update();
	return n;
}


static void client_close(id_t id, kms_parked_t *answer, uint32_t *n)
{
	kms_client_t *cl = client_get(id);
	kms_crtc_state_t *c = &srv.crtc[0];
	uint32_t i, p, off;

	if (cl == NULL) {
		return;
	}
	for (i = 0u; i < KMS_MAX_PARKED; i++) {
		kms_parked_t *pk = &srv.parked[i];
		if (pk->used && (pk->client == id)) {
			if (pk->kind == KMS_PARK_READ) {
				pk->msg.o.err = -EPIPE;
			}
			else {
				set_vblank_reply(pk, -EPIPE, c->seq, c->last_vbl_cnt);
			}
			claim(i, answer, n);
		}
	}
	for (i = 0u; i < KMS_MAX_VBL_EVENTS; i++) {
		if (srv.vblev[i].used && (srv.vblev[i].client == id)) {
			srv.vblev[i].used = 0;
		}
	}
	if ((c->pend != KMS_PEND_NONE) && (c->pclient == id)) {
		if (c->pend == KMS_PEND_FENCE) {
			for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
				if ((c->pmask & (1u << p)) != 0u) {
					kms_fb_unref(kms_fb_lookup(c->pst[p].fb_id));
				}
			}
			c->pend = KMS_PEND_NONE;
			c->pmask = 0u;
		}
		else {
			c->pevent = 0;
			complete(c, kms_cnt());   /* the firmware already has it; settle it now, no event */
		}
	}
	off = planes_off(c, (uint32_t)id, 0u);   /* the console comes back when a client dies */
	kms_bo_client_gone((uint32_t)id);
	cl->used = 0;
	KMS_LOG("srv client %u closed planes_off=%u dropped_events=%u", (unsigned)id, off, cl->dropped);
	if ((srv.fst.flips != 0u) || (srv.fst.deferred != 0u)) {
		const kms_flipstat_t *f = &srv.fst;
		uint32_t napplied = f->flips;   /* every armed commit completed (client_close settles its own) */
		KMS_LOG("srv flipstat client=%u flips=%u vbl1=%u vbl2=%u vbl3p=%u deferred=%u applied_gate=%u "
			"applied_vblank=%u kicks=%u late_target=%u q2a_us_avg=%u q2a_us_max=%u kick=%d gate_us=%u",
			(unsigned)id, f->flips, f->vbl1, f->vbl2, f->vbl3p, f->deferred, f->applied_gate, f->applied_vblank,
			f->kicks, f->late_target, (napplied != 0u) ? (uint32_t)(f->q2a_us_sum / napplied) : 0u, f->q2a_us_max,
			!srv.no_kick, srv.gate_us);
		memset(&srv.fst, 0, sizeof(srv.fst));
	}
}


/* ========================================================================= */
/* Properties                                                                 */
/* ========================================================================= */

static const kms_prop_enum_t en_type[] = { { 0, "Overlay" }, { 1, "Primary" }, { 2, "Cursor" } };
static const kms_prop_enum_t en_dpms[] = { { 0, "On" }, { 1, "Standby" }, { 2, "Suspend" }, { 3, "Off" } };
static const kms_prop_enum_t en_link[] = { { 0, "Good" }, { 1, "Bad" } };
static const kms_prop_enum_t en_rot[] = { { 0, "rotate-0" }, { 2, "rotate-180" }, { 4, "reflect-x" }, { 5, "reflect-y" } };

typedef struct {
	uint32_t id;
	const char *name;
	uint32_t flags;
	uint64_t min, max;
	const kms_prop_enum_t *en;
	uint32_t nen;
} propdesc_t;

#define RANGE KMS_PROP_FLAG_RANGE
#define IMM   KMS_PROP_FLAG_IMMUTABLE
#define ATOM  KMS_PROP_FLAG_ATOMIC

static const propdesc_t props[] = {
	{ KMS_PROP_TYPE, "type", KMS_PROP_FLAG_ENUM | IMM, 0, 0, en_type, 3 },
	{ KMS_PROP_FB_ID, "FB_ID", KMS_PROP_FLAG_OBJECT | ATOM, KMS_OBJ_FB, 0, NULL, 0 },
	{ KMS_PROP_CRTC_ID, "CRTC_ID", KMS_PROP_FLAG_OBJECT | ATOM, KMS_OBJ_CRTC, 0, NULL, 0 },
	{ KMS_PROP_SRC_X, "SRC_X", RANGE | ATOM, 0, 0xffffffffu, NULL, 0 },
	{ KMS_PROP_SRC_Y, "SRC_Y", RANGE | ATOM, 0, 0xffffffffu, NULL, 0 },
	{ KMS_PROP_SRC_W, "SRC_W", RANGE | ATOM, 0, 0xffffffffu, NULL, 0 },
	{ KMS_PROP_SRC_H, "SRC_H", RANGE | ATOM, 0, 0xffffffffu, NULL, 0 },
	{ KMS_PROP_CRTC_X, "CRTC_X", KMS_PROP_FLAG_SIGNED | ATOM, (uint64_t)(int64_t)-2147483648LL, 2147483647u, NULL, 0 },
	{ KMS_PROP_CRTC_Y, "CRTC_Y", KMS_PROP_FLAG_SIGNED | ATOM, (uint64_t)(int64_t)-2147483648LL, 2147483647u, NULL, 0 },
	{ KMS_PROP_CRTC_W, "CRTC_W", RANGE | ATOM, 0, 2147483647u, NULL, 0 },
	{ KMS_PROP_CRTC_H, "CRTC_H", RANGE | ATOM, 0, 2147483647u, NULL, 0 },
	{ KMS_PROP_IN_FENCE_FD, "IN_FENCE_FD", KMS_PROP_FLAG_SIGNED | ATOM, (uint64_t)(int64_t)-1, 2147483647u, NULL, 0 },
	{ KMS_PROP_IN_FORMATS, "IN_FORMATS", KMS_PROP_FLAG_BLOB | IMM, 0, 0, NULL, 0 },
	{ KMS_PROP_ZPOS, "zpos", RANGE, 0, 7, NULL, 0 },
	{ KMS_PROP_ALPHA, "alpha", RANGE, 0, 0xffffu, NULL, 0 },
	{ KMS_PROP_ROTATION, "rotation", KMS_PROP_FLAG_BITMASK, 0, 0, en_rot, 4 },
	{ KMS_PROP_ACTIVE, "ACTIVE", RANGE | ATOM, 0, 1, NULL, 0 },
	{ KMS_PROP_MODE_ID, "MODE_ID", KMS_PROP_FLAG_BLOB | ATOM, 0, 0, NULL, 0 },
	{ KMS_PROP_OUT_FENCE_PTR, "OUT_FENCE_PTR", RANGE | ATOM, 0, ~0ull, NULL, 0 },
	{ KMS_PROP_VRR_ENABLED, "VRR_ENABLED", RANGE, 0, 1, NULL, 0 },
	{ KMS_PROP_DPMS, "DPMS", KMS_PROP_FLAG_ENUM, 0, 0, en_dpms, 4 },
	{ KMS_PROP_EDID, "EDID", KMS_PROP_FLAG_BLOB | IMM, 0, 0, NULL, 0 },
	{ KMS_PROP_LINK_STATUS, "link-status", KMS_PROP_FLAG_ENUM, 0, 0, en_link, 2 },
	{ KMS_PROP_NON_DESKTOP, "non-desktop", RANGE | IMM, 0, 1, NULL, 0 },
};


static const propdesc_t *prop_desc(uint32_t id)
{
	uint32_t i;

	for (i = 0u; i < sizeof(props) / sizeof(props[0]); i++) {
		if (props[i].id == id) {
			return &props[i];
		}
	}
	return NULL;
}


/* The property list of an object with current values. Returns the count. */
static uint32_t obj_props(uint32_t obj, kms_prop_value_t *out, uint32_t max)
{
	kms_prop_value_t v[20];
	const kms_crtc_state_t *c = &srv.crtc[0];
	uint32_t n = 0u, i, p;

#define PV(id_, val_) do { v[n].prop_id = (id_); v[n].pad = 0u; v[n].value = (uint64_t)(val_); n++; } while (0)
	if (obj == KMS_ID_CONNECTOR(0)) {
		PV(KMS_PROP_CRTC_ID, KMS_ID_CRTC(0));
		PV(KMS_PROP_DPMS, 0);
		PV(KMS_PROP_EDID, c->edid_blob);
		PV(KMS_PROP_LINK_STATUS, 0);
		PV(KMS_PROP_NON_DESKTOP, 0);
	}
	else if (obj == KMS_ID_CRTC(0)) {
		PV(KMS_PROP_ACTIVE, 1);
		PV(KMS_PROP_MODE_ID, c->mode_blob);
		PV(KMS_PROP_OUT_FENCE_PTR, 0);
		PV(KMS_PROP_VRR_ENABLED, 0);
	}
	else if ((plane_idx(c, obj, &p) == 0)) {
		const kms_atomic_plane_t *s = &c->cur[p];
		PV(KMS_PROP_TYPE, plane_type(p));
		PV(KMS_PROP_FB_ID, s->fb_id);
		PV(KMS_PROP_CRTC_ID, s->crtc_id);
		PV(KMS_PROP_SRC_X, s->src_x);
		PV(KMS_PROP_SRC_Y, s->src_y);
		PV(KMS_PROP_SRC_W, s->src_w);
		PV(KMS_PROP_SRC_H, s->src_h);
		PV(KMS_PROP_CRTC_X, (int64_t)s->crtc_x);
		PV(KMS_PROP_CRTC_Y, (int64_t)s->crtc_y);
		PV(KMS_PROP_CRTC_W, s->crtc_w);
		PV(KMS_PROP_CRTC_H, s->crtc_h);
		PV(KMS_PROP_IN_FENCE_FD, (int64_t)-1);
		PV(KMS_PROP_IN_FORMATS, m.in_formats_blob[p]);
		if (srv.be->id == KMS_BACKEND_PLANE) {
			PV(KMS_PROP_ZPOS, (p == 7u) ? 7u : p);
			PV(KMS_PROP_ALPHA, (s->fb_id != 0u) ? s->alpha : 0xffffu);
			PV(KMS_PROP_ROTATION, (s->rotation != 0u) ? s->rotation : 1u);
		}
	}
#undef PV
	for (i = 0u; (i < n) && (i < max); i++) {
		out[i] = v[i];
	}
	return n;
}


/* DRM struct drm_format_modifier_blob + formats[] + one LINEAR modifier. */
static uint32_t in_formats_blob(uint32_t p)
{
	uint32_t buf[32], fmts[KMS_PLANE_MAX_FMTS], n, i, off_mod;
	uint8_t *b = (uint8_t *)buf;

	n = srv.be->formats(p, fmts, KMS_PLANE_MAX_FMTS);
	memset(buf, 0, sizeof(buf));
	off_mod = (24u + 4u * n + 7u) & ~7u;
	buf[0] = 1u;       /* FORMAT_BLOB_CURRENT */
	buf[1] = 0u;
	buf[2] = n;
	buf[3] = 24u;
	buf[4] = 1u;       /* one modifier */
	buf[5] = off_mod;
	for (i = 0u; i < n; i++) {
		buf[6u + i] = fmts[i];
	}
	{
		uint64_t fmask = (n >= 64u) ? ~0ull : ((1ull << n) - 1ull), mod = KMS_MOD_LINEAR;
		uint32_t zero = 0u;
		memcpy(b + off_mod, &fmask, 8u);        /* formats bitmask */
		memcpy(b + off_mod + 8u, &zero, 4u);    /* offset */
		memcpy(b + off_mod + 12u, &zero, 4u);   /* pad */
		memcpy(b + off_mod + 16u, &mod, 8u);    /* modifier */
	}
	return kms_blob_create(0u, buf, off_mod + 24u);
}


/* ========================================================================= */
/* Requests                                                                   */
/* ========================================================================= */

static int get_cap(uint64_t cap, uint64_t *v)
{
	switch (cap) {
		case KMS_CAP_DUMB_BUFFER: *v = 1u; return 0;
		case KMS_CAP_VBLANK_HIGH_CRTC: *v = 1u; return 0;
		case KMS_CAP_DUMB_PREFERRED_DEPTH: *v = 24u; return 0;
		case KMS_CAP_DUMB_PREFER_SHADOW: *v = 0u; return 0;
		case KMS_CAP_PRIME: *v = 3u; return 0;   /* IMPORT | EXPORT (export in Stage A) */
		case KMS_CAP_TIMESTAMP_MONOTONIC: *v = 1u; return 0;
		case KMS_CAP_ASYNC_PAGE_FLIP: *v = 0u; return 0;
		case KMS_CAP_CURSOR_WIDTH: *v = 64u; return 0;
		case KMS_CAP_CURSOR_HEIGHT: *v = 64u; return 0;
		case KMS_CAP_ADDFB2_MODIFIERS: *v = 1u; return 0;
		case KMS_CAP_PAGE_FLIP_TARGET: *v = 0u; return 0;
		case KMS_CAP_CRTC_IN_VBLANK_EVENT: *v = 1u; return 0;
		case KMS_CAP_SYNCOBJ: *v = 0u; return 0;   /* syncobjs live on the render node */
		case KMS_CAP_SYNCOBJ_TIMELINE: *v = 0u; return 0;
		case KMS_CAP_ATOMIC_ASYNC_PAGE_FLIP: *v = 0u; return 0;
		default: return -EINVAL;
	}
}


static int op_get_connector(uint32_t id, kms_connector_t *o, msg_t *msg, uint32_t max)
{
	const kms_crtc_state_t *c = &srv.crtc[0];

	if (id != KMS_ID_CONNECTOR(0)) {
		return -ENOENT;
	}
	o->type = KMS_CONNECTOR_HDMIA;
	o->type_id = 1u;
	o->connection = c->connection;
	o->mm_width = c->mm_w;
	o->mm_height = c->mm_h;
	o->subpixel = 1u;   /* DRM_MODE_SUBPIXEL_UNKNOWN */
	o->encoder_id = KMS_ID_ENCODER(0);
	o->nmodes = 1u;
	o->nprops = obj_props(id, NULL, 0u);
	o->fw_display_id = c->fw_display_id;
	if ((max >= 1u) && (msg->o.data != NULL) && (msg->o.size >= sizeof(kms_modeinfo_t))) {
		memcpy(msg->o.data, &c->mode, sizeof(kms_modeinfo_t));
	}
	return 0;
}


static int op_get_property(uint32_t id, kms_property_t *o, msg_t *msg, uint32_t max)
{
	const propdesc_t *d = prop_desc(id);

	if (d == NULL) {
		return -ENOENT;
	}
	memset(o, 0, sizeof(*o));
	snprintf(o->name, sizeof(o->name), "%s", d->name);
	o->flags = d->flags;
	if ((d->flags & (KMS_PROP_FLAG_ENUM | KMS_PROP_FLAG_BITMASK)) != 0u) {
		o->nenums = d->nen;
		o->nvalues = d->nen;
		if ((msg->o.data != NULL) && (max != 0u)) {
			uint32_t n = (d->nen < max) ? d->nen : max;
			if (msg->o.size >= n * sizeof(kms_prop_enum_t)) {
				memcpy(msg->o.data, d->en, n * sizeof(kms_prop_enum_t));
			}
		}
	}
	else if ((d->flags & (KMS_PROP_FLAG_RANGE | KMS_PROP_FLAG_SIGNED | KMS_PROP_FLAG_OBJECT)) != 0u) {
		uint64_t v[2] = { d->min, d->max };
		o->nvalues = ((d->flags & KMS_PROP_FLAG_OBJECT) != 0u) ? 1u : 2u;
		if ((msg->o.data != NULL) && (max != 0u) && (msg->o.size >= o->nvalues * 8u)) {
			memcpy(msg->o.data, v, o->nvalues * 8u);
		}
	}
	return 0;
}


static int op_page_flip(uint32_t client, const kms_page_flip_req_t *rq, kms_flip_resp_t *out)
{
	kms_crtc_state_t *c = crtc_by_id(rq->crtc_id);
	kms_atomic_plane_t st;
	kms_fb_t *fb;

	if (c == NULL) {
		return -ENOENT;
	}
	if ((rq->flags & KMS_PAGE_FLIP_ASYNC) != 0u) {
		return -EINVAL;   /* DRM_CAP_ASYNC_PAGE_FLIP = 0 in Stage A */
	}
	fb = kms_fb_lookup(rq->fb_id);
	if (fb == NULL) {
		return -ENOENT;
	}
	if (c->cur[0].fb_id != 0u) {
		st = c->cur[0];
		st.fb_id = rq->fb_id;
		st.src_w = fb->w << 16;
		st.src_h = fb->h << 16;
	}
	else {
		state_fullscreen(c, fb, 0u, &st);
	}
	st.in_fence = rq->in_fence;
	return commit(client, c, &st, 1u, rq->flags & KMS_PAGE_FLIP_EVENT, rq->user_data, out);
}


static int op_set_crtc(uint32_t client, const kms_set_crtc_req_t *rq)
{
	kms_crtc_state_t *c = crtc_by_id(rq->crtc_id);
	kms_atomic_plane_t sts[KMS_PLANES_PER_CRTC];
	kms_fb_t *fb;
	uint32_t p, n = 0u;

	if (c == NULL) {
		return -ENOENT;
	}
	if (rq->fb_id == 0u) {
		/* disable: every plane off, the console shows */
		for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
			if ((c->plane_mask & (1u << p)) != 0u) {
				memset(&sts[n], 0, sizeof(sts[n]));
				sts[n].plane_id = KMS_ID_PLANE((uint32_t)c->idx, p);
				n++;
			}
		}
		return commit(client, c, sts, n, 0u, 0u, NULL);
	}
	if ((rq->x != 0) || (rq->y != 0) || ((rq->mode_hdisplay != 0u) && ((rq->mode_hdisplay != c->mode.hdisplay) ||
			(rq->mode_vdisplay != c->mode.vdisplay)))) {
		return -EINVAL;   /* Stage A cannot change the firmware's mode */
	}
	fb = kms_fb_lookup(rq->fb_id);
	if (fb == NULL) {
		return -ENOENT;
	}
	state_fullscreen(c, fb, 0u, &sts[0]);
	return commit(client, c, sts, 1u, 0u, 0u, NULL);
}


static int op_atomic(uint32_t client, const kms_atomic_req_t *rq, const msg_t *msg, kms_flip_resp_t *out)
{
	kms_crtc_state_t *c = crtc_by_id(rq->crtc_id);
	kms_atomic_plane_t sts[KMS_PLANES_PER_CRTC];
	uint32_t n = rq->nplanes, p;

	if (c == NULL) {
		return -ENOENT;
	}
	if (rq->mode_blob != 0u) {
		kms_srvblob_t *b = kms_blob_get(rq->mode_blob);
		const kms_modeinfo_t *mi = (b != NULL) ? (const kms_modeinfo_t *)b->data : NULL;
		if ((b == NULL) || (b->len < sizeof(kms_modeinfo_t)) || (mi->hdisplay != c->mode.hdisplay) ||
				(mi->vdisplay != c->mode.vdisplay)) {
			return -EINVAL;   /* Stage A: only the current mode */
		}
	}
	if (rq->active == 0u) {
		n = 0u;
		for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
			if ((c->plane_mask & (1u << p)) != 0u) {
				memset(&sts[n], 0, sizeof(sts[n]));
				sts[n].plane_id = KMS_ID_PLANE((uint32_t)c->idx, p);
				n++;
			}
		}
	}
	else {
		if ((n == 0u) || (n > KMS_ATOMIC_MAX_PLANES) || (msg->i.data == NULL) ||
				(msg->i.size < n * sizeof(kms_atomic_plane_t))) {
			return -EINVAL;
		}
		memcpy(sts, msg->i.data, n * sizeof(kms_atomic_plane_t));
	}
	return commit(client, c, sts, n, rq->flags, rq->user_data, out);
}


static int op_vblank(uint32_t client, const kms_req_t *rq, msg_t *msg, msg_rid_t rid, kms_resp_t *r)
{
	kms_crtc_state_t *c = crtc_by_id(rq->u.vblank.crtc_id);
	uint64_t target;
	uint32_t i;

	if (c == NULL) {
		return -ENOENT;
	}
	if (rq->op == KMS_OP_CRTC_GET_SEQUENCE) {
		r->u.vblank.sequence = c->seq;
		r->u.vblank.time_ns = kms_cnt_to_mono_ns(c->last_vbl_cnt);
		return 0;
	}
	if (rq->op == KMS_OP_CRTC_QUEUE_SEQUENCE) {
		/* flags: DRM_CRTC_SEQUENCE_RELATIVE 1, NEXT_ON_MISS 2 */
		target = ((rq->u.vblank.type & 1u) != 0u) ? c->seq + rq->u.vblank.sequence : rq->u.vblank.sequence;
		if (((rq->u.vblank.type & 2u) != 0u) && (target <= c->seq)) {
			target = c->seq + 1u;
		}
	}
	else {
		target = ((rq->u.vblank.type & KMS_VBL_ABSOLUTE) != 0u) ? rq->u.vblank.sequence : c->seq + rq->u.vblank.sequence;
	}
	if ((rq->op == KMS_OP_CRTC_QUEUE_SEQUENCE) || ((rq->u.vblank.type & KMS_VBL_EVENT) != 0u)) {
		for (i = 0u; i < KMS_MAX_VBL_EVENTS; i++) {
			kms_vblev_t *e = &srv.vblev[i];
			if (!e->used) {
				e->used = 1;
				e->client = client;
				e->crtc = KMS_ID_CRTC((uint32_t)c->idx);
				e->type = (rq->op == KMS_OP_CRTC_QUEUE_SEQUENCE) ? KMS_DRM_EVENT_CRTC_SEQUENCE : KMS_DRM_EVENT_VBLANK;
				e->target_seq = target;
				e->user_data = rq->u.vblank.user_data;
				r->u.vblank.sequence = target;
				r->u.vblank.time_ns = kms_cnt_to_mono_ns(c->last_vbl_cnt);
				return 0;
			}
		}
		return -ENOSPC;
	}
	if (target <= c->seq) {
		r->u.vblank.sequence = c->seq;
		r->u.vblank.time_ns = kms_cnt_to_mono_ns(c->last_vbl_cnt);
		return 0;
	}
	/* Blocking form: parked raw-only (no payload window), answered at the vblank. */
	if (park(KMS_PARK_VBLANK, client, KMS_ID_CRTC((uint32_t)c->idx), target, msg, rid) != 0) {
		return -ENOSPC;
	}
	return 1;   /* parked */
}


/* A raw request. Returns 1 to respond now, 0 when parked. Lock held. */
static int handle_raw(msg_t *msg, msg_rid_t rid, kms_parked_t *answer, uint32_t *nans)
{
	kms_req_t rq;
	kms_resp_t *r = (kms_resp_t *)msg->o.raw;
	kms_client_t *cl;
	const kms_crtc_state_t *c = &srv.crtc[0];
	uint32_t i, n, max;
	int rc = -ENOSYS;

	memcpy(&rq, msg->i.raw, sizeof(rq));
	memset(r, 0, sizeof(*r));
	r->op = rq.op;
	msg->o.err = EOK;

	cl = client_get(msg->oid.id);
	if (cl == NULL) {
		r->err = -EBADF;
		return 1;
	}
	if ((msg->pid != cl->pid) && (m.pid_notes++ == 0u)) {
		KMS_LOG("srv note: request pid %d != client pid %d (logged once, not enforced in Stage A)", msg->pid, cl->pid);
	}
	max = rq.u.obj.max;

	switch (rq.op) {
		case KMS_OP_GET_CAP:
			rc = get_cap(rq.u.cap.cap, &r->u.cap.value);
			break;

		case KMS_OP_SET_CLIENT_CAP:
			if ((rq.u.cap.cap == KMS_CLIENT_CAP_UNIVERSAL_PLANES) || (rq.u.cap.cap == KMS_CLIENT_CAP_ATOMIC) ||
					(rq.u.cap.cap == KMS_CLIENT_CAP_ASPECT_RATIO)) {
				if (rq.u.cap.value != 0u) {
					cl->caps |= 1u << rq.u.cap.cap;
				}
				else {
					cl->caps &= ~(1u << rq.u.cap.cap);
				}
				rc = 0;
			}
			else {
				rc = (rq.u.cap.value == 0u) ? 0 : -EINVAL;
			}
			break;

		case KMS_OP_SET_MASTER:
		case KMS_OP_DROP_MASTER:
		case KMS_OP_AUTH_MAGIC:
			rc = 0;   /* one seat, one user: accepted as no-ops (SDL2 KMSDRM calls them) */
			break;

		case KMS_OP_GET_RESOURCES:
			r->u.res.min_w = 1u;
			r->u.res.min_h = 1u;
			r->u.res.max_w = 4096u;
			r->u.res.max_h = 4096u;
			r->u.res.ncrtc = (uint8_t)srv.ncrtc;
			r->u.res.nconn = (uint8_t)srv.ncrtc;
			r->u.res.nenc = (uint8_t)srv.ncrtc;
			for (i = 0u; i < srv.ncrtc; i++) {
				r->u.res.crtc[i] = KMS_ID_CRTC(i);
				r->u.res.conn[i] = KMS_ID_CONNECTOR(i);
				r->u.res.enc[i] = KMS_ID_ENCODER(i);
			}
			for (i = 0u, n = 0u; i < KMS_MAX_FBS; i++) {
				if (srv.fbs[i].used && srv.fbs[i].user_ref && (srv.fbs[i].owner == cl->id)) {
					if ((msg->o.data != NULL) && (n < max) && ((n + 1u) * 4u <= msg->o.size)) {
						((uint32_t *)msg->o.data)[n] = srv.fbs[i].id;
					}
					n++;
				}
			}
			r->u.res.nfb = n;
			rc = 0;
			break;

		case KMS_OP_GET_CONNECTOR:
			rc = op_get_connector(rq.u.obj.id, &r->u.conn, msg, max);
			break;

		case KMS_OP_GET_ENCODER:
			if (rq.u.obj.id != KMS_ID_ENCODER(0)) {
				rc = -ENOENT;
				break;
			}
			r->u.enc.type = KMS_ENCODER_TMDS;
			r->u.enc.crtc_id = KMS_ID_CRTC(0);
			r->u.enc.possible_crtcs = 1u;
			r->u.enc.possible_clones = 0u;
			rc = 0;
			break;

		case KMS_OP_GET_CRTC:
			if (rq.u.obj.id != KMS_ID_CRTC(0)) {
				rc = -ENOENT;
				break;
			}
			r->u.crtc.fb_id = c->cur[0].fb_id;
			r->u.crtc.mode_valid = 1u;
			r->u.crtc.hdisplay = c->mode.hdisplay;
			r->u.crtc.vdisplay = c->mode.vdisplay;
			r->u.crtc.vrefresh = c->mode.vrefresh;
			r->u.crtc.sequence = c->seq;
			r->u.crtc.pending_fb = (c->pend != KMS_PEND_NONE) ? c->pst[0].fb_id : 0u;
			if ((msg->o.data != NULL) && (msg->o.size >= sizeof(kms_modeinfo_t))) {
				memcpy(msg->o.data, &c->mode, sizeof(kms_modeinfo_t));
			}
			rc = 0;
			break;

		case KMS_OP_SET_CRTC:
			rc = op_set_crtc(cl->id, &rq.u.set_crtc);
			break;

		case KMS_OP_GET_PLANE_RESOURCES:
			for (i = 0u, n = 0u; (i < KMS_PLANES_PER_CRTC) && (n < 13u); i++) {
				if ((c->plane_mask & (1u << i)) == 0u) {
					continue;
				}
				/* DRM: without UNIVERSAL_PLANES only overlays are listed */
				if (((cl->caps & (1u << KMS_CLIENT_CAP_UNIVERSAL_PLANES)) == 0u) &&
						(plane_type(i) != KMS_PLANE_TYPE_OVERLAY)) {
					continue;
				}
				r->u.plane_res.plane[n++] = KMS_ID_PLANE(0u, i);
			}
			r->u.plane_res.nplanes = n;
			rc = 0;
			break;

		case KMS_OP_GET_PLANE:
			rc = plane_idx(c, rq.u.obj.id, &i);
			if (rc != 0) {
				rc = -ENOENT;
				break;
			}
			r->u.plane.crtc_id = c->cur[i].crtc_id;
			r->u.plane.fb_id = c->cur[i].fb_id;
			r->u.plane.possible_crtcs = 1u;
			r->u.plane.type = plane_type(i);
			r->u.plane.nformats = srv.be->formats(i, r->u.plane.formats, KMS_PLANE_MAX_FMTS);
			break;

		case KMS_OP_GET_PROPERTIES:
			n = obj_props(rq.u.obj.id, NULL, 0u);
			if (n == 0u) {
				rc = -ENOENT;
				break;
			}
			if ((msg->o.data != NULL) && (max != 0u)) {
				uint32_t fit = msg->o.size / sizeof(kms_prop_value_t);
				(void)obj_props(rq.u.obj.id, msg->o.data, (max < fit) ? max : fit);
			}
			r->u.count = n;
			rc = 0;
			break;

		case KMS_OP_GET_PROPERTY:
			rc = op_get_property(rq.u.obj.id, &r->u.prop, msg, max);
			break;

		case KMS_OP_GET_BLOB: {
			kms_srvblob_t *b = kms_blob_get(rq.u.blob.id);
			if (b == NULL) {
				rc = -ENOENT;
				break;
			}
			r->u.blob.id = b->id;
			r->u.blob.length = b->len;
			if (msg->o.data != NULL) {
				memcpy(msg->o.data, b->data, (b->len < msg->o.size) ? b->len : msg->o.size);
			}
			rc = 0;
			break;
		}

		case KMS_OP_CREATE_BLOB:
			if ((msg->i.data == NULL) || (msg->i.size == 0u) || (msg->i.size > KMS_BLOB_MAX_LEN)) {
				rc = -EINVAL;
				break;
			}
			r->u.blob.id = kms_blob_create(cl->id, msg->i.data, (uint32_t)msg->i.size);
			r->u.blob.length = (uint32_t)msg->i.size;
			rc = (r->u.blob.id != 0u) ? 0 : -ENOSPC;
			break;

		case KMS_OP_DESTROY_BLOB:
			rc = kms_blob_destroy(cl->id, rq.u.blob.id);
			break;

		case KMS_OP_CREATE_DUMB:
			rc = kms_bo_create(cl->id, &rq.u.create_dumb, &r->u.dumb);
			break;

		case KMS_OP_MAP_DUMB:
			rc = kms_bo_map(cl->id, rq.u.handle.handle, &r->u.dumb);
			break;

		case KMS_OP_PRIME_EXPORT: {
			kms_bo_t *b = kms_bo_get(cl->id, rq.u.handle.handle);
			if (b == NULL) {
				rc = -ENOENT;
				break;
			}
			b->prime = 1;
			rc = kms_bo_map(cl->id, rq.u.handle.handle, &r->u.dumb);
			break;
		}

		case KMS_OP_DESTROY_DUMB:
			rc = kms_bo_destroy(cl->id, rq.u.handle.handle);
			break;

		case KMS_OP_ADDFB2:
			rc = kms_fb_add(cl->id, &rq.u.addfb2, &r->u.fb.fb_id);
			break;

		case KMS_OP_RMFB: {
			kms_fb_t *fb = kms_fb_lookup(rq.u.fb.fb_id);
			uint32_t p;
			if ((fb == NULL) || (fb->owner != cl->id) || !fb->user_ref) {
				rc = -ENOENT;
				break;
			}
			for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
				if ((srv.crtc[0].pend != KMS_PEND_NONE) && ((srv.crtc[0].pmask & (1u << p)) != 0u) &&
						(srv.crtc[0].pst[p].fb_id == fb->id)) {
					break;
				}
			}
			if (p != KMS_PLANES_PER_CRTC) {
				rc = -EBUSY;   /* a flip to it is in flight */
				break;
			}
			(void)planes_off(&srv.crtc[0], cl->id, fb->id);   /* DRM: RMFB of a shown fb disables the plane */
			rc = kms_fb_rm(cl->id, rq.u.fb.fb_id);
			break;
		}

		case KMS_OP_PAGE_FLIP:
			rc = op_page_flip(cl->id, &rq.u.flip, &r->u.flip);
			break;

		case KMS_OP_ATOMIC:
			rc = op_atomic(cl->id, &rq.u.atomic, msg, &r->u.flip);
			break;

		case KMS_OP_WAIT_VBLANK:
		case KMS_OP_CRTC_GET_SEQUENCE:
		case KMS_OP_CRTC_QUEUE_SEQUENCE:
			rc = op_vblank(cl->id, &rq, msg, rid, r);
			if (rc == 1) {
				return 0;
			}
			break;

		case KMS_OP_DBG_BO_CHECKSUM:
			rc = kms_bo_checksum(cl->id, &rq.u.checksum, &r->u.checksum);
			break;

		case KMS_OP_DBG_STATS:
			srv.st.pool_free_kib = 0u;
			if (srv.pool_ok) {
				size_t used = 0u;
				for (i = 0u; i < KMS_MAX_BOS; i++) {
					if (srv.bos[i].used && (srv.bos[i].kind == KMS_BOK_POOL)) {
						used += srv.bos[i].size;
					}
				}
				srv.st.pool_free_kib = (uint32_t)((srv.pool_size - used) / 1024u);
			}
			r->u.stats = srv.st;
			rc = 0;
			break;

		case KMS_OP_DBG_QUIT:
			for (i = 0u; i < KMS_MAX_PARKED; i++) {
				if (srv.parked[i].used) {
					if (srv.parked[i].kind == KMS_PARK_READ) {
						srv.parked[i].msg.o.err = -ENODEV;
					}
					else {
						set_vblank_reply(&srv.parked[i], -ENODEV, c->seq, c->last_vbl_cnt);
					}
					claim(i, answer, nans);
				}
			}
			srv.quit = 1;
			rc = 0;
			break;

		default:
			rc = -EINVAL;
			break;
	}
	r->err = rc;
	return 1;
}


/* KMS_OP_PRIME_IMPORT (G7). Not in handle_raw: mapping the exporter's buffer is IPC
 * to another server (open, lseek, page faults), so it runs without srv.lock - the
 * vblank thread must never wait on another server. Lock, look up; unlock, map;
 * lock, recheck the client and install. */
static void op_prime_import(msg_t *msg)
{
	kms_req_t rq;
	kms_resp_t *r = (kms_resp_t *)msg->o.raw;
	kms_import_map_t im;
	kms_client_t *cl;
	uint32_t client = (uint32_t)msg->oid.id;
	int rc, pid = -1;

	memcpy(&rq, msg->i.raw, sizeof(rq));
	memset(r, 0, sizeof(*r));
	r->op = rq.op;
	msg->o.err = EOK;

	(void)mutexLock(srv.lock);
	cl = client_get(msg->oid.id);
	if (cl != NULL) {
		pid = cl->pid;
		rc = kms_import_lookup(client, &rq.u.prime_import, &r->u.dumb);
	}
	else {
		rc = -EBADF;
	}
	(void)mutexUnlock(srv.lock);

	if (rc == 1) {
		rc = kms_import_map(&rq.u.prime_import, &im);
		if (rc != 0) {
			KMS_LOG("import FAIL client=%u ns=v3dbuf id=%llu port=%u size=%llu rc=%d why=map", client,
				(unsigned long long)rq.u.prime_import.id, rq.u.prime_import.port,
				(unsigned long long)rq.u.prime_import.size, rc);
		}
		else {
			(void)mutexLock(srv.lock);
			cl = client_get(msg->oid.id);
			if ((cl != NULL) && (cl->pid == pid)) {
				rc = kms_import_install(client, &rq.u.prime_import, &im, &r->u.dumb);
				(void)mutexUnlock(srv.lock);
			}
			else {
				(void)mutexUnlock(srv.lock);
				kms_import_unmap(&im);   /* the client closed while we mapped */
				rc = -EBADF;
			}
		}
	}
	r->err = rc;
}


/* An ioctl()-packed mtDevCtl: only HELLO. */
static void handle_ioctl(msg_t *msg)
{
	unsigned long request;
	id_t id;
	const void *in;
	kms_hello_t h;
	kms_client_t *cl;

	in = ioctl_unpack(msg, &request, &id);
	if ((request != KMS_IOC_HELLO) || (in == NULL)) {
		ioctl_setResponse(msg, request, -ENOTTY, NULL);
		return;
	}
	memcpy(&h, in, sizeof(h));
	(void)mutexLock(srv.lock);
	cl = client_get(id);
	/* Every protocol since M2 Stage A: a proto-1 client (the staged Weston, kmstest,
	 * drmprobe builds before G7) sends nothing a proto-2 server does not serve. */
	if ((cl == NULL) || (h.proto < KMS_PROTO_BASE) || (h.proto > KMS_PROTO_VERSION)) {
		(void)mutexUnlock(srv.lock);
		ioctl_setResponse(msg, request, (cl == NULL) ? -EBADF : -EPROTO, NULL);
		return;
	}
	memset(&h, 0, sizeof(h));
	h.proto = KMS_PROTO_VERSION;
	h.client_id = cl->id;
	h.server_pid = (uint32_t)getpid();
	h.backend = (uint32_t)srv.be->id;
	h.vblank_src = (uint32_t)srv.vbl_src;
	h.ncrtc = srv.ncrtc;
	h.buf_port = srv.buf_port;
	h.refresh_mhz = srv.crtc[0].refresh_mhz;
	h.width = (srv.be->id == KMS_BACKEND_PAN) ? srv.fb_w : srv.crtc[0].mode.hdisplay;
	h.height = (srv.be->id == KMS_BACKEND_PAN) ? srv.fb_h : srv.crtc[0].mode.vdisplay;
	(void)mutexUnlock(srv.lock);
	ioctl_setResponse(msg, request, 0, &h);
}


/* Names outlive their server: neither devfs nodes (create_dev) nor kernel port
 * names (portRegister) are removed when a port dies (proc/name.c keeps the
 * dcache entry; E5 saw a second ipcprobe fail to register). So a clean exit
 * deregisters both, and start-up reclaims a name whose owner no longer answers. */
static void names_release(void)
{
	if (m.dri_name) {
		(void)destroy_dev("/dev/" KMS_DRI_NAME);
		m.dri_name = 0;
	}
	(void)destroy_dev("/dev/" KMS_DEV_NAME);
	(void)portUnregister(KMS_BUF_NS);
}


/* Does the server behind `path` still answer? [inferred: a dead port makes
 * msgSend fail at once; a recycled port id owned by another server would answer
 * and be taken as live - the safe direction] */
static int name_alive(const char *path)
{
	oid_t oid;
	msg_t msg;

	if (lookup(path, NULL, &oid) < 0) {
		return 0;
	}
	memset(&msg, 0, sizeof(msg));
	msg.type = mtGetAttr;
	msg.oid = oid;
	msg.i.attr.type = atMode;
	return (msgSend(oid.port, &msg) == EOK) ? 1 : 0;
}


static int claim_names(void)
{
	oid_t dev;
	int rc;

	dev.port = srv.port;
	dev.id = 0;
	rc = create_dev(&dev, KMS_DEV_NAME);
	if ((rc < 0) && !name_alive("/dev/" KMS_DEV_NAME)) {
		KMS_LOG("srv note: /dev/%s left by a dead server (rc=%d) - reclaiming", KMS_DEV_NAME, rc);
		(void)destroy_dev("/dev/" KMS_DEV_NAME);
		rc = create_dev(&dev, KMS_DEV_NAME);
	}
	if (rc < 0) {
		KMS_LOG("srv FAIL could not create /dev/%s rc=%d (another rpi4-kms running?)", KMS_DEV_NAME, rc);
		return 2;
	}
	dev.port = srv.buf_port;
	dev.id = 0;
	rc = portRegister(srv.buf_port, KMS_BUF_NS, &dev);
	if ((rc < 0) && !name_alive(KMS_BUF_NS)) {
		KMS_LOG("srv note: %s left by a dead server (rc=%d) - reclaiming", KMS_BUF_NS, rc);
		(void)portUnregister(KMS_BUF_NS);
		rc = portRegister(srv.buf_port, KMS_BUF_NS, &dev);
	}
	if (rc < 0) {
		KMS_LOG("srv FAIL portRegister %s rc=%d", KMS_BUF_NS, rc);
		(void)destroy_dev("/dev/" KMS_DEV_NAME);
		return 2;
	}

	/* G10 (M3 part 2): the DRM primary node name, an alias of /dev/kms on the same
	 * port (one node: one dev_t). Best effort - /dev/kms above is the single-owner
	 * guard, so a live /dev/dri/card0 here is someone else's and is left alone, and
	 * a stale one (dead server) is reclaimed. Old clients never need it. */
	dev.port = srv.port;
	dev.id = 0;
	rc = create_dev(&dev, KMS_DRI_NAME);
	if ((rc < 0) && !name_alive("/dev/" KMS_DRI_NAME)) {
		(void)destroy_dev("/dev/" KMS_DRI_NAME);
		rc = create_dev(&dev, KMS_DRI_NAME);
	}
	m.dri_name = (rc >= 0) ? 1 : 0;
	KMS_LOG("srv dri name=/dev/%s rc=%d registered=%d (G10; alias of /dev/%s)", KMS_DRI_NAME, rc, m.dri_name,
		KMS_DEV_NAME);
#ifdef KMS_POLL_NOTIFY
	/* A kernel without pollNotify answers -EINVAL; with it, notifying our own oid
	 * with no poller watching is a no-op returning 0. */
	dev.port = srv.port;
	dev.id = 0;
	rc = pollNotify(&dev);
	m.poll_notify = (rc == 0) ? 1 : 0;
	KMS_LOG("srv poll_notify=%d rc=%d (1: poll() on a card fd wakes at the event, not on the kernel's 20 ms re-poll)",
		m.poll_notify, rc);
#endif
	return 0;
}


static void restore_display(void)
{
	uint32_t i;

	for (i = 0u; i < srv.ncrtc; i++) {
		srv.be->restore(&srv.crtc[i]);
	}
	if (srv.fb_blanked) {
		(void)kms_fw_blank(0);
		srv.fb_blanked = 0;
	}
	if (srv.console_disabled) {
		(void)kms_fw_console(1);
		srv.console_disabled = 0;
	}
}


static void dispatch_loop(void)
{
	static kms_parked_t answers[KMS_MAX_PARKED];
	msg_t msg;
	msg_rid_t rid;
	uint32_t nans;
	int err, respond, id, quitting;

	for (;;) {
		err = msgRecv(srv.port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			break;
		}
		respond = 1;
		quitting = 0;
		nans = 0u;

		switch (msg.type) {
			case mtOpen:
				(void)mutexLock(srv.lock);
				id = client_open(msg.pid);
				(void)mutexUnlock(srv.lock);
				msg.o.err = id;   /* > 0: becomes the descriptor's oid.id */
				if (srv.verbose && (id > 0)) {
					KMS_LOG("srv client %d opened by pid %d", id, msg.pid);
				}
				break;

			case mtClose:
				(void)mutexLock(srv.lock);
				client_close(msg.oid.id, answers, &nans);
				(void)mutexUnlock(srv.lock);
				msg.o.err = EOK;
				break;

			case mtRead: {
				kms_client_t *cl;
				(void)mutexLock(srv.lock);
				cl = client_get(msg.oid.id);
				if (cl == NULL) {
					msg.o.err = -EBADF;
				}
				else if ((msg.o.data == NULL) || (msg.o.size < 32u)) {
					msg.o.err = -EINVAL;
				}
				else if (cl->evhead != cl->evtail) {
					msg.o.err = ev_fill(cl, msg.o.data, msg.o.size);
					read_dump("immediate", &msg, msg.o.err);
				}
				else if ((msg.i.io.mode & O_NONBLOCK) != 0u) {
					msg.o.err = -EAGAIN;
				}
				else if (park(KMS_PARK_READ, cl->id, 0u, 0u, &msg, rid) == 0) {
					respond = 0;   /* answered by the vblank thread (event, deadline, close, quit) */
				}
				else {
					msg.o.err = -EAGAIN;
				}
				(void)mutexUnlock(srv.lock);
				break;
			}

			case mtWrite:
				msg.o.err = -EINVAL;
				break;

			case mtDevCtl:
				if (((const uint32_t *)msg.i.raw)[0] != KMS_MAGIC) {
					handle_ioctl(&msg);
					break;
				}
				if (((const kms_req_t *)msg.i.raw)->op == KMS_OP_PRIME_IMPORT) {
					op_prime_import(&msg);   /* takes srv.lock itself, around IPC it does unlocked */
					break;
				}
				(void)mutexLock(srv.lock);
				respond = handle_raw(&msg, rid, answers, &nans);
				quitting = srv.quit;
				(void)mutexUnlock(srv.lock);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atMode) {
					msg.o.attr.val = S_IFCHR | 0666;
					msg.o.err = EOK;
				}
				else if (msg.i.attr.type == atPollStatus) {
					/* A snapshot; block_ms (bits 16+) is ignored. Built with KMS_POLL_NOTIFY
					 * on a kernel that has pollNotify, ev_push() wakes a sleeping poll()
					 * at once; otherwise poll() on this fd sees an event up to the
					 * kernel's 20 ms POLL_INTERVAL late (E5). */
					kms_client_t *cl;
					(void)mutexLock(srv.lock);
					cl = client_get(msg.oid.id);
					msg.o.attr.val = ((cl != NULL) && (cl->evhead != cl->evtail)) ? POLLIN : 0;
					(void)mutexUnlock(srv.lock);
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;

			case mtGetAttrAll:
				/* G2 (M3 part 2): fstat() on a card descriptor. Mesa's gbm_create_device()
				 * refuses a descriptor whose fstat fails or is not S_ISCHR. st_rdev is the
				 * descriptor's port (kernel posix_fstat), so /dev/kms and /dev/dri/card0 -
				 * one node, two names - share one dev_t. */
				msg.o.err = kms_attr_all(&msg, S_IFCHR | 0666, 0u, srv.port);
				if ((msg.o.err == 0) && (m.attr_notes++ == 0u)) {
					KMS_LOG("srv fstat answered (mtGetAttrAll, G2) client=%u pid=%d", (unsigned)msg.oid.id, msg.pid);
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		/* G7: a request that dropped an import's last reference (DESTROY_DUMB, RMFB,
		 * mtClose) releases the exporter's reference before it is answered, so the
		 * client that asked sees the buffer gone. */
		if (__atomic_load_n(&srv.nreap, __ATOMIC_RELAXED) != 0u) {
			kms_reap();
		}
		kms_answer(answers, nans);
		if (respond) {
			(void)msgRespond(srv.port, &msg, rid);
		}
		if (quitting) {
			(void)mutexLock(srv.lock);
			restore_display();
			kms_pool_fini();
			(void)mutexUnlock(srv.lock);
			kms_vblank_fini();
			names_release();
			KMS_LOG("srv exit flips_completed=%u vblanks=%u restored=1", srv.st.flips_completed, srv.st.vblanks);
			usleep(50000);   /* let the vblank thread's in-flight responds finish */
			exit(0);
		}
	}
}


/* ========================================================================= */
/* Start-up                                                                   */
/* ========================================================================= */

static void on_signal(int sig)
{
	(void)sig;
	/* Best effort: put the console back before dying (mailbox calls from a
	 * handler are not async-signal-safe; this is the last thing the process does). */
	restore_display();
	names_release();
	_exit(1);
}


static void usage(const char *prog)
{
	printf("usage: %s [-f] [-b plane|pan] [-o overlays] [-p pool_mib] [-m <max_end>] [-c] "
		"[-V irq|hvs|fwvsync|timer] [-g gate_us] [-K] [-L guard_us] [-G] [-B] [-C] [-F] [-v]\n"
		"       %s -R   (restore the display and exit)\n",
		prog, prog);
}


static int restore_only(void)
{
	uint32_t p;
	int rc;

	srv.tty_fd = -1;
	if (kms_fw_init() != 0) {
		return 1;
	}
	for (p = 0u; p < KMS_PLANES_PER_CRTC; p++) {
		(void)kms_backend_plane.apply(&srv.crtc[0], p, NULL, NULL, NULL, NULL);   /* unset: harmless if never set */
	}
	rc = kms_fw_pan(0u, NULL);
	KMS_LOG("restore done planes_unset=%u pan0_rc=%d unblank_rc=%d console_rc=%d", KMS_PLANES_PER_CRTC, rc,
		kms_fw_blank(0), kms_fw_console(1));
	return 0;
}


int main(int argc, char **argv)
{
	struct timespec ts;
	uint64_t f;
	int c, i, rc, readyfd = -1, restore = 0, pool_set = 0;
	const char *bname = "plane";

	setvbuf(stdout, NULL, _IOLBF, 0);
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
	kms_cnt_hz = (f != 0u) ? f : 54000000u;

	srv.pool_max_end = KMS_GIB;
	srv.gate_us = 500u;
	srv.latch_guard_us = 2000u;
	srv.want_vbl = KMS_VBL_NONE;
	srv.tty_fd = -1;
	srv.next_handle = 1u;
	srv.next_fb = KMS_ID_FB_BASE;
	srv.next_blob = KMS_ID_BLOB_BASE;

	while ((c = getopt(argc, argv, "fb:o:p:m:cV:g:KL:GBCFvRh")) != -1) {
		switch (c) {
			case 'f': srv.foreground = 1; break;
			case 'b': bname = optarg; break;
			case 'o': kms_plane_overlays = (uint32_t)strtoul(optarg, NULL, 0); break;
			case 'p': srv.pool_mib = (uint32_t)strtoul(optarg, NULL, 0); pool_set = 1; break;
			case 'm':
				srv.pool_max_end = strtoull(optarg, NULL, 0);
				if ((srv.pool_max_end == 0u) || (srv.pool_max_end > KMS_GIB)) {
					srv.pool_max_end = KMS_GIB;   /* the firmware scans the low 1 GiB only (E6) */
				}
				break;
			case 'c': srv.bus_c0 = 1; break;
			case 'V':
				srv.want_vbl = (strcmp(optarg, "irq") == 0) ? KMS_VBL_IRQ : (strcmp(optarg, "hvs") == 0) ? KMS_VBL_HVS :
					(strcmp(optarg, "fwvsync") == 0) ? KMS_VBL_FWVSYNC : (strcmp(optarg, "timer") == 0) ? KMS_VBL_TIMER :
					KMS_VBL_NONE;
				break;
			case 'g': srv.gate_us = (uint32_t)strtoul(optarg, NULL, 0); break;
			case 'K': srv.no_kick = 1; break;
			case 'G': srv.connect_v3d = 1; break;
			case 'L': srv.latch_guard_us = (uint32_t)strtoul(optarg, NULL, 0); break;
			case 'B': srv.blank_fb = 1; break;
			case 'C': srv.console_off = 1; break;
			case 'F': srv.force = 1; break;
			case 'v': srv.verbose = 1; break;
			case 'R': restore = 1; break;
			default: usage(argv[0]); return 1;
		}
	}
	for (i = optind; i < argc; i++) {
		if (strcmp(argv[i], "&") != 0) {
			printf("KMS srv unexpected argument '%s'\n", argv[i]);
			usage(argv[0]);
			return 1;
		}
	}
	if (restore) {
		return restore_only();
	}
	if (strcmp(bname, "plane") == 0) {
		srv.be = &kms_backend_plane;
	}
	else if (strcmp(bname, "pan") == 0) {
		srv.be = &kms_backend_pan;
	}
	else {
		usage(argv[0]);
		return 1;
	}
	if (!pool_set) {
		srv.pool_mib = (srv.be->id == KMS_BACKEND_PLANE) ? 32u : 4u;
	}
	if (srv.gate_us < 100u) {
		srv.gate_us = 100u;
	}

	/* Detach BEFORE any port, thread, mapping or interrupt exists (none survive a
	 * fork); the parent waits for one 'R' byte written once everything is up. */
	if (!srv.foreground) {
		int pfd[2];
		pid_t pid;
		char r = 0;

		if (pipe(pfd) < 0) {
			printf("KMS srv FAIL pipe errno=%d\n", errno);
			return 1;
		}
		fflush(stdout);
		pid = fork();
		if (pid < 0) {
			printf("KMS srv FAIL fork errno=%d\n", errno);
			return 1;
		}
		if (pid > 0) {
			close(pfd[1]);
			if ((read(pfd[0], &r, 1) != 1) || (r != 'R')) {
				printf("KMS srv FAIL child did not come up (see its lines above)\n");
				return 1;
			}
			printf("KMS srv detached pid=%d\n", (int)pid);
			fflush(stdout);
			_exit(0);
		}
		close(pfd[0]);
		readyfd = pfd[1];
	}

	(void)clock_gettime(CLOCK_MONOTONIC, &ts);
	m.cnt0 = kms_cnt();
	m.mono0_ns = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;

	if ((mutexCreate(&srv.lock) != EOK) || (mutexCreate(&srv.vbl_lock) != EOK) || (condCreate(&srv.vbl_cond) != EOK)) {
		KMS_LOG("srv FAIL lock/cond create");
		return 1;
	}

	/* Claim the names FIRST: the device node is the single-owner guard. */
	if ((portCreate(&srv.port) != EOK) || (portCreate(&srv.buf_port) != EOK)) {
		KMS_LOG("srv FAIL portCreate");
		return 1;
	}
	rc = claim_names();
	if (rc != 0) {
		return rc;
	}

	rc = kms_fw_init();
	if (rc != 0) {
		return 3;
	}
	/* Exclusivity guard: an old-lane app pans the same framebuffer through the raw
	 * FIFO. Panned away from 0 at our start = someone else is flipping. */
	if ((srv.fb_yoff0 != 0u) && !srv.force) {
		KMS_LOG("srv FAIL firmware fb panned to y=%u: another display owner is running (-F overrides)", srv.fb_yoff0);
		return 4;
	}
	srv.fb_yoff0 = 0u;   /* restore target: the console slot */

	rc = kms_pool_init();
	if ((rc != 0) && (srv.be->id == KMS_BACKEND_PLANE)) {
		return 5;
	}
	for (i = 0; i < (int)srv.ncrtc; i++) {
		rc = srv.be->init(&srv.crtc[i]);
		if (rc != 0) {
			return 6;
		}
	}
	for (i = 0; i < (int)KMS_PLANES_PER_CRTC; i++) {
		if ((srv.crtc[0].plane_mask & (1u << i)) != 0u) {
			m.in_formats_blob[i] = in_formats_blob((uint32_t)i);
		}
	}
	if (srv.connect_v3d) {
		v3d_connect();
	}
	(void)kms_vblank_init();

	signal(SIGTERM, on_signal);
	signal(SIGINT, on_signal);

	if (beginthread(kms_vblank_thread, 1, m.vblank_stack, sizeof(m.vblank_stack), NULL) != 0) {
		KMS_LOG("srv FAIL vblank thread");
		return 7;
	}
	if (beginthread(kms_bufns_thread, 4, m.bufns_stack, sizeof(m.bufns_stack), NULL) != 0) {
		KMS_LOG("srv FAIL buffer namespace thread");
		return 7;
	}

	KMS_LOG("srv ready dev=/dev/%s buf=%s backend=%s planes=0x%02x vblank_src=%s mode=%ux%u refresh_mhz=%u xl=%d "
		"pool=%d pool_mib=%u slots=%u fmt=%.4s bus=%s v3d=%d console_off=%d blank_fb=%d guard_us=%u gate_us=%u kick=%d proto=%u..%u import=v3dbuf,kmsbuf",
		KMS_DEV_NAME, KMS_BUF_NS, srv.be->name, srv.crtc[0].plane_mask, kms_vbl_name(srv.vbl_src),
		srv.crtc[0].mode.hdisplay, srv.crtc[0].mode.vdisplay, srv.crtc[0].refresh_mhz, srv.xl, srv.pool_ok,
		srv.pool_mib, srv.fb_slots, (const char *)&srv.fb_format, srv.bus_c0 ? "c0" : "raw", srv.v3d_fp != NULL,
		srv.console_off, srv.blank_fb, srv.latch_guard_us, srv.gate_us, !srv.no_kick, KMS_PROTO_BASE,
		KMS_PROTO_VERSION);

	if (readyfd >= 0) {
		char r = 'R';
		(void)write(readyfd, &r, 1);
		close(readyfd);
	}
	(void)setPriority(3);
	dispatch_loop();
	return 0;
}
