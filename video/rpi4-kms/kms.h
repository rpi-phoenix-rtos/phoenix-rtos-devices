/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - internal interfaces
 *
 * NEW GPU LANE, M2 Stage A (docs/gpu-new-lane/M2-kms-server.md).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _KMS_H_
#define _KMS_H_

#include <stddef.h>
#include <stdint.h>

#include <sys/msg.h>
#include <sys/types.h>

#include "kms_proto.h"


#ifndef _PAGE_SIZE
#define _PAGE_SIZE 4096UL
#endif

#define KMS_PAGE_ROUND(x) (((x) + _PAGE_SIZE - 1UL) & ~(_PAGE_SIZE - 1UL))
#define KMS_MIB           (1024UL * 1024UL)
#define KMS_GIB           (1024ULL * 1024ULL * 1024ULL)

#define KMS_MAX_CLIENTS   16u
#define KMS_MAX_BOS       64u
#define KMS_MAX_FBS       64u
#define KMS_MAX_BLOBS     16u
#define KMS_BLOB_MAX_LEN  256u
#define KMS_EVQ_LEN       64u    /* events per open file */
#define KMS_MAX_PARKED    32u    /* parked read()s + blocking WAIT_VBLANKs */
#define KMS_MAX_VBL_EVENTS 64u   /* queued WAIT_VBLANK(EVENT) / QUEUE_SEQUENCE requests */
#define KMS_FB_SLOTS      4u     /* firmware-fb stacked buffers the pan backend can see */


/* ------------------------------------------------------------------------- */
/* Timing                                                                     */
/* ------------------------------------------------------------------------- */

static inline uint64_t kms_cnt(void)
{
	uint64_t v;
	__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v)::"memory");
	return v;
}

extern uint64_t kms_cnt_hz;

static inline uint64_t kms_cnt_us(uint64_t ticks)
{
	return (ticks * 1000000ULL) / kms_cnt_hz;
}

static inline uint64_t kms_us_cnt(uint64_t us)
{
	return (us * kms_cnt_hz) / 1000000ULL;
}

/* cntvct -> CLOCK_MONOTONIC nanoseconds (calibrated once at start). */
int64_t kms_cnt_to_mono_ns(uint64_t cnt);


/* ------------------------------------------------------------------------- */
/* Objects                                                                    */
/* ------------------------------------------------------------------------- */

typedef struct {
	int used;
	uint32_t id;
	int pid;
	uint32_t caps;               /* bit n = DRM client cap n set */
	uint8_t evq[KMS_EVQ_LEN][32];   /* every Stage A event is 32 bytes */
	uint32_t evhead, evtail;     /* free-running */
	uint32_t dropped;
} kms_client_t;

/* POOL: a window of the scan-out pool; SLOT: a firmware-fb slot (pan backend);
 * IMPORT: another server's exported buffer (PRIME_IMPORT, G7); ALIAS: another
 * client's PRIME-exported pool BO, imported as a handle of this client (it holds
 * one reference on that BO, which keeps its pages and its /kmsbuf name alive). */
enum { KMS_BOK_POOL = 0, KMS_BOK_SLOT = 1, KMS_BOK_IMPORT = 2, KMS_BOK_ALIAS = 3 };

typedef struct {
	int used;
	uint32_t handle;             /* nonzero, never reused; == export id for pool BOs */
	uint32_t owner;              /* creating client (its handle namespace) */
	int handle_open;             /* the owner's handle is still open */
	uint32_t refs;               /* handle (1 while open) + one per framebuffer */
	int kind;                    /* KMS_BOK_* */
	uint32_t slot;               /* KMS_BOK_SLOT: firmware-fb buffer index */
	uint64_t pa;
	size_t off;                  /* KMS_BOK_POOL: offset in the pool */
	size_t size;                 /* page-rounded */
	volatile uint8_t *va;        /* server's uncached view */
	uint32_t w, h, bpp, pitch;
	int exported;
	int prime;                   /* PRIME_EXPORT'ed: any process may open its /kmsbuf name */
	/* KMS_BOK_IMPORT (G7) and KMS_BOK_ALIAS */
	int imp_fd;                  /* IMPORT: the exporter's name, open: holds the exporter's reference */
	kms_memref_t imp_mem;        /* the exporter's OID memref (MAP_DUMB / PRIME_EXPORT answer it) */
	uint32_t alias_src;          /* ALIAS: index in srv.bos of the pool BO it references */
	const char *imp_why;         /* NULL = scan-out capable; else why ADDFB2 refuses it */
	int imp_shown;               /* the first commit showing it was logged */
} kms_bo_t;

/* An import's resources, released OUTSIDE srv.lock (close() is IPC to the exporter). */
typedef struct {
	int fd;
	void *va;
	size_t size;
	uint32_t handle;
	uint64_t id;
} kms_reap_t;

typedef struct {
	int used;
	uint32_t id;
	uint32_t owner;
	int user_ref;                /* 1 until RMFB (or the owner's death) */
	uint32_t refs;               /* user_ref + one per plane showing it or about to */
	uint32_t bo;                 /* index into srv.bos */
	uint32_t w, h, format, pitch, offset;
} kms_fb_t;

typedef struct {
	int used;
	uint32_t id;
	uint32_t owner;              /* 0 = server-owned (EDID, IN_FORMATS, MODE of the current mode) */
	uint32_t len;
	uint8_t data[KMS_BLOB_MAX_LEN];
} kms_srvblob_t;

enum { KMS_PEND_NONE = 0, KMS_PEND_FENCE, KMS_PEND_ARMED };

typedef struct {
	int idx;
	uint32_t fw_display_id;
	uint32_t connection;
	kms_modeinfo_t mode;
	int mode_from_fw;            /* 1 = GET_DISPLAY_TIMING answered */
	uint32_t refresh_mhz;
	uint32_t mode_blob;          /* server blob describing the current mode */
	uint32_t edid_blob;          /* 0 = none */
	uint32_t mm_w, mm_h;

	/* planes (index 0 primary, 1..6 overlay, 7 cursor, as the firmware numbers them) */
	uint32_t plane_mask;         /* planes the backend exposes */
	kms_atomic_plane_t cur[KMS_PLANES_PER_CRTC];   /* state on screen (fb_id 0 = off) */

	/* one commit in flight per CRTC */
	int pend;                    /* KMS_PEND_* */
	uint32_t pmask;              /* planes the commit touches */
	kms_atomic_plane_t pst[KMS_PLANES_PER_CRTC];
	uint32_t pclient;
	uint64_t puser;
	int pevent;
	uint64_t parmed_cnt;         /* cntvct when the last mailbox call of the commit returned */
	uint64_t pqueued_cnt;        /* cntvct when the commit was accepted */
	uint64_t pqueued_seq;        /* seq when the commit was accepted (flip latency in vblanks) */
	uint64_t ptarget_seq;        /* armed: the vblank from which the new state is scanned */

	/* vblank */
	uint64_t seq;
	uint64_t last_vbl_cnt;
} kms_crtc_state_t;


/* A request that is answered later (read() or blocking WAIT_VBLANK). Exactly one
 * owner: removed from srv.parked under srv.lock before anyone responds. */
enum { KMS_PARK_READ = 1, KMS_PARK_VBLANK };

typedef struct {
	int used;
	int kind;
	uint32_t client;
	uint32_t crtc;
	uint64_t target_seq;         /* KMS_PARK_VBLANK */
	uint64_t deadline_cnt;
	msg_t msg;
	msg_rid_t rid;
} kms_parked_t;

typedef struct {
	int used;
	uint32_t client;
	uint32_t crtc;
	uint32_t type;               /* KMS_DRM_EVENT_VBLANK or _CRTC_SEQUENCE */
	uint64_t target_seq;
	uint64_t user_data;
} kms_vblev_t;


/* ------------------------------------------------------------------------- */
/* Backends                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct {
	const char *name;
	int id;                      /* enum kms_backend_id */
	/* after the display query: set crtc->plane_mask, prepare. 0 or -errno. */
	int (*init)(kms_crtc_state_t *crtc);
	/* formats a plane of this backend scans (writes up to max, returns count) */
	uint32_t (*formats)(uint32_t plane, uint32_t *out, uint32_t max);
	/* can plane p show fb (with bo) as described by st? 0 or -EINVAL/-ERANGE */
	int (*check)(const kms_crtc_state_t *crtc, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
		const kms_bo_t *bo);
	/* hand plane p's new state to the firmware; fb == NULL disables the plane.
	 * Returns the mailbox status; *lat_us gets the call latency. */
	int (*apply)(kms_crtc_state_t *crtc, uint32_t p, const kms_atomic_plane_t *st, const kms_fb_t *fb,
		const kms_bo_t *bo, uint32_t *lat_us);
	/* put the display back the way the server found it (idempotent) */
	void (*restore)(kms_crtc_state_t *crtc);
} kms_backend_t;

extern const kms_backend_t kms_backend_pan;
extern const kms_backend_t kms_backend_plane;


/* ------------------------------------------------------------------------- */
/* Server state                                                               */
/* ------------------------------------------------------------------------- */

/* Flip-latency counters, printed as one `KMS srv flipstat` line when a client
 * that flipped closes, then reset. Server-local: kms_stats_t (the kmstest wire
 * struct) is unchanged. */
typedef struct {
	uint32_t flips;              /* completed commits */
	uint32_t vbl1, vbl2, vbl3p;  /* completed 1, 2, >= 3 vblanks after they were accepted */
	uint32_t deferred;           /* accepted with an unsignalled in-fence */
	uint32_t applied_gate;       /* deferred commits armed by the fence poll (between vblanks) */
	uint32_t applied_vblank;     /* deferred commits armed at a vblank */
	uint32_t kicks;              /* vbl_wait() calls ended by gate_kick */
	uint32_t late_target;        /* armed too late for the next latch: scanned 2 vblanks on */
	uint64_t q2a_us_sum;         /* accepted -> armed */
	uint32_t q2a_us_max;
} kms_flipstat_t;

typedef struct {
	/* options */
	int foreground;
	int verbose;
	int force;                   /* -F: start although someone else seems to own the display */
	int console_off;             /* -C: FBCONSETMODE(FBCON_DISABLED) while a client shows a plane */
	int bus_c0;                  /* -c: hand the firmware 0xC0000000|PA instead of the raw PA */
	uint32_t pool_mib;
	uint64_t pool_max_end;       /* pool must end at or below this PA */
	int want_vbl;                /* -V: forced vblank source, KMS_VBL_NONE = auto */
	uint32_t gate_us;            /* fence poll period while a commit waits for its fence */
	int no_kick;                 /* -K: A/B control, a deferred commit does not wake the vblank thread */
	int connect_v3d;             /* -G: map rpi4-v3d-async's fence page for in-fences */
	uint32_t latch_guard_us;     /* -L: a call returning later than frame - guard after a vblank misses the next one */
	int blank_fb;                /* -B: FRAMEBUFFER_BLANK the firmware fb while the primary plane shows */
	int fb_blanked;

	/* ports */
	uint32_t port;               /* /dev/kms */
	uint32_t buf_port;           /* /kmsbuf */
	handle_t lock;               /* one server mutex */

	const kms_backend_t *be;
	int xl;                      /* /dev/vcmbox large-buffer calls available */

	/* display */
	uint32_t ncrtc;
	kms_crtc_state_t crtc[KMS_MAX_CRTCS];

	/* firmware framebuffer (plo-allocated; fbcon + /dev/fb0 draw into slot 0) */
	uint64_t fb_pa;
	uint32_t fb_w, fb_h, fb_pitch, fb_virt_h, fb_yoff0, fb_pixel_order, fb_format;
	int32_t fb_layer;
	int have_fb_layer;
	uint32_t fb_slots;           /* fb_virt_h / fb_h */
	uint32_t slot_used;          /* bit i: slot i handed out as a BO (slot 0 never) */
	int console_disabled;        /* we issued FBCONSETMODE(DISABLED) */
	int tty_fd;

	/* pool */
	volatile uint8_t *pool_va;
	uint64_t pool_pa;
	size_t pool_size;
	int pool_ok;

	/* objects */
	kms_client_t clients[KMS_MAX_CLIENTS];
	kms_bo_t bos[KMS_MAX_BOS];
	kms_fb_t fbs[KMS_MAX_FBS];
	kms_srvblob_t blobs[KMS_MAX_BLOBS];
	uint32_t next_handle, next_fb, next_blob;
	kms_parked_t parked[KMS_MAX_PARKED];
	kms_vblev_t vblev[KMS_MAX_VBL_EVENTS];
	kms_reap_t reap[KMS_MAX_BOS];   /* released imports awaiting kms_reap() */
	uint32_t nreap;
	uint32_t imports_live;
	uint32_t aliases_live;

	/* vblank */
	int vbl_src;                 /* enum kms_vblank_src in use */
	handle_t vbl_lock, vbl_cond; /* the ISR signals vbl_cond */
	/* dispatch -> vblank thread: a fence-gated commit is waiting. commit() bumps
	 * it and signals vbl_cond; vbl_wait() returns early when it changes, so the
	 * fence poll (gate_us) starts at once instead of at the next vblank. */
	uint32_t gate_kick;
	int hvs_ch;

	/* render-server fence page (optional) */
	volatile const void *v3d_fp;

	/* stats */
	kms_stats_t st;
	kms_flipstat_t fst;
	uint32_t nparked;
	int quit;
} kms_srv_t;

extern kms_srv_t srv;

#define KMS_LOG(...) kms_log(__VA_ARGS__)
void kms_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));


/* kms_fw.c - firmware (mailbox via /dev/vcmbox) */
int kms_fw_init(void);
int kms_fw_prop(uint32_t tag, uint32_t valWords, const uint32_t *in, uint32_t nIn, uint32_t *out);
int kms_fw_pan(uint32_t yoff, uint32_t *got);
int kms_fw_console(int enable);
int kms_bus_addr(uint64_t pa, size_t len, uint32_t *bus);
void kms_mode_set_refresh(kms_crtc_state_t *c, uint32_t mhz);
int kms_fw_blank(int on);

/* kms_bo.c - pool, dumb BOs, framebuffers, the /kmsbuf namespace */
int kms_pool_init(void);
void kms_pool_fini(void);
int kms_bo_create(uint32_t client, const kms_create_dumb_req_t *rq, kms_dumb_resp_t *out);
int kms_bo_map(uint32_t client, uint32_t handle, kms_dumb_resp_t *out);
int kms_bo_destroy(uint32_t client, uint32_t handle);
kms_bo_t *kms_bo_get(uint32_t client, uint32_t handle);
void kms_bo_unref(uint32_t idx);
int kms_bo_checksum(uint32_t client, const kms_checksum_req_t *rq, kms_checksum_resp_t *out);
int kms_fb_add(uint32_t client, const kms_addfb2_req_t *rq, uint32_t *fb_id);
int kms_fb_rm(uint32_t client, uint32_t fb_id);
kms_fb_t *kms_fb_lookup(uint32_t fb_id);
void kms_fb_ref(kms_fb_t *fb);
void kms_fb_unref(kms_fb_t *fb);
void kms_bo_client_gone(uint32_t client);
/* PRIME_IMPORT (G7): the unlocked mapping half, then the locked install. */
typedef struct {
	int fd;
	void *va;
	size_t size;
	uint64_t pa;
	uint32_t pages;
	int contiguous;
} kms_import_map_t;
int kms_import_map(const kms_prime_import_req_t *rq, kms_import_map_t *im);   /* no lock held */
void kms_import_unmap(kms_import_map_t *im);                                   /* no lock held */
int kms_import_lookup(uint32_t client, const kms_prime_import_req_t *rq, kms_dumb_resp_t *out);   /* locked */
int kms_import_install(uint32_t client, const kms_prime_import_req_t *rq, kms_import_map_t *im,
	kms_dumb_resp_t *out);                                                     /* locked; takes over im */
void kms_import_shown(kms_bo_t *b, uint32_t fb_id);                            /* locked: log the first scan-out */
void kms_reap(void);                                                           /* no lock held */
void kms_bufns_thread(void *arg);
int kms_attr_all(msg_t *msg, uint32_t mode, uint64_t size, uint32_t port);   /* mtGetAttrAll reply (G2) */
uint32_t kms_blob_create(uint32_t owner, const void *data, uint32_t len);
kms_srvblob_t *kms_blob_get(uint32_t id);
int kms_blob_destroy(uint32_t owner, uint32_t id);

/* kms_vblank.c */
int kms_vblank_init(void);
void kms_vblank_thread(void *arg);
void kms_vblank_fini(void);
const char *kms_vbl_name(int src);

/* kms_main.c - commit + events (called by the vblank thread, lock held) */
/* `answer` collects claimed parked requests (copies); the caller responds to
 * them with kms_answer() AFTER dropping srv.lock (proc_respond reschedules). */
void kms_on_vblank(kms_crtc_state_t *c, uint64_t cnt, uint32_t nvbl, kms_parked_t *answer, uint32_t *nanswer);
void kms_expire_parked(uint64_t now, kms_parked_t *answer, uint32_t *nanswer);
int kms_try_apply(kms_crtc_state_t *c);
int kms_fence_signaled(const kms_fence_t *f);
void kms_answer(kms_parked_t *list, uint32_t n);

#endif /* _KMS_H_ */
