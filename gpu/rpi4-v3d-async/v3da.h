/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - internal types
 *
 * One process, one mutex (srv.lock) over clients, BOs, queues, syncobjs and parked
 * requests. The IRQ handler never takes it: it only ORs status bits into atomic
 * event words that the event thread drains. Design:
 * docs/gpu-new-lane/M1-async-render-server.md.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3DA_H_
#define _V3DA_H_

#include <stdint.h>
#include <sys/msg.h>
#include <sys/types.h>

#include "v3da_proto.h"
#include "v3da_lowmem.h"   /* v3da_pool_block_t, the scan-out placement policy */


#define V3DA_MAX_CLIENTS  V3DA_FENCE_NSLOTS   /* one fence-page row each */
#define V3DA_MAX_BOS      4096u
#define V3DA_MAX_HOLES    2048u
#define V3DA_MAX_SYNCOBJS 256u                /* per client */
#define V3DA_MAX_POOL     1024u               /* pooled BO blocks */
#define V3DA_MAX_CHUNKS   512u                /* binner-overflow chunks */
#define V3DA_MAX_FLIPS    4u                  /* queued (fence-gated) flips */
#define V3DA_SCANOUT_MAX  3u                  /* firmware-fb page-flip buffers */

/* Handles: (generation << 13) | (slot + 1). A handle is never reused while the
 * server lives (until a slot's generation wraps after 2^19 reuses), so a stale
 * handle fails cleanly instead of naming a newer BO - the winsys learned this
 * the hard way (a recycled handle put texture data into a control list). */
#define V3DA_HANDLE_SLOT_BITS 13u
#define V3DA_HANDLE_SLOT_MASK ((1u << V3DA_HANDLE_SLOT_BITS) - 1u)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(V3DA_MAX_CLIENTS <= 64u, "v3da_bo_t.sharers is a 64-bit client mask");
#endif


/* ------------------------------------------------------------------------- */
/* Hardware (v3da_hw.c)                                                       */
/* ------------------------------------------------------------------------- */

typedef struct {
	volatile uint32_t *hub;
	volatile uint32_t *core0;
	volatile uint32_t *pt;        /* flat MMU page table (uncached, contiguous) */
	uintptr_t pt_pa;
	uint32_t pt_entries;
	uintptr_t scratch_pa;         /* MMU illegal-access redirect page */
	uint32_t ident[7];            /* CORE0_IDENT0/1/2, HUB_UIFCFG, HUB_IDENT1/2/3 */
	uint32_t clk_rate_hz;
	uint32_t clk_meas_hz;
	unsigned clk_tries;

	/* GPU VA allocator (bump + first-fit holes; old daemon's va_alloc) */
	uint32_t next_gpuva;
	struct { uint32_t gpuva, pages; } holes[V3DA_MAX_HOLES];
	uint32_t nholes;

	/* TLB bookkeeping: pt_gen bumps on every PTE change; tlb_gen = pt_gen at the
	 * last completed MMU flush. A quarantined BO needs tlb_gen > its clear gen. */
	uint64_t pt_gen;
	uint64_t tlb_gen;
	uint32_t tlb_flushes;

	/* Interrupt plumbing */
	unsigned irq_num;
	int irq_on;                   /* handler registered + sources unmasked */
	handle_t irq_handle;
	handle_t irq_cond;            /* == srv.cond: the handler broadcasts it */

	/* Shared with the handler (atomics only). */
	volatile uint32_t ev_core;    /* CTL_INT_STS bits seen, not yet drained */
	volatile uint32_t ev_hub;     /* HUB_INT_STS bits seen, not yet drained */
	volatile uint32_t irq_count;  /* handler invocations */
	volatile uint32_t irq_spurious;   /* reset by the event thread every loop */
	volatile uint32_t irq_count_snap; /* irq_count at the last event-thread loop */
	volatile uint32_t storm;      /* set by the handler's storm guard (1 spurious, 2 burst) */
	volatile uint32_t mmu_ctl_seen;
	/* Pre-staged binner-overflow chunk the handler may hand out on OUTOMEM
	 * (zero = nothing staged). ovf_consumed = VA of the chunk it handed out (the
	 * event thread attributes it); ovf_missed counts OUTOMEMs that found nothing
	 * staged (the event thread grants those directly). */
	volatile uint32_t ovf_stage_va;
	volatile uint32_t ovf_stage_size;
	volatile uint32_t ovf_consumed;
	volatile uint32_t ovf_missed;
} v3da_hw_t;


/* ------------------------------------------------------------------------- */
/* BOs (v3da_bo.c)                                                            */
/* ------------------------------------------------------------------------- */

enum v3da_bo_state { V3DA_BO_FREE = 0, V3DA_BO_LIVE, V3DA_BO_QUARANTINE };

typedef struct {
	int state;                    /* enum v3da_bo_state */
	uint32_t handle;              /* slot + 1 */
	uint32_t owner;               /* creating client id (0 = server) */
	uint32_t refs;                /* handle refs: creator + importers */
	uint32_t inflight;            /* jobs referencing it (dropped when the job's fence completes) */
	uint32_t flags;               /* V3DA_BO_* */
	void *cpu;                    /* server's own mapping */
	uintptr_t pa;
	uint32_t gpuva;
	uint32_t pages;
	/* last use per queue (WAIT_BO / implicit sync; G6: the reservation object of a shared BO) */
	v3da_fence_t last[V3DA_Q_COUNT];
	uint64_t last_gseq[V3DA_Q_COUNT];   /* the submission order of last[q] (BO_LAST_FENCE: newest first) */
	int scanout;                  /* 0, or 1 + the firmware-fb buffer backing its GPU pages */
	int low;                      /* V3DA_BO_LOWMEM placed below 1 GiB: its footprint counts in srv.low_live */
	/* PRIME import (BO_IMPORT): the pages belong to another server's export; `cpu`
	 * is this server's mapping of it (the E1 window reference that keeps the pages
	 * alive), released with munmap() after quarantine - never pooled. */
	int imported;
	v3da_memref_t imp_mem;        /* {OID, cache, port, size, id}: what BO_MMAP answers */
	/* PRIME export (BO_EXPORT, G4). `refs` counts: the creator's handle (owner != 0),
	 * one per importing client (a bit in `sharers`, bit = client id - 1) and one per
	 * open descriptor of the /v3dbuf name (`fd_opens`). The name is withdrawn when
	 * refs reaches 0; the block then waits out the quarantine as any BO's. */
	int exported;                 /* published: memExport under {srv.buf_port, handle} */
	uint32_t fd_opens;
	uint64_t sharers;
	/* quarantine */
	uint64_t clear_pt_gen;        /* pt_gen after the PTE clear */
	uint64_t pass[V3DA_Q_COUNT];  /* hw_submitted snapshot: wait for hw_completed >= pass */
} v3da_bo_t;



/* ------------------------------------------------------------------------- */
/* Clients, syncobjs                                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
	int used;
	int state;                    /* enum v3da_syncobj_state */
	v3da_fence_t fence;
} v3da_syncobj_t;

typedef struct {
	int used;
	uint32_t id;                  /* == index + 1 == the descriptor's oid.id */
	int pid;
	int hello;                    /* HELLO done */
	uint32_t slot;                /* == index */
	uint64_t next_seq[V3DA_Q_COUNT];   /* last assigned seqno per queue */
	uint32_t nparked;
	v3da_syncobj_t sync[V3DA_MAX_SYNCOBJS];
} v3da_client_t;


/* ------------------------------------------------------------------------- */
/* Jobs and queues (v3da_sched.c)                                             */
/* ------------------------------------------------------------------------- */

typedef struct v3da_job {
	struct v3da_job *next;
	int queue;                    /* enum v3da_queue */
	uint32_t client;              /* client id */
	v3da_fence_t fence;
	uint64_t gseq;                /* global submission order */
	uint64_t t_submit_us;
	uint64_t t_kick_us;
	int error;                    /* completes with an error */

	/* In-fences, resolved from the submit's in-syncobjs at submit time. */
	v3da_fence_t dep[V3DA_SUBMIT_MAX_SEMS];
	uint32_t ndep;

	/* CL pairs: bin->render points at its render job, render->bin at its bin job
	 * while that is not yet complete (NULL afterwards). A render whose bin failed
	 * is completed with an error without ever being kicked. */
	struct v3da_job *render;
	struct v3da_job *bin;
	int dep_error;

	/* BOs referenced by the submit (held by the LAST job of the submit: the
	 * render, TFU or CSD job). Dropped at completion. */
	uint32_t *bos;
	uint32_t nbo;

	/* Binner-overflow chunks written by this job's bin (chunk index list; moved
	 * from the bin job to its render job at bin completion, freed when the render
	 * completes: the render reads the tile lists the binner wrote there). */
	int ovf_head;

	/* Watchdog */
	uint64_t wd_check_us;         /* next progress check */
	uint64_t wd_progress_us;      /* last time progress was seen */
	uint32_t wd_ca, wd_ra;        /* CTnCA / CTnRA (CSD: CURRENT_CFG4) at the last check */
	uint32_t qpu_acks;

	union {
		v3da_cl_desc_t cl;
		v3da_tfu_desc_t tfu;
		v3da_csd_desc_t csd;
	} d;

	/* NOP test job; `join`: a G6 BO_ATTACH_FENCE job (zero length, not a NOP in the stats) */
	uint32_t delay_us;
	uint64_t done_at_us;
	int join;
} v3da_job_t;

typedef struct {
	v3da_job_t *head[V3DA_MAX_CLIENTS];   /* per-client FIFO */
	v3da_job_t *tail[V3DA_MAX_CLIENTS];
	v3da_job_t *active;                   /* one job in flight */
	uint32_t rr;                          /* round-robin cursor (client index) */
	uint32_t pending;                     /* queued, not kicked */

	/* statistics (DBG_QSTATS) */
	uint32_t st_jobs, st_errors, st_max_us, st_oom;
	uint64_t st_busy_us, st_wait_us;
} v3da_queue_t;

/* Binner-overflow pool: premapped at init (uncached, contiguous, one GPU VA
 * range), cut into equal chunks. A chunk is FREE, STAGED (armed for the IRQ
 * handler), or owned by exactly one job (chunk_next links a job's list). */
typedef struct {
	uint32_t gpuva;               /* base of the pool */
	uint32_t chunk_bytes;
	uint32_t nchunks;
	int free_head;
	uint32_t nfree;
	int staged;                   /* chunk index armed in hw.ovf_stage_*, -1 = none */
	int next[V3DA_MAX_CHUNKS];
	uint32_t starved;             /* stat: OUTOMEM with nothing to hand out at once */
	int bin_waiting;              /* the active bin job is stalled for memory */
} v3da_ovf_t;

/* Transitional present family (firmware pan through /dev/vcmbox). */
typedef struct {
	uint32_t pa;                  /* buffer 0 */
	uint32_t width, height, pitch;
	uint32_t bytes;               /* one buffer */
	uint32_t virt_h;
	uint32_t nbuf;                /* 0 = no SCANOUT_INFO yet */
	uint32_t claimed[V3DA_SCANOUT_MAX];   /* handle of the BO backed by buffer i, 0 = free */
	struct {
		uint32_t buf;
		int gated;
		v3da_fence_t fence;
	} q[V3DA_MAX_FLIPS];
	uint32_t nq;
	uint32_t flips;
	uint32_t flips_deferred;
	uint32_t shown;               /* buffer on screen */
	/* Present self-check (qstat pan_err= px_chg=): the firmware's answer to each
	 * pan, and whether the buffer being panned to changed since the last pan. */
	uint32_t pan_err;             /* SET_VIRTUAL_OFFSET calls that returned an error */
	const volatile uint32_t *fb;  /* uncached read-only map of all nbuf buffers, or NULL */
	size_t fb_len;
	uint32_t px_last;             /* sample hash at the previous pan */
	uint32_t px_sampled;          /* pans sampled */
	uint32_t px_changed;          /* ... whose sample differed from the previous pan's */
} v3da_scanout_t;


/* ------------------------------------------------------------------------- */
/* Parked requests                                                            */
/* ------------------------------------------------------------------------- */

enum v3da_wait_kind { V3DA_WAIT_FENCE = 1, V3DA_WAIT_SYNCOBJ, V3DA_WAIT_BO };

/* A request the server has received and not answered. Owned by exactly one list
 * (srv.waits, or a local "to answer" list after wait_claim()); whoever claims it
 * under srv.lock is its only responder. Rids are reused immediately by the kernel
 * (E5 condition 3), so a second respond would answer a stranger. */
typedef struct v3da_wait {
	struct v3da_wait *next, *prev;
	int kind;
	uint32_t client;
	msg_t msg;                    /* the received message; o.raw is filled on answer */
	msg_rid_t rid;
	uint32_t port;                /* the port it arrived on: rids are per port (M3 part 2, card1) */
	uint64_t deadline_us;
	v3da_fence_t fence[V3DA_SYNCOBJ_WAIT_MAX];
	uint32_t sync_handle[V3DA_SYNCOBJ_WAIT_MAX];
	uint32_t n;
	int all;
	int for_submit;
} v3da_wait_t;


/* ------------------------------------------------------------------------- */
/* The server                                                                 */
/* ------------------------------------------------------------------------- */

typedef struct {
	uint32_t port;                /* /dev/v3d-async and /dev/dri/renderD128 */
	uint32_t port_card1;          /* /dev/dri/card1 (0 = not created); same protocol, shared clients */
	uint32_t rx_port;             /* port of the request being handled (set under srv.lock) */
	uint32_t imports;             /* live imported BOs (logged) */
	uint32_t buf_port;            /* V3DA_BUF_NS (0 = not registered: BO_EXPORT -ENODEV) */
	uint32_t exports;             /* BOs published under V3DA_BUF_NS right now (logged) */
	handle_t lock;
	handle_t cond;                /* event thread wakes on it (IRQ handler + dispatch) */
	int quit;
	int verbose;

	v3da_hw_t hw;

	v3da_fence_page_t *fp;        /* server's cached mapping */
	uintptr_t fp_pa;

	v3da_client_t clients[V3DA_MAX_CLIENTS];
	uint32_t nclients;
	uint64_t slot_gen[V3DA_MAX_CLIENTS];

	v3da_queue_t q[V3DA_Q_COUNT];
	int mode;                     /* enum v3da_mode */
	uint32_t knobs;               /* V3DA_KNOB_* */
	uint32_t wedge_ms;            /* no progress this long = wedge (default 500) */
	uint32_t stat_ms;             /* periodic qstat line (0 = off) */
	uint32_t ovf_chunk_kib;       /* -c */
	v3da_ovf_t ovf;
	v3da_scanout_t scan;

	/* concurrency accounting (all under srv.lock) */
	uint32_t nbusy;               /* hardware queues with a job on the hardware */
	uint64_t acct_t_us;           /* last accounting point */
	uint64_t acct_t0_us;          /* window start */
	uint64_t any_busy_us, overlap_us;
	uint32_t wedges;
	uint64_t stat_next_us;
	uint32_t stat_jobs_seen;
	uint32_t submit_rejects;      /* SUBMIT_* refused (each names itself in a `V3DA reject` line) */
	uint32_t cl_bcl_wrap;         /* SUBMIT_CL accepted with bcl_end < bcl_start (a chained BCL) */
	uint32_t cl_render_only;      /* SUBMIT_CL with bcl_start == bcl_end: no bin job (DRM semantics) */
	/* G6 cross-process implicit sync (the `V3DA srv g6` line at exit / DBG_STATS time) */
	uint32_t g6_queries;          /* BO_LAST_FENCE answered */
	uint32_t g6_pending;          /* ... with at least one pending fence */
	uint32_t g6_attach;           /* BO_ATTACH_FENCE join jobs queued */
	uint32_t g6_attach_busy;      /* ... refused -EBUSY (too many dependencies) */
	uint32_t g6_implicit;         /* submits that got >= 1 cross-client implicit dependency */
	uint32_t g6_implicit_deps;    /* implicit dependencies added in total */
	uint32_t g6_dropped;          /* implicit dependencies dropped (the job's dependency list was full) */

	v3da_bo_t bos[V3DA_MAX_BOS];
	uint32_t bo_gen[V3DA_MAX_BOS];   /* per-slot handle generation */
	uint32_t nbos;                /* high-water mark of slots ever used */
	v3da_pool_block_t pool[V3DA_MAX_POOL];
	uint32_t npool;
	/* Scan-out placement (V3DA_BO_LOWMEM, proto 5; v3da_lowmem.h): the qstat line's
	 * `low=<live>/<budget>KiB` and the `V3DA srv low` lines */
	uint64_t low_budget;          /* -L: bytes of low memory LOWMEM BOs may hold (buddy footprint) */
	uint64_t low_live;            /* ... held now (live + quarantined LOWMEM BOs placed low) */
	uint64_t low_peak;
	uint32_t low_bos;             /* LOWMEM BOs placed below 1 GiB */
	uint32_t low_from_pool;       /* ... of which from a pooled low block */
	uint32_t low_fallback;        /* LOWMEM BOs NOT placed low (budget, no low block in the tries, no memory) */
	uint32_t low_tries;           /* fresh blocks taken for LOWMEM BOs */
	uint32_t low_rejected;        /* ... handed back to the kernel (not counted in pages_to_kernel) */
	uint32_t low_notes;           /* `V3DA srv low` lines printed (capped) */

	v3da_wait_t *waits;           /* parked requests (doubly linked) */
	uint32_t nparked;
	uint32_t parked_max;
	uint64_t dead_flush[V3DA_MAX_CLIENTS][V3DA_Q_COUNT];   /* last seqno of a dead client's discarded jobs */

	/* poll-mode knobs */
	uint32_t poll_us;

	/* stats */
	uint32_t loops;
	uint32_t timeouts;
	uint32_t nops_done;
	uint32_t resets;
	uint32_t bo_quarantine_passed;
	uint32_t pages_to_kernel;
	uint32_t stray_fldone;        /* FLDONE with no active bin job (self-test) */
	uint32_t stray_tfuc;          /* TFUC with no active TFU job (self-test) */
} v3da_srv_t;

extern v3da_srv_t srv;


/* ------------------------------------------------------------------------- */
/* Cross-file API                                                             */
/* ------------------------------------------------------------------------- */

/* time */
uint64_t v3da_now_us(void);

/* fence-page helpers (release / acquire u64) */
static inline void v3da_store64(volatile uint64_t *p, uint64_t v)
{
	__atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline uint64_t v3da_load64(const volatile uint64_t *p)
{
	return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

/* v3da_hw.c */
int v3da_hw_init(v3da_hw_t *hw);
int v3da_hw_irq_enable(v3da_hw_t *hw);      /* locked; returns interrupt() rc */
void v3da_hw_irq_disable(v3da_hw_t *hw);    /* locked */
void v3da_hw_poll_status(v3da_hw_t *hw);    /* poll mode: read + W1C STS into the event words */
void v3da_hw_drain(v3da_hw_t *hw);          /* before a kick: fold + W1C everything but OUTOMEM */
void v3da_hw_mmu_flush(v3da_hw_t *hw);      /* MMUC flush + TLB clear, bounded spins */
uint32_t v3da_hw_va_alloc(v3da_hw_t *hw, uint32_t pages);
void v3da_hw_va_free(v3da_hw_t *hw, uint32_t gpuva, uint32_t pages);
int v3da_hw_selftest(v3da_hw_t *hw, v3da_irq_selftest_resp_t *out);   /* called unlocked */
void v3da_hw_shutdown(v3da_hw_t *hw);       /* locked: mask + unregister */
/* Wedge recovery (locked): mask, drain the GMP, true V3D reset (PM_V3DRSTN held,
 * power on again through vcmbox with clock read-back), core registers, TLB,
 * interrupt masks back if IRQ mode is on. Returns the power-on result. */
int v3da_hw_reset(v3da_hw_t *hw);
void v3da_hw_l2t_flush_wait(v3da_hw_t *hw); /* bounded spin: no L2T flush pending (GFXH-1897) */
/* Firmware property call through /dev/vcmbox (2 value words). */
int v3da_hw_vc_prop2(uint32_t tag, uint32_t w0, uint32_t w1, uint32_t nIn, uint32_t *ans0, uint32_t *ans1);

/* v3da_param.c (separate: the DRM uapi headers clash with <sys/ioctl.h>) */
int v3da_get_param(uint32_t param, uint64_t *value);

/* v3da_bo.c */
int v3da_bo_create(uint32_t client, uint32_t size, uint32_t flags, v3da_bo_create_resp_t *out);
int v3da_bo_pin_for_job(const uint32_t *handles, uint32_t n);   /* validate + inflight++ (all or none) */
void v3da_bo_mark_use(const uint32_t *handles, uint32_t n, const v3da_fence_t *f, uint64_t gseq);
int v3da_fence_set_add(v3da_fence_t *dep, uint32_t *ndep, uint32_t max, const v3da_fence_t *f);   /* 1 = full */
/* G6 (locked). Implicit dependencies of a submit by fence-page slot `slot`: every
 * pending last-use fence of ANOTHER slot on the named BOs is added to dep[] (one per
 * {slot, queue}, the newest). Returns how many had to be dropped (dep[] full). */
uint32_t v3da_bo_implicit_deps(uint32_t slot, const uint32_t *handles, uint32_t n, v3da_fence_t *dep, uint32_t *ndep,
	uint32_t max);
/* G6 (locked): the BOs a v3da_bo_sync_req_t names (by the client's handle, a /v3dbuf
 * export, or every import of a /kmsbuf name), up to max. Returns the count or -errno. */
int v3da_bo_sync_targets(uint32_t client, const v3da_bo_sync_req_t *rq, v3da_bo_t **out, uint32_t max);
/* G6 (locked): BO_LAST_FENCE */
int v3da_bo_last_fence(uint32_t client, const v3da_bo_sync_req_t *rq, v3da_bo_fences_resp_t *out);
void v3da_bo_unpin(const uint32_t *handles, uint32_t n);        /* inflight--, quarantine if unreferenced */
int v3da_bo_map_ovf_pool(uint32_t bytes, uint32_t *gpuva);      /* init: the binner-overflow pool */
int v3da_bo_close(uint32_t client, uint32_t handle);
int v3da_bo_mmap(uint32_t handle, v3da_bo_resp_t *out);
int v3da_bo_offset(uint32_t handle, uint32_t *gpuva);
int v3da_bo_checksum(uint32_t handle, uint32_t off, uint32_t len, v3da_bo_checksum_resp_t *out);
void v3da_bo_client_gone(uint32_t client);
void v3da_bo_quarantine_poll(void);
void v3da_bo_counts(uint32_t *live, uint32_t *quar, uint32_t *pooled);
void v3da_bo_low_counts(uint64_t *live, uint64_t *budget, uint32_t *bos, uint32_t *fallback);   /* qstat low= */
v3da_bo_t *v3da_bo_find(uint32_t handle);
/* BO_IMPORT. Called UNLOCKED (it opens and maps another server's buffer name, IPC
 * that must not stall the event thread); takes srv.lock itself for the table work. */
int v3da_bo_import(uint32_t client, const v3da_bo_import_req_t *rq, v3da_bo_create_resp_t *out);
/* BO_EXPORT (locked, G4) and the V3DA_BUF_NS namespace thread (takes srv.lock). */
int v3da_bo_export(uint32_t client, uint32_t handle, v3da_bo_resp_t *out);
void v3da_bufns_thread(void *arg);

/* v3da_main.c: an mtGetAttrAll answer (fstat) for a node or a buffer name */
int v3da_attr_all(msg_t *msg, uint32_t mode, uint64_t size, uint32_t port);

/* v3da_sched.c */
int v3da_sched_init(void);
int v3da_submit(v3da_client_t *c, int op, const v3da_submit_t *hdr, const void *data, size_t size,
	v3da_submit_resp_t *out);
int v3da_set_mode(const v3da_mode_req_t *rq, v3da_mode_resp_t *out);
int v3da_qstats(const v3da_qstats_req_t *rq, v3da_resp_t *r);
int v3da_scanout_info(const v3da_scanout_req_t *rq, v3da_scanout_resp_t *out);
int v3da_flip(v3da_client_t *c, const v3da_flip_req_t *rq, v3da_flip_resp_t *out);
int v3da_syncobj_import(v3da_client_t *c, uint32_t handle, const v3da_fence_t *f);
void v3da_event_thread(void *arg);
int v3da_submit_nop(v3da_client_t *c, uint32_t delay_us, v3da_fence_t *out);
int v3da_bo_attach_fence(v3da_client_t *c, const v3da_bo_sync_req_t *rq, v3da_fence_t *out);   /* G6, v3da_jobs.c */
int v3da_fence_signaled(const v3da_fence_t *f, int *error);
int v3da_fence_valid(const v3da_client_t *c, const v3da_fence_t *f);
/* Park msg as a wait. Returns 0 when parked (the server now owns the request:
 * the caller must NOT respond), 1 when already satisfied (the response fields in
 * msg->o.raw are filled; the caller responds with err 0), or a negative errno
 * (the caller responds with it). */
int v3da_wait_park(v3da_client_t *c, int kind, msg_t *msg, msg_rid_t rid,
	const v3da_fence_t *fences, const uint32_t *handles, uint32_t n, int all,
	int for_submit, uint32_t timeout_ms);
void v3da_waits_client_gone(uint32_t client, v3da_wait_t **answer);
void v3da_waits_all(v3da_wait_t **answer, int err);
void v3da_waits_answer(v3da_wait_t *list);  /* UNLOCKED: respond + free */
uint32_t v3da_jobs_inflight(void);
void v3da_jobs_client_gone(uint32_t client);
void v3da_kick_event_thread(void);          /* locked */
int v3da_slot_busy(uint32_t slot);          /* a job of this slot is still in flight */
void v3da_slot_assign(uint32_t slot);       /* new generation, zeroed row */

/* v3da_jobs.c (all locked) */
int v3da_ovf_init(void);
int v3da_jobs_hw_busy(void);
void v3da_jobs_events(uint32_t core, uint32_t hub, uint64_t now);   /* event thread */
void v3da_jobs_tick(uint64_t now);                                  /* event thread */
uint64_t v3da_jobs_next_deadline(uint64_t now);
void v3da_sched_run(void);
void v3da_fence_complete(const v3da_job_t *j, int error);           /* v3da_sched.c */

/* syncobjs (v3da_sched.c) */
int v3da_syncobj_create(v3da_client_t *c, uint32_t flags, uint32_t *handle);
int v3da_syncobj_destroy(v3da_client_t *c, uint32_t handle);
int v3da_syncobj_reset(v3da_client_t *c, uint32_t handle);
int v3da_syncobj_signal(v3da_client_t *c, uint32_t handle);
int v3da_syncobj_query(v3da_client_t *c, uint32_t handle, v3da_syncobj_resp_t *out);
v3da_syncobj_t *v3da_syncobj_get(v3da_client_t *c, uint32_t handle);

#endif /* _V3DA_H_ */
