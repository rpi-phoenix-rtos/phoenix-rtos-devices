/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - wire protocol
 *
 * The contract between rpi4-kms (/dev/kms + the /kmsbuf buffer namespace) and its
 * clients (kmstest today, libdrm-phoenix in M3). Design:
 * docs/gpu-new-lane/M2-kms-server.md.
 *
 * Transport (the rpi4-v3d-async pattern, M1 section 10)
 *   - A client open()s /dev/kms. The server answers mtOpen with a positive
 *     per-open CLIENT ID, which the kernel stores as the descriptor's oid.id; the
 *     kernel's mtClose for that id is the client-death notification.
 *   - HELLO is an ioctl() on that descriptor (so the server sees the id); every
 *     later request is a direct msgSend(mtDevCtl) to {server port, client id}.
 *   - Requests: kms_req_t in msg.i.raw, starting with KMS_MAGIC (bit 31 clear, so a
 *     raw request is never confused with an ioctl()-packed one, whose first word
 *     has IOC_INOUT in bits 31:30).
 *   - Responses: kms_resp_t in msg.o.raw; msg.o.err is the IPC status (EOK when
 *     the request was handled), kms_resp_t.err the per-op status.
 *   - Arrays (modes, property lists, atomic plane states, blobs) travel in
 *     msg.i.data / msg.o.data. Every hot-path op (PAGE_FLIP, WAIT_VBLANK,
 *     CRTC_*_SEQUENCE) is raw-only. Clients should keep i.data/o.data page-aligned:
 *     each unaligned end costs a shadow page and a copy (E5: 41 vs 73 us).
 *   - EVENTS are read() from the card descriptor, in the DRM wire layout
 *     (struct drm_event + drm_event_vblank / drm_event_crtc_sequence, copied below
 *     from libdrm's MIT drm.h), so libdrm's drmHandleEvent parses them unchanged.
 *     read() returns as many whole events as fit; it blocks while the queue is
 *     empty, at most KMS_READ_MAX_MS, then fails with -EAGAIN (a parked Phoenix
 *     client cannot be interrupted, E5 condition 1 - callers loop). O_NONBLOCK:
 *     -EAGAIN at once.
 *
 * Memory
 *   Buffers are never copied. A dumb BO is handed out as a MEMREF:
 *     KMS_MEM_OID  - a window of the server's contiguous pool published with
 *                    memExport() under {buffer port, id}: open("/kmsbuf/<id>",
 *                    O_RDONLY) and mmap() the descriptor with MAP_UNCACHED (the
 *                    export's memory type; any other type fails with -EINVAL).
 *     KMS_MEM_PHYS - a physical range the client maps with MAP_PHYSMEM |
 *                    MAP_UNCACHED (the firmware-framebuffer slots the "pan"
 *                    backend flips between; firmware memory cannot be exported).
 *   The layout is identical to v3da_memref_t (rpi4-v3d-async).
 *
 * Fences
 *   An in-fence is a fence of rpi4-v3d-async: (slot, queue, gen, seqno), the
 *   v3da_fence_t layout. The server checks it against the render server's fence
 *   page with one load - no IPC on the flip path.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _KMS_PROTO_H_
#define _KMS_PROTO_H_

#include <stdint.h>


#define KMS_DEV_NAME      "kms"        /* /dev/kms */
#define KMS_DRI_NAME      "dri/card0"  /* M3 part 2 (G10): the DRM primary node, same port as /dev/kms */
#define KMS_BUF_NS        "/kmsbuf"    /* buffer namespace (own port) */
#define KMS_PROTO_VERSION 2u           /* 2: gap G7 (KMS_OP_PRIME_IMPORT) */
/* The oldest protocol a client may HELLO with: 1 = M2 Stage A. The server accepts
 * KMS_PROTO_BASE..KMS_PROTO_VERSION and replies with ITS version, which is how a
 * client learns what it may send (a proto-1 server refuses any other HELLO with
 * -EPROTO; a proto-2 client then retries with 1). */
#define KMS_PROTO_BASE    1u
#define KMS_PROTO_PRIME_IMPORT 2u      /* first version with KMS_OP_PRIME_IMPORT */
#define KMS_MAGIC         0x31534d4bu  /* "KMS1" little-endian; bit 31 clear */
#define KMS_DRIVER_NAME   "vc4"        /* DRM_IOCTL_VERSION name: Mesa kmsro pairs v3d with "vc4" */

#define KMS_READ_MAX_MS   2000u        /* longest a read()/WAIT_VBLANK stays parked */


/* ------------------------------------------------------------------------- */
/* Object ids (one id space, as in DRM)                                       */
/* ------------------------------------------------------------------------- */

#define KMS_MAX_CRTCS     2u
#define KMS_PLANES_PER_CRTC 8u   /* firmware: 0 primary, 1..6 overlay, 7 cursor */

#define KMS_ID_CONNECTOR(i) (0x20u + (i))
#define KMS_ID_ENCODER(i)   (0x30u + (i))
#define KMS_ID_CRTC(i)      (0x40u + (i))
#define KMS_ID_PLANE(c, p)  (0x50u + (c) * KMS_PLANES_PER_CRTC + (p))
#define KMS_ID_FB_BASE      0x1000u
#define KMS_ID_BLOB_BASE    0x10000u

/* DRM object types (drm_mode.h DRM_MODE_OBJECT_*) */
#define KMS_OBJ_CRTC      0xccccccccu
#define KMS_OBJ_CONNECTOR 0xc0c0c0c0u
#define KMS_OBJ_ENCODER   0xe0e0e0e0u
#define KMS_OBJ_PLANE     0xeeeeeeeeu
#define KMS_OBJ_FB        0xfbfbfbfbu
#define KMS_OBJ_BLOB      0xbbbbbbbbu

/* Property ids. Names are the DRM names; the value ranges are DRM's. */
enum kms_prop {
	KMS_PROP_NONE = 0,
	/* plane */
	KMS_PROP_TYPE = 0x100,   /* enum, immutable: Overlay 0, Primary 1, Cursor 2 */
	KMS_PROP_FB_ID,
	KMS_PROP_CRTC_ID,        /* plane and connector */
	KMS_PROP_SRC_X,          /* 16.16 */
	KMS_PROP_SRC_Y,
	KMS_PROP_SRC_W,
	KMS_PROP_SRC_H,
	KMS_PROP_CRTC_X,         /* signed */
	KMS_PROP_CRTC_Y,
	KMS_PROP_CRTC_W,
	KMS_PROP_CRTC_H,
	KMS_PROP_IN_FENCE_FD,    /* signed, -1 = none (M3: sync-file fd -> v3da fence) */
	KMS_PROP_IN_FORMATS,     /* blob, immutable */
	KMS_PROP_ZPOS,           /* range */
	KMS_PROP_ALPHA,          /* range 0..0xffff */
	KMS_PROP_ROTATION,       /* bitmask: rotate-0, rotate-180, reflect-x, reflect-y */
	/* crtc */
	KMS_PROP_ACTIVE = 0x140,
	KMS_PROP_MODE_ID,        /* blob */
	KMS_PROP_OUT_FENCE_PTR,  /* u64 pointer, filled by libdrm-phoenix client-side */
	KMS_PROP_VRR_ENABLED,
	/* connector */
	KMS_PROP_DPMS = 0x180,   /* enum On 0, Standby 1, Suspend 2, Off 3 */
	KMS_PROP_EDID,           /* blob, immutable (0 = none) */
	KMS_PROP_LINK_STATUS,    /* enum Good 0, Bad 1 */
	KMS_PROP_NON_DESKTOP,    /* range 0..1, immutable */
	KMS_PROP_MAX
};

#define KMS_PROP_NAME_LEN 32u


/* ------------------------------------------------------------------------- */
/* DRM wire layouts reused verbatim (libdrm include/drm/drm.h, drm_mode.h; MIT) */
/* ------------------------------------------------------------------------- */

#define KMS_DRM_EVENT_VBLANK        0x01u
#define KMS_DRM_EVENT_FLIP_COMPLETE 0x02u
#define KMS_DRM_EVENT_CRTC_SEQUENCE 0x03u

typedef struct {
	uint32_t type;
	uint32_t length;
} kms_drm_event_t;

typedef struct {
	kms_drm_event_t base;
	uint64_t user_data;
	uint32_t tv_sec;
	uint32_t tv_usec;
	uint32_t sequence;
	uint32_t crtc_id;
} kms_drm_event_vblank_t;

typedef struct {
	kms_drm_event_t base;
	uint64_t user_data;
	int64_t time_ns;
	uint64_t sequence;
} kms_drm_event_crtc_sequence_t;

#define KMS_DISPLAY_MODE_LEN 32u

/* struct drm_mode_modeinfo */
typedef struct {
	uint32_t clock;          /* kHz */
	uint16_t hdisplay, hsync_start, hsync_end, htotal, hskew;
	uint16_t vdisplay, vsync_start, vsync_end, vtotal, vscan;
	uint32_t vrefresh;
	uint32_t flags;
	uint32_t type;           /* DRM_MODE_TYPE_PREFERRED (1<<3) | DRIVER (1<<6) */
	char name[KMS_DISPLAY_MODE_LEN];
} kms_modeinfo_t;

#define KMS_MODE_TYPE_PREFERRED (1u << 3)
#define KMS_MODE_TYPE_DRIVER    (1u << 6)
#define KMS_MODE_FLAG_PHSYNC    (1u << 0)
#define KMS_MODE_FLAG_NHSYNC    (1u << 1)
#define KMS_MODE_FLAG_PVSYNC    (1u << 2)
#define KMS_MODE_FLAG_NVSYNC    (1u << 3)
#define KMS_MODE_FLAG_INTERLACE (1u << 4)

/* DRM caps (drm.h DRM_CAP_*) */
#define KMS_CAP_DUMB_BUFFER          0x1u
#define KMS_CAP_VBLANK_HIGH_CRTC     0x2u
#define KMS_CAP_DUMB_PREFERRED_DEPTH 0x3u
#define KMS_CAP_DUMB_PREFER_SHADOW   0x4u
#define KMS_CAP_PRIME                0x5u
#define KMS_CAP_TIMESTAMP_MONOTONIC  0x6u
#define KMS_CAP_ASYNC_PAGE_FLIP      0x7u
#define KMS_CAP_CURSOR_WIDTH         0x8u
#define KMS_CAP_CURSOR_HEIGHT        0x9u
#define KMS_CAP_ADDFB2_MODIFIERS     0x10u
#define KMS_CAP_PAGE_FLIP_TARGET     0x11u
#define KMS_CAP_CRTC_IN_VBLANK_EVENT 0x12u
#define KMS_CAP_SYNCOBJ              0x13u
#define KMS_CAP_SYNCOBJ_TIMELINE     0x14u
#define KMS_CAP_ATOMIC_ASYNC_PAGE_FLIP 0x15u

/* DRM client caps (drm.h DRM_CLIENT_CAP_*) */
#define KMS_CLIENT_CAP_STEREO_3D        1u
#define KMS_CLIENT_CAP_UNIVERSAL_PLANES 2u
#define KMS_CLIENT_CAP_ATOMIC           3u
#define KMS_CLIENT_CAP_ASPECT_RATIO     4u
#define KMS_CLIENT_CAP_WRITEBACK_CONNECTORS 5u

/* drm_mode.h flags */
#define KMS_PAGE_FLIP_EVENT          0x01u
#define KMS_PAGE_FLIP_ASYNC          0x02u
#define KMS_ATOMIC_TEST_ONLY         0x0100u
#define KMS_ATOMIC_NONBLOCK          0x0200u
#define KMS_ATOMIC_ALLOW_MODESET     0x0400u

#define KMS_CONNECTOR_HDMIA 11u   /* DRM_MODE_CONNECTOR_HDMIA */
#define KMS_ENCODER_TMDS    2u    /* DRM_MODE_ENCODER_TMDS */
#define KMS_CONNECTED       1u
#define KMS_DISCONNECTED    2u
#define KMS_CONN_UNKNOWN    3u

#define KMS_PLANE_TYPE_OVERLAY 0u
#define KMS_PLANE_TYPE_PRIMARY 1u
#define KMS_PLANE_TYPE_CURSOR  2u

/* drm_fourcc.h: fourcc_code(a, b, c, d) */
#define KMS_FOURCC(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define KMS_FMT_XRGB8888 KMS_FOURCC('X', 'R', '2', '4')   /* u32 x:R:G:B */
#define KMS_FMT_ARGB8888 KMS_FOURCC('A', 'R', '2', '4')
#define KMS_FMT_XBGR8888 KMS_FOURCC('X', 'B', '2', '4')   /* u32 x:B:G:R (the firmware fb here) */
#define KMS_FMT_ABGR8888 KMS_FOURCC('A', 'B', '2', '4')
#define KMS_MOD_LINEAR   0ull


/* ------------------------------------------------------------------------- */
/* Memory references and fences (same layouts as rpi4-v3d-async)              */
/* ------------------------------------------------------------------------- */

enum kms_mem_kind {
	KMS_MEM_NONE = 0,
	KMS_MEM_PHYS = 1,   /* addr = physical address; MAP_PHYSMEM | MAP_UNCACHED */
	KMS_MEM_OID = 2     /* {port, addr = id}: open(KMS_BUF_NS "/<id>") + mmap(fd, MAP_UNCACHED) */
};

enum kms_mem_cache {
	KMS_CACHE_CACHED = 0,
	KMS_CACHE_UNCACHED = 1
};

typedef struct {
	uint16_t kind;    /* enum kms_mem_kind */
	uint16_t cache;   /* enum kms_mem_cache */
	uint32_t port;    /* KMS_MEM_OID only */
	uint64_t size;    /* bytes, page multiple */
	uint64_t addr;    /* PHYS: physical address; OID: object id */
} kms_memref_t;

/* A fence of rpi4-v3d-async (v3da_fence_t). seqno == 0 means "no fence". */
typedef struct {
	uint16_t slot;
	uint16_t queue;
	uint32_t gen;
	uint64_t seqno;
} kms_fence_t;


/* ------------------------------------------------------------------------- */
/* Opcodes                                                                    */
/* ------------------------------------------------------------------------- */

enum kms_op {
	KMS_OP_NONE = 0,
	KMS_OP_HELLO,              /* ioctl only (KMS_IOC_HELLO) */
	KMS_OP_GET_CAP,            /* DRM_IOCTL_GET_CAP */
	KMS_OP_SET_CLIENT_CAP,     /* DRM_IOCTL_SET_CLIENT_CAP */
	KMS_OP_SET_MASTER,         /* DRM_IOCTL_SET_MASTER / DROP_MASTER / AUTH_MAGIC: accepted no-ops */
	KMS_OP_DROP_MASTER,
	KMS_OP_AUTH_MAGIC,

	KMS_OP_GET_RESOURCES = 16, /* DRM_IOCTL_MODE_GETRESOURCES (o.data: fb ids) */
	KMS_OP_GET_CONNECTOR,      /* DRM_IOCTL_MODE_GETCONNECTOR (o.data: kms_modeinfo_t[]) */
	KMS_OP_GET_ENCODER,        /* DRM_IOCTL_MODE_GETENCODER */
	KMS_OP_GET_CRTC,           /* DRM_IOCTL_MODE_GETCRTC (o.data: kms_modeinfo_t, optional) */
	KMS_OP_SET_CRTC,           /* DRM_IOCTL_MODE_SETCRTC (current mode only in Stage A) */
	KMS_OP_GET_PLANE_RESOURCES,/* DRM_IOCTL_MODE_GETPLANERESOURCES */
	KMS_OP_GET_PLANE,          /* DRM_IOCTL_MODE_GETPLANE */
	KMS_OP_GET_PROPERTIES,     /* DRM_IOCTL_MODE_OBJ_GETPROPERTIES (o.data: kms_prop_value_t[]) */
	KMS_OP_GET_PROPERTY,       /* DRM_IOCTL_MODE_GETPROPERTY (o.data: u64 values / kms_prop_enum_t[]) */
	KMS_OP_GET_BLOB,           /* DRM_IOCTL_MODE_GETPROPBLOB (o.data: bytes) */
	KMS_OP_CREATE_BLOB,        /* DRM_IOCTL_MODE_CREATEPROPBLOB (i.data: bytes) */
	KMS_OP_DESTROY_BLOB,       /* DRM_IOCTL_MODE_DESTROYPROPBLOB */

	KMS_OP_CREATE_DUMB = 32,   /* DRM_IOCTL_MODE_CREATE_DUMB (+ memref) */
	KMS_OP_MAP_DUMB,           /* DRM_IOCTL_MODE_MAP_DUMB (memref instead of an mmap offset) */
	KMS_OP_DESTROY_DUMB,       /* DRM_IOCTL_MODE_DESTROY_DUMB / GEM_CLOSE */
	KMS_OP_ADDFB2,             /* DRM_IOCTL_MODE_ADDFB2 (single plane, linear) */
	KMS_OP_RMFB,               /* DRM_IOCTL_MODE_RMFB */
	KMS_OP_PRIME_EXPORT,       /* DRM_IOCTL_PRIME_HANDLE_TO_FD: returns the memref; library open()s it */
	KMS_OP_PRIME_IMPORT,       /* = 38, proto 2 (G7): DRM_IOCTL_PRIME_FD_TO_HANDLE of a foreign buffer */

	KMS_OP_PAGE_FLIP = 48,     /* DRM_IOCTL_MODE_PAGE_FLIP (+ in-fence) */
	KMS_OP_ATOMIC,             /* DRM_IOCTL_MODE_ATOMIC, flattened (i.data: kms_atomic_plane_t[]) */
	KMS_OP_WAIT_VBLANK,        /* DRM_IOCTL_WAIT_VBLANK (blocking form parks; event form queues) */
	KMS_OP_CRTC_GET_SEQUENCE,  /* DRM_IOCTL_CRTC_GET_SEQUENCE */
	KMS_OP_CRTC_QUEUE_SEQUENCE,/* DRM_IOCTL_CRTC_QUEUE_SEQUENCE */

	KMS_OP_DBG_BO_CHECKSUM = 112, /* server-side FNV-1a of a BO range (sharing proof) */
	KMS_OP_DBG_STATS,
	KMS_OP_DBG_QUIT
};


/* ------------------------------------------------------------------------- */
/* Per-op arguments                                                           */
/* ------------------------------------------------------------------------- */

enum kms_backend_id {
	KMS_BACKEND_PAN = 0,    /* firmware-framebuffer panning (SET_VIRTUAL_OFFSET): primary only (fallback) */
	KMS_BACKEND_PLANE = 1   /* firmware planes (SET_PLANE): primary + cursor (+ overlays) (default) */
};

enum kms_vblank_src {
	KMS_VBL_NONE = 0,
	KMS_VBL_IRQ,      /* SMI interrupt (GIC SPI 112 = Phoenix IRQ 144) */
	KMS_VBL_HVS,      /* HVS frame counter, polled */
	KMS_VBL_FWVSYNC,  /* firmware SET_VSYNC, blocking (holds /dev/vcmbox one frame per call) */
	KMS_VBL_TIMER     /* free-running timer at the reported refresh */
};

/* HELLO travels as ioctl(fd, KMS_IOC_HELLO, &hello): in = proto, out = the rest. */
typedef struct {
	uint32_t proto;         /* in: KMS_PROTO_VERSION; out: server's */
	uint32_t client_id;     /* out: == the descriptor's oid.id */
	uint32_t server_pid;
	uint32_t backend;       /* enum kms_backend_id */
	uint32_t vblank_src;    /* enum kms_vblank_src */
	uint32_t ncrtc;
	uint32_t buf_port;      /* port of KMS_BUF_NS (KMS_MEM_OID memrefs) */
	uint32_t refresh_mhz;   /* vblank rate of CRTC 0, millihertz */
	uint32_t width, height; /* current mode of CRTC 0 */
	uint32_t pad[2];
} kms_hello_t;

/* 'K' group, nr 1: IOC_INOUT | (len << 16) | ('K' << 8) | 1 (== _IOWR('K', 1, kms_hello_t)). */
#define KMS_IOC_HELLO ((unsigned long)(0xc0000000UL | ((unsigned long)(sizeof(kms_hello_t) & 0x1fffu) << 16) | \
	((unsigned long)'K' << 8) | 1UL))

typedef struct {
	uint64_t cap;
	uint64_t value;         /* SET_CLIENT_CAP in */
} kms_cap_req_t;

typedef struct {
	uint64_t value;
} kms_cap_resp_t;

typedef struct {
	uint32_t min_w, max_w, min_h, max_h;
	uint8_t ncrtc, nconn, nenc, nplane;
	uint32_t crtc[KMS_MAX_CRTCS];
	uint32_t conn[KMS_MAX_CRTCS];
	uint32_t enc[KMS_MAX_CRTCS];
	uint32_t nfb;           /* fb ids of this client in o.data (u32[]), if given */
} kms_resources_t;

typedef struct {
	uint32_t id;            /* object id */
	uint32_t type;          /* GET_PROPERTIES: KMS_OBJ_*; else 0 */
	uint32_t max;           /* capacity of o.data in elements (0 = counts only) */
	uint32_t pad;
} kms_obj_req_t;

typedef struct {
	uint32_t type;          /* KMS_CONNECTOR_HDMIA */
	uint32_t type_id;       /* 1-based, per type */
	uint32_t connection;    /* KMS_CONNECTED / KMS_DISCONNECTED / KMS_CONN_UNKNOWN */
	uint32_t mm_width, mm_height;
	uint32_t subpixel;
	uint32_t encoder_id;
	uint32_t nmodes;        /* total; min(nmodes, max) written to o.data */
	uint32_t nprops;
	uint32_t fw_display_id; /* firmware display id (GET_DISPLAY_ID) */
	uint32_t pad[2];
} kms_connector_t;

typedef struct {
	uint32_t type;          /* KMS_ENCODER_TMDS */
	uint32_t crtc_id;
	uint32_t possible_crtcs;
	uint32_t possible_clones;
} kms_encoder_t;

typedef struct {
	uint32_t fb_id;         /* on the primary plane, 0 = console (firmware fb) */
	uint32_t x, y;
	uint32_t gamma_size;
	uint32_t mode_valid;
	uint32_t hdisplay, vdisplay, vrefresh;
	uint64_t sequence;      /* vblank count */
	uint32_t pending_fb;    /* flip queued/armed, not yet complete */
	uint32_t pad;
} kms_crtc_t;

typedef struct {
	uint32_t crtc_id;
	uint32_t fb_id;         /* 0 = disable the primary (show the console) */
	int32_t x, y;           /* must be 0 in Stage A */
	uint32_t conn_id;
	uint32_t mode_hdisplay; /* must match the current mode (Stage A cannot set modes) */
	uint32_t mode_vdisplay;
	uint32_t pad;
} kms_set_crtc_req_t;

typedef struct {
	uint32_t nplanes;
	uint32_t plane[13];     /* Stage A exposes at most 8 per CRTC, one CRTC in practice */
} kms_plane_res_t;

#define KMS_PLANE_MAX_FMTS 6u

typedef struct {
	uint32_t crtc_id;       /* currently bound */
	uint32_t fb_id;
	uint32_t possible_crtcs;
	uint32_t type;          /* KMS_PLANE_TYPE_* */
	uint32_t nformats;
	uint32_t formats[KMS_PLANE_MAX_FMTS];
	uint32_t pad;
} kms_plane_t;

typedef struct {
	uint32_t prop_id;
	uint32_t pad;
	uint64_t value;
} kms_prop_value_t;

typedef struct {
	uint64_t value;
	char name[KMS_PROP_NAME_LEN];
} kms_prop_enum_t;

#define KMS_PROP_FLAG_RANGE     (1u << 1)   /* DRM_MODE_PROP_RANGE */
#define KMS_PROP_FLAG_IMMUTABLE (1u << 2)
#define KMS_PROP_FLAG_ENUM      (1u << 3)
#define KMS_PROP_FLAG_BLOB      (1u << 4)
#define KMS_PROP_FLAG_BITMASK   (1u << 5)
#define KMS_PROP_FLAG_OBJECT    (1u << 6)   /* DRM_MODE_PROP_OBJECT = 1 << 6 (extended type 1) */
#define KMS_PROP_FLAG_SIGNED    (2u << 6)   /* DRM_MODE_PROP_SIGNED_RANGE (extended type 2) */
#define KMS_PROP_FLAG_ATOMIC    0x80000000u

typedef struct {
	char name[KMS_PROP_NAME_LEN];
	uint32_t flags;         /* KMS_PROP_FLAG_* */
	uint32_t nvalues;       /* range: 2 (min, max) u64 in o.data; enum/bitmask: nenums */
	uint32_t nenums;        /* kms_prop_enum_t in o.data */
	uint32_t pad;
} kms_property_t;

typedef struct {
	uint32_t id;            /* blob id */
	uint32_t length;        /* bytes (GET: total, min(length, o.size) copied) */
} kms_blob_t;

/* CREATE_DUMB flags. Without KMS_DUMB_POOL the "pan" backend hands a mode-sized
 * 32-bpp request a firmware-fb slot (flippable, KMS_MEM_PHYS) while one is free,
 * and anything else a pool window (KMS_MEM_OID, not scannable by "pan"). */
#define KMS_DUMB_POOL (1u << 0)   /* always a pool window (exported with memExport) */

typedef struct {
	uint32_t width, height, bpp, flags;
} kms_create_dumb_req_t;

typedef struct {
	uint32_t handle;
	uint32_t pitch;
	uint64_t size;
	kms_memref_t mem;
} kms_dumb_resp_t;

typedef struct {
	uint32_t handle;
	uint32_t pad;
} kms_handle_req_t;

/*
 * PRIME_IMPORT (proto 2, gap G7): DRM_IOCTL_PRIME_FD_TO_HANDLE of a buffer another
 * server exported, so that ADDFB2 can scan it out (direct scan-out of a client
 * buffer, Present flips of client pixmaps). The client resolved the dma-buf
 * descriptor to {namespace port, id} (the byte layout of v3da_bo_import_req_t).
 *   ns = KMS_IMPORT_NS_V3DBUF ("/v3dbuf/<id>", an rpi4-v3d-async BO_EXPORT): the
 *   server opens the name itself, checks that it resolves to `port`, sizes it with
 *   lseek(SEEK_END), maps it with MAP_UNCACHED (the export's memory type) and
 *   resolves every page. It KEEPS THE DESCRIPTOR OPEN for the handle's whole life:
 *   an open /v3dbuf descriptor holds a reference on the render BO, so the block is
 *   never quarantined, pooled and zeroed while a framebuffer of it can still be on
 *   a plane. The descriptor is closed only when the handle is closed AND no
 *   framebuffer references it (the C1 rule of every BO here: a flip releases the
 *   outgoing framebuffer only once it has completed at a vblank).
 *   ns = KMS_IMPORT_NS_KMSBUF: one of this server's own /kmsbuf exports. The
 *   importer's OWN export returns the original handle (DRM; libdrm-phoenix
 *   short-circuits it locally). Another client's PRIME-exported pool BO becomes an
 *   alias: a new handle of the importer on the same pages, holding one reference on
 *   the exporter's BO (which keeps its pages and its /kmsbuf name until the alias
 *   handle is closed and no framebuffer of it is left), memref = the exporter's
 *   name; again on the same client: the same handle. A BO that was never
 *   PRIME-exported: -EACCES.
 * Refused at import: -ENOENT (the name is not, or no longer, exported), -EINVAL
 * (wrong port, bad size, pages not physically contiguous: the firmware plane
 * fetches one linear range), -ENOSPC (no BO slot).
 * An imported buffer that the firmware cannot fetch - any part at or above 1 GiB
 * (E3/E6), the render server's MAP_CONTIGUOUS blocks have no placement guarantee -
 * is imported, but ADDFB2 of it answers -EINVAL with a tagged "KMS fb FAIL ...
 * why=above_1g" line: never a commit that fails late or scans garbage.
 * ADDFB2 of an import also needs pitch and offset multiples of 64 bytes (what the
 * pool's own BOs get) and the LINEAR modifier (the HVS path here scans no Broadcom
 * UIF/SAND/T-tiled layout).
 * Reply: kms_dumb_resp_t {handle, pitch 0, size, mem = the exporter's OID memref},
 * so MAP_DUMB of the handle maps the exporter's name, and PRIME_EXPORT of it
 * answers that name too (the re-export of an imported GEM object). Importing the
 * same buffer again on the same client returns the same handle and takes no extra
 * reference (DRM: one GEM_CLOSE / DESTROY_DUMB releases it).
 */
#define KMS_HAVE_PRIME_IMPORT 1
#define KMS_IMPORT_NS_KMSBUF  1u   /* == V3DA_IMPORT_NS_KMSBUF */
#define KMS_IMPORT_NS_V3DBUF  2u   /* == V3DA_IMPORT_NS_V3DBUF */
#define KMS_IMPORT_MAX_SIZE   0x10000000u   /* 256 MiB, == V3DA_IMPORT_MAX_SIZE */

typedef struct {
	uint32_t port;          /* exporter's buffer-namespace port */
	uint32_t cache;         /* enum kms_mem_cache of the export (both namespaces: UNCACHED) */
	uint64_t id;            /* object id under that port */
	uint64_t size;          /* bytes (page multiple, <= the export); 0 = the whole export */
	uint32_t ns;            /* KMS_IMPORT_NS_* */
	uint32_t pad;
} kms_prime_import_req_t;

typedef struct {
	uint32_t width, height;
	uint32_t format;        /* KMS_FMT_* */
	uint32_t handle;
	uint32_t pitch;
	uint32_t offset;
	uint64_t modifier;      /* KMS_MOD_LINEAR only */
} kms_addfb2_req_t;

typedef struct {
	uint32_t fb_id;
	uint32_t pad;
} kms_fb_resp_t;

typedef struct {
	uint32_t crtc_id;
	uint32_t fb_id;
	uint32_t flags;         /* KMS_PAGE_FLIP_EVENT (ASYNC: -EINVAL in Stage A) */
	uint32_t pad;
	uint64_t user_data;
	kms_fence_t in_fence;   /* seqno 0 = none */
} kms_page_flip_req_t;

typedef struct {
	uint64_t sequence;      /* vblank count when the flip was accepted */
	uint32_t applied;       /* 1 = handed to the firmware before the reply; 0 = waits for its fence */
	uint32_t apply_us;      /* mailbox call latency when applied */
} kms_flip_resp_t;

/* One plane's complete new state (a flattened DRM atomic request). */
typedef struct {
	uint32_t plane_id;
	uint32_t fb_id;         /* 0 = disable */
	uint32_t crtc_id;
	int32_t crtc_x, crtc_y;
	uint32_t crtc_w, crtc_h;
	uint32_t src_x, src_y, src_w, src_h;   /* 16.16 */
	int32_t zpos;
	uint32_t alpha;         /* 0..0xffff */
	uint32_t rotation;      /* DRM_MODE_ROTATE_0 (1) / ROTATE_180 (4) | REFLECT_X (16) | REFLECT_Y (32) */
	uint32_t pad[2];
	kms_fence_t in_fence;
} kms_atomic_plane_t;

#define KMS_ATOMIC_MAX_PLANES 8u

typedef struct {
	uint32_t flags;         /* KMS_PAGE_FLIP_EVENT | KMS_ATOMIC_TEST_ONLY | NONBLOCK | ALLOW_MODESET */
	uint32_t nplanes;       /* kms_atomic_plane_t in i.data */
	uint64_t user_data;
	uint32_t crtc_id;       /* the CRTC every plane is on (Stage A: one CRTC per commit) */
	uint32_t mode_blob;     /* MODE_ID, 0 = unchanged; must describe the current mode */
	uint32_t active;        /* ACTIVE; 0 = disable all planes (console) */
	uint32_t pad;
} kms_atomic_req_t;

/* NOT the DRM encoding (M3 F2): DRM has _DRM_VBLANK_ABSOLUTE = 0 and
 * _DRM_VBLANK_RELATIVE = 1; this server tests bit 0 as ABSOLUTE. Never pass a DRM
 * `type` through - convert (libdrm-phoenix always sends absolute targets). */
#define KMS_VBL_RELATIVE 0x0u   /* bit 0 clear: sequence is relative to the current count */
#define KMS_VBL_ABSOLUTE 0x1u   /* bit 0 set: sequence is an absolute 64-bit count */
#define KMS_VBL_EVENT    0x4000000u   /* _DRM_VBLANK_EVENT: queue an event instead of blocking */

typedef struct {
	uint32_t crtc_id;
	uint32_t type;          /* KMS_VBL_* */
	uint64_t sequence;
	uint64_t user_data;
} kms_vblank_req_t;

typedef struct {
	uint64_t sequence;
	int64_t time_ns;        /* CLOCK_MONOTONIC-shaped (see the design doc) */
} kms_vblank_resp_t;

typedef struct {
	uint32_t handle;
	uint32_t offset;
	uint32_t len;
	uint32_t pad;
} kms_checksum_req_t;

typedef struct {
	uint32_t sum;           /* FNV-1a 32 as the SERVER reads it */
	uint32_t first_word;
} kms_checksum_resp_t;

typedef struct {
	uint32_t vblanks;       /* since start, CRTC 0 */
	uint32_t vblank_src;
	uint32_t irq_count;
	uint32_t irq_spurious;
	uint32_t flips_applied;
	uint32_t flips_completed;
	uint32_t flips_fence_deferred;
	uint32_t apply_us_max;
	uint16_t apply_errors;   /* backend calls that failed after a commit was accepted */
	uint16_t pad16;
	uint32_t events_queued;
	uint32_t events_dropped;
	uint32_t reads_parked_max;
	uint16_t bos_live;
	uint16_t exports_live;
	uint32_t pool_free_kib;
} kms_stats_t;


/* ------------------------------------------------------------------------- */
/* Envelopes                                                                  */
/* ------------------------------------------------------------------------- */

typedef struct {
	uint32_t magic;         /* KMS_MAGIC */
	uint32_t op;            /* enum kms_op */
	uint32_t flags;
	uint32_t pad;
	union {
		uint8_t raw[48];
		kms_cap_req_t cap;
		kms_obj_req_t obj;
		kms_set_crtc_req_t set_crtc;
		kms_blob_t blob;
		kms_create_dumb_req_t create_dumb;
		kms_handle_req_t handle;
		kms_prime_import_req_t prime_import;
		kms_addfb2_req_t addfb2;
		kms_fb_resp_t fb;
		kms_page_flip_req_t flip;
		kms_atomic_req_t atomic;
		kms_vblank_req_t vblank;
		kms_checksum_req_t checksum;
	} u;
} kms_req_t;

typedef struct {
	int32_t err;            /* 0 or a negative errno */
	uint32_t op;            /* echo */
	union {
		uint8_t raw[56];
		kms_cap_resp_t cap;
		kms_resources_t res;
		kms_connector_t conn;
		kms_encoder_t enc;
		kms_crtc_t crtc;
		kms_plane_res_t plane_res;
		kms_plane_t plane;
		kms_property_t prop;
		kms_blob_t blob;
		kms_dumb_resp_t dumb;
		kms_fb_resp_t fb;
		kms_flip_resp_t flip;
		kms_vblank_resp_t vblank;
		kms_checksum_resp_t checksum;
		kms_stats_t stats;
		uint32_t count;     /* GET_PROPERTIES: elements written/total */
	} u;
} kms_resp_t;


#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(kms_req_t) == 64, "kms_req_t must be exactly msg.i.raw[64]");
_Static_assert(sizeof(kms_resp_t) == 64, "kms_resp_t must be exactly msg.o.raw[64]");
_Static_assert(sizeof(kms_hello_t) <= 48, "HELLO must fit i.raw after the 16-byte ioctl_in_t header");
_Static_assert(sizeof(kms_memref_t) == 24, "memref layout is ABI (== v3da_memref_t)");
_Static_assert(sizeof(kms_fence_t) == 16, "fence layout is ABI (== v3da_fence_t)");
_Static_assert(sizeof(kms_drm_event_vblank_t) == 32, "struct drm_event_vblank is 32 bytes");
_Static_assert(sizeof(kms_drm_event_crtc_sequence_t) == 32, "struct drm_event_crtc_sequence is 32 bytes");
_Static_assert(sizeof(kms_modeinfo_t) == 68, "struct drm_mode_modeinfo is 68 bytes");
_Static_assert(sizeof(kms_stats_t) <= 56, "stats must fit o.raw");
_Static_assert(sizeof(kms_crtc_t) <= 56, "crtc must fit o.raw");
_Static_assert(sizeof(kms_plane_res_t) <= 56, "plane resources must fit o.raw");
_Static_assert(sizeof(kms_resources_t) <= 56, "resources must fit o.raw");
_Static_assert(sizeof(kms_page_flip_req_t) <= 48, "page flip must fit i.raw");
_Static_assert(sizeof(kms_atomic_req_t) <= 48, "atomic header must fit i.raw");
_Static_assert(sizeof(kms_atomic_plane_t) == 80, "atomic plane state layout");
_Static_assert(sizeof(kms_prime_import_req_t) == 32, "PRIME_IMPORT request layout (== v3da_bo_import_req_t)");
_Static_assert(KMS_OP_PRIME_IMPORT == 38, "KMS_OP_PRIME_IMPORT is 38 (drm_phoenix_ext.h history)");
#endif


#endif /* _KMS_PROTO_H_ */
