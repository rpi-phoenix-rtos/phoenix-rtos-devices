/*
 * v3d_phoenix_winsys.c — Phoenix winsys backend for Mesa's v3d gallium driver
 * (GLQuake Path C, Phase 2). The driver talks to the "kernel" only through
 * drmIoctl(fd, DRM_IOCTL_V3D_*, arg); this provides those ioctls on Phoenix using
 * the PROVEN rpi4-v3d-scout primitives (BO=mmap+va2pa, GPU VA via the V3D MMU flat
 * PT, SUBMIT_CL=CT0/CT1 QBA/QEA + FLDONE/FRDONE + L2T flush, GET_PARAM=real
 * V3D-4.2 device info). No DRM, no kernel driver. Synchronous submit (no real
 * fences) -> drmSyncobj* are stubbed elsewhere in the libdrm shim.
 *
 * STATUS: design crystallized from the scout + the confirmed v3d_drm.h struct
 * mapping; pending integration (cross-built libv3d-phoenix.a + a gallium harness)
 * before it can be compiled/tested on HW. The submit/MMU/BO logic mirrors
 * rpi4-v3d-scout.c (HW-proven: render-clear, 4096/4096 px).
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>   /* getpid (M0 concurrent-GPU instrumentation) */
#include <string.h>
#include <time.h>  /* clock_gettime for the flipstat window */
#include <errno.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include "drm-uapi/v3d_drm.h"   /* Mesa's vendored UAPI — same structs the driver uses */

/* V3D 4.2 MMIO (ARM low-peri), HUB + CORE0 — see rpi4-v3d-scout. */
#define V3D_HUB_BASE        0xfec00000u
#define V3D_MMIO_LEN        0x10000u
#define V3D_CORE0_OFFS      0x4000u
/* MMU (HUB-relative) */
#define MMU_PT_PA_BASE      0x1204u
#define MMU_CTL             0x1200u
#define MMU_CTL_ENABLE      (1u<<0)
#define MMU_CTL_PTI_ABORT   (1u<<19)   /* = PT_INVALID_ABORT */
/* Full MMU fault config (mirror linux v3d_mmu_set_page_table). Our prior config set ABORT
 * without PT_INVALID_ENABLE, so PT-invalid detection was effectively OFF — an illegal/unmapped
 * access was neither aborted nor reported (it could hang). Enable detection + abort + INT for
 * PT-invalid, write-violation and cap-exceeded, and arm a scratch page so an illegal access is
 * redirected to harmless memory instead of hanging. */
#define MMU_CTL_PTI_ENABLE      (1u<<16)
#define MMU_CTL_PTI_INT         (1u<<18)
#define MMU_CTL_WRITEVIO_ABORT  (1u<<11)
#define MMU_CTL_WRITEVIO_INT    (1u<<10)
#define MMU_CTL_CAPEXC_ABORT    (1u<<26)
#define MMU_CTL_CAPEXC_INT      (1u<<25)
#define MMUC_CONTROL        0x1000u
#define MMUC_ENABLE         (1u<<0)
#define MMUC_FLUSH          (1u<<1)    /* flush the MMU PTE cache */
#define MMUC_FLUSHING       (1u<<2)    /* set while the PTE-cache flush is in progress */
#define MMU_CTL_TLB_CLEAR   (1u<<2)    /* clear the MMU TLB */
#define MMU_CTL_TLB_CLEARING (1u<<7)   /* set while the TLB clear is in progress */
#define MMU_ILLEGAL_ADDR    0x1230u
#define MMU_ILLEGAL_ENABLE  (1u<<31)
/* The REAL MMU fault-report regs (distinct from MMU_ILLEGAL_ADDR, which is the scratch-page
 * REDIRECT config we program in apply_core_regs — reading it back just echoes scratch_pa>>12).
 * On a page fault the HW latches the faulting VA (as VA>>8, so bytes = value<<8) in VIO_ADDR
 * and the faulting AXI client id in VIO_ID's low byte (cf. Linux v3d_irq.c). */
#define MMU_VIO_ADDR        0x1234u
#define MMU_VIO_ID          0x122cu
#define PTE_W               (1u<<29)
#define PTE_V               (1u<<28)
#define PAGE_SHIFT          12u
/* CORE0-relative submit/sync (see scout) */
#define CTL_INT_STS         0x0050u
#define CTL_INT_CLR         0x0058u
#define INT_FRDONE          (1u<<0)
#define INT_FLDONE          (1u<<1)
#define INT_OUTOMEM         (1u<<2)   /* binner exhausted its tile-allocation pool */
#define INT_CSDDONE         (1u<<7)   /* compute-shader dispatch done (V3D 4.2; V3D_INT_CSDDONE ver<71 = BIT(7)) */
#define INT_QPU_MASK        (0xfffu<<16) /* CTL_INT QPU-interrupt bits 27:16 (Linux V3D_INT_QPU_MASK, v3d_regs.h) */
/* CSD (Compute Shader Dispatch) config/status regs, CORE0-relative (V3D 4.2, ver<71).
 * cfg[0..6] map to CSD_QUEUED_CFG0..6; writing CFG0 KICKS the dispatch. See linux
 * v3d_regs.h (V3D_CSD_*) and v3d_sched.c v3d_csd_job_run. */
#define CSD_STATUS          0x0900u   /* NUM_COMPLETED[11:4], HAVE_CURRENT(1), HAVE_QUEUED(0) */
/* SUBMIT SERIALIZATION.
 *
 * Every path below programs SHARED V3D MMIO (the CT0/CT1 control-list queues, the
 * CSD dispatch queue, the MMU and the cache-control registers) and mutates global
 * state (the BO table, the VA allocator, the page table). None of that was ever
 * guarded, and it is reachable from more than one thread: vkQuake runs its render
 * and compute work through a task system whose workers are DETACHED threads
 * (Quake/tasks.c: SDL_DetachThread(SDL_CreateThread(Task_Worker, ...))), driven from
 * gl_rmain.c / r_world.c.
 *
 * Linux never needs this lock: its kernel driver holds its own, and the DRM
 * scheduler keeps a single job per queue in flight. A poll-based winsys in the
 * client process has neither, so two threads could interleave register writes --
 * pairing one job's control-list address with another job's buffer, or kicking a
 * CSD dispatch into a unit that already had one CURRENT and one QUEUED.
 *
 * The race is as old as the file; what changed is the exposure. libphoenix altered
 * detached-thread exit and stack reclamation on 2026-09-01 (4c97a79, c8ee89e),
 * which shifted the interleaving, and the symptoms appeared in the 08-28..09-03
 * window: CSD timeouts with status=0x7 and presentation frozen at frame 26, plus
 * (suspected) #67's "GPU executing image data".
 *
 * The lock is created on the first ioctl, which happens on the main thread during
 * device creation -- long before any worker thread exists -- so the lazy init is
 * not itself a race in practice. */
static handle_t v3d_submit_lock;
static int v3d_submit_lock_ready;

static void v3d_submit_lock_acquire(void)
{
	if (v3d_submit_lock_ready == 0) {
		if (mutexCreate(&v3d_submit_lock) != 0) {
			fprintf(stderr, "v3d-winsys: mutexCreate failed -- submits UNSERIALIZED\n");
			return;
		}
		v3d_submit_lock_ready = 1;
	}
	(void)mutexLock(v3d_submit_lock);
}

static void v3d_submit_lock_release(void)
{
	if (v3d_submit_lock_ready != 0) {
		(void)mutexUnlock(v3d_submit_lock);
	}
}

#define CSD_STATUS_HAVE_QUEUED  (1u<<0)  /* linux V3D_CSD_STATUS_HAVE_QUEUED_DISPATCH */
#define CSD_STATUS_HAVE_CURRENT (1u<<1)  /* linux V3D_CSD_STATUS_HAVE_CURRENT_DISPATCH */
#define CSD_QUEUED_CFG0     0x0904u   /* CFG1..6 follow at +4 each; CFG0 write kicks the job */
#define CLE_CT0QTS          0x015cu
#define CT0QTS_ENABLE       (1u<<1)
#define CLE_CT0QBA          0x0160u
#define CLE_CT1QBA          0x0164u
#define CLE_CT0QEA          0x0168u
#define CLE_CT1QEA          0x016cu
#define CLE_CT0QMA          0x0170u
#define CLE_CT0QMS          0x0174u
#define CTL_L2CACTL         0x0020u    /* general L2 cache control (V3D 4.x) */
#define L2CACTL_L2CENA      (1u<<0)    /* enable the L2 cache */
#define L2CACTL_L2CCLR      (1u<<2)    /* clear the L2 cache */
#define CTL_L2TFLSTA        0x0034u    /* L2T flush start address */
#define CTL_L2TFLEND        0x0038u    /* L2T flush end address */
#define CTL_L2TCACTL        0x0030u
#define L2TCACTL_L2TFLS     (1u<<0)
#define L2TCACTL_FLM_CLEAN  (2u<<1)    /* FLM field = CLEAN (write back dirty L2T lines to RAM) */
#define L2TCACTL_TMUWCF     (1u<<8)    /* TMU write-combiner flush (drain partial tiled writes) */
#define CTL_SLCACTL         0x0024u    /* slices cache control (V3D 4.x) */
#define SLCACTL_INVAL_ALL   0x0f0f0f0fu /* invalidate TVCCS/TDCCS/UCC(uniform)/ICC(instr) */
#define PTB_BPCA            0x0300u   /* binner primitive-list current address (advances as the binner runs) */
#define PTB_BPCS            0x0304u   /* binner primitive-list current status */
#define PTB_BPOA            0x0308u   /* binner pool overflow address (GPU VA) */
#define PTB_BPOS            0x030cu   /* binner pool overflow size (bytes) */
#define GMP_STATUS          0x0800u   /* global memory protection status (RD/WR_ACTIVE, VIO) */
#define GMP_CFG             0x0804u   /* GMP config (STOP_REQ for safe AXI drain) */
#define GMP_CFG_STOP_REQ    (1u<<1)   /* request the GMP to quiesce outstanding AXI transactions */
#define GMP_STATUS_RD_WR_CNT 0x7f7f0000u /* RD_COUNT(22:16)|WR_COUNT(30:24) — nonzero = txns in flight */
#define GMP_STATUS_CFG_BUSY (1u<<3)
/* HUB block (W.hub[0]): the AXI config. GFXH-1383 — the V3D AXI master must cap its max burst
 * length (MAX_LEN field, bits 3:0). Linux v3d_reset_by_bridge restores it to MAX_LEN_MASK after
 * a bridge SW_INIT; if our power-on path leaves it unset/wrong the AXI can deadlock under
 * sustained load (GMP RD+WR stuck active, binner ct0ca frozen) — the workload-dependent wedge. */
#define HUB_AXICFG          0x0000u
#define HUB_AXICFG_MAX_LEN  0x0000000fu
/* HUB interrupt status/clear (HUB block, see linux v3d_regs.h V3D_HUB_INT_*). The TFU
 * raises HUB_INT bit1 (TFUC = "conversion complete") when a job finishes. HUB_INT_STS
 * (0x50) is the RAW status latch (the separate HUB_INT_MSK_STS at 0x5c gates the CPU IRQ
 * line), so polling STS works without unmasking — we have no V3D IRQ handler. */
#define HUB_INT_STS         0x0050u
#define HUB_INT_CLR         0x0058u
#define HUB_INT_MSK_STS     0x005cu   /* mask status (diagnostic only; STS is raw) */
#define HUB_INT_TFUC        (1u<<1)   /* TFU conversion complete */
#define HUB_INT_TFUF        (1u<<0)   /* TFU conversion failed */
/* Texture Formatting Unit (HUB block, V3D 4.2 / ver<71 register layout — see linux
 * v3d_regs.h V3D_TFU_*(ver) and v3d_sched.c v3d_tfu_job_run). The TFU is a small
 * fixed-function unit that copies a (raster or tiled) source buffer into a tiled/UIF
 * destination image: program the input/output addresses+strides+the format/tiling
 * config word, then write ICFG (with IOC) to kick. Like the CL submit, the addresses
 * here are GPU virtual addresses translated by the V3D MMU (the TFU sits behind it),
 * so Mesa has already folded each BO's GPU-VA base into iia/ioa — we program them as-is.
 * All offsets are HUB-relative (V3D_WRITE in linux targets hub_regs == our W.hub). */
#define TFU_CS              0x0400u   /* control/status: bit0 BUSY, bit31 TFURST */
#define TFU_CS_BUSY         (1u<<0)
#define TFU_ICFG            0x0408u   /* input config (format/tiling/ttype/opad); write kicks */
#define TFU_ICFG_IOC        (1u<<0)   /* raise the done interrupt when the job completes */
#define TFU_IIA             0x040cu   /* input image address (GPU VA) */
#define TFU_ICA             0x0410u   /* input chroma address (GPU VA; 0 for non-planar) */
#define TFU_IIS             0x0414u   /* input image stride */
#define TFU_IUA             0x0418u   /* input u-plane address (GPU VA; 0 for non-planar) */
#define TFU_IOA             0x041cu   /* output image address (GPU VA) + dest tiling format */
#define TFU_IOS             0x0420u   /* output image size: (height<<16)|width */
#define TFU_COEF0           0x0424u   /* YUV coefficient 0 (bit31 USECOEF gates COEF1..3) */
#define TFU_COEF0_USECOEF   (1u<<31)
#define TFU_COEF1           0x0428u
#define TFU_COEF2           0x042cu
#define TFU_COEF3           0x0430u
/* Binner tile-allocation overflow pool. When the binner exhausts the per-job
 * tile_alloc memory Mesa supplies via CT0QMA/QMS, it raises INT_OUTOMEM and stalls
 * until handed a fresh pool via PTB_BPOA/BPOS (linux v3d_overflow_mem_work). The
 * pool need grows with tile count: 1024x768 (~192 tiles) never overflowed, but
 * 1920x1080 (~510 tiles) does. We pre-allocate one generous pool at init and arm it
 * on the first OUTOMEM of each job (Quake's per-frame overflow stays well under this). */
/* Binner overflow (spill) pool. The binner spills per-tile primitive lists here when Mesa's
 * initial tile_alloc (CT0QMA/QMS) is exhausted; a complex 1080p frame can need many MiB. Linux
 * allocates a FRESH 256 KiB BO per OUTOMEM event, UNBOUNDED (v3d_overflow_mem_work) — our prior
 * 4 MiB fixed pool was too small, so a heavy scene exhausted it and the binner wedged EVERY frame
 * (OUTOMEM|SPILLUSE, ovf_armed=exhausted, ~3 fps). 32 MiB covers far more; the servicer hands the
 * whole remaining pool on the first OUTOMEM so the binner has it all at once. (A truly pathological
 * frame exceeding 32 MiB would still need Linux-style unbounded fresh allocation — logged.) */
#define BINOVF_PAGES        8192u     /* 32 MiB persistent binner-overflow/spill pool (8x the old 4 MiB) */
#define BINOVF_CHUNK_BYTES  (BINOVF_PAGES * 4096u)  /* hand the whole pool on the first OUTOMEM */
#define CTL_MISCCFG         0x0018u
#define MISCCFG_OVRTMUOUT   (1u<<0)
#define MISCCFG_QRMAXCNT_SHIFT 1u    /* QRMAXCNT = MISCCFG bits 3:1 (QPU reserve bin-vs-render split) */
/* Tune the binner-coord vs render-frag QPU split. -1 = leave the firmware default (Linux behaviour
 * on V3D 4.2 — it never writes MISCCFG). 0..7 = write MISCCFG=(QRMAXCNT<<1)|0 ONCE at init: low
 * favours the render (frag) shaders, high favours the binner (coord) shaders. The render-stall
 * residual is a balance point — find a value with 0 bin AND 0 render wedges. */
#define V3D_QRMAXCNT        (2)

/* GPU VA space: bump-allocate page-aligned, starting past the null guard. */
#define GPUVA_BASE          0x100000u
/* MMU flat page table size. Each PTE (4 bytes) maps one 4 KiB page, so one 4 KiB
 * PT page = 1024 PTEs = 4 MiB of GPU VA window. A 256x256 RT fits in one page, but
 * a 1024x768 color+depth target (3 MiB each) overflows it -> PTEs written past the
 * table (OOB) at unmapped VAs -> the GPU store lands nowhere -> all-zero RT. Grow to
 * 32 PT pages = 128 MiB of GPU VA, enough for fullscreen color+depth+tile state plus
 * texture/CL working set (Quake). PT must be physically contiguous (the MMU walks it
 * as a flat array from MMU_PT_PA_BASE). */
/* STEP-1 EXPERIMENT (render-stall hunt, task #13): V3D_VA_NO_RECYCLE makes va_alloc
 * never reuse a freed GPU VA (monotonic bump only). Tests whether the intermittent
 * render wedge is caused by a recycled VA whose GPU cache still holds the prior BO's
 * (stale) content — a fresh never-used VA cannot have a stale cache entry. Needs a larger
 * VA window since per-frame BOs are never reclaimed; sized for a short validation run.
 * HELD OFF (advisor 2026-06-21): a 12-boot A/B cannot resolve a 15-30% base rate, and the
 * "zeroing BOs changed wedge content garbage->zeros but NOT the rate" evidence shows CT1
 * overruns ct1ea regardless of memory content -> the wedge is not a memory/VA/cache effect.
 * Pivoted to STEP-3 first: instrument cold-power-on HW state (clock cfg-vs-measured, PLL,
 * temp) and correlate clean-vs-stalled boots for a deterministic discriminator. */
/* EXPERIMENT RUN AND REVERTED (2026-09-03, #67): flipping this to 1 does NOT fix
 * the upload wedge, so VA recycling is not the cause. With recycling off, one
 * wedged job's RCL read back as a perfectly VALID control list (opcodes 0x79,
 * 0x7e, 0x7b, 0x7a, 0x7c, 0x1a, 0x1d STORE_TILE_BUFFER_GENERAL, with addresses
 * inside its own BO) -- while another still read back as RGBA image data at the
 * same address as before the change. So there are (at least) TWO failure modes
 * here, and "the GPU is executing pixels" is not the whole story. Left at 0: the
 * flag doubles the PT to 2 GiB and risks VA exhaustion in long runs, for no
 * measured benefit. */
#define V3D_VA_NO_RECYCLE   0
#if V3D_VA_NO_RECYCLE
#define GPUVA_PT_PAGES      512u   /* 512 * 4 MiB = 2 GiB monotonic VA window (no reclaim) */
#else
/* 256 * 4 MiB = 1 GiB GPU VA window. Was 64 (256 MiB) — enough for Quake's working
 * set but a full SuperTuxKart race exhausts it: STK's deferred-render pipeline (many
 * render targets) + track + all-kart working set, with uncompressed textures (the
 * launcher's --disable-texture-compression, ~4x the DXT size), needs >256 MiB of live
 * GPU VA and faulted (a NULL BO -> EL0 Data Abort). 1 GiB is ~4x the measured need and
 * stays under the 32-bit VA sign bit (GPUVA_BASE 0x100000 + 1 GiB ~= 0x40100000, so no
 * signed-int VA overflow). VAs are still recycled on free (V3D_VA_NO_RECYCLE=0), so this
 * raises the live-set ceiling, not a leak budget. Costs 1 MiB of contiguous PT RAM (vs
 * 256 KiB); the PT mmap fails loudly at init if that contiguous block is unavailable. */
#define GPUVA_PT_PAGES      256u   /* 256 * 4 MiB = 1 GiB GPU VA window */
#endif
#define GPUVA_PT_ENTRIES    (GPUVA_PT_PAGES * (_PAGE_SIZE / 4u))   /* total PTEs */

struct pbo {            /* Phoenix BO */
	uint32_t handle;
	void    *cpu;       /* mmap'd uncached va */
	uintptr_t pa;       /* physical */
	uint32_t gpuva;     /* assigned V3D virtual address (= drm offset) */
	uint32_t size;
	int      used;      /* slot in use (freed by GEM_CLOSE -> reusable) */
	int      scanout;   /* this BO aliases the scanout surface (clear W.scanout_claimed on close) */
	int      cacheable; /* created with V3D_CREATE_BO_CACHEABLE, so b->cpu is NOT MAP_UNCACHED */
	uint32_t nmaps;     /* times MMAP_BO handed this BO's pointer to Mesa (see the corrupt-list report) */
};

/* Freed GPU-VA range, available for reuse. Without this the GPU VA + the BO slot
 * array were monotonic (never reclaimed), so sustained rendering (e.g. Quake demo
 * playback: per-frame CL / tile-state BOs) exhausted both -> the >256th BO indexed
 * past bos[] and the RCL emit wrote through a garbage pointer. Mesa's bufmgr frees
 * BOs (GEM_CLOSE) as its cache evicts, so reclaiming here keeps both bounded. */
struct vahole {
	uint32_t gpuva;
	uint32_t pages;
};

#define MAX_BOS    4096u
#define MAX_HOLES  2048u

static struct {
	volatile uint32_t *hub;   /* V3D regs (HUB base) */
	volatile uint32_t *core0;
	volatile uint32_t *pt;    /* MMU flat page table */
	uintptr_t pt_pa;
	uint32_t next_gpuva;
	uint32_t binovf_gpuva;    /* persistent binner-overflow pool GPU VA (0 = none) */
	uint32_t binovf_bytes;    /* its size in bytes */
	uint32_t binovf_used;     /* bytes of the pool handed out this job (re-armable overflow) */
	uintptr_t scratch_pa;     /* MMU illegal-access scratch page PA (0 = none); redirects faults */
	uint32_t scanout_pa;      /* HDMI framebuffer physical addr / buffer 0 (0 = unavailable) */
	uint32_t scanout_pa2;     /* multi-buffer: buffer 1 PA (= scanout_pa + pitch*phys_h), else 0 */
	uint32_t scanout_pa3;     /* triple-buffer: buffer 2 PA (= scanout_pa + 2*pitch*phys_h), else 0 */
	uint32_t scanout_phys_h;  /* physical (displayed) height — page-flip pans by buffer*phys_h rows */
	uint32_t scanout_bytes;   /* one buffer's byte size (pitch*phys_h) */
	uint32_t scanout_disp_off; /* byte offset (from scanout_pa) of the CURRENTLY-DISPLAYED buffer (last flip) */
	int      scanout_nbuf;    /* number of page-flip buffers granted: 1 (none), 2 (double), 3 (triple) */
	int      scanout_double;  /* convenience: scanout_nbuf >= 2 (page-flip available) */
	int      scanout_claim_idx; /* multi-buffer: which buffer the next scanout BO is backed by */
	int      scanout_claimed; /* single-buffer: only one BO may alias the single scanout surface */
	int      next_scanout;    /* one-shot: back the NEXT ioc_create_bo with the scanout surface
	                           * (set by a client that can't pass V3D_CREATE_BO_SCANOUT through its
	                           * own BO-alloc path, e.g. the V3DV present image). Cleared on use. */
	struct pbo bos[MAX_BOS];
	uint32_t nbos;            /* high-water mark of slots ever used */
	uint32_t next_handle;     /* monotonic BO handle source; handles are never reused */
	struct vahole holes[MAX_HOLES];
	uint32_t nholes;
	int inited;
} W;

static volatile uint32_t *map_dev(uint32_t pa, uint32_t len)
{
	void *p = mmap(NULL, len, PROT_READ|PROT_WRITE,
		MAP_DEVICE|MAP_UNCACHED|MAP_PHYSMEM|MAP_ANONYMOUS, -1, (addr_t)pa);
	return (p==MAP_FAILED) ? NULL : (volatile uint32_t *)p;
}

int v3d_phoenix_powerOn(void);   /* v3d_phoenix_power.c — BCM2711 V3D power-on */
int v3d_phoenix_reset(void);     /* v3d_phoenix_power.c — true reset cycle (hold + power on) */
void v3d_phoenix_logColdState(void);  /* v3d_phoenix_power.c — STEP-3 cold-power-on state probe */

/* Render-timeout counter, exported for the stall-repro harness (rpi4-v3d-stalltest):
 * incremented every time a CT1 render submit hits the spin-timeout. Lets the harness
 * count stalls across many in-boot submits instead of one-boot-per-sample roulette. */
volatile unsigned v3d_phoenix_render_timeouts = 0;

/* V3D_BO_TRACE=1: one line per BO create / mmap / close, so the lifetime of the
 * memory behind a corrupt control list can be reconstructed from a single boot.
 *
 * Needed because DRM_V3D_MMAP_BO hands Mesa `b->cpu` itself and ioc_close_bo USED
 * TO munmap it, so the same CPU address could be handed out twice with a free in
 * between -- the cause of control lists arriving full of texels. Kept after the
 * fix: it is how that class of bug is recognised, and how V3D_KEEP_CLOSED_BO=1
 * A/B runs are read. It is also how the 56 recycled CPU addresses that disproved
 * the W36 premise were counted. */
/* Back to default-off. It was briefly default-on so a fired page could be matched
 * against BO ranges offline, and that cost ~1470 UART lines a run -- during which
 * the event did not fire once in 7 runs, against a 4-in-10 baseline. Rather than
 * spend more runs deciding whether the trace suppresses the bug or that was
 * chance, the attribution moved to c1_seen[] above, which answers the same
 * question from a table, in-process, with no output at all. */
static int bo_trace_on(void)
{
	static int on = -1;

	if (on < 0) {
		const char *e = getenv("V3D_BO_TRACE");
		on = ((e != NULL) && (*e == '1')) ? 1 : 0;
	}
	return on;
}

/* Recently CLOSED BOs, so a corrupt control list can name the BO that previously
 * owned its memory -- in the report itself, at the moment of failure.
 *
 * The full V3D_BO_TRACE log can reconstruct this, but only if corruption happens
 * to occur in a traced boot; the first traced boot recorded none. This keeps the
 * last few closes in memory always, so a single hit answers the question that
 * decides the remaining half of the bug: was this memory or this address just
 * handed back by a different BO?
 *
 * Small, fixed, no allocation. Handles are monotonic now, so a matching handle
 * would itself be a bug worth shouting about. */
#define BO_HIST 16

struct bo_hist_ent {
	uint32_t handle;
	uint32_t gpuva;
	uint32_t size;
	void    *cpu;
};

static struct bo_hist_ent bo_hist[BO_HIST];
static uint32_t bo_hist_next;

static void bo_hist_record(const struct pbo *b)
{
	struct bo_hist_ent *e = &bo_hist[bo_hist_next % BO_HIST];

	e->handle = b->handle;
	e->gpuva = b->gpuva;
	e->size = b->size;
	e->cpu = b->cpu;
	bo_hist_next++;
}

#ifdef V3D_C1_HUNT
/* ---------------------------------------------------------------------------
 * C1 PAGE ATTRIBUTION -- always on, silent, and passive.
 *
 * The allocator's page+4 report knows WHICH page was corrupted but not what that
 * page used to be. This winsys runs in the same process as malloc, so malloc can
 * simply ask it, and answering out of a table costs two stores per BO and not one
 * byte of output.
 *
 * That property is the whole point. Every instrument tried before this one either
 * wrote to the UART (V3D_BO_TRACE, ~1470 lines a run) or withheld pages from the
 * kernel (V3D_KEEP_CLOSED_BO, the closed-BO quarantine), and every one of them
 * SUPPRESSED the event rather than catching it: 0 fires in 8, 5 and 7 runs
 * respectively against a 4-in-10 baseline. Whatever C1 is, it is fragile to
 * perturbation, so an instrument that perturbs nothing is the only kind that can
 * still see it.
 *
 * bo_hist above is not usable for this -- it holds 16 entries, a fraction of the
 * ~300 closes in a run -- so this keeps its own full-run table.
 *
 * malloc declares v3d_c1_lookup_page() weak, so a binary without this driver
 * still links and simply reports the page as unattributable.
 * ------------------------------------------------------------------------- */
#define C1_SEEN_MAX MAX_BOS

static struct {
	void    *cpu;
	uint32_t size;
	uint32_t handle;
	uint32_t closed;    /* 0 = still open, else the ordinal of its close */
} c1_seen[C1_SEEN_MAX];

static uint32_t c1_seen_n, c1_close_ord;


static void c1_seen_create(const struct pbo *b)
{
	if ((c1_seen_n >= C1_SEEN_MAX) || (b->cpu == NULL)) {
		return;
	}
	c1_seen[c1_seen_n].cpu = b->cpu;
	c1_seen[c1_seen_n].size = b->size;
	c1_seen[c1_seen_n].handle = b->handle;
	c1_seen[c1_seen_n].closed = 0u;
	c1_seen_n++;
}


static void c1_seen_close(const struct pbo *b)
{
	uint32_t i;

	c1_close_ord++;
	for (i = c1_seen_n; i > 0u; i--) {
		if (c1_seen[i - 1u].handle == b->handle) {
			c1_seen[i - 1u].closed = c1_close_ord;
			return;
		}
	}
}


/* Was this page ever a BO? Returns the NUMBER of BOs whose CPU range covered it
 * -- more than one means the address was recycled between BOs, which is itself a
 * result -- and fills in the most recent of them. */
int v3d_c1_lookup_page(unsigned long page, unsigned int *handle, unsigned long *off,
	int *closed, unsigned int *total)
{
	uint32_t i, matches = 0u;

	for (i = c1_seen_n; i > 0u; i--) {
		unsigned long lo = (unsigned long)c1_seen[i - 1u].cpu;

		if ((page >= lo) && (page < (lo + c1_seen[i - 1u].size))) {
			if (matches == 0u) {
				if (handle != NULL) *handle = c1_seen[i - 1u].handle;
				if (off != NULL) *off = page - lo;
				if (closed != NULL) *closed = (int)c1_seen[i - 1u].closed;
			}
			matches++;
		}
	}
	if (total != NULL) {
		*total = c1_seen_n;
	}
	return (int)matches;
}


#endif /* V3D_C1_HUNT */


/* ---------------------------------------------------------------------------
 * C1 ATTRIBUTION BY PHYSICAL FRAME (always built; ~4 KiB)
 *
 * WHY A SECOND TABLE, AND WHY PHYSICAL. c1_seen above answers "was this CPU
 * VIRTUAL address inside a BO's mapping". For the recycling hypothesis that is
 * the WRONG QUESTION: when a closed BO's page goes back to the kernel and malloc
 * is handed the frame, malloc maps it at a DIFFERENT virtual address. A VA-keyed
 * table therefore cannot see the recycled-frame case at all -- it only matches
 * when the VA happens to be reused, which is a separate phenomenon.
 *
 * What survives recycling is the PHYSICAL frame, and the physical frame is
 * exactly what malloc can report (hpa). So attribution has to be keyed on it.
 *
 * WHY IT IS SMALL, AND WHY THAT MATTERS MORE THAN IT LOOKS. c1_seen is
 * MAX_BOS(4096) x 24 B = 96 KiB, and a "passive 96 KB BSS table" is already on
 * record in this project as SUPPRESSING C1 -- very likely this one. An instrument
 * that silences the event cannot measure it. This table stores only what the
 * question needs, for CLOSED BOs only:
 *
 *     { first frame, page count, close ordinal } = 8 bytes
 *
 * A run closes ~300 BOs, so 512 entries cover a whole run at 4 KiB -- 1/24th of
 * the footprint, with no loss of coverage for the question being asked.
 *
 * BOs are MAP_CONTIGUOUS (see ioc_create_bo), so a BO is ONE physical range and
 * a single {frame, count} pair describes it exactly. The frame is read out of the
 * GPU page table at close time, immediately before those PTEs are invalidated --
 * the last moment the mapping still exists.
 * ------------------------------------------------------------------------- */
#define C1_CLOSED_N 512u

static struct {
	uint32_t pfn;      /* first physical page number (PA >> PAGE_SHIFT) */
	uint16_t npages;
	uint16_t ord;      /* 1-based close ordinal; 0 = slot never used */
} c1_closed[C1_CLOSED_N];

static uint32_t c1_closed_next, c1_closed_ord;


static void c1_closed_record(uint32_t pfn, uint32_t npages)
{
	uint32_t i = c1_closed_next % C1_CLOSED_N;

	c1_closed_ord++;
	c1_closed[i].pfn = pfn;
	c1_closed[i].npages = (npages > 0xffffu) ? 0xffffu : (uint16_t)npages;
	c1_closed[i].ord = (uint16_t)(c1_closed_ord & 0xffffu);
	c1_closed_next++;
}


/* Was this PHYSICAL address inside a BO we closed? Returns the number of matching
 * records (>1 means the frame has been recycled through more than one BO, which is
 * itself worth knowing); *ord receives the MOST RECENT close ordinal and *total the
 * number of closes recorded, so "0 matches out of 300 closes" reads as a real
 * answer rather than a missing instrument.
 *
 * Declared weak on the malloc side, so a binary without this driver still links. */
int v3d_c1_lookup_pa(unsigned long pa, unsigned int *npages, unsigned int *ord,
	unsigned int *total)
{
	uint32_t i, matches = 0u, best = 0u;
	uint32_t want = (uint32_t)(pa >> PAGE_SHIFT);

	for (i = 0; i < C1_CLOSED_N; i++) {
		if (c1_closed[i].ord == 0u) {
			continue;
		}
		if ((want >= c1_closed[i].pfn) && (want < (c1_closed[i].pfn + c1_closed[i].npages))) {
			if (c1_closed[i].ord >= best) {
				best = c1_closed[i].ord;
				if (npages != NULL) *npages = c1_closed[i].npages;
			}
			matches++;
		}
	}
	if (ord != NULL) *ord = best;
	if (total != NULL) *total = c1_closed_ord;
	return (int)matches;
}



/* Report any recently-closed BO that overlapped this one's GPU VA or CPU address.
 * Overlap, not equality: a smaller BO reusing part of a bigger one's range is the
 * dangerous case and an equality test would miss it. */
/* Report the LIVE BOs adjacent to this one in GPU VA space.
 *
 * The closed-BO history above answered its question with a clean negative: across
 * a full bench it never fired, so no recently-closed BO overlaps a corrupt list.
 * What remains is an out-of-bounds write from a neighbour that is still OPEN --
 * which fits the evidence exactly: the arena is densely packed with no gaps, a
 * copy RCL uses ~101 bytes of its 4096, and the corruption comes in two flavours,
 * tail-only (a small overrun) and whole-BO (a larger one).
 *
 * This driver has already been bitten by that class once: gpuva_bo_remaining()
 * exists so the CPU UIF tiler "can never write past the dest image BO into an
 * adjacent texture in the contiguous dmammap pool". So print who the neighbours
 * are and how big they are; across enough hits the overrunning role identifies
 * itself. */
static void bo_neighbours_report(const struct pbo *b)
{
	uint32_t i;

	for (i = 0; i < W.nbos; i++) {
		const struct pbo *n = &W.bos[i];

		if (!n->used || (n == b)) {
			continue;
		}
		if ((n->gpuva + n->size) == b->gpuva) {
			fprintf(stderr, "v3d-winsys:   LIVE neighbour BELOW: handle=%u gpuva=0x%08x..0x%08x "
				"size=%u (ends exactly at the corrupt BO)\n",
				n->handle, n->gpuva, n->gpuva + n->size, n->size);
		}
		else if (n->gpuva == (b->gpuva + b->size)) {
			fprintf(stderr, "v3d-winsys:   LIVE neighbour ABOVE: handle=%u gpuva=0x%08x..0x%08x size=%u\n",
				n->handle, n->gpuva, n->gpuva + n->size, n->size);
		}
	}
}

/* Search the WHOLE BO for the remainder of a truncated control list.
 *
 * Measured: 15 of 15 partial-corrupt lists stop at exactly offset 66, right after
 * the second tile's TILE_COORDINATES packet, and everything past it is the BO's
 * previous content. An invariant structural boundary is truncation or diversion of
 * the WRITE, not an overwrite by somebody else -- so the question is no longer
 * "who scribbled on it" but "where did the rest of the list go".
 *
 * In a clean copy RCL the tail carries FLUSH_VCD_CACHE followed by
 * START_ADDRESS_OF_GENERIC_TILE_LIST (0x13 0x14) at offsets 81..82, and ends with
 * END_OF_RENDERING (0x0d). If either turns up elsewhere in the page, the list was
 * written at a shifted offset and this is an addressing bug; if neither appears at
 * all, the emission genuinely stopped early. Two answers, one boot, no guessing. */
static void rcl_find_remainder(const struct pbo *b, uint32_t declared_len)
{
	const uint8_t *p = (const uint8_t *)b->cpu;
	uint32_t limit = b->size;
	uint32_t i;
	uint32_t n_sig = 0;
	uint32_t n_eor = 0;

	if (p == NULL) {
		return;
	}
	if (limit > 4096u) {
		limit = 4096u;   /* one page is plenty and bounds the scan cost */
	}
	for (i = 0; (i + 1u) < limit; i++) {
		if ((p[i] == 0x13u) && (p[i + 1u] == 0x14u)) {
			fprintf(stderr, "v3d-winsys:   RCL tail signature (13 14) found at offset %u "
				"(declared list len %u)\n", i, declared_len);
			n_sig++;
			if (n_sig >= 4u) {
				break;
			}
		}
	}
	for (i = 0; i < limit; i++) {
		if (p[i] == 0x0du) {
			n_eor++;
		}
	}
	if (n_sig != 0u) {
		return;
	}
	fprintf(stderr, "v3d-winsys:   RCL tail signature (13 14) ABSENT from this %u-byte BO "
		"(0x0d bytes seen: %u) -- searching every other live BO\n", limit, n_eor);

	/* Not in this BO. The remaining possibility that fits a boundary landing
	 * mid-loop-iteration is that Mesa's CL grew into a SECOND BO part-way through
	 * emission, leaving exactly the first 66 bytes here while the rest went
	 * elsewhere -- with our submit still describing this BO. If the tail turns up
	 * in another live BO, that is the answer and it is an integration bug, not a
	 * scribble. Bounded: at most one page scanned per BO. */
	{
		uint32_t bi;

		for (bi = 0; bi < W.nbos; bi++) {
			const struct pbo *o = &W.bos[bi];
			const uint8_t *q;
			uint32_t olim;
			uint32_t k;

			if (!o->used || (o == b) || (o->cpu == NULL)) {
				continue;
			}
			q = (const uint8_t *)o->cpu;
			olim = (o->size > 4096u) ? 4096u : o->size;
			for (k = 0; (k + 1u) < olim; k++) {
				if ((q[k] == 0x13u) && (q[k + 1u] == 0x14u)) {
					uint32_t d;

					fprintf(stderr, "v3d-winsys:   *** RCL tail found in a DIFFERENT BO: "
						"handle=%u gpuva=0x%08x size=%u at offset %u (declared len %u)\n",
						o->handle, o->gpuva, o->size, k, declared_len);
					/* The one question left. Mesa writes the list through the pointer
					 * MMAP_BO handed it for the SUBMITTED bo. If that pointer is this
					 * other BO's memory, then we gave two BOs the same CPU mapping and
					 * the bug is ours, right here. If the pointers differ, Mesa wrote
					 * somewhere we did not give it and the bug is above us. */
					fprintf(stderr, "v3d-winsys:   cpu: submitted BO handle=%u -> %p | other BO handle=%u -> %p | %s\n",
						b->handle, b->cpu, o->handle, o->cpu,
						(b->cpu == o->cpu) ? "*** SAME CPU MAPPING -- our bug ***" : "different mappings");
					/* Print that BO's bytes across the SAME window where this one went
					 * bad. Two readings must be told apart and the bytes do it:
					 *  - if 62..80 here hold the packets missing from the offending BO
					 *    (7c coords, 1a END_OF_LOADS, 1d STORE, 1b END_OF_TILE), the
					 *    remainder was written at the SAME offsets into the WRONG BO,
					 *    i.e. the list really was split mid-emission;
					 *  - if instead this BO holds an independent complete list from a
					 *    previous job, that is a long-lived CL BO and the match is
					 *    incidental -- which would make "split" the wrong conclusion. */
					fprintf(stderr, "v3d-winsys:   other BO bytes 56..%u:", (k + 4u));
					for (d = 56u; (d <= (k + 4u)) && (d < olim); d++) {
						fprintf(stderr, " %02x", q[d]);
					}
					fprintf(stderr, "\n");
					return;
				}
			}
		}
	}
	fprintf(stderr, "v3d-winsys:   RCL tail not found in ANY live BO -- the emission truly stopped\n");
}

static void bo_hist_report(const char *label, const struct pbo *b)
{
	uint32_t i;

	for (i = 0; i < BO_HIST; i++) {
		const struct bo_hist_ent *e = &bo_hist[i];
		int va_overlap;
		int cpu_overlap;

		if (e->size == 0u) {
			continue;
		}
		va_overlap = ((b->gpuva < (e->gpuva + e->size)) && (e->gpuva < (b->gpuva + b->size)));
		cpu_overlap = ((b->cpu != NULL) && (e->cpu != NULL) &&
			((char *)b->cpu < ((char *)e->cpu + e->size)) &&
			((char *)e->cpu < ((char *)b->cpu + b->size)));
		if (va_overlap || cpu_overlap) {
			fprintf(stderr, "v3d-winsys: %s this BO (handle=%u gpuva=0x%08x cpu=%p size=%u) REUSES "
				"memory of recently CLOSED handle=%u gpuva=0x%08x cpu=%p size=%u [%s%s]\n",
				label, b->handle, b->gpuva, b->cpu, b->size,
				e->handle, e->gpuva, e->cpu, e->size,
				va_overlap ? "gpuva" : "", cpu_overlap ? (va_overlap ? "+cpu" : "cpu") : "");
		}
	}
}

/* Count of render lists that were ALREADY not a control list when we were handed
 * them (see the entry-time check in ioc_submit_cl). Exported for the same reason
 * as the timeout counter: so a harness can read a rate instead of grepping UART. */
volatile unsigned v3d_phoenix_rcl_bad_at_entry = 0;
static uint32_t va_alloc(uint32_t pages);   /* defined below; used by the init overflow pool */
static void apply_core_regs(void);          /* defined below; used by winsys_init + reset path */

static int winsys_init(void)
{
	if (W.inited) return 0;
	/* M0 instrumentation (concurrent-GPU #13, docs/inprogress/2026-08-22-concurrent-gpu-
	 * v3d-server-feasibility.md): log which PROCESS brings up the GPU winsys. A 2nd GPU
	 * process re-runs this (re-power-on + steals the single MMU_PT_PA_BASE + overlapping VA)
	 * -> the abort. This line + the apply_core_regs one turn the root-cause hypothesis into an
	 * HW observation. Harmless single-process (one line per boot). */
	fprintf(stderr, "v3d-winsys: winsys_init pid=%d (power-on + map regs + install PT)\n",
		(int)getpid());
	/* Power on the V3D ourselves (self-contained; no dependency on a separate
	 * scout process whose concurrent clock-toggle/reset would race our submit and
	 * leave core0 reading 0xdeadbeef). Idempotent. */
	/* Check it. Everything below reads V3D core MMIO, and an MMIO read of an unpowered or
	 * unclocked block NEVER COMPLETES -- with SError masked on this target (TD-10) there is
	 * no abort to take, so the process hangs forever having printed nothing more. That is
	 * exactly how an intermittent launch stall was reaching >=229 s with the log ending on
	 * the cold-state line (2026-09-11). The return was being discarded here. */
	if (v3d_phoenix_powerOn() != 0) {
		fprintf(stderr, "v3d-winsys: V3D power-on FAILED -- refusing to read V3D MMIO "
			"(an unclocked read would hang this process forever)\n");
		return -ENODEV;
	}
	W.hub = map_dev(V3D_HUB_BASE, V3D_MMIO_LEN);
	if (!W.hub) return -ENOMEM;
	W.core0 = W.hub + (V3D_CORE0_OFFS/4);
	/* STEP-3 render-stall discriminator: log the firmware-controlled cold-power-on state
	 * (V3D clock cfg-vs-measured, PLL/clock state, temp, throttle, core voltage, PM_GRAFX)
	 * plus the live V3D core identity. Captured ONCE per boot so a multi-boot run can diff a
	 * stalled boot's line against a clean one for a deterministic separator. CORE0_IDENT0 of
	 * a powered-but-mis-clocked core would read back wrong/0xdeadXXXX. */
	v3d_phoenix_logColdState();
	fprintf(stderr, "v3d-coldstate: CORE0_IDENT0=0x%08x HUB_IDENT1=0x%08x cold_HUB_AXICFG=0x%08x "
		"cold_GMP_STATUS=0x%08x cold_MISCCFG=0x%08x (QRMAXCNT=%u)\n",
		W.core0[0x0000/4], W.hub[0x000c/4], W.hub[HUB_AXICFG/4], W.core0[GMP_STATUS/4],
		W.core0[CTL_MISCCFG/4], (W.core0[CTL_MISCCFG/4] >> 1) & 0x7u);
	/* MMU page table: GPUVA_PT_PAGES contiguous pages = GPUVA_PT_PAGES*4 MiB GPU VA. */
	W.pt = mmap(NULL, GPUVA_PT_PAGES*_PAGE_SIZE, PROT_READ|PROT_WRITE,
		MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
	if (W.pt==MAP_FAILED) return -ENOMEM;
	W.pt_pa = (uintptr_t)va2pa((void*)W.pt);
	for (uint32_t i=0;i<GPUVA_PT_ENTRIES;i++) W.pt[i]=0;
	/* MMU illegal-access scratch page (linux v3d mmu_scratch): one harmless page the MMU
	 * redirects faulting accesses to, armed via MMU_ILLEGAL_ADDR in apply_core_regs. Without it
	 * an illegal access has nowhere to land and can stall the bus. */
	{
		void *sp = mmap(NULL, _PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
		if (sp != MAP_FAILED) {
			memset(sp, 0, _PAGE_SIZE);
			W.scratch_pa = (uintptr_t)va2pa(sp);
		}
	}
	/* NOTE: assumes V3D already powered on (rpi4-v3d-scout v3d_powerOn sequence). */
	/* MMU base/enable, MMUC, + general-L2 clear+enable. The L2C enable mirrors linux
	 * v3d_init_core (L2CACTL = L2CCLR|L2CENA): the QPUs fetch shader instructions/uniforms
	 * through L2C and our init never enabled it (an init-correctness gap regardless; it did
	 * not by itself resolve the intermittent first-frame render stall). Factored into
	 * apply_core_regs() so the in-job reset path can re-establish identical state. */
	apply_core_regs();
	W.next_gpuva = GPUVA_BASE;

	/* Pre-allocate the persistent binner-overflow pool (uncached DMA, like the CL/tile
	 * BOs): contiguous pages mapped into the flat MMU at a stable GPU VA, reused every
	 * frame. The binner writes tile lists here on OUTOMEM; the render reads them back. */
	{
		uint32_t gpuva = va_alloc(BINOVF_PAGES);
		void *cpu = (gpuva == 0) ? MAP_FAILED :
			mmap(NULL, BINOVF_PAGES*_PAGE_SIZE, PROT_READ|PROT_WRITE,
				MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
		if (cpu != MAP_FAILED) {
			/* Zero the pool: Phoenix mmap(MAP_CONTIGUOUS) returns NON-zeroed DRAM (the kernel
			 * zeroes only the vm_object struct, not page contents — unlike Linux's __GFP_ZERO
			 * shmem). The binner writes tile-list next-block pointers into this pool; any slot
			 * it doesn't populate must read 0 (a clean halt), not cold-boot garbage that CT1
			 * would follow as a wild pointer. Root cause of the intermittent render wedge. */
			memset(cpu, 0, BINOVF_PAGES*_PAGE_SIZE);
			for (uint32_t i=0;i<BINOVF_PAGES;i++) {
				uintptr_t ppa = (uintptr_t)va2pa((char*)cpu + (size_t)i*_PAGE_SIZE);
				W.pt[(gpuva>>PAGE_SHIFT)+i] = (uint32_t)(ppa>>PAGE_SHIFT)|PTE_W|PTE_V;
			}
			W.binovf_gpuva = gpuva;
			W.binovf_bytes = BINOVF_PAGES*_PAGE_SIZE;
		}
		else {
			fprintf(stderr, "v3d-winsys: WARN no binner-overflow pool — large RTs may stall\n");
		}
	}

	W.inited = 1;
	return 0;
}

/* Tell the winsys where the HDMI scanout framebuffer lives (PA + byte size). Called by
 * the present layer (which owns /dev/fb0 and queries RPI4FB_GETMODE) before any rendering.
 * A render target backed by this PA lets the GPU raster-store straight to the displayed
 * framebuffer — no glReadPixels/blit/fb0 CPU copies (render-to-scanout). Kept out of the
 * Mesa-context winsys build's platformctl path (a stale sysroot platform.h there lacks the
 * graphmode.height field), so the caller passes the values it already has. */
void v3d_phoenix_set_scanout(uint32_t pa, uint32_t bytes);
void v3d_phoenix_set_scanout(uint32_t pa, uint32_t bytes)
{
	W.scanout_pa = pa;
	W.scanout_bytes = bytes;
}

/* Scanout init with double-buffer detection (render-stall complete fix). Given the displayed
 * framebuffer (pa, w, h, pitch), probe the firmware's granted VIRTUAL height: if plo allocated
 * >= 2x physical, a second buffer sits at pa + pitch*h and we can page-flip — the GPU renders +
 * resolves to the OFF-screen buffer and we pan the display to it, so the live (displayed) buffer
 * is never GPU-written (no display contention -> no depth/fragment-pipeline stall). Returns 2 if
 * double-buffer is active, 1 if single (caller falls back to the in-place blit-resolve). */
extern unsigned v3d_phoenix_fb_virtual_height(void);
int v3d_phoenix_scanout_init(uint32_t pa, uint32_t w, uint32_t h, uint32_t pitch);
int v3d_phoenix_scanout_init(uint32_t pa, uint32_t w, uint32_t h, uint32_t pitch)
{
	(void)w;
	W.scanout_pa = pa;
	W.scanout_phys_h = h;
	W.scanout_bytes = pitch * h;
	W.scanout_claim_idx = 0;
	W.scanout_claimed = 0;
	unsigned vh = (h != 0u && pitch != 0u) ? v3d_phoenix_fb_virtual_height() : 0u;
	unsigned nbuf = (h != 0u) ? (vh / h) : 1u;   /* how many stacked buffers the firmware granted */
	if (nbuf > 3u) nbuf = 3u;                     /* we use at most 3 (triple-buffer) */
	if (nbuf < 1u) nbuf = 1u;
	W.scanout_nbuf = (int)nbuf;
	W.scanout_double = (nbuf >= 2u);
	W.scanout_pa2 = (nbuf >= 2u) ? (pa + pitch * h) : 0u;        /* buffer 1 directly below buffer 0 */
	W.scanout_pa3 = (nbuf >= 3u) ? (pa + 2u * pitch * h) : 0u;   /* buffer 2 below buffer 1 */
	fprintf(stderr, "v3d-winsys: scanout init pa=0x%08x %ux%u pitch=%u virt_h=%u -> %d buffer(s) %s "
		"(buf1=0x%08x buf2=0x%08x)\n", pa, w, h, pitch, vh, W.scanout_nbuf,
		(nbuf >= 3u) ? "TRIPLE-BUFFER+page-flip" : (nbuf == 2u) ? "DOUBLE-BUFFER+page-flip" : "single (blit-resolve)",
		W.scanout_pa2, W.scanout_pa3);
	return W.scanout_nbuf;
}

/* Page-flip the display to buffer `buf` (0..nbuf-1). Called after resolving into that buffer.
 * Pans the display origin to row buf*phys_h. (Previously hard-coded a 2-position 0/phys_h map
 * from the double-buffer era, so a THIRD buffer was mis-displayed as buffer 1 — buffer 2's
 * renders were never scanned out. Now scales to nbuf.) Records the displayed byte offset so
 * screenshot capture can read exactly the region HDMI shows. */
extern void v3d_phoenix_fb_flip(unsigned yoff);

/* Presented-frame counter.
 *
 * Every GL and Vulkan app on this stack presents through this one function, so a
 * counter here is the only frame-rate instrument we have that is independent of
 * the application. That matters because the alternatives are all unsound:
 *
 *   - An engine's own on-screen readout has to be OCR'd out of an HDMI snapshot,
 *     is a single instant, and mixes "slower code" with "busier frame". A 9 -> 7
 *     fps question on SuperTuxKart went unresolved for two days on exactly that
 *     confound (scene complexity differed 26-65 vs 45-78 KTris between samples).
 *   - SuperTuxKart's --profile-* summary looks authoritative and is not: its
 *     `Number of frames` is incremented in ProfileWorld::update(int ticks), whose
 *     own comment reads "number of physics time steps", so its "Average FPS" is
 *     the PHYSICS TICK RATE and is nearly constant however slowly you render.
 *
 * What this prints is frames actually scanned out, over a wall-clock window, on
 * the UART -- no screenshot, no OCR, same units for every app.
 *
 * Cost: one increment and one clock read per presented frame, i.e. at most a few
 * dozen per second, and one printf per interval. Off with V3D_FLIPSTAT=0; the
 * interval is V3D_FLIPSTAT_MS (default 5000).
 *
 * ⚠ Only counts real page flips. In single-buffer (blit-resolve) mode the whole
 * body below is skipped, so a silent counter means nbuf==1, NOT a stalled app --
 * check the "scanout init" line for the buffer count before reading anything into
 * silence. The glamor X server presents by GPU readback into /dev/fb0 and does
 * not come through here either. */
static unsigned long v3d_flip_total;
static unsigned long v3d_flip_window;
static uint64_t v3d_flip_t0_us;
static int v3d_flipstat = -1;      /* -1 = not yet read from the environment */
static unsigned v3d_flipstat_ms = 5000u;

static uint64_t v3d_flip_now_us(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		return 0;
	}

	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

void v3d_phoenix_flip(int buf);
void v3d_phoenix_flip(int buf)
{
	if (W.scanout_double) {
		if (buf < 0) buf = 0;
		if (buf >= W.scanout_nbuf) buf = W.scanout_nbuf - 1;
		W.scanout_disp_off = (uint32_t)buf * W.scanout_bytes;
		v3d_phoenix_fb_flip((unsigned)buf * W.scanout_phys_h);

		if (v3d_flipstat < 0) {
			const char *e = getenv("V3D_FLIPSTAT");
			const char *m = getenv("V3D_FLIPSTAT_MS");
			v3d_flipstat = (e != NULL && e[0] == '0') ? 0 : 1;
			if (m != NULL) {
				int v = atoi(m);
				if (v > 0) {
					v3d_flipstat_ms = (unsigned)v;
				}
			}
			v3d_flip_t0_us = v3d_flip_now_us();
		}

		v3d_flip_total++;

		if (v3d_flipstat != 0) {
			uint64_t now = v3d_flip_now_us();
			uint64_t dt = now - v3d_flip_t0_us;

			v3d_flip_window++;
			if (dt >= (uint64_t)v3d_flipstat_ms * 1000u) {
				/* Integer centi-fps so this needs no float formatting. dt is in
				 * MICROseconds, so frames-per-second x 100 is frames * 1e8 / dt.
				 * (First cut used 1e5 and printed 0.03 fps for a measured 30 --
				 * caught by running the counter against a known 30 Hz source
				 * before it ever reached the target.) 64-bit throughout: a frame
				 * count large enough to overflow this would be ~1e11 frames. */
				unsigned long cfps = (unsigned long)((v3d_flip_window * 100000000ull + dt / 2u) / dt);
				fprintf(stderr, "v3d-winsys: flipstat %lu frames in %lu ms = %lu.%02lu fps (total %lu)\n",
					v3d_flip_window, (unsigned long)(dt / 1000u),
					cfps / 100u, cfps % 100u, v3d_flip_total);
				v3d_flip_window = 0;
				v3d_flip_t0_us = now;
			}
		}
	}
}

/* Total frames presented since process start, for a caller that wants to compute
 * its own rate over a window it chooses rather than reading the periodic line. */
unsigned long v3d_phoenix_flip_count(void);
unsigned long v3d_phoenix_flip_count(void) { return v3d_flip_total; }

int v3d_phoenix_scanout_double(void);
int v3d_phoenix_scanout_double(void) { return W.scanout_double; }

int v3d_phoenix_scanout_nbuf(void);
int v3d_phoenix_scanout_nbuf(void) { return W.scanout_nbuf ? W.scanout_nbuf : 1; }

/* Capture readback. A render-to-scanout BO aliases its GPU VA to the framebuffer PA, but
 * its GL-resource CPU mapping is a SEPARATE fresh anonymous mmap — so glReadPixels on the
 * scanout FBO reads uninitialized CPU pages (noise) while the display reads the fb PA
 * (correct). For screenshot capture, read the actual framebuffer PA instead. `buf` selects
 * the page-flip buffer (0/1/2 — the just-rendered one at capture time); copies up to
 * min(bytes, one-buffer-size) bytes of native 32bpp scanout pixels (top-to-bottom) into
 * dst. Returns bytes copied, 0 if scanout unavailable. Lazily maps the whole fb region
 * uncached on first use. */
static volatile uint8_t *scanout_cpu;   /* uncached CPU view of the full fb region (nbuf buffers) */
uint32_t v3d_phoenix_scanout_readback(void *dst, int buf, uint32_t bytes);
uint32_t v3d_phoenix_scanout_readback(void *dst, int buf, uint32_t bytes)
{
	uint32_t n, off, total;
	if (W.scanout_pa == 0u || W.scanout_bytes == 0u || dst == NULL)
		return 0u;
	if (scanout_cpu == NULL) {
		void *m;
		total = (uint32_t)(W.scanout_nbuf ? W.scanout_nbuf : 1) * W.scanout_bytes;
		/* Map the physical framebuffer for CPU read. MAP_SHARED is required: without it the
		 * MAP_PHYSMEM|MAP_ANONYMOUS mapping returns fresh private anon pages (noise) instead of
		 * the fb RAM. Matches the rpi4-fb driver's proven fb mapping. */
		m = mmap(NULL, total, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_UNCACHED | MAP_ANONYMOUS | MAP_PHYSMEM, -1, (addr_t)W.scanout_pa);
		if (m == MAP_FAILED)
			return 0u;
		scanout_cpu = (volatile uint8_t *)m;
	}
	/* buf == -1: read the CURRENTLY-DISPLAYED buffer (the region the firmware is scanning out,
	 * as set by the last flip) — exactly what HDMI shows, robust to any buffer-index assumption.
	 * buf >= 0: read that specific buffer. */
	if (buf < 0)
		off = W.scanout_disp_off;
	else {
		if (buf >= (W.scanout_nbuf ? W.scanout_nbuf : 1))
			buf = 0;
		off = (uint32_t)buf * W.scanout_bytes;
	}
	n = (bytes < W.scanout_bytes) ? bytes : W.scanout_bytes;
	memcpy(dst, (const void *)(scanout_cpu + off), n);
	return n;
}

/* One-shot: request that the NEXT BO created be backed by the scanout surface. Lets a client
 * whose BO-alloc path can't set V3D_CREATE_BO_SCANOUT (e.g. V3DV's vkAllocateMemory for a present
 * image) still get a scanout-backed BO: call this immediately before the allocation. */
void v3d_phoenix_set_next_scanout(void);
void v3d_phoenix_set_next_scanout(void)
{
	W.next_scanout = 1;
}

/* Non-destructive peek: is the NEXT BO going to be scanout-backed? The Mesa v3d resource layout
 * decides tiling BEFORE the BO alloc, and the HVS display can only scan a LINEAR (raster) surface —
 * a UIF-tiled scanout RT is read back as horizontal-shred garbage. Mesa's tiling gate can't tell a
 * pure scanout render target from a sampled texture (Mesa tags BOTH with PIPE_BIND_SAMPLER_VIEW —
 * see main/renderbuffer.c), so v3d_resource_create consults this to force the scanout RT to RASTER
 * while leaving sampled textures (e.g. the quake3 lightmap atlas) tiled. Read-only: the flag is
 * cleared later by ioc_create_bo when the alloc actually happens. */
int v3d_phoenix_peek_next_scanout(void);
int v3d_phoenix_peek_next_scanout(void)
{
	return W.next_scanout;
}

/* Handles are NEVER recycled -- see the comment on W.next_handle in ioc_create_bo --
 * so this is a scan by id rather than an index. MAX_BOS is small and this runs on
 * ioctl paths that already take the submit lock, so the cost is irrelevant next to
 * silently resolving a stale handle onto a live, unrelated BO. */
static struct pbo *bo_find(uint32_t handle)
{
	uint32_t i;

	if (handle == 0) {
		return NULL;
	}
	for (i = 0; i < W.nbos; i++) {
		if (W.bos[i].used && (W.bos[i].handle == handle)) {
			return &W.bos[i];
		}
	}
	return NULL;
}

/* Allocate a page-aligned GPU VA range: first-fit a freed hole (so reclaimed VA is
 * reused and the window doesn't grow unboundedly), else bump-allocate past the
 * high-water mark. Returns 0 on exhaustion. */
static uint32_t va_alloc(uint32_t pages)
{
#if !V3D_VA_NO_RECYCLE
	for (uint32_t i = 0; i < W.nholes; i++) {
		if (W.holes[i].pages >= pages) {
			uint32_t va = W.holes[i].gpuva;
			if (W.holes[i].pages == pages) {
				W.holes[i] = W.holes[--W.nholes];   /* remove */
			}
			else {
				W.holes[i].gpuva += pages * _PAGE_SIZE;   /* shrink */
				W.holes[i].pages -= pages;
			}
			return va;
		}
	}
#endif
	if ((W.next_gpuva >> PAGE_SHIFT) + pages > GPUVA_PT_ENTRIES)
		return 0;   /* window exhausted */
	uint32_t va = W.next_gpuva;
	W.next_gpuva += pages * _PAGE_SIZE;
	return va;
}

static void va_free(uint32_t gpuva, uint32_t pages)
{
	for (uint32_t i = 0; i < pages; i++)
		W.pt[(gpuva >> PAGE_SHIFT) + i] = 0;   /* unmap (PT cleared; TLB flushed per submit) */
	if (W.nholes < MAX_HOLES)
		W.holes[W.nholes++] = (struct vahole){ gpuva, pages };
	/* else: VA leaks (bounded); the next-bump path still serves new allocs. */
}

/* ---------------------------------------------------------------------------
 * BO PAGE POOL (V3D_BO_POOL=1)
 *
 * WHY. The one statistically supported fact about issue C1 is that keeping a
 * closed BO's pages out of the kernel's free pool suppresses the corruption:
 * `V3D_KEEP_CLOSED_BO=1` gives 0 fires in 12 runs against 7 in 19 with it unset,
 * Fisher one-tailed p = 0.019, all 31 runs in one byte-identical binary with the
 * arm chosen by `export`. What lands on those pages after they are recycled is
 * still unidentified -- but the lever is real and measured.
 *
 * KEEP_CLOSED_BO itself is not shippable: it never releases anything, and a
 * six-lap run closes ~299 BOs totalling ~96 MiB. This gets the same "pages never
 * go back to the kernel" property with bounded memory, by RECYCLING them inside
 * the driver instead: a closed BO's mapping goes on a size-keyed free list and
 * the next create of that exact size reuses it. Steady state costs the high-water
 * mark of simultaneously-free BO bytes, not the sum of all closes.
 *
 * Two side benefits: it removes an mmap/munmap pair from the per-BO path, and it
 * makes CPU-address recycling explicit and bounded rather than incidental -- today
 * munmap+mmap already hands the same address to a later BO (56 addresses served
 * more than one handle in a single measured run).
 *
 * ⚠ This is a MITIGATION, not a fix: the writer is still unidentified, so this
 * removes the symptom by keeping the pages ours. Documented as such in
 * docs/KNOWN-ISSUES.md (C1).
 * ------------------------------------------------------------------------- */
#define BOPOOL_MAX_ENT   256u
#define BOPOOL_MAX_BYTES (24u * 1024u * 1024u)

static struct {
	void    *cpu;
	uint32_t size;
} boPool[BOPOOL_MAX_ENT];

static uint32_t boPoolN, boPoolBytes, boPoolHiBytes, boPoolHits, boPoolMiss, boPoolSpill, boPoolGives;


static int boPool_on(void)
{
	static int on = -1;

	if (on < 0) {
		const char *e = getenv("V3D_BO_POOL");
		on = ((e != NULL) && (*e == '1')) ? 1 : 0;
		if (on != 0) {
			fprintf(stderr, "v3d-pool: BO page pool ON (<=%u entries, <=%u MiB) -- closed BOs are "
				"recycled in-driver instead of returned to the kernel (C1 mitigation)\n",
				BOPOOL_MAX_ENT, BOPOOL_MAX_BYTES / (1024u * 1024u));
		}
	}
	return on;
}


/* An exact-size mapping from the pool, or NULL. Exact size only: a partial reuse
 * would leave the tail mapped but unaccounted, which is how VA bookkeeping bugs
 * start. */
static void *boPool_take(uint32_t size)
{
	uint32_t i;

	if (boPool_on() == 0) {
		return NULL;
	}
	for (i = boPoolN; i > 0u; i--) {
		if (boPool[i - 1u].size == size) {
			void *cpu = boPool[i - 1u].cpu;

			boPool[i - 1u] = boPool[boPoolN - 1u];
			boPoolN--;
			boPoolBytes -= size;
			boPoolHits++;
			return cpu;
		}
	}
	boPoolMiss++;
	return NULL;
}


/* Returns 1 if the pool took ownership (caller must NOT munmap), 0 otherwise. */
static int boPool_give(void *cpu, uint32_t size)
{
	if ((boPool_on() == 0) || (cpu == NULL) || (size == 0u)) {
		return 0;
	}
	if ((boPoolN >= BOPOOL_MAX_ENT) || ((boPoolBytes + size) > BOPOOL_MAX_BYTES)) {
		/* Over the cap: release it normally. Bounded by construction -- the whole
		 * point is that memory cannot grow without limit. */
		boPoolSpill++;
		return 0;
	}
	boPool[boPoolN].cpu = cpu;
	boPool[boPoolN].size = size;
	boPoolN++;
	boPoolBytes += size;
	if (boPoolBytes > boPoolHiBytes) {
		boPoolHiBytes = boPoolBytes;
	}
	/* Deterministic cadence: the first give and every 128th after it. An earlier
	 * version keyed this on (hits + misses) % 512 sampled inside give(), which can
	 * simply never align -- the pool then looks idle whether or not it is working,
	 * and a silent instrument cannot be asserted. */
	boPoolGives++;
	if ((boPoolGives == 1u) || ((boPoolGives % 128u) == 0u)) {
		fprintf(stderr, "v3d-pool: %u held (%u KiB, peak %u KiB), %u hits / %u misses, %u spilled, "
			"%u given\n",
			boPoolN, boPoolBytes / 1024u, boPoolHiBytes / 1024u, boPoolHits, boPoolMiss,
			boPoolSpill, boPoolGives);
	}
	return 1;
}


static int ioc_create_bo(struct drm_v3d_create_bo *c)
{
	/* A zero-byte BO request (e.g. vkQuake's empty lightstyles buffer) would compute 0 pages
	 * and mmap(len=0) fails with "BO mmap FAILED (0 pages)". Round up to one page so the
	 * handle/GPU-VA are valid (mirrors Mesa always allocating full pages) — a legitimate
	 * 0-size allocation just gets one unused page rather than a spurious -ENOMEM. */
	uint32_t pages = (c->size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	if (pages == 0)
		pages = 1;
	uint32_t slot, gpuva;
	void *cpu;
	uintptr_t pa;

	/* Reclaim a freed slot if any, else extend the high-water mark. */
	for (slot = 0; slot < W.nbos; slot++)
		if (!W.bos[slot].used) break;
	if (slot == W.nbos) {
		if (W.nbos >= MAX_BOS) {
			fprintf(stderr, "v3d-winsys: BO table full (%u)\n", (unsigned)MAX_BOS);
			return -ENOMEM;
		}
		W.nbos++;
	}

	gpuva = va_alloc(pages);
	if (gpuva == 0) {
		fprintf(stderr, "v3d-winsys: GPU VA exhausted (need %u pages; PT window = %u MiB). "
			"Grow GPUVA_PT_PAGES or check for a BO leak.\n", pages, (GPUVA_PT_PAGES*4u));
		return -ENOMEM;
	}
	/* VA-collision detector (render-stall probe, task #13): behavior-neutral check for two LIVE
	 * BOs overlapping the same GPU VA — the only aliasing that could overwrite a live RCL/tile-
	 * list with another BO's content. If any PTE in the new range is already VALID, a previous
	 * BO is still mapped there. Logged, not fixed, so a wedge log can be correlated with a real
	 * collision (vs the fdbgs=EZTEST drain-stall reading, which implies NO corruption). */
	for (uint32_t i = 0; i < pages; i++) {
		if (W.pt[(gpuva>>PAGE_SHIFT)+i] & PTE_V) {
			fprintf(stderr, "v3d-winsys: VA COLLISION new handle gpuva=0x%x page %u already "
				"mapped (PTE=0x%08x) — live-BO overlap\n",
				gpuva, i, W.pt[(gpuva>>PAGE_SHIFT)+i]);
			break;
		}
	}
	/* A SCANOUT BO (flags bit 1 = V3D_CREATE_BO_SCANOUT, set by Mesa for the
	 * full-screen render target) is backed by the HDMI framebuffer's physical pages
	 * instead of fresh DRAM: the GPU raster-stores straight to the displayed surface
	 * (render-to-scanout), eliminating the per-frame glReadPixels/blit/fb0 CPU copies.
	 * The scanout PA is physically contiguous, so map it pa+i. */
	/* Select the scanout buffer this BO should alias, or 0 (not a scanout BO). In DOUBLE-BUFFER
	 * mode the first scanout RT is backed by buffer 0, the second by buffer 1 (the renderer
	 * creates two scanout FBOs and page-flips between them); in single-buffer mode only one RT
	 * may claim the (live) fb. */
	uint32_t sel_pa = 0;
	if (((c->flags & 0x2u) || W.next_scanout) && W.scanout_pa) {
		if (W.scanout_double) {
			if (W.scanout_claim_idx < W.scanout_nbuf)
				sel_pa = (W.scanout_claim_idx == 0) ? W.scanout_pa
				       : (W.scanout_claim_idx == 1) ? W.scanout_pa2
				       : W.scanout_pa3;
		}
		else if (!W.scanout_claimed) {
			sel_pa = W.scanout_pa;
		}
	}
	if (sel_pa) {
		W.next_scanout = 0;
		/* Back the visible rows with the scanout buffer's physical pages. The RT BO is a little
		 * larger than the fb (V3D stores tile-aligned rows — 1088 for a 1080 RT — plus driver
		 * padding); those extra rows are never displayed, so map them to fresh scratch DRAM. */
		uint32_t scanout_pages = W.scanout_bytes / _PAGE_SIZE;
		if (scanout_pages > pages) scanout_pages = pages;
		cpu = mmap(NULL, pages*_PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_CONTIGUOUS|MAP_UNCACHED|MAP_ANONYMOUS, -1, 0);
		if (cpu==MAP_FAILED) { va_free(gpuva, pages); return -ENOMEM; }
		pa = sel_pa;
		for (uint32_t i=0;i<pages;i++) {
			uint32_t pfn = (i < scanout_pages)
				? (sel_pa>>PAGE_SHIFT)+i
				: (uint32_t)((uintptr_t)va2pa((char*)cpu + (size_t)i*_PAGE_SIZE) >> PAGE_SHIFT);
			W.pt[(gpuva>>PAGE_SHIFT)+i] = pfn|PTE_W|PTE_V;
		}
		fprintf(stderr, "v3d-winsys: RT scanout buf%d PA 0x%08x gpuva 0x%x, %u/%u pages "
			"(%u scratch) — %s\n", W.scanout_double ? W.scanout_claim_idx : 0, sel_pa, gpuva,
			scanout_pages, pages, pages-scanout_pages,
			W.scanout_double ? "double-buffer" : "render-to-scanout");
		if (W.scanout_double) W.scanout_claim_idx++; else W.scanout_claimed = 1;
	}
	else {
		/* Scanout was requested one-shot but couldn't be honored (already claimed / no PA);
		 * don't leak the request to a later BO. */
		W.next_scanout = 0;
		/* Default: uncached contiguous DMA memory. A cacheable BO (flags bit 0 =
		 * V3D_CREATE_BO_CACHEABLE, set by Mesa for CPU-read-back-only render targets)
		 * drops MAP_UNCACHED so the CPU readback hits cache; Mesa invalidates it
		 * (dc ivac) before each read. Keeps MAP_CONTIGUOUS for the flat V3D MMU. */
		int mapflags = MAP_CONTIGUOUS | MAP_ANONYMOUS;
		if ((c->flags & 0x1u) == 0u)
			mapflags |= MAP_UNCACHED;
		/* Only the default uncached kind is pooled: a cacheable BO has different
		 * mapping attributes and must not be handed out as an uncached one. */
		/* A pooled mapping is reused as-is; the zeroing and per-page PT setup below
		 * are shared with the fresh path, so a pooled BO is indistinguishable from a
		 * freshly mmap'd one.
		 *
		 * One implicit dependency changes here and is worth naming: the kernel
		 * clean+invalidates a page when it CREATES an uncached mapping
		 * (_pmap_cacheOpAfterChange), which is what protects an uncached BO from a
		 * previous cacheable owner's dirty lines (#67). A pooled page never gets a
		 * new mapping, so that guarantee is not re-applied -- and does not need to be:
		 * the page never left this uncached mapping, so no cacheable owner can have
		 * touched it in between. */
		cpu = ((c->flags & 0x1u) == 0u) ? boPool_take(pages*_PAGE_SIZE) : NULL;
		if (cpu == NULL) {
			cpu = mmap(NULL, pages*_PAGE_SIZE, PROT_READ|PROT_WRITE, mapflags, -1, 0);
			if (cpu==MAP_FAILED) {
				fprintf(stderr, "v3d-winsys: BO mmap FAILED (%u pages, %u KiB, flags 0x%x)\n",
					pages, (unsigned)(pages*_PAGE_SIZE/1024u), c->flags);
				va_free(gpuva, pages); return -ENOMEM;
			}
		}
		/* Zero freshly-allocated BO memory. Phoenix mmap(MAP_CONTIGUOUS) returns NON-zeroed
		 * DRAM (kernel zeroes only the vm_object struct, not pages — unlike Linux __GFP_ZERO
		 * shmem). Binner-output BOs (tile_alloc / tile_state / CLs) chain per-tile sub-lists via
		 * pointers written into this memory; an unpopulated slot holding cold-boot garbage makes
		 * CT1 branch to a wild address past rcl_end and wedge (the intermittent render stall).
		 * The huge scanout-backed RT (above) is excluded — it is fully rendered each frame and
		 * claimed once, and zeroing 8 MB of uncached fb memory per frame would tank fps. */
		/* NOTE: this used to clean+invalidate the range here, because a page handed
		 * out for an uncached BO could still carry the previous owner's dirty cache
		 * lines and they would be evicted later on top of Mesa's uncached writes --
		 * which corrupted control lists at 64-byte granularity and was the root cause
		 * of #67. That guarantee now lives in the kernel, where every uncached mapping
		 * gets it (hal/aarch64/pmap.c, _pmap_cacheOpAfterChange), so the local
		 * workaround is removed rather than left as a second, silently load-bearing
		 * copy of the same fix.
		 */
		memset(cpu, 0, pages*_PAGE_SIZE);
		pa = (uintptr_t)va2pa(cpu);
		/* Map each page by its ACTUAL physical address rather than assuming pa+i.
		 * MAP_CONTIGUOUS gives contiguous pages for uncached DMA, but a cacheable mapping
		 * may not be physically contiguous — assuming pa+i would map the GPU to the wrong
		 * pages (MMU fault / hang). Per-page va2pa is correct either way. */
		for (uint32_t i=0;i<pages;i++) {
			uintptr_t ppa = (uintptr_t)va2pa((char*)cpu + (size_t)i*_PAGE_SIZE);
			W.pt[(gpuva>>PAGE_SHIFT)+i] = (uint32_t)(ppa>>PAGE_SHIFT)|PTE_W|PTE_V;
		}
	}

	struct pbo *b = &W.bos[slot];
	b->used = 1;
	/* Monotonic, NEVER reused. It used to be slot + 1, which recycles a handle the
	 * moment its slot is reclaimed -- measured in one traced boot: handles 118, 122,
	 * 124, 125, 139 and 284 were each created twice, and handle 122's second life
	 * took 95 more MMAP_BO calls. Any Mesa reference surviving the close therefore
	 * resolved onto a DIFFERENT live BO and wrote into it, which is how a freshly
	 * created control list ends up holding texture data. With a monotonic id, that
	 * same stale reference fails cleanly in bo_find() (-EINVAL) instead. 32 bits at
	 * a few hundred BOs per boot cannot wrap in any plausible session. */
	b->nmaps = 0;
	b->handle = ++W.next_handle;
	b->cpu = cpu; b->pa = pa; b->gpuva = gpuva; b->size = pages*_PAGE_SIZE;
	b->scanout = (W.scanout_pa != 0 && (pa == W.scanout_pa ||
		(W.scanout_pa2 != 0 && pa == W.scanout_pa2) ||
		(W.scanout_pa3 != 0 && pa == W.scanout_pa3)));   /* this BO aliases a scanout buffer */
	b->cacheable = ((c->flags & 0x1u) != 0u);
#ifdef V3D_C1_HUNT
	c1_seen_create(b);
#endif
	c->handle = b->handle;
	c->offset = gpuva;          /* V3D address-space offset (nonzero) */
	if (bo_trace_on() != 0) {
		fprintf(stderr, "v3d-bo: CREATE handle=%u gpuva=0x%08x size=%u cpu=%p\n",
			b->handle, b->gpuva, b->size, b->cpu);
	}
	return 0;
}

/* The C1 hunt instruments below are compiled OUT by default, and that is a
 * measured requirement rather than tidiness. C1 is sensitive to binary layout:
 * four separate builds carrying one instrument each produced 0 fires in 24 runs
 * where the uninstrumented binary fires about 1 run in 3, and merely adding
 * c1_seen[]'s 96 KB of BSS is enough to shift every address after it. So master
 * must build the layout that still reproduces the bug; -DV3D_C1_HUNT turns the
 * instruments on, and doing so is itself a new layout that has to be re-baselined.
 * See docs/KNOWN-ISSUES.md (C1). */
#ifdef V3D_C1_HUNT
/* ---------------------------------------------------------------------------
 * CLOSED-BO QUARANTINE  (C1 instrument; V3D_BO_QUARANTINE=1, or =selftest)
 *
 * WHAT IT IS FOR. C1 is a single stray u32 that appears in memory the allocator
 * has handed out -- historically noticed only when it lands on a malloc heap
 * header. The standing hypothesis is that the landing zone is a page this file
 * just returned: GEM_CLOSE -> munmap -> the kernel recycles the page -> malloc
 * hands it out -> a LATE write from the GPU side arrives on it. Evidence for
 * that shape: V3D_KEEP_CLOSED_BO=1 (never recycle) showed 0 fires in 8 runs
 * where the default showed fires.
 *
 * Testing that by fire RATE costs ~14 runs per arm. This instrument answers it
 * in one run instead, by watching the hypothesised dartboard directly: hold each
 * closed BO's pages for a while instead of releasing them immediately, fill them
 * with a known pattern, and scan them before they finally go back. Anything that
 * writes a closed BO is then reported with its handle, its offset, its value and
 * how long after the close it arrived -- none of which the rate measurement can
 * give.
 *
 * WHY THE SCAN IS TRUSTWORTHY HERE. Default BOs are MAP_UNCACHED (see
 * ioc_create_bo), so both the fill and the scan go straight to DRAM; there is no
 * CPU cache line of ours to hide a foreign write behind. The two BO kinds that
 * would NOT be trustworthy are skipped at the call site: a cacheable BO (our own
 * dirty lines would mask it) and a scanout BO (its CPU pages are decoupled
 * scratch -- the GPU writes the framebuffer PA, not this mapping).
 *
 * The silence of an instrument that has never been seen to fire means nothing,
 * so V3D_BO_QUARANTINE=selftest plants the exact C1 word in a quarantined BO and
 * the normal scan has to find it.
 * ------------------------------------------------------------------------- */
#define QT_SLOTS        64u
#define QT_MAX_BYTES    (12u * 1024u * 1024u)
#define QT_PATTERN      0xa5a5a5a5u     /* not 0, not 0x80000001, not a plausible pointer */
#define QT_MAX_REPORTS  12u

static struct {
	void    *cpu;
	uint32_t size;
	uint32_t handle;
	uint32_t seq;
} qt_slot[QT_SLOTS];

static uint32_t qt_head, qt_count, qt_seq, qt_bytes, qt_reports, qt_released;
static uint32_t qt_plant_handle;   /* the selftest BO, so its hit is not counted as a finding */
static uint32_t qt_plant_seen;     /* and so a scan that never found it is loud */

/* Back to DEFAULT-OFF after its first five runs, and the reason is worth keeping.
 *
 * The quarantine answered its own question: across 5 runs it held and fully
 * scanned ~1000 closed BOs and found NOTHING written to them, with the scan proven
 * on every run by the planted word. But those same 5 runs produced 0 heap fires
 * against a 4/10 baseline -- i.e. holding the pages SUPPRESSES C1 exactly as
 * V3D_KEEP_CLOSED_BO=1 did (pooled: retention 0/13, default 4/10, Fisher p ~ 0.024).
 *
 * So the instrument cannot see the event it was built to catch: whatever it is
 * needs the pages to go back to the kernel, and turning the quarantine on stops
 * that from happening. Leaving it on would spend Pi time measuring silence. The
 * question moves to attributing a FIRED page, which needs a firing run, which
 * needs this off. V3D_BO_QUARANTINE=1 re-enables it. */
static int qt_on(void)
{
	static int on = -1;

	if (on < 0) {
		const char *e = getenv("V3D_BO_QUARANTINE");
		on = ((e != NULL) && (*e == '1')) ? 1 : 0;
		if (on != 0) {
			fprintf(stderr, "v3d-qt: closed-BO quarantine ON (%u slots, %u MiB cap, "
				"pattern 0x%08x)\n", QT_SLOTS, QT_MAX_BYTES / (1024u * 1024u), QT_PATTERN);
		}
	}
	return on;
}


/* Scan one held BO for any word that is no longer the pattern. Does not release it. */
static void qt_scan(uint32_t idx)
{
	volatile uint32_t *p = (volatile uint32_t *)qt_slot[idx].cpu;
	uint32_t words = qt_slot[idx].size / sizeof(uint32_t);
	uint32_t i, hits = 0u;

	for (i = 0u; i < words; i++) {
		if (p[i] == QT_PATTERN) {
			continue;
		}
		/* The selftest word is ours; count and label it separately so it can never
		 * be read as a finding, and so its ABSENCE is loud. */
		if ((qt_slot[idx].handle == qt_plant_handle) && (i == 1u) && (p[i] == 0x80000001u)) {
			fprintf(stderr, "v3d-qt: SELFTEST PLANT found at +4 of handle=%u -- "
				"the scan works on this run\n", qt_slot[idx].handle);
			qt_plant_seen = 1u;
			p[i] = QT_PATTERN;
			continue;
		}
		hits++;
		if (qt_reports < QT_MAX_REPORTS) {
			uint32_t off = i * (uint32_t)sizeof(uint32_t);
			uint32_t lo = (i >= 4u) ? (i - 4u) : 0u;
			uint32_t hi = ((i + 5u) < words) ? (i + 5u) : words;
			uint32_t k;

			qt_reports++;
			fprintf(stderr, "v3d-qt: STRAY WRITE into CLOSED BO handle=%u size=%u "
				"off=0x%x (page %u, +0x%x) value=0x%08x closes-since=%u\n",
				qt_slot[idx].handle, qt_slot[idx].size, off,
				(unsigned)(off / _PAGE_SIZE), (unsigned)(off & (_PAGE_SIZE - 1u)),
				p[i], qt_seq - qt_slot[idx].seq);
			/* The discriminator: an isolated store, or a run of foreign words? */
			for (k = lo; k < hi; k++) {
				fprintf(stderr, "v3d-qt:    w[%+d] = 0x%08x%s\n", (int)(k - i), p[k],
					(p[k] == QT_PATTERN) ? "" : "  <-- not pattern");
			}
		}
	}
	if (hits != 0u) {
		fprintf(stderr, "v3d-qt: handle=%u total %u stray word(s)\n", qt_slot[idx].handle, hits);
	}
}


/* Scan, then hand the pages back to the kernel. */
static void qt_release(uint32_t idx)
{
	qt_scan(idx);
	qt_released++;
	qt_bytes -= qt_slot[idx].size;
	munmap(qt_slot[idx].cpu, qt_slot[idx].size);
	qt_slot[idx].cpu = NULL;

	/* Report from the FIRST release as well as periodically: a run in which the
	 * quarantine never engaged would otherwise print nothing at all, and read
	 * exactly like a run in which it engaged and found nothing. */
	if ((qt_released == 1u) || ((qt_released % 64u) == 0u)) {
		fprintf(stderr, "v3d-qt: released %u closed BOs, %u still held, %u stray report(s)\n",
			qt_released, qt_count - 1u, qt_reports);
	}
}


static void qt_retire_oldest(void)
{
	qt_release((qt_head + QT_SLOTS - qt_count) % QT_SLOTS);
	qt_count--;
}


/* Scan everything still held. Without this the BOs in the ring at exit -- up to
 * QT_SLOTS of them, including the selftest plant if the run closed fewer BOs than
 * the ring holds -- would never be looked at, and a short run would report nothing
 * while appearing to have run. */
static void qt_drain(void)
{
	uint32_t n = qt_count;
	uint32_t i;

	for (i = 0u; i < n; i++) {
		qt_scan((qt_head + QT_SLOTS - qt_count + i) % QT_SLOTS);
	}
	fprintf(stderr, "v3d-qt: EXIT -- %u BOs released, %u scanned at exit, "
		"%u quarantined in total, %u stray report(s), selftest %s\n",
		qt_released, n, qt_seq, qt_reports,
		(qt_plant_seen != 0u) ? "FOUND" : "NOT FOUND -- scan is not trustworthy this run");
}


/* Take ownership of a just-closed BO's mapping. Returns 1 if the quarantine now
 * owns it (the caller must NOT munmap), 0 if the caller should release it itself. */
static int qt_push(struct pbo *b)
{
	volatile uint32_t *p;
	uint32_t words, i;

	if (qt_on() == 0) {
		return 0;
	}
	if ((b->cpu == NULL) || (b->size == 0u)) {
		return 0;
	}

	/* Make room. A BO larger than the whole cap is simply not held. */
	while ((qt_count >= QT_SLOTS) || ((qt_bytes + b->size) > QT_MAX_BYTES)) {
		if (qt_count == 0u) {
			return 0;
		}
		qt_retire_oldest();
	}

	p = (volatile uint32_t *)b->cpu;
	words = b->size / sizeof(uint32_t);
	for (i = 0u; i < words; i++) {
		p[i] = QT_PATTERN;
	}

	qt_seq++;
	if (qt_seq == 1u) {
		/* Plant on the FIRST quarantined BO, and plant on EVERY run rather than
		 * behind a separate mode. An instrument that has never been seen to fire is
		 * no evidence when it stays quiet, and this one's entire value is that its
		 * silence is meaningful -- so each run carries its own proof that the scan
		 * works, instead of that proof living in a build from hours earlier.
		 *
		 * The plant is identified by handle so the report says so explicitly and a
		 * reader never has to subtract it by hand. First BO, not a later one: a
		 * short run may close fewer BOs than the ring holds. qt_drain() guarantees
		 * it is scanned even if the ring never cycles. */
		if (words > 2u) {
			p[1] = 0x80000001u;   /* the exact C1 word, at the offset it is seen at */
			qt_plant_handle = b->handle;
			fprintf(stderr, "v3d-qt: selftest -- planted 0x80000001 at +4 of handle=%u; "
				"the scan must report it\n", b->handle);
		}
		(void)atexit(qt_drain);
	}

	qt_slot[qt_head].cpu = b->cpu;
	qt_slot[qt_head].size = b->size;
	qt_slot[qt_head].handle = b->handle;
	qt_slot[qt_head].seq = qt_seq;
	qt_head = (qt_head + 1u) % QT_SLOTS;
	qt_count++;
	qt_bytes += b->size;
	return 1;
}


#endif /* V3D_C1_HUNT */


/* DRM core GEM_CLOSE: free the BO so its slot + GPU VA are reclaimed. */
static int ioc_close_bo(struct drm_gem_close *gc)
{
	struct pbo *b = bo_find(gc->handle);
	int was_scanout;
	if (b == NULL) return 0;   /* already gone / never ours */
	was_scanout = b->scanout;  /* captured: the claim-release below clears it */
	/* If the scanout-backed RT is being freed, release the single-claim so the NEXT full-screen
	 * RT can re-acquire scanout backing. Without this, a freed+realloc'd RT silently fell back to
	 * plain DRAM while the present path still expected render-to-scanout -> a frozen screen. */
	if (b->scanout) {
		if (W.scanout_double) {
			if (W.scanout_claim_idx > 0) W.scanout_claim_idx--;
		}
		else {
			W.scanout_claimed = 0;
		}
		b->scanout = 0;
	}
	if (bo_trace_on() != 0) {
		fprintf(stderr, "v3d-bo: CLOSE  handle=%u gpuva=0x%08x size=%u cpu=%p\n",
			b->handle, b->gpuva, b->size, b->cpu);
	}
	bo_hist_record(b);
#ifdef V3D_C1_HUNT
	c1_seen_close(b);
#endif

	/* Invalidate this BO's page-table entries BEFORE the VA and the memory go back.
	 *
	 * The old code freed the VA range and munmap'd the pages but left W.pt still
	 * mapping that VA to them. munmap returns the physical pages to the Phoenix
	 * kernel, which hands them to the next allocation -- so a stale PTE points the
	 * GPU at memory that now belongs to somebody else, and the GPU reads or writes
	 * it SILENTLY. Two ways that bites, both live here:
	 *
	 *  - va_free() returns N pages to the pool and a later, SMALLER BO takes the
	 *    same VA. Its own mapping loop rewrites only the pages it uses; the tail
	 *    pages keep pointing at the recycled memory.
	 *  - anything that still holds the old VA (a pointer left inside a reused
	 *    control list, say) resolves through a translation that is no longer ours.
	 *
	 * With the PTEs invalid instead, either case becomes a REPORTED MMU fault --
	 * MMU_CTL_PTI_ABORT is already enabled -- rather than a quiet read of foreign
	 * data. That matters right now: dropped jobs have been found holding RGBA pixel
	 * data where their control list should be, and this is one of the few paths that
	 * can put a live GPU address on memory owned by something else.
	 *
	 * Safe against a free-while-referenced: submits here are synchronous (the ioctl
	 * spin-waits the job to completion), so no job is in flight when Mesa's bufmgr
	 * evicts a BO. The TLB needs no explicit clear -- every submit path already
	 * calls mmu_flush_tlb() before kicking work. */
	{
		uint32_t first = b->gpuva >> PAGE_SHIFT;
		uint32_t npages = b->size / _PAGE_SIZE;
		uint32_t i;

		/* Record this BO's PHYSICAL range for C1 attribution BEFORE the PTEs go,
		 * because the page table is the only place the driver still knows it. BOs
		 * are MAP_CONTIGUOUS, so the first entry's frame plus the page count
		 * describes the whole BO. */
		if ((npages > 0u) && ((W.pt[first] & PTE_V) != 0u)) {
			/* Mask the PFN FIELD explicitly rather than clearing the two flags we
			 * happen to have named. A PTE is written as (pa >> PAGE_SHIFT) | PTE_W |
			 * PTE_V, so the frame lives in the low 28 bits; ~(PTE_W|PTE_V) would also
			 * carry bits 30-31 through if anything ever set them. */
			c1_closed_record(W.pt[first] & 0x0fffffffu, npages);
		}

		for (i = 0; i < npages; i++) {
			W.pt[first + i] = 0u;   /* !PTE_V => PT_INVALID abort if ever touched */
		}
	}

	va_free(b->gpuva, b->size / _PAGE_SIZE);
	/* DO NOT unmap a closed BO's CPU mapping. This is deliberate and it is the fix
	 * for control lists that arrived at submit already full of RGBA pixel data.
	 *
	 * Mesa's contract, from its own source: v3dv_bo.c:303 takes the pointer
	 * DRM_V3D_MMAP_BO returns and CACHES it on the BO, and v3dv_bo.c:359 states
	 * that the mapping "was not produced by libc mmap(), so there is nothing to
	 * munmap" and merely nulls the field. So Mesa never releases it, and it is
	 * entitled to assume the pointer stays valid. Real DRM agrees: closing a GEM
	 * handle does not invalidate a CPU mapping the process already holds.
	 *
	 * We were the ones breaking that. MMAP_BO hands out `b->cpu` itself, this
	 * function munmap'd it, and the next BO's mmap was handed the same address --
	 * so a write through any pointer Mesa still held landed inside a freshly
	 * created BO. Measured in one traced boot: 5 CPU addresses served more than one
	 * handle.
	 *
	 * Cost, measured rather than guessed: 439 creates per boot total 142 MiB, but
	 * only 224 closes totalling 1.6 MiB -- the big buffers are never closed. So not
	 * recycling costs ~1.6 MiB per process, reclaimed at exit. That is cheap enough
	 * to be the default; silent GPU corruption is not.
	 *
	 * V3D_UNMAP_CLOSED_BO=1 restores the old behaviour, for A/B measurement only.
	 *
	 * ---------------------------------------------------------------------------
	 * CORRECTED 2026-09-12. The premise above -- "Mesa never releases it" -- was
	 * read out of the VULKAN driver (v3dv_bo.c), and it is FALSE for the GL path
	 * that SuperTuxKart uses: v3d_bufmgr.c:220 munmap'd b->cpu on every BO free.
	 *
	 * So the address recycling this function was disabled to prevent never
	 * stopped. Measured on that build, with this unmap OFF: 56 CPU addresses
	 * still served more than one handle in a single 6-lap run (the W36 figure was
	 * 5). We were suppressing the symptom on the wrong side of the contract.
	 *
	 * With Mesa's munmap removed there is exactly ONE owner of the mapping -- us
	 * -- so releasing it here is both correct and necessary. Necessary because
	 * leaving it mapped leaks: the same run closed 299 BOs totalling 96.6 MiB,
	 * accelerating with play (92 closes in the first half, 207 in the second), so
	 * "not unmapping" costs ~2.8 MiB per 100 frames, not the ~1.6 MiB per process
	 * measured back when Mesa was still doing the freeing for us.
	 *
	 * Safe here because GEM_CLOSE is the point at which Mesa has already dropped
	 * the BO (v3d_bo_free() closes immediately after its now-removed unmap, and
	 * v3dv_bo_unmap() nulls bo->map first), and because b->used/b->cpu are
	 * cleared below, so gpuva_to_cpu() stops handing this address out.
	 *
	 * THESE TWO CHANGES SHIP TOGETHER. Re-enabling Mesa's munmap without
	 * disabling this one gives a double munmap: the second call would free a
	 * range the process may have since re-mmap'd for something else.
	 *
	 * V3D_KEEP_CLOSED_BO=1 opts out again, for A/B measurement only. */
	if (b->cpu != NULL) {
		static int keep = -1;

		if (keep < 0) {
			const char *e = getenv("V3D_KEEP_CLOSED_BO");
			keep = (e != NULL && *e == '1') ? 1 : 0;
			if (keep != 0) {
				fprintf(stderr, "v3d-winsys: V3D_KEEP_CLOSED_BO=1 -- not unmapping closed "
					"BOs; expect ~2.8 MiB leaked per 100 frames (A/B only)\n");
			}
		}
		if (keep == 0) {
#ifdef V3D_C1_HUNT
			/* V3D_BO_QUARANTINE holds the pages a while and scans them before they
			 * go back, instead of releasing them straight into the kernel's free
			 * pool. The two kinds it cannot answer for are released as usual. */
			if ((was_scanout != 0) || (b->cacheable != 0) || (qt_push(b) == 0)) {
				/* Recycle in-driver rather than returning the page to the kernel;
				 * falls through to munmap when the pool is off or over its cap. */
				if (boPool_give(b->cpu, b->size) == 0) {
					munmap(b->cpu, b->size);
				}
			}
#else
			(void)was_scanout;
			/* Recycle in-driver rather than returning the page to the kernel;
			 * falls through to munmap when the pool is off or over its cap.
			 * Cacheable BOs are rejected inside boPool_give's caller check below,
			 * so only the default uncached kind is ever pooled. */
			if ((b->cacheable != 0) || (boPool_give(b->cpu, b->size) == 0)) {
				munmap(b->cpu, b->size);
			}
#endif
		}
	}
	b->used = 0;
	b->cpu = NULL;
	b->handle = 0;
	return 0;
}

/* Translate a GPU VA back to the CPU (uncached) pointer of the BO that covers it, or NULL
 * if no live BO maps it. Used only by the render-timeout instrumentation below. */
static void *gpuva_to_cpu(uint32_t gpuva)
{
	for (uint32_t i = 0; i < W.nbos; i++) {
		struct pbo *b = &W.bos[i];
		if (b->used && b->cpu != NULL && gpuva >= b->gpuva && gpuva < b->gpuva + b->size)
			return (char *)b->cpu + (gpuva - b->gpuva);
	}
	return NULL;
}

/* Handle lookup for the diagnostics (const, and does not care about ordering). */
static const struct pbo *bo_find_by_handle_const(uint32_t handle)
{
	uint32_t i;

	for (i = 0; i < W.nbos; i++) {
		if (W.bos[i].used && (W.bos[i].handle == handle)) {
			return &W.bos[i];
		}
	}
	return NULL;
}

/* Same scan, but hand back the BO itself -- the diagnostics need its handle, size
 * and CPU address, not just a translated pointer. */
static const struct pbo *bo_find_covering(uint32_t gpuva)
{
	uint32_t i;

	for (i = 0; i < W.nbos; i++) {
		const struct pbo *b = &W.bos[i];

		if (b->used && (gpuva >= b->gpuva) && (gpuva < (b->gpuva + b->size))) {
			return b;
		}
	}
	return NULL;
}

/* --- Binner-determinism diagnostic (#67 model-geometry glitch) ---------------------------
 * The #67 glitch is a NON-DETERMINISTIC per-frame geometry loss on complete (non-wedged)
 * frames. The open question is WHETHER the binner (CT0) produces a bad/varying per-tile
 * list, or the render (CT1) reads a good list wrongly. This hook checksums the tile_alloc
 * (CT0QMA) region right after the bin's L2T output flush — so a fixed-scene test binary
 * (submit the SAME scene N times) can detect binner non-determinism directly: identical CRCs
 * across submits => binner is deterministic (look at CT1's read path); varying CRCs => the
 * binner itself is non-deterministic. Env-gated (V3D_BIN_CRC=1) so it is a strict no-op for
 * Quake and every normal client — it must never perturb the marginal wedge timing. */
static uint32_t crc32_le(const void *buf, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	uint32_t crc = 0xffffffffu;
	for (uint32_t i = 0; i < len; i++) {
		crc ^= p[i];
		for (int k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)(-(int32_t)(crc & 1u)));
	}
	return ~crc;
}

static int      g_bincrc_on = -1;   /* -1 = not yet read from env */
static uint32_t g_last_qma, g_last_qms, g_last_bincrc, g_bin_seq;
static uint32_t gpuva_bo_remaining(uint32_t gpuva);   /* defined just below */

/* Called from ioc_submit_cl after the binner is done + its L2T output is flushed to RAM
 * (so the uncached CPU read below sees the binner's tile_alloc writes). No-op unless enabled. */
static void bincrc_capture(uint32_t qma, uint32_t qms)
{
	if (g_bincrc_on < 0) {
		const char *e = getenv("V3D_BIN_CRC");
		g_bincrc_on = (e != NULL && e[0] == '1') ? 1 : 0;
	}
	if (!g_bincrc_on || qma == 0 || qms == 0)
		return;
	void *ta = gpuva_to_cpu(qma);
	if (ta == NULL)
		return;
	uint32_t n = gpuva_bo_remaining(qma);   /* never read past the tile_alloc BO */
	if (n > qms) n = qms;
	g_last_qma = qma;
	g_last_qms = qms;
	g_last_bincrc = crc32_le(ta, n);
	g_bin_seq++;
}

/* Test-binary getter: the tile_alloc CRC + (gpuva,size) of the most recent bin. Returns the
 * monotonic capture sequence (0 = nothing captured / diagnostic disabled). */
uint32_t v3d_phoenix_last_bin_crc(uint32_t *qma, uint32_t *qms, uint32_t *crc);
uint32_t v3d_phoenix_last_bin_crc(uint32_t *qma, uint32_t *qms, uint32_t *crc)
{
	if (qma) *qma = g_last_qma;
	if (qms) *qms = g_last_qms;
	if (crc) *crc = g_last_bincrc;
	return g_bin_seq;
}

/* Bytes remaining in the BO that covers gpuva, from gpuva to the BO's end (0 if none).
 * Used to bounds-check the CPU UIF tiler so it can never write past the dest image BO into
 * an adjacent texture in the contiguous dmammap pool. */
static uint32_t gpuva_bo_remaining(uint32_t gpuva)
{
	for (uint32_t i = 0; i < W.nbos; i++) {
		struct pbo *b = &W.bos[i];
		if (b->used && b->cpu != NULL && gpuva >= b->gpuva && gpuva < b->gpuva + b->size)
			return (b->gpuva + b->size) - gpuva;
	}
	return 0;
}

/* Instrument-validation probe (render-stall, task #13): log EVERY live BO whose VA range
 * contains gpuva — handle, base, size, and (gpuva-base). If >1 matches, gpuva_to_cpu's
 * first-match is ambiguous and any "head is zero" reading is a WRONG-BO artifact (overlapping
 * VA ranges in the BO table even with disjoint PTEs) rather than a real RCL-offset bug. */
static void gpuva_describe(const char *label, uint32_t gpuva)
{
	unsigned matches = 0;
	for (uint32_t i = 0; i < W.nbos; i++) {
		struct pbo *b = &W.bos[i];
		if (b->used && b->cpu != NULL && gpuva >= b->gpuva && gpuva < b->gpuva + b->size) {
			fprintf(stderr, "v3d-winsys: %s gpuva=0x%08x -> BO handle=%u base=0x%08x size=%u "
				"off=0x%x cpu=%p\n", label, gpuva, b->handle, b->gpuva, b->size,
				(unsigned)(gpuva - b->gpuva), b->cpu);
			matches++;
		}
	}
	if (matches == 0)
		fprintf(stderr, "v3d-winsys: %s gpuva=0x%08x -> NO live BO covers it\n", label, gpuva);
	else if (matches > 1)
		fprintf(stderr, "v3d-winsys: %s gpuva=0x%08x -> %u OVERLAPPING BOs (gpuva_to_cpu ambiguous!)\n",
			label, gpuva, matches);
}

/* =========================================================================
 * Post-wedge control-list observation. DIAGNOSTIC ONLY.
 *
 * Everything below is called exclusively from the `spins == 0` timeout branches
 * of ioc_submit_cl(); a job that completes never enters them, so this code is
 * unreachable on a healthy run and costs a healthy frame nothing.
 *
 * This file is the IN-PROCESS winsys — the path the shipping games actually
 * take (they link it; the rpi4-v3d server is not auto-launched), so this is
 * where docs/KNOWN-ISSUES.md "V3D-binner-wedge" gets its named observation
 * ("at the next recurrence, read CL@ct0ca") answered. The server's twin lives
 * in ../v3d_gpu.c (bo_covering / ca_verdict / wedge_cl_dump); the two print
 * deliberately comparable lines.
 *
 *   - foreign content  => BO aliasing survived the handle-recycling / stale
 *                         dirty-line / single-owner-mapping fixes;
 *   - a valid draw item with FDBGS stall bits and int_qpu=0 => a genuine
 *                         front-end starve.
 * ========================================================================= */

#define WEDGE_CL_BEFORE   32u   /* bytes of context dumped before the parked address */
#define WEDGE_CL_AFTER    64u   /* ... and from it onwards (the parked opcode + successors) */
/* A wedge is the rare event we are trying to capture, so the old "first 3 only" gate threw
 * away exactly the recurrences worth having. It is not unbounded either: one boot recorded
 * 94 CT1 render timeouts (see the fix-A comment in ioc_submit_cl), so keep a generous cap
 * and SAY so when it starts suppressing. */
#define WEDGE_DBG_MAX     64

/* One-word verdict on where a parked CL address sits relative to the SUBMITTED extent
 * [start, end): "inside" (mid-list), "at-end" (the whole list was fetched yet the done
 * interrupt never fired), "before"/"after" (outside the extent but still in the CL's own
 * BO), or "foreign" (a different live BO, or no live BO at all — i.e. the aliasing case). */
static const char *ca_verdict(uint32_t ca, uint32_t start, uint32_t end)
{
	const struct pbo *bca = bo_find_covering(ca);
	const struct pbo *bstart = bo_find_covering(start);

	if (ca == end) {
		return "at-end";
	}
	if ((ca >= start) && (ca < end)) {
		return "inside";
	}
	if ((bca != NULL) && (bca == bstart)) {
		return (ca < start) ? "before" : "after";
	}
	return "foreign";
}

/* Report where `ca` lies and hexdump the control list around it. Only ever dereferences a
 * CPU pointer this winsys itself mmap'd for a live BO (b->cpu); anything else prints a
 * reason and touches no memory. */
static void wedge_cl_dump(const char *tag, uint32_t ca, uint32_t start, uint32_t end)
{
	const struct pbo *b = bo_find_covering(ca);
	const uint8_t *p;
	uint32_t lo, hi, i;
	char bodesc[96];

	if (b != NULL) {
		(void)snprintf(bodesc, sizeof(bodesc), "handle%u@0x%08x+%u off=0x%x nmaps=%u scanout=%d",
			b->handle, b->gpuva, b->size, (unsigned)(ca - b->gpuva), b->nmaps, b->scanout);
	}
	else if ((W.binovf_gpuva != 0u) && (ca >= W.binovf_gpuva) &&
			(ca < (W.binovf_gpuva + W.binovf_bytes))) {
		/* The binner-overflow pool is not a pbo, so bo_find_covering never reports it. */
		(void)snprintf(bodesc, sizeof(bodesc), "none (inside the binner-overflow pool)");
	}
	else {
		(void)snprintf(bodesc, sizeof(bodesc), "none");
	}
	fprintf(stderr, "v3d-winsys: %s ca=0x%08x cl=[0x%08x..0x%08x) where=%s bo=%s\n",
		tag, ca, start, end, ca_verdict(ca, start, end), bodesc);

	if ((b == NULL) || (b->cpu == NULL)) {
		/* Never dereference an address we have not proven is mapped. */
		fprintf(stderr, "v3d-winsys: %s no CPU view of 0x%08x — no live BO maps it\n", tag, ca);
		return;
	}
	if (b->scanout) {
		/* A scanout BO's visible pages are re-pointed at the framebuffer PA in the V3D MMU
		 * while b->cpu keeps its own fresh anonymous DRAM (see ioc_create_bo), so the CPU
		 * mapping does NOT alias what the GPU fetched. Do not guess. */
		fprintf(stderr, "v3d-winsys: %s no CPU view of 0x%08x — scanout BO, the CPU mapping "
			"does not alias the GPU pages\n", tag, ca);
		return;
	}

	/* Window clamped to the BO on BOTH sides (the old cp[-4] could read below the BO start).
	 * CL BOs are allocated uncached (Mesa sets V3D_CREATE_BO_CACHEABLE only for CPU-readback
	 * render targets), so this reads what the GPU reads; a "foreign" hit landing in a
	 * cacheable BO could in principle show a stale CPU-side view. */
	lo = ((ca - b->gpuva) >= WEDGE_CL_BEFORE) ? (ca - WEDGE_CL_BEFORE) : b->gpuva;
	hi = ca + WEDGE_CL_AFTER;
	if (hi > (b->gpuva + b->size)) {
		hi = b->gpuva + b->size;
	}
	p = (const uint8_t *)b->cpu + (lo - b->gpuva);
	fprintf(stderr, "v3d-winsys: %s bytes 0x%08x..0x%08x ('*' marks ca):", tag, lo, hi);
	for (i = lo; i < hi; i++) {
		fprintf(stderr, "%s%02x", (i == ca) ? " *" : " ", p[i - lo]);
	}
	fprintf(stderr, "\n");

	/* The parked opcode on its own line, read AT ca — CLE packets are byte-granular, so a
	 * word-aligned read is the wrong byte whenever ca is unaligned. CLE opcode 0 = HALT, so
	 * a wedge sitting on a zero page / a HALT byte is a different story from one parked
	 * mid-draw. No opcode table here on purpose — decode the bytes above by hand. */
	fprintf(stderr, "v3d-winsys: %s opcode at ca = 0x%02x (0x00 = CLE HALT)\n",
		tag, ((const uint8_t *)b->cpu)[ca - b->gpuva]);
}

/* The CL byte at `ca`, or -1 if no live non-scanout BO with a CPU mapping covers it.
 * Byte-exact: the callers used to mask ca down to a word/16-byte boundary and read THAT
 * byte, which is only the parked opcode when ca happens to be aligned. */
static int wedge_op_at(uint32_t ca)
{
	const struct pbo *b = bo_find_covering(ca);

	if ((b == NULL) || (b->cpu == NULL) || b->scanout) {
		return -1;
	}
	return (int)((const uint8_t *)b->cpu)[ca - b->gpuva];
}

/* GFXH-1897 (Broadcom erratum, see linux v3d_gem.c v3d_clean_caches): a new L2TCACTL
 * flush must not be issued while a previous L2T flush is still in progress, or the new
 * flush malfunctions — the consumer then reads a stale/transient view. Our submit issues
 * L2T flushes back-to-back (bin pre-flush -> render pre-flush -> readback clean) with no
 * wait, so when the render-side flush lands while the bin/binner flush is still busy the
 * render fetches a stale tile-list and wedges (ct1ca parks past rcl_end, FRDONE never
 * fires). Identical demo content stalls ~50% of boots = exactly this flush-completion race.
 * Spin until the L2TFLS busy bit clears before each L2TCACTL write. */
static inline void l2t_flush_wait(volatile uint32_t *c0)
{
	uint32_t spins;
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL/4] & L2TCACTL_L2TFLS); spins--) {}
}

/* Flush the MMU PTE cache + clear the TLB (mirror linux v3d_mmu_flush_all). ioc_create_bo
 * writes fresh PTEs but never invalidates the MMU's cached translations, so a job whose
 * BOs were just mapped at new GPU VAs is fetched through a stale TLB. Both the CL and the
 * TFU submit paths must do this before kicking work that references freshly-created BOs
 * (every texture upload allocates a staging BO + a destination image BO). */
static void mmu_flush_tlb(volatile uint32_t *h)
{
	uint32_t spins;
	h[MMUC_CONTROL/4] = MMUC_FLUSH | MMUC_ENABLE;
	for (spins = 1000000u; spins && (h[MMUC_CONTROL/4] & MMUC_FLUSHING); spins--) {}
	h[MMU_CTL/4] |= MMU_CTL_TLB_CLEAR;
	for (spins = 1000000u; spins && (h[MMU_CTL/4] & MMU_CTL_TLB_CLEARING); spins--) {}
}

/* Re-apply the per-power-on core registers over the (surviving) MMU page table: MMU base +
 * enable, MMUC enable, and the general-L2 clear+enable. Used at init and after an in-job
 * reset. The PT, BO pool and GPU VAs are unchanged — only the V3D's own register state is
 * re-established. */
static void apply_core_regs(void)
{
	/* M0 instrumentation (concurrent-GPU #13): the single global MMU_PT_PA_BASE register is
	 * written here to THIS process's page table. A 2nd GPU process overwrites it -> the 1st
	 * process's BOs are no longer mapped -> MMU illegal-address fault. Logging pid + the PA
	 * base makes the "PT theft" conflict visible on HW. */
	fprintf(stderr, "v3d-winsys: apply_core_regs pid=%d MMU_PT_PA_BASE<=0x%08x\n",
		(int)getpid(), (uint32_t)(W.pt_pa>>PAGE_SHIFT));
	W.hub[MMU_PT_PA_BASE/4] = (uint32_t)(W.pt_pa>>PAGE_SHIFT);
	/* Full MMU fault config (mirror linux v3d_mmu_set_page_table): enable PT-invalid detection
	 * (not just abort), write-violation and cap-exceeded aborts+INTs. Our prior config set ABORT
	 * without the matching ENABLE, leaving detection effectively off so an illegal access could
	 * hang silently. */
	W.hub[MMU_CTL/4] = MMU_CTL_ENABLE | MMU_CTL_PTI_ENABLE | MMU_CTL_PTI_ABORT | MMU_CTL_PTI_INT |
		MMU_CTL_WRITEVIO_ABORT | MMU_CTL_WRITEVIO_INT |
		MMU_CTL_CAPEXC_ABORT | MMU_CTL_CAPEXC_INT;
	/* Arm the illegal-access scratch page: a faulting GPU access is redirected here (harmless
	 * mapped DRAM) instead of stalling the bus — and MMU_ILLEGAL_ADDR then reports the fault. */
	if (W.scratch_pa)
		W.hub[MMU_ILLEGAL_ADDR/4] = (uint32_t)(W.scratch_pa>>PAGE_SHIFT) | MMU_ILLEGAL_ENABLE;
	W.hub[MMUC_CONTROL/4] = MMUC_ENABLE;
	W.core0[CTL_L2CACTL/4] = L2CACTL_L2CCLR | L2CACTL_L2CENA;
	/* Define the L2T flush range as the WHOLE cache (L2TFLSTA=0, L2TFLEND=~0), matching what
	 * the Linux v3d driver programs at core init. Without this, every L2TCACTL flush we issue
	 * covers an indeterminate cold-boot range, so a
	 * bin->render flush can leave stale tile-list lines and the render fetches a stale
	 * next-block pointer -> CT1 wedges. These core regs aren't otherwise written by init or
	 * the reset path, so their cold-boot value persisted across software resets. */
	W.core0[CTL_L2TFLSTA/4] = 0u;
	W.core0[CTL_L2TFLEND/4] = ~0u;
	/* GFXH-1383: cap the V3D AXI master's max burst length. Our power-on path (mailbox +
	 * PM_V3DRSTN + rpivid_asb) never wrote HUB_AXICFG, so it kept whatever cold-boot/bridge
	 * value it had; an unbounded burst length lets the AXI deadlock under sustained load
	 * (GMP RD+WR stuck active, binner ct0ca frozen mid-CL = the workload-dependent wedge).
	 * Linux restores exactly this after every bridge reset. Written here so init AND the
	 * in-job reset path both establish it. */
	W.hub[HUB_AXICFG/4] = HUB_AXICFG_MAX_LEN;
#if (V3D_QRMAXCNT) >= 0
	/* MISCCFG = QRMAXCNT (QPU bin/render split — 2 zeroes both wedge classes here) | OVRTMUOUT.
	 * OVRTMUOUT (TMU uses the shader's texture-output-type field) IS required by our Mesa build —
	 * dropping it broke texture sampling (solid HUD/particles/fonts, wrong colours). Written once
	 * (init + reset path), so QRMAXCNT is NOT clobbered per submit (that was the binner-wedge bug). */
	W.core0[CTL_MISCCFG/4] = ((uint32_t)(V3D_QRMAXCNT) << MISCCFG_QRMAXCNT_SHIFT) | MISCCFG_OVRTMUOUT;
#endif
}

/* Render-stall MITIGATION + probe: how many times a submit recovered a wedged GPU by
 * resetting + retrying. "A reboot clears it" implies an in-process reset clears it too;
 * if so this turns the fatal first-frame render stall into a sub-second hitch. Exported
 * for visibility. */
volatile unsigned v3d_phoenix_render_recoveries = 0;

/* Reset the V3D and re-establish core register state, then return so the caller can
 * re-submit the same job. Re-runs the BCM2711 power-on (clock toggle + RSTN deassert +
 * ASB bridge re-enable) — the same sequence a reboot performs — then re-applies the core
 * regs over the surviving page table. */
/* Best-effort safe AXI drain: ask the GMP to quiesce any outstanding transaction before reset
 * so a stuck RD/WR (the wedge signature: GMP_STATUS RD+WR active) is drained rather than carried
 * across the reset. Mirrors linux v3d_idle_axi (GMP_CFG_STOP_REQ + wait for RD/WR counts +
 * CFG_BUSY to clear). Bounded spin — on a truly wedged AXI it may not drain, which is fine: the
 * subsequent hold-in-reset clears it. */
static void idle_axi(volatile uint32_t *c0)
{
	uint32_t spins;
	c0[GMP_CFG/4] = GMP_CFG_STOP_REQ;
	for (spins = 1000000u; spins; spins--) {
		if ((c0[GMP_STATUS/4] & (GMP_STATUS_RD_WR_CNT | GMP_STATUS_CFG_BUSY)) == 0u)
			break;
	}
}

static void reset_reinit_core(void)
{
	/* The weak powerOn (re-deassert RSTN only) was empirically unable to clear the binner
	 * hang: ct0ca stayed frozen at the same address across 3 resets, and the next frame's
	 * DIFFERENT CL still showed ct0ca stuck there — the AXI/GMP transaction survived the soft
	 * re-deassert. Do a TRUE reset instead: drain the GMP, hold the core in reset + power back
	 * on (v3d_phoenix_reset = asbStop + assert PM_V3DRSTN + powerOn), then re-establish core
	 * regs incl. the GFXH-1383 HUB_AXICFG burst cap. */
	if (W.core0)
		idle_axi(W.core0);
	/* Same hazard as winsys_init, but on the RUNTIME path: apply_core_regs() writes V3D MMIO,
	 * and if the reset left the block unclocked that access never completes -- a forever-hang
	 * mid-render rather than at startup. v3d_phoenix_reset() ends in v3d_phoenix_powerOn(),
	 * which now confirms the clock, so honour its result instead of discarding it. */
	if (v3d_phoenix_reset() != 0) {
		fprintf(stderr, "v3d-winsys: TRUE reset FAILED (clock/ASB not confirmed) -- skipping "
			"core-reg re-init; touching V3D MMIO now would hang this process forever\n");
		return;
	}
	apply_core_regs();
}

/* Harness-only: force a TRUE V3D reset (hold-in-reset + power back on) then re-establish
 * core register state over the surviving page table. Lets rpi4-v3d-stalltest re-create the
 * cold first-frame-after-power-on condition per iteration, so each loop iteration is an
 * independent trial of the intermittent stall — converting one boot into hundreds of
 * samples. Requires winsys already inited (the harness renders one warm-up frame first). */
void v3d_phoenix_harness_reset(void);
void v3d_phoenix_harness_reset(void)
{
	if (v3d_phoenix_reset() != 0) {
		fprintf(stderr, "v3d-winsys: harness reset FAILED (clock/ASB not confirmed) -- "
			"skipping core-reg re-init\n");
		return;
	}
	apply_core_regs();
}

/* The wedge is data-dependent (re-submitting the same frame re-hangs across true resets,
 * HW-confirmed), so the mitigation does NOT re-submit: on a wedge it does ONE true reset to
 * clean the core for the next (different) frame and drops the current frame. See the job_failed
 * handling in ioc_submit_cl. */

static int ioc_submit_cl(struct drm_v3d_submit_cl *s)
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins;
	int job_failed = 0;     /* set if bin or render wedged */

	int attempt = 0;        /* kept for the timeout-dump "attempt" field (always 0 now: no resubmit) */

	/* Drain CPU stores BEFORE reading the list below. Mesa writes control lists
	 * through an uncached (Normal-NC) mapping, where stores may still sit in the
	 * write buffer until a `dsb`. The check that follows reads that memory, so
	 * without this barrier it can report a truncation that is merely a too-early
	 * read -- an instrument bug, and exactly the kind that manufactures a phantom
	 * root cause. The submit path already does this before kicking the GPU (see the
	 * dsb further down); doing it here too costs one barrier per submit and makes
	 * the diagnostic read the same state the hardware will.
	 *
	 * NOTE the underlying finding does not rest on this read alone: the RCLBYTES
	 * dumps taken after a wedge -- seconds later, long past any drain -- show the
	 * same truncated content. */
	__asm__ volatile("dsb sy" ::: "memory");

	/* ENTRY-TIME RCL SANITY CHECK (2026-09-03). Decoding the wedged jobs showed that
	 * only about HALF of them have a control list at all: over
	 * the .log files under artifacts/rpi4b-uart, 65 recorded RCLBYTES dumps split 29 opening with
	 * 0x79 (TILE_RENDERING_MODE_CFG) and 36 opening with RGBA pixel data or zeros,
	 * parked at offset 0. That leaves one question that decides where
	 * the bug is, and it cannot be answered from the wedge dump alone:
	 *
	 *   is the list already garbage when Mesa hands it to us (a BO-lifetime/aliasing
	 *   bug ABOVE this layer), or is it valid at entry and overwritten while the job
	 *   runs (something in OUR submit path, or the GPU, scribbling on it)?
	 *
	 * Two byte reads per submit settle it. Cheap enough to leave on: no allocation, no
	 * MMIO, and the print is capped so a per-frame failure cannot flood the UART.
	 *
	 * 0x79 TILE_RENDERING_MODE_CFG is the normal opener; 0x7c TILE_COORDINATES and
	 * 0x7e TILE_LIST_INITIAL_BLOCK_SIZE are the other legitimate ones. */
	{
		const uint8_t *rp0 = (const uint8_t *)gpuva_to_cpu(s->rcl_start);

		/* >= 8 bytes, because the report below reads first8 -- a shorter list would
		 * over-read past the BO. A real RCL is far longer than 8 bytes anyway. */
		/* Bound the length by what the covering BO actually holds. rcl_end is the
		 * caller's word for where the list ends; trusting it to index rp0 would
		 * over-read past the BO if it were ever wrong -- and a list arriving as
		 * pixel data is exactly the situation where "ever wrong" is on the table. */
		uint32_t avail = gpuva_bo_remaining(s->rcl_start);
		uint32_t want = (uint32_t)(s->rcl_end - s->rcl_start);

		if ((rp0 != NULL) && (want >= 8u) && (avail >= want)) {
			uint32_t rn = want;
			uint8_t op = rp0[0];
			/* A well-formed RCL ENDS in END_OF_RENDERING (0x0d). Checking the tail as
			 * well as the head is what catches PARTIAL corruption -- a list whose first
			 * packets are intact and whose tail has been replaced by pixels. Validated
			 * over every RCLBYTES dump on record: head+tail together flag all 49 corrupt
			 * lists (41 wholly garbage, 8 partial) and none of the 28 intact ones. A
			 * head-only test misses the 8 partial cases entirely. Two byte reads. */
			uint8_t tail = rp0[rn - 1u];
			int head_bad = ((op != 0x79u) && (op != 0x7cu) && (op != 0x7eu));
			int tail_bad = (tail != 0x0du);

			if (head_bad || tail_bad) {
				v3d_phoenix_rcl_bad_at_entry++;
				if (v3d_phoenix_rcl_bad_at_entry <= 8u) {
					fprintf(stderr, "v3d-winsys: RCL IS NOT A LIST AT ENTRY (%s) "
						"gpuva=0x%08x n=%u first8=%02x %02x %02x %02x %02x %02x %02x %02x last=%02x (n_bad=%u)\n",
						head_bad ? (tail_bad ? "head+tail" : "head") : "tail",
						s->rcl_start, rn,
						rp0[0], rp0[1], rp0[2], rp0[3], rp0[4], rp0[5], rp0[6], rp0[7],
						tail, v3d_phoenix_rcl_bad_at_entry);
					{
						const struct pbo *rb = bo_find_covering(s->rcl_start);

						if (rb != NULL) {
							/* Did Mesa ever ASK for a pointer to the BO we are about to
							 * submit? Mesa writes a control list through the address
							 * MMAP_BO returns, so if this count is 0 it never held a
							 * pointer to this memory and cannot have written the list
							 * here -- which would explain a submitted BO containing only
							 * its prior content, with no spill and no second CL BO. */
							fprintf(stderr, "v3d-winsys:   submitted BO handle=%u was MMAP_BO'd %u time(s)%s\n",
								rb->handle, rb->nmaps,
								(rb->nmaps == 0u) ? "  *** NEVER MAPPED -- Mesa never had this pointer ***" : "");
							bo_hist_report("RCL-AT-ENTRY", rb);
							bo_neighbours_report(rb);
							rcl_find_remainder(rb, rn);
						}
						/* The job's own BO list. Mesa passes every BO the job
						 * references, so if its control list spilled into a second
						 * BO that BO is in here -- and this says so directly instead
						 * of being inferred from a signature search. Both truncations
						 * seen so far stop partway through a run of small packets
						 * (offset 66 inside the GFXH-1742 tile loop, offset 128 inside
						 * the supertile-coordinate loop, with the 13th packet's opcode
						 * written and its payload not), which is what a mid-emission BO
						 * switch looks like from here. */
						if ((s->bo_handles != 0u) && (s->bo_handle_count != 0u) &&
								(s->bo_handle_count < 64u)) {
							const uint32_t *hs = (const uint32_t *)(uintptr_t)s->bo_handles;
							uint32_t hi;

							fprintf(stderr, "v3d-winsys:   job references %u BOs:", s->bo_handle_count);
							for (hi = 0; hi < s->bo_handle_count; hi++) {
								const struct pbo *jb = bo_find_by_handle_const(hs[hi]);

								if (jb != NULL) {
									fprintf(stderr, " %u@0x%08x/%u%s", hs[hi], jb->gpuva, jb->size,
										(jb->gpuva == s->rcl_start) ? "(RCL)" : "");
								}
								else {
									fprintf(stderr, " %u@?", hs[hi]);
								}
							}
							fprintf(stderr, "\n");
						}
					}
					if (v3d_phoenix_rcl_bad_at_entry == 8u) {
						fprintf(stderr, "v3d-winsys: (further RCL-at-entry reports suppressed; "
							"read v3d_phoenix_rcl_bad_at_entry for the count)\n");
					}
				}
			}
		}
	}

	W.binovf_used = 0;      /* re-armable binner overflow: reset the per-job hand-out cursor */
	/* Drain CPU stores into uncached GPU BOs to DRAM BEFORE the first GPU MMIO poke below.
	 * On aarch64, writes to Normal-Non-Cacheable (our MAP_UNCACHED BOs) are NOT ordered
	 * before writes to Device (MMIO) memory without an explicit barrier. The per-frame
	 * dynamic-lightmap upload (quakespasm R_UploadLightmap -> glTexSubImage2D ->
	 * v3d_store_tiled_image) CPU-writes fresh texels into the uncached lightmap BO and is
	 * then sampled by THIS submit's render job. Without a drain here the GPU can be kicked
	 * and its TMU fetch the lightmap before the CPU write buffer has reached DRAM -> the
	 * surface samples a half-updated lightmap -> per-frame flicker of dynamically-lit
	 * surfaces (r_dynamic 1; load-dependent, worst during combat). The TFU CPU-tile path
	 * already drains for the same reason (see the barrier in the TFU submit); the
	 * CL/subdata path was missing the symmetric barrier. One dsb per submit — negligible.
	 *
	 * MUST be a `dsb` (completion), NOT `__sync_synchronize()` (aarch64 `dmb ish` = ordering
	 * only). The V3D is a NON-COHERENT external DMA master: it reads these BOs straight from
	 * DRAM, outside the CPU's inner-shareable domain. `dmb` orders the Normal-NC store before
	 * the MMIO kick w.r.t. inner-shareable observers, but does NOT guarantee the store has
	 * DRAINED to the point of coherency the GPU sees before CT0QEA is written. `dsb sy` waits
	 * for completion. The old `dmb` usually worked because the many MMIO writes + spin-waits
	 * between here and the kick gave the write buffer time to drain — but that is timing, not a
	 * guarantee, so a slow-to-drain per-draw uniform/lightmap store could still race the kick =
	 * intermittent flicker of per-frame-updated content (dynamic lightmaps; per-draw model
	 * LightColor). Matches the comment's own stated intent ("one dsb per submit"). */
	__asm__ volatile("dsb sy" ::: "memory");
	/* #67 ORDERING FIX (2026-07-26): issue the fire-and-forget SLCACTL slice-cache invalidate
	 * (TVCCS/TDCCS vertex caches + UCC/ICC) as EARLY as possible — right here, before mmu_flush_tlb
	 * and the L2T flush+waits below — so that every subsequent per-submit spin-wait (mmu_flush_tlb,
	 * the pre-bin L2T flush waits, and fix-A's extra L2T waits) becomes FREE settle latency for it
	 * before the CT0 kick. Root cause of the residual torch/small-model mangle: SLCACTL has no
	 * completion bit on V3D 4.2, and Phoenix's line-~945 `l2t_flush_wait` REMOVES the interlock
	 * Linux relies on (Linux leaves the L2T flush in-flight so the binner hardware-stalls its first
	 * CL read on it, flooring the slice-invalidate settle window); with the wait, SLCACTL at its old
	 * site got only ~5 MMIO writes of settle before the kick and the coordinate-shader vertex fetch
	 * raced it. Confirmed a RACE (not a producer bug) by a 5-boot HW discriminator: VBO source bytes
	 * byte-identical across boots yet torch/monster geometry still mangle-varied. Moving the
	 * invalidate to the front maximizes its settle window using waits already on the path (zero fps
	 * cost); the idle core does not refill the slice caches in the interim, so the earlier drop holds.
	 * See docs/inprogress/2026-07-26-gpu-linux-ordering-analysis.md. */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;
	/* Flush the MMU PTE cache + TLB before the job. ioc_create_bo writes fresh PTEs but
	 * never invalidated the MMU's cached translations, so a job whose CL/RT BOs were just
	 * mapped at new GPU VAs (every Quake frame allocates fresh BOs) is fetched through a
	 * stale TLB -> the render thread reads an unmapped/wrong VA and hangs at RCL packet 0.
	 * (The cube reuses one persistent job at stable VAs, so it never hit this.) Mirrors the
	 * linux v3d_mmu_flush_all sequence: MMUC flush, then MMU_CTL TLB clear, each spin-waited. */
	mmu_flush_tlb(h);
	/* DO NOT write CTL_MISCCFG here. It is {QRMAXCNT[3:1], OVRTMUOUT[0]} — writing OVRTMUOUT
	 * (0x1) every submit CLOBBERED QRMAXCNT (the QPU-reserve-max-count that balances QPUs between
	 * the binner's coordinate shaders and the render's fragment shaders) to 0, which intermittently
	 * starved the coordinate shaders -> the binner wedged with "coordinate-shader QPUs pending"
	 * (the residual CT0 wedge). Linux v3d only writes MISCCFG for ver<41 (and only at init); on
	 * V3D 4.2 it never touches it, leaving the firmware's QRMAXCNT default. OVRTMUOUT is moot on
	 * 4.2 (Mesa sets the TMU output type in the shader/texture state). So leave MISCCFG alone. */
	/* Invalidate the V3D caches before the job: SLCACTL slices (TVCCS/TDCCS/UCC=uniform
	 * cache/ICC=instruction cache) + an L2T flush — matches the scout's v3d_invalidateCaches.
	 * The SLCACTL slices invalidation is essential for multi-frame rendering: without it the
	 * GPU serves stale uniforms from its uniform cache, so per-frame matrix/uniform changes
	 * never render (every frame looks like frame 0). */
	/* Order matches the linux v3d_invalidate_caches "outside-in" sequence (and our own
	 * bin->render handoff at l.916-918): L2T flush (clean+invalidate) FIRST, THEN the SLCACTL
	 * slice-cache invalidate. Doing SLCACTL first (as this used to) risked the read-only slice
	 * caches being dropped while stale lines still sat in the not-yet-flushed L2T; harmless while
	 * the core is idle here, but the contract order is safer and consistent. */
	l2t_flush_wait(c0);                       /* wait-old: prior L2T flush must be idle first */
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);                       /* wait-new: flush must complete before the bin reads its CL/vertex data */
	/* (SLCACTL slice-cache invalidate moved to the FRONT of the submit — see the #67 ORDERING FIX
	 * comment above the dsb. It fires right after the dsb so all these waits are its settle window.) */
	/* "fix-A": an extra waited-L2T-flush before the CT0 kick. Its ORIGINAL purpose (settling the
	 * fire-and-forget SLCACTL slice invalidate) is now served by the #67 ORDERING FIX (SLCACTL
	 * issued at the front of the submit), so this is redundant FOR THAT. BUT a 2026-07-26 attempt
	 * to remove it (to reclaim its latency) REGRESSED: 1 of 3 boots hit 94 CT1 RENDER TIMEOUTs
	 * (ct1ca wedged in a stale per-tile sublist, mmu_ill set) — the marginal binner->render
	 * tile-list wedge. So this waited flush ALSO provides timing margin that suppresses that
	 * SEPARATE render-side wedge, not just the SLCACTL settle. KEEP IT: ~10 boots with it = 0
	 * faults; without it = intermittent render wedges. Removal is NOT a safe fps optimization.
	 * See docs/inprogress/2026-07-24-quake-glitch-coherency-localization.md. */
	l2t_flush_wait(c0);
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);
	/* --- bin (CT0); wait FLDONE --- */
	/* Also clear any latched QPU-interrupt bits (27:16). Linux's IRQ handler clears the
	 * FULL INT status every interrupt (v3d_irq.c: INT_CLR = INT_STS); this poll-based winsys
	 * historically cleared only FLDONE|FRDONE, so QPU bits could stay latched across jobs.
	 * Suspected in the STK in-game render stall (int_sts=0x00ff0000, FRDONE never fires). */
	c0[CTL_INT_CLR/4] = INT_FLDONE|INT_FRDONE|INT_QPU_MASK;
	c0[PTB_BPOS/4] = 0;
	if (s->qma) { c0[CLE_CT0QMA/4]=s->qma; c0[CLE_CT0QMS/4]=s->qms; }
	if (s->qts) { c0[CLE_CT0QTS/4]=CT0QTS_ENABLE|s->qts; }
	c0[CLE_CT0QBA/4]=s->bcl_start; c0[CLE_CT0QEA/4]=s->bcl_end;
	/* Wait for bin done, servicing binner OUT-OF-MEMORY: when the tile_alloc pool
	 * (CT0QMA/QMS) exhausts, the binner raises INT_OUTOMEM and stalls until handed a
	 * fresh pool via PTB_BPOA/BPOS. Arm our persistent overflow pool once per job (OOM
	 * is edge-signaled, so clear INT_OUTOMEM after servicing). Without this, large RTs
	 * (1080p ~510 tiles) hang the binner -> FLDONE never fires -> ~2.5 s spin timeout. */
	{
		int ovf_armed = 0;
		uint32_t sts;
		uint32_t last_ca = c0[0x0110/4];   /* ct0ca — frozen = binner wedged (fast wedge detect) */
		unsigned frozen = 0;
		for (spins=8000000u; spins; spins--) {
			sts = c0[CTL_INT_STS/4];
			if (sts & INT_FLDONE) break;
			/* Re-armable overflow servicer (was single-shot !ovf_armed, which STARVED the binner
			 * on a 2nd OUTOMEM in one job — linux hands a fresh block per event). Hand successive
			 * chunks of the persistent pool on each OUTOMEM until it is exhausted; a frame needing
			 * more than the whole pool logs that it ran dry (a real wedge cause worth growing the
			 * pool for). OUTOMEM is edge-signalled, so clear it after each hand-out. */
			if ((sts & INT_OUTOMEM) && W.binovf_gpuva) {
				if (W.binovf_used < W.binovf_bytes) {
					uint32_t chunk = W.binovf_bytes - W.binovf_used;
					if (chunk > BINOVF_CHUNK_BYTES) chunk = BINOVF_CHUNK_BYTES;
					c0[PTB_BPOA/4] = W.binovf_gpuva + W.binovf_used;
					c0[PTB_BPOS/4] = chunk;
					c0[CTL_INT_CLR/4] = INT_OUTOMEM;
					W.binovf_used += chunk;
					ovf_armed = 1;
					frozen = 0; last_ca = c0[0x0110/4];   /* binner just re-armed; restart frozen window */
				}
				else if (ovf_armed != 2) {
					ovf_armed = 2;   /* pool exhausted — record once; binner will wedge */
					fprintf(stderr, "v3d-winsys: binner overflow pool EXHAUSTED (%u KiB) — grow BINOVF_PAGES\n",
						W.binovf_bytes / 1024u);
				}
			}
			if ((spins & 0xfffffu) == 0u) {            /* sample ct0ca ~every 1M spins (~160 ms) */
				uint32_t ca = c0[0x0110/4];
				if (ca == last_ca) {
					if (++frozen >= 5u) { spins = 0; break; }   /* frozen ~0.8 s -> wedged */
				}
				else { frozen = 0; last_ca = ca; }
			}
		}
		if (spins == 0) {
			job_failed = 1;
			fprintf(stderr, "v3d-winsys: BIN TIMEOUT int_sts=0x%08x ct0cs=0x%08x "
				"ct0ca=0x%08x[%x..%x] gmp=0x%08x gmpvio=0x%08x mmu_ill=0x%08x ovf_armed=%d (attempt %d)\n",
				c0[CTL_INT_STS/4], c0[0x0100/4], c0[0x0110/4], s->bcl_start, s->bcl_end,
				c0[GMP_STATUS/4], c0[0x0808/4], W.hub[MMU_ILLEGAL_ADDR/4], ovf_armed, attempt);
			/* PTB binner pointers + AXICFG localise the hang: BPCA advancing = binner alive but
			 * downstream stalled; BPCA frozen = binner itself wedged. gmp RD/WR-active with
			 * BPCA frozen = stuck AXI transaction (GFXH-1383). */
			fprintf(stderr, "v3d-winsys: BIN PTB bpca=0x%08x bpcs=0x%08x bpoa=0x%08x bpos=0x%08x "
				"hub_axicfg=0x%08x int_qpu=0x%03x\n",
				c0[PTB_BPCA/4], c0[PTB_BPCS/4], c0[PTB_BPOA/4], c0[PTB_BPOS/4],
				W.hub[HUB_AXICFG/4], (c0[CTL_INT_STS/4] >> 16) & 0xfffu);
			/* Instrument-validation probe (same as the render path): which BO backs bcl_start and
			 * ct0ca, at what offset / ambiguous? + full BCL head dump — head-zero vs wrong-BO. */
			{
				/* DIAGNOSTIC ONLY - the timeout branch, so none of this is reachable on a
				 * healthy run. The gate used to be `bdbg++ < 3`, i.e. a FOURTH wedge in a
				 * process printed nothing - and a recurrence is exactly what we are trying
				 * to capture. Generous cap plus an explicit suppression notice instead. */
				static int bdbg = 0;
				if (bdbg < WEDGE_DBG_MAX) {
					uint32_t ca0 = c0[0x0110/4];
					bdbg++;
					/* The REAL fault: VIO_ADDR is VA>>8 → bytes = <<8. Map it to the owning BO
					 * (texture? CL? tile-alloc? or NO BO = unmapped VA / stale PTE). */
					uint32_t vio = W.hub[MMU_VIO_ADDR/4];
					uint32_t fva = vio << 8;
					fprintf(stderr, "v3d-winsys: BIN MMU-VIO vio_addr=0x%08x fault_va=0x%08x "
						"vio_id=0x%08x (mmu_ill printed above = scratch-cfg, NOT the fault)\n",
						vio, fva, W.hub[MMU_VIO_ID/4]);
					gpuva_describe("BINFAULT", fva);
					gpuva_describe("BCLSTART", s->bcl_start);
					gpuva_describe("BINCA", ca0);
					/* Localise the SILENT stall (no MMU/GMP/OOM error): the CLE control regs
					 * (ct0pc frozen = no primitive progress) + the front-end debug/STALL regs
					 * (FDBGS per-stage STALL bits name the wedged sub-unit) + the CL window AT
					 * ct0ca — the BCLFULL dump below starts at bcl_start, but the binner is parked
					 * ~167 words in, so it never shows the actual stall opcode. Offsets confirmed
					 * vs Linux v3d_regs.h (CLE_CT0* 0x100-0x134, FDBG 0xf04-0xf10). */
					fprintf(stderr, "v3d-winsys: BIN CT0 ct0cs=0x%08x ct0lc=0x%08x ct0pc=0x%08x "
						"pcs=0x%08x bfc=0x%08x\n",
						c0[0x0100/4], c0[0x0120/4], c0[0x0128/4], c0[0x0130/4], c0[0x0134/4]);
					fprintf(stderr, "v3d-winsys: BIN FDBG o=0x%08x b=0x%08x r=0x%08x s=0x%08x\n",
						c0[0x0f04/4], c0[0x0f08/4], c0[0x0f0c/4], c0[0x0f10/4]);
					{
						/* Two defects fixed here, both diagnostic-only:
						 *  (1) op= was read at `ca0 & ~3`, but CLE packets are BYTE-granular, so
						 *      every historical op= in a wedge log is only trustworthy when
						 *      ct0ca & 3 == 0. Read the byte AT ct0ca instead.
						 *  (2) the word window ran cp[-4..7] unclamped, i.e. it could read BELOW
						 *      the start of the covering BO. Clamp both ends to that BO.
						 * The printed shape is unchanged so old and new logs still compare.
						 * NOTE this line and the wedge_cl_dump() below share the prefix
						 * "BIN CL@ct0ca" and overlap in content -- grep "CL@ct0ca(" for this
						 * word window, "CL@ct0ca ca=" for the byte dump. */
						const struct pbo *cb = bo_find_covering(ca0);
						int cop = wedge_op_at(ca0);
						if ((cb != NULL) && (cop >= 0)) {
							uint32_t wbase = ca0 & ~3u;
							uint32_t wlo = ((wbase - cb->gpuva) >= 16u) ? (wbase - 16u) : cb->gpuva;
							uint32_t whi = wbase + 32u;
							const uint32_t *wp;
							if (whi > (cb->gpuva + cb->size)) whi = cb->gpuva + cb->size;
							wp = (const uint32_t *)(const void *)((const char *)cb->cpu + (wlo - cb->gpuva));
							fprintf(stderr, "v3d-winsys: BIN CL@ct0ca(0x%08x) op=0x%02x:",
								ca0, (unsigned)cop);
							for (uint32_t w = wlo; w < whi; w += 4u) fprintf(stderr, " %08x", wp[(w - wlo) / 4u]);
							fprintf(stderr, "\n");
						}
						/* Where ct0ca parked relative to the SUBMITTED extent (inside/at-end/before/
						 * after/foreign) + the byte-granular CL window. "foreign" is the answer the
						 * KNOWN-ISSUES observation is after: it means BO aliasing. Prints a refusal
						 * reason rather than dereferencing an address it has not proven is mapped. */
						wedge_cl_dump("BIN CL@ct0ca", ca0, s->bcl_start, s->bcl_end);
					}
					uint32_t bw = (s->bcl_end - s->bcl_start + 3u) / 4u;
					if (bw > 40u) bw = 40u;
					uint32_t *bs = (uint32_t *)gpuva_to_cpu(s->bcl_start);
					fprintf(stderr, "v3d-winsys: BCLFULL gpuva=0x%08x (%u words):", s->bcl_start, bw);
					if (bs) { for (uint32_t i = 0; i < bw; i++) fprintf(stderr, " %08x", bs[i]); }
					else { fprintf(stderr, " (no BO)"); }
					fprintf(stderr, "\n");
				}
				else if (bdbg == WEDGE_DBG_MAX) {
					bdbg++;
					fprintf(stderr, "v3d-winsys: BIN wedge dump SUPPRESSED after %d dumps "
						"(further bin wedges print the TIMEOUT + PTB lines only)\n", WEDGE_DBG_MAX);
				}
			}
		}
	}
	/* If the binner wedged, don't kick the render against a bad tile state — reset+retry. */
	if (job_failed)
		goto job_retry;
	c0[CTL_INT_CLR/4]=INT_FLDONE|INT_FRDONE|INT_QPU_MASK;   /* +QPU bits: Linux-parity full-status clear */
	/* Bin->render coherency handoff. The binner (CT0, just done) writes the per-tile sub-lists
	 * into tile_alloc/overflow through the GPU's L2T; CT1's CL executor then FETCHES those
	 * tile-lists. Two things must hold before CT1 starts:
	 *   1. the binner's tile-list output must be flushed to RAM, and that flush must COMPLETE
	 *      (not merely be issued) — else CT1 reads an INCOMPLETE tile-list and parks near
	 *      rcl_end with FRDONE never firing (the data-dependent render wedge: worse on complex
	 *      frames whose larger tile-lists take longer to drain);
	 *   2. the render-side slice caches must be invalidated so a reused GPU VA doesn't serve a
	 *      prior BO's stale lines.
	 * The previous code only ISSUED the L2T flush then kicked CT1 immediately (no wait-new), so
	 * on a heavy frame the flush was still in flight when CT1 began fetching. Correct order:
	 * clean + WAIT, then invalidate, then kick. Linux v3d runs an invalidate before both bin
	 * and render for the same coherency reason. */
	l2t_flush_wait(c0);                       /* wait-old: any prior L2T flush must be idle */
	c0[CTL_L2TCACTL/4]=L2TCACTL_L2TFLS;       /* clean the binner's tile-list output to RAM */
	l2t_flush_wait(c0);                       /* wait-new: it must COMPLETE before CT1 fetches */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;    /* then drop stale render-side slice-cache lines */
	/* NOTE (2026-07-25): a symmetric handoff completion barrier here did NOT fix the reported
	 * dynamic-lighting (r_dynamic 1) anomaly, so it is NOT a render-side sample-coherency race.
	 * Follow-up instrumentation then ruled out the two data hypotheses too: the STATIC lightmap
	 * built at map load is byte-deterministic across boots (LMBUILD crc identical x4), and the
	 * early demo frames that scored worst cross-boot had NO per-frame lightmap uploads (lm=0).
	 * The cross-boot "flicker" score was dominated by early-frame warmup + occasional black/
	 * truncated boots — i.e. the cross-boot harness (correct for the #67 per-boot collapse) is
	 * the wrong instrument for a within-run flicker. Barrier removed here (dead weight); the
	 * within-run behaviour is being measured directly. The #67 GEOMETRY fix is the pre-bin
	 * barrier only. */
	/* #67 diagnostic: the binner's tile_alloc output is now flushed to RAM — checksum it (no-op
	 * unless V3D_BIN_CRC=1) so a fixed-scene test can tell binner non-determinism from a CT1
	 * read fault. Placed AFTER the flush completes and BEFORE the CT1 kick. */
	bincrc_capture(s->qma, s->qms);
	/* --- render (CT1); wait FRDONE --- */
	c0[CLE_CT1QBA/4]=s->rcl_start; c0[CLE_CT1QEA/4]=s->rcl_end;
	/* Wait for FRDONE, detecting a wedge two ways: (a) FAST — ct1ca FROZEN (the confirmed wedge
	 * signature: CT1 stuck at one address) for ~0.8 s while not done; a legitimately slow frame
	 * keeps advancing ct1ca between samples, so this never false-trips and is safe for heavy
	 * frames; (b) BACKSTOP — the absolute spin cap. Frozen-detection shrinks the mitigation hitch
	 * from the full ~2.5 s cap to ~0.8 s. On done: spins != 0; on wedge: spins == 0 (unchanged
	 * downstream handling). */
	unsigned qpu_acks = 0;   /* # of poll-samples that found QPU int bits latched + cleared them */
	{
		uint32_t last_ca = c0[0x0114/4];
		unsigned frozen = 0;
		for (spins = 16000000u; spins; spins--) {
			if (c0[CTL_INT_STS/4] & INT_FRDONE)
				break;                                  /* render done */
			if ((spins & 0xfffffu) == 0u) {             /* sample ct1ca ~every 1M spins (~160 ms) */
				/* Service latched QPU-interrupt bits mid-render (Linux clears them every IRQ;
				 * we poll). Write-1-to-clear ONLY the QPU bits — never bit0 — so FRDONE cannot
				 * be lost. If an unacknowledged QPU int stalls fragment dispatch, acking it here
				 * lets the render complete. qpu_acks tells us whether the bits re-assert. */
				uint32_t sts = c0[CTL_INT_STS/4];
				if (sts & INT_QPU_MASK) { c0[CTL_INT_CLR/4] = sts & INT_QPU_MASK; qpu_acks++; }
				uint32_t ca = c0[0x0114/4];
				if (ca == last_ca) {
					if (++frozen >= 5u) { spins = 0; break; }   /* frozen ~0.8 s -> wedged */
				}
				else { frozen = 0; last_ca = ca; }
			}
		}
	}
	if (spins == 0) {
		uint32_t ca1 = c0[0x0114/4];
		v3d_phoenix_render_timeouts++;   /* stall counter for the repro harness */
		job_failed = 1;
		fprintf(stderr, "v3d-winsys: RENDER TIMEOUT int_sts=0x%08x ct1cs=0x%08x "
			"ct1ca=0x%08x[%x..%x] ct1ea=0x%08x gmp=0x%08x gmpvio=0x%08x mmu_ill=0x%08x\n",
			c0[CTL_INT_STS/4], c0[0x0104/4], ca1, s->rcl_start, s->rcl_end,
			c0[0x010c/4], c0[0x0800/4], c0[0x0808/4], W.hub[MMU_ILLEGAL_ADDR/4]);
		fprintf(stderr, "v3d-winsys: RENDER qpu_int_acks=%u (QPU bits serviced mid-render; "
			"nonzero+persisting = bits re-assert continuously)\n", qpu_acks);
		/* V3D error-debug registers (FDBGO/FDBGS/ERRSTAT) localize the wedged render
		 * pipeline stage; + decode the CL opcode byte CT1 is parked on. */
		{
			/* The opcode byte was read at `ca1 & ~0xf` - CLE packets are BYTE-granular,
			 * so every historical wedge_op= is only trustworthy when ct1ca & 0xf == 0.
			 * Read the byte AT ct1ca; the 0xffffffff sentinel still means "no CPU view". */
			int opb = wedge_op_at(ca1);
			unsigned op = (opb >= 0) ? (unsigned)opb : 0xffffffffu;
			const char *opn = (op==21)?"BRANCH_TO_IMPLICIT":(op==18)?"RETURN_FROM_SUBLIST":
				(op==124)?"TILE_COORDINATES":(op==23)?"SUPERTILE_COORDS":(op==0)?"HALT/zero":"?";
			fprintf(stderr, "v3d-winsys: RENDER DBG fdbgo=0x%08x fdbgs=0x%08x errstat=0x%08x "
				"wedge_op=0x%02x(%s)\n",
				c0[0x0f04/4], c0[0x0f10/4], c0[0x0f20/4], op, opn);
		}
		/* The REAL fault (VIO_ADDR = VA>>8) + which BO owns it — the definitive signal for
		 * the intermittent q3dm7 wedge (texture VA-recycle vs render-CL/tile-alloc vs unmapped). */
		{
			/* DIAGNOSTIC ONLY - the timeout branch. The gate was `rdbg++ < 3`, so a 4th
			 * render wedge in a process printed nothing; generous cap + a notice. */
			static int rdbg = 0;
			if (rdbg < WEDGE_DBG_MAX) {
				rdbg++;
				uint32_t vio = W.hub[MMU_VIO_ADDR/4];
				uint32_t fva = vio << 8;
				fprintf(stderr, "v3d-winsys: RENDER MMU-VIO vio_addr=0x%08x fault_va=0x%08x "
					"vio_id=0x%08x (mmu_ill above = scratch-cfg, NOT the fault)\n",
					vio, fva, W.hub[MMU_VIO_ID/4]);
				gpuva_describe("RENDERFAULT", fva);
				gpuva_describe("RCLSTART", s->rcl_start);
				gpuva_describe("RENDERCA", ca1);
				/* Where ct1ca parked relative to the SUBMITTED extent + the byte-granular
				 * CL window there. Same shapes as the bin queue and as the server. */
				wedge_cl_dump("RENDER CL@ct1ca", ca1, s->rcl_start, s->rcl_end);
			}
			else if (rdbg == WEDGE_DBG_MAX) {
				rdbg++;
				fprintf(stderr, "v3d-winsys: RENDER wedge dump SUPPRESSED after %d dumps "
					"(further render wedges print the TIMEOUT + DBG lines only)\n", WEDGE_DBG_MAX);
			}
		}
		/* re-read CT1CA to see if it is advancing (slow) or wedged (stall) */
		fprintf(stderr, "v3d-winsys: RENDER ct1ca recheck=0x%08x\n", c0[0x0114/4]);
	}
job_retry:
	/* MITIGATION. The wedge is a HW-marginal fragment/depth-pipeline drain stall (fdbgs shows
	 * DEPTHO_FIFO/INTERPZ stalled with valid work queued) triggered by specific complex
	 * geometry under the render-to-scanout (RASTER+uncached) store — part of the V3D RT
	 * coherency wall this port has hit several ways. It is NOT corruption/aliasing/cold-state
	 * (all ruled out via the instrument-validated wedge dump: valid RCL, single-match BOs).
	 *
	 * Recovery strategy: the wedge is DATA-dependent — re-submitting the SAME frame re-hangs
	 * at the same ct1ca/ct0ca across true resets (HW-confirmed). So do NOT burn another
	 * multi-second spin-timeout re-running known-bad work. Instead do ONE true reset (asbStop +
	 * assert RSTN + power-on + re-apply core regs), which clears the wedged core so the NEXT
	 * (different) frame renders, and DROP this frame. That converts a wedge from a multi-second
	 * re-submit freeze into a single dropped frame — the difference between unplayable and
	 * playable. (Earlier code re-submitted up to SUBMIT_MAX_RETRIES times, stacking timeouts.) */
	if (job_failed) {
		v3d_phoenix_render_recoveries++;
		fprintf(stderr, "v3d-winsys: GPU wedged — true reset + drop this frame "
			"(mitigation; drops=%u). Wedge is HW-marginal depth-pipeline drain stall.\n",
			v3d_phoenix_render_recoveries);
		/* Identify WHAT was dropped. "Drop the frame" is a sound trade for a
		 * frame that will be redrawn next tick, which is what this mitigation
		 * was written for -- but the same path also swallows ONE-SHOT jobs, and
		 * for those a drop is silent, permanent data loss.
		 *
		 * On V3D 4.2 that is not hypothetical: vkCmdCopyBuffer is a CL RENDER
		 * job, not a TFU blit (v3dvx_meta_common.c gates TFU on V3D_VERSION>=71),
		 * so V3DV's buffer uploads come through here. Drop one and the
		 * destination BO keeps the zeros it was created with, for the rest of the
		 * process -- which is the current best explanation for #67, where one
		 * alias model's geometry is missing for a whole boot while the frame is
		 * otherwise perfect (docs/misc/2026-09-03-torch-intermittency-driver-analysis.md,
		 * KNOWN-ISSUES #67; the torch verdict is predicted 26/26 by whether a
		 * wedge was logged on this control-list page).
		 *
		 * A tiny CL is the tell: a real render frame has a substantial binner
		 * list, whereas a meta-copy is a handful of words. Print the extents so
		 * the two classes can be told apart in a UART log instead of guessed at.
		 * Cheap: this runs only on a wedge, which already costs a reset. */
		/* Which BO actually backs this job's lists? The RCL byte dump below
		 * showed RGBA pixel data (every 4th byte 0xff) where control-list packets
		 * belong, with real packets only appearing ~0x40 later -- so the address
		 * V3DV handed us resolves into a DATA buffer, not into its RCL. Name the
		 * BOs so an aliasing/VA-reuse bug is visible instead of inferred: >1 match
		 * means two BOs overlap, 0 means the address is not mapped at all. */
		gpuva_describe("DROP-BCLSTART", s->bcl_start);
		gpuva_describe("DROP-RCLSTART", s->rcl_start);
		/* For a SMALL wedged RCL, dump the whole list as BYTES with the park
		 * offset marked. These lists wedge deterministically at the same offset
		 * on a freshly reset core (measured: ct1ca parks at 0x43 of a 98-byte
		 * RCL, every attempt), so the stalling packet is identifiable by reading
		 * the bytes -- which is cheaper and more certain than inferring what
		 * V3DV emitted. Only for tiny lists and only on a wedge: a full render
		 * RCL would flood the UART. */
		if ((s->rcl_end - s->rcl_start) <= 256u) {
			uint8_t *rp = (uint8_t *)gpuva_to_cpu(s->rcl_start);
			uint32_t rn = (uint32_t)(s->rcl_end - s->rcl_start);
			uint32_t park = (c0[0x0114/4] & ~0xfu) >= s->rcl_start
				? (uint32_t)((c0[0x0114/4] & 0xfffffffu) - (s->rcl_start & 0xfffffffu))
				: 0xffffffffu;
			fprintf(stderr, "v3d-winsys: RCLBYTES gpuva=0x%08x n=%u park_off=0x%x:",
				s->rcl_start, rn, park);
			if (rp != NULL) {
				for (uint32_t i = 0; i < rn; i++) {
					fprintf(stderr, "%s%02x", (i == park) ? " >" : " ", rp[i]);
				}
			}
			else {
				fprintf(stderr, " (no BO)");
			}
			fprintf(stderr, "\n");
		}
		fprintf(stderr, "v3d-winsys: DROPPED job bcl=[0x%08x..0x%08x] %u B  "
			"rcl=[0x%08x..0x%08x] %u B%s\n",
			s->bcl_start, s->bcl_end, (unsigned)(s->bcl_end - s->bcl_start),
			s->rcl_start, s->rcl_end, (unsigned)(s->rcl_end - s->rcl_start),
			((s->bcl_end - s->bcl_start) < 256u)
				? "  <-- TINY CL: likely a one-shot upload/meta-copy, NOT a redrawable frame"
				: "");
		reset_reinit_core();   /* clean the wedged core so the next (different) frame renders */
		(void)attempt;

		/* NOTE (2026-09-03): retrying these was TRIED AND FAILED on hardware.
		 * A one-shot upload CL re-wedges on a freshly reset core, every time:
		 * ct1ca parks at the SAME offset (0x43 of a 98-byte RCL) on all 3
		 * attempts, int_sts=0, gmpvio=0. So the wedge on these jobs is
		 * DETERMINISTIC, not a marginal stall. The retry cost ~10 s of futile
		 * resets per boot and moved the #67 rate 0/2, so it was reverted.
		 *
		 * CORRECTION (2026-09-03, later): this comment used to conclude from that
		 * determinism that "the CL itself is malformed". That is WRONG, and the
		 * wrong diagnosis matters because it points the fix at Mesa instead of at
		 * us. The 98-byte RCL was decoded packet by packet against
		 * v3dvx_meta_common.c and it matches what upstream Mesa emits, byte for
		 * byte. Offset 0x43 is the second dummy STORE_TILE_BUFFER_GENERAL
		 * (buffer_to_store = NONE, address 0) of Mesa's GFXH-1742 workaround --
		 * a no-op tile store that upstream issues on every Linux Pi 4, and which
		 * sits BEFORE the generic tile list, so the copy never even begins. The
		 * list is well-formed; what wedges is THIS PORT's tile-store path on a
		 * w x 1 raster frame (the recorded case is w=183, item_size=1 -- an odd
		 * width, i.e. the same RT-coherency wall this file records at :1296).
		 *
		 * So the dropped upload IS the data loss (every decodable dropped job is
		 * a 14 B BCL / ~100 B RCL meta-copy, and rcl_len == 92 + 3*n_supertiles
		 * accounts for every size on record: 98, 101, 122, 143), and the cure is
		 * to perform the copy on the CPU rather than re-submit it.
		 *
		 * SECOND, SEPARATE DEFECT, also on record: only about HALF the drops have
		 * a decodable list at all. Counted over the .log files under artifacts/rpi4b-uart:
		 * 65 RCLBYTES dumps, 29 beginning with 0x79 (TILE_RENDERING_MODE_CFG, a
		 * valid RCL) and 36 beginning with RGBA pixel data or zeros (0x74 x15,
		 * 0x2a x13, 0xff x3, 0x90 x3, 0x00 x2 -- none a defined CLE opcode), all
		 * parked at offset 0. Treat the totals as a moving target (every bench
		 * adds logs); the RATIO has stayed near half. For those there is no LOAD,
		 * no STORE, no address and no extent to recover -- the RCL BO is holding
		 * image data. That is a different bug from this one and a CPU fallback
		 * cannot help it; see docs/misc/2026-09-03-v3d-dropped-metacopy-fix-plan.md.
		 *
		 * Left as a comment so nobody re-tries the retry. */
	}
	/* L2T flush so RT stores reach RAM before CPU readback (scout finding). */
	l2t_flush_wait(c0);                       /* GFXH-1897: render flush must complete first */
	c0[CTL_L2TCACTL/4]=L2TCACTL_L2TFLS|(2u<<1); /* FLM_CLEAN */

	/* DRM_V3D_SUBMIT_CL_FLUSH_CACHE. Mesa sets this on a job whose shaders wrote
	 * through the TMU -- `job->tmu_dirty_rcl && screen->has_cache_flush`, v3d_job.c:694
	 * -- and Linux answers it by queueing a CACHE_CLEAN job that runs v3d_clean_caches()
	 * before the job is reported complete (v3d_submit.c:998 -> v3d_gem.c:202).
	 *
	 * We advertise SUPPORTS_CACHE_FLUSH=1 in ioc_get_param -- which is also what makes
	 * Mesa expose shader images and SSBOs in the first place (v3d_screen.c:173,178,305)
	 * -- and then never honoured the flag. The clean just above is a bare FLM_CLEAN that
	 * is issued and NOT awaited, and nothing anywhere on this path drains the TMU write
	 * combiner. The combiner holds PARTIAL cache-line writes that never reach RAM on
	 * their own; this file says so itself in the TFU epilogue, which is the only place
	 * the drain exists. A u32 parked there can arrive in DRAM arbitrarily later -- after
	 * GEM_CLOSE has munmap'd the page and the kernel has recycled it into somebody
	 * else's allocation. That is exactly the shape of C1.
	 *
	 * Env-gated while it is being measured against the closed-BO quarantine; see
	 * docs/KNOWN-ISSUES.md (C1). The counter is here because the whole hypothesis
	 * depends on this workload actually producing flagged jobs -- if the count stays
	 * zero, there is no writer and the idea is dead, and that must be visible in the
	 * log rather than inferred from silence. */
	if ((s->flags & DRM_V3D_SUBMIT_CL_FLUSH_CACHE) != 0u) {
		static int clean = -1;
		static uint32_t flagged;

		if (clean < 0) {
			const char *e = getenv("V3D_CL_CACHE_CLEAN");
			clean = ((e != NULL) && (*e == '1')) ? 1 : 0;
		}
		flagged++;
		if ((flagged == 1u) || ((flagged % 512u) == 0u)) {
			fprintf(stderr, "v3d-winsys: CL FLUSH_CACHE job #%u -- %s\n", flagged,
				(clean != 0) ? "cleaning (TMUWCF + L2T clean, both awaited)"
				             : "NOT cleaning (legacy behaviour, A/B baseline)");
		}
		if (clean != 0) {
			uint32_t ccspins;

			l2t_flush_wait(c0);                              /* GFXH-1897: prior flush idle */
			c0[CTL_L2TCACTL/4] = L2TCACTL_TMUWCF;            /* drain the TMU write combiner... */
			for (ccspins = 1000000u; ccspins && (c0[CTL_L2TCACTL/4] & L2TCACTL_TMUWCF); ccspins--) {}
			c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN;  /* ...then write back... */
			l2t_flush_wait(c0);                              /* ...and wait for the clean */
		}
	}
	return 0;
}

/* UIF byte offset of pixel (x,y) for a cpp=4 image of height image_h — transcribed from mesa
 * v3d_tiling.c v3d_get_uif_pixel_offset (verified). Used only by the TFU readback probe to test
 * that the TFU output matches the UIF layout the TMU reads BEYOND the first 8x8 block (the prior
 * dst[16] test only covered block 0, which is XOR/stride-invariant; horizontal striping is a
 * vertical-period artifact that only shows up across blocks/pages). do_xor for UIF_XOR images. */
static uint32_t uif_pixel_off(uint32_t image_h, uint32_t x, uint32_t y, int do_xor)
{
	const uint32_t uw = 4u, uh = 4u, lmbw = 3u, lmbh = 3u;   /* microtile 4x4; macroblock 8x8 px */
	uint32_t mb_x = x >> lmbw, mb_y = y >> lmbh;
	uint32_t px = x - (mb_x << lmbw), py = y - (mb_y << lmbh);
	if (do_xor && ((mb_x / 4u) & 1u)) mb_y ^= 0x10u;
	/* mb_h is the per-column macroblock stride; it MUST be derived from the slice's PADDED
	 * height (mesa v3dv_image.c v3d_setup_plane_slices: padded_height = align(h,uif_block_h) +
	 * ub_pad*uif_block_h), NOT the bare image height. image_h here is already the padded height
	 * (callers pass the ub-padded value). For ub_pad==0 sizes (all power-of-two heights, e.g.
	 * 64/128/256 — exactly what the original TFU vcheck probe covered) padded==align(h,8), so this
	 * is unchanged; the padding only matters for the rare NPOT heights v3d_get_ub_pad pads. */
	uint32_t mb_h = (image_h + (1u << lmbh) - 1u) >> lmbh;
	uint32_t mb_id = ((mb_x / 4u) * ((mb_h - 1u) * 4u)) + mb_x + mb_y * 4u;
	uint32_t base = mb_id * 256u;
	uint32_t tile_off = ((py < uh) ? 0u : 128u) + ((px < uw) ? 0u : 64u);
	uint32_t ux = px & (uw - 1u), uy = py & (uh - 1u);
	return base + tile_off + (ux * 4u + uy * uw * 4u);   /* cpp=4 */
}

/* DRM_V3D_SUBMIT_TFU: run a Texture Formatting Unit job — the hardware buffer->image (and
 * image->image) copy V3DV uses for every TILED/OPTIMAL texture upload, blit and mipmap
 * generation. Previously the winsys's ioctl dispatch handled only SUBMIT_CL and fell through
 * to `default: return 0`, so every TFU job was a silent no-op: the destination image stayed
 * at its alloc-time zero and every sampled texture rendered BLACK (#29). This programs the
 * TFU registers exactly as linux v3d_tfu_job_run does for V3D 4.2 (ver<71) and waits for the
 * unit to finish, bracketed by the same MMU-TLB + cache coherency operations ioc_submit_cl
 * uses (texture BOs were just created by ioc_create_bo, which writes PTEs but doesn't flush
 * the TLB; the source staging buffer must be coherent in RAM before the TFU reads it, and the
 * tiled destination must be flushed to RAM before a later sampling CL job reads it).
 *
 * The submit struct's iia/ica/iua/ioa are already full GPU virtual addresses — Mesa folds
 * each BO's GPU-VA base (the winsys's create_bo c->offset) into them in meta_emit_tfu_job /
 * copy_*_tfu — and the TFU is behind the V3D MMU, so we write them verbatim (no va2pa). */
static int ioc_submit_tfu(struct drm_v3d_submit_tfu *t)
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins;

#ifdef VKQ_CPU_TILE
	/* --- CPU-tile discriminator (task #29 striping triage) -----------------------------------
	 * Instead of kicking the TFU to tile a RASTER staging buffer into the UIF destination image,
	 * CPU-tile the texels directly into the (uncached) destination BO using the SAME uif_pixel_off
	 * tiler the `TFU vcheck` probe proved byte-correct (UIF-VERIFIED 6/6). The TFU hardware kick is
	 * SKIPPED for this image. If the texture then samples CLEAN, the TFU's produced layout differed
	 * from what the descriptor reads at points vcheck did not sample (upload/blit-side bug) — and we
	 * have a shippable CPU-tile fallback. If it STILL samples striped, the bug is independent of the
	 * upload path: uif_pixel_off(t->ios height) itself disagrees with the TMU descriptor's slice
	 * height ⇒ a v3dv image-creation / TEXTURE_SHADER_STATE bug (look in v3dv_image.c, not the blit).
	 *
	 * Strict gating (anything not satisfied falls through to the real TFU below):
	 *   - dest tiling is UIF (IOA FORMAT field 6=UIF_NO_XOR / 7=UIF_XOR); uif_pixel_off is UIF+cpp=4
	 *     only, so LINEARTILE/UBLINEAR mips MUST take the HW path or they would be corrupted;
	 *   - source is RASTER (ICFG FORMAT field == 0); only then is the staging buffer a plain raster
	 *     image we can index as src[y*srcstride + x]. A tiled (image->image / mipmap) source is NOT
	 *     raster and must use the TFU;
	 *   - the dest BO has room for the whole tiled image (bounds check vs gpuva_bo_remaining). */
	{
		uint32_t iofmt = (t->ioa >> 3) & 0x7u;            /* IOA dst FORMAT field (V3D33) */
		uint32_t ifmt  = (t->icfg >> 18) & 0xfu;          /* ICFG src FORMAT field (V3D33) */
		uint32_t w   = t->ios & 0xffffu;
		uint32_t hgt = t->ios >> 16;
		int dst_is_uif = (iofmt == 6u || iofmt == 7u);
		int src_is_raster = (ifmt == 0u);                 /* V3D33_TFU_ICFG_FORMAT_RASTER */
		int xor = (iofmt == 7u);
		/* The UIF tiler's per-column macroblock stride uses the slice PADDED height, not the bare
		 * copy height (= ios>>16). Transcribe mesa v3dv_image.c v3d_get_ub_pad (cpp=4: utile_h=4,
		 * uif_block_h=8) + padded_height = align(h,8) + ub_pad*8, so the CPU layout matches the TMU
		 * descriptor (which encodes image_height + level_0_ub_pad) for NPOT heights too. For all
		 * power-of-two heights ub_pad==0, so phgt == align(h,8) and this is a no-op. */
		const uint32_t uif_block_h = 8u, page_cache_ub_rows = 32u,
		               page_ub_rows_x1_5 = 6u, page_cache_minus_1_5 = 26u;
		uint32_t aligned_h = (hgt + uif_block_h - 1u) & ~(uif_block_h - 1u);
		uint32_t height_ub = aligned_h / uif_block_h;
		uint32_t off_in_pc = height_ub % page_cache_ub_rows;
		uint32_t ub_pad = 0u;
		if (off_in_pc != 0u) {
			if (off_in_pc < page_ub_rows_x1_5)
				ub_pad = (height_ub < page_cache_ub_rows) ? 0u
				         : (page_ub_rows_x1_5 - off_in_pc);
			else if (off_in_pc > page_cache_minus_1_5)
				ub_pad = page_cache_ub_rows - off_in_pc;
		}
		uint32_t phgt = aligned_h + ub_pad * uif_block_h;   /* slice->padded_height */
		/* Source row stride in PIXELS. For a RASTER source Mesa sets iis = src_stride/cpp; a 0 iis
		 * means "broadcast 1 row" (clear/fill) which we don't CPU-tile. Fall back to width if the
		 * field is the implicit no-stride case but width is sane. */
		uint32_t src_stride_px = (t->iis & 0xffffffu);
		if (src_stride_px == 0u) src_stride_px = w;

		if (dst_is_uif && src_is_raster && w >= 1u && hgt >= 1u) {
			const uint32_t *src = gpuva_to_cpu(t->iia);
			uint32_t dst_gpuva = t->ioa & ~0x3fu;         /* strip the tiling-format tag bits */
			uint32_t *dst = gpuva_to_cpu(dst_gpuva);
			uint32_t avail = gpuva_bo_remaining(dst_gpuva);
			/* Conservative upper bound on bytes the tiler touches, using the PADDED height (phgt).
			 * The XOR variant flips bit 4 of mb_y inside uif_pixel_off, so the farthest write is NOT
			 * simply (w-1,h-1) — a lower row with bit-4 set can land higher. Bound by the full padded
			 * slice: mb columns = ceil(w/8), mb rows = ceil(phgt/8), and XOR can raise the effective
			 * mb_y by up to 0x10 blocks, so reserve (mb_h+0x10) rows. Each macroblock is 256 B. This
			 * over-estimates slightly but can never under-bound, so it cannot mis-size into a neighbor. */
			uint32_t mb_w = (w + 7u) / 8u;
			uint32_t mb_h_b = (phgt + 7u) / 8u + (xor ? 0x10u : 0u);
			uint64_t max_off = (uint64_t)mb_w * (uint64_t)mb_h_b * 256u;

			if (src && dst && max_off <= (uint64_t)avail) {
				for (uint32_t y = 0; y < hgt; y++) {
					const uint32_t *srow = src + (uint64_t)y * src_stride_px;
					for (uint32_t x = 0; x < w; x++)
						dst[uif_pixel_off(phgt, x, y, xor) / 4u] = srow[x];
				}
				__asm__ volatile("dsb sy" ::: "memory");   /* drain (completion, not dmb) the CPU-tiled stores before the GPU (TMU) samples dst — matches the CL-submit barrier */

				/* Keep the read-side coherency epilogue so the TMU's L2T/slice caches drop any
				 * stale view of this GPU VA before the sampling CL — identical to the HW-path
				 * epilogue below, minus the TFU write-combiner drain (no TFU ran). */
				l2t_flush_wait(c0);
				c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN;
				l2t_flush_wait(c0);
				c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;

				{
					static unsigned cpu_n = 0;
					cpu_n++;
					if (cpu_n <= 12u || (cpu_n & 0x3ffu) == 0u)
						fprintf(stderr, "v3d-winsys: TFU CPU-TILE path %ux%u xor=%d phgt=%u ub_pad=%u "
							"iia=0x%08x->ioa=0x%08x (n=%u, HW TFU skipped)\n",
							w, hgt, xor, phgt, ub_pad, t->iia, t->ioa, cpu_n);
				}
				return 0;   /* image populated by the CPU; do NOT kick the TFU */
			}
			else {
				static unsigned cpu_skip_n = 0;
				cpu_skip_n++;
				if (cpu_skip_n <= 8u)
					fprintf(stderr, "v3d-winsys: TFU CPU-TILE FALLTHROUGH %ux%u "
						"(src=%p dst=%p max_off=%llu avail=%u) — using HW TFU\n",
						w, hgt, (void *)src, (void *)dst,
						(unsigned long long)max_off, avail);
			}
		}
		else {
			static unsigned cpu_hw_n = 0;
			cpu_hw_n++;
			if (cpu_hw_n <= 8u)
				fprintf(stderr, "v3d-winsys: TFU CPU-TILE skip (non-UIF/non-raster: "
					"iofmt=%u ifmt=%u %ux%u) — using HW TFU\n", iofmt, ifmt, w, hgt);
		}
	}
#endif /* VKQ_CPU_TILE */

	/* --- prologue: make the source coherent + translations fresh ---
	 * Drain CPU stores into the uncached source BO *and* the page tables to DRAM BEFORE the
	 * first GPU MMIO poke below (aarch64 Normal-NC vs Device ordering); MUST be dsb
	 * (completion), not dmb — the TFU is a non-coherent external master reading both straight
	 * from DRAM. Identical reasoning and instruction to the CL path's barrier above.
	 *
	 * ⚠ This was MISSING until 2026-09-18, and the CL path's own comment made it look covered:
	 * it says "the TFU CPU-tile path already drains for the same reason (see the barrier in the
	 * TFU submit)" — but that barrier lives inside `#ifdef VKQ_CPU_TILE`, which is OFF in
	 * shipping builds and returns before the kick. So every shipping GL mipmap/blit
	 * (v3d_blit.c) and every Vulkan image copy (v3dv_queue.c) kicked the TFU with nothing
	 * ordering the source texels or the fresh PTEs against the register writes. A slip here
	 * shows as a garbage or zeroed texture level, or a TFU fault — not as a binner wedge.
	 *
	 * Then flush the MMU TLB so the just-created source/dest BOs are translated by their new
	 * PTEs, and invalidate the slice caches + flush L2T so the TFU reads the source staging
	 * buffer from RAM rather than a stale cached view (mirrors the ioc_submit_cl pre-bin
	 * sequence). */
	__asm__ volatile("dsb sy" ::: "memory");
	mmu_flush_tlb(h);
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;
	l2t_flush_wait(c0);                        /* prior L2T flush must be idle (GFXH-1897) */
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);                        /* and complete before the TFU reads source */

	/* Clear any stale TFU done/fail latch so our post-kick poll sees only this job. */
	h[HUB_INT_CLR/4] = HUB_INT_TFUC | HUB_INT_TFUF;

	/* --- kick: program the TFU regs (ICFG last, with IOC). Verbatim from v3d_tfu_job_run. --- */
	h[TFU_IIA/4]  = t->iia;
	h[TFU_IIS/4]  = t->iis;
	h[TFU_ICA/4]  = t->ica;
	h[TFU_IUA/4]  = t->iua;
	h[TFU_IOA/4]  = t->ioa;
	h[TFU_IOS/4]  = t->ios;
	h[TFU_COEF0/4] = t->coef[0];
	if (t->coef[0] & TFU_COEF0_USECOEF) {      /* YUV: COEF1..3 valid only when USECOEF set */
		h[TFU_COEF1/4] = t->coef[1];
		h[TFU_COEF2/4] = t->coef[2];
		h[TFU_COEF3/4] = t->coef[3];
	}
	h[TFU_ICFG/4] = t->icfg | TFU_ICFG_IOC;    /* this write starts the job */

	/* --- wait for done. Primary signal: HUB_INT TFUC (set on completion) / TFUF (on failure) —
	 * sticky once set (cleared only by us, pre-kick) so it can't be missed, and HUB_INT_STS is the
	 * raw latch so this works without unmasking. Fallback signal: CS BUSY clearing AFTER we have
	 * observed it set at least once — mask-independent, so if STS ever turns out post-mask on this
	 * silicon the job still completes instead of false-timing-out. BUSY alone races the kick (it
	 * reads 0 in the window before the unit asserts busy), hence the "saw_busy" gate. Bounded spin
	 * like the CL paths — a wedged TFU must not hang the process. --- */
	{
		int saw_busy = 0;
		for (spins = 8000000u; spins; spins--) {
			if (h[HUB_INT_STS/4] & (HUB_INT_TFUC | HUB_INT_TFUF))
				break;
			uint32_t cs = h[TFU_CS/4];
			if (cs & TFU_CS_BUSY) saw_busy = 1;
			else if (saw_busy) break;   /* completed (mask-independent fallback) */
		}
	}
	{
		uint32_t isr = h[HUB_INT_STS/4];
		int failed = (spins == 0) || (isr & HUB_INT_TFUF);
		h[HUB_INT_CLR/4] = HUB_INT_TFUC | HUB_INT_TFUF;
		if (failed) {
			fprintf(stderr, "v3d-winsys: TFU TIMEOUT/FAIL hub_int=0x%08x mskts=0x%08x cs=0x%08x "
				"iia=0x%08x ioa=0x%08x ios=0x%08x icfg=0x%08x\n",
				isr, h[HUB_INT_MSK_STS/4], h[TFU_CS/4], t->iia, t->ioa, t->ios, t->icfg);
			/* Don't abort the client — return success so rendering proceeds (a failed
			 * upload leaves the image zero, same as before, just visible in the log). */
		}
		else {
			static unsigned tfu_n = 0;
			tfu_n++;
			/* Discriminator probe (gated; the dest BO is uncached + just L2T-clean-flushed, so a
			 * CPU read sees RAM). On the instrumented boot this separates the three outcomes:
			 *   src=0            -> the upload-to-staging is the bug, not the TFU;
			 *   src!=0, dst=0    -> TFU register programming is wrong (tune on HW);
			 *   src!=0, dst!=0, still black -> the copy works; the bug is the sampler/descriptor
			 *                       path (resolves the "CL fallback also black" mystery — the TFU
			 *                       was necessary but not sufficient).
			 * The marker proves the TFU KICKED; this readback proves whether pixels LANDED. */
			if (tfu_n <= 12u || (tfu_n & 0x3ffu) == 0u) {
				/* Coherency discriminator (striping triage): the dest BO is uncached, and we have
				 * L2T-clean-flushed the TFU output to RAM above (the flush is BEFORE this read).
				 * Read the SOURCE (raster staging) first words, and the DEST (tiled image) at the
				 * START of the buffer and a MIDDLE row, after re-deriving stride from ios width.
				 * - src all-stale/0 -> CPU-staging->TFU input coherency gap (staging cached, not
				 *   cleaned before the TFU read);
				 * - dst start nonzero but mid-row 0/garbage -> TFU only partially wrote the tiled
				 *   image (write/flush gap) -> the BO itself is striped;
				 * - dst BO looks fully populated here but the SAMPLE is striped -> the gap is
				 *   TMU-side (L2T not invalidated before the sampling CL reads it). */
				const uint32_t *src = gpuva_to_cpu(t->iia);
				/* IOA carries the dest tiling-format field in its low bits (DIMTW bit0 +
				 * FORMAT bits3..5, e.g. |0x30 for UIF_NO_XOR), so the address part is t->ioa
				 * with those low bits cleared. gpuva_to_cpu does NOT mask, so passing the raw
				 * tagged t->ioa would offset the CPU pointer by up to 0x38 bytes and scramble
				 * the discriminator. Mask the low 6 bits (the whole tag) to recover the BO base.
				 * (iia carries NO format bits — input format lives in ICFG — and includes the
				 * staging sub-alloc byte offset, so it must NOT be masked.) */
				const uint32_t *dst = gpuva_to_cpu(t->ioa & ~0x3fu);
				uint32_t w = t->ios & 0xffffu;
				uint32_t hgt = t->ios >> 16;
				int src_nz = src ? (src[0] | src[1] | src[2] | src[3]) != 0u : -1;
				int dst_nz = dst ? (dst[0] | dst[1] | dst[2] | dst[3]) != 0u : -1;
				/* TILED-vs-LINEAR discriminator (the decisive striping test). The source staging
				 * buffer is RASTER: src word k = pixel (x=k%w, y=k/w). For a UIF_NO_XOR dest the
				 * V3D byte layout (verified against mesa v3d_tiling.c) puts dest byte 64 (word 16)
				 * = source pixel (4,0) = src[4]; a LINEAR/raster dest would instead have dest
				 * word 16 = pixel (16,0) = src[16]. So for w>=17:
				 *   dst[16] == src[4]  -> TFU produced correct UIF tiling (gap is elsewhere/read-side)
				 *   dst[16] == src[16] -> TFU wrote LINEAR (a winsys/icfg tiling-config bug)
				 * Bytes 0..63 are the top-left 4x4 microtile in raster order (identical to linear),
				 * so only word>=16 discriminates. Print dst[4],dst[16] alongside src[4],src[16]. */
				uint32_t s4  = (src && w >= 5u)  ? src[4]  : 0u;   /* src pixel (4,0)  */
				uint32_t s16 = (src && w >= 17u) ? src[16] : 0u;   /* src pixel (16,0) */
				uint32_t d4  = dst ? dst[4]  : 0u;                 /* dst word 4  (byte 16) */
				uint32_t d16 = dst ? dst[16] : 0u;                 /* dst word 16 (byte 64) — the test */
				const char *verdict = "n/a";
				if (dst && src && w >= 17u) {
					if (d16 == s4 && d16 != s16)      verdict = "UIF-OK";
					else if (d16 == s16 && d16 != s4) verdict = "LINEAR!";
					else                              verdict = "??";
				}
				/* Also print what Mesa REQUESTED (ioa low bits carry the dest tiling-format field,
				 * icfg the input format/ttype/opad). Combined with the produced-tiling verdict this
				 * is 3-way: Mesa-asked-UIF + produced-UIF + striped -> pure read-side (descriptor/
				 * L2T); Mesa-asked-UIF + produced-LINEAR -> TFU ignored IOA (winsys/HW); Mesa-asked-
				 * LINEAR (ioa format field == LINEARTILE/RASTER) -> Mesa dst_tiling bug upstream.
				 * NOTE: the decisive verdict comes from a texture with distinct (4,0)/(16,0) source
				 * pixels — read the BLUENOISE 64x64 line (random data), NOT conchars (blank top row
				 * -> src[4]==src[16] -> verdict ??). */
				fprintf(stderr, "v3d-winsys: TFU copy iia=0x%08x->ioa=0x%08x icfg=0x%08x %ux%u done (n=%u) "
					"src_nz=%d dst_nz=%d | TILING=%s dst[16]=%08x src(4,0)=%08x src(16,0)=%08x "
					"dst[4]=%08x\n",
					t->iia, t->ioa, t->icfg, w, hgt, tfu_n, src_nz, dst_nz, verdict,
					d16, s4, s16, d4);
				/* VERTICAL / inter-block probe (the actual striping test). dst[16]==src[4] only
				 * proves block 0 is UIF; horizontal striping lives in the inter-block vertical
				 * stride / page interleave / XOR, which block 0 cannot see. For each test pixel
				 * (x,y) compare the UIF-computed dest word against the raster source word. The
				 * dest tiling-format field is in t->ioa bits3..5 (6=UIF_NO_XOR, 7=UIF_XOR); pick
				 * do_xor from it so we test the SAME variant the TFU was told to write. A mismatch
				 * at any y>=8 point = the TFU's vertical layout diverges from the UIF formula the
				 * TMU (and the working GLQuake CPU-tiler) use -> the striping root cause. */
				if (dst && src && w >= 17u && hgt >= 17u) {
					uint32_t iofmt = (t->ioa >> 3) & 0x7u;   /* IOA FORMAT field */
					int xor = (iofmt == 7u);                 /* 6=UIF_NO_XOR, 7=UIF_XOR */
					static const uint16_t tx[6] = { 4, 0, 0, 8, 0, 16 };
					static const uint16_t ty[6] = { 0, 8, 16, 8, 32, 16 };
					int okN = 0, total = 0;
					char buf[160]; int bl = 0;
					for (int i = 0; i < 6; i++) {
						uint32_t px = tx[i], py = ty[i];
						if (px >= w || py >= hgt) continue;
						uint32_t doff = uif_pixel_off(hgt, px, py, xor) / 4u;   /* dst word */
						uint32_t soff = py * w + px;                            /* src word */
						uint32_t dv = dst[doff], sv = src[soff];
						int ok = (dv == sv);
						okN += ok; total++;
						bl += snprintf(buf + bl, sizeof(buf) - bl, " (%u,%u)%c", px, py, ok ? '=' : 'X');
					}
					fprintf(stderr, "v3d-winsys: TFU vcheck %ux%u %s xor=%d match=%d/%d%s\n",
						w, hgt, (okN == total) ? "UIF-VERIFIED" : "VERTICAL-MISMATCH",
						xor, okN, total, buf);
				}
			}
		}
	}

	/* --- epilogue: make the TFU-written tiled image visible to the TMU (sampler). The TFU writes
	 * the destination through the TMU's WRITE COMBINER + L2T; a plain L2T clean is NOT enough — the
	 * write combiner holds partial cache-line writes that never reach RAM, so the sampler reads a
	 * partially-stale image (the classic horizontal STRIPING on a freshly-TFU'd texture). Mirror
	 * linux v3d_clean_caches EXACTLY (the CACHE_CLEAN job the kernel runs after a write-producing
	 * job): wait for any in-flight L2T flush (GFXH-1897), then TMU write-combiner flush + WAIT, then
	 * L2T clean + WAIT. The waits are essential — the prior code issued FLM_CLEAN without waiting,
	 * so the sampling CL could start before the clean drained. */
	l2t_flush_wait(c0);                                  /* GFXH-1897: any prior L2T flush idle */
	c0[CTL_L2TCACTL/4] = L2TCACTL_TMUWCF;                /* drain the TMU write combiner... */
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL/4] & L2TCACTL_TMUWCF); spins--) {}  /* ...and wait */
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN;   /* write back dirty L2T lines... */
	l2t_flush_wait(c0);                                  /* ...and wait for the clean to complete */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;              /* drop stale read-only slice/TMU cache view */
	return 0;
}

/* device info from the real Pi4 IDENTs (rpi4-v3d-scout): IDENT0=0x04443356, etc. */
static int ioc_get_param(struct drm_v3d_get_param *gp)
{
	switch (gp->param) {
	case DRM_V3D_PARAM_V3D_UIFCFG:        gp->value = 0x00000045; return 0;
	case DRM_V3D_PARAM_V3D_HUB_IDENT1:    gp->value = 0x000e1124; return 0;
	case DRM_V3D_PARAM_V3D_HUB_IDENT2:    gp->value = 0x00000100; return 0;
	case DRM_V3D_PARAM_V3D_HUB_IDENT3:    gp->value = 0x00000e00; return 0;
	case DRM_V3D_PARAM_V3D_CORE0_IDENT0:  gp->value = 0x04443356; return 0; /* "V3D" */
	case DRM_V3D_PARAM_V3D_CORE0_IDENT1:  gp->value = 0x81001422; return 0;
	case DRM_V3D_PARAM_V3D_CORE0_IDENT2:  gp->value = 0x40078121; return 0;
	case DRM_V3D_PARAM_SUPPORTS_TFU:      gp->value = 1; return 0;
	/* V3DV's device_has_expected_features() (v3dv_device.c) gates physical-device
	 * creation on TFU && CSD && CACHE_FLUSH && CPU_QUEUE && MULTISYNC — all must be 1
	 * or vkEnumeratePhysicalDevices returns VK_ERROR_INITIALIZATION_FAILED ("requires
	 * kernel 6.8+"). The V3D 4.2 HW *has* CSD (compute) + the TFU; CPU_QUEUE is a
	 * kernel-side convenience we don't implement. We advertise all 1 so device-create
	 * succeeds; classic graphics (vkQuake) never dispatches compute or CPU jobs, so the
	 * unimplemented SUBMIT_CSD / CPU-queue paths are not exercised. Revisit if a Tier-2+
	 * client issues vkCmdDispatch / a CPU job. MULTISYNC_EXT MUST be 1: this Mesa's
	 * handle_cl_job unconditionally uses DRM_V3D_SUBMIT_EXTENSION (no legacy sync path);
	 * the winsys ignores the chained extensions (submit is synchronous), so 1 is safe. */
	case DRM_V3D_PARAM_SUPPORTS_CSD:      gp->value = 1; return 0;
	case DRM_V3D_PARAM_SUPPORTS_CACHE_FLUSH: gp->value = 1; return 0;
	case DRM_V3D_PARAM_SUPPORTS_MULTISYNC_EXT: gp->value = 1; return 0;
	case DRM_V3D_PARAM_SUPPORTS_PERFMON:       gp->value = 0; return 0;
	case DRM_V3D_PARAM_SUPPORTS_CPU_QUEUE:     gp->value = 1; return 0;
	default: gp->value = 0; return 0;
	}
}

/* True once the full-screen render target has been backed by the scanout framebuffer
 * (render-to-scanout). The present layer queries this to skip the CPU readback/blit/fb0
 * copies — the GPU already wrote the displayed surface. */
int v3d_phoenix_scanout_active(void);
int v3d_phoenix_scanout_active(void)
{
	/* "active" = a scanout buffer is backing an RT: single-buffer sets scanout_claimed; double-
	 * buffer tracks scanout_claim_idx (>0 once a buffer is claimed). Without the double case this
	 * read 0 in double mode and the present path wrongly fell back to CPU readback. */
	return W.scanout_claimed || (W.scanout_double && W.scanout_claim_idx > 0);
}

/* DRM_V3D_SUBMIT_CSD: run a Compute Shader Dispatch job. Previously a no-op stub, so EVERY
 * compute shader (vkQuake's water/lava/slime/teleport warp, and the GPU-compute lightmap path)
 * silently never ran -> warpimages sampled black. The V3D 4.2 CSD hardware is programmed via
 * CSD_QUEUED_CFG0..6 (cfg[0..6] from the submit); writing CFG0 KICKS the dispatch, which raises
 * INT_CSDDONE (core INT, BIT 7) when complete. Mirrors linux v3d_csd_job_run, wrapped in the same
 * coherency bracket ioc_submit_cl uses: drain CPU stores + invalidate slice caches + flush MMU TLB
 * and L2T BEFORE (so the compute reads current inputs), and flush L2T (clean) + TMU-write-combiner
 * AFTER (so the compute's image/SSBO writes reach DRAM and are visible to a CPU readback and the
 * next render job's TMU). BOs are already resident in the flat page table (ioc_create_bo), so no
 * per-submit MMU mapping is needed — this is register writes + a synchronous wait, like CL/TFU. */
static int ioc_submit_csd(struct drm_v3d_submit_csd *s)
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins, sts = 0, csd_status;
	int i, timed_out = 0;

	__asm__ volatile("dsb sy" ::: "memory");
	c0[CTL_SLCACTL / 4] = SLCACTL_INVAL_ALL;
	mmu_flush_tlb(h);
	l2t_flush_wait(c0);
	c0[CTL_L2TCACTL / 4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);

	/* Do NOT kick while a dispatch is still current: V3D 4.2's CSD holds one
	 * CURRENT plus one QUEUED dispatch, so writing CFG0 into a busy unit merely
	 * enqueues behind work we are not waiting on, and our INT_CSDDONE wait then
	 * belongs to the wrong dispatch.
	 *
	 * Measured on hardware (2026-09-03): at a CSD TIMEOUT, CSD_STATUS reads
	 * 0x00000007 = HAVE_QUEUED_DISPATCH | HAVE_CURRENT_DISPATCH | NUM_ACTIVE=1,
	 * NUM_COMPLETED=0 (decoded against linux v3d_regs.h V3D_CSD_STATUS_*). So a
	 * stuck dispatch stayed current and everything piled up behind it: 14
	 * timeouts per bad boot and vkQuake's presentation frozen forever at frame 26
	 * while the engine kept rendering. That is ~3/8 boots.
	 *
	 * Linux never hits this because the DRM scheduler keeps one CSD job in flight
	 * per queue; this poll-based winsys has no such serialization. */
	{
		uint32_t spins = 8000000u;
		while ((c0[CSD_STATUS / 4] & CSD_STATUS_HAVE_CURRENT) != 0u && spins != 0u)
			spins--;
		if (spins == 0u) {
			fprintf(stderr, "v3d-winsys: CSD BUSY before kick (status=0x%08x) -- "
				"resetting so this dispatch is not queued behind stuck work\n",
				c0[CSD_STATUS / 4]);
			reset_reinit_core();
		}
	}

	/* Kick: write CFG1..6, then CFG0 (the CFG0 write starts the dispatch). */
	c0[CTL_INT_CLR / 4] = INT_CSDDONE;
	for (i = 1; i <= 6; i++)
		c0[(CSD_QUEUED_CFG0 + 4u * (uint32_t)i) / 4] = s->cfg[i];
	c0[CSD_QUEUED_CFG0 / 4] = s->cfg[0];

	/* Synchronous wait for the dispatch to finish (INT_CSDDONE), like CL/TFU. */
	/* The budget has to cover V3DV's LARGE compute dispatches, not just the
	 * small ones the standalone probe issues. vkQuake's lightmap passes submit
	 * jobs with CFG0 up to ~0x00570000 (vs 0x00010000 for the probe) and were
	 * hitting this limit hundreds of times per run, each time returning with
	 * num_completed=0 -- so the caller proceeded without its lightmaps and the
	 * world rendered black. Raised 10x; still bounded so a genuinely wedged
	 * dispatch cannot hang the process. */
	for (spins = 80000000u; spins; spins--) {
		sts = c0[CTL_INT_STS / 4];
		if (sts & INT_CSDDONE)
			break;
	}
	if (!spins)
		timed_out = 1;
	csd_status = c0[CSD_STATUS / 4];
	c0[CTL_INT_CLR / 4] = INT_CSDDONE;

	/* Write back the compute's dirty L2T lines to DRAM so the output is visible to
	 * the CPU. Match linux v3d_clean_caches (v3d_gem.c): FIRST drain the TMU
	 * write-combiner into L2T with TMUWCF *alone*, THEN flush L2T to RAM with
	 * L2TFLS + FLM=CLEAN. The prior single combined write (L2TFLS|TMUWCF, FLM=FLUSH)
	 * did NOT make compute TMU-general stores visible — the combiner data hadn't
	 * reached L2T when the flush ran, and FLM=FLUSH invalidates rather than cleans. */
	l2t_flush_wait(c0);                       /* GFXH-1897: pending L2TFLS must be idle */
	c0[CTL_L2TCACTL / 4] = L2TCACTL_TMUWCF;   /* drain TMU write-combiner -> L2T */
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL / 4] & L2TCACTL_TMUWCF); spins--) {}
	c0[CTL_L2TCACTL / 4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN; /* write dirty L2T -> RAM */
	l2t_flush_wait(c0);
	__asm__ volatile("dsb sy" ::: "memory");

	/* Report only on TIMEOUT/error. The former unconditional per-dispatch "CSD done"
	 * line went to UART on every compute dispatch; at serial baud that dominated any
	 * compute-perf measurement (an empty kernel timed slower than a real matmul).
	 * Gated now that CSD dispatch is validated (matches rpi4-v3d.c v3d_gpu.c). */
	if (timed_out) {
		fprintf (stderr, "v3d-winsys: CSD TIMEOUT cfg0=0x%08x int_sts=0x%08x status=0x%08x num_completed=%u\n",
		         s->cfg[0], sts, csd_status, (csd_status >> 4) & 0xffu);
		/* Drain the unit. Leaving the timed-out dispatch CURRENT is what turned a
		 * single slow dispatch into a permanent freeze: every later dispatch
		 * queued behind it and never ran. Resetting costs one dispatch; not
		 * resetting cost the whole process's presentation. */
		if ((csd_status & (CSD_STATUS_HAVE_CURRENT | CSD_STATUS_HAVE_QUEUED)) != 0u) {
			fprintf(stderr, "v3d-winsys: CSD still busy after timeout -- reset to drain\n");
			reset_reinit_core();
		}
	}
	return 0;
}

/* The single entry the libdrm shim's drmIoctl() dispatches into. */
int phoenix_v3d_ioctl(int fd, unsigned long request, void *arg);

/* Thin locking wrapper. The body is a separate function so that no early return
 * inside it can leak the lock -- there are many, and adding an unlock to each is
 * exactly the sort of edit that goes wrong later. */
static int v3d_ioctl_locked(int fd, unsigned long request, void *arg);

int phoenix_v3d_ioctl(int fd, unsigned long request, void *arg)
{
	int ret;

	/* GET_PARAM and WAIT_BO return constants and touch neither MMIO nor global
	 * state, so they stay outside the lock: serializing them would put every
	 * screen-create query behind in-flight GPU work for no benefit. */
	unsigned cmd_nr = _IOC_NR(request) - DRM_COMMAND_BASE;
	if ((cmd_nr == DRM_V3D_GET_PARAM) || (cmd_nr == DRM_V3D_WAIT_BO)) {
		return v3d_ioctl_locked(fd, request, arg);
	}

	v3d_submit_lock_acquire();
	ret = v3d_ioctl_locked(fd, request, arg);
	v3d_submit_lock_release();
	return ret;
}

static int v3d_ioctl_locked(int fd, unsigned long request, void *arg)
{
	(void)fd;
	/* Mesa builds requests as DRM_IOWR(DRM_COMMAND_BASE + DRM_V3D_*, ...), so the
	 * ioctl NR field is DRM_COMMAND_BASE (0x40) + the bare command. Strip the base
	 * to recover the DRM_V3D_* command our cases are keyed on. (Missing this made
	 * every ioctl fall through to the default -> GET_PARAM returned 0 -> ver=0 ->
	 * v3d_get_device_info failed -> v3d_screen_create NULL.) */
	unsigned cmd = _IOC_NR(request) - DRM_COMMAND_BASE;
	/* GET_PARAM / WAIT_BO return constants and touch no MMIO — serve them WITHOUT
	 * winsys_init() so screen-create (which is GET_PARAM-only, allocates no BO) needs
	 * no V3D power-on / MMU bring-up. The MMIO paths below init lazily. */
	switch (cmd) {
	case DRM_V3D_GET_PARAM:
		return ioc_get_param(arg);
	case DRM_V3D_WAIT_BO:
		return 0;   /* submit is synchronous */
	}
	/* Everything below touches HUB/CORE MMIO + the MMU PT -> requires power-on. */
	if (winsys_init() != 0)
		return -1;
	/* DRM core GEM_CLOSE (NR 0x09, below DRM_COMMAND_BASE so not a DRM_V3D_* cmd):
	 * Mesa's bufmgr issues it to free a BO; reclaim the slot + GPU VA. */
	if (_IOC_NR(request) == _IOC_NR(DRM_IOCTL_GEM_CLOSE))
		return ioc_close_bo(arg);
	switch (cmd) {
	case DRM_V3D_CREATE_BO:
		return ioc_create_bo(arg);
	case DRM_V3D_GET_BO_OFFSET: {
		struct drm_v3d_get_bo_offset *g = arg;
		struct pbo *b = bo_find(g->handle);
		if (!b)
			return -EINVAL;
		g->offset = b->gpuva;
		return 0;
	}
	case DRM_V3D_MMAP_BO: {
		/* Our BOs are already CPU-mapped (uncached); return the va as the offset
		 * and have the libdrm-shim mmap() return it directly. */
		struct drm_v3d_mmap_bo *m = arg;
		struct pbo *b = bo_find(m->handle);
		if (!b)
			return -EINVAL;
		m->offset = (uint64_t)(uintptr_t)b->cpu;
		b->nmaps++;
		if (bo_trace_on() != 0) {
			fprintf(stderr, "v3d-bo: MMAP   handle=%u gpuva=0x%08x size=%u cpu=%p\n",
				b->handle, b->gpuva, b->size, b->cpu);
		}
		return 0;
	}
	case DRM_V3D_SUBMIT_CL:
		return ioc_submit_cl(arg);
	case DRM_V3D_SUBMIT_TFU:
		return ioc_submit_tfu(arg);
	case DRM_V3D_SUBMIT_CSD:
		return ioc_submit_csd(arg);
	default:
		return 0;   /* perfmon: no-op */
	}
}
