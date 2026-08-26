/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 GPU server - GPU-owning + BO-management core
 *
 * The GPU-owning + BO-lifecycle logic for the rpi4-v3d daemon, copied
 * essentially verbatim from the in-process winsys backend
 * (tools/v3d-driver-port/v3d_phoenix_winsys.c, the register #defines, state
 * struct W, va_alloc/va_free/bo_find, apply_core_regs and the ioc_create_bo /
 * ioc_close_bo bodies) and its self-contained BCM2711 power-on
 * (tools/v3d-driver-port/v3d_phoenix_power.c: mboxProp / asbEnable /
 * v3d_phoenix_powerOn). That code is all process-local static state in the
 * winsys, so moving it into the sole GPU owner is a copy, not a rewrite; the
 * winsys copy stays byte-for-byte untouched and keeps working in-process.
 *
 * Only what BO management needs was copied: power-on, register map, the single
 * flat MMU page table + GPU-VA allocator + BO table, and the four BO ioctls.
 * The submit path (ioc_submit_cl/tfu/csd), the wedge reset/recovery
 * (reset_reinit_core), the scanout/present layer and the disproved render-stall
 * diagnostics were deliberately left behind - step 2b lifts the submit path.
 * The scanout fields of W and the scanout branch of ioc_create_bo are kept
 * verbatim but stay dormant (W.scanout_pa is never set here), so the copy does
 * not diverge from the winsys source.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include "v3d_drm.h"   /* vendored UAPI: drm_v3d_* arg structs + drm_gem_close */
#include "v3d_gpu.h"

/* ========================================================================= */
/* Register map + tunables - copied verbatim from v3d_phoenix_winsys.c.       */
/* ========================================================================= */

/* V3D 4.2 MMIO (ARM low-peri), HUB + CORE0 - see rpi4-v3d-scout. */
#define V3D_HUB_BASE        0xfec00000u
#define V3D_MMIO_LEN        0x10000u
#define V3D_CORE0_OFFS      0x4000u
/* MMU (HUB-relative) */
#define MMU_PT_PA_BASE      0x1204u
#define MMU_CTL             0x1200u
#define MMU_CTL_ENABLE      (1u<<0)
#define MMU_CTL_PTI_ABORT   (1u<<19)   /* = PT_INVALID_ABORT */
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
#define PTE_W               (1u<<29)
#define PTE_V               (1u<<28)
#define PAGE_SHIFT          12u
/* CORE0-relative submit/sync + CSD dispatch (V3D 4.2, ver<71) - see winsys. */
#define CTL_INT_STS         0x0050u
#define CTL_INT_CLR         0x0058u
#define INT_CSDDONE         (1u<<7)   /* compute-shader dispatch done */
#define CSD_STATUS          0x0900u   /* NUM_COMPLETED[11:4], HAVE_CURRENT(1), HAVE_QUEUED(0) */
#define CSD_QUEUED_CFG0     0x0904u   /* CFG1..6 follow at +4 each; CFG0 write kicks the job */
#define CTL_L2TCACTL        0x0030u
#define L2TCACTL_L2TFLS     (1u<<0)
#define L2TCACTL_FLM_CLEAN  (2u<<1)    /* FLM field = CLEAN (write back dirty L2T lines to RAM) */
#define L2TCACTL_TMUWCF     (1u<<8)    /* TMU write-combiner flush (drain partial tiled writes) */
#define CTL_SLCACTL         0x0024u    /* slices cache control (V3D 4.x) */
#define SLCACTL_INVAL_ALL   0x0f0f0f0fu /* invalidate TVCCS/TDCCS/UCC(uniform)/ICC(instr) */
/* CORE0-relative core init regs (see winsys apply_core_regs). */
#define CTL_L2CACTL         0x0020u    /* general L2 cache control (V3D 4.x) */
#define L2CACTL_L2CENA      (1u<<0)    /* enable the L2 cache */
#define L2CACTL_L2CCLR      (1u<<2)    /* clear the L2 cache */
#define CTL_L2TFLSTA        0x0034u    /* L2T flush start address */
#define CTL_L2TFLEND        0x0038u    /* L2T flush end address */
/* HUB block: the AXI config (GFXH-1383 - cap the V3D AXI master's max burst). */
#define HUB_AXICFG          0x0000u
#define HUB_AXICFG_MAX_LEN  0x0000000fu
#define CTL_MISCCFG         0x0018u
#define MISCCFG_OVRTMUOUT   (1u<<0)
#define MISCCFG_QRMAXCNT_SHIFT 1u    /* QRMAXCNT = MISCCFG bits 3:1 (QPU reserve bin-vs-render split) */
#define V3D_QRMAXCNT        (2)
/* CL (render) submit: CT0 (bin) / CT1 (render) queue regs + binner overflow, CORE0-relative. */
#define INT_FRDONE          (1u<<0)
#define INT_FLDONE          (1u<<1)
#define INT_OUTOMEM         (1u<<2)   /* binner exhausted its tile-allocation pool */
#define CLE_CT0QTS          0x015cu
#define CT0QTS_ENABLE       (1u<<1)
#define CLE_CT0QBA          0x0160u
#define CLE_CT1QBA          0x0164u
#define CLE_CT0QEA          0x0168u
#define CLE_CT1QEA          0x016cu
#define CLE_CT0QMA          0x0170u
#define CLE_CT0QMS          0x0174u
#define PTB_BPCA            0x0300u   /* binner primitive-list current address */
#define PTB_BPCS            0x0304u   /* binner primitive-list current status */
#define PTB_BPOA            0x0308u   /* binner pool overflow address (GPU VA) */
#define PTB_BPOS            0x030cu   /* binner pool overflow size (bytes) */
/* GMP (global memory protection) - used by the AXI drain in the reset path. */
#define GMP_STATUS          0x0800u
#define GMP_CFG             0x0804u
#define GMP_CFG_STOP_REQ    (1u<<1)   /* request the GMP to quiesce outstanding AXI transactions */
#define GMP_STATUS_RD_WR_CNT 0x7f7f0000u /* RD_COUNT(22:16)|WR_COUNT(30:24) - nonzero = txns in flight */
#define GMP_STATUS_CFG_BUSY (1u<<3)
/* HUB interrupt status/clear + TFU (HUB block; h[] not c0[]). TFU raises HUB_INT bit1. */
#define HUB_INT_STS         0x0050u
#define HUB_INT_CLR         0x0058u
#define HUB_INT_MSK_STS     0x005cu   /* mask status (diagnostic only; STS is raw) */
#define HUB_INT_TFUC        (1u<<1)   /* TFU conversion complete */
#define HUB_INT_TFUF        (1u<<0)   /* TFU conversion failed */
/* Texture Formatting Unit (HUB block, V3D 4.2 / ver<71) - see winsys / linux v3d_tfu_job_run. */
#define TFU_CS              0x0400u   /* control/status: bit0 BUSY */
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

/* Binner tile-allocation overflow/spill pool. Pre-allocated at init to match the
 * winsys winsys_init ordering exactly (it maps this pool into the flat MMU right
 * after apply_core_regs); it is consumed only by the submit path, which step 2b
 * lifts. Kept here so 2b re-inserts the submit path into an unmodified init. */
#define BINOVF_PAGES        8192u     /* 32 MiB persistent binner-overflow/spill pool */
#define BINOVF_CHUNK_BYTES  (BINOVF_PAGES * 4096u)

/* GPU VA space: bump-allocate page-aligned, starting past the null guard. */
#define GPUVA_BASE          0x100000u
#define V3D_VA_NO_RECYCLE   0
#if V3D_VA_NO_RECYCLE
#define GPUVA_PT_PAGES      512u   /* 512 * 4 MiB = 2 GiB monotonic VA window (no reclaim) */
#else
#define GPUVA_PT_PAGES      64u    /* 64 * 4 MiB = 256 MiB GPU VA window */
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
};

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
	uint32_t scanout_pa2;     /* multi-buffer: buffer 1 PA */
	uint32_t scanout_pa3;     /* triple-buffer: buffer 2 PA, else 0 */
	uint32_t scanout_phys_h;  /* physical (displayed) height */
	uint32_t scanout_bytes;   /* one buffer's byte size (pitch*phys_h) */
	uint32_t scanout_disp_off; /* byte offset of the currently-displayed buffer */
	int      scanout_nbuf;    /* number of page-flip buffers granted */
	int      scanout_double;  /* convenience: scanout_nbuf >= 2 (page-flip available) */
	int      scanout_claim_idx; /* multi-buffer: which buffer the next scanout BO is backed by */
	int      scanout_claimed; /* single-buffer: only one BO may alias the single scanout surface */
	int      next_scanout;    /* one-shot: back the NEXT create_bo with the scanout surface */
	struct pbo bos[MAX_BOS];
	uint32_t nbos;            /* high-water mark of slots ever used */
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


/* ========================================================================= */
/* BCM2711 V3D power-on - copied verbatim from v3d_phoenix_power.c            */
/* (only the power-on path; the reset/coldstate/fb-flip helpers stay behind). */
/* ========================================================================= */

/* firmware mailbox (property channel) */
#define RPI_MAILBOX_BASE        0xfe00b880u
#define VC_MBOX_STATUS          0x18u
#define VC_MBOX_WRITE           0x20u
#define VC_MBOX_READ            0x00u
#define VC_MBOX_STATUS_FULL     0x80000000u
#define VC_MBOX_STATUS_EMPTY    0x40000000u
#define VC_MBOX_RESP_OK         0x80000000u
#define VC_MBOX_PROP_CHANNEL    8u
#define MBOX_FAIL               0xffffffffu
#define MBOX_SPINS              4000000u
#define VC_PROP_SET_QPU_ENABLE   0x00030012u
#define VC_PROP_SET_DOMAIN_STATE 0x00038030u
#define RPI_POWER_DOMAIN_V3D     10u
#define VC_PROP_SET_CLOCK_STATE  0x00038001u
#define RPI_CLOCK_V3D            5u
/* PM + rpivid_asb (the BCM2711 V3D power/reset path) */
#define PM_BASE                 0xfe100000u
#define RPIVID_ASB_BASE         0xfec11000u
#define PM_GRAFX                0x10cu
#define PM_V3DRSTN              (1u << 6)
#define ASB_V3D_S_CTRL          0x08u
#define ASB_V3D_M_CTRL          0x0cu
#define ASB_REQ_STOP            (1u << 0)
#define ASB_ACK                 (1u << 1)
#define PM_PASSWORD             0x5a000000u
#define ASB_ACK_SPINS           100000u

static uint32_t mboxProp(uint32_t tag, int nw, uint32_t w0, uint32_t w1)
{
	addr_t pa_base = (addr_t)RPI_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request, result = MBOX_FAIL, spins;
	void *mbox_page, *msg_page;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, pa_base);
	if (mbox_page == MAP_FAILED)
		return MBOX_FAIL;
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return MBOX_FAIL;
	}
	msg = msg_page;
	msg[0] = (uint32_t)(6 + nw) * 4u;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = (uint32_t)nw * 4u;
	msg[4] = 0;
	msg[5] = w0;
	if (nw > 1)
		msg[6] = w1;
	msg[5 + nw] = 0;

	msg_pa = (uintptr_t)va2pa(msg);
	if (msg_pa == (uintptr_t)-1) {
		munmap(msg_page, _PAGE_SIZE);
		munmap(mbox_page, _PAGE_SIZE);
		return MBOX_FAIL;
	}
	request = ((uint32_t)msg_pa & ~0xFu) | VC_MBOX_PROP_CHANNEL;

	for (spins = MBOX_SPINS; (mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_FULL) != 0u; spins--) {
		if (spins == 0u) {
			munmap(msg_page, _PAGE_SIZE);
			munmap(mbox_page, _PAGE_SIZE);
			return MBOX_FAIL;
		}
	}
	mbox[VC_MBOX_WRITE / 4] = request;
	for (spins = MBOX_SPINS; spins != 0u; spins--) {
		if ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) == 0u &&
		    mbox[VC_MBOX_READ / 4] == request)
			break;
	}
	if (spins != 0u && msg[1] == VC_MBOX_RESP_OK)
		result = msg[5 + nw - 1];

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}

static int asbEnable(volatile uint32_t *asb, uint32_t reg)
{
	uint32_t val = asb[reg / 4] & ~ASB_REQ_STOP;
	uint32_t spins;
	asb[reg / 4] = PM_PASSWORD | val;
	for (spins = ASB_ACK_SPINS; spins != 0u; spins--) {
		if ((asb[reg / 4] & ASB_ACK) == 0u)
			return 0;
	}
	return -1;
}

/* Stop a V3D async-AXI bridge (set REQ_STOP, wait for ACK) - the power-off direction,
 * mirror of asbEnable. Used by v3d_gpu_reset to quiesce the bridges before reset. */
static int asbStop(volatile uint32_t *asb, uint32_t reg)
{
	uint32_t val = asb[reg / 4] | ASB_REQ_STOP;
	uint32_t spins;
	asb[reg / 4] = PM_PASSWORD | val;
	for (spins = ASB_ACK_SPINS; spins != 0u; spins--) {
		if ((asb[reg / 4] & ASB_ACK) != 0u)
			return 0;
	}
	return -1;
}

/* Full HW-proven V3D power-on. Returns 0 on success (both ASB bridges ACK). */
static int v3d_gpu_powerOn(void)
{
	volatile uint32_t *pm, *asb;
	void *pm_page, *asb_page;
	uint32_t grafx;
	int rcM, rcS;

	/* firmware-side enables (QPU + power domain + clock) */
	(void)mboxProp(VC_PROP_SET_QPU_ENABLE, 1, 1u, 0u);
	(void)mboxProp(VC_PROP_SET_DOMAIN_STATE, 2, RPI_POWER_DOMAIN_V3D, 1u);
	(void)mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);

	pm_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)PM_BASE);
	if (pm_page == MAP_FAILED)
		return -1;
	pm = (volatile uint32_t *)pm_page;
	asb_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)RPIVID_ASB_BASE);
	if (asb_page == MAP_FAILED) {
		munmap(pm_page, _PAGE_SIZE);
		return -1;
	}
	asb = (volatile uint32_t *)asb_page;

	/* clock-toggle around the reset deassert (canonical bcm2835_asb_power_on) */
	(void)mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);
	usleep(50);
	(void)mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 0u);
	grafx = pm[PM_GRAFX / 4];
	pm[PM_GRAFX / 4] = PM_PASSWORD | (grafx | PM_V3DRSTN);
	(void)mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);
	usleep(50);

	rcM = asbEnable(asb, ASB_V3D_M_CTRL);
	rcS = asbEnable(asb, ASB_V3D_S_CTRL);
	printf("rpi4-v3d: powerOn PM_GRAFX 0x%08x->0x%08x asb M=%s S=%s\n",
	       grafx, pm[PM_GRAFX / 4], rcM ? "TIMEOUT" : "ok", rcS ? "TIMEOUT" : "ok");
	usleep(2000);
	munmap(asb_page, _PAGE_SIZE);
	munmap(pm_page, _PAGE_SIZE);
	return (rcM == 0 && rcS == 0) ? 0 : -1;
}

/* TRUE V3D reset cycle (copied from v3d_phoenix_power.c:v3d_phoenix_reset): quiesce the
 * AXI bridges, assert PM_V3DRSTN (hold the V3D in reset), then power back on. Used by the
 * CL wedge-recovery path (reset_reinit_core). Returns 0 on success. */
static int v3d_gpu_reset(void)
{
	volatile uint32_t *pm, *asb;
	void *pm_page, *asb_page;
	uint32_t grafx;

	pm_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)PM_BASE);
	if (pm_page == MAP_FAILED)
		return -1;
	pm = (volatile uint32_t *)pm_page;
	asb_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)RPIVID_ASB_BASE);
	if (asb_page == MAP_FAILED) {
		munmap(pm_page, _PAGE_SIZE);
		return -1;
	}
	asb = (volatile uint32_t *)asb_page;

	(void)asbStop(asb, ASB_V3D_M_CTRL);
	(void)asbStop(asb, ASB_V3D_S_CTRL);
	grafx = pm[PM_GRAFX / 4];
	pm[PM_GRAFX / 4] = PM_PASSWORD | (grafx & ~PM_V3DRSTN);   /* assert reset (hold in reset) */
	usleep(100);

	munmap(asb_page, _PAGE_SIZE);
	munmap(pm_page, _PAGE_SIZE);

	/* power back on (clock toggle + RSTN deassert + ASB enable) */
	return v3d_gpu_powerOn();
}


/* ========================================================================= */
/* GPU-VA allocator + BO table - copied verbatim from v3d_phoenix_winsys.c.    */
/* ========================================================================= */

static struct pbo *bo_find(uint32_t handle)
{
	if (handle == 0 || handle > W.nbos) return NULL;
	struct pbo *b = &W.bos[handle - 1];   /* handle == slot index + 1 */
	return (b->used && b->handle == handle) ? b : NULL;
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
	/* DAEMON-SPECIFIC (diverges from the in-process winsys): clear the PTEs of this
	 * freshly bump-allocated range before returning. On HW the daemon's PT-backing
	 * DRAM was observed NON-zero at first-ever-used client VAs (the 2b run logged
	 * false "VA COLLISION" at 0x2100000+ with garbage PTEs, unlike the in-process
	 * winsys which reads them zero) - i.e. the one-time init clear did not survive
	 * to first use in the standalone server process. Clearing the exact range at
	 * hand-out, immediately before the caller's collision check + PTE writes,
	 * guarantees a never-before-used VA always presents invalid PTEs. The hole-reuse
	 * path above needs no clear: va_free() already zeroed those PTEs. This makes the
	 * collision detector report only genuine live-BO overlaps and ensures the binner
	 * can never fetch a stale PTE for a freshly-mapped tile-list/overflow BO. */
	for (uint32_t i = 0; i < pages; i++)
		W.pt[(va >> PAGE_SHIFT) + i] = 0;
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


/* ========================================================================= */
/* Core register bring-up - copied verbatim from v3d_phoenix_winsys.c         */
/* apply_core_regs (M0 pid/theft instrumentation dropped - this daemon is the  */
/* sole writer of MMU_PT_PA_BASE by construction).                            */
/* ========================================================================= */

static void apply_core_regs(void)
{
	W.hub[MMU_PT_PA_BASE/4] = (uint32_t)(W.pt_pa>>PAGE_SHIFT);
	/* Full MMU fault config (mirror linux v3d_mmu_set_page_table): enable PT-invalid
	 * detection (not just abort), write-violation and cap-exceeded aborts+INTs. */
	W.hub[MMU_CTL/4] = MMU_CTL_ENABLE | MMU_CTL_PTI_ENABLE | MMU_CTL_PTI_ABORT | MMU_CTL_PTI_INT |
		MMU_CTL_WRITEVIO_ABORT | MMU_CTL_WRITEVIO_INT |
		MMU_CTL_CAPEXC_ABORT | MMU_CTL_CAPEXC_INT;
	/* Arm the illegal-access scratch page: a faulting GPU access is redirected here
	 * (harmless mapped DRAM) instead of stalling the bus. */
	if (W.scratch_pa)
		W.hub[MMU_ILLEGAL_ADDR/4] = (uint32_t)(W.scratch_pa>>PAGE_SHIFT) | MMU_ILLEGAL_ENABLE;
	W.hub[MMUC_CONTROL/4] = MMUC_ENABLE;
	W.core0[CTL_L2CACTL/4] = L2CACTL_L2CCLR | L2CACTL_L2CENA;
	/* Define the L2T flush range as the WHOLE cache (matches linux v3d core init). */
	W.core0[CTL_L2TFLSTA/4] = 0u;
	W.core0[CTL_L2TFLEND/4] = ~0u;
	/* GFXH-1383: cap the V3D AXI master's max burst length (linux restores this after
	 * every bridge reset). */
	W.hub[HUB_AXICFG/4] = HUB_AXICFG_MAX_LEN;
#if (V3D_QRMAXCNT) >= 0
	/* MISCCFG = QRMAXCNT (QPU bin/render split) | OVRTMUOUT (required by our Mesa build). */
	W.core0[CTL_MISCCFG/4] = ((uint32_t)(V3D_QRMAXCNT) << MISCCFG_QRMAXCNT_SHIFT) | MISCCFG_OVRTMUOUT;
#endif
}


/* ========================================================================= */
/* GPU ownership init - adapted from v3d_phoenix_winsys.c:winsys_init.        */
/* Same ordering: power-on -> map regs -> install flat PT + scratch ->         */
/* apply_core_regs -> arm VA allocator -> pre-map the binner-overflow pool.    */
/* Diagnostics-only steps (v3d_phoenix_logColdState, the disproved render-     */
/* stall cold-state probe) are dropped; the scanout layer is not set up here.  */
/* ========================================================================= */

int v3d_gpu_init(void)
{
	if (W.inited) return 0;

	/* Power on the V3D ourselves (self-contained, idempotent). */
	if (v3d_gpu_powerOn() != 0) {
		fprintf(stderr, "rpi4-v3d: v3d powerOn FAILED (ASB bridge did not ACK)\n");
		return -EIO;
	}
	W.hub = map_dev(V3D_HUB_BASE, V3D_MMIO_LEN);
	if (!W.hub) return -ENOMEM;
	W.core0 = W.hub + (V3D_CORE0_OFFS/4);
	fprintf(stderr, "rpi4-v3d: V3D up CORE0_IDENT0=0x%08x HUB_IDENT1=0x%08x\n",
		W.core0[0x0000/4], W.hub[0x000c/4]);

	/* MMU page table: GPUVA_PT_PAGES contiguous pages = GPUVA_PT_PAGES*4 MiB GPU VA. */
	W.pt = mmap(NULL, GPUVA_PT_PAGES*_PAGE_SIZE, PROT_READ|PROT_WRITE,
		MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
	if (W.pt==MAP_FAILED) return -ENOMEM;
	W.pt_pa = (uintptr_t)va2pa((void*)W.pt);
	for (uint32_t i=0;i<GPUVA_PT_ENTRIES;i++) W.pt[i]=0;
	/* MMU illegal-access scratch page (linux v3d mmu_scratch): one harmless page the MMU
	 * redirects faulting accesses to, armed via MMU_ILLEGAL_ADDR in apply_core_regs. */
	{
		void *sp = mmap(NULL, _PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
		if (sp != MAP_FAILED) {
			memset(sp, 0, _PAGE_SIZE);
			W.scratch_pa = (uintptr_t)va2pa(sp);
		}
	}
	apply_core_regs();
	W.next_gpuva = GPUVA_BASE;

	/* Pre-allocate the persistent binner-overflow pool (uncached DMA, like the CL/tile
	 * BOs): contiguous pages mapped into the flat MMU at a stable GPU VA. Consumed only
	 * by the submit path (step 2b); mapped here so init ordering matches the winsys. */
	{
		uint32_t gpuva = va_alloc(BINOVF_PAGES);
		void *cpu = (gpuva == 0) ? MAP_FAILED :
			mmap(NULL, BINOVF_PAGES*_PAGE_SIZE, PROT_READ|PROT_WRITE,
				MAP_UNCACHED|MAP_CONTIGUOUS|MAP_ANONYMOUS, -1, 0);
		if (cpu != MAP_FAILED) {
			/* Zero the pool: Phoenix mmap(MAP_CONTIGUOUS) returns NON-zeroed DRAM. */
			memset(cpu, 0, BINOVF_PAGES*_PAGE_SIZE);
			for (uint32_t i=0;i<BINOVF_PAGES;i++) {
				uintptr_t ppa = (uintptr_t)va2pa((char*)cpu + (size_t)i*_PAGE_SIZE);
				W.pt[(gpuva>>PAGE_SHIFT)+i] = (uint32_t)(ppa>>PAGE_SHIFT)|PTE_W|PTE_V;
			}
			W.binovf_gpuva = gpuva;
			W.binovf_bytes = BINOVF_PAGES*_PAGE_SIZE;
		}
		else {
			fprintf(stderr, "rpi4-v3d: WARN no binner-overflow pool - large RTs may stall\n");
		}
	}

	/* NOTE (PT-persistence, resolved 2026-08-22 at M3a/M3b HW bring-up): a startup probe
	 * of unallocated PT slots read all-zero on HW → the init full-PT zero persists and the
	 * MMU fault net is intact. The va_alloc clear-at-hand-out (see va_alloc) additionally
	 * guarantees every allocated range is invalid before first use. Probe removed. */
	W.inited = 1;
	return 0;
}


/* ========================================================================= */
/* BO ioctl bodies - copied verbatim from v3d_phoenix_winsys.c.               */
/* ========================================================================= */

static int ioc_create_bo(struct drm_v3d_create_bo *c)
{
	/* A zero-byte BO request (e.g. vkQuake's empty lightstyles buffer) would compute 0 pages
	 * and mmap(len=0) fails with "BO mmap FAILED (0 pages)". Round up to one page so the
	 * handle/GPU-VA are valid (mirrors Mesa always allocating full pages) - a legitimate
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
			fprintf(stderr, "rpi4-v3d: BO table full (%u)\n", (unsigned)MAX_BOS);
			return -ENOMEM;
		}
		W.nbos++;
	}

	gpuva = va_alloc(pages);
	if (gpuva == 0) {
		fprintf(stderr, "rpi4-v3d: GPU VA exhausted (need %u pages; PT window = %u MiB). "
			"Grow GPUVA_PT_PAGES or check for a BO leak.\n", pages, (GPUVA_PT_PAGES*4u));
		return -ENOMEM;
	}
	/* VA-collision detector (behavior-neutral): if any PTE in the new range is already VALID,
	 * a previous BO is still mapped there. Logged, not fixed. */
	for (uint32_t i = 0; i < pages; i++) {
		if (W.pt[(gpuva>>PAGE_SHIFT)+i] & PTE_V) {
			fprintf(stderr, "rpi4-v3d: VA COLLISION new handle gpuva=0x%x page %u already "
				"mapped (PTE=0x%08x) - live-BO overlap\n",
				gpuva, i, W.pt[(gpuva>>PAGE_SHIFT)+i]);
			break;
		}
	}
	/* Select the scanout buffer this BO should alias, or 0 (not a scanout BO). Dormant in the
	 * server: W.scanout_pa is never set here, so sel_pa stays 0 and the default DRAM path runs.
	 * Kept verbatim from the winsys so the copy does not diverge. */
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
		/* Back the visible rows with the scanout buffer's physical pages; extra tile-aligned
		 * rows map to fresh scratch DRAM. */
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
		fprintf(stderr, "rpi4-v3d: RT scanout buf%d PA 0x%08x gpuva 0x%x, %u/%u pages "
			"(%u scratch) - %s\n", W.scanout_double ? W.scanout_claim_idx : 0, sel_pa, gpuva,
			scanout_pages, pages, pages-scanout_pages,
			W.scanout_double ? "double-buffer" : "render-to-scanout");
		if (W.scanout_double) W.scanout_claim_idx++; else W.scanout_claimed = 1;
	}
	else {
		/* Scanout was requested one-shot but couldn't be honored; don't leak the request. */
		W.next_scanout = 0;
		/* Default: uncached contiguous DMA memory. A cacheable BO (flags bit 0 =
		 * V3D_CREATE_BO_CACHEABLE) drops MAP_UNCACHED so the CPU readback hits cache. */
		int mapflags = MAP_CONTIGUOUS | MAP_ANONYMOUS;
		if ((c->flags & 0x1u) == 0u)
			mapflags |= MAP_UNCACHED;
		cpu = mmap(NULL, pages*_PAGE_SIZE, PROT_READ|PROT_WRITE, mapflags, -1, 0);
		if (cpu==MAP_FAILED) {
			/* NOTE: cast vs the verbatim winsys line - _PAGE_SIZE is `unsigned long`
			 * here, and the devices tree builds with -Wall -Werror (the winsys' looser
			 * tools build did not), so the %u arg must be narrowed explicitly. */
			fprintf(stderr, "rpi4-v3d: BO mmap FAILED (%u pages, %u KiB, flags 0x%x)\n",
				pages, (unsigned)(pages*_PAGE_SIZE/1024u), c->flags);
			va_free(gpuva, pages); return -ENOMEM;
		}
		/* Zero freshly-allocated BO memory (Phoenix mmap(MAP_CONTIGUOUS) returns NON-zeroed
		 * DRAM; a binner-output BO with cold-boot garbage makes CT1 wedge). */
		memset(cpu, 0, pages*_PAGE_SIZE);
		pa = (uintptr_t)va2pa(cpu);
		/* Map each page by its ACTUAL physical address (a cacheable mapping may not be
		 * physically contiguous; per-page va2pa is correct either way). */
		for (uint32_t i=0;i<pages;i++) {
			uintptr_t ppa = (uintptr_t)va2pa((char*)cpu + (size_t)i*_PAGE_SIZE);
			W.pt[(gpuva>>PAGE_SHIFT)+i] = (uint32_t)(ppa>>PAGE_SHIFT)|PTE_W|PTE_V;
		}
	}

	struct pbo *b = &W.bos[slot];
	b->used = 1;
	b->handle = slot + 1;       /* nonzero, stable per slot */
	b->cpu = cpu; b->pa = pa; b->gpuva = gpuva; b->size = pages*_PAGE_SIZE;
	b->scanout = (W.scanout_pa != 0 && (pa == W.scanout_pa ||
		(W.scanout_pa2 != 0 && pa == W.scanout_pa2) ||
		(W.scanout_pa3 != 0 && pa == W.scanout_pa3)));   /* this BO aliases a scanout buffer */
	c->handle = b->handle;
	c->offset = gpuva;          /* V3D address-space offset (nonzero) */
	return 0;
}

/* DRM core GEM_CLOSE: free the BO so its slot + GPU VA are reclaimed. */
static int ioc_close_bo(struct drm_gem_close *gc)
{
	struct pbo *b = bo_find(gc->handle);
	if (b == NULL) return 0;   /* already gone / never ours */
	/* If the scanout-backed RT is being freed, release the single-claim so the NEXT
	 * full-screen RT can re-acquire scanout backing. */
	if (b->scanout) {
		if (W.scanout_double) {
			if (W.scanout_claim_idx > 0) W.scanout_claim_idx--;
		}
		else {
			W.scanout_claimed = 0;
		}
		b->scanout = 0;
	}
	va_free(b->gpuva, b->size / _PAGE_SIZE);
	if (b->cpu != NULL) munmap(b->cpu, b->size);
	b->used = 0;
	b->cpu = NULL;
	b->handle = 0;
	return 0;
}


/* ========================================================================= */
/* Exported server API - thin scalar wrappers over the verbatim ioc_* bodies. */
/* ========================================================================= */

int v3d_gpu_createBo(uint32_t size, uint32_t flags, v3d_gpu_bo_t *out)
{
	struct drm_v3d_create_bo c;
	struct pbo *b;
	int rc;

	memset(&c, 0, sizeof(c));
	c.size = size;
	c.flags = flags;
	rc = ioc_create_bo(&c);
	if (rc != 0)
		return rc;
	/* Fetch the BO's PA (ioc_create_bo returns handle + gpuva in the drm struct but keeps
	 * the PA in the BO table). The client shares the BO by PA via mmap(MAP_PHYSMEM). */
	b = bo_find(c.handle);
	if (b == NULL)
		return -EIO;
	out->handle = c.handle;
	out->pa = (uint64_t)b->pa;
	out->size = b->size;
	out->gpuva = c.offset;
	return 0;
}

int v3d_gpu_getBoOffset(uint32_t handle, uint32_t *gpuva)
{
	struct pbo *b = bo_find(handle);
	if (b == NULL)
		return -EINVAL;
	*gpuva = b->gpuva;
	return 0;
}

int v3d_gpu_mmapBo(uint32_t handle, uint64_t *pa, uint32_t *size)
{
	/* The winsys returns the in-process CPU VA here (its libdrm shim mmaps it directly). The
	 * server owns no per-client CPU mapping, so it returns the BO's PA + size and the client
	 * does its own mmap(MAP_PHYSMEM, pa). */
	struct pbo *b = bo_find(handle);
	if (b == NULL)
		return -EINVAL;
	*pa = (uint64_t)b->pa;
	*size = b->size;
	return 0;
}

int v3d_gpu_closeBo(uint32_t handle)
{
	struct drm_gem_close gc;
	memset(&gc, 0, sizeof(gc));
	gc.handle = handle;
	return ioc_close_bo(&gc);
}


/* ========================================================================= */
/* CSD (compute) submit - copied verbatim from v3d_phoenix_winsys.c            */
/* (l2t_flush_wait / mmu_flush_tlb helpers + ioc_submit_csd body). The submit  */
/* consumes only cfg[0..6]; it never touches the binner, overflow pool or the  */
/* reset path, so it lifts cleanly ahead of the CL/TFU paths (step 2c).        */
/* ========================================================================= */

static inline void l2t_flush_wait(volatile uint32_t *c0)
{
	uint32_t spins;
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL/4] & L2TCACTL_L2TFLS); spins--) {}
}

/* Flush the MMU PTE cache + clear the TLB (mirror linux v3d_mmu_flush_all): a job
 * whose BOs were just mapped at new GPU VAs would otherwise be fetched through a
 * stale TLB. */
static void mmu_flush_tlb(volatile uint32_t *h)
{
	uint32_t spins;
	h[MMUC_CONTROL/4] = MMUC_FLUSH | MMUC_ENABLE;
	for (spins = 1000000u; spins && (h[MMUC_CONTROL/4] & MMUC_FLUSHING); spins--) {}
	h[MMU_CTL/4] |= MMU_CTL_TLB_CLEAR;
	for (spins = 1000000u; spins && (h[MMU_CTL/4] & MMU_CTL_TLB_CLEARING); spins--) {}
}

int v3d_gpu_submitCsd(const uint32_t cfg[7])
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins, sts = 0, csd_status;
	int i, timed_out = 0;

	if (!W.inited)
		return -EIO;

	__asm__ volatile("dsb sy" ::: "memory");
	c0[CTL_SLCACTL / 4] = SLCACTL_INVAL_ALL;
	mmu_flush_tlb(h);
	l2t_flush_wait(c0);
	c0[CTL_L2TCACTL / 4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);

	/* Kick: write CFG1..6, then CFG0 (the CFG0 write starts the dispatch). */
	c0[CTL_INT_CLR / 4] = INT_CSDDONE;
	for (i = 1; i <= 6; i++)
		c0[(CSD_QUEUED_CFG0 + 4u * (uint32_t)i) / 4] = cfg[i];
	c0[CSD_QUEUED_CFG0 / 4] = cfg[0];

	/* Synchronous wait for the dispatch to finish (INT_CSDDONE), like CL/TFU. */
	for (spins = 8000000u; spins; spins--) {
		sts = c0[CTL_INT_STS / 4];
		if (sts & INT_CSDDONE)
			break;
	}
	if (!spins)
		timed_out = 1;
	csd_status = c0[CSD_STATUS / 4];
	c0[CTL_INT_CLR / 4] = INT_CSDDONE;

	/* Write back the compute's dirty L2T lines to DRAM so the output is visible to
	 * the CPU. Match linux v3d_clean_caches: FIRST drain the TMU write-combiner into
	 * L2T with TMUWCF *alone*, THEN flush L2T to RAM with L2TFLS + FLM=CLEAN. */
	l2t_flush_wait(c0);                       /* GFXH-1897: pending L2TFLS must be idle */
	c0[CTL_L2TCACTL / 4] = L2TCACTL_TMUWCF;   /* drain TMU write-combiner -> L2T */
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL / 4] & L2TCACTL_TMUWCF); spins--) {}
	c0[CTL_L2TCACTL / 4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN; /* write dirty L2T -> RAM */
	l2t_flush_wait(c0);
	__asm__ volatile("dsb sy" ::: "memory");

	/* Only report the CSD completion on TIMEOUT/error. The former unconditional
	 * per-dispatch "CSD done" line went to UART on EVERY compute dispatch, which at
	 * serial baud (~6 ms/line) dominated any compute-perf measurement (an empty
	 * kernel "measured" slower than a real matmul) and spammed the console during
	 * any GPU-compute workload. The success path is silent now; TIMEOUT still logs. */
	if (timed_out)
		fprintf(stderr, "rpi4-v3d: CSD TIMEOUT cfg0=0x%08x int_sts=0x%08x status=0x%08x num_completed=%u\n",
		        cfg[0], sts, csd_status, (csd_status >> 4) & 0xffu);
	return 0;
}


/* ========================================================================= */
/* CL (render) + TFU submit - copied from v3d_phoenix_winsys.c (step 2c).      */
/*                                                                             */
/* The register/flush/kick/wait sequences (incl. binner OUT-OF-MEMORY          */
/* servicing from the persistent overflow pool, the bin->render coherency      */
/* handoff, and the wedge true-reset mitigation) are lifted VERBATIM. Only the */
/* post-wedge diagnostic DUMP sub-blocks that depended on the winsys           */
/* gpuva_describe / gpuva_to_cpu / bincrc helpers (and the #ifdef VKQ_CPU_TILE */
/* discriminator + the gated TFU striping probe) were trimmed - they run only  */
/* after a wedge/on a debug env and never touch the MMIO sequence, so removing  */
/* them keeps the delicate register path byte-identical while dropping a large  */
/* diagnostic cluster (and its Mesa-free-but-verbose helpers) from the server. */
/* The register-only wedge one-liners (int_sts/ct0ca/ct1ca/gmp/mmu_ill/PTB/    */
/* fdbg) are kept.                                                             */
/* ========================================================================= */

/* Wedge counters (verbatim names; file-static here - no external harness reads them
 * in the server). render_timeouts counts CT1 spin-timeouts; render_recoveries counts
 * true-reset recoveries. */
static volatile unsigned v3d_phoenix_render_timeouts = 0;
static volatile unsigned v3d_phoenix_render_recoveries = 0;

/* Best-effort safe AXI drain before a reset: ask the GMP to quiesce any outstanding
 * transaction (mirror linux v3d_idle_axi). Bounded spin. */
static void idle_axi(volatile uint32_t *c0)
{
	uint32_t spins;
	c0[GMP_CFG/4] = GMP_CFG_STOP_REQ;
	for (spins = 1000000u; spins; spins--) {
		if ((c0[GMP_STATUS/4] & (GMP_STATUS_RD_WR_CNT | GMP_STATUS_CFG_BUSY)) == 0u)
			break;
	}
}

/* Reset the V3D and re-establish core register state (drain GMP, true reset, re-apply
 * core regs over the surviving page table), so the caller can proceed with the next
 * job. Mirrors v3d_phoenix_winsys.c:reset_reinit_core. */
static void reset_reinit_core(void)
{
	if (W.core0)
		idle_axi(W.core0);
	(void)v3d_gpu_reset();
	apply_core_regs();
}

static int ioc_submit_cl(struct drm_v3d_submit_cl *s)
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins;
	int job_failed = 0;     /* set if bin or render wedged */
	int attempt = 0;        /* kept for the timeout-dump "attempt" field (always 0 now: no resubmit) */
	W.binovf_used = 0;      /* re-armable binner overflow: reset the per-job hand-out cursor */
	/* Drain CPU stores into uncached GPU BOs to DRAM BEFORE the first GPU MMIO poke below
	 * (aarch64 Normal-NC vs Device ordering); MUST be dsb (completion), not dmb - the V3D is a
	 * non-coherent external DMA master reading these BOs straight from DRAM. */
	__asm__ volatile("dsb sy" ::: "memory");
	/* #67 ORDERING FIX: issue the fire-and-forget SLCACTL slice-cache invalidate as EARLY as
	 * possible so the subsequent spin-waits become free settle latency before the CT0 kick. */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;
	/* Flush the MMU PTE cache + TLB before the job (fresh PTEs from ioc_create_bo). */
	mmu_flush_tlb(h);
	/* L2T flush (clean+invalidate) then the SLCACTL slice invalidate already issued above -
	 * outside-in order matching linux v3d_invalidate_caches. */
	l2t_flush_wait(c0);                       /* wait-old: prior L2T flush must be idle first */
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);                       /* wait-new: flush must complete before the bin reads its CL/vertex data */
	/* "fix-A": an extra waited-L2T-flush before the CT0 kick - provides timing margin that
	 * suppresses a separate binner->render tile-list wedge (removal regressed on HW). KEEP. */
	l2t_flush_wait(c0);
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS;
	l2t_flush_wait(c0);
	/* --- bin (CT0); wait FLDONE --- */
	c0[CTL_INT_CLR/4] = INT_FLDONE|INT_FRDONE;
	c0[PTB_BPOS/4] = 0;
	if (s->qma) { c0[CLE_CT0QMA/4]=s->qma; c0[CLE_CT0QMS/4]=s->qms; }
	if (s->qts) { c0[CLE_CT0QTS/4]=CT0QTS_ENABLE|s->qts; }
	c0[CLE_CT0QBA/4]=s->bcl_start; c0[CLE_CT0QEA/4]=s->bcl_end;
	/* Wait for bin done, servicing binner OUT-OF-MEMORY from the persistent overflow pool. */
	{
		int ovf_armed = 0;
		uint32_t sts;
		uint32_t last_ca = c0[0x0110/4];   /* ct0ca - frozen = binner wedged (fast wedge detect) */
		unsigned frozen = 0;
		for (spins=8000000u; spins; spins--) {
			sts = c0[CTL_INT_STS/4];
			if (sts & INT_FLDONE) break;
			/* Re-armable overflow servicer: hand successive chunks of the persistent pool on each
			 * OUTOMEM until it is exhausted (OUTOMEM is edge-signalled, so clear it after each). */
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
					ovf_armed = 2;   /* pool exhausted - record once; binner will wedge */
					fprintf(stderr, "rpi4-v3d: binner overflow pool EXHAUSTED (%u KiB) - grow BINOVF_PAGES\n",
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
			fprintf(stderr, "rpi4-v3d: BIN TIMEOUT int_sts=0x%08x ct0cs=0x%08x "
				"ct0ca=0x%08x[%x..%x] gmp=0x%08x gmpvio=0x%08x mmu_ill=0x%08x ovf_armed=%d (attempt %d)\n",
				c0[CTL_INT_STS/4], c0[0x0100/4], c0[0x0110/4], s->bcl_start, s->bcl_end,
				c0[GMP_STATUS/4], c0[0x0808/4], W.hub[MMU_ILLEGAL_ADDR/4], ovf_armed, attempt);
			fprintf(stderr, "rpi4-v3d: BIN PTB bpca=0x%08x bpcs=0x%08x bpoa=0x%08x bpos=0x%08x "
				"hub_axicfg=0x%08x int_qpu=0x%03x\n",
				c0[PTB_BPCA/4], c0[PTB_BPCS/4], c0[PTB_BPOA/4], c0[PTB_BPOS/4],
				W.hub[HUB_AXICFG/4], (c0[CTL_INT_STS/4] >> 16) & 0xfffu);
		}
	}
	/* If the binner wedged, don't kick the render against a bad tile state - reset+retry. */
	if (job_failed)
		goto job_retry;
	c0[CTL_INT_CLR/4]=INT_FLDONE|INT_FRDONE;
	/* Bin->render coherency handoff: clean + WAIT (binner tile-list output to RAM), then
	 * invalidate the render-side slice caches, then kick CT1. */
	l2t_flush_wait(c0);                       /* wait-old: any prior L2T flush must be idle */
	c0[CTL_L2TCACTL/4]=L2TCACTL_L2TFLS;       /* clean the binner's tile-list output to RAM */
	l2t_flush_wait(c0);                       /* wait-new: it must COMPLETE before CT1 fetches */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;    /* then drop stale render-side slice-cache lines */
	/* --- render (CT1); wait FRDONE --- */
	c0[CLE_CT1QBA/4]=s->rcl_start; c0[CLE_CT1QEA/4]=s->rcl_end;
	{
		uint32_t last_ca = c0[0x0114/4];
		unsigned frozen = 0;
		for (spins = 16000000u; spins; spins--) {
			if (c0[CTL_INT_STS/4] & INT_FRDONE)
				break;                                  /* render done */
			if ((spins & 0xfffffu) == 0u) {             /* sample ct1ca ~every 1M spins (~160 ms) */
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
		v3d_phoenix_render_timeouts++;   /* stall counter */
		job_failed = 1;
		fprintf(stderr, "rpi4-v3d: RENDER TIMEOUT int_sts=0x%08x ct1cs=0x%08x "
			"ct1ca=0x%08x[%x..%x] ct1ea=0x%08x gmp=0x%08x gmpvio=0x%08x mmu_ill=0x%08x\n",
			c0[CTL_INT_STS/4], c0[0x0104/4], ca1, s->rcl_start, s->rcl_end,
			c0[0x010c/4], c0[0x0800/4], c0[0x0808/4], W.hub[MMU_ILLEGAL_ADDR/4]);
		/* V3D error-debug registers localize the wedged render pipeline stage. */
		fprintf(stderr, "rpi4-v3d: RENDER DBG fdbgo=0x%08x fdbgs=0x%08x errstat=0x%08x "
			"ct1ca recheck=0x%08x\n",
			c0[0x0f04/4], c0[0x0f10/4], c0[0x0f20/4], c0[0x0114/4]);
	}
job_retry:
	/* MITIGATION. The wedge is DATA-dependent (re-submitting the same frame re-hangs across
	 * true resets, HW-confirmed), so do NOT re-submit: do ONE true reset to clean the core for
	 * the next (different) frame and DROP this frame. */
	if (job_failed) {
		v3d_phoenix_render_recoveries++;
		fprintf(stderr, "rpi4-v3d: GPU wedged - true reset + drop this frame "
			"(mitigation; drops=%u).\n", v3d_phoenix_render_recoveries);
		reset_reinit_core();   /* clean the wedged core so the next (different) frame renders */
		(void)attempt;
	}
	/* L2T flush so RT stores reach RAM before CPU readback (scout finding). */
	l2t_flush_wait(c0);                       /* GFXH-1897: render flush must complete first */
	c0[CTL_L2TCACTL/4]=L2TCACTL_L2TFLS|L2TCACTL_FLM_CLEAN;
	return 0;
}

static int ioc_submit_tfu(struct drm_v3d_submit_tfu *t)
{
	volatile uint32_t *c0 = W.core0;
	volatile uint32_t *h = W.hub;
	uint32_t spins;

	/* --- prologue: make the source coherent + translations fresh (mirror the CL pre-bin
	 * sequence): flush MMU TLB, invalidate slice caches, flush L2T + WAIT. --- */
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

	/* --- wait for done. Primary: HUB_INT TFUC/TFUF (sticky, raw STS). Fallback: CS BUSY
	 * clearing AFTER we observed it set (mask-independent). Bounded spin. --- */
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
			fprintf(stderr, "rpi4-v3d: TFU TIMEOUT/FAIL hub_int=0x%08x mskts=0x%08x cs=0x%08x "
				"iia=0x%08x ioa=0x%08x ios=0x%08x icfg=0x%08x\n",
				isr, h[HUB_INT_MSK_STS/4], h[TFU_CS/4], t->iia, t->ioa, t->ios, t->icfg);
			/* Don't abort the client - return success so rendering proceeds (a failed upload
			 * leaves the image zero, same as before, just visible in the log). */
		}
	}

	/* --- epilogue: make the TFU-written tiled image visible to the TMU. Mirror linux
	 * v3d_clean_caches EXACTLY: wait any in-flight L2T flush (GFXH-1897), TMU write-combiner
	 * flush + WAIT, L2T clean + WAIT, then slice invalidate. --- */
	l2t_flush_wait(c0);                                  /* GFXH-1897: any prior L2T flush idle */
	c0[CTL_L2TCACTL/4] = L2TCACTL_TMUWCF;                /* drain the TMU write combiner... */
	for (spins = 1000000u; spins && (c0[CTL_L2TCACTL/4] & L2TCACTL_TMUWCF); spins--) {}  /* ...and wait */
	c0[CTL_L2TCACTL/4] = L2TCACTL_L2TFLS | L2TCACTL_FLM_CLEAN;   /* write back dirty L2T lines... */
	l2t_flush_wait(c0);                                  /* ...and wait for the clean to complete */
	c0[CTL_SLCACTL/4] = SLCACTL_INVAL_ALL;              /* drop stale read-only slice/TMU cache view */
	return 0;
}

int v3d_gpu_submitCl(const struct drm_v3d_submit_cl *s)
{
	int rc;
	static unsigned cl_n = 0;

	if (!W.inited)
		return -EIO;
	/* ioc_submit_cl only reads the descriptor; the cast drops const to keep its
	 * verbatim (non-const) winsys signature. */
	rc = ioc_submit_cl((struct drm_v3d_submit_cl *)s);
	/* Bounded positive confirmation on the server UART that a render CL actually ran
	 * through the daemon (ioc_submit_cl is silent on success; it only logs on a wedge).
	 * First few only, so a real per-frame workload doesn't flood the log. */
	if (cl_n < 8u) {
		cl_n++;
		fprintf(stderr, "rpi4-v3d: CL submit #%u done bcl=0x%08x..0x%08x rcl=0x%08x..0x%08x rc=%d\n",
			cl_n, s->bcl_start, s->bcl_end, s->rcl_start, s->rcl_end, rc);
	}
	return rc;
}

int v3d_gpu_submitTfu(const struct drm_v3d_submit_tfu *t)
{
	int rc;
	static unsigned tfu_n = 0;

	if (!W.inited)
		return -EIO;
	rc = ioc_submit_tfu((struct drm_v3d_submit_tfu *)t);
	if (tfu_n < 8u) {
		tfu_n++;
		fprintf(stderr, "rpi4-v3d: TFU submit #%u done iia=0x%08x ioa=0x%08x ios=0x%08x rc=%d\n",
			tfu_n, t->iia, t->ioa, t->ios, rc);
	}
	return rc;
}
