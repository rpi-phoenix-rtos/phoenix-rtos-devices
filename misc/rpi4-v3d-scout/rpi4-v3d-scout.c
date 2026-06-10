/*
 * Phoenix-RTOS — Raspberry Pi 4 (BCM2711) V3D scout (GPU Tier 3)
 *
 * One-shot diagnostic that confirms the V3D 4.2 GPU core is alive and reachable
 * from Phoenix userspace, before any V3D driver work (GLQuake roadmap Tier 3).
 * Mirrors the SDHCI/GENET scouts used for WiFi/Ethernet bring-up and the
 * userspace-mmio-driver pattern proven by rpi4-thermal / rpi4-hwrng / rpi4-fb /
 * rpi4-gpio.
 *
 * Sequence:
 *   1. Ask the VideoCore firmware to power on the V3D power domain (the firmware
 *      genpd leaves it gated until a client asks). RPI_FIRMWARE_SET_DOMAIN_STATE
 *      (tag 0x00038030), domain RPI_POWER_DOMAIN_V3D = 10, state = 1 (on).
 *   2. physmmap the V3D MMIO window (ARM-side 0xfe004000, per the Pi4 DTB) and
 *      RAW-DUMP the first words. We dump rather than decode fixed offsets because
 *      the exact V3D_HUB_IDENT* offset is unsettled between sources; the HUB
 *      IDENT block contains ASCII 'V','3','D', so a non-zero dump with "V3D"
 *      bytes visible is the validation gate ("core is alive + accessible").
 *
 * Isolation note: this is a separate process. If V3D is still gated when we read
 * (power-on tag/domain wrong), the MMIO read may external-abort — that kills only
 * this scout (logged via the EL0 exception dump) and the rest of the system
 * keeps booting. So it is safe to run unattended.
 *
 * Copyright 2026 Phoenix Systems
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/threads.h>

/* Mesa-ported V3D 4.2 control-list packers (generated from external/mesa
 * cle/v3d_packet.xml; see v3d_gen.h shim). Lets us emit CLE packets through
 * Mesa's authoritative *_pack() functions instead of hand-encoding bytes. */
#include "v3d_packet_v42_pack.h"

/* cl_emit-style helper: declare a zero/header-initialised packet struct, run the
 * field-setup block, then pack it at byte cursor `p` (uint8_t*) and advance p. */
#define CL_EMIT(p, PKT, name) \
	for (struct V3D42_##PKT name = { V3D42_##PKT##_header }, *_cldone = (void *)0; \
		_cldone == (void *)0; \
		(V3D42_##PKT##_pack(NULL, (p), &name), (p) += V3D42_##PKT##_length, _cldone = (void *)1))


/* VideoCore property mailbox (carried over from rpi4-thermal). */
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

/* Firmware "set QPU/V3D enabled" tag (0x00030012). Per the Pi forum thread
 * t=346267, THIS is what makes the V3D HUB/CORE MMIO answer on BCM2711 — it does
 * the full V3D power-up (domain + clock + AXI bridge + reset). The separate
 * power-domain / clock calls below alone leave the registers reading 0xdeadbeef.
 * On success V3D core IDENT0 reads 0x02443356 ("V3D" + generation). */
#define VC_PROP_SET_QPU_ENABLE   0x00030012u

/* Firmware power-domain control (Linux raspberrypi-power.c / firmware mailbox). */
#define VC_PROP_SET_DOMAIN_STATE 0x00038030u
#define RPI_POWER_DOMAIN_V3D     10u

/* Firmware clock control. The V3D core only answers on the bus once its clock
 * runs (power-domain-on alone leaves MMIO reads returning the BCM2711 0xdeadbeef
 * bus-error sentinel). Clock id 5 = V3D in the firmware clock-id list. */
#define VC_PROP_SET_CLOCK_STATE  0x00038001u
#define VC_PROP_GET_CLOCK_RATE   0x00030002u
#define VC_PROP_GET_MAX_CLK_RATE 0x00030004u
#define VC_PROP_SET_CLOCK_RATE   0x00038002u
#define RPI_CLOCK_V3D            5u

/* V3D MMIO (BCM2711, confirmed from bcm2711-rpi-4-b.dtb `v3d@7ec04000`,
 * reg = <0x7ec00000 0x4000 (hub), 0x7ec04000 0x4000 (core0)>):
 *   HUB   VPU 0x7ec00000 -> ARM 0xfec00000
 *   CORE0 VPU 0x7ec04000 -> ARM 0xfec04000 (= HUB + 0x4000)
 * Map from the HUB base; 0x10000 covers both blocks. */
#define V3D_MMIO_BASE           0xfec00000u
#define V3D_MMIO_LEN            0x10000u
#define V3D_CORE0_OFFS          0x4000u

/* BCM2711 PM (power management) + V3D ASB (async bridge) — the registers the
 * Linux bcm2835-power driver pokes to power/reset V3D (the firmware mailbox above
 * is the WRONG layer for BCM2711; STEP-0 recon proved the overlay leaves V3D
 * asleep). On BCM2711 the V3D async bridge is the SEPARATE rpivid_asb region, NOT
 * the legacy ASB. ARM-side, from the DTB watchdog@7e100000 reg-names pm/asb/
 * rpivid_asb:  pm 0x7e100000 -> 0xfe100000,  rpivid_asb 0x7ec11000 -> 0xfec11000
 * (legacy asb 0x7e00a000 -> 0xfe00a000 drives ISP/H264, not V3D).
 * This block is currently used by the READ-ONLY recon only (no writes yet). */
#define PM_BASE                 0xfe100000u
#define RPIVID_ASB_BASE         0xfec11000u
#define PM_GRAFX                0x10cu       /* GRAFX power island = V3D's parent domain */
#define PM_POWUP                (1u << 0)
#define PM_POWOK                (1u << 1)
#define PM_ISPOW                (1u << 2)
#define PM_MEMREP               (1u << 3)
#define PM_MRDONE               (1u << 4)
#define PM_ISFUNC               (1u << 5)
#define PM_V3DRSTN              (1u << 6)    /* V3D reset-deassert bit in PM_GRAFX */
#define ASB_V3D_S_CTRL          0x08u        /* offsets within rpivid_asb */
#define ASB_V3D_M_CTRL          0x0cu
#define ASB_REQ_STOP            (1u << 0)
#define ASB_ACK                 (1u << 1)
#define PM_PASSWORD             0x5a000000u  /* top-byte key; PM/ASB ignore writes without it */
#define ASB_ACK_SPINS           100000u      /* bounded poll (Linux uses 100us); never infinite */

/* V3D identity registers (external/linux drivers/gpu/drm/v3d/v3d_regs.h). While
 * V3D is gated every read returns the 0xdeadbeef bus-error sentinel; once alive:
 *   CORE0 V3D_CTL_IDENT0 (CORE0 + 0x00): low 24 bits = 'V','3','D' (0x443356),
 *     top byte = tech version (HW reads 0x04 on the Pi4 V3D 4.2 core).
 *   HUB   V3D_HUB_IDENT0 (HUB + 0x08): ASCII "VHUB" (0x42554856).
 *   HUB   V3D_HUB_IDENT1 (HUB + 0x0c): TVER (3:0) . REV (7:4) => 4.2 on Pi4. */
#define V3D_CORE0_IDENT0_OFFS   (V3D_CORE0_OFFS + 0x00u)
#define V3D_HUB_IDENT1_OFFS     0x0cu
#define V3D_CORE_IDENT0_SIG     0x00443356u  /* "V3D" in the low 24 bits */

/* --- Tier-4: V3D MMU (HUB-relative) + CLE control-list submit (CORE0-relative),
 * per external/linux drivers/gpu/drm/v3d/{v3d_regs.h,v3d_mmu.c,v3d_sched.c,v3d_gem.c}.
 * v3d_mmuCleTest() proves the GPU executes a control list fetched from
 * Phoenix-allocated memory through the V3D MMU (BO + MMU + CLE path live). --- */
#define V3D_MMU_PAGE_SHIFT          12u
#define V3D_MMUC_CONTROL            0x1000u
#define V3D_MMUC_CONTROL_ENABLE     (1u << 0)
#define V3D_MMUC_CONTROL_FLUSH      (1u << 1)
#define V3D_MMUC_CONTROL_FLUSHING   (1u << 2)
#define V3D_MMU_CTL                 0x1200u
#define V3D_MMU_CTL_ENABLE          (1u << 0)
#define V3D_MMU_CTL_TLB_STATS_ENABLE (1u << 1)
#define V3D_MMU_CTL_TLB_CLEAR       (1u << 2)
#define V3D_MMU_CTL_TLB_STATS_CLEAR (1u << 3)
#define V3D_MMU_CTL_TLB_CLEARING    (1u << 7)
#define V3D_MMU_CTL_WRITE_VIOLATION_ABORT (1u << 11)
#define V3D_MMU_CTL_PT_INVALID_ABORT      (1u << 19)
#define V3D_MMU_CTL_CAP_EXCEEDED_ABORT    (1u << 26)
#define V3D_MMU_PT_PA_BASE          0x1204u
#define V3D_MMU_HIT                 0x1208u
#define V3D_MMU_MISSES              0x120cu
#define V3D_MMU_ILLEGAL_ADDR        0x1230u
#define V3D_MMU_ILLEGAL_ADDR_ENABLE (1u << 31)
#define V3D_PTE_WRITEABLE           (1u << 29)
#define V3D_PTE_VALID               (1u << 28)
/* CORE0-relative offsets (add V3D_CORE0_OFFS to reach from the HUB mapping base). */
#define V3D_CTL_MISCCFG             0x0018u
#define V3D_MISCCFG_OVRTMUOUT       (1u << 0)
#define V3D_CTL_L2CACTL             0x0020u
#define V3D_L2CACTL_L2CENA          (1u << 0)
#define V3D_L2CACTL_L2CCLR          (1u << 2)
#define V3D_CLE_CT0CA               0x0110u
#define V3D_CLE_CT0QBA              0x0160u
#define V3D_CLE_CT0QEA              0x0168u
#define V3D_PTB_BPOS                0x030cu
/* Control list: map at GPU VA page 1 (page 0 left unmapped as a null guard). */
#define V3D_CL_GPUVA                0x1000u
#define V3D_CL_NOPS                 64u
#define CLE_NOP                     0x01u
#define CLE_HALT                    0x00u
#define V3D_SPIN_BOUND              200000u

/* --- Tier-4 4b-1: minimal BIN pass (binner runs over an empty 64x64 frame and
 * initialises tile state in our memory). Packet/sizing values verified against
 * external/mesa (cle/v3d_packet.xml gen_pack_header -> v42; common/v3d_util.c
 * v3d_tile_alloc_sizes; common/v3d_limits.h) and external/linux v3d_sched.c
 * (v3d_bin_job_run) / v3d_gem.c (v3d_invalidate_caches). --- */
#define V3D_CLE_CT0QTS              0x015cu
#define V3D_CLE_CT0QTS_ENABLE       (1u << 1)
#define V3D_CLE_CT0QMA              0x0170u
#define V3D_CLE_CT0QMS              0x0174u
#define V3D_CTL_SLCACTL             0x0024u
#define V3D_CTL_L2TCACTL            0x0030u
#define V3D_L2TCACTL_L2TFLS         (1u << 0)   /* FLM_FLUSH=0 in bits 2:1 */
#define CLE_TILE_BINNING_MODE_CFG   120u
#define CLE_START_TILE_BINNING      6u
#define CLE_FLUSH                   4u
/* V3D_INTERNAL_BPP_32 / _TYPE_8 / _MEMORY_FORMAT_RASTER / _OUTPUT_IMAGE_FORMAT_RGBA8
 * come from the Mesa pack header's enums (v3d_packet_v42_pack.h). */
/* GPU VA layout (distinct windows; page 0 left unmapped as a null guard). */
#define V3D_BIN_CL_VA               0x1000u     /* 1 page */
#define V3D_TILEALLOC_VA            0x10000u    /* 4 pages (16 KiB) */
#define V3D_TILESTATE_VA            0x20000u    /* 1 page */
#define V3D_TILEALLOC_PAGES         4u          /* v3d_tile_alloc_sizes(1,1,1,1)=16384 */

/* --- Tier-4 4b-2: render clear-to-color on CT1 (a 64x64 RGBA8 RT). Opcodes,
 * byte layouts, enums and sub_ids verified via the v42 gen_pack_header output and
 * external/mesa v3dx_rcl.c emit order. --- */
#define V3D_CLE_CT1CA               0x0114u
#define V3D_CLE_CT1QBA              0x0164u
#define V3D_CLE_CT1QEA              0x016cu
#define V3D_CTL_INT_STS             0x0050u   /* CORE-relative */
#define V3D_CTL_INT_CLR             0x0058u
#define V3D_INT_FRDONE              (1u << 0) /* render/frame done */
#define V3D_INT_FLDONE              (1u << 1) /* binner/flush done */
#define V3D_INT_OUTOMEM             (1u << 2) /* binner out of tile-alloc memory */
#define V3D_CLE_CT1CS               0x0104u   /* render control-thread status */
#define V3D_GMP_STATUS              0x0800u   /* global memory protection status (VIO=BIT0) */
#define V3D_ERR_FDBGS               0x0f10u   /* render pipeline stall-stage status */
#define V3D_ERR_FDBGO               0x0f04u
#define V3D_ERR_STAT                0x0f20u
#define CLE_TRM_CFG                 121u  /* Tile Rendering Mode Cfg; sub_id discriminates */
#define CLE_MULTICORE_SUPERTILE_CFG 122u
#define CLE_MULTICORE_TILE_LIST_BASE 123u
#define CLE_TILE_COORDINATES        124u
#define CLE_TILE_COORDINATES_IMPL   125u
#define CLE_CLEAR_TILE_BUFFERS      25u
#define CLE_END_OF_LOADS            26u
#define CLE_END_OF_TILE_MARKER      27u
#define CLE_STORE_GENERAL           29u
#define CLE_FLUSH_VCD_CACHE         19u
#define CLE_BRANCH_TO_IMPLICIT      21u
#define CLE_SUPERTILE_COORDINATES   23u
#define CLE_END_OF_RENDERING        13u
#define CLE_START_ADDR_GENERIC      20u
#define CLE_RETURN_FROM_SUB_LIST    18u
#define TRM_SUBID_COMMON            0u
#define TRM_SUBID_COLOR             1u
#define TRM_SUBID_CLEARCOL1         3u
#define V3D_BUFFER_RT0              0u
#define V3D_BUFFER_NONE             8u
#define V3D_RCL_VA                  0x30000u  /* main RCL (offset 0) + sub-list (offset 0x800) */
#define V3D_SUBLIST_OFFS            0x800u
#define V3D_RT_VA                   0x40000u  /* render-target BO */
#define V3D_RT_W                    64u
#define V3D_RT_H                    64u
#define V3D_RT_PAGES                4u   /* 64*64*4 = 16384 */
#define V3D_CLEAR_COLOR             0x11223344u


/*
 * Two-u32-in property call (domain, state). Returns the firmware's resulting
 * state word, or MBOX_FAIL. Same cache/transport discipline as rpi4-thermal's
 * 1in1out: device-mapped uncached mailbox window + uncached, contiguous,
 * physically-pinned property buffer (no explicit clean/invalidate needed).
 */
/* Generic property call with `nw` value words (1 or 2). Returns the LAST value
 * word of the response (msg[5] for nw=1, msg[6] for nw=2) — i.e. the result for
 * 1-word tags like set-qpu-enable, and the second word (state/rate) for 2-word
 * tags like set-domain/clock. Critically, valbuf size = nw*4 must match the
 * tag's expectation or the firmware ignores it. */
static uint32_t v3d_mboxProp(uint32_t tag, int nw, uint32_t w0, uint32_t w1)
{
	addr_t pa_base = (addr_t)RPI_MAILBOX_BASE & ~(addr_t)(_PAGE_SIZE - 1);
	addr_t pa_offs = (addr_t)RPI_MAILBOX_BASE & (addr_t)(_PAGE_SIZE - 1);
	volatile uint32_t *mbox;
	uint32_t *msg;
	uintptr_t msg_pa;
	uint32_t request;
	uint32_t result = MBOX_FAIL;
	uint32_t spins;
	void *mbox_page;
	void *msg_page;

	mbox_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, pa_base);
	if (mbox_page == MAP_FAILED) {
		return MBOX_FAIL;
	}
	mbox = (volatile uint32_t *)((volatile uint8_t *)mbox_page + pa_offs);

	msg_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (msg_page == MAP_FAILED) {
		munmap(mbox_page, _PAGE_SIZE);
		return MBOX_FAIL;
	}
	msg = msg_page;

	/* [size, REQUEST, tag, valbuf=nw*4, req=0, w0[, w1], END]. */
	msg[0] = (uint32_t)(6 + nw) * 4u;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = (uint32_t)nw * 4u;
	msg[4] = 0;
	msg[5] = w0;
	if (nw > 1) {
		msg[6] = w1;
	}
	msg[5 + nw] = 0; /* END tag */

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
		if ((mbox[VC_MBOX_STATUS / 4] & VC_MBOX_STATUS_EMPTY) == 0u) {
			if (mbox[VC_MBOX_READ / 4] == request) {
				break;
			}
		}
	}
	if (spins != 0u && msg[1] == VC_MBOX_RESP_OK) {
		result = msg[5 + nw - 1];
	}

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}


/*
 * READ-ONLY recon of the PM + rpivid_asb registers a direct V3D power-on would
 * poke. Advisor-gated zero-risk step before any write build: confirms the regions
 * are real (not the 0xdeadbeef bus-error sentinel that the V3D MMIO returns),
 * reveals whether the GRAFX power island is already up (PM_POWUP/POWOK/ISFUNC) and
 * V3D already out of reset (PM_V3DRSTN) — i.e. which half of the bring-up sequence
 * is load-bearing — and validates that the V3D ASB ctrl regs read ASB-shaped
 * values (REQ_STOP/ACK) at the rpivid_asb 0x08/0x0c offsets. No writes: a faulting
 * read just kills this isolated scout and the rest of the system keeps booting.
 */
static void v3d_reconPmAsb(void)
{
	volatile uint32_t *pm, *asb;
	void *pm_page, *asb_page;
	uint32_t grafx, sctrl, mctrl;

	pm_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)PM_BASE);
	if (pm_page == MAP_FAILED) {
		printf("rpi4-v3d-scout: recon mmap(PM 0x%08x) FAILED\n", PM_BASE);
		return;
	}
	pm = (volatile uint32_t *)pm_page;

	asb_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)RPIVID_ASB_BASE);
	if (asb_page == MAP_FAILED) {
		printf("rpi4-v3d-scout: recon mmap(rpivid_asb 0x%08x) FAILED\n", RPIVID_ASB_BASE);
		munmap(pm_page, _PAGE_SIZE);
		return;
	}
	asb = (volatile uint32_t *)asb_page;

	grafx = pm[PM_GRAFX / 4];
	printf("rpi4-v3d-scout: PM_GRAFX (0x%08x+0x%03x) = 0x%08x\n", PM_BASE, PM_GRAFX, grafx);
	printf("rpi4-v3d-scout:   POWUP=%u POWOK=%u ISPOW=%u MEMREP=%u MRDONE=%u ISFUNC=%u V3DRSTN=%u\n",
		!!(grafx & PM_POWUP), !!(grafx & PM_POWOK), !!(grafx & PM_ISPOW),
		!!(grafx & PM_MEMREP), !!(grafx & PM_MRDONE), !!(grafx & PM_ISFUNC),
		!!(grafx & PM_V3DRSTN));

	sctrl = asb[ASB_V3D_S_CTRL / 4];
	mctrl = asb[ASB_V3D_M_CTRL / 4];
	printf("rpi4-v3d-scout: ASB_V3D_S_CTRL (rpivid+0x%02x) = 0x%08x (REQ_STOP=%u ACK=%u)\n",
		ASB_V3D_S_CTRL, sctrl, !!(sctrl & ASB_REQ_STOP), !!(sctrl & ASB_ACK));
	printf("rpi4-v3d-scout: ASB_V3D_M_CTRL (rpivid+0x%02x) = 0x%08x (REQ_STOP=%u ACK=%u)\n",
		ASB_V3D_M_CTRL, mctrl, !!(mctrl & ASB_REQ_STOP), !!(mctrl & ASB_ACK));

	munmap(asb_page, _PAGE_SIZE);
	munmap(pm_page, _PAGE_SIZE);
}


/*
 * Enable one V3D ASB async-AXI bridge (master or slave): clear ASB_REQ_STOP and
 * poll until the controller drops ASB_ACK (bridge running). Mirrors the canonical
 * bcm2835_asb_control(enable=true). Bounded spin — never blocks forever.
 */
static int v3d_asbEnable(volatile uint32_t *asb, uint32_t reg)
{
	uint32_t val = asb[reg / 4] & ~ASB_REQ_STOP;
	uint32_t spins;

	asb[reg / 4] = PM_PASSWORD | val;
	for (spins = ASB_ACK_SPINS; spins != 0u; spins--) {
		if ((asb[reg / 4] & ASB_ACK) == 0u) {
			return 0;
		}
	}
	return -1;
}


/*
 * Direct V3D power-on, the canonical BCM2711 path (external/linux
 * drivers/pmdomain/bcm/bcm2835-power.c bcm2835_asb_power_on for GRAFX_V3D). On
 * BCM2711 the PM power-island POWUP sequence is a NO-OP ("We don't run this on
 * BCM2711"); the real bring-up is: clock-toggle around a reset-deassert, then
 * enable the V3D master + slave async-AXI bridges. The Linux clk_{en,dis}able
 * maps to the firmware mailbox SET_CLOCK_STATE the scout already drives.
 *
 * Only ONE PM write (set PM_V3DRSTN in PM_GRAFX) + two rpivid_asb writes, each
 * keyed with PM_PASSWORD and bounded; recon already proved both regions are real.
 */
static void v3d_powerOn(void)
{
	volatile uint32_t *pm, *asb;
	void *pm_page, *asb_page;
	uint32_t grafx;
	int rcM, rcS;

	pm_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)PM_BASE);
	if (pm_page == MAP_FAILED) {
		printf("rpi4-v3d-scout: powerOn mmap(PM) FAILED\n");
		return;
	}
	pm = (volatile uint32_t *)pm_page;

	asb_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)RPIVID_ASB_BASE);
	if (asb_page == MAP_FAILED) {
		printf("rpi4-v3d-scout: powerOn mmap(rpivid_asb) FAILED\n");
		munmap(pm_page, _PAGE_SIZE);
		return;
	}
	asb = (volatile uint32_t *)asb_page;

	/* clk on -> wait 32 clocks for reset to propagate -> clk off (canonical). */
	(void)v3d_mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);
	usleep(50);
	(void)v3d_mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 0u);

	/* Deassert the V3D reset (the only PM write), with the clock off. */
	grafx = pm[PM_GRAFX / 4];
	pm[PM_GRAFX / 4] = PM_PASSWORD | (grafx | PM_V3DRSTN);
	printf("rpi4-v3d-scout: powerOn PM_GRAFX 0x%08x -> deassert V3DRSTN -> 0x%08x\n",
		grafx, pm[PM_GRAFX / 4]);

	/* clk back on, then re-assert the rate (gate-off may have dropped it). */
	(void)v3d_mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);
	usleep(50);

	/* Enable the V3D master then slave async-AXI bridges. */
	rcM = v3d_asbEnable(asb, ASB_V3D_M_CTRL);
	rcS = v3d_asbEnable(asb, ASB_V3D_S_CTRL);
	printf("rpi4-v3d-scout: powerOn asb_enable M=%s S=%s; M_CTRL=0x%08x S_CTRL=0x%08x\n",
		(rcM == 0) ? "ok" : "TIMEOUT", (rcS == 0) ? "ok" : "TIMEOUT",
		asb[ASB_V3D_M_CTRL / 4], asb[ASB_V3D_S_CTRL / 4]);

	usleep(2000);
	munmap(asb_page, _PAGE_SIZE);
	munmap(pm_page, _PAGE_SIZE);
}


/* Allocate one uncached, physically-contiguous page (a minimal GPU "BO") and
 * return its VA; *pa_out gets the physical address. Uncached => writes are
 * coherent for the GPU with no explicit dcache clean (the scout's mailbox pattern). */
static void *v3d_boAlloc(uintptr_t *pa_out)
{
	void *p = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (p == MAP_FAILED) {
		return NULL;
	}
	*pa_out = (uintptr_t)va2pa(p);
	if (*pa_out == (uintptr_t)-1) {
		munmap(p, _PAGE_SIZE);
		return NULL;
	}
	return p;
}


/*
 * Tier-4 minimal CLE probe (RETAINED, not called by default): proves the V3D
 * executes a control list fetched from Phoenix-allocated memory THROUGH the V3D
 * MMU, using a bare NOP/HALT list (no binner tile-state). It is the simplest
 * standalone proof of the BO+MMU+CLE path (validated + manifested as
 * 2026-06-10-v3d-mmu-cle-foundation). v3d_binTest() below subsumes it (a real bin
 * job also exercises CT0 + the MMU), and only one CT0 job can run per boot without
 * a control-thread reset, so main() runs the bin test instead. Kept (unused) as a
 * documented fallback probe. All BOs uncached; only V3D regs touched.
 */
static __attribute__((unused)) void v3d_mmuCleTest(volatile uint32_t *v3d)
{
	volatile uint32_t *core0 = v3d + (V3D_CORE0_OFFS / 4);
	void *ptPage, *clPage, *scratchPage;
	uintptr_t ptPa, clPa, scratchPa;
	volatile uint32_t *pt;
	volatile uint8_t *cl;
	uint32_t hitsBefore, hitsAfter, misses, ca, endVa, spins;
	uint32_t i;

	ptPage = v3d_boAlloc(&ptPa);
	clPage = v3d_boAlloc(&clPa);
	scratchPage = v3d_boAlloc(&scratchPa);
	if (ptPage == NULL || clPage == NULL || scratchPage == NULL) {
		printf("rpi4-v3d-scout: cle test BO alloc FAILED\n");
		return;
	}
	pt = (volatile uint32_t *)ptPage;
	cl = (volatile uint8_t *)clPage;

	/* Page table: everything invalid, then map V3D_CL_GPUVA -> CL BO page. */
	for (i = 0; i < (_PAGE_SIZE / 4u); i++) {
		pt[i] = 0u;
	}
	pt[V3D_CL_GPUVA >> V3D_MMU_PAGE_SHIFT] =
		(uint32_t)(clPa >> V3D_MMU_PAGE_SHIFT) | V3D_PTE_WRITEABLE | V3D_PTE_VALID;

	/* Control list: V3D_CL_NOPS NOPs then a HALT. */
	for (i = 0; i < V3D_CL_NOPS; i++) {
		cl[i] = CLE_NOP;
	}
	cl[V3D_CL_NOPS] = CLE_HALT;
	endVa = V3D_CL_GPUVA + V3D_CL_NOPS + 1u;

	/* Program + enable the MMU (PT base & illegal-addr scratch are physical). */
	v3d[V3D_MMU_PT_PA_BASE / 4] = (uint32_t)(ptPa >> V3D_MMU_PAGE_SHIFT);
	v3d[V3D_MMU_ILLEGAL_ADDR / 4] =
		(uint32_t)(scratchPa >> V3D_MMU_PAGE_SHIFT) | V3D_MMU_ILLEGAL_ADDR_ENABLE;
	v3d[V3D_MMU_CTL / 4] = V3D_MMU_CTL_ENABLE | V3D_MMU_CTL_TLB_STATS_ENABLE |
		V3D_MMU_CTL_TLB_STATS_CLEAR | V3D_MMU_CTL_PT_INVALID_ABORT |
		V3D_MMU_CTL_WRITE_VIOLATION_ABORT | V3D_MMU_CTL_CAP_EXCEEDED_ABORT;
	v3d[V3D_MMUC_CONTROL / 4] = V3D_MMUC_CONTROL_ENABLE;
	/* Flush MMU cache + TLB so the new PT is seen. */
	v3d[V3D_MMUC_CONTROL / 4] = V3D_MMUC_CONTROL_FLUSH | V3D_MMUC_CONTROL_ENABLE;
	for (spins = V3D_SPIN_BOUND; spins != 0u &&
		(v3d[V3D_MMUC_CONTROL / 4] & V3D_MMUC_CONTROL_FLUSHING) != 0u; spins--) {
	}
	v3d[V3D_MMU_CTL / 4] |= V3D_MMU_CTL_TLB_CLEAR;
	for (spins = V3D_SPIN_BOUND; spins != 0u &&
		(v3d[V3D_MMU_CTL / 4] & V3D_MMU_CTL_TLB_CLEARING) != 0u; spins--) {
	}

	/* Core init (matches v3d_init_core) + clear/enable the L2 cache so the CLE
	 * fetch reads fresh memory. */
	core0[V3D_CTL_MISCCFG / 4] = V3D_MISCCFG_OVRTMUOUT;
	core0[V3D_CTL_L2CACTL / 4] = V3D_L2CACTL_L2CCLR | V3D_L2CACTL_L2CENA;

	hitsBefore = v3d[V3D_MMU_HIT / 4];

	/* Kick the bin control thread over the NOP/HALT list at the GPU VA. */
	core0[V3D_PTB_BPOS / 4] = 0u;
	core0[V3D_CLE_CT0QBA / 4] = V3D_CL_GPUVA;
	core0[V3D_CLE_CT0QEA / 4] = endVa;

	/* Poll until the control pointer reaches the end of the list (or bound out). */
	for (spins = 1000000u; spins != 0u; spins--) {
		ca = core0[V3D_CLE_CT0CA / 4];
		if (ca >= endVa) {
			break;
		}
	}
	ca = core0[V3D_CLE_CT0CA / 4];
	hitsAfter = v3d[V3D_MMU_HIT / 4];
	misses = v3d[V3D_MMU_MISSES / 4];

	printf("rpi4-v3d-scout: CLE test: CT0CA=0x%08x (start 0x%08x end 0x%08x) "
		"MMU_HIT %u->%u MISSES=%u\n",
		ca, V3D_CL_GPUVA, endVa, hitsBefore, hitsAfter, misses);
	/* Proof: the control pointer walked from the GPU VA start cleanly to the exact
	 * end of the list (CA==endVa), the MMU recorded a translation (HIT incremented),
	 * and PT_INVALID_ABORT was armed so a bad translation would have aborted instead
	 * of completing. Only possible if the MMU translated GPU VA 0x1000 to our CL BO
	 * and the CLE fetched+executed every byte. MISSES are the expected cold TLB fills
	 * for the one mapped page (first access walks the PT, then the MMUC line-caches
	 * the rest), not a failure. */
	if (ca == endVa && hitsAfter > hitsBefore) {
		printf("rpi4-v3d-scout: *** GPU EXECUTED CONTROL LIST from MMU-mapped memory *** "
			"(Tier-4 BO+MMU+CLE path live; MMU_HIT=%u, cold-fill MISSES=%u)\n",
			hitsAfter, misses);
	}
	else {
		printf("rpi4-v3d-scout: CLE test inconclusive "
			"(CA=0x%08x want 0x%08x, MMU_HIT %u->%u)\n", ca, endVa, hitsBefore, hitsAfter);
	}

	munmap(scratchPage, _PAGE_SIZE);
	munmap(clPage, _PAGE_SIZE);
	munmap(ptPage, _PAGE_SIZE);
}


/* Allocate N uncached, physically-contiguous pages (a multi-page GPU BO). */
static void *v3d_boAllocN(uint32_t npages, uintptr_t *pa_out)
{
	void *p = mmap(NULL, (size_t)npages * _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (p == MAP_FAILED) {
		return NULL;
	}
	*pa_out = (uintptr_t)va2pa(p);
	if (*pa_out == (uintptr_t)-1) {
		munmap(p, (size_t)npages * _PAGE_SIZE);
		return NULL;
	}
	return p;
}


/* Map npages of a BO at GPU VA `va` into the flat page table `pt`. */
static void v3d_mapBo(volatile uint32_t *pt, uint32_t va, uintptr_t pa, uint32_t npages)
{
	uint32_t i;
	for (i = 0; i < npages; i++) {
		pt[(va >> V3D_MMU_PAGE_SHIFT) + i] =
			(uint32_t)((pa >> V3D_MMU_PAGE_SHIFT) + i) | V3D_PTE_WRITEABLE | V3D_PTE_VALID;
	}
}


/* Program + enable the V3D MMU over page table at ptPa, with scratchPa as the
 * illegal-access landing page, then flush the MMU cache + TLB. */
static void v3d_mmuEnable(volatile uint32_t *v3d, uintptr_t ptPa, uintptr_t scratchPa)
{
	uint32_t spins;

	v3d[V3D_MMU_PT_PA_BASE / 4] = (uint32_t)(ptPa >> V3D_MMU_PAGE_SHIFT);
	v3d[V3D_MMU_ILLEGAL_ADDR / 4] =
		(uint32_t)(scratchPa >> V3D_MMU_PAGE_SHIFT) | V3D_MMU_ILLEGAL_ADDR_ENABLE;
	v3d[V3D_MMU_CTL / 4] = V3D_MMU_CTL_ENABLE | V3D_MMU_CTL_TLB_STATS_ENABLE |
		V3D_MMU_CTL_TLB_STATS_CLEAR | V3D_MMU_CTL_PT_INVALID_ABORT |
		V3D_MMU_CTL_WRITE_VIOLATION_ABORT | V3D_MMU_CTL_CAP_EXCEEDED_ABORT;
	v3d[V3D_MMUC_CONTROL / 4] = V3D_MMUC_CONTROL_ENABLE;
	v3d[V3D_MMUC_CONTROL / 4] = V3D_MMUC_CONTROL_FLUSH | V3D_MMUC_CONTROL_ENABLE;
	for (spins = V3D_SPIN_BOUND; spins != 0u &&
		(v3d[V3D_MMUC_CONTROL / 4] & V3D_MMUC_CONTROL_FLUSHING) != 0u; spins--) {
	}
	v3d[V3D_MMU_CTL / 4] |= V3D_MMU_CTL_TLB_CLEAR;
	for (spins = V3D_SPIN_BOUND; spins != 0u &&
		(v3d[V3D_MMU_CTL / 4] & V3D_MMU_CTL_TLB_CLEARING) != 0u; spins--) {
	}
}


/* Invalidate the V3D caches before a job (V3D 4.2: L2T flush + slices; the L2C
 * invalidate is a no-op on >=v3.3 per v3d_invalidate_l2c). */
static void v3d_invalidateCaches(volatile uint32_t *core0)
{
	core0[V3D_CTL_L2TCACTL / 4] = V3D_L2TCACTL_L2TFLS; /* FLM_FLUSH (0) in bits 2:1 */
	core0[V3D_CTL_SLCACTL / 4] = 0x0f0f0f0fu;          /* invalidate TVCCS/TDCCS/UCC/ICC */
}


/*
 * Tier-4 4b-1: minimal BIN pass. Build a bin control list (Tile Binning Mode Cfg
 * for a 64x64 1-RT frame + Start Tile Binning + Flush), hand the binner its
 * tile-allocation + tile-state memory (CT0QMA/QMS/QTS), kick the bin control
 * thread, and check it ran to completion. Also reports whether the binner wrote
 * the (pre-zeroed) tile_state BO — answering the open question of whether an
 * empty frame initialises tile state. All BOs uncached; only V3D regs touched.
 */
static __attribute__((unused)) void v3d_binTest(volatile uint32_t *v3d)
{
	volatile uint32_t *core0 = v3d + (V3D_CORE0_OFFS / 4);
	void *ptPage, *clPage, *allocPage, *statePage, *scratchPage;
	uintptr_t ptPa, clPa, allocPa, statePa, scratchPa;
	volatile uint32_t *pt;
	volatile uint8_t *cl;
	volatile uint8_t *state;
	uint32_t ca, spins, endVa, nonzero, hitsBefore, hitsAfter, misses, i, n;

	ptPage = v3d_boAllocN(1, &ptPa);
	clPage = v3d_boAllocN(1, &clPa);
	allocPage = v3d_boAllocN(V3D_TILEALLOC_PAGES, &allocPa);
	statePage = v3d_boAllocN(1, &statePa);
	scratchPage = v3d_boAllocN(1, &scratchPa);
	if (ptPage == NULL || clPage == NULL || allocPage == NULL ||
		statePage == NULL || scratchPage == NULL) {
		printf("rpi4-v3d-scout: bin test BO alloc FAILED\n");
		return;
	}
	pt = (volatile uint32_t *)ptPage;
	cl = (volatile uint8_t *)clPage;
	state = (volatile uint8_t *)statePage;

	/* Zero the PT and the tile_state BO (so binner writes are detectable). */
	for (i = 0; i < (_PAGE_SIZE / 4u); i++) {
		pt[i] = 0u;
	}
	for (i = 0; i < _PAGE_SIZE; i++) {
		state[i] = 0u;
	}

	/* Map the three BOs at their GPU VAs. */
	v3d_mapBo(pt, V3D_BIN_CL_VA, clPa, 1u);
	v3d_mapBo(pt, V3D_TILEALLOC_VA, allocPa, V3D_TILEALLOC_PAGES);
	v3d_mapBo(pt, V3D_TILESTATE_VA, statePa, 1u);

	/* Build the bin control list. */
	n = 0u;
	cl[n++] = CLE_TILE_BINNING_MODE_CFG;
	cl[n++] = (uint8_t)((0u << 4) | (1u << 2));      /* overflow block 64b(0), initial 128b(1) */
	cl[n++] = (uint8_t)((V3D_INTERNAL_BPP_32 << 4) | ((1u - 1u) & 0xfu)); /* bpp32, 1 RT */
	cl[n++] = 0u;
	cl[n++] = 0u;
	cl[n++] = (uint8_t)((64u - 1u) & 0xffu);         /* width-1 lo */
	cl[n++] = (uint8_t)(((64u - 1u) >> 8) & 0xffu);  /* width-1 hi */
	cl[n++] = (uint8_t)((64u - 1u) & 0xffu);         /* height-1 lo */
	cl[n++] = (uint8_t)(((64u - 1u) >> 8) & 0xffu);  /* height-1 hi */
	cl[n++] = CLE_START_TILE_BINNING;
	cl[n++] = CLE_FLUSH;
	endVa = V3D_BIN_CL_VA + n;

	v3d_mmuEnable(v3d, ptPa, scratchPa);

	core0[V3D_CTL_MISCCFG / 4] = V3D_MISCCFG_OVRTMUOUT;
	v3d_invalidateCaches(core0);

	hitsBefore = v3d[V3D_MMU_HIT / 4];

	/* Kick the bin control thread (writing CT0QEA starts it). */
	core0[V3D_PTB_BPOS / 4] = 0u;
	core0[V3D_CLE_CT0QMA / 4] = V3D_TILEALLOC_VA;
	core0[V3D_CLE_CT0QMS / 4] = V3D_TILEALLOC_PAGES * _PAGE_SIZE;
	core0[V3D_CLE_CT0QTS / 4] = V3D_CLE_CT0QTS_ENABLE | V3D_TILESTATE_VA;
	core0[V3D_CLE_CT0QBA / 4] = V3D_BIN_CL_VA;
	core0[V3D_CLE_CT0QEA / 4] = endVa;

	for (spins = 2000000u; spins != 0u; spins--) {
		ca = core0[V3D_CLE_CT0CA / 4];
		if (ca >= endVa) {
			break;
		}
	}
	ca = core0[V3D_CLE_CT0CA / 4];
	hitsAfter = v3d[V3D_MMU_HIT / 4];
	misses = v3d[V3D_MMU_MISSES / 4];

	nonzero = 0u;
	for (i = 0; i < _PAGE_SIZE; i++) {
		if (state[i] != 0u) {
			nonzero++;
		}
	}

	printf("rpi4-v3d-scout: BIN test: CT0CA=0x%08x (end 0x%08x) tile_state nonzero=%u/%u "
		"MMU_HIT %u->%u MISSES=%u\n",
		ca, endVa, nonzero, (uint32_t)_PAGE_SIZE, hitsBefore, hitsAfter, misses);
	if (ca == endVa && hitsAfter > hitsBefore) {
		printf("rpi4-v3d-scout: *** BIN PASS EXECUTED *** (binner ran the CL; tile_state %s)\n",
			(nonzero > 0u) ? "written -> empty-frame DOES init tile state" :
			"unchanged -> empty frame does not write tile state (needs a primitive)");
	}
	else {
		printf("rpi4-v3d-scout: BIN test inconclusive (CA=0x%08x want 0x%08x, MMU_HIT %u->%u)\n",
			ca, endVa, hitsBefore, hitsAfter);
	}

	munmap(scratchPage, _PAGE_SIZE);
	munmap(statePage, _PAGE_SIZE);
	munmap(allocPage, (size_t)V3D_TILEALLOC_PAGES * _PAGE_SIZE);
	munmap(clPage, _PAGE_SIZE);
	munmap(ptPage, _PAGE_SIZE);
}


/* Little-endian u32 store into a CL byte buffer (retained helper; the render CL
 * now emits via Mesa packers, the bin CL is still hand-built). */
static __attribute__((unused)) void v3d_put32(volatile uint8_t *b, uint32_t off, uint32_t v)
{
	b[off + 0] = (uint8_t)v;
	b[off + 1] = (uint8_t)(v >> 8);
	b[off + 2] = (uint8_t)(v >> 16);
	b[off + 3] = (uint8_t)(v >> 24);
}


/*
 * Tier-4 4b-2: clear a 64x64 RGBA8 render-target BO to a known color via the V3D
 * render pipeline, then read it back. Runs a minimal bin pass (CT0) to init tile
 * state, then a render control list (CT1): Tile Rendering Mode Cfg (common/color/
 * clear-colors) -> tile-list base -> supertile cfg -> GFXH-1742 initial-clear dance
 * -> generic per-tile sub-list (implicit coords + store RT0) -> supertile coords ->
 * end of rendering. Verified by reading the RT BO back on the ARM side. All BOs
 * uncached; only V3D regs touched (isolated scout).
 */
static void v3d_renderClearTest(volatile uint32_t *v3d)
{
	volatile uint32_t *core0 = v3d + (V3D_CORE0_OFFS / 4);
	void *ptPage, *binPage, *allocPage, *statePage, *rclPage, *rtPage, *scratchPage;
	uintptr_t ptPa, binPa, allocPa, statePa, rclPa, rtPa, scratchPa;
	volatile uint32_t *pt;
	uint8_t *bin, *bp, *m, *mp, *s, *sp;
	volatile uint32_t *rt;
	uint32_t i, binEnd, rclEnd, sublistVa, sublen, ca, spins;
	uint32_t px0, pxN, okpix;

	ptPage = v3d_boAllocN(1, &ptPa);
	binPage = v3d_boAllocN(1, &binPa);
	allocPage = v3d_boAllocN(V3D_TILEALLOC_PAGES, &allocPa);
	statePage = v3d_boAllocN(1, &statePa);
	rclPage = v3d_boAllocN(1, &rclPa);
	rtPage = v3d_boAllocN(V3D_RT_PAGES, &rtPa);
	scratchPage = v3d_boAllocN(1, &scratchPa);
	if (ptPage == NULL || binPage == NULL || allocPage == NULL || statePage == NULL ||
		rclPage == NULL || rtPage == NULL || scratchPage == NULL) {
		printf("rpi4-v3d-scout: render test BO alloc FAILED\n");
		return;
	}
	pt = (volatile uint32_t *)ptPage;
	bin = (uint8_t *)binPage;
	m = (uint8_t *)rclPage;                        /* main RCL at offset 0 */
	s = (uint8_t *)rclPage + V3D_SUBLIST_OFFS;     /* per-tile sub-list */
	rt = (volatile uint32_t *)rtPage;
	sublistVa = V3D_RCL_VA + V3D_SUBLIST_OFFS;

	for (i = 0; i < (_PAGE_SIZE / 4u); i++) {
		pt[i] = 0u;
	}
	v3d_mapBo(pt, V3D_BIN_CL_VA, binPa, 1u);
	v3d_mapBo(pt, V3D_TILEALLOC_VA, allocPa, V3D_TILEALLOC_PAGES);
	v3d_mapBo(pt, V3D_TILESTATE_VA, statePa, 1u);
	v3d_mapBo(pt, V3D_RCL_VA, rclPa, 1u);
	v3d_mapBo(pt, V3D_RT_VA, rtPa, V3D_RT_PAGES);

	for (i = 0; i < (V3D_RT_PAGES * _PAGE_SIZE) / 4u; i++) {
		rt[i] = 0xa5a5a5a5u;
	}

	/* --- Bin CL via Mesa packers (mirrors v3dX(start_binning)). --- */
	bp = bin;
	CL_EMIT(bp, TILE_BINNING_MODE_CFG, c) {
		c.width_in_pixels = V3D_RT_W;
		c.height_in_pixels = V3D_RT_H;
		c.number_of_render_targets = 1;
		c.maximum_bpp_of_all_render_targets = V3D_INTERNAL_BPP_32;
		c.tile_allocation_initial_block_size = 1; /* 128 B */
		c.tile_allocation_block_size = 0;         /* 64 B */
	}
	CL_EMIT(bp, FLUSH_VCD_CACHE, c) { (void)c; }
	CL_EMIT(bp, OCCLUSION_QUERY_COUNTER, c) { (void)c; } /* address 0 => disabled */
	CL_EMIT(bp, START_TILE_BINNING, c) { (void)c; }
	CL_EMIT(bp, FLUSH, c) { (void)c; }
	binEnd = V3D_BIN_CL_VA + (uint32_t)(bp - bin);

	/* --- Per-tile generic sub-list (mesa v3d_rcl_emit_generic_per_tile_list). --- */
	sp = s;
	CL_EMIT(sp, TILE_COORDINATES_IMPLICIT, c) { (void)c; }
	CL_EMIT(sp, END_OF_LOADS, c) { (void)c; }
	CL_EMIT(sp, PRIM_LIST_FORMAT, c) { c.primitive_type = LIST_TRIANGLES; }
	CL_EMIT(sp, SET_INSTANCEID, c) { c.instance_id = 0; }
	CL_EMIT(sp, BRANCH_TO_IMPLICIT_TILE_LIST, c) { c.tile_list_set_number = 0; }
	CL_EMIT(sp, STORE_TILE_BUFFER_GENERAL, c) {
		c.buffer_to_store = V3D_BUFFER_RT0;
		c.memory_format = V3D_MEMORY_FORMAT_RASTER;
		c.output_image_format = V3D_OUTPUT_IMAGE_FORMAT_RGBA8;
		c.height_in_ub_or_stride = V3D_RT_W * 4u;
		c.height = V3D_RT_H;
		c.address.offset = V3D_RT_VA;
	}
	CL_EMIT(sp, END_OF_TILE_MARKER, c) { (void)c; }
	CL_EMIT(sp, RETURN_FROM_SUB_LIST, c) { (void)c; }
	sublen = (uint32_t)(sp - s);

	/* --- Main RCL (mesa emit_rcl + emit_render_layer): COMMON first, ZS_CLEAR last. --- */
	mp = m;
	CL_EMIT(mp, TILE_RENDERING_MODE_CFG_COMMON, c) {
		c.number_of_render_targets = 1;
		c.image_width_pixels = V3D_RT_W;
		c.image_height_pixels = V3D_RT_H;
		c.maximum_bpp_of_all_render_targets = V3D_INTERNAL_BPP_32;
		c.early_z_disable = true;
	}
	CL_EMIT(mp, TILE_RENDERING_MODE_CFG_COLOR, c) {
		c.render_target_0_internal_type = V3D_INTERNAL_TYPE_8;
		c.render_target_0_internal_bpp = V3D_INTERNAL_BPP_32;
	}
	CL_EMIT(mp, TILE_RENDERING_MODE_CFG_CLEAR_COLORS_PART1, c) {
		c.render_target_number = 0;
		c.clear_color_low_32_bits = V3D_CLEAR_COLOR;
		c.clear_color_next_24_bits = 0;
	}
	CL_EMIT(mp, TILE_RENDERING_MODE_CFG_ZS_CLEAR_VALUES, c) { (void)c; }
	CL_EMIT(mp, MULTICORE_RENDERING_TILE_LIST_SET_BASE, c) {
		c.address.offset = V3D_TILEALLOC_VA;
		c.tile_list_set_number = 0;
	}
	CL_EMIT(mp, MULTICORE_RENDERING_SUPERTILE_CFG, c) {
		c.number_of_bin_tile_lists = 1;
		c.total_frame_width_in_tiles = 1;
		c.total_frame_height_in_tiles = 1;
		c.supertile_width_in_tiles = 1;
		c.supertile_height_in_tiles = 1;
		c.total_frame_width_in_supertiles = 1;
		c.total_frame_height_in_supertiles = 1;
	}
	CL_EMIT(mp, TILE_COORDINATES, c) { c.tile_column_number = 0; c.tile_row_number = 0; }
	for (i = 0; i < 2u; i++) {
		if (i > 0u) {
			CL_EMIT(mp, TILE_COORDINATES, c) { c.tile_column_number = 0; c.tile_row_number = 0; }
		}
		CL_EMIT(mp, END_OF_LOADS, c) { (void)c; }
		CL_EMIT(mp, STORE_TILE_BUFFER_GENERAL, c) { c.buffer_to_store = V3D_BUFFER_NONE; }
		if (i == 0u) {
			CL_EMIT(mp, CLEAR_TILE_BUFFERS, c) { c.clear_all_render_targets = true; }
		}
		CL_EMIT(mp, END_OF_TILE_MARKER, c) { (void)c; }
	}
	CL_EMIT(mp, FLUSH_VCD_CACHE, c) { (void)c; }
	CL_EMIT(mp, START_ADDRESS_OF_GENERIC_TILE_LIST, c) {
		c.start.offset = sublistVa;
		c.end.offset = sublistVa + sublen;
	}
	CL_EMIT(mp, SUPERTILE_COORDINATES, c) {
		c.column_number_in_supertiles = 0;
		c.row_number_in_supertiles = 0;
	}
	CL_EMIT(mp, END_OF_RENDERING, c) { (void)c; }
	rclEnd = V3D_RCL_VA + (uint32_t)(mp - m);

	/* Enable MMU + core init. */
	v3d_mmuEnable(v3d, ptPa, scratchPa);
	core0[V3D_CTL_MISCCFG / 4] = V3D_MISCCFG_OVRTMUOUT;
	v3d_invalidateCaches(core0);

	/* Bin on CT0; wait for FLDONE. */
	core0[V3D_CTL_INT_CLR / 4] = V3D_INT_FLDONE | V3D_INT_FRDONE | V3D_INT_OUTOMEM;
	core0[V3D_PTB_BPOS / 4] = 0u;
	core0[V3D_CLE_CT0QMA / 4] = V3D_TILEALLOC_VA;
	core0[V3D_CLE_CT0QMS / 4] = V3D_TILEALLOC_PAGES * _PAGE_SIZE;
	core0[V3D_CLE_CT0QTS / 4] = V3D_CLE_CT0QTS_ENABLE | V3D_TILESTATE_VA;
	core0[V3D_CLE_CT0QBA / 4] = V3D_BIN_CL_VA;
	core0[V3D_CLE_CT0QEA / 4] = binEnd;
	for (spins = 8000000u; spins != 0u; spins--) {
		if ((core0[V3D_CTL_INT_STS / 4] & (V3D_INT_FLDONE | V3D_INT_OUTOMEM)) != 0u) {
			break;
		}
	}
	printf("rpi4-v3d-scout: render: bin INT_STS=0x%08x\n", core0[V3D_CTL_INT_STS / 4]);
	core0[V3D_CTL_INT_CLR / 4] = V3D_INT_FLDONE | V3D_INT_FRDONE | V3D_INT_OUTOMEM;
	v3d_invalidateCaches(core0);

	/* Render on CT1; wait for FRDONE. */
	core0[V3D_CLE_CT1QBA / 4] = V3D_RCL_VA;
	core0[V3D_CLE_CT1QEA / 4] = rclEnd;
	for (spins = 16000000u; spins != 0u; spins--) {
		if ((core0[V3D_CTL_INT_STS / 4] & V3D_INT_FRDONE) != 0u) {
			break;
		}
	}
	ca = core0[V3D_CLE_CT1CA / 4];
	printf("rpi4-v3d-scout: render: CT1CA=0x%08x (end 0x%08x) INT_STS=0x%08x CT1CS=0x%08x ERR_FDBGS=0x%08x\n",
		ca, rclEnd, core0[V3D_CTL_INT_STS / 4], core0[V3D_CLE_CT1CS / 4], core0[V3D_ERR_FDBGS / 4]);

	/* Flush the V3D L2T cache (clean -> write back) so the TLB store reaches RAM. */
	core0[V3D_CTL_L2TCACTL / 4] = V3D_L2TCACTL_L2TFLS | (2u << 1); /* FLM_CLEAN */
	for (spins = V3D_SPIN_BOUND; spins != 0u &&
		(core0[V3D_CTL_L2TCACTL / 4] & V3D_L2TCACTL_L2TFLS) != 0u; spins--) {
	}

	px0 = rt[0];
	pxN = rt[(V3D_RT_W * V3D_RT_H) - 1u];
	okpix = 0u;
	for (i = 0; i < (V3D_RT_W * V3D_RT_H); i++) {
		if (rt[i] == V3D_CLEAR_COLOR) {
			okpix++;
		}
	}
	printf("rpi4-v3d-scout: RENDER test: RT[0]=0x%08x RT[last]=0x%08x match=%u/%u (want 0x%08x)\n",
		px0, pxN, okpix, V3D_RT_W * V3D_RT_H, V3D_CLEAR_COLOR);
	if (okpix == V3D_RT_W * V3D_RT_H) {
		printf("rpi4-v3d-scout: *** GPU CLEARED THE RENDER TARGET *** (Tier-4 4b-2 done; triangle next)\n");
	}
	else if (okpix > 0u) {
		printf("rpi4-v3d-scout: render PARTIAL: %u px cleared\n", okpix);
	}
	else {
		printf("rpi4-v3d-scout: render: no cleared pixels\n");
	}

	munmap(scratchPage, _PAGE_SIZE);
	munmap(rtPage, (size_t)V3D_RT_PAGES * _PAGE_SIZE);
	munmap(rclPage, _PAGE_SIZE);
	munmap(statePage, _PAGE_SIZE);
	munmap(allocPage, (size_t)V3D_TILEALLOC_PAGES * _PAGE_SIZE);
	munmap(binPage, _PAGE_SIZE);
	munmap(ptPage, _PAGE_SIZE);
}


int main(void)
{
	volatile uint32_t *v3d;
	void *v3d_page;
	uint32_t powerState, clkState, clkMax, clkSet, clkCur0, clkCur1;
	int i;

	/* Tier 3 step 0 (the key step, per Pi forum t=346267): "set QPU enabled" —
	 * the firmware brings the whole V3D up (domain+clock+AXI bridge+reset). Without
	 * this the V3D MMIO reads 0xdeadbeef even with the domain/clock calls below. */
	{
		uint32_t qpuEn = v3d_mboxProp(VC_PROP_SET_QPU_ENABLE, 1, 1u, 0u);
		printf("rpi4-v3d-scout: SET_QPU_ENABLE(1) -> 0x%08x%s\n", qpuEn,
			(qpuEn == MBOX_FAIL) ? " (mbox FAIL)" : "");
	}

	/* Tier 3 step 1: power on the V3D domain (gated until asked). */
	powerState = v3d_mboxProp(VC_PROP_SET_DOMAIN_STATE, 2, RPI_POWER_DOMAIN_V3D, 1u);
	printf("rpi4-v3d-scout: SET_DOMAIN_STATE(V3D=%u, on) -> 0x%08x%s\n",
		RPI_POWER_DOMAIN_V3D, powerState, (powerState == MBOX_FAIL) ? " (mbox FAIL)" : "");

	/* Tier 3 step 1b: enable + set the V3D core clock. Power-domain-on alone
	 * leaves the V3D MMIO returning 0xdeadbeef (bus error); the core needs its
	 * clock running. Each result on its OWN line (a single long line was getting
	 * truncated by concurrent smp: prints on the shared UART). */
	clkState = v3d_mboxProp(VC_PROP_SET_CLOCK_STATE, 2, RPI_CLOCK_V3D, 1u);
	printf("rpi4-v3d-scout: SET_CLOCK_STATE(5,on) -> 0x%08x\n", clkState);
	usleep(3000);
	clkCur0 = v3d_mboxProp(VC_PROP_GET_CLOCK_RATE, 2, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_CLOCK_RATE(5) before = %u\n", clkCur0);
	usleep(3000);
	clkMax = v3d_mboxProp(VC_PROP_GET_MAX_CLK_RATE, 2, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_MAX_CLK_RATE(5) = %u\n", clkMax);
	usleep(3000);
	clkSet = (clkMax != MBOX_FAIL && clkMax != 0u)
		? v3d_mboxProp(VC_PROP_SET_CLOCK_RATE, 2, RPI_CLOCK_V3D, clkMax)
		: MBOX_FAIL;
	printf("rpi4-v3d-scout: SET_CLOCK_RATE(5,%u) -> %u\n", clkMax, clkSet);
	usleep(3000);
	clkCur1 = v3d_mboxProp(VC_PROP_GET_CLOCK_RATE, 2, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_CLOCK_RATE(5) after = %u\n", clkCur1);

	/* Brief settle for the domain + clock to come up before the first MMIO access. */
	usleep(50000);

	/* Tier 3 step 1c (READ-ONLY recon): read the PM + rpivid_asb registers that a
	 * direct bcm2835-power-style V3D power-on would poke, BEFORE attempting any
	 * write. Tells us the regions are real and which half of the sequence is
	 * load-bearing (GRAFX island already up? V3D already de-reset?). */
	v3d_reconPmAsb();

	/* Tier 3 step 1d (WRITE): direct V3D power-on via PM_V3DRSTN deassert + the
	 * V3D async-AXI bridges (canonical BCM2711 bcm2835-power path). Recon proved
	 * the regions real + the bridges stopped + V3D in reset, so this is the
	 * load-bearing step the firmware/overlay didn't do. */
	v3d_powerOn();

	/* Tier 3 step 2: map the V3D MMIO and raw-dump the HUB identity region.
	 * Mapped uncached/device so reads hit the hardware directly. */
	v3d_page = mmap(NULL, V3D_MMIO_LEN, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)V3D_MMIO_BASE);
	if (v3d_page == MAP_FAILED) {
		printf("rpi4-v3d-scout: mmap(0x%08x) FAILED\n", V3D_MMIO_BASE);
		return 1;
	}
	v3d = (volatile uint32_t *)v3d_page;

	/* Dump the first 16 words (0x00..0x3c). The V3D HUB IDENT block lives here
	 * and embeds ASCII 'V','3','D'; a non-zero dump confirms the core is alive
	 * and reachable (the Tier-3 validation gate). Decode exact IDENT fields
	 * offline against Linux v3d_regs.h once the layout is confirmed from this. */
	printf("rpi4-v3d-scout: HUB @0x%08x dump:\n", V3D_MMIO_BASE);
	for (i = 0; i < 16; i++) {
		printf("rpi4-v3d-scout:   hub[0x%02x] = 0x%08x\n", i * 4, v3d[i]);
	}
	printf("rpi4-v3d-scout: CORE0 @0x%08x dump:\n", V3D_MMIO_BASE + V3D_CORE0_OFFS);
	for (i = 0; i < 16; i++) {
		printf("rpi4-v3d-scout:   core0[0x%02x] = 0x%08x\n", i * 4, v3d[(V3D_CORE0_OFFS / 4) + i]);
	}
	{
		uint32_t coreId0 = v3d[V3D_CORE0_IDENT0_OFFS / 4];
		uint32_t hubId1 = v3d[V3D_HUB_IDENT1_OFFS / 4];
		int alive = ((coreId0 & 0x00ffffffu) == V3D_CORE_IDENT0_SIG);
		if (alive) {
			printf("rpi4-v3d-scout: *** V3D ALIVE *** CORE0_IDENT0=0x%08x (\"V3D\" ver %u); "
				"HUB_IDENT1=0x%08x => V3D %u.%u, %u core(s)\n",
				coreId0, (coreId0 >> 24) & 0xffu, hubId1,
				hubId1 & 0xfu, (hubId1 >> 4) & 0xfu, (hubId1 >> 8) & 0xfu);

			/* Tier-4 BO+MMU+CLE proof is committed/manifested
			 * (2026-06-10-v3d-mmu-cle-foundation); skip re-running it here because a
			 * second CT0 job in the same boot won't re-arm without a control-thread
			 * reset. The bin pass below subsumes it (also exercises CT0 + the MMU).
			 *   v3d_mmuCleTest(v3d);
			 */

			/* Tier-4 4b-1 (GREEN, manifested): bin pass inits tile state. The 4b-2
			 * render clear (v3d_renderClearTest) now emits its CL via the ported Mesa
			 * packers and bin/render sync via FLDONE/FRDONE; bin completes (FLDONE),
			 * but the render thread parks at END_OF_LOADS in the per-tile list (FRDONE
			 * never fires) — a render-pipeline semantics issue under investigation. */
			v3d_renderClearTest(v3d); /* 4b-2 run */
		}
		else {
			printf("rpi4-v3d-scout: V3D still gated: CORE0_IDENT0=0x%08x (want \"V3D\" 0x..%06x)\n",
				coreId0, V3D_CORE_IDENT0_SIG);
		}
	}
	printf("rpi4-v3d-scout: done\n");

	munmap(v3d_page, V3D_MMIO_LEN);

	/* One-shot probe: park so the process stays a well-defined no-op rather than
	 * exiting (matches the other rpi4 device daemons' lifetime). */
	for (;;) {
		sleep(3600);
	}

	return 0;
}
