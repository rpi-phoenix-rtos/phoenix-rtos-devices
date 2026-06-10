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
		if ((coreId0 & 0x00ffffffu) == V3D_CORE_IDENT0_SIG) {
			printf("rpi4-v3d-scout: *** V3D ALIVE *** CORE0_IDENT0=0x%08x (\"V3D\" ver %u); "
				"HUB_IDENT1=0x%08x => V3D %u.%u, %u core(s)\n",
				coreId0, (coreId0 >> 24) & 0xffu, hubId1,
				hubId1 & 0xfu, (hubId1 >> 4) & 0xfu, (hubId1 >> 8) & 0xfu);
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
