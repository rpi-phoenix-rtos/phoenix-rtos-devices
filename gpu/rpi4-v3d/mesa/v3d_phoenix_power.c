/*
 * v3d_phoenix_power.c — self-contained BCM2711 V3D power-on for the Phoenix winsys.
 *
 * Ported verbatim from the HW-proven rpi4-v3d-scout power-on (manifest
 * 2026-06-10-v3d-power-on-alive): mailbox SET_QPU_ENABLE + SET_DOMAIN_STATE +
 * SET_CLOCK_STATE, then the canonical bcm2835_asb_power_on (clock-toggle around a
 * PM_V3DRSTN deassert, then enable the V3D master+slave async-AXI bridges on
 * rpivid_asb). Without this the V3D core MMIO reads 0xdeadbeef (unpowered) and
 * SUBMIT_CL never executes. Idempotent — safe to call once at winsys_init.
 *
 * The winsys owns power-on itself (rather than depending on a separate scout
 * process) to avoid a race: a concurrent scout's clock-toggle/reset overlapping
 * our submit window left core0 reading 0xdeadbeef.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include "libvcmbox.h"   /* serialized /dev/vcmbox client (single-FIFO arbitration) */

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
/* GET property tags for the cold-power-on state probe (render-stall STEP-3 discriminator). */
#define VC_PROP_GET_CLOCK_RATE      0x00030002u   /* configured rate (Hz), w0=clock id */
#define VC_PROP_GET_CLOCK_MEASURED  0x00030047u   /* actual measured rate (Hz) — reveals an unlocked/unsettled PLL */
#define VC_PROP_GET_CLOCK_STATE     0x00030001u   /* bit0 on, bit1 not-exists */
#define VC_PROP_GET_TEMPERATURE     0x00030006u   /* SoC temp (milli-degC), w0=temp id 0 */
#define VC_PROP_GET_THROTTLED       0x00030046u   /* throttle/undervolt bitmask */
#define VC_PROP_GET_VOLTAGE         0x00030003u   /* w0=voltage id; id 1 = core */
#define VC_PROP_GET_VIRTUAL_WH      0x00040004u   /* virtual (buffer) w,h — h>=2*phys => double-buffer */
#define VC_PROP_SET_VIRTUAL_OFFSET  0x00048009u   /* pan: (x,y) — page-flip by setting y=buffer*phys_h */
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

/* Stop a V3D async-AXI bridge (set REQ_STOP, wait for ACK) — the power-off direction,
 * mirror of asbEnable. Used by v3d_phoenix_reset to quiesce the bridges before reset. */
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

/* TRUE V3D reset cycle: quiesce the AXI bridges, assert PM_V3DRSTN (hold the V3D in
 * reset), then power back on (clock toggle + RSTN deassert + bridge re-enable). This is
 * the "reboot-like" reset a per-iteration repro harness needs to re-roll the power-on
 * lottery each trial (v3d_phoenix_powerOn alone only re-deasserts; it does not hold the
 * core in reset, so it cannot re-create the cold first-frame condition). Returns 0 on
 * success. */
int v3d_phoenix_reset(void)
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
	return v3d_phoenix_powerOn();
}

/* Full HW-proven V3D power-on. Returns 0 on success (both ASB bridges ACK). */
int v3d_phoenix_powerOn(void)
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
	printf("v3d_phoenix_powerOn: PM_GRAFX 0x%08x->0x%08x asb M=%s S=%s\n",
	       grafx, pm[PM_GRAFX / 4], rcM ? "TIMEOUT" : "ok", rcS ? "TIMEOUT" : "ok");
	usleep(2000);
	munmap(asb_page, _PAGE_SIZE);
	munmap(pm_page, _PAGE_SIZE);
	return (rcM == 0 && rcS == 0) ? 0 : -1;
}

/* STEP-3 cold-power-on state probe (render-stall hunt, task #13). The render wedge is
 * cold-boot-determined and software-reset-immune (60/60 in-process resets never reproduced
 * it; clean boots render thousands of frames, stalled boots wedge from frame 1), and is
 * INDIFFERENT to BO memory content (zeroing changed the wedge bytes but not the rate). That
 * profile points away from memory/cache and at firmware-controlled cold-power-on HW state —
 * the one thing a reboot re-rolls but an in-process reset does not. This logs the candidate
 * discriminators ONCE at winsys init so a multi-boot run can diff a stalled boot's line vs a
 * clean one: a register that cleanly separates the two populations is the root cause, found in
 * far fewer boots than an underpowered rate A/B. KEY signal: clk_v3d configured-vs-measured
 * mismatch (an unlocked/unsettled V3D PLL would let the render control thread run wild). */
void v3d_phoenix_logColdState(void);
void v3d_phoenix_logColdState(void)
{
	uint32_t rate_cfg  = mboxProp(VC_PROP_GET_CLOCK_RATE,     2, RPI_CLOCK_V3D, 0u);
	uint32_t rate_meas = mboxProp(VC_PROP_GET_CLOCK_MEASURED, 2, RPI_CLOCK_V3D, 0u);
	uint32_t clk_state = mboxProp(VC_PROP_GET_CLOCK_STATE,    2, RPI_CLOCK_V3D, 0u);
	uint32_t temp      = mboxProp(VC_PROP_GET_TEMPERATURE,    2, 0u, 0u);
	uint32_t throttled = mboxProp(VC_PROP_GET_THROTTLED,      1, 0u, 0u);
	uint32_t volt      = mboxProp(VC_PROP_GET_VOLTAGE,        2, 1u, 0u);   /* core voltage */
	uint32_t grafx = 0xffffffffu;
	void *pm_page = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (addr_t)PM_BASE);
	if (pm_page != MAP_FAILED) {
		grafx = ((volatile uint32_t *)pm_page)[PM_GRAFX / 4];
		munmap(pm_page, _PAGE_SIZE);
	}
	/* delta_kHz: signed gap between firmware-configured and hardware-measured V3D clock. A
	 * large gap => the V3D PLL had not locked/settled at first render = the prime stall lead. */
	long delta_khz = ((long)rate_meas - (long)rate_cfg) / 1000;
	printf("v3d-coldstate: clk_v3d cfg=%u Hz meas=%u Hz delta=%ld kHz clkstate=0x%x "
	       "temp=%u mC throttled=0x%08x corevolt=%u uV pm_grafx=0x%08x\n",
	       rate_cfg, rate_meas, delta_khz, clk_state, temp, throttled, volt, grafx);
}

/* Double-buffer / page-flip support (render-stall complete fix). plo allocates the fb with
 * virtual height = 2x physical so a second buffer sits below the first; these query the granted
 * virtual height (to confirm double-buffer is available) and page-flip by panning the display
 * origin. The displayed buffer is never the one the GPU is writing -> no GPU-write/display-read
 * contention -> the depth/fragment-pipeline stall cannot occur. */
unsigned v3d_phoenix_fb_virtual_height(void);
unsigned v3d_phoenix_fb_virtual_height(void)
{
	/* PRIMARY: query the granted virtual height through the SERIALIZED /dev/vcmbox server
	 * (libvcmbox). The BCM2711 has a single VideoCore property-mailbox FIFO with NO hardware
	 * arbitration: if two processes drive it concurrently, one read loop pops and discards the
	 * other's response. The winsys was driving the FIFO DIRECTLY (mboxProp below), so it raced
	 * the vcmbox server and the other mailbox clients (thermal, usb, genet); a concurrent reader
	 * drained our GET_VIRTUAL_WH response -> spurious failure -> single-buffer -> dynamic-model
	 * flicker (netboot won the race often, SD lost it consistently -> always flickered). Routing
	 * the transaction through the server (which owns the FIFO and handles one msg at a time)
	 * eliminates the race. GET_VIRTUAL_WH returns {width, height}; out[1] = granted virt height.
	 * SAFE: reports only what the firmware actually granted — never assumes ungranted buffers. */
	{
		uint32_t in2[2] = { 0u, 0u };
		uint32_t out2[2] = { 0u, 0u };
		if ((vcmbox_call(VC_PROP_GET_VIRTUAL_WH, 8u, in2, 2u, out2, 2u) == 0) && (out2[1] != 0u)) {
			return out2[1];
		}
	}

	/* FALLBACK: /dev/vcmbox unavailable (server never registered). Retry the direct-FIFO GET —
	 * still racy, but better than nothing. A single failed GET demotes the renderer to
	 * single-buffer (render-to-scanout into the live fb -> tearing/flicker). */
	for (int i = 0; i < 8; i++) {
		uint32_t h = mboxProp(VC_PROP_GET_VIRTUAL_WH, 2, 0u, 0u);   /* returns granted virtual height */
		if (h != MBOX_FAIL && h != 0u) {
			if (i > 0) {
				printf("v3d-winsys: GET_VIRTUAL_WH (direct fallback) recovered on retry %d -> virt_h=%u\n", i, h);
			}
			return h;
		}
		usleep(3000);
	}
	printf("v3d-winsys: GET_VIRTUAL_WH failed via /dev/vcmbox AND 8 direct retries -> single-buffer "
	       "fallback (render-to-scanout into live fb; expect tearing).\n");
	return 0u;
}

void v3d_phoenix_fb_flip(unsigned yoff);
void v3d_phoenix_fb_flip(unsigned yoff)
{
	(void)mboxProp(VC_PROP_SET_VIRTUAL_OFFSET, 2, 0u, yoff);   /* pan display origin to (0, yoff) */
}
