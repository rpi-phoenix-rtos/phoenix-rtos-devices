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


/*
 * Two-u32-in property call (domain, state). Returns the firmware's resulting
 * state word, or MBOX_FAIL. Same cache/transport discipline as rpi4-thermal's
 * 1in1out: device-mapped uncached mailbox window + uncached, contiguous,
 * physically-pinned property buffer (no explicit clean/invalidate needed).
 */
static uint32_t v3d_mboxProp2(uint32_t tag, uint32_t w0, uint32_t w1)
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

	/* [size, REQUEST, tag, valbuf=8, req=0, w0, w1, END]. */
	msg[0] = 32;
	msg[1] = 0;
	msg[2] = tag;
	msg[3] = 8;
	msg[4] = 0;
	msg[5] = w0;
	msg[6] = w1;
	msg[7] = 0;

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
		result = msg[6];
	}

	munmap(msg_page, _PAGE_SIZE);
	munmap(mbox_page, _PAGE_SIZE);
	return result;
}


int main(void)
{
	volatile uint32_t *v3d;
	void *v3d_page;
	uint32_t powerState, clkState, clkMax, clkSet, clkCur0, clkCur1;
	int i;

	/* Tier 3 step 1: power on the V3D domain (gated until asked). */
	powerState = v3d_mboxProp2(VC_PROP_SET_DOMAIN_STATE, RPI_POWER_DOMAIN_V3D, 1u);
	printf("rpi4-v3d-scout: SET_DOMAIN_STATE(V3D=%u, on) -> 0x%08x%s\n",
		RPI_POWER_DOMAIN_V3D, powerState, (powerState == MBOX_FAIL) ? " (mbox FAIL)" : "");

	/* Tier 3 step 1b: enable + set the V3D core clock. Power-domain-on alone
	 * leaves the V3D MMIO returning 0xdeadbeef (bus error); the core needs its
	 * clock running. Each result on its OWN line (a single long line was getting
	 * truncated by concurrent smp: prints on the shared UART). */
	clkState = v3d_mboxProp2(VC_PROP_SET_CLOCK_STATE, RPI_CLOCK_V3D, 1u);
	printf("rpi4-v3d-scout: SET_CLOCK_STATE(5,on) -> 0x%08x\n", clkState);
	usleep(3000);
	clkCur0 = v3d_mboxProp2(VC_PROP_GET_CLOCK_RATE, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_CLOCK_RATE(5) before = %u\n", clkCur0);
	usleep(3000);
	clkMax = v3d_mboxProp2(VC_PROP_GET_MAX_CLK_RATE, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_MAX_CLK_RATE(5) = %u\n", clkMax);
	usleep(3000);
	clkSet = (clkMax != MBOX_FAIL && clkMax != 0u)
		? v3d_mboxProp2(VC_PROP_SET_CLOCK_RATE, RPI_CLOCK_V3D, clkMax)
		: MBOX_FAIL;
	printf("rpi4-v3d-scout: SET_CLOCK_RATE(5,%u) -> %u\n", clkMax, clkSet);
	usleep(3000);
	clkCur1 = v3d_mboxProp2(VC_PROP_GET_CLOCK_RATE, RPI_CLOCK_V3D, 0u);
	printf("rpi4-v3d-scout: GET_CLOCK_RATE(5) after = %u\n", clkCur1);

	/* Brief settle for the domain + clock to come up before the first MMIO access. */
	usleep(50000);

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
	printf("rpi4-v3d-scout: done (non-zero IDENT words => V3D alive)\n");

	munmap(v3d_page, V3D_MMIO_LEN);

	/* One-shot probe: park so the process stays a well-defined no-op rather than
	 * exiting (matches the other rpi4 device daemons' lifetime). */
	for (;;) {
		sleep(3600);
	}

	return 0;
}
