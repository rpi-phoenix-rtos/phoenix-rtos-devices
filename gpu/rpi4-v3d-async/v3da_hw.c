/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - hardware
 *
 * Power/clock (through /dev/vcmbox only), MMIO, identity, the one MMU page table,
 * the GPU-VA allocator, core register set-up, and the interrupt path (handler,
 * poll fallback, runtime on/off, self-test).
 *
 * Taken from the rpi4-v3d daemon (gpu/rpi4-v3d/v3d_gpu.c: power-on order, asbEnable/
 * asbStop, apply_core_regs, va_alloc/va_free, mmu_flush_tlb, idle_axi), which was
 * itself copied from the in-process winsys (mesa/v3d_phoenix_winsys.c,
 * mesa/v3d_phoenix_power.c). Changes against that copy:
 *   - every mailbox call goes through the serialized /dev/vcmbox, with the V3D
 *     clock read back until the firmware confirms it (the lesson of the 2026-09-11
 *     clock race); the direct-FIFO mboxProp() is not carried at all;
 *   - interrupt masks are programmed explicitly (the old lane never touched them).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/threads.h>

#include "libvcmbox.h"
#include "v3da.h"
#include "v3da_regs.h"


/* Firmware property tags (VideoCore mailbox, via /dev/vcmbox) */
#define VC_PROP_GET_CLOCK_STATE    0x00030001u
#define VC_PROP_GET_CLOCK_RATE     0x00030002u
#define VC_PROP_SET_QPU_ENABLE     0x00030012u
#define VC_PROP_GET_CLOCK_MEASURED 0x00030047u
#define VC_PROP_SET_CLOCK_STATE    0x00038001u
#define VC_PROP_SET_DOMAIN_STATE   0x00038030u
#define RPI_POWER_DOMAIN_V3D       10u
#define RPI_CLOCK_V3D              5u
#define V3D_CLOCK_ON_TRIES         20u

/* PM + rpivid_asb (BCM2711 V3D power/reset path) */
#define PM_BASE                    0xfe100000u
#define RPIVID_ASB_BASE            0xfec11000u
#define PM_GRAFX                   0x10cu
#define PM_V3DRSTN                 (1u << 6)
#define PM_PASSWORD                0x5a000000u
#define ASB_V3D_S_CTRL             0x08u
#define ASB_V3D_M_CTRL             0x0cu
#define ASB_REQ_STOP               (1u << 0)
#define ASB_ACK                    (1u << 1)
#define ASB_ACK_SPINS              100000u

/* GPU VA window: 256 PT pages * 4 MiB = 1 GiB (as the old lane, sized for STK). */
#define GPUVA_BASE                 0x100000u
#define GPUVA_PT_PAGES             256u

/* Storm guard thresholds (handler side). */
#define IRQ_SPURIOUS_LIMIT         1000u
#define IRQ_BURST_LIMIT            200000u


static volatile uint32_t *map_dev(addr_t pa, size_t len)
{
	void *p = mmap(NULL, len, PROT_READ | PROT_WRITE,
		MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, pa);
	return (p == MAP_FAILED) ? NULL : (volatile uint32_t *)p;
}


/* ========================================================================= */
/* Power / clock through /dev/vcmbox                                          */
/* ========================================================================= */

/* One property call, two value words. Returns 0 and both answer words, or <0. */
int v3da_hw_vc_prop2(uint32_t tag, uint32_t w0, uint32_t w1, uint32_t nIn, uint32_t *ans0, uint32_t *ans1)
{
	uint32_t in[2], out[2];
	int rc;

	in[0] = w0;
	in[1] = w1;
	out[0] = 0u;
	out[1] = 0u;
	rc = vcmbox_call(tag, 8u, in, nIn, out, 2u);
	if (rc == 0) {
		if (ans0 != NULL) {
			*ans0 = out[0];
		}
		if (ans1 != NULL) {
			*ans1 = out[1];
		}
	}
	return rc;
}


/* The id+value form (clock / power-domain tags answer in word 1). */
static int vc_prop2(uint32_t tag, uint32_t w0, uint32_t w1, uint32_t nIn, uint32_t *answer)
{
	return v3da_hw_vc_prop2(tag, w0, w1, nIn, NULL, answer);
}


static int v3d_clock_set(uint32_t on)
{
	return vc_prop2(VC_PROP_SET_CLOCK_STATE, RPI_CLOCK_V3D, on, 2u, NULL);
}


/* Enable the V3D clock and CONFIRM it came on (read back; never trust the SET).
 * An MMIO read of an unclocked V3D never completes; the in-process winsys hung
 * forever that way on 2026-09-11 (mesa/v3d_phoenix_power.c:394-461). */
static int v3d_clock_on_confirmed(unsigned *tries)
{
	uint32_t st;
	unsigned t;

	for (t = 0u; t < V3D_CLOCK_ON_TRIES; t++) {
		(void)v3d_clock_set(1u);
		if ((vc_prop2(VC_PROP_GET_CLOCK_STATE, RPI_CLOCK_V3D, 0u, 1u, &st) == 0) && ((st & 1u) != 0u)) {
			*tries = t;
			return 0;
		}
		usleep(200);
	}
	*tries = t;
	return -1;
}


static int asb_enable(volatile uint32_t *asb, uint32_t reg)
{
	uint32_t val = asb[reg / 4u] & ~ASB_REQ_STOP;
	uint32_t spins;

	asb[reg / 4u] = PM_PASSWORD | val;
	for (spins = ASB_ACK_SPINS; spins != 0u; spins--) {
		if ((asb[reg / 4u] & ASB_ACK) == 0u) {
			return 0;
		}
	}
	return -1;
}


static int asb_stop(volatile uint32_t *asb, uint32_t reg)
{
	uint32_t val = asb[reg / 4u] | ASB_REQ_STOP;
	uint32_t spins;

	asb[reg / 4u] = PM_PASSWORD | val;
	for (spins = ASB_ACK_SPINS; spins != 0u; spins--) {
		if ((asb[reg / 4u] & ASB_ACK) != 0u) {
			return 0;
		}
	}
	return -1;
}


/* The HW-proven power-on order (v3d_gpu.c:425-470, v3d_phoenix_power.c:465-522):
 * QPU enable, power domain, clock on; clock on/off around the PM_V3DRSTN deassert;
 * clock on WITH read-back; both async-AXI bridges. Every vcmbox result is checked. */
static int v3d_power_on(v3da_hw_t *hw)
{
	volatile uint32_t *pm, *asb;
	uint32_t grafx, grafx2, ans;
	int rcQ, rcD, rcM, rcS, rcClk;

	{
		/* One value word, as the proven sequence sends it (mboxProp nw=1). */
		uint32_t qin = 1u, qout = 0u;
		rcQ = vcmbox_call(VC_PROP_SET_QPU_ENABLE, 4u, &qin, 1u, &qout, 1u);
	}
	rcD = vc_prop2(VC_PROP_SET_DOMAIN_STATE, RPI_POWER_DOMAIN_V3D, 1u, 2u, &ans);
	(void)v3d_clock_set(1u);

	pm = map_dev((addr_t)PM_BASE, _PAGE_SIZE);
	asb = map_dev((addr_t)RPIVID_ASB_BASE, _PAGE_SIZE);
	if ((pm == NULL) || (asb == NULL)) {
		printf("V3DA srv power map=FAILED\n");
		return -ENOMEM;
	}

	(void)v3d_clock_set(1u);
	usleep(50);
	(void)v3d_clock_set(0u);
	grafx = pm[PM_GRAFX / 4u];
	pm[PM_GRAFX / 4u] = PM_PASSWORD | (grafx | PM_V3DRSTN);
	rcClk = v3d_clock_on_confirmed(&hw->clk_tries);
	usleep(50);

	rcM = asb_enable(asb, ASB_V3D_M_CTRL);
	rcS = asb_enable(asb, ASB_V3D_S_CTRL);
	grafx2 = pm[PM_GRAFX / 4u];
	usleep(2000);

	printf("V3DA srv power vcmbox=ok qpu=%d domain=%d clk=%s tries=%u grafx=0x%08x->0x%08x asb_m=%s asb_s=%s\n",
		rcQ, rcD, (rcClk == 0) ? "on" : "NOT-CONFIRMED", hw->clk_tries, grafx, grafx2,
		(rcM == 0) ? "ok" : "TIMEOUT", (rcS == 0) ? "ok" : "TIMEOUT");

	(void)munmap((void *)asb, _PAGE_SIZE);
	(void)munmap((void *)pm, _PAGE_SIZE);

	/* An unconfirmed clock is fatal: the next act would be a V3D MMIO read. */
	return ((rcClk == 0) && (rcM == 0) && (rcS == 0)) ? 0 : -EIO;
}


/* TRUE V3D reset: quiesce both bridges, hold PM_V3DRSTN asserted, power on again.
 * Used by the wedge recovery (v3da_hw_reset). */
static int v3da_hw_reset_power(v3da_hw_t *hw)
{
	volatile uint32_t *pm, *asb;
	uint32_t grafx;

	pm = map_dev((addr_t)PM_BASE, _PAGE_SIZE);
	asb = map_dev((addr_t)RPIVID_ASB_BASE, _PAGE_SIZE);
	if ((pm == NULL) || (asb == NULL)) {
		return -ENOMEM;
	}
	(void)asb_stop(asb, ASB_V3D_M_CTRL);
	(void)asb_stop(asb, ASB_V3D_S_CTRL);
	grafx = pm[PM_GRAFX / 4u];
	pm[PM_GRAFX / 4u] = PM_PASSWORD | (grafx & ~PM_V3DRSTN);
	usleep(100);
	(void)munmap((void *)asb, _PAGE_SIZE);
	(void)munmap((void *)pm, _PAGE_SIZE);

	return v3d_power_on(hw);
}


/* ========================================================================= */
/* Core registers, MMU                                                        */
/* ========================================================================= */

/* Verbatim register set of the old lane's apply_core_regs (v3d_gpu.c:575-599). */
static void apply_core_regs(v3da_hw_t *hw)
{
	hw->hub[MMU_PT_PA_BASE / 4u] = (uint32_t)(hw->pt_pa >> V3D_PAGE_SHIFT);
	hw->hub[MMU_CTL / 4u] = MMU_CTL_ENABLE | MMU_CTL_PTI_ENABLE | MMU_CTL_PTI_ABORT | MMU_CTL_PTI_INT |
		MMU_CTL_WRITEVIO_ABORT | MMU_CTL_WRITEVIO_INT | MMU_CTL_CAPEXC_ABORT | MMU_CTL_CAPEXC_INT;
	if (hw->scratch_pa != 0u) {
		hw->hub[MMU_ILLEGAL_ADDR / 4u] = (uint32_t)(hw->scratch_pa >> V3D_PAGE_SHIFT) | MMU_ILLEGAL_ENABLE;
	}
	hw->hub[MMUC_CONTROL / 4u] = MMUC_ENABLE;
	hw->core0[CTL_L2CACTL / 4u] = L2CACTL_L2CCLR | L2CACTL_L2CENA;
	hw->core0[CTL_L2TFLSTA / 4u] = 0u;
	hw->core0[CTL_L2TFLEND / 4u] = ~0u;
	hw->hub[HUB_AXICFG / 4u] = HUB_AXICFG_MAX_LEN;   /* GFXH-1383 */
	hw->core0[CTL_MISCCFG / 4u] = (V3D_QRMAXCNT << MISCCFG_QRMAXCNT_SHIFT) | MISCCFG_OVRTMUOUT;
}


/* Best-effort AXI drain before a reset: ask the GMP to quiesce outstanding
 * transactions (old daemon's idle_axi, v3d_gpu.c:1041-1049; Linux v3d_idle_axi). */
static void idle_axi(v3da_hw_t *hw)
{
	uint32_t spins;

	hw->core0[GMP_CFG / 4u] = GMP_CFG_STOP_REQ;
	for (spins = 1000000u; spins != 0u; spins--) {
		if ((hw->core0[GMP_STATUS / 4u] & (GMP_STATUS_RD_WR_CNT | GMP_STATUS_CFG_BUSY)) == 0u) {
			break;
		}
	}
}


void v3da_hw_l2t_flush_wait(v3da_hw_t *hw)
{
	uint32_t spins;

	for (spins = 1000000u; (spins != 0u) && ((hw->core0[CTL_L2TCACTL / 4u] & L2TCACTL_L2TFLS) != 0u); spins--) {
	}
}


/* Mask every source and drop anything latched (design section 3.3, steps 2-3). */
static void irq_mask_all(v3da_hw_t *hw)
{
	hw->core0[CTL_INT_MSK_SET / 4u] = ~0u;
	hw->hub[HUB_INT_MSK_SET / 4u] = ~0u;
	hw->core0[CTL_INT_CLR / 4u] = ~0u;
	hw->hub[HUB_INT_CLR / 4u] = ~0u;
}


/* MMUC PTE-cache flush + TLB clear (old lane's mmu_flush_tlb, v3d_gpu.c:952-959). */
void v3da_hw_mmu_flush(v3da_hw_t *hw)
{
	uint32_t spins;

	__asm__ volatile("dsb sy" ::: "memory");   /* PTE stores (uncached) before the flush */
	hw->hub[MMUC_CONTROL / 4u] = MMUC_FLUSH | MMUC_ENABLE;
	for (spins = 1000000u; (spins != 0u) && ((hw->hub[MMUC_CONTROL / 4u] & MMUC_FLUSHING) != 0u); spins--) {
	}
	hw->hub[MMU_CTL / 4u] |= MMU_CTL_TLB_CLEAR;
	for (spins = 1000000u; (spins != 0u) && ((hw->hub[MMU_CTL / 4u] & MMU_CTL_TLB_CLEARING) != 0u); spins--) {
	}
	hw->tlb_gen = hw->pt_gen;
	hw->tlb_flushes++;
}


/* GPU VA allocator: first-fit hole, else bump; new bump ranges get their PTEs
 * cleared at hand-out (old daemon, v3d_gpu.c:522-566). */
uint32_t v3da_hw_va_alloc(v3da_hw_t *hw, uint32_t pages)
{
	uint32_t i, va;

	for (i = 0u; i < hw->nholes; i++) {
		if (hw->holes[i].pages >= pages) {
			va = hw->holes[i].gpuva;
			if (hw->holes[i].pages == pages) {
				hw->holes[i] = hw->holes[--hw->nholes];
			}
			else {
				hw->holes[i].gpuva += pages * _PAGE_SIZE;
				hw->holes[i].pages -= pages;
			}
			return va;
		}
	}
	if ((hw->next_gpuva >> V3D_PAGE_SHIFT) + pages > hw->pt_entries) {
		return 0u;
	}
	va = hw->next_gpuva;
	hw->next_gpuva += pages * _PAGE_SIZE;
	for (i = 0u; i < pages; i++) {
		hw->pt[(va >> V3D_PAGE_SHIFT) + i] = 0u;
	}
	return va;
}


/* Return a VA range. The caller has already cleared its PTEs AND passed the
 * quarantine (TLB flushed, fence pass), so the range is safe to hand out again. */
void v3da_hw_va_free(v3da_hw_t *hw, uint32_t gpuva, uint32_t pages)
{
	if (hw->nholes < V3DA_MAX_HOLES) {
		hw->holes[hw->nholes].gpuva = gpuva;
		hw->holes[hw->nholes].pages = pages;
		hw->nholes++;
	}
	/* else: the VA leaks (bounded); bump allocation still serves. */
}


/* ========================================================================= */
/* Interrupts                                                                 */
/* ========================================================================= */

/* Read, clear and record both status registers. Shared by the IRQ handler
 * (kernel context: MMIO + atomics only - no libc, no locks, no faults) and the
 * poll path. Returns the bits seen. */
static inline uint32_t hw_service(v3da_hw_t *hw, uint32_t *hub_out)
{
	uint32_t core = hw->core0[CTL_INT_STS / 4u];
	uint32_t hub = hw->hub[HUB_INT_STS / 4u];
	uint32_t va, size;

	if (core != 0u) {
		/* Raw status, QPU bits included: Linux parity (v3d_irq.c:107-110); an
		 * unacknowledged QPU interrupt stalls fragment dispatch (E2 step 13). */
		hw->core0[CTL_INT_CLR / 4u] = core;
	}
	if (hub != 0u) {
		if ((hub & HUB_INT_MMU_ANY) != 0u) {
			uint32_t ctl = hw->hub[MMU_CTL / 4u];
			__atomic_store_n(&hw->mmu_ctl_seen, ctl, __ATOMIC_RELAXED);
			hw->hub[MMU_CTL / 4u] = ctl;   /* W1C the fault status (v3d_irq.c:202) */
		}
		hw->hub[HUB_INT_CLR / 4u] = hub;
	}
	if ((core & INT_OUTOMEM) != 0u) {
		/* Hand the pre-staged overflow chunk at once: the binner is stalled until
		 * it gets memory, and waking a thread first would add its latency. With
		 * nothing staged, count it: the event thread grants from the pool. */
		va = __atomic_exchange_n(&hw->ovf_stage_va, 0u, __ATOMIC_ACQ_REL);
		if (va != 0u) {
			size = __atomic_load_n(&hw->ovf_stage_size, __ATOMIC_ACQUIRE);
			hw->core0[PTB_BPOA / 4u] = va;
			hw->core0[PTB_BPOS / 4u] = size;
			__atomic_store_n(&hw->ovf_consumed, va, __ATOMIC_RELEASE);
		}
		else {
			(void)__atomic_add_fetch(&hw->ovf_missed, 1u, __ATOMIC_RELEASE);
		}
	}
	if (core != 0u) {
		(void)__atomic_fetch_or(&hw->ev_core, core, __ATOMIC_RELEASE);
	}
	if (hub != 0u) {
		(void)__atomic_fetch_or(&hw->ev_hub, hub, __ATOMIC_RELEASE);
	}
	*hub_out = hub;
	return core;
}


static int v3da_irq_handler(unsigned int n, void *arg)
{
	v3da_hw_t *hw = arg;
	uint32_t core, hub, cnt;

	(void)n;

	cnt = __atomic_add_fetch(&hw->irq_count, 1u, __ATOMIC_RELAXED);
	core = hw_service(hw, &hub);

	/* Storm guard. (a) The line fires with nothing pending: something we did not
	 * clear keeps it asserted. (b) A source re-latches as fast as we clear it: the
	 * event thread snapshots irq_count every loop, so a burst this long without a
	 * single event-thread loop is a livelock. Either way mask the whole block and
	 * let the event thread fall back to polling. */
	if (((core | hub) == 0u) &&
			(__atomic_add_fetch(&hw->irq_spurious, 1u, __ATOMIC_RELAXED) > IRQ_SPURIOUS_LIMIT)) {
		hw->core0[CTL_INT_MSK_SET / 4u] = ~0u;
		hw->hub[HUB_INT_MSK_SET / 4u] = ~0u;
		__atomic_store_n(&hw->storm, 1u, __ATOMIC_RELEASE);
		return 1;
	}
	if ((cnt - __atomic_load_n(&hw->irq_count_snap, __ATOMIC_RELAXED)) > IRQ_BURST_LIMIT) {
		hw->core0[CTL_INT_MSK_SET / 4u] = ~0u;
		hw->hub[HUB_INT_MSK_SET / 4u] = ~0u;
		__atomic_store_n(&hw->storm, 2u, __ATOMIC_RELEASE);
		return 1;
	}
	return ((core | hub) != 0u) ? 1 : -1;   /* -1: nothing for us, no wake-up */
}


/* Thread context, before a kick: fold every pending status bit EXCEPT OUTOMEM
 * into the event words and clear it. OUTOMEM stays latched so that exactly one
 * path (handler or poll) hands out overflow memory - two grants for one stall
 * would re-point a binner that already runs on the first chunk. Safe against a
 * concurrent handler: both only read, W1C and OR. */
void v3da_hw_drain(v3da_hw_t *hw)
{
	uint32_t core = hw->core0[CTL_INT_STS / 4u] & ~INT_OUTOMEM;
	uint32_t hub = hw->hub[HUB_INT_STS / 4u];

	if (core != 0u) {
		hw->core0[CTL_INT_CLR / 4u] = core;
		(void)__atomic_fetch_or(&hw->ev_core, core, __ATOMIC_RELEASE);
	}
	if (hub != 0u) {
		if ((hub & HUB_INT_MMU_ANY) != 0u) {
			uint32_t ctl = hw->hub[MMU_CTL / 4u];
			__atomic_store_n(&hw->mmu_ctl_seen, ctl, __ATOMIC_RELAXED);
			hw->hub[MMU_CTL / 4u] = ctl;
		}
		hw->hub[HUB_INT_CLR / 4u] = hub;
		(void)__atomic_fetch_or(&hw->ev_hub, hub, __ATOMIC_RELEASE);
	}
}


/* Poll mode: the same service routine, run by the event thread. */
void v3da_hw_poll_status(v3da_hw_t *hw)
{
	uint32_t hub;

	(void)hw_service(hw, &hub);
}


/* Switch interrupt-driven completion on (design section 3.3, steps 2-5). Called
 * with srv.lock held. Returns the interrupt() result (>= 0 on success). */
int v3da_hw_irq_enable(v3da_hw_t *hw)
{
	int rc;

	if (hw->irq_on != 0) {
		return 0;
	}
	irq_mask_all(hw);
	__atomic_store_n(&hw->storm, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->irq_spurious, 0u, __ATOMIC_RELEASE);

	rc = interrupt(hw->irq_num, v3da_irq_handler, hw, hw->irq_cond, &hw->irq_handle);
	if (rc < 0) {
		return rc;
	}
	hw->core0[CTL_INT_MSK_CLR / 4u] = CORE_IRQS;
	hw->hub[HUB_INT_MSK_CLR / 4u] = HUB_IRQS;
	hw->irq_on = 1;
	return rc;
}


/* Back to polling: mask everything, then drop the handler (userintr_put disables
 * the GIC line when it was the last handler). Called with srv.lock held. */
void v3da_hw_irq_disable(v3da_hw_t *hw)
{
	hw->core0[CTL_INT_MSK_SET / 4u] = ~0u;
	hw->hub[HUB_INT_MSK_SET / 4u] = ~0u;
	if (hw->irq_on != 0) {
		(void)resourceDestroy(hw->irq_handle);
		hw->irq_on = 0;
	}
}


/* Wedge recovery (design section 7). Called with srv.lock held by the event
 * thread. Same order as the old lane's reset_reinit_core (drain GMP, true reset,
 * core registers over the surviving page table), plus: interrupts masked across
 * the reset and re-enabled after (Linux v3d_irq_reset), a TLB flush, and every
 * handler-shared word cleared (a stale staged chunk must not survive). */
int v3da_hw_reset(v3da_hw_t *hw)
{
	int rc;

	hw->core0[CTL_INT_MSK_SET / 4u] = ~0u;
	hw->hub[HUB_INT_MSK_SET / 4u] = ~0u;
	idle_axi(hw);
	rc = v3da_hw_reset_power(hw);
	apply_core_regs(hw);
	irq_mask_all(hw);
	__atomic_store_n(&hw->ovf_stage_va, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->ovf_stage_size, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->ovf_consumed, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->ovf_missed, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->ev_core, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&hw->ev_hub, 0u, __ATOMIC_RELEASE);
	hw->pt_gen++;
	v3da_hw_mmu_flush(hw);
	if (hw->irq_on != 0) {
		hw->core0[CTL_INT_MSK_CLR / 4u] = CORE_IRQS;
		hw->hub[HUB_INT_MSK_CLR / 4u] = HUB_IRQS;
	}
	return rc;
}


void v3da_hw_shutdown(v3da_hw_t *hw)
{
	v3da_hw_irq_disable(hw);
	hw->core0[CTL_INT_CLR / 4u] = ~0u;
	hw->hub[HUB_INT_CLR / 4u] = ~0u;
}


/* Raise core FLDONE and hub TFUC through INT_SET with no job on either queue,
 * and report where they surfaced. Poll mode: the raw status must show them, and
 * the poll path feeds the event thread. IRQ mode: the handler must run and the
 * event thread must see both. [inferred: INT_SET latches on V3D 4.2] */
int v3da_hw_selftest(v3da_hw_t *hw, v3da_irq_selftest_resp_t *out)
{
	uint32_t cnt0, fl0, tf0, core, hub;

	memset(out, 0, sizeof(*out));

	(void)mutexLock(srv.lock);
	if ((srv.q[V3DA_Q_BIN].active != NULL) || (srv.q[V3DA_Q_TFU].active != NULL)) {
		(void)mutexUnlock(srv.lock);
		return -EBUSY;
	}
	out->mode = (uint32_t)hw->irq_on;
	cnt0 = __atomic_load_n(&hw->irq_count, __ATOMIC_RELAXED);
	fl0 = srv.stray_fldone;
	tf0 = srv.stray_tfuc;

	hw->core0[CTL_INT_SET / 4u] = INT_FLDONE;
	hw->hub[HUB_INT_SET / 4u] = HUB_INT_TFUC;
	__asm__ volatile("dsb sy" ::: "memory");
	core = hw->core0[CTL_INT_STS / 4u];
	hub = hw->hub[HUB_INT_STS / 4u];
	out->core_sts_seen = ((core & INT_FLDONE) != 0u) ? 1u : 0u;
	out->hub_sts_seen = ((hub & HUB_INT_TFUC) != 0u) ? 1u : 0u;
	if (hw->irq_on == 0) {
		v3da_hw_poll_status(hw);
	}
	v3da_kick_event_thread();
	(void)mutexUnlock(srv.lock);

	usleep(30000);

	(void)mutexLock(srv.lock);
	out->handler_delta = __atomic_load_n(&hw->irq_count, __ATOMIC_RELAXED) - cnt0;
	out->core_events = srv.stray_fldone - fl0;
	out->hub_events = srv.stray_tfuc - tf0;
	out->storm = __atomic_load_n(&hw->storm, __ATOMIC_ACQUIRE);
	out->core_msk = hw->core0[CTL_INT_MSK_STS / 4u];
	out->hub_msk = hw->hub[HUB_INT_MSK_STS / 4u];
	(void)mutexUnlock(srv.lock);
	return 0;
}


/* ========================================================================= */
/* Bring-up                                                                   */
/* ========================================================================= */

int v3da_hw_init(v3da_hw_t *hw)
{
	uint32_t st, msk_core, msk_hub, i;
	void *p;
	int match, rc;

	/* 1. /dev/vcmbox must be there: the new lane never drives the FIFO directly.
	 * vcmbox_call resolves with a bounded ~5 s budget. */
	rc = vc_prop2(VC_PROP_GET_CLOCK_STATE, RPI_CLOCK_V3D, 0u, 1u, &st);
	if (rc != 0) {
		printf("V3DA srv power vcmbox=missing rc=%d - refusing to drive the mailbox FIFO directly\n", rc);
		return -ENODEV;
	}

	/* 2. power on */
	rc = v3d_power_on(hw);
	if (rc != 0) {
		return rc;
	}

	/* 3. MMIO + identity */
	hw->hub = map_dev((addr_t)V3D_HUB_BASE, V3D_MMIO_LEN);
	if (hw->hub == NULL) {
		return -ENOMEM;
	}
	hw->core0 = hw->hub + (V3D_CORE0_OFFS / 4u);
	hw->ident[0] = hw->core0[CTL_IDENT0 / 4u];
	hw->ident[1] = hw->core0[CTL_IDENT1 / 4u];
	hw->ident[2] = hw->core0[CTL_IDENT2 / 4u];
	hw->ident[3] = hw->hub[HUB_UIFCFG / 4u];
	hw->ident[4] = hw->hub[HUB_IDENT1 / 4u];
	hw->ident[5] = hw->hub[HUB_IDENT2 / 4u];
	hw->ident[6] = hw->hub[HUB_IDENT3 / 4u];
	match = (hw->ident[0] == V3D_EXPECT_CORE0_IDENT0) && (hw->ident[1] == V3D_EXPECT_CORE0_IDENT1) &&
		(hw->ident[2] == V3D_EXPECT_CORE0_IDENT2) && (hw->ident[3] == V3D_EXPECT_HUB_UIFCFG) &&
		(hw->ident[4] == V3D_EXPECT_HUB_IDENT1) && (hw->ident[5] == V3D_EXPECT_HUB_IDENT2) &&
		(hw->ident[6] == V3D_EXPECT_HUB_IDENT3);
	printf("V3DA srv ident core0=0x%08x/0x%08x/0x%08x uif=0x%08x hub=0x%08x/0x%08x/0x%08x match=%d\n",
		hw->ident[0], hw->ident[1], hw->ident[2], hw->ident[3], hw->ident[4], hw->ident[5],
		hw->ident[6], match);
	if (hw->ident[0] != V3D_EXPECT_CORE0_IDENT0) {
		printf("V3DA srv ident WRONG core0 ident - not serving a half-powered GPU\n");
		return -EIO;
	}

	/* Reset-time interrupt masks, recorded before we change them (design R5). */
	msk_core = hw->core0[CTL_INT_MSK_STS / 4u];
	msk_hub = hw->hub[HUB_INT_MSK_STS / 4u];

	(void)vc_prop2(VC_PROP_GET_CLOCK_RATE, RPI_CLOCK_V3D, 0u, 1u, &hw->clk_rate_hz);
	(void)vc_prop2(VC_PROP_GET_CLOCK_MEASURED, RPI_CLOCK_V3D, 0u, 1u, &hw->clk_meas_hz);

	/* 4. the one flat MMU page table + the illegal-access scratch page */
	hw->pt_entries = GPUVA_PT_PAGES * (uint32_t)(_PAGE_SIZE / 4u);
	p = mmap(NULL, GPUVA_PT_PAGES * _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (p == MAP_FAILED) {
		return -ENOMEM;
	}
	hw->pt = p;
	hw->pt_pa = (uintptr_t)va2pa(p);
	for (i = 0u; i < hw->pt_entries; i++) {
		hw->pt[i] = 0u;
	}
	p = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (p != MAP_FAILED) {
		memset(p, 0, _PAGE_SIZE);
		hw->scratch_pa = (uintptr_t)va2pa(p);
	}
	apply_core_regs(hw);
	irq_mask_all(hw);
	hw->next_gpuva = GPUVA_BASE;
	hw->pt_gen = 1u;
	v3da_hw_mmu_flush(hw);

	printf("V3DA srv mmu pt_pa=0x%08lx entries=%u scratch=0x%08lx reset_msk_core=0x%08x reset_msk_hub=0x%08x "
		"clk_rate_hz=%u clk_meas_hz=%u\n",
		(unsigned long)hw->pt_pa, hw->pt_entries, (unsigned long)hw->scratch_pa, msk_core, msk_hub,
		hw->clk_rate_hz, hw->clk_meas_hz);
	return 0;
}
