/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) display server rpi4-kms - vblank source + thread
 *
 * Sources, in order of preference (auto-picked at start, each given 100 ms to
 * prove itself; the winner is logged):
 *   irq     - the firmware raises the SMI interrupt at vblank (GIC SPI 112 =
 *             Phoenix IRQ 144, level-high). ACK as the Linux fkms handler does:
 *             SMICS = 0, then per-display bit 0 of SMIDSW0/1 acknowledged by
 *             writing 0xabcd0000 (vc4_firmware_kms.c:262-271, :1225-1273; facts
 *             only). Timestamped in the ISR.
 *   hvs     - the HVS5 6-bit frame counter of the channel scanning our display,
 *             polled (read-only MMIO; stamp = when the change was seen).
 *   fwvsync - the firmware's blocking SET_VSYNC tag. Holds /dev/vcmbox for up to a
 *             frame per call (thermal, the flip path wait behind it): a last resort.
 *   timer   - free-running at the mode's refresh.
 *
 * The ISR runs at EL1 in this process's address space (proc/userintr.c): MMIO
 * and volatile counters only - no libc, no locks, no calls (built with
 * -mno-outline-atomics, like rpi4-v3d-async's handler).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/threads.h>

#include "kms.h"


#define SMI_BASE        0xfe600000UL
#define SMICS           0x00u
#define SMIDSW0         0x14u
#define SMIDSW1         0x1cu
#define SMICS_INTS      ((1u << 9) | (1u << 10) | (1u << 11))   /* INTD|INTT|INTR */
#define SMI_NEW         0xabcd0000u
#define SMI_IRQ_PHOENIX (112u + 32u)   /* GIC SPI 112 + 32 (bcm-genet's convention) */

#define HVS_BASE          0xfe400000UL
#define HVS_SIZE          0x8000UL
#define SCALER_DISPLISTX(x) (0x20u + 4u * (x))
#define SCALER_DISPSTATX(x) (0x48u + 0x10u * (x))
#define SCALER_DISPSTAT1  0x58u
#define SCALER_DISPSTAT2  0x68u
#define HVS_DLIST_OFF     0x4000u
#define HVS_DLIST_WORDS   4096u
#define HVS_CHANNELS      3u
#define CTL0_END          (1u << 31)
#define CTL0_SIZE(w)      (((w) >> 24) & 0x3fu)
#define DISPSTAT_MODE(v)  (((v) >> 30) & 3u)

#define TAG_FB_SET_VSYNC  0x0004800eu

#define VB_RING 64u


static struct {
	volatile uint32_t *smi;
	volatile uint32_t *hvs;
	/* written by the ISR */
	volatile uint32_t count;
	volatile uint32_t spurious;
	volatile uint32_t head;
	volatile uint64_t ts[VB_RING];
	/* thread side */
	handle_t irq;
	int registered;
	uint32_t seen;
	uint32_t storm_base;
	uint64_t storm_t0;
	uint32_t hvs_last;
	uint64_t timer_next;
	uint64_t period_cnt;
	uint64_t last_vbl;
	/* refresh measurement */
	uint64_t meas_t0;
	uint32_t meas_n;
	int measured;
} vb;


const char *kms_vbl_name(int src)
{
	switch (src) {
		case KMS_VBL_IRQ: return "irq";
		case KMS_VBL_HVS: return "hvs";
		case KMS_VBL_FWVSYNC: return "fwvsync";
		case KMS_VBL_TIMER: return "timer";
		default: return "none";
	}
}


/* ========================================================================= */
/* SMI interrupt                                                              */
/* ========================================================================= */

static inline __attribute__((always_inline)) int smi_ack(uint64_t now)
{
	volatile uint32_t *smi = vb.smi;
	uint32_t cs = smi[SMICS / 4u], d0, d1;
	int ours = 0;

	if ((cs & SMICS_INTS) == 0u) {
		return -1;
	}
	smi[SMICS / 4u] = 0u;
	d0 = smi[SMIDSW0 / 4u];
	if ((d0 & 0xffff0000u) != SMI_NEW) {
		ours = 1;   /* old firmware: one event for all displays */
	}
	else {
		if ((d0 & 1u) != 0u) {
			smi[SMIDSW0 / 4u] = SMI_NEW;
			ours = 1;   /* display 0 = the only one Stage A drives */
		}
		d1 = smi[SMIDSW1 / 4u];
		if ((d1 & 1u) != 0u) {
			smi[SMIDSW1 / 4u] = SMI_NEW;
		}
	}
	if (ours != 0) {
		vb.ts[vb.head % VB_RING] = now;
		vb.head = vb.head + 1u;
	}
	return ours;
}


static int smi_isr(unsigned int n, void *arg)
{
	(void)n;
	(void)arg;
	if (smi_ack(kms_cnt()) < 0) {
		/* Line up without the firmware's bits: clear SMICS anyway (nothing else in
		 * Phoenix uses the SMI) and do not wake the thread. */
		vb.smi[SMICS / 4u] = 0u;
		vb.spurious = vb.spurious + 1u;
		return -1;
	}
	vb.count = vb.count + 1u;
	return 1;
}


static int irq_register(void)
{
	int rc;

	if (vb.registered) {
		return 0;
	}
	if (vb.smi == NULL) {
		return -ENODEV;
	}
	vb.smi[SMICS / 4u] = 0u;   /* as the Linux driver does before requesting the IRQ */
	rc = interrupt(SMI_IRQ_PHOENIX, smi_isr, NULL, srv.vbl_cond, &vb.irq);
	if (rc < 0) {
		return rc;
	}
	vb.registered = 1;
	vb.seen = vb.head;
	vb.storm_base = vb.count + vb.spurious;
	vb.storm_t0 = kms_cnt();
	return 0;
}


static void irq_unregister(void)
{
	if (vb.registered) {
		(void)resourceDestroy(vb.irq);
		vb.registered = 0;
	}
}


/* A level interrupt the ISR cannot silence would pin a CPU. A sane vblank is
 * <= 2 x 85 Hz; 2000 entries/s means something else holds the line. */
static int storm_check(void)
{
	uint64_t us;
	uint32_t d;

	if (!vb.registered) {
		return 0;
	}
	us = kms_cnt_us(kms_cnt() - vb.storm_t0);
	if (us < 2000000u) {
		return 0;
	}
	d = (vb.count + vb.spurious) - vb.storm_base;
	if (((uint64_t)d * 1000000ULL) / us > 2000u) {
		irq_unregister();
		KMS_LOG("vblank storm entries=%u in_us=%llu spurious=%u -> handler removed", d, (unsigned long long)us,
			vb.spurious);
		return 1;
	}
	vb.storm_base = vb.count + vb.spurious;
	vb.storm_t0 = kms_cnt();
	return 0;
}


/* ========================================================================= */
/* HVS frame counter                                                          */
/* ========================================================================= */

static inline uint32_t hvs_rd(uint32_t off)
{
	return vb.hvs[off / 4u];
}


static uint32_t hvs_frcnt(int ch)
{
	switch (ch) {
		case 0: return (hvs_rd(SCALER_DISPSTAT1) >> 20) & 0x3fu;
		case 1: return (hvs_rd(SCALER_DISPSTAT1) >> 14) & 0x3fu;
		default: return (hvs_rd(SCALER_DISPSTAT2) >> 14) & 0x3fu;
	}
}


/* The channel whose display list points into the firmware fb, else the first
 * running one (the walk of vc4_hvs_debugfs_dlist, low-30-bit address compare). */
static int hvs_channel(void)
{
	uint32_t lo = (uint32_t)(srv.fb_pa & 0x3fffffffu), hi = lo + srv.fb_pitch * (srv.fb_virt_h ? srv.fb_virt_h : srv.fb_h);
	int ch;

	if (vb.hvs == NULL) {
		return -1;
	}
	for (ch = 0; (srv.fb_pa != 0u) && (ch < (int)HVS_CHANNELS); ch++) {
		uint32_t j = hvs_rd(SCALER_DISPLISTX(ch)) % HVS_DLIST_WORDS, next = j, steps;
		uint32_t mode = DISPSTAT_MODE(hvs_rd(SCALER_DISPSTATX(ch)));

		if ((mode != 1u) && (mode != 2u)) {
			continue;
		}
		for (steps = 0; steps < HVS_DLIST_WORDS; steps++, j = (j + 1u) % HVS_DLIST_WORDS) {
			uint32_t w = vb.hvs[(HVS_DLIST_OFF / 4u) + j];
			if (j == next) {
				if (((w & CTL0_END) != 0u) || (CTL0_SIZE(w) == 0u)) {
					break;
				}
				next = (j + CTL0_SIZE(w)) % HVS_DLIST_WORDS;
				continue;
			}
			if (((w & 0x3fffffffu) >= lo) && ((w & 0x3fffffffu) < hi)) {
				return ch;
			}
		}
	}
	for (ch = 0; ch < (int)HVS_CHANNELS; ch++) {
		if (DISPSTAT_MODE(hvs_rd(SCALER_DISPSTATX(ch))) == 2u) {
			return ch;
		}
	}
	return -1;
}


/* ========================================================================= */
/* Waiting                                                                    */
/* ========================================================================= */

/* Has commit() deferred a commit since the caller read srv.gate_kick? */
static int kicked(uint32_t kick)
{
	return __atomic_load_n(&srv.gate_kick, __ATOMIC_ACQUIRE) != kick;
}


/* Wait up to max_us for the next vblank. 1 = vblank(s) happened (*n of them,
 * *stamp = cntvct of the last), 0 = timeout, or commit() bumped srv.gate_kick
 * past `kick` (irq and hvs sources; fwvsync blocks in the firmware and timer
 * sleeps, so those two notice a kick only when they next return). A vblank that
 * is already in wins over a kick. */
static int vbl_wait(uint32_t max_us, uint32_t kick, uint64_t *stamp, uint32_t *n)
{
	uint64_t t0 = kms_cnt(), lim = kms_us_cnt(max_us);

	switch (srv.vbl_src) {
		case KMS_VBL_IRQ:
			(void)mutexLock(srv.vbl_lock);
			while ((vb.head == vb.seen) && !kicked(kick) && ((kms_cnt() - t0) < lim)) {
				/* Neither the ISR nor commit() takes the mutex, so a wake-up can land
				 * between the check and the wait: a <= 2 ms timeout bounds that. */
				uint64_t left = kms_cnt_us(lim - (kms_cnt() - t0));
				(void)condWait(srv.vbl_cond, srv.vbl_lock, (left > 2000u) ? 2000u : ((left == 0u) ? 1u : left));
			}
			(void)mutexUnlock(srv.vbl_lock);
			if (vb.head == vb.seen) {
				return 0;
			}
			*n = vb.head - vb.seen;
			vb.seen = vb.head;
			*stamp = vb.ts[(vb.seen - 1u) % VB_RING];
			return 1;

		case KMS_VBL_HVS: {
			uint32_t f;
			for (;;) {
				f = hvs_frcnt(srv.hvs_ch);
				if (f != vb.hvs_last) {
					*n = (f - vb.hvs_last) & 0x3fu;
					vb.hvs_last = f;
					*stamp = kms_cnt();
					return 1;
				}
				if (((kms_cnt() - t0) >= lim) || kicked(kick)) {
					return 0;
				}
				usleep(250);
			}
		}

		case KMS_VBL_FWVSYNC: {
			uint32_t z = 0u;
			(void)kms_fw_prop(TAG_FB_SET_VSYNC, 1u, &z, 1u, NULL);
			*stamp = kms_cnt();
			*n = 1u;
			return 1;
		}

		default: {
			uint64_t now = kms_cnt();
			if (vb.timer_next == 0u) {
				vb.timer_next = now + vb.period_cnt;
			}
			if (now < vb.timer_next) {
				uint64_t wait = kms_cnt_us(vb.timer_next - now);
				if (wait > max_us) {
					usleep(max_us);
					return 0;
				}
				usleep((useconds_t)wait);
			}
			*n = 1u;
			vb.timer_next += vb.period_cnt;
			if (kms_cnt() > vb.timer_next + vb.period_cnt) {
				vb.timer_next = kms_cnt() + vb.period_cnt;   /* fell behind: resync, no burst */
			}
			*stamp = kms_cnt();
			return 1;
		}
	}
}


static int probe_src(int src)
{
	uint64_t stamp;
	uint32_t n;
	int got;

	srv.vbl_src = src;
	switch (src) {
		case KMS_VBL_IRQ:
			if (irq_register() != 0) {
				return 0;
			}
			got = vbl_wait(100000u, srv.gate_kick, &stamp, &n);   /* no client yet: no kick */
			if (!got) {
				irq_unregister();
			}
			return got;
		case KMS_VBL_HVS:
			srv.hvs_ch = hvs_channel();
			if (srv.hvs_ch < 0) {
				return 0;
			}
			vb.hvs_last = hvs_frcnt(srv.hvs_ch);
			return vbl_wait(100000u, srv.gate_kick, &stamp, &n);
		case KMS_VBL_FWVSYNC: {
			uint64_t t0 = kms_cnt();
			uint32_t z = 0u;
			/* A tag that answers at once did not wait for anything. */
			return (kms_fw_prop(TAG_FB_SET_VSYNC, 1u, &z, 1u, NULL) == 0) && (kms_cnt_us(kms_cnt() - t0) > 2000u);
		}
		default:
			return 1;
	}
}


int kms_vblank_init(void)
{
	static const int order[] = { KMS_VBL_IRQ, KMS_VBL_HVS, KMS_VBL_FWVSYNC, KMS_VBL_TIMER };
	void *va;
	uint32_t i, mhz = srv.crtc[0].refresh_mhz ? srv.crtc[0].refresh_mhz : 60000u;
	uint64_t t0 = kms_cnt();

	va = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1,
		(off_t)SMI_BASE);
	vb.smi = (va == MAP_FAILED) ? NULL : va;
	va = mmap(NULL, HVS_SIZE, PROT_READ, MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)HVS_BASE);
	vb.hvs = (va == MAP_FAILED) ? NULL : va;
	vb.period_cnt = ((uint64_t)kms_cnt_hz * 1000u) / mhz;
	srv.hvs_ch = -1;

	if ((srv.want_vbl != KMS_VBL_NONE) && probe_src(srv.want_vbl)) {
		KMS_LOG("vblank src=%s (forced) probe_ms=%llu hvs_ch=%d", kms_vbl_name(srv.vbl_src),
			(unsigned long long)(kms_cnt_us(kms_cnt() - t0) / 1000u), srv.hvs_ch);
		return 0;
	}
	for (i = 0u; i < sizeof(order) / sizeof(order[0]); i++) {
		if (order[i] == srv.want_vbl) {
			continue;
		}
		if (probe_src(order[i])) {
			break;
		}
	}
	if (srv.hvs_ch < 0) {
		srv.hvs_ch = hvs_channel();   /* for the log line and a later fallback */
	}
	KMS_LOG("vblank src=%s probe_ms=%llu smi_mapped=%d hvs_mapped=%d hvs_ch=%d irq=%u", kms_vbl_name(srv.vbl_src),
		(unsigned long long)(kms_cnt_us(kms_cnt() - t0) / 1000u), vb.smi != NULL, vb.hvs != NULL, srv.hvs_ch,
		SMI_IRQ_PHOENIX);
	return 0;
}


void kms_vblank_fini(void)
{
	irq_unregister();
}


/* ========================================================================= */
/* The thread                                                                 */
/* ========================================================================= */

static void fallback(const char *why)
{
	int old = srv.vbl_src;

	irq_unregister();
	if ((old != KMS_VBL_HVS) && (vb.hvs != NULL) && (srv.hvs_ch >= 0)) {
		srv.vbl_src = KMS_VBL_HVS;
		vb.hvs_last = hvs_frcnt(srv.hvs_ch);
	}
	else {
		srv.vbl_src = KMS_VBL_TIMER;
		vb.timer_next = 0u;
	}
	KMS_LOG("vblank fallback from=%s to=%s why=%s", kms_vbl_name(old), kms_vbl_name(srv.vbl_src), why);
}


static void measure(kms_crtc_state_t *c, uint64_t stamp, uint32_t n)
{
	uint64_t us;
	uint32_t mhz;

	if (vb.measured) {
		return;
	}
	if (vb.meas_t0 == 0u) {
		vb.meas_t0 = stamp;
		return;
	}
	vb.meas_n += n;
	if (vb.meas_n < 120u) {
		return;
	}
	us = kms_cnt_us(stamp - vb.meas_t0);
	mhz = (us != 0u) ? (uint32_t)(((uint64_t)vb.meas_n * 1000000000ULL) / us) : 0u;
	vb.measured = 1;
	KMS_LOG("vblank measured src=%s n=%u us=%llu hz=%u.%03u mode_mhz=%u", kms_vbl_name(srv.vbl_src), vb.meas_n,
		(unsigned long long)us, mhz / 1000u, mhz % 1000u, c->refresh_mhz);
	if ((mhz != 0u) && (srv.vbl_src != KMS_VBL_TIMER)) {
		kms_mode_set_refresh(c, mhz);   /* synthesized modes only */
	}
}


void kms_vblank_thread(void *arg)
{
	static kms_parked_t answers[KMS_MAX_PARKED];
	kms_crtc_state_t *c = &srv.crtc[0];
	uint64_t stamp = 0u, last_tick = kms_cnt();
	uint32_t n = 0u, nans, max_us, kick;
	int got, quit, gate;

	(void)arg;
	vb.last_vbl = kms_cnt();
	for (;;) {
		/* kick before pend: commit() stores pend, then bumps gate_kick (release), so
		 * either this read of pend sees the commit or vbl_wait sees the kick. */
		kick = __atomic_load_n(&srv.gate_kick, __ATOMIC_ACQUIRE);
		gate = (c->pend == KMS_PEND_FENCE);   /* racy read: at worst one extra/late poll */
		max_us = gate ? srv.gate_us : 20000u;
		got = vbl_wait(max_us, kick, &stamp, &n);

		(void)mutexLock(srv.lock);
		nans = 0u;
		if (got > 0) {
			vb.last_vbl = stamp;
			srv.st.vblanks += n;
			measure(c, stamp, n);
			kms_on_vblank(c, stamp, n, answers, &nans);
		}
		else if (c->pend == KMS_PEND_FENCE) {
			/* Between vblanks: the gate_us poll, or the first look after a kick.
			 * Its fence may have signalled mid-frame; arming now makes the next
			 * latch when there is time before it (kms_try_apply's guard rule). */
			if (!gate && kicked(kick)) {
				srv.fst.kicks++;
			}
			if (kms_try_apply(c)) {
				srv.fst.applied_gate++;
			}
		}
		kms_expire_parked(kms_cnt(), answers, &nans);
		srv.st.vblank_src = (uint32_t)srv.vbl_src;
		srv.st.irq_count = vb.count;
		srv.st.irq_spurious = vb.spurious;
		quit = srv.quit;
		(void)mutexUnlock(srv.lock);

		kms_answer(answers, nans);
		if (__atomic_load_n(&srv.nreap, __ATOMIC_RELAXED) != 0u) {
			kms_reap();   /* G7: a completed flip dropped an import's last reference (rare) */
		}
		if (quit) {
			break;
		}
		if ((kms_cnt() - last_tick) >= 2u * kms_cnt_hz) {
			last_tick = kms_cnt();
			if ((srv.vbl_src == KMS_VBL_IRQ) && storm_check()) {
				fallback("storm");
			}
		}
		/* A source that stops delivering (e.g. the firmware stops raising SMI) must
		 * not freeze every flip: 250 ms of silence = fall back. */
		if ((srv.vbl_src == KMS_VBL_IRQ || srv.vbl_src == KMS_VBL_HVS) &&
				(kms_cnt_us(kms_cnt() - vb.last_vbl) > 250000u)) {
			fallback("silent_250ms");
			vb.last_vbl = kms_cnt();
		}
	}
	/* A Phoenix thread entry must never return: its fresh stack has no caller, so a
	 * return jumps to the kernel's fill pattern (the m2-kms-a exit crash: PC
	 * alignment fault at pc=lr=0x1e1e1e1e1e1e1e1e right after "srv exit"). */
	endthread();
}
