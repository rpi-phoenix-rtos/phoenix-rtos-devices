/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - register map
 *
 * Offsets and bits the old lane already uses are copied from the rpi4-v3d daemon
 * (gpu/rpi4-v3d/v3d_gpu.c, itself copied from mesa/v3d_phoenix_winsys.c). The
 * interrupt-mask, INT_SET, identity, CTnRA and MMU-fault registers the old lane
 * never needed are hardware facts taken from Linux drivers/gpu/drm/v3d/v3d_regs.h
 * (offsets only; no code). HUB-relative offsets index `hub`, CORE0-relative ones
 * index `core0` (= hub + 0x4000).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _V3DA_REGS_H_
#define _V3DA_REGS_H_

/* MMIO window (ARM low-peripheral view of 0x7ec00000) */
#define V3D_HUB_BASE        0xfec00000u
#define V3D_MMIO_LEN        0x10000u
#define V3D_CORE0_OFFS      0x4000u

/* GIC: DT `interrupts = <GIC_SPI 74 IRQ_TYPE_LEVEL_HIGH>` (bcm2711.dtsi:613); Phoenix
 * numbers SPIs as GIC IDs (SPI + 32). One line for hub AND core on BCM2711. */
#define V3D_IRQ             106u

/* --- HUB --------------------------------------------------------------- */
#define HUB_AXICFG          0x0000u
#define HUB_AXICFG_MAX_LEN  0x0000000fu
#define HUB_UIFCFG          0x0004u
#define HUB_IDENT0          0x0008u
#define HUB_IDENT1          0x000cu
#define HUB_IDENT2          0x0010u
#define HUB_IDENT3          0x0014u
#define HUB_INT_STS         0x0050u   /* raw status, independent of the mask */
#define HUB_INT_SET         0x0054u
#define HUB_INT_CLR         0x0058u
#define HUB_INT_MSK_STS     0x005cu
#define HUB_INT_MSK_SET     0x0060u   /* 1 = masked */
#define HUB_INT_MSK_CLR     0x0064u
#define HUB_INT_TFUF        (1u << 0)
#define HUB_INT_TFUC        (1u << 1)
#define HUB_INT_MSO         (1u << 2)
#define HUB_INT_MMU_CAP     (1u << 3)
#define HUB_INT_MMU_PTI     (1u << 4)
#define HUB_INT_MMU_WRV     (1u << 5)
#define HUB_INT_MMU_ANY     (HUB_INT_MMU_CAP | HUB_INT_MMU_PTI | HUB_INT_MMU_WRV)
/* Linux parity (v3d_irq.c:29-33): TFUF is not enabled as a source. */
#define HUB_IRQS            (HUB_INT_MMU_WRV | HUB_INT_MMU_PTI | HUB_INT_MMU_CAP | HUB_INT_TFUC)

/* TFU (V3D 4.2) */
#define TFU_CS              0x0400u
#define TFU_CS_BUSY         (1u << 0)
#define TFU_ICFG            0x0408u
#define TFU_ICFG_IOC        (1u << 0)
#define TFU_IIA             0x040cu
#define TFU_ICA             0x0410u
#define TFU_IIS             0x0414u
#define TFU_IUA             0x0418u
#define TFU_IOA             0x041cu
#define TFU_IOS             0x0420u
#define TFU_COEF0           0x0424u
#define TFU_COEF0_USECOEF   (1u << 31)
#define TFU_COEF1           0x0428u
#define TFU_COEF2           0x042cu
#define TFU_COEF3           0x0430u

/* MMU */
#define MMUC_CONTROL        0x1000u
#define MMUC_ENABLE         (1u << 0)
#define MMUC_FLUSH          (1u << 1)
#define MMUC_FLUSHING       (1u << 2)
#define MMU_CTL             0x1200u
#define MMU_CTL_ENABLE          (1u << 0)
#define MMU_CTL_TLB_CLEAR       (1u << 2)
#define MMU_CTL_TLB_CLEARING    (1u << 7)
#define MMU_CTL_WRITEVIO_INT    (1u << 10)
#define MMU_CTL_WRITEVIO_ABORT  (1u << 11)
#define MMU_CTL_PTI_ENABLE      (1u << 16)
#define MMU_CTL_PTI_INT         (1u << 18)
#define MMU_CTL_PTI_ABORT       (1u << 19)
#define MMU_CTL_CAPEXC_INT      (1u << 25)
#define MMU_CTL_CAPEXC_ABORT    (1u << 26)
#define MMU_PT_PA_BASE      0x1204u
#define MMU_VIO_ID          0x122cu
#define MMU_ILLEGAL_ADDR    0x1230u
#define MMU_ILLEGAL_ENABLE  (1u << 31)
#define MMU_VIO_ADDR        0x1234u
#define MMU_DEBUG_INFO      0x1238u
#define PTE_V               (1u << 28)
#define PTE_W               (1u << 29)
#define V3D_PAGE_SHIFT      12u

/* --- CORE0 ------------------------------------------------------------- */
#define CTL_IDENT0          0x0000u   /* "V3D" + version: 0x04443356 on this board */
#define CTL_IDENT1          0x0004u
#define CTL_IDENT2          0x0008u
#define CTL_MISCCFG         0x0018u
#define MISCCFG_OVRTMUOUT   (1u << 0)
#define MISCCFG_QRMAXCNT_SHIFT 1u
#define V3D_QRMAXCNT        2u
#define CTL_L2CACTL         0x0020u
#define L2CACTL_L2CENA      (1u << 0)
#define L2CACTL_L2CCLR      (1u << 2)
#define CTL_SLCACTL         0x0024u
#define SLCACTL_INVAL_ALL   0x0f0f0f0fu
#define CTL_L2TCACTL        0x0030u
#define L2TCACTL_L2TFLS     (1u << 0)
#define L2TCACTL_FLM_CLEAN  (2u << 1)
#define L2TCACTL_TMUWCF     (1u << 8)
#define CTL_L2TFLSTA        0x0034u
#define CTL_L2TFLEND        0x0038u
#define CTL_INT_STS         0x0050u   /* raw status */
#define CTL_INT_SET         0x0054u
#define CTL_INT_CLR         0x0058u
#define CTL_INT_MSK_STS     0x005cu
#define CTL_INT_MSK_SET     0x0060u   /* 1 = masked */
#define CTL_INT_MSK_CLR     0x0064u
#define INT_FRDONE          (1u << 0)
#define INT_FLDONE          (1u << 1)
#define INT_OUTOMEM         (1u << 2)
#define INT_SPILLUSE        (1u << 3)
#define INT_TRFB            (1u << 4)
#define INT_GMPV            (1u << 5)
#define INT_PCTR            (1u << 6)
#define INT_CSDDONE         (1u << 7)   /* V3D < 7.1 */
#define INT_QPU_MASK        (0xfffu << 16)
/* Linux parity (v3d_irq.c:23-27). */
#define CORE_IRQS           (INT_OUTOMEM | INT_FLDONE | INT_FRDONE | INT_CSDDONE | INT_GMPV)

/* Control-list executors */
#define CLE_CT0CS           0x0100u
#define CLE_CT1CS           0x0104u
#define CLE_CT0EA           0x0108u
#define CLE_CT1EA           0x010cu
#define CLE_CT0CA           0x0110u
#define CLE_CT1CA           0x0114u
#define CLE_CT0RA           0x0118u
#define CLE_CT1RA           0x011cu
#define CLE_CT0QTS          0x015cu
#define CT0QTS_ENABLE       (1u << 1)
#define CLE_CT0QBA          0x0160u
#define CLE_CT1QBA          0x0164u
#define CLE_CT0QEA          0x0168u
#define CLE_CT1QEA          0x016cu
#define CLE_CT0QMA          0x0170u
#define CLE_CT0QMS          0x0174u

/* Binner (PTB) */
#define PTB_BPCA            0x0300u
#define PTB_BPCS            0x0304u
#define PTB_BPOA            0x0308u
#define PTB_BPOS            0x030cu

/* GMP */
#define GMP_STATUS          0x0800u
#define GMP_STATUS_CFG_BUSY (1u << 3)
#define GMP_STATUS_RD_WR_CNT 0x7f7f0000u
#define GMP_CFG             0x0804u
#define GMP_CFG_STOP_REQ    (1u << 1)
#define GMP_VIO_ADDR        0x0808u

/* CSD (V3D 4.2) */
#define CSD_STATUS          0x0900u
#define CSD_STATUS_HAVE_CURRENT (1u << 1)
#define CSD_QUEUED_CFG0     0x0904u
#define CSD_CURRENT_CFG4    0x0930u

/* Front-end debug (raw dumps only) */
#define ERR_FDBGO           0x0f04u
#define ERR_FDBGB           0x0f08u
#define ERR_FDBGR           0x0f0cu
#define ERR_FDBGS           0x0f10u
#define ERR_STAT            0x0f20u

/* Expected identity (what the old client reports for GET_PARAM,
 * gpu/rpi4-v3d/libv3d-client.c:233-239). */
#define V3D_EXPECT_CORE0_IDENT0  0x04443356u
#define V3D_EXPECT_CORE0_IDENT1  0x81001422u
#define V3D_EXPECT_CORE0_IDENT2  0x40078121u
#define V3D_EXPECT_HUB_UIFCFG    0x00000045u
#define V3D_EXPECT_HUB_IDENT1    0x000e1124u
#define V3D_EXPECT_HUB_IDENT2    0x00000100u
#define V3D_EXPECT_HUB_IDENT3    0x00000e00u

#endif /* _V3DA_REGS_H_ */
