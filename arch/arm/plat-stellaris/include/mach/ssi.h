/************************************************************************************
 * arch/arm/src/lm3s/lm3s_ssi.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_PLAT_STELLARIS_SSI_H
#define __ARCH_PLAT_STELLARIS_SSI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#if STLR_NSSI > 0

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* SSI register offsets *************************************************************/

#define STLR_SSI_CR0_OFFSET       0x000 /* SSI Control 0 */
#define STLR_SSI_CR1_OFFSET       0x004 /* SSI Control 1 */
#define STLR_SSI_DR_OFFSET        0x008 /* SSI Data */
#define STLR_SSI_SR_OFFSET        0x00c /* SSI Status */
#define STLR_SSI_CPSR_OFFSET      0x010 /* SSI Clock Prescale */
#define STLR_SSI_IM_OFFSET        0x014 /* SSI Interrupt Mask */
#define STLR_SSI_RIS_OFFSET       0x018 /* SSI Raw Interrupt Status */
#define STLR_SSI_MIS_OFFSET       0x01c /* SSI Masked Interrupt Status */
#define STLR_SSI_ICR_OFFSET       0x020 /* SSI Interrupt Clear */
#define STLR_SSI_DMACTL_OFFSET    0x024 /* SSI DMA Control */
#define STLR_SSI_PERIPHID4_OFFSET 0xfd0 /* SSI Peripheral Identification 4 */
#define STLR_SSI_PERIPHID5_OFFSET 0xfd4 /* SSI Peripheral Identification 5 */
#define STLR_SSI_PERIPHID6_OFFSET 0xfd8 /* SSI Peripheral Identification 6 */
#define STLR_SSI_PERIPHID7_OFFSET 0xfdc /* SSI Peripheral Identification 7 */
#define STLR_SSI_PERIPHID0_OFFSET 0xfe0 /* SSI Peripheral Identification 0 */
#define STLR_SSI_PERIPHID1_OFFSET 0xfe4 /* SSI Peripheral Identification 1 */
#define STLR_SSI_PERIPHID2_OFFSET 0xfe8 /* SSI Peripheral Identification 2 */
#define STLR_SSI_PERIPHID3_OFFSET 0xfec /* SSI Peripheral Identification 3 */
#define STLR_SSI_PCELLID0_OFFSET  0xff0 /* SSI PrimeCell Identification 0 */
#define STLR_SSI_PCELLID1_OFFSET  0xff4 /* SSI PrimeCell Identification 1 */
#define STLR_SSI_PCELLID2_OFFSET  0xff8 /* SSI PrimeCell Identification 2 */
#define STLR_SSI_PCELLID3_OFFSET  0xffc /* SSI PrimeCell Identification 3 */

/* SSI register addresses ***********************************************************/

#define STLR_SSI0_CR0             (STLR_SSI0_BASE + STLR_SSI_CR0_OFFSET)
#define STLR_SSI0_CR1             (STLR_SSI0_BASE + STLR_SSI_CR1_OFFSET)
#define STLR_SSI0_DR              (STLR_SSI0_BASE + STLR_SSI_DR_OFFSET)
#define STLR_SSI0_SR              (STLR_SSI0_BASE + STLR_SSI_SR_OFFSET)
#define STLR_SSI0_CPSR            (STLR_SSI0_BASE + STLR_SSI_CPSR_OFFSET)
#define STLR_SSI0_IM              (STLR_SSI0_BASE + STLR_SSI_IM_OFFSET)
#define STLR_SSI0_RIS             (STLR_SSI0_BASE + STLR_SSI_RIS_OFFSET)
#define STLR_SSI0_MIS             (STLR_SSI0_BASE + STLR_SSI_MIS_OFFSET)
#define STLR_SSI0_ICR             (STLR_SSI0_BASE + STLR_SSI_ICR_OFFSET)
#define STLR_SSI0_DMACTL          (STLR_SSI0_BASE + STLR_SSI_DMACTL_OFFSET)
#define STLR_SSI0_PERIPHID4       (STLR_SSI0_BASE + STLR_SSI_PERIPHID4_OFFSET)
#define STLR_SSI0_PERIPHID5       (STLR_SSI0_BASE + STLR_SSI_PERIPHID5_OFFSET)
#define STLR_SSI0_PERIPHID6       (STLR_SSI0_BASE + STLR_SSI_PERIPHID6_OFFSET)
#define STLR_SSI0_PERIPHID7       (STLR_SSI0_BASE + STLR_SSI_PERIPHID7_OFFSET)
#define STLR_SSI0_PERIPHID0       (STLR_SSI0_BASE + STLR_SSI_PERIPHID0_OFFSET)
#define STLR_SSI0_PERIPHID1       (STLR_SSI0_BASE + STLR_SSI_PERIPHID1_OFFSET)
#define STLR_SSI0_PERIPHID2       (STLR_SSI0_BASE + STLR_SSI_PERIPHID2_OFFSET)
#define STLR_SSI0_PERIPHID3       (STLR_SSI0_BASE + STLR_SSI_PERIPHID3_OFFSET)
#define STLR_SSI0_PCELLID0        (STLR_SSI0_BASE + STLR_SSI_PCELLID0_OFFSET)
#define STLR_SSI0_PCELLID1        (STLR_SSI0_BASE + STLR_SSI_PCELLID1_OFFSET)
#define STLR_SSI0_PCELLID2        (STLR_SSI0_BASE + STLR_SSI_PCELLID2_OFFSET)
#define STLR_SSI0_PCELLID3        (STLR_SSI0_BASE + STLR_SSI_PCELLID3_OFFSET)

#if STLR_NSSI > 1
#define STLR_SSI1_CR0             (STLR_SSI1_BASE + STLR_SSI_CR0_OFFSET)
#define STLR_SSI1_CR1             (STLR_SSI1_BASE + STLR_SSI_CR1_OFFSET)
#define STLR_SSI1_DR              (STLR_SSI1_BASE + STLR_SSI_DR_OFFSET)
#define STLR_SSI1_SR              (STLR_SSI1_BASE + STLR_SSI_SR_OFFSET)
#define STLR_SSI1_CPSR            (STLR_SSI1_BASE + STLR_SSI_CPSR_OFFSET)
#define STLR_SSI1_IM              (STLR_SSI1_BASE + STLR_SSI_IM_OFFSET)
#define STLR_SSI1_RIS             (STLR_SSI1_BASE + STLR_SSI_RIS_OFFSET)
#define STLR_SSI1_MIS             (STLR_SSI1_BASE + STLR_SSI_MIS_OFFSET)
#define STLR_SSI1_ICR             (STLR_SSI1_BASE + STLR_SSI_ICR_OFFSET)
#define STLR_SSI1_DMACTL          (STLR_SSI1_BASE + STLR_SSI_DMACTL_OFFSET)
#define STLR_SSI1_PERIPHID4       (STLR_SSI1_BASE + STLR_SSI_PERIPHID4_OFFSET)
#define STLR_SSI1_PERIPHID5       (STLR_SSI1_BASE + STLR_SSI_PERIPHID5_OFFSET)
#define STLR_SSI1_PERIPHID6       (STLR_SSI1_BASE + STLR_SSI_PERIPHID6_OFFSET)
#define STLR_SSI1_PERIPHID7       (STLR_SSI1_BASE + STLR_SSI_PERIPHID7_OFFSET)
#define STLR_SSI1_PERIPHID0       (STLR_SSI1_BASE + STLR_SSI_PERIPHID0_OFFSET)
#define STLR_SSI1_PERIPHID1       (STLR_SSI1_BASE + STLR_SSI_PERIPHID1_OFFSET)
#define STLR_SSI1_PERIPHID2       (STLR_SSI1_BASE + STLR_SSI_PERIPHID2_OFFSET)
#define STLR_SSI1_PERIPHID3       (STLR_SSI1_BASE + STLR_SSI_PERIPHID3_OFFSET)
#define STLR_SSI1_PCELLID0        (STLR_SSI1_BASE + STLR_SSI_PCELLID0_OFFSET)
#define STLR_SSI1_PCELLID1        (STLR_SSI1_BASE + STLR_SSI_PCELLID1_OFFSET)
#define STLR_SSI1_PCELLID2        (STLR_SSI1_BASE + STLR_SSI_PCELLID2_OFFSET)
#define STLR_SSI1_PCELLID3        (STLR_SSI1_BASE + STLR_SSI_PCELLID3_OFFSET)

#define STLR_SSI_BASE(n)          (STLR_SSI0_BASE + (n)*0x01000)

#define STLR_SSI_CR0(n)           (STLR_SSI_BASE(n) + STLR_SSI_CR0_OFFSET)
#define STLR_SSI_CR1(n)           (STLR_SSI_BASE(n) + STLR_SSI_CR1_OFFSET)
#define STLR_SSI_DR(n)            (STLR_SSI_BASE(n) + STLR_SSI_DR_OFFSET)
#define STLR_SSI_SR(n)            (STLR_SSI_BASE(n) + STLR_SSI_SR_OFFSET)
#define STLR_SSI_CPSR(n)          (STLR_SSI_BASE(n) + STLR_SSI_CPSR_OFFSET)
#define STLR_SSI_IM(n)            (STLR_SSI_BASE(n) + STLR_SSI_IM_OFFSET)
#define STLR_SSI_RIS(n)           (STLR_SSI_BASE(n) + STLR_SSI_RIS_OFFSET)
#define STLR_SSI_MIS(n)           (STLR_SSI_BASE(n) + STLR_SSI_MIS_OFFSET)
#define STLR_SSI_ICR(n)           (STLR_SSI_BASE(n) + STLR_SSI_ICR_OFFSET)
#define STLR_SSI_DMACTL(n)        (STLR_SSI_BASE(n) + STLR_SSI_DMACTL_OFFSET)
#define STLR_SSI_PERIPHID4(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID4_OFFSET)
#define STLR_SSI_PERIPHID5(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID5_OFFSET)
#define STLR_SSI_PERIPHID6(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID6_OFFSET)
#define STLR_SSI_PERIPHID7(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID7_OFFSET)
#define STLR_SSI_PERIPHID0(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID0_OFFSET)
#define STLR_SSI_PERIPHID1(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID1_OFFSET)
#define STLR_SSI_PERIPHID2(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID2_OFFSET)
#define STLR_SSI_PERIPHID3(n)     (STLR_SSI_BASE(n) + STLR_SSI_PERIPHID3_OFFSET)
#define STLR_SSI_PCELLID0(n)      (STLR_SSI_BASE(n) + STLR_SSI_PCELLID0_OFFSET)
#define STLR_SSI_PCELLID1(n)      (STLR_SSI_BASE(n) + STLR_SSI_PCELLID1_OFFSET)
#define STLR_SSI_PCELLID2(n)      (STLR_SSI_BASE(n) + STLR_SSI_PCELLID2_OFFSET)
#define STLR_SSI_PCELLID3(n)      (STLR_SSI_BASE(n) + STLR_SSI_PCELLID3_OFFSET)
#endif /* STLR_NSSI > 1 */

/* SSI register bit defitiions ******************************************************/

/* SSI Control 0 (SSICR0), offset 0x000 */

#define SSI_CR0_DSS_SHIFT         0         /* Bits 3-0: SSI Data Size Select */
#define SSI_CR0_DSS_MASK          (0x0f << SSI_CR0_DSS_SHIFT)
#define   SSI_CR0_DSS(n)          ((n-1) << SSI_CR0_DSS_SHIFT) /* n={4,5,..16} */
#define SSI_CR0_FRF_SHIFT         4         /* Bits 5-4: SSI Frame Format Select */
#define SSI_CR0_FRF_MASK          (3 << SSI_CR0_FRF_SHIFT)
#define   SSI_CR0_FRF_SPI         (0 << SSI_CR0_FRF_SHIFT)     /* Freescale SPI format */
#define   SSI_CR0_FRF_SSFF        (1 << SSI_CR0_FRF_SHIFT)     /* TI synchronous serial fram format */
#define   SSI_CR0_FRF_UWIRE       (2 << SSI_CR0_FRF_SHIFT)     /* MICROWIRE frame format */
#define SSI_CR0_SPO               (1 << 6)  /* Bit 6:  SSI Serial Clock Polarity */
#define SSI_CR0_SPH               (1 << 7)  /* Bit 7:  SSI Serial Clock Phase */
#define SSI_CR0_SCR_SHIFT         8         /* Bits 15-8: SSI Serial Clock Rate */
#define SSI_CR0_SCR_MASK          (0xff << SSI_CR0_SCR_SHIFT)

/* SSI Control 1 (SSICR1), offset 0x004 */

#define SSI_CR1_LBM               (1 << 0)  /* Bit 0:  SSI Loopback Mode */
#define SSI_CR1_SSE               (1 << 1)  /* Bit 1:  SSI Synchronous Serial Port Enable */
#define SSI_CR1_MS                (1 << 2)  /* Bit 2:  SSI Master/Slave Select slave */
#define SSI_CR1_SOD               (1 << 3)  /* Bit 3:  SSI Slave Mode Output Disable */

/* SSI Data (SSIDR), offset 0x008 */

#define SSI_DR_MASK               0xffff    /* Bits 15-0: SSI data */

/* SSI Status (SSISR), offset 0x00c */

#define SSI_SR_TFE                (1 << 0)  /* Bit 0:  SSI Transmit FIFO Empty */
#define SSI_SR_TNF                (1 << 1)  /* Bit 1:  SSI Transmit FIFO Not Full */
#define SSI_SR_RNE                (1 << 2)  /* Bit 2:  SSI Receive FIFO Not Empty */
#define SSI_SR_RFF                (1 << 3)  /* Bit 3:  SSI Receive FIFO Full */
#define SSI_SR_BSY                (1 << 4)  /* Bit 4:  SSI Busy Bit */

/* SSI Clock Prescale (SSICPSR), offset 0x010 */

#define SSI_CPSR_DIV_MASK         0xff      /* Bits 7-0: SSI Clock Prescale Divisor */

/* SSI Interrupt Mask (SSIIM), offset 0x014 */

#define SSI_IM_ROR                (1 << 0)  /* Bit 0:  SSI Receive Overrun Interrupt Mask */
#define SSI_IM_RT                 (1 << 1)  /* Bit 1:  SSI Receive Time-Out Interrupt Mask */
#define SSI_IM_RX                 (1 << 2)  /* Bit 2:  SSI Receive FIFO Interrupt Mask */
#define SSI_IM_TX                 (1 << 3)  /* Bit 3:  SSI Transmit FIFO Interrupt Mask */
#define SSI_IM_DMARX              (1 << 4)  /* Bit 4:  SSI Receive DMA Interrupt Mask */
#define SSI_IM_DMATX              (1 << 5)  /* Bit 5:  SSI Transmit DMA Interrupt Mask */

/* SSI Raw Interrupt Status (SSIRIS), offset 0x018 */

#define SSI_RIS_ROR               (1 << 0)  /* Bit 0:  SSI Receive Overrun Raw Interrupt Status */
#define SSI_RIS_RT                (1 << 1)  /* Bit 1:  SSI Receive Time-Out Raw Interrupt Status */
#define SSI_RIS_RX                (1 << 2)  /* Bit 2:  SSI Receive FIFO Raw Interrupt Status */
#define SSI_RIS_TX                (1 << 3)  /* Bit 3:  SSI Transmit FIFO Raw Interrupt Status */
#define SSI_RIS_DMARX             (1 << 4)  /* Bit 4:  SSI Receive DMA Raw Interrupt Status */
#define SSI_RIS_DMATX             (1 << 5)  /* Bit 5:  SSI Transmit DMA Raw Interrupt Status */

/* SSI DMA Control (SSIDMACTL), offset 0x024 */

#define SSI_DMACTL_RXDMAE         (1 << 0) /* Bit 0: SSI Receive DMA Enable */
#define SSI_DMACTL_TXDMAE         (1 << 1) /* Bit 1: SSI Receive DMA Enable */

/* SSI Masked Interrupt Status (SSIMIS), offset 0x01c */

#define SSI_MIS_ROR               (1 << 0)  /* Bit 0:  SSI Receive Overrun Masked Interrupt Status */
#define SSI_MIS_RT                (1 << 1)  /* Bit 1:  SSI Receive Time-Out Masked Interrupt Status */
#define SSI_MIS_RX                (1 << 2)  /* Bit 2:  SSI Receive FIFO Masked Interrupt Status */
#define SSI_MIS_TX                (1 << 3)  /* Bit 3:  SSI Transmit FIFO Masked Interrupt Status */

/* SSI Interrupt Clear (SSIICR), offset 0x020 */

#define SSI_ICR_ROR               (1 << 0)  /* Bit 0: SSI Receive Overrun Interrupt Clear */
#define SSI_ICR_RT                (1 << 1)  /* Bit 1: SSI Receive Time-Out Interrupt Clear */
#define SSI_ICR_DMARX             (1 << 4)  /* Bit 4: SSI Receive DMA Interrupt Clear */
#define SSI_ICR_DMATX             (1 << 5)  /* Bit 5: SSI Transmit DMA Interrupt Clear */

/* SSI Peripheral Identification n (SSIPERIPHIDn), offset 0xfd0-0xfec */

#define SSI_PERIPHID_MASK         0xff      /* Bits 7-0: SSI Peripheral ID n */

/* SSI PrimeCell Identification n (SSIPCELLIDn), offset 0xff0-0xffc */

#define SSI_PCELLID_MASK          0xff      /* Bits 7-0: SSI Prime cell ID */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* STLR_NSSI > 0 */
#endif /* __ARCH_PLAT_STELLARIS_SSI_H */