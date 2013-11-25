/************************************************************************************
 * arch/arm/cpu/cortex-m4/tm4c/ccm.h
 *
 *   Copyright (C) 20013 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <Max.Nekludov@us.elster.com>
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

#ifndef __ARCH_ARM_SRC_TM4C12_CCM_H
#define __ARCH_ARM_SRC_TM4C12_CCM_H

/* CCM register offsets ***********************************************************/

#define CCM_SHA_MD5_BASE             0x44034000
#define CCM_AES_BASE                 0x44036000

#define CCM_SHA_IDIGEST_OFFSET       0x020 /* SHA idigest */
#define CCM_SHA_DIGEST_COUNT_OFFSET  0x040 /* SHA Digest Count */
#define CCM_SHA_MODE_OFFSET          0x044 /* SHA Mode */
#define CCM_SHA_LENGTH_OFFSET        0x048 /* SHA Length */
#define CCM_SHA_DATA_OFFSET          0x080 /* SHA Data */
#define CCM_SHA_SYSCONFIG_OFFSET     0x110 /* SHA System Configuration */
#define CCM_SHA_IRQSTATUS_OFFSET     0x118 /* SHA Interrupt Status */

#define CCM_AES_KEY1_0_OFFSET        0x038
#define CCM_AES_KEY1_1_OFFSET        0x03C
#define CCM_AES_KEY1_2_OFFSET        0x030
#define CCM_AES_KEY1_3_OFFSET        0x034
#define CCM_AES_KEY1_4_OFFSET        0x028
#define CCM_AES_KEY1_5_OFFSET        0x02C
#define CCM_AES_KEY1_6_OFFSET        0x020
#define CCM_AES_KEY1_7_OFFSET        0x024
#define CCM_AES_CTRL_OFFSET          0x050
#define CCM_AES_DATA_IN_OFFSET       0x060
#define CCM_AES_SYSCONFIG_OFFSET     0x084
#define CCM_AES_SYSSTATUS_OFFSET     0x088
#define CCM_AES_IRQSTATUS_OFFSET     0x08C

/* CCM registers ******************************************************************/

#define CCM_SHA_IDIGEST              (CCM_SHA_MD5_BASE + CCM_SHA_IDIGEST_OFFSET)
#define CCM_SHA_DIGEST_COUNT         (CCM_SHA_MD5_BASE + CCM_SHA_DIGEST_COUNT_OFFSET)
#define CCM_SHA_MODE                 (CCM_SHA_MD5_BASE + CCM_SHA_MODE_OFFSET)
#define CCM_SHA_LENGTH               (CCM_SHA_MD5_BASE + CCM_SHA_LENGTH_OFFSET)
#define CCM_SHA_DATA                 (CCM_SHA_MD5_BASE + CCM_SHA_DATA_OFFSET)
#define CCM_SHA_SYSCONFIG            (CCM_SHA_MD5_BASE + CCM_SHA_SYSCONFIG_OFFSET)
#define CCM_SHA_IRQSTATUS            (CCM_SHA_MD5_BASE + CCM_SHA_IRQSTATUS_OFFSET)

#define CCM_AES_KEY1_0               (CCM_AES_BASE + CCM_AES_KEY1_0_OFFSET)
#define CCM_AES_KEY1_1               (CCM_AES_BASE + CCM_AES_KEY1_1_OFFSET)
#define CCM_AES_KEY1_2               (CCM_AES_BASE + CCM_AES_KEY1_2_OFFSET)
#define CCM_AES_KEY1_3               (CCM_AES_BASE + CCM_AES_KEY1_3_OFFSET)
#define CCM_AES_KEY1_4               (CCM_AES_BASE + CCM_AES_KEY1_4_OFFSET)
#define CCM_AES_KEY1_5               (CCM_AES_BASE + CCM_AES_KEY1_5_OFFSET)
#define CCM_AES_KEY1_6               (CCM_AES_BASE + CCM_AES_KEY1_6_OFFSET)
#define CCM_AES_KEY1_7               (CCM_AES_BASE + CCM_AES_KEY1_7_OFFSET)
#define CCM_AES_CTRL                 (CCM_AES_BASE + CCM_AES_CTRL_OFFSET)
#define CCM_AES_DATA_IN              (CCM_AES_BASE + CCM_AES_DATA_IN_OFFSET)
#define CCM_AES_SYSCONFIG            (CCM_AES_BASE + CCM_AES_SYSCONFIG_OFFSET)
#define CCM_AES_SYSSTATUS            (CCM_AES_BASE + CCM_AES_SYSSTATUS_OFFSET)
#define CCM_AES_IRQSTATUS            (CCM_AES_BASE + CCM_AES_IRQSTATUS_OFFSET)

/* CCM register bit defitiions ****************************************************/

#define CCM_SHA_MODE_ALGO_SHIFT      0
#define CCM_SHA_MODE_ALGO_MASK       (0x07 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_MD5        (0x00 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_SHA386     (0x01 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_SHA1       (0x02 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_SHA512     (0x03 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_SHA224     (0x04 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_SHA256     (0x06 << CCM_SHA_MODE_ALGO_SHIFT)
#define CCM_SHA_MODE_ALGO_CONSTANT   (1 << 3)
#define CCM_SHA_MODE_CLOSE_HASH      (1 << 4)

#define CCM_SHA_IRQSTATUS_OUTPUT_READY (1 << 0)
#define CCM_SHA_IRQSTATUS_INPUT_READY  (1 << 1)

/* AES Control (AES_CTRL), offset 0x050 */
#define CCM_AES_CTRL_OUTPUT_READY        (1 << 0)
#define CCM_AES_CTRL_INPUT_READY         (1 << 1)
#define CCM_AES_CTRL_DIRECTION_SHIFT     2
#define CCM_AES_CTRL_DIRECTION_MASK      (1 << CCM_AES_CTRL_DIRECTION_SHIFT)
# define CCM_AES_CTRL_DIRECTION_ENCRYPT  (1 << CCM_AES_CTRL_DIRECTION_SHIFT)
# define CCM_AES_CTRL_DIRECTION_DECRYPT  (0 << CCM_AES_CTRL_DIRECTION_SHIFT)
#define CCM_AES_CTRL_KEY_SIZE_SHIFT      3
#define CCM_AES_CTRL_KEY_SIZE_MASK       (3 << CCM_AES_CTRL_KEY_SIZE_SHIFT)
# define CCM_AES_CTRL_KEY_SIZE_128       (1 << CCM_AES_CTRL_KEY_SIZE_SHIFT)
# define CCM_AES_CTRL_KEY_SIZE_192       (2 << CCM_AES_CTRL_KEY_SIZE_SHIFT)
# define CCM_AES_CTRL_KEY_SIZE_256       (3 << CCM_AES_CTRL_KEY_SIZE_SHIFT)
#define CCM_AES_CTRL_MODE_SHIFT          5
#define CCM_AES_CTRL_MODE_MASK           (1 << CCM_AES_CTRL_MODE_SHIFT)
#define CCM_AES_CTRL_MODE_ECB            (0 << CCM_AES_CTRL_MODE_SHIFT)
#define CCM_AES_CTRL_MODE_CBC            (1 << CCM_AES_CTRL_MODE_SHIFT)
#define CCM_AES_CTRL_CTR                 (1 << 6)
#define CCM_AES_CTRL_CTR_WIDTH_SHIFT     7
#define CCM_AES_CTRL_CTR_WIDTH_MASK      (3 << CCM_AES_CTRL_CTR_WIDTH_SHIFT)
#define CCM_AES_CTRL_SAVE_CONTEXT        (1 << 29)
#define CCM_AES_CTRL_SVCTXTRDY           (1 << 30)
#define CCM_AES_CTRL_CTXTRDY             (1 << 31)

/* AES System Configuration (AES_SYSCONFIG), offset 0x084 */
#define CCM_AES_SYSCONFIG_SOFTRESET      (1 << 1)

/* AES System Status (AES_SYSSTATUS), offset 0x088 */
#define CCM_AES_SYSSTATUS_RESETDONE      (1 << 0)

#ifndef __ASSEMBLY__

void ccm_sha1_simple(void *result, void *data, uint32_t size);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_TM4C12_CCM_H */