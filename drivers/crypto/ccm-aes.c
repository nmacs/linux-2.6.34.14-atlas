/*
 * CCM AES driver
 *
 * Based on padlock-aes.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <mach/sram.h>
#include <mach/hardware.h>
#include "ccm.h"

struct aes_ctx {
	u8 key[AES_MAX_KEY_SIZE];
	int keylen;
};

static spinlock_t lock;

static int aes_set_key(struct crypto_tfm *tfm, const u8 *key,
                       unsigned int keylen)
{
	struct aes_ctx *ctx = crypto_tfm_ctx(tfm);
	
	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256) {
		return -EINVAL;
	}
	
	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	
	return 0;
}

static void __sram ccm_reset(void)
{
	// Reset AES module
	putreg32(CCM_AES_SYSCONFIG_SOFTRESET, CCM_AES_SYSCONFIG);
	while ((getreg32(CCM_AES_SYSSTATUS) & CCM_AES_SYSSTATUS_RESETDONE) == 0) {}
}

static void __sram ccm_apply_key(struct aes_ctx *ctx)
{
	u8* key = ctx->key;
	int keylen = ctx->keylen;
	uint32_t ctrl = getreg32(CCM_AES_CTRL);
	
	if (keylen == AES_KEYSIZE_128) {
		ctrl |= CCM_AES_CTRL_KEY_SIZE_128;
	}
	else if (keylen == AES_KEYSIZE_192) {
		ctrl |= CCM_AES_CTRL_KEY_SIZE_192;
	}
	else if (keylen == AES_KEYSIZE_256) {
		ctrl |= CCM_AES_CTRL_KEY_SIZE_256;
	}
	else {
		printk(KERN_ERR PFX "Invalid key length");
		BUG();
	}

	putreg32(ctrl, CCM_AES_CTRL);

	putreg32(((uint32_t*)key)[0], CCM_AES_KEY1_0);
	putreg32(((uint32_t*)key)[1], CCM_AES_KEY1_1);
	putreg32(((uint32_t*)key)[2], CCM_AES_KEY1_2);
	putreg32(((uint32_t*)key)[3], CCM_AES_KEY1_3);
	if (keylen > AES_KEYSIZE_128) {
		putreg32(((uint32_t*)key)[4], CCM_AES_KEY1_4);
		putreg32(((uint32_t*)key)[5], CCM_AES_KEY1_5);
	}
	if (keylen > AES_KEYSIZE_192) {
		putreg32(((uint32_t*)key)[6], CCM_AES_KEY1_6);
		putreg32(((uint32_t*)key)[7], CCM_AES_KEY1_7);
	}
}

static void __sram ccm_crypt(struct aes_ctx *ctx, u8 *out, const u8 *in, int encrypt)
{
	uint32_t ctrl;
	unsigned long iflags;
	
	spin_lock_irqsave(&lock, iflags);
	
	ccm_reset();
	ccm_apply_key(ctx);
	
	ctrl = getreg32(CCM_AES_CTRL);
	if (encrypt) {
		ctrl |= CCM_AES_CTRL_DIRECTION_ENCRYPT;
	}
	else {
		ctrl &= ~CCM_AES_CTRL_DIRECTION_ENCRYPT;
	}
	
	putreg32(ctrl, CCM_AES_CTRL);
	
	putreg32(((uint32_t*)in)[0], CCM_AES_DATA_IN + 0);
	putreg32(((uint32_t*)in)[1], CCM_AES_DATA_IN + 4);
	putreg32(((uint32_t*)in)[2], CCM_AES_DATA_IN + 8);
	putreg32(((uint32_t*)in)[3], CCM_AES_DATA_IN + 12);
	
	while ((getreg32(CCM_AES_CTRL) & CCM_AES_CTRL_INPUT_READY) == 0) {}
	
	putreg32(((uint32_t*)in)[0], CCM_AES_DATA_IN + 0);
	putreg32(((uint32_t*)in)[1], CCM_AES_DATA_IN + 4);
	putreg32(((uint32_t*)in)[2], CCM_AES_DATA_IN + 8);
	putreg32(((uint32_t*)in)[3], CCM_AES_DATA_IN + 12);
	
	while ((getreg32(CCM_AES_CTRL) & CCM_AES_CTRL_OUTPUT_READY) == 0) {}
	
	((uint32_t*)out)[0] = getreg32(CCM_AES_DATA_IN + 0);
	((uint32_t*)out)[1] = getreg32(CCM_AES_DATA_IN + 4);
	((uint32_t*)out)[2] = getreg32(CCM_AES_DATA_IN + 8);
	((uint32_t*)out)[3] = getreg32(CCM_AES_DATA_IN + 12);
	
	spin_unlock_irqrestore(&lock, iflags);
}

static void aes_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct aes_ctx *ctx = crypto_tfm_ctx(tfm);
	ccm_crypt(ctx, out, in, 1);
}

static void aes_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct aes_ctx *ctx = crypto_tfm_ctx(tfm);
	ccm_crypt(ctx, out, in, 0);
}

static struct crypto_alg aes_alg = {
	.cra_name		=	"aes",
	.cra_driver_name	=	"aes-ccm",
	.cra_priority		=	CCM_CRA_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct aes_ctx),
	.cra_alignmask		=	CCM_ALIGNMENT - 1,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(aes_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey	   	= 	aes_set_key,
			.cia_encrypt	 	=	aes_encrypt,
			.cia_decrypt	  	=	aes_decrypt,
		}
	}
};

static int __init ccm_init(void)
{
	int ret;
	if ((ret = crypto_register_alg(&aes_alg))) {
		printk(KERN_ERR PFX "CCM AES initialization failed.\n");
		return ret;
	}

	spin_lock_init(&lock);
	ccm_clock_ctrl(SYSCON_CCMCGREQ_AESCFG, SYS_ENABLE_CLOCK);

	return 0;
}

static void __exit ccm_fini(void)
{
	crypto_unregister_alg(&aes_alg);
}

module_init(ccm_init);
module_exit(ccm_fini);

MODULE_DESCRIPTION("CCM AES cipher support");
MODULE_LICENSE("GPL");
/*MODULE_AUTHOR("");*/

MODULE_ALIAS("aes");