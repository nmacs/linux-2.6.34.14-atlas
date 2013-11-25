/*
 * CCM SHA driver
 *
 * Based on sha1_generic.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */

#include <crypto/internal/hash.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/cryptohash.h>
#include <linux/types.h>
#include <crypto/sha.h>
#include <asm/byteorder.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include "ccm.h"

struct ccm_sha1_state {
	u64 count;
	u32 partial;
	u32 state[SHA1_DIGEST_SIZE / 4];
	u8 buffer[SHA1_BLOCK_SIZE];
	u32 ccm_count;
};

static spinlock_t lock;

static void hexdump(char *note, unsigned char *buf, unsigned int len)
{
               printk(KERN_CRIT "%s\n", note);
               print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
                               16, 1,
                                buf, len, false);
}

static void __sram ccm_sha1_transform(struct ccm_sha1_state *sctx, const u8 *data, unsigned int len, int close)
{
	uint32_t mode;
	uint32_t dst;
	uint32_t i;
	unsigned long iflags;
	
	mode = CCM_SHA_MODE_ALGO_SHA1;
	if (sctx->ccm_count == 0)
		mode |= CCM_SHA_MODE_ALGO_CONSTANT;
	if (close)
		mode |= CCM_SHA_MODE_CLOSE_HASH;
	
	spin_lock_irqsave(&lock, iflags);
	
	if (sctx->ccm_count != 0) {
		dst = CCM_SHA_IDIGEST;
		for (i = 0; i < SHA1_DIGEST_SIZE/sizeof(uint32_t); i++, dst += sizeof(uint32_t)) {
			putreg32(sctx->state[i], dst);
		}
		putreg32(sctx->ccm_count, CCM_SHA_DIGEST_COUNT);
	}
	
	putreg32(mode, CCM_SHA_MODE);
	putreg32(len, CCM_SHA_LENGTH);
	
	while ((getreg32(CCM_SHA_IRQSTATUS) & CCM_SHA_IRQSTATUS_INPUT_READY) == 0) {}

	for (i = 0, dst = CCM_SHA_DATA; i < SHA1_BLOCK_SIZE/sizeof(uint32_t) && len > 0; i++) {
		if (len >= sizeof(uint32_t)) {
			putreg32(*(uint32_t*)data, dst);
			dst += sizeof(uint32_t);
			data += sizeof(uint32_t);
			len -= sizeof(uint32_t);
		}
		else {
			uint32_t tmp = 0;
			memcpy(&tmp, data, len);
			putreg32(tmp, dst);
			len = 0;
		}
	}

	while ((getreg32(CCM_SHA_IRQSTATUS) & CCM_SHA_IRQSTATUS_OUTPUT_READY) == 0) {}

	memcpy(sctx->state, (void*)CCM_SHA_IDIGEST, SHA1_DIGEST_SIZE);
	sctx->ccm_count = getreg32(CCM_SHA_DIGEST_COUNT);

	spin_unlock_irqrestore(&lock, iflags);
}

static int sha1_init(struct shash_desc *desc)
{
	struct ccm_sha1_state *sctx = shash_desc_ctx(desc);
	memset(sctx, 0, sizeof(*sctx));
	return 0;
}

static int sha1_update(struct shash_desc *desc, const u8 *data,
			unsigned int len)
{
	struct ccm_sha1_state *sctx = shash_desc_ctx(desc);
	unsigned int partial, done;
	const u8 *src;

	partial = sctx->partial;
	sctx->count += len;
	done = 0;
	src = data;

	if ((partial + len) > 64) {
		if (partial) {
			done = -partial;
			memcpy(sctx->buffer + partial, data, done + 64);
			src = sctx->buffer;
		}

		do {
			ccm_sha1_transform(sctx, src, 64, 0);
			done += 64;
			src = data + done;
		} while (done + 64 < len);
		partial = 0;
	}
	memcpy(sctx->buffer + partial, src, len - done);
	sctx->partial = partial + len - done;

	return 0;
}

/* Add padding and return the message digest. */
static int sha1_final(struct shash_desc *desc, u8 *out)
{
	struct ccm_sha1_state *sctx = shash_desc_ctx(desc);

	ccm_sha1_transform(sctx, sctx->buffer, sctx->partial, 1);
	memcpy(out, sctx->state, SHA1_DIGEST_SIZE);

	/* Wipe context */
	memset(sctx, 0, sizeof *sctx);

	return 0;
}

static int sha1_export(struct shash_desc *desc, void *out)
{
	struct ccm_sha1_state *sctx = shash_desc_ctx(desc);

	memcpy(out, sctx, sizeof(*sctx));
	return 0;
}

static int sha1_import(struct shash_desc *desc, const void *in)
{
	struct ccm_sha1_state *sctx = shash_desc_ctx(desc);

	memcpy(sctx, in, sizeof(*sctx));
	return 0;
}

static struct shash_alg ccm_sha1_alg = {
	.digestsize	=	SHA1_DIGEST_SIZE,
	.init		=	sha1_init,
	.update		=	sha1_update,
	.final		=	sha1_final,
	.export		=	sha1_export,
	.import		=	sha1_import,
	.descsize	=	sizeof(struct ccm_sha1_state),
	.statesize	=	sizeof(struct ccm_sha1_state),
	.base		=	{
		.cra_name	=	"sha1",
		.cra_driver_name=	"sha1-ccm",
		.cra_flags	=	CRYPTO_ALG_TYPE_SHASH,
		.cra_priority		=	CCM_CRA_PRIORITY,
		.cra_blocksize	=	SHA1_BLOCK_SIZE,
		.cra_module	=	THIS_MODULE,
	}
};

static int __init ccm_sha_init(void)
{
	spin_lock_init(&lock);
	
	return crypto_register_shash(&ccm_sha1_alg);
}

static void __exit ccm_sha_exit(void)
{
	crypto_unregister_shash(&ccm_sha1_alg);
}

module_init(ccm_sha_init);
module_exit(ccm_sha_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CCM SHA hash support");
/*MODULE_AUTHOR("");*/

MODULE_ALIAS("sha");