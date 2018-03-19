/*
 * (C) Copyright 2010
 * Linaro LTD, www.linaro.org
 * Author: John Rigby <john.rigby@linaro.org>
 * Based on TI's signGP.c
 *
 * (C) Copyright 2009
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * (C) Copyright 2008
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* Required to obtain the getline prototype from stdio.h */
#define _GNU_SOURCE

#include "mkimage.h"
#include <image.h>
#include "omapimage.h"

#include <icom/sha.h>
#include <icom/rsa.h>

#ifdef CONFIG_ANDROID_TRUST_BOOT
#include <polarssl/config.h>
#include <polarssl/rsa.h>
#include <polarssl/x509.h>

#define STRINGIFY(x)	#x
#define EXPAND(x)	STRINGIFY(x)
#define RSA_PRIVATE_KEY_FILEPATH	EXPAND(CONFIG_RSA_PRIVATE_KEY)

#define OMAP_FILE_PIGGYBACK_SIZE (IH_SHA_LENGTH + IH_RSA_LENGTH)

#else /* !CONFIG_ANDROID_TRUST_BOOT */

#define OMAP_FILE_PIGGYBACK_SIZE (IH_SHA_LENGTH)

#endif /* CONFIG_ANDROID_TRUST_BOOT */

/* Header size is CH header rounded up to 512 bytes plus GP header */
#define OMAP_CH_HDR_SIZE 512
#define OMAP_GP_HDR_SIZE (sizeof(struct gp_header))
#define OMAP_FILE_HDR_SIZE (OMAP_CH_HDR_SIZE+OMAP_GP_HDR_SIZE)

static uint8_t omapimage_header[OMAP_FILE_HDR_SIZE];

static uint8_t omapimage_piggyback[OMAP_FILE_PIGGYBACK_SIZE];

static void omapimage_verify_piggyback(const void *ptr)
{
	struct gp_header *gph = (struct gp_header *)(ptr + OMAP_CH_HDR_SIZE);
	int image_size = (int)(gph->size + OMAP_FILE_HDR_SIZE);
	SHA_CTX sha_ctx;
	const uint8_t *sha1;
	uint8_t *ptr_sha = (uint8_t *)(ptr + image_size);
#ifdef CONFIG_ANDROID_TRUST_BOOT
	const RSAPublicKey *key;
	uint8_t *ptr_rsa = (uint8_t *)(((void *)ptr_sha) + IH_SHA_LENGTH);
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, (const void*)ptr, image_size);
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(sha1, ptr_sha, IH_SHA_LENGTH)) {
		fprintf(stderr, "\n****** ERROR: corrupted image data! ******\n\n");
		exit(EXIT_FAILURE);
	}

#ifdef CONFIG_ANDROID_TRUST_BOOT
	/* RSA signature */
	key = board_get_trust_boot_key();
	if (key) {
		if (RSA_verify(key, ptr_rsa, IH_RSA_LENGTH, sha1) == 0) {
			fprintf(stderr, "\n****** ERROR: corrupted image data against key! ******\n\n");
			exit(EXIT_FAILURE);
		}
	} else {
		fprintf(stderr, "\n****** ERROR: key not found! ******\n\n");
		exit(EXIT_FAILURE);
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */
}

static void omapimage_set_piggyback(void *ptr, struct stat *sbuf, off_t total_size,
				int ifd, struct mkimage_params *params)
{
	struct gp_header *gph = (struct gp_header *)(ptr + OMAP_CH_HDR_SIZE);
	SHA_CTX sha_ctx;
	const uint8_t *sha1;
	uint8_t *ptr_sha = (uint8_t *)(ptr + sbuf->st_size);
#ifdef CONFIG_ANDROID_TRUST_BOOT
	int ret;
	rsa_context rsa;
	struct stat st;
	uint8_t *ptr_rsa = (uint8_t *)(((void *)ptr_sha) + IH_SHA_LENGTH);
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	if (gph->size != (sbuf->st_size - OMAP_FILE_HDR_SIZE)) {
		fprintf(stderr,
			"Error: invalid image size %x\n", gph->size);
		exit(EXIT_FAILURE);
	}

	if (total_size != (sbuf->st_size + OMAP_FILE_PIGGYBACK_SIZE)) {
		fprintf(stderr,
			"Error: invalid image total size %x\n", (unsigned)total_size);
		exit(EXIT_FAILURE);
	}

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, (const void*)ptr, sbuf->st_size);
	sha1 = SHA_final(&sha_ctx);
	memcpy(ptr_sha, sha1, IH_SHA_LENGTH);

#ifdef CONFIG_ANDROID_TRUST_BOOT
	ret = stat(RSA_PRIVATE_KEY_FILEPATH, &st);
	if (ret != 0) {
		fprintf(stderr, "%s: not found (stat err %d)\n", RSA_PRIVATE_KEY_FILEPATH, ret);
		fprintf(stderr, "Please define CONFIG_ANDROID_TRUST_BOOT_RSA_PRIVATE_KEY correctly\n\n");
		exit(EXIT_FAILURE);
	}
	if (!S_ISREG(st.st_mode)) {
		fprintf(stderr, "%s: not a regular file\n", RSA_PRIVATE_KEY_FILEPATH);
		fprintf(stderr, "Please define CONFIG_ANDROID_TRUST_BOOT_RSA_PRIVATE_KEY correctly\n\n");
		exit(EXIT_FAILURE);
	}
	if (st.st_size < 512) {
		fprintf(stderr, "%s: file size is too small (%u)\n", RSA_PRIVATE_KEY_FILEPATH, (unsigned)st.st_size);
		fprintf(stderr, "Please define CONFIG_ANDROID_TRUST_BOOT_RSA_PRIVATE_KEY correctly\n\n");
		exit(EXIT_FAILURE);
	}

	/* RSA/SHA-1 signature creation */
	rsa_init(&rsa, RSA_PKCS_V15, 0);
	ret = x509parse_keyfile(&rsa, RSA_PRIVATE_KEY_FILEPATH, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: x509parse_keyfile err %d\n", RSA_PRIVATE_KEY_FILEPATH, ret);
		fprintf(stderr, "Please define CONFIG_ANDROID_TRUST_BOOT_RSA_PRIVATE_KEY correctly\n\n");
		exit(EXIT_FAILURE);
	} else {
		if (rsa.len != IH_RSA_LENGTH) {
			fprintf(stderr, "%s: unsupported RSA length %d\n", RSA_PRIVATE_KEY_FILEPATH, (int)rsa.len);
			exit(EXIT_FAILURE);
		} else {
			const RSAPublicKey *key;
			unsigned char rsa_signature[512];

			ret = rsa_pkcs1_sign( &rsa, NULL, NULL, RSA_PRIVATE, SIG_RSA_SHA1, IH_SHA_LENGTH, sha1, rsa_signature);
			if (ret != 0) {
				fprintf(stderr, "%s: rsa_pkcs1_sign err %d\n\n", RSA_PRIVATE_KEY_FILEPATH, ret);
				exit(EXIT_FAILURE);
			}
			memcpy(ptr_rsa, rsa_signature, IH_RSA_LENGTH);

			/* RSA signature verifying */
			key = board_get_trust_boot_key();
			if (key) {
				if (RSA_verify(key, rsa_signature, IH_RSA_LENGTH, sha1) == 0) {
					fprintf(stderr, "\n****** ERROR: corrupted image data against key! ******\n\n");
					exit(EXIT_FAILURE);
				}
			} else {
				fprintf(stderr, "\n****** ERROR: key not found! ******\n\n");
				exit(EXIT_FAILURE);
			}
		}
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */
}

static int omapimage_check_image_types(uint8_t type)
{
	if (type == IH_TYPE_OMAPIMAGE)
		return EXIT_SUCCESS;
	else {
		return EXIT_FAILURE;
	}
}

/*
 * Only the simplest image type is currently supported:
 * TOC pointing to CHSETTINGS
 * TOC terminator
 * CHSETTINGS
 *
 * padding to OMAP_CH_HDR_SIZE bytes
 *
 * gp header
 *   size
 *   load_addr
 */
static int valid_gph_size(uint32_t size)
{
	return size;
}

static int valid_gph_load_addr(uint32_t load_addr)
{
	return load_addr;
}

static int omapimage_verify_header(unsigned char *ptr, int image_size,
			struct mkimage_params *params)
{
	struct ch_toc *toc = (struct ch_toc *)ptr;
	struct gp_header *gph = (struct gp_header *)(ptr+OMAP_CH_HDR_SIZE);
	uint32_t offset, size;

	while (toc->section_offset != 0xffffffff
			&& toc->section_size != 0xffffffff) {
		offset = toc->section_offset;
		size = toc->section_size;
		if (!offset || !size)
			return -1;
		if (offset >= OMAP_CH_HDR_SIZE ||
		    offset+size >= OMAP_CH_HDR_SIZE)
			return -1;
		toc++;
	}
	if (!valid_gph_size(gph->size))
		return -1;
	if (!valid_gph_load_addr(gph->load_addr))
		return -1;

	return 0;
}

static void omapimage_print_section(struct ch_settings *chs)
{
	const char *section_name;

	if (chs->section_key)
		section_name = "CHSETTINGS";
	else
		section_name = "UNKNOWNKEY";

	printf("%s (%x) "
		"valid:%x "
		"version:%x "
		"reserved:%x "
		"flags:%x\n",
		section_name,
		chs->section_key,
		chs->valid,
		chs->version,
		chs->reserved,
		chs->flags);
}

static void omapimage_print_header(const void *ptr)
{
	const struct ch_toc *toc = (struct ch_toc *)ptr;
	const struct gp_header *gph =
			(struct gp_header *)(ptr+OMAP_CH_HDR_SIZE);
	uint32_t offset, size;

	while (toc->section_offset != 0xffffffff
			&& toc->section_size != 0xffffffff) {
		offset = toc->section_offset;
		size = toc->section_size;

		if (offset >= OMAP_CH_HDR_SIZE ||
		    offset+size >= OMAP_CH_HDR_SIZE)
			exit(EXIT_FAILURE);

		printf("Section %s offset %x length %x\n",
			toc->section_name,
			toc->section_offset,
			toc->section_size);

		omapimage_print_section((struct ch_settings *)(ptr+offset));
		toc++;
	}

	if (!valid_gph_size(gph->size)) {
		fprintf(stderr,
			"Error: invalid image size %x\n",
			gph->size);
		exit(EXIT_FAILURE);
	}

	if (!valid_gph_load_addr(gph->load_addr)) {
		fprintf(stderr,
			"Error: invalid image load address %x\n",
			gph->size);
		exit(EXIT_FAILURE);
	}

	printf("GP Header: Size %x LoadAddr %x\n",
		gph->size, gph->load_addr);

	printf ("Image Type:   TI OMAP CH/GP Boot Image\n");
	printf ("Data Size:    ");
	genimg_print_size (gph->size);
	printf ("Load Address: %08x\n", gph->load_addr);

	omapimage_verify_piggyback(ptr);
}

static int toc_offset(void *hdr, void *member)
{
	return member - hdr;
}

static void omapimage_set_header(void *ptr, struct stat *sbuf, int ifd,
				struct mkimage_params *params)
{
	struct ch_toc *toc = (struct ch_toc *)ptr;
	struct ch_settings *chs = (struct ch_settings *)
					(ptr + 2 * sizeof(*toc));
	struct gp_header *gph = (struct gp_header *)(ptr + OMAP_CH_HDR_SIZE);

	toc->section_offset = toc_offset(ptr, chs);
	toc->section_size = sizeof(struct ch_settings);
	strcpy((char *)toc->section_name, "CHSETTINGS");

	chs->section_key = KEY_CHSETTINGS;
	chs->valid = 0;
	chs->version = 1;
	chs->reserved = 0;
	chs->flags = 0;

	toc++;
	memset(toc, 0xff, sizeof(*toc));

	gph->size = sbuf->st_size - OMAP_FILE_HDR_SIZE;
	gph->load_addr = params->addr;
}

int omapimage_check_params(struct mkimage_params *params)
{
	return	(params->dflag && (params->fflag || params->lflag)) ||
		(params->fflag && (params->dflag || params->lflag)) ||
		(params->lflag && (params->dflag || params->fflag));
}

/*
 * omapimage parameters
 */
static struct image_type_params omapimage_params = {
	.name		= "TI OMAP CH/GP Boot Image support",
	.header_size	= OMAP_FILE_HDR_SIZE,
	.hdr		= (void *)&omapimage_header,
	.check_image_type = omapimage_check_image_types,
	.verify_header	= omapimage_verify_header,
	.print_header	= omapimage_print_header,
	.set_header	= omapimage_set_header,
	.check_params	= omapimage_check_params,
	.piggyback_size	= OMAP_FILE_PIGGYBACK_SIZE,
	.piggyback	= (void *)&omapimage_piggyback,
	.set_piggyback = omapimage_set_piggyback,
};

void init_omap_image_type(void)
{
	mkimage_register(&omapimage_params);
}
