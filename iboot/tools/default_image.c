/*
 * (C) Copyright 2008 Semihalf
 *
 * (C) Copyright 2000-2004
 * DENX Software Engineering
 * Wolfgang Denk, wd@denx.de
 *
 * Updated-by: Prafulla Wadaskar <prafulla@marvell.com>
 *		default_image specific code abstracted from mkimage.c
 *		some functions added to address abstraction
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "mkimage.h"
#include <image.h>
#include <u-boot/crc.h>

#include <icom/sha.h>
#include <icom/rsa.h>

#ifdef CONFIG_ANDROID_TRUST_BOOT
#include <polarssl/config.h>
#include <polarssl/rsa.h>
#include <polarssl/x509.h>
#define STRINGIFY(x)	#x
#define EXPAND(x)	STRINGIFY(x)
#define RSA_PRIVATE_KEY_FILEPATH	EXPAND(CONFIG_RSA_PRIVATE_KEY)
#endif /* CONFIG_ANDROID_TRUST_BOOT */

static image_header_t header;

static int image_verify_android_trust(unsigned char *ptr, int image_size)
{
	image_header_t *hdr = (image_header_t *)ptr;
	SHA_CTX sha_ctx;
	const uint8_t* sha1;
#ifdef CONFIG_ANDROID_TRUST_BOOT
	const RSAPublicKey *key;
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx,
			(const unsigned char *)(ptr +
				sizeof(image_header_t)),
			image_size - sizeof(image_header_t));
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(sha1, hdr->ih_sha, IH_SHA_LENGTH)) {
		fprintf(stderr, "\n****** ERROR: corrupted image data! ******\n\n");
		return EXIT_FAILURE;
	}

#ifdef CONFIG_ANDROID_TRUST_BOOT
	/* RSA signature */
	key = board_get_trust_boot_key();
	if (key) {
		if (RSA_verify(key, hdr->ih_rsa, IH_RSA_LENGTH, sha1) == 0) {
			fprintf(stderr, "\n****** ERROR: corrupted image data against key! ******\n\n");
			return EXIT_FAILURE;
		}
	} else {
		fprintf(stderr, "\n****** ERROR: key not found! ******\n\n");
		return EXIT_FAILURE;
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	return EXIT_SUCCESS;
}

static int image_check_image_types(uint8_t type)
{
	if (((type > IH_TYPE_INVALID) && (type < IH_TYPE_FLATDT)) ||
	    (type == IH_TYPE_KERNEL_NOLOAD))
		return EXIT_SUCCESS;
	else
		return EXIT_FAILURE;
}

static int image_check_params(struct mkimage_params *params)
{
	return	((params->dflag && (params->fflag || params->lflag)) ||
		(params->fflag && (params->dflag || params->lflag)) ||
		(params->lflag && (params->dflag || params->fflag)));
}

static int image_verify_header(unsigned char *ptr, int image_size,
			struct mkimage_params *params)
{
	uint32_t len;
	const unsigned char *data;
	uint32_t checksum;
	image_header_t header;
	image_header_t *hdr = &header;

	/*
	 * create copy of header so that we can blank out the
	 * checksum field for checking - this can't be done
	 * on the PROT_READ mapped data.
	 */
	memcpy(hdr, ptr, sizeof(image_header_t));

	if (be32_to_cpu(hdr->ih_magic) != IH_MAGIC) {
		fprintf(stderr,
			"%s: Bad Magic Number: \"%s\" is no valid image\n",
			params->cmdname, params->imagefile);
		return -FDT_ERR_BADMAGIC;
	}

	if (be32_to_cpu(hdr->ih_magic2) != IH_MAGIC2) {
		fprintf(stderr,
			"%s: Bad Magic Number 2: \"%s\" is no valid image\n",
			params->cmdname, params->imagefile);
		return -FDT_ERR_BADMAGIC;
	}

#if 0
	data = (const unsigned char *)hdr;
	len  = sizeof(image_header_t);

	checksum = be32_to_cpu(hdr->ih_hcrc);
	hdr->ih_hcrc = cpu_to_be32(0);	/* clear for re-calculation */

	if (crc32(0, data, len) != checksum) {
		fprintf(stderr,
			"%s: ERROR: \"%s\" has bad header checksum!\n",
			params->cmdname, params->imagefile);
		return -FDT_ERR_BADSTATE;
	}
#else
	if (hdr->ih_full_hcrc) {
		data = (const unsigned char *)hdr;
		len  = sizeof(image_header_t);

		checksum = be32_to_cpu(hdr->ih_full_hcrc);
		hdr->ih_hcrc = cpu_to_be32(0);	/* clear for re-calculation */
		hdr->ih_full_hcrc = cpu_to_be32(0);	/* clear for re-calculation */

		if (crc32(0, data, len) != checksum) {
			fprintf(stderr,
				"%s: ERROR: \"%s\" has bad header checksum!\n",
				params->cmdname, params->imagefile);
#if 0
			return -FDT_ERR_BADSTATE;
#else
			exit(EXIT_FAILURE);
#endif
		}
	} else {
		data = (const unsigned char *)hdr;
		len  = IH_HCRC_LENGTH;

		checksum = be32_to_cpu(hdr->ih_hcrc);
		hdr->ih_hcrc = cpu_to_be32(0);	/* clear for re-calculation */

		if (crc32(0, data, len) != checksum) {
			fprintf(stderr,
				"%s: ERROR: \"%s\" has bad header checksum!\n",
				params->cmdname, params->imagefile);
#if 0
			return -FDT_ERR_BADSTATE;
#else
			exit(EXIT_FAILURE);
#endif
		}
	}

	if (image_verify_android_trust(ptr, image_size)) {
		fprintf(stderr,
			"%s: ERROR: \"%s\" has corrupted data!\n",
			params->cmdname, params->imagefile);
#if 0
		return -FDT_ERR_BADSTRUCTURE;
#else
		exit(EXIT_FAILURE);
#endif
	}
#endif

	data = (const unsigned char *)ptr + sizeof(image_header_t);
	len  = image_size - sizeof(image_header_t) ;

	checksum = be32_to_cpu(hdr->ih_dcrc);
	if (crc32(0, data, len) != checksum) {
		fprintf(stderr,
			"%s: ERROR: \"%s\" has corrupted data!\n",
			params->cmdname, params->imagefile);
		return -FDT_ERR_BADSTRUCTURE;
	}
	return 0;
}

static void image_local_print_contents(const void *ptr)
{
	uint32_t len;
	const unsigned char *data;
	uint32_t checksum;
	image_header_t header;
	image_header_t *hdr = &header;

	/*
	 * create copy of header so that we can blank out the
	 * checksum field for checking - this can't be done
	 * on the PROT_READ mapped data.
	 */
	memcpy(hdr, ptr, sizeof(image_header_t));

	image_print_contents(ptr);

#ifdef CONFIG_ANDROID_TRUST_BOOT
	printf("RSA key file: %s\n\n", RSA_PRIVATE_KEY_FILEPATH);
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	if (hdr->ih_full_hcrc) {
		data = (const unsigned char *)hdr;
		len  = sizeof(image_header_t);

		checksum = be32_to_cpu(hdr->ih_full_hcrc);
		hdr->ih_hcrc = cpu_to_be32(0);	/* clear for re-calculation */
		hdr->ih_full_hcrc = cpu_to_be32(0);	/* clear for re-calculation */

		if (crc32(0, data, len) != checksum)
			fprintf(stderr, "\n****** ERROR: bad header checksum! ******\n\n");
	} else {
		data = (const unsigned char *)hdr;
		len  = IH_HCRC_LENGTH;

		checksum = be32_to_cpu(hdr->ih_hcrc);
		hdr->ih_hcrc = cpu_to_be32(0);	/* clear for re-calculation */

		if (crc32(0, data, len) != checksum)
			fprintf(stderr, "\n****** ERROR: bad header checksum! ******\n\n");
	}

	image_verify_android_trust((unsigned char *)ptr, image_get_data_size(hdr) + sizeof(image_header_t));
}

static void image_set_android_trust(void *ptr, int image_size)
{
	image_header_t *hdr = (image_header_t *)ptr;
	SHA_CTX sha_ctx;
	const uint8_t* sha1;
#ifdef CONFIG_ANDROID_TRUST_BOOT
	int ret;
	struct stat st;
	rsa_context rsa;
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx,
			(const unsigned char *)(ptr +
				sizeof(image_header_t)),
			image_size - sizeof(image_header_t));
	sha1 = SHA_final(&sha_ctx);
	memcpy(hdr->ih_sha, sha1, IH_SHA_LENGTH);

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
			memcpy(hdr->ih_rsa, rsa_signature, IH_RSA_LENGTH);

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

static void image_set_header(void *ptr, struct stat *sbuf, int ifd,
				struct mkimage_params *params)
{
	uint32_t checksum;

	image_header_t * hdr = (image_header_t *)ptr;

	image_set_android_trust(ptr, sbuf->st_size);

	checksum = crc32(0,
			(const unsigned char *)(ptr +
				sizeof(image_header_t)),
			sbuf->st_size - sizeof(image_header_t));

	/* Build new header */
	image_set_magic(hdr, IH_MAGIC);
	image_set_magic2(hdr, IH_MAGIC2);
	image_set_time(hdr, sbuf->st_mtime);
	image_set_size(hdr, sbuf->st_size - sizeof(image_header_t));
	image_set_load(hdr, params->addr);
	image_set_ep(hdr, params->ep);
	image_set_dcrc(hdr, checksum);
	image_set_os(hdr, params->os);
	image_set_arch(hdr, params->arch);
	image_set_type(hdr, params->type);
	image_set_comp(hdr, params->comp);

	image_set_name(hdr, params->imagename);

#if 0
	checksum = crc32(0, (const unsigned char *)hdr,
				sizeof(image_header_t));

	image_set_hcrc(hdr, checksum);
#else
	image_set_hcrc(hdr, 0);	/* clear for re-calculation */
	image_set_full_hcrc(hdr, 0);	/* clear for re-calculation */

#ifdef CONFIG_ANDROID_TRUST_BOOT
	checksum = crc32(0, (const unsigned char *)hdr,
				sizeof(image_header_t));
	image_set_full_hcrc(hdr, checksum);
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	checksum = crc32(0, (const unsigned char *)hdr, IH_HCRC_LENGTH);
	image_set_hcrc(hdr, checksum);
#endif
}

/*
 * Default image type parameters definition
 */
static struct image_type_params defimage_params = {
	.name = "Default Image support",
	.header_size = sizeof(image_header_t),
	.hdr = (void*)&header,
	.check_image_type = image_check_image_types,
	.verify_header = image_verify_header,
#if 0
	.print_header = image_print_contents,
#else
	.print_header = image_local_print_contents,
#endif
	.set_header = image_set_header,
	.check_params = image_check_params,
};

void init_default_image_type(void)
{
	mkimage_register(&defimage_params);
}
