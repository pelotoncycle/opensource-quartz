/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/u-boot.h>
#include <asm/utils.h>
#include <asm/arch/sys_proto.h>
#include <nand.h>
#include <mmc.h>
#include <fat.h>
#include <version.h>
#include <asm/omap_common.h>
#include <asm/arch/mmc_host_def.h>
#include <i2c.h>
#include <image.h>
#include <malloc.h>
#include <linux/compiler.h>
#include <errno.h>
#if defined(CONFIG_SPL_I2C_SUPPORT) && defined(CONFIG_SPL_POWER_SUPPORT)
#if defined(CONFIG_TWL6030_POWER)
#include <twl6030.h>
#endif /* CONFIG_TWL6030_POWER */
#endif /* CONFIG_SPL_I2C_SUPPORT, CONFIG_SPL_POWER_SUPPORT */

DECLARE_GLOBAL_DATA_PTR;

#if 0
u32* boot_params_ptr = NULL;
#endif
struct spl_image_info spl_image;

/* Define global data structure pointer to it*/
static gd_t gdata __attribute__ ((section(".data")));
static bd_t bdata __attribute__ ((section(".data")));

#ifdef CONFIG_SPL_BOARD_INIT
static int spl_board_init_done = -1;
void hang(void)
{
#if defined(CONFIG_SPL_I2C_SUPPORT) && defined(CONFIG_SPL_POWER_SUPPORT)
#if defined(CONFIG_TWL6030_POWER)
	if (spl_board_init_done == 1) {
		puts("\nERR: Power down!\n\n");
		twl6030_poweroff();
	}
#endif /* CONFIG_TWL6030_POWER */
#endif /* CONFIG_SPL_I2C_SUPPORT, CONFIG_SPL_POWER_SUPPORT */
	puts("\nERR: Halted!\n\n");
	for (;;) ;
}
#else
void hang(void)
{
	puts("### ERROR ### Please RESET the board ###\n");
	for (;;)
		;
}
#endif /* CONFIG_SPL_BOARD_INIT */

void board_init_f(ulong dummy)
{
	u32 boot_dev = omap_boot_device();
	switch (boot_dev) {
	case BOOT_DEVICE_MMC1:
		puts("<MMC1");
		break;
	case BOOT_DEVICE_MMC2:
		puts("<MMC2");
		break;
	case BOOT_DEVICE_USB:
		puts("<USB");
		break;
	default:
		printf("<0x%02X", boot_dev);
		break;
	}
#if defined(CONFIG_OMAP44XX)
	printf(" boot 0x%02X>\n", __raw_readl(CONTROL_STATUS) & SYS_BOOT_MASK);
#else
	puts(" boot>\n");
#endif /* CONFIG_OMAP44XX */

	/*
	 * We call relocate_code() with relocation target same as the
	 * CONFIG_SYS_SPL_TEXT_BASE. This will result in relocation getting
	 * skipped. Instead, only .bss initialization will happen. That's
	 * all we need
	 */
	debug(">>board_init_f(0x%08x, 0x%p, 0x%08x)\n", CONFIG_SPL_STACK, &gdata, CONFIG_SPL_TEXT_BASE);
	relocate_code(CONFIG_SPL_STACK, &gdata, CONFIG_SPL_TEXT_BASE);
}

/*
 * Default function to determine if u-boot or the OS should
 * be started. This implementation always returns 1.
 *
 * Please implement your own board specific funcion to do this.
 *
 * RETURN
 * 0 to not start u-boot
 * positive if u-boot should start
 */
#ifdef CONFIG_SPL_OS_BOOT
__weak int spl_start_uboot(void)
{
	printf("SPL: Please implement spl_start_uboot() for your board\n");
	printf("SPL: Direct Linux boot not active!\n");
	return 1;
}
#endif

#if 0
void spl_parse_image_header(const struct image_header *header)
{
	u32 header_size = sizeof(struct image_header);

	if (__be32_to_cpu(header->ih_magic) == IH_MAGIC) {
		spl_image.size = __be32_to_cpu(header->ih_size) + header_size;
		spl_image.entry_point = __be32_to_cpu(header->ih_load);
		/* Load including the header */
		spl_image.load_addr = spl_image.entry_point - header_size;
		spl_image.os = header->ih_os;
		spl_image.name = (const char *)&header->ih_name;
		debug("spl: payload image: %s load addr: 0x%x size: %d\n",
			spl_image.name, spl_image.load_addr, spl_image.size);
	} else {
		/* Signature not found - assume u-boot.bin */
		printf("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		debug("Assuming u-boot.bin ..\n");
		/* Let's assume U-Boot will not be more than 200 KB */
		spl_image.size = 200 * 1024;
		spl_image.entry_point = CONFIG_SYS_TEXT_BASE;
		spl_image.load_addr = CONFIG_SYS_TEXT_BASE;
		spl_image.os = IH_OS_U_BOOT;
		spl_image.name = "U-Boot";
	}
}
#else
int spl_parse_image_header(struct image_header *header)
{
	uint32_t checksum;

	/* Header magic & magic 2 */
	if (__be32_to_cpu(header->ih_magic) != IH_MAGIC ||
			__be32_to_cpu(header->ih_magic2) != IH_MAGIC2) {
		puts("bad magic\n");
		return -EINVAL;
	}

	/* Header checksum */
#if 0
	checksum = header->ih_hcrc;

	header->ih_hcrc = 0;	/* clear for re-calculation */
	header->ih_hcrc = cpu_to_be32(
			crc32(0, (const unsigned char *)header, (uint32_t)sizeof(image_header_t)));
	if (checksum != header->ih_hcrc) {
		puts("bad header\n");
		return -EINVAL;
	}
#else
	if (header->ih_full_hcrc) {
		uint32_t ih_hcrc;

		checksum = header->ih_full_hcrc;
		ih_hcrc = header->ih_hcrc;

		header->ih_hcrc = 0;	/* clear for re-calculation */
		header->ih_full_hcrc = 0;	/* clear for re-calculation */
		header->ih_full_hcrc = cpu_to_be32(
				crc32(0, (const unsigned char *)header, (uint32_t)sizeof(image_header_t)));
		if (checksum != header->ih_full_hcrc) {
			puts("bad header\n");
			return -EINVAL;
		}
		header->ih_hcrc = ih_hcrc;
	} else {
		checksum = header->ih_hcrc;

		header->ih_hcrc = 0;	/* clear for re-calculation */
		header->ih_hcrc = cpu_to_be32(
				crc32(0, (const unsigned char *)header, (uint32_t)IH_HCRC_LENGTH));
		if (checksum != header->ih_hcrc) {
			puts("bad header\n");
			return -EINVAL;
		}
	}
#endif

	/* Validate the board name including the NULL terminator */
	if (memcmp(header->ih_name, CONFIG_SYS_BOARD_NAME, sizeof(CONFIG_SYS_BOARD_NAME))) {
		printf("\nWARN: bad board '%s'<->'%s'\n", header->ih_name, CONFIG_SYS_BOARD_NAME);
	}

	spl_image.size = __be32_to_cpu(header->ih_size) + sizeof(struct image_header);
	spl_image.entry_point = __be32_to_cpu(header->ih_load);
	/* Load including the header */
	spl_image.load_addr = spl_image.entry_point - sizeof(struct image_header);
	spl_image.os = header->ih_os;
	spl_image.name = (const char *)&header->ih_name;

	debug("spl: payload image: %s load addr: 0x%x size: %d\n",
		spl_image.name, spl_image.load_addr, spl_image.size);

	return 0;
}
#endif

#if 0

int spl_verify_image_data(struct image_header *header,
		const unsigned char *data, uint32_t len)
{
	uint32_t checksum;

	checksum = crc32(0, data, len);
	if (checksum != be32_to_cpu(header->ih_dcrc)) {
		puts("corrupted data!\n");
		return -EBADMSG;
	}

	return 0;
}

#else

#include <icom/sha.h>
#include <icom/rsa.h>

static SHA_CTX sha_ctx;

int spl_verify_image_data(struct image_header *header,
		const unsigned char *data, uint32_t len)
{
	const uint8_t* sha1;
#ifdef CONFIG_ANDROID_TRUST_BOOT
	const RSAPublicKey *key;
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	/* SHA */
	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, data, len);
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(sha1, header->ih_sha, IH_SHA_LENGTH)) {
		puts("corrupted data!\n");
		return -EBADMSG;
	}

#ifdef CONFIG_ANDROID_TRUST_BOOT
	/* RSA signature */
	key = board_get_trust_boot_key();
	if (key && RSA_verify(key, header->ih_rsa, IH_RSA_LENGTH, sha1) == 0) {
		puts("corrupted data against key!\n");
		return -EBADMSG;
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	return 0;
}

#endif

/*
 * This function jumps to an image with argument. Normally an FDT or ATAGS
 * image.
 * arg: Pointer to paramter image in RAM
 */
#ifdef CONFIG_SPL_OS_BOOT
static void __noreturn jump_to_image_linux(void *arg)
{
	debug("Entering kernel arg pointer: 0x%p\n", arg);
	typedef void (*image_entry_arg_t)(int, int, void *)
		__attribute__ ((noreturn));
	image_entry_arg_t image_entry =
		(image_entry_arg_t) spl_image.entry_point;
	cleanup_before_linux();
	image_entry(0, CONFIG_MACH_TYPE, arg);
}
#endif

static void __noreturn jump_to_image_no_args(void)
{
	typedef void __noreturn (*image_entry_noargs_t)(u32 *);
	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image.entry_point;

	debug("image entry point: 0x%X\n", spl_image.entry_point);
	/* Pass the saved boot_params from rom code */
#if defined(CONFIG_VIRTIO) || defined(CONFIG_ZEBU)
	image_entry = (image_entry_noargs_t)0x80100000;
#endif
#if 0
	u32 boot_params_ptr_addr = (u32)&boot_params_ptr;
#else
	u32 boot_params_ptr_addr = (u32)&boot_params;
	debug("Boot ROM R0 0x%08X\n", (u32)boot_params.boot_message);
	debug("SPL boot_params 0x%08X\n", boot_params_ptr_addr);
#endif
	image_entry((u32 *)boot_params_ptr_addr);
}

void board_init_r(gd_t *id, ulong dummy)
{
	u32 boot_device;
	debug(">>spl:board_init_r()\n");

#if defined(CONFIG_SYS_SPL_MALLOC_SIZE)
	mem_malloc_init(CONFIG_SYS_SPL_MALLOC_START,
			CONFIG_SYS_SPL_MALLOC_SIZE);
#endif /* CONFIG_SYS_SPL_MALLOC_SIZE */

#ifdef CONFIG_SPL_BOARD_INIT
	spl_board_init();
	spl_board_init_done = 1;
#endif

	boot_device = omap_boot_device();
	debug("boot device - %d\n", boot_device);
	switch (boot_device) {
#ifdef CONFIG_SPL_MMC_SUPPORT
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		spl_mmc_load_image();
		break;
#endif
#ifdef CONFIG_SPL_NAND_SUPPORT
	case BOOT_DEVICE_NAND:
		spl_nand_load_image();
		break;
#endif
#ifdef CONFIG_SPL_YMODEM_SUPPORT
	case BOOT_DEVICE_UART:
		spl_ymodem_load_image();
		break;
#endif
#ifdef CONFIG_SPL_USB_SUPPORT
	case BOOT_DEVICE_USB:
		spl_usb_load_image();
		break;
#endif
	default:
#if 0
		printf("SPL: Un-supported Boot Device - %d!!!\n", boot_device);
#else
		printf("bad boot dev %d\n", boot_device);
#endif
		hang();
		break;
	}

	switch (spl_image.os) {
	case IH_OS_U_BOOT:
		debug("Jumping to U-Boot\n");
		jump_to_image_no_args();
		break;
#ifdef CONFIG_SPL_OS_BOOT
	case IH_OS_LINUX:
		debug("Jumping to Linux\n");
		spl_board_prepare_for_linux();
		jump_to_image_linux((void *)CONFIG_SYS_SPL_ARGS_ADDR);
		break;
#endif
	default:
#if 0
		puts("Unsupported OS image.. Jumping nevertheless..\n");
#else
		puts("bad image\n");
#endif
		jump_to_image_no_args();
	}
}

/* This requires UART clocks to be enabled */
void preloader_console_init(void)
{
#if 0
	const char *u_boot_rev = U_BOOT_VERSION;

	gd = &gdata;
	gd->bd = &bdata;
	gd->flags |= GD_FLG_RELOC;
	gd->baudrate = CONFIG_BAUDRATE;

	serial_init();		/* serial communications setup */

	gd->have_console = 1;

	/* Avoid a second "U-Boot" coming from this string */
	u_boot_rev = &u_boot_rev[7];

	printf("\nU-Boot SPL %s (%s - %s)\n", u_boot_rev, U_BOOT_DATE,
		U_BOOT_TIME);

#else

	gd = &gdata;
	gd->bd = &bdata;
	gd->flags |= GD_FLG_RELOC;
	gd->baudrate = CONFIG_BAUDRATE;

	serial_init();		/* serial communications setup */

	gd->have_console = 1;

#ifdef CONFIG_SYS_IBOOT_VERSION
	puts("\n\ni-Boot " CONFIG_SYS_IBOOT_VERSION " " PLAIN_VERSION "\n");
#else
	puts("\n\ni-Boot MLO " PLAIN_VERSION "\n");
#endif /* CONFIG_SYS_IBOOT_VERSION */

#endif
	omap_rev_string();
}

#if defined(CONFIG_OMAP44XX)
#else
void __weak omap_rev_string()
{
	printf("Texas Instruments Revision detection unimplemented\n");
}
#endif /* CONFIG_OMAP44XX */
