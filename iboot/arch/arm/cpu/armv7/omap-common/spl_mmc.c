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
#include <mmc.h>
#include <fat.h>
#include <version.h>
#include <asm/omap_common.h>
#include <asm/arch/mmc_host_def.h>
#include <errno.h>
#if defined(CONFIG_SPL_I2C_SUPPORT) && defined(CONFIG_SPL_POWER_SUPPORT)
#if defined(CONFIG_TWL6030_POWER)
#include <twl6030.h>
#include <icom/keypad.h>
#endif /* CONFIG_TWL6030_POWER */
#endif /* CONFIG_SPL_I2C_SUPPORT, CONFIG_SPL_POWER_SUPPORT */

DECLARE_GLOBAL_DATA_PTR;

/* eMMC RAW mode booting device */
static int mmc_raw_dev __attribute__ ((section(".data"))) = 0;

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	switch (omap_boot_device()) {
	case BOOT_DEVICE_MMC1:
#ifdef CONFIG_SPL_FAT_SUPPORT
		omap_mmc_init(0, 0, 0);
#else
		/* use MMC2 if no FAT support */
		omap_mmc_init(1, 0, 0);
#endif /* CONFIG_SPL_FAT_SUPPORT */
		break;
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
#ifdef CONFIG_SPL_FAT_SUPPORT
		if (mmc_raw_dev == 1)
			omap_mmc_init(0, 0, 0);
#endif /* CONFIG_SPL_FAT_SUPPORT */
		omap_mmc_init(1, 0, 0);
		break;
	}
	return 0;
}
#endif

#if 0
static void mmc_load_image_raw(struct mmc *mmc)
{
	u32 image_size_sectors, err;
	const struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR, 1,
			(void *)header);

	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + MMCSD_SECTOR_SIZE - 1) /
				MMCSD_SECTOR_SIZE;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR,
			image_size_sectors, (void *)spl_image.load_addr);

end:
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		hang();
	}
}
#else
static int mmc_load_image_raw(block_dev_desc_t *dev_desc, unsigned long blk_start)
{
	int err = 0;
	struct image_header *header;
	unsigned long blk_cnt, n;
	const unsigned char *data;
	uint32_t len;
	unsigned int retry;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
			sizeof(struct image_header));
	/* read image header to find the image size & load address */
	retry = 3;
	while (retry--) {
		n = dev_desc->block_read(mmc_raw_dev, blk_start, 1, (void *)header);
		if (n == 1)
			break;
	}
	if (n != 1) {
		err = -EIO;
		goto end;
	}

	err = spl_parse_image_header(header);
	if (err)
		goto end;

	/* Align the size to sectors */
	blk_cnt = ALIGN(spl_image.size, dev_desc->blksz) / dev_desc->blksz;

	/* Read the header too to avoid extra memcpy */
	retry = 3;
	while (retry--) {
		n = dev_desc->block_read(mmc_raw_dev, blk_start, blk_cnt, (void *)spl_image.load_addr);
		if (n == blk_cnt)
			break;
	}
	if (n != blk_cnt) {
		err = -EIO;
		goto end;
	}

	header = (struct image_header *)spl_image.load_addr;
	data = ((const unsigned char *)header) + sizeof(image_header_t);
	len  = spl_image.size - sizeof(image_header_t);

	err = spl_verify_image_data(header, data, len);

end:
	return err;
}

#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
static void mmc_load_uboot_bin(block_dev_desc_t *dev_desc, unsigned long blk_start)
{
	unsigned long blk_cnt, n;
	unsigned int retry;

	spl_image.entry_point = CONFIG_SYS_TEXT_BASE;
	spl_image.load_addr = CONFIG_SYS_TEXT_BASE;
	spl_image.os = IH_OS_U_BOOT;

	blk_cnt = CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS;

	/* Read the header too to avoid extra memcpy */
	retry = 3;
	while (retry--) {
		n = dev_desc->block_read(mmc_raw_dev, blk_start, blk_cnt, (void *)CONFIG_SYS_TEXT_BASE);
		if (n == blk_cnt)
			break;
	}
	if (n != blk_cnt) {
		printf("eMMC err %lu\n", n);
		hang();
	}
}
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
#endif

#ifdef CONFIG_SPL_FAT_SUPPORT
#if 0
static void mmc_load_image_fat(struct mmc *mmc)
{
	s32 err;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = fat_register_device(&mmc->block_dev,
				CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
	if (err) {
		printf("spl: fat register err - %d\n", err);
		hang();
	}

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)header, sizeof(struct image_header));
	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)spl_image.load_addr, 0);

end:
	if (err <= 0) {
		printf("spl: error reading image %s, err - %d\n",
			CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME, err);
		hang();
	}
}
#else
static int mmc_load_image_fat(struct mmc *mmc)
{
	int err = 0;
	struct image_header *header;
	long n = 0;
	const unsigned char *data;
	uint32_t len;
	unsigned int retry;

	puts("## Loading from SD...\n");

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	retry = 3;
	while (retry--) {
		err = fat_register_device(&mmc->block_dev,
					CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
		if (!err)
			break;
	}
	if (err) {
		printf("FAT dev err %d\n", err);
		err = -EIO;
		goto end;
	}

	retry = 3;
	while (retry--) {
		n = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
					(u8 *)header, sizeof(struct image_header));
		if (n > 0)
			break;
	}
	if (n <= 0) {
		err = -EIO;
		goto end;
	}

	err = spl_parse_image_header(header);
	if (err)
		goto end;

	retry = 3;
	while (retry--) {
		n = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
					(u8 *)spl_image.load_addr, 0);
		if (n > 0)
			break;
	}
	if (n <= 0) {
		err = -EIO;
		goto end;
	}

	header = (struct image_header *)spl_image.load_addr;
	data = ((const unsigned char *)header) + sizeof(image_header_t);
	len  = spl_image.size - sizeof(image_header_t);

	err = spl_verify_image_data(header, data, len);

end:
	return err;
}
#endif /* CONFIG_SPL_FAT_SUPPORT */
#endif

#if 0
void spl_mmc_load_image(void)
{
	struct mmc *mmc;
	int err;
	u32 boot_mode;

	mmc_initialize(gd->bd);
	/* We register only one device. So, the dev id is always 0 */
	mmc = find_mmc_device(0);
	if (!mmc) {
		puts("spl: mmc device not found!!\n");
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
		printf("spl: mmc init failed: err - %d\n", err);
		hang();
	}
	boot_mode = omap_boot_mode();
	if (boot_mode == MMCSD_MODE_RAW) {
		debug("boot mode - RAW\n");
		mmc_load_image_raw(mmc);
	} else if (boot_mode == MMCSD_MODE_FAT) {
		debug("boot mode - FAT\n");
		mmc_load_image_fat(mmc);
	} else {
		puts("spl: wrong MMC boot mode\n");
		hang();
	}
}
#else

#ifdef CONFIG_SPL_FAT_SUPPORT
static int spl_mmc_switch_to_sdcard(void)
{
	int switch_to_sdcard = 0;
#if (CONFIG_ABOOT_FASTBOOT_ROWBITS > 0) && (CONFIG_ABOOT_FASTBOOT_COLBITS > 0)
	int check_keys = 0;
	u32 row_bits = 0, col_bits = 0;

	if (board_is_factory_mode()) {
		puts("<FACTORY>\n");
		check_keys = 1;
	} else {
#if defined(CONFIG_SPL_I2C_SUPPORT) && defined(CONFIG_SPL_POWER_SUPPORT)
#if defined(CONFIG_TWL6030_POWER)
		u8 charger_stat = 0;

#define CONTROLLER_STAT1		0x03
#define		VAC_DET				(1 << 3)	/* VAC is present */
#define		VBUS_DET			(1 << 2)	/* VBUS is present */

		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charger_stat, CONTROLLER_STAT1);
		if (charger_stat & (VBUS_DET | VAC_DET))
			check_keys = 1;
#endif /* CONFIG_TWL6030_POWER */
#endif /* CONFIG_SPL_I2C_SUPPORT, CONFIG_SPL_POWER_SUPPORT */
	}

	if (check_keys) {
		keypad_poll(&row_bits, &col_bits);
		printf("ROW:0x%03X, COL:0x%03X\n", row_bits, col_bits);

		/* Don't check the Power Button */
		row_bits &= CONFIG_SYS_KEYPAD_ROW_MASKS;
		col_bits &= CONFIG_SYS_KEYPAD_COL_MASKS;
		if (row_bits == (CONFIG_ABOOT_FASTBOOT_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				col_bits == (CONFIG_ABOOT_FASTBOOT_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS))
			switch_to_sdcard = 1;
	}
#endif /* FASTBOOT */

	return switch_to_sdcard;
}
#endif /* CONFIG_SPL_FAT_SUPPORT */

void spl_mmc_load_image(void)
{
	struct mmc *mmc;
	int err;
	u32 boot_mode;

	boot_mode = omap_boot_mode();
#ifdef CONFIG_SPL_FAT_SUPPORT
	if (boot_mode == MMCSD_MODE_RAW && omap_boot_device() == BOOT_DEVICE_MMC2) {
		if (spl_mmc_switch_to_sdcard())
			mmc_raw_dev = 1; /* SD card (MMC1) */
	}
#endif /* CONFIG_SPL_FAT_SUPPORT */

	mmc_initialize(gd->bd);

	/* Get the first MMC device */
	mmc = find_mmc_device(0);
	if (!mmc) {
		puts("no MMC dev 0\n");
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
		/* If mmc_raw_dev is 1 and no SD card, mmc_init() will return error */
		if (mmc_raw_dev != 1)
			err = mmc_rescan(mmc);
		if (err) {
			printf("dev 0 init err %d\n", err);
			if (mmc_raw_dev != 1)
				hang();
		}
	}
#ifdef CONFIG_SPL_FAT_SUPPORT
	if (boot_mode == MMCSD_MODE_RAW) {
#else
	if (boot_mode == MMCSD_MODE_RAW || boot_mode == MMCSD_MODE_FAT) {
#endif /* CONFIG_SPL_FAT_SUPPORT */
		block_dev_desc_t *dev_desc;
		unsigned long blk_start;
		u32 num;

		debug("mmc_raw_dev %d, err %d\n", mmc_raw_dev, err);

#ifdef CONFIG_SPL_FAT_SUPPORT
		if (mmc_raw_dev == 1) {
			if (err == 0) {
				debug("boot mode - FAT\n");
				err = mmc_load_image_fat(mmc);
				if (err == 0) {
					/* Overwrite boot device */
					boot_params.omap_bootdevice = BOOT_DEVICE_MMC1;
					return;
				}
				printf("FAT err %d\n", err);
			}

			mmc = find_mmc_device(mmc_raw_dev);
			if (!mmc) {
				printf("no MMC dev %d\n", mmc_raw_dev);
				hang();
			}

			err = mmc_init(mmc);
			if (err) {
				err = mmc_rescan(mmc);
				if (err) {
					printf("MMC%d init err %d\n", mmc_raw_dev, err);
					hang();
				}
			}
		}
#endif /* CONFIG_SPL_FAT_SUPPORT */

		debug("boot mode - RAW\n");

		dev_desc = &(mmc->block_dev);
		for (num = 0 ; num < CONFIG_SYS_U_BOOT_REPEAT_NUM; num++) {
			blk_start = num * CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
#ifdef DEBUG
			printf ("##<%u> Loading at 0x%lX from eMMC...", num, blk_start);
#else
			printf ("##<%u> Loading from eMMC...", num);
#endif
			err = mmc_load_image_raw(dev_desc, blk_start);
			if (err == 0) {
				putc('\n');
				break;
			} else
				printf("err %d\n", err);
		}

		if (num == CONFIG_SYS_U_BOOT_REPEAT_NUM) {
#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
			puts("u-boot ???\n");
			mmc_load_uboot_bin(dev_desc, CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR);
#else
			printf("eMMC err %d\n", err);
			hang();
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
		}
#ifdef CONFIG_SPL_FAT_SUPPORT
	} else if (boot_mode == MMCSD_MODE_FAT) {
		debug("boot mode - FAT\n");
		err = mmc_load_image_fat(mmc);
		if (err) {
			printf("FAT err %d\n", err);
			hang();
		}
#endif /* CONFIG_SPL_FAT_SUPPORT */
	} else {
		puts("bad MMC boot\n");
		hang();
	}
}
#endif
