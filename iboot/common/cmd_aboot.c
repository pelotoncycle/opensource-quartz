/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * aboot commands
 */
#include <common.h>
#include <command.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <mmc.h>

#include <icom/aboot.h>

/*-------------------------------------------------------------------------*/

/*#define CONFIG_ABOOT_DEBUG*/
/*#define CONFIG_ABOOT_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_ABOOT_DEBUG
#define CONFIG_ABOOT_DEBUG
#endif /* CONFIG_ABOOT_DEBUG */
#ifndef CONFIG_ABOOT_VERBOSE_DEBUG
#define CONFIG_ABOOT_VERBOSE_DEBUG
#endif /* CONFIG_ABOOT_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_ABOOT_DEBUG
#define ABOOT_DPRINT(fmt, args...) \
	do {printf("[aboot] " fmt, ##args);} while (0)
#define ABOOT_DPUTS(fmt) \
	do {puts("[aboot] " fmt);} while (0)
#else /* !CONFIG_ABOOT_DEBUG */
#define ABOOT_DPRINT(fmt, args...) \
	do {} while (0)
#define ABOOT_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_ABOOT_DEBUG */

#ifdef CONFIG_ABOOT_VERBOSE_DEBUG
#define ABOOT_VPRINT(fmt, args...) \
	do {printf("[aboot] " fmt, ##args);} while (0)
#define ABOOT_VPUTS(fmt) \
	do {puts("[aboot] " fmt);} while (0)
#else /* !CONFIG_ABOOT_VERBOSE_DEBUG */
#define ABOOT_VPRINT(fmt, args...) \
	do {} while (0)
#define ABOOT_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_ABOOT_VERBOSE_DEBUG */

#define ABOOT_PRINT(fmt, args...) \
	do {printf("aboot: " fmt, ##args);} while (0)
#define ABOOT_PUTS(fmt) \
	do {puts("aboot: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)

/*-------------------------------------------------------------------------*/

DECLARE_GLOBAL_DATA_PTR;

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_CMD_ABOOT)
int do_aboot (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;

	if (argc < 4) {
		ABOOT_PUTS("usage: aboot <interface> <dev[:part]> <name/addr>\n");
		return 0;
	}

	if (strcmp("mmc", argv[1]) == 0) {
		int dev = 0;
		int part = 1;
		char *ep;
		int err;
		struct mmc *mmc;

		dev = (int)simple_strtoul(argv[2], &ep, 16);
		mmc = find_mmc_device(dev);
		if (!mmc) {
			ABOOT_PUTS("Invalid mmc device\n");
			return 1;
		}

		err = mmc_rescan(mmc);
		if (err) {
			ABOOT_PRINT("mmc%d init failed: %d\n", dev, err);
			return 1;
		}

		if (IS_SD(mmc)) {
			char filename[16];

			if (*ep) {
				if (*ep != ':') {
					ABOOT_PUTS("Invalid mmc device, use 'dev[:part]\n");
					return 1;
				}
				part = (int)simple_strtoul(++ep, NULL, 16);
			}

			sprintf(filename, "%.8s.img", argv[3]);
			ABOOT_DPRINT("interface: %s\n", argv[1]);
			ABOOT_DPRINT("dev: %d\n", dev);
			ABOOT_DPRINT("part: %d\n", part);
			ABOOT_DPRINT("filename: %s\n", filename);
			ret = aboot_sdcard_fat_boot(dev, part, filename);
		} else {
			ABOOT_DPRINT("interface: %s\n", argv[1]);
			ABOOT_DPRINT("dev: %d\n", dev);
			ABOOT_DPRINT("partition: %s\n", argv[3]);
			ret = aboot_mmc_boot(dev, argv[3]);
		}
	} else if (strcmp("nand", argv[1]) == 0) {
		ABOOT_PUTS("Not implemented yet!\n");
	} else {
		ABOOT_PUTS("Invalid boot device!\n");
	}

	return ret;
}

U_BOOT_CMD(
	aboot,	4,	0,	do_aboot,
	"boot Android bootimg",
	"<interface> <dev[:part]> <name/addr>\n"
	"\t- boot Android bootimg from 'dev' on 'interface' by 'name'/'addr'\n"
	"\t- 'interface' can be 'mmc' or 'nand'\n"
	"\t- 'dev' can be 0 or 1\n"
	"\t- 'part' is the partition number in SD card\n"
	"\t- 'name' is the EFI partition name in eMMC or\n"
	"\t-           the base file without extension in SD card\n"
	"\t- 'addr' is the address in NAND\n"
    "\t- Examples:\n"
    "\t\taboot mmc 0 boot\n"
    "\t\taboot mmc 1 boot\n"
    "\t\taboot nand 0 0x011E0000\n"
);

static int do_aparts(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	aboot_print_parts();
	return 0;
}

U_BOOT_CMD(
	aparts,	1,	0,	do_aparts,
	"Android partitions",
	"Print Android partitions"
);
#endif /* CONFIG_CMD_ABOOT */

/*******************************************************************/
/* bootd - boot default image */
/*******************************************************************/
#if defined(CONFIG_CMD_BOOTD)
int do_bootd (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	aboot();
	return 0;
}

U_BOOT_CMD(
	boot,	1,	1,	do_bootd,
	"boot default",
	""
);
#endif /* CONFIG_CMD_BOOTD */

