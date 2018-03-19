/*
 * (C) Copyright 2013
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>

#include <mmc.h>

#include <icom/fastboot.h>
#include <icom/aboot.h>

#include "../common/board_omap4_common.h"

/*-------------------------------------------------------------------------*/

/* Partition Scheme for InnoComm Quartz:
**   1 Block/LBA = 512 Bytes
** +---------------------------------------+ 0x00000000
** | MBR & GPT partition tables            |   128KB
** +---------------------------------------+ 0x00000100
** | MLO (SPL) (RAW)                       |   384KB
** +---------------------------------------+ 0x00000400
** | Bootloader (InnoComm Bootloader) (RAW)|   1536KB
** +---------------------------------------+ 0x00001000
** | Bootloader Environment (RAW)          |   1MB
** +---------------------------------------+ 0x00001800
** | Android crypto (RAW)                  |   64KB
** +---------------------------------------+ 0x00002000
** | Android efs (SPARSE EXT4)             |   20MB
** +---------------------------------------+ 0x0000C000
** | Android misc (RAW)                    |   128KB
** +---------------------------------------+ 0x0000D000
** | Boot Logo for Bootloader (RAW)        |   6MB
** +---------------------------------------+ 0x00010000
** | Other Logos for Bootloader (RAW)      |   32MB
** +---------------------------------------+ 0x00020000
** | Android recovery (ABOOT)              |   16MB
** +---------------------------------------+ 0x00028000
** | Android boot (ABOOT)                  |   10MB
** +---------------------------------------+ 0x0002D000
** | Android cache (SPARSE EXT4)           |   928MB
** +---------------------------------------+ 0x001FD000
** | Android system (SPARSE EXT4)          |   928MB (Limited by the SDRAM size)
** +---------------------------------------+ 0x003CD000
** | Android userdata (SPARSE EXT4)        |   Size is varied due to the eMMC Capacity
** +---------------------------------------+ eMMC Capacity - 1 (LBAs - 1)
** | Secondary/Alternate GPT Header        |   1 Block (512 Bytes)
** +---------------------------------------+ eMMC Capacity (LBAs)
** 
** MBR & GPT partition tables:
**   http://en.wikipedia.org/wiki/GUID_Partition_Table
**   https://wiki.archlinux.org/index.php/GUID_Partition_Table
** +------------------------------------+ 0x00000000
** | Protective MBR                     |   1 Block (512 Bytes)
** +------------------------------------+ 0x00000001
** | Primary GPT Header                 |   1 Block (512 Bytes)
** +------------------------------------+ 0x00000002
** | Primary GPT Entries                |   32 Blocks
** | Entry1 | Entry2 | Entry3 | Entry4  |   (128 GPT Entries)
** | Entry 5~128                        |
** +------------------------------------+ 0x00000022
** | Secondary/Alternate GPT Entries    |   32 Blocks
** | Entry1 | Entry2 | Entry3 | Entry4  |   (128 GPT Entries)
** | Entry 5~128                        |
** +------------------------------------+ 0x00000042
** | ////////////////////////////////// |
** | ////////////////////////////////// |
** +------------------------------------+ eMMC Capacity - 1 (LBAs - 1)
** | Secondary/Alternate GPT Header     |   1 Block (512 Bytes)
** +------------------------------------+ eMMC Capacity (LBAs)
** PS.
**   1. The start of the Secondary/Alternate GPT Entries doesn't follow the GPT/EFI spec.
**      They are stored from 0x00000022 because we would like to optimize the usages of the partitions.
**   2. If both GPT tables are invalid, ABOOT will reset it to the default predefined partition table.
**
**
** SPL/MLO:
** +------------------------------------+ 0x00000100
** | MLO 1                              |   128KB
** +------------------------------------+ 0x00000200
** | MLO 2                              |   128KB
** +------------------------------------+ 0x00000300
** | MLO 3                              |   128KB
** +------------------------------------+ 0x00000400
**
** i-Boot (InnoComm Bootloader):
** +------------------------------------+ 0x00000400
** | i-Boot 1                           |   512KB
** +------------------------------------+ 0x00000800
** | i-Boot 2                           |   512KB
** +------------------------------------+ 0x00000C00
** | i-Boot 3                           |   512KB
** +------------------------------------+ 0x00001000
**
*/

static const aboot_partition partitions[] = {
	{
		.name = "xloader",
		.part_blk_start = 0x0100,
		.part_blk_num   = 0x0100, /* 0x100 x 3 = 128 KBytes x 3 = 0x300 blocks */
		.flags = ABOOT_PART_FLAGS_TYPE_OMAP_MLO |
			ABOOT_PART_FLAGS_REPEAT(3),
	},
	{
		.name = "bootloader",
		.part_blk_start = 0x0400,
		.part_blk_num   = 0x0400, /* 0x400 x 3 = 512 KBytes x 3 = 0xC00 blocks */
		.flags = ABOOT_PART_FLAGS_TYPE_IBOOT |
			ABOOT_PART_FLAGS_REPEAT(3),
	},
	{
		.name = "param", /* environment */
		.part_blk_start = 0x01000,
		.part_blk_num   = 0x00400, /* 0x400 x 2 = 512 KBytes x 2 = 0x800 blocks */
		.flags = ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_HIDDEN | ABOOT_PART_FLAGS_LOCKED |
			ABOOT_PART_FLAGS_TYPE_RAW | ABOOT_PART_FLAGS_REPEAT(2),
	},
	{
		.name = "crypto", /* metadata */
		.part_blk_start = 0x01800,
		.part_blk_num   = 0x00080, /* 64 KBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_RAW | ABOOT_PART_FLAGS_LOCKED,
	},
	{
		.name = "efs",
		.part_blk_start = 0x02000,
		.part_blk_num   = 0x0A000, /* 20 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4,
	},
	{
		.name = "misc",
		.part_blk_start = 0x0C000,
		.part_blk_num   = 0x00100, /* 128 KBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_RAW,
	},
	{
		.name = "bootlogo",
		.part_blk_start = 0x0D000, /* fixed at 0xD000. DON'T modify. */
		.part_blk_num   = 0x03000, /* 6 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_RAW,
	},
	{
		.name = "logos",
		.part_blk_start = 0x010000,
		.part_blk_num   = 0x010000, /* 32 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_RAW,
	},
	{
		.name = "recovery",
		.part_blk_start = 0x020000,
		.part_blk_num   = 0x008000, /* 16 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_ABOOT,
	},
	{
		.name = "boot",
		.part_blk_start = 0x028000,
		.part_blk_num   = 0x005000, /* 10 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_ABOOT,
	},
	{
		.name = "cache",
		.part_blk_start = ABOOT_PART_START_APPEND,
		.part_blk_num   = 0x01D0000, /* 928 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4 |
				ABOOT_PART_FLAGS_ERASABLE,
	},
	{
		.name = "system",
		.part_blk_start = ABOOT_PART_START_APPEND,
		.part_blk_num   = 0x01D0000, /* 928 MBytes */
		.flags = ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4,
	},
	{
		.name = "userdata",
		.part_blk_start = ABOOT_PART_START_APPEND,
		.part_blk_num   = ABOOT_PART_NUM_FULL,
		.flags = ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4 |
				ABOOT_PART_FLAGS_ERASABLE | ABOOT_PART_FLAGS_SIZE_4K_ALIGNED,
	},
};

/*-------------------------------------------------------------------------*/

/**
 * Outputs:
 *   The partitions
 *
 * Returns:
 *   The number of the partitions
 */
int aboot_board_get_parts(aboot_partition **parts)
{
	*parts = (aboot_partition *)partitions;
	return ARRAY_SIZE(partitions);
}

/*-------------------------------------------------------------------------*/

void fastboot_board_init(struct usb_string *strings, struct usb_device_descriptor *desc)
{
	innocomm_fastboot_board_init(strings, desc);

	strings[FASTBOOT_STR_MANUFACTURER_IDX].s	= "Peloton";
	strings[FASTBOOT_STR_PRODUCT_IDX].s			= "Android Device";

	desc->idVendor	= 0x0451;	/* TI VID */
	desc->idProduct	= 0xD022;	/* TI Fastboot PID */
}

/*-------------------------------------------------------------------------*/
