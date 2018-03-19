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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _ABOOT_H
#define _ABOOT_H

#include <linux/list.h>
#include <mmc.h>

/*-------------------------------------------------------------------------*/

/**
 * ABOOT Partition definition structure:
 *
 * For each partition, these fields are available:
 *
 * name: string that will be used to label the partition device.
 *
 * part_blk_start: absolute starting block of the partition.
 *   * if defined as ABOOT_PART_START_APPEND, the partition will start where the previous one ended.
 *   * if defined as ABOOT_PART_START_DUMMY, the starting block is 0.
 *
 * part_blk_num: the basic number of blocks in the partition;
 *   * if defined as ABOOT_PART_NUM_FULL, the partition will extend to the end of the device.
 *   * if defined as ABOOT_PART_NUM_DYNAMIC, the partition size is gotten from aboot_board_get_part_size().
 *   * if ABOOT_PART_FLAGS_REPEAT(n) is defined, the total number of blocks in partition is
 *     the basic number of blocks multiplied by the repeat number.
 *
 * flags: contains flags that control the details of the operations
 *   See the ABOOT_PART_FLAGS__*'s defined below.
 */
typedef struct _aboot_partition {
	const char *name;
	u32 part_blk_start;	/* # of first block in partition */
	u32 part_blk_num;		/* basic number of blocks in partition */
	u32 flags;
} aboot_partition;

#define ABOOT_PART_NAME_LEN				(15)

#define ABOOT_PART_START_APPEND			(-1)
#define ABOOT_PART_START_DUMMY			(-2)

#define ABOOT_PART_NUM_FULL				(0)
#define ABOOT_PART_NUM_DYNAMIC			(1)

/**
 * The total number of blocks in partition is
 * the basic number of blocks multiplied by the repeated number
 */
#define ABOOT_PART_FLAGS_REPEAT(n)		(n & 0x0F)
#define ABOOT_PART_FLAGS_REPEAT_MASK	0x0000000F
#define ABOOT_PART_GET_REPEAT(flags)	(flags & ABOOT_PART_FLAGS_REPEAT_MASK)

/* The device type: eMMC/NAND */
#define ABOOT_PART_FLAGS_DEVICE_SHIFT	4
#define ABOOT_PART_FLAGS_DEVICE_MASK	0x000000F0
#define ABOOT_PART_GET_DEVICE(flags)	(flags & ABOOT_PART_FLAGS_DEVICE_MASK)
#define ABOOT_PART_FLAGS_DEVICE_EMMC	(0 << ABOOT_PART_FLAGS_DEVICE_SHIFT)
#define ABOOT_PART_FLAGS_DEVICE_NAND	(1 << ABOOT_PART_FLAGS_DEVICE_SHIFT)
#ifdef CONFIG_CMD_NAND
#define ABOOT_PART_FLAGS_DEVICE_MAX		2
#else
#define ABOOT_PART_FLAGS_DEVICE_MAX		1
#endif /* CONFIG_CMD_NAND */

/* The partition type: RAW/YAFFS2/EXT4/VFAT */
#define ABOOT_PART_FLAGS_TYPE_SHIFT			8
#define ABOOT_PART_FLAGS_TYPE_MASK			0x0000FF00
#define ABOOT_PART_GET_TYPE_INDEX(flags)	((flags & ABOOT_PART_FLAGS_TYPE_MASK) >> ABOOT_PART_FLAGS_TYPE_SHIFT)
#define ABOOT_PART_GET_TYPE(flags)			(flags & ABOOT_PART_FLAGS_TYPE_MASK)
/* RAW filesystem on NAND or eMMC */
#define ABOOT_PART_FLAGS_TYPE_RAW			(0 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* Modem/Radio */
#define ABOOT_PART_FLAGS_TYPE_RADIO			(1 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* InnoComm multi images (multimg) on NAND or eMMC */
#define ABOOT_PART_FLAGS_TYPE_ICOM_MULTIMG	(2 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* i-Boot image on NAND or eMMC */
#define ABOOT_PART_FLAGS_TYPE_IBOOT			(3 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* Android mkbootimg format on NAND or eMMC */
#define ABOOT_PART_FLAGS_TYPE_ABOOT			(4 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* Sparse EXT4 filesystem on eMMC */
#define ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4	(5 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* YAFFS2 filesystem on NAND */
#define ABOOT_PART_FLAGS_TYPE_YAFFS2		(6 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* FAT32 filesystem on eMMC */
#define ABOOT_PART_FLAGS_TYPE_VFAT			(7 << ABOOT_PART_FLAGS_TYPE_SHIFT)
/* OMAP MLO */
#define ABOOT_PART_FLAGS_TYPE_OMAP_MLO		(8 << ABOOT_PART_FLAGS_TYPE_SHIFT)
#define ABOOT_PART_FLAGS_TYPE_MAX			9

/* The other attributes in bit fields of the partition */
#define ABOOT_PART_FLAGS_OTHER_MASK			0xFFFF0000
#define ABOOT_PART_GET_OTHER(flags)			(flags & ABOOT_PART_FLAGS_OTHER_MASK)
/* The partition is data erasable */
#define ABOOT_PART_FLAGS_ERASABLE			0x00010000
/* The partition is locked */
#define ABOOT_PART_FLAGS_LOCKED				0x00020000
/* Reserved partition and Kernel won't see it */
#define ABOOT_PART_FLAGS_RESERVED			0x00040000
/* The hidden partition in NAND device and Android won't see it */
#define ABOOT_PART_FLAGS_HIDDEN				0x00080000
/* The partition is dummy and its size will be ingored */
#define ABOOT_PART_FLAGS_DUMMY				0x00100000
/* The partition size must be 4096 block-aligned */
#define ABOOT_PART_FLAGS_SIZE_4K_ALIGNED	0x00200000

/*-------------------------------------------------------------------------*/

typedef struct aboot_ptentry aboot_ptentry;

struct aboot_ptentry {
	char name[ABOOT_PART_NAME_LEN+1];
	int part_index;
	u32 part_blk_start;
	u32 part_blk_num;
	u32 part_blk_size;
	u32 blk_size;
	u32 erase_size;
	u32 flags;
	struct list_head list;
};

/*-------------------------------------------------------------------------*/

/**
 * Register the last OTG notifier
 */
void aboot_register_last_otg_notifier(void);

/**
 * ABOOT is initiated?
 */
int aboot_initiated(void);

/**
 * ABOOT initialization
 */
int aboot_init(void);

/**
 * Inputs:
 *   The partition name
 *
 * Returns:
 *   1. NULL: The specific partition is not found
 *   2. The instance of aboot_ptentry structure of the specific name
 */
aboot_ptentry *aboot_get_part_by_name(const char *name);

/**
 * Inputs:
 *   The partition index
 *
 * Returns:
 *   1. NULL: The specific partition is not found
 *   2. The instance of aboot_ptentry structure of the specific name
 */
aboot_ptentry *aboot_get_emmc_part(int index);

/**
 * Inputs:
 *   The partition index
 *
 * Returns:
 *   1. NULL: The specific partition is not found
 *   2. The instance of aboot_ptentry structure of the specific name
 */
aboot_ptentry *aboot_get_nand_part(int index);

/**
 * Print the current aboot partitions to the console
 */
void aboot_print_parts(void);

/**
 * Start to boot Linux kernel
 */
void aboot(void);

int aboot_sdcard_fat_boot(int dev, int part, const char* filename);
int aboot_mmc_boot(int dev, const char *part_name);

/**
 * Boot the specific Linux kernel from memory
 */
void do_aboot_linux(const char* bootargs, ulong kernel_entry, ulong initrd_entry, ulong initrd_size);

/*-------------------------------------------------------------------------*/

/* Boot Reason (InnoComm Proprietary Feature) */
#define ABOOT_REASON_POWEROFF		0x55667700	/* Power off the device */
#define ABOOT_REASON_NORMAL			0x55667701	/* Normal Android Boot */
#define ABOOT_REASON_CHARGER		0x55667702	/* Power-off Charging */
#define ABOOT_REASON_RECOVERY		0x55667703	/* Android Recovery Boot */
#define ABOOT_REASON_FASTBOOT		0x55667704	/* Android Bootloader Fastboot */
#define ABOOT_REASON_ALARM			0x55667705	/* Power-on Alarm */
#define ABOOT_REASON_PCBA			0x55667706	/* Android Recovery Boot for PCBA */
#define ABOOT_REASON_PC_CHARGER		0x55667707	/* USB PC Charger */

/**
 * Get the current ABOOT_REASON_XXXXXX
 */
u32 aboot_get_reason(void);

/* Android Boot Mode (Android Official Feature) */
#define ABOOT_MODE_NORMAL			0x77665501	/* Android Normal Mode */
#define ABOOT_MODE_FACTORY			0x77665502	/* Android Factory Mode */
#define ABOOT_MODE_CHARGER			0x77665503	/* Android Power-off Charger Mode */

/**
 * Get the current ABOOT_REASON_XXXXXX
 */
void aboot_set_mode(u32 mode);

/**
 * Get the current ABOOT_MODE_XXXXXX
 */
u32 aboot_get_mode(void);

/*-------------------------------------------------------------------------*/

enum aboot_state {
	ABOOT_STATE_TRICKLE_CHARGING,
	ABOOT_STATE_PRECHARGING,
	ABOOT_STATE_NOT_CHARGING,
	ABOOT_STATE_CHARGING_TIMEDOUT,
	ABOOT_STATE_VBAT_LOW,
	ABOOT_STATE_BOOT_FASTBOOT,
	ABOOT_STATE_BOOT_RECOVERY,
	ABOOT_STATE_BOOT_CHARGER,
	ABOOT_STATE_BOOT_ANDROID,
	ABOOT_STATE_REBOOT,
	ABOOT_STATE_POWEROFF,
};

/**
 * Inputs:
 *   the current aboot state (ABOOT_STATE_XXXXXX)
 */
void aboot_set_state(int state);

/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/
/* Board-level Implementations                                             */
/*-------------------------------------------------------------------------*/

/**
 * Inputs:
 *   state - the current aboot state (ABOOT_STATE_XXXXXX)
 *   vbat - the current battery voltage (mV)
 */
void aboot_board_set_state(int state, int vbat);

/**
 * Outputs:
 *   The partitions
 *
 * Returns:
 *   The number of the partitions
 */
int aboot_board_get_parts(aboot_partition **parts);

/**
 * Inputs:
 *   1. mmc: eMMC device
 *   2. name: partition name
 *   3. blk_offset: the current block offset for this specific partition
 *
 * Returns:
 *   The total block size of this partition
 */
u32 aboot_board_get_part_size(struct mmc *mmc, const char *name, u32 blk_offset);

/**
 * Return the serial number of the device
 */
const char *aboot_board_get_serialno(void);

/**
 * Return the Bootloader unlocked status of the device
 */
int aboot_board_is_unlocked(void);

/**
 * Returns ABOOT_REASON_XXXXXX for the booting
 */
u32 aboot_board_get_poweron_reason(int *is_cold_reset);

/*-------------------------------------------------------------------------*/

#endif /* _ABOOT_H */
