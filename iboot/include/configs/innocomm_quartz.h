/*
 * (C) Copyright 2012
 * InnoComm Mobile Technology Corp.
 * Jiahe Jou <jiahe.jou@innocomm.com>
 * James Wu <james.wu@innocomm.com>
 *
 * Configuration settings for the InnoComm OMAP4 Quartz board.
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

#ifndef __CONFIG_INNOCOMM_QUARTZ_H
#define __CONFIG_INNOCOMM_QUARTZ_H

/*
 * High Level Configuration Options
 */
#define CONFIG_SYS_BOARD_NAME		"quartz"
#define CONFIG_SYS_IBOOT_VERSION	"QUZXXX02"
#define CONFIG_INNOCOMM_QUARTZ		1		/* working with InnoComm Quartz */
#define CONFIG_MACH_TYPE		9991	/* Board id for Linux */
#define CONFIG_BOARD_DEBUG
/*#define CONFIG_BOARD_VERBOSE_DEBUG*/

#define CONFIG_ANDROID_TRUST_BOOT	/* Android Trust Boot */
#define CONFIG_ANDROID_TRUST_BOOT_RSA_PRIVATE_KEY	"quartz.pem"

#if 0
/* OMAP44xx MPU at OPPTURBO */
#define CONFIG_OMAP44XX_MPU_OPPTURBO
#else
/* OMAP44xx MPU at OPP50 */
#define CONFIG_OMAP44XX_MPU_OPP50
#endif
/*#define CONFIG_OMAP44XX_CORE_233MHZ*/	/* OMAP4470 CORE at 233MHz */

#if 0
#define	CONFIG_OMAP4_PRCM_DEBUG		/* OMAP4 PRCM debug commands: prcm & prcmon */
#endif

/* Don't check the expected SDRAM size */
#ifndef CONFIG_SYS_SDRAM_EXPECTED_SIZE
#define CONFIG_SYS_SDRAM_EXPECTED_SIZE	0
#endif

/* TWL6032 PMIC configurations */
#define CONFIG_SYS_I2C_SPEED		400000 /* 400Kbps */
#if 0
#define CONFIG_OMAP4_PMIC_SPEED		400 /* Kbps */
#endif
#define CONFIG_TWL6030_MSECURE_GPIO	6
#define CONFIG_TWL6030_PWRBUTTON_TURNON	/* Cold reset by the power button */

/* TWL6040 & Vibrator */
#if 0 /* no Vibrator */
#define CONFIG_SYS_VIBRATOR
#define CONFIG_TWL6040_VIBRATOR
#define CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED		0x36	/* n x 50mV = 54 x 50mV = 2700mV */
/*#define CONFIG_TWL6040_VIB_DEBUG*/
/*#define CONFIG_TWL6040_VIB_VERBOSE_DEBUG*/
#endif

/* Display CPU and Board Info */
#define CONFIG_DISPLAY_CPUINFO		1
/*#define CONFIG_DISPLAY_BOARDINFO*/

#ifndef CONFIG_SPL_BUILD

/*
 * Display configurations
 */
/* Don't show the bootlogo on Fastboot for Display developments (DEBUG purpose) */
/*#define CONFIG_ABOOT_DISPLAY_DEV_FASTBOOT_NO_BOOTLOGO*/
#define CONFIG_OMAP2_DSS
#define CONFIG_OMAP2_FIFO_LOW_THRESHOLD		3
#define CONFIG_OMAP4_DSS_FIFO_WORKAROUND
/*#define CONFIG_OMAP4_DSS_FIFO_WORKAROUND_DEBUG*/
/*#define CONFIG_OMAP4_DSS_FIFO_UNDERFLOW_WORKAROUND*/
/*#define CONFIG_OMAP2_DSS_DEBUG_SUPPORT*/
/*#define CONFIG_OMAP2_DSS_VERBOSE_DEBUG_SUPPORT*/
/* Don't enable DSS debug on Kernel */
/*#define CONFIG_OMAP2_DSS_KERNEL_DEBUG_SUPPORT*/
#define CONFIG_OMAP2_DSS_DSI
#define CONFIG_OMAP2_VRAM_SIZE				16 /* MB */
#define CONFIG_SYS_WHITE_ON_BLACK			/* BLACK BG & WHITE FG */
/*#define CONFIG_VIDEO_TEST_PATTERN*/
#define CONFIG_VIDEO_BOOTLOGO_MAX_SIZE		524288 /* Bytes */
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE		(1920*1080*3 + 1024/*Header*/)
/* Display Panel */
#define CONFIG_PANEL_TC358765				/* Toshiba TC358765 DSI-to-LVDS chip */
#define CONFIG_TC358765_I2C_BUS				1 /* I2C2 */
/*#define CONFIG_TC358765_DEBUG*/
/* Display Backlight */
#define CONFIG_VIDEO_DEFAULT_BRIGHTNESS		30	/* 1~100 */
#define CONFIG_SYS_PWM
#define CONFIG_TWL6030_PWM
/*#define CONFIG_TWL6030_PWM_DEBUG*/
/*#define CONFIG_TWL6030_PWM_VERBOSE_DEBUG*/

/*
 * Fastboot setup & configurations
 */
#define CONFIG_USB_DMA_MODE				/* Enable USB DMA mode */
#define CONFIG_USB_DEBUG			0	/* USB Deubg Level: 0=> disabled; 1~10=> enabled */
/*#define CONFIG_FASTBOOT_DEBUG*/
/*#define CONFIG_FASTBOOT_VERBOSE_DEBUG*/
#define CONFIG_CMD_FASTBOOT	/* fastboot commands */

/*
 * Power Supply & Charger configurations
 */
#define CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY		/* Board has no main battery */
#define CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV	3000
#define CONFIG_POWER_SUPPLY_DEBUG
/*#define CONFIG_POWER_SUPPLY_VERBOSE_DEBUG*/

#endif /* !CONFIG_SPL_BUILD */

/*
 * Console configurations
 */
/*#define CONFIG_PRE_CONSOLE_BUFFER*/

/*
 * SPL dependant configurations
 */
#define CONFIG_SPL_FAT_SUPPORT		/* Support MMC1 SD card booting in SPL */
#define CONFIG_SPL_USB_SUPPORT		/* Support USB booting in SPL */

#include <configs/innocomm_omap4_1024m.h>

#undef CONFIG_BOOTDELAY

#define CONFIG_SYS_PROMPT			"Quartz# "

/*
 * Environment setup
 */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		1	/* eMMC(1) */
#define CONFIG_ENV_OFFSET			0x200000	/* in byte offset (block offset is 0x1000) */
#define CONFIG_CMD_SAVEENV
#define CONFIG_USE_BACKUP_ENV
#define CONFIG_BACKUP_ENV_OFFSET	0x280000	/* in byte offset (block offset is 0x1400) */
#define CONFIG_BOARD_HAS_CMDLINE_ANDROID_CARRIER	/* androidboot.carrier= */
#define CONFIG_BOARD_HAS_CMDLINE_ANDROID_ETHADDR	/* androidboot.ethaddr= */

#ifndef CONFIG_SPL_BUILD

/* I2C  */
#define CONFIG_I2C_MULTI_BUS
/*#define CONFIG_SYS_I2C_OMAP_BOARD_INIT*/
/*#define CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN*/
#define CONFIG_SYS_I2C2_SPEED		400000
#define CONFIG_SYS_I2C3_SPEED		400000
#define CONFIG_SYS_I2C4_SPEED		400000

#endif /* !CONFIG_SPL_BUILD */

/* Keypad */
#define CONFIG_SYS_KEYPAD_ROWS			1
#define CONFIG_SYS_KEYPAD_COLS			2
/*#define CONFIG_KEYPAD_DEBUG*/
/*#define CONFIG_KEYPAD_VERBOSE_DEBUG*/

/*
 * ABOOT configurations
 */
/* PCBA uses Fastboot not Android recovery */
#define CONFIG_ABOOT_PCBA_USE_FASTBOOT
/* The keys to enter Fastboot: Volume Down + POWER */
#define CONFIG_ABOOT_FASTBOOT_ROWBITS					0x101
#define CONFIG_ABOOT_FASTBOOT_COLBITS					0x102
/* The keys to enter Android Recovery: Volume Up + POWER */
#define CONFIG_ABOOT_RECOVERY_ROWBITS					0x101
#define CONFIG_ABOOT_RECOVERY_COLBITS					0x101
/* Don't power on the device if the last turn off was due to a long POWER button */
/*#define CONFIG_ABOOT_NO_DEVOFF_LPK_POWERON*/
/* Don't support Android charger mode (Android power-off charging) */
#define CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE
/* Don't support the power-off charging by USB PC charger */
#define CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER
/* Don't allow USB PC charger to power on the device even if VBAT is high enough */
#define CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON

#ifndef CONFIG_SPL_BUILD
/*
 * SPL independant configurations
 */

/*
 * ABOOT SPL independant configurations
 */
/* Minimal boot-up voltage when the system is not charging */
#define CONFIG_ABOOT_MINIMAL_VOLTAGE					3500	/* mV */
/* The vibrating time for the COLD reset */
#define CONFIG_ABOOT_POWERON_VIBRATION_TIME				70		/* ms */
/* eMMC device number */
#define CONFIG_ABOOT_EMMC_DEVICE_NO						1
/*#define CONFIG_ABOOT_STATE_DEBUG*/
/*#define CONFIG_ABOOT_DEBUG*/
/*#define CONFIG_ABOOT_VERBOSE_DEBUG*/
#define CONFIG_CMD_ABOOT	/* aboot commands */

#else /* CONFIG_SPL_BUILD */
/*
 * SPL dependant configurations
 */

#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	0x400 /* 512KB */
#define CONFIG_SYS_U_BOOT_REPEAT_NUM		3

#endif /* !CONFIG_SPL_BUILD */

/*
 * Special configurations
 */
#undef CONFIG_SYS_ALLOW_UBOOT_BIN				/* NOT allow to boot/flash u-boot.bin */
#undef CONFIG_SYS_ALLOW_INCORRECT_BOARD_NAME	/* NOT allow to boot/flash incorrect board name */

#endif /* __CONFIG_INNOCOMM_QUARTZ_H */
