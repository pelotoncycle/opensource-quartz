/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * Common configuration settings for
 * InnoComm OMAP4 boards with 1024MB LPDDR2 SDRAM.
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

#ifndef __CONFIG_INNOCOMM_COMMON_H
#define __CONFIG_INNOCOMM_COMMON_H

/* Enable ARM Thumb/Thumb2 mode build */
#define CONFIG_SYS_THUMB_BUILD

/*
 * High Level Configuration Options
 */
#define CONFIG_OMAP				1	/* in a TI OMAP core */
#define CONFIG_OMAP44XX			1	/* which is a 44XX */
#define CONFIG_TWL6030_POWER	1	/* TWL6030 */
#define CONFIG_TWL6032_POWER	1	/* TWL6032 PMIC */
#define CONFIG_OMAP_GPIO

/*
 * Android Trust Boot configurations
 */
#define CONFIG_ANDROID_MINCRYPT_SHA
#ifdef CONFIG_ANDROID_TRUST_BOOT
#define CONFIG_ANDROID_MINCRYPT_RSA
#endif /* CONFIG_ANDROID_TRUST_BOOT */

/* Get CPU defs */
#include <asm/arch/cpu.h>
#include <asm/arch/omap.h>

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MISC_INIT_R
#define	CONFIG_PANIC_HANG

/* Clock Defines */
#define V_OSCK		38400000	/* Clock output from T2 */
#define V_SCLK		V_OSCK

#undef CONFIG_USE_IRQ				/* no support for IRQs */

/* Kernel ATAGs */
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SERIAL_TAG
#define CONFIG_VIDEOLFB_TAG

/*
 * Size of malloc() pool
 * Total Size Environment - 256K
 * Malloc - 8192K
 */
#define CONFIG_ENV_SIZE			(256 << 10)
#define CONFIG_SYS_MALLOC_LEN		(8192 << 10)
/* Vector Base */
#define CONFIG_SYS_CA9_VECTOR_BASE	SRAM_ROM_VECT_BASE

/*
 * Hardware drivers
 */

/*
 * USB OTG configurations
 */
#ifndef CONFIG_USB_OTG_POLL_TIME
#define CONFIG_USB_OTG_POLL_TIME		1200	/* USB OTG transceiver polling time in ms */
#endif

/*
 * Console configurations
 */
#define CONFIG_SYS_CONSOLE_INFO_QUIET
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_ENV_OVERWRITE			/* allow to overwrite serial and ethaddr */
#ifndef CONFIG_CONS_INDEX
#define CONFIG_CONS_INDEX		3		/* 1~3: UART# on Board */
#endif /* CONFIG_CONS_INDEX */

/*
 * serial port - NS16550 compatible
 */
#define V_NS16550_CLK			48000000
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{115200}
#if (CONFIG_CONS_INDEX == 1)
#define CONFIG_SYS_NS16550_COM1		UART1_BASE
#elif (CONFIG_CONS_INDEX == 2)
#define CONFIG_SYS_NS16550_COM2		UART2_BASE
#elif (CONFIG_CONS_INDEX == 3)
#define CONFIG_SYS_NS16550_COM3		UART3_BASE
#else
#error "Please define CONFIG_CONS_INDEX\n"
#endif

/* I2C  */
#define CONFIG_HARD_I2C				1
#ifndef CONFIG_SYS_I2C_SPEED
#define CONFIG_SYS_I2C_SPEED		400000 /* 400Kbps */
#if 0
#define CONFIG_OMAP4_PMIC_SPEED		400 /* Kbps */
#endif
#endif
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS			0
#define CONFIG_SYS_I2C_BUS_SELECT	1
#define CONFIG_DRIVER_OMAP34XX_I2C	1
/*#define CONFIG_I2C_MULTI_BUS		1*/

/*
 * Power Supply & Charger configurations
 */
#define CONFIG_SYS_POWER_SUPPLY
#if defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY) && defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY)
#error "Conflict definitions for No Main Battery Feature"
#endif
/* PMIC power-off voltage */
#ifndef CONFIG_POWER_SUPPLY_POWEROFF_VOLTAGE
/* TWL6032A1B6 VSYSMIN_LO_MAX (BOOT0 = 0) */
#define CONFIG_POWER_SUPPLY_POWEROFF_VOLTAGE	2750 /* mV */
#endif
/* PMIC power-on voltage */
#ifndef CONFIG_POWER_SUPPLY_POWERON_VOLTAGE
/* TWL6032A1B6 VSYSMIN_HI_MAX (BOOT0 = 0) */
#define CONFIG_POWER_SUPPLY_POWERON_VOLTAGE		3200 /* mV */
#endif
/* Treat USB carkit as the USB AC charger */
/*#define CONFIG_POWER_SUPPLY_TREAT_USB_CARKIT_AS_AC_CHARGER*/
#ifndef CONFIG_POWER_SUPPLY_POLL_MS_TIME
/* TWL6032 Charger has a 127-second watchdog timer */
#define CONFIG_POWER_SUPPLY_POLL_MS_TIME				(CONFIG_USB_OTG_POLL_TIME * 50) /* ms */
#endif
#ifndef CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV	2500 /* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY
#define CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY	20 /* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV	2200 /* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_AC_CHARGE_CURRENTMA
#define CONFIG_POWER_SUPPLY_USB_AC_CHARGE_CURRENTMA		1200 /* mA */
#endif

/*
 * TWL6032 Charger Configurations
 */
#define CONFIG_TWL6032_CHARGER
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#define CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER	/* Board has no internal TWL6032 charger functionality */
#endif
#ifndef CONFIG_TWL6032_CHARGER_SENSE_RESISTOR
#define CONFIG_TWL6032_CHARGER_SENSE_RESISTOR	15 /* mOhm */
#endif
#ifndef CONFIG_TWL6032_CHARGER_PROFILES
#define CONFIG_TWL6032_CHARGER_PROFILES { \
/* volt, input current, charge current, increase_volt, decrease_volt */ \
	{0,		1800,	100,	0,		0},		\
	{2120,	1800,	100,	20,		20},	\
	{2420,	1800,	300,	100,	20},	\
	{2520,	1800,	400,	200,	100},	\
	{2720,	1800,	400,	170,	200},	\
	{2890,	1800,	400,	200,	170},	\
	{3090,	1800,	300,	80,		200},	\
	{3170,	1800,	200,	80,		80},	\
	{3300,	1800,	500,	80,		80},	\
	{3380,	1800,	1200,	50,		80},	\
	{9999,	1800,	1200,	0,		0},		\
}
#endif

/*
 * BQ2416x Charger configurations
 */
#ifdef CONFIG_CHARGER_BQ2416x
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER) && !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)
#define CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER	/* Board has the external AC charger IC */
#endif
#ifndef CONFIG_POWER_SUPPLY_ENABLE_CHARGE_TERMINATION
#define CONFIG_POWER_SUPPLY_ENABLE_CHARGE_TERMINATION
#endif
/* BQ24161 has a 30-second watchdog timer */
#undef CONFIG_POWER_SUPPLY_POLL_MS_TIME
#define CONFIG_POWER_SUPPLY_POLL_MS_TIME	(CONFIG_USB_OTG_POLL_TIME * 18) /* 1.2s * 18 = 21.6s */
/*#define CONFIG_CHARGER_BQ2416x_I2C_BUS	0*/
/*#define CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY*/	/* BQ2416x has USB supply */
/*#define CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY*/	/* BQ2416x has AC IN supply */
#ifndef CONFIG_CHARGER_BQ2416x_PROFILES
#define CONFIG_CHARGER_BQ2416x_PROFILES { \
/* volt, input current, charge current, use_half, increase_volt, decrease_volt */ \
	{0,		1500,	550,	0,	0,		0},		\
	{2420,	1500,	550,	0,	150,	150},	\
	{2680,	1500,	925,	0,	250,	150},	\
	{3400,	1500,	1225,	0,	350,	250},	\
	/*{4100,	1500,	550,	1,	150,	350},*/	\
	{9999,	1500,	1225,	0,	0,		0},		\
}
#endif
/*#define CONFIG_CHARGER_BQ2416x_DEBUG*/
/*#define CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG*/
#endif /* CONFIG_CHARGER_BQ2416x */

/*
 * BQ27541 Fuel Gauge configurations
 */
#ifdef CONFIG_FUELGAUGE_BQ27541
#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
#define CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE	/* Board has the external Fuel Gauge */
#endif
#ifndef CONFIG_FUELGAUGE_BQ27541_MIN_VBAT
#define CONFIG_FUELGAUGE_BQ27541_MIN_VBAT	2450 /* mV */
#endif
#ifndef CONFIG_FUELGAUGE_BQ27541_MAX_VBAT
#define CONFIG_FUELGAUGE_BQ27541_MAX_VBAT	4350 /* mV */
#endif
/*#define CONFIG_FUELGAUGE_BQ27541_I2C_BUS	3*/
/*#define CONFIG_FUELGAUGE_BQ27541_DEBUG*/
/*#define CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG*/
#endif /* CONFIG_FUELGAUGE_BQ27541 */

/*
 * BQ27520 Fuel Gauge configurations
 */
#ifdef CONFIG_FUELGAUGE_BQ27520
#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
#define CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE	/* Board has the external Fuel Gauge */
#endif
#ifndef CONFIG_FUELGAUGE_BQ27520_MIN_VBAT
#define CONFIG_FUELGAUGE_BQ27520_MIN_VBAT	2450 /* mV */
#endif
#ifndef CONFIG_FUELGAUGE_BQ27520_MAX_VBAT
#define CONFIG_FUELGAUGE_BQ27520_MAX_VBAT	4350 /* mV */
#endif
/*#define CONFIG_FUELGAUGE_BQ27520_I2C_BUS	3*/
/*#define CONFIG_FUELGAUGE_BQ27520_DEBUG*/
/*#define CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG*/
#endif /* CONFIG_FUELGAUGE_BQ27520 */

/*
 * TWL6032 configurations
 */
/* TWL6032 VSYSLOW delay. Minimum: delay x 40 ms + 20 ms; Maximum: delay x 40 ms + 30 ms */
#ifndef CONFIG_TWL6032_VSYSLOW_DELAY_MS
#define CONFIG_TWL6032_VSYSLOW_DELAY_MS		120 /* ms */
#endif

/* TWL6040 */
#define CONFIG_TWL6040_POWERON_GPIO		127

/* Keypad */
#define CONFIG_SYS_KEYPAD
#define CONFIG_SYS_KEYPAD_ROW_MASKS		0x0FF
#define CONFIG_SYS_KEYPAD_COL_MASKS		0x0FF
#define CONFIG_KEYPAD_PWRBUTTON_ROW		0x100
#define CONFIG_KEYPAD_PWRBUTTON_COL		0x100
#define CONFIG_OMAP4_KEYPAD

/* MMC */
#define CONFIG_GENERIC_MMC			1
#define CONFIG_MMC					1
#define CONFIG_OMAP_HSMMC			1
#define CONFIG_DOS_PARTITION		1

/* FAT filesystem */
#define CONFIG_SYS_MINI_FAT	/* Minimal FAT filesystem support */

/* Flash */
#define CONFIG_SYS_NO_FLASH

/* Enabled commands */
#define	CONFIG_CMD_BOOTD	/* bootd						*/
#undef	CONFIG_CMD_RUN		/* run command in env variable	*/
#define	CONFIG_CMD_FAT		/* FAT command support			*/
#undef	CONFIG_CMD_MMC		/* MMC support                  */
#undef	CONFIG_CMD_I2C		/* no I2C command support		*/

#ifndef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	2
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"stdout=serial\0" \
	"stdin=serial\0" \
	"stderr=serial\0"

#define CONFIG_BOOTCOMMAND \
	"boot"

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_NO_REPEATABLE_COMMAND /* no repeatable command feature */
#undef	CONFIG_SYS_LONGHELP		/* undef to save memory */
#undef	CONFIG_SYS_HUSH_PARSER	/* don't use "hush" command parser to save memory */
#define CONFIG_SYS_CBSIZE		512
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		(CONFIG_SYS_CBSIZE)

/*
 * memtest setup
 */
#define CONFIG_SYS_MEMORY_POST
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (32 << 20))

/* Default load address */
#define CONFIG_SYS_LOAD_ADDR		0x80000000

/* Use General purpose timer 1 */
#define CONFIG_SYS_TIMERBASE		GPT2_BASE
#define CONFIG_SYS_PTV			2	/* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/*
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#ifndef CONFIG_STACKSIZE
#define CONFIG_STACKSIZE	(1024 << 10)	/* Regular stack */
#endif
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4 << 10)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4 << 10)	/* FIQ stack */
#endif

/*
 * SDRAM Memory Map
 * Even though we use two CS all the memory
 * is mapped to one contiguous block
 */
#define CONFIG_NR_DRAM_BANKS	1

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#ifndef CONFIG_SYS_SDRAM_EXPECTED_SIZE
#define CONFIG_SYS_SDRAM_EXPECTED_SIZE	0x40000000	/* SZ_1G */
#endif
#if 0
/* James Wu: It's safe to use 0x4030E000 only if we are not using ARM exceptions & ROM code API in SPL */
#define CONFIG_SYS_INIT_RAM_ADDR	0x4030D800
#define CONFIG_SYS_INIT_RAM_SIZE	0x800
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
					 CONFIG_SYS_INIT_RAM_SIZE - \
					 GENERATED_GBL_DATA_SIZE)
#else
#define CONFIG_SYS_INIT_SP_ADDR		(LOW_LEVEL_SRAM_STACK - GENERATED_GBL_DATA_SIZE)
#endif

#if 0
/* Turn off Data Cache */
#define CONFIG_SYS_DCACHE_OFF
/* Turn off L2 Cache */
#define CONFIG_SYS_L2CACHE_OFF
#endif
#ifndef CONFIG_SYS_L2CACHE_OFF
#define CONFIG_SYS_L2_PL310		1
#define CONFIG_SYS_PL310_BASE	0x48242000
#endif
#define CONFIG_SYS_CACHELINE_SIZE	32

/* Defines for SDRAM init */
#undef	CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS
#define CONFIG_SYS_AUTOMATIC_SDRAM_DETECTION
#define CONFIG_SYS_DEFAULT_LPDDR2_TIMINGS

/* Defines for SPL */
#define CONFIG_SPL
#define CONFIG_SPL_BOARD_INIT
#ifdef CONFIG_OMAP_HS
#define CONFIG_SPL_TEXT_BASE		0x40304350
#define CONFIG_SPL_MAX_SIZE		(31950)
#else
#define CONFIG_SPL_TEXT_BASE		0x40300000
#define CONFIG_SPL_MAX_SIZE		(42 * 1024)
#endif /* CONFIG_OMAP_HS */
#define CONFIG_SPL_STACK		LOW_LEVEL_SRAM_STACK

/*
 * 64 bytes before this address should be set aside for u-boot.img's
 * header. That is 80E7FFC0--0x80E80000 should not be used for any
 * other needs.
 */
#define CONFIG_SYS_TEXT_BASE		0x80E80000

/*
 * BSS and malloc area 64MB into memory to allow enough
 * space for the kernel at the beginning of memory
 */
#define CONFIG_SPL_BSS_START_ADDR	0x84000000
#define CONFIG_SPL_BSS_MAX_SIZE		0x100000	/* 1 MB */
#if 0
#define CONFIG_SYS_SPL_MALLOC_START	0x84100000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x100000	/* 1 MB */
#else
#define CONFIG_SPL_FAT_BLOCK_ADDR		0x84100000	/* Max 512KB */
#define CONFIG_SPL_FAT_BUFFER_ADDR		0x84180000	/* Max 512KB */
#define CONFIG_SPL_FAT_TMP_BUFFER_ADDR	0x84200000	/* Max 512KB */
#endif

#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x400 /* address 0x80000 */
#define CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION	1
#define CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME	"iboot.img"

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#ifdef CONFIG_SPL_FAT_SUPPORT
#define CONFIG_SPL_LIBDISK_SUPPORT
#ifdef CONFIG_SPL_BUILD
#define CONFIG_PARTITIONS
#endif /* CONFIG_SPL_BUILD */
#endif /* CONFIG_SPL_FAT_SUPPORT */
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_INPUT_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_POWER_SUPPORT
#define CONFIG_SPL_LDSCRIPT "$(CPUDIR)/omap-common/u-boot-spl.lds"

/*
 * SPL independant configurations
 */
#ifndef CONFIG_SPL_BUILD

#define CONFIG_USE_ARCH_MEMSET
#define CONFIG_USE_ARCH_MEMCPY

/*
 * Display configurations
 */
#if defined(CONFIG_OMAP2_DSS)
/* OMAP DSS */
#define	CONFIG_OMAP2_VRAM
#ifndef CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK
#define CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK	2
#endif
#define	CONFIG_FB_OMAP2_NUM_FBS				1
/* CFB_CONSOLE */
#define	CONFIG_CFB_CONSOLE
#define	CONFIG_VIDEO
#define CONFIG_VIDEO_LINE_STRIDE
#define	CONFIG_VIDEO_FLUSH
#define	CONFIG_VIDEO_BRIGHTNESS
#ifdef CONFIG_VIDEO_AS_CONSOLE_DEVICE
/* Use LCD as the console device (DEBUG purpose) */
#define CONFIG_VGA_AS_SINGLE_DEVICE
#undef CONFIG_VIDEO_NOT_CONSOLE_DEVICE
#undef CONFIG_SYS_CONSOLE_INFO_QUIET
#undef CONFIG_SYS_CONSOLE_IS_IN_ENV
#else
/* Don't use LCD as the console device */
#define	CONFIG_VIDEO_NOT_CONSOLE_DEVICE
#endif
#define	CONFIG_VIDEO_BOOTLOGO
#define	CONFIG_VIDEO_BOOTLOGO_ALIGN
#ifndef CONFIG_VIDEO_BOOTLOGO_IMAGE_ADDR
#define	CONFIG_VIDEO_BOOTLOGO_IMAGE_ADDR	(CONFIG_SYS_SDRAM_BASE + 0x00600000)
#endif
#ifndef CONFIG_VIDEO_BOOTLOGO_HEIGHT
#define	CONFIG_VIDEO_BOOTLOGO_HEIGHT		0
#endif
#define	CONFIG_VIDEO_BMP_GZIP
#ifndef CONFIG_SYS_VIDEO_LOGO_ADDR
#define	CONFIG_SYS_VIDEO_LOGO_ADDR			(CONFIG_SYS_SDRAM_BASE + 0x00900000)
#endif
#ifndef CONFIG_SYS_VIDEO_LOGO_MAX_SIZE
#define	CONFIG_SYS_VIDEO_LOGO_MAX_SIZE		(1920*1080*3 + 1024/*Header*/)
#endif
#endif /* CONFIG_OMAP2_DSS */

/*
 * ABOOT configurations
 */
/* Default Android Recovery ZIP filename (on SD card) */
#ifndef CONFIG_ABOOT_SDCARD_RECOVERY_FILENAME
#define CONFIG_ABOOT_SDCARD_RECOVERY_FILENAME	"update.zip"
#endif
#define CONFIG_SYS_64BIT_LBA
#define CONFIG_EFI_PARTITION
#ifndef CONFIG_ABOOT_WAIT_FOR_CHARGING_CONTINUE_BOOT_COUNT
#define CONFIG_ABOOT_WAIT_FOR_CHARGING_CONTINUE_BOOT_COUNT	3
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_CHARGING_POLL_TIME
#define CONFIG_ABOOT_WAIT_FOR_CHARGING_POLL_TIME		(CONFIG_USB_OTG_POLL_TIME * 8)	/* 1.2s * 8 = 9.6s */
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_WAKEUP_VOLTAGE_TIMES
#define CONFIG_ABOOT_WAIT_FOR_WAKEUP_VOLTAGE_TIMES		2250 /* 9.6s * 2250 = 21600s = 360 mins */
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_CHARGING_TIMES
#define CONFIG_ABOOT_WAIT_FOR_CHARGING_TIMES			2250 /* 9.6s * 2250 = 21600s = 360 mins */
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES
#define CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES		2250 /* 9.6s * 2250 = 21600s = 360 mins */
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_EXTERNAL_FUELGAUGE_TIMES
#define CONFIG_ABOOT_WAIT_FOR_EXTERNAL_FUELGAUGE_TIMES	2250 /* 9.6s * 2250 = 21600s = 360 mins */
#endif
#ifndef CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES
#define CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES		600  /* 9.6s * 600  = 5760s   = 96 mins */
#endif
/* Minimal voltage to vibrate the vibrator */
#ifndef CONFIG_ABOOT_POWERON_VIBRATION_MINIMAL_VOLTAGE
#define CONFIG_ABOOT_POWERON_VIBRATION_MINIMAL_VOLTAGE	2950 /* mV */
#endif
/* Define the waiting time for VBAT LOW logo before shutting down */
#ifndef CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME
#define CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME		500 /* ms */
#endif
/* Minimal voltage to show the VBAT LOW logo if there is no charger */
#ifndef CONFIG_ABOOT_VBAT_LOW_LOGO_MINIMAL_VOLTAGE
#define CONFIG_ABOOT_VBAT_LOW_LOGO_MINIMAL_VOLTAGE	3350 /* mV */
#endif
/*#define CONFIG_ABOOT_DUMP_CHARGING_REGS*/
/* PCBA uses Fastboot not Android recovery */
/*#define CONFIG_ABOOT_PCBA_USE_FASTBOOT*/
/* Timer to wait for the factory tool when the device is booted from SD card or USB */
#define CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME	10 /* x50ms */
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER) || defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)
	#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
		#ifdef CONFIG_ABOOT_POWERON_CHECK_BATTERY
			#error "Don't define CONFIG_ABOOT_POWERON_CHECK_BATTERY for CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY"
		#endif
		#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
			#error "Don't define CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY if no external AC charger"
		#endif
		/* Define to show "<No Main Battery>" text on LCD */
		#define CONFIG_ABOOT_SHOW_NO_MAIN_BATTERY_INFO
	#else /* !CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
		#ifdef CONFIG_ABOOT_POWERON_CHECK_BATTERY
			#error "Don't define CONFIG_ABOOT_POWERON_CHECK_BATTERY in Device's config file"
		#endif
		/* Don't power on the device if there is no main battery */
		#define CONFIG_ABOOT_POWERON_CHECK_BATTERY
	#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
#else /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER || CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER */
	#ifdef CONFIG_ABOOT_POWERON_CHECK_BATTERY
		#error "Don't define CONFIG_ABOOT_POWERON_CHECK_BATTERY if no external charger"
	#endif
	#ifdef CONFIG_ABOOT_CHARGER_POWERON_CHECK_BATTERY
		#error "Don't define CONFIG_ABOOT_CHARGER_POWERON_CHECK_BATTERY if no external charger"
	#endif
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER || CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER */
/* Bypass the Battery Temp Fault while checking the power-on voltage and there is no external FG */
/*#define CONFIG_ABOOT_POWERON_SKIP_BAT_TEMP_FAULT*/
/* Don't support the power-off charging by USB PC charger */
/*#define CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER*/
/* Don't allow USB PC charger to power on the device even if VBAT is high enough */
/*#define CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON*/
/* Don't power on the device if the last turn off was due to a long POWER button */
/*#define CONFIG_ABOOT_NO_DEVOFF_LPK_POWERON*/
/* Don't power on the device if the reset reason is CPU Watchdog timer reset */
/*#define CONFIG_ABOOT_NO_CPU_WATCHDOG_RESET_POWERON*/
/* Don't support Android charger mode (Android power-off charging) */
/*#define CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE*/
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) \
    || defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY) \
    || !defined(CONFIG_TWL6030_PWRBUTTON_TURNON)
/* Don't allow USB OTG to power on the device even if VBAT is high enough */
#ifndef CONFIG_ABOOT_NO_USB_OTG_POWERON
#define CONFIG_ABOOT_NO_USB_OTG_POWERON
#endif
#endif

/*
 * Fastboot setup & configurations
 */
/* Output the specific Fastboot Debug messages to video console: 0=>disabled; >0=>enabled */
#if !defined(CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG) && !defined(CONFIG_ABOOT_HAS_FASTBOOT_LOGO)
#define CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG		1
#endif
/* CONFIG_SYS_SDRAM_BASE + SZ_16M */
#define CONFIG_FASTBOOT_BUFFER		(CONFIG_SYS_SDRAM_BASE + 0x1000000)
#ifndef CONFIG_FASTBOOT_BUFFER_SIZE
/* ram_size - SZ_16M - SZ_64M */
#define CONFIG_FASTBOOT_BUFFER_SIZE	(gd->ram_size - 0x05000000)
#endif /* !CONFIG_FASTBOOT_BUFFER_SIZE */
#define CONFIG_OMAP4_HSOTG_ED_CORRECTION	/* Improved HSOTG USB EYE diagram */
#define CONFIG_USB_OTG_TRANSCEIVER		/* USB OTG transceiver */
#define CONFIG_USB_GADGET				/* USB Gadget */
#define CONFIG_USB_GADGET_MUSB_HDRC		/* MUSB HDRC USB Peripheral */
#ifdef CONFIG_USB_DMA_MODE
#define CONFIG_USB_INVENTRA_DMA			/* DMA transfers using Mentor's engine */
#define CONFIG_USB_INVENTRA_DMA_MODE1	/* Support MUSB DMA mode 1 */
#define CONFIG_USB_INVENTRA_DMA_MAX_LEN	CONFIG_FASTBOOT_BUFFER_SIZE
#else
#define CONFIG_MUSB_PIO_ONLY			/* Disable DMA (always use PIO) */
#endif /* CONFIG_USB_DMA_MODE */
#define CONFIG_USB_GADGET_FASTBOOT		/* Fastboot USB Gadget */
#define CONFIG_TWL6030_USB				/* TWL6030 USB Transceiver Driver */
#define CONFIG_USB_GADGET_VBUS_DRAW	500	/* Maximum VBUS Power usage (2-500 mA) */

#endif /* !CONFIG_SPL_BUILD */

#endif /* __CONFIG_INNOCOMM_COMMON_H */
