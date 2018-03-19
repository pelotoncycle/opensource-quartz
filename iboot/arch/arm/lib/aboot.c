/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <common.h>
#include <exports.h>
#include <command.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <errno.h>
#include <watchdog.h>
#include <video_fb.h>
#include <video.h>

#include <part.h>
#include <mmc.h>
#include <fat.h>

#include <linux/usb/otg.h>

#include <icom/fastboot.h>
#include <icom/aboot.h>
#include <icom/vibrator.h>
#include <icom/pwm.h>
#include <icom/power_supply.h>
/* Android mkbootimg format */
#include <icom/bootimg.h>
/* InnoComm mkmultimg format */
#include <icom/multimg.h>

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

#if defined(CONFIG_CFB_CONSOLE) && defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) \
		&& defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG) && (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG > 0)
#define aboot_wait_4_charging_video_printf(level, format, args...) do { \
	if (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG >= (level)) { \
		video_printf(format, ## args); \
	} } while (0)
#define aboot_wait_4_charging_video_puts(level, format) do { \
	if (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG >= (level)) { \
		video_puts(format); \
	} } while (0)
#define ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(level, fmt, args...) \
	aboot_wait_4_charging_video_printf(level, fmt, ## args)
#define ABOOT_WAIT_4_CHARGING_VIDEO_PUTS(level, fmt) \
	aboot_wait_4_charging_video_puts(level, fmt)
#else
#define ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(level, fmt, args...)	do{}while(0)
#define ABOOT_WAIT_4_CHARGING_VIDEO_PUTS(level, fmt)	do{}while(0)
#endif /* CONFIG_CFB_CONSOLE && CONFIG_VIDEO_NOT_CONSOLE_DEVICE && CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG */

/*-------------------------------------------------------------------------*/

#define STRINGIFY(x)	#x
#define EXPAND_STR(x)	STRINGIFY(x)

/*-------------------------------------------------------------------------*/

#define ABOOT_CMDLINE_SPACE_CHAR		" "
#define ABOOT_CMDLINE_LOG_LEVEL			" loglevel="
#define ABOOT_CMDLINE_LOG_LEVEL_QUIET	" quiet"
#define ABOOT_CMDLINE_LOG_LEVEL_DEBUG	" debug"
#define ABOOT_CMDLINE_REASON			" androidboot.reason="
#define		ABOOT_REASON_NORMAL_STR			"normal"
#define		ABOOT_REASON_CHARGER_STR		"charger"
#define		ABOOT_REASON_RECOVERY_STR		"recovery"
#define		ABOOT_REASON_ALARM_STR			"alarm"
#define		ABOOT_REASON_PCBA_STR			"pcba"
#define ABOOT_CMDLINE_MODE				" androidboot.mode="
#define		ABOOT_MODE_NORMAL_STR			"normal"
#define		ABOOT_MODE_FACTORY_STR			"factory"
#define		ABOOT_MODE_FACTORY2_STR			"factory2"
#define		ABOOT_MODE_CHARGER_STR			"charger"
#define ABOOT_CMDLINE_CHARGER_MODE_1_CPU	" maxcpus=1 nr_cpus=1"

/*-------------------------------------------------------------------------*/

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SETUP_MEMORY_TAGS) || \
	defined(CONFIG_CMDLINE_TAG) || \
	defined(CONFIG_INITRD_TAG) || \
	defined(CONFIG_SERIAL_TAG) || \
	defined(CONFIG_VIDEOLFB_TAG) || \
	defined(CONFIG_REVISION_TAG)
static struct tag *params;
#endif

/*-------------------------------------------------------------------------*/

static int ainitiated __attribute__ ((section (".data"))) = 0;
static int aboot_state __attribute__ ((section (".data"))) = ABOOT_STATE_POWEROFF;
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
static int aboot_has_no_main_battery __attribute__ ((section (".data"))) = 0;
#endif

static int aboot_video_initiated __attribute__ ((section (".data"))) = 0;

static int aboot_wait_4_charging __attribute__ ((section (".data"))) = 0;
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
static int aboot_bat_locked __attribute__ ((section (".data"))) = 0;
#endif
static int aboot_wait_4_charging_video_debug_lines __attribute__ ((section (".data"))) = 0;
static int aboot_vbat __attribute__ ((section (".data"))) = 0;
static unsigned aboot_ac_type __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;
static unsigned aboot_usb_type __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;

static int aboot_cold_reset = 0;
static u32 aboot_reason = ABOOT_REASON_NORMAL;
static u32 aboot_mode = ABOOT_MODE_NORMAL;

static aboot_partition *def_parts = NULL;
static int def_parts_num = 0;

/*-------------------------------------------------------------------------*/

static void aboot_video_init(void);

/*-------------------------------------------------------------------------*/

/* board_preboot_os */
static inline void def_preboot_os(void) {}
void board_preboot_os(void) __attribute__((weak, alias("def_preboot_os")));
/* arch_preboot_os */
void arch_preboot_os(void) __attribute__((weak, alias("def_preboot_os")));
/* board_setup_linux_cmdline */
static inline size_t def_setup_linux_cmdline(char *cmdline) {return 0;}
size_t board_setup_linux_cmdline(char *cmdline) __attribute__((weak, alias("def_setup_linux_cmdline")));
/* arch_setup_linux_cmdline */
size_t arch_setup_linux_cmdline(char *cmdline) __attribute__((weak, alias("def_setup_linux_cmdline")));

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
DEFINE_CACHE_ALIGN_BUFFER(unsigned char, aboot_mmc_blk, 512);
#if defined(CONFIG_ABOOT_HAS_MULTI_LOGOS)
static int aboot_mmc_read_multimg(int dev, const char *part_name,
		const char *image_name, void *buffer, unsigned long *size);
#endif
static int aboot_mmc_read_raw(int dev, const char *part_name, void *buffer, unsigned long *size);
#if !defined(CONFIG_ABOOT_PCBA_USE_FASTBOOT) || defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
static int aboot_mmc_write_raw(int dev, const char *part_name, void *buffer, unsigned long *size);
#endif
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

/*-------------------------------------------------------------------------*/

#if 0
/* Sample codes to get SP register */
static ulong get_sp(void)
{
	ulong ret;

	asm("mov %0, sp" : "=r"(ret) : );
	return ret;
}
#endif

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO

/* We only support this feature while using the external AC/USB charger  */
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER) && !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)
#if defined(CONFIG_OMAP44XX)
#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
#error "CONFIG_TWL6032_CHARGER_SENSE_RESISTOR must not be 0"
#endif
#else /* !(CONFIG_OMAP44XX) */
#error "Please define CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER or CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER"
#endif /* CONFIG_OMAP44XX */
#endif

#if !defined(CONFIG_CFB_CONSOLE) || !defined(CONFIG_VIDEO_NOT_CONSOLE_DEVICE) || !defined(CONFIG_VIDEO_BOOTLOGO)
#error "Please define CONFIG_CFB_CONSOLE, CONFIG_VIDEO_NOT_CONSOLE_DEVICE and CONFIG_VIDEO_BOOTLOGO"
#endif

#endif /* CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO */

#ifdef CONFIG_CFB_CONSOLE

#if !defined(CONFIG_VIDEO_BOOTLOGO)
#error "Please define CONFIG_VIDEO_BOOTLOGO"
#endif

static int aboot_video_get_brightness(void)
{
	if (!aboot_video_initiated)
		return 0;

#if defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) && defined(CONFIG_ABOOT_HAS_MULTI_LOGOS)
	if (aboot_wait_4_charging) {
#if defined(CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS)
		if (aboot_state == ABOOT_STATE_VBAT_LOW || aboot_state == ABOOT_STATE_CHARGING_TIMEDOUT) {
			if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS)
				return CONFIG_ABOOT_VBAT_LOW_LOGO_AC_CHARGER_BRIGHTNESS;
			else
				return CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS;
		}
#endif
		return CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_BRIGHTNESS;
	}
#endif

#if defined(CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS)
	if (aboot_state == ABOOT_STATE_VBAT_LOW) {
		if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS)
			return CONFIG_ABOOT_VBAT_LOW_LOGO_AC_CHARGER_BRIGHTNESS;
		else
			return CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS;
	}
#endif

#if defined(CONFIG_ABOOT_FASTBOOT_LOGO_BRIGHTNESS)
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
	if (aboot_reason == ABOOT_REASON_FASTBOOT || aboot_reason == ABOOT_REASON_PCBA)
#else
	if (aboot_reason == ABOOT_REASON_FASTBOOT)
#endif
		return CONFIG_ABOOT_FASTBOOT_LOGO_BRIGHTNESS;
#endif

#if defined(CONFIG_ABOOT_RECOVERY_LOGO_BRIGHTNESS)
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
	if (aboot_reason == ABOOT_REASON_RECOVERY)
#else
	if (aboot_reason == ABOOT_REASON_RECOVERY || aboot_reason == ABOOT_REASON_PCBA)
#endif
		return CONFIG_ABOOT_RECOVERY_LOGO_BRIGHTNESS;
#endif

#if defined(CONFIG_ABOOT_CHARGER_BRIGHTNESS)
	if (aboot_reason == ABOOT_REASON_CHARGER)
		return CONFIG_ABOOT_CHARGER_BRIGHTNESS;
	else if (aboot_reason == ABOOT_REASON_CHARGER)
#if defined(CONFIG_ABOOT_PC_CHARGER_BRIGHTNESS)
		return CONFIG_ABOOT_PC_CHARGER_BRIGHTNESS;
#else
		return CONFIG_ABOOT_CHARGER_BRIGHTNESS;
#endif
#endif

	return CONFIG_VIDEO_DEFAULT_BRIGHTNESS;
}

static void aboot_video_init(void)
{
#ifdef CONFIG_VIDEO_NOT_CONSOLE_DEVICE
	if (aboot_video_initiated) {
		video_clear();
		aboot_wait_4_charging_video_debug_lines = 0;
		VIDEO_SET_BRIGHTNESS(aboot_video_get_brightness());
	} else if (video_fb_init() == 0) {
		aboot_video_initiated = 1;
		VIDEO_SET_BRIGHTNESS(aboot_video_get_brightness());
	}
#else
	/* debug purpose */
	if (aboot_video_initiated) {
		video_clear();
		aboot_wait_4_charging_video_debug_lines = 0;
		VIDEO_SET_BRIGHTNESS(aboot_video_get_brightness());
	} else {
		aboot_video_skip = 0;
		if (drv_video_init() == 1) {
			setenv("stderr", "vga");
			setenv("stdout", "vga");
			aboot_video_initiated = 1;
			VIDEO_SET_BRIGHTNESS(aboot_video_get_brightness());
		}
	}
#endif /* CONFIG_VIDEO_NOT_CONSOLE_DEVICE */
}

#ifdef CONFIG_VIDEO_BOOTLOGO

#if defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) && defined(CONFIG_ABOOT_HAS_MULTI_LOGOS)
static const char *aboot_wait_4_charging_logo[] = {
	[ABOOT_STATE_TRICKLE_CHARGING]	= "pre_charging", /* Trickle-Charging/Pre-Charging/Charging */
	[ABOOT_STATE_PRECHARGING]		= "pre_charging", /* Trickle-Charging/Pre-Charging/Charging */
	[ABOOT_STATE_NOT_CHARGING]		= "not_charging", /* Not Charging */
	[ABOOT_STATE_CHARGING_TIMEDOUT]	= "not_charging", /* Not Charging */
	[ABOOT_STATE_VBAT_LOW]			= "bat_low", /* Battery Voltage is too LOW */
};
#endif

int board_video_bootlogo(ulong bmp_image, ulong bmp_max_size)
{
	int ret = -EACCES;

	if (ainitiated) {
		unsigned long size = bmp_max_size;

#ifdef CONFIG_ABOOT_HAS_MULTI_LOGOS
		{
			const char *logo_part_name;
			const char *logo_name;

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO
			if (aboot_wait_4_charging) {
				logo_part_name = "logos";

				if (aboot_state >= ABOOT_STATE_TRICKLE_CHARGING &&
					aboot_state <= ABOOT_STATE_VBAT_LOW) {
					logo_name = aboot_wait_4_charging_logo[aboot_state];
				} else {
					logo_name = "offcharger";
				}
#if defined(CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS)
			} else if (aboot_state == ABOOT_STATE_VBAT_LOW) {
				logo_part_name = "logos";
				logo_name = "bat_low";
#endif
			} else
#endif
			if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER) {
				logo_part_name = "logos";
				logo_name = "offcharger";
#ifdef CONFIG_ABOOT_HAS_FASTBOOT_LOGO
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
			} else if (aboot_reason == ABOOT_REASON_FASTBOOT || aboot_reason == ABOOT_REASON_PCBA)
#else
			} else if (aboot_reason == ABOOT_REASON_FASTBOOT)
#endif
			{
				logo_part_name = "logos";
				logo_name = "fastboot";
#endif
#ifdef CONFIG_ABOOT_HAS_RECOVERY_LOGO
			} else if (aboot_reason == ABOOT_REASON_RECOVERY) {
				logo_part_name = "logos";
				logo_name = "recovery";
#endif
			} else {
				logo_part_name = "bootlogo";
				logo_name = "bootlogo";
			}

			ret = aboot_mmc_read_multimg(CONFIG_ABOOT_EMMC_DEVICE_NO, logo_part_name, logo_name,
				(void *)bmp_image, &size);
			if (ret == -ENXIO) {
				int read_again = 0;

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO
				if (aboot_wait_4_charging) {
					logo_part_name = "logos";
					logo_name = "offcharger";
					read_again = 1;
				} else
#endif
				{
#ifdef CONFIG_ABOOT_HAS_FASTBOOT_LOGO
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
					if (aboot_reason == ABOOT_REASON_FASTBOOT || aboot_reason == ABOOT_REASON_PCBA)
#else
					if (aboot_reason == ABOOT_REASON_FASTBOOT)
#endif
					{
						logo_part_name = "bootlogo";
						logo_name = "bootlogo";
						read_again = 1;
					}
#endif
#ifdef CONFIG_ABOOT_HAS_RECOVERY_LOGO
					if (aboot_reason == ABOOT_REASON_RECOVERY) {
						logo_part_name = "bootlogo";
						logo_name = "bootlogo";
						read_again = 1;
					}
#endif
				}

				if (read_again) {
					ret = aboot_mmc_read_multimg(CONFIG_ABOOT_EMMC_DEVICE_NO, logo_part_name, logo_name,
						(void *)bmp_image, &size);
				}
			}
		}

		if (ret == -ENOEXEC)
#endif /* CONFIG_ABOOT_HAS_MULTI_LOGOS */
		{
			const char *logo_part_name;

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO
			if (aboot_wait_4_charging) {
				logo_part_name = "logos";
			} else
#endif
			if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER) {
				logo_part_name = "logos";
			} else {
				logo_part_name = "bootlogo";
			}

			ret = aboot_mmc_read_raw(CONFIG_ABOOT_EMMC_DEVICE_NO, logo_part_name,
				(void *)bmp_image, &size);
		}
	}

	return ret;
}
#endif

#ifndef CONFIG_VIDEO_NOT_CONSOLE_DEVICE
static int aboot_video_skip = 1;

int board_video_skip(void)
{
	return aboot_video_skip;
}
#endif

#endif /* CONFIG_CFB_CONSOLE */

/*-------------------------------------------------------------------------*/

static void __aboot_board_set_state(int state, int vbat)
{
	switch (state) {
	case ABOOT_STATE_TRICKLE_CHARGING:
	case ABOOT_STATE_PRECHARGING:
#if !(defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) && defined(CONFIG_ABOOT_HAS_MULTI_LOGOS))
#if defined(CONFIG_SYS_VIBRATOR)
		if (!aboot_video_initiated) {
			if (vbat >= CONFIG_ABOOT_POWERON_VIBRATION_MINIMAL_VOLTAGE ||
					power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
				vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME);
			}
		}
#endif
#endif /* !(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO && CONFIG_ABOOT_HAS_MULTI_LOGOS) */
		break;
	case ABOOT_STATE_NOT_CHARGING:
	case ABOOT_STATE_CHARGING_TIMEDOUT:
	case ABOOT_STATE_VBAT_LOW:
#if !(defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) && defined(CONFIG_ABOOT_HAS_MULTI_LOGOS))
#if defined(CONFIG_SYS_VIBRATOR)
		if (!aboot_video_initiated) {
			if (vbat >= CONFIG_ABOOT_POWERON_VIBRATION_MINIMAL_VOLTAGE ||
					power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
				vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME);
				mdelay(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
				vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME);
				mdelay(CONFIG_ABOOT_POWERON_VIBRATION_TIME);
			}
		}
#endif
#endif /* !(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO && CONFIG_ABOOT_HAS_MULTI_LOGOS) */
		break;
	default:
		break;
	}
}

/**
 * Inputs:
 *   state - the current aboot state (ABOOT_STATE_XXXXXX)
 *   vbat - the current battery voltage (mV)
 */
void aboot_board_set_state(int state, int vbat)
		__attribute__((weak, alias("__aboot_board_set_state")));

void aboot_set_state(int state)
{
#if defined(CONFIG_ABOOT_STATE_DEBUG) || defined(CONFIG_ABOOT_VERBOSE_DEBUG)
	switch (state) {
	case ABOOT_STATE_TRICKLE_CHARGING:
		ABOOT_PUTS("state TRICKLE CHARGING\n");
		break;
	case ABOOT_STATE_PRECHARGING:
		ABOOT_PUTS("state PRECHARGING\n");
		break;
	case ABOOT_STATE_NOT_CHARGING:
		ABOOT_PUTS("state NOT CHARGING\n");
		break;
	case ABOOT_STATE_CHARGING_TIMEDOUT:
		ABOOT_PUTS("state CHARGING TIMED OUT\n");
		break;
	case ABOOT_STATE_VBAT_LOW:
		ABOOT_PUTS("state VBAT LOW\n");
		break;
	case ABOOT_STATE_BOOT_FASTBOOT:
		ABOOT_PUTS("state BOOT FASTBOOT\n");
		break;
	case ABOOT_STATE_BOOT_RECOVERY:
		ABOOT_PUTS("state BOOT RECOVERY\n");
		break;
	case ABOOT_STATE_BOOT_CHARGER:
		ABOOT_PUTS("state BOOT CHARGER\n");
		break;
	case ABOOT_STATE_BOOT_ANDROID:
		ABOOT_PUTS("state BOOT ANDROID\n");
		break;
	case ABOOT_STATE_REBOOT:
		ABOOT_PUTS("state REBOOT\n");
		break;
	case ABOOT_STATE_POWEROFF:
		ABOOT_PUTS("state POWEROFF\n");
		break;
	default:
		ABOOT_PRINT("** unknown state %d\n", state);
		break;
	}
#endif /* CONFIG_ABOOT_STATE_DEBUG || CONFIG_ABOOT_VERBOSE_DEBUG */

	if (aboot_vbat <= 0) {
		aboot_vbat = power_supply_get_battery_voltage();
	}

	if (state == ABOOT_STATE_POWEROFF) {
		PRINT("<<POWEROFF>> Main battery %dmV\n", aboot_vbat);
	}

#if defined(CONFIG_ABOOT_STATE_DEBUG) || defined(CONFIG_ABOOT_VERBOSE_DEBUG)
	ABOOT_PUTS("calling aboot_board_set_state\n");
#endif
	aboot_board_set_state(state, aboot_vbat);
#if defined(CONFIG_ABOOT_STATE_DEBUG) || defined(CONFIG_ABOOT_VERBOSE_DEBUG)
	ABOOT_PUTS("called aboot_board_set_state\n");
#endif

#if defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) && defined(CONFIG_ABOOT_HAS_MULTI_LOGOS)
	if (aboot_wait_4_charging && aboot_video_initiated && aboot_state != state && state != ABOOT_STATE_POWEROFF) {
		aboot_state = state;

		if (state == ABOOT_STATE_VBAT_LOW || state == ABOOT_STATE_CHARGING_TIMEDOUT)
			VIDEO_SET_BRIGHTNESS(aboot_video_get_brightness());

		video_clear();
		aboot_wait_4_charging_video_debug_lines = 0;

		if (state == ABOOT_STATE_VBAT_LOW || state == ABOOT_STATE_CHARGING_TIMEDOUT) {
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
			if (aboot_has_no_main_battery) {
				video_printf("<No Main Battery>\n");
				ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "System Voltage: %d mV\n", power_supply_get_sys_voltage());
			} else {
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)
				ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "System Voltage: %d mV (%d mV)\n",
						power_supply_get_sys_voltage(), aboot_vbat);
#else
				ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "Battery Voltage: %d mV\n", aboot_vbat);
#endif
			}
			aboot_wait_4_charging_video_debug_lines++;
#endif

			if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
				mdelay((CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME) * 4);
			} else if (aboot_vbat >= (CONFIG_ABOOT_VBAT_LOW_LOGO_MINIMAL_VOLTAGE)
					|| power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB_DCP) {
				mdelay(CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME);
#if (CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME > 250)
			} else if (aboot_vbat >= (CONFIG_ABOOT_VBAT_LOW_LOGO_MINIMAL_VOLTAGE - 100)) {
				mdelay(250); /* assume: 60Hz x 15 frames */
#endif /* CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME */
			}
		}
#if defined(CONFIG_ABOOT_VBAT_LOW_LOGO_BRIGHTNESS)
	} else if (state == ABOOT_STATE_VBAT_LOW) {
		aboot_state = state;

		if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS
				|| aboot_vbat >= (CONFIG_ABOOT_VBAT_LOW_LOGO_MINIMAL_VOLTAGE)
				|| power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB_DCP) {
			aboot_video_init();

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
			if (aboot_has_no_main_battery) {
				video_printf("<No Main Battery>\n");
				ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "System Voltage: %d mV\n", power_supply_get_sys_voltage());
			} else {
				ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "Battery Voltage: %d mV\n", aboot_vbat);
			}
#endif

			mdelay(CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME);
		}

		if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS)
			mdelay((CONFIG_ABOOT_VBAT_LOW_LOGO_WAIT_TIME) * 3);
#endif
	} else
#endif /* CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO && CONFIG_ABOOT_HAS_MULTI_LOGOS */
		aboot_state = state;
}

/*-------------------------------------------------------------------------*/

/* Android Recovery */

/* Bootloader Message
 *
 * This structure describes the content of a block in flash
 * that is used for recovery and the bootloader to talk to
 * each other.
 *
 * The command field is updated by linux when it wants to
 * reboot into recovery or to update radio or bootloader firmware.
 * It is also updated by the bootloader when firmware update
 * is complete (to boot into recovery for any final cleanup)
 *
 * The status field is written by the bootloader after the
 * completion of an "update-radio" or "update-hboot" command.
 *
 * The recovery field is only written by linux and used
 * for the system to send a message to recovery or the
 * other way around.
 */
struct bootloader_message {
    char command[32];
    char status[32];
    char recovery[1024];
};

/* Android recovery<->bootloader message */
static struct bootloader_message aboot_recovery_msg;

/* Read and write the bootloader command from the "misc" partition.
 * These return zero on success.
 */
static int aboot_get_bootloader_message(struct bootloader_message *out)
{
	int ret = -EPERM;
	aboot_ptentry *pte;

	pte = aboot_get_part_by_name("misc");
	if (!pte) {
		ABOOT_PUTS("invalid part 'misc'\n");
		ret = -ENOENT;
		goto fail0;
	}

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	if (ABOOT_PART_GET_DEVICE(pte->flags) == ABOOT_PART_FLAGS_DEVICE_EMMC) {
		unsigned long size = ALIGN(sizeof(struct bootloader_message), pte->blk_size);
		u8 *buffer = NULL;

		buffer = memalign(ARCH_DMA_MINALIGN, size);
		if (!buffer) {
			ABOOT_PUTS("out of memory\n");
			ret = -ENOMEM;
			goto emmc_fail0;
		}
		memset(buffer, 0, size);

		ret = aboot_mmc_read_raw(CONFIG_ABOOT_EMMC_DEVICE_NO, "misc", (void *)buffer, &size);
		if (ret) {
			ABOOT_PRINT("read 'misc' part err %d (%lu bytes)\n", ret, size);
			goto emmc_fail1;
		}
		memcpy(out, buffer, sizeof(struct bootloader_message));

emmc_fail1:
		free(buffer);
emmc_fail0:
		;
	} else
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */
#ifdef CONFIG_CMD_NAND
	if (ABOOT_PART_GET_DEVICE(pte->flags) == ABOOT_PART_FLAGS_DEVICE_NAND) {
		ABOOT_PUTS("unimplemented NAND dev\n");
		ret = -ENXIO;
		goto fail0;
	} else
#endif /* CONFIG_CMD_NAND */
	{
		ABOOT_PRINT("unknown dev type %u\n", ABOOT_PART_GET_DEVICE(pte->flags) >> ABOOT_PART_FLAGS_DEVICE_SHIFT);
		ret = -EINVAL;
		goto fail0;
	}

fail0:
	return ret;
}

#if !defined(CONFIG_ABOOT_PCBA_USE_FASTBOOT) || defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
static int aboot_set_bootloader_message(const struct bootloader_message *in)
{
	int ret = -EPERM;
	aboot_ptentry *pte;

	pte = aboot_get_part_by_name("misc");
	if (!pte) {
		ABOOT_PUTS("invalid part 'misc'\n");
		ret = -ENOENT;
		goto fail0;
	}

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	if (ABOOT_PART_GET_DEVICE(pte->flags) == ABOOT_PART_FLAGS_DEVICE_EMMC) {
		unsigned long size = ALIGN(sizeof(struct bootloader_message), pte->blk_size);
		u8 *buffer = NULL;

		buffer = memalign(ARCH_DMA_MINALIGN, size);
		if (!buffer) {
			ABOOT_PUTS("out of memory\n");
			ret = -ENOMEM;
			goto emmc_fail0;
		}
		memcpy(buffer, in, sizeof(struct bootloader_message));

		ret = aboot_mmc_write_raw(CONFIG_ABOOT_EMMC_DEVICE_NO, "misc", (void *)buffer, &size);
		if (ret) {
			ABOOT_PRINT("write 'misc' part err %d (%lu bytes)\n", ret, size);
			goto emmc_fail1;
		}

emmc_fail1:
		free(buffer);
emmc_fail0:
		;
	} else
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */
#ifdef CONFIG_CMD_NAND
	if (ABOOT_PART_GET_DEVICE(pte->flags) == ABOOT_PART_FLAGS_DEVICE_NAND) {
		ABOOT_PUTS("unimplemented NAND dev\n");
		ret = -ENXIO;
		goto fail0;
	} else
#endif /* CONFIG_CMD_NAND */
	{
		ABOOT_PRINT("unknown dev type %u\n", ABOOT_PART_GET_DEVICE(pte->flags) >> ABOOT_PART_FLAGS_DEVICE_SHIFT);
		ret = -EINVAL;
		goto fail0;
	}

fail0:
	return ret;
}
#endif /* !CONFIG_ABOOT_PCBA_USE_FASTBOOT || CONFIG_ABOOT_SDCARD_DEVICE_NO */

/**
 * The arguments which may be supplied in the recovery.command file:
 *   --send_intent=anystring - write the text out to recovery.intent
 *   --update_package=path - verify install an OTA package file
 *   --wipe_data - erase user data (and cache), then reboot
 *   --wipe_cache - wipe cache (but not user data), then reboot
 *   --set_encrypted_filesystem=on|off - enables / diasables encrypted fs
 *   --just_exit - do nothing; exit and reboot
 */
static void aboot_parse_recovery_message(void)
{
	memset(&aboot_recovery_msg, 0, sizeof(aboot_recovery_msg));
	aboot_get_bootloader_message(&aboot_recovery_msg);

	if (aboot_recovery_msg.command[0] != 0 && aboot_recovery_msg.command[0] != 255) {
		ABOOT_PRINT("Recovery command: %s\n", aboot_recovery_msg.command);

		if (aboot_recovery_msg.status[0] != 0 && aboot_recovery_msg.status[0] != 255)
			ABOOT_PRINT("Recovery status: %s\n", aboot_recovery_msg.status);

		if (aboot_recovery_msg.recovery[0] != 0 && aboot_recovery_msg.recovery[0] != 255)
			ABOOT_PRINT("Recovery args: %s\n", aboot_recovery_msg.recovery);

		if (!memcmp(aboot_recovery_msg.command, "boot-recovery\0", 14)
				&& !memcmp(aboot_recovery_msg.recovery, "recovery\n", 9)) {
			PUTS("<Android BCB> < switch to RECOVERY reason >\n");
			aboot_reason = ABOOT_REASON_RECOVERY;
		} else {
			memset(&aboot_recovery_msg, 0, sizeof(aboot_recovery_msg));
		}
	}
}

#if defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
static int aboot_sdcard_has_recovery_zip(int dev, int part, const char* filename)
{
	int ret = -EPERM;
	fat_read_info fat;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	long n;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	PRINT("## Checking '%s' from SD card...\n", filename);

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_rescan(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (!IS_SD(mmc)) {
		ABOOT_PUTS("not SD device\n");
		ret = -EINVAL;
		goto fail1;
	}
	dev_desc = &(mmc->block_dev);

	if (fat_register_device(dev_desc, part) != 0) {
		ABOOT_PUTS("invalid FAT\n");
		ret = -ENXIO;
		goto fail1;
	}

	n = file_fat_read_init(&fat, filename, 4096/*bytes*/);
	if (n <= 0) {
		ABOOT_PRINT("%s not found (%ld)\n", filename, n);
		ret = -ENOENT;
		goto fail1;
	}

	file_fat_read_exit(&fat);
	return 1;

#if 0
fail3:
	if (buffer) {
		free(buffer);
		buffer = NULL;
	}
fail2:
	file_fat_read_exit(&fat);
#endif
fail1:
	/* James Wu @TODO@ suspend MMC */
	;
fail0:
	return 0;
}
#endif /* CONFIG_ABOOT_SDCARD_DEVICE_NO */

/*-------------------------------------------------------------------------*/

static inline u32 __aboot_board_get_part_size(struct mmc *mmc, const char *name, u32 blk_offset)
{
	return 0;
}

static inline int __aboot_board_get_parts(aboot_partition **parts)
{
	*parts = NULL;
	return 0;
}

/**
 * Outputs:
 *   The partitions
 *
 * Returns:
 *   The number of the partitions
 */
int aboot_board_get_parts(aboot_partition **parts)
		__attribute__((weak, alias("__aboot_board_get_parts")));

/**
 * Inputs:
 *   1. mmc: eMMC device
 *   2. name: partition name
 *   3. blk_offset: the current block offset for this specific partition
 *
 * Returns:
 *   The total block size of this partition
 */
u32 aboot_board_get_part_size(struct mmc *mmc, const char *name, u32 blk_offset)
		__attribute__((weak, alias("__aboot_board_get_part_size")));

/*-------------------------------------------------------------------------*/

static void part_print_one(aboot_ptentry *pte)
{
	u32 flag_others = ABOOT_PART_GET_OTHER(pte->flags);

#ifdef CONFIG_ABOOT_VERBOSE_DEBUG
#else
	if (flag_others & ABOOT_PART_FLAGS_HIDDEN)
		return;
#endif /* CONFIG_ABOOT_VERBOSE_DEBUG */

	if (pte->part_index < 0)
		PUTS(" X:");
	else
		PRINT("%02d:", pte->part_index);

#ifdef CONFIG_ABOOT_VERBOSE_DEBUG
	PRINT(" 0x%08x-0x%08x; 0x%08x blks: %s", pte->part_blk_start,
			pte->part_blk_start + (pte->part_blk_size - 1),
			pte->part_blk_size, pte->name);
#elif defined(CONFIG_ABOOT_DEBUG)
	PRINT(" 0x%08x; 0x%08x blks: %s", pte->part_blk_start,
			pte->part_blk_size, pte->name);
#else
	PRINT(" 0x%08x blks: %s", pte->part_blk_size, pte->name);
#endif

	if (flag_others) {
		PUTS(" (");
		if (flag_others & ABOOT_PART_FLAGS_ERASABLE)
			PUTS(" E");
		if (flag_others & ABOOT_PART_FLAGS_LOCKED)
			PUTS(" L");
		if (flag_others & ABOOT_PART_FLAGS_RESERVED)
			PUTS(" R");
		if (flag_others & ABOOT_PART_FLAGS_HIDDEN)
			PUTS(" H");
		if (flag_others & ABOOT_PART_FLAGS_DUMMY)
			PUTS(" X");
		PUTS(" )\n");
	} else
		PUTS("\n");
}

static aboot_ptentry *part_clone(aboot_partition *part)
{
	aboot_ptentry *pte;

	if (!part)
		return NULL;

	pte = memalign(ARCH_DMA_MINALIGN, sizeof(aboot_ptentry));
	if (!pte) {
		ABOOT_PUTS("out of memory\n");
		return NULL;
	}

	strncpy(pte->name, part->name, ABOOT_PART_NAME_LEN);
	pte->part_blk_start = part->part_blk_start;
	pte->part_blk_size = pte->part_blk_num = part->part_blk_num;
	pte->flags = part->flags;
	pte->part_index = -1;

	INIT_LIST_HEAD(&(pte->list));

	return pte;
}

static void part_destroy(aboot_ptentry **pte)
{
	if (pte && *pte) {
		free(*pte);
		*pte = NULL;
	}
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_CMD_NAND
static struct list_head nand_parts_list;
static nand_info_t *aboot_nand;

static int curr_nand_part_num = 0;
static int curr_nand_part_num_skip_reserved = 0;

static aboot_ptentry *part_nand_add(aboot_partition *part, u32 *curr_offset)
{
	u32 repeat = 0;
	aboot_ptentry *pte;

	pte = part_clone(part);
	if (!pte)
		return NULL;

	pte->blk_size = aboot_nand->writesize;
	pte->erase_size = aboot_nand->erasesize;

	if (pte->part_blk_start == ABOOT_PART_START_APPEND) {
		pte->part_blk_start = *curr_offset;
	} else if (pte->part_blk_start == ABOOT_PART_START_DUMMY) {
		pte->part_blk_start = 0;
	} else {
		*curr_offset = pte->part_blk_start;
	}

	if (pte->part_blk_num == ABOOT_PART_NUM_FULL) {
		pte->part_blk_size = pte->part_blk_num =
				(u32)(aboot_nand->size / aboot_nand->erasesize) - pte->part_blk_start;
	} else if (pte->part_blk_num == ABOOT_PART_NUM_DYNAMIC) {
		ABOOT_PRINT("bad NAND part '%s' size\n", part->name);
		part_destroy(&pte);
		return NULL;
	} else {
		repeat = ABOOT_PART_GET_REPEAT(pte->flags);
		if (repeat)
			pte->part_blk_size = pte->part_blk_num * repeat;
	}

	if (!(pte->flags & ABOOT_PART_FLAGS_DUMMY))
		*curr_offset += pte->part_blk_size;

	ABOOT_VPRINT("[NAND]  0x%08x 0x%08x %s (%u) 0x%08x\n",
			pte->part_blk_start, pte->part_blk_size, pte->name, repeat, *curr_offset);

	if (!(pte->flags & ABOOT_PART_FLAGS_DUMMY) && pte->part_blk_size == 0) {
		ABOOT_PRINT("part '%s' has no size\n", part->name);
		part_destroy(&pte);
	} else if (*curr_offset > (u32)(aboot_nand->size / aboot_nand->erasesize)) {
		ABOOT_PRINT("part '%s' is out of NAND capacity\n", part->name);
		part_destroy(&pte);
	} else {
		list_add_tail(&(pte->list), &nand_parts_list);

		curr_nand_part_num++;
		if (!(pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)))
			pte->part_index = curr_nand_part_num_skip_reserved++;
	}

	return pte;
}
#endif /* CONFIG_CMD_NAND */

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO

#define EMMC_PART_ERR_DEFAULT		(-4) /* The default partitions are incorrect! */
#define EMMC_PART_ERR_NAME			(-5) /* The GPT name is not equal */
#define EMMC_PART_ERR_START			(-6) /* The GPT start offset is not equal */
#define EMMC_PART_ERR_SIZE			(-7) /* The GPT block size is not equal */
#define EMMC_PART_ERR_INCONSISTENT	(-8) /* The number of the partitions is not equal */

static struct list_head emmc_parts_list;
static struct mmc *aboot_mmc;

static int curr_emmc_part_num = 0;
static int curr_emmc_part_num_skip_reserved = 0;
static int curr_emmc_part = 0;
static u32 curr_emmc_offset = 0;

static aboot_ptentry *part_emmc_add(aboot_partition *part, u32 *curr_offset)
{
	u32 repeat = 0;
	aboot_ptentry *pte;

	pte = part_clone(part);
	if (!pte)
		return NULL;

	pte->blk_size = aboot_mmc->write_bl_len;
	pte->erase_size = 4 * 1024 * 1024;	/* 4 MB */
	if ((pte->erase_size & (pte->blk_size - 1)) != 0)
		ABOOT_PRINT("%s: not 0x%X-aligned erase size 0x%08x\n", pte->name,
				pte->blk_size, pte->erase_size);

	if (pte->part_blk_start == ABOOT_PART_START_APPEND) {
		pte->part_blk_start = *curr_offset;
	} else if (pte->part_blk_start == ABOOT_PART_START_DUMMY) {
		pte->part_blk_start = 0;
	} else {
		if (pte->part_blk_start < *curr_offset)
			ABOOT_PRINT("%s: bad start 0x%08x<->0x%08x\n", pte->name,
					pte->part_blk_start, *curr_offset);
		else if (pte->part_blk_start != *curr_offset && curr_emmc_part_num > 6) {
			ABOOT_DPRINT("%s: start 0x%08x<->0x%08x (some blks are skipped)\n",
					pte->name, pte->part_blk_start, *curr_offset);
		}
		*curr_offset = pte->part_blk_start;
	}

	if (pte->part_blk_num == ABOOT_PART_NUM_FULL) {
		/* The last LBA is reserved for the alternate GPT header */
		if ((pte->part_blk_start + 1) < (u32)aboot_mmc->block_dev.lba) {
			repeat = (u32)aboot_mmc->block_dev.lba - pte->part_blk_start - 1;
			pte->part_blk_size = pte->part_blk_num = repeat;
		} else
			pte->part_blk_size = pte->part_blk_num = 0;

		pte->flags &= ~(ABOOT_PART_FLAGS_REPEAT_MASK);
	} else if (pte->part_blk_num == ABOOT_PART_NUM_DYNAMIC) {
		repeat = aboot_board_get_part_size(aboot_mmc, part->name, pte->part_blk_start);

		/* The last LBA is reserved for the alternate GPT header */
		if ((pte->part_blk_start + repeat) >= (u32)aboot_mmc->block_dev.lba)
			repeat--;

		pte->part_blk_size = pte->part_blk_num = repeat;

		pte->flags &= ~(ABOOT_PART_FLAGS_REPEAT_MASK);
	} else {
		repeat = ABOOT_PART_GET_REPEAT(pte->flags);
		if (repeat)
			pte->part_blk_size = pte->part_blk_num * repeat;
	}

	if (!(pte->flags & ABOOT_PART_FLAGS_DUMMY))
		*curr_offset += pte->part_blk_size;

	if (pte->flags & ABOOT_PART_FLAGS_SIZE_4K_ALIGNED)
		pte->part_blk_size &= ~0x00000FFF;

	if ((pte->flags & ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4) == ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4 &&
			(pte->part_blk_start & 0x000007FF)) {
		ABOOT_PRINT("EXT4 part '%s': offset 0x%08x is not 2K-aligned\n", part->name, pte->part_blk_start);
	}

	ABOOT_VPRINT("[eMMC]  0x%08x 0x%08x %s (%u) 0x%08x\n",
			pte->part_blk_start, pte->part_blk_size, pte->name, repeat, *curr_offset);

	if (!(pte->flags & ABOOT_PART_FLAGS_DUMMY) && pte->part_blk_size == 0) {
		ABOOT_PRINT("part '%s' has no size\n", part->name);
		part_destroy(&pte);
	} else if (*curr_offset >= (u32)aboot_mmc->block_dev.lba) {
		/* The last LBA is reserved for the alternate GPT header */
		ABOOT_PRINT("part '%s' is out of MMC capacity (%u)\n",
				part->name, *curr_offset);
		part_destroy(&pte);
	} else {
		list_add_tail(&(pte->list), &emmc_parts_list);

		curr_emmc_part_num++;
		if (!(pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)))
			pte->part_index = curr_emmc_part_num_skip_reserved++;
	}

	return pte;
}

static void part_emmc_remove(aboot_ptentry *pte)
{
	curr_emmc_part_num--;
	if (!(pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)))
		curr_emmc_part_num_skip_reserved--;

	list_del(&(pte->list));
	part_destroy(&pte);
}

static void part_emmc_clear(void)
{
	aboot_ptentry *pte;

	while (!list_empty(&emmc_parts_list)) {
		pte = list_first_entry(&emmc_parts_list, aboot_ptentry, list);
		part_emmc_remove(pte);
	}

	curr_emmc_part_num = 0;
	curr_emmc_part_num_skip_reserved = 0;
}

static int part_emmc_reset(void)
{
	u32 curr_offset = 0;
	int i;

	/* Make sure the list is empty before invoking part_emmc_reset() */

	curr_emmc_part_num = 0;
	curr_emmc_part_num_skip_reserved = 0;

	for (i = 0 ; i < def_parts_num ; i++) {
		if (ABOOT_PART_GET_DEVICE(def_parts[i].flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
			/* not eMMC partition */
			continue;
		}
		part_emmc_add(&(def_parts[i]), &curr_offset);
	}

	curr_emmc_part = 0;
	curr_emmc_offset = 0;

	return 0;
}

static int part_import_emmc_remant(void)
{
	/* Find the next eMMC partition */
	while (curr_emmc_part < def_parts_num) {
		if (ABOOT_PART_GET_DEVICE(def_parts[curr_emmc_part].flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
			/* not eMMC partition */
			curr_emmc_part++;
			continue;
		} else if (def_parts[curr_emmc_part].flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)) {
			/* Reserved/Dummy partition */
			if (!part_emmc_add(&(def_parts[curr_emmc_part]), &curr_emmc_offset))
				return EMMC_PART_ERR_DEFAULT;
			curr_emmc_part++;
			continue;
		}
		/* Found the remant partition */
		if (!part_emmc_add(&(def_parts[curr_emmc_part]), &curr_emmc_offset))
			return EMMC_PART_ERR_DEFAULT;
		ABOOT_VPRINT("[eMMC] found remant part '%s'\n", def_parts[curr_emmc_part].name);
		return EMMC_PART_ERR_INCONSISTENT;
	}

	return 0;
}

static int part_import_emmc(block_dev_desc_t *dev_desc, int part, disk_partition_t *info)
{
	aboot_ptentry *pte;

	ABOOT_VPRINT("[GPT%02d] 0x%08lx 0x%08lx %s\n", part, info->start, info->size, info->name);

	/* Find the next eMMC partition */
	while (curr_emmc_part < def_parts_num) {
		if (ABOOT_PART_GET_DEVICE(def_parts[curr_emmc_part].flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
			/* not eMMC partition */
			curr_emmc_part++;
			continue;
		} else if (def_parts[curr_emmc_part].flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)) {
			/* Reserved/Dummy partition */
			if (!part_emmc_add(&(def_parts[curr_emmc_part]), &curr_emmc_offset))
				return EMMC_PART_ERR_DEFAULT;
			curr_emmc_part++;
			continue;
		}
		break;
	}

	if (curr_emmc_part < def_parts_num) {
		if (strncmp((char*)info->name, (char*)def_parts[curr_emmc_part].name, ABOOT_PART_NAME_LEN)) {
			ABOOT_PRINT("bad GPT name: %s<->%s\n", info->name, def_parts[curr_emmc_part].name);
			return EMMC_PART_ERR_NAME;
		} else {
			pte = part_emmc_add(&(def_parts[curr_emmc_part]), &curr_emmc_offset);
			if (!pte)
				return EMMC_PART_ERR_DEFAULT;
			else if (pte->part_blk_start != info->start) {
				ABOOT_PRINT("bad GPT %s: 0x%08lx<->0x%08x, 0x%08lx<->0x%08x\n",
					info->name, info->start, pte->part_blk_start, info->size, pte->part_blk_size);
				return EMMC_PART_ERR_START;
			} else if (pte->part_blk_size != info->size) {
				ABOOT_PRINT("bad GPT %s: 0x%08lx<->0x%08x, 0x%08lx<->0x%08x\n",
					info->name, info->start, pte->part_blk_start, info->size, pte->part_blk_size);
				return EMMC_PART_ERR_SIZE;
			}
			curr_emmc_part++;
		}
	} else {
		ABOOT_PRINT("bad GPT %s: %d<->%d (%d)\n",
				info->name, part, curr_emmc_part, def_parts_num);
		return EMMC_PART_ERR_INCONSISTENT;
	}

	return 0;
}

static int part_export_emmc(block_dev_desc_t *dev_desc, int part, disk_partition_t *info)
{
	static struct list_head *curr_entry = NULL;
	aboot_ptentry *pte = NULL;

	if (curr_entry == NULL)
		curr_entry = emmc_parts_list.next;

	if (curr_entry == &emmc_parts_list) {
		ABOOT_PRINT("out of parts\n");
		return EMMC_PART_ERR_INCONSISTENT;
	}

	/* Find the next eMMC partition */
	while (curr_entry != &emmc_parts_list) {
		pte = list_entry(curr_entry, aboot_ptentry, list);
		if (ABOOT_PART_GET_DEVICE(pte->flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
			/* not eMMC partition */
			curr_entry = curr_entry->next;
			continue;
		} else if (pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_DUMMY)) {
			/* Reserved/Dummy partition */
			curr_entry = curr_entry->next;
			continue;
		}
		break;
	}

	if (curr_entry != &emmc_parts_list) {
		/* the maximum disk size is 2 TB */
		info->start = pte->part_blk_start;
		info->size = pte->part_blk_size;
		info->blksz = aboot_mmc->read_bl_len;
		strncpy((char*)info->name, pte->name, ABOOT_PART_NAME_LEN);
		curr_entry = curr_entry->next;

		ABOOT_VPRINT("[GPT%02d] 0x%08lx 0x%08lx %s\n", part, info->start, info->size, info->name);
	} else {
		ABOOT_PRINT("out of parts\n");
		return EMMC_PART_ERR_INCONSISTENT;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ABOOT_HAS_MULTI_LOGOS)
static multi_img_entry *aboot_multimg_find(multi_img_hdr *hdr, const char *name)
{
	multi_img_entry *image = NULL;

	if (hdr && name) {
		int i;
		size_t length = strlen(name);
		multi_img_entry *images = hdr->images;

		for (i = 0 ; i < hdr->num_of_images ; i++) {
			if (!memcmp(images[i].name, name, length)) {
				image = images + i;
				break;
			}
		}
	}

	return image;
}

/**
 * Inputs:
 *   1. dev: eMMC device number
 *   2. part_name: partition name
 *   2. image_name: image name
 *   3. buffer: buffer pointer (buffer size must be in block alignment)
 *   4. size: the size to be read
 *
 * Outputs:
 *   1. buffer: data in buffer
 *   2. size: the read size in block alignment
 *
 * Returns:
 *   1. 0: the operation is successful
 *   2. none zero: error occurred
 */
static int aboot_mmc_read_multimg(int dev, const char *part_name,
		const char *image_name, void *buffer, unsigned long *size)
{
	multi_img_hdr *hdr;
	int ret = -EPERM;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	unsigned long blk_start, blk_cnt, n;
	aboot_ptentry *pte;
	unsigned offset = 0;
	multi_img_entry *image;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	if (!aboot_wait_4_charging)
		otg_handle_interrupts(1);
#endif

	PRINT("## Loading multimg '%s' from MMC%d at '%s' ...\n", image_name, dev, part_name);

	pte = aboot_get_part_by_name(part_name);
	if (!pte) {
		ABOOT_PRINT("invalid part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	if (ABOOT_PART_GET_DEVICE(pte->flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
		ABOOT_PRINT("not eMMC part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_init(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (IS_SD(mmc)) {
		ABOOT_PUTS("not eMMC device\n");
		ret = -EINVAL;
		goto fail1;
	}
	dev_desc = &(mmc->block_dev);

_retry:
	/* Header */
	if (offset & (dev_desc->blksz - 1))
		ABOOT_PRINT("multi_img_hdr offset (%u) not aligned (%u)\n", offset, (u32)dev_desc->blksz);
	blk_start = pte->part_blk_start;
	blk_start += offset / dev_desc->blksz;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading @%08x (0x%lx,1) ...\n", (unsigned int)aboot_mmc_blk, blk_start);
#endif
	n = dev_desc->block_read(dev, blk_start, 1, aboot_mmc_blk);
	if (n != 1) {
		ABOOT_PRINT("read multi_img_hdr err: %lu\n", n);
		ret = -EIO;
		goto fail1;
	}
	hdr = (multi_img_hdr *)aboot_mmc_blk;

	/* Header Validation */
	if (memcmp(hdr->magic, MULTI_IMAGE_MAGIC, MULTI_IMAGE_MAGIC_SIZE)) {
		PUTS("## Bad multimg magic\n");
		ret = -ENOEXEC;
		goto fail1;
	}
	if ((hdr->page_size != 2048) && (hdr->page_size != 4096)) {
		PRINT("## Bad multimg page size %u\n", hdr->page_size);
		ret = -ENOEXEC;
		goto fail1;
	}
#if 0
	/* Header checksum */
	if (hdr->header_checksum) {
		unsigned checksum = hdr->header_checksum;
		hdr->header_checksum = 0;
		hdr->header_checksum = (unsigned)crc32(0, (uchar*)hdr, sizeof(multi_img_hdr));
		if (checksum != hdr->header_checksum) {
			PUTS("## Bad multimg header\n");
			ABOOT_DPRINT("multimg header checksum 0x%08x <-> 0x%08x\n", checksum, hdr->header_checksum);
			ret = -EBADMSG;
			goto fail1;
		}
	}
#endif

	image = aboot_multimg_find(hdr, image_name);
	if (!image) {
		if (hdr->next_header) {
			offset = hdr->next_header;
			goto _retry;
		}
		PRINT("## multimg '%s' NOT found\n", image_name);
		ret = -ENXIO;
		goto fail1;
	}

	blk_start = pte->part_blk_start;
	if (image->image_offset & (dev_desc->blksz - 1))
		ABOOT_PRINT("multi_img_entry offset (%u) not aligned (%u)\n", image->image_offset, (u32)dev_desc->blksz);
	blk_start += image->image_offset / dev_desc->blksz;
	blk_cnt = ALIGN(image->image_size, dev_desc->blksz) / dev_desc->blksz;
	blk_cnt = min(blk_cnt, pte->part_blk_size);
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading @%08x (0x%lx,%lu) ...\n", (unsigned int)buffer, blk_start, blk_cnt);
#endif
	n = dev_desc->block_read(dev, blk_start, blk_cnt, buffer);
	if (n != blk_cnt) {
		ABOOT_PRINT("read err: %lu\n", n);
		if (size)
			*size = n * dev_desc->blksz;
		ret = -EIO;
		goto fail1;
	}

	if (size)
		*size = n * dev_desc->blksz;

	ret = 0;

fail1:
	;
fail0:
	return ret;
}
#endif /* CONFIG_ABOOT_HAS_MULTI_LOGOS */

/*-------------------------------------------------------------------------*/

/**
 * Inputs:
 *   1. dev: eMMC device number
 *   2. part_name: partition name
 *   3. buffer: buffer pointer (buffer size must be in block alignment)
 *   4. size: the size to be read
 *
 * Outputs:
 *   1. buffer: data in buffer
 *   2. size: the read size in block alignment
 *
 * Returns:
 *   1. 0: the operation is successful
 *   2. none zero: error occurred
 */
static int aboot_mmc_read_raw(int dev, const char *part_name, void *buffer, unsigned long *size)
{
	int ret = -EPERM;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	unsigned long blk_start, blk_cnt, n;
	aboot_ptentry *pte;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	if (!aboot_wait_4_charging)
		otg_handle_interrupts(1);
#endif

	PRINT("## Reading from MMC%d at '%s' ...\n", dev, part_name);

	pte = aboot_get_part_by_name(part_name);
	if (!pte) {
		ABOOT_PRINT("invalid part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	if (ABOOT_PART_GET_DEVICE(pte->flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
		ABOOT_PRINT("not eMMC part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_init(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (IS_SD(mmc)) {
		ABOOT_PUTS("not eMMC device\n");
		ret = -EINVAL;
		goto fail0;
	}
	dev_desc = &(mmc->block_dev);

	blk_start = pte->part_blk_start;
	if (size && *size != 0) {
		/* bytes -> blocks */
		blk_cnt = ALIGN(*size, dev_desc->blksz) / dev_desc->blksz;
		blk_cnt = min(blk_cnt, pte->part_blk_size);
	} else
		blk_cnt = pte->part_blk_size;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading @%08x (0x%lx,%lu) ...\n", (unsigned int)buffer, blk_start, blk_cnt);
#endif /* CONFIG_ABOOT_DEBUG */
	n = dev_desc->block_read(dev, blk_start, blk_cnt, buffer);
	if (n != blk_cnt) {
		ABOOT_PRINT("read err: %lu\n", n);
		if (size)
			*size = n * dev_desc->blksz;
		ret = -EIO;
		goto fail0;
	}

	if (size)
		*size = n * dev_desc->blksz;

	return 0;

fail0:
	return ret;
}

#if !defined(CONFIG_ABOOT_PCBA_USE_FASTBOOT) || defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
/**
 * Inputs:
 *   1. dev: eMMC device number
 *   2. part_name: partition name
 *   3. buffer: buffer pointer (buffer size must be in block alignment)
 *   4. size: the size to be written
 *
 * Outputs:
 *   1. size: the written size in block alignment
 *
 * Returns:
 *   1. 0: the operation is successful
 *   2. none zero: error occurred
 */
static int aboot_mmc_write_raw(int dev, const char *part_name, void *buffer, unsigned long *size)
{
	int ret = -EPERM;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	unsigned long blk_start, blk_cnt, n;
	aboot_ptentry *pte;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	PRINT("## Writing into MMC%d at '%s' ...\n", dev, part_name);

	pte = aboot_get_part_by_name(part_name);
	if (!pte) {
		ABOOT_PRINT("invalid part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	if (ABOOT_PART_GET_DEVICE(pte->flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
		ABOOT_PRINT("not eMMC part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_init(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (IS_SD(mmc)) {
		ABOOT_PUTS("not eMMC device\n");
		ret = -EINVAL;
		goto fail0;
	}
	dev_desc = &(mmc->block_dev);

	blk_start = pte->part_blk_start;
	if (size && *size != 0) {
		/* bytes -> blocks */
		blk_cnt = ALIGN(*size, dev_desc->blksz) / dev_desc->blksz;
		blk_cnt = min(blk_cnt, pte->part_blk_size);
	} else
		blk_cnt = pte->part_blk_size;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Writing @%08x (0x%lx,%lu) ...\n", (unsigned int)buffer, blk_start, blk_cnt);
#endif /* CONFIG_ABOOT_DEBUG */
	n = dev_desc->block_write(dev, blk_start, blk_cnt, buffer);
	if (n != blk_cnt) {
		ABOOT_PRINT("write err: %lu\n", n);
		if (size)
			*size = n * dev_desc->blksz;
		ret = -EIO;
		goto fail0;
	}

	if (size)
		*size = n * dev_desc->blksz;

	return 0;

fail0:
	return ret;
}
#endif /* !CONFIG_ABOOT_PCBA_USE_FASTBOOT || CONFIG_ABOOT_SDCARD_DEVICE_NO */
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

/*-------------------------------------------------------------------------*/

/**
 * Returns:
 *   > 0 : The partition table is reset
 *   = 0 : The partition table is ready
 *   -1  : Can not get the default partitions
 *   -2  : Can not get the NAND device
 *   -3  : Can not get the eMMC device
 */
static int aboot_parts_init(void)
{
	int ret = 0;
	int r;

	/* Initialize the list */
#ifdef CONFIG_CMD_NAND
	INIT_LIST_HEAD(&nand_parts_list);
#endif /* CONFIG_CMD_NAND */
#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	INIT_LIST_HEAD(&emmc_parts_list);
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

	def_parts_num = aboot_board_get_parts(&def_parts);
	if (def_parts_num <= 0 || def_parts == NULL) {
		ABOOT_PUTS("no parts\n");
		/* Can not get the default partitions */
		return -1;
	} else {
		ABOOT_VPRINT("%d default parts\n", def_parts_num);
	}

#ifdef CONFIG_CMD_NAND
	if (nand_curr_device < 0 ||
			nand_curr_device >= CONFIG_SYS_MAX_NAND_DEVICE ||
			!nand_info[nand_curr_device].name) {
		ABOOT_PUTS("no NAND dev\n");
		ret = -2;
	} else {
		aboot_nand = &(nand_info[nand_curr_device]);
		for (r = 0 ; r < parts_num ; r++) {
			if (ABOOT_PART_GET_DEVICE(def_parts[r]->flags) == ABOOT_PART_FLAGS_DEVICE_NAND) {
				part_nand_add(def_parts[r]);
			}
		}
		if (list_empty(&nand_parts_list)) {
			ABOOT_PUTS("no NAND parts\n");
#ifndef CONFIG_ABOOT_EMMC_DEVICE_NO
			ret = -1;
#endif
		}
	}
#endif /* CONFIG_CMD_NAND */

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	aboot_mmc = find_mmc_device(CONFIG_ABOOT_EMMC_DEVICE_NO);
	if (!aboot_mmc) {
		ABOOT_PRINT("bad MMC dev %d\n", CONFIG_ABOOT_EMMC_DEVICE_NO);
	} else {
		r = mmc_init(aboot_mmc);
		if (r) {
			ABOOT_PRINT("mmc init err: %d\n", r);
			aboot_mmc = NULL;
		} else if (IS_SD(aboot_mmc)) {
			ABOOT_PRINT("not eMMC dev %d\n", CONFIG_ABOOT_EMMC_DEVICE_NO);
			aboot_mmc = NULL;
		} else if (aboot_mmc->block_dev.part_type != PART_TYPE_EFI) {
			ABOOT_PRINT("not EFI: %d\n", aboot_mmc->block_dev.part_type);
		} else if (aboot_mode == ABOOT_MODE_FACTORY || aboot_reason == ABOOT_REASON_PCBA) {
			ABOOT_PUTS("apply FACTORY mode\n");
		} else {
			ABOOT_VPUTS("found EFI\n");

			r = load_part(&aboot_mmc->block_dev, part_import_emmc);
			if (r) {
				ABOOT_PRINT("load parts err %d (%d)\n", r, curr_emmc_part_num);
				part_emmc_clear();
			} else {
				r = part_import_emmc_remant();
				if (r && r != EMMC_PART_ERR_DEFAULT) {
					ABOOT_PRINT("remant eMMC parts err: %d\n", r);
					part_emmc_clear();
				}
			}
		}
	}

	if (!aboot_mmc) {
		/* Can not get the eMMC device */
		ret = -3;
	} else if (list_empty(&emmc_parts_list)) {
		ABOOT_PUTS("reset to default eMMC parts\n");
		part_emmc_reset();
		if (list_empty(&emmc_parts_list))
			ABOOT_PUTS("no default parts\n");
		else {
			int retry = 3;

			/* Make sure we are using EFI partition type on eMMC */
			aboot_mmc->block_dev.part_type = PART_TYPE_EFI;

			do {
				r = save_part(&aboot_mmc->block_dev,
						curr_emmc_part_num_skip_reserved, part_export_emmc);
				if (r) {
					ABOOT_PRINT("%d: save parts err %d (%d)\n",
							retry, r, curr_emmc_part_num_skip_reserved);
				}
				retry--;
			} while (r && retry > 0);

			/* The partition table is reset */
			ret = 1;
		}
	}
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

	return ret;
}

/*-------------------------------------------------------------------------*/

void aboot_print_parts(void)
{
#if defined(CONFIG_CMD_NAND) || defined(CONFIG_ABOOT_EMMC_DEVICE_NO)
	struct list_head *entry;
	aboot_ptentry *pte;

#ifdef CONFIG_CMD_NAND
	if (aboot_nand) {
		if (list_empty(&nand_parts_list)) {
			PUTS("\nno NAND partitions (0x%08llx)\n", aboot_nand->size);
		} else {
			PUTS("\nNAND partitions (0x%08llx):\n", aboot_nand->size);
			list_for_each(entry, &(nand_parts_list)) {
				pte = list_entry(entry, aboot_ptentry, list);
				part_print_one(pte);
			}
		}
	}
#endif /* CONFIG_CMD_NAND */

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	if (aboot_mmc) {
		PRINT("\neMMC %u-bit %s%s(0x%x,0x%x,%c%c%c%c%c) ",
				aboot_mmc->bus_width,
				(aboot_mmc->card_caps & MMC_MODE_HS) ? "HS " : "",
				(aboot_mmc->card_caps & MMC_MODE_DDR_52MHz) ? "DDR " : "",
				aboot_mmc->cid[0] >> 24, (aboot_mmc->cid[0] >> 8) & 0xffff,
				aboot_mmc->cid[0] & 0xff, (aboot_mmc->cid[1] >> 24), (aboot_mmc->cid[1] >> 16) & 0xff,
				(aboot_mmc->cid[1] >> 8) & 0xff, aboot_mmc->cid[1] & 0xff);
		print_size(aboot_mmc->capacity, "\n");
		if (list_empty(&emmc_parts_list)) {
			PRINT("no eMMC partitions (0x%08x blks)\n", (u32)aboot_mmc->block_dev.lba);
		} else {
			PRINT("eMMC partitions (0x%08x blks):\n", (u32)aboot_mmc->block_dev.lba);
			list_for_each(entry, &(emmc_parts_list)) {
				pte = list_entry(entry, aboot_ptentry, list);
				part_print_one(pte);
			}
		}
	}
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

	PUTS("\n");
#else
#error "Please define CONFIG_CMD_NAND or CONFIG_ABOOT_EMMC_DEVICE_NO\n"
#endif /* CONFIG_CMD_NAND || CONFIG_ABOOT_EMMC_DEVICE_NO */
}

static void aboot_partition_table_init(void)
{
	int ret = aboot_parts_init();
	if (ret > 0) {
		/* The partition table is reset */
	} else if (ret == -1) {
		/* Can not get the default partitions */
		PUTS("\n\n================================\n");
		PUTS("= ERROR: no default PARTITIONS =\n");
		PUTS("================================\n\n");
		board_poweroff();
#ifdef CONFIG_CMD_NAND
	} else if (ret == -2) {
		/* Can not get the NAND device */
		PUTS("\n\n=========================\n");
		PUTS("= ERROR: no NAND device =\n");
		PUTS("=========================\n\n");
		vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
		board_poweroff();
#endif /* CONFIG_CMD_NAND */
#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	} else if (ret == -3) {
		/* Can not get the eMMC device */
		PUTS("\n\n=========================\n");
		PUTS("= ERROR: no eMMC device =\n");
		PUTS("=========================\n\n");
		vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
		board_poweroff();
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */
	}

	aboot_print_parts();
}

aboot_ptentry *aboot_get_part_by_name(const char *name)
{
#if defined(CONFIG_CMD_NAND) || defined(CONFIG_ABOOT_EMMC_DEVICE_NO)
	struct list_head *entry;
	aboot_ptentry *pte = NULL;

#ifdef CONFIG_CMD_NAND
	if (!list_empty(&nand_parts_list)) {
		list_for_each(entry, &(nand_parts_list)) {
			pte = list_entry(entry, aboot_ptentry, list);
			if (!strncmp(pte->name, name, ABOOT_PART_NAME_LEN))
				goto found;
		}
		pte = NULL;
	}
#endif /* CONFIG_CMD_NAND */

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	if (!list_empty(&emmc_parts_list)) {
		list_for_each(entry, &(emmc_parts_list)) {
			pte = list_entry(entry, aboot_ptentry, list);
			if (!strncmp(pte->name, name, ABOOT_PART_NAME_LEN))
				goto found;
		}
		pte = NULL;
	}
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

found:
	return pte;
#else
	return NULL;
#endif /* CONFIG_CMD_NAND || CONFIG_ABOOT_EMMC_DEVICE_NO */
}

aboot_ptentry *aboot_get_emmc_part(int index)
{
#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
	struct list_head *entry;
	aboot_ptentry *pte = NULL;

	if (!list_empty(&emmc_parts_list)) {
		list_for_each(entry, &(emmc_parts_list)) {
			pte = list_entry(entry, aboot_ptentry, list);
			if (pte->part_index == index)
				goto found;
		}
		pte = NULL;
	}

found:
	return pte;
#else
	return NULL;
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */
}

aboot_ptentry *aboot_get_nand_part(int index)
{
#ifdef CONFIG_CMD_NAND
	struct list_head *entry;
	aboot_ptentry *pte = NULL;

	if (!list_empty(&nand_parts_list)) {
		list_for_each(entry, &(nand_parts_list)) {
			pte = list_entry(entry, aboot_ptentry, list);
			if (pte->part_index == index)
				goto found;
		}
		pte = NULL;
	}

found:
	return pte;
#else
	return NULL;
#endif /* CONFIG_CMD_NAND */
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_SYS_POWER_SUPPLY

#ifndef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
static int aboot_check_main_battery(void)
{
	if (power_supply_has_main_battery() == 0) {
		aboot_has_no_main_battery = 1;

		if (aboot_vbat <= 0)
			aboot_vbat = power_supply_get_battery_voltage();

		PRINT("<%d> <NO Main Battery> VSYS %dmV; VBAT %dmV\n",
				power_supply_get_charger_source(), power_supply_get_sys_voltage(), aboot_vbat);

		return -1;
	}

	aboot_has_no_main_battery = 0;

	return 0;
}
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY || CONFIG_ABOOT_POWERON_CHECK_BATTERY */

#if defined(CONFIG_CFB_CONSOLE) && defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) \
	&& defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG) && (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG > 0)
static void aboot_wait_for_charging_video_debug(const char* status, int poweron_volt,
		int vbat, int capacity, int vsys, int charge_current, int charger_volt)
{
#if (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG == 1)
	static int prev_vbat_10th = -1;
	int curr_vbat_10th;
#endif

	if (!aboot_video_initiated)
		return;

#if (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG == 1)
	curr_vbat_10th = vbat / 10;
	if (curr_vbat_10th == prev_vbat_10th)
		return;
	prev_vbat_10th = curr_vbat_10th;
#endif

	if (aboot_wait_4_charging_video_debug_lines > CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG_MAX_LINES) {
		video_clear();
		aboot_wait_4_charging_video_debug_lines = 0;
	}
	if (aboot_wait_4_charging_video_debug_lines == 0) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
		if (aboot_bat_locked)
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "Power-on Voltage: %d mV (Battery was Locked?)\n",
				poweron_volt);
		else
#endif
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1, "Power-on Voltage: %d mV\n",
				poweron_volt);
		aboot_wait_4_charging_video_debug_lines++;
	}

#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) \
		&& !defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT) \
		&& !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
	extern int twl6032_get_battery_voltage(void);
	if (power_supply_has_external_fuelgauge()) {
		int twl6032_vbat = twl6032_get_battery_voltage();
		if (twl6032_vbat == CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> Battery Voltage: %d mV (%d%%); System Voltage: %d mV\n",
				(status ? status : "X"), vbat, capacity, vsys);
		} else {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> Battery Voltage: %d mV (%d%%) (%dmV); System Voltage: %d mV\n",
				(status ? status : "X"), vbat, capacity, twl6032_vbat, vsys);
		}
		aboot_wait_4_charging_video_debug_lines++;
	} else
#endif
	{
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)
		if (charger_volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> System Voltage: %d mV (%d mV); Charger Voltage: %d mV\n",
				(status ? status : "X"), vsys, vbat, charger_volt);
		} else {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> System Voltage: %d mV (%d mV)\n",
				(status ? status : "X"), vsys, vbat);
		}
		aboot_wait_4_charging_video_debug_lines++;

		return;
#else
		if (capacity < 0) {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> Battery Voltage: %d mV; System Voltage: %d mV\n",
				(status ? status : "X"), vbat, vsys);
		} else {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"<%s> Battery Voltage: %d mV (%d%%); System Voltage: %d mV\n",
				(status ? status : "X"), vbat, capacity, vsys);
		}
		aboot_wait_4_charging_video_debug_lines++;
#endif
	}

	if (charge_current != CONFIG_POWER_SUPPLY_INVALID_CURRENT) {
		if (charger_volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"\tCharge Current: %d mA; Charger Voltage: %d mV\n",
				charge_current, charger_volt);
		} else {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"\tCharge Current: %d mA\n", charge_current);
		}
		aboot_wait_4_charging_video_debug_lines++;
	} else {
		if (charger_volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			ABOOT_WAIT_4_CHARGING_VIDEO_PRINT(1,
				"\tCharger Voltage: %d mV\n", charger_volt);
			aboot_wait_4_charging_video_debug_lines++;
		}
	}
}
#endif /* CONFIG_CFB_CONSOLE && CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO && CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG */

static int aboot_wait_for_charging_get_source(int *charger_source)
{
	int source;

	/* Avoid the debounce */
	mdelay(64);
#if defined(CONFIG_USB_OTG_TRANSCEIVER)
	otg_handle_interrupts(1);
#endif

	source = power_supply_get_charger_source();
	if (source == POWER_SUPPLY_TYPE_UNKNOWN) {
		/* Retry to avoid the debounce */
		mdelay(64);
#if defined(CONFIG_USB_OTG_TRANSCEIVER)
		otg_handle_interrupts(1);
#endif
		source = power_supply_get_charger_source();
		if (source == POWER_SUPPLY_TYPE_UNKNOWN) {
			*charger_source = source;
			return -1;
		}
	}

	*charger_source = source;

	return 0;
}

static int aboot_wait_for_charging(int ac_poweron_volt, int usb_poweron_volt)
{
	const static int wakeup_volt = CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV;
	const static int wakeup_max_count = CONFIG_ABOOT_WAIT_FOR_WAKEUP_VOLTAGE_TIMES + 1;
	unsigned long charging_timeout = 0, charging_ticks = 0;
	int charger_source = power_supply_get_charger_source();
	int poweron_volt;
	int prev_charger_source = POWER_SUPPLY_TYPE_UNKNOWN;
	unsigned int charging_count = 0;
	unsigned int wakeup_count = 0;
	unsigned int skip_not_charging_count = CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES;
	unsigned int aboot_continue = 0;
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	unsigned int ext_fg_count = CONFIG_ABOOT_WAIT_FOR_EXTERNAL_FUELGAUGE_TIMES;
#endif

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT

#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
#error "Please define CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE for CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT"
#endif

#if (CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT < 0) || (CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT > 500)
#error "Invalid CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT (0-500)"
#endif

#if (CONFIG_ABOOT_WAIT_FOR_CHARGING_BOOT_CONTINUE_CHARGE_CURRENT < 0) || (CONFIG_ABOOT_WAIT_FOR_CHARGING_BOOT_CONTINUE_CHARGE_CURRENT > 1500)
#error "Invalid CONFIG_ABOOT_WAIT_FOR_CHARGING_BOOT_CONTINUE_CHARGE_CURRENT (0-1500)"
#endif

	unsigned int check_current_count = 0;
#endif /* CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT */

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
	unsigned int temp_fault_count = CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES;
#endif

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
	if (aboot_check_main_battery())
		return -999;
#endif

	aboot_wait_4_charging = 1;

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (!aboot_bat_locked && power_supply_is_battery_locked() > 0) {
		aboot_bat_locked = 1;
		PUTS("\n<<Battery was Locked?>>\n\n");
	}
#endif

	/* Update the first polling time */
	charging_timeout = timeout_init(CONFIG_USB_OTG_POLL_TIME<<1, &charging_ticks);

#ifdef CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE
	if (ac_poweron_volt <= 0) {
		/* AC charger */
		ac_poweron_volt = CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;
	}
#endif

	if (usb_poweron_volt <= 0) {
		/* USB AC/PC charger */
#ifdef CONFIG_ABOOT_MINIMAL_USB_AC_CHARGING_VOLTAGE
		if (charger_source == POWER_SUPPLY_TYPE_USB_DCP)
			usb_poweron_volt = CONFIG_ABOOT_MINIMAL_USB_AC_CHARGING_VOLTAGE + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;
		else
#endif
			usb_poweron_volt = CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;
	}

	if (charger_source == POWER_SUPPLY_TYPE_MAINS)
		poweron_volt = ac_poweron_volt;
	else
		poweron_volt = usb_poweron_volt;

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (!power_supply_has_external_fuelgauge()) {
		aboot_set_state(ABOOT_STATE_NOT_CHARGING);
	} else
#endif
		aboot_set_state(ABOOT_STATE_PRECHARGING);

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO
	/**
	 * 1. 'aboot_wait_4_charging' & 'aboot_state' are ready
	 * 2. We only support this feature while using the external AC/USB charger.
	 */
	if (
		power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS
#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_SUPPORT_USB_AC
			|| power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB_DCP
#endif
		) {
		aboot_video_init();
	}
#endif

	WATCHDOG_RESET();
	UART_FIFO_FLUSH_ONCE();

	/* Wait for the charging */
	do {
		if (aboot_wait_for_charging_get_source(&charger_source) != 0) {
			/* Get the battery voltage */
			aboot_vbat = power_supply_get_battery_voltage();
			PRINT("\n<%d> Charger was unplugged (Main battery %dmV is too low)!\n\n", charger_source, aboot_vbat);
			aboot_set_state(ABOOT_STATE_VBAT_LOW);
			return -1;
		}

		if (charger_source != prev_charger_source) {
#ifdef CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON
#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
#error "CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER is not defined"
#endif
			if (prev_charger_source == POWER_SUPPLY_TYPE_MAINS && charger_source != POWER_SUPPLY_TYPE_MAINS) {
				/* Get the battery voltage */
				aboot_vbat = power_supply_get_battery_voltage();
				PRINT("\n<%d> <USB CHARGER NOT ALLOWED> (Main battery %dmV is too low)!\n\n", charger_source, aboot_vbat);
				aboot_set_state(ABOOT_STATE_VBAT_LOW);
				return -1;
			}
#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */

			prev_charger_source = charger_source;
			if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
				PRINT("\n<%d> Allowed AC wake-up volt: %d mV; power-on volt: %d mV\n",
						charger_source, wakeup_volt, ac_poweron_volt);
				poweron_volt = ac_poweron_volt;
			} else {
				PRINT("\n<%d> Allowed USB wake-up volt: %d mV; power-on volt: %d mV\n",
						charger_source, wakeup_volt, usb_poweron_volt);
				poweron_volt = usb_poweron_volt;
			}
			PRINT("Main battery (%dmV/%dmV) is too low. Wait for %s...\n\n",
					aboot_vbat, power_supply_get_sys_voltage(),
					(aboot_vbat <= wakeup_volt) ? "trickle charging" : "pre-charging");
		}

#if defined(CONFIG_OMAP44XX)
		mdelay(CONFIG_USB_OTG_POLL_TIME>>2);
#else
		mdelay(CONFIG_USB_OTG_POLL_TIME);
#endif

#ifdef CONFIG_ABOOT_DUMP_CHARGING_REGS
		power_supply_dump_regs();
#endif /* CONFIG_ABOOT_DUMP_CHARGING_REGS */

		if (timeout_check(&charging_timeout, &charging_ticks)) {
			int check_charging = 1;

			WATCHDOG_RESET();
			UART_FIFO_FLUSH_ONCE();

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
			if (!power_supply_has_external_fuelgauge()) {
				if (power_supply_reinit_external_fuelgauge()) {
					ext_fg_count = CONFIG_ABOOT_WAIT_FOR_EXTERNAL_FUELGAUGE_TIMES;
					if (power_supply_is_battery_locked() > 0) {
						aboot_bat_locked = 1;
						POWER_SUPPLY_INFO("<<Battery was Locked?>> use external Fuel Gauge [%u]\n\n",
								ext_fg_count);
					} else {
						POWER_SUPPLY_INFO("use external Fuel Gauge [%u]\n\n", ext_fg_count);
					}

					/* Get the new battery voltage from the external fuel gauge */
					aboot_vbat = power_supply_get_battery_voltage();
					if (aboot_vbat != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE &&
							power_supply_has_external_fuelgauge()) {
						/* Restart the charger */
#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT
						check_current_count = 0;
#endif
						(void)power_supply_clear_charger_error();
						(void)power_supply_start_charger(aboot_vbat);
					}
				} else {
					--ext_fg_count;
					if (ext_fg_count > 0) {
						if (aboot_state != ABOOT_STATE_NOT_CHARGING)
							aboot_set_state(ABOOT_STATE_NOT_CHARGING);
						POWER_SUPPLY_INFO("no external Fuel Gauge! [%u/%u]\n\n",
								ext_fg_count, CONFIG_ABOOT_WAIT_FOR_EXTERNAL_FUELGAUGE_TIMES);
					} else {
						PRINT("\nTimeout in waiting for external Fuel Gauge!\n");
						PRINT("<%d> <NO ext Fuel Gauge> Main battery (%dmV/%dmV) is too low!\n\n",
								charger_source, aboot_vbat, power_supply_get_sys_voltage());
						aboot_set_state(ABOOT_STATE_VBAT_LOW);
						return -1;
					}
				}
			}
#endif

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
			if (aboot_check_main_battery())
				return -999;
#endif

			/* Get the battery voltage */
			aboot_vbat = power_supply_get_battery_voltage();
			/* Update the next polling time */
			charging_timeout = timeout_init(CONFIG_ABOOT_WAIT_FOR_CHARGING_POLL_TIME, &charging_ticks);

			if (aboot_vbat <= wakeup_volt) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
				if (!power_supply_has_external_fuelgauge()) {
					if (aboot_state != ABOOT_STATE_NOT_CHARGING)
						aboot_set_state(ABOOT_STATE_NOT_CHARGING);
				} else
#endif
					if (aboot_state != ABOOT_STATE_TRICKLE_CHARGING)
						aboot_set_state(ABOOT_STATE_TRICKLE_CHARGING);

				if (wakeup_count++ <= wakeup_max_count) {
					skip_not_charging_count = CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES;
					check_charging = 0;
				} else {
					PRINT("\nTimeout in waiting for wakeup-charging! [%u]\n", skip_not_charging_count);
					check_charging = 1;
				}
			}

			if (check_charging) {
				int charging_status = power_supply_is_charging();
				if (charging_status == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
					wakeup_count = 0;
					skip_not_charging_count = CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES;
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
					temp_fault_count = CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES;
#endif
				} else {
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
					if (charging_status == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT) { /* Battery Temp Fault */
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
						if (aboot_check_main_battery())
							return -999;
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

						--temp_fault_count;
						if (temp_fault_count > 0) {
							POWER_SUPPLY_INFO("Battery Temp Fault! [%u/%u]\n\n",
									temp_fault_count, CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES);
						} else {
							PRINT("\nTimeout in waiting for Battery Temp Good!\n");
							PRINT("<%d> <Battery Temp Fault> Main battery (%dmV/%dmV) is too low!\n\n",
									charger_source, aboot_vbat, power_supply_get_sys_voltage());
							aboot_set_state(ABOOT_STATE_VBAT_LOW);
							return -1;
						}
					} else {
						temp_fault_count = CONFIG_ABOOT_WAIT_FOR_BAT_TEMP_FAULT_TIMES;
						if (charging_status == POWER_SUPPLY_CHARGER_STATE_NO_BATTERY) { /* no Main Battery */
							if (aboot_check_main_battery())
								return -999;
						}
					}
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY || CONFIG_ABOOT_POWERON_CHECK_BATTERY */

					/* Check the charger source again if it's NOT charging */
					if (aboot_wait_for_charging_get_source(&charger_source) != 0) {
						/* Get the battery voltage */
						aboot_vbat = power_supply_get_battery_voltage();
						PRINT("\n<%d> Charger was unplugged (Main battery %dmV is too low)!\n\n",
								charger_source, aboot_vbat);
						aboot_set_state(ABOOT_STATE_VBAT_LOW);
						return -1;
					}

					if (skip_not_charging_count > 0) {
						PRINT("<%d> <NOT Charging> retry...count [%u/%u]\n",
								charger_source, skip_not_charging_count, CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES);
						--skip_not_charging_count;
						if (aboot_state != ABOOT_STATE_NOT_CHARGING)
							aboot_set_state(ABOOT_STATE_NOT_CHARGING);
					} else {
						PRINT("<%d> <NOT Charging> Main battery (%dmV/%dmV) is too low!\n\n",
								charger_source, aboot_vbat, power_supply_get_sys_voltage());
						aboot_set_state(ABOOT_STATE_VBAT_LOW);
						return -1;
					}
				}
			}

			if (aboot_vbat > wakeup_volt &&
					skip_not_charging_count == CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES) {
				/* Charging */
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
				if (!power_supply_has_external_fuelgauge()) {
					if (aboot_state != ABOOT_STATE_NOT_CHARGING)
						aboot_set_state(ABOOT_STATE_NOT_CHARGING);
				} else
#endif
					if (aboot_state != ABOOT_STATE_PRECHARGING)
						aboot_set_state(ABOOT_STATE_PRECHARGING);
			}

			if (aboot_vbat > poweron_volt
					&& /*Charging*/ skip_not_charging_count == CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
				if (power_supply_is_battery_locked() > 0) {
					aboot_bat_locked = 1;
					aboot_continue = 0;
					PUTS("<<Battery was Locked?>>\n");
				} else {
					aboot_bat_locked = 0;
					aboot_continue++;
				}
#else
				aboot_continue++;
#endif
			} else {
				aboot_continue = 0;
			}

			if (aboot_continue <= CONFIG_ABOOT_WAIT_FOR_CHARGING_CONTINUE_BOOT_COUNT) {
				int charge_currentmA = power_supply_get_battery_current();
				int bat_capacity = power_supply_has_external_fuelgauge() ?
						power_supply_get_battery_capacity() : -1;
				int vsys = power_supply_get_sys_voltage();
				int charger_volt = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
				const char *wait_for_charging_status;

				if (bat_capacity >= 0)
					PRINT("Battery Capacity: %d%%\n", bat_capacity);

				if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
					charger_volt = power_supply_get_ac_voltage();
					PRINT("<%d> AC charger voltage: %d mV\n", charger_source, charger_volt);
				} else if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
					charger_volt = power_supply_get_usb_voltage();
					PRINT("<%d> USB charger voltage: %d mV\n", charger_source, charger_volt);
				}

#ifdef CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT
				if (power_supply_has_external_fuelgauge()) {
					if (check_charging &&
							charge_currentmA < CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT) {
						aboot_continue = 0; /* Don't exit because the charge current is too LOW! */
						++check_current_count;
						PRINT("[%u] <%d> Charge Current: %d mA is too LOW (%dmA)\n",
								check_current_count, charger_source, charge_currentmA,
								CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT);
						if ((check_current_count & 0x01) == 0) {
							if (power_supply_add_charger_error() ||
									(/*Not Charging*/ skip_not_charging_count
										!= CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES)) {
								(void)power_supply_start_charger(aboot_vbat);
							}
						}
					} else {
						if (charge_currentmA <= CONFIG_ABOOT_WAIT_FOR_CHARGING_BOOT_CONTINUE_CHARGE_CURRENT) {
							aboot_continue = 0; /* Don't exit because the charge current is not high enough! */
						}
						check_current_count = 0;
						PRINT("<%d> Charge Current: %d mA\n", charger_source, charge_currentmA);
						if (power_supply_clear_charger_error())
							(void)power_supply_start_charger(aboot_vbat);
					}
				} else {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
					aboot_continue = 0; /* Don't exit because no external Fuel Gauge! */
#endif
					if (charge_currentmA != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
						PRINT("<%d> Charge Current: %d mA\n", charger_source, charge_currentmA);
				}
#else /* !CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT */
				if (charge_currentmA != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
					PRINT("<%d> Charge Current: %d mA\n", charger_source, charge_currentmA);
#endif /* CONFIG_ABOOT_WAIT_FOR_CHARGING_CHECK_CHARGE_CURRENT */

				wait_for_charging_status =
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
					(!power_supply_has_external_fuelgauge()) ? "NO ext Fuel Gauge" :
						(skip_not_charging_count == CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES) ? "Charging" :
							(aboot_vbat <= wakeup_volt) ? "Unknown" : "NOT Charging";
#else
					(skip_not_charging_count == CONFIG_ABOOT_WAIT_FOR_NOT_CHARGING_TIMES) ? "Charging" :
						(aboot_vbat <= wakeup_volt) ? "Unknown" : "NOT Charging";
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE */

				charging_count++;

				PRINT("<<COUNTER>> Continue: [%u/%u]; Charging: [%u/%u]; Wakeup: [%u/%u]\n",
						aboot_continue, CONFIG_ABOOT_WAIT_FOR_CHARGING_CONTINUE_BOOT_COUNT,
						charging_count, CONFIG_ABOOT_WAIT_FOR_CHARGING_TIMES,
						wakeup_count, wakeup_max_count);
				PRINT("<%d> Wake-up volt: %d mV; Power-on volt: %d mV\n",
						charger_source, wakeup_volt, poweron_volt);
				PRINT("<%s> Main battery (%dmV/%dmV) is too low. Wait for %s...\n\n",
						wait_for_charging_status, aboot_vbat, vsys,
						(aboot_vbat <= wakeup_volt) ? "trickle charging" : "pre-charging");

#if defined(CONFIG_CFB_CONSOLE) && defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO) \
	&& defined(CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG) && (CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG > 0)
				aboot_wait_for_charging_video_debug(wait_for_charging_status, poweron_volt,
					aboot_vbat, bat_capacity, vsys, charge_currentmA, charger_volt);
#endif /* CONFIG_CFB_CONSOLE && CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO && CONFIG_ABOOT_WAIT_FOR_CHARGING_SHOW_LOGO_DEBUG */

				if (charging_count > CONFIG_ABOOT_WAIT_FOR_CHARGING_TIMES && aboot_vbat <= poweron_volt) {
					if (aboot_state != ABOOT_STATE_CHARGING_TIMEDOUT)
						aboot_set_state(ABOOT_STATE_CHARGING_TIMEDOUT);

					PRINT("<%d> <Timeout> Main battery (%dmV/%dmV) is too low!\n\n",
							charger_source, aboot_vbat, vsys);
					return -1;
				}
			}
		} /* timeout_check */
	} while (aboot_continue <= CONFIG_ABOOT_WAIT_FOR_CHARGING_CONTINUE_BOOT_COUNT);

	aboot_wait_4_charging = 0;

	return 0;
}
#endif /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

/*
 * Check the minimal power-on voltage
 *
 * Return value:
 *    >  0 => Passed: Charging and allowed to power on
 *    == 0 => Passed: Not charging but allowed to power on
 *    <  0 => Failed: Invalid charger or not charging (not allowed to power on).
 */
static int aboot_check_poweron_voltage(void)
{
	int minimal_vbat = 0;

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	aboot_vbat = power_supply_get_sys_voltage();

#if (CONFIG_ABOOT_MINIMAL_VOLTAGE > 0)
	minimal_vbat = CONFIG_ABOOT_MINIMAL_VOLTAGE;

	ABOOT_PRINT("VSYS: %dmV; MINI Threshold: %dmV\n", aboot_vbat, minimal_vbat);
	if (aboot_vbat < minimal_vbat) {
		PRINT("<NO Main Battery> VSYS (%dmV) is too low! (%dmV)\n\n",
				aboot_vbat, minimal_vbat);
		aboot_set_state(ABOOT_STATE_VBAT_LOW);
		return -1;
	} else
#endif
		PRINT("<NO Main Battery> VSYS %dmV\n\n", aboot_vbat);

	return 0; /* Not charging but allowed to power on */

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
	if (aboot_mode == ABOOT_MODE_FACTORY || aboot_reason == ABOOT_REASON_PCBA)
		ABOOT_PRINT("<skip to check the Main Battery>\n\n");
	else if (aboot_check_main_battery())
		return -999;
#endif

	aboot_vbat = power_supply_get_battery_voltage();

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
		/* External AC charger */
		int poweron_volt;

		if (!power_supply_has_external_fuelgauge() &&
				aboot_vbat > (poweron_volt - CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY)) {
#ifndef CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS
			mdelay(150);
#endif /* !CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS */
			aboot_vbat = power_supply_get_battery_voltage();
		}

#if defined(CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE) && (CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE > 0)

		if (aboot_mode == ABOOT_MODE_FACTORY || aboot_reason == ABOOT_REASON_PCBA) {
			ABOOT_PRINT("<skip to check the power-on voltage (%dmV)>\n\n", aboot_vbat);
		} else {
			int enter_wait_4_charging = 0;

			minimal_vbat = poweron_volt = CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE;
			poweron_volt += CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;

#if (CONFIG_ABOOT_MINIMAL_FASTBOOT_AC_CHARGING_VOLTAGE > 0)
			if (aboot_reason == ABOOT_REASON_FASTBOOT) {
				minimal_vbat = CONFIG_ABOOT_MINIMAL_FASTBOOT_AC_CHARGING_VOLTAGE;
				poweron_volt = minimal_vbat + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;

				if (!aboot_cold_reset)
					minimal_vbat -= CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY;
			}
#endif

			ABOOT_PRINT("VBAT: %dmV; MINI Threshold: %dmV; POWER-ON: %dmV\n",
					aboot_vbat, minimal_vbat, poweron_volt);

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
			if (aboot_check_main_battery())
				return -999;
#endif

			if (aboot_vbat < minimal_vbat) {
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT)
				if (aboot_vbat == CONFIG_POWER_SUPPLY_INVALID_VOLTAGE && !aboot_cold_reset
						&& !power_supply_has_external_fuelgauge()) {
					/**
					 * 1. VBAT is invalid
					 * 2. WARM reset
					 * 3. CANNOT find external Fuel Gauge
					 */
					PUTS("<<WARM>><NO ext Fuel Gauge> VBAT is invalid\n");
				} else
#endif
					enter_wait_4_charging = 1;
			} else if (!power_supply_has_external_fuelgauge()) {
				int charging_status = power_supply_is_charging();
				if (charging_status <= 0) {
					if (aboot_cold_reset) {
						enter_wait_4_charging = 1;
#ifdef CONFIG_ABOOT_POWERON_SKIP_BAT_TEMP_FAULT
						if (charging_status == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT)
							enter_wait_4_charging = 0;
#endif
					} else {
						/**
						 * 1. VBAT is high enough
						 * 2. WARM reset
						 */
#ifdef CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS
						PUTS("<<WARM>><NO ext Fuel Gauge><Not Charging> VBAT is VSYS\n");
#else
						PUTS("<<WARM>><NO ext Fuel Gauge><Not Charging> VBAT is high enough\n");
#endif /* !CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS */
					}
				}
			}

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
			if (!enter_wait_4_charging && aboot_cold_reset && power_supply_is_battery_locked() > 0) {
				aboot_bat_locked = 1;
				enter_wait_4_charging = 1;
				PUTS("\n<<COLD>> <<Battery was Locked?>>\n\n");
			}
#endif

			if (enter_wait_4_charging) {
				int ret = aboot_wait_for_charging(poweron_volt, -1);
				if (ret)
					return ret;
			} /* enter_wait_4_charging */
		}
		PRINT("<AC %sCharging> Main battery: %d mV (%dmV)\n\n",
				(power_supply_is_charging() == POWER_SUPPLY_CHARGER_STATE_CHARGING) ? "" : "NOT ",
				aboot_vbat, power_supply_get_sys_voltage());

		return 1; /* External AC charging and allowed to power on */

#else /* CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE <= 0 */

#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) || defined(CONFIG_ABOOT_POWERON_CHECK_BATTERY)
		if (aboot_check_main_battery())
			return -999;
#endif
		PRINT("<AC> Main battery: %d mV (%dmV)\n\n", aboot_vbat, power_supply_get_sys_voltage());
		return 0; /* Allowed to power on */

#endif /* CONFIG_ABOOT_MINIMAL_AC_CHARGING_VOLTAGE > 0 */

	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
		/* USB charger */
#if (CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE > 0)
		int poweron_volt = 0;
#endif

#ifdef CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON
		if (aboot_reason == ABOOT_REASON_CHARGER) {
			PRINT("\n<NOT AC> Power-off Charging only supports AC charger (%dmV)!\n", aboot_vbat);
			PUTS("\nPlease press POWER button to boot!\n\n");
			return -1;
		}
#endif

#ifdef CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON
		if (aboot_reason == ABOOT_REASON_PC_CHARGER) {
			PRINT("\n<NOT USB AC> Power-off Charging only supports AC charger (%dmV)!\n", aboot_vbat);
			PUTS("\nPlease press POWER button to boot!\n\n");
			return -1;
		}
#endif

#if (CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE > 0)
#if defined(CONFIG_ABOOT_MINIMAL_USB_AC_CHARGING_VOLTAGE) && (CONFIG_ABOOT_MINIMAL_USB_AC_CHARGING_VOLTAGE > 0)
		if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB_DCP)
			minimal_vbat = CONFIG_ABOOT_MINIMAL_USB_AC_CHARGING_VOLTAGE;
		else
#endif
			minimal_vbat = CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE;

		poweron_volt = minimal_vbat + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;

		if (aboot_mode == ABOOT_MODE_FACTORY || aboot_reason == ABOOT_REASON_PCBA) {
			ABOOT_PRINT("<skip to check the power-on voltage (%dmV)>\n\n", aboot_vbat);
		} else {
			int enter_wait_4_charging = 0;

			if (aboot_reason == ABOOT_REASON_FASTBOOT) {
				minimal_vbat = CONFIG_ABOOT_MINIMAL_FASTBOOT_USB_CHARGING_VOLTAGE;
				poweron_volt = minimal_vbat + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;

				if (!aboot_cold_reset)
					minimal_vbat -= CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY;
			}

			ABOOT_PRINT("VBAT: %dmV; MINI Threshold: %dmV; POWER-ON: %dmV\n",
					aboot_vbat, minimal_vbat, poweron_volt);

			if (aboot_vbat < minimal_vbat) {
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT)
				if (aboot_vbat == CONFIG_POWER_SUPPLY_INVALID_VOLTAGE && !aboot_cold_reset
						&& !power_supply_has_external_fuelgauge()) {
					/**
					 * 1. VBAT is invalid
					 * 2. WARM reset
					 * 3. CANNOT find external Fuel Gauge
					 */
					PUTS("<<WARM>><NO ext Fuel Gauge> VBAT is invalid\n");
				} else
#endif
					enter_wait_4_charging = 1;
			} else if (!power_supply_has_external_fuelgauge()) {
				int charging_status = power_supply_is_charging();
				if (charging_status <= 0) {
					if (aboot_cold_reset) {
						enter_wait_4_charging = 1;
					} else {
						/**
						 * 1. VBAT is high enough
						 * 2. WARM reset
						 */
#ifdef CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS
						PUTS("<<WARM>><NO ext Fuel Gauge><Not Charging> VBAT is VSYS\n");
#else
						PUTS("<<WARM>><NO ext Fuel Gauge><Not Charging> VBAT is high enough\n");
#endif /* !CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS */
					}
				}
			}

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
			if (!enter_wait_4_charging && aboot_cold_reset && power_supply_is_battery_locked() > 0) {
				aboot_bat_locked = 1;
				enter_wait_4_charging = 1;
				PUTS("\n<<COLD>> <<Battery was Locked?>>\n\n");
			}
#endif

			if (enter_wait_4_charging) {
#ifdef CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON

#ifndef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
#error "CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER is not defined"
#endif
				PRINT("\n<VBAT LOW> <USB CHARGER NOT ALLOWED>\n<%d> Main Battery (%dmV/%dmV) is too low! (%dmV)\n\n",
						power_supply_get_charger_source(),
						aboot_vbat, power_supply_get_sys_voltage(), minimal_vbat);
				aboot_set_state(ABOOT_STATE_VBAT_LOW);
				return -1;

#else /* !CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */

#ifdef CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON
				if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_USB_DCP
#ifdef CONFIG_ABOOT_HAS_WARM_RESET_POWEROFF_USB_PC_CHARGER
						&& aboot_cold_reset
#endif
					) {
					PRINT("\n<VBAT LOW> <USB PC CHARGER NOT ALLOWED>\n<%d> Main Battery (%dmV/%dmV) is too low! (%dmV)\n\n",
							power_supply_get_charger_source(),
							aboot_vbat, power_supply_get_sys_voltage(), minimal_vbat);
					aboot_set_state(ABOOT_STATE_VBAT_LOW);
					return -1;
				}
#endif /* CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON */

				{
					int ret = aboot_wait_for_charging(-1, poweron_volt);
					if (ret)
						return ret;
				}

#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */
			} /* enter_wait_4_charging */
		}
		PRINT("<USB %s %sCharging> Main battery: %dmV (%dmV)\n\n",
				(power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) ? "PC" : "AC",
				(power_supply_is_charging() == POWER_SUPPLY_CHARGER_STATE_CHARGING) ? "" : "NOT ",
				aboot_vbat, power_supply_get_sys_voltage());

		return 1; /* Charging and allowed to power on */

#else /* CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE <= 0 */

		PRINT("<USB %s> Main battery: %dmV (%dmV)\n\n",
				(power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) ? "PC" : "AC",
				aboot_vbat, power_supply_get_sys_voltage());
		return 0; /* Allowed to power on */

#endif /* CONFIG_ABOOT_MINIMAL_USB_CHARGING_VOLTAGE > 0 */

	} else {

#if (CONFIG_ABOOT_MINIMAL_VOLTAGE > 0)

#if (CONFIG_ABOOT_MINIMAL_FASTBOOT_VOLTAGE > 0)
		if (aboot_reason == ABOOT_REASON_FASTBOOT) {
#if defined(CONFIG_ABOOT_MINIMAL_FASTBOOT_USB_CHARGING_VOLTAGE) && (CONFIG_ABOOT_MINIMAL_FASTBOOT_USB_CHARGING_VOLTAGE > 0)
			if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) {
				minimal_vbat = CONFIG_ABOOT_MINIMAL_FASTBOOT_USB_CHARGING_VOLTAGE;
			} else
#endif
				minimal_vbat = CONFIG_ABOOT_MINIMAL_FASTBOOT_VOLTAGE;

			if (!aboot_cold_reset)
				minimal_vbat -= CONFIG_POWER_SUPPLY_BATTERY_VOLTAGE_INACCURACY;
		} else
#endif /* CONFIG_ABOOT_MINIMAL_FASTBOOT_VOLTAGE > 0 */
			minimal_vbat = CONFIG_ABOOT_MINIMAL_VOLTAGE;

		ABOOT_PRINT("VBAT: %dmV; MINI Threshold: %dmV\n", aboot_vbat, minimal_vbat);

		if (aboot_vbat < minimal_vbat) {
			if (aboot_mode == ABOOT_MODE_FACTORY || aboot_reason == ABOOT_REASON_PCBA) {
				ABOOT_PRINT("<skip to check the power-on voltage (%dmV)>\n\n", aboot_vbat);
			} else {
				if (aboot_reason == ABOOT_REASON_FASTBOOT) {
					int poweron_volt = 0;

					PRINT("<Fastboot> Main Battery (%dmV/%dmV) is too low! (%dmV)\n\n",
							aboot_vbat, power_supply_get_sys_voltage(), minimal_vbat);

					if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) {
						poweron_volt = minimal_vbat + CONFIG_ABOOT_EXTRA_CHARGING_VOLTAGE;

						ABOOT_PRINT("VBAT: %dmV; MINI Threshold: %dmV; POWER-ON: %dmV\n",
								aboot_vbat, minimal_vbat, poweron_volt);
						int ret = aboot_wait_for_charging(-1, poweron_volt);
						if (ret)
							return ret;

						PRINT("<Fastboot> Main battery: %dmV (%dmV)\n\n",
								aboot_vbat, power_supply_get_sys_voltage());
						return 1; /* Charging and allowed to power on */
					}
				} else {
					PRINT("<%s> Main Battery (%dmV/%dmV) is too low! (%dmV)\n\n",
							(power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) ? "PC Charger" : "NOT Charging",
							aboot_vbat, power_supply_get_sys_voltage(), minimal_vbat);
				}

				aboot_set_state(ABOOT_STATE_VBAT_LOW);
				return -1;
			}
		} else {
			PRINT("<%s> Main battery: %dmV (%dmV)\n\n",
					(power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) ? "PC Charger" : "NOT Charging",
					aboot_vbat, power_supply_get_sys_voltage());
		}

#else /* CONFIG_ABOOT_MINIMAL_VOLTAGE <= 0 */

		PRINT("Main battery: %dmV (%dmV)\n\n", aboot_vbat, power_supply_get_sys_voltage());
		return 0; /* Allowed to power on */

#endif /* CONFIG_ABOOT_MINIMAL_VOLTAGE > 0 */
	}

	return 0; /* Not charging but allowed to power on */

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}
#else
#define aboot_check_poweron_voltage()	(0)
#endif /* CONFIG_SYS_POWER_SUPPLY */

#ifdef CONFIG_USB_OTG_TRANSCEIVER
static struct otg_transceiver *otg;
static struct notifier_block otg_notifier;

static int aboot_otg_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
    switch (event) {
    case USB_EVENT_VBUS:
		aboot_usb_type = *((unsigned *)data);
		if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was plugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
        break;
    case USB_EVENT_ENUMERATED:
		if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was plugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
        break;
    case USB_EVENT_CHARGER:
		aboot_usb_type = POWER_SUPPLY_TYPE_USB_DCP;
		if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was plugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
        break;
    case USB_EVENT_NONE:
		aboot_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER) {
			if (power_supply_has_ac_feature() && aboot_ac_type == POWER_SUPPLY_TYPE_MAINS)
				break;
			if (power_supply_has_usb_feature()) {
				PRINT("\n<USB CHARGER> Charger was unplugged (%dmV)!\n\n", aboot_vbat);
				if (!aboot_wait_4_charging)
					board_poweroff();
			}
		} else if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was unplugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
        break;
    case USB_EVENT_ID:
    default:
		aboot_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER) {
			if (power_supply_has_ac_feature() && aboot_ac_type == POWER_SUPPLY_TYPE_MAINS)
				break;
			if (power_supply_has_usb_feature()) {
				PRINT("\n<USB CHARGER> Charger was unplugged (%dmV)!\n\n", aboot_vbat);
				if (!aboot_wait_4_charging)
					board_poweroff();
			}
		}
		break;
    }

	return 0;
}
#endif /* CONFIG_USB_OTG_TRANSCEIVER */

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
static struct notifier_block ac_notifier;

static int aboot_ac_event_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
   	switch (event) {
   	case POWER_SUPPLY_AC_EVENT_CHARGER:
		aboot_ac_type = POWER_SUPPLY_TYPE_MAINS;
		if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was plugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
       	break;
   	case POWER_SUPPLY_AC_EVENT_NONE:
	default:
		aboot_ac_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (aboot_reason == ABOOT_REASON_CHARGER) {
			if (power_supply_has_usb_feature() && aboot_usb_type != POWER_SUPPLY_TYPE_UNKNOWN)
				break;

			if (power_supply_has_ac_feature()) {
				PRINT("\n<AC CHARGER> Charger was unplugged (%dmV)!\n\n", aboot_vbat);
				if (!aboot_wait_4_charging)
					board_poweroff();
			}
		} else if (!aboot_wait_4_charging) {
			if (aboot_state == ABOOT_STATE_BOOT_ANDROID)
				/* USB charger was unplugged so we have to update aboot state again */
				aboot_set_state(aboot_state);
		}
#ifdef CONFIG_ABOOT_POWERON_HAS_VAC_CHECK
		if (power_supply_has_ac_feature() && !board_is_factory_mode()) {
			PRINT("\n<AC CHARGER> Charger was unplugged (%dmV)!\n", aboot_vbat);
			PUTS("<NOT ALLOWED> Please plugin External AC charger!\n\n");
			board_poweroff();
		}
#endif /* CONFIG_ABOOT_POWERON_HAS_VAC_CHECK */
       	break;
   	}

	return 0;
}
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

void aboot_register_last_otg_notifier(void)
{
#ifdef CONFIG_USB_OTG_TRANSCEIVER
	int ret;

	otg = otg_get_transceiver();
	otg_notifier.notifier_call = aboot_otg_notifier_call;
	otg_notifier.priority = INT_MIN;
	ret = otg_register_notifier(otg, &otg_notifier);
	if (ret)
		ABOOT_PRINT("otg err %d\n", ret);
#endif /* CONFIG_USB_OTG_TRANSCEIVER */

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	ac_notifier.notifier_call = aboot_ac_event_notifier_call;
	ret = ac_charger_register_notifier(&ac_notifier);
	if (ret)
		ABOOT_PRINT("AC err %d\n", ret);
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
}

/*-------------------------------------------------------------------------*/

u32 aboot_get_reason(void)
{
	return aboot_reason;
}

void aboot_set_mode(u32 mode)
{
	switch (mode) {
	case ABOOT_MODE_CHARGER:
		PUTS("< CHARGER mode >\n");
		break;
	case ABOOT_MODE_FACTORY:
		PUTS("< FACTORY mode >\n");
		break;
	case ABOOT_MODE_NORMAL:
	default:
		PUTS("< NORMAL mode >\n");
		break;
	}
	aboot_mode = mode;
}

u32 aboot_get_mode(void)
{
	return aboot_mode;
}

/*-------------------------------------------------------------------------*/

int aboot_initiated(void)
{
	return ainitiated;
}

int aboot_init(void)
{
	int ret;

	ainitiated = 1;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	aboot_reason = aboot_board_get_poweron_reason(&aboot_cold_reset);
#ifdef CONFIG_ABOOT_DEBUG
	switch (aboot_reason) {
	case ABOOT_REASON_POWEROFF:
		ABOOT_DPUTS("< POWEROFF reason >\n");
		break;
	case ABOOT_REASON_NORMAL:
		ABOOT_DPUTS("< NORMAL reason >\n");
		break;
	case ABOOT_REASON_CHARGER:
		ABOOT_DPUTS("< CHARGER reason >\n");
		break;
	case ABOOT_REASON_PC_CHARGER:
		ABOOT_DPUTS("< PC CHARGER reason >\n");
		break;
	case ABOOT_REASON_RECOVERY:
		ABOOT_DPUTS("< RECOVERY reason >\n");
		break;
	case ABOOT_REASON_FASTBOOT:
		ABOOT_DPUTS("< FASTBOOT reason >\n");
		break;
	case ABOOT_REASON_ALARM:
		ABOOT_DPUTS("< ALARM reason >\n");
		break;
	case ABOOT_REASON_PCBA:
		ABOOT_DPUTS("< PCBA reason >\n");
		break;
	default:
		ABOOT_DPUTS("< UNKNOWN reason >\n");
		break;
	}
#endif /* CONFIG_ABOOT_DEBUG */
	putc('\n');

	if (aboot_reason == ABOOT_REASON_POWEROFF) {
		board_poweroff(); /* Shutdown immediately */
	} else {
		int ret;

		aboot_partition_table_init();

		ret = aboot_check_poweron_voltage();
		if (ret < 0) {
			if (ret == -999) {
				/* no Main Battery */
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY)
				if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
					if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER) {
						if (aboot_cold_reset) {
#ifdef CONFIG_ABOOT_NO_CHARGER_POWERON_IF_NO_BATTERY
							PUTS("\n<AC> <CHARGER> <COLD RESET NOT ALLOWED>\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							board_poweroff();
#else
							PUTS("\n<AC> <CHARGER> <Support NO Main Battery> < switch to NORMAL reason >\n\n");
							aboot_reason = ABOOT_REASON_NORMAL;
#endif
						} else {
							PUTS("\n<AC> <CHARGER> <WARM RESET NOT ALLOWED>\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							board_poweroff();
						}
					} else {
						PUTS("\n<AC CHARGER> <Support NO Main Battery>\n\n");
					}
				} else {
					PUTS("\n<NOT AC CHARGER> <NOT ALLOWED>\n");
					PUTS("\nPlease insert Main Battery and press POWER button to boot!\n\n");
					aboot_set_state(ABOOT_STATE_VBAT_LOW);
					board_poweroff();
				}
#else /* CONFIG_ABOOT_POWERON_CHECK_BATTERY */
				PUTS("\n<NO Main Battery> <NOT ALLOWED>\n");
				PUTS("\nPlease insert Main Battery and press POWER button to boot!\n\n");
				aboot_set_state(ABOOT_STATE_VBAT_LOW);
				board_poweroff();
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
			} else {
				board_poweroff();
			}
		}
	}

#if defined(CONFIG_ABOOT_POWERON_VIBRATION_TIME) && (CONFIG_ABOOT_POWERON_VIBRATION_TIME > 0)
	if (aboot_cold_reset)
		vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME);
#endif /* CONFIG_ABOOT_POWERON_VIBRATION_TIME */

#if defined(CONFIG_CFB_CONSOLE)
#if defined(CONFIG_ABOOT_DISPLAY_DEV_FASTBOOT_NO_BOOTLOGO)
	/* DEBUG purpose for Display development */
	if (aboot_reason != ABOOT_REASON_FASTBOOT && aboot_reason != ABOOT_REASON_PCBA)
#endif
	{
		aboot_video_init();
#if defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) && defined(CONFIG_ABOOT_SHOW_NO_MAIN_BATTERY_INFO)
		if (aboot_has_no_main_battery && aboot_video_initiated
			&& power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS)
			video_printf("<No Main Battery>\n");
#endif
	}
#endif /* CONFIG_CFB_CONSOLE */

	UART_FIFO_FLUSH_ONCE();

#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
	if (aboot_reason == ABOOT_REASON_FASTBOOT || aboot_reason == ABOOT_REASON_PCBA)
#else
	if (aboot_reason == ABOOT_REASON_FASTBOOT)
#endif /* CONFIG_ABOOT_PCBA_USE_FASTBOOT */
	{
#ifdef CONFIG_USB_GADGET_FASTBOOT
#if defined(CONFIG_USB_OTG_TRANSCEIVER)
		otg_handle_interrupts(1);
#endif
		ret = fastboot_init();
		if (!ret) {
			aboot_set_state(ABOOT_STATE_BOOT_FASTBOOT);
			while (1) {
				ret = fastboot_poll();
				if (ret)
					break;
			}
			fastboot_shutdown();

			switch (ret) {
			case FASTBOOT_EXIT_REBOOT_BOOTLOADER:
				PUTS("<FASTBOOT> Rebooting to Bootloader ...\n");
				board_reboot("bootloader");
				break;
			case FASTBOOT_EXIT_REBOOT:
				PUTS("<FASTBOOT> Rebooting ...\n");
				board_reboot(NULL);
				break;
			case FASTBOOT_EXIT_SHUTDOWN:
				PUTS("<FASTBOOT> Shutting down ...\n");
				board_poweroff();
				break;
			case FASTBOOT_EXIT_CONTINUE:
			default:
				if (aboot_mode == ABOOT_MODE_FACTORY && aboot_reason == ABOOT_REASON_PCBA &&
						aboot_usb_type == POWER_SUPPLY_TYPE_USB) {
					/* Force to shutdown the device */
					PUTS("<FASTBOOT> <FACTORY> <PCBA> Force to Shutdown ...\n");
					board_poweroff();
				}
				PUTS("<FASTBOOT> Continue the booting ...\n");
				break;
			}
		}
#else
		/* Because Fastboot gadget is not enabled, we apply Android Recovery instead. */
		PUTS("<no FASTBOOT support> < switch to RECOVERY reason >\n");
		aboot_reason = ABOOT_REASON_RECOVERY;
#endif /* CONFIG_USB_GADGET_FASTBOOT */
	}

#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
	if (aboot_reason == ABOOT_REASON_RECOVERY)
#else
	if (aboot_reason == ABOOT_REASON_RECOVERY || aboot_reason == ABOOT_REASON_PCBA)
#endif /* CONFIG_ABOOT_PCBA_USE_FASTBOOT */
		aboot_set_state(ABOOT_STATE_BOOT_RECOVERY);
	else {
		aboot_parse_recovery_message();

		if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER)
			aboot_set_state(ABOOT_STATE_BOOT_CHARGER);
		else if (aboot_reason == ABOOT_REASON_RECOVERY)
			aboot_set_state(ABOOT_STATE_BOOT_RECOVERY);
		else
			aboot_set_state(ABOOT_STATE_BOOT_ANDROID);
	}

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	UART_FIFO_FLUSH_ONCE();

	return 0;
}

/*-------------------------------------------------------------------------*/

static void announce_and_cleanup(void)
{
	PUTS("\nStarting kernel ...\n\n");
	bootstage_mark_name(BOOTSTAGE_ID_BOOTM_HANDOFF, "start_kernel");
#ifdef CONFIG_BOOTSTAGE_REPORT
	bootstage_report();
#endif

#ifdef CONFIG_USB_DEVICE
	udc_disconnect();
#endif
	cleanup_before_linux();
}

#if defined(CONFIG_SETUP_MEMORY_TAGS) || \
	defined(CONFIG_CMDLINE_TAG) || \
	defined(CONFIG_INITRD_TAG) || \
	defined(CONFIG_SERIAL_TAG) || \
	defined(CONFIG_VIDEOLFB_TAG) || \
	defined(CONFIG_REVISION_TAG)
static void setup_start_tag (bd_t *bd)
{
	params = (struct tag *)bd->bi_boot_params;

	params->hdr.tag = ATAG_CORE;
	params->hdr.size = tag_size (tag_core);

	params->u.core.flags = 0;
	params->u.core.pagesize = 0;
	params->u.core.rootdev = 0;

	params = tag_next (params);
}
#endif

#ifdef CONFIG_SETUP_MEMORY_TAGS
static void setup_memory_tags(bd_t *bd)
{
	int i;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		params->hdr.tag = ATAG_MEM;
		params->hdr.size = tag_size (tag_mem32);

		params->u.mem.start = bd->bi_dram[i].start;
		params->u.mem.size = bd->bi_dram[i].size;

		params = tag_next (params);
	}
}
#endif

#ifdef CONFIG_CMDLINE_TAG
static void setup_commandline_tag(bd_t *bd, char *commandline)
{
	char *p;

	if (!commandline)
		return;

	/* eat leading white space */
	for (p = commandline; *p == ' '; p++);

	/* skip non-existent command lines so the kernel will still
	 * use its default command line.
	 */
	if (*p == '\0')
		return;

	params->hdr.tag = ATAG_CMDLINE;
	params->hdr.size =
		(sizeof (struct tag_header) + strlen (p) + 1 + 4) >> 2;

	strcpy (params->u.cmdline.cmdline, p);

	params = tag_next (params);
}
#endif

#ifdef CONFIG_INITRD_TAG
static void setup_initrd_tag (bd_t *bd, ulong initrd_start, ulong initrd_size)
{
	/* an ATAG_INITRD node tells the kernel where the compressed
	 * ramdisk can be found. ATAG_RDIMG is a better name, actually.
	 */
	params->hdr.tag = ATAG_INITRD2;
	params->hdr.size = tag_size (tag_initrd);

	params->u.initrd.start = initrd_start;
	params->u.initrd.size = initrd_size;

	params = tag_next (params);
}
#endif

#ifdef CONFIG_SERIAL_TAG
static void setup_serial_tag(struct tag **tmp)
{
	struct tag *params = *tmp;
	struct tag_serialnr serialnr;
	void get_board_serial(struct tag_serialnr *serialnr);

	get_board_serial(&serialnr);
	params->hdr.tag = ATAG_SERIAL;
	params->hdr.size = tag_size (tag_serialnr);
	params->u.serialnr.low = serialnr.low;
	params->u.serialnr.high= serialnr.high;
	params = tag_next (params);
	*tmp = params;
}
#endif

#ifdef CONFIG_VIDEOLFB_TAG
static u32 __videolfb_get_fbbase(void) { return 0; }
static u32 __videolfb_get_fbsize(void) { return 0; }

u32 videolfb_get_fbbase(void) __attribute__((weak, alias("__videolfb_get_fbbase")));
u32 videolfb_get_fbsize(void) __attribute__((weak, alias("__videolfb_get_fbsize")));

static void setup_videolfb_tag(gd_t *gd)
{
	/* An ATAG_VIDEOLFB node tells the Kernel where and how large
	 * the framebuffer for video was allocated (among other things).
	 * Note that a _physical_ address is passed !
	 *
	 * We only use it to pass the address and size, the other entries
	 * in the tag_videolfb are not of interest.
	 */
	params->hdr.tag = ATAG_VIDEOLFB;
	params->hdr.size = tag_size (tag_videolfb);

	params->u.videolfb.lfb_base = videolfb_get_fbbase();
	params->u.videolfb.lfb_size = videolfb_get_fbsize();

	params = tag_next (params);
}
#endif

#ifdef CONFIG_REVISION_TAG
static void setup_revision_tag(struct tag **in_params)
{
	u32 rev = 0;
	u32 get_board_rev(void);

	rev = get_board_rev();
	params->hdr.tag = ATAG_REVISION;
	params->hdr.size = tag_size (tag_revision);
	params->u.revision.rev = rev;
	params = tag_next (params);
}
#endif

#if defined(CONFIG_SETUP_MEMORY_TAGS) || \
	defined(CONFIG_CMDLINE_TAG) || \
	defined(CONFIG_INITRD_TAG) || \
	defined(CONFIG_SERIAL_TAG) || \
	defined(CONFIG_VIDEOLFB_TAG) || \
	defined(CONFIG_REVISION_TAG)
static void setup_end_tag(bd_t *bd)
{
	params->hdr.tag = ATAG_NONE;
	params->hdr.size = 0;
}
#endif

/*-------------------------------------------------------------------------*/

static size_t aboot_setup_linux_cmdline(char *cmdline)
{
	char* cmdline_start = cmdline;

#ifdef CONFIG_ABOOT_KERNEL_LOG_LEVEL
#if 0
/* Kernel wll show the log if it's level is less than CONFIG_ABOOT_KERNEL_LOG_LEVEL */
#define KERN_EMERG	"<0>"	/* system is unusable			*/
#define KERN_ALERT	"<1>"	/* action must be taken immediately	*/
#define KERN_CRIT	"<2>"	/* critical conditions			*/
#define KERN_ERR	"<3>"	/* error conditions			*/
#define KERN_WARNING	"<4>"	/* warning conditions			*/
#define KERN_NOTICE	"<5>"	/* normal but significant condition	*/
#define KERN_INFO	"<6>"	/* informational			*/
#define KERN_DEBUG	"<7>"	/* debug-level messages			*/
#endif
#if (CONFIG_ABOOT_KERNEL_LOG_LEVEL == 0)
	/* quiet */
	strcpy(cmdline, ABOOT_CMDLINE_LOG_LEVEL_QUIET);
	cmdline += sizeof(ABOOT_CMDLINE_LOG_LEVEL_QUIET) - 1;
#elif (CONFIG_ABOOT_KERNEL_LOG_LEVEL > 0) && (CONFIG_ABOOT_KERNEL_LOG_LEVEL < 7)
	/* loglevel */
	strcpy(cmdline, ABOOT_CMDLINE_LOG_LEVEL);
	cmdline += sizeof(ABOOT_CMDLINE_LOG_LEVEL) - 1;
	strcpy(cmdline, EXPAND_STR(CONFIG_ABOOT_KERNEL_LOG_LEVEL));
	cmdline += sizeof(EXPAND_STR(CONFIG_ABOOT_KERNEL_LOG_LEVEL)) - 1;
#elif (CONFIG_ABOOT_KERNEL_LOG_LEVEL == 7)
	/* Do nothing because the default log level is 7 */
#elif (CONFIG_ABOOT_KERNEL_LOG_LEVEL > 7)
	/* debug */
	strcpy(cmdline, ABOOT_CMDLINE_LOG_LEVEL_DEBUG);
	cmdline += sizeof(ABOOT_CMDLINE_LOG_LEVEL_DEBUG) - 1;
#else
#error "Invalid CONFIG_ABOOT_KERNEL_LOG_LEVEL (0~7)"
#endif
#endif /* CONFIG_ABOOT_KERNEL_LOG_LEVEL */

	/* androidboot.reason */
	strcpy(cmdline, ABOOT_CMDLINE_REASON);
	cmdline += sizeof(ABOOT_CMDLINE_REASON) - 1;
	switch (aboot_reason) {
#ifndef CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE
	case ABOOT_REASON_CHARGER:
	case ABOOT_REASON_PC_CHARGER:
		strcpy(cmdline, ABOOT_REASON_CHARGER_STR);
		cmdline += sizeof(ABOOT_REASON_CHARGER_STR) - 1;
#endif /* !CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE */
		break;
	case ABOOT_REASON_RECOVERY:
		strcpy(cmdline, ABOOT_REASON_RECOVERY_STR);
		cmdline += sizeof(ABOOT_REASON_RECOVERY_STR) - 1;
		break;
	case ABOOT_REASON_ALARM:
		strcpy(cmdline, ABOOT_REASON_ALARM_STR);
		cmdline += sizeof(ABOOT_REASON_ALARM_STR) - 1;
		break;
	case ABOOT_REASON_PCBA:
		strcpy(cmdline, ABOOT_REASON_PCBA_STR);
		cmdline += sizeof(ABOOT_REASON_PCBA_STR) - 1;
		break;
	default:
		strcpy(cmdline, ABOOT_REASON_NORMAL_STR);
		cmdline += sizeof(ABOOT_REASON_NORMAL_STR) - 1;
		break;
	}

	/* androidboot.mode */
	strcpy(cmdline, ABOOT_CMDLINE_MODE);
	cmdline += sizeof(ABOOT_CMDLINE_MODE) - 1;
	if (aboot_reason == ABOOT_REASON_CHARGER || aboot_reason == ABOOT_REASON_PC_CHARGER)
	{
#ifdef CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE
		strcpy(cmdline, ABOOT_MODE_NORMAL_STR);
		cmdline += sizeof(ABOOT_MODE_NORMAL_STR) - 1;
#else
		/* Android power-off charger */
		strcpy(cmdline, ABOOT_MODE_CHARGER_STR);
		cmdline += sizeof(ABOOT_MODE_CHARGER_STR) - 1;
#ifdef CONFIG_ABOOT_ANDROID_CHARGER_MODE_USE_1_CPU
		/* Maximum CPU number is 1 */
		strcpy(cmdline, ABOOT_CMDLINE_CHARGER_MODE_1_CPU);
		cmdline += sizeof(ABOOT_CMDLINE_CHARGER_MODE_1_CPU) - 1;
#endif
#endif /* CONFIG_ABOOT_NO_ANDROID_CHARGER_MODE */
	} else {
		switch (aboot_mode) {
		case ABOOT_MODE_FACTORY:
#if 0
			/* Low-level factory test */
			strcpy(cmdline, ABOOT_MODE_FACTORY_STR);
			cmdline += sizeof(ABOOT_MODE_FACTORY_STR) - 1;
#else
			/* High-level factory test */
			strcpy(cmdline, ABOOT_MODE_FACTORY2_STR);
			cmdline += sizeof(ABOOT_MODE_FACTORY2_STR) - 1;
#endif
			break;
		default:
			strcpy(cmdline, ABOOT_MODE_NORMAL_STR);
			cmdline += sizeof(ABOOT_MODE_NORMAL_STR) - 1;
			break;
		}
	}

	return (size_t)(cmdline - cmdline_start);
}

/*-------------------------------------------------------------------------*/

/* Subcommand: PREP */
static void boot_prep_linux(const char* bootargs, ulong initrd_entry, ulong initrd_size)
{
#ifdef CONFIG_CMDLINE_TAG
	size_t length;
	char *commandline;
	char *cmdline;

	cmdline = commandline = (char *)memalign(ARCH_DMA_MINALIGN, CONFIG_ABOOT_CMDLINE_SIZE);

	if (bootargs && ((length = strlen(bootargs)) > 0)) {
		strcpy(cmdline, bootargs);
		cmdline += length;
	}

	ABOOT_VPUTS("ABOOT Linux commands...\n");
	length = aboot_setup_linux_cmdline(cmdline);
	if (length > 0)
		cmdline += length;

	ABOOT_VPUTS("Arch-specific Linux commands...\n");
	length = arch_setup_linux_cmdline(cmdline);
	if (length > 0)
		cmdline += length;

	ABOOT_VPUTS("Board-specific Linux commands...\n");
	length = board_setup_linux_cmdline(cmdline);
	if (length > 0)
		cmdline += length;

	ABOOT_VPRINT("Length of Kernel command line: %u\n", (uint32_t)(cmdline - commandline));
	if ((uint32_t)(cmdline - commandline) >= CONFIG_ABOOT_CMDLINE_SIZE) {
		ABOOT_PRINT("Kernel cmdline: overflow occurred (%u)\n", (uint32_t)(cmdline - commandline));
		vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
		hang();
	}
#endif /* CONFIG_CMDLINE_TAG */

#if defined(CONFIG_SETUP_MEMORY_TAGS) || \
	defined(CONFIG_CMDLINE_TAG) || \
	defined(CONFIG_INITRD_TAG) || \
	defined(CONFIG_SERIAL_TAG) || \
	defined(CONFIG_VIDEOLFB_TAG) || \
	defined(CONFIG_REVISION_TAG)
	debug("using: ATAGS\n");
	setup_start_tag(gd->bd);
#ifdef CONFIG_SERIAL_TAG
	setup_serial_tag(&params);
#endif
#ifdef CONFIG_CMDLINE_TAG
	setup_commandline_tag(gd->bd, commandline);
#endif
#ifdef CONFIG_REVISION_TAG
	setup_revision_tag(&params);
#endif
#ifdef CONFIG_SETUP_MEMORY_TAGS
	setup_memory_tags(gd->bd);
#endif
#ifdef CONFIG_INITRD_TAG
	if (initrd_entry && initrd_size)
		setup_initrd_tag (gd->bd, initrd_entry, initrd_size);
#endif
#ifdef CONFIG_VIDEOLFB_TAG
#ifndef CONFIG_NO_VIDEOLFB_TAG
	setup_videolfb_tag((gd_t *) gd);
#endif /* !CONFIG_NO_VIDEOLFB_TAG */
#endif
	setup_end_tag(gd->bd);
#else /* all tags */
	PUTS("ATAGS support not compiled in - hanging\n");
	hang();
#endif /* all tags */
}

/*-------------------------------------------------------------------------*/

/* Subcommand: GO */
static void boot_jump_linux(ulong kernel_entry, ulong initrd_entry)
{
	unsigned long machid = gd->bd->bi_arch_number;
	void (*theKernel)(int zero, int arch, uint params);
	unsigned long r2;

	theKernel = (void (*)(int, int, uint))kernel_entry;

	ABOOT_VPRINT("machine id: %lu\n", machid);

	/* James Wu: @TODO@ we must turn off SD card before Linux Kernel */

	bootstage_mark(BOOTSTAGE_ID_RUN_OS);
	announce_and_cleanup();

	r2 = gd->bd->bi_boot_params;

	ABOOT_DPRINT("Jumping to Linux(0,%lu,0x%08lx) at 0x%08lx ...\n", machid, r2, kernel_entry);

	theKernel(0, machid, r2);
}

void do_aboot_linux(const char* bootargs, ulong kernel_entry, ulong initrd_entry, ulong initrd_size)
{
	ulong	iflag;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	/* James Wu: @TODO@ check the power button during the power-off charger boot reason */

	/*
	 * We have reached the point of no return: we are going to
	 * overwrite all exception vector code, so we cannot easily
	 * recover from any failures any more...
	 */
	iflag = disable_interrupts();

#if defined(CONFIG_CMD_USB)
	/*
	 * turn off USB to prevent the host controller from writing to the
	 * SDRAM while Linux is booting. This could happen (at least for OHCI
	 * controller), because the HCCA (Host Controller Communication Area)
	 * lies within the SDRAM and the host controller writes continously to
	 * this area (as busmaster!). The HccaFrameNumber is for example
	 * updated every 1 ms within the HCCA structure in SDRAM! For more
	 * details see the OpenHCI specification.
	 */
	usb_stop();
#endif

	bootstage_mark(BOOTSTAGE_ID_CHECK_BOOT_OS);

	ABOOT_VPUTS("Board-specific preboot before Linux...\n");
	board_preboot_os();

	ABOOT_VPUTS("Arch-specific preboot before Linux...\n");
	arch_preboot_os();

	boot_prep_linux(bootargs, initrd_entry, initrd_size);
	boot_jump_linux(kernel_entry, initrd_entry);

	/* does not return */
	bootstage_error(BOOTSTAGE_ID_BOOT_OS_RETURNED);
	ABOOT_DPUTS("Control returned to monitor - resetting...\n");
	vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
	do_reset (NULL, 0, 0, NULL);
}

/*-------------------------------------------------------------------------*/

int aboot_sdcard_fat_boot(int dev, int part, const char* filename)
{
	boot_img_hdr hdr;
	fat_read_info fat;
	int ret = -EPERM;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	uint32_t offset = 0;
	long n, size;
	void *buffer = NULL;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	PRINT("## Loading %s from SD card...\n", filename);

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_rescan(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (!IS_SD(mmc)) {
		ABOOT_PUTS("not SD device\n");
		ret = -EINVAL;
		goto fail1;
	}
	dev_desc = &(mmc->block_dev);

	if (fat_register_device(dev_desc, part) != 0) {
		ABOOT_PUTS("invalid FAT\n");
		ret = -ENXIO;
		goto fail1;
	}

	n = file_fat_read_init(&fat, filename, 2*1024*1024/*2MB*/);
	if (n <= 0) {
		ABOOT_PRINT("%s not found (%ld)\n", filename, n);
		ret = -ENOENT;
		goto fail1;
	}

	/* Header */
	/* allocate the header buffer */
	size = ALIGN(sizeof(boot_img_hdr), dev_desc->blksz);
	buffer = (void *)memalign(ARCH_DMA_MINALIGN, size);
	if (!buffer) {
		ABOOT_PUTS("out of memory\n");
		ret = -ENOMEM;
		goto fail2;
	}
	n = file_fat_read_data(&fat, offset, buffer, (uint32_t)size);
	if (n != size) {
		ABOOT_PRINT("read boot_img_hdr err: %lu\n", n);
		ret = -EIO;
		goto fail3;
	}
	memcpy(&hdr, buffer, sizeof(hdr));
	free(buffer);
	buffer = NULL;

	/* Header Validation */
	if (memcmp(hdr.magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		PUTS("## Bad aboot image magic\n");
		ret = -ENOEXEC;
		goto fail3;
	}
	if ((hdr.page_size != 2048) && (hdr.page_size != 4096)) {
		PRINT("## Bad aboot page size %u\n", hdr.page_size);
		ret = -ENOEXEC;
		goto fail3;
	}
	if (hdr.unused[0] != 0) {
		/* Header checksum */
		unsigned checksum = hdr.unused[0];
		hdr.unused[0] = 0;
		hdr.unused[0] = (unsigned)crc32(0, (uchar*)&hdr, sizeof(hdr));
		if (checksum != hdr.unused[0]) {
			PUTS("## Bad aboot image header\n");
			ABOOT_DPRINT("header checksum 0x%08x <-> 0x%08x\n", checksum, hdr.unused[0]);
			ret = -ENOEXEC;
			goto fail3;
		}
	}

	/* Kernel */
	offset += ALIGN(sizeof(boot_img_hdr), hdr.page_size);
	size = (long)hdr.kernel_size;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading kernel @%08x(%u) (%u) ...\n", hdr.kernel_addr, hdr.kernel_size, offset);
#else
	PRINT("## Loading kernel @%08x(%u) ...\n", hdr.kernel_addr, hdr.kernel_size);
#endif /* CONFIG_ABOOT_DEBUG */
	n = file_fat_read_data(&fat, offset, (void *)hdr.kernel_addr, (uint32_t)size);
	if (n != size) {
		ABOOT_PRINT("read kernel err: %lu\n", n);
		ret = -EIO;
		goto fail3;
	}

	/* Ramdisk */
	offset += ALIGN(hdr.kernel_size, hdr.page_size);
	size = (long)hdr.ramdisk_size;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading ramdisk@%08x(%u) (%u) ...\n", hdr.ramdisk_addr, hdr.ramdisk_size, offset);
#else
	PRINT("## Loading ramdisk@%08x(%u) ...\n", hdr.ramdisk_addr, hdr.ramdisk_size);
#endif /* CONFIG_ABOOT_DEBUG */
	n = file_fat_read_data(&fat, offset, (void *)hdr.ramdisk_addr, (uint32_t)size);
	if (n != size) {
		ABOOT_PRINT("read ramdisk err: %lu\n", n);
		ret = -EIO;
		goto fail3;
	}

	file_fat_read_exit(&fat);

	do_aboot_linux((char*)hdr.cmdline, hdr.kernel_addr, hdr.ramdisk_addr, hdr.ramdisk_size);

fail3:
	if (buffer) {
		free(buffer);
		buffer = NULL;
	}
fail2:
	file_fat_read_exit(&fat);
fail1:
	/* James Wu @TODO@ suspend MMC */
	;
fail0:
	return ret;
}

/*-------------------------------------------------------------------------*/

int aboot_mmc_boot(int dev, const char *part_name)
{
	boot_img_hdr *hdr;
	int ret = -EPERM;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	unsigned long blk_start, blk_cnt, n;
	aboot_ptentry *pte;

#if defined(CONFIG_USB_OTG_TRANSCEIVER) && !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif

	PRINT("## Loading bootimg from MMC%d at '%s' ...\n", dev, part_name);

	pte = aboot_get_part_by_name(part_name);
	if (!pte) {
		ABOOT_PRINT("invalid part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	if (ABOOT_PART_GET_DEVICE(pte->flags) != ABOOT_PART_FLAGS_DEVICE_EMMC) {
		ABOOT_PRINT("not eMMC part '%s'\n", part_name);
		ret = -ENOENT;
		goto fail0;
	}

	mmc = find_mmc_device(dev);
	if (!mmc) {
		ABOOT_PRINT("invalid mmc device: %d\n", dev);
		ret = -ENODEV;
		goto fail0;
	}

	ret = mmc_init(mmc);
	if (ret) {
		ABOOT_PRINT("mmc init err %d\n", ret);
		ret = -EIO;
		goto fail0;
	}
	if (IS_SD(mmc)) {
		ABOOT_PUTS("not eMMC device\n");
		ret = -EINVAL;
		goto fail1;
	}
	dev_desc = &(mmc->block_dev);

	/* Header */
	blk_start = pte->part_blk_start;
	n = dev_desc->block_read(dev, blk_start, 1, aboot_mmc_blk);
	if (n != 1) {
		ABOOT_PRINT("read boot_img_hdr err: %lu\n", n);
		ret = -EIO;
		goto fail1;
	}
	hdr = (boot_img_hdr *)aboot_mmc_blk;

	/* Header Validation */
	if (memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		PUTS("## Bad aboot image magic\n");
		ret = -ENOEXEC;
		goto fail1;
	}
	if ((hdr->page_size != 2048) && (hdr->page_size != 4096)) {
		PRINT("## Bad aboot page size %u\n", hdr->page_size);
		ret = -ENOEXEC;
		goto fail1;
	}
#if 0
	if (hdr->unused[0] != 0) {
		/* Header checksum */
		unsigned checksum = hdr->unused[0];
		hdr->unused[0] = 0;
		hdr->unused[0] = (unsigned)crc32(0, (uchar*)hdr, sizeof(boot_img_hdr));
		if (checksum != hdr->unused[0]) {
			PUTS("## Bad aboot image header\n");
			ABOOT_DPRINT("header checksum 0x%08x <-> 0x%08x\n", checksum, hdr->unused[0]);
			ret = -ENOEXEC;
			goto fail1;
		}
	}
#endif

	/* Kernel */
	blk_start += ALIGN(ALIGN(sizeof(boot_img_hdr), hdr->page_size), dev_desc->blksz) / dev_desc->blksz;
	blk_cnt = ALIGN(hdr->kernel_size, dev_desc->blksz) / dev_desc->blksz;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading kernel @%08x(%u) (0x%lx,%lu) ...\n", hdr->kernel_addr, hdr->kernel_size, blk_start, blk_cnt);
#else
	PRINT("## Loading kernel @%08x(%u) ...\n", hdr->kernel_addr, hdr->kernel_size);
#endif /* CONFIG_ABOOT_DEBUG */
	n = dev_desc->block_read(dev, blk_start, blk_cnt, (void*)hdr->kernel_addr);
	if (n != blk_cnt) {
		ABOOT_PRINT("read kernel err: %lu\n", n);
		ret = -EIO;
		goto fail1;
	}

	/* Ramdisk */
	blk_start += ALIGN(ALIGN(hdr->kernel_size, hdr->page_size), dev_desc->blksz) / dev_desc->blksz;
	blk_cnt = ALIGN(hdr->ramdisk_size, dev_desc->blksz) / dev_desc->blksz;
#ifdef CONFIG_ABOOT_DEBUG
	PRINT("## Loading ramdisk@%08x(%u) (0x%lx,%lu) ...\n", hdr->ramdisk_addr, hdr->ramdisk_size, blk_start, blk_cnt);
#else
	PRINT("## Loading ramdisk@%08x(%u) ...\n", hdr->ramdisk_addr, hdr->ramdisk_size);
#endif /* CONFIG_ABOOT_DEBUG */
	n = dev_desc->block_read(dev, blk_start, blk_cnt, (void*)hdr->ramdisk_addr);
	if (n != blk_cnt) {
		ABOOT_PRINT("read ramdisk err: %lu\n", n);
		ret = -EIO;
		goto fail1;
	}

	do_aboot_linux((char*)hdr->cmdline, hdr->kernel_addr, hdr->ramdisk_addr, hdr->ramdisk_size);

fail1:
	;
fail0:
	return ret;
}

/*-------------------------------------------------------------------------*/

void aboot(void)
{
	int ret;

	switch (aboot_reason) {
	case ABOOT_REASON_PCBA:
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
		PUTS("\n\n===========================================\n");
		PUTS("= PCBA uses Fastboot not Android Recovery =\n");
		PUTS("===========================================\n\n");
#else
		memset(&aboot_recovery_msg, 0, sizeof(aboot_recovery_msg));
		snprintf(aboot_recovery_msg.command, sizeof(aboot_recovery_msg.command),
				"boot-recovery");
		snprintf(aboot_recovery_msg.recovery, sizeof(aboot_recovery_msg.recovery),
				"recovery\n--show_text\n--update_package=/sdcard/%s.zip\n--factory_pcba\n--wipe_cache\n",
				CONFIG_SYS_BOARD_NAME);
		if (aboot_set_bootloader_message(&aboot_recovery_msg)) {
			PUTS("\n\n=====================================\n");
			PUTS("= <Android BCB> Recovery PCBA error =\n");
			PUTS("=====================================\n\n");
		} else {
#if defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
			ret = aboot_sdcard_fat_boot(CONFIG_ABOOT_SDCARD_DEVICE_NO, 1, "recovery.img");
#else
			ret = aboot_sdcard_fat_boot(0, 1, "recovery.img");
#endif /* CONFIG_ABOOT_SDCARD_DEVICE_NO */
			PUTS("\n\n=============================\n");
			PUTS("= CAN NOT run PCBA download =\n");
			PUTS("=============================\n\n");
		}
#endif /* CONFIG_ABOOT_PCBA_USE_FASTBOOT */
		break;
	case ABOOT_REASON_RECOVERY:
#if defined(CONFIG_ABOOT_SDCARD_DEVICE_NO)
		if (aboot_cold_reset /* COLD RESET only */
			&& aboot_sdcard_has_recovery_zip(CONFIG_ABOOT_SDCARD_DEVICE_NO, 1, CONFIG_ABOOT_SDCARD_RECOVERY_FILENAME)) {
			snprintf(aboot_recovery_msg.command, sizeof(aboot_recovery_msg.command),
					"boot-recovery");
			if (!memcmp(aboot_recovery_msg.recovery, "recovery\n", 9)) {
				ABOOT_PRINT("<DROP> previous Recovery args: %s\n", aboot_recovery_msg.recovery);
			}
			snprintf(aboot_recovery_msg.recovery, sizeof(aboot_recovery_msg.recovery),
					"recovery\n--update_package=/sdcard/%s\n", CONFIG_ABOOT_SDCARD_RECOVERY_FILENAME);
			ABOOT_PRINT("Recovery args: %s\n", aboot_recovery_msg.recovery);
			if (aboot_set_bootloader_message(&aboot_recovery_msg)) {
				PUTS("==============================\n");
				PUTS("= Android Recovery BCB error =\n");
				PUTS("==============================\n");
			} else {
				PRINT("## Recovery update package is '%s'\n", CONFIG_ABOOT_SDCARD_RECOVERY_FILENAME);
			}
		}
#endif /* CONFIG_ABOOT_SDCARD_DEVICE_NO */
		ret = aboot_mmc_boot(CONFIG_ABOOT_EMMC_DEVICE_NO, "recovery");
		PUTS("\n\n================================\n");
		PUTS("= CAN NOT run Android RECOVERY =\n");
		PUTS("================================\n\n");
		break;
	default:
		ret = aboot_mmc_boot(CONFIG_ABOOT_EMMC_DEVICE_NO, "boot");
		PUTS("\n\n============================\n");
		PUTS("= CAN NOT run Android BOOT =\n");
		PUTS("============================\n\n");
		break;
	}
	vibrate(CONFIG_ABOOT_POWERON_VIBRATION_TIME<<1);
	board_poweroff();
}

/*-------------------------------------------------------------------------*/

int do_aboot_reason(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 2) {
		if (!strcmp(argv[1], ABOOT_REASON_RECOVERY_STR)) {
			PUTS("RECOVERY\n");
			aboot_reason = ABOOT_REASON_RECOVERY;
			aboot_set_state(ABOOT_STATE_BOOT_RECOVERY);
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
#else
		} else if (!strcmp(argv[1], ABOOT_REASON_PCBA_STR) && board_is_factory_mode()) {
			PUTS("PCBA\n");
			aboot_reason = ABOOT_REASON_PCBA;
			aboot_set_state(ABOOT_STATE_BOOT_RECOVERY);
#endif /* CONFIG_ABOOT_PCBA_USE_FASTBOOT */
		} else if (!strcmp(argv[1], ABOOT_REASON_CHARGER_STR)) {
			PUTS("CHARGER\n");
			aboot_reason = ABOOT_REASON_CHARGER;
			aboot_set_state(ABOOT_STATE_BOOT_CHARGER);
		} else {
			PUTS("NORMAL\n");
			aboot_reason = ABOOT_REASON_NORMAL;
			aboot_set_state(ABOOT_STATE_BOOT_ANDROID);
		}
	} else {
		switch (aboot_reason) {
		case ABOOT_REASON_NORMAL:
			PUTS("NORMAL\n");
			break;
		case ABOOT_REASON_CHARGER:
			PUTS("CHARGER\n");
			break;
		case ABOOT_REASON_PC_CHARGER:
			PUTS("PC CHARGER\n");
			break;
		case ABOOT_REASON_RECOVERY:
			PUTS("RECOVERY\n");
			break;
		case ABOOT_REASON_FASTBOOT:
			PUTS("FASTBOOT\n");
			break;
		case ABOOT_REASON_ALARM:
			PUTS("ALARM\n");
			break;
		case ABOOT_REASON_PCBA:
			PUTS("PCBA\n");
			break;
		default:
			PRINT("0x%08x\n", aboot_reason);
			break;
		}
	}

	return 0;
}

U_BOOT_CMD(
	bootreason, 2, 1, do_aboot_reason,
	"Boot reason",
	""
);

/*-------------------------------------------------------------------------*/

int do_aboot_mode(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 2) {
		if (!strcmp(argv[1], ABOOT_MODE_FACTORY_STR)) {
			aboot_mode = ABOOT_MODE_FACTORY;
			PUTS("FACTORY\n");
		} else {
			aboot_mode = ABOOT_MODE_NORMAL;
			PUTS("NORMAL\n");
		}
	} else {
		switch (aboot_mode) {
		case ABOOT_MODE_NORMAL:
			PUTS("NORMAL\n");
			break;
		case ABOOT_MODE_CHARGER:
			PUTS("CHARGER\n");
			break;
		case ABOOT_MODE_FACTORY:
			PUTS("FACTORY\n");
			break;
		default:
			PRINT("0x%08x\n", aboot_mode);
			break;
		}
	}

	return 0;
}

U_BOOT_CMD(
	bootmode, 2, 1, do_aboot_mode,
	"Boot mode",
	""
);

