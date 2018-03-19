/*
 * (C) Copyright 2014
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 * Sinjhe Yu <sinjhe.yu@innocomm.com>
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

#if !defined(CONFIG_SPL_BUILD)

#include <common.h>
#include <exports.h>
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux_omap4.h>

#include <icom/aboot.h>
#include <icom/pwm.h>
#include <icom/omapdss.h>
#include <icom/omap-panel-icom-dpi2lvds.h>

#include "../common/board_omap4_common.h"

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_BOARD_DEBUG
#define CONFIG_BOARD_DEBUG
#endif /* CONFIG_BOARD_DEBUG */
#ifndef CONFIG_BOARD_VERBOSE_DEBUG
#define CONFIG_BOARD_VERBOSE_DEBUG
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_BOARD_DEBUG
#define BOARD_DPRINT(fmt, args...) \
	do {printf("[dpi2lvds] " fmt, ##args);} while (0)
#define BOARD_DPUTS(fmt) \
	do {puts("[dpi2lvds] " fmt);} while (0)
#else /* !CONFIG_BOARD_DEBUG */
#define BOARD_DPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_DEBUG */

#ifdef CONFIG_BOARD_VERBOSE_DEBUG
#define BOARD_VPRINT(fmt, args...) \
	do {printf("[dpi2lvds] " fmt, ##args);} while (0)
#define BOARD_VPUTS(fmt) \
	do {puts("[dpi2lvds] " fmt);} while (0)
#else /* !CONFIG_BOARD_VERBOSE_DEBUG */
#define BOARD_VPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */

#define BOARD_PRINT(fmt, args...) \
	do {printf("dpi2lvds: " fmt, ##args);} while (0)
#define BOARD_PUTS(fmt) \
	do {puts("dpi2lvds: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)

/**************************************************************************************************/

static const char *panel_driver_name = NULL;
static struct omap_dss_device *omap4_lcd_device = NULL;
static struct panel_icom_dpi2lvds_data icom_dpi2lvds_data;

static const struct pad_conf_entry *dispc2_dynamic_mux = NULL;
static int dispc2_dynamic_mux_size;

/**************************************************************************************************/

static const struct pad_conf_entry dispc2_safemode_pads[] = {
	{DPM_EMU8, (PTD | IEN | M7)},	/* SafeMode: dispc2_pclk */
	{DPM_EMU10, (PTD | IEN | M7)},	/* SafeMode: dispc2_de */
	{DPM_EMU7, (PTD | IEN | M7)},	/* SafeMode: dispc2_hsync */
	{DPM_EMU9, (PTD | IEN | M7)},	/* SafeMode: dispc2_vsync */
};

static const struct pad_conf_entry dispc2_dynamic_pads[] = {
	{DPM_EMU8, (IDIS | M5)},		/* dispc2_pclk */
	{DPM_EMU10, (IDIS | M5)},		/* dispc2_de */
	{DPM_EMU7, (IDIS | M5)},		/* dispc2_hsync */
	{DPM_EMU9, (IDIS | M5)},		/* dispc2_vsync */
};

static const struct pad_conf_entry dispc2_no_sync_dynamic_pads[] = {
	{DPM_EMU8, (IDIS | M5)},		/* dispc2_pclk */
	{DPM_EMU10, (IDIS | M5)},		/* dispc2_de */
	{DPM_EMU7, (PTD | IEN | M7)},	/* SafeMode: dispc2_hsync */
	{DPM_EMU9, (PTD | IEN | M7)},	/* SafeMode: dispc2_vsync */
};

/**************************************************************************************************/

static void __omap4_dpi2lvds_board_set_brightness(struct omap_dss_device *dssdev, int level)
{
	BOARD_PRINT("no Brightness support\n");
}

static void __omap4_dpi2lvds_board_enable(struct omap_dss_device *dssdev) {}
static void __omap4_dpi2lvds_board_disable(struct omap_dss_device *dssdev) {}

void omap4_dpi2lvds_board_set_brightness(struct omap_dss_device *dssdev, int level)
		__attribute__((weak, alias("__omap4_dpi2lvds_board_set_brightness")));
void omap4_dpi2lvds_board_enable(struct omap_dss_device *dssdev)
		__attribute__((weak, alias("__omap4_dpi2lvds_board_enable")));
void omap4_dpi2lvds_board_disable(struct omap_dss_device *dssdev)
		__attribute__((weak, alias("__omap4_dpi2lvds_board_disable")));

/**************************************************************************************************/

static int omap4_panel_set_brightness(struct omap_dss_device *dssdev, int brightness)
{
	omap4_dpi2lvds_board_set_brightness(dssdev, brightness);
	return 0;
}

/**************************************************************************************************/

static inline struct panel_icom_dpi2lvds_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct panel_icom_dpi2lvds_data*) dssdev->data;
}

static void omap4_dpi_mux_pads(bool enable)
{
	if (dispc2_dynamic_mux) {
		if (enable)
			do_set_mux(CONTROL_PADCONF_CORE, dispc2_dynamic_mux, dispc2_dynamic_mux_size);
		else
			do_set_mux(CONTROL_PADCONF_CORE, dispc2_safemode_pads,
				sizeof(dispc2_safemode_pads) / sizeof(struct pad_conf_entry));
	}
}

static int omap4_panel_enable(struct omap_dss_device *dssdev)
{
	/* Power on LCM voltages */
	omap4_dpi2lvds_board_enable(dssdev);

	if (dssdev->reset_gpio >= 0) {
		struct panel_icom_dpi2lvds_data *panel_data = get_panel_data(dssdev);
		if (panel_data) {
			/* RESET pin is inactive */
			gpio_set_value(dssdev->reset_gpio, !panel_data->reset_high);

			/* Wait for Voltage ready */
			if (panel_data->power_on_reset_delay)
				mdelay(panel_data->power_on_reset_delay);

			/* Reset LCM: toggle RESET pin */
			/* RESET pin is active */
			gpio_set_value(dssdev->reset_gpio, panel_data->reset_high);
			udelay(1500);
			/* RESET pin is inactive */
			gpio_set_value(dssdev->reset_gpio, !panel_data->reset_high);
		} else {
			BOARD_PRINT("%s: no panel data\n", __func__);
		}
	}

	return 0;
}

static void omap4_panel_disable(struct omap_dss_device *dssdev)
{
	/* RESET pin is LOW to avoid the current leakage */
	if (dssdev->reset_gpio >= 0) {
		struct panel_icom_dpi2lvds_data *panel_data = get_panel_data(dssdev);
#if 0
		if (panel_data->reset_high)
			gpio_set_value(dssdev->reset_gpio, 1);
		else
			gpio_set_value(dssdev->reset_gpio, 0);
#else
		gpio_set_value(dssdev->reset_gpio, panel_data->reset_high);
#endif
	}
	/* Power off LCM voltages */
	omap4_dpi2lvds_board_disable(dssdev);
}

/**************************************************************************************************/

/**
 * Power-on Sequence:
 *   1. Enable DPI mux configurations
 *   2. Power on LCM voltages
 *   3. RESET (if reset_gpio >= 0)
 *      3.a. RESET pin is INACTIVE
 *      3.b. Wait <power_on_reset_delay> ms
 *      3.c. RESET pin is ACTIVE
 *      3.d. Wait ~1500 us
 *      3.e. RESET pin is INACTIVE
 *   4. (OPTIONAL) Wait <power_on_dpi_delay> ms
 *   5. Enable DPI signals
 *   6. Wait <power_on_delay> ms
 *
 * Power-off Sequence:
 *   1. Disable DPI signals
 *   2. Disable DPI mux configurations
 *   3. (OPTIONAL) Wait <power_off_dpi_delay> ms
 *   4. RESET pin is low (if reset_gpio >= 0)
 *   5. Power off LCM voltages
 *   6. Wait <power_off_delay> ms
 */
static struct icom_dpi2lvds_device {
	const char *panel_name;		/* used to match panel device */

	u16 power_on_reset_delay;	/* See above Power-on Sequence */
	u16 power_on_dpi_delay;		/* (OPTIONAL) See above Power-on Sequence */
	u16 power_on_delay;			/* See above Power-on Sequence */

	u16 power_off_dpi_delay;	/* (OPTIONAL) See above Power-off Sequence */
	u16 power_off_delay;		/* See above Power-off Sequence */

	struct omap_dss_device dssdev;
} icom_dpi2lvds_devices[] /*__initdata*/ = {
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_EJ070NA)
{
	/** 7"
	 * CHIMEI EJ070NA panel on Talos7/PBx/PWx/...
	 */
	.panel_name = "ej070na_chimei",
	.power_on_reset_delay = 50,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = 46,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_GATE_HVSYNC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 600,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 120 + 100 + 100 = 1344
				 * ytot = y_res + vsw + vfp + vbp = 600 + 15 + 10 + 10 = 635
				 * HSYNC = pixel_clock / xtot = 51200000 / 1344 = 38095 Hz
				 * VSYNC = HSYNC / ytot = 38095 / 635 = 59.99 Hz
				 */
				.pixel_clock	= 51200, /* KHz */
				.hsw			= 120,
				.hfp			= 100,
				.hbp			= 100,
				.vsw			= 15,
				.vfp			= 10,
				.vbp			= 10,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (153600 + 500) / 1000 = 154
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (90000 + 500) / 1000 = 90
			 * xdpi = (1024 * 25.4f / width_in_mm) = 168
			 * ydpi = (600 * 25.4f / height_in_mm) = 169
			 */
			.width_in_um = 153600,
			.height_in_um = 90000,
		},
		.min_fck_per_pck = 3,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_HJ101NA)
{
	/** 10.1"
	 * CHIMEI HJ101NA panel on Talos10
	 */
	.panel_name = "hj101na_chimei",
	.power_on_reset_delay = 50,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = 46,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_GATE_HVSYNC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1280,
				.y_res			= 800,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1280 + 100 + 50 + 50 = 1480
				 * ytot = y_res + vsw + vfp + vbp = 800 + 25 + 20 + 20 = 865
				 * HSYNC = pixel_clock / xtot = 76800000 / 1480 = 51891 Hz
				 * VSYNC = HSYNC / ytot = 51891 / 865 = 59.98 Hz
				 */
				.pixel_clock	= 76800, /* KHz */
				.hsw			= 100,
				.hfp			= 50,
				.hbp			= 50,
				.vsw			= 25,
				.vfp			= 20,
				.vbp			= 20,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (216960 + 500) / 1000 = 217
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (135600 + 500) / 1000 = 136
			 * xdpi = (1280 * 25.4f / width_in_mm) = 149
			 * ydpi = (800 * 25.4f / height_in_mm) = 149
			 */
			.width_in_um = 216960,
			.height_in_um = 135600,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_HJ070IA)
{
	/** 7"
	 * CHIMEI HJ070IA panel on Uriel
	 */
	.panel_name = "hj070ia_chimei",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1280,
				.y_res			= 800,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1280 + 146 = 1426
				 * ytot = y_res + vsw + vfp + vbp = 800 + 16 = 816
				 * HSYNC = pixel_clock / xtot = 69818000 / 1426 = 48960 Hz
				 * VSYNC = HSYNC / ytot = 48960 / 816 = 60 Hz
				 */
				.pixel_clock	= 69818, /* KHz */
				.hsw			= 46,
				.hfp			= 50,
				.hbp			= 50,
				.vsw			= 6,
				.vfp			= 5,
				.vbp			= 5,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (149760 + 500) / 1000 = 150
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (93600 + 500) / 1000 = 94
			 * xdpi = (1280 * 25.4f / width_in_mm) = 216
			 * ydpi = (800 * 25.4f / height_in_mm) = 216
			 */
			.width_in_um = 149760,
			.height_in_um = 93600,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_GREENTHAN_GTN070WX1)
{
	/** 7"
	 * Greenthan GTN070WX1 panel on Uriel
	 */
	.panel_name = "gtn070wx1_greenthan",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 18,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1280,
				.y_res			= 800,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1280 + 146 = 1426
				 * ytot = y_res + vsw + vfp + vbp = 800 + 16 = 816
				 * HSYNC = pixel_clock / xtot = 69818000 / 1426 = 48960 Hz
				 * VSYNC = HSYNC / ytot = 48960 / 816 = 60 Hz
				 */
				.pixel_clock	= 69818, /* KHz */
				.hsw			= 46,
				.hfp			= 50,
				.hbp			= 50,
				.vsw			= 6,
				.vfp			= 5,
				.vbp			= 5,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (150720 + 500) / 1000 = 151
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (94200 + 500) / 1000 = 94
			 * xdpi = (1280 * 25.4f / width_in_mm) = 215
			 * ydpi = (800 * 25.4f / height_in_mm) = 216
			 */
			.width_in_um = 150720,
			.height_in_um = 94200,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_ONATION_OT101ZBWDLV)
{
	/** 10.1"
	 * ONation OT101ZBWDLV panel on Uriel
	 */
	.panel_name = "ot101zbwdlv_onation",
	.power_on_reset_delay = 0,
	.power_on_delay = 100,
	.power_off_delay = 100,
	.dssdev = {
#ifdef CONFIG_PANEL_DPI2LVDS_ENFORCE_18_DATALINES
		.phy.dpi.data_lines = 18,
#else
		.phy.dpi.data_lines = 24,
#endif
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC /*| OMAP_DSS_LCD_GATE_HVSYNC*/,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1280,
				.y_res			= 800,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1280 + 146 = 1426
				 * ytot = y_res + vsw + vfp + vbp = 800 + 16 = 816
				 * HSYNC = pixel_clock / xtot = 69818000 / 1426 = 48960 Hz
				 * VSYNC = HSYNC / ytot = 48960 / 816 = 60 Hz
				 */
				.pixel_clock	= 69818, /* KHz */
				.hsw			= 46,
				.hfp			= 50,
				.hbp			= 50,
				.vsw			= 6,
				.vfp			= 5,
				.vbp			= 5,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (216960 + 500) / 1000 = 217
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (135600 + 500) / 1000 = 136
			 * xdpi = (1280 * 25.4f / width_in_mm) = 149
			 * ydpi = (800 * 25.4f / height_in_mm) = 149
			 */
			.width_in_um = 216960,
			.height_in_um = 135600,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_HJ080IA)
{
	/** 8"
	 * CHIMEI HJ080IA panel on Logan
	 */
	.panel_name = "hj080ia_chimei",
	.power_on_reset_delay = 50,
	.power_on_delay = 50,
	.power_off_delay = 100,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = 46,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_GATE_HVSYNC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 768,
			    /**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 176 = 1200
				 * ytot = y_res + vsw + vfp + vbp = 768 + 22 = 790
				 * HSYNC = pixel_clock / xtot = 56888000 / 1200 = 47406 Hz
				 * VSYNC = HSYNC / ytot = 47406 / 790 = 60 Hz
				 */
				.pixel_clock	= 56888, /* KHz */
				.hsw			= 56,
				.hfp			= 60,
				.hbp			= 60,
				.vsw			= 6,
				.vfp			= 8,
				.vbp			= 8,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (162050 + 500) / 1000 = 162
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (121540 + 500) / 1000 = 122
			 * xdpi = (1024 * 25.4f / width_in_mm) = 160
			 * ydpi = (768 * 25.4f / height_in_mm) = 159
			 */
			.width_in_um = 162050,
			.height_in_um = 121540,
		},
		.min_fck_per_pck = 3,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_G121X1)
{
	/** 12.1"
	 * CHIMEI G121X1 panel on iSwing
	 */
	.panel_name = "g121x1_chimei",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 200,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_GATE_HVSYNC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 768,
			    /**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 140 + 120 + 120 = 1404
				 * ytot = y_res + vsw + vfp + vbp = 768 + 20 + 15 + 15 = 818
				 * HSYNC = pixel_clock / xtot = 69818000 / 1404 = 49727 Hz
				 * VSYNC = HSYNC / ytot = 49727 / 818 = 60.79 Hz
				 */
				.pixel_clock	= 69818, /* KHz */
				.hsw			= 140,
				.hfp			= 120,
				.hbp			= 120,
				.vsw			= 20,
				.vfp			= 15,
				.vbp			= 15,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (245760 + 500) / 1000 = 246
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (184320 + 500) / 1000 = 184
			 * xdpi = (1024 * 25.4f / width_in_mm) = 105
			 * ydpi = (768 * 25.4f / height_in_mm) = 106
			 */
			.width_in_um = 245760,
			.height_in_um = 184320,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_LG_LP097X02)
{
	/** 9.7"
	 * LG P097X02 panel on Oracle/Delphi/DelphiV/...
	 */
	.panel_name = "p097x02_lg",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 18,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 768,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 240 + 260 + 436 = 1960
				 * ytot = y_res + vsw + vfp + vbp = 768 + 3 + 1 + 4 = 776
				 * HSYNC = pixel_clock / xtot = 85333000 / 1960 = 43537 Hz
				 * VSYNC = HSYNC / ytot = 43537 / 776 = 56.10 Hz
				 */
				.pixel_clock	= 85333, /* KHz */
				.hsw			= 240,
				.hfp			= 260,
				.hbp			= 436,
				.vsw            = 3,
				.vfp			= 1,
				.vbp			= 4,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (196610 + 500) / 1000 = 197
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (147460 + 500) / 1000 = 147
			 * xdpi = (1024 * 25.4f / width_in_mm) = 132
			 * ydpi = (768 * 25.4f / height_in_mm) = 132
			 */
			.width_in_um = 196610,
			.height_in_um = 147460,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_CHIMEI_BF097XN)
{
	/** 9.7"
	 * CHIMEI BF097XN panel
	 */
	.panel_name = "bf097xn_chimei",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 18,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 768,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 240 + 260 + 436 = 1960
				 * ytot = y_res + vsw + vfp + vbp = 768 + 3 + 1 + 4 = 776
				 * HSYNC = pixel_clock / xtot = 85333000 / 1960 = 43537 Hz
				 * VSYNC = HSYNC / ytot = 43537 / 776 = 56.10 Hz
				 */
				.pixel_clock	= 85333, /* KHz */
				.hsw			= 240,
				.hfp			= 260,
				.hbp			= 436,
				.vsw            = 3,
				.vfp			= 1,
				.vbp			= 4,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (196608 + 500) / 1000 = 197
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (147456 + 500) / 1000 = 147
			 * xdpi = (1024 * 25.4f / width_in_mm) = 132
			 * ydpi = (768 * 25.4f / height_in_mm) = 132
			 */
			.width_in_um = 196608,
			.height_in_um = 147456,
		},
		.min_fck_per_pck = 2,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_TM_097TDH02)
{
	/** 9.7"
	 * TM 097TDH02 panel on TC978C
	 */
	.panel_name = "097tdh02_tm",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 18,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 1024,
				.y_res			= 768,
			    /**
				 * xtot = x_res + hsw + hfp + hbp = 1024 + 3 + 90 + 90 = 1207
				 * ytot = y_res + vsw + vfp + vbp = 768 + 3 + 10 + 10 = 791
				 * HSYNC = pixel_clock / xtot = 56888000 / 1207 = 47131 Hz
				 * VSYNC = HSYNC / ytot = 47131 / 791 = 59.58 Hz
				 */
				.pixel_clock	= 56888, /* KHz */
				.hsw			= 3,
				.hfp			= 90,
				.hbp			= 90,
				.vsw            = 3,
				.vfp			= 10,
				.vbp			= 10,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (196610 + 500) / 1000 = 197
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (147460 + 500) / 1000 = 147
			 * xdpi = (1024 * 25.4f / width_in_mm) = 132
			 * ydpi = (768 * 25.4f / height_in_mm) = 132
			 */
			.width_in_um = 196610,
			.height_in_um = 147460,
		},
		.min_fck_per_pck = 3,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
#if defined(CONFIG_PANEL_INNOLUX_A07083TT)
{
	/** 7"
	 * InnoLux A070-83-TT
	 */
	.panel_name = "a07083tt_innolux",
	.power_on_reset_delay = 0,
	.power_on_delay = 50,
	.power_off_delay = 50,
	.dssdev = {
		.phy.dpi.data_lines = 24,
		.reset_gpio = -1,
		.panel = {
			.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS
					| OMAP_DSS_LCD_IPC,
			.acb = 0,
			.acbi = 0,
			.timings = {
				.x_res			= 800,
				.y_res			= 480,
				/**
				 * xtot = x_res + hsw + hfp + hbp = 800 + 18 + 40 + 30 = 888
				 * ytot = y_res + vsw + vfp + vbp = 480 + 3 + 13 + 29 = 525
				 * HSYNC = pixel_clock / xtot = 30720000 / 888 = 34594 Hz
				 * VSYNC = HSYNC / ytot = 34594 / 525 = 65.89 Hz
				 */
				.pixel_clock	= 30720, /* KHz */
				.hsw			= 18,
				.hfp			= 40,
				.hbp			= 30,
				.vsw            = 3,
				.vfp			= 13,
				.vbp			= 29,
			},
			/**
			 * width_in_mm = DIV_ROUND_CLOSEST(width_in_um, 1000) = (152400 + 500) / 1000 = 152
			 * height_in_mm = DIV_ROUND_CLOSEST(height_in_um, 1000) = (91440 + 500) / 1000 = 91
			 * xdpi = (800 * 25.4f / width_in_mm) = 133
			 * ydpi = (480 * 25.4f / height_in_mm) = 133
			 */
			.width_in_um = 152400,
			.height_in_um = 91440,
		},
		.min_fck_per_pck = 5,	/* See OMAP2_DSS_MIN_FCK_PER_PCK definition */

		.name = "lcd2",
		.type = OMAP_DISPLAY_TYPE_DPI,
		.channel = OMAP_DSS_CHANNEL_LCD2,
		.clocks = {
			.dispc = {
				.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
			},
		},
		.platform_enable = omap4_panel_enable,
		.platform_disable = omap4_panel_disable,
		.set_backlight = omap4_panel_set_brightness,
		.get_backlight = NULL,
	},
},
#endif
/**************************************************************************************************/
};

/**************************************************************************************************/

#define OMAP4_DPI2LVDS_CMDLINE_PANEL			" panel="
#define OMAP4_DPI2LVDS_CMDLINE_PANEL_RESET_GPIO	" panel_rst="

#ifdef CONFIG_PANEL_DPI2LVDS_SETUP_CMDLINE
size_t omap4_dpi2lvds_setup_linux_cmdline(char *cmdline)
{
	char* cmdline_start = cmdline;

	if (panel_driver_name) {
#ifdef CONFIG_PANEL_DPI2LVDS_RESET_GPIO
		char buff[32];
#endif
		size_t length;

		/* panel= */
		strcpy(cmdline, OMAP4_DPI2LVDS_CMDLINE_PANEL);
		cmdline += sizeof(OMAP4_DPI2LVDS_CMDLINE_PANEL) - 1;
		length = strlen(panel_driver_name);
		strncpy(cmdline, panel_driver_name, length);
		cmdline += length;

#ifdef CONFIG_PANEL_DPI2LVDS_RESET_GPIO
		/* panel_rst= */
		strcpy(cmdline, OMAP4_DPI2LVDS_CMDLINE_PANEL_RESET_GPIO);
		cmdline += sizeof(OMAP4_DPI2LVDS_CMDLINE_PANEL_RESET_GPIO) - 1;
		length = sprintf(buff, "%d,%d", omap4_lcd_device->reset_gpio,
					icom_dpi2lvds_data.reset_high);
		strncpy(cmdline, buff, length);
		cmdline += length;
#endif
	}

	return (size_t)(cmdline - cmdline_start);
}
#endif /* CONFIG_PANEL_DPI2LVDS_SETUP_CMDLINE */

/**************************************************************************************************/

struct omap_dss_device *omap4_dpi2lvds_display_setup(const char *panel_name, int reset_gpio, int reset_high)
{
	int i;

	omap4_lcd_device = NULL;
	dispc2_dynamic_mux = NULL;

	for (i = 0 ; i < ARRAY_SIZE(icom_dpi2lvds_devices) ; i++) {
		if (strncmp(panel_name, icom_dpi2lvds_devices[i].panel_name, 32))
			continue;

		panel_driver_name = icom_dpi2lvds_devices[i].panel_name;

		icom_dpi2lvds_data.power_on_reset_delay = icom_dpi2lvds_devices[i].power_on_reset_delay;
		icom_dpi2lvds_data.power_on_dpi_delay = icom_dpi2lvds_devices[i].power_on_dpi_delay;
		icom_dpi2lvds_data.power_on_delay = icom_dpi2lvds_devices[i].power_on_delay;
		icom_dpi2lvds_data.power_off_dpi_delay = icom_dpi2lvds_devices[i].power_off_dpi_delay;
		icom_dpi2lvds_data.power_off_delay = icom_dpi2lvds_devices[i].power_off_delay;
		icom_dpi2lvds_data.reset_high = reset_high;
		icom_dpi2lvds_data.is_regulator_enabled = NULL;
		icom_dpi2lvds_data.dpi_mux_pads = omap4_dpi_mux_pads;

		omap4_lcd_device = &(icom_dpi2lvds_devices[i].dssdev);
		omap4_lcd_device->driver_name = "dpi2lvds";
		omap4_lcd_device->data = &icom_dpi2lvds_data;
		if (omap4_lcd_device->reset_gpio >= 0 || icom_dpi2lvds_data.power_on_reset_delay)
			omap4_lcd_device->reset_gpio = reset_gpio;

		BOARD_PRINT("found '%s' device (rst %d%s)\n",
				panel_driver_name, omap4_lcd_device->reset_gpio,
				(omap4_lcd_device->reset_gpio < 0) ? "" :
					(icom_dpi2lvds_data.reset_high ? " HIGH" : " LOW"));

		break;
	}

	if (omap4_lcd_device) {
		if (omap4_lcd_device->channel == OMAP_DSS_CHANNEL_LCD2 && omap4_lcd_device->type == OMAP_DISPLAY_TYPE_DPI) {
			if (omap4_lcd_device->panel.config & OMAP_DSS_LCD_GATE_HVSYNC) {
				dispc2_dynamic_mux = dispc2_no_sync_dynamic_pads;
				dispc2_dynamic_mux_size = sizeof(dispc2_no_sync_dynamic_pads) / sizeof(struct pad_conf_entry);
			} else {
				dispc2_dynamic_mux = dispc2_dynamic_pads;
				dispc2_dynamic_mux_size = sizeof(dispc2_dynamic_pads) / sizeof(struct pad_conf_entry);
			}

			do_set_mux(CONTROL_PADCONF_CORE, dispc2_dynamic_mux, dispc2_dynamic_mux_size);
		}

		if (omap4_lcd_device->reset_gpio >= 0) {
			/* LCD RESET pin mux is configured in board_mux_data.h */
			gpio_request(omap4_lcd_device->reset_gpio, NULL);
			gpio_direction_output(omap4_lcd_device->reset_gpio, icom_dpi2lvds_data.reset_high);
		}
	} else {
		BOARD_PRINT("no '%s' LCD device\n", panel_name);
	}

	return omap4_lcd_device;
}

#endif /* !CONFIG_SPL_BUILD */
