/*
 * InnoComm DPI to LVDS panel support
 *
 * (C) Copyright 2013-2014
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

#include <common.h>
#include <exports.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/gpio.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/compiler.h>

#include <icom/omapdss.h>
#include <icom/omap-panel-icom-dpi2lvds.h>

#define	PANEL_NAME	"dpi2lvds"

/**************************************************************************************************/

/*#define PANEL_DEBUG*/
/*#define PANEL_VERBOSE_DEBUG*/

/*----------------------------------------------------------------------*/

#ifdef PANEL_DEBUG
#define PANEL_DBG(fmt, args...) \
	do {printf("[panel] " PANEL_NAME ": " fmt, ##args);} while (0)
#define PANEL_DBGF(fmt, args...) \
	do {printf("[panel] " PANEL_NAME ": " fmt, ##args);} while (0)
#else /* PANEL_DEBUG */
#define PANEL_DBG(fmt, args...) \
	do {} while (0)
#define PANEL_DBGF(fmt, args...) \
	do {} while (0)
#endif /* PANEL_DEBUG */

#ifdef PANEL_VERBOSE_DEBUG
#define PANEL_VDBG(fmt, args...) \
	do {printf("[panel] " PANEL_NAME ": " fmt, ##args);} while (0)
#else /* PANEL_VERBOSE_DEBUG */
#define PANEL_VDBG(fmt, args...) \
	do {} while (0)
#endif /* PANEL_VERBOSE_DEBUG */

#define PANEL_ERR(fmt, args...) \
	do {printf("panel " PANEL_NAME " err: " fmt, ##args);} while (0)
#define PANEL_INFO(fmt, args...) \
	do {printf("panel " PANEL_NAME ": " fmt, ##args);} while (0)
#define PANEL_WARN(fmt, args...) \
	do {printf("panel " PANEL_NAME ": " fmt, ##args);} while (0)

/**************************************************************************************************/

static inline struct panel_icom_dpi2lvds_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct panel_icom_dpi2lvds_data*) dssdev->data;
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

static void panel_poweron(struct omap_dss_device *dssdev)
{
	struct panel_icom_dpi2lvds_data *panel_data = get_panel_data(dssdev);
	int r;

	PANEL_VDBG("%s\n", __func__);

	if (panel_data && panel_data->dpi_mux_pads)
		panel_data->dpi_mux_pads(true);

	if (dssdev->platform_enable) {
		/* Power on, Reset LCD */
		r = dssdev->platform_enable(dssdev);
		if (r)
			PANEL_ERR("%s: platform_enable err %d\n", __func__, r);
	}

	if (panel_data && panel_data->power_on_dpi_delay)
		mdelay(panel_data->power_on_dpi_delay);

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		PANEL_ERR("%s: dpi enable err %d\n", __func__, r);

	/* wait couple of vsyncs after enabling the LCD */
	if (panel_data && panel_data->power_on_delay)
		mdelay(panel_data->power_on_delay);

	PANEL_VDBG("%s exit\n", __func__);
}

static void panel_poweroff(struct omap_dss_device *dssdev)
{
	struct panel_icom_dpi2lvds_data *panel_data = get_panel_data(dssdev);

	PANEL_VDBG("%s\n", __func__);

	omapdss_dpi_display_disable(dssdev);

	if (panel_data) {
		if (panel_data->dpi_mux_pads)
			panel_data->dpi_mux_pads(false);

		if (panel_data->power_off_dpi_delay)
			mdelay(panel_data->power_off_dpi_delay);
	}

	/* Control RESET pin, Power off LCD */
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	if (panel_data) {
		/* wait couple of vsyncs after disabling the LCD */
		if (panel_data->power_off_delay)
			mdelay(panel_data->power_off_delay);

#if 0
		/* RESET pin is LOW to avoid the current leakage */
		if (gpio_is_valid(dssdev->reset_gpio) && panel_data->reset_high)
			gpio_set_value(dssdev->reset_gpio, 0);
#endif
	}

	PANEL_VDBG("%s exit\n", __func__);
}

/**************************************************************************************************/

static int panel_probe(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	PANEL_VDBG("%s exit\n", __func__);

	return 0;
}

static int panel_enable(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("%s: was enabled\n", __func__);
	} else {
		panel_poweron(dssdev);
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		PANEL_DBG("%s: enabled\n", __func__);
	}

	PANEL_VDBG("%s exit\n", __func__);

	return 0;
}

static void panel_disable(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		panel_poweroff(dssdev);
		PANEL_DBG("%s: disabled\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		PANEL_DBG("%s: suspended\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		PANEL_DBG("%s: was disabled\n", __func__);
	} else {
		PANEL_DBG(dssdev, "%s: unknown state %d\n", __func__, dssdev->state);
	}

	PANEL_VDBG("%s exit\n", __func__);
}

/**************************************************************************************************/

static void panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_VDBG("%s\n", __func__);
	dpi_set_timings(dssdev, timings);
}

static void panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_VDBG("%s\n", __func__);
	*timings = dssdev->panel.timings;
}

static int panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_VDBG("%s\n", __func__);
	return dpi_check_timings(dssdev, timings);
}

/**************************************************************************************************/

static struct omap_dss_driver panel_driver = {
	.driver_name			= PANEL_NAME,

	.probe					= panel_probe,

	.enable					= panel_enable,
	.disable				= panel_disable,

	.set_timings			= panel_set_timings,
	.get_timings			= panel_get_timings,
	.check_timings			= panel_check_timings,
};

/**************************************************************************************************/

int panel_dpi2lvds_init(void)
{
	PANEL_VDBG("%s\n", __func__);
	return omap_dss_register_driver(&panel_driver);
}
