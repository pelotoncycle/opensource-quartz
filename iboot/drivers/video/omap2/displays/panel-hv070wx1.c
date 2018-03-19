/*
 * Hydis HV070WX1 panel support on OMAP DSS platform
 *
 * (C) Copyright 2013
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 * SinJhe Yu <sinjhe.yu@innocomm.com>
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
#include <linux/porting-compat.h>

#include <icom/omapdss.h>

#define	PANEL_NAME	"hv070wx1"

/*----------------------------------------------------------------------*/

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

/*----------------------------------------------------------------------*/

/* device private data structure */
static struct panel_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
} paneldata;

static struct panel_data *d2d = NULL;

/*----------------------------------------------------------------------*/

static int hv070wx1_write_init_config(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[2];

	buf[0] = 0xAE;
	buf[1] = 0x0D;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to initialize LCM %d\n", r);
		return r;
	}

/*
	buf[0] = 0xB1;
	buf[1] = 0xEF;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2); // BIST mode
	if (r) {
		PANEL_ERR("failed to go to BIST mode %d\n", r);
		return r;
	}
*/

	return r;
}

/*----------------------------------------------------------------------*/

static int panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	PANEL_VDBG("%s\n", __func__);

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err0;
	}

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		PANEL_ERR("failed to enable DSI %d\n", r);
		goto err_dsi_enable;
	}

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

	r = hv070wx1_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */
	dsi_video_mode_enable(dssdev, 0x3e);

	PANEL_VDBG("%s exit\n", __func__);

	return 0;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_dsi_enable:
	mdelay(50);
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	mdelay(20);
err0:
	PANEL_ERR("failed to enable (%d)\n", r);
	return r;
}

static void panel_power_off(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	dsi_video_mode_disable(dssdev);

	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, false);
	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, false);

	omapdss_dsi_display_disable(dssdev, false, false);

	mdelay(120);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	PANEL_VDBG("%s exit\n", __func__);
}

static int panel_probe(struct omap_dss_device *dssdev)
{
	int r;

	PANEL_VDBG("%s\n", __func__);

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = &paneldata;
	memset(d2d, 0x0, sizeof(paneldata));

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r) {
		PANEL_ERR("failed to get virtual channel0\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r) {
		PANEL_ERR("failed to set VC_ID0\n");
		goto err;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r) {
		PANEL_ERR("failed to get virtual channel1\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r) {
		PANEL_ERR("failed to set VC_ID1\n");
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	PANEL_VDBG("%s exit\n", __func__);

	return 0;

err:
	return r;
}

static int panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	PANEL_VDBG("%s\n", __func__);

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("%s: already enabled\n", __func__);
	} else {
		dsi_bus_lock(dssdev);
		r = panel_power_on(dssdev);
		dsi_bus_unlock(dssdev);
		if (r) {
			PANEL_DBG("%s: enable failed %d\n", __func__, r);
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			PANEL_DBG("%s: enabled\n", __func__);
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}
	}

	mutex_unlock(&d2d->lock);

	PANEL_VDBG("%s exit\n", __func__);

	return r;
}

static void panel_disable(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		dsi_bus_lock(dssdev);
		panel_power_off(dssdev);
		dsi_bus_unlock(dssdev);
		PANEL_DBG("%s: disabled\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {	
		PANEL_DBG("%s: suspended\n", __func__);
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {	
		PANEL_DBG("%s: already disabled\n", __func__);
	}

	mutex_unlock(&d2d->lock);

	PANEL_VDBG("%s exit\n", __func__);
}

static void panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_VDBG("%s (do nothing)\n", __func__);
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
	if (!dssdev || !timings ||
			dssdev->panel.timings.x_res != timings->x_res ||
			dssdev->panel.timings.y_res != timings->y_res ||
			dssdev->panel.timings.pixel_clock != timings->pixel_clock ||
			dssdev->panel.timings.hsw != timings->hsw ||
			dssdev->panel.timings.hfp != timings->hfp ||
			dssdev->panel.timings.hbp != timings->hbp ||
			dssdev->panel.timings.vsw != timings->vsw ||
			dssdev->panel.timings.vfp != timings->vfp ||
			dssdev->panel.timings.vbp != timings->vbp)
		return -EINVAL;
	return 0;
}

static struct omap_dss_driver panel_driver = {
	.driver_name			= PANEL_NAME,

	.probe					= panel_probe,

	.enable					= panel_enable,
	.disable				= panel_disable,

	.set_timings			= panel_set_timings,
	.get_timings			= panel_get_timings,
	.check_timings			= panel_check_timings,

	.get_resolution			= omapdss_default_get_resolution,
	.get_recommended_bpp	= omapdss_default_get_recommended_bpp,
};

int panel_hv070wx1_init(void)
{
	PANEL_VDBG("%s\n", __func__);
	return omap_dss_register_driver(&panel_driver);
}
