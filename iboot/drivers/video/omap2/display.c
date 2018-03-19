/*
 * linux/drivers/video/omap2/dss/display.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DISPLAY"

#include <common.h>
#include <exports.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/compiler.h> 

#include <icom/omapdss.h>
#include "dss.h"

/*----------------------------------------------------------------------*/

void omapdss_default_get_resolution(struct omap_dss_device *dssdev,
			u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

void default_get_overlay_fifo_thresholds(enum omap_plane plane,
		u32 fifo_size, enum omap_burst_size *burst_size,
		u32 *fifo_low, u32 *fifo_high)
{
	unsigned burst_size_bytes;

	*burst_size = OMAP_DSS_BURST_16x32;
	burst_size_bytes = 16 * 32 / 8;

	*fifo_high = fifo_size - 1;
	*fifo_low = fifo_size - burst_size_bytes;
}

#if 0
void omapdss_display_get_dimensions(struct omap_dss_device *dssdev,
				u32 *width_in_um, u32 *height_in_um)
{
	if (dssdev->driver->get_dimensions) {
		dssdev->driver->get_dimensions(dssdev,
						width_in_um, width_in_um);
	} else {
		*width_in_um = dssdev->panel.width_in_um;
		*height_in_um = dssdev->panel.height_in_um;
	}
}
#endif

int omapdss_default_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	switch (dssdev->type) {
	case OMAP_DISPLAY_TYPE_DPI:
		if (dssdev->phy.dpi.data_lines > 16)
			return 24;
		else
			return 16;

#if 0
	case OMAP_DISPLAY_TYPE_DBI:
#endif
	case OMAP_DISPLAY_TYPE_DSI:
		if (dssdev->ctrl.pixel_size > 16)
			return 24;
		else
			return 16;
#if 0
	case OMAP_DISPLAY_TYPE_VENC:
	case OMAP_DISPLAY_TYPE_SDI:
	case OMAP_DISPLAY_TYPE_HDMI:
		return 24;
#endif
	default:
		BUG();
	}
}
EXPORT_SYMBOL(omapdss_default_get_recommended_bpp);

/* Checks if replication logic should be used. Only use for active matrix,
 * when overlay is in RGB12U or RGB16 mode, and LCD interface is
 * 18bpp or 24bpp */
bool dss_use_replication(struct omap_dss_device *dssdev,
		enum omap_color_mode mode)
{
	int bpp;

	if (mode != OMAP_DSS_COLOR_RGB12U && mode != OMAP_DSS_COLOR_RGB16)
		return false;

	if (dssdev->type == OMAP_DISPLAY_TYPE_DPI &&
			(dssdev->panel.config & OMAP_DSS_LCD_TFT) == 0)
		return false;

	switch (dssdev->type) {
	case OMAP_DISPLAY_TYPE_DPI:
		bpp = dssdev->phy.dpi.data_lines;
		break;
#if 0
	case OMAP_DISPLAY_TYPE_HDMI:
	case OMAP_DISPLAY_TYPE_VENC:
	case OMAP_DISPLAY_TYPE_SDI:
		bpp = 24;
		break;
	case OMAP_DISPLAY_TYPE_DBI:
#endif
	case OMAP_DISPLAY_TYPE_DSI:
		bpp = dssdev->ctrl.pixel_size;
		break;
	default:
		BUG();
	}

	return bpp > 16;
}

void dss_init_device(struct omap_dss_device *dssdev)
{
#if 0
	int i;
#endif
	int r;

	switch (dssdev->type) {
#ifdef CONFIG_OMAP2_DSS_DPI
	case OMAP_DISPLAY_TYPE_DPI:
		r = dpi_init_display(dssdev);
		break;
#endif
#if 0
#ifdef CONFIG_OMAP2_DSS_RFBI
	case OMAP_DISPLAY_TYPE_DBI:
		r = rfbi_init_display(dssdev);
		break;
#endif
#ifdef CONFIG_OMAP2_DSS_VENC
	case OMAP_DISPLAY_TYPE_VENC:
		r = venc_init_display(dssdev);
		break;
#endif
#ifdef CONFIG_OMAP2_DSS_SDI
	case OMAP_DISPLAY_TYPE_SDI:
		r = sdi_init_display(dssdev);
		break;
#endif
#endif
#ifdef CONFIG_OMAP2_DSS_DSI
	case OMAP_DISPLAY_TYPE_DSI:
		r = dsi_init_display(dssdev);
		break;
#endif
#if 0
	case OMAP_DISPLAY_TYPE_HDMI:
		r = hdmi_init_display(dssdev);
		break;
#endif
	default:
		DSSERR("unsupported display '%s', type %d, driver_name '%s'\n",
			dssdev->name, dssdev->type,
			dssdev->driver_name ? dssdev->driver_name : "unknown");
		return;
	}

	if (r) {
		DSSERR("failed to init display '%s', type %d\n",
			dssdev->name, dssdev->type);
		return;
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&dssdev->state_notifiers);

#if 0
	/* create device sysfs files */
	i = 0;
	while ((attr = display_sysfs_attrs[i++]) != NULL) {
		r = device_create_file(&dssdev->dev, attr);
		if (r)
			DSSERR("failed to create sysfs file\n");
	}

	/* create display? sysfs links */
	r = sysfs_create_link(&pdev->dev.kobj, &dssdev->dev.kobj,
			dev_name(&dssdev->dev));
	if (r)
		DSSERR("failed to create sysfs display link\n");
#endif
}

void dss_uninit_device(struct omap_dss_device *dssdev)
{
#if 0
	int i = 0;

	sysfs_remove_link(&pdev->dev.kobj, dev_name(&dssdev->dev));

	while ((attr = display_sysfs_attrs[i++]) != NULL)
		device_remove_file(&dssdev->dev, attr);
#endif

	if (dssdev->manager)
		dssdev->manager->unset_device(dssdev->manager);
}

