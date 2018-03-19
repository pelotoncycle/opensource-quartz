/*
 * InnoComm Talos10 panel support
 *
 * Copyright (C) 2011 InnoComm Mobile Technology Corp.
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

/*#define DSS_DEBUG*/
/*#define DSS_VERBOSE_DEBUG*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <plat/gpio.h>
#include <video/omapdss.h>

/**************************************************************************************************/

/* DSS printk */
#define DSS_ERR(dssdev, format, arg...)		\
	dev_printk(KERN_ERR, &((dssdev)->dev), format, ##arg)
#define DSS_INFO(dssdev, format, arg...)	\
	dev_printk(KERN_INFO, &((dssdev)->dev), format, ##arg)
#ifdef DEBUG
#ifdef DSS_DEBUG
#define DSS_DBG(dssdev, format, arg...)		\
	dev_printk(KERN_INFO, &((dssdev)->dev), format, ##arg)
#else
#define DSS_DBG(dssdev, format, arg...)		\
	dev_printk(KERN_DEBUG, &((dssdev)->dev), format, ##arg)
#endif /* DSS_DEBUG */
#else
#define DSS_DBG(dssdev, format, arg...)
#endif /* DEBUG */
#ifdef VERBOSE_DEBUG
#ifdef DSS_VERBOSE_DEBUG
#define DSS_VDBG(dssdev, format, arg...)	\
	dev_printk(KERN_INFO, &((dssdev)->dev), format, ##arg)
#else
#define DSS_VDBG(dssdev, format, arg...)	\
	dev_printk(KERN_DEBUG, &((dssdev)->dev), format, ##arg)
#endif /* DSS_VERBOSE_DEBUG */
#else
#define DSS_VDBG(dssdev, format, arg...)
#endif /* VERBOSE_DEBUG */

/**************************************************************************************************/

static DEFINE_MUTEX(panel_mutex);

/**************************************************************************************************/

/*
 * defines HFB, HSW, HBP, VFP, VSW, VBP as shown below
 */
static struct omap_video_timings panel_timings = {
	.x_res          = 1280,
	.y_res          = 800,
	.pixel_clock	= 76800,
	.hsw		= 100,
	.hfp		= 50,
	.hbp		= 50,
	.vsw		= 25,
	.vfp		= 20,
	.vbp		= 20,
};

static void panel_poweron(struct omap_dss_device *dssdev)
{
    int r;

    DSS_VDBG(dssdev, "%s\n", __func__);    

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			DSS_ERR(dssdev, "%s: platform_enable failed %d\n", __func__, r);
	}

	/* dssdev->skip_init is cleared in omapdss_dpi_display_enable() */
	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		DSS_ERR(dssdev, "%s: dpi enable failed %d\n", __func__, r);
}

static void panel_poweroff(struct omap_dss_device *dssdev)
{
    DSS_VDBG(dssdev, "%s\n", __func__);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);

	/* Wait for the voltage off & vsyncs */
	msleep(40);
}


/**************************************************************************************************/
/* OMAP DSS functions */
/**************************************************************************************************/

static int panel_probe(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_GATE_HVSYNC;
	
	memcpy(&dssdev->panel.timings, &panel_timings, sizeof(struct omap_video_timings));


	mutex_unlock(&panel_mutex);

	return 0;
}

static void panel_remove(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);
}

static int panel_enable(struct omap_dss_device *dssdev)
{
	int ret = 0;

	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		DSS_INFO(dssdev, "%s: was enabled\n", __func__);
	} else {
		panel_poweron(dssdev);
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		DSS_INFO(dssdev, "%s: enabled\n", __func__);
	}

	mutex_unlock(&panel_mutex);

	return ret;
}

static void panel_disable(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {	
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		panel_poweroff(dssdev);
		DSS_INFO(dssdev, "%s: disabled\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		DSS_INFO(dssdev, "%s: was suspended\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		DSS_INFO(dssdev, "%s: was disabled\n", __func__);
	}

	mutex_unlock(&panel_mutex);
}

static int panel_suspend(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
		panel_poweroff(dssdev);
		DSS_INFO(dssdev, "%s: suspended\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		DSS_INFO(dssdev, "%s: was suspended\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		DSS_INFO(dssdev, "%s: was disabled\n", __func__);
	}

	mutex_unlock(&panel_mutex);

	return 0;
}

static int panel_resume(struct omap_dss_device *dssdev)
{
	int ret = 0;

	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		panel_poweron(dssdev);
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE ;
		DSS_INFO(dssdev, "%s: resumed\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		DSS_INFO(dssdev, "%s: was enabled\n", __func__);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		DSS_INFO(dssdev, "%s: was disabled\n", __func__);
	}

	mutex_unlock(&panel_mutex);

	return ret;
}

static void panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

/**************************************************************************************************/

static void panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static struct omap_dss_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,

	.enable		= panel_enable,
	.disable	= panel_disable,
	.suspend	= panel_suspend,
	.resume		= panel_resume,
	.set_timings	= panel_set_timings,
	.get_timings	= panel_get_timings,
	.check_timings	= panel_check_timings,

	.get_resolution	= panel_get_resolution,	
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.driver		= {
		.name	= "talos10_panel",
		.owner 	= THIS_MODULE,
	},
};

/**************************************************************************************************/

static int __init panel_init(void)
{
	return omap_dss_register_driver(&panel_driver);
}

static void __exit panel_exit(void)
{
	omap_dss_unregister_driver(&panel_driver);
}

module_init(panel_init);
module_exit(panel_exit);
MODULE_DESCRIPTION("OMAP4 Talos10 Panel");
MODULE_LICENSE("GPL");
