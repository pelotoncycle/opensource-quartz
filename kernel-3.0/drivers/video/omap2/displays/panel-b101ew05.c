/*
 * InnoComm Echo panel support
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
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

//#define DEBUG
//#define VERBOSE_DEBUG

//#define DSS_DEBUG
//#define DSS_VERBOSE_DEBUG

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
static struct omap_video_timings b101ew05_timings = {
	.x_res          = 1280,
	.y_res          = 800,
	.pixel_clock	= 76800,
	.hsw		= 42,
	.hfp		= 110,
	.hbp		= 110,
	.vsw		= 10,
	.vfp		= 40,
	.vbp		= 40,
};

static int b101ew05_poweron(struct omap_dss_device *dssdev)
{
    int r;

    DSS_VDBG(dssdev, "%s\n", __func__);    

    // Could not find exact pixel clock. Requested 100032 kHz, got 85333 kHz
	r = omapdss_dpi_display_enable(dssdev);
	if (r)
	{
        goto err_dpi;
	}

    /* Wait for vsyncs */
    msleep(40);

    return 0;

err_dpi:
	return r;

}

static void b101ew05_poweroff(struct omap_dss_device *dssdev)
{
    DSS_VDBG(dssdev, "%s\n", __func__);

    /* Wait for the voltage off & vsyncs */
	msleep(40);

	omapdss_dpi_display_disable(dssdev);

	msleep(50);
}


/**************************************************************************************************/
/* OMAP DSS functions */
/**************************************************************************************************/

static int b101ew05_probe(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC;
	
	memcpy(&dssdev->panel.timings, &b101ew05_timings, sizeof(struct omap_video_timings));


	mutex_unlock(&panel_mutex);

	return 0;
}

static void b101ew05_remove(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);
}

static int b101ew05_enable(struct omap_dss_device *dssdev)
{
	int ret = 0;

	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {	
		DSS_DBG(dssdev, "panel_enable: panel is already enabled\n");
	} else {
		if (dssdev->platform_enable) {
			ret = dssdev->platform_enable(dssdev);
			if (ret) {
				dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
				mutex_unlock(&panel_mutex);
				DSS_ERR(dssdev, "failed to enable panel\n");
				return ret;
			}
		}

		ret = b101ew05_poweron(dssdev);
		if (ret) {
			DSS_ERR(dssdev, "failed to power on panel (%d)\n", ret);
		}

		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&panel_mutex);

	return ret;
}

static void b101ew05_disable(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {	
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		if (dssdev->platform_disable)
			dssdev->platform_disable(dssdev);

		b101ew05_poweroff(dssdev);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {	
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		DSS_DBG(dssdev, "panel_disable: panel is disabled\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {	
		DSS_DBG(dssdev, "panel_disable: panel is already disabled\n");
	}

	mutex_unlock(&panel_mutex);
}

static int b101ew05_suspend(struct omap_dss_device *dssdev)
{
	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {	
        dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
		if (dssdev->platform_disable)
			dssdev->platform_disable(dssdev);

		b101ew05_poweroff(dssdev);
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {	
		DSS_DBG(dssdev, "panel_suspend: panel is already suspended\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {	
		DSS_DBG(dssdev, "panel_suspend: panel is disabled\n");
	}

	mutex_unlock(&panel_mutex);

	return 0;
}

static int b101ew05_resume(struct omap_dss_device *dssdev)
{
	int ret = 0;

	DSS_VDBG(dssdev, "%s\n", __func__);

	mutex_lock(&panel_mutex);

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		if (dssdev->platform_enable) {
			ret = dssdev->platform_enable(dssdev);
			if (ret) {
				mutex_unlock(&panel_mutex);
				DSS_ERR(dssdev, "failed to resume panel\n");
				return ret;
			}
		}

		ret = b101ew05_poweron(dssdev);
		if (ret) {
			DSS_ERR(dssdev, "failed to power on panel (%d)\n", ret);
		}

		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE ;
	} else if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		DSS_DBG(dssdev, "panel_resume: panel is already enabled\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
	    DSS_DBG(dssdev, "panel_resume: panel is disabled\n");
	}

	mutex_unlock(&panel_mutex);

	return ret;
}

static void b101ew05_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void b101ew05_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int b101ew05_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

/**************************************************************************************************/

static void b101ew05_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static struct omap_dss_driver panel_driver = {
	.probe		= b101ew05_probe,
	.remove		= b101ew05_remove,

	.enable		= b101ew05_enable,
	.disable	= b101ew05_disable,
	.suspend	= b101ew05_suspend,
	.resume		= b101ew05_resume,
	.set_timings	= b101ew05_set_timings,
	.get_timings	= b101ew05_get_timings,
	.check_timings	= b101ew05_check_timings,

	.get_resolution	= b101ew05_get_resolution,	
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.driver		= {
		.name	= "b101ew05_panel",
		.owner 	= THIS_MODULE,
	},
};

/**************************************************************************************************/

static int __init panel_init(void)
{
#ifdef VERBOSE_DEBUG
	pr_info("b101ew05: panel_init()\n");
#endif /* VERBOSE_DEBUG */

	return omap_dss_register_driver(&panel_driver);
}

static void __exit panel_exit(void)
{
#ifdef VERBOSE_DEBUG
	pr_info("b101ew05: panel_exit()\n");
#endif /* VERBOSE_DEBUG */

	omap_dss_unregister_driver(&panel_driver);
}

module_init(panel_init);
module_exit(panel_exit);
MODULE_DESCRIPTION("OMAP4 Echo Panel");
MODULE_LICENSE("GPL");
