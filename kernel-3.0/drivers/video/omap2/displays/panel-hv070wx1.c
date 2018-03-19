/*
 * InnoComm Zeus panel support
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

#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <video/omapdss.h>




static struct omap_video_timings hv070wx1_timings;
static struct hv070wx1_board_data *get_board_data(struct omap_dss_device
					*dssdev) __attribute__ ((unused));

/* device private data structure */
struct hv070wx1_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
};

static struct hv070wx1_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct hv070wx1_board_data *)dssdev->data;
}

static void hv070wx1_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hv070wx1_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_info(&dssdev->dev, "set_timings() not implemented\n");
}

static int hv070wx1_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (hv070wx1_timings.x_res != timings->x_res ||
			hv070wx1_timings.y_res != timings->y_res ||
			hv070wx1_timings.pixel_clock != timings->pixel_clock ||
			hv070wx1_timings.hsw != timings->hsw ||
			hv070wx1_timings.hfp != timings->hfp ||
			hv070wx1_timings.hbp != timings->hbp ||
			hv070wx1_timings.vsw != timings->vsw ||
			hv070wx1_timings.vfp != timings->vfp ||
			hv070wx1_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void hv070wx1_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = hv070wx1_timings.x_res;
	*yres = hv070wx1_timings.y_res;
}

static int hv070wx1_probe(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "hv070wx1_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	hv070wx1_timings = dssdev->panel.timings;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");
		goto err;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel1\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	dev_dbg(&dssdev->dev, "hv070wx1_probe done\n");
	return 0;
err:
	kfree(d2d);
	return r;
}

static void hv070wx1_remove(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	kfree(d2d);
}


static int hv070wx1_write_init_config(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[2];

	buf[0] = 0xAE;
	buf[1] = 0x0D;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		dev_dbg(&dssdev->dev, "failed to initialize LCM\n");
		return r;
	}

/*
	buf[0] = 0xB1;
	buf[1] = 0xEF;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2); // BIST mode
	if (r) {
		dev_dbg(&dssdev->dev, "failed to go to BIST mode\n");
		return r;
	}
*/

	return r;
}



static int hv070wx1_power_on(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	dev_dbg(&dssdev->dev, "power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	if (dssdev->skip_init) {
		dev_dbg(&dssdev->dev, "skip init\n");
		dssdev->manager->enable(dssdev->manager);
		dssdev->skip_init = false;
	} else {
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

		/* reset tc358765 bridge */
		//tc358765_hw_reset(dssdev);

		/* do extra job to match kozio registers (???) */
		//dsi_videomode_panel_preinit(dssdev);

		/* Need to wait a certain time - Toshiba Bridge Constraint */
		/* msleep(400); */

		/* configure D2L chip DSI-RX configuration registers */
		r = hv070wx1_write_init_config(dssdev);
		if (r)
			goto err_write_init;

		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

		/* 0x0e - 16bit
		 * 0x1e - packed 18bit
		 * 0x2e - unpacked 18bit
		 * 0x3e - 24bit
		 */
		switch (dssdev->ctrl.pixel_size) {
		case 18:
			dsi_video_mode_enable(dssdev, 0x1e);
			break;
		case 24:
			dsi_video_mode_enable(dssdev, 0x3e);
			break;
		default:
			dev_warn(&dssdev->dev, "not expected pixel size: %d\n",
						dssdev->ctrl.pixel_size);
		}
	}

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	dssdev->skip_init = false;
	return r;
}

static void hv070wx1_power_off(struct omap_dss_device *dssdev)
{
	//struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);

	dsi_video_mode_disable(dssdev);

	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, false);
	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, false);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void hv070wx1_disable(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		dsi_bus_lock(dssdev);

		hv070wx1_power_off(dssdev);

		dsi_bus_unlock(dssdev);

		dev_dbg(&dssdev->dev, "hv070wx1_disable: disabled\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dev_dbg(&dssdev->dev, "hv070wx1_disable: suspended\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		if (dssdev->skip_init) {
			dev_dbg(&dssdev->dev, "hv070wx1_disable: skip init\n");
			dsi_bus_lock(dssdev);
			if (hv070wx1_power_on(dssdev) == 0)
				hv070wx1_power_off(dssdev);
			dsi_bus_unlock(dssdev);
		} else
			dev_dbg(&dssdev->dev, "hv070wx1_disable: already disabled\n");
	}

	mutex_unlock(&d2d->lock);
}

static int hv070wx1_enable(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&d2d->lock);
	
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "hv070wx1_enable: already enabled\n");
	} else {
		dsi_bus_lock(dssdev);

		r = hv070wx1_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			dev_dbg(&dssdev->dev, "hv070wx1_enable: enable failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dev_dbg(&dssdev->dev, "hv070wx1_enable: enabled\n");
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}

	}

	mutex_unlock(&d2d->lock);

	return r;
}

static int hv070wx1_suspend(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

		dsi_bus_lock(dssdev);

		hv070wx1_power_off(dssdev);

		dsi_bus_unlock(dssdev);

		dev_dbg(&dssdev->dev, "hv070wx1_suspend: suspended\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dev_dbg(&dssdev->dev, "hv070wx1_suspend: already suspended\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		dev_dbg(&dssdev->dev, "hv070wx1_suspend: disabled\n");
	}

	mutex_unlock(&d2d->lock);

	return 0;
}

static int hv070wx1_resume(struct omap_dss_device *dssdev)
{
	struct hv070wx1_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "hv070wx1_resume: enabled\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		dev_dbg(&dssdev->dev, "hv070wx1_resume: disabled\n");
	} else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dsi_bus_lock(dssdev);

		r = hv070wx1_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			dev_dbg(&dssdev->dev, "hv070wx1_resume: resume failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dev_dbg(&dssdev->dev, "hv070wx1_resume: resumed\n");
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static struct omap_dss_driver hv070wx1_driver = {
	.probe		= hv070wx1_probe,
	.remove		= hv070wx1_remove,

	.enable		= hv070wx1_enable,
	.disable	= hv070wx1_disable,
	.suspend	= hv070wx1_suspend,
	.resume		= hv070wx1_resume,

	.get_resolution	= hv070wx1_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= hv070wx1_get_timings,
	.set_timings	= hv070wx1_set_timings,
	.check_timings	= hv070wx1_check_timings,

	.driver         = {
		.name   = "zeus_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init hv070wx1_init(void)
{
	omap_dss_register_driver(&hv070wx1_driver);
	return 0;
}

static void __exit hv070wx1_exit(void)
{
	omap_dss_unregister_driver(&hv070wx1_driver);
}

module_init(hv070wx1_init);
module_exit(hv070wx1_exit);

MODULE_AUTHOR("SinJhe Yu <sinjhe.yu@innocomm.com>");
MODULE_DESCRIPTION("OMAP4 Zeus Panel");
MODULE_LICENSE("GPL");
