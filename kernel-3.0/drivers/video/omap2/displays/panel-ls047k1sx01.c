/*
 * InnoComm Saga panel support
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
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




static struct omap_video_timings ls047k1sx01_timings;
static struct ls047k1sx01_board_data *get_board_data(struct omap_dss_device
					*dssdev) __attribute__ ((unused));

/* device private data structure */
struct ls047k1sx01_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
};

static struct ls047k1sx01_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct ls047k1sx01_board_data *)dssdev->data;
}

static void ls047k1sx01_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void ls047k1sx01_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_info(&dssdev->dev, "set_timings() not implemented\n");
}

static int ls047k1sx01_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (ls047k1sx01_timings.x_res != timings->x_res ||
			ls047k1sx01_timings.y_res != timings->y_res ||
			ls047k1sx01_timings.pixel_clock != timings->pixel_clock ||
			ls047k1sx01_timings.hsw != timings->hsw ||
			ls047k1sx01_timings.hfp != timings->hfp ||
			ls047k1sx01_timings.hbp != timings->hbp ||
			ls047k1sx01_timings.vsw != timings->vsw ||
			ls047k1sx01_timings.vfp != timings->vfp ||
			ls047k1sx01_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void ls047k1sx01_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = ls047k1sx01_timings.x_res;
	*yres = ls047k1sx01_timings.y_res;
}





static int ls047k1sx01_write_init_config(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[6];


	gpio_direction_output(dssdev->reset_gpio, 1);
	msleep(1);

	/* reset the panel */
	gpio_direction_output(dssdev->reset_gpio, 0);
	/* assert reset */
	msleep(1);
	gpio_direction_output(dssdev->reset_gpio, 1);

	/* wait after releasing reset */
	msleep(20);

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		dev_err(&dssdev->dev, "failed to sleep out\n");
		return r;
	}

	msleep(120);

	buf[0] = 0xB9;
	buf[1] = 0xFF;
	buf[2] = 0x83;
	buf[3] = 0x92;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		dev_err(&dssdev->dev, "failed to initial setting\n");
		return r;
	}

	buf[0] = 0xD4;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		dev_err(&dssdev->dev, "failed to EQ funtion enable\n");
		return r;
	}

	buf[0] = 0xBA;
	buf[1] = 0x12; //3 lanes
	buf[2] = 0x83;
	buf[3] = 0x00;
	buf[4] = 0xD6;
	buf[5] = 0xC6;
	buf[6] = 0x00;
	buf[7] = 0x0A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 8);
	if (r) {
		dev_err(&dssdev->dev, "failed to modify DSI settings\n");
		return r;
	}

	buf[0] = 0xC0;
	buf[1] = 0x01;
	buf[2] = 0x94;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		dev_err(&dssdev->dev, "failed to STBA setting for Power saving\n");
		return r;
	}

	buf[0] = 0xC6;
	buf[1] = 0x35;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x04;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 5);
	if (r) {
		dev_err(&dssdev->dev, "failed to Countermeasure for mode change flashing issue\n");
		return r;
	}

	buf[0] = 0xD5;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x02;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		dev_err(&dssdev->dev, "failed to EQ delay\n");
		return r;
	}

	buf[0] = 0xBF;
	buf[1] = 0x05;
	buf[2] = 0x60;
	buf[3] = 0x02;
	//buf[4] = 0x00; //SJ add
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		dev_err(&dssdev->dev, "failed to PTBF setting\n");
		return r;
	}

	buf[0] = 0xC2;
	buf[1] = 0x03;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		dev_err(&dssdev->dev, "failed to enter video mode\n");
		return r;
	}

	buf[0] = 0x35;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		dev_err(&dssdev->dev, "failed to TE-ON\n");
		return r;
	}

	buf[0] = 0x29;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		dev_err(&dssdev->dev, "failed to display out\n");
		return r;
	}

	return r;
}



static int ls047k1sx01_power_on(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);
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

		r = ls047k1sx01_write_init_config(dssdev);
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

static int ls047k1sx01_write_pwroff_config(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[16];

	buf[0] = 0x28;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		dev_dbg(&dssdev->dev, "failed to display off\n");
		return r;
	}

	buf[0] = 0x10;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		dev_dbg(&dssdev->dev, "failed to sleep in\n");
		return r;
	}

	msleep(100);

	gpio_set_value(dssdev->reset_gpio, 0);

	msleep(10);

	return r;
}

static void ls047k1sx01_power_off(struct omap_dss_device *dssdev)
{
	int r;

	r = ls047k1sx01_write_pwroff_config(dssdev);
	if (r)
		dev_dbg(&dssdev->dev, "panel pwroff fail\n");

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void ls047k1sx01_stop(struct omap_dss_device *dssdev)
{
	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	ls047k1sx01_power_off(dssdev);

	dsi_bus_unlock(dssdev);
}

static void ls047k1sx01_disable(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&d2d->lock);

	

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_disable: disable\n");

		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		dsi_bus_lock(dssdev);

		ls047k1sx01_power_off(dssdev);

		dsi_bus_unlock(dssdev);
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_disable: suspended\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		if (dssdev->skip_init) {
			dev_dbg(&dssdev->dev, "ls047k1sx01_disable: skip init\n");
			dsi_bus_lock(dssdev);
			if (ls047k1sx01_power_on(dssdev) == 0)
				ls047k1sx01_power_off(dssdev);
			dsi_bus_unlock(dssdev);
		} else
			dev_dbg(&dssdev->dev, "ls047k1sx01_disable: already disabled\n");
	}	
	
	
	
	mutex_unlock(&d2d->lock);
}

static int ls047k1sx01_enable(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&d2d->lock);
	
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_enable: already enabled\n");
		mutex_unlock(&d2d->lock);
		return 0;
	}
	else {
		dev_dbg(&dssdev->dev, "ls047k1sx01_enable: enable\n");

		dsi_bus_lock(dssdev);

		r = ls047k1sx01_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			dev_dbg(&dssdev->dev, "ls047k1sx01_enable: enable failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}

		mutex_unlock(&d2d->lock);

		return r;
	}
}

static int LCM_ESD_INT = 0;

extern struct ls047k1sx01_data *g_d2d;
static int esd_int_irq = 0;
static struct delayed_work update_work;
extern spinlock_t lock;

static irqreturn_t esd_int_fn(int irq, void *handle)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	disable_irq_nosync(esd_int_irq);

	schedule_delayed_work(&update_work, msecs_to_jiffies(20));

	spin_unlock_irqrestore(&lock, flags);

	return IRQ_HANDLED;
}

static void esd_int_work(struct work_struct *work)
{
	if (gpio_get_value(LCM_ESD_INT) == 0) {
		printk("LCM reset because ESD INT \n");

		ls047k1sx01_disable(g_d2d->dssdev);

		msleep(2000);

		ls047k1sx01_enable(g_d2d->dssdev);
	}

	enable_irq(esd_int_irq);
	msleep(500);
}

static int ls047k1sx01_suspend(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&d2d->lock);

	disable_irq(esd_int_irq);
#if !defined(CONFIG_GPIO_OMAP_IRQ_SUPPORT_DISABLE_FUNCTION)
	msleep(500); // need this delay time to prevent interrupt during panel suspending
#endif

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_suspend: suspend\n");

		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

		ls047k1sx01_stop(dssdev);
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_suspend: already suspended\n");
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_suspend: disabled\n");
	}

	mutex_unlock(&d2d->lock);

	return 0;
}

static int ls047k1sx01_resume(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_resume: enabled\n");
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_resume: disabled\n");
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		dev_dbg(&dssdev->dev, "ls047k1sx01_resume: resume\n");

		dsi_bus_lock(dssdev);

		r = ls047k1sx01_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			dev_dbg(&dssdev->dev, "ls047k1sx01_resume: resume failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}
	}

	enable_irq(esd_int_irq);

	mutex_unlock(&d2d->lock);

	return r;
}




static int ls047k1sx01_probe(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d;
	int r = 0;
	int err = 0;

	dev_dbg(&dssdev->dev, "ls047k1sx01_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	ls047k1sx01_timings = dssdev->panel.timings;
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

	if (system_rev >= 3)
		LCM_ESD_INT = 45;
	else
		LCM_ESD_INT = 120;

	esd_int_irq = OMAP_GPIO_IRQ(LCM_ESD_INT);

	err = gpio_request(LCM_ESD_INT, "LCM_ESD_INT");
	if (err) {
		printk(KERN_ERR "Could not request GPIO %d\n", LCM_ESD_INT);
	} else {
		gpio_direction_input(LCM_ESD_INT);
	}

	g_d2d = d2d;

	INIT_DELAYED_WORK(&update_work, esd_int_work);

	err = request_irq(esd_int_irq, esd_int_fn, IRQF_DISABLED|IRQF_TRIGGER_FALLING, "esd_int_irq", d2d);
	if (err == 0) {
		printk("%s, irq(%d)\n", __func__, esd_int_irq);
	} else {
		printk("%s, request_irq failed\n", __func__);
	}

	dev_dbg(&dssdev->dev, "ls047k1sx01_probe done\n");
	return 0;
err:
	kfree(d2d);
	return r;
}

static void ls047k1sx01_remove(struct omap_dss_device *dssdev)
{
	struct ls047k1sx01_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	free_irq(esd_int_irq, d2d);

	kfree(d2d);
}

static struct omap_dss_driver ls047k1sx01_driver = {
	.probe		= ls047k1sx01_probe,
	.remove		= ls047k1sx01_remove,

	.enable		= ls047k1sx01_enable,
	.disable	= ls047k1sx01_disable,
	.suspend	= ls047k1sx01_suspend,
	.resume		= ls047k1sx01_resume,

	.get_resolution	= ls047k1sx01_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= ls047k1sx01_get_timings,
	.set_timings	= ls047k1sx01_set_timings,
	.check_timings	= ls047k1sx01_check_timings,

	.driver         = {
		.name   = "ls047k1sx01_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init ls047k1sx01_init(void)
{
	omap_dss_register_driver(&ls047k1sx01_driver);
	return 0;
}

static void __exit ls047k1sx01_exit(void)
{
	omap_dss_unregister_driver(&ls047k1sx01_driver);
}

module_init(ls047k1sx01_init);
module_exit(ls047k1sx01_exit);
