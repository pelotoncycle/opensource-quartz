


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

#define	PANEL_NAME	"ls047k1sx01f"

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

#if 0
static struct ls047k1sx01f_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct ls047k1sx01f_board_data *)dssdev->data;
}
#endif

static void ls047k1sx01f_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_DBG("%s\n", __func__);
	*timings = dssdev->panel.timings;
}

static void ls047k1sx01f_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_DBG("set_timings() not implemented\n");
}

static int ls047k1sx01f_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_DBG("%s\n", __func__);

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

static int ls047k1sx01f_probe(struct omap_dss_device *dssdev)
{
	int r = 0, panel_id = 0;
	u8 lcm_id = 0x01;
	u8 id[3];

	PANEL_DBG("ls047k1sx01f_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;	
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = &paneldata;
	memset(d2d, 0x0, sizeof(paneldata));

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	r = gpio_request(dssdev->reset_gpio, NULL);		/* LCM_RSTN */
	if (r) {
		PANEL_ERR("failed to request LCM_RSTN pin\n");
		goto err;
	}
	
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

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		PANEL_ERR("SJ: failed to enable DSI\n");
		goto err;
	}

	gpio_direction_output(dssdev->reset_gpio, 1);
	mdelay(10);
	gpio_direction_output(dssdev->reset_gpio, 0);
	/* assert reset */
	mdelay(1);
	gpio_direction_output(dssdev->reset_gpio, 1);
	mdelay(50);

	dsi_bus_lock(dssdev);

	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xF4, &lcm_id, 1);
	PANEL_INFO("HXID(0xF4)=0x%02x\n", lcm_id);

#if 0
	id[0] = id[1] = id[2] = 0;
	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0x04, id, 3);
	if (r > 0)
		PANEL_INFO("0x04: ID1=0x%02x; ID2=0x%02x; ID3=0x%02x\n", id[0], id[1], id[2]);
#endif

	id[0] = id[1] = id[2] = 0;
	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xDA, &(id[0]), 1);
	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xDB, &(id[1]), 1);
	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xDC, &(id[2]), 1);
	PANEL_INFO("ID1=0x%02x; ID2=0x%02x; ID3=0x%02x\n", id[0], id[1], id[2]);

	dsi_bus_unlock(dssdev);

#if 0
	gpio_direction_output(dssdev->reset_gpio, 0);
	omapdss_dsi_display_disable(dssdev, false, false);
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
#endif

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	if (lcm_id == 0x90) { //Novetek IC
		PANEL_DBG("Novatek driver IC detected\nls047k1sx01f_probe done\n");
		panel_id = 0;
	} else if (lcm_id == 0x00) { //Himax IC ID is 0x92, but the return value afeter reseting will be 0x00
		PANEL_DBG("Himax driver IC deteced\n");
		panel_id = 1;
	} else {
		PANEL_ERR("Unknown panel\n");
		panel_id = 2;
	}

	return panel_id;

err:
	return r;
}

static u8 set_polarity_porch[] = {0x3B, 0x03, 0x01, 0x03, 0x0B, 0x2E};

static int ls047k1sx01f_write_init_config(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[3]; //for DCS short write


	gpio_direction_output(dssdev->reset_gpio, 1);
	mdelay(1);

	buf[0] = 0xFF;
	buf[1] = 0xEE;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to enter test mode\n");
		return r;
	}

	buf[0] = 0x26;
	buf[1] = 0x08;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to turn off OSC\n");
		return r;
	}

	mdelay(1);

	buf[0] = 0x26;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to turn on OSC\n");
		return r;
	}

	buf[0] = 0xFF;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to exit test mode\n");
		return r;
	}

	mdelay(10);

	/* reset the panel */
	gpio_direction_output(dssdev->reset_gpio, 0);
	/* assert reset */
	mdelay(1);
	gpio_direction_output(dssdev->reset_gpio, 1);

	/* wait after releasing reset */
	mdelay(20);

	buf[0] = 0xC2;
	buf[1] = 0x0B;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set LCM timings\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_polarity_porch, sizeof(set_polarity_porch));
	if (r) {
		PANEL_ERR("failed to set clock polarity and LCM porch\n");
		return r;
	}



	buf[0] = 0xFF;
	buf[1] = 0x05;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set CMD2 Page4\n");
		return r;
	}

	buf[0] = 0xFB;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to not reload MTP\n");
		return r;
	}

	buf[0] = 0x2B;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set all gates 2B\n");
		return r;
	}

	buf[0] = 0x2F;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set all gate 2F\n");
		return r;
	}

	buf[0] = 0xFF;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to exit CMD2 Page4\n");
		return r;
	}

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to sleep out\n");
		return r;
	}

	buf[0] = 0xBA;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to initialize LCM\n");
		return r;
	}

	mdelay(100);

	buf[0] = 0xFF;
	buf[1] = 0xEE;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to enter test mode\n");
		return r;
	}

	buf[0] = 0x12;
	buf[1] = 0x50;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set VDD level 12h\n");
		return r;
	}

	buf[0] = 0x13;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set VDD level 13h\n");
		return r;
	}

	buf[0] = 0x6A;
	buf[1] = 0x60;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set internal read margin\n");
		return r;
	}

	buf[0] = 0xFF;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to exit test mode\n");
		return r;
	}

	buf[0] = 0x29;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to display out\n");
		return r;
	}

	return r;
}


static int ls047k1sx01f_power_on(struct omap_dss_device *dssdev)
{
	int r;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	PANEL_DBG("power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		PANEL_ERR("failed to enable DSI\n");
		goto err_disp_enable;
	}

	r = ls047k1sx01f_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);
	

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */
	dsi_video_mode_enable(dssdev, 0x3e);

	PANEL_DBG("power_on done\n");

	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
		
	return r;
}

static int ls047k1sx01f_write_pwroff_config(struct omap_dss_device *dssdev)
{

	int r = 0;
	u8 buf[1];

	buf[0] = 0x28;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to display off\n");
		return r;
	}

	buf[0] = 0x10;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to sleep in\n");
		return r;
	}

	mdelay(100);

	return r;
}

static void ls047k1sx01f_power_off(struct omap_dss_device *dssdev)
{
	int r;

	dsi_video_mode_disable(dssdev);

	r = ls047k1sx01f_write_pwroff_config(dssdev);
	if (r)
		PANEL_ERR("panel pwroff fail\n");

	gpio_direction_output(dssdev->reset_gpio, 0);

	mdelay(20);

	omapdss_dsi_display_disable(dssdev, false, false);

	mdelay(300);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

#if 0
static void ls047k1sx01f_stop(struct omap_dss_device *dssdev)
{
	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	ls047k1sx01f_power_off(dssdev);

	dsi_bus_unlock(dssdev);
}
#endif

static void ls047k1sx01f_disable(struct omap_dss_device *dssdev)
{
	PANEL_DBG("%s\n", __func__);
	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("ls047k1sx01f_disable: disable\n");

		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		dsi_bus_lock(dssdev);

		ls047k1sx01f_power_off(dssdev);

		dsi_bus_unlock(dssdev);
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		PANEL_DBG("ls047k1sx01f_disable: suspended\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		PANEL_DBG("ls047k1sx01f_disable: already disabled\n");
	}	
	
	mutex_unlock(&d2d->lock);

}

static int ls047k1sx01f_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&d2d->lock);
	
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("ls047k1sx01f_enable: already enabled\n");
		mutex_unlock(&d2d->lock);
		return 0;
	}
	else {
		PANEL_DBG("ls047k1sx01f_enable: enable\n");

		dsi_bus_lock(dssdev);

		r = ls047k1sx01f_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			PANEL_ERR("ls047k1sx01f_enable: enable failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}

		mutex_unlock(&d2d->lock);

		return r;
	}
}



static struct omap_dss_driver ls047k1sx01f_driver = {
	.driver_name	= PANEL_NAME,
	.probe		= ls047k1sx01f_probe,

	.enable		= ls047k1sx01f_enable,
	.disable	= ls047k1sx01f_disable,


	.get_resolution	= omapdss_default_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= ls047k1sx01f_get_timings,
	.set_timings	= ls047k1sx01f_set_timings,
	.check_timings	= ls047k1sx01f_check_timings,

};

int __init panel_ls047k1sx01f_init(void)
{
	PANEL_DBG("%s\n", __func__);
	return omap_dss_register_driver(&ls047k1sx01f_driver);

}

