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

#define	PANEL_NAME	"ls047k1sx01"

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
static struct ls047k1sx01_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct ls047k1sx01f_board_data *)dssdev->data;
}
#endif

static void ls047k1sx01_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_DBG("%s\n", __func__);
	*timings = dssdev->panel.timings;
}

static void ls047k1sx01_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	PANEL_DBG("set_timings() not implemented\n");
}

static int ls047k1sx01_check_timings(struct omap_dss_device *dssdev,
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

static int ls047k1sx01_probe(struct omap_dss_device *dssdev)
{
	int r = 0;

	PANEL_DBG("ls047k1sx01_probe\n");

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

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	PANEL_DBG("ls047k1sx01_probe done\n");
	return 0;

err:
	return r;
}

static u8 set_ext_cmd[] = {0xB9, 0xFF, 0x83, 0x92};
static u8 set_mipi[] = {0xBA, 0x12, 0x83, 0x00, 0xD6, 0xC6, 0x00, 0x0A};
static u8 set_gamma_pwr[] = {0xB1, 0x7C, 0x00, 0x44, 0x25, 0x00, 0x0D, 0x0D, 0x1B, 0x12, 0x3F, 0x3F, 0x42, 0x72};
static u8 set_lpts_cmdmode[] = {0xB4, 0x00, 0x00, 0x05, 0x00, 0xA1, 0x05, 0x06, 0x92, 0x00, 0x01, 0x06, 0x00, 0x02, 0x02, 0x00, 0x1D, 0x08, 0x0D, 0x0D, 0x01, 0x00, 0x07, 0x7A};
static u8 set_lpts_videomode[] = {0xD8, 0x00, 0x00, 0x05, 0x00, 0xA1, 0x05, 0x06, 0x92, 0x00, 0x01, 0x06, 0x00, 0x02, 0x02, 0x00, 0x1D, 0x08, 0x0D, 0x0D, 0x01, 0x00, 0x07, 0x7A};
static u8 set_gamma_r[] = {0xE0, 0x00, 0x09, 0x0E, 0x24, 0x28, 0x31, 0x1C, 0x35, 0x05, 0x0D, 0x10, 0x13, 0x17, 0x13, 0x15, 0x12, 0x18, 0x00, 0x09, 0x0F, 0x2F, 0x34, 0x3E, 0x1C, 0x3A, 0x06, 0x0E, 0x0E, 0x11, 0x15, 0x12, 0x14, 0x11, 0x18};
static u8 set_gamma_g[] = {0xE1, 0x00, 0x09, 0x0E, 0x24, 0x28, 0x31, 0x1C, 0x35, 0x05, 0x0D, 0x10, 0x13, 0x17, 0x13, 0x15, 0x12, 0x18, 0x00, 0x09, 0x0F, 0x2F, 0x34, 0x3E, 0x1C, 0x3A, 0x06, 0x0E, 0x0E, 0x11, 0x15, 0x12, 0x14, 0x11, 0x18};
static u8 set_gamma_b[] = {0xE2, 0x00, 0x09, 0x0E, 0x24, 0x28, 0x31, 0x1C, 0x35, 0x05, 0x0D, 0x10, 0x13, 0x17, 0x13, 0x15, 0x12, 0x18, 0x00, 0x09, 0x0F, 0x2F, 0x34, 0x3E, 0x1C, 0x3A, 0x06, 0x0E, 0x0E, 0x11, 0x15, 0x12, 0x14, 0x11, 0x18};
static u8 flash_issue[] = {0xC6, 0x35, 0x00, 0x00, 0x04};
static u8 set_eq_delay[] = {0xD5, 0x00, 0x00, 0x02};
static u8 set_ptba[] = {0xBF, 0x05, 0x60, 0x02};

static int ls047k1sx01_write_init_config(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[3]; //for DCS short write
	u8 lcm_id = 0;

	gpio_direction_output(dssdev->reset_gpio, 1);
	mdelay(1);

	/* reset the panel */
	gpio_direction_output(dssdev->reset_gpio, 0);
	/* assert reset */
	mdelay(1);
	gpio_direction_output(dssdev->reset_gpio, 1);

	/* wait after releasing reset */
	mdelay(20);

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to sleep out\n");
		return r;
	}

	mdelay(120);

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_ext_cmd, sizeof(set_ext_cmd));
	if (r) {
		PANEL_ERR("failed to set extension command\n");
		return r;
	}

	r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xF4, &lcm_id, 1);
	if (r == 0) {
		PANEL_ERR("failed to read panel ID\n");
		return r;
	}
	PANEL_DBG("lcm_id = 0x%02x\n", lcm_id);
	if (lcm_id != 0x92)
		PANEL_WARN("unkown HXID(0xF4) (0x%02x), but still using Himax panel setting\n", lcm_id);

	buf[0] = 0xD4;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to enable EQ funtion\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_mipi, sizeof(set_mipi));
	if (r) {
		PANEL_ERR("failed to modify DSI settings\n");
		return r;
	}

	buf[0] = 0xC0;
	buf[1] = 0x01;
	buf[2] = 0x94;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set STBA for Power saving\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_gamma_pwr, sizeof(set_gamma_pwr));
	if (r) {
		PANEL_ERR("failed to set power for gamma 2.2\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_lpts_cmdmode, sizeof(set_lpts_cmdmode));
	if (r) {
		PANEL_ERR("failed to Set LTPS timing for Command mode\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_lpts_videomode, sizeof(set_lpts_videomode));
	if (r) {
		PANEL_ERR("failed to Set LTPS timing for Video mode\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_gamma_r, sizeof(set_gamma_r));
	if (r) {
		PANEL_ERR("failed to Set R color for Gamma2.2\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_gamma_g, sizeof(set_gamma_g));
	if (r) {
		PANEL_ERR("failed to Set G color for Gamma2.2\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_gamma_b, sizeof(set_gamma_g));
	if (r) {
		PANEL_ERR("failed to Set B color for Gamma2.2\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, flash_issue, sizeof(flash_issue));
	if (r) {
		PANEL_ERR("failed to Countermeasure for mode change flashing issue\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_eq_delay, sizeof(set_eq_delay));
	if (r) {
		PANEL_ERR("failed to EQ delay\n");
		return r;
	}

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, set_ptba, sizeof(set_ptba));
	if (r) {
		PANEL_ERR("failed to set PTBF\n");
		return r;
	}

/*
	buf[0] = 0x35;
	buf[1] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to TE-ON\n");
		return r;
	}

	buf[0] = 0xC2;
	buf[1] = 0x08;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to enter video mode\n");
		return r;
	}
*/

	buf[0] = 0x29;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to display out\n");
		return r;
	}

	return r;
}

static int ls047k1sx01_power_on(struct omap_dss_device *dssdev)
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

	r = ls047k1sx01_write_init_config(dssdev);
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

static int ls047k1sx01_write_pwroff_config(struct omap_dss_device *dssdev)
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

static void ls047k1sx01_power_off(struct omap_dss_device *dssdev)
{
	int r;

	dsi_video_mode_disable(dssdev);

	r = ls047k1sx01_write_pwroff_config(dssdev);
	if (r)
		PANEL_ERR("panel pwroff fail\n");

	gpio_direction_output(dssdev->reset_gpio, 0);

	mdelay(20);

	omapdss_dsi_display_disable(dssdev, false, false);

	mdelay(300); // prevent panel flicking

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

#if 0
static void ls047k1sx01_stop(struct omap_dss_device *dssdev)
{
	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	ls047k1sx01_power_off(dssdev);

	dsi_bus_unlock(dssdev);
}
#endif

static void ls047k1sx01_disable(struct omap_dss_device *dssdev)
{
	PANEL_DBG("%s\n", __func__);
	mutex_lock(&d2d->lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("ls047k1sx01_disable: disable\n");

		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		dsi_bus_lock(dssdev);

		ls047k1sx01_power_off(dssdev);

		dsi_bus_unlock(dssdev);
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		PANEL_DBG("ls047k1sx01_disable: suspended\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	}
	else if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		PANEL_DBG("ls047k1sx01_disable: already disabled\n");
	}	
	
	mutex_unlock(&d2d->lock);

}

static int ls047k1sx01_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&d2d->lock);
	
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		PANEL_DBG("ls047k1sx01_enable: already enabled\n");
		mutex_unlock(&d2d->lock);
		return 0;
	}
	else {
		PANEL_DBG("ls047k1sx01_enable: enable\n");

		dsi_bus_lock(dssdev);

		r = ls047k1sx01_power_on(dssdev);

		dsi_bus_unlock(dssdev);

		if (r) {
			PANEL_ERR("ls047k1sx01_enable: enable failed\n");
			dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
		} else {
			dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		}

		mutex_unlock(&d2d->lock);

		return r;
	}
}



static struct omap_dss_driver ls047k1sx01_driver = {
	.driver_name	= PANEL_NAME,
	.probe		= ls047k1sx01_probe,

	.enable		= ls047k1sx01_enable,
	.disable	= ls047k1sx01_disable,


	.get_resolution	= omapdss_default_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= ls047k1sx01_get_timings,
	.set_timings	= ls047k1sx01_set_timings,
	.check_timings	= ls047k1sx01_check_timings,

};

int __init panel_ls047k1sx01_init(void)
{
	PANEL_DBG("%s\n", __func__);
	return omap_dss_register_driver(&ls047k1sx01_driver);

}

