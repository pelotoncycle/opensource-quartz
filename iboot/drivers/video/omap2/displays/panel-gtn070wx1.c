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

#define	PANEL_NAME	"gtn070wx1"

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

static int gtn070wx1_write_init_config(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[48];

	/* assert reset */
	mdelay(1);
	gpio_direction_output(dssdev->reset_gpio, 1);
	mdelay(130);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set F0\n");
		return r;
	}

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set F1\n");
		return r;
	}

	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0x10;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set D0\n");
		return r;
	}

	buf[0] = 0xC3;
	buf[1] = 0x40;
	buf[2] = 0x00;
	buf[3] = 0x28;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		PANEL_ERR("failed to set C3\n");
		return r;
	}

	mdelay(50);

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 11\n");
		return r;
	}

	mdelay(120);

	buf[0] = 0x35;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 35\n");
		return r;
	}

	buf[0] = 0x29;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 29\n");
		return r;
	}

#if 0
	dsi_vc_dcs_read(dssdev, d2d->channel1, 0x0A, &err, 1);
	PANEL_DBG("RDDPM = 0x%x\n",err);
#endif

/*
	mdelay(50);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set F0\n");
		return r;
	}



	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set F1\n");
		return r;
	}

	buf[0] = 0xFC;
	buf[1] = 0xA5;
	buf[2] = 0xA5;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set FC\n");
		return r;
	}

	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0x10;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set D0\n");
		return r;
	}



	buf[0] = 0xB1;
	buf[1] = 0x10;
	//buf[2] = 0xA0;//SJ
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set B1\n");
		return r;
	}

	buf[0] = 0xB2;
	buf[1] = 0x14;
	buf[2] = 0x22;
	buf[3] = 0x2F;
	buf[4] = 0x04;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 5);
	if (r) {
		PANEL_ERR("failed to set B2\n");
		return r;
	}

	buf[0] = 0xF2;
	buf[1] = 0x02;
	buf[2] = 0x0A;
	buf[3] = 0x0A;
	buf[4] = 0x40;
	buf[5] = 0x10;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 6);
	if (r) {
		PANEL_ERR("failed to set F2\n");
		return r;
	}

//buf[0] = buf[1]=buf[2]=buf[3]=buf[4]=buf[5] =0x00;
//r = dsi_vc_dcs_read(dssdev, d2d->channel1, 0xF2, buf, 5);



	buf[0] = 0xB0;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set B0\n");
		return r;
	}


	buf[0] = 0xFD;
	buf[1] = 0x09;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 2);
	if (r) {
		PANEL_ERR("failed to set FD\n");
		return r;
	}

	buf[0] = 0xF3;
	buf[1] = 0x01;
	buf[2] = 0xD7;
	buf[3] = 0xE2;
	buf[4] = 0x62;
	buf[5] = 0xF4;
	buf[6] = 0xF7;
	buf[7] = 0x77;
	buf[8] = 0x3C;
	buf[9] = 0x26;
	buf[10] = 0x00;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 11);
	if (r) {
		PANEL_ERR("failed to set F3\n");
		return r;
	}

	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0x02;
	buf[3] = 0x03;
	buf[4] = 0x26;
	buf[5] = 0x03;
	buf[6] = 0x02;
	buf[7] = 0x09;
	buf[8] = 0x00;
	buf[9] = 0x07;
	buf[10] = 0x16;
	buf[11] = 0x16;
	buf[12] = 0x03;
	buf[13] = 0x00;
	buf[14] = 0x08;
	buf[15] = 0x08;
	buf[16] = 0x03;
	buf[17] = 0x0E;
	buf[18] = 0x0F;
	buf[19] = 0x12;
	buf[20] = 0x1C;
	buf[21] = 0x1D;
	buf[22] = 0x1E;
	buf[23] = 0x0C;
	buf[24] = 0x09;
	buf[25] = 0x01;
	buf[26] = 0x04;
	buf[27] = 0x02;
	buf[28] = 0x61;
	buf[29] = 0x74;
	buf[30] = 0x75;
	buf[31] = 0x72;
	buf[32] = 0x83;
	buf[33] = 0x80;
	buf[34] = 0x80;
	buf[35] = 0xB0;
	buf[36] = 0x00;
	buf[37] = 0x01;
	buf[38] = 0x01;
	buf[39] = 0x28;
	buf[40] = 0x04;
	buf[41] = 0x03;
	buf[42] = 0x28;
	buf[43] = 0x01;
	buf[44] = 0xD1;
	buf[45] = 0x32;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 46);
	if (r) {
		PANEL_ERR("failed to set F4\n");
		return r;
	}

	buf[0] = 0xF5;
	buf[1] = 0x91;
	buf[2] = 0x28;
	buf[3] = 0x28;
	buf[4] = 0x5F;
	buf[5] = 0xAB;
	buf[6] = 0x98;
	buf[7] = 0x52;
	buf[8] = 0x0F;
	buf[9] = 0x33;
	buf[10] = 0x43;
	buf[11] = 0x04;
	buf[12] = 0x59;
	buf[13] = 0x54;
	buf[14] = 0x52;
	buf[15] = 0x05;
	buf[16] = 0x40;
	buf[17] = 0x60;
	buf[18] = 0x4E;
	buf[19] = 0x60;
	buf[20] = 0x40;
	buf[21] = 0x27;
	buf[22] = 0x26;
	buf[23] = 0x52;
	buf[24] = 0x25;
	buf[25] = 0x6D;
	buf[26] = 0x18;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 27);
	if (r) {
		PANEL_ERR("failed to set F5\n");
		return r;
	}

	buf[0] = 0xEE;
	buf[1] = 0x25;
	buf[2] = 0x00;
	buf[3] = 0x25;
	buf[4] = 0x00;
	buf[5] = 0x25;
	buf[6] = 0x00;
	buf[7] = 0x25;
	buf[8] = 0x00;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 9);
	if (r) {
		PANEL_ERR("failed to set EE\n");
		return r;
	}

	buf[0] = 0xEF;
	buf[1] = 0x12;
	buf[2] = 0x12;
	buf[3] = 0x43;
	buf[4] = 0x43;
	buf[5] = 0xA0;
	buf[6] = 0x04;
	buf[7] = 0x24;
	buf[8] = 0x81;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 9);
	if (r) {
		PANEL_ERR("failed to set EF\n");
		return r;
	}

	buf[0] = 0xF7;
	buf[1] = 0x0A;
	buf[2] = 0x0A;
	buf[3] = 0x08;
	buf[4] = 0x08;
	buf[5] = 0x0B;
	buf[6] = 0x0B;
	buf[7] = 0x09;
	buf[8] = 0x09;
buf[9] = 0x04; 
buf[10] = 0x05;
buf[11] = 0x01;
buf[12] = 0x01;
buf[13] = 0x01;
buf[14] = 0x01;
buf[15] = 0x01;
buf[16] = 0x01;
buf[17] = 0x0A;
buf[18] = 0x0A;
buf[19] = 0x08;
buf[20] = 0x08; 
buf[21] = 0x0B;
buf[22] = 0x0B;
buf[23] = 0x09;
buf[24] = 0x09;
buf[25] = 0x04;
buf[26] = 0x05;
buf[27] = 0x01;
buf[28] = 0x01;
buf[29] = 0x01;
buf[30] = 0x01;
buf[31] = 0x01;
buf[32] = 0x01;


	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 33);
	if (r) {
		PANEL_ERR("failed to set F7\n");
		return r;
	}

	buf[0] = 0xBC;
	buf[1] = 0x01;
	buf[2] = 0x4E;
	buf[3] = 0x08;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		PANEL_ERR("failed to set BC\n");
		return r;
	}

	buf[0] = 0xE1;
	buf[1] = 0x03;
	buf[2] = 0x10;
	buf[3] = 0x1C;
	buf[4] = 0xA0;
	buf[5] = 0x10;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 6);
	if (r) {
		PANEL_ERR("failed to set E1\n");
		return r;
	}

	buf[0] = 0xF6;
	buf[1] = 0x60;
	buf[2] = 0x25;
	buf[3] = 0xA6;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 7);
	if (r) {
		PANEL_ERR("failed to set F6\n");
		return r;
	}

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x34;
	buf[3] = 0x06;
	buf[4] = 0x0D;
	buf[5] = 0x04;
	buf[6] = 0x0A;
buf[7] = 0x0F;
buf[8] = 0x0F;
buf[9] = 0x12;
buf[10] = 0x1B;
buf[11] = 0x1E;
buf[12] = 0x1D;
buf[13] = 0x1E;
buf[14] = 0x1D;
buf[15] = 0x1D;
buf[16] = 0x1D;
buf[17] = 0x25;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 18);
	if (r) {
		PANEL_ERR("failed to set FA\n");
		return r;
	}



	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x34;
	buf[3] = 0x06;
	buf[4] = 0x0D;
	buf[5] = 0x04;
	buf[6] = 0x0A;
buf[7] = 0x0F;
buf[8] = 0x0F;
buf[9] = 0x12;
buf[10] = 0x1B;
buf[11] = 0x1E;
buf[12] = 0x1D;
buf[13] = 0x1E;
buf[14] = 0x1D;
buf[15] = 0x1D;
buf[16] = 0x1D;
buf[17] = 0x25;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 18);
	if (r) {
		PANEL_ERR("failed to set FB\n");
		return r;
	}

	buf[0] = 0xFE;
	buf[1] = 0x00;
	buf[2] = 0x0D;
	buf[3] = 0x03;
	buf[4] = 0x21;
	buf[5] = 0x00;
	buf[6] = 0x08;

	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 7);
	if (r) {
		PANEL_ERR("failed to set FE\n");
		return r;
	}

	buf[0] = 0xC3;
	buf[1] = 0x40;
	buf[2] = 0x00;
	buf[3] = 0x28;


	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		PANEL_ERR("failed to set C3\n");
		return r;
	}


	buf[0] = 0x35;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 35\n");
		return r;
	}

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 11\n");
		return r;
	}

mdelay(5);




#if 1
	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 3);
	if (r) {
		PANEL_ERR("failed to set F0\n");
		return r;
	}

	buf[0] = 0x11;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 11\n");
		return r;
	}
msleep(10);

	buf[0] = 0xC3;
	buf[1] = 0x40;
	buf[2] = 0x00;
	buf[3] = 0x28;


	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 4);
	if (r) {
		PANEL_ERR("failed to set C3\n");
		return r;
	}


	buf[0] = 0x13;
r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 13\n");
		return r;
	}
#endif
mdelay(120);

	buf[0] = 0x29;
	r = dsi_vc_dcs_write_nosync(dssdev, d2d->channel1, buf, 1);
	if (r) {
		PANEL_ERR("failed to set 29\n");
		return r;
	}
*/
//mdelay(1000);
	//dsi_vc_dcs_read(dssdev, d2d->channel1, 0x0A, &err, 1);
	//PANEL_DBG("RDDPM = 0x%x\n",err);
        //dsi_vc_send_bta_sync(dssdev, d2d->channel1);
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

static int panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	PANEL_VDBG("%s\n", __func__);

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	gpio_direction_output(dssdev->reset_gpio, 0);

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

	r = gtn070wx1_write_init_config(dssdev);
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


static int gtn070wx1_write_pwroff_config(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[16];

	PANEL_VDBG("power-off cmds\n");

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

	msleep(100);

	PANEL_VDBG("power-off cmds done\n");

	return r;
}

static void panel_power_off(struct omap_dss_device *dssdev)
{
	int r;

	PANEL_VDBG("%s\n", __func__);

	dsi_video_mode_disable(dssdev);

	r = gtn070wx1_write_pwroff_config(dssdev);
	if (r)
		PANEL_ERR("panel pwroff fail\n");

	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, false);
	//omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, false);

	gpio_direction_output(dssdev->reset_gpio, 0);

	mdelay(20);

	omapdss_dsi_display_disable(dssdev, false, false);

	msleep(120);

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

int panel_gtn070wx1_init(void)
{
	PANEL_VDBG("%s\n", __func__);
	return omap_dss_register_driver(&panel_driver);
}
