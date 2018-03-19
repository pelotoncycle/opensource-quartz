/*
 * Toshiba TC358765 DSI-to-LVDS chip driver on OMAP DSS platform
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 * Based on original version from Jerry Alexander <x0135174@ti.com>
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
#include <i2c.h>

#include <icom/omapdss.h>
#include <icom/omap-panel-tc358765.h>

#include "panel-tc358765.h"

#define	PANEL_NAME	"tc358765"

/*----------------------------------------------------------------------*/

#ifdef CONFIG_TC358765_DEBUG
#define PANEL_DEBUG
/*#define PANEL_VERBOSE_DEBUG*/
#endif /* CONFIG_TC358765_DEBUG */

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

#define A_RO 0x1
#define A_WO 0x2
#define A_RW (A_RO|A_WO)

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

/*----------------------------------------------------------------------*/

/* device private data structure */
struct tc358765_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
} paneldata;

static struct tc358765_data *d2d = NULL;

/*----------------------------------------------------------------------*/

#ifdef CONFIG_TC358765_DEBUG

#if 0
struct {
	struct device *dev;
	struct dentry *dir;
} tc358765_debug;
#endif

struct tc358765_reg {
	const char *name;
	u16 reg;
	u8 perm:2;
} tc358765_regs[] = {
	/* DSI D-PHY Layer Registers */
	{ "D0W_DPHYCONTTX", D0W_DPHYCONTTX, A_RW },
	{ "CLW_DPHYCONTRX", CLW_DPHYCONTRX, A_RW },
	{ "D0W_DPHYCONTRX", D0W_DPHYCONTRX, A_RW },
	{ "D1W_DPHYCONTRX", D1W_DPHYCONTRX, A_RW },
	{ "D2W_DPHYCONTRX", D2W_DPHYCONTRX, A_RW },
	{ "D3W_DPHYCONTRX", D3W_DPHYCONTRX, A_RW },
	{ "COM_DPHYCONTRX", COM_DPHYCONTRX, A_RW },
	{ "CLW_CNTRL", CLW_CNTRL, A_RW },
	{ "D0W_CNTRL", D0W_CNTRL, A_RW },
	{ "D1W_CNTRL", D1W_CNTRL, A_RW },
	{ "D2W_CNTRL", D2W_CNTRL, A_RW },
	{ "D3W_CNTRL", D3W_CNTRL, A_RW },
	{ "DFTMODE_CNTRL", DFTMODE_CNTRL, A_RW },
	/* DSI PPI Layer Registers */
	{ "PPI_STARTPPI", PPI_STARTPPI, A_RW },
	{ "PPI_BUSYPPI", PPI_BUSYPPI, A_RO },
	{ "PPI_LINEINITCNT", PPI_LINEINITCNT, A_RW },
	{ "PPI_LPTXTIMECNT", PPI_LPTXTIMECNT, A_RW },
	{ "PPI_LANEENABLE", PPI_LANEENABLE, A_RW },
	{ "PPI_TX_RX_TA", PPI_TX_RX_TA, A_RW },
	{ "PPI_CLS_ATMR", PPI_CLS_ATMR, A_RW },
	{ "PPI_D0S_ATMR", PPI_D0S_ATMR, A_RW },
	{ "PPI_D1S_ATMR", PPI_D1S_ATMR, A_RW },
	{ "PPI_D2S_ATMR", PPI_D2S_ATMR, A_RW },
	{ "PPI_D3S_ATMR", PPI_D3S_ATMR, A_RW },
	{ "PPI_D0S_CLRSIPOCOUNT", PPI_D0S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D1S_CLRSIPOCOUNT", PPI_D1S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D2S_CLRSIPOCOUNT", PPI_D2S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D3S_CLRSIPOCOUNT", PPI_D3S_CLRSIPOCOUNT, A_RW },
	{ "CLS_PRE", CLS_PRE, A_RW },
	{ "D0S_PRE", D0S_PRE, A_RW },
	{ "D1S_PRE", D1S_PRE, A_RW },
	{ "D2S_PRE", D2S_PRE, A_RW },
	{ "D3S_PRE", D3S_PRE, A_RW },
	{ "CLS_PREP", CLS_PREP, A_RW },
	{ "D0S_PREP", D0S_PREP, A_RW },
	{ "D1S_PREP", D1S_PREP, A_RW },
	{ "D2S_PREP", D2S_PREP, A_RW },
	{ "D3S_PREP", D3S_PREP, A_RW },
	{ "CLS_ZERO", CLS_ZERO, A_RW },
	{ "D0S_ZERO", D0S_ZERO, A_RW },
	{ "D1S_ZERO", D1S_ZERO, A_RW },
	{ "D2S_ZERO", D2S_ZERO, A_RW  },
	{ "D3S_ZERO", D3S_ZERO, A_RW },
	{ "PPI_CLRFLG", PPI_CLRFLG, A_RW },
	{ "PPI_CLRSIPO", PPI_CLRSIPO, A_RW },
	{ "PPI_HSTimeout", PPI_HSTimeout, A_RW },
	{ "PPI_HSTimeoutEnable", PPI_HSTimeoutEnable, A_RW },
	/* DSI Protocol Layer Registers */
	{ "DSI_STARTDSI", DSI_STARTDSI, A_WO },
	{ "DSI_BUSYDSI", DSI_BUSYDSI, A_RO },
	{ "DSI_LANEENABLE", DSI_LANEENABLE, A_RW },
	{ "DSI_LANESTATUS0", DSI_LANESTATUS0, A_RO },
	{ "DSI_LANESTATUS1", DSI_LANESTATUS1, A_RO },
	{ "DSI_INTSTATUS", DSI_INTSTATUS, A_RO },
	{ "DSI_INTMASK", DSI_INTMASK, A_RW },
	{ "DSI_INTCLR", DSI_INTCLR, A_WO },
	{ "DSI_LPTXTO", DSI_LPTXTO, A_RW },
	/* DSI General Registers */
	{ "DSIERRCNT", DSIERRCNT, A_RW },
	/* DSI Application Layer Registers */
	{ "APLCTRL", APLCTRL, A_RW },
	{ "RDPKTLN", RDPKTLN, A_RW },
	/* Video Path Registers */
	{ "VPCTRL", VPCTRL, A_RW },
	{ "HTIM1", HTIM1, A_RW },
	{ "HTIM2",  HTIM2, A_RW },
	{ "VTIM1", VTIM1, A_RW },
	{ "VTIM2", VTIM2, A_RW },
	{ "VFUEN", VFUEN, A_RW },
	/* LVDS Registers */
	{ "LVMX0003", LVMX0003, A_RW },
	{ "LVMX0407", LVMX0407, A_RW },
	{ "LVMX0811", LVMX0811, A_RW },
	{ "LVMX1215", LVMX1215, A_RW },
	{ "LVMX1619", LVMX1619, A_RW },
	{ "LVMX2023", LVMX2023, A_RW },
	{ "LVMX2427", LVMX2427, A_RW },
	{ "LVCFG", LVCFG, A_RW },
	{ "LVPHY0", LVPHY0, A_RW },
	{ "LVPHY1", LVPHY1, A_RW },
	/* System Registers */
	{ "SYSSTAT", SYSSTAT, A_RO },
	{ "SYSRST", SYSRST, A_WO },
	/* GPIO Registers */
	{ "GPIOC", GPIOC, A_RW },
	{ "GPIOO", GPIOO, A_RW },
	{ "GPIOI", GPIOI, A_RO },
	/* I2C Registers */
	{ "I2CTIMCTRL", I2CTIMCTRL, A_RW },
	{ "I2CMADDR", I2CMADDR, A_RW },
	{ "WDATAQ", WDATAQ, A_WO },
	{ "RDATAQ", RDATAQ, A_WO },
	/* Chip/Rev Registers */
	{ "IDREG", IDREG, A_RO },
	/* Debug Registers */
	{ "DEBUG00", DEBUG00, A_RW },
	{ "DEBUG01", DEBUG01, A_RW },
};
#endif

/*----------------------------------------------------------------------*/

static struct tc358765_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct tc358765_board_data *)dssdev->data;
}

/*----------------------------------------------------------------------*/

#define TC358765_I2C_ADDR	0x0F

static int tc358765_has_i2c = 1;

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void tc358765_save_i2c_bus(void) __attribute__((always_inline));
static inline void tc358765_save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_TC358765_I2C_BUS)
		i2c_set_bus_num(CONFIG_TC358765_I2C_BUS);
}

static inline void tc358765_restore_i2c_bus(void) __attribute__((always_inline));
static inline void tc358765_restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_TC358765_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define tc358765_save_i2c_bus()
#define tc358765_restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static int tc358765_read_block(u16 reg, u8 *data, int len)
{
#if 0
	unsigned char wb[2];
	struct i2c_msg msg[2];
	int r;
	mutex_lock(&sd1->xfer_lock);
	wb[0] = (reg & 0xff00) >> 8;
	wb[1] = reg & 0xff;
	msg[0].addr = sd1->client->addr;
	msg[0].len = 2;
	msg[0].flags = 0;
	msg[0].buf = wb;
	msg[1].addr = sd1->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	r = i2c_transfer(sd1->client->adapter, msg, 2);
	mutex_unlock(&sd1->xfer_lock);

	if (r == 2)
		return len;

	return r;
#else
	int ret = 0;
	int retries = 2;

	tc358765_save_i2c_bus();

_retry:
	ret = i2c_read(TC358765_I2C_ADDR, reg, 2, data, len);
	if (ret) {
		if (--retries > 0) {
			mdelay(10);
			goto _retry;
		} else {
			PANEL_ERR("i2c read (0x%04X/%d) err %d\n", reg, len, ret);
			if (ret > 0)
				ret = -EIO;
		}
	} else {
		ret = len;
	}

	tc358765_restore_i2c_bus();

	return ret;
#endif
}

static int tc358765_i2c_read(u16 reg, u32 *val)
{
	int r;
	u8 data[4];
	data[0] = data[1] = data[2] = data[3] = 0;

	r = tc358765_read_block(reg, data, 4);
	if (r != 4)
		return -EIO;

	*val = ((int)data[3] << 24) | ((int)(data[2]) << 16) |
	    ((int)(data[1]) << 8) | ((int)(data[0]));
	return 0;
}

static int tc358765_dsi_read(struct omap_dss_device *dssdev, u16 reg, u32 *val)
{
#if 0
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
#endif
	u8 buf[4];
	int r;

	r = dsi_vc_gen_read_2(dssdev, d2d->channel1, reg, buf, 4);
	if (r < 0) {
		PANEL_ERR("0x%x read failed with status %d\n", reg, r);
		return r;
	}

	*val = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	return 0;
}

static int tc358765_read_register(struct omap_dss_device *dssdev,
					u16 reg, u32 *val)
{
	int ret = 0;
#if 0
	pm_runtime_get_sync(&dssdev->dev);
#endif
	/* I2C is preferred way of reading, but fall back to DSI
	 * if I2C didn't got initialized
	*/
	if (tc358765_has_i2c)
		ret = tc358765_i2c_read(reg, val);
	else
		ret = tc358765_dsi_read(dssdev, reg, val);
#if 0
	pm_runtime_put_sync(&dssdev->dev);
#endif
	return ret;
}

static int tc358765_write_register(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
#if 0
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
#endif
	u8 buf[6];
	int r;

	buf[0] = (reg >> 0) & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = (value >> 0) & 0xff;
	buf[3] = (value >> 8) & 0xff;
	buf[4] = (value >> 16) & 0xff;
	buf[5] = (value >> 24) & 0xff;

	r = dsi_vc_gen_write_nosync(dssdev, d2d->channel1, buf, 6);
	if (r)
		PANEL_ERR("reg write reg(%x) val(%x) failed: %d\n", reg, value, r);

	return r;
}

/*----------------------------------------------------------------------*/

static int tc358765_hw_reset(struct omap_dss_device *dssdev)
{
	if (dssdev == NULL || dssdev->reset_gpio == -1)
		return 0;

	gpio_set_value(dssdev->reset_gpio, 1);
	udelay(200);
	/* reset the panel */
	gpio_set_value(dssdev->reset_gpio, 0);
	/* assert reset */
	udelay(200);
	gpio_set_value(dssdev->reset_gpio, 1);

	/* wait after releasing reset */
	msleep(200);

	return 0;
}

/*----------------------------------------------------------------------*/

static int tc358765_init_ppi(struct omap_dss_device *dssdev)
{
	u32 go_cnt, sure_cnt, val = 0;
	u8 lanes = 0;
	int ret = 0;
	struct tc358765_board_data *board_data = get_board_data(dssdev);

	/* this register setting is required only if host wishes to
	 * perform DSI read transactions
	 */
	go_cnt = (board_data->lp_time * 5 - 3) / 4;
	sure_cnt = DIV_ROUND_UP(board_data->lp_time * 3, 2);
	val = FLD_MOD(val, go_cnt, 26, 16);
	val = FLD_MOD(val, sure_cnt, 10, 0);
	ret |= tc358765_write_register(dssdev, PPI_TX_RX_TA, val);

	/* SYSLPTX Timing Generation Counter */
	ret |= tc358765_write_register(dssdev, PPI_LPTXTIMECNT,
					board_data->lp_time);

	/* D*S_CLRSIPOCOUNT = [(THS-SETTLE + THS-ZERO) /
					HS_byte_clock_period ] */

	if (dssdev->phy.dsi.clk_lane)
		lanes |= (1 << 0);

	if (dssdev->phy.dsi.data1_lane) {
		lanes |= (1 << 1);
		ret |= tc358765_write_register(dssdev, PPI_D0S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data2_lane) {
		lanes |= (1 << 2);
		ret |= tc358765_write_register(dssdev, PPI_D1S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data3_lane) {
		lanes |= (1 << 3);
		ret |= tc358765_write_register(dssdev, PPI_D2S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data4_lane) {
		lanes |= (1 << 4);
		ret |= tc358765_write_register(dssdev, PPI_D3S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}

	ret |= tc358765_write_register(dssdev, PPI_LANEENABLE, lanes);
	ret |= tc358765_write_register(dssdev, DSI_LANEENABLE, lanes);

	return ret;
}

static int tc358765_init_video_timings(struct omap_dss_device *dssdev)
{
	u32 val;
	struct tc358765_board_data *board_data = get_board_data(dssdev);
	int ret;
	ret = tc358765_read_register(dssdev, VPCTRL, &val);
	if (ret < 0) {
		PANEL_WARN("couldn't access VPCTRL, going on with reset value\n");
		val = 0;
	}

	if (dssdev->ctrl.pixel_size == 18) {
		/* Magic Square FRC available for RGB666 only */
		val = FLD_MOD(val, board_data->msf, 0, 0);
		val = FLD_MOD(val, 0, 8, 8);
	} else {
		val = FLD_MOD(val, 1, 8, 8);
	}

	val = FLD_MOD(val, board_data->vtgen, 4, 4);
	val = FLD_MOD(val, board_data->evtmode, 5, 5);
	val = FLD_MOD(val, board_data->vsdelay, 31, 20);

	ret = tc358765_write_register(dssdev, VPCTRL, val);

	ret |= tc358765_write_register(dssdev, HTIM1,
		(dssdev->panel.timings.hbp << 16) | dssdev->panel.timings.hsw);
	ret |= tc358765_write_register(dssdev, HTIM2,
		((dssdev->panel.timings.hfp << 16) | dssdev->panel.timings.x_res));
	ret |= tc358765_write_register(dssdev, VTIM1,
		((dssdev->panel.timings.vbp << 16) | dssdev->panel.timings.vsw));
	ret |= tc358765_write_register(dssdev, VTIM2,
		((dssdev->panel.timings.vfp << 16) | dssdev->panel.timings.y_res));
	return ret;
}

static int tc358765_write_init_config(struct omap_dss_device *dssdev)
{
	struct tc358765_board_data *board_data = get_board_data(dssdev);
	u32 val;
	int r;

	/* HACK: dummy read: if we read via DSI, first reads always fail */
	tc358765_read_register(dssdev, DSI_INTSTATUS, &val);

	r = tc358765_init_ppi(dssdev);
	if (r) {
		PANEL_ERR("failed to initialize PPI layer\n");
		return r;
	}

	r = tc358765_write_register(dssdev, PPI_STARTPPI, 0x1);
	if (r) {
		PANEL_ERR("failed to start PPI-TX\n");
		return r;
	}

	r = tc358765_write_register(dssdev, DSI_STARTDSI, 0x1);
	if (r) {
		PANEL_ERR("failed to start DSI-RX\n");
		return r;
	}

	/* reset LVDS-PHY */
	tc358765_write_register(dssdev, LVPHY0, (1 << 22));
	mdelay(2);

	r = tc358765_read_register(dssdev, LVPHY0, &val);
	if (r < 0) {
		PANEL_WARN("couldn't access LVPHY0, going on with reset value\n");
		val = 0;
	}
	val = FLD_MOD(val, 0, 22, 22);
	val = FLD_MOD(val, board_data->lv_is, 15, 14);
	val = FLD_MOD(val, board_data->lv_nd, 4, 0);
	r = tc358765_write_register(dssdev, LVPHY0, val);
	if (r) {
		PANEL_ERR("failed to initialize LVDS-PHY\n");
		return r;
	}
	r = tc358765_init_video_timings(dssdev);
	if (r) {
		PANEL_ERR("failed to initialize video path layer\n");
		return r;
	}

	r = tc358765_read_register(dssdev, LVCFG, &val);
	if (r < 0) {
		PANEL_WARN("couldn't access LVCFG, going on with reset value\n");
		val = 0;
	}

	val = FLD_MOD(val, board_data->pclkdiv, 9, 8);
	val = FLD_MOD(val, board_data->pclksel, 11, 10);
	val = FLD_MOD(val, board_data->lvdlink, 1, 1);
	/* enable LVDS transmitter */
	val = FLD_MOD(val, 1, 0, 0);
	r = tc358765_write_register(dssdev, LVCFG, val);
	if (r) {
		PANEL_ERR("failed to start LVDS transmitter\n");
		return r;
	}

	if (board_data->board_init) {
		r = board_data->board_init(dssdev,
				tc358765_read_register, tc358765_write_register);
		if (r) {
			PANEL_ERR("%pf returns err %d\n", board_data->board_init, r);
			return r;
		}
	}

	/* Issue a soft reset to LCD Controller for a clean start */
	r = tc358765_write_register(dssdev, SYSRST, (1 << 2));
	/* commit video configuration */
	r |= tc358765_write_register(dssdev, VFUEN, 0x1);
	if (r)
		PANEL_ERR("failed to latch video timings\n");
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

	/* reset tc358765 bridge */
	tc358765_hw_reset(dssdev);

	/* do extra job to match kozio registers (???) */
	dsi_videomode_panel_preinit(dssdev);

	/* Need to wait a certain time - Toshiba Bridge Constraint */
	/* msleep(400); */


	/* configure D2L chip DSI-RX configuration registers */
	r = tc358765_write_init_config(dssdev);
	if (r) {
		goto err_write_init;
	}

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
		PANEL_DBG("not expected pixel size: %d\n", dssdev->ctrl.pixel_size);
		break;
	}

	PANEL_VDBG("%s exit\n", __func__);

	return 0;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_dsi_enable:
	mdelay(50);
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	mdelay(50);
err0:
	PANEL_ERR("failed to enable (%d)\n", r);
	return r;
}

static void panel_power_off(struct omap_dss_device *dssdev)
{
	PANEL_VDBG("%s\n", __func__);

	dsi_video_mode_disable(dssdev);

	gpio_set_value(dssdev->reset_gpio, 0);

	omapdss_dsi_display_disable(dssdev, false, false);

	mdelay(200);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

#if 0
	gpio_set_value(dssdev->reset_gpio, 0);
#endif

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
			PANEL_ERR("%s: enable failed %d\n", __func__, r);
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

/****************************
********* DEBUG *************
****************************/
#ifdef CONFIG_TC358765_DEBUG
static int tc358765_write_register_i2c(u16 reg, u32 val)
{
#if 0
	int ret = -ENODEV;
	unsigned char buf[6];
	struct i2c_msg msg;

	if (!sd1) {
		dev_err(&sd1->client->dev, "%s: I2C not initilized\n",
							__func__);
		return ret;
	}

	buf[0] = (reg >> 8) & 0xff;
	buf[1] = (reg >> 0) & 0xff;
	buf[2] = (val >> 0) & 0xff;
	buf[3] = (val >> 8) & 0xff;
	buf[4] = (val >> 16) & 0xff;
	buf[5] = (val >> 24) & 0xff;
	msg.addr = sd1->client->addr;
	msg.len = sizeof(buf);
	msg.flags = 0;
	msg.buf = buf;

	mutex_lock(&sd1->xfer_lock);
	ret = i2c_transfer(sd1->client->adapter, &msg, 1);
	mutex_unlock(&sd1->xfer_lock);

	if (ret != 1)
		return ret;
	return 0;
#else
	int ret;
	int retries = 2;
	unsigned char buf[4];

	buf[0] = (val >> 0) & 0xff;
	buf[1] = (val >> 8) & 0xff;
	buf[2] = (val >> 16) & 0xff;
	buf[3] = (val >> 24) & 0xff;

	tc358765_save_i2c_bus();

_retry:
	ret = i2c_write(TC358765_I2C_ADDR, reg, 2, buf, 4);
	if (ret) {
		if (--retries > 0) {
			mdelay(10);
			goto _retry;
		} else {
			PANEL_ERR("i2c write (0x%04X/0x%08X) err %d\n", reg, val, ret);
		}
	}

	tc358765_restore_i2c_bus();

	return ret;
#endif
}

static int do_tc358765_register_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned i, reg_count;
	char *name;
	u32 value;
	int error;

	if (argc < 2)
		return CMD_RET_USAGE;

	name = argv[1];

	reg_count = sizeof(tc358765_regs) / sizeof(tc358765_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tc358765_regs[i].name)) {
			if (!(tc358765_regs[i].perm & A_WO)) {
				PANEL_ERR("%s is write-protected\n", name);
				return 1;
			}

			value = (u32)simple_strtoul(argv[2], NULL, 16);
			error = tc358765_write_register_i2c(tc358765_regs[i].reg, value);
			if (error) {
				PANEL_ERR("%s: failed to write %s\n", __func__, name);
				return 1;
			}
			return 0;
		}
	}

	PANEL_ERR("%s: CANNOT find register '%s'\n", __func__, name);

	return 0;
}

U_BOOT_CMD(
	tc358765w, 3, 0,	do_tc358765_register_write,
	"Write TC358765 register value",
	"reg_name value(hex)\n"
);

/* tc358765 */
static int do_tc358765_registers_show(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned i, reg_count;
	uint value;

	puts("----------------------------------------------\n");
	reg_count = sizeof(tc358765_regs) / sizeof(tc358765_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (tc358765_regs[i].perm & A_RO) {
			tc358765_i2c_read(tc358765_regs[i].reg, &value);
			printf("%-20s = 0x%02X\n", tc358765_regs[i].name, value);
		}
	}
	puts("----------------------------------------------\n");

	return 0;
}

U_BOOT_CMD(
	tc358765, 1, 0,	do_tc358765_registers_show,
	"Dump TC358765 register values",
	""
);
#endif /* CONFIG_TC358765_DEBUG */

int panel_tc358765_init(void)
{
	PANEL_VDBG("%s\n", __func__);
	return omap_dss_register_driver(&panel_driver);
}
