/*
 * linux/drivers/video/omap2/dss/dpi.c
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

#define DSS_SUBSYS_NAME "DPI"

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/arch/clocks.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>

#include <icom/omapdss.h>

#include "dss.h"

/*----------------------------------------------------------------------*/


#ifdef CONFIG_OMAP2_DSS_DSI
static bool dpi_use_dsi_pll(struct omap_dss_device *dssdev)
{
	if (dssdev->clocks.dispc.dispc_fclk_src ==
			OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC ||
			dssdev->clocks.dispc.dispc_fclk_src ==
			OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC ||
			dssdev->clocks.dispc.channel.lcd_clk_src ==
			OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC ||
			dssdev->clocks.dispc.channel.lcd_clk_src ==
			OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC)
		return true;
	else
		return false;
}

static int dpi_set_dsi_clk(struct omap_dss_device *dssdev, bool is_tft,
		unsigned long pck_req, unsigned long *fck, int *lck_div,
		int *pck_div)
{
	struct dsi_clock_info dsi_cinfo;
	struct dispc_clock_info dispc_cinfo;
	int r;

	dispc_cinfo.min_fck_per_pck = dssdev->min_fck_per_pck;

	r = dsi_pll_calc_clock_div_pck(dpi.dsidev, is_tft, pck_req,
			&dsi_cinfo, &dispc_cinfo);
	if (r)
		return r;

	r = dsi_pll_set_clock_div(dpi.dsidev, &dsi_cinfo);
	if (r)
		return r;

	dss_select_dispc_clk_source(dssdev->clocks.dispc.dispc_fclk_src);

	r = dispc_set_clock_div(dssdev->manager->id, &dispc_cinfo);
	if (r)
		return r;

	*fck = dsi_cinfo.dsi_pll_hsdiv_dispc_clk;
	*lck_div = dispc_cinfo.lck_div;
	*pck_div = dispc_cinfo.pck_div;

	return 0;
}
#endif /* CONFIG_OMAP2_DSS_DSI */

static int dpi_set_dispc_clk(struct omap_dss_device *dssdev, bool is_tft,
		unsigned long pck_req, unsigned long *fck, int *lck_div,
		int *pck_div)
{
	struct dss_clock_info dss_cinfo;
	struct dispc_clock_info dispc_cinfo;
	int r;

	dispc_cinfo.min_fck_per_pck = dssdev->min_fck_per_pck;

	r = dss_calc_clock_div(is_tft, pck_req, &dss_cinfo, &dispc_cinfo);
	if (r)
		return r;

	r = dss_set_clock_div(&dss_cinfo);
	if (r)
		return r;

	r = dispc_set_clock_div(dssdev->manager->id, &dispc_cinfo);
	if (r)
		return r;

	*fck = dss_cinfo.fck;
	*lck_div = dispc_cinfo.lck_div;
	*pck_div = dispc_cinfo.pck_div;

	return 0;
}

/*----------------------------------------------------------------------*/

static int dpi_set_mode(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *t = &dssdev->panel.timings;
	int lck_div = 0, pck_div = 0;
	unsigned long fck = 0;
	unsigned long pck;
	bool is_tft;
	int r = 0;

	dispc_set_pol_freq(dssdev->manager->id, dssdev->panel.config,
			dssdev->panel.acbi, dssdev->panel.acb);

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

#ifdef CONFIG_OMAP2_DSS_DSI
	if (dpi_use_dsi_pll(dssdev))
		r = dpi_set_dsi_clk(dssdev, is_tft, t->pixel_clock * 1000,
				&fck, &lck_div, &pck_div);
	else
#endif /* CONFIG_OMAP2_DSS_DSI */
		r = dpi_set_dispc_clk(dssdev, is_tft, t->pixel_clock * 1000,
				&fck, &lck_div, &pck_div);
	if (r)
		return r;

	pck = fck / lck_div / pck_div / 1000;

	if (pck != t->pixel_clock) {
		DSSWARN("Could not find exact pixel clock. "
				"Requested %d kHz, got %lu kHz\n",
				t->pixel_clock, pck);

		t->pixel_clock = pck;
	}

	dispc_set_lcd_timings(dssdev->manager->id, t);

	return 0;
}

static void dpi_basic_init(struct omap_dss_device *dssdev)
{
	bool is_tft;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

	dispc_set_parallel_interface_mode(dssdev->manager->id,
			OMAP_DSS_PARALLELMODE_BYPASS);
	dispc_set_lcd_display_type(dssdev->manager->id, is_tft ?
			OMAP_DSS_LCD_DISPLAY_TFT : OMAP_DSS_LCD_DISPLAY_STN);
	dispc_set_tft_data_lines(dssdev->manager->id,
			dssdev->phy.dpi.data_lines);

	if (dssdev->panel.config & OMAP_DSS_LCD_GATE_HVSYNC)
		dispc_gate_hsync_vsync(dssdev->manager->id);

	/* Enable OMAP DSS dither mode */
	if (dssdev->phy.dpi.data_lines != 24)
		dispc_set_dither_mode(dssdev->manager->id, OMAP_DSS_DITHER_SPTP_4);
}

/*----------------------------------------------------------------------*/

int omapdss_dpi_display_enable(struct omap_dss_device *dssdev)
{
	int r;
#if defined(CONFIG_OMAP34XX)
	struct omap_dss_board_info *board_info = omap_dss_get_board_info();
#endif

#if 0
	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err_start_dev;
	}
#endif

#if 0
	if (cpu_is_omap34xx()) {
		r = regulator_enable(dpi.vdds_dsi_reg);
		if (r)
			goto err_reg_enable;
	}
#else
#if defined(CONFIG_OMAP34XX)
	if (board_info->dss_platform_enable) {
		r = board_info->dss_platform_enable();
		if (r)
			goto err_reg_enable;
	}
#endif
#endif

	r = dss_runtime_get();
	if (r)
		goto err_get_dss;

	r = dispc_runtime_get();
	if (r)
		goto err_get_dispc;

	dpi_basic_init(dssdev);

#ifdef CONFIG_OMAP2_DSS_DSI
	if (dpi_use_dsi_pll(dssdev)) {
		r = dsi_runtime_get(dpi.dsidev);
		if (r)
			goto err_get_dsi;

		r = dsi_pll_init(dpi.dsidev, 0, 1);
		if (r)
			goto err_dsi_pll_init;
	}
#else
#endif /* CONFIG_OMAP2_DSS_DSI */

	r = dpi_set_mode(dssdev);
	if (r)
		goto err_set_mode;

	mdelay(2);

	dssdev->manager->enable(dssdev->manager);

	return 0;

err_set_mode:
#ifdef CONFIG_OMAP2_DSS_DSI
	if (dpi_use_dsi_pll(dssdev))
		dsi_pll_uninit(dpi.dsidev, true);
err_dsi_pll_init:
	if (dpi_use_dsi_pll(dssdev))
		dsi_runtime_put(dpi.dsidev);
err_get_dsi:
#endif /* CONFIG_OMAP2_DSS_DSI */
	dispc_runtime_put();
err_get_dispc:
	dss_runtime_put();
err_get_dss:
#if 0
	if (cpu_is_omap34xx())
		regulator_disable(dpi.vdds_dsi_reg);
#else
#if defined(CONFIG_OMAP34XX)
	if (board_info->dss_platform_disable)
		board_info->dss_platform_disable();
err_reg_enable:
#endif
#endif
#if 0
	omap_dss_stop_device(dssdev);
err_start_dev:
#endif
	return r;
}
EXPORT_SYMBOL(omapdss_dpi_display_enable);

void omapdss_dpi_display_disable(struct omap_dss_device *dssdev)
{
#if defined(CONFIG_OMAP34XX)
	struct omap_dss_board_info *board_info = omap_dss_get_board_info();
#endif

	dssdev->manager->disable(dssdev->manager);

#ifdef CONFIG_OMAP2_DSS_DSI
	if (dpi_use_dsi_pll(dssdev)) {
		dss_select_dispc_clk_source(OMAP_DSS_CLK_SRC_FCK);
		dsi_pll_uninit(dpi.dsidev, true);
		dsi_runtime_put(dpi.dsidev);
	}
#endif /* CONFIG_OMAP2_DSS_DSI */

	dispc_runtime_put();
	dss_runtime_put();

#if 0
	if (cpu_is_omap34xx())
		regulator_disable(dpi.vdds_dsi_reg);
#else
#if defined(CONFIG_OMAP34XX)
	if (board_info->dss_platform_disable)
		board_info->dss_platform_disable(board_info);
#endif
#endif

#if 0
	omap_dss_stop_device(dssdev);
#endif
}
EXPORT_SYMBOL(omapdss_dpi_display_disable);

void dpi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	int r;

	DSSVDBG("dpi_set_timings\n");
	dssdev->panel.timings = *timings;
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		r = dss_runtime_get();
		if (r)
			return;

		r = dispc_runtime_get();
		if (r) {
			dss_runtime_put();
			return;
		}

		dpi_set_mode(dssdev);
		dispc_go(dssdev->manager->id);

		dispc_runtime_put();
		dss_runtime_put();
	}
}
EXPORT_SYMBOL(dpi_set_timings);

int dpi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	bool is_tft;
	int r;
	int lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;
	struct dispc_clock_info dispc_cinfo;

	if (!dispc_lcd_timings_ok(timings))
		return -EINVAL;

	if (timings->pixel_clock == 0)
		return -EINVAL;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

	dispc_cinfo.min_fck_per_pck = dssdev->min_fck_per_pck;

#ifdef CONFIG_OMAP2_DSS_DSI
	if (dpi_use_dsi_pll(dssdev)) {
		struct dsi_clock_info dsi_cinfo;
		r = dsi_pll_calc_clock_div_pck(dpi.dsidev, is_tft,
				timings->pixel_clock * 1000,
				&dsi_cinfo, &dispc_cinfo);

		if (r)
			return r;

		fck = dsi_cinfo.dsi_pll_hsdiv_dispc_clk;
	} else {
#else
	{
#endif /* CONFIG_OMAP2_DSS_DSI */
		struct dss_clock_info dss_cinfo;
		r = dss_calc_clock_div(is_tft, timings->pixel_clock * 1000,
				&dss_cinfo, &dispc_cinfo);

		if (r)
			return r;

		fck = dss_cinfo.fck;
	}

	lck_div = dispc_cinfo.lck_div;
	pck_div = dispc_cinfo.pck_div;

	pck = fck / lck_div / pck_div / 1000;

	timings->pixel_clock = pck;

	return 0;
}
EXPORT_SYMBOL(dpi_check_timings);

int dpi_init_display(struct omap_dss_device *dssdev)
{
	DSSVDBG("init_display\n");

#if 0
	if (cpu_is_omap34xx() && dpi.vdds_dsi_reg == NULL) {
		struct regulator *vdds_dsi;

		vdds_dsi = dss_get_vdds_dsi();

		if (IS_ERR(vdds_dsi)) {
			DSSERR("can't get VDDS_DSI regulator\n");
			return PTR_ERR(vdds_dsi);
		}

		dpi.vdds_dsi_reg = vdds_dsi;
	}
#endif

#if 0
	if (dpi_use_dsi_pll(dssdev)) {
		enum omap_dss_clk_source dispc_fclk_src =
			dssdev->clocks.dispc.dispc_fclk_src;
		dpi.dsidev = dpi_get_dsidev(dispc_fclk_src);
	}
#endif

	return 0;
}

int dpi_init(void)
{
	return 0;
}

void dpi_exit(void)
{
}

