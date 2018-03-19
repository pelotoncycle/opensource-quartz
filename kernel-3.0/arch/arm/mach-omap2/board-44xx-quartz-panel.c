/*
 * arch/arm/mach-omap2/board-44xx-quartz-panel.c
 *
 * Copyright (C) 2012 Texas Instruments
 * SinJhe Yu <sinjhe.yu@innocomm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <video/omapdss.h>
#include <video/omap-panel-tc358765.h>
#include <linux/regulator/machine.h>

#include <linux/i2c/twl.h>
#include <linux/wakelock.h>

#include <plat/android-display.h>
#include <plat/vram.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/videolfb.h>
#include <asm/mach-types.h>
#include "board-44xx-quartz.h"
#include "control.h"
#include "mux.h"

#define DP_4430_GPIO_59         59
#define LED_DISP_EN		102
#define DSI2_GPIO_59		59

#define HDMI_GPIO_CT_CP_HPD             60
#define HDMI_GPIO_HPD                   63  /* Hot plug pin for HDMI */

#define HDMI_GPIO_LS_OE 41 /* Level shifter for HDMI */
#define LCD_BL_GPIO		59 /*27*/	/* LCD Backlight GPIO */
/* PWM2 and TOGGLE3 register offsets */
#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define TWL6030_TOGGLE3		0x92
/* PWM 1 */
#define LED_PWM1ON		0x00
#define LED_PWM1OFF		0x01

#define OMAP_HDMI_HPD_ADDR      0x4A100098
#define OMAP_HDMI_PULLTYPE_MASK 0x00000010

/* Quartz */
#define LCD_BL_EN_N	61
#define LCD_RESET	43

static void innocomm_hdmi_mux_safemode(void)
{
	omap_mux_init_signal("hdmi_hpd.safe_mode",
				OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.safe_mode",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_ddc_scl.safe_mode",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_ddc_sda.safe_mode",
			OMAP_PIN_INPUT_PULLDOWN);
}

#ifdef CONFIG_OMAP4_DSS_HDMI
static struct omap_dss_device innocomm_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.panel = {
		.hdmi_default_cea_code = 34,
		.timings = {
			.x_res = 1920,
			.y_res = 1080,
			.pixel_clock = 74250,
			.hsw = 44,
			.hfp = 88,
			.hbp = 148,
			.vsw = 5,
			.vfp = 4,
			.vbp = 36,
		},
	},
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
			.max_pixclk_khz = 148500,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};
#endif /* CONFIG_OMAP4_DSS_HDMI */

static struct regulator *v5v = NULL;
static struct regulator *lcm_5v = NULL;
static struct regulator *v_lcm_3v3 = NULL;

static struct wake_lock delayed_pm_suspend;

#include <linux/pm_qos_params.h>
static struct pm_qos_request_list pm_qos_cpuidle;

static int innocomm_panel_platform_init(struct omap_dss_device *dssdev)
{
	static int initialized = 0;
	static int last_ret = 0;

	if (initialized)
		return last_ret;

	initialized = 1;

	wake_lock_init(&delayed_pm_suspend, WAKE_LOCK_SUSPEND, "delayed_pm_suspend");

	v5v = regulator_get(&dssdev->dev, "lcm_v5v");
	if (IS_ERR(v5v)) {
		dev_err(&dssdev->dev, "Panel V5V_EN regulator missing (%ld) ??\n", PTR_ERR(v5v));
		v5v = NULL;
		last_ret = -ENODEV;
		goto _exit;
	}

	lcm_5v = regulator_get(&dssdev->dev, "lcm_5V");
	if (IS_ERR(lcm_5v)) {
		dev_err(&dssdev->dev, "Panel LCM_5V_EN_N regulator missing (%ld) ??\n", PTR_ERR(lcm_5v));
		lcm_5v = NULL;
		last_ret = -ENODEV;
		goto _exit;
	}

	v_lcm_3v3 = regulator_get(&dssdev->dev, "v_lcm_3v3");
	if (IS_ERR(v_lcm_3v3)) {
		dev_err(&dssdev->dev, "Panel V_LCM_3V3 regulator missing (%ld) ??\n", PTR_ERR(v_lcm_3v3));
		v_lcm_3v3 = NULL;
		last_ret = -ENODEV;
		goto _exit;
	}

_exit:
	return last_ret;
}

static int innocomm_panel_enable(struct omap_dss_device *dssdev)
{
	int ret;

	ret = innocomm_panel_platform_init(dssdev);
	if (ret)
		return ret;

	pm_qos_add_request(&pm_qos_cpuidle,
			PM_QOS_CPU_DMA_LATENCY, 100);

	ret = regulator_enable(v5v);
	if (ret)
		dev_err(&dssdev->dev, "failed to turn on V5V_EN %d\n", ret);

	ret = regulator_enable(lcm_5v);
	if (ret)
		dev_err(&dssdev->dev, "failed to turn on LCM_5V_EN_N %d\n", ret);

	ret = regulator_enable(v_lcm_3v3);
	if (ret)
		dev_err(&dssdev->dev, "failed to turn on V_LCM_3V3 %d\n", ret);

	msleep(10);

	return 0;
}

static void innocomm_panel_disable(struct omap_dss_device *dssdev)
{
	int ret;

	if (lcm_5v) {
		ret = regulator_disable(lcm_5v);
		if (ret)
			dev_err(&dssdev->dev, "failed to turn off LCM_5V_EN_N %d\n", ret);
	}

	if (v5v) {
		ret = regulator_disable(v5v);
		if (ret)
			dev_err(&dssdev->dev, "failed to turn off V5V_EN %d\n", ret);
	}

	if (v_lcm_3v3) {
		ret = regulator_disable(v_lcm_3v3);
		if (ret)
			dev_err(&dssdev->dev, "failed to turn off V_LCM_3V3 %d\n", ret);
	}

	pm_qos_remove_request(&pm_qos_cpuidle);

	msleep(50); /* wait for the voltage off */

	/* Hold a 5-sec wakelock to avoid the immediate Linux PM suspend */
	wake_lock_timeout(&delayed_pm_suspend, 5 * HZ);
}

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
static struct twl_pwm_device *twl6032_pwm;

static int innocomm_panel_set_backlight(struct omap_dss_device *dssdev, int level)
{
#if defined(CONFIG_LEDS_TWL6032_PWM)
	if (twl6032_pwm)
		twl_pwm_set_brightness(twl6032_pwm, level);
	else {
		WARN_ONCE(1, "incorrect pwm driver\n");
	}
#else
#error "Please define CONFIG_LEDS_TWL6032_PWM"
#endif /* CONFIG_LEDS_TWL6032_PWM */

	return 0;
}
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

/* LVDS Registers */
#define LVMX0003		0x0480		/* LVDS Mux Input - Bit 0 to 3 */
#define LVMX0407		0x0484		/* LVDS Mux Input - Bit 4 to 7 */
#define LVMX0811		0x0488		/* LVDS Mux Input - Bit 8 to 11 */
#define LVMX1215		0x048c		/* LVDS Mux Input - Bit 12 to 15 */
#define LVMX1619		0x0490		/* LVDS Mux Input - Bit 16 to 19 */
#define LVMX2023		0x0494		/* LVDS Mux Input - Bit 20 to 23 */
#define LVMX2427		0x0498		/* LVDS Mux Input - Bit 24 to 27 */

static int tc358765_board_init(struct omap_dss_device *dssdev,
		int (*read)(struct omap_dss_device *dssdev, u16 reg, u32 *val),
		int (*write)(struct omap_dss_device *dssdev, u16 reg, u32 val))
{
	u32 val = 0;
	int r;

	/* Video Signal Mapping */
	val = 0;
	val = FLD_MOD(val, 0, 4, 0);	// in0: R0
	val = FLD_MOD(val, 1, 12, 8);	// in1: R1
	val = FLD_MOD(val, 2, 20, 16);	// in2: R2
	val = FLD_MOD(val, 3, 28, 24);	// in3: R3
	r = write(dssdev, LVMX0003, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX0003\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 4, 4, 0);	// in4: R4
	val = FLD_MOD(val, 7, 12, 8);	// in5: R7
	val = FLD_MOD(val, 5, 20, 16);	// in6: R5
	val = FLD_MOD(val, 8, 28, 24);	// in7: G0
	r = write(dssdev, LVMX0407, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX0407\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 9, 4, 0);	// in8: G1
	val = FLD_MOD(val, 10, 12, 8);	// in9: G2
	val = FLD_MOD(val, 14, 20, 16);	// in10: G6
	val = FLD_MOD(val, 15, 28, 24);	// in11: G7
	r = write(dssdev, LVMX0811, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX0811\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 11, 4, 0);	// in12: G3
	val = FLD_MOD(val, 12, 12, 8);	// in13: G4
	val = FLD_MOD(val, 13, 20, 16);	// in14: G5
	val = FLD_MOD(val, 16, 28, 24);	// in15: B0
	r = write(dssdev, LVMX1215, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX1215\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 22, 4, 0);	// in16: B6
	val = FLD_MOD(val, 23, 12, 8);	// in17: B7
	val = FLD_MOD(val, 17, 20, 16);	// in18: B1
	val = FLD_MOD(val, 18, 28, 24);	// in19: B2
	r = write(dssdev, LVMX1619, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX1619\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 19, 4, 0);	// in20: B3
	val = FLD_MOD(val, 20, 12, 8);	// in21: B4
	val = FLD_MOD(val, 21, 20, 16);	// in22: B5
	val = FLD_MOD(val, 27, 28, 24);	// in23: RSVD
	r = write(dssdev, LVMX2023, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX2023\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 24, 4, 0);	// in24: HSYNC
	val = FLD_MOD(val, 25, 12, 8);	// in25: VSYNC
	val = FLD_MOD(val, 26, 20, 16);	// in26: DE
	val = FLD_MOD(val, 6, 28, 24);	// in27: R6
	r = write(dssdev, LVMX2427, val);
	if (r) {
		dev_err(&dssdev->dev, "failed to LVMX2427\n");
		return r;
	}

	return r;
}

static struct tc358765_board_data dsi_data_tc358765 = {
	.lp_time	= 0x4,
	.clrsipo	= 0xc,
	.lv_is		= 0x2,
	.lv_nd		= 0x6,
	.vtgen		= 0x0,
	.vsdelay	= 0x0,
	.pclkdiv	= 0x0,
	.pclksel	= 0x1,
	.lvdlink	= 0x1,
	.msf		= 0x0,
	.evtmode	= 0x1,
	.board_init	= tc358765_board_init,
};

static struct omap_dsi_timings dsi_timings_tc358765 = {
	.hbp		= 74,
	.hfp		= 73,
	.hsa		= 0,
	.vbp		= 18,
	.vfp		= 18,
	.vsa		= 5,
	.vact		= 1080,
	.tl		= 1590,
	.hsa_hs_int	= 0,
	.hfp_hs_int	= 0,
	.hbp_hs_int	= 0,
	.hsa_lp_int	= 130,
	.hfp_lp_int	= 223,
	.hbp_lp_int	= 59,
	.bl_lp_int	= 0,
	.bl_hs_int	= 1482,
	.exit_lat	= 23,
	.enter_lat	= 27,
};

static struct omap_video_timings dispc_timings_tc358765 = {
	.x_res		= 1920,
	.y_res		= 1080,
	.hfp		= 62,
	.hsw		= 5,
	.hbp		= 133,
	.vfp		= 17,
	.vsw		= 6,
	.vbp		= 18,
};

static struct omap_dss_device innocomm_lcd_device = {
	.name                   = "lcd",
	.driver_name            = "tc358765",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi_data_tc358765,
	.phy.dsi                = {
		.clk_lane       = 3, //DSI1_DX2, DSI1_DY2
		.clk_pol        = 0,
		.data1_lane     = 1, //DSI1_DX0, DSI1_DY0
		.data1_pol      = 0,
		.data2_lane     = 2, //DSI1_DX1, DSI1_DY1
		.data2_pol      = 0,
		.data3_lane     = 4,
		.data3_pol      = 0,
		.data4_lane     = 5,
		.data4_pol      = 0,

		.type = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
		.line_bufs	= 0,
	},

	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 1,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
#if 0
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
#else
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
#endif
		},

		.dsi = {
			.regn           = 16,
			.regm           = 333,
			.regm_dispc     = 12,
			.regm_dsi       = 9,
			.lp_clk_div     = 13,
			.offset_ddr_clk = 0,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
			.tlpx	= 22,
			.tclk = {
				.zero	 = 106,
				.prepare = 26,
				.trail	 = 26,
			},
			.ths = {
				.zero	 = 42,
				.prepare = 30,
				.exit	 = 58,
				.trail	 = 45,
			},
		},
	},

	.panel = {
		.timings = {
			.x_res		= 1920,
			.y_res		= 1080,
			.pixel_clock	= 133200,
			.hfp		= 62,
			.hsw		= 5,
			.hbp		= 133,
			.vfp		= 17,
			.vsw		= 6,
			.vbp		= 18,
		},
		.width_in_um = 476640,
		.height_in_um = 268110,
	},

	.ctrl = {
		.pixel_size = 24,
	},

	.reset_gpio = LCD_RESET,
	.channel = OMAP_DSS_CHANNEL_LCD,
	.skip_init = false,

	.platform_enable = &innocomm_panel_enable,
	.platform_disable = &innocomm_panel_disable,

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
	.set_backlight = innocomm_panel_set_backlight,
	.get_backlight = NULL,
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */

	.dispc_timings = &dispc_timings_tc358765,
	.dsi_timings = &dsi_timings_tc358765,
};

static struct omap_dss_device *innocomm_dss_devices[] = {
	&innocomm_lcd_device,
#ifdef CONFIG_OMAP4_DSS_HDMI
	&innocomm_hdmi_device,
#endif /* CONFIG_OMAP4_DSS_HDMI */
};

static struct omap_dss_board_info innocomm_dss_data = {
	.num_devices	= ARRAY_SIZE(innocomm_dss_devices),
	.devices	= innocomm_dss_devices,
	.default_device	= &innocomm_lcd_device,
};

/*
 * We calculate tiler1d slot size automatically in android-display.c.
 * If you see the below Kernel log, increase and configure the correct tiler1d slot size here.
 * "misc dsscomp: tiler slot not big enough for frame ..."
 */
static struct dsscomp_platform_data dsscomp_config_icom = {
	.tiler1d_slotsz = 0,
};

/* OMAPLFB_NUM_DEV could be 1 for Android 4.1.x */
#define OMAPLFB_NUM_DEV 1

static struct sgx_omaplfb_config omaplfb_config_icom[OMAPLFB_NUM_DEV] = {
	{
		.tiler2d_buffers = 2,
		.vram_buffers = 0, /* if tiler2d_buffers is 0, vram_buffers should be 2 */
		.vram_reserve = 0,
		.swap_chain_length = 2,

		/*
		 * Total size of tiler1d and tiler2d. The size must be aligned with 128KB.
		 * If you see out of memory error in GraphicBufferAllocator, try to increase the total size here.
		 * "E/SurfaceFlinger(  117): GraphicBufferAlloc::createGraphicBuffer(w=1408, h=832) failed (Out of memory), handle=0x0"
		 *
		 * But we still have no solution for the below Kernel error.
		 * "omap_tiler_alloc: failure to allocate address space from tiler"
		 */
#if 1
		.tiler_total_size = (SZ_128M + SZ_32M + SZ_8M),
#else
		 /* If Android Wallpaper Service is disabled, we can reduce 24MB size here. */
		.tiler_total_size = (SZ_128M + SZ_16M),
#endif
	}
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_icom = {
	.num_configs = OMAPLFB_NUM_DEV,
	.configs = omaplfb_config_icom,
};

struct omap_icom_panel_data {
	struct omap_dss_board_info *board_info;
	struct dsscomp_platform_data *dsscomp_data;
	struct sgx_omaplfb_platform_data *omaplfb_data;
};

static struct omap_icom_panel_data panel_data_icom = {
	.board_info = &innocomm_dss_data,
	.dsscomp_data = &dsscomp_config_icom,
	.omaplfb_data = &omaplfb_plat_data_icom,
};

static struct omapfb_platform_data icom_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
	},
};

void quartz_android_display_setup(struct omap_ion_platform_data *ion)
{
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (videolfb_is_enabled())
		innocomm_lcd_device.skip_init = true;
#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */

	omap_android_display_setup(panel_data_icom.board_info,
				   panel_data_icom.dsscomp_data,
				   panel_data_icom.omaplfb_data,
				   &icom_fb_pdata,
				   ion);
}

static void innocomm_lcd_init(void)
{
	u32 reg;

	/* Enable 4 lanes in DSI1 module, disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);

	omap_mux_init_gpio(LCD_RESET, OMAP_PIN_OUTPUT);
	if (gpio_request(LCD_RESET, "LCD_RESET") < 0)
		pr_err("LCM: can't reserve reset GPIO: %d\n", LCD_RESET);
	if (videolfb_is_enabled())
		gpio_direction_output(LCD_RESET, 1);
	else
		gpio_direction_output(LCD_RESET, 0);
}

#ifdef CONFIG_LEDS_TWL6032_PWM
void innocomm_panel_set_twl_pwm(struct twl_pwm_device *pwm, int brightness)
{
	twl6032_pwm = pwm;
	pwm->dev = &(innocomm_lcd_device.dev);

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
	omap_dss_backlight_init_brightness(brightness);
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */
}
#endif /* CONFIG_LEDS_TWL6032_PWM */

int __init quartz_panel_init(void)
{
	innocomm_lcd_init();

	innocomm_hdmi_mux_safemode();

	omapfb_set_platform_data(&icom_fb_pdata);

	omap_display_init(panel_data_icom.board_info);

	return 0;
}
