/*
 * (C) Copyright 2012
 * InnoComm Mobile Technology Corp.
 * Jiahe Jou <jiahe.jou@innocomm.com>
 * James Wu <james.wu@innocomm.com>
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
#include <twl6030.h>
#include <errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/gpio.h>
#include <linux/usb/otg.h>

#include <icom/aboot.h>
#include <icom/pwm.h>
#include <icom/omapdss.h>
#include <icom/omap-panel-tc358765.h>

#include "../common/board_omap4_common.h"
#include "board_mux_data.h"

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_BOARD_DEBUG
#define CONFIG_BOARD_DEBUG
#endif /* CONFIG_BOARD_DEBUG */
#ifndef CONFIG_BOARD_VERBOSE_DEBUG
#define CONFIG_BOARD_VERBOSE_DEBUG
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_BOARD_DEBUG
#define BOARD_DPRINT(fmt, args...) \
	do {printf("[board] " fmt, ##args);} while (0)
#define BOARD_DPUTS(fmt) \
	do {puts("[board] " fmt);} while (0)
#else /* !CONFIG_BOARD_DEBUG */
#define BOARD_DPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_DEBUG */

#ifdef CONFIG_BOARD_VERBOSE_DEBUG
#define BOARD_VPRINT(fmt, args...) \
	do {printf("[board] " fmt, ##args);} while (0)
#define BOARD_VPUTS(fmt) \
	do {puts("[board] " fmt);} while (0)
#else /* !CONFIG_BOARD_VERBOSE_DEBUG */
#define BOARD_VPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */

#define BOARD_PRINT(fmt, args...) \
	do {printf("board: " fmt, ##args);} while (0)
#define BOARD_PUTS(fmt) \
	do {puts("board: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)

/*-------------------------------------------------------------------------*/

DECLARE_GLOBAL_DATA_PTR;

/*-------------------------------------------------------------------------*/

#if !defined(CONFIG_SPL_BUILD)
static u32 board_revision = 0xFFFFFFFF;

u32 get_board_rev(void)
{
	return board_revision;
}
#endif /* CONFIG_SPL_BUILD */

/*-------------------------------------------------------------------------*/

void set_muxconf_regs_essential(void)
{
#if defined(CONFIG_SPL_BUILD)
	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_essential,
		   sizeof(core_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_essential,
		   sizeof(wkup_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	innocomm_board_detect_factory_tool();
#endif /* CONFIG_SPL_BUILD */
}

void set_muxconf_regs_non_essential(void)
{
#if !defined(CONFIG_SPL_BUILD)
	board_revision = innocomm_board_identify();

	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_non_essential,
		   sizeof(core_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_non_essential,
		   sizeof(wkup_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));

	innocomm_board_detect_factory_tool();
#endif /* !CONFIG_SPL_BUILD */
}

int board_is_factory_mode(void)
{
	return innocomm_board_is_factory_mode();
}

/*-------------------------------------------------------------------------*/

#if !defined(CONFIG_SPL_BUILD)

/*-------------------------------------------------------------------------*/

#define BOARD_FIXED_GPIO_LDO_ETHERNET		33	/* ETHERNET_EN(_N) */
#define BOARD_FIXED_GPIO_LDO_VTP_N		37	/* VTP_EN_N */
#define BOARD_FIXED_GPIO_LDO_V5V		41	/* V5V_EN */
#define BOARD_FIXED_GPIO_LDO_CAM_2V8		42	/* CAM_2V8_EN */
#define BOARD_FIXED_GPIO_LDO_LVDS_1V2		44	/* LVDS_1V2_EN */
#define BOARD_FIXED_GPIO_LDO_LVDS_1V8		45	/* LVDS_1V8_EN */
#define BOARD_FIXED_GPIO_LDO_LCM_5V_N		51	/* LCM_5V_EN_N */
#define BOARD_FIXED_GPIO_LDO_WL			54	/* WL_EN */
#define BOARD_FIXED_GPIO_LDO_BT			55	/* BT_EN */
#define BOARD_FIXED_GPIO_LDO_RS232		59	/* RS232_EN */
#define BOARD_FIXED_GPIO_LDO_MODEM_CLK		60	/* MODEM_CLK_EN */
#define BOARD_FIXED_GPIO_LDO_BL_N		61	/* BL_EN_N */
#define BOARD_FIXED_GPIO_LDO_MIC_BIAS		161	/* MIC_BIAS_EN */

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_TWL6030_PWM

static void innocomm_pwm1_init(void)
{
}

static void innocomm_pwm1_enable(void)
{
	mdelay(45); /* delay to wait for PWM signal stable & avoid the LCD flicker */

	gpio_direction_output(BOARD_FIXED_GPIO_LDO_BL_N, 0);
}

static void innocomm_pwm1_disable(void)
{
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_BL_N, 1);
}

static struct twl6030_pwm innocomm_pwms[] = {
	{	/* PWM1 */
		.id					= 0,	/* TWL6030 PWM1 */
		.clock_cycle		= 64,	/* 128:256Hz, 64:512Hz */
		.invert_duty_cycle	= 1,
		.max_duty_cycle		= 100,
		.min_duty_cycle		= 0,
		.platform_init		= innocomm_pwm1_init,
		.platform_enable	= innocomm_pwm1_enable,
		.platform_disable	= innocomm_pwm1_disable,
	},
};
#endif /* CONFIG_TWL6030_PWM */

/*-------------------------------------------------------------------------*/

/**
 * @brief board_early_init_f
 *
 * James Wu: triggered by board_init_f()
 *  1.	BSS is not clear
 *  2.	UART console is NOT ready
 *  3.	timer is NOT ready
 *
 * @return 0
 */
int board_early_init_f(void)
{
	innocomm_board_early_init_f();

	return 0;
}

/**
 * @brief board_power_resources_init -
 * Turn on/off the board-specific regulators.
 */
void board_power_resources_init(void)
{
	/* Turn off LDOs */

	/* Turn off TWL6032 LDOs */
	regulator_disable(TWL6032_LDO1);	/* VPMIC_OMAP_VPP_CUST */
	regulator_disable(TWL6032_LDO3);	/* V_LVDS_3V3 */
	regulator_disable(TWL6032_LDO4);	/* VNET_D3V3 */
	regulator_disable(TWL6032_LDO7);	/* V_CAM_1V8 */

	/* Turn off Fixed GPIO LDOs */
	writew((IDIS | M3), CONTROL_PADCONF_CORE + GPMC_WAIT0);	/* gpio_61: BL_EN_N active low */
	gpio_request(BOARD_FIXED_GPIO_LDO_BL_N, NULL);
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_BL_N, 1);

	gpio_request(BOARD_FIXED_GPIO_LDO_ETHERNET, NULL);		/* ETHERNET_EN(_N) */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_ETHERNET, 1);

	gpio_request(BOARD_FIXED_GPIO_LDO_VTP_N, NULL);			/* VTP_EN_N */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_VTP_N, 1);

	gpio_request(BOARD_FIXED_GPIO_LDO_V5V, NULL);			/* V5V_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_V5V, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_CAM_2V8, NULL);		/* CAM_2V8_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_CAM_2V8, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_LVDS_1V2, NULL);		/* LVDS_1V2_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_LVDS_1V2, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_LVDS_1V8, NULL);		/* LVDS_1V8_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_LVDS_1V8, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_LCM_5V_N, NULL);		/* LCM_5V_EN_N */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_LCM_5V_N, 1);

	gpio_request(BOARD_FIXED_GPIO_LDO_WL, NULL);			/* WL_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_WL, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_BT, NULL);			/* BT_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_BT, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_RS232, NULL);			/* RS232_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_RS232, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_MODEM_CLK, NULL);		/* MODEM_CLK_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_MODEM_CLK, 0);

	gpio_request(BOARD_FIXED_GPIO_LDO_MIC_BIAS, NULL);		/* MIC_BIAS_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_MIC_BIAS, 0);

	mdelay(50);
}

/**
 * @brief board_init
 *
 * James Wu:
 *  1.	UART console is ready
 *  2.	timer is ready
 *  3.  before mem_malloc_init()
 *  4.  before interrupt_init()
 *
 * @return 0
 */
int board_init(void)
{
#ifdef CONFIG_TWL6030_PWM
	twl6030_pwm_init(innocomm_pwms, ARRAY_SIZE(innocomm_pwms));
#endif /* CONFIG_TWL6030_PWM */

	innocomm_board_init();

	/* James Wu: ALL otg notifiers must be registered above! */
#ifdef CONFIG_USB_OTG_TRANSCEIVER
	aboot_register_last_otg_notifier();
#if !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
	otg_handle_interrupts(1);
#endif
#endif /* CONFIG_USB_OTG_TRANSCEIVER */
	return 0;
}

#if defined(CONFIG_OMAP2_DSS)

#define LCD_RESET	43

static int innocomm_panel_set_backlight(struct omap_dss_device *dssdev, int level)
{
#ifdef CONFIG_TWL6030_PWM
	pwm_set_duty(0, level);
#else
#error "Please define CONFIG_TWL6030_PWM\n"
#endif /* CONFIG_TWL6030_PWM */
	return 0;
}

static int innocomm_panel_enable(struct omap_dss_device *dssdev)
{
	/* V5V_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_V5V, 1);

	regulator_set_voltage(TWL6032_LDO3, 3300);

	/* LCM_5V_EN_N */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_LCM_5V_N, 0);

	/* V_LVDS_3V3 */
	regulator_enable(TWL6032_LDO3);

	mdelay(10);

	return 0;
}

static void innocomm_panel_disable(struct omap_dss_device *dssdev)
{
	/* LCM_5V_EN_N */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_LCM_5V_N, 1);
	/* V_LVDS_3V3 */
	regulator_disable(TWL6032_LDO3);

	mdelay(10);

	/* V5V_EN */
	gpio_direction_output(BOARD_FIXED_GPIO_LDO_V5V, 0);

	mdelay(40);
}

static int innocomm_dss_enable(void)
{
	/* Turn on TWL6032 LDO6 for vdds_dsi */
	regulator_enable(TWL6032_LDO6);	/* VCXIO_1V8 */
	return 0;
}

static void innocomm_dss_disable(void)
{
	/* We can't turn off TWL6032_LDO6 (VCXIO_1V8) because it will cause the system halt */
}

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
		BOARD_PUTS("failed to LVMX0003\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 4, 4, 0);	// in4: R4
	val = FLD_MOD(val, 7, 12, 8);	// in5: R7
	val = FLD_MOD(val, 5, 20, 16);	// in6: R5
	val = FLD_MOD(val, 8, 28, 24);	// in7: G0
	r = write(dssdev, LVMX0407, val);
	if (r) {
		BOARD_PUTS("failed to LVMX0407\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 9, 4, 0);	// in8: G1
	val = FLD_MOD(val, 10, 12, 8);	// in9: G2
	val = FLD_MOD(val, 14, 20, 16);	// in10: G6
	val = FLD_MOD(val, 15, 28, 24);	// in11: G7
	r = write(dssdev, LVMX0811, val);
	if (r) {
		BOARD_PUTS("failed to LVMX0811\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 11, 4, 0);	// in12: G3
	val = FLD_MOD(val, 12, 12, 8);	// in13: G4
	val = FLD_MOD(val, 13, 20, 16);	// in14: G5
	val = FLD_MOD(val, 16, 28, 24);	// in15: B0
	r = write(dssdev, LVMX1215, val);
	if (r) {
		BOARD_PUTS("failed to LVMX1215\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 22, 4, 0);	// in16: B6
	val = FLD_MOD(val, 23, 12, 8);	// in17: B7
	val = FLD_MOD(val, 17, 20, 16);	// in18: B1
	val = FLD_MOD(val, 18, 28, 24);	// in19: B2
	r = write(dssdev, LVMX1619, val);
	if (r) {
		BOARD_PUTS("failed to LVMX1619\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 19, 4, 0);	// in20: B3
	val = FLD_MOD(val, 20, 12, 8);	// in21: B4
	val = FLD_MOD(val, 21, 20, 16);	// in22: B5
	val = FLD_MOD(val, 27, 28, 24);	// in23: RSVD
	r = write(dssdev, LVMX2023, val);
	if (r) {
		BOARD_PUTS("failed to LVMX2023\n");
		return r;
	}

	val = 0;
	val = FLD_MOD(val, 24, 4, 0);	// in24: HSYNC
	val = FLD_MOD(val, 25, 12, 8);	// in25: VSYNC
	val = FLD_MOD(val, 26, 20, 16);	// in26: DE
	val = FLD_MOD(val, 6, 28, 24);	// in27: R6
	r = write(dssdev, LVMX2427, val);
	if (r) {
		BOARD_PUTS("failed to LVMX2427\n");
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

	.platform_enable		= innocomm_panel_enable,
	.platform_disable		= innocomm_panel_disable,
	.set_backlight			= innocomm_panel_set_backlight,

	.dispc_timings = &dispc_timings_tc358765,
	.dsi_timings = &dsi_timings_tc358765,
};

static struct omap_dss_device *innocomm_dss_devices[] = {
	&innocomm_lcd_device,
};

static struct omap_dss_board_info innocomm_dss_data = {
	.num_devices	= ARRAY_SIZE(innocomm_dss_devices),
	.devices	    = innocomm_dss_devices,
	.default_device	= &innocomm_lcd_device,
	.dss_platform_enable	= innocomm_dss_enable,
	.dss_platform_disable	= innocomm_dss_disable,
};

#if defined(CONFIG_PANEL_TC358765)
extern int panel_tc358765_init(void);
#endif /* CONFIG_PANEL_TC358765 */

int board_display_init(void)
{
	int ret;
	u32 reg;

	/* Enable 4 lanes in DSI1 module, disable pull down */
	reg = __raw_readl(CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
	__raw_writel(reg, CONTROL_DSIPHY);

	/* LVDS_RST */
	writew((IDIS | M3), CONTROL_PADCONF_CORE + GPMC_A19);	/* gpio_43: LVDS_RST */
	gpio_request(LCD_RESET, NULL);
	gpio_direction_output(LCD_RESET, 0);

	ret = omap_display_init(&innocomm_dss_data);
	if (ret)
		return ret;

#if defined(CONFIG_PANEL_TC358765)
	ret = panel_tc358765_init();
	if (ret)
		return ret;
#endif /* CONFIG_PANEL_TC358765 */

	return 0;
}
#endif /* CONFIG_OMAP2_DSS */

/**
 * @brief misc_init_r - misc platform dependent initialisations
 *
 * James Wu:
 *  1.  after mem_malloc_init()
 *  2.	after env_relocate()
 *  3.  before interrupt_init()
 *  4.	before board_late_init()
 *
 * @return 0
 */
int misc_init_r(void)
{
	innocomm_board_misc_init();

	return 0;
}

/**
 * @brief board_late_init - late platform dependent initialisations
 *
 * James Wu:
 *  1.  after mem_malloc_init()
 *  2.  after interrupt_init()
 *
 * @return 0
 */
int board_late_init(void)
{
	innocomm_board_late_init();

	return 0;
}

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0);
	omap_mmc_init(1, 0, 0);
	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	/* James Wu: @TODO@ get cd status */
	return -1;
}
#endif

/**
 * @brief board_preboot_os - Board-specific preboot before Linux
 */
void board_preboot_os(void)
{
	innocomm_board_preboot_os();
}

/**
 * @brief board_setup_linux_cmdline - Board-specific Linux commands
 */
size_t board_setup_linux_cmdline(char *cmdline)
{
	char* cmdline_start = cmdline;

	/* Common Commands */
   	cmdline += innocomm_board_setup_linux_cmdline(cmdline);

	return (size_t)(cmdline - cmdline_start);
}

#if 0
/**
 * @brief board_shutdown_drivers - Shutdown the device drivers
 */
void board_shutdown_drivers(void)
{
	/* Shutdown the device drivers */
}
#endif

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_INIT
void i2c_omap_board_init(unsigned int bus)
{
}
#endif /* CONFIG_SYS_I2C_OMAP_BOARD_INIT */

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_LATE_INIT
void i2c_omap_board_late_init(unsigned int bus)
{
}
#endif /* CONFIG_SYS_I2C_OMAP_BOARD_LATE_INIT */

#endif /* CONFIG_SPL_BUILD */
