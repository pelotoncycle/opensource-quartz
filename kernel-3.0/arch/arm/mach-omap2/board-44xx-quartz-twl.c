/*
 * Board support file for Innocomm Quartz Project.
 *
 * Copyright (C) 2012 Innocomm Mobile Technology Inc.
 *
 * Author: 
 * John.Lin <John.Lin@innocomm.com>
 * 
 * Based on mach-omap2/board-44xx-quartz-twl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/twl6040-vib.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/i2c.h>
#include <linux/cdc_tcxo.h>

#include <asm/mach-types.h>
 
#include <plat/usb.h>
#include "mux.h"
#include "common-board-devices.h"
#include "board-44xx-quartz.h"
#include <linux/delay.h>
#include <plat/videolfb.h>

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

/* V_LVDS_3V3 */
static struct regulator_consumer_supply twl_vldo3_supply[] = {
	REGULATOR_SUPPLY("v_lcm_3v3","display0"),
};

/* VNET_D3V3 */
static struct regulator_consumer_supply twl_vldo4_supply[] = {
#if defined(CONFIG_USB_NET_AX88XXX)
	REGULATOR_SUPPLY("vnet_d3v3",NULL),
#endif
};

/* VPMIC_VMMC */
static struct regulator_consumer_supply twl_vldo5_supply[] = {
	REGULATOR_SUPPLY("vmmc","omap_hsmmc.0"),
};

/* VCXIO_1V8 */
static struct regulator_consumer_supply twl_vldo6_supply[] = {
	REGULATOR_SUPPLY("cam_csi_1v8", NULL),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),/*The is need for DSI driver*/
};

/* V_CAM_1V8 */
static struct regulator_consumer_supply twl_vldo7_supply[] = {
	REGULATOR_SUPPLY("cam_pwr_1v8",NULL),
};

/* VPMIC_VDAC */
static struct regulator_consumer_supply twl_vldoln_supply[] = {
	REGULATOR_SUPPLY("hdmi_vref", NULL),
#ifdef CONFIG_USB_EHCI_HCD_OMAP //For USBPHY REFCLK, used by ehci-omap.c
	REGULATOR_SUPPLY("hsusb0", "ehci-omap.0"),
	REGULATOR_SUPPLY("hsusb1", "ehci-omap.0"),
	//hsusb port_mode[2] can't be used in PHY mode, so we don't need hsusb2
#endif
};

/* VPMIC_OMAP_VPP_CUST */
static struct regulator_init_data twl_vldo1 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
};

/* VeMMC_2V9 */
static struct regulator_init_data twl_vldo2 = {
	.constraints = {
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.always_on	= true,
	},
};

/* V_LVDS_3V3 */
static struct regulator_init_data twl_vldo3 = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(twl_vldo3_supply),
	.consumer_supplies = twl_vldo3_supply,
};

/* VNET_D3V3 */
static struct regulator_init_data twl_vldo4 = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			//.enabled	= false,
			//.disabled	= true,
			.enabled	= true,
			.disabled	= false,
		},
	},
	.num_consumer_supplies = ARRAY_SIZE(twl_vldo4_supply),
	.consumer_supplies = twl_vldo4_supply,
};

/* VPMIC_VMMC */
static struct regulator_init_data twl_vldo5 = {
	.constraints = {
		/* We could support 1.8V~3.3V */
		.min_uV			= 1800000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = ARRAY_SIZE(twl_vldo5_supply),
	.consumer_supplies      = twl_vldo5_supply,
};

/* VCXIO_1V8 */
static struct regulator_init_data twl_vldo6 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.always_on	= true,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl_vldo6_supply),
	.consumer_supplies = twl_vldo6_supply,	
};

/* V_CAM_1V8 */
static struct regulator_init_data twl_vldo7 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = ARRAY_SIZE(twl_vldo7_supply),
	.consumer_supplies      = twl_vldo7_supply,
};

/* VPMIC_VDAC */
static struct regulator_init_data twl_vldoln = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.always_on		= true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(twl_vldoln_supply),
	.consumer_supplies      = twl_vldoln_supply,
};

/* VPMIC_VUSB_3V3 */
static struct regulator_init_data twl_vldousb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
	},
};

static struct regulator_init_data twl_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data twl_clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data sysen = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
#if 0
		.initial_state		= PM_SUSPEND_MEM,
#endif
	},
};

static struct regulator_init_data regen1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
#if 0
		.initial_state		= PM_SUSPEND_MEM,
#endif
	},
};

static struct twl4030_madc_platform_data quartz_gpadc_data = {
	.irq_line	= 1,
};

static int quartz_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct ntc_res_temp_desc innocomm_batt_table[]={
	{-40,	195652},	{-35,	148171},	{-30,	113347},	{-25,	87558},  
	{-20,	68236},	{-15,	53649}, 	{-10,	42506},	{- 5,	33892},   
	{   0,	27218},	{   2,	24987},	{   4,	22962},	{   6,	21123},
	{   8,	19449},	{ 10,	17925},	{ 12,	16534}, 	{ 14,	15265},
	{ 16,	14107},	{ 18,	13048}, 	{ 20,	12080},	{ 22,	11194},
	{ 24,	10381},	{ 26,	 9634},	{ 28,	 8947},	{ 30,	 8314},
	{ 32, 	7733},	{ 34,	 7199},	{ 36,	 6706}, 	{ 38,	 6252},
	{ 40,	 5833},	{ 42,	 5445},	{ 44,	 5086},	{ 46,	 4753},  
	{ 48,	 4446},	{ 50,	 4160}, 	{ 52,	 3896}, 	{ 54,	 3651},  
	{ 56,	 3423},	{ 58,	 3211},	{ 60,	 3014}, 	{ 65,	 2586},
	{ 70,	 2227}, 	{ 75,	 1924},	{ 80,	 1668},	{ 100,	 0},
};

static struct batt_volt_cap_desc	Quartz_OCVtable[BATT_VOLT_CAP_SIZE] = {
	{3647,	0	},{3676,	5	},{3692,	10	},{3730,	15	},{3751,	20	},
	{3762,	25	},{3770,	30	},{3780,	35	},{3792,	40	},{3806,	45	},
	{3824,	50	},{3847,	55	},{3876,	60	},{3911,	65	},{3950,	70	},
	{3986,	75	},{4024,	80	},{4065,	85	},{4108,	90	},{4154,	95	},
	{4205,	100	}};

static struct batt_volt_cap_desc	Quartz_PwrOntable60[BATT_VOLT_CAP_SIZE] = {
	{3653,	0	},{3673,	5	},{3696,	10	},{3731,	15	},{3749,	20	},	
	{3762,	25	},{3769,	30	},{3778,	35	},{3789,	40	},{3803,	45	},
	{3822,	50	},{3851,	55	},{3891,	60	},{3922,	65	},{3952,	70	},
	{3985,	75	},{4022,	80	},{4062,	85	},{4105,	90	},{4151,	95	},
	{4202,	100	}};

static struct batt_volt_cap_desc	Quartz_PwrOntable800[BATT_VOLT_CAP_SIZE] = {
	{3553,	0	},{3598,	5	},{3614,	10	},{3650,	15	},{3669,	20	},	
	{3680,	25	},{3689,	30	},{3699,	35	},{3713,	40	},{3729,	45	},
	{3747,	50	},{3769,	55	},{3796,	60	},{3829,	65	},{3864,	70	},
	{3901,	75	},{3942,	80	},{3985,	85	},{4031,	90	},{4078,	95	},
	{4140,	100	}};

static struct batt_volt_cap_desc	Quartz_PwrOntable1000[BATT_VOLT_CAP_SIZE] = {
	{3547,	0	},{3588,	5	},{3607,	10	},{3638,	15	},{3657,	20	},
	{3668,	25	},{3677,	30	},{3688,	35	},{3703,	40	},{3720,	45	},
	{3739,	50	},{3761,	55	},{3787,	60	},{3818,	65	},{3853,	70	},
	{3891,	75	},{3931,	80	},{3975,	85	},{4021,	90	},{4069,	95	},
	{4132,	100	}};

static struct batt_volt_cap_desc	Quartz_PwrOntable1200[BATT_VOLT_CAP_SIZE] = {
	{3510,	0	},{3568,	5	},{3589,	10	},{3617,	15	},{3638,	20	},
	{3651,	25	},{3661,	30	},{3671,	35	},{3686,	40	},{3702,	45	},
	{3721,	50	},{3744,	55	},{3770,	60	},{3801,	65	},{3835,	70	},
	{3872,	75	},{3912,	80	},{3956,	85	},{4002,	90	},{4051,	95	},
	{4123,	100	}};

static struct batt_volt_cap_desc	Quartz_PwrOntable1800[BATT_VOLT_CAP_SIZE] = {
	{3458,	0	},{3516,	5	},{3542,	10	},{3563,	15	},{3582,	20	},
	{3596,	25	},{3608,	30	},{3620,	35	},{3635,	40	},{3654,	45	},
	{3674,	50	},{3697,	55	},{3723,	60	},{3753,	65	},{3785,	70	},
	{3822,	75	},{3863,	80	},{3907,	85	},{3954,	90	},{4003,	95	},
	{4075,	100	}};

static struct Loading_Table quartz_loading_Table[]={
	{0,Quartz_OCVtable},
	{-60,Quartz_PwrOntable60},
	{-800,Quartz_PwrOntable800}, 
	{-1000,Quartz_PwrOntable1000},
	{-1200,Quartz_PwrOntable1200},
	{-1800,Quartz_PwrOntable1800},
	{0,NULL},
};

static struct twl4030_bci_platform_data quartz_bci_data = {
	.monitoring_interval		= 2,
	.max_charger_currentmA		= 2000,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.termination_currentmA	=300,
	.battery_tmp_tbl		= quartz_batt_table,
	.tblsize	= ARRAY_SIZE(quartz_batt_table),
	.battery_tmp_tbl_603x		= innocomm_batt_table,
	.tblsize_603x	= ARRAY_SIZE(innocomm_batt_table),
	.loading_Table=quartz_loading_Table,
	.features = TWL6032_SUBCLASS,
	.use_eeprom_config = 0,
	.poll_msecs=2000,
	.capacity_mAh=6600,
	//.crazy_charger=NULL,
	.max_backupbat_voltagemV = 3000,  //0, 2500, 3000, 3150, 4200
	.no_main_battery = 1,
	.no_main_battery_monitoring_interval = 60 * 60 * 1000, /* 60mins */
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1, ES1.2 (both share same ASICREV value),
	 * ES1.3, ES2.0 and ES2.2
	 */
	if ((rev == TWL6040_REV_1_1) ||
	    (rev == TWL6040_REV_1_3) ||
	    (rev == TWL6041_REV_2_0) ||
	    (rev == TWL6041_REV_2_2)) {
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);
	}

	return 0;
}

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

#if 0
static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout		= 15000,
	.initial_vibrate	= 0,
	.voltage_raise_speed = 0x26,
};
#endif

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
#if 0
	.vibra		= &twl6040_vibra,
#endif
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};

#if defined(CONFIG_LEDS_TWL6032_PWM)

#define LCD_BL_EN_N	61

extern void innocomm_panel_set_twl_pwm(struct twl_pwm_device *pwm, int brightness);

static int innocomm_pwm1_init(struct twl_pwm_device *pwm, int brightness)
{
	innocomm_panel_set_twl_pwm(pwm, videolfb_is_enabled() ? brightness : 0);

	omap_mux_init_gpio(LCD_BL_EN_N, OMAP_PIN_OUTPUT);
	if (gpio_request(LCD_BL_EN_N, "LCD_BL_EN_N") < 0)
		pr_err("pwm1: cannot request gpio %d\n", LCD_BL_EN_N);

	if (videolfb_is_enabled())
		gpio_direction_output(LCD_BL_EN_N, 0);
	else
		gpio_direction_output(LCD_BL_EN_N, 1);

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
	return 0;
#else
	return 1;
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */
}

static void innocomm_pwm1_enable(struct twl_pwm_device *pwm)
{
	msleep(85); /* delay to wait for PWM signal stable & avoid the LCD flicker */

	gpio_set_value(LCD_BL_EN_N, 0);
}

static void innocomm_pwm1_disable(struct twl_pwm_device *pwm)
{
	gpio_set_value(LCD_BL_EN_N, 1);
}

static struct twl_pwm_led innocomm_pwms[] = {
	{
		.id						= 0,	/* TWL6032 PWM1 */
		.name					= "lcd-backlight",
		.default_trigger		= "none",
		.default_clock_cycle	= 64,	/* 128:256Hz, 64:512Hz */
		.invert_duty_cycle		= 1,
		.max_duty_cycle			= 100,	/* Limit the max duty cycle to bypass the HW flicker */
		.min_duty_cycle			= 0,
		.platform_init			= innocomm_pwm1_init,
		.platform_enable		= innocomm_pwm1_enable,
		.platform_disable		= innocomm_pwm1_disable,
	},
};

static struct twl_pwm_data innocomm_pwm_data = {
	.num_pwms	= ARRAY_SIZE(innocomm_pwms),
	.pwms		= innocomm_pwms,
};
#endif /* CONFIG_LEDS_TWL6032_PWM */

static struct twl_rtc_data innocomm_rtc_data = {
	.msecure_gpio = -1,
	.default_time = {
		.tm_sec	= 0,
		.tm_min	= 0,
		.tm_hour	= 0,
		.tm_mday	= 1,	/* 1=>1st */
		.tm_mon	= 11,	/* 11=>DEC */
		.tm_year	= 114,	/* 0=>1990, 114=>2014 */
	},
};

static struct twl4030_platform_data quartz_twldata6032 = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.ldo1		= &twl_vldo1,
	.ldo2		= &twl_vldo2,
	.ldo3		= &twl_vldo3,
	.ldo4		= &twl_vldo4,
	.ldo5		= &twl_vldo5,
	.ldo6		= &twl_vldo6,
	.ldo7		= &twl_vldo7,
	.ldoln		= &twl_vldoln,
	.ldousb		= &twl_vldousb,
	.clk32kg	= &twl_clk32kg,
	.clk32kaudio	= &twl_clk32kaudio,

	/* children */
	.usb		= &omap4_usbphy_data,
	.bci		= &quartz_bci_data,
	.codec		= &twl6040_codec,
	.madc		= &quartz_gpadc_data,
	.sysen		= &sysen,
	.regen1		= &regen1,
#if defined(CONFIG_LEDS_TWL6032_PWM)
	.pwm		= &innocomm_pwm_data,
#endif /* CONFIG_LEDS_TWL6032_PWM */
	.rtc		= &innocomm_rtc_data,
};

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data quartz_cdc_data = {
	.no_err_if_not_active = 1,
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

		CDC_TCXO_LDOEN1,

		0 },
};
static struct i2c_board_info __initdata quartz_i2c1_boardinfo[] = {
#ifdef CONFIG_CDC_TCXO
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &quartz_cdc_data,
	},
#endif
};
void __init quartz_twl_init(void)
{
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (videolfb_is_enabled())
		twl_vldo3.constraints.boot_on = 1;
#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */

	omap4_pmic_init("twl6032", &quartz_twldata6032);

#if defined(CONFIG_CDC_TCXO)
	if (system_rev == 0) /* EP1 */
		quartz_cdc_data.no_err_if_not_active = 0;
	i2c_register_board_info(1, quartz_i2c1_boardinfo,
			ARRAY_SIZE(quartz_i2c1_boardinfo));
#endif
}
