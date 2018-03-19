/*
 * linux/arch/arm/mach-omap2/board-44xx-quartz-ldos.c
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 * John Lin <john.Lin@innocomm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <plat/videolfb.h>

#include "board-44xx-quartz.h"

// LCM_5V consumers
static struct regulator_consumer_supply quartz_lcm_5V_supply[] = {
    REGULATOR_SUPPLY("lcm_5V", "display0"),
};

static struct regulator_init_data quartz_lcm_5V_data = {
	.constraints = {
		.min_uV				= 5000000,
		.max_uV				= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(quartz_lcm_5V_supply),
	.consumer_supplies		= quartz_lcm_5V_supply,
};

static struct fixed_voltage_config quartz_fixed_lcm_5V_regulator = {
	.supply_name		= "LCM_5V",
	.microvolts			= 5000000,
	.gpio				= 51,
	.enable_high		= 0, //low enable
	.enabled_at_boot	= 0,
	.init_data			= &quartz_lcm_5V_data,
};

static struct platform_device quartz_lcm_5V_regulator = {
	.name	= "reg-fixed-voltage",
	.id		= 0, /* unique id */
	.dev	= {
		.platform_data = &quartz_fixed_lcm_5V_regulator,
	},
};

/* CAM_2V8 consumers */
static struct regulator_consumer_supply quartz_cam2v8_supply[] = {
	REGULATOR_SUPPLY("cam_2v8", NULL),
};

static struct regulator_init_data quartz_cam2v8_data = {
	.constraints = {
		.min_uV				= 2800000,
		.max_uV				= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(quartz_cam2v8_supply),
	.consumer_supplies		= quartz_cam2v8_supply,
};

static struct fixed_voltage_config quartz_fixed_cam2v8_regulator = {
	.supply_name		= "CAM_2V8",
	.microvolts			= 2800000,
	.gpio				= 42,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data			= &quartz_cam2v8_data,
};

static struct platform_device quartz_cam2v8_regulator = {
	.name	= "reg-fixed-voltage",
	.id		= 3, /* unique id */
	.dev	= {
		.platform_data = &quartz_fixed_cam2v8_regulator,
	},
};

// VTP_5V consumers
static struct regulator_consumer_supply quartz_tp5v_supply[] = {
    REGULATOR_SUPPLY("vtp5v", QUARTZ_TOUCH_NAME),
};

static struct regulator_init_data quartz_tp5v_data = {
	.constraints = {
		.min_uV				= 5000000,
		.max_uV				= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(quartz_tp5v_supply),
	.consumer_supplies		= quartz_tp5v_supply,
};

static struct fixed_voltage_config quartz_fixed_tp5v_regulator = {
	.supply_name		= "VTP5V",
	.microvolts			= 5000000,
	.gpio				= 37,
	.enable_high		= 0, // active low
	.enabled_at_boot	= 0,
	.init_data			= &quartz_tp5v_data,
};

static struct platform_device quartz_tp5v_regulator = {
	.name	= "reg-fixed-voltage",
	.id		= 2, /* unique id */
	.dev	= {
		.platform_data = &quartz_fixed_tp5v_regulator,
	},
};

/* V5V_EN consumers */
static struct regulator_consumer_supply quartz_v5v_supply[] = {
	REGULATOR_SUPPLY("usb_phy_v5v", NULL),
	REGULATOR_SUPPLY("vtp_v5v", QUARTZ_TOUCH_NAME),
	REGULATOR_SUPPLY("lcm_v5v", "display0"),
	
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),	
	
};

static struct regulator_init_data quartz_v5v_data = {
	.constraints = {
		.min_uV				= 5000000,
		.max_uV				= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.boot_on	= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(quartz_v5v_supply),
	.consumer_supplies		= quartz_v5v_supply,
};

static struct fixed_voltage_config quartz_fixed_v5v_regulator = {
	.supply_name		= "V5V",
	.microvolts			= 5000000,
	.gpio				= 41,
	.enable_high		= 1,
	.enabled_at_boot	= 1,
	.disable_at_shutdown	= 1,
	.shutdown_delay			= 3000, /* us */
	.init_data			= &quartz_v5v_data,
};

static struct platform_device quartz_v5v_regulator = {
	.name	= "reg-fixed-voltage",
	.id		= 4, /* unique id */
	.dev	= {
		.platform_data = &quartz_fixed_v5v_regulator,
	},
};
/* Ant power LDO control*/
static struct regulator_consumer_supply quartz_ant50v_supply[] = {
	REGULATOR_SUPPLY("ant_v50", NULL),
};

static struct regulator_init_data quartz_ant50v_data = {
	.constraints = {
		.min_uV				= 5000000,
		.max_uV				= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(quartz_ant50v_supply),
	.consumer_supplies		= quartz_ant50v_supply,
};

static struct fixed_voltage_config quartz_fixed_ant50v_regulator = {
	.supply_name		= "ANT_5V0",
	.microvolts			= 5000000,
	.gpio				= 60,
	.enable_high		= 0,// active low
	.enabled_at_boot	= 1,
	.init_data			= &quartz_ant50v_data,
};

static struct platform_device quartz_ant50v_regulator = {
	.name	= "reg-fixed-voltage",
	.id		= 6, /* unique id */
	.dev	= {
		.platform_data = &quartz_fixed_ant50v_regulator,
	},
};
void __init quartz_ldo_init(void)
{	
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (videolfb_is_enabled()) {
		/* V5V */
		quartz_fixed_v5v_regulator.enabled_at_boot = 1;
		quartz_v5v_data.constraints.boot_on = 1;
		/* LCM_5V */
		quartz_fixed_lcm_5V_regulator.enabled_at_boot = 1;
		quartz_lcm_5V_data.constraints.boot_on = 1;
	}
#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */

	if (platform_device_register(&quartz_lcm_5V_regulator) < 0) {
		printk(KERN_ERR "unable to register lcm_5V_en_n fixed regulator\n");
	}

	/* Currently the fixed regulator driver supports only one fix regulator */
	if (platform_device_register(&quartz_cam2v8_regulator) < 0) {
		printk(KERN_ERR "unable to register cam2v8 fixed regulator\n");
	}

	if (platform_device_register(&quartz_ant50v_regulator) < 0) {
		printk(KERN_ERR "unable to register ant 5V fixed regulator\n");
	}
		

	if (platform_device_register(&quartz_tp5v_regulator) < 0) {
		printk(KERN_ERR "unable to register tp5v fixed regulator\n");
	}

	if (platform_device_register(&quartz_v5v_regulator) < 0) {
		printk(KERN_ERR "unable to register v5ven fixed regulator\n");
	}
}
