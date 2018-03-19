/*
 * Board support file for Quartz
 *
 * Copyright (C) 2011 InnoComm Mobile Technology Corp.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <plat/common.h>
#include <plat/omap_hwmod.h>
#include <asm/mach-types.h>
#include "mux.h"
#include "board-44xx-quartz.h"

static struct i2c_board_info __initdata quartz_i2c_2_boardinfo[] = {
#if defined(CONFIG_PANEL_TC358765)
	{
		I2C_BOARD_INFO("tc358765_i2c_driver", 0x0F),
	}
#endif
};

static struct i2c_board_info __initdata quartz_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("5M_CAM", 0x10),
	},
	{
		I2C_BOARD_INFO("2M_CAM", 0x20),
	},
};

static struct i2c_board_info __initdata quartz_i2c_4_boardinfo[] = {
#if defined(CONFIG_OMAP4_TMP102_SENSOR) /*Workaround for thermal warning messages*/
	{
		I2C_BOARD_INFO("tmp105_temp_sensor", 0x48),
	},
#endif
};

void __init quartz_i2c_init(void)
{
	omap_register_i2c_bus(2, 400,
			quartz_i2c_2_boardinfo, ARRAY_SIZE(quartz_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400,
			quartz_i2c_3_boardinfo, ARRAY_SIZE(quartz_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400,
			quartz_i2c_4_boardinfo, ARRAY_SIZE(quartz_i2c_4_boardinfo));
}
