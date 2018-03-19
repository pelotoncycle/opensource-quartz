/*
 * arch/arm/mach-omap2/board-44xx-tablet.h
 *
 * Copyright (C) 2011 Texas Instruments
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
/*This is a common header file for Talos7/Talos10/Zeus/Olympus*/
#ifndef _BOARD_44XX_RB_COMMON_H
#define _BOARD_44XX_RB_COMMON_H

//For Talos7/Talos10/Zeus/Olympus
void omap4_create_board_props(void);
void board_serial_init(void);
struct omap_ion_platform_data;

#define PCB_THERMAL_NAME			"tmp105" //For Talos7/Talo10/Zesu/Olympus

#if defined(CONFIG_INPUT_ELAN6804) //For Talos7/Talo10/Zesu/Olympus
  #define LIGHT_DRIVER_NAME "elan-epl6804"
#endif
#if defined(CONFIG_INPUT_ELAN6802) // for Saga
  #define LIGHT_DRIVER_NAME "elan-epl6802"
#endif

//For Talos7/Talos10/Zeus/Olympus
struct sensors_pm_platform_data {
	char *name;
	struct platform_device *sensor_devices;
	struct device *i2c_client_dev;
	struct regulator *reg_vcore;
	struct regulator *reg_vio;
	int onoff_vcore;
	int onoff_vio;
	int	 (*init_platform_hw)(struct sensors_pm_platform_data *pdata);
	void (*exit_platform_hw)(struct sensors_pm_platform_data *pdata);
	int	 (*suspend_platform_hw)(struct sensors_pm_platform_data *pdata);
	int	 (*resume_platform_hw)(struct sensors_pm_platform_data *pdata);
	const char * vcore_name;
	const char * vio_name;
	const char * reset_gpio_name;
	int	reset_gpio;
	int	reset_active_level;
	int poll_interval;
	int min_interval;

	u8 range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};

struct touch_platform_data {
	struct platform_device *device;
	struct device *i2c_client_dev;
	struct regulator *reg_vcore;
	struct regulator *reg_vio;
	int onoff_vcore;
	int onoff_vio;
#if defined(CONFIG_MACH_OMAP4_ZEUS) //For Zeus only
	struct regulator *reg_lcm3v3;
	int onoff_lcm3v3;
	const char *vlcm_3v3;		
#endif
	const char *vcore_name;
	const char *vio_name;
	bool tp_probed; /* If one of the tp drivers probed, set to TRUE */
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int mode; //0:Normal, 1:Download
	int irq_gpio;
	int reset_gpio; /*Oracle products will use this gpio*/
	int (*get_irq_state)(void);
	int (*init_platform_hw)(struct touch_platform_data *pdata);
	void (*exit_platform_hw)(struct touch_platform_data *pdata);
	int (*suspend_platform_hw)(struct touch_platform_data *pdata);
	int (*resume_platform_hw)(struct touch_platform_data *pdata);
};
#endif

