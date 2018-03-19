/*
 * arch/arm/mach-omap2/board-44xx-quartz.h
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
#ifndef _MACH_OMAP_BOARD_44XX_QUARTZ_H
#define _MACH_OMAP_BOARD_44XX_QUARTZ_H
#include "board-44xx-rb-common.h"

void omap_quartz_reserve(void);
void omap_quartz_map_io(void);
void omap_quartz_init_early(void);
void omap_quartz_init(void);
int quartz_panel_init(void);
void quartz_twl_init(void);
void quartz_i2c_init(void);
void quartz_ldo_init(void);
void quartz_spi_init(void);
void quartz_modem_init(void);
void quartz_serial_init(void);
void quartz_android_display_setup(struct omap_ion_platform_data *ion);

#if defined(CONFIG_CAM_FLASH_DRIVER)
#define QUARTZ_FLASH_DRIVER_NAME      "ssl3252_fs"
#endif /* defined(CONFIG_CAM_FLASH_DRIVER) */

#if defined(CONFIG_CHARGER_BQ2416x)
#define BQ2416x		"bq2416x"
#define BQ2416x_ADDR		0x6B
#endif /* defined(CONFIG_CHARGER_BQ2416x) */

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
#define QUARTZ_TOUCH_NAME	"quartz_tp"
struct quartz_ts_pdata {	
	int (*power_init) (struct platform_device *pdev);
	int (*power_free) (struct platform_device *pdev);
	int (*power_enable) (struct platform_device *pdev);
	int (*power_disable) (struct platform_device *pdev);
	struct regulator *reg_v5v;
	struct regulator *reg_vtp5v;
	bool onoff_v5v;
	bool onoff_vtp5v;
};
#endif /* CONFIG_TOUCHSCREEN_QUARTZ */

#endif /* _MACH_OMAP_BOARD_44XX_QUARTZ_H */

