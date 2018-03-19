/*
 * linux/drivers/power/twl6030_bci_battery.c
 *
 * OMAP4:TWL6030/TWL6032 battery driver for Linux
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Copyright (C) 2011 Innocomm Mobile Technology Corp.
 *
 * Code re-written by:
 * Innocomm Mobile Technology Corp.
 *
 * Initial Code:
 * Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _LINUX_TWL_6030_BCI_BATTERY_H
#define _LINUX_TWL_6030_BCI_BATTERY_H



#define BCI_SET_START_CHARGING		0x0001
#define BCI_SET_STOP_CHARGING		0x0002
#define BCI_READ_FAULT_REASON		0x0004
 
#define BCI_SET_CHARGE_CURRENT		0x0008
#define BCI_SET_EOC_CURRENT		0x0010
#define BCI_SET_INPUT_CURRENT_LIMIT		0x0020
#define BCI_RESET_TIMER		0x0040
#define BCI_SET_REGULATION_VOLTAGE		0x0080
#define BCI_SET_INPUT_VOLTAGE_LIMIT		0x0100
#define BCI_SET_EOC_ENABLE		0x0200
#define BCI_SET_EOC_DISABLE		0x0400
#define BCI_DEBUG_MONITOR		0x0800
#define BCI_RESET		0x1000
#define BCI_SET_EXT_DEBUG_EN		0x2000 //set debug_en_ext for external charger

#define BCI_FAULT_THERMAL_SHUTDOWN	1
#define BCI_FAULT_BAT_TEMP_FAULT		2
#define BCI_FAULT_WD_TIMER		3
#define BCI_FAULT_SAFTY_TIMER		4
#define BCI_FAULT_IN_SUPPLY_FAULT	5
#define BCI_FAULT_USB_SUPPLY_FAULT	6
#define BCI_FAULT_BAT_FAULT		7

#define BCI_EXT_FG_CAP 0x0001
#define BCI_EXT_FG_VOLT 0x0002
#define BCI_EXT_FG_CURR 0x0004
#define BCI_EXT_FG_TEMP 0x0008

#define BCI_EXT_FG_FIRMWARE_READ  0x0010
#define BCI_EXT_FG_FIRMWARE_WRITE 0x0020

#define BCI_EXT_FG_BLOCK 0x0100
#define BCI_EXT_FG_SET_TERM_VOLT 0x0200
#define BCI_EXT_FG_SET_SYSDN_VOLT 0x0400
#define BCI_EXT_FG_SET_SOC1 0x0800

#define NOTIFIER_REG_EXT_CHARGER 1
#define NOTIFIER_REG_EXT_FUELGAUGE 2

struct bci_fg_read {
	int				capacity;
	int				curr;
	int 				voltage;
	int 				temperature;
};
/* not a bq generated event,we use this to reset the
 * the timer from the twl driver.
 */

struct bci_read {
	int				status;
	int				reasons;
};
struct bq_read {
	int				value;
};
#endif
