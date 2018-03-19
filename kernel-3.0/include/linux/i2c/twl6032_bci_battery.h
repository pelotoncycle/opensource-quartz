/*
 * linux/drivers/power/twl6030_bci_battery.h
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

#ifndef _LINUX_TWL_6032_BCI_BATTERY_H
#define _LINUX_TWL_6032_BCI_BATTERY_H

/*no parameter*/
#define BCI_RESET				0x00000001 /*SoftReset*/
#define BCI_RESET_TIMER			0x00000002/*Reset timer*/
#define BCI_SHUTDOWN			0x00000004/*System shutting down*/
/*Simple parameter*/
#define BCI_START_STOP_CHARGING		0x00000010 /*para:NULL  stop, non-NULL start with current*/
#define BCI_ENABLE_POWER_PATH			0x00000020 /*para:NULL disable, non-NULL enable*/
#define BCI_ENABLE_EOC					0x00000040/*para:NULL disable, non-NULL enable*/
#define BCI_SET_CHARGE_CURRENT		0x00000080/*para:int current mA*/
#define BCI_SET_EOC_CURRENT			0x00000100/*para:int current mA*/
#define BCI_SET_INPUT_CURRENT_LIMIT	0x00000200/*para:int current mA*/
#define BCI_READ_CURRENT				0x00000400/*para:int current mA*/

/*Complex parameters*/
#define BCI_CONFIG						0x00001000/**/
#define BCI_READ_FAULT_REASON			0x00002000/*bci_charger_read_fault*/
#define BCI_CHAEGER_STATUS_REQUEST	0x00004000/*bci_charger_status_requset*/
#define BCI_FUELGAUGE_STATUS_REQUEST	0x00008000 /*bci_fuel_gauge_status_request*/

/*Type Field, "|" to commands*/
#define BCI_INTERNAL_FUELGAUGE				0x10000000
#define BCI_EXTERNAL_FUELGAUGE			0x20000000
#define BCI_INTERNAL_CHARGER				0x40000000
#define BCI_EXTERNAL_CHARGER				0x80000000
#define BCI_FUELGAUGE						(BCI_INTERNAL_FUELGAUGE|BCI_EXTERNAL_FUELGAUGE)
#define BCI_CHARGER							(BCI_INTERNAL_CHARGER|BCI_EXTERNAL_CHARGER)
#define BCI_TYPE_MASK						(BCI_INTERNAL_CHARGER|BCI_EXTERNAL_CHARGER|\
											BCI_INTERNAL_FUELGAUGE|BCI_EXTERNAL_FUELGAUGE)

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



/*BCI registers offset-TWL6030_MODULE_CHARGER*/
#define CONTROLLER_INT_MASK	0x00
#define CONTROLLER_CTRL1	0x01
#define CONTROLLER_WDG		0x02
#define CONTROLLER_STAT1	0x03
#define CHARGERUSB_INT_STATUS	0x04
#define CHARGERUSB_INT_MASK	0x05
#define CHARGERUSB_STATUS_INT1	0x06
#define CHARGERUSB_STATUS_INT2	0x07
#define CHARGERUSB_CTRL1	0x08
#define CHARGERUSB_CTRL2	0x09
#define CHARGERUSB_CTRL3	0x0A
#define CHARGERUSB_STAT1	0x0B /*Not present in 6032*/
#define CHARGERUSB_VOREG	0x0C
#define CHARGERUSB_VICHRG	0x0D
#define CHARGERUSB_CINLIMIT	0x0E
#define CHARGERUSB_CTRLLIMIT1	0x0F
#define CHARGERUSB_CTRLLIMIT2	0x10
#define ANTICOLLAPSE_CTRL1	0x11
#define ANTICOLLAPSE_CTRL2	0x12 /*Not present in 6032*/
#define LED_PWM_CTRL1		0x14 /*Not present in 6030*/
#define LED_PWM_CTRL2		0x15 /*Not present in 6030*/
/* TWL6032 registers 0xDA to 0xDE - TWL6032_MODULE_CHARGER */
#define CONTROLLER_CTRL2	0x00
#define CONTROLLER_VSEL_COMP	0x01
#define CHARGERUSB_VSYSREG	0x02
#define CHARGERUSB_VICHRG_PC	0x03
#define LINEAR_CHRG_STS		0x04
/*Config is to set the charger default parameters, 
some paraments will be override run time.
*/
struct bci_config_charger{
	int				input_current;/*limit to the chager input current, runtime changeable*/
	int				max_input_current;/*abs the chager input current limit, hw limitation*/
	int				max_input_voltage;/*abs the chager input voltage limit, hw limitation*/
	int				charge_current;/*the current charge to battery in CC mode */
	int				eoc_current;/*the eoc current */
	int				eoc_enable;/*the eoc default is enable or disable */
	int 				battery_regulation_voltage;/*Battery CV voltage*/
//	int 				battery_max_voltage;/*Battery charge protect voltage*/
};
struct bci_config_fuel_gauge{
	int				full_capcity_mAh;/*the battery full charged capacity*/
	int				battery_voltage;/*The battery full voltage*/
	int				battery_cutoff_voltage;/*the battery lowest working voltage */
	int				eoc_current;/*the eoc current */
};
struct bci_fuel_gauge_status_request{
	int				current_state;
	int				previous_state;
	int				capacity;
	int				current_uA;
	int				temp_adc_voltage_mv;
	int 				voltage;
	int 				temperature;
	int				capacity_changed;
	int				data_valid;
};
struct bci_charger_status_requset{
	int				current_state;
	int				previous_state;
	int				capacity;
	int				current_uA;
	int 				voltage;	
};
struct bci_charger_read_fault{
	int				status;
	int				reasons;
	int				continue_charge;
	int				capacity;
	int				current_uA;
	int 				voltage;
};
/*The debug info is by category*/
enum debug_config {
    debug_common=0x01, 	// 0001
    debug_plug_in_out = 0x02, 		//0010
    debug_fault = 0x04,		//0100
    debug_int_fuel_gauge = 0x08,		//1000		for discharging
    debug_temperature = 0x10,		//0001 0000 	for charging
    debug_int_charger = 0x20,		//0010 0000
    debug_ext_charger = 0x40,		//0100 0000
    debug_continuous_info = 0x80,		//1000 0000	
    //debug_leve_max = 0x80,
    debug_leve_max = 0xFF,
};
extern int twl6030_bci_function_register(const char* name,struct notifier_block *nb, unsigned int events, int type,void* priv);
extern int twl6030_bci_function_unregister(const char* name);

extern void *twl6030_bci_function_get_priv( const char* name);
extern unsigned int twl6030_bci_debug_mask(void);
#ifdef CONFIG_TWL6032_BCI_CHARGER
extern int __init twl6030_charger_init(struct platform_device *pdev);
#else
static inline int __init twl6030_charger_init(struct platform_device *pdev){return 0;}
#endif
#ifdef CONFIG_TWL6032_BCI_FUEL_GAUGE
extern int __init twl6030_fuel_gauge_init(struct platform_device *pdev);
#else
static inline  int __init twl6030_fuel_gauge_init(struct platform_device *pdev){ return 0;}
#endif
#define bci_dbg(x,f...) if(twl6030_bci_debug_mask()&x)printk(KERN_INFO f) 
#endif
