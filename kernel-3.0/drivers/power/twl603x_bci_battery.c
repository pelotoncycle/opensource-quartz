/*
 * linux/drivers/power/twl603x_bci_battery.c
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
#define DEBUG
#define POWEROFF_CHARGE
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/twl6030_bci_battery.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/mach-types.h>
#ifdef POWEROFF_CHARGE
#include <linux/android_boot.h>
#endif
#include <linux/i2c/bq27520.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/mutex.h>
#include <linux/semaphore.h>

#define GG_LOW_VOLT_REMAIN_CAP 10
#define GG_LOW_VOLT_TH_MV 3450
enum debug_config {
    debug_level_common=0x01, 	// 0001
    debug_level_A = 0x02, 		//0010
    debug_level_B = 0x04,		//0100
    debug_level_C = 0x08,		//1000		for discharging
    debug_level_D = 0x10,		//0001 0000 	for charging
    debug_level_E = 0x20,		//0010 0000
    debug_level_F = 0x40,		//0100 0000
    debug_level_G = 0x80,		//1000 0000	for external debug message
    debug_leve_max = 0x80,
};


enum irq_handler_reg {
    irq_reg_charge_state,
    irq_reg_hw_state,
    irq_reg_int_mask,
};

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
#define LINEAR_CHRG_STS_CRYSTL_OSC_OK		0x40
#define LINEAR_CHRG_STS_END_OF_CHARGE		0x20
#define LINEAR_CHRG_STS_VBATOV		0x10
#define LINEAR_CHRG_STS_VSYSOV		0x08
#define LINEAR_CHRG_STS_DPPM_STS		0x04
#define LINEAR_CHRG_STS_CV_STS		0x02
#define LINEAR_CHRG_STS_CC_STS		0x01
#define FG_REG_00	0x00
#define FG_REG_01	0x01
#define FG_REG_02	0x02
#define FG_REG_03	0x03
#define FG_REG_04	0x04
#define FG_REG_05	0x05
#define FG_REG_06	0x06
#define FG_REG_07	0x07
#define FG_REG_08	0x08
#define FG_REG_09	0x09
#define FG_REG_10	0x0A
#define FG_REG_11	0x0B
/*TWL 6032 CONTROLLER_CTRL2*/
#define EN_DPPM		(1 << 1)
#define SUP_MASK	(1 << 0)
/*TWL 6032 CHARGERUSB_VSYSREG*/
#define VSYS_SW_CTRL (1<<7)
/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 7)
#define MAC_EOC			(1 << 6)
#define LINCH_GATED		(1 << 5)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)
/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_LINCH	(1 << 5)
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)
/* CONTROLLER_STAT1 */
#define CONTROLLER_STAT1_EXTCHRG_STATZ	(1 << 7)
#define CONTROLLER_STAT1_LINCH_GATED	(1 << 6)
#define CONTROLLER_STAT1_CHRG_DET_N	(1 << 5)
#define CONTROLLER_STAT1_FAULT_WDG	(1 << 4)
#define CONTROLLER_STAT1_VAC_DET	(1 << 3)
#define VAC_DET	(1 << 3)
#define CONTROLLER_STAT1_VBUS_DET	(1 << 2)
#define VBUS_DET	(1 << 2)
#define CONTROLLER_STAT1_BAT_REMOVED	(1 << 1)
#define CONTROLLER_STAT1_BAT_TEMP_OVRANGE (1 << 0)
/* CHARGERUSB_INT_STATUS */
#define CURRENT_TERM_INT	(1 << 3)
#define CHARGERUSB_STAT		(1 << 2)
#define CHARGERUSB_THMREG	(1 << 1)
#define CHARGERUSB_FAULT	(1 << 0)
/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)
/* CHARGERUSB_STATUS_INT1 */
#define CHARGERUSB_STATUS_INT1_TMREG	(1 << 7)
#define CHARGERUSB_STATUS_INT1_NO_BAT	(1 << 6)
#define CHARGERUSB_STATUS_INT1_BST_OCP	(1 << 5)
#define CHARGERUSB_STATUS_INT1_TH_SHUTD	(1 << 4)
#define CHARGERUSB_STATUS_INT1_BAT_OVP	(1 << 3)
#define CHARGERUSB_STATUS_INT1_POOR_SRC	(1 << 2)
#define CHARGERUSB_STATUS_INT1_SLP_MODE	(1 << 1)
#define CHARGERUSB_STATUS_INT1_VBUS_OVP	(1 << 0)
/* CHARGERUSB_STATUS_INT2 */
#define ICCLOOP		(1 << 3)
#define CURRENT_TERM	(1 << 2)
#define CHARGE_DONE	(1 << 1)
#define ANTICOLLAPSE	(1 << 0)
/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT	(1 << 7)
#define OPA_MODE	(1 << 6)
#define HZ_MODE		(1 << 5)
#define TERM		(1 << 4)
/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400	(7 << 5)
/* CHARGERUSB_CTRL3 */
#define VBUSCHRG_LDO_OVRD	(1 << 7)
#define CHARGE_ONCE		(1 << 6)
#define BST_HW_PR_DIS		(1 << 5)
#define AUTOSUPPLY		(1 << 3)
#define BUCK_HSILIM		(1 << 0)
/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52		0x01
#define CHARGERUSB_VOREG_4P0		0x19
#define CHARGERUSB_VOREG_4P2		0x23
#define CHARGERUSB_VOREG_4P76		0x3F

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_300		0x0
#define CHARGERUSB_VICHRG_500		0x4
#define CHARGERUSB_VICHRG_1500		0xE

/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100	0x1
#define CHARGERUSB_CIN_LIMIT_300	0x5
#define CHARGERUSB_CIN_LIMIT_500	0x9
#define CHARGERUSB_CIN_LIMIT_NONE	0xF

/* CHARGERUSB_CTRLLIMIT1 */
#define VOREGL_4P16			0x21
#define VOREGL_4P56			0x35

/* CHARGERUSB_CTRLLIMIT2 */
#define CHARGERUSB_CTRLLIMIT2_1500	0x0E
#define		LOCK_LIMIT		(1 << 4)

/* ANTICOLLAPSE_CTRL2 */
#define BUCK_VTH_SHIFT			5

/* FG_REG_00 */
#define CC_ACTIVE_MODE_SHIFT	6
#define CC_AUTOCLEAR		(1 << 2)
#define CC_CAL_EN		(1 << 1)
#define CC_PAUSE		(1 << 0)

#define REG_TOGGLE1		0x90
#define FGDITHS			(1 << 7)
#define FGDITHR			(1 << 6)
#define FGS			(1 << 5)
#define FGR			(1 << 4)
#define GPADC_SAMP_WINDOW (1<<2)

#define PWDNSTATUS1		0x93
#define PWDNSTATUS2		0x94

#define REG_MISC1		0xE4
#define VAC_MEAS		0x04
#define VBAT_MEAS		0x02
#define BB_MEAS			0x01

#define REG_USB_VBUS_CTRL_SET	0x04
#define VBUS_MEAS		0x01
#define REG_USB_ID_CTRL_SET	0x06
#define ID_MEAS			0x01

#define BBSPOR_CFG			0xE6
#define		BB_CHG_EN		(1 << 3)

#define BB_SEL_MASK		(3 << 1)  /* Back-up battery end-of-charge voltage selection */
#define BB_SEL_3V0		(0 << 1)  /* 3.0V */
#define BB_SEL_2V5		(1 << 1)  /* 2.5V */
#define BB_SEL_3V15		(2 << 1)  /* 3.15V */
#define BB_SEL_VSYS		(3 << 1)  /* VSYS */

#define STS_HW_CONDITIONS	0x21
#define STS_USB_ID		(1 << 2)	/* Level status of USB ID */

/* PMC Master Register Map */
#define VSYSMIN_LO_THRESHOLD	0x04
#define VSYSMIN_HI_THRESHOLD	0x05
#define VBATMIN_HI_THRESHOLD	0x07
#define PRINT_LOG_PERIOD		5 /* print every (PRINT_LOG_PERIODx2)10 second period*/
#define bci_dbg_period(f...) do {if(!(log_counter%PRINT_LOG_PERIOD)) printk(KERN_INFO"[BCI]" f);}while(0)
#define bci_dbg(x,f...) do {if((debug_en&x)>0) printk(KERN_INFO"[BCI]" f);}while(0)



#define BCI_NOTIFIER(nh, val, v) if((val&registered_events)>0)blocking_notifier_call_chain(nh, val,v)
#define BCI_NOTIFIER_BQ(nh, val, v) if((val&registered_events_bq)>0)blocking_notifier_call_chain(nh, val,v)


/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};
static const unsigned int fuelgauge_time[4] = {250000, 62500, 15600, 3900};
static unsigned long log_counter = 0;

/*es1.0*/
#define RES_CURR_SENSOR_10 20
#define TEMP_RX_10 3
#define TEMP_RY_10 22
/*es1.1*/
#define RES_CURR_SENSOR_11 15
#define TEMP_RX_11 13
#define TEMP_RY_11 56




/*for low battery from 3.3V to 3.6V,
we should re-calculate loading table capacity */

#define EOD_COUNT 30
#define FULL_CONSUME 30*5
#define CAPACITY_THRESHOLD 15

#define IRQ_INFO_SIZE 10

#define START_FAKE_CHECK_TIMER 60
#define REGULATION_CHANGED_COUNT 15

#define CHARGER_PLUGOUT_DETECTING   0x01;

#define LOW_BAT_MAH 40 //  40/4400 < 1%

#define LAST_LOADING_DIFF (50)
#define _90_PERCENT_VOLT 4047
#define FULL_BATT_UAH (di->fg_full_uAh)
#define PERCENT_FACTOR (FULL_BATT_UAH/100)
#define USB_PLUG_IN_VOLT_OFFSET 100 /* usb plug in will increase battery voltage (mV) */
struct twl6030_irq_info {
    u8 charge_state;
    u8 hw_state;
    unsigned long time;
};


struct dynamic_charging_t {
	bool enable;
	bool stage_fixed;
	int stage;
	int previous_time;
};

struct gas_gauge_t {
	bool can_read;
	struct semaphore sem;
};

struct twl6030_bci_device_info {
    struct device		*dev;
    struct ntc_res_temp_desc *battery_tmp_tbl_603x;
    //unsigned int		tblsize;
    //int *battery_tmp_tbl;
    unsigned int tblsize;

    int			voltage_mV;
    int			sys_voltage_mV;
    int			bk_voltage_mV;
    int			current_uA; /* instantaneous current is the "average" current during the last 250ms! */
    int			current_avg_uA;
    int			current_suspend_uA;
    int			temp_C;
	volatile int charge_status;
    int			charge_prev_status;
    int			vac_priority;
    int			bat_health;
    int			charger_source;
    int			fuelgauge_mode;
    u8			usb_online;
    u8			ac_online;
    u8			stat1;
    u8			linear_stat;
    u8			status_int1;
    u8			status_int2;
    u8			watchdog_duration;
    u16			current_avg_interval;
    u16			monitoring_interval;
    unsigned int		min_vbus;
    unsigned int		max_charger_voltagemV;
    unsigned int		max_charger_currentmA;
    unsigned int		charger_incurrentmA;
    unsigned int		charger_outcurrentmA;
    unsigned int		regulation_voltagemV;
    unsigned int		low_bat_voltagemV;
    unsigned int		termination_currentmA;
	int 				battery_esr;
    unsigned long		usb_max_power;
    unsigned long		event;
    unsigned int		use_hw_charger;
    unsigned int		use_linear_charger;
    unsigned int		use_eeprom_config;
    unsigned int		power_path;

    unsigned int		capacity;
    unsigned int		capacity_debounce_count;
    unsigned int		ac_last_refresh;
    unsigned int		bci_probe_time;
    unsigned int		max_backupbat_voltagemV;
	unsigned int		no_main_battery;
	unsigned int		no_main_battery_monitoring_interval;

    int			fg_timer;
    int			fg_timer_prev;
    int			fg_timer_init;
    s32			fg_acc;
    s32			fg_acc_prev;
    s32			fg_acc_init;
    s16			fg_cc_offset;
    int			fg_timer_avg;
    s32			fg_acc_avg;
    int			fg_timer_suspend;
    s32			fg_acc_suspend;

    int			fg_full_uAh;
    int			fg_acc_uAh; /*accumulate uAh*/
    int			fg_init_acc_uAh;
    int			fg_full_eod;
    int			fg_full_consume;	//for capacity=1 and accumulate <0
    int			fg_full_charged;
    int			fg_remain_uAh;
    int			fg_reserved_uAh;
    int			fg_hidden_uAh; /* [di->fg_acc_uAh-uah];last - loading. less is better. Only initial in Init Capacity function(boot process). And Set zero at full and empty capacity. */
    int			fg_last_delta_uAh; /* [di->fg_acc_uAh-di->fg_remain_uAh]. less is better. Only initial in Discharge regression function, that means it used in discharging. */
    int			fg_chip_acc_uAh;
    int			fg_chip_acc_uAh_prev;
    int			fg_acc_delta; /*uAh, (fg_acc - fg_acc_prev)*(unit time second/hours)  that's consumption(-) or provided(+) uAh */
    int			fg_resistance;
    int			regressin_Voltage_mV;
    int			plugged_out_debounce_count; /* this purpose is to delay wake_unlock() for slowly enter suspend state*/
    struct power_supply	bat;
    struct power_supply	usb;
    struct power_supply	ac;
    struct power_supply	bk_bat;

    struct otg_transceiver	*otg;
    struct notifier_block	nb;
    struct work_struct	usb_work;

    struct delayed_work	bci_monitor_work;

    struct Loading_Table* loading_Table;
    struct wake_lock wakelock;
    /* timer for polling charger status */
    int			timer_msecs;
    struct work_struct	timer_work;
    struct timer_list	timer;
    spinlock_t		timer_lock;
    int			timer_on;
    struct blocking_notifier_head notifier_list_bq;
    struct blocking_notifier_head notifier_list;

    int			ctl_event;
    int			fault_event;
    int usb_ac_in_detection;
    int usb_ac_detection_result;
    unsigned long		features;


    int temp_Rx;
    int temp_Ry;

	struct twl6030_irq_info ctrl_irq[IRQ_INFO_SIZE];
    volatile int push_index;
    volatile int pop_index;
	struct mutex ctrl_irq_mutex;
    int charger_glitch_debounce;

#ifdef POWEROFF_CHARGE
    u32 boot_mode;
#endif

    bool bscreen_off;
#ifdef CONFIG_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif

    int bFlagMonitorStart;
    struct delayed_work	ir_handler_work;
	bool ac_charger_enable;
	bool bci_notifier_shutdown;
	struct dynamic_charging_t dynamic_charging;
	struct gas_gauge_t gas_gauge;
};

static struct wake_lock resume_wakelock;

bq_dataram bqdr;
extern u32 wakeup_timer_seconds;
static int debug_en = debug_level_common;
static bool watchdogflasg = false;
static unsigned int registered_events=0;
static unsigned int registered_events_bq=0;
static struct twl6030_bci_device_info *twl603x_bci=NULL;
static void twl6030_stop_ac_charger(struct twl6030_bci_device_info *di);
static void twl6030_config_extern_charger(struct twl6030_bci_device_info *di);
static int twl6030_bci_notifier_is_registered(int type);
static int twl6030backupbatt_setup(struct twl6030_bci_device_info *di, bool enable);
    
static void  twl6030_gg_capacity(struct twl6030_bci_device_info *di);
static void twl6030_config_extern_gg(struct twl6030_bci_device_info *di);
/*external gas gauge*/
static int twl6030_fg_calculate_uAh_from_capacity(struct twl6030_bci_device_info *di);
static void twl6030_print_gg_data(void) {
	bci_dbg(debug_level_G, "\n\n--== twl6030_print_gg_data ==--\n");
	bci_dbg(debug_level_G, "AtRate: %d\n",bqdr.AtRate);
	bci_dbg(debug_level_G, "AtRateTTE: %d\n",bqdr.AtRateTTE);
	bci_dbg(debug_level_G, "AvailEnergy: %d\n",bqdr.AvailEnergy);
	bci_dbg(debug_level_G, "AvgCurr: %d\n",bqdr.AvgCurr);
	bci_dbg(debug_level_G, "AvgPow: %d\n",bqdr.AvgPow);
	bci_dbg(debug_level_G, "Control: %d\n",bqdr.Control);
	bci_dbg(debug_level_G, "CycleCnt: %d\n",bqdr.CycleCnt);
	bci_dbg(debug_level_G, "Flags: %d\n",bqdr.Flags);
	bci_dbg(debug_level_G, "FullAvailCap: %d\n",bqdr.FullAvailCap);
	bci_dbg(debug_level_G, "FullChgCap: %d\n",bqdr.FullChgCap);
	bci_dbg(debug_level_G, "MaxLoadCurr: %d\n",bqdr.MaxLoadCurr);
	bci_dbg(debug_level_G, "MaxLoadTTE: %d\n",bqdr.MaxLoadTTE);
	bci_dbg(debug_level_G, "NomAvailCap: %d\n",bqdr.NomAvailCap);
	bci_dbg(debug_level_G, "RemCap: %d\n",bqdr.RemCap);
	bci_dbg(debug_level_G, "StateOfChg: %d\n",bqdr.StateOfChg);
	bci_dbg(debug_level_G, "StbyCurr: %d\n",bqdr.StbyCurr);
	bci_dbg(debug_level_G, "StbyTTE: %d\n",bqdr.StbyTTE);
	bci_dbg(debug_level_G, "Temp: %d\n",bqdr.Temp);
	bci_dbg(debug_level_G, "TTE: %d\n",bqdr.TTE);
	bci_dbg(debug_level_G, "TTEAtConstPow: %d\n",bqdr.TTEAtConstPow);
	bci_dbg(debug_level_G, "TTF: %d\n",bqdr.TTF);
	bci_dbg(debug_level_G, "Voltage: %d\n",bqdr.Voltage);
}
static void twl6030_update_gg_data(struct twl6030_bci_device_info *di) {
    if (&di->notifier_list_bq) {
        BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_BLOCK, &bqdr);
        twl6030_print_gg_data();

        //power off early
        if (bqdr.Voltage<GG_LOW_VOLT_TH_MV && bqdr.StateOfChg>0) {
            bci_dbg(debug_level_common, "[GG]--voltage = %d, cap = %d--\n", bqdr.Voltage, bqdr.StateOfChg);
            di->fg_full_eod++;
            if (di->fg_full_eod>=EOD_COUNT) {
                bci_dbg(debug_level_common, "[GG]--di->fg_full_eod = %d--\n", di->fg_full_eod);
                bqdr.StateOfChg = 0;
            }
        } else if (bqdr.Voltage>=GG_LOW_VOLT_TH_MV && di->fg_full_eod>0) {
            di->fg_full_eod = 0;
        }
    }
}
static void  twl6030_gg_capacity(struct twl6030_bci_device_info *di) {
    int prev_capacity=di->capacity;

    di->capacity = bqdr.StateOfChg;
    if ((di->charge_prev_status!= di->charge_status)||(prev_capacity!=di->capacity)) {
        if (!di->charger_glitch_debounce) {
            power_supply_changed(&di->bat);
        }
    }

#if 1

    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        di->charge_status=POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else if (di->charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {

    } else if (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
        if ((di->fg_full_charged)&&(di->capacity<99))
            di->fg_full_charged=0;
    } else  if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
        if ((di->fg_full_charged)&&(di->capacity<=99))
			di->fg_full_charged=0;

        if (di->fg_full_charged)
			di->charge_status = POWER_SUPPLY_STATUS_FULL;
		
        di->fg_full_eod=0;		
		if (di->capacity == 100) {
			di->charge_status = POWER_SUPPLY_STATUS_FULL;
			power_supply_changed(&di->bat);
		}
    }
    else	if (di->charge_status == POWER_SUPPLY_STATUS_FULL)
    {
        if (!di->fg_full_charged)di->fg_full_charged=1;
        di->fg_full_eod=0;
    }
#endif


    bci_dbg_period("twl6030_gg_capacity() - vol=%d, vsys=%d, cap=%d,tmp=%d,curr=%d, detect rst=%d, state %d <- %d, screen off = %d\n",
            bqdr.Voltage,di->sys_voltage_mV,bqdr.StateOfChg,di->temp_C,bqdr.AvgCurr, di->usb_ac_detection_result,di->charge_status,di->charge_prev_status, di->bscreen_off);

}

/*reset charge for external charger*/
static void twl603x_recharging(struct twl6030_bci_device_info *di) {

	if (di->no_main_battery)
		return;

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
        twl6030_stop_ac_charger(di);
        msleep(100);
        twl6030_config_extern_charger(di);
        BCI_NOTIFIER(&di->notifier_list, BCI_RESET_TIMER, NULL);
        BCI_NOTIFIER(&di->notifier_list, BCI_SET_START_CHARGING, NULL);

        twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                         CONTROLLER_CTRL1_EN_CHARGER |
                         CONTROLLER_CTRL1_SEL_CHARGER,
                         CONTROLLER_CTRL1);


        BCI_NOTIFIER(&di->notifier_list, BCI_SET_INPUT_CURRENT_LIMIT,&di->charger_incurrentmA);
    }

}

/*update capacity and print log*/
static void twl6030_store_capacity(struct twl6030_bci_device_info *di, int cap) {
    twl6030_write_last_capacity(cap);
    bci_dbg(debug_level_common, "twl6030_store_capacity()-vol=%d, vsys=%d, cap=%d,tmp=%d,curr=%d, detect rst=%d\n", di->voltage_mV,di->sys_voltage_mV,di->capacity,di->temp_C,di->current_uA/1000, di->usb_ac_detection_result);
}

/*push interrupt event and consume later*/
static int irq_handler_init(struct twl6030_bci_device_info *di) 
{
	mutex_lock(&di->ctrl_irq_mutex); 
    di->push_index=di->pop_index=0;
	mutex_unlock(&di->ctrl_irq_mutex); 
    return 0;
}
/*
* irq_handler_push() is push "CONTROLLER_STAT1" and "STS_HW_CONDITIONS" onto queses.
* Let twl6030charger_ctrl_event_handler() process them. Normally only ISR will call irq_handler_push(), but 
* sometimes will call it in some place to add registers onto queses.
*/
static int irq_handler_push(struct twl6030_bci_device_info *di) 
{
    int ret=0;
    if (di->bFlagMonitorStart == 0) {
        bci_dbg(debug_level_common, "bci monitor does not start, gas gauge doesn't ready, delay 500ms \n");
        return schedule_delayed_work_on(0,&di->ir_handler_work, msecs_to_jiffies(500));
    } else {
		mutex_lock(&di->ctrl_irq_mutex); 
        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &di->ctrl_irq[di->push_index].charge_state,CONTROLLER_STAT1);

        ret |= twl_i2c_read_u8(TWL6030_MODULE_ID0, &di->ctrl_irq[di->push_index].hw_state, STS_HW_CONDITIONS);

        di->ctrl_irq[di->push_index].time=jiffies;


        bci_dbg(debug_level_common, "[push data:%d] 0x%02x, 0x%02x \n",di->push_index,di->ctrl_irq[di->push_index].charge_state, di->ctrl_irq[di->push_index].hw_state);

        di->push_index = (++di->push_index)%IRQ_INFO_SIZE;

        di->ctl_event++;
		mutex_unlock(&di->ctrl_irq_mutex); 
        return 0;
    }
}
static void irq_handler_push_ex(struct work_struct *work)
{
    struct twl6030_bci_device_info *di = container_of(work,
                                                     struct twl6030_bci_device_info, ir_handler_work.work);
    int ret=0;
    if (di->bFlagMonitorStart == 0) {
        bci_dbg(debug_level_common, "bci monitor does not start, gas gauge doesn't ready, delay 500ms \n");
        schedule_delayed_work_on(0,&di->ir_handler_work, msecs_to_jiffies(500));
    } else {
		mutex_lock(&di->ctrl_irq_mutex);	    
        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &di->ctrl_irq[di->push_index].charge_state,CONTROLLER_STAT1);
        ret |= twl_i2c_read_u8(TWL6030_MODULE_ID0, &di->ctrl_irq[di->push_index].hw_state, STS_HW_CONDITIONS);
        di->ctrl_irq[di->push_index].time=jiffies;


        bci_dbg(debug_level_common, "[push data:%d] 0x%02x, 0x%02x \n",di->push_index,di->ctrl_irq[di->push_index].charge_state, di->ctrl_irq[di->push_index].hw_state);

        di->push_index = (++di->push_index)%IRQ_INFO_SIZE;

        di->ctl_event++;
		mutex_unlock(&di->ctrl_irq_mutex);			
    }
}

/*register/unregister for external charger and gas gauge */
static int twl6030_bci_notifier_is_registered(int type) {
    switch (type) {
    case NOTIFIER_REG_EXT_CHARGER:
        if (!twl603x_bci)return 0;
        else if (!twl603x_bci->notifier_list.head)return 0;
        break;
    case NOTIFIER_REG_EXT_FUELGAUGE:
        if (!twl603x_bci)return 0;
        else if (!twl603x_bci->notifier_list_bq.head)return 0;
        break;
    }

    return 1;
}
int twl6030_register_bci_notifier(struct notifier_block *nb,
                                  unsigned int events, int type)
{
    int ret=-1;
    u8 controller_stat;
    bci_dbg(debug_level_common, "603x register, type = %d\n", type);
    switch (type) {
    case NOTIFIER_REG_EXT_CHARGER:
        if (!twl603x_bci) {
            twl603x_bci = kzalloc(sizeof(*twl603x_bci), GFP_KERNEL);
            twl603x_bci->charge_status=POWER_SUPPLY_STATUS_UNKNOWN;
        }

        registered_events = 0x0;
        BLOCKING_INIT_NOTIFIER_HEAD(&twl603x_bci->notifier_list);
        ret= blocking_notifier_chain_register(&twl603x_bci->notifier_list, nb);
        bci_dbg(debug_level_common, "registered_events  = 0x%08x, events=0x%08x\n", registered_events, events);
        registered_events = registered_events | events;
        bci_dbg(debug_level_common, "registered_events  = 0x%08x\n", registered_events);
        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

        if ((controller_stat & VAC_DET)&&
                (twl603x_bci->charge_status==POWER_SUPPLY_STATUS_UNKNOWN)) {

            twl6030_stop_ac_charger(twl603x_bci);
        }
        twl6030_config_extern_charger(twl603x_bci);
        break;

    case NOTIFIER_REG_EXT_FUELGAUGE:
        if (!twl603x_bci) {
            bci_dbg(debug_level_common, "bci is null, alloc bci \n");
            twl603x_bci = kzalloc(sizeof(*twl603x_bci), GFP_KERNEL);

        }
        registered_events_bq= 0x0;

        BLOCKING_INIT_NOTIFIER_HEAD(&twl603x_bci->notifier_list_bq);

        ret= blocking_notifier_chain_register(&twl603x_bci->notifier_list_bq, nb);

        bci_dbg(debug_level_common, "registered_events_bq  = 0x%08x, events=0x%08x\n", registered_events_bq, events);
        registered_events_bq = registered_events_bq | events;
        bci_dbg(debug_level_common, "registered_events_bq  = 0x%08x\n", registered_events_bq);
		/* bq27541 gas gauge doesn't need to be initialized. */
		if (!machine_is_omap4_saga()) {
			twl6030_config_extern_gg(twl603x_bci);
		}
	
        break;

    default:
        break;
    }
	if (twl603x_bci)
		twl603x_bci->bci_notifier_shutdown = false;
    return ret;
}

EXPORT_SYMBOL_GPL(twl6030_register_bci_notifier);

int twl6030_unregister_bci_notifier(struct notifier_block *nb,
                                    unsigned int events, int type)
{
    int ret = 0;
    switch (type) {
    case NOTIFIER_REG_EXT_CHARGER:
        ret = blocking_notifier_chain_unregister(&twl603x_bci->notifier_list, nb);
        break;

    case NOTIFIER_REG_EXT_FUELGAUGE:
        ret = blocking_notifier_chain_unregister(&twl603x_bci->notifier_list_bq, nb);
        break;

    }
    return ret;
}
EXPORT_SYMBOL_GPL(twl6030_unregister_bci_notifier);

int twl6030_bci_notifier_shutdown(void)
{
	if (twl603x_bci)
		twl603x_bci->bci_notifier_shutdown = true;
    return 0;
}
EXPORT_SYMBOL_GPL(twl6030_bci_notifier_shutdown);

/*external charger information*/
static void print_external_charger_status(struct twl6030_bci_device_info *di) {
    struct bci_read read;
    //check external charger status
    BCI_NOTIFIER(&di->notifier_list, BCI_READ_FAULT_REASON, &read);
    switch (read.status) {
    case POWER_SUPPLY_STATUS_NOT_CHARGING:
        bci_dbg( debug_level_common,"!!bq24161: POWER_SUPPLY_STATUS_NOT_CHARGING\n");
        break;

    case POWER_SUPPLY_STATUS_DISCHARGING:
        bci_dbg( debug_level_common,"!!bq24161: POWER_SUPPLY_STATUS_DISCHARGING\n");
        break;

    case POWER_SUPPLY_STATUS_CHARGING:
        bci_dbg( debug_level_common,"!!bq24161: POWER_SUPPLY_STATUS_CHARGING\n");
        break;
    case POWER_SUPPLY_STATUS_UNKNOWN:
        bci_dbg( debug_level_common,"!!bq24161: POWER_SUPPLY_STATUS_UNKNOWN\n");
        break;
    case POWER_SUPPLY_STATUS_FULL:
        bci_dbg( debug_level_common,"!!bq24161: POWER_SUPPLY_STATUS_FULL\n");
        break;
    }
}

/*update status from external charger*/
static void update_ext_status(struct twl6030_bci_device_info *di) {
    if (!((di->features & TWL6032_SUBCLASS) && di->use_hw_charger)) {
        if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {//external registered, update charging status from external
            struct bci_read read;
            BCI_NOTIFIER(&di->notifier_list, BCI_READ_FAULT_REASON, &read);
            if (di->capacity==100 && read.status==POWER_SUPPLY_STATUS_CHARGING)
                di->charge_status =POWER_SUPPLY_STATUS_FULL;
            else
                di->charge_status = read.status;
            bci_dbg( debug_level_common,"!!bci status = %d, 161 status = %d\n", di->charge_status, read.status);

        }
    }
}


/*configuration of es1.0 and es1.1*/
static void chip_config(struct twl6030_bci_device_info *di) {
    u8 reg;
    twl_i2c_read_u8(TWL6030_MODULE_ID2, &reg,JTAGVERNUM);
    bci_dbg(debug_level_common, "chip_config() - JTAGVERNUM = %x\n",reg);
    switch (reg) {
    case 0://for es1.0, using old values
        di->fg_resistance=RES_CURR_SENSOR_10;
        di->temp_Rx = TEMP_RX_10;
        di->temp_Ry = TEMP_RY_10;
        break;
    case 1://for es1.1, using new values
        di->fg_resistance=RES_CURR_SENSOR_11;
        di->temp_Rx = TEMP_RX_11;
        di->temp_Ry = TEMP_RY_11;
        break;
    case 2://for es1.2, reuse es1.1 values
    default://apply these settings as default, just in case...
        di->fg_resistance=RES_CURR_SENSOR_11;
        di->temp_Rx = TEMP_RX_11;
        di->temp_Ry = TEMP_RY_11;
        break;
    }
    bci_dbg(debug_level_common, "chip_config() - res = %d, rx = %d, ry = %d \n", di->fg_resistance, di->temp_Rx, di->temp_Ry);
}

static void twl6030_kick_watchdog(struct twl6030_bci_device_info *di)
{

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, di->watchdog_duration , CONTROLLER_WDG);

}
static void twl6030_set_vbat_tracking_voltage(struct twl6030_bci_device_info *di,int vol)
{
    u8 value;

    vol=(vol-50)/50;
    if (vol>3)vol=3;
    if (vol<0)vol=0;
    twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_VSEL_COMP);
    value&=~(3<<5);
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value|(vol<<5),CONTROLLER_VSEL_COMP);


}
static void twl6030_enable_dppm(struct twl6030_bci_device_info *di,int enable)
{
    u8 value;


    twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_CTRL2);
    if (enable)value|=(EN_DPPM);
    else  value&=~(EN_DPPM);
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value,CONTROLLER_CTRL2);


}
/*
* twl6032_disable_power_path() is to disable USB interface VBUS input supply power path 
* not VAC external charger(bq24161).
*/
static void twl6032_disable_power_path(struct twl6030_bci_device_info *di)
{
	u8 value;
	bci_dbg(debug_level_common, "%s() \n", __func__);
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
	                CONTROLLER_CTRL1);
	
	/*VAC input supply path is selected, that means AC charger in and with ext. charger(bq24161)*/
	if(CONTROLLER_CTRL1_SEL_CHARGER&value){ 
		
		dev_warn(di->dev, 
			"[BCI]Disable Power Path, but external charger(bq24161) was selected \n");
		return;
	}
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
		0, 
		CONTROLLER_CTRL1); 
}
static void twl6032_enable_power_path(struct twl6030_bci_device_info *di)
{
	u8 value = 0;
	bci_dbg(debug_level_common, "%s() \n", __func__);
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
	                CONTROLLER_CTRL1);
	
	value |= (CONTROLLER_CTRL1_EN_LINCH | CONTROLLER_CTRL1_EN_CHARGER);		
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,value,CONTROLLER_CTRL1); 
}
static void twl6030_config_min_vbus_reg(struct twl6030_bci_device_info *di,
                                        unsigned int value)
{
    u8 rd_reg = 0;
    if (value > 4760 || value < 4200) {
        bci_dbg(debug_level_common, "invalid min vbus\n");
        return;
    }

    /* not required on TWL6032 */
    if (di->features & TWL6032_SUBCLASS)
        return;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, ANTICOLLAPSE_CTRL2);
    rd_reg = rd_reg & 0x1F;
    rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg, ANTICOLLAPSE_CTRL2);

    return;
}

static void twl6030_config_iterm_reg(struct twl6030_bci_device_info *di,
                                     unsigned int term_currentmA)
{
    if ((term_currentmA > 400) || (term_currentmA < 50)) {
        bci_dbg(debug_level_common, "invalid termination current\n");
        return;
    }

    term_currentmA = ((term_currentmA - 50)/50) << 5;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
                     CHARGERUSB_CTRL2);
    return;
}

static unsigned int twl6030_get_iterm_reg(struct twl6030_bci_device_info *di)
{
    unsigned int currentmA;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRL2);
    currentmA = 50 + (val >> 5) * 50;

    return currentmA;
}

static void twl6030_config_voreg_reg(struct twl6030_bci_device_info *di,
                                     unsigned int voltagemV)
{
    if ((voltagemV < 3500) || (voltagemV > 4760)) {
        bci_dbg(debug_level_common, "invalid charger_voltagemV\n");
        return;
    }

    voltagemV = (voltagemV - 3500) / 20;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
                     CHARGERUSB_VOREG);
    return;
}

static unsigned int twl6030_get_voreg_reg(struct twl6030_bci_device_info *di)
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_VOREG);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}

static void twl6030_config_vichrg_reg(struct twl6030_bci_device_info *di,
                                      unsigned int currentmA)
{
    //bci_dbg(debug_level_common, "set charging current as %d \n", currentmA);

    if (di->use_linear_charger) {//pop=1?
        if (currentmA<100)currentmA=100;
        if (currentmA>1500)currentmA=1500;
        currentmA = (currentmA - 100) / 100;
    } else {//pop=0?
        if ((currentmA >= 300) && (currentmA <= 450))
            currentmA = (currentmA - 300) / 50;
        else if ((currentmA >= 500) && (currentmA <= 1500))
            currentmA = (currentmA - 500) / 100 + 4;
        else {
            bci_dbg(debug_level_common, "invalid charger_currentmA\n");
            return;
        }
    }

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_VICHRG);
    return;
}

static const int vichrg[] = {
    300, 350, 400, 450, 500, 600, 700, 800,
    900, 1000, 1100, 1200, 1300, 1400, 1500, 300
};

static void twl6030_set_ci_limit(struct twl6030_bci_device_info *di,
                                 unsigned int currentmA)
{
    //bci_dbg(debug_level_common, "set input current as %d \n", currentmA);

    if ((currentmA >= 50) && (currentmA <= 750))
        currentmA = (currentmA - 50) / 50;
    else if (currentmA >= 750) {
        if (di->features & TWL6032_SUBCLASS) {
            if (currentmA>2250)currentmA=0x1F;
            else if (currentmA==2250)currentmA=0x22;
            else  if (currentmA>2100)currentmA=0x21;
            else  if (currentmA>1800)currentmA=0x20;
            else  if (currentmA>1500)currentmA=0x2E;
            else  currentmA = (currentmA % 100) ? 0x30 : 0x20 + (currentmA - 100) / 100;
        } else
            currentmA = (800 - 50) / 50;
    } else {
        bci_dbg(debug_level_common, "invalid input current limit\n");
        return;
    }

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_CINLIMIT);
    return;
}



/*
 * set System supply/battery regulation voltage limit:
 */ 
static void twl6030_set_vo_hard_limit(struct twl6030_bci_device_info *di,
                                      unsigned int voltagemV)
{
    if ((voltagemV < 3500) || (voltagemV > 4760)) {
        bci_dbg(debug_level_common, "invalid max_charger_voltagemV\n");
        return;
    }

    voltagemV = (voltagemV - 3500) / 20;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
                     CHARGERUSB_CTRLLIMIT1);
    return;
}

static unsigned int twl6030_get_vo_hard_limit(struct twl6030_bci_device_info *di)
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT1);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}

static void twl6030_set_co_hard_limit(struct twl6030_bci_device_info *di,
                                      unsigned int currentmA)
{
    if (currentmA<100)currentmA=100;
    if (currentmA>1500)currentmA=1500;
    currentmA = (currentmA - 100) / 100;

    currentmA |= LOCK_LIMIT;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_CTRLLIMIT2);
    return;
}

static unsigned int twl6030_get_co_hard_limit(struct twl6030_bci_device_info *di)
{
    unsigned int currentmA;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT2);
    currentmA = vichrg[val & 0xF];

    return currentmA;
}
static void twl6030_config_extern_gg(struct twl6030_bci_device_info *di){
    //int thres_vol = 3800;
    int soc1 = LOW_BAT_MAH;

    // update terminate voltage
    BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_SET_TERM_VOLT, &di->low_bat_voltagemV); 

    // update system down volt	
    //BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_SET_SYSDN_VOLT, &thres_vol); 	

    // update low bat mah interrupt
    BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_SET_SOC1, &soc1);

}
static void twl6030_config_extern_charger(struct twl6030_bci_device_info *di)
{
    int curr=500;
    int eoc = 50;

    twl6030_set_vo_hard_limit(di, di->max_charger_voltagemV);
    twl6030_config_voreg_reg(di, di->regulation_voltagemV);

    BCI_NOTIFIER(&di->notifier_list, BCI_RESET, NULL);
    msleep(100);
    //BCI_NOTIFIER(&di->notifier_list, BCI_SET_EOC_CURRENT, &di->termination_currentmA);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_EOC_CURRENT, &eoc);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_INPUT_CURRENT_LIMIT, &curr);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_CHARGE_CURRENT,&di->max_charger_currentmA);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_REGULATION_VOLTAGE, &di->regulation_voltagemV);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_INPUT_VOLTAGE_LIMIT, &di->max_charger_voltagemV);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_EOC_DISABLE,NULL);

    return;
}
static int  resistance_to_temp(struct ntc_res_temp_desc *ntc_table,int  res)
{
    int ret_temp,i;


    for (i=0;i<ntc_table->res;i++) {
        if (res>=ntc_table[i].res) {
            ret_temp=(res-ntc_table[i].res)*(ntc_table[i-1].temp-ntc_table[i].temp)*10/(ntc_table[i-1].res-ntc_table[i].res)
                     +ntc_table[i].temp*10;
            return ret_temp;
        }
    }
    return -EINVAL;
}
static int loadingVoltToCap(struct Loading_Table *index,int  currentNow, int voltageNow)
{
    int j,current1,current2;
    int res;
    struct batt_volt_cap_desc newArray[BATT_VOLT_CAP_SIZE];
    struct batt_volt_cap_desc *table1,*table2;

    if (currentNow>0)currentNow=0;

    {
        voltageNow+=(currentNow*twl603x_bci->battery_esr/1000);		
        for (j=0;((index[j+1].batt_volt_cap!=NULL)&&(index[j].curr>=currentNow));j++);

        current1=index[j-1].curr;
        current2=index[j].curr;
        table1=index[j-1].batt_volt_cap;
        table2=index[j].batt_volt_cap;


        for (j=0;j<BATT_VOLT_CAP_SIZE;j++)
        {

            int a=(table2[j].volt-table1[j].volt)*100/(current2-current1);
            int b=(current2*table1[j].volt-current1*table2[j].volt)*100/(current2-current1);
            newArray[j].cap=table1[j].cap;
            newArray[j].volt=(currentNow*a+b)/100;
        }


        for (j=0;j<BATT_VOLT_CAP_SIZE;j++)
        {
            if (newArray[j].volt==voltageNow)
            {
                res = newArray[j].cap;
                return res;
            }
            if (newArray[j].volt > voltageNow)
                break;
        }
        if (j == 0)		res = newArray[j].cap;
        else if (j == BATT_VOLT_CAP_SIZE)	res = newArray[j-1].cap;
        else
        {
            res = ((voltageNow-newArray[j-1].volt)*
                   (newArray[j].cap-newArray[j-1].cap));
            res = res / (newArray[j].volt-newArray[j-1].volt);
            res += newArray[j-1].cap;
        }

    }
    if (res<0)res=0;
    if (res>100)res=100;
    return res;
}
static int twl6030_fg_avg_sample_current_uA(struct twl6030_bci_device_info *di,
        int delta_acc,int delta_counter ) {

    int cc_offset= di->fg_cc_offset * delta_counter;
    int factor=((62000000/di->fg_resistance)*fuelgauge_rate[di->fuelgauge_mode]/delta_counter)>>15;
    //bci_dbg(debug_level_common, "delta_acc = %d, offset = %d, factor = %d, \n",delta_acc,  cc_offset, factor);
    return (delta_acc-cc_offset)*factor;
}

/*
* twl6030_fg_update() is mainly to update below status
* current_avg_uA -> average current
* fg_acc_delta -> consumption mAh, twl6030_fg_avg_sample_current_uA()*hours
* current_uA -> instantaneous current
*/
static int twl6030_fg_update(struct twl6030_bci_device_info *di )
{
    s32 samples = 0,samples_avg=0;
    s16 cc_offset = 0;//,cc_offset_avg=0;
    int time_interval;
    int ret = 0;
    u16 read_value = 0;

    /* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
    ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
                       FG_REG_10, 2);
    if (ret < 0) {
        bci_dbg( debug_level_common,"failed to read FG_REG_10: current_now\n");
        return -1;
    }


    di->fg_acc_prev = di->fg_acc;
    di->fg_timer_prev = di->fg_timer;

    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->fg_timer,
                 FG_REG_01, 3);
    /*
     * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
     * accumulates instantaneous current value
     */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->fg_acc,
                 FG_REG_04, 4);
    /* FG_REG_08, 09 is 10 bit signed calibration offset value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &cc_offset,
                 FG_REG_08, 2);
    di->fg_cc_offset= ((s16)(cc_offset << 6) >> 6);

    /*update avg current*/
    samples_avg = di->fg_timer - di->fg_timer_avg;
    /* check for timer overflow */
    if (di->fg_timer < di->fg_timer_avg)
        samples_avg = samples_avg + (1 << 24);
	
	/* calcualte evey current_avg_interval=2 second*/
    if (samples_avg/fuelgauge_rate[di->fuelgauge_mode]>=di->current_avg_interval) {

        di->current_avg_uA =twl6030_fg_avg_sample_current_uA(di,
                            di->fg_acc - di->fg_acc_avg,samples_avg);

        di->fg_timer_avg=di->fg_timer;
        di->fg_acc_avg=di->fg_acc;

    }

    /*update Acc*/
    samples = di->fg_timer - di->fg_timer_prev;
    if (samples) {
        /* check for timer overflow */
        if (di->fg_timer < di->fg_timer_prev)
            samples = samples + (1 << 24);

        time_interval=samples/fuelgauge_rate[di->fuelgauge_mode];
		if (time_interval >= 1) { /* calculate once at least 1 second*/
	        di->fg_acc_delta =twl6030_fg_avg_sample_current_uA(di,
	                          di->fg_acc - di->fg_acc_prev,samples)*time_interval/3600;
	        bci_dbg(debug_level_C ,"di->fg_acc_delta = %d, di->fg_acc=%d, di->fg_acc_prev= %d\n", di->fg_acc_delta, di->fg_acc, di->fg_acc_prev);
		}
		
    }

    /*update current now*/
    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
        di->current_uA = bqdr.AvgCurr*1000;
    } else {

        di->current_uA = twl6030_fg_avg_sample_current_uA(di,
                         (s16)(read_value << 2) >> 2,1);
    }

    bci_dbg(debug_level_A,"curr=%dmA, avg_curr=%d mA,acc_delta=%d uAh\n",
            di->current_uA/1000,di->current_avg_uA/1000,di->fg_acc_delta);
    bci_dbg(debug_level_A,"chip=%d, chipTime=%d sec\n",
            di->fg_acc,(di->fg_timer/fuelgauge_rate[di->fuelgauge_mode])&0x0fffffff);


    return 0;
}

static void twl6030_fg_mode_changed(struct twl6030_bci_device_info *di)
{
    int timer,acc;
    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &timer,
                 FG_REG_01, 3);
    /*
     * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
     * accumulates instantaneous current value
     */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &acc,
                 FG_REG_04, 4);
    di->fg_timer_prev=timer;
    di->fg_timer=timer;
    di->fg_acc=acc;
    di->fg_acc_prev=acc;
    di->fg_timer_avg=timer;
    di->fg_acc_avg=acc;

}
static void twl6030_fg_set_mode(struct twl6030_bci_device_info *di,
                                int clean, int pause, int sample_mode,int auto_cal)
{
    u8 value=0;
    twl6030_fg_update(di);
    if ((sample_mode<4)&&(sample_mode>=0)) {
        value=sample_mode<<CC_ACTIVE_MODE_SHIFT;
        di->fuelgauge_mode=sample_mode;
    } else value=di->fuelgauge_mode<<CC_ACTIVE_MODE_SHIFT;

    if (clean) value|=CC_AUTOCLEAR;
    if (auto_cal) value|=CC_CAL_EN;
    if (pause) value|=CC_PAUSE;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, value,FG_REG_00);
    twl6030_fg_mode_changed(di);
    return;
}

/*
 * twl6030_fg_sync() to reset(CC_AUTOCLEAR) timer and acc when current is changing from positive to negative or
 * negative to positive. ex: charging -> discharging or discharging -> charging.
 */
static void twl6030_fg_sync(struct twl6030_bci_device_info *di)
{
    int acc;
    int factor= fuelgauge_rate[di->fuelgauge_mode]*3600;
	bci_dbg(debug_level_common, "%s()\n",__func__);
	di->fg_acc_uAh = twl6030_fg_calculate_uAh_from_capacity(di);
    acc=((di->fg_acc_uAh/125)<<6)*factor/(375);
    bci_dbg(debug_level_common, "Sync CAR: %d uAh, acc=0x%08x\n",di->fg_acc_uAh,acc);
    twl6030_fg_set_mode(di,1,0,-1,0);
    return;
}


static int twl6030_fg_calculate_capacity(struct twl6030_bci_device_info *di)
{
    int full_uAh = di->fg_full_uAh;
    int R= full_uAh/200;// full/100=>1%, full/200=>0.5%, for round off
    int capacity = ((di->fg_acc_uAh + di->fg_hidden_uAh - di->fg_reserved_uAh +R ) *100)/ (full_uAh - di->fg_reserved_uAh);

    if (capacity >= 100) capacity= 100;
    if (capacity <= 1)	capacity= 1;


    if (di->fg_full_eod>=EOD_COUNT) {//touch low voltage over 30 times for all capacity
        bci_dbg(debug_level_common, "-------=======  eod>=%d, return capacity 0 =======-------- \n", EOD_COUNT);
        capacity= 0;

    } else if (di->fg_full_eod>=5 && di->capacity==1) {//touch low voltage over 5 times for capacity as 1
        bci_dbg(debug_level_common, "-------=======  eod>=5 & cap = 1, return capacity 0 =======-------- \n");
        capacity= 0;

    } else if (di->fg_full_consume>= FULL_CONSUME) {
        bci_dbg(debug_level_common, "-------=======  full consume>=%d, return capacity 0 =======-------- \n", FULL_CONSUME);
        capacity= 0;
    }
    return capacity;
}
static int twl6030_fg_calculate_uAh_from_capacity(struct twl6030_bci_device_info *di)
{
    return (di->capacity*(di->fg_full_uAh - di->fg_reserved_uAh)/100)+
           di->fg_reserved_uAh-di->fg_hidden_uAh;
}
static void twl6030_fg_discharge_regression(struct twl6030_bci_device_info *di)
{
    if ((di->voltage_mV> di->low_bat_voltagemV)&&(di->fg_full_eod<EOD_COUNT)) {
        int		cap1, cap2;

        bci_dbg(debug_level_C ,"\n==> \ndi->voltage_mV = %d, di->regressin_Voltage_mV=%d, di->current_uA=%d\n", di->voltage_mV, di->regressin_Voltage_mV, di->current_uA);
        if (di->voltage_mV <di->regressin_Voltage_mV) {/*Only lower then some voltage*/
            cap1=loadingVoltToCap(di->loading_Table,di->current_uA/1000,di->voltage_mV);
            cap2=loadingVoltToCap(di->loading_Table,di->current_uA/1000,di->low_bat_voltagemV);
            bci_dbg(debug_level_C ,"cap1 = %d, cap2=%d, di->fg_reserved_uAh = %d\n", cap1, cap2, di->fg_reserved_uAh);
            di->fg_remain_uAh = di->fg_reserved_uAh+ (cap1 - cap2)* di->fg_full_uAh / 100;
            bci_dbg(debug_level_C ,"di->fg_remain_uAh = %d (%d%%)\n", di->fg_remain_uAh,di->fg_remain_uAh/PERCENT_FACTOR);
        } else {
            di->fg_remain_uAh=di->fg_acc_uAh;
            bci_dbg(debug_level_C ,"di->fg_remain_uAh = %d (%d%%)\n", di->fg_remain_uAh,di->fg_remain_uAh/PERCENT_FACTOR);
        }

        di->fg_last_delta_uAh=di->fg_acc_uAh-di->fg_remain_uAh;
        bci_dbg(debug_level_C ,"di->fg_last_delta_uAh = %d (%d%%)\n", di->fg_last_delta_uAh,di->fg_last_delta_uAh/PERCENT_FACTOR);

        bci_dbg(debug_level_C ,"di->fg_hidden_uAh = %d (%d%%)\n", di->fg_hidden_uAh,di->fg_hidden_uAh/PERCENT_FACTOR);
        if (((di->fg_hidden_uAh<0)&&(di->fg_last_delta_uAh>0))||
                ((di->fg_hidden_uAh>0)&&(di->fg_last_delta_uAh<0))) {

            di->fg_hidden_uAh+=di->fg_last_delta_uAh;
            di->fg_acc_uAh-=di->fg_last_delta_uAh;
        }
        else if (((di->fg_hidden_uAh<0)&&(di->fg_last_delta_uAh<0))||
                 ((di->fg_hidden_uAh>0)&&(di->fg_last_delta_uAh>0))) {

            di->fg_hidden_uAh-=di->fg_last_delta_uAh;
            di->fg_acc_uAh+=di->fg_last_delta_uAh;
        }

        bci_dbg(debug_level_C ,"di->fg_acc_uAh = %d (%d%%), di->fg_remain_uAh=%d (%d%%), di->fg_acc_delta=%d\n", di->fg_acc_uAh,di->fg_acc_uAh/PERCENT_FACTOR, di->fg_remain_uAh,di->fg_remain_uAh/PERCENT_FACTOR, di->fg_acc_delta);
        if (di->fg_acc_uAh > di->fg_remain_uAh) {
            if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) { //with 24161
                if (di->capacity<=10) {
                    bci_dbg(debug_level_C ,"dis->1.8\n");
                    di->fg_acc_uAh += (int)((180*di->fg_acc_delta)/100);

                } else if (di->capacity<=20) {
                    bci_dbg(debug_level_C ,"dis->1.8\n");
                    di->fg_acc_uAh += (int)((180*di->fg_acc_delta)/100);

                } else if (di->capacity<=30) {
                    bci_dbg(debug_level_C ,"dis->2.5\n");
                    di->fg_acc_uAh += (int)((250*di->fg_acc_delta)/100);

                } else if (di->capacity<=40) {
                    bci_dbg(debug_level_C ,"dis->1.4\n");
                    di->fg_acc_uAh += (int)((140*di->fg_acc_delta)/100);

                } else if (di->capacity<=50) {
                    bci_dbg(debug_level_C ,"dis->1.3\n");
                    di->fg_acc_uAh += (int)((130*di->fg_acc_delta)/100);

                } else if (di->capacity<=60) {
                    bci_dbg(debug_level_C ,"dis->1.2\n");
                    di->fg_acc_uAh += (int)((120*di->fg_acc_delta)/100);

                } else if (di->capacity<=70) {
                    bci_dbg(debug_level_C ,"dis->1.2\n");
                    di->fg_acc_uAh += (int)((120*di->fg_acc_delta)/100);

                } else if (di->capacity<=80) {
                    bci_dbg(debug_level_C ,"dis->1.1\n");
                    di->fg_acc_uAh += (int)((110*di->fg_acc_delta)/100);

                } else if (di->capacity<=90) {
                    bci_dbg(debug_level_C ,"dis->1.1\n");
                    di->fg_acc_uAh += (int)((110*di->fg_acc_delta)/100);

                } else if (di->capacity<=100) {
                    bci_dbg(debug_level_C ,"dis->1\n");
                    di->fg_acc_uAh += (int)((100*di->fg_acc_delta)/100);
                }
            } else { //without 24161
                if (di->capacity<=10) {
                    bci_dbg(debug_level_C ,"dis->1.8\n");
                    di->fg_acc_uAh += (int)((180*di->fg_acc_delta)/100);

                } else if (di->capacity<=20) {
                    bci_dbg(debug_level_C ,"dis->1.8\n");
                    di->fg_acc_uAh += (int)((180*di->fg_acc_delta)/100);

                } else if (di->capacity<=30) {
                    bci_dbg(debug_level_C ,"dis->1.7\n");
                    di->fg_acc_uAh += (int)((170*di->fg_acc_delta)/100);

                } else if (di->capacity<=40) {
                    bci_dbg(debug_level_C ,"dis->1.6\n");
                    di->fg_acc_uAh += (int)((160*di->fg_acc_delta)/100);

                } else if (di->capacity<=50) {
                    bci_dbg(debug_level_C ,"dis->1.5\n");
                    di->fg_acc_uAh += (int)((150*di->fg_acc_delta)/100);

                } else if (di->capacity<=60) {
                    bci_dbg(debug_level_C ,"dis->1.4\n");
                    di->fg_acc_uAh += (int)((140*di->fg_acc_delta)/100);

                } else if (di->capacity<=70) {
                    bci_dbg(debug_level_C ,"dis->1.2\n");
                    di->fg_acc_uAh += (int)((120*di->fg_acc_delta)/100);

                } else if (di->capacity<=80) {
                    bci_dbg(debug_level_C ,"dis->1.1\n");
                    di->fg_acc_uAh += (int)((110*di->fg_acc_delta)/100);

                } else if (di->capacity<=90) {
                    bci_dbg(debug_level_C ,"dis->1.1\n");
                    di->fg_acc_uAh += (int)((110*di->fg_acc_delta)/100);

                } else if (di->capacity<=100) {
                    bci_dbg(debug_level_C ,"dis->1\n");
                    di->fg_acc_uAh += (int)((100*di->fg_acc_delta)/100);
                }
            }


        } else if (di->fg_remain_uAh > di->fg_acc_uAh + di->fg_full_uAh/100) {
            /*Skip*/
			bci_dbg(debug_level_C ,"Skip. remain=%duAh,acc=%duAh\n",di->fg_remain_uAh,di->fg_acc_uAh);
        } else {
			bci_dbg(debug_level_C ,"Sync. remain=%duAh,acc=%duAh\n",di->fg_remain_uAh,di->fg_acc_uAh);        
            di->fg_acc_uAh += di->fg_acc_delta;
        }
		
		if (di->capacity) {
			int cap_uAh = di->fg_full_uAh*di->capacity/100;
			bci_dbg(debug_level_C ,"extra consumed %duAh\n",
			(int)((di->fg_acc_delta/100)*((int)abs(cap_uAh-di->fg_remain_uAh)*100/cap_uAh)));		  

			di->fg_acc_uAh += (int)((di->fg_acc_delta/100)*((int)abs(cap_uAh-di->fg_remain_uAh)*100/cap_uAh));
		}
        di->fg_full_eod=0;
    }
    else { // < bat low voltage

        int delta=di->fg_full_uAh/2000; // 0.05 %
        di->fg_acc_uAh -= delta;

        di->fg_full_eod++;
        if (di->fg_full_eod>=EOD_COUNT) {
            di->fg_acc_uAh=di->fg_reserved_uAh;
            di->fg_hidden_uAh=0;
            bci_dbg(debug_level_common, "EOD: curr=%d, volt=%d temp=%d, eod = %d\n", di->current_uA,di->voltage_mV,di->temp_C, di->fg_full_eod);
        }

        bci_dbg(debug_level_common, "eod = %d , acc will - 1/2000 full capacity\n",di->fg_full_eod);
    }


    //if accumulate <0 and capacity = 1, let eod++ to make better user feeling when cap = 1
    if (di->fg_acc_uAh<0 && di->capacity==1) {
        di->fg_full_consume++;
        bci_dbg(debug_level_common, "accumulate<0 and capacity = 1, to make better user feeling fg_full_consume = %d \n",di->fg_full_consume);

        if (di->fg_full_consume>= FULL_CONSUME) {
            di->fg_acc_uAh=di->fg_reserved_uAh;
            di->fg_hidden_uAh=0;
        }
    }

    bci_dbg(debug_level_C ,"<==\n");
}
static void twl6030_fg_charge_process(struct twl6030_bci_device_info *di)
{
    int charger_incurrentuA=di->charger_incurrentmA*1000;
    int compensate_voltmV = _90_PERCENT_VOLT+USB_PLUG_IN_VOLT_OFFSET; /*the voltage which begins to compensate */
	di->fg_acc_uAh += di->fg_acc_delta; 		
    bci_dbg(debug_level_A ,"\n==> \ndi->current_uA = %d, charger_incurrentuA=%d\n", di->current_uA, charger_incurrentuA);
    if ((di->current_uA>0)&&(charger_incurrentuA>di->current_uA)) {

        int delta_samples=di->fg_timer-di->fg_timer_prev,delta_time_sec,delta_uAh;
        if (di->fg_timer < di->fg_timer_prev)
            delta_samples = delta_samples + (1 << 24);

        delta_time_sec=delta_samples/fuelgauge_rate[di->fuelgauge_mode];
		/* delta_uAh is to fetch up di->current_uA not arrive charger_incurrentuA */
        delta_uAh=(delta_time_sec*(charger_incurrentuA-di->current_uA))/3600;
        bci_dbg(debug_level_A ,"di->fg_hidden_uAh = %d\n", di->fg_hidden_uAh);
        bci_dbg(debug_level_A ,"di->voltage_mV = %d, di->regulation_voltagemV=%d\n", di->voltage_mV, di->regulation_voltagemV);

        // 1. consume hidden 
        if (di->fg_hidden_uAh != 0) {
			if (di->fg_hidden_uAh < 0) {
	            di->fg_hidden_uAh+=delta_uAh;
	            di->fg_acc_uAh-=delta_uAh;
	            if (di->fg_hidden_uAh>0)di->fg_hidden_uAh=0;
			} else {
                di->fg_hidden_uAh-=delta_uAh;
	            di->fg_acc_uAh+=delta_uAh;				
                if (di->fg_hidden_uAh<0)di->fg_hidden_uAh=0;
	        }
			bci_dbg(debug_level_A,"Release hidden:%d uAh, hidden=%d mAh\n",delta_uAh,di->fg_hidden_uAh/1000);
			bci_dbg(debug_level_A,"Acc=%d uAh\n",di->fg_acc_uAh);
        } 
        // 2. bat voltage > compensate_voltmV means  begining to compensate.
        else if  (di->voltage_mV >= compensate_voltmV) {
            int termination_currentuA=di->termination_currentmA*1000;
            int factor=0,  compensate_delta_uAh=0;
            int scale=(charger_incurrentuA-termination_currentuA)>>2;
            int sec1 = 90, sec2 = 95, sec3 = 97, sec4 = 99;
			bci_dbg(debug_level_A,"compensate=%dmV begins\n",compensate_voltmV);
            bci_dbg(debug_level_A ,"di->current_uA = %d, termination_currentmA=%d\n", di->current_uA, termination_currentuA);
            // a. current >= termination
            if (di->current_uA >= termination_currentuA) { //curr>300ma
                bci_dbg(debug_level_A ,"charger_incurrentuA = %d, termination_currentmA=%d, scale = %d\n", charger_incurrentuA, termination_currentuA, scale);

                if (di->current_uA>=termination_currentuA+scale*3) { // > 1425mA
                    /*The current should be 3<current<4, if lower the 3, need compensate*/
					bci_dbg(debug_level_A ,"> 1425mA stage,curr=%d\n",di->current_uA);					
                    if (di->capacity<sec1) {
                        factor=4;
                    }
                } else if (di->current_uA>=termination_currentuA+scale*2) { //1050mA ~ 1425mA
					bci_dbg(debug_level_A ,"1050mA ~ 1425mA stage,curr=%d\n",di->current_uA);					
                    if (di->capacity<sec2) factor=3;
                    if (di->capacity<sec1)factor++;

                } else if (di->current_uA>=termination_currentuA+scale) { //675mA ~ 1050mA
                    int currmA = di->current_uA/1000;
					bci_dbg(debug_level_A ,"675mA ~ 1050mA stage,curr=%d\n",di->current_uA);					
                    if (di->capacity<70) {// <70
                        if (currmA<=980) {

                            factor=10;
                        }

                    } else if (di->capacity<75) {// <75
                        if (currmA<=930) {

                            factor=10;
                        }


                    } else if (di->capacity<80) { //<80
                        if (currmA<=880) {

                            factor=10;
                        }

                    } else if (di->capacity<85) { //<85
                        if (currmA<=820) {

                            factor=10;
                        }
                    } else { //<90
                        if (currmA<=760) {

                            factor=10;
                        }
                    }

                } else if ((di->current_uA>termination_currentuA)) { //300mA ~ 675mA

                    int currmA = di->current_uA/1000;
					bci_dbg(debug_level_A ,"300mA ~ 675mA stage,curr=%d\n",di->current_uA);					
                    if (di->capacity<70) {// <70

                        factor=14;

                    } else if (di->capacity<75) {// <75

                        factor=12;

                    } else if (di->capacity<80) { //<80

                        factor=12;
                    } else if (di->capacity<85) { //<85

                        factor=12;

                    } else  if (di->capacity<sec1) {// <90
                        if (currmA<=650) {

                            factor=10;
                        }
                    } else if (di->capacity<sec2) { //90~95
                        if (currmA<=600) {

                            factor=10;
                        }

                    } else if (di->capacity<sec3) { //95~96
                        if (currmA<=550) {

                            factor=8;
                        }
                    }
                    //97,98,99,100 normal run



                }

                if (di->fg_acc_uAh>di->fg_full_uAh) {
                    di->fg_acc_uAh=di->fg_full_uAh-1000;
                }
            }

            // b. current < termination , curr<300ma
            else {
                if (di->capacity==100) {
                    di->charge_status =POWER_SUPPLY_STATUS_FULL;
                    factor=0;
                } else {
                    factor=2;
                    if (di->capacity<sec4) factor++;
                    if (di->capacity<sec3) factor+=2;
                    if (di->capacity<sec2)factor+=3;
                    if (di->capacity<sec1)factor+=4;
                }
            }

            if (factor)
                compensate_delta_uAh=(delta_time_sec*(termination_currentuA+scale*factor*100/100-di->current_uA))/3600;

            di->fg_acc_uAh+=compensate_delta_uAh;
            bci_dbg(debug_level_A,"Compensate:%d uAh Acc=%d uAh\n",compensate_delta_uAh,di->fg_acc_uAh);
        }
    }

	if (di->fg_acc_uAh>di->fg_full_uAh) {
		di->fg_acc_uAh=di->fg_full_uAh;
		di->fg_hidden_uAh = 0;
		di->fg_last_delta_uAh = 0;
		di->charge_status = POWER_SUPPLY_STATUS_FULL;
		power_supply_changed(&di->bat);
	}

    bci_dbg(debug_level_A ,"<== \n");
}


static void twl6030_fg_calculate_init_capacity(struct twl6030_bci_device_info *di) {
    /*
    ICOM_DEVOFF_BCK	 (1 << 2)	// DEVOFF_BCK in PHOENIX_LAST_TURNOFF_STS
    ICOM_RESTART_BB	 (1 << 1)	// RESTART_BB in PHOENIX_START_CONDITION
    ICOM_FIRST_SYS_INS	(1 << 0)	// FIRST_SYS_INS in PHOENIX_START_CONDITION
    */
    u8 bat_insert = twl6030_read_bat_insert();
//	twl6032_disable_power_path(di);
    bci_dbg(debug_level_common, "----==== bat insert = %d \n", bat_insert);
    bci_dbg(debug_level_common,"twl6030_fg_capacity Init !!volt %dmV, current %duA, last_capacity=%d\n",
            di->voltage_mV,di->current_uA,di->capacity);

    if (di->capacity==0 ||bat_insert>0) {
        di->capacity=loadingVoltToCap(di->loading_Table,di->current_uA/1000,di->voltage_mV);
        di->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(di);
		bci_dbg(debug_level_common,"use loading table as init capacity=%d,line=%d\n",di->capacity,__LINE__);

    } else { //last >0, use last capacity , not read table
        int uah,cap=loadingVoltToCap(di->loading_Table,di->current_uA/1000,di->voltage_mV); //loading table for cap
        int pre_cap = di->capacity; //keep last capacity
		if (abs(pre_cap - cap) < LAST_LOADING_DIFF) { /* last - loading is less than LAST_LOADING_DIFF %*/
			bci_dbg(debug_level_common,"use stored value as init capacity=%d,diff=%d\n",di->capacity,(int)abs(pre_cap - cap));
			
	        di->capacity=cap;
	        uah=twl6030_fg_calculate_uAh_from_capacity(di); //get uah for capacity in loading table
	        di->capacity = pre_cap;

	        if (di->capacity<=2) {
	            di->capacity=2;
	            di->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(di);
	        } else {//last cap >2
	            di->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(di); //get uah for capacity in last value
	            di->fg_hidden_uAh=di->fg_acc_uAh-uah;//last - loading
	            di->fg_acc_uAh=uah;
	        }
		} else {
			di->capacity=loadingVoltToCap(di->loading_Table,di->current_uA/1000,di->voltage_mV);
			di->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(di);
			bci_dbg(debug_level_common,"last-loading more than %d percent,use loading table as init capacity=%d,line=%d\n",LAST_LOADING_DIFF,di->capacity,__LINE__);
			
		}
    }
//    twl6030_fg_sync(di);

    // write init capacity to mem
    if ((di->capacity>=0 && di->capacity<=2) || di->fg_full_eod>=EOD_COUNT)  // when capacity = 0~2
        twl6030_store_capacity(di, 2);
    else if (di->capacity>2) // 3~100
        twl6030_store_capacity(di, di->capacity);

    bci_dbg(debug_level_common,"twl6030_fg_calculate_init_capacity = %d, hidden %dmAh,acc %dmAh!!\n",
            di->capacity,di->fg_hidden_uAh/1000,di->fg_acc_uAh/1000);

}

static int  twl6030_fg_capacity(struct twl6030_bci_device_info *di)
{
    int prev_capacity=di->capacity;
    int update=0;
    int time_delta = jiffies_to_msecs(jiffies - di->ac_last_refresh);

    if (twl6030_fg_update(di))return 0;

    if (time_delta > (1000 * 60 * 10)) {
        di->ac_last_refresh = jiffies;
        update=1;
		if (di->no_main_battery) update = 0; /* skip time checking for no main battery case */
    }


    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        twl6030_fg_calculate_init_capacity(di);
        di->charge_status=POWER_SUPPLY_STATUS_NOT_CHARGING;

    }
    else if (di->charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {

        if ((di->fg_full_charged)&&(di->capacity<99))di->fg_full_charged=0;

        /*Charger Plugged but not charging, error handleing is need*/
        bci_dbg(debug_level_A,"twl6030 NOT CHARGING!!!\n");
        if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
			twl6030_fg_sync(di); 
			twl6030_fg_update(di);
    	}
		
        if (di->current_uA<0)twl6030_fg_discharge_regression(di);
        else twl6030_fg_charge_process(di);

    } else if (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
        bci_dbg(debug_level_A,"twl6030 DISCHARGING!!!\n");

        if ((di->fg_full_charged)&&(di->capacity<99))di->fg_full_charged=0;

        if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
            twl6030_fg_sync(di);
        } else if (di->charge_prev_status == POWER_SUPPLY_STATUS_FULL) {

            di->fg_acc_uAh=di->fg_full_uAh;
            twl6030_fg_sync(di);
        }

        if (di->current_uA<0)twl6030_fg_discharge_regression(di);

    }
    else  if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
        bci_dbg(debug_level_A,"twl6030 CHARGING!!!\n");
        if ((di->fg_full_charged)&&(di->capacity<=99))di->fg_full_charged=0;

        if (di->fg_full_charged)di->charge_status = POWER_SUPPLY_STATUS_FULL;
        if (di->charge_prev_status == POWER_SUPPLY_STATUS_DISCHARGING)	{
            twl6030_fg_sync(di);

        } else if (di->charge_prev_status == POWER_SUPPLY_STATUS_FULL) {

            di->fg_acc_uAh=di->fg_full_uAh;

            twl6030_fg_sync(di);
        }
        twl6030_fg_charge_process(di);
        di->fg_full_eod=0;

    }
    else if (di->charge_status == POWER_SUPPLY_STATUS_FULL)
    {
        if (!di->fg_full_charged)di->fg_full_charged=1;
        if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
            di->fg_acc_uAh =di->fg_full_uAh;
            di->fg_hidden_uAh=0;
            di->fg_last_delta_uAh=0;

            twl6030_fg_sync(di);
        }
        di->fg_full_eod=0;

        if (di->current_uA<0) {
            twl6030_fg_discharge_regression(di);
            //di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        } else {
            twl6030_fg_charge_process(di);
        }
    }

	if (di->no_main_battery)
		di->capacity = 100;
	else
		di->capacity = twl6030_fg_calculate_capacity(di);

	if (di->current_uA < 0) {
	    bci_dbg(debug_level_common,"twl6030_fg_capacity() - vol=%d, vsys=%d, cap=%d(diff=%d%%),tmp=%d,curr=%d, detect rst=%d, state %d <- %d,  screen off = %d\n",
	            di->voltage_mV,di->sys_voltage_mV,di->capacity,di->capacity-di->fg_remain_uAh/PERCENT_FACTOR,di->temp_C,di->current_uA/1000, di->usb_ac_detection_result,di->charge_status,di->charge_prev_status, di->bscreen_off);
	} else {
	    bci_dbg(debug_level_common,"twl6030_fg_capacity() - vol=%d, vsys=%d, cap=%d,tmp=%d,curr=%d, detect rst=%d, state %d <- %d,  screen off = %d\n",
	            di->voltage_mV,di->sys_voltage_mV,di->capacity,di->temp_C,di->current_uA/1000, di->usb_ac_detection_result,di->charge_status,di->charge_prev_status, di->bscreen_off);
	}

    return (prev_capacity!=di->capacity)||update;
}

static int twl6030_get_gpadc_conversion(struct twl6030_bci_device_info *di,int channel_no)
{

    struct twl6030_gpadc_request req;
    int temp = 0;
    int ret;

    req.channels = (1 << channel_no);
    req.method = TWL6030_GPADC_SW2;

    req.active = 0;
    req.func_cb = NULL;
    ret = twl6030_gpadc_conversion(&req);

    if (ret < 0)
        return ret;

    if (req.rbuf[channel_no] > 0)
        temp = req.rbuf[channel_no];

    return temp;
}

static int is_battery_present(struct twl6030_bci_device_info *di)
{
    int val;
    /*
     * Prevent charging on batteries were id resistor is
     * less than 5K.
     */
    val = twl6030_get_gpadc_conversion(di,0);
    //if (val <10000)
    //	return 0;
    return 1;
}

static void twl6030_stop_usb_charger(struct twl6030_bci_device_info *di)
{
    int ret=0;

	if (di->no_main_battery) {
		di->charger_source = 0;
		return;
	}

    bci_dbg(debug_level_common, "%s() \n", __func__);
    di->charger_source = 0;
    if (di->charge_status != POWER_SUPPLY_STATUS_UNKNOWN)
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0x10, CONTROLLER_CTRL1);
    if (ret)
        pr_err("%s: Error access to TWL6030 (%d)\n", __func__, ret);
}

static void twl6030_start_usb_charger(struct twl6030_bci_device_info *di)
{
	if (di->no_main_battery) {
		bci_dbg(debug_level_common, "NO MAIN BATTERY,line=%d\n",__LINE__);		
		return;
	}

    if ((di->features & TWL6032_SUBCLASS) && di->use_hw_charger) {		
		bci_dbg(debug_level_common, "TWL6032_SUBCLASS USE HW CHARGER,line=%d\n",__LINE__);		
        return;
	}
    if (machine_is_omap4_talos10()) {
        bci_dbg(debug_level_common, "TALOS 10 - disable usb charging \n");
        return;
    } else if(machine_is_omap4_puzzle()) {
        bci_dbg(debug_level_common, "Puzzle - disable usb charging \n");
        return;
    } else if(machine_is_omap4_quartz()) {
        bci_dbg(debug_level_common, "Quartz - disable usb charging \n");
        return;
    } else if(machine_is_omap4_wedjat()) {
        bci_dbg(debug_level_common, "Wedjat - disable usb charging \n");
        return;
    } else if(machine_is_omap4_pb1icom()) {
        bci_dbg(debug_level_common, "pb1icom - disable usb charging \n");
        return;
    } 	else if(machine_is_omap4_winmate()) {
        bci_dbg(debug_level_common, "winmate - disable usb charging \n");
        return;
    }

    if (!is_battery_present(di)) {
        bci_dbg(debug_level_common, "BATTERY NOT DETECTED!\n");
        return;
    }


    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS) {
		bci_dbg(debug_level_common, "not start_usb_charger!!,charger_source is main\n");		
        return;
	}
	
	bci_dbg(debug_level_common, "%s() \n", __func__);
    twl6030backupbatt_setup(di, true);

    if (di->use_eeprom_config)
        goto enable;

    di->charger_source = POWER_SUPPLY_TYPE_USB;
    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    di->usb_ac_in_detection= 6;
    twl6030_config_voreg_reg(di, di->regulation_voltagemV);
    if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS)
        twl6030_set_ci_limit(di, di->charger_incurrentmA);
    else
        twl6030_set_ci_limit(di, 500);

    if (!(di->features & TWL6032_SUBCLASS) ||(!di->use_linear_charger)) {
        twl6030_config_vichrg_reg(di, di->charger_outcurrentmA);
    }

enable:
    twl6030_config_iterm_reg(di, 50);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                     CONTROLLER_CTRL1_EN_CHARGER,
                     CONTROLLER_CTRL1);

}
static void twl6032_start_linear_charger(struct twl6030_bci_device_info *di)
{
	if (di->no_main_battery) {
		bci_dbg(debug_level_common, "NO MAIN BATTERY,line=%d\n",__LINE__);
		return;
	}

    if (!(di->features & TWL6032_SUBCLASS)||
            twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
            
			bci_dbg(debug_level_common, "NOT TWL6032_SUBCLASS or registered ext. charger,line=%d\n",__LINE__);
        return;
	}

    if (!is_battery_present(di)) {
        bci_dbg(debug_level_common, "BATTERY NOT DETECTED!\n");
        return;
    }
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS) {
		bci_dbg(debug_level_common, "not start_linear_charger!!,charger_source is main\n");
        return;
	}
	
    bci_dbg(debug_level_common, "%s() \n", __func__);
	
    twl6030backupbatt_setup(di, true);

    if (di->use_eeprom_config)
        goto enable;


    if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS)
        twl6030_config_vichrg_reg(di,di->charger_outcurrentmA);
    else
        twl6030_config_vichrg_reg(di,500);


    twl6030_config_voreg_reg(di, di->regulation_voltagemV);
enable:
    twl6030_config_iterm_reg(di, 50);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                     CONTROLLER_CTRL1_EN_CHARGER|CONTROLLER_CTRL1_EN_LINCH,
                     CONTROLLER_CTRL1);

}
static void twl6032_restart_linear_charger(struct twl6030_bci_device_info *di)
{
	if (di->no_main_battery)
		return;

    if (!(di->features & TWL6032_SUBCLASS)||
            twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER))
        return;

    if (!is_battery_present(di)) {
        bci_dbg(debug_level_common, "BATTERY NOT DETECTED!\n");
        return;
    }
    bci_dbg(debug_level_common, "%s() \n", __func__);
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
        return;

    if (di->use_eeprom_config)
        goto enable;

#if 0
    twl6030_config_vichrg_reg(di,
                              (di->charger_outcurrentmA>di->charger_incurrentmA)?
                              di->charger_incurrentmA:di->charger_outcurrentmA);
#else
    if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS)
        twl6030_config_vichrg_reg(di,di->charger_outcurrentmA);
    else
        twl6030_config_vichrg_reg(di,500);
#endif

    di->charger_source = POWER_SUPPLY_TYPE_USB;
    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;

    twl6030_config_voreg_reg(di, di->regulation_voltagemV);
    if(di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS)   
        twl6030_set_ci_limit(di, di->charger_incurrentmA);
    else
        twl6030_set_ci_limit(di, 500);

enable:
    twl6030_config_iterm_reg(di, 50);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                     CONTROLLER_CTRL1_EN_CHARGER|CONTROLLER_CTRL1_EN_LINCH,
                     CONTROLLER_CTRL1);

}


static void twl6032_stop_linear_charger(struct twl6030_bci_device_info *di)
{
    u8 value;

	if (di->no_main_battery)
		return;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);
    value&=~CONTROLLER_CTRL1_EN_LINCH;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,value,
                     CONTROLLER_CTRL1);

}
static void twl6030_stop_ac_charger(struct twl6030_bci_device_info *di)
{
	if (di->ac_charger_enable == false) {
		bci_dbg(debug_level_common, "ac_charger has already stop\n");
		return;
	}
    di->charger_source = 0;

	if (di->no_main_battery)
		return;

    if (di->charge_status != POWER_SUPPLY_STATUS_UNKNOWN)
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    bci_dbg(debug_level_common, "%s() \n", __func__);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_STOP_CHARGING, NULL);

    if ((di->features & TWL6032_SUBCLASS) && di->use_hw_charger)
        return;

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0x10, CONTROLLER_CTRL1);
	
	di->ac_charger_enable = false;
}

static void twl6030_start_ac_charger(struct twl6030_bci_device_info *di)
{
    u8 rd_reg;
	if (di->ac_charger_enable == true) {		
		bci_dbg(debug_level_common, "ac_charger has already enable\n");		
		return;
	}

	if (di->no_main_battery) {
		bci_dbg(debug_level_common, "NO MAIN BATTERY,line=%d\n",__LINE__);		
		return;
	}

    if (!is_battery_present(di)) {
        bci_dbg(debug_level_common, "BATTERY NOT DETECTED!\n");
        return;
    }

    bci_dbg(debug_level_common, "%s()\n", __func__);
    di->charger_source = POWER_SUPPLY_TYPE_MAINS;
    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    di->usb_ac_in_detection= 6;/*0005071: delay report 500ms*/
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
                     CONTROLLER_INT_MASK);
    twl6030_config_extern_charger(di);

    BCI_NOTIFIER(&di->notifier_list, BCI_RESET_TIMER, NULL);
    BCI_NOTIFIER(&di->notifier_list, BCI_SET_START_CHARGING, NULL);
    if ((di->features & TWL6032_SUBCLASS) && di->use_hw_charger) {
		bci_dbg(debug_level_common, "TWL6032_SUBCLASS USE HW CHARGER,line=%d\n",__LINE__);		
        return;
	}

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                     CONTROLLER_CTRL1_EN_CHARGER |
                     CONTROLLER_CTRL1_SEL_CHARGER,
                     CONTROLLER_CTRL1);
	di->ac_charger_enable = true;

    //check external charger status
    print_external_charger_status(di);

}

static void twl6030_stop_charger(struct twl6030_bci_device_info *di)
{
	if (di->no_main_battery)
		return;

    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
        twl6030_stop_ac_charger(di);
    else if (di->charger_source == POWER_SUPPLY_TYPE_USB)
        twl6030_stop_usb_charger(di);
}
static void twl6032_charger_poll_func(struct twl6030_bci_device_info *di )
{

    u8 charge_state, linear_state;
    int err;

    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        bci_dbg(debug_level_common, "TWL6032 charger poll:charge_status is UNKNOWN\n");
        return ;
    }

    err = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                          CONTROLLER_STAT1);

    err = twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &linear_state,
                          LINEAR_CHRG_STS);
    if (!(charge_state & (VBUS_DET | VAC_DET))) {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        bci_dbg(debug_level_common, "Linear status %x %x: POWER_SUPPLY_STATUS_NOT_CHARGING\n"
                ,linear_state,charge_state);
    } else if (linear_state &
               (LINEAR_CHRG_STS_CC_STS | LINEAR_CHRG_STS_CV_STS|LINEAR_CHRG_STS_DPPM_STS)) {
        bci_dbg_period("Linear status %x %x: POWER_SUPPLY_STATUS_CHARGING\n"
                ,linear_state,charge_state);
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else if (linear_state & LINEAR_CHRG_STS_END_OF_CHARGE) {
		if (di->capacity == 100) {
	        di->charge_status = POWER_SUPPLY_STATUS_FULL;
			bci_dbg(debug_level_common, "Linear status %x %x: POWER_SUPPLY_STATUS_FULL\n"
                ,linear_state,charge_state);				
		} else {
			bci_dbg(debug_level_common, "linear_state is EOC but capacity is %d\n",di->capacity);
		}
    } else {
		if (di->capacity == 100) {
			di->charge_status = POWER_SUPPLY_STATUS_FULL;
			bci_dbg(debug_level_common, "Linear status %x %x: POWER_SUPPLY_STATUS_FULL\n"
				,linear_state,charge_state);				
		} else {
			bci_dbg(debug_level_common, "Linear status %x %x: POWER_SUPPLY_STATUS_NOT_CHARGING\n"
				,linear_state,charge_state);
			di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
    }
}



/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static int twl6030charger_ctrl_event_handler(struct twl6030_bci_device_info *di)
{
    int charger_fault = 0;
    u8 stat_toggle, stat_reset, stat_set = 0;
    u8 charge_state = 0;
    u8 hw_state = 0, temp = 0;
    int index=di->pop_index;

    bci_dbg(debug_level_common, "twl6030charger_ctrl_event_handler() \n");
    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        return -1;
    }
	mutex_lock(&di->ctrl_irq_mutex); 
    if (di->push_index==di->pop_index) {
        if (di->charger_glitch_debounce) {
            if (time_after(jiffies, di->ctrl_irq[(index-1+IRQ_INFO_SIZE)%IRQ_INFO_SIZE].time + HZ/2)) {
				
                charge_state = di->ctrl_irq[index].charge_state;
                if (charge_state & (VAC_DET|VBUS_DET))
                    di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                else
                    di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;				
				
                di->charger_glitch_debounce=0;
                bci_dbg(debug_level_common, "charger_glitch_debounce release!!\n");
                power_supply_changed(&di->bat);
                power_supply_changed(&di->usb);
                power_supply_changed(&di->ac);
            }

        }
		mutex_unlock(&di->ctrl_irq_mutex); 		
        return 0;
    }
    /*Mark as processed*/
    di->pop_index = (++di->pop_index)%IRQ_INFO_SIZE;


    /* read charger controller_stat1 */
    charge_state = di->ctrl_irq[index].charge_state;
    hw_state =di->ctrl_irq[index].hw_state;
	mutex_unlock(&di->ctrl_irq_mutex); 

    bci_dbg(debug_level_common, "[POP %d]: CONTROLLER_STAT1 %02x->%02x HW:%02x \n",index,di->stat1,charge_state,hw_state);
    if (di->stat1==charge_state) {
        bci_dbg(debug_level_common, "**** line : %d,stat1==charge_state return here \n", __LINE__);
        power_supply_changed(&di->bat);
        power_supply_changed(&di->ac);
        return 0;
    }

    stat_toggle = di->stat1 ^ charge_state;
    stat_set = stat_toggle & charge_state;
    stat_reset = stat_toggle & di->stat1;

    if (!(charge_state&(VBUS_DET|VAC_DET))&&
            (di->stat1&(VBUS_DET|VAC_DET))) {
		if (di->no_main_battery)
			di->plugged_out_debounce_count = 0;
		else
        di->plugged_out_debounce_count=5;
	}


    if ((stat_reset & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
        di->ac_online = 0;
        bci_dbg(debug_level_common,"vac removed\n");

        twl6030_stop_ac_charger(di);
		if (di->no_main_battery) {
			di->charger_glitch_debounce = 0;
			cancel_delayed_work(&di->bci_monitor_work);
		} else
        di->charger_glitch_debounce|=CHARGER_PLUGOUT_DETECTING;
		schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(500));		
    }



    if (stat_reset & VBUS_DET) {
        /* On a USB detach, UNMASK VBUS OVP if masked*/
        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

        if (temp & MASK_MCHARGERUSB_FAULT)
			/* turn on usb charger fault interrupt */
            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                             (temp & ~MASK_MCHARGERUSB_FAULT),
                             CHARGERUSB_INT_MASK);
        di->usb_online = 0;
        bci_dbg( debug_level_common,"usb removed\n");
		di->dynamic_charging.enable = false;
        twl6030_stop_usb_charger(di);
		if (di->no_main_battery) {
			di->charger_glitch_debounce = 0;
			cancel_delayed_work(&di->bci_monitor_work);
		} else
        di->charger_glitch_debounce|=CHARGER_PLUGOUT_DETECTING;
		di->usb_ac_detection_result = 0; 
		di->usb_ac_in_detection = -1;
		schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(500));
    }

    if ((charge_state & VAC_DET) &&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)&&
            (charge_state & CONTROLLER_STAT1_EXTCHRG_STATZ)&&
            (di->ac_online == POWER_SUPPLY_TYPE_MAINS)) {
        struct bci_read read;
        BCI_NOTIFIER(&di->notifier_list, BCI_READ_FAULT_REASON, &read);
        switch (read.status) {
        case POWER_SUPPLY_STATUS_NOT_CHARGING:

            break;
        }
    }


    if ((stat_set & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
        bci_dbg(debug_level_common, "vac det\n");

        di->ac_online = POWER_SUPPLY_TYPE_MAINS;
        twl6030_start_ac_charger(di);
        di->charger_glitch_debounce=0;
    }

    if (stat_set & VBUS_DET) {
        /* In HOST mode (ID GROUND) when a device is connected, Mask
         * VBUS OVP interrupt and do no enable usb charging
         */
        bci_dbg(debug_level_common, "vbus det\n");

        if (hw_state & STS_USB_ID) { /* OTG devices */

            twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

            if (!(temp & MASK_MCHARGERUSB_FAULT))
				/* turn off usb charger fault interrupt */
                twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                                 (temp | MASK_MCHARGERUSB_FAULT),CHARGERUSB_INT_MASK);
        } else { /* NOT OTG devices */
            di->usb_online = POWER_SUPPLY_TYPE_USB;
            bci_dbg(debug_level_common,"line = %d,di->usb_online = POWER_SUPPLY_TYPE_USB\n",__LINE__);
            if ((charge_state & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER) &&
                    di->charger_source == POWER_SUPPLY_TYPE_MAINS) {
                bci_dbg(debug_level_common,"USB charger detected, continue with VAC\n");
            } else {
	            di->charger_source = POWER_SUPPLY_TYPE_USB;
                twl6030_start_usb_charger(di);
            }
        }
        di->charger_glitch_debounce=0;
    }


    if (di->usb_online == POWER_SUPPLY_TYPE_USB) {
        if (stat_set & CONTROLLER_STAT1_FAULT_WDG) {
            if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING)
            {
            charger_fault = 1;
                watchdogflasg = true;
            }
            bci_dbg(debug_level_common, "Fault watchdog fired\n");
        } else if (stat_reset & CONTROLLER_STAT1_FAULT_WDG) {
            bci_dbg(debug_level_common, "Fault watchdog recovered\n");
        }

        if (stat_set & CONTROLLER_STAT1_BAT_REMOVED) {
            bci_dbg(debug_level_common, "Battery removed\n");
        } else if (stat_reset & CONTROLLER_STAT1_BAT_REMOVED) {
            bci_dbg(debug_level_common, "Battery inserted\n");
        }

        if (stat_set & CONTROLLER_STAT1_BAT_TEMP_OVRANGE) {
            bci_dbg(debug_level_common, "Battery temperature overrange\n");
        } else if (stat_reset & CONTROLLER_STAT1_BAT_TEMP_OVRANGE) {
            bci_dbg(debug_level_common, "Battery temperature within range\n");
        }

        if (charger_fault) {
            twl6030_stop_usb_charger(di);
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            bci_dbg(debug_level_common, "Charger Fault stop charging\n");
        }
    }
    if ((di->capacity != -1)&&(!di->charger_glitch_debounce))
        power_supply_changed(&di->bat);

    di->stat1 = charge_state;

#ifdef POWEROFF_CHARGE
    /*report immediately in charger mode*/
    if (di->boot_mode == ANDROID_BOOT_MODE_CHARGER) {
        power_supply_changed(&di->bat);
        power_supply_changed(&di->ac);
    }
#endif

    return 0;
}
static irqreturn_t twl6030charger_vbat_low_interrupt(int irq, void *_di){
    struct twl6030_bci_device_info *di = _di;
    bci_dbg(debug_level_common, "%s(), capacity = %d \n", __func__, di->capacity);

    power_supply_changed(&di->bat);
    power_supply_changed(&di->ac);	
    power_supply_changed(&di->bat);
	
    return IRQ_HANDLED;	
}

static irqreturn_t twl6030charger_ctrl_interrupt(int irq, void *_di)
{
    struct twl6030_bci_device_info *di = _di;
    bci_dbg(debug_level_common, "%s(), status = %d \n", __func__, di->charge_status);
    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        return IRQ_HANDLED;
    } else {
        //bci_dbg(debug_level_common, "CALLS irq_handler_push() \n");

        irq_handler_push(di);

		if (di->no_main_battery)
			cancel_delayed_work(&di->bci_monitor_work);
        schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(100));

    }

    return IRQ_HANDLED;
}

static irqreturn_t twl6030charger_fault_interrupt(int irq, void *_di)
{
    struct twl6030_bci_device_info *di = _di;
    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        return IRQ_HANDLED;
    } else {
        di->fault_event++;

        schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(100));

    }

    return IRQ_HANDLED;
}
static int  twl6030charger_fault_event_handler(struct twl6030_bci_device_info *di)
{
    u8 controller_stat;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

    //bci_dbg(debug_level_common, "twl6030charger_fault_event_handler() \n");

    /*for external 24161*/
    if ((di->features & TWL6032_SUBCLASS) && twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER) && (controller_stat & VAC_DET)) {
        struct bci_read read;
        int result = 0;
        int charger_fault = 0;


        BCI_NOTIFIER(&di->notifier_list, BCI_READ_FAULT_REASON, &read);
        result = read.reasons;

        switch (result) {
        case 0:
            bci_dbg(debug_level_common, "normal\n");
            break;
        case 1:
            bci_dbg(debug_level_common, "Thermal Shutdown\n");
            break;
        case 2:
            bci_dbg(debug_level_common, "Battery Temperature Fault\n");
            twl603x_recharging(di);
            break;
        case 3:
            bci_dbg(debug_level_common, "Watchdog Timer Expired\n");
            twl603x_recharging(di);
            break;
        case 4:
            bci_dbg(debug_level_common, "Safety Timer Expired\n");
            twl603x_recharging(di);
            break;
        case 5:
            bci_dbg(debug_level_common, "IN Supply Fault\n");
            break;
        case 6:
            bci_dbg(debug_level_common, "USB Supply Fault\n");
            break;
        case 7:
            bci_dbg(debug_level_common, "Battery Fault\n");
            break;

        default:
            bci_dbg(debug_level_common, "default\n");
        }


        if (charger_fault) {
            twl6030_stop_ac_charger(di);
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            bci_dbg(debug_level_common, "Charger Fault stop charging\n");
        }


    }

    /*for internal 6032*/
    else {
        int charger_fault = 0;
        int ret;

        u8 usb_charge_sts = 0, usb_charge_sts1 = 0, usb_charge_sts2 = 0,charge_state=0;
        if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
            return IRQ_HANDLED;
        }

        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                              CONTROLLER_STAT1);

        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
                              CHARGERUSB_INT_STATUS);
        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
                              CHARGERUSB_STATUS_INT1);
        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
                              CHARGERUSB_STATUS_INT2);

        di->status_int1 = usb_charge_sts1;
        di->status_int2 = usb_charge_sts2;
        if (charge_state&VBUS_DET ) {
            if (usb_charge_sts & CURRENT_TERM_INT)
                bci_dbg(debug_level_common, "USB CURRENT_TERM_INT\n");
            if (usb_charge_sts & CHARGERUSB_THMREG)
                bci_dbg(debug_level_common, "USB CHARGERUSB_THMREG\n");
            if (usb_charge_sts & CHARGERUSB_FAULT)
                bci_dbg(debug_level_common, "USB CHARGERUSB_FAULT\n");

            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TMREG)
                bci_dbg(debug_level_common, "USB CHARGER Thermal regulation activated\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_NO_BAT)
                bci_dbg(debug_level_common, "No Battery Present\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BST_OCP)
                bci_dbg(debug_level_common, "USB CHARGER Boost Over current protection\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TH_SHUTD) {
                charger_fault = 1;
                bci_dbg(debug_level_common, "USB CHARGER Thermal Shutdown\n");
            }
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BAT_OVP)
                bci_dbg(debug_level_common, "USB CHARGER Bat Over Voltage Protection\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_POOR_SRC)
                bci_dbg(debug_level_common, "USB CHARGER Poor input source\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_SLP_MODE)
                bci_dbg(debug_level_common, "USB CHARGER Sleep mode\n");
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_VBUS_OVP)
                bci_dbg(debug_level_common, "USB CHARGER VBUS over voltage\n");

            if (usb_charge_sts2 & CHARGE_DONE) {
                di->charge_status = POWER_SUPPLY_STATUS_FULL;
                bci_dbg(debug_level_common, "USB charge done\n");
            }
            if (usb_charge_sts2 & CURRENT_TERM)
                bci_dbg(debug_level_common, "USB CURRENT_TERM\n");
            if (usb_charge_sts2 & ICCLOOP)
                bci_dbg(debug_level_common, "USB ICCLOOP\n");
            if (usb_charge_sts2 & ANTICOLLAPSE)
                bci_dbg(debug_level_common, "USB ANTICOLLAPSE\n");
        }
		
        if (charger_fault) {
			bci_dbg(debug_level_common,"Charger fault detected STS, INT1, INT2 0x%02x 0x%02x 0x%02x\n",
				usb_charge_sts, usb_charge_sts1, usb_charge_sts2);
			bci_dbg(debug_level_common, "charge_state = 0x%02x \n", charge_state);		
            bci_dbg(debug_level_common,"Charger Fault stop charging\n");
            twl6030_stop_usb_charger(di);
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }

    }

    if (!di->charger_glitch_debounce)
        power_supply_changed(&di->bat);
    return IRQ_HANDLED;
}

static void twl6030battery_current(struct twl6030_bci_device_info *di)
{
    int ret = 0;
    u16 read_value = 0;

    /* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
    ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
                       FG_REG_10, 2);
    if (ret < 0) {
        bci_dbg(debug_level_common, "failed to read FG_REG_10: current_now\n");
        return;
    }
    di->current_uA = twl6030_fg_avg_sample_current_uA(di,
                     (s16)(read_value << 2) >> 2,1);
    return;
}

/*
 * Setup the twl6030 BCI module to enable backup
 * battery charging.
 */
static int twl6030backupbatt_setup(struct twl6030_bci_device_info *di, bool enable)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);

    if ((di->max_backupbat_voltagemV == 0) || (!enable)) {
        /* No backup battery */
        rd_reg &= ~BB_CHG_EN;
    } else if (di->max_backupbat_voltagemV == 2500) {
        rd_reg &= ~BB_SEL_MASK;
        rd_reg |= BB_CHG_EN | BB_SEL_2V5;
    } else if (di->max_backupbat_voltagemV == 3000) {
        rd_reg &= ~BB_SEL_MASK;
        rd_reg |= BB_CHG_EN | BB_SEL_3V0;
    } else if (di->max_backupbat_voltagemV == 3150) {
        rd_reg &= ~BB_SEL_MASK;
        rd_reg |= BB_CHG_EN | BB_SEL_3V15;
    } else if (di->max_backupbat_voltagemV == 4200) {
        rd_reg &= ~BB_SEL_MASK;
        rd_reg |= BB_CHG_EN | BB_SEL_VSYS;
    } else {
        rd_reg |= BB_CHG_EN;
        bci_dbg(debug_level_common, "Invalid CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV (0, 2500, 3000, 3150, 4200)!\n");
    }

    ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);

    return ret;
}

/*
 * Setup the twl6030 BCI module to measure battery
 * temperature
 */
static int twl6030battery_temp_setup(bool enable)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);


    if (enable)
        rd_reg |= TWL6030_GPADC_CTRL_TEMP1_EN |TWL6030_GPADC_CTRL_TEMP1_EN_MONITOR;
    else
        rd_reg ^= TWL6030_GPADC_CTRL_TEMP1_EN |TWL6030_GPADC_CTRL_TEMP1_EN_MONITOR;


    ret |= twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

    return ret;
}

static int twl6030battery_voltage_setup(struct twl6030_bci_device_info *di)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);

    if (di->max_backupbat_voltagemV == 0) {
        /* No backup battery */
        rd_reg = rd_reg & ~(BB_MEAS);
        rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS;
    } else {
        rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
    }

    ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);

    ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
    rd_reg = rd_reg | VBUS_MEAS;
    ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);

    ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
    rd_reg = rd_reg | ID_MEAS;
    ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);

    return ret;
}

static int twl6030battery_current_setup(bool enable)
{
    int ret = 0;
    u8 reg = 0;
#if 0
    ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &rd_reg, PWDNSTATUS2);
    rd_reg = (rd_reg & 0x30) >> 2;
    rd_reg = rd_reg | FGDITHS | FGS;
    ret |= twl_i2c_write_u8(TWL6030_MODULE_ID1, rd_reg, REG_TOGGLE1);
    ret |= twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);
#else
    /*
    * Writing 0 to REG_TOGGLE1 has no effect, so
    * can directly set/reset FG.
    */
    if (enable)
        reg = FGDITHS | FGS;
    else
        reg = FGDITHR | FGR;

    //bci_dbg(debug_level_common, "reg of toggle1 before = 0x%02x \n", reg);
    //reg&=~GPADC_SAMP_WINDOW;
    // bci_dbg(debug_level_common, "reg of toggle1 after = 0x%02x \n", reg);

    ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, reg, REG_TOGGLE1);
    if (ret)
        return ret;
    ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

#endif

    return ret;
}
/*end - mantis:0004660 [FR] [PM]TWL6030: Battery: Handle Fuel Gauge during Suspend.*/
#define VBUS_THRESHOLD (4600)
static void twl6032_usb_dynamic_charging_current(struct twl6030_bci_device_info *di)
{

	if (di->dynamic_charging.enable == true) {
		
		int vbus_voltage = 0;
		int curr_time = jiffies_to_msecs(jiffies);	
		int current_update = 500;
		
		vbus_voltage = twl6030_get_gpadc_conversion(di,10);
		if (vbus_voltage == 0) {
			di->dynamic_charging.stage = 0;
			di->dynamic_charging.previous_time = 0;
			return;
		} else if (vbus_voltage < VBUS_THRESHOLD) {

			twl6030_set_ci_limit(di,500);
			msleep(200);
			twl6030_config_vichrg_reg(di,500);
			di->dynamic_charging.stage = 0;
			di->dynamic_charging.previous_time = 0;
			di->dynamic_charging.enable = false;
			di->dynamic_charging.stage_fixed = false;			
			printk("[BCI]twl6032_usb_dynamic_charging_current,vbus=%d,stage=%d,curr=%d\n",vbus_voltage,di->dynamic_charging.stage,bqdr.AvgCurr);
			printk("[BCI]vbus is below %d!,set charging curr=500\n",VBUS_THRESHOLD);
			return;
		}
		if (curr_time - di->dynamic_charging.previous_time > 2000 && !di->dynamic_charging.stage_fixed) {
			printk("[BCI]twl6032_usb_dynamic_charging_current,vbus=%d,stage=%d,curr=%d\n",vbus_voltage,di->dynamic_charging.stage,bqdr.AvgCurr);
			
			if (vbus_voltage > VBUS_THRESHOLD) {
				if (di->dynamic_charging.stage == 0) {
					current_update = 600;
					di->dynamic_charging.stage = 1;
				} else if (di->dynamic_charging.stage == 1) {
					current_update = 700;
					di->dynamic_charging.stage = 2;
				} else if (di->dynamic_charging.stage == 2) {
					current_update = 800;
					di->dynamic_charging.stage = 3;
				} else if (di->dynamic_charging.stage == 3) {
					current_update = 900;
					di->dynamic_charging.stage = 4;
				} else if (di->dynamic_charging.stage == 4) {
					current_update = 1000;
					di->dynamic_charging.stage = 5;
				} else if (di->dynamic_charging.stage == 5) {
					current_update = 1100;
					di->dynamic_charging.stage = 6;
				} else if (di->dynamic_charging.stage == 6) {
					current_update = 1200;
					di->dynamic_charging.stage = 7;
				} else if (di->dynamic_charging.stage == 7) {
					current_update = 1500;
					di->dynamic_charging.stage_fixed = true; /* finish dynamic function*/
				} else {
					printk("[BCI]dynamic charging current unknown range!\n");
				}
				printk("[BCI]set dynamic charging current=%d\n",current_update);
			}
			twl6030_set_ci_limit(di,current_update);
			msleep(200);
			twl6030_config_vichrg_reg(di,current_update);
		}
		di->dynamic_charging.previous_time = curr_time;
	}
}

static enum power_supply_property twl6030_bci_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property twl6030_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_bk_bci_battery_props[] = {
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static void twl6030_bci_battery_work(struct work_struct *work)
{
    u8 controller_stat;
    struct twl6030_bci_device_info *di = container_of(work,
                                                     struct twl6030_bci_device_info, bci_monitor_work.work);
    int local_status = di->charge_status;
	
	if (di->bci_notifier_shutdown == true)
		return;
	
    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
        twl6030_update_gg_data(di);
    }

    if (di->bFlagMonitorStart==0)
        di->bFlagMonitorStart =1;
	
	if (log_counter == 0xFFFFFFFF)
		log_counter = 0;
	else
		log_counter++;
	
    /*1. read hw plug status and adcs */
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

    di->sys_voltage_mV= twl6030_get_gpadc_conversion(di,7);
    if (di->features & TWL6032_SUBCLASS) {
        if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
            di->voltage_mV = bqdr.Voltage;
        } else {
            di->voltage_mV = twl6030_get_gpadc_conversion(di,18);
        }
    } else {
        di->voltage_mV = di->sys_voltage_mV;
    }
    di->bk_voltage_mV =twl6030_get_gpadc_conversion(di,8);

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {		
        di->temp_C = bqdr.Temp/10;
    } else if (di->battery_tmp_tbl_603x != NULL) {
        int adc_code = 0;
        int r_ntc = 0;
        int denominator;
		
        adc_code =twl6030_get_gpadc_conversion(di,1);
        denominator = di->temp_Ry*1250-(di->temp_Ry+di->temp_Rx)*adc_code;
        if (denominator>0) {
            r_ntc=di->temp_Rx*di->temp_Ry*adc_code*1000/denominator;
            di->temp_C=resistance_to_temp(di->battery_tmp_tbl_603x,r_ntc);
        }
        bci_dbg(debug_level_common,"[FG]adc V=%d, R=%d ,Temp=%d\n",adc_code,r_ntc,di->temp_C);
    }
    /*Rntc=Rpu*Rpd*Vadc/(Rpd*Vref-(Rpu+Rpd)*Vadc)*/


    /*2. handle interrupt and  Update new status after handle result */
    if ((di->push_index!=di->pop_index) ||(di->charger_glitch_debounce)||(di->ctl_event>0)) { //have some push events
        if (!twl6030charger_ctrl_event_handler(di))
            if (di->ctl_event>0)  di->ctl_event--;
    }
	
    if (di->fault_event>0) {
        twl6030charger_fault_event_handler(di);
        di->fault_event--;
    }


    /*3. Do right action in each state and state transition */
    /*3.1. POWER_SUPPLY_STATUS_DISCHARGING The most static state*/
    if (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
        /*We only care about state transition and process all uncomplete action if need*/
        if ((di->charge_prev_status != POWER_SUPPLY_STATUS_DISCHARGING)||di->usb_ac_detection_result>0) {
			/* charger unplug case */
            bci_dbg(debug_level_common,"clear in detection\n");
            di->usb_ac_in_detection=-1;
            di->usb_ac_detection_result=0;
            di->usb_max_power=100;

            /*plug out charger, but no isr be called*/
            if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
                bci_dbg(debug_level_common,"prev = charg, status = discharg\n");
                irq_handler_push(di);
            }

        } else {

        }
        if (di->plugged_out_debounce_count>0) {
            di->plugged_out_debounce_count--;
        } else if (wake_lock_active(&di->wakelock)) {
            if (di->boot_mode == ANDROID_BOOT_MODE_CHARGER) 
                printk("[BCI]power off mode - do not wake unlock, let power off charging into shutdown \n");
            else
                wake_unlock(&di->wakelock);
        } else if ((controller_stat & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
            wake_lock(&di->wakelock);
        }

    } else {
        /*3.2. !POWER_SUPPLY_STATUS_DISCHARGING it is implied charger plugged*/
        if (!wake_lock_active(&di->wakelock))
            wake_lock(&di->wakelock);

        twl6030_kick_watchdog(di);
		
        /*3.2.1. Update battery charge state from charger */
        if ((di->usb_ac_in_detection<0)&&((di->charge_status != POWER_SUPPLY_STATUS_UNKNOWN))) {
			/*Never update battery charge status when usb/ac in detection or  in
			POWER_SUPPLY_STATUS_UNKNOWN state */
			/*If we have external charger, update status from external*/
            if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
                if (debug_en&debug_level_G)
                    BCI_NOTIFIER(&di->notifier_list, BCI_DEBUG_MONITOR, 0);


            } else if ((di->features & TWL6032_SUBCLASS) &&(di->use_linear_charger)) {
                twl6032_charger_poll_func(di);
            } else {
                /*Twl6030 or 6032 w/o use linear charger case*/
            }
        }

        /*3.2.2 POWER_SUPPLY_STATUS_NOT_CHARGING*/
        if (di->charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {

			/*The most complicated state is POWER_SUPPLY_STATUS_NOT_CHARGING
			it may do error handling here*/

            /*3.2.2.1 Auto correct the battery charge state */
            if (controller_stat & (VAC_DET|VBUS_DET))
                di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            else
                di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;


			/*3.2.2.2 POWER_SUPPLY_STATUS_UNKNOWN->POWER_SUPPLY_STATUS_NOT_CHARGING
			*it is only happen from power on and wait for fg init,
			*After fg initialized, the state become POWER_SUPPLY_STATUS_NOT_CHARGING
			*/
            if (di->charge_prev_status == POWER_SUPPLY_STATUS_UNKNOWN) {
                /*This mean it comes from boot, and the fuel gauge is ready*/
                u8 hw_state;
				/*
				* In HOST mode (ID GROUND) with a device connected,
				* do no enable any charging
				*/
                twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);

                if (!(hw_state & STS_USB_ID)) { /* not OTG devices*/

                    /*Check VBUS and VAC to start charging or not */
                    if ((controller_stat & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
                        di->ac_online = POWER_SUPPLY_TYPE_MAINS;
                        twl6030_start_ac_charger(di);
                    } else if (controller_stat & VBUS_DET) {
                        di->usb_online = POWER_SUPPLY_TYPE_USB;
                        bci_dbg(debug_level_common,"line = %d,di->usb_online = POWER_SUPPLY_TYPE_USB\n",__LINE__);
                        twl6030_start_usb_charger(di);

                    } else {
                        /*Only update battery status*/
                        di->charge_status =POWER_SUPPLY_STATUS_DISCHARGING;
                    }

                } else { /* is OTG devices*/
					/*Update battery status to POWER_SUPPLY_STATUS_DISCHARGING
					if OTG host is working */
                    di->charge_status =POWER_SUPPLY_STATUS_DISCHARGING;
                }

                /* Enable interrupt to complete power on */
                twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
                                         REG_INT_MSK_LINE_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
                                         REG_INT_MSK_STS_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
                                         REG_INT_MSK_LINE_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
                                         REG_INT_MSK_STS_C);

                /*USB interrupt and detection may happen before fuel gauge ready*/
                if (di->usb_ac_detection_result)di->usb_ac_in_detection=0;

                di->stat1 = controller_stat;
                //update_ext_status(di);

            } else if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {

                bci_dbg(debug_level_common,"detection = %d\n",di->usb_ac_detection_result);
				{ //not reach full charged

                    if (!twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) { //without 24161

                       int boot_mode = android_boot_get_mode();
                       if (boot_mode == ANDROID_BOOT_MODE_CHARGER) {
                           twl6032_start_linear_charger(di);					   	
                       }
						else {
							if (watchdogflasg)
							{
								watchdogflasg = false;
								printk("[BCI]twl6032_restart_linear_charger(di), charger_source[%d]\n", di->charger_source);
								twl6032_restart_linear_charger(di);
							}
						}

                    } else {//24161
                        bci_dbg(debug_level_common,"twl603x_recharging(di),detect = main\n");
                        twl603x_recharging(di);
                    }
                }

            } else if (di->charge_prev_status == POWER_SUPPLY_STATUS_DISCHARGING) {
                /*means plugged in charger, but not charging*/

                irq_handler_push(di);
            }
        }


        /*3.2.3 POWER_SUPPLY_STATUS_CHARGING*/
        else  if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING || 
			di->charge_status == POWER_SUPPLY_STATUS_FULL)
        {
            if (di->usb_ac_in_detection==0) {
                di->usb_ac_in_detection=-1;
                if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS) {
                    di->charger_incurrentmA=di->max_charger_currentmA;
                    printk(KERN_INFO"[BCI]twl6030_bci: Set AC %d mA \n",di->charger_incurrentmA);

                } else if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB) {
                    di->charger_incurrentmA=di->usb_max_power>0?di->usb_max_power:500;
					if (di->usb_max_power >= 500) {
						di->dynamic_charging.enable = true;
						di->dynamic_charging.stage_fixed = false;	
						di->dynamic_charging.stage = 0;
						di->dynamic_charging.previous_time = 0;						
					}
                    printk(KERN_INFO"[BCI]twl6030_bci: Set USB %d mA \n",di->charger_incurrentmA);
                } else {
                    if (android_boot_get_reason() == ANDROID_BOOT_REASON_RECOVERY)
                    {
                        u8 hw_state;

                        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
                        if (!(hw_state & STS_USB_ID)) {
                            di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;
                            di->charger_incurrentmA=500;
                            printk(KERN_INFO"[BCI]twl6030_bci: Set USB %d mA \n",di->charger_incurrentmA);
                        } else {
                            di->charger_incurrentmA=di->max_charger_currentmA;
                            di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
                            printk(KERN_INFO"[BCI]twl6030_bci: Set CarKit %d mA \n",di->charger_incurrentmA);
                        }                
                    } else {
                        di->charger_incurrentmA=di->max_charger_currentmA;
                        di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
                        printk(KERN_INFO"[BCI]twl6030_bci: Set CarKit %d mA \n",di->charger_incurrentmA);
                    }
					power_supply_changed(&di->ac);
					power_supply_changed(&di->bat);
					power_supply_changed(&di->usb); 					
					
                }
                if ((controller_stat & VAC_DET)&&twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {

                    BCI_NOTIFIER(&di->notifier_list, BCI_SET_INPUT_CURRENT_LIMIT,&di->charger_incurrentmA);
					twl6030_start_ac_charger(di);

                } else {

                    twl6030_set_ci_limit(di, di->charger_incurrentmA);

                    if ((di->features & TWL6032_SUBCLASS)&&(di->use_linear_charger)) {
                        msleep(100);
                        twl6032_start_linear_charger(di);
                    }
                }

            }
        }
		#if 0
        /*3.2.4 POWER_SUPPLY_STATUS_FULL*/
        else if (di->charge_status == POWER_SUPPLY_STATUS_FULL)
        {
            if (di->charge_prev_status != POWER_SUPPLY_STATUS_FULL) {
            }
            /*If implement recharge, place code here*/

        }
		#endif
    }

    /*4. update fuel gauge and calculate remain capacity */

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
        twl6030_gg_capacity(di);
    } else {
        if (twl6030_fg_capacity(di)||(di->charge_prev_status!= di->charge_status)) {

            if ((di->capacity>=0 && di->capacity<=2) || di->fg_full_eod>=EOD_COUNT) // // when capacity = 0~2
                twl6030_store_capacity(di, 2);
            else if (di->capacity>2) // 3~100
                twl6030_store_capacity(di, di->capacity);
            if (!di->charger_glitch_debounce )
                power_supply_changed(&di->bat);

        }
    }
	if (di->gas_gauge.can_read == false) {
		di->gas_gauge.can_read = true;
		/* wake up the down semaphore. */
		up(&(di->gas_gauge.sem));
	}
	
// update status from external charger
    if (machine_is_omap4_talos10() || machine_is_omap4_puzzle() || machine_is_omap4_quartz() || machine_is_omap4_saga() ||
        machine_is_omap4_pb1icom() || machine_is_omap4_winmate()) {
        update_ext_status(di);
    }
    di->charge_prev_status= local_status;

    //reset timer of external charger
    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
        BCI_NOTIFIER(&di->notifier_list, BCI_RESET_TIMER, NULL);
    }
    watchdogflasg = false;
	
	if (machine_is_omap4_saga()) {
		twl6032_usb_dynamic_charging_current(di);
	}
	
    /*5. reschedule next work */
    if ((di->usb_ac_in_detection>0)||(di->charger_glitch_debounce)||(di->ctl_event>0)||(di->fault_event>0)) {
        int ret = -1;
        if (di->usb_ac_in_detection>0)
            di->usb_ac_in_detection--;

        ret = schedule_delayed_work_on(0,&di->bci_monitor_work,
                                    msecs_to_jiffies(500));
        if (ret==0) {
            cancel_delayed_work(&di->bci_monitor_work);
            schedule_delayed_work_on(0,&di->bci_monitor_work,
                                  msecs_to_jiffies(500));
        }
	} else if (di->no_main_battery) {
		unsigned long delay_ms = 2000;
        int ret;

		/* current == previous: stable status */
		if (di->charge_status == di->charge_prev_status)
			delay_ms = di->no_main_battery_monitoring_interval;
#if 0
    	bci_dbg(debug_level_common, "delay_ms %lu, charge_prev_status %d, charge_status %d\n",
				delay_ms, di->charge_prev_status, di->charge_status);
#endif
		ret = schedule_delayed_work_on(0,&di->bci_monitor_work,
					msecs_to_jiffies(delay_ms));
		if (ret == 0) {
			cancel_delayed_work(&di->bci_monitor_work);
			schedule_delayed_work_on(0,&di->bci_monitor_work,
					msecs_to_jiffies(delay_ms));
		}
    } else {
        int ret = -1;
		ret = schedule_delayed_work_on(0,&di->bci_monitor_work,
					msecs_to_jiffies(2000));
		if (ret == 0) {
			cancel_delayed_work(&di->bci_monitor_work);
			schedule_delayed_work_on(0,&di->bci_monitor_work,
					msecs_to_jiffies(2000));
		}
    }

}


static void twl6030_work_interval_changed(struct twl6030_bci_device_info *di)
{
    cancel_delayed_work(&di->bci_monitor_work);
    schedule_delayed_work_on(0,&di->bci_monitor_work,
                          msecs_to_jiffies(1000 * di->monitoring_interval));
}

#define to_twl6030_bci_device_info(x) container_of((x), \
			struct twl6030_bci_device_info, bat);

static void twl6030_bci_battery_external_power_changed(struct power_supply *psy)
{
    struct twl6030_bci_device_info *di = to_twl6030_bci_device_info(psy);

    cancel_delayed_work(&di->bci_monitor_work);
    schedule_delayed_work_on(0,&di->bci_monitor_work, 100);
}

#define to_twl6030_ac_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, ac);

static int twl6030_ac_get_property(struct power_supply *psy,
                                   enum power_supply_property psp,
                                   union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = to_twl6030_ac_device_info(psy);
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
    if(machine_is_omap4_wedjat() || machine_is_omap4_talos10()||machine_is_omap4_talos7()||machine_is_omap4_zeus()||
		machine_is_omap4_puzzle()||machine_is_omap4_quartz()||machine_is_omap4_saga() || machine_is_innocomm_oracle() ||
        machine_is_omap4_pb1icom() || machine_is_omap4_winmate()){
        if(di->boot_mode==ANDROID_BOOT_MODE_CHARGER){
            u8 controller_stat;
            twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

            if ((controller_stat & VAC_DET) && 
				(machine_is_omap4_wedjat() || machine_is_omap4_talos10()||machine_is_omap4_puzzle()||machine_is_omap4_quartz()||
                 machine_is_omap4_pb1icom() || machine_is_omap4_winmate())) {
                di->ac_online = POWER_SUPPLY_TYPE_MAINS;
            } else if ((controller_stat & VBUS_DET) && (machine_is_omap4_talos7()||machine_is_omap4_zeus() 
	            || machine_is_innocomm_oracle()||machine_is_omap4_saga())) {
                di->ac_online = POWER_SUPPLY_TYPE_MAINS;
            } else {
                di->ac_online = 0;
                di->usb_ac_detection_result = 0;
            }
            bci_dbg(debug_level_common, "power off charging mode => di->ac_online = %d , controller_stat = 0x%02x\n", di->ac_online, controller_stat);
        }
    }
        val->intval = di->ac_online||(di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS);


        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = twl6030_get_gpadc_conversion(di,9);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
                                    enum power_supply_property psp,
                                    union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = to_twl6030_usb_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
#if 0
    if(machine_is_omap4_zeus()){
        if(di->boot_mode==ANDROID_BOOT_MODE_CHARGER){
            u8 controller_stat;
            twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);
            if (((controller_stat&(1<<2))>0)) {
                di->usb_online = POWER_SUPPLY_TYPE_USB;
                di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;				
            } else {
                di->usb_online = 0;
                di->usb_ac_detection_result = 0;
            }
            bci_dbg(debug_level_common, "power off charging mode => d i->usb_online = %d , controller_stat = 0x%02x\n", di->usb_online, controller_stat);
        }
    }
#endif
        val->intval = di->usb_online&&((di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB));
        //bci_dbg(debug_level_common, "usb -val  = %d\n", val->intval);
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = twl6030_get_gpadc_conversion(di,10);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define to_twl6030_bk_bci_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, bk_bat);

static int twl6030_bk_bci_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = to_twl6030_bk_bci_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = di->bk_voltage_mV * 1000;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int twl6030_bci_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = NULL;

    di = to_twl6030_bci_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        /*When USB charging, report Not Charging*/
        if (di->charge_status==POWER_SUPPLY_STATUS_FULL) {
            val->intval = POWER_SUPPLY_STATUS_FULL;
        } else if ((di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB)&&
                   (di->charge_status==POWER_SUPPLY_STATUS_CHARGING)) {
            val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

        } else if ((di->usb_ac_detection_result == POWER_SUPPLY_TYPE_MAINS) &&
	         (di->capacity < 100)) {
            val->intval = POWER_SUPPLY_STATUS_CHARGING;

        } else if ((di->usb_online == POWER_SUPPLY_TYPE_USB)&&
                   (di->charge_status==POWER_SUPPLY_STATUS_CHARGING)) {
            val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
        } else if (di->usb_ac_detection_result == POWER_SUPPLY_TYPE_MAINS &&
			di->charge_status==POWER_SUPPLY_STATUS_CHARGING && di->capacity == 100) {
			val->intval = POWER_SUPPLY_STATUS_FULL;
        } else {
            val->intval = di->charge_status;

        }

        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        if (di->features & TWL6032_SUBCLASS) {
            di->voltage_mV = twl6030_get_gpadc_conversion(di,18);
        } else {
            di->voltage_mV = twl6030_get_gpadc_conversion(di,7);
        }
        val->intval = di->voltage_mV * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        if (!twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
            twl6030battery_current(di);
        }
        val->intval = di->current_uA;
        break;
    case POWER_SUPPLY_PROP_TEMP:
		if (di->no_main_battery)
        	val->intval = 333;
		else
#if defined(CONFIG_MACH_OMAP4_QUARTZ)
        	val->intval = 333;
#else
			val->intval = di->temp_C;
#endif
        break;
    case POWER_SUPPLY_PROP_ONLINE:

#if 1/*do not show charging info while usb plugged*/
        val->intval = di->usb_ac_detection_result;
        //bci_dbg(debug_level_common, "val->intval = %d\n", val->intval);
#else
        val->intval = (di->charger_source==POWER_SUPPLY_TYPE_USB)?
                      di->usb_ac_detection_result:POWER_SUPPLY_TYPE_MAINS;
#endif

        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = di->current_avg_uA;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = di->bat_health;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
		if (di->no_main_battery) {
        	val->intval = 100;
		} else
		{
			if (twl603x_bci && (di->gas_gauge.can_read == false)) {
				/* wait for gas gauge can be read. */
				if (down_interruptible(&(di->gas_gauge.sem))) {
					printk("[BCI]down_interruptible error\n");
					return -ERESTARTSYS;				
				}
				
			}
			val->intval = di->capacity;        
		}
        break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
    default:
        return -EINVAL;
    }
    return 0;
}


static int twl6030_usb_notifier_call(struct notifier_block *nb,
                                     unsigned long event, void *data)
{
    struct twl6030_bci_device_info *di =
                    container_of(nb, struct twl6030_bci_device_info, nb);

    bci_dbg(debug_level_common, "twl6030_usb_notifier_call() \n");

    di->event = event;
    switch (event) {
    case USB_EVENT_VBUS:
        di->usb_online = *((unsigned int *) data);
        switch (di->usb_online) {
        case POWER_SUPPLY_TYPE_USB_CDP:
            /*
             * Only 500mA here or high speed chirp
             * handshaking may break
             */
            bci_dbg(debug_level_common, "POWER_SUPPLY_TYPE_USB_CDP , incurr = 500\n");
            di->charger_incurrentmA = 500;
            break;
        case POWER_SUPPLY_TYPE_USB:
            bci_dbg(debug_level_common, "POWER_SUPPLY_TYPE_USB\n");
            if(di->boot_mode==ANDROID_BOOT_MODE_CHARGER) {
                di->usb_max_power = 500;
                di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;
            }
            break;
        }
        break;
    case USB_EVENT_ENUMERATED:
        di->usb_max_power = *((unsigned int *) data);
        bci_dbg(debug_level_common, "twl6030_usb_notifier_call:set current %d mA\n",(int)di->usb_max_power);
        if (di->usb_online == POWER_SUPPLY_TYPE_USB_CDP)
            di->charger_incurrentmA = 560;
        else {
            di->charger_incurrentmA = di->usb_max_power;
            if (di->usb_max_power >10) {
                di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;
                bci_dbg(debug_level_common, "USB_EVENT_ENUMERATED\n");
            }
        }
        break;
    case USB_EVENT_CHARGER:
        /* POWER_SUPPLY_TYPE_USB_DCP */

        di->usb_online = POWER_SUPPLY_TYPE_USB_DCP;
        bci_dbg(debug_level_common,"line = %d,di->usb_online = POWER_SUPPLY_TYPE_USB_DCP\n",__LINE__);
        di->charger_incurrentmA = di->max_charger_currentmA;
		if (di->no_main_battery)
			di->usb_ac_in_detection = -1;
		else
                      di->usb_ac_in_detection= 1;
        di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
		power_supply_changed(&di->ac);		
        bci_dbg(debug_level_common,"line = %d,usb_ac_detection_result= main\n",__LINE__);
        break;
    case USB_EVENT_NONE:
        di->usb_online = 0;
        bci_dbg(debug_level_common,"line = %d,di->usb_online = 0\n",__LINE__);
        di->charger_incurrentmA = 0;
		if (di->no_main_battery) {
			di->charge_prev_status = di->charge_status;
			di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}

        break;
    case USB_EVENT_ID:
    default:
        return NOTIFY_OK;
    }

    if (!di->charger_glitch_debounce)
        power_supply_changed(&di->usb);
    return NOTIFY_OK;
}

static ssize_t set_fg_mode(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val > 3))
        return -EINVAL;
    di->fuelgauge_mode = val;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, (val << 6) | CC_CAL_EN,
                     FG_REG_00);
    twl6030_fg_mode_changed(di);

    return status;
}

static ssize_t show_fg_mode(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->fuelgauge_mode;
    return sprintf(buf, "%d\n", val);
}

static ssize_t set_charge_src(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 2) || (val > 3))
        return -EINVAL;
    di->vac_priority = val;
    return status;
}

static ssize_t show_charge_src(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->vac_priority;
    return sprintf(buf, "%d\n", val);
}

static ssize_t show_vbus_voltage(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    val = twl6030_get_gpadc_conversion(di,10);

    return sprintf(buf, "%d\n", val);
}

static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    val = twl6030_get_gpadc_conversion(di,14);

    return sprintf(buf, "%d\n", val);
}

static ssize_t set_watchdog(struct device *dev,
                            struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 1) || (val > 127))
        return -EINVAL;
    di->watchdog_duration = val;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);

    return status;
}

static ssize_t show_watchdog(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->watchdog_duration;
    return sprintf(buf, "%d\n", val);
}

static ssize_t show_fg_counter(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int fg_counter = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_counter,
                 FG_REG_01, 3);
    return sprintf(buf, "%d\n", fg_counter);
}

static ssize_t show_fg_accumulator(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    long fg_accum = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_accum, FG_REG_04, 4);

    return sprintf(buf, "%ld\n", fg_accum);
}

static ssize_t show_fg_offset(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    s16 fg_offset = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_offset, FG_REG_08, 2);
    fg_offset = ((s16)(fg_offset << 6) >> 6);

    return sprintf(buf, "%d\n", fg_offset);
}

static ssize_t set_fg_clear(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    long val;
    int status = count;

    if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
        return -EINVAL;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_AUTOCLEAR, FG_REG_00);

    return status;
}

static ssize_t set_fg_cal(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    long val;
    int status = count;

    if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
        return -EINVAL;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

    return status;
}

static ssize_t set_charging(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if (strncmp(buf, "startac", 7) == 0) {
        if (di->charger_source == POWER_SUPPLY_TYPE_USB)
            twl6030_stop_usb_charger(di);
        twl6030_start_ac_charger(di);
    } else if (strncmp(buf, "startusb", 8) == 0) {
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
            twl6030_stop_ac_charger(di);
        di->charger_source = POWER_SUPPLY_TYPE_USB;
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        twl6030_start_usb_charger(di);
    } else if (strncmp(buf, "stop" , 4) == 0)
        twl6030_stop_charger(di);
    else
        return -EINVAL;

    return status;
}

static ssize_t set_regulation_voltage(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
            || (val > di->max_charger_voltagemV))
        return -EINVAL;
    di->regulation_voltagemV = val;
    twl6030_config_voreg_reg(di, val);

    return status;
}

static ssize_t show_regulation_voltage(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->regulation_voltagemV;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_termination_current(struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
        return -EINVAL;
    di->termination_currentmA = val;
    twl6030_config_iterm_reg(di, val);

    return status;
}

static ssize_t show_termination_current(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->termination_currentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
                             struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
        return -EINVAL;
    di->charger_incurrentmA = val;
    twl6030_set_ci_limit(di, val);

    return status;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->charger_incurrentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
                                  struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
            || (val > di->max_charger_currentmA))
        return -EINVAL;
    di->charger_outcurrentmA = val;
    twl6030_config_vichrg_reg(di, val);

    return status;
}

static ssize_t show_charge_current(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->charger_outcurrentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
        return -EINVAL;
    di->min_vbus = val;
    twl6030_config_min_vbus_reg(di, val);

    return status;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->min_vbus;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_current_avg_interval(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
        return -EINVAL;
    di->current_avg_interval = val;
    twl6030_fg_mode_changed(di);

    return status;
}

static ssize_t show_current_avg_interval(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->current_avg_interval;
    return sprintf(buf, "%u\n", val);
}


static ssize_t set_monitoring_interval(struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
        return -EINVAL;
    di->monitoring_interval = val;
    twl6030_work_interval_changed(di);

    return status;
}

static ssize_t show_monitoring_interval(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->monitoring_interval;
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_bsi(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    val = twl6030_get_gpadc_conversion(di,0);
    return sprintf(buf, "%d\n", val);
}

static ssize_t show_stat1(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    unsigned val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->stat1;
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int1(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    unsigned val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->status_int1;
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int2(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    unsigned val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->status_int2;
    return sprintf(buf, "%u\n", val);
}
static ssize_t set_ext_charger(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) )
        return -EINVAL;

    BCI_NOTIFIER(&di->notifier_list, BCI_DEBUG_MONITOR, &val);

    return status;
}
static ssize_t show_ext_charger(struct device *dev,
                                struct device_attribute *attr, char *buf)
{

    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    BCI_NOTIFIER(&di->notifier_list, BCI_DEBUG_MONITOR, NULL);
    return 0;
}

static ssize_t show_regss(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    u8 value;

    pr_info("\n==============================================\n");

#define SHOW_REG(m,r) \
	value = 0; \
	twl_i2c_read_u8(m, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS1);
    SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS2);
    SHOW_REG(TWL6030_MODULE_ID0, REG_MISC1);
    SHOW_REG(TWL_MODULE_MADC, TWL6030_GPADC_CTRL);
    SHOW_REG(TWL_MODULE_MADC, TWL6030_GPADC_CTRL2);
    SHOW_REG(TWL4030_MODULE_PM_MASTER, VSYSMIN_LO_THRESHOLD);
    SHOW_REG(TWL4030_MODULE_PM_MASTER, VSYSMIN_HI_THRESHOLD);
    SHOW_REG(TWL4030_MODULE_PM_MASTER, VBATMIN_HI_THRESHOLD);
    pr_info("----------------------------------------------\n");
#undef SHOW_REG

#define SHOW_6030_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_6030_REG(CONTROLLER_INT_MASK);
    SHOW_6030_REG(CONTROLLER_CTRL1);
    SHOW_6030_REG(CONTROLLER_WDG);
    SHOW_6030_REG(CONTROLLER_STAT1);
    SHOW_6030_REG(CHARGERUSB_INT_STATUS);
    SHOW_6030_REG(CHARGERUSB_INT_MASK);
    SHOW_6030_REG(CHARGERUSB_STATUS_INT1);
    SHOW_6030_REG(CHARGERUSB_STATUS_INT2);
    SHOW_6030_REG(CHARGERUSB_CTRL1);
    SHOW_6030_REG(CHARGERUSB_CTRL2);
    SHOW_6030_REG(CHARGERUSB_CTRL3);
    SHOW_6030_REG(CHARGERUSB_VOREG);
    SHOW_6030_REG(CHARGERUSB_VICHRG);
    SHOW_6030_REG(CHARGERUSB_CINLIMIT);
    SHOW_6030_REG(CHARGERUSB_CTRLLIMIT1);
    SHOW_6030_REG(CHARGERUSB_CTRLLIMIT2);
    SHOW_6030_REG(ANTICOLLAPSE_CTRL1);

    pr_info("----------------------------------------------\n");
#undef SHOW_6030_REG

#define SHOW_6032_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_6032_REG(CONTROLLER_CTRL2);
    SHOW_6032_REG(CONTROLLER_VSEL_COMP);
    SHOW_6032_REG(CHARGERUSB_VSYSREG);
    SHOW_6032_REG(CHARGERUSB_VICHRG_PC);
    SHOW_6032_REG(LINEAR_CHRG_STS);

    pr_info("----------------------------------------------\n");
#undef SHOW_6032_REG

#define SHOW_FG_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_FG_REG(FG_REG_00);
    SHOW_FG_REG(FG_REG_01);
    SHOW_FG_REG(FG_REG_02);
    SHOW_FG_REG(FG_REG_03);
    SHOW_FG_REG(FG_REG_04);
    SHOW_FG_REG(FG_REG_05);
    SHOW_FG_REG(FG_REG_06);
    SHOW_FG_REG(FG_REG_07);
    SHOW_FG_REG(FG_REG_08);
    SHOW_FG_REG(FG_REG_09);
    SHOW_FG_REG(FG_REG_10);
    SHOW_FG_REG(FG_REG_11);

#undef SHOW_FG_REG

    pr_info("==============================================\n");

    return sprintf(buf, "See Kernel logs\n");
}

static ssize_t show_debug_level(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int val;
    //struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = debug_en;
    return sprintf(buf, "debug_level: %d\n", val);
}

static ssize_t set_debug_level(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    int tmp;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    tmp = strict_strtol(buf, 16, &val);
    if ((val < 0) || (val> 0xff)) { //debug_en: 0x00~0x80
        bci_dbg(debug_level_common, "debug level set failed, buf = %s\n", buf);
        return -EINVAL;
    }
    debug_en = val;
    bci_dbg(debug_level_common, "set debug level:0x%02x successful\n", debug_en);


    if ((debug_en&debug_level_G)>0) { //notify external to turn on debug mode
        if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER)) {
            BCI_NOTIFIER(&di->notifier_list, BCI_SET_EXT_DEBUG_EN, NULL);
        }
    }

    return status;
}

static ssize_t show_bq27520_firmware(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    bq_ver bqver;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
        BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_FIRMWARE_READ, &bqver);
    }
    return sprintf(buf, "bq27520 firmware version[%04x]   Data flash version[%04x]  Control Status[%04x]\n", bqver.Fw_version, bqver.Df_version, bqver.Control_status);
}

static ssize_t set_bq27520_firmware(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if (strict_strtol(buf, 10, &val) < 0)
        return -EINVAL;

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
        BCI_NOTIFIER_BQ(&di->notifier_list_bq, BCI_EXT_FG_FIRMWARE_WRITE, &val);
    }
    return status;
}


static ssize_t 	show_current_mA(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n",di->current_uA/1000);
}

static DEVICE_ATTR(fg_mode, S_IWUSR | S_IRUGO, show_fg_mode, set_fg_mode);
static DEVICE_ATTR(charge_src, S_IWUSR | S_IRUGO, show_charge_src,
                   set_charge_src);
static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(watchdog, S_IWUSR | S_IRUGO, show_watchdog, set_watchdog);
static DEVICE_ATTR(fg_counter, S_IRUGO, show_fg_counter, NULL);
static DEVICE_ATTR(fg_accumulator, S_IRUGO, show_fg_accumulator, NULL);
static DEVICE_ATTR(fg_offset, S_IRUGO, show_fg_offset, NULL);
static DEVICE_ATTR(fg_clear, S_IWUSR, NULL, set_fg_clear);
static DEVICE_ATTR(fg_cal, S_IWUSR, NULL, set_fg_cal);
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO, NULL, set_charging);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
                   show_regulation_voltage, set_regulation_voltage);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
                   show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
                   set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
                   set_charge_current);
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);
static DEVICE_ATTR(monitoring_interval, S_IWUSR | S_IRUGO,
                   show_monitoring_interval, set_monitoring_interval);
static DEVICE_ATTR(current_avg_interval, S_IWUSR | S_IRUGO,
                   show_current_avg_interval, set_current_avg_interval);
static DEVICE_ATTR(bsi, S_IRUGO, show_bsi, NULL);
static DEVICE_ATTR(stat1, S_IRUGO, show_stat1, NULL);
static DEVICE_ATTR(status_int1, S_IRUGO, show_status_int1, NULL);
static DEVICE_ATTR(status_int2, S_IRUGO, show_status_int2, NULL);
static DEVICE_ATTR(ext_charger,  S_IWUSR | S_IRUGO, show_ext_charger, set_ext_charger);
static DEVICE_ATTR(regs,  S_IRUGO, show_regss, NULL);
static DEVICE_ATTR(debug_level,  S_IWUSR | S_IRUGO, show_debug_level, set_debug_level);
static DEVICE_ATTR(bq_fw,  S_IWUSR | S_IRUGO, show_bq27520_firmware, set_bq27520_firmware);
static DEVICE_ATTR(current_mA, S_IRUGO, show_current_mA, NULL);

static struct attribute *twl6030_bci_attributes[] = {
    &dev_attr_fg_mode.attr,
    &dev_attr_charge_src.attr,
    &dev_attr_vbus_voltage.attr,
    &dev_attr_id_level.attr,
    &dev_attr_watchdog.attr,
    &dev_attr_fg_counter.attr,
    &dev_attr_fg_accumulator.attr,
    &dev_attr_fg_offset.attr,
    &dev_attr_fg_clear.attr,
    &dev_attr_fg_cal.attr,
    &dev_attr_charging.attr,
    &dev_attr_regulation_voltage.attr,
    &dev_attr_termination_current.attr,
    &dev_attr_cin_limit.attr,
    &dev_attr_charge_current.attr,
    &dev_attr_min_vbus.attr,
    &dev_attr_monitoring_interval.attr,
    &dev_attr_current_avg_interval.attr,
    &dev_attr_bsi.attr,
    &dev_attr_stat1.attr,
    &dev_attr_status_int1.attr,
    &dev_attr_status_int2.attr,
    &dev_attr_ext_charger.attr,
    &dev_attr_regs.attr,
    &dev_attr_debug_level.attr,
    &dev_attr_bq_fw.attr,
    &dev_attr_current_mA.attr,	
    NULL,
};
#ifdef CONFIG_EARLYSUSPEND
static void twl603x_bci_early_suspend(struct early_suspend *handler)
{
    struct twl6030_bci_device_info *di;
    di = container_of(handler, struct twl6030_bci_device_info, early_suspend);
    di->bscreen_off = 1;
    bci_dbg(debug_level_common, "%s()\n", __func__); //screen off

}

static void twl603x_bci_early_resume(struct early_suspend *handler)
{
    struct twl6030_bci_device_info *di;


    di = container_of(handler, struct twl6030_bci_device_info, early_suspend);

    bci_dbg(debug_level_common, "%s()\n", __func__); //screen on
    di->bscreen_off = 0;

}
#endif
static const struct attribute_group twl6030_bci_attr_group = {
    .attrs = twl6030_bci_attributes,
};

static char *twl6030_bci_supplied_to[] = {
    "twl6030_battery",
};

static int __devinit twl6030_bci_battery_probe(struct platform_device *pdev)
{
    struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
    struct twl6030_bci_device_info *di;
    int irq;
    int ret;

    bci_dbg(debug_level_common, "%s()\n",__func__);
    if (twl603x_bci) {
        bci_dbg(debug_level_common, "assign 603x to di \n");
        di=twl603x_bci;
    } else {
        bci_dbg(debug_level_common, "bci alloc \n");
        di = kzalloc(sizeof(*di), GFP_KERNEL);
        if (!di)
            return -ENOMEM;
    }

    if (!pdata) {
        bci_dbg(debug_level_common, "platform_data not available\n");
        ret = -EINVAL;
        goto err_pdata;
    }

    if (pdata->monitoring_interval == 0) {
        di->monitoring_interval = 10;
        di->current_avg_interval = 10;
    } else {
        di->monitoring_interval = pdata->monitoring_interval;
        di->current_avg_interval = pdata->monitoring_interval;
    }

    di->max_backupbat_voltagemV = pdata->max_backupbat_voltagemV;

	di->no_main_battery = pdata->no_main_battery;
	di->no_main_battery_monitoring_interval = pdata->no_main_battery_monitoring_interval;
	if (di->no_main_battery) {
		if (di->no_main_battery_monitoring_interval == 0)
			di->no_main_battery_monitoring_interval = 30 * 1000; /* 30 secs */
		dev_info(&pdev->dev, "[BCI]no main battery, charging disabled (%u secs)\n", di->no_main_battery_monitoring_interval / 1000);
	}

    di->dev = &pdev->dev;

    di->features = pdata->features;
    di->use_eeprom_config = pdata->use_eeprom_config;

    if (pdata->use_eeprom_config) {
        di->max_charger_currentmA = twl6030_get_co_hard_limit(di);
        di->max_charger_voltagemV = twl6030_get_vo_hard_limit(di);
        di->termination_currentmA = twl6030_get_iterm_reg(di);
        di->regulation_voltagemV = twl6030_get_voreg_reg(di);
        di->low_bat_voltagemV = pdata->low_bat_voltagemV;
        dev_dbg(di->dev, "[BCI]EEPROM max charge %d mA\n",
                di->max_charger_currentmA);
        dev_dbg(di->dev, "[BCI]EEPROM max voltage %d mV\n",
                di->max_charger_voltagemV);
        dev_dbg(di->dev, "[BCI]EEPROM termination %d mA\n",
                di->termination_currentmA);
        dev_dbg(di->dev, "[BCI]EEPROM regulation %d mV\n",
                di->regulation_voltagemV);
    } else {
        di->max_charger_currentmA = pdata->max_charger_currentmA;
        di->max_charger_voltagemV = pdata->max_bat_voltagemV;
        di->termination_currentmA = pdata->termination_currentmA;
        di->regulation_voltagemV = pdata->max_bat_voltagemV;
        di->low_bat_voltagemV = pdata->low_bat_voltagemV;
    }

    di->battery_tmp_tbl_603x = (struct ntc_res_temp_desc*)pdata->battery_tmp_tbl_603x;
    di->tblsize = pdata->tblsize;
    di->loading_Table= pdata->loading_Table;
    di->bFlagMonitorStart =0;

    di->use_hw_charger = pdata->use_hw_charger;
    di->fg_full_uAh= pdata->capacity_mAh*1000;
    di->battery_esr= pdata->battery_esr;
    di->bscreen_off = 0; //default screen on, so do not apply
    chip_config(di);/*0004615: [FR] [PM] config bci temperature Rx, Ry by board id*/

    di->dev = &pdev->dev;
    di->bat.name = "twl6030_battery";
    di->bat.supplied_to = twl6030_bci_supplied_to;
    di->bat.num_supplicants = ARRAY_SIZE(twl6030_bci_supplied_to);
    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    di->bat.properties = twl6030_bci_battery_props;
    di->bat.num_properties = ARRAY_SIZE(twl6030_bci_battery_props);
    di->bat.get_property = twl6030_bci_battery_get_property;
    /*di->bat.external_power_changed =
    		twl6030_bci_battery_external_power_changed;*/
    di->bat_health = POWER_SUPPLY_HEALTH_GOOD;

    di->usb.name = "twl6030_usb";
    di->usb.type = POWER_SUPPLY_TYPE_USB;
    di->usb.properties = twl6030_usb_props;
    di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
    di->usb.get_property = twl6030_usb_get_property;
    di->usb.external_power_changed =
        twl6030_bci_battery_external_power_changed;

    di->ac.name = "twl6030_ac";
    di->ac.type = POWER_SUPPLY_TYPE_MAINS;
    di->ac.properties = twl6030_ac_props;
    di->ac.num_properties = ARRAY_SIZE(twl6030_ac_props);
    di->ac.get_property = twl6030_ac_get_property;
    di->ac.external_power_changed =
        twl6030_bci_battery_external_power_changed;

    di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

    di->bk_bat.name = "twl6030_bk_battery";
    di->bk_bat.type = POWER_SUPPLY_TYPE_UPS;
    di->bk_bat.properties = twl6030_bk_bci_battery_props;
    di->bk_bat.num_properties = ARRAY_SIZE(twl6030_bk_bci_battery_props);
    di->bk_bat.get_property = twl6030_bk_bci_battery_get_property;

    di->vac_priority = 2;
    di->capacity =twl6030_read_last_capacity();
    bci_dbg(debug_level_common, "bci capacity = %d \n", di->capacity);
    di->capacity_debounce_count = 0;
    di->ac_last_refresh = jiffies;
    di->bci_probe_time = jiffies;
    platform_set_drvdata(pdev, di);

    wake_lock_init(&di->wakelock, WAKE_LOCK_SUSPEND, "bci_wakelock");
    wake_lock_init(&resume_wakelock, WAKE_LOCK_SUSPEND, "bci_resume");

    /* settings for temperature sensing */
    ret = twl6030battery_temp_setup(true);

    if (ret)
        goto temp_setup_fail;

    /* request charger fault interruption */
    irq = platform_get_irq(pdev, 1);

    ret = request_threaded_irq(irq, NULL, twl6030charger_fault_interrupt,
                               0, "twl_bci_fault", di);

    if (ret) {
        bci_dbg(debug_level_common, "could not request irq %d, status %d\n",
                irq, ret);
        goto temp_setup_fail;
    }
    if(machine_is_omap4_zeus()){
        if (gpio_is_valid(48)) {
      
            ret = request_threaded_irq(gpio_to_irq(48), NULL, twl6030charger_vbat_low_interrupt,
                               IRQF_TRIGGER_RISING, "twl_bci_bat_low", di);
            if (ret) {
                bci_dbg(debug_level_common, "could not request bat low irq 1 %d, status %d\n",
                irq, ret);
            }                
	 
        }else{
            printk("[BCI]gpio 48 invalid\n");
        }
    }

    /* request charger ctrl interruption */
    irq = platform_get_irq(pdev, 0);

    ret = request_threaded_irq(irq, NULL, twl6030charger_ctrl_interrupt,
                               0, "twl_bci_ctrl", di);


    if (ret) {
        bci_dbg(debug_level_common, "could not request irq %d, status %d\n",
                irq, ret);
        goto chg_irq_fail;
    }

    ret = power_supply_register(&pdev->dev, &di->bat);
    if (ret) {
        bci_dbg(debug_level_common, "failed to register main battery\n");
        goto batt_failed;
    }


    INIT_DELAYED_WORK_DEFERRABLE(&di->bci_monitor_work,
                                 twl6030_bci_battery_work);
    INIT_DELAYED_WORK_DEFERRABLE(&di->ir_handler_work,
                                 irq_handler_push_ex);


    ret = power_supply_register(&pdev->dev, &di->usb);
    if (ret) {
        bci_dbg(debug_level_common, "failed to register usb power supply\n");
        goto usb_failed;
    }

    ret = power_supply_register(&pdev->dev, &di->ac);
    if (ret) {
        bci_dbg(debug_level_common, "failed to register ac power supply\n");
        goto ac_failed;
    }

    di->fg_acc = 0;
    di->fg_timer = 0;

    ret = twl6030battery_voltage_setup(di);
    if (ret)
        bci_dbg(debug_level_common, "voltage measurement setup failed\n");

    ret = twl6030battery_current_setup(true);
    if (ret)
        bci_dbg(debug_level_common, "current measurement setup failed\n");

    if (!pdata->use_eeprom_config) {
        /* initialize for USB charging */
        twl6030_set_vo_hard_limit(di, di->max_charger_voltagemV);
		/* lock CHARGERUSB_CTRLLIMIT2 and CHARGERUSB_CTRLLIMIT1 */
        twl6030_set_co_hard_limit(di, di->max_charger_currentmA); 

    }
	
	{
		u8 charge_state,rd_reg;
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
					CONTROLLER_STAT1);
		if (charge_state & CONTROLLER_STAT1_EXTCHRG_STATZ) { // when no charger is attached
			printk("[BCI]no external charger is attached\n");
			twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
			rd_reg |= (MVAC_FAULT | MAC_EOC | MBAT_TEMP);
			twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg,
				         CONTROLLER_INT_MASK);
		} else {
			twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
				         CONTROLLER_INT_MASK);
		}
	}
	
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MASK_MCHARGERUSB_THMREG,
                     CHARGERUSB_INT_MASK);
	if (machine_is_omap4_saga()) {
		/* continue to charge when VBUS in. */
		twl6032_enable_power_path(di);
	} else {
		/* disable power path in order to SW gas gauge calculate. */
		twl6032_disable_power_path(di);
	}
	twl6030_fg_sync(di); /*TWL6032 internal fuel gauge current flow detect mechanism works begin*/
    twl6030_fg_update(di);/*update fg_acc and fg_time for next current*/

    di->fg_acc_init=di->fg_acc;
    di->fg_timer_init=di->fg_timer;
    di->fg_timer_avg=di->fg_timer;
    di->fg_acc_avg=di->fg_acc;
    di->current_avg_uA =0;
    di->current_uA =0;
    di->fg_acc_delta =0;
    if (!(di->features & TWL6032_SUBCLASS)) {
        /*TWL6030 without linear charger*/
        di->use_linear_charger=0;
    } else {
        di->use_linear_charger=1;

    }
#ifdef POWEROFF_CHARGE
    di->boot_mode = android_boot_get_mode();
#endif

    di->charger_incurrentmA = di->max_charger_currentmA;
    di->charger_outcurrentmA = di->max_charger_currentmA;
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, 0x04,ANTICOLLAPSE_CTRL1);

    di->watchdog_duration = 32;
    if (di->features & TWL6032_SUBCLASS) {

        twl6030_enable_dppm(di,1);

        di->voltage_mV = twl6030_get_gpadc_conversion(di,18);

        if (di->use_linear_charger) {

            twl6030_set_vbat_tracking_voltage(di,200);
            //twl6032_set_tracking_mode_vsys_threshold(di,4300);

            //twl6030_set_Q2_Q3_limit(di,3700);
            /*Disable Linear Charger before Fuel Gauge initialized anyway*/
            twl6032_stop_linear_charger(di);

        } else {
            /*Disable Charger before Fuel Gauge initialized*/
            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                             0, CONTROLLER_CTRL1);

        }
    } else {
        u8 charge_state,hw_state;
        /* read charger controller_stat1 */
        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                        CONTROLLER_STAT1);
        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);

        di->voltage_mV = twl6030_get_gpadc_conversion(di,7);
        twl6030_config_min_vbus_reg(di,di->regulation_voltagemV);

        if ((charge_state&VBUS_DET)&&!(hw_state & STS_USB_ID)) {
            /*Disable Charger before Fuel Gauge initialized*/
            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                             0, CONTROLLER_CTRL1);
        }
    }

    di->regressin_Voltage_mV=di->regulation_voltagemV;/*Todo: need always regression??*/

    dev_info(&pdev->dev, "[BCI]Battery Voltage at Bootup is %d mV\n",
             di->voltage_mV);
    twl6030backupbatt_setup(di, true);
    di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
    di->charge_prev_status= POWER_SUPPLY_STATUS_UNKNOWN;
	di->ac_charger_enable = false;
	di->bci_notifier_shutdown = false;
	di->dynamic_charging.enable = false;
	mutex_init(&di->ctrl_irq_mutex); 	
	di->gas_gauge.can_read = false;
	sema_init(&(di->gas_gauge.sem), 0);
	
    if (pdata->poll_msecs)
        di->timer_msecs = pdata->poll_msecs;
    else
        di->timer_msecs = 5000;

    spin_lock_init(&di->timer_lock);


    di->nb.notifier_call = twl6030_usb_notifier_call;
    di->otg = otg_get_transceiver();
    ret = otg_register_notifier(di->otg, &di->nb);
    if (ret)
        dev_err(&pdev->dev, "[BCI]otg register notifier failed %d\n", ret);

    /*Start Monitor Work*/
    schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(2000));

	/* /sys/devices/platform/omap/omap_i2c.1/i2c-1/1-0049/twl6030_bci */
    ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    if (ret)
        dev_dbg(&pdev->dev, "[BCI]could not create sysfs files\n");

    ret = irq_handler_init(di);
    if (ret<0)
        bci_dbg(debug_level_common, "irq_handler_init() failed\n");


#ifdef CONFIG_EARLYSUSPEND
    bci_dbg(debug_level_common, "register bci early suspend\n ");
    di->early_suspend.suspend = twl603x_bci_early_suspend;
    di->early_suspend.resume = twl603x_bci_early_resume;
    register_early_suspend(&di->early_suspend);
#endif

    if (!twl603x_bci) {
        bci_dbg(debug_level_common, "assign di to twl603x_bci\n");
        twl603x_bci = di;
    }
	printk("[BCI]twl6030_bci_battery_probe success!!\n");
    return 0;


ac_failed:
    power_supply_unregister(&di->usb);
usb_failed:
    cancel_delayed_work(&di->bci_monitor_work);
    power_supply_unregister(&di->bat);
batt_failed:
    free_irq(irq, di);
chg_irq_fail:
    irq = platform_get_irq(pdev, 1);
    free_irq(irq, di);
temp_setup_fail:
    wake_lock_destroy(&di->wakelock);
    wake_lock_destroy(&resume_wakelock);
    platform_set_drvdata(pdev, NULL);
err_pdata:
    kfree(di);
	twl603x_bci = NULL;

    return ret;
}

static int __devexit twl6030_bci_battery_remove(struct platform_device *pdev)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
    int irq;

    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_STS_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_STS_C);

    irq = platform_get_irq(pdev, 0);
    free_irq(irq, di);

    irq = platform_get_irq(pdev, 1);
    free_irq(irq, di);

    otg_unregister_notifier(di->otg, &di->nb);
    sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    cancel_delayed_work_sync(&di->bci_monitor_work);

    //flush_scheduled_work();
    
    power_supply_unregister(&di->bat);
    power_supply_unregister(&di->usb);
    power_supply_unregister(&di->ac);
    power_supply_unregister(&di->bk_bat);
    wake_lock_destroy(&di->wakelock);
    wake_lock_destroy(&resume_wakelock);
    platform_set_drvdata(pdev, NULL);
	mutex_destroy(&di->ctrl_irq_mutex);
    kfree(di);
	twl603x_bci = NULL;
    return 0;
}

#ifdef CONFIG_PM
static int twl6030_bci_battery_suspend(struct platform_device *pdev,
                                       pm_message_t state)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);

    u8 rd_reg = 0;
    int ret;
    /* mask to prevent wakeup due to 32s timeout from External charger */
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg |= (MVAC_FAULT | MAC_EOC | MBAT_TEMP);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg,
                     CONTROLLER_INT_MASK);

    cancel_delayed_work(&di->bci_monitor_work);


	/* We cannot tolarate a sleep longer than 30 seconds
	 * while on ac charging we have to reset the BQ watchdog timer.
	 */
    if ((di->charger_source == POWER_SUPPLY_TYPE_MAINS) &&
            ((wakeup_timer_seconds > 25) || !wakeup_timer_seconds)) {
        wakeup_timer_seconds = 25;
    }

    /*reset the BQ watch dog*/
    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER))
        BCI_NOTIFIER(&di->notifier_list, BCI_RESET_TIMER, NULL);
	
    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->fg_timer_suspend,
                 FG_REG_01, 3);
	
	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->fg_acc_suspend,
                 FG_REG_04, 4);
    if (di->max_backupbat_voltagemV > 0) {
        twl6030backupbatt_setup(di, true);
    } else {
        twl6030backupbatt_setup(di, false);
    }

    ret = twl6030battery_temp_setup(false);
    if (ret) {
        pr_err("%s: Temp measurement setup failed (%d)!\n",
               __func__, ret);
        return ret;
    }


    return 0;
}

static int twl6030_bci_battery_resume(struct platform_device *pdev)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);

    u8 rd_reg = 0;
    int ret = 0;
    int timer_suspend=0,charge_suspend,samples_avg;//,cc_offset_avg,current_avg_uA;
    int time_interval = 0;
	int	fg_acc_delta = 0;
	
    if (!twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_FUELGAUGE)) {
	    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &timer_suspend,
	                 FG_REG_01, 3);
		/*
		 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
		 * accumulates instantaneous current value
		 */
	    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &charge_suspend,
	                 FG_REG_04, 4);
	    samples_avg = timer_suspend - di->fg_timer_suspend;

	    if (samples_avg) {
	        /* check for timer overflow */
	        if (timer_suspend < di->fg_timer_suspend)
	            samples_avg = samples_avg + (1 << 24);
			
	        time_interval = samples_avg /fuelgauge_rate[di->fuelgauge_mode];		
	        di->current_suspend_uA = twl6030_fg_avg_sample_current_uA(di,
	                                             charge_suspend - di->fg_acc_suspend ,samples_avg);
			if (di->current_suspend_uA < 0) {
				fg_acc_delta = di->current_suspend_uA * time_interval/3600;
				di->fg_acc_uAh += fg_acc_delta;
		        printk(KERN_INFO"[BCI][suspend] Avg current %d uA, time_interval = %d s, fg_acc_delta = %d uAh,fg_acc_uAh = %d uAh\n",
					di->current_suspend_uA,time_interval,fg_acc_delta,di->fg_acc_uAh);
			}
	    }
	}
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg |= (MVAC_FAULT | MAC_EOC | MBAT_TEMP);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg,
                     CONTROLLER_INT_MASK);

    twl6030backupbatt_setup(di, true);
    
    ret = twl6030battery_temp_setup(true);
    if (ret) {
        pr_err("%s: Temp measurement setup failed (%d)!\n",
               __func__, ret);
        return ret;
    }

    twl6030battery_voltage_setup(di);

#if 0
    schedule_delayed_work_on(0,&di->bci_monitor_work, msecs_to_jiffies(100));/*mantis: 4628 [TR] the battery voltage will down to 2xx mV when resume	*/
#else
	cancel_delayed_work(&di->bci_monitor_work);
	schedule_delayed_work_on(0,&di->bci_monitor_work, HZ);/*mantis: 4628 [TR] the battery voltage will down to 2xx mV when resume	*/

	/* Hold a 3-sec wakelock to avoid the immediate Linux PM suspend */
	wake_lock_timeout(&resume_wakelock, 3 * HZ);
#endif

    if (twl6030_bci_notifier_is_registered(NOTIFIER_REG_EXT_CHARGER))
        BCI_NOTIFIER(&di->notifier_list, BCI_RESET_TIMER, NULL);

    return 0;
}
static void  twl6030_bci_battery_shutdown(struct platform_device *pdev) {
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
    u8 controller_stat=0;
    int irq;
	
	
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,
                    CONTROLLER_STAT1);
    /*Stop Charging that Fuel Gauge can lookup loading table*/
    if (controller_stat & VBUS_DET) {
        u8 hw_state;
        /*
         * In HOST mode (ID GROUND) with a device connected,
         * do no enable usb charging
         */
        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
        if (!(hw_state & STS_USB_ID)) {
            twl6030_stop_usb_charger(di);
        }
    }


    if (controller_stat & VAC_DET) {
        twl6030_stop_ac_charger(di);
    }


    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_STS_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_STS_C);

    irq = platform_get_irq(pdev, 0);
    free_irq(irq, di);

    irq = platform_get_irq(pdev, 1);
    free_irq(irq, di);

    otg_unregister_notifier(di->otg, &di->nb);
    sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    cancel_delayed_work_sync(&di->bci_monitor_work);

    //flush_scheduled_work();
//    power_supply_unregister(&di->bat);
//    power_supply_unregister(&di->usb);
//    power_supply_unregister(&di->ac);
//    power_supply_unregister(&di->bk_bat);
    wake_lock_destroy(&di->wakelock);
    platform_set_drvdata(pdev, NULL);
	mutex_destroy(&di->ctrl_irq_mutex);		
    kfree(di);
	twl603x_bci = NULL;

}

#else
#define twl6030_bci_battery_suspend	NULL
#define twl6030_bci_battery_resume	NULL
#define twl6030_bci_battery_shutdown NULL
#endif /* CONFIG_PM */


static struct platform_driver twl6030_bci_battery_driver = {
    .probe		= twl6030_bci_battery_probe,
    .remove		= __devexit_p(twl6030_bci_battery_remove),
    .suspend	= twl6030_bci_battery_suspend,
    .resume		= twl6030_bci_battery_resume,
    .shutdown =	twl6030_bci_battery_shutdown,
    .driver		= {
        //.name	= "twl603x_bci",
		.name	= "twl6030_bci",	
    },
};

static int __init twl6030_battery_init(void)
{
    return platform_driver_register(&twl6030_bci_battery_driver);
}
module_init(twl6030_battery_init);

static void __exit twl6030_battery_exit(void)
{
    platform_driver_unregister(&twl6030_bci_battery_driver);
}
module_exit(twl6030_battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_bci");
MODULE_AUTHOR("Texas Instruments Inc");


