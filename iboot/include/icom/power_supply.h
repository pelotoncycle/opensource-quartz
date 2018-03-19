/*
 * (C) Copyright 2012
 * InnoComm Mobile Technology Corp.
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

#ifndef _POWER_SUPPLY_H
#define _POWER_SUPPLY_H

/*----------------------------------------------------------------------*/

#include <common.h>
#include <linux/notifier.h>

/*----------------------------------------------------------------------*/

#ifndef INT_MAX
#define INT_MAX		((int)(~0U>>1))
#endif
#ifndef INT_MIN
#define INT_MIN		(-INT_MAX - 1)
#endif

#define CONFIG_POWER_SUPPLY_INVALID_CURRENT	INT_MIN
#define CONFIG_POWER_SUPPLY_INVALID_VOLTAGE	(-999)

/*----------------------------------------------------------------------*/

#ifndef CONFIG_POWER_SUPPLY_MAX_CHARGER_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MAX_CHARGER_VOLTAGEMV		4560	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_MAX_CHARGER_CURRENTMA
#define CONFIG_POWER_SUPPLY_MAX_CHARGER_CURRENTMA		1500
#endif
#ifndef CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV			4200	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV		4100	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV	2200	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA
#define CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA		50	/* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA
#define CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA			500	/* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_CHARGE_CURRENTMA
#define CONFIG_POWER_SUPPLY_USB_CHARGE_CURRENTMA		500	/* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA
#define CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA		1800 /* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_AC_CHARGE_CURRENTMA
#define CONFIG_POWER_SUPPLY_USB_AC_CHARGE_CURRENTMA		1500 /* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_AC_INPUT_CURRENTMA
#define CONFIG_POWER_SUPPLY_AC_INPUT_CURRENTMA			1500 /* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_AC_CHARGE_CURRENTMA
#define CONFIG_POWER_SUPPLY_AC_CHARGE_CURRENTMA			1000 /* mA */
#endif
#ifndef CONFIG_POWER_SUPPLY_AC_DPM_VOLTAGE
#define CONFIG_POWER_SUPPLY_AC_DPM_VOLTAGE				4520 /* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_USB_DPM_VOLTAGE
#define CONFIG_POWER_SUPPLY_USB_DPM_VOLTAGE				4520 /* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV
#define CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV			750	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV
#define CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV				240	/* mV */
#endif
#ifndef CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV
#define CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV	0	/* mV */
#endif

/*----------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_POWER_SUPPLY_DEBUG
#define CONFIG_POWER_SUPPLY_DEBUG
#endif
#ifndef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
#define CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_POWER_SUPPLY_DEBUG
#ifdef POWER_SUPPLY_SUBSYS_NAME
#define POWER_SUPPLY_DBG(fmt, args...) \
	do {printf("[power supply] " POWER_SUPPLY_SUBSYS_NAME ": " fmt, ##args);} while (0)
#define POWER_SUPPLY_DBGF(fmt, args...) \
	do {printf("[power supply] " POWER_SUPPLY_SUBSYS_NAME ": %s(" fmt  ")\n", __func__, ##args);} while (0)
#else
#define POWER_SUPPLY_DBG(fmt, args...) \
	do {printf("[power supply] " fmt, ##args);} while (0)
#define POWER_SUPPLY_DBGF(fmt, args...) \
	do {printf("[power supply] %s(" fmt  ")\n", __func__, ##args);} while (0)
#endif /* POWER_SUPPLY_SUBSYS_NAME */
#else /* CONFIG_POWER_SUPPLY_DEBUG */
#define POWER_SUPPLY_DBG(fmt, args...) \
	do {} while (0)
#define POWER_SUPPLY_DBGF(fmt, args...) \
	do {} while (0)
#endif /* CONFIG_POWER_SUPPLY_DEBUG */

#ifdef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
#ifdef POWER_SUPPLY_SUBSYS_NAME
#define POWER_SUPPLY_VDBG(fmt, args...) \
	do {printf("[power supply] " POWER_SUPPLY_SUBSYS_NAME ": " fmt, ##args);} while (0)
#define POWER_SUPPLY_VDBGF(fmt, args...) \
	do {printf("[power supply] " POWER_SUPPLY_SUBSYS_NAME ": %s(" fmt  ")\n", __func__, ##args);} while (0)
#else
#define POWER_SUPPLY_VDBG(fmt, args...) \
	do {printf("[power supply] " fmt, ##args);} while (0)
#define POWER_SUPPLY_VDBGF(fmt, args...) \
	do {printf("[power supply] %s(" fmt  ")\n", __func__, ##args);} while (0)
#endif /* POWER_SUPPLY_SUBSYS_NAME */
#else /* CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */
#define POWER_SUPPLY_VDBG(fmt, args...) \
	do {} while (0)
#define POWER_SUPPLY_VDBGF(fmt, args...) \
	do {} while (0)
#endif /* CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */

#ifdef POWER_SUPPLY_SUBSYS_NAME
#define POWER_SUPPLY_ERR(fmt, args...) \
	do {printf(POWER_SUPPLY_SUBSYS_NAME ": " fmt, ##args);} while (0)
#else
#define POWER_SUPPLY_ERR(fmt, args...) \
	do {printf("power supply: " fmt, ##args);} while (0)
#endif /* POWER_SUPPLY_SUBSYS_NAME */

#ifdef POWER_SUPPLY_SUBSYS_NAME
#define POWER_SUPPLY_INFO(fmt, args...) \
	do {printf(POWER_SUPPLY_SUBSYS_NAME ": " fmt, ##args);} while (0)
#else
#define POWER_SUPPLY_INFO(fmt, args...) \
	do {printf("power supply: " fmt, ##args);} while (0)
#endif /* POWER_SUPPLY_SUBSYS_NAME */

#ifdef POWER_SUPPLY_SUBSYS_NAME
#define POWER_SUPPLY_WARN(fmt, args...) \
	do {printf(POWER_SUPPLY_SUBSYS_NAME ": " fmt, ##args);} while (0)
#else
#define POWER_SUPPLY_WARN(fmt, args...) \
	do {printf("power supply: " fmt, ##args);} while (0)
#endif /* POWER_SUPPLY_SUBSYS_NAME */

#define POWER_SUPPLY_PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

/* Features */
enum power_supply_features {
	/* USB charger */
	POWER_SUPPLY_FEAT_INT_USB_CHARGER	= 0x00000001,
	POWER_SUPPLY_FEAT_EXT_USB_CHARGER	= 0x00000002,
	/* AC charger */
	POWER_SUPPLY_FEAT_EXT_AC_CHARGER	= 0x00000010,
	/* Fuel Gauge */
	POWER_SUPPLY_FEAT_INT_FUELGAUGE		= 0x00000100,
	POWER_SUPPLY_FEAT_EXT_FUELGAUGE		= 0x00000200,
};

/* Charger events */
enum power_supply_charger_events {
/*0*/	POWER_SUPPLY_CHARGER_INIT = 0,
/*1*/	POWER_SUPPLY_CHARGER_DUMP_REGS,
/*2*/	POWER_SUPPLY_CHARGER_START,
/*3*/	POWER_SUPPLY_CHARGER_STOP,
/*4*/	POWER_SUPPLY_CHARGER_RESET_TIMER,
/*5*/	POWER_SUPPLY_CHARGER_SET_CURRENT,
/*6*/	POWER_SUPPLY_CHARGER_UPDATE_PROFILE,
/*7*/	POWER_SUPPLY_CHARGER_IS_CHARGING, /* See POWER_SUPPLY_CHARGER_STATE_XXX */
/*8*/	POWER_SUPPLY_CHARGER_IS_CHARGING_NO_RESTART, /* See POWER_SUPPLY_CHARGER_STATE_XXX */
/*9*/	POWER_SUPPLY_CHARGER_ADD_ERROR,	/* 0:Don't restart charger; 1:Restart charger */
/*10*/	POWER_SUPPLY_CHARGER_CLEAR_ERROR,
/*11*/	POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY,	/* 0:No Battery; 1:Has Battery; -1:Not Implemented; */
/*12*/	POWER_SUPPLY_CHARGER_IS_BATTERY_LOCKED, /* 0:Not Locked; 1:Locked; -1:Not Implemented; */
};

/* POWER_SUPPLY_CHARGER_STATE_XXX (Charger Status) */
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY

/* Charging or Bypass the Charging Fault (> 0) */
#define POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT	5	/* Battery Temp Fault */
#define POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP	4	/* Battery OVP */
#define POWER_SUPPLY_CHARGER_STATE_NO_BATTERY	3	/* No Battery */
#define POWER_SUPPLY_CHARGER_STATE_EOC			2	/* EOC */
#define POWER_SUPPLY_CHARGER_STATE_CHARGING		1	/* Charging */
/* Not Charging or Charging Fault (<= 0) */
#define POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING	0	/* NOT Charging */

#else /* !CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

/* Charging or Bypass the Charging Fault (> 0) */
#define POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP	4	/* Battery OVP */
#define POWER_SUPPLY_CHARGER_STATE_EOC			2	/* EOC */
#define POWER_SUPPLY_CHARGER_STATE_CHARGING		1	/* Charging */
/* Not Charging or Charging Fault (<= 0) */
#define POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING	0	/* NOT Charging */
#define POWER_SUPPLY_CHARGER_STATE_NO_BATTERY	-3	/* No Battery */
#define POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT	-5	/* Battery Temp Fault */

#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

/* Fuel Gauge events */
enum power_supply_fg_events {
/*0*/	POWER_SUPPLY_FG_INIT = 0,
/*1*/	POWER_SUPPLY_FG_GET_VSYS,			/* int;mV */
/*2*/	POWER_SUPPLY_FG_GET_VBAT,			/* int;mV */
/*3*/	POWER_SUPPLY_FG_GET_VBBAT,			/* int;mV */
/*4*/	POWER_SUPPLY_FG_GET_VUSB_BUS,		/* int;mV */
/*5*/	POWER_SUPPLY_FG_GET_VAC,			/* int;mV */
/*6*/	POWER_SUPPLY_FG_GET_TEMP,			/* inti: battery temperature in units of 0.1C */
/*7*/	POWER_SUPPLY_FG_GET_CURRENT,		/* int;mA */
/*8*/	POWER_SUPPLY_FG_GET_CAPACITY,		/* int;percentage */
/*9*/	POWER_SUPPLY_FG_IS_FULL_CHARGED,	/* int */
/*10*/	POWER_SUPPLY_FG_HAS_MAIN_BATTERY,	/* int: 0:No Battery; 1:Has Battery; -1:Not Implemented; */
};

/* AC charger connection events */
enum power_supply_ac_events {
	POWER_SUPPLY_AC_EVENT_NONE = 0,	/* no events or AC charger disconnected */
	POWER_SUPPLY_AC_EVENT_CHARGER,	/* AC dedicated charger */
};

/*----------------------------------------------------------------------*/

struct power_supply_charger_profile {
	int				battery_voltage;
	unsigned int	input_current;
	unsigned int	charge_current;
	int				extra_increase_voltage;
	int				extra_decrease_voltage;
};

/*----------------------------------------------------------------------*/

enum power_supply_type {
	POWER_SUPPLY_TYPE_UNKNOWN = 0,
	POWER_SUPPLY_TYPE_MAINS,	/* AC Dedicated Charger */
	POWER_SUPPLY_TYPE_USB,		/* Standard Downstream Port */
	POWER_SUPPLY_TYPE_USB_DCP,	/* Dedicated Charging Port */
	POWER_SUPPLY_TYPE_USB_CDP,	/* Charging Downstream Port */
	POWER_SUPPLY_TYPE_USB_ACA,	/* Accessory Charger Adapters */
};

/*----------------------------------------------------------------------*/

#ifdef CONFIG_SYS_POWER_SUPPLY
/* AC charger connection events notifier */
int ac_charger_register_notifier(struct notifier_block *nb);
/* Power Supply Feature register */
int power_supply_register_notifier(enum power_supply_features feature, struct notifier_block *nb);
int power_supply_unregister_notifier(enum power_supply_features feature, struct notifier_block *nb);

int power_supply_has_ac_feature(void);
int power_supply_has_usb_feature(void);
int power_supply_has_external_fuelgauge(void);
int power_supply_reinit_external_fuelgauge(void);
unsigned power_supply_get_charger_source(void);
unsigned power_supply_get_usb_type(void);
int power_supply_is_charging(void);
int power_supply_has_main_battery(void);
int power_supply_is_battery_locked(void);
int power_supply_get_battery_current(void);
int power_supply_get_battery_capacity(void);
int power_supply_is_full_charged(void);
int power_supply_get_backup_battery_voltage(void);
int power_supply_get_battery_voltage(void);
int power_supply_get_sys_voltage(void);
int power_supply_get_usb_voltage(void);
int power_supply_get_ac_voltage(void);
unsigned int power_supply_start_charger(int vbat);
void power_supply_stop_charger(void);
void power_supply_poll(void);
void power_supply_dump_regs(void);
void power_supply_shutdown(void);
int power_supply_add_charger_error(void);
int power_supply_clear_charger_error(void);
#else
#define ac_charger_register_notifier(nb)			(-EPERM)
#define power_supply_register_notifier(feature, nb)	(-EPERM)
#define power_supply_unregister_notifier(feature, nb)	(-EPERM)
#define power_supply_has_ac_feature()			(0)
#define power_supply_has_usb_feature()			(0)
#define power_supply_has_external_fuelgauge()	(0)
#define power_supply_reinit_external_fuelgauge()	(0)
#define power_supply_get_charger_source()		(POWER_SUPPLY_TYPE_UNKNOWN)
#define power_supply_get_usb_type()				(POWER_SUPPLY_TYPE_UNKNOWN)
#define power_supply_is_charging()				(0)
#define power_supply_get_battery_current()		(0)
#define power_supply_get_battery_capacity()		(-1)
#define power_supply_is_full_charged()	(0)
#define power_supply_get_backup_battery_voltage()	(0)
#define power_supply_get_battery_voltage()		(0)
#define power_supply_get_sys_voltage()			(0)
#define power_supply_get_usb_voltage()			(0)
#define power_supply_get_ac_voltage()			(0)
#define power_supply_start_charger(vbat)		(0)
#define power_supply_stop_charger() \
			do {} while (0)
#define power_supply_poll() \
			do {} while (0)
#define power_supply_dump_regs() \
			do {} while (0)
#define power_supply_shutdown() \
			do {} while (0)
#define power_supply_add_charger_error()		(0)
#define power_supply_clear_charger_error()		(0)
#endif /* CONFIG_SYS_POWER_SUPPLY */

/*----------------------------------------------------------------------*/

#endif /* _POWER_SUPPLY_H */
