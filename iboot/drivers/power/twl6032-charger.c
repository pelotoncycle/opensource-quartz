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

/*----------------------------------------------------------------------*/

#define POWER_SUPPLY_SUBSYS_NAME	"twl-charger"

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <watchdog.h>
#include <linux/usb/otg.h>
#include <twl6030.h>

#include <icom/power_supply.h>

/* External AC/USB Charger */
#include "bq2416x.h"

/* External Fuel Gauge */
#include "bq27541.h"
#include "bq27520.h"

/*----------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_TWL6032_CHARGER_DEBUG
#define CONFIG_TWL6032_CHARGER_DEBUG
#endif
#ifndef CONFIG_TWL6032_CHARGER_VERBOSE_DEBUG
#define CONFIG_TWL6032_CHARGER_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6032_CHARGER_DEBUG
#define CHARGER_DPRINT(fmt, args...) \
	do {printf("[twl-charger] " fmt, ##args);} while (0)
#define CHARGER_DPUTS(fmt) \
	do {puts("[twl-charger] " fmt);} while (0)
#else /* CONFIG_TWL6032_CHARGER_DEBUG */
#define CHARGER_DPRINT(fmt, args...) \
	do {} while (0)
#define CHARGER_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6032_CHARGER_DEBUG */

#ifdef CONFIG_TWL6032_CHARGER_VERBOSE_DEBUG
#define CHARGER_VPRINT(fmt, args...) \
	do {printf("[twl-charger] " fmt, ##args);} while (0)
#define CHARGER_VPUTS(fmt) \
	do {puts("[twl-charger] " fmt);} while (0)
#else /* CONFIG_TWL6032_CHARGER_VERBOSE_DEBUG */
#define CHARGER_VPRINT(fmt, args...) \
	do {} while (0)
#define CHARGER_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6032_CHARGER_VERBOSE_DEBUG */

/*----------------------------------------------------------------------*/

#define	VALIDITY0				0x17	/* iCOM: Battery capacity */

/*----------------------------------------------------------------------*/

#define CONTROLLER_INT_MASK		0x00
#define		MVAC_FAULT		(1 << 7)	/* Mask interrupt 21(EXT_CHRG) generation on an external charger fault */
#define		MAC_EOC			(1 << 6)	/* Mask interrupt 21(EXT_CHRG) generation on an external charger End Of Charge request */
#define		LINCH_GATED		(1 << 5)	/* Mask interupt generation on LINCH_GATED event (CONTROLLER_STAT1[6]) */
#define		MBAT_REMOVED	(1 << 4)	/* Mask interupt generation on BAT_REMOVED event (CONTROLLER_STAT1[1]) */
#define		MFAULT_WDG		(1 << 3)	/* Mask interupt generation on FAULT_WDG event (CONTROLLER_STAT1[4]) */
#define		MBAT_TEMP		(1 << 2)	/* Mask interupt generation on BAT_TEMP_OVRANGE event (CONTROLLER_STAT1[0]) */
#define		MVBUS_DET		(1 << 1)	/* Mask interupt generation on VBUS_DET event (CONTROLLER_STAT1[2]) */
#define		MVAC_DET		(1 << 0)	/* Mask interupt generation on VAC_DET event (CONTROLLER_STAT1[3]) */
#define CONTROLLER_CTRL1		0x01
#define		CONTROLLER_CTRL1_EN_LINCH		(1 << 5)	/* Linear charge enable if EN_CHARGE = 1 */
#define		CONTROLLER_CTRL1_EN_CHARGER		(1 << 4)	/* Charger is enabled */
#define		CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)	/* VAC input supply path is selected */
#define CONTROLLER_WDG			0x02
#define		CONTROLLER_WDG_RST			(1 << 7)		/* Reset the charging watchdog timer */
#define		CONTROLLER_WDG_WDT(s)		((s) & 0x7F)	/* Software mode charging watchdog time value, from 0 to 127 seconds */
#define CONTROLLER_STAT1		0x03
#define		CHRG_EXTCHRG_STATZ	(1 << 7)	/* CHRG_EXTCHRG_STATZ line is 1 */
#define		LINCH_GATED_STAT	(1 << 6)	/* Linear charge gated */
#define		CHRG_DET_N			(1 << 5)	/* CHRG_DET_N line is 1 */
#define		FAULT_WDG			(1 << 4)	/* Watchdog fault is active */
#define		VAC_DET				(1 << 3)	/* VAC is present */
#define		VBUS_DET			(1 << 2)	/* VBUS is present */
#define		BAT_REMOVED			(1 << 1)	/* Battery has been removed */
#define		BAT_TEMP_OVRANGE	(1 << 0)	/* Battery temperature measurement is out of range */
#define CHARGERUSB_INT_STATUS	0x04
#define		EN_LINCH			(1 << 4)	/* Linear charge has started */
#define		CURRENT_TERM		(1 << 3)	/* VBUS charger current termination has been reached */
#define		CHARGERUSB_STAT		(1 << 2)	/* CHARGERUSB_STATUS_INT2 [3:0] value change */
#define		CHARGERUSB_THMREG	(1 << 1)	/* CHARGERUSB_STATUS_INT1 [7] value change */
#define		CHARGERUSB_FAULT	(1 << 0)	/* CHARGERUSB_STATUS_INT1 [6:0] value change */
#define CHARGERUSB_INT_MASK		0x05
#define		MCURRENT_TERM		(1 << 3)	/* Mask CURRENT_TERM interrupt */
#define		MCHARGERUSB_STAT	(1 << 2)	/* Mask CHARGERUSB_STAT interrupt */
#define		MCHARGERUSB_THMREG	(1 << 1)	/* Mask CHARGERUSB_THMREG interrupt */
#define		MCHARGERUSB_FAULT	(1 << 0)	/* Mask CHARGERUSB_FAULT interrupt */
#define CHARGERUSB_STATUS_INT1	0x06
#define		TMREG			(1 << 7)	/* Thermal regulation is on-going */
#define		NO_BAT			(1 << 6)	/* No battery is currently present */
#define		BST_OCP			(1 << 5)	/* Boost over current load */
#define		TH_SHUTD		(1 << 4)	/* Charger thermal shutdown occurred */
#define		BAT_OVP			(1 << 3)	/* Battery over voltage */
#define		POOR_SRC		(1 << 2)	/* Poor input source */
#define		SLP_MODE		(1 << 1)	/* Sleep mode (VBAT ≥ VBUS in charger mode)  */
#define		VBUS_OVP		(1 << 0)	/* VBUS over voltage */
#define CHARGERUSB_STATUS_INT2	0x07
#define		ICCLOOP				(1 << 3)	/* Input current (VBUS), control loop is acting */
#define		CURRENT_TERM_STAT	(1 << 2)	/* Current termination mode has occurred */
#define		CHARGE_DONE			(1 << 1)	/* Reached CHARGE_DONE state */
#define		ANTICOLLAPSE		(1 << 0)	/* Anticollapse feature is currently activated */
#define CHARGERUSB_CTRL1		0x08
#define		SUSPEND_BOOT	(1 << 7)	/* Suspend boot is gating the USB charge */
#define		OPA_MODE		(1 << 6)	/* Boost mode for OTG purpose, hardware protection is set to forbid the charge */
#define		HZ_MODE			(1 << 5)	/* USB charger is in high-impedance mode */
#define		TERM			(1 << 4)	/* Enable charge current termination */
#define CHARGERUSB_CTRL2		0x09	/* Termination current level */
#define CHARGERUSB_CTRL3		0x0A
#define		VBUSCHRG_LDO_OVRD	(1 << 7)	/* Force the LDOUSB LDO to turn on */
#define		CHARGE_ONCE			(1 << 6)	/* Charge once feature is turned on */
#define		BST_HW_PR_DIS		(1 << 5)	/* Boost stays ON */
#define		AUTOSUPPLY			(1 << 3)	/* Q1 is disabled (Q1 blocking FET affects only the boost) */
#define		GSMCAL_TM			(1 << 2)	/* Enable production test mode for GSM calibration in factory */
#define		BUCK_HSILIM_3A		(1 << 1)	/* Change cycle by cycle current limit for buck regulator */
#define		BUCK_HSILIM			(1 << 0)	/* Set Q2 and Q3 current limit when in charging mode */
#define CHARGERUSB_VOREG		0x0C	/* System supply/battery regulation voltage */
#define CHARGERUSB_VICHRG		0x0D	/* Charge current in full charge mode */
#define CHARGERUSB_CINLIMIT		0x0E	/* Current limitation value on VBUS input */
#define CHARGERUSB_CTRLLIMIT1	0x0F	/* System supply/battery regulation voltage limit */
#define CHARGERUSB_CTRLLIMIT2	0x10	/* Charge current limit */
#define		LOCK_LIMIT			(1 << 4)
#define ANTICOLLAPSE_CTRL1		0x11
#define		BUCK_VTH_SHIFT		5
#define		BUCK_VTH_MASK		(7 << BUCK_VTH_SHIFT)
#define		ANTICOLL_ANA		(1 << 2)	/* Enable anticollapse loop */

/* TWL6032 registers 0xDA to 0xDE - TWL6032_MODULE_CHARGER */
#define CONTROLLER_CTRL2		0x00
#define		VSYS_PC2			(1 << 6)	/* In trickle charge VSYS level */
#define		VSELSUPPCOMP_SHIFT	4
#define		VSELSUPPCOMP_MASK	(0x3 << 4)	/* Supplement comparator threshold selection */
#define		VSYS_PC				(1 << 3)	/* VSYS voltage selction in precharge: 0: 3.6V, 1: 3.8V */
#define		LINCH_DLY			(1 << 2)	/* Linear charge control 2 clock cycles after supplement mode goes low */
#define		EN_DPPM				(1 << 1)	/* DPPM enable */
#define		SUP_MASK			(1 << 0)	/* Supplement/current comparator unmasked */
#define CONTROLLER_VSEL_COMP	0x01
#define		CONTROLLER_VSEL_COMP_DLIN_SHIFT	5
#define		CONTROLLER_VSEL_COMP_DLIN_MASK	(0x3 << 5)	/* Vbat tracking reference selection */
#define		VBATFULL_CHRG_SHIFT				2
#define		VBATFULL_CHRG_MASK				(0x9 << 2)	/* Vbat full charge comparator threshold */
#define		VBATSHORT_SHIFT					0
#define		VBATSHORT_MASK					(0x3 << 0)	/* VBAT short threshold */
#define CHARGERUSB_VSYSREG		0x02	/* System supply/battery regulation voltage */
#define CHARGERUSB_VICHRG_PC	0x03	/* Charge current in precharge mode */
#define LINEAR_CHRG_STS			0x04
#define 	LINEAR_CHRG_STS_CRYSTL_OSC_OK	0x40
#define 	LINEAR_CHRG_STS_END_OF_CHARGE	0x20
#define 	LINEAR_CHRG_STS_VBATOV			0x10
#define 	LINEAR_CHRG_STS_VSYSOV			0x08
#define 	LINEAR_CHRG_STS_DPPM_STS		0x04
#define 	LINEAR_CHRG_STS_CV_STS			0x02
#define 	LINEAR_CHRG_STS_CC_STS			0x01

/* Fuel Gauge */
#define FG_REG_00	0x00
#define		CC_ACTIVE_MODE_SHIFT	6			/* CC_ACTIVE_MODE:
												 * 00 = 250-ms update rate
												 * 01 = 62.5-ms update rate
												 * 10 = 15.6-ms update rate
												 * 11 = 3.9-ms update rate
												 */
#define		CC_ACTIVE_MODE_MASK		(3 << CC_ACTIVE_MODE_SHIFT)
#define		CC_AUTOCLEAR			(1 << 2)	/* Clears the values */
#define		CC_CAL_EN				(1 << 1)	/* Enables calibration sequence */
#define		CC_PAUSE				(1 << 0)	/* Analog updates to registers inhibited */
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

#define PWDNSTATUS1				0x93
#define PWDNSTATUS2				0x94

/* GPADC */
#define TWL6030_GPADC_CTRL			0x00    /* 0x2e */
#define		GPADC_CTRL_ISOURCE_EN		(1 << 7)	/* Current source Enabled */
#define			GPADC_ISOURCE_22uA		22
#define			GPADC_ISOURCE_7uA		7
#define			BATTERY_RESISTOR			10000
#define			SIMULATOR_RESISTOR			5000
#define			BATTERY_DETECT_THRESHOLD	((BATTERY_RESISTOR + SIMULATOR_RESISTOR) / 2)
#define		GPADC_CTRL_TEMP2_EN_MONITOR	(1 << 6)	/* Thermal shutdown #2: GPADC monitoring */
#define		GPADC_CTRL_TEMP1_EN_MONITOR	(1 << 5)	/* Thermal shutdown #1: GPADC monitoring */
#define		GPADC_CTRL_SCALER_EN_CH11	(1 << 4)    /* GPADC_IN11 */
#define		GPADC_CTRL_VSYS_SCALER_DIV4	(1 << 3)	/* GPADC_IN7 */
#define		GPADC_CTRL_SCALER_EN_CH2	(1 << 2)    /* GPADC_IN2 */
#define		GPADC_CTRL_TEMP2_EN			(1 << 1)    /* GPADC_IN4 */
#define		GPADC_CTRL_TEMP1_EN			(1 << 0)    /* GPADC_IN1 */
#define TWL6032_GPADC_CTRL2			0x01    /* 0x2F */
#define		GPADC_CTRL2_VBAT_SCALER_DIV4	(1 << 3)	/* VBAT DIV4 scaler selected */
#define		GPADC_CTRL2_SCALER_EN_CH18		(1 << 2)	/* Scaler Enable */
#define		GPADC_CTRL2_REMSENSE_0uA		(0 << 0)	/* 0 uA */
#define		GPADC_CTRL2_REMSENSE_10uA		(1 << 0)	/* 10 uA */
#define		GPADC_CTRL2_REMSENSE_400uA		(2 << 0)	/* 400 uA */
#define		GPADC_CTRL2_REMSENSE_800uA		(3 << 0)	/* 800 uA */
#define		GPADC_CTRL2_REMSENSE_MASK		(3 << 0)
#define TWL6032_GPADC_TRIM17	0xDD
#define		RATIO_TLO_MASK	0x07
#define TWL6032_GPADC_TRIM18	0xDE
#define		RATIO_THI_MASK	0x07
#define TWL6032_GPADC_TRIM19	0xFD

#define REG_TOGGLE1				0x90
#define		FGDITHS					(1 << 7)	/* FG DITH driver set signal (fuel gauge DITH digital enabled) */
#define		FGDITHR					(1 << 6)	/* FG DITH driver reset signal (fuel gauge DITH digital disabled) */
#define		FGS						(1 << 5)	/* FG driver set signal (fuel gauge digital enabled) */
#define		FGR						(1 << 4)	/* FG driver reset signal (fuel gauge digital disabled) */
#define		GPADC_SAMP_WINDOW_3US	(0 << 2)	/* 0: 3-μs sampling time window (default) */
#define		GPADC_SAMP_WINDOW_450US	(1 << 2)	/* 1: 450-μs sampling time window */
#define		GPADCS					(1 << 1)	/* GPADC set signal (GPADC enabled) */
#define		GPADCR					(1 << 0)	/* GPADC reset signal (GPADC disabled) */

#define REG_MISC1				0xE4
#define		VAC_MEAS		0x04		/* the resistor divider for the VAC voltage measurement */
#define		VBAT_MEAS		0x02		/* the resistor divider for the main battery voltage measurement */
#define		BB_MEAS			0x01		/* the resistor divider for the backup battery voltage measurement */
#define REG_USB_VBUS_CTRL_SET	0x04
#define		VBUS_MEAS		0x01		/* VBUS voltage measurement through GPADC */
#define REG_USB_ID_CTRL_SET		0x06
#define		ID_MEAS			0x01		/* ID voltage measurement through GPADC */
#define BBSPOR_CFG				0xE6
#define		BB_CHG_EN		(1 << 3)	/* The backup battery charge is enabled */
#define		BB_SEL_MASK		(3 << 1)	/* Back-up battery end-of-charge voltage selection */
#define		BB_SEL_3V0		(0 << 1)	/* 3.0V */
#define		BB_SEL_2V5		(1 << 1)	/* 2.5V */
#define		BB_SEL_3V15		(2 << 1)	/* 3.15V */
#define		BB_SEL_VSYS		(3 << 1)	/* VSYS */

/* need to access USB_PRODUCT_ID_LSB to identify which 6030 varient we are */
#define USB_PRODUCT_ID_LSB	0x02

/*----------------------------------------------------------------------*/

#ifdef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG

#define NOTIFY_USB_CHARGER_EVENT(event, value) { \
		POWER_SUPPLY_INFO("USB CHARGER event %d\n", event); \
		if (twl_charger.num_usb_charger_notifiers && \
				NOTIFY_STOP == blocking_notifier_call_chain(&twl_charger.usb_charger_notifiers, event, value)) { \
			POWER_SUPPLY_INFO("<STOP> USB CHARGER event %d\n", event); \
		} \
	}

#define NOTIFY_AC_CHARGER_EVENT(event, value) { \
		POWER_SUPPLY_INFO("AC CHARGER event %d\n", event); \
		if (twl_charger.num_ac_charger_notifiers && \
				NOTIFY_STOP == blocking_notifier_call_chain(&twl_charger.ac_charger_notifiers, event, value)) { \
			POWER_SUPPLY_INFO("<STOP> AC CHARGER event %d\n", event); \
		} \
	}

#define NOTIFY_FUELGAUGE_EVENT(event, value) { \
		POWER_SUPPLY_INFO("FG event %d\n", event); \
		if (NOTIFY_STOP == blocking_notifier_call_chain(&twl_charger.fg_notifiers, event, value)) { \
			POWER_SUPPLY_INFO("<STOP> FG event %d\n", event); \
		} \
	}

#else /* !CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */

#define NOTIFY_USB_CHARGER_EVENT(event, value) { \
		if (twl_charger.num_usb_charger_notifiers) { \
			(void)blocking_notifier_call_chain(&twl_charger.usb_charger_notifiers, event, value); \
		} \
	}
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
#define NOTIFY_AC_CHARGER_EVENT(event, value) { \
		if (twl_charger.num_ac_charger_notifiers) { \
			(void)blocking_notifier_call_chain(&twl_charger.ac_charger_notifiers, event, value); \
		} \
	}
#else
#define NOTIFY_AC_CHARGER_EVENT(event, value) do{}while(0);
#endif

#define NOTIFY_FUELGAUGE_EVENT(event, value) \
		(void)blocking_notifier_call_chain(&twl_charger.fg_notifiers, event, value);

#endif /* CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */

/*----------------------------------------------------------------------*/

struct twl6032_charger_info {
	/* USB Charger */
	struct otg_transceiver			*otg;
	struct notifier_block			usb_nb;				/* USB OTG events */
	unsigned int					usb_currentmA;		/* USB charge current limit */
	struct blocking_notifier_head	usb_charger_notifiers;	/* <features> USB charger notifiers */
	unsigned int					num_usb_charger_notifiers;
	/* TWL6032 internal USB Charger */
	struct notifier_block			usb_int_charger_nb;

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	/* AC Charger */
	struct notifier_block			ac_nb;				/* AC charger events */
	unsigned int					ac_currentmA;		/* AC charge current limit */
	struct blocking_notifier_head	ac_charger_notifiers;	/* <features> AC charger notifiers */
	unsigned int					num_ac_charger_notifiers;
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

	/* Fuel Gauge */
	struct blocking_notifier_head	fg_notifiers;	/* <features> Fuel Gauge notifiers */
	unsigned int					num_fg_notifiers;
	/* TWL6032 internal Fuel Gauge */
	struct notifier_block			int_fg_nb;
};

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static struct power_supply_charger_profile charger_profiles[] __attribute__ ((section (".data"))) =
		CONFIG_TWL6032_CHARGER_PROFILES;
static int nbr_of_charger_profiles __attribute__ ((section (".data"))) = ARRAY_SIZE(charger_profiles);
static int curr_profile_idx __attribute__ ((section (".data"))) = 0;
static int curr_active_profile_idx __attribute__ ((section (".data"))) = -1;
static unsigned int curr_active_currentmA __attribute__ ((section (".data"))) = 0;
static unsigned int curr_active_input_currentmA __attribute__ ((section (".data"))) = 0;
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
/* Workaround to decrease the charging current in order to fix TWL6032 Charger BUG! */
static unsigned int twl_charger_errs __attribute__ ((section (".data"))) = 0;
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */
static int twl_charger_vbat __attribute__ ((section (".data"))) = -1;

/*----------------------------------------------------------------------*/

static int has_usb_charging __attribute__ ((section (".data"))) = 1;
static unsigned charger_source __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;
#if !(defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY) && defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER))
static unsigned twl_charger_ac_type __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
static unsigned twl_charger_usb_type __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;

#ifndef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
static unsigned long twl_charger_timeout __attribute__ ((section (".data"))) = 0;
static unsigned long twl_charger_ticks __attribute__ ((section (".data"))) = 0;
#endif /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
static struct twl6032_charger_info twl_charger __attribute__ ((section (".data")));

/* Ptr to thermistor table */
#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
#else
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};
#endif

static unsigned long twl_capacity_timeout __attribute__ ((section (".data"))) = 0;
static unsigned long twl_capacity_ticks __attribute__ ((section (".data"))) = 0;

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static unsigned int twl_charger_start_usb_charger(int vbat, unsigned int currentmA);
static unsigned int twl6032_charger_setup_charge_current(int profile_idx, unsigned int currentmA);
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static int twl6032_get_charger_profile_index(int vbat)
{
	int index, i, j;
	struct power_supply_charger_profile *curr_profile;

	if (vbat <= 0)
		return 0;

	index = curr_profile_idx;
	curr_profile = charger_profiles + index;

	if (vbat < curr_profile->battery_voltage) {
		if (index > 0 &&
				vbat <= (curr_profile->battery_voltage - curr_profile->extra_decrease_voltage)) {
			for (i = index - 1, j = index ; j > 0; i--, j--) {
				if (vbat >= charger_profiles[i].battery_voltage &&
						vbat < charger_profiles[j].battery_voltage) {
					break;
				}
			}
			index = i;
		}
	} else if (vbat > curr_profile->battery_voltage) {
		if (index < (nbr_of_charger_profiles - 1) &&
				vbat >= (curr_profile->battery_voltage + curr_profile->extra_increase_voltage)) {
			for (i = index, j = index + 1 ; j < nbr_of_charger_profiles; i++, j++) {
				if (vbat >= charger_profiles[i].battery_voltage &&
						vbat < charger_profiles[j].battery_voltage) {
					break;
				}
			}
			index = i;
		}
	}

	if (index < 0) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", index);
		index = 0;
	} else if (index >= nbr_of_charger_profiles) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", index);
		index = nbr_of_charger_profiles - 1;
	}

	return index;
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

#ifndef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static void twl6032_charger_update_charger_profile(int vbat)
{
	int profile_idx;

	profile_idx = twl6032_get_charger_profile_index(vbat);
	if (profile_idx != curr_profile_idx) {
		POWER_SUPPLY_VDBG("%s: vbat %dmV; charger profile %d<->%d\n", __func__, vbat, curr_profile_idx, profile_idx);
		(void)twl_charger_start_usb_charger(vbat, twl_charger.usb_currentmA);
	} else {
		POWER_SUPPLY_VDBG("%s: vbat %dmV; charger profile %d\n", __func__, vbat, profile_idx);
		(void)twl6032_charger_setup_charge_current(profile_idx, twl_charger.usb_currentmA);
	}
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */
#endif

/*----------------------------------------------------------------------*/

static void twl_battery_current_setup(int enable)
{
#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
#else
	/*
	 * Writing 0 to REG_TOGGLE1 has no effect, so
	 * can directly set/reset FG.
	 */
	if (enable) {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, FGDITHS | FGS, REG_TOGGLE1);
		twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);
	} else {
#if 0
		twl_i2c_write_u8(TWL6030_MODULE_ID1, FGDITHR | FGR, REG_TOGGLE1);
#endif
	}
#endif
}

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
#else
static int twl_battery_current(void)
{
	int ret = 0;
	u16 read_value = 0;
	s16 cc_offset;
	int current_now;

	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&cc_offset,
							FG_REG_08, 2);
	if (ret < 0) {
		POWER_SUPPLY_DBG("failed to read FG_REG_8 %d\n", ret);
		return 0;
	}
	cc_offset = ((s16)(cc_offset << 6) >> 6);

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
								FG_REG_10, 2);
	if (ret < 0) {
		POWER_SUPPLY_DBG("failed to read FG_REG_10 %d\n", ret);
		return 0;
	}
	current_now = ((s16)(read_value << 2) >> 2);
	current_now = current_now - cc_offset;

#if 0
	/* current drawn per sec */
	current_now = current_now * fuelgauge_rate[0];
	/* current in mAmperes */
	current_now = current_now * DIV_ROUND_UP(62000, CONFIG_TWL6032_CHARGER_SENSE_RESISTOR);
	current_now >>= 15;
#else
	/* current in uAmperes */
	current_now = current_now *
			((DIV_ROUND_UP(62000000, CONFIG_TWL6032_CHARGER_SENSE_RESISTOR) * fuelgauge_rate[0]) >> 15);
	current_now /= 1000;
#endif

	return current_now;
}

static int twl6032_get_battery_current(void)
{
	static int first_time = 1;

	if (first_time) {
		ulong start;
		int ret;
		u8 value;

		first_time = 0;

		start = get_timer(0);
		do {
			value = 0;
			ret = twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, &value, FG_REG_00);
			if (ret < 0) {
				POWER_SUPPLY_DBG("failed to read FG_REG_0 %d\n", ret);
				return 0;
			}
			if ((value & (CC_AUTOCLEAR | CC_CAL_EN)) == 0)
				break;
			mdelay(10);
		} while ((get_timer(0) - start) <= 300);
	}

	return twl_battery_current();
}
#endif

static int twl6032_get_battery_capacity(void)
{
	static int capacity = -1;

	/**
	 * Bootloader doesn't support to calculate the battery capacity
	 * so we will drop the stored capacity if it's expired (timeout).
	 */
	if (timeout_check(&twl_capacity_timeout, &twl_capacity_ticks))
		return -1; /* -1: invalid capacity */
	else if (capacity == -1) {
		u8 bat_capacity = -1;

		twl_i2c_read_u8(TWL6030_MODULE_ID0, &bat_capacity, VALIDITY0); /* iCOM: Battery capacity */
		if (bat_capacity > 100) {
			POWER_SUPPLY_ERR("BAD capacity %d %%\n", bat_capacity);
			bat_capacity = -1;
		}

		capacity = bat_capacity;
	}

	return capacity;
}

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static int twl6032_is_charging(u8 *state)
{
	/* Check the USB Charging status */
	*state = 0;
	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, state, LINEAR_CHRG_STS);
	if (*state & (LINEAR_CHRG_STS_CC_STS | LINEAR_CHRG_STS_CV_STS | LINEAR_CHRG_STS_DPPM_STS))
		return POWER_SUPPLY_CHARGER_STATE_CHARGING;
	else if (*state & LINEAR_CHRG_STS_END_OF_CHARGE)
		return POWER_SUPPLY_CHARGER_STATE_EOC;
	else
		return POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING;
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

static int twl_get_gpadc_conversion(u8 channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channel = channel_no;

	ret = twl6030_gpadc_conversion(&req);
	if (ret != 1) {
		POWER_SUPPLY_ERR("gpadc conversion err %d\n", ret);
		goto out;
	}

	if (req.rbuf > 0)
		temp = req.rbuf;

out:
	return temp;
}

static int twl6032_get_sys_voltage(void)
{
	static unsigned int skip_times = 1;
	int count = 3;
	int vsys;

_get_vsys:
	vsys = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VSYS);
	while (vsys < 3000 && count > 0) {
		mdelay(50);
		vsys = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VSYS);
		count--;
	}

	if (skip_times) {
		--skip_times;
		mdelay(50);
		goto _get_vsys;
	}

	return vsys;
}

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) \
		&& !defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT) \
		&& !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
static int twl6032_emergency_get_vbat(void)
{
	int vbat = 0;

	vbat = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VBAT);

	POWER_SUPPLY_VDBG("%s: %d\n", __func__, vbat);

	return vbat;
}
#endif

int twl6032_get_battery_voltage(void)
{
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) \
		&& !defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT) \
		&& !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
	return twl6032_emergency_get_vbat();
#else
	return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
#endif
}

static int twl6032_get_vbat(void)
{
	static unsigned int skip_times = 1;
	int vbat = 0;
	int count = 3;
	int debounce = 5;

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && \
		defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT)

	if (twl_charger_usb_type != POWER_SUPPLY_TYPE_UNKNOWN
			|| twl_charger_ac_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		POWER_SUPPLY_VDBG("%s: %d\n", __func__, CONFIG_POWER_SUPPLY_INVALID_VOLTAGE);
		return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
	}

#endif

_get_vbat:
	vbat = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VBAT);

	if (vbat < 10 && count > 0) {
		count--;
		mdelay(50);
		goto _get_vbat;
	}

	if (skip_times > 0) {
		--skip_times;
		mdelay(128);
		goto _get_vbat;
	}

	if (vbat >= CONFIG_POWER_SUPPLY_POWERON_VOLTAGE
			&& twl_charger_vbat <= CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV && twl_charger_vbat >= 0
			&& debounce > 0) {
		debounce--;
		POWER_SUPPLY_ERR("[%d] bad VBAT %d mV (prev %d mV)...retry...\n", debounce, vbat, twl_charger_vbat);
		mdelay(128);
		goto _get_vbat;
	}

	twl_charger_vbat = vbat;

	POWER_SUPPLY_VDBG("%s: %d\n", __func__, vbat);

	return vbat;
}

int twl_charger_get_battery_voltage(void)
{
	int vbat;

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
	if (power_supply_has_external_fuelgauge()) {
		vbat = 0;
		NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VBAT, &vbat);
	} else
#endif
#if defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)
		vbat = twl6032_get_sys_voltage();
#else
		vbat = twl6032_get_vbat();
#endif

	return vbat;
}

static inline int twl6032_get_battery_temp(void)
{
	return twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_BAT_TEMP);
}

static int twl6032_get_usb_voltage(void)
{
	int value = 0;
	u8 state = 0;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state, CONTROLLER_STAT1);
	if (state & VBUS_DET)
		value = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VBUS);

	return value;
}

static int twl6032_get_ac_voltage(void)
{
	int value = 0;
	u8 state = 0;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state, CONTROLLER_STAT1);
	if (state & VAC_DET)
		value = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_VAC);

	return value;
}

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static void twl6032_charger_dump_regs(void)
{
	u8 value;

#define SHOW_REG(m,r) \
	value = 0; \
	twl_i2c_read_u8(m, &value, r); \
	printf("%-35s 0x%02X\n", #r, value);

	puts("==============================================\n");
#if 0
	SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS1);
	SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS2);
	SHOW_REG(TWL6030_MODULE_ID0, REG_MISC1);
	SHOW_REG(TWL_MODULE_MADC, TWL6030_GPADC_CTRL);
	SHOW_REG(TWL_MODULE_MADC, TWL6032_GPADC_CTRL2);
	SHOW_REG(TWL_MODULE_USB, REG_USB_VBUS_CTRL_SET);
	SHOW_REG(TWL_MODULE_USB, REG_USB_ID_CTRL_SET);
	puts("----------------------------------------------\n");
#endif
	SHOW_REG(TWL6030_MODULE_CHARGER, CONTROLLER_INT_MASK);
	SHOW_REG(TWL6030_MODULE_CHARGER, CONTROLLER_CTRL1);
	SHOW_REG(TWL6030_MODULE_CHARGER, CONTROLLER_WDG);

#if 0
	SHOW_REG(TWL6030_MODULE_CHARGER, CONTROLLER_STAT1);
#else
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CONTROLLER_STAT1);
	printf("%-35s 0x%02X\n", "CONTROLLER_STAT1", value);
	if (value) {
		if (value & CHRG_EXTCHRG_STATZ)
			puts(" EXTCHRG_STATZ");
		if (value & LINCH_GATED_STAT)
			puts(" LINCH_GATED");
		if (value & CHRG_DET_N)
			puts(" CHRG_DET_N");
		if (value & FAULT_WDG)
			puts(" FAULT_WDG");
		if (value & VAC_DET)
			puts(" VAC");
		if (value & VBUS_DET)
			puts(" VBUS");
		if (value & BAT_REMOVED)
			puts(" BAT_REMOVED");
		if (value & BAT_TEMP_OVRANGE)
			puts(" TEMP_OVRANGE");
		putc('\n');
	}
#endif

#if 0
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_INT_STATUS);
#else
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CHARGERUSB_INT_STATUS);
	printf("%-35s 0x%02X\n", "CHARGERUSB_INT_STATUS", value);
	if (value) {
		if (value & EN_LINCH)
			puts(" EN_LINCH");
		if (value & CURRENT_TERM)
			puts(" TERM");
		if (value & CHARGERUSB_STAT)
			puts(" STAT");
		if (value & CHARGERUSB_THMREG)
			puts(" THMREG");
		if (value & CHARGERUSB_FAULT)
			puts(" FAULT");
		putc('\n');
	}
#endif

	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_INT_MASK);

#if 0
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_STATUS_INT1);
#else
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CHARGERUSB_STATUS_INT1);
	printf("%-35s 0x%02X\n", "CHARGERUSB_STATUS_INT1", value);
	if (value) {
		if (value & TMREG)
			puts(" TMREG");
		if (value & NO_BAT)
			puts(" NO_BAT");
		if (value & BST_OCP)
			puts(" BST_OCP");
		if (value & TH_SHUTD)
			puts(" TH_SHUTD");
		if (value & BAT_OVP)
			puts(" BAT_OVP");
		if (value & POOR_SRC)
			puts(" POOR_SRC");
		if (value & SLP_MODE)
			puts(" SLP_MODE");
		if (value & VBUS_OVP)
			puts(" VBUS_OVP");
		putc('\n');
	}
#endif

#if 0
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_STATUS_INT2);
#else
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CHARGERUSB_STATUS_INT2);
	printf("%-35s 0x%02X\n", "CHARGERUSB_STATUS_INT2", value);
	if (value) {
		if (value & ICCLOOP)
			puts(" ICCLOOP");
		if (value & CURRENT_TERM_STAT)
			puts(" TERM_STAT");
		if (value & CHARGE_DONE)
			puts(" CHARGE_DONE");
		if (value & ANTICOLLAPSE)
			puts(" ANTICOLLAPSE");
		putc('\n');
	}
#endif

	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CTRL1);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CTRL2);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CTRL3);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_VOREG);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_VICHRG);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CINLIMIT);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CTRLLIMIT1);
	SHOW_REG(TWL6030_MODULE_CHARGER, CHARGERUSB_CTRLLIMIT2);
	SHOW_REG(TWL6030_MODULE_CHARGER, ANTICOLLAPSE_CTRL1);
	puts("----------------------------------------------\n");
	SHOW_REG(TWL6032_MODULE_CHARGER, CONTROLLER_CTRL2);
	SHOW_REG(TWL6032_MODULE_CHARGER, CONTROLLER_VSEL_COMP);
	SHOW_REG(TWL6032_MODULE_CHARGER, CHARGERUSB_VSYSREG);
	SHOW_REG(TWL6032_MODULE_CHARGER, CHARGERUSB_VICHRG_PC);

#if 0
	SHOW_REG(TWL6032_MODULE_CHARGER, LINEAR_CHRG_STS);
#else
	value = 0;
	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value, LINEAR_CHRG_STS);
	printf("%-35s 0x%02X (%s)\n", "LINEAR_CHRG_STS", value,
		(value & (LINEAR_CHRG_STS_CC_STS | LINEAR_CHRG_STS_CV_STS | LINEAR_CHRG_STS_DPPM_STS)) ? "Charging" :
		((value & LINEAR_CHRG_STS_END_OF_CHARGE) ? "END of Charging" : "NOT Charging??"));
	if (value) {
		if (value & LINEAR_CHRG_STS_CRYSTL_OSC_OK)
			puts(" CRYSTL_OSC_OK");
		if (value & LINEAR_CHRG_STS_END_OF_CHARGE)
			puts(" END_OF_CHARGE");
		if (value & LINEAR_CHRG_STS_VBATOV)
			puts(" VBATOV");
		if (value & LINEAR_CHRG_STS_VSYSOV)
			puts(" VSYSOV");
		if (value & LINEAR_CHRG_STS_DPPM_STS)
			puts(" DPPM");
		if (value & LINEAR_CHRG_STS_CV_STS)
			puts(" CV");
		if (value & LINEAR_CHRG_STS_CC_STS)
			puts(" CC");
		putc('\n');
	}
#endif

#if 0
	puts("----------------------------------------------\n");
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_00);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_01);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_02);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_03);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_04);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_05);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_06);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_07);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_08);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_09);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_10);
	SHOW_REG(TWL6030_MODULE_GASGAUGE, FG_REG_11);
#endif

	if (twl_charger.otg) {
		int temp = twl6032_get_battery_temp();

		puts("----------------------------------------------\n");
		POWER_SUPPLY_PRINT("Battery TEMP: %d mV%s\n", temp,
			(temp <= CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV || temp >= CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV) ?
			" (ERROR?)" : "");
	}
	puts("==============================================\n");
{
	int currentmA = power_supply_get_battery_current();
	if (currentmA != CONFIG_POWER_SUPPLY_INVALID_CURRENT) {
		POWER_SUPPLY_PRINT("Current: %d mA\n", currentmA);
		puts("==============================================\n");
	}
}

#undef SHOW_REG

	twl6030_clear_interrupts();
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

static void twl_charger_kick_watchdog(void)
{
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_WDG_RST | CONTROLLER_WDG_WDT(127/*secs*/), CONTROLLER_WDG);
}

/*----------------------------------------------------------------------*/

#if 0
static int twl_charger_isource_val __attribute__ ((section (".data"))) = 0;

static void twl_charger_battery_detect_debug(void)
{
	int val;

	/*
	 * twl_get_gpadc_conversion for
	 * 6030 return resistance, for 6032 - voltage and
	 * it should be converted to resistance before
	 * using.
	 */
	if (!twl_charger_isource_val) {
		u8 reg = 0;

		if (twl_i2c_read_u8(TWL_MODULE_MADC, &reg, TWL6030_GPADC_CTRL))
			POWER_SUPPLY_DBG("%s: Error reading TWL6030_GPADC_CTRL\n", __func__);

		twl_charger_isource_val = (reg & GPADC_CTRL_ISOURCE_EN) ?
					GPADC_ISOURCE_22uA : GPADC_ISOURCE_7uA;
		printf("GPADC_IN0: %d uA\n", twl_charger_isource_val);
	}

	/*
	 * Prevent charging on batteries if id resistor is less than 5K.
	 */
	val = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_BATTERY_TYPE);
	printf("GPADC_IN0: %d mV\n", val);

	val = (val * 1000) / twl_charger_isource_val;
	printf("GPADC_IN0: %d ohm\n", val);
}
#endif

/*----------------------------------------------------------------------*/

/* Vbat tracking reference selection & Vbat full charge comparator threshold */
static void twl_set_vbat_comp(int tracking_vol, int full_charge_vol)
{
	u8 value = 0;

	tracking_vol = (tracking_vol - 50) / 50;
	if (tracking_vol > 3)
		tracking_vol = 3;
	else if (tracking_vol < 0)
		tracking_vol = 0;

	CHARGER_VPRINT("tracking voltage: %u mV\n", tracking_vol * 50 + 50);

	full_charge_vol = (full_charge_vol - 2650) / 100;
	if (full_charge_vol > 9)
		full_charge_vol = 9;
	else if (full_charge_vol < 0)
		full_charge_vol = 0;

	CHARGER_VPRINT("full charge voltage: %u mV\n", full_charge_vol * 100 + 2650);

	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_VSEL_COMP);
	value &= ~(CONTROLLER_VSEL_COMP_DLIN_MASK | VBATFULL_CHRG_MASK);
	value = value | (tracking_vol << CONTROLLER_VSEL_COMP_DLIN_SHIFT) |
			(full_charge_vol << VBATFULL_CHRG_SHIFT);
	twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value, CONTROLLER_VSEL_COMP);
}

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
/* Enable DPPM */
static void twl_enable_dppm(int enable)
{
	u8 value = 0;

	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_CTRL2);
	if (enable)
		value |= EN_DPPM;
	else
		value &= ~EN_DPPM;
	twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value, CONTROLLER_CTRL2);
}

/* Setup the termination current level */
static void twl_config_iterm_reg(unsigned int term_currentmA)
{
	if (term_currentmA > 400)
		term_currentmA = 400;
	else if (term_currentmA < 50)
		term_currentmA = 50;

	term_currentmA = ((term_currentmA - 50) / 50) << 5;

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
		CHARGERUSB_CTRL2);

#ifdef CONFIG_POWER_SUPPLY_ENABLE_CHARGE_TERMINATION
	/* Enable charge current termination */
	twl_clear_n_set(TWL6030_MODULE_CHARGER, 0, TERM, CHARGERUSB_CTRL1);
	CHARGER_VPRINT("enable termination current: %u mA\n", (term_currentmA >> 5) * 50 + 50);
#else
	/* Disable charge current termination */
	twl_clear_n_set(TWL6030_MODULE_CHARGER, TERM, 0, CHARGERUSB_CTRL1);
	CHARGER_VPRINT("disable termination current: %u mA\n", (term_currentmA >> 5) * 50 + 50);
#endif
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

/* System supply/battery regulation voltage */
static void twl_config_voreg_reg(unsigned int voltagemV)
{
	/* TWL6032 ES1.1 POP = 1 */

	if (voltagemV < 3500)
		voltagemV = 3500;
	else if  (voltagemV > 4760)
		voltagemV = 4760;

	voltagemV = (voltagemV - 3500) / 20;

	CHARGER_VPRINT("battery regulation voltage: %u mV\n", voltagemV * 20 + 3500);

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
		CHARGERUSB_VOREG);
}

/* Setup the charge current in precharge mode */
static unsigned int twl_config_vichrg_pc(unsigned int currentmA)
{
	/* TWL6032 ES1.1 POP = 1 */

	if (currentmA < 100)
		currentmA = 100;
	else if (currentmA > 400)
		currentmA = 400;

	currentmA = (currentmA - 100) / 100;

	twl_i2c_write_u8(TWL6032_MODULE_CHARGER, currentmA,
		CHARGERUSB_VICHRG_PC);

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 || CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 20)
	currentmA = (currentmA * 100 + 100);
#else
	currentmA = (currentmA * 100 + 100) * 20 / CONFIG_TWL6032_CHARGER_SENSE_RESISTOR;
#endif
	CHARGER_DPRINT("precharge current: %u mA\n", currentmA);

	return currentmA;
}

/* Setup the charge current in full charge mode */
static unsigned int twl_config_vichrg_reg(unsigned int currentmA)
{
	/* TWL6032 ES1.1 POP = 1 */

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 || CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 20)
#else
	currentmA = currentmA * CONFIG_TWL6032_CHARGER_SENSE_RESISTOR / 20;
#endif

	if (currentmA < 100)
		currentmA = 100;
	else if (currentmA > 1500)
		currentmA = 1500;

#if 0
	currentmA = (currentmA - 100) / 100;
#else
	/* Use DIV_ROUND() for the better full charge current */
	currentmA = (currentmA - 100 + 50) / 100;
#endif

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
		CHARGERUSB_VICHRG);

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 || CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 20)
	currentmA = currentmA * 100 + 100;
#else
	currentmA = (currentmA * 100 + 100) * 20 / CONFIG_TWL6032_CHARGER_SENSE_RESISTOR;
#endif
	CHARGER_DPRINT("charge current: %u mA\n", currentmA);

	return currentmA;
}

/* Current limitation value on VBUS input */
static unsigned int twl_config_cinlimit_reg(unsigned int currentmA)
{
	/* TWL6032 ES1.1 POP = 1 */

	unsigned int input_currentmA = 0;

	if (currentmA == 0 || currentmA > 2250) {
		currentmA = 0x3F; /* XX1111: No input current limit */
		CHARGER_DPRINT("input current: no limit\n");
		input_currentmA = 2500;
	} else {
		if (currentmA < 50)
			currentmA = 50;

		if (currentmA < 800) {
			currentmA = (currentmA - 50) / 50;
			input_currentmA = currentmA * 50 + 50;
		} else if (currentmA < 1500) {
			currentmA = ((currentmA % 100) ? 0x30 : 0x20) + (currentmA - 100) / 100;
			input_currentmA = ((currentmA & 0x30) == 0x30) ? 50 : 0;
			input_currentmA += (currentmA & ~0x30) * 100 + 100;
		} else if (currentmA < 1800) {
			currentmA = 0x2E;
			input_currentmA = 1500;
		} else if (currentmA < 2100) {
			currentmA = 0x20;
			input_currentmA = 1800;
		} else if (currentmA < 2250) {
			currentmA = 0x21;
			input_currentmA = 2100;
		} else {
			currentmA = 0x22;
			input_currentmA = 2250;
		}
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
		CHARGERUSB_CINLIMIT);

	CHARGER_DPRINT("input current: %u mA\n", input_currentmA);

	return input_currentmA;
}

/* System supply/battery regulation voltage limit */
static void twl_config_limit1_reg(unsigned int voltagemV)
{
	/* TWL6032 ES1.1 POP = 1 */

	if (voltagemV < 3500)
		voltagemV = 3500;
	else if (voltagemV > 4760)
		voltagemV = 4760;

	voltagemV = (voltagemV - 3500) / 20;

	CHARGER_VPRINT("battery regulation voltage limit: %u mV\n", voltagemV * 20 + 3500);

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, (u8)voltagemV,
		CHARGERUSB_CTRLLIMIT1);
}

/* Charge current limit */
static void twl_config_limit2_reg(unsigned int currentmA)
{
	/* TWL6032 ES1.1 POP = 1 */

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 || CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 20)
#else
	currentmA = currentmA * CONFIG_TWL6032_CHARGER_SENSE_RESISTOR / 20;
#endif

	if (currentmA < 100)
		currentmA = 100;
	else if (currentmA > 1500)
		currentmA = 1500;

	currentmA = (currentmA - 100) / 100;

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 || CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 20)
	CHARGER_VPRINT("charge current limit: %u mA\n", (currentmA * 100 + 100));
#else
	CHARGER_VPRINT("charge current limit: %u mA\n", (currentmA * 100 + 100) * 20 / CONFIG_TWL6032_CHARGER_SENSE_RESISTOR);
#endif

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
		CHARGERUSB_CTRLLIMIT2);
}

/*
 * Disable backup battery charging.
 */
static void twl_charger_backup_battery_disable(void)
{
	u8 rd_reg = 0;

	twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
	/* Disable backup battery charging */
	rd_reg &= ~BB_CHG_EN;
	twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);
}

/*
 * Setup the BCI module to enable backup battery charging.
 */
void twl_charger_backup_battery_setup(void)
{
	u8 rd_reg = 0;

	twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
#if (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 0)
	/* No backup battery */
	rd_reg &= ~BB_CHG_EN;
#elif (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 2500) /* 2.5V */
	rd_reg &= ~BB_SEL_MASK;
	rd_reg |= BB_CHG_EN | BB_SEL_2V5;
#elif (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 3000) /* 3.0V */
	rd_reg &= ~BB_SEL_MASK;
	rd_reg |= BB_CHG_EN | BB_SEL_3V0;
#elif (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 3150) /* 3.15V */
	rd_reg &= ~BB_SEL_MASK;
	rd_reg |= BB_CHG_EN | BB_SEL_3V15;
#elif (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 4200) /* VSYS */
	rd_reg &= ~BB_SEL_MASK;
	rd_reg |= BB_CHG_EN | BB_SEL_VSYS;
#else
#error "Invalid CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV (0, 2500, 3000, 3150, 4200)!"
#endif /* CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);
}

/*
 * Setup the twl6030 BCI module to measure battery
 * temperature
 */
static void twl_battery_temp_setup(void)
{
#if 0
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);
	if (ret)
		return ret;

	if (enable)
		rd_reg |= (GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP2_EN |
			GPADC_CTRL_TEMP1_EN_MONITOR |
			GPADC_CTRL_TEMP2_EN_MONITOR | GPADC_CTRL_SCALER_DIV4);
	else
		rd_reg ^= (GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP2_EN |
			GPADC_CTRL_TEMP1_EN_MONITOR |
			GPADC_CTRL_TEMP2_EN_MONITOR | GPADC_CTRL_SCALER_DIV4);

	ret = twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

	return ret;
#else
	twl_clear_n_set(TWL_MODULE_MADC,
			/* clear */ GPADC_CTRL_TEMP2_EN | GPADC_CTRL_TEMP2_EN_MONITOR,
			/* set   */ GPADC_CTRL_TEMP1_EN | GPADC_CTRL_TEMP1_EN_MONITOR,
			TWL6030_GPADC_CTRL);
#endif
}

static void twl_battery_voltage_setup(void)
{
#if 0
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);
	if (ret)
		return ret;

	rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
	ret = twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);
	if (ret)
		return ret;

	ret = twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
	if (ret)
		return ret;

	rd_reg = rd_reg | VBUS_MEAS;
	ret = twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);
	if (ret)
		return ret;

	ret = twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
	if (ret)
		return ret;

	rd_reg = rd_reg | ID_MEAS;
	ret = twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);
	if (ret)
		return ret;

	if (di->features & TWL6032_SUBCLASS)
		ret = twl_i2c_write_u8(TWL_MODULE_MADC,
					GPADC_CTRL2_CH18_SCALER_EN,
					TWL6030_GPADC_CTRL2);

	return ret;
#else
#if (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 0)
	/* No backup battery */
	twl_clear_n_set(TWL6030_MODULE_ID0, BB_MEAS, VAC_MEAS | VBAT_MEAS, REG_MISC1);
#else
	twl_clear_n_set(TWL6030_MODULE_ID0, 0, VAC_MEAS | VBAT_MEAS | BB_MEAS, REG_MISC1);
#endif /* CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV */

	twl_i2c_write_u8(TWL_MODULE_USB, VBUS_MEAS, REG_USB_VBUS_CTRL_SET);

#if 0
	twl_i2c_write_u8(TWL_MODULE_USB, ID_MEAS, REG_USB_ID_CTRL_SET);
#endif

	/*
	 * Clear GPADC_CTRL_VSYS_SCALER_DIV4
	 */
	twl_clear_n_set(TWL_MODULE_MADC,
			/* clear */ GPADC_CTRL_VSYS_SCALER_DIV4,
			/* set   */ 0,
			TWL6030_GPADC_CTRL);

	/*
	 * Clear GPADC_CTRL2_VBAT_SCALER_DIV4, GPADC_CTRL2_REMSENSE_MASK
	 * Set   GPADC_CTRL2_SCALER_EN_CH18, GPADC_CTRL2_REMSENSE_0uA
	 */
	twl_clear_n_set(TWL_MODULE_MADC,
			/* clear */ GPADC_CTRL2_VBAT_SCALER_DIV4 | GPADC_CTRL2_REMSENSE_MASK,
			/* set   */ GPADC_CTRL2_SCALER_EN_CH18 | GPADC_CTRL2_REMSENSE_0uA,
			TWL6032_GPADC_CTRL2);
#endif
}

/*----------------------------------------------------------------------*/

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
static unsigned int twl6032_charger_setup_charge_current(int profile_idx, unsigned int currentmA)
{
	struct power_supply_charger_profile *curr_profile;
	unsigned int decrease_currentmA;
	unsigned int input_currentmA;
	unsigned int pc_currentmA;

	POWER_SUPPLY_VDBG("%s(%d, %u)\n", __func__, profile_idx, currentmA);

	if (profile_idx < 0) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", profile_idx);
		profile_idx = 0;
	} else if (profile_idx >= nbr_of_charger_profiles) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", profile_idx);
		profile_idx = nbr_of_charger_profiles - 1;
	}

	if (currentmA == 0) {
		POWER_SUPPLY_ERR("setup charger current (0 mA) (source %u, ac %u, usb %u)\n",
			charger_source, twl_charger_ac_type, twl_charger_usb_type);
	}

	if (curr_active_currentmA && curr_active_profile_idx == profile_idx) {
		return curr_active_currentmA;
	}

	curr_active_profile_idx = profile_idx;
	curr_profile = charger_profiles + profile_idx;
	POWER_SUPPLY_VDBG("PROFILE %d: [%d..%dmV..%d] INPUT %umA, CHARGE %umA\n",
			profile_idx,
			curr_profile->battery_voltage - curr_profile->extra_decrease_voltage,
			curr_profile->battery_voltage,
			curr_profile->battery_voltage + curr_profile->extra_increase_voltage,
			curr_profile->input_current, curr_profile->charge_current);

	if (curr_profile->charge_current < currentmA)
		currentmA = curr_profile->charge_current;

	decrease_currentmA = twl_charger_errs * 100/*mA*/;
	if (decrease_currentmA) {
		if (currentmA > decrease_currentmA) {
			currentmA -= decrease_currentmA;
			if (currentmA < 200)
				currentmA = 200;
		} else if (currentmA > 200)
			currentmA = 200;
	}

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
	/* Setup the input current */
	input_currentmA = curr_profile->input_current;
	if (charger_source == POWER_SUPPLY_TYPE_USB_DCP) {
		if (input_currentmA > CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA)
			input_currentmA = CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA;
	} else {
		if (input_currentmA > CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA)
			input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA;
	}
	if (decrease_currentmA) {
		if (input_currentmA > decrease_currentmA) {
			input_currentmA -= decrease_currentmA;
			if (input_currentmA <= (CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA - 50))
				input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA - 50;
			else if (input_currentmA < CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA)
				input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA;
		} else if (input_currentmA > (CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA - 50))
			input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA - 50;
	}
#else /* CONFIG_TWL6032_CHARGER_SENSE_RESISTOR != 0 */
	/* Setup the input current */
	if (charger_source == POWER_SUPPLY_TYPE_USB_DCP)
		input_currentmA = CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA;
	else
		input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA;
	if (curr_profile->input_current < input_currentmA)
		input_currentmA = curr_profile->input_current;
#endif /* CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 */
	input_currentmA = twl_config_cinlimit_reg(input_currentmA);

	/* Setup the charge current */
	pc_currentmA = twl_config_vichrg_pc(currentmA);
	currentmA = twl_config_vichrg_reg(currentmA);

#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
	curr_active_currentmA = input_currentmA;
	curr_active_input_currentmA = input_currentmA;

#ifdef CONFIG_TWL6032_CHARGER_DEBUG
	POWER_SUPPLY_DBG("setup current: %u/%u mA\n", curr_active_currentmA, curr_active_input_currentmA);
#endif

	return input_currentmA;
#else /* CONFIG_TWL6032_CHARGER_SENSE_RESISTOR != 0 */
	POWER_SUPPLY_VDBG("pre-charge current: %u mA\n", pc_currentmA);

	if (input_currentmA < currentmA)
		currentmA = input_currentmA;

	curr_active_currentmA = currentmA;
	curr_active_input_currentmA = input_currentmA;

#ifdef CONFIG_TWL6032_CHARGER_DEBUG
	POWER_SUPPLY_DBG("setup current: %u/%u mA\n", curr_active_currentmA, curr_active_input_currentmA);
#endif

	return currentmA;
#endif /* CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0 */
}

/*----------------------------------------------------------------------*/

static void twl_charger_stop_usb_charger(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	POWER_SUPPLY_VDBG("<NO Main Battery> %s\n", __func__);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	int current = power_supply_get_battery_current();

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (power_supply_has_external_fuelgauge()) {
		int capacity = power_supply_get_battery_capacity();
		if (capacity >= 0) {
			if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
				POWER_SUPPLY_INFO("stop USB charger @ %d%% (%dmA)\n", capacity, current);
			else
				POWER_SUPPLY_INFO("stop USB charger @ %d%%\n", capacity);
		} else {
			if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
				POWER_SUPPLY_INFO("stop USB charger @ (%dmA)\n", current);
			else
				POWER_SUPPLY_INFO("stop USB charger\n");
		}
	} else
#endif
	{
		if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
			POWER_SUPPLY_INFO("stop USB charger @ (%dmA)\n", current);
		else
			POWER_SUPPLY_INFO("stop USB charger\n");
	}

	twl_charger_kick_watchdog();

	/* Disable TWL6032 USB linear charger */
	twl_clear_n_set(TWL6030_MODULE_CHARGER,
			/* clear */ CONTROLLER_CTRL1_EN_LINCH,
			/* set   */ CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);
	mdelay(50);

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

static unsigned int twl_charger_start_usb_charger(int vbat, unsigned int currentmA)
{
	twl_charger_kick_watchdog();

	/* Enable TWL6032 USB power path, but Disable TWL6032 USB linear charger */
	twl_clear_n_set(TWL6030_MODULE_CHARGER,
			/* clear */ CONTROLLER_CTRL1_EN_LINCH,
			/* set   */ CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);
	mdelay(50);

	twl_config_limit1_reg(CONFIG_POWER_SUPPLY_MAX_CHARGER_VOLTAGEMV);
	twl_config_limit2_reg(CONFIG_POWER_SUPPLY_MAX_CHARGER_CURRENTMA);
	twl_config_voreg_reg(CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			CONTROLLER_INT_MASK); /* Mask interupt generation on BAT_TEMP_OVRANGE event */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MCHARGERUSB_THMREG,
			CHARGERUSB_INT_MASK); /* Mask CHARGERUSB_THMREG interrupt */

	/* Charge once feature is disabled */
	twl_clear_n_set(TWL6030_MODULE_CHARGER, CHARGE_ONCE, 0, CHARGERUSB_CTRL3);

	/* Setup the charge current */
	if (vbat < 0 && vbat != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE)
		vbat = twl_charger_get_battery_voltage();
	curr_active_profile_idx = -1;
	curr_active_currentmA = 0;
	curr_profile_idx = twl6032_get_charger_profile_index(vbat);
	currentmA = twl6032_charger_setup_charge_current(curr_profile_idx, currentmA);

	/* MIN full charge volt: 2650mV~3350mV(100mV step) (no effect on TWL6032 power path) */
	twl_set_vbat_comp(200/*mV*/, 3250/*mV*/);
	twl_enable_dppm(1); /* Enable DPPM */

	/* Setup the termination current */
	twl_config_iterm_reg(CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA);

	/* Enable TWL6032 USB linear charger */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_CTRL1_EN_CHARGER | CONTROLLER_CTRL1_EN_LINCH,
			CONTROLLER_CTRL1);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (power_supply_has_external_fuelgauge()) {
		int capacity = power_supply_get_battery_capacity();
		if (capacity >= 0)
			POWER_SUPPLY_INFO("start USB charger (%u/%umA) @ %dmV; %d%%\n",
					currentmA, curr_active_input_currentmA, vbat, capacity);
		else
			POWER_SUPPLY_INFO("start USB charger (%u/%umA) @ %dmV\n",
					currentmA, curr_active_input_currentmA, vbat);
	} else
#endif
		POWER_SUPPLY_INFO("start USB charger (%u/%umA) @ %dmV\n",
				currentmA, curr_active_input_currentmA, vbat);

	mdelay(128);

	twl_charger_backup_battery_setup();

	twl_charger_kick_watchdog();

	return currentmA;
}

static int twl_charger_is_charging(void)
{
	int charging = 0;
	u8 state = 0;

	/* Check the USB charger status */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state, CONTROLLER_STAT1);
	if (state & VBUS_DET) {
		charging = twl6032_is_charging(&state);
		if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
			return charging;
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_EOC) {
			int vbat = twl_charger_get_battery_voltage();
			if (vbat != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE && vbat < CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV) {
				POWER_SUPPLY_INFO("VBAT %d mV, [EOC] restart charger (%u mA)\n\n", vbat, twl_charger.usb_currentmA);
				(void)power_supply_start_charger(vbat);
				mdelay(128);
				charging = twl6032_is_charging(&state);
			} else {
				POWER_SUPPLY_INFO("<EOC> VBAT %d mV\n\n", vbat);
				twl_charger_kick_watchdog();
			}
		} else {
			int temp;

			POWER_SUPPLY_PRINT("\n\n<NOT Charging> >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
			twl6032_charger_dump_regs();
			POWER_SUPPLY_PRINT("<NOT Charging> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");

			temp = twl6032_get_battery_temp();
			POWER_SUPPLY_PRINT("TEMP LOW: %dmV, HIGH: %dmV, CURR: %dmV\n",
					CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV, CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV, temp);
			if (temp >= CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV && temp <= CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV) {
				(void)power_supply_start_charger(-1);
				mdelay(128);
				charging = twl6032_is_charging(&state);
			}
		}
	}

	return charging;
}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

/*----------------------------------------------------------------------*/

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
static void twl_charger_stop_ac_charger(void)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

	/* Disable TWL6032 AC charger functionality */
#ifdef CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER
	twl_clear_n_set(TWL6030_MODULE_CHARGER,
			/* clear */ CONTROLLER_CTRL1_EN_CHARGER | CONTROLLER_CTRL1_SEL_CHARGER,
			/* set   */ 0,
			CONTROLLER_CTRL1);
#else
	twl_clear_n_set(TWL6030_MODULE_CHARGER,
			/* clear */ CONTROLLER_CTRL1_SEL_CHARGER,
			/* set   */ CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);
#endif
}

static void twl_charger_start_ac_charger(void)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

	/* Select VAC input supply path to enable external AC charger */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_CTRL1_EN_CHARGER | CONTROLLER_CTRL1_SEL_CHARGER,
			CONTROLLER_CTRL1);
}
#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

/*----------------------------------------------------------------------*/

static void twl_charger_init(void)
{
	twl_battery_temp_setup();
	twl_battery_voltage_setup();

	twl_config_limit1_reg(CONFIG_POWER_SUPPLY_MAX_CHARGER_VOLTAGEMV);
	twl_config_limit2_reg(CONFIG_POWER_SUPPLY_MAX_CHARGER_CURRENTMA);
	twl_config_voreg_reg(CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV);

	/* MIN full charge volt: 2650mV~3350mV(100mV step) (no effect on TWL6032 power path) */
	twl_set_vbat_comp(200/*mV*/, 3250/*mV*/);

	/* Setup the input current */
	(void)twl_config_cinlimit_reg(CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA);
	/* Setup the charge current */
	(void)twl_config_vichrg_pc(400);
	(void)twl_config_vichrg_reg(CONFIG_POWER_SUPPLY_USB_CHARGE_CURRENTMA);

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			CONTROLLER_INT_MASK); /* Mask interupt generation on BAT_TEMP_OVRANGE event */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MCHARGERUSB_THMREG,
			CHARGERUSB_INT_MASK); /* Mask CHARGERUSB_THMREG interrupt */
}

/*----------------------------------------------------------------------*/

#ifndef CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER
static int twl_usb_charger_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int ret = NOTIFY_DONE;

    switch (event) {
	case POWER_SUPPLY_CHARGER_INIT:
		break;
	case POWER_SUPPLY_CHARGER_DUMP_REGS:
		twl6032_charger_dump_regs();
		break;
	case POWER_SUPPLY_CHARGER_START:
	{
		struct power_supply_charger_profile *profile = (struct power_supply_charger_profile *)data;
		profile->charge_current = twl_charger_start_usb_charger(profile->battery_voltage, profile->input_current);
	}
		break;
	case POWER_SUPPLY_CHARGER_STOP:
		twl_charger_stop_usb_charger();
		break;
	case POWER_SUPPLY_CHARGER_RESET_TIMER:
		twl_charger_kick_watchdog();
		break;
	case POWER_SUPPLY_CHARGER_IS_CHARGING:
		*((int *) data) = twl_charger_is_charging();
		break;
	case POWER_SUPPLY_CHARGER_IS_CHARGING_NO_RESTART:
	{
		int charging = 0;
		u8 state = 0;

		/* Check the USB charger status */
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state, CONTROLLER_STAT1);
		if (state & VBUS_DET) {
			charging = twl6032_is_charging(&state);
		}

		*((int *) data) = charging;
	}
		break;
#ifndef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
	case POWER_SUPPLY_CHARGER_UPDATE_PROFILE:
		twl6032_charger_update_charger_profile(*((int *)data));
		break;
	case POWER_SUPPLY_CHARGER_SET_CURRENT:
	{
		unsigned int prev_active_currentmA = curr_active_currentmA;
		(void)twl6032_charger_setup_charge_current(*((int *)data), twl_charger.usb_currentmA);
		if (prev_active_currentmA != curr_active_currentmA) {
			mdelay(256);
		}
	}
		break;
	case POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY:
	{
		int has_battery = -1;
		int charging = 0;
		u8 state = 0;

		/* Check the USB charger status */
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state, CONTROLLER_STAT1);
		if (state & VBUS_DET)
			charging = twl6032_is_charging(&state);

		if (charging > 0)
			has_battery = 1;

		*((int *) data) = has_battery;
	}
		break;
#else
	case POWER_SUPPLY_CHARGER_UPDATE_PROFILE:
		break;
	case POWER_SUPPLY_CHARGER_SET_CURRENT:
		break;
	case POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY:
		*((int *) data) = -1;
		break;
#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
	case POWER_SUPPLY_CHARGER_IS_BATTERY_LOCKED:
		/* Not Supported */
		*((int *) data) = -1;
		break;
	case POWER_SUPPLY_CHARGER_ADD_ERROR:
		if (twl_charger_errs++ < 13)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		break;
	case POWER_SUPPLY_CHARGER_CLEAR_ERROR:
		if (twl_charger_errs)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		twl_charger_errs = 0;
		break;
    default:
		POWER_SUPPLY_ERR("unknown USB charger event %lu\n", event);
		break;
    }

	return ret;
}
#endif

/*----------------------------------------------------------------------*/

static int twl_fuelgauge_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	switch (event) {
	case POWER_SUPPLY_FG_GET_VSYS: /* int;mV */
		*((int *) data) = twl6032_get_sys_voltage();
		break;
	case POWER_SUPPLY_FG_GET_VBAT: /* int;mV */
		*((int *) data) = twl6032_get_vbat();
		break;
	case POWER_SUPPLY_FG_GET_CURRENT: /* int;mA */
#if (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
		*((int *) data) = CONFIG_POWER_SUPPLY_INVALID_CURRENT;
#else
		*((int *) data) = twl6032_get_battery_current();
#endif
		break;
	case POWER_SUPPLY_FG_GET_CAPACITY: /* int;percentage */
		*((int *) data) = twl6032_get_battery_capacity();
		break;
	case POWER_SUPPLY_FG_IS_FULL_CHARGED:
		if (twl6032_get_battery_capacity() == 100)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		break;
	case POWER_SUPPLY_FG_GET_VBBAT: /* int;mV */
		twl_charger_backup_battery_disable();
		mdelay(128);
		*((int *) data) = twl_get_gpadc_conversion(TWL6032_GPADC_CHANNEL_BK_VBAT);
		twl_charger_backup_battery_setup();
		break;
	case POWER_SUPPLY_FG_GET_VUSB_BUS: /* int;mV */
		*((int *) data) = twl6032_get_usb_voltage();
		break;
	case POWER_SUPPLY_FG_GET_VAC: /* int;mV */
		*((int *) data) = twl6032_get_ac_voltage();
		break;
	case POWER_SUPPLY_FG_GET_TEMP: /* int;mV */
		*((int *) data) = twl6032_get_battery_temp();
		break;
	case POWER_SUPPLY_FG_INIT:
		twl_capacity_timeout = timeout_init(20*1000/*ms*/, &twl_capacity_ticks);
		break;
	case POWER_SUPPLY_FG_HAS_MAIN_BATTERY:
		*((int *) data) = -1;
		break;
	default:
		POWER_SUPPLY_ERR("unknown FG event %lu\n", event);
		break;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static void power_supply_stop_usb_charger(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	POWER_SUPPLY_VDBG("<NO Main Battery> %s\n", __func__);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	POWER_SUPPLY_VDBG("%s\n", __func__);

	if (twl_charger_usb_type != POWER_SUPPLY_TYPE_UNKNOWN && charger_source == twl_charger_usb_type) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_STOP, NULL);

		charger_source = POWER_SUPPLY_TYPE_UNKNOWN;

#ifdef CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER
		if (twl_charger.num_usb_charger_notifiers)
#else
		if (twl_charger.num_usb_charger_notifiers > 1)
#endif
			/* There are more than 1 USB charger IC, so we assume it's the external charger functionality */
			twl_charger_stop_ac_charger();
	}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

static int twl_usb_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	POWER_SUPPLY_VDBG("USB event %lu\n", event);
	switch (event) {
	case USB_EVENT_VBUS:
		twl_charger_usb_type = *((unsigned *)data);
		twl_charger.usb_currentmA = CONFIG_POWER_SUPPLY_USB_CHARGE_CURRENTMA;
		(void)power_supply_start_charger(-1);
		break;
	case USB_EVENT_ENUMERATED:
		if (twl_charger_usb_type == POWER_SUPPLY_TYPE_USB) {
			twl_charger.usb_currentmA = *((unsigned int *) data);
			if (twl_charger.usb_currentmA)
				(void)power_supply_start_charger(-1);
		}
		break;
	case USB_EVENT_CHARGER:
		twl_charger_usb_type = POWER_SUPPLY_TYPE_USB_DCP;
		twl_charger.usb_currentmA = CONFIG_POWER_SUPPLY_USB_AC_CHARGE_CURRENTMA;
		(void)power_supply_start_charger(-1);
		break;
	case USB_EVENT_NONE:
		power_supply_stop_usb_charger();
		twl_charger_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case USB_EVENT_ID:
		power_supply_stop_usb_charger();
		twl_charger_usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		POWER_SUPPLY_ERR("unknown USB event %lu\n", event);
		break;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
static void power_supply_stop_ac_charger(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	POWER_SUPPLY_VDBG("<NO Main Battery> %s\n", __func__);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	POWER_SUPPLY_VDBG("%s\n", __func__);

	if (twl_charger_ac_type != POWER_SUPPLY_TYPE_UNKNOWN && charger_source == twl_charger_ac_type) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_STOP, NULL);

		charger_source = POWER_SUPPLY_TYPE_UNKNOWN;

		twl_charger_stop_ac_charger();
	}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

static int twl_ac_event_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if (twl_charger.num_ac_charger_notifiers) {
		POWER_SUPPLY_VDBG("AC event %lu\n", event);
    	switch (event) {
    	case POWER_SUPPLY_AC_EVENT_CHARGER:
			POWER_SUPPLY_VDBG("AC CHARGER\n");
			twl_charger_ac_type = POWER_SUPPLY_TYPE_MAINS;
			twl_charger.ac_currentmA = CONFIG_POWER_SUPPLY_AC_CHARGE_CURRENTMA;
			(void)power_supply_start_charger(-1);
        	break;
    	case POWER_SUPPLY_AC_EVENT_NONE:
			POWER_SUPPLY_VDBG("AC NONE\n");
			power_supply_stop_ac_charger();
			twl_charger_ac_type = POWER_SUPPLY_TYPE_UNKNOWN;
			/* try the USB charger */
			(void)power_supply_start_charger(-1);
        	break;
    	default:
			POWER_SUPPLY_ERR("unknown AC event %lu\n", event);
			break;
    	}
	} else {
		POWER_SUPPLY_ERR("no AC charger IC\n");
	}

	return 0;
}
#else /* !CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
#define power_supply_stop_ac_charger()
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

/*----------------------------------------------------------------------*/

void twl6032_charger_init(void)
{
	int ret;

	POWER_SUPPLY_VDBG("%s\n", __func__);

	memset(&twl_charger, 0, sizeof(twl_charger));

#ifdef CONFIG_ABOOT_DUMP_CHARGING_REGS
	POWER_SUPPLY_PRINT("\n>>>>>>> Dump Charger regs before init >>>>>>>\n");
	twl6032_charger_dump_regs();
	POWER_SUPPLY_PRINT("<<<<<<< Dump Charger regs before init <<<<<<<\n\n");
#endif /* CONFIG_ABOOT_DUMP_CHARGING_REGS */

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
	/* enforce to stop TWL6032 charger functionality */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	if (twl6032_rev == 1) {
		/*
		 * Set Anti-collapse threshold correspond to the ERRATA DB00119490 (4.4 volts).
		 * Fixed in ES1.2
		 */
		twl_clear_n_set(TWL6030_MODULE_CHARGER,
				BUCK_VTH_MASK, ((0x3) << BUCK_VTH_SHIFT) | ANTICOLL_ANA,
				ANTICOLLAPSE_CTRL1);
	}

	twl6032_gpadc_init();
	twl_charger_backup_battery_setup();
	twl_battery_current_setup(1);

	/* USB Charger notifiers */
	BLOCKING_INIT_NOTIFIER_HEAD(&twl_charger.usb_charger_notifiers);
	twl_charger.num_usb_charger_notifiers = 0;
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	/* AC Charger notifiers */
	BLOCKING_INIT_NOTIFIER_HEAD(&twl_charger.ac_charger_notifiers);
	twl_charger.num_ac_charger_notifiers = 0;
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	/* Fuel Gauge notifiers */
	BLOCKING_INIT_NOTIFIER_HEAD(&twl_charger.fg_notifiers);
	twl_charger.num_fg_notifiers = 0;

	/* USB Charger connection events */
	twl_charger.otg = otg_get_transceiver();
	twl_charger.usb_nb.notifier_call = twl_usb_notifier_call;
	twl_charger.usb_nb.priority = INT_MAX;
	ret = otg_register_notifier(twl_charger.otg, &twl_charger.usb_nb);
	if (ret)
		POWER_SUPPLY_ERR("USB otg err %d\n", ret);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	/* AC Charger connection events */
	twl_charger.ac_nb.notifier_call = twl_ac_event_notifier_call;
	ret = ac_charger_register_notifier(&twl_charger.ac_nb);
	if (ret)
		POWER_SUPPLY_ERR("external AC charger err %d\n", ret);
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

	/* External Fuel Gauge */
#ifdef CONFIG_FUELGAUGE_BQ27541
	bq27541_init();
#endif
#ifdef CONFIG_FUELGAUGE_BQ27520
	bq27520_init();
#endif

	/* Internal Fuel Gauge */
	twl_charger.int_fg_nb.notifier_call = twl_fuelgauge_notifier_call;
	twl_charger.int_fg_nb.priority = INT_MIN;
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_INT_FUELGAUGE, &twl_charger.int_fg_nb);
	if (ret)
		POWER_SUPPLY_ERR("internal Fuel Gauge err %d\n", ret);

	/* External AC/USB Charger */
#ifdef CONFIG_CHARGER_BQ2416x
	bq2416x_init();
#endif

	twl_charger_init();
#ifdef CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER
	/* Disable TWL6032 USB linear charger */
	twl_clear_n_set(TWL6030_MODULE_CHARGER,
			/* clear */ CONTROLLER_CTRL1_EN_LINCH, 0,
			CONTROLLER_CTRL1);
#else
	/* Internal USB Charger */
	twl_charger.usb_int_charger_nb.notifier_call = twl_usb_charger_notifier_call;
	twl_charger.usb_int_charger_nb.priority = INT_MIN;
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_INT_USB_CHARGER, &twl_charger.usb_int_charger_nb);
	if (ret)
		POWER_SUPPLY_ERR("internal USB charger err %d\n", ret);
	else
		POWER_SUPPLY_INFO("Rsense %d mOhm\n", CONFIG_TWL6032_CHARGER_SENSE_RESISTOR);
#endif /* CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

	if (twl_charger.num_usb_charger_notifiers == 0) {
		has_usb_charging = 0;
#if !defined(CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY)
		POWER_SUPPLY_INFO("no USB charger support\n");
#endif
	}

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	if (twl_charger.num_ac_charger_notifiers) {
#if !defined(CONFIG_POWER_SUPPLY_HAS_USB_AND_EXTERNAL_AC_CHARGERS)
		has_usb_charging = 0;
		if (twl_charger.num_usb_charger_notifiers != 0) {
			twl_charger.num_usb_charger_notifiers = 0;
			POWER_SUPPLY_INFO("disabled USB charger support\n");
		}
#endif
	} else {
		POWER_SUPPLY_ERR("no external AC charger chip!\n");
	}
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (!power_supply_has_external_fuelgauge()) {
		POWER_SUPPLY_INFO("no external Fuel Gauge!\n");
	}
#ifdef CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT
	else {
		POWER_SUPPLY_INFO("use internal VBAT\n");
	}
#endif
#endif

}

void twl6032_charger_shutdown(void)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

	twl_charger_kick_watchdog();

	twl_battery_current_setup(0);

	twl6032_gpadc_shutdown();

	/* External Fuel Gauge */
#ifdef CONFIG_FUELGAUGE_BQ27541
	bq27541_shutdown();
#endif
#ifdef CONFIG_FUELGAUGE_BQ27520
	bq27520_shutdown();
#endif

	/* External AC/USB Charger */
#ifdef CONFIG_CHARGER_BQ2416x
	bq2416x_shutdown();
#endif

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	/* enforce to stop TWL6032 charger functionality */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
#if !defined(CONFIG_POWER_SUPPLY_HAS_USB_AND_EXTERNAL_AC_CHARGERS)
	power_supply_stop_usb_charger();
#endif
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_POWER_SUPPLY_DEBUG
/* startcharger */
static int do_start_charger(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	(void)power_supply_start_charger(-1);
	return 0;
}

U_BOOT_CMD(
	startcharger, 1, 0, do_start_charger,
	"Start Charger",
	""
);

/* stopharging */
static int do_stop_charger(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	power_supply_stop_charger();
	return 0;
}

U_BOOT_CMD(
	stopcharger, 1, 0, do_stop_charger,
	"Stop Charger",
	""
);

/* bci */
static int do_dump_charger_regs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	power_supply_dump_regs();

	return 0;
}

U_BOOT_CMD(
	charger, 1, 0,	do_dump_charger_regs,
	"Dump Charger IC registers",
	""
);
#endif /* CONFIG_POWER_SUPPLY_DEBUG */

int do_vbat(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	u8 state = 0;
	int value;

	twl_i2c_read_u8(TWL_MODULE_USB, &state, USB_PRODUCT_ID_LSB);
	POWER_SUPPLY_PRINT("\nTWL6032 USB PID: 0x%02X\n", state);
	POWER_SUPPLY_PRINT("TWL6032 rev: 0x%02X\n", twl6032_rev);
	POWER_SUPPLY_PRINT("TWL6032 EPROM rev: 0x%02X\n", twl6032_eprom_rev);
	POWER_SUPPLY_PRINT("TWL6040 rev: 0x%02X\n\n", twl6040_rev);

	if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
		int charging = power_supply_is_charging();
		if (power_supply_is_full_charged()) {
			if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING)
				POWER_SUPPLY_PRINT("Status: Full Charged (Charging)\n");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_EOC)
				POWER_SUPPLY_PRINT("Status: Full Charged (END of Charging)\n");
			else
				POWER_SUPPLY_PRINT("Status: Full Charged (NOT Charging)\n");
		} else {
			if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING)
				POWER_SUPPLY_PRINT("Status: Charging\n");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_EOC)
				POWER_SUPPLY_PRINT("Status: END of Charging\n");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_NO_BATTERY)
				POWER_SUPPLY_PRINT("Status: NO Battery\n");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT)
				POWER_SUPPLY_PRINT("Status: Battery Temp Fault\n");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP)
				POWER_SUPPLY_PRINT("Status: Battery OVP\n");
			else
				POWER_SUPPLY_PRINT("Status: NOT Charging\n");
		}
	} else
		POWER_SUPPLY_PRINT("Status: Discharge\n");
	value = power_supply_get_battery_current();
	if (value != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
		POWER_SUPPLY_PRINT("Charge Current: %d mA\n", value);
	value = power_supply_get_battery_capacity();
	if (value >= 0)
		POWER_SUPPLY_PRINT("Capacity: %d %%\n\n", value);

	POWER_SUPPLY_PRINT("System: %d mV\n", power_supply_get_sys_voltage());
	POWER_SUPPLY_PRINT("Battery: %d mV\n", power_supply_get_battery_voltage());
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) \
		&& !defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT) \
		&& !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
	if (power_supply_has_external_fuelgauge()) {
		/* TWL6032 internal VBAT */
		int volt = twl6032_emergency_get_vbat();
		if (volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE)
			POWER_SUPPLY_PRINT("TWL6032 Battery: %d mV\n", volt);
		else
			POWER_SUPPLY_PRINT("TWL6032 Battery: N/A\n");
	}
#endif
	POWER_SUPPLY_PRINT("Backup Battery: %d mV\n\n", power_supply_get_backup_battery_voltage());

	POWER_SUPPLY_PRINT("VBUS: %d mV\n", power_supply_get_usb_voltage());
	POWER_SUPPLY_PRINT("VAC: %d mV\n", power_supply_get_ac_voltage());

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN && charger_source != POWER_SUPPLY_TYPE_MAINS
			&& twl_charger.num_usb_charger_notifiers == 1) {
		/* TWL6032 internal USB charger */
		state = 0;
		twl_i2c_read_u8(TWL6030_MODULE_ID2, &state, TWL6032_GPADC_TRIM17);
		state &= RATIO_TLO_MASK;
		POWER_SUPPLY_PRINT("\nTWL6032 GPADC low-temp(%d): 0.%d x VREF\n", state, 2 + state);
		state = 0;
		twl_i2c_read_u8(TWL6030_MODULE_ID2, &state, TWL6032_GPADC_TRIM18);
		state &= RATIO_THI_MASK;
		POWER_SUPPLY_PRINT("TWL6032 GPADC high-temp(%d): 0.%d x VREF\n", state, 1 + state);
		POWER_SUPPLY_PRINT("TWL6032 BAT SW LOW temp: %dmV, HIGH: %dmV\n",
				CONFIG_POWER_SUPPLY_BAT_TEMP_LOW_MV, CONFIG_POWER_SUPPLY_BAT_TEMP_HIGH_MV);
		value = twl6032_get_battery_temp();
		POWER_SUPPLY_PRINT("TWL6032 BAT temp: %d mV\n", value);
	}
#endif /* !CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */

	POWER_SUPPLY_PRINT("\n");

	return 0;
}

U_BOOT_CMD(
	bat, 1, 0, do_vbat,
	"battery voltage measurement",
	""
);

/*----------------------------------------------------------------------*/

int power_supply_register_notifier(enum power_supply_features feature, struct notifier_block *nb)
{
	int ret = -EPERM;

	POWER_SUPPLY_VDBG("%s\n", __func__);

	switch (feature) {
	case POWER_SUPPLY_FEAT_INT_USB_CHARGER:
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER
	case POWER_SUPPLY_FEAT_EXT_USB_CHARGER:
#endif
		ret = blocking_notifier_chain_register(&(twl_charger.usb_charger_notifiers), nb);
		if (!ret) {
			twl_charger.num_usb_charger_notifiers++;
			nb->notifier_call(nb, POWER_SUPPLY_CHARGER_INIT, NULL);
		}
#if defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
		if (twl_charger.num_usb_charger_notifiers > 1)
#else
		if (twl_charger.num_usb_charger_notifiers > 2)
#endif
			POWER_SUPPLY_ERR("too many USB charger ICs %u\n", twl_charger.num_usb_charger_notifiers);
		break;
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	case POWER_SUPPLY_FEAT_EXT_AC_CHARGER:
		ret = blocking_notifier_chain_register(&(twl_charger.ac_charger_notifiers), nb);
		if (!ret) {
			twl_charger.num_ac_charger_notifiers++;
			nb->notifier_call(nb, POWER_SUPPLY_CHARGER_INIT, NULL);
		}
		if (twl_charger.num_ac_charger_notifiers > 1)
			POWER_SUPPLY_ERR("too many AC charger chips %u\n", twl_charger.num_ac_charger_notifiers);
		break;
#endif
	case POWER_SUPPLY_FEAT_INT_FUELGAUGE:
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	case POWER_SUPPLY_FEAT_EXT_FUELGAUGE:
#endif
		ret = blocking_notifier_chain_register(&(twl_charger.fg_notifiers), nb);
		if (!ret) {
			twl_charger.num_fg_notifiers++;
			nb->notifier_call(nb, POWER_SUPPLY_FG_INIT, NULL);
		}
		break;
	default:
		POWER_SUPPLY_ERR("unsupported feature 0x%x\n", feature);
		break;
	}

	return ret;
}

int power_supply_unregister_notifier(enum power_supply_features feature, struct notifier_block *nb)
{
	int ret = -EPERM;

	POWER_SUPPLY_VDBG("%s\n", __func__);

	switch (feature) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	case POWER_SUPPLY_FEAT_EXT_FUELGAUGE:
		ret = blocking_notifier_chain_unregister(&(twl_charger.fg_notifiers), nb);
		if (!ret) {
			twl_charger.num_fg_notifiers--;
		}
		break;
#endif
	default:
		POWER_SUPPLY_ERR("unsupported feature 0x%x\n", feature);
		break;
	}

	return ret;
}

int power_supply_has_ac_feature(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (twl_charger.num_ac_charger_notifiers)
		return 1;
	else
		return 0;
#else
	return 0;
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
}

int power_supply_has_usb_feature(void)
{
	return has_usb_charging;
}

int power_supply_has_external_fuelgauge(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (twl_charger.num_fg_notifiers > 1)
		return 1;
	else
		return 0;
#else
	return 0;
#endif
}

int power_supply_reinit_external_fuelgauge(void)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	if (twl_charger.num_fg_notifiers == 1) {
		/* External Fuel Gauge */
#ifdef CONFIG_FUELGAUGE_BQ27541
		bq27541_init();
#endif
	}

	if (twl_charger.num_fg_notifiers > 1)
		return 1;
	else
		return 0;
#else
	return 0;
#endif
}

unsigned power_supply_get_charger_source(void)
{
	return charger_source;
}

unsigned power_supply_get_usb_type(void)
{
	return twl_charger_usb_type;
}

int power_supply_is_charging(void)
{
	int charging = 0;

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_CHARGING, &charging);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_CHARGING, &charging);
	}

	POWER_SUPPLY_VDBG("%s: %d\n", __func__, charging);

	return charging;
}

static int twl_charger_has_main_battery(void)
{
	int has_battery = -1;

	/* has main battery? */
	if (power_supply_has_external_fuelgauge()) {
		NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_HAS_MAIN_BATTERY, &has_battery);
	}

	if (has_battery < 0) {
		/* has main battery? */
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
		if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
			NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY, &has_battery);
		} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
		if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
			NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY, &has_battery);
		}
	}

	POWER_SUPPLY_VDBG("%s: battery status %d\n", __func__, has_battery);

	return has_battery;
}

int board_has_main_battery(void) __attribute__((weak, alias("twl_charger_has_main_battery")));

int power_supply_has_main_battery(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	return 0;

#else

	return board_has_main_battery();

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

static int twl_charger_is_battery_locked(void)
{
	int is_locked = -1;

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_BATTERY_LOCKED, &is_locked);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_BATTERY_LOCKED, &is_locked);
	}
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE */

	POWER_SUPPLY_VDBG("%s: battery locked? %d\n", __func__, is_locked);

	return is_locked;
}

int board_is_battery_locked(void) __attribute__((weak, alias("twl_charger_is_battery_locked")));

int power_supply_is_battery_locked(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	return 0;

#else

	return board_is_battery_locked();

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

int power_supply_get_battery_current(void)
{
	int current = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_CURRENT, &current);
	return current;
}

int power_supply_get_battery_capacity(void)
{
	int capacity = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_CAPACITY, &capacity);
	return capacity;
}

int power_supply_is_full_charged(void)
{
	int full_charged = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_IS_FULL_CHARGED, &full_charged);
	return full_charged;
}

int power_supply_get_backup_battery_voltage(void)
{
#if (CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV == 0)
	/* No backup battery */
	return 0;
#else
	int volt = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VBBAT, &volt);
	return volt;
#endif /* CONFIG_POWER_SUPPLY_MAX_BACKUP_BAT_VOLTAGEMV */
}

#ifndef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)
#else
static void power_supply_set_charge_current(int profile_index)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_SET_CURRENT, &profile_index);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_SET_CURRENT, &profile_index);
	}
}

static void power_supply_update_charge_profile(int vbat)
{
	POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_UPDATE_PROFILE, &vbat);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_UPDATE_PROFILE, &vbat);
	}
}
#endif /* !CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE && CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS */
#endif /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

int power_supply_get_battery_voltage(void)
{
	int vbat = 0;

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	/* no main battery => return VSYS */
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VSYS, &vbat);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)

	vbat = twl_charger_get_battery_voltage();
	POWER_SUPPLY_VDBG("%s: VBAT %d mV\n", __func__, vbat);

#else

	int charging = POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING;

	/* Is charging? */
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_CHARGING_NO_RESTART, &charging);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_IS_CHARGING_NO_RESTART, &charging);
	}

	POWER_SUPPLY_VDBG("%s: charging status %d\n", __func__, charging);

	/* Dump TWL6032 VBAT for debugging */
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) \
		&& !defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT) \
		&& !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
	if (power_supply_has_external_fuelgauge() && charger_source != POWER_SUPPLY_TYPE_MAINS
			&& charger_source != POWER_SUPPLY_TYPE_UNKNOWN && twl_charger.num_usb_charger_notifiers == 1) {
		/* TWL6032 internal USB charger */
		int volt = twl6032_emergency_get_vbat();
		if (volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE)
			POWER_SUPPLY_PRINT("<<TWL6032 VBAT %dmV>>\n", volt);
	}
#else
	if (power_supply_has_external_fuelgauge() && charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		/* TWL6032 internal VBAT */
		int volt = twl6032_emergency_get_vbat();
		if (volt != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE)
			POWER_SUPPLY_PRINT("<<TWL6032 VBAT %dmV>>\n", volt);
	}
#endif /* CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER */
#endif

#if defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
	/* enforce to use Internal Fuel Gauge */
#else
	/* Internal Fuel Gauge only */
	if (!power_supply_has_external_fuelgauge())
#endif
	{
		/* Apply the lowest charge current before measuring VBAT */
#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER) && (CONFIG_TWL6032_CHARGER_SENSE_RESISTOR == 0)
		if (charger_source != POWER_SUPPLY_TYPE_MAINS && charger_source != POWER_SUPPLY_TYPE_UNKNOWN
				&& twl_charger.num_usb_charger_notifiers == 1) {
			/* TWL6032 internal USB charger && internal VBAT */
			if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
				NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_STOP, NULL);
				charging = POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING;
			} else if (twl_charger_vbat >= 0 && twl_charger_vbat <= CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV) {
				NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_STOP, NULL);
				charging = POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING;
			}
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
			power_supply_set_charge_current(0);
		}
#else
		if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
			power_supply_set_charge_current(0);
		}
#endif /* CONFIG_TWL6032_CHARGER_SENSE_RESISTOR */
	}

	vbat = twl_charger_get_battery_voltage();
	POWER_SUPPLY_VDBG("%s: VBAT %d mV\n", __func__, vbat);

	if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING) {
		/* Restore the charge current */
		power_supply_update_charge_profile(vbat);
	} else if (twl_charger_usb_type != POWER_SUPPLY_TYPE_UNKNOWN
			|| twl_charger_ac_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		/* Not Charging */
		if (charging == POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING)
			(void)power_supply_start_charger(vbat);
	}

#endif /* !CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE && CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS */

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	return vbat;
}

int power_supply_get_sys_voltage(void)
{
	int volt = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VSYS, &volt);
	return volt;
}

int power_supply_get_usb_voltage(void)
{
	int volt = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VUSB_BUS, &volt);
	return volt;
}

int power_supply_get_ac_voltage(void)
{
	int volt = 0;
	NOTIFY_FUELGAUGE_EVENT(POWER_SUPPLY_FG_GET_VAC, &volt);
	return volt;
}

unsigned int power_supply_start_charger(int vbat)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	POWER_SUPPLY_VDBG("%s: <NO Main Battery> skip charger\n", __func__);
	return 0;

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	unsigned int charger_currentmA = 0;

	POWER_SUPPLY_VDBG("%s: source %u, ac %u, usb %u\n", __func__,
			charger_source, twl_charger_ac_type, twl_charger_usb_type);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (twl_charger_ac_type != POWER_SUPPLY_TYPE_UNKNOWN && twl_charger.num_ac_charger_notifiers) {
		struct power_supply_charger_profile profile;

		power_supply_stop_usb_charger();

		POWER_SUPPLY_VDBG("start AC charger: VBAT %d mV; current %u mA\n",
				vbat, twl_charger.ac_currentmA);

		profile.battery_voltage = vbat; /* IN (vbat) */
		profile.input_current = twl_charger.ac_currentmA; /* IN (charge current) */
		profile.charge_current = 0; /* OUT (charge current) */

		charger_source = twl_charger_ac_type;

		/**
		 * Don't disable TWL6032 external AC charger power path.
		 * It will cause the device off when the battery is LOW.
		 */
		twl_charger_start_ac_charger();

		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_START, &profile);

		charger_currentmA = profile.charge_current;
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (twl_charger_usb_type != POWER_SUPPLY_TYPE_UNKNOWN && twl_charger.num_usb_charger_notifiers) {
		struct power_supply_charger_profile profile;

		power_supply_stop_ac_charger();

		POWER_SUPPLY_VDBG("start USB charger: VBAT %d mV; current %u mA\n",
				vbat, twl_charger.usb_currentmA);

		profile.battery_voltage = vbat; /* IN (vbat) */
		profile.input_current = twl_charger.usb_currentmA; /* IN (charge current) */
		profile.charge_current = 0; /* OUT (charge current) */

		charger_source = twl_charger_usb_type;

#ifndef CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER
		if (twl_charger.num_usb_charger_notifiers > 1)
#endif
			/* There are more than 1 USB charger IC, so we assume it's the external charger functionality */
			twl_charger_start_ac_charger();

		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_START, &profile);

		charger_currentmA = profile.charge_current;
	} else {
		charger_source = POWER_SUPPLY_TYPE_UNKNOWN;
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
		if (twl_charger_ac_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			twl_charger_stop_ac_charger();
		} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
		if (twl_charger_usb_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			/* Disable TWL6032 USB linear charger */
			twl_clear_n_set(TWL6030_MODULE_CHARGER,
					/* clear */ CONTROLLER_CTRL1_EN_LINCH, 0,
					CONTROLLER_CTRL1);
		} else {
			/* enforce to stop TWL6032 charger functionality */
			POWER_SUPPLY_INFO("enforce to stop charger function\n");
			twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
		}
	}

	return charger_currentmA;

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

void power_supply_stop_charger(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	POWER_SUPPLY_VDBG("%s: <NO Main Battery> skip stop charger\n", __func__);

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	POWER_SUPPLY_VDBG("%s: source %u, ac %u, usb %u\n", __func__,
			charger_source, twl_charger_ac_type, twl_charger_usb_type);

	power_supply_stop_usb_charger();

	power_supply_stop_ac_charger();

	charger_source = POWER_SUPPLY_TYPE_UNKNOWN;

	if (twl_charger_usb_type == POWER_SUPPLY_TYPE_UNKNOWN
			&& twl_charger_ac_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		/* enforce to stop TWL6032 charger functionality */
		POWER_SUPPLY_INFO("stop charger function\n");
		twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

void power_supply_poll(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	if (timeout_check(&twl_charger_timeout, &twl_charger_ticks)) {
		WATCHDOG_RESET();

		POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
		if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
			NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_RESET_TIMER, NULL);
		} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
		if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
			NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_RESET_TIMER, NULL);
		}

		twl_charger_timeout = timeout_init(CONFIG_POWER_SUPPLY_POLL_MS_TIME, &twl_charger_ticks);
	}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

void power_supply_dump_regs(void)
{
	static u32 dump_times = 0;

	++dump_times;

	POWER_SUPPLY_PRINT("\n[%u]>>>>>>>>>>>>> Dump Charger regs >>>>>>>>>>>>>\n", dump_times);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (power_supply_has_ac_feature()) {
		POWER_SUPPLY_PRINT("AC Charger IC:\n");
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_DUMP_REGS, NULL);
	}
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

	if (power_supply_has_usb_feature()) {
		POWER_SUPPLY_PRINT("USB Charger IC:\n");
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_DUMP_REGS, NULL);
	}

#if !defined(CONFIG_TWL6032_CHARGER_NO_INTERNAL_CHARGER)
	if (twl_charger.num_usb_charger_notifiers > 1) {
		/* There are more than 1 USB charger IC */
		POWER_SUPPLY_PRINT("TWL6032 Charger IC:\n");
		twl6032_charger_dump_regs();
	}
#endif

	POWER_SUPPLY_PRINT("[%u]<<<<<<<<<<<<< Dump Charger regs <<<<<<<<<<<<<\n", dump_times);
}

void power_supply_shutdown(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	POWER_SUPPLY_VDBG("%s: source %u, ac %u, usb %u\n", __func__,
			charger_source, twl_charger_ac_type, twl_charger_usb_type);

	power_supply_stop_usb_charger();

	power_supply_stop_ac_charger();

#if 0
	/* enforce to stop TWL6032 charger functionality */
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	charger_source = POWER_SUPPLY_TYPE_UNKNOWN;
#endif

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

int power_supply_add_charger_error(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	return 0;

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	int restart_charger = 1;

	POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_ADD_ERROR, &restart_charger);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_ADD_ERROR, &restart_charger);
	}

	return restart_charger;

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}

int power_supply_clear_charger_error(void)
{
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	return 0;

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	int restart_charger = 1;

	POWER_SUPPLY_VDBG("%s\n", __func__);

#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (charger_source == POWER_SUPPLY_TYPE_MAINS) {
		NOTIFY_AC_CHARGER_EVENT(POWER_SUPPLY_CHARGER_CLEAR_ERROR, &restart_charger);
	} else
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	if (charger_source != POWER_SUPPLY_TYPE_UNKNOWN) {
		NOTIFY_USB_CHARGER_EVENT(POWER_SUPPLY_CHARGER_CLEAR_ERROR, &restart_charger);
	}

	return restart_charger;

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
}
