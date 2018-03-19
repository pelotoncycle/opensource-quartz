/*
 * (C) Copyright 2013
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

#define POWER_SUPPLY_SUBSYS_NAME	"bq2416x"

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <twl6030.h>

#include <icom/power_supply.h>

#include "bq2416x.h"

/*----------------------------------------------------------------------*/

#define BQ2416x_ADDR		0x6B

/*----------------------------------------------------------------------*/

/* TWL6030 */
#define CONTROLLER_CTRL1		0x01
#define		CONTROLLER_CTRL1_EN_LINCH		(1 << 5)	/* Linear charge enable if EN_CHARGE = 1 */
#define		CONTROLLER_CTRL1_EN_CHARGER		(1 << 4)	/* Charger is enabled */
#define		CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)	/* VAC input supply path is selected */
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
#define CHARGERUSB_STATUS_INT1	0x06
#define CHARGERUSB_STATUS_INT2	0x07
#define CHARGERUSB_CTRL1		0x08
/* TWL6032 */
#define LINEAR_CHRG_STS			0x04

/*----------------------------------------------------------------------*/

/* BQ24160 / BQ24161  */

#define	BQ24160					24160

#define NBR_OF_REGISTERS        0x08

/* Status/Control Register */
#define REG_STATUS_CONTROL			0x00
#define		TIMER_RST				(1 << 7)	/* write "1" to reset the watchdog timer */
/** STATUS (STAT_0~STAT_2)
 * 0x00: No Valid Source Detected
 * 0x01: IN Ready
 * 0x02: USB Ready
 * 0x03: Charging from IN
 * 0x04: Charging from USB
 * 0x05: Charge Done
 * 0x06: NA
 * 0x07: Fault
 */
#define		STATUS_SHIFT			4
#define		STATUS_MASK				(0x7 << STATUS_SHIFT)	/* charging status */
#define		STATUS_NO_VALID_SOURCE		(0x00 << STATUS_SHIFT)
#define		STATUS_IN_READY				(0x01 << STATUS_SHIFT)
#define		STATUS_USB_READY			(0x02 << STATUS_SHIFT)
#define		STATUS_CHARGING_FROM_IN		(0x03 << STATUS_SHIFT)
#define		STATUS_CHARGING_FROM_USB	(0x04 << STATUS_SHIFT)
#define		STATUS_CHARGE_DONE			(0x05 << STATUS_SHIFT)
#define		STATUS_NA					(0x06 << STATUS_SHIFT)
#define		STATUS_FAULT				(0x07 << STATUS_SHIFT)
#define		SUPPLY_SEL				(1 << 3)	/* 0: IN, 1: USB */
/** FAULT (FAULT_0~FAULT_2)
 * 0x00: Normal
 * 0x01: Thermal Shutdown
 * 0x02: Battery Temperature Fault
 * 0x03: Watchdog Timer Expired
 * 0x04: Safety Timer Expired
 * 0x05: IN Supply Fault
 * 0x06: USB Supply Fault
 * 0x07: Battery Fault
 */
#define		FAULT_SHIFT				0
#define		FAULT_MASK				(0x7 << FAULT_SHIFT)	/* fault reason */
#define		FAULT_THERMAL				(0x01 << FAULT_SHIFT)
#define		FAULT_BAT_TEMP				(0x02 << FAULT_SHIFT)
#define		FAULT_WATCHDOG				(0x03 << FAULT_SHIFT)
#define		FAULT_SAFETY_TIMER			(0x04 << FAULT_SHIFT)
#define		FAULT_IN_SUPPLY				(0x05 << FAULT_SHIFT)
#define		FAULT_USB_SUPPLY			(0x06 << FAULT_SHIFT)
#define		FAULT_BATTERY				(0x07 << FAULT_SHIFT)

/* Battery/Supply Status Register */
#define REG_BATT_SUPPLY_REGISTER	0x01
#define		IN_STATUS_SHIFT			6
#define		IN_STATUS_MASK			(0x03 << IN_STATUS_SHIFT)
#define			IN_STATUS_OVP		(0x01 << IN_STATUS_SHIFT)
#define			IN_STATUS_WEAK		(0x02 << IN_STATUS_SHIFT)
#define			IN_STATUS_UVLO		(0x03 << IN_STATUS_SHIFT)
#define		USB_STATUS_SHIFT		4
#define		USB_STATUS_MASK			(0x03 << USB_STATUS_SHIFT)
#define			USB_STATUS_OVP		(0x01 << USB_STATUS_SHIFT)
#define			USB_STATUS_WEAK		(0x02 << USB_STATUS_SHIFT)
#define			USB_STATUS_UVLO		(0x03 << USB_STATUS_SHIFT)
#define		OTG_LOCK				(1 << 3)
#define		BAT_STATUS_SHIFT		1
#define		BAT_STATUS_MASK			(0x03 << BAT_STATUS_SHIFT)
#define			BAT_STATUS_OVP		(0x01 << BAT_STATUS_SHIFT)
#define			BAT_STATUS_NOBAT	(0x02 << BAT_STATUS_SHIFT)
#define		EN_NOBATOP				(1 << 0)

/* Control Register */
#define REG_CONTROL_REGISTER		0x02
#define		RESET_CHIP				(1 << 7)
#define		USB_CURRENT_LIMIT_SHIFT	4
#define		USB_CURRENT_LIMIT_MASK	(0x07 << USB_CURRENT_LIMIT_SHIFT)
#define		EN_STAT_OUT				(1 << 3)
#define		EOC_EN					(1 << 2)
#define		CHARGE_DISABLE			(1 << 1)
#define		HiZ_MODE				(1 << 0)

/* Control/Battery Voltage Register */
#define REG_BATTERY_VOLTAGE			0x03
#define		BATT_VOLTAGE_SHIFT		2	/* offset 3.5V, step 20mV that is (a*20 + 3500mV) */
#define		BATT_VOLTAGE_MASK		(0x3F << BATT_VOLTAGE_SHIFT)
#define		I_LIMIT_SEL_2A5			(1 << 1)
#define		DP_DM_DET_EN			(1 << 0)

/* Vender/Part/Revision Register */
#define REG_PART_REVISION			0x04

/* Battery Termination/Fast Charge Current Register */
#define REG_BATTERY_CURRENT			0x05
#define		CHARGE_CURRENT_SHIFT	3	/* a*75 + 550mA */
#define		CHARGE_CURRENT_MASK		(0x1F << CHARGE_CURRENT_SHIFT)
#define		EOC_CURRENT_SHIFT		0	/* a*50 + 50 mA*/
#define		EOC_CURRENT_MASK		(0x07 << EOC_CURRENT_SHIFT)

/* VIN-DPM Voltage/ DPPM Status Register */
#define REG_DPM_VOLTAGE_AND_STATUS	0x06
#define		MINSYS_STATUS			(1 << 7)
#define		DPM_STATUS				(1 << 6)
#define		USB_DPM_VOLTAGE_SHIFT	3  /*a*80 +4200mV*/
#define		USB_DPM_VOLTAGE_MASK	(0x07 << USB_DPM_VOLTAGE_SHIFT)
#define		IN_DPM_VOLTAGE_SHIFT	0  /*a*80 +4200mV*/
#define		IN_DPM_VOLTAGE_MASK		(0x07 << IN_DPM_VOLTAGE_SHIFT)

/* Safety Timer/ NTC Monitor Register */
#define REG_SAFETY_LIMIT			0x07
#define		TMR2X_EN				(1 << 7)	/* Timer slowed by 2x when in thermal regulation, input current limit, Vin_dpm or DPPM */
#define		SAFETY_TIMER_SHIFT		5
#define		SAFETY_TIMER_MASK		(0x03 << SAFETY_TIMER_SHIFT)
#define		SAFETY_TIMER_27MIN		(0x00 << SAFETY_TIMER_SHIFT)	/* 27 minute fast charge */
#define		SAFETY_TIMER_6HR		(0x01 << SAFETY_TIMER_SHIFT)	/* 6 hour fast charge */
#define		SAFETY_TIMER_9HR		(0x02 << SAFETY_TIMER_SHIFT)	/* 9 hour fast charge */
#define		SAFETY_TIMER_DISABLE	(0x03 << SAFETY_TIMER_SHIFT)	/* Disable safety timer */
#define		SAFETY_TIMER_DEFAULT	SAFETY_TIMER_9HR
#define		TS_EN					(1 << 3)	/* temp sense (TS) function enabled */
/* TS_FAULT (TS_FAULT0~TS_FAULT1)
 * 0x00: Normal, No TS fault
 * 0x01: TStemp < Tcold or TStemp > Thot (Charging suspended)
 * 0x02: Tcool > TStemp > Tcold (Charge current reduced by half, bq24160 only)
 * 0x03: Twarm < TStemp < Thot (Charge voltage reduced by 140mV, bq24160 only)
 */
#define		TS_FAULT_SHIFT			1
#define		TS_FAULT_MASK			(0x03 << TS_FAULT_SHIFT)
#define		LOW_CHG					(1 << 0)	/* Charge current is half programmed value */

/*----------------------------------------------------------------------*/

static int has_bq2416x __attribute__ ((section (".data"))) = 0;
static u8 registers[NBR_OF_REGISTERS] __attribute__ ((section (".data")));
static struct notifier_block bq_charger_nb __attribute__ ((section (".data")));

/*----------------------------------------------------------------------*/

static struct bq2416x_charger_profile charger_profiles[] __attribute__ ((section (".data"))) =
		CONFIG_CHARGER_BQ2416x_PROFILES;
static int nbr_of_charger_profiles __attribute__ ((section (".data"))) = ARRAY_SIZE(charger_profiles);
static int curr_profile_idx __attribute__ ((section (".data"))) = 0;
static int curr_active_profile_idx __attribute__ ((section (".data"))) = -1;
static unsigned int curr_active_currentmA __attribute__ ((section (".data"))) = 0;
static unsigned int curr_active_input_currentmA __attribute__ ((section (".data"))) = 0;

static int bq2416x_poweron_minimal_vsys __attribute__ ((section (".data"))) = 0;

static int bq2416x_has_no_main_battery __attribute__ ((section (".data"))) = 0;
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
static unsigned long bq2416x_dump_timeout __attribute__ ((section (".data"))) = 0;
static unsigned long bq2416x_dump_ticks __attribute__ ((section (".data"))) = 0;
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

/*----------------------------------------------------------------------*/

static unsigned int bq2416x_charge_currentmA __attribute__ ((section (".data"))) = 0;

/* Workaround to decrease the charging current */
static unsigned int bq2416x_charger_errs __attribute__ ((section (".data"))) = 0;

/*----------------------------------------------------------------------*/

static unsigned int bq2416x_start_charger(int vbat, unsigned int currentmA);
static unsigned int bq2416x_setup_charge_current(int profile_idx, unsigned int currentmA);

/*----------------------------------------------------------------------*/

static const char* bq2416x_revision[8] = {
/* 000 */ "1.0",
/* 001 */ "1.1",
/* 010 */ "2.0",
/* 011 */ "2.1",
/* 100 */ "2.2",
/* 101 */ "2.3",
/* 110 */ "0b110",
/* 111 */ "0b111",
};

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
static const char* register_name[NBR_OF_REGISTERS] = {
	"REG_STATUS_CONTROL",
	"REG_BATT_SUPPLY_REGISTER",
	"REG_CONTROL_REGISTER",
	"REG_BATTERY_VOLTAGE",
	"REG_PART_REVISION",
	"REG_BATTERY_CURRENT",
	"REG_DPM_VOLTAGE_AND_STATUS",
	"REG_SAFETY_LIMIT",
};
#endif

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void bq2416x_save_i2c_bus(void) __attribute__((always_inline));
static inline void bq2416x_save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_CHARGER_BQ2416x_I2C_BUS)
		i2c_set_bus_num(CONFIG_CHARGER_BQ2416x_I2C_BUS);
}

static inline void bq2416x_restore_i2c_bus(void) __attribute__((always_inline));
static inline void bq2416x_restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_CHARGER_BQ2416x_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define bq2416x_save_i2c_bus()
#define bq2416x_restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static int bq2416x_write_byte(int retries, u8 value, u8 reg)
{
	int ret;

	bq2416x_save_i2c_bus();

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	POWER_SUPPLY_VDBG("write(0x%02x): [0x%02x]=0x%02x...\n", BQ2416x_ADDR, reg, value);
#endif

_retry:
	ret = i2c_write(BQ2416x_ADDR, reg, 1, &value, 1);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: write(0x%02x): [0x%02x]=0x%02x err %d\n", retries, BQ2416x_ADDR, reg, value, ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
			POWER_SUPPLY_ERR("write(0x%02x): [0x%02x]=0x%02x err %d\n", BQ2416x_ADDR, reg, value, ret);
		}
	}

	bq2416x_restore_i2c_bus();

	return ret;
}

static int bq2416x_read_byte(int retries, u8 *value, u8 reg)
{
	int ret;

	bq2416x_save_i2c_bus();

_retry:
	ret = i2c_read(BQ2416x_ADDR, reg, 1, value, 1);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: read(0x%02x): [0x%02x] err %d\n", retries, BQ2416x_ADDR, reg, ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
			POWER_SUPPLY_ERR("read(0x%02x): [0x%02x] err %d\n", BQ2416x_ADDR, reg, ret);
		}
	} else {
#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
		POWER_SUPPLY_VDBG("read(0x%02x): [0x%02x]=0x%02x\n", BQ2416x_ADDR, reg, *value);
#endif
	}

	bq2416x_restore_i2c_bus();

	return ret;
}

static int bq2416x_read_registers(void)
{
	int i;
	int ret;

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	puts("==============================================\n");
#endif
	for (i = 0; i < NBR_OF_REGISTERS; i++) {
		ret = bq2416x_read_byte(3, &(registers[i]), i);
		if (ret)
			return ret;
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		printf("%-35s 0x%02x\n", register_name[i], registers[i]);
#endif
	}
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	puts("==============================================\n");
#endif

	registers[REG_STATUS_CONTROL] &= ~(STATUS_MASK | FAULT_MASK);
	registers[REG_STATUS_CONTROL] |= TIMER_RST;
	registers[REG_BATT_SUPPLY_REGISTER] &= ~(IN_STATUS_MASK | USB_STATUS_MASK | BAT_STATUS_MASK);
	registers[REG_CONTROL_REGISTER] &= ~RESET_CHIP;
	registers[REG_CONTROL_REGISTER] |= EN_STAT_OUT;
	if (registers[REG_DPM_VOLTAGE_AND_STATUS] & MINSYS_STATUS)
		bq2416x_poweron_minimal_vsys = 1;
	registers[REG_DPM_VOLTAGE_AND_STATUS] &= ~MINSYS_STATUS;

	return 0;
}

/*----------------------------------------------------------------------*/

/* Refer to twl6032_get_charger_profile_index() */
static int bq2416x_get_charger_profile_index(int vbat)
{
	int index, i, j;
	struct bq2416x_charger_profile *curr_profile;

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

/* Refer to twl6032_charger_update_charger_profile() */
static void bq2416x_update_charger_profile(int vbat)
{
	POWER_SUPPLY_VDBG("%s(%dmV)\n", __func__, vbat);

	if (bq2416x_charge_currentmA) {
		int profile_idx;

		profile_idx = bq2416x_get_charger_profile_index(vbat);
		if (profile_idx != curr_profile_idx) {
			POWER_SUPPLY_VDBG("%s: charger profile %d<->%d\n", __func__, curr_profile_idx, profile_idx);
			(void)bq2416x_start_charger(vbat, bq2416x_charge_currentmA);
		} else {
			(void)bq2416x_setup_charge_current(profile_idx, bq2416x_charge_currentmA);
		}
	}
}

/*----------------------------------------------------------------------*/

static inline u8 bq2416x_get_status_control(void)
{
	u8 value = 0;
	(void)bq2416x_read_byte(3, &value, REG_STATUS_CONTROL);
	return value;
}

static inline u8 bq2416x_get_battery_supply_status(void)
{
	u8 value = 0;
	(void)bq2416x_read_byte(3, &value, REG_BATT_SUPPLY_REGISTER);
	return value;
}

/*----------------------------------------------------------------------*/

#define SHOW_REG(r) \
	value = 0; \
	bq2416x_read_byte(3, &value, r); \
	printf("%-35s 0x%02x\n", #r, value);

static void bq2416x_dump_status_control(u8 value)
{
	switch (value & STATUS_MASK) {
	case STATUS_NO_VALID_SOURCE:
		puts("<No Valid Source Detected>");
		break;
	case STATUS_IN_READY:
		puts("<IN Ready>");
		break;
	case STATUS_USB_READY:
		puts("<USB Ready>");
		break;
	case STATUS_CHARGING_FROM_IN:
		puts("<Charging from IN>");
		break;
	case STATUS_CHARGING_FROM_USB:
		puts("<Charging from USB>");
		break;
	case STATUS_CHARGE_DONE:
		puts("<Charge Done>");
		break;
	case STATUS_NA:
		puts("<NA>");
		break;
	case STATUS_FAULT:
		puts("<Fault>");
		break;
	default:
		break;
	}

	switch (value & FAULT_MASK) {
	case FAULT_THERMAL:
		puts("<Thermal Shutdown>");
		break;
	case FAULT_BAT_TEMP:
		puts("<Battery Temperature Fault>");
		break;
	case FAULT_WATCHDOG:
		puts("<Watchdog Timer Expired>");
		break;
	case FAULT_SAFETY_TIMER:
		puts("<Safety Timer Expired>");
		break;
	case FAULT_IN_SUPPLY:
		puts("<IN Supply Fault>");
		break;
	case FAULT_USB_SUPPLY:
		puts("<USB Supply Fault>");
		break;
	case FAULT_BATTERY:
		puts("<Battery Fault>");
		break;
	break;
		break;
	}

	putc('\n');
}

static void bq2416x_dump_batt_supply(u8 value)
{
	switch (value & IN_STATUS_MASK) {
	case IN_STATUS_OVP:
		puts("<IN Supply OVP>");
		break;
	case IN_STATUS_WEAK:
		puts("<IN Weak Source Connected>");
		break;
	case IN_STATUS_UVLO:
		puts("<Vin < Vuvlo>");
		break;
	}

	switch (value & USB_STATUS_MASK) {
	case USB_STATUS_OVP:
		puts("<USB Supply OVP>");
		break;
	case USB_STATUS_WEAK:
		puts("<USB Weak Source Connected>");
		break;
	case USB_STATUS_UVLO:
		puts("<Vusb < Vuvlo>");
		break;
	}

	if (value & OTG_LOCK)
		puts("<OTG supply present>");

	switch (value & BAT_STATUS_MASK) {
	case BAT_STATUS_OVP:
		puts("<Battery OVP>");
		break;
	case BAT_STATUS_NOBAT:
		puts("<Battery Not Present>");
		break;
	}

	if (value & EN_NOBATOP)
		puts("<No Battery Operation enabled>");

	if (value)
		putc('\n');

#if defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY) && defined(CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY)
	if (value & (IN_STATUS_MASK | USB_STATUS_MASK))
		POWER_SUPPLY_INFO("VAC: %d mV; VUSB/VBUS: %d mV\n",
				power_supply_get_ac_voltage(), power_supply_get_usb_voltage());
#elif defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY)
	if (value & IN_STATUS_MASK)
		POWER_SUPPLY_INFO("VAC: %d mV\n", power_supply_get_ac_voltage());
#else
	if (value & USB_STATUS_MASK)
		POWER_SUPPLY_INFO("VUSB/VBUS: %d mV\n", power_supply_get_usb_voltage());
#endif
}

static void bq2416x_dump_control(u8 value)
{
	if (value & EN_STAT_OUT)
		puts("<STAT output>");
	if (value & EOC_EN)
		puts("<EOC>");
	if (value & CHARGE_DISABLE)
		puts("<Charging disabled>");
	if (value & HiZ_MODE)
		puts("<HiZ>");
	if (value & (EN_STAT_OUT | EOC_EN | CHARGE_DISABLE | HiZ_MODE))
		putc('\n');
}

static void bq2416x_dump_regs(void)
{
	u8 value;

	puts("==============================================\n");
	SHOW_REG(REG_STATUS_CONTROL);
	bq2416x_dump_status_control(value);

	SHOW_REG(REG_BATT_SUPPLY_REGISTER);
	bq2416x_dump_batt_supply(value);

	SHOW_REG(REG_CONTROL_REGISTER);
	bq2416x_dump_control(value);

	SHOW_REG(REG_BATTERY_VOLTAGE);
	SHOW_REG(REG_BATTERY_CURRENT);

	SHOW_REG(REG_DPM_VOLTAGE_AND_STATUS);
	if (value & MINSYS_STATUS)
		puts("<Minimum VSYS mode is active (Low Battery)>\n");
	if (value & DPM_STATUS)
		puts("<Vin-DPM mode is active>\n");

	SHOW_REG(REG_SAFETY_LIMIT);
	switch ((value & TS_FAULT_MASK) >> TS_FAULT_SHIFT) {
	case 0x01:
		puts("<Charging suspended> TStemp < Tcold or TStemp > Thot\n");
		break;
	case 0x02:
		puts("<Charge current reduced by half> Tcool > TStemp > Tcold\n");
		break;
	case 0x03:
		puts("<Charge voltage reduced by 140mV> Twarm < TStemp < Thot\n");
		break;
	}
	if (value & LOW_CHG)
		puts("<Charge current is half programmed value>\n");
	puts("----------------------------------------------\n");

	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CONTROLLER_CTRL1);
	printf("%-35s 0x%02x\n", "twl6032 CONTROLLER_CTRL1", value);
	if (value & (CONTROLLER_CTRL1_EN_LINCH | CONTROLLER_CTRL1_EN_CHARGER | CONTROLLER_CTRL1_SEL_CHARGER)) {
		if (value & CONTROLLER_CTRL1_EN_LINCH)
			puts("<EN_LINCH>");
		if (value & CONTROLLER_CTRL1_EN_CHARGER)
			puts("<EN_CHARGER>");
		if (value & CONTROLLER_CTRL1_SEL_CHARGER)
			puts("<SEL_CHARGER>");
		putc('\n');
	}

	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CONTROLLER_STAT1);
	printf("%-35s 0x%02x\n", "twl6032 CONTROLLER_STAT1", value);
	if (value) {
		if (value & CHRG_EXTCHRG_STATZ)
			puts("<EXTCHRG_STATZ>");
		if (value & LINCH_GATED_STAT)
			puts("<LINCH_GATED>");
		if (value & CHRG_DET_N)
			puts("<CHRG_DET_N>");
		if (value & FAULT_WDG)
			puts("<FAULT_WDG>");
		if (value & VAC_DET)
			puts("<VAC>");
		if (value & VBUS_DET)
			puts("<VBUS>");
		if (value & BAT_REMOVED)
			puts("<BAT_REMOVED>");
		if (value & BAT_TEMP_OVRANGE)
			puts("<TEMP_OVRANGE>");
		putc('\n');
	}

	puts("==============================================\n");
}

#undef SHOW_REG

#if 0
static int bq2416x_is_minimal_vsys(void)
{
	int ret;
	u8 value = 0;

	ret = bq2416x_read_byte(3, &value, REG_DPM_VOLTAGE_AND_STATUS);
	if (ret)
		return 1;

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("MINSYS: %s\n", (value & MINSYS_STATUS) ? "Yes" : "No");
#endif

	if (value & MINSYS_STATUS)
		return 1;
	else
		return 0;
}
#endif

static void bq2416x_reset_timer(void)
{
	registers[REG_STATUS_CONTROL] |= TIMER_RST;
	bq2416x_write_byte(3, registers[REG_STATUS_CONTROL], REG_STATUS_CONTROL);
}

static void bq2416x_select_supply(int ac_in)
{
	registers[REG_STATUS_CONTROL] |= TIMER_RST;

	if (ac_in) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("select AC IN supply\n");
#endif
		registers[REG_STATUS_CONTROL] &= ~SUPPLY_SEL;	/* AC IN */
	} else {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("select USB supply\n");
#endif
		registers[REG_STATUS_CONTROL] |= SUPPLY_SEL;	/* USB */
	}

	bq2416x_write_byte(3, registers[REG_STATUS_CONTROL], REG_STATUS_CONTROL);
}

static unsigned int bq2416x_set_charger_current(unsigned int currentmA)
{
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_VDBG("%s: %umA\n", __func__, currentmA);
#endif

	if (currentmA < 550)
		currentmA = 550;
	else if (currentmA > 2500)
		currentmA = 2500;

#if 0
	currentmA = (currentmA - 550) / 75;
#else
	/* Use DIV_ROUND() for the better charge current */
	currentmA = (currentmA - 550 + 37) / 75;
#endif

	registers[REG_BATTERY_CURRENT] &= ~CHARGE_CURRENT_MASK;
	registers[REG_BATTERY_CURRENT] |= (currentmA << CHARGE_CURRENT_SHIFT);
	bq2416x_write_byte(3, registers[REG_BATTERY_CURRENT], REG_BATTERY_CURRENT);

	return currentmA * 75 + 550;
}

static void bq2416x_set_eoc_enable(int enable)
{
	registers[REG_CONTROL_REGISTER] &= ~RESET_CHIP;

	if (enable) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("enable charge current termination\n");
#endif
		registers[REG_CONTROL_REGISTER] |= EOC_EN;
	} else {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("disable charge current termination\n");
#endif
		registers[REG_CONTROL_REGISTER] &= ~EOC_EN;
	}

	bq2416x_write_byte(3, registers[REG_CONTROL_REGISTER], REG_CONTROL_REGISTER);
}

static void bq2416x_set_eoc_current(int currentmA)
{
	if (currentmA < 50)
		currentmA = 50;
	else if (currentmA > 400)
		currentmA = 400;

	currentmA = (currentmA - 50) / 50;
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %umA\n", __func__, 50 + currentmA * 50);
#endif

	registers[REG_BATTERY_CURRENT] &= ~EOC_CURRENT_MASK;
	registers[REG_BATTERY_CURRENT] |= (currentmA << EOC_CURRENT_SHIFT);
	bq2416x_write_byte(3, registers[REG_BATTERY_CURRENT], REG_BATTERY_CURRENT);
}

static void bq2416x_set_regulation_voltage(int voltagemV)
{
	if (voltagemV < 3500)
		voltagemV = 3500;
	else if (voltagemV > 4440)
		voltagemV = 4440;

	voltagemV = (voltagemV - 3500) / 20;
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %dmV\n", __func__, 3500 + voltagemV * 20);
#endif

	registers[REG_BATTERY_VOLTAGE] &= ~BATT_VOLTAGE_MASK;
	registers[REG_BATTERY_VOLTAGE] |= (voltagemV << BATT_VOLTAGE_SHIFT);
	bq2416x_write_byte(3, registers[REG_BATTERY_VOLTAGE], REG_BATTERY_VOLTAGE);
}

static unsigned int bq2416x_set_in_current_limit(unsigned int currentmA)
{
	if (currentmA > 1500) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("AC input current: 2500mA\n");
#endif
		registers[REG_BATTERY_VOLTAGE] |= I_LIMIT_SEL_2A5;
		currentmA = 2500;
	} else {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("AC input current: 1500mA\n");
#endif
		registers[REG_BATTERY_VOLTAGE] &= ~I_LIMIT_SEL_2A5;
		currentmA = 1500;
	}

	bq2416x_write_byte(3, registers[REG_BATTERY_VOLTAGE], REG_BATTERY_VOLTAGE);

	return currentmA;
}

static unsigned int bq2416x_set_usb_current_limit(unsigned int currentmA)
{
	u8 current;

	if (currentmA < 150) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 100mA\n");
#endif
		current = 0; /* 100mA */
		currentmA = 100;
	} else if(currentmA < 500) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 150mA\n");
#endif
		current = 1; /* 150mA */
		currentmA = 150;
	} else if(currentmA < 800) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 500mA\n");
#endif
		current = 2; /* 500mA */
		currentmA = 500;
	} else if(currentmA < 900) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 800mA\n");
#endif
		current = 3; /* 800mA */
		currentmA = 800;
	} else if(currentmA < 1500) {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 900mA\n");
#endif
		current = 4; /* 900mA */
		currentmA = 900;
	} else {
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		POWER_SUPPLY_DBG("USB input current: 1500mA\n");
#endif
		current = 5; /* 1500mA */
		currentmA = 1500;
	}

	registers[REG_CONTROL_REGISTER] &= ~(RESET_CHIP | USB_CURRENT_LIMIT_MASK);
	registers[REG_CONTROL_REGISTER] |= (current << USB_CURRENT_LIMIT_SHIFT);
	bq2416x_write_byte(3, registers[REG_CONTROL_REGISTER], REG_CONTROL_REGISTER);

	return currentmA;
}

static void bq2416x_set_in_vdpm(int vdpm)
{
	if (vdpm < 4200)
		vdpm = 4200;
	else if (vdpm > 4760)
		vdpm = 4760;

	vdpm = (vdpm - 4200) / 80;
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %dmV\n", __func__, 4200 + vdpm * 80);
#endif

	registers[REG_DPM_VOLTAGE_AND_STATUS] &= IN_DPM_VOLTAGE_MASK;
	registers[REG_DPM_VOLTAGE_AND_STATUS] |= (vdpm << IN_DPM_VOLTAGE_SHIFT);

	bq2416x_write_byte(3, registers[REG_DPM_VOLTAGE_AND_STATUS], REG_DPM_VOLTAGE_AND_STATUS);
}

static void bq2416x_set_usb_vdpm(int vdpm)
{
	if (vdpm < 4200)
		vdpm = 4200;
	else if (vdpm > 4760)
		vdpm = 4760;

	vdpm = (vdpm - 4200) / 80;
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %dmV\n", __func__, 4200 + vdpm * 80);
#endif

	registers[REG_DPM_VOLTAGE_AND_STATUS] &= USB_DPM_VOLTAGE_MASK;
	registers[REG_DPM_VOLTAGE_AND_STATUS] |= (vdpm << USB_DPM_VOLTAGE_SHIFT);

	bq2416x_write_byte(3, registers[REG_DPM_VOLTAGE_AND_STATUS], REG_DPM_VOLTAGE_AND_STATUS);
}

static void bq2416x_config_safety_timer(u8 value)
{
	value &= SAFETY_TIMER_MASK;
	registers[REG_SAFETY_LIMIT] &= ~SAFETY_TIMER_MASK;
	if (value == SAFETY_TIMER_DISABLE) {
		registers[REG_SAFETY_LIMIT] &= ~TS_EN;
		registers[REG_SAFETY_LIMIT] |= value;
	} else {
		registers[REG_SAFETY_LIMIT] |= value | TS_EN;
	}

	bq2416x_write_byte(3, registers[REG_SAFETY_LIMIT], REG_SAFETY_LIMIT);
}

static void bq2416x_set_low_charge_current(int enable)
{
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("low charge current: %d\n", enable);
#endif

	if (enable)
		registers[REG_SAFETY_LIMIT] |= LOW_CHG;
	else
		registers[REG_SAFETY_LIMIT] &= ~LOW_CHG;

	bq2416x_write_byte(3, registers[REG_SAFETY_LIMIT], REG_SAFETY_LIMIT);
}

static void bq2416x_enable_charge(int enable)
{
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %d\n", __func__, enable);
#endif

	registers[REG_CONTROL_REGISTER] &= ~RESET_CHIP;
	registers[REG_CONTROL_REGISTER] |= EN_STAT_OUT;

	if (enable)
		registers[REG_CONTROL_REGISTER] &= ~(CHARGE_DISABLE | HiZ_MODE);
	else
		registers[REG_CONTROL_REGISTER] |= CHARGE_DISABLE;

	bq2416x_write_byte(3, registers[REG_CONTROL_REGISTER], REG_CONTROL_REGISTER);
}

#if !defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY) \
	|| (defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT))
static void bq2416x_enable_HiZ(int enable)
{
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("%s: %d\n", __func__, enable);
#endif

	registers[REG_CONTROL_REGISTER] &= ~RESET_CHIP;
	registers[REG_CONTROL_REGISTER] |= EN_STAT_OUT;

	if (enable)
		registers[REG_CONTROL_REGISTER] |= HiZ_MODE;
	else
		registers[REG_CONTROL_REGISTER] &= ~HiZ_MODE;

	bq2416x_write_byte(3, registers[REG_CONTROL_REGISTER], REG_CONTROL_REGISTER);
}
#endif

#if 0
static void bq2416x_force_reset(void)
{
	u8 value;

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("force reset\n");
#endif

	value = registers[REG_CONTROL_REGISTER] | RESET_CHIP;

	if (bq2416x_write_byte(3, value, REG_CONTROL_REGISTER) == 0)
		mdelay(50);
}
#endif

/*----------------------------------------------------------------------*/

/* Refer to twl6032_charger_setup_charge_current() */
static unsigned int bq2416x_setup_charge_current(int profile_idx, unsigned int currentmA)
{
	struct bq2416x_charger_profile *curr_profile;
	unsigned int supply_path = 0; /* USB */
	unsigned int decrease_currentmA;
	unsigned int input_currentmA;
	int use_half_charge_current = 0;

	POWER_SUPPLY_VDBG("%s(%d, %umA)\n", __func__, profile_idx, currentmA);

	if (profile_idx < 0) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", profile_idx);
		profile_idx = 0;
	} else if (profile_idx >= nbr_of_charger_profiles) {
		POWER_SUPPLY_ERR("incorrect profile: %d\n", profile_idx);
		profile_idx = nbr_of_charger_profiles - 1;
	}

	if (currentmA == 0) {
		POWER_SUPPLY_INFO("setup charger current (0 mA) (source %u)\n",
			power_supply_get_charger_source());
	}

	if (curr_active_currentmA && curr_active_profile_idx == profile_idx) {
		return curr_active_currentmA;
	}

	curr_active_profile_idx = profile_idx;
	curr_profile = charger_profiles + profile_idx;
	use_half_charge_current = curr_profile->use_half_charge_current;
#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("PROFILE %d: [%d..%dmV..%d] INPUT %umA, CHARGE %umA%s\n",
			profile_idx,
			curr_profile->battery_voltage - curr_profile->extra_decrease_voltage,
			curr_profile->battery_voltage,
			curr_profile->battery_voltage + curr_profile->extra_increase_voltage,
			curr_profile->input_current, curr_profile->charge_current,
			use_half_charge_current ? " (Half)" : "");
#endif

	if (curr_profile->charge_current < currentmA)
		currentmA = curr_profile->charge_current;

	decrease_currentmA = bq2416x_charger_errs * 150/*mA*/;
	if (decrease_currentmA) {
		if (currentmA > decrease_currentmA) {
			currentmA -= decrease_currentmA;
			if (currentmA <= 400) {
				currentmA = 550;
				use_half_charge_current = 1;
			} if (currentmA < 550)
				currentmA = 550;
		} else {
			currentmA = 550;
			use_half_charge_current = 1;
		}
	}

	/* Setup the input current */
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER

#ifndef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
#error "Please define CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY"
#endif

	input_currentmA = CONFIG_POWER_SUPPLY_AC_INPUT_CURRENTMA;
	if (curr_profile->input_current < input_currentmA)
		input_currentmA = curr_profile->input_current;

#ifdef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
	if (input_currentmA < 2500)
		supply_path = 0; /* USB */
	else
#endif
		supply_path = 1; /* AC IN */

#elif defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)

#ifndef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
#error "Please define CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY"
#endif

	if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB_DCP)
		input_currentmA = CONFIG_POWER_SUPPLY_USB_AC_INPUT_CURRENTMA;
	else
		input_currentmA = CONFIG_POWER_SUPPLY_USB_INPUT_CURRENTMA;
	if (curr_profile->input_current < input_currentmA)
		input_currentmA = curr_profile->input_current;

#ifdef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
	if (input_currentmA >= 2500)
		supply_path = 1; /* AC IN */
	else
#endif
		supply_path = 0; /* USB */

#endif

	if (supply_path) {
		/* AC IN */
		input_currentmA = bq2416x_set_in_current_limit(input_currentmA);
		(void)bq2416x_set_usb_current_limit(input_currentmA);
		bq2416x_select_supply(1);
	} else {
		/* USB */
		input_currentmA = bq2416x_set_usb_current_limit(input_currentmA);
		(void)bq2416x_set_in_current_limit(input_currentmA);
		bq2416x_select_supply(0);
	}

	/* Setup the charge current */
	currentmA = bq2416x_set_charger_current(currentmA);

	/* Setup the Half charge current */
	if (use_half_charge_current) {
		bq2416x_set_low_charge_current(1);
		currentmA = currentmA >> 1;
	} else
		bq2416x_set_low_charge_current(0);

	if (input_currentmA < currentmA)
		currentmA = input_currentmA;

	curr_active_currentmA = currentmA;
	curr_active_input_currentmA = input_currentmA;

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("setup current: %u/%u mA\n", curr_active_currentmA, curr_active_input_currentmA);
#endif

	return currentmA;
}

/*----------------------------------------------------------------------*/

static void bq2416x_stop_charger(void)
{
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
	const char* s = "<Support no BAT> ";
#else
	const char* s = "";
#endif /* !CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
	int current;

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	putc('\n');
	POWER_SUPPLY_INFO("%s() >>>>>>>>>>>>>>>>>>>>\n", __func__);
	bq2416x_dump_regs();
#endif

	current = power_supply_get_battery_current();

	bq2416x_enable_charge(0);

	if (bq2416x_has_no_main_battery) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
		if (power_supply_has_external_fuelgauge() && power_supply_has_main_battery() > 0) {
			int capacity = power_supply_get_battery_capacity();
			if (capacity >= 0) {
				if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
					POWER_SUPPLY_INFO("%sstop charger @ %d%% (%dmA) (no main battery?)\n", s, capacity, current);
				else
					POWER_SUPPLY_INFO("%sstop charger @ %d%% (no main battery?)\n", s, capacity);
			} else {
				if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
					POWER_SUPPLY_INFO("%sstop charger @ (%dmA) (no main battery?)\n", s, current);
				else
					POWER_SUPPLY_INFO("%sstop charger (no main battery?)\n", s);
			}
		} else
#endif
		{
			if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
				POWER_SUPPLY_INFO("%sstop charger @ (%dmA) (no main battery?)\n", s, current);
			else
				POWER_SUPPLY_INFO("%sstop charger (no main battery?)\n", s);
		}
	} else {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
		if (power_supply_has_external_fuelgauge()) {
			int capacity = power_supply_get_battery_capacity();
			if (capacity >= 0) {
				if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
					POWER_SUPPLY_INFO("%sstop charger @ %d%% (%dmA)\n", s, capacity, current);
				else
					POWER_SUPPLY_INFO("%sstop charger @ %d%%\n", s, capacity);
			} else {
				if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
					POWER_SUPPLY_INFO("%sstop charger @ (%dmA)\n", s, current);
				else
					POWER_SUPPLY_INFO("%sstop charger\n", s);
			}
		} else
#endif
		{
			if (current != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
				POWER_SUPPLY_INFO("%sstop charger @ (%dmA)\n", s, current);
			else
				POWER_SUPPLY_INFO("%sstop charger\n", s);
		}
	}

	mdelay(50);

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	POWER_SUPPLY_INFO("%s() <<<<<<<<<<<<<<<<<<<<\n\n", __func__);
#endif
}

static unsigned int bq2416x_start_charger(int vbat, unsigned int currentmA)
{
	u8 status_control, battery_supply_status;

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	putc('\n');
	POWER_SUPPLY_INFO("%s(%dmV, %umA) >>>>>>>>>>>>>>>>>>>>\n", __func__, vbat , currentmA);
	bq2416x_dump_regs();
#endif

	bq2416x_charge_currentmA = currentmA;

	bq2416x_reset_timer();
	bq2416x_config_safety_timer(SAFETY_TIMER_DISABLE);

	/* Stop charger */
	bq2416x_enable_charge(0);
	mdelay(128);

	if (vbat < 0 && vbat != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE)
		vbat = twl_charger_get_battery_voltage();

	/**
	 * Before writing to increase VBATREG register following a BATOVP event (e.g., IN or USB voltage is applied,
	 * IC remains in DEFAULT mode and then VBAT>3.6V is attached), toggle the HiZ bit or CD pin to clear the
	 * BATOVP fault.
	 */
#if !defined(CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY)
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT)
#error "Please define CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY for CONFIG_TWL6032_CHARGER_HAS_NO_INTERNAL_VBAT"
#endif
#if !defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) && defined(CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS)
#error "Please define CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY for CONFIG_TWL6032_CHARGER_VBAT_IS_VSYS"
#endif
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE)
	if (power_supply_has_external_fuelgauge())
#endif
		if (!bq2416x_has_no_main_battery && vbat > CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV) {
			POWER_SUPPLY_INFO("toggle HiZ\n");
			bq2416x_enable_HiZ(1);
		}
#elif defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE) /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
	if (power_supply_has_external_fuelgauge()) {
#if !defined(CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT)
		if (!bq2416x_has_no_main_battery && vbat > CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV) {
			POWER_SUPPLY_INFO("<Support no BAT> toggle HiZ\n");
			bq2416x_enable_HiZ(1);
		}
#endif
	}
#endif /* !CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

	/* try to read REG_STATUS_CONTROL to clear fault bits */
	status_control = bq2416x_get_status_control();
	/* try to read REG_BATT_SUPPLY_REGISTER to clear fault bits */
	battery_supply_status = bq2416x_get_battery_supply_status();
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	puts("bq2416x_start_charger >>>>>>>>>>>>>>>>>>>>>>>>\n");
	bq2416x_dump_status_control(status_control);
	bq2416x_dump_batt_supply(battery_supply_status);
	puts("bq2416x_start_charger <<<<<<<<<<<<<<<<<<<<<<<<\n");
#endif

	/* Setup the battery regulation voltage */
	bq2416x_set_regulation_voltage(CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV);

	/* Setup the termination current */
	bq2416x_set_eoc_current(CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA);

	/* Setup the charge current */
	curr_active_profile_idx = -1;
	curr_active_currentmA = 0;
	curr_profile_idx = bq2416x_get_charger_profile_index(vbat);
#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
	POWER_SUPPLY_DBG("VBAT %d mV; Profile %d (%d mV)\n",
		vbat, curr_profile_idx, charger_profiles[curr_profile_idx].battery_voltage);
#endif
	currentmA = bq2416x_setup_charge_current(curr_profile_idx, currentmA);

	/* Setup DPM voltages */
	bq2416x_set_in_vdpm(CONFIG_POWER_SUPPLY_AC_DPM_VOLTAGE);
	bq2416x_set_usb_vdpm(CONFIG_POWER_SUPPLY_USB_DPM_VOLTAGE);

	bq2416x_config_safety_timer(SAFETY_TIMER_DEFAULT);

	/* Enable EOC */
	bq2416x_set_eoc_enable(1);

	/* Enable charger & disable Hi-Z */
	bq2416x_enable_charge(1);

	if (bq2416x_has_no_main_battery) {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
		if (power_supply_has_external_fuelgauge() && power_supply_has_main_battery() > 0) {
			int capacity = power_supply_get_battery_capacity();
			if (capacity >= 0)
				POWER_SUPPLY_INFO("start charger (%u/%umA) @ %dmV; %d%% (no main battery?)\n",
						currentmA, curr_active_input_currentmA, vbat, capacity);
			else
				POWER_SUPPLY_INFO("start charger (%u/%umA) @ %dmV (no main battery?)\n",
						currentmA, curr_active_input_currentmA, vbat);
		} else
#endif
			POWER_SUPPLY_INFO("start charger (%u/%umA) @ VSYS %dmV (no main battery?)\n",
					currentmA, curr_active_input_currentmA, power_supply_get_sys_voltage());
	} else {
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
		if (power_supply_has_external_fuelgauge()) {
			int capacity = power_supply_get_battery_capacity();
			if (capacity >= 0)
				POWER_SUPPLY_INFO("start charger (%u/%umA) @ %dmV; %d%%\n",
						currentmA, curr_active_input_currentmA, vbat, capacity);
			else
				POWER_SUPPLY_INFO("start charger (%u/%umA) @ %dmV\n",
						currentmA, curr_active_input_currentmA, vbat);
		} else
#endif
			POWER_SUPPLY_INFO("start charger (%u/%umA) @ %dmV\n",
					currentmA, curr_active_input_currentmA, vbat);
	}

	twl_charger_backup_battery_setup();

	mdelay(250);

	bq2416x_reset_timer();

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	POWER_SUPPLY_INFO("%s(%dmV, %umA) <<<<<<<<<<<<<<<<<<<<\n\n", __func__, vbat , currentmA);
#endif

	return currentmA;
}

/*----------------------------------------------------------------------*/

static int bq2416x_is_charging(u8 *status_ctrl)
{
	int charging = POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING;
	u8 status_control, status;

	status_control = bq2416x_get_status_control();
	status = status_control & STATUS_MASK;
#if defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY) && defined(CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY)
	if (status == STATUS_CHARGING_FROM_IN || status == STATUS_CHARGING_FROM_USB)
#elif defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY)
	if (status == STATUS_CHARGING_FROM_IN)
#else
	if (status == STATUS_CHARGING_FROM_USB)
#endif
	{
		bq2416x_has_no_main_battery = 0;
		charging = POWER_SUPPLY_CHARGER_STATE_CHARGING;
	} else {
		if (status == STATUS_CHARGE_DONE) {
			bq2416x_has_no_main_battery = 0;
			charging = POWER_SUPPLY_CHARGER_STATE_EOC;
		} else if (status != STATUS_NO_VALID_SOURCE) {
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
			if (power_supply_has_external_fuelgauge() && power_supply_has_main_battery() > 0) {
				POWER_SUPPLY_PRINT("\n<bq2416x><Support no BAT><Not Charging> >>>>>>>>>>\n");
				bq2416x_dump_status_control(status_control);
				bq2416x_dump_regs();
				POWER_SUPPLY_PRINT("<bq2416x><Support no BAT><Not Charging> <<<<<<<<<<\n\n");
			} else
#endif
			if (!bq2416x_has_no_main_battery) {
				if (timeout_check(&bq2416x_dump_timeout, &bq2416x_dump_ticks)) {
					POWER_SUPPLY_PRINT("\n<bq2416x><Support no BAT><Not Charging> >>>>>>>>>>\n");
					bq2416x_dump_status_control(status_control);
					bq2416x_dump_regs();
					POWER_SUPPLY_PRINT("<bq2416x><Support no BAT><Not Charging> <<<<<<<<<<\n\n");

					bq2416x_dump_timeout = timeout_init((CONFIG_POWER_SUPPLY_POLL_MS_TIME<<2)-1000, &bq2416x_dump_ticks);
				}
			}
#else
			POWER_SUPPLY_PRINT("\n<bq2416x><Not Charging> >>>>>>>>>>>>>>>>>>>>>>>>\n");
			bq2416x_dump_status_control(status_control);
			bq2416x_dump_regs();
			POWER_SUPPLY_PRINT("<bq2416x><Not Charging> <<<<<<<<<<<<<<<<<<<<<<<<\n\n");
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
		}
	}

	if (status_ctrl)
		*status_ctrl = status_control;

	if ((status_control & FAULT_MASK) != 0) {
		if ((status_control & FAULT_MASK) == FAULT_BAT_TEMP) {
			/* clear fault bits */
			(void)bq2416x_get_status_control();
			(void)bq2416x_get_battery_supply_status();
			charging = POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT; /* Battery Temp Fault */
		} else if ((status_control & FAULT_MASK) == FAULT_BATTERY) {
			u8 battery_supply_status = bq2416x_get_battery_supply_status();
			if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_NOBAT) {
				bq2416x_has_no_main_battery = 1;
				/* clear fault bits */
				(void)bq2416x_get_status_control();
				(void)bq2416x_get_battery_supply_status();
				charging = POWER_SUPPLY_CHARGER_STATE_NO_BATTERY; /* no Battery */
			} else if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_OVP) {
				bq2416x_has_no_main_battery = 0;
				/* clear fault bits */
				(void)bq2416x_get_status_control();
				(void)bq2416x_get_battery_supply_status();
				charging = POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP; /* Battery OVP */
			}
		}
	}

	return charging;
}

static int bq2416x_fault_recover(u8 status_control)
{
	int recovered = 0;

	if (bq2416x_charge_currentmA) {
		u8 old_status_control = status_control;
		u8 battery_supply_status;
		u8 status, fault;

		status_control = bq2416x_get_status_control();
		battery_supply_status = bq2416x_get_battery_supply_status();

		putc('\n');
		POWER_SUPPLY_INFO("recover: STATUS_CTRL 0x%02x<->0x%02x; BATT_SUPPLY 0x%02x >>>>>>\n",
				old_status_control, status_control, battery_supply_status);
		bq2416x_dump_status_control(status_control);
		bq2416x_dump_batt_supply(battery_supply_status);

		status = status_control & STATUS_MASK;
		fault = status_control & FAULT_MASK;
		if (fault != 0) {
			old_status_control = status_control;

#if defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY) && defined(CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY)
			if (fault == FAULT_IN_SUPPLY || fault == FAULT_USB_SUPPLY)
#elif defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY)
			if (fault == FAULT_IN_SUPPLY)
#else
			if (fault == FAULT_USB_SUPPLY)
#endif
			{
				ulong start;

				if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_NOBAT) {
					bq2416x_has_no_main_battery = 1;
					goto _exit;
				}
#ifdef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
				if ((battery_supply_status & IN_STATUS_MASK) == IN_STATUS_OVP)
					goto _exit;
#endif
#ifdef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
				if ((battery_supply_status & USB_STATUS_MASK) == USB_STATUS_OVP)
					goto _exit;
#endif

				POWER_SUPPLY_INFO("wait for supply stable...\n");
				start = get_timer(0);
				do {
					mdelay(100);

					status_control = bq2416x_get_status_control();
					battery_supply_status = bq2416x_get_battery_supply_status();
					bq2416x_dump_status_control(status_control);
					bq2416x_dump_batt_supply(battery_supply_status);

					status = status_control & STATUS_MASK;
					if (status != STATUS_FAULT) {
						break;
					}
					fault = status_control & FAULT_MASK;
#if defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY) && defined(CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY)
					if (fault != FAULT_IN_SUPPLY && fault != FAULT_USB_SUPPLY)
#elif defined(CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY)
					if (fault != FAULT_IN_SUPPLY)
#else
					if (fault != FAULT_USB_SUPPLY)
#endif
					{
						break;
					}
				} while ((get_timer(0) - start) < 700/*ms*/);

				if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_NOBAT) {
					bq2416x_has_no_main_battery = 1;
					goto _exit;
				}
#ifdef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
				if ((battery_supply_status & IN_STATUS_MASK) == IN_STATUS_OVP)
					goto _exit;
#endif
#ifdef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
				if ((battery_supply_status & USB_STATUS_MASK) == USB_STATUS_OVP)
					goto _exit;
#endif
			} else if (fault == FAULT_BAT_TEMP) {
				goto _exit;
			} else if (fault == FAULT_BATTERY) {
				if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_NOBAT) {
					bq2416x_has_no_main_battery = 1;
					goto _exit;
				}
#if 0
			} else if (fault == FAULT_THERMAL || fault == FAULT_WATCHDOG || fault == FAULT_SAFETY_TIMER) {
				(void)power_supply_start_charger(-1);
				goto _out;
#endif
			}

			if (status != STATUS_NO_VALID_SOURCE && status != STATUS_CHARGE_DONE) {
				(void)power_supply_start_charger(-1);
				goto _out;
			}
		}

		old_status_control = status_control;

		if (status == STATUS_NO_VALID_SOURCE) {
			goto _exit;
		}
#ifdef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
		else if (status == STATUS_CHARGING_FROM_IN) {
			bq2416x_has_no_main_battery = 0;
			recovered = 1;
			POWER_SUPPLY_INFO("recovered: charging from AC IN\n");
			bq2416x_reset_timer();
			goto _exit;
		}
#endif
#ifdef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
		else if (status == STATUS_CHARGING_FROM_USB) {
			bq2416x_has_no_main_battery = 0;
			recovered = 1;
			POWER_SUPPLY_INFO("recovered: charging from USB\n");
			bq2416x_reset_timer();
			goto _exit;
		}
#endif
		else if (status == STATUS_CHARGE_DONE) {
			int vbat = twl_charger_get_battery_voltage();

			bq2416x_has_no_main_battery = 0;

			if (vbat != CONFIG_POWER_SUPPLY_INVALID_VOLTAGE && vbat < CONFIG_POWER_SUPPLY_MAX_EOC_BAT_VOLTAGEMV) {
				POWER_SUPPLY_INFO("VBAT %d mV, [EOC] restart charger\n", vbat);
				(void)power_supply_start_charger(vbat);
			} else {
				POWER_SUPPLY_INFO("<EOC> VBAT %d mV\n", vbat);
				bq2416x_reset_timer();
				goto _exit;
			}
		} else {
			(void)power_supply_start_charger(-1);
		}

_out:
		status_control = bq2416x_get_status_control();
		battery_supply_status = bq2416x_get_battery_supply_status();
		status = status_control & STATUS_MASK;
#ifdef CONFIG_CHARGER_BQ2416x_HAS_AC_IN_SUPPLY
		if (status == STATUS_CHARGING_FROM_IN) {
			bq2416x_has_no_main_battery = 0;
			recovered = 1;
			POWER_SUPPLY_INFO("recovered: charging from AC IN\n");
			bq2416x_reset_timer();
		} else
#endif
#ifdef CONFIG_CHARGER_BQ2416x_HAS_USB_SUPPLY
		if (status == STATUS_CHARGING_FROM_USB) {
			bq2416x_has_no_main_battery = 0;
			recovered = 1;
			POWER_SUPPLY_INFO("recovered: charging from USB\n");
			bq2416x_reset_timer();
		} else
#endif
		if (status == STATUS_CHARGE_DONE) {
			bq2416x_has_no_main_battery = 0;
			recovered = 1;
			POWER_SUPPLY_INFO("recovered: <EOC> VBAT %d mV\n", twl_charger_get_battery_voltage());
			bq2416x_reset_timer();
		} else {
			if ((battery_supply_status & BAT_STATUS_MASK) == BAT_STATUS_NOBAT) {
				bq2416x_has_no_main_battery = 1;
			}
		}

#ifdef CONFIG_CHARGER_BQ2416x_DEBUG
		bq2416x_dump_regs();
#else
		puts("----------------------------------------------\n");
		bq2416x_dump_status_control(status_control);
		bq2416x_dump_batt_supply(battery_supply_status);
#endif
_exit:
		POWER_SUPPLY_INFO("recover: STATUS_CTRL 0x%02x<->0x%02x; BATT_SUPPLY 0x%02x <<<<<<\n\n",
				old_status_control, status_control, battery_supply_status);
	}

	return recovered;
}

/*----------------------------------------------------------------------*/

static int bq2416x_charger_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
    switch (event) {
	case POWER_SUPPLY_CHARGER_INIT:
		/* Stop charger */
		bq2416x_enable_charge(0);
		/* Setup the battery regulation voltage */
		bq2416x_set_regulation_voltage(CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV);
		/* try to read REG_STATUS_CONTROL to clear fault bits */
		(void)bq2416x_get_status_control();
		/* try to read REG_BATT_SUPPLY_REGISTER to clear fault bits */
		(void)bq2416x_get_battery_supply_status();
		bq2416x_config_safety_timer(SAFETY_TIMER_DEFAULT);
		bq2416x_reset_timer();
		break;
    case POWER_SUPPLY_CHARGER_DUMP_REGS:
		bq2416x_dump_regs();
        break;
    case POWER_SUPPLY_CHARGER_START:
	{
		struct power_supply_charger_profile *profile = (struct power_supply_charger_profile *)data;
		profile->charge_current = bq2416x_start_charger(profile->battery_voltage, profile->input_current);
	}
		break;
    case POWER_SUPPLY_CHARGER_STOP:
		bq2416x_stop_charger();
		bq2416x_charge_currentmA = 0;
        break;
	case POWER_SUPPLY_CHARGER_RESET_TIMER:
		if (bq2416x_charge_currentmA) {
			int charging = 0;
			u8 status_control = 0;

			bq2416x_reset_timer();

			charging = bq2416x_is_charging(&status_control);
			if (charging == POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING
#ifndef CONFIG_CHARGER_BQ2416x_NO_EOC_RECOVERY
				|| charging == POWER_SUPPLY_CHARGER_STATE_EOC
#endif
				) {
				(void)bq2416x_fault_recover(status_control);
			}
		}
        break;
	case POWER_SUPPLY_CHARGER_IS_CHARGING:
	{
		int charging = 0;
		u8 status_control = 0;

		charging = bq2416x_is_charging(&status_control);
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
		if (charging == POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING
#ifndef CONFIG_CHARGER_BQ2416x_NO_EOC_RECOVERY
			|| charging == POWER_SUPPLY_CHARGER_STATE_EOC
#endif
			) {
			if (bq2416x_fault_recover(status_control))
				charging = bq2416x_is_charging(NULL);
		}
#else
		if (charging == POWER_SUPPLY_CHARGER_STATE_NOT_CHARGING
#ifndef CONFIG_CHARGER_BQ2416x_NO_EOC_RECOVERY
			|| charging == POWER_SUPPLY_CHARGER_STATE_EOC
#endif
			|| charging == POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP
			) {
			if (bq2416x_fault_recover(status_control))
				charging = bq2416x_is_charging(NULL);
		}
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

		*((int *) data) = charging;
	}
		break;
	case POWER_SUPPLY_CHARGER_IS_CHARGING_NO_RESTART:
	{
		*((int *) data) = bq2416x_is_charging(NULL);
	}
		break;
	case POWER_SUPPLY_CHARGER_UPDATE_PROFILE:
		bq2416x_update_charger_profile(*((int *)data));
		break;
	case POWER_SUPPLY_CHARGER_SET_CURRENT:
		if (bq2416x_charge_currentmA) {
			unsigned int prev_active_currentmA = curr_active_currentmA;
			(void)bq2416x_setup_charge_current(*((int *)data), bq2416x_charge_currentmA);
			if (prev_active_currentmA != curr_active_currentmA)
				mdelay(256);
		}
		break;
	case POWER_SUPPLY_CHARGER_HAS_MAIN_BATTERY:
	{
		int has_battery = -1;
		int charging;

		charging = bq2416x_is_charging(NULL);
#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY
		if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING
			|| charging == POWER_SUPPLY_CHARGER_STATE_EOC) {
			has_battery = 1;
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_NO_BATTERY) {
			has_battery = 0; /* no Main Battery */
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT) {
			has_battery = 0; /* Battery Temp Fault */
		}
#else
		if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING
			|| charging == POWER_SUPPLY_CHARGER_STATE_EOC) {
			has_battery = 1;
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_NO_BATTERY) {
			has_battery = 0; /* no Main Battery */
/* Don't check FAULT_BAT_TEMP. Otherwise it will shutdown immediately if there is no dummy TEMP on bq2416x. */
#if 0
		} else if (charging == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT) {
			has_battery = 0; /* Battery Temp Fault */
#endif
		}
#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */

		*((int *) data) = has_battery;
	}
		break;
	case POWER_SUPPLY_CHARGER_IS_BATTERY_LOCKED:
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE
	{
		int is_battery_locked = -1;
		int charging;

		charging = bq2416x_is_charging(NULL);
		if (bq2416x_has_no_main_battery) {
			if (power_supply_has_external_fuelgauge() && power_supply_has_main_battery() > 0)
				is_battery_locked = 1;
		} else {
			if (!power_supply_has_external_fuelgauge())
				is_battery_locked = 1;
		}
		*((int *) data) = is_battery_locked;
	}
#else
		*((int *) data) = -1;
#endif
		break;
	case POWER_SUPPLY_CHARGER_ADD_ERROR:
		if (bq2416x_charger_errs < 10) {
			++bq2416x_charger_errs;
			*((int *) data) = 1;
		} else {
			*((int *) data) = 0;
		}
		break;
	case POWER_SUPPLY_CHARGER_CLEAR_ERROR:
		if (bq2416x_charger_errs)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		bq2416x_charger_errs = 0;
		break;
    default:
		POWER_SUPPLY_ERR("unknown charger event %lu\n", event);
		break;
    }

#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	return 0;
#elif defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)
	return NOTIFY_STOP;
#endif
}

/*----------------------------------------------------------------------*/

void bq2416x_init(void)
{
	int ret;
	u8 revision = 0;

	POWER_SUPPLY_VDBG("%s\n", __func__);

	ret = bq2416x_read_byte(1, &revision, REG_PART_REVISION);
	if (ret) {
		mdelay(250);
		ret = bq2416x_read_byte(1, &revision, REG_PART_REVISION);
		if (ret) {
			POWER_SUPPLY_PRINT("power supply: no bq2416x\n");
			return;
		}
	}
	if ((revision & 0x40) == 0) {
		POWER_SUPPLY_ERR("unknown BQ chip 0x%02x\n", revision);
		return;
	}

#ifdef CONFIG_CHARGER_BQ2416x_VERBOSE_DEBUG
	bq2416x_dump_regs();
#endif

	ret = bq2416x_read_registers();
	if (ret) {
		POWER_SUPPLY_ERR("read registers err %d\n", ret);
		return;
	}

	bq_charger_nb.notifier_call = bq2416x_charger_notifier_call;
	bq_charger_nb.priority = INT_MAX;
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_EXT_AC_CHARGER, &bq_charger_nb);
	if (ret)
		POWER_SUPPLY_ERR("AC charger err %d\n", ret);
#elif defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER)
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_EXT_USB_CHARGER, &bq_charger_nb);
	if (ret)
		POWER_SUPPLY_ERR("USB charger err %d\n", ret);
#else
#error "Invalid Configuration for bq2416x\n"
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER, CONFIG_POWER_SUPPLY_HAS_EXTERNAL_USB_CHARGER */

	has_bq2416x = 1;

	POWER_SUPPLY_PRINT("power supply: found bq2416x rev %s\n",
			bq2416x_revision[(revision & 0x07)]);

/* Enable EOC for Battery detection */
#ifndef CONFIG_POWER_SUPPLY_ENABLE_CHARGE_TERMINATION
#error "Please define CONFIG_POWER_SUPPLY_ENABLE_CHARGE_TERMINATION"
#endif

#ifdef CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY

	POWER_SUPPLY_INFO("MINSYS: %s (EOC %umA; no BAT)\n\n",
			bq2416x_poweron_minimal_vsys ? "Yes" : "No",
			CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA);

#else

	POWER_SUPPLY_INFO("MINSYS: %s (EOC %umA)\n\n",
			bq2416x_poweron_minimal_vsys ? "Yes" : "No",
			CONFIG_POWER_SUPPLY_TERMINATION_CURRENTMA);

#endif /* CONFIG_POWER_SUPPLY_SUPPORT_NO_BATTERY */
}

void bq2416x_shutdown(void)
{
	if (has_bq2416x) {
		POWER_SUPPLY_VDBG("%s\n", __func__);

		if (bq2416x_charge_currentmA) {
			bq2416x_reset_timer();
			bq2416x_config_safety_timer(SAFETY_TIMER_DEFAULT);
		}
	}
}
