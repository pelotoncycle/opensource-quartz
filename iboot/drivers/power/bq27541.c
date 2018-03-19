/*
 * (C) Copyright 2013-2014
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

#define POWER_SUPPLY_SUBSYS_NAME	"bq27541"

/*----------------------------------------------------------------------*/

/*#define CONFIG_FUELGAUGE_BQ27541_DEBUG*/
/*#define CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG*/

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
#ifndef CONFIG_POWER_SUPPLY_DEBUG
#define CONFIG_POWER_SUPPLY_DEBUG
#endif
#ifndef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
#define CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
#endif
#endif

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <twl6030.h>

#include <icom/power_supply.h>

/*----------------------------------------------------------------------*/

#if (CONFIG_FUELGAUGE_BQ27541_MIN_VBAT < 0)
#error "Invalid CONFIG_FUELGAUGE_BQ27541_MIN_VBAT"
#endif
#if (CONFIG_FUELGAUGE_BQ27541_MAX_VBAT <= 4200)
#error "Invalid CONFIG_FUELGAUGE_BQ27541_MAX_VBAT"
#endif
#define CONFIG_FUELGAUGE_BQ27541_VBAT_RETRY_TIMES	5

/*----------------------------------------------------------------------*/

#define BQ27541_ADDR		0x55

/*----------------------------------------------------------------------*/

/* Commands */
#define BQ27541_CMD_CTRL	0x00	/* Control(): control specific features */
#define BQ27541_CMD_AR		0x02	/* AtRate(): discharge current value in mA */
#define BQ27541_CMD_UFSOC	0x04	/* UnfilteredSOC(): predicted remaining battery capacity as a percentage */
#define BQ27541_CMD_TEMP	0x06	/* Temperature(): battery temperature in units of 0.1K */
#define BQ27541_CMD_VOLT	0x08	/* Voltage(): measured cell-pack voltage in mV */
#define BQ27541_CMD_FLAGS	0x0a	/* Flags(): gas-gauge status register */
#define BQ27541_CMD_NAC		0x0c	/* NominalAvailableCapacity(): the uncompensated battery capacity remaining (mAh) */
#define BQ27541_CMD_FAC		0x0e	/* FullAvailableCapacity(): the uncompensated capacity of the battery when fully charged (mAh) */
#define BQ27541_CMD_RM		0x10	/* RemainingCapacity(): the compensated battery capacity remaining (mAh) */
#define BQ27541_CMD_FCC		0x12	/* FullChargeCapacity(): the compensated capacity of fully charged battery (mAh) */
#define BQ27541_CMD_AI		0x14	/* AverageCurrent(): the average current (updated every 1 sec) (mA) */
#define BQ27541_CMD_TTE		0x16	/* TimeToEmpty(): the predicted remaining battery life in minutes */
#define BQ27541_CMD_FFCC	0x18	/* FilteredFCC(): the filtered compensated capacity (mAh) */
#define BQ27541_CMD_SI		0x1a	/* StandbyCurrent(): the measured system standby current */
#define BQ27541_CMD_UFFCC	0x1c	/* UnfilteredFCC(): the compensated capacity of the battery (mAh) */
#define BQ27541_CMD_MLI		0x1e	/* MaxLoadCurrent(): the maximum load conditions of the system (mA) */
#define BQ27541_CMD_UFRM	0x20	/* UnfilteredRM(): the compensated battery capacity remaining (mAh) */
#define BQ27541_CMD_FRM		0x22	/* FilteredRM(): the filtered compensated battery capacity remaining (mAh) */
#define BQ27541_CMD_AP		0x24	/* AveragePower(): the average power of the current discharge (mW) */
#define BQ27541_CMD_INTTEMP	0x28	/* InternalTemperature(): the measured internal temperature in units of 0.1K */
#define BQ27541_CMD_CC		0x2a	/* CycleCount(): the number of cycles */
#define BQ27541_CMD_SOC		0x2c	/* StateOfCharge(): the predicted RemainingCapacity() expressed as a percentage of FullChargeCapacity() */
#define BQ27541_CMD_SOH		0x2e	/* StateOfHealth(): expressed as a percentage of the ratio of predicted FCC over the DesignCapacity() */
#define BQ27541_CMD_PCHG	0x34	/* PassedCharge(): the amount of charge passed through the sense resistor in mAh */
#define BQ27541_CMD_DOD0	0x36	/* DOD0(): the depth of discharge during the most recent OCV reading */
#define BQ27541_CMD_SDSG	0x38	/* SelfDischargeCurrent(): estimates the battery self discharge current */

/*----------------------------------------------------------------------*/

/* Control() Subcommands */
#define CTRL_SUB_CONTROL_STATUS	0x0000
#define CTRL_SUB_DEVICE_TYPE	0x0001
#define CTRL_SUB_FW_VERSION		0x0002
#define CTRL_SUB_HW_VERSION		0x0003
#define CTRL_SUB_DF_VERSION		0x000C
#define CTRL_SUB_CLEAR_HIBERNATE	0x0012
#define CTRL_SUB_CLEAR_SHUTDOWN	0x0014
#define CTRL_SUB_IT_ENABLE		0x0021

/* Flags() */
#define BQ27541_FLAG_OTC	(1 << 15)	/* Over-Temperature in Charge condition is detected */
#define BQ27541_FLAG_OTD	(1 << 14)	/* Over-Temperature in Discharge condition is detected */
#define BQ27541_FLAG_BATHI	(1 << 13)	/* Battery High */
#define BQ27541_FLAG_BATLOW	(1 << 12)	/* Battery Low */
#define BQ27541_FLAG_CHG_INH	(1 << 11)	/* Charge Inhibit indicates the temperature is outside the range */
#define BQ27541_FLAG_FC	(1 << 9)	/* Full-charged is detected */
#define BQ27541_FLAG_CHG	(1 << 8)	/* (Fast) charging allowed */
#define BQ27541_FLAG_OCVTAKEN	(1 << 7)	/* OCV measurement is performed in relax */
#define BQ27541_FLAG_ISD	(1 << 6)	/* Internal Short is detected */
#define BQ27541_FLAG_TDD	(1 << 5)	/* Tab Disconnect is detected */
#define BQ27541_FLAG_SOC1	(1 << 2)	/* State-of-Charge-Threshold 1 reached */
#define BQ27541_FLAG_SOCF	(1 << 1)	/* State-of-Charge-Threshold Final reached */
#define BQ27541_FLAG_DSG	(1 << 0)	/* Discharging detected */

/*----------------------------------------------------------------------*/

#define KELVIN2CELSIUS(k)	((int)(k) - 2732)

/*----------------------------------------------------------------------*/

static int has_bq27541 __attribute__ ((section (".data"))) = 0;
static struct notifier_block bq_fg_nb __attribute__ ((section (".data")));

static int bq27541_vbat __attribute__ ((section (".data"))) = -1;

static u16 control_status, device_type, fw_version, hw_version, df_version;

/*----------------------------------------------------------------------*/

static int bq27541_get_control_status(void);

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void bq27541_save_i2c_bus(void) __attribute__((always_inline));
static inline void bq27541_save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_FUELGAUGE_BQ27541_I2C_BUS)
		i2c_set_bus_num(CONFIG_FUELGAUGE_BQ27541_I2C_BUS);
}

static inline void bq27541_restore_i2c_bus(void) __attribute__((always_inline));
static inline void bq27541_restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_FUELGAUGE_BQ27541_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define bq27541_save_i2c_bus()
#define bq27541_restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static void bq27541_remove(void)
{
	if (has_bq27541) {
		int ret = power_supply_unregister_notifier(POWER_SUPPLY_FEAT_EXT_FUELGAUGE, &bq_fg_nb);
		if (ret) {
			POWER_SUPPLY_ERR("unregister err %d\n\n", ret);
		} else {
			POWER_SUPPLY_PRINT("\npower supply: bq27541 unregistered\n\n");
		}
	}
}

/*----------------------------------------------------------------------*/

static int bq27541_write_subcommand(int retries, u16 subcmd, u8 reg)
{
	int ret;
	u8 buf[2];

	buf[0] = subcmd & 0xff;
	buf[1] = (subcmd >> 8) & 0xff;

	bq27541_save_i2c_bus();

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("write_subcmd(0x%02X): [0x%02X]=0x%04X...\n", BQ27541_ADDR, reg, subcmd);
#endif

_retry:
	ret = i2c_write(BQ27541_ADDR, reg, 1, buf, 2);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: write_subcmd(0x%02X): [0x%02X]=0x%04X err %d\n", retries, BQ27541_ADDR, reg, subcmd, ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("write_subcmd(0x%02X): [0x%02X]=0x%04X err %d\n", BQ27541_ADDR, reg, subcmd, ret);
#endif
		}
	} else {
		/* Wait for bq27541 ready to response */
		mdelay(10);
	}

	bq27541_restore_i2c_bus();

	return ret;
}

static int bq27541_read_u16(int retries, u16 *value, u8 reg)
{
	int ret;
	u8 buf[2];

	buf[0] = buf[1] = 0;
	*value = 0;

	bq27541_save_i2c_bus();

_retry:
	ret = i2c_read(BQ27541_ADDR, reg, 1, buf, 2);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: read(0x%02X): [0x%02X] err %d\n", retries, BQ27541_ADDR, reg, ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("read(0x%02X): [0x%02X] err %d\n", BQ27541_ADDR, reg, ret);
#endif
		}
	} else {
		*value = ((u16)(buf[1] << 8)) | ((u16)buf[0]);
#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
		POWER_SUPPLY_DBG("read(0x%02X): [0x%02X]=0x%04X\n", BQ27541_ADDR, reg, *value);
#endif
	}

	bq27541_restore_i2c_bus();

	return ret;
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_FUELGAUGE_BQ27541_ENABLE_IT
static int bq27541_enable_impedance_track(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_IT_ENABLE, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write IT ENABLE cmd err %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

/*----------------------------------------------------------------------*/

#ifdef CONFIG_FUELGAUGE_BQ27541_CLEAR_HIBERNATE
static int bq27541_clear_hibernate(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_CLEAR_HIBERNATE, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write CLEAR HIBERNATE cmd err %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

/*----------------------------------------------------------------------*/

static int bq27541_get_capacity(void)
{
	u16 capacity = 0;
	int ret;

	ret = bq27541_read_u16(3, &capacity, BQ27541_CMD_SOC);
	if (ret)
		POWER_SUPPLY_ERR("get CAPACITY err %d\n", ret);

	if (capacity > 100) {
		POWER_SUPPLY_ERR("invalid capacity %d %%\n", capacity);
	}

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("capacity %d %%\n", capacity);
#endif
	return capacity;
}

static int bq27541_get_voltage(void)
{
	u16 volt;
	int count = 0;
	int debounce = CONFIG_FUELGAUGE_BQ27541_VBAT_RETRY_TIMES;
	int ret;

_get_volt:
	volt = 0;
	ret = bq27541_read_u16(3, &volt, BQ27541_CMD_VOLT);
	if (ret)
		POWER_SUPPLY_ERR("get VOLT err %d\n", ret);

	if (volt < CONFIG_FUELGAUGE_BQ27541_MIN_VBAT || volt > CONFIG_FUELGAUGE_BQ27541_MAX_VBAT) {
		if (count < CONFIG_FUELGAUGE_BQ27541_VBAT_RETRY_TIMES) {
			count++;
			POWER_SUPPLY_ERR("[%d] invalid VBAT %d mV...retry...\n", count, volt);
			mdelay(200);
			goto _get_volt;
		} else {
			bq27541_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
		}
	}

	if (volt >= CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV) {
		int capacity = bq27541_get_capacity();
		if (capacity < 50 || capacity > 100) {
			if (count < CONFIG_FUELGAUGE_BQ27541_VBAT_RETRY_TIMES) {
				count++;
				POWER_SUPPLY_ERR("Capacity %d%%; invalid VBAT %dmV\n", capacity, volt);
				mdelay(200);
				goto _get_volt;
			} else {
				bq27541_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
				return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			}
		}
	}

	if (volt >= CONFIG_POWER_SUPPLY_POWERON_VOLTAGE
			&& bq27541_vbat <= CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV && bq27541_vbat >= 0) {
		if (debounce > 0) {
			debounce--;
			POWER_SUPPLY_ERR("[%d] bad VBAT %d mV (prev %d mV)...retry...\n", debounce, volt, bq27541_vbat);
			volt = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			mdelay(200);
			goto _get_volt;
		} else {
			bq27541_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
		}
	}

	bq27541_vbat = volt;

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("vbat %d mV\n", volt);
#endif
	return volt;
}

static int bq27541_get_current(void)
{
	s16 curr = 0;
	int ret;

	ret = bq27541_read_u16(3, (u16*)&curr, BQ27541_CMD_AI);
	if (ret)
		POWER_SUPPLY_ERR("get CURRENT err %d\n", ret);

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("current %d mA\n", curr);
#endif
	return curr;
}

static int bq27541_get_temperature(void)
{
	int temperature = 0;
	u16 temp = 0;
	int ret;

	/* battery temperature in units of 0.1K */
	ret = bq27541_read_u16(3, &temp, BQ27541_CMD_TEMP);
	if (ret) {
		POWER_SUPPLY_ERR("get TEMP err %d\n", ret);
	} else {
		/* °C = K − 273.15 */
		/* battery temperature in units of 0.1C */
		temperature = KELVIN2CELSIUS(temp);
	}

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("temperature %u (0.1K); %d (0.1C)\n", temp, temperature);
#endif
	return temperature;
}

/*----------------------------------------------------------------------*/

static void print_temperature(const char *prefix, int temperature, const char *postfix)
{
	int decimal_integer, decimal_fraction;

	decimal_integer = temperature / 10;
	decimal_fraction = temperature - decimal_integer * 10;

	if (prefix)
		printf("%s", prefix);
	printf("%d.%d (degree C)", decimal_integer, decimal_fraction);
	if (postfix)
		printf("%s", postfix);
}

/*----------------------------------------------------------------------*/

static int bq27541_fuelgauge_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int ret;

	if (!has_bq27541)
		return 0;

	ret = NOTIFY_DONE;
	switch (event) {
	case POWER_SUPPLY_FG_GET_VBAT: /* int;mV */
#ifdef CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT
#else
	{
		int vbat = bq27541_get_voltage();
		*((int *) data) = vbat;
		if (vbat == CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			bq27541_remove();
		}
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
	}
#endif /* CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT */
		break;
	case POWER_SUPPLY_FG_GET_CURRENT: /* int;mA */
		*((int *) data) = bq27541_get_current();
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_GET_CAPACITY: /* int;percentage */ {
		int capacity = bq27541_get_capacity();
		if (capacity > 100)
			capacity = 100;
		*((int *) data) = capacity;
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
	}	break;
	case POWER_SUPPLY_FG_IS_FULL_CHARGED:
		if (bq27541_get_capacity() >= 100)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_HAS_MAIN_BATTERY:
		/* bq27541 is in Battery Pack */
		*((int *) data) = 1;
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_INIT:
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_GET_TEMP: /* int: battery temperature in units of 0.1C */
		*((int *) data) = bq27541_get_temperature();
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_GET_VSYS: /* int;mV */
	case POWER_SUPPLY_FG_GET_VBBAT: /* int;mV */
	case POWER_SUPPLY_FG_GET_VUSB_BUS: /* int;mV */
	case POWER_SUPPLY_FG_GET_VAC: /* int;mV */
		break;
	default:
		POWER_SUPPLY_ERR("unknown FG event %lu\n", event);
		break;
	}

	return ret;
}

/*----------------------------------------------------------------------*/

static int bq27541_get_control_status(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_CONTROL_STATUS, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write CONTROL STATUS cmd err %d\n", ret);
		return ret;
	}

	control_status = 0;
	ret = bq27541_read_u16(2, &control_status, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get CONTROL STATUS err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27541_get_device_type(void)
{
	int ret;

	ret = bq27541_write_subcommand(1, CTRL_SUB_DEVICE_TYPE, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write DEVICE TYPE cmd err %d\n", ret);
		return ret;
	}

	device_type = 0;
	ret = bq27541_read_u16(1, &device_type, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get DEVICE TYPE err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27541_get_fw_version(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_FW_VERSION, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write FW VERSION cmd err %d\n", ret);
		return ret;
	}

	fw_version = 0;
	ret = bq27541_read_u16(2, &fw_version, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get FW VERSION err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27541_get_hw_version(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_HW_VERSION, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write HW VERSION cmd err %d\n", ret);
		return ret;
	}

	hw_version = 0;
	ret = bq27541_read_u16(2, &hw_version, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get FW VERSION err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27541_get_df_version(void)
{
	int ret;

	ret = bq27541_write_subcommand(2, CTRL_SUB_DF_VERSION, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write DF VERSION cmd err %d\n", ret);
		return ret;
	}

	df_version = 0;
	ret = bq27541_read_u16(2, &df_version, BQ27541_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get DF VERSION err %d\n", ret);
		return ret;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int bq27541_dump_info(void)
{
	int ret;
	u16 flags = 0, health = 0, cycles = 0;
	u16 vbat = 0, capacity = 0, temp = 0;
	s16 curr = 0;

	putc('\n');

#ifdef CONFIG_FUELGAUGE_BQ27541_CLEAR_HIBERNATE
	ret = bq27541_clear_hibernate();
	if (ret) {
		POWER_SUPPLY_ERR("CLEAR HIBERNATE err %d\n", ret);
		goto _out;
	}
#endif

	ret = bq27541_get_hw_version();
	if (ret) {
		POWER_SUPPLY_ERR("get HW VERSION err %d\n", ret);
		goto _out;
	}
	ret = bq27541_get_fw_version();
	if (ret) {
		POWER_SUPPLY_ERR("get FW VERSION err %d\n", ret);
		goto _out;
	}
	ret = bq27541_get_df_version();
	if (ret) {
		POWER_SUPPLY_ERR("get DF VERSION err %d\n", ret);
		goto _out;
	}

#ifdef CONFIG_FUELGAUGE_BQ27541_ENABLE_IT
	ret = bq27541_enable_impedance_track();
	if (ret) {
		POWER_SUPPLY_ERR("IT ENABLE err %d\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27541: enable IT\n");
#endif

	ret = bq27541_get_control_status();
	if (ret) {
		POWER_SUPPLY_ERR("get CTRL STATUS err %d\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27541: HW 0x%04x, FW 0x%04x, DF 0x%04x, STATUS 0x%04x\n",
			hw_version, fw_version, df_version, control_status);

	ret = bq27541_read_u16(2, &flags, BQ27541_CMD_FLAGS);
	if (ret) {
		POWER_SUPPLY_ERR("get FLAGS err %d\n\n", ret);
		goto _out;
	}
	ret = bq27541_read_u16(2, &health, BQ27541_CMD_SOH);
	if (ret) {
		POWER_SUPPLY_ERR("get SOH err %d\n\n", ret);
		goto _out;
	}
	ret = bq27541_read_u16(2, &cycles, BQ27541_CMD_CC);
	if (ret) {
		POWER_SUPPLY_ERR("get CC err %d\n\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27541: Flags 0x%04x, Health %d%%, Cycles %d\n",
			flags, health, cycles);

	ret = bq27541_read_u16(2, &capacity, BQ27541_CMD_SOC);
	if (ret) {
		POWER_SUPPLY_ERR("get SOC err %d\n\n", ret);
		goto _out;
	}
	ret = bq27541_read_u16(2, &vbat, BQ27541_CMD_VOLT);
	if (ret) {
		POWER_SUPPLY_ERR("get VOLT err %d\n\n", ret);
		goto _out;
	}
	ret = bq27541_read_u16(2, (u16*)&curr, BQ27541_CMD_AI);
	if (ret) {
		POWER_SUPPLY_ERR("get AI err %d\n\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27541: Current %dmA, Capacity %d%%, VBAT %dmV\n",
			curr, capacity, vbat);

	/* battery temperature in units of 0.1K */
	ret = bq27541_read_u16(2, &temp, BQ27541_CMD_TEMP);
	if (ret) {
		POWER_SUPPLY_ERR("get TEMP err %d\n\n", ret);
		goto _out;
	}
	/* °C = K − 273.15 */
	/* battery temperature in units of 0.1C */
#if 0
	POWER_SUPPLY_PRINT("bq27541: BAT Temp %d (0.1C)\n",
			(((int)temp) - 2732));
#else
	print_temperature("bq27541: BAT Temp ", KELVIN2CELSIUS(temp), "\n");
#endif

	if (capacity > 100) {
		POWER_SUPPLY_ERR("invalid Capacity %d%%\n\n", capacity);
		ret = -EAGAIN;
		goto _out;
	}
	if (vbat < CONFIG_FUELGAUGE_BQ27541_MIN_VBAT || vbat > CONFIG_FUELGAUGE_BQ27541_MAX_VBAT
			|| (capacity <= 50 && vbat >= CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV)) {
		POWER_SUPPLY_ERR("invalid VBAT %dmV\n\n", vbat);
		ret = -EAGAIN;
		goto _out;
	}

	putc('\n');

_out:
	return ret;
}

/*----------------------------------------------------------------------*/

void bq27541_init(void)
{
	int ret;

	ret = bq27541_get_device_type();
	if (ret) {
		mdelay(100);
		ret = bq27541_get_device_type();
		if (ret) {
			POWER_SUPPLY_PRINT("power supply: no bq27541\n");
			return;
		}
	}
	if (device_type != 0x0541) {
		POWER_SUPPLY_ERR("unknown BQ27541 chip 0x%04X\n", device_type);
		return;
	}
	POWER_SUPPLY_VDBG("found bq27541\n");
	ret = bq27541_dump_info();
	if (ret) {
		return;
	}

	bq_fg_nb.notifier_call = bq27541_fuelgauge_notifier_call;
	bq_fg_nb.priority = INT_MAX;
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_EXT_FUELGAUGE, &bq_fg_nb);
	if (ret)
		POWER_SUPPLY_ERR("FuelGauge err %d\n", ret);

	has_bq27541 = 1;
}

void bq27541_shutdown(void)
{
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_FUELGAUGE_BQ27541_DEBUG
/* bq27541_dataflash */
static int do_bq27541_dataflash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;

	if (argc < 3) {
		puts("usage: bq27541_dataflash <class> <block>\n");
		return 0;
	}

	return 0;
}

U_BOOT_CMD(
	bq27541_dataflash, 3, 0, do_bq27541_dataflash,
	"bq27541 dataflash",
	""
);
#endif /* CONFIG_FUELGAUGE_BQ27541_DEBUG */

#ifdef CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG
#include <linux/usb/otg.h>

/* bq27541_test */
static int do_bq27541_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int vbat;
	int capacity;
	int curr;

	while (1) {
		vbat = bq27541_get_voltage();

		if (vbat <= CONFIG_FUELGAUGE_BQ27541_MIN_VBAT || vbat >= CONFIG_FUELGAUGE_BQ27541_MAX_VBAT) {
			printf("** bq27541: invalid VBAT %dmV\n", vbat);
		} else {
			printf("bq27541: VBAT %dmV\n", vbat);
		}

		capacity = bq27541_get_capacity();
		if (capacity == 0) {
			printf("** bq27541: CAPACITY %d%%\n", capacity);
		} else if (capacity > 100) {
			printf("** bq27541: invalid CAPACITY %d%%\n", capacity);
		} else {
			printf("bq27541: CAPACITY %d%%\n", capacity);
		}

		curr = bq27541_get_current();
		printf("bq27541: CURRENT %dmA\n", curr);

#if defined(CONFIG_USB_OTG_TRANSCEIVER)
		otg_handle_interrupts(0);
#endif /* CONFIG_USB_OTG_TRANSCEIVER */

		mdelay(1000);
		UART_FIFO_FLUSH_ONCE();

		putc('\n');
	}

	return 0;
}

U_BOOT_CMD(
	bq27541_test, 1, 0, do_bq27541_test,
	"bq27541 test",
	""
);
#endif /* CONFIG_FUELGAUGE_BQ27541_VERBOSE_DEBUG */
