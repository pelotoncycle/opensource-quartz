/*
 * (C) Copyright 2014
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

#define POWER_SUPPLY_SUBSYS_NAME	"bq27520"

/*----------------------------------------------------------------------*/

/*#define CONFIG_FUELGAUGE_BQ27520_DEBUG*/
/*#define CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG*/

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
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

#if (CONFIG_FUELGAUGE_BQ27520_MIN_VBAT < 0)
#error "Invalid CONFIG_FUELGAUGE_BQ27520_MIN_VBAT"
#endif
#if (CONFIG_FUELGAUGE_BQ27520_MAX_VBAT <= 4200)
#error "Invalid CONFIG_FUELGAUGE_BQ27520_MAX_VBAT"
#endif
#define CONFIG_FUELGAUGE_BQ27520_VBAT_RETRY_TIMES	5

/*----------------------------------------------------------------------*/

#define BQ27520_ADDR		0x55

/*----------------------------------------------------------------------*/

/* G3 Commands */
#define BQ27520_G3_CMD_CTRL	0x00
#define BQ27520_G3_CMD_TEMP	0x06
#define BQ27520_G3_CMD_VOLT	0x08
#define BQ27520_G3_CMD_FLAGS	0x0a
#define BQ27520_G3_CMD_AI		0x14
#define BQ27520_G3_CMD_SOH		0x28
#define BQ27520_G3_CMD_CC		0x2a
#define BQ27520_G3_CMD_SOC		0x2c

/* G4 Commands */
#define BQ27520_G4_CMD_CTRL	0x00
#define BQ27520_G4_CMD_TEMP	0x06
#define BQ27520_G4_CMD_VOLT	0x08
#define BQ27520_G4_CMD_FLAGS	0x0a
#define BQ27520_G4_CMD_AI		0x14
#define BQ27520_G4_CMD_SOH		0x1c
#define BQ27520_G4_CMD_CC		0x1e
#define BQ27520_G4_CMD_SOC		0x20

/*----------------------------------------------------------------------*/

/* Control() Subcommands */
#define CTRL_SUB_CONTROL_STATUS	0x0000
#define CTRL_SUB_DEVICE_TYPE	0x0001
#define CTRL_SUB_FW_VERSION		0x0002
#define CTRL_SUB_CLEAR_HIBERNATE	0x0012
#define CTRL_SUB_DF_VERSION		0x001F
#define CTRL_SUB_IT_ENABLE		0x0021

/*----------------------------------------------------------------------*/

enum {
	BQ27520_CMD_CTRL = 0,
	BQ27520_CMD_TEMP,
	BQ27520_CMD_VOLT,
	BQ27520_CMD_FLAGS,
	BQ27520_CMD_AI,
	BQ27520_CMD_SOH,
	BQ27520_CMD_CC,
	BQ27520_CMD_SOC,
};

/*----------------------------------------------------------------------*/

#define KELVIN2CELSIUS(k)	((int)(k) - 2732)

/*----------------------------------------------------------------------*/

static u8 bq27520_g3_regs[] = {
	BQ27520_G3_CMD_CTRL,
	BQ27520_G3_CMD_TEMP,
	BQ27520_G3_CMD_VOLT,
	BQ27520_G3_CMD_FLAGS,
	BQ27520_G3_CMD_AI,
	BQ27520_G3_CMD_SOH,
	BQ27520_G3_CMD_CC,
	BQ27520_G3_CMD_SOC,
};

static u8 bq27520_g4_regs[] = {
	BQ27520_G4_CMD_CTRL,
	BQ27520_G4_CMD_TEMP,
	BQ27520_G4_CMD_VOLT,
	BQ27520_G4_CMD_FLAGS,
	BQ27520_G4_CMD_AI,
	BQ27520_G4_CMD_SOH,
	BQ27520_G4_CMD_CC,
	BQ27520_G4_CMD_SOC,
};

static u8 *bq27520_regs __attribute__ ((section (".data"))) = bq27520_g3_regs;

/*----------------------------------------------------------------------*/

static int has_bq27520 __attribute__ ((section (".data"))) = 0;
static struct notifier_block bq_fg_nb __attribute__ ((section (".data")));

static int bq27520_vbat __attribute__ ((section (".data"))) = -1;

static u16 control_status, device_type, fw_version, df_version;

/*----------------------------------------------------------------------*/

static int bq27520_get_control_status(void);

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void bq27520_save_i2c_bus(void) __attribute__((always_inline));
static inline void bq27520_save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_FUELGAUGE_BQ27520_I2C_BUS)
		i2c_set_bus_num(CONFIG_FUELGAUGE_BQ27520_I2C_BUS);
}

static inline void bq27520_restore_i2c_bus(void) __attribute__((always_inline));
static inline void bq27520_restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_FUELGAUGE_BQ27520_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define bq27520_save_i2c_bus()
#define bq27520_restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static void bq27520_remove(void)
{
	if (has_bq27520) {
		int ret = power_supply_unregister_notifier(POWER_SUPPLY_FEAT_EXT_FUELGAUGE, &bq_fg_nb);
		if (ret) {
			POWER_SUPPLY_ERR("unregister err %d\n\n", ret);
		} else {
			POWER_SUPPLY_PRINT("\npower supply: bq27520 unregistered\n\n");
		}
	}
}

/*----------------------------------------------------------------------*/

static int bq27520_write_subcommand(int retries, u16 subcmd, u8 reg)
{
	int ret;
	u8 buf[2];

	buf[0] = subcmd & 0xff;
	buf[1] = (subcmd >> 8) & 0xff;

	bq27520_save_i2c_bus();

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("write_subcmd(0x%02X): [0x%02X]=0x%04X...\n", BQ27520_ADDR,
			bq27520_regs[reg], subcmd);
#endif

_retry:
	ret = i2c_write(BQ27520_ADDR, bq27520_regs[reg], 1, buf, 2);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: write_subcmd(0x%02X): [0x%02X]=0x%04X err %d\n", retries, BQ27520_ADDR,
					bq27520_regs[reg], subcmd, ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("write_subcmd(0x%02X): [0x%02X]=0x%04X err %d\n", BQ27520_ADDR,
					bq27520_regs[reg], subcmd, ret);
#endif
		}
	} else {
		/* Wait for bq27520 ready to response */
		mdelay(10);
	}

	bq27520_restore_i2c_bus();

	return ret;
}

static int bq27520_read_u16(int retries, u16 *value, u8 reg)
{
	int ret;
	u8 buf[2];

	buf[0] = buf[1] = 0;
	*value = 0;

	bq27520_save_i2c_bus();

_retry:
	ret = i2c_read(BQ27520_ADDR, bq27520_regs[reg], 1, buf, 2);
	if (ret) {
		if (--retries > 0) {
#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("%d: read(0x%02X): [0x%02X] err %d\n", retries, BQ27520_ADDR,
					bq27520_regs[reg], ret);
#endif
			mdelay(10);
			goto _retry;
		} else {
#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
			POWER_SUPPLY_DBG("read(0x%02X): [0x%02X] err %d\n", BQ27520_ADDR,
					bq27520_regs[reg], ret);
#endif
		}
	} else {
		*value = ((u16)(buf[1] << 8)) | ((u16)buf[0]);
#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
		POWER_SUPPLY_DBG("read(0x%02X): [0x%02X]=0x%04X\n", BQ27520_ADDR,
				bq27520_regs[reg], *value);
#endif
	}

	bq27520_restore_i2c_bus();

	return ret;
}

/*----------------------------------------------------------------------*/

static int bq27520_enable_impedance_track(void)
{
	int ret;

	ret = bq27520_write_subcommand(2, CTRL_SUB_IT_ENABLE, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write IT ENABLE cmd err %d\n", ret);
		return ret;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int bq27520_clear_hibernate(void)
{
	int ret;

	ret = bq27520_write_subcommand(2, CTRL_SUB_CLEAR_HIBERNATE, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write CLEAR HIBERNATE cmd err %d\n", ret);
		return ret;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int bq27520_get_capacity(void)
{
	u16 capacity = 0;
	int ret;

	ret = bq27520_read_u16(3, &capacity, BQ27520_CMD_SOC);
	if (ret)
		POWER_SUPPLY_ERR("get CAPACITY err %d\n", ret);

	if (capacity > 100) {
		POWER_SUPPLY_ERR("invalid capacity %d %%\n", capacity);
	}

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("capacity %d %%\n", capacity);
#endif
	return capacity;
}

static int bq27520_get_voltage(void)
{
	u16 volt;
	int count = 0;
	int debounce = CONFIG_FUELGAUGE_BQ27520_VBAT_RETRY_TIMES;
	int ret;

_get_volt:
	volt = 0;
	ret = bq27520_read_u16(3, &volt, BQ27520_CMD_VOLT);
	if (ret)
		POWER_SUPPLY_ERR("get VOLT err %d\n", ret);

	if (volt < CONFIG_FUELGAUGE_BQ27520_MIN_VBAT || volt > CONFIG_FUELGAUGE_BQ27520_MAX_VBAT) {
		if (count < CONFIG_FUELGAUGE_BQ27520_VBAT_RETRY_TIMES) {
			count++;
			POWER_SUPPLY_ERR("[%d] invalid VBAT %d mV...retry...\n", count, volt);
			mdelay(200);
			goto _get_volt;
		} else {
			bq27520_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
		}
	}

	if (volt >= CONFIG_POWER_SUPPLY_MAX_BAT_VOLTAGEMV) {
		int capacity = bq27520_get_capacity();
		if (capacity < 50 || capacity > 100) {
			if (count < CONFIG_FUELGAUGE_BQ27520_VBAT_RETRY_TIMES) {
				count++;
				POWER_SUPPLY_ERR("Capacity %d%%; invalid VBAT %dmV\n", capacity, volt);
				mdelay(200);
				goto _get_volt;
			} else {
				bq27520_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
				return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			}
		}
	}

	if (volt >= CONFIG_POWER_SUPPLY_POWERON_VOLTAGE
			&& bq27520_vbat <= CONFIG_POWER_SUPPLY_MIN_PRE_CHARGE_VOLTAGEMV && bq27520_vbat >= 0) {
		if (debounce > 0) {
			debounce--;
			POWER_SUPPLY_ERR("[%d] bad VBAT %d mV (prev %d mV)...retry...\n", debounce, volt, bq27520_vbat);
			volt = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			mdelay(200);
			goto _get_volt;
		} else {
			bq27520_vbat = CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
			return CONFIG_POWER_SUPPLY_INVALID_VOLTAGE;
		}
	}

	bq27520_vbat = volt;

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("vbat %d mV\n", volt);
#endif
	return volt;
}

static int bq27520_get_current(void)
{
	s16 curr = 0;
	int ret;

	ret = bq27520_read_u16(3, (u16*)&curr, BQ27520_CMD_AI);
	if (ret)
		POWER_SUPPLY_ERR("get CURRENT err %d\n", ret);

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
	POWER_SUPPLY_DBG("current %d mA\n", curr);
#endif
	return curr;
}

static int bq27520_get_temperature(void)
{
	int temperature = 0;
	u16 temp = 0;
	int ret;

	/* battery temperature in units of 0.1K */
	ret = bq27520_read_u16(3, &temp, BQ27520_CMD_TEMP);
	if (ret) {
		POWER_SUPPLY_ERR("get TEMP err %d\n", ret);
	} else {
		/* °C = K − 273.15 */
		/* battery temperature in units of 0.1C */
		temperature = KELVIN2CELSIUS(temp);
	}

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
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

static int bq27520_fuelgauge_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int ret;

	if (!has_bq27520)
		return 0;

	ret = NOTIFY_DONE;
	switch (event) {
	case POWER_SUPPLY_FG_GET_VBAT: /* int;mV */
#ifdef CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT
#else
	{
		int vbat = bq27520_get_voltage();
		*((int *) data) = vbat;
		if (vbat == CONFIG_POWER_SUPPLY_INVALID_VOLTAGE) {
			bq27520_remove();
		}
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
	}
#endif /* CONFIG_POWER_SUPPLY_USE_INTERNAL_FUELGAUGE_VBAT */
		break;
	case POWER_SUPPLY_FG_GET_CURRENT: /* int;mA */
		*((int *) data) = bq27520_get_current();
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_GET_CAPACITY: /* int;percentage */ {
		int capacity = bq27520_get_capacity();
		if (capacity > 100)
			capacity = 100;
		*((int *) data) = capacity;
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
	}	break;
	case POWER_SUPPLY_FG_IS_FULL_CHARGED:
		if (bq27520_get_capacity() >= 100)
			*((int *) data) = 1;
		else
			*((int *) data) = 0;
		/* Returns NOTIFY_STOP to skip Internal Fuel Gauge */
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_HAS_MAIN_BATTERY:
		/* bq27520 is NOT in Battery Pack */
{
		int ret, bat_det = -1;
		u16 flags = 0;

		ret = bq27520_read_u16(3, &flags, BQ27520_CMD_FLAGS);
		if (ret) {
			POWER_SUPPLY_ERR("get FLAGS err %d\n\n", ret);
		} else if (flags & 0x08) {
			bat_det = 1;
		}
		*((int *) data) = bat_det;
		ret = NOTIFY_STOP;
}
		break;
	case POWER_SUPPLY_FG_INIT:
		ret = NOTIFY_STOP;
		break;
	case POWER_SUPPLY_FG_GET_TEMP: /* int: battery temperature in units of 0.1C */
		*((int *) data) = bq27520_get_temperature();
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

static int bq27520_get_control_status(void)
{
	int ret;

	ret = bq27520_write_subcommand(2, CTRL_SUB_CONTROL_STATUS, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write CONTROL STATUS cmd err %d\n", ret);
		return ret;
	}

	control_status = 0;
	ret = bq27520_read_u16(2, &control_status, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get CONTROL STATUS err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27520_get_device_type(void)
{
	int ret;

	ret = bq27520_write_subcommand(1, CTRL_SUB_DEVICE_TYPE, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write DEVICE TYPE cmd err %d\n", ret);
		return ret;
	}

	device_type = 0;
	ret = bq27520_read_u16(1, &device_type, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get DEVICE TYPE err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27520_get_fw_version(void)
{
	int ret;

	ret = bq27520_write_subcommand(2, CTRL_SUB_FW_VERSION, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write FW VERSION cmd err %d\n", ret);
		return ret;
	}

	fw_version = 0;
	ret = bq27520_read_u16(2, &fw_version, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get FW VERSION err %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27520_get_df_version(void)
{
	int ret;

	ret = bq27520_write_subcommand(2, CTRL_SUB_DF_VERSION, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("write DF VERSION cmd err %d\n", ret);
		return ret;
	}

	df_version = 0;
	ret = bq27520_read_u16(2, &df_version, BQ27520_CMD_CTRL);
	if (ret) {
		POWER_SUPPLY_VDBG("get DF VERSION err %d\n", ret);
		return ret;
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int bq27520_dump_info(void)
{
	int ret;
	u16 flags = 0, health = 0, cycles = 0;
	u16 vbat = 0, capacity = 0, temp = 0;
	s16 curr = 0;

	putc('\n');

	ret = bq27520_clear_hibernate();
	if (ret) {
		POWER_SUPPLY_ERR("CLEAR HIBERNATE err %d\n", ret);
		goto _out;
	}

	ret = bq27520_get_df_version();
	if (ret) {
		POWER_SUPPLY_ERR("get DF VERSION err %d\n", ret);
		goto _out;
	}

	ret = bq27520_enable_impedance_track();
	if (ret) {
		POWER_SUPPLY_ERR("IT ENABLE err %d\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27520: enable IT\n");

	ret = bq27520_get_control_status();
	if (ret) {
		POWER_SUPPLY_ERR("get CTRL STATUS err %d\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27520-%s: FW 0x%04x, DF 0x%04x, STATUS 0x%04x\n",
			(fw_version >= 0x0329) ? "G4" : "G3",
			fw_version, df_version, control_status);

	ret = bq27520_read_u16(2, &flags, BQ27520_CMD_FLAGS);
	if (ret) {
		POWER_SUPPLY_ERR("get FLAGS err %d\n\n", ret);
		goto _out;
	}
	ret = bq27520_read_u16(2, &health, BQ27520_CMD_SOH);
	if (ret) {
		POWER_SUPPLY_ERR("get SOH err %d\n\n", ret);
		goto _out;
	}
	if (((health >> 8) & 0x0FF) > 0)
		health = health & 0x0FF;
	else
		health = -1;
	ret = bq27520_read_u16(2, &cycles, BQ27520_CMD_CC);
	if (ret) {
		POWER_SUPPLY_ERR("get CC err %d\n\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27520: Flags 0x%04x, Health %d%%, Cycles %d\n",
			flags, health, cycles);

	ret = bq27520_read_u16(2, &capacity, BQ27520_CMD_SOC);
	if (ret) {
		POWER_SUPPLY_ERR("get SOC err %d\n\n", ret);
		goto _out;
	}
	ret = bq27520_read_u16(2, &vbat, BQ27520_CMD_VOLT);
	if (ret) {
		POWER_SUPPLY_ERR("get VOLT err %d\n\n", ret);
		goto _out;
	}
	ret = bq27520_read_u16(2, (u16*)&curr, BQ27520_CMD_AI);
	if (ret) {
		POWER_SUPPLY_ERR("get AI err %d\n\n", ret);
		goto _out;
	}
	POWER_SUPPLY_PRINT("bq27520: Current %dmA, Capacity %d%%, VBAT %dmV\n",
			curr, capacity, vbat);

	/* battery temperature in units of 0.1K */
	ret = bq27520_read_u16(2, &temp, BQ27520_CMD_TEMP);
	if (ret) {
		POWER_SUPPLY_ERR("get TEMP err %d\n\n", ret);
		goto _out;
	}
	/* °C = K − 273.15 */
	/* battery temperature in units of 0.1C */
#if 0
	POWER_SUPPLY_PRINT("bq27520: BAT Temp %d (0.1C)\n",
			(((int)temp) - 2732));
#else
	print_temperature("bq27520: BAT Temp ", KELVIN2CELSIUS(temp), "\n");
#endif

	if (capacity > 100) {
		POWER_SUPPLY_ERR("invalid Capacity %d%%\n\n", capacity);
		ret = -EAGAIN;
		goto _out;
	}
	if (vbat < CONFIG_FUELGAUGE_BQ27520_MIN_VBAT || vbat > CONFIG_FUELGAUGE_BQ27520_MAX_VBAT
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

void bq27520_init(void)
{
	int ret;

	ret = bq27520_get_device_type();
	if (ret) {
		mdelay(100);
		ret = bq27520_get_device_type();
		if (ret) {
			POWER_SUPPLY_PRINT("power supply: no bq27520\n");
			return;
		}
	}
	if (device_type != 0x0520) {
		POWER_SUPPLY_ERR("unknown BQ27520 chip 0x%04X\n", device_type);
		return;
	}

	ret = bq27520_get_fw_version();
	if (ret) {
		POWER_SUPPLY_ERR("get FW VERSION err %d\n", ret);
		return;
	}
	if (fw_version >= 0x0329) {
		bq27520_regs = bq27520_g4_regs;
		POWER_SUPPLY_VDBG("found bq27520-G4\n");
	} else {
		POWER_SUPPLY_VDBG("found bq27520-G3\n");
	}

	ret = bq27520_dump_info();
	if (ret) {
		return;
	}

	bq_fg_nb.notifier_call = bq27520_fuelgauge_notifier_call;
	bq_fg_nb.priority = INT_MAX;
	ret = power_supply_register_notifier(POWER_SUPPLY_FEAT_EXT_FUELGAUGE, &bq_fg_nb);
	if (ret)
		POWER_SUPPLY_ERR("FuelGauge err %d\n", ret);

	has_bq27520 = 1;
}

void bq27520_shutdown(void)
{
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_FUELGAUGE_BQ27520_DEBUG
/* bq27520_dataflash */
static int do_bq27520_dataflash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;

	if (argc < 3) {
		puts("usage: bq27520_dataflash <class> <block>\n");
		return 0;
	}

	return 0;
}

U_BOOT_CMD(
	bq27520_dataflash, 3, 0, do_bq27520_dataflash,
	"bq27520 dataflash",
	""
);
#endif /* CONFIG_FUELGAUGE_BQ27520_DEBUG */

#ifdef CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG
#include <linux/usb/otg.h>

/* bq27520_test */
static int do_bq27520_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int vbat;
	int capacity;
	int curr;

	while (1) {
		vbat = bq27520_get_voltage();

		if (vbat <= CONFIG_FUELGAUGE_BQ27520_MIN_VBAT || vbat >= CONFIG_FUELGAUGE_BQ27520_MAX_VBAT) {
			printf("** bq27520: invalid VBAT %dmV\n", vbat);
		} else {
			printf("bq27520: VBAT %dmV\n", vbat);
		}

		capacity = bq27520_get_capacity();
		if (capacity == 0) {
			printf("** bq27520: CAPACITY %d%%\n", capacity);
		} else if (capacity > 100) {
			printf("** bq27520: invalid CAPACITY %d%%\n", capacity);
		} else {
			printf("bq27520: CAPACITY %d%%\n", capacity);
		}

		curr = bq27520_get_current();
		printf("bq27520: CURRENT %dmA\n", curr);

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
	bq27520_test, 1, 0, do_bq27520_test,
	"bq27520 test",
	""
);
#endif /* CONFIG_FUELGAUGE_BQ27520_VERBOSE_DEBUG */
