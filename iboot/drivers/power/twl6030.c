/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#if 0

#include <config.h>
#ifdef CONFIG_TWL6030_POWER

#include <twl6030.h>

/* Functions to read and write from TWL6030 */
static inline int twl6030_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	return i2c_write(chip_no, reg, 1, &val, 1);
}

static inline int twl6030_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	return i2c_read(chip_no, reg, 1, val, 1);
}

static int twl6030_gpadc_read_channel(u8 channel_no)
{
	u8 lsb = 0;
	u8 msb = 0;
	int ret = 0;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &lsb,
				GPCH0_LSB + channel_no * 2);
	if (ret)
		return ret;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &msb,
				GPCH0_MSB + channel_no * 2);
	if (ret)
		return ret;

	return (msb << 8) | lsb;
}

static int twl6030_gpadc_sw2_trigger(void)
{
	u8 val;
	int ret = 0;

	ret = twl6030_i2c_write_u8(TWL6030_CHIP_ADC, CTRL_P2_SP2, CTRL_P2);
	if (ret)
		return ret;

	/* Waiting until the SW1 conversion ends*/
	val =  CTRL_P2_BUSY;

	while (!((val & CTRL_P2_EOCP2) && (!(val & CTRL_P2_BUSY)))) {
		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &val, CTRL_P2);
		if (ret)
			return ret;
		udelay(1000);
	}

	return 0;
}

void twl6030_stop_usb_charging(void)
{
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, 0, CONTROLLER_CTRL1);

	return;
}

void twl6030_start_usb_charging(void)
{
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VICHRG_1500,
							CHARGERUSB_VICHRG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CIN_LIMIT_NONE,
							CHARGERUSB_CINLIMIT);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MBAT_TEMP,
							CONTROLLER_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MASK_MCHARGERUSB_THMREG,
							CHARGERUSB_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VOREG_4P0,
							CHARGERUSB_VOREG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CTRL2_VITERM_400,
							CHARGERUSB_CTRL2);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, TERM, CHARGERUSB_CTRL1);
	/* Enable USB charging */
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CONTROLLER_CTRL1_EN_CHARGER,
							CONTROLLER_CTRL1);
	return;
}

int twl6030_get_battery_current(void)
{
	int battery_current = 0;
	u8 msb = 0;
	u8 lsb = 0;

	twl6030_i2c_read_u8(TWL6030_CHIP_CHARGER, &msb, FG_REG_11);
	twl6030_i2c_read_u8(TWL6030_CHIP_CHARGER, &lsb, FG_REG_10);
	battery_current = ((msb << 8) | lsb);

	/* convert 10 bit signed number to 16 bit signed number */
	if (battery_current >= 0x2000)
		battery_current = (battery_current - 0x4000);

	battery_current = battery_current * 3000 / 4096;
	printf("Battery Current: %d mA\n", battery_current);

	return battery_current;
}

int twl6030_get_battery_voltage(void)
{
	int battery_volt = 0;
	int ret = 0;

	/* Start GPADC SW conversion */
	ret = twl6030_gpadc_sw2_trigger();
	if (ret) {
		printf("Failed to convert battery voltage\n");
		return ret;
	}

	/* measure Vbat voltage */
	battery_volt = twl6030_gpadc_read_channel(7);
	if (battery_volt < 0) {
		printf("Failed to read battery voltage\n");
		return ret;
	}
	battery_volt = (battery_volt * 25 * 1000) >> (10 + 2);
	printf("Battery Voltage: %d mV\n", battery_volt);

	return battery_volt;
}

void twl6030_init_battery_charging(void)
{
	u8 stat1 = 0;
	int battery_volt = 0;
	int ret = 0;

	/* Enable VBAT measurement */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, VBAT_MEAS, MISC1);

	/* Enable GPADC module */
	ret = twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, FGS | GPADCS, TOGGLE1);
	if (ret) {
		printf("Failed to enable GPADC\n");
		return;
	}

	battery_volt = twl6030_get_battery_voltage();
	if (battery_volt < 0)
		return;

	if (battery_volt < 3000)
		printf("Main battery voltage too low!\n");

	/* Check for the presence of USB charger */
	twl6030_i2c_read_u8(TWL6030_CHIP_CHARGER, &stat1, CONTROLLER_STAT1);

	/* check for battery presence indirectly via Fuel gauge */
	if ((stat1 & VBUS_DET) && (battery_volt < 3300))
		twl6030_start_usb_charging();

	return;
}

void twl6030_power_mmc_init()
{
	/* set voltage to 3.0 and turnon for APP */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, 0x15, VMMC_CFG_VOLTATE);
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, 0x21, VMMC_CFG_STATE);
}

void twl6030_usb_device_settings()
{
	u8 data = 0;

	/* Select APP Group and set state to ON */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, 0x21, VUSB_CFG_STATE);

	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &data, MISC2);
	data |= 0x10;

	/* Select the input supply for VBUS regulator */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, data, MISC2);
}
#endif

#else

/*----------------------------------------------------------------------*/

#include <common.h>
#include <errno.h>
#include <twl6030.h>
#include <asm/gpio.h>
#if defined(CONFIG_TWL6030_USB) && defined(CONFIG_OMAP44XX)
#include <asm/arch/usb.h>
#endif /* CONFIG_TWL6030_USB && CONFIG_OMAP44XX */
#if defined(CONFIG_OMAP44XX)
#include <asm/arch/sys_proto.h>
#endif /* CONFIG_OMAP44XX */

/*----------------------------------------------------------------------*/

/*#define CONFIG_TWL6030_DEBUG*/
/*#define CONFIG_TWL6030_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL6030_DEBUG
#define CONFIG_TWL6030_DEBUG
#endif
#ifndef CONFIG_TWL6030_VERBOSE_DEBUG
#define CONFIG_TWL6030_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6030_DEBUG
#define TWL_DPRINT(fmt, args...) \
	do {printf("[twl] " fmt, ##args);} while (0)
#define TWL_DPUTS(fmt) \
	do {puts("[twl] " fmt);} while (0)
#else /* CONFIG_TWL6030_DEBUG */
#define TWL_DPRINT(fmt, args...) \
	do {} while (0)
#define TWL_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_DEBUG */

#ifdef CONFIG_TWL6030_VERBOSE_DEBUG
#define TWL_VPRINT(fmt, args...) \
	do {printf("[twl] " fmt, ##args);} while (0)
#define TWL_VPUTS(fmt) \
	do {puts("[twl] " fmt);} while (0)
#else /* CONFIG_TWL6030_VERBOSE_DEBUG */
#define TWL_VPRINT(fmt, args...) \
	do {} while (0)
#define TWL_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_VERBOSE_DEBUG */

#define TWL_PRINT(fmt, args...) \
	do {printf("twl: " fmt, ##args);} while (0)
#define TWL_PUTS(fmt) \
	do {puts("twl: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define TWL_ERR(fmt, args...) \
	do {printf("twl: " fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

#define SUB_CHIP_ID0 0x48
#define SUB_CHIP_ID1 0x49
#define SUB_CHIP_ID2 0x4A
#define SUB_CHIP_ID3 0x4B

/* Base Address defns for twl4030_map[] */

/* subchip/slave 0 - USB ID */
#define TWL4030_BASEADD_USB		0x0000

/* subchip/slave 1 - AUD ID */
#define TWL4030_BASEADD_AUDIO_VOICE	0x0000
#define TWL4030_BASEADD_GPIO		0x0098
#define TWL4030_BASEADD_INTBR		0x0085
#define TWL4030_BASEADD_PIH		0x0080
#define TWL4030_BASEADD_TEST		0x004C

/* subchip/slave 2 - AUX ID */
#define TWL4030_BASEADD_INTERRUPTS	0x00B9
#define TWL4030_BASEADD_LED		0x00EE
#define TWL4030_BASEADD_MADC		0x0000
#define TWL4030_BASEADD_MAIN_CHARGE	0x0074
#define TWL4030_BASEADD_PRECHARGE	0x00AA
#define TWL4030_BASEADD_PWM0		0x00F8
#define TWL4030_BASEADD_PWM1		0x00FB
#define TWL4030_BASEADD_PWMA		0x00EF
#define TWL4030_BASEADD_PWMB		0x00F1
#define TWL4030_BASEADD_KEYPAD		0x00D2

#define TWL5031_BASEADD_ACCESSORY	0x0074 /* Replaces Main Charge */
#define TWL5031_BASEADD_INTERRUPTS	0x00B9 /* Different than TWL4030's
						  one */

/* subchip/slave 3 - POWER ID */
#define TWL4030_BASEADD_BACKUP		0x0014
#define TWL4030_BASEADD_INT		0x002E
#define TWL4030_BASEADD_PM_MASTER	0x0036
#define TWL4030_BASEADD_PM_RECEIVER	0x005B
#define TWL4030_BASEADD_RTC		0x001C
#define TWL4030_BASEADD_SECURED_REG	0x0000

/* Triton Core internal information (END) */

/* subchip/slave 0 0x48 - POWER */
#define TWL6030_BASEADD_RTC		0x0000
#define TWL6030_BASEADD_MEM		0x0017
#define TWL6030_BASEADD_PM_MASTER	0x001F
#define TWL6030_BASEADD_PM_SLAVE_MISC	0x0030 /* PM_RECEIVER */
#define TWL6030_BASEADD_PM_SLAVE_RES	0x00AD
#define TWL6030_BASEADD_PM_MISC		0x00E2
#define TWL6030_BASEADD_PM_PUPD		0x00F0

/* subchip/slave 1 0x49 - FEATURE */
#define TWL6030_BASEADD_USB		0x0000
#define TWL6030_BASEADD_GPADC_CTRL	0x002E
#define TWL6030_BASEADD_AUX		0x0090
#define TWL6030_BASEADD_PWM		0x00BA
#define TWL6030_BASEADD_GASGAUGE	0x00C0
#define TWL6030_BASEADD_PIH		0x00D0
#define TWL6030_BASEADD_CHARGER		0x00E0
#define TWL6025_BASEADD_CHARGER		0x00DA

/* subchip/slave 2 0x4A - DFT */
#define TWL6030_BASEADD_DIEID		0x00C0

/* subchip/slave 3 0x4B - AUDIO */
#define TWL6030_BASEADD_AUDIO		0x0000
#define TWL6030_BASEADD_RSV		0x0000
#define TWL6030_BASEADD_ZERO		0x0000

/*----------------------------------------------------------------------*/

/* LDO control registers ... offset is from the base of its register bank.
 * The first three registers of all power resource banks help hardware to
 * manage the various resource groups.
 */
/* Common offset in TWL4030/6030 */
#define VREG_GRP		0
/* TWL4030 register offsets */
#define VREG_TYPE		1
#define VREG_REMAP		2
#define VREG_DEDICATED		3	/* LDO control */
/* TWL6030 register offsets */
#define VREG_TRANS		1
#define VREG_STATE		2
#define VREG_VOLTAGE		3
#define VREG_VOLTAGE_SMPS	4
/* TWL6030 Misc register offsets */
#define VREG_BC_ALL		1
#define VREG_BC_REF		2
#define VREG_BC_PROC		3
#define VREG_BC_CLK_RST		4

/* TWL6030 LDO register values for CFG_TRANS */
#define TWL6030_CFG_TRANS_STATE_MASK	0x03
#define TWL6030_CFG_TRANS_STATE_OFF	0x00
/*
 * Auto means the following:
 * SMPS:	AUTO(PWM/PFM)
 * LDO:		AMS(SLP/ACT)
 * resource:	ON
 */
#define TWL6030_CFG_TRANS_STATE_AUTO	0x01
#define TWL6030_CFG_TRANS_SLEEP_SHIFT	2

/* TWL6030 LDO register values for CFG_STATE */
#define TWL6030_CFG_STATE_OFF	0x00
#define TWL6030_CFG_STATE_ON	0x01
#define TWL6030_CFG_STATE_OFF2	0x02
#define TWL6030_CFG_STATE_SLEEP	0x03
#define TWL6030_CFG_STATE_GRP_SHIFT	5
#define TWL6030_CFG_STATE_APP_SHIFT	2
#define TWL6030_CFG_STATE_MASK		0x03
#define TWL6030_CFG_STATE_APP_MASK	(TWL6030_CFG_STATE_MASK << \
						TWL6030_CFG_STATE_APP_SHIFT)
#define TWL6030_CFG_STATE_APP(v)	(((v) & TWL6030_CFG_STATE_APP_MASK) >>\
						TWL6030_CFG_STATE_APP_SHIFT)

/*
 * Enable/disable regulators by joining/leaving the P1 (processor) group.
 * We assume nobody else is updating the DEV_GRP registers.
 */
/* definition for 4030 family */
#define P3_GRP_4030	BIT(7)		/* "peripherals" */
#define P2_GRP_4030	BIT(6)		/* secondary processor, modem, etc */
#define P1_GRP_4030	BIT(5)		/* CPU/Linux */
/* definition for 6030 family */
#define P3_GRP_6030	BIT(2)		/* secondary processor, modem, etc */
#define P2_GRP_6030	BIT(1)		/* "peripherals" */
#define P1_GRP_6030	BIT(0)		/* CPU/Linux */

/*----------------------------------------------------------------------*/

/* TWL6030 */
#define TWL6030_VSYSMIN_LO_THRESHOLD	0x23
#define TWL6030_VSYSMIN_HI_THRESHOLD	0x24
#define TWL6030_PHOENIX_DEV_ON		0x25
#define		APP_DEVOFF	(1<<0)
#define		CON_DEVOFF	(1<<1)
#define		MOD_DEVOFF	(1<<2)
#define TWL6030_PHOENIX_CFG_VSYSLOW	0x28
#define		VSYSLOW_DELAY_SHIFT		0		/* VSYSLOW delay: Minimum: delay x 40 ms + 20 ms */
#define		VSYSLOW_DELAY_MASK		0x3F	/* VSYSLOW delay: Maximum: delay x 40 ms + 30 ms */

/*----------------------------------------------------------------------*/

/* need to access USB_PRODUCT_ID_LSB to identify which 6030 varient we are */
#define USB_PRODUCT_ID_LSB	0x02

/* need to check eeprom revision and jtagver number */
#define TWL6030_REG_EPROM_REV	0xdf
#define TWL6030_REG_JTAGVERNUM	0x87

/*----------------------------------------------------------------------*/

/* mapping the module id to slave id and base address */
struct twl_mapping {
	unsigned char sid;	/* Slave ID */
	unsigned char base;	/* base address */
};

static struct twl_mapping twl_map[] = {
	/*
	 * NOTE:  don't change this table without updating the
	 * <linux/i2c/twl.h> defines for TWL4030_MODULE_*
	 * so they continue to match the order in this table.
	 */
	/* 0x00 */ { SUB_CHIP_ID1, TWL6030_BASEADD_USB },
	/* 0x01 */ { SUB_CHIP_ID3, TWL6030_BASEADD_AUDIO },
	/* 0x02 */ { SUB_CHIP_ID2, TWL6030_BASEADD_DIEID },
	/* 0x03 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x04 */ { SUB_CHIP_ID1, TWL6030_BASEADD_PIH },

	/* 0x05 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x06 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x07 */ { SUB_CHIP_ID1, TWL6030_BASEADD_GPADC_CTRL },
	/* 0x08 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x09 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },

	/* 0x0A */ { SUB_CHIP_ID1, TWL6030_BASEADD_CHARGER },
	/* 0x0B */ { SUB_CHIP_ID1, TWL6030_BASEADD_GASGAUGE },
	/* 0x0C */ { SUB_CHIP_ID1, TWL6030_BASEADD_PWM },
	/* 0x0D */ { SUB_CHIP_ID0, TWL6030_BASEADD_ZERO },
	/* 0x0E */ { SUB_CHIP_ID1, TWL6030_BASEADD_ZERO },

	/* 0x0F */ { SUB_CHIP_ID2, TWL6030_BASEADD_ZERO },
	/* 0x10 */ { SUB_CHIP_ID2, TWL6030_BASEADD_ZERO },
	/* 0x11 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x12 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x13 */ { SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	/* 0x14 */ { SUB_CHIP_ID0, TWL6030_BASEADD_PM_MASTER },
	/* 0x15 */ { SUB_CHIP_ID0, TWL6030_BASEADD_PM_SLAVE_MISC },

	/* 0x16 */ { SUB_CHIP_ID0, TWL6030_BASEADD_RTC },
	/* 0x17 */ { SUB_CHIP_ID0, TWL6030_BASEADD_MEM },
	/* 0x18 */ { SUB_CHIP_ID1, TWL6025_BASEADD_CHARGER },
	/* 0x19 */ { SUB_CHIP_ID0, TWL6030_BASEADD_PM_SLAVE_RES },
};

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void twl_save_i2c_bus(void) __attribute__((always_inline));
static inline void twl_save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_SYS_I2C_BUS)
		i2c_set_bus_num(CONFIG_SYS_I2C_BUS);
}

static inline void twl_restore_i2c_bus(void) __attribute__((always_inline));
static inline void twl_restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_SYS_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define twl_save_i2c_bus()
#define twl_restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

/* Exported Functions */

/**
 * twl_i2c_write - Writes a n bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: an array of num_bytes containing data to write
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * IMPORTANT: for 'value' parameter: Allocate value num_bytes and
 * valid data starts at Offset 0.
 *
 * Returns the result of operation - 0 is success
 */
int twl_i2c_write(u8 mod_no, u8 *value, u8 reg, int num_bytes)
{
	int ret;
	int retries = 3;

	TWL_VPRINT("write(0x%02X): [0x%02X,%d]\n", twl_map[mod_no].sid,
			twl_map[mod_no].base + reg, num_bytes);

	twl_save_i2c_bus();

_retry:
	ret = i2c_write(twl_map[mod_no].sid, twl_map[mod_no].base + reg, 1, value, num_bytes);
	if (ret) {
		if (--retries > 0) {
			TWL_VPRINT("%d: write(0x%02X): [0x%02X,%d] err %d\n", retries, twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, num_bytes, ret);
			mdelay(10);
			goto _retry;
		} else {
#if !defined(CONFIG_SPL_BUILD)
			TWL_ERR("write(0x%02X): [0x%02X,%d] err %d\n", twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, num_bytes, ret);
#endif
		}
	}

	twl_restore_i2c_bus();

	return ret;
}

/**
 * twl_i2c_read - Reads a n bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: an array of num_bytes containing data to be read
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * Returns result of operation - num_bytes is success else failure.
 */
int twl_i2c_read(u8 mod_no, u8 *value, u8 reg, int num_bytes)
{
	int ret;
	int retries = 3;

	TWL_VPRINT("read(0x%02X): [0x%02X,%d]\n", twl_map[mod_no].sid,
			twl_map[mod_no].base + reg, num_bytes);

	twl_save_i2c_bus();

_retry:
	ret = i2c_read(twl_map[mod_no].sid, twl_map[mod_no].base + reg, 1, value, num_bytes);
	if (ret) {
		if (--retries > 0) {
			TWL_VPRINT("%d: read(0x%02X): [0x%02X,%d] err %d\n", retries, twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, num_bytes, ret);
			mdelay(10);
			goto _retry;
		} else {
#if !defined(CONFIG_SPL_BUILD)
			TWL_ERR("read(0x%02X): [0x%02X,%d] err %d\n", twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, num_bytes, ret);
#endif
		}
	}

	twl_restore_i2c_bus();

	return ret;
}

/**
 * twl_i2c_write_u8 - Writes a 8 bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: the value to be written 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl_i2c_write_u8(u8 mod_no, u8 value, u8 reg)
{
	int ret;
	int retries = 3;

	TWL_VPRINT("write(0x%02X): [0x%02X]=0x%02X\n", twl_map[mod_no].sid,
			twl_map[mod_no].base + reg, value);

	twl_save_i2c_bus();

_retry:
	ret = i2c_write(twl_map[mod_no].sid, twl_map[mod_no].base + reg, 1, &value, 1);
	if (ret) {
		if (--retries > 0) {
			TWL_VPRINT("%d: write(0x%02X): [0x%02X]=0x%02X err %d\n", retries, twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, value, ret);
			mdelay(10);
			goto _retry;
		} else {
#if !defined(CONFIG_SPL_BUILD)
			TWL_ERR("write(0x%02X): [0x%02X]=0x%02X err %d\n", twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, value, ret);
#endif
		}
	}

	twl_restore_i2c_bus();

	return ret;
}

/**
 * twl_i2c_read_u8 - Reads a 8 bit register from TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: the value read 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl_i2c_read_u8(u8 mod_no, u8 *value, u8 reg)
{
	int ret;
	int retries = 3;

	twl_save_i2c_bus();

_retry:
	ret = i2c_read(twl_map[mod_no].sid, twl_map[mod_no].base + reg, 1, value, 1);
	if (ret) {
		if (--retries > 0) {
			TWL_VPRINT("%d: read(0x%02X): [0x%02X] err %d\n", retries, twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, ret);
			mdelay(10);
			goto _retry;
		} else {
#if !defined(CONFIG_SPL_BUILD)
			TWL_ERR("read(0x%02X): [0x%02X] err %d\n", twl_map[mod_no].sid,
					twl_map[mod_no].base + reg, ret);
#endif
		}
	} else {
		TWL_VPRINT("read(0x%02X): [0x%02X]=0x%02X\n", twl_map[mod_no].sid,
				twl_map[mod_no].base + reg, *value);
	}

	twl_restore_i2c_bus();

	return ret;
}

/* Sets and clears bits on an given register on a given module */
int twl_clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	int ret = 0;
	u8 old_val;
	u8 val;

	twl_save_i2c_bus();

	/* Gets the initial register value */
	ret = twl_i2c_read_u8(mod_no, &old_val, reg);
	if (ret)
		goto _out;

	/* Clearing all those bits to clear */
	val = old_val & ~(clear);

	/* Setting all those bits to set */
	val |= set;

	/* Update the register */
	ret = twl_i2c_write_u8(mod_no, val, reg);
	if (ret)
		goto _out;

_out:
	twl_restore_i2c_bus();
	return ret;
}

/*----------------------------------------------------------------------*/

/* Regulators */
#define REGULATOR_TYPE_ADJUSTABLE_LDO	0
#define REGULATOR_TYPE_FIXED_LDO		1
#define REGULATOR_TYPE_FIXED_RESOURCE	2
#define REGULATOR_TYPE_ADJUSTABLE_SMPS	3

struct twl6030reg_mapping {
	const char*	name;
	int			type;

	/* start of regulator's PM_RECEIVER control register bank */
	u8			base;

	/* chip constraints on regulator behavior */
	u16			min_mV;
	u16			max_mV;
};

static struct twl6030reg_mapping twl6030reg_map[] = {
#if defined(CONFIG_SPL_BUILD)

#ifdef CONFIG_TWL6032_POWER
/* TWL6032 */
/* 0x00 */ { "ldo2",	REGULATOR_TYPE_ADJUSTABLE_LDO,	0x54,	1000,	3300, },
/* 0x01 */ { "ldo5",	REGULATOR_TYPE_ADJUSTABLE_LDO,	0x68,	1000,	3300, },
#else /* CONFIG_TWL6030_POWER */
/* 0x00 */ { "vaux1",	REGULATOR_TYPE_ADJUSTABLE_LDO,	0x54,	1000,	3300, },
/* 0x01 */ { "vmmc",	REGULATOR_TYPE_ADJUSTABLE_LDO,	0x68,	1000,	3300, },
#endif

#else /* !CONFIG_SPL_BUILD */

#ifdef CONFIG_TWL6032_POWER
/* TWL6032 */
/* 0x00 */ { "ldo1",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x6C,	1000,	3300, },
/* 0x01 */ { "ldo2",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x54,	1000,	3300, },
/* 0x02 */ { "ldo3",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x5C,	1000,	3300, },
/* 0x03 */ { "ldo4",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x58,	1000,	3300, },
/* 0x04 */ { "ldo5",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x68,	1000,	3300, },
/* 0x05 */ { "ldo6",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x60,	1000,	3300, },
/* 0x06 */ { "ldo7",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x74,	1000,	3300, },
/* 0x07 */ { "ldoln",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x64,	1000,	3300, },
/* 0x08 */ { "ldousb",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x70,	1000,	3300, },
/* 0x09 */ { "clk32kg",		REGULATOR_TYPE_FIXED_RESOURCE,	0x8C,	0,		0, },
/* 0x0A */ { "clk32kaudio",	REGULATOR_TYPE_FIXED_RESOURCE,	0x8F,	0,		0, },
#else /* CONFIG_TWL6030_POWER */
/* 0x00 */ { "vaux1",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x54,	1000,	3300, },
/* 0x01 */ { "vaux2",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x58,	1000,	3300, },
/* 0x02 */ { "vaux3",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x5c,	1000,	3300, },
/* 0x03 */ { "vmmc",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x68,	1000,	3300, },
/* 0x04 */ { "vpp",			REGULATOR_TYPE_ADJUSTABLE_LDO,	0x6c,	1000,	3300, },
/* 0x05 */ { "vusim",		REGULATOR_TYPE_ADJUSTABLE_LDO,	0x74,	1000,	3300, },
/* 0x06 */ { "vana",		REGULATOR_TYPE_FIXED_LDO,		0x50,	2100,	0, },
/* 0x07 */ { "vcxio",		REGULATOR_TYPE_FIXED_LDO,		0x60,	1800,	0, },
/* 0x08 */ { "vdac",		REGULATOR_TYPE_FIXED_LDO,		0x64,	1800,	0, },
/* 0x09 */ { "vusb",		REGULATOR_TYPE_FIXED_LDO,		0x70,	3300,	0, },
/* 0x0A */ { "clk32kg",		REGULATOR_TYPE_FIXED_RESOURCE,	0x8C,	0,		0, },
/* 0x0B */ { "clk32kaudio",	REGULATOR_TYPE_FIXED_RESOURCE,	0x8F,	0,		0, },
#endif

#endif /* CONFIG_SPL_BUILD */
};

static inline int
twlreg_read(int regulator, u8 slave_subgp, u8 offset)
{
	u8 value = 0;
	int ret;
	offset += twl6030reg_map[regulator].base;
	ret = twl_i2c_read_u8(slave_subgp, &value, offset);
	return (ret < 0) ? ret : value;
}

static inline int
twlreg_write(int regulator, u8 slave_subgp, u8 offset, u8 value)
{
	offset += twl6030reg_map[regulator].base;
	return twl_i2c_write_u8(slave_subgp, value, offset);
}

static int twl6030reg_set_trans_state(int regulator, u8 shift, u8 val)
{
	int rval;
	u8 mask;

	/* Read CFG_TRANS register of TWL6030 */
	rval = twlreg_read(regulator, TWL_MODULE_PM_RECEIVER, VREG_TRANS);

	if (rval < 0)
		return rval;

	mask = TWL6030_CFG_TRANS_STATE_MASK << shift;
	val = (val << shift) & mask;

	/* If value is already set, no need to write to reg */
	if (val == (rval & mask))
		return 0;

	rval &= ~mask;
	rval |= val;

	return twlreg_write(regulator, TWL_MODULE_PM_RECEIVER, VREG_TRANS, rval);
}

static int twl6030ldo_set_voltage(int regulator, int mV)
{
	int ret;
	int vsel;

	if ((mV < twl6030reg_map[regulator].min_mV) ||
			(mV > twl6030reg_map[regulator].max_mV))
		return -EDOM;

	/*
	 * Use the below formula to calculate VSEL
	 * mV = 1000mv + 100mv * (VSEL - 1)
	 */
	vsel = (mV - 1000) / 100 + 1;

	ret = twlreg_write(regulator, TWL_MODULE_PM_RECEIVER, VREG_VOLTAGE, vsel);

	if (ret) {
		TWL_PRINT("LDO <%s> volt %d mV err %d\n",
				twl6030reg_map[regulator].name, mV, ret);
	} else {
		TWL_DPRINT("LDO <%s> volt %d mV\n",
				twl6030reg_map[regulator].name, mV);
	}

	return ret;
}

int regulator_set_voltage(int regulator, int mV)
{
	int ret;

	switch (twl6030reg_map[regulator].type) {
	case REGULATOR_TYPE_ADJUSTABLE_LDO:
		ret = twl6030ldo_set_voltage(regulator, mV);
		break;
#ifdef CONFIG_TWL6032_POWER
#else /* CONFIG_TWL6030_POWER */
	case REGULATOR_TYPE_FIXED_LDO:
		ret = 0;
		TWL_DPRINT("LDO <%s> volt %d mV\n",
				twl6030reg_map[regulator].name, twl6030reg_map[regulator].min_mV);
		break;
#endif
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

int regulator_enable(int regulator)
{
	int	grp = 0;
	int	ret;

#if !defined(CONFIG_TWL6032_POWER)
	grp = twlreg_read(regulator, TWL_MODULE_PM_RECEIVER, VREG_GRP);
	if (grp < 0)
		return grp;
#endif /* !CONFIG_TWL6032_POWER */

	ret = twlreg_write(regulator, TWL_MODULE_PM_RECEIVER, VREG_STATE,
			grp << TWL6030_CFG_STATE_GRP_SHIFT |
			TWL6030_CFG_STATE_ON);
	/*
	 * Ensure it stays in Auto mode when we enter suspend state.
	 * (TWL6030 in sleep mode).
	 */
	if (!ret)
		ret = twl6030reg_set_trans_state(regulator,
				TWL6030_CFG_TRANS_SLEEP_SHIFT,
				TWL6030_CFG_TRANS_STATE_AUTO);

	if (ret) {
		TWL_PRINT("LDO <%s> enable err %d\n",
				twl6030reg_map[regulator].name, ret);
	} else {
		TWL_DPRINT("LDO <%s> enabled\n",
				twl6030reg_map[regulator].name);
	}

	return ret;
}

int regulator_disable(int regulator)
{
	int grp = 0;
	int ret;

#if !defined(CONFIG_TWL6032_POWER)
	grp = P1_GRP_6030 | P2_GRP_6030 | P3_GRP_6030;
#endif /* !CONFIG_TWL6032_POWER */

	/* For 6030, set the off state for all grps enabled */
	ret = twlreg_write(regulator, TWL_MODULE_PM_RECEIVER, VREG_STATE,
			(grp) << TWL6030_CFG_STATE_GRP_SHIFT |
			TWL6030_CFG_STATE_OFF);

	/* Ensure it remains OFF when we enter suspend (TWL6030 in sleep). */
	if (!ret)
		ret = twl6030reg_set_trans_state(regulator,
				TWL6030_CFG_TRANS_SLEEP_SHIFT,
				TWL6030_CFG_TRANS_STATE_OFF);

	if (ret) {
		TWL_PRINT("LDO <%s> disable err %d\n",
				twl6030reg_map[regulator].name, ret);
	} else {
		TWL_DPRINT("LDO <%s> disabled\n",
				twl6030reg_map[regulator].name);
	}

	return ret;
}

int regulator_is_enabled(int regulator)
{
#if defined(CONFIG_TWL6032_POWER)
	int val = twlreg_read(regulator, TWL_MODULE_PM_RECEIVER, VREG_STATE);
	val &= TWL6030_CFG_STATE_MASK;
	return (val == TWL6030_CFG_STATE_ON);
#else
	int grp, val;

	grp = twlreg_read(regulator, TWL_MODULE_PM_RECEIVER, VREG_GRP);
	if (grp < 0)
		return grp;
	grp &= P1_GRP_6030;
	val = twlreg_read(regulator, TWL_MODULE_PM_RECEIVER, VREG_STATE);
	val = TWL6030_CFG_STATE_APP(val);

	return grp && (val == TWL6030_CFG_STATE_ON);
#endif /* CONFIG_TWL6032_POWER */
}

/*----------------------------------------------------------------------*/

void twl6030_poweroff(void)
{
#ifdef CONFIG_SPL_BUILD
	u8 val = 0;
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, TWL6030_PHOENIX_DEV_ON);
	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;
	twl_i2c_write_u8(TWL6030_MODULE_ID0, val, TWL6030_PHOENIX_DEV_ON);
#else
	int err;
	u8 val;

	val = 0;
	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, TWL6030_PHOENIX_DEV_ON);
	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;
	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val, TWL6030_PHOENIX_DEV_ON);
	if (err) {
		TWL_PRINT("I2C err %d on PHOENIX_DEV_ON\n", err);
		board_reboot(NULL);
	} else
		TWL_VPUTS("wait for device off\n");
#endif /* CONFIG_SPL_BUILD */
	for (;;);
}

/*----------------------------------------------------------------------*/

#ifndef CONFIG_SPL_BUILD

/*----------------------------------------------------------------------*/

/* TWL6040 */
#define TWL6040_REG_ASICREV		0x02
#define		TWL6040_REV_1_0				0x00
#define		TWL6040_REV_1_1				0x01
#define		TWL6040_REV_1_3				0x02
#define		TWL6041_REV_2_0				0x10
#define		TWL6041_REV_2_2				0x12
#define TWL6040_REG_HPPLLCTL	0x07
#define TWL6040_REG_LPPLLCTL	0x08
#define TWL6040_REG_ACCCTL		0x2D
#define		TWL6040_I2CSEL				0x01
#define		TWL6040_I2CMODE_MASK		(0x3 << 4)
#define		TWL6040_I2CMODE_NORMAL		(0x0 << 4)
#define		TWL6040_I2CMODE_FAST		(0x1 << 4)
#define		TWL6040_I2CMODE_FAST_PLUS	(0x2 << 4)
#define		TWL6040_I2CMODE_HIGH		(0x3 << 4)
#define		TWL6040_INTCLRMODE			0x08
#define		TWL6040_CLK32KSEL			0x40

/*----------------------------------------------------------------------*/

#if defined(CONFIG_TWL6030_USB)
#if defined(CONFIG_OMAP44XX)
static struct twl6030_usb_data omap4_usbphy_data = {
	.phy_init		= omap4_usb_phy_init,
	.phy_exit		= omap4_usb_phy_exit,
	.phy_power		= omap4_usb_phy_power,
};
#else
#error "TWL6030 USB only supports OMAP44xx\n"
#endif /* CONFIG_OMAP44XX */
#endif /* CONFIG_TWL6030_USB */

/* board_power_init */
static void inline __board_power_resources_init(void) {}
void board_power_resources_init(void) __attribute__((weak, alias("__board_power_resources_init")));

static int twl6030_init_done __attribute__ ((section(".data"))) = -1;
u8 twl6032_eprom_rev __attribute__ ((section(".data"))) = 0;
u8 twl6032_rev __attribute__ ((section(".data"))) = 0;
u8 twl6032_usb_id __attribute__ ((section(".data"))) = 0;
u8 twl6040_rev __attribute__ ((section(".data"))) = 0;

/*----------------------------------------------------------------------*/

#if defined(CONFIG_TWL6032_VSYSLOW_DELAY_MS) && (CONFIG_TWL6032_VSYSLOW_DELAY_MS > 0)
static void twl6030_setup_vsyslow_delay(int ms)
{
	u8 val = 0;

	if (ms < 40)
		ms = 40;
	else if (ms > 400)
		ms = 400;

	ms = (ms / 40) & VSYSLOW_DELAY_MASK;

	twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, TWL6030_PHOENIX_CFG_VSYSLOW);
	val &= ~VSYSLOW_DELAY_MASK;
	val |= (ms << VSYSLOW_DELAY_SHIFT);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, val, TWL6030_PHOENIX_CFG_VSYSLOW);

#ifdef CONFIG_TWL6030_DEBUG
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, TWL6030_PHOENIX_CFG_VSYSLOW);
	TWL_DPRINT("PH_CFG_VSYSLOW=0x%02X\n", val);
#endif /* CONFIG_TWL6030_DEBUG */
}
#endif /* CONFIG_TWL6032_VSYSLOW_DELAY_MS */

#if !defined(CONFIG_BOARD_HAS_NO_TWL6040)
static void twl6040_init(void)
{
	int ret;
	u8 clear, set;

	/* interrupts cleared on read */
	/* dual-access registers controlled by I2C only */
	clear = TWL6040_INTCLRMODE;
	set = TWL6040_I2CSEL | TWL6040_INTCLRMODE;
#if defined(CONFIG_OMAP4_PMIC_SPEED) && (CONFIG_OMAP4_PMIC_SPEED > 400)
	/* James Wu: the maximum speed of I2C-1 bus frequency depends on the system clock:
	 * 2.2 MHz (if 19.2 MHz), 2.4 MHz (26 MHz) or 2.9 MHz (38.4 MHz).
	 */
#if (CONFIG_OMAP4_PMIC_SPEED >= 1000) && (CONFIG_OMAP4_PMIC_SPEED <= 3400)
	/* Setup TWL6040 I2C mode to be High mode (up to 3.4 Mbit/s). */
	TWL_PUTS("twl6040 i2c is High mode\n");
	clear |= TWL6040_I2CMODE_MASK;
	set |= TWL6040_I2CMODE_HIGH;
#elif (CONFIG_OMAP4_PMIC_SPEED >= 400) && (CONFIG_OMAP4_PMIC_SPEED < 1000)
	/* Setup TWL6040 I2C mode to be Fast-mode Plus (up to 1 Mbit/s). */
	TWL_PUTS("twl6040 i2c is Fast-mode Plus\n");
	clear |= TWL6040_I2CMODE_MASK;
	set |= TWL6040_I2CMODE_FAST_PLUS;
#endif
#endif /* CONFIG_OMAP4_PMIC_SPEED */
	twl_clear_n_set(TWL_MODULE_AUDIO_VOICE, clear, set, TWL6040_REG_ACCCTL);

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &twl6040_rev, TWL6040_REG_ASICREV);
	if (ret == 0) {
		TWL_PRINT("6040 rev: 0x%02X\n", twl6040_rev);
		/*
		 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
		 * when AUDPWRON = 0, which causes current drain on this pin's
		 * pull-down on OMAP side. The workaround consists of disabling
		 * pull-down resistor of ABE_PDM_UL_DATA pin
		 * Impacted revisions: ES1.1, ES1.2 (both share same ASICREV value),
		 * ES1.3, ES2.0 and ES2.2
		 */
		if ((twl6040_rev == TWL6040_REV_1_1) ||
			(twl6040_rev == TWL6040_REV_1_3) ||
			(twl6040_rev == TWL6041_REV_2_0) ||
			(twl6040_rev == TWL6041_REV_2_2)) {
#if defined(CONFIG_OMAP44XX)
#if 0
			omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
				OMAP_PIN_INPUT);
#else
			writew((IEN | M0), CONTROL_PADCONF_CORE + ABE_PDM_UL_DATA);	/* abe_pdm_ul_data */
#endif
#endif /* CONFIG_OMAP44XX */
		}
	}
}
#endif /* !CONFIG_BOARD_HAS_NO_TWL6040 */

void twl6030_interrupt_unmask(u8 bit_mask, u8 offset)
{
	u8 unmask_value;

	twl_i2c_read_u8(TWL_MODULE_PIH, &unmask_value,
			REG_INT_STS_A + offset);
	unmask_value &= (~(bit_mask));
	twl_i2c_write_u8(TWL_MODULE_PIH, unmask_value,
			REG_INT_STS_A + offset); /* unmask INT_MSK_A/B/C */
}

void twl6030_interrupt_mask(u8 bit_mask, u8 offset)
{
	u8 mask_value;

	twl_i2c_read_u8(TWL_MODULE_PIH, &mask_value,
			REG_INT_STS_A + offset);
	mask_value |= (bit_mask);
	twl_i2c_write_u8(TWL_MODULE_PIH, mask_value,
			REG_INT_STS_A + offset); /* mask INT_MSK_A/B/C */
}

void twl6030_interrupt_clear(u8 bit_mask, u8 offset)
{
	twl_i2c_write_u8(TWL_MODULE_PIH, bit_mask,
			REG_INT_STS_A + offset); /* mask INT_MSK_A/B/C */
}

void twl6030_clear_interrupts(void)
{
#if 0
	/*
	 * NOTE:
	 * Simulation confirms that documentation is wrong w.r.t the
	 * interrupt status clear operation. A single *byte* write to
	 * any one of STS_A to STS_C register results in all three
	 * STS registers being reset. Since it does not matter which
	 * value is written, all three registers are cleared on a
	 * single byte write, so we just use 0x0 to clear.
	 */
	twl_i2c_write_u8(TWL_MODULE_PIH, 0x00, REG_INT_STS_A);
#else
	/* In Bootloader, we force to clear all interrupts */
	u32 mask = 0xFFFFFFFF;
	twl_i2c_write(TWL_MODULE_PIH, (u8 *)&mask, REG_INT_STS_A, 3); /* clear INT_STS_A,B,C */
#endif
}

/* Platform Dependent Power Initialisations */
void power_init(void)
{
	u32 mask;

	putc('\n');
	twl_i2c_read_u8(TWL_MODULE_USB, &twl6032_usb_id, USB_PRODUCT_ID_LSB);
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &twl6032_rev, TWL6030_REG_JTAGVERNUM);
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &twl6032_eprom_rev, TWL6030_REG_EPROM_REV);
	TWL_PRINT("6032 USB pid: 0x%02X\n", twl6032_usb_id);
	TWL_PRINT("6032 rev: 0x%02X\n", twl6032_rev);
	TWL_PRINT("6032 EPROM rev: 0x%02X\n", twl6032_eprom_rev);

#if 0
{
	u8 value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &value, TWL6030_VSYSMIN_LO_THRESHOLD);
	TWL_PRINT("6032 VSYSMIN_LO: %d mV\n", 2000 + 50 * value);
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &value, TWL6030_VSYSMIN_HI_THRESHOLD);
	TWL_PRINT("6032 VSYSMIN_HI: %d mV\n", 2000 + 50 * value);
}
#endif

	/* twl6030_init_irq() */
	mask = 0xFFFFFFFF;
	twl_i2c_write(TWL_MODULE_PIH, (u8 *)&mask, REG_INT_MSK_LINE_A, 3); /* MASK ALL INT LINES */
	twl_i2c_write(TWL_MODULE_PIH, (u8 *)&mask, REG_INT_MSK_STS_A, 3); /* MASK ALL INT STS */
	twl_i2c_write(TWL_MODULE_PIH, (u8 *)&mask, REG_INT_STS_A, 3); /* clear INT_STS_A,B,C */

	regulator_enable(TWL6032_CLK32KG);
#if defined(CONFIG_BOARD_HAS_NO_TWL6040)
	regulator_disable(TWL6032_CLK32KAUDIO);
#else
	regulator_enable(TWL6032_CLK32KAUDIO);
	udelay(100);
	twl6040_init();
#endif /* !CONFIG_BOARD_HAS_NO_TWL6040 */

#if defined(CONFIG_TWL6032_VSYSLOW_DELAY_MS) && (CONFIG_TWL6032_VSYSLOW_DELAY_MS > 0)
	twl6030_setup_vsyslow_delay(CONFIG_TWL6032_VSYSLOW_DELAY_MS);
#endif /* CONFIG_TWL6032_VSYSLOW_DELAY_MS */

	/* Turn on/off the board-specific regulators. */
	board_power_resources_init();

#ifdef CONFIG_TWL6030_USB
#ifdef CONFIG_OMAP44XX
	twl6030_usb_init(&omap4_usbphy_data);
#endif /* CONFIG_OMAP44XX */
#endif /* CONFIG_TWL6030_USB */
#ifdef CONFIG_TWL6032_CHARGER
	twl6032_charger_init();
#endif /* CONFIG_TWL6032_CHARGER */

#if !defined(CONFIG_BOARD_HAS_NO_TWL6040)
#ifdef CONFIG_TWL6040_VIBRATOR
	twl6040_vib_init();
#else
#if defined(CONFIG_TWL6040_POWERON_GPIO) && \
		(CONFIG_TWL6040_POWERON_GPIO >= 0) && (CONFIG_TWL6040_POWERON_GPIO < 192)
	gpio_direction_output(CONFIG_TWL6040_POWERON_GPIO, 0);
	/* power-down sequence latency */
	udelay(500);
	/* enable internal 32kHz oscillator */
	twl_clear_n_set(TWL_MODULE_AUDIO_VOICE, 0, TWL6040_CLK32KSEL, TWL6040_REG_ACCCTL);
#else
#error "Invalid CONFIG_TWL6040_POWERON_GPIO\n"
#endif /* CONFIG_TWL6040_POWERON_GPIO */
#endif /* CONFIG_TWL6040_VIBRATOR */
#endif /* !CONFIG_BOARD_HAS_NO_TWL6040 */

	twl6030_init_done = 1;
}

void power_shutdown(void)
{
	if (twl6030_init_done != 1)
		return;

#if !defined(CONFIG_BOARD_HAS_NO_TWL6040)
#ifdef CONFIG_TWL6040_VIBRATOR
	twl6040_vib_shutdown();
#endif /* CONFIG_TWL6040_VIBRATOR */
#endif /* !CONFIG_BOARD_HAS_NO_TWL6040 */

#ifdef CONFIG_TWL6032_CHARGER
	twl6032_charger_shutdown();
#endif /* CONFIG_TWL6032_CHARGER */

#ifdef CONFIG_TWL6030_USB
	twl6030_usb_shutdown();
#endif /* CONFIG_TWL6030_USB */

	regulator_disable(TWL6032_CLK32KAUDIO);

#if 0
	/* Force to turn off MMC1 power */
#ifdef CONFIG_TWL6032_POWER
	if (regulator_is_enabled(TWL6032_LDO5)) {
		regulator_disable(TWL6032_LDO5);
		mdelay(10);
	}
#else
	if (regulator_is_enabled(TWL6030_VMMC)) {
		regulator_disable(TWL6030_VMMC);
		mdelay(10);
	}
#endif
#endif

	twl6030_clear_interrupts();
}

#endif /* !CONFIG_SPL_BUILD */

#endif
