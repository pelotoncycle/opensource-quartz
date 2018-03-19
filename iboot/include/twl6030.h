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

#include <common.h>
#include <i2c.h>

/* I2C chip addresses */
#define TWL6030_CHIP_PM		0x48

#define TWL6030_CHIP_USB	0x49
#define TWL6030_CHIP_ADC	0x49
#define TWL6030_CHIP_CHARGER	0x49
#define TWL6030_CHIP_PWM	0x49

/* Slave Address 0x48 */
#define VMMC_CFG_STATE		0x9A
#define VMMC_CFG_VOLTATE	0x9B
#define VUSB_CFG_STATE		0xA2

#define MISC1			0xE4
#define VAC_MEAS		(1 << 2)
#define VBAT_MEAS		(1 << 1)
#define BB_MEAS			(1 << 0)

#define MISC2			0xE5

/* Slave Address 0x49 */

/* Battery CHARGER REGISTERS */
#define CONTROLLER_INT_MASK	0xE0
#define CONTROLLER_CTRL1	0xE1
#define CONTROLLER_WDG		0xE2
#define CONTROLLER_STAT1	0xE3
#define CHARGERUSB_INT_STATUS	0xE4
#define CHARGERUSB_INT_MASK	0xE5
#define CHARGERUSB_STATUS_INT1	0xE6
#define CHARGERUSB_STATUS_INT2	0xE7
#define CHARGERUSB_CTRL1	0xE8
#define CHARGERUSB_CTRL2	0xE9
#define CHARGERUSB_CTRL3	0xEA
#define CHARGERUSB_STAT1	0xEB
#define CHARGERUSB_VOREG	0xEC
#define CHARGERUSB_VICHRG	0xED
#define CHARGERUSB_CINLIMIT	0xEE
#define CHARGERUSB_CTRLLIMIT1	0xEF

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_500		0x4
#define CHARGERUSB_VICHRG_1500		0xE
/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100	0x1
#define CHARGERUSB_CIN_LIMIT_300	0x5
#define CHARGERUSB_CIN_LIMIT_500	0x9
#define CHARGERUSB_CIN_LIMIT_NONE	0xF
/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 6)
#define MAC_EOC			(1 << 5)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)
/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)
/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52		0x01
#define CHARGERUSB_VOREG_4P0		0x19
#define CHARGERUSB_VOREG_4P2		0x23
#define CHARGERUSB_VOREG_4P76		0x3F
/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT		(1 << 7)
#define OPA_MODE		(1 << 6)
#define HZ_MODE			(1 << 5)
#define TERM			(1 << 4)
/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400	(7 << 5)
/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)
/* CONTROLLER_STAT1 */
#define CHRG_EXTCHRG_STATZ	(1 << 7)
#define CHRG_DET_N		(1 << 5)
#define VAC_DET			(1 << 3)
#define VBUS_DET		(1 << 2)

#define FG_REG_10	0xCA
#define FG_REG_11	0xCB

#define TOGGLE1		0x90
#define FGS		(1 << 5)
#define FGR		(1 << 4)
#define GPADCS		(1 << 1)
#define GPADCR		(1 << 0)

#define CTRL_P2		0x34
#define CTRL_P2_SP2	(1 << 2)
#define CTRL_P2_EOCP2	(1 << 1)
#define CTRL_P2_BUSY	(1 << 0)

#define GPCH0_LSB	0x57
#define GPCH0_MSB	0x58

void twl6030_init_battery_charging(void);
void twl6030_usb_device_settings(void);
void twl6030_start_usb_charging(void);
void twl6030_stop_usb_charging(void);
int twl6030_get_battery_voltage(void);
int twl6030_get_battery_current(void);
void twl6030_power_mmc_init(void);

#else

/*----------------------------------------------------------------------*/

#include <common.h>
#include <i2c.h>

#include <icom/regulator.h>

#ifndef BIT
#define BIT(x)			(1 << (x))
#endif

/*----------------------------------------------------------------------*/

/*
 * Using the twl4030 core we address registers using a pair
 *	{ module id, relative register offset }
 * which that core then maps to the relevant
 *	{ i2c slave, absolute register address }
 *
 * The module IDs are meaningful only to the twl4030 core code,
 * which uses them as array indices to look up the first register
 * address each module uses within a given i2c slave.
 */

/* Slave 0 (i2c address 0x48) */
#define TWL4030_MODULE_USB		0x00

/* Slave 1 (i2c address 0x49) */
#define TWL4030_MODULE_AUDIO_VOICE	0x01
#define TWL4030_MODULE_GPIO		0x02
#define TWL4030_MODULE_INTBR		0x03
#define TWL4030_MODULE_PIH		0x04
#define TWL4030_MODULE_TEST		0x05

/* Slave 2 (i2c address 0x4a) */
#define TWL4030_MODULE_KEYPAD		0x06
#define TWL4030_MODULE_MADC		0x07
#define TWL4030_MODULE_INTERRUPTS	0x08
#define TWL4030_MODULE_LED		0x09
#define TWL4030_MODULE_MAIN_CHARGE	0x0A
#define TWL4030_MODULE_PRECHARGE	0x0B
#define TWL4030_MODULE_PWM0		0x0C
#define TWL4030_MODULE_PWM1		0x0D
#define TWL4030_MODULE_PWMA		0x0E
#define TWL4030_MODULE_PWMB		0x0F

#define TWL5031_MODULE_ACCESSORY	0x10
#define TWL5031_MODULE_INTERRUPTS	0x11

/* Slave 3 (i2c address 0x4b) */
#define TWL4030_MODULE_BACKUP		0x12
#define TWL4030_MODULE_INT		0x13
#define TWL4030_MODULE_PM_MASTER	0x14
#define TWL4030_MODULE_PM_RECEIVER	0x15
#define TWL4030_MODULE_RTC		0x16
#define TWL4030_MODULE_SECURED_REG	0x17
#define TWL6030_MODULE_SLAVE_RES	0x19

#define TWL_MODULE_USB		TWL4030_MODULE_USB
#define TWL_MODULE_AUDIO_VOICE	TWL4030_MODULE_AUDIO_VOICE
#define TWL_MODULE_PIH		TWL4030_MODULE_PIH
#define TWL_MODULE_MADC		TWL4030_MODULE_MADC
#define TWL_MODULE_MAIN_CHARGE	TWL4030_MODULE_MAIN_CHARGE
#define TWL_MODULE_PM_MASTER	TWL4030_MODULE_PM_MASTER
#define TWL_MODULE_PM_RECEIVER	TWL4030_MODULE_PM_RECEIVER
#define TWL_MODULE_RTC		TWL4030_MODULE_RTC
#define TWL_MODULE_PWM		TWL4030_MODULE_PWM0
#define TWL_MODULE_PM_SLAVE_RES	TWL6030_MODULE_SLAVE_RES
#define TWL6030_MODULE_CHARGER	TWL4030_MODULE_MAIN_CHARGE
#define TWL6032_MODULE_CHARGER	0x18

#define TWL6030_MODULE_GASGAUGE 0x0B
#define TWL6030_MODULE_ID0	0x0D
#define TWL6030_MODULE_ID1	0x0E
#define TWL6030_MODULE_ID2	0x0F

/* TWL6030 INT register offsets */
#define REG_INT_STS_A			0x00
#define REG_INT_STS_B			0x01
#define REG_INT_STS_C			0x02

#define REG_INT_MSK_LINE_A		0x03
#define REG_INT_MSK_LINE_B		0x04
#define REG_INT_MSK_LINE_C		0x05

#define REG_INT_MSK_STS_A		0x06
#define REG_INT_MSK_STS_B		0x07
#define REG_INT_MSK_STS_C		0x08

/* MASK INT REG GROUP A */
#define TWL6030_PWR_INT_MASK 		0x07
#define TWL6030_RTC_INT_MASK 		0x18
#define TWL6030_HOTDIE_INT_MASK 	0x20
#define TWL6030_SMPSLDOA_INT_MASK	0xC0

/* MASK INT REG GROUP B */
#define TWL6030_SMPSLDOB_INT_MASK 	0x01
#define TWL6030_BATDETECT_INT_MASK 	0x02
#define TWL6030_SIMDETECT_INT_MASK 	0x04
#define TWL6030_MMCDETECT_INT_MASK 	0x08
#define TWL6030_GPADC_INT_MASK 		0x60
#define TWL6030_GASGAUGE_INT_MASK 	0x80

/* MASK INT REG GROUP C */
#define TWL6030_USBOTG_INT_MASK  	0x0F
#define TWL6030_CHARGER_CTRL_INT_MASK 	0x10
#define TWL6030_CHARGER_FAULT_INT_MASK 	0x60

/*----------------------------------------------------------------------*/

#if defined(CONFIG_SPL_BUILD)

/* TWL6030 regulators */
#define TWL6030_VAUX1	0x00
#define TWL6030_VMMC	0x01

/* TWL6032 regulators */
#define TWL6032_LDO2	0x00
#define TWL6032_LDO5	0x01

#else /* !CONFIG_SPL_BUILD */

/* TWL6030 regulators */
#define TWL6030_VAUX1	0x00
#define TWL6030_VAUX2	0x01
#define TWL6030_VAUX3	0x02
#define TWL6030_VMMC	0x03
#define TWL6030_VPP		0x04
#define TWL6030_VUSIM	0x05
#define TWL6030_VANA	0x06
#define TWL6030_VCXIO	0x07
#define TWL6030_VDAC	0x08
#define TWL6030_VUSB	0x09
#define TWL6030_CLK32KG	0x0A
#define TWL6030_CLK32KAUDIO	0x0B

/* TWL6032 regulators */
#define TWL6032_LDO1	0x00
#define TWL6032_LDO2	0x01
#define TWL6032_LDO3	0x02
#define TWL6032_LDO4	0x03
#define TWL6032_LDO5	0x04
#define TWL6032_LDO6	0x05
#define TWL6032_LDO7	0x06
#define TWL6032_LDOLN	0x07
#define TWL6032_LDOUSB	0x08
#define TWL6032_CLK32KG	0x09
#define TWL6032_CLK32KAUDIO	0x0A

#endif /* CONFIG_SPL_BUILD */

/*----------------------------------------------------------------------*/

extern u8 twl6032_eprom_rev;
extern u8 twl6032_rev;
extern u8 twl6040_rev;

/*----------------------------------------------------------------------*/

/* Platform Dependent Power Initialisations */
void power_init(void);
void power_shutdown(void);

/* register in offset */
int twl_i2c_write(u8 mod_no, u8 *value, u8 reg, int num_bytes);
int twl_i2c_read(u8 mod_no, u8 *value, u8 reg, int num_bytes);
int twl_i2c_write_u8(u8 mod_no, u8 value, u8 reg);
int twl_i2c_read_u8(u8 mod_no, u8 *value, u8 reg);
int twl_clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg);

void twl6030_poweroff(void) __attribute__ ((noreturn));

void twl6030_interrupt_unmask(u8 bit_mask, u8 offset);
void twl6030_interrupt_mask(u8 bit_mask, u8 offset);
void twl6030_interrupt_clear(u8 bit_mask, u8 offset);
void twl6030_clear_interrupts(void);

/*----------------------------------------------------------------------*/

struct twl6030_usb_data {
	void (*phy_init)(void);
	void (*phy_exit)(void);
	/* Power on/off the PHY */
	void (*phy_power)(int ID, int on);
#if 0
	/* Suspend/resume of phy */
	void (*phy_suspend)(int suspend);
#endif
};

void twl6030_usb_init(struct twl6030_usb_data *pdata);
void twl6030_usb_shutdown(void);

/*----------------------------------------------------------------------*/

#define TWL6032_GPADC_CHANNEL_BATTERY_TYPE	0
#define TWL6032_GPADC_CHANNEL_BAT_TEMP		1
#define TWL6032_GPADC_CHANNEL_VSYS			7
#define TWL6032_GPADC_CHANNEL_BK_VBAT		8
#define TWL6032_GPADC_CHANNEL_VAC			9
#define TWL6032_GPADC_CHANNEL_VBUS			10
#define TWL6032_GPADC_CHANNEL_CCURRENT		17
#define TWL6032_GPADC_CHANNEL_VBAT			18

/*
 * raw_code - raw adc value
 * raw_channel_value - adc * channel gain
 * code - calibrated adc value
 */
struct twl6030_value {
	int raw_code;
	int raw_channel_value;
	int code;
};

struct twl6030_gpadc_request {
	u8 channel;
	int rbuf;
	struct twl6030_value buf;
};

int twl6030_gpadc_conversion(struct twl6030_gpadc_request *req);
void twl6032_gpadc_init(void);
void twl6032_gpadc_shutdown(void);

/*----------------------------------------------------------------------*/

void twl6032_charger_init(void);
void twl6032_charger_shutdown(void);
int twl6032_get_battery_voltage(void);
int twl_charger_get_battery_voltage(void);
void twl_charger_backup_battery_setup(void);

/*----------------------------------------------------------------------*/

void twl6040_vib_init(void);
void twl6040_vib_shutdown(void);

/*----------------------------------------------------------------------*/

struct twl6030_pwm {
	/**
	 *	id 0: TWL6030 PWM1
	 *	id 1: TWL6030 PWM2
	 */
	unsigned	id;
	/**
	 * TWL6030 PWM1/PWM2 clock rate:
	 *     * 128:	256Hz
	 *     * 64:	512Hz
	 */
	int	clock_cycle;
	/**
	 * Invert/Reverse the duty cycle:
	 *   output duty cycle = 100 - current duty cycle
	 */
	int	invert_duty_cycle;
	int	max_duty_cycle;
	int	min_duty_cycle;
#ifdef CONFIG_TWL6030_PWM_CHECK_AC_CHARGER
	int charger_max_duty_cycle;
	int charger_min_duty_cycle;
#endif

	void (*platform_init)(void);
	void (*platform_enable)(void);
	void (*platform_disable)(void);

	/* Internal Use */
	u8	is_enabled;
	u8	duty_cycle_range;
	u8	pwmoff_level;		/* the fixed PWMOFF value */
	u8	pwmon_max_level;	/* the maximal PWMON value */
	u8	pwmon_min_level;	/* the minimal PWMON value */
	int	pwmon_shift;		/* the shift value for PWMON interval */
	int	pwmon_interval;		/* the PWMON interval value */
	int	curr_pwmon_level;	/* current PWMON value */
	int	curr_duty_cycle;
#ifdef CONFIG_TWL6030_PWM_CHECK_AC_CHARGER
	int no_charger_max_duty_cycle;  /* Internal Use */
	int no_charger_min_duty_cycle;  /* Internal Use */
#endif
};

void twl6030_pwm_init(struct twl6030_pwm *pwms, int num);
void twl6030_pwm_shutdown(void);
void twl6030_pwm_set_duty(unsigned id, unsigned int duty_cycle);

/*----------------------------------------------------------------------*/

#endif
