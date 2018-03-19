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

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <asm/gpio.h>

#include <twl6030.h>

#include <icom/vibrator.h>

/*----------------------------------------------------------------------*/

/*#define CONFIG_TWL6040_VIB_DEBUG*/
/*#define CONFIG_TWL6040_VIB_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL6040_VIB_DEBUG
#define CONFIG_TWL6040_VIB_DEBUG
#endif
#ifndef CONFIG_TWL6040_VIB_VERBOSE_DEBUG
#define CONFIG_TWL6040_VIB_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6040_VIB_DEBUG
#define VIB_DPRINT(fmt, args...) \
	do {printf("[twl-vib] " fmt, ##args);} while (0)
#define VIB_DPUTS(fmt) \
	do {puts("[twl-vib] " fmt);} while (0)
#else /* CONFIG_TWL6040_VIB_DEBUG */
#define VIB_DPRINT(fmt, args...) \
	do {} while (0)
#define VIB_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6040_VIB_DEBUG */

#ifdef CONFIG_TWL6040_VIB_VERBOSE_DEBUG
#define VIB_VPRINT(fmt, args...) \
	do {printf("[twl-vib] " fmt, ##args);} while (0)
#define VIB_VPUTS(fmt) \
	do {puts("[twl-vib] " fmt);} while (0)
#else /* CONFIG_TWL6040_VIB_VERBOSE_DEBUG */
#define VIB_VPRINT(fmt, args...) \
	do {} while (0)
#define VIB_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6040_VIB_VERBOSE_DEBUG */

#define VIB_PRINT(fmt, args...) \
	do {printf("twl-vib: " fmt, ##args);} while (0)
#define VIB_PUTS(fmt) \
	do {puts("twl-vib: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define VIB_ERR(fmt, args...) \
	do {printf("twl-vib err:" fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

#define TWL6040_REG_ASICID		0x01
#define TWL6040_REG_ASICREV		0x02
#define TWL6040_REG_INTID		0x03
#define		TWL6040_THINT			0x01
#define		TWL6040_PLUGINT			0x02
#define		TWL6040_UNPLUGINT		0x04
#define		TWL6040_HOOKINT			0x08
#define		TWL6040_HFINT			0x10
#define		TWL6040_VIBINT			0x20
#define		TWL6040_READYINT		0x40
#define TWL6040_REG_INTMR		0x04
#define TWL6040_REG_NCPCTL		0x05
#define		TWL6040_NCPENA			0x01
#define		TWL6040_NCPOPEN			0x40
#define		TWL6040_TSHUTENA		0x80
#define TWL6040_REG_LDOCTL		0x06
#define		TWL6040_LSLDOENA		0x01
#define		TWL6040_HSLDOENA		0x04
#define		TWL6040_REFENA			0x40
#define		TWL6040_OSCENA			0x80
#define TWL6040_REG_HPPLLCTL	0x07
#define		TWL6040_HPLLENA			0x01
#define		TWL6040_HPLLRST			0x02
#define		TWL6040_HPLLBP			0x04
#define		TWL6040_HPLLSQRENA		0x08
#define		TWL6040_HPLLSQRBP		0x10
#define		TWL6040_MCLK_12000KHZ	(0 << 5)
#define		TWL6040_MCLK_19200KHZ	(1 << 5)
#define		TWL6040_MCLK_26000KHZ	(2 << 5)
#define		TWL6040_MCLK_38400KHZ	(3 << 5)
#define		TWL6040_MCLK_MSK		0x60
#define TWL6040_REG_LPPLLCTL	0x08
#define		TWL6040_LPLLENA			0x01
#define		TWL6040_LPLLRST			0x02
#define		TWL6040_LPLLSEL			0x04
#define		TWL6040_LPLLFIN			0x08
#define		TWL6040_HPLLSEL			0x10

#define TWL6040_REG_VIBCTLL		0x18
#define TWL6040_REG_VIBDATL		0x19
#define TWL6040_REG_VIBCTLR		0x1A
#define TWL6040_REG_VIBDATR		0x1B

#define TWL6040_REG_ACCCTL		0x2D
#define		TWL6040_I2CSEL			0x01
#define		TWL6040_RESETSPLIT		0x04
#define		TWL6040_INTCLRMODE		0x08
#define		TWL6040_CLK32KSEL		0x40
#define TWL6040_REG_STATUS		0x2E

/*----------------------------------------------------------------------*/

/* VIBCTLL/R (0x18/0x1A) fields */
#define TWL6040_VIBENA			(1 << 0)
#define TWL6040_VIBSEL			(1 << 1)
#define TWL6040_VIBCTRL			(1 << 2)
#define TWL6040_VIBCTRL_P		(1 << 3)
#define TWL6040_VIBCTRL_N		(1 << 4)

/* VIBDATL/R (0x19/0x1B) fields */
#define TWL6040_VIBDAT_MAX		0x64

/* STATUS (0x2E) fields */
#define TWL6040_PLUGCOMP		0x02
#define TWL6040_HFLOCDET		0x04
#define TWL6040_HFROCDET		0x08
#define TWL6040_VIBLOCDET		0x10
#define TWL6040_VIBROCDET		0x20
#define TWL6040_TSHUTDET		0x40

/*----------------------------------------------------------------------*/

#define TWL6040_POWER_UP_TIME		20	/* ms */

/*----------------------------------------------------------------------*/

static u8 twl6040_reg_read(u8 reg)
{
	u8 data = 0;
#if defined(CONFIG_TWL6040_VIB_DEBUG) || defined(CONFIG_TWL6040_VIB_VERBOSE_DEBUG)
	int ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &data, reg);
	if (ret) {
		VIB_DPRINT("read(twl6040): [0x%02X] err %d\n", reg, ret);
	} else {
		VIB_VPRINT("read(twl6040): [0x%02X]=0x%02X\n", reg, data);
	}
#else
	twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &data, reg);
#endif /* CONFIG_TWL6040_VIB_VERBOSE_DEBUG */
	return data;
}

static inline int twl6040_reg_write(u8 reg, u8 val)
{
	VIB_VPRINT("write(twl6040): [0x%02X]=0x%02X\n", reg, val);
	return twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, reg);
}

static inline int twl6040_set_bits(u8 reg, u8 mask)
{
	VIB_VPRINT("set_bits(twl6040): [0x%02X] 0x%02X\n", reg, mask);
	return twl_clear_n_set(TWL_MODULE_AUDIO_VOICE, 0, mask, reg);
}

static inline int twl6040_clear_bits(u8 reg, u8 mask)
{
	VIB_VPRINT("clear_bits(twl6040): [0x%02X] 0x%02X\n", reg, mask);
	return twl_clear_n_set(TWL_MODULE_AUDIO_VOICE, mask, 0, reg);
}

static int twl6040_power(int enable)
{
	VIB_VPRINT("twl6040_power: %d\n", enable);

	if (enable) {
		ulong start;
		int round = 0;
		int retry = 0;
		u8 intid;
		u8 ncpctl;
		u8 ldoctl;
		u8 lppllctl;
		u8 ncpctl_exp;
		u8 ldoctl_exp;
		u8 lppllctl_exp;

		/* disable internal 32kHz oscillator */
		twl6040_clear_bits(TWL6040_REG_ACCCTL, TWL6040_CLK32KSEL);

		/* twl6040_power_up_completion() */

		/* NCPCTL expected value: NCP enabled */
		ncpctl_exp = (TWL6040_TSHUTENA | TWL6040_NCPENA);

		/* LDOCTL expected value: HS/LS LDOs and Reference enabled */
		ldoctl_exp = (TWL6040_REFENA | TWL6040_HSLDOENA | TWL6040_LSLDOENA);

		/* LPPLLCTL expected value: Low-Power PLL enabled */
		lppllctl_exp = TWL6040_LPLLENA;

		do {
			gpio_direction_output(CONFIG_TWL6040_POWERON_GPIO, 1);
			mdelay(TWL6040_POWER_UP_TIME);

			start = get_timer(0);
			do {
				intid = twl6040_reg_read(TWL6040_REG_INTID);
				if (intid & TWL6040_READYINT)
					break;
			} while ((get_timer(0) - start) <= 700);

			if (!(intid & TWL6040_READYINT)) {
				VIB_ERR("powering timeout 0x%02X\n", intid);
				return -ETIMEDOUT;
			}

			/*
			 * Power on seemingly completed.
			 * Look for clues that the twl6040 might be still booting.
			 */

			retry = 0;
			ncpctl = twl6040_reg_read(TWL6040_REG_NCPCTL);
			if (ncpctl != ncpctl_exp)
				retry++;

			ldoctl = twl6040_reg_read(TWL6040_REG_LDOCTL);
			if (ldoctl != ldoctl_exp)
				retry++;

			lppllctl = twl6040_reg_read(TWL6040_REG_LPPLLCTL);
			if (lppllctl != lppllctl_exp)
				retry++;

			if (retry) {
				VIB_DPRINT("NCPCTL: 0x%02x (should be 0x%02x)\n", ncpctl, ncpctl_exp);
				VIB_DPRINT("LDOCTL: 0x%02x (should be 0x%02x)\n", ldoctl, ldoctl_exp);
				VIB_DPRINT("LPLLCTL: 0x%02x (should be 0x%02x)\n", lppllctl, lppllctl_exp);
				round++;
				gpio_set_value(CONFIG_TWL6040_POWERON_GPIO, 0);
				udelay(1500);
				continue;
			}
		} while (round && (round < 3));

		if (round >= 3) {
			VIB_ERR("power on err\n");
			return -EIO;
		}

		/* Errata: PDMCLK can fail to generate at cold temperatures
		 * The workaround consists of resetting HPPLL and LPPLL
		 * after Sleep/Deep-Sleep mode and before application mode.
		 */
		twl6040_set_bits(TWL6040_REG_HPPLLCTL, TWL6040_HPLLRST);
		twl6040_clear_bits(TWL6040_REG_HPPLLCTL, TWL6040_HPLLRST);
		twl6040_set_bits(TWL6040_REG_LPPLLCTL, TWL6040_LPLLRST);
		twl6040_clear_bits(TWL6040_REG_LPPLLCTL, TWL6040_LPLLRST);
	} else {
		/* use AUDPWRON line */
		gpio_direction_output(CONFIG_TWL6040_POWERON_GPIO, 0);

		/* power-down sequence latency */
		udelay(500);

		/* enable internal 32kHz oscillator */
		twl6040_set_bits(TWL6040_REG_ACCCTL, TWL6040_CLK32KSEL);
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int twl6040_vib_on(void)
{
	int ret;
#if defined(CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED)
#if (CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED > 0x64) || (CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED <= 0)
#error "Invalid CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED (0x0-0x64)"
#else
	const u8 speed = CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED;
#endif
#else
	const u8 speed = 0x32;
#endif /* CONFIG_TWL6040_VIBRATOR_VOLTAGE_RAISE_SPEED */

	ret = twl6040_power(1);
	if (ret)
		return ret;

#if 0
	if (speed > TWL6040_VIBDAT_MAX)
		speed = TWL6040_VIBDAT_MAX;
#endif

	/* 0x01~0x63: Differential output increase by 50mV per LSB (n*50mV) */
	twl6040_reg_write(TWL6040_REG_VIBDATL, speed);
	/* 0x01~0x63: Differential output increase by 50mV per LSB (n*50mV) */
	twl6040_reg_write(TWL6040_REG_VIBDATR, speed);

	/*
	 * ERRATA: Disable overcurrent protection for at least
	 * 2.5ms when enabling vibrator drivers to avoid false
	 * overcurrent detection
	 */
	twl6040_set_bits(TWL6040_REG_VIBCTLL,
			TWL6040_VIBENA | TWL6040_VIBCTRL_P);
	twl6040_set_bits(TWL6040_REG_VIBCTLR,
			TWL6040_VIBENA | TWL6040_VIBCTRL_P);

	mdelay(4);

	twl6040_clear_bits(TWL6040_REG_VIBCTLL, TWL6040_VIBCTRL_P);
	twl6040_clear_bits(TWL6040_REG_VIBCTLR, TWL6040_VIBCTRL_P);

	return 0;
}

static void twl6040_vib_off(void)
{
	twl6040_reg_write(TWL6040_REG_VIBDATL, 0x00);
	twl6040_reg_write(TWL6040_REG_VIBDATR, 0x00);

	twl6040_clear_bits(TWL6040_REG_VIBCTLL, TWL6040_VIBENA);
	twl6040_clear_bits(TWL6040_REG_VIBCTLR, TWL6040_VIBENA);

	twl6040_power(0);
}

/*----------------------------------------------------------------------*/

void vibrate(unsigned int ms)
{
	if (!twl6040_vib_on()) {
		unsigned long vib_timeout, vib_ticks = 0;
		u8 intid;

		VIB_DPRINT("vibrator on: %d ms\n", ms);

		UART_FIFO_FLUSH_ONCE();

		vib_timeout = timeout_init(ms, &vib_ticks);
		do {
			mdelay(5); /* 5ms */
			intid = twl6040_reg_read(TWL6040_REG_INTID);
			if (intid & TWL6040_VIBINT) {
				u8 status = twl6040_reg_read(TWL6040_REG_STATUS);
				if (status & TWL6040_VIBLOCDET) {
					VIB_PUTS("left overcurrent detected\n");
					twl6040_clear_bits(TWL6040_REG_VIBCTLL, TWL6040_VIBENA);
				}
				if (status & TWL6040_VIBROCDET) {
					VIB_PUTS("right overcurrent detected\n");
					twl6040_clear_bits(TWL6040_REG_VIBCTLR, TWL6040_VIBENA);
				}
				if (status & (TWL6040_VIBLOCDET | TWL6040_VIBROCDET))
					break;
			}
		} while (!timeout_check(&vib_timeout, &vib_ticks));

		twl6040_vib_off();

		VIB_DPUTS("vibrator off\n");

		UART_FIFO_FLUSH_ONCE();
	} else {
		twl6040_power(0);
	}
}

/*----------------------------------------------------------------------*/

void twl6040_vib_init(void)
{
	/* power on TWL6040 */
	twl6040_power(1);

	/* power off TWL6040 */
	twl6040_power(0);

	/* reset interrupts */
	twl6040_reg_read(TWL6040_REG_INTID);
}

void twl6040_vib_shutdown(void)
{
	/* power off TWL6040 */
	twl6040_power(0);

	/* reset interrupts */
	twl6040_reg_read(TWL6040_REG_INTID);
}

#ifdef CONFIG_TWL6040_VIB_VERBOSE_DEBUG
/* vibrate <ms> */
static int do_vibrating(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ms = 0;

	if (argc != 2) {
		puts("usage: vibrate <ms>\n");
		return 0;
	}

	ms = (int)simple_strtoul(argv[1], NULL, 10);
	if (ms > 0) {
		printf("start vibrating %d ms\n", ms);
		vibrate(ms);
	}

	return 0;
}

U_BOOT_CMD(
	vibrate, 2, 0, do_vibrating,
	"<ms>",
	""
);
#endif /* CONFIG_TWL6040_VIB_VERBOSE_DEBUG */
