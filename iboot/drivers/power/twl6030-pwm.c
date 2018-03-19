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
#include <linux/notifier.h>
#include <asm/gpio.h>

#include <twl6030.h>

#include <icom/pwm.h>
#include <icom/power_supply.h>

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*#define CONFIG_TWL6030_PWM_DEBUG*/
/*#define CONFIG_TWL6030_PWM_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL6030_PWM_DEBUG
#define CONFIG_TWL6030_PWM_DEBUG
#endif
#ifndef CONFIG_TWL6030_PWM_VERBOSE_DEBUG
#define CONFIG_TWL6030_PWM_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6030_PWM_DEBUG
#define PWM_DPRINT(fmt, args...) \
	do {printf("*twl-pwm" fmt, ##args);} while (0)
#define PWM_DPUTS(fmt) \
	do {puts("*twl-pwm" fmt);} while (0)
#else /* CONFIG_TWL6030_PWM_DEBUG */
#define PWM_DPRINT(fmt, args...) \
	do {} while (0)
#define PWM_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_PWM_DEBUG */

#ifdef CONFIG_TWL6030_PWM_VERBOSE_DEBUG
#define PWM_VPRINT(fmt, args...) \
	do {printf("**twl-pwm" fmt, ##args);} while (0)
#define PWM_VPUTS(fmt) \
	do {puts("**twl-pwm" fmt);} while (0)
#else /* CONFIG_TWL6030_PWM_VERBOSE_DEBUG */
#define PWM_VPRINT(fmt, args...) \
	do {} while (0)
#define PWM_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_PWM_VERBOSE_DEBUG */

#define PWM_PRINT(fmt, args...) \
	do {printf("twl-pwm" fmt, ##args);} while (0)
#define PWM_PUTS(fmt) \
	do {puts("twl-pwm" fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)

/*----------------------------------------------------------------------*/

#define TWL6030_MAX_PWMS				(2)

#define TWL6030_PWM1_ID	0
#define TWL6030_PWM2_ID	1

#define	PWM_128_INTERVAL_SHIFT		3
#define	PWM_64_INTERVAL_SHIFT		(PWM_128_INTERVAL_SHIFT - 1)

#define TWL_PWM_MAX_DUTY_CYCLE			100
#define TWL_PWM_LED_MAX_BRIGHTNESS		255

/*----------------------------------------------------------------------*/

/* TOGGLE3 */
#define REG_TOGGLE3			0x92
#define		TOGGLE3_PWM2EN		(0x01 << 5)
#define		TOGGLE3_PWM2S		(0x01 << 4)
#define		TOGGLE3_PWM2R		(0x01 << 3)
#define		TOGGLE3_PWM2_MASK	(0x07 << 3)
#define		TOGGLE3_PWM1EN		(0x01 << 2)
#define		TOGGLE3_PWM1S		(0x01 << 1)
#define		TOGGLE3_PWM1R		(0x01 << 0)
#define		TOGGLE3_PWM1_MASK	(0x07 << 0)
/* PWM1 */
#define REG_PWM1_ON			0x00
#define REG_PWM1_OFF		0x01
/* PWM2 */
#define REG_PWM2_ON			0x03
#define REG_PWM2_OFF		0x04

/*----------------------------------------------------------------------*/

static u8 twl6030_toggle3_value __attribute__ ((section (".data"))) = 0;

static const struct twl6030_pwm_reg {
	u8			enable_mod;		/* TWL module for PWM enable */
	u8			enable_reg;		/* TWL register for PWM enable */
	u8			enable_mask;	/* PWM mask value */
	u8			enable_clock;	/* PWM clock */
	u8			enable_set;		/* PWM set driver signal */
	u8			enable_reset;	/* PWM reset driver signal */

	u8			mod;		/* TWL module for PWM on/off */
	u8			on_reg;		/* TWL register for PWM on */
	u8			off_reg;	/* TWL register for PWM off */
} twl6030_pwm_reg_map[TWL6030_MAX_PWMS] = {
	[TWL6030_PWM1_ID] = {	/* PWM1 */
		.enable_mod		= TWL6030_MODULE_ID1,
		.enable_reg		= REG_TOGGLE3,
		.enable_mask	= TOGGLE3_PWM1_MASK,
		.enable_clock	= TOGGLE3_PWM1EN,
		.enable_set		= TOGGLE3_PWM1S,
		.enable_reset	= TOGGLE3_PWM1R,

		.mod		= TWL_MODULE_PWM,
		.on_reg		= REG_PWM1_ON,
		.off_reg	= REG_PWM1_OFF,
	},
	[TWL6030_PWM2_ID] = {	/* PWM2 */
		.enable_mod		= TWL6030_MODULE_ID1,
		.enable_reg		= REG_TOGGLE3,
		.enable_mask	= TOGGLE3_PWM2_MASK,
		.enable_clock	= TOGGLE3_PWM2EN,
		.enable_set		= TOGGLE3_PWM2S,
		.enable_reset	= TOGGLE3_PWM2R,

		.mod		= TWL_MODULE_PWM,
		.on_reg		= REG_PWM2_ON,
		.off_reg	= REG_PWM2_OFF,
	},
};

static struct twl6030_pwm *twl6030_pwms[TWL6030_MAX_PWMS] __attribute__((section (".data"))) = {
	[TWL6030_PWM1_ID] = NULL,	/* PWM1 */
	[TWL6030_PWM2_ID] = NULL,	/* PWM2 */
};

#if defined(CONFIG_TWL6030_PWM_CHECK_AC_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
static unsigned pwm_ac_charger_type __attribute__ ((section (".data"))) = POWER_SUPPLY_TYPE_UNKNOWN;
static struct notifier_block pwm_ac_notifier;
#endif /* CONFIG_TWL6030_PWM_CHECK_AC_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

/*----------------------------------------------------------------------*/

static void twl6030_pwm_change(struct twl6030_pwm *pwm, u8 duty_cycle, int interval)
{
	unsigned id = pwm->id;
	u8 old_duty_cycle = duty_cycle;
	/* 128 (default value: bit 7 at 0) or 64 (bit 7 at 1) */
	u8 pwm_length = (pwm->clock_cycle == 128) ? 0x0 : 0x80;
	u8 target_level = 0;
	u8 curr_level;

	if (duty_cycle > TWL_PWM_MAX_DUTY_CYCLE)
		duty_cycle = TWL_PWM_MAX_DUTY_CYCLE;

	if (pwm->duty_cycle_range != TWL_PWM_MAX_DUTY_CYCLE)
		duty_cycle = pwm->min_duty_cycle +
				DIV_ROUND_CLOSEST(duty_cycle * pwm->duty_cycle_range, TWL_PWM_MAX_DUTY_CYCLE);

	if (pwm->invert_duty_cycle) {
		/* Invert the duty cycle */
		duty_cycle = TWL_PWM_MAX_DUTY_CYCLE - duty_cycle;
		if (duty_cycle > (TWL_PWM_MAX_DUTY_CYCLE - pwm->min_duty_cycle))
			duty_cycle = TWL_PWM_MAX_DUTY_CYCLE - pwm->min_duty_cycle;
		else if (duty_cycle < (TWL_PWM_MAX_DUTY_CYCLE - pwm->max_duty_cycle))
			duty_cycle = TWL_PWM_MAX_DUTY_CYCLE - pwm->max_duty_cycle;
	} else {
		if (duty_cycle > pwm->max_duty_cycle)
			duty_cycle = pwm->max_duty_cycle;
		else if (duty_cycle < pwm->min_duty_cycle)
			duty_cycle = pwm->min_duty_cycle;
	}

	if (pwm->is_enabled)
		PWM_PRINT("%u: duty cycle (%d%%=>%d%%)\n", id+1, old_duty_cycle, duty_cycle);

	curr_level = pwm->curr_pwmon_level;

	if (duty_cycle) {
#ifdef CONFIG_TWL6030_PWM_DEBUG
		if (pwm->invert_duty_cycle) {
			/* Invert the duty cycle */
			old_duty_cycle = TWL_PWM_MAX_DUTY_CYCLE - duty_cycle;
		} else {
			old_duty_cycle = duty_cycle;
		}
		if (pwm->duty_cycle_range != TWL_PWM_MAX_DUTY_CYCLE) {
			if (old_duty_cycle > pwm->min_duty_cycle)
				old_duty_cycle -= pwm->min_duty_cycle;
			else
				old_duty_cycle = pwm->min_duty_cycle;
			old_duty_cycle = DIV_ROUND_CLOSEST(old_duty_cycle * TWL_PWM_MAX_DUTY_CYCLE, pwm->duty_cycle_range);
		}
		PWM_DPRINT("%u: brightness %d (1-%d)\n", id+1,
				DIV_ROUND_CLOSEST(old_duty_cycle * TWL_PWM_LED_MAX_BRIGHTNESS, TWL_PWM_MAX_DUTY_CYCLE),
				TWL_PWM_LED_MAX_BRIGHTNESS);
#endif /* CONFIG_TWL6030_PWM_DEBUG */
		/* PWM1 & PWM2 */
		if (!pwm->invert_duty_cycle && duty_cycle > (TWL_PWM_MAX_DUTY_CYCLE - 1)) {
			target_level = pwm->pwmoff_level;
			pwm->curr_pwmon_level = pwm->pwmon_max_level;
		} else {
			target_level = (u8)(pwm->clock_cycle * duty_cycle / TWL_PWM_MAX_DUTY_CYCLE);
			target_level = pwm->pwmoff_level - target_level;

			if (target_level < pwm->pwmon_min_level)
				target_level = pwm->pwmon_min_level;
			else if (target_level > pwm->pwmon_max_level)
				target_level = pwm->pwmon_max_level;
			pwm->curr_pwmon_level = target_level;
		}
	} else if (pwm->invert_duty_cycle) {
		target_level = pwm->pwmon_max_level;
		pwm->curr_pwmon_level = target_level;
	} else {
		target_level = pwm->pwmon_min_level;
		pwm->curr_pwmon_level = target_level;
	}

	if (interval) {
		int i;

		/* Apply the interval change */
		if (curr_level > pwm->curr_pwmon_level) {
			i = (curr_level - pwm->curr_pwmon_level) >> pwm->pwmon_shift;
			while (i > 0) {
				i--;
				curr_level -= pwm->pwmon_interval;
				twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
						curr_level | pwm_length, twl6030_pwm_reg_map[id].on_reg);
				PWM_VPRINT("%u: PWMON=0x%02X\n", id+1, curr_level | pwm_length);
#if defined(CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME) && (CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME > 0)
				/* delay to wait for the change */
				if (curr_level != target_level)
					mdelay(CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME);
#endif /* CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME */
			}
		} else {
			i = (pwm->curr_pwmon_level - curr_level) >> pwm->pwmon_shift;
			while (i > 0) {
				i--;
				curr_level += pwm->pwmon_interval;
				twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
						curr_level | pwm_length, twl6030_pwm_reg_map[id].on_reg);
				PWM_VPRINT("%u: PWMON=0x%02X\n", id+1, curr_level | pwm_length);
#if defined(CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME) && (CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME > 0)
				/* delay to wait for the change */
				if (curr_level != target_level)
					mdelay(CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME);
#endif /* CONFIG_TWL6030_PWM_INTERVAL_CHANGE_TIME */
			}
		}
	}

	/* PWM_ON value */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
				target_level | pwm_length, twl6030_pwm_reg_map[id].on_reg);
	PWM_VPRINT("%u: PWMON=0x%02X\n", id+1, target_level | pwm_length);
}

static void twl6030_pwm_config(struct twl6030_pwm* pwm)
{
	twl6030_pwms[pwm->id] = pwm;

	pwm->is_enabled = 0;
	pwm->curr_duty_cycle = 0;

	if (pwm->clock_cycle == 128) {
		/* PWM1 & PWM2 */
		pwm->pwmoff_level		= 0x7F;
		pwm->pwmon_max_level	= 0x7E;
		pwm->pwmon_min_level	= 0x01;
		pwm->pwmon_shift		= PWM_128_INTERVAL_SHIFT;
		PWM_VPRINT("%u: 128 clock cycle (256Hz)\n", pwm->id+1);
	} else {
		/* PWM1 & PWM2 */
		pwm->pwmoff_level		= 0x3F;
		pwm->pwmon_max_level	= 0x3E; /* no PWM_LENGTH bit */
		pwm->pwmon_min_level	= 0x01; /* no PWM_LENGTH bit */
		pwm->pwmon_shift		= PWM_64_INTERVAL_SHIFT;
		PWM_VPRINT("%u: 64 clock cycle (512Hz)\n", pwm->id+1);
	}
	pwm->pwmon_interval = 1 << pwm->pwmon_shift;
	pwm->curr_pwmon_level = pwm->pwmon_min_level;

	if (pwm->max_duty_cycle > TWL_PWM_MAX_DUTY_CYCLE || pwm->max_duty_cycle <= 0)
		pwm->max_duty_cycle = TWL_PWM_MAX_DUTY_CYCLE;

	if (pwm->min_duty_cycle >= TWL_PWM_MAX_DUTY_CYCLE || pwm->min_duty_cycle < 0
			|| pwm->min_duty_cycle >= pwm->max_duty_cycle)
		pwm->min_duty_cycle = 0;

#ifdef CONFIG_TWL6030_PWM_CHECK_AC_CHARGER
	if (pwm->charger_max_duty_cycle > 0 && pwm->charger_min_duty_cycle >= 0) {
		pwm->no_charger_max_duty_cycle = pwm->max_duty_cycle;
		if (pwm->charger_max_duty_cycle > TWL_PWM_MAX_DUTY_CYCLE)
			pwm->charger_max_duty_cycle = TWL_PWM_MAX_DUTY_CYCLE;

		pwm->no_charger_min_duty_cycle = pwm->min_duty_cycle;
		if (pwm->charger_min_duty_cycle >= TWL_PWM_MAX_DUTY_CYCLE
				|| pwm->charger_min_duty_cycle >= pwm->charger_max_duty_cycle)
			pwm->charger_min_duty_cycle = 0;
	} else {
		pwm->charger_max_duty_cycle = 0; /* Disable this feature */
	}
#endif /* CONFIG_TWL6030_PWM_CHECK_AC_CHARGER */

	pwm->duty_cycle_range = pwm->max_duty_cycle - pwm->min_duty_cycle;
}

static void twl6030_pwm_enable(struct twl6030_pwm *pwm)
{
	unsigned id = pwm->id;

	if (!pwm->is_enabled) {
		PWM_PRINT("%u: turn on PWM\n", id+1);

		twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
				pwm->pwmoff_level, twl6030_pwm_reg_map[id].off_reg);
		/* PWM_OFF value */
		PWM_VPRINT("%u: PWMOFF=0x%02X\n", id+1, pwm->pwmoff_level);

		/* The clock inputs for generation of PWM signals are enabled */
		twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_mask;
		twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_clock;
		PWM_VPRINT("%u: enable input clock: 0x%02X\n", id+1, twl6030_toggle3_value);
		twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
				twl6030_toggle3_value,
				twl6030_pwm_reg_map[id].enable_reg);

		/* Setup the minimal duty cycle: 1% */
		twl6030_pwm_change(pwm, 1, 0);

		/* Start the PWM signal generation */
		twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_set;
		PWM_VPRINT("%u: start clock: 0x%02X\n", id+1, twl6030_toggle3_value);
		twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
				twl6030_toggle3_value,
				twl6030_pwm_reg_map[id].enable_reg);

		if (pwm->platform_enable) {
			PWM_VPRINT("%u: platform_enable\n", id+1);
			pwm->platform_enable();
		}

		pwm->is_enabled = 1;
	}
}

static void twl6030_pwm_disable(struct twl6030_pwm *pwm)
{
	unsigned id = pwm->id;

	PWM_PRINT("%u: turn off PWM\n", id+1);

	pwm->is_enabled = 0;

	if (pwm->platform_disable) {
		PWM_VPRINT("%u: platform_disable\n", id+1);
		pwm->platform_disable();
	}

	/* Stop the PWM signal generation */
	twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_set;
	twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_reset;
	PWM_VPRINT("%u: stop clock: 0x%02X\n", id+1, twl6030_toggle3_value);
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
			twl6030_toggle3_value,
			twl6030_pwm_reg_map[id].enable_reg);

	/* The clock inputs for generation of PWM signals are disabled */
	twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_mask;
	PWM_VPRINT("%u: disable input clock: 0x%02X\n", id+1, twl6030_toggle3_value);
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
			twl6030_toggle3_value,
			twl6030_pwm_reg_map[id].enable_reg);

	/* PWM_OFF value */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod, pwm->pwmoff_level + 1, twl6030_pwm_reg_map[id].off_reg);
	PWM_VPRINT("%u: PWMOFF=0x%02X\n", id+1, pwm->pwmoff_level + 1);

	/* PWM_ON value */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod, 0, twl6030_pwm_reg_map[id].on_reg);
	PWM_VPRINT("%u: PWMON=0x0\n", id+1);
}

/*----------------------------------------------------------------------*/

#if defined(CONFIG_TWL6030_PWM_CHECK_AC_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
static void twl6030_pwm_sync_charger(int charger)
{
	int i;
	struct twl6030_pwm *pwm;

	for (i = 0 ; i < TWL6030_MAX_PWMS ; i++) {
		pwm = twl6030_pwms[i];
		if (pwm && (pwm->charger_max_duty_cycle > 0)) {
			PWM_PRINT("%u: charger status %d\n", pwm->id+1, charger);
			if (charger) {
				/* apply the new duty cycle limits for the charger case */
				pwm->max_duty_cycle = pwm->charger_max_duty_cycle;
				pwm->min_duty_cycle = pwm->charger_min_duty_cycle;
			} else {
				/* apply the new duty cycle limits for the battery only case */
				pwm->max_duty_cycle = pwm->no_charger_max_duty_cycle;
				pwm->min_duty_cycle = pwm->no_charger_min_duty_cycle;
			}
			/* the range of duty cycle */
			pwm->duty_cycle_range = pwm->max_duty_cycle - pwm->min_duty_cycle;

			if (pwm->curr_duty_cycle && pwm->is_enabled)
				twl6030_pwm_change(pwm, pwm->curr_duty_cycle, 0);
		}
	}
}

static int twl6030_pwm_ac_event_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
   	switch (event) {
   	case POWER_SUPPLY_AC_EVENT_CHARGER:
		pwm_ac_charger_type = POWER_SUPPLY_TYPE_MAINS;
		twl6030_pwm_sync_charger(1);
       	break;
   	case POWER_SUPPLY_AC_EVENT_NONE:
	default:
		pwm_ac_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		twl6030_pwm_sync_charger(0);
       	break;
   	}

	return 0;
}
#endif /* CONFIG_TWL6030_PWM_CHECK_AC_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */

/*----------------------------------------------------------------------*/

void pwm_set_duty(unsigned id, unsigned int duty_cycle)
{
	if (id < TWL6030_MAX_PWMS && twl6030_pwms[id] != NULL) {
		struct twl6030_pwm *pwm = twl6030_pwms[id];

		pwm->curr_duty_cycle = duty_cycle;

		if (duty_cycle > 0) {
			/* Enable PWM first */
			twl6030_pwm_enable(pwm);

			twl6030_pwm_change(pwm, duty_cycle, 1);
		} else {
			/* Disable PWM */
			twl6030_pwm_disable(pwm);
		}
	}
}

/*----------------------------------------------------------------------*/

void twl6030_pwm_init(struct twl6030_pwm *pwms, int num)
{
	if (pwms) {
		struct twl6030_pwm *pwm;
		int has_pwm = 0;
		int i;

		for (i = 0 ; i < num ; i++) {
			pwm = pwms++;
			if (pwm && pwm->id < TWL6030_MAX_PWMS) {
				twl6030_pwm_config(pwm);

				if (pwm->platform_init) {
					PWM_VPRINT("%u: platform_init\n", pwm->id+1);
					pwm->platform_init();
				}

				/* Force to turn off PWM */
				twl6030_pwm_disable(pwm);

				has_pwm = 1;
			}
		}

#if defined(CONFIG_TWL6030_PWM_CHECK_AC_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
		if (has_pwm) {
			pwm_ac_notifier.notifier_call = twl6030_pwm_ac_event_notifier_call;
			i = ac_charger_register_notifier(&pwm_ac_notifier);
			if (i)
				PWM_PRINT(": AC event err %d\n", i);
		}
#endif /* CONFIG_TWL6030_PWM_CHECK_AC_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	}
}

void twl6030_pwm_shutdown(void)
{
	int i;
	struct twl6030_pwm *pwm;

	for (i = 0 ; i < TWL6030_MAX_PWMS ; i++) {
		pwm = twl6030_pwms[i];
		if (pwm && pwm->is_enabled)
			twl6030_pwm_disable(pwm);
	}
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_TWL6030_PWM_DEBUG
int do_twl6030_pwm(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 3) {
		unsigned id;
		unsigned int duty_cycle;

		id = (unsigned)simple_strtoul(argv[1], NULL, 10);
		if (id >= TWL6030_MAX_PWMS || twl6030_pwms[id] == NULL) {
			puts("No PWM device found\n");
			return 1;
		}

		duty_cycle = (unsigned int)simple_strtoul(argv[2], NULL, 10);
		if (duty_cycle > TWL_PWM_MAX_DUTY_CYCLE) {
			puts("Invalid duty (0~100)\n");
			return 1;
		}

		printf("twl6030_pwm%u: set duty %u%%\n", id+1, duty_cycle);
		pwm_set_duty(id, duty_cycle);
	}

	return 0;
}

U_BOOT_CMD(
	pwm, 3, 1, do_twl6030_pwm,
	"TWL6030 PWM duty",
	"[id] [duty cycle] - set TWL6030 PWM duty\n"
	""
);
#endif /* CONFIG_TWL6030_PWM_DEBUG */
