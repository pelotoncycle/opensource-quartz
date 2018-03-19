/*
 * linux/drivers/leds/leds-twl6032-pwm.c
 *
 * TWL6032 PWM based LED control
 *
 * Copyright (C) 2009-2013 InnoComm Mobile Technology Corp.
 * Author: James Wu <james.wu@innocomm.com>
 *
 * based on leds-pwm.c by Luotao Fu (l.fu@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/i2c/twl.h>

/*#define TWL6030_DEBUG_PWM_ON_OFF_VALUES*/

#define TWL6030_MAX_PWMS				(2)

#define TWL6030_PWM1_ID	0
#define TWL6030_PWM2_ID	1

#define	PWM_128_INTERVAL_SHIFT		4
#define	PWM_64_INTERVAL_SHIFT		(PWM_128_INTERVAL_SHIFT - 1)

#define TWL_PWM_MAX_DUTY_CYCLE			100

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

static DEFINE_MUTEX(twl6030_pwm_mutex);
static u8 twl6030_toggle3_value = 0;

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

struct twl6030_pwm {
	struct twl_pwm_device base;

	struct led_classdev	cdev;
	struct mutex	lock;

	unsigned	id;

	int			ready;
	int			clock_cycle;

	u8			pwmoff_level;		/* the fixed PWMOFF value */
	u8			pwmon_max_level;	/* the maximal PWMON value */
	u8			pwmon_min_level;	/* the minimal PWMON value */
	int			pwmon_shift;		/* the shift value for PWMON interval */
	int			pwmon_interval;		/* the PWMON interval value */
	int			curr_pwmon_level;	/* current PWMON value */

	int			invert_duty_cycle;
	u8			max_duty_cycle;
	u8			min_duty_cycle;
	u8			duty_cycle_range;
	int			brightness;

	int (*platform_init)(struct twl_pwm_device *pwm, int brightness);
	void (*platform_early_enable)(struct twl_pwm_device *pwm);
	void (*platform_enable)(struct twl_pwm_device *pwm);
	void (*platform_disable)(struct twl_pwm_device *pwm);
};

static void twl6030_pwm_change(struct twl6030_pwm *pwm, u8 duty_cycle, int interval)
{
	unsigned id = pwm->id;
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
	u8 old_duty_cycle = duty_cycle;
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
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

#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
	if (pwm->base.dev)
		dev_info(pwm->base.dev, "pwm%u: set brightness %d (%d%%=>%d%%)\n",
				id+1, pwm->brightness, old_duty_cycle, duty_cycle);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */

	curr_level = pwm->curr_pwmon_level;

	if (duty_cycle) {
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
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
				pr_info("twl6030_pwm%u: PWMON=0x%02X\n", id+1, curr_level | pwm_length);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */
			}
		} else {
			i = (pwm->curr_pwmon_level - curr_level) >> pwm->pwmon_shift;
			while (i > 0) {
				i--;
				curr_level += pwm->pwmon_interval;
				twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
						curr_level | pwm_length, twl6030_pwm_reg_map[id].on_reg);
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
				pr_info("twl6030_pwm%u: PWMON=0x%02X\n", id+1, curr_level | pwm_length);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */
			}
		}
	}

	twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
			target_level | pwm_length, twl6030_pwm_reg_map[id].on_reg);
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
	pr_info("twl6030_pwm%u: PWMON=0x%02X\n", id+1, target_level | pwm_length);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */
}

static void twl6030_pwm_config(struct twl6030_pwm* pwm, struct twl_pwm_led *pwm_led)
{
	unsigned id = pwm_led->id;
	u8 duty_cycle = 0;
	u8 pwm_off = 0;

	pwm->id = id;
	pwm->ready = 1;
	pwm->base.name = pwm_led->name;
	pwm->clock_cycle = pwm_led->default_clock_cycle;

	/* PWM_OFF value */
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	twl_i2c_read_u8(twl6030_pwm_reg_map[id].mod, &pwm_off, twl6030_pwm_reg_map[id].off_reg);
	if (pwm_off == 0x7F) {
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		pr_info("pwm%u: found 128 clock cycle\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
		pwm->clock_cycle = 128;
	} else if (pwm_off == 0x3F) {
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		pr_info("pwm%u: found 64 clock cycle\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
		pwm->clock_cycle = 64;
	}
#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */

	if (pwm->clock_cycle == 128) {
		/* PWM1 & PWM2 */
		pwm->pwmoff_level		= 0x7F;
		pwm->pwmon_max_level	= 0x7E;
		pwm->pwmon_min_level	= 0x01;
		pwm->pwmon_shift		= PWM_128_INTERVAL_SHIFT;
#ifdef CONFIG_LEDS_TWL6032_PWM_DEBUG
		pr_info("pwm%u: 128 clock cycle (256Hz)\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_DEBUG */
	} else {
		/* PWM1 & PWM2 */
		pwm->pwmoff_level		= 0x3F;
		pwm->pwmon_max_level	= 0x3E; /* no PWM_LENGTH bit */
		pwm->pwmon_min_level	= 0x01; /* no PWM_LENGTH bit */
		pwm->pwmon_shift		= PWM_64_INTERVAL_SHIFT;
#ifdef CONFIG_LEDS_TWL6032_PWM_DEBUG
		pr_info("pwm%u: 64 clock cycle (512Hz)\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_DEBUG */
	}
	pwm->pwmon_interval = 1 << pwm->pwmon_shift;
	pwm->curr_pwmon_level = pwm->pwmon_min_level;
	pwm->brightness = 0;

	/* the maximum duty cycle */
	if (pwm_led->max_duty_cycle > TWL_PWM_MAX_DUTY_CYCLE || pwm_led->max_duty_cycle <= 0)
		pwm->max_duty_cycle = TWL_PWM_MAX_DUTY_CYCLE;
	else
		pwm->max_duty_cycle = (u8)pwm_led->max_duty_cycle;
	/* the minimum duty cycle */
	if (pwm_led->min_duty_cycle >= TWL_PWM_MAX_DUTY_CYCLE || pwm_led->min_duty_cycle < 0
			|| pwm_led->min_duty_cycle >= pwm->max_duty_cycle)
		pwm->min_duty_cycle = 0;
	else
		pwm->min_duty_cycle = (u8)pwm_led->min_duty_cycle;
	/* the range of duty cycle */
	pwm->duty_cycle_range = pwm->max_duty_cycle - pwm->min_duty_cycle;
	/* invert the duty cycle? */
	if (pwm_led->invert_duty_cycle == 1)
		pwm->invert_duty_cycle = 1;
	else
		pwm->invert_duty_cycle = 0;

	if (pwm_led->platform_init)
		pwm->platform_init = pwm_led->platform_init;
	if (pwm_led->platform_early_enable)
		pwm->platform_early_enable = pwm_led->platform_early_enable;
	if (pwm_led->platform_enable)
		pwm->platform_enable = pwm_led->platform_enable;
	if (pwm_led->platform_disable)
		pwm->platform_disable = pwm_led->platform_disable;

	/* PWM1 & PWM2 */

	/* PWM_OFF value */
	if ((pwm_off == 0x7F) || (pwm_off == 0x3F)) {
		/* PWM is enabled */
		u8 pwm_on = 0;

		mutex_lock(&twl6030_pwm_mutex);
		twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_mask;
		twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_clock | twl6030_pwm_reg_map[id].enable_set;
		mutex_unlock(&twl6030_pwm_mutex);

		/* PWM_ON value */
		twl_i2c_read_u8(twl6030_pwm_reg_map[id].mod, &pwm_on, twl6030_pwm_reg_map[id].on_reg);
		if (pwm->clock_cycle != 128)
			pwm_on = pwm_on & 0x7F; /* Remove PWM_LENGTH */
		if (pwm_on == 0) {
			/* Setup the minimal duty cycle: 1% */
			pwm->curr_pwmon_level = 0;
			duty_cycle = 1;
			pwm->brightness = (duty_cycle * TWL_PWM_LED_MAX_BRIGHTNESS) / TWL_PWM_MAX_DUTY_CYCLE;
			twl6030_pwm_change(pwm, duty_cycle, 0);
		} else {
			if (pwm_on == pwm->pwmoff_level) {
				pwm->curr_pwmon_level = pwm->pwmon_max_level;
				duty_cycle = TWL_PWM_MAX_DUTY_CYCLE;
			} else {
				pwm->curr_pwmon_level = pwm_on;
				duty_cycle = DIV_ROUND_UP(
						(pwm->pwmoff_level - pwm_on) * TWL_PWM_MAX_DUTY_CYCLE,
						pwm->clock_cycle);
			}
			if (pwm->invert_duty_cycle) {
				/* Invert the duty cycle */
				duty_cycle = TWL_PWM_MAX_DUTY_CYCLE - duty_cycle;
			}
			if (pwm->duty_cycle_range != TWL_PWM_MAX_DUTY_CYCLE) {
				duty_cycle = DIV_ROUND_CLOSEST(duty_cycle * TWL_PWM_MAX_DUTY_CYCLE, pwm->duty_cycle_range);
				if (duty_cycle > pwm->min_duty_cycle)
					duty_cycle -= pwm->min_duty_cycle;
			}
			pwm->brightness = DIV_ROUND_CLOSEST(duty_cycle * TWL_PWM_LED_MAX_BRIGHTNESS, TWL_PWM_MAX_DUTY_CYCLE);
		}
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		pr_info("twl6030_pwm%u: PWMOFF=0x%02X, PWMON=0x%02X, brightness %d, current PWM duty cycle: %d%% (%d Hz)\n",
				id+1, pwm_off, pwm_on, pwm->brightness, duty_cycle, (pwm->clock_cycle != 128 ? 512 : 256));
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
	} else {
		twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
				pwm->pwmoff_level,
				twl6030_pwm_reg_map[id].off_reg);
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
		pr_info("twl6030_pwm%u: PWMOFF=0x%02X\n", id+1, pwm->pwmoff_level);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */
	}
}

static void twl6030_pwm_enable(struct twl6030_pwm *pwm)
{
	unsigned id = pwm->id;

	mutex_lock(&twl6030_pwm_mutex);

	if ((twl6030_toggle3_value & twl6030_pwm_reg_map[id].enable_clock) !=
			twl6030_pwm_reg_map[id].enable_clock) {
#ifdef CONFIG_LEDS_TWL6032_PWM_DEBUG
		if (pwm->base.dev) dev_info(pwm->base.dev, "turn on PWM%u\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_DEBUG */

		twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
				pwm->pwmoff_level, twl6030_pwm_reg_map[id].off_reg);
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
		pr_info("twl6030_pwm%u: PWMOFF=0x%02X\n", id+1, pwm->pwmoff_level);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */

		/* The clock inputs for generation of PWM signals are enabled */
		twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_mask;
		twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_clock;
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		if (pwm->base.dev)
			dev_info(pwm->base.dev, "pwm%u: enable input clock: 0x%02X\n",
				id+1, twl6030_toggle3_value);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
		twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
				twl6030_toggle3_value,
				twl6030_pwm_reg_map[id].enable_reg);

		/* Setup the minimal duty cycle: 1% */
		twl6030_pwm_change(pwm, 1, 0);

		/* Start the PWM signal generation */
		twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_set;
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		if (pwm->base.dev)
			dev_info(pwm->base.dev, "pwm%u: start clock: 0x%02X\n",
				id+1, twl6030_toggle3_value);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
		twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
				twl6030_toggle3_value,
				twl6030_pwm_reg_map[id].enable_reg);

		if (pwm->platform_enable) {
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
			if (pwm->base.dev)
				dev_info(pwm->base.dev, "pwm%u: platform_enable\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
			pwm->platform_enable((struct twl_pwm_device*)pwm);
		}
	}

	mutex_unlock(&twl6030_pwm_mutex);
}

static void twl6030_pwm_disable(struct twl6030_pwm *pwm)
{
	unsigned id = pwm->id;

#ifdef CONFIG_LEDS_TWL6032_PWM_DEBUG
	if (pwm->base.dev) dev_info(pwm->base.dev, "turn off PWM%u\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_DEBUG */

	mutex_lock(&twl6030_pwm_mutex);

	if (pwm->platform_disable) {
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
		if (pwm->base.dev)
			dev_info(pwm->base.dev, "pwm%u: platform_disable\n", id+1);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
		pwm->platform_disable((struct twl_pwm_device*)pwm);
	}

	/* Stop the PWM signal generation */
	twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_set;
	twl6030_toggle3_value |= twl6030_pwm_reg_map[id].enable_reset;
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
	if (pwm->base.dev)
		dev_info(pwm->base.dev, "pwm%u: stop clock: 0x%02X\n",
				id+1, twl6030_toggle3_value);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
			twl6030_toggle3_value,
			twl6030_pwm_reg_map[id].enable_reg);

	/* The clock inputs for generation of PWM signals are disabled */
	twl6030_toggle3_value &= ~twl6030_pwm_reg_map[id].enable_mask;
#ifdef CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG
	if (pwm->base.dev)
		dev_info(pwm->base.dev, "pwm%u: disable input clock: 0x%02X\n",
				id+1, twl6030_toggle3_value);
#endif /* CONFIG_LEDS_TWL6032_PWM_VERBOSE_DEBUG */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].enable_mod,
			twl6030_toggle3_value,
			twl6030_pwm_reg_map[id].enable_reg);

	/* PWM_OFF value */
	twl_i2c_write_u8(twl6030_pwm_reg_map[id].mod,
			pwm->pwmoff_level - 1,
			twl6030_pwm_reg_map[id].off_reg);
#ifdef TWL6030_DEBUG_PWM_ON_OFF_VALUES
	pr_info("twl6030_pwm%u: PWMOFF=0x%02X\n", id+1, pwm->pwmoff_level - 1);
#endif /* TWL6030_DEBUG_PWM_ON_OFF_VALUES */

	mutex_unlock(&twl6030_pwm_mutex);
}

void twl_pwm_set_brightness(struct twl_pwm_device *pwm_device, int brightness)
{
	struct twl6030_pwm *pwm = (struct twl6030_pwm*)pwm_device;
	u8 duty_cycle;

	mutex_lock(&pwm->lock);

	if (pwm->ready) {
		if (brightness > 0) {
			duty_cycle = DIV_ROUND_UP(brightness * TWL_PWM_MAX_DUTY_CYCLE, TWL_PWM_LED_MAX_BRIGHTNESS);
		} else
			duty_cycle = 0;

		pwm->brightness = brightness;

		if (brightness > 0) {
			/* Enable PWM first */
			twl6030_pwm_enable(pwm);
			twl6030_pwm_change(pwm, duty_cycle, 1);
		} else {
			/* Disable PWM */
			twl6030_pwm_disable(pwm);
		}
	}

	mutex_unlock(&pwm->lock);
}
EXPORT_SYMBOL_GPL(twl_pwm_set_brightness);

static enum led_brightness twl6030_pwm_get(struct led_classdev *led_cdev)
{
	struct twl6030_pwm *pwm = container_of(led_cdev, struct twl6030_pwm, cdev);
	enum led_brightness brightness;

	mutex_lock(&pwm->lock);
	brightness = (enum led_brightness)pwm->brightness;
	mutex_unlock(&pwm->lock);

	return brightness;
}

static void twl6030_pwm_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct twl6030_pwm *pwm = container_of(led_cdev, struct twl6030_pwm, cdev);
	twl_pwm_set_brightness((struct twl_pwm_device*)pwm, (int)brightness);
}

static int twl6030_pwm_probe(struct platform_device *pdev)
{
	struct twl_pwm_data *pdata = pdev->dev.platform_data;
	struct twl_pwm_led *pwm_led;
	struct twl6030_pwm *pwms, *pwm;
	int i = 0, ret = 0;

	if (!pdata)
		return -EINVAL;

	if (pdata->num_pwms > TWL6030_MAX_PWMS) {
		dev_err(&pdev->dev, "invalid PWMs (%d)\n", pdata->num_pwms);
		pdata->num_pwms = TWL6030_MAX_PWMS;
	}

	pwms = kzalloc(sizeof(struct twl6030_pwm) * pdata->num_pwms,
							GFP_KERNEL);
	if (!pwms)
		return -ENOMEM;

	for (; i < pdata->num_pwms; i++) {
		pwm_led = &pdata->pwms[i];
		pwm = &pwms[i];

		if (pwm_led->id >= TWL6030_MAX_PWMS) {
			dev_err(&pdev->dev, "invalid PWM id (%u)\n", pwm_led->id);
			BUG();
			return -EINVAL;
		}

		if (pwm_led->default_clock_cycle != 128 && pwm_led->default_clock_cycle != 64) {
			dev_err(&pdev->dev, "invalid default clock cycle (%d)\n", pwm_led->default_clock_cycle);
			pwm_led->default_clock_cycle = 128;
		}

		mutex_init(&pwm->lock);
		twl6030_pwm_config(pwm, pwm_led);

		ret = 1;
		if (pwm->platform_init)
			ret = pwm->platform_init((struct twl_pwm_device*)pwm, pwm->brightness);
		if (ret) {
			pwm->base.dev = &pdev->dev;

			pwm->cdev.name = pwm_led->name;
			pwm->cdev.default_trigger = pwm_led->default_trigger;

			pwm->cdev.max_brightness = TWL_PWM_LED_MAX_BRIGHTNESS;
			pwm->cdev.brightness = pwm->brightness;

			pwm->cdev.brightness_set = twl6030_pwm_set;
			pwm->cdev.brightness_get = twl6030_pwm_get;
			/*pwm->cdev.flags |= LED_CORE_SUSPENDRESUME;*/

			ret = led_classdev_register(&pdev->dev, &pwm->cdev);
			if (ret < 0) {
				dev_err(&pdev->dev, "%s: failed to register led\n", pwm_led->name);
				twl6030_pwm_disable(pwm);
				goto err;
			}
			pwm->base.dev = pwm->cdev.dev;

			twl6030_pwm_disable(pwm);
		} else if (!pwm->base.dev) {
			dev_err(&pdev->dev, "%s: no dev\n", pwm_led->name);
		}
	}

	platform_set_drvdata(pdev, pwms);

	return 0;

err:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			if (pwms[i].cdev.name) {
				led_classdev_unregister(&pwms[i].cdev);
			}
		}
	}

	kfree(pwms);

	return ret;
}

static int __devexit twl6030_pwm_remove(struct platform_device *pdev)
{
	int i;
	struct twl_pwm_data *pdata = pdev->dev.platform_data;
	struct twl6030_pwm *pwms;

	pwms = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_pwms; i++) {
		if (pwms[i].base.dev == pwms[i].cdev.dev) {
			led_classdev_unregister(&pwms[i].cdev);
		}
		twl6030_pwm_disable(&pwms[i]);
	}

	kfree(pwms);

	return 0;
}

static void twl6030_pwm_shutdown(struct platform_device *pdev)
{
	int i;
	struct twl_pwm_data *pdata = pdev->dev.platform_data;
	struct twl6030_pwm *pwms;

	pwms = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_pwms; i++) {
		struct twl6030_pwm *pwm = &pwms[i];
		unsigned id = pwm->id;
		int enabled;

		mutex_lock(&pwm->lock);

		pwm->ready = 0;

		mutex_lock(&twl6030_pwm_mutex);
		enabled = ((twl6030_toggle3_value & twl6030_pwm_reg_map[id].enable_clock) ==
						twl6030_pwm_reg_map[id].enable_clock) ? 1 : 0;
		mutex_unlock(&twl6030_pwm_mutex);

		if (enabled) {
			/* Setup the minimal duty cycle: 1% */
			twl6030_pwm_change(pwm, 1, 0);
			/* turn off PWM */
			twl6030_pwm_disable(pwm);
		}

		pwm->brightness = 0;

		mutex_unlock(&pwm->lock);
	}
}

static struct platform_driver twl6030_pwm_driver = {
	.probe		= twl6030_pwm_probe,
	.remove		= __devexit_p(twl6030_pwm_remove),
	.shutdown	= twl6030_pwm_shutdown,
	.driver		= {
		.name	= "twl6032_pwm",
		.owner	= THIS_MODULE,
	},
};

static int __init twl6030_pwm_init(void)
{
	return platform_driver_register(&twl6030_pwm_driver);
}

static void __exit twl6030_pwm_exit(void)
{
	platform_driver_unregister(&twl6030_pwm_driver);
}

subsys_initcall(twl6030_pwm_init);
module_exit(twl6030_pwm_exit);

MODULE_AUTHOR("James Wu <james.wu@innocomm.com>");
MODULE_DESCRIPTION("PWM LED driver for TWL6032");
MODULE_LICENSE("GPL");
