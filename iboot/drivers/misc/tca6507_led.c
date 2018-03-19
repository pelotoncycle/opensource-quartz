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
#include <i2c.h>

#include <icom/aboot.h>
#include <icom/tca6507_led.h>

/*----------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_TCA6507_LED_DEBUG
#define CONFIG_TCA6507_LED_DEBUG
#endif
#ifndef CONFIG_TCA6507_LED_VERBOSE_DEBUG
#define CONFIG_TCA6507_LED_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TCA6507_LED_DEBUG
#define LED_DPRINT(fmt, args...) \
	do {printf("[tca6507] " fmt, ##args);} while (0)
#define LED_DPUTS(fmt) \
	do {puts("[tca6507] " fmt);} while (0)
#else /* CONFIG_TCA6507_LED_DEBUG */
#define LED_DPRINT(fmt, args...) \
	do {} while (0)
#define LED_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TCA6507_LED_DEBUG */

#ifdef CONFIG_TCA6507_LED_VERBOSE_DEBUG
#define LED_VPRINT(fmt, args...) \
	do {printf("[tca6507] " fmt, ##args);} while (0)
#define LED_VPUTS(fmt) \
	do {puts("[tca6507] " fmt);} while (0)
#else /* CONFIG_TCA6507_LED_VERBOSE_DEBUG */
#define LED_VPRINT(fmt, args...) \
	do {} while (0)
#define LED_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TCA6507_LED_VERBOSE_DEBUG */

#define LED_PRINT(fmt, args...) \
	do {printf("tca6507: " fmt, ##args);} while (0)
#define LED_PUTS(fmt) \
	do {puts("tca6507: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define LED_ERR(fmt, args...) \
	do {printf("tca6507: " fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

/* LED select registers determine the source that drives LED outputs */
#define TCA6507_LS_LED_OFF	0x0	/* Output HI-Z (off) */
#define TCA6507_LS_LED_OFF1	0x1	/* Output HI-Z (off) - not used */
#define TCA6507_LS_LED_PWM0	0x2	/* Output LOW with Bank0 rate */
#define TCA6507_LS_LED_PWM1	0x3	/* Output LOW with Bank1 rate */
#define TCA6507_LS_LED_ON	0x4	/* Output LOW (on) */
#define TCA6507_LS_LED_MIR	0x5	/* Output LOW with Master Intensity */
#define TCA6507_LS_BLINK0	0x6	/* Blink at Bank0 rate */
#define TCA6507_LS_BLINK1	0x7	/* Blink at Bank1 rate */

/* PWM registers */
#define	TCA6507_REG_CNT			11

/*
 * 0x00, 0x01, 0x02 encode the TCA6507_LS_* values, each output
 * owns one bit in each register
 */
#define	TCA6507_SELECT0			0x00
#define	TCA6507_SELECT1			0x01
#define	TCA6507_SELECT2			0x02
#define	TCA6507_FADE_ON			0x03
#define	TCA6507_FULL_ON			0x04
#define	TCA6507_FADE_OFF		0x05
#define	TCA6507_FIRST_OFF		0x06
#define	TCA6507_SECOND_OFF		0x07
#define	TCA6507_MAX_INTENSITY		0x08
#define	TCA6507_MASTER_INTENSITY	0x09
#define	TCA6507_INITIALIZE		0x0A

#define	INIT_CODE			0x8

/*----------------------------------------------------------------------*/

#define NUM_LEDS 7

struct tca6507_chip {
	int			enabled;

	u8			zero_reg_file[TCA6507_REG_CNT];
	u8			reg_file[TCA6507_REG_CNT];

	/* Bank 2 is Master Intensity and doesn't use times */
	struct bank {
		int level;
		int ontime, offtime;
		int on_dflt, off_dflt;
		int time_use, level_use;
	} bank[3];

	struct tca6507_led {
		struct tca6507_chip	*chip;
		int			brightness;
		int			num;
		int			ontime, offtime;
		int			on_dflt, off_dflt;
		int			bank;	/* Bank used, or -1 */
		int			blink;	/* Set if hardware-blinking */
	} leds[NUM_LEDS];
};

/*----------------------------------------------------------------------*/


enum led_brightness {
	LED_OFF		= 0,
	LED_HALF	= 127,
	LED_FULL	= 255,
};

enum {
	BANK0,
	BANK1,
	MASTER,
};

static int bank_source[3] = {
	TCA6507_LS_LED_PWM0,
	TCA6507_LS_LED_PWM1,
	TCA6507_LS_LED_MIR,
};

static int blink_source[2] = {
	TCA6507_LS_BLINK0,
	TCA6507_LS_BLINK1,
};

#define TIMECODES 16
static int time_codes[TIMECODES] = {
	0, 64, 128, 192, 256, 384, 512, 768,
	1024, 1536, 2048, 3072, 4096, 5760, 8128, 16320
};

/*----------------------------------------------------------------------*/

static int has_tca6507 __attribute__ ((section (".data"))) = 0;
static struct tca6507_chip tca __attribute__ ((section (".data")));
static struct tca6507_platform_data *tca6507_pdata __attribute__((section (".data"))) = NULL;

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void save_i2c_bus(void) __attribute__((always_inline));
static inline void save_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_TCA6507_LED_I2C_BUS)
		i2c_set_bus_num(CONFIG_TCA6507_LED_I2C_BUS);
}

static inline void restore_i2c_bus(void) __attribute__((always_inline));
static inline void restore_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_TCA6507_LED_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define save_i2c_bus()
#define restore_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static int tca6507_write_block(int retries, u8 *value, int len, u8 reg)
{
	int ret;

	save_i2c_bus();

_retry:
	ret = i2c_write(TCA6507_ADDR, reg, 1, value, len);
	if (ret) {
		if (--retries > 0) {
			LED_VPRINT("%d: write_block(0x%02X): reg: 0x%02X, length: %d err %d\n", retries, TCA6507_ADDR, reg, len, ret);
			mdelay(10);
			goto _retry;
		} else {
			LED_ERR("write_block(0x%02X): reg: 0x%02X, length: %d err %d\n", TCA6507_ADDR, reg, len, ret);
		}
	} else {
		LED_VPRINT("write_block(0x%02X): reg: 0x%02X, length: %d\n", TCA6507_ADDR, reg, len);
	}

	restore_i2c_bus();

	return ret;
}

#ifdef CONFIG_TCA6507_LED_VERBOSE_DEBUG
static int tca6507_write_byte(int retries, u8 value, u8 reg)
{
	int ret;

	save_i2c_bus();

_retry:
	ret = i2c_write(TCA6507_ADDR, reg, 1, &value, 1);
	if (ret) {
		if (--retries > 0) {
			LED_VPRINT("%d: write(0x%02X): [0x%02X]=0x%02X err %d\n", retries, TCA6507_ADDR, reg, value, ret);
			mdelay(10);
			goto _retry;
		} else {
			LED_ERR("write(0x%02X): [0x%02X]=0x%02X err %d\n", TCA6507_ADDR, reg, value, ret);
		}
	} else {
		LED_VPRINT("write(0x%02X): [0x%02X]=0x%02X\n", TCA6507_ADDR, reg, value);
	}

	restore_i2c_bus();

	return ret;
}
#endif

static int tca6507_read_byte(int retries, u8 *value, u8 reg)
{
	int ret;

	save_i2c_bus();

_retry:
	ret = i2c_read(TCA6507_ADDR, reg, 1, value, 1);
	if (ret) {
		if (--retries > 0) {
			LED_VPRINT("%d: read(0x%02X): [0x%02X] err %d\n", retries, TCA6507_ADDR, reg, ret);
			mdelay(10);
			goto _retry;
		} else {
			LED_ERR("read(0x%02X): [0x%02X] err %d\n", TCA6507_ADDR, reg, ret);
		}
	} else {
		LED_VPRINT("read(0x%02X): [0x%02X]=0x%02X\n", TCA6507_ADDR, reg, *value);
	}

	restore_i2c_bus();

	return ret;
}

/* Convert an led.brightness level (0..255) to a TCA6507 level (0..15) */
static inline int TO_LEVEL(int brightness)
{
	return brightness >> 4;
}

/* ...and convert back */
static inline int TO_BRIGHT(int level)
{
	if (level)
		return (level << 4) | 0xf;
	return 0;
}

/*----------------------------------------------------------------------*/

static int choose_times(int msec, int *c1p, int *c2p)
{
	/*
	 * Choose two timecodes which add to 'msec' as near as possible.
	 * The first returned is the 'on' or 'off' time.  The second is to be
	 * used as a 'fade-on' or 'fade-off' time.  If 'msec' is even,
	 * the first will not be smaller than the second.  If 'msec' is odd,
	 * the first will not be larger than the second.
	 * If we cannot get a sum within 1/8 of 'msec' fail with -EINVAL,
	 * otherwise return the sum that was achieved, plus 1 if the first is
	 * smaller.
	 * If two possibilities are equally good (e.g. 512+0, 256+256), choose
	 * the first pair so there is more change-time visible (i.e. it is
	 * softer).
	 */
	int c1, c2;
	int tmax = msec * 9 / 8;
	int tmin = msec * 7 / 8;
	int diff = 65536;

	/* We start at '1' to ensure we never even think of choosing a
	 * total time of '0'.
	 */
	for (c1 = 1; c1 < TIMECODES; c1++) {
		int t = time_codes[c1];
		if (t < tmin)
			continue;
		if (t > tmax)
			break;
		for (c2 = 0; c2 <= c1; c2++) {
			int tt = time_codes[c2];
			int d;
			if (tt < tmin)
				continue;
			if (tt > tmax)
				break;
			/* This works! */
			d = abs(msec - tt);
			if (d >= diff)
				continue;
			/* Best yet */
			*c1p = c1;
			*c2p = c2;
			diff = d;
			if (d == 0)
				return msec;
		}
	}
	if (diff < 65536) {
		int actual;
		if (msec & 1) {
			c1 = *c2p;
			*c2p = *c1p;
			*c1p = c1;
		}
		actual = time_codes[*c1p] + time_codes[*c2p];
		if (*c1p < *c2p)
			return actual + 1;
		else
			return actual;
	}
	/* No close match */
	LED_ERR("no close match time: %d\n", msec);
#if 0
	return -EINVAL;
#else
	*c1p = TIMECODES - 1;
	*c2p = TIMECODES - 2;
	return time_codes[TIMECODES-1];
#endif
}

/*
 * Update the register file with the appropriate 3-bit state for
 * the given led.
 */
static void set_select(struct tca6507_chip *tca, int led, int val)
{
	int mask = (1 << led);
	int bit;

	for (bit = 0; bit < 3; bit++) {
		int n = tca->reg_file[bit] & ~mask;
		if (val & (1 << bit))
			n |= mask;
		tca->reg_file[bit] = n;
	}
}

/* Update the register file with the appropriate 4-bit code for
 * one bank or other.  This can be used for timers, for levels, or
 * for initialisation.
 */
static void set_code(struct tca6507_chip *tca, int reg, int bank, int new)
{
	int mask = 0xF;
	int n;
	if (bank) {
		mask <<= 4;
		new <<= 4;
	}
	n = tca->reg_file[reg] & ~mask;
	n |= new;
	tca->reg_file[reg] = n;
}

/* Update brightness level. */
static void set_level(struct tca6507_chip *tca, int bank, int level)
{
	switch (bank) {
	case BANK0:
	case BANK1:
		set_code(tca, TCA6507_MAX_INTENSITY, bank, level);
		break;
	case MASTER:
		set_code(tca, TCA6507_MASTER_INTENSITY, 0, level);
		break;
	}
	tca->bank[bank].level = level;
}

/* Record all relevant time code for a given bank */
static void set_times(struct tca6507_chip *tca, int bank)
{
	int c1, c2;
	int result;

	result = choose_times(tca->bank[bank].ontime, &c1, &c2);
	LED_VPRINT("Chose on times %d(%d) %d(%d) for %dms\n", c1, time_codes[c1],
			c2, time_codes[c2], tca->bank[bank].ontime);
	set_code(tca, TCA6507_FADE_ON, bank, 0);
	set_code(tca, TCA6507_FULL_ON, bank, c1);
	tca->bank[bank].ontime = result;

	result = choose_times(tca->bank[bank].offtime, &c1, &c2);
	LED_VPRINT("Chose off times %d(%d) %d(%d) for %dms\n", c1, time_codes[c1],
			c2, time_codes[c2], tca->bank[bank].offtime);
	set_code(tca, TCA6507_FADE_OFF, bank, 0);
	set_code(tca, TCA6507_FIRST_OFF, bank, c1);
	set_code(tca, TCA6507_SECOND_OFF, bank, c1);
	tca->bank[bank].offtime = result;

	set_code(tca, TCA6507_INITIALIZE, bank, INIT_CODE);
}

/*----------------------------------------------------------------------*/

static void led_release(struct tca6507_led *led)
{
	/* If led owns any resource, release it. */
	struct tca6507_chip *tca = led->chip;
	if (led->bank >= 0) {
		struct bank *b = tca->bank + led->bank;
		if (led->blink) {
			if (b->time_use > 0)
				b->time_use--;
		}
		if (b->level_use > 0)
			b->level_use--;
	}
	led->blink = 0;
	led->bank = -1;
}

static int led_prepare(struct tca6507_led *led)
{
	/* Assign this led to a bank, configuring that bank if necessary. */
	int level = TO_LEVEL(led->brightness);
	struct tca6507_chip *tca = led->chip;
#if 0
	int c1, c2;
#endif
	int i;
	struct bank *b;
	int need_init = 0;

	led->brightness = TO_BRIGHT(level);
	if (level == 0) {
		set_select(tca, led->num, TCA6507_LS_LED_OFF);
		return 0;
	}

	if (led->ontime == 0 || led->offtime == 0) {
		/*
		 * Just set the brightness, choosing first usable bank.
		 * If none perfect, choose best.
		 * Count backwards so we check MASTER bank first
		 * to avoid wasting a timer.
		 */
		int best = -1;/* full-on */
		int diff = 15-level;

		if (level == 15) {
			set_select(tca, led->num, TCA6507_LS_LED_ON);
			return 0;
		}

		for (i = MASTER; i >= BANK0; i--) {
			int d;
			if (tca->bank[i].level == level ||
			    tca->bank[i].level_use == 0) {
				best = i;
				break;
			}
			d = abs(level - tca->bank[i].level);
			if (d < diff) {
				diff = d;
				best = i;
			}
		}
		if (best == -1) {
			/* Best brightness is full-on */
			set_select(tca, led->num, TCA6507_LS_LED_ON);
			led->brightness = LED_FULL;
			return 0;
		}

		if (!tca->bank[best].level_use)
			set_level(tca, best, level);

		tca->bank[best].level_use++;
		led->bank = best;
		set_select(tca, led->num, bank_source[best]);
		led->brightness = TO_BRIGHT(tca->bank[best].level);
		return 0;
	}

#if 0
	/*
	 * We have on/off time so we need to try to allocate a timing bank.
	 * First check if times are compatible with hardware and give up if
	 * not.
	 */
	if (choose_times(led->ontime, &c1, &c2) < 0)
		return -EINVAL;
	if (choose_times(led->offtime, &c1, &c2) < 0)
		return -EINVAL;
#endif

	for (i = BANK0; i <= BANK1; i++) {
		if (tca->bank[i].level_use == 0)
			/* not in use - it is ours! */
			break;
		if (tca->bank[i].level != level)
			/* Incompatible level - skip */
			/* FIX: if timer matches we maybe should consider
			 * this anyway...
			 */
			continue;

		if (tca->bank[i].time_use == 0)
			/* Timer not in use, and level matches - use it */
			break;

		if (!(tca->bank[i].on_dflt ||
		      led->on_dflt ||
		      tca->bank[i].ontime == led->ontime))
			/* on time is incompatible */
			continue;

		if (!(tca->bank[i].off_dflt ||
		      led->off_dflt ||
		      tca->bank[i].offtime == led->offtime))
			/* off time is incompatible */
			continue;

		/* looks like a suitable match */
		break;
	}

	if (i > BANK1) {
		/* Nothing matches - how sad */
		LED_ERR("no match bank\n");
		return -EINVAL;
	}

	b = &tca->bank[i];
	if (b->level_use == 0)
		set_level(tca, i, level);
	b->level_use++;
	led->bank = i;

	if (b->on_dflt ||
	    !led->on_dflt ||
	    b->time_use == 0) {
		b->ontime = led->ontime;
		b->on_dflt = led->on_dflt;
		need_init = 1;
	}

	if (b->off_dflt ||
	    !led->off_dflt ||
	    b->time_use == 0) {
		b->offtime = led->offtime;
		b->off_dflt = led->off_dflt;
		need_init = 1;
	}

	if (need_init)
		set_times(tca, i);

	led->ontime = b->ontime;
	led->offtime = b->offtime;

	b->time_use++;
	led->blink = 1;
	led->brightness = TO_BRIGHT(b->level);
	set_select(tca, led->num, blink_source[i]);
	return 0;
}

static int led_assign(struct tca6507_led *led)
{
	int err;

	led_release(led);
	err = led_prepare(led);
	if (err) {
		LED_ERR("led_prepare %d\n", err);
		/*
		 * Can only fail on timer setup.  In that case we need to
		 * re-establish as steady level.
		 */
		led->ontime = 0;
		led->offtime = 0;
		led_prepare(led);
	}

	return err;
}

/*----------------------------------------------------------------------*/

static void tca6507_brightness_set(struct tca6507_led *led, int brightness)
{
	led->brightness = brightness;
	led->ontime = 0;
	led->offtime = 0;
	led_assign(led);
}

static void tca6507_blink_set(struct tca6507_led *led, int delay_on, int delay_off)
{
	led->on_dflt = 1;
	led->off_dflt = 1;
	led->ontime = delay_on;
	led->offtime = delay_off;

	if (led_assign(led) < 0) {
		led->ontime = 0;
		led->offtime = 0;
		led->brightness = LED_OFF;
	}
}

/*----------------------------------------------------------------------*/

void tca6507_set_light(int port, int brightness, int flashOnMS, int flashOffMS)
{
	struct tca6507_led *led;

	if (!has_tca6507)
		return;

	if (port < 0 || port >= NUM_LEDS)
		return;

	LED_DPRINT("%d: brightness %d; on %d; off %d\n", port, brightness, flashOnMS, flashOffMS);

	led = tca.leds + port;
	tca6507_brightness_set(led, brightness);

	if (flashOnMS > 0 && flashOffMS > 0) {
		if (TO_LEVEL(brightness) > 0)
			tca6507_blink_set(led, flashOnMS, flashOffMS);
	}
}

int tca6507_enable_lights(void)
{
	if (!has_tca6507)
		return -ENODEV;

	if (!tca.enabled) {
		if (tca6507_pdata->platform_enable)
			tca6507_pdata->platform_enable();
		tca.enabled = 1;
		LED_VPRINT("tca6507 enabled\n");
	}

	/* Clear previous states (skip TCA6507_INITIALIZE) */
	tca6507_write_block(3, tca.zero_reg_file, TCA6507_REG_CNT-1, 0);
	/* Reset the current states */
	tca6507_write_block(3, tca.reg_file, TCA6507_REG_CNT, 0);

	if (tca.reg_file[TCA6507_SELECT0] == 0 && tca.reg_file[TCA6507_SELECT1] == 0 &&
			tca.reg_file[TCA6507_SELECT2] == 0) {
		if (tca6507_pdata->platform_disable)
			tca6507_pdata->platform_disable();
		tca.enabled = 0;
		LED_VPRINT("tca6507 disabled\n");
	}

	return 0;
}

/*----------------------------------------------------------------------*/

void tca6507_led_init(struct tca6507_platform_data *pdata)
{
	int i;
	u8 value = 0;

	tca6507_pdata = pdata;
	has_tca6507 = 0;

	memset(&tca, 0, sizeof(tca));

	if (!pdata) {
		LED_VPRINT("no pdata\n");
		return;
	}

	if (pdata->platform_init)
		pdata->platform_init();

	if (pdata->platform_enable)
		pdata->platform_enable();

	if (tca6507_read_byte(2, &value, TCA6507_SELECT0)) {
		LED_PUTS("no tca6507\n");
		goto _exit;
	}

	has_tca6507 = 1;

	for (i = 0; i < NUM_LEDS; i++) {
		struct tca6507_led *l = tca.leds + i;

		l->chip = &tca;
		l->num = i;
		l->bank = -1;
	}

	LED_VPRINT("found tca6507\n");

_exit:
	if (pdata->platform_disable)
		pdata->platform_disable();
}

void tca6507_led_shutdown(void)
{
	if (has_tca6507 && tca.enabled) {
		if (tca6507_pdata->platform_disable)
			tca6507_pdata->platform_disable();
		has_tca6507 = 0;
		tca.enabled = 0;
		LED_VPRINT("tca6507 disabled\n");
	}
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_TCA6507_LED_DEBUG
static int do_tca6507_set_light(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (!has_tca6507) {
		puts("no tc6507 found\n");
		return 0;
	}

	if (argc == 5) {
		int port;
		int brightness;
		int flashOnMS, flashOffMS;

		port = (int)simple_strtoul(argv[1], NULL, 10);
		if (port < 0 || port >= NUM_LEDS) {
			printf("bad port %d (0~%d)\n", port, NUM_LEDS-1);
			return 1;
		}

		brightness = (int)simple_strtoul(argv[2], NULL, 10);
		if (brightness > LED_FULL || brightness < LED_OFF) {
			printf("bad brightness %d (%d~%d)\n", brightness, LED_OFF, LED_FULL);
			return 1;
		}

		flashOnMS = (int)simple_strtoul(argv[3], NULL, 10);
		if (flashOnMS < 0) {
			printf("bad flashOn time %d\n", flashOnMS);
			return 1;
		}

		flashOffMS = (int)simple_strtoul(argv[4], NULL, 10);
		if (flashOffMS < 0) {
			printf("bad flashOff time %d\n", flashOffMS);
			return 1;
		}

		printf("port=%u, brightness=%u, onMS=%u, offMS=%u\n", port, brightness, flashOnMS, flashOffMS);

		tca6507_set_light(port, brightness, flashOnMS, flashOffMS);
	} else {
		puts("usage: tca6507_set <port> <brightness> <flashOnMs> <flashOffMs>\n");
	}

	return 0;
}

U_BOOT_CMD(
	tca6507_set,	5,	0,	do_tca6507_set_light,
	"tca6507_set",
	""
);

static int do_tca6507_enable_lights(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (!has_tca6507) {
		puts("no tc6507 found\n");
		return 0;
	}

	if (argc == 1) {
		tca6507_enable_lights();
	} else {
		puts("usage: tca6507_enable\n");
	}

	return 0;
}

U_BOOT_CMD(
	tca6507_enable,	1,	0,	do_tca6507_enable_lights,
	"tca6507_enable",
	""
);
#endif /* CONFIG_TCA6507_LED_DEBUG */

/*----------------------------------------------------------------------*/

#ifdef CONFIG_TCA6507_LED_VERBOSE_DEBUG
static int do_tca6507_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (!has_tca6507) {
		puts("no tc6507 found\n");
		return 0;
	}

	if (argc == 3) {
		u8 reg, value;

		reg = (u8)simple_strtoul(argv[1], NULL, 16);
		if (reg >= TCA6507_REG_CNT) {
			printf("bad reg %u\n", reg);
			return 1;
		}

		value = (u8)simple_strtoul(argv[2], NULL, 16);

		printf("write(0x%02X): [0x%02X]=0x%02X\n", TCA6507_ADDR, reg, value);
		tca6507_write_byte(3, value, reg);
	} else {
		puts("usage: tca6507_write <reg> <value>\n");
	}

	return 0;
}

U_BOOT_CMD(
	tca6507_write,	6,	0,	do_tca6507_write,
	"tca6507 write",
	""
);

static int do_tca6507_dump(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	u8 r, value;

	if (!has_tca6507) {
		puts("no tc6507 found\n");
		return 0;
	}

	save_i2c_bus();
	for (r = 0; r < TCA6507_INITIALIZE; r++) {
		ret = tca6507_read_byte(3, &value, r);
		if (ret)
			printf("read(0x%02X): [0x%02X] err %d\n", TCA6507_ADDR, r, ret);
		else
			printf("read(0x%02X): [0x%02X]=0x%02X\n", TCA6507_ADDR, r, value);
	}
	restore_i2c_bus();

	return 0;
}

U_BOOT_CMD(
	tca6507_dump,	6,	0,	do_tca6507_dump,
	"tca6507 dump",
	""
);
#endif /* CONFIG_TCA6507_LED_VERBOSE_DEBUG */
