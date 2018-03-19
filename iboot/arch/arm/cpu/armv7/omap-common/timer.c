/*
 * (C) Copyright 2008
 * Texas Instruments
 *
 * Richard Woodruff <r-woodruff2@ti.com>
 * Syed Moahmmed Khasim <khasim@ti.com>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
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

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clocks.h>

DECLARE_GLOBAL_DATA_PTR;

static struct gptimer *timer_base = (struct gptimer *)CONFIG_SYS_TIMERBASE;

/*
 * Nothing really to do with interrupts, just starts up a counter.
 */

#define TIMER_CLOCK		(V_SCLK / (2 << CONFIG_SYS_PTV))
#define TIMER_OVERFLOW_VAL	0xffffffff
#define TIMER_LOAD_VAL		0

int timer_init(void)
{
#if defined(CONFIG_OMAP34XX)
#if (CONFIG_SYS_TIMERBASE != OMAP34XX_GPT2)
#error "CONFIG_SYS_TIMERBASE != OMAP34XX_GPT2\n"
#endif /* CONFIG_SYS_TIMERBASE != OMAP34XX_GPT2 */
	/* GPT2 timer source is SYS_CLK */
#if 1
	setbits_le32(0x48005040/*CM_CLKSEL_PER*/, (0x01 << 0));
#else
	sr32(0x48005040/*CM_CLKSEL_PER*/, 0, 1, 0x1);
#endif
	/* Enable GPT2 clocks */
	omap_clk_enable(0x48005000/*CM_FCLKEN_PER*/, 0x48005010/*CM_ICLKEN_PER*/,
			0x48005020/*CM_IDLEST_PER*/, 3);
#elif defined(CONFIG_OMAP44XX)
#if (CONFIG_SYS_TIMERBASE != GPT2_BASE)
#error "CONFIG_SYS_TIMERBASE != GPT2_BASE\n"
#endif /* CONFIG_SYS_TIMERBASE != GPT2_BASE */
	/* The default GPT2 timer source is SYS_CLK */
	omap_clk_enable(&prcm->cm_l4per_gptimer2_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif

	/* start the counter ticking up, reload value on overflow */
	writel(TIMER_LOAD_VAL, &timer_base->tldr);
	/* enable timer */
	writel((CONFIG_SYS_PTV << 2) | TCLR_PRE | TCLR_AR | TCLR_ST,
		&timer_base->tclr);

	/* reset time, capture current incrementer value time */
	gd->lastinc = readl(&timer_base->tcrr) / (TIMER_CLOCK / CONFIG_SYS_HZ);
	gd->tbl = 0;		/* start "advancing" time stamp from 0 */

	return 0;
}

/*
 * timer without interrupts
 */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/* delay x useconds */
#if 0
void __udelay(unsigned long usec)
{
	long tmo = usec * (TIMER_CLOCK / 1000) / 1000;
	unsigned long now, last = readl(&timer_base->tcrr);

	while (tmo > 0) {
		now = readl(&timer_base->tcrr);
		if (last > now) /* count up timer overflow */
			tmo -= TIMER_OVERFLOW_VAL - last + now + 1;
		else
			tmo -= now - last;
		last = now;
	}
}
#else
static inline void __udelay_internal(unsigned long usec)
{
	long tmo = usec * (TIMER_CLOCK / 1000) / 1000;
	unsigned long now, last = readl(&timer_base->tcrr);

	while (tmo > 0) {
		now = readl(&timer_base->tcrr);
		if (last > now) /* count up timer overflow */
			tmo -= TIMER_OVERFLOW_VAL - last + now + 1;
		else
			tmo -= now - last;
		last = now;
	}
}

/* we're limited to 32-bit because SPL doesn't have 64-bit libraries
 * so for large values, call again.  modeled after time.c udelay(). */

#ifndef CONFIG_MAX_UDELAY
#define CONFIG_MAX_UDELAY 100000
#endif

void __udelay(unsigned long usec)
{
	ulong kv;
	do {
		kv = usec > CONFIG_MAX_UDELAY ? CONFIG_MAX_UDELAY : usec;
		__udelay_internal(kv);
		usec -= kv;
	} while (usec);
}
#endif

ulong get_timer_masked(void)
{
	/* current tick value */
	ulong now = readl(&timer_base->tcrr) / (TIMER_CLOCK / CONFIG_SYS_HZ);

	if (now >= gd->lastinc)	/* normal mode (non roll) */
		/* move stamp fordward with absoulte diff ticks */
		gd->tbl += (now - gd->lastinc);
	else	/* we have rollover of incrementer */
		gd->tbl += ((TIMER_OVERFLOW_VAL / (TIMER_CLOCK / CONFIG_SYS_HZ))
			     - gd->lastinc) + 1 + now;
	gd->lastinc = now;
	return gd->tbl;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}

unsigned long timeout_init(int msec, unsigned long *ticks)
{
	/* current tick value */
	*ticks = (unsigned long)readl(&timer_base->tcrr);

	return msec * (TIMER_CLOCK / CONFIG_SYS_HZ);
}

int timeout_check(unsigned long *timeout, unsigned long *last_ticks)
{
	if (*timeout != 0) {
		/* current tick value */
		unsigned long ticks = readl(&timer_base->tcrr);
		unsigned long diffs;

		if (ticks >= *last_ticks) { /* normal mode (non roll) */
			/* move stamp fordward with absoulte diff ticks */
			diffs = ticks - *last_ticks;
		} else { /* we have rollover of incrementer */
			diffs = TIMER_OVERFLOW_VAL - *last_ticks + 1 + ticks;
		}
		*last_ticks = ticks;

		if (*timeout > diffs) {
			*timeout -= diffs;
			return 0;
		} else {
			*timeout = 0;
			return 1;
		}
	} else {
		return 1;
	}
}
