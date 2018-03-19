/*
 * Copyright (c) 2009 Wind River Systems, Inc.
 * Tom Rix <Tom.Rix@windriver.com>
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
 *
 * This work is derived from the linux 2.6.27 kernel source
 * To fetch, use the kernel repository
 * git://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux-2.6.git
 * Use the v2.6.27 tag.
 *
 * Below is the original's header including its copyright
 *
 *  linux/arch/arm/plat-omap/gpio.c
 *
 * Support functions for OMAP GPIO
 *
 * Copyright (C) 2003-2005 Nokia Corporation
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>

/*#define CONFIG_OMAP_GPIO_DEBUG*/
/*#define CONFIG_OMAP_GPIO_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_OMAP_GPIO_DEBUG
#define CONFIG_OMAP_GPIO_DEBUG
#endif /* CONFIG_OMAP_GPIO_DEBUG */
#endif /* DEBUG */

#ifdef CONFIG_OMAP_GPIO_DEBUG
#define GPIO_DPRINT(gpio, fmt, args...) \
	do {printf("gpio_%d: " fmt, gpio, ##args);} while (0)
#else /* CONFIG_OMAP_GPIO_DEBUG */
#define GPIO_DPRINT(gpio, fmt, args...) \
	do {} while (0)
#endif /* CONFIG_OMAP_GPIO_DEBUG */

#ifdef CONFIG_OMAP_GPIO_VERBOSE_DEBUG
#define GPIO_VPRINT(gpio, fmt, args...) \
	do {printf("gpio_%d: " fmt, gpio, ##args);} while (0)
#else /* CONFIG_OMAP_GPIO_VERBOSE_DEBUG */
#define GPIO_VPRINT(gpio, fmt, args...) \
	do {} while (0)
#endif /* CONFIG_OMAP_GPIO_VERBOSE_DEBUG */

#define GPIO_PRINT(gpio, fmt, args...) \
	do {printf("gpio_%d: " fmt, gpio, ##args);} while (0)

struct omap_gpio_dev {
	unsigned int		clk_enabled;
#if defined(CONFIG_OMAP34XX)
	/* OMAP3 clock registers */
	const unsigned int	*fclk_addr;
	const unsigned int	*iclk_addr;
	const unsigned int	*idlest_addr;
	const unsigned int	clk_bit;
#elif defined(CONFIG_OMAP44XX)
	/* OMAP4 clock register */
	const unsigned int	*clkctrl_addr;
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
};

#if defined(CONFIG_OMAP34XX)
static struct omap_gpio_dev gpio_dev_34xx[6] __attribute__ ((section (".data"))) = {
	[0] = { /* GPIO1 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48004C00, /* CM_FCLKEN_WKUP */
		.iclk_addr		= (const unsigned int*)0x48004C10, /* CM_ICLKEN_WKUP */
		.idlest_addr	= (const unsigned int*)0x48004C20, /* CM_IDLEST_WKUP */
		.clk_bit		= 3,
	},
	[1] = { /* GPIO2 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48005000, /* CM_FCLKEN_PER */
		.iclk_addr		= (const unsigned int*)0x48005010, /* CM_ICLKEN_PER */
		.idlest_addr	= (const unsigned int*)0x48005020, /* CM_IDLEST_PER */
		.clk_bit		= 13,
	},
	[2] = { /* GPIO3 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48005000, /* CM_FCLKEN_PER */
		.iclk_addr		= (const unsigned int*)0x48005010, /* CM_ICLKEN_PER */
		.idlest_addr	= (const unsigned int*)0x48005020, /* CM_IDLEST_PER */
		.clk_bit		= 14,
	},
	[3] = { /* GPIO4 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48005000, /* CM_FCLKEN_PER */
		.iclk_addr		= (const unsigned int*)0x48005010, /* CM_ICLKEN_PER */
		.idlest_addr	= (const unsigned int*)0x48005020, /* CM_IDLEST_PER */
		.clk_bit		= 15,
	},
	[4] = { /* GPIO5 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48005000, /* CM_FCLKEN_PER */
		.iclk_addr		= (const unsigned int*)0x48005010, /* CM_ICLKEN_PER */
		.idlest_addr	= (const unsigned int*)0x48005020, /* CM_IDLEST_PER */
		.clk_bit		= 16,
	},
	[5] = { /* GPIO6 */
		.clk_enabled	= 0,
		.fclk_addr		= (const unsigned int*)0x48005000, /* CM_FCLKEN_PER */
		.iclk_addr		= (const unsigned int*)0x48005010, /* CM_ICLKEN_PER */
		.idlest_addr	= (const unsigned int*)0x48005020, /* CM_IDLEST_PER */
		.clk_bit		= 17,
	},
};
static struct omap_gpio_dev *omap_gpio_device = gpio_dev_34xx;
#elif defined(CONFIG_OMAP44XX)
static struct omap_gpio_dev gpio_dev_44xx[6] __attribute__ ((section (".data"))) = {
	[0] = { /* GPIO1 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A307838, /* CM_WKUP_GPIO1_CLKCTRL */
	},
	[1] = { /* GPIO2 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A009460, /* CM_L4PER_GPIO2_CLKCTRL */
	},
	[2] = { /* GPIO3 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A009468, /* CM_L4PER_GPIO3_CLKCTRL */
	},
	[3] = { /* GPIO4 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A009470, /* CM_L4PER_GPIO4_CLKCTRL */
	},
	[4] = { /* GPIO5 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A009478, /* CM_L4PER_GPIO5_CLKCTRL */
	},
	[5] = { /* GPIO6 */
		.clk_enabled	= 0,
		.clkctrl_addr	= (const unsigned int*)0x4A009480, /* CM_L4PER_GPIO6_CLKCTRL */
	},
};
static struct omap_gpio_dev *omap_gpio_device = gpio_dev_44xx;
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif

static inline struct omap_gpio_dev *get_gpio_device(int gpio)
{
	return &omap_gpio_device[gpio >> 5];
}

#define OMAP_GPIO_DIR_OUT	0
#define OMAP_GPIO_DIR_IN	1

static inline const struct gpio_bank *get_gpio_bank(int gpio)
{
	struct omap_gpio_dev *dev = get_gpio_device(gpio);
	if (!dev->clk_enabled) {
#if defined(CONFIG_OMAP34XX)
		if (dev->fclk_addr)
			omap_clk_enable((u32 *)dev->fclk_addr, (u32 *)dev->iclk_addr,
					(u32 *)dev->idlest_addr, dev->clk_bit);
#elif defined(CONFIG_OMAP44XX)
		if (dev->clkctrl_addr)
			omap_clk_enable((u32 *)dev->clkctrl_addr,
					MODULE_CLKCTRL_MODULEMODE_HW_AUTO);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
		dev->clk_enabled = 1;
		GPIO_DPRINT(gpio, "GPIO%d clock enabled\n", (gpio >> 5) + 1);
	}
	return &omap_gpio_bank[gpio >> 5];
}

static inline int get_gpio_index(int gpio)
{
	return gpio & 0x1f;
}

static inline int gpio_valid(int gpio)
{
	if (gpio < 0)
		return -1;
	if (gpio < 192)
		return 0;
	return -1;
}

static int check_gpio(int gpio)
{
	if (gpio_valid(gpio) < 0) {
#if 0
		printf("ERROR : check_gpio: invalid GPIO %d\n", gpio);
#else
		GPIO_PRINT(gpio, "bad GPIO\n");
#endif
		return -1;
	}
	return 0;
}

static void _set_gpio_direction(const struct gpio_bank *bank, int gpio,
				int is_input)
{
	void *reg = bank->base;
	u32 l;

	switch (bank->method) {
	case METHOD_GPIO_24XX:
		reg += OMAP_GPIO_OE;
		break;
	default:
		return;
	}
	l = __raw_readl(reg);
	if (is_input)
		l |= 1 << gpio;
	else
		l &= ~(1 << gpio);
	__raw_writel(l, reg);
}

/**
 * Get the direction of the GPIO by reading the GPIO_OE register
 * corresponding to the specified bank.
 */
static int _get_gpio_direction(const struct gpio_bank *bank, int gpio)
{
	void *reg = bank->base;
	u32 v;

	switch (bank->method) {
	case METHOD_GPIO_24XX:
		reg += OMAP_GPIO_OE;
		break;
	default:
		return -1;
	}

	v = __raw_readl(reg);

	if (v & (1 << gpio))
		return OMAP_GPIO_DIR_IN;
	else
		return OMAP_GPIO_DIR_OUT;
}

static void _set_gpio_dataout(const struct gpio_bank *bank, int gpio,
				int enable)
{
	void *reg = bank->base;
	u32 l = 0;

	switch (bank->method) {
	case METHOD_GPIO_24XX:
		if (enable)
			reg += OMAP_GPIO_SETDATAOUT;
		else
			reg += OMAP_GPIO_CLEARDATAOUT;
		l = 1 << gpio;
		break;
	default:
#if 0
		printf("omap3-gpio unknown bank method %s %d\n",
		       __FILE__, __LINE__);
#else
		GPIO_DPRINT(gpio, "bad GPIO bank method %d\n", bank->method);
#endif
		return;
	}
	__raw_writel(l, reg);
}

/**
 * Set value of the specified gpio
 */
int gpio_set_value(unsigned gpio, int value)
{
	const struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return -1;
	bank = get_gpio_bank(gpio);
	_set_gpio_dataout(bank, get_gpio_index(gpio), value);

	return 0;
}

/**
 * Get value of the specified gpio
 */
int gpio_get_value(unsigned gpio)
{
	const struct gpio_bank *bank;
	void *reg;
	int input;

	if (check_gpio(gpio) < 0)
		return -1;
	bank = get_gpio_bank(gpio);
	reg = bank->base;
	switch (bank->method) {
	case METHOD_GPIO_24XX:
		input = _get_gpio_direction(bank, get_gpio_index(gpio));
		switch (input) {
		case OMAP_GPIO_DIR_IN:
			reg += OMAP_GPIO_DATAIN;
			break;
		case OMAP_GPIO_DIR_OUT:
			reg += OMAP_GPIO_DATAOUT;
			break;
		default:
			return -1;
		}
		break;
	default:
		return -1;
	}
	return (__raw_readl(reg)
			& (1 << get_gpio_index(gpio))) != 0;
}

/**
 * Set gpio direction as input
 */
int gpio_direction_input(unsigned gpio)
{
	const struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return -1;

	bank = get_gpio_bank(gpio);
	_set_gpio_direction(bank, get_gpio_index(gpio), 1);

	return 0;
}

/**
 * Set gpio direction as output
 */
int gpio_direction_output(unsigned gpio, int value)
{
	const struct gpio_bank *bank;

	if (check_gpio(gpio) < 0)
		return -1;

	bank = get_gpio_bank(gpio);
	_set_gpio_dataout(bank, get_gpio_index(gpio), value);
	_set_gpio_direction(bank, get_gpio_index(gpio), 0);

	return 0;
}

/**
 * Request a gpio before using it.
 *
 * NOTE: Argument 'label' is unused.
 */
int gpio_request(unsigned gpio, const char *label)
{
	if (check_gpio(gpio) < 0)
		return -1;

	return 0;
}

/**
 * Reset and free the gpio after using it.
 */
int gpio_free(unsigned gpio)
{
	return 0;
}

void omap_gpio_init(void)
{
	int i;

#if defined(CONFIG_OMAP34XX)

#error "Not Implemented"

#elif defined(CONFIG_OMAP44XX)
	const struct gpio_bank *bank;
	struct omap_gpio_dev *dev;
	u32 reg;

#ifdef CONFIG_TWL6032_POWER
	i = 0;
#else
	i = 1;	/* skip GPIO1 module */
#endif /* CONFIG_TWL6032_POWER */

	for (; i < 6 ; i++) {
		dev = omap_gpio_device + i;

		if (dev->clkctrl_addr) {
			bank = omap_gpio_bank + i;
			reg = (u32)bank->base;

			omap_clk_enable((u32 *)dev->clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_HW_AUTO);
			setbits_le32((u32 *)dev->clkctrl_addr, GPIOx_CLKCTRL_OPTFCLKEN_MASK);

			if (ocp_softreset(reg + OMAP_GPIO_SYSCONFIG, reg + OMAP_GPIO_SYSSTATUS,
					SYSC_SIDLE_SMART_WAKEUP | SYSC_ENWAKEUP | SYSC_AUTOIDLE)) {
				printf("GPIO%d: timedout\n", i + 1);
			}

			GPIO_VPRINT("GPIO%d base: 0x%08x\n", i + 1, reg);
			GPIO_VPRINT("SYSC(0x%08x)=0x%08x\n",
					reg + OMAP_GPIO_SYSCONFIG, __raw_readl(reg + OMAP_GPIO_SYSCONFIG));
			GPIO_VPRINT("SYSS(0x%08x)=0x%08x\n",
					reg + OMAP_GPIO_SYSSTATUS, __raw_readl(reg + OMAP_GPIO_SYSSTATUS));

			clrbits_le32((u32 *)dev->clkctrl_addr, GPIOx_CLKCTRL_OPTFCLKEN_MASK);
			omap_clk_disable((u32 *)dev->clkctrl_addr);
		}
	}

#elif defined(CONFIG_OMAP54XX)

#error "Not Implemented"

#else

#error "Not Implemented"

#endif
}
