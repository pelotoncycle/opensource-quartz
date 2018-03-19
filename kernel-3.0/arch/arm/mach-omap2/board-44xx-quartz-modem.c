/*
 * Board support file for OMAP44xx Quartz.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Author: 
 *	Dan Murphy <dmurphy@ti.com>
 *	John.Lin <john.lin@innocomm.com
 *
 * Based on mach-omap2/board-44xx-quartz-modem.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "board-44xx-quartz.h"
#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)

#include "mux.h"

#define GPIO_MODEM_POWER	59
#define GPIO_MODEM_RESET	56
#define GPIO_MODEM_W_DISABLE	47
#define GPIO_MODEM_USIF_SW	49

static int __init quartz_modem_late_init(void)
{
	printk(KERN_INFO "%s()\n", __FUNCTION__);

	gpio_set_value(GPIO_MODEM_POWER, 1);
	msleep(10);
	gpio_set_value(GPIO_MODEM_RESET, 1);

	/* Amazon     1:USB enable, 0:USB disable */
	gpio_set_value(GPIO_MODEM_W_DISABLE, 1);

	return 0;
}

//late_initcall(quartz_modem_late_init);
#endif

void __init quartz_modem_init(void)
{
#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
	int status = 0;

	printk(KERN_INFO "%s()\n", __FUNCTION__);

	omap_mux_init_gpio(GPIO_MODEM_POWER, OMAP_PIN_OUTPUT |
		OMAP_PIN_OFF_NONE);

	status = gpio_request(GPIO_MODEM_POWER, "modem_power");
	if (status) {
		printk(KERN_ERR "Request GPIO %d failed=%d\n", GPIO_MODEM_POWER, status);
	}

	gpio_direction_output(GPIO_MODEM_POWER, 0);

	omap_mux_init_gpio(GPIO_MODEM_RESET, OMAP_PIN_OUTPUT |
		OMAP_PIN_OFF_NONE);

	status = gpio_request(GPIO_MODEM_RESET, "modem_reset");
	if (status) {
		printk(KERN_ERR "Request GPIO %d failed=%d\n", GPIO_MODEM_RESET, status);
	}

	gpio_direction_output(GPIO_MODEM_RESET, 0);

	/* Configure GPIO for controlling Amazon USB function */
	omap_mux_init_gpio(GPIO_MODEM_W_DISABLE, OMAP_PIN_OUTPUT);
	status = gpio_request(GPIO_MODEM_W_DISABLE, "modem_usb_en");
	if (status) {
		printk(KERN_ERR "Request GPIO %d failed=%d\n", GPIO_MODEM_W_DISABLE, status);
	}

	gpio_direction_output(GPIO_MODEM_W_DISABLE, 0);

	/* Disable USIF Switch */
	omap_mux_init_gpio(GPIO_MODEM_USIF_SW, OMAP_PIN_OUTPUT);
	status = gpio_request(GPIO_MODEM_USIF_SW, "modem_usif_sw");
	if (status) {
		printk(KERN_ERR "Request GPIO %d failed=%d\n", GPIO_MODEM_USIF_SW, status);
	}

	gpio_direction_output(GPIO_MODEM_USIF_SW, 0);
#endif
}
