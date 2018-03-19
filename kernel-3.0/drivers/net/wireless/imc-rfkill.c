/*
 * drivers/net/wireless/imc-rfkill.c
 *
 * Driver file for IMC rfkill driver
 *
 * Copyright (C) 2012 Innocomm Mobile
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_data/modem-rfkill.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define POWRSTATE_POLLING_MS        20 /* 20ms delay for polling */

/*------------------- RFKILL DRIVER DEVICE DATA STRUCTURE -------------------*/

struct imc_rfkill_data {
	struct rfkill			*rfkdev;
};


/*------------------------------ RFKILL DRIVER ------------------------------*/

enum {
	RFKILL_IMC_XG6260,
};

static int imc_modem_set_power(struct platform_device *pdev, int enable)
{
	struct modem_rfkill_platform_data *pdata;

	pdata = dev_get_platdata(&pdev->dev);

	/* Control Modem power switch */
	if (enable) {
		if (gpio_is_valid(pdata->reset_pmu_req_gpio))
			gpio_direction_output(pdata->reset_pmu_req_gpio, enable);

		if (gpio_is_valid(pdata->on_off_gpio)) {
			/*
			 * Although there is no min. timing between PWRDWN_N and ON_x
			 * specified in XMM6260 document(max. timing is specified as 400us)
			 * , it is observed that time between PWRDWN_N and ON_x might
			 * shorter than 3us and it is not long enough for XMM6260 to power
			 * on.
			 * So 10ms is added here to make ON event work.
			 */
			msleep(10);
			gpio_direction_output(pdata->on_off_gpio, enable);
		}
	} else {
		if (gpio_is_valid(pdata->on_off_gpio))
			gpio_direction_output(pdata->on_off_gpio, enable);

		if (gpio_is_valid(pdata->reset_pmu_req_gpio))
			gpio_direction_output(pdata->reset_pmu_req_gpio, enable);
	}

	return 0;
}

static int imc_modem_check_status(struct platform_device *pdev, int enable)
{
	int modem_state;
	struct modem_rfkill_platform_data *pdata;
	int i = 0;

	pdata = dev_get_platdata(&pdev->dev);

	/* Monitor Modem power state */
	do {
		modem_state = gpio_get_value(pdata->pwrstate_gpio);
		if (unlikely(enable == modem_state)) {
			pr_info("%s() Modem powered %s\n", __func__, enable ? "on" :
								   "off");
			return 0;
		}
		msleep(POWRSTATE_POLLING_MS);
		i += POWRSTATE_POLLING_MS;
	} while (i < pdata->power_on_pwrstate_timeout_ms);

	pr_err("%s() Modem does not powered %s\n", __func__, enable ? "on" :
								   "off");
	return -1;
}

static int imc_rfkill_set_block(void *data, bool blocked)
{
	struct platform_device *pdev = data;

	pr_info("%s(): blocked=%d\n", __func__, blocked);

	return imc_modem_set_power(pdev, !blocked);
}

static void imc_rfkill_poll(struct rfkill *rfkill, void *data)
{
	struct modem_rfkill_platform_data *pdata;
	struct platform_device *pdev = data;

	pdata = dev_get_platdata(&pdev->dev);

	if (gpio_get_value(pdata->pwrstate_gpio)) {
		rfkill_set_sw_state(rfkill, 0);
	} else {
		rfkill_set_sw_state(rfkill, 1);
	}
}

/*
 * Switch-on or switch-off the modem and return modem status
 * Pre-requisites:
 * - Modem Vbat supplied with
 * - MDM_ONSWC and MDM_PWRSTATE gpio reserved and PAD configured
 */
static int modem_switch(struct platform_device *pdev, int new_state)
{
	int modem_pwrrst_ca;

	imc_modem_set_power(pdev, new_state);
	modem_pwrrst_ca = imc_modem_check_status(pdev, new_state);

	if (modem_pwrrst_ca == 0)
		return 1;

	return 0;
}

static irqreturn_t imc_rfkill_pwrstate_isr(int irq, void *dev_id)
{
	struct modem_rfkill_platform_data *pdata;
	struct imc_rfkill_data *rfkill;
	int value;
	struct platform_device *pdev = dev_id;

	pdata = dev_get_platdata(&pdev->dev);
	rfkill = platform_get_drvdata(pdev);

	if (!pdata) {
		printk(KERN_INFO "\nCRASH: NULL access in imc_rfkill_pwrstate_isr\n");
		goto exit;
	}

	value = gpio_get_value(pdata->pwrstate_gpio);

	if (value == 0) {
		rfkill_set_sw_state(rfkill->rfkdev, 1);
		pr_info("===================IMC: MODEM is DOWN!!!=================\n");
	} else {
		rfkill_set_sw_state(rfkill->rfkdev, 0);
		pr_info("===================IMC: MODEM is UP!!!=================\n");
	}

exit:
	return IRQ_HANDLED;
}

static irqreturn_t imc_rfkill_wakeup_isr(int irq, void *dev_id)
{
	struct modem_rfkill_platform_data *pdata;
	int value;
	struct platform_device *pdev = dev_id;

	pdata = dev_get_platdata(&pdev->dev);

	if (!pdata) {
		printk(KERN_INFO "\nCRASH: NULL access in imc_rfkill_wakeup_isr\n");
		goto exit;
	}

	value = gpio_get_value(pdata->wakeup_gpio);

	if (value != 1)
		pr_info("=====Modem has gave up wakeup AP====\n");

exit:
	return IRQ_HANDLED;
}

/*
 * Modem is tentatively switched ON to detect it then switched back to OFF
 * Prerequisite: IO pads enabled
 */
bool __init is_modem_detected(struct platform_device *pdev)
{
	int modem_enabled;
	bool modem_detected = false;
	struct modem_rfkill_platform_data *pdata;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	pdata = dev_get_platdata(&pdev->dev);
	/* Prepare MDM_STATE to get modem status */
	if (unlikely(!gpio_is_valid(pdata->pwrstate_gpio) ||
		     gpio_request(pdata->pwrstate_gpio, "MODEM POWER STATE")
		     < 0)) {
		pr_err("%s: Cannot control pwrstate_gpio %d\n",
			__func__, pdata->pwrstate_gpio);
		goto err_pwrstate;
	}
	gpio_direction_input(pdata->pwrstate_gpio);

	if (unlikely(gpio_get_value(pdata->pwrstate_gpio)))
		pr_warn("%s: Modem MOD_PWR_STATUS is already ON\n", __func__);

	/* Enable Modem ONSWC */
	if (gpio_is_valid(pdata->on_off_gpio) &&
		   gpio_request(pdata->on_off_gpio, "MODEM SWITCH ON") < 0) {
		pr_err("Cannot control on_off_gpio %d\n", pdata->on_off_gpio);
		goto err_onswc;
	}

	/* Enable Modem ONSWC */
	if (gpio_is_valid(pdata->reset_pmu_req_gpio) &&
		   gpio_request(pdata->reset_pmu_req_gpio, "MODEM PMU RST") < 0) {
		pr_err("Cannot control reset_pmu_req_gpio %d\n", pdata->reset_pmu_req_gpio);
		goto err_pmu_rst;
	}

	modem_detected = modem_switch(pdev, 1);
	pr_info("Modem %sdetected\n", modem_detected ? "" : "NOT ");

	/* Disable Modem ONSWC */
	modem_enabled = modem_switch(pdev, 0);
	if (unlikely(modem_detected && modem_enabled))
		pr_err("%s: Modem cannot be switched-off\n", __func__);

	goto exit;

err_pmu_rst:
	gpio_free(pdata->on_off_gpio);
err_onswc:
	gpio_free(pdata->pwrstate_gpio);
err_pwrstate:
exit:

	return modem_detected;
}

static const struct rfkill_ops imc_rfkill_ops = {
	.set_block = imc_rfkill_set_block,
	.poll = imc_rfkill_poll,
};

static int __init imc_rfkill_probe(struct platform_device *pdev)
{
	struct imc_rfkill_data *rfkill;
	struct rfkill *rfkdev;
	int ret = 0;
	struct modem_rfkill_platform_data *pdata;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	pdata = dev_get_platdata(&pdev->dev);

	/* Configure pad muxing for modem control */
	if (pdata->io_pads_enable() < 0)
		goto error0;

	is_modem_detected(pdev); /* Test if a modem is connected */

	/* Enable Modem wakeup capability (e.g incoming call) */
	if (pdata->io_wakeup_enable() < 0)
		goto error1;

	if (pdata->ipc_init() < 0) /* Configure IPC */
		goto error1;

	rfkill = kzalloc(sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill) {
		dev_err(&pdev->dev,
			"%s: no memory to alloc driver data\n", __func__);
		ret = -ENOMEM;
		goto error1;
	}

	platform_set_drvdata(pdev, rfkill);

	/* WWAN rfkill device registration */
	rfkill->rfkdev = rfkill_alloc(pdev->name,
					&pdev->dev,
					RFKILL_TYPE_WWAN,
					&imc_rfkill_ops,
					pdev);
	rfkdev = rfkill->rfkdev;
	if (!rfkdev) {
		dev_err(&pdev->dev,
			"%s: Error allocating modem rfkdev\n", __func__);
		ret = -ENOMEM;
		goto error2;
	}

	/* S/W blocked by default, persistent */
	rfkill_init_sw_state(rfkdev, 1);
	ret = rfkill_register(rfkdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Error registering modem rfkdev: %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto error3;
	}

	/* hardware unblocked */
	if (rfkill->rfkdev)
		rfkill_set_hw_state(rfkdev, 0);

	ret = request_irq(pdata->pwrstate_irq,
				&imc_rfkill_pwrstate_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"IMC_PWRSTATE_IRQ",
				pdev);
	dev_info(&pdev->dev, "request_irq(IMC_PWRSTATE_IRQ) result=%d!\n", ret);

	ret = enable_irq_wake(pdata->pwrstate_irq);
	dev_info(&pdev->dev, "enable_irq_wake(IMC_PWRSTATE_IRQ) result=%d!\n", ret);

	if (gpio_is_valid(pdata->active_gpio)) {
		gpio_request(pdata->active_gpio, "AP ACTIVE STATE");
		gpio_direction_output(pdata->active_gpio, 1);
	}

	if (gpio_is_valid(pdata->wakeup_gpio)) {
		gpio_request(pdata->wakeup_gpio, "MODEM WAKEUP AP");
		gpio_direction_input(pdata->wakeup_gpio);

		ret = request_irq(pdata->wakeup_irq,
					&imc_rfkill_wakeup_isr,
					IRQF_TRIGGER_RISING,
					"IMC_WAKEUP_IRQ",
					pdev);
		dev_info(&pdev->dev, "request_irq(IMC_WAKEUP_IRQ) result=%d!\n", ret);

		ret = enable_irq_wake(pdata->wakeup_irq);
		dev_info(&pdev->dev, "enable_irq_wake(IMC_WAKEUP_IRQ) result=%d!\n", ret);
	}

	return 0;

error3:
	rfkill_destroy(rfkdev);
error2:
	kfree(rfkill);
error1:
	pdata->io_pads_disable();
error0:
	dev_err(&pdev->dev, "Error %d returned by %s\n", ret, __func__);
	return ret;
}


static int imc_rfkill_remove(struct platform_device *pdev)
{
	struct imc_rfkill_data *rfkill = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	/* XXX IO pads could be disabled but before one should ensure that
	 * io_pads_disable is not be part of the __init section
	 */

	rfkill_unregister(rfkill->rfkdev);
	rfkill_destroy(rfkill->rfkdev);
	kfree(rfkill);

	return 0;
}

static int imc_rfkill_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct modem_rfkill_platform_data *pdata;

	pdata = dev_get_platdata(&pdev->dev);

	if (pdata && gpio_is_valid(pdata->active_gpio)) {
		/* Notify modem that AP is going to suspend */
		gpio_set_value(pdata->active_gpio, 0);
	}

	return 0;
}

static int imc_rfkill_resume(struct platform_device *pdev)
{
	struct modem_rfkill_platform_data *pdata;

	pdata = dev_get_platdata(&pdev->dev);

	if (pdata && gpio_is_valid(pdata->active_gpio)) {
		/* Notify modem that AP is resumed */
		gpio_set_value(pdata->active_gpio, 1);
	}

	return 0;
}

/* List of device names supported by this driver */
static struct platform_device_id imc_rfkill_id_table[] = {
	{"imc-xg6260", RFKILL_IMC_XG6260},
	{},
};
MODULE_DEVICE_TABLE(platform, imc_rfkill_id_table);

/* Set the platform driver __refdata to silence spurious section mismatches */
static struct platform_driver __refdata imc_rfkill_driver = {
	.probe = imc_rfkill_probe,
	.remove = __devexit_p(imc_rfkill_remove),
	.suspend = imc_rfkill_suspend,
	.resume = imc_rfkill_resume,
	.driver = {
		.name = "imc_rfkill",
		.owner = THIS_MODULE,
	},
	.id_table = imc_rfkill_id_table,
};


static int __init imc_rfkill_init(void)
{
	pr_debug("%s\n", __func__);

	return platform_driver_register(&imc_rfkill_driver);
}

static void __exit imc_rfkill_exit(void)
{
	pr_debug("%s\n", __func__);

	platform_driver_unregister(&imc_rfkill_driver);
}

module_init(imc_rfkill_init);
module_exit(imc_rfkill_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments Inc");
MODULE_DESCRIPTION("RFKILL driver for IMC modem");
