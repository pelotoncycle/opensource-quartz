/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/max3218.h>

#include "../../../arch/arm/mach-omap2/board-44xx-quartz.h"
#include "../../../arch/arm/mach-omap2/mux.h"

#define GPIO_RS232_DET_N	52
#define GPIO_RS232_SHDN_N	56
#define	GPIO_RS232_EN		59

#define GPIO_RS232_DET_DEBOUNCE_TIME	7936 /* us */

#define PIN_HIGH 1
#define PIN_LOW	 0

struct max3218_data *global_max3218_data;

static void max3218_gpio_settings(struct max3218_data *maxd)
{
	mutex_lock(&maxd->lock);

	maxd->rs232_det_n = gpio_get_value(GPIO_RS232_DET_N);
	if (maxd->rs232_det_n == PIN_LOW) { // if earphone is plugged
		if (!maxd->plugged) { /* was unplugged */
			if (maxd->rs232_shdn_n == PIN_LOW) {
				maxd->rs232_shdn_n = PIN_HIGH; // power on
				gpio_set_value(GPIO_RS232_SHDN_N, maxd->rs232_shdn_n);
			}
			if (maxd->rs232_en == PIN_LOW) {
				maxd->rs232_en = PIN_HIGH; // switch on
				gpio_set_value(GPIO_RS232_EN, maxd->rs232_en);
			}
			maxd->plugged = true;
			switch_set_state(&maxd->detect_switch, 1);
			dev_info(maxd->dev, "UART plugged: shdn_n=%d, en=%d, det_n=%d\n",
				maxd->rs232_shdn_n, maxd->rs232_en, maxd->rs232_det_n);
		}
	} else if (maxd->rs232_det_n == PIN_HIGH) { // if earphone is unplugged
		if (maxd->plugged) { /* was plugged */
			maxd->plugged = false;
			if (maxd->rs232_shdn_n == PIN_HIGH) {
				maxd->rs232_shdn_n = PIN_LOW; // power off
				gpio_set_value(GPIO_RS232_SHDN_N, maxd->rs232_shdn_n);
			}
			if (maxd->rs232_en == PIN_HIGH) {
				maxd->rs232_en = PIN_LOW; // switch off
				gpio_set_value(GPIO_RS232_EN, maxd->rs232_en);
			}
			switch_set_state(&maxd->detect_switch, 0);
			dev_info(maxd->dev, "UART unplugged: shdn_n=%d, en=%d, det_n=%d\n",
				maxd->rs232_shdn_n, maxd->rs232_en, maxd->rs232_det_n);
		}
	}

	mutex_unlock(&maxd->lock);
}

static irqreturn_t max3218_irq_det_n_handler(int irqNo, void *handle)
{
	struct max3218_data *maxd = (struct max3218_data*)handle;

	dev_vdbg(maxd->dev, "interrupt\n");

	/* sleep 384ms for the SW debounce */
	msleep(384);

	max3218_gpio_settings(maxd);

	return IRQ_HANDLED;
}

static int max3218_request_gpio(struct max3218_data *maxd)
{
	int err = 0;

	// setting for GPIO_RS232_SHDN_N
	err = omap_mux_init_gpio(GPIO_RS232_SHDN_N, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
	if (err < 0) {
		dev_err(maxd->dev, "[error] init gpio_rs232_shdn_n with error %d.\n", err);
		return err;
	}
	err = gpio_request(GPIO_RS232_SHDN_N, "GPIO_RS232_SHDN_N");
	if (err < 0) {
		dev_err(maxd->dev, "[error] gpio_rs232_shdn_n request with error %d.\n", err);
		return err;
	} else {
		gpio_direction_output(GPIO_RS232_SHDN_N, maxd->rs232_shdn_n);
	}

	// setting for GPIO_RS232_EN
	err = omap_mux_init_gpio(GPIO_RS232_EN, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
	if (err < 0) {
		dev_err(maxd->dev, "[error] init gpio_rs232_en with error %d.\n", err);
		return err;
	}
	err = gpio_request(GPIO_RS232_EN, "GPIO_RS232_EN");
	if (err < 0) {
		dev_err(maxd->dev, "[error] gpio_rs232_en request with error %d.\n", err);
		return err;
	} else {
		gpio_direction_output(GPIO_RS232_EN, maxd->rs232_en);
	}

	// setting for GPIO_RS232_DET_N
	err = omap_mux_init_gpio(GPIO_RS232_DET_N, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);
	if (err < 0) {
		dev_err(maxd->dev, "[error] init gpio_rs232_det_n with error %d.\n", err);
		return err;
	}
	err = gpio_request(GPIO_RS232_DET_N, "GPIO_RS232_DET_N");
	if (err < 0) {
		dev_err(maxd->dev, "[error] gpio_rs232_det_n request with error %d.\n", err);
		return err;
	} else {
		gpio_direction_input(GPIO_RS232_DET_N);
		gpio_set_debounce(GPIO_RS232_DET_N, GPIO_RS232_DET_DEBOUNCE_TIME);
		msleep(10);
	}
	maxd->irq_det_n = OMAP_GPIO_IRQ(GPIO_RS232_DET_N);
	if (maxd->irq_det_n < 0) {
		dev_err(maxd->dev, "[error] no irq for det_n is available.");
		return -EBUSY;
	}
	err = request_threaded_irq(maxd->irq_det_n, NULL, max3218_irq_det_n_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"quartz_max3218", maxd);
	if (err < 0) {
		dev_err(maxd->dev, "[error] request irq_det_n pin %d fail for gpio\n", err);
		return err;
	}
	disable_irq(maxd->irq_det_n);

	return 0;
}

static int max3218_probe(struct platform_device *pdev)
{
	struct max3218_data *maxd = NULL;
	int err = 0;

	dev_info(&pdev->dev, "probe start.\n");

	maxd = kzalloc(sizeof(struct max3218_data), GFP_KERNEL);
	if (!maxd) {
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, maxd);
	maxd->max_pdev = pdev;
	maxd->dev = &pdev->dev;
	global_max3218_data = maxd;

	mutex_init(&maxd->lock);
	maxd->detect_switch.name = "uart";
	err = switch_dev_register(&maxd->detect_switch);
	if (err) {
		dev_err(&pdev->dev, "uart switch err %d\n", err);
		maxd->detect_switch.name = NULL;
		goto err_failed;
	}

	maxd->rs232_shdn_n = PIN_LOW; 
	maxd->rs232_en = PIN_LOW; 
	maxd->rs232_det_n = PIN_HIGH; 

	err = max3218_request_gpio(maxd);
	if (err != 0) {
		goto err_failed;
	}

	max3218_gpio_settings(maxd);

	enable_irq(maxd->irq_det_n);

	dev_info(&pdev->dev, "probe end.\n");

	return err;

err_failed:
	if (maxd->detect_switch.name)
		switch_dev_unregister(&maxd->detect_switch);
	mutex_destroy(&maxd->lock);
	kfree(maxd);

	return err;
}

static void max3218_shutdown(struct platform_device *pdev)
{
	struct max3218_data *maxd = global_max3218_data;

	disable_irq(maxd->irq_det_n);

	mutex_lock(&maxd->lock);
	if (maxd->rs232_en == PIN_HIGH) {
		maxd->rs232_en = PIN_LOW;
		gpio_set_value(GPIO_RS232_EN, maxd->rs232_en);
	}
	if (maxd->rs232_shdn_n == PIN_HIGH) {
		maxd->rs232_shdn_n = PIN_LOW;
		gpio_set_value(GPIO_RS232_EN, maxd->rs232_shdn_n);
	}
	mutex_unlock(&maxd->lock);
}

static int max3218_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct max3218_data *maxd = global_max3218_data;

	disable_irq(maxd->irq_det_n);
	synchronize_irq(maxd->irq_det_n);

	/* simulate to unplug the UART */
	mutex_lock(&maxd->lock);
	maxd->plugged = false;
	maxd->rs232_en = PIN_LOW;
	gpio_set_value(GPIO_RS232_EN, maxd->rs232_en);
	maxd->rs232_shdn_n = PIN_LOW;
	gpio_set_value(GPIO_RS232_SHDN_N, maxd->rs232_shdn_n);
	mutex_unlock(&maxd->lock);

	dev_info(maxd->dev, "UART suspended: plugged=%d\n", maxd->plugged);

	return 0;
}

static int max3218_resume(struct platform_device *pdev)
{
	struct max3218_data *maxd = global_max3218_data;

	max3218_gpio_settings(maxd);

	dev_info(maxd->dev, "UART resumed: plugged=%d\n", maxd->plugged);

	enable_irq(maxd->irq_det_n);

	return 0;
}

static struct platform_driver max3218_platform_driver = {
	.probe		= max3218_probe,
	.shutdown	= max3218_shutdown,
	.suspend	= max3218_suspend,
	.resume		= max3218_resume,
	.driver		= {
		.name	= QUARTZ_MAX3218_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init max3218_init(void)
{
	return platform_driver_register(&max3218_platform_driver);
}
#if 0
module_init(max3218_init);
#else
late_initcall(max3218_init);
#endif

static void __exit max3218_exit(void)
{
	platform_driver_unregister(&max3218_platform_driver);
}
module_exit(max3218_exit);

MODULE_DESCRIPTION("MAX3218 Driver");
MODULE_LICENSE("GPL");

