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
#include <linux/android_boot.h>

#include "../../../arch/arm/mach-omap2/board-44xx-zeus.h"
#include "../../../arch/arm/mach-omap2/mux.h"
#include "../../../kernel/power/power.h"  

#define INT_HALL 7

struct ah180_data {
	struct input_dev *ah180_input_dev;
	int (*init)(struct sensors_pm_platform_data *pdata);
	void (*exit)(struct sensors_pm_platform_data *pdata);  
	int (*suspend)(struct sensors_pm_platform_data *pdata);
	int (*resume)(struct sensors_pm_platform_data *pdata);
	int irq;
	int setEnable;	
	int hallValue;
};

struct ah180_data *ah180wg7_data;
static int ah180_request_gpio(struct ah180_data *ahd);

static ssize_t ah180_hallvalue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct ah180_data *ahd = ah180wg7_data;

	ret = sprintf(buf, "Current hall value = %d\n", ahd->hallValue);

	return ret;
}

static ssize_t ah180_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct ah180_data *ahd = ah180wg7_data;

	ret = sprintf(buf, "%d\n", ahd->setEnable);

	return ret;
}

static ssize_t ah180_enable_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int item;
	struct ah180_data *ahd = ah180wg7_data;

	sscanf(buf,"%d\n",&item);
	ahd->setEnable = item;

	return count;
}

static DEVICE_ATTR(ah180_enable, S_IRUGO | S_IWUSR | S_IWGRP, ah180_enable_show, ah180_enable_store);
static DEVICE_ATTR(ah180_hallvalue, S_IRUGO, ah180_hallvalue_show, NULL);

static struct attribute *ah180_attributes[] = {
	&dev_attr_ah180_enable.attr,
	&dev_attr_ah180_hallvalue.attr,
	NULL,
};

static struct attribute_group ah180_attr_group = {
    .attrs = ah180_attributes,
};


static irqreturn_t ah180_irq_handler(int irqNo, void *handle)
{
	struct ah180_data *ahd = (struct ah180_data*)handle;
	int curr_hall;

	if(ahd->setEnable == 0) {
		printk("[AH180] hall sensor is not enabled\n");
		return IRQ_HANDLED;
	}

	disable_irq_nosync(ahd->irq);
	printk("[AH180] irq_handler: \n");

	curr_hall = gpio_get_value(INT_HALL);

	if(curr_hall == 1) {
		ahd->hallValue = 1;
		if(get_suspend_state() != 0) {
			printk("  wake up by hall sensor\n");
			input_report_key(ahd->ah180_input_dev, KEY_WAKEUP, 1);
			input_sync(ahd->ah180_input_dev);
			input_report_key(ahd->ah180_input_dev, KEY_WAKEUP, 0);
			input_sync(ahd->ah180_input_dev);
		} else {
			printk("  already waked up\n");
		}
	} else if(curr_hall == 0) {
		ahd->hallValue = 0;
		if(get_suspend_state() != 3) {
			printk("  sleep by hall sensor\n");
			input_report_key(ahd->ah180_input_dev, KEY_SLEEP, 1);
			input_sync(ahd->ah180_input_dev);
			input_report_key(ahd->ah180_input_dev, KEY_SLEEP, 0);
			input_sync(ahd->ah180_input_dev);
		} else {
			printk("  already slept\n");
		}
	} else
		printk("[AH180 error] invalid hall value.\n");
	
	enable_irq(ahd->irq);

	return IRQ_HANDLED;
}

static int ah180_request_gpio(struct ah180_data *ahd)
{
	int err = 0;

	err = omap_mux_init_gpio(INT_HALL, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);
	if (err < 0) {
		printk("[AH180 error] init_gpio with error %d.\n", err);
		return err;
	}

	err = gpio_request(INT_HALL, "INT_HALL");
	if (err < 0) {
		printk("[AH180 error] gpio_request with error %d.\n", err);
		return err;
	} else {
		gpio_direction_input(INT_HALL);
	}

	ahd->irq = OMAP_GPIO_IRQ(INT_HALL);
	if (ahd->irq < 0) {
		printk("[AH180 error] no irq is available.");
		return -EBUSY;
	}

	err = request_threaded_irq(ahd->irq, NULL, ah180_irq_handler, (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING), "ZEUS_HALL_DRIVER_NAME", ahd);
	if (err < 0) {
		printk("[AH180 error] request irq pin %d fail for gpio\n", err);
		return err;
	}

	return 0;
}

static int ah180_probe(struct platform_device *pdev)
{
	struct ah180_data *ahd = NULL;
	int err = 0;
	int boot_reason;

	boot_reason = android_boot_get_reason();
	if (boot_reason != ANDROID_BOOT_REASON_NORMAL) {
		printk("Not normal boot. Stop AH180.\n");
		return err;
	}

	printk("\n[AH180] probe start.\n");

	ahd = kzalloc(sizeof(struct ah180_data), GFP_KERNEL);
	if (!ahd) {
		return -ENOMEM;
	}

   	ahd->ah180_input_dev = input_allocate_device();
	if (!ahd->ah180_input_dev) {
		printk("[AH180 error] could not allocate input device\n");
		goto err_allocate_device;
	}

	input_set_capability(ahd->ah180_input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(ahd->ah180_input_dev, EV_KEY, KEY_SLEEP);

	ahd->ah180_input_dev->name = "AH180";

	err = input_register_device(ahd->ah180_input_dev);
	if (err < 0) {
		printk("[AH180 error] cannot register input device\n");
		goto err_register_device;
	}

	err = sysfs_create_group(&ahd->ah180_input_dev->dev.kobj, &ah180_attr_group);
	if (err !=0) {
		printk("[AH180 error] create sysfs group error\n");
		goto err_sysfs_create_group;		
	}

	err = ah180_request_gpio(ahd);
	if (err !=0) {
		printk("[AH180 error] request gpio error\n");
		goto err_failed;
	}

	platform_set_drvdata(pdev, ahd);

	err = device_init_wakeup(&pdev->dev, 1);
	if (err != 0) {
		printk("[AH180 errpr] device init wakeup error\n");
		goto err_failed;
	}

	ahd->setEnable = 1;
	ahd->hallValue = 1;
	ah180wg7_data = ahd;

	printk("[AH180] probe end.\n");

	return err;

err_failed:
err_sysfs_create_group:
err_register_device:
	input_unregister_device(ahd->ah180_input_dev);
err_allocate_device:
	input_free_device(ahd->ah180_input_dev);
	kfree(ahd);

	return err;
}

static void ah180_shutdown(struct platform_device *pdev)
{
}

static int ah180_suspend(struct device *dev)
{
	struct ah180_data *ahd = dev_get_drvdata(dev);
	printk("ah180 suspend\n");

	if (device_may_wakeup(dev))
		enable_irq_wake(ahd->irq);

	return 0;
}

static int ah180_resume(struct device *dev)
{
	struct ah180_data *ahd = dev_get_drvdata(dev);
	printk("ah180 resume\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(ahd->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ah180_pm_ops, ah180_suspend, ah180_resume);

static struct platform_driver ah180_driver = {
	.probe		= ah180_probe,
	.shutdown	= ah180_shutdown,
	.driver		= {
		.name	= ZEUS_HALL_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm     = &ah180_pm_ops,
	},
};

static struct platform_device ah180_device = {
	.name      = ZEUS_HALL_DRIVER_NAME,
	.id        = 0,
};

static int __init ah180_init(void)
{
	if (platform_device_register(&ah180_device) < 0) {
		printk(KERN_ERR "unable to register AH180 device\n");
	}

	return platform_driver_register(&ah180_driver);
}
module_init(ah180_init);

static void __exit ah180_exit(void)
{
	platform_driver_unregister(&ah180_driver);
	platform_device_unregister(&ah180_device);
}
module_exit(ah180_exit);

MODULE_DESCRIPTION("AH180 Driver");
MODULE_LICENSE("GPL");

