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
//#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/usb.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include "board-ant-ldo.h"

/**************************************************************************************************/

/*#define REGISTER_ANT_DUMMY_USB_DRIVER*/

/**************************************************************************************************/

static DEFINE_MUTEX(ant_mutex);

static struct regulator *ant_power;
static struct kobject *ant_sysfs_kobj;
static int and_power_enabled = 0;

static struct wake_lock ant_reset_wakelock;

/**************************************************************************************************/

void quartz_antplus_set_power(int enable)
{
	if (!ant_power) {
		pr_err("ant power: no '%s' regulator\n", ANT_DEVICE_POWER_NAME);
		return;
	}

	mutex_lock(&ant_mutex);

	if (enable) {
		/* enable */
		if (!and_power_enabled) {
			int ret = regulator_enable(ant_power);
			if (ret) {
				pr_err("ant power: enable err %d\n", ret);
				return;
			} else {
				and_power_enabled = 1;
				pr_info("ant power: enabled\n");
				msleep(10);
			}
		} else {
			pr_info("ant power: was enabled\n");
		}
	} else {
		/* disable */
		if (and_power_enabled) {
			int ret = regulator_disable(ant_power);
			if (ret) {
				pr_err("ant power: disable err %d\n", ret);
				return;
			} else {
				and_power_enabled = 0;
				pr_info("ant power: disabled\n");
			}
		} else {
			pr_info("ant power: was disabled\n");
		}
	}

	mutex_unlock(&ant_mutex);
}
EXPORT_SYMBOL_GPL(quartz_antplus_set_power);

static void ant_reset(void)
{
	/* Hold a 30-sec wakelock to avoid the immediate Linux PM suspend */
	wake_lock_timeout(&ant_reset_wakelock, 30 * HZ);

	quartz_antplus_set_power(0);
	msleep(350);
	quartz_antplus_set_power(1);
}

/**************************************************************************************************/

static ssize_t ant_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t s;

	mutex_lock(&ant_mutex);
	if (ant_power) {
		if (and_power_enabled)
			s = sprintf(buf, "enabled\n");
		else
			s = sprintf(buf, "disabled\n");
	} else {
		s = sprintf(buf, "no '%s' regulator\n", ANT_DEVICE_POWER_NAME);
	}
	mutex_unlock(&ant_mutex);

	return s;
}

static ssize_t ant_power_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int enable = 0;

	if (sscanf(buf, "%d", &enable) != 1) {
		pr_err("%s: invalid value (0/1)\n", __func__);
		return -EINVAL;
	}

	quartz_antplus_set_power(enable);

	return count;
}

static ssize_t ant_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON) && defined(REGISTER_ANT_DUMMY_USB_DRIVER)
	quartz_antplus_set_power(0);
#else
	ant_reset();
#endif
	return sprintf(buf, "quartz usb ant+ reset\n");
}

static DEVICE_ATTR(power, S_IRUGO | S_IWUSR, ant_power_show, ant_power_store);
static DEVICE_ATTR(reset, S_IRUGO, ant_reset_show, NULL);

static void ant_sysfs_init(struct platform_device *pdev)
{
	ant_sysfs_kobj = kobject_create_and_add("quartz_ant", NULL) ;
	if (ant_sysfs_kobj == NULL) {
		dev_err(&pdev->dev, "kobject err\n");
	} else {
		int ret;

		ret = sysfs_create_file(ant_sysfs_kobj, &dev_attr_power.attr);
		if (ret)
			dev_err(&pdev->dev, "power sysfs err %d\n", ret);
		ret = sysfs_create_file(ant_sysfs_kobj, &dev_attr_reset.attr);
		if (ret)
			dev_err(&pdev->dev, "reset sysfs err %d\n", ret);
	}
}

static void ant_sysfs_deinit(void)
{
	if (ant_sysfs_kobj) {
		sysfs_remove_file(ant_sysfs_kobj, &dev_attr_reset.attr);
		sysfs_remove_file(ant_sysfs_kobj, &dev_attr_power.attr);
		kobject_del(ant_sysfs_kobj);
	}
}

static int ant_probe(struct platform_device *pdev)
{
	dev_vdbg(&pdev->dev, "%s()\n", __func__);

	ant_power = regulator_get(NULL, ANT_DEVICE_POWER_NAME);
	if (IS_ERR(ant_power)) {
		dev_err(&pdev->dev, "get regulator %s err %ld\n", ANT_DEVICE_POWER_NAME, PTR_ERR(ant_power));
		ant_power = NULL;
		return -EINVAL;
	} else {
		quartz_antplus_set_power(1);
	}

	ant_sysfs_init(pdev);

	wake_lock_init(&ant_reset_wakelock, WAKE_LOCK_SUSPEND, "ant_reset");

	dev_dbg(&pdev->dev, "power loaded\n");

	return 0;
}

static int ant_remove(struct platform_device *pdev)
{
	dev_vdbg(&pdev->dev, "%s()\n", __func__);

	ant_sysfs_deinit();
	regulator_put(ant_power);

	return 0;
}

static void ant_shutdown(struct platform_device *pdev)
{
	dev_vdbg(&pdev->dev, "%s()\n", __func__);
	quartz_antplus_set_power(0);
}

#ifndef CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON
static int ant_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_vdbg(&pdev->dev, "%s()\n", __func__);

	quartz_antplus_set_power(0);

	return 0;
}

static int ant_resume(struct platform_device *pdev)
{
	dev_vdbg(&pdev->dev, "%s()\n", __func__);

	quartz_antplus_set_power(1);

	return 0;
}
#endif

static struct platform_driver ant_driver = {
	.probe		= ant_probe,
	.remove		= ant_remove,
	.shutdown	= ant_shutdown,
#ifndef CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON
	.suspend	= ant_suspend,
	.resume		= ant_resume,
#endif
	.driver		= {
		.name	= ANT_USB_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

/**************************************************************************************************/

#if defined(CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON) && defined(REGISTER_ANT_DUMMY_USB_DRIVER)
static int ant_usb_probe(struct usb_interface *interface,
			       const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(interface);

	dev_vdbg(&usbdev->dev, "%s()\n", __func__);

    return 0;
}

static void ant_usb_disconnect(struct usb_interface *interface)
{
	struct usb_device *usbdev = interface_to_usbdev(interface);

	dev_vdbg(&usbdev->dev, "%s()\n", __func__);
}

#if 0
static int ant_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);

	dev_vdbg(&usbdev->dev, "%s()\n", __func__);

	return 0;
}

static int ant_usb_resume(struct usb_interface *intf)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);

	dev_vdbg(&usbdev->dev, "%s()\n", __func__);

	return 0;
}

static int ant_usb_reset_resume(struct usb_interface *intf)
{
	struct usb_device *usbdev = interface_to_usbdev(intf);

	dev_vdbg(&usbdev->dev, "%s()\n", __func__);

	return 0;
}
#endif

static const struct usb_device_id id_table[] = {
	{USB_DEVICE(0x0fcf, 0x1008)},
	{ }				/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver ant_usb_driver = {
	.name			= "ant-usb",
	.probe			= ant_usb_probe,
	.disconnect		= ant_usb_disconnect,
	.id_table		= id_table,
#if 0
	.suspend		= ant_usb_suspend,
	.resume			= ant_usb_resume,
	.reset_resume	= ant_usb_reset_resume,
#endif
	.supports_autosuspend	= 0,
};
#endif

/**************************************************************************************************/

static int __init ant_init(void)
{
	int ret;

	ret = platform_driver_register(&ant_driver);
	if (ret) {
		pr_err("%s: platform_driver_register err %d\n",  __func__, ret);
		return ret;
	}

#if defined(CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON) && defined(REGISTER_ANT_DUMMY_USB_DRIVER)
	ret = usb_register(&ant_usb_driver);
	if (ret) {
		pr_err("%s: usb_register err %d\n", __func__, ret);
		return ret;
	}
#endif

	return 0;
}

static void __exit ant_exit(void)
{
#if defined(CONFIG_MACH_OMAP_USB_ANT_POWER_ALWAYS_ON) && defined(REGISTER_ANT_DUMMY_USB_DRIVER)
	usb_deregister(&ant_usb_driver);
#endif
	platform_driver_unregister(&ant_driver);
}

fs_initcall(ant_init);
module_exit(ant_exit);

MODULE_DESCRIPTION("ANT+ USB Driver");
MODULE_LICENSE("GPL");
