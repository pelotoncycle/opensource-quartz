
/*
 *  Innocomm Quartz USB Touch Power Control
 */
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include "../../../arch/arm/mach-omap2/board-44xx-quartz.h"

#define DONT_USE_EARLYSUSPEND_PM

/* 0:Don't turn off the power; 1:Turn off the power */
#define QUARTZ_TOUCH_PM_INIT_VALUE	0

struct quartz_ts_data {
	struct platform_device *pdevice;
	struct quartz_ts_pdata *pdata; // platform data
#if !defined(DONT_USE_EARLYSUSPEND_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct kobject *debug_kobj;
	bool pm;
	struct wake_lock wakelock;
};

struct quartz_ts_data *quartz_ts;

#if !defined(DONT_USE_EARLYSUSPEND_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
static void quartz_ts_early_suspend(struct early_suspend *h)
{
	struct quartz_ts_data *ts;

	ts = container_of(h, struct quartz_ts_data, early_suspend);

	if (ts && ts->pdata && ts->pdata->power_disable && ts->pm) {
		dev_info(&ts->pdevice->dev, "%s()\n", __func__);
		ts->pdata->power_disable(ts->pdevice);
	}
}

static void quartz_ts_late_resume(struct early_suspend *h)
{
	struct quartz_ts_data *ts;

	ts = container_of(h, struct quartz_ts_data, early_suspend);

	if (ts && ts->pdata && ts->pdata->power_enable && ts->pm) {
		dev_info(&ts->pdevice->dev, "%s()\n", __func__);
		ts->pdata->power_enable(ts->pdevice);
	}
}
#else
static int quartz_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct quartz_ts_data *ts = dev_get_drvdata(&pdev->dev);

	if (ts && ts->pdata && ts->pdata->power_disable && ts->pm) {
		ts->pdata->power_disable(ts->pdevice);
	}

	return 0;
}

static int quartz_ts_resume(struct platform_device *pdev)
{
	struct quartz_ts_data *ts = dev_get_drvdata(&pdev->dev);

	if (ts && ts->pdata && ts->pdata->power_enable && ts->pm) {
		ts->pdata->power_enable(ts->pdevice);
	}

	return 0;
}
#endif

static ssize_t pm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quartz_ts_data *ts = quartz_ts;

	ts->pm = !ts->pm;

	return sprintf(buf, "pm(%d)\n", ts->pm);
}

static ssize_t reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quartz_ts_data *ts = quartz_ts;

	/* Hold a 30-sec wakelock to avoid the immediate Linux PM suspend */
	wake_lock_timeout(&ts->wakelock, 30 * HZ);

	dev_info(&ts->pdevice->dev, "reset\n");

	if (ts && ts->pdata && ts->pdata->power_disable)
		ts->pdata->power_disable(ts->pdevice);

	if (ts && ts->pdata && ts->pdata->power_enable) {
		msleep(350);
		ts->pdata->power_enable(ts->pdevice);
	}

	return sprintf(buf, "quartz usb touch reset\n");
}

static ssize_t hub_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct quartz_ts_data *ts = quartz_ts;
	
	dev_info(&ts->pdevice->dev, "hub gpio50 reset\n");
	gpio_set_value(50, 0);
	mdelay(2);
	gpio_set_value(50, 1);
	mdelay(2);
	gpio_set_value(50, 0);

	return sprintf(buf, "quartz usb hub reset\n");
}

static DEVICE_ATTR(pm, 0444, pm_show, NULL);
static DEVICE_ATTR(reset, 0444, reset_show, NULL);
static DEVICE_ATTR(hub_reset, 0444, hub_reset_show, NULL);

static int quartz_ts_sysfs_init(struct quartz_ts_data *ts)
{
	int ret ;

	ts->debug_kobj = kobject_create_and_add("tpdebug", NULL) ;
	if (ts->debug_kobj == NULL) {
		dev_info(&ts->pdevice->dev, "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(ts->debug_kobj, &dev_attr_pm.attr);
	if (ret) {
		dev_info(&ts->pdevice->dev, "%s: sysfs_create_pm_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(ts->debug_kobj, &dev_attr_reset.attr);
	if (ret) {
		dev_info(&ts->pdevice->dev, "%s: sysfs_create_reset_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(ts->debug_kobj, &dev_attr_hub_reset.attr);
	if (ret) {
		dev_info(&ts->pdevice->dev, "%s: sysfs_create_hub_reset_file failed\n", __func__);
		return ret;
	}
	dev_info(&ts->pdevice->dev, "Quartz debug sysfs create success\n");
	return 0 ;
}

#if 0
static void quartz_ts_sysfs_deinit(struct quartz_ts_data *ts)
{
	sysfs_remove_file(ts->debug_kobj, &dev_attr_pm.attr);
	sysfs_remove_file(ts->debug_kobj, &dev_attr_reset.attr);
	kobject_del(ts->debug_kobj);
}
#endif

static int quartz_ts_probe(struct platform_device *pdev)
{
	struct quartz_ts_pdata *pdata = pdev->dev.platform_data;
	struct quartz_ts_data *ts = NULL;

	dev_info(&pdev->dev, "%s()\n", __func__);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		return -ENOMEM;
	}
	ts->pdata = pdata;
	ts->pdevice = pdev;
	dev_set_drvdata(&pdev->dev, ts);
	quartz_ts = ts;

	if (ts && ts->pdata && ts->pdata->power_init) {
		ts->pdata->power_init(ts->pdevice);
	}
	
	if (ts && ts->pdata && ts->pdata->power_enable) {
		ts->pdata->power_enable(ts->pdevice);
	}

	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "quartz_ts");

#if !defined(DONT_USE_EARLYSUSPEND_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = quartz_ts_early_suspend;
	ts->early_suspend.resume = quartz_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	ts->pm = QUARTZ_TOUCH_PM_INIT_VALUE;

	quartz_ts_sysfs_init(ts);

	return 0;
}

static void quartz_ts_shutdown(struct platform_device *pdev)
{
	struct quartz_ts_data *ts = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "%s()\n", __func__);

	if (ts && ts->pdata && ts->pdata->power_disable) {
		ts->pdata->power_disable(ts->pdevice);
	}
}

static struct platform_driver quartz_ts_driver = {
	.probe = quartz_ts_probe,
	.shutdown = quartz_ts_shutdown,
#if !defined(DONT_USE_EARLYSUSPEND_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
#else
	.suspend = quartz_ts_suspend,
	.resume = quartz_ts_resume,
#endif
	.driver = {
		.name = QUARTZ_TOUCH_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init quartz_ts_init(void)
{
	return platform_driver_register(&quartz_ts_driver);
}

static void __exit quartz_ts_deinit(void)
{
	platform_driver_unregister(&quartz_ts_driver);
}

#if 0
module_init(quartz_ts_init);
#else
fs_initcall(quartz_ts_init);
#endif
module_exit(quartz_ts_deinit);

MODULE_AUTHOR("Innocomm");
MODULE_DESCRIPTION("Innocomm Quartz USB Touch Power Control");
MODULE_LICENSE("GPL");
