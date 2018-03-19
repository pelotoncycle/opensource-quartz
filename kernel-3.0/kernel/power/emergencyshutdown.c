/*
 * linux/kernel/power/emergencyshutdown.c
 *
 * Emergency shutdown machine
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * Author: James Wu <james.wu@innocomm.com>
 *
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/freezer.h>
#include <linux/cpu.h>
#include <linux/kmod.h>
#include <linux/kmsg_dump.h>
#include <linux/wakelock.h>

#include <linux/emergencyshutdown.h>

#if 0
#define NO_DEVICE_POWEROFF_FOR_HW_TEST
#endif

extern void emergency_shutdown_remount(void);
extern void emergency_shutdown_sync(void);

static int emergency_irq = -1;
static void (*emergency_platform_shutdown)(void) = NULL;
static struct wake_lock emergency_shutdown_wakelock;

static void do_emergency_shutdown(struct work_struct *work)
{
	lockdep_off();

	if (emergency_platform_shutdown)
		emergency_platform_shutdown();

#ifdef CONFIG_EMERGENCY_SHUTDOWN_SYNC_FS
	emergency_shutdown_sync();
#endif /* CONFIG_EMERGENCY_SHUTDOWN_SYNC_FS */

#ifdef CONFIG_EMERGENCY_SHUTDOWN_REMOUNT_RO
	emergency_shutdown_remount();
#endif /* CONFIG_EMERGENCY_SHUTDOWN_REMOUNT_RO */

	system_state = SYS_POWER_OFF;
	usermodehelper_disable();
	disable_nonboot_cpus();
#ifdef NO_DEVICE_POWEROFF_FOR_HW_TEST
	printk(KERN_EMERG "Emergency DUMMY down.\n");
	kmsg_dump(KMSG_DUMP_POWEROFF);
#else
	printk(KERN_EMERG "Emergency down.\n");
	kmsg_dump(KMSG_DUMP_POWEROFF);
	machine_power_off();
	while (1);
#endif /* NO_DEVICE_POWEROFF_FOR_HW_TEST */
}

static DECLARE_WORK(emergency_shutdown_work, do_emergency_shutdown);

static irqreturn_t emergency_shutdown_irq(int irq, void *dev)
{
	wake_lock(&emergency_shutdown_wakelock);

	printk(KERN_EMERG EMERGENCY_SHUTDOWN_DRV_NAME ": shutting down\n");

	/* run emergency shutdown on boot cpu */
	schedule_work_on(cpumask_first(cpu_online_mask), &emergency_shutdown_work);

	return IRQ_HANDLED;
}

static int emergency_shutdown_probe(struct platform_device *pdev)
{
	struct emergency_shutdown_platform_data *pdata = pdev->dev.platform_data;
	unsigned long flags = IRQF_DISABLED;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "no pdata\n");
		return -EINVAL;
	}

	if (!pdata->platform_init) {
		dev_err(&pdev->dev, "no platform_init()\n");
		return -EINVAL;
	}

	ret = pdata->platform_init(&emergency_irq, &flags);
	if (ret || emergency_irq < 0) {
		dev_err(&pdev->dev, "init err %d, irq %d\n", ret, emergency_irq);
		return -EINVAL;
	}

	emergency_platform_shutdown = pdata->platform_shutdown;

	wake_lock_init(&emergency_shutdown_wakelock,
			WAKE_LOCK_SUSPEND, EMERGENCY_SHUTDOWN_DRV_NAME);

	ret = request_irq(emergency_irq, emergency_shutdown_irq, flags,
			EMERGENCY_SHUTDOWN_DRV_NAME, NULL);
	if (ret) {
		dev_err(&pdev->dev, "irq %d err %d\n", emergency_irq, ret);
		goto err0;
	}

	enable_irq_wake(emergency_irq);

	dev_info(&pdev->dev, "ready\n");

	return 0;

err0:
	wake_lock_destroy(&emergency_shutdown_wakelock);
	return ret;
}

static int emergency_shutdown_dev_suspend(struct device *dev)
{
	disable_irq_nosync(emergency_irq);
	disable_irq_wake(emergency_irq);
	return 0;
}

static int emergency_shutdown_dev_resume(struct device *dev)
{
	enable_irq(emergency_irq);
	enable_irq_wake(emergency_irq);
	return 0;
}

static const struct dev_pm_ops emergency_shutdown_dev_pm_ops = {
	.suspend	= emergency_shutdown_dev_suspend,
	.resume		= emergency_shutdown_dev_resume,
};

static struct platform_driver emergency_shutdown_driver = {
	.probe		= emergency_shutdown_probe,
	.driver		= {
		.name	= EMERGENCY_SHUTDOWN_DRV_NAME,
		.pm		= &emergency_shutdown_dev_pm_ops,
	},
};

static int emergency_shutdown_init(void)
{
	return platform_driver_register(&emergency_shutdown_driver);
}

late_initcall(emergency_shutdown_init);

MODULE_AUTHOR("James Wu <james.wu@innocomm.com>");
MODULE_DESCRIPTION("Emergency shutdown machine");
MODULE_LICENSE("GPL");
