/*
 * arch/arm/mach-omap2/xmd-boot-common.c
 *
 * xmd-boot-common.c -- Boot control for IFX XMM modem.
 *
 * Copyright (C) 2011 Intel Mobile Communications GmbH
 *
 * Author: Khened Chaitanya <Chaitanya.Khened@intel.com>
 *
 * Updates for XMM rfkill driver integration
 * Copyright (C) 2012 Texas Instruments
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
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/regulator/machine.h>

#include <plat/omap_hwmod.h>
#include "mux.h"
#include <mach/gpio.h>
#ifdef CONFIG_MACH_OMAP_XMM_SPI
#include <mach/msm_smd.h>
#include "smd_private.h"
#endif
#include <mach/xmd.h>
#include "xmd-ch.h"

/*
* Local definitions
*/

static DEFINE_MUTEX(boot_state_mutex);
static DEFINE_MUTEX(flashless_config_mutex);

static DEFINE_MUTEX(rild_pid_mutex);

static int flashless = 0;

static long int rild_pid = -1;

struct xmd_data my_xmd;

#define b (&my_xmd.boot_data)

#define XMD_ALIVE_WAKE_LOCK_NAME "xmd_alive"
struct wake_lock xmd_alive_wake_lock;

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
enum {XMD_BOOT_OFF, XMD_BOOT_FIRMWARE, XMD_BOOT_ON, XMD_BOOT_RECOVERY};
static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON", "RECOVERY"};
static int xmd_alive_state = XMD_BOOT_OFF;
#endif

/*
* OMAP mux pad configuration function
*/
int xmd_omap_mux_init_signal(struct xmm_pad_mux_config muxcfg)
{
	return omap_mux_init_signal(muxcfg.muxname, muxcfg.muxval);
};


/*------------------------- POWER-ON RESET SUPPORT -------------------------*/

int xmd_boot_power_off(void)
{
	int status = 0;
	struct xmd_boot_platform_data *pd;

	pd = b->boot_platform_data;
	if (!pd)
	{
		printk(KERN_INFO "\nCRASH: NULL access in xmd_boot_power_off\n");
		return -EINVAL;
	}

	/* XMM power off - after software switch off, make sure reset asserted */
	if (!xmd_board_pci_adapter()) {
		if (pd->on_off_gpio != -1)
			gpio_set_value(pd->on_off_gpio, pd->on_off_active);
		gpio_set_value(pd->reset_pmu_req_gpio, !(pd->reset_pmu_req_active));
	} else {
		if (pd->on_off_gpio != -1)
			gpio_set_value(pd->on_off_gpio, !(pd->on_off_active));
		gpio_set_value(pd->reset_pmu_req_gpio, pd->reset_pmu_req_active);
		if (pd->pwr_supply_gpio != -1)
			gpio_set_value(pd->pwr_supply_gpio, !(pd->pwr_supply_active));
	}

	return (status);
}


int xmd_board_power_off(void)
{
	int status = 0;

	pr_info("XMD: xmd_board_power_off\n");

#ifdef CONFIG_MACH_OMAP_XMM_SPI
	xmd_spi_power_off();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
	xmd_hsi_power_off();
#endif

	xmd_boot_power_off();

	return (status);
}


int xmd_boot_power_on_reset(void)
{
	int status = 0;
	struct xmd_boot_platform_data *pd;

	pd = b->boot_platform_data;
	if (!pd)
	{
		printk(KERN_INFO "\nCRASH: NULL access in xmd_boot_power_on_reset\n");
		return -EINVAL;
	}

	/* Pin muxing and default states: ON_OFF / RESET_PMU_REQ and alike */
	if (!xmd_board_pci_adapter()) {
		if (pd->on_off_gpio != -1)
			gpio_set_value(pd->on_off_gpio, !(pd->on_off_active));

		/* XMM Power On Reset */
		msleep(50);
		gpio_set_value(pd->reset_pmu_req_gpio, !(pd->reset_pmu_req_active));
		msleep(100);
		if (pd->on_off_gpio != -1)
			gpio_set_value(pd->on_off_gpio, pd->on_off_active);
	} else {
		if (pd->pwr_supply_gpio != -1) {
			gpio_set_value(pd->pwr_supply_gpio, pd->pwr_supply_active);
			msleep(1);
		}
		/*
		* The power on sequence for 6260 requires that "On" signal goes high
		* after releasing the reset signal.
		*/
		gpio_set_value(pd->reset_pmu_req_gpio, !(pd->reset_pmu_req_active));
		if (pd->baseband_reset_gpio != -1)
			gpio_set_value(pd->baseband_reset_gpio, !(pd->baseband_reset_active));
		msleep(500); /* additional 1000 us delay */
		if (pd->on_off_gpio != -1)
			gpio_set_value(pd->on_off_gpio, pd->on_off_active);
	}

	return (status);
}


int xmd_board_power_on_reset(void)
{
	int status = 0;

	pr_info("XMD: xmd_board_power_on_reset\n");

	xmd_boot_power_on_reset();
#ifdef CONFIG_MACH_OMAP_XMM_SPI
	xmd_spi_power_on_reset();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
	xmd_hsi_power_on_reset();
	//msleep(1000); // allow some time for modem to boot
#endif

	return (status);
}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
void xmd_board_wait_for_cp_ready(void)
{
	while (1) {
		if ((xmd_alive_state == XMD_BOOT_RECOVERY) || (xmd_alive_state == XMD_BOOT_ON))
			break;
		msleep(100);
	}
}

void xmd_board_notify_recovery_complete(void)
{
	pr_info("%s()\n", __func__);
	xmd_alive_state = XMD_BOOT_ON;
}

#endif

static irqreturn_t xmd_boot_alive_isr(int irq, void *dev_id)
{
	struct pid *pid = NULL;
	/* Detect both falling and rising of alive signal */
	struct xmd_boot_platform_data *pd;
	int value;

	pd = b->boot_platform_data;
	if (!pd) {
		printk(KERN_INFO "\nCRASH: NULL access in xmd_boot_alive_isr\n");
		goto exit;
	}

	value = gpio_get_value(pd->alive_gpio);

	if (value == 0) {
		pr_info("===================XMD: MODEM is DOWN!!!=================\n");
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		xmd_alive_state = XMD_BOOT_OFF;
#endif
		goto exit;
	}

	pr_info("===================XMD: MODEM is UP, Activate 5 Secs timeout wakelock!!!=================\n");
	/* Activate 5 Secs timeout wakelock to make sure RILD handle the reset event */
	wake_lock_timeout(&xmd_alive_wake_lock, 5 * HZ);

	if(rild_pid > 0) {
		/* send signal to user space process (RILD) */
		pid = find_get_pid(rild_pid);
	}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	xmd_alive_state = XMD_BOOT_RECOVERY;
	xmd_dlp_recovery();
#endif

	if (pid) {
		pr_info("XMD: Send SIGTERM to [rild_pid=%ld] (pid=0x%p).\n", rild_pid, pid);
		kill_pid(pid, SIGTERM, 1);
		put_pid(pid);
	} else {
		pr_info("XMD: [rild_pid=%ld, pid=%p]... Do nothing!\n", rild_pid, pid);
	}
	rild_pid = -1;

exit:

	return IRQ_HANDLED;
}

int xmd_boot_board_init(struct xmd_boot_platform_data *boot_platform_data)
{
	int retval = 0;
	struct xmd_boot_platform_data *pd = boot_platform_data;

	b->boot_platform_data = pd;

	if (pd->pwr_supply_gpio != -1)
		xmd_omap_mux_init_signal(pd->pwr_supply_pinmux);

	if (pd->reset_pmu_req_gpio != -1)
		xmd_omap_mux_init_signal(pd->reset_pmu_req_pinmux);

	if (pd->baseband_reset_gpio != -1)
		xmd_omap_mux_init_signal(pd->baseband_reset_pinmux);

	if (pd->on_off_gpio != -1)
		xmd_omap_mux_init_signal(pd->on_off_pinmux);

	if (pd->alive_gpio != -1)
		xmd_omap_mux_init_signal(pd->alive_pinmux);

	if (pd->usb_en_gpio != -1)
		xmd_omap_mux_init_signal(pd->usb_en_pinmux);

	retval = gpio_request(pd->reset_pmu_req_gpio, "XMD_RESET_PMU_REQ_GPIO");
	if(retval < 0) {
		pr_err("Can't get RESET_PMU_REQ GPIO\n");
		goto out;
	}
	gpio_direction_output(pd->reset_pmu_req_gpio, pd->reset_pmu_req_active);

	if (pd->on_off_gpio != -1) {
		retval = (gpio_request(pd->on_off_gpio, "XMD_ON_OFF_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_ON_OFF_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->on_off_gpio, !(pd->on_off_active));
	}

	/* BASEBAND_RESET - optional signal */
	if (pd->baseband_reset_gpio != -1) {
		retval = (gpio_request(pd->baseband_reset_gpio,
						"XMD_BB_RESET_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_BB_RESET_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->baseband_reset_gpio, pd->baseband_reset_active);
	}

	/* PWR_SUPPLY - optional signal */
	if (pd->pwr_supply_gpio != -1) {
		retval = (gpio_request(pd->pwr_supply_gpio,
					"XMD_PWR_SUPPLY_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_PWR_SUPPLY_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->pwr_supply_gpio, !(pd->pwr_supply_active));
	}

	/* ALIVE - optional signal */
	if (pd->alive_gpio != -1) {
		wake_lock_init(&xmd_alive_wake_lock, WAKE_LOCK_SUSPEND, XMD_ALIVE_WAKE_LOCK_NAME);

		retval = (gpio_request(pd->alive_gpio,
					"XMD_ALIVE_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_ALIVE_GPIO\n");
			goto out;
		}
		gpio_direction_input(pd->alive_gpio);

		/* Detect both falling and rising of alive signal */
		/* Register interrupt handler for AP_WAKE_UP (used to detect modem reset itself) */
		retval = request_irq(OMAP_GPIO_IRQ(pd->alive_gpio), &xmd_boot_alive_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "XMD_ALIVE_IRQ", NULL);

		if(retval < 0)
		{
			pr_err("XMD: Request_irq(XMD_ALIVE_IRQ) failed=%d!\n", retval);
			goto out;
		}

		/* Enable MODEM_ALIVE interrupt to wake up AP */
		retval = enable_irq_wake(OMAP_GPIO_IRQ(pd->alive_gpio));
		pr_err("XMD: enable_irq_wake(XMD_ALIVE_IRQ) returns=%d!\n", retval);
	}

	if (pd->usb_en_gpio != -1) {
		retval = (gpio_request(pd->usb_en_gpio,
					"XMD_USB_EN_GPIO") < 0);
		if(retval < 0) {
			pr_err("Can't get XMD_USB_EN_GPIO\n");
			goto out;
		}
		gpio_direction_output(pd->usb_en_gpio, !(pd->usb_en_active));
	}

out:
	return retval;
}


int xmd_board_init(struct xmd_platform_data *platform_data)
{
	int retval = 0;
	struct xmd_platform_data *pd = platform_data;

#ifdef CONFIG_MACH_OMAP_XMM_SPI
	xmd_spi_board_init (&(pd->spi_platform_data));
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
	xmd_hsi_board_init (&(pd->hsi_platform_data));
#endif

	xmd_boot_board_init (&(pd->boot_platform_data));

	xmd_board_power_off();

	return retval;
}
EXPORT_SYMBOL_GPL(xmd_board_init);


/*---------------------------- SYSFS BOOT STATES ----------------------------*/

#ifndef XMD_DLP_RECOVERY_ENHANCEMENT
enum {XMD_BOOT_OFF, XMD_BOOT_FIRMWARE, XMD_BOOT_ON};
static const char * xmd_boot_state_str[] = {"OFF", "FIRMWARE", "ON"};
#endif

static ssize_t xmd_boot_modem_state_show
	(struct device *, struct device_attribute *, char *);
static ssize_t xmd_boot_modem_state_store(struct device *,
	struct device_attribute *, const char *, size_t);
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR, xmd_boot_modem_state_show,
	xmd_boot_modem_state_store);

int xmd_boot_state_change(int boot_state);

static ssize_t xmd_boot_modem_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int status;

	struct xmd_boot_platform_data *pd;

	mutex_lock(&boot_state_mutex);
	pd = b->boot_platform_data;

#ifndef XMD_DLP_RECOVERY_ENHANCEMENT
	if (!gpio_get_value(pd->alive_gpio))
	{
		pr_info("XMD: %s()- BOOT STATE to XMD_BOOT_OFF\n", __func__);
		xmd_boot_state_change(XMD_BOOT_OFF);
	}
#endif

	if (attr == &dev_attr_state) {
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		if (xmd_is_cp_in_recovery_state()) {
			status = snprintf(buf, PAGE_SIZE, "%s\n",
				xmd_boot_state_str[XMD_BOOT_RECOVERY]);
		} else {
			status = snprintf(buf, PAGE_SIZE, "%s\n",
				xmd_boot_state_str[xmd_alive_state]);
		}
#else
		status = snprintf(buf, PAGE_SIZE, "%s\n",
			xmd_boot_state_str[b->boot_state]);
#endif
	} else
		status = -EINVAL;

	mutex_unlock(&boot_state_mutex);
	return status;
}


int xmd_boot_state_change(int boot_state)
{
	int status;

	pr_info("XMD: BOOT STATE (%s) -> (%s)\n",
		xmd_boot_state_str[b->boot_state],
		xmd_boot_state_str[boot_state]);

	switch (boot_state) {
	case XMD_BOOT_OFF:
		switch(b->boot_state) {
		case XMD_BOOT_ON:
#ifdef CONFIG_MACH_OMAP_XMM_SPI
			xmd_mux_disable(); /* Ignore returned value */
#endif
			/* Electrical power-off */
			xmd_board_power_off();
			break;
		case XMD_BOOT_FIRMWARE:
			xmd_boot_disable_fw_tty();
			/* Electrical power-off */
			xmd_board_power_off();
			break;
		default:
			goto error1;
		}
		break;

	case XMD_BOOT_FIRMWARE:
		if (!flashless)
			goto error1;
		switch(b->boot_state) {
		case XMD_BOOT_OFF:
			xmd_boot_enable_fw_tty();
			/* Electrical power-on/reset sequence,
			   immediate return for ROM code interaction */
			xmd_board_power_on_reset();
			break;
		default:
			goto error1;
		}
		break;

	case XMD_BOOT_ON:
		switch(b->boot_state) {
		case XMD_BOOT_OFF:
			if (flashless)
				goto error1;
			/* Electrical power-on/reset sequence,
			   and modem firmware boot time      */
			xmd_board_power_on_reset();
#ifdef CONFIG_MACH_OMAP_XMM_HSI
			msleep(1000); // allow some time for modem to boot
#endif

			/* No break */
		case XMD_BOOT_FIRMWARE:
			if (flashless)
				xmd_boot_disable_fw_tty();
#ifdef CONFIG_MACH_OMAP_XMM_SPI
			status = xmd_mux_enable(); /* wait for MODEM to send msg on startup to make sure MODEM is on */
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
			status = wait_for_xmd_ack(); /* wait for MODEM to send msg on startup to make sure MODEM is on */
#endif
			if (status < 0)
				goto error2;
			break;
		default:
			goto error1;
		}
		break;
	}

	/* Transition success */
	b->boot_state = boot_state;
	status = 0;
	goto end;

error1:
	pr_err("XMD: BOOT wrong state transition\n");
	status = -1;
	goto end;
error2:
	pr_err("XMD: BOOT transition failed\n");
	/* Unknown modem state, force power-off */
	xmd_board_power_off();
	b->boot_state = XMD_BOOT_OFF;
	status =  -2;
	goto end;

end:
	return status;
}


static ssize_t xmd_boot_modem_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i, status;

	mutex_lock(&boot_state_mutex);

	if (count > 0) {
		for (i = 0; i < ARRAY_SIZE(xmd_boot_state_str); i++) {
			if (strstr(buf, xmd_boot_state_str[i]))
				break;
		}

		if (i < ARRAY_SIZE(xmd_boot_state_str)) {
			xmd_boot_state_change(i);
			status = count;
		} else
			status = -EINVAL;
	} else
		status = -EINVAL;

	mutex_unlock(&boot_state_mutex);
	return status;
}


/*------------------------------ SYSFS BOOT IO ------------------------------*/

static ssize_t xmd_boot_io_show
	(struct device *, struct device_attribute *, char *);
static ssize_t xmd_boot_io_store(struct device *,
	struct device_attribute *, const char *, size_t);
static DEVICE_ATTR(io, S_IRUGO | S_IWUSR, xmd_boot_io_show, xmd_boot_io_store);


struct xmd_boot_io_attr {
	char *name;
	int  pd_gpio_offset;
	int  pd_pinmux_offset;
};

static const struct xmd_boot_io_attr platform_data_io_attr[] =
{
	{"RESET_PMU_REQ",
	offsetof(struct xmd_boot_platform_data, reset_pmu_req_gpio),
	offsetof(struct xmd_boot_platform_data, reset_pmu_req_pinmux)},
	{"BASEBAND_RESET",
	offsetof(struct xmd_boot_platform_data, baseband_reset_gpio),
	offsetof(struct xmd_boot_platform_data, baseband_reset_pinmux)},
	{"ON_OFF",
	offsetof(struct xmd_boot_platform_data, on_off_gpio),
	offsetof(struct xmd_boot_platform_data, on_off_pinmux)},
	{"PWR_SUPPLY",
	offsetof(struct xmd_boot_platform_data, pwr_supply_gpio),
	offsetof(struct xmd_boot_platform_data, pwr_supply_pinmux)},
	{"ALIVE",
	offsetof(struct xmd_boot_platform_data, alive_gpio),
	offsetof(struct xmd_boot_platform_data, alive_pinmux)},
	{"USB_EN",
	offsetof(struct xmd_boot_platform_data, usb_en_gpio),
	offsetof(struct xmd_boot_platform_data, usb_en_pinmux)},
};


enum {XMD_BOOT_IO_ENABLE, XMD_BOOT_IO_SAFE, XMD_BOOT_IO_SET_1, XMD_BOOT_IO_SET_0};

struct xmd_boot_action_attr {
	char *name;
	int  id;
};

static const struct xmd_boot_action_attr action_attr[] =
{
	{"ENABLE", XMD_BOOT_IO_ENABLE},
	{"SAFE",   XMD_BOOT_IO_SAFE},
	{"1",      XMD_BOOT_IO_SET_1},
	{"0",      XMD_BOOT_IO_SET_0}
};

static ssize_t xmd_boot_io_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int i;
	char *_buf = buf;

	_buf += snprintf(_buf, PAGE_SIZE, "XMD IO SYNTAX: <IO> <ACTION>\n");
	_buf += snprintf(_buf, PAGE_SIZE, "\nIO:\n");

	for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
		_buf += snprintf(_buf, PAGE_SIZE, "%s\n",
					platform_data_io_attr[i].name);
	}
	_buf += snprintf(_buf, PAGE_SIZE, "\nACTION:\n");
	for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
		_buf += snprintf(_buf, PAGE_SIZE, "%s\n",
					action_attr[i].name);
	}
	_buf += snprintf(_buf, PAGE_SIZE, "\n");

	return (_buf - buf);
}


static ssize_t xmd_boot_io_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i, j;

	if (count > 0) {
		for (i = 0; i < ARRAY_SIZE(platform_data_io_attr); i++) {
			if (strstr(buf, platform_data_io_attr[i].name))
				break;
		}
		if (i < ARRAY_SIZE(platform_data_io_attr)) {
			buf += strlen(platform_data_io_attr[i].name);
			for (j = 0; j < ARRAY_SIZE(action_attr); j++) {
				if (strstr(buf, action_attr[j].name))
					break;
			}
			if (j < ARRAY_SIZE(action_attr)) {
				int gpio;
				struct xmm_pad_mux_config pinmux;
				struct xmd_boot_platform_data *bpd;

				pr_info("XMD IO: %s %s\n",
						platform_data_io_attr[i].name,
						action_attr[j].name);
				bpd = b->boot_platform_data;
				gpio = *((int*) ((char*) bpd +
					platform_data_io_attr[i].
							pd_gpio_offset));
				pinmux = *((struct xmm_pad_mux_config*)
						((char*) bpd +
						platform_data_io_attr[i].
							pd_pinmux_offset));


				switch(action_attr[j].id) {
				case XMD_BOOT_IO_ENABLE:
					break;
				case XMD_BOOT_IO_SAFE:
					break;
				case XMD_BOOT_IO_SET_1:
					if (gpio != -1)
						gpio_set_value(gpio, 1);
					else
						pr_err("XMD: undef GPIO\n");
					break;
				case XMD_BOOT_IO_SET_0:
					if (gpio != -1)
						gpio_set_value(gpio, 0);
					else
						pr_err("XMD: undef GPIO\n");
					break;
				}

			} else {
				pr_err("XMD: Uknown Action\n");
				return -EINVAL;
			}
		} else {
			pr_err("XMD: Uknown IO\n");
			return -EINVAL;
		}

		return count;
	}
	return -EINVAL;
}


module_param(flashless, int, 0444);
MODULE_PARM_DESC(flashless,
	"Set to true if XMM modem is flashless (firmware required)");


static ssize_t xmd_boot_flashless_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	char *_buf = buf;

	mutex_lock(&flashless_config_mutex);

	_buf += snprintf(_buf, PAGE_SIZE, "%d\n", flashless);

	mutex_unlock(&flashless_config_mutex);

	return (_buf - buf);
}

static ssize_t xmd_boot_flashless_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{

	mutex_lock(&flashless_config_mutex);

	if (*buf == '0')
	{
		pr_info("XMD: Set to flashed mode\n");
		flashless = 0;
	}
	else
	{
		pr_info("XMD: Set to flashless mode\n");
		flashless = 1;
	}

	mutex_unlock(&flashless_config_mutex);

	return count;
}

#if 0
static DEVICE_ATTR(flashless, S_IRUGO, xmd_boot_flashless_show, NULL);
#else
static DEVICE_ATTR(flashless, S_IRUGO | S_IWUSR, xmd_boot_flashless_show, xmd_boot_flashless_store);
#endif


/*-------------------------------SYSFS RILD_PID------------------------------*/
static ssize_t xmd_boot_rild_pid_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	mutex_lock(&rild_pid_mutex);

	rild_pid = simple_strtol(buf, NULL, 10);

	switch (rild_pid)
	{
	case LONG_MIN:
	case LONG_MAX:
		pr_err("XMD: Set RIL PID failed=%ld\n", rild_pid);
		rild_pid = -1;
		break;
	case -1:
		pr_err("XMD: Clear RIL PID\n");
		break;
	default:
		pr_info("XMD: Set RIL PID=%ld\n", rild_pid);
		break;
	}

	mutex_unlock(&rild_pid_mutex);

	return count;
}

static DEVICE_ATTR(rild_pid, S_IRUGO | S_IWUSR, NULL, xmd_boot_rild_pid_store);

enum {XMD_MODEM_VBUS_OFF, XMD_MODEM_VBUS_ON};
static const char * xmd_boot_modem_vbus_str[] = {"OFF", "ON"};
static struct regulator *reg_vbus = NULL;

static ssize_t xmd_boot_modem_vbus_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i, status = -EINVAL;

	if (count > 0) {
		struct xmd_boot_platform_data *pd;
		pd = b->boot_platform_data;

		for (i = 0; i < ARRAY_SIZE(xmd_boot_modem_vbus_str); i++) {
			if (strstr(buf, xmd_boot_modem_vbus_str[i]))
				break;
		}

		if (i > ARRAY_SIZE(xmd_boot_modem_vbus_str))
			goto exit;

		if (!pd)
			goto exit;

		if (!(pd->modem_vbus_regulator))
			goto exit;

		/* Try to get VMBUS regulator */
		if (!reg_vbus) {
			reg_vbus = regulator_get(NULL, pd->modem_vbus_regulator);
			if (!reg_vbus) {
				pr_info("XMD: %s()- Cannot find regulator '%s'\n", __func__, pd->modem_vbus_regulator);
				goto exit;
			}
		}

		if (i == XMD_MODEM_VBUS_OFF) {
			if (regulator_is_enabled(reg_vbus))
				status = regulator_disable(reg_vbus);
		} else {
			if (!regulator_is_enabled(reg_vbus))
				status = regulator_enable(reg_vbus);
		}
	}

exit:
	return status;
}

static DEVICE_ATTR(modem_vbus, S_IRUGO | S_IWUSR, NULL, xmd_boot_modem_vbus_store);

/*------------------------------ BOOT DRIVER ------------------------------*/

static int pci_adapter = 1;
module_param(pci_adapter, int, 0644);
MODULE_PARM_DESC(pci_adapter,
		"Set to true if PCI-E/COM6 Zoom adapter is being used");

int xmd_board_pci_adapter(void) {
	return pci_adapter;
}


static int xmd_boot_probe(struct platform_device *);
static int xmd_boot_remove(struct platform_device *);
static void xmd_boot_shutdown(struct platform_device *);

static struct platform_driver xmd_boot_driver = {
	.probe = xmd_boot_probe,
	.remove = xmd_boot_remove,
	.shutdown = xmd_boot_shutdown,
	.driver = {
		.name ="xmm_boot",
		.owner = THIS_MODULE,
	},
};


static int xmd_boot_probe(struct platform_device *pdev)
{
	int status;

	printk("\nXMD: boot probe function\n");

	/* Modem has been forced to reset in board-xxx.c */
	b->boot_state = XMD_BOOT_OFF;

	/* sysfs entries for IO control */
	status = device_create_file(&(pdev->dev), &dev_attr_io);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry io\n");
		return status;
	}

	/* sysfs entries for boot control */
	status = device_create_file(&pdev->dev, &dev_attr_state);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry state\n");
		return status;
	}
	status = device_create_file(&pdev->dev, &dev_attr_flashless);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry flashless\n");
		return status;
	}

	status = device_create_file(&(pdev->dev), &dev_attr_rild_pid);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry rild_pid\n");
		return status;
	}

	if (flashless)
		pr_info("XMD: ### Flash-less modem\n");
	else
		pr_info("XMD: ### Flash modem\n");

	status = device_create_file(&(pdev->dev), &dev_attr_modem_vbus);
	if (status) {
		pr_err("XMD: BOOT error creating sysfs entry modem_vbus\n");
		return status;
	}

	return 0;
}


static int xmd_boot_remove(struct platform_device *pdev)
{
	struct xmd_boot_platform_data *pd;

	pd = b->boot_platform_data;
	if (!pd) {
		printk(KERN_INFO "\nCRASH: NULL access in xmd_boot_remove\n");
	} else {
		if (pd->alive_gpio != -1) {
			free_irq(OMAP_GPIO_IRQ(pd->alive_gpio), NULL);
			wake_lock_destroy(&xmd_alive_wake_lock);
		}
	}

	/* Disgraceful modem switch off */
	xmd_board_power_off();

	/* sysfs entries for IO control */
	device_remove_file(&(pdev->dev), &dev_attr_io);

	/* sysfs entries for boot control */
	device_remove_file(&pdev->dev, &dev_attr_state);
	device_remove_file(&pdev->dev, &dev_attr_flashless);

	device_remove_file(&pdev->dev, &dev_attr_rild_pid);

	return 0;
}

static void xmd_boot_shutdown(struct platform_device *pdev)
{
	xmd_boot_remove(pdev);
}

static int __init xmd_boot_init(void)
{
	int status = 0;

	status = platform_driver_register(&xmd_boot_driver);
	if (status < 0){
		pr_err("Failed to register XMD BOOT driver");
		return status;
	}
	return 0;
}

static void __exit xmd_boot_exit(void)
{
	platform_driver_unregister(&xmd_boot_driver);
}

static int __init xmd_module_init(void)
{
	int status = 0;

	/* Upper layers initialization */
	xmd_boot_init();
#ifdef CONFIG_MACH_OMAP_XMM_SPI
	xmd_spi_init();
	xmd_mux_init();
	xmd_smd_init();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
	xmd_hsi_init();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_C2C
	xmd_c2c_init();
#endif

	return status;
}

static void __exit xmd_module_exit(void)
{
	/* Upper layers deinitialization */
#ifdef CONFIG_MACH_OMAP_XMM_SPI
	xmd_smd_exit();
	xmd_mux_exit();
	xmd_spi_exit();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_HSI
	xmd_hsi_exit();
#endif
#ifdef CONFIG_MACH_OMAP_XMM_C2C
	xmd_c2c_exit();
#endif

	xmd_boot_exit();
}

#ifdef MODULE
module_init(xmd_module_init);
#else
/* HSI devices to be initialized first */
late_initcall(xmd_module_init);
#endif
module_exit(xmd_module_exit);

MODULE_LICENSE("GPL");
