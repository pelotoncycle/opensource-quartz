/*
 * arch/arm/mach-omap2/xmd-boot-c2c.c
 *
 * xmd-boot-c2c.c -- Configuration of C2C IOs.
 *
 * Copyright (C) 2011 Texas Instruments
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
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#include <plat/omap_hwmod.h>
#include "mux.h"
#include <mach/ctrl_module_pad_core_44xx.h>
#include <mach/ctrl_module_pad_wkup_44xx.h>
#include <mach/msm_smd.h>
#include "smd_private.h"
#include <mach/xmd.h>

extern struct xmd_data my_xmd;
#define c (&my_xmd.c2c_data)


/*
 * XMD module init
 */

int __init xmd_c2c_init()
{
	int ret;

	ret = xmd_c2c_dd_init();

	return ret;
}


/*
 * XMD module exit
 */

void __exit xmd_c2c_exit()
{
	xmd_c2c_dd_exit();
	return;
}
