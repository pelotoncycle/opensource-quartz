/*
 * arch/arm/mach-omap2/xmd-boot-hsi.c
 *
 * xmd-boot-hsi.c -- Configuration of HSI GPIOs.
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
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#include <plat/omap_hwmod.h>
#include "mux.h"

#ifdef CONFIG_MACH_OMAP_XMM_SPI
#include <mach/msm_smd.h>
#include "smd_private.h"
#endif
#include <mach/xmd.h>
#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"

extern struct xmd_data my_xmd;
extern void hsi_ll_wakeup_cp(unsigned int val);

#define h (&my_xmd.hsi_data)

struct
{
  wait_queue_head_t cp_ready_wait;
  char cp_ready_flag;
  spinlock_t lock;
} cp_ready;

void xmd_hsi_boot_cb (void);

void
xmd_hsi_init (void)
{
  printk("\nhsiboot: in xmd hsi init function\n");

  hsi_mem_init ();
  xmd_ch_init ();
  xmd_ch_register_xmd_boot_cb (xmd_hsi_boot_cb);

  init_waitqueue_head (&cp_ready.cp_ready_wait);
  cp_ready.cp_ready_flag = 0;
}

void
xmd_hsi_exit (void)
{
  xmd_ch_exit ();
}

int
xmd_boot_enable_fw_tty (void)
{
  return 0;			//FIRMWARE download not supported.
}

void
xmd_boot_disable_fw_tty (void)
{
  return;			//FIRMWARE download not supported.
}

int
xmd_hsi_power_off (void)
{
  int status = 0;
  struct xmd_hsi_platform_data *pd;

  pd = h->hsi_platform_data;
  if (!pd)
    {
      printk (KERN_INFO "\nCRASH: NULL access in xmd_hsi_power_off\n");
      return -EINVAL;
    }

  xmd_omap_mux_init_signal (pd->hsi_cawake_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_cadata_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caflag_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acready_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acwake_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acdata_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acflag_safe_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caready_safe_pinmux);

  //hsi_ll_wakeup_cp(0);

  cp_ready.cp_ready_flag = 0;

  return (status);
}

void xmd_hsi_power_on_reset (void)
{

  struct xmd_hsi_platform_data *pd;

  pd = h->hsi_platform_data;
  if (!pd)
    {
      printk (KERN_INFO "\nCRASH: NULL access in xmd_hsi_power_on_reset\n");
      return;
    }

  //hsi_ll_wakeup_cp(0);

  xmd_omap_mux_init_signal (pd->hsi_cawake_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_cadata_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caflag_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acready_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acwake_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acdata_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_acflag_pinmux);
  xmd_omap_mux_init_signal (pd->hsi_caready_pinmux);

}

void
xmd_hsi_board_init (void *hsi_platform_data)
{
  //Set wake lines to high [untill power mgmt is ready] TBD
    h->hsi_platform_data = (struct xmd_hsi_platform_data *)hsi_platform_data;
}

int
wait_for_xmd_ack (void)
{

  wait_event_interruptible_timeout (cp_ready.cp_ready_wait,
				    cp_ready.cp_ready_flag == 1, 500);
#if 0
  if (!cp_ready.cp_ready_flag)	//spinlock protection TBD
    return -EINVAL;
#endif
  return 0;
}

void
xmd_hsi_boot_cb (void)
{
  cp_ready.cp_ready_flag = 1;	//spinlock protection TBD
  wake_up_interruptible (&cp_ready.cp_ready_wait);
}
