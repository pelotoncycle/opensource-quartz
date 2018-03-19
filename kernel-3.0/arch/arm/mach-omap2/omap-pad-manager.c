/*
 * linux/arch/arm/mach-omap2/omap-pads-manager.c
 *
 * OMAP2, OMAP3 and OMAP4 pin suspend/resume multiplexing configurations
 *
 * Copyright (C) 2011 - 2012 Innocomm Mobile Technology Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <asm/system.h>
#include <linux/gpio.h>
#include <plat/omap_hwmod.h>
#include <linux/syscore_ops.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "control.h"
#include "mux.h"


static struct omap_pad_manager {
	int				nr_pads;
	struct omap_device_pad		*pads;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
} omap_managed_pads;

int  __init
omap_pads_manager_init(struct omap_pad_state_description *bpads, int nr_pads)
{
	struct omap_device_pad *pads;
	int i,j=0;

	if (!bpads || nr_pads < 1)
		return  -EINVAL;

	for (i = 0; i < nr_pads; i++) {
		struct omap_pad_state_description *bpad = &bpads[i];
		if(bpad->shutdown_mux_name||bpad->suspend_mux_name)
		  j++;
	}
	pads = kzalloc(sizeof(struct omap_device_pad)*j, GFP_KERNEL);
	if (!pads)
		return  -EINVAL;


	omap_managed_pads.pads =pads;
	omap_managed_pads.nr_pads=0;

	for (i = 0; i < nr_pads; i++) {
		struct omap_mux_partition *partition;
		struct omap_pad_state_description *bpad = &bpads[i];
		struct omap_device_pad *pad;
		struct omap_mux *mux;
		int mux_mode;
		if(bpad->shutdown_mux_name||bpad->suspend_mux_name){
			/*We need only record the Dynamic pads*/
			pad = &omap_managed_pads.pads[omap_managed_pads.nr_pads++];
			pad->flags=bpad->flags;
		}else{
			pad=NULL;
		}
		if(bpad->init_mux_name){
			
			mux_mode = omap_mux_get_by_name(bpad->init_mux_name, &partition, &mux);
			if (mux_mode >=0){
				u16 old_mode,new_mode=bpad->init_config|(u16)mux_mode;
				if(pad){
					if (!pad->partition)
						pad->partition = partition;
					if (!pad->mux)
						pad->mux = mux;
					if(!pad->name)
						pad->name=mux->muxnames[0];
					
					pad->enable=new_mode;
				}
				old_mode = omap_mux_read(partition, mux->reg_offset);
				pr_debug("%s: Setting signal %s 0x%04x -> 0x%04x\n",
			 			__func__, bpad->init_mux_name, old_mode, new_mode);
				omap_mux_write(partition, new_mode, mux->reg_offset);


			}
		}		
		if(bpad->suspend_mux_name){
			mux_mode = omap_mux_get_by_name(bpad->suspend_mux_name, &partition, &mux);
			if (mux_mode >=0){
				if (!pad->partition)
					pad->partition = partition;
				if (!pad->mux)
					pad->mux = mux;
				if(!pad->name)
					pad->name=mux->muxnames[0];
				
				pad->idle=(u16)mux_mode|bpad->suspend_config;
				if((mux_mode==3)&&((pad->flags&OMAP_DEVICE_PAD_SUSPEND_GPIO_HI)||
					(pad->flags&OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW))){
					
					gpio_request(mux->gpio,mux->muxnames[3]);
					gpio_direction_output(mux->gpio,(pad->flags&OMAP_DEVICE_PAD_SUSPEND_GPIO_HI));

				}
				pad->flags|=OMAP_DEVICE_PAD_SUSPEND;

			}
		}		
		if(bpad->shutdown_mux_name){
			mux_mode = omap_mux_get_by_name(bpad->shutdown_mux_name, &partition, &mux);
			if (mux_mode >=0){
				if (!pad->partition)
					pad->partition = partition;
				if (!pad->mux)
					pad->mux = mux;
				if(!pad->name)
					pad->name=mux->muxnames[0];
				pad->off=(u16)mux_mode|bpad->shutdown_config;
				pad->flags|=OMAP_DEVICE_PAD_SHUTDOWN;
			}

		}

	}

	return 0;
}

static   int 
omap_pads_manager_suspend(void)
{
	int i;

	if ((!omap_managed_pads.pads)||(!omap_managed_pads.nr_pads))
		return 0;

	for (i = 0; i < omap_managed_pads.nr_pads; i++) {
		struct omap_device_pad *pad=&omap_managed_pads.pads[i];
		
		if(((pad->flags&OMAP_DEVICE_PAD_EARLY_SUSPEND)==OMAP_DEVICE_PAD_SUSPEND)
			||(pad->flags&(OMAP_DEVICE_PAD_WAKEUP))){
			
			u16 val=omap_mux_read(pad->partition,pad->mux->reg_offset);
		
			/*store the current config for next resume*/
			if((pad->flags&OMAP_DEVICE_PAD_EARLY_SUSPEND)==OMAP_DEVICE_PAD_SUSPEND){
				/*The suspend config is specified, replace it by suspend config*/
				pad->enable=val;
				val=pad->idle;

			}
			if(pad->flags&(OMAP_DEVICE_PAD_WAKEUP)){
				/*The pad was config as wakeup enable*/
				val|=OMAP_WAKEUP_EN;

			}

			omap_mux_write(pad->partition,val,pad->mux->reg_offset);
			
		}
	}

	return 0;


}
static  void 
omap_pads_manager_resume(void)
{
	int i;

	if ((!omap_managed_pads.pads)||(!omap_managed_pads.nr_pads))
		return;

	for (i = 0; i < omap_managed_pads.nr_pads; i++) {
		struct omap_device_pad *pad=&omap_managed_pads.pads[i];
		if(((pad->flags&OMAP_DEVICE_PAD_EARLY_SUSPEND)==OMAP_DEVICE_PAD_SUSPEND)
			||(pad->flags&(OMAP_DEVICE_PAD_WAKEUP))){
			omap_mux_write(pad->partition,pad->enable,pad->mux->reg_offset);
		}
	}

	return;


}
static   void 
omap_pads_manager_shutdown(void)
{
	int i;

	if ((!omap_managed_pads.pads)||(!omap_managed_pads.nr_pads))
		return;

	for (i = 0; i < omap_managed_pads.nr_pads; i++) {
		struct omap_device_pad *pad=&omap_managed_pads.pads[i];
		if(pad->flags&(OMAP_DEVICE_PAD_SHUTDOWN)){
			omap_mux_write(pad->partition,pad->off,pad->mux->reg_offset);
		}

	}

	return;


}

#ifdef CONFIG_HAS_EARLYSUSPEND
static   void
omap_pads_manager_early_suspend(struct early_suspend * h)
{
	int i;

	if ((!omap_managed_pads.pads)||(!omap_managed_pads.nr_pads))
		return;

	for (i = 0; i < omap_managed_pads.nr_pads; i++) {
		struct omap_device_pad *pad=&omap_managed_pads.pads[i];
		if((pad->flags&OMAP_DEVICE_PAD_EARLY_SUSPEND)==OMAP_DEVICE_PAD_EARLY_SUSPEND){
				/*store the current config for next resume*/
					pad->enable=omap_mux_read(pad->partition,pad->mux->reg_offset);
				omap_mux_write(pad->partition,pad->idle,pad->mux->reg_offset);
		}
	}

	return;


}
static   void 
omap_pads_manager_late_resume(struct early_suspend * h)
{
	int i;

	if ((!omap_managed_pads.pads)||(!omap_managed_pads.nr_pads))
		return;

	for (i = 0; i < omap_managed_pads.nr_pads; i++) {
		struct omap_device_pad *pad=&omap_managed_pads.pads[i];
		if((pad->flags&OMAP_DEVICE_PAD_EARLY_SUSPEND)==OMAP_DEVICE_PAD_EARLY_SUSPEND){
			omap_mux_write(pad->partition,pad->enable,pad->mux->reg_offset);

		}
	}

	return;

}
#endif



static struct syscore_ops omap_pads_manager_ops = {
	.suspend	= omap_pads_manager_suspend,
	.resume		= omap_pads_manager_resume,
	.shutdown= omap_pads_manager_shutdown,
};


static int __init omap_pads_manager_init_call(void)
{
	register_syscore_ops(&omap_pads_manager_ops);
#ifdef CONFIG_HAS_EARLYSUSPEND
	omap_managed_pads.early_suspend.level = INT_MAX;
	omap_managed_pads.early_suspend.suspend = omap_pads_manager_early_suspend;
	omap_managed_pads.early_suspend.resume = omap_pads_manager_late_resume;
	register_early_suspend(&omap_managed_pads.early_suspend);
#endif	
	return 0;
}

device_initcall(omap_pads_manager_init_call);
