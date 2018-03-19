/*
 * arch/arm/mach-omap2/xmd-c2c.c
 *
 * xmd-c2c.c -- XMM C2C logical driver
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

#undef DEBUG

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
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/memblock.h>
#include <linux/c2c.h>

#include <mach/gpio.h>
#include <mach/msm_smd.h>
#include "smd_private.h"
#include <mach/xmd.h>



extern struct xmd_data my_xmd;
#define c (&my_xmd.c2c_data)


#define XMD_C2C_GENO_OPP_TRIG	27
#define XMD_C2C_GENO_OPP_D0	28
#define XMD_C2C_GENO_OPP_D1	29
#define XMD_C2C_IRQ_NAME "xmd_geno_opp"

#define XMD_C2C_MISC_DEV_NAME "c2c"
#define XMD_C2C_DEFAULT_ADDRESS	0x9C000000
#define XMD_C2C_DEFAULT_SIZE 	(32*SZ_1M)


/*---------------------------- MMAP DEVICE ----------------------------*/

/*
 * Get the C2C memory address/size: c2c=size@start,
 * where start and size are "size[KkMm]"
 */

unsigned long c2c_mem_address = XMD_C2C_DEFAULT_ADDRESS;
unsigned long c2c_mem_size = XMD_C2C_DEFAULT_SIZE;

static int __init xmd_c2c_early_mem(char *p)
{
	char *endp;

	c2c_mem_size = memparse(p, &endp);
	if (*endp == '@')
		c2c_mem_address = memparse(endp + 1, NULL);

	return 0;
}
early_param("c2c", xmd_c2c_early_mem);


/*
 * xmd-c2c device open() method
 */
static int xmd_c2c_open(struct inode *inode, struct file *filp)
{
	dev_dbg(c->dev, "%s\n", __func__);

#if 0
	/*
	 * Map Distant Device memory to IO mem
	 */

	if (!request_mem_region(c->mem_pa, c->mem_size,
					XMD_C2C_MISC_DEV_NAME)) {
		dev_err(c->dev, "%s ERROR request mem region\n", __func__);

		return -EBUSY;
	}

	c->mem_va = ioremap_nocache(c->mem_pa, c->mem_size);
	if (!c->mem_va) {
		dev_err(c->dev, "%s ERROR ioremap\n", __func__);
		release_mem_region(c->mem_pa, c->mem_size);
		return -ENOMEM;
	}
#endif

	return 0;
}


/*
 * xmd-c2c device release() method
 */
static int xmd_c2c_release(struct inode *inode, struct file *filp)
{
	dev_dbg(c->dev, "%s\n", __func__);

#if 0
	/*
	 * Unmap Distant Device shared memory from IO mem
	 */

	iounmap(c->mem_va);
	release_mem_region(c->mem_pa, c->mem_size);
#endif

	return 0;
}


/*
 * xmd-c2c device mmap() methods
 */

void xmd_c2c_vma_open(struct vm_area_struct *vma)
{
	dev_dbg(c->dev, "%s\n", __func__);
}

void xmd_c2c_vma_close(struct vm_area_struct *vma)
{
	dev_dbg(c->dev, "%s\n", __func__);
}

static struct vm_operations_struct xmd_c2c_remap_vm_ops = {
	.open  = xmd_c2c_vma_open,
	.close = xmd_c2c_vma_close,
};

static int xmd_c2c_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long size;
	unsigned long off;

	dev_dbg(c->dev, "%s\n", __func__);

	start = (unsigned long) c->mem_pa;
	size = (vma->vm_end - vma->vm_start);
	off = vma->vm_pgoff << PAGE_SHIFT;

	if ((size + off) > c->mem_size) {
		pr_err("XMD ERROR: mmap len incorrect\n");
		return -EINVAL;
	}

	vma->vm_flags |= (VM_RESERVED | VM_IO | VM_PFNMAP);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(		vma,
					vma->vm_start,
					(start + off) >> PAGE_SHIFT,
					vma->vm_end - vma->vm_start,
					vma->vm_page_prot)) {
		pr_err("XMD ERROR: mmaping c2c memory\n");
		return -EAGAIN;
	}

	vma->vm_ops = &xmd_c2c_remap_vm_ops;
	xmd_c2c_vma_open(vma);
	return 0;
}


static const struct file_operations xmd_c2c_fops = {
	.owner		= THIS_MODULE,
	.open		= xmd_c2c_open,
	.release	= xmd_c2c_release,
	.mmap		= xmd_c2c_mmap,
};

static struct miscdevice xmd_c2c_miscdev = {
	MISC_DYNAMIC_MINOR,
	XMD_C2C_MISC_DEV_NAME,
	&xmd_c2c_fops
};


/*---------------------------- GENO OPP CONTROL ----------------------------*/


/*
 * C2C_GENO_TRIG worker
 */

struct xmd_c2c_bwp {
	int rx_bw;
	int tx_bw;
};

static const struct xmd_c2c_bwp xmd_c2c_geno_code_to_bw[] = {
/* RX Bandwidth (MHz), TX Bandwidth (MHz) */
	{-1, 	-1},	/* {D1, D0} = {0, 0} - RESERVED */
	{-1,	-1},	/* {D1, D0} = {0, 1} - RESERVED */
	{200,	200},	/* {D1, D0} = {1, 0} - OPP50    */
	{400,	400},	/* {D1, D0} = {1, 1} - OPP100   */
};


static void xmd_c2c_geno_work (struct work_struct *work)
{
	int d0, d1, bwp, ret;
	int rx, tx;

	dev_dbg(c->dev, "%s\n", __func__);

	d0 = c2c_geno_get_value(XMD_C2C_GENO_OPP_D0);
	d1 = c2c_geno_get_value(XMD_C2C_GENO_OPP_D1);

	if ((d0 < 0) || (d1 < 0)) {
		dev_err(c->dev, "%s ERROR getting GENO values D0: %d D1: %d\n",
			__func__, d0, d1);
		return;
	}
	bwp = ((d1 << 1) | (d0 << 0));

	rx = xmd_c2c_geno_code_to_bw[bwp].rx_bw;
	tx = xmd_c2c_geno_code_to_bw[bwp].tx_bw;

	if ((rx < 0) || (tx < 0)) {
		dev_err(c->dev, "%s ERROR unknown c2c bw config: %d\n",
			__func__, bwp);
		return;
	}

	ret = c2c_set_min_frequency(tx, rx);
	if (ret)
		dev_err(c->dev, "%s ERROR during c2c min freq config\n",
								__func__);
}


/*
 * C2C_GENO_TRIG hard irq
 */

static irqreturn_t xmd_c2c_geno_inth(int irq, void *data)
{
	/* Kick workqueue */
	schedule_work(&c->geno_work);

	return IRQ_HANDLED;
}


/*---------------------------- DRIVER INIT ----------------------------*/

/*
 * Memory boot time reservation - called from board init file
 */

long xmd_c2c_mem_reserve(void)
{
	long ret, add, len;

	pr_info("Reserving C2C SDRAM block phys addr: 0x%08lx  size: 0x%8lx\n",
				c2c_mem_address, c2c_mem_size);

	/*
	 * Double reservation size as region on EMIF2 is not interleaved hence
	 * shall not be considered by kernel as valid memory due to lower
	 * performances.
	 */
	add = c2c_mem_address;
	len = 2*c2c_mem_size;
	ret = memblock_remove(add, len);
	if (ret < 0)
		pr_err("XMD ERROR: could not remove c2c memory block\n");

	return ret;
}


/*
 * XMD module init
 */

int __init xmd_c2c_dd_init()
{
	int ret = 0, irq, err;
	struct c2c_config_param config;

	pr_info("XMD: %s\n", __func__);

	/* Register misc c2c device */
	ret = misc_register(&xmd_c2c_miscdev);
	if (ret < 0) {
		pr_err("XMD ERROR: could not register %s device\n",
						XMD_C2C_MISC_DEV_NAME);
		return ret;
	}
	c->dev = xmd_c2c_miscdev.this_device;

	c->mem_pa = c2c_mem_address;
	c->mem_size = c2c_mem_size;
	pr_info("XMD: c2c mem phys 0x%08lx size 0x%8lx\n",
		(long int) c->mem_pa, c->mem_size);

	/* C2C physical driver setup */
	config.rx_width = 8;
	config.tx_width = 16;
	config.geni_mask = ~(0L); /* GENI software control only */
	ret = c2c_config(&config);
	if (ret < 0) {
		pr_err("XMD ERROR: invalid C2C config\n");
		goto rollback1;
	}

	/* GENO allocation for DD OPP control */
	ret = c2c_geno_request(XMD_C2C_GENO_OPP_TRIG);
	if (ret < 0) {
		pr_err("XMD ERROR: could not request XMD_C2C_GENO_OPP_TRIG\n");
		goto rollback2;
	}
	ret = c2c_geno_request(XMD_C2C_GENO_OPP_D0);
	if (ret < 0) {
		pr_err("XMD ERROR: could not request XMD_C2C_GENO_OPP_D0\n");
		goto rollback2;
	}
	ret = c2c_geno_request(XMD_C2C_GENO_OPP_D1);
	if (ret < 0) {
		pr_err("XMD ERROR: could not request XMD_C2C_GENO_OPP_D1\n");
		goto rollback2;
	}

	INIT_WORK(&c->geno_work, xmd_c2c_geno_work);
	irq = c2c_geno_to_irq(XMD_C2C_GENO_OPP_TRIG);
	err = request_irq(irq, xmd_c2c_geno_inth,
				IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
							XMD_C2C_IRQ_NAME, c);
	if (err < 0) {
		pr_err("XMD ERROR: failed to request IRQ %s number %d\n",
						XMD_C2C_IRQ_NAME, irq);
		goto rollback2;
	}
	enable_irq_wake(irq);

	return 0;
rollback2:
	c2c_geno_free(XMD_C2C_GENO_OPP_TRIG);
	c2c_geno_free(XMD_C2C_GENO_OPP_D0);
	c2c_geno_free(XMD_C2C_GENO_OPP_D1);
rollback1:
	misc_deregister(&xmd_c2c_miscdev);
	return ret;
}


/*
 * XMD module exit
 */

void __exit xmd_c2c_dd_exit()
{
	int irq;

	irq = c2c_geno_to_irq(XMD_C2C_GENO_OPP_TRIG);
	disable_irq_wake(irq);
	free_irq(irq, c);
	c2c_geno_free(XMD_C2C_GENO_OPP_TRIG);
	c2c_geno_free(XMD_C2C_GENO_OPP_D0);
	c2c_geno_free(XMD_C2C_GENO_OPP_D1);

	misc_deregister(&xmd_c2c_miscdev);
	return;
}
