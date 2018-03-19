/*
 * Intel Mobile Communcation xmm6xx Serial USB driver
 *
 *	Copyright (c) 2011 IMC Incorporated.
 *	Copyright (c) 2011 Quan Zhang <quan.zhang@intel.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */
#define DEBUG

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/slab.h>
#include "usb-wwan.h"

#define DRIVER_AUTHOR "IMC <quan.zhang@intel.com>"
#define DRIVER_DESC "Intel Mobile Communication USB Serial driver"

static int debug = 1;

/* see modem-side bootloader : usb_config.[h|c] */
static const struct usb_device_id id_table[] = {
	{USB_DEVICE(0x058b, 0x0041)},
	{ }				/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver imc_driver = {
	.name			= "imcserial",
	.probe			= usb_serial_probe,
	.disconnect		= usb_serial_disconnect,
	.id_table		= id_table,
	.suspend		= usb_serial_suspend,
	.resume			= usb_serial_resume,
	.supports_autosuspend	= true,
};

static int imc_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
	struct usb_wwan_intf_private *data;
	struct usb_host_interface *intf = serial->interface->cur_altsetting;
	int retval = 0;
	__u8 nintf;
	__u8 ifnum;

	err("%s", __func__);

	nintf = serial->dev->actconfig->desc.bNumInterfaces;
	err("Num Interfaces = %d", nintf);
	ifnum = intf->desc.bInterfaceNumber;
	err("This Interface = %d", ifnum);

	data = serial->private = kzalloc(sizeof(struct usb_wwan_intf_private),
					 GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->susp_lock);

	return retval;
}

static struct usb_serial_driver imc_serial_driver = {
	.driver = {
		.owner     = THIS_MODULE,
		.name      = "imcserial",
	},
	.description         = "Intel Mobile Communication USB modem",
	.id_table            = id_table,
	.usb_driver          = &imc_driver,
	.num_ports           = 1,
	.probe               = imc_probe,
	.open		     = usb_wwan_open,
	.close		     = usb_wwan_close,
	.write		     = usb_wwan_write,
	.write_room	     = usb_wwan_write_room,
	.chars_in_buffer     = usb_wwan_chars_in_buffer,
	.attach		     = usb_wwan_startup,
	.disconnect	     = usb_wwan_disconnect,
	.release	     = usb_wwan_release,
#ifdef CONFIG_PM
	.suspend	     = usb_wwan_suspend,
	.resume		     = usb_wwan_resume,
#endif
};

static int __init imc_init(void)
{
	int retval;

	retval = usb_serial_register(&imc_serial_driver);
	if (retval) {
		pr_err("%s(): usb_serial_register() returns %d\n",  __func__, retval);
		return retval;
	}

	retval = usb_register(&imc_driver);
	if (retval) {
		pr_err("%s(): usb_registerr() returns %d, deregister imc_serial_driver\n", __func__, retval);
		usb_serial_deregister(&imc_serial_driver);
		return retval;
	}

	return 0;
}

static void __exit imc_exit(void)
{
	usb_deregister(&imc_driver);
	usb_serial_deregister(&imc_serial_driver);
}

module_init(imc_init);
module_exit(imc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
