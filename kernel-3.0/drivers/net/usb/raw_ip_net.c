/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>

#define RMNET_ENABLE_USB_RE_ENUMERATION
#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
#define RMNET_ENABLE_FLOW_CONTROL
#endif

#ifdef RMNET_ENABLE_FLOW_CONTROL
#include <linux/usb/cdc.h>

/*
 * Output control lines.
 */

#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02

/*
 * Requests.
 */

#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#endif

/* #define USE_PRIVATE_STATS */

#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

/* ethernet packet ethertype for IP packets */
#define NET_IP_ETHERTYPE		0x08, 0x00

#define	TX_TIMEOUT		10

#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  /* USB_NET_BUFSIZ */

/* maximum interface number supported */
#define MAX_INTFS	5

MODULE_LICENSE("GPL");

static int g_i;

/* To support more rmnet interfaces, increase the default max_intfs or
 * pass kernel module parameter.
 * e.g. insmod raw_ip_net.ko max_intfs=5
 */
static int max_intfs = 2;	/* default number of interfaces */

static unsigned long usb_net_raw_ip_intf[MAX_INTFS] = { 3, 5, 9, 11, 13};
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

module_param(max_intfs, int, 0644);
MODULE_PARM_DESC(max_intfs, "usb net (raw-ip) - max. interfaces supported");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");
/* Make interfaces number configurable */
module_param_array(usb_net_raw_ip_intf, ulong, &max_intfs, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_intf, "usb net (raw-ip) - interface number list");

struct baseband_usb {
	/* semaphore between disconnect/suspend/resume */
	struct semaphore sem;
	/* instance */
	int baseband_index;
#ifdef USE_PRIVATE_STATS
	/* network statistics */
	struct net_device_stats stats;
#endif
	/* usb context */
	struct {
		struct usb_driver *driver;
		struct usb_device *device;
		struct usb_interface *interface;
#ifdef RMNET_ENABLE_FLOW_CONTROL
		struct usb_interface *ctl_interface;
#endif
		struct {
			struct {
				unsigned int in;
				unsigned int out;
			} isoch, bulk, interrupt;
		} pipe;
		/* currently active rx urb */
		struct urb *rx_urb;
		/* currently active tx urb */
		struct urb *tx_urb;
		struct usb_anchor tx_urb_deferred;
		struct workqueue_struct *tx_workqueue;
		struct work_struct tx_work;
	} usb;
	/* re-usable rx urb */
	struct urb *urb_r;
	void *buff;
	/* suspend count */
	int susp_count;
};

static struct baseband_usb *baseband_usb_net[MAX_INTFS] = { 0, 0, 0, 0, 0};

static struct net_device *usb_net_raw_ip_dev[MAX_INTFS] = { 0, 0, 0, 0, 0};

static struct usb_interface *g_usb_interface[MAX_INTFS];

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_rx_urb_comp(struct urb *urb);

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb);
static void usb_net_raw_ip_tx_urb_work(struct work_struct *work);
static void usb_net_raw_ip_tx_urb_comp(struct urb *urb);

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
static struct baseband_usb *baseband_usb_init(int index, unsigned int intf);
static int baseband_usb_open(int index, struct usb_interface *intf, struct baseband_usb *usb);
static void find_usb_pipe(struct baseband_usb *usb);
void baseband_usb_close(struct baseband_usb *usb);
static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb);
static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb);

static int baseband_usb_netdev_init(struct net_device *dev);
static void baseband_usb_netdev_uninit(struct net_device *dev);
static int baseband_usb_netdev_open(struct net_device *dev);
static int baseband_usb_netdev_stop(struct net_device *dev);
static netdev_tx_t baseband_usb_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev);

static struct net_device_ops usb_net_raw_ip_ops;
#endif

static struct usb_driver baseband_usb_driver;

/* Code is from msm-rmnet.c */
static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP)) {
		return 0;
	}

	return 1;
}

#ifdef RMNET_ENABLE_FLOW_CONTROL
/*
 * Functions for ACM control messages.
 */

static int bb_usb_ctrl_msg(struct baseband_usb *bb_usb, int request, int value,
							void *buf, int len)
{
	int retval = usb_control_msg(bb_usb->usb.device, usb_sndctrlpipe(bb_usb->usb.device, 0),
		request, USB_RT_ACM, value,
		bb_usb->usb.ctl_interface->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);
	dev_dbg(&bb_usb->usb.ctl_interface->dev,
			"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
			__func__, request, value, len, retval);
	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
#define bb_usb_set_control(bb_usb, control) \
	bb_usb_ctrl_msg(bb_usb, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0)
#endif

static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
	int i;

#ifdef RMNET_ENABLE_FLOW_CONTROL
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int call_interface_num = -1;
	int data_interface_num = -1;
	int combined_interfaces = 0;
#endif
#else
	int i = g_i, j;

	pr_debug("%s(%d) { intf %p id %p\n", __func__, __LINE__, intf, id);

	pr_debug("i %d\n", i);
#endif

	pr_debug("intf->cur_altsetting->desc.bInterfaceNumber %02x\n",
		intf->cur_altsetting->desc.bInterfaceNumber);
	pr_debug("intf->cur_altsetting->desc.bAlternateSetting %02x\n",
		intf->cur_altsetting->desc.bAlternateSetting);
	pr_debug("intf->cur_altsetting->desc.bNumEndpoints %02x\n",
		intf->cur_altsetting->desc.bNumEndpoints);
	pr_debug("intf->cur_altsetting->desc.bInterfaceClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceSubClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceSubClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceProtocol %02x\n",
		intf->cur_altsetting->desc.bInterfaceProtocol);
	pr_debug("intf->cur_altsetting->desc.iInterface %02x\n",
		intf->cur_altsetting->desc.iInterface);

	/* register interfaces that are assigned to raw-ip */
#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
#ifdef RMNET_ENABLE_FLOW_CONTROL
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_dbg(&intf->dev,
				"Zero length descriptor references\n");
			goto check_interface;
		}
	}

	while (buflen > 0) {
		if (buffer[1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			if (union_header) {
				dev_err(&intf->dev, "More than one "
					"union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			ac_management_function = buffer[3];
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			call_management_function = buffer[3];
			call_interface_num = buffer[4];
			if ((call_management_function & 3) != 3)
				dev_dbg(&intf->dev, "This device cannot do calls on its own. It is not a modem.\n");
			break;
		default:
			/* there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: "
					"type %02x, length %d\n",
					buffer[2], buffer[0]);
			break;
		}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (!union_header) {
		if (call_interface_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			control_interface = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev,"No union descriptor, giving up\n");
				return -ENODEV;
			} else {
				dev_warn(&intf->dev,"No union descriptor, testing for castrated device\n");
				combined_interfaces = 1;
				control_interface = intf;
				return -ENODEV;
			}
		}
	} else {
		control_interface = usb_ifnum_to_if(usb_dev, (call_interface_num = union_header->bMasterInterface0));
		data_interface_num = union_header->bSlaveInterface0;
		if (!control_interface) {
			dev_dbg(&intf->dev, "no interfaces\n");
			return -ENODEV;
		}
	}
check_interface:
#endif

	for (i = 0; i < max_intfs; i++) {
		int err;

#ifdef RMNET_ENABLE_FLOW_CONTROL
		if (usb_net_raw_ip_intf[i] ==
				data_interface_num) {
			if (control_interface) {
				/* Just save the control interface then return directly */
				baseband_usb_net[i]->usb.ctl_interface = control_interface;
				return -ENODEV;
			}
		}
#endif
		if (usb_net_raw_ip_intf[i] ==
				intf->cur_altsetting->desc.bInterfaceNumber) {
			g_usb_interface[i] = intf;

			err = baseband_usb_open(i, intf, baseband_usb_net[i]);
			return err;
		}
	}
#else
	for (j = 0; j < max_intfs; j++) {
		if (usb_net_raw_ip_intf[j] ==
				intf->cur_altsetting->desc.bInterfaceNumber) {
			pr_info("%s: raw_ip using interface %d\n", __func__,
				intf->cur_altsetting->desc.bInterfaceNumber);
			g_usb_interface[j] = intf;
			return 0;
		}
	}
#endif
	pr_debug("%s(%d) }\n", __func__, __LINE__);
	return -ENODEV;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	int i;
	struct urb *urb;

	pr_debug("%s intf %p\n", __func__, intf);

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
		/* unregister network device */
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
#endif

		/* kill usb tx */
		while ((urb = usb_get_from_anchor(&baseband_usb_net[i]->
			usb.tx_urb_deferred)) != (struct urb *) 0) {
			pr_info("%s: kill deferred tx urb %p\n",
				__func__, urb);
			/* decrement count from usb_get_from_anchor() */
			usb_free_urb(urb);
			/* kill tx urb */
			usb_kill_urb(urb);
			/* free tx urb + tx urb transfer buffer */
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
		}
		if (baseband_usb_net[i]->usb.tx_workqueue) {
			flush_workqueue(baseband_usb_net[i]
				->usb.tx_workqueue);
#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
			destroy_workqueue(baseband_usb_net[i]
				->usb.tx_workqueue);
			baseband_usb_net[i]->usb.tx_workqueue
				= (struct workqueue_struct *) 0;
#endif
		}
		if (baseband_usb_net[i]->usb.tx_urb) {
			usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
			baseband_usb_net[i]->usb.tx_urb
				= (struct urb *) 0;
		}
		/* kill usb rx */
		if (baseband_usb_net[i]->usb.rx_urb) {
			usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
			baseband_usb_net[i]->usb.rx_urb
				= (struct urb *) 0;
		}

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
		usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
#endif

#ifdef RMNET_ENABLE_FLOW_CONTROL
		if (baseband_usb_net[i]->usb.ctl_interface) {
			usb_driver_release_interface(&baseband_usb_driver, baseband_usb_net[i]->usb.ctl_interface);
			baseband_usb_net[i]->usb.ctl_interface
				= (struct usb_interface *) 0;
		}
#endif

		usb_driver_release_interface(&baseband_usb_driver, baseband_usb_net[i]->usb.interface);
		/* mark interface as disconnected */
		baseband_usb_net[i]->usb.interface
			= (struct usb_interface *) 0;
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

}

#ifdef CONFIG_PM
static int baseband_usb_driver_suspend(struct usb_interface *intf,
	pm_message_t message)
{
	int i, susp_count;

	pr_debug("%s intf %p\n", __func__, intf);

	pr_debug("%s: cnt %d intf=%p &intf->dev=%p kobj=%s\n",
			__func__, atomic_read(&intf->dev.power.usage_count),
			intf, &intf->dev, kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* increment suspend count */
		susp_count = (baseband_usb_net[i]->susp_count)++;
		if (susp_count > 0) {
			pr_debug("%s: susp_count %d > 0 (already suspended)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_debug("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_debug("%s: susp_count = %d (suspending...)\n",
			__func__, susp_count);
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}
		/* kill usb rx */
		if (!baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already killed\n");
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		pr_debug("%s: kill rx_urb {\n",__func__);
		usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
		pr_debug("%s: kill rx_urb }\n",__func__);
		baseband_usb_net[i]->usb.rx_urb = (struct urb *) 0;
		/* cancel tx urb work (will restart after resume) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		pr_debug("%s: cancel_work_sync {\n",__func__);
		cancel_work_sync(&baseband_usb_net[i]->usb.tx_work);
		pr_debug("%s: cancel_work_sync }\n",__func__);
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

	return 0;
}

static int baseband_usb_driver_resume(struct usb_interface *intf)
{
	int i, err, susp_count;

	pr_debug("%s intf %p\n", __func__, intf);

	pr_debug("%s: cnt %d intf=%p &intf->dev=%p kobj=%s\n",
			__func__, atomic_read(&intf->dev.power.usage_count),
			intf, &intf->dev, kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* decrement suspend count */
		susp_count = --(baseband_usb_net[i]->susp_count);
		if (susp_count > 0) {
			pr_debug("%s: susp_count %d > 0 (not resuming yet)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_debug("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_debug("%s: susp_count = %d (resuming...)\n",
			__func__, susp_count);
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}
		/* start usb rx */
		if (baseband_usb_net[i]->usb.rx_urb) {
			pr_debug("rx_usb already exists\n");
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		/* restart tx urb work (cancelled in suspend) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		queue_work(baseband_usb_net[i]->usb.tx_workqueue,
			&baseband_usb_net[i]->usb.tx_work);
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

	return 0;
}
static int baseband_usb_driver_reset_resume(struct usb_interface *intf)
{
	pr_debug("%s intf %p\n", __func__, intf);
	return baseband_usb_driver_resume(intf);
}
#endif /* CONFIG_PM */

static struct usb_device_id baseband_usb_driver_id_table[] = {
	/* xmm modem vid, pid */
	{ USB_DEVICE(0x1519, 0x0020), },
	{ },
};

static struct usb_driver baseband_usb_driver = {
#if 0
		.name = "bb_raw_ip_net",
#else
		.name = "raw_ip_net",
#endif
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table,
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
};

MODULE_DEVICE_TABLE(usb, baseband_usb_driver_id_table);

static void find_usb_pipe(struct baseband_usb *usb)
{
	struct usb_device *usbdev = usb->usb.device;
	struct usb_interface *intf = usb->usb.interface;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_isoc_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous in\n", n);
			usb->usb.pipe.isoch.in = usb_rcvisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_isoc_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous out\n", n);
			usb->usb.pipe.isoch.out = usb_sndisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk in\n", n);
			usb->usb.pipe.bulk.in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk out\n", n);
			usb->usb.pipe.bulk.out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt in\n", n);
			usb->usb.pipe.interrupt.in = usb_rcvintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt out\n", n);
			usb->usb.pipe.interrupt.out = usb_sndintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else {
			pr_debug("endpoint[%d] skipped\n", n);
		}
	}
}

void baseband_usb_close(struct baseband_usb *usb);

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
static struct baseband_usb *baseband_usb_init(int index, unsigned int intf)
{
	struct baseband_usb *usb;

	pr_debug("baseband_usb_init {\n");

	/* allocate baseband usb structure */
	usb = kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb)
		return (struct baseband_usb *) 0;

	/* create semaphores */
	sema_init(&usb->sem, 1);

	pr_debug("baseband_usb_init }\n");
	return usb;
}
#endif

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
static int baseband_usb_open(int index, struct usb_interface *intf, struct baseband_usb *usb)
{
	char name[32];
	int err;
#else
struct baseband_usb *baseband_usb_open(int index, unsigned int intf)
{
	struct baseband_usb *usb;
	int i;
#endif

	pr_debug("baseband_usb_open {\n");

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
	/* Suspend count need to be reset as 0 */
	usb->susp_count = 0;
#else
	/* allocate baseband usb structure */
	usb = kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb)
		return (struct baseband_usb *) 0;

	/* create semaphores */
	sema_init(&usb->sem, 1);
#endif

	/* open usb interface */
	usb->baseband_index = index;
	usb->usb.driver = &baseband_usb_driver;

#ifndef RMNET_ENABLE_USB_RE_ENUMERATION
	if (!g_usb_interface[index]) {
		/* wait for usb probe */
		for (i = 0; i < 50; i++)
			if (!g_usb_interface[index])
				msleep(20);
		if (!g_usb_interface[index]) {
			pr_err("can't open usb: !g_usb_interface[%d]\n", index);
			kfree(usb);
			return NULL;
		}
	}
#endif

	usb->usb.device = interface_to_usbdev(g_usb_interface[index]);
	usb->usb.interface = g_usb_interface[index];
	find_usb_pipe(usb);
	usb->usb.rx_urb = (struct urb *) 0;
	usb->usb.tx_urb = (struct urb *) 0;
	g_usb_interface[index] = (struct usb_interface *) 0;
	pr_debug("usb->usb.driver->name %s\n", usb->usb.driver->name);
	pr_debug("usb->usb.device %p\n", usb->usb.device);
	pr_debug("usb->usb.interface %p\n", usb->usb.interface);
	pr_debug("usb->usb.pipe.isoch.in %x\n", usb->usb.pipe.isoch.in);
	pr_debug("usb->usb.pipe.isoch.out %x\n", usb->usb.pipe.isoch.out);
	pr_debug("usb->usb.pipe.bulk.in %x\n", usb->usb.pipe.bulk.in);
	pr_debug("usb->usb.pipe.bulk.out %x\n", usb->usb.pipe.bulk.out);
	pr_debug("usb->usb.pipe.interrupt.in %x\n", usb->usb.pipe.interrupt.in);
	pr_debug("usb->usb.pipe.interrupt.out %x\n",
		usb->usb.pipe.interrupt.out);

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
	/* Following codes are copied from usb_net_raw_ip_init() */
	/* register network device */
	usb_net_raw_ip_dev[index] = alloc_netdev(0,
		BASEBAND_USB_NET_DEV_NAME,
		ether_setup);
	if (!usb_net_raw_ip_dev[index]) {
		pr_err("alloc_netdev() failed\n");
		err = -ENOMEM;
		goto error_exit;
	}
	usb_net_raw_ip_dev[index]->netdev_ops = &usb_net_raw_ip_ops;
	usb_net_raw_ip_dev[index]->watchdog_timeo = TX_TIMEOUT;
	random_ether_addr(usb_net_raw_ip_dev[index]->dev_addr);
	err = register_netdev(usb_net_raw_ip_dev[index]);
	if (err < 0) {
		pr_err("cannot register network device - %d\n", err);
		goto error_exit;
	}
	pr_debug("registered baseband usb network device"
			" - dev %p name %s\n", usb_net_raw_ip_dev[index],
			 BASEBAND_USB_NET_DEV_NAME);
	/* start usb rx */
	err = usb_net_raw_ip_setup_rx_urb(baseband_usb_net[index]);
	if (err < 0) {
		pr_err("setup reusable rx urb failed - err %d\n", err);
		goto error_exit;
	}
	err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[index]);
	if (err < 0) {
		pr_err("submit rx failed - err %d\n", err);
		goto error_exit;
	}
	/* start usb tx */
	sprintf(name, "raw_ip_tx_wq-%d",
		baseband_usb_net[index]->baseband_index);
	baseband_usb_net[index]->usb.tx_workqueue
		= create_singlethread_workqueue(name);
	if (!baseband_usb_net[index]->usb.tx_workqueue) {
		pr_err("cannot create workqueue\n");
		goto error_exit;
	}
	INIT_WORK(&baseband_usb_net[index]->usb.tx_work,
		usb_net_raw_ip_tx_urb_work);

	dev_info(&intf->dev, "rmnet%d device\n", index);
	return 0;
error_exit:
	/* unregister network device */
	if (usb_net_raw_ip_dev[index]) {
		unregister_netdev(usb_net_raw_ip_dev[index]);
		free_netdev(usb_net_raw_ip_dev[index]);
		usb_net_raw_ip_dev[index] = (struct net_device *) 0;
	}
	/* close baseband usb */
	if (baseband_usb_net[index]) {
		/* stop usb tx */
		if (baseband_usb_net[index]->usb.tx_workqueue) {
			destroy_workqueue(baseband_usb_net[index]
				->usb.tx_workqueue);
			baseband_usb_net[index]->usb.tx_workqueue
				= (struct workqueue_struct *) 0;
		}
		if (baseband_usb_net[index]->usb.tx_urb) {
			usb_kill_urb(baseband_usb_net[index]->usb.tx_urb);
			baseband_usb_net[index]->usb.tx_urb
				= (struct urb *) 0;
		}
		/* stop usb rx */
		if (baseband_usb_net[index]->usb.rx_urb) {
			usb_kill_urb(baseband_usb_net[index]->usb.rx_urb);
			baseband_usb_net[index]->usb.rx_urb
				= (struct urb *) 0;
		}
		usb_net_raw_ip_free_rx_urb(baseband_usb_net[index]);
		/* close usb */
		//baseband_usb_close(baseband_usb_net[index]);
		//baseband_usb_net[index] = (struct baseband_usb *) 0;
	}
	return err;
#else
	pr_debug("baseband_usb_open }\n");
	return usb;
#endif
}

void baseband_usb_close(struct baseband_usb *usb)
{
	pr_debug("baseband_usb_close {\n");

	/* check input */
	if (!usb)
		return;

	/* close usb driver */
	usb->usb.driver = (struct usb_driver *) 0;

	/* destroy semaphores */
	memset(&usb->sem, 0, sizeof(usb->sem));

	/* free baseband usb structure */
	kfree(usb);

	pr_debug("baseband_usb_close }\n");
}

static int baseband_usb_netdev_init(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_init\n");
	return 0;
}

static void baseband_usb_netdev_uninit(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_uninit\n");
}

static int baseband_usb_netdev_open(struct net_device *dev)
{
#ifdef RMNET_ENABLE_FLOW_CONTROL
	int i;
#endif

	pr_debug("baseband_usb_netdev_open\n");

#ifdef RMNET_ENABLE_FLOW_CONTROL
	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!usb_net_raw_ip_dev[i])
			continue;
		if (usb_net_raw_ip_dev[i] != dev) {
			pr_debug("%p != %p\n",
				usb_net_raw_ip_dev[i], dev);
			continue;
		}

		/* Send DTR to modem */
		if (baseband_usb_net[i] && baseband_usb_net[i]->usb.ctl_interface)
			bb_usb_set_control(baseband_usb_net[i], ACM_CTRL_DTR | ACM_CTRL_RTS);
		else
			pr_err("Cannot enable flow control for rmnet%d\n", i);
	}
#endif

	netif_start_queue(dev);
	return 0;
}

static int baseband_usb_netdev_stop(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_stop\n");
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t baseband_usb_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{
	int i;
	struct baseband_usb *usb;
	int err;

	pr_debug("baseband_usb_netdev_start_xmit\n");

	/* check input */
	if (!skb) {
		pr_err("no skb\n");
		return NETDEV_TX_BUSY;
	}
	if (!dev) {
		pr_err("no net dev\n");
		return NETDEV_TX_BUSY;
	}

	/* find index of network device which is transmitting */
	for (i = 0; i < max_intfs; i++) {
		if (usb_net_raw_ip_dev[i] == dev)
			break;
	}
	if (i >= max_intfs) {
		pr_err("unknown net dev %p\n", dev);
		return NETDEV_TX_BUSY;
	}
	usb = baseband_usb_net[i];

	/* autoresume if suspended */
	if (usb->usb.interface) {
		usb_autopm_get_interface_async(usb->usb.interface);
	} else {
		pr_err("%s: tx get interface error\n", __func__);
		netif_stop_queue(dev);
#ifdef USE_PRIVATE_STATS
		usb->stats.tx_errors++;
#else
		dev->stats.tx_errors++;
#endif
		return NETDEV_TX_BUSY;
	}

	/* submit tx urb */
	err = usb_net_raw_ip_tx_urb_submit(usb, skb);
	if (err < 0) {
		pr_err("%s: tx urb submit error\n", __func__);
		netif_stop_queue(dev);
#ifdef USE_PRIVATE_STATS
		usb->stats.tx_errors++;
#else
		dev->stats.tx_errors++;
#endif
		return NETDEV_TX_BUSY;
	}

	return NETDEV_TX_OK;
}

#ifdef USE_PRIVATE_STATS
static struct net_device_stats *baseband_usb_netdev_get_stats(
				struct net_device *dev)
{
	int i;
	for (i = 0; i < max_intfs; i++) {
		if (dev == usb_net_raw_ip_dev[i]) {
			pr_debug("%s idx(%d)\n", __func__, i);
			return &baseband_usb_net[i]->stats;
		}
	}
	pr_debug("%s mismatch dev, default idx(0)\n", __func__);
	return &baseband_usb_net[0]->stats;
}
#endif

static struct net_device_ops usb_net_raw_ip_ops = {
	.ndo_init =		baseband_usb_netdev_init,
	.ndo_uninit =		baseband_usb_netdev_uninit,
	.ndo_open =		baseband_usb_netdev_open,
	.ndo_stop =		baseband_usb_netdev_stop,
	.ndo_start_xmit =	baseband_usb_netdev_start_xmit,
#ifdef USE_PRIVATE_STATS
	.ndo_get_stats = baseband_usb_netdev_get_stats,
#endif
};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb)
{
	struct urb *urb;
	void *buf;
	int err;

	pr_debug("usb_net_raw_ip_rx_urb_submit { usb %p\n", usb);

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting rx urb\n");
		return -EINVAL;
	}
	if (usb->usb.rx_urb) {
		pr_err("previous urb still active\n");
		return -EBUSY;
	}
	if (!usb->urb_r || !usb->buff) {
		pr_err("no reusable rx urb found\n");
		return -ENOMEM;
	}

	/* reuse rx urb */
	urb = usb->urb_r;
	buf = usb->buff;
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.in,
		buf, USB_NET_BUFSIZ,
		usb_net_raw_ip_rx_urb_comp,
		usb);
	urb->transfer_flags = 0;

	/* submit rx urb */
	usb_mark_last_busy(usb->usb.device);
	usb->usb.rx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.rx_urb = (struct urb *) 0;
		return err;
	}

	pr_debug("usb_net_raw_ip_rx_urb_submit }\n");
	return err;
}

static void usb_net_raw_ip_rx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb;
	int i;
	struct sk_buff *skb;
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
		/* Destination MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* Source MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* EtherType */
		NET_IP_ETHERTYPE,
	};

	pr_debug("usb_net_raw_ip_rx_urb_comp { urb %p\n", urb);

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	usb = (struct baseband_usb *)urb->context;
	i = usb->baseband_index;
	switch (urb->status) {
	case 0:
		break;
	case -ESHUTDOWN:
		/* fall through */
		pr_info("%s: rx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		goto err_exit;
	case -EPROTO:
		pr_info("%s: rx urb %p - link shutdown %d EPROTO\n",
			__func__, urb, urb->status);
		goto err_exit;
	default:
		pr_info("%s: rx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}

	/* put rx urb data in rx buffer */
	if (urb->actual_length > 0) {
		pr_debug("usb_net_raw_ip_rx_urb_comp - "
			"urb->actual_length %d\n", urb->actual_length);
		/* allocate skb with space for
		 * - dummy ethernet header
		 * - rx IP packet from modem
		 */
		skb = netdev_alloc_skb(usb_net_raw_ip_dev[i],
			NET_IP_ALIGN + 14 + urb->actual_length);
		if (skb) {
			unsigned char *ptr;
			/* generate a dummy ethernet header
			 * since modem sends IP packets without
			 * any ethernet headers
			 */
			memcpy(ethernet_header + 0,
				usb_net_raw_ip_dev[i]->dev_addr, 6);
			memcpy(ethernet_header + 6,
				"0x01\0x02\0x03\0x04\0x05\0x06", 6);
			/* fill skb with
			 * - dummy ethernet header
			 * - rx IP packet from modem
			 */
			skb_reserve(skb, NET_IP_ALIGN);
			dst = skb_put(skb, 14);
			ptr = dst;
			memcpy(dst, ethernet_header, 14);
			if ((((unsigned char *) urb->transfer_buffer)[0]
				& 0xf0) == 0x60) {
				/* ipv6 ether type */
				dst[12] = 0x86;
				dst[13] = 0xdd;
			}
			dst = skb_put(skb, urb->actual_length);
			memcpy(dst, urb->transfer_buffer, urb->actual_length);
			skb->protocol = eth_type_trans(skb,
				usb_net_raw_ip_dev[i]);
			pr_debug("%s: ntohs(skb->protocol) %04x (%s)\n",
				__func__, ntohs(skb->protocol),
				(ntohs(skb->protocol) == 0x0800)
					? "IPv4"
					: (ntohs(skb->protocol) == 0x86dd)
					? "IPv6"
					: "unknown");
			pr_debug("%s: %02x %02x %02x %02x\n", __func__,
				((unsigned char *)urb->transfer_buffer)[0],
				((unsigned char *)urb->transfer_buffer)[1],
				((unsigned char *)urb->transfer_buffer)[2],
				((unsigned char *)urb->transfer_buffer)[3]);
			/* pass skb to network stack */
			if (netif_rx(skb) < 0) {
				pr_err("usb_net_raw_ip_rx_urb_comp_work - "
					"netif_rx(%p) failed\n", skb);
#if 0
				kfree_skb(skb);
#else
				dev_kfree_skb_irq(skb);
#endif
#ifdef USE_PRIVATE_STATS
				usb->stats.rx_errors++;
			} else {
				usb->stats.rx_packets++;
				usb->stats.rx_bytes +=
				    (14 + urb->actual_length);
#else
				usb_net_raw_ip_dev[i]->stats.rx_errors++;
			} else {
				if (count_this_packet(ptr, skb->len)) {
					usb_net_raw_ip_dev[i]->stats.rx_packets++;
					usb_net_raw_ip_dev[i]->stats.rx_bytes += skb->len;
				}
#endif
			}
		} else {
			pr_err("usb_net_raw_ip_rx_urb_comp_work - "
				"netdev_alloc_skb() failed\n");
		}
	}

	/* mark rx urb complete */
	usb->usb.rx_urb = (struct urb *) 0;

	/* do not submit urb if interface is suspending */
	if (urb->status == -ENOENT)
		return;

	/* submit next rx urb */
	usb_net_raw_ip_rx_urb_submit(usb);
	return;

err_exit:
	/* mark rx urb complete */
	usb->usb.rx_urb = (struct urb *) 0;

	pr_debug("usb_net_raw_ip_rx_urb_comp }\n");
	return;
}

static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_setup_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (usb->urb_r) {
		pr_err("%s: reusable rx urb already allocated\n", __func__);
		return -EINVAL;
	}

	/* allocate reusable rx urb */
	usb->urb_r = usb_alloc_urb(0, GFP_ATOMIC);
	if (!usb->urb_r) {
		pr_err("usb_alloc_urb() failed\n");
		return -ENOMEM;
	}
	usb->buff = kzalloc(USB_NET_BUFSIZ, GFP_ATOMIC);
	if (!usb->buff) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
		return -ENOMEM;
	}

	pr_debug("usb_net_raw_setup_ip_rx_urb }\n");
	return 0;
}

static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb)
{
	pr_debug("usb_net_raw_ip_free_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return;
	}

	/* free reusable rx urb */
	if (usb->urb_r) {
		usb_free_urb(usb->urb_r);
		usb->urb_r = (struct urb *) 0;
	}
	if (usb->buff) {
		kfree(usb->buff);
		usb->buff = (void *) 0;
	}

	pr_debug("usb_net_raw_ip_free_rx_urb }\n");
}

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb)
{
	struct urb *urb;
	unsigned char *buf;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_submit {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting tx urb\n");
		return -EINVAL;
	}
	if (!skb) {
		pr_err("%s: !skb\n", __func__);
		usb_autopm_put_interface_async(usb->usb.interface);
		return -EINVAL;
	}

	/* allocate urb */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		usb_autopm_put_interface_async(usb->usb.interface);
		return -ENOMEM;
	}
	buf = kzalloc(skb->len - 14, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return -ENOMEM;
	}
	err = skb_copy_bits(skb, 14, buf, skb->len - 14);
	if (err < 0) {
		pr_err("skb_copy_bits() failed - %d\n", err);
		kfree(buf);
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		return err;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.out,
		buf, skb->len - 14,
		usb_net_raw_ip_tx_urb_comp,
		usb);
	urb->transfer_flags = URB_ZERO_PACKET;
	pr_debug("%s: ntohs(skb->protocol) %04x (%s)\n",
		__func__, ntohs(skb->protocol),
		(ntohs(skb->protocol) == 0x0800)
			? "IPv4"
			: (ntohs(skb->protocol) == 0x86dd)
			? "IPv6"
			: "unknown");
	pr_debug("%s: %02x %02x %02x %02x\n", __func__,
		((unsigned char *)urb->transfer_buffer)[0],
		((unsigned char *)urb->transfer_buffer)[1],
		((unsigned char *)urb->transfer_buffer)[2],
		((unsigned char *)urb->transfer_buffer)[3]);

	/* queue tx urb work */
	usb_anchor_urb(urb, &usb->usb.tx_urb_deferred);
	queue_work(usb->usb.tx_workqueue, &usb->usb.tx_work);

	/* free skb */
#if 0
	consume_skb(skb);
#else
	dev_kfree_skb_irq(skb);
#endif

	pr_debug("usb_net_raw_ip_tx_urb_submit }\n");
	return 0;
}

static void usb_net_raw_ip_tx_urb_work(struct work_struct *work)
{
	struct baseband_usb *usb
		= container_of(work, struct baseband_usb, usb.tx_work);
	struct urb *urb;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_work {\n");

	/* check if tx urb(s) queued */
	if (usb == NULL ||
		(!usb->usb.tx_urb &&
		usb_anchor_empty(&usb->usb.tx_urb_deferred))) {
		pr_debug("%s: nothing to do!\n", __func__);
		return;
	}

	/* check if usb interface disconnected */
	if (!usb->usb.interface) {
		pr_err("%s: not submitting tx urb -interface disconnected\n",
			__func__);
		return;
	}

	/* check if suspended */
	if (usb->susp_count > 0) {
		pr_info("%s: usb->susp_count %d > 0 (suspended)\n",
			__func__, usb->susp_count);
		return;
	}

	/* submit queued tx urb(s) */
	while ((urb = usb_get_from_anchor(&usb->usb.tx_urb_deferred))
		!= (struct urb *) 0) {
		/* decrement count from usb_get_from_anchor() */
		usb_free_urb(urb);
		/* check if usb interface disconnected */
		if (!usb->usb.interface) {
			pr_err("%s: not submitting tx urb %p"
				" - interface disconnected\n",
				__func__, urb);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
#ifdef USE_PRIVATE_STATS
			usb->stats.tx_errors++;
#else
			usb_net_raw_ip_dev[usb->baseband_index]->stats.tx_errors++;
#endif
			continue;
		}
		/* autoresume before tx */
		usb_mark_last_busy(usb->usb.device);
		/* submit tx urb */
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			pr_err("%s: usb_submit_urb(%p) failed - err %d\n",
				__func__, urb, err);
			usb_autopm_put_interface_async(usb->usb.interface);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
#ifdef USE_PRIVATE_STATS
			usb->stats.tx_errors++;
#else
			usb_net_raw_ip_dev[usb->baseband_index]->stats.tx_errors++;
#endif
			continue;
		}
		/* free tx urb
		 * - actual urb free occurs when refcnt which was incremented
		 *   in usb_submit_urb is decremented to 0 (usually after urb
		 *   completion function returns)
		 * - tx urb transfer buffer will be freed in urb completion
		 *   function
		 */
		usb_free_urb(urb);
	}

	pr_debug("usb_net_raw_ip_tx_urb_work }\n");
}

static void usb_net_raw_ip_tx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb;

	pr_debug("usb_net_raw_ip_tx_urb_comp {\n");

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	usb = (struct baseband_usb *)urb->context;
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
		/* fall through */
	case -ESHUTDOWN:
		/* fall through */
	case -EPROTO:
		pr_info("%s: tx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		usb_autopm_put_interface_async(usb->usb.interface);
		goto err_exit;
	default:
		pr_info("%s: tx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}
#ifdef USE_PRIVATE_STATS
	if (urb->status)
		usb->stats.tx_errors++;
	else {
		usb->stats.tx_packets++;
		usb->stats.tx_bytes += urb->transfer_buffer_length;
	}
#else
	if (urb->status)
		usb_net_raw_ip_dev[usb->baseband_index]->stats.tx_errors++;
	else {
		usb_net_raw_ip_dev[usb->baseband_index]->stats.tx_packets++;
		usb_net_raw_ip_dev[usb->baseband_index]->stats.tx_bytes += urb->transfer_buffer_length;
	}
#endif

	/* autosuspend after tx completed */
	if (!usb->usb.interface) {
		pr_err("%s: usb interface disconnected"
			" before tx urb completed!\n",
			__func__);
		goto err_exit;
	}
#if 0
	usb_autopm_put_interface(usb->usb.interface);
#else
	usb_autopm_put_interface_async(usb->usb.interface);
#endif

err_exit:
	/* free tx urb transfer buffer */
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}
	pr_debug("usb_net_raw_ip_tx_urb_comp }\n");
}

static int usb_net_raw_ip_init(void)
{
	int i;
	int err;
#ifndef RMNET_ENABLE_USB_RE_ENUMERATION
	char name[32];
#endif

	pr_debug("usb_net_raw_ip_init {\n");

#ifndef RMNET_ENABLE_USB_RE_ENUMERATION
	err = usb_register(&baseband_usb_driver);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		return err;
	}
#endif

	/* create multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* open baseband usb */
		g_i = i;
#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
		baseband_usb_net[i] = baseband_usb_init(i,
						usb_net_raw_ip_intf[i]);
#else
		baseband_usb_net[i] = baseband_usb_open(i,
						usb_net_raw_ip_intf[i]);
#endif
		if (!baseband_usb_net[i]) {
			pr_err("cannot open baseband usb net\n");
			err = -1;
			goto error_exit;
		}
		init_usb_anchor(&baseband_usb_net[i]->usb.tx_urb_deferred);
#ifndef RMNET_ENABLE_USB_RE_ENUMERATION
		/* register network device */
		usb_net_raw_ip_dev[i] = alloc_netdev(0,
			BASEBAND_USB_NET_DEV_NAME,
			ether_setup);
		if (!usb_net_raw_ip_dev[i]) {
			pr_err("alloc_netdev() failed\n");
			err = -ENOMEM;
			goto error_exit;
		}
		usb_net_raw_ip_dev[i]->netdev_ops = &usb_net_raw_ip_ops;
		usb_net_raw_ip_dev[i]->watchdog_timeo = TX_TIMEOUT;
		random_ether_addr(usb_net_raw_ip_dev[i]->dev_addr);
		err = register_netdev(usb_net_raw_ip_dev[i]);
		if (err < 0) {
			pr_err("cannot register network device - %d\n", err);
			goto error_exit;
		}
		pr_debug("registered baseband usb network device"
				" - dev %p name %s\n", usb_net_raw_ip_dev[i],
				 BASEBAND_USB_NET_DEV_NAME);
		/* start usb rx */
		err = usb_net_raw_ip_setup_rx_urb(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("setup reusable rx urb failed - err %d\n", err);
			goto error_exit;
		}
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			goto error_exit;
		}
		/* start usb tx */
		sprintf(name, "raw_ip_tx_wq-%d",
			baseband_usb_net[i]->baseband_index);
		baseband_usb_net[i]->usb.tx_workqueue
			= create_singlethread_workqueue(name);
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("cannot create workqueue\n");
			goto error_exit;
		}
		INIT_WORK(&baseband_usb_net[i]->usb.tx_work,
			usb_net_raw_ip_tx_urb_work);
#endif	/* RMNET_ENABLE_USB_RE_ENUMERATION */
	}

#ifdef RMNET_ENABLE_USB_RE_ENUMERATION
	err = usb_register(&baseband_usb_driver);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		return err;
	}
#endif

	pr_debug("usb_net_raw_ip_init }\n");
	return 0;

error_exit:
	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* unregister network device */
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		/* close baseband usb */
		if (baseband_usb_net[i]) {
			/* stop usb tx */
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			/* stop usb rx */
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			/* close usb */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}
	usb_deregister(&baseband_usb_driver);

	return err;
}

static void usb_net_raw_ip_exit(void)
{
	int i;

	pr_debug("usb_net_raw_ip_exit {\n");

	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* unregister network device */
		if (usb_net_raw_ip_dev[i]) {
			unregister_netdev(usb_net_raw_ip_dev[i]);
			free_netdev(usb_net_raw_ip_dev[i]);
			usb_net_raw_ip_dev[i] = (struct net_device *) 0;
		}
		/* close baseband usb */
		if (baseband_usb_net[i]) {
			/* stop usb tx */
			if (baseband_usb_net[i]->usb.tx_workqueue) {
				destroy_workqueue(baseband_usb_net[i]
					->usb.tx_workqueue);
				baseband_usb_net[i]->usb.tx_workqueue
					= (struct workqueue_struct *) 0;
			}
			if (baseband_usb_net[i]->usb.tx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
				baseband_usb_net[i]->usb.tx_urb
					= (struct urb *) 0;
			}
			/* stop usb rx */
			if (baseband_usb_net[i]->usb.rx_urb) {
				usb_kill_urb(baseband_usb_net[i]->usb.rx_urb);
				baseband_usb_net[i]->usb.rx_urb
					= (struct urb *) 0;
			}
			usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
			/* close usb */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}

	pr_debug("close usb driver {\n");
	usb_deregister(&baseband_usb_driver);
	pr_debug("close usb driver }\n");

	pr_debug("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)
