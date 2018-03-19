/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _FASTBOOT_H
#define _FASTBOOT_H

#include <errno.h>
#include <linux/usb/ch9.h>
#include <usbdescriptors.h>
#include <linux/usb/gadget.h>

/*-------------------------------------------------------------------------*/

#define FASTBOOT_STR_PRODUCT_IDX		1
#define FASTBOOT_STR_SERIAL_IDX			2
#define FASTBOOT_STR_CONFIG_IDX			3
#define FASTBOOT_STR_INTERFACE_IDX		4
#define FASTBOOT_STR_MANUFACTURER_IDX	5
#define FASTBOOT_STR_PROC_REV_IDX		6
#define FASTBOOT_STR_PROC_TYPE_IDX		7
#define FASTBOOT_STR_PROC_VERSION_IDX	8
#define FASTBOOT_STR_MAX_IDX			(FASTBOOT_STR_PROC_VERSION_IDX + 1)

/**
 * Inputs:
 *   1. struct usb_string *
 *   2. struct usb_device_descriptor *
 *
 * Outputs:
 *   1. USB strings
 *   2. USB VID & PID
 */
void fastboot_board_init(struct usb_string *strings, struct usb_device_descriptor *desc);

/*-------------------------------------------------------------------------*/

typedef void (*fb_work_func_t)(struct usb_ep *ep, void *data);

int fastboot_queue_work(fb_work_func_t func, struct usb_ep *ep, void *data);

/*-------------------------------------------------------------------------*/

int fastboot_init(void);
void fastboot_shutdown(void);

#define FASTBOOT_EXIT_REBOOT			0x66557701
#define FASTBOOT_EXIT_REBOOT_BOOTLOADER	0x66557702
#define FASTBOOT_EXIT_SHUTDOWN			0x66557703
#define FASTBOOT_EXIT_CONTINUE			0x66557704

int fastboot_poll(void);

#endif /* _FASTBOOT_H */
