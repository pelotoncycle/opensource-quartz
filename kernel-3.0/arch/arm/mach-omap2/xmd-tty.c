/*
 * xmd-tty.c
 *
 * Copyright (C) 2011-2012 Intel Mobile Communications GmbH
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>

#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"

/* #define XMD_TTY_ENABLE_DEBUG_MSG */
#define XMD_TTY_ENABLE_ERR_MSG

#ifdef XMD_TTY_ENABLE_DEBUG_MSG
#define xmd_tty_dbg(fmt, args...)  printk("\nxmdtty: " fmt "\n", ## args)
#else
#define xmd_tty_dbg(fmt, args...)  do { } while (0)
#endif

#ifdef XMD_TTY_ENABLE_ERR_MSG
#define xmd_tty_err(fmt, args...)  printk("\nxmdtty: " fmt "\n", ## args)
#else
#define xmd_tty_err(fmt, args...)  do { } while (0)
#endif

static DEFINE_MUTEX(xmd_tty_lock);

static struct xmd_ch_info tty_channels[XMD_MAX_TTYS];

#define XMD_TTY_CORE_MAX_BUF_REQUEST (20*1024)

static void xmd_ch_tty_send_to_user(int chno, void *arg)
{
	struct tty_struct *tty = NULL;
	u8 *tbuf = NULL;
	int i;
	int data_pending = 1;
	int bytes_pending = 0;
	int inc_ptr = 0;
	struct hsi_ll_rx_tx_data *rx_buf = (struct hsi_ll_rx_tx_data*)arg;
	char *buf = rx_buf->ptr;
	int len = rx_buf->size;

#if defined(CONFIG_EN_HSI_EDLP)
	if((*((u32*)buf) & 0xFFFF0000) == HSI_LL_PDU_SIGNATURE) {
		len = (*((u32*)buf + 2) & 0x0001FFFF);
		buf = ((char*)(buf[4]+buf));
	}
#endif
#if defined(XMD_TTY_ENABLE_DEBUG_MSG)
	{
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		xmd_tty_dbg("CP => AP, data of size %d to ch %d, buf = %s",
					len,chno, str);
		kfree(str);
	}
#endif

	for (i=0; i < XMD_MAX_TTYS; i++) {
		if (tty_channels[i].chno == chno) {
			tty = (struct tty_struct *)tty_channels[i].priv;
			break;
		}
	}

	if (!tty) {
		xmd_tty_err("Invalid chno %d.", chno);
		return;
	}
	bytes_pending = len;
	while(data_pending) {
		if(bytes_pending > XMD_TTY_CORE_MAX_BUF_REQUEST) {
			len = XMD_TTY_CORE_MAX_BUF_REQUEST;
			bytes_pending -= XMD_TTY_CORE_MAX_BUF_REQUEST;
		} else {
			len = bytes_pending;
			data_pending = 0;
		}
		tty->low_latency = 1;
		tty_prepare_flip_string(tty, &tbuf, len);
		if (!tbuf) {
			xmd_tty_err("Mem not allocated by tty core to send to user space.");
			return;
		}
		memcpy((void *)tbuf, (void *)buf+inc_ptr, len);
		tty_flip_buffer_push(tty);
		tty_wakeup(tty);
		inc_ptr += len;
	}
}

static int xmd_ch_tty_open(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch;
	char init_flag = 0;

	int n = tty->index;

	if (n >= XMD_MAX_TTYS) {
		xmd_tty_err("Error opening channel %d.", n);
		return -ENODEV;
	}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	if (xmd_is_cp_in_recovery_state()) {
		xmd_tty_err("channel %d: CP is in recovery state.", n);
		return -EBUSY;
	}
#endif

	xmd_tty_dbg("Opening channel xmd-tty%d.",n);

	tty_ch = tty_channels + n;

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 0)
		init_flag = 1;

	tty_ch->open_count++;

	if(init_flag) {
		mutex_unlock(&xmd_tty_lock);
		xmd_tty_dbg("Channel already opened successfully %d.", tty_ch->chno);
		return 0;
	}

	tty_ch->chno = xmd_ch_open(tty_ch, xmd_ch_tty_send_to_user);
	if (0 > tty_ch->chno) {
		xmd_tty_err("Error opening channel xmd-tty%d",n);
		tty_ch->open_count = 0;
		tty_ch->chno = 0;
		mutex_unlock(&xmd_tty_lock);
		return -ENOMEM;
	}

	xmd_tty_dbg("Channel opened successfully %d.", tty_ch->chno);
	tty->driver_data = (void *)tty_ch;
	tty_ch->priv = (void*) tty;
	mutex_unlock(&xmd_tty_lock);

	return 0;
}

static void xmd_ch_tty_close(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch = (struct xmd_ch_info*)tty->driver_data;
	char cleanup_flag = 1;

	/* move following statement to after the pointer validatation check of tty_ch */
#if 0
	xmd_tty_dbg("Channel close function [ch %d].",tty_ch->chno);
#endif
	if (!tty_ch) {
		xmd_tty_dbg("Channel close function.");
		return;
	}

	xmd_tty_dbg("Channel close function [ch %d].",tty_ch->chno);

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 1)
		cleanup_flag = 0;

	tty_ch->open_count--;
	if (cleanup_flag) {
		xmd_ch_close(tty_ch->chno);
		tty->driver_data = NULL;
	}
	mutex_unlock(&xmd_tty_lock);
}

static int xmd_ch_tty_write(
	struct tty_struct *tty,
	const u8 *buf,
	int len)
{
	struct xmd_ch_info *tty_ch = NULL;
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	int ret;
#endif

	if(!tty->driver_data) {
		xmd_tty_dbg("Invalid param");
		return -ENODEV;
	}
	tty_ch = tty->driver_data;
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	{
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		xmd_tty_dbg("AP => CP, data of size %d to ch %d, data: %s",
					len,tty_ch->chno,str);
		kfree(str);
	}
#endif

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	ret = xmd_ch_write(tty_ch->chno, (void *)buf, len);
	if (ret < 0)
		len = ret;
#else
	xmd_ch_write(tty_ch->chno, (void *)buf, len);
#endif

	return len;
}

static int xmd_ch_tty_write_room(struct tty_struct *tty)
{
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	if (xmd_is_mcm_in_recovery_state())
		return 0;
#endif

	return 8192;
}

static int xmd_ch_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static void xmd_ch_tty_unthrottle(struct tty_struct *tty)
{
	return;
}

static struct tty_operations xmd_ch_tty_ops = {
	.open = xmd_ch_tty_open,
	.close = xmd_ch_tty_close,
	.write = xmd_ch_tty_write,
	.write_room = xmd_ch_tty_write_room,
	.chars_in_buffer = xmd_ch_tty_chars_in_buffer,
	.unthrottle = xmd_ch_tty_unthrottle,
};

static struct tty_driver *xmd_ch_tty_driver;

static int __init xmd_ch_tty_init(void)
{
	int ret, i;

	xmd_tty_dbg("xmd_ch_tty_init");
	xmd_ch_tty_driver = alloc_tty_driver(XMD_MAX_TTYS);
	if (xmd_ch_tty_driver == 0) {
		return -ENOMEM;
	}

	xmd_ch_tty_driver->owner = THIS_MODULE;
	xmd_ch_tty_driver->driver_name = "xmd_ch_tty_driver";
	xmd_ch_tty_driver->name = "xmd-tty"; /* "ttyspi"; "xmd-tty"; */
	xmd_ch_tty_driver->major = 0;
	xmd_ch_tty_driver->minor_start = 0;
	xmd_ch_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	xmd_ch_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	xmd_ch_tty_driver->init_termios = tty_std_termios;
	xmd_ch_tty_driver->init_termios.c_iflag = 0;
	xmd_ch_tty_driver->init_termios.c_oflag = 0;
	xmd_ch_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	xmd_ch_tty_driver->init_termios.c_lflag = 0;
	xmd_ch_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
								TTY_DRIVER_REAL_RAW 	|
								TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(xmd_ch_tty_driver, &xmd_ch_tty_ops);

	ret = tty_register_driver(xmd_ch_tty_driver);
	if (ret) {
		return ret;
	}

	for (i = 0; i < XMD_MAX_TTYS; i++){
		tty_channels[i].chno = 0;
		tty_channels[i].id = i;
		tty_channels[i].user = XMD_TTY;
		tty_channels[i].priv = NULL;
		tty_channels[i].open_count = 0;
		HSI_CHANNEL_NAME(tty_channels[i].name, (i+XMD_TTY_HSI_CH));
		tty_register_device(xmd_ch_tty_driver, tty_channels[i].id, 0);
	}

	return 0;
}

module_init(xmd_ch_tty_init);
