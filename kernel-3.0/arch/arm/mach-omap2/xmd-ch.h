/*
 * xmd-ch.h
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

#ifndef __XMD_CH_H__
#define __XMD_CH_H__

#ifdef CONFIG_MACH_OMAP_XMM_SPI
#ifdef CONFIG_MACH_OMAP_XMM_HSI
#error "Both XMD SPI DRIVER and XMD HSI DRIVER should not be enabled"
#endif
#endif

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#if defined (CONFIG_EN_HSI_EDLP)
#define XMD_MAX_TTYS 1
#define XMD_MAX_NET  3
#else
#define XMD_MAX_TTYS 12
#define XMD_MAX_NET  3
#endif

#define XMD_MAX_HSI_CH_IN_USE (XMD_MAX_TTYS + XMD_MAX_NET + 1)

#if defined (CONFIG_EN_HSI_EDLP)
#define XMD_RMNET_HSI_CH 2 /*Indicates Logical channel used by rmnet*/
#else
#define XMD_RMNET_HSI_CH 13
#endif

#if XMD_RMNET_HSI_CH > 1
#define XMD_TTY_HSI_CH 1
#else
#define XMD_TTY_HSI_CH (XMD_RMNET_HSI_CH+XMD_MAX_NET)
#endif

#define XMD_RIL_RECOVERY_CHANNEL 1
#define XMD_DLP_RECOVERY_ENHANCEMENT
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
#define XMD_RIL_FIRST_CHANNEL	1
#define XMD_RIL_LAST_CHANNEL	8
#endif

#define HSI_CHANNEL_NAME(str, ch) snprintf(str, 32, "%s %d", "CHANNEL", ch)

typedef enum _HSI_CH_USER_
{
	XMD_TTY,
	XMD_NET,
} XMD_CH_USER;

struct xmd_ch_info {
	int id;
	char name[32];
	int chno;
	XMD_CH_USER user;
	void *priv;
	int open_count;
};

struct hsi_board_info {
	char  name[32];
	const void *platform_data;
};

void xmd_hsi_register_board_info(
	struct hsi_board_info *board_info,
	int size);

void xmd_hsi_init(void);

void xmd_hsi_exit(void);

int xmd_boot_enable_fw_tty(void);

void xmd_boot_disable_fw_tty(void);

/* Release interrupt and set all the pins to safe */
int xmd_hsi_power_off(void);

/* Configure gpios and request irq */
void xmd_hsi_power_on_reset(void);

/* runtime gpio configuration like keeping wake line to high */
void xmd_hsi_board_init(void *hsi_platform_data);

void xmd_ch_init(void);

void xmd_ch_exit(void);

int xmd_ch_open(
	struct xmd_ch_info* xmd_ch,
	void (*notify_cb)(int chno, void *arg));

void xmd_ch_close(int chno);

/* Returns the buffer pointer in which recieved data is stored.
   should be called from notify_cb.
   memory will be freed after notify_cb returns. */
void *xmd_ch_read(int chno, int* len);

/* Data is copied in local buffer and then its transmitted.
   Freeing of *data is not done. */
int xmd_ch_write(int  chno, void *data, int len);

void xmd_ch_register_xmd_boot_cb(void (*fn)(void));

int wait_for_xmd_ack(void);

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
int xmd_board_power_off(void);
int xmd_board_power_on_reset(void);
void xmd_board_wait_for_cp_ready(void);
void xmd_board_notify_recovery_complete(void);
void xmd_dlp_recovery(void);
int xmd_is_cp_in_recovery_state(void);
int xmd_is_mcm_in_recovery_state(void);
#endif

#endif
