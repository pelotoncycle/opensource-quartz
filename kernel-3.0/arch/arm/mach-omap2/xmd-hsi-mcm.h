/*
 * xmd-hsi-mcm.h
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

#ifndef __HSI_MEM_MGR_H__
#define __HSI_MEM_MGR_H__

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"

#define XMD_QUEUE_ENHANCEMENT

#ifdef XMD_QUEUE_ENHANCEMENT
#define NUM_NET_X_BUF	160
#define NUM_X_BUF 20
#else
#define NUM_X_BUF 20
#endif

#define HSI_TRUE 1
#define HSI_FALSE 0

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ) || defined(CONFIG_EN_HSI_EDLP)
#define HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ
#endif

enum {
	HSI_MCM_STATE_UNDEF,
	HSI_MCM_STATE_INITIALIZED,
	HSI_MCM_STATE_ERR_RECOVERY,
};

typedef enum _HSI_CH_STATE_ {
	HSI_CH_FREE,
	HSI_CH_BUSY,
	HSI_CH_NOT_USED,
} HSI_CH_STATE;

struct hsi_chn {
	char name[32];
	HSI_CH_STATE state;
};

typedef enum {
	XMD_RX_Q,
	XMD_TX_Q,
} XQ_TYPE;

struct x_data {
	void *buf;
	unsigned int being_used;
	unsigned int size;
};

struct xq {
#ifdef XMD_QUEUE_ENHANCEMENT
	struct x_data *data;
	unsigned int num_x_data;
#else
	struct x_data data[NUM_X_BUF];
#endif
	unsigned char head;
	unsigned char tail;
};

struct hsi_channel {
	struct xq rx_q;
	struct x_data *curr;
	struct xq tx_q;
	HSI_CH_STATE state;
	struct xmd_ch_info *info;
	/*memory will be freed after completion of notify function. */
	void (*notify)(int chno, void *arg);
	void* (*read)(int chno, int* len);
	int (*write)(int chno, void *data, int len);
	char name[32];
	struct work_struct read_work;
#if !defined (CONFIG_EN_HSI_EDLP)
	struct work_struct write_work;
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
	unsigned int pending_rx_size;
	unsigned int rx_blocked;
	struct work_struct buf_alloc_work;
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	unsigned int pdu_size;
	void *dl_buf;
#endif
	wait_queue_head_t write_wait;
	int write_happening;
	int write_queued;
	wait_queue_head_t read_wait;
	int read_happening;
	int read_queued;
	int tx_blocked;
	int pending_rx_msgs;
	spinlock_t lock;
};

#endif /* __HSI_MEM_MGR_H__ */
