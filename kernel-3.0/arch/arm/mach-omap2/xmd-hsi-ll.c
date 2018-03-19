/*
 * xmd-hsi-ll.c
 *
 * HSI Link Layer
 *
 * Copyright (C) 2011-2012 Intel Mobile Communications GmbH
 *
 * Author: ThippaReddy <thippareddy.dammur@intel.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hsi_driver_if.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"
#include "xmd-hsi-ll-cfg.h"
#include "xmd-hsi-ll-internal.h"
#include "xmd_hsi_mem.h"

DEFINE_MUTEX(hsi_ll_open_mutex);
DEFINE_MUTEX(hsi_ll_close_mutex);
#if !defined (CONFIG_EN_HSI_EDLP)
DEFINE_MUTEX(hsi_ll_acwake);
#endif

static struct hsi_ll_data_struct hsi_ll_data;
static struct hsi_ll_if_struct   hsi_ll_if;
static hsi_ll_notify hsi_ll_cb;

static struct hsi_device_driver  hsi_ll_iface =
{
	.ctrl_mask  = ANY_HSI_CONTROLLER,
	.ch_mask[0] = 0,
	.probe      = hsi_ll_probe_cb,
	.remove     = hsi_ll_remove_cb,
	.driver     =
	{
		.name = "HSI_LINK_LAYER",
	},
};

#if defined (CONFIG_EN_HSI_EDLP)
const u32 hsi_ll_ul_buf_sz[XMD_MAX_HSI_CH_IN_USE] = {
	0,
	2048,
	3056,
	3056,
	3056,
};

/*Note: Should be <= HSI_LL_MAX_UL_BUFS*/
const u32 hsi_ll_no_of_ul_buf[XMD_MAX_HSI_CH_IN_USE] = {
	0,
	1,
	4,
	1,
	1,
};

/*Note: Should be <= HSI_LL_MAX_DL_BUFS*/
const u32 hsi_ll_no_of_dl_buf[XMD_MAX_HSI_CH_IN_USE] = {
	0,
	2,
	4,
	4,
	4,
};
#endif

static const char * channel_tx_state[] = {
	"HSI_LL_TX_STATE_UNDEF",
	"HSI_LL_TX_STATE_CLOSED",
	"HSI_LL_TX_STATE_IDLE",
	"HSI_LL_TX_STATE_OPEN_CONN",
	"HSI_LL_TX_STATE_TX",
	"HSI_LL_TX_STATE_TX_READY",
	"HSI_LL_TX_STATE_WAIT4_ECHO",
	"HSI_LL_TX_STATE_WAIT4_ACK",
	"HSI_LL_TX_STATE_WAIT4_CREDITS",
	"HSI_LL_TX_STATE_WAIT4_CREDITS_TO_TX",
	"HSI_LL_TX_STATE_C_ACK_CONF",
	"HSI_LL_TX_STATE_WAIT4_CONN_CLOSED",
	"HSI_LL_TX_STATE_WAIT4_TX_COMPLETE",
	"HSI_LL_TX_STATE_WAIT4_NEXT_TX",
	"HSI_LL_TX_STATE_WAIT4_CONF_ACK",
	"HSI_LL_TX_STATE_WAIT4_MEM",
};

static const char * channel_rx_state[] = {
	"HSI_LL_RX_STATE_UNDEF",
	"HSI_LL_RX_STATE_CLOSED",
	"HSI_LL_RX_STATE_IDLE",
	"HSI_LL_RX_STATE_POWER_DOWN",
	"HSI_LL_RX_STATE_BLOCKED",
	"HSI_LL_RX_STATE_SEND_ACK",
	"HSI_LL_RX_STATE_SEND_NACK",
	"HSI_LL_RX_STATE_RX",
	"HSI_LL_RX_STATE_WAIT4_RX_BUF",
	"HSI_LL_RX_STATE_CANCEL_CONN",
	"HSI_LL_RX_STATE_CANCEL_ACK",
	"HSI_LL_RX_STATE_WAIT4_CH_OPEN",
	"HSI_LL_RX_STATE_WAIT4_MEM",
	"HSI_LL_RX_STATE_WAIT4_CREDITS_ACK",
	"HSI_LL_RX_STATE_WAIT4_CANCEL_CONN_ACK",
};

static const char * if_state[] = {
	"HSI_LL_IF_STATE_UN_INIT",
	"HSI_LL_IF_STATE_READY",
	"HSI_LL_IF_STATE_CONFIG",
	"HSI_LL_IF_STATE_ERR_RECOVERY",
	"HSI_LL_IF_STATE_PERM_ERROR",
};

static int hsi_ll_send_cmd_queue(
	u32 cmd,
	u32 ch)
{
	if (hsi_ll_data.tx_cmd.cnt >= HSI_LL_MAX_CMD_Q_SIZE) {
#if 0
		hsi_ll_dbg("TX CMD Queue FULL.");
#else
		hsi_ll_info("TX CMD Queue FULL.");
#endif
		return -ENOSPC;
	} else {
		hsi_ll_data.tx_cmd.cnt++;
	}

	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.wr_idx].cmd = cmd;
	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.wr_idx].ch  = ch;

	hsi_ll_data.tx_cmd.wr_idx++;

	if (hsi_ll_data.tx_cmd.wr_idx >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.wr_idx = 0;
	}

	hsi_ll_if.msg_avaliable_flag = 1;
	wake_up_interruptible(&hsi_ll_if.msg_avaliable);

	return 0;
}

static int hsi_ll_read_cmd_queue(
	u32 *cmd,
	u32 *ch)
{
	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	if (hsi_ll_data.tx_cmd.cnt > 0) {
		hsi_ll_data.tx_cmd.cnt--;
	} else {
		spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
		return -1;
	}

	*cmd = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.rd_idx].cmd;
	*ch = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.rd_idx].ch;

	hsi_ll_data.tx_cmd.rd_idx++;

	if (hsi_ll_data.tx_cmd.rd_idx >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.rd_idx = 0;
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return 0;
}

static int hsi_ll_command_decode(
	u32* message,
	u32* ll_msg_type,
	u32* ch,
	u32* arg)
{
	int ret = 0;
	u32 msg = *message;

	*ll_msg_type = ((msg & 0xF0000000) >> 28);

	switch(*ll_msg_type) {
	case HSI_LL_MSG_BREAK:
		*ch = HSI_LL_INVALID_CHANNEL;
		break;
	case HSI_LL_MSG_ECHO:
		if (msg != 0x10ACAFFE) {
			*ch          = HSI_LL_INVALID_CHANNEL;
			*ll_msg_type = HSI_LL_MSG_INVALID;
			hsi_ll_err("Unexpected case. Received CMD = 0x%08X", msg);
		}
		break;
	case HSI_LL_MSG_CONF_CH:{
		*ch    = ((msg & 0x0F000000) >> 24);
		*arg = ((msg & 0x0000FF00) >> 8);
		}
		break;
	case HSI_LL_MSG_CONN_READY:
		*ch = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		*ch = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN:
		*ch  = ((msg & 0x0F000000) >> 24);
		*arg = ((msg & 0x00FF0000) >> 16);
		break;
	case HSI_LL_MSG_ACK:
		*ch = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_NAK:
		*ch = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONF_RATE:
		*ch  = ((msg & 0x0F000000) >> 24);
		*arg = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		*ch  = ((msg & 0x0F000000) >> 24);
		*arg = (msg & 0x0000FFFF);
		}
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET:
		*ch  = ((msg & 0x0F000000) >> 24);
		*arg = (msg & 0x00FFFFFF);
		if (*ch == 0)
			hsi_ll_err("Unexpected case. Received CMD = 0x%08X", msg);
		break;
	case HSI_LL_MSG_CREDITS:
		*ch  = ((msg & 0x0F000000) >> 24);
		*arg = ((msg & 0x00FF0000) >> 16);
		break;
	default: {
		*ll_msg_type = HSI_LL_MSG_INVALID;
		*ch = HSI_LL_INVALID_CHANNEL;
		ret = -1;
		hsi_ll_err("Unexpected case. Received CMD = 0x%08X", msg);
		}
		break;
	}

	return ret;
}

static int hsi_ll_send_command(
	int cmd_type,
	u32 ch,
	void* arg)
{
	u32 cmd = 0;
	int ret = 0;

	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	switch(cmd_type) {
	case HSI_LL_MSG_BREAK:
		cmd = 0;
		break;
	case HSI_LL_MSG_ECHO:
		cmd = 0x10ACAFFE;
		break;
	case HSI_LL_MSG_CONN_READY:
		cmd = ((HSI_LL_MSG_CONN_READY & 0x0000000F) << 28) |
			  ((ch                    & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		cmd = ((HSI_LL_MSG_CONN_CLOSED & 0x0000000F) << 28) |
			   ((ch                    & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN: {
		u32 dirn = *(u32*)arg;

		cmd = ((HSI_LL_MSG_CANCEL_CONN & 0x0000000F) << 28) |
			  ((ch                     & 0x0000000F) << 24) |
			  ((dirn                   & 0x000000FF) << 16);
		}
		break;
	case HSI_LL_MSG_ACK: {
		u32 echo_params = *(u32*)arg;

		cmd = ((HSI_LL_MSG_ACK & 0x0000000F) << 28) |
			  ((ch             & 0x0000000F) << 24) |
			  ((echo_params    & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_NAK:
		cmd = ((HSI_LL_MSG_NAK & 0x0000000F) << 28) |
			  ((ch             & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONF_RATE: {
		u32 baud_rate = *(u32*)arg;

		cmd = ((HSI_LL_MSG_CONF_RATE & 0x0000000F) << 28) |
			  ((ch                   & 0x0000000F) << 24) |
			  ((baud_rate            & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		u32 size = *(u32*)arg;
		cmd = ((HSI_LL_MSG_OPEN_CONN & 0x0000000F) << 28) |
			  ((ch                   & 0x0000000F) << 24) |
			   (size                 & 0x0000FFFF);
		}
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		u32 size = *(u32*)arg;

		cmd = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F) << 28) |
			  ((ch                         & 0x0000000F) << 24) |
			  ((size                       & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_CREDITS: {
		u32 credits = *(u32*)arg;
		cmd = (((HSI_LL_MSG_CREDITS & 0x0000000F) << 28) |
			  ((ch                  & 0x0000000F) << 24) |
			  ((credits             & 0x000000FF) << 16));
		}
		break;
	default:
		ret = -1;
		break;
	}

	if (ret == 0) {
		if (0 > (ret = hsi_ll_send_cmd_queue(cmd, ch)))
			hsi_ll_err("hsi_ll_send_cmd_queue fail for Channel %d.", ch);
	} else {
		hsi_ll_err("Invalid command issued for Channel %d.", ch);
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return ret;
}

#if defined (CONFIG_EN_HSI_EDLP)
static void hsi_ll_dl_work(struct work_struct *work)
{
	struct hsi_ll_rx_ch *rx_ch= (struct hsi_ll_rx_ch*) container_of(work,
											struct hsi_ll_rx_ch, dl_work);
	u32 ch = rx_ch->channel;
	u32 *rd_idx = &hsi_ll_data.ch[ch].rx.rd_idx;
	u32 *wr_idx = &hsi_ll_data.ch[ch].rx.wr_idx;
	u32 cont_dl_work = FALSE;
	struct hsi_ll_rx_tx_data dl_buf;
	u32 no_of_bufs = hsi_ll_no_of_dl_buf[ch];

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		hsi_ll_data.ch[ch].rx.wq_active = FALSE;
		return;
	}
	if(hsi_ll_data.ch[ch].rx.wq_init) {
		return;
	}
	hsi_ll_data.ch[ch].rx.wq_init = 1;

	do {
		if(cont_dl_work == FALSE) {
			wait_event (hsi_ll_data.ch[ch].rx.wq_notify,
						hsi_ll_data.ch[ch].rx.wq_active == TRUE);
		}
		if(hsi_ll_data.ch[ch].rx.dl_buf[*rd_idx].state == BUF_READ) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			{
				u32 sign    = (*((u32*)dl_buf.ptr) & 0xFFFF0000) >> 16;
				u32 seq_num = (*((u32*)dl_buf.ptr) & 0x0000FFFF);
				hsi_ll_dbg ("DL packet for ch %d. "
							"Signature = 0x%04X, Sequence Number = 0x%04X.",
							ch, sign, seq_num);
			}
#endif
			dl_buf.ptr  = hsi_ll_data.ch[ch].rx.dl_buf[*rd_idx].ptr;
			dl_buf.size = hsi_ll_data.ch[ch].rx.pdu_size << 2;
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_DL_PACKET,
					  &dl_buf); /*Note: PDU will be decoded in rmnet/tty layer. */

			hsi_ll_data.ch[ch].rx.dl_buf[*rd_idx].state = BUF_FREE;
			(*rd_idx)++;
			if (*rd_idx >= no_of_bufs) {
				*rd_idx = 0;
			}
			cont_dl_work = TRUE;
			continue;
		} else {
			hsi_ll_data.ch[ch].rx.wq_active = cont_dl_work = FALSE;
			if(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_WAIT4_RX_BUF) {
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_RX;
				hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state = BUF_RX;
				if(0 > hsi_read(hsi_ll_data.dev[ch],
								hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].ptr,
								hsi_ll_data.ch[ch].rx.pdu_size)) {
					hsi_ll_err("hsi_read err for ch %d", ch);
				}
			}
		}
	} while(1);
}
#endif

/* Read callback for channel 1 to 15 */
static void hsi_ll_read_cb(struct hsi_device *dev, u32 size)
{
	u32 ch = dev->n_ch;
	/* Data Rx callback*/
	if ((hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_RX)          ||
		(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_IDLE)        ||
		(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_CANCEL_CONN)) {
		struct hsi_ll_rx_tx_data dl_buf;
		if (hsi_ll_data.ch[ch].rx.close_req == TRUE) {
			u32 dirn = HSI_LL_DIRN_RX;
			hsi_ll_dbg("Read close requested for ch %d.", ch);

			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								ch,
								&dirn);

			dl_buf.ptr = hsi_ll_data.ch[ch].rx.ptr;
			dl_buf.size   = hsi_ll_data.ch[ch].rx.size;

			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_FREE_MEM,
					  &dl_buf);
			hsi_ll_data.ch[ch].rx.ptr    = NULL;
			hsi_ll_data.ch[ch].rx.close_req = FALSE;
			hsi_ll_data.ch[ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
		} else {
#if defined (CONFIG_EN_HSI_EDLP)
		if ((hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_PACKET) ||
			(hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_RAW)) {
				u32 *wr_idx = &hsi_ll_data.ch[ch].rx.wr_idx;

				hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state = BUF_READ;
				(*wr_idx)++;

			if(*wr_idx >= hsi_ll_no_of_dl_buf[ch]) {
				*wr_idx = 0;
			}
			if (hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state == BUF_FREE) {
				hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state = BUF_RX;
				if(0 > hsi_read(hsi_ll_data.dev[ch],
								hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].ptr,
								hsi_ll_data.ch[ch].rx.pdu_size))
					hsi_ll_err("hsi_read failed for %d", ch);
			} else {
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_WAIT4_RX_BUF;
			}
			if(!hsi_ll_data.ch[ch].rx.wq_active) {
				hsi_ll_data.ch[ch].rx.wq_active = TRUE;
				wake_up(&hsi_ll_data.ch[ch].rx.wq_notify);
			}
			return;
		}
#endif
			dl_buf.ptr = hsi_ll_data.ch[ch].rx.ptr;
			dl_buf.size = hsi_ll_data.ch[ch].rx.size;

			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_READ_COMPLETE,
					  &dl_buf);

			hsi_ll_data.ch[ch].rx.ptr = NULL;
			hsi_ll_send_command(HSI_LL_MSG_CONN_CLOSED,
								ch,
								NULL);
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_IDLE;
		}
	} else {
#if 0
		hsi_ll_err("Error. Invalid state for channel %d.", ch);
#else
		hsi_ll_err("Error. Invalid state %d(%s) for channel %d.",
					hsi_ll_data.ch[ch].rx.state,
					channel_rx_state[hsi_ll_data.ch[ch].rx.state], ch);
#endif
	}
}

/* Read callback for control channel */
static void hsi_ll_read_complete_cb(struct hsi_device *dev, u32 size)
{
	u32 ch = 0, param = 0, ll_msg_type = 0;
	int ret = hsi_ll_command_decode(&hsi_ll_data.rx_cmd,
									&ll_msg_type,
									&ch,
									&param);

#if 0
	hsi_ll_info("CP => AP CMD = 0x%08X.", hsi_ll_data.rx_cmd);
#else
	hsi_ll_dbg("CP => AP CMD = 0x%08X.", hsi_ll_data.rx_cmd);
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	if(hsi_ll_if.ping_modem_once)
		hsi_ll_if.ping_modem_once = 0;
#endif
	if (hsi_ll_if.rd_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.rd_complete_flag = 1;
		wake_up_interruptible(&hsi_ll_if.rd_complete);
	} else {
		if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
			hsi_ll_dbg("Recovery in progress so LL will drop current CP CMD.");
		} else {
			hsi_ll_err("Spurious Callback ???.");
		}
		return;
	}

	if (ret != 0) {
		hsi_ll_send_command(HSI_LL_MSG_NAK, ch, NULL);
		hsi_ll_err("Received Invalid MSG %d from CP"
					" for channel %d.", ll_msg_type, ch);
		return;
	}

	switch(ll_msg_type) {
	case HSI_LL_MSG_BREAK: {
		hsi_ll_err("Break received.");
		}
		break;
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_MSG_ECHO: {
		hsi_ll_if.echo_flag = 1;
		wake_up_interruptible(&hsi_ll_if.wait_4_echo);
		}
		break;
#endif
	case HSI_LL_MSG_CONN_CLOSED: {
		switch(hsi_ll_data.ch[ch].tx.state) {
		case HSI_LL_TX_STATE_TX:
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_TX_COMPLETE;
			break;
		case HSI_LL_TX_STATE_WAIT4_CONN_CLOSED: {
			struct hsi_ll_rx_tx_data temp;
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
			temp.ptr = hsi_ll_data.ch[ch].tx.ptr;
			temp.size   = hsi_ll_data.ch[ch].tx.size;
			hsi_ll_data.ch[ch].tx.ptr = NULL;
			hsi_ll_dbg("Data transfer over channel %d completed.", ch);
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
			}
			break;
		default:
			hsi_ll_err("Invalid State.");
			break;
		}
		}
		break;
	case HSI_LL_MSG_CANCEL_CONN:{
#if defined (CONFIG_EN_HSI_EDLP)
		if(param == 0) { /*TX*/
		} else if (param == 1) { /*RX*/
			if(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_RX) {
				hsi_ll_data.ch[ch].rx.close_req = TRUE;
				hsi_read_cancel(hsi_ll_data.dev[ch]);
			}
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_CANCEL_ACK;
		} else {
			hsi_ll_dbg("Ch = %d, CANCEL_CONN not expected for both"
						" directions.", ch);
		}
#endif
		}
		break;
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_MSG_CREDITS: {
		switch(hsi_ll_data.ch[ch].tx.state) {
		case HSI_LL_TX_STATE_IDLE:
		case HSI_LL_TX_STATE_TX:
		case HSI_LL_TX_STATE_WAIT4_CREDITS: {
			hsi_ll_data.ch[ch].tx.credits+= param;
			if(!hsi_ll_data.ch[ch].tx.credits) {
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CREDITS;
				hsi_ll_err("Error.Invalid credits %d updated by Modem", param);
				break;
			}
			if (hsi_ll_data.ch[ch].ip_ch == TRUE) {
				volatile u32 *wr_idx = &hsi_ll_data.ch[ch].tx.wr_idx;
				spin_lock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
				if (hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state == TX_READY) {
					void *ul_buf = hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].ptr;
					hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_BUSY;
					spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
					hsi_ll_data.ch[ch].tx.credits--;
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
					HSI_LL_SET_AC_WAKE(ch)
					if(0 > hsi_write(hsi_ll_data.dev[ch],
									 ul_buf,
									 hsi_ll_data.ch[ch].tx.pdu_size)) {
						hsi_ll_err("hsi_write err for ch %d", ch);
						hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_FREE;
						hsi_ll_data.ch[ch].tx.wq_active = FALSE;
						hsi_ll_data.ch[ch].tx.credits++;
					}
				} else {
					spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
					if(hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state == TX_BUSY) {
						hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
					} else {
						hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
					}
				}
				if (!hsi_ll_data.ch[ch].tx.wq_init){
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
					PREPARE_WORK(&hsi_ll_data.ch[ch].tx.ul_work,
								 hsi_ll_ul_work);
					queue_work(hsi_ll_if.ul_wq[ch],
							  &hsi_ll_data.ch[ch].tx.ul_work);
				}
				if (!hsi_ll_data.ch[ch].tx.wq_active) {
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
					hsi_ll_data.ch[ch].tx.wq_active = TRUE;
					wake_up(&hsi_ll_data.ch[ch].tx.wq_notify);
				} else {
					volatile u32 *wr_idx = &hsi_ll_data.ch[ch].tx.wr_idx;
					if (hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state == TX_BUSY)
						hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
					else
						hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
				}
			} else {
				if (hsi_ll_data.ch[ch].tx.is_first_time == 1){
						hsi_ll_data.ch[ch].tx.is_first_time = 0;
				}
				hsi_ll_data.ch[ch].tx.credits_flag = TRUE;
				wake_up_interruptible(&hsi_ll_data.ch[ch].tx.ev_credits);
			}

			}
			break;
		default:
#if 0
			hsi_ll_err("Error.Invalid state. Current channel %d state %d.",
						ch,hsi_ll_data.ch[ch].rx.state);
#else
			hsi_ll_err("Error.Invalid state. Current channel %d state %d(%s).",
						ch, hsi_ll_data.ch[ch].rx.state,
						channel_rx_state[hsi_ll_data.ch[ch].rx.state]);
#endif
			break;
		}
		}
		break;
#endif
	case HSI_LL_MSG_ACK: {
		switch(hsi_ll_data.ch[ch].tx.state) {
#if defined (CONFIG_EN_HSI_EDLP)
		case HSI_LL_TX_STATE_WAIT4_ACK: {
			if ((hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_PACKET) ||
				(hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_RAW)){
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CREDITS;
			} else {
			/*Unexpected state*/
				hsi_ll_err("Unexpected state %d for ch %d.",
							hsi_ll_data.ch[ch].tx.state, ch);
			}
			}
			break;
#endif
		case HSI_LL_TX_STATE_OPEN_CONN: {
			int ret;
			u32 size = hsi_ll_data.ch[ch].tx.size;
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
			HSI_LL_GET_SIZE_IN_WORDS(size);
			ret = hsi_write(hsi_ll_data.dev[ch],
							hsi_ll_data.ch[ch].tx.ptr,
							size);

			if (ret!=0) {
				hsi_ll_dbg("hsi_write failed for ch %d"
							" with error %d.", ch, ret);
			}
		}
		break;
		case HSI_LL_TX_STATE_CLOSED: { /* ACK as response to CANCEL_CONN */
			if (hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_WAIT4_CANCEL_CONN_ACK) {
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_CLOSED;
			} else {
#if 0
				hsi_ll_err("Error.");
#else
				hsi_ll_err("Error.Invalid state. Current channel %d state %d(%s).",
							ch, hsi_ll_data.ch[ch].rx.state,
							channel_rx_state[hsi_ll_data.ch[ch].rx.state]);
#endif
			}
			}
			break;
		 /* ACK as response to CONF_RATE */
		case HSI_LL_TX_STATE_WAIT4_CONF_ACK:
			break;
		default:
#if 0
			hsi_ll_err("Error.Invalid state. Current channel %d state %d.",
						ch,hsi_ll_data.ch[ch].rx.state);
#else
			hsi_ll_err("Error.Invalid state. Current channel %d state %d(%s).",
						ch, hsi_ll_data.ch[ch].rx.state,
						channel_rx_state[hsi_ll_data.ch[ch].rx.state]);
#endif
			break;
		}
		}
		break;
	case HSI_LL_MSG_NAK:{
		hsi_ll_dbg("NAK received for ch %d state %d. retry %d.",
					ch, hsi_ll_data.ch[ch].tx.state,
					hsi_ll_data.ch[ch].tx.retry);
		switch(hsi_ll_data.ch[ch].tx.state) {
			case HSI_LL_TX_STATE_OPEN_CONN: {
				hsi_ll_data.ch[ch].tx.retry++;

				if (hsi_ll_data.ch[ch].tx.retry == HSI_LL_MAX_OPEN_CONN_RETRIES) {
					struct hsi_ll_rx_tx_data temp;
					temp.ptr = hsi_ll_data.ch[ch].tx.ptr;
					temp.size   = hsi_ll_data.ch[ch].tx.size;
					hsi_ll_data.ch[ch].tx.state  = HSI_LL_TX_STATE_IDLE;
					hsi_ll_data.ch[ch].tx.ptr = NULL;
					hsi_ll_err("%d retry attempts failed, "
								"droping packet for channel %d.",
							hsi_ll_data.ch[ch].tx.retry, ch);
					hsi_ll_cb(ch,
							  -1,
							  HSI_LL_EV_WRITE_COMPLETE,
							 &temp);
				} else {
#if !defined (HSI_LL_ENABLE_TX_RETRY_WQ)
					hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
										ch,
										&hsi_ll_data.ch[ch].tx.size);
#else
					/* In NAK case Modem needs some delay  */
					PREPARE_WORK(&hsi_ll_data.ch[ch].tx.retry_work,
								  hsi_ll_retry_work);
					queue_work(hsi_ll_if.tx_retry_wq,
							  &hsi_ll_data.ch[ch].tx.retry_work);
#endif
					}
				}
				break;
			/* NAK as response to CONF_RATE */
			case HSI_LL_TX_STATE_WAIT4_CONF_ACK: {
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
				}
				break;
			default:
				hsi_ll_err("Error. Invalid state. Current channel %d state %d.",
				ch, hsi_ll_data.ch[ch].tx.state);
				break;
		}
		}
		break;
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_MSG_OPEN_CONN: {
		switch(hsi_ll_data.ch[ch].rx.state) {
		case HSI_LL_RX_STATE_IDLE: {
			if (hsi_ll_data.ch[ch].rx.dl_buf[0].ptr != NULL) {
				return;
			}
			hsi_ll_data.ch[ch].rx.pdu_size = param;
			hsi_ll_data.ch[ch].rx.dl_buf[0].size = (param << 2);
			hsi_ll_data.ch[ch].rx.dl_buf[0].state = 0;
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_ALLOC_MEM_BULK,
					  hsi_ll_data.ch[ch].rx.dl_buf);
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_WAIT4_MEM;
			return;
			}
			break;
		default:
#if 0
			hsi_ll_err("Unexpected state %d for ch %d.",
						hsi_ll_data.ch[ch].rx.state, ch);
#else
			hsi_ll_err("Unexpected state %d(%s) for ch %d.",
						hsi_ll_data.ch[ch].rx.state,
						channel_rx_state[hsi_ll_data.ch[ch].rx.state], ch);
#endif
			break;
		}
		}
		break;
#endif
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		switch(hsi_ll_data.ch[ch].rx.state) {
			case HSI_LL_RX_STATE_IDLE: {
			struct hsi_ll_rx_tx_data dl_buf;

			dl_buf.ptr  = NULL;
			dl_buf.size = hsi_ll_data.ch[ch].rx.size = param;

			/*Make size an multiple of word as data is always
			read in multiple of words */
			HSI_LL_TO_MUL_OF_WORD(dl_buf.size)

			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_ALLOC_MEM,
					  &dl_buf);

			if (dl_buf.ptr == NULL) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
				hsi_ll_dbg("Could not get mem (size=%d)for channel "
							"%d, so currently blocking this ch.",
							dl_buf.size, ch);
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_BLOCKED;
				return;
#else
				hsi_ll_send_command(HSI_LL_MSG_NAK,
									ch,
									NULL);
#endif
			} else {
				u32 size = hsi_ll_data.ch[ch].rx.size;
				hsi_ll_data.ch[ch].rx.ptr = dl_buf.ptr;

				hsi_ll_send_command(HSI_LL_MSG_ACK,
									ch,
									&size);

				HSI_LL_GET_SIZE_IN_WORDS(size);

				hsi_read(hsi_ll_data.dev[ch], dl_buf.ptr, size);
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_RX;
			}
			}
			break;
		case HSI_LL_RX_STATE_BLOCKED: {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			hsi_ll_send_command(HSI_LL_MSG_NAK,
								ch,
								NULL);
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_SEND_NACK;
#endif
			}
			break;
		default:
#if 0
			hsi_ll_err("Error.Invalid state. Current channel %d state %d.",
						ch,hsi_ll_data.ch[ch].rx.state);
#else
			hsi_ll_err("Error.Invalid state. Current channel %d state %d(%s).",
						ch,hsi_ll_data.ch[ch].rx.state,
						channel_rx_state[hsi_ll_data.ch[ch].rx.state]);
#endif
			break;
		}
		}
		break;
	default:
		hsi_ll_err("Error.Invalid message encountered for channel %d.", ch);
		break;
	}
}

/* TX Retry Work Queue */
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
static void hsi_ll_retry_work(struct work_struct *work)
{
	struct hsi_ll_tx_ch *tx_ch= (struct hsi_ll_tx_ch*) container_of(work,
											struct hsi_ll_tx_ch,retry_work);
	u32 ch = tx_ch->channel;

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		return;
	}

	hsi_ll_dbg("Retrying Open_connect for ch %d. Size %d. Retry count %d.",
				ch, hsi_ll_data.ch[ch].tx.size,
				hsi_ll_data.ch[ch].tx.retry);
	udelay(100); /*If required fine tune this delay.*/
	hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
						ch,
						&hsi_ll_data.ch[ch].tx.size);
	return;
}
#endif

#if defined (CONFIG_EN_HSI_EDLP)
static void hsi_ll_ul_work(struct work_struct *work)
{
	struct hsi_ll_tx_ch *tx_ch= (struct hsi_ll_tx_ch*) container_of(work,
											struct hsi_ll_tx_ch, ul_work);
	u32 ch = tx_ch->channel;
	u32 cont_ul_work = FALSE;
	u32 tot_sz  = 0;
	u32 base_idx  = 0;
	u32 hdr_idx  = 0;
	u32 no_of_packets = 0;
	u32 idx = 0;
	void *ul_buf = NULL;
	void *ptr = NULL;
	u32 pckt_sz = 0;
	volatile u32 *cpy_idx = &hsi_ll_data.ch[ch].tx.cpy_idx;
	volatile u32 *wr_idx = &hsi_ll_data.ch[ch].tx.wr_idx;
	struct sk_buff *skb;
	struct hsi_ll_ul_bufs ul_pckts[16];

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		if ((hsi_ll_data.ch[ch].tx.pending) &&
			(hsi_ll_data.ch[ch].tx.ptr)) {
			dev_kfree_skb_any(hsi_ll_data.ch[ch].tx.ptr);
			hsi_ll_data.ch[ch].tx.ptr = NULL;
		}
		hsi_ll_data.ch[ch].tx.wq_active = FALSE;
		return;
	}

	if(hsi_ll_data.ch[ch].tx.wq_init) {
		hsi_ll_dbg("UL_WQ already initilized for ch %d", ch);
		return;
	}
	hsi_ll_dbg("UL_WQ init for CH %d", ch);
	hsi_ll_data.ch[ch].tx.wq_init = 1;

	do {
		if(cont_ul_work == FALSE) {
			wait_event (hsi_ll_data.ch[ch].tx.wq_notify,
						hsi_ll_data.ch[ch].tx.wq_active == TRUE);
		}
		spin_lock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
		if(hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state == TX_READY) {
			if (!hsi_ll_data.ch[ch].tx.credits)
			{
				spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CREDITS;
				hsi_ll_data.ch[ch].tx.credits_flag = FALSE;
				HSI_LL_RESET_AC_WAKE(ch)
			} else {
				hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_BUSY;
				spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
				ul_buf = hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].ptr;
				hsi_ll_data.ch[ch].tx.credits--;
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
				HSI_LL_SET_AC_WAKE(ch)
				if(0 > hsi_write(hsi_ll_data.dev[ch],
								 ul_buf,
								 hsi_ll_data.ch[ch].tx.pdu_size)) {
					hsi_ll_err("hsi_write err for ch %d", ch);
					hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_FREE;
					cont_ul_work = hsi_ll_data.ch[ch].tx.wq_active = FALSE;
					hsi_ll_data.ch[ch].tx.credits++;
					continue;
				}
			}
		} else {
			spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
		}

		if (hsi_ll_data.ch[ch].tx.ul_buf[*cpy_idx].state == TX_FREE) {
			tot_sz = 0;
			no_of_packets = 0;
			for(idx = 0; idx < 16; idx++) {
				if (hsi_ll_data.ch[ch].tx.pending) {
					ptr = hsi_ll_data.ch[ch].tx.ptr;
					pckt_sz = hsi_ll_data.ch[ch].tx.size;
					hsi_ll_data.ch[ch].tx.pending = FALSE;
					no_of_packets++;
				} else {
					hsi_ll_data.ch[ch].tx.read_ip_pckt(ch, &pckt_sz, &ptr);
					if(pckt_sz == 0)
						break;
					no_of_packets++;
				}
				tot_sz += (pckt_sz + 2 + 16); /*Plus Header+2 for DMA alignment*/
				HSI_LL_TO_MUL_OF_QWORD(tot_sz); /*Aligned size*/
				ul_pckts[idx].ptr  = ptr;
				ul_pckts[idx].size = pckt_sz;
				if((tot_sz + (no_of_packets << 3) + 8) > hsi_ll_ul_buf_sz[ch]) {
					/*Can't accommodate this IP in current PDU ? Use it in next PDU*/
					hsi_ll_data.ch[ch].tx.ptr = ul_pckts[idx].ptr;
					hsi_ll_data.ch[ch].tx.size = ul_pckts[idx].size;
					hsi_ll_data.ch[ch].tx.pending = TRUE;
					no_of_packets--;
					break;
				}
			}
			if(!tot_sz) {
				cont_ul_work = hsi_ll_data.ch[ch].tx.wq_active = FALSE;
				continue;
			}
			hsi_ll_data.ch[ch].tx.ul_buf[*cpy_idx].state = TX_PREPARE;
			ul_buf = hsi_ll_data.ch[ch].tx.ul_buf[*cpy_idx].ptr;

			/*Frame PDU*/
			*((u32*)ul_buf) = (HSI_LL_PDU_SIGNATURE |
							(hsi_ll_data.ch[ch].tx.seq_num & 0x0000FFFF)); /*Signature+Seq Number*/
			hsi_ll_data.ch[ch].tx.seq_num++;

			idx = 0;
			hdr_idx  = 1;
			base_idx = (no_of_packets << 3);
			HSI_LL_TO_MUL_OF_QWORD(base_idx)
			while(no_of_packets) {
				*((u32*)ul_buf + hdr_idx)     = base_idx;
				*((u32*)ul_buf + hdr_idx + 1) = ((ul_pckts[idx].size + 2) |
									((no_of_packets == 1) ? 0x60000000:0xE0000000));
				skb = ul_pckts[idx].ptr;
				memcpy((ul_buf + base_idx + 16),
						(void *)((char *) skb->data + 14), /*Skip eth header*/
						(ul_pckts[idx].size - 14));
				dev_kfree_skb_any(skb);
				base_idx += (ul_pckts[idx].size + 2 + 16);
				HSI_LL_TO_MUL_OF_QWORD(base_idx);
				idx++;
				hdr_idx += 2;
				no_of_packets--;
			}
			hsi_ll_data.ch[ch].tx.ul_buf[*cpy_idx].state = TX_READY;
			(*cpy_idx)++;
			if (*cpy_idx >= hsi_ll_no_of_ul_buf[ch])
				*cpy_idx = 0;
			cont_ul_work = TRUE;
		} else if (hsi_ll_data.ch[ch].tx.ul_buf[*cpy_idx].state == TX_BUSY) {
			if(hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state != TX_READY)
				cont_ul_work = hsi_ll_data.ch[ch].tx.wq_active = FALSE;
			continue;
		}
	} while(1);
}
#endif

/* Write callback for channels 1 to 15 */
static void hsi_ll_write_cb(struct hsi_device *dev, u32 size)
{
	u32 ch = dev->n_ch;
	hsi_ll_dbg("Write complete for channel %d.", ch);
	switch(hsi_ll_data.ch[ch].tx.state) {
	case HSI_LL_TX_STATE_TX:
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_TX_STATE_WAIT4_CREDITS:
	case HSI_LL_TX_STATE_IDLE:
		if ((hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_PACKET) ||
			(hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_RAW)) {
			if (hsi_ll_data.ch[ch].ip_ch) {
				volatile u32 *wr_idx = &hsi_ll_data.ch[ch].tx.wr_idx;

				hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_FREE;
				(*wr_idx)++;
				if(*wr_idx >= hsi_ll_no_of_ul_buf[ch])
					*wr_idx = 0;

				spin_lock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
				if (hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state == TX_READY) {
					if(!hsi_ll_data.ch[ch].tx.credits) {
						spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
						hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CREDITS;
						HSI_LL_RESET_AC_WAKE(ch)
						break;
					}
					hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].state = TX_BUSY;
					spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
					hsi_ll_data.ch[ch].tx.credits--;
					if(0 > hsi_write(hsi_ll_data.dev[ch],
							hsi_ll_data.ch[ch].tx.ul_buf[*wr_idx].ptr,
							hsi_ll_data.ch[ch].tx.pdu_size)) {
						hsi_ll_err("write failed for ch %d", ch);
						hsi_ll_data.ch[ch].tx.credits++;
					}
				} else {
					spin_unlock_bh(&hsi_ll_data.ch[ch].tx.ul_lock);
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
					HSI_LL_RESET_AC_WAKE(ch)
					hsi_ll_data.ch[ch].tx.wq_active = TRUE;
					wake_up(&hsi_ll_data.ch[ch].tx.wq_notify);
				}
			} else {
				struct hsi_ll_rx_tx_data temp;
				temp.ptr = NULL;
				HSI_LL_RESET_AC_WAKE(ch)
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
				hsi_ll_dbg("Data transfer over channel %d completed.", ch);
				hsi_ll_cb(ch, 0,
						  HSI_LL_EV_WRITE_COMPLETE,
						  &temp);
			}
			break;
		}
#endif
		hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CONN_CLOSED;
		break;
	case HSI_LL_TX_STATE_WAIT4_TX_COMPLETE: {
		struct hsi_ll_rx_tx_data temp;

		hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
		temp.ptr = hsi_ll_data.ch[ch].tx.ptr;
		temp.size   = hsi_ll_data.ch[ch].tx.size;
		hsi_ll_data.ch[ch].tx.ptr = NULL;
		hsi_ll_dbg("Data transfer over channel %d completed.", ch);
		hsi_ll_cb(ch, 0,
				  HSI_LL_EV_WRITE_COMPLETE,
				 &temp);
		}
		break;
	default:
#if 0
		hsi_ll_err("Error.Invalid state for channel %d.", ch);
#else
		hsi_ll_err("Error.Invalid state %d(%s)for channel %d.",
					hsi_ll_data.ch[ch].tx.state,
					channel_tx_state[hsi_ll_data.ch[ch].tx.state], ch);
#endif
		break;
	}
}

/* Write callback for control channel */
static void hsi_ll_write_complete_cb(struct hsi_device *dev, u32 size)
{
	hsi_ll_dbg("Write Complete callback.");

	if (hsi_ll_if.wr_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.wr_complete_flag = 1;
#if !defined (CONFIG_EN_HSI_EDLP) && defined (HSI_LL_ENABLE_PM)
		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_IDLE;
#endif
		wake_up_interruptible(&hsi_ll_if.wr_complete);
	}
}

static int hsi_ll_create_rx_thread(void)
{
	hsi_ll_if.rd_th = kthread_run(hsi_ll_rd_ctrl_ch_th,
								  NULL,
								 "hsi_ll_rd_ctrlch");

	if (IS_ERR(hsi_ll_if.rd_th)) {
		hsi_ll_err("Cannot create Read CTRL Thread.");
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return -1;
	}

	return 0;
}

static int hsi_ll_config_bus(void)
{
	int ret = 0;

	hsi_ll_data.rx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.rx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.rx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.rx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.rx_cfg.ctx.counters   = HSI_LL_COUNTERS_VALUE;
	hsi_ll_data.rx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_RX,
				    &hsi_ll_data.rx_cfg.ctx);

	if (ret) {
		hsi_ll_err("RX config error %d.", ret);
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return ret;
	}

	hsi_ll_data.tx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.tx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.tx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.tx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.tx_cfg.ctx.arb_mode   = HSI_LL_ARBMODE_MODE;
	hsi_ll_data.tx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_TX,
					&hsi_ll_data.tx_cfg.ctx);

	if (ret) {
		hsi_ll_err("TX config error %d.", ret);
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
	}
#if 0 /*Required for XMM7160*/
	{
		u32 state;
		ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						HSI_IOCTL_SET_HI_SPEED,
						&state);
		if (ret) {
			hsi_ll_err("Err setting HSI_IOCTL_SET_HI_SPEED,err = %d.", ret);
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		}
	}
#endif
	return ret;
}

static int hsi_ll_wr_ctrl_ch_th(void *data)
{
	int ret;
#if !defined (CONFIG_EN_HSI_EDLP)
	int i;
#endif
	wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1);

	hsi_ll_dbg("Write thread Started.");
	ret = hsi_ll_open(HSI_LL_CTRL_CHANNEL);

	if (ret) {
		hsi_ll_err("Error while opening CTRL channel %d.", ret);
		hsi_ll_data.initialized = FALSE;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;

		return ret;
	}

#if !defined (CONFIG_EN_HSI_EDLP)
	for(i=1; i < HSI_LL_MAX_CHANNELS; i++) {
		/*Note: This is just a WA to receive packets on channels that
				are not opened by application, with this WA it is possible
				to receive and drop broadcast packets on channels
				closed/not opened by application */
		ret = hsi_ll_open(i);
		if (ret != 0)
			hsi_ll_err("Error opening Channel %d,err=%d.", i, ret);
	}
#endif

	ret = hsi_ll_config_bus();

	if(ret) {
		return -1;
	}

#if !defined (HSI_LL_ENABLE_PM)
	hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);
#endif
	ret = hsi_ll_create_rx_thread();

	if (ret) {
		hsi_ll_err("Failed to create RX thread."
					" Initiating HSI Link Layer Shutdown.");
		hsi_ll_shutdown();
		return -1;
	}

	wait_event_interruptible(hsi_ll_if.msg_avaliable,
							 hsi_ll_if.msg_avaliable_flag == 1);
#if defined (CONFIG_EN_HSI_EDLP)
	hsi_ll_if.ping_modem_once = 1;
#endif
	while(1) {
		u32 command, ch;

		hsi_ll_if.msg_avaliable_flag = 0;

		while(0 > hsi_ll_read_cmd_queue(&command,
										&ch)) {
#if defined (HSI_LL_ENABLE_PM)
			wait_event_interruptible_timeout(hsi_ll_if.msg_avaliable,
											 hsi_ll_if.msg_avaliable_flag == 1,
											 20);
			if (hsi_ll_if.msg_avaliable_flag == 0) {
#if !defined (CONFIG_EN_HSI_EDLP)
				spin_lock_bh(&hsi_ll_if.psv_flag_lock);
				hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_ENABLE;
				spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
				wake_up_interruptible(&hsi_ll_if.psv_event);
				hsi_ll_dbg("PSV enable requested.");
#else
				HSI_LL_RESET_AC_WAKE(HSI_LL_CTRL_CHANNEL)
#endif
#endif
				wait_event_interruptible(hsi_ll_if.msg_avaliable,
										 hsi_ll_if.msg_avaliable_flag == 1);
#if defined (HSI_LL_ENABLE_PM)
			}
#if !defined (CONFIG_EN_HSI_EDLP)
			spin_lock_bh(&hsi_ll_if.psv_flag_lock);
			hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
			spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
			hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_UNDEF;
#endif
#endif
			hsi_ll_if.msg_avaliable_flag = 0;
		}

		if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
			hsi_ll_dbg("Stop CMD transfer till recovery completes.");
			wait_event_interruptible(hsi_ll_if.wr_complete,
									hsi_ll_if.wr_complete_flag == 2);
			hsi_ll_if.wr_complete_flag = 0;
			hsi_ll_dbg("HSI LL recovery completed .Start CMD transfer.");
		}
#if 0
		if (hsi_ll_if.ping_modem_once) {
			u32 modem_status = 0;
			hsi_ll_ioctl(0,
						 HSI_LL_IOCTL_MODEM_ALIVE,
						 &modem_status);
			if(!modem_status) {
				hsi_ll_err("Modem is not responding.");
			}
			hsi_ll_if.ping_modem_once = 0;
		}
#endif
		/* Wakeup Other Side */
#if !defined (CONFIG_EN_HSI_EDLP)
		mutex_lock(&hsi_ll_acwake);
		if (hsi_ll_data.tx_cfg.ac_wake == HSI_LL_WAKE_LINE_LOW) {
			hsi_ll_dbg("Requesting AC wake line High.");
			hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);
		}
		mutex_unlock(&hsi_ll_acwake);
#else
		HSI_LL_SET_AC_WAKE(HSI_LL_CTRL_CHANNEL)
#endif
#if 0
		hsi_ll_info("AP => CP CMD = 0x%08X.", command);
#else
		hsi_ll_dbg("AP => CP CMD = 0x%08X.", command);
#endif
		hsi_ll_data.tx_cmd.cmd = command;
		hsi_ll_data.tx_cmd.ch = ch;
#if !defined (CONFIG_EN_HSI_EDLP) && defined (HSI_LL_ENABLE_PM)
		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_TX;
#endif

		if (0 > (ret = hsi_write(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
								&command,
								1))) {
			hsi_ll_err("hsi_write failed for ctrl chanel, err=%d.", ret);
		} else {
			wait_event_interruptible(hsi_ll_if.wr_complete,
									hsi_ll_if.wr_complete_flag == 1);
			hsi_ll_if.wr_complete_flag = 0;
		}
	}

	return 0;
}

static int hsi_ll_rd_ctrl_ch_th(void *data)
{
	int ret;
	hsi_ll_dbg("Read thread Started.");

	while(1) {
		if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
			hsi_ll_dbg("Stop CMD reception till recovery completes.");
			wait_event_interruptible(hsi_ll_if.rd_complete,
						 hsi_ll_if.rd_complete_flag == 2);
			hsi_ll_if.rd_complete_flag = 0;
			hsi_ll_dbg("HSI LL recovery completed .Start CMD reception.");
		}
		if (0 > (ret = hsi_read(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
								&hsi_ll_data.rx_cmd,
								1))) {
			hsi_ll_err("hsi_read failed for ctrl channel, err=%d.", ret);
		} else {
			wait_event_interruptible(hsi_ll_if.rd_complete,
									 hsi_ll_if.rd_complete_flag == 1);
			hsi_ll_if.rd_complete_flag = 0;
		}
	}

	return 0;
}

/**
 * hsi_ll_write - HSI LL Write
 * @ch: Channel Number.
 * @char: pointer to buffer.
 * @size: Number of bytes to be transferred.
 */
int hsi_ll_write(int ch, void *buf, u32 size)
{
	int ret = 0;

	if ((ch <= 0) || (ch >= HSI_LL_MAX_CHANNELS)) {
		hsi_ll_err("write attempted for invalid channel %d.", ch);
		return -EINVAL;
	}

	if (hsi_ll_data.ch[ch].open == FALSE) {
		hsi_ll_err("write failed as ch %d is not opened.", ch);
		return -1;
	}

	if ((buf == NULL) || (size == 0)) {
		ret = -EINVAL;
		hsi_ll_err("write failed for ch %d due to invalid params.", ch);
		goto quit_write;
	}
#if defined (CONFIG_EN_HSI_EDLP)
	if ((hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_PACKET) ||
		(hsi_ll_data.ch[ch].pdu_type == HSI_LL_PDU_TYPE_RAW)){
		if ((hsi_ll_data.ch[ch].ip_ch == FALSE) &&
			(hsi_ll_data.ch[ch].tx.is_first_time)){
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_ACK;
			hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN,
								ch,
								&hsi_ll_data.ch[ch].tx.pdu_size);
			hsi_ll_dbg("Waiting for ACK resp for open conn on ch %d", ch);
			wait_event_interruptible (hsi_ll_data.ch[ch].tx.ev_credits,
								hsi_ll_data.ch[ch].tx.credits_flag == TRUE);
			hsi_ll_data.ch[ch].tx.credits_flag = FALSE;
		}

		if(hsi_ll_data.ch[ch].ip_ch == TRUE) {
			if (!hsi_ll_data.ch[ch].tx.wq_active){
				hsi_ll_data.ch[ch].tx.wq_active = TRUE;
				wake_up(&hsi_ll_data.ch[ch].tx.wq_notify);
			}
			ret = 0;
		} else {
		if ((hsi_ll_data.ch[ch].tx.state == HSI_LL_TX_STATE_IDLE)  ||
			(hsi_ll_data.ch[ch].tx.state == HSI_LL_TX_STATE_WAIT4_CREDITS)) {
				u32 *ul_buf = hsi_ll_data.ch[ch].tx.ul_buf[0].ptr;
				/*Frame PDU*/
				*((u32*)ul_buf) = (HSI_LL_PDU_SIGNATURE |  /*Signature+Seq Number*/
							(hsi_ll_data.ch[ch].tx.seq_num & 0x0000FFFF));
				hsi_ll_data.ch[ch].tx.seq_num++;
				*((u32*)ul_buf + 1) = 0x10;
				*((u32*)ul_buf + 2) = (size | 0x60000000);
				memcpy((ul_buf + 4),
						buf,
						size);
				if (!hsi_ll_data.ch[ch].tx.credits){
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_CREDITS;
					hsi_ll_data.ch[ch].tx.credits_flag = FALSE;
					hsi_ll_dbg("Waiting for CREDITS for ch %d", ch);
					wait_event_interruptible (hsi_ll_data.ch[ch].tx.ev_credits,
										hsi_ll_data.ch[ch].tx.credits_flag == TRUE);
					hsi_ll_data.ch[ch].tx.credits_flag = FALSE;
				}
				hsi_ll_data.ch[ch].tx.credits--;
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_TX;
				HSI_LL_SET_AC_WAKE(ch)
				if(0 > hsi_write(hsi_ll_data.dev[ch],
								 ul_buf,
								 hsi_ll_data.ch[ch].tx.pdu_size)) {
					hsi_ll_err("hsi_write err for ch %d", ch);
					ret = -1;
					hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
					hsi_ll_data.ch[ch].tx.credits++;
				}
				hsi_ll_dbg("hsi_write issued for ch %d, available credits %d",
							ch, hsi_ll_data.ch[ch].tx.credits);
			}
		}
	} else {
#endif
		hsi_ll_data.ch[ch].tx.ptr  = buf;
		hsi_ll_data.ch[ch].tx.size = size;

		if ((hsi_ll_data.ch[ch].tx.state == HSI_LL_TX_STATE_IDLE)  &&
			(hsi_ll_data.state				!= HSI_LL_IF_STATE_CONFIG)) {
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_OPEN_CONN;
			hsi_ll_data.ch[ch].tx.retry = 0;
			ret = hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
									  ch,
									  &size);

			if (ret != 0) {
				hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
				hsi_ll_err("write failed for channel %d.", ch);
				goto quit_write;
			}
		} else {
#if 0
			hsi_ll_err("write failed . Ch %d is busy.", ch);
#else
			hsi_ll_err("write failed . Ch %d is busy, "
						"hsi_ll_data.ch[%d].tx.state=%d(%s), "
						"hsi_ll_data.state=%d(%s)", ch, ch,
						hsi_ll_data.ch[ch].tx.state,
						channel_tx_state[hsi_ll_data.ch[ch].tx.state],
						hsi_ll_data.state, if_state[hsi_ll_data.state]);
#endif
			ret = -1;
			goto quit_write;
		}
#if defined (CONFIG_EN_HSI_EDLP)
	}
#endif
quit_write:
	return ret;
}

/**
 * hsi_ll_open - HSI channel open.
 * @ch: Channel number.
 */
int hsi_ll_open(int ch)
{
	int ret = 0;

	mutex_lock(& hsi_ll_open_mutex);

	if (hsi_ll_data.ch[ch].open == TRUE) {
#if 0
		hsi_ll_info("channel %d already open.", ch);
#else
		hsi_ll_dbg("channel %d already open.", ch);
#endif
		goto quit_open;
	}

	if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY ||
		hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR   ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
#if 0
		hsi_ll_err("Invalid state for channel %d.", ch);
#else
		hsi_ll_err("Invalid state %d(%s)for channel %d.",
					hsi_ll_data.state, if_state[hsi_ll_data.state], ch);
#endif
		ret = -1;
		goto quit_open;
	}

	hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_IDLE;
	hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;

	if (0 > hsi_open(hsi_ll_data.dev[ch])) {
		hsi_ll_err("Could not open channel %d.", ch);
		ret = -1;
	} else {
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_data.ch[ch].tx.channel = ch;
		INIT_WORK(&hsi_ll_data.ch[ch].tx.retry_work, hsi_ll_retry_work);
#endif
		hsi_ll_data.ch[ch].open = TRUE;
#if defined (CONFIG_EN_HSI_EDLP)
	hsi_ll_data.ch[ch].tx.is_first_time = 1;
	if(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_WAIT4_CH_OPEN) {
		hsi_ll_data.ch[ch].rx.ch_open_flag = TRUE;
		wake_up_interruptible(&hsi_ll_data.ch[ch].rx.ch_open);
	}
#endif
	}

quit_open:
	mutex_unlock(&hsi_ll_open_mutex);
	return ret;
}

/**
 * hsi_ll_close - HSI channel close.
 * @ch: Channel number.
 */
int hsi_ll_close(int ch)
{
	int ret = 0;

	mutex_lock(& hsi_ll_close_mutex);

	if (hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
		ret = -1;
		goto quit_close;
	}
#if 0
	/*NOTE: Below code block is commented as a WA. This is done to ensure that
			all hsi channels are kept open all the time so that
			broadcast messages from CP are not blocked */
	if ((hsi_ll_data.ch[ch].rx.state != HSI_LL_RX_STATE_IDLE)		||
		(hsi_ll_data.ch[ch].rx.state != HSI_LL_RX_STATE_POWER_DOWN)) {
		if (ch == 0) {
			/* wait until end of transmission */
			hsi_ll_data.ch[ch].rx.close_req = TRUE;
		} else if (hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_RX) {
			u32 dirn = HSI_LL_DIRN_RX;
			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								ch,
								&dirn);
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_FREE_MEM,
					  hsi_ll_data.ch[ch].rx.ptr);
			hsi_ll_data.ch[ch].rx.ptr    = NULL;
			hsi_ll_data.ch[ch].rx.close_req = FALSE;
			hsi_ll_data.ch[ch].rx.state     = HSI_LL_RX_STATE_CANCEL_CONN;
		} else if (hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_SEND_ACK  ||
			hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_SEND_NACK) {
			hsi_ll_data.ch[ch].rx.close_req = TRUE;
		}
	} else {
		hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_CLOSED;
	}
	/* Upper layers should take care that noting is
		sent on this ch till it is opened again*/
	hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_CLOSED;
	hsi_close(hsi_ll_data.dev[ch]);
	hsi_ll_data.ch[ch].open = FALSE;
#endif

quit_close:
	mutex_unlock(&hsi_ll_close_mutex);
	return 0;
}

static void hsi_ll_wakeup_cp(u32 val)
{
	int ret = -1;

	spin_lock_bh(&hsi_ll_if.psv_flag_lock);
	if (val == HSI_LL_WAKE_LINE_HIGH) {
		hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_HIGH;
		spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
		ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						HSI_IOCTL_ACWAKE_UP,
						NULL);
		hsi_ll_dbg("Setting AC wake line to HIGH.");
	} else {
#if defined (HSI_LL_ENABLE_PM)
#if !defined (CONFIG_EN_HSI_EDLP)
		if(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_DISABLE) {
#else
		if(hsi_ll_if.edlp_tx_ch_active) {
#endif
			spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
			hsi_ll_dbg("PSV revoked.");
			ret = 0;
		} else {
			hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#endif
			hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_LOW;
			spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
			ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
							HSI_IOCTL_ACWAKE_DOWN,
							NULL);
			hsi_ll_dbg("Setting AC wake line to LOW.");
#if defined (HSI_LL_ENABLE_PM)
		}
#endif
	}

	if (ret) {
#if 0
		hsi_ll_err("Error setting AC_WAKE line err=%d.", ret);
#else
		hsi_ll_err("Error setting AC_WAKE line to %d err=%d.", val, ret);
#endif
	}

	return;
}

/*
 * Processes wakeup
 */
#if defined (HSI_LL_ENABLE_PM)
static int hsi_ll_psv_th(void *data)
{
#if !defined(CONFIG_EN_HSI_EDLP)
	u32 psv_done = 0;
#endif
	u32 ch = 0;
	u32 count = 0;

	while(1) {
#if !defined(CONFIG_EN_HSI_EDLP)
		wait_event_interruptible(hsi_ll_if.psv_event,
				(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE));
		psv_done = 0;
		count = 0;
		do{
			if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_DISABLE) {
				hsi_ll_dbg("PSV_Enable Revoked.");
				break;
			}

			for(ch = 0; ch < HSI_LL_MAX_CHANNELS; ch++) {
				if (hsi_ll_data.ch[ch].tx.state != HSI_LL_TX_STATE_IDLE) {
#if 0
					hsi_ll_info("PSV_Enable - Channel[%d] is busy with"
								" status %d, retry after 200ms.",
								ch,hsi_ll_data.ch[ch].tx.state);
#else
					if (!(count % HSI_LL_PSV_BUSY_POLL_PRINT_INTERVAL)) {
						hsi_ll_dbg("PSV_Enable - Channel[%d] is busy with"
								" status %d(%s), retry after %dms, count=%d",
								ch,hsi_ll_data.ch[ch].tx.state,
								channel_tx_state[hsi_ll_data.ch[ch].tx.state],
								HSI_LL_PSV_BUSY_POLL_RETRY_DELAY, count);
					}
					count++;
#endif
					msleep_interruptible(HSI_LL_PSV_BUSY_POLL_RETRY_DELAY);
					break;
				}
			}

			if (ch >= HSI_LL_MAX_CHANNELS) {
				/* All channels are idle. Check if PSV can be Enabled */
				if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) {
					hsi_ll_dbg("Requesting to set AC wake line to low.");
					mutex_lock(&hsi_ll_acwake);
					hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);
					msleep(1);
					mutex_unlock(&hsi_ll_acwake);
					psv_done = 1;
				}
			}
		} while(!psv_done);
#else
		static u32 is_first_time = 1;

		wait_event_interruptible(hsi_ll_if.psv_event,
				((hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) ||
				 (hsi_ll_if.psv_ca_flag == HSI_LL_PSV_EVENT_RX_PSV_ENABLE)));
		if(hsi_ll_if.psv_ca_flag == HSI_LL_PSV_EVENT_RX_PSV_ENABLE) {
			msleep(12); /*To minimize transitions*/
			if(is_first_time) {
				is_first_time = 0;
			} else {
				for (ch=1; ch < XMD_MAX_HSI_CH_IN_USE; ch++) {
					volatile u32 *wr_idx = &hsi_ll_data.ch[ch].rx.wr_idx;
					spin_lock_bh(&hsi_ll_if.psv_flag_lock);
					if((!hsi_ll_if.edlp_ca_wake_status) &&
						(hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state == BUF_RX)) {
						hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state = BUF_RX_PENDING;
						spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
						hsi_read_cancel(hsi_ll_data.dev[ch]);
					} else {
						spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
					}
				}
			}
			spin_lock_bh(&hsi_ll_if.psv_flag_lock);
			hsi_ll_if.psv_ca_flag = HSI_LL_PSV_EVENT_RX_PSV_DISABLE;
			spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
		}
		/*TODO:Check if sleep is required*/
		if(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) {
			msleep(10); /*To minimize AC_WAKE transitions*/
			spin_lock_bh(&hsi_ll_if.psv_flag_lock);
			if ((!hsi_ll_if.edlp_tx_ch_active) &&
				(hsi_ll_data.tx_cfg.ac_wake == HSI_LL_WAKE_LINE_HIGH)){
				spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
				hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);
			} else {
				spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
				hsi_ll_dbg("TX PSV_Enable Revoked.");
			}
		}
#endif
	}

	return 1;
}
#endif

static int hsi_ll_events_init(void)
{
	init_waitqueue_head(&hsi_ll_if.wr_complete);
	init_waitqueue_head(&hsi_ll_if.rd_complete);
	init_waitqueue_head(&hsi_ll_if.msg_avaliable);
	init_waitqueue_head(&hsi_ll_if.reg_complete);
#if defined (CONFIG_EN_HSI_EDLP)
	init_waitqueue_head(&hsi_ll_if.wait_4_echo);
#endif
#if defined (HSI_LL_ENABLE_PM)
	init_waitqueue_head(&hsi_ll_if.psv_event);
#endif
	/* Initialize spin_lock */
	spin_lock_init(&hsi_ll_if.phy_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cmd_lock);
	spin_lock_init(&hsi_ll_if.psv_flag_lock);
	return 0;
}

static int hsi_ll_probe_cb(struct hsi_device *dev)
{
	int port;

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (hsi_ll_iface.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	if (dev->n_ch >= HSI_LL_MAX_CHANNELS) {
		hsi_ll_err("Invalid channel %d.", dev->n_ch);
		return -ENXIO;
	}

	hsi_ll_dbg("Probe Channel ID = %d , Dev[%d] = 0x%08X.",
				dev->n_ch, dev->n_ch, (int)dev);

	spin_lock_bh(&hsi_ll_if.phy_cb_lock);

	if (dev->n_ch == 0) {
		hsi_set_read_cb(dev, hsi_ll_read_complete_cb);
		hsi_set_write_cb(dev, hsi_ll_write_complete_cb);
		hsi_set_port_event_cb(dev, hsi_ll_port_event_cb);
	} else {
		hsi_set_read_cb(dev, hsi_ll_read_cb);
		hsi_set_write_cb(dev, hsi_ll_write_cb);
	}

	hsi_ll_data.dev[dev->n_ch] = dev;
	hsi_ll_data.ch[dev->n_ch].tx.state     = HSI_LL_TX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.close_req = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.pending   = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.ptr    = NULL;
	hsi_ll_data.ch[dev->n_ch].rx.ptr    = NULL;

	hsi_ll_if.reg_complete_ch_count++;

	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	if (hsi_ll_if.reg_complete_ch_count == HSI_LL_MAX_CHANNELS) {
		hsi_ll_if.reg_complete_flag = 1;
		hsi_ll_dbg("Registration for %d channels completed.",
					(u32)HSI_LL_MAX_CHANNELS);
		wake_up_interruptible(&hsi_ll_if.reg_complete);
	}

	return 0;
}

static int hsi_ll_remove_cb(struct hsi_device *dev)
{
	hsi_ll_dbg("Invoked.");
	spin_lock_bh(&hsi_ll_if.phy_cb_lock);
	hsi_set_read_cb(dev, NULL);
	hsi_set_write_cb(dev, NULL);
	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	return 0;
}

#if defined (HSI_LL_ENABLE_PM) && defined (CONFIG_EN_HSI_EDLP)
static void hsi_ll_rx_ch_enable(void)
{
	u32 ch;
	for (ch=1; ch < XMD_MAX_HSI_CH_IN_USE; ch++) {
		if ((hsi_ll_data.ch[ch].pdu_type = HSI_LL_PDU_TYPE_PACKET) ||
			(hsi_ll_data.ch[ch].pdu_type = HSI_LL_PDU_TYPE_RAW)) {
			volatile u32 *wr_idx = &hsi_ll_data.ch[ch].rx.wr_idx;
			spin_lock_bh(&hsi_ll_if.psv_flag_lock);
			if(hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state == BUF_RX_PENDING) {
				hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].state = BUF_RX;
				spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
				if (0 > hsi_read(hsi_ll_data.dev[ch],
					hsi_ll_data.ch[ch].rx.dl_buf[*wr_idx].ptr,
					hsi_ll_data.ch[ch].rx.pdu_size)) {
					hsi_ll_err("hsi_read err for ch %d", ch);
				}
			} else {
				spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
			}
		}
	}
}
#endif

static void hsi_ll_port_event_cb(
	struct hsi_device *dev,
	u32 event,
	void *arg)
{
	if (hsi_ll_data.dev[0] != dev)
		return; /*Ignore repeated callback for other ch*/

	switch(event) {
	case HSI_EVENT_BREAK_DETECTED:
		hsi_ll_err("Break Event detected.");
		break;
	case HSI_EVENT_ERROR:
		hsi_ll_err("Error Event detected.");
		break;
	case HSI_EVENT_PRE_SPEED_CHANGE:
		hsi_ll_dbg("Pre Speed changer Event detected.");
		break;
	case HSI_EVENT_POST_SPEED_CHANGE:
		hsi_ll_dbg("Post Speed changer Event detected.");
		break;
	case HSI_EVENT_CAWAKE_UP:
		hsi_ll_dbg("CA wakeup line UP detected.");
#if defined (HSI_LL_ENABLE_PM) && defined (CONFIG_EN_HSI_EDLP)
		spin_lock_bh(&hsi_ll_if.psv_flag_lock);
		hsi_ll_if.edlp_ca_wake_status = 1;
		hsi_ll_if.psv_ca_flag = HSI_LL_PSV_EVENT_RX_PSV_DISABLE;
		spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
		hsi_ll_rx_ch_enable();
#endif
		break;
	case HSI_EVENT_CAWAKE_DOWN:
		hsi_ll_dbg("CA wakeup line DOWN detected.");
#if defined (HSI_LL_ENABLE_PM) && defined (CONFIG_EN_HSI_EDLP)
		spin_lock_bh(&hsi_ll_if.psv_flag_lock);
		hsi_ll_if.edlp_ca_wake_status = 0;
		hsi_ll_if.psv_ca_flag = HSI_LL_PSV_EVENT_RX_PSV_ENABLE;
		spin_unlock_bh(&hsi_ll_if.psv_flag_lock);
		wake_up_interruptible(&hsi_ll_if.psv_event);
#endif
		break;
	case HSI_EVENT_HSR_DATAAVAILABLE:
		hsi_ll_dbg("HSR Event detected.");
		break;
	default:
		hsi_ll_err("Invalid Event detected.");
		break;
	}
}

/**
 * hsi_ll_init - Initilizes all resources and registers with HSI PHY driver.
 * @port: Number of ports.
 * @cb: pointer to callback.
 */
int hsi_ll_init(int port, const hsi_ll_notify cb)
{
	int i;
	int ret = 0;

	if (!hsi_ll_data.initialized) {
		if ((port <= 0)            ||
			(port > HSI_MAX_PORTS) ||
			(cb == NULL)) {
			ret = -EINVAL;
			hsi_ll_err("Invalid port or callback pointer.");
			goto quit_init;
		}

		port-=1;

		for (i = port; i < HSI_MAX_PORTS; i++) {
			hsi_ll_iface.ch_mask[i] = (0xFFFFFFFF ^
									  (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));
		}

		if (0 != hsi_ll_events_init()) {
			hsi_ll_err("Cannot create Events.");
			ret = -1;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

		hsi_ll_cb = cb;
		hsi_ll_if.reg_complete_flag = 0;
#if !defined(CONFIG_HSI_FLASHLESS_SUPPORT)
		ret = hsi_register_driver(&hsi_ll_iface);
		if (ret) {
			hsi_ll_err("Error while registering with HSI PHY driver "
						"err=%d.", ret);
			goto quit_init;
		}
#endif
		hsi_ll_data.state = HSI_LL_IF_STATE_READY;

		hsi_ll_dbg("Creating threads.");

		hsi_ll_if.wr_th = kthread_run(hsi_ll_wr_ctrl_ch_th,
									  NULL,
									 "hsi_ll_wr_ctrlch");

		if (IS_ERR(hsi_ll_if.wr_th)) {
			hsi_ll_err("Cannot create write ctrl Thread.");
			ret = -1;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

#if defined (HSI_LL_ENABLE_PM)
		hsi_ll_if.psv_th = kthread_run(hsi_ll_psv_th,
									   NULL,
									  "hsi_ll_psv_thread");

		if (IS_ERR(hsi_ll_if.psv_th)) {
			hsi_ll_err("Cannot create PSV Thread.");
			ret = -1;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_if.tx_retry_wq = create_workqueue("hsi_tx_retry_wq");
#endif
#if defined (CONFIG_EN_HSI_EDLP)
		for (i = 1; i < XMD_MAX_HSI_CH_IN_USE; i++) {
			u32 idx;
			u8 hsi_wq_name[32];
			/*Init UL Bufs*/
			u32 sz = hsi_ll_ul_buf_sz[i];
			HSI_LL_TO_MUL_OF_QWORD(sz)
			for(idx= 0; idx < hsi_ll_no_of_ul_buf[i]; idx++) {
				hsi_ll_data.ch[i].tx.ul_buf[idx].ptr = hsi_mem_alloc(sz);
				if(hsi_ll_data.ch[i].tx.ul_buf[idx].ptr == NULL){
					hsi_ll_err("Mem alloc for UL ch %d failed.", i);
					ret = -1;
					hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
					goto quit_init;
				}
			}
			HSI_LL_GET_SIZE_IN_WORDS(sz)
			hsi_ll_data.ch[i].tx.pdu_size = sz;
			HSI_LL_WQ_NAME(hsi_wq_name, "hsi_dl_wq", i);
			hsi_ll_if.dl_wq[i] = create_workqueue(hsi_wq_name);
			init_waitqueue_head(&hsi_ll_data.ch[i].tx.ev_credits);
			init_waitqueue_head(&hsi_ll_data.ch[i].rx.ch_open);
			init_waitqueue_head(&hsi_ll_data.ch[i].rx.wq_notify);
			INIT_WORK(&hsi_ll_data.ch[i].rx.dl_work, hsi_ll_dl_work);
		}
#endif
#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
#else
		hsi_ll_data.state = HSI_LL_IF_STATE_READY;
#endif
	}

quit_init:
	return ret;
}

/**
 * hsi_ll_shutdown - Releases all resources.
 */
int hsi_ll_shutdown(void)
{
	if (hsi_ll_data.initialized) {
		hsi_ll_dbg("HSI Link Layer Shutdown initiated.");
		hsi_ll_close(HSI_LL_CTRL_CHANNEL);
		hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
				  HSI_IOCTL_ACWAKE_DOWN,
				  NULL);
		kthread_stop(hsi_ll_if.rd_th);
		kthread_stop(hsi_ll_if.wr_th);
#if defined (HSI_LL_ENABLE_PM)
		kthread_stop(hsi_ll_if.psv_th);
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		destroy_workqueue(hsi_ll_if.tx_retry_wq);
#endif
#if defined (CONFIG_EN_HSI_EDLP)
		destroy_workqueue(hsi_ll_if.dl_wq[0]);
		destroy_workqueue(hsi_ll_if.ul_wq[0]);
#endif
		hsi_unregister_driver(&hsi_ll_iface);
		hsi_ll_data.initialized = FALSE;
	}
	return 0;
}

/* Resets DLP */
int hsi_ll_reset(void)
{
	int ch = 0;
	struct hsi_ll_rx_tx_data temp;

	hsi_ll_data.initialized = FALSE;
	hsi_ll_dbg("DLP recovery started.");
	hsi_ll_data.state = HSI_LL_IF_STATE_ERR_RECOVERY;
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	flush_workqueue(hsi_ll_if.tx_retry_wq);
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	flush_workqueue(hsi_ll_if.dl_wq[0]);
	flush_workqueue(hsi_ll_if.ul_wq[0]);
#endif

	hsi_ll_dbg("Requesting read/write cancel.");
	for(ch = 0; ch < HSI_LL_MAX_CHANNELS; ch++) {
		hsi_ll_data.ch[ch].open = FALSE;
		hsi_write_cancel(hsi_ll_data.dev[ch]);
		hsi_read_cancel(hsi_ll_data.dev[ch]);
	}

	hsi_ll_if.wr_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	hsi_ll_if.rd_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.rd_complete);

	hsi_ll_dbg("Unblocking all read/write callbacks.");

	for(ch = 1; ch < HSI_LL_MAX_CHANNELS; ch++) {
		if (hsi_ll_data.ch[ch].tx.ptr != NULL) {
			temp.ptr = hsi_ll_data.ch[ch].tx.ptr;
			temp.size   = hsi_ll_data.ch[ch].tx.size;
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
		}
		if (hsi_ll_data.ch[ch].rx.ptr != NULL) {
			temp.ptr = hsi_ll_data.ch[ch].rx.ptr;
			temp.size   = hsi_ll_data.ch[ch].rx.size;
			hsi_ll_cb(ch,
					  0,
					  HSI_LL_EV_READ_COMPLETE,
					  &temp);
		}
		hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
		hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_IDLE;
		hsi_ll_data.ch[ch].open = TRUE;
	}

	hsi_ll_dbg("Reset HSI PHY Driver.");
	if(0 > hsi_ioctl(hsi_ll_data.dev[0], HSI_IOCTL_SW_RESET, NULL)) {
		hsi_ll_err("HSI PHY reset error.");
	}

#if defined (HSI_LL_ENABLE_PM)
	hsi_ll_dbg("Revoking PSV enable.");
	hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#endif

	hsi_ll_data.tx_cmd.cnt    = 0;
	hsi_ll_data.tx_cmd.wr_idx = 0;
	hsi_ll_data.tx_cmd.rd_idx = 0;
	hsi_ll_if.msg_avaliable_flag   = 0;
#if !defined(CONFIG_HSI_FLASHLESS_SUPPORT)
	for(ch=0; ch < HSI_LL_MAX_CHANNELS; ch++) {
		hsi_open(hsi_ll_data.dev[ch]);
	}

	hsi_ll_dbg("Reconfiguring HSI Buses.");
	if(0 > hsi_ll_config_bus()) {
		hsi_ll_err("HSI bus reconfig failed. Recovery failed.");
		return -1;
	}
	hsi_ll_data.state = HSI_LL_IF_STATE_READY;
	hsi_ll_dbg("Resuming read/write threads.");
	hsi_ll_if.wr_complete_flag = 2;
	hsi_ll_if.rd_complete_flag = 2;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	wake_up_interruptible(&hsi_ll_if.rd_complete);
	hsi_ll_dbg("DLP recovery completed.");
#else
	hsi_unregister_driver(&hsi_ll_iface);
	hsi_ll_if.reg_complete_ch_count = 0;
	hsi_ll_if.reg_complete_flag = 0;
#endif /* CONFIG_HSI_FLASHLESS_SUPPORT */
	hsi_ll_data.initialized = TRUE;
	return 0;
}

/**
 * hsi_ll_ioctl - HSI LL IOCTL
 * @ch     : Channel Number.
 * @cmd: command to execute
 * @arg    : pointer to a variable specific to command.
 */
int hsi_ll_ioctl(int ch, int cmd, void *arg)
{
	int ret = 0;

	if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT)
		return -1;

	switch(cmd) {
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_IOCTL_TX_RESUME: {
		if (hsi_ll_data.ch[ch].ip_ch == TRUE) {
			hsi_ll_data.ch[ch].tx.wq_active = TRUE;
			wake_up(&hsi_ll_data.ch[ch].tx.wq_notify);
		}
		}
		break;
#endif
	case HSI_LL_IOCTL_RX_RESUME: {
		if(hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_BLOCKED) {
			struct hsi_ll_rx_tx_data *tmp = (struct hsi_ll_rx_tx_data*)arg;
			if(arg == NULL) {
				return -1;
			}
			hsi_ll_dbg("Got mem(size=%d), so unblocking ch %d.",
						tmp->size,ch);
			hsi_ll_data.ch[ch].rx.ptr  = tmp->ptr;
			hsi_ll_data.ch[ch].rx.size = tmp->size;
			hsi_ll_send_command(HSI_LL_MSG_ACK,
								ch,
								&tmp->size);
			HSI_LL_GET_SIZE_IN_WORDS(tmp->size);
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_RX;
			hsi_read(hsi_ll_data.dev[ch],
				tmp->ptr,
				tmp->size);
			ret = 0;
#if defined (CONFIG_EN_HSI_EDLP)
		} else if (hsi_ll_data.ch[ch].rx.state == HSI_LL_RX_STATE_WAIT4_MEM) {
			hsi_ll_data.ch[ch].rx.wr_idx = 0;
			hsi_ll_data.ch[ch].rx.rd_idx = 0;
			hsi_ll_send_command(HSI_LL_MSG_ACK,
								ch,
								&hsi_ll_data.ch[ch].rx.pdu_size);
			if (hsi_ll_data.ch[ch].open == FALSE) {
				hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_WAIT4_CH_OPEN;
				hsi_ll_data.ch[ch].rx.ch_open_flag = FALSE;
				wait_event_interruptible (hsi_ll_data.ch[ch].rx.ch_open,
										  hsi_ll_data.ch[ch].rx.ch_open_flag == TRUE);
			}
			hsi_ll_data.ch[ch].rx.state = HSI_LL_RX_STATE_RX;
			hsi_ll_data.ch[ch].rx.dl_buf[0].state = BUF_RX;
			hsi_ll_data.ch[ch].rx.wq_active = FALSE;
			if (0> hsi_read(hsi_ll_data.dev[ch],
							hsi_ll_data.ch[ch].rx.dl_buf[0].ptr,
							hsi_ll_data.ch[ch].rx.pdu_size))
				hsi_ll_err("hsi_read failed for ch %d", ch);
			 ret = 0;
#endif
			} else {
				ret = -1;
			}
		}
		break;
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_IOCTL_MODEM_ALIVE: {
		int i = HSI_LL_NO_ECHO_RETRY;
		u32 command = 0x10ACAFFE;
		u32 *alive = (u32 *)arg;
		while(i) {
#if 0
			hsi_ll_info("AP => CP CMD = 0x%08X.", command);
#else
			hsi_ll_dbg("AP => CP CMD = 0x%08X.", command);
#endif
			hsi_ll_if.echo_flag = 0;
			hsi_write(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					&command,
					1);
			wait_event_interruptible_timeout(hsi_ll_if.wait_4_echo,
											hsi_ll_if.echo_flag == 1,
											2000);
			if(hsi_ll_if.echo_flag) {
				break;
			}
			i--;
		}
		if (hsi_ll_if.echo_flag == 0) {
			*alive = 0;
		} else {
			*alive = 1;
		}
		}
		break;
	case HSI_LL_CONFIG_IP_CH:
	case HSI_LL_CONFIG_TTY_CH: {
		if(hsi_ll_data.ch[ch].ch_init == TRUE) {
			return 0; /*Probably triggered by recovery case */
		}

		hsi_ll_data.ch[ch].rx.channel   = ch;
		hsi_ll_data.ch[ch].rx.wq_active = FALSE;
		hsi_ll_data.ch[ch].rx.wq_init   = 0;
		hsi_ll_data.ch[ch].rx.pdu_size  = 0;
		hsi_ll_data.ch[ch].rx.rd_idx    = 0;
		hsi_ll_data.ch[ch].rx.wr_idx    = 0;
		hsi_ll_data.ch[ch].tx.channel   = ch;
		hsi_ll_data.ch[ch].tx.wq_active = FALSE;
		hsi_ll_data.ch[ch].tx.wr_idx    = 0;
		hsi_ll_data.ch[ch].tx.cpy_idx   = 0;
		hsi_ll_data.ch[ch].tx.credits   = 0;

		if (HSI_LL_CONFIG_IP_CH == cmd) {
			u8 hsi_wq_name[32];
			hsi_ll_data.ch[ch].ip_ch = TRUE;
			hsi_ll_data.ch[ch].tx.read_ip_pckt = arg;
			hsi_ll_data.ch[ch].pdu_type = HSI_LL_PDU_TYPE_PACKET;
			init_waitqueue_head(&hsi_ll_data.ch[ch].tx.wq_notify);
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_IDLE;
			HSI_LL_WQ_NAME(hsi_wq_name, "hsi_ul_wq", ch);
			hsi_ll_if.ul_wq[ch] = create_workqueue(hsi_wq_name);
			INIT_WORK(&hsi_ll_data.ch[ch].tx.ul_work, hsi_ll_ul_work);
			spin_lock_init(&hsi_ll_data.ch[ch].tx.ul_lock);
		} else {
			hsi_ll_data.ch[ch].ip_ch = FALSE;
			hsi_ll_data.ch[ch].pdu_type = *(u32*)arg;
		}
		PREPARE_WORK(&hsi_ll_data.ch[ch].rx.dl_work,
					  hsi_ll_dl_work);
		queue_work(hsi_ll_if.dl_wq[ch],
				  &hsi_ll_data.ch[ch].rx.dl_work);
		hsi_ll_data.ch[ch].ch_init = TRUE;
		if((hsi_ll_data.ch[ch].tx.state == HSI_LL_TX_STATE_IDLE) &&
			(hsi_ll_data.ch[ch].ip_ch == TRUE)) {
			hsi_ll_data.ch[ch].tx.state = HSI_LL_TX_STATE_WAIT4_ACK;
			hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN,
								ch,
								&hsi_ll_data.ch[ch].tx.pdu_size);
		} else {
			return -1;
		}
		ret = 0;
		}
		break;
#endif
	default:
		ret = -1;
		break;
	}

	return ret;
}

#if defined(CONFIG_HSI_FLASHLESS_SUPPORT)
int hsi_ll_phy_drv_init(void)
{
	int result;
	int ch;
	hsi_ll_iface.ch_mask[0] = (0xFFFFFFFF ^
							  (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));

	result = hsi_register_driver(&hsi_ll_iface);
	if (result) {
		hsi_ll_err("Error while registering with HSI PHY driver"
					" err=%d.", result);
		return result;
	}

	hsi_ll_data.state = HSI_LL_IF_STATE_READY;

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
		wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1);
		for(ch=0; ch < HSI_LL_MAX_CHANNELS; ch++) {
			hsi_open(hsi_ll_data.dev[ch]);
		}

		hsi_ll_dbg("Reconfiguring HSI Buses.");
		if(0 > hsi_ll_config_bus()) {
			hsi_ll_dbg("HSI bus reconfiguration failed. Recovery failed.");
			return -1;
		}

		hsi_ll_dbg("Resuming read/write threads.");

		hsi_ll_if.wr_complete_flag = 2;
		hsi_ll_if.rd_complete_flag = 2;
		wake_up_interruptible(&hsi_ll_if.wr_complete);
		wake_up_interruptible(&hsi_ll_if.rd_complete);

		hsi_ll_info("DLP recovery completed.");
	}
	return result;
}
#endif
