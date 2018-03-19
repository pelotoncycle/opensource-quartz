/*
 * xmd-hsi-mcm.c
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
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include "xmd-ch.h"
#include "xmd-hsi-mcm.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"
#include <linux/delay.h>
#include <linux/slab.h>

/* #define XMD_MCM_ENABLE_DEBUG_MSG */
#define XMD_MCM_ENABLE_RECOVERY_LOG
#define XMD_MCM_ENABLE_ERR_MSG

#ifdef XMD_MCM_ENABLE_DEBUG_MSG
#define xmd_mcm_dbg(fmt, args...) printk("\nxmd_mcm: " fmt "\n", ## args)
#else
#define xmd_mcm_dbg(fmt, args...) do { } while (0)
#endif

#ifdef XMD_MCM_ENABLE_ERR_MSG
#define xmd_mcm_err(fmt, args...) printk("\nxmd_mcm: " fmt "\n", ## args)
#else
#define xmd_mcm_err(fmt, args...) do { } while (0)
#endif

#ifdef XMD_MCM_ENABLE_RECOVERY_LOG
#define xmd_mcm_recovery_log(fmt, args...) printk("\nxmd_mcm: " fmt "\n", ## args)
#else
#define xmd_mcm_recovery_log(fmt, args...) do { } while (0)
#endif

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ) || defined(CONFIG_EN_HSI_EDLP)
#define HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ
#endif

static u32 hsi_mcm_state;
static int is_dlp_reset_in_progress;

static struct work_struct XMD_DLP_RECOVERY_wq;
static struct hsi_channel hsi_channels[XMD_MAX_HSI_CH_IN_USE];
static struct workqueue_struct *hsi_read_wq;
#if !defined (CONFIG_EN_HSI_EDLP)
static struct workqueue_struct *hsi_write_wq;
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
static struct workqueue_struct *hsi_buf_alloc_wq;
#endif

void (*xmd_boot_cb)(void);
static void hsi_read_work(struct work_struct *work);
#if !defined (CONFIG_EN_HSI_EDLP)
static void hsi_write_work(struct work_struct *work);
#else
void hsi_mcm_read_ip_pckt(u32 chno, u32 *size, void** buf);
#endif
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
void xmd_dlp_recovery(void);
#else
static void xmd_dlp_recovery(void);
#endif
static void xmd_ch_reinit(void);
static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq);
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
static void hsi_buf_alloc_work(struct work_struct *work);
#endif
extern void ifx_pmu_reset(void);
extern void rmnet_restart_queue(int chno);

#if defined (CONFIG_EN_HSI_EDLP)
static const u32 xmd_ch_map[XMD_MAX_HSI_CH_IN_USE] = {
	HSI_LL_PDU_TYPE_INVALID,
	HSI_LL_PDU_TYPE_PACKET,
	HSI_LL_PDU_TYPE_PACKET,
	HSI_LL_PDU_TYPE_PACKET,
	HSI_LL_PDU_TYPE_PACKET,
};
#endif

void init_q(int chno)
{
	int i;

	hsi_channels[chno].rx_q.head  = 0;
	hsi_channels[chno].rx_q.tail  = 0;
	hsi_channels[chno].tx_q.head  = 0;
	hsi_channels[chno].tx_q.tail  = 0;
	hsi_channels[chno].tx_blocked = 0;
	hsi_channels[chno].pending_rx_msgs = 0;
#if defined (CONFIG_EN_HSI_EDLP)
	hsi_channels[chno].pdu_size = 0;
#endif
#ifdef XMD_QUEUE_ENHANCEMENT
	hsi_channels[chno].rx_q.num_x_data = NUM_X_BUF;
	if (!(hsi_channels[chno].rx_q.data))
		hsi_channels[chno].rx_q.data = kmalloc(sizeof(struct x_data) * hsi_channels[chno].rx_q.num_x_data,
								  GFP_DMA | GFP_KERNEL);
	if (!(hsi_channels[chno].rx_q.data)) {
		xmd_mcm_err("No memory available for rx queue of chnno=%d", chno);
		hsi_channels[chno].rx_q.num_x_data = 0;
	}

	if (chno >= XMD_RMNET_HSI_CH)
		hsi_channels[chno].tx_q.num_x_data = NUM_NET_X_BUF;
	else
		hsi_channels[chno].tx_q.num_x_data = NUM_X_BUF;

	if (!(hsi_channels[chno].tx_q.data))
		hsi_channels[chno].tx_q.data = kmalloc(sizeof(struct x_data) * hsi_channels[chno].tx_q.num_x_data,
								  GFP_DMA | GFP_KERNEL);

	if (!(hsi_channels[chno].tx_q.data)) {
		xmd_mcm_err("No memory available for tx queue of chnno=%d", chno);
		hsi_channels[chno].tx_q.num_x_data = 0;
	}

	for (i = 0; i < hsi_channels[chno].rx_q.num_x_data; i++) {
		hsi_channels[chno].rx_q.data[i].being_used = HSI_FALSE;
	}

	for (i = 0; i < hsi_channels[chno].tx_q.num_x_data; i++) {
#else
	for (i=0; i < NUM_X_BUF; i++) {
		hsi_channels[chno].rx_q.data[i].being_used = HSI_FALSE;
#endif
#if defined (CONFIG_EN_HSI_EDLP)
		if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
			if ((xmd_ch_map[i] == HSI_LL_PDU_TYPE_PACKET) &&
				(hsi_channels[chno].tx_q.data[i].buf != NULL) ){
				dev_kfree_skb_any(hsi_channels[chno].tx_q.data[i].buf);
			}
		}
#endif
		hsi_channels[chno].tx_q.data[i].buf = NULL;
	}
}

/* Head grows on reading from q. "data=q[head];head++;" */
struct x_data* read_q(int chno, struct xq* q)
{
	struct x_data *data = NULL;

	if (!q) {
		xmd_mcm_err("NULL Q instance");
		return NULL;
	}

	xmd_mcm_dbg("[read_q]  head = %d, tail = %d.", q->head,q->tail);

	spin_lock_bh(&hsi_channels[chno].lock);

	if (q->head == q->tail) {
		spin_unlock_bh(&hsi_channels[chno].lock);
		xmd_mcm_dbg("Q empty [read].");
		return NULL;
	}

	data = q->data + q->head;
#ifdef XMD_QUEUE_ENHANCEMENT
	q->head = (q->head + 1) % q->num_x_data;
#else
	q->head = (q->head + 1) % NUM_X_BUF;
#endif

	spin_unlock_bh(&hsi_channels[chno].lock);

	return data;
}

/* Tail grows on writing in q. "q[tail]=data;tail++;" */
int write_q(struct xq* q, void *buf, int size)
{
	int temp = 0;

	if (!q) {
		xmd_mcm_err("NULL q instance");
		return 0;
	}

#ifdef XMD_QUEUE_ENHANCEMENT
	temp = (q->tail + 1) % q->num_x_data;
#else
	temp = (q->tail + 1) % NUM_X_BUF;
#endif

	if (temp != q->head) {
		q->data[q->tail].buf  = buf;
		q->data[q->tail].size = size;
		q->tail = temp;
	} else {
#ifdef XMD_QUEUE_ENHANCEMENT
		xmd_mcm_err("Q full [write], head = %d, tail = %d, total = %d.", q->head, q->tail, q->num_x_data);
#else
		xmd_mcm_dbg("Q full [write], head = %d, tail = %d.", q->head,q->tail);
#endif
		return 0;
	}

#ifdef XMD_QUEUE_ENHANCEMENT
	return q->tail > q->head ? q->tail - q->head:q->tail - q->head + q->num_x_data;
#else
	return q->tail > q->head ? q->tail - q->head:q->tail - q->head + NUM_X_BUF;
#endif
}

static int hsi_ch_net_write(int chno, void *data, int len)
{
	/* Non blocking write */
	int n = 0;
	int ret = 0;

#if !defined (CONFIG_EN_HSI_EDLP)
	void *buf = hsi_mem_alloc(len);
	if (!buf || !data) {
		xmd_mcm_err("Failed to alloc memory So Cannot transfer packet.");
		return -ENOMEM;
	}

	memcpy(buf, data, len);
	n = write_q(&hsi_channels[chno].tx_q, buf, len);
#else
	n = write_q(&hsi_channels[chno].tx_q, data, len);
#endif

	xmd_mcm_dbg("n = %d.", n);
	if (n == 0) {
		xmd_mcm_dbg("rmnet TX queue is full for channel %d,"
					" So cannot transfer this packet.", chno);
		hsi_channels[chno].tx_blocked = 1;
		ret = -EBUSY;
#if !defined(CONFIG_EN_HSI_EDLP)
		hsi_mem_free(buf);
	} else if (n == 1) {
		//PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
		//queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
#else
	} else {
		ret = hsi_ll_write(chno, data, len);
#endif
	}

PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
queue_work(hsi_write_wq, &hsi_channels[chno].write_work);

	return ret;
}

static int hsi_ch_tty_write(int chno, void *data, int len)
{
	int err;
#if !defined (CONFIG_EN_HSI_EDLP)
	void *buf = hsi_mem_alloc(len);

	if (!buf) {
		return -ENOMEM;
	}

	memcpy(buf, data, len);
#else
	void *buf = data;
#endif
	hsi_channels[chno].write_happening = HSI_TRUE;

	err = hsi_ll_write(chno, buf, len);
	if (err < 0) {
#if 0
		xmd_mcm_err("hsi_ll_write() failed. err=%d.", err);
#else
		xmd_mcm_err("hsi_ll_write() ch %d failed. err=%d.", chno, err);
#endif
		hsi_channels[chno].write_happening = HSI_FALSE;
	} else {
		xmd_mcm_dbg("Locking mutex for ch: %d.", chno);
		wait_event (hsi_channels[chno].write_wait,
					hsi_channels[chno].write_happening == HSI_FALSE);
	}

	return err;
}

int xmd_ch_write(int chno, void *data, int len)
{
	int err;

	xmd_mcm_dbg("Write entering, ch %d.", chno);

	if (!hsi_channels[chno].write) {
		xmd_mcm_err("Write func NULL for ch: %d.", chno);
		return -EINVAL;
	}

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		xmd_mcm_recovery_log("Dropping packets of channel %d as "
							 "error recovery is in progress.", chno);
			return -ENOTBLK;
	}

	err = hsi_channels[chno].write(chno, data, len);

	xmd_mcm_dbg("Write returning, ch %d.", chno);
	return err;
}

void xmd_ch_close(int chno)
{
	xmd_mcm_dbg("Closing channel %d.", chno);

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	if (((chno >= XMD_RIL_FIRST_CHANNEL) && (chno <= XMD_RIL_LAST_CHANNEL)) &&
#else
	if ((chno == XMD_RIL_RECOVERY_CHANNEL) &&
#endif
		(hsi_mcm_state == HSI_MCM_STATE_INITIALIZED)){
		xmd_mcm_recovery_log("Ch %d closed so starting Recovery.", chno);
		xmd_dlp_recovery();
	}
	if (hsi_channels[chno].read_happening == HSI_TRUE) {
		xmd_mcm_dbg("Locking read mutex for ch: %d.", chno);
		wait_event(hsi_channels[chno].read_wait,
					hsi_channels[chno].read_happening == HSI_FALSE);
	}
	hsi_ll_close(chno);
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].state = HSI_CH_FREE;
	spin_unlock_bh(&hsi_channels[chno].lock);
}

int xmd_ch_open(
	struct xmd_ch_info* info,
	void (*notify_cb)(int chno, void *arg))
{
	int i = 0;

	for (i=0; i < XMD_MAX_HSI_CH_IN_USE; i++) {
		if (hsi_channels[i].name) {
			if (!strcmp(info->name, hsi_channels[i].name)) {
				if (hsi_channels[i].state == HSI_CH_BUSY ||
					hsi_channels[i].state == HSI_CH_NOT_USED) {
					xmd_mcm_err("Channel state not suitable %d", i);
					return -EINVAL;
				}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
				if (((i >= XMD_RIL_FIRST_CHANNEL) && (i <= XMD_RIL_LAST_CHANNEL)) &&
#else
				if ((i == XMD_RIL_RECOVERY_CHANNEL) &&
#endif
					(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY)) {
					xmd_mcm_recovery_log("Recovery completed.");
					xmd_ch_reinit();
				}
				if (0 != hsi_ll_open(i)) {
					xmd_mcm_err("hsi_ll_open failed for channel %d", i);
					return -EINVAL;
				}

				hsi_channels[i].info = info;

				spin_lock_bh(&hsi_channels[i].lock);
				hsi_channels[i].state = HSI_CH_BUSY;
				spin_unlock_bh(&hsi_channels[i].lock);

				hsi_channels[i].notify = notify_cb;
				switch(info->user) {
				case XMD_TTY:
					hsi_channels[i].write = hsi_ch_tty_write;
#if defined (CONFIG_EN_HSI_EDLP)
					if ((xmd_ch_map[i] == HSI_LL_PDU_TYPE_PACKET) ||
						(xmd_ch_map[i] == HSI_LL_PDU_TYPE_RAW)) {
						u32 ch_type = xmd_ch_map[i];
						hsi_ll_ioctl(i, HSI_LL_CONFIG_TTY_CH, &ch_type);
					}
#endif
					break;
				case XMD_NET:
					hsi_channels[i].write = hsi_ch_net_write;
#if defined (CONFIG_EN_HSI_EDLP)
					hsi_ll_ioctl(i, HSI_LL_CONFIG_IP_CH, hsi_mcm_read_ip_pckt);
#endif
					break;
				default:
					xmd_mcm_err("Neither TTY nor NET.");
					return -EINVAL;
				}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
				PREPARE_WORK(&hsi_channels[i].read_work, hsi_read_work);
#if !defined (CONFIG_EN_HSI_EDLP)
				PREPARE_WORK(&hsi_channels[i].write_work, hsi_write_work);
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
				PREPARE_WORK(&hsi_channels[i].buf_alloc_work, hsi_buf_alloc_work);
#endif
#else		/* !XMD_DLP_RECOVERY_ENHANCEMENT */
				INIT_WORK(&hsi_channels[i].read_work, hsi_read_work);
#if !defined (CONFIG_EN_HSI_EDLP)
				INIT_WORK(&hsi_channels[i].write_work, hsi_write_work);
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
				INIT_WORK(&hsi_channels[i].buf_alloc_work, hsi_buf_alloc_work);
#endif
#endif
				return i;
			}
		}
	}
	xmd_mcm_err("Invalid Channel name => %s.", info->name);
	return -EINVAL;
}

void hsi_read_work(struct work_struct *work)
{
	/* function registered with read work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													read_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;

	if (hsi_channels[chno].read_queued == HSI_TRUE) {
		xmd_mcm_dbg("Read wq already in progress.");
		return;
	}

	hsi_channels[chno].read_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].rx_q)) != NULL) {
		void *buf = data->buf;
		struct hsi_ll_rx_tx_data rx_buf;
		rx_buf.ptr = data->buf;
		rx_buf.size = data->size;
		if (hsi_mcm_state != HSI_MCM_STATE_ERR_RECOVERY) {
			hsi_channels[chno].notify(chno, &rx_buf);
		} else {
			xmd_mcm_recovery_log("Dropping RX packets of channel %d from "
								 "WQ as error recovery is in progress.", chno);
		}

		hsi_mem_free(buf);
		if(chno > XMD_MAX_TTYS) {
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_alloc_work,
							  hsi_buf_alloc_work);
				queue_work(hsi_buf_alloc_wq,
						  &hsi_channels[chno].buf_alloc_work);
			}
#endif
			hsi_channels[chno].pending_rx_msgs--;
		}
	}
	hsi_channels[chno].read_queued = HSI_FALSE;
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].read_happening = HSI_FALSE;
	spin_unlock_bh(&hsi_channels[chno].lock);

	wake_up(&hsi_channels[chno].read_wait);
}

#if !defined (CONFIG_EN_HSI_EDLP)
void hsi_write_work(struct work_struct *work)
{
	/* function registered with write work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													write_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;
	int err;

#ifndef XMD_DLP_RECOVERY_ENHANCEMENT
	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		xmd_mcm_recovery_log("Dropping packets of channel %d from WQ as "
							 " error recovery is in progress.", chno);
		goto quit_write_wq;
	}
#endif

	if (hsi_channels[chno].write_queued == HSI_TRUE) {
		xmd_mcm_dbg("Write wq already in progress.");
		return;
	}

	hsi_channels[chno].write_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].tx_q)) != NULL) {
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
			xmd_mcm_recovery_log("Dropping packets of channel %d from WQ as "
								 " error recovery is in progress.", chno);
			hsi_mem_free(data->buf);
			continue;
		}
#endif
		hsi_channels[chno].write_happening = HSI_TRUE;
		data->being_used = HSI_TRUE;
		err = hsi_ll_write(chno, data->buf, data->size);
		if (err < 0) {
			xmd_mcm_err("hsi_ll_write failed.");
			hsi_channels[chno].write_happening = HSI_FALSE;
		} else {
			xmd_mcm_dbg("Locking mutex for ch: %d.", chno);
			wait_event(hsi_channels[chno].write_wait,
						hsi_channels[chno].write_happening == HSI_FALSE);
		}
		data->being_used = HSI_FALSE;
		if (hsi_channels[chno].tx_blocked == 1) {
			hsi_channels[chno].tx_blocked = 0;
			xmd_mcm_dbg("Channel queue free , "
						"restarting TX queue for ch %d.", chno);
			rmnet_restart_queue(chno);
		}
	}

#ifndef XMD_DLP_RECOVERY_ENHANCEMENT
quit_write_wq:
#endif
	hsi_channels[chno].write_queued = HSI_FALSE;
}
#else
void xmd_restart_tx(u32 chno)
{
	hsi_ll_ioctl(chno, HSI_LL_IOCTL_TX_RESUME, NULL);
}

void hsi_mcm_read_ip_pckt(
	u32 chno,
	u32 *size,
	void** buf)
{
	struct x_data *data = NULL;
	if(NULL == (data = read_q(chno, &hsi_channels[chno].tx_q))) {
		*size = 0;
	} else {
		*size = data->size;
		*buf  = data->buf; /*Note: This is skb ptr*/
	}
	if (hsi_channels[chno].tx_blocked == 1) {
		hsi_channels[chno].tx_blocked = 0;
		xmd_mcm_dbg("Channel queue free , "
					"restarting TX queue for ch %d.", chno);
		rmnet_restart_queue(chno);
	}
}
#endif

#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
static void hsi_buf_alloc_work(struct work_struct *work)
{
	struct hsi_ll_rx_tx_data temp_data;
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													buf_alloc_work);
	int chno = ch->info->chno;

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		return;
	}
#if defined (CONFIG_EN_HSI_EDLP)
	if(ch->pdu_size) {
		extern const u32 hsi_ll_no_of_dl_buf[XMD_MAX_HSI_CH_IN_USE];
		int tx_rx =0;
		int i = 0;
		void *ptr = NULL;
		struct hsi_ll_rx_tx_data *dl_buf = ch->dl_buf;
		u32 no_of_bufs = hsi_ll_no_of_dl_buf[chno];

		/*Can't be greater than 0xFFFF*/
		if(ch->pdu_size > 0xFFFF)
			xmd_mcm_err("Mem alloc request for more than 0xFFFF bytes is not expected");
		ch->pdu_size = (ch->pdu_size & 0x0000FFFF);
		tx_rx = dl_buf[0].state;
		while(i < no_of_bufs) {
			{
				u32 max_retry = 100;
				ptr = NULL;
				while(max_retry) {
					ptr = kmalloc(ch->pdu_size,
								  GFP_DMA | GFP_KERNEL);
					if(NULL == ptr) {
						max_retry--;
					} else {
						break;
					}
				}
				if(NULL == ptr) {
					max_retry = 100 - max_retry;
					xmd_mcm_err("Failed to alloc mem(bytes %d), retry cnt %d",
					ch->pdu_size, max_retry);
				}
			}
			dl_buf[i].ptr = ptr;
			dl_buf[i].state = 0;
			i++;
		}
		if(tx_rx) {
			hsi_ll_ioctl(chno, HSI_LL_IOCTL_TX_RESUME, NULL);
		} else {
			hsi_ll_ioctl(chno, HSI_LL_IOCTL_RX_RESUME, NULL);
		}
		return;
	}
#endif
	temp_data.size = ch->pending_rx_size;
	temp_data.ptr = NULL;
	/*GFP_NOFAIL not available so switching to while loop*/
	while(NULL == (temp_data.ptr = kmalloc(temp_data.size,
											  GFP_DMA | GFP_KERNEL)));
	xmd_mcm_dbg("Allocating mem(size=%d) in retry Q for ch %d.",
			temp_data.size, chno);
	if(0 > hsi_ll_ioctl(chno, HSI_LL_IOCTL_RX_RESUME, &temp_data)) {
		kfree(temp_data.ptr);
	}
	ch->pending_rx_size = 0;
}
#endif

void hsi_ch_cb(u32 chno, int result, int event, void* arg)
{
	struct hsi_ll_rx_tx_data *data = (struct hsi_ll_rx_tx_data *) arg;

	if (!(chno <= XMD_MAX_HSI_CH_IN_USE && chno >= 0) ||
		hsi_channels[chno].state == HSI_CH_NOT_USED) {
		xmd_mcm_err("Wrong channel number or channel not used.");
		return;
	}

	switch(event) {
	case HSI_LL_EV_ALLOC_MEM: {
#if !defined (CONFIG_EN_HSI_EDLP)
		if(chno > XMD_MAX_TTYS) {
#ifdef XMD_QUEUE_ENHANCEMENT
			if (hsi_channels[chno].pending_rx_msgs >= hsi_channels[chno].rx_q.num_x_data) {
#else
			if (hsi_channels[chno].pending_rx_msgs >= NUM_X_BUF) {
#endif
				data->ptr = 0;
#if !defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
				xmd_mcm_err("Channel %d RX queue is full "
							"so sending NAK to CP.", chno);
#else
				hsi_channels[chno].pending_rx_size = data->size;
				hsi_channels[chno].rx_blocked = 1;
#endif
				break;
			} else {
				hsi_channels[chno].pending_rx_msgs++;
			}
		}
#endif
		xmd_mcm_dbg("Allocating read memory of size %d to channel %d.",
					data->size, chno);
		/* MODEM can't handle NAK so we allocate memory and
			drop the packet after recieving from MODEM */
#if 0
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
			xmd_mcm_err("Channel not yet opened so not allocating memory.");
			data->ptr = NULL;
			break;
		}
		spin_unlock_bh(&hsi_channels[chno].lock);
#endif
		data->ptr = hsi_mem_alloc(data->size);

#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
		if(data->ptr == NULL) {
			hsi_channels[chno].pending_rx_size = data->size;
			PREPARE_WORK(&hsi_channels[chno].buf_alloc_work,
						 hsi_buf_alloc_work);
			queue_work(hsi_buf_alloc_wq,
					   &hsi_channels[chno].buf_alloc_work);
		}
#endif
		}
		break;
#if defined(CONFIG_EN_HSI_EDLP)
		case HSI_LL_EV_ALLOC_MEM_BULK: {
			hsi_channels[chno].pdu_size = data[0].size;
			hsi_channels[chno].dl_buf  = data;
			PREPARE_WORK(&hsi_channels[chno].buf_alloc_work,
						 hsi_buf_alloc_work);
			queue_work(hsi_buf_alloc_wq,
					   &hsi_channels[chno].buf_alloc_work);
		}
		break;
#endif
	case HSI_LL_EV_FREE_MEM: {
		xmd_mcm_dbg("Freeing memory for channel %d, ptr = 0x%p.",
					chno, data->ptr);
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
			xmd_mcm_err("Channel not yet opened so cant free mem.");
			break;
			}
		spin_unlock_bh(&hsi_channels[chno].lock);
		hsi_mem_free(data->ptr);
		}
		break;

	case HSI_LL_EV_RESET_MEM:
		/* if event is break, handle it somehow. */
		break;

	case HSI_LL_EV_WRITE_COMPLETE: {
		hsi_channels[chno].write_happening = HSI_FALSE;
		wake_up(&hsi_channels[chno].write_wait);
		if (data->ptr != NULL)
			hsi_mem_free(data->ptr);
		xmd_mcm_dbg("Write complete cb, ch %d.", chno);
		}
		break;

	case HSI_LL_EV_READ_COMPLETE: {
		int n = 0;
		xmd_mcm_dbg("Read complete... size %d, channel %d, ptr = 0x%p.",
					data->size, chno, data->ptr);
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
#if !defined(CONFIG_EN_HSI_EDLP)
			if(chno > XMD_MAX_TTYS) {
				hsi_channels[chno].pending_rx_msgs--;
			}
#endif
			spin_unlock_bh(&hsi_channels[chno].lock);
			xmd_mcm_dbg("Channel %d not yet opened so dropping the packet.",
					chno);
			hsi_mem_free(data->ptr);
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_alloc_work,
							  hsi_buf_alloc_work);
				queue_work(hsi_buf_alloc_wq,
						  &hsi_channels[chno].buf_alloc_work);
			}
#endif
			break;
		}

		n = write_q(&hsi_channels[chno].rx_q, data->ptr, data->size);

		spin_unlock_bh(&hsi_channels[chno].lock);

		if (n == 0) {
			xmd_mcm_err("Dropping the packet as channel %d is "
					"busy sending already read data.", chno);
			hsi_mem_free(data->ptr);
			/* Schedule work Q to send data to upper layers */
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		} else if (n == 1) {
			if (hsi_channels[chno].read_happening == HSI_FALSE) {
				hsi_channels[chno].read_happening = HSI_TRUE;
			}
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		}
		/* if n > 1, no need to schdule the wq again. */
		}
		break;
#if defined (CONFIG_EN_HSI_EDLP)
	case HSI_LL_EV_DL_PACKET: { /* This event is invoked from WQ context. */
		hsi_channels[chno].notify(chno, data);
		}
		break;
#endif
	default:/* Wrong event. */
		xmd_mcm_err("Wrong event.ch %d, event %d", chno, event);
		break;
	}
}

void xmd_ch_register_xmd_boot_cb(void (*fn)(void))
{
	xmd_boot_cb = fn;
}

void xmd_ch_init(void)
{
	int i;

	xmd_mcm_dbg("xmd_ch_init++");

	for (i=0; i < XMD_MAX_HSI_CH_IN_USE; i++) {
		hsi_channels[i].state = HSI_CH_FREE;
		HSI_CHANNEL_NAME(hsi_channels[i].name, i);
		hsi_channels[i].write_happening = HSI_FALSE;
		hsi_channels[i].write_queued = HSI_FALSE;
		hsi_channels[i].read_queued = HSI_FALSE;
		hsi_channels[i].read_happening = HSI_FALSE;
		spin_lock_init(&hsi_channels[i].lock);
		init_waitqueue_head(&hsi_channels[i].write_wait);
		init_waitqueue_head(&hsi_channels[i].read_wait);
		init_q(i);
	}

	hsi_mem_init();
	hsi_ll_init(1, hsi_ch_cb);

	/* Create and initialize work q */
	hsi_read_wq = create_workqueue("hsi-read-wq");
#if !defined (CONFIG_EN_HSI_EDLP)
	hsi_write_wq = create_workqueue("hsi-write-wq");
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
	hsi_buf_alloc_wq = create_workqueue("hsi_buf_alloc_wq");
#endif
	INIT_WORK(&XMD_DLP_RECOVERY_wq, xmd_dlp_recovery_wq);
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	for (i = 0; i < XMD_MAX_HSI_CH_IN_USE; i++) {
		INIT_WORK(&hsi_channels[i].read_work, hsi_read_work);
#if !defined (CONFIG_EN_HSI_EDLP)
		INIT_WORK(&hsi_channels[i].write_work, hsi_write_work);
#endif
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
		INIT_WORK(&hsi_channels[i].buf_alloc_work, hsi_buf_alloc_work);
#endif
	}
#endif
	hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
}

void xmd_ch_exit(void)
{
#if defined (HSI_MCM_ENABLE_RX_BUF_ALLOC_WQ)
	flush_workqueue(hsi_buf_alloc_wq);
#endif
	flush_workqueue(hsi_read_wq);
	destroy_workqueue(hsi_read_wq);
#if !defined (CONFIG_EN_HSI_EDLP)
	flush_workqueue(hsi_write_wq);
	destroy_workqueue(hsi_write_wq);
#endif
	hsi_ll_shutdown();
	hsi_mcm_state = HSI_MCM_STATE_UNDEF;
}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
int xmd_is_mcm_in_recovery_state(void)
{
	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY)
		return 1;

	return 0;
}

int xmd_is_cp_in_recovery_state(void)
{
	return is_dlp_reset_in_progress;
}

#endif

int xmd_ch_reset(void)
{
	int ch_i;

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	xmd_mcm_recovery_log("HSI DLP Error Recovery Power down modem.");
	xmd_board_power_off();
#else
	if(	hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		return -1;
	}

	hsi_mcm_state = HSI_MCM_STATE_ERR_RECOVERY;
#endif

	xmd_mcm_recovery_log("HSI DLP Error Recovery initiated.");

#if 0 /*Note: Replace below API with the one specific to this project*/
	xmm_pmu_reset();
#endif

	for (ch_i=0; ch_i < XMD_MAX_HSI_CH_IN_USE; ch_i++) {
		if (hsi_channels[ch_i].write_happening == HSI_TRUE) {
			hsi_channels[ch_i].write_happening = HSI_FALSE;
			wake_up(&hsi_channels[ch_i].write_wait);
		}
	}
#if !defined (CONFIG_EN_HSI_EDLP)
	flush_workqueue(hsi_write_wq);
#endif
	hsi_ll_reset();
	flush_workqueue(hsi_read_wq);

	for (ch_i=0; ch_i < XMD_MAX_HSI_CH_IN_USE; ch_i++) {
		init_q(ch_i);
	}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	msleep(100);
	xmd_mcm_recovery_log("HSI DLP Error Recovery Power on modem.");
	xmd_board_power_on_reset();
	xmd_board_wait_for_cp_ready();
	msleep(1000);
	xmd_board_notify_recovery_complete();
#else
	/*Note: Fine tune Modem Reset delay to required value.*/
	msleep(5000);
#endif

	xmd_mcm_recovery_log("HSI DLP Error Recovery completed waiting "
						 "for CP ready indication from RIL.");
	/* Change MCM state to initilized when CP ready
		indication from tty ctrl channel is issued */

	return 0;
}

static void xmd_ch_reinit(void)
{
	if(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		/*
		 Re-init memory pool so that we wont run out of
		 memory after recovery completed.
		*/
		hsi_mem_reinit();
#endif
		hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
	}
}

void xmd_dlp_recovery(void)
{
	if(!is_dlp_reset_in_progress) {
		is_dlp_reset_in_progress = 1;
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		hsi_mcm_state = HSI_MCM_STATE_ERR_RECOVERY;
#endif
		schedule_work(&XMD_DLP_RECOVERY_wq);
	}
}

static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq)
{
	xmd_ch_reset(); /* Start MCM/DLP cleanup */
	is_dlp_reset_in_progress = 0;
}
