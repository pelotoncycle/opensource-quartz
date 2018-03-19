/*
 * xmd-hsi-ll-internal.h
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

#ifndef __XMD_HSI_LL_INTERNAL_H__
#define __XMD_HSI_LL_INTERNAL_H__

#include <linux/timer.h>
#include <linux/interrupt.h>

#define HSI_LL_CTRL_CHANNEL      0
#define HSI_LL_INVALID_CHANNEL   0xFF
#define HSI_LL_MAX_CMD_Q_SIZE    32

#ifdef HSI_LL_ENABLE_DEBUG_LOG
#define hsi_ll_dbg(fmt, args...) \
printk("\nHSI_LL: " fmt " %s %d\n", ## args, __func__, __LINE__)
#else
#define hsi_ll_dbg(fmt, args...)  do { } while (0)
#endif

#ifdef HSI_LL_ENABLE_INFO_LOG
#define hsi_ll_info(fmt, args...) \
printk("\nHSI_LL: " fmt "\n", ## args)
#else
#define hsi_ll_info(fmt, args...)  do { } while (0)
#endif

#ifdef HSI_LL_ENABLE_ERROR_LOG
#define hsi_ll_err(fmt, args...) \
printk("\nHSI_LL: " fmt " %s %d\n", ## args, __func__, __LINE__)
#else
#define hsi_ll_err(fmt, args...)  do { } while (0)
#endif

#define HSI_LL_WQ_NAME(str, wq_name, ch) \
	snprintf(str, 32, "%s_%d", wq_name, ch)

#define HSI_LL_GET_SIZE_IN_WORDS(size)                       \
	if (size > 4)                                            \
		size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2); \
	else                                                     \
		size = 1;

#define HSI_LL_TO_MUL_OF_WORD(size)                          \
	if(size & 0x3) {                                         \
		size += (4 - (size & 0x3));                          \
	}

#define HSI_LL_TO_MUL_OF_QWORD(size)                         \
	if(size & 0xF) {                                         \
		size += (0x10 - (size & 0xF));                       \
	}

#if defined (CONFIG_EN_HSI_EDLP) && defined (HSI_LL_ENABLE_PM)
#define HSI_LL_SET_AC_WAKE(ch) 									\
{																\
	u32 en;														\
	spin_lock_bh(&hsi_ll_if.psv_flag_lock);						\
	hsi_ll_if.edlp_tx_ch_active |= (1 << ch);					\
	if (hsi_ll_data.tx_cfg.ac_wake == HSI_LL_WAKE_LINE_LOW) {	\
		en = 1;													\
	} else {													\
		hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;\
		en = 0;													\
	}															\
	spin_unlock_bh(&hsi_ll_if.psv_flag_lock);					\
	if (en) { 													\
		hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);				\
	}															\
}

#define HSI_LL_RESET_AC_WAKE(ch) 								\
{																\
	u32 dis;													\
	spin_lock_bh(&hsi_ll_if.psv_flag_lock);						\
	hsi_ll_if.edlp_tx_ch_active &= (~(1 << ch));				\
	if(hsi_ll_if.edlp_tx_ch_active) {							\
		dis = 0;												\
	} else {													\
		dis = 1;												\
		hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_ENABLE; \
	}															\
	spin_unlock_bh(&hsi_ll_if.psv_flag_lock);					\
	if(dis) {													\
		wake_up_interruptible(&hsi_ll_if.psv_event);			\
	}															\
}
#else
#define HSI_LL_SET_AC_WAKE(ch)
#define HSI_LL_RESET_AC_WAKE(ch)
#endif

enum {
	FALSE,
	TRUE,
};

/* Constants to indicate HSI Wake line status */
enum {
	HSI_LL_WAKE_LINE_LOW,
	HSI_LL_WAKE_LINE_HIGH,
};

/* Constants to indicate data transfer direction */
enum {
	HSI_LL_DIRN_TX,
	HSI_LL_DIRN_RX,
	HSI_LL_DIRN_TX_RX,
	HSI_LL_ROLE_RECEIVER,
};

enum {
	TX_FREE,
	TX_PREPARE,
	TX_BUSY,
	TX_READY,
};

enum {
	BUF_FREE,
	BUF_READ,
	BUF_RX,
	BUF_RX_PENDING,
};
/* PSV request modes */
enum {
	HSI_LL_PSV_EVENT_INVALID,
	HSI_LL_PSV_EVENT_PSV_DISABLE,
	HSI_LL_PSV_EVENT_PSV_ENABLE,
	HSI_LL_PSV_EVENT_RX_PSV_DISABLE,
	HSI_LL_PSV_EVENT_RX_PSV_ENABLE,
};

/* Interface state */
enum {
	HSI_LL_IF_STATE_UN_INIT,
	HSI_LL_IF_STATE_READY,
	HSI_LL_IF_STATE_CONFIG,
	HSI_LL_IF_STATE_ERR_RECOVERY,
	HSI_LL_IF_STATE_PERM_ERROR,
};

/* Link Layer commands */
enum {
	HSI_LL_MSG_BREAK           = 0x00,
	HSI_LL_MSG_ECHO            = 0x01,
	HSI_LL_MSG_INFO_REQ        = 0x02,
	HSI_LL_MSG_INFO            = 0x03,
	HSI_LL_MSG_CONFIGURE       = 0x04,
	HSI_LL_MSG_CONF_CH         = 0x05,
	HSI_LL_MSG_RELEASE_CH      = 0x06,
	HSI_LL_MSG_OPEN_CONN       = 0x07,
	HSI_LL_MSG_CONN_READY      = 0x08,
	HSI_LL_MSG_CONN_CLOSED     = 0x09,
	HSI_LL_MSG_CANCEL_CONN     = 0x0A,
	HSI_LL_MSG_ACK             = 0x0B,
	HSI_LL_MSG_NAK             = 0x0C,
	HSI_LL_MSG_CONF_RATE       = 0x0D,
	HSI_LL_MSG_OPEN_CONN_OCTET = 0x0E,
	HSI_LL_MSG_CREDITS         = 0x0F,
	HSI_LL_MSG_INVALID         = 0xFF,
};

/* Channel TX state */
enum {
	HSI_LL_TX_STATE_UNDEF,
	HSI_LL_TX_STATE_CLOSED,
	HSI_LL_TX_STATE_IDLE,
	HSI_LL_TX_STATE_OPEN_CONN,
	HSI_LL_TX_STATE_TX,
	HSI_LL_TX_STATE_TX_READY,
	HSI_LL_TX_STATE_WAIT4_ECHO,
	HSI_LL_TX_STATE_WAIT4_ACK,
	HSI_LL_TX_STATE_WAIT4_CREDITS,
	HSI_LL_TX_STATE_WAIT4_CREDITS_TO_TX,
	HSI_LL_TX_STATE_C_ACK_CONF,
	HSI_LL_TX_STATE_WAIT4_CONN_CLOSED,
	HSI_LL_TX_STATE_WAIT4_TX_COMPLETE,
	HSI_LL_TX_STATE_WAIT4_NEXT_TX,
	HSI_LL_TX_STATE_WAIT4_CONF_ACK,
	HSI_LL_TX_STATE_WAIT4_MEM,
};

/* Channel RX state */
enum {
	HSI_LL_RX_STATE_UNDEF,
	HSI_LL_RX_STATE_CLOSED,
	HSI_LL_RX_STATE_IDLE,
	HSI_LL_RX_STATE_POWER_DOWN,
	HSI_LL_RX_STATE_BLOCKED,
	HSI_LL_RX_STATE_SEND_ACK,
	HSI_LL_RX_STATE_SEND_NACK,
	HSI_LL_RX_STATE_RX,
	HSI_LL_RX_STATE_WAIT4_RX_BUF,
	HSI_LL_RX_STATE_CANCEL_CONN,
	HSI_LL_RX_STATE_CANCEL_ACK,
	HSI_LL_RX_STATE_WAIT4_CH_OPEN,
	HSI_LL_RX_STATE_WAIT4_MEM,
	HSI_LL_RX_STATE_WAIT4_CREDITS_ACK,
	HSI_LL_RX_STATE_WAIT4_CANCEL_CONN_ACK,
};

/* struct hsi_ll_cmd_queue - Command queue
 * @cmd: HSI command
 * @ch: channel number
 */
struct hsi_ll_cmd_queue {
	u32 cmd;
	u32 ch;
};

/* struct hsi_ll_tx_cmd_q - TX command Queue
 * @rd_idx: read index
 * @wr_idx: write index
 * @cnt: queue count
 * @ch:channel number
 * @cmd_q: See struct hsi_ll_cmd_queue
 */
struct hsi_ll_tx_cmd_q {
	u32 rd_idx;
	u32 wr_idx;
	u32 cnt;
	u32 ch;
	u32 dirn;
	u32 cmd;
	struct hsi_ll_cmd_queue cmd_q[HSI_LL_MAX_CMD_Q_SIZE];
};

struct hsi_ll_ul_bufs {
	void *ptr;
	u32 size;
};
/* struct hsi_ll_tx_ch - TX channel structure
 * @state: TX channel state
 * @close_req: Close req indicator
 * @pending: write pending indication
 * @retry: retry count
 * @ptr: pointer to buffer
 * @size: number of bytes
 * @work_struct: retry work Queue ID
 */
struct hsi_ll_tx_ch {
	u32 channel;
	u32 state;
	u32 close_req;
	u32 pending;
	u32 retry;
	void *ptr;
	u32 size;
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	struct work_struct retry_work;
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	u32 is_first_time;
	u32 wq_init;
	u32 credits_flag;
	u32 pdu_size;
	volatile u32 credits;
	volatile u32 credits_cmd_cnt;
	u32 wr_idx;
	u32 cpy_idx;
	u32 wq_active;
	u32 seq_num;
	u32 tasklet_active;
	spinlock_t ul_lock;
	hsi_ll_read_ip_data read_ip_pckt;
	struct hsi_ll_rx_tx_data ul_buf[HSI_LL_MAX_IP_BUFS];
	struct work_struct ul_work;
	wait_queue_head_t  ev_credits;
	wait_queue_head_t  wq_notify;
	struct tasklet_struct ul_tasklet;
#endif
};

/* struct hsi_ll_rx_ch - RX channel structure
 * @state: RX channel state
 * @ptr: pointer to buffer
 * @size: number of bytes
 * @close_req: Close req indicator
 */
struct hsi_ll_rx_ch {
	u32 state;
	void		*ptr;
	u32 size;
	u32 close_req;
#if defined (CONFIG_EN_HSI_EDLP)
	u32 wq_init;
	u32 ch_open_flag;
	u32 channel;
	u32 pdu_size;
	u32 rd_idx;
	u32 wr_idx;
	u32 wq_active;
	struct hsi_ll_rx_tx_data dl_buf[HSI_LL_MAX_IP_BUFS];
	struct work_struct dl_work;
	wait_queue_head_t  ch_open;
	wait_queue_head_t  wq_notify;
	struct tasklet_struct dl_tasklet;
#endif
};

/* struct hsi_ll_channel - HSI LL channel structure
 * @open: channel open/close indicator
 * @tx: see struct hsi_ll_tx_ch
 * @rx: see struct hsi_ll_rx_ch
 */
struct hsi_ll_channel {
	u32        open;
#if defined (CONFIG_EN_HSI_EDLP)
	u32 ip_ch;
	u32 pdu_type;
	u32 ch_init;
#endif
	struct hsi_ll_tx_ch tx;
	struct hsi_ll_rx_ch rx;
};

/* struct hsi_ll_tx_cfg - TX configuration structure
 * @new_data_rate_valid: new baud rate
 * @baud_rate: baud rate
 * @ac_wake: AC wakeline status
 * @ctx: see struct hst_ctx
 */
struct hsi_ll_tx_cfg {
	u32   new_data_rate_valid;
	u32   baud_rate;
	u32   ac_wake;
	struct hst_ctx ctx;
};

/* struct hsi_ll_rx_cfg - RX configuration structure
 * @new_data_rate_valid: new baud rate
 * @baud_rate: baud rate
 * @ca_wake: CA wakeline status
 * @ctx: see struct hsr_ctx
 */
struct hsi_ll_rx_cfg {
	u32   new_data_rate_valid;
	u32   baud_rate;
	u32   ca_wake;
	struct hsr_ctx ctx;
};

/* struct hsi_ll_data_struct - LL data structure
 * @initialized: HSI LL init state
 * @state: Current HSI LL state
 * @rx_cmd: HSI LL CMD from CP
 * @tx_cmd: see struct hsi_ll_tx_cmd_q
 * @tx_cfg: see struct hsi_ll_tx_cfg
 * @rx_cfg: see struct hsi_ll_rx_cfg
 * @ch: see struct hsi_ll_channel
 * @dev:  pointer to hsi_device
 */
struct hsi_ll_data_struct {
	u32            initialized;
	u32            state;
	u32            rx_cmd;
	struct hsi_ll_tx_cmd_q  tx_cmd;
	struct hsi_ll_tx_cfg    tx_cfg;
	struct hsi_ll_rx_cfg    rx_cfg;
	struct hsi_ll_channel   ch[HSI_LL_MAX_CHANNELS];
	struct hsi_device      *dev[HSI_LL_MAX_CHANNELS];
};

/* struct hsi_ll_if_struct - LL IF structure.
 * @wr_complete_flag: Flag to indicate write complete.
 * @rd_complete_flag: Flag to indicate write complete.
 * @msg_avaliable_flag: Flag to indicate message  availability.
 * @reg_complete_flag: Flag to indicate HSI LL registration with HSI PHY driver
 * @psv_event_flag: Flag to indicate PSV event.
 * @reg_complete_ch_count: Counter to indicate number of channels registered.
 * @phy_cb_lock: spin_lock for PHY driver calledbacks.
 * @wr_cmd_cb_lock:  spin_lock for ch 0 write complete callback.
 * @rd_cmd_cb_lock: spin_lock for ch 0 read complete callback.
 * @wr_cb_lock: spin_lock for write complete callback.
 * @rd_cb_lock: spin_lock for read complete callback.
 * @wr_complete: event for write complete.
 * @rd_complete: event for read complete.
 * @msg_avaliable: event for message availability.
 * @reg_complete: event for HSI LL registration complete.
 * @psv_event: event for PSV requests.
 * @rd_th: Read thread ID.
 * @wr_th: write Thread ID.
 * @psv_th: PSV Thread ID.
 */
struct hsi_ll_if_struct {
	int wr_complete_flag;
	int rd_complete_flag;
	int msg_avaliable_flag;
	int reg_complete_flag;
#if defined (HSI_LL_ENABLE_PM)
	int psv_event_flag;
	int psv_ca_flag;
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	int echo_flag;
	u32 ping_modem_once;
	volatile u32 edlp_tx_ch_active;
	volatile u32 edlp_ca_wake_status;
#endif
	u32 reg_complete_ch_count;
	spinlock_t            phy_cb_lock;
	spinlock_t            wr_cmd_lock;
	spinlock_t            psv_flag_lock;
#if defined (CONFIG_EN_HSI_EDLP)
	wait_queue_head_t     wait_4_echo;
#endif
	wait_queue_head_t     wr_complete;
	wait_queue_head_t     rd_complete;
	wait_queue_head_t     msg_avaliable;
	wait_queue_head_t     reg_complete;
#if defined (HSI_LL_ENABLE_PM)
	wait_queue_head_t     psv_event;
#endif
	struct task_struct   *rd_th;
	struct task_struct   *wr_th;
#if defined (HSI_LL_ENABLE_PM)
	struct task_struct   *psv_th;
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	struct workqueue_struct *tx_retry_wq;
#endif
#if defined (CONFIG_EN_HSI_EDLP)
	struct workqueue_struct *dl_wq[XMD_MAX_HSI_CH_IN_USE];
	struct workqueue_struct *ul_wq[XMD_MAX_HSI_CH_IN_USE];
#endif
};

static int hsi_ll_probe_cb(
	struct hsi_device *dev);

static int hsi_ll_remove_cb(
	struct hsi_device *dev);

static void hsi_ll_read_complete_cb(
	struct hsi_device *dev,
	u32 size);

static void hsi_ll_write_complete_cb(
	struct hsi_device *dev,
	u32 size);

static void hsi_ll_port_event_cb(
	struct hsi_device *dev,
	u32 event, void *arg);

static void hsi_ll_read_cb(
	struct hsi_device *dev,
	u32 size);

static void hsi_ll_write_cb(
	struct hsi_device *dev,
	u32 size);

static int hsi_ll_rd_ctrl_ch_th(
	void *data);

static int hsi_ll_wr_ctrl_ch_th(
	void *data);

static void hsi_ll_wakeup_cp(
	u32 val);

#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
static void hsi_ll_retry_work(
	struct work_struct *work);
#endif

#if defined (CONFIG_EN_HSI_EDLP)
static void hsi_ll_ul_work(struct work_struct *work);
static void hsi_ll_dl_work(struct work_struct *work);
#endif

#endif /* __XMD_HSI_LL_INTERNAL_H__ */
