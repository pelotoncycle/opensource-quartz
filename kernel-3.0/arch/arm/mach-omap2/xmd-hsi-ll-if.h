/*
 * xmd-hsi-ll-if.h
 *
 * Header for the HSI link layer interface.
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

#ifndef __XMD_HSI_LL_IF_H__
#define __XMD_HSI_LL_IF_H__

#include "xmd-hsi-ll-cfg.h"

/* Event types */
enum {
	HSI_LL_EV_AP_READY,        /* AP ready to send/receive data. */
	HSI_LL_EV_CP_READY,        /* CP ready to send/receive data. */
	HSI_LL_EV_BREAK_DETECTED,  /* Break detected. */
	HSI_LL_EV_ERROR_DETECTED,  /* error detected. */
	HSI_LL_EV_ALLOC_MEM,       /* Memory alloction request. */
	HSI_LL_EV_ALLOC_MEM_BULK,  /* Memory alloction request for multiple packets.*/
	HSI_LL_EV_FREE_MEM,        /* Memory free request. */
	HSI_LL_EV_RESET_MEM,       /* request to release all memory and
								  reset mem alloc module. */
	HSI_LL_EV_WRITE_COMPLETE,  /* Write complete indication. */
	HSI_LL_EV_READ_COMPLETE,   /* Read complete indication. */
	HSI_LL_EV_DL_PACKET,       /* IP packet available. */
};

/* IOCTL commands */
enum {
	HSI_LL_IOCTL_TX_RESUME,
	HSI_LL_IOCTL_RX_RESUME,
	HSI_LL_IOCTL_START_IP_XMIT,
	HSI_LL_IOCTL_MODEM_ALIVE,
	HSI_LL_CONFIG_TTY_CH,
	HSI_LL_CONFIG_IP_CH,
	HSI_LL_IOCTL_INVALID,
};

/* pdu format to be used for TX and RX */
enum {
	HSI_LL_PDU_TYPE_DLP,
	HSI_LL_PDU_TYPE_PACKET,
	HSI_LL_PDU_TYPE_RAW,
	HSI_LL_PDU_TYPE_INVALID,
};

/* Tx/Rx data structure */
struct hsi_ll_rx_tx_data {
	u32 state;
	u32 size;
	void* ptr;
};

/* Callback function prototype - To read IP packets.
 * @channel: channel number.
 * @size: pointer to size, will be updated with number of bytes buf holds.
 * @arg: pointer to struct/variable where ptr to be copied.
 */
typedef void (*hsi_ll_read_ip_data)(
	u32 channel,
	u32 *size,
	void** arg);

/* Callback function prototype
 * @channel: channel number.
 * @result: result, check for result type.
 * @event: event, check for event type.
 * @arg: Event specific argument.
 */
typedef void (*hsi_ll_notify)(
	u32 channel,
	int result,
	int event,
	void* arg);

/* hsi_ll_init - HSI LL initialization function ,
 *				 returns 0 on success.
 * @port: Number of ports to be used.
 *		  Should match with number of ports defined in hsi_driverif.h.
 * @cb: pointer to callback .
 */
int hsi_ll_init(
	int port,
	const hsi_ll_notify cb);

/* hsi_ll_open - HSI LL open channel, returns 0 on success.
 * @channel: Channel number.
 */
int hsi_ll_open(int channel);

/* hsi_ll_close - HSI LL Close channel,
 *				  returns 0 on success.
 * @channel: Channel number.
 */
int hsi_ll_close(int channel);

/* hsi_ll_write - HSI LL write data, returns 0 on success.
 * buffer should be retained until callback for write completion is invoked
 * by HSI LL.
 * @channel: Channel number.
 * @buf: Pointer to buffer.
 * @size: number of bytes to be transferred.
 * returns 0 on success.
 */
int hsi_ll_write(
	int channel,
	void *buf,
	u32 size);

/* hsi_ll_shutdown - HSI LL shutdown, to be invoked to release all resources
 *					 during shutdown,
 * returns 0 on success.
 */
int hsi_ll_shutdown(void);

/* hsi_ll_reset - HSI LL RESET, to be invoked to reset HSI LL and HSI PHY driver
 *				  to recovery from error,
 * returns 0 on success.
 */
int hsi_ll_reset(void);

/* hsi_ll_ioctl - HSI LL IOCTL, to be invoked to execute IO CTRL CMDs
 * returns 0 on success.
 */
int hsi_ll_ioctl(
	int channel,
	int command,
	void *arg);

#endif /* __XMD_HSI_LL_IF_H__ */
