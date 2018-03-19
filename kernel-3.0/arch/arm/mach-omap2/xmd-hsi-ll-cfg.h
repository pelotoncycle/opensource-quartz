/*
 * xmd-hsi-ll-cfg.h
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

#ifndef __XMD_HSI_LL_CFG_H__
#define __XMD_HSI_LL_CFG_H__

#include <plat/omap_hsi.h>

/* Max channels supported */
#if defined (CONFIG_EN_HSI_EDLP)
#define HSI_LL_MAX_CHANNELS         8
#else
#define HSI_LL_MAX_CHANNELS         HSI_CHANNELS_MAX
#endif

/* Receiver modes */
/* Synchronized Data Flow */
#define HSI_LL_SYNC_DATA_FLOW       HSI_FLOW_SYNCHRONIZED
/* Pipelined Data Flow    */
#define HSI_LL_PIPELINED_DATA_FLOW  HSI_FLOW_PIPELINED

/* Interface modes  */
#define HSI_LL_STREAM_MODE          HSI_MODE_STREAM
#define HSI_LL_FRAME_MODE           HSI_MODE_FRAME

/* Priority mode */
/* Round Robin Mode */
#define HSI_LL_ARBMODE_ROUNDROBIN   HSI_ARBMODE_ROUNDROBIN
/* Priority Mode */
#define HSI_LL_ARBMODE_PRIORITY     HSI_ARBMODE_PRIORITY

/* Default settings */
#define HSI_LL_INTERFACE_MODE       HSI_LL_FRAME_MODE
#define HSI_LL_RECEIVER_MODE        HSI_LL_SYNC_DATA_FLOW

#define HSI_LL_ARBMODE_MODE         HSI_LL_ARBMODE_ROUNDROBIN

/* Frame Size */
#define HSI_LL_MAX_FRAME_SIZE       HSI_FRAMESIZE_DEFAULT

/* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3)
    Divisor value => HSI CLK == HSI base CLK/(1+divisor value)
    Clock Change 48MHz => 96MHz */
/* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3) */
#if defined (CONFIG_EN_HSI_EDLP)
#define HSI_LL_DIVISOR_VALUE        0
#else
#define HSI_LL_DIVISOR_VALUE        HSI_DIVISOR_DEFAULT
#endif

/*To enable Power management */
#define HSI_LL_ENABLE_PM

#define HSI_LL_COUNTERS_VALUE      (HSI_COUNTERS_FT_DEFAULT | \
									HSI_COUNTERS_TB_DEFAULT | \
									HSI_COUNTERS_FB_DEFAULT)

/* Max Retries for OPEN_CONNECT_OCTECT */
#define HSI_LL_MAX_OPEN_CONN_RETRIES          200

/* Use this define if TX retry delay WQ has to be enabled.
   If MODEM has logic where it does not send NAK, then below define
   is not required.*/
/* #define HSI_LL_ENABLE_TX_RETRY_WQ */

/* Enable this to make sure that NAK is not sent to MODEM if buf is not
   available. When buf is not available AP does not send any response(NAK)
   instead waits for buffer and then sends NAK. Also it's necessary that
   MODEM TX Timers should be disabled to avoid CP side TX timeouts.*/
#define HSI_LL_ENABLE_RX_BUF_RETRY_WQ

#define HSI_LL_PSV_BUSY_POLL_PRINT_INTERVAL 100

/*Minimum time to sleep before polling for CH status in PSV thread */
#define HSI_LL_PSV_BUSY_POLL_RETRY_DELAY 100

/* #define HSI_LL_ENABLE_DEBUG_LOG */
#if 0
/* #define HSI_LL_ENABLE_INFO_LOG */
#else
#define HSI_LL_ENABLE_INFO_LOG
#endif
#define HSI_LL_ENABLE_ERROR_LOG

#define HSI_LL_PDU_SIGNATURE     0xF9A80000

/*Note: Change this as per requirement */
/*MAX PDUs to be allocated for RX/TX - Applicable only for eDLP case. */
#define HSI_LL_NO_ECHO_RETRY 3 /* Applicable only if eDLP is enabled. */

#define HSI_LL_MAX_IP_BUFS 4
#define HSI_LL_IP_TX_SIZE (1024*4)

#define HSI_LL_MAX_AT_BUFS 2
#define HSI_LL_AT_TX_SIZE (1024*2)

#endif

/* __XMD_HSI_LL_CFG_H__ */
