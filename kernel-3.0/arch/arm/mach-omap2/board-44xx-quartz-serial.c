/* OMAP Identity file for OMAP4 boards.
 *
 * Copyright (C) 2012 Texas Instruments
 * John.Lin <John.Lin@innocomm.com>
 *
 * Based on
 * mach-omap2/board-44xx-quartz-serial.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <plat/omap-serial.h>
#include <linux/serial_reg.h>
#include "mux.h"

static struct omap_device_pad quartz_uart1_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs1.uart1_rx",
#if 0
		.flags	= OMAP_DEVICE_PAD_REMUX,
#endif
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad quartz_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
			  OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad quartz_uart3_pads[] __initdata = {
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
#if 0
		.flags	= OMAP_DEVICE_PAD_REMUX,
#else
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
#endif
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad quartz_uart4_pads[] __initdata = {
};

static struct omap_uart_port_info quartz_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = 0,
};

#if defined(CONFIG_OMAP_FIQ_DEBUGGER) && defined(CONFIG_FIQ_DEBUGGER_CONSOLE)
#else
static struct omap_uart_port_info quartz_uart_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX),
};
#endif /* CONFIG_OMAP_FIQ_DEBUGGER && CONFIG_FIQ_DEBUGGER_CONSOLE */

static struct omap_uart_port_info quartz_uart1_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = -1,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX),
/**
 * The trigger level for the RX FIFO:
 * 00: 8 characters
 * 01: 16 characters
 * 10: 56 characters
 * 11: 60 characters
 * The trigger level for the TX FIFO:
 * 00: 8 spaces
 * 01: 16 spaces
 * 10: 32 spaces
 * 11: 56 spaces
 */
#if 0
	/* @TODO@ NOT verified yet! */
	.fcr_trigger = UART_FCR_R_TRIG_10 | UART_FCR_T_TRIG_01,
#else
	.fcr_trigger = UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_01,
#endif
	.no_iflag = 1,
};

static struct omap_uart_port_info quartz_wilink_uart_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
	.rts_mux_driver_control = 1,
};

void __init quartz_serial_init(void)
{
	omap_serial_init_port_pads(0, quartz_uart1_pads,
		ARRAY_SIZE(quartz_uart1_pads), &quartz_uart1_info);
	omap_serial_init_port_pads(1, quartz_uart2_pads,
		ARRAY_SIZE(quartz_uart2_pads), &quartz_wilink_uart_info);
#if defined(CONFIG_OMAP_FIQ_DEBUGGER) && defined(CONFIG_FIQ_DEBUGGER_CONSOLE)
#else
	omap_serial_init_port_pads(2, quartz_uart3_pads,
		ARRAY_SIZE(quartz_uart3_pads), &quartz_uart_info);
#endif /* CONFIG_OMAP_FIQ_DEBUGGER && CONFIG_FIQ_DEBUGGER_CONSOLE */
	omap_serial_init_port_pads(3, quartz_uart4_pads,
		ARRAY_SIZE(quartz_uart4_pads), &quartz_uart_info_uncon);
}

#if defined(CONFIG_OMAP_FIQ_DEBUGGER) && defined(CONFIG_FIQ_DEBUGGER_CONSOLE)
#include <mach/omap_fiq_debugger.h>

/* fiq_debugger initializes really early but OMAP resource mgmt
 * is not yet ready @ arch_init, so init the serial debugger later */
static int __init board_serial_debug_init(void)
{
	/* Enable fiq debugger & ttyFIQ0 if Console is on FIQ Serial Debugger port */
	return omap_serial_debug_init(2, false, true,
		quartz_uart3_pads, ARRAY_SIZE(quartz_uart3_pads));
}
device_initcall(board_serial_debug_init);
#endif /* CONFIG_OMAP_FIQ_DEBUGGER && CONFIG_FIQ_DEBUGGER_CONSOLE */
