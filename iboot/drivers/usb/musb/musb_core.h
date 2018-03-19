#if 0

/******************************************************************
 * Copyright 2008 Mentor Graphics Corporation
 * Copyright (C) 2008 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

#ifndef __MUSB_HDRC_DEFS_H__
#define __MUSB_HDRC_DEFS_H__

#include <usb.h>
#include <usb_defs.h>
#include <asm/io.h>

#ifdef CONFIG_USB_BLACKFIN
# include "blackfin_usb.h"
#endif

#define MUSB_EP0_FIFOSIZE	64	/* This is non-configurable */

/* EP0 */
struct musb_ep0_regs {
	u16	reserved4;
	u16	csr0;
	u16	reserved5;
	u16	reserved6;
	u16	count0;
	u8	host_type0;
	u8	host_naklimit0;
	u8	reserved7;
	u8	reserved8;
	u8	reserved9;
	u8	configdata;
};

/* EP 1-15 */
struct musb_epN_regs {
	u16	txmaxp;
	u16	txcsr;
	u16	rxmaxp;
	u16	rxcsr;
	u16	rxcount;
	u8	txtype;
	u8	txinterval;
	u8	rxtype;
	u8	rxinterval;
	u8	reserved0;
	u8	fifosize;
};

/* Mentor USB core register overlay structure */
#ifndef musb_regs
struct musb_regs {
	/* common registers */
	u8	faddr;
	u8	power;
	u16	intrtx;
	u16	intrrx;
	u16	intrtxe;
	u16	intrrxe;
	u8	intrusb;
	u8	intrusbe;
	u16	frame;
	u8	index;
	u8	testmode;
	/* indexed registers */
	u16	txmaxp;
	u16	txcsr;
	u16	rxmaxp;
	u16	rxcsr;
	u16	rxcount;
	u8	txtype;
	u8	txinterval;
	u8	rxtype;
	u8	rxinterval;
	u8	reserved0;
	u8	fifosize;
	/* fifo */
	u32	fifox[16];
	/* OTG, dynamic FIFO, version & vendor registers */
	u8	devctl;
	u8	reserved1;
	u8	txfifosz;
	u8	rxfifosz;
	u16	txfifoadd;
	u16	rxfifoadd;
	u32	vcontrol;
	u16	hwvers;
	u16	reserved2a[1];
	u8	ulpi_busctl;
	u8	reserved2b[1];
	u16	reserved2[3];
	u8	epinfo;
	u8	raminfo;
	u8	linkinfo;
	u8	vplen;
	u8	hseof1;
	u8	fseof1;
	u8	lseof1;
	u8	reserved3;
	/* target address registers */
	struct musb_tar_regs {
		u8	txfuncaddr;
		u8	reserved0;
		u8	txhubaddr;
		u8	txhubport;
		u8	rxfuncaddr;
		u8	reserved1;
		u8	rxhubaddr;
		u8	rxhubport;
	} tar[16];
	/*
	 * endpoint registers
	 * ep0 elements are valid when array index is 0
	 * otherwise epN is valid
	 */
	union musb_ep_regs {
		struct musb_ep0_regs ep0;
		struct musb_epN_regs epN;
	} ep[16];

} __attribute__((packed, aligned(USB_DMA_MINALIGN)));
#endif

/*
 * MUSB Register bits
 */

/* POWER */
#define MUSB_POWER_ISOUPDATE	0x80
#define MUSB_POWER_SOFTCONN	0x40
#define MUSB_POWER_HSENAB	0x20
#define MUSB_POWER_HSMODE	0x10
#define MUSB_POWER_RESET	0x08
#define MUSB_POWER_RESUME	0x04
#define MUSB_POWER_SUSPENDM	0x02
#define MUSB_POWER_ENSUSPEND	0x01
#define MUSB_POWER_HSMODE_SHIFT	4

/* INTRUSB */
#define MUSB_INTR_SUSPEND	0x01
#define MUSB_INTR_RESUME	0x02
#define MUSB_INTR_RESET		0x04
#define MUSB_INTR_BABBLE	0x04
#define MUSB_INTR_SOF		0x08
#define MUSB_INTR_CONNECT	0x10
#define MUSB_INTR_DISCONNECT	0x20
#define MUSB_INTR_SESSREQ	0x40
#define MUSB_INTR_VBUSERROR	0x80	/* For SESSION end */

/* DEVCTL */
#define MUSB_DEVCTL_BDEVICE	0x80
#define MUSB_DEVCTL_FSDEV	0x40
#define MUSB_DEVCTL_LSDEV	0x20
#define MUSB_DEVCTL_VBUS	0x18
#define MUSB_DEVCTL_VBUS_SHIFT	3
#define MUSB_DEVCTL_HM		0x04
#define MUSB_DEVCTL_HR		0x02
#define MUSB_DEVCTL_SESSION	0x01

/* ULPI VBUSCONTROL */
#define ULPI_USE_EXTVBUS	0x01
#define ULPI_USE_EXTVBUSIND	0x02

/* TESTMODE */
#define MUSB_TEST_FORCE_HOST	0x80
#define MUSB_TEST_FIFO_ACCESS	0x40
#define MUSB_TEST_FORCE_FS	0x20
#define MUSB_TEST_FORCE_HS	0x10
#define MUSB_TEST_PACKET	0x08
#define MUSB_TEST_K		0x04
#define MUSB_TEST_J		0x02
#define MUSB_TEST_SE0_NAK	0x01

/* Allocate for double-packet buffering (effectively doubles assigned _SIZE) */
#define MUSB_FIFOSZ_DPB		0x10
/* Allocation size (8, 16, 32, ... 4096) */
#define MUSB_FIFOSZ_SIZE	0x0f

/* CSR0 */
#define MUSB_CSR0_FLUSHFIFO	0x0100
#define MUSB_CSR0_TXPKTRDY	0x0002
#define MUSB_CSR0_RXPKTRDY	0x0001

/* CSR0 in Peripheral mode */
#define MUSB_CSR0_P_SVDSETUPEND	0x0080
#define MUSB_CSR0_P_SVDRXPKTRDY	0x0040
#define MUSB_CSR0_P_SENDSTALL	0x0020
#define MUSB_CSR0_P_SETUPEND	0x0010
#define MUSB_CSR0_P_DATAEND	0x0008
#define MUSB_CSR0_P_SENTSTALL	0x0004

/* CSR0 in Host mode */
#define MUSB_CSR0_H_DIS_PING		0x0800
#define MUSB_CSR0_H_WR_DATATOGGLE	0x0400	/* Set to allow setting: */
#define MUSB_CSR0_H_DATATOGGLE		0x0200	/* Data toggle control */
#define MUSB_CSR0_H_NAKTIMEOUT		0x0080
#define MUSB_CSR0_H_STATUSPKT		0x0040
#define MUSB_CSR0_H_REQPKT		0x0020
#define MUSB_CSR0_H_ERROR		0x0010
#define MUSB_CSR0_H_SETUPPKT		0x0008
#define MUSB_CSR0_H_RXSTALL		0x0004

/* CSR0 bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MUSB_CSR0_P_WZC_BITS	\
	(MUSB_CSR0_P_SENTSTALL)
#define MUSB_CSR0_H_WZC_BITS	\
	(MUSB_CSR0_H_NAKTIMEOUT | MUSB_CSR0_H_RXSTALL \
	| MUSB_CSR0_RXPKTRDY)

/* TxType/RxType */
#define MUSB_TYPE_SPEED		0xc0
#define MUSB_TYPE_SPEED_SHIFT	6
#define MUSB_TYPE_SPEED_HIGH 	1
#define MUSB_TYPE_SPEED_FULL 	2
#define MUSB_TYPE_SPEED_LOW	3
#define MUSB_TYPE_PROTO		0x30	/* Implicitly zero for ep0 */
#define MUSB_TYPE_PROTO_SHIFT	4
#define MUSB_TYPE_REMOTE_END	0xf	/* Implicitly zero for ep0 */
#define MUSB_TYPE_PROTO_BULK 	2
#define MUSB_TYPE_PROTO_INTR 	3

/* CONFIGDATA */
#define MUSB_CONFIGDATA_MPRXE		0x80	/* Auto bulk pkt combining */
#define MUSB_CONFIGDATA_MPTXE		0x40	/* Auto bulk pkt splitting */
#define MUSB_CONFIGDATA_BIGENDIAN	0x20
#define MUSB_CONFIGDATA_HBRXE		0x10	/* HB-ISO for RX */
#define MUSB_CONFIGDATA_HBTXE		0x08	/* HB-ISO for TX */
#define MUSB_CONFIGDATA_DYNFIFO		0x04	/* Dynamic FIFO sizing */
#define MUSB_CONFIGDATA_SOFTCONE	0x02	/* SoftConnect */
#define MUSB_CONFIGDATA_UTMIDW		0x01	/* Data width 0/1 => 8/16bits */

/* TXCSR in Peripheral and Host mode */
#define MUSB_TXCSR_AUTOSET		0x8000
#define MUSB_TXCSR_MODE			0x2000
#define MUSB_TXCSR_DMAENAB		0x1000
#define MUSB_TXCSR_FRCDATATOG		0x0800
#define MUSB_TXCSR_DMAMODE		0x0400
#define MUSB_TXCSR_CLRDATATOG		0x0040
#define MUSB_TXCSR_FLUSHFIFO		0x0008
#define MUSB_TXCSR_FIFONOTEMPTY		0x0002
#define MUSB_TXCSR_TXPKTRDY		0x0001

/* TXCSR in Peripheral mode */
#define MUSB_TXCSR_P_ISO		0x4000
#define MUSB_TXCSR_P_INCOMPTX		0x0080
#define MUSB_TXCSR_P_SENTSTALL		0x0020
#define MUSB_TXCSR_P_SENDSTALL		0x0010
#define MUSB_TXCSR_P_UNDERRUN		0x0004

/* TXCSR in Host mode */
#define MUSB_TXCSR_H_WR_DATATOGGLE	0x0200
#define MUSB_TXCSR_H_DATATOGGLE		0x0100
#define MUSB_TXCSR_H_NAKTIMEOUT		0x0080
#define MUSB_TXCSR_H_RXSTALL		0x0020
#define MUSB_TXCSR_H_ERROR		0x0004
#define MUSB_TXCSR_H_DATATOGGLE_SHIFT	8

/* TXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MUSB_TXCSR_P_WZC_BITS	\
	(MUSB_TXCSR_P_INCOMPTX | MUSB_TXCSR_P_SENTSTALL \
	| MUSB_TXCSR_P_UNDERRUN | MUSB_TXCSR_FIFONOTEMPTY)
#define MUSB_TXCSR_H_WZC_BITS	\
	(MUSB_TXCSR_H_NAKTIMEOUT | MUSB_TXCSR_H_RXSTALL \
	| MUSB_TXCSR_H_ERROR | MUSB_TXCSR_FIFONOTEMPTY)

/* RXCSR in Peripheral and Host mode */
#define MUSB_RXCSR_AUTOCLEAR		0x8000
#define MUSB_RXCSR_DMAENAB		0x2000
#define MUSB_RXCSR_DISNYET		0x1000
#define MUSB_RXCSR_PID_ERR		0x1000
#define MUSB_RXCSR_DMAMODE		0x0800
#define MUSB_RXCSR_INCOMPRX		0x0100
#define MUSB_RXCSR_CLRDATATOG		0x0080
#define MUSB_RXCSR_FLUSHFIFO		0x0010
#define MUSB_RXCSR_DATAERROR		0x0008
#define MUSB_RXCSR_FIFOFULL		0x0002
#define MUSB_RXCSR_RXPKTRDY		0x0001

/* RXCSR in Peripheral mode */
#define MUSB_RXCSR_P_ISO		0x4000
#define MUSB_RXCSR_P_SENTSTALL		0x0040
#define MUSB_RXCSR_P_SENDSTALL		0x0020
#define MUSB_RXCSR_P_OVERRUN		0x0004

/* RXCSR in Host mode */
#define MUSB_RXCSR_H_AUTOREQ		0x4000
#define MUSB_RXCSR_H_WR_DATATOGGLE	0x0400
#define MUSB_RXCSR_H_DATATOGGLE		0x0200
#define MUSB_RXCSR_H_RXSTALL		0x0040
#define MUSB_RXCSR_H_REQPKT		0x0020
#define MUSB_RXCSR_H_ERROR		0x0004
#define MUSB_S_RXCSR_H_DATATOGGLE	9

/* RXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MUSB_RXCSR_P_WZC_BITS	\
	(MUSB_RXCSR_P_SENTSTALL | MUSB_RXCSR_P_OVERRUN \
	| MUSB_RXCSR_RXPKTRDY)
#define MUSB_RXCSR_H_WZC_BITS	\
	(MUSB_RXCSR_H_RXSTALL | MUSB_RXCSR_H_ERROR \
	| MUSB_RXCSR_DATAERROR | MUSB_RXCSR_RXPKTRDY)

/* HUBADDR */
#define MUSB_HUBADDR_MULTI_TT		0x80

/* Endpoint configuration information. Note: The value of endpoint fifo size
 * element should be either 8,16,32,64,128,256,512,1024,2048 or 4096. Other
 * values are not supported
 */
struct musb_epinfo {
	u8	epnum;	/* endpoint number 	*/
	u8	epdir;	/* endpoint direction	*/
	u16	epsize;	/* endpoint FIFO size	*/
};

/*
 * Platform specific MUSB configuration. Any platform using the musb
 * functionality should create one instance of this structure in the
 * platform specific file.
 */
struct musb_config {
	struct	musb_regs	*regs;
	u32			timeout;
	u8			musb_speed;
	u8			extvbus;
};

/* externally defined data */
extern struct musb_config	musb_cfg;
extern struct musb_regs		*musbr;

/* exported functions */
extern void musb_start(void);
extern void musb_configure_ep(const struct musb_epinfo *epinfo, u8 cnt);
extern void write_fifo(u8 ep, u32 length, void *fifo_data);
extern void read_fifo(u8 ep, u32 length, void *fifo_data);

#if defined(CONFIG_USB_BLACKFIN)
/* Every USB register is accessed as a 16-bit even if the value itself
 * is only 8-bits in size.  Fun stuff.
 */
# undef  readb
# define readb(addr)     (u8)bfin_read16(addr)
# undef  writeb
# define writeb(b, addr) bfin_write16(addr, b)
# undef MUSB_TXCSR_MODE /* not supported */
# define MUSB_TXCSR_MODE 0
/*
 * The USB PHY on current Blackfin processors is a UTMI+ level 2 PHY.
 * However, it has no ULPI support - so there are no registers at all.
 * That means accesses to ULPI_BUSCONTROL have to be abstracted away.
 */
static inline u8 musb_read_ulpi_buscontrol(struct musb_regs *musbr)
{
	return 0;
}
static inline void musb_write_ulpi_buscontrol(struct musb_regs *musbr, u8 val)
{}
#else
static inline u8 musb_read_ulpi_buscontrol(struct musb_regs *musbr)
{
	return readb(&musbr->ulpi_busctl);
}
static inline void musb_write_ulpi_buscontrol(struct musb_regs *musbr, u8 val)
{
	writeb(val, &musbr->ulpi_busctl);
}
#endif

#endif	/* __MUSB_HDRC_DEFS_H__ */

#else

/*
 * MUSB OTG driver defines
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 *
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp., <www.innocomm.com>
 *
 * Author :
 *  James Wu <james.wu@innocomm.com>
 *
 * Codes and ideas taken from linux/drivers/usb/musb/musb_core.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MUSB_CORE_H__
#define __MUSB_CORE_H__

#include <asm/errno.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <malloc.h>

#include <linux/types.h>
#include <linux/list.h>
#include <linux/compiler.h>
#include <linux/porting-compat.h>
#include <linux/usb/ch9.h>
#include <usbdescriptors.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/musb.h>

struct musb;
struct musb_hw_ep;
struct musb_ep;

/* Helper defines for struct musb->hwvers */
#define MUSB_HWVERS_MAJOR(x)	((x >> 10) & 0x1f)
#define MUSB_HWVERS_MINOR(x)	(x & 0x3ff)
#define MUSB_HWVERS_RC		0x8000
#define MUSB_HWVERS_1300	0x52C
#define MUSB_HWVERS_1400	0x590
#define MUSB_HWVERS_1800	0x720
#define MUSB_HWVERS_1900	0x784
#define MUSB_HWVERS_2000	0x800

#ifndef	BOOL_WAS_DEFINED
#define BOOL_WAS_DEFINED
typedef enum _bool{false,true} bool;
#endif

#include "musb_debug.h"
#include "musb_dma.h"

#include "musb_io.h"
#include "musb_regs.h"

#include "musb_gadget.h"

#define	is_peripheral_enabled(musb)	is_peripheral_capable()
#define	is_host_enabled(musb)		is_host_capable()
#define	is_otg_enabled(musb)		0

#define	is_peripheral_active(musb)	is_peripheral_capable()
#define	is_host_active(musb)		is_host_capable()

/****************************** PERIPHERAL ROLE *****************************/

#define	is_peripheral_capable()	(1)

extern irqreturn_t musb_g_ep0_irq(struct musb *);
extern void musb_g_tx(struct musb *, u8);
extern void musb_g_rx(struct musb *, u8);
extern void musb_g_reset(struct musb *);
extern void musb_g_suspend(struct musb *);
extern void musb_g_resume(struct musb *);
extern void musb_g_wakeup(struct musb *);
extern void musb_g_disconnect(struct musb *);

/****************************** HOST ROLE ***********************************/

#define	is_host_capable()	(0)

static inline irqreturn_t musb_h_ep0_irq(struct musb *m) { return IRQ_NONE; }
static inline void musb_host_tx(struct musb *m, u8 e) {}
static inline void musb_host_rx(struct musb *m, u8 e) {}



/****************************** CONSTANTS ********************************/

#ifndef MUSB_C_NUM_EPS
#define MUSB_C_NUM_EPS ((u8)16)
#endif

#ifndef MUSB_MAX_END0_PACKET
#define MUSB_MAX_END0_PACKET ((u16)MUSB_EP0_FIFOSIZE)
#endif

/* host side ep0 states */
enum musb_h_ep0_state {
	MUSB_EP0_IDLE,
	MUSB_EP0_START,			/* expect ack of setup */
	MUSB_EP0_IN,			/* expect IN DATA */
	MUSB_EP0_OUT,			/* expect ack of OUT DATA */
	MUSB_EP0_STATUS,		/* expect ack of STATUS */
} __attribute__ ((packed));

/* peripheral side ep0 states */
enum musb_g_ep0_state {
	MUSB_EP0_STAGE_IDLE,		/* idle, waiting for SETUP */
	MUSB_EP0_STAGE_SETUP,		/* received SETUP */
	MUSB_EP0_STAGE_TX,		/* IN data */
	MUSB_EP0_STAGE_RX,		/* OUT data */
	MUSB_EP0_STAGE_STATUSIN,	/* (after OUT data) */
	MUSB_EP0_STAGE_STATUSOUT,	/* (after IN data) */
	MUSB_EP0_STAGE_ACKWAIT,		/* after zlp, before statusin */
} __attribute__ ((packed));

/*
 * OTG protocol constants.  See USB OTG 1.3 spec,
 * sections 5.5 "Device Timings" and 6.6.5 "Timers".
 */
#define OTG_TIME_A_WAIT_VRISE	100		/* msec (max) */
#define OTG_TIME_A_WAIT_BCON	1100		/* min 1 second */
#define OTG_TIME_A_AIDL_BDIS	200		/* min 200 msec */
#define OTG_TIME_B_ASE0_BRST	100		/* min 3.125 ms */


/*************************** REGISTER ACCESS ********************************/

/* Endpoint registers (other than dynfifo setup) can be accessed either
 * directly with the "flat" model, or after setting up an index register.
 */

#if defined(CONFIG_ARCH_DAVINCI) || defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP34XX) \
		|| defined(CONFIG_BLACKFIN)
/* REVISIT indexed access seemed to
 * misbehave (on DaVinci) for at least peripheral IN ...
 */
#define	MUSB_FLAT_REG
#endif

/* TUSB mapping: "flat" plus ep0 special cases */
#if	defined(CONFIG_USB_MUSB_TUSB6010)
#define musb_ep_select(_mbase, _epnum) \
	musb_writeb((_mbase), MUSB_INDEX, (_epnum))
#define	MUSB_EP_OFFSET			MUSB_TUSB_OFFSET

/* "flat" mapping: each endpoint has its own i/o address */
#elif	defined(MUSB_FLAT_REG)
#define musb_ep_select(_mbase, _epnum)	(((void)(_mbase)), ((void)(_epnum)))
#define	MUSB_EP_OFFSET			MUSB_FLAT_OFFSET

/* "indexed" mapping: INDEX register controls register bank select */
#else
#define musb_ep_select(_mbase, _epnum) \
	musb_writeb((_mbase), MUSB_INDEX, (_epnum))
#define	MUSB_EP_OFFSET			MUSB_INDEXED_OFFSET
#endif

/****************************** FUNCTIONS ********************************/

#define MUSB_HST_MODE(_musb)\
	{ (_musb)->is_host = true; }
#define MUSB_DEV_MODE(_musb) \
	{ (_musb)->is_host = false; }

#define test_devctl_hst_mode(_x) \
	(musb_readb((_x)->mregs, MUSB_DEVCTL)&MUSB_DEVCTL_HM)

#define MUSB_MODE(musb) ((musb)->is_host ? "Host" : "Peripheral")

/******************************** TYPES *************************************/

/*
 * struct musb_hw_ep - endpoint hardware (bidirectional)
 *
 * Ordered slightly for better cacheline locality.
 */
struct musb_hw_ep {
	struct musb		*musb;
	void __iomem		*fifo;
	void __iomem		*regs;

#ifdef CONFIG_USB_MUSB_TUSB6010
	void __iomem		*conf;
#endif

	/* index in musb->endpoints[]  */
	u8			epnum;

	/* hardware configuration, possibly dynamic */
	bool			is_shared_fifo;
	bool			tx_double_buffered;
	bool			rx_double_buffered;
	u16			max_packet_sz_tx;
	u16			max_packet_sz_rx;

	struct dma_channel	*tx_channel;
	struct dma_channel	*rx_channel;

#ifdef CONFIG_USB_MUSB_TUSB6010
	/* TUSB has "asynchronous" and "synchronous" dma modes */
	dma_addr_t		fifo_async;
	dma_addr_t		fifo_sync;
	void __iomem		*fifo_sync_va;
#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	void __iomem		*target_regs;

	/* currently scheduled peripheral endpoint */
	struct musb_qh		*in_qh;
	struct musb_qh		*out_qh;

	u8			rx_reinit;
	u8			tx_reinit;
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	/* peripheral side */
	struct musb_ep		ep_in;			/* TX */
	struct musb_ep		ep_out;			/* RX */
#endif
};

static inline struct musb_request *next_in_request(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	return next_request(&hw_ep->ep_in);
#else
	return NULL;
#endif
}

static inline struct musb_request *next_out_request(struct musb_hw_ep *hw_ep)
{
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	return next_request(&hw_ep->ep_out);
#else
	return NULL;
#endif
}

/*
 * struct musb - Driver instance data.
 */
struct musb {
	/* device lock */
	spinlock_t		lock;
#if 0
	struct clk		*clock;
#endif
	irqreturn_t		(*isr)(int, void *);
#if 0
	struct work_struct	irq_work;
	struct workqueue_struct	*otg_notifier_wq;
#endif
	u16			hwvers;

/* this hub status bit is reserved by USB 2.0 and not seen by usbcore */
#define MUSB_PORT_STAT_RESUME	(1 << 31)

	u32			port1_status;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	unsigned long		rh_timer;

	enum musb_h_ep0_state	ep0_stage;

	/* bulk traffic normally dedicates endpoint hardware, and each
	 * direction has its own ring of host side endpoints.
	 * we try to progress the transfer at the head of each endpoint's
	 * queue until it completes or NAKs too much; then we try the next
	 * endpoint.
	 */
	struct musb_hw_ep	*bulk_ep;

	struct list_head	control;	/* of musb_qh */
	struct list_head	in_bulk;	/* of musb_qh */
	struct list_head	out_bulk;	/* of musb_qh */

	struct timer_list	otg_timer;
#endif

	struct notifier_block	nb;
	/* called with IRQs blocked; ON/nonzero implies starting a session,
	 * and waiting at least a_wait_vrise_tmout.
	 */
	void			(*board_set_vbus)(struct musb *, int is_on);

	struct dma_controller	*dma_controller;

	struct device		*controller;
	void __iomem		*ctrl_base;
	void __iomem		*mregs;

#ifdef CONFIG_USB_MUSB_TUSB6010
	dma_addr_t		async;
	dma_addr_t		sync;
	void __iomem		*sync_va;
#endif

	/* passed down from chip/board specific irq handlers */
	u8			int_usb;
	u16			int_rx;
	u16			int_tx;

	struct otg_transceiver	*xceiv;

#if 0
	int nIrq;
	unsigned		irq_wake:1;
#endif

	struct musb_hw_ep	 endpoints[MUSB_C_NUM_EPS];
#define control_ep		endpoints

#define VBUSERR_RETRY_COUNT	3
	u16			vbuserr_retry;
	u16 epmask;
	u8 nr_endpoints;

	u8 board_mode;		/* enum musb_mode */
	int			(*board_set_power)(int state);

#if 0
	int			(*set_clock)(struct clk *clk, int is_active);
#endif

	u8			min_power;	/* vbus for periph, in mA/2 */

	bool			is_host;

	int			a_wait_bcon;	/* VBUS timeout in msecs */
	unsigned long		idle_timeout;	/* Next timeout in jiffies */

	/* active means connected and not suspended */
	unsigned		is_active:1;

	unsigned is_multipoint:1;
	unsigned ignore_disconnect:1;	/* during bus resets */

	unsigned		hb_iso_rx:1;	/* high bandwidth iso rx? */
	unsigned		hb_iso_tx:1;	/* high bandwidth iso tx? */
	unsigned		dyn_fifo:1;	/* dynamic FIFO supported? */

	unsigned		bulk_split:1;
#define	can_bulk_split(musb,type) \
	(((type) == USB_ENDPOINT_XFER_BULK) && (musb)->bulk_split)

	unsigned		bulk_combine:1;
#define	can_bulk_combine(musb,type) \
	(((type) == USB_ENDPOINT_XFER_BULK) && (musb)->bulk_combine)

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	/* is_suspended means USB B_PERIPHERAL suspend */
	unsigned		is_suspended:1;

	/* may_wakeup means remote wakeup is enabled */
	unsigned		may_wakeup:1;

	/* is_self_powered is reported in device status and the
	 * config descriptor.  is_bus_powered means B_PERIPHERAL
	 * draws some VBUS current; both can be true.
	 */
	unsigned		is_self_powered:1;
#if 0
	unsigned		is_bus_powered:1;
#endif

	unsigned		set_address:1;
	unsigned		test_mode:1;
	unsigned		softconnect:1;

	u8			address;
	u8			test_mode_nr;
	u16			ackpend;		/* ep0 */
	enum musb_g_ep0_state	ep0_state;
	struct usb_gadget	g;			/* the gadget */
	struct usb_gadget_driver *gadget_driver;	/* its driver */
#endif
	bool			is_ac_charger:1;

	/*
	 * FIXME: Remove this flag.
	 *
	 * This is only added to allow Blackfin to work
	 * with current driver. For some unknown reason
	 * Blackfin doesn't work with double buffering
	 * and that's enabled by default.
	 *
	 * We added this flag to forcefully disable double
	 * buffering until we get it working.
	 */
	unsigned                double_buffer_not_ok:1 __deprecated;

	struct musb_hdrc_config	*config;

#ifdef MUSB_CONFIG_PROC_FS
	struct proc_dir_entry *proc_entry;
#endif
	bool			is_initiated:1;
	bool			is_pc_charger:1;
};


static inline void musb_set_vbus(struct musb *musb, int is_on)
{
	musb->board_set_vbus(musb, is_on);
}

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
static inline struct musb *gadget_to_musb(struct usb_gadget *g)
{
	return container_of(g, struct musb, g);
}
#endif

#ifdef CONFIG_BLACKFIN
static inline int musb_read_fifosize(struct musb *musb,
		struct musb_hw_ep *hw_ep, u8 epnum)
{
	musb->nr_endpoints++;
	musb->epmask |= (1 << epnum);

	if (epnum < 5) {
		hw_ep->max_packet_sz_tx = 128;
		hw_ep->max_packet_sz_rx = 128;
	} else {
		hw_ep->max_packet_sz_tx = 1024;
		hw_ep->max_packet_sz_rx = 1024;
	}
	hw_ep->is_shared_fifo = false;

	return 0;
}

static inline void musb_configure_ep0(struct musb *musb)
{
	musb->endpoints[0].max_packet_sz_tx = MUSB_EP0_FIFOSIZE;
	musb->endpoints[0].max_packet_sz_rx = MUSB_EP0_FIFOSIZE;
	musb->endpoints[0].is_shared_fifo = true;
}

#else

static inline int musb_read_fifosize(struct musb *musb,
		struct musb_hw_ep *hw_ep, u8 epnum)
{
	void *mbase = musb->mregs;
	u8 reg = 0;

	/* read from core using indexed model */
	reg = musb_readb(mbase, MUSB_EP_OFFSET(epnum, MUSB_FIFOSIZE));
	/* 0's returned when no more endpoints */
	if (!reg)
		return -ENODEV;

	musb->nr_endpoints++;
	musb->epmask |= (1 << epnum);

	hw_ep->max_packet_sz_tx = 1 << (reg & 0x0f);

	/* shared TX/RX FIFO? */
	if ((reg & 0xf0) == 0xf0) {
		hw_ep->max_packet_sz_rx = hw_ep->max_packet_sz_tx;
		hw_ep->is_shared_fifo = true;
		return 0;
	} else {
		hw_ep->max_packet_sz_rx = 1 << ((reg & 0xf0) >> 4);
		hw_ep->is_shared_fifo = false;
	}

	return 0;
}

static inline void musb_configure_ep0(struct musb *musb)
{
	musb->endpoints[0].max_packet_sz_tx = MUSB_EP0_FIFOSIZE;
	musb->endpoints[0].max_packet_sz_rx = MUSB_EP0_FIFOSIZE;
	musb->endpoints[0].is_shared_fifo = true;
}
#endif /* CONFIG_BLACKFIN */


/***************************** Glue it together *****************************/

extern const char musb_driver_name[];

extern void musb_start(struct musb *musb);
extern void musb_stop(struct musb *musb);

extern void musb_write_fifo(struct musb_hw_ep *ep, u16 len, const u8 *src);
extern void musb_read_fifo(struct musb_hw_ep *ep, u16 len, u8 *dst);

extern void musb_load_testpacket(struct musb *);

extern irqreturn_t musb_interrupt(struct musb *);
extern irqreturn_t dma_controller_irq(struct musb *);

extern void musb_platform_enable(struct musb *musb);
extern void musb_platform_disable(struct musb *musb);

extern void musb_hnp_stop(struct musb *musb);

extern int musb_platform_set_mode(struct musb *musb, u8 musb_mode);

#if defined(CONFIG_USB_MUSB_TUSB6010) || defined(CONFIG_BLACKFIN)
extern void musb_platform_try_idle(struct musb *musb, unsigned long timeout);
#else
#define musb_platform_try_idle(x, y)		do {} while (0)
#endif

#if defined(CONFIG_USB_MUSB_TUSB6010) || defined(CONFIG_BLACKFIN)
extern int musb_platform_get_vbus_status(struct musb *musb);
#else
#define musb_platform_get_vbus_status(x)	0
#endif

extern int __init musb_platform_init(struct musb *musb);
extern int musb_platform_exit(struct musb *musb);
extern void musb_platform_handle_interrupts(struct musb *musb);

#endif	/* __MUSB_CORE_H__ */

#endif
