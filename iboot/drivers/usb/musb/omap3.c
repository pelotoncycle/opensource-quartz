/*
 * Copyright (C) 2005-2007 by Texas Instruments
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp., <www.innocomm.com>
 *
 * Author :
 *  James Wu <james.wu@innocomm.com>
 *
 * Codes and ideas taken from linux/drivers/usb/musb/omap2430.c
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
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/usb.h>

#include "musb_core.h"

/*----------------------------------------------------------------------*/

#define OTG_REVISION		0x400

#define OTG_SYSCONFIG		0x404
#	define	MIDLEMODE	12	/* bit position */
#	define	FORCESTDBY		(0 << MIDLEMODE)
#	define	NOSTDBY			(1 << MIDLEMODE)
#	define	SMARTSTDBY		(2 << MIDLEMODE)

#	define	SIDLEMODE		3	/* bit position */
#	define	FORCEIDLE		(0 << SIDLEMODE)
#	define	NOIDLE			(1 << SIDLEMODE)
#	define	SMARTIDLE		(2 << SIDLEMODE)
#	define	SMARTIDLEWKUP		(3 << SIDLEMODE)

#	define	ENABLEWAKEUP		(1 << 2)
#	define	SOFTRST			(1 << 1)
#	define	AUTOIDLE		(1 << 0)

#define OTG_SYSSTATUS		0x408
#if !defined(RESETDONE)
#	define	RESETDONE		(1 << 0)
#elif (RESETDONE != 1)
#undef RESETDONE
#	define	RESETDONE		(1 << 0)
#endif /* !RESETDONE */

#define OTG_INTERFSEL		0x40c
#	define	EXTCP			(1 << 2)
#	define	PHYSEL			0	/* bit position */
#	define	PHYSEL_MASK		(3 << PHYSEL)
#	define	UTMI_8BIT		(0 << PHYSEL)
#	define	ULPI_12PIN		(1 << PHYSEL)
#	define	ULPI_8PIN		(2 << PHYSEL)

#define OTG_SIMENABLE		0x410
#	define	TM1			(1 << 0)

#define OTG_FORCESTDBY		0x414
#	define	ENABLEFORCE		(1 << 0)

#define USBOTGHS_CONTROL	0x33c
#	define	AVALID			(1 << 0)
#	define	BVALID			(1 << 1)
#	define	VBUSVALID		(1 << 2)
#	define	SESSEND			(1 << 3)
#	define	IDDIG			(1 << 4)

#define CONTROL_DEV_CONF	0x300
#	define PHY_PD			(1 << 0)

#define USBA0_OTG_CE_PAD1_USBA0_OTG_DP	0x194
#	define	DP_WAKEUPENABLE		(1 << 30)

/*----------------------------------------------------------------------*/

static inline void omap_musb_set_clock(int state)
{
#if defined(CONFIG_OMAP44XX)
	omap4_usb_otg_set_clk(state);
#elif defined(CONFIG_OMAP34XX)
#error "Not implemented for OMAP3\n"
#endif /* CONFIG_OMAP44XX, CONFIG_OMAP34XX */
}

static int musb_platform_suspend(struct musb *musb)
{
	u32 l;

	/* in any role */
	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l |= ENABLEFORCE;	/* enable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);

#if 0
	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l |= ENABLEWAKEUP;	/* enable wakeup */
	musb_writel(musb->mregs, OTG_SYSCONFIG, l);
#else
	/* configure in force idle/ standby */
	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l &= ~(SMARTIDLEWKUP | NOSTDBY | ENABLEWAKEUP);
	l |= FORCEIDLE | FORCESTDBY;
	musb_writel(musb->mregs, OTG_SYSCONFIG,	l);
#endif

	otg_set_clk(musb->xceiv, 0);
	otg_set_suspend(musb->xceiv, 1);

	/** James Wu:
	 * In order to detect charger type correctly on OMAP4, OTG_INTERFSEL must be ULPI_12PIN.
	 * We should change to ULPI_12PIN before jumping to Linux kernel.
	 */
	l = musb_readl(musb->mregs, OTG_INTERFSEL);
	l |= ULPI_12PIN;
	musb_writel(musb->mregs, OTG_INTERFSEL, l);

	if (musb->is_pc_charger) {
		/* James Wu: check the status to avoid the problem on the unknown charger case */
		DBG(1, "shutdown OTG\n");
		otg_shutdown(musb->xceiv);
		musb->is_pc_charger = false;
	}

	/* flush pending interrupts */
	l = musb_readb(musb->mregs, MUSB_INTRUSB);
	l = musb_readw(musb->mregs, MUSB_INTRTX);
	l = musb_readw(musb->mregs, MUSB_INTRRX);

	omap_musb_set_clock(0);

	return 0;
}

static int musb_platform_resume(struct musb *musb)
{
	u32 l;

	otg_set_suspend(musb->xceiv, 0);

	omap_musb_set_clock(1);

	l = musb_readl(musb->mregs, OTG_SYSCONFIG);
	l &= ~ENABLEWAKEUP;	/* disable wakeup */
	musb_writel(musb->mregs, OTG_SYSCONFIG, l);

	l = musb_readl(musb->mregs, OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;	/* disable MSTANDBY */
	musb_writel(musb->mregs, OTG_FORCESTDBY, l);

	return 0;
}

void musb_platform_enable(struct musb *musb)
{
	/* Do nothing */
}

void musb_platform_disable(struct musb *musb)
{
	/* Do nothing */
}

int musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	return 0;
}

static int musb_otg_notifications(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	struct musb	*musb = container_of(nb, struct musb, nb);
	enum usb_xceiv_events xceiv_event = (enum usb_xceiv_events)event;
	u32 val;

	switch (xceiv_event) {
	case USB_EVENT_ID:
		DBG(1, "ID GND\n");
		musb->is_ac_charger = false;
		musb->is_pc_charger = false;
		break;

	case USB_EVENT_CHARGER:
		DBG(1, "Dedicated charger connect\n");
		musb->is_ac_charger = true;
		musb->is_pc_charger = false;
		break;

	case USB_EVENT_VBUS:
		DBG(1, "VBUS Connect\n");
#if 0
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		if (musb->gadget_driver) {
			/*pm_runtime_get_sync(musb->controller);*/
			val = musb_readl(musb->mregs, OTG_INTERFSEL);
#if defined(CONFIG_OMAP44XX)
			val &= ~ULPI_12PIN;
			val |= UTMI_8BIT;
#else
			val |= ULPI_12PIN;
#endif /* CONFIG_OMAP44XX */
			musb_writel(musb->mregs, OTG_INTERFSEL, val);
		}
#endif
#else
		/* James Wu: force to switch to UTMI-8 mode even if there is no gadget driver */
		val = musb_readl(musb->mregs, OTG_INTERFSEL);
#if defined(CONFIG_OMAP44XX)
		val &= ~ULPI_12PIN;
		val |= UTMI_8BIT;
#else
		val |= ULPI_12PIN;
#endif /* CONFIG_OMAP44XX */
		musb_writel(musb->mregs, OTG_INTERFSEL, val);
#endif
		musb->is_pc_charger = true;
		otg_init(musb->xceiv);

		/* James Wu: Workaround solution:
		 * Force to handle the MUSB interrupts to make sure we could receive CONNECT interrupt.
		 */
		{
			unsigned int times = 20;
			while (times--)
				usb_gadget_handle_interrupts();
		}
		break;

	case USB_EVENT_NONE:
		if (musb->is_ac_charger) {
			DBG(1, "Dedicated charger disconnect\n");
			musb->is_ac_charger = false;
			break;
		}

		DBG(1, "VBUS Disconnect\n");
#if 0
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		if (is_otg_enabled(musb) || is_peripheral_enabled(musb))
			if (musb->gadget_driver)
#endif
			{
				pm_runtime_mark_last_busy(musb->controller);
				pm_runtime_put_autosuspend(musb->controller);
			}
#endif

#if 0
#if defined(CONFIG_OMAP44XX)
			omap2430_musb_set_vbus(musb, 0);
			if (musb->xceiv->set_vbus)
				otg_set_vbus(musb->xceiv, 0);
#endif /* CONFIG_OMAP44XX */
#endif

#if 0
		otg_shutdown(musb->xceiv);
		val = musb_readl(musb->mregs, OTG_INTERFSEL);
		val |= ULPI_12PIN;
		musb_writel(musb->mregs, OTG_INTERFSEL, val);
#else
		/* James Wu: change the shutdown sequence to receive MUSB DISCONNECT interrupt */
		val = musb_readl(musb->mregs, OTG_INTERFSEL);
		val |= ULPI_12PIN;
		musb_writel(musb->mregs, OTG_INTERFSEL, val);

		/* James Wu: Workaround solution:
		 * Force to handle the MUSB interrupts to make sure we could receive DISCONNECT interrupt.
		 */
		{
			unsigned int times = 20;
			while (times--)
				usb_gadget_handle_interrupts();
		}

		if (musb->is_pc_charger) {
			/* James Wu: check the status to avoid the problem on the unknown charger case */
			otg_shutdown(musb->xceiv);
			musb->is_pc_charger = false;
		}
#endif
		break;

	case USB_EVENT_ENUMERATED:
		/*DBG(1, "Gadget driver enumerated\n");*/
		break;

	default:
		DBG(1, "ID float\n");
		break;
	}

	return 0;
}

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.num_eps	= 16,
	.ram_bits	= 12,
};

static void omap_musb_enable_all_clocks(void)
{
	/* Enable the USBOTG clocks */
#if defined(CONFIG_OMAP44XX)
	/* Enable usb_otg_hs_xclk & Disable otg_60m_gfclk */
	clrsetbits_le32(&prcm->cm_l3init_hsusbotg_clkctrl,
			USBOTG_CLKCTRL_CLKSEL_60M_MASK,
			USBOTG_CLKCTRL_OPTFCLKEN_XCLK_MASK);

	/* usb_otg_hs_ick */
	omap_clk_enable(&prcm->cm_l3init_hsusbotg_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_HW_AUTO);
#else
#error "omap_musb_enable_all_clocks() not implemented\n"
#endif
}

static void omap_musb_disable_all_clocks(void)
{
	/* Disable the USBOTG clocks */
#if defined(CONFIG_OMAP44XX)
	/* Disable usb_otg_hs_xclk & otg_60m_gfclk */
	clrbits_le32(&prcm->cm_l3init_hsusbotg_clkctrl,
			USBOTG_CLKCTRL_CLKSEL_60M_MASK |
			USBOTG_CLKCTRL_OPTFCLKEN_XCLK_MASK);

	/* usb_otg_hs_ick */
	omap_clk_disable(&prcm->cm_l3init_hsusbotg_clkctrl);
#else
#error "omap_musb_disable_all_clocks() not implemented\n"
#endif
}

static void musb_reset(struct musb *musb)
{
	ulong start;

	/* Reset USB OTG controller */
	DBG(2, "USB OTG: resetting\n");

	omap_musb_enable_all_clocks();

	start = get_timer(0);

	musb_writel(musb->mregs, OTG_SYSCONFIG, SOFTRST);
	while (!(musb_readl(musb->mregs, OTG_SYSSTATUS) & RESETDONE)) {
		if (get_timer(0) - start > 200) {
			ERR("musb reset timedout\n");
			break;
		}
	}

	/* Refer to the configurations of OMAP4 usb_otg_hs hwmod */
	/* configure in no standby, no idle, auto idle */
	musb_writel(musb->mregs, OTG_SYSCONFIG, (NOSTDBY | NOIDLE | AUTOIDLE));

	omap_musb_disable_all_clocks();
}

int __init musb_platform_init(struct musb *musb)
{
	struct musb_hw_ep *ep;
	int epnum;
	u32 l;
	int status = 0;

	/* allocate_instance() >> */
	musb->mregs = (void*)MUSB_BASE;
	musb->ctrl_base = (void*)MUSB_BASE;
	musb->config = &musb_config;
	BUG_ON(musb->config->num_eps > MUSB_C_NUM_EPS);
	for (epnum = 0, ep = musb->endpoints;
			epnum < musb->config->num_eps;
			epnum++, ep++) {
		ep->musb = musb;
		ep->epnum = epnum;
	}
	/* << allocate_instance() */

	/* musb_init_controller() >> */
	musb->board_mode = MUSB_PERIPHERAL;
	/* << musb_init_controller() */

	/* We require some kind of external transceiver, hooked
	 * up through ULPI.  TWL4030-family PMICs include one,
	 * which needs a driver, drivers aren't always needed.
	 */
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		INFO("no USB otg transceiver\n");
		return -ENODEV;
	}

	musb_reset(musb);

	/* Fixme this can be enabled when load the gadget driver also*/
	musb_platform_resume(musb);

	/** James Wu:
	 * In order to detect charger type correctly on OMAP4,
	 * OTG_INTERFSEL must be ULPI_12PIN.
	 * We should change to ULPI_12PIN after the charger detection is done.
	 */
	/* Config INFERFSEL for the correct charger detection */
	l = musb_readl(musb->mregs, OTG_INTERFSEL);
	l |= ULPI_12PIN;
	musb_writel(musb->mregs, OTG_INTERFSEL, l);

	DBG(1, "USB OTG: revision 0x%x, sysconfig 0x%02x,\n"
			"\tsysstatus 0x%x, intrfsel 0x%x, simenable 0x%x\n",
			musb_readl(musb->mregs, OTG_REVISION),
			musb_readl(musb->mregs, OTG_SYSCONFIG),
			musb_readl(musb->mregs, OTG_SYSSTATUS),
			musb_readl(musb->mregs, OTG_INTERFSEL),
			musb_readl(musb->mregs, OTG_SIMENABLE));

	musb->nb.notifier_call = musb_otg_notifications;
	musb->nb.priority = 65535;
	status = otg_register_notifier(musb->xceiv, &musb->nb);
	if (status)
		ERR("musb otg err %d\n", status);

	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	/* unregister for transciever notification*/
	otg_unregister_notifier(musb->xceiv, &musb->nb);

	musb_platform_suspend(musb);

	return 0;
}

void musb_platform_handle_interrupts(struct musb *musb)
{
#if defined(CONFIG_OMAP44XX)
#define OMAP44XX_GIC_INTR_BASE	0x48241000
#define		SPI_STATUS0		0xD04	/* SPI[31:0] */
#define		SPI_STATUS1		0xD08	/* SPI[63:32] */
#define		SPI_STATUS2		0xD0C	/* SPI[95:64] */
#define		SPI_STATUS3		0xD10	/* SPI[127:96] */

	u32 spi_status2 = __raw_readl(OMAP44XX_GIC_INTR_BASE + SPI_STATUS2);

	if (spi_status2 & 0x10000000) { /* MA_IRQ_92: HSUSB_OTG_IRQ */
		musb->isr(0, musb);
	}

#ifdef CONFIG_USB_INVENTRA_DMA
	if (spi_status2 & 0x20000000) { /* MA_IRQ_93: HSUSB_OTG_DMA_IRQ */
		dma_controller_irq(musb);
	}
#endif

#else

	/*static irqreturn_t generic_interrupt(int irq, void *__hci)*/
	musb_data.isr(0, musb);
#ifdef CONFIG_USB_INVENTRA_DMA
	if (musb_readb(musb->mregs, 0x200))
		dma_controller_irq(musb);
#endif /* CONFIG_USB_INVENTRA_DMA */

#endif
}
