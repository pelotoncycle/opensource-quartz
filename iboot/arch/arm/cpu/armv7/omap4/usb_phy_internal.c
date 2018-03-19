/*
 * This file configures the internal USB PHY in OMAP4.
 * Used with TWL6030 transceiver and MUSB on OMAP4.
 *
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp., <www.innocomm.com>
 *
 * Author :
 *  James Wu <james.wu@innocomm.com>
 *
 * Codes and ideas taken from
 * linux/arch/arm/mach-omap2/omap_phy_internal.c by Hema HK.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/omap_common.h>
#include <asm/arch/usb.h>
#include <asm/arch/clocks.h>

#include <icom/power_supply.h>

/*#define CONFIG_OMAP4_USBPHY_DEBUG*/
/*#define CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG*/

#if defined(CONFIG_USB_DEBUG)
#if (CONFIG_USB_DEBUG > 0)
#ifndef CONFIG_OMAP4_USBPHY_DEBUG
#define CONFIG_OMAP4_USBPHY_DEBUG
#endif
#endif /* CONFIG_USB_DEBUG > 0 */
#if (CONFIG_USB_DEBUG > 3)
#ifndef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
#define CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
#endif
#endif /* CONFIG_USB_DEBUG > 3 */
#endif /* CONFIG_USB_DEBUG */

#ifdef CONFIG_OMAP4_USBPHY_DEBUG
#define USBOTG_DPRINT(fmt, args...) \
	do {printf("USBOTG: " fmt, ##args);} while (0)
#define USBOTG_DPUTS(fmt) \
	do {puts("USBOTG: " fmt);} while (0)
#define USBPHY_DPRINT(fmt, args...) \
	do {printf("USBPHY: " fmt, ##args);} while (0)
#define USBPHY_DPUTS(fmt) \
	do {puts("USBPHY: " fmt);} while (0)
#else
#define USBOTG_DPRINT(fmt, args...) \
	do {} while (0)
#define USBOTG_DPUTS(fmt) \
	do {} while (0)
#define USBPHY_DPRINT(fmt, args...) \
	do {} while (0)
#define USBPHY_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_OMAP4_USBPHY_DEBUG */

#ifdef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
#define USBOTG_VPRINT(fmt, args...) \
	do {printf("USBOTG: " fmt, ##args);} while (0)
#define USBOTG_VPUTS(fmt) \
	do {puts("USBOTG: " fmt);} while (0)
#define USBPHY_VPRINT(fmt, args...) \
	do {printf("USBPHY: " fmt, ##args);} while (0)
#define USBPHY_VPUTS(fmt) \
	do {puts("USBPHY: " fmt);} while (0)
#else
#define USBOTG_VPRINT(fmt, args...) \
	do {} while (0)
#define USBOTG_VPUTS(fmt) \
	do {} while (0)
#define USBPHY_VPRINT(fmt, args...) \
	do {} while (0)
#define USBPHY_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG */

#define USBOTG_PRINT(fmt, args...) \
	do {printf("otg: " fmt, ##args);} while (0)
#define USBOTG_PUTS(fmt) \
	do {puts("otg: " fmt);} while (0)
#define USBPHY_PRINT(fmt, args...) \
	do {printf("usb: " fmt, ##args);} while (0)
#define USBPHY_PUTS(fmt) \
	do {puts("usb: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)

/* MUSB OTG */
#define MUSB_INTRTX			(MUSB_BASE + 0x02)	/* 16-bit */
#define MUSB_INTRRX			(MUSB_BASE + 0x04)
#define MUSB_INTRTXE		(MUSB_BASE + 0x06)
#define MUSB_INTRRXE		(MUSB_BASE + 0x08)
#define MUSB_INTRUSB		(MUSB_BASE + 0x0A)	/* 8 bit */
#define MUSB_INTRUSBE		(MUSB_BASE + 0x0B)	/* 8 bit */
#define MUSB_DEVCTL			(MUSB_BASE + 0x60)	/* 8 bit */
/* OTG_INTERFSEL */
#define OTG_INTERFSEL		(MUSB_BASE + 0x40c)
#define	OTG_PHYSEL_SHIFT	0	/* bit position */
#define	OTG_PHYSEL_MASK		(3 << OTG_PHYSEL_SHIFT)
#define	OTG_UTMI_8BIT		(0 << OTG_PHYSEL_SHIFT)
#define	OTG_ULPI_12PIN		(1 << OTG_PHYSEL_SHIFT)
#define	OTG_ULPI_8PIN		(2 << OTG_PHYSEL_SHIFT)

/* OMAP control module register for UTMI PHY */
/* CONTROL_DEV_CONF */
#define CONTROL_DEV_CONF	0x4A002300
#define PHY_PD				0x1

/* USBOTGHS_CONTROL */
#define USBOTGHS_CONTROL	0x4A00233C
#define	AVALID				(0x1 << 0)
#define	BVALID				(0x1 << 1)
#define	VBUSVALID			(0x1 << 2)
#define	SESSEND				(0x1 << 3)
#define	IDDIG				(0x1 << 4)

/* OCP2SCP_TIMING */
#define OCP2SCP_TIMING		0x4A0AD018
#define OCP2SCP_SYNC2_MASK	0x0F

/* USBPHY_PWR_CNTL */
#define USBPHY_PWR_CNTL		0x4A0AD098

/* USBPHY_CHRG_DET */
#define USBPHY_CHRG_DET		0x4A0AD094

/* CONTROL_USB2PHYCORE */
#define CONTROL_USB2PHYCORE		0x4A100620
#define USB2PHY_CHG_DET_STATUS_MASK		0x0E00000
#define USB2PHY_CHG_DET_STATUS_SHIFT	21
#define CHARGER_TYPE_WAITSTATE	0x0
#define CHARGER_TYPE_PS2		0x2
#define CHARGER_TYPE_DEDICATED	0x4
#define CHARGER_TYPE_HOST		0x5
#define CHARGER_TYPE_PC			0x6
#define USB2PHY_UTMIRESETDONE	(0x1 << 9)
#define USB2PHY_CHGDETECTED		(0x1 << 13)
#define USB2PHY_CHGDETDONE		(0x1 << 14)
#define USB2PHY_RESTARTCHGDET	(0x1 << 15)
#define USB2PHY_GPIOMODE		(0x1 << 29)
#define USB2PHY_DISCHGDET		(0x1 << 30)

static int otg_use_count = 0;
static int phy_use_count = 0;

static inline void omap4_usb_otg_clk_enable(void)
{
	/* Enable the USBOTG clocks */

	/* Disable usb_otg_hs_xclk & otg_60m_gfclk */
	clrbits_le32(&prcm->cm_l3init_hsusbotg_clkctrl,
			USBOTG_CLKCTRL_CLKSEL_60M_MASK | USBOTG_CLKCTRL_OPTFCLKEN_XCLK_MASK);

	/* Enable usb_otg_hs_ick */
	omap_clk_enable(&prcm->cm_l3init_hsusbotg_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_HW_AUTO);
}

static void omap4_usb_otg_clk_disable(void)
{
	/* Disable the USBOTG clocks */

	/* Disable usb_otg_hs_xclk & otg_60m_gfclk */
	clrbits_le32(&prcm->cm_l3init_hsusbotg_clkctrl,
			USBOTG_CLKCTRL_CLKSEL_60M_MASK |
			USBOTG_CLKCTRL_OPTFCLKEN_XCLK_MASK);

	/* Disable usb_otg_hs_ick */
	omap_clk_disable(&prcm->cm_l3init_hsusbotg_clkctrl);
}

void omap4_usb_otg_set_clk(int on)
{
	if (on) {
		if (otg_use_count == 0) {
			/* Enable the USBOTG clocks */
			omap4_usb_otg_clk_enable();
		}
		++otg_use_count;
#ifdef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
		if (otg_use_count == 1)
			USBOTG_VPRINT("clk enabled\n");
		else
			USBOTG_VPRINT("clk already enabled %d\n", otg_use_count);
#endif /* CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG */
	} else {
		if (otg_use_count == 1) {
			/* Disable the USBOTG clocks */
			omap4_usb_otg_clk_disable();
		}
		--otg_use_count;
		if (otg_use_count < 0) {
			USBOTG_DPRINT("unbalanced clk %d\n", otg_use_count);
			otg_use_count = 0;
		}
#ifdef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
		else if (otg_use_count > 0)
			USBOTG_VPRINT("clk still enabled %d\n", otg_use_count);
		else
			USBOTG_VPRINT("clk disabled\n");
#endif /* CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG */
	}
}

static void omap4_usb_phy_set_clk(int on)
{
	if (on) {
		/* Enable the phy clocks */

		/* ocp2scp_usb_phy_ick */
		omap_clk_enable(&prcm->cm_l3init_usbphy_clkctrl,
				MODULE_CLKCTRL_MODULEMODE_HW_AUTO);

		/* Enable optional 48M functional clock for USB PHY */
		/* ocp2scp_usb_phy_phy_48m */
		setbits_le32(&prcm->cm_l3init_usbphy_clkctrl,
				USBPHY_CLKCTRL_OPTFCLKEN_PHY_48M_MASK);

		/* Enable optional 32K functional clock for USB PHY */
		/* usb_phy_cm_clk32k */
		setbits_le32(&prcm->cm_alwon_usbphy_clkctrl,
				USBPHY_CLKCTRL_OPTFCLKEN_CLK32K_MASK);
	} else {
		/* Disable the phy clocks */

		/* Disable optional 32K functional clock for USB PHY */
		/* usb_phy_cm_clk32k */
		clrbits_le32(&prcm->cm_alwon_usbphy_clkctrl,
				USBPHY_CLKCTRL_OPTFCLKEN_CLK32K_MASK);

		/* Disable optional 48M functional clock for USB PHY */
		/* ocp2scp_usb_phy_phy_48m */
		clrbits_le32(&prcm->cm_l3init_usbphy_clkctrl,
				USBPHY_CLKCTRL_OPTFCLKEN_PHY_48M_MASK);

		/* ocp2scp_usb_phy_ick */
		omap_clk_disable(&prcm->cm_l3init_usbphy_clkctrl);
	}
}

static void omap4_usb_phy_set_power(int on)
{
	if (on) {
		if (phy_use_count == 0) {
			unsigned long timeout, ticks = 0;
			u32 usb2phycore;

			/* Enabled the clocks */
			omap4_usb_phy_set_clk(1);

			/* Power on the phy */
			__raw_writel(~PHY_PD, CONTROL_DEV_CONF);

			timeout = timeout_init(200/*ms*/, &ticks);
			do {
				usb2phycore = __raw_readl(CONTROL_USB2PHYCORE);
				/* Check UTMI FSM reset status */
				if (usb2phycore & USB2PHY_UTMIRESETDONE)
					break;
			} while (!timeout_check(&timeout, &ticks));

#ifndef CONFIG_OMAP4_HSOTG_ED_CORRECTION
			/** OMAP4 TRM:
			 * For a correct read out of the USBPHY registers, the value of the
			 * OCP2SCP_TIMING[3:0] SYNC2 bit field must be set to 0x6 or more, and the
			 * OCP2SCP_TIMING[9:7] DIVISIONRATIO bit field must be left untouched.
			 */
			clrsetbits_le32(OCP2SCP_TIMING, OCP2SCP_SYNC2_MASK, 0x06);
#endif /* !CONFIG_OMAP4_HSOTG_ED_CORRECTION */
		}

		++phy_use_count;
#ifdef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
		if (phy_use_count == 1)
			USBPHY_VPRINT("clk enabled\n");
		else
			USBPHY_VPRINT("clk already enabled %d\n", phy_use_count);
#endif /* CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG */
	} else {
		if (phy_use_count == 1) {
			/* Disable the clocks */
			omap4_usb_phy_set_clk(0);

			/* Power down the phy */
			__raw_writel(PHY_PD, CONTROL_DEV_CONF);
		}
		--phy_use_count;
		if (phy_use_count < 0) {
			USBPHY_DPRINT("unbalanced clk %d\n", phy_use_count);
			phy_use_count = 0;
		}
#ifdef CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG
		else if (phy_use_count > 0)
			USBPHY_VPRINT("clk still enabled %d\n", phy_use_count);
		else
			USBPHY_VPRINT("clk disabled\n");
#endif /* CONFIG_OMAP4_USBPHY_VERBOSE_DEBUG */
	}
}

void omap4_usb_phy_init(void)
{
	/* Enable the USBPHY clocks */
	omap4_usb_phy_set_clk(1);

	/* Enable session END and IDIG to high impedance. */
	__raw_writel(SESSEND | IDDIG, USBOTGHS_CONTROL);

	/* Disable charger detection by default */
	setbits_le32(CONTROL_USB2PHYCORE, USB2PHY_DISCHGDET);

	/* Disable the USBOTG clocks */
	omap4_usb_otg_clk_disable();

	/* Disable the USBPHY clocks */
	omap4_usb_phy_set_clk(0);

	/* Power down the phy */
	__raw_writel(PHY_PD, CONTROL_DEV_CONF);
}

void omap4_usb_phy_exit(void)
{
	u32 val;

	/* Enable the USBPHY clocks */
	omap4_usb_phy_set_clk(1);

	/* Enable the USBOTG clocks */
	omap4_usb_otg_clk_enable();

	/**
	 * In order to detect charger type correctly on OMAP4, OTG_INTERFSEL must be ULPI_12PIN.
	 * We should change to ULPI_12PIN before jumping to Linux kernel.
	 */
	clrsetbits_le32(OTG_INTERFSEL, OTG_PHYSEL_MASK, OTG_ULPI_12PIN);

	/* Enable session END and IDIG to high impedance. */
	__raw_writel(SESSEND | IDDIG, USBOTGHS_CONTROL);

	/* Disable charger detection by default */
	setbits_le32(CONTROL_USB2PHYCORE, USB2PHY_DISCHGDET);

	/* disable USBOTG interrupts */
	__raw_writeb(0, MUSB_INTRUSBE);
	__raw_writew(0, MUSB_INTRTXE);
	__raw_writew(0, MUSB_INTRRXE);

	/* USBOTG off */
	__raw_writeb(0, MUSB_DEVCTL);

	/* flush USBOTG pending interrupts */
	val = __raw_readb(MUSB_INTRUSB);
	val = __raw_readw(MUSB_INTRTX);
	val = __raw_readw(MUSB_INTRRX);

	/* Disable the USBOTG clocks */
	omap4_usb_otg_clk_disable();

	/* Disable the USBPHY clocks */
	omap4_usb_phy_set_clk(0);

	/* Power down the phy */
	__raw_writel(PHY_PD, CONTROL_DEV_CONF);

	USBPHY_VPRINT("power down\n");
}

#ifdef CONFIG_OMAP4_HSOTG_ED_CORRECTION
#define OMAP4_HSOTG_SWTRIM_MASK		0xFFFF00FF
#define OMAP4_HSOTG_REF_GEN_TEST_MASK	0xF8FFFFFF
static void omap44xx_hsotg_ed_correction(void)
{
	u32 omap4_rev = omap_revision();
	u32 val;

	/*
	 * Software workaround #1
	 * By this way we improve HS OTG
	 * eye diagramm by 2-3%
	 * Allow this change for all OMAP4 family
	 */

	/*
	 * For prevent 4-bit shift issue
	 * bit field SYNC2 of OCP2SCP_TIMING
	 * should be set to value >6
	 */

	val = __raw_readl(MUSB_BASE + 0x2018);
	val |= 0x0F;
	__raw_writel(val, MUSB_BASE + 0x2018);

	/*
	 * USBPHY_ANA_CONFIG2[16:15] = RTERM_TEST = 11b
	 */
	val = __raw_readl(MUSB_BASE + 0x20D4);
	val |= (3<<15);
	__raw_writel(val, MUSB_BASE + 0x20D4);

	/*
	 * USBPHY_TERMINATION_CONTROL[13:11] = HS_CODE_SEL = 011b
	 */
	val = __raw_readl(MUSB_BASE + 0x2080);
	val &= ~(7<<11);
	val |= (3<<11);
	__raw_writel(val, MUSB_BASE + 0x2080);

	/*
	 * Software workaround #2
	 * Reducing interface output impedance
	 * By this way we improve HS OTG eye diagramm by 8%
	 * This change needed only for 4430 CPUs
	 * because this change can impact Rx performance
	 */

	/*
	 * Increase SWCAP trim code by 0x24
	 * NOTE: Value should be between 0 and 0x24
	 */
	val = __raw_readl(MUSB_BASE + 0x20B8);
	if (omap4_rev < OMAP4460_ES1_0 /*OMAP4430*/ && !(val & 0x8000)) {
		val = min((val + (0x24<<8)), (val | (0x7F<<8))) | 0x8000;
		__raw_writel(val, MUSB_BASE + 0x20B8);
	}

	/*
	 * For 4460 and 4470 CPUs there is 10-15mV adjustable
	 * improvement available via REF_GEN_TEST[26:24]=110
	 */
	if (omap4_rev >= OMAP4460_ES1_0 /*OMAP4460 || OMAP4470*/) {
		val = __raw_readl(MUSB_BASE + 0x20D4);
		val &= OMAP4_HSOTG_REF_GEN_TEST_MASK;
		val |= (0x6<<24);
		__raw_writel(val, MUSB_BASE + 0x20D4);
	}
}
#endif

void omap4_usb_phy_power(int ID, int on)
{
	if (on) {
		omap4_usb_phy_set_power(1);

#ifdef CONFIG_OMAP4_HSOTG_ED_CORRECTION
		/* apply eye diagram improvement settings */
		omap44xx_hsotg_ed_correction();
#endif

		if (ID)
			/* Enable VBUS valid, IDDIG groung */
			__raw_writel(AVALID | VBUSVALID, USBOTGHS_CONTROL);
		else
			/*
			 * Enable VBUS Valid, AValid and IDDIG
			 * high impedance
			 */
			__raw_writel(IDDIG | AVALID | VBUSVALID, USBOTGHS_CONTROL);
	} else {
		/* Enable session END and IDIG to high impedance. */
		__raw_writel(SESSEND | IDDIG, USBOTGHS_CONTROL);

		omap4_usb_phy_set_power(0);
	}
}

int omap4_usb_charger_detect(void)
{
	int charger = -1;
	u32 usb2phycore = 0;
	u32 chargertype = 0;
	unsigned long timeout, ticks = 0;
	u32 otg_interfsel;

	omap4_usb_otg_set_clk(1);
	/* Backup the current OTG_INTERFSEL */
	otg_interfsel = __raw_readl(OTG_INTERFSEL);
	if (!(otg_interfsel & OTG_ULPI_12PIN))
		USBOTG_PRINT("not ULPI\n");
	/* Config OTG_INTERFSEL for the correct charger detection */
	clrsetbits_le32(OTG_INTERFSEL, OTG_PHYSEL_MASK, OTG_ULPI_12PIN);

	/* Enabled USB phy */
	omap4_usb_phy_power(0, 1);

	/* Enable charger detection and restart it */
	clrsetbits_le32(CONTROL_USB2PHYCORE,
		/* clear */ USB2PHY_DISCHGDET,
		/* set   */ USB2PHY_RESTARTCHGDET);
	mdelay(2);
	clrbits_le32(CONTROL_USB2PHYCORE, USB2PHY_RESTARTCHGDET);

	timeout = timeout_init(500/*ms*/, &ticks);
	do {
		usb2phycore = __raw_readl(CONTROL_USB2PHYCORE);
		/* Charger detection done? */
		if (usb2phycore & USB2PHY_CHGDETDONE)
			break;
	} while (!timeout_check(&timeout, &ticks));

	chargertype = (usb2phycore & USB2PHY_CHG_DET_STATUS_MASK) >> USB2PHY_CHG_DET_STATUS_SHIFT;

#if defined(CONFIG_USB_DEBUG) && (CONFIG_USB_DEBUG > 0)
	if (!timeout && chargertype != CHARGER_TYPE_PC) {
		USBPHY_DPUTS("charger detection timeout!\n");
	}
#endif /* CONFIG_USB_DEBUG */

	switch (chargertype) {
	case CHARGER_TYPE_DEDICATED:
		charger = POWER_SUPPLY_TYPE_USB_DCP;
		PUTS("usb: AC charger\n");
		break;
	case CHARGER_TYPE_HOST:
		charger = POWER_SUPPLY_TYPE_USB_CDP;
		PUTS("usb: HOST charger\n");
		break;
	case CHARGER_TYPE_PC:
		charger = POWER_SUPPLY_TYPE_USB;
		PUTS("usb: PC charger\n");
		break;
	case CHARGER_TYPE_PS2:
		charger = POWER_SUPPLY_TYPE_UNKNOWN;
		PUTS("usb: PS/2 charger\n");
		break;
	default:
		charger = POWER_SUPPLY_TYPE_UNKNOWN;
		PRINT("usb: unknown charger %d\n", chargertype);
		break;
	}

	setbits_le32(CONTROL_USB2PHYCORE, USB2PHY_RESTARTCHGDET);
	mdelay(2);

	/* Disable USB phy */
	omap4_usb_phy_power(0, 0);

	/* Restore the current OTG_INTERFSEL */
	__raw_writel(otg_interfsel, OTG_INTERFSEL);
	omap4_usb_otg_set_clk(0);

	return charger;
}
