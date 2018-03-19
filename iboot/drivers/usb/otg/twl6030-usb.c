/*
 * twl6030_usb - TWL6030 USB transceiver, talking to OMAP OTG driver
 *
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp., <www.innocomm.com>
 *
 * Author :
 *  James Wu <james.wu@innocomm.com>
 *
 * Some codes and ideas taken from
 * linux/drivers/usb/otg/twl6030-usb.c by Hema HK.
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
#include <exports.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/usb.h>
#include <linux/usb/otg.h>

#include <twl6030.h>

#include <icom/power_supply.h>

/*-------------------------------------------------------------------------*/

/*#define CONFIG_TWL6030_USB_DEBUG*/
/*#define CONFIG_TWL6030_USB_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL6030_USB_DEBUG
#define CONFIG_TWL6030_USB_DEBUG
#endif
#ifndef CONFIG_TWL6030_USB_VERBOSE_DEBUG
#define CONFIG_TWL6030_USB_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6030_USB_DEBUG
#define TWL_USB_DPRINT(fmt, args...) \
	do {printf("[twl-usb] " fmt, ##args);} while (0)
#define TWL_USB_DPUTS(fmt) \
	do {puts("[twl-usb] " fmt);} while (0)
#else /* CONFIG_TWL6030_USB_DEBUG */
#define TWL_USB_DPRINT(fmt, args...) \
	do {} while (0)
#define TWL_USB_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_USB_DEBUG */

#ifdef CONFIG_TWL6030_USB_VERBOSE_DEBUG
#define TWL_USB_VPRINT(fmt, args...) \
	do {printf("[twl-usb] " fmt, ##args);} while (0)
#define TWL_USB_VPUTS(fmt) \
	do {puts("[twl-usb] " fmt);} while (0)
#else /* CONFIG_TWL6030_USB_VERBOSE_DEBUG */
#define TWL_USB_VPRINT(fmt, args...) \
	do {} while (0)
#define TWL_USB_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6030_USB_VERBOSE_DEBUG */

#define TWL_USB_PRINT(fmt, args...) \
	do {printf("twl-usb: " fmt, ##args);} while (0)
#define TWL_USB_PUTS(fmt) \
	do {puts("twl-usb: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define TWL_USB_ERR(fmt, args...) \
	do {printf("twl-usb: " fmt, ##args);} while (0)

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_TWL6032_POWER
#define USB_REGULATOR	TWL6032_LDOUSB
#else /* CONFIG_TWL6030_POWER */
#define USB_REGULATOR	TWL6030_VUSB
#endif

/*-------------------------------------------------------------------------*/

/* usb register definitions */
#define USB_VENDOR_ID_LSB		0x00
#define USB_VENDOR_ID_MSB		0x01
#define USB_PRODUCT_ID_LSB		0x02
#define USB_PRODUCT_ID_MSB		0x03
#define USB_VBUS_CTRL_SET		0x04
#define USB_VBUS_CTRL_CLR		0x05
#define USB_ID_CTRL_SET			0x06
#define USB_ID_CTRL_CLR			0x07
#define USB_VBUS_INT_SRC		0x08
#define USB_VBUS_INT_LATCH_SET		0x09
#define USB_VBUS_INT_LATCH_CLR		0x0A
#define USB_VBUS_INT_EN_LO_SET		0x0B
#define USB_VBUS_INT_EN_LO_CLR		0x0C
#define USB_VBUS_INT_EN_HI_SET		0x0D
#define USB_VBUS_INT_EN_HI_CLR		0x0E
#define USB_ID_INT_SRC			0x0F
#define USB_ID_INT_LATCH_SET		0x10
#define USB_ID_INT_LATCH_CLR		0x11

#define USB_ID_INT_EN_LO_SET		0x12
#define USB_ID_INT_EN_LO_CLR		0x13
#define USB_ID_INT_EN_HI_SET		0x14
#define USB_ID_INT_EN_HI_CLR		0x15
#define USB_OTG_ADP_CTRL		0x16
#define USB_OTG_ADP_HIGH		0x17
#define USB_OTG_ADP_LOW			0x18
#define USB_OTG_ADP_RISE		0x19
#define USB_OTG_REVISION		0x1A

#define TWL6030_BACKUP_REG		0xFA
#define TWL6030_MISC2			0xE5
#define TWL6030_CFG_LDO_PD2		0xF5

/*#define STS_HW_CONDITIONS		0x21*/

/* In module TWL6030_MODULE_PM_MASTER */
#define STS_HW_CONDITIONS		0x21
#define		STS_USB_ID			BIT(2)

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module TWL6030_MODULE_MAIN_CHARGE */

#define CHARGERUSB_CTRL1		0x8

#define CONTROLLER_STAT1		0x03
#define		VAC_DET				BIT(3)
#define		VBUS_DET			BIT(2)

/*-------------------------------------------------------------------------*/

static struct twl6030_usb {
	struct otg_transceiver	otg;
	struct twl6030_usb_data *pdata;

	u8			linkstat;
	u8			asleep;
	u8			prev_vbus;
#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	u8			prev_vac;
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
#ifdef CONFIG_SYS_POWER_SUPPLY
	/* for notification of ac_charger_events */
	struct atomic_notifier_head	ac_charger_notifier;
#endif /* CONFIG_SYS_POWER_SUPPLY */
} twl_usb __attribute__ ((section (".data")));

#define xceiv_to_twl(x)		container_of((x), struct twl6030_usb, otg)

/*-------------------------------------------------------------------------*/

static int twl6030_writeb(u8 module, u8 data, u8 address)
{
#ifdef CONFIG_TWL6030_USB_DEBUG
	int ret = twl_i2c_write_u8(module, data, address);
	if (ret)
		TWL_USB_PRINT("write[0x%x,0x%x] err %d\n", module, address, ret);
	return ret;
#else
	return twl_i2c_write_u8(module, data, address);
#endif /* CONFIG_TWL6030_USB_DEBUG */
}

static u8 twl6030_readb(u8 module, u8 address)
{
	u8 data = 0;
#ifdef CONFIG_TWL6030_USB_DEBUG
	int ret = twl_i2c_read_u8(module, &data, address);
	if (ret)
		TWL_USB_PRINT("readb[0x%x,0x%x] err %d\n", module, address, ret);
#else
	twl_i2c_read_u8(module, &data, address);
#endif /* CONFIG_TWL6030_USB_DEBUG */
	return data;
}

/*-------------------------------------------------------------------------*/

static int twl6030_phy_init(struct otg_transceiver *x)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	struct twl6030_usb_data *pdata = twl->pdata;

	if (twl->linkstat == USB_EVENT_ID)
		pdata->phy_power(1, 1);
	else
		pdata->phy_power(0, 1);

	return 0;
}

static void twl6030_phy_shutdown(struct otg_transceiver *x)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	struct twl6030_usb_data *pdata = twl->pdata;

	pdata->phy_power(0, 0);

	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP, and VBUS_DISCHARGE bit */
	twl6030_writeb(TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);
}

#if 0
static int twl6030_phy_suspend(struct otg_transceiver *x, int suspend)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	struct twl6030_usb_data *pdata = twl->pdata;

	pdata->phy_suspend(suspend);

	return 0;
}
#endif

#if 0
static int twl6030_start_srp(struct otg_transceiver *x)
{
	/*struct twl6030_usb *twl = xceiv_to_twl(x);*/

	twl6030_writeb(TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);
	mdelay(50);
	twl6030_writeb(TWL_MODULE_USB, 0x20, USB_VBUS_CTRL_CLR);
	twl6030_writeb(TWL_MODULE_USB, 0x84, USB_VBUS_CTRL_SET);

	mdelay(100);
	twl6030_writeb(TWL_MODULE_USB, 0x80, USB_VBUS_CTRL_CLR);
	twl6030_writeb(TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);

	return 0;
}
#endif

static void twl6030_usb_ldo_init(void)
{
	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP, and VBUS_DISCHARGE bit */
	/* Sauz: If the discharge bit not set, the plug out detection fail some time. */
	twl6030_writeb(TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);

	/*
	 * Program the USB_ID_CTRL_SET register to enable GND drive
	 * and the ID comparators
	 */
	twl6030_writeb(TWL_MODULE_USB, 0x14, USB_ID_CTRL_SET);
}

static void twl6030_usb_irq_handler(struct otg_transceiver *x)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	u8 vbus_state, hw_state;
#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	u8 vac_state;
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
#if defined(CONFIG_OMAP44XX)
	u32 spi_status0 = 0;
#endif /* CONFIG_OMAP44XX */

#if defined(CONFIG_OMAP44XX)
#define OMAP44XX_GIC_INTR_BASE	0x48241000
#define		SPI_STATUS0		0xD04	/* SPI[31:0] */

#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	if (twl->prev_vbus == 0xFF || twl->prev_vac == 0xFF) {
		u32 int_sts = 0;

		twl_usb.prev_vbus = 0;
		twl_usb.prev_vac = 0;

		twl_i2c_read(TWL_MODULE_PIH, (u8 *)&int_sts, REG_INT_STS_A, 3);
		if (int_sts)
			spi_status0 = 0x080;
		else {
			vbus_state = twl6030_readb(TWL_MODULE_MAIN_CHARGE,
						CONTROLLER_STAT1);
			if (vbus_state & (VAC_DET | VBUS_DET))
				spi_status0 = 0x080;
		}
	}
#else
	if (twl->prev_vbus == 0xFF) {
		u32 int_sts = 0;

		twl_usb.prev_vbus = 0;

		twl_i2c_read(TWL_MODULE_PIH, (u8 *)&int_sts, REG_INT_STS_A, 3);
		if (int_sts)
			spi_status0 = 0x080;
		else {
			vbus_state = twl6030_readb(TWL_MODULE_MAIN_CHARGE,
						CONTROLLER_STAT1);
			if (vbus_state & VBUS_DET)
				spi_status0 = 0x080;
		}
	}
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	else
		spi_status0 = __raw_readl(OMAP44XX_GIC_INTR_BASE + SPI_STATUS0);

	if (spi_status0 & 0x080) { /* MA_IRQ_7: sys_nirq1 */
		twl6030_clear_interrupts();
	} else {
#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
		if (twl->prev_vbus || twl->prev_vac)
#else
		if (twl->prev_vbus)
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
			/* Poll to update the power supply status */
			power_supply_poll();

		return;
	}
#endif /* CONFIG_OMAP44XX */

	vbus_state = twl6030_readb(TWL_MODULE_MAIN_CHARGE,
						CONTROLLER_STAT1);
#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	vac_state = vbus_state & VAC_DET;
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
	vbus_state = vbus_state & VBUS_DET;

#if defined(CONFIG_TWL6032_CHARGER)
#ifdef CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER
	if (vac_state != twl->prev_vac) {
		int status;

		if (vac_state)
			status = POWER_SUPPLY_AC_EVENT_CHARGER;
		else
			status = POWER_SUPPLY_AC_EVENT_NONE;

		atomic_notifier_call_chain(&twl->ac_charger_notifier,
					status, NULL);

		twl->prev_vac = vac_state;
	}

	if (vbus_state || vac_state)
		/* Poll to update the power supply status */
		power_supply_poll();
#else
	if (vbus_state)
		/* Poll to update the power supply status */
		power_supply_poll();
#endif /* CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
#endif /* CONFIG_TWL6032_CHARGER */

	/* Ignore charger events other than VBUS */
	if (vbus_state == twl->prev_vbus)
		return;

	putc('\n');

	hw_state = twl6030_readb(TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	/* Ignore VBUS when in BOOST mode */
	if (!(hw_state & STS_USB_ID)) {
		int status;
		if (vbus_state) {
			unsigned charger_type;

			twl6030_usb_ldo_init();
			regulator_enable(USB_REGULATOR);

			charger_type = omap4_usb_charger_detect();
			if ((charger_type == POWER_SUPPLY_TYPE_USB_CDP)
					|| (charger_type == POWER_SUPPLY_TYPE_USB)) {
				status = USB_EVENT_VBUS;
				twl->otg.default_a = 0;
				twl->asleep = 1;
				twl->otg.state = OTG_STATE_B_IDLE;
				twl->linkstat = status;
				twl->otg.last_event = status;
			} else if (charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
				status = USB_EVENT_CHARGER;
				regulator_disable(USB_REGULATOR);
				twl->linkstat = status;
				twl->otg.last_event = status;
			} else if (charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
				/* If USB D+ & D- are not short, the USB type will be UNKNOWN */
#ifdef CONFIG_POWER_SUPPLY_TREAT_USB_CARKIT_AS_AC_CHARGER
				/* Treat the unknown USB charger as the USB AC charger */
				charger_type = POWER_SUPPLY_TYPE_USB_DCP;
				status = USB_EVENT_CHARGER;
				PUTS("usb: treat as AC charger\n");
#else
				/* Treat the unknown USB charger as the Accessory Charger Adapters */
				charger_type = POWER_SUPPLY_TYPE_USB_ACA;
				status = USB_EVENT_VBUS;
				PUTS("usb: treat as Accessory Charger\n");
#endif /* CONFIG_POWER_SUPPLY_TREAT_USB_CARKIT_AS_AC_CHARGER */
				regulator_disable(USB_REGULATOR);
				twl->linkstat = status;
				twl->otg.last_event = status;
			} else {
				regulator_disable(USB_REGULATOR);
				twl->prev_vbus = vbus_state;
				return;
			}
			atomic_notifier_call_chain(&twl->otg.notifier,
						status, &charger_type);
		} else {
			status = USB_EVENT_NONE;
			twl->linkstat = status;
			twl->otg.last_event = status;

			atomic_notifier_call_chain(&twl->otg.notifier,
						status, twl->otg.gadget);

			if (twl->asleep) {
				regulator_disable(USB_REGULATOR);
				twl->asleep = 0;
			}
		}
		twl->prev_vbus = vbus_state;
	}
}

static void twl6030_enable_irq(void)
{
	twl6030_interrupt_unmask(0x08 | TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(0x08 | TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);
}

static int twl6030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct twl6030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int twl6030_set_power(struct otg_transceiver *x, unsigned mA)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	unsigned int vbus_draw = mA;

	TWL_USB_DPRINT("set power: %u mA\n", mA);

	if (mA)
		atomic_notifier_call_chain(&twl->otg.notifier,
				USB_EVENT_ENUMERATED, &vbus_draw);

	return 0;
}

void twl6030_usb_init(struct twl6030_usb_data *pdata)
{
	memset(&twl_usb, 0, sizeof(twl_usb));

	twl_usb.pdata				= pdata;
#if 0
	twl_usb.otg.set_host		= twl6030_set_host;
	twl_usb.otg.set_vbus		= twl6030_set_vbus;
#endif
	twl_usb.otg.set_peripheral	= twl6030_set_peripheral;
	twl_usb.otg.init			= twl6030_phy_init;
	twl_usb.otg.set_power		= twl6030_set_power;
	twl_usb.otg.shutdown		= twl6030_phy_shutdown;
#if 0
	twl_usb.otg.set_suspend		= twl6030_phy_suspend;
	twl_usb.otg.start_srp		= twl6030_start_srp;
#endif
	twl_usb.otg.handle_irq		= twl6030_usb_irq_handler;
	twl_usb.otg.state			= OTG_STATE_UNDEFINED;

	regulator_disable(USB_REGULATOR);
	regulator_set_voltage(USB_REGULATOR, 3300);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	twl_clear_n_set(TWL6030_MODULE_ID0, 0x0, 0x1, TWL6030_CFG_LDO_PD2);

	/* Program MISC2 register and set bit VUSB_IN_VBAT */
	twl_clear_n_set(TWL6030_MODULE_ID0, 0x0, 0x10, TWL6030_MISC2);

	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP */
	twl6030_writeb(TWL6030_MODULE_ID0, 0x1, TWL6030_BACKUP_REG);

	twl6030_usb_ldo_init();

	otg_set_transceiver(&(twl_usb.otg));

	ATOMIC_INIT_NOTIFIER_HEAD(&(twl_usb.otg.notifier));
#ifdef CONFIG_SYS_POWER_SUPPLY
	ATOMIC_INIT_NOTIFIER_HEAD(&(twl_usb.ac_charger_notifier));
#endif /* CONFIG_SYS_POWER_SUPPLY */

	pdata->phy_init();

	twl6030_enable_irq();

	twl_usb.prev_vbus = 0xFF;
#if defined(CONFIG_TWL6032_CHARGER) && defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER)
	twl_usb.prev_vac = 0xFF;
#endif /* CONFIG_TWL6032_CHARGER && CONFIG_POWER_SUPPLY_HAS_EXTERNAL_AC_CHARGER */
}

void twl6030_usb_shutdown(void)
{
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK | TWL6030_CHARGER_CTRL_INT_MASK,
		REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK | TWL6030_CHARGER_CTRL_INT_MASK,
		REG_INT_MSK_STS_C);

	twl_usb.pdata->phy_exit();

	regulator_disable(USB_REGULATOR);
}

#ifdef CONFIG_SYS_POWER_SUPPLY
/* AC Charger notifiers */
int ac_charger_register_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&(twl_usb.ac_charger_notifier), nb);
}
#endif /* CONFIG_SYS_POWER_SUPPLY */
