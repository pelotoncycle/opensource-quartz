/*
 * Board support file for OMAP44xx Quartz.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Author: 
 * Dan Murphy 	<dmurphy@ti.com>
 * John.Lin 	<John.Lin@innocomm.com>
 *
 * Based on mach-omap2/board-44xx-quartz.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/max3218.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#include <linux/mfd/twl6040-codec.h>
#include <mach/dmm.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/omap4_ion.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/remoteproc.h>
#include <plat/omap-pm.h>
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "pm.h"
#include "board-44xx-quartz.h"
#include "omap_ram_console.h"

#define WILINK_UART_DEV_NAME	"/dev/ttyO1"

#define GPIO_WIFI_PMENA		54
#define GPIO_WIFI_IRQ		53

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
#include <linux/clk.h>
#include "clock.h"
#endif

#define GPIO_USB3320_PHY_RESETB	    40
#define GPIO_USB3320_PHY_REF_CLOCK	60

#define GPIO_USB_HUB_RESETB	    50

static int quartz_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data quartz_keymap_data = {
	.keymap			= quartz_keymap,
	.keymap_size		= ARRAY_SIZE(quartz_keymap),
};

static struct omap4_keypad_platform_data quartz_keypad_data = {
	.keymap_data		= &quartz_keymap_data,
	.rows			= 1,
	.cols			= 2,

};

/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is
 * capable enough to wake-up the OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 55,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device max3218_platform_device = {
	.name		= QUARTZ_MAX3218_DRIVER_NAME,
	.id		= -1,
};

#ifdef CONFIG_USB_HUB_PORT_PLATFORM_RESET
static struct platform_device *usb_touch_pdev;
#endif

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
static int quartz_ts_power_init(struct platform_device *pdev)
{
	struct quartz_ts_pdata *ts_pdata = pdev->dev.platform_data;

	dev_info(&pdev->dev, "%s()\n", __func__);

#ifdef CONFIG_USB_HUB_PORT_PLATFORM_RESET
	usb_touch_pdev = pdev;
#endif

	ts_pdata->reg_v5v = regulator_get(&pdev->dev, "vtp_v5v");
	if (IS_ERR(ts_pdata->reg_v5v)) {		
		ts_pdata->reg_v5v = NULL;
		dev_info(&pdev->dev, "%s() reg_v5v get error!!\n", __func__);
		return -1;
	}

	ts_pdata->reg_vtp5v = regulator_get(&pdev->dev, "vtp5v");
	if (IS_ERR(ts_pdata->reg_vtp5v)) {		
		ts_pdata->reg_vtp5v = NULL;
		dev_info(&pdev->dev, "%s() reg_vtp5v get error!!\n", __func__);
		return -1;
	}
	
	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
static int quartz_ts_power_free(struct platform_device *pdev)
{
	struct quartz_ts_pdata *ts_pdata = pdev->dev.platform_data;

	dev_info(&pdev->dev, "%s()\n", __func__);

#ifdef CONFIG_USB_HUB_PORT_PLATFORM_RESET
	usb_touch_pdev = NULL;
#endif

	if (ts_pdata->reg_vtp5v) {
		regulator_put(ts_pdata->reg_vtp5v);
		ts_pdata->reg_vtp5v = NULL;
	}

	if (ts_pdata->reg_v5v) {
		regulator_put(ts_pdata->reg_v5v);
		ts_pdata->reg_v5v = NULL;
	}	
	
	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
static int quartz_ts_power_enable(struct platform_device *pdev)
{
	int ret = -1;
	struct quartz_ts_pdata *ts_pdata = pdev->dev.platform_data;

	// enable V5V & VTP_5V
	dev_info(&pdev->dev, "%s()\n", __func__);

	if (ts_pdata->reg_v5v) {
		if (!ts_pdata->onoff_v5v) {
			ret = regulator_enable(ts_pdata->reg_v5v);
			if (ret < 0) {
				dev_info(&pdev->dev, "%s() reg_v5v enable error!!\n", __func__);
				return ret;
			}
			ts_pdata->onoff_v5v = true;
		}
	} else {
		dev_info(&pdev->dev, "%s() reg_v5v NULL!!\n", __func__);
	}
		

	if (ts_pdata->reg_vtp5v) {
		if (!ts_pdata->onoff_vtp5v) {
			ret = regulator_enable(ts_pdata->reg_vtp5v);
			if (ret < 0) {
				dev_info(&pdev->dev, "%s() reg_vtp5v enable error!!\n", __func__);
				return ret;
			}
			ts_pdata->onoff_vtp5v= true;
		}
	} else {
		dev_info(&pdev->dev, "%s() reg_vtp5v NULL!!\n", __func__);
	}
	
	return ret;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
static int quartz_ts_power_disable(struct platform_device *pdev)
{
	int ret = -1;
	struct quartz_ts_pdata *ts_pdata = pdev->dev.platform_data;

	// disable V5V & VTP_5V
	dev_info(&pdev->dev, "%s()\n", __func__);

	if (ts_pdata->reg_vtp5v) {
		if (ts_pdata->onoff_vtp5v) {
			ret = regulator_disable(ts_pdata->reg_vtp5v);
			if (ret < 0) {
				dev_info(&pdev->dev, "%s() reg_vtp5v disable error!!\n", __func__);
				return ret;
			}
			ts_pdata->onoff_vtp5v= false;
		}
	} else {
		dev_info(&pdev->dev, "%s() reg_vtp5v NULL!!\n", __func__);
	}

	if (ts_pdata->reg_v5v) {
		if (ts_pdata->onoff_v5v) {
			ret = regulator_disable(ts_pdata->reg_v5v);
			if (ret < 0) {
				dev_info(&pdev->dev, "%s() reg_v5v disable error!!\n", __func__);
				return ret;
			}
			ts_pdata->onoff_v5v = false;
		}
	} else {
		dev_info(&pdev->dev, "%s() reg_v5v NULL!!\n", __func__);
	}

	return ret;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_QUARTZ)
static struct quartz_ts_pdata ts_pdata = {
	.power_init = quartz_ts_power_init,
	.power_free = quartz_ts_power_free,
	.power_enable = quartz_ts_power_enable,
	.power_disable = quartz_ts_power_disable,
	.reg_v5v = NULL,
	.reg_vtp5v = NULL,
	.onoff_v5v = false,
	.onoff_vtp5v = false,
};

static struct platform_device quartz_ts_device = {
	.name	= QUARTZ_TOUCH_NAME,
	.id		= -1,
	.dev.platform_data = &ts_pdata,
};
#endif

static struct omap_board_config_kernel quartz_config[] __initdata = {
};

void __init omap_quartz_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.no_off_init	= true,
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= 0x001FFF80, /* 1.65V~3.3V */
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply omap4_quartz_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data quartz_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_quartz_vmmc5_supply,
};

static struct fixed_voltage_config quartz_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= GPIO_WIFI_PMENA,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &quartz_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &quartz_vwlan,
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	/* Setting MMC5 SDIO card .built-in variable
	  * This is to make sure that if WiFi driver is not loaded
	  * at all, then the MMC/SD/SDIO driver does not keep
	  * turning on/off the voltage to the SDIO card
	  */
	if (pdev->id == 4) {
		ret = 0;
		pdata->slots[0].mmc_data.built_in = 1;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
}

static void __init quartz_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}


static void __init quartz_cam_clk_init(void)
{
	omap_writew(omap_readw(CONTROL_CORE_PAD1_FREF_CLK1_OUT) &
		    ~(OMAP44XX_PADCONF_INPUTENABLE0 |
		      OMAP44XX_PADCONF_PULLUDENABLE0 |
		      OMAP44XX_PADCONF_MUXMODE_MASK0),
		    CONTROL_CORE_PAD1_FREF_CLK1_OUT);
#if 0
   	omap_writew(omap_readw(CONTROL_CORE_PAD1_FREF_CLK2_OUT) &
		    ~(OMAP44XX_PADCONF_INPUTENABLE0 |
		      OMAP44XX_PADCONF_PULLUDENABLE0 |
		      OMAP44XX_PADCONF_MUXMODE_MASK0),
		    CONTROL_CORE_PAD1_FREF_CLK2_OUT);	

        omap_mux_init_signal("csi21_dx2.csi21_dx2", OMAP_MUX_MODE0 |
						OMAP_INPUT_EN);
        omap_mux_init_signal("csi21_dy2.csi21_dy2", OMAP_MUX_MODE0 |
						OMAP_INPUT_EN);	
#endif
}


static struct omap_i2c_bus_board_data __initdata quartz_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata quartz_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata quartz_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata quartz_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &quartz_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &quartz_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &quartz_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &quartz_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &quartz_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &quartz_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &quartz_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &quartz_i2c_4_bus_pdata);

	quartz_twl_init();
	
	//init i2c devices(touch, sensors .....,etc.)
	quartz_i2c_init();

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}


static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

/*#ifdef CONFIG_OMAP_MUX*/
#if 0
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
					| OMAP_PULL_ENA),

	/* IO optimization pdpu and offmode settings to reduce leakage */
	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
	OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
					| OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#define board_wkup_mux	NULL
#endif

#if !defined(CONFIG_OMAP4_EMIF_LPDDR2_AUTO_SETUP)
/*
 * LPDDR2 Configuration Data for 4430/4460 SOMs:
 * The memory organization is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
};

/*
 * LPDDR2 Configuration Data for 4470 SOMs:
 * The memory organization is as below :
 *	EMIF1 - CS0 -	4 Gb
 *	EMIF2 - CS0 -	4 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices_4470 = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
};
#endif /* !CONFIG_OMAP4_EMIF_LPDDR2_AUTO_SETUP */

/* Relation to sysfs entry /sys/wl12xx/wl_enable */
static ssize_t omap4_quartz_wlan_power_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	int wlan_state;

	if (sscanf(buf, "%du", &wlan_state) != 1) {
		printk("Set Wl_EN fail!\n");
		return -EINVAL;
	}

	printk("Set Wl_EN, wlen_state:%d\n",wlan_state);
	gpio_set_value(GPIO_WIFI_PMENA, wlan_state);
	return count;
}

static ssize_t omap4_quartz_wlan_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int wlan_state;

	wlan_state = gpio_get_value(GPIO_WIFI_PMENA);
	printk("Get WL_EN status:%d\n", wlan_state);
	return sprintf(buf, "%d\n", wlan_state);
}

static struct kobject *wl_kobj;
static struct kobj_attribute wl_attr = __ATTR(wl_enable, 0660, omap4_quartz_wlan_status_show, omap4_quartz_wlan_power_store);

static struct attribute *attrs[] = {
	&wl_attr.attr,
	NULL,
};

static struct attribute_group attr_grp = {
	.attrs = attrs,
};

static void omap4_quartz_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data omap4_quartz_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

//static void omap4_quartz_wifi_init(void)
static void __init omap4_quartz_wifi_init(void)
{
	omap4_quartz_wifi_mux_init();
	if (wl12xx_set_platform_data(&omap4_quartz_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);

	/* Create an entry /sys/wl12xx/wl_enable */
	wl_kobj = kobject_create_and_add("wl12xx", NULL);
	if (!wl_kobj) {
		pr_err("Create sysfs node /sys/wl12xx/wl_enable error.\n");
		return;
	}

	if (sysfs_create_group(wl_kobj, &attr_grp))
		kobject_put(wl_kobj);
}

#ifdef CONFIG_USB_HUB_PORT_PLATFORM_RESET
#include <linux/wakelock.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#ifdef CONFIG_MACH_OMAP_USB_ANT
extern void quartz_antplus_set_power(int enable);
#endif
static void quartz_usb_antplus_reset_workfunc(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_USB_ANT
	quartz_antplus_set_power(1);
#endif
	pr_info("%s: done\n", __func__);
}
static struct wake_lock usb_antplus_reset_wakelock;
DECLARE_DELAYED_WORK(usb_antplus_reset_work, quartz_usb_antplus_reset_workfunc);

#ifdef CONFIG_USB_NET_AX88XXX
extern int asix_init_set_power(bool on);
#endif
static void quartz_usb_eth_reset_workfunc(struct work_struct *work)
{
#ifdef CONFIG_USB_NET_AX88XXX
	asix_init_set_power(true);
#endif
	pr_info("%s: done\n", __func__);
}
static struct wake_lock usb_eth_reset_wakelock;
DECLARE_DELAYED_WORK(usb_eth_reset_work, quartz_usb_eth_reset_workfunc);

static void quartz_usb_touch_reset_workfunc(struct work_struct *work)
{
#ifdef CONFIG_TOUCHSCREEN_QUARTZ
	if (usb_touch_pdev)
		quartz_ts_power_enable(usb_touch_pdev);
#endif
	pr_info("%s: done\n", __func__);
}
static struct wake_lock usb_touch_reset_wakelock;
DECLARE_DELAYED_WORK(usb_touch_reset_work, quartz_usb_touch_reset_workfunc);

void usb_hub_port_platform_reset(struct usb_device *hdev, struct device *hub_dev, int port1)
{
	dev_info(hub_dev, "port %d: try the unplug->plug simulation...\n", port1);

	if (port1 == 1) {
		/* ANT USBStick2 */

		/* Hold a 30-sec wakelock to avoid the immediate Linux PM suspend */
		wake_lock_timeout(&usb_antplus_reset_wakelock, 30 * HZ);

#ifdef CONFIG_MACH_OMAP_USB_ANT
		quartz_antplus_set_power(0);
#endif

		schedule_delayed_work_on(0, &usb_antplus_reset_work, msecs_to_jiffies(300));
	} else if (port1 == 3) {
		/* eGalaxTouch EXC7200-75D2v1.002 USB touch */

		/* Hold a 30-sec wakelock to avoid the immediate Linux PM suspend */
		wake_lock_timeout(&usb_touch_reset_wakelock, 30 * HZ);

#ifdef CONFIG_TOUCHSCREEN_QUARTZ
		if (usb_touch_pdev)
			quartz_ts_power_disable(usb_touch_pdev);
		else
			pr_err("%s: no usb_touch_pdev\n", __func__);
#endif

		schedule_delayed_work_on(0, &usb_touch_reset_work, msecs_to_jiffies(300));
	} else if (port1 == 4) {
		/* AX88772B USB ethernet */

		/* Hold a 30-sec wakelock to avoid the immediate Linux PM suspend */
		wake_lock_timeout(&usb_eth_reset_wakelock, 30 * HZ);

#ifdef CONFIG_USB_NET_AX88XXX
		asix_init_set_power(false);
#endif

		schedule_delayed_work_on(0, &usb_eth_reset_work, msecs_to_jiffies(300));
	}
}
EXPORT_SYMBOL_GPL(usb_hub_port_platform_reset);
#endif

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static int phy_clk_enable(struct clk *clk)
{
	pr_info("enable usb phy clk\n");

	if (system_rev == 0) /* EP1 */
		gpio_set_value(GPIO_USB3320_PHY_REF_CLOCK, 1);

	return 0;
}

static void phy_clk_disable(struct clk *clk)
{
	pr_info("disable usb phy clk\n");
	if (system_rev == 0) /* EP1 */
		gpio_set_value(GPIO_USB3320_PHY_REF_CLOCK, 0);
}

static const struct clkops clkops_phy = {
	.enable		= phy_clk_enable,
	.disable	= phy_clk_disable,
};

static struct clk phy_ck = {
	.name		= "PHY_38_4M",
	.rate		= 38400000,
	.ops		= &clkops_phy,
};

static const struct usbhs_omap_board_data quartz_usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = true,
	.hub_reset  = true,
	.hub_reset_gpio_port = GPIO_USB_HUB_RESETB,
	.reset_gpio_port[0]  = GPIO_USB3320_PHY_RESETB,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
	.transceiver_clk[0] = &phy_ck,
};

static void __init omap4_ehci_ohci_init(void)
{
	/* configure the EHCI PHY USB3320C RESET GPIO */
	omap_mux_init_gpio(GPIO_USB3320_PHY_RESETB, OMAP_PIN_OUTPUT |
		OMAP_PIN_OFF_NONE);

	if (system_rev == 0) /* EP1 */ {
		/* Initial USB Phy 38.4Mhz GPIO control */
		omap_mux_init_gpio(GPIO_USB3320_PHY_REF_CLOCK, OMAP_PIN_OUTPUT);
		gpio_request(GPIO_USB3320_PHY_REF_CLOCK, "usbphy_refclk");
		gpio_direction_output(GPIO_USB3320_PHY_REF_CLOCK, 0);
	}

	/* Enable here to avoid warning "Trying disable clock PHY_38_4M with 0 usecount" */
	clk_enable(quartz_usbhs_bdata.transceiver_clk[0]);

	usbhs_init(&quartz_usbhs_bdata);

#ifdef CONFIG_USB_HUB_PORT_PLATFORM_RESET
	wake_lock_init(&usb_antplus_reset_wakelock, WAKE_LOCK_SUSPEND, "ant+_usb_rst");
	wake_lock_init(&usb_eth_reset_wakelock, WAKE_LOCK_SUSPEND, "eth_usb_rst");
	wake_lock_init(&usb_touch_reset_wakelock, WAKE_LOCK_SUSPEND, "touch_usb_rst");
#endif
}
#else
static void __init omap4_ehci_ohci_init(void)
{
	omap_mux_init_gpio(GPIO_USB3320_PHY_RESETB, OMAP_PIN_OUTPUT |
		OMAP_PIN_OFF_NONE);

	if (gpio_request(GPIO_USB3320_PHY_RESETB, "USB1_PHY_RESET")) {
		printk(KERN_ERR "Could not request GPIO %d\n", GPIO_USB3320_PHY_RESETB);
		return;
	}

	gpio_direction_output(GPIO_USB3320_PHY_RESETB, 0);

	if (system_rev == 0) /* EP1 */ {
		omap_mux_init_gpio(GPIO_USB3320_PHY_REF_CLOCK, OMAP_PIN_OUTPUT);
		if (gpio_request(GPIO_USB3320_PHY_REF_CLOCK, "usbphy_refclk")) {
			printk(KERN_ERR "Could not request GPIO %d\n", GPIO_USB3320_PHY_REF_CLOCK);
			return;
		}
		gpio_direction_output(GPIO_USB3320_PHY_REF_CLOCK, 0);
	}
}
#endif

#ifdef CONFIG_EMERGENCY_SHUTDOWN
#include <linux/emergencyshutdown.h>

#define EMERGENCY_SHUTDOWN_IRQ_GPIO	7

static int emergency_shutdown_init(int *irq_no, unsigned long *irq_flags)
{
#if 0
	omap_mux_init_gpio(EMERGENCY_SHUTDOWN_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif

	if (gpio_request(EMERGENCY_SHUTDOWN_IRQ_GPIO, "VAC_INT") < 0) {
		pr_err(EMERGENCY_SHUTDOWN_DRV_NAME ": cannot request gpio %d\n", EMERGENCY_SHUTDOWN_IRQ_GPIO);
		WARN_ON(1);
	}
	gpio_direction_input(EMERGENCY_SHUTDOWN_IRQ_GPIO);

	*irq_no = OMAP_GPIO_IRQ(EMERGENCY_SHUTDOWN_IRQ_GPIO);
	*irq_flags = IRQF_TRIGGER_FALLING | IRQF_DISABLED | IRQF_ONESHOT;

	return 0;
}

#ifdef CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ
void omap_emergency_shutdown_max_cpufreq(unsigned int max_freq);
#endif /* CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ */

static void emergency_shutdown_shutdown(void)
{
#if 0
	printk(KERN_EMERG EMERGENCY_SHUTDOWN_DRV_NAME ": platform shutting down\n");
#endif

	/* turn off V5V */
	gpio_set_value(41, 0);

	/* turn off LCM backlight */
	gpio_set_value(61, 1);

	/* turn off LCM_5V */
	gpio_set_value(51, 1);

	/* turn off VTP_5V */
	gpio_set_value(37, 1);

	/* turn off wifi */
	gpio_set_value(54, 0);

	/* turn off bluetooth */
	gpio_set_value(55, 0);

	/* turn off ethernet */
	gpio_set_value(36, 0);
	gpio_set_value(33, 1);

	/* turn off Audio AMP */
	gpio_set_value(62, 0);

	/* turn off CAM_2V8 */
	gpio_set_value(42, 0);

	/* turn off RS232 */
	gpio_set_value(56, 0);
	gpio_set_value(59, 0);

	/* turn off twl6032 LDOs by the special orders */
	/* LDO7 (V_CAM_1V8) */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0xA6); /* LDO7_CFG_STATE */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x01, 0xA5); /* LDO7_CFG_TRANS */
	/* LDO4 (VNET_D3V3) */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0x8A); /* LDO4_CFG_STATE */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x01, 0x89); /* LDO4_CFG_TRANS */
#if 0
	/* LDO3 (V_LVDS_3V3) */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0x8E); /* LDO3_CFG_STATE */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x01, 0x8D); /* LDO3_CFG_TRANS */
#endif

#ifdef CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ
	omap_emergency_shutdown_max_cpufreq(396800); /* 396.8MHz */
#endif /* CONFIG_EMERGENCY_SHUTDOWN_LOCK_CPU_MAX_FREQ */
}

//static struct emergency_shutdown_platform_data emergency_shutdown_data __initdata = {
static struct emergency_shutdown_platform_data emergency_shutdown_data = {
	.platform_init	= emergency_shutdown_init,
	.platform_shutdown	= emergency_shutdown_shutdown,
};

static struct platform_device emergency_shutdown_device = {
	.name	= EMERGENCY_SHUTDOWN_DRV_NAME,
	.id		= -1,
	.dev = {
		.platform_data = &emergency_shutdown_data,
	},
};
#endif /* CONFIG_EMERGENCY_SHUTDOWN */

static struct platform_device *quartz_devices[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_QUARTZ)	
	&quartz_ts_device,
#endif
	&wl128x_device,
	&btwilink_device,
	&max3218_platform_device,
#ifdef CONFIG_EMERGENCY_SHUTDOWN
	&emergency_shutdown_device,
#endif /* CONFIG_EMERGENCY_SHUTDOWN */
};

static void quartz_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}
static struct omap_pad_state_description quartz_dynamic_pads[] __initdata = {
/*> I2C 2,3,4 Followed Boot loader config, 
the controlled devices power will be switched off, 
the related I2C pads need pull down prevent leakage*/	
	{
		.suspend_mux_name	= "i2c2_scl.gpio_128",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
	{
		.suspend_mux_name	= "i2c2_sda.gpio_129",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
	{
		.suspend_mux_name	= "i2c3_scl.gpio_130",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
	{
		.suspend_mux_name	= "i2c3_sda.gpio_131",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
	{
		.suspend_mux_name	= "i2c4_scl.gpio_132",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
	{
		.suspend_mux_name	= "i2c4_sda.gpio_133",
		.suspend_config	= OMAP_PIN_OUTPUT,
		.flags	= OMAP_DEVICE_PAD_SUSPEND_GPIO_LOW,
	},
/*< I2C 2,3,4 */


	{
		.init_mux_name	= "gpmc_ncs4.safe_mode",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	{
		.init_mux_name	= "gpmc_ncs5.safe_mode",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	{
		.init_mux_name	= "gpmc_ncs6.safe_mode",
		.init_config	= OMAP_PIN_OUTPUT,
	},

	{
		.init_mux_name	= "hdq_sio.gpio_127",
		.init_config	= OMAP_PIN_OUTPUT,
	},

	/* TP_RESET  */
	{
		.init_mux_name	= "gpmc_ad8.gpio_32",
		.init_config	= OMAP_PIN_OUTPUT,
		.suspend_mux_name	= "gpmc_ad8.gpio_32",
		.suspend_config	= OMAP_PIN_INPUT_PULLDOWN,
	},

	/* ETHERNET_EN */
	{
		.init_mux_name	= "gpmc_ad9.gpio_33",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* ETHERNET_PME */
	{
		.init_mux_name	= "gpmc_ad10.gpio_34",
		.init_config	= OMAP_PIN_INPUT_PULLDOWN,
	},
	/* ETHERNET_RST  */
	{
		.init_mux_name	= "gpmc_ad12.gpio_36",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* VTP_EN_N */
	{
		.init_mux_name	= "gpmc_ad13.gpio_37",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* MAIN_CAM_nSTANDBY  */
	{
		.init_mux_name	= "gpmc_ad14.gpio_38",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/*TEMP_IRQ*/
	{
		.init_mux_name	= "gpmc_ad15.gpio_39",
		.init_config	= OMAP_PIN_INPUT,
	},
	/* M_USBB1_RST  */
	{
		.init_mux_name	= "gpmc_a16.gpio_40",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* V5V_EN  */
	{
		.init_mux_name	= "gpmc_a17.gpio_41",
		.init_config	= OMAP_PIN_OUTPUT,
	},

	/* CAM_2V8_EN */
	{
		.init_mux_name	= "gpmc_a18.gpio_42",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* LVDS_RST */
	{
		.init_mux_name	= "gpmc_a19.gpio_43",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* LVDS_1V2_EN unused */
	{
		.init_mux_name	= "gpmc_a20.safe_mode",
		.init_config	= OMAP_PIN_INPUT,
	},
	/* LVDS_1V8_EN unused */
	{
		.init_mux_name	= "gpmc_a21.safe_mode",
		.init_config	= OMAP_PIN_INPUT,
	},
	/*HUB_RST_N gpmc_ncs0(gpio_50)*/
	{
		.init_mux_name	= "gpmc_ncs0.gpio_50",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/*LCM_5V_EN_N gpmc_ncs1(gpio_51) */
	{
		.init_mux_name	= "gpmc_ncs1.gpio_51",
		.init_config	= OMAP_PIN_OUTPUT,
	},

	/* TP_INT  */
	{
		.init_mux_name	= "gpmc_ncs2.gpio_52",
		.init_config	= OMAP_PIN_INPUT_PULLUP,
		.suspend_mux_name	= "gpmc_ncs2.gpio_52",
		.suspend_config	= OMAP_PIN_INPUT_PULLDOWN,
	},
	/*WLAN_IRQ#  */
	{
		.init_mux_name	= "gpmc_ncs3.gpio_53",
		.init_config	= OMAP_PIN_INPUT_PULLUP,
	},	
	/*WL_EN  */
	{
		.init_mux_name	= "gpmc_nwp.gpio_54",
		.init_config	= OMAP_PIN_OUTPUT,
	},	
	/*BT_EN  */
	{
		.init_mux_name	= "gpmc_clk.gpio_55",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/*MODEM_CLK_EN  */
	{
		.init_mux_name	= "gpmc_nbe1.gpio_60",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/* BL_EN_N */
	{
		.init_mux_name	= "gpmc_wait0.gpio_61",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/*AUDIO_EX_AMP_EN  */
	{
		.init_mux_name	= "gpmc_wait1.gpio_62",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	{
		.init_mux_name	= "abe_dmic_clk1.abe_dmic_clk1",
		.init_config	= OMAP_PIN_OUTPUT,
	},		
	{
		.init_mux_name	= "abe_dmic_din1.abe_dmic_din1",
		.init_config	= OMAP_PIN_INPUT,
	},
	{
		.init_mux_name	= "usbb2_ulpitll_dat0.gpio_161",
		.init_config	= OMAP_PIN_OUTPUT,
	},
	/*NC */
	{
		.init_mux_name	= "fref_clk3_req.safe_mode",
#if 0
		.init_config	= OMAP_PIN_INPUT_PULLDOWN,
#else
		.init_config	= OMAP_PIN_OUTPUT,
#endif
	},
	{
#ifdef CONFIG_EMERGENCY_SHUTDOWN
		.init_mux_name	= "fref_clk4_req.gpio_wk7",
		.init_config	= OMAP_PIN_INPUT_PULLUP,
#else
		.init_mux_name	= "fref_clk4_req.gpio_wk7",
#if 0
		.init_config	= OMAP_PIN_INPUT_PULLDOWN,
#else
		.init_config	= OMAP_PIN_OUTPUT,
#endif
#endif /* CONFIG_EMERGENCY_SHUTDOWN */
	},

};

#ifdef CONFIG_MACH_OMAP_USB_ANT
#include "board-ant-ldo.h"

struct platform_device ant_platform_device = {
	.name = ANT_USB_DRIVER_NAME,
	.id = -1,
};
#endif

void __init omap_quartz_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;
	int quartz_rev = 0;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);
	omap_pads_manager_init(quartz_dynamic_pads,ARRAY_SIZE(quartz_dynamic_pads));

#ifdef CONFIG_OMAP4_EMIF_LPDDR2_AUTO_SETUP
	lpddr2_omap4_emif_setup();
#else
	if (cpu_is_omap447x())
		omap_emif_setup_device_details(&emif_devices_4470,
					       &emif_devices_4470);
	else
		omap_emif_setup_device_details(&emif_devices, &emif_devices);
#endif /* CONFIG_OMAP4_EMIF_LPDDR2_AUTO_SETUP */

	omap_board_config = quartz_config;
	omap_board_config_size = ARRAY_SIZE(quartz_config);
	quartz_rev = omap_init_board_version(0);
	omap4_create_board_props();
	omap4_audio_conf();

	quartz_cam_clk_init();

	quartz_ldo_init();
	omap4_i2c_init();

	status = omap4_keyboard_init(&quartz_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);

	omap_dmm_init();
	quartz_panel_init();
	quartz_pmic_mux_init();
	quartz_set_osc_timings();
	omap4_register_ion();
	quartz_serial_init();
	omap4_quartz_wifi_init();
	omap4_twl6030_hsmmc_init(mmc);
#ifdef CONFIG_MACH_OMAP_USB_ANT
	if (system_rev != 0) /* not EP1 */ {
		status = platform_device_register(&ant_platform_device);
		if (status)
			pr_err("ANT initialization err: %d\n", status);
	}
#endif
	platform_add_devices(quartz_devices,
			ARRAY_SIZE(quartz_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	omap4_ehci_ohci_init();
	//quartz_modem_init();
	usb_musb_init(&musb_board_data);	
	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();

}

void __init omap_quartz_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

void __init omap_quartz_reserve(void)
{
	omap_init_ram_size();
#ifdef CONFIG_ION_OMAP
	quartz_android_display_setup(get_omap_ion_platform_data());
	omap_ion_init();
#else
	quartz_android_display_setup(NULL);
#endif

#ifdef CONFIG_ION_OMAP
	omap_ram_console_init(omap_ram_console_mem_addr(),
			omap_ram_console_mem_size());
#else
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);
#endif

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_QUARTZ, "OMAP4 Quartz")
	/* Maintainer: Dan Murphy - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_quartz_reserve,
	.map_io		= omap_quartz_map_io,
	.init_early	= omap_quartz_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_quartz_init,
	.timer		= &omap_timer,
MACHINE_END

