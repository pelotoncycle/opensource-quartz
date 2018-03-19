/*
 * linux/drivers/power/twl6030_bci_battery.c
 *
 * OMAP4:TWL6030/TWL6032 battery driver for Linux
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Copyright (C) 2011 Innocomm Mobile Technology Corp.
 *
 * Code re-written by:
 * Innocomm Mobile Technology Corp.
 *
 * Initial Code:
 * Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/twl6032_bci_battery.h>
//#include <linux/i2c/bq2415x.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/mach-types.h>
#ifdef POWEROFF_CHARGE
#include <linux/android_boot.h> /*0004597: [FR] [PM] implement power off charger*/
#endif
#include <linux/i2c/bq27520.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define SUPPORT_FUNC (BCI_RESET_TIMER|BCI_START_STOP_CHARGING|BCI_ENABLE_POWER_PATH|\
		BCI_ENABLE_EOC|BCI_SET_CHARGE_CURRENT|BCI_SET_EOC_CURRENT|\
		BCI_SET_INPUT_CURRENT_LIMIT|BCI_READ_FAULT_REASON|\
		BCI_CHAEGER_STATUS_REQUEST)

#define LINEAR_CHRG_STS_CRYSTL_OSC_OK		0x40
#define LINEAR_CHRG_STS_END_OF_CHARGE		0x20
#define LINEAR_CHRG_STS_VBATOV		0x10
#define LINEAR_CHRG_STS_VSYSOV		0x08
#define LINEAR_CHRG_STS_DPPM_STS		0x04
#define LINEAR_CHRG_STS_CV_STS		0x02
#define LINEAR_CHRG_STS_CC_STS		0x01

/*TWL 6032 CONTROLLER_CTRL2*/
#define EN_DPPM		(1 << 1)
#define SUP_MASK	(1 << 0)
/*TWL 6032 CHARGERUSB_VSYSREG*/
#define VSYS_SW_CTRL (1<<7)
/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 7)
#define MAC_EOC			(1 << 6)
#define LINCH_GATED		(1 << 5)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)
/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_LINCH	(1 << 5)
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)
/* CONTROLLER_STAT1 */
#define CONTROLLER_STAT1_EXTCHRG_STATZ	(1 << 7)
#define CONTROLLER_STAT1_LINCH_GATED	(1 << 6)
#define CONTROLLER_STAT1_CHRG_DET_N	(1 << 5)
#define CONTROLLER_STAT1_FAULT_WDG	(1 << 4)
#define CONTROLLER_STAT1_VAC_DET	(1 << 3)
#define VAC_DET	(1 << 3)
#define CONTROLLER_STAT1_VBUS_DET	(1 << 2)
#define VBUS_DET	(1 << 2)
#define CONTROLLER_STAT1_BAT_REMOVED	(1 << 1)
#define CONTROLLER_STAT1_BAT_TEMP_OVRANGE (1 << 0)
/* CHARGERUSB_INT_STATUS */
#define CURRENT_TERM_INT	(1 << 3)
#define CHARGERUSB_STAT		(1 << 2)
#define CHARGERUSB_THMREG	(1 << 1)
#define CHARGERUSB_FAULT	(1 << 0)
/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)
/* CHARGERUSB_STATUS_INT1 */
#define CHARGERUSB_STATUS_INT1_TMREG	(1 << 7)
#define CHARGERUSB_STATUS_INT1_NO_BAT	(1 << 6)
#define CHARGERUSB_STATUS_INT1_BST_OCP	(1 << 5)
#define CHARGERUSB_STATUS_INT1_TH_SHUTD	(1 << 4)
#define CHARGERUSB_STATUS_INT1_BAT_OVP	(1 << 3)
#define CHARGERUSB_STATUS_INT1_POOR_SRC	(1 << 2)
#define CHARGERUSB_STATUS_INT1_SLP_MODE	(1 << 1)
#define CHARGERUSB_STATUS_INT1_VBUS_OVP	(1 << 0)
/* CHARGERUSB_STATUS_INT2 */
#define ICCLOOP		(1 << 3)
#define CURRENT_TERM	(1 << 2)
#define CHARGE_DONE	(1 << 1)
#define ANTICOLLAPSE	(1 << 0)
/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT	(1 << 7)
#define OPA_MODE	(1 << 6)
#define HZ_MODE		(1 << 5)
#define TERM		(1 << 4)
/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400	(7 << 5)
/* CHARGERUSB_CTRL3 */
#define VBUSCHRG_LDO_OVRD	(1 << 7)
#define CHARGE_ONCE		(1 << 6)
#define BST_HW_PR_DIS		(1 << 5)
#define AUTOSUPPLY		(1 << 3)
#define BUCK_HSILIM		(1 << 0)
/* CHARGERUSB_VOREG */
#define CHARGERUSB_VOREG_3P52		0x01
#define CHARGERUSB_VOREG_4P0		0x19
#define CHARGERUSB_VOREG_4P2		0x23
#define CHARGERUSB_VOREG_4P76		0x3F

/* CHARGERUSB_VICHRG */
#define CHARGERUSB_VICHRG_300		0x0
#define CHARGERUSB_VICHRG_500		0x4
#define CHARGERUSB_VICHRG_1500		0xE

/* CHARGERUSB_CINLIMIT */
#define CHARGERUSB_CIN_LIMIT_100	0x1
#define CHARGERUSB_CIN_LIMIT_300	0x5
#define CHARGERUSB_CIN_LIMIT_500	0x9
#define CHARGERUSB_CIN_LIMIT_NONE	0xF

/* CHARGERUSB_CTRLLIMIT1 */
#define VOREGL_4P16			0x21
#define VOREGL_4P56			0x35

/* CHARGERUSB_CTRLLIMIT2 */
#define CHARGERUSB_CTRLLIMIT2_1500	0x0E
#define		LOCK_LIMIT		(1 << 4)

/* ANTICOLLAPSE_CTRL2 */
#define BUCK_VTH_SHIFT			5


#define PWDNSTATUS1		0x93
#define PWDNSTATUS2		0x94

#define REG_MISC1		0xE4
#define VAC_MEAS		0x04
#define VBAT_MEAS		0x02
#define BB_MEAS			0x01

#define REG_USB_VBUS_CTRL_SET	0x04
#define VBUS_MEAS		0x01
#define REG_USB_ID_CTRL_SET	0x06
#define ID_MEAS			0x01

#define BBSPOR_CFG			0xE6
#define		BB_CHG_EN		(1 << 3)

#define STS_HW_CONDITIONS	0x21
#define STS_USB_ID		(1 << 2)	/* Level status of USB ID */

#define VSYSMIN_HI_THRESHOLD	0x24

#define MAX_CURRENT 1800 /*0004712: [FR] [PM] porting bq2416x & register event modification*/


struct twl6030_bci_charger_info {
    struct device		*dev;

    int				charge_status;
    u8				linear_chg_prev_state;
    u8				watchdog_duration;
    unsigned int		min_vbus;
    unsigned int		max_charger_voltagemV;
    unsigned int		max_charger_currentmA;
    unsigned int		charger_incurrentmA;
    unsigned int		charger_outcurrentmA;
    unsigned int		charger_targetcurrentmA;
    unsigned int		regulation_voltagemV;
    unsigned int		low_bat_voltagemV;
    unsigned int		termination_currentmA;
    unsigned int		use_linear_charger;
    unsigned int		power_path;
    struct notifier_block	nb;
    unsigned long		features;
    int current_resistance;


};
const char* str_charge_state[]={
	"unknown ",
	"charging",
	"discharge",
	"n_charge",
	"full",
};
static void twl6030_kick_watchdog(struct twl6030_bci_charger_info *ci )
{

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, ci ->watchdog_duration , CONTROLLER_WDG);

}
static void twl6030_set_vbat_tracking_voltage(struct twl6030_bci_charger_info *ci ,int vol)
{
    u8 value;

    vol=(vol-50)/50;
    if (vol>3)vol=3;
    if (vol<0)vol=0;
    twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_VSEL_COMP);
    value&=~(3<<5);
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value|(vol<<5),CONTROLLER_VSEL_COMP);


}
static void twl6030_enable_dppm(struct twl6030_bci_charger_info *ci ,int enable)
{
    u8 value;


    twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_CTRL2);
    if (enable)value|=(EN_DPPM);
    else  value&=~(EN_DPPM);
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value,CONTROLLER_CTRL2);


}

static void twl6030_set_supplement_threshold(struct twl6030_bci_charger_info *ci ,int voltage)
{
    u8 value;

    twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value,	CONTROLLER_CTRL2);
    if (voltage>=0)value|=(SUP_MASK);
    else  value&=~(SUP_MASK);
    switch (voltage) {
    case 20:
        value&=~(0x03<<4);
        break;
    case 30:
        value&=~(0x03<<4);
        value|=(0x01<4);
        break;
    case 40:
        value&=~(0x03<<4);
        value|=(0x02<4);
        break;
    case 50:
        value&=~(0x03<<4);
        value|=(0x03<4);
        break;


    }
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value,CONTROLLER_CTRL2);

}


static void twl6032_set_tracking_mode_vsys_threshold(struct twl6030_bci_charger_info *ci ,int voltage)
{
    u8 value=0;
    if ((voltage<3500)||(voltage>4760))value&=~(VSYS_SW_CTRL);
    else  value|=(VSYS_SW_CTRL);

    value|=((voltage-3500)/20);

    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, value,CHARGERUSB_VSYSREG);


}


static void twl6030_config_min_vbus_reg(struct twl6030_bci_charger_info *ci ,
                                        unsigned int value)
{
    u8 rd_reg = 0;
    if (value > 4760 || value < 4200) {
        bci_dbg(debug_common, "invalid min vbus\n");
        return;
    }

    /* not required on TWL6032 */
    if (ci ->features & TWL6032_SUBCLASS)
        return;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, ANTICOLLAPSE_CTRL2);
    rd_reg = rd_reg & 0x1F;
    rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg, ANTICOLLAPSE_CTRL2);

    return;
}

static void twl6030_set_eoc_current(struct twl6030_bci_charger_info *ci ,
                                     unsigned int term_currentmA)
{
	if(!term_currentmA){
		/*turn off eoc*/
		u8 value;

		 twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CHARGERUSB_CTRL1);
		 twl_i2c_write_u8(TWL6030_MODULE_CHARGER, (value&(~TERM)),
                     CHARGERUSB_CTRL1);		
		term_currentmA=50;

	}else if (term_currentmA > 400) {
		term_currentmA=400;
        bci_dbg(debug_common, "invalid termination current,set to %d mA\n",term_currentmA);
     
    }else if (term_currentmA&&(term_currentmA < 50)){
		term_currentmA=50;
		 bci_dbg(debug_common, "invalid termination current,set to %d mA\n",term_currentmA);
    }
    term_currentmA = ((term_currentmA - 50)/50) << 5;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
                     CHARGERUSB_CTRL2);
    return;
}

static unsigned int twl6030_get_iterm_reg(struct twl6030_bci_charger_info *ci )
{
    unsigned int currentmA;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRL2);
    currentmA = 50 + (val >> 5) * 50;

    return currentmA;
}

static void twl6030_config_voreg_reg(struct twl6030_bci_charger_info *ci ,
                                     unsigned int voltagemV)
{
    if ((voltagemV < 3500) || (voltagemV > 4760)) {
        bci_dbg(debug_common, "invalid charger_voltagemV\n");
        return;
    }

    voltagemV = (voltagemV - 3500) / 20;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
                     CHARGERUSB_VOREG);
    return;
}

static unsigned int twl6030_get_voreg_reg(struct twl6030_bci_charger_info *ci )
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_VOREG);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}

static void twl6030_config_vichrg_reg(struct twl6030_bci_charger_info *ci ,
                                      unsigned int currentmA)
{
//    printk("set charging current as %d \n", currentmA);
    if (ci ->current_resistance==20) {
    } else if (ci ->current_resistance==15) {
        bci_dbg(debug_int_charger, "%s() -charging current for resistance 15 is %d\n", __func__, currentmA);
        currentmA = currentmA*15/20; //let currentmA * 15/20, to easy register mapping
        bci_dbg(debug_int_charger, "after *(15/20) is %d \n", currentmA);
    }

    if (ci ->use_linear_charger) {//pop=1?
        if (currentmA<100)currentmA=100;
        if (currentmA>1500)currentmA=1500;
        currentmA = (currentmA - 100) / 100;
    } else {//pop=0?
        if ((currentmA >= 300) && (currentmA <= 450))
            currentmA = (currentmA - 300) / 50;
        else if ((currentmA >= 500) && (currentmA <= 1500))
            currentmA = (currentmA - 500) / 100 + 4;
        else {
            bci_dbg(debug_common, "invalid charger current %d mA\n",currentmA);
            return;
        }
    }

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_VICHRG);
    return;
}

static const int vichrg[] = {
    300, 350, 400, 450, 500, 600, 700, 800,
    900, 1000, 1100, 1200, 1300, 1400, 1500, 300
};
#if 0
static void twl6030_vsysmin_hi_threshold(struct twl6030_bci_charger_info *ci ,
        unsigned int voltagemV)
{
    //u8 vsysmin;
    if ((voltagemV < 2050) || (voltagemV > 4600)) {
        bci_dbg(debug_common, "invalid vsysmin_voltagemV\n");
        return;
    }
#if 0 //original
    twl_i2c_read_u8(TWL6030_MODULE_ID0, &vsysmin, VSYSMIN_HI_THRESHOLD);
    printk("!! VSYSMIN_HI_THRESHOLD = 0x%02x \n", vsysmin);
#endif


    voltagemV = (voltagemV - 2000) / 50;
    twl_i2c_write_u8(TWL6030_MODULE_ID0, voltagemV,
                     VSYSMIN_HI_THRESHOLD);

#if 0 //new
    twl_i2c_read_u8(TWL6030_MODULE_ID0, &vsysmin, VSYSMIN_HI_THRESHOLD);
    printk("!! VSYSMIN_HI_THRESHOLD = 0x%02x \n", vsysmin);
#endif

    return;
}
#endif
static void twl6030_set_ci_limit(struct twl6030_bci_charger_info *ci ,
                                 unsigned int currentmA)
{
    //bci_dbg(debug_common, "set input current as %d \n", currentmA);

    if ((currentmA >= 50) && (currentmA <= 750))
        currentmA = (currentmA - 50) / 50;
    else if (currentmA >= 750) {
        if (ci ->features & TWL6032_SUBCLASS) {
            if (currentmA>2250)currentmA=0x1F;
            else if (currentmA==2250)currentmA=0x22;
            else  if (currentmA>2100)currentmA=0x21;
            else  if (currentmA>1800)currentmA=0x20;
            else  if (currentmA>1500)currentmA=0x2E;
            else  currentmA = (currentmA % 100) ? 0x30 : 0x20 + (currentmA - 100) / 100;
        } else
            currentmA = (800 - 50) / 50;
    } else {
        bci_dbg(debug_common, "invalid input current limit\n");
        return;
    }

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_CINLIMIT);
    return;
}

static void twl6030_set_vo_hard_limit(struct twl6030_bci_charger_info *ci ,
                                      unsigned int voltagemV)
{
    if ((voltagemV < 3500) || (voltagemV > 4760)) {
        bci_dbg(debug_common, "invalid max_charger_voltagemV\n");
        return;
    }

    voltagemV = (voltagemV - 3500) / 20;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
                     CHARGERUSB_CTRLLIMIT1);
    return;
}

static unsigned int twl6030_get_vo_hard_limit(struct twl6030_bci_charger_info *ci )
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT1);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}

static void twl6030_set_co_hard_limit(struct twl6030_bci_charger_info *ci ,
                                      unsigned int currentmA)
{
    if (ci ->current_resistance==15) {
        bci_dbg(debug_common, "%s() -input current for resistance 15 is %d\n", __func__, currentmA);
        currentmA = currentmA*15/20; //let currentmA * 15/20, to easy register mapping
        bci_dbg(debug_common, "after *(15/20) is %d \n", currentmA);
    }

#if 1
    if (currentmA<100)currentmA=100;
    if (currentmA>1500)currentmA=1500;
    currentmA = (currentmA - 100) / 100;
#else
    if ((currentmA >= 300) && (currentmA <= 450))
        currentmA = (currentmA - 300) / 50;
    else if ((currentmA >= 500) && (currentmA <= 1500))
        currentmA = (currentmA - 500) / 100 + 4;
    else {
        bci_dbg(debug_common, "invalid max_charger_currentmA\n");
        return;
    }
#endif
    currentmA |= LOCK_LIMIT;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
                     CHARGERUSB_CTRLLIMIT2);
    return;
}

static unsigned int twl6030_get_co_hard_limit(struct twl6030_bci_charger_info *ci )
{
    unsigned int currentmA;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT2);
    currentmA = vichrg[val & 0xF];

    return currentmA;
}
#if 0
static void twl6030_set_Q2_Q3_limit(struct twl6030_bci_charger_info *ci ,int currentmA)
{

    u8 value;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CHARGERUSB_CTRL3);

    switch (currentmA) {

    case 2550:
        value&=~(0x03);
        value|=0x02;
        break;
    case 1900:
        value&=~(0x03);
        value|=0x03;
        break;
    case 3700:
        value&=~(0x03);
        value|=0x00;
        break;
    case 3150:
        value&=~(0x03);
        value|=0x01;
        break;


    }
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, value, CHARGERUSB_CTRL3);

    return ;
}
#endif

static int twl6030_get_gpadc_conversion(struct twl6030_bci_charger_info *ci,
					int channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0)
		return ret;

	if (req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no];

	return temp;
}

 static void twl6032_stop_linear_charger(struct twl6030_bci_charger_info *ci )
{
    u8 value;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);
    value&=~CONTROLLER_CTRL1_EN_LINCH;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,value,
                     CONTROLLER_CTRL1);

}
 static void twl6032_start_linear_charger(struct twl6030_bci_charger_info *ci )
{
    u8 value;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);

	if(CONTROLLER_CTRL1_SEL_CHARGER&value){
		
		dev_warn(ci ->dev, 
			"Start twl6032 internal linear charger, but external charger enable was selected \n");
	}
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
		CONTROLLER_CTRL1_EN_CHARGER|CONTROLLER_CTRL1_EN_LINCH,
                     CONTROLLER_CTRL1);

}
static void twl6032_enable_power_path(struct twl6030_bci_charger_info *ci )
{
    u8 value;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);

	if(CONTROLLER_CTRL1_SEL_CHARGER&value){
		
		dev_warn(ci ->dev, 
			"Enable Power Path, but external charger enable was selected \n");
	}
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
		CONTROLLER_CTRL1_EN_CHARGER, 
		CONTROLLER_CTRL1);

}
static void twl6032_disable_power_path(struct twl6030_bci_charger_info *di)
{
   u8 value;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);

	if(CONTROLLER_CTRL1_SEL_CHARGER&value){
		
		dev_warn(di->dev, 
			"Disable Power Path, but external charger enable was selected \n");
		return;
	}
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
		0, 
		CONTROLLER_CTRL1);

 
}
  static void twl6030_stop_internal_charger(struct twl6030_bci_charger_info *ci )
{
    u8 value;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value, CONTROLLER_CTRL1);
	if(CONTROLLER_CTRL1_SEL_CHARGER&value)return;

    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,0, CONTROLLER_CTRL1);

}

static void twl6030_start_internal_charger(struct twl6030_bci_charger_info *ci )
{
    u8 value;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER,&value,
                    CONTROLLER_CTRL1);

	if(CONTROLLER_CTRL1_SEL_CHARGER&value){
		
		dev_warn(ci ->dev, 
			"Start twl6030 internal linear charger, but external charger enable was selected \n");
	}
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,CONTROLLER_CTRL1_EN_CHARGER, 
		CONTROLLER_CTRL1);

}
static void twl6032_charger_poll_func(struct twl6030_bci_charger_info *ci )
{

    u8 charge_state, linear_state;
    int prev_charge_status=ci->charge_status;
    int err;

    if (ci->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        return ;
    }

    err = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                          CONTROLLER_STAT1);

    err = twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &linear_state,
                          LINEAR_CHRG_STS);

    if (!(charge_state & (VBUS_DET | VAC_DET))) {
        ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else if (linear_state &
               (LINEAR_CHRG_STS_CC_STS | LINEAR_CHRG_STS_CV_STS|LINEAR_CHRG_STS_DPPM_STS)) {
        ci->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else  if (linear_state & LINEAR_CHRG_STS_END_OF_CHARGE){
    		ci->charge_status = POWER_SUPPLY_STATUS_FULL;
    }else{
        ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    if((ci->linear_chg_prev_state!=linear_state)||
		(ci->charge_status!=prev_charge_status)){
	 bci_dbg(debug_common, "%s%s%s%s%s%s%s << %s >>\n",
	 	(linear_state&LINEAR_CHRG_STS_CRYSTL_OSC_OK)?"twl6032 charger: ":"",
	 	(linear_state&LINEAR_CHRG_STS_END_OF_CHARGE)?"eoc ":"",
	 	(linear_state&LINEAR_CHRG_STS_VBATOV)?"vbatov ":"",
	 	(linear_state&LINEAR_CHRG_STS_VSYSOV)?"vsysov ":"",
	 	(linear_state&LINEAR_CHRG_STS_DPPM_STS)?"dppm ":"",
	 	(linear_state&LINEAR_CHRG_STS_CV_STS)?"cv ":"",
	 	(linear_state&LINEAR_CHRG_STS_CC_STS)?"cc ":"",
	 	str_charge_state[ ci->charge_status ]);
    }
	
    ci->linear_chg_prev_state=linear_state;

}
static void  twl6030charger_fault_event_handler(struct twl6030_bci_charger_info *ci)
{
	u8 controller_stat;
	int ret,charger_fault=0;
	u8 usb_charge_sts = 0, usb_charge_sts1 = 0, usb_charge_sts2 = 0,charge_state=0;
	
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);


	if (ci->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
            return ;
	}


	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                              CONTROLLER_STAT1);

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
                              CHARGERUSB_INT_STATUS);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
                              CHARGERUSB_STATUS_INT1);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
                              CHARGERUSB_STATUS_INT2);

        bci_dbg(debug_common, "charge_state = 0x%02x \n", charge_state);
        if (charge_state&VBUS_DET ) {
            if (usb_charge_sts & CURRENT_TERM_INT)
                bci_dbg(debug_common, "BIT CURRENT_TERM_INT\n");
            if (usb_charge_sts & CHARGERUSB_THMREG)
                bci_dbg(debug_common, "BIT CHARGERUSB_THMREG\n");
            if (usb_charge_sts & CHARGERUSB_FAULT)
                bci_dbg(debug_common, "BIT CHARGERUSB_FAULT\n");

            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TMREG){
			bci_dbg(debug_common, "INTERNAL_CHARGER  Thermal regulation activated\n");
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_NO_BAT){
			bci_dbg(debug_common, "No Battery Present\n");
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BST_OCP){
			bci_dbg(debug_common, "USB Boost Over current protection\n");
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TH_SHUTD) {
			charger_fault = 1;
			bci_dbg(debug_common, "INTERNAL_CHARGER Thermal Shutdown\n");
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BAT_OVP){
			bci_dbg(debug_common, "INTERNAL_CHARGER Bat Over Voltage Protection\n");
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;			
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_POOR_SRC){
			bci_dbg(debug_common, "INTERNAL_CHARGER Poor input source\n");
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;				
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_SLP_MODE){
			bci_dbg(debug_common, "INTERNAL_CHARGER Sleep mode\n");
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            	}
            if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_VBUS_OVP){
			bci_dbg(debug_common, "INTERNAL_CHARGER VBUS over voltage %d V\n",
				twl6030_get_gpadc_conversion(ci,10));
			ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            	}

            if (usb_charge_sts2 & CHARGE_DONE) {
                bci_dbg(debug_common, "INTERNAL_CHARGER charge done\n");
            }
            if (usb_charge_sts2 & CURRENT_TERM)
                bci_dbg(debug_common, "INTERNAL_CHARGER CURRENT_TERM\n");
            if (usb_charge_sts2 & ICCLOOP)
                bci_dbg(debug_common, "INTERNAL_CHARGER ICCLOOP\n");
            if (usb_charge_sts2 & ANTICOLLAPSE)
                bci_dbg(debug_common, "INTERNAL_CHARGER ANTICOLLAPSE\n");

        } 
        if (charger_fault) {
		if(ci ->features & TWL6032_SUBCLASS){
			twl6032_stop_linear_charger(ci);
			twl6032_disable_power_path(ci);
		}else{
			twl6030_stop_internal_charger(ci);
			
		}
		ci->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            bci_dbg(debug_common,"Charger Fault stop charging\n");
        }
  

}
static int twl603x_charger_event(struct notifier_block *nb, unsigned long event,void *_data)
{

	struct twl6030_bci_charger_info *ci ;

	int ret = NOTIFY_DONE;
	if(!(event&BCI_INTERNAL_CHARGER))return ret;
	ci  = container_of(nb, struct twl6030_bci_charger_info, nb);
	event&=~BCI_TYPE_MASK;
	
	switch(event){
		case BCI_START_STOP_CHARGING:
			if(_data){
				int *current_mA=_data;
				ci->charger_outcurrentmA=ci->charger_targetcurrentmA=*current_mA;
				twl6030_config_vichrg_reg(ci,ci->charger_outcurrentmA);
	          		if(ci ->features & TWL6032_SUBCLASS){
					twl6030_set_ci_limit(ci , ci ->charger_incurrentmA);
					twl6030_config_voreg_reg(ci,ci->regulation_voltagemV);
					twl6030_set_eoc_current(ci ,50);
					twl6032_start_linear_charger(ci );
	            		}else{
	            			twl6030_start_internal_charger(ci );
	            		}
			}else{
				if(ci ->features & TWL6032_SUBCLASS){
	                		twl6032_stop_linear_charger(ci );
	            		}else{
	            			twl6030_stop_internal_charger(ci );
	            		}
			}
			
		break;
		case BCI_ENABLE_POWER_PATH:
			if(_data){
	          		if(ci ->features & TWL6032_SUBCLASS){
	                		twl6032_enable_power_path(ci );
	            		}
			}else{
				if(ci ->features & TWL6032_SUBCLASS){
	                		twl6032_disable_power_path(ci );
	            		}
			}
			
		break;
		case BCI_RESET_TIMER:
			twl6030_kick_watchdog(ci );

		break;
		case BCI_SET_CHARGE_CURRENT:
			{
				unsigned int *current_mA=_data;
				twl6030_config_vichrg_reg(ci , *current_mA);
			}
		break;

		case BCI_SET_INPUT_CURRENT_LIMIT:
			{
				unsigned int *current_mA=_data;
			
				ci ->charger_incurrentmA=*current_mA;
				
				twl6030_set_ci_limit(ci , ci ->charger_incurrentmA);
		
			}
		break;
		case BCI_SET_EOC_CURRENT:
			{
				unsigned int *current_mA=_data;
				ci->termination_currentmA=*current_mA;
				twl6030_set_eoc_current(ci ,ci->termination_currentmA);

			}
		break;
		case BCI_ENABLE_EOC:
			if(_data)
				twl6030_set_eoc_current(ci ,ci->termination_currentmA);
			else
				twl6030_set_eoc_current(ci ,0);
		break;

		
		case BCI_READ_FAULT_REASON:
		{
			struct bci_charger_read_fault *f=_data;
			twl6030charger_fault_event_handler(ci);

			f->status=ci->charge_status;
		}
		break;
		case BCI_CHAEGER_STATUS_REQUEST:
		{
			struct bci_charger_status_requset *s=_data;
			s->previous_state=ci->charge_status=s->current_state;
			if(ci ->features & TWL6032_SUBCLASS){
				twl6030_kick_watchdog(ci );
				twl6032_charger_poll_func(ci);
			}else{
				twl6030charger_fault_event_handler(ci);
			}
			s->current_state=ci->charge_status;
		}
		break;

			
	}
	return ret;
}


static ssize_t set_watchdog(struct device *dev,
                            struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci  = twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 1) || (val > 127))
        return -EINVAL;
    ci ->watchdog_duration = val;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);

    return status;
}

static ssize_t show_watchdog(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_charger_info *ci  = twl6030_bci_function_get_priv("twl603x_charger");

    val = ci ->watchdog_duration;
    return sprintf(buf, "%d\n", val);
}



static ssize_t set_regulation_voltage(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci = twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
            || (val > ci ->max_charger_voltagemV))
        return -EINVAL;
    ci ->regulation_voltagemV = val;
    twl6030_config_voreg_reg(ci, val);

    return status;
}

static ssize_t show_regulation_voltage(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_charger_info *ci = twl6030_bci_function_get_priv("twl603x_charger");

    val = ci ->regulation_voltagemV;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_termination_current(struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci = twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
        return -EINVAL;
    ci->termination_currentmA = val;
    twl6030_set_eoc_current(ci, val);

    return status;
}

static ssize_t show_termination_current(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_charger_info *ci =  twl6030_bci_function_get_priv("twl603x_charger");

    val = ci ->termination_currentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
                             struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci =  twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
        return -EINVAL;
    ci->charger_incurrentmA = val;
    twl6030_set_ci_limit(ci, val);

    return status;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
    unsigned int val;
    struct twl6030_bci_charger_info *ci  = twl6030_bci_function_get_priv("twl603x_charger");

    val = ci->charger_incurrentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
                                  struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci = twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
            || (val > ci->max_charger_currentmA))
        return -EINVAL;
    ci->charger_outcurrentmA = val;
    twl6030_config_vichrg_reg(ci, val);

    return status;
}

static ssize_t show_charge_current(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_charger_info *ci = twl6030_bci_function_get_priv("twl603x_charger");

    val = ci->charger_outcurrentmA;
    return sprintf(buf, "%u\n", val);
}

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_charger_info *ci  =  twl6030_bci_function_get_priv("twl603x_charger");

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
        return -EINVAL;
    ci->min_vbus = val;
    twl6030_config_min_vbus_reg(ci, val);

    return status;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
    unsigned int val;
    struct twl6030_bci_charger_info *ci =  twl6030_bci_function_get_priv("twl603x_charger");

    val = ci->min_vbus;
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_bsi(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_charger_info *ci =  twl6030_bci_function_get_priv("twl603x_charger");
    val = twl6030_get_gpadc_conversion(ci,0);
    return sprintf(buf, "%d\n", val);
}

static ssize_t show_stat1(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    u8 val;

	 twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val,
						CONTROLLER_STAT1);
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int1(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    u8 val;
	 twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val,
						CHARGERUSB_STATUS_INT1);
    return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int2(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    u8 val;
 
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val,
						CHARGERUSB_STATUS_INT1);	
    return sprintf(buf, "%u\n", val);
}

static DEVICE_ATTR(watchdog, S_IWUSR | S_IRUGO, show_watchdog, set_watchdog);


static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
                   show_regulation_voltage, set_regulation_voltage);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
                   show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
                   set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
                   set_charge_current);
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);

static DEVICE_ATTR(bsi, S_IRUGO, show_bsi, NULL);
static DEVICE_ATTR(stat1, S_IRUGO, show_stat1, NULL);
static DEVICE_ATTR(status_int1, S_IRUGO, show_status_int1, NULL);
static DEVICE_ATTR(status_int2, S_IRUGO, show_status_int2, NULL);


static struct attribute *twl6030_bci_attributes[] = {
    &dev_attr_watchdog.attr,
    &dev_attr_regulation_voltage.attr,
    &dev_attr_termination_current.attr,
    &dev_attr_cin_limit.attr,
    &dev_attr_charge_current.attr,
    &dev_attr_min_vbus.attr,
    &dev_attr_bsi.attr,
    &dev_attr_stat1.attr,
    &dev_attr_status_int1.attr,
    &dev_attr_status_int2.attr,
    NULL,
};
static const struct attribute_group twl6030_bci_attr_group = {
    .attrs = twl6030_bci_attributes,
};

static int __devinit _twl6030_charger_init(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_bci_charger_info *ci ;
	int ret;

	ci  = kzalloc(sizeof(*ci ), GFP_KERNEL);
		if (!ci )
			return -ENOMEM;

	if (!pdata) {
		bci_dbg(debug_common, "platform_data not available\n");
		ret = -EINVAL;
		 goto err_pdata;
	}


	ci ->dev = &pdev->dev;
	ci ->features = pdata->features;
 	ci ->power_path = pdata->use_power_path;
	if (pdata->use_eeprom_config) {
		ci ->max_charger_currentmA = twl6030_get_co_hard_limit(ci );
		ci ->max_charger_voltagemV = twl6030_get_vo_hard_limit(ci );
		ci ->termination_currentmA = twl6030_get_iterm_reg(ci );
		ci ->regulation_voltagemV = twl6030_get_voreg_reg(ci );
		ci ->low_bat_voltagemV = pdata->low_bat_voltagemV;
		dev_dbg(ci ->dev, "EEPROM max charge %d mA\n",
                ci ->max_charger_currentmA);
		dev_dbg(ci ->dev, "EEPROM max voltage %d mV\n",
                ci ->max_charger_voltagemV);
		dev_dbg(ci ->dev, "EEPROM termination %d mA\n",
                ci ->termination_currentmA);
		dev_dbg(ci ->dev, "EEPROM regulation %d mV\n",
                ci ->regulation_voltagemV);
	} else {
		ci ->max_charger_currentmA = pdata->max_charger_currentmA;
		ci ->max_charger_voltagemV = pdata->max_charger_voltagemV;
		ci ->termination_currentmA = pdata->termination_currentmA;
		ci ->regulation_voltagemV = pdata->max_bat_voltagemV;
		ci ->low_bat_voltagemV = pdata->low_bat_voltagemV;
	}

	ci->current_resistance=pdata->sense_resistor_mohm;
    if (!pdata->use_eeprom_config) {
        /* initialize for USB charging */
        twl6030_set_vo_hard_limit(ci , ci ->max_charger_voltagemV);
        twl6030_set_co_hard_limit(ci , ci ->max_charger_currentmA);
		
    }

    if ((ci ->features & TWL6032_SUBCLASS)&&(ci->power_path)) {
        /*TWL6030 without linear charger*/
	ci ->use_linear_charger=1;

    } else {
        ci ->use_linear_charger=0;        

    }

    ci ->charger_incurrentmA = ci ->max_charger_currentmA;
    ci ->charger_outcurrentmA = ci ->max_charger_currentmA;
	

    ci ->watchdog_duration = 32;

    if ((ci ->features & TWL6032_SUBCLASS)&&(ci ->use_linear_charger)){
		 twl6032_stop_linear_charger(ci );
		twl6030_enable_dppm(ci ,1);
		twl6030_set_vbat_tracking_voltage(ci ,200);
		//twl6032_set_tracking_mode_vsys_threshold(ci ,4300);
		//twl6030_set_Q2_Q3_limit(ci ,3700);
            /*Disable Linear Charger before Fuel Gauge initialized anyway*/
           
       
    } else {
        u8 charge_state,hw_state;
        /* read charger controller_stat1 */
        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &charge_state,
                        CONTROLLER_STAT1);
        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
 
        twl6030_config_min_vbus_reg(ci ,ci ->regulation_voltagemV);
        if ((charge_state&VBUS_DET)&&!(hw_state & STS_USB_ID)) {
            /*Disable Charger before Fuel Gauge initialized*/
            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
                             0, CONTROLLER_CTRL1);
        }
    }
    twl_i2c_write_u8(TWL6032_MODULE_CHARGER, 0x04,ANTICOLLAPSE_CTRL1);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MASK_MCHARGERUSB_THMREG,
                     CHARGERUSB_INT_MASK);		
    twl6030_config_voreg_reg(ci,ci->regulation_voltagemV);
    ci ->nb.notifier_call = twl603x_charger_event;
    ret = twl6030_bci_function_register("twl603x_charger",&ci ->nb,SUPPORT_FUNC,BCI_INTERNAL_CHARGER,ci);
    if (ret)
        dev_err(&pdev->dev, "bci register notifier failed %d\n", ret);



    ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    if (ret)
        dev_dbg(&pdev->dev, "could not create sysfs files\n");

	
    return 0;

err_pdata:
    kfree(ci );

    return ret;
}



int __init twl6030_charger_init(struct platform_device *pdev)
{
    return _twl6030_charger_init(pdev);
}


