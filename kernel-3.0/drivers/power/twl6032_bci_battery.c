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
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <asm/mach-types.h>
#include <linux/android_boot.h> 
#include <linux/i2c/bq27520.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

enum irq_handler_reg {
    irq_reg_charge_state,
    irq_reg_hw_state,
    irq_reg_int_mask,
};


#define LINEAR_CHRG_STS_CRYSTL_OSC_OK		0x40
#define LINEAR_CHRG_STS_END_OF_CHARGE		0x20
#define LINEAR_CHRG_STS_VBATOV		0x10
#define LINEAR_CHRG_STS_VSYSOV		0x08
#define LINEAR_CHRG_STS_DPPM_STS		0x04
#define LINEAR_CHRG_STS_CV_STS		0x02
#define LINEAR_CHRG_STS_CC_STS		0x01
#define FG_REG_00	0x00
#define FG_REG_01	0x01
#define FG_REG_02	0x02
#define FG_REG_03	0x03
#define FG_REG_04	0x04
#define FG_REG_05	0x05
#define FG_REG_06	0x06
#define FG_REG_07	0x07
#define FG_REG_08	0x08
#define FG_REG_09	0x09
#define FG_REG_10	0x0A
#define FG_REG_11	0x0B
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

/* FG_REG_00 */
#define CC_ACTIVE_MODE_SHIFT	6
#define CC_AUTOCLEAR		(1 << 2)
#define CC_CAL_EN		(1 << 1)
#define CC_PAUSE		(1 << 0)

#define REG_TOGGLE1		0x90
#define FGDITHS			(1 << 7)
#define FGDITHR			(1 << 6)
#define FGS			(1 << 5)
#define FGR			(1 << 4)
#define GPADC_SAMP_WINDOW (1<<2)

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

#define BAT_INSERT_ADC		0
#define TEMP_ADC			1
#define VSYS_ADC			7
#define VBK_ADC				8
#define VAC_ADC				9
#define VBUS_ADC			10
#define ID_ADC				14
#define VBAT_ADC			18

#define BCI_NOTIFIER(nh, val, v) if((val&registered_events)&&(val&twl603x_bci->function_type))\
									blocking_notifier_call_chain(nh, val,v)


/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};
static const unsigned int fuelgauge_time[4] = {250000, 62500, 15600, 3900};



#define IRQ_INFO_SIZE 10

#define CHARGER_PLUGOUT_DETECTING   0x01;


struct twl6030_irq_info {
    u8 charge_state;
    u8 hw_state;
    unsigned long time;
};
struct twl6030_function {
	struct list_head node;
	char name[32];
	struct notifier_block *nb;
	int type;
	void *priv;
	unsigned int events;
};

struct gas_gauge_t {
	bool can_read;
	struct semaphore sem;
};

struct twl6030_bci_device_info {
    struct device		*dev;
    int			voltage_mV;
    int			sys_voltage_mV;
    int			bk_voltage_mV;
    int			vac_voltage_mV;
    int			vbus_voltage_mV;
    int			temp_adc_voltage_mv;	
    int			current_uA;
    int			current_avg_uA;
    int			current_suspend_uA;
    int			temp_C;
    int			charge_status;
    int			charge_prev_status;

    int			bat_health;
    int			charger_source;/*From which connector*/

    u8			usb_online;
    u8			ac_online;
    u8			stat1;
    u8			linear_stat;
    u8			status_int1;
    u8			status_int2;

    u16			current_avg_interval;
    u16			monitoring_interval;

    unsigned int		max_charger_voltagemV;
    unsigned int		max_charger_currentmA;
    unsigned int		charger_incurrentmA;
    unsigned int		charger_outcurrentmA;
    unsigned int		regulation_voltagemV;
    unsigned int		low_bat_voltagemV;
    unsigned int		termination_currentmA;
    unsigned int		usb_max_power;
    unsigned long		event;
    unsigned int		sense_resistor_mohm;	
    unsigned int		power_path;
    unsigned int		fully_capacity_mAh;
    int				ac_share_usb_connector;
    unsigned int		capacity;
	atomic_t			sched_set;
	struct Loading_Table* loading_Table;
    struct power_supply	bat;
    struct power_supply	usb;
    struct power_supply	ac;
    struct power_supply	bk_bat;

    struct otg_transceiver	*otg;
    struct notifier_block	nb;
    struct delayed_work	bci_monitor_work;


    struct wake_lock wakelock;


    struct blocking_notifier_head notifier_list;
    int			fault_event;
	int			ctl_event;
    int usb_ac_in_detection;
    int usb_ac_detection_result;
    unsigned long		features;
   struct twl6030_irq_info ctrl_irq[IRQ_INFO_SIZE];
    int push_index;
    int pop_index;
    int charger_glitch_debounce;
    int dynamic_current_limit_en;
    int dynamic_limit_current;
    int charger_eoc;

    unsigned long jiffies_plug_out_in;
    u32 boot_mode;

#ifdef CONFIG_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
	struct list_head functions;
	int init_complete;
	int plugged_out_debounce_count;
	unsigned int function_type;
	int temp_Rx;
	int temp_Ry;
	struct ntc_res_temp_desc *battery_tmp_tbl;
	struct gas_gauge_t gas_gauge;
	
};


//static unsigned int debug_en=(0xFFFFFFFF&~(debug_temperature|debug_int_fuel_gauge|debug_continuous_info));
static unsigned int debug_en = (debug_leve_max & ~(debug_temperature | debug_int_fuel_gauge | debug_continuous_info));
static unsigned int registered_events=0;
static struct twl6030_bci_device_info *twl603x_bci=NULL;
static void twl6030_bci_handle_plugged_in(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state);

unsigned int twl6030_bci_debug_mask(void){
return debug_en;
}
int twl6030_avg_sample_current_uA(int cc_offset,int mode, int resistance,
        int delta_acc,int delta_counter ) {

    int _cc_offset= cc_offset * delta_counter;
    int factor=((62000000/resistance)*fuelgauge_rate[mode]/delta_counter)>>15;
    return (delta_acc-_cc_offset)*factor;
}
static int irq_handler_init(struct twl6030_bci_device_info *di) {
  	di->push_index=di->pop_index=0;
    return 0;
}
static int irq_handler_push(struct twl6030_bci_device_info *di) {

        int ret=0;
        ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &di->ctrl_irq[di->push_index].charge_state,CONTROLLER_STAT1);

        ret |= twl_i2c_read_u8(TWL6030_MODULE_ID0, &di->ctrl_irq[di->push_index].hw_state, STS_HW_CONDITIONS);

      	  di->ctrl_irq[di->push_index].time=jiffies;

        printk("\n [push data:%d] 0x%02x, 0x%02x \n",di->push_index,di->ctrl_irq[di->push_index].charge_state, di->ctrl_irq[di->push_index].hw_state);

        di->push_index = (++di->push_index)%IRQ_INFO_SIZE;

  
  
    return 0;
}


int twl6030_bci_function_register(const char* name,struct notifier_block *nb, unsigned int events, int type,void* priv)
{
	int ret=-EINVAL;
	u8 controller_stat,hw_state;
	struct twl6030_function *func;

	if (!twl603x_bci) {
		twl603x_bci = kzalloc(sizeof(*twl603x_bci), GFP_KERNEL);
		if(!twl603x_bci)
			return ret;
		twl603x_bci->charge_status=POWER_SUPPLY_STATUS_UNKNOWN;
		INIT_LIST_HEAD(&twl603x_bci->functions);
		BLOCKING_INIT_NOTIFIER_HEAD(&twl603x_bci->notifier_list);
	}
	func= kzalloc(sizeof(struct twl6030_function), GFP_KERNEL);
	if(!func)
		return ret;
	INIT_LIST_HEAD(&func->node);
	list_add(&func->node,&twl603x_bci->functions);
	strcpy(func->name,name);	
	func->events=events;
	func->type=type;	
	func->nb=nb;
	func->priv=priv;
	registered_events |= events;
	ret= blocking_notifier_chain_register(&twl603x_bci->notifier_list, nb);
	switch(type){
        case BCI_EXTERNAL_CHARGER:
		if(twl603x_bci->function_type&BCI_INTERNAL_CHARGER){
			 bci_dbg( debug_ext_charger,
			 	"external charger %s registered, unregister internal charger\n",name);
			twl603x_bci->function_type&=~BCI_INTERNAL_CHARGER;
			twl6030_bci_function_unregister( "twl603x_charger");
		}
		twl603x_bci->function_type|=BCI_EXTERNAL_CHARGER;
		if((events&BCI_CONFIG)&&(twl603x_bci->init_complete)){
			struct bci_config_charger c;
			c.input_current=twl603x_bci->charger_incurrentmA;
			c.battery_regulation_voltage=twl603x_bci->regulation_voltagemV;
			c.charge_current = twl603x_bci->charger_outcurrentmA;
			c.eoc_current = twl603x_bci->termination_currentmA;
			c.eoc_enable = 1;
			c.max_input_current=twl603x_bci->max_charger_currentmA;
			c.max_input_voltage=twl603x_bci->max_charger_voltagemV;
			
			BCI_NOTIFIER(&twl603x_bci->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_CONFIG),&c);
		}
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);
		twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
		if ((controller_stat & VAC_DET)&& (twl603x_bci->charge_status == POWER_SUPPLY_STATUS_UNKNOWN)) {
			twl6030_bci_handle_plugged_in(twl603x_bci,controller_stat,hw_state);
			twl603x_bci->charge_status = POWER_SUPPLY_STATUS_CHARGING;
			twl603x_bci->stat1 = controller_stat;
		}
	
        break;
	case BCI_EXTERNAL_FUELGAUGE:
		twl603x_bci->function_type&=~BCI_INTERNAL_FUELGAUGE;
		twl603x_bci->function_type|=BCI_EXTERNAL_FUELGAUGE;
		twl6030_bci_function_unregister( "twl603x_fuel_gauge");
		if((events&BCI_CONFIG)&&(twl603x_bci->init_complete)){
			struct bci_config_fuel_gauge f;
			f.full_capcity_mAh=twl603x_bci->fully_capacity_mAh;
			f.battery_voltage=twl603x_bci->regulation_voltagemV;
			f.battery_cutoff_voltage=twl603x_bci->low_bat_voltagemV;
			f.eoc_current=twl603x_bci->termination_currentmA;
			BCI_NOTIFIER(&twl603x_bci->notifier_list, (BCI_EXTERNAL_FUELGAUGE|BCI_CONFIG),&f);
		}

		
        break;
        default:
        break;
    }
	

    return ret;
}


int twl6030_bci_function_unregister( const char* name)
{
    int ret = -1;
	struct twl6030_function *func,*n;
	list_for_each_entry_safe(func, n, &twl603x_bci->functions, node){
		if(!strcmp(func->name,name)){
			list_del(&func->node);
			blocking_notifier_chain_unregister(&twl603x_bci->notifier_list, func->nb);
			kfree(func);
			ret=0;
			
		}

	}

    return ret;
}

void *twl6030_bci_function_get_priv( const char* name)
{
   
	struct twl6030_function *func,*n;
	list_for_each_entry_safe(func, n, &twl603x_bci->functions, node){
		if(!strcmp(func->name,name)){
			return func->priv;
		}
	}

    return NULL;
}
static unsigned int twl6030_get_co_hard_limit(void)
{
	unsigned int currentmA;
	u8 val;
	static const int vichrg[] = {
		300, 350, 400, 450, 500, 600, 700, 800,
		900, 1000, 1100, 1200, 1300, 1400, 1500, 300
	};
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT2);
	currentmA = vichrg[val & 0xF];

	return currentmA;
}
static unsigned int twl6030_get_vo_hard_limit(void)
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRLLIMIT1);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}
static unsigned int twl6030_get_iterm_reg(void)
{
    unsigned int currentmA;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_CTRL2);
    currentmA = 50 + (val >> 5) * 50;

    return currentmA;
}
static unsigned int twl6030_get_voreg_reg(void )
{
    unsigned int voltagemV;
    u8 val;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &val, CHARGERUSB_VOREG);
    voltagemV = 3500 + (val * 20);

    return voltagemV;
}

static int twl6030_get_gpadc_conversion(struct twl6030_bci_device_info *di,int channel_no)
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
static void twl6030_get_gpadcs(struct twl6030_bci_device_info *di)
{
    struct twl6030_gpadc_request req;
    int ret;

    req.channels = (1 << TEMP_ADC)|
				(1 << VSYS_ADC)|
				(1 << VBK_ADC)|
				(1 << VAC_ADC)|
				(1 << VBUS_ADC);
			
    if (di->features & TWL6032_SUBCLASS) {
		 req.channels |=(1 << VBAT_ADC);
    }
    req.method = TWL6030_GPADC_SW2;

    req.active = 0;
    req.func_cb = NULL;
    ret = twl6030_gpadc_conversion(&req);
    if (ret < 0)
        return;

    if (req.rbuf[TEMP_ADC] > 0)
        di->temp_adc_voltage_mv= req.rbuf[TEMP_ADC];
	
    if (req.rbuf[VSYS_ADC] > 0)
        di->sys_voltage_mV= req.rbuf[VSYS_ADC];
	
    if (req.rbuf[VBK_ADC] > 0)
        di->bk_voltage_mV= req.rbuf[VBK_ADC];
	
    if (req.rbuf[VAC_ADC] > 0)
        di->vac_voltage_mV= req.rbuf[VAC_ADC];

    if (req.rbuf[VBUS_ADC] > 0)
        di->vbus_voltage_mV= req.rbuf[VBUS_ADC];

    if (di->features & TWL6032_SUBCLASS) {
	  if (req.rbuf[VBAT_ADC] > 0)
        	di->voltage_mV= req.rbuf[VBAT_ADC];
    } else {
		di->voltage_mV = di->sys_voltage_mV;
    }
    return;
}
#if 0
static int is_battery_present(struct twl6030_bci_device_info *di)
{
    int val;
    /*
     * Prevent charging on batteries were id resistor is
     * less than 5K.
     */
    val = twl6030_get_gpadc_conversion(di,BAT_INSERT_ADC);
    //if (val <10000)
    //	return 0;
    return 1;
}
#endif

static void twl6030_disable_external_charger(struct twl6030_bci_device_info *di)
{
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,0, CONTROLLER_CTRL1);
}

static void twl6030_enable_external_charger(struct twl6030_bci_device_info *di)
{
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
		CONTROLLER_CTRL1_EN_CHARGER|CONTROLLER_CTRL1_SEL_CHARGER,
		CONTROLLER_CTRL1);
}

static void twl6030_disable_all_charger(struct twl6030_bci_device_info *di)
{
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER,0, CONTROLLER_CTRL1);
}

/* for TWL6032 to disable linear charger*/
static void twl6030_disable_linear_charger(struct twl6030_bci_device_info *di)
{
    u8 value;

    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, CONTROLLER_CTRL1);
    value &= ~CONTROLLER_CTRL1_EN_LINCH;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, value, CONTROLLER_CTRL1);
}

static void twl6030_bci_handle_plugged_in(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){

		if (controller_stat & VAC_DET) {
			bci_dbg( debug_plug_in_out,"external charger start\n");
			di->ac_online = POWER_SUPPLY_TYPE_MAINS;
			twl6030_enable_external_charger(di);
			BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),"enable");	
			if(di->ac_share_usb_connector){
				/*AC shared usb connector need wait USB/AC dectect
				That is use external charger as usb/ac charger*/
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->usb_max_power);
				if(!di->usb_ac_detection_result){
					di->usb_ac_in_detection=6;
            			}else di->usb_ac_in_detection=0;
				di->charger_source = POWER_SUPPLY_TYPE_USB;
			}else{
				/*stand alone external charger*/
				di->charger_source = POWER_SUPPLY_TYPE_MAINS;
				di->charger_incurrentmA=di->max_charger_currentmA;
				printk(KERN_INFO"twl6030_bci: Set AC %d mA \n",di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),&di->charger_outcurrentmA);
				
			}
	        di->charger_glitch_debounce=0;
	    }

	    if (controller_stat & VBUS_DET) {
	        /* In HOST mode (ID GROUND) when a device is connected, Mask
	         * VBUS OVP interrupt and do no enable usb charging
	         */

	        if (hw_state & STS_USB_ID) {
			u8 temp;
	            twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

	            if (!(temp & MASK_MCHARGERUSB_FAULT))
	                twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
	                                 (temp | MASK_MCHARGERUSB_FAULT),CHARGERUSB_INT_MASK);
				bci_dbg(debug_plug_in_out,"Vbus and usb id were detected!!");
				/*if USB_ID is set Stop Internal Charger anyway*/
				BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);	
	 			if ((di->features & TWL6032_SUBCLASS)&&(di->power_path) )
					BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),0);
	                		// twl6032_disable_power_path(di);
	        } else  {
	        /*Use intenal charger as AC/USB charger*/
	            di->usb_online = POWER_SUPPLY_TYPE_USB;
			if(di->ac_share_usb_connector)
				di->charger_source = POWER_SUPPLY_TYPE_USB;
	            bci_dbg(debug_plug_in_out,"Vbus was detected");
	            if ((controller_stat & VAC_DET)&&( di->charger_source == POWER_SUPPLY_TYPE_MAINS)){
	                bci_dbg(debug_plug_in_out,"USB charger detected, continue with VAC\n");
	            } else {
	            		di->charger_source = POWER_SUPPLY_TYPE_USB;           
            			if(!di->usb_ac_detection_result){
					di->usb_ac_in_detection=6;
            			}else di->usb_ac_in_detection=0;
				BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->usb_max_power);
	                  if ((di->features & TWL6032_SUBCLASS)&&(di->power_path) ){
					BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),"enable");
	                	}else
	            			BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),&di->usb_max_power);	

	            }
	        }
	        di->charger_glitch_debounce=0;
	    }
}

/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */

static int twl6030charger_ctrl_event_handler(struct twl6030_bci_device_info *di)
{

	u8 stat_toggle, stat_reset, stat_set = 0;
	u8 charge_state = 0;
	u8 hw_state = 0, temp = 0;
	int index=di->pop_index;


	if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		return -1;
	}
	
	if (di->push_index==di->pop_index) {
		if (di->charger_glitch_debounce) {
			if (time_after(jiffies, di->ctrl_irq[(index-1+IRQ_INFO_SIZE)%IRQ_INFO_SIZE].time + HZ/2)) {
				charge_state = di->ctrl_irq[index].charge_state;
				di->charger_glitch_debounce=0;
				bci_dbg(debug_plug_in_out, "charger_glitch_debounce release!!\n");
				power_supply_changed(&di->bat);
				power_supply_changed(&di->usb);
				power_supply_changed(&di->ac);
			}

		}
		return 0;
	}
	/*Mark as processed*/
	di->pop_index = (++di->pop_index)%IRQ_INFO_SIZE;


	/* read charger controller_stat1 */
	charge_state = di->ctrl_irq[index].charge_state;
	hw_state =di->ctrl_irq[index].hw_state;


	bci_dbg(debug_plug_in_out,"[POP %d]: CONTROLLER_STAT1 %02x->%02x HW:%02x \n",index,di->stat1,charge_state,hw_state);
	if (di->stat1==charge_state) {
		return 0;
	}
	
	stat_toggle = di->stat1 ^ charge_state;
	stat_set = stat_toggle & charge_state;
	stat_reset = stat_toggle & di->stat1;

	if(stat_reset|stat_set|(VAC_DET|VBUS_DET)){
	    if (stat_reset & VAC_DET) {
			
		bci_dbg(debug_plug_in_out,"vac removed\n");
		BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
		twl6030_disable_external_charger(di);
		di->charger_glitch_debounce|=CHARGER_PLUGOUT_DETECTING;
		di->plugged_out_debounce_count=5;
		di->ac_online = 0;
		di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
	    }



	    if (stat_reset & VBUS_DET) {
			
	        /* On a USB detach, UNMASK VBUS OVP if masked*/
	        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

	        if (temp & MASK_MCHARGERUSB_FAULT)
	            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
	                             (temp & ~MASK_MCHARGERUSB_FAULT),
	                             CHARGERUSB_INT_MASK);
	        bci_dbg( debug_plug_in_out,"usb removed\n");
			BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);	
	 		if ((di->features & TWL6032_SUBCLASS)&&(di->power_path) )
				BCI_NOTIFIER(&di->notifier_list,( BCI_INTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),0);
	                	// twl6032_disable_power_path(di);
	    
		di->charger_glitch_debounce|=CHARGER_PLUGOUT_DETECTING;
		di->plugged_out_debounce_count=5;
		di->usb_online = 0;
		di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
	    }

		twl6030_bci_handle_plugged_in(di,stat_set,hw_state);

	} 
	/*Other reasons except plug in/out*/
	else if (di->usb_online == POWER_SUPPLY_TYPE_USB) {
		struct bci_charger_read_fault read;
		BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_READ_FAULT_REASON), &read);
	if (!read.continue_charge) {
		BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
		//twl6030_stop_internal_charger(di);
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		bci_dbg(debug_fault,"Internal Charger Fault stop charging!!\n");
		}	
	}
	if ((di->capacity != -1)&&(!di->charger_glitch_debounce))
		power_supply_changed(&di->bat);

	di->stat1 = charge_state;

	return 0;

}

static irqreturn_t twl6030charger_ctrl_interrupt(int irq, void *_di)
{
    struct twl6030_bci_device_info *di = _di;
    printk("%s(), status = %d \n", __func__, di->charge_status);
    if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        return IRQ_HANDLED;
    } else {

	irq_handler_push(di);

	di->ctl_event++;
	if(!schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10))){
			/*work already on the Queue*/
		if(cancel_delayed_work(&di->bci_monitor_work)){
				/*success*/
				schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10));
		}else 
				atomic_xchg(&di->sched_set, 10);
	}
    }

    return IRQ_HANDLED;
}

static irqreturn_t twl6030charger_fault_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;

	if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		return IRQ_HANDLED;
	} else {
		u8 controller_stat;
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);
		if ((controller_stat &(VBUS_DET|VAC_DET))&&(di->fault_event<=1)){
			di->fault_event++;
		
		
			if(!schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10))){
				/*work already on the Queue*/
				if(cancel_delayed_work(&di->bci_monitor_work)){
					/*success*/
					schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10));
				}else 
					atomic_xchg(&di->sched_set, 10);
			}
		}else
			di->fault_event=0;
	}

    return IRQ_HANDLED;
}
static void  twl6030charger_dump_fault(struct twl6030_bci_device_info *di,
	u8 usb_charge_sts,u8 usb_charge_sts1,u8 usb_charge_sts2)
{
	if (usb_charge_sts & CURRENT_TERM_INT)
                bci_dbg(debug_common, "BIT CURRENT_TERM_INT\n");
	if (usb_charge_sts & CHARGERUSB_THMREG)
                bci_dbg(debug_common, "BIT CHARGERUSB_THMREG\n");
	if (usb_charge_sts & CHARGERUSB_FAULT)
                bci_dbg(debug_common, "BIT CHARGERUSB_FAULT\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TMREG)
		   bci_dbg(debug_common, "INTERNAL_CHARGER  Thermal regulation activated\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_NO_BAT)
		   bci_dbg(debug_common, "No Battery Present\n");
      if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BST_OCP)
		   bci_dbg(debug_common, "USB Boost Over current protection\n");
      if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TH_SHUTD)
		   bci_dbg(debug_common, "INTERNAL_CHARGER Thermal Shutdown\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BAT_OVP)
		   bci_dbg(debug_common, "INTERNAL_CHARGER Bat Over Voltage Protection\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_POOR_SRC)
		   bci_dbg(debug_common, "INTERNAL_CHARGER Poor input source\n");
      if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_SLP_MODE)
		   bci_dbg(debug_common, "INTERNAL_CHARGER Sleep mode\n");
      if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_VBUS_OVP)
		   bci_dbg(debug_common, "INTERNAL_CHARGER VBUS over voltage %d V\n",
				twl6030_get_gpadc_conversion(di,VBUS_ADC));
	if (usb_charge_sts2 & CHARGE_DONE)
                bci_dbg(debug_common, "INTERNAL_CHARGER charge done\n");
	if (usb_charge_sts2 & CURRENT_TERM)
                bci_dbg(debug_common, "INTERNAL_CHARGER CURRENT_TERM\n");
	if (usb_charge_sts2 & ICCLOOP)
                bci_dbg(debug_common, "INTERNAL_CHARGER ICCLOOP\n");
	if (usb_charge_sts2 & ANTICOLLAPSE)
                bci_dbg(debug_common, "INTERNAL_CHARGER ANTICOLLAPSE\n");
}
static int  twl6030_battery_fault_event_handler(struct twl6030_bci_device_info *di)
{
    u8 controller_stat,clear_reg;
        struct bci_charger_read_fault read;
 

	read.continue_charge=1;
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

	bci_dbg(debug_common,"twl6030_battery_fault_event_handler() \n");


	if (controller_stat & VAC_DET) {
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_INT_STATUS);
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_STATUS_INT1);
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_STATUS_INT2);
		BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_READ_FAULT_REASON), &read);
		if (!read.continue_charge) {
			twl6030_disable_external_charger(di);
			di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			bci_dbg(debug_fault,"External Charger Fault stop charging!!\n");
		}
	}else  if (controller_stat & VBUS_DET) {
		if(di->function_type&BCI_INTERNAL_CHARGER){
			read.capacity=di->capacity;
			read.current_uA=di->current_uA;
			read.voltage=di->voltage_mV;
			BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_READ_FAULT_REASON), &read);
			if (!read.continue_charge) {
				BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
				//twl6030_stop_internal_charger(di);
				di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				bci_dbg(debug_fault,"Internal Charger Fault stop charging!!\n");
			}
		}else{
		
			u8 usb_charge_sts = 0, usb_charge_sts1 = 0, usb_charge_sts2 = 0;

			twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
                              CHARGERUSB_INT_STATUS);
			twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
                              CHARGERUSB_STATUS_INT1);
			twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
                              CHARGERUSB_STATUS_INT2);
			twl6030charger_dump_fault(di,usb_charge_sts,usb_charge_sts1,usb_charge_sts2);	
		}
	}
	return 0;

}

static int twl6030battery_current(struct twl6030_bci_device_info *di)
{
    int ret = 0;
    u16 read_value = 0;
	s16 cc_offset=0;
	u8 value=0,mode=0;
	twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, &value,FG_REG_00);
	value>>=CC_ACTIVE_MODE_SHIFT;
	mode=value&0x03;
	
    /* FG_REG_08, 09 is 10 bit signed calibration offset value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &read_value,
                 FG_REG_08, 2);
   	cc_offset= ((s16)(read_value << 6) >> 6);
    /* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
    ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
                       FG_REG_10, 2);
    if (ret < 0) {
        bci_dbg(debug_common, "failed to read FG_REG_10: current_now\n");
        return 0;
    }
    return twl6030_avg_sample_current_uA(cc_offset,mode,di->sense_resistor_mohm,
                     (s16)(read_value << 2) >> 2,1);

}
static int  resistance_to_temp(struct ntc_res_temp_desc *ntc_table,int  res)
{
    int ret_temp,i;

		
    for (i=0;i<ntc_table->res;i++) {
        if (res>=ntc_table[i].res) {
            ret_temp=(res-ntc_table[i].res)*(ntc_table[i-1].temp-ntc_table[i].temp)*10/(ntc_table[i-1].res-ntc_table[i].res)
                     +ntc_table[i].temp*10;
            return ret_temp;
        }
    }
    return -EINVAL;
}
static int twl6030battery_temp(struct twl6030_bci_device_info *di)
{
	int temp_C=250;
	if (di->battery_tmp_tbl != NULL) {
			int adc_code = 0;
			int r_ntc = 0;
			int denominator;
			adc_code =twl6030_get_gpadc_conversion(di,TEMP_ADC);
			denominator = di->temp_Ry*1250-(di->temp_Ry+di->temp_Rx)*adc_code;
			if (denominator>0) {
				r_ntc=(di->temp_Rx*di->temp_Ry/denominator)*adc_code;
				temp_C=resistance_to_temp(di->battery_tmp_tbl,r_ntc);
			}

	}
	return temp_C;

}
static int capacity_lookup(struct twl6030_bci_device_info *di, int volt, int curr)
{
	unsigned int i, j;
	struct batt_volt_cap_desc *capacity_table;
	if(curr>0)
		j=0;
	else 
		for (j=0;((di->loading_Table[j+1].batt_volt_cap!=NULL)&&(di->loading_Table[j].curr>=curr));j++);
		
	capacity_table=di->loading_Table[j].batt_volt_cap;
	
	if (volt <= capacity_table[0].volt)
		return capacity_table[0].cap;

	/*
	 * The highest capacity has already been covered.
	 * Don't compare with that - else you overrun the array
	 * in the i+1 comparison
	 */
	for (i = 0; i < BATT_VOLT_CAP_SIZE-1; i++) {
		if ((volt >= capacity_table[i].volt) &&
				(volt <= capacity_table[i + 1].volt))
			return capacity_table[i].cap;
	}

	/* Default - should not reach here */
	return capacity_table[0].cap;
}

/*
 * Setup the twl6030 BCI module to enable backup
 * battery charging.
 */
static int twl6030backupbatt_setup(void)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
    rd_reg |= BB_CHG_EN;
    ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);

    return ret;
}

/*
 * Setup the twl6030 BCI module to measure battery
 * temperature
 */
static int twl6030battery_temp_setup(bool enable)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);

#if 0
    rd_reg |= TWL6030_GPADC_CTRL_TEMP1_EN |TWL6030_GPADC_CTRL_TEMP1_EN_MONITOR  ;
#else
    if (enable)
        rd_reg |= TWL6030_GPADC_CTRL_TEMP1_EN |TWL6030_GPADC_CTRL_TEMP1_EN_MONITOR;
    else
        rd_reg ^= TWL6030_GPADC_CTRL_TEMP1_EN |TWL6030_GPADC_CTRL_TEMP1_EN_MONITOR;
#endif

    ret |= twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

    return ret;
}

static int twl6030battery_voltage_setup(void)
{
    int ret;
    u8 rd_reg = 0;

    ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);
    rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
    ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);

    ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
    rd_reg = rd_reg | VBUS_MEAS;
    ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);

    ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
    rd_reg = rd_reg | ID_MEAS;
    ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);

    return ret;
}
/*Turn of current measurement before shutdown only*/
static int twl6030battery_current_setup(bool enable)
{
    int ret = 0;

    /*
    * Writing 0 to REG_TOGGLE1 has no effect, so
    * can directly set/reset FG.
    */

    ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 
		enable?(FGDITHS | FGS):(FGDITHR | FGR), 
		REG_TOGGLE1);
    if (ret)
        return ret;
    ret = twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);


    return ret;
}

static enum power_supply_property twl6030_bci_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property twl6030_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_bk_bci_battery_props[] = {
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};
static void twl6030_bci_late_init(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){

           /*This mean it comes from boot, and the fuel gauge is ready*/

              /*
               * In HOST mode (ID GROUND) with a device connected,
               * do no enable any charging
               */

		if ((controller_stat & (VBUS_DET|VAC_DET))) {
			twl6030_bci_handle_plugged_in(di,controller_stat,hw_state);
		 }
	       
 		/* Enable interrupt to complete power on */               
                twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
                                         REG_INT_MSK_LINE_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
                                         REG_INT_MSK_STS_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
                                         REG_INT_MSK_LINE_C);
                twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
                                         REG_INT_MSK_STS_C);

                /*USB interrupt and detection may happen before fuel gauge ready*/
                if (di->usb_ac_detection_result)di->usb_ac_in_detection=0;

                di->stat1 = controller_stat;
}
static void twl6030_bci_ack_usb_ac_result(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){


		di->usb_ac_in_detection=-1;
		if(di->ac_share_usb_connector){
			if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS) {
				di->charger_incurrentmA=di->max_charger_currentmA;
				printk(KERN_INFO"twl6030_bci: Set AC %d mA \n",di->charger_incurrentmA);
			    di->usb_online = 0;   
				di->ac_online = POWER_SUPPLY_TYPE_MAINS;            
			} else if (di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB) {
				di->charger_incurrentmA=di->usb_max_power;
				printk(KERN_INFO"twl6030_bci: Set USB %d mA \n",di->charger_incurrentmA);
			    di->usb_online = POWER_SUPPLY_TYPE_USB;   
				di->ac_online = 0; 
			} else {
				di->charger_incurrentmA=di->max_charger_currentmA;
				di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
				printk(KERN_INFO"twl6030_bci: Set CarKit %d mA \n",di->charger_incurrentmA);
			    di->usb_online = 0;   
				di->ac_online = POWER_SUPPLY_TYPE_MAINS; 			
			}
			
			BCI_NOTIFIER(&di->notifier_list,(BCI_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->charger_incurrentmA);
			if (di->features & TWL6032_SUBCLASS) {
				BCI_NOTIFIER(&di->notifier_list,(BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),
					di->charger_incurrentmA>di->charger_outcurrentmA?&di->charger_outcurrentmA:&di->charger_incurrentmA);
			}
			if (controller_stat & VAC_DET){
				BCI_NOTIFIER(&di->notifier_list,(BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),
					di->charger_incurrentmA>di->charger_outcurrentmA?&di->charger_outcurrentmA:&di->charger_incurrentmA);
			}
		}
		
			

	
}

static void twl6030_bci_unknown(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	
	if (!(controller_stat & VAC_DET)){
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static void twl6030_bci_charging(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	
	/*Reset current in usb/ac detect finished*/
	if (di->usb_ac_in_detection==0) {
		twl6030_bci_ack_usb_ac_result(di,controller_stat,hw_state);
	}
		


}
static void twl6030_bci_discharging(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	/*We only care about transition and process all uncomplete action if need*/
	if (di->charge_prev_status == POWER_SUPPLY_STATUS_UNKNOWN) {

		twl6030_bci_late_init(di,controller_stat,hw_state);

	}else {
		if(di->charge_prev_status != POWER_SUPPLY_STATUS_DISCHARGING) {
	            bci_dbg(debug_common,"clear in detection\n");
	            di->usb_ac_in_detection=-1;
	            di->usb_ac_detection_result=0;
	            di->usb_max_power=100;
			di->charger_eoc=0;
			di->dynamic_limit_current= di->termination_currentmA;
		}
		if (di->plugged_out_debounce_count>0) {
	            di->plugged_out_debounce_count--;
		}
	}


}

static void twl6030_bci_not_charging(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
		/*The most complicated state is POWER_SUPPLY_STATUS_NOT_CHARGING
		 it may do error handling here*/	


	if (di->charge_prev_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		twl6030_bci_late_init(di,controller_stat,hw_state);
	}else{ 
		/*Reset current in usb/ac detect finished*/
		if (di->usb_ac_in_detection==0) {
			twl6030_bci_ack_usb_ac_result(di,controller_stat,hw_state);
		}
		if (di->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
				

		}
		if ((di->charge_prev_status == POWER_SUPPLY_STATUS_NOT_CHARGING)&&
		(di->capacity<95)&&(di->usb_ac_in_detection==-1)&&di->charger_eoc){
			printk(KERN_INFO"twl6030_bci: Re-start charger from NOT_CHARGING!!%d mA \n",di->charger_incurrentmA);
			if((VAC_DET&controller_stat)&&(!di->ac_share_usb_connector)){

					/*stand alone external charger*/
				di->charger_incurrentmA=di->max_charger_currentmA;
				printk(KERN_INFO"twl6030_bci: Set AC %d mA \n",di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),&di->charger_outcurrentmA);
			}else{
				twl6030_bci_ack_usb_ac_result(di,controller_stat,hw_state);
			}

		}
	}


}
static void twl6030_bci_full(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	

	if (di->charge_prev_status != POWER_SUPPLY_STATUS_FULL) {
				
		di->charger_eoc=1;
				
		if (controller_stat & VBUS_DET) {   
			if (di->features & TWL6032_SUBCLASS) {
				/*Just disable linear charger, but keep DC/DC on*/
				BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
				if(di->power_path) 
					BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),"enable");
					//twl6032_enable_power_path(di);
					
			}

			
		}


              
	} 
	else if (di->charge_prev_status == POWER_SUPPLY_STATUS_FULL){
			/*If implement recharge, place code here*/

		if((di->capacity<95)&&(di->usb_ac_in_detection==-1)&&di->charger_eoc){
			printk(KERN_INFO"twl6030_bci: Re-start charger from FULL!! %d mA \n",di->charger_incurrentmA);
			if((VAC_DET&controller_stat)&&(!di->ac_share_usb_connector)){

					/*stand alone external charger*/
				di->charger_incurrentmA=di->max_charger_currentmA;
				printk(KERN_INFO"twl6030_bci: Set AC %d mA \n",di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_SET_INPUT_CURRENT_LIMIT),&di->charger_incurrentmA);
				BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),&di->charger_outcurrentmA);
			}else{
				twl6030_bci_ack_usb_ac_result(di,controller_stat,hw_state);
			}

		}
		if (controller_stat & VAC_DET) {
			BCI_NOTIFIER(&di->notifier_list,(BCI_EXTERNAL_CHARGER| BCI_RESET_TIMER),0);
			
		}


	}
        

}
typedef void (*batt_state)(struct twl6030_bci_device_info *,  u8 , u8);
batt_state	bci_stat[]={
twl6030_bci_unknown,
twl6030_bci_charging,
twl6030_bci_discharging,
twl6030_bci_not_charging,
twl6030_bci_full
};
static void twl6030_bci_decide_default_state(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){

    if (controller_stat & (VAC_DET|VBUS_DET)){
	
		if(((!di->ac_share_usb_connector)&&(controller_stat & VAC_DET))||
			((di->ac_share_usb_connector)&&!(hw_state & STS_USB_ID))){
		
        	if ((di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING)||
				(di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN))
	  			di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					
			if (!wake_lock_active(&di->wakelock))
	            	wake_lock(&di->wakelock);
		
		}else if ((!di->ac_share_usb_connector)&&(controller_stat & VBUS_DET)){
			//if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN)
	      			di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
				 if (wake_lock_active(&di->wakelock)&&
	 				!di->charger_glitch_debounce&&
	 				!di->plugged_out_debounce_count&&
	 				di->charge_prev_status == POWER_SUPPLY_STATUS_DISCHARGING) {
	          			wake_unlock(&di->wakelock);
				}
		}
	
    }else{
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		if (wake_lock_active(&di->wakelock)&&
			!di->charger_glitch_debounce&&
			!di->plugged_out_debounce_count&&
			di->charge_prev_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		       wake_unlock(&di->wakelock);
		}
		   
    }
   	
}
static void twl6030_bci_handle_interrupts(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
    if(di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN)
		return;
	
    if ((di->push_index!=di->pop_index) ||(di->charger_glitch_debounce)||(di->ctl_event>0)){ //have some push events
	if (!twl6030charger_ctrl_event_handler(di)){
		if(di->ctl_event>0)  di->ctl_event--;

	}
    }else if (di->fault_event>0) {
	twl6030_battery_fault_event_handler(di);
	di->fault_event--;
    }

    twl6030_bci_decide_default_state(di,controller_stat,hw_state);
   
}

static void twl6030_bci_update_status_from_charger(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	if (((di->charge_status == POWER_SUPPLY_STATUS_CHARGING)||
		(di->charge_status == POWER_SUPPLY_STATUS_FULL)||
		(di->charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING))&&
		(di->usb_ac_in_detection<0)){
		/*Discharge and unknow should not poll charger status,
		keep current status for feul gauge*/
		struct bci_charger_status_requset charger_status;
		/*Nerver update battery charge status when usb/ac in detection */
		charger_status.current_state=di->charge_status;
		charger_status.previous_state=di->charge_prev_status;
		charger_status.capacity=di->capacity;
		charger_status.current_uA=di->current_uA;
		charger_status.voltage=di->voltage_mV;
		BCI_NOTIFIER(&di->notifier_list, (BCI_CHARGER|BCI_CHAEGER_STATUS_REQUEST), &charger_status);
		di->charge_status=charger_status.current_state;
		di->charge_prev_status=charger_status.previous_state;
	}
}
static int twl6030_bci_update_status_from_gauge(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
	struct bci_fuel_gauge_status_request fg_status={0};
	
	fg_status.current_state=di->charge_status;
	fg_status.previous_state=di->charge_prev_status;
	fg_status.capacity=di->capacity;
	fg_status.temp_adc_voltage_mv=di->temp_adc_voltage_mv;
	fg_status.voltage=di->voltage_mV;
	
	BCI_NOTIFIER(&di->notifier_list, (BCI_FUELGAUGE|BCI_FUELGAUGE_STATUS_REQUEST), &fg_status);
		if(fg_status.data_valid){
			di->voltage_mV=fg_status.voltage;
			di->current_uA=fg_status.current_uA;
			di->capacity=fg_status.capacity;
			di->temp_C=fg_status.temperature;

			if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN){
				twl6030_bci_decide_default_state(di,controller_stat,hw_state);
			}else{
				/*Fuel Gauge should not modify charge status,
				except it need time to catch up full for batter user
				feeling*/
				di->charge_status=fg_status.current_state;
			}

	} else {
		di->current_uA=twl6030battery_current(di);
		di->temp_C=twl6030battery_temp(di);
		/*Dummy Gauge*/
		if(!(di->function_type&BCI_FUELGAUGE)){
			di->capacity=capacity_lookup(di, di->voltage_mV,di->current_uA/1000);
			if(di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN){
				twl6030_bci_decide_default_state(di,controller_stat,hw_state);
			}
		}
	}
	
	if (di->gas_gauge.can_read == false) {
		di->gas_gauge.can_read = true;
		/* wake up the down semaphore. */
		up(&(di->gas_gauge.sem));
	}
	
	return fg_status.capacity_changed;
}
static const char* state_str[]={
"unknown",
"charging",
"discharge",
"n_charge",
"full",
};
static void twl6030_bci_battery_work(struct work_struct *work)
{
	u8 controller_stat,hw_state;
	struct twl6030_bci_device_info *di = container_of(work,
                                                     struct twl6030_bci_device_info, bci_monitor_work.work);
	
	int next_interval=0,capacity_changed=0;
	
	 di->charge_prev_status= di->charge_status; 	
/*1. read hw plug status and adcs */

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
	twl6030_get_gpadcs(di);	
#if 0
	di->temp_adc_voltage_mv =twl6030_get_gpadc_conversion(di,TEMP_ADC);
	di->sys_voltage_mV= twl6030_get_gpadc_conversion(di,VSYS_ADC);
	di->bk_voltage_mV =twl6030_get_gpadc_conversion(di,VBK_ADC);
	di->vac_voltage_mV=twl6030_get_gpadc_conversion(di,VAC_ADC);
	di->vbus_voltage_mV=twl6030_get_gpadc_conversion(di,VBUS_ADC);
	if (di->features & TWL6032_SUBCLASS) {
		di->voltage_mV = twl6030_get_gpadc_conversion(di,VBAT_ADC);
	} else {
		di->voltage_mV = di->sys_voltage_mV;
	}
#endif
/*2. handle interrupt and  Update new status from charger and gauge */
	twl6030_bci_handle_interrupts(di,controller_stat,hw_state);
	twl6030_bci_update_status_from_charger(di,controller_stat,hw_state);
	capacity_changed=twl6030_bci_update_status_from_gauge(di,controller_stat,hw_state);
		
/*3. Do right action in each state and state transition */	
	bci_stat[di->charge_status](di,controller_stat,hw_state);
	bci_dbg(debug_continuous_info, 
		"vsys=%d, vbat=%d, curr=%d, temp=%d, cap=%d vac=%d vbus=%d %s <- %s\n",
		 di->sys_voltage_mV,di->voltage_mV, di->current_uA/1000,di->temp_C,di->capacity,
		 di->vac_voltage_mV,di->vbus_voltage_mV,
		 state_str[di->charge_status],state_str[di->charge_prev_status]);	
/*4. report status to power supply framework */
    if (capacity_changed||(di->charge_prev_status!= di->charge_status)) {
		
		bci_dbg(debug_common, 
			"vsys=%d, vbat=%d, curr=%d, temp=%d, cap=%d vac=%d vbus=%d %s <- %s\n",
			 di->sys_voltage_mV,di->voltage_mV, di->current_uA/1000,di->temp_C,di->capacity,
			 di->vac_voltage_mV,di->vbus_voltage_mV,
			 state_str[di->charge_status],state_str[di->charge_prev_status]);	
	
		if(!di->charger_glitch_debounce)
			power_supply_changed(&di->bat);
    }

/*5. reschedule next work */
	if((next_interval=atomic_xchg(&di->sched_set,0))){
		schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(next_interval));	
	}else if((di->ctl_event)||(di->fault_event)){
        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(50));		
    }else if((di->usb_ac_in_detection>0)||(di->charger_glitch_debounce)){
	if (di->usb_ac_in_detection>0) 
        di->usb_ac_in_detection--;
	

        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(500));

    } else {
        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(1000 * di->monitoring_interval));
    }
	


}

/*============================== No Main Battery Special Works =============================*/

static int twl6030charger_no_battery_ctrl_event_handler(struct twl6030_bci_device_info *di)
{

	u8 stat_toggle, stat_reset, stat_set = 0;
	u8 charge_state = 0;
	u8 hw_state = 0, temp = 0;
	int index=di->pop_index;

	if (di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		return -1;
	}

	if (di->push_index==di->pop_index) {
		if (di->charger_glitch_debounce) {
			if (time_after(jiffies, di->ctrl_irq[(index-1+IRQ_INFO_SIZE)%IRQ_INFO_SIZE].time + HZ/2)) {
				charge_state = di->ctrl_irq[index].charge_state;
				di->charger_glitch_debounce=0;
				bci_dbg(debug_plug_in_out, "charger_glitch_debounce release!!\n");
				power_supply_changed(&di->bat);
				power_supply_changed(&di->usb);
				power_supply_changed(&di->ac);
			}

		}
		return 0;
	}
	/*Mark as processed*/
	di->pop_index = (++di->pop_index)%IRQ_INFO_SIZE;


	/* read charger controller_stat1 */
	charge_state = di->ctrl_irq[index].charge_state;
	hw_state =di->ctrl_irq[index].hw_state;


	bci_dbg(debug_plug_in_out,"[POP %d]: CONTROLLER_STAT1 %02x->%02x HW:%02x \n",index,di->stat1,charge_state,hw_state);
	if (di->stat1==charge_state) {
		return 0;
	}
	
	stat_toggle = di->stat1 ^ charge_state;
	stat_set = stat_toggle & charge_state;
	stat_reset = stat_toggle & di->stat1;

	if(stat_reset|stat_set|(VAC_DET|VBUS_DET)){
	    if (stat_reset) {
			
	        /* On a USB detach, UNMASK VBUS OVP if masked*/
	        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

	        if (temp & MASK_MCHARGERUSB_FAULT)
	            twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
	                             (temp & ~MASK_MCHARGERUSB_FAULT),
	                             CHARGERUSB_INT_MASK);
		 twl6030_disable_all_charger(di);   
		di->charger_glitch_debounce=0;
		di->plugged_out_debounce_count=0;
		di->usb_online = 0;
		di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
		di->usb_ac_in_detection=-1;
	      di->usb_ac_detection_result=0;
	      di->usb_max_power=100;
		di->charger_eoc=0;
		di->dynamic_limit_current= di->termination_currentmA;;
	    }

	    if (stat_set & VBUS_DET) {
	        /* In HOST mode (ID GROUND) when a device is connected, Mask
	         * VBUS OVP interrupt and do no enable usb charging
	         */

	        if (hw_state & STS_USB_ID) {
		
	            twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp, CHARGERUSB_INT_MASK);

	            if (!(temp & MASK_MCHARGERUSB_FAULT))
	                twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
	                                 (temp | MASK_MCHARGERUSB_FAULT),CHARGERUSB_INT_MASK);
				bci_dbg(debug_plug_in_out,"Vbus and usb id were detected!!");
				/*if USB_ID is set Stop Internal Charger anyway*/
				 twl6030_disable_all_charger(di);   
	        } else  {
	        /*Use intenal charger as AC/USB charger*/
	            di->usb_online = POWER_SUPPLY_TYPE_USB;
	            bci_dbg(debug_plug_in_out,"Vbus was detected");
	        }
	        di->charger_glitch_debounce=0;
	    }
	} 
	
	if ((di->capacity != -1)&&(!di->charger_glitch_debounce))
		power_supply_changed(&di->bat);

	di->stat1 = charge_state;

	return 0;

}
static int  twl6030_no_battery_fault_event_handler(struct twl6030_bci_device_info *di)
{
    u8 controller_stat,clear_reg;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);

	if (controller_stat) {
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_INT_STATUS);
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_STATUS_INT1);
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &clear_reg,
                              CHARGERUSB_STATUS_INT2);
	}
	return 0;

}
static void twl6030_bci_no_battery_handle_interrupts(struct twl6030_bci_device_info *di,  u8 controller_stat, u8 hw_state){
    if(di->charge_status == POWER_SUPPLY_STATUS_UNKNOWN)
		return;
	
    if ((di->push_index!=di->pop_index) ||(di->charger_glitch_debounce)||(di->ctl_event>0)){ //have some push events
	if (!twl6030charger_no_battery_ctrl_event_handler(di)){
		if(di->ctl_event>0)  di->ctl_event--;

	}
    }else if (di->fault_event>0) {
	twl6030_no_battery_fault_event_handler(di);
	di->fault_event--;
    }

  
}
static void twl6030_bci_no_battery_work(struct work_struct *work)
{
	u8 controller_stat,hw_state;
	struct twl6030_bci_device_info *di = container_of(work,
                                                     struct twl6030_bci_device_info, bci_monitor_work.work);
	
	int next_interval=0,capacity_changed=0;
	
	 di->charge_prev_status= di->charge_status; 	
/*1. read hw plug status and adcs */

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,	CONTROLLER_STAT1);
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
	
	di->voltage_mV=di->sys_voltage_mV= twl6030_get_gpadc_conversion(di,VSYS_ADC);
	di->bk_voltage_mV =twl6030_get_gpadc_conversion(di,VBK_ADC);
	di->vac_voltage_mV=twl6030_get_gpadc_conversion(di,VAC_ADC);
	di->vbus_voltage_mV=twl6030_get_gpadc_conversion(di,VBUS_ADC);
	di->capacity=100;
	di->current_uA=twl6030battery_current(di);
/*2. handle interrupt and  Update new status from charger and gauge */

    if (controller_stat & (VAC_DET|VBUS_DET)){
		di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	
    }else{
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    }
   	
	twl6030_bci_no_battery_handle_interrupts(di,controller_stat,hw_state);
		
/*3. Do right action in each state and state transition */	

/*4. report status to power supply framework */
    if (capacity_changed||(di->charge_prev_status!= di->charge_status)) {
		
	bci_dbg(debug_common, 
		"vsys=%d, vbat=%d, curr=%d, temp=%d, cap=%d vac=%d vbus=%d %s <- %s\n",
		 di->sys_voltage_mV,di->voltage_mV, di->current_uA/1000,di->temp_C,di->capacity,
		 di->vac_voltage_mV,di->vbus_voltage_mV,
		 state_str[di->charge_status],state_str[di->charge_prev_status]);	
	
	if(!di->charger_glitch_debounce)
		power_supply_changed(&di->bat);

    }

/*5. reschedule next work */
	if((next_interval=atomic_xchg(&di->sched_set,0))){
		schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(next_interval));	
	}else if((di->ctl_event)||(di->fault_event)){
        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(50));		
    }else if((di->usb_ac_in_detection>0)||(di->charger_glitch_debounce)){
	if (di->usb_ac_in_detection>0) 
        di->usb_ac_in_detection--;
	

        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(500));

    } else {
        schedule_delayed_work(&di->bci_monitor_work,
                              msecs_to_jiffies(1000 * di->monitoring_interval));
    }
	


}
/*=======================================================================================*/
static void twl6030_work_interval_changed(struct twl6030_bci_device_info *di)
{
    schedule_delayed_work(&di->bci_monitor_work,
                          msecs_to_jiffies(1000 * di->monitoring_interval));
}

#define to_twl6030_bci_device_info(x) container_of((x), \
			struct twl6030_bci_device_info, bat);

static void twl6030_bci_battery_external_power_changed(struct power_supply *psy)
{
    struct twl6030_bci_device_info *di = to_twl6030_bci_device_info(psy);
	
	if(!schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10))){
			/*work already on the Queue*/
		if(cancel_delayed_work(&di->bci_monitor_work)){
			/*success*/
			schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(10));
		}else 
			atomic_xchg(&di->sched_set, 10);
	}
}

#define to_twl6030_ac_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, ac);

static int twl6030_ac_get_property(struct power_supply *psy,
                                   enum power_supply_property psp,
                                   union power_supply_propval *val)
{

    struct twl6030_bci_device_info *di = to_twl6030_ac_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
	if(di->ac_share_usb_connector)
        	val->intval = di->ac_online||(di->usb_ac_detection_result==POWER_SUPPLY_TYPE_MAINS);
	else
		val->intval = di->ac_online;
	
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = twl6030_get_gpadc_conversion(di,VAC_ADC);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
                                    enum power_supply_property psp,
                                    union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = to_twl6030_usb_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
	if(di->ac_share_usb_connector)
        	 val->intval = di->usb_online&&((di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB));
	else
		val->intval = di->usb_online;
       
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = twl6030_get_gpadc_conversion(di,VBUS_ADC);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

#define to_twl6030_bk_bci_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, bk_bat);

static int twl6030_bk_bci_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di = to_twl6030_bk_bci_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = di->bk_voltage_mV * 1000;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int twl6030_bci_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct twl6030_bci_device_info *di;

    di = to_twl6030_bci_device_info(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        if ((di->charge_status==POWER_SUPPLY_STATUS_FULL)||(di->charger_eoc)) {
            val->intval = POWER_SUPPLY_STATUS_FULL;
        } else if ((di->usb_ac_detection_result==POWER_SUPPLY_TYPE_USB)&&
                   (di->charge_status==POWER_SUPPLY_STATUS_CHARGING)) {
            val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

        } else if ((di->usb_ac_detection_result == POWER_SUPPLY_TYPE_MAINS)) {
            val->intval = POWER_SUPPLY_STATUS_CHARGING;

        }
        else {
            val->intval = di->charge_status;

        }
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        if (di->features & TWL6032_SUBCLASS) {
            di->voltage_mV = twl6030_get_gpadc_conversion(di,VBAT_ADC);
        } else {
            di->voltage_mV = twl6030_get_gpadc_conversion(di,VSYS_ADC);
        }
        val->intval = di->voltage_mV * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        //twl6030battery_current(di);
        BCI_NOTIFIER(&di->notifier_list, (BCI_FUELGAUGE|BCI_READ_CURRENT), &di->current_uA);
        val->intval = di->current_uA;

        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = di->temp_C;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
		
	if(di->ac_share_usb_connector)
        	val->intval = di->usb_ac_detection_result;
	else
        	val->intval = di->charger_source;

        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = di->current_avg_uA;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = di->bat_health;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
		//pr_info("The process is \"%s\" (pid %i)\n", current->comm, current->pid);
		if (di->init_complete == 1 && (di->gas_gauge.can_read == false)) {
			/* wait for gas gauge can be read. */			
			if (down_interruptible(&(di->gas_gauge.sem))) {
				printk("[BCI]down_interruptible error\n");
				return -ERESTARTSYS;				
			}
		}
        val->intval =di->charger_eoc?100:di->capacity;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}


static int twl6030_usb_notifier_call(struct notifier_block *nb,
                                     unsigned long event, void *data)
{
    struct twl6030_bci_device_info *di =
                    container_of(nb, struct twl6030_bci_device_info, nb);

    bci_dbg(debug_common, "twl6030_usb_notifier_call(), event=%lu\n", event);

    di->event = event;
    switch (event) {
    case USB_EVENT_VBUS:
        di->usb_online = *((unsigned int *) data);
        switch (di->usb_online) {
        case POWER_SUPPLY_TYPE_USB_CDP:
            /*
             * Only 500mA here or high speed chirp
             * handshaking may break
             */
            bci_dbg(debug_common, 
            "POWER_SUPPLY_TYPE_USB_CDP , incurr = 500\n");
            di->charger_incurrentmA = 500;
            break;
        case POWER_SUPPLY_TYPE_USB_DCP:

            if (di->boot_mode == ANDROID_BOOT_MODE_CHARGER) {
          		di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
			di->charger_incurrentmA = di->max_charger_currentmA;
			bci_dbg(debug_common, 
				"POWER_SUPPLY_TYPE_USB_DCP , incurr = %d\n",
				di->charger_incurrentmA); 
			di->usb_ac_in_detection= 1;
            }
           

            break;
        case POWER_SUPPLY_TYPE_USB:
            if (di->boot_mode == ANDROID_BOOT_MODE_CHARGER) {
                di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;
                //Laker: In power-off charging, no android USB gadgets is configured such that USB_EVENT_ENUMERATED will not be received here
                //       (charging current = default usb_max_power = 100mA)
                di->usb_max_power = 500; //it will be applied in twl6030_bci_ack_usb_ac_result() as well
                bci_dbg(debug_common, "POWER_SUPPLY_TYPE_USB: set current %d mA\n", (int)di->usb_max_power);                
                di->charger_incurrentmA = di->usb_max_power;
                di->usb_ac_in_detection= 1;
            }
            break;

        }
        break;
    case USB_EVENT_ENUMERATED:
        di->usb_max_power = *((unsigned int *) data);
        bci_dbg(debug_common, "USB_EVENT_ENUMERATED: set current %d mA\n",(int)di->usb_max_power);
        if (di->usb_online == POWER_SUPPLY_TYPE_USB_CDP)
            di->charger_incurrentmA = 560;
        else {
            di->charger_incurrentmA = di->usb_max_power;
            if (di->usb_max_power >10) {
                di->usb_ac_detection_result=POWER_SUPPLY_TYPE_USB;
                bci_dbg(debug_common, "USB_EVENT_ENUMERATED\n");
            }
        }
        break;
    case USB_EVENT_CHARGER:
        /* POWER_SUPPLY_TYPE_USB_DCP */
        di->usb_online = POWER_SUPPLY_TYPE_USB_DCP;
        bci_dbg(debug_common,"line = %d,di->usb_online = POWER_SUPPLY_TYPE_USB_DCP\n",__LINE__);
        di->charger_incurrentmA = di->max_charger_currentmA;
        di->usb_ac_in_detection= 1;
        di->usb_ac_detection_result=POWER_SUPPLY_TYPE_MAINS;
        bci_dbg(debug_common,"line = %d,usb_ac_detection_result= main\n",__LINE__);
        break;
    case USB_EVENT_NONE:
        bci_dbg(debug_common,"USB_EVENT_NONE: usb_ac_detection_result=%d, usb_online=%d ; di->ac_online=%d\n",
                di->usb_ac_detection_result, di->usb_online, di->ac_online);
        if (di->usb_ac_detection_result == POWER_SUPPLY_TYPE_MAINS) { //for USB-AC and car kit
            di->ac_online = 0;
            bci_dbg(debug_common,"USB_EVENT_NONE: set di->ac_online = 0\n");
        }
        di->usb_ac_detection_result = 0;
        di->usb_online = 0;
        bci_dbg(debug_common,"line = %d, set di->usb_online = 0\n",__LINE__);
        di->charger_incurrentmA = 0;
        break;
    case USB_EVENT_ID:
    default:
        return NOTIFY_OK;
    }

    if (!di->charger_glitch_debounce)
        power_supply_changed(&di->usb);
    return NOTIFY_OK;
}

static void twl6030_bci_register_otg_event_handler(struct twl6030_bci_device_info *di){
	int ret;
    di->nb.notifier_call = twl6030_usb_notifier_call;
    di->otg = otg_get_transceiver();
    ret = otg_register_notifier(di->otg, &di->nb);
    if (ret)
        dev_err(di->dev, "otg register notifier failed %d\n", ret);
}
static ssize_t show_vbus_voltage(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    int val;
       struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    val = twl6030_get_gpadc_conversion(di ,VBUS_ADC);

    return sprintf(buf, "%d\n", val);
}


static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
    int val;
    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
    val = twl6030_get_gpadc_conversion(di ,ID_ADC);

    return sprintf(buf, "%d\n", val);
}
static ssize_t set_monitoring_interval(struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
 	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
        return -EINVAL;
    di->monitoring_interval = val;
    twl6030_work_interval_changed(di);

    return status;
}

static ssize_t show_monitoring_interval(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    unsigned int val;
 	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = di->monitoring_interval;
    return sprintf(buf, "%u\n", val);
}
static ssize_t show_regss(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    u8 value;

    pr_info("\n==============================================\n");

#define SHOW_REG(m,r) \
	value = 0; \
	twl_i2c_read_u8(m, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS1);
    SHOW_REG(TWL6030_MODULE_ID1, PWDNSTATUS2);
    SHOW_REG(TWL6030_MODULE_ID0, REG_MISC1);
    SHOW_REG(TWL_MODULE_MADC, TWL6030_GPADC_CTRL);
    SHOW_REG(TWL_MODULE_MADC, TWL6030_GPADC_CTRL2);

    pr_info("----------------------------------------------\n");
#undef SHOW_REG

#define SHOW_6030_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_6030_REG(CONTROLLER_INT_MASK);
    SHOW_6030_REG(CONTROLLER_CTRL1);
    SHOW_6030_REG(CONTROLLER_WDG);
    SHOW_6030_REG(CONTROLLER_STAT1);
    SHOW_6030_REG(CHARGERUSB_INT_STATUS);
    SHOW_6030_REG(CHARGERUSB_INT_MASK);
    SHOW_6030_REG(CHARGERUSB_STATUS_INT1);
    SHOW_6030_REG(CHARGERUSB_STATUS_INT2);
    SHOW_6030_REG(CHARGERUSB_CTRL1);
    SHOW_6030_REG(CHARGERUSB_CTRL2);
    SHOW_6030_REG(CHARGERUSB_CTRL3);
    SHOW_6030_REG(CHARGERUSB_VOREG);
    SHOW_6030_REG(CHARGERUSB_VICHRG);
    SHOW_6030_REG(CHARGERUSB_CINLIMIT);
    SHOW_6030_REG(CHARGERUSB_CTRLLIMIT1);
    SHOW_6030_REG(CHARGERUSB_CTRLLIMIT2);
    SHOW_6030_REG(ANTICOLLAPSE_CTRL1);

    pr_info("----------------------------------------------\n");
#undef SHOW_6030_REG

#define SHOW_6032_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6032_MODULE_CHARGER, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_6032_REG(CONTROLLER_CTRL2);
    SHOW_6032_REG(CONTROLLER_VSEL_COMP);
    SHOW_6032_REG(CHARGERUSB_VSYSREG);
    SHOW_6032_REG(CHARGERUSB_VICHRG_PC);
    SHOW_6032_REG(LINEAR_CHRG_STS);

    pr_info("----------------------------------------------\n");
#undef SHOW_6032_REG

#define SHOW_FG_REG(r) \
	value = 0; \
	twl_i2c_read_u8(TWL6030_MODULE_GASGAUGE, &value, r); \
	pr_info("%-35s 0x%02X\n", #r, value);

    SHOW_FG_REG(FG_REG_00);
    SHOW_FG_REG(FG_REG_01);
    SHOW_FG_REG(FG_REG_02);
    SHOW_FG_REG(FG_REG_03);
    SHOW_FG_REG(FG_REG_04);
    SHOW_FG_REG(FG_REG_05);
    SHOW_FG_REG(FG_REG_06);
    SHOW_FG_REG(FG_REG_07);
    SHOW_FG_REG(FG_REG_08);
    SHOW_FG_REG(FG_REG_09);
    SHOW_FG_REG(FG_REG_10);
    SHOW_FG_REG(FG_REG_11);

#undef SHOW_FG_REG

    pr_info("==============================================\n");

    return sprintf(buf, "See Kernel logs\n");
}
static ssize_t show_debug_level(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int val;
    //struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    val = debug_en;
    return sprintf(buf, "debug_level: 0x%02x\n", val);
}

static ssize_t set_debug_level(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    int tmp;
//    struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

    tmp = strict_strtol(buf, 16, &val);
    if ((val < 0) || (val> debug_leve_max)) { //debug_en: 0x00~0xFF
        printk("debug level set failed, buf = %s\n", buf);
        return -EINVAL;
    }
    debug_en = val;
    printk("set debug level:0x%02x successful\n", debug_en);

    return status;
}
static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(monitoring_interval, S_IWUSR | S_IRUGO,
                   show_monitoring_interval, set_monitoring_interval);
static DEVICE_ATTR(regs,  S_IRUGO, show_regss, NULL);
static DEVICE_ATTR(debug_level,  S_IWUSR | S_IRUGO, show_debug_level, set_debug_level);

static struct attribute *twl6030_bci_attributes[] = {
    &dev_attr_vbus_voltage.attr,
    &dev_attr_id_level.attr,	
    &dev_attr_monitoring_interval.attr,
    &dev_attr_regs.attr,
    &dev_attr_debug_level.attr,
    NULL,
};
#ifdef CONFIG_EARLYSUSPEND
static void twl603x_bci_early_suspend(struct early_suspend *handler)
{
    struct twl6030_bci_device_info *di;
    di = container_of(handler, struct twl6030_bci_device_info, early_suspend);
    printk("\n\n###### %s() \n\n", __func__); //screen off

}

static void twl603x_bci_early_resume(struct early_suspend *handler)
{
    struct twl6030_bci_device_info *di;


    di = container_of(handler, struct twl6030_bci_device_info, early_suspend);
    printk("\n\n######  %s() \n\n", __func__); //screen on


}
#endif
static const struct attribute_group twl6030_bci_attr_group = {
    .attrs = twl6030_bci_attributes,
};

static char *twl6030_bci_supplied_to[] = {
    "twl6030_battery",
};

static int __devinit twl6030_bci_battery_probe(struct platform_device *pdev)
{
    struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
    struct twl6030_bci_device_info *di;
    int irq;
    int ret;

    printk("6032 bci probe here \n");
    if (twl603x_bci){
	printk("assign 603x to di \n");		
	di=twl603x_bci;	
    }else {
        printk("bci alloc \n");	
        di = kzalloc(sizeof(*di), GFP_KERNEL);
        if (!di)
            return -ENOMEM;
        
	
	BLOCKING_INIT_NOTIFIER_HEAD(&di->notifier_list);
	INIT_LIST_HEAD(&di->functions);	
    }

    if (!pdata) {
        bci_dbg(debug_common, "platform_data not available\n");
        ret = -EINVAL;
        goto err_pdata;
    }

    if (pdata->monitoring_interval == 0) {
        di->monitoring_interval = 10;
    } else {
        di->monitoring_interval = pdata->monitoring_interval;
    }

    di->dev = &pdev->dev;
    di->dynamic_current_limit_en=0;

    di->features = pdata->features;
    di->ac_share_usb_connector=pdata->ac_share_usb_connector;
    di->usb_max_power=100;
    if (pdata->use_eeprom_config) {
        di->max_charger_currentmA = twl6030_get_co_hard_limit();
        di->max_charger_voltagemV = twl6030_get_vo_hard_limit();
        di->termination_currentmA = twl6030_get_iterm_reg();
        di->regulation_voltagemV = twl6030_get_voreg_reg();
        di->low_bat_voltagemV = pdata->low_bat_voltagemV;
        dev_dbg(di->dev, "EEPROM max charge %d mA\n",
                di->max_charger_currentmA);
        dev_dbg(di->dev, "EEPROM max voltage %d mV\n",
                di->max_charger_voltagemV);
        dev_dbg(di->dev, "EEPROM termination %d mA\n",
                di->termination_currentmA);
        dev_dbg(di->dev, "EEPROM regulation %d mV\n",
                di->regulation_voltagemV);
    } else {
        di->max_charger_currentmA = pdata->max_charger_currentmA;
        di->max_charger_voltagemV = pdata->max_bat_voltagemV;
        di->termination_currentmA = pdata->termination_currentmA;
        di->regulation_voltagemV = pdata->max_bat_voltagemV;
        di->low_bat_voltagemV = pdata->low_bat_voltagemV;
    }
    if(di->function_type&BCI_EXTERNAL_CHARGER){
	 pdata->charger_fuel_gauge_type&=~BCI_INTERNAL_CHARGER;	
    }
    if(di->function_type&BCI_EXTERNAL_FUELGAUGE){
	 pdata->charger_fuel_gauge_type&=~BCI_INTERNAL_FUELGAUGE;	
    }
    di->function_type |= pdata->charger_fuel_gauge_type;
    di->fully_capacity_mAh= pdata->capacity_mAh;
    di->power_path= pdata->use_power_path;
    di->jiffies_plug_out_in = 0;
    di->sense_resistor_mohm=pdata->sense_resistor_mohm;
    di->loading_Table= pdata->loading_Table;
    di->battery_tmp_tbl = (struct ntc_res_temp_desc*)pdata->battery_tmp_tbl_603x;
    di->temp_Rx=pdata->temp_rpu_ohm;
    di->temp_Ry=pdata->temp_rpd_ohm;
    di->dev = &pdev->dev;
    di->bat.name = "twl6030_battery";
    di->bat.supplied_to = twl6030_bci_supplied_to;
    di->bat.num_supplicants = ARRAY_SIZE(twl6030_bci_supplied_to);
    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    di->bat.properties = twl6030_bci_battery_props;
    di->bat.num_properties = ARRAY_SIZE(twl6030_bci_battery_props);
    di->bat.get_property = twl6030_bci_battery_get_property;
    /*di->bat.external_power_changed =
    		twl6030_bci_battery_external_power_changed;*/
    di->bat_health = POWER_SUPPLY_HEALTH_GOOD;

    di->usb.name = "twl6030_usb";
    di->usb.type = POWER_SUPPLY_TYPE_USB;
    di->usb.properties = twl6030_usb_props;
    di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
    di->usb.get_property = twl6030_usb_get_property;
    di->usb.external_power_changed =
        twl6030_bci_battery_external_power_changed;

    di->ac.name = "twl6030_ac";
    di->ac.type = POWER_SUPPLY_TYPE_MAINS;
    di->ac.properties = twl6030_ac_props;
    di->ac.num_properties = ARRAY_SIZE(twl6030_ac_props);
    di->ac.get_property = twl6030_ac_get_property;
    di->ac.external_power_changed =
        twl6030_bci_battery_external_power_changed;


    di->bk_bat.name = "twl6030_bk_battery";
    di->bk_bat.type = POWER_SUPPLY_TYPE_UPS;
    di->bk_bat.properties = twl6030_bk_bci_battery_props;
    di->bk_bat.num_properties = ARRAY_SIZE(twl6030_bk_bci_battery_props);
    di->bk_bat.get_property = twl6030_bk_bci_battery_get_property;

    //di->capacity =0;//= twl6030_read_last_capacity();
    di->capacity = 50; //Laker: set initial capacity to 50, in case that report capacity = 0 --> power off
    printk("bci capacity = %d \n", di->capacity);



    platform_set_drvdata(pdev, di);


    wake_lock_init(&di->wakelock, WAKE_LOCK_SUSPEND, "bci_wakelock");

    /* settings for temperature sensing */
    ret = twl6030battery_temp_setup(true);
	
    if (ret)
        goto temp_setup_fail;

    /* request charger fault interruption */
    irq = platform_get_irq(pdev, 1);
	
    ret = request_threaded_irq(irq, NULL, twl6030charger_fault_interrupt,
                               0, "twl_bci_fault", di);

    if (ret) {
        bci_dbg(debug_common, "could not request irq %d, status %d\n",
                irq, ret);
        goto temp_setup_fail;
    }

    /* request charger ctrl interruption */
    irq = platform_get_irq(pdev, 0);
	
    ret = request_threaded_irq(irq, NULL, twl6030charger_ctrl_interrupt,
                               0, "twl_bci_ctrl", di);
	

    if (ret) {
        bci_dbg(debug_common, "could not request irq %d, status %d\n",
                irq, ret);
        goto chg_irq_fail;
    }

    ret = power_supply_register(&pdev->dev, &di->bat);
    if (ret) {
        bci_dbg(debug_common, "failed to register main battery\n");
        goto batt_failed;
    }

    if(!pdata->no_main_battery){
    		INIT_DELAYED_WORK_DEFERRABLE(&di->bci_monitor_work,
                                 twl6030_bci_battery_work);
    }else{
		di->function_type=0;
		INIT_DELAYED_WORK_DEFERRABLE(&di->bci_monitor_work,
                                 twl6030_bci_no_battery_work);
    }



    ret = power_supply_register(&pdev->dev, &di->usb);
    if (ret) {
        bci_dbg(debug_common, "failed to register usb power supply\n");
        goto usb_failed;
    }

    ret = power_supply_register(&pdev->dev, &di->ac);
    if (ret) {
        bci_dbg(debug_common, "failed to register ac power supply\n");
        goto ac_failed;
    }

    ret = twl6030battery_voltage_setup();
    if (ret)
        bci_dbg(debug_common, "voltage measurement setup failed\n");

    ret = twl6030battery_current_setup(true);
    if (ret)
        bci_dbg(debug_common, "current measurement setup failed\n");


    di->boot_mode = android_boot_get_mode();

    di->charger_incurrentmA = di->max_charger_currentmA;
    di->charger_outcurrentmA = di->max_charger_currentmA;


    twl6030backupbatt_setup();
    di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
    di->charge_prev_status= POWER_SUPPLY_STATUS_UNKNOWN;
	di->gas_gauge.can_read = false;
	sema_init(&(di->gas_gauge.sem), 0);

    //di->nb.notifier_call = twl6030_usb_notifier_call;
    //di->otg = otg_get_transceiver();
    //ret = otg_register_notifier(di->otg, &di->nb);
    //if (ret)
       // dev_err(&pdev->dev, "otg register notifier failed %d\n", ret);

    schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(2000));


    ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    if (ret)
        dev_dbg(&pdev->dev, "could not create sysfs files\n");


    ret = irq_handler_init(di);
    if (ret<0)
        printk("irq_handler_init() failed\n");

    //twl6030_disable_all_charger(di);
    if (di->features & TWL6032_SUBCLASS) {
        twl6030_disable_linear_charger(di);
    } else {
        /* TWL6030 has no linear charger */
        twl6030_disable_all_charger(di);
    }

    {
        u8 controller_stat, hw_state = 0;
    
        twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat, CONTROLLER_STAT1);
        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
        //VAC_DET, in case twl6030_bci_late_init() is triggered too late
        if (controller_stat & VAC_DET) {
            bci_dbg(debug_plug_in_out, "%s: VAC_DET, set ac_online\n", __func__);
            di->ac_online = POWER_SUPPLY_TYPE_MAINS;
        }
        //VBUS_DET, in case twl6030_usb_irq (5s delay as irq_protection) is handled too late.
        if (controller_stat & VBUS_DET) {
            if (hw_state & STS_USB_ID) {
                bci_dbg(debug_plug_in_out, "%s: VBUS_DET and STS_USB_ID were detected!", __func__);
            } else {
                bci_dbg(debug_plug_in_out, "%s: VBUS_DET, set usb_online\n", __func__);
                di->usb_online = POWER_SUPPLY_TYPE_USB;
                if(di->ac_share_usb_connector) {
				    di->usb_ac_detection_result = POWER_SUPPLY_TYPE_USB;
                }
            }
        }
    }

#ifdef CONFIG_EARLYSUSPEND
    printk("register bci early suspend\n");
    di->early_suspend.suspend = twl603x_bci_early_suspend;
    di->early_suspend.resume = twl603x_bci_early_resume;
    register_early_suspend(&di->early_suspend);
#endif

if(!twl603x_bci){
	printk("assign di to twl603x_bci\n");
	twl603x_bci = di;
}
	/*
		Note: internal charger and fuel gauge are config at probe, 
		they are not support BCI_CONFIG.
		External charger or fuel gauge can use self config or BCI_CONFIG
		to get the battery information. 
		Export init functions to bci are not recommand 
	*/
	if(di->function_type&BCI_INTERNAL_CHARGER)
		twl6030_charger_init(pdev);
	if(di->function_type&BCI_INTERNAL_FUELGAUGE)
		twl6030_fuel_gauge_init(pdev);
	
		

	if(di->function_type&(BCI_EXTERNAL_FUELGAUGE|BCI_EXTERNAL_CHARGER)){
		struct twl6030_function *func,*n;
		int charger_found=0,fuel_gauge_found=0;
		list_for_each_entry_safe(func, n, &di->functions, node){
			if((func->type&BCI_EXTERNAL_CHARGER)&&
				(func->events&BCI_CONFIG))
				charger_found=1;
			if((func->type&BCI_EXTERNAL_FUELGAUGE)&&
				(func->events&BCI_CONFIG))
				fuel_gauge_found=1;
		
		}
		if(charger_found){
			struct bci_config_charger c;
			c.input_current=di->charger_incurrentmA;
			c.battery_regulation_voltage=di->regulation_voltagemV;
			c.charge_current=0;
			c.eoc_current=0;
			c.eoc_enable=1;
			c.max_input_current=di->max_charger_currentmA;
			c.max_input_voltage=di->max_charger_voltagemV;
			
			BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_CONFIG),&c);
		}

		if(fuel_gauge_found){
			struct bci_config_fuel_gauge f;
			f.full_capcity_mAh=di->fully_capacity_mAh;
			f.battery_voltage=di->regulation_voltagemV;
			f.battery_cutoff_voltage=di->low_bat_voltagemV;
			f.eoc_current=di->termination_currentmA;
			BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_FUELGAUGE|BCI_CONFIG),&f);
		}
	}
	twl6030_bci_register_otg_event_handler(di);
	di->init_complete=1;	
	dev_info(&pdev->dev, "Bci Bootup complete!! \n");
    return 0;

//bk_batt_failed:
//    power_supply_unregister(&di->ac);
ac_failed:
    power_supply_unregister(&di->usb);
usb_failed:
    cancel_delayed_work(&di->bci_monitor_work);
    power_supply_unregister(&di->bat);
batt_failed:
    free_irq(irq, di);
chg_irq_fail:
    irq = platform_get_irq(pdev, 1);
    free_irq(irq, di);
temp_setup_fail:
    wake_lock_destroy(&di->wakelock);
    platform_set_drvdata(pdev, NULL);
err_pdata:
    kfree(di);

    return ret;
}

static int __devexit twl6030_bci_battery_remove(struct platform_device *pdev)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
    int irq;

    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
                           REG_INT_MSK_STS_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_LINE_C);
    twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
                           REG_INT_MSK_STS_C);

    irq = platform_get_irq(pdev, 0);
    free_irq(irq, di);

    irq = platform_get_irq(pdev, 1);
    free_irq(irq, di);

    otg_unregister_notifier(di->otg, &di->nb);
    sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
    cancel_delayed_work(&di->bci_monitor_work);

    flush_scheduled_work();
    power_supply_unregister(&di->bat);
    power_supply_unregister(&di->usb);
    power_supply_unregister(&di->ac);
    power_supply_unregister(&di->bk_bat);
    wake_lock_destroy(&di->wakelock);
    platform_set_drvdata(pdev, NULL);
    kfree(di);

    return 0;
}

#ifdef CONFIG_PM
static int twl6030_bci_battery_suspend(struct platform_device *pdev,
                                       pm_message_t state)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);

    u8 rd_reg = 0;
    int ret;

    /* mask to prevent wakeup due to 32s timeout from External charger */
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
    rd_reg |= MVAC_FAULT;
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
                     CONTROLLER_INT_MASK);

    cancel_delayed_work(&di->bci_monitor_work);

    ret = twl6030battery_temp_setup(false);
    if (ret) {
        pr_err("%s: Temp measurement setup failed (%d)!\n",
               __func__, ret);
        return ret;
    }


    return 0;
}

static int twl6030_bci_battery_resume(struct platform_device *pdev)
{
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);

    u8 rd_reg = 0;
    int ret;

 
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
    rd_reg &= ~(0xFF & MVAC_FAULT);
    twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
                     CONTROLLER_INT_MASK);
    ret = twl6030battery_temp_setup(true);
    if (ret) {
        pr_err("%s: Temp measurement setup failed (%d)!\n",
               __func__, ret);
        return ret;
    }

    twl6030battery_voltage_setup();

    schedule_delayed_work(&di->bci_monitor_work, msecs_to_jiffies(100));

    return 0;
}
static void  twl6030_bci_battery_shutdown(struct platform_device *pdev) {
    struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
    u8 controller_stat=0;
    twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,
                    CONTROLLER_STAT1);
    /*Stop Charging that Fuel Gauge can lookup loading table*/
    if (controller_stat & VBUS_DET) {
        u8 hw_state;
        /*
         * In HOST mode (ID GROUND) with a device connected,
         * do no enable usb charging
         */
        twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
        if (!(hw_state & STS_USB_ID)) {
            BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
		if ((di->features & TWL6032_SUBCLASS)&&(di->power_path) ){
			BCI_NOTIFIER(&di->notifier_list, (BCI_INTERNAL_CHARGER|BCI_ENABLE_POWER_PATH),0);
		}

			
        }
    }


    if (controller_stat & VAC_DET) {
       BCI_NOTIFIER(&di->notifier_list, (BCI_EXTERNAL_CHARGER|BCI_START_STOP_CHARGING),0);
    }
	
    BCI_NOTIFIER(&di->notifier_list, (BCI_TYPE_MASK|BCI_SHUTDOWN),0);

}

#else
#define twl6030_bci_battery_suspend	NULL
#define twl6030_bci_battery_resume	NULL
#define twl6030_bci_battery_shutdown NULL
#endif /* CONFIG_PM */

#if 0
int twl6030_register_notifier(struct notifier_block *nb,
                              unsigned int events) { return 0;}
int twl6030_unregister_notifier(struct notifier_block *nb,
                                unsigned int events) {return 0;}
#endif
static struct platform_driver twl6030_bci_battery_driver = {
    .probe		= twl6030_bci_battery_probe,
    .remove		= __devexit_p(twl6030_bci_battery_remove),
    .suspend	= twl6030_bci_battery_suspend,
    .resume		= twl6030_bci_battery_resume,
    .shutdown =	twl6030_bci_battery_shutdown,
    .driver		= {
        .name	= "twl6030_bci",
    },
};

static int __init twl6030_battery_init(void)
{
    return platform_driver_register(&twl6030_bci_battery_driver);
}
module_init(twl6030_battery_init);

static void __exit twl6030_battery_exit(void)
{
    platform_driver_unregister(&twl6030_bci_battery_driver);
}
module_exit(twl6030_battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_bci");
MODULE_AUTHOR("Texas Instruments Inc");


