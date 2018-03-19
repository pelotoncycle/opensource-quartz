/*
 * drivers/power/bq2416x_battery.c
 *
 * BQ24153 / BQ24156 battery charging driver
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2416x.h>
#include <linux/i2c/twl6032_bci_battery.h>
#include <linux/power_supply.h>
#if defined (bci_dbg) 
#define bq_dbg bci_dbg
#else
#define bq_dbg
#endif
#define SUPPORT_FUNC (BCI_RESET_TIMER|BCI_START_STOP_CHARGING|BCI_ENABLE_POWER_PATH|\
		BCI_ENABLE_EOC|BCI_SET_CHARGE_CURRENT|BCI_SET_EOC_CURRENT|\
		BCI_SET_INPUT_CURRENT_LIMIT|BCI_READ_FAULT_REASON|\
		BCI_CHAEGER_STATUS_REQUEST|BCI_CONFIG)
/* BQ24160 / BQ24161  */
/* Status/Control Register */
#define BQ24160	24160
#define REG_STATUS_CONTROL			0x00
#define		TIMER_RST			(1 << 7)
#define		SUPPLY_SEL			(1 << 3)
#define		STATUS_SHIFT		4
#define		FAULT_SHIFT			0

/* Battery/Supply Status Register */
#define REG_BATT_SUPPLY_REGISTER			0x01
#define		IN_STATUS_SHIFT					6
#define		USB_STATUS_SHIFT				4
#define		BAT_STATUS_SHIFT				1
#define		OTG_LOCK						(1<<3)


/* Control Register */
#define REG_CONTROL_REGISTER			0x02
#define		RESET_CHIP						(1<<7)
#define		USB_CURRENT_LIMIT_SHIFT	4
#define		EN_STAT_OUT				(1<<3)
#define		EOC_EN						(1<<2)
#define		CHARGE_DISABLE					(1<<1)
#define		HiZ_MODE					(1<<0)


/* Control/Battery Voltage Register */
#define REG_BATTERY_VOLTAGE			0x03
#define		BATT_VOLTAGE_SHIFT		2 /* offset 3.5V, step 20mV that is (a*20+3500mV) */
#define		I_LIMIT_SEL_2A5			(1<<1)
#define		DP_DM_DET_EN			(1<<0)


/* Vender/Part/Revision Register */
#define REG_PART_REVISION			0x04


/* Battery Termination/Fast Charge Current Register */
#define REG_BATTERY_CURRENT			0x05
#define		CHARGE_CURRENT_SHIFT	3	/* a*75+550mA */
#define		EOC_CURRENT_SHIFT		0	/* a*50+50 mA*/


/* VIN-DPM Voltage/ DPPM Status Register */
#define REG_DPM_VOLTAGE_AND_STATUS		0x06
#define MINSYS_STATUS					(1<<7)
#define DPM_STATUS						(1<<6)
#define USB_DPM_VOLTAGE_SHIFT			3  /*a*80 +4200mV*/
#define IN_DPM_VOLTAGE_SHIFT			0  /*a*80 +4200mV*/


/* Safety Timer/ NTC Monitor Register */
#define REG_SAFETY_LIMIT				0x07
#define		TMR2X_EN				(1<<7)
#define		SAFETY_TIMER_SHIFT		5	/*00:27min, 01: 6hr, 10:9hr, 11:disable*/
#define		BATGD_EN				(1<<4)
#define		TS_EN					(1<<3)
#define		TS_FAULT_SHIFT			1	/*00:Normal, 01:T over cold, hot range charger suspend
										, 10:cold <T<cool range charger curr reduce half, 
										  11:warm<T<hot range Charge voltage reduced by 140mV */

#define DRIVER_NAME			"bq2416x"


struct bq2416x_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct notifier_block	nb;
	unsigned short		bqchip_version;
	unsigned int		current_mA;
	unsigned int		input_current_mA;
	unsigned int		reg_voltage_mV;
	unsigned int		max_current_mA;
	unsigned int		max_voltage_mV;
	unsigned int		eoc_current_mA;
	int			active;
	int			eoc_enabled;
	int			debug;
};

static int bq2416x_write_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[1];
	int ret;

	*value		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= value;
	msg[0].len	= num_bytes + 1;

	ret = i2c_transfer(di->client->adapter, msg, 1);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
#ifdef DEBUG_I2C
		int i;
	for(i=0;i<num_bytes;i++)
		printk(KERN_INFO"wr:reg%d=0x%02x\n",value[i*2],value[i*2+1]);
		return 0;
#endif
	}
	return 0;
}

static int bq2416x_read_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	buf		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= &buf;
	msg[0].len	= 1;

	msg[1].addr	= di->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= value;
	msg[1].len	= num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
#ifdef DEBUG_I2C
	int i;
	for(i=0;i<num_bytes;i++)
		printk(KERN_INFO"rd:reg%d=0x%02x\n",i+reg,value[i]);
		return 0;
#endif
	}
	return 0;
}

static int bq2416x_write_byte(struct bq2416x_device_info *di, u8 value, u8 reg)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return bq2416x_write_block(di, temp_buffer, reg, 1);
}

static int bq2416x_read_byte(struct bq2416x_device_info *di, u8 *value, u8 reg)
{
	return bq2416x_read_block(di, value, reg, 1);
}


static void bq2416x_reset_timer(struct bq2416x_device_info *di)
{
	u8 reg;
	bq2416x_read_byte(di, &reg, REG_STATUS_CONTROL);
	reg|=TIMER_RST;
	bq2416x_write_byte(di, reg, REG_STATUS_CONTROL);
	return;
}
static void bq2416x_path_select(struct bq2416x_device_info *di, int in)
{
	u8 reg;
	bq2416x_read_byte(di, &reg, REG_STATUS_CONTROL);
	if(in){
		reg&=~SUPPLY_SEL;
	}else{
		reg|=SUPPLY_SEL;
	}
	reg|=TIMER_RST;
	bq2416x_write_byte(di, reg, REG_STATUS_CONTROL);
	return;
}
static void bq2416x_set_charger_current(struct bq2416x_device_info *di,int curr)
{
	u8 curr_reg,value=0;
	bq2416x_read_byte(di, &curr_reg, REG_BATTERY_CURRENT);

	if(curr<550)curr=550;
	else if(curr>2500)curr=2500;

	value=(curr-550)/75;

	curr_reg &=~(0xF8);

	curr_reg |= (value << CHARGE_CURRENT_SHIFT);
	bq2416x_write_byte(di, curr_reg, REG_BATTERY_CURRENT);
	return;
}
static void bq2416x_set_eoc_current(struct bq2416x_device_info *di,int curr)
{
	u8 curr_reg,value=0;
	bq2416x_read_byte(di, &curr_reg, REG_BATTERY_CURRENT);
	if(curr<50)curr=50;
	else if(curr>400)curr=400;

	value=(curr-50)/50;

	curr_reg &=~(0x07);

	curr_reg |= (value << EOC_CURRENT_SHIFT);
	bq2416x_write_byte(di, curr_reg, REG_BATTERY_CURRENT);
	return;
}

static void bq2416x_set_regulation_voltage(struct bq2416x_device_info *di,int voltagemV)
{
	u8 Voreg;
	u8 value=0;
	bq2416x_read_byte(di, &Voreg, REG_BATTERY_VOLTAGE);
	if (voltagemV < 3500)
		voltagemV = 3500;
	else if (voltagemV > 4440)
		voltagemV = 4440;

	value = (voltagemV - 3500)/20;
	Voreg&=~(0xFC);
	Voreg|= (value << BATT_VOLTAGE_SHIFT);
	bq2416x_write_byte(di, Voreg, REG_BATTERY_VOLTAGE);
	return;
}

static void bq2416x_set_in_current_limit(struct bq2416x_device_info *di,int current_mA)
{
	u8 Voreg;
	bq2416x_read_byte(di, &Voreg, REG_BATTERY_VOLTAGE);
	
	if (current_mA >=2500)Voreg|=I_LIMIT_SEL_2A5;
	else Voreg&=~I_LIMIT_SEL_2A5;

	bq2416x_write_byte(di, Voreg, REG_BATTERY_VOLTAGE);
	return;
}
#if 0
static void bq2416x_force_dpdm_detection(struct bq2416x_device_info *di)
{
	u8 Voreg;
	bq2416x_read_byte(di, &Voreg, REG_BATTERY_VOLTAGE);
	
	Voreg|=DP_DM_DET_EN;
	
	bq2416x_write_byte(di, Voreg, REG_BATTERY_VOLTAGE);
	return;
}
#endif
static void bq2416x_force_reset(struct bq2416x_device_info *di)
{
	u8 Voreg;
	bq2416x_read_byte(di, &Voreg, REG_CONTROL_REGISTER);
	
	Voreg|=RESET_CHIP;
	
	bq2416x_write_byte(di, Voreg, REG_CONTROL_REGISTER);
	return;
}
static void bq2416x_set_usb_current_limit(struct bq2416x_device_info *di,int curr)
{
	u8 curr_reg,value=0;
	bq2416x_read_byte(di, &curr_reg, REG_CONTROL_REGISTER);
	curr_reg&=~RESET_CHIP;
	if(curr<=100)value=0;
	else if(curr<=150)value=1;
	else if(curr<=500)value=2;
	else if(curr<=800)value=3;
	else if(curr<=900)value=4;
	else value=7;

	curr_reg &=~(0x70);

	curr_reg |= (value << USB_CURRENT_LIMIT_SHIFT);
	
	bq2416x_write_byte(di, curr_reg, REG_CONTROL_REGISTER);
	return;
}

static void bq2416x_set_state_pin(struct bq2416x_device_info *di, int en)
{
	u8 reg;
	bq2416x_read_byte(di, &reg, REG_CONTROL_REGISTER);
	reg&=~RESET_CHIP;
	if(en)reg|=EN_STAT_OUT;
	else reg&=~EN_STAT_OUT;
	bq2416x_write_byte(di, reg, REG_CONTROL_REGISTER);
	return;
}

static void bq2416x_set_eoc_enable(struct bq2416x_device_info *di, int en)
{
	u8 reg;
	bq2416x_read_byte(di, &reg, REG_CONTROL_REGISTER);
	reg&=~RESET_CHIP;
	di->eoc_enabled=en;
	if(en)reg|=EOC_EN;
	else reg&=~EOC_EN;
	bq2416x_write_byte(di, reg, REG_CONTROL_REGISTER);
	return;
}
static void bq2416x_set_charge_disable(struct bq2416x_device_info *di, int disable)
{
	u8 reg;
	bq2416x_read_byte(di, &reg, REG_CONTROL_REGISTER);
	reg&=~RESET_CHIP;
	if(disable)reg|=CHARGE_DISABLE|HiZ_MODE;
	else reg&=~(CHARGE_DISABLE|HiZ_MODE);
	bq2416x_write_byte(di, reg, REG_CONTROL_REGISTER);
	return;
}


static void bq2416x_config_safety_reg(struct bq2416x_device_info *di,u8 reg)
{
	bq2416x_write_byte(di, reg, REG_SAFETY_LIMIT);
	return;
}
static char * str_status[]={
"No Valid Source Detected",
"AC_IN Ready",
"USB Ready",
"Charging from AC_IN",
"Charging from USB",
"Charge Done",
"NA",
"Fault"
};

static char * str_fault[]={
"NO Fault",
"Thermal Shutdown",
"Battery Temperature Fault",
"Watchdog Timer Expired",
"Safety Timer Expired",
"AC_IN Supply Fault",
"USB Supply Fault",
"Battery Fault"
};
static char * str_supply_condition[]={
"Normal",
"Supply OVP",
"Weak Source Connected (No Charging)",
"Volt<VUVLO"
};
static char * str_batt_condition[]={
"Battery Present and Normal",
"Battery OVP",
"Battery Not Present",
"NA"
};
static char * str_safe_timer[]={
"27 minute",
"6 hour",
"9 hour",
"Disabled"
};
static char * str_iusb_limit[]={
"USB2.0 host 100mA limit",
"USB3.0 host 150mA limit",
"USB2.0 host 500mA limit",
"host/charger 800mA limit"
"USB3.0 host 900mA  limit",
"NA",
"NA",
"USB host/charger 1500mA limit"
};
static char * str_ts_fault[]={
"No TS fault",
"TS temp < TCOLD or TS temp > THOT",
"TCOOL > TS temp > TCOLD ",
"TWARM < TS temp < THOT"
};
static int bq2416x_charger_debug(struct bq2416x_device_info *di)	{
	int i;
	u8 read_reg[8];
	bq2416x_read_block(di, &read_reg[0], 0, 8);
	bci_dbg(debug_ext_charger, "\n----====begin: bq2416x: debug info ====----\n");
	for(i=0;i<8;i++){
		bci_dbg(debug_ext_charger, "rd:reg%d=0x%02x\n",i,read_reg[i]);
		switch(i){
		case REG_STATUS_CONTROL:
			bci_dbg(debug_ext_charger, "SUPPLY_SEL:%s, STATUS:%s, FAULT:%s\n",read_reg[i]&SUPPLY_SEL?"USB":"AC_IN",
					str_status[(read_reg[i]>>STATUS_SHIFT)&0x7],str_fault[(read_reg[i])&0x7]);
			break;
		case REG_BATT_SUPPLY_REGISTER:
			bci_dbg(debug_ext_charger, "IN:%s, USB:%s, BATT:%s\n",
				str_supply_condition[(read_reg[i]>>IN_STATUS_SHIFT)&0x3],
				str_supply_condition[(read_reg[i]>>USB_STATUS_SHIFT)&0x3],
				str_batt_condition[(read_reg[i]>>BAT_STATUS_SHIFT)&0x3]);
			break;
		case REG_CONTROL_REGISTER:
			bci_dbg(debug_ext_charger, "%s, %s STAT output, %s EOC, %s Charge, %s HiZ\n",
				str_iusb_limit[(read_reg[i]>>USB_CURRENT_LIMIT_SHIFT)&0x7],
					(read_reg[i]&EN_STAT_OUT)?"Enable":"Disable",
					(read_reg[i]&EOC_EN)?"Enable":"Disable",
					(read_reg[i]&CHARGE_DISABLE)?"Disable":"Enable",
					(read_reg[i]&CHARGE_DISABLE)?"Enable":"Disable");
			break;
		case REG_BATTERY_VOLTAGE:
			bci_dbg(debug_ext_charger, "Regulation Voltage %dmV, In Curr limit: %s\n",
					(read_reg[i]>>BATT_VOLTAGE_SHIFT)*20+3500,
					(read_reg[i]&I_LIMIT_SEL_2A5)?"2.5A":"1.5A");
			break;
		case REG_PART_REVISION:
				//printk(KERN_INFO"\n");
			break;
		case REG_BATTERY_CURRENT:
			bci_dbg(debug_ext_charger, "Charge Curr: %dmA, Eoc Curr:%dmA\n",
					(read_reg[i]>>CHARGE_CURRENT_SHIFT)*75+550,
					(read_reg[i]&0x07)*50+50);
			break;
		case REG_DPM_VOLTAGE_AND_STATUS:
			bci_dbg(debug_ext_charger, "MINSYS_STATUS: %s , DPM_STATUS:%s, USB DPM %dmV, IN DPM %dmV\n",
					(read_reg[i]&EN_STAT_OUT)?"Activated":"NA",
					(read_reg[i]&DPM_STATUS)?"Activated":"NA",
					((read_reg[i]>>USB_DPM_VOLTAGE_SHIFT)&0x07)*80+4200,
					((read_reg[i])&0x07)*80+4200);
			break;
		case REG_SAFETY_LIMIT:
			bci_dbg(debug_ext_charger, "TMR2X: %s, BATGD:%s, Temp Sense: %s, safe timer: %s, %s\n",
					(read_reg[i]&TMR2X_EN)?"Enable":"Disable",
					(read_reg[i]&BATGD_EN)?"Enable":"Disable",
					(read_reg[i]&TS_EN)?"Enable":"Disable",
					str_safe_timer[((read_reg[i]>>SAFETY_TIMER_SHIFT)&0x03)],
					str_ts_fault[((read_reg[i]>>TS_FAULT_SHIFT)&0x03)]);
			break;
		}
				
	}
	bci_dbg(debug_ext_charger, "----====end: bq2416x: debug info ====----\n\n");
	return 0;				
}
static void  bq2416x_charger_config(struct bq2416x_device_info *di){
	/*bq2416x registers will reset to default value after disable or fault,
	but bci assume the registers config will apply to charger and not change
	except reconfig, the driver should handle by itself.*/
	bq2416x_force_reset(di);
	bq2416x_set_state_pin(di,0);
	bq2416x_config_safety_reg(di,TMR2X_EN|BATGD_EN|TS_EN|(2<<SAFETY_TIMER_SHIFT));
	bq2416x_set_charger_current(di, di->current_mA);
	bq2416x_set_regulation_voltage(di,di->reg_voltage_mV);
	//bq2416x_set_eoc_enable(di, di->eoc_enabled);
	bq2416x_set_eoc_enable(di, 1);
	//if(di->eoc_enabled){
		bq2416x_set_eoc_current(di,50);
	//}
	//if(di->eoc_enabled){
		//bq2416x_set_eoc_current(di,di->eoc_current_mA);
	//}
	if(di->input_current_mA<2500){
	/*select usb path*/
		bq2416x_set_usb_current_limit(di, di->input_current_mA);
		bq2416x_path_select(di,0);
	}else{
	/*select in path*/
		bq2416x_set_in_current_limit(di,di->input_current_mA);
		bq2416x_path_select(di,1);
	
	}
}
static const char* state_str[]={
"unknown",
"charging",
"discharge",
"n_charge",
"full",
};
static int bq2416x_charger_event(struct notifier_block *nb, unsigned long event,
				void *_data)
{

	struct bq2416x_device_info *di;
	u8 read_reg[8] = {0};
	int ret = NOTIFY_DONE;
	if(!(event&BCI_EXTERNAL_CHARGER))return ret;
	event&=~BCI_TYPE_MASK;
	di = container_of(nb, struct bq2416x_device_info, nb);
	
	switch(event){
		case BCI_START_STOP_CHARGING:
			if(_data){
				unsigned int *curr=_data;
				bci_dbg(debug_ext_charger, "BCI_SET_START_CHARGING\n");
				di->current_mA=*curr;
				if  (di->active == 0) {
					bq2416x_charger_config(di);	
					bq2416x_set_charge_disable(di, 0);
					di->active = 1;
				}
			}else{
				bci_dbg(debug_ext_charger,"BCI_SET_STOP_CHARGING\n");
				if  (di->active == 1) {
					bq2416x_set_charge_disable(di, 1);
					di->active = 0;
				}

			}

		break;

		case BCI_RESET_TIMER:
			if(di->debug>2)
			dev_info(di->dev, "BCI_RESET_TIMER\n");
			//if (di->active == 1) 
				{
				/* reset 32 second timer */
				bq2416x_reset_timer(di);
			}
		break;
		case BCI_SET_CHARGE_CURRENT:
			{
				unsigned int *current_mA=_data;
				bci_dbg(debug_ext_charger,"BCI_SET_CHARGE_CURRENT %dmA\n",*current_mA);
				
				bq2416x_set_charger_current(di, *current_mA);
			}
		break;
		case BCI_SET_EOC_CURRENT:
			{
				unsigned int *current_mA=_data;
				bci_dbg(debug_ext_charger,"BCI_SET_EOC_CURRENT %dmA\n",*current_mA);
				bq2416x_set_eoc_current(di,*current_mA);

			}
		break;
		case BCI_SET_INPUT_CURRENT_LIMIT:
			{
				unsigned int *current_mA=_data;
				bci_dbg(debug_ext_charger,"BCI_SET_INPUT_CURRENT_LIMIT %dmA\n",*current_mA);
				di->input_current_mA=*current_mA;
				bq2416x_charger_config(di);
				if  (di->active) {
					bq2416x_set_charge_disable(di, 0);
				}
					
			}
		break;
		case BCI_CONFIG:
			{
				struct bci_config_charger *cfg=_data;
				bci_dbg(debug_ext_charger,"BCI_CONFIG \n");
				
				di->reg_voltage_mV=cfg->battery_regulation_voltage;
				di->input_current_mA=cfg->input_current;/*limit to the chager input current, runtime changeable*/
				di->max_current_mA=cfg->max_input_current;/*abs the chager input current limit, hw limitation*/
				di->max_voltage_mV=cfg->max_input_voltage;/*abs the chager input voltage limit, hw limitation*/
				di->current_mA=cfg->charge_current;/*the current charge to battery in CC mode */
				di->eoc_current_mA=cfg->eoc_current;/*the eoc current */
				di->eoc_enabled=cfg->eoc_enable;/*the eoc default is enable or disable */	
				
				bq2416x_charger_config(di);

				
			}
		break;

		case BCI_ENABLE_EOC:
			if(_data){
				bci_dbg(debug_ext_charger,"BCI_SET_EOC_ENABLE\n");
				bq2416x_set_eoc_enable(di,1);
			}else{
				bci_dbg(debug_ext_charger,"BCI_SET_EOC_DISABLE\n");
				
				bq2416x_set_eoc_enable(di,0);
			}
			
		break;
		
		case BCI_READ_FAULT_REASON:
			{
				struct bci_charger_read_fault *status=_data;
				u8 state;
				bq2416x_read_block(di, &read_reg[0], 0, 8);
				state=(read_reg[0]>>STATUS_SHIFT)&0x07;
				status->reasons=0;
				status->continue_charge=1;
				bq2416x_charger_debug(di);
				switch(state){
					case 0:
						//status->status=POWER_SUPPLY_STATUS_DISCHARGING;
					break;
					
					case 3:
					case 4:
						status->status=POWER_SUPPLY_STATUS_CHARGING;
					break;
					case 5:
						status->status=POWER_SUPPLY_STATUS_FULL;
					break;
					default:
						status->status=POWER_SUPPLY_STATUS_NOT_CHARGING;
						status->reasons=read_reg[0]&0x07;
						bci_dbg(debug_common,"reason %x, hw %02x, safety %02x\n",status->reasons,read_reg[1],read_reg[7]);
						if ((di->active)&&((read_reg[0]&0x7)==7)){
							int delta=di->reg_voltage_mV-status->voltage;
							if(delta>0){
								bq2416x_set_charge_disable(di, 1);
								bq2416x_charger_config(di);
								bq2416x_set_regulation_voltage(di,
								(delta>di->reg_voltage_mV*101/100)?
								di->reg_voltage_mV*101/100:delta+di->reg_voltage_mV);
								bq2416x_set_charge_disable(di, 0);
							
							}else{
								bq2416x_set_regulation_voltage(di,di->reg_voltage_mV);
							}
						}
						if ((di->active)&&((read_reg[0]&0x7)==3)){
							bq2416x_set_charge_disable(di, 1);
							bq2416x_charger_config(di);
							bq2416x_set_charge_disable(di, 0);
		
						}
					break;
				}
				bq2416x_reset_timer(di);
			}
		break;
		case BCI_CHAEGER_STATUS_REQUEST:
			{
				struct bci_charger_status_requset *status=_data;
				u8 state;
				
				bq2416x_read_block(di, &read_reg[0], 0, 8);
				state=(read_reg[0]>>STATUS_SHIFT)&0x07;
				status->previous_state=status->current_state;
				switch(state){
					case 0:
						//status->current_state=POWER_SUPPLY_STATUS_DISCHARGING;
					break;
					
					case 3:
					case 4:
						status->current_state=POWER_SUPPLY_STATUS_CHARGING;
						if ((di->active)&&(status->voltage>=di->reg_voltage_mV))
							if((status->capacity>=99)&&(status->capacity>=0)&&
								(status->current_uA<=di->eoc_current_mA*1000)){
								status->current_state=POWER_SUPPLY_STATUS_FULL;
								bci_dbg(debug_ext_charger,"bq2416x: Fake FULL %d\n", state);
								break;
							
						}
					break;
					case 5:
						status->current_state=POWER_SUPPLY_STATUS_FULL;
					break;
					default:

						if((di->active)&&((read_reg[0]&0x7)==7)){
							if(status->voltage>=di->reg_voltage_mV){
								status->current_state=POWER_SUPPLY_STATUS_FULL;
								bci_dbg(debug_ext_charger,"bq2416x: Fake FULL %d\n", state);
								break;
							}
						
						}
						status->current_state=POWER_SUPPLY_STATUS_NOT_CHARGING;
						bq2416x_charger_debug(di);
					break;
				}
				bq2416x_reset_timer(di);
				bci_dbg(debug_continuous_info,"bq2416x: state= %s, %s\n", str_status[state],state_str[status->current_state]);
				if((status->previous_state!=status->current_state)&&
					(status->previous_state!=POWER_SUPPLY_STATUS_FULL))
					bci_dbg(debug_common,"bq2416x: state= %d, %s <- %s\n", state,
					state_str[status->current_state],state_str[status->previous_state]);
			}
		break;
		 case BCI_RESET:
		 	bci_dbg(debug_ext_charger,"BCI_RESET\n");
		 	bq2416x_force_reset(di);
		 	break;
			
	}
	if(di->debug > 0)
		bq2416x_charger_debug(di);
	return ret;
}
static int __devinit bq2416x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2416x_device_info *di;
	//struct bq2416x_platform_data *pdata = client->dev.platform_data;
	int ret;
	u8 read_reg = 0;  
	printk("-------======= bq2416x_charger_probe() here =======------- \n");

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &client->dev;
	di->client = client;
	
	i2c_set_clientdata(client, di);

	ret = bq2416x_read_byte(di, &read_reg, REG_PART_REVISION);

	if (ret < 0) {
		dev_err(&client->dev, "chip not present at address %x\n",
								client->addr);
		ret = -EINVAL;
		goto err_kfree;
	}

	if ((read_reg & 0x18) == 0x00 && (client->addr == 0x6B))
		di->bqchip_version = BQ24160;


	if (di->bqchip_version == 0) {
		dev_dbg(&client->dev, "unknown bq chip\n");
		dev_dbg(&client->dev, "Chip address %x", client->addr);
		dev_dbg(&client->dev, "bq chip version reg value %x", read_reg);
		ret = -EINVAL;
		goto err_kfree;
	}

	di->nb.notifier_call = bq2416x_charger_event;
	bq2416x_config_safety_reg(di,TMR2X_EN|BATGD_EN|TS_EN|(2<<SAFETY_TIMER_SHIFT));
	di->active = 0;
	di->eoc_enabled= 0;
	di->debug=0;

//	ret = sysfs_create_group(&client->dev.kobj, &bq2416x_attr_group);
	if (ret)
		dev_dbg(&client->dev, "could not create sysfs files\n");
	
//	bq2416x_set_charge_disable(di, 1);

    	ret = twl6030_bci_function_register(BQ2416x,&di ->nb,
			SUPPORT_FUNC,BCI_EXTERNAL_CHARGER,di);

	bq2416x_charger_debug(di);

	dev_info(&client->dev, "bq2416x charger was found!!\n");
	printk("-------======= bq2416x charger register successfully !! =======------- \n");
	return 0;

err_kfree:
	kfree(di);

	return ret;
}

static int __devexit bq2416x_charger_remove(struct i2c_client *client)
{
	struct bq2416x_device_info *di = i2c_get_clientdata(client);
	twl6030_bci_function_unregister(BQ2416x);
	kfree(di);
	return 0;
}

static const struct i2c_device_id bq2416x_id[] = {
	{ BQ2416x, 0 },
	{},
};

static struct i2c_driver bq2416x_charger_driver = {
	.probe		= bq2416x_charger_probe,
	.remove		= __devexit_p(bq2416x_charger_remove),
	.id_table	= bq2416x_id,
	.driver		= {
		.name	= BQ2416x,
	},
};

static int __init bq2416x_charger_init(void)
{
	printk("-------======= bq2416x_charger_init() =======------- \n");
	return i2c_add_driver(&bq2416x_charger_driver);
}
module_init(bq2416x_charger_init);
  
static void __exit bq2416x_charger_exit(void)
{
	i2c_del_driver(&bq2416x_charger_driver);
}
module_exit(bq2416x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
