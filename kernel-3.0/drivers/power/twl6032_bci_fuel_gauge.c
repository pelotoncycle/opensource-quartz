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
#include <linux/usb/otg.h>
#include <asm/mach-types.h>

#define SUPPORT_FUNC (BCI_FUELGAUGE_STATUS_REQUEST|BCI_READ_CURRENT|BCI_SHUTDOWN)
		
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

/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};
static const unsigned int fuelgauge_time[4] = {250000, 62500, 15600, 3900};

#define EOD_COUNT 30
#define FULL_CONSUME 30

struct twl6030_bci_fuel_gauge_info {
	struct device		*dev;
	struct ntc_res_temp_desc *battery_tmp_tbl_603x;
	struct charge_cv_profile	*battery_cv_profile;
	int			voltage_mV;
	int			temp_voltage_mV;
	int			current_uA;
	int			current_avg_uA;
	int			current_suspend_uA;
	int			temp_C;
	int			charge_status;
	int			charge_prev_status;
	int			fuelgauge_mode;
	u16			current_avg_interval;
	u16			monitoring_interval;
	 int		min_vbus;
	 int		charger_incurrentmA;
	 int		charger_outcurrentmA;
	 int		regulation_voltagemV;
	 int		low_bat_voltagemV;
	 int		termination_currentmA;
	 int		capacity;
	 int		last_update;
	 int		battery_esr;
	struct Loading_Table* loading_Table;
	struct ntc_res_temp_desc *battery_tmp_tbl;

	int			fg_timer;
	 int			fg_timer_prev;
	int			fg_timer_init;
	s32			fg_acc;
	s32			fg_acc_prev;
	s32			fg_acc_init;
	s16			fg_cc_offset;
	int			fg_timer_avg;
	s32			fg_acc_avg;
	int			fg_timer_suspend;
	s32			fg_acc_suspend;

	int			fg_full_uAh;
	int			fg_acc_uAh;
	int			fg_init_acc_uAh;
	int			fg_full_eod;
	int			fg_full_consume;	//for capacity=1 and accumulate <0
	int			fg_full_charged;
	int			fg_remain_uAh;
	int			fg_reserved_uAh;
	int			fg_hidden_uAh;
	int			fg_last_delta_uAh;
	int			fg_chip_acc_uAh;
	int			fg_chip_acc_uAh_prev;
	int			fg_acc_delta;
	int			fg_resistance;
	int			regressin_Voltage_mV;

	struct notifier_block	nb;
    	
	unsigned long		features;
		   
	int temp_Rx;
	int temp_Ry;

	int initialed;
};

/*update capacity and print log*/
static void twl6030_store_capacity(struct twl6030_bci_fuel_gauge_info *fi, int cap) {
    twl6030_write_last_capacity(cap);
    printk("vol=%d, cap=%d,tmp=%d,curr=%d\n", fi->voltage_mV,fi->capacity,fi->temp_C,fi->current_uA/1000);
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
#if 0
static int  temp_to_resistance(struct ntc_res_temp_desc *ntc_table,int  temp)
{
    int ret_res,i;
    for (i=0;i<ntc_table->res;i++) {
        if (temp<=ntc_table[i].temp*10) {
            ret_res=(temp-ntc_table[i].temp*10)*(ntc_table[i-1].res-ntc_table[i].res)/((ntc_table[i-1].temp-ntc_table[i].temp)*10)
                    +ntc_table[i].res;
            return ret_res;
        }
    }
    return -EINVAL;
}
#endif
static int loadingVoltToCap(struct Loading_Table *index,int  currentNow, int voltageNow,int esr)
{
    int j,current1,current2;
    int res;
    struct batt_volt_cap_desc newArray[BATT_VOLT_CAP_SIZE];
    struct batt_volt_cap_desc *table1,*table2;

    if (currentNow>0)currentNow=0;
	voltageNow-=currentNow*esr/1000;
    {
        for (j=0;((index[j+1].batt_volt_cap!=NULL)&&(index[j].curr>=currentNow));j++);

        current1=index[j-1].curr;
        current2=index[j].curr;
        table1=index[j-1].batt_volt_cap;
        table2=index[j].batt_volt_cap;


        for (j=0;j<BATT_VOLT_CAP_SIZE;j++)
        {

            int a=(table2[j].volt-table1[j].volt)*100/(current2-current1);
            int b=(current2*table1[j].volt-current1*table2[j].volt)*100/(current2-current1);
            newArray[j].cap=table1[j].cap;
            newArray[j].volt=(currentNow*a+b)/100;
        }


        for (j=0;j<BATT_VOLT_CAP_SIZE;j++)
        {
            if (newArray[j].volt==voltageNow)
            {
                res = newArray[j].cap;
                return res;
            }
            if (newArray[j].volt > voltageNow)
                break;
        }
        if (j == 0)		res = newArray[j].cap;
        else if (j == BATT_VOLT_CAP_SIZE)	res = newArray[j-1].cap;
        else
        {
            res = ((voltageNow-newArray[j-1].volt)*
                   (newArray[j].cap-newArray[j-1].cap));
            res = res / (newArray[j].volt-newArray[j-1].volt);
            res += newArray[j-1].cap;
        }

    }

    if (res<0)res=0;
    if (res>100)res=100;
    return res;
}
static int twl6030_fg_avg_sample_current_uA(struct twl6030_bci_fuel_gauge_info *fi,
        int delta_acc,int delta_counter ) {

    int cc_offset= fi->fg_cc_offset * delta_counter;
    int factor=((62000000/fi->fg_resistance)*fuelgauge_rate[fi->fuelgauge_mode]/delta_counter)>>15;
    //printk("delta_acc = %d, offset = %d, factor = %d, \n",delta_acc,  cc_offset, factor);
    return (delta_acc-cc_offset)*factor;
}
static int twl6030_fg_update(struct twl6030_bci_fuel_gauge_info *fi )
{
    s32 samples = 0,samples_avg=0;
    s16 cc_offset = 0;//,cc_offset_avg=0;
    //int current_avg_uA=0;
    int time_interval;
    int ret = 0;
    u16 read_value = 0;
    //int current_now = 0;
    /* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
    ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
                       FG_REG_10, 2);
    if (ret < 0) {
        bci_dbg( debug_common,"failed to read FG_REG_10: current_now\n");
        return -1;
    }


    fi->fg_acc_prev = fi->fg_acc;
    fi->fg_timer_prev = fi->fg_timer;
    fi->fg_chip_acc_uAh_prev=fi->fg_chip_acc_uAh;

    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fi->fg_timer,
                 FG_REG_01, 3);
    /*
     * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
     * accumulates instantaneous current value
     */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fi->fg_acc,
                 FG_REG_04, 4);
    /* FG_REG_08, 09 is 10 bit signed calibration offset value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &cc_offset,
                 FG_REG_08, 2);
    fi->fg_cc_offset= ((s16)(cc_offset << 6) >> 6);

    /*update avg current*/
    samples_avg = fi->fg_timer - fi->fg_timer_avg;
    /* check for timer overflow */
    if (fi->fg_timer < fi->fg_timer_avg)
        samples_avg = samples_avg + (1 << 24);

    if (samples_avg/fuelgauge_rate[fi->fuelgauge_mode]>=fi->current_avg_interval) {

        fi->current_avg_uA =twl6030_fg_avg_sample_current_uA(fi,
                            fi->fg_acc - fi->fg_acc_avg,samples_avg);

        fi->fg_timer_avg=fi->fg_timer;
        fi->fg_acc_avg=fi->fg_acc;

    }

    /*update Acc*/

    samples = fi->fg_timer - fi->fg_timer_prev;
    if (samples) {
        /* check for timer overflow */
        if (fi->fg_timer < fi->fg_timer_prev)
            samples = samples + (1 << 24);

        time_interval=samples/fuelgauge_rate[fi->fuelgauge_mode];
        fi->fg_acc_delta =twl6030_fg_avg_sample_current_uA(fi,
                          fi->fg_acc - fi->fg_acc_prev,samples)*time_interval/3600;
        bci_dbg(debug_int_fuel_gauge ,"fi->fg_acc_delta = %d, fi->fg_acc=%d, fi->fg_acc_prev= %d\n", fi->fg_acc_delta, fi->fg_acc, fi->fg_acc_prev);
    }


	fi->current_uA = twl6030_fg_avg_sample_current_uA(fi,
                     (s16)(read_value << 2) >> 2,1);


    bci_dbg(debug_int_fuel_gauge,"curr=%dmA, avg_curr=%d mA,acc_delta=%d uAh\n",
            fi->current_uA/1000,fi->current_avg_uA/1000,fi->fg_acc_delta);
    bci_dbg(debug_int_fuel_gauge,"chip=0x%08x, chipTime=%d sec\n",
            fi->fg_acc,fi->fg_timer/fuelgauge_rate[fi->fuelgauge_mode]);


    return 0;
}

static void twl6030_fg_mode_changed(struct twl6030_bci_fuel_gauge_info *fi)
{
    int timer,acc;
    /* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &timer,
                 FG_REG_01, 3);
    /*
     * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
     * accumulates instantaneous current value
     */
    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &acc,
                 FG_REG_04, 4);
    fi->fg_timer_prev=timer;
    fi->fg_timer=timer;
    fi->fg_acc=acc;
    fi->fg_acc_prev=acc;
    fi->fg_timer_avg=timer;
    fi->fg_acc_avg=acc;

}
static void twl6030_fg_set_mode(struct twl6030_bci_fuel_gauge_info *fi,
                                int clean, int pause, int sample_mode,int auto_cal)
{
    u8 value=0;
    twl6030_fg_update(fi);
    if ((sample_mode<4)&&(sample_mode>=0)) {
        value=sample_mode<<CC_ACTIVE_MODE_SHIFT;
        fi->fuelgauge_mode=sample_mode;
    } else value=fi->fuelgauge_mode<<CC_ACTIVE_MODE_SHIFT;

    if (clean) value|=CC_AUTOCLEAR;
    if (auto_cal) value|=CC_CAL_EN;
    if (pause) value|=CC_PAUSE;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, value,FG_REG_00);
    twl6030_fg_mode_changed(fi);
    return;
}

static void twl6030_fg_sync(struct twl6030_bci_fuel_gauge_info *fi)
{
    int acc;
    int factor= fuelgauge_rate[fi->fuelgauge_mode]*3600;

    acc=((fi->fg_acc_uAh/125)<<6)*factor/(375);
    printk("Sync CAR: %d uAh, acc=0x%08x \n",fi->fg_acc_uAh,acc);
    twl6030_fg_set_mode(fi,1,0,-1,0);
    twl6030_store_capacity(fi, fi->capacity);

    return;
}


static int twl6030_fg_calculate_capacity(struct twl6030_bci_fuel_gauge_info *fi)
{

    int full_uAh = fi->fg_full_uAh;
    int R= full_uAh/200;// full/100=>1%, full/200=>0.5%, for round off
    int capacity = ((fi->fg_acc_uAh + fi->fg_hidden_uAh - fi->fg_reserved_uAh +R ) *100)/ (full_uAh - fi->fg_reserved_uAh);


    //printk("fg_acc_uAh = %d , fg_hidden_uAh = %d , fg_reserved_uAh = %d\n", fi->fg_acc_uAh, fi->fg_hidden_uAh, fi->fg_reserved_uAh);

    if (capacity >= 100) capacity= 100;
    if (capacity <= 1)	capacity= 1;


    if (fi->fg_full_eod>=EOD_COUNT) {
        printk("-------=======  eod>=%d, return capacity 0 =======-------- \n", EOD_COUNT);
        capacity= 0;

    } else if (fi->fg_full_eod>=5 && fi->capacity==1) {
        printk("-------=======  eod>=5 & cap = 1, return capacity 0 =======-------- \n");
        capacity= 0;

    } else if (fi->fg_full_consume>= FULL_CONSUME) {
        //printk("-------=======  full consume>=%d, return capacity 0 =======-------- \n", FULL_CONSUME);
        //capacity= 0;
    }
    return capacity;
}
static int twl6030_fg_calculate_uAh_from_capacity(struct twl6030_bci_fuel_gauge_info *fi)
{
    return (fi->capacity*(fi->fg_full_uAh - fi->fg_reserved_uAh)/100)+
           fi->fg_reserved_uAh-fi->fg_hidden_uAh;
}
static void twl6030_fg_discharge_regression(struct twl6030_bci_fuel_gauge_info *fi)
{
    if ((fi->voltage_mV> fi->low_bat_voltagemV)&&(fi->fg_full_eod<EOD_COUNT)) {
        int		cap1=0, cap2=0;

        bci_dbg(debug_int_fuel_gauge ,"\n==> \ndi->voltage_mV = %d, fi->regressin_Voltage_mV=%d, fi->current_uA=%d\n", fi->voltage_mV, fi->regressin_Voltage_mV, fi->current_uA);
        if (fi->voltage_mV <fi->regressin_Voltage_mV) {/*Only lower then some voltage*/
            cap1=loadingVoltToCap(fi->loading_Table,fi->current_uA/1000,fi->voltage_mV,fi->battery_esr);
            cap2=loadingVoltToCap(fi->loading_Table,fi->current_uA/1000,fi->low_bat_voltagemV,fi->battery_esr);
            bci_dbg(debug_int_fuel_gauge ,"cap1 = %d, cap2=%d, fi->fg_reserved_uAh = %d\n", cap1, cap2, fi->fg_reserved_uAh);
            fi->fg_remain_uAh = fi->fg_reserved_uAh+ (cap1 - cap2)* fi->fg_full_uAh / 100;
            bci_dbg(debug_int_fuel_gauge ,"fi->fg_remain_uAh = %d\n", fi->fg_remain_uAh);
        } else {
            fi->fg_remain_uAh=fi->fg_acc_uAh;
            bci_dbg(debug_int_fuel_gauge ,"fi->fg_remain_uAh = %d\n", fi->fg_remain_uAh);
        }

        fi->fg_last_delta_uAh=fi->fg_acc_uAh-fi->fg_remain_uAh;
        bci_dbg(debug_int_fuel_gauge ,"fi->fg_last_delta_uAh = %d\n", fi->fg_last_delta_uAh);

        bci_dbg(debug_int_fuel_gauge ,"fi->fg_hidden_uAh = %d\n", fi->fg_hidden_uAh);
        if (((fi->fg_hidden_uAh<0)&&(fi->fg_last_delta_uAh>0))||
                ((fi->fg_hidden_uAh>0)&&(fi->fg_last_delta_uAh<0))) {

            fi->fg_hidden_uAh+=fi->fg_last_delta_uAh;
            fi->fg_acc_uAh-=fi->fg_last_delta_uAh;
        }
        else if (((fi->fg_hidden_uAh<0)&&(fi->fg_last_delta_uAh<0))||
                 ((fi->fg_hidden_uAh>0)&&(fi->fg_last_delta_uAh>0))) {

            fi->fg_hidden_uAh-=fi->fg_last_delta_uAh;
            fi->fg_acc_uAh+=fi->fg_last_delta_uAh;
        }

        bci_dbg(debug_int_fuel_gauge ,"fi->fg_acc_uAh = %d, fi->fg_remain_uAh=%d, fi->fg_acc_delta=%d\n", fi->fg_acc_uAh, fi->fg_remain_uAh, fi->fg_acc_delta);
        if (fi->fg_acc_uAh > fi->fg_remain_uAh) {
			int cap_delta=cap1 - cap2;
			if((cap_delta<10)&&(cap_delta>1))
		 		fi->fg_acc_uAh += (int)(cap_delta*fi->fg_acc_delta);
			else
				fi->fg_acc_uAh += (int)((180*fi->fg_acc_delta)/100);

        } else if (fi->fg_remain_uAh > fi->fg_acc_uAh + fi->fg_full_uAh/100) {
            /*Skip*/

        } else {
            fi->fg_acc_uAh += fi->fg_acc_delta;

        }

        fi->fg_full_eod=0;
    }
    else { // < bat low voltage
        int delta=fi->fg_full_uAh/2000; // 0.05 %
        fi->fg_acc_uAh -= delta;

        fi->fg_full_eod++;
        if (fi->fg_full_eod>=EOD_COUNT) {
            fi->fg_acc_uAh=fi->fg_reserved_uAh;
            fi->fg_hidden_uAh=0;
            printk("EOD: curr=%d, volt=%d temp=%d, eod = %d\n", fi->current_uA,fi->voltage_mV,fi->temp_C, fi->fg_full_eod);
        }

        printk("eod = %d \n",fi->fg_full_eod);
    }


}

static void twl6030_fg_charge_process(struct twl6030_bci_fuel_gauge_info *fi)
{
    int charger_incurrentuA=fi->charger_incurrentmA*1000;
    int compensate_voltuV = fi->regulation_voltagemV*988/1000;

    fi->fg_acc_uAh += fi->fg_acc_delta;

    bci_dbg(debug_int_fuel_gauge ,"di->current_uA = %d, charger_incurrentuA=%d\n",
		fi->current_uA, charger_incurrentuA);
	
    if ((fi->current_uA>0)&&(charger_incurrentuA>fi->current_uA)) {

        int delta_samples=fi->fg_timer-fi->fg_timer_prev,delta_time_sec,delta_uAh;
        if (fi->fg_timer < fi->fg_timer_prev)
            delta_samples = delta_samples + (1 << 24);

        delta_time_sec=delta_samples/fuelgauge_rate[fi->fuelgauge_mode];
        delta_uAh=(delta_time_sec*(charger_incurrentuA-fi->current_uA))/3600;
        bci_dbg(debug_int_fuel_gauge ,"fi->fg_hidden_uAh = %d, fi->fg_last_delta_uAh=%d\n",
			fi->fg_hidden_uAh, fi->fg_last_delta_uAh);
        bci_dbg(debug_int_fuel_gauge ,"fi->voltage_mV = %d, fi->regulation_voltagemV=%d\n",
			fi->voltage_mV, fi->regulation_voltagemV);


        // 1. consume hidden and last delta
        if ((fi->fg_hidden_uAh<0)||(fi->fg_last_delta_uAh<0)) {
            if (fi->fg_hidden_uAh<0) {
                fi->fg_hidden_uAh+=delta_uAh;
                bci_dbg(debug_int_fuel_gauge,"Release hidden:%d uAh, hidden=%d mAh\n",
					delta_uAh,fi->fg_hidden_uAh/1000);
				
                if (fi->fg_hidden_uAh>0)fi->fg_hidden_uAh=0;
            } else if (fi->fg_last_delta_uAh<0) {
                if (fi->fg_hidden_uAh>0) {
                    fi->fg_acc_uAh+=fi->fg_hidden_uAh;
                    bci_dbg(debug_int_fuel_gauge,"Release hidden:%d uAh, Acc=%d mAh\n",
						fi->fg_hidden_uAh,fi->fg_acc_uAh/1000);
					
                    fi->fg_hidden_uAh=0;
                }
                fi->fg_last_delta_uAh+=delta_uAh;
                fi->fg_acc_uAh +=delta_uAh;
                bci_dbg(debug_int_fuel_gauge,"Release last delta:%d uAh, last=%d mAh\n",
					delta_uAh,fi->fg_last_delta_uAh/1000);
				
                if (fi->fg_last_delta_uAh>0)fi->fg_last_delta_uAh=0;

            }
            if (fi->fg_acc_uAh>fi->fg_full_uAh)fi->fg_acc_uAh=fi->fg_full_uAh-1000;
        }

        // 2. bat voltage > regulation voltage
        //else if  (fi->voltage_mV >= (fi->regulation_voltagemV-(fi->regulation_voltagemV*2/100))){
        if  (fi->voltage_mV >= compensate_voltuV) {

            int termination_currentuA=fi->termination_currentmA*1000;
		int factor=0,  delta_uAh=0;
		int current_uA=(fi->current_uA>0)?fi->current_uA:0;
		int current_mA=current_uA/1000;

            /*After the hidden and last delta consumed */
            bci_dbg(debug_int_fuel_gauge ,"fi->current_uA = %d, termination_currentmA=%d\n",
            		fi->current_uA, termination_currentuA);



            // a. current >= termination
            if (fi->current_uA >= termination_currentuA) {
               int curr_status;


                bci_dbg(debug_int_fuel_gauge ,"charger_incurrentuA = %d, termination_currentmA=%d,\n",
					charger_incurrentuA, termination_currentuA);
		   if (current_mA<=fi->battery_cv_profile[0].curr){
			for(curr_status=0;fi->battery_cv_profile[curr_status].cap<100;curr_status++){
				if (fi->battery_cv_profile[curr_status].curr<=current_mA) break;
			}

			if(fi->capacity<fi->battery_cv_profile[curr_status].cap){
				factor=fi->battery_cv_profile[curr_status].cap-fi->capacity;
			}
		   }

                if (fi->fg_acc_uAh>fi->fg_full_uAh) {
                    if (fi->fg_hidden_uAh>0)fi->fg_hidden_uAh=0;
                    fi->fg_acc_uAh=fi->fg_full_uAh-1000;
                }
				
            }

            // b. current < termination
            else {
                if (fi->capacity==100) {
                    fi->charge_status =POWER_SUPPLY_STATUS_FULL;
                    factor=0;
                    if (fi->fg_hidden_uAh)fi->fg_hidden_uAh=0;
                }else{
                	/*Current < termination, but not full
				speed up*/
			factor=(100-fi->capacity)*2;
                }
            }
		if(factor)
			delta_uAh=(delta_time_sec*(termination_currentuA+current_uA)*factor)/3600;
			

            fi->fg_acc_uAh+=delta_uAh;
            if (fi->fg_hidden_uAh>0)fi->fg_hidden_uAh-=delta_uAh;
            bci_dbg(debug_int_fuel_gauge,"Compensate:%d uAh Acc=%d mAh, Hidden=%d mAh\n",
                    delta_uAh,fi->fg_acc_uAh/1000,fi->fg_hidden_uAh/1000);
        }

        // 3. hidden >0
        else if (fi->fg_hidden_uAh>0) {
            fi->fg_acc_uAh+=fi->fg_hidden_uAh;
            bci_dbg(debug_int_fuel_gauge,"Release hidden:%d uAh, Acc=%d mAh\n",
				fi->fg_hidden_uAh,fi->fg_acc_uAh/1000);
            fi->fg_hidden_uAh=0;
        }

    }

    if (fi->fg_acc_uAh>fi->fg_full_uAh)fi->fg_acc_uAh=fi->fg_full_uAh;

}



static void twl6030_fg_calculate_init_capacity(struct twl6030_bci_fuel_gauge_info *fi) {

    /*
    ICOM_DEVOFF_BCK	 (1 << 2)	// DEVOFF_BCK in PHOENIX_LAST_TURNOFF_STS
    ICOM_RESTART_BB	 (1 << 1)	// RESTART_BB in PHOENIX_START_CONDITION
    ICOM_FIRST_SYS_INS	(1 << 0)	// FIRST_SYS_INS in PHOENIX_START_CONDITION
    */
    u8 bat_insert= twl6030_read_bat_insert();
    fi->capacity=twl6030_read_last_capacity();

    if (fi->capacity==0 ||bat_insert>0) {/*0005078: [FR] [PM] re-calculate capacity*/
        fi->capacity=loadingVoltToCap(fi->loading_Table,fi->current_uA/1000,fi->voltage_mV,fi->battery_esr);
        fi->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(fi);
	bci_dbg(debug_common,
		"twl6030_fg_calculate_init_capacity: using Loading table for init, "
		"volt=%d mV, curr=%d mA, cap=%d\n",
		fi->voltage_mV,fi->current_uA/1000,fi->capacity);
   
    } else { //last >0, use last capacity , not read table
        int cap=loadingVoltToCap(fi->loading_Table,fi->current_uA/1000,fi->voltage_mV,fi->battery_esr); //loading table for cap
        int pre_cap =fi->capacity; //keep last capacity
        if (fi->capacity<=2) {
            fi->capacity=2;
            fi->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(fi);
	      fi->fg_hidden_uAh=0;
		bci_dbg(debug_common,
			"twl6030_fg_calculate_init_capacity: last cap too low %d, "
			"volt=%d mV, curr=%d mA, last_cap=%d\n",
			pre_cap,fi->voltage_mV,fi->current_uA/1000,fi->capacity);
        } else if(pre_cap>cap+10){
        	/*The device may switched off for long time
			using loading table for initial*/
	        fi->capacity=cap;
        	 fi->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(fi); //get uah for capacity in loading table
        	 fi->fg_hidden_uAh=0;
		 bci_dbg(debug_common,
		 	"prev capcacity > Loading table+10%%,using loading table for init "
		 	"volt=%d mV, curr=%d mA, cap=%d\n",
			fi->voltage_mV,fi->current_uA/1000,fi->capacity);   	
        }else{
        	/*last cap >2, using last capcity for initial*/
		int acc_uah=(pre_cap*(fi->fg_full_uAh - fi->fg_reserved_uAh)/100); //get uah for capacity in last value
             int loading_table_uah=(cap*(fi->fg_full_uAh - fi->fg_reserved_uAh)/100);
 	      fi->capacity = pre_cap;
           	fi->fg_hidden_uAh=acc_uah-loading_table_uah;//last - loading
           	fi->fg_acc_uAh=twl6030_fg_calculate_uAh_from_capacity(fi); //get uah for capacity in last value
           	bci_dbg(debug_common,
			"twl6030_calculate_init_capacity: using last capacity for init,"
			"volt=%d mV, curr=%d mA, last_cap=%d\n",
			fi->voltage_mV,fi->current_uA/1000,fi->capacity);
            
        }
    }
     twl6030_fg_sync(fi);
    // write init capacity to mem
    if ((fi->capacity>=0 && fi->capacity<=2) || fi->fg_full_eod>=EOD_COUNT)  // when capacity = 0~2
        twl6030_store_capacity(fi, 2);
    else if (fi->capacity>2) // 3~100
        twl6030_store_capacity(fi, fi->capacity);

    bci_dbg(debug_common,"twl6030_fg_capacity capacity = %d, hidden %dmAh,acc %dmAh!!\n",
            fi->capacity,fi->fg_hidden_uAh/1000,fi->fg_acc_uAh/1000);

}

static int  twl6030_fg_capacity(struct twl6030_bci_fuel_gauge_info *fi)
{
    int prev_capacity=fi->capacity;
    int update=0;
    int time_delta = jiffies_to_msecs(jiffies - fi->last_update);

    if (twl6030_fg_update(fi))return 0;
	
    if (fi->charge_status == POWER_SUPPLY_STATUS_UNKNOWN) {
        twl6030_fg_calculate_init_capacity(fi);
		fi->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 1;
    }
    if (time_delta > (1000 * 60 * 10)) {
        update=1;
    }
    if (fi->charge_status != POWER_SUPPLY_STATUS_UNKNOWN) {
	     if (fi->current_uA<0)
		 	twl6030_fg_discharge_regression(fi);
	     else 
		 	twl6030_fg_charge_process(fi);
    }
		

    if (fi->charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
        /*Charger Plugged but not charging, error handleing is need*/
        bci_dbg(debug_int_fuel_gauge,"twl6030 NOT CHARGING!!!\n");


    } else if (fi->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
        bci_dbg(debug_int_fuel_gauge,"twl6030 DISCHARGING!!!\n");

        if (fi->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
            twl6030_fg_sync(fi);
        } else if (fi->charge_prev_status == POWER_SUPPLY_STATUS_FULL) {

            fi->fg_acc_uAh=fi->fg_full_uAh;
            twl6030_fg_sync(fi);
        }

		/*Charge full release*/	
	 if ((fi->fg_full_charged)&&(twl6030_fg_calculate_capacity(fi)<99))
	 	fi->fg_full_charged=0;

    }
    else  if (fi->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
        bci_dbg(debug_int_fuel_gauge,"twl6030 CHARGING!!!\n"); 
		
	
        if (fi->fg_full_charged)
			fi->charge_status = POWER_SUPPLY_STATUS_FULL;
		
        if (fi->charge_prev_status == POWER_SUPPLY_STATUS_DISCHARGING)	{
            twl6030_fg_sync(fi);

        } else if (fi->charge_prev_status == POWER_SUPPLY_STATUS_FULL) {

            fi->fg_acc_uAh=fi->fg_full_uAh;

            twl6030_fg_sync(fi);
        }
        
        fi->fg_full_eod=0;

    }
    else	if (fi->charge_status == POWER_SUPPLY_STATUS_FULL)
    {
	if (fi->capacity == 99){    
	        fi->fg_acc_uAh =fi->fg_full_uAh;
	        fi->fg_hidden_uAh=0;
	        fi->fg_last_delta_uAh=0;
	        if (!fi->fg_full_charged)fi->fg_full_charged=1;
	        if (fi->charge_prev_status == POWER_SUPPLY_STATUS_CHARGING) {
	            twl6030_fg_sync(fi);
	        }
	        fi->fg_full_eod=0;
	}else if (fi->capacity == 100){
		  fi->fg_acc_uAh =fi->fg_full_uAh;
	        fi->fg_hidden_uAh=0;
	        fi->fg_last_delta_uAh=0;
	}else{
	/*modify charge status to charging, 
	   tell bci we need time to catch up full for batter user feeling*/
		fi->charge_status =POWER_SUPPLY_STATUS_CHARGING;
	}
    }

	fi->capacity = twl6030_fg_calculate_capacity(fi);
	if(fi->fg_full_charged){
		fi->capacity=100;
	}else if ((fi->capacity>=100)&&((fi->current_uA/1000)>=fi->termination_currentmA)) {
        fi->capacity=99;
    	}
 
     bci_dbg(debug_int_fuel_gauge,"vol=%d, cap=%d,tmp=%d,curr=%d, state %d <- %d\n",
                fi->voltage_mV,fi->capacity,fi->temp_C,
                fi->current_uA/1000,fi->charge_status,fi->charge_prev_status);
	 
	if((prev_capacity!=fi->capacity)||update){
		 fi->last_update = jiffies;
		 update=1;
	}
    return update;
}

static int twl6030battery_current(struct twl6030_bci_fuel_gauge_info *fi)
{
    int ret = 0;
    u16 read_value = 0;

    /* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
    ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
                       FG_REG_10, 2);
    if (ret < 0) {
        bci_dbg(debug_common, "failed to read FG_REG_10: current_now\n");
        return 0;
    }
   return twl6030_fg_avg_sample_current_uA(fi,
                     (s16)(read_value << 2) >> 2,1);

}

static int twl603x_fuel_gauge_event(struct notifier_block *nb, unsigned long event,void *_data)
{

	struct twl6030_bci_fuel_gauge_info *fi;

	int ret = NOTIFY_DONE;
	if(!(event&BCI_INTERNAL_FUELGAUGE))return ret;
	
	fi = container_of(nb, struct twl6030_bci_fuel_gauge_info, nb);
	if(!fi->initialed)return ret;
	
	if(event&BCI_FUELGAUGE_STATUS_REQUEST){
		struct bci_fuel_gauge_status_request *status=_data;
		
		 fi->voltage_mV=status->voltage;
		 fi->temp_voltage_mV=status->temp_adc_voltage_mv;

		if (fi->battery_tmp_tbl != NULL) {
			int r_ntc = 0;
			int denominator;
			
			denominator = fi->temp_Ry*1250-(fi->temp_Ry+fi->temp_Rx)*fi->temp_voltage_mV;
			if (denominator>0) {
				r_ntc=(fi->temp_Rx*fi->temp_Ry/denominator)*fi->temp_voltage_mV;
				fi->temp_C=resistance_to_temp(fi->battery_tmp_tbl,r_ntc);
			}

			bci_dbg(debug_temperature,"adc V=%d, R=%d ,Temp=%d\n",
				fi->temp_voltage_mV,r_ntc,fi->temp_C);
		}
   		 /*Rntc=Rpu*Rpd*Vadc/(Rpd*Vref-(Rpu+Rpd)*Vadc)*/
		fi->charge_status=status->current_state;
		fi->charge_prev_status=status->previous_state;
		status->capacity_changed=twl6030_fg_capacity(fi);
		status->voltage=fi->voltage_mV;
		status->capacity=fi->capacity;
		status->current_uA=fi->current_uA;
		status->temperature=fi->temp_C;
		status->current_state=fi->charge_status;
		status->previous_state=fi->charge_prev_status;
		status->data_valid=1;
		ret	=NOTIFY_OK;
	}
	else if(event&BCI_READ_CURRENT){
		int*current_uA=_data;
		*current_uA=twl6030battery_current(fi);
		ret	=NOTIFY_OK;
	} if(event&BCI_SHUTDOWN){
		 twl6030_fg_sync(fi);
		 ret	=NOTIFY_OK;
	}
	return ret;
}

static ssize_t set_fg_mode(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_fuel_gauge_info *fi = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val > 3))
        return -EINVAL;
    fi->fuelgauge_mode = val;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, (val << 6) | CC_CAL_EN,
                     FG_REG_00);
    twl6030_fg_mode_changed(fi);

    return status;
}

static ssize_t show_fg_mode(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    int val;
    struct twl6030_bci_fuel_gauge_info *fi = dev_get_drvdata(dev);

    val = fi->fuelgauge_mode;
    return sprintf(buf, "%d\n", val);
}

static ssize_t show_fg_counter(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int fg_counter = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_counter,
                 FG_REG_01, 3);
    return sprintf(buf, "%d\n", fg_counter);
}

static ssize_t show_fg_accumulator(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    long fg_accum = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_accum, FG_REG_04, 4);

    return sprintf(buf, "%ld\n", fg_accum);
}

static ssize_t show_fg_offset(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    s16 fg_offset = 0;

    twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_offset, FG_REG_08, 2);
    fg_offset = ((s16)(fg_offset << 6) >> 6);

    return sprintf(buf, "%d\n", fg_offset);
}

static ssize_t set_fg_clear(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    long val;
    int status = count;

    if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
        return -EINVAL;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_AUTOCLEAR, FG_REG_00);

    return status;
}

static ssize_t set_fg_cal(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    long val;
    int status = count;

    if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
        return -EINVAL;
    twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

    return status;
}


static ssize_t set_current_avg_interval(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
    long val;
    int status = count;
    struct twl6030_bci_fuel_gauge_info *fi = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
        return -EINVAL;
    fi->current_avg_interval = val;
    twl6030_fg_mode_changed(fi);

    return status;
}

static ssize_t show_current_avg_interval(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned int val;
    struct twl6030_bci_fuel_gauge_info *fi = dev_get_drvdata(dev);

    val = fi->current_avg_interval;
    return sprintf(buf, "%u\n", val);
}

static DEVICE_ATTR(fg_mode, S_IWUSR | S_IRUGO, show_fg_mode, set_fg_mode);
static DEVICE_ATTR(fg_counter, S_IRUGO, show_fg_counter, NULL);
static DEVICE_ATTR(fg_accumulator, S_IRUGO, show_fg_accumulator, NULL);
static DEVICE_ATTR(fg_offset, S_IRUGO, show_fg_offset, NULL);
static DEVICE_ATTR(fg_clear, S_IWUSR, NULL, set_fg_clear);
static DEVICE_ATTR(fg_cal, S_IWUSR, NULL, set_fg_cal);
static DEVICE_ATTR(current_avg_interval, S_IWUSR | S_IRUGO,
                   show_current_avg_interval, set_current_avg_interval);

static struct attribute *twl6030_bci_attributes[] = {
    &dev_attr_fg_mode.attr,
    &dev_attr_fg_counter.attr,
    &dev_attr_fg_accumulator.attr,
    &dev_attr_fg_offset.attr,
    &dev_attr_fg_clear.attr,
    &dev_attr_fg_cal.attr,
    &dev_attr_current_avg_interval.attr,
    NULL,
};

static const struct attribute_group twl6030_bci_attr_group = {
    .attrs = twl6030_bci_attributes,
};
static int __devinit _twl6030_fuel_gauge_init(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_bci_fuel_gauge_info *fi;
	int ret;

	fi = kzalloc(sizeof(*fi), GFP_KERNEL);
	if (!fi)
		return -ENOMEM;
	if (!pdata) {
		bci_dbg(debug_common, "platform_data not available\n");
		ret = -EINVAL;
		goto err_pdata;
	}

	fi->dev = &pdev->dev;
	fi->features = pdata->features;


	fi->termination_currentmA = pdata->termination_currentmA;
	fi->regulation_voltagemV = pdata->max_bat_voltagemV;
	fi->low_bat_voltagemV = pdata->low_bat_voltagemV;
	fi->battery_tmp_tbl = (struct ntc_res_temp_desc*)pdata->battery_tmp_tbl_603x;
	fi->temp_Rx=pdata->temp_rpu_ohm;
	fi->temp_Ry=pdata->temp_rpd_ohm;
	fi->loading_Table= pdata->loading_Table;
	fi->fg_resistance=pdata->sense_resistor_mohm;
	fi->fg_full_uAh= pdata->capacity_mAh*1000;
	fi->battery_esr= pdata->battery_esr;
	fi->charger_incurrentmA=pdata->max_charger_currentmA;
	fi->capacity =0;
	fi->current_avg_interval=pdata->monitoring_interval;
	fi->battery_cv_profile=	pdata->battery_cv_profile;

	fi->last_update = jiffies;
	fi->fg_acc = 0;
	fi->fg_timer = 0;
    	twl6030_fg_update(fi);/*update fg_acc and fg_time for next current*/
	
	fi->fg_acc_init=fi->fg_acc;
	fi->fg_timer_init=fi->fg_timer;
	fi->fg_timer_avg=fi->fg_timer;
	fi->fg_acc_avg=fi->fg_acc;
	fi->current_avg_uA =0;
	fi->current_uA =0;
	fi->fg_acc_delta =0;
	fi->regressin_Voltage_mV=fi->regulation_voltagemV;/*Todo: need always regression??*/
	fi->temp_voltage_mV=500;
	fi->voltage_mV=3800;


	fi->nb.notifier_call =twl603x_fuel_gauge_event;
	ret = twl6030_bci_function_register("twl603x_fuel_gauge",&fi->nb,SUPPORT_FUNC,BCI_INTERNAL_FUELGAUGE,fi);
	if (ret)
		dev_err(&pdev->dev, "bci register notifier failed %d\n", ret);
   

	ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	if (ret)
		dev_dbg(&pdev->dev, "could not create sysfs files\n");
	fi->initialed=1;
	return 0;

err_pdata:
    kfree(fi);

    return ret;
}

 int __init twl6030_fuel_gauge_init(struct platform_device *pdev)
{
    return _twl6030_fuel_gauge_init(pdev);
}


