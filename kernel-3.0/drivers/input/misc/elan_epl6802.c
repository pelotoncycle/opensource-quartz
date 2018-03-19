/* drivers/i2c/chips/elan_epl6802.c - light and proxmity sensors driver
 * Copyright (C) 2011 ELAN Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "elan_interface.h" 
#include "../../../arch/arm/mach-omap2/board-44xx-rb-common.h"
#include <linux/input-polldev.h>

#define TXBYTES 		2
#define RXBYTES 		2
#define POLLING_RATE 		2000
#define DIV_CONST		10
#define PACKAGE_SIZE 		2
#define I2C_RETRY_COUNT 	10
#define P_SENSOR_LTHD		20000
#define P_SENSOR_HTHD		10000

#define DEBUG_INT_TIME

 
extern struct kset *devices_kset;
 

//#define DEBUG_REG_DUMP

static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
static int elan_epl6802_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data);
static int elan_epl6802_I2C_Read(struct i2c_client *client);
static int set_psensor_intr_threshold(uint16_t, uint16_t);
#ifdef DEBUG_REG_DUMP
static void elan_epl6802_register_dump(void);
#endif

typedef struct _epl_raw_data 
{
	u8 raw_bytes[PACKAGE_SIZE];
	u16 ps_raw;
	u16 als_raw;
} epl_raw_data;

struct elan_epl_data {
	struct i2c_client *client;
	struct input_dev *epl_input_dev;
	struct workqueue_struct *epl_wq;
	struct wake_lock prox_wake_lock;
	int (*init)(struct sensors_pm_platform_data *pdata);
	void (*exit)(struct sensors_pm_platform_data *pdata);  
	int (*suspend)(struct sensors_pm_platform_data *pdata);
	int (*resume)(struct sensors_pm_platform_data *pdata);
	int epl_opened;
	bool light_flag;
	bool prox_flag;
	int LPmode;   // 0: light, 1: prox, 2: power off
	bool prox_wake_lock_locked;
	uint8_t epl_integrationtime_item;	
	uint8_t epl_sensor_gain_item;
	uint16_t epl_intthreshold_hvalue;
	uint16_t epl_intthreshold_lvalue;
	uint8_t epl_adc_item;
	uint8_t epl_average_item;
	uint8_t epl_intpersistency;
	uint8_t epl_op_mode;
	uint8_t epl_sc_mode;
	uint16_t ch0;
	uint16_t ch1;
	int reg_1;
} ;

struct elan_epl_data *epl_data;
static epl_raw_data gRawData; 
static void elan_epl6802_sensor_chooser(int light_enable, int prox_enable);
static void elan_epl6802_lsensor_enable(struct elan_epl_data *epld);
static void elan_epl6802_psensor_enable(struct elan_epl_data *epld);
static void elan_epl6802_sensor_disable(struct elan_epl_data *epld);

static ssize_t elan_ls_adcresolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&item);	

	epld->epl_adc_item = item;

	elan_epl6802_I2C_Write(client,REG_1,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xFC; 
	regdata |= epld->epl_adc_item;

	elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

	return count;
}

static ssize_t elan_ls_adcresolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	ret = sprintf(buf, "%d\n", epld->epl_adc_item);

	return ret;
}

static ssize_t elan_ls_averagecycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&item);
	
	epld->epl_average_item = item;

	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];  
	regdata &= 0x1F;    
	regdata |= (epld->epl_average_item << 5);	

	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

	return count;	
}

static ssize_t elan_ls_averagecycle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	ret = sprintf(buf, "%d\n", epld->epl_average_item);

	return ret;
}

static ssize_t elan_ls_integrationtime_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int item;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&item);

	epld->epl_integrationtime_item = item;
	elan_epl6802_I2C_Write(client,REG_1,R_SINGLE_BYTE,0X01,0X00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0x0F;
	regdata |= (epld->epl_integrationtime_item << 4);

	elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

	return count;
}

static ssize_t elan_ls_integrationtime_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	
	ret = sprintf(buf, "%d\n", epld->epl_integrationtime_item);

	return ret;
}

static ssize_t elan_ls_intpersistency_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&item);

	epld->epl_intpersistency = item;
	elan_epl6802_I2C_Write(client,REG_1,R_SINGLE_BYTE,0X01,0X00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xF3;
	regdata |= (epld->epl_intpersistency << 2);

	elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

	return count;

}

static ssize_t elan_ls_intpersistency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	ret = sprintf(buf, "%d\n", epld->epl_intpersistency);

	return ret;
}

static ssize_t elan_ls_intHthreshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t hthd;
	struct elan_epl_data *epld = epl_data;

	hthd = buf[1]<<8 | buf[0];

	epld->epl_intthreshold_hvalue = hthd;

	set_psensor_intr_threshold(epld->epl_intthreshold_lvalue,epld->epl_intthreshold_hvalue);

	return 1;
}

static ssize_t elan_ls_intHthreshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long *tmp = (long*)buf;
	struct elan_epl_data *epld = epl_data;

	tmp[0] = epld->epl_intthreshold_hvalue;

	return 4;
}

static ssize_t elan_ls_intLthreshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t lthd;
	struct elan_epl_data *epld = epl_data;

	lthd = buf[1]<<8 | buf[0];	

	epld->epl_intthreshold_lvalue = lthd;

	set_psensor_intr_threshold(epld->epl_intthreshold_lvalue,epld->epl_intthreshold_hvalue);

	return 1;
}

static ssize_t elan_ls_intLthreshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long *tmp = (long*)buf;
	struct elan_epl_data *epld = epl_data;

	tmp[0] = epld->epl_intthreshold_lvalue;

	return 4;
}

static ssize_t elan_ls_operationmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&mode);

	epld->epl_op_mode = mode;  
	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0X01,0X00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xF3;
	regdata |= (epld->epl_op_mode << 2);  

	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

	return count;
}

static ssize_t elan_ls_operationmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	ret = sprintf(buf, "%d\n", epld->epl_op_mode);

    return ret;
}

static ssize_t elan_ls_singlecontinuousmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&mode);

	epld->epl_sc_mode = mode;
	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0X01,0X00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xEF;
	regdata |= (epld->epl_sc_mode << 4);

	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);	
	
	return count;
}

static ssize_t elan_ls_singlecontinuousmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	ret = sprintf(buf, "%d\n", epld->epl_sc_mode);

	return ret;
}

static ssize_t elan_ls_sensorgain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	uint8_t regdata;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	sscanf(buf,"%d\n",&item);

	epld->epl_sensor_gain_item = item;
	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0X01,0X00);
	elan_epl6802_I2C_Read(client);
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xFC;
	regdata |= epld->epl_sensor_gain_item;

	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);
	
	return count;
}

static ssize_t elan_ls_sensorgain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	
	ret = sprintf(buf, "%d\n", epld->epl_sensor_gain_item);

	return ret;
}

#ifdef DEBUG_REG_DUMP
static ssize_t elan_ls_channel01_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;
	u16 ch0 = 0, ch1 = 0;
	u8 reg_13 = 0, reg_1 = 0;

	ret = elan_epl6802_I2C_Write(client, REG_1, R_SINGLE_BYTE, 0x01, 0x00);
	if (ret > 0) {
		ret = elan_epl6802_I2C_Read(client);
		if (ret > 0) {
			reg_1 = gRawData.raw_bytes[0];
		}
	}

	ret = elan_epl6802_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0x00);
	if (ret > 0) {
		ret = elan_epl6802_I2C_Read(client);
		if (ret > 0) {
			reg_13 = gRawData.raw_bytes[0];
		}
	}

	ret = elan_epl6802_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
	if (ret > 0) {
		ret = elan_epl6802_I2C_Read(client);
		if (ret > 0) {
			ch0 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
		}
	}

	ret = elan_epl6802_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	if (ret > 0) {
		ret = elan_epl6802_I2C_Read(client);
		if (ret > 0) {
			ch1 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
		}
	}

	ret = sprintf(buf, "reg_1:%03d(0x%02x), reg_13:%03d(0x%02x), ch0:%05d(0x%04x), ch1:%05d(0x%04x)\n", reg_1, reg_1, reg_13, reg_13, ch0, ch0, ch1, ch1);
	
	return ret;
}
#endif

#ifdef DEBUG_REG_DUMP
static ssize_t elan_ls_debug_reg_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int ret = 0, i = 0;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	for (i = 0; i <= 0x14; i++) {
		if (i == 0x06 || i == 0x08 || i == 0x0c || i == 0x12) {
			/* Reserved registers*/
			continue;
		}
		
		ret = elan_epl6802_I2C_Write(client, i, R_SINGLE_BYTE, 0x01, 0x00);
		if (ret > 0) {
			ret = elan_epl6802_I2C_Read(client);
			if (ret > 0) {
				count += sprintf(&buf[count], "reg_%02X=0x%02x\n", i, gRawData.raw_bytes[0]);
			}
		}
	}

	return count;
}
#endif

static ssize_t elan_ls_enable_light_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	struct elan_epl_data *epld = epl_data;

	sscanf(buf,"%d\n",&item);
	epld->light_flag = (item==1) ? true : false;

	if (epld->light_flag) {
		elan_epl6802_sensor_chooser(1,-1); // light: true		
	} else {
		elan_epl6802_sensor_chooser(0,-1); // light: false
	} 

	return count;
}

static ssize_t elan_ls_enable_light_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;

	if(epld->light_flag)
		ret = sprintf(buf, "light enabled\n");
	else
		ret = sprintf(buf, "light disabled\n");

	return ret;
}

static ssize_t elan_ls_enable_prox_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int item;
	struct elan_epl_data *epld = epl_data;

	sscanf(buf,"%d\n",&item);
	epld->prox_flag = (item==1) ? true : false;

	if (epld->prox_flag) {
		elan_epl6802_sensor_chooser(-1,1); // prox: true
	} else {
		elan_epl6802_sensor_chooser(-1,0); // prox: false
	} 

	return count;
}

static ssize_t elan_ls_enable_prox_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	
	if(epld->prox_flag)
		ret = sprintf(buf, "prox enabled\n");
	else
		ret = sprintf(buf, "prox disabled\n");
	
	return ret;
}

#ifdef DEBUG_INT_TIME
static ssize_t elan_ls_integration_time_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) 
{
	int item;
	struct elan_epl_data *epld = epl_data;

	sscanf(buf,"%d\n",&item);

	switch(item) {
		case 16:
			epld->reg_1 = INT_TIME_16;
			break;
		case 24:
			epld->reg_1 = INT_TIME_24;
			break;
		case 32:
			epld->reg_1 = INT_TIME_32;
			break;
		case 48:
			epld->reg_1 = INT_TIME_48;
			break;
		case 80:
			epld->reg_1 = INT_TIME_80;
			break;
		case 144:
			epld->reg_1 = INT_TIME_144;
			break;
		case 272:
			epld->reg_1 = INT_TIME_272;
			break;
		case 384:
			epld->reg_1 = INT_TIME_384;
			break;
		default:
			epld->reg_1 = INT_TIME_24;
			break;
	}

	return count;
}
#endif

#ifdef DEBUG_INT_TIME
static ssize_t elan_ls_integration_time_set_show(struct device *dev, struct device_attribute *attr, char *buf) 
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	
	ret = sprintf(buf, "reg_1 = %x\n", epld->reg_1 | 0x02);

	return ret;
}
#endif

static DEVICE_ATTR(elan_ls_adcresolution, S_IROTH, elan_ls_adcresolution_show, elan_ls_adcresolution_store);
static DEVICE_ATTR(elan_ls_averagecycle, S_IROTH, elan_ls_averagecycle_show, elan_ls_averagecycle_store);
static DEVICE_ATTR(elan_ls_integrationtime, S_IROTH, elan_ls_integrationtime_show, elan_ls_integrationtime_store);
static DEVICE_ATTR(elan_ls_intpersistency, S_IROTH, elan_ls_intpersistency_show, elan_ls_intpersistency_store);
static DEVICE_ATTR(elan_ls_intHthreshold, S_IROTH, elan_ls_intHthreshold_show, elan_ls_intHthreshold_store);
static DEVICE_ATTR(elan_ls_intLthreshold, S_IROTH, elan_ls_intLthreshold_show, elan_ls_intLthreshold_store);
static DEVICE_ATTR(elan_ls_operationmode, S_IROTH, elan_ls_operationmode_show,elan_ls_operationmode_store);
static DEVICE_ATTR(elan_ls_singlecontinuousmode, S_IROTH, elan_ls_singlecontinuousmode_show,elan_ls_singlecontinuousmode_store);
static DEVICE_ATTR(elan_ls_sensorgain, S_IROTH, elan_ls_sensorgain_show,elan_ls_sensorgain_store);
#ifdef DEBUG_REG_DUMP
static DEVICE_ATTR(elan_ls_ch01, S_IRUGO, elan_ls_channel01_show, NULL);
static DEVICE_ATTR(elan_ls_regshow, S_IRUGO, elan_ls_debug_reg_show, NULL);
#endif
static DEVICE_ATTR(enable_light, S_IRUGO | S_IWUSR, elan_ls_enable_light_set_show, elan_ls_enable_light_set_store);
static DEVICE_ATTR(enable_prox, S_IRUGO | S_IWUSR, elan_ls_enable_prox_set_show, elan_ls_enable_prox_set_store);
#ifdef DEBUG_INT_TIME
static DEVICE_ATTR(integration_time, S_IRUGO | S_IWUSR, elan_ls_integration_time_set_show, elan_ls_integration_time_set_store);
#endif

static struct attribute *ets_attributes[] = {
	&dev_attr_elan_ls_adcresolution.attr,
	&dev_attr_elan_ls_averagecycle.attr,
	&dev_attr_elan_ls_integrationtime.attr,
	&dev_attr_elan_ls_intpersistency.attr,
	&dev_attr_elan_ls_intHthreshold.attr,
	&dev_attr_elan_ls_intLthreshold.attr,
	&dev_attr_elan_ls_operationmode.attr,
	&dev_attr_elan_ls_singlecontinuousmode.attr,
	&dev_attr_elan_ls_sensorgain.attr,
	#ifdef DEBUG_REG_DUMP
	&dev_attr_elan_ls_ch01.attr,	
	&dev_attr_elan_ls_regshow.attr,
	#endif
	&dev_attr_enable_light.attr,
	&dev_attr_enable_prox.attr,
#ifdef DEBUG_INT_TIME
	&dev_attr_integration_time.attr,
#endif
	NULL,
};

static struct attribute_group ets_attr_group = {
	.attrs = ets_attributes,
};

/*
 * I2C write operation
 *
 * regaddr: ELAN epl6802 Register Address.
 * bytecount: How many bytes to be written to epl6802 register via i2c bus.
 * txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
 * data: setting value.
 *
 * Example: If you want to write single byte to 0x1D register address, show below 
 *	      elan_epl6802_I2C_Write(client,0x1D,0x01,0X02,0xff);
 *
 */            
static int elan_epl6802_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry;

	buffer[0] = (regaddr<<3) | bytecount ;
	buffer[1] = data;

	for(retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret == txbyte) 
			break;	
		msleep(1);
	}

	if(retry>=I2C_RETRY_COUNT)
	{
		printk(KERN_ERR "[ELAN epl6802.c error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
		ret = -EINVAL; 
	}

	return ret;
}

static int elan_epl6802_I2C_Read(struct i2c_client *client)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i =0;
	int retry;
		
	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		ret = i2c_master_recv(client, buffer, RXBYTES);
		if (ret == RXBYTES) {
			break;		
		}
		msleep(1);
	}
	
	if (retry >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ELAN epl6802 error] %s retry over %d\n", __func__, I2C_RETRY_COUNT);
		ret = -EINVAL; 
		goto i2c_error;
	}
		
	for (i = 0; i < PACKAGE_SIZE; i++) {
		gRawData.raw_bytes[i] = buffer[i];
	}
	
i2c_error:	
	return ret;
}

/* Through elan_epl6802_sensor_chooser to choose light or proximity sensor.
 * At any time, only one sensor is activated.
 * If both flags are true, light sensor is enabled.
 */
static void elan_epl6802_sensor_chooser(int light_enable, int prox_enable)
{
	struct elan_epl_data *epld = epl_data;
	static bool light_flag = false;
	static bool prox_flag = false;
	

	if (light_enable == 1) {
		light_flag = true;
	} else if (light_enable == 0) {
		light_flag = false;
	}

	if (prox_enable == 1) {
		prox_flag = true;
	} else if (prox_enable == 0) {
		prox_flag = false;
	}

	if (light_flag && !prox_flag) {
		elan_epl6802_lsensor_enable(epld);
		printk("epl6802: light sensor mode\n");
	} else if (!light_flag && prox_flag) {
		elan_epl6802_psensor_enable(epld);
		printk("epl6802: prox sensor mode\n");
	} else if (!light_flag && !prox_flag) {
		elan_epl6802_sensor_disable(epld);
		printk("epl6802: shut down power\n");
	} else {
		elan_epl6802_psensor_enable(epld);
		printk("epl6802: prox first\n");
		msleep(2);
	}
}

static void elan_epl6802_lsensor_enable(struct elan_epl_data *epld)
{
	uint8_t regdata;
	struct i2c_client *client = epld->client;

	// cancel current queued work, wait for register setting
	cancel_delayed_work(&polling_work);

	// reg7: Power Up
	elan_epl6802_I2C_Write(client,REG_7,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xFD;
	regdata |= EPL_C_P_UP;;
	elan_epl6802_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,regdata);	

	msleep(2);

	// reg0: LS mode
	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xF3;
	regdata |= EPL_ALS_MODE;     
   	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

	msleep(2);

	// reg1:
	elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0x02,0x32);    

	msleep(2);

	// unlock the wake lock acquired by proximity
	if (epld->prox_wake_lock_locked) {
		wake_unlock(&epld->prox_wake_lock);
		epld->prox_wake_lock_locked = false;
	}

	epld->LPmode = 0;   
	queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(POLLING_RATE));
}

static void elan_epl6802_psensor_enable(struct elan_epl_data *epld)
{
	uint8_t regdata;
	struct i2c_client *client = epld->client;

	// acquire wake lock when proximity is enabled
	if (!epld->prox_wake_lock_locked) {
		wake_lock(&epld->prox_wake_lock);
		epld->prox_wake_lock_locked = true;
		printk("[epl6802] wake lock locked\n");
	}

	// cancel current queued work, wait for register setting
	cancel_delayed_work(&polling_work);

	// reg7: Power Up
	elan_epl6802_I2C_Write(client,REG_7,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xFD;
	regdata |= EPL_C_P_UP;;
	elan_epl6802_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,regdata);	

	msleep(2);

	// reg0: PS mode
	elan_epl6802_I2C_Write(client,REG_0,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xF3;
	regdata |= EPL_PS_MODE;   
	elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

	msleep(2);

	// reg1:
	elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0x02,epld->reg_1 | 0x02);    

	msleep(2);

	epld->LPmode = 1;   
	queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(POLLING_RATE/DIV_CONST));
}

static void elan_epl6802_sensor_disable(struct elan_epl_data *epld)
{
	uint8_t regdata;
	struct i2c_client *client = epld->client;

	// cancel current queued work, wait for register setting
	cancel_delayed_work(&polling_work);

	msleep(2);

	// reg7 : Power Down
	elan_epl6802_I2C_Write(client,REG_7,R_SINGLE_BYTE,0x01,0x00);
	elan_epl6802_I2C_Read(client);	
	regdata = gRawData.raw_bytes[0];
	regdata &= 0xFD;
	regdata |= EPL_C_P_DOWN;
	elan_epl6802_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,regdata);

	// unlock the wake lock acquired by proximity
	if (epld->prox_wake_lock_locked) {
		wake_unlock(&epld->prox_wake_lock);
		epld->prox_wake_lock_locked = false;
		printk("[epl6802] wake lock unlocked\n");
	}
	
	epld->LPmode = 2; 
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;

	uint8_t high_msb ,high_lsb, low_msb, low_lsb;
	
	high_msb = (uint8_t) (high_thd >> 8);
	high_lsb = (uint8_t) (high_thd & 0x00ff);
	low_msb  = (uint8_t) (low_thd >> 8);
	low_lsb  = (uint8_t) (low_thd & 0x00ff);

	elan_epl6802_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
	elan_epl6802_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
	elan_epl6802_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
	elan_epl6802_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);

	return ret;
}

static void polling_do_work(struct work_struct *work)
{
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;
	u16 ch0 = 0, ch1 = 0;	
	int retr, retw;

	/* get ch0 and ch1 from register */
	// Channel 0
	retw = elan_epl6802_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
	retr = elan_epl6802_I2C_Read(client);
	if (retr > 0 && retw > 0) {
		ch0 = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]) ;
	}
	// Channel 1
	retw = elan_epl6802_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	retr = elan_epl6802_I2C_Read(client);
	if (retr > 0 && retw > 0) {
		ch1 = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]);
	}
	// force uplight sensor HAL to update
	if ((ch0 == epld->ch0) && (ch1 == epld->ch1)) {
		if (ch1 != 0xffff) {
			ch1++;
		}
		else {
			ch1--;
		}
	}

	if (epld->LPmode == 0) {
		printk("----- als Ch0:Ch1 = %d:%d\n", ch0, ch1);
		input_report_abs(epld->epl_input_dev, ABS_X, ch0); 
		input_report_abs(epld->epl_input_dev, ABS_Y, ch1);
		input_report_abs(epld->epl_input_dev, ABS_DISTANCE, 1);
		input_sync(epld->epl_input_dev);
		queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(POLLING_RATE));
	} else if (epld->LPmode == 1) {
		printk("----- ps Ch0:Ch1 = %d:%d\n", ch0, ch1);
		input_report_abs(epld->epl_input_dev, ABS_X, 0); 
		input_report_abs(epld->epl_input_dev, ABS_Y, 0);
		input_report_abs(epld->epl_input_dev, ABS_DISTANCE, ch1);
		input_sync(epld->epl_input_dev);
		queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(POLLING_RATE/DIV_CONST));
	} else if (epld->LPmode == 2) {
		//printk("----- power off\n");
		//input_report_abs(epld->epl_input_dev, ABS_X, 0); 
		//input_report_abs(epld->epl_input_dev, ABS_Y, 0);
		//input_report_abs(epld->epl_input_dev, ABS_DISTANCE, 1);
		//input_sync(epld->epl_input_dev);
		//cancel_delayed_work(&polling_work);
	}

	epld->ch0 = ch0;
	epld->ch1 = ch1;
}

static int elan_epl_open(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	if (epld->epl_opened) {
		return -EBUSY;
	}
	epld->epl_opened = 1;
   
	return 0;
}

static int elan_epl_release(struct inode *inode, struct file *file)
{
	struct elan_epl_data *epld = epl_data;

	epld->epl_opened = 0;
    
	return 0;
}

static struct file_operations elan_epl_fops = {
	.owner = THIS_MODULE,
	.open = elan_epl_open,
	.release = elan_epl_release,
};

static struct miscdevice elan_epl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "elan_ls",
	.fops = &elan_epl_fops
};

#ifdef DEBUG_REG_DUMP
static void elan_epl6802_register_dump(void)
{
	int ret = -1;
	uint8_t regdata[REG_21+1];
	struct elan_epl_data *epld = epl_data;
	struct i2c_client *client = epld->client;
	volatile int i = 0;

	memset((void *)regdata, 0x00, sizeof(regdata));

	for(i = REG_0; i <= REG_21; i++)
	{
		if(i == REG_8 || i == REG_12 || i == REG_21)
			continue;
		ret = elan_epl6802_I2C_Write(client, i, R_SINGLE_BYTE, 0x01, 0x00);
		ret = elan_epl6802_I2C_Read(client);
		if(ret < 0) {
			goto error;
		}
		
		regdata[i] = gRawData.raw_bytes[0];
	}

	printk("0-10 %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x \n11-21 %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
		regdata[0], regdata[1], regdata[2], regdata[3], regdata[4],
		regdata[5], regdata[6], regdata[7], regdata[8], regdata[9],
		regdata[10], regdata[11], regdata[12], regdata[13], regdata[14],
		regdata[15], regdata[16], regdata[17], regdata[18], regdata[19],
		regdata[20], regdata[21]);

error:
	if(ret<0)
		printk("%s() reg r/w error\n", __func__);

}
#endif

static int initial_epl6802(struct elan_epl_data *epld)
{
	int ret = 0;
	struct i2c_client *client = epld->client;
	epld->reg_1 = INT_TIME_DEFAULT;

	ret = elan_epl6802_I2C_Read(client);
	if(ret < 0) {
		return -EINVAL;
	}

	//control initial procedure 
	ret = elan_epl6802_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02,0x62);
	if(ret < 0)
		return -EINVAL;
	ret = elan_epl6802_I2C_Write(client,REG_1,W_SINGLE_BYTE,0x02,0x32);    
	// Disable interrupt mode, set Reg_9 (INTTY[1:0]) to 0x02
	ret = elan_epl6802_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,0x02);  	
	ret = elan_epl6802_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);  
	ret = elan_epl6802_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);  

   	//Set proximity sensor High threshold and Low threshold  
	set_psensor_intr_threshold(P_SENSOR_LTHD,P_SENSOR_HTHD);  

#ifdef DEBUG_REG_DUMP
        elan_epl6802_register_dump();
#endif

	return ret;
}

static int epl6802_setup(struct elan_epl_data *epld)
{
	int err = 0;

	msleep(5);
	
	printk(KERN_INFO " elan EPL sensor probe enter.\n");

	epld->epl_input_dev = input_allocate_device();
	if (!epld->epl_input_dev) {
		pr_err("[ELAN epl6802 error]%s: could not allocate ls input device\n",__func__);
		return -ENOMEM;
	}
	
	epld->epl_input_dev->name = LIGHT_DRIVER_NAME;

	set_bit(EV_ABS, epld->epl_input_dev->evbit);
	set_bit(EV_KEY, epld->epl_input_dev->evbit);
  	
  	set_bit(ABS_MISC, epld->epl_input_dev->absbit);
	input_set_abs_params(epld->epl_input_dev, ABS_MISC, 0, 65535, 0, 0);

	set_bit(ABS_DISTANCE, epld->epl_input_dev->absbit);
	input_set_abs_params(epld->epl_input_dev, ABS_DISTANCE, 0, 65535, 0, 0);
	set_bit(ABS_X, epld->epl_input_dev->absbit);
	input_set_abs_params(epld->epl_input_dev, ABS_X, 0, 65535, 0, 0);
	set_bit(ABS_Y, epld->epl_input_dev->absbit);
	input_set_abs_params(epld->epl_input_dev, ABS_Y, 0, 65535, 0, 0);

	err = input_register_device(epld->epl_input_dev);
	if (err < 0) {
		pr_err("[ELAN epl6802 error]%s: can not register ls input device\n",__func__);
		goto err_free_epl_input_device;
	}

	err = misc_register(&elan_epl_device);
	if (err < 0) {
		pr_err("[ELAN epl6802 error]%s: can not register ls misc device\n",__func__);
		goto err_unregister_epl_input_device;
	}

	err = initial_epl6802(epld);
	if (err < 0) {
		pr_err("[ELAN epl6802 error]%s: fail to initial epl6802 (%d)\n",__func__, err);
		goto initial_fail;
	}

	return err;

initial_fail:
	misc_deregister(&elan_epl_device);
err_unregister_epl_input_device:
	input_unregister_device(epld->epl_input_dev);
err_free_epl_input_device:
	input_free_device(epld->epl_input_dev);
	return err;
}

static int elan_epl6802_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct elan_epl_data *epld = NULL;
	struct sensors_pm_platform_data *pdata = NULL; /*elan_epl_platform_data*/ 
	int err = 0;
	
	printk("elan-epl6802 probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[ELAN epl6802 error] i2c func error\n");
		return -ENOTSUPP;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		printk("[ELAN epl6802 error] platform data null\n");
		return -1;
	}
  
	epld = kzalloc(sizeof(struct elan_epl_data), GFP_KERNEL);
	if (!epld) {
		return -ENOMEM;
	}
	
	pdata->i2c_client_dev=&client->dev;
	i2c_set_clientdata(client, epld);
	epld->client	= client;
	epld->init 	= pdata->init_platform_hw;
	epld->exit	= pdata->exit_platform_hw;
	epld->suspend	= pdata->suspend_platform_hw;
	epld->resume	= pdata->resume_platform_hw;
	
	//Add for epl6802 Tool
	epld->epl_integrationtime_item = 0x05;
	epld->epl_intthreshold_hvalue = P_SENSOR_HTHD;
	epld->epl_intthreshold_lvalue = P_SENSOR_LTHD;
	epld->epl_sensor_gain_item = 0x02;

	epld->epl_op_mode = 0x01;
	epld->epl_adc_item = 0x01;
	epld->epl_average_item = 0x03;
	epld->epl_intpersistency = 0x00;
	epld->epl_sc_mode = 0x00;

	epl_data = epld;   

	err = epld->init(pdata);
	if(err < 0) {
		printk("[ELAN epl6802 error] epl6802 initilization failure\n");
		return err;
	}

	epld->epl_wq = create_singlethread_workqueue("epl6802_wq");
	if (!epld->epl_wq) {
		printk("[ELAN epl6802 error] can't create workqueue\n");
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	err = epl6802_setup(epld);
	if (err < 0) {
		printk("[ELAN epl6802 error] epl6802_setup error\n");
		goto err_epl6802_setup;
	}

	err = queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(POLLING_RATE));
	if(err<0)
	{
		printk("[ELAN epl6802 error] queue_delayed_work fail\n");
		goto err_delayed_work;
	}

	// acquire wake lock for proximity sensor to prevent suspend during call
	wake_lock_init(&epld->prox_wake_lock, WAKE_LOCK_SUSPEND, "prox_wake_lock");
	epld->prox_wake_lock_locked = false;

	elan_epl6802_sensor_disable(epld);

	//Add for epl6802 Tool
	err = sysfs_create_group(&epld->epl_input_dev->dev.kobj, &ets_attr_group);
	if (err !=0) {
		printk("[ELAN epl6802 error] create sysfs group error\n");
		goto err_fail;		
	}

 
	if (sysfs_create_link(&devices_kset->kobj, &epld->epl_input_dev->dev.kobj, "light")) {
		printk("[ELAN epl6802 error] create link to device failed\n");
	}
 

	printk("elan-epl6802 probe end\n");
	return err;

err_fail:
	wake_lock_destroy(&epld->prox_wake_lock);
err_delayed_work:
	input_unregister_device(epld->epl_input_dev);
	input_free_device(epld->epl_input_dev);
err_epl6802_setup:
	destroy_workqueue(epld->epl_wq);
err_create_singlethread_workqueue:
	if (epld) {
		if(epld->exit) {
			epld->exit(pdata);
		}
		kfree(epld);
	}

	return err;
}

static int elan_epl6802_remove(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);
	struct sensors_pm_platform_data *pdata = client->dev.platform_data;

	cancel_delayed_work(&polling_work);
	input_unregister_device(epld->epl_input_dev);
	input_free_device(epld->epl_input_dev);
	misc_deregister(&elan_epl_device);
	destroy_workqueue(epld->epl_wq);
	wake_lock_destroy(&epld->prox_wake_lock);

	if(epld->exit) {
		epld->exit(pdata);
	}

	kfree(epld);
	return 0;
}

static int elan_epl6802_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);
	struct sensors_pm_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	if (epld->suspend){
		ret = epld->suspend(pdata);
		if (ret < 0) {
			dev_err(&client->dev, "epl6802_suspend suspend light power failed\n");
			return ret;
		} else {
			printk("[epl6802] suspend\n");
		}	
	}

	return 0;
}

static int elan_epl6802_resume(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);
	struct sensors_pm_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	if (epld->resume) {
		ret = epld->resume(pdata);
		if (ret < 0){
			dev_err(&client->dev, "epl6802_resume light power on failed\n");
			return ret;
		} else {
			printk("[epl6802] resume\n");
		}			
	}

	/* After system suspend, the registers are cleared.
	   So we should re-initialize the registers. */
	initial_epl6802(epld);

	return 0;
}

static void elan_epl6802_shutdown(struct i2c_client *client)
{
	struct elan_epl_data *epld = i2c_get_clientdata(client);
	struct sensors_pm_platform_data *pdata = client->dev.platform_data;

	cancel_delayed_work(&polling_work);
	input_unregister_device(epld->epl_input_dev);
	input_free_device(epld->epl_input_dev);
	misc_deregister(&elan_epl_device);
	destroy_workqueue(epld->epl_wq);

	if(epld->exit) {
		epld->exit(pdata);
	}

	kfree(epld);

	printk("[epl6802] shutdown\n");
}

static const struct i2c_device_id elan_epl6802_id[] = {
	{ LIGHT_DRIVER_NAME, 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, elan_epl6802_id);

static struct i2c_driver elan_epl6802_driver = {
	.probe = elan_epl6802_probe,
	.remove = elan_epl6802_remove,
	.suspend = elan_epl6802_suspend,
	.resume = elan_epl6802_resume,
	.shutdown = elan_epl6802_shutdown,
	.id_table = elan_epl6802_id,
	.driver = {
		.name = LIGHT_DRIVER_NAME, 
		.owner = THIS_MODULE,
	},
};

static int __init elan_epl6802_init(void)
{
	return i2c_add_driver(&elan_epl6802_driver);
}

static void __exit elan_epl6802_exit(void)
{
	i2c_del_driver(&elan_epl6802_driver);
}

module_init(elan_epl6802_init);
module_exit(elan_epl6802_exit);

MODULE_DESCRIPTION("Innocomm ELAN epl6802 driver");
MODULE_LICENSE("GPL");
