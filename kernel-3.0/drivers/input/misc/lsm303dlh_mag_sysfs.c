/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name	: lsm303dlhc_mag_sys.c
* Authors	: MSH - Motion Mems BU - Application Team
*		: Carmine Iascone (carmine.iascone@st.com)
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Both authors are willing to be considered the contact
*		: and update points for the driver.*
* Version	: V.1.0.10
* Date		: 2011/Aug/16
* Description	: LSM303DLHC 6D module sensor device driver sysfs
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
 Revision 1.0.7: 2010/Nov/22
  corrects bug in enable/disable of polling polled device;
 Revision 1.0.9: 2011/May/23
  SLEEP_MODE correction; update_odr func correct.; get/set_polling_rate f. corr.
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c; sensitivities correction;
 Revision 1.0.10.1: 2012/Mar/21 by morris chen
  1. change the sysfs attribute for android CTS
  2. correct the parameter number on line 880
*******************************************************************************/

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/slab.h>

#include "../../../arch/arm/mach-omap2/board-44xx-talos.h"
#include "lsm303dlh_sysfs.h"

#define DEBUG 0
//#define CONFIG_VIA_POLLDEV

#if DEBUG
#define dprintk(fmt, arg...) \
	do { printk(KERN_INFO "[%s] " fmt, LSM303DLHC_MAG_DEV_NAME, ##arg); } while (false)
#else
#define dprintk(fmt, arg...) \
	do { } while (false)
#endif



/** Maximum polled-device-reported g value */
#define H_MAX			8100

/* Magnetometer registers */
#define CRA_REG_M		0x00	/* Configuration register A */
#define CRB_REG_M		0x01	/* Configuration register B */
#define MR_REG_M		0x02	/* Mode register */

/* resume state index */
#define RES_CRA_REG_M		0	/* Configuration register A */
#define RES_CRB_REG_M		1	/* Configuration register B */
#define RES_MR_REG_M		2	/* Mode register */

/* Output register start address*/
#define OUT_X_M			0x03

/* Magnetic Sensor Operation Mode */
#define NORMAL_MODE		0x00
#define POS_BIAS		0x01
#define NEG_BIAS		0x02
#define CC_MODE			0x00
#define SC_MODE			0x01
#define SLEEP_MODE		0x03

/* Magnetometer X-Y sensitivity  */
#define XY_SENSITIVITY_1_3	1100	/* XY sensitivity at 1.3G */
#define XY_SENSITIVITY_1_9	 855	/* XY sensitivity at 1.9G */
#define XY_SENSITIVITY_2_5	 670	/* XY sensitivity at 2.5G */
#define XY_SENSITIVITY_4_0	 450	/* XY sensitivity at 4.0G */
#define XY_SENSITIVITY_4_7	 400	/* XY sensitivity at 4.7G */
#define XY_SENSITIVITY_5_6	 330	/* XY sensitivity at 5.6G */
#define XY_SENSITIVITY_8_1	 230	/* XY sensitivity at 8.1G */

/* Magnetometer Z sensitivity  */
#define Z_SENSITIVITY_1_3	 980	/* Z sensitivity at 1.3G */
#define Z_SENSITIVITY_1_9	 760	/* Z sensitivity at 1.9G */
#define Z_SENSITIVITY_2_5	 600	/* Z sensitivity at 2.5G */
#define Z_SENSITIVITY_4_0	 400	/* Z sensitivity at 4.0G */
#define Z_SENSITIVITY_4_7	 355	/* Z sensitivity at 4.7G */
#define Z_SENSITIVITY_5_6	 295	/* Z sensitivity at 5.6G */
#define Z_SENSITIVITY_8_1	 205	/* Z sensitivity at 8.1G */

/* Magnetometer output data rate  */
#define LSM303DLHC_MAG_ODR_75		0x00	/* 0.75Hz output data rate */
#define LSM303DLHC_MAG_ODR1_5		0x04	/* 1.5Hz output data rate */
#define LSM303DLHC_MAG_ODR3_0		0x08	/* 3Hz output data rate */
#define LSM303DLHC_MAG_ODR7_5		0x0C	/* 7.5Hz output data rate */
#define LSM303DLHC_MAG_ODR15		0x10	/* 15Hz output data rate */
#define LSM303DLHC_MAG_ODR30		0x14	/* 30Hz output data rate */
#define LSM303DLHC_MAG_ODR75		0x18	/* 75Hz output data rate */
#define LSM303DLHC_MAG_ODR220		0x1C	/* 220Hz output data rate */

#define FUZZ			0
#define FLAT			0

extern struct kset *devices_kset;

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	LSM303DLHC_MAG_MIN_POLL_PERIOD_MS,	LSM303DLHC_MAG_ODR220},
	{	14,	LSM303DLHC_MAG_ODR75},
	{	34,	LSM303DLHC_MAG_ODR30},
	{	67,	LSM303DLHC_MAG_ODR15},
	{	134,	LSM303DLHC_MAG_ODR7_5},
	{	334,	LSM303DLHC_MAG_ODR3_0},
	{	667,	LSM303DLHC_MAG_ODR1_5},
	{	1334,	LSM303DLHC_MAG_ODR_75},
};

static int use_smbus = 0;

struct lsm303dlhc_mag_data {
	struct i2c_client *client;
	struct sensors_pm_platform_data *pdata;
	struct input_dev *input_dev;

	struct mutex lock;

#ifdef CONFIG_VIA_POLLDEV
		struct input_polled_dev *input_poll_dev;
#else
		struct workqueue_struct *input_wq;
		struct delayed_work input_work;
#endif

	int hw_initialized;
	int on_before_suspend;
	atomic_t enabled;

	u16 xy_sensitivity;
	u16 z_sensitivity;

	u8 reg_addr;
	u8 resume_state[3];
};

static const struct lsm303dlhc_mag_platform_data default_lsm303dlhc_mag_pdata = {
	.poll_interval = 100,
	.min_interval = 10,
	.h_range = LSM303DLHC_H_1_3G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};



static int lsm303dlhc_mag_i2c_read(struct lsm303dlhc_mag_data *mag,
				u8 * buf, int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
#if DEBUG
	int ii;
#endif

/*
	if( len > sizeof(buf) )
			dev_err(&mag->client->dev,
				"read error insufficient buffer length: "
				"len:%d, buf size=%d\n",
				len, sizeof(buf));
*/

	if (use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(mag->client, cmd);
			buf[0] = ret & 0xff;
#if DEBUG
			dev_warn(&mag->client->dev, 
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if ( len > 1) {
			//cmd =  = I2C_AUTO_INCREMENT | reg;
			ret = i2c_smbus_read_i2c_block_data(mag->client, 
								cmd, len, buf);
#if DEBUG
			dev_warn(&mag->client->dev, 
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			for(ii=0;ii<len;ii++){
				printk("buf[%d]=0x%02x,",ii,buf[ii]);
			}
			printk("\n");
#endif
		} else ret = -1;
		if (ret < 0) {
			dev_err(&mag->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd );
			return 0; // failure
		}
		return len; // success
	}

	//cmd =  = I2C_AUTO_INCREMENT | reg;
	ret = i2c_master_send(mag->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(mag->client, buf, len);
}

static int lsm303dlhc_mag_i2c_write(struct lsm303dlhc_mag_data *mag, u8 * buf,
								int len)
{
	int ret;
	u8 reg, value;
#if DEBUG
	int ii;
#endif

	reg = buf[0];
	value = buf[1];

	if (use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(mag->client, reg, value);
#if DEBUG
			dev_warn(&mag->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(mag->client, 
							reg, len, buf + 1 );
#if DEBUG
			dev_warn(&mag->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg );
			for(ii=0;ii<len+1;ii++){
				printk("value[%d]=0x%02x,",ii,buf[ii]);
			}
			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(mag->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

int lsm303dlhc_mag_update_h_range(struct lsm303dlhc_mag_data *mag,
								u8 new_h_range)
{
	int err = -1;
	u8 buf[2];

	switch (new_h_range) {
	case LSM303DLHC_H_1_3G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_3;
		mag->z_sensitivity = Z_SENSITIVITY_1_3;
		break;
	case LSM303DLHC_H_1_9G:
		mag->xy_sensitivity = XY_SENSITIVITY_1_9;
		mag->z_sensitivity = Z_SENSITIVITY_1_9;
		break;
	case LSM303DLHC_H_2_5G:
		mag->xy_sensitivity = XY_SENSITIVITY_2_5;
		mag->z_sensitivity = Z_SENSITIVITY_2_5;
		break;
	case LSM303DLHC_H_4_0G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_0;
		mag->z_sensitivity = Z_SENSITIVITY_4_0;
		break;
	case LSM303DLHC_H_4_7G:
		mag->xy_sensitivity = XY_SENSITIVITY_4_7;
		mag->z_sensitivity = Z_SENSITIVITY_4_7;
		break;
	case LSM303DLHC_H_5_6G:
		mag->xy_sensitivity = XY_SENSITIVITY_5_6;
		mag->z_sensitivity = Z_SENSITIVITY_5_6;
		break;
	case LSM303DLHC_H_8_1G:
		mag->xy_sensitivity = XY_SENSITIVITY_8_1;
		mag->z_sensitivity = Z_SENSITIVITY_8_1;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&mag->enabled)) {

		buf[0] = CRB_REG_M;
		buf[1] = new_h_range;
		err = lsm303dlhc_mag_i2c_write(mag, buf, 1);
		if (err < 0)
			return err;
		mag->resume_state[RES_CRB_REG_M] = new_h_range;
	}



	return 0;
}

int lsm303dlhc_mag_update_odr(struct lsm303dlhc_mag_data *mag,
							int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval)
							|| (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= NORMAL_MODE;

	if (atomic_read(&mag->enabled)) {
		config[0] = CRA_REG_M;
		err = lsm303dlhc_mag_i2c_write(mag, config, 1);
		if (err < 0)
			return err;
		mag->resume_state[RES_CRA_REG_M] = config[1];
	}

	return 0;
}

static int lsm303dlhc_mag_get_data(struct lsm303dlhc_mag_data *mag,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware HxL, HxH, HyL, HyH, HzL, HzH */
	u8 mag_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	mag_data[0] = OUT_X_M;
	err = lsm303dlhc_mag_i2c_read(mag, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((mag_data[0]) << 8) | mag_data[1]);
	hw_d[1] = (int) (((mag_data[4]) << 8) | mag_data[5]);
	hw_d[2] = (int) (((mag_data[2]) << 8) | mag_data[3]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	if (hw_d[0] != 0XF000)
		hw_d[0] = hw_d[0] * 1000 / mag->xy_sensitivity;
	else
		hw_d[0] = 0X8000;
	if (hw_d[1] != 0XF000)
		hw_d[1] = hw_d[1] * 1000 / mag->xy_sensitivity;
	else
		hw_d[1] = 0x8000;
	if (hw_d[2] != 0XF000)
		hw_d[2] = hw_d[2] * 1000 / mag->z_sensitivity;
	else
		hw_d[2] = 0x8000;
#if 1
		xyz[0] = ((mag->pdata->negate_x) ? (-hw_d[mag->pdata->axis_map_x])
			  : (hw_d[mag->pdata->axis_map_x]));
		xyz[1] = ((mag->pdata->negate_y) ? (-hw_d[mag->pdata->axis_map_y])
			  : (hw_d[mag->pdata->axis_map_y]));
		xyz[2] = ((mag->pdata->negate_z) ? (-hw_d[mag->pdata->axis_map_z])
			  : (hw_d[mag->pdata->axis_map_z]));
#else

	if ((hw_d[mag->pdata->axis_map_x] != 0x8000) && (mag->pdata->negate_x))
		xyz[0] = -hw_d[mag->pdata->axis_map_x];
	else
		xyz[0] = hw_d[mag->pdata->axis_map_x];

	if ((hw_d[mag->pdata->axis_map_y] != 0x8000) && (mag->pdata->negate_y))
		xyz[1] = -hw_d[mag->pdata->axis_map_y];
	else
		xyz[1] = hw_d[mag->pdata->axis_map_y];

	if ((hw_d[mag->pdata->axis_map_z] != 0x8000) && (mag->pdata->negate_z))
		xyz[2] = -hw_d[mag->pdata->axis_map_z];
	else
		xyz[2] = hw_d[mag->pdata->axis_map_z];
#endif
	return err;
}

static void lsm303dlhc_mag_report_values(struct input_dev *input,
					int *xyz)
{
	input_report_abs(input, ABS_X, xyz[0]);
	input_report_abs(input, ABS_Y, xyz[1]);
	input_report_abs(input, ABS_Z, xyz[2]);
	input_sync(input);

	dprintk("mag_out: x = %d y = %d z= %d\n",
	xyz[0], xyz[1], xyz[2]);
}

static int lsm303dlhc_mag_hw_init(struct lsm303dlhc_mag_data *mag)
{
	int err = -1;
	u8 buf[4];

	dprintk("hw init");
	buf[0] = CRA_REG_M;
	buf[1] = mag->resume_state[RES_CRA_REG_M];
	buf[2] = mag->resume_state[RES_CRB_REG_M];
	buf[3] = mag->resume_state[RES_MR_REG_M];
	err = lsm303dlhc_mag_i2c_write(mag, buf, 3);

	if (err < 0)
		return err;

	mag->hw_initialized = 1;

	return 0;
}

static void lsm303dlhc_mag_device_power_off(struct lsm303dlhc_mag_data *mag)
{
	int err;
	u8 buf[2] = { MR_REG_M, SLEEP_MODE };

	dprintk("power off\n");

	err = lsm303dlhc_mag_i2c_write(mag, buf, 1);
	if (err < 0)
		dev_err(&mag->client->dev, "soft power off failed\n");

	if (mag->pdata->suspend_platform_hw) {
		mag->pdata->suspend_platform_hw(mag->pdata);
		mag->hw_initialized = 0;
	}
}

static int lsm303dlhc_mag_device_power_on(struct lsm303dlhc_mag_data *mag)
{
	int err;
	u8 buf[2] = { MR_REG_M, NORMAL_MODE };

	dprintk("power on\n");

	if (mag->pdata->resume_platform_hw) {
		err = mag->pdata->resume_platform_hw(mag->pdata);
		if (err < 0)
			return err;
	}

	if (!mag->hw_initialized) {
		err = lsm303dlhc_mag_hw_init(mag);
		if (err < 0) {
			lsm303dlhc_mag_device_power_off(mag);
			return err;
		}
	}

	err = lsm303dlhc_mag_i2c_write(mag, buf, 1);

	return 0;
}

static int lsm303dlhc_mag_enable(struct lsm303dlhc_mag_data *mag)
{
	int err;

	dprintk("enable");
	if (!atomic_cmpxchg(&mag->enabled, 0, 1)) {

		err = lsm303dlhc_mag_device_power_on(mag);
		if (err < 0) {
			atomic_set(&mag->enabled, 0);

			dprintk("failed");
			return err;
		}
	}
#ifdef CONFIG_VIA_POLLDEV
#else
			queue_delayed_work(mag->input_wq, &mag->input_work, msecs_to_jiffies(
				mag->pdata->poll_interval));
#endif

	return 0;
}

static int lsm303dlhc_mag_disable(struct lsm303dlhc_mag_data *mag)
{
	if (atomic_cmpxchg(&mag->enabled, 1, 0)) {
#ifdef CONFIG_VIA_POLLDEV
#else
		cancel_delayed_work_sync(&mag->input_work);
#endif
		lsm303dlhc_mag_device_power_off(mag);

		dprintk("disabled\n");

	}

	return 0;
}

#if 0
static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	int range = 0;
	u8 val;
	mutex_lock(&mag->lock);
	val = mag->pdata->range;
	pr_info("%s, h_range = %d", __func__, val);
	switch (val) {
	case LSM303DLHC_H_1_3G:
		range = 1300;
		break;
	case LSM303DLHC_H_1_9G:
		range = 1900;
		break;
	case LSM303DLHC_H_2_5G:
		range = 2500;
		break;
	case LSM303DLHC_H_4_0G:
		range = 4000;
		break;
	case LSM303DLHC_H_4_7G:
		range = 4700;
		break;
	case LSM303DLHC_H_5_6G:
		range = 5600;
		break;
	case LSM303DLHC_H_8_1G:
		range = 8100;
		break;
	}
	mutex_unlock(&mag->lock);
	return sprintf(buf, "%d mGauss\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&mag->lock);
	mag->pdata->range = val;
	lsm303dlhc_mag_update_h_range(mag, val);
	mutex_unlock(&mag->lock);
	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&mag->lock);
	x[0] = mag->reg_addr;
	mutex_unlock(&mag->lock);
	x[1] = val;
	rc = lsm303dlhc_mag_i2c_write(mag, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&mag->lock);
	data = mag->reg_addr;
	mutex_unlock(&mag->lock);
	rc = lsm303dlhc_mag_i2c_read(mag, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&mag->lock);

	mag->reg_addr = val;

	mutex_unlock(&mag->lock);

	return size;
}
#endif /* DEBUG */

static struct device_attribute attributes[] = {
	__ATTR(delay, 0644, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0644, attr_get_range, attr_set_range),
	__ATTR(enable, 0644, attr_get_enable, attr_set_enable),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif /* DEBUG */
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#endif

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max(interval_ms,(unsigned long)mag->pdata->min_interval);
	mutex_lock(&mag->lock);
#ifdef CONFIG_VIA_POLLDEV
	mag->input_poll_dev->poll_interval = interval_ms;
#endif
	mag->pdata->poll_interval = interval_ms;
	lsm303dlhc_mag_update_odr(mag, interval_ms);
	mutex_unlock(&mag->lock);
	return size;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	mutex_lock(&mag->lock);
	val = mag->pdata->poll_interval;
	mutex_unlock(&mag->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	int val = atomic_read(&mag->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303dlhc_mag_data *mag = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303dlhc_mag_enable(mag);
	else
		lsm303dlhc_mag_disable(mag);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


#ifdef CONFIG_VIA_POLLDEV
static void lsm303dlhc_mag_input_poll_func(struct input_polled_dev *dev)
{
	struct lsm303dlhc_mag_data *mag = dev->private;

	int xyz[3] = { 0 };

	int err;

	mutex_lock(&mag->lock);
	err = lsm303dlhc_mag_get_data(mag, xyz);
	if (err < 0)
		dev_err(&mag->client->dev, "get_magnetometer_data failed\n");
	else
		lsm303dlhc_mag_report_values(mag->input_dev, xyz);

	mutex_unlock(&mag->lock);
}

void lsm303dlhc_mag_input_poll_open(struct input_polled_dev *input)
{
	//struct lsm303dlhc_mag_data *mag = input->private;

	//lsm303dlhc_mag_enable(mag);
}

void lsm303dlhc_mag_input_poll_close(struct input_polled_dev *input)
{
	//struct lsm303dlhc_mag_data *mag = input->private;

	//lsm303dlhc_mag_disable(mag);
}
#else
static void lsm303dlhc_mag_input_work_func(struct work_struct *work)
{
	struct lsm303dlhc_mag_data *mag = container_of((struct delayed_work *)work,
			struct lsm303dlhc_mag_data, input_work);
	
	int xyz[3] = { 0 };

	int err;

	mutex_lock(&mag->lock);
	err = lsm303dlhc_mag_get_data(mag, xyz);
	if (err < 0)
		dev_err(&mag->client->dev, "get_magnetometer_data failed\n");
	else
		lsm303dlhc_mag_report_values(mag->input_dev, xyz);
	
	queue_delayed_work(mag->input_wq, &mag->input_work, msecs_to_jiffies(
					mag->pdata->poll_interval));

	mutex_unlock(&mag->lock);
}

int lsm303dlhc_mag_input_open(struct input_dev *input)
{
	//return lsm303dlhc_mag_enable(input_get_drvdata(input));
	return 0;
}

void lsm303dlhc_mag_input_close(struct input_dev *input)
{
	//lsm303dlhc_mag_disable(input_get_drvdata(input));
}

#endif

static int lsm303dlhc_mag_validate_pdata(struct lsm303dlhc_mag_data *mag)
{
	/* checks for correctness of minimal polling period */
	mag->pdata->min_interval =
		max(LSM303DLHC_MAG_MIN_POLL_PERIOD_MS,
						mag->pdata->min_interval);

	mag->pdata->poll_interval = max(mag->pdata->poll_interval,
					mag->pdata->min_interval);

	if (mag->pdata->axis_map_x > 2 ||
	    mag->pdata->axis_map_y > 2 || mag->pdata->axis_map_z > 2) {
		dev_err(&mag->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			mag->pdata->axis_map_x, mag->pdata->axis_map_y,
			mag->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (mag->pdata->negate_x > 1 || mag->pdata->negate_y > 1 ||
	    mag->pdata->negate_z > 1) {
		dev_err(&mag->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			mag->pdata->negate_x, mag->pdata->negate_y,
			mag->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (mag->pdata->poll_interval < mag->pdata->min_interval) {
		dev_err(&mag->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303dlhc_mag_input_init(struct lsm303dlhc_mag_data *mag)
{
	int err = -1;
	struct input_dev *input;

#ifdef CONFIG_VIA_POLLDEV
	mag->input_poll_dev = input_allocate_polled_device();
	if (!mag->input_poll_dev) {
		err = -ENOMEM;
		dev_err(&mag->client->dev, "input device allocate failed\n");
		goto err0;
	}
	mag->input_poll_dev->private = mag;
	mag->input_poll_dev->poll = lsm303dlhc_mag_input_poll_func;
	mag->input_poll_dev->poll_interval = mag->pdata->poll_interval;
	mag->input_poll_dev->open = lsm303dlhc_mag_input_poll_open;
	mag->input_poll_dev->close = lsm303dlhc_mag_input_poll_close;
	mag->input_dev = mag->input_poll_dev->input;
	input = mag->input_poll_dev->input;
#else
	mag->input_wq = create_singlethread_workqueue("input_wq");
	INIT_DELAYED_WORK(&mag->input_work, lsm303dlhc_mag_input_work_func);

	mag->input_dev = input_allocate_device();
	if (!mag->input_dev) {
		err = -ENOMEM;
		dev_err(&mag->client->dev, "input device allocation failed\n");
		goto err0;
	}
	input = mag->input_dev;
	input->open = lsm303dlhc_mag_input_open;
	input->close = lsm303dlhc_mag_input_close;
#endif
	input->name = LSM303DLHC_MAG_DEV_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &mag->client->dev;

	input_set_drvdata(input, mag);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -H_MAX, H_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -H_MAX, H_MAX, FUZZ, FLAT);

#ifdef CONFIG_VIA_POLLDEV
	err = input_register_polled_device(mag->input_poll_dev);
#else
	err = input_register_device(input);
#endif

	if (err) {
		dev_err(&mag->client->dev,
			"unable to register input polled device %s\n",
			input->name);
		goto err1;
	}

	return 0;

err1:
#ifdef CONFIG_VIA_POLLDEV
	input_free_polled_device(mag->input_poll_dev);
#else
	input_free_device(input);
#endif

err0:
	return err;
}

static void lsm303dlhc_mag_input_cleanup(struct lsm303dlhc_mag_data *mag)
{
#ifdef CONFIG_VIA_POLLDEV
	input_unregister_polled_device(mag->input_poll_dev);
	input_free_polled_device(mag->input_poll_dev);
#else
	input_unregister_device(mag->input_dev);
	input_free_device(mag->input_dev);
#endif

}

static int lsm303dlhc_mag_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lsm303dlhc_mag_data *mag;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA | 
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

	int err = -1;

	dprintk("probe start\n");

	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)){
		use_smbus = 1;
		dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto err0;
		}
	}

	mag = kzalloc(sizeof(*mag), GFP_KERNEL);
	if (mag == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&mag->lock);
	mutex_lock(&mag->lock);
	mag->client = client;

	mag->pdata = kmalloc(sizeof(*mag->pdata), GFP_KERNEL);
	if (mag->pdata == NULL)
		goto err1;

	if (client->dev.platform_data == NULL) {	
		memcpy(mag->pdata, &default_lsm303dlhc_mag_pdata,
							sizeof(*mag->pdata));	
	} else {
		memcpy(mag->pdata, client->dev.platform_data,
							sizeof(*mag->pdata));
	}
	mag->pdata->i2c_client_dev=&client->dev;

	err = lsm303dlhc_mag_validate_pdata(mag);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, mag);

	if (mag->pdata->init_platform_hw) {
		err = mag->pdata->init_platform_hw(mag->pdata);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}
	
	err = lsm303dlhc_mag_input_init(mag);
		if (err < 0)
			goto err1_1;

	memset(mag->resume_state, 0, ARRAY_SIZE(mag->resume_state));

	mag->resume_state[RES_CRA_REG_M] =
				LSM303DLHC_MAG_ODR15 | LSM303DLHC_MAG_NORMAL_MODE;
	mag->resume_state[RES_CRB_REG_M] = LSM303DLHC_H_1_3G;
	mag->resume_state[RES_MR_REG_M] = SLEEP_MODE;

	err = lsm303dlhc_mag_device_power_on(mag);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	err = lsm303dlhc_mag_update_h_range(mag, mag->pdata->range);
	if (err < 0) {
		dev_err(&client->dev, "update_h_range failed\n");
		goto err2;
	}

	err = lsm303dlhc_mag_update_odr(mag, mag->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "%s register failed\n",
						LSM303DLHC_MAG_DEV_NAME);
		goto err4;
	}
	
	/* create symlink to dev under /sys/devices */
	err = sysfs_create_link(&devices_kset->kobj, &client->dev.kobj,
						"magnetometer");
	if (err < 0) {
		dev_err(&client->dev,
			"create short path to input device failed");
		goto err4;
	}

	//lsm303dlhc_mag_device_power_off(mag);
	lsm303dlhc_mag_device_power_off(mag);

	atomic_set(&mag->enabled, 0);

	mutex_unlock(&mag->lock);

	dprintk("probed: %s device created successfully\n",
#ifdef CONFIG_VIA_POLLDEV
						"input poll"
#else
						"input"
	
#endif
							);
	return 0;

err4:
	lsm303dlhc_mag_input_cleanup(mag);
err2:
	if (mag->pdata->exit_platform_hw)
		mag->pdata->exit_platform_hw(mag->pdata);
err1_1:
	mutex_unlock(&mag->lock);
	kfree(mag->pdata);
err1:
	kfree(mag);
err0:
	dev_err(&client->dev, "%s: Driver Initialization failed\n", LSM303DLHC_MAG_DEV_NAME);
	return err;
}

static int lsm303dlhc_mag_remove(struct i2c_client *client)
{
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);
	
	dprintk("remove\n");
	
	mag->input_dev->close(mag->input_dev);
	lsm303dlhc_mag_input_cleanup(mag);
	remove_sysfs_interfaces(&client->dev);

	if (mag->pdata->exit_platform_hw)
		mag->pdata->exit_platform_hw(mag->pdata);

	kfree(mag->pdata);
	kfree(mag);
	return 0;
}

static void lsm303dlhc_mag_shutdown(struct i2c_client *client)
{
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);
	
	dprintk("shutdown\n");
	
	mag->input_dev->close(mag->input_dev);
	lsm303dlhc_mag_input_cleanup(mag);
	lsm303dlhc_mag_disable(mag);
	remove_sysfs_interfaces(&client->dev);
	if (mag->pdata->exit_platform_hw)
		mag->pdata->exit_platform_hw(mag->pdata);

	kfree(mag->pdata);
	kfree(mag);
}

static int lsm303dlhc_mag_suspend(struct device *dev)
{
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);
	
	dprintk("suspend\n");
	
	mag->on_before_suspend = atomic_read(&mag->enabled);
	
#ifdef CONFIG_VIA_POLLDEV
	mag->input_dev->close(mag->input_dev);
#endif
	lsm303dlhc_mag_disable(mag);
#endif /* CONFIG_SUSPEND */
	return 0;
}

static int lsm303dlhc_mag_resume(struct device *dev)
{
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_mag_data *mag = i2c_get_clientdata(client);

	dprintk("resume\n");

	if (mag->on_before_suspend) {
		
		lsm303dlhc_mag_enable(mag);
#ifdef CONFIG_VIA_POLLDEV
		mag->input_dev->open(mag->input_dev);
#endif
	}
	return 0;
	/* TO DO */
#endif /* CONFIG_SUSPEND */
}

static const struct i2c_device_id lsm303dlhc_mag_id[] = {
	{LSM303DLHC_MAG_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm303dlhc_mag_id);

static struct dev_pm_ops lsm303dlhc_pm = {
	.suspend = lsm303dlhc_mag_suspend,
	.resume = lsm303dlhc_mag_resume,
};

static struct i2c_driver lsm303dlhc_mag_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM303DLHC_MAG_DEV_NAME,
			.pm = &lsm303dlhc_pm,
		   },
	.probe = lsm303dlhc_mag_probe,
	.remove = __devexit_p(lsm303dlhc_mag_remove),
	.shutdown = lsm303dlhc_mag_shutdown,
	.id_table = lsm303dlhc_mag_id,
};

static int __init lsm303dlhc_mag_init(void)
{
	dprintk("init\n");
	return i2c_add_driver(&lsm303dlhc_mag_driver);
}

static void __exit lsm303dlhc_mag_exit(void)
{
	dprintk("exit\n");
	i2c_del_driver(&lsm303dlhc_mag_driver);
	return;
}

module_init(lsm303dlhc_mag_init);
module_exit(lsm303dlhc_mag_exit);

MODULE_DESCRIPTION("lsm303dlhc sys driver for the magnetometer section");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
