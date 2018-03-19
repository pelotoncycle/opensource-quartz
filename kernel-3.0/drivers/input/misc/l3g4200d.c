/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name	: l3g4200d_gyr_sysfs.c
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.4 sysfs
* Date			: 2011/Sep/24
* Description		: L3G4200D digital output gyroscope sensor API
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
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/11/19	| Carmine Iascone	| First Release
* 1.1.0		| 2011/02/28	| Matteo Dameno		| Self Test Added
* 1.1.1		| 2011/05/25	| Matteo Dameno		| Corrects Polling Bug
* 1.1.2		| 2011/05/30	| Matteo Dameno		| Corrects ODR Bug
* 1.1.3		| 2011/06/24	| Matteo Dameno		| Corrects ODR Bug
* 1.1.4		| 2011/09/24	| Matteo Dameno		| forces BDU setting
* 1.1.4.1 | 2012/3/21   | Morris Chen     | change sysfs attribute for android 
*                                           CTS
*******************************************************************************/
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/slab.h>

#include "../../../arch/arm/mach-omap2/board-44xx-talos.h"
#include "l3g4200d.h"

#define DEBUG 0
//#define CONFIG_VIA_POLLDEV

#if DEBUG
#define dprintk(fmt, arg...) \
	do { printk(KERN_INFO "[%s] " fmt, L3G4200D_GYR_DEV_NAME, ##arg); } while (false)
#else
#define dprintk(fmt, arg...) \
	do { } while (false)
#endif

extern struct kset *devices_kset;


/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */
struct l3g4200d_triple {
	short	x,	/* x-axis angular rate data. */
			y,	/* y-axis angluar rate data. */
			z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {
	{	2,	ODR800|BW10},
	{	3,	ODR400|BW01},
	{	5,	ODR200|BW00},
	{	10,	ODR100|BW00},
};

struct l3g4200d_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct sensors_pm_platform_data *pdata;

	struct mutex lock;

#ifdef CONFIG_VIA_POLLDEV
	struct input_polled_dev *input_poll_dev;
#else
	struct workqueue_struct *input_wq;
	struct delayed_work input_work;
#endif
	int hw_initialized;
	int on_before_suspend;
	int selftest_enabled;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];
};

static int l3g4200d_i2c_read(struct l3g4200d_data *gyro,
				  u8 *buf, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = gyro->client->addr,
		 .flags = gyro->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = gyro->client->addr,
		 .flags = (gyro->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(gyro->client->adapter, msgs, 2);

	if (err != 2) {
		dev_err(&gyro->client->dev, "read transfer error: %d\n",err);
		return -EIO;
	}

	return 0;
}

static int l3g4200d_i2c_write(struct l3g4200d_data *gyro,
						u8 *buf,
						int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = gyro->client->addr,
		 .flags = gyro->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(gyro->client->adapter, msgs, 1);

	if (err != 1) {
		dev_err(&gyro->client->dev, "write transfer error\n");
		return -EIO;
	}

	return 0;
}

static int l3g4200d_register_write(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = l3g4200d_i2c_write(gyro, buf, 1);
	if (err < 0)
		return err;

	return err;
}

static int l3g4200d_register_read(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address)
{
	int err = -1;
	buf[0] = (reg_address);
	err = l3g4200d_i2c_read(gyro, buf, 1);
	return err;
}

static int l3g4200d_register_update(struct l3g4200d_data *gyro, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = l3g4200d_register_read(gyro, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = l3g4200d_register_write(gyro, buf, reg_address,
				updated_val);
	}
	return err;
}


static int l3g4200d_update_fs_range(struct l3g4200d_data *gyro,
							u8 new_fs)
{
	int res ;
	u8 buf[2];

	buf[0] = CTRL_REG4;

	res = l3g4200d_register_update(gyro, buf, CTRL_REG4,
							FS_MASK, new_fs);

	if (res < 0) {
		pr_err("%s : failed to update fs:0x%02x\n",
			__func__, new_fs);
		return res;
	}
	gyro->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs ) |
		( ~FS_MASK & gyro->resume_state[RES_CTRL_REG4]));

	return res;
}




static int l3g4200d_update_odr(struct l3g4200d_data *gyro,
				int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval) || (i == 0)){
			break;
		}

	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&gyro->enabled)) {
		config[0] = CTRL_REG1;
		err = l3g4200d_i2c_write(gyro, config, 1);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;
}

/* gyroscope data readout */
static int l3g4200d_get_data(struct l3g4200d_data *gyro,
			     struct l3g4200d_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3] = { 0 };

	gyro_out[0] = (AUTO_INCREMENT | AXISDATA_REG);

	err = l3g4200d_i2c_read(gyro, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s16) (((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16) (((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16) (((gyro_out[5]) << 8) | gyro_out[4]);

	data->x = ((gyro->pdata->negate_x) ? (-hw_d[gyro->pdata->axis_map_x])
		   : (hw_d[gyro->pdata->axis_map_x]));
	data->y = ((gyro->pdata->negate_y) ? (-hw_d[gyro->pdata->axis_map_y])
		   : (hw_d[gyro->pdata->axis_map_y]));
	data->z = ((gyro->pdata->negate_z) ? (-hw_d[gyro->pdata->axis_map_z])
		   : (hw_d[gyro->pdata->axis_map_z]));

	return err;
}

static void l3g4200d_report_values(struct l3g4200d_data *gyro)
{
	struct input_dev *input = gyro->input_dev;
	struct l3g4200d_triple data;
	
	int err;
	
	mutex_lock(&gyro->lock);
	err = l3g4200d_get_data(gyro, &data);
	if (err < 0)
		dprintk("get_gyroscope_data failed\n");
	else {

		input_report_abs(input, ABS_X, data.x);
		input_report_abs(input, ABS_Y, data.y);
		input_report_abs(input, ABS_Z, data.z);
		input_sync(input);
		
		dprintk("gyro_out: y = %d p = %d r= %d\n",
			data.x, data.y, data.z);
	}
	mutex_unlock(&gyro->lock);
}

static int l3g4200d_hw_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	u8 buf[6];

	dprintk("hw init\n");

	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	buf[1] = gyro->resume_state[RES_CTRL_REG1];
	buf[2] = gyro->resume_state[RES_CTRL_REG2];
	buf[3] = gyro->resume_state[RES_CTRL_REG3];
	buf[4] = gyro->resume_state[RES_CTRL_REG4];
	buf[5] = gyro->resume_state[RES_CTRL_REG5];

	err = l3g4200d_i2c_write(gyro, buf, 5);
	if (err < 0)
		return err;

	gyro->hw_initialized = 1;

	return err;
}

static void l3g4200d_device_power_off(struct l3g4200d_data *gyro)
{
	int err;
	u8 buf[2];

	dprintk("power off\n");

	buf[0] = CTRL_REG1;
	buf[1] = PM_OFF;
	err = l3g4200d_i2c_write(gyro, buf, 1);
	if (err < 0)
		dev_err(&gyro->client->dev, "soft power off failed\n");

	if (gyro->pdata->suspend_platform_hw) {
		gyro->pdata->suspend_platform_hw(gyro->pdata);
		gyro->hw_initialized = 0;
	}

	if (gyro->hw_initialized)
		gyro->hw_initialized = 0;

}

static int l3g4200d_device_power_on(struct l3g4200d_data *gyro)
{
	int err;

	dprintk("power on\n");

	if (gyro->pdata->resume_platform_hw) {
		err = gyro->pdata->resume_platform_hw(gyro->pdata);
		if (err < 0)
			return err;
	}


	if (!gyro->hw_initialized) {
		err = l3g4200d_hw_init(gyro);
		if (err < 0) {
			l3g4200d_device_power_off(gyro);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_enable(struct l3g4200d_data *gyro)
{
	int err;
	
	dprintk("enable\n");

	if (!atomic_cmpxchg(&gyro->enabled, 0, 1)) {

		err = l3g4200d_device_power_on(gyro);
		if (err < 0) {
			atomic_set(&gyro->enabled, 0);
			return err;
		}
#ifdef CONFIG_VIA_POLLDEV
#else
		queue_delayed_work(gyro->input_wq, &gyro->input_work, msecs_to_jiffies(
			gyro->pdata->poll_interval));
#endif
	}

	return 0;
}

static int l3g4200d_disable(struct l3g4200d_data *gyro)
{
	dprintk("disabled\n");
	if (atomic_cmpxchg(&gyro->enabled, 1, 0)) {
#ifdef CONFIG_VIA_POLLDEV
#else
		cancel_delayed_work_sync(&gyro->input_work);
#endif
		l3g4200d_device_power_off(gyro);
	}
	return 0;
}

#ifdef CONFIG_VIA_POLLDEV
void l3g4200d_input_poll_open(struct input_polled_dev *input)
{
	//l3g4200d_enable(input->private);
}

void l3g4200d_input_poll_close(struct input_polled_dev *input)
{
	//l3g4200d_disable(input->private);
}

static void l3g4200d_input_poll_func(struct input_polled_dev *dev)
{
	struct l3g4200d_data *gyro = dev->private;
	l3g4200d_report_values(gyro);
}

#else
int l3g4200d_input_open(struct input_dev *input)
{
	return 0;
	//return l3g4200d_enable(input_get_drvdata(input));
}

void l3g4200d_input_close(struct input_dev *input)
{
	//l3g4200d_disable(input_get_drvdata(input));
}

static void l3g4200d_input_work_func(struct work_struct *work)
{
	struct l3g4200d_data *gyro = container_of((struct delayed_work *)work,
			struct l3g4200d_data, input_work);

	l3g4200d_report_values(gyro);

	queue_delayed_work(gyro->input_wq, &gyro->input_work, msecs_to_jiffies(
				gyro->pdata->poll_interval));

}

#endif

#if 0 // Unnecessary funcions.
static int l3g4200d_selftest(struct l3g4200d_data *gyro, u8 enable)
{
	int err = -1;
	u8 buf[2] = {0x00,0x00};
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = SELFTEST_MASK;
	if (enable > 0)
		bit_values = L3G4200D_SELFTEST_EN_POS;
	else
		bit_values = L3G4200D_SELFTEST_DIS;
	if (atomic_read(&gyro->enabled)) {
		mutex_lock(&gyro->lock);
		err = l3g4200d_register_update(gyro, buf, reg_address,
				mask, bit_values);
		gyro->selftest_enabled = enable;
		mutex_unlock(&gyro->lock);
		if (err < 0)
			return err;
		gyro->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				( ~mask & gyro->resume_state[RES_CTRL_REG4]));
	}
	return err;
}

#endif

#if 0 // Sysfs for debug, shouldn't create for security issue.
static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int range = 0;
	char val;
	mutex_lock(&gyro->lock);
	val = gyro->pdata->range;
	switch (val) {
	case L3G4200D_GYR_FS_250DPS:
		range = 250;
		break;
	case L3G4200D_GYR_FS_500DPS:
		range = 500;
		break;
	case L3G4200D_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&gyro->lock);
	gyro->pdata->range = val;
	l3g4200d_update_fs_range(gyro, val);
	mutex_unlock(&gyro->lock);
	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	mutex_lock(&gyro->lock);
	val = gyro->selftest_enabled;
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	l3g4200d_selftest(gyro, val);

	return size;
}

#if DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&gyro->lock);
	x[0] = gyro->reg_addr;
	mutex_unlock(&gyro->lock);
	x[1] = val;
	rc = l3g4200d_i2c_write(gyro, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&gyro->lock);
	data = gyro->reg_addr;
	mutex_unlock(&gyro->lock);
	rc = l3g4200d_i2c_read(gyro, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);

	gyro->reg_addr = val;

	mutex_unlock(&gyro->lock);

	return size;
}
#endif 
static struct device_attribute attributes[] = {
	__ATTR(delay, 0644, attr_polling_rate_show,
						attr_polling_rate_store),
	__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable, 0644, attr_enable_show, attr_enable_store),
	__ATTR(enable_selftest, 0644, attr_get_selftest, attr_set_selftest),
#if DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
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

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	mutex_lock(&gyro->lock);
	val = gyro->pdata->poll_interval;
	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max(interval_ms,(unsigned long )gyro->pdata->min_interval);
	mutex_lock(&gyro->lock);
#ifdef CONFIG_VIA_POLLDEV
	gyro->input_poll_dev->poll_interval = interval_ms;
#endif
	gyro->pdata->poll_interval = interval_ms;
	l3g4200d_update_odr(gyro, interval_ms);
	mutex_unlock(&gyro->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int val = atomic_read(&gyro->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		l3g4200d_enable(gyro);
	else
		l3g4200d_disable(gyro);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll, 0664, attr_polling_rate_show, attr_polling_rate_store),
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

#else

#endif

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	/* checks for correctness of minimal polling period */
	gyro->pdata->min_interval =
		max(L3G4200D_MIN_POLL_PERIOD_MS,
						gyro->pdata->min_interval);

	gyro->pdata->poll_interval = max(gyro->pdata->poll_interval,
			gyro->pdata->min_interval);

	if (gyro->pdata->axis_map_x > 2 ||
	    gyro->pdata->axis_map_y > 2 ||
	    gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			gyro->pdata->axis_map_x,
			gyro->pdata->axis_map_y,
			gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
	    gyro->pdata->negate_y > 1 ||
	    gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x,
			gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (gyro->pdata->poll_interval < gyro->pdata->min_interval) {
		dev_err(&gyro->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int l3g4200d_input_init(struct l3g4200d_data *gyro)
{
	int err = -1;
	struct input_dev *input;

#ifdef CONFIG_VIA_POLLDEV
	gyro->input_poll_dev = input_allocate_polled_device();
	if (!gyro->input_poll_dev) {
		err = -ENOMEM;
		dev_err(&gyro->client->dev,
			"input device allocate failed\n");
		goto err0;
	}

	gyro->input_poll_dev->private = gyro;
	gyro->input_poll_dev->poll_interval = gyro->pdata->poll_interval;
	
	gyro->input_poll_dev->open = l3g4200d_input_poll_open;
	gyro->input_poll_dev->close = l3g4200d_input_poll_close;
	gyro->input_poll_dev->poll = l3g4200d_input_poll_func;
	gyro->input_dev = gyro->input_poll_dev->input;
	
	input = gyro->input_poll_dev->input;
#else
	gyro->input_wq = create_singlethread_workqueue("input_wq");
	INIT_DELAYED_WORK(&gyro->input_work, l3g4200d_input_work_func);

	gyro->input_dev = input_allocate_device();
	if (!gyro->input_dev) {
		err = -ENOMEM;
		dev_err(&gyro->client->dev, "input device allocation failed\n");
		goto err0;
	}
	input = gyro->input_dev;
	input->open = l3g4200d_input_open;
	input->close = l3g4200d_input_close;
#endif

	input->name = L3G4200D_GYR_DEV_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &gyro->client->dev;

	input_set_drvdata(input, gyro);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -FS_MAX, FS_MAX, FUZZ, FLAT);

	
#ifdef CONFIG_VIA_POLLDEV
	err = input_register_polled_device(gyro->input_poll_dev);
#else
	err = input_register_device(input);
#endif
	if (err) {
		dev_err(&gyro->client->dev,
			"unable to register input device: %s\n", input->name);
		goto err1;
	}

	return 0;

err1:
#ifdef CONFIG_VIA_POLLDEV
	input_free_polled_device(gyro->input_poll_dev);
#else
	input_free_device(input);
#endif
err0:
	return err;
}

static void l3g4200d_input_cleanup(struct l3g4200d_data *gyro)
{
#ifdef CONFIG_VIA_POLLDEV
	input_unregister_polled_device(gyro->input_poll_dev);
	input_free_polled_device(gyro->input_poll_dev);
#else
	input_unregister_device(gyro->input_dev);
	input_free_device(gyro->input_dev);
#endif
}

static int l3g4200d_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct l3g4200d_data *gyro;

	int err = -1;

	dprintk("probe start\n");

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable:1\n");
		err = -ENODEV;
		goto err0;
	}

	gyro = kzalloc(sizeof(*gyro), GFP_KERNEL);
	if (gyro == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&gyro->lock);
	mutex_lock(&gyro->lock);
	gyro->client = client;

	gyro->pdata = kmalloc(sizeof(*gyro->pdata), GFP_KERNEL);
	if (gyro->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}
	memcpy(gyro->pdata, client->dev.platform_data,
						sizeof(*gyro->pdata));
	gyro->pdata->i2c_client_dev=&client->dev;

	err = l3g4200d_validate_pdata(gyro);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, gyro);

	if (gyro->pdata->init_platform_hw) {
		err = gyro->pdata->init_platform_hw(gyro->pdata);
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}

	memset(gyro->resume_state, 0, ARRAY_SIZE(gyro->resume_state));

	gyro->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES;
	gyro->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	gyro->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	gyro->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	gyro->resume_state[RES_CTRL_REG5] = ALL_ZEROES;

	err = l3g4200d_device_power_on(gyro);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&gyro->enabled, 1);

	err = l3g4200d_update_fs_range(gyro, gyro->pdata->range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = l3g4200d_update_odr(gyro, gyro->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = l3g4200d_input_init(gyro);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", L3G4200D_GYR_DEV_NAME);
		goto err4;
	}


	/* create symlink to dev under /sys/devices */
	err = sysfs_create_link(&devices_kset->kobj, &client->dev.kobj,
						"gyroscope");
	if (err < 0) {
		dev_err(&client->dev,
			"create short path to input device failed");
		goto err4;
	}

	l3g4200d_device_power_off(gyro);

	/* As default, do not report information */
	atomic_set(&gyro->enabled, 0);

	mutex_unlock(&gyro->lock);

	dprintk("probed: %s device created successfully\n",
#ifdef CONFIG_VIA_POLLDEV
						"input poll"
#else
						"input"
#endif
							);
	return 0;

err4:
	l3g4200d_input_cleanup(gyro);
err3:
	l3g4200d_device_power_off(gyro);
err2:
	if (gyro->pdata->exit_platform_hw)
		gyro->pdata->exit_platform_hw(gyro->pdata);
err1_1:
	mutex_unlock(&gyro->lock);
	kfree(gyro->pdata);
err1:
	kfree(gyro);
err0:
		dev_err(&client->dev,
			"%s: Driver Initialization failed\n",
							L3G4200D_GYR_DEV_NAME);
		return err;
}

static int l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);
	
	dprintk("remove\n");
	
	gyro->input_dev->close(gyro->input_dev);
	l3g4200d_input_cleanup(gyro);
	remove_sysfs_interfaces(&client->dev);

	kfree(gyro->pdata);
	kfree(gyro);
	return 0;
}

static int l3g4200d_suspend(struct device *dev)
{
	
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);

	dprintk("suspend\n");

	gyro->on_before_suspend = atomic_read(&gyro->enabled);
#ifdef CONFIG_VIA_POLLDEV
	gyro->input_dev->close(gyro->input_dev);
#endif
	l3g4200d_disable(gyro);
#endif /*CONFIG_SUSPEND*/

	return 0;
}

static int l3g4200d_resume(struct device *dev)
{
	
#ifdef CONFIG_SUSPEND
	struct i2c_client *client = to_i2c_client(dev);
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);

	dprintk("resume\n");

	if (gyro->on_before_suspend) {
		l3g4200d_enable(gyro);
#ifdef CONFIG_VIA_POLLDEV
		gyro->input_dev->open(gyro->input_dev);
#endif
	}
#endif /*CONFIG_SUSPEND*/

	return 0;
}


static const struct i2c_device_id l3g4200d_id[] = {
	{ L3G4200D_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

static struct dev_pm_ops l3g4200d_pm = {
	.suspend = l3g4200d_suspend,
	.resume = l3g4200d_resume,
};

static struct i2c_driver l3g4200d_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = L3G4200D_GYR_DEV_NAME,
			.pm = &l3g4200d_pm,
	},
	.probe = l3g4200d_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.id_table = l3g4200d_id,

};

static int __init l3g4200d_init(void)
{
	dprintk("init\n");

	return i2c_add_driver(&l3g4200d_driver);
}

static void __exit l3g4200d_exit(void)
{
	dprintk("exit\n");

	i2c_del_driver(&l3g4200d_driver);
	return;
}

module_init(l3g4200d_init);
module_exit(l3g4200d_exit);

MODULE_DESCRIPTION("l3g4200d digital gyroscope sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
