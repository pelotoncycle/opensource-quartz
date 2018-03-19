
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "../../../arch/arm/mach-omap2/board-44xx-rb-common.h"
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/input/mt.h>

#define READ_TOUCH_ADDR_H   0x0F
#define READ_TOUCH_ADDR_L   0x40
#define READ_KEY_ADDR_H     0x0F
#define READ_KEY_ADDR_L     0x41
#define READ_COOR_ADDR_H    0x0F
#define READ_COOR_ADDR_L    0x42
#define READ_ID_ADDR_H      0x00
#define READ_ID_ADDR_L      0xff
#define READ_FW_MSG_ADDR_H    0x0F
#define READ_FW_MSG_ADDR_L    0x7C
#define UPDATE_FW_MSG_ADDR_H  0x40
#define UPDATE_FW_MSG_ADDR_L  0x50
#define READ_MSK_VER_ADDR_H   0xC0
#define READ_MSK_VER_ADDR_L   0x09

#define TOUCH_MAX_WIDTH INNOCOMM_TP_ABS_X_MAX
#define TOUCH_MAX_HEIGHT INNOCOMM_TP_ABS_Y_MAX

#define gt828_MULTI_TOUCH
#ifdef gt828_MULTI_TOUCH
	#define MAX_FINGER_NUM 5
#else
	#define MAX_FINGER_NUM 2
#endif

struct gt828_ts_data {		
	struct i2c_client *client;
	struct input_dev *input_dev;	
	int (*get_irq_state)(void);
	int (*init_hw)(struct touch_platform_data *pdata);
	void (*exit_hw)(struct touch_platform_data *pdata);
	int (*suspend_hw)(struct touch_platform_data *pdata);
	int (*resume_hw)(struct touch_platform_data *pdata);
	struct early_suspend early_suspend;
	spinlock_t lock;
	int irq;
	int reset_gpio;
	char phys[32];
	bool vlog;
};

struct i2c_client *gt828_i2c_client = NULL;
static struct kobject *gt828_debug_kobj;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt828_ts_early_suspend(struct early_suspend *h);
static void gt828_ts_late_resume(struct early_suspend *h);
#endif

/*******************************************************
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client: i2c device.
	buf[0-1]:operate address.
	buf[2-len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int gt828_i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retry = 0;
	
	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 2;
	msgs[0].buf = &buf[0];
	
	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len-2;
	msgs[1].buf = &buf[2];

	/*
	 * i2c_transfer() returns negative errno, else the number of messages executed
	 * touchscreen works in IIC wake up green mode, retry for remote I/O error, EREMOTEIO 121 
	 */
	while (retry < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2) 
			break; /* success */
		else {
			//dev_info(&client->dev, "%s(),retry(%d)reg(0x%x,0x%x)ret(%d)\n", __func__, retry, buf[0], buf[1], ret);
		}
		retry++;            
	}

	if (retry == 5) {
		dev_info(&client->dev, "%s(),error:reg(0x%x,0x%x)ret(%d)\n", __func__, buf[0], buf[1], ret);
	}

	return ret;
}

/*******************************************************
Description:
	write data to the i2c slave device.

Parameter:
	client: i2c device.
	buf[0-1]:operate address.
	buf[2-len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int gt828_i2c_write_bytes(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retry = 0;
	
	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;

	/*
	 * i2c_transfer() returns negative errno, else the number of messages executed
	 * touchscreen works in IIC wake up green mode, retry for remote I/O error, EREMOTEIO 121 
	 */
	while (retry < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) 
			break; /* success */
		else {
			//dev_info(&client->dev, "%s(),retry(%d)reg(0x%x,0x%x)ret(%d)\n", __func__, retry, data[0], data[1], ret);
		}
		retry++;            
	}

	if (retry == 5) {
		dev_info(&client->dev, "%s(),error:reg(0x%x,0x%x)ret(%d)\n", __func__, data[0], data[1], ret);
	}
	
	return ret;
}

// send pre command
static int gt828_i2c_pre_cmd(struct gt828_ts_data *ts)
{
	int ret;
	uint8_t pre_cmd_data[2] = { 0 };

	pre_cmd_data[0] = 0x0f;
	pre_cmd_data[1] = 0xff;
	ret = gt828_i2c_write_bytes(ts->client,pre_cmd_data, 2);
	//msleep(2);

	return ret;
}

// send end command
static int gt828_i2c_end_cmd(struct gt828_ts_data *ts)
{
	int ret;
	uint8_t end_cmd_data[2] = { 0 };
	
	end_cmd_data[0] = 0x80;
	end_cmd_data[1] = 0x00;
	ret = gt828_i2c_write_bytes(ts->client, end_cmd_data, 2);
	//msleep(2);
	
	return ret;
}

/* enable more debug log */
static ssize_t gt828_vlog_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gt828_ts_data *ts;

	ts = i2c_get_clientdata(gt828_i2c_client);
	if (ts == NULL) {
		return 0;
	}

	printk("%s, vlog(%d)\n", __func__, ts->vlog);
	ts->vlog = !ts->vlog;
	printk("%s, vlog(%d)\n", __func__, ts->vlog);
	
	return 0;
}

static DEVICE_ATTR(vlog, S_IRUGO, gt828_vlog_show, NULL);

static int gt828_debug_sysfs_init(void)
{
	int ret ;

	gt828_debug_kobj = kobject_create_and_add("tpdebug", NULL) ;
	if (gt828_debug_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(gt828_debug_kobj, &dev_attr_vlog.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_vlog_file failed\n", __func__);
		return ret;
	}
	printk("Goodix debug sysfs create success!\n");
	return 0 ;
}

static void gt828_debug_sysfs_deinit(void)
{
	sysfs_remove_file(gt828_debug_kobj, &dev_attr_vlog.attr);
	kobject_del(gt828_debug_kobj);
}

static void gt828_reset(struct gt828_ts_data *ts,s32 ms)
{
	int ret = -1;

	if ((system_rev >= 4)&&(ts->reset_gpio!=-1)) {	
		printk("%s\n", __func__);
		ret = gpio_request(ts->reset_gpio, "INNOCOMM_TP_RESET_GPIO");
		if (ret) {
			printk(KERN_ERR "Could not request GPIO %d\n", ts->reset_gpio);
			return;
		} else {	
			gpio_direction_output(ts->reset_gpio, 0);
			msleep(ms);
			gpio_direction_input(ts->reset_gpio);

			gpio_free(ts->reset_gpio);
			msleep(60);
		}
	} else {
		printk("%s not support! system_rev(%d)\n", __func__, system_rev);
	}
}

static bool gt828_init_panel(struct gt828_ts_data *ts)
{
	int ret = -1;
	uint8_t config_info[] = {
		0x0F,0x80, /*config address*/
		// 001019_TC978_GT828_Config_20120604_173105.cfg
		0x00,0x0F,0x01,0x10,0x02,0x11,0x03,0x12,
		0x04,0x13,0x05,0x14,0x06,0x15,0x07,0x16,
		0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,0x1A,
		0x0C,0x1B,0x0D,0x1C,0x0E,0x1D,0x13,0x09,
		0x12,0x08,0x11,0x07,0x10,0x06,0x0F,0x05,
		0x0E,0x04,0x0D,0x03,0x0C,0x02,0x0B,0x01,
		0x0A,0x00,0x1B,0x03,0xB0,0x10,0x10,0x2A,
		0x00,0x00,0x0A,0x00,0x00,0x0E,0x50,0x24,
		0x1D,// INT:1, rising edge trigger
		0x03,0x00,0x05,0x00,0x03,0x00,0x04,
		0x00,0x5A,0x5A,0x46,0x46,0x06,0x00,0x0A,
		0x19,0x00,0x14,0x10,0x00,0x04,0x00,0x00,
		0x00,0x00,0x00,0x00,0x38,0x28,0x60,0x20,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01
	};

	dev_info(&(ts->client->dev), "%s\n", __func__);

	ret = gt828_i2c_write_bytes(ts->client, config_info, (sizeof(config_info) /sizeof(config_info[0])));
	if (ret < 0) {
		return false;
	}
	gt828_i2c_end_cmd(ts);

	msleep(10);
	return true;
}

static int gt828_touch_num(uint8_t value, int max)
{
	int tmp = 0;

	while((tmp < max) && value)
	{
		if ((value & 0x01) == 1)
		{
			tmp++;
		}
		value = value >> 1;
	}

	return tmp;
}

/* If the track id can be found in touch id array, return true */
static bool gt828_search_touch_id(int8_t id, int8_t *array, int8_t array_len)
{
	int i;

	for (i = 0; i < array_len; i++) {
		if (id == array[i]) {
			return true;
		}
	}

	return false;
}

static irqreturn_t gt828_irq_thread_fn(int irq, void *handle)
{
	struct gt828_ts_data *ts = (struct gt828_ts_data*)handle;
	unsigned int input_x = 0, input_y = 0, input_w = 0;
	unsigned int count = 0, position = 0, point_count = 0;
	int i = 0, ret = -1;
	uint8_t touch_data[2 + 2 + 5*MAX_FINGER_NUM + 1] = { READ_TOUCH_ADDR_H, READ_TOUCH_ADDR_L, 0, 0 };
	int8_t track_id[MAX_FINGER_NUM];
	static int8_t track_id_last[MAX_FINGER_NUM];
	uint8_t finger = 0, chk_sum = 0, finger_bit = 0;
	uint8_t *coor_point;
	unsigned char index = 0, id = 0;
	bool to_report = false;
	
	if (ts->vlog) {
		printk("\n%s\n", __func__);
	}

	ret = gt828_i2c_read_bytes(ts->client, touch_data, sizeof(touch_data));
	gt828_i2c_end_cmd(ts);
	if(ret <= 0) {
		goto XFER_ERROR;
	}

	if (ts->vlog) {
		int j = 0;
		for (i = 0, j = 0; i < sizeof(touch_data); i++) {
			if (i == (4 + j * 5)) {
				printk("\n");
				j++;
			}
			printk("[%02d]=0x%02x ", i, touch_data[i]);
		}
		printk("\n");
	}

	if ((touch_data[2] & 0xC0) != 0x80) {
		// buffer state not ready
		dev_err(&(ts->client->dev), "DATA Not Ready!!\n");
		goto DATA_NOT_READY;
	}	
	
	if ((touch_data[3] & 0x0f) == 0x0f) {
		// if 0xF40 == 0x0F, reload config
		dev_err(&(ts->client->dev), "exception!! reload config, 0x0F40(%02x/%02x)\n", touch_data[2], touch_data[3]);
		if (!gt828_init_panel(ts)) {
			dev_err(&(ts->client->dev), "gt828_init_panel() failed!!\n");			
		}
		goto XFER_ERROR;
	}
	
	memset((void *)track_id, -1, sizeof(track_id));
	finger = (uint8_t)gt828_touch_num((touch_data[2] & 0x1f), MAX_FINGER_NUM);

	coor_point = &touch_data[4];
	chk_sum = 0;
	for ( i = 0; i < 5*finger; i++)
		chk_sum += coor_point[i];
	if (chk_sum != coor_point[5*finger]) {
		dev_err(&(ts->client->dev), "Checksum Error!!\n ");		
		goto XFER_ERROR;
	}

	if (finger) {
		point_count = 0, finger_bit = (touch_data[2] & 0x1f);
		for (count = 0; (finger_bit != 0) && (count < MAX_FINGER_NUM); count++) {
			if (finger_bit & 0x01) {
				track_id[point_count] = count;
				point_count++;
			}
			finger_bit >>= 1;
		}
	}

	if (ts->vlog) {
		printk("finger(%d),track_id[0]=%d,%d,%d,%d,%d\n", finger, track_id[0], track_id[1], track_id[2], track_id[3], track_id[4]);
		printk("track_id_last[0]=%d,%d,%d,%d,%d\n", track_id_last[0], track_id_last[1], track_id_last[2], track_id_last[3], track_id_last[4]);
	}
	
	for (id = 0, index = 0; id < MAX_FINGER_NUM; id++) {
		// track_id == 0, 1, 2, 3 or 4
		// track_id == -1 means no touch point
		bool current_track_id = gt828_search_touch_id(id, track_id, MAX_FINGER_NUM);
		bool previous_track_id = gt828_search_touch_id(id, track_id_last, MAX_FINGER_NUM);
		bool just_lift = ((previous_track_id && !current_track_id) ? true : false);

		if (current_track_id || previous_track_id) {
			if (just_lift) {
				if (ts->vlog) {
					printk("id(%d) released\n", id);
				}
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);				
			}
			else {
				position = 4 + 5 * index;
				input_y = ((unsigned int)(touch_data[position]<<8) + (unsigned int)( touch_data[position+1]));
				input_x = ((unsigned int)(touch_data[position+2]<<8) + (unsigned int)(touch_data[position+3]));
				input_w =(unsigned int)(touch_data[position+4]);

				if (ts->vlog) {
					printk("id(%d):x(%d)y(%d)w(%d)\n", id, input_x, input_y, input_w);
				}

				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				index++;
			}
			to_report = true;
		}
	}

	if (to_report) {
		input_sync(ts->input_dev);
	}
	
	for (index = 0; index < MAX_FINGER_NUM; index++) {
		track_id_last[index] = track_id[index];
	}
	

XFER_ERROR:
DATA_NOT_READY:

	return IRQ_HANDLED;	
}

// read 0xF7D, 0xF7E, 0xF7F ->
// Product ID, Product version High byte, Product version low byte
static int gt828_read_version(struct gt828_ts_data *ts)
{
	int ret = -1;
	uint8_t version_data[2+3] = {0x0F, 0x7D};

	ret = gt828_i2c_read_bytes(ts->client, version_data, sizeof(version_data));
	if (ret > 0) {
		printk("%s-Product ID(0x%02x)Product version(0x%02x,0x%02x)\n", 
			__func__, version_data[2], version_data[3], version_data[4]);
	}

	return 1;
}

/*******************************************************
Description:
	touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int gt828_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -EPERM, retry = 0;
	bool init_ret = false;
	struct gt828_ts_data *ts = NULL;
	struct touch_platform_data *pdata = NULL;	

	//dev_info(&client->dev, "%s()\n", __func__);

	// to support multiple touch panels, to check if already has tp probed
	if(client && client->dev.platform_data) {
		pdata = (struct touch_platform_data *)client->dev.platform_data;
		if (pdata->tp_probed) {
			dev_err(&client->dev, "tp already probed!\n");
			return -EPERM;
		}
	} else {
		dev_err(&client->dev, "client data not exit!\n");
		return -EPERM;
	}

	/* Check I2C function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "allocate gt828_ts_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	gt828_i2c_client = client;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	if (pdata) {
		ts->get_irq_state = pdata->get_irq_state;
		ts->init_hw = pdata->init_platform_hw;
		ts->exit_hw = pdata->exit_platform_hw;
		ts->suspend_hw = pdata->suspend_platform_hw;
		ts->resume_hw = pdata->resume_platform_hw;
		ts->reset_gpio =pdata->reset_gpio;
	}

	pdata->i2c_client_dev = &client->dev;

	if (ts->init_hw) {
		ret = ts->init_hw(pdata);
		if (ret < 0) {
			dev_err(&client->dev, "%s Power On Failed!!\n", __func__);
			goto err_power_failed;
		}
	}

	gt828_reset(ts,10);

	//Test I2C connection.    
    for (retry = 0; retry < 3; retry++) {
        ret = gt828_i2c_pre_cmd(ts);
        if (ret > 0)
            break;
        msleep(20);
    }	
    if(ret <= 0) {
        dev_err(&client->dev, "i2c ERROR!\n");
        goto err_i2c_error;
    }

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev,"gt828_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	spin_lock_init(&ts->lock);

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name =  "gt828-ts";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427; //screen firmware version

	/* Set default values: */
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	dev_info(&client->dev, "abs_x_max(%d)abs_y_max(%d)x_min(%d)y_min(%d)\n", 
        pdata->abs_x_max, 
        pdata->abs_y_max,
        pdata->abs_x_min,
        pdata->abs_y_min);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ts->irq = client->irq;
	if (ts->irq) {
		// set the inerrrupt to rising edge trigger according to the addr 0xFC0 (Md_switch:INT)
		ret = request_threaded_irq(ts->irq, NULL, gt828_irq_thread_fn, IRQF_TRIGGER_RISING, client->dev.driver->name, ts);
		if (ret == 0) {			
			disable_irq(ts->irq);
		}
		else {
			dev_err(&client->dev, "request irq failed\n");
			goto err_irq_request_failed;
		}
	}

	gt828_read_version(ts);
	
	for (retry = 0; retry < 3; retry++) {
		init_ret = gt828_init_panel(ts);
		if(init_ret)
			break;
		else
			continue;
	}
	if (!init_ret) {
		dev_info(&client->dev, "gt828_init_panel() failed!!\n");
		goto err_panel_init_failed;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = gt828_ts_early_suspend;
	ts->early_suspend.resume = gt828_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
	#endif

	gt828_debug_sysfs_init();

	enable_irq(ts->irq);
	gt828_i2c_end_cmd(ts);

	pdata->tp_probed = true;	
	return 0;

err_panel_init_failed:
	free_irq(client->irq, ts);
err_irq_request_failed:	
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_power_failed:	
err_i2c_error:	
err_alloc_data_failed:
	if (ts) {
		if (ts->exit_hw) {
			ts->exit_hw(pdata);
		}
		kfree(ts);
	}
err_check_functionality_failed:    
	return ret;
}

/*******************************************************
Description:
	touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int gt828_ts_remove(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct gt828_ts_data *ts = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s()\n", __func__);
    
	pdata = client->dev.platform_data;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif

	gt828_debug_sysfs_deinit();
	
	free_irq(ts->irq, ts);

	if (ts->exit_hw)
		ts->exit_hw(pdata);
	
	i2c_set_clientdata(client, NULL);
	input_mt_destroy_slots(ts->input_dev);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static void gt828_ts_shutdown(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct gt828_ts_data *ts = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s()\n", __func__);

	pdata = client->dev.platform_data;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif

	gt828_debug_sysfs_deinit();
	
	free_irq(client->irq, ts);

	if (ts->exit_hw) {
		ts->exit_hw(pdata);
	}

	input_mt_destroy_slots(ts->input_dev);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);

	kfree(ts);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt828_ts_early_suspend(struct early_suspend *h)
{
	struct gt828_ts_data *ts;
    struct touch_platform_data *pdata;
	int ret;
    
	ts = container_of(h, struct gt828_ts_data, early_suspend);
    pdata = ts->client->dev.platform_data;

	if (ts->vlog) {
		printk("%s\n", __func__);
	}
    
	disable_irq(ts->irq);	

	if (ts->suspend_hw) {
		ret=ts->suspend_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s suspend power failed\n", __func__);
		}
	}
}

static void gt828_ts_late_resume(struct early_suspend *h)
{
	struct gt828_ts_data *ts;
	struct touch_platform_data *pdata;
    int ret = 0, retry = 0;
    
	ts = container_of(h, struct gt828_ts_data, early_suspend);
	pdata = ts->client->dev.platform_data;

    if (ts->vlog) {
		printk("%s\n", __func__);
	}

	if (ts->resume_hw) {
		ret = ts->resume_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s power on failed\n", __func__);
			return;
		}
	}

    gt828_reset(ts,10);
	//Test I2C connection.    
    for (retry = 0; retry < 3; retry++) {
        ret = gt828_i2c_pre_cmd(ts);
        if (ret > 0)
            break;
        msleep(20);
    }

	// from power resume, GT828 will have interrupt with checksum error continusely
	// workaround >> reload config data
	if (!gt828_init_panel(ts)) {
		dev_err(&(ts->client->dev), "gt828_init_panel() failed!!\n");
	}

	enable_irq(ts->irq);
}
#endif

static const struct i2c_device_id gt828_ts_i2c_id[] = {
	{ "gt828-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, gt828_ts_i2c_id);

static struct i2c_driver gt828_ts_driver = {
	.id_table	= gt828_ts_i2c_id,
	.probe		= gt828_ts_probe,
	.remove		= gt828_ts_remove,
	.shutdown	= gt828_ts_shutdown,
	//.suspend	= gt828_ts_suspend,
	//.resume 	= gt828_ts_resume,
	.driver = {
		.name	=  "gt828-ts",
	},
};

static int __devinit gt828_ts_init(void)
{
	return i2c_add_driver(&gt828_ts_driver);
}

static void __exit gt828_ts_exit(void)
{
	i2c_del_driver(&gt828_ts_driver);
}

//module_init(gt828_ts_init);
late_initcall(gt828_ts_init);
module_exit(gt828_ts_exit);

MODULE_DESCRIPTION("Innocomm Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");

