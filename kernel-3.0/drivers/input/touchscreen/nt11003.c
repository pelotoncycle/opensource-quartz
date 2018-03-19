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
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include "../../../arch/arm/mach-omap2/board-44xx-rb-common.h"


#define MAX_FINGER_NUM 5

struct nt11003_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;	
	int irq;
	int reset_gpio;
	int (*get_irq_state)(void);
	int (*init_hw)(struct touch_platform_data *pdata);
	void (*exit_hw)(struct touch_platform_data *pdata);
	int (*suspend_hw)(struct touch_platform_data *pdata);
	int (*resume_hw)(struct touch_platform_data *pdata);
	struct early_suspend early_suspend;
	spinlock_t lock;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_ts_early_suspend(struct early_suspend *h);
static void nt11003_ts_late_resume(struct early_suspend *h);
#endif

/* Active Low */
static void nt11003_reset(struct nt11003_ts_data *ts)
{
	int ret = -1;

	ret = gpio_request(ts->reset_gpio, "TALOS_TP_RESET_GPIO");
	if (ret) {
		printk(KERN_ERR "Could not request GPIO %d\n", ts->reset_gpio);
	}
	else {
		gpio_direction_output(ts->reset_gpio, 0);
		msleep(10);
		gpio_set_value(ts->reset_gpio, 1);
		gpio_free(ts->reset_gpio);
	}
	
	msleep(300);
}

/*******************************************************
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int nt11003_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retry = 0;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];

	while (retry < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2) 
			break; /* success */
		retry++;
	}

	if (retry == 5) {
		dev_err(&client->dev, "%s(),error:reg(0x%x)ret(%d)\n", __func__, buf[0], ret);
	}

	return ret;
}

static int nt11003_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retry = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;

	while (retry < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) 
			break; /* success */
		retry++;            
	}

	if (retry == 5) {
		dev_err(&client->dev, "%s(),error:reg(0x%x)ret(%d)\n", __func__, data[0], ret);
	}
    
	return ret;
}

static int nt11003_write_command_address(struct nt11003_ts_data *ts)
{
	uint8_t test_data[5] = {0x3C,0x0, 0x0};
	int ret = 0;
	
	test_data[0] = 0xFF;
	test_data[1] = 0x8B;
	test_data[2] = 0x00;
	ret = nt11003_write_bytes(ts->client, test_data, 3);
	if (ret != 1) {
		dev_info(&ts->client->dev, "Write Command Address Failed!\n");
	}

	return ret;
}

/*
 * NT11003 utilizes high voltage (15V) driving pulse. Host should issue power off sequence to release
 * VINT1,2,3 voltages when host execute power off
 */
static int nt11003_prepoweroff_cmd(struct nt11003_ts_data *ts)
{
	uint8_t test_data[5];
	int ret = 0;
	
	test_data[0] = 0xFF;
	test_data[1] = 0x8F;
	test_data[2] = 0xFF;
	ret = nt11003_write_bytes(ts->client, test_data, 3);
	if (ret != 1) {
		dev_info(&ts->client->dev, "%s failed\n", __func__);
		return ret;
	}

	test_data[0] = 0x00;
	test_data[1] = 0xAF;
	ret = nt11003_write_bytes(ts->client, test_data, 2);
	if (ret != 1) {
		dev_info(&ts->client->dev, "%s failedd\n", __func__);
		return ret;
	}

	msleep(50);

	return ret;
}


bool nt11003_search_touch_id(int8_t id, int8_t *array, int8_t array_len)
{
	int i;

	for (i = 0; i < array_len; i++) {
		if (id == array[i]) {
			return true;
		}
	}

	return false;
}

static irqreturn_t nt11003_irq_thread_fn(int irq, void *handle)
{
	struct nt11003_ts_data *ts = (struct nt11003_ts_data *)handle;
	int ret = -1;
	uint8_t point_data[1 + 6*MAX_FINGER_NUM + 1] ={ 0};
	unsigned int position = 0;	
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char id = 0;
	int8_t track_id[MAX_FINGER_NUM];
	static int8_t track_id_last[MAX_FINGER_NUM];
	bool to_report = false;

	ret = nt11003_read_bytes(ts->client, point_data, sizeof(point_data)/sizeof(point_data[0]));
	if (ret < 0)
		goto I2C_READ_ERROR;

	memset((void *)track_id, -1, sizeof(track_id));

	// get available touch
	for (index=0; index < MAX_FINGER_NUM; index++)
	{
		unsigned char _id = 0, _status = 0;
		
		position = 1 + 6*index;

		_id = ((point_data[position] & 0xf8) >> 3);
		_status = (point_data[position] & 0x03);

		if (_status == 1 || _status == 2) // enter or move event
			track_id[index] = _id-1; // ID: 0-4
		else
			track_id[index] = -1;
	}	

	for (id = 0, index = 0; id < MAX_FINGER_NUM; id++) {
		bool current_track_id = nt11003_search_touch_id(id, track_id, MAX_FINGER_NUM);
		bool previous_track_id = nt11003_search_touch_id(id, track_id_last, MAX_FINGER_NUM);
		bool just_lift = ((previous_track_id && !current_track_id) ? true : false);

		if (current_track_id || previous_track_id) {
			if (just_lift) {
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				index++;
				to_report = true;
			}
			else {
				position = 1 + 6*index;			
				input_x = (unsigned int) (point_data[position+1]<<4) + (unsigned int)( point_data[position+3]>>4);
				input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
				input_w =(unsigned int) (point_data[position+4])+1;
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				index++;
				to_report = true;
			}
		}
	}
    
	if (to_report) {
		input_sync(ts->input_dev);
	}

	memcpy((void *)track_id_last, (const void *)track_id, sizeof(track_id_last));

I2C_READ_ERROR:
	return IRQ_HANDLED;
}

static int nt11003_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct nt11003_ts_data *ts = NULL;
	struct touch_platform_data *pdata = NULL;

	/* Check I2C function */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "allocate nt11003_ts_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}	

	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata) {
		ts->get_irq_state = pdata->get_irq_state;
		ts->init_hw = pdata->init_platform_hw;
		ts->exit_hw = pdata->exit_platform_hw;
		ts->suspend_hw = pdata->suspend_platform_hw;
		ts->resume_hw = pdata->resume_platform_hw;
		#ifdef TOUCH_TO_SET_FREQ
		ts->set_min_mpu_freq = pdata->set_min_mpu_freq;
		#endif //#ifdef TOUCH_TO_SET_FREQ
		ts->reset_gpio = pdata->reset_gpio;
	}

	pdata->i2c_client_dev = &client->dev;
	
	if (ts->init_hw) {
		ret = ts->init_hw(pdata);
		if (ret < 0) {
			dev_err(&client->dev, "Power On Failed!!\n");
			goto err_power_failed;
		}
	}

	nt11003_reset(ts);
	ret = nt11003_write_command_address(ts);
	if (ret != 1) {
		goto err_i2c_failed;
	}

	ts->abs_x_max = pdata->abs_x_max;
	ts->abs_y_max = pdata->abs_y_max;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev,"nt11003_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	spin_lock_init(&ts->lock);

	ts->input_dev->name = "nt11003_tp";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	/* Set default values: */
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) ;
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	dev_info(&ts->client->dev,"X_MAX(%d)Y_MAX(%d)\n", ts->abs_x_max, ts->abs_y_max);
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ts->irq = client->irq;
	if (ts->irq) {
		ret = request_threaded_irq(ts->irq, NULL, nt11003_irq_thread_fn, IRQF_TRIGGER_FALLING, client->dev.driver->name, ts);
		if (ret == 0) {
			disable_irq(ts->irq);
		}
		else {
			dev_err(&client->dev, "request_irq failed\n");
			goto err_irq_request_failed;
		}
	}	
    
	#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nt11003_ts_early_suspend;
	ts->early_suspend.resume = nt11003_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
	#endif

	enable_irq(ts->irq);
	return 0;

err_irq_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_i2c_failed:	
	if(ts && ts->exit_hw) {
		ts->exit_hw(pdata);
	}
err_power_failed:
	i2c_set_clientdata(client, NULL);
	if (ts) {
		kfree(ts);
	}		
err_check_functionality_failed:
err_alloc_data_failed:
	return ret;
}

static int nt11003_ts_remove(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s()\n", __func__);
    
	pdata = client->dev.platform_data;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif

	free_irq(ts->irq, ts);

	nt11003_prepoweroff_cmd(ts);
	
	if (ts->exit_hw)
		ts->exit_hw(pdata);
	
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static void nt11003_ts_shutdown(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

	pdata = client->dev.platform_data;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif

	free_irq(client->irq, ts);

	nt11003_prepoweroff_cmd(ts);

	if (ts->exit_hw) {
		ts->exit_hw(pdata);
	}

	input_unregister_device(ts->input_dev);

	kfree(ts);
}

static void nt11003_clean_touch(struct nt11003_ts_data *ts) {
	int id = 0;
	for (id = 0; id < MAX_FINGER_NUM; id++) {
		input_mt_slot(ts->input_dev, id);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(ts->input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_ts_early_suspend(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;
    struct touch_platform_data *pdata;
	int ret;
	uint8_t version[5] = {0x3C,0x0, 0x0};
    
	ts = container_of(h, struct nt11003_ts_data, early_suspend);
    pdata = ts->client->dev.platform_data;

	disable_irq(ts->irq);

	ret = nt11003_read_bytes(ts->client, version, 3);
	if (ret == 2) {
		dev_info(&ts->client->dev, "FW(0x%02x,0x%02x)", version[1], version[2]);
	}

	nt11003_prepoweroff_cmd(ts);
	nt11003_clean_touch(ts);
	
	if (ts->suspend_hw) {
		ret=ts->suspend_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s suspend power failed\n", __func__);
		}
	}
}

static void nt11003_ts_late_resume(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;
	struct touch_platform_data *pdata;
    int ret;
    
	ts = container_of(h, struct nt11003_ts_data, early_suspend);
	pdata = ts->client->dev.platform_data;

	if (ts->resume_hw) {
		ret = ts->resume_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s power on failed\n", __func__);
		}
	}

    nt11003_reset(ts);
	nt11003_write_command_address(ts);

	enable_irq(ts->irq);
}
#endif

static const struct i2c_device_id nt11003_ts_i2c_id[] = {
	{ "nt11003_tp", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, nt11003_ts_i2c_id);

static struct i2c_driver nt11003_ts_driver = {
	.id_table	= nt11003_ts_i2c_id,
	.probe		= nt11003_ts_probe,
	.remove		= nt11003_ts_remove,
	.shutdown	= nt11003_ts_shutdown,
	.driver = {
		.name	= "nt11003_tp",
	},
};

static int __devinit nt11003_ts_init(void)
{
	return i2c_add_driver(&nt11003_ts_driver);
}

static void __exit nt11003_ts_exit(void)
{
	i2c_del_driver(&nt11003_ts_driver);
}

late_initcall(nt11003_ts_init);
module_exit(nt11003_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
