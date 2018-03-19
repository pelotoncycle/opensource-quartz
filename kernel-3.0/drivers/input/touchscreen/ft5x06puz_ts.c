
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include "../../../arch/arm/mach-omap2/board-44xx-puzzle.h"

enum ft5x0x_ts_regs {
	FT5X0X_REG_THGROUP					= 0x80,
	FT5X0X_REG_THPEAK					= 0x81,
	FT5X0X_REG_TIMEENTERMONITOR			= 0x87,
	FT5X0X_REG_PERIODACTIVE				= 0x88,
	FT5X0X_REG_PERIODMONITOR			= 0x89,
	FT5X0X_REG_AUTO_CLB_MODE			= 0xa0,
	FT5X0X_REG_CIPHER					= 0xa3,
	FT5X0X_REG_PMODE					= 0xa5,	/* Power Consume Mode */	
	FT5X0X_REG_FIRMID					= 0xa6,
	FT5X0X_REG_FT5201ID					= 0xa8,
	FT5X0X_REG_ERR						= 0xa9,
	FT5X0X_REG_CLB						= 0xaa,
};

//FT5X0X_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03

#define MAX_FINGER_NUM 5
//#define VERBOSE_DEBUG

static struct i2c_client *this_client;

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;	
	int irq;
	int (*get_irq_state)(void);
	int (*init_hw)(struct touch_platform_data *pdata);
	void (*exit_hw)(struct touch_platform_data *pdata);
	int (*suspend_hw)(struct touch_platform_data *pdata);
	int (*resume_hw)(struct touch_platform_data *pdata);
	struct early_suspend early_suspend;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	bool just_resume;
};

//#define CONFIG_SUPPORT_FTS_CTP_UPG
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
typedef enum {
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
} E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;    //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL 0x0
#define FTS_TRUE 0x01
#define FTS_FALSE 0x0

#define I2C_CTPM_ADDRESS 0x70 //0xFF

#define FTS_PACKET_LENGTH 128
#define FTS_FIRMID 3
#define FTS_FW_SIZE 26480

int fts_ctpm_fw_upgrade_request_file(void);
#endif

struct ft5x0x_ts_data *g_ft5x0x_ts = NULL;

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&this_client->dev, "msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};
	
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

	buf[0] = addr;

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&this_client->dev, "msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;  
}

/* Active Low */
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static void ft5x0x_reset(void)
{
	printk("%s\n", __func__);
	
	gpio_set_value(PUZZLE_TP_RESET, 0);
	mdelay(5);
	gpio_set_value(PUZZLE_TP_RESET, 1);

	mdelay(300);
}
#endif

static void ft5x0x_read_chip_info(void)
{
	unsigned char info = 0xff;
	int retry = 0;
	int ret = -1;

	while (retry < 3) {
		ret = ft5x0x_read_reg(FT5X0X_REG_CIPHER, &info);
		if (ret > 0) {
			dev_info(&this_client->dev, "FT5X0X_REG_CIPHER(0x%x)\n", info);
			if (info == FT5X0X_REG_CIPHER) {
				// ic init hasn't finished
				msleep(300);
				retry++;
				continue;
			}
		} 
		break;
	}

	info = 0xff;
	ret = ft5x0x_read_reg(FT5X0X_REG_FIRMID, &info);
	if (ret > 0) {
		dev_info(&this_client->dev, "FT5X0X_REG_FIRMID(0x%x)\n", info);
	}

	info = 0xff;
	ret = ft5x0x_read_reg(FT5X0X_REG_FT5201ID, &info);
	if (ret > 0) {
		dev_info(&this_client->dev, "FT5X0X_REG_FT5201ID(0x%x)\n", info);
	}
}

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static unsigned char ft5x0x_read_fw_id(void)
{
	unsigned char info = 0xff;
	int ret = -1;
	
	ret = ft5x0x_read_reg(FT5X0X_REG_FIRMID, &info);
	if (ret > 0) {
		dev_info(&this_client->dev, "FT5X0X_REG_FIRMID(0x%x)\n", info);
	} else {
		info = 0xff;
	}

	return info;
}
#endif

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static unsigned char ft5x0x_read_ft5201_id(void)
{
	unsigned char info = 0xff;
	int ret = -1;
	
	ret = ft5x0x_read_reg(FT5X0X_REG_FT5201ID, &info);
	if (ret > 0) {
		dev_info(&this_client->dev, "FT5X0X_REG_FT5201ID(0x%x)\n", info);
	} else {
		info = 0xff;
	}

	return info;
}
#endif

static bool ft5x0x_search_touch_id(int8_t id, int8_t *array, int8_t array_len)
{
	int i;

	for (i = 0; i < array_len; i++) {
		if (id == array[i]) {
			return true;
		}
	}

	return false;
}

static void ft5x0x_clean_touch(struct ft5x0x_ts_data *ts) {
	int id = 0;
	for (id = 0; id < MAX_FINGER_NUM; id++) {
		input_mt_slot(ts->input_dev, id);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(ts->input_dev);
}


static irqreturn_t ft5x0x_irq_thread_fn(int irq, void *handle)
{
	struct ft5x0x_ts_data *ts = (struct ft5x0x_ts_data *)handle;
	int ret = -1;
	uint8_t point_data[3 + 6*MAX_FINGER_NUM + 1] = { 0 };
	unsigned int position = 0;
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned int x = 0, y = 0;
	unsigned char index = 0;
	unsigned char id = 0, id_xy = 0;
	int8_t track_id[MAX_FINGER_NUM];
	static int8_t track_id_last[MAX_FINGER_NUM];
	bool to_report = false;
	bool current_track_id = false, previous_track_id = false, just_lift = false;

	//dev_info(&this_client->dev, "%s\n", __func__);

	ret = ft5x0x_i2c_rxdata(point_data, sizeof(point_data));
	if (ret < 0)
		goto I2C_READ_ERROR;

	memset((void *)track_id, -1, sizeof(track_id));

	// get available touch
	for (index=0; index < MAX_FINGER_NUM; index++) {
		unsigned char _id = 0, _status = 0;
		
		position = 3 + 6*index;

		_id = (point_data[position + 2] >> 4);
		_status = (point_data[position] >> 6);

		if (_status == 0 || _status == 2) // down or move event
			track_id[index] = _id; // ID: 0-4
		else
			track_id[index] = -1;
	}

	if (ts->just_resume) {
		if ((track_id[0] == 0 && track_id[1] == 0) || (point_data[1] != 0) || (point_data[2] > MAX_FINGER_NUM)) {
			dev_info(&this_client->dev, "irq data ignored!!\n");
			memset((void *)track_id, -1, sizeof(track_id));
			memset((void *)track_id_last, -1, sizeof(track_id_last));
			goto I2C_READ_ERROR;
		}
	}

	for (id = 0, index = 0; id < MAX_FINGER_NUM; id++) {
		current_track_id = ft5x0x_search_touch_id(id, track_id, MAX_FINGER_NUM);
		previous_track_id = ft5x0x_search_touch_id(id, track_id_last, MAX_FINGER_NUM);
		just_lift = ((previous_track_id && !current_track_id) ? true : false);		

		if (current_track_id || previous_track_id) {
			if (just_lift) {
				#ifdef VERBOSE_DEBUG
				dev_info(&this_client->dev, "id(%d) released\n", id);
				#endif
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				to_report = true;
			}
			else {
				position = 3 + 6*index;
				x = (unsigned int) (point_data[position+1]) + (unsigned int)((point_data[position] & 0x0f) << 8);
				y = (unsigned int)(point_data[position+3]) + (unsigned int) ((point_data[position+2] & 0x0f) << 8);
				input_w = 100;
				id_xy = point_data[position+2] >> 4;				

				input_x = PUZZLE_TP_ABS_X_MAX - y -1;
				input_y = PUZZLE_TP_ABS_Y_MAX - x -1;

				if (ts->just_resume) {
					if (input_x > PUZZLE_TP_ABS_X_MAX || input_y > PUZZLE_TP_ABS_Y_MAX) {
						dev_info(&this_client->dev, "irq data ignored!!!\n");
						memset((void *)track_id, -1, sizeof(track_id));
						memset((void *)track_id_last, -1, sizeof(track_id_last));
						goto I2C_READ_ERROR;
					}
				}

				#ifdef VERBOSE_DEBUG
				dev_info(&this_client->dev, "id(%d),x(%d)y(%d),x_(%d)y_(%d)\n", 
					id_xy, input_x, input_y, x, y);
				#endif
				
				input_mt_slot(ts->input_dev, id_xy);
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
	if (ts->just_resume) {
		ts->just_resume = false;
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts = NULL;
	struct touch_platform_data *pdata = NULL;
	int ret = 0;

	printk("%s\n", __func__);
    
	ts = container_of(handler, struct ft5x0x_ts_data, early_suspend);
	pdata = ts->client->dev.platform_data;

	disable_irq(ts->irq);

	ft5x0x_clean_touch(ts);
	
	if (ts->suspend_hw) {
		ret = ts->suspend_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s suspend power failed\n", __func__);
		}
	}

}

static void ft5x0x_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts = NULL;
	struct touch_platform_data *pdata = NULL;
	int ret = 0;

	printk("%s\n", __func__);
    
	ts = container_of(handler, struct ft5x0x_ts_data, early_suspend);
	pdata = ts->client->dev.platform_data;

	if (ts->resume_hw) {
		ret = ts->resume_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s power on failed\n", __func__);
		}
	}

	ts->just_resume = true;

	enable_irq(ts->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct touch_platform_data *pdata = NULL;
	int err = 0, ret = 0;	
	
	dev_info(&client->dev, "%s.\n", __func__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_ft5x0x_ts = ft5x0x_ts;
	this_client = client;
	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);
	pdata = client->dev.platform_data;
	if (pdata) {
		ft5x0x_ts->init_hw = pdata->init_platform_hw;
		ft5x0x_ts->exit_hw = pdata->exit_platform_hw;
		ft5x0x_ts->suspend_hw = pdata->suspend_platform_hw;
		ft5x0x_ts->resume_hw = pdata->resume_platform_hw;
	}

	pdata->i2c_client_dev = &client->dev;
	
	if (ft5x0x_ts->init_hw) {
		ret = ft5x0x_ts->init_hw(pdata);
		if (ret < 0) {
			dev_err(&client->dev, "Power On Failed!!\n");
		}
	}

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	do {
		int retry = 0;
		unsigned char firmware_id = 0xff, ft5201_id = 0xff;
		
		mdelay(300);
		firmware_id = ft5x0x_read_fw_id();
		ft5201_id = ft5x0x_read_ft5201_id();
		if ((firmware_id != FTS_FIRMID && ft5201_id == 0x42) || 
			(firmware_id == 0xa6 && ft5201_id == 0xa8)) {
			while (retry < 7) {
				retry++;
				ret = fts_ctpm_fw_upgrade_request_file();
				if (ret == ERR_READID) {
					continue;
				} else if (ret == ERR_ECC) {
					mdelay(400);
					continue;
				}
				else {
					break;
				}
			}
			ft5x0x_reset();
		}
	} while (0);
#endif

	ft5x0x_ts->irq = client->irq;
	if (ft5x0x_ts->irq) {
		ret = request_threaded_irq(ft5x0x_ts->irq, NULL, ft5x0x_irq_thread_fn, IRQF_TRIGGER_FALLING, PUZZLE_TOUCH_NAME, ft5x0x_ts);
		if (ret == 0) {
			disable_irq(ft5x0x_ts->irq);
		}
		else {
			dev_err(&client->dev, "request_irq failed\n");
			goto exit_irq_request_failed;
		}
	}	

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;
	ft5x0x_ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) ;
	input_mt_init_slots(ft5x0x_ts->input_dev, MAX_FINGER_NUM);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	ft5x0x_ts->abs_x_max = pdata->abs_x_max;
	ft5x0x_ts->abs_y_max = pdata->abs_y_max;
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_POSITION_X, 0, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_POSITION_Y, 0, pdata->abs_y_max, 0, 0);
	dev_info(&ft5x0x_ts->client->dev,"X_MAX(%d)Y_MAX(%d)\n", pdata->abs_x_max, pdata->abs_y_max);

	input_dev->name	= PUZZLE_TOUCH_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	dev_info(&client->dev, "==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_early_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_late_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	ft5x0x_read_chip_info();

	enable_irq(ft5x0x_ts->irq);	

	dev_info(&client->dev, "==probe over =\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
//	free_irq(client->irq, ft5x0x_ts);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
exit_irq_request_failed:
	dev_info(&client->dev, "==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s..\n", __func__);
	
	pdata = client->dev.platform_data;	
	if (ft5x0x_ts->exit_hw) {
		ft5x0x_ts->exit_hw(pdata);
	}

	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	
	return 0;
}

static void ft5x0x_ts_shutdown(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

	printk("%s\n", __func__);

	pdata = client->dev.platform_data;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif	

	if (ts->exit_hw) {
		ts->exit_hw(pdata);
	}

	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);
	i2c_set_clientdata(client, NULL);
}

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
void delay_qt_ms(unsigned long  w_ms)
{
#if 1
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
#else
	mdelay(w_ms);
#endif
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}


static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if (ret <= 0) {
		printk("[TSP]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	if (ret <= 0) {
		printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{    
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
   	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_DWRD i = 0;

	FTS_DWRD  packet_number;
	FTS_DWRD  j;
	FTS_DWRD  temp;
	FTS_DWRD  lenght;
	FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_BYTE  bt_ecc;
	int i_ret;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5x0x_write_reg(0xfc,0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5x0x_write_reg(0xfc,0x55);
	printk("[TSP] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i ++;
		i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	} while (i_ret <= 0 && i < 5 );

	/*********Step 3:check READ-ID***********************/  
	cmd_write(0x90,0x00,0x00,0x00,4);
	byte_read(reg_val,2);
	printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3) {
		//printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else {
		return ERR_READID;
		//i_is_new_protocol = 1;
	}

	/*********Step 4:erase app*******************************/
	cmd_write(0x61,0x00,0x00,0x00,1);

	delay_qt_ms(1500);
	printk("[TSP] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++) {
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0) {
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);    
		delay_qt_ms(20);
	}

	//send the last six byte
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp = 1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/* send the opration head */
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc) {
		return ERR_ECC;
	}

	//reset_new_fw:	

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);
	printk("[TSP] Step 7: reset the new FW\n");

	return ERR_OK;
}

int fts_ctpm_fw_upgrade_request_file(void)
{	
	const struct firmware *fw = NULL;
	struct device *dev = &g_ft5x0x_ts->client->dev;
	const char *fw_name = "ft5xxx.bin";
	int i_ret = -1;
	int ret;

	ret = request_firmware(&fw, fw_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request fw: %s\n", fw_name);
	}
	else {
		dev_info(dev, "fw size(%d)\n", fw->size);
		if (fw->size == FTS_FW_SIZE) {
			i_ret = fts_ctpm_fw_upgrade((FTS_BYTE *)fw->data, fw->size);
			if (i_ret != ERR_OK) {
				printk("tp_fw upgrade failed!!\n");
			}
		}
		release_firmware(fw);
	}

	return i_ret;
}

#endif


static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ PUZZLE_TOUCH_NAME, 0 },{ }
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.shutdown	= ft5x0x_ts_shutdown,
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= PUZZLE_TOUCH_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x0x_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");


