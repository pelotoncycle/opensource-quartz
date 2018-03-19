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
#include <linux/string.h>
#include <linux/input/mt.h>

#define READ_COOR_ADDR 0x01

//set GT801 PLUS trigger mode
#define INT_TRIGGER	0
/*
#if INT_TRIGGER==0
	#define GT801_PLUS_IRQ_TYPE IRQ_TYPE_EDGE_RISING
#elif INT_TRIGGER==1
	#define GT801_PLUS_IRQ_TYPE IRQ_TYPE_EDGE_FALLING
#elif INT_TRIGGER==2
	#define GT801_PLUS_IRQ_TYPE IRQ_TYPE_LEVEL_LOW
#elif INT_TRIGGER==3
	#define GT801_PLUS_IRQ_TYPE IRQ_TYPE_LEVEL_HIGH
#endif
*/

#define GOODIX_MULTI_TOUCH
#ifdef GOODIX_MULTI_TOUCH
	#define MAX_FINGER_NUM	5
#else
	#define MAX_FINGER_NUM	1
#endif

#ifdef TOUCH_TO_SET_FREQ
#define VDD1_FREQ_CONST	(cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 600000000 : 500000000) : 0)
#define VDD1_FREQ_MIN	(cpu_is_omap34xx() ? \
(cpu_is_omap3630() ? 300000000 : 125000000) : 0)
#endif //#ifdef TOUCH_TO_SET_FREQ


struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	uint8_t bad_data;
	int retry;
	int use_irq;
	int irq;
  	int reported_finger_count;
	int pendown;
	int	(*get_irq_state)(void);
	int	 (*init_hw)(struct touch_platform_data *pdata);
	void (*exit_hw)(struct touch_platform_data *pdata);
	int	 (*suspend_hw)(struct touch_platform_data *pdata);
	int	 (*resume_hw)(struct touch_platform_data *pdata);
#ifdef TOUCH_TO_SET_FREQ
	void (*set_min_mpu_freq)(struct device *dev, unsigned long f);
#endif //#ifdef TOUCH_TO_SET_FREQ
	struct early_suspend early_suspend;
	spinlock_t	lock;
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	int read_mode;		//read moudle mode
	int isUpdating;
#endif
    int version;
	bool vlog;
};

//For Guitar IAP firmware update
struct i2c_client * i2c_connect_client = NULL; 
static struct proc_dir_entry *goodix_proc_entry;
static struct kobject *goodix_debug_kobj;

#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
#include <asm/uaccess.h>

#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/proc_fs.h>

static int goodix_update_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static int goodix_update_read( char *page, char **start, off_t off, int count, int *eof, void *data );
//#define UPDATE_NEW_PROTOCOL

unsigned int oldcrc32 = 0xFFFFFFFF;
unsigned int crc32_table[256];
unsigned int ulPolynomial = 0x04c11db7;

#define PACK_SIZE 					64					//update file package size
#define MAX_TIMEOUT					60000				//update time out conut
#define MAX_I2C_RETRIES				20					//i2c retry times

//I2C buf address
#define ADDR_CMD					80
#define ADDR_STA					81
#ifdef UPDATE_NEW_PROTOCOL
	#define ADDR_DAT				0
#else
	#define ADDR_DAT				82
#endif

//moudle state
#define UPDATE_START				0x02
#define SLAVE_READY					0x08
#define UNKNOWN_ERROR				0x00
#define FRAME_ERROR					0x10
#define CHECKSUM_ERROR				0x20
#define TRANSLATE_ERROR				0x40
#define FLASH_ERROR					0X80

//error no
#define ERROR_NO_FILE				2	//ENOENT
#define ERROR_FILE_READ				23	//ENFILE
#define ERROR_FILE_TYPE				21	//EISDIR
#define ERROR_GPIO_REQUEST			4	//EINTR
#define ERROR_I2C_TRANSFER			5	//EIO
#define ERROR_NO_RESPONSE			16	//EBUSY
#define ERROR_TIMEOUT				110	//ETIMEDOUT

//update steps
#define STEP_SET_PATH				1
#define STEP_CHECK_FILE				2
#define STEP_WRITE_SYN				3
#define STEP_WAIT_SYN				4
#define STEP_WRITE_LENGTH			5
#define STEP_WAIT_READY				6
#define STEP_WRITE_DATA				7
#define STEP_READ_STATUS			8
#define FUN_CLR_VAL					9
#define FUN_CMD						10
#define FUN_WRITE_CONFIG			11

//fun cmd
#define CMD_DISABLE_TP				0
#define CMD_ENABLE_TP				1
#define CMD_READ_VER				2
#define CMD_READ_RAW				3
#define CMD_READ_DIF				4
#define CMD_READ_CFG				5
#define CMD_SYS_REBOOT				101

//read mode
#define MODE_RD_VER					1
#define MODE_RD_RAW					2
#define MODE_RD_DIF					3
#define MODE_RD_CFG					4

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

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
static int goodix_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];

	ret=i2c_transfer(client->adapter,msgs, 2);
	if (ret < 0) {
		dev_err(&client->dev, "read from addr:0x%x , error:%d\n", buf[0], ret);
	}
	return ret;
}

/*******************************************************
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int goodix_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	
	ret=i2c_transfer(client->adapter,&msg, 1);

	if (ret < 0) {
		dev_err(&client->dev, "write to addr:0x%x , error:%d\n", data[0], ret);
	}
	return ret;
}

/*******************************************************
Description:
	Goodix touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int goodix_init_panel(struct goodix_ts_data *ts)
{
	int ret=-1;

	if(ts->version >= 313) {
		uint8_t config_info[]= { 
			0x65,0x02,0x04,0x00,0x03,0x00,MAX_FINGER_NUM,0x3C,0x1E,0xE7,0x32,0x02,
			0x08,0x10,0x4C,0x46,0x46,0x20,0x00,0x00,0x63,0x63,0x3C,0x8C,
			0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
			0x0C,0x0E,0x0D,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
			0x18,0x19,0x1A,0x1B,0x1D,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00 };
		ret = goodix_write_bytes(ts->client,config_info, (sizeof(config_info)/sizeof(config_info[0])));
		dev_info(&ts->client->dev, "Load New Config : version:%d\n", ts->version);
	}
	else
	{
		uint8_t config_info[]= { 
			0x65,0x02,0x04,0x00,0x03,0x00,MAX_FINGER_NUM,0x3C,0x1E,0xE7,0x32,0x02,
			0x08,0x10,0x4C,0x47,0x47,0x20,0x03,0x00,0x60,0x60,0x3C,0x96,
			0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
			0x0C,0x0E,0x0D,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
			0x18,0x19,0x1A,0x1B,0x1D,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		ret = goodix_write_bytes(ts->client,config_info, (sizeof(config_info)/sizeof(config_info[0])));
		dev_info(&ts->client->dev, "Load Old Config : version:%d\n", ts->version);
	}

	if (ret < 0) 
		return ret;
	
	msleep(10);
	
	return 0;
}

/*******************************************************
Description:
	Read goodix touchscreen version function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int  goodix_read_version(struct goodix_ts_data *ts, char **version)
{
	int ret = -1, count = 0;
	//unsigned char version_data[18];
	char *version_data;
	char *p;

	*version = (char *)vmalloc(18);
	version_data = *version;
	if(!version_data)
		return -ENOMEM;
	p = version_data;
	memset(version_data, 0, sizeof(version_data));
	version_data[0]=240;
	ret=goodix_read_bytes(ts->client,version_data, 17);
	if (ret < 0) 
		return ret;
	version_data[17]='\0';

	if(*p == '\0')
		return 0;
	do
	{
		if((*p > 122) || (*p < 48 && *p != 32) || (*p >57 && *p  < 65)
			||(*p > 90 && *p < 97 && *p  != '_'))		//check illeqal character
			count++;
	}while(*++p != '\0' );
	if(count > 2)
		return 0;
	else
		return 1;
}

static void goodix_parse_version(struct goodix_ts_data *ts, char *version)
{
    char* buf = (char *)vmalloc(3);
	char *p = strchr(version, '_'); // only need the second part

	if (p)
	{
		p++;
	    //The version rule is 3R13 3 is major version, 13 is minor version
	    //And 3R13 means 3.13
	    memset(buf, 0, sizeof(buf));
	    buf[0] = p[0];
	    buf[1] = p[2];
	    buf[2] = p[3];

	    ts->version = simple_strtol(buf, NULL, 10);
	}
	else {
		ts->version = 0;
	}
   
}

#ifdef TOUCH_TO_SET_FREQ
void goodix_ts_set_min_mpu_freq(struct goodix_ts_data *ts, int fingers){
	if(ts->reported_finger_count == 0){
		if(fingers){
			//Set min mpu
			//omap_pm_set_min_bus_tput(&ts->client->dev, OCP_INITIATOR_AGENT, 166*1000*4);
			ts->set_min_mpu_freq(&ts->client->dev, VDD1_FREQ_CONST);
		}
	}else{
		if(!fingers){
			//release set
			//omap_pm_set_min_bus_tput(&ts->client->dev, OCP_INITIATOR_AGENT, 0);
			ts->set_min_mpu_freq(&ts->client->dev, VDD1_FREQ_MIN);
		}
	}
}
#endif //#ifdef TOUCH_TO_SET_FREQ

/* If the track id can be found in touch id array, return true */
bool goodix_search_touch_id(int8_t id, int8_t *array, int8_t array_len)
{
	int i;

	for (i = 0; i < array_len; i++) {
		if (id == array[i]) {
			return true;
		}
	}

	return false;
}

static irqreturn_t goodix_irq_thread_fn(int irq, void *handle)
{
	struct goodix_ts_data *ts = (struct goodix_ts_data*)handle;
	int ret=-1;
	uint8_t point_data[(1-READ_COOR_ADDR)+1+2+5*MAX_FINGER_NUM+1]={ 0 }; 
	uint8_t check_sum = 0;
	uint16_t finger_current = 0;
	uint16_t finger_bit = 0;
	unsigned int count = 0, point_count = 0;
	unsigned int position = 0;
	int8_t track_id[MAX_FINGER_NUM];
	static int8_t track_id_last[MAX_FINGER_NUM];
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0, id = 0;
	unsigned char touch_num = 0;

	if (ts->vlog) {
		printk("\n%s\n", __func__);
	}

	if(ts->bad_data) {
		//TODO:Is sending config once again (to reset the chip) useful?	
		dev_err(&ts->client->dev," innocomm_goodix ts->bad_data !!!\n ");
		msleep(20);
	}

	memset((void *)track_id, -1, MAX_FINGER_NUM);

	point_data[0] = READ_COOR_ADDR;		//read coor address
	ret=goodix_read_bytes(ts->client, point_data, sizeof(point_data)/sizeof(point_data[0]));
	if(ret <= 0) {
		dev_err(&ts->client->dev,"I2C transfer error. Number:%d\n ", ret);
		ts->bad_data = 1;
		ts->retry++;
		goto XFER_ERROR;
	}

	if (ts->vlog) {
		int i = 0, j = 0;
		for (i = 0, j = 0; i < sizeof(point_data); i++) {
			if (i == (3 + j * 5)) {
				printk("\n");
				j++;
			}
			printk("[%02d]=0x%02x ", i, point_data[i]);
		}
		printk("\n");
	}
	
	ts->bad_data = 0; 
	finger_current = (point_data[3 - READ_COOR_ADDR] << 8) + point_data[2 - READ_COOR_ADDR];
	
	if (ts->vlog) {
		dev_info(&ts->client->dev, "finger_current=%d\n", finger_current);
	}
	
	//The bits indicate which finger(s) pressed down
	if(finger_current) {
		point_count = 0, finger_bit = finger_current;
		for(count = 0; (finger_bit != 0) && (count < MAX_FINGER_NUM); count++) {
			if(finger_bit & 0x01) {
				track_id[point_count] = count;
				point_count++;
			}
			finger_bit >>= 1;
		}		
		
		touch_num = point_count;
		check_sum = point_data[2 - READ_COOR_ADDR] + point_data[3 - READ_COOR_ADDR]; 			//cal coor checksum
		count = 4 - READ_COOR_ADDR;
		for(point_count *= 5; point_count > 0; point_count--)
			check_sum += point_data[count++];
		check_sum += point_data[count];
		if(check_sum  != 0) {
			//checksum verify error
			if (ts->vlog) {
				dev_info(&ts->client->dev, "Check_sum:%d, Data:%d\n", check_sum, point_data[count]);
				printk(KERN_INFO "Finger Bit:%d\n",finger_current);
				for( ; count > 0; count--)
					printk(KERN_INFO "count=%d:%d ",count, point_data[count]);
				printk(KERN_INFO "\n");
			}
			
			printk("coor checksum error!\n");
			goto XFER_ERROR;
		}
	}

	if (ts->vlog) {
		printk("finger_bit(0x%x),track_id[0]=%d,%d,%d,%d,%d\n", 
			finger_bit, track_id[0], track_id[1], track_id[2], track_id[3], track_id[4]);
		printk("track_id_last[0]=%d,%d,%d,%d,%d\n", 
			track_id_last[0], track_id_last[1], track_id_last[2], track_id_last[3], track_id_last[4]);
	}
	
	for (id = 0, index = 0; id < MAX_FINGER_NUM; id++) {		
		// track_id == 0, 1, 2, 3 or 4
		// track_id == -1 means no touch point
		bool current_track_id = goodix_search_touch_id(id, track_id, MAX_FINGER_NUM);
		bool previous_track_id = goodix_search_touch_id(id, track_id_last, MAX_FINGER_NUM);
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
				position = 4 - READ_COOR_ADDR + 5 * index;
				input_x = (unsigned int)(point_data[position]<<8) + (unsigned int)( point_data[position+1]);
				input_y = (unsigned int)(point_data[position+2]<<8) + (unsigned int)(point_data[position+3]);
				input_w =(unsigned int)(point_data[position+4]);

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
		}
	}	
	for (index = 0; index < MAX_FINGER_NUM; index++) {
		track_id_last[index] = track_id[index];
	}

	input_sync(ts->input_dev);


XFER_ERROR:

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Goodix debug sysfs cat version function.

Parameter:
	standard sysfs show param.
	
return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int len; 
	char *version_info = NULL;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;

	ret = goodix_read_version(ts, &version_info);
	if(ret <= 0)
	{
		printk(KERN_INFO"Read version data failed!\n");
		vfree(version_info);
		return 0;
	}

	for(len=0;len<100;len++)
	{
		if(*(version_info + len) == '\0')
			break;
	}
	printk(KERN_INFO"Goodix TouchScreen Version:%s\n", (version_info+1));
	strncpy(buf, version_info+1, len + 1);
	vfree(version_info);

	ret = len + 1;
	return ret;
}

/* enable more debug log */
static ssize_t vlog_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	if (ts == NULL) {
		return 0;
	}

	printk("%s, vlog(%d)\n", __func__, ts->vlog);

	ts->vlog = !ts->vlog;

	printk("%s, vlog(%d)\n", __func__, ts->vlog);
	
	return 0;
}

static DEVICE_ATTR(version, S_IRUGO, goodix_debug_version_show, NULL);
static DEVICE_ATTR(vlog, S_IRUGO, vlog_show, NULL);

/*******************************************************
Description:
	Goodix debug sysfs init function.

Parameter:
	none.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_debug_sysfs_init(void)
{
	int ret ;

	goodix_debug_kobj = kobject_create_and_add("tpdebug", NULL) ;
	if (goodix_debug_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_version.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_vlog.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_vlog_file failed\n", __func__);
		return ret;
	}
	printk("Goodix debug sysfs create success!\n");
	return 0 ;
}

static void goodix_debug_sysfs_deinit(void)
{
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_version.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_vlog.attr);
	kobject_del(goodix_debug_kobj);
}

/*******************************************************
Description:
	Goodix touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -EPERM;
	int retry = 0;
	struct goodix_ts_data *ts = NULL;
	char *version_info = NULL;
	struct touch_platform_data *pdata = NULL;
	char version_data[7] = { 0x65 };

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

	// Check I2C function
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "allocate goodix_ts_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	//pdata = client->dev.platform_data;
	if (pdata) {
		ts->get_irq_state = pdata->get_irq_state;
		ts->init_hw = pdata->init_platform_hw;
		ts->exit_hw = pdata->exit_platform_hw;
		ts->suspend_hw = pdata->suspend_platform_hw;
		ts->resume_hw = pdata->resume_platform_hw;
#ifdef TOUCH_TO_SET_FREQ
		ts->set_min_mpu_freq = pdata->set_min_mpu_freq;
#endif //#ifdef TOUCH_TO_SET_FREQ
	}

	pdata->i2c_client_dev = &client->dev;

	if (ts->init_hw) {
		ret = ts->init_hw(pdata);
		if (ret < 0) {
			dev_err(&client->dev, "GOODIX Power On Failed!!\n");
			goto err_power_failed;
		}
	}

	i2c_connect_client = client;	//used by Guitar_Update

	// test i2c communication
	for (retry = 0; retry < 3; retry++) {
		ret = goodix_read_bytes(ts->client, version_data, sizeof(version_data));
		if (ret <= 0) {
			dev_err(&client->dev, "retry(%d)\n", retry);
	        msleep(20);
	    } else {
			dev_info(&client->dev, "ROTATE(%d)X-HIGH(0x%x)X-LOW(0x%x)Y-HIGH(0x%x)Y-LOW(0x%x)FINGER(%d)\n", 
				version_data[1], version_data[2],
				version_data[3], version_data[4],
				version_data[5], version_data[6]);
			break;
		}
	}
    if(ret <= 0) {
        dev_err(&client->dev, "i2c ERROR!\n");
        goto err_i2c_error;
    }

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev,"goodix_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	spin_lock_init(&ts->lock);

	ts->input_dev->name = "gt801-ts";
	//ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;	//screen firmware version

	//Set default values:
	ts->pendown = 0;
	ts->reported_finger_count =0;

	/*@TODO: Hardcode here need move to header file*/
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,"Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}


	ts->bad_data = 0;
	ts->irq = client->irq;
	if (ts->irq) {
		ret = request_threaded_irq(ts->irq, NULL, goodix_irq_thread_fn, IRQF_TRIGGER_RISING, client->dev.driver->name, ts);
		if (ret == 0) {
			ts->use_irq = 1;
			disable_irq(ts->irq);
		}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}	

	dev_err(&client->dev, "registered with irq (%d)\n", ts->irq);

	// read touch firmware version
	ret = goodix_read_version(ts, &version_info);
	if (ret <= 0) {
		printk(KERN_INFO"Read version data failed!\n");
	} else {
		printk(KERN_INFO"Goodix TouchScreen Version:%s\n", (version_info+1));
		goodix_parse_version(ts, version_info+1);
	}
	vfree(version_info);
	
	ret = goodix_init_panel(ts);
	if (ret != 0) { //Initiall failed
		ts->bad_data=1;
		goto err_panel_init_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	goodix_proc_entry = create_proc_entry("goodix-update", 0666, NULL);
	if (goodix_proc_entry == NULL) {
		dev_info(&client->dev, "Couldn't create proc entry!\n");
		ret = -ENOMEM;
		goto err_create_proc_entry;
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		goodix_proc_entry->write_proc = goodix_update_write;
		goodix_proc_entry->read_proc = goodix_update_read;
		//goodix_proc_entry->owner =THIS_MODULE;
	}
#endif
	goodix_debug_sysfs_init();
	dev_info(&client->dev, "Start touchscreen %s in %s mode\n",
		ts->input_dev->name, (ts->use_irq ? "interrupt" : "polling"));
	enable_irq(ts->irq);

	pdata->tp_probed = true;	
	return 0;

err_create_proc_entry:
err_panel_init_failed:
	free_irq(client->irq, ts);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_alloc_data_failed:
err_i2c_error:
err_power_failed:	
	if (ts) {
	if(ts->exit_hw) {
		ts->exit_hw(pdata);
	}
	kfree(ts);
	}
err_check_functionality_failed:	
	return ret;
}


/*******************************************************
Description:
	Goodix touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	remove_proc_entry("goodix-update", NULL);
#endif

	goodix_debug_sysfs_deinit();

	free_irq(ts->irq, ts);

	if(ts->exit_hw) {
		ts->exit_hw(pdata);
	}

	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static void goodix_ts_shutdown(struct i2c_client *client)
{
	struct touch_platform_data *pdata;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	remove_proc_entry("goodix-update", NULL);
#endif

	goodix_debug_sysfs_deinit();

	free_irq(client->irq, ts);

	if(ts->exit_hw) {
		ts->exit_hw(pdata);
	}

	input_unregister_device(ts->input_dev);

	kfree(ts);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
    struct touch_platform_data *pdata;
	int ret;
    
	ts = container_of(h, struct goodix_ts_data, early_suspend);
    pdata = ts->client->dev.platform_data;

	if (ts->vlog) {
		printk("%s\n", __func__);
	}

	disable_irq(ts->irq);

	if (ts->suspend_hw){
		ret = ts->suspend_hw(pdata);
		if (ret < 0) {
			dev_err(&ts->client->dev, "%s suspend power failed\n", __func__);
		}
	}
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	struct touch_platform_data *pdata;
    int ret = 0;
    
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	pdata = ts->client->dev.platform_data;

	if (ts->vlog) {
		printk("%s\n", __func__);
	}

    if (ts->resume_hw) {
		ret = ts->resume_hw(pdata);
		if (ret < 0){
			dev_err(&ts->client->dev, "%s power on failed\n", __func__);
		}
	}

    msleep(100);

	enable_irq(ts->irq);
}
#endif


//******************************Begin of firmware update surpport*******************************
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
/**
@brief CRC cal proc,include : Reflect,init_crc32_table,GenerateCRC32
@param global var oldcrc32
@return states
*/
static unsigned int Reflect(unsigned long int ref, char ch)
{
	unsigned int value=0;
	int i;
	for(i = 1; i < (ch + 1); i++)
	{
		if(ref & 1)
			value |= 1 << (ch - i);
		ref >>= 1;
	}
	return value;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  CRC Check Program INIT								                                           		   */
/*---------------------------------------------------------------------------------------------------------*/
static void init_crc32_table(void)
{
	unsigned int temp;
	unsigned int t1,t2;
	unsigned int flag;
	int i,j;
	for(i = 0; i <= 0xFF; i++)
	{
		temp=Reflect(i, 8);
		crc32_table[i]= temp<< 24;
		for (j = 0; j < 8; j++)
		{

			flag=crc32_table[i]&0x80000000;
			t1=(crc32_table[i] << 1);
			if(flag==0)
				t2=0;
			else
				t2=ulPolynomial;
			crc32_table[i] =t1^t2 ;

		}
		crc32_table[i] = Reflect(crc32_table[i], 32);
	}
}
/*---------------------------------------------------------------------------------------------------------*/
/*  CRC main Program									                                           		   */
/*---------------------------------------------------------------------------------------------------------*/
static void GenerateCRC32(unsigned char * buf, unsigned int len)
{
	unsigned int i;
	unsigned int t;

	for (i = 0; i != len; ++i)
	{
		t = (oldcrc32 ^ buf[i]) & 0xFF;
		oldcrc32 = ((oldcrc32 >> 8) & 0xFFFFFF) ^ crc32_table[t];
	}
}

static struct file * update_file_open(char * path, mm_segment_t * old_fs_p)
{
	struct file * filp = NULL;
	int errno = -1;

	filp = filp_open(path, O_RDONLY, 0644);

	if(!filp || IS_ERR(filp))
	{
		if(!filp)
			errno = -ENOENT;
		else 
			errno = PTR_ERR(filp);
		printk(KERN_ERR "The update file for Guitar open error.\n");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp,0,0);
	return filp ;
}

static void update_file_close(struct file * filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if(filp)
		filp_close(filp, NULL);
}
static int update_get_flen(char * path)
{
	struct file * file_ck = NULL;
	mm_segment_t old_fs;
	int length ;

	file_ck = update_file_open(path, &old_fs);
	if(file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	//printk("File length: %d\n", length);
	if(length < 0)
		length = 0;
	update_file_close(file_ck, old_fs);
	return length;
}
static int update_file_check(char * path)
{
	unsigned char buffer[64] = { 0 } ;
	struct file * file_ck = NULL;
	mm_segment_t old_fs;
	int count, ret, length ;

	file_ck = update_file_open(path, &old_fs);

	if(path != NULL)
		printk("File Path:%s\n", path);

	if(file_ck == NULL)
		return -ERROR_NO_FILE;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
#ifdef GUITAR_MESSAGE
	printk(KERN_INFO "gt801 update: File length: %d\n",length);
#endif
	if(length <= 0 || (length%4) != 0)
	{
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_TYPE;
	}

	//set file point to the begining of the file
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);
	oldcrc32 = 0xFFFFFFFF;
	init_crc32_table();
	while(length > 0)
	{
		ret = file_ck->f_op->read(file_ck, buffer, sizeof(buffer), &file_ck->f_pos);
		if(ret > 0)
		{
			for(count = 0; count < ret;  count++)
				GenerateCRC32(&buffer[count],1);
		}
		else 
		{
			update_file_close(file_ck, old_fs);
			return -ERROR_FILE_READ;
		}
		length -= ret;
	}
	oldcrc32 = ~oldcrc32;
#ifdef GUITAR_MESSAGE
	printk("CRC_Check: %u\n", oldcrc32);
#endif
	update_file_close(file_ck, old_fs);
	return 1;
}

unsigned char wait_slave_ready(struct goodix_ts_data *ts, unsigned short *timeout)
{
	unsigned char i2c_state_buf[2] = {ADDR_STA, UNKNOWN_ERROR};
	int ret;
	while(*timeout < MAX_TIMEOUT)
	{
		ret = goodix_read_bytes(ts->client, i2c_state_buf, 2);
		if(ret <= 0)
			return ERROR_I2C_TRANSFER;
		if(i2c_state_buf[1] & SLAVE_READY)
		{
			return i2c_state_buf[1];
			//return 1;
		}
		msleep(10);
		*timeout += 5;
	}
	return 0;
}

static int goodix_update_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	unsigned char cmd[120];
	int ret = -1;
	int start_addr = 0;	

	static unsigned char update_path[60];
	static unsigned short time_count = 0;
	static unsigned int file_len = 0;
	
	unsigned char i2c_control_buf[2] = {ADDR_CMD, 0};
	unsigned char i2c_states_buf[2] = {ADDR_STA, 0};
	unsigned char i2c_data_buf[PACK_SIZE+1+8] = {ADDR_DAT,};
	unsigned char retries = 0;
	unsigned int rd_len;
	unsigned char i = 0;

#ifdef UPDATE_NEW_PROTOCOL
	unsigned char checksum_error_times = 0;
	unsigned int frame_checksum = 0;
	unsigned int frame_number = 0;
#else
	unsigned char send_crc = 0;
#endif

	struct file * file_data = NULL;
	mm_segment_t old_fs;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;

	if(copy_from_user(&cmd, buff, len))
	{
		return -EFAULT;
	}
	switch(cmd[0])
	{
		case STEP_SET_PATH:
			printk(KERN_INFO"Write cmd is:%d,cmd arg is:%s,write len is:%ld\n",cmd[0], &cmd[1], len);
			memset(update_path, 0, 60);
			strncpy(update_path, cmd+1, 60);
			if(update_path[0] == 0)
				return 0;
			else
				return 1;
		case STEP_CHECK_FILE:
			printk(KERN_INFO"Begin to firmware update ......\n");
			ret = update_file_check(update_path);
			if(ret <= 0)
			{
				printk(KERN_INFO"fialed to check update file!\n");
				return ret;
			}
			msleep(500);
			printk(KERN_INFO"Update check file success!\n");
			return 1;
		case STEP_WRITE_SYN:
			printk(KERN_INFO"STEP1:Write synchronization signal!\n");
			i2c_control_buf[1] = UPDATE_START;
			ret = goodix_write_bytes(ts->client, i2c_control_buf, 2);
			if(ret <= 0)
			{
				ret = ERROR_I2C_TRANSFER;
				return ret;
			}
			//the time include time(APROM -> LDROM) and time(LDROM init)
			msleep(1000);
			return 1;
		case STEP_WAIT_SYN:
			printk(KERN_INFO"STEP2:Wait synchronization signal!\n");
			while(retries < MAX_I2C_RETRIES)
			{
				i2c_states_buf[1] = UNKNOWN_ERROR;
				ret = goodix_read_bytes(ts->client, i2c_states_buf, 2);
				printk(KERN_INFO"The read byte is:%d\n", i2c_states_buf[1]);
				if(i2c_states_buf[1] & UPDATE_START)
					break;
				msleep(5);
				retries++;
				time_count += 10;
			}
			if((retries >= MAX_I2C_RETRIES) && (!(i2c_states_buf[1] & UPDATE_START)))
			{
				if(ret <= 0)
					return 0;
				else
					return -1;
			}
			return 1;
		case STEP_WRITE_LENGTH:
			printk(KERN_INFO"STEP3:Write total update file length!\n");
			file_len = update_get_flen(update_path);
			if(file_len <= 0)
			{
				printk(KERN_INFO"get update file length failed!\n");
				return -1;
			}
			file_len += 4;
			i2c_data_buf[1] = (file_len>>24) & 0xff;
			i2c_data_buf[2] = (file_len>>16) & 0xff;
			i2c_data_buf[3] = (file_len>>8) & 0xff;
			i2c_data_buf[4] = file_len & 0xff;
			file_len -= 4;
			ret = goodix_write_bytes(ts->client, i2c_data_buf, 5);
			if(ret <= 0)
			{
				ret = ERROR_I2C_TRANSFER;
				return 0;
			}
			return 1;
		case STEP_WAIT_READY:
			printk(KERN_INFO"STEP4:Wait slave ready!\n");
			ret = wait_slave_ready(ts, &time_count);
			if(ret == ERROR_I2C_TRANSFER)
				return 0;
			if(!ret)
			{
				return -1;
			}
			printk(KERN_INFO"Slave ready!\n");
			return 1;
		case STEP_WRITE_DATA:
#ifdef UPDATE_NEW_PROTOCOL
			printk(KERN_INFO"STEP5:Begin to send file data use NEW protocol!\n");
			file_data = update_file_open(update_path, &old_fs);
			if(file_data == NULL)	//file_data has been opened at the last time
			{
				return -1;
			}
			frame_number = 0;
			while((file_len >= 0) /*&& (!send_crc)*/)
			{
				i2c_data_buf[0] = ADDR_DAT;
				rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
				frame_checksum = 0;
				if(file_len)
				{
					ret = file_data->f_op->read(file_data, i2c_data_buf+1+4, rd_len, &file_data->f_pos);
					if(ret <= 0)
					{
						printk("[GOODiX_ISP_NEW]:Read File Data Failed!\n");
						return -1;
					}
					i2c_data_buf[1] = (frame_number>>24)&0xff;
					i2c_data_buf[2] = (frame_number>>16)&0xff;
					i2c_data_buf[3] = (frame_number>>8)&0xff;
					i2c_data_buf[4] = frame_number&0xff;
					frame_number++;
					frame_checksum = 0;
					for(i=0; i<rd_len; i++)
					{
						frame_checksum += i2c_data_buf[5+i];
					}
					frame_checksum = 0 - frame_checksum;
					i2c_data_buf[5+rd_len+0] = frame_checksum&0xff;
					i2c_data_buf[5+rd_len+1] = (frame_checksum>>8)&0xff;
					i2c_data_buf[5+rd_len+2] = (frame_checksum>>16)&0xff;
					i2c_data_buf[5+rd_len+3] = (frame_checksum>>24)&0xff;
				}
rewrite:
				printk(KERN_INFO"[GOODiX_ISP_NEW]:%d\n", file_len);
				ret = goodix_write_bytes(ts->client, i2c_data_buf, 1+4+rd_len+4);
				//if(ret <= 0)
				if(ret != 1)
				{
					printk("[GOODiX_ISP_NEW]:Write File Data Failed!Return:%d\n", ret);
					return 0;
				}
				
				//Wait for slave ready signal.and read the checksum
				ret = wait_slave_ready(ts, &time_count);
				if(ret & (CHECKSUM_ERROR))
				{
					printk("[GOODiX_ISP_NEW]:File Data Frame checksum Error!\n");
					checksum_error_times++;
					msleep(20);
					if(checksum_error_times > 20)				//max retry times.
						return 0;
					goto rewrite;
				}
				checksum_error_times = 0;
				if(ret & (FRAME_ERROR))
				{
					printk("[GOODiX_ISP_NEW]:File Data Frame Miss!\n");
					return 0;
				}
				if(ret == ERROR_I2C_TRANSFER)
					return 0;
				if(!ret)
				{
					return -1;
				}
				if(file_len < PACK_SIZE)
				{
					update_file_close(file_data, old_fs);
					break;
				}
				file_len -= rd_len;
			}//end of while((file_len >= 0))
			return 1;
#else
			printk(KERN_INFO"STEP5:Begin to send file data use OLD protocol!\n");
			file_data = update_file_open(update_path, &old_fs);
			if(file_data == NULL)	//file_data has been opened at the last time
			{
				return -1;
			}
			while((file_len >= 0) && (!send_crc))
			{
				printk(KERN_INFO"[GOODiX_ISP_OLD]:%d\n", file_len);
				i2c_data_buf[0] = ADDR_DAT;
				rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
				if(file_len)
				{
					ret = file_data->f_op->read(file_data, i2c_data_buf+1, rd_len, &file_data->f_pos);
					if(ret <= 0)
					{
						return -1;
					}
				}
				if(file_len < PACK_SIZE)
				{
					send_crc = 1;
					update_file_close(file_data, old_fs);
					i2c_data_buf[file_len+1] = oldcrc32&0xff;
					i2c_data_buf[file_len+2] = (oldcrc32>>8)&0xff;
					i2c_data_buf[file_len+3] = (oldcrc32>>16)&0xff;
					i2c_data_buf[file_len+4] = (oldcrc32>>24)&0xff;
					ret = goodix_write_bytes(ts->client, i2c_data_buf, (file_len+1+4));
					//if(ret <= 0)
					if(ret != 1)
					{
						printk("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n", ret);
						return 0;
					}
					break;
				}
				else
				{
					ret = goodix_write_bytes(ts->client, i2c_data_buf, PACK_SIZE+1);
					//if(ret <= 0)
					if(ret != 1)
					{
						printk("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n", ret);
						return 0;
					}
				}
				file_len -= rd_len;

				//Wait for slave ready signal.
				ret = wait_slave_ready(ts, &time_count);
				if(ret == ERROR_I2C_TRANSFER)
					return 0;
				if(!ret)
				{
					return -1;
				}
				//Slave is ready.
			}//end of while((file_len >= 0) && (!send_crc))
			return 1;
#endif
		case STEP_READ_STATUS:
			printk(KERN_INFO"STEP6:Read update status!\n");
			while(time_count < MAX_TIMEOUT)
			{
				ret = goodix_read_bytes(ts->client, i2c_states_buf, 2);
				if(ret <= 0)
				{
					return 0;
				}
				if(i2c_states_buf[1] & SLAVE_READY)
				{
					if(!(i2c_states_buf[1] &0xf0))
					{
						printk(KERN_INFO"The firmware updating succeed!update state:0x%x\n",i2c_states_buf[1]);
						return 1;
					}
					else
					{
						printk(KERN_INFO"The firmware updating failed!update state:0x%x\n",i2c_states_buf[1]);
						return 0;

					}
				}
				msleep(1);
				time_count += 5;
			}
			return -1;
		case FUN_CLR_VAL:								//clear the static val
			time_count = 0;
			file_len = 0;
			return 1;
		case FUN_CMD:							//functional command
			if(cmd[1] == CMD_DISABLE_TP)
			{
				printk(KERN_INFO"Disable TS int!\n");
				if(ts->use_irq)
					disable_irq(ts->irq);
				//ts->isUpdating = 1;
			}
			else if(cmd[1] == CMD_ENABLE_TP)
			{
				printk(KERN_INFO"Enable TS int!\n");
				if(ts->use_irq)
					enable_irq(ts->irq);
					
			}
			else if(cmd[1] == CMD_READ_VER)
			{
				printk(KERN_INFO"Read version!\n");
				ts->read_mode = MODE_RD_VER;
			}
			else if(cmd[1] == CMD_READ_RAW)
			{
				printk(KERN_INFO"Read raw data!\n");
				ts->read_mode = MODE_RD_RAW;
				i2c_control_buf[1] = 201;
				ret = goodix_write_bytes(ts->client, i2c_control_buf, 2);			//read raw data cmd
				if(ret <= 0)
				{
					printk(KERN_INFO"Write read raw data cmd failed!\n");
					return 0;
				}
				msleep(200);
			}
			else if(cmd[1] == CMD_READ_DIF)
			{
				printk(KERN_INFO"Read diff data!\n");
				ts->read_mode = MODE_RD_DIF;
				i2c_control_buf[1] = 202;
				ret = goodix_write_bytes(ts->client, i2c_control_buf, 2);			//read diff data cmd
				if(ret <= 0)
				{
					printk(KERN_INFO"Write read raw data cmd failed!\n");
					return 0;
				}
				msleep(200);
			}
			else if(cmd[1] == CMD_READ_CFG)
			{
				printk(KERN_INFO"Read config info!\n");
				ts->read_mode = MODE_RD_CFG;
			}
			else if(cmd[1] == CMD_SYS_REBOOT)
			{
				printk(KERN_INFO"System reboot!\n");
				sys_sync();
				msleep(200);
				//ts->isUpdating = 0;
				//TODO: Alex, workaround to avoid kernel panic, should call Android standard reboot function to reboot from android.
				//kernel_restart(NULL);
				machine_restart(NULL);
			}
			return 1;
		case FUN_WRITE_CONFIG:
			//printk(KERN_INFO"Write config info!\n");
			//cmd[1] config length, cmd[2]config start addr
			if(cmd[1]> 84 || cmd[2] > (0x65+84) || cmd[2] < 0x65)
			{	//invalid config info
				printk(KERN_INFO "Config data is invalid.\n");
				return 0;
			}

			#define EFFECT_CONFIG_POS 3
			if((cmd[2]-0x65) <= 7)
			{
				start_addr = (cmd[2]-0x65) + EFFECT_CONFIG_POS + 1;
				//restore some default config.
				for(i = 0; i < 6; i++)
				{
					if(start_addr > (EFFECT_CONFIG_POS+6))
						break;

					switch(i)
					{
					case 0:
						//cmd[start_addr] = TOUCH_MAX_HEIGHT>>8;			//malata no need
						break;
					case 1:
						//cmd[start_addr] = TOUCH_MAX_HEIGHT&0xff;			//malata no need 
						break;
					case 2:
						//cmd[start_addr] = TOUCH_MAX_WIDTH>>8;				//malata no need
						break;
					case 3:
						//cmd[start_addr] = TOUCH_MAX_WIDTH&0xff;			//malata no need
						break;
					case 4:
						cmd[start_addr] = (cmd[start_addr]&0xf0) | MAX_FINGER_NUM;
						break;
					case 5:
						cmd[start_addr] = (cmd[start_addr]&0xf0) | INT_TRIGGER;
						break;
					}
					start_addr++;
				}
			}

			printk(KERN_INFO"Write config info!\n");
			for(i=3; i<cmd[1];i++)
			{
				if((i-3)%5 == 0)printk("\n");
				printk("No.%d:0x%x ", i-3, cmd[i]);
			}
			printk("\n");

			ret = goodix_write_bytes(ts->client, cmd+2, cmd[1]); 
			if(ret > 0)
				return 1;
			else
				return -1;
		default:
			return -ENOSYS;
	}
	return 0;
}

static int goodix_update_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int ret = -1;
	struct goodix_ts_data *ts;
	int len = 0;
	char *version_info = NULL;
	unsigned char read_data[1201] = {80, };
	/*if (off > 0)
	{
		*eof = 1;
		return 0;
	}*/
	ts = i2c_get_clientdata(i2c_connect_client);
	if(ts==NULL)
		return 0;

	if(ts->read_mode == MODE_RD_VER)		//read version data
	{
		ret = goodix_read_version(ts, &version_info);
		if(ret <= 0)
		{
			printk(KERN_INFO"Read version data failed!\n");
			vfree(version_info);
			return 0;
		}

		for(len=0;len<100;len++)
		{
			if(*(version_info + len) == '\0')
				break;
		}
		printk(KERN_INFO"Gt801_plus ROM version is:%s,and len is:%d\n", (version_info+1), len);
		strncpy(page, version_info+1, len + 1);
		vfree(version_info);
		*eof = 1;
		return len+1;
	}
	else if((ts->read_mode == MODE_RD_RAW)||(ts->read_mode == MODE_RD_DIF))		//read raw data or diff
	{
		//printk(KERN_INFO"Read raw data\n");
		ret = goodix_read_bytes(ts->client, read_data, 1201);
		if(ret <= 0)
		{
			if(ts->read_mode == 2)
				printk(KERN_INFO"Read raw data failed!\n");
			if(ts->read_mode == 3)
				printk(KERN_INFO"Read diff data failed!\n");
			return 0;
		}
		memcpy(page, read_data+1, 1200);
		*eof = 1;
		*start = NULL;
		return 1200;
	}
	else if(ts->read_mode == MODE_RD_CFG)
	{
		read_data[0] = 0x65;
		ret = goodix_read_bytes(ts->client, read_data, 84);
		if(ret <= 0)
		{
			printk(KERN_INFO"Read config info failed!\n");
			return 0;
		}
		memcpy(page, read_data+1, 84);
		return 84;
	}
	return len;
}

#endif
//******************************End of firmware update surpport*******************************
static const struct i2c_device_id goodix_ts_i2c_id[] = {
	{ "gt801-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, goodix_ts_i2c_id);

static struct i2c_driver goodix_ts_driver = {
	.id_table	= goodix_ts_i2c_id,
	.probe		= goodix_ts_probe,
	.remove		= goodix_ts_remove,
	.shutdown	= goodix_ts_shutdown,
	//.suspend	= goodix_ts_suspend,
	//.resume 	= goodix_ts_resume,
	.driver = {
		.name	= "gt801-ts",
	},
};

static int __devinit goodix_ts_init(void)
{
	return i2c_add_driver(&goodix_ts_driver);
}

static void __exit goodix_ts_exit(void)
{
	i2c_del_driver(&goodix_ts_driver);
}
//module_init(goodix_ts_init);
late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Innocomm Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");
