/*
 *   GPS Power Driver for ublox chip.
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <linux/uaccess.h>
#include <linux/tty.h>
#include <linux/sched.h>
#include <linux/gpio.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

//#undef VERBOSE
#undef DEBUG
#define VERBOSE

/* Debug macros*/
#if defined(DEBUG)		/* limited debug messages */
#define GPS_DBG(fmt, arg...)  \
	printk(KERN_INFO "[GPS] (gpsdrv):"fmt"\n" , ## arg)
#define GPS_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define GPS_DBG(fmt, arg...)  \
	printk(KERN_INFO "[GPS] (gpsdrv):"fmt"\n" , ## arg)
#define GPS_VER(fmt, arg...)  \
	printk(KERN_INFO "[GPS] (gpsdrv):"fmt"\n" , ## arg)
#define GPS_ERR(fmt, arg...)  \
	printk(KERN_ERR "[GPS] (gpsdrv):"fmt"\n" , ## arg)
#else /* Error msgs only */
#define GPS_ERR(fmt, arg...)  \
	printk(KERN_ERR "[GPS] (gpsdrv):"fmt"\n" , ## arg)
#define GPS_VER(fmt, arg...)
#define GPS_DBG(fmt, arg...)
#endif

#define DEVICE_NAME     "ubloxgps"

// #define CONTROL_GPIO 129

#define UBLOXGPS_MAGIC 0x99
#define UBLOXGPS_SET_PWR	_IOW(UBLOXGPS_MAGIC, 0x01, unsigned int)

static struct regulator *vgps;

/* List of error codes returned by the gps driver*/
enum {
	GPS_ERR_FAILURE = -1,	/* check struct */
	GPS_SUCCESS,
	GPS_ERR_CLASS = -15,
	GPS_ERR_CPY_TO_USR,
	GPS_ERR_CPY_FRM_USR,
	GPS_ERR_UNKNOWN,
};

/*********Functions Called from GPS host***************************************/

int ubloxgps_open(struct inode *inod, struct file *file)
{
    GPS_DBG("ubloxgps_open");
    return GPS_SUCCESS;
}

ssize_t ubloxgps_read(struct file *file, char __user *data, size_t size,
		loff_t *offset)
{
    int gpio_get_result = 0;

#if defined(CONTROL_GPIO)
    if (gpio_request(CONTROL_GPIO, "gps_control")) {
        printk(" gpio_request error\n");
        return  -ENODEV;
    }

    gpio_direction_output(CONTROL_GPIO, 0);
    gpio_set_value(CONTROL_GPIO, 1);
    msleep(50);
    gpio_get_result = gpio_get_value(CONTROL_GPIO);
    msleep(10);
    gpio_set_value(CONTROL_GPIO, 0);
    msleep(10);
    gpio_free(CONTROL_GPIO);
#endif

    return sprintf(data, "%d", gpio_get_result);

}

ssize_t ubloxgps_write(struct file *file, const char __user *data,
		size_t size, loff_t *offset)
{
    char kbuf[10];
    int num = 0;

    GPS_DBG("ubloxgps_write");

    num = copy_from_user(kbuf, data, size);

    if ((num == 0) && (size != 0)) {
        switch(simple_strtol(kbuf, NULL, 10))
        {
            case 0:
                GPS_DBG("-- ublox GPS off");
                regulator_disable(vgps);
                break;
            case 1:
                GPS_DBG("-- ublox GPS on");
                regulator_enable(vgps);
                break;
            default:
                GPS_DBG("-- unkonw input  [%c]", kbuf[0]);
                break;
        }
    } else {
        GPS_DBG("-- no input");
    }

    return size;
}

static long ubloxgps_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case UBLOXGPS_SET_PWR:
            if (arg == 0) {
                if(regulator_is_enabled(vgps)) {
                    GPS_DBG("Disable GPS power");
                    regulator_disable(vgps);
                } else {
                    GPS_DBG("GPS power is not enable, so do nothing.");
                }
            } else if (arg == 1) {
                if(regulator_is_enabled(vgps)) {
                    GPS_DBG("GPS power already enable, so do nothing.");
                } else {
                    GPS_DBG("Enable GPS power");
                    regulator_enable(vgps);
                }
            } else {
                GPS_DBG("Un-Identified IOCTL arg [%lu]", arg);
            }
            break;
        default:
            GPS_DBG("Un-Identified IOCTL cmd [%u]", cmd);
            break;
    }

    return GPS_SUCCESS;
}

const struct file_operations ubloxgps_chrdev_ops = {
	.owner = THIS_MODULE,
	.open = ubloxgps_open,
	.read = ubloxgps_read,
	.write = ubloxgps_write,
	.unlocked_ioctl = ubloxgps_ioctl,
};

/*********Functions called during insmod and delmod****************************/

static int ubloxgps_major;		/* GPS major number */
static struct class *ubloxgps_class;	/* GPS class during class_create */
static struct device *ubloxgps_dev;	/* GPS dev during device_create */

static int __init ubloxgps_init(void)
{
    GPS_DBG(" Inside %s", __func__);

    vgps = regulator_get(ubloxgps_dev, "vgps");
    if (IS_ERR(vgps)) {
        vgps = NULL;
        GPS_ERR("GPS regulator_get fail");
        return GPS_ERR_FAILURE;
    }

    /* Expose the device DEVICE_NAME to user space
     * And obtain the major number for the device
    */
    ubloxgps_major = register_chrdev(0, DEVICE_NAME, &ubloxgps_chrdev_ops);
    if (0 > ubloxgps_major) {
        GPS_ERR("Error when registering to char dev");
        return GPS_ERR_FAILURE;
    }
    GPS_VER("allocated %d, %d", ubloxgps_major, 0);

    /*  udev */
    ubloxgps_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(ubloxgps_class)) {
        GPS_ERR(" Something went wrong in class_create");
        unregister_chrdev(ubloxgps_major, DEVICE_NAME);
        return GPS_ERR_CLASS;
    }

    ubloxgps_dev = device_create(ubloxgps_class, NULL, MKDEV(ubloxgps_major, 0),NULL, DEVICE_NAME);
    if (IS_ERR(ubloxgps_dev)) {
        GPS_ERR(" Error in class_create");
        unregister_chrdev(ubloxgps_major, DEVICE_NAME);
        class_destroy(ubloxgps_class);
        return GPS_ERR_CLASS;
    }

    return GPS_SUCCESS;
}

static void __exit ubloxgps_exit(void)
{
    GPS_DBG(" Inside %s", __func__);
    GPS_VER(" Bye.. freeing up %d", ubloxgps_major);

    device_destroy(ubloxgps_class, MKDEV(ubloxgps_major, 0));
    class_destroy(ubloxgps_class);
    unregister_chrdev(ubloxgps_major, DEVICE_NAME);
}

module_init(ubloxgps_init);
module_exit(ubloxgps_exit);
MODULE_LICENSE("GPL");
