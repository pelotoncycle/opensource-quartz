/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/nfc/pn544_uni.h>
#include <linux/regulator/consumer.h>

#define MAX_BUFFER_SIZE	512

static struct regulator *nfc3v3;
static struct regulator *nfc1v8;
static struct regulator *nfc2v8fake;

enum pn544_irq {
	PN544_NONE,
	PN544_INT,
};

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		mutex;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	enum pn544_irq read_irq;
};

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	mutex_lock(&pn544_dev->read_mutex);
	pn544_dev->read_irq = PN544_INT;
	mutex_unlock(&pn544_dev->read_mutex);

	/* Wake up waiting readers */
	wake_up_interruptible(&pn544_dev->read_wq);
	return IRQ_HANDLED;
}

static enum pn544_irq pn544_dev_irq_state(struct pn544_dev *pn544_dev)
{
	enum pn544_irq irq;

	mutex_lock(&pn544_dev->read_mutex);
	irq = pn544_dev->read_irq;
	mutex_unlock(&pn544_dev->read_mutex);

	return irq;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	enum pn544_irq irq;
	int ret;

	mutex_lock(&pn544_dev->mutex);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	irq = pn544_dev_irq_state(pn544_dev);
	if (irq == PN544_NONE) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		if (wait_event_interruptible(pn544_dev->read_wq,
					     gpio_get_value(pn544_dev->irq_gpio))) {
			ret = -ERESTARTSYS;
			goto fail;
		}
	}

	/* Read data */
	mutex_lock(&pn544_dev->read_mutex);
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	pn544_dev->read_irq = PN544_NONE;
	mutex_unlock(&pn544_dev->read_mutex);
	mutex_unlock(&pn544_dev->mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn544_dev->mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
static int pn544_dev_ioctl(struct inode *inode, struct file *filp,
                              unsigned int cmd, unsigned long arg)
#else
static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#endif    
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_info("%s power on with firmware\n", __func__);
//			regulator_enable(nfc3v3);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 1);
//			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			pr_info("%s power on\n", __func__);
//			regulator_enable(nfc3v3);
			msleep(10);
//			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			pr_info("%s power off\n", __func__);
//			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(10);
//			regulator_disable(nfc3v3);
			msleep(10);
		} else {
			pr_err("%s bad arg %u\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
	.ioctl  = pn544_dev_ioctl,
#else
  .unlocked_ioctl = pn544_dev_ioctl,
#endif  
};

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev;

	platform_data = client->dev.platform_data;

	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret)
		return  -ENODEV;
	ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
	if (ret)
		goto err_ven;
//	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
//	if (ret)
//		goto err_firm;

	gpio_direction_output(platform_data->ven_gpio, 0);

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	nfc3v3 = regulator_get(&client->dev, "nfc_3v3");
	regulator_enable(nfc3v3);

	nfc1v8 = regulator_get(&client->dev, "nfc_1v8");
	regulator_enable(nfc1v8);

	nfc2v8fake = regulator_get(&client->dev, "nfc_2v8_fake");
	regulator_enable(nfc2v8fake);

	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	pn544_dev->client   = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->mutex);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;
	pn544_dev->read_irq = PN544_NONE;
	ret = request_threaded_irq(client->irq, NULL, pn544_dev_irq_handler,
				 IRQF_TRIGGER_RISING, client->name,
				 pn544_dev);

	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->mutex);
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
//	gpio_free(platform_data->firm_gpio);
err_firm:
	gpio_free(platform_data->ven_gpio);
err_ven:
	gpio_free(platform_data->irq_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	pn544_dev->read_irq = PN544_NONE;
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->mutex);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
//	gpio_free(pn544_dev->firm_gpio);
	regulator_disable(nfc2v8fake);
	regulator_disable(nfc1v8);
	regulator_disable(nfc3v3);
	kfree(pn544_dev);

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

static int nfc_suspend(struct device *dev)
{
    if(regulator_is_enabled(nfc3v3)) {
        regulator_disable(nfc3v3);
    }
    
    if (regulator_is_enabled(nfc1v8))
    {
       regulator_disable(nfc1v8);
    }
    if (regulator_is_enabled(nfc2v8fake))
    {
       regulator_disable(nfc2v8fake);
    }
	
    return 0;
}

static int nfc_resume(struct device *dev)
{
    if(!regulator_is_enabled(nfc3v3)) {
        regulator_enable(nfc3v3);
    }

    if(!regulator_is_enabled(nfc1v8)) {
        regulator_enable(nfc1v8);
    }

    if(!regulator_is_enabled(nfc2v8fake)) {
        regulator_enable(nfc2v8fake);
    }

    return 0;
}

static struct dev_pm_ops nfc_pm = {
	.suspend = nfc_suspend,
	.resume  = nfc_resume,
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn544",
//		.pm = &nfc_pm,	
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
