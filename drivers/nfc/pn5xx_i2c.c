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
 /******************************************************************************
  *
  *  The original Work has been changed by NXP Semiconductors.
  *
  *  Copyright (C) 2015 NXP Semiconductors
  *
  *  Licensed under the Apache License, Version 2.0 (the "License");
  *  you may not use this file except in compliance with the License.
  *  You may obtain a copy of the License at
  *
  *  http://www.apache.org/licenses/LICENSE-2.0
  *
  *  Unless required by applicable law or agreed to in writing, software
  *  distributed under the License is distributed on an "AS IS" BASIS,
  *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  *  See the License for the specific language governing permissions and
  *  limitations under the License.
  *
  ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>

#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include <linux/clk.h>
/*FIXED Screen off transcation proformance issue*/
#include <linux/wakelock.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "pn5xx_i2c.h"

/*#define pr_err NFC_DBG_MSG
*#define pr_debug NFC_DBG_MSG
*#define pr_warning NFC_DBG_MSG
*/
#define DBG_MODULE 0
#if DBG_MODULE
#define NFC_DBG_MSG(fmt, msg...) pr_err(" %s: "fmt, __func__, ##msg)
#else
#define NFC_DBG_MSG(fmt, msg...)
#endif
#define NFC_ERR_MSG(fmt, msg...) pr_err(" %s: "fmt, __func__, ##msg)

#define MAX_BUFFER_SIZE	512

#define SIG_NFC 44
#define PN5XX_DRIVER_NAME         "pn548"

#define NFC_RF_CLK_FREQ			(19200000)

/*#define ENABLE_NFC_DOUBLE_SIM_SWITCH*/
/*#define CONFIG_NFC_ENABLE_BB_CLK2*/

struct pn5xx_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct mutex		write_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn5xx_device;
	unsigned int		ven_gpio;
	unsigned int		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int        ese_pwr_gpio; /* gpio used by SPI to provide power to ese via NFCC */
	unsigned int        swp2_pwr_gpio; /* gpio used by SPI to provide power to ese via NFCC */
	bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
	bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static struct pn5xx_dev    *pn5xx_dev = NULL;
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
static struct clk *bb_clk2;
bool          clk_run = false;
/*static struct wakeup_source *nfc_wake_lock;*/
#endif

/*add nfc id information begin*/

static char nfc_id[51] = "NXP-PN65T-NA-NA-NA";
static int nfc_id_read_proc(struct seq_file *m, void *v)
{
	return seq_printf(m, "%s\n", nfc_id);
}

static int nfc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_id_read_proc, NULL);
}

static const struct file_operations nfc_proc_fops = {
	.open		= nfc_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct proc_dir_entry *nfc_id_proc_file;

static void create_nfc_info_proc_file(void)
{
	nfc_id_proc_file = proc_create("driver/nfc_id", S_IWUGO | S_IRUGO, NULL, &nfc_proc_fops);
	NFC_DBG_MSG("goes to create_nfc_info_proc_file\n");
	if (nfc_id_proc_file) {
	} else{
		NFC_DBG_MSG("proc file create failed!\n");
	}
}

static void remove_nfc_info_proc_file(void)
{
	NFC_DBG_MSG("goes to remove_nfc_info_proc_file\n");
	if (nfc_id_proc_file) {
		remove_proc_entry("driver/nfc_id", NULL);
		nfc_id_proc_file = NULL;
	}
}
/*add nfc id information end*/

static void pn5xx_disable_irq(struct pn5xx_dev *pn5xx_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn5xx_dev->irq_enabled_lock, flags);
	if (pn5xx_dev->irq_enabled) {
		disable_irq_nosync(pn5xx_dev->client->irq);
		pn5xx_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn5xx_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn5xx_dev_irq_handler(int irq, void *dev_id)
{
	struct pn5xx_dev *pn5xx_dev = dev_id;

	NFC_DBG_MSG("start\n");

	if (!gpio_get_value(pn5xx_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn5xx_disable_irq(pn5xx_dev);

	/* Wake up waiting readers */
	wake_up(&pn5xx_dev->read_wq);
	NFC_DBG_MSG("end\n");
	return IRQ_HANDLED;
}

static ssize_t pn5xx_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn5xx_dev *pn5xx_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	NFC_DBG_MSG("reading %zu bytes.\n", count);
	/*__pm_wakeup_event(nfc_wake_lock, 5 * MSEC_PER_SEC);*/
	mutex_lock(&pn5xx_dev->read_mutex);

	if (!gpio_get_value(pn5xx_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn5xx_dev->irq_enabled = true;
		enable_irq(pn5xx_dev->client->irq);
		NFC_DBG_MSG("wait_event_interruptible start\n");
		ret = wait_event_interruptible(pn5xx_dev->read_wq,
				gpio_get_value(pn5xx_dev->irq_gpio));

		pn5xx_disable_irq(pn5xx_dev);
		NFC_DBG_MSG("wait_event_interruptible end\n");
		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn5xx_dev->client, tmp, count);
	NFC_DBG_MSG("i2c_master_recv end\n");
	mutex_unlock(&pn5xx_dev->read_mutex);

    /* pn5xx seems to be slow in handling I2C read requests
     * so add 1ms delay after recv operation
	 */
	udelay(1000);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warn("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	/*printk("NFCC->DH:");
	*for(i = 0; i < ret; i++){
	*	printk(" %02X", tmp[i]);
	*}
	*printk("\n");
	*/

	return ret;

fail:
	mutex_unlock(&pn5xx_dev->read_mutex);
	return ret;
}

static ssize_t pn5xx_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn5xx_dev  *pn5xx_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn5xx_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	/*printk("%s : writing %zu bytes.\n", __func__, count);
	*printk("DH->NFCC:");
	*for(i = 0; i < count; i++){
	*	printk(" %02X", tmp[i]);
	*}
	*printk("\n");
	*/
	/* Write data */
	ret = i2c_master_send(pn5xx_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	/* pn5xx seems to be slow in handling I2C write requests
    * so add 1ms delay after I2C send oparation
	*/
	udelay(1000);
	return ret;
}

static int pn5xx_dev_open(struct inode *inode, struct file *filp)
{
	struct pn5xx_dev *pn5xx_dev = container_of(filp->private_data,
						struct pn5xx_dev,
						pn5xx_device);

	filp->private_data = pn5xx_dev;
	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn5xx_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn5xx_dev *pn5xx_dev = filp->private_data;
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
	int ret = 0;
#endif
	switch (cmd) {
	case PN5XX_SET_PWR:
	{
		if (arg == 2) {
		    pr_info("%s PN5xx_SET_PWR\n", __func__);
		    /* power on with firmware download (requires hw reset)
		    */
		    NFC_DBG_MSG("power on with firmware\n");
			gpio_set_value(pn5xx_dev->ven_gpio, 1);
			gpio_set_value(pn5xx_dev->firm_gpio, 1);
		    msleep(20);
		    gpio_set_value(pn5xx_dev->ven_gpio, 0);
		    msleep(50);
		    gpio_set_value(pn5xx_dev->ven_gpio, 1);
		    msleep(20);
		} else if (arg == 1) {
			/* power on */
			NFC_DBG_MSG("power on\n");
			gpio_set_value(pn5xx_dev->firm_gpio, 0);
			gpio_set_value(pn5xx_dev->ven_gpio, 1);
			msleep(20);
			irq_set_irq_wake(pn5xx_dev->client->irq, 1);
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
			if (clk_run == false) {
			   msleep(20);
			   ret = clk_prepare_enable(bb_clk2);
		       if (ret) {
					pr_err("%s: prepare bb_clk2 failed ret:%d\n", __func__, ret);
			   }
			   clk_run = true;
			   pr_err("%s BBCLK2 ON\n", __func__);
			}
#endif
		} else  if (arg == 0) {
			/* power off */
			NFC_DBG_MSG("power off\n");
			gpio_set_value(pn5xx_dev->firm_gpio, 0);
			gpio_set_value(pn5xx_dev->ven_gpio, 0);

			msleep(20);
			irq_set_irq_wake(pn5xx_dev->client->irq, 0);
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
			if (clk_run == true) {
				msleep(20);
				clk_disable_unprepare(bb_clk2);
				clk_run = false;
				pr_err("%s BBCLK2 OFF\n", __func__);
			}
#endif
		} else {
			/*printk("%s bad arg %x\n", __func__, arg);*/
		    NFC_DBG_MSG("unlock sigal\n");
			return -EINVAL;
		}
	}
	break;
	default:
		NFC_ERR_MSG("bad ioctl %x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static int nxp_pn5xx_reset(void)
{
	int rc;
	/*rc = gpio_tlmm_config(GPIO_CFG(pn5xx_dev->irq_gpio, 0,
	*			GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
	*			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	*/
	if (!gpio_is_valid(pn5xx_dev->irq_gpio)) {
		NFC_ERR_MSG("Could not configure nfc gpio%d\n", pn5xx_dev->irq_gpio);
		return -EIO;
	}
	rc = gpio_request(pn5xx_dev->irq_gpio, "nxp_pn5xx_IRQ");
	if (rc) {
		NFC_ERR_MSG("unable to request nfc gpio%d(%d)\n", pn5xx_dev->irq_gpio, rc);
		return -EIO;
	}
	rc = gpio_direction_input(pn5xx_dev->irq_gpio);
	if (rc) {
		NFC_ERR_MSG("unable to set input nfc gpio%d(%d)\n", pn5xx_dev->irq_gpio, rc);
		return -EIO;
	}
	/*rc = gpio_tlmm_config(GPIO_CFG(pn5xx_dev->firm_gpio, 0,
	*				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
	*				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	*/
	/*printk("pn5xx config firmgpio pull down\n");*/
	if (!gpio_is_valid(pn5xx_dev->firm_gpio)) {
		NFC_ERR_MSG("Could not configure nfc gpio%d\n", pn5xx_dev->firm_gpio);
		return -EIO;
	}
	rc = gpio_request(pn5xx_dev->firm_gpio, "nxp_pn5xx_download");
	if (rc) {
		NFC_ERR_MSG("unable to request nfc gpio%d(%d)\n", pn5xx_dev->firm_gpio, rc);
		return -EIO;
	}
	rc = gpio_direction_output(pn5xx_dev->firm_gpio, 0);
	if (rc) {
		NFC_ERR_MSG("unable to set output nfc gpio%d(%d)\n", pn5xx_dev->firm_gpio, rc);
		return -EIO;
	}
	/*ven gpio out*/
	/*rc = gpio_tlmm_config(GPIO_CFG(pn5xx_dev->ven_gpio, 0,
	*				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	*				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	*/
	/*printk("pn5xx config vengpio out put no pull\n");*/
	if (!gpio_is_valid(pn5xx_dev->ven_gpio)) {
		NFC_ERR_MSG("Could not configure nfc gpio%d\n", pn5xx_dev->ven_gpio);
		return -EIO;
	}

	rc = gpio_request(pn5xx_dev->ven_gpio, "nxp_pn5xx_en");
	if (rc) {
		NFC_ERR_MSG("unable to request nfc gpio%d(%d)\n", pn5xx_dev->ven_gpio, rc);
		return -EIO;
	}
	/*disable NFC first,Ven is inversed*/
	rc = gpio_direction_output(pn5xx_dev->ven_gpio, 0);
	if (rc) {
		NFC_ERR_MSG("unable to set output nfc gpio%d(%d)\n", pn5xx_dev->ven_gpio, rc);
		return -EIO;
	}

	if (!gpio_is_valid(pn5xx_dev->ese_pwr_gpio)) {
		NFC_ERR_MSG("Could not configure nfc gpio%d\n", pn5xx_dev->ese_pwr_gpio);
		return -EIO;
	}

	rc = gpio_request(pn5xx_dev->ese_pwr_gpio, "nxp_pn5xx_ese");
	if (rc) {
		NFC_ERR_MSG("unable to request nfc gpio%d(%d)\n", pn5xx_dev->ese_pwr_gpio, rc);
		return -EIO;
	}
	/*disable NFC first,Ven is inversed*/
	rc = gpio_direction_output(pn5xx_dev->ese_pwr_gpio, 0);
	if (rc) {
		NFC_ERR_MSG("unable to set output nfc gpio%d(%d)\n", pn5xx_dev->ese_pwr_gpio, rc);
		return -EIO;
	}
#ifdef ENABLE_NFC_DOUBLE_SIM_SWITCH
    /*uicc2 power contrl*/
	if (!gpio_is_valid(pn5xx_dev->swp2_pwr_gpio)) {
		NFC_ERR_MSG("Could not configure nfc gpio%d\n", pn5xx_dev->swp2_pwr_gpio);
		return -EIO;
	}
	rc = gpio_request(pn5xx_dev->swp2_pwr_gpio, "nxp_pn5xx_swp2_pwr");
	if (rc) {
		NFC_ERR_MSG("unable to request nfc gpio%d(%d)\n", pn5xx_dev->swp2_pwr_gpio, rc);
		return -EIO;
	}
	rc = gpio_direction_output(pn5xx_dev->swp2_pwr_gpio, 1);
	if (rc) {
		NFC_ERR_MSG("unable to set input nfc gpio%d (%d)\n", pn5xx_dev->swp2_pwr_gpio, rc);
		return -EIO;
	}
	/*set uicc2 pwr*/
	gpio_set_value(pn5xx_dev->swp2_pwr_gpio, 1);
#endif
	return 0;
}

static const struct file_operations pn5xx_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn5xx_dev_read,
	.write	= pn5xx_dev_write,
	.open	= pn5xx_dev_open,
	.unlocked_ioctl  = pn5xx_dev_ioctl,
	.compat_ioctl  = pn5xx_dev_ioctl,
};

static int pn5xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct device_node *of_node = NULL;

	NFC_DBG_MSG("start\n");

	if (pn5xx_dev != NULL) {
		NFC_ERR_MSG("multiple devices NOT supported\n");
		ret = -ENODEV;
		goto err_single_device;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pn5xx_dev = kzalloc(sizeof(*pn5xx_dev), GFP_KERNEL);
	if (pn5xx_dev == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}

	pn5xx_dev->client = client;
	if (client->dev.of_node) {
		of_node = client->dev.of_node;
		ret = of_get_named_gpio(of_node, "nxp,pn5xx-irq", 0);
		if (ret > 0) {
			pn5xx_dev->irq_gpio = ret;
		} else{
			NFC_ERR_MSG(" of_property_read(irq_gpio) fail:%d\n", ret);
			goto err_device_create_failed;
		}

		ret = of_get_named_gpio(of_node, "nxp,pn5xx-fw-dwnld", 0);
		if (ret > 0) {
			pn5xx_dev->firm_gpio = ret;
		} else{
			NFC_ERR_MSG("of_property_read(firm_gpio) fail:%d\n", ret);
			goto err_device_create_failed;
		}

		ret = of_get_named_gpio(of_node, "nxp,pn5xx-ven", 0);
		if (ret > 0) {
			pn5xx_dev->ven_gpio = ret;
		} else{
			NFC_ERR_MSG("of_property_read(ven_gpio) fail:%d\n", ret);
			goto err_device_create_failed;
		}

		ret = of_get_named_gpio(of_node, "nxp,pn5xx-ese-pwr", 0);
		if (ret > 0) {
			pn5xx_dev->ese_pwr_gpio = ret;
		} else{
			NFC_ERR_MSG("of_property_read(ese_pwr_gpio) fail:%d\n", ret);
			goto err_device_create_failed;
		}
#ifdef ENABLE_NFC_DOUBLE_SIM_SWITCH
		ret = of_get_named_gpio(of_node, "nxp,pn5xx-swp2-pwr", 0);
		if (ret > 0) {
			pn5xx_dev->swp2_pwr_gpio = ret;
		} else{
			NFC_ERR_MSG("of_property_read(swp2_pwr_gpio) fail:%d\n", ret);
			goto err_device_create_failed;
		}
#endif
		/* init mutex and queues */
		init_waitqueue_head(&pn5xx_dev->read_wq);
		mutex_init(&pn5xx_dev->read_mutex);
		mutex_init(&pn5xx_dev->write_mutex);
		spin_lock_init(&pn5xx_dev->irq_enabled_lock);

		/*nfc_wake_lock = wakeup_source_register("nfctimer");*/
		pn5xx_dev->pn5xx_device.minor = MISC_DYNAMIC_MINOR;
		pn5xx_dev->pn5xx_device.name = "pn548";
		pn5xx_dev->pn5xx_device.fops = &pn5xx_dev_fops;

		ret = misc_register(&pn5xx_dev->pn5xx_device);
		if (ret) {
			pr_err("%s : misc_register failed\n", __FILE__);
			goto err_misc_register;
		}

		ret = nxp_pn5xx_reset();
		NFC_DBG_MSG("pn5xx reset\n");
		if (ret < 0) {
			NFC_ERR_MSG("can't reset device\n");
			goto err_device_create_file_failed;
		}
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
		/*bb_clk2 = clk_get(&client->dev, "bb_clk2");*/
		bb_clk2 = devm_clk_get(&client->dev, "bb_clk2");
		if (IS_ERR(bb_clk2)) {
			NFC_DBG_MSG("Error getting bb_clk2\n");
			bb_clk2 = NULL;
			/*return -ENOENT;*/
		} else{
			NFC_DBG_MSG("start prepare bb_clk2\n");
			/*ret = clk_set_rate(bb_clk2,NFC_RF_CLK_FREQ);
			*if(ret) {
			*	NFC_DBG_MSG("set bb_clk2 rate failed ret:%d\n", ret);
			*}
			//ret = clk_prepare_enable(bb_clk2);
			*if(ret) {
			*	NFC_DBG_MSG("prepare bb_clk2 failed ret:%d\n", ret);
			}
			*/
		}
#endif
		/*pr_info("%s : requesting IRQ %d\n", __func__, client->irq);*/
		pn5xx_dev->irq_enabled = true;

		ret = request_irq(client->irq, pn5xx_dev_irq_handler,
				  IRQF_TRIGGER_HIGH, client->name, pn5xx_dev);
		if (ret) {
			NFC_ERR_MSG("request_irq failed\n");
			goto err_request_irq_failed;
		} else {
			/* test to see if irq is really wakeup capable */
			ret = irq_set_irq_wake(client->irq, 1);
			if (ret) {
				NFC_ERR_MSG("irq %d cannot set wakeup (%d)\n", client->irq, ret);
				ret = 0;
			} else {
				irq_set_irq_wake(client->irq, 0);
			}
			NFC_DBG_MSG("request_irq ok\n");
		}

		pn5xx_disable_irq(pn5xx_dev);
		i2c_set_clientdata(client, pn5xx_dev);
		create_nfc_info_proc_file();
		NFC_DBG_MSG("nfc probe is ok\n");

		return 0;
	}
err_request_irq_failed:
	misc_deregister(&pn5xx_dev->pn5xx_device);
err_misc_register:
	mutex_destroy(&pn5xx_dev->read_mutex);
	mutex_destroy(&pn5xx_dev->write_mutex);
	kfree(pn5xx_dev);
err_device_create_failed:
	kfree(pn5xx_dev);
	pn5xx_dev = NULL;
err_device_create_file_failed:
    /*device_destroy(pn5xx_dev_class, MKDEV(pn5xx_major, pn5xx_minor));*/
err_exit:
	/*gpio_free(platform_data->firm_gpio);*/
err_single_device:
	return ret;

}

static int pn5xx_remove(struct i2c_client *client)
{
	/*struct pn5xx_dev *pn5xx_dev;*/
	NFC_DBG_MSG("start\n");
#ifdef CONFIG_NFC_ENABLE_BB_CLK2
	if (clk_run == true) {
	   clk_disable_unprepare(bb_clk2);
	   clk_run = false;
	}
#endif
	pn5xx_dev = i2c_get_clientdata(client);
#ifdef ENABLE_NFC_DOUBLE_SIM_SWITCH
	if (gpio_is_valid(pn5xx_dev->swp2_pwr_gpio)) {
		gpio_set_value(pn5xx_dev->swp2_pwr_gpio, 0);
	}
#endif
	pn5xx_disable_irq(pn5xx_dev);
	free_irq(client->irq, pn5xx_dev);
	misc_deregister(&pn5xx_dev->pn5xx_device);
	mutex_destroy(&pn5xx_dev->read_mutex);
	mutex_destroy(&pn5xx_dev->write_mutex);
	gpio_free(pn5xx_dev->irq_gpio);
	gpio_free(pn5xx_dev->ven_gpio);
	gpio_free(pn5xx_dev->ese_pwr_gpio);
#ifdef ENABLE_NFC_DOUBLE_SIM_SWITCH
	gpio_free(pn5xx_dev->swp2_pwr_gpio);
#endif
	gpio_free(pn5xx_dev->firm_gpio);
	kfree(pn5xx_dev);
	pn5xx_dev = NULL;

	remove_nfc_info_proc_file();

	return 0;
}

static const struct i2c_device_id pn5xx_id[] = {
	{PN5XX_DRIVER_NAME, 0 },
	{ }
};

static const struct of_device_id nfc_match_table[] = {
	{.compatible = "nxp,pn544",},
	{.compatible = "nxp,pn547",},
	{.compatible = "nxp,pn548",},
	{ },
};

static struct i2c_driver pn5xx_driver = {
	.id_table	= pn5xx_id,
	.probe		= pn5xx_probe,
	.remove		= pn5xx_remove,
	.driver		= {
		.name   = "pn548",
		.owner = THIS_MODULE,
		.of_match_table = nfc_match_table,
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn5xx_dev_init(void)
{
	pr_info("Loading pn5xx driver\n");
	return i2c_add_driver(&pn5xx_driver);
}
module_init(pn5xx_dev_init);

static void __exit pn5xx_dev_exit(void)
{
	pr_info("Unloading pn5xx driver\n");
	i2c_del_driver(&pn5xx_driver);
}
module_exit(pn5xx_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN548 driver");
MODULE_LICENSE("GPL");
