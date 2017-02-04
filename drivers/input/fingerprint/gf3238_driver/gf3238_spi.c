/*Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *     Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/wakelock.h>/* for wakelock hold */
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "gf3238_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define GF_SPIDEV_NAME     "goodix,fingerprint"
/*device name after register in character*/
#define GF_DEV_NAME            "goodix_fp"
#define GF_INPUT_NAME       "qwerty"    /*"goodix_fp" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"
#define SPIDEV_MAJOR		154	/* assigned */
#define N_SPI_MINORS		32	/* ... up to 256 */

struct wake_lock gf_milan_wake_lock;
int gf_log_mask = GF_LOG_INFO | GF_LOG_ERR;
module_param_named(gf_log_mask, gf_log_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define GF_INPUT_HOME_KEY KEY_HOMEPAGE /* KEY_HOME */
#define GF_INPUT_MENU_KEY  KEY_MENU
#define GF_INPUT_BACK_KEY  KEY_BACK
#define GF_INPUT_FF_KEY  KEY_POWER
#define GF_INPUT_CAMERA_KEY  KEY_CAMERA
#define GF_INPUT_OTHER_KEY KEY_VOLUMEDOWN  /* temporary key value for capture use */
#define GF_NAV_UP_KEY  KEY_UP
#define GF_NAV_DOWN_KEY  KEY_DOWN
#define GF_NAV_LEFT_KEY  KEY_LEFT
#define GF_NAV_RIGHT_KEY  KEY_RIGHT

#define GF_CLICK_KEY  114
#define GF_DOUBLE_CLICK_KEY  115
#define GF_LONG_PRESS_KEY  217

struct gf_key_map milan_key_map[] = { { "POWER", KEY_POWER }, { "HOME", KEY_HOME }, {
		"MENU", KEY_MENU
	}, { "BACK", KEY_BACK }, { "UP", KEY_UP }, { "DOWN",
		KEY_DOWN
	}, { "LEFT", KEY_LEFT }, { "RIGHT", KEY_RIGHT }, { "FORCE",
		KEY_F9
	}, { "CLICK", KEY_F19 },
};


/**************************debug******************************/


/*Global variables*/
/*static MODE g_mode = GF_IMAGE_MODE;*/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		FP_LOG(INFO, "IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		FP_LOG(INFO, "IRQ has been disabled.\n");
	}
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate(struct clk *clk,  unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
		|| ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	long rate;
	int rc;

	rate = spi_clk_max_rate(gf_dev->core_clk, speed);
	if (rate < 0) {
		FP_LOG(ERR, "no match found for requested clock frequency:%d", speed);
		return;
	}

	rc = clk_set_rate(gf_dev->core_clk, rate);
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
	FP_LOG(DBG, "enter\n");

	data->clk_enabled = 0;
	data->core_clk = clk_get(&data->spi->dev, "core_clk");
	if (IS_ERR_OR_NULL(data->core_clk)) {
		FP_LOG(ERR, "fail to get core_clk\n");
		return -EPERM;
	}
	data->iface_clk = clk_get(&data->spi->dev, "iface_clk");
	if (IS_ERR_OR_NULL(data->iface_clk)) {
		FP_LOG(ERR, "fail to get iface_clk\n");
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -ENOENT;
	}
	return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
	int err;

	FP_LOG("%s: enter\n", __func__);

	if (data->clk_enabled)
		return 0;

	err = clk_prepare_enable(data->core_clk);
	if (err) {
		FP_LOG(ERR, "fail to enable core_clk\n");
		return -EPERM;
	}

	err = clk_prepare_enable(data->iface_clk);
	if (err) {
		FP_LOG(ERR, "fail to enable iface_clk\n");
		clk_disable_unprepare(data->core_clk);
		return -ENOENT;
	}

	data->clk_enabled = 1;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	FP_LOG("%s: enter\n", __func__);

	if (!data->clk_enabled)
		return 0;

	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
	data->clk_enabled = 0;

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
	FP_LOG("%s: enter\n", __func__);

	if (data->clk_enabled)
		gfspi_ioctl_clk_disable(data);

	if (!IS_ERR_OR_NULL(data->core_clk)) {
		clk_put(data->core_clk);
		data->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(data->iface_clk)) {
		clk_put(data->iface_clk);
		data->iface_clk = NULL;
	}

	return 0;
}
#endif

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key;
	int retval = 0;
	/* int i; */
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;
	uint32_t key_event;
#ifdef AP_CONTROL_CLK
	unsigned int speed = 0;
#endif

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval =
			!access_ok(VERIFY_WRITE, (void __user *)arg,
				_IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval =
			!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER)) {
			FP_LOG(INFO, "power cmd\n");
		} else {
			FP_LOG(INFO, "Sensor is power off currently.\n");
			return -ENODEV;
		}
	}

	switch (cmd) {
	case GF_IOC_INIT:
		pr_info("%s GF_IOC_INIT .\n", __func__);
		if (copy_to_user((void __user *) arg, (void *) &netlink_route,
				 sizeof(u8))) {
			retval = -EFAULT;
			break;
		}
		break;
	case GF_IOC_EXIT:
		pr_info("%s GF_IOC_EXIT .\n", __func__);
		break;
	case GF_IOC_DISABLE_IRQ:
		FP_LOG(INFO, "GF_IOC_DISABEL_IRQ .\n");
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		FP_LOG(INFO, "GF_IOC_ENABLE_IRQ .\n");
		gf_enable_irq(gf_dev);
		break;
#if 0
	case GF_IOC_SETSPEED:
#ifdef AP_CONTROL_CLK
		retval = __get_user(speed, (u32 __user *) arg);
		if (retval == 0) {
			if (speed > 8 * 1000 * 1000) {
				FP_LOG(INFO, "Set speed:%d is larger than 8Mbps.\n", speed);
			} else {
				spi_clock_set(gf_dev, speed);
			}
		} else {
			FP_LOG(INFO, "Failed to get speed from user. retval = %d\n",  retval);
		}
#else
		FP_LOG(INFO, "This kernel doesn't support control clk in AP\n");
#endif
		break;
#endif
	case GF_IOC_RESET:
		FP_LOG(INFO, "GF_IOC_RESET.\n");
		gf_milan_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		FP_LOG(INFO, "GF_IOC_INPUT_KEY_EVENT.\n");
		if (copy_from_user(&gf_key, (struct gf_key *) arg,
				sizeof(struct gf_key))) {
			FP_LOG(INFO, "Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (gf_key.key == GF_KEY_HOME) {
			key_event = GF_INPUT_HOME_KEY;
		} else if (gf_key.key == GF_KEY_POWER) {
			key_event = GF_INPUT_FF_KEY;
		} else if (gf_key.key == GF_KEY_CAPTURE) {
			key_event = GF_INPUT_CAMERA_KEY;
		} else if (gf_key.key == GF_KEY_LONG_PRESS) {
			key_event = GF_LONG_PRESS_KEY;
		} else if (gf_key.key == GF_KEY_DOUBLE_TAP) {
			key_event = GF_DOUBLE_CLICK_KEY;
		} else if (gf_key.key == GF_KEY_TAP) {
			key_event = GF_CLICK_KEY;
		} else {
			key_event = gf_key.key;
		}
		FP_LOG(INFO, "received key event[%d], key=%d, value=%d\n",
			key_event, gf_key.key, gf_key.value);

		if ((gf_key.key == GF_KEY_POWER || gf_key.key == GF_KEY_CAPTURE)
			&& (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_event, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_event, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_UP) {
			input_report_key(gf_dev->input, GF_NAV_UP_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_UP_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_DOWN) {
			input_report_key(gf_dev->input, GF_NAV_DOWN_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_DOWN_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_RIGHT) {
			input_report_key(gf_dev->input, GF_NAV_RIGHT_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_RIGHT_KEY, 0);
			input_sync(gf_dev->input);
		} else if (gf_key.key == GF_KEY_LEFT) {
			input_report_key(gf_dev->input, GF_NAV_LEFT_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_NAV_LEFT_KEY, 0);
			input_sync(gf_dev->input);
		} else if ((gf_key.key != GF_KEY_POWER)
				&& (gf_key.key != GF_KEY_CAPTURE)) {
			input_report_key(gf_dev->input, key_event, gf_key.value);
			input_sync(gf_dev->input);
		}
		break;
	case GF_IOC_ENABLE_SPI_CLK:
		FP_LOG(INFO, "GF_IOC_ENABLE_SPI_CLK.\n");
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_enable(gf_dev);
#else
		FP_LOG(INFO, "Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_DISABLE_SPI_CLK:
		FP_LOG(INFO, "GF_IOC_DISABLE_SPI_CLK.\n");
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_disable(gf_dev);
#else
		FP_LOG(INFO, "Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_ENABLE_POWER:
		FP_LOG(INFO, "GF_IOC_ENABLE_POWER.\n");
		if (gf_dev->device_available == 1)
			FP_LOG(INFO, "Sensor has already powered-on.\n");
		else
			gf_milan_power_on(gf_dev);
		gf_dev->device_available = 1;
		break;
	case GF_IOC_DISABLE_POWER:
		FP_LOG(INFO, "GF_IOC_DISABLE_POWER.\n");
		if (gf_dev->device_available == 0)
			FP_LOG(INFO, "Sensor has already powered-off.\n");
		else
			gf_milan_power_off(gf_dev);
		gf_dev->device_available = 0;
		break;
	case GF_IOC_ENTER_SLEEP_MODE:
		pr_info("%s GF_IOC_ENTER_SLEEP_MODE.\n", __func__);
		break;
	case GF_IOC_GET_FW_INFO:
		FP_LOG(INFO, "GF_IOC_GET_FW_INFO.\n");
		break;
	case GF_IOC_REMOVE:
		FP_LOG(INFO, "GF_IOC_REMOVE.\n");
		break;
	case GF_IOC_CHIP_INFO:
		FP_LOG(INFO, "GF_IOC_CHIP_INFO.\n");
		if (copy_from_user(&info, (struct gf_ioc_chip_info *) arg,
				sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		FP_LOG(INFO, " vendor_id : 0x%x\n", info.vendor_id);
		FP_LOG(INFO, " mode : 0x%x\n", info.mode);
		FP_LOG(INFO, " operation: 0x%x\n", info.operation);
		break;
	default:
		FP_LOG(DEBUG, "Unsupport cmd:0x%x\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
#if defined(GF_NETLINK_ENABLE)
	char temp = GF_NET_EVENT_IRQ;

	sendnlmsg(&temp);
#elif defined(GF_FASYNC)
	struct gf_dev *gf_dev = &gf;

	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			FP_LOG(DEBUG, "Device Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			FP_LOG(DEBUG, "Succeed to open device. irq = %d\n", gf_dev->irq);
			if (gf_dev->users == 1)
				gf_enable_irq(gf_dev);
			/*power the sensor*/
			gf_milan_power_on(gf_dev);
			gf_milan_hw_reset(gf_dev, 360);
			gf_dev->device_available = 1;
		}
	} else {
		FP_LOG(DEBUG, "No device for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	FP_LOG(DEBUG, "ret = %d\n", ret);
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {
		FP_LOG(DEBUG, "disble_irq. irq = %d\n", gf_dev->irq);
		gf_disable_irq(gf_dev);
		/*power off the sensor*/
		gf_dev->device_available = 0;
		gf_milan_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	char temp = 0;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	FP_LOG(INFO, "go to the goodix_fb_state_chg_callback value = %d\n", (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_BLACK;
				sendnlmsg(&temp);
#elif defined(GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO,
						POLL_IN);
				}
#endif
				/*device unavailable */
				/* gf_dev->device_available = 0; */
			}
			break;
		case FB_BLANK_UNBLANK:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_UNBLACK;
				sendnlmsg(&temp);
#elif defined(GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO,
						POLL_IN);
				}
#endif
				/*device available */
				/* gf_dev->device_available = 1; */
			}
			break;
		default:
			FP_LOG(INFO, "defalut\n");
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
	int i;

	set_bit(EV_KEY, gf_dev->input->evbit); /* tell the kernel is key event */
	for (i = 0; i < ARRAY_SIZE(milan_key_map); i++) {
		set_bit(milan_key_map[i].val, gf_dev->input->keybit);
	}

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input))
		FP_LOG(INFO, "Failed to register GF as input device.\n");
}

extern bool is_goodix_milan_fp(void);
static struct class *gf_class;
#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int ret;

	FP_LOG(INFO, "probe begin\n");
	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
#if defined(USE_SPI_BUS)
	gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
	gf_dev->spi = pdev;
#endif
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

	if (gf_milan_parse_dts(gf_dev))
		goto error;
	/*
	*if (gf_milan_power_on(gf_dev))
	*goto error;
	*gf_dev->device_available = 1;
	*/
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		FP_LOG(INFO, "major:%d minor:%lu\n", SPIDEV_MAJOR, minor);
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		FP_LOG(INFO, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			FP_LOG(INFO, "Faile to allocate input device.\n");
			status = -ENOMEM;
		}
		__set_bit(EV_KEY, gf_dev->input->evbit);
		__set_bit(GF_INPUT_HOME_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_MENU_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_BACK_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_FF_KEY, gf_dev->input->keybit);
		__set_bit(GF_NAV_UP_KEY, gf_dev->input->keybit);
		__set_bit(GF_NAV_DOWN_KEY, gf_dev->input->keybit);
		__set_bit(GF_NAV_RIGHT_KEY, gf_dev->input->keybit);
		__set_bit(GF_NAV_LEFT_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_CAMERA_KEY, gf_dev->input->keybit);
		__set_bit(GF_CLICK_KEY, gf_dev->input->keybit);
		__set_bit(GF_DOUBLE_CLICK_KEY, gf_dev->input->keybit);
		__set_bit(GF_LONG_PRESS_KEY, gf_dev->input->keybit);
	}
#ifdef AP_CONTROL_CLK
	FP_LOG(INFO, "Get the clk resource.\n");
	/* Enable spi clock */
	if (gfspi_ioctl_clk_init(gf_dev))
		goto gfspi_probe_clk_init_failed;

	if (gfspi_ioctl_clk_enable(gf_dev))
		goto gfspi_probe_clk_enable_failed;

	spi_clock_set(gf_dev, 1000000);
#endif

	gf_dev->notifier = goodix_noti_block;
	fb_register_client(&gf_dev->notifier);
	gf_reg_key_kernel(gf_dev);

	wake_lock_init(&gf_milan_wake_lock, WAKE_LOCK_SUSPEND, "goodix_lock");

	gf_dev->irq = gf_milan_irq_num(gf_dev);

	ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"gf", gf_dev);

	if (!ret) {
		enable_irq_wake(gf_dev->irq);
		gf_disable_irq(gf_dev);
	}

	FP_LOG(INFO, "probe end successful\n");

	return status;

error:
	FP_LOG(ERR, "error in probe\n");
	gf_milan_cleanup(gf_dev);
	gf_dev->device_available = 0;
	if (gf_dev->devt != 0) {
		FP_LOG(ERR, "Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
		gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
	}

	return status;
}

/*static int __devexit gf_remove(struct spi_device *spi)*/
#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
	input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0)
		kfree(gf_dev);

	fb_unregister_client(&gf_dev->notifier);
	mutex_unlock(&device_list_lock);

	return 0;
}

#if defined(USE_SPI_BUS)
static int gf_suspend(struct spi_device *spi, pm_message_t mesg)
#elif defined(USE_PLATFORM_BUS)
static int gf_suspend(struct platform_device *pdev, pm_message_t state)
#endif
{
#if 0/* defined(USE_SPI_BUS) */
	struct gf_dev *gfspi_device;

	pr_debug("%s: enter\n", __func__);

	gfspi_device = spi_get_drvdata(spi);
	gfspi_ioctl_clk_disable(gfspi_device);
#endif
	FP_LOG(INFO, "gf_suspend_test.\n");
	return 0;
}

#if defined(USE_SPI_BUS)
static int gf_resume(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_resume(struct platform_device *pdev)
#endif
{
#if 0/* defined(USE_SPI_BUS) */
	struct gf_dev *gfspi_device;

	pr_debug("%s: enter\n", __func__);

	gfspi_device = spi_get_drvdata(spi);
	gfspi_ioctl_clk_enable(gfspi_device);
#endif
	FP_LOG(INFO, "gf_resume_test.\n");
	return 0;
}

/*
*static const struct dev_pm_ops gx_pm = {
*   .suspend = gf_suspend_test,
*  .resume = gf_resume_test
*};
*/
static const struct of_device_id gx_match_table[] = {
	{.compatible = GF_SPIDEV_NAME,},
	{},
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
#if defined(USE_SPI_BUS)
		/* .bus    = &spi_bus_type, */
#endif
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
	.suspend = gf_suspend,
	.resume = gf_resume,
};

static int __init gf_init(void)
{
	int status;

	/* 0:Goodix 1:Synapatics */
	if (is_goodix_milan_fp()) {
		pr_info("<HW> Goodix GF3238 fingerprint hw detected\n");
	} else {
		pr_info("<HW> not GF3238 Goodix fingerprint hw, skip\n");
		goto out;
	}

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		FP_LOG(INFO, "Failed to register char device!\n");
		return status;
	}
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		FP_LOG(INFO, "Failed to create class.\n");
		return PTR_ERR(gf_class);
	}
#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
	status = spi_register_driver(&gf_driver);
#endif
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		FP_LOG(INFO, "Failed to register SPI driver.\n");
	}

#ifdef GF_NETLINK_ENABLE
	netlink_init();
#endif
	FP_LOG(INFO, "status = 0x%x\n", status);

out:
	return 0;
}

module_init(gf_init);

static void __exit gf_exit(void)
{
#ifdef GF_NETLINK_ENABLE
	netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&gf_driver);
#endif
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}

module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
