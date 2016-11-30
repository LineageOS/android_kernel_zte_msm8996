/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.0   
 * Revision Record: 
 *      V1.0:  first release. 2014/09/28.
 *
 */

#include <linux/irq.h>
#include "gt1x.h"
#include "gt1x_generic.h"
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

static bool g_enable = false;
static s32 gt1x_halt = 0;
static struct work_struct gt1x_work;
static struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";
static struct delayed_work gProbeWork;
static struct workqueue_struct *gProbeWorkqueue;
int goodix_irq_gpio;
int goodix_reset_gpio;
int g_fwudone_status = 0;
struct regulator *dvdd;
extern bool globle_usb_present;

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
int goodix_avdd_gpio;
#else
struct regulator *avdd;
#endif

#if GTP_GESTURE_WAKEUP
static ssize_t gtp_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t gtp_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute attrs[] = {
	__ATTR(wake_gesture, 0644,
			gtp_wake_gesture_show,
			gtp_wake_gesture_store),
};
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
struct tp_proc_entry gProcEntry;//add by zte
atomic_t inter_count = ATOMIC_INIT(0);
atomic_t report_count = ATOMIC_INIT(0);
#endif

#ifdef GOODIX_ZTE_PROC_FLAG
struct proc_dir_entry *mt_entry = NULL;
struct tsc_id_t	tsc_id_info;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt1x_ts_early_suspend(struct early_suspend *h);
static void gt1x_ts_late_resume(struct early_suspend *h);
#endif

#if defined(CONFIG_FB)
struct notifier_block gt1x_fb_notif;
static int gt1x_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif

#if GTP_GESTURE_WAKEUP
static ssize_t gtp_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    GTP_INFO("show gesture");
    return snprintf(buf, PAGE_SIZE, "%u\n",gesture_enabled);
}

static ssize_t gtp_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

    GTP_INFO("set gesture");
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	gesture_enabled = input;

	if (gesture_enabled){
		enable_irq_wake(gt1x_i2c_client->irq);
	} else {
		disable_irq_wake(gt1x_i2c_client->irq);
	}
	
	return count;
}
#endif

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static spinlock_t irq_lock;
static s32 irq_is_disable = 0;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void gt1x_power_switch(s32 state)
{
}

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#if GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
	if(globle_usb_present)
		return true;
	else
		return false;
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_HANDLED: interrupt handled successfully
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	GTP_DEBUG_FUNC();

#ifdef GOODIX_ZTE_DEBUG_FLAG
	if((unsigned int)atomic_read(&inter_count) > 10000)
		{atomic_set(&inter_count, 1);}
	else
		{atomic_inc(&inter_count);}
#endif

	queue_work(gt1x_wq, &gt1x_work);
	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 z, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
	if((unsigned int)atomic_read(&report_count) > 10000)
		{atomic_set(&report_count, 1);}
	else
		{atomic_inc(&report_count);}
#endif

#if GTP_ICS_SLOT_REPORT
	
	input_mt_slot(input_dev, id);
	//input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, finger_status);
	input_report_key(input_dev, BTN_TOUCH, 1);
	input_report_key(input_dev,BTN_TOOL_FINGER, 1);
	input_report_abs(input_dev, ABS_MT_PRESSURE, z);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_report_key(input_dev,BTN_TOOL_FINGER, 0);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };

	//GTP_INFO("receive a irq");
	
#if GTP_GESTURE_WAKEUP
	ret = gesture_event_handler(input_dev);
	if (ret >= 0) {
		goto exit_work_func;
	}
#endif

	if (gt1x_halt) {
		GTP_DEBUG("return for interrupt after suspend...  ");
		return;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00) {
		gt1x_request_event_handler();
	}

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			GTP_ERROR("buffer not ready:0x%02x", finger);
			return;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret) {
		goto exit_work_func;
	}
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0) {
		goto exit_work_func;
	}
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0) {
			GTP_INFO("I2C write end_cmd  error!");
		}
	}
}

/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);

	if (ret < 0) {
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}

	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);

	ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler, irq_table[gt1x_int_type], gt1x_i2c_client->name, gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);

		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(input_dev, 16, INPUT_MT_DIRECT);	// in case of "out of memory"
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, 0x260);
	input_set_capability(input_dev, EV_KEY, 0x261);
	input_set_capability(input_dev, EV_KEY, 0x262);
	input_set_capability(input_dev, EV_KEY, 0x263);
	input_set_capability(input_dev, EV_KEY, 0x264);	
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}

#ifdef GOODIX_ZTE_PROC_FLAG
static int  tsc_id_proc_read(struct seq_file *m, void *v)
{

	switch (tsc_id_info.sensor_id)
		{
		case 0x00:
		 	seq_printf(m, "%s-", "Samsung");
			break ;
		case 0x02:
		 	seq_printf(m, "%s-", "truly");
			break ;
		 default:
		 	seq_printf(m, "%s-", "NA");
		}

	seq_printf(m, "goodix%s-NA-0x%02X-0x5d\n", tsc_id_info.product_id, gt1x_config[0]);


	return 0;
}

static int tsc_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tsc_id_proc_read, NULL);
}

 static const struct file_operations tsc_id = {
	.open		= tsc_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
//--------------------------zte debug start---------------------------------
static int zte_debug_read(struct seq_file *m, void *v)
{
	unsigned long irqflags = 0;
	unsigned char TempData = 0;
	
	seq_printf(m, "\n\n>>>>>> Goodix Driver Debug <<<<<\n\n");
	
	seq_printf(m, "gt1x halt		%d\n", gt1x_halt);
	
	spin_lock_irqsave(&irq_lock, irqflags);
	seq_printf(m, "gt1x irq enable	%s\n", irq_is_disable ? "no":"yes");
	spin_unlock_irqrestore(&irq_lock, irqflags);
	
	seq_printf(m, "irq count:		%u\n", atomic_read(&inter_count));
	seq_printf(m, "report count:		%u\n", atomic_read(&report_count));


	
#if GTP_GESTURE_WAKEUP
	seq_printf(m, "gesture enabled		%d\n", gesture_enabled);

	switch (gesture_doze_status)
		{
		case DOZE_DISABLED:
		 	seq_printf(m, "gesture doze status: %s\n", "DOZE_DISABLED");
			break;
		case DOZE_ENABLED:
		 	seq_printf(m, "gesture doze status: %s\n", "DOZE_ENABLED");
			break;
		case DOZE_WAKEUP:
		 	seq_printf(m, "gesture doze status: %s\n", "DOZE_WAKEUP");
			break;
		default:
		 	seq_printf(m, "gesture doze status: %s\n", "xxxxx unknow xxxxx");
		}
#endif
	if(gt1x_halt)
		return 0;

	if (gt1x_i2c_read(GTP_READ_COOR_ADDR, &TempData, sizeof(TempData)) >= 0) {
		seq_printf(m, "Coor data ready: %s\n", (TempData & 0x80) ? "yes" : "no");

		seq_printf(m, "Have key: %s\n", (TempData & 0x10) ? "yes" : "no");

		seq_printf(m, "Number of touch points: %u\n", TempData & 0x0F);
	}
	
	return 0;
}

 static int zte_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, zte_debug_read, NULL);
}

 static const struct file_operations debug_id = {
	.open		= zte_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
//--------------------------zte debug end---------------------------------

 static void create_tp_info_proc_file(void)
{
	memset(&gProcEntry, 0, sizeof(struct tp_proc_entry));

	gProcEntry.proc_path_entry = proc_mkdir("driver/TP_DEBUG", NULL);
	if(gProcEntry.proc_path_entry){
		GTP_INFO("create TP_DEBUG success");
	} else {
		GTP_INFO("create TP_DEBUG fail");
		return;
	}
	
	gProcEntry.debug_entry = proc_create("driver/TP_DEBUG/SynRegDebug", 0444, NULL, &debug_id);
	if (gProcEntry.debug_entry)	{
		GTP_INFO("create SynRegDebug success");
	} else {
		GTP_INFO("create SynRegDebug fail");
		return;
	}

#if 0
	gProcEntry.firmware_ver_entry = proc_create("driver/TP_DEBUG/FirmwareVer", 0444, NULL, &FirmwareVer_id);
	if (gProcEntry.firmware_ver_entry)
		{printk("create SynRegDebug success\n");}
	else
		{printk("create SynRegDebug fail\n");return;}
#endif
}

static void remove_tp_info_proc_file(void)
{
	if(gProcEntry.debug_entry){
		remove_proc_entry("driver/TP_DEBUG/SynRegDebug", NULL);
		gProcEntry.debug_entry = NULL;
	}

	if(gProcEntry.firmware_ver_entry){
		remove_proc_entry("driver/TP_DEBUG/FirmwareVer", NULL);
		gProcEntry.firmware_ver_entry = NULL;
	}
	//Note:update

	if(gProcEntry.proc_path_entry){
		remove_proc_entry("driver/TP_DEBUG", NULL);
		gProcEntry.proc_path_entry = NULL;
	}
	
}

static void synaptics_zte_debug_work(void)
{

	atomic_set(&inter_count, 0);
	atomic_set(&report_count, 0);

	return;
}
#endif

static int gt1x_ts_regulator(bool enable)
{
	int ret = 0;

	if (g_enable == enable){
		GTP_INFO("same enable reg");
		return ret;
	} else {
		//GTP_INFO("different enable reg");
		g_enable = enable;
	}
	
	if (enable){
#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
		if (goodix_avdd_gpio > 0){
			gpio_set_value(goodix_avdd_gpio, 1);
			msleep(10);
		} else {
			GTP_INFO("goodix_avdd_gpio less 0 while enable");
		}
#else
		ret = regulator_enable(avdd);
		if(ret < 0)
		{
			GTP_ERROR("%s: Failed to enable regulator avdd\n", __func__);
		}
#endif
		
		ret = regulator_enable(dvdd);
		if(ret < 0)
		{
			GTP_ERROR("%s: Failed to enable regulator dvdd\n", __func__);
		}
	} else {
		ret = regulator_disable(dvdd);
		if(ret < 0)
		{
			GTP_ERROR("%s: Failed to disable regulator dvdd\n", __func__);
		}
	
#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
		if (goodix_avdd_gpio > 0){
			gpio_set_value(goodix_avdd_gpio, 0);
			msleep(10);
		} else {
			GTP_INFO("goodix_avdd_gpio less 0 while disable");
		}
#else
		ret = regulator_disable(avdd);
		if(ret < 0)
		{
			GTP_ERROR("%s: Failed to disable regulator avdd\n", __func__);
		}
#endif
	}

	return ret;
}

static int gt1x_fwupdate_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n",g_fwudone_status);
	return 0;
}

static int gt1x_fwupdate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gt1x_fwupdate_proc_show, NULL);
}

static const struct file_operations gt1x_fwupdate_proc_fops = {
	.open		= gt1x_fwupdate_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int gt1x_fwupdate_proc_init(void)
{
       struct proc_dir_entry *res;
	printk("%s %d \n", __func__, __LINE__);
	res = proc_create("gt1x_fwudone_switch", S_IWUGO | S_IRUGO, NULL,
			  &gt1x_fwupdate_proc_fops);
	if (!res)
	{
		printk(KERN_INFO "failed to create /proc/gt1x_fwudone_switch\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "created /proc/gt1x_fwudone_switch\n");
	return 0;
}

static void gt1x_ts_probe_work(struct work_struct *work)
{
	s32 ret = -1;
	struct i2c_client *client = gt1x_i2c_client;
	struct device_node *np = client->dev.of_node;
	
#if GTP_AUTO_UPDATE
	struct task_struct *thread = NULL;
#endif
	
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return;
	}

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
	if (of_find_property(np, "goodix,avdd-gpio", NULL))
		goodix_avdd_gpio = of_get_named_gpio_flags(np, "goodix,avdd-gpio", 0, NULL);
	else
		goodix_avdd_gpio = -1;
#endif

	if (of_find_property(np, "goodix,reset-gpio", NULL))
		goodix_reset_gpio = of_get_named_gpio_flags(np, "goodix,reset-gpio", 0, NULL);
	else
		goodix_reset_gpio = -1;
	
	if (of_find_property(np, "goodix,irq-gpio", NULL))
		goodix_irq_gpio = of_get_named_gpio_flags(np, "goodix,irq-gpio", 0, NULL);
	else
		goodix_irq_gpio = -1;

	GTP_INFO("GTP reset-gpio = %d, irq-gpio =%d\n", goodix_reset_gpio, goodix_irq_gpio);

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return;
	}

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
	if (gpio_request(goodix_avdd_gpio, "tp_avdd")) {
		GTP_ERROR("%s: Failed to request avdd\n", __func__);
		return;
	} else {
		gpio_direction_output(goodix_avdd_gpio, 0);
	}
#else
	avdd = regulator_get(&client->dev, "avdd");
	if(IS_ERR(avdd)) 
	{
		GTP_ERROR("%s: Failed to get regulator avdd\n", __func__);
		return;
	}
#endif

	dvdd = regulator_get(&client->dev, "dvdd");
	if(IS_ERR(dvdd)) 
	{
		GTP_ERROR("%s: Failed to get regulator dvdd\n", __func__);
		return;
	}

	ret = gt1x_ts_regulator(true);
	if(ret < 0)
	{
		GTP_ERROR("%s: Failed to enable regulator avdd or dvdd\n", __func__);
		return;
	}

	msleep(10);
	
	gt1x_init();

	INIT_WORK(&gt1x_work, gt1x_ts_work_func);

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

//#if GTP_GESTURE_WAKEUP
//	enable_irq_wake(client->irq);
//#endif

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_GESTURE_WAKEUP
    ret = sysfs_create_file(&input_dev->dev.kobj,&attrs[0].attr);
    if (ret < 0) {
        GTP_ERROR("%s:create attr file failed",__func__);
    }
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_INFO("Failed to create auto-update thread: %d.", ret);
	}
#endif

#ifdef GOODIX_ZTE_PROC_FLAG
	mt_entry = proc_create("driver/tsc_id", 0666, NULL, &tsc_id);
	if (mt_entry == NULL) {
		GTP_ERROR("create tsc_id fail");
	}
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
	create_tp_info_proc_file();
#endif

#ifdef CONFIG_CREATE_GLOVED_INTERFACE
	gloved_finger_proc_init();
#endif
	gt1x_fwupdate_proc_init();
#if defined(CONFIG_FB)
	gt1x_fb_notif.notifier_call = gt1x_fb_notifier_callback;
	ret = fb_register_client(&gt1x_fb_notif);
	if (ret)
		GTP_ERROR("Unable to register fb_notifier: %d\n", ret);
#endif
}
/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	GTP_INFO("gt1x_ts_probe in");
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	gt1x_i2c_client = client;
	
	gProbeWorkqueue = create_singlethread_workqueue("gt1x_ts_probe_wq");
	
	INIT_DELAYED_WORK(&gProbeWork, gt1x_ts_probe_work);

	queue_delayed_work(gProbeWorkqueue,	&gProbeWork,msecs_to_jiffies(0));
#if 0	
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
	if (of_find_property(np, "goodix,avdd-gpio", NULL))
		goodix_avdd_gpio = of_get_named_gpio_flags(np, "goodix,avdd-gpio", 0, NULL);
	else
		goodix_avdd_gpio = -1;
#endif

	if (of_find_property(np, "goodix,reset-gpio", NULL))
		goodix_reset_gpio = of_get_named_gpio_flags(np, "goodix,reset-gpio", 0, NULL);
	else
		goodix_reset_gpio = -1;
	
	if (of_find_property(np, "goodix,irq-gpio", NULL))
		goodix_irq_gpio = of_get_named_gpio_flags(np, "goodix,irq-gpio", 0, NULL);
	else
		goodix_irq_gpio = -1;

	GTP_INFO("GTP reset-gpio = %d, irq-gpio =%d\n", goodix_reset_gpio, goodix_irq_gpio);

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return ret;
	}

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
	if (gpio_request(goodix_avdd_gpio, "tp_avdd")) {
		GTP_ERROR("%s: Failed to request avdd\n", __func__);
		return -1;
	} else {
		gpio_direction_output(goodix_avdd_gpio, 0);
	}
#else
	avdd = regulator_get(&client->dev, "avdd");
	if(IS_ERR(avdd)) 
	{
		GTP_ERROR("%s: Failed to get regulator avdd\n", __func__);
		return -1;
	}
#endif

	dvdd = regulator_get(&client->dev, "dvdd");
	if(IS_ERR(dvdd)) 
	{
		GTP_ERROR("%s: Failed to get regulator dvdd\n", __func__);
		return -1;
	}

	ret = gt1x_ts_regulator(true);
	if(ret < 0)
	{
		GTP_ERROR("%s: Failed to enable regulator avdd or dvdd\n", __func__);
		return ret;
	}

	msleep(10);
	
	gt1x_init();

	INIT_WORK(&gt1x_work, gt1x_ts_work_func);

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

//#if GTP_GESTURE_WAKEUP
//	enable_irq_wake(client->irq);
//#endif

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_GESTURE_WAKEUP
    ret = sysfs_create_file(&input_dev->dev.kobj,&attrs[0].attr);
    if (ret < 0) {
        GTP_ERROR("%s:create attr file failed",__func__);
    }
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_INFO("Failed to create auto-update thread: %d.", ret);
	}
#endif

#ifdef GOODIX_ZTE_PROC_FLAG
	mt_entry = proc_create("driver/tsc_id", 0666, NULL, &tsc_id);
	if (mt_entry == NULL) {
		GTP_ERROR("create tsc_id fail");
	}
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
	create_tp_info_proc_file();
#endif

#ifdef CONFIG_CREATE_GLOVED_INTERFACE
	gloved_finger_proc_init();
#endif

#if defined(CONFIG_FB)
	gt1x_fb_notif.notifier_call = gt1x_fb_notifier_callback;
	ret = fb_register_client(&gt1x_fb_notif);
	if (ret)
		GTP_ERROR("Unable to register fb_notifier: %d\n", ret);
#endif

#endif //CONFIG_PROBE_WORK
	GTP_INFO("gt1x_ts_probe out");
	return 0;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	int ret;
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");

#ifdef GOODIX_ZTE_PROC_FLAG
	if(mt_entry){
		remove_proc_entry("driver/tsc_id", NULL);
		mt_entry = NULL;
	}
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
	remove_tp_info_proc_file();
#endif

#if GTP_GESTURE_WAKEUP
	if (gesture_enabled)
		disable_irq_wake(client->irq);
#endif

#if GTP_CREATE_WR_NODE
	gt1x_deinit_tool_node();
#endif

#if GTP_ESD_PROTECT
	gt1x_deinit_esd_protect();
#endif

	ret = gt1x_ts_regulator(false);
	if(ret < 0)
	{
		GTP_ERROR("%s: Failed to disable regulator dvdd or avdd\n", __func__);
	}
	regulator_put(dvdd);
	dvdd = NULL;

#ifdef CONFIG_TOUCHSCREEN_AVDD_FROM_TLV
	gpio_free(goodix_avdd_gpio);
#else
	regulator_put(avdd);
	avdd = NULL;
#endif
	
	input_unregister_device(input_dev);

	return 0;
}

#ifdef CONFIG_PM
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_suspend(struct device *dev)
{
	s32 ret = -1;
#if GTP_HOTKNOT && !HOTKNOT_BLOCK_RW
	u8 buf[1] = { 0 };
#endif

	GTP_INFO("TPD suspend start...");

	ndt_stress_suspend();

#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Suspend: proximity is detected!");
		return 0;
	}
#endif

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			GTP_INFO("Suspend: hotknot is paired!");
			return 0;
		}
#else
		gt1x_i2c_read(GTP_REG_HN_PAIRED, buf, sizeof(buf));
		GTP_DEBUG("0x81AA: 0x%02X", buf[0]);
		if (buf[0] == 0x55 || hotknot_transfer_mode) {
			GTP_INFO("Suspend: hotknot is paired!");
			return 0;
		}
#endif
	}
#endif

	gt1x_halt = 1;
#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_OFF);
#endif
	gt1x_irq_disable();
	cancel_work_sync(&gt1x_work);

#if GTP_GESTURE_WAKEUP
	gesture_clear_wakeup_data();
	if (gesture_enabled) {
		gesture_enter_doze();
		gt1x_irq_enable();
		gt1x_halt = 0;
	} else
#endif
	{
		ret = gt1x_enter_sleep();
		if (ret < 0) {
			GTP_ERROR("GTP early suspend failed.");
		} 

		GTP_INFO("disable regulator");
		gt1x_ts_regulator(false);
	}

#ifdef GOODIX_ZTE_DEBUG_FLAG
   	printk("[TP] *************** In suspend: irq count %u, report count %u\n", atomic_read(&inter_count), atomic_read(&report_count));
#endif
	/* to avoid waking up while not sleeping
	   delay 48 + 10ms to ensure reliability */
	msleep(58);
	return 0;
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_resume(struct device *dev)
{
	s32 ret = -1;

	GTP_INFO("TPD resume start...");
	
	ndt_stress_resume();

#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Resume: proximity is on!");
		return 0;
	}
#endif

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			hotknot_paired_flag = 0;
			GTP_INFO("Resume: hotknot is paired!");
			return 0;
		}
#endif
	}
#endif

#if GTP_GESTURE_WAKEUP
	if (gesture_enabled) {
	} else
#endif
	{
		GTP_INFO("enable regulator");
		gt1x_ts_regulator(true);
	}

	ret = gt1x_wakeup_sleep();
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
#if GTP_HOTKNOT
	if (!hotknot_enabled) {
		gt1x_send_cmd(GTP_CMD_HN_EXIT_SLAVE, 0);
	}
#endif

#if GTP_CHARGER_SWITCH
	gt1x_charger_config(0);
	gt1x_charger_switch(SWITCH_ON);
#endif

	gt1x_halt = 0;
	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_ON);
#endif

#ifdef CONFIG_CREATE_GLOVED_INTERFACE
	write_gloved_value(current_gloved_finger_state);
#endif

#ifdef GOODIX_ZTE_DEBUG_FLAG
	synaptics_zte_debug_work();
#endif

	GTP_DEBUG("tpd resume end.");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_ts_suspend(NULL);
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_ts_resume(NULL);
}

struct early_suspend early_suspend;

#endif

//static const struct dev_pm_ops gt1x_ts_pm_ops = {
//#ifndef CONFIG_HAS_EARLYSUSPEND
//	.suspend = gt1x_ts_suspend,
//	.resume = gt1x_ts_resume,
//#endif
//};
#endif

#ifdef CONFIG_FB
static int gt1x_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) 
	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			gt1x_ts_resume(NULL);
		else if (*blank == FB_BLANK_POWERDOWN)
			gt1x_ts_suspend(NULL);
	}

	return 0;
}
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,gt1x",},
	{ },
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = goodix_match_table,
//#ifdef CONFIG_PM
//		   .pm = &gt1x_ts_pm_ops,
//#endif
		   },
};

/**   
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
//static int __devinit gt1x_ts_init(void)
static int gt1x_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = gt1x_ts_early_suspend;
	early_suspend.resume = gt1x_ts_late_resume;
	register_early_suspend(&early_suspend);
#endif

	ret = i2c_add_driver(&gt1x_ts_driver);
	return ret;
}

/**   
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq) {
		destroy_workqueue(gt1x_wq);
	}

	gt1x_deinit();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
}

late_initcall(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
