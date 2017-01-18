/* hd3ss3220.c -- TI HD3SS3220 USB TYPE-C Controller device driver */
/* Copyright (c) 2009-2016, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/errno.h>
#include <linux/err.h>

#include <linux/power_supply.h>


#define TYPEC_PM_OP

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_BASE                    0x00
/* 0x00 - 0x07 reserved for ID */
#define REG_MOD                     0x08
#define REG_INT                     0x09
#define REG_SET                     0x0A
#define REG_CTL                     0X45
#define REG_REVISION                0XA0

/******************************************************************************
* Register bits
******************************************************************************/
/* REG_ID (0x00) */
#define ID_REG_LEN  0X08

/* REG_MOD (0x08) */
#define MOD_ACTIVE_CABLE_DETECTION  0X01    /*RU*/
#define MOD_ACCESSORY_CONNECTED_SHIFT   1
#define MOD_ACCESSORY_CONNECTED         (0x07 << MOD_ACCESSORY_CONNECTED_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_DETECT_SHIFT    4
#define MOD_CURRENT_MODE_DETECT         (0x03 << MOD_CURRENT_MODE_DETECT_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_ADVERTISE_SHIFT    6
#define MOD_CURRENT_MODE_ADVERTISE          (0x03 << MOD_CURRENT_MODE_ADVERTISE_SHIFT)    /*RW*/

/* REG_INT (0x09) */
#define INT_DRP_DUTY_CYCLE_SHIFT    1
#define INT_DRP_DUTY_CYCLE          (0x03 << INT_DRP_DUTY_CYCLE_SHIFT)      /*RW*/
#define INT_VCONN_FAULT_SHIFT       3
#define INT_VCONN_FAULT             (0x01 << INT_VCONN_FAULT_SHIFT)         /*RCU*/
#define INT_INTERRUPT_STATUS_SHIFT  4
#define INT_INTERRUPT_STATUS        (0x01 << INT_INTERRUPT_STATUS_SHIFT)    /*RCU*/
#define INT_CABLE_DIR_SHIFT         5
#define INT_CABLE_DIR               (0x01 << INT_CABLE_DIR_SHIFT)           /*RU*/
#define INT_ATTACHED_STATE_SHIFT    6
#define INT_ATTACHED_STATE          (0x03 << INT_ATTACHED_STATE_SHIFT)      /*RU*/

/* REG_SET (0x0A) */
#define SET_I2C_DISABLE_TERM        0x01    /*RW*/
#define SET_I2C_SOURCE_PREF_SHIFT   1
#define SET_I2C_SOURCE_PREF         (0x03 << SET_I2C_SOURCE_PREF_SHIFT)  /*RW*/
#define SET_I2C_SOFT_RESET_SHIFT    3
#define SET_I2C_SOFT_RESET          (0x01 << SET_I2C_SOFT_RESET_SHIFT)  /*RSU*/
#define SET_MODE_SELECT_SHIFT       4
#define SET_MODE_SELECT             (0x03 << SET_MODE_SELECT_SHIFT)     /*RW*/
#define SET_DEBOUNCE_SHIFT          6
#define SET_DEBOUNCE                (0x03 << SET_DEBOUNCE_SHIFT)        /*RW*/

/* REG_CTR (0x45) */
#define CTR_DISABLE_RD_RP_SHIFT     2
#define CTR_DISABLE_RD_RP           (0x01 << CTR_DISABLE_RD_RP_SHIFT)   /*RW*/

/******************************************************************************
 * Register values
 ******************************************************************************/
/* SET_MODE_SELECT */
#define SET_MODE_SELECT_DEFAULT  0x00
#define SET_MODE_SELECT_SNK       0x01
#define SET_MODE_SELECT_SRC       0x02
#define SET_MODE_SELECT_DRP       0x03
/* MOD_CURRENT_MODE_ADVERTISE */
#define MOD_CURRENT_MODE_ADVERTISE_DEFAULT      0x00
#define MOD_CURRENT_MODE_ADVERTISE_MID          0x01
#define MOD_CURRENT_MODE_ADVERTISE_HIGH         0x02
/* MOD_CURRENT_MODE_DETECT */
#define MOD_CURRENT_MODE_DETECT_DEFAULT      0x00
#define MOD_CURRENT_MODE_DETECT_MID          0x01
#define MOD_CURRENT_MODE_DETECT_ACCESSARY    0x02
#define MOD_CURRENT_MODE_DETECT_HIGH         0x03


/******************************************************************************
 * Constants
 ******************************************************************************/

enum drp_toggle_type {
	TOGGLE_DFP_DRP_30 = 0,
	TOGGLE_DFP_DRP_40,
	TOGGLE_DFP_DRP_50,
	TOGGLE_DFP_DRP_60
};

enum current_adv_type {
	HOST_CUR_USB = 0,   /*default 500mA or 900mA*/
	HOST_CUR_1P5,      /*1.5A*/
	HOST_CUR_3A       /*3A*/
};

enum current_det_type {
	DET_CUR_USB = 0,    /*default 500mA or 900mA*/
	DET_CUR_1P5,
	DET_CUR_ACCESSORY,  /*charg through accessory 500mA*/
	DET_CUR_3A
};

enum accessory_attach_type {
	ACCESSORY_NOT_ATTACHED = 0,
	ACCESSORY_AUDIO = 4,
	ACCESSORY_CHG_THRU_AUDIO = 5,
	ACCESSORY_DEBUG = 6
};

enum cable_attach_type {
	CABLE_NOT_ATTACHED = 0,
	CABLE_ATTACHED
};

enum cable_state_type {
	CABLE_STATE_NOT_ATTACHED = 0,
	CABLE_STATE_AS_DFP,
	CABLE_STATE_AS_UFP,
	CABLE_STATE_TO_ACCESSORY
};

enum cable_dir_type {
	ORIENT_CC2 = 0,
	ORIENT_CC1
};

enum vconn_fault_type {
	VCONN_NO_FAULT = 0,
	VCONN_FAULT
};

enum cc_modes_type {
	MODE_DEFAULT = 0,
	MODE_UFP,
	MODE_DFP,
	MODE_DRP
};

/* Type-C Attrs */
struct type_c_parameters {
	enum current_det_type current_det;         /*charging current on UFP*/
	enum accessory_attach_type accessory_attach;     /*if an accessory is attached*/
	enum cable_attach_type active_cable_attach;         /*if an active_cable is attached*/
	enum cable_state_type attach_state;        /*DFP->UFP or UFP->DFP*/
	enum cable_dir_type cable_dir;           /*cc1 or cc2*/
	enum vconn_fault_type vconn_fault;         /*vconn fault*/
};

struct state_disorder_monitor {
	int count;
	int err_detected;
	enum cable_state_type former_state;
	unsigned long time_before;
};

enum en_gpio_type {
	EN_CC_MUX = 0,
	EN_RE_DRIVE,
	EN_VREG_5V,
	EN_VREG_3V3,
	EN_GPIO_MAX = 4
};

struct en_gpio_item {
	char *name;
	int index;
};

static struct en_gpio_item en_gpio_info[] = {
	{
		.name   = "hd3ss,en_cc_gpio",
		.index  = EN_CC_MUX,

	},
	{
		.name   = "hd3ss,re_drv_gpio",
		.index  = EN_RE_DRIVE,

	},
	{
		.name   = "hd3ss,vreg_5v_gpio",
		.index  = EN_VREG_5V,

	},
	{
		.name   = "hd3ss,vreg_3v3_gpio",
		.index  = EN_VREG_3V3,

	},
	/*add more gpio here if need*/
};

struct hd3ss3220_info {
	struct i2c_client  *i2c;
	struct device  *dev_t;
	struct mutex  mutex;
	struct class  *device_class;

	struct pinctrl  *pinctrl;
	struct pinctrl_state  *cc_int_cfg;

	int irq_gpio;
	int en_gpio[EN_GPIO_MAX];

	struct type_c_parameters type_c_param;
	struct state_disorder_monitor monitor;

};

/* i2c operate interfaces */
static int hd3ss3220_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int hd3ss3220_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);

	return ret;
}

static int hd3ss3220_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
	int ret;
	u8 old_val, new_val;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);

	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&info->mutex);
	return ret;
}

/* Config DFP/UFP/DRP mode */
/* e.g #echo 1 >/sys/class/type-c/hd3ss3220/mode_select */
static ssize_t mode_select_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = hd3ss3220_read_reg(info->i2c, REG_SET, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & SET_MODE_SELECT) >> SET_MODE_SELECT_SHIFT;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t mode_select_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t size)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	int mode;
	u8 value;
	int ret;

	ret = kstrtoint(buf, 0, &mode);
	if (ret != 0)
		return -EINVAL;

	if (mode == MODE_DFP)
		value = SET_MODE_SELECT_SRC;
	else if (mode == MODE_UFP)
		value = SET_MODE_SELECT_SNK;
	else if (mode == MODE_DRP)
		value = SET_MODE_SELECT_DRP;
	else
		return -EINVAL;

	value = value << SET_MODE_SELECT_SHIFT;
	ret = hd3ss3220_update_reg(info->i2c, REG_SET, value, SET_MODE_SELECT);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	return ret;
}


/* Advertise current when act as DFP */
static ssize_t current_advertise_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & MOD_CURRENT_MODE_ADVERTISE) >> MOD_CURRENT_MODE_ADVERTISE_SHIFT;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t current_advertise_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	int mode;
	u8 value;
	int ret;

	ret = kstrtoint(buf, 0, &mode);
	if (ret != 0)
		return -EINVAL;

	if (mode == HOST_CUR_USB)
		value = MOD_CURRENT_MODE_ADVERTISE_DEFAULT;
	else if (mode == HOST_CUR_1P5)
		value = MOD_CURRENT_MODE_ADVERTISE_MID;
	else if (mode == HOST_CUR_3A)
		value = MOD_CURRENT_MODE_ADVERTISE_HIGH;
	else
		return -EINVAL;

	value = value << MOD_CURRENT_MODE_ADVERTISE_SHIFT;

	ret = hd3ss3220_update_reg(info->i2c, REG_MOD, value, MOD_CURRENT_MODE_ADVERTISE);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	return ret;
}

/* Detct current when act as UFP */
static ssize_t current_detect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT;

	if (value == MOD_CURRENT_MODE_DETECT_DEFAULT)
		return snprintf(buf, PAGE_SIZE, "500mA or 900mA\n");
	else if (value == MOD_CURRENT_MODE_DETECT_MID)
		return snprintf(buf, PAGE_SIZE, "mid 1P5A\n");
	else if (value == MOD_CURRENT_MODE_DETECT_HIGH)
		return snprintf(buf, PAGE_SIZE, "high 3A\n");
	else if (value == MOD_CURRENT_MODE_DETECT_ACCESSARY)
		return snprintf(buf, PAGE_SIZE, "accessary 500mA\n");
	else
		return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t gpio_debug_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "EN_CC_MUX:%d, EN_RE_DRIVE:%d\n",
			gpio_get_value(info->en_gpio[EN_CC_MUX]),
			gpio_get_value(info->en_gpio[EN_RE_DRIVE]));
}

static ssize_t gpio_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	int level;
	int ret;

	ret = kstrtoint(buf, 0, &level);
	if (ret != 0)
		return -EINVAL;

	if (info->en_gpio[EN_CC_MUX])
		gpio_direction_output(info->en_gpio[EN_CC_MUX], !!level);
	if (info->en_gpio[EN_RE_DRIVE])
		gpio_direction_output(info->en_gpio[EN_RE_DRIVE], !!level);

	return 1;
}

/* For Emode, read typec chip info */
static ssize_t show_chip_info(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);
	u8 reg_val;
	char name[ID_REG_LEN + 1] = { 0 };
	u8 revision = 0;
	int i, j, ret;

	for (i = 0, j = 0; i < ID_REG_LEN; i++) {
		reg_val = 0;
		ret = hd3ss3220_read_reg(info->i2c, (REG_BASE + (ID_REG_LEN - 1) - i), &reg_val);
		if (ret < 0) {
			pr_err("%s: read reg fail!\n", __func__);
			snprintf(name, ID_REG_LEN, "%s", "Error");
			break;
		}
		if (!reg_val) {
			j++;
			continue;
		}
		name[i - j] = (char)reg_val;
	}

	ret = hd3ss3220_read_reg(info->i2c, REG_REVISION, &revision);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return snprintf(buf, PAGE_SIZE, "USB TypeC chip info:\n"
				"	Manufacture: TI\n"
				"	Chip Name  : %s\n"
				"	Revision ID : Error\n",
				name);
	} else {
		return snprintf(buf, PAGE_SIZE, "USB TypeC chip info:\n"
				"	Manufacture: TI\n"
				"	Chip Name  : %s\n"
				"	Revision ID : %d\n",
				name, revision);
	}
}

/**************************************************************************/
#define TYPE_C_ATTR(field, format_string) \
		static ssize_t \
		field ## _show(struct device *dev, struct device_attribute *attr, \
			char *buf) \
		{ \
			struct hd3ss3220_info *info = dev_get_drvdata(dev); \
			return snprintf(buf, PAGE_SIZE, \
			format_string, info->type_c_param.field); \
		} \
		static DEVICE_ATTR(field, S_IRUGO, field ## _show, NULL);

static DEVICE_ATTR(gpio_debug,  S_IRUGO | S_IWUSR,
			gpio_debug_show, gpio_debug_store);
static DEVICE_ATTR(mode_select, S_IRUGO | S_IWUSR,
			mode_select_show, mode_select_store);
static DEVICE_ATTR(current_advertise, S_IRUGO | S_IWUSR,
			current_advertise_show, current_advertise_store);
static DEVICE_ATTR(current_det_string, S_IRUGO, current_detect_show, NULL);

/* For Emode, read typec chip info */
static DEVICE_ATTR(chip_info, S_IRUGO, show_chip_info, NULL);

TYPE_C_ATTR(current_det, "%d\n")
TYPE_C_ATTR(accessory_attach, "%d\n")
TYPE_C_ATTR(active_cable_attach, "%d\n")
TYPE_C_ATTR(attach_state, "%d\n")
TYPE_C_ATTR(cable_dir, "%d\n")
TYPE_C_ATTR(vconn_fault, "%d\n")

static struct device_attribute *usb_typec_attributes[] = {
	&dev_attr_mode_select,
	&dev_attr_current_advertise,
	&dev_attr_current_det_string,
	&dev_attr_chip_info,
	&dev_attr_current_det,
	&dev_attr_accessory_attach,
	&dev_attr_active_cable_attach,
	&dev_attr_attach_state,
	&dev_attr_cable_dir,
	&dev_attr_vconn_fault,
	&dev_attr_gpio_debug,
	/*end*/
	NULL
};
/******************************************************************************/

/* Detect non-DFP->DFP changes that happen more than 3 times within 10 Secs */
static void state_disorder_detect(struct hd3ss3220_info *info)
{
	unsigned long timeout;

	/* count the (non-DFP -> DFP) changes */
	if ((info->monitor.former_state != info->type_c_param.attach_state)
	    && (info->type_c_param.attach_state == CABLE_STATE_AS_DFP)) {
		if (!info->monitor.count) {
			info->monitor.time_before = jiffies;
		}
		info->monitor.count++;
	}

	/* store the state */
	info->monitor.former_state = info->type_c_param.attach_state;

	if (info->monitor.count > 3) {
		timeout = msecs_to_jiffies(10 * 1000); /* 10 Seconds */
		if (time_before(jiffies, info->monitor.time_before + timeout)) {
			info->monitor.err_detected = 1;
			/* disbale id irq before qpnp react to cc chip's id output */
			interfere_id_irq_from_usb(0);
		}
		info->monitor.count = 0;
	}

	if ((info->type_c_param.attach_state == CABLE_STATE_NOT_ATTACHED)
	    && info->monitor.err_detected) {
		/* enable id irq */
		interfere_id_irq_from_usb(1);
		info->monitor.err_detected = 0;
	}
}

static void process_mode_register(struct hd3ss3220_info *info, u8 status)
{
	u8 val;
	u8 tmp = status;

	/* check current_detect */
	val = ((tmp & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT);
	info->type_c_param.current_det = val;

	/* check accessory attch */
	tmp = status;
	val = ((tmp & MOD_ACCESSORY_CONNECTED) >> MOD_ACCESSORY_CONNECTED_SHIFT);
	info->type_c_param.accessory_attach = val;

	/* check cable attach */
	tmp = status;
	val = (tmp & MOD_ACTIVE_CABLE_DETECTION);
}

static void process_interrupt_register(struct hd3ss3220_info *info, u8 status)
{
	u8 val;
	u8 tmp = status;

	/* check attach state */
	val = ((tmp & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT);
	info->type_c_param.attach_state = val;

	/* update current adv when act as DFP */
	if (info->type_c_param.attach_state == CABLE_STATE_AS_DFP ||
	    info->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		val = (HOST_CUR_USB << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	} else {
		val = (HOST_CUR_3A << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	}
	hd3ss3220_update_reg(info->i2c, REG_MOD, val, MOD_CURRENT_MODE_ADVERTISE);

	/* Hd3ss3220 configured as DRP may detect some non-standard SDP */
	/* chargers as UFP, which may lead to a cyclic switching of DFP */
	/* and UFP on state detection result. */
	state_disorder_detect(info);

	/* check cable dir */
	tmp = status;
	val = ((tmp & INT_CABLE_DIR) >> INT_CABLE_DIR_SHIFT);
	info->type_c_param.cable_dir = val;

	/* check vconn fault */
	tmp = status;
	val = ((tmp & INT_VCONN_FAULT) >> INT_VCONN_FAULT_SHIFT);
	info->type_c_param.vconn_fault = val;
}

static irqreturn_t hd3ss3220_irq_thread(int irq, void *handle)
{
	struct hd3ss3220_info *info = (struct hd3ss3220_info *)handle;
	u8 reg_val;
	int ret;

	pr_debug("%s enter\n", __func__);

	ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &reg_val);
	if (ret)
		goto done;

	pr_debug("%s mode_register:0x%x\n", __func__, reg_val);
	process_mode_register(info, reg_val);

	ret = hd3ss3220_read_reg(info->i2c, REG_INT, &reg_val);
	if (ret)
		goto done;

	pr_debug("%s interrupt_register:0x%x\n", __func__, reg_val);
	process_interrupt_register(info, reg_val);

	ret = hd3ss3220_update_reg(info->i2c,
				   REG_INT, (0x1 << INT_INTERRUPT_STATUS_SHIFT), INT_INTERRUPT_STATUS);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

done:
	return IRQ_HANDLED;
}

#ifdef TYPEC_PM_OP
static int  hd3ss3220_suspend(struct device *dev)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);

	pr_info("%s enter\n", __func__);
	disable_irq_wake(info->i2c->irq);
	disable_irq(info->i2c->irq);

	if (info->en_gpio[EN_CC_MUX])
		gpio_direction_output(info->en_gpio[EN_CC_MUX], 0);
	if (info->en_gpio[EN_RE_DRIVE])
		gpio_direction_output(info->en_gpio[EN_RE_DRIVE], 0);

	pr_debug("%s gpio value:%d,%d,%d,%d\n", __func__,
		 gpio_get_value(info->en_gpio[EN_CC_MUX]),
		 gpio_get_value(info->en_gpio[EN_RE_DRIVE]),
		 gpio_get_value(info->en_gpio[EN_VREG_5V]),
		 gpio_get_value(info->en_gpio[EN_VREG_3V3]));

	return 0;
}

static int  hd3ss3220_resume(struct device *dev)
{
	struct hd3ss3220_info *info = dev_get_drvdata(dev);

	pr_info("%s enter\n", __func__);

	if (info->en_gpio[EN_CC_MUX])
		gpio_direction_output(info->en_gpio[EN_CC_MUX], 1);
	if (info->en_gpio[EN_RE_DRIVE])
		gpio_direction_output(info->en_gpio[EN_RE_DRIVE], 1);

	msleep(20);
	pr_debug("%s gpio value:%d,%d,%d,%d\n", __func__,
		 gpio_get_value(info->en_gpio[EN_CC_MUX]),
		 gpio_get_value(info->en_gpio[EN_RE_DRIVE]),
		 gpio_get_value(info->en_gpio[EN_VREG_5V]),
		 gpio_get_value(info->en_gpio[EN_VREG_3V3]));

	enable_irq_wake(info->i2c->irq);
	enable_irq(info->i2c->irq);

	return 0;
}
#endif

static int hd3ss3220_pinctrl_init(struct hd3ss3220_info *info)
{
	struct pinctrl_state *set_state;

	info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		pr_err("%s: pinctrl not defined\n", __func__);
		return PTR_ERR(info->pinctrl);
	}

	pr_debug("%s: doing pinctrl\n", __func__);
	set_state = pinctrl_lookup_state(info->pinctrl, "hd3ss3220_pin_cfg");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("%s: pinctrl lookup failed\n", __func__);
		info->pinctrl = NULL;
		return PTR_ERR(set_state);
	}
	info->cc_int_cfg = set_state;

	/* get more pins conig here, if need */
	return 0;
}

static int hd3ss3220_initialization(struct hd3ss3220_info *info)
{
	int ret = 0;
	u8 reg_val;
	/* do initialization here, before enable irq,
	 * clear irq,
	 * config DRP/UFP/DFP mode,
	 * and etc..
	 */
	pr_debug("%s enter\n", __func__);
	ret = hd3ss3220_read_reg(info->i2c, REG_REVISION, &reg_val);
	if (ret < 0) {
		return ret;
	}

	/* 1 Disable term(default 0) [bit 0]
	 * 2 Source pref(DRP performs try.SNK 01) [bit 2:1]
	 * 3 Soft reset(default 0) [bit 3]
	 * 4 Mode slect(DRP start from Try.SNK 00) [bit 5:4]
	 * 5 Debounce time of voltage change on CC pin(default 0) [bit 7:6]
	 */
	reg_val = 0x02;
	ret = hd3ss3220_write_reg(info->i2c, REG_SET, reg_val);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return ret;
	}

	/* CURRENT MODE ADVERTISE 3A [bit 7:6] */
	reg_val = (HOST_CUR_3A << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	ret = hd3ss3220_update_reg(info->i2c, REG_MOD, reg_val, MOD_CURRENT_MODE_ADVERTISE);
	if (ret < 0) {
		pr_err("%s: init REG_MOD fail!\n", __func__);
		return ret;
	}

	return ret;
}

static int hd3ss3220_create_device(struct hd3ss3220_info *info)
{
	struct device_attribute **attrs = usb_typec_attributes;
	struct device_attribute *attr;
	int err;

	pr_debug("%s:\n", __func__);
	info->device_class = class_create(THIS_MODULE, "type-c");
	if (IS_ERR(info->device_class))
		return PTR_ERR(info->device_class);

	info->dev_t = device_create(info->device_class, NULL, 0, NULL, "chip");
	if (IS_ERR(info->dev_t))
		return PTR_ERR(info->dev_t);

	dev_set_drvdata(info->dev_t, info);

	while ((attr = *attrs++)) {
		err = device_create_file(info->dev_t, attr);
		if (err) {
			device_destroy(info->device_class, 0);
			return err;
		}
	}
	return 0;
}

static void hd3ss3220_destroy_device(struct hd3ss3220_info *info)
{
	struct device_attribute **attrs = usb_typec_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(info->dev_t, attr);

	device_destroy(info->device_class, 0);
	class_destroy(info->device_class);
	info->device_class = NULL;
}

static int hd3ss3220_parse_gpio_from_dts(struct device_node *np, struct hd3ss3220_info *info)
{
	int ret;
	int i;

	/* irq_line */
	ret = of_get_named_gpio(np, "hd3ss,irq_gpio", 0);
	if (ret < 0) {
		pr_err("%s: error invalid irq gpio err: %d\n", __func__, ret);
		return ret;
	}
	info->irq_gpio = ret;
	pr_debug("%s: valid irq_gpio number: %d\n", __func__, ret);

	/*
	 * en gpios, for changing state during suspend to reduce power consumption,
	 * as these gpios may already have initial config in the dts files,
	 * getting gpio failure here may not be a critical error.
	 */
	for (i = 0; i < EN_GPIO_MAX; i++) {
		ret = of_get_named_gpio(np, en_gpio_info[i].name, 0);
		if (ret < 0) {
			info->en_gpio[en_gpio_info[i].index] = 0;
			pr_err("%s: error invalid en gpio [%s] err: %d\n",
			       __func__, en_gpio_info[i].name, ret);
		} else {
			info->en_gpio[en_gpio_info[i].index] = ret;
			pr_debug("%s: valid en gpio number: %d\n", __func__, ret);
		}
	}

	return 0;
}

static int hd3ss3220_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hd3ss3220_info *info;
	struct device_node *np = client->dev.of_node;
	int ret, irq;

	pr_debug("%s: enter\n", __func__);
	info = kzalloc(sizeof(struct hd3ss3220_info), GFP_KERNEL);
	info->i2c = client;

	/* initialize pinctrl */
	if (!hd3ss3220_pinctrl_init(info)) {
		ret  = pinctrl_select_state(info->pinctrl, info->cc_int_cfg);
		if (ret) {
			pr_err("%s: error initialize pinctrl\n", __func__);
			goto err_pinctrl;
		} else
			pr_debug("%s: initialize pinctrl success\n", __func__);
	}

	/* get gpio(s) */
	ret = hd3ss3220_parse_gpio_from_dts(np, info);
	if (ret < 0) {
		pr_err("%s: error invalid irq gpio number: %d\n", __func__, ret);
		goto err_pinctrl;
	}

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	/* try initialize device before request irq */
	ret = hd3ss3220_initialization(info);
	if (ret < 0) {
		pr_err("%s: fails to do initialization %d\n", __func__, ret);
		goto err_init_dev;
	}

	/* create device and sysfs nodes */
	ret = hd3ss3220_create_device(info);
	if (ret) {
		pr_err("%s: create device failed\n", __func__);
		goto err_device_create;
	}

	irq = gpio_to_irq(info->irq_gpio);
	if (irq < 0) {
		pr_err("%s: error gpio_to_irq returned %d\n", __func__, irq);
		goto err_request_irq;
	} else {
		pr_debug("%s: requesting IRQ %d\n", __func__, irq);
		client->irq = irq;
	}
	ret = request_threaded_irq(client->irq, NULL, hd3ss3220_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT, "hd3ss3220_irq", info);
	if (ret) {
		dev_err(&client->dev, "error failed to request IRQ\n");
		goto err_request_irq;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
		goto err_enable_irq;
	}

	dev_info(&client->dev, "hd3ss3220 usb type-c ship finish probe\n");
	return 0;

err_enable_irq:
	free_irq(client->irq, NULL);
err_request_irq:
	hd3ss3220_destroy_device(info);
err_device_create:
err_init_dev:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
err_pinctrl:
	kfree(info);
	info = NULL;
	return ret;
}

static int hd3ss3220_remove(struct i2c_client *client)
{
	struct hd3ss3220_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}

	hd3ss3220_destroy_device(info);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);

	kfree(info);
	return 0;
}

static const struct of_device_id hd3ss3220_dt_match[] = {
	{
		.compatible = "ti,hd3ss3220",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, hd3ss3220_dt_match);

static const struct i2c_device_id hd3ss3220_id_table[] = {
	{
		.name = "hd3ss3220",
	},
};

#ifdef TYPEC_PM_OP
static SIMPLE_DEV_PM_OPS(hd3ss3220_dev_pm, hd3ss3220_suspend, hd3ss3220_resume);
#endif

static struct i2c_driver hd3ss3220_i2c_driver = {
	.driver = {
		.name = "hd3ss3220",
		.of_match_table = of_match_ptr(hd3ss3220_dt_match),
#ifdef TYPEC_PM_OP
		.pm = &hd3ss3220_dev_pm,
#endif
	},
	.probe    = hd3ss3220_probe,
	.remove   = hd3ss3220_remove,
	.id_table = hd3ss3220_id_table,
};

static __init int hd3ss3220_i2c_init(void)
{
	return i2c_add_driver(&hd3ss3220_i2c_driver);
}

static __exit void hd3ss3220_i2c_exit(void)
{
	i2c_del_driver(&hd3ss3220_i2c_driver);
}

module_init(hd3ss3220_i2c_init);
module_exit(hd3ss3220_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for HD3SS3220 USB Type-C");
MODULE_LICENSE("GPL v2");
