/*
 * pi5usb302.c -- TI PI5USB302x USB TYPE-C Controller device driver
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

#define TYPEC_PM_OP

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_ID              0x01
#define REG_CTL             0x02
#define REG_INT			    0x03
#define REG_CC_STAT		    0x04
#define REG_COUNT_SUM       REG_CC_STAT

/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_ID (0x01)   R */
#define ID_REG_LEN  0X01

/*    REG_CTL (0x02)   RW */
#define CTL_INT_MASK                    0X01
#define CTL_PORT_SETTING_SHIFT          1
#define CTL_PORT_SETTING                (0x03 << CTL_PORT_SETTING_SHIFT)
#define CTL_CHG_CURRENT_ADV_SHIFT       3
#define CTL_CHG_CURRENT_ADV             (0x03 << CTL_CHG_CURRENT_ADV_SHIFT)
#define CTL_ACCESSORY_DET_SHIFT         5
#define CTL_ACCESSORY_DET               (0x01 << CTL_ACCESSORY_DET_SHIFT)
#define CTL_DUAL2_TRY_SRC_SNK_SHIFT     6
#define CTL_CTL_DUAL2_TRY_SRC_SNK       (0x01 << CTL_DUAL2_TRY_SRC_SNK_SHIFT)
#define CTL_PWER_SAVE_SHIFT             7
#define CTL_PWER_SAVE                   (0x01 << CTL_PWER_SAVE_SHIFT)

/*    REG_INT (0x03)   RU bit 7:0 are cleared when read*/
#define INT_ATTACH_EVENT                0X01
#define INT_DETACH_EVENT_SHIFT          1
#define INT_DETACH_EVENT                (0x01 << INT_DETACH_EVENT_SHIFT)
#define INT_FAULT_OCCUR_SHIFT           2
#define INT_FAULT_OCCUR                 (0x01 << INT_FAULT_OCCUR_SHIFT)
#define INT_OTP_EVENT_SHIFT             3
#define INT_OTP_EVENT                   (0x01 << INT_OTP_EVENT_SHIFT)
#define INT_OVP_EVENT_SHIFT             5
#define INT_OVP_EVENT                   (0x01 << INT_OVP_EVENT_SHIFT)
#define INT_OCP_EVENT_SHIFT             6
#define INT_OCP_EVENT                   (0x01 << INT_OCP_EVENT_SHIFT)
#define INT_FAULT_RECOVER_SHIFT         7
#define INT_FAULT_RECOVER               (0x01 << INT_FAULT_RECOVER_SHIFT)

/*    REG_CC_STAT (0x04)   R */
#define CC_STAT_PLUG_POLAR              0x03
#define CC_STAT_ATTACH_PORT_SHIFT       2
#define CC_STAT_ATTACH_PORT             (0x07 << CC_STAT_ATTACH_PORT_SHIFT)
#define CC_STAT_CHG_CURRENT_DET_SHIFT   5
#define CC_STAT_CHG_CURRENT_DET         (0x03 << CC_STAT_CHG_CURRENT_DET_SHIFT)
#define CC_STAT_VBUS_DET_SHIFT          7
#define CC_STAT_VBUS_DET                (0x01 << CC_STAT_VBUS_DET_SHIFT)


/******************************************************************************
* Register values
******************************************************************************/
/* CTL_PORT_SETTING */
#define CTL_PORT_SETTING_DEVICE     0x00
#define CTL_PORT_SETTING_HOST       0x01
#define CTL_PORT_SETTING_DUAL       0x02
#define CTL_PORT_SETTING_DUAL2      0x03
/* CTL_CURRENT_MODE_ADVERTISE */
#define CTL_CURRENT_MODE_ADVERTISE_DEFAULT      0x00
#define CTL_CURRENT_MODE_ADVERTISE_MID          0x01
#define CTL_CURRENT_MODE_ADVERTISE_HIGH         0x02
/* CC_CURRENT_MODE_DETECT */
#define CC_CURRENT_MODE_DETECT_STANDBY      0x00
#define CC_CURRENT_MODE_DETECT_DEFAULT      0x01
#define CC_CURRENT_MODE_DETECT_MID          0x02
#define CC_CURRENT_MODE_DETECT_HIGH         0x03
/* CC_STAT_ATTACH_PORT */
#define CC_STAT_ATTACH_PORT_STANDBY             0x00
#define CC_STAT_ATTACH_PORT_DEVICE              0x01
#define CC_STAT_ATTACH_PORT_HOST                0x02
#define CC_STAT_ATTACH_PORT_AUDIO_ACCESSORY     0x03
#define CC_STAT_ATTACH_PORT_DEBUG_ACCESSORY     0x04


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
	MODE_UNKNOWN = 0,
	MODE_PORT_PIN,   /*According to hardware port pin*/
	MODE_UFP,
	MODE_DFP,
	MODE_DRP,
	MODE_DRP2
};

/* Type-C Attrs */
struct type_c_parameters {
	enum current_det_type current_det;         /*charging current on UFP*/
	enum accessory_attach_type accessory_attach;     /*if an accessory is attached*/
	enum cable_attach_type active_cable_attach;         /*if an active_cable is attached*/
	enum cable_state_type attach_state;        /*DFP->UFP or UFP->DFP*/
	enum cable_dir_type cable_dir;           /*cc1 or cc2*/
	enum vconn_fault_type vconn_fault;         /*vconn fault*/
	u8 attach_event;
	u8 detach_event;
	u8 otp_event;
	u8 ovp_event;
	u8 ocp_event;
	u8 fault_recover;
	u8 vbus_det;
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
		.name   = "pi5usb,en_cc_gpio",
		.index  = EN_CC_MUX,

	},
	{
		.name   = "pi5usb,re_drv_gpio",
		.index  = EN_RE_DRIVE,

	},
	{
		.name   = "pi5usb,vreg_5v_gpio",
		.index  = EN_VREG_5V,

	},
	{
		.name   = "pi5usb,vreg_3v3_gpio",
		.index  = EN_VREG_3V3,

	},
	/*add more gpio here if need*/
};

struct pi5usb302_info {
	struct i2c_client  *i2c;
	struct device  *dev_t;
	struct mutex  mutex;
	struct class  *device_class;

	struct pinctrl  *pinctrl;
	struct pinctrl_state  *cc_int_cfg;

	int irq_gpio;
	int en_gpio[EN_GPIO_MAX];

	struct type_c_parameters type_c_param;
};

/* i2c operate interfaces */
/* parameter reg is used as a register offset here */
static int pi5usb302_read_bytes(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct pi5usb302_info *info = i2c_get_clientdata(i2c);
	int ret;

	reg = (reg > REG_COUNT_SUM) ? REG_COUNT_SUM : reg;
	mutex_lock(&info->mutex);
	ret = i2c_master_recv(i2c, dest, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	return 0;
}

/* Notice this read register operation may clear some bits in lower offset registers */
static int pi5usb302_read_byte(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct pi5usb302_info *info = i2c_get_clientdata(i2c);
	int ret;
	u8 val[REG_COUNT_SUM] = {0};

	reg = (reg > REG_COUNT_SUM) ? REG_COUNT_SUM : reg;
	mutex_lock(&info->mutex);
	ret = i2c_master_recv(i2c, val, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	*dest = val[reg - 1] & 0xff;
	return 0;
}

static int pi5usb302_update_byte(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct pi5usb302_info *info = i2c_get_clientdata(i2c);
	int ret;
	u8 buf[REG_COUNT_SUM] = {0};
	u8 old_val, new_val;

	reg = (reg > REG_COUNT_SUM) ? REG_COUNT_SUM : reg;
	mutex_lock(&info->mutex);
	ret = i2c_master_recv(i2c, buf, reg);

	if (ret >= 0) {
		/* first, get one byte data from buf */
		old_val = buf[reg - 1] & 0xff;
		/* then, generate one byte data for buf */
		new_val = (val & mask) | (old_val & (~mask));
		/* last, store one byte data into buf */
		buf[reg - 1] = new_val;

		ret = i2c_master_send(i2c, buf, reg);
	}
	mutex_unlock(&info->mutex);
	return ret;
}

/* Config DFP/UFP/DRP mode */
/* e.g #echo 1 >/sys/class/type-c/pi5usb302/mode_select */
static ssize_t mode_select_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = pi5usb302_read_byte(info->i2c, REG_CTL, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & CTL_PORT_SETTING) >> CTL_PORT_SETTING_SHIFT;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t mode_select_store(struct device *dev,
				 struct device_attribute *attr, const char *buf, size_t size)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	int mode;
	u8 value;
	int ret;

	ret = kstrtoint(buf, 0, &mode);
	if (ret != 0)
		return -EINVAL;

	if (mode == MODE_DFP)
		value = CTL_PORT_SETTING_HOST;
	else if (mode == MODE_UFP)
		value = CTL_PORT_SETTING_DEVICE;
	else if (mode == MODE_DRP)
		value = CTL_PORT_SETTING_DUAL;
	else if (mode == MODE_DRP2)
		value = CTL_PORT_SETTING_DUAL2;
	else
		return -EINVAL;

	value = value << CTL_PORT_SETTING_SHIFT;

	ret = pi5usb302_update_byte(info->i2c, REG_CTL, value, CTL_PORT_SETTING);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	return ret;
}


/* Advertise current when act as DFP */
static ssize_t current_advertise_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = pi5usb302_read_byte(info->i2c, REG_CTL, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & CTL_CHG_CURRENT_ADV) >> CTL_CHG_CURRENT_ADV_SHIFT;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t current_advertise_store(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	int mode;
	u8 value;
	int ret;

	ret = kstrtoint(buf, 0, &mode);
	if (ret != 0)
		return -EINVAL;

	if (mode == HOST_CUR_USB)
		value = CTL_CURRENT_MODE_ADVERTISE_DEFAULT;
	else if (mode == HOST_CUR_1P5)
		value = CTL_CURRENT_MODE_ADVERTISE_MID;
	else if (mode == HOST_CUR_3A)
		value = CTL_CURRENT_MODE_ADVERTISE_HIGH;
	else
		return -EINVAL;

	value = value << CTL_CHG_CURRENT_ADV_SHIFT;

	ret = pi5usb302_update_byte(info->i2c, REG_CTL, value, CTL_CHG_CURRENT_ADV);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	return ret;
}

/* Detct current when act as UFP */
static ssize_t current_detect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	u8 value;
	int ret;

	ret = pi5usb302_read_byte(info->i2c, REG_CC_STAT, &value);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return ret;
	}
	value = (value & CC_STAT_CHG_CURRENT_DET) >> CC_STAT_CHG_CURRENT_DET_SHIFT;

	if (value == CC_CURRENT_MODE_DETECT_DEFAULT)
		return snprintf(buf, PAGE_SIZE, "500mA or 900mA\n");
	else if (value == CC_CURRENT_MODE_DETECT_MID)
		return snprintf(buf, PAGE_SIZE, "mid 1P5A\n");
	else if (value == CC_CURRENT_MODE_DETECT_HIGH)
		return snprintf(buf, PAGE_SIZE, "high 3A\n");
	else
		return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t gpio_debug_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "EN_CC_MUX:%d, EN_RE_DRIVE:%d\n",
			gpio_get_value(info->en_gpio[EN_CC_MUX]),
			gpio_get_value(info->en_gpio[EN_RE_DRIVE]));
}

static ssize_t gpio_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
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

static ssize_t dump_regs(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	u8 reg_val[REG_COUNT_SUM] = {0};
	int ret;

	ret = pi5usb302_read_bytes(info->i2c, REG_COUNT_SUM, reg_val);
	if (ret < 0) {
		return snprintf(buf, PAGE_SIZE, "Dump Regs Fail\n");
	} else {
		return snprintf(buf, PAGE_SIZE, "Dump Regs:\n"
			"	REG_ID  : 0x%x\n"
			"	REG_CTL : 0x%x\n"
			"	REG_INT : 0x%x\n"
			"	REG_CC_STAT : 0x%x\n",
			reg_val[REG_ID - 1], reg_val[REG_CTL - 1],
			reg_val[REG_INT - 1], reg_val[REG_CC_STAT - 1]);
	}
}

/* For Emode, read typec chip info */
static ssize_t show_chip_info(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);
	u8 val;
	u8 vid, revision = 0;
	int ret;

	ret = pi5usb302_read_byte(info->i2c, REG_ID, &val);
	if (ret < 0) {
		pr_err("%s: read reg fail!\n", __func__);
		return snprintf(buf, PAGE_SIZE, "USB TypeC chip info:\n"
			"	Manufacture: Pericom\n"
			"	Chip Name  : Pi5usb30213\n"
			"	Vendor ID : Error\n"
			"	Revision ID : Error\n");
	} else {
		vid = val;
		vid = (vid & 0x07);
		revision = ((val & 0xF8) >> 3);
		return snprintf(buf, PAGE_SIZE, "USB TypeC chip info:\n"
			"	Manufacture: Pericom\n"
			"	Chip Name  : Pi5usb30213\n"
			"	Vendor ID : 0x%x\n"
			"	Revision ID : 0x%x\n",
			vid, revision);
	}
}

/**************************************************************************/
#define TYPE_C_ATTR(field, format_string) \
		static ssize_t \
		field ## _show(struct device *dev, struct device_attribute *attr, \
			char *buf) \
		{ \
			struct pi5usb302_info *info = dev_get_drvdata(dev); \
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
static DEVICE_ATTR(dump_reg, S_IRUGO, dump_regs, NULL);

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
	&dev_attr_dump_reg,
	/*end*/
	NULL
};
/******************************************************************************/

static void process_control_register(struct pi5usb302_info *info, u8 status)
{

}

static void process_interrupt_register(struct pi5usb302_info *info, u8 status)
{
	u8 val;
	u8 tmp = status;

	/* check attach state */
	val = (tmp & INT_ATTACH_EVENT);
	info->type_c_param.attach_event = val;

	/* check detach state */
	tmp = status;
	val = ((tmp & INT_DETACH_EVENT) >> INT_DETACH_EVENT_SHIFT);
	info->type_c_param.detach_event = val;

	/* check fault state */
	tmp = status;
	val = ((tmp & INT_FAULT_OCCUR) >> INT_FAULT_OCCUR_SHIFT);
	info->type_c_param.vconn_fault = val;

	/* check otp event */
	tmp = status;
	val = ((tmp & INT_OTP_EVENT) >> INT_OTP_EVENT_SHIFT);
	info->type_c_param.otp_event = val;

	/* check ovp event */
	tmp = status;
	val = ((tmp & INT_OVP_EVENT) >> INT_OVP_EVENT_SHIFT);
	info->type_c_param.ovp_event = val;

	/* check ocp event */
	tmp = status;
	val = ((tmp & INT_OCP_EVENT) >> INT_OCP_EVENT_SHIFT);
	info->type_c_param.ocp_event = val;

	/* check fault recover */
	tmp = status;
	val = ((tmp & INT_FAULT_RECOVER) >> INT_FAULT_RECOVER_SHIFT);
	info->type_c_param.fault_recover = val;
}

static void process_status_register(struct pi5usb302_info *info, u8 status)
{
	u8 val;
	u8 tmp = status;

	/* check plug polarity */
	tmp = status;
	val = (tmp & CC_STAT_PLUG_POLAR);
	info->type_c_param.cable_dir = val;

	/* check attach port state */
	val = ((tmp & CC_STAT_ATTACH_PORT) >> CC_STAT_ATTACH_PORT_SHIFT);
	info->type_c_param.attach_state = val;
	/* 000 = Standby */
	/* 001 = Device */
	/* 010 = Host */
	/* 011 = Audio Accessory */
	/* 100 = Debug Accessory */
	if (info->type_c_param.attach_state == CABLE_STATE_AS_DFP ||
	    info->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		val = (HOST_CUR_USB << CTL_CHG_CURRENT_ADV_SHIFT);
	} else {
		val = (HOST_CUR_3A << CTL_CHG_CURRENT_ADV_SHIFT);
	}
	pi5usb302_update_byte(info->i2c, REG_CTL, val, CTL_CHG_CURRENT_ADV);

	/* check current detect */
	val = ((tmp & CC_STAT_CHG_CURRENT_DET) >> CC_STAT_CHG_CURRENT_DET_SHIFT);
	info->type_c_param.current_det = val;

	/* check vbus detect */
	val = ((tmp & CC_STAT_VBUS_DET) >> CC_STAT_VBUS_DET_SHIFT);
	info->type_c_param.vbus_det = val;
}

static irqreturn_t pi5usb302_irq_thread(int irq, void *handle)
{
	struct pi5usb302_info *info = (struct pi5usb302_info *)handle;
	u8 reg_val[REG_COUNT_SUM] = {0};
	int ret;

	pr_debug("%s enter\n", __func__);

	ret = pi5usb302_read_bytes(info->i2c, REG_COUNT_SUM, reg_val);
	if (ret)
		goto done;

	pr_debug("%s control_register:0x%x\n", __func__, reg_val[REG_CTL - 1]);
	process_control_register(info, reg_val[REG_CTL - 1]);

	pr_debug("%s interrupt_register:0x%x\n", __func__, reg_val[REG_INT - 1]);
	process_interrupt_register(info, reg_val[REG_INT - 1]);

	pr_debug("%s status_register:0x%x\n", __func__, reg_val[REG_CC_STAT - 1]);
	process_status_register(info, reg_val[REG_CC_STAT - 1]);

done:
	return IRQ_HANDLED;
}

#ifdef TYPEC_PM_OP
static int  pi5usb302_suspend(struct device *dev)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);

	pr_info("%s enter\n", __func__);
	disable_irq_wake(info->i2c->irq);
	disable_irq_nosync(info->i2c->irq);

	if (info->en_gpio[EN_CC_MUX])
		gpio_direction_output(info->en_gpio[EN_CC_MUX], 0);
	if (info->en_gpio[EN_RE_DRIVE])
		gpio_direction_output(info->en_gpio[EN_RE_DRIVE], 0);

	if (info->en_gpio[EN_VREG_5V])
		gpio_direction_output(info->en_gpio[EN_VREG_5V], 0);
	if (info->en_gpio[EN_VREG_3V3])
		gpio_direction_output(info->en_gpio[EN_VREG_3V3], 0);

	pr_debug("%s gpio value:%d,%d,%d,%d\n", __func__,
		 gpio_get_value(info->en_gpio[EN_CC_MUX]),
		 gpio_get_value(info->en_gpio[EN_RE_DRIVE]),
		 gpio_get_value(info->en_gpio[EN_VREG_5V]),
		 gpio_get_value(info->en_gpio[EN_VREG_3V3]));

	return 0;
}

static int  pi5usb302_resume(struct device *dev)
{
	struct pi5usb302_info *info = dev_get_drvdata(dev);

	pr_info("%s enter\n", __func__);

	if (info->en_gpio[EN_VREG_5V])
		gpio_direction_output(info->en_gpio[EN_VREG_5V], 1);
	if (info->en_gpio[EN_VREG_3V3])
		gpio_direction_output(info->en_gpio[EN_VREG_3V3], 1);

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

static int pi5usb302_pinctrl_init(struct pi5usb302_info *info)
{
	struct pinctrl_state *set_state;

	info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		pr_err("%s: pinctrl not defined\n", __func__);
		return PTR_ERR(info->pinctrl);
	}

	pr_debug("%s: doing pinctrl\n", __func__);
	set_state = pinctrl_lookup_state(info->pinctrl, "pi5usb302_pin_cfg");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("%s: pinctrl lookup failed\n", __func__);
		info->pinctrl = NULL;
		return PTR_ERR(set_state);
	}
	info->cc_int_cfg = set_state;

	/* get more pins conig here, if need */
	return 0;
}

static int pi5usb302_initialization(struct pi5usb302_info *info)
{
	int ret = 0;
	u8 reg_val;
	/* do initialization here, before enable irq,
	 * clear irq,
	 * config DRP/UFP/DFP mode,
	 * and etc..
	 */
	pr_debug("%s enter\n", __func__);
	ret = pi5usb302_read_byte(info->i2c, REG_ID, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_ID fail!\n", __func__);
		return ret;
	}

	/* 1 Enable interrupt 0 [bit 0]
	 * 2 Dual role 2 with try.snc supported 11 [bit 2:1]
	 * 3 Chg current adv default 10 [bit 4:3]
	 * 4 Enable accessory detect on device mode 1 [bit 5]
	 * 5 Enable try.snc 1 [bit 6]
	 * 6 Disable power saving 0 [bit 7]
	 */
	reg_val = 0x76;
	ret = pi5usb302_update_byte(info->i2c, REG_CTL, reg_val, 0xFF);
	if (ret < 0) {
		pr_err("%s: init REG_CTL fail!\n", __func__);
		return ret;
	}

	return ret;
}

static int pi5usb302_create_device(struct pi5usb302_info *info)
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

static void pi5usb302_destroy_device(struct pi5usb302_info *info)
{
	struct device_attribute **attrs = usb_typec_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(info->dev_t, attr);

	device_destroy(info->device_class, 0);
	class_destroy(info->device_class);
	info->device_class = NULL;
}

static int pi5usb302_parse_gpio_from_dts(struct device_node *np, struct pi5usb302_info *info)
{
	int ret;
	int i;

	/* irq_line */
	ret = of_get_named_gpio(np, "pi5usb,irq_gpio", 0);
	if (ret < 0) {
		pr_err("%s: error invalid irq gpio err: %d\n", __func__, ret);
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

static int pi5usb302_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pi5usb302_info *info;
	struct device_node *np = client->dev.of_node;
	int ret, irq;

	pr_debug("%s: enter\n", __func__);
	info = kzalloc(sizeof(struct pi5usb302_info), GFP_KERNEL);
	info->i2c = client;

	/* initialize pinctrl */
	if (!pi5usb302_pinctrl_init(info)) {
		ret  = pinctrl_select_state(info->pinctrl, info->cc_int_cfg);
		if (ret) {
			pr_err("%s: error initialize pinctrl\n", __func__);
			goto err_pinctrl;
		} else
			pr_debug("%s: initialize pinctrl success\n", __func__);
	}

	/* get gpio(s) */
	ret = pi5usb302_parse_gpio_from_dts(np, info);
	if (ret < 0) {
		pr_err("%s: error invalid irq gpio number: %d\n", __func__, ret);
		goto err_pinctrl;
	}

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	/* try initialize device before request irq */
	ret = pi5usb302_initialization(info);
	if (ret < 0) {
		pr_err("%s: fails to do initialization %d\n", __func__, ret);
		goto err_init_dev;
	}

	/* create device and sysfs nodes */
	ret = pi5usb302_create_device(info);
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
	ret = request_threaded_irq(client->irq, NULL, pi5usb302_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT, "pi5usb302_irq", info);
	if (ret) {
		dev_err(&client->dev, "error failed to request IRQ\n");
		goto err_request_irq;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
		goto err_enable_irq;
	}

	dev_info(&client->dev, "pi5usb302 usb type-c ship finish probe\n");
	return 0;

err_enable_irq:
	free_irq(client->irq, NULL);
err_request_irq:
	pi5usb302_destroy_device(info);
err_device_create:
err_init_dev:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
err_pinctrl:
	kfree(info);
	info = NULL;
	return ret;
}

static int pi5usb302_remove(struct i2c_client *client)
{
	struct pi5usb302_info *info = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}

	pi5usb302_destroy_device(info);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);

	kfree(info);
	return 0;
}

static const struct of_device_id pi5usb302_dt_match[] = {
	{
		.compatible = "ti,pi5usb302",
	},
	{},
};
MODULE_DEVICE_TABLE(of, pi5usb302_dt_match);

static const struct i2c_device_id pi5usb302_id_table[] = {
	{
		.name = "pi5usb302",
	},
};

#ifdef TYPEC_PM_OP
static SIMPLE_DEV_PM_OPS(pi5usb302_dev_pm, pi5usb302_suspend, pi5usb302_resume);
#endif

static struct i2c_driver pi5usb302_i2c_driver = {
	.driver = {
		.name = "pi5usb302",
		.of_match_table = of_match_ptr(pi5usb302_dt_match),
#ifdef TYPEC_PM_OP
		.pm = &pi5usb302_dev_pm,
#endif
	},
	.probe    = pi5usb302_probe,
	.remove   = pi5usb302_remove,
	.id_table = pi5usb302_id_table,
};

static __init int pi5usb302_i2c_init(void)
{
	return i2c_add_driver(&pi5usb302_i2c_driver);
}

static __exit void pi5usb302_i2c_exit(void)
{
	i2c_del_driver(&pi5usb302_i2c_driver);
}

module_init(pi5usb302_i2c_init);
module_exit(pi5usb302_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for pi5usb302 USB Type-C");
MODULE_LICENSE("GPL v2");
