/*
 * rgb_bh1745.c - Linux kernel modules for ambient light + proximity sensor
 *
 * Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 * Copyright (C) 2012 Avago Technologies
 * Copyright (C) 2013 LGE Inc.
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <rohm_bh1745.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>

#define JUDEG_COEFF   (1000)
#define TRANS  2

#define BH1745_REG_LEN 0x0a

static unsigned short bh1745_atime[6] = { 160, 320, 640, 1280, 2560, 5120 };
static unsigned char bh1745_again[3] = { 1, 2, 16 };

static long cofficient_judge = 163;
static long cofficient_red[2] = { 124, 122 };
static long cofficient_green[2] = { 363, 210 };
static long cofficient_blue[2] = { 0, 0 };

#define SAVE_LUX_NUM 5
int lux[SAVE_LUX_NUM];
int curr_pos = -1;

#define BH1745_DRV_NAME "bh1745"
#define DRIVER_VERSION		"1.0.0"

#define BH1745_I2C_RETRY_COUNT	3	/* Number of times to retry i2c */

/*wait more time to try read or write to avoid potencial risk*/
/* Timeout between retry (miliseconds) */
#define BH1745_I2C_RETRY_TIMEOUT	3

#define BH1745_I2C_BYTE 0
#define BH1745_I2C_WORD 1
/*dynamic debug mask to control log print,
  *you can echo value to rgb_bh1745_debug to control
  */
static int rgb_bh1745_debug_mask;

#define MAX_DATA_LENGTH 256

#define BH1745_ERR(x...) do {\
	if (rgb_bh1745_debug_mask >= 0) \
		pr_err(x);\
	} while (0)

#define BH1745_INFO(x...) do {\
	if (rgb_bh1745_debug_mask >= 1) \
		pr_err(x);\
	} while (0)
#define BH1745_FLOW(x...) do {\
	if (rgb_bh1745_debug_mask >= 2) \
		pr_err(x);\
	} while (0)

struct rgb_bh1745_rgb_data {
	int red;
	int green;
	int blue;
	int clear;
	int lx;
	int color_temp;
};

struct rgb_bh1745_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation */
	struct mutex update_lock;
	/*to protect only one thread to control the device register */
	struct mutex single_lock;
	struct work_struct als_dwork;	/* for ALS polling */

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;
	int regulator_vdd_vmin;
	int regulator_vdd_vmax;
	int regulator_vio_vmin;
	int regulator_vio_vmax;

	/* pinctrl data */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;

	struct rgb_bh1745_rgb_data rgb_data;

	struct hrtimer timer;
	unsigned int enable;
	unsigned int pers;
	unsigned int control;
	unsigned int measure_time;

	unsigned int irq_control;

	/* control flag from HAL */
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend */
	unsigned int enable_als_state;

	/* ALS parameters */
	int als_prev_lux;/* to store previous lux value */
	/* needed for light sensor polling : micro-second (us) */
	unsigned int als_poll_delay;
	bool device_exist;

	int (*init)(struct rgb_bh1745_data *data);
	void (*exit)(struct rgb_bh1745_data *data);
	int (*power_onoff)(bool, struct rgb_bh1745_data *data);
};

static struct i2c_client *bh1745_i2c_client;

static struct workqueue_struct *rgb_bh1745_workqueue;

/*init the register of device function for
  *probe and every time the chip is powered on
  */
static int rgb_bh1745_init_client(struct i2c_client *client);
/*we use the unified the function for i2c write and read operation*/
static int rgb_bh1745_i2c_write(struct i2c_client *client, u8 reg,
	u16 value, bool flag)
{
	int err, loop;

	struct rgb_bh1745_data *data = i2c_get_clientdata(client);

	loop = BH1745_I2C_RETRY_COUNT;
	/*we give three times to repeat
	  *the i2c operation if i2c errors happen
	  */
	while (loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,
		  *1 is i2c_smbus_write_word_data
		  */
		if (flag == BH1745_I2C_BYTE) {
			err = i2c_smbus_write_byte_data(client, reg,
						(u8) value);
		} else if (flag == BH1745_I2C_WORD) {
			err = i2c_smbus_write_word_data(client, reg, value);
		} else {
			BH1745_ERR(
				"%s,line %d:attention: i2c write wrong flag\n",
				__func__, __LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if (err < 0) {
			loop--;
			msleep(BH1745_I2C_RETRY_TIMEOUT);
		} else
			break;
	}
	/*after three times,we print the register and regulator value */
	if (loop == 0) {
		BH1745_ERR(
			"%s,line %d:attention:i2c write err = %d\n",
			__func__, __LINE__, err);
	}

	return err;
}

static int rgb_bh1745_i2c_read(struct i2c_client *client, u8 reg,
	bool flag)
{
	int err, loop;

	struct rgb_bh1745_data *data = i2c_get_clientdata(client);

	loop = BH1745_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation
	  *if i2c errors happen
	  */
	while (loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,
		  *1 is i2c_smbus_read_word_data */
		if (flag == BH1745_I2C_BYTE) {
			err = i2c_smbus_read_byte_data(client, reg);
		} else if (flag == BH1745_I2C_WORD) {
			err = i2c_smbus_read_word_data(client, reg);
		} else {
			BH1745_ERR(
				"%s,line %d:attention: i2c read wrong flag\n",
				__func__, __LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if (err < 0) {
			loop--;
			msleep(BH1745_I2C_RETRY_TIMEOUT);
		} else
			break;
	}
	/*after three times,we print the register
	  *and regulator value
	  */
	if (loop == 0) {
		BH1745_ERR(
			"%s,line %d:attention: i2c read err = %d,reg=0x%x\n",
			__func__, __LINE__, err, reg);
	}

	return err;
}

/************************************************************
 * NAME       : rgb_driver_reset
 * FUNCTION   : reset BH1745 register
 * REMARKS    :
 ************************************************************/
static int rgb_bh1745_driver_reset(struct i2c_client *client)
{
	int ret;

	/* set soft ware reset */
	ret = rgb_bh1745_i2c_write(client, BH1745_SYSTEMCONTROL,
		(SW_RESET | INT_RESET), BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR(
			"%s,line %d: i2c error,rgb_bh1745_driver_reset fail %d\n",
			__func__, __LINE__, ret);
		return ret;
	}
	BH1745_FLOW("%s,line %d:rgb_bh1745 reset\n",
			__func__, __LINE__);
	/*wait for device reset success */
	usleep_range(800, 1000);
	return ret;
}

static int rgb_bh1745_set_enable(struct i2c_client *client, int enable)
{
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_MODECONTROL2,
		enable, BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR("%s,line %d:i2c error,enable = %d\n",
			__func__, __LINE__, enable);
		return ret;
	}
	BH1745_FLOW("%s,line %d:rgb_bh1745 enable = %d\n",
		__func__, __LINE__, enable);
	return ret;
}

static int rgb_bh1745_set_pers(struct i2c_client *client, int pers)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_PERSISTENCE,
		pers, BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR("%s,line %d:i2c error,pers = %d\n",
			__func__, __LINE__, pers);
		return ret;
	}

	data->pers = pers;
	BH1745_FLOW("%s,line %d:rgb_bh1745 pers = %d\n",
		__func__, __LINE__, pers);
	return ret;
}

static int rgb_bh1745_set_interrupt(struct i2c_client *client,
	int irq_control)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_INTERRUPT,
		irq_control, BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR("%s,line %d:i2c error,irq_control = %d\n",
			__func__, __LINE__, irq_control);
		return ret;
	}

	data->irq_control = irq_control;
	BH1745_FLOW("%s,line %d:rgb_bh1745 irq_control = %d\n",
		__func__, __LINE__, irq_control);
	return ret;
}

static int rgb_bh1745_set_control(struct i2c_client *client, int control)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_MODECONTROL3,
		control, BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR("%s,line %d:i2c error,control = %d\n",
			__func__, __LINE__, control);
		return ret;
	}

	data->control = control;
	BH1745_FLOW("%s,line %d:rgb_bh1745 control = %d\n",
		__func__, __LINE__, control);
	return ret;
}

static int rgb_bh1745_set_measure_time(struct i2c_client *client,
	int measure_time)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_MODECONTROL1,
		measure_time, BH1745_I2C_BYTE);
	if (ret < 0) {
		BH1745_ERR("%s,line %d:i2c error,measure_time = %d\n",
			__func__, __LINE__, measure_time);
		return ret;
	}

	data->measure_time = measure_time;
	BH1745_FLOW("%s,line %d:rgb_bh1745 measure_time = %d\n",
		__func__, __LINE__, measure_time);
	return ret;
}
static void save_lux_info(int curr_lux)
{
	int pos;
	int num = SAVE_LUX_NUM;

	pos = (curr_pos + 1) % num;
	lux[pos] = curr_lux;
	curr_pos = pos;
}

/*************************************************************
 * FUNCTION   : Calculate lux
 * REMARKS    :
 * INPUT      : data  : each data value from IC (Red, Green, Blue, Clear)
 *            : gain  : gain's value of when sensor is gotten the data
 *            : itime : time's value of when sensor is getten data (unit is ms)
 * RETURN     : Lux value
 *************************************************************/
static int rgb_bh1745_calc_lx(struct i2c_client *client,
	struct rgb_bh1745_rgb_data *data,
	unsigned char gain, unsigned short itime)
{
	long long int lx;
	long long int lx_tmp;

	if ((data->red >= BH1745_RGB_DATA_MAX)
		|| (data->green >= BH1745_RGB_DATA_MAX)
		|| (data->blue >= BH1745_RGB_DATA_MAX)) {
		lx = BH1745_LUX_MAX;
		return lx;
	}

	if (data->green < 1) {
		lx_tmp = 0;
	} else if ((data->clear * JUDEG_COEFF) <
		(cofficient_judge * data->green)) {
		lx_tmp =
			data->green * cofficient_green[0] +
			data->red * cofficient_red[0];
		BH1745_FLOW("%s,line %d:lx_temp 1: %lld\n",
					__func__, __LINE__, lx_tmp);
	} else {
		lx_tmp =
			data->green * cofficient_green[1] +
			data->red * cofficient_red[1];
		BH1745_FLOW("%s,line %d:lx_temp 1: %lld\n",
					__func__, __LINE__, lx_tmp);
	}

	BH1745_FLOW("%s,line %d: cal lx_tmp is %lld\n",
		__func__, __LINE__, lx_tmp);

	if (lx_tmp < 0)
		lx_tmp = 0;

	lx = lx_tmp * TRANS * 160 / gain / itime / 1000;

	BH1745_INFO("%s,line %d:gain = %d, itime=%d, lux = %lld\n",
		__func__, __LINE__, gain, itime, lx);
	BH1745_FLOW("%s,line %d:judge = %ld\n",
				__func__, __LINE__, cofficient_judge);
	BH1745_FLOW("%s,line %d: r[0] = %ld, r[1]=%ld\n",
				__func__, __LINE__,
				cofficient_red[0], cofficient_red[1]);
	BH1745_FLOW("%s,line %d: g[0] = %ld, g[1]=%ld\n",
				__func__, __LINE__, cofficient_green[0],
				cofficient_green[1]);
	BH1745_FLOW("%s,line %d: b[0]=%ld,b[1]=%ld\n",
				__func__, __LINE__, cofficient_blue[0],
				cofficient_blue[1]);

	return (int)lx;
}

/* ALS polling routine */
static void rgb_bh1745_als_polling_work_handler(struct work_struct *work)
{
	struct rgb_bh1745_data *data =
		container_of(work, struct rgb_bh1745_data, als_dwork);
	struct i2c_client *client = data->client;
	int luxValue = 0;
	unsigned char gain = 0;
	unsigned short time = 0;
	int tmp = 0;

	unsigned char lux_is_valid = 1;
	int ret;

	ret = rgb_bh1745_i2c_read(client, BH1745_MODECONTROL2,
		BH1745_I2C_WORD);
	if (ret < 0) {
		BH1745_ERR(
			"%s,line %d i2c read fail, read BH1745_MODECONTROL2 error\n",
			__func__, __LINE__);
		goto restart_timer;
	}
	ret &= MODECONTROL2_VALID;
	if (ret) {
		data->rgb_data.red
			= rgb_bh1745_i2c_read(client, BH1745_RED_DATA_LSB,
			BH1745_I2C_WORD);
		data->rgb_data.green
			= rgb_bh1745_i2c_read(client, BH1745_GREEN_DATA_LSB,
			BH1745_I2C_WORD);
		data->rgb_data.blue
			= rgb_bh1745_i2c_read(client, BH1745_BLUE_DATA_LSB,
			BH1745_I2C_WORD);
		data->rgb_data.clear
			= rgb_bh1745_i2c_read(client, BH1745_CLEAR_DATA_LSB,
			BH1745_I2C_WORD);
	} else {
		BH1745_FLOW("%s,line %d the data is not update\n",
			__func__, __LINE__);
		goto restart_timer;
	}

	BH1745_FLOW("%s,line %d:red(%d); green(%d);blue(%d);clear(%d)\n",
		__func__, __LINE__, data->rgb_data.red, data->rgb_data.green,
		data->rgb_data.blue, data->rgb_data.clear);

	if ((data->rgb_data.red < 0) || (data->rgb_data.green < 0) ||
		(data->rgb_data.blue < 0) || (data->rgb_data.clear < 0)) {
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
		BH1745_ERR(
		     "%s,line %d i2c read fail, red:%d, green:%d,blue:%d, clear:%d\n",
			__func__, __LINE__,
			data->rgb_data.red, data->rgb_data.green,
			data->rgb_data.blue, data->rgb_data.clear);
	} else {
		tmp = rgb_bh1745_i2c_read(client, BH1745_MODECONTROL1,
			BH1745_I2C_BYTE);
		if (tmp < 0) {
			BH1745_ERR("%s:%d i2c read error tmp = %d\n",
				__func__, __LINE__, tmp);
			tmp = 0;
		}
		tmp = tmp & 0x7;
		time = bh1745_atime[tmp];
		tmp = rgb_bh1745_i2c_read(client, BH1745_MODECONTROL2,
			BH1745_I2C_BYTE);
		if (tmp < 0) {
			BH1745_ERR("%s:%d i2c read error tmp = %d\n",
				__func__, __LINE__, tmp);
			tmp = 0;
		}
		tmp = tmp & 0x3;
		gain = bh1745_again[tmp];
		luxValue = rgb_bh1745_calc_lx(client,
					&(data->rgb_data), gain, time);
	}

	if (luxValue >= 0) {
		luxValue =
			luxValue < BH1745_LUX_MAX ? luxValue : BH1745_LUX_MAX;
		data->als_prev_lux = luxValue;
	} else {
		BH1745_ERR(
			"%s:%d cal lux error, luxValue = %d lux_is_valid =%d\n",
			__func__, __LINE__, luxValue, lux_is_valid);
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	save_lux_info(luxValue);
	/* restart timer */
restart_timer:
	hrtimer_start(&data->timer,
		ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
}

static enum hrtimer_restart rgb_bh1745_als_timer_func(struct hrtimer
	*timer)
{
	struct rgb_bh1745_data *data
		= container_of(timer, struct rgb_bh1745_data, timer);

	queue_work(rgb_bh1745_workqueue, &data->als_dwork);
	return HRTIMER_NORESTART;
}

/*
 * IOCTL support
 */

static int rgb_bh1745_enable_als_sensor(struct i2c_client *client, int val)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	BH1745_FLOW("%s,line %d:enable als val=%d\n", __func__, __LINE__, val);

	mutex_lock(&data->single_lock);
	if (val == 1) {
		/* turn on light  sensor */
		BH1745_FLOW("%s,line %d:judge = %ld\n",
					__func__, __LINE__, cofficient_judge);
		BH1745_FLOW("%s,line %d: r[0] = %ld, r[1]=%ld\n",
					__func__, __LINE__,
					cofficient_red[0], cofficient_red[1]);
		BH1745_FLOW("%s,line %d: g[0] = %ld, g[1]=%ld\n",
					__func__, __LINE__, cofficient_green[0],
					cofficient_green[1]);
		BH1745_FLOW("%s,line %d: b[0]=%ld,b[1]=%ld\n",
					__func__, __LINE__, cofficient_blue[0],
					cofficient_blue[1]);
		if (data->enable_als_sensor == 0) {
			/* Power on and initialize the device */
			if (data->power_onoff)
				data->power_onoff(true, data);

			ret = rgb_bh1745_init_client(client);
			if (ret) {
				BH1745_ERR(
					"%s:line:%d,Failed to init rgb_bh1745\n",
					__func__, __LINE__);
				mutex_unlock(&data->single_lock);
				return ret;
			}
			data->enable_als_sensor = 1;
			data->enable = (data->enable) | RGBC_EN_ON;
			rgb_bh1745_set_enable(client, data->enable);
			BH1745_INFO(
				"%s: line:%d enable als sensor,data->enable=0x%x\n",
				__func__, __LINE__, data->enable);
			/* enable als sensor, start data report hrtimer */
			hrtimer_start(&data->timer,
				ktime_set(0, data->als_poll_delay * 1000000),
				HRTIMER_MODE_REL);

		}
	} else {
		/*
		   * turn off light sensor
		 */
		if (data->enable_als_sensor == 1) {
			data->enable_als_sensor = 0;
			data->enable = ADC_GAIN_X2 | RGBC_EN_OFF;
			rgb_bh1745_set_enable(client, data->enable);

			BH1745_INFO(
				"%s: line:%d,disable rgb bh1745 als sensor,data->enable = 0x%x\n",
				__func__, __LINE__, data->enable);
			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->als_dwork);
			/*avoid hrtimer restart in data->als_dwork */
			hrtimer_cancel(&data->timer);
		}

	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) && (data->power_onoff))
		data->power_onoff(false, data);

	mutex_unlock(&data->single_lock);
	BH1745_FLOW("%s: line:%d,enable als sensor success\n",
		__func__, __LINE__);
	return 0;
}

static ssize_t rgb_bh1745_show_rgbc_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(bh1745_i2c_client);
	uint16_t len = 0;

	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d %d %d %d\n",
		data->rgb_data.red, data->rgb_data.green,
		data->rgb_data.blue, data->rgb_data.clear);
	return len;
}

static ssize_t rgb_bh1745_show_red_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int red_data;
	uint16_t len = 0;

	red_data = rgb_bh1745_i2c_read(bh1745_i2c_client,
		BH1745_RED_DATA_LSB, BH1745_I2C_WORD);
	if (red_data < 0) {
		BH1745_ERR("%s,line %d:read green_data failed\n",
			__func__, __LINE__);
	}
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n", red_data);
	return len;
}

static ssize_t rgb_bh1745_show_green_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int green_data;
	uint16_t len = 0;

	green_data = rgb_bh1745_i2c_read(bh1745_i2c_client,
		BH1745_GREEN_DATA_LSB, BH1745_I2C_WORD);
	if (green_data < 0) {
		BH1745_ERR("%s,line %d:read green_data failed\n",
			__func__, __LINE__);
	}
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n", green_data);
	return len;
}

static ssize_t rgb_bh1745_show_blue_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int blue_data;
	uint16_t len = 0;

	blue_data = rgb_bh1745_i2c_read(bh1745_i2c_client,
		BH1745_BLUE_DATA_LSB, BH1745_I2C_WORD);
	if (blue_data < 0) {
		BH1745_ERR("%s,line %d:read blue_data failed\n", __func__,
			__LINE__);
	}
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n", blue_data);
	return len;
}

static ssize_t rgb_bh1745_show_clear_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int clear_data;
	uint16_t len = 0;

	clear_data = rgb_bh1745_i2c_read(bh1745_i2c_client,
		BH1745_CLEAR_DATA_LSB, BH1745_I2C_WORD);
	if (clear_data < 0) {
		BH1745_ERR("%s,line %d:read clear_data failed\n",
			__func__, __LINE__);
	}
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n", clear_data);
	return len;
}

static ssize_t rgb_bh1745_show_lux_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	struct rgb_bh1745_data *data = i2c_get_clientdata(bh1745_i2c_client);

	BH1745_INFO("%s,line %d:lux=%d\n",
		__func__, __LINE__, data->als_prev_lux);
	len +=
		snprintf(buf + len, MAX_DATA_LENGTH, "%d\n",
				data->als_prev_lux);
	return len;
}
static ssize_t rgb_bh1745_show_lux_avg_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	int i = 0;
	int sum = 0, lux_avg = 0;

	for (i = 0; i < SAVE_LUX_NUM; i++)
		sum += lux[i];
	lux_avg = sum / SAVE_LUX_NUM;
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n", lux_avg);
	return len;
}

static ssize_t rgb_bh1745_show_sensor_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;
	struct rgb_bh1745_data *data = i2c_get_clientdata(bh1745_i2c_client);

	BH1745_ERR("%s,line %d:enable_sensor=%d\n",
		__func__, __LINE__, data->enable_als_sensor);
	len += snprintf(buf + len, MAX_DATA_LENGTH, "%d\n",
		data->enable_als_sensor);
	return len;
}

static ssize_t rgb_bh1745_set_sensor_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long enable = 0;
	int rc = 0;

	rc = kstrtol(buf, 10, &enable);
	BH1745_ERR("%s,line %d:enable=%ld\n", __func__, __LINE__, enable);

	rgb_bh1745_enable_als_sensor(bh1745_i2c_client, enable);

	return count;
}

static ssize_t rgb_bh1745_show_sensor_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;

	len += snprintf(buf + len, MAX_DATA_LENGTH, "%s\n",
		bh1745_i2c_client->name);
	return len;
}

static ssize_t rgb_bh1745_show_cofficient_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint16_t len = 0;

	len += snprintf(buf + len, MAX_DATA_LENGTH, "%ld %ld %ld %ld %ld\n",
		cofficient_judge, cofficient_red[0], cofficient_red[1],
		cofficient_green[0], cofficient_green[1]);

	return len;
}

static ssize_t rgb_bh1745_write_cofficient_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf != NULL)
		sscanf(buf, "%ld %ld %ld %ld %ld", &cofficient_judge,
			&cofficient_red[0], &cofficient_red[1],
			&cofficient_green[0], &cofficient_green[1]);
	return count;
}

/*
* show all registers' value to userspace
*/
static ssize_t rgb_bh1745_print_reg_buf(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	char reg[BH1745_REG_LEN];

	/* read all register value and print to user */
	for (i = 0; i < BH1745_REG_LEN; i++) {
		reg[i] = rgb_bh1745_i2c_read(bh1745_i2c_client,
			(0x50 + i), BH1745_I2C_BYTE);
		if (reg[i] < 0) {
			BH1745_ERR("%s,line %d:read %d reg failed\n",
				__func__, __LINE__, i);
			return reg[i];
		}
	}

	return snprintf(buf, 512,
		"reg[0x0~0x8]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
		"reg[0x09~0x11]0x%2x\n",
		reg[0x00], reg[0x01], reg[0x02], reg[0x03],
		reg[0x04], reg[0x05], reg[0x06], reg[0x07],
		reg[0x08], reg[0x09]);
}

/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t rgb_bh1745_write_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val_len_max = 4;
	char *input_str = NULL;
	char reg_addr_str[10] = { '\0' };
	char reg_val_str[10] = { '\0' };
	long reg_addr, reg_val;
	int addr_length = 0, value_length = 0, buf_len = 0, ret = -1;
	char *strtok = NULL;

	buf_len = strlen(buf);
	input_str = kzalloc(buf_len, GFP_KERNEL);
	if (!input_str) {
		BH1745_ERR("%s:kmalloc fail!\n", __func__);
		return -ENOMEM;
	}

	snprintf(input_str, 10, "%s", buf);
	/*Split the string when encounter "|",
	   *for example "0x08|0x12" will be splited "0x18" "0x12"
	 */
	strtok = strsep(&input_str, "|");
	if (strtok != NULL) {
		addr_length = strlen(strtok);
		memcpy(reg_addr_str, strtok,
		    (addr_length > val_len_max) ? val_len_max : addr_length);
	} else {
		BH1745_ERR("%s: buf name Invalid:%s", __func__, buf);
		goto parse_fail_exit;
	}
	strtok = strsep(&input_str, "|");
	if (strtok != NULL) {
		value_length = strlen(strtok);
		memcpy(reg_val_str, strtok,
			((value_length >
				(val_len_max)) ? (val_len_max) : value_length));
	} else {
		BH1745_ERR("%s: buf value Invalid:%s", __func__, buf);
		goto parse_fail_exit;
	}
	/* transform string to long int */
	ret = kstrtol(reg_addr_str, 16, &reg_addr);
	if (ret)
		goto parse_fail_exit;

	ret = kstrtol(reg_val_str, 16, &reg_val);
	if (ret)
		goto parse_fail_exit;

	/* write the parsed value in the register */
	ret = rgb_bh1745_i2c_write(client, (char)reg_addr,
		(char)reg_val, BH1745_I2C_BYTE);
	if (ret < 0)
		goto parse_fail_exit;

	return count;

parse_fail_exit:
	kfree(input_str);

	return ret;
}

static DEVICE_ATTR(sensor_name, S_IRUGO, rgb_bh1745_show_sensor_name,
	NULL);
static DEVICE_ATTR(sensor_enable, S_IRUGO | S_IWUSR | S_IWGRP,
	rgb_bh1745_show_sensor_enable, rgb_bh1745_set_sensor_enable);
static DEVICE_ATTR(rgbc_data, S_IRUGO, rgb_bh1745_show_rgbc_data, NULL);
static DEVICE_ATTR(red_data, S_IRUGO, rgb_bh1745_show_red_data, NULL);
static DEVICE_ATTR(green_data, S_IRUGO, rgb_bh1745_show_green_data, NULL);
static DEVICE_ATTR(blue_data, S_IRUGO, rgb_bh1745_show_blue_data, NULL);
static DEVICE_ATTR(clear_data, S_IRUGO, rgb_bh1745_show_clear_data, NULL);
static DEVICE_ATTR(lux_data, S_IRUGO, rgb_bh1745_show_lux_data, NULL);
static DEVICE_ATTR(lux_avg_data, S_IRUGO, rgb_bh1745_show_lux_avg_data,
	NULL);
static DEVICE_ATTR(dump_reg, S_IRUGO | S_IWUSR | S_IWGRP,
	rgb_bh1745_print_reg_buf, rgb_bh1745_write_reg);
static DEVICE_ATTR(cofficient_data, S_IRUGO | S_IWUSR,
	rgb_bh1745_show_cofficient_data, rgb_bh1745_write_cofficient_data);

static struct device device_rgb_sensor;
static struct bus_type rgb_sensor_subsys = {
	.name = "rgbsensor",
	.dev_name = "rgbsensor",
};

const struct device_attribute *rgb_sensor_dev_attrs[] = {
	&dev_attr_sensor_name,
	&dev_attr_sensor_enable,
	&dev_attr_rgbc_data,
	&dev_attr_red_data,
	&dev_attr_green_data,
	&dev_attr_blue_data,
	&dev_attr_clear_data,
	&dev_attr_lux_data,
	&dev_attr_lux_avg_data,
	&dev_attr_dump_reg,
	&dev_attr_cofficient_data,
};

static struct attribute *lux_attrs[] = {
	&dev_attr_lux_data.attr,
	&dev_attr_lux_avg_data.attr,
	NULL,
};

static const struct attribute_group lux_group = {
	.attrs = lux_attrs,
};

static const struct attribute_group *lux_groups[] = {
	&lux_group,
	NULL,
};
/*
 * Initialization function
 */
static int rgb_bh1745_read_device_id(struct i2c_client *client)
{
	int id;

	id = rgb_bh1745_i2c_read(client, BH1745_SYSTEMCONTROL,
		BH1745_I2C_BYTE);
	id &= 0x3f;
	if (id == 0x0b) {
		BH1745_ERR("%s: ROHM BH1745\n", __func__);
	} else {
		BH1745_INFO("%s: ROHM BH1745 Does not exist id=0x%x\n",
				__func__, id);
		return -ENODEV;
	}
	return 0;
}

static int rgb_bh1745_init_client(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int err;

	data->enable = ADC_GAIN_X2 | RGBC_EN_OFF;
	err = rgb_bh1745_set_enable(client, data->enable);
	if (err < 0) {
		BH1745_ERR("%s,line%d:rgb_bh1745_set_enable FAIL ",
			__func__, __LINE__);
		return err;
	}

	err = rgb_bh1745_set_interrupt(client, BH1745_IRQ_DISABLE);
	if (err < 0) {
		BH1745_ERR("%s,line%d:rgb_bh1745_set_interrupt FAIL ",
			__func__, __LINE__);
		return err;
	}

	err = rgb_bh1745_set_measure_time(client, MEASURE_160MS);
	if (err < 0) {
		BH1745_ERR("%s,line%d:rgb_bh1745_set_measure_time FAIL ",
			__func__, __LINE__);
		return err;
	}

	err = rgb_bh1745_set_pers(client, BH1745_PPERS_1);
	if (err < 0) {
		BH1745_ERR("%s,line%d:rgb_bh1745_set_pers FAIL ",
			__func__, __LINE__);
		return err;
	}

	err = rgb_bh1745_set_control(client, MODE_CTL_FIX_VAL);
	if (err < 0) {
		BH1745_ERR("%s,line%d:rgb_bh1745_set_pers FAIL ",
			__func__, __LINE__);
		return err;
	}

	return 0;
}

/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct rgb_bh1745_data *data,
	bool on)
{
	int rc;

	if (!on) {
		regulator_put(data->vdd);
		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			BH1745_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",
				__func__, __LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
					data->regulator_vdd_vmin,
					data->regulator_vdd_vmax);
			if (rc) {
				BH1745_ERR(
					"%s,line%d:Regulator set failed vdd rc=%d\n",
					__func__, __LINE__, rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			BH1745_ERR("%s,line%d:Regulator get failed vio rc=%d\n",
				__func__, __LINE__, rc);
			goto reg_vdd_put;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
					data->regulator_vio_vmin,
					data->regulator_vio_vmax);
			if (rc) {
				BH1745_ERR(
					"%s,line%d:Regulator set failed vio rc=%d\n",
					__func__, __LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
static void rgb_bh1745_parameter_init(struct rgb_bh1745_data *data)
{
	/* default mode is standard */
	data->enable = ADC_GAIN_X2 | RGBC_EN_OFF;
	data->enable_als_sensor = 0;	/* default to 0 */
	data->als_poll_delay = 160;	/* default to 320ms */
	data->als_prev_lux = 300;
}
static int sensor_regulator_power_on(struct rgb_bh1745_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			BH1745_ERR("%s: Regulator vdd disable failed rc=%d\n",
				__func__, rc);
			goto exit;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			BH1745_ERR("%s: Regulator vdd disable failed rc=%d\n",
				__func__, rc);
			goto exit;
		}
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			BH1745_ERR("%s:Regulator vdd enable failed rc=%d\n",
				__func__, rc);
			goto exit;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			BH1745_ERR("%s:Regulator vio enable failed rc=%d\n",
				__func__, rc);
			rc = regulator_disable(data->vdd);
			goto exit;
		}
	}
exit:
	BH1745_FLOW("%s:Sensor regulator power on =%d\n", __func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,
	struct rgb_bh1745_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				/*after poweron,set the INT
				  *pin the default state
				  */
				err =
					pinctrl_select_state(data->pinctrl,
							data->pin_default);
			if (err)
				BH1745_ERR(
					"%s,line%d:Can't select pinctrl state\n",
					__func__, __LINE__);
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			BH1745_ERR("%s,line%d:Can't configure regulator!\n",
				__func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}
static int sensor_platform_hw_init(struct rgb_bh1745_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		BH1745_ERR("%s,line %d:unable to configure regulator\n",
			__func__, __LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct rgb_bh1745_data *data)
{
	int error;

	error = sensor_regulator_configure(data, false);
	if (error < 0) {
		BH1745_ERR("%s,line %d:unable to configure regulator\n",
			__func__, __LINE__);
	}
}
static int rgb_bh1745_pinctrl_init(struct rgb_bh1745_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		BH1745_ERR("%s,line %d:Failed to get pinctrl\n",
			__func__, __LINE__);
		return PTR_ERR(data->pinctrl);
	}
	/*we have not set the sleep state of INT pin */
	data->pin_default = pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		BH1745_ERR("%s,line %d:Failed to look up default state\n",
			__func__, __LINE__);
		return PTR_ERR(data->pin_default);
	}

	return 0;
}

static int sensor_parse_dt(struct device *dev,
	struct rgb_bh1745_data *data)
{
	u32 voltage_supply[2];
	int rc = 0;
	struct device_node *np = dev->of_node;

	data->init = sensor_platform_hw_init;
	data->exit = sensor_platform_hw_exit;
	data->power_onoff = sensor_platform_hw_power_on;

	rc = of_property_read_u32_array(np, "rohm,vdd-voltage", voltage_supply,
		2);
	if (rc < 0) {
		pr_err("%s failed get vdd voltage\n", __func__);
		return rc;
	}

	data->regulator_vdd_vmin = voltage_supply[0];
	data->regulator_vdd_vmax = voltage_supply[1];

	rc = of_property_read_u32_array(np, "rohm,vio-voltage", voltage_supply,
		2);
	data->regulator_vio_vmin = voltage_supply[0];
	data->regulator_vio_vmax = voltage_supply[1];
	if (rc < 0) {
		pr_err("%s failed get vio voltage\n", __func__);
		return rc;
	}

	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver rgb_bh1745_driver;
static int rgb_bh1745_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rgb_bh1745_data *data;
	int err = 0;
	int i = 0;

	BH1745_INFO("%s,line %d:PROBE START.\n", __func__, __LINE__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		BH1745_ERR("%s,line %d:Failed to i2c_check_functionality\n",
			__func__, __LINE__);
		err = -EIO;
		goto exit;
	}

	if (client->dev.of_node) {
		/*Memory allocated with this function is
		  *automatically freed on driver detach.
		  */
		data = kzalloc(sizeof(struct rgb_bh1745_data), GFP_KERNEL);
		if (!data) {
			BH1745_ERR("%s,line %d:Failed to allocate memory\n",
				__func__, __LINE__);
			err = -ENOMEM;
			goto exit;
		}
	}

	data->client = client;
	data->device_exist = false;

	err = sensor_parse_dt(&client->dev, data);
	if (err) {
		BH1745_ERR("%s: sensor_parse_dt() err\n", __func__);
		data->regulator_vdd_vmin = BH1745_VDD_MIN_UV;
		data->regulator_vdd_vmax = BH1745_VDD_MAX_UV;
		data->regulator_vio_vmin = BH1745_VIO_MIN_UV;
		data->regulator_vio_vmax = BH1745_VIO_MAX_UV;
	}

	/* h/w initialization */
	if (data->init)
		err = data->init(data);

	if (data->power_onoff)
		err = data->power_onoff(true, data);

	i2c_set_clientdata(client, data);
	rgb_bh1745_parameter_init(data);
	/* initialize pinctrl */
	err = rgb_bh1745_pinctrl_init(data);
	if (err) {
		BH1745_ERR("%s,line %d:Can't initialize pinctrl\n",
			__func__, __LINE__);
		data->pinctrl = NULL;
	} else {
		BH1745_ERR("%s,line %d:RGB BH1745 use pinctrl\n",
			__func__, __LINE__);
	}

	if (!IS_ERR_OR_NULL(data->pinctrl)) {
		err = pinctrl_select_state(data->pinctrl, data->pin_default);
		if (err) {
			BH1745_ERR(
				"%s,line %d:Can't select pinctrl default state\n",
				__func__, __LINE__);
		}
		BH1745_ERR(
			"%s,line %d:RGB BH1745 select pinctrl default state\n",
			__func__, __LINE__);
	}

	mutex_init(&data->update_lock);
	mutex_init(&data->single_lock);

	INIT_WORK(&data->als_dwork, rgb_bh1745_als_polling_work_handler);

	/* Initialize the BH1745 chip and judge who am i */
	err = rgb_bh1745_read_device_id(client);
	if (err) {
		BH1745_ERR("%s: Failed to read rgb_bh1745\n", __func__);
		goto exit_power_off;
	}

	err = rgb_bh1745_driver_reset(client);
	if (err) {
		BH1745_ERR("%s: Failed to reset rgb_bh1745\n", __func__);
		goto exit_power_off;
	}

	err = rgb_bh1745_init_client(client);
	if (err) {
		BH1745_ERR("%s: Failed to init rgb_bh1745\n", __func__);
		goto exit_power_off;
	}

	rgb_bh1745_workqueue = create_workqueue("rgb_bh1745_work_queue");
	if (!rgb_bh1745_workqueue)
		BH1745_ERR("%s: Create ps_workqueue fail.\n", __func__);

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rgb_bh1745_als_timer_func;

/*
	if (data->power_onoff)
		err = data->power_onoff(false, data);
*/
/***/
	rgb_bh1745_enable_als_sensor(client, 1);
/***/

	err = subsys_system_register(&rgb_sensor_subsys, lux_groups);

	device_rgb_sensor.id = 0;
	device_rgb_sensor.bus = &rgb_sensor_subsys;

	err = device_register(&device_rgb_sensor);

	for (i = 0; i < ARRAY_SIZE(rgb_sensor_dev_attrs); ++i) {
		err = device_create_file(&device_rgb_sensor,
			rgb_sensor_dev_attrs[i]);
	}

	BH1745_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);
	data->device_exist = true;
	bh1745_i2c_client = client;

	return 0;

exit_power_off:
	if (data->power_onoff)
		data->power_onoff(false, data);
	if (data->exit)
		data->exit(data);
	kfree(data);
exit:
	return err;
}

static int rgb_bh1745_remove(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);

	data->enable = ADC_GAIN_X2 | RGBC_EN_OFF;
	rgb_bh1745_set_enable(client, data->enable);

	free_irq(client->irq, data);
	hrtimer_cancel(&data->timer);

	if (data->power_onoff)
		data->power_onoff(false, data);

	if (data->exit)
		data->exit(data);

	kfree(data);

	return 0;
}

/*In suspend and resume function,we only control the als,leave pls alone*/
static int rgb_bh1745_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int rc;

	BH1745_ERR("%s,line%d:BH1745 SUSPEND\n", __func__, __LINE__);

	data->enable_als_state = data->enable_als_sensor;
	if (data->enable_als_sensor) {
		BH1745_INFO("%s,line%d:BH1745 SUSPEND and disable als\n",
			__func__, __LINE__);
		rc = rgb_bh1745_enable_als_sensor(data->client, 0);
		if (rc) {
			BH1745_ERR(
				"%s,line%d:Disable rgb light sensor fail! rc=%d\n",
				__func__, __LINE__, rc);
		}
	}

	return 0;
}

static int rgb_bh1745_resume(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret = 0;

	BH1745_ERR("%s,line%d:BH1745 RESUME\n",
				__func__, __LINE__);

	if (data->enable_als_state) {
		ret = rgb_bh1745_enable_als_sensor(data->client, 1);
		if (ret) {
			BH1745_ERR(
				"%s,line%d:enable rgb  light sensor fail!rc=%d\n",
				__func__, __LINE__, ret);
		}
	}

	return 0;
}

static const struct i2c_device_id rgb_bh1745_id[] = {
	{"bh1745", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rgb_bh1745_id);

static const struct of_device_id rgb_bh1745_match_table[] = {
	{.compatible = "rohm,bh1745",},
	{},
};

static struct i2c_driver rgb_bh1745_driver = {
	.driver = {
			.name = BH1745_DRV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = rgb_bh1745_match_table,
		},
	.probe = rgb_bh1745_probe,
	.remove = rgb_bh1745_remove,
	.suspend = rgb_bh1745_suspend,
	.resume = rgb_bh1745_resume,
	.id_table = rgb_bh1745_id,
};

static int __init rgb_bh1745_init(void)
{
	return i2c_add_driver(&rgb_bh1745_driver);
}

static void __exit rgb_bh1745_exit(void)
{
	if (rgb_bh1745_workqueue) {
		destroy_workqueue(rgb_bh1745_workqueue);
		rgb_bh1745_workqueue = NULL;
	}

	i2c_del_driver(&rgb_bh1745_driver);
}

MODULE_DESCRIPTION("BH1745 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rgb_bh1745_init);
module_exit(rgb_bh1745_exit);
