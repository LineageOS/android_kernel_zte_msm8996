/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ak49xx/ak49xx-slimslave.h>
#include <linux/mfd/ak49xx/core.h>
#include <linux/mfd/ak49xx/core-resource.h>
#include <linux/mfd/ak49xx/pdata.h>
#include <linux/mfd/ak49xx/ak496x_registers.h>
#ifdef CONFIG_AK4960_CODEC
#include <linux/mfd/ak49xx/ak4960_registers.h>
#endif
#ifdef CONFIG_AK4961_CODEC
#include <linux/mfd/ak49xx/ak4961_registers.h>
#endif
#ifdef CONFIG_AK4962_CODEC
#include <linux/mfd/ak49xx/ak4962_registers.h>
#endif

#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>

#define AK49XX_ENABLE_SUPPLIES
#define AK49XX_REGISTER_START_OFFSET 0x800
#define AK49XX_SLIM_SLICE_SIZE 6
#define AK49XX_SLIM_RW_MAX_TRIES 3
#define SLIMBUS_PRESENT_TIMEOUT 100

#define CODEC_DT_MAX_PROP_SIZE   40
/* for slim_spi 2014-11-17 */
#define CONTROL_IF_SPI
#ifdef CONTROL_IF_SPI
struct ak49xx *ak49xx_slim_spi;
#endif

struct pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *extncodec_sus;
	struct pinctrl_state *extncodec_act;
};

static struct pinctrl_info pinctrl_info;
#ifdef CONFIG_AK4962_CODEC
struct mutex ak49xx_spi_mutex;/*add by shengguanghui for spi mutex at 20160719*/
#endif
int ak49xx_spi_read(struct ak49xx *ak49xx, unsigned short reg, int bytes, void *dest, bool interface_reg);
int ak49xx_spi_write(struct ak49xx *ak49xx, unsigned short reg, int bytes, void *src, bool interface_reg);
static int ak49xx_slim_write_device(struct ak49xx *ak49xx,
		unsigned short reg, int bytes, void *src, bool interface);
static int ak49xx_slim_read_device(struct ak49xx *ak49xx, unsigned short reg,
				int bytes, void *dest, bool interface);

static int extcodec_get_pinctrl(struct device *dev)
{
	struct pinctrl *pinctrl;

	pinctrl = pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}
	pinctrl_info.pinctrl = pinctrl;
	/* get all the states handles from Device Tree */
	pinctrl_info.extncodec_sus = pinctrl_lookup_state(pinctrl, "suspend");
	if (IS_ERR(pinctrl_info.extncodec_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle, err: %ld\n",
				__func__, PTR_ERR(pinctrl_info.extncodec_sus));
		return -EINVAL;
	}
	pinctrl_info.extncodec_act = pinctrl_lookup_state(pinctrl, "active");
	if (IS_ERR(pinctrl_info.extncodec_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle, err: %ld\n",
				__func__, PTR_ERR(pinctrl_info.extncodec_act));
		return -EINVAL;
	}
	return 0;
}

static int ak49xx_dt_parse_vreg_info(struct device *dev,
				      struct ak49xx_regulator *vreg,
				      const char *vreg_name, bool ondemand);
static struct ak49xx_pdata *ak49xx_populate_dt_pdata(struct device *dev);

static int ak49xx_intf = AK49XX_INTERFACE_TYPE_PROBING;
static struct spi_device *ak49xx_spi = NULL;

static int ak49xx_read(struct ak49xx *ak49xx, unsigned short reg,
		       int bytes, void *dest, bool interface_reg)
{
	int ret;
	u8 *buf = dest;

	if (bytes <= 0) {
		dev_err(ak49xx->dev, "Invalid byte read length %d\n", bytes);
		return -EINVAL;
	}

	ret = ak49xx->read_dev(ak49xx, reg, bytes, dest, interface_reg);
	if (ret < 0) {
		dev_err(ak49xx->dev, "Codec read failed\n");
		return ret;
	} else
		dev_dbg(ak49xx->dev, "Read 0x%02x from 0x%x\n",
			 *buf, reg);

	return 0;
}

static int __ak49xx_reg_read(
	struct ak49xx *ak49xx,
	unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&ak49xx->io_lock);
	ret = ak49xx_read(ak49xx, reg, 1, &val, false);
	mutex_unlock(&ak49xx->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}

int ak49xx_reg_read(
	struct ak49xx_core_resource *core_res,
	unsigned short reg)
{
	struct ak49xx *ak49xx = (struct ak49xx *) core_res->parent;

	return __ak49xx_reg_read(ak49xx, reg);

}
EXPORT_SYMBOL(ak49xx_reg_read);

static int ak49xx_write(struct ak49xx *ak49xx, unsigned short reg,
			int bytes, void *src, bool interface_reg)
{
	u8 *buf = src;

	if (bytes <= 0) {
		pr_err("%s: Error, invalid write length\n", __func__);
		return -EINVAL;
	}

	dev_dbg(ak49xx->dev, "Write %02x to 0x%x\n",
		 *buf, reg);

	return ak49xx->write_dev(ak49xx, reg, bytes, src, interface_reg);
}

static int __ak49xx_reg_write(
	struct ak49xx *ak49xx,
	unsigned short reg, u8 val)
{
	int ret;

	mutex_lock(&ak49xx->io_lock);
	ret = ak49xx_write(ak49xx, reg, 1, &val, false);
	mutex_unlock(&ak49xx->io_lock);

	return ret;
}

int ak49xx_reg_write(
	struct ak49xx_core_resource *core_res,
	unsigned short reg, u8 val)
{
	struct ak49xx *ak49xx = (struct ak49xx *) core_res->parent;

	return __ak49xx_reg_write(ak49xx, reg, val);
}
EXPORT_SYMBOL(ak49xx_reg_write);

static u8 ak49xx_pgd_la;
static u8 ak49xx_inf_la;

int ak49xx_interface_reg_read(struct ak49xx *ak49xx, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&ak49xx->io_lock);
/*	ret = ak49xx_read(ak49xx, reg, 1, &val, true); */
	ret = ak49xx_slim_read_device(ak49xx, reg, 1, &val, true);
	mutex_unlock(&ak49xx->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL(ak49xx_interface_reg_read);

int ak49xx_interface_reg_write(struct ak49xx *ak49xx, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&ak49xx->io_lock);
	/* ret = ak49xx_write(ak49xx, reg, 1, &val, true); */
	ret = ak49xx_slim_write_device(ak49xx, reg, 1, &val, true);
	mutex_unlock(&ak49xx->io_lock);

	return ret;
}
EXPORT_SYMBOL(ak49xx_interface_reg_write);

static int __ak49xx_bulk_read(
	struct ak49xx *ak49xx,
	unsigned short reg,
	int count, u8 *buf)
{
	int ret;

	mutex_lock(&ak49xx->io_lock);
	/* for slim_spi 2014-11-17 */
	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI || ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		ret = ak49xx_read(ak49xx, reg, count, buf, false);
	} else {
		ret = -1;
	}

	mutex_unlock(&ak49xx->io_lock);

	return ret;
}

int ak49xx_bulk_read(
	struct ak49xx_core_resource *core_res,
	unsigned short reg,
	int count, u8 *buf)
{
	struct ak49xx *ak49xx =
			(struct ak49xx *) core_res->parent;
	return __ak49xx_bulk_read(ak49xx, reg, count, buf);
}
EXPORT_SYMBOL(ak49xx_bulk_read);

static int __ak49xx_bulk_write(struct ak49xx *ak49xx, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	mutex_lock(&ak49xx->io_lock);
	/* for slim_spi 2014-11-17 */
	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI || ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		ret = ak49xx_write(ak49xx, reg, count, buf, false);
	} else {
		ret = -1;
	}

	mutex_unlock(&ak49xx->io_lock);

	return ret;
}

int ak49xx_bulk_write(
	struct ak49xx_core_resource *core_res,
	unsigned short reg, int count, u8 *buf)
{
	struct ak49xx *ak49xx =
			(struct ak49xx *) core_res->parent;
	return __ak49xx_bulk_write(ak49xx, reg, count, buf);
}
EXPORT_SYMBOL(ak49xx_bulk_write);

int ak49xx_ram_write(struct ak49xx *ak49xx, u8 vat, u8 page,
					 u16 start, int count, u8 *buf) {
	int ret, i, addr, line;
/*	u8  prif; */

	mutex_lock(&ak49xx->io_lock);

	ret = ak49xx_write(ak49xx, VIRTUAL_ADDRESS_CONTROL, 1, &vat, false);

	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS) {

		ret = ak49xx_write(ak49xx, PAGE_SETTING, 1, &page, false);

		if (ret) {
			mutex_unlock(&ak49xx->io_lock);
			return ret;
		}

		line = count / 6;
		pr_debug("%s: Line = %d.\n", __func__, line);

		for (addr = 0x200 + start, i = 0, ret = 0; i < line; addr++, i++, buf += 6) {

			ret += ak49xx_write(ak49xx, addr, 6, buf, false);

			if (addr == 0x2FF) {
				addr = 0x1FF;
				page++;
				ret += ak49xx_write(ak49xx, PAGE_SETTING, 1, &page, false);
				pr_debug("%s: page = %X.\n", __func__, page);
			}
			if (ret) {
				pr_err("failed to write ram data in SLIMbus mode.\n");
			}
		}
		/* for slim_spi 2014-11-17 */
	} else if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI ||
		ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {

		ret = spi_write(ak49xx_spi, buf, count);
		if (ret) {
			pr_err("failed to write ram data in SPI mode.\n");
		}

	} else {
		ret = -1;
	}

	mutex_unlock(&ak49xx->io_lock);

	return ret;
}
EXPORT_SYMBOL(ak49xx_ram_write);

int ak49xx_run_ram_write(struct ak49xx *ak49xx, u8 *buf)
{
	int i, ret = 0;
	u8 runc = 0x01;

	mutex_lock(&ak49xx->io_lock);

	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS) {

		ret += ak49xx_write(ak49xx, RUN_STATE_DATA_LENGTH, 1, buf + 3, false);
		ret += ak49xx_write(ak49xx, RUN_STATE_START_ADDR1, 1, buf + 4, false);
		ret += ak49xx_write(ak49xx, RUN_STATE_START_ADDR2, 1, buf + 5, false);

		for (i = 0; i <= buf[3]; i++) {
			ret += ak49xx_write(ak49xx, RUN_STATE_DATA_1 + i*3, 1, buf + i*3 + 6, false);
			ret += ak49xx_write(ak49xx, RUN_STATE_DATA_2 + i*3, 1, buf + i*3 + 7, false);
			ret += ak49xx_write(ak49xx, RUN_STATE_DATA_3 + i*3, 1, buf + i*3 + 8, false);
		}
		ret += ak49xx_write(ak49xx, CRAM_RUN_EXE, 1, &runc, false);

	} else if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI || ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {

		ret += spi_write(ak49xx_spi, buf, buf[3] * 3 + 9);
		ret += ak49xx_write(ak49xx, CRAM_RUN_EXE, 1, &runc, false);

		if (ret) {
			pr_err("failed to write ram data in SPI mode.\n");
		}

	} else {
		ret = -1;
	}

	mutex_unlock(&ak49xx->io_lock);

	return ret;
}
EXPORT_SYMBOL(ak49xx_run_ram_write);
#ifdef CONFIG_AK4962_CODEC
int ak49xx_cram_read(unsigned short reg,
			int bytes, void *dest)
{
	u8 tx[3];
	u8 *d = dest;
	int ret = 0;

	tx[0] = 0x25;
	tx[1] = reg >> 8;
	tx[2] = reg & 0xFF;

	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI ||
		ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {

		mutex_lock(&ak49xx_spi_mutex);
		ret = spi_write_then_read(ak49xx_spi, tx, 3, d, bytes);
		mutex_unlock(&ak49xx_spi_mutex);
		if (ret != 0) {
			pr_err("failed to read ak49xx cram\n");
		}
	}
	return 0;
}
EXPORT_SYMBOL(ak49xx_cram_read);
#endif

static int ak49xx_slim_read_device(struct ak49xx *ak49xx, unsigned short reg,
				int bytes, void *dest, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	int slim_read_tries = AK49XX_SLIM_RW_MAX_TRIES;
	u8 gd_buf[AK49XX_SLIM_SLICE_SIZE];
	void *buf;
	u8 buf_size;

	msg.start_offset = AK49XX_REGISTER_START_OFFSET + reg;
	if (interface) {
		msg.num_bytes = bytes;
		buf_size = bytes;
		buf = dest;
	} else {
		msg.num_bytes = AK49XX_SLIM_SLICE_SIZE;
		buf_size = AK49XX_SLIM_SLICE_SIZE;
		buf = gd_buf;
	}
	msg.comp = NULL;

	while (1) {
		mutex_lock(&ak49xx->xfer_lock);
		ret = slim_request_val_element(interface ?
			       ak49xx->slim_slave : ak49xx->slim,
			       &msg, buf, buf_size);
		mutex_unlock(&ak49xx->xfer_lock);
		if (likely(ret == 0) || (--slim_read_tries == 0))
			break;
		usleep_range(5000, 5000);
	}

	if (ret) {
		pr_err("%s: Error, Codec read failed (%d)\n", __func__, ret);
	} else if (!interface) {
		memcpy(dest, buf, (buf_size > bytes) ? bytes : buf_size);
	}

	return ret;
}
/* Interface specifies whether the write is to the interface or general
 * registers.
 */
static int ak49xx_slim_write_device(struct ak49xx *ak49xx,
		unsigned short reg, int bytes, void *src, bool interface)
{
	int ret;
	struct slim_ele_access msg;
	int slim_write_tries = AK49XX_SLIM_RW_MAX_TRIES;
	u8 gd_buf[AK49XX_SLIM_SLICE_SIZE] = {0};
	void *buf;
	u8 buf_size;

	msg.start_offset = AK49XX_REGISTER_START_OFFSET + reg;
	if (interface) {
		msg.num_bytes = bytes;
		buf_size = bytes;
		buf = src;
	} else {
		msg.num_bytes = AK49XX_SLIM_SLICE_SIZE;
		buf_size = AK49XX_SLIM_SLICE_SIZE;
		memcpy(gd_buf, src, (buf_size > bytes) ? bytes : buf_size);
		buf = gd_buf;
	}
	msg.comp = NULL;

	while (1) {
		mutex_lock(&ak49xx->xfer_lock);
		ret = slim_change_val_element(interface ?
			      ak49xx->slim_slave : ak49xx->slim,
			      &msg, buf, buf_size);
		mutex_unlock(&ak49xx->xfer_lock);
		if (likely(ret == 0) || (--slim_write_tries == 0))
			break;
		usleep_range(5000, 5000);
	}

	if (ret)
		pr_err("%s: Error, Codec write failed (%d)\n", __func__, ret);

	return ret;
}

static int ak49xx_spi_read_device(unsigned short reg,
			int bytes, void *dest)
{
	u8 tx[3];
	u8 *d = dest;
	int ret;

	tx[0] = 0x01;
	tx[1] = reg >> 8;
	tx[2] = reg & 0xFF;
	ret = spi_write_then_read(ak49xx_spi, tx, 3, d, bytes);
	if (ret != 0) {
		pr_err("failed to read ak49xx register\n");
		return ret;
	}
	return 0;
}

static int ak49xx_spi_write_device(unsigned short reg,
			int bytes, void *src)
{
	u8 tx[bytes + 3];
	int ret;

	tx[0] = 0x81;
	tx[1] = reg >> 8;
	tx[2] = reg & 0xFF;
	memcpy(tx + 3, src, bytes);

	ret = spi_write(ak49xx_spi, tx, bytes + 3);
	if (ret != 0) {
		pr_err("[LHS]failed to write ak49xx register\n");
		return ret;
	}
	return 0;
}

static struct mfd_cell ak4960_dev[] = {
	{
		.name = "ak4960_codec",
	},
};

static struct mfd_cell ak4961_dev[] = {
	{
		.name = "ak4961_codec",
	},
};

static struct mfd_cell ak4962_dev[] = {
	{
		.name = "ak4962_codec",
	},
};

static void ak49xx_bring_up(struct ak49xx *ak49xx)
{
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY, 0x0A);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT1_ARRAY, 0x0B);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT2_ARRAY, 0x0C);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT3_ARRAY, 0x0D);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT4_ARRAY, 0x0E);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT5_ARRAY, 0x0F);

	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT8_ARRAY, 0x02); */
	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT9_ARRAY, 0x04); */
}

void ak49xx_slimbus_interfacereg_setup(struct ak49xx *ak49xx)
{
	pr_err("[LHS] %s in\n", __func__);

	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY, 0x0A);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT1_ARRAY, 0x0B);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT2_ARRAY, 0x0C);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT3_ARRAY, 0x0D);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT4_ARRAY, 0x0E);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT5_ARRAY, 0x0F);

	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT8_ARRAY, 0x02); */
	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT9_ARRAY, 0x04); */
	pr_err("[LHS] %s out\n", __func__);
}


static void ak49xx_bring_down(struct ak49xx *ak49xx)
{

}


int ak49xx_power_test(struct ak49xx *ak49xx, int val)
{
	int ret;

	pr_err("[LHS] %s line %d enter val =%d!\n", __func__, __LINE__, val);
	if (val == 0) {
		if (ak49xx->ldoen_gpio) {
			pr_err("[LHS] %s ldo on\n", __func__);
			ret = gpio_direction_output(ak49xx->ldoen_gpio, 1);
		}
	} else if (val == 1) {
		if (ak49xx->ldoen_gpio) {
			pr_err("[LHS] %s ldo off\n", __func__);
			ret = gpio_direction_output(ak49xx->ldoen_gpio, 0);
		}
	} else if (val == 2) {
		if (ak49xx->reset_gpio) {
			pr_err("[LHS] %s reset on\n", __func__);
			ret = gpio_direction_output(ak49xx->reset_gpio, 1);
		}
	} else if (val == 3) {
		if (ak49xx->reset_gpio) {
			pr_err("[LHS] %s reset off\n", __func__);
			ret = gpio_direction_output(ak49xx->reset_gpio, 0);
		}
	} else
		pr_err("[LHS] %s nothing test !\n", __func__);

	return ret;
}
EXPORT_SYMBOL(ak49xx_power_test);



static int ak49xx_ldo_en(struct ak49xx *ak49xx)
{
	int ret;

	if (ak49xx->ldo_use_pinctrl) {
		ret = pinctrl_select_state(ak49xx->pinctrl,
						    ak49xx->gpio_state_active);
		/* msleep(30); // chenjun:orig:100*/
		if (ret) {
			pr_err("%s(): error select active state",
					__func__);
		}

		pr_err("%s(): 2: ldoen_gpio(%d) value(%d)",
				__func__, ak49xx->ldoen_gpio, gpio_get_value_cansleep(ak49xx->ldoen_gpio));
	}

	if (ak49xx->ldoen_gpio) {
		ret = gpio_request(ak49xx->ldoen_gpio, "LDO_EN");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
			ak49xx->ldoen_gpio);
			/* ak49xx->ldoen_gpio = 0; */
			/* return ret; */
		}
	}

	if (ak49xx->ldoen_gpio) {
		gpio_direction_output(ak49xx->ldoen_gpio, 1);
		msleep(30); /* chenjun:orig:100 */
	}

	pr_err("%s(): 3: ldoen_gpio(%d) value(%d)",
		__func__, ak49xx->ldoen_gpio, gpio_get_value_cansleep(ak49xx->ldoen_gpio));

	pr_err("%s\n", __func__);
	return 0;
}

static int ak49xx_reset(struct ak49xx *ak49xx)
{
	int ret;
	/*for slim_spi 2014-11-17 */
	/* #ifdef CONTROL_IF_SPI	*/
	if (ak49xx->cif1_gpio) {
		ret = gpio_request(ak49xx->cif1_gpio, "CDC_CIF1");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
			ak49xx->cif1_gpio);
			ak49xx->cif1_gpio = 0;
			return ret;
		}
	}

	if (ak49xx->cif1_gpio) {
#if defined(CONTROL_IF_SPI)
		gpio_direction_output(ak49xx->cif1_gpio, 0);/* 1-slimbus, 0-spi	 //for test */
#else
		gpio_direction_output(ak49xx->cif1_gpio, 1);/* 1-slimbus, 0-spi	 //for test */
#endif
		usleep_range(1000, 1100);
	}

/*	#endif */
	if (ak49xx->reset_gpio && ak49xx->slim_device_bootup
			&& !ak49xx->use_pinctrl) {
		ret = gpio_request(ak49xx->reset_gpio, "CDC_RESET");
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n", __func__,
				ak49xx->reset_gpio);
			ak49xx->reset_gpio = 0;
			return ret;
		}
	}
	if (ak49xx->reset_gpio) {
		if (ak49xx->use_pinctrl) {
			/* Reset the CDC PDM TLMM pins to a default state */
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
				pinctrl_info.extncodec_act);
			if (ret != 0) {
				pr_err("%s: Failed to enable gpio pins\n",
						__func__);
				return -EIO;
			}
			gpio_set_value_cansleep(ak49xx->reset_gpio, 0);
			msleep(20);
			gpio_set_value_cansleep(ak49xx->reset_gpio, 1);
			msleep(20);
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
					pinctrl_info.extncodec_sus);
			if (ret != 0) {
				pr_err("%s: Failed to suspend reset pins\n",
						__func__);
				return -EIO;
			}
		} else {
			gpio_direction_output(ak49xx->reset_gpio, 0);
			usleep_range(1000, 1100);
			gpio_direction_output(ak49xx->reset_gpio, 1);
			msleep(20);
		}
	}

#if 0
/* ZTE_chenjun */
      if (ak49xx->pinctrl) {
	  ret = pinctrl_select_state(ak49xx->pinctrl,
						ak49xx->smartpa_rst_normal);

	  if (ret) {
	      pr_err("%s(): error select normal state",
			 __func__);
	  }
      }
#endif
#ifndef CONFIG_AK4962_CODEC
/* ZTE_chenjun */
      if (ak49xx->smartpa_rst1_gpio)
      {
	  ret = gpio_request(ak49xx->smartpa_rst1_gpio, "SPA_RST1");
	  if (ret) {
	      pr_err("%s: Failed to request gpio %d\n", __func__,
				ak49xx->smartpa_rst1_gpio);
	  }
	  gpio_direction_output(ak49xx->smartpa_rst1_gpio, 0);

	  pr_err("%s():SPA_RST1(%d) value(%d)",
			 __func__, ak49xx->smartpa_rst1_gpio, gpio_get_value_cansleep(ak49xx->smartpa_rst1_gpio));
      }

      if (ak49xx->smartpa_rst2_gpio)
      {
	  ret = gpio_request(ak49xx->smartpa_rst2_gpio, "SPA_RST2");
	  if (ret) {
	      pr_err("%s: Failed to request gpio %d\n", __func__,
				ak49xx->smartpa_rst2_gpio);
	  }
	  gpio_direction_output(ak49xx->smartpa_rst2_gpio, 0);

	  pr_err("%s():SPA_RST2(%d) value(%d)",
			 __func__, ak49xx->smartpa_rst2_gpio, gpio_get_value_cansleep(ak49xx->smartpa_rst2_gpio));
      }
#endif
	return 0;
}

static void ak49xx_free_reset(struct ak49xx *ak49xx)
{
	if (ak49xx->reset_gpio) {
		if (!ak49xx->use_pinctrl) {
			gpio_free(ak49xx->reset_gpio);
			ak49xx->reset_gpio = 0;
		} else
			pinctrl_put(pinctrl_info.pinctrl);
	}
}

static int ak49xx_device_init(struct ak49xx *ak49xx)
{
	int ret;
	int num_irqs = 0;
	int ak49xx_dev_size = 0;
	struct mfd_cell *ak49xx_dev = NULL;
	struct ak49xx_core_resource *core_res = &ak49xx->core_res;
	int cnt;

	mutex_init(&ak49xx->io_lock);
	mutex_init(&ak49xx->xfer_lock);

	dev_set_drvdata(ak49xx->dev, ak49xx);

	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS) {
		ak49xx_bring_up(ak49xx);

		if (ak49xx->slim->e_addr[0] == 0x00 &&
			ak49xx->slim->e_addr[1] == 0x02 &&
			ak49xx->slim->e_addr[2] == 0x60 &&
			ak49xx->slim->e_addr[3] == 0x49 &&
			ak49xx->slim->e_addr[4] == 0xdd &&
			ak49xx->slim->e_addr[5] == 0x01) {

			ak49xx_dev = ak4960_dev;
			ak49xx_dev_size = ARRAY_SIZE(ak4960_dev);
			ak49xx->codec_id = CODEC_AK4960_ID;
			num_irqs = AK4960_NUM_IRQS;

		} else if (ak49xx->slim->e_addr[0] == 0x00 &&
			ak49xx->slim->e_addr[1] == 0x02 &&
			ak49xx->slim->e_addr[2] == 0x61 &&
			ak49xx->slim->e_addr[3] == 0x49 &&
			ak49xx->slim->e_addr[4] == 0xdd &&
			ak49xx->slim->e_addr[5] == 0x01) {

			ak49xx_dev = ak4961_dev;
			ak49xx_dev_size = ARRAY_SIZE(ak4961_dev);
			ak49xx->codec_id = CODEC_AK4961_ID;
			num_irqs = AK4961_NUM_IRQS;
		} else if (ak49xx->slim->e_addr[0] == 0x00 &&
			ak49xx->slim->e_addr[1] == 0x02 &&
			ak49xx->slim->e_addr[2] == 0x62 &&
			ak49xx->slim->e_addr[3] == 0x49 &&
			ak49xx->slim->e_addr[4] == 0xdd &&
			ak49xx->slim->e_addr[5] == 0x01) {

			ak49xx_dev = ak4962_dev;
			ak49xx_dev_size = ARRAY_SIZE(ak4962_dev);
			ak49xx->codec_id = CODEC_AK4962_ID;
			num_irqs = AK4962_NUM_IRQS;
		}


	} else if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI || ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		ak49xx_bring_up(ak49xx);

	/*	if (ak49xx->cif1_gpio) {
			gpio_direction_output(ak49xx->cif1_gpio, 0);// 1-slimbus, 0-spi
			usleep_range(1000, 1100);
			}*/
		usleep_range(1000, 1100);
		for (cnt = 0; cnt < 10; cnt++) {
			if (ak49xx_spi == NULL) {
				pr_err("[lhs] %s spi interface not ready !\n", __func__);
				msleep(500);
			} else
				break;
		}
		ak49xx->read_dev = ak49xx_spi_read;
		ak49xx->write_dev = ak49xx_spi_write;
		usleep_range(1000, 1100);
		ret = __ak49xx_reg_read(ak49xx, DEVICE_CODE);
		pr_err("[lhs] device id ret=0x%2x !\n", ret);
		if (ret < 0) {
			goto err;
		} else {
			if (ret == 0x60) {
				ak49xx_dev = ak4960_dev;
				ak49xx_dev_size = ARRAY_SIZE(ak4960_dev);
				ak49xx->codec_id = CODEC_AK4960_ID;
				num_irqs = AK4960_NUM_IRQS;
			}
			if (ret == 0x61) {
				ak49xx_dev = ak4961_dev;
				ak49xx_dev_size = ARRAY_SIZE(ak4961_dev);
				ak49xx->codec_id = CODEC_AK4961_ID;
				num_irqs = AK4961_NUM_IRQS;
			}
			if (ret == 0x62) {
				ak49xx_dev = ak4962_dev;
				ak49xx_dev_size = ARRAY_SIZE(ak4962_dev);
				ak49xx->codec_id = CODEC_AK4962_ID;
				num_irqs = AK4962_NUM_IRQS;
			}
		}
	}

	core_res->parent = ak49xx;
	core_res->dev = ak49xx->dev;

	ak49xx_core_res_init(&ak49xx->core_res, num_irqs,
				AK49XX_NUM_IRQ_REGS,
				ak49xx_reg_read, ak49xx_reg_write,
				ak49xx_bulk_read, ak49xx_bulk_write);

	if (ak49xx_core_irq_init(&ak49xx->core_res))
		goto err;

	if (ak49xx->dev) {
		ret = mfd_add_devices(ak49xx->dev, -1, ak49xx_dev, ak49xx_dev_size,
			      NULL, 0, NULL);
	} else {
		pr_err("%s: mfd_add_devices no ak49xx->dev\n", __func__);
		ret = -ENOMEM;
	}
	if (ret != 0) {
		dev_err(ak49xx->dev, "Failed to add children: %d\n", ret);
		goto err_irq;
	}

	ret = device_init_wakeup(ak49xx->dev, true);
	if (ret) {
		dev_err(ak49xx->dev, "Device wakeup init failed: %d\n", ret);
		goto err_irq;
	}

	return ret;

err_irq:
	ak49xx_irq_exit(&ak49xx->core_res);
err:
	ak49xx_bring_down(ak49xx);
	ak49xx_core_res_deinit(&ak49xx->core_res);
	mutex_destroy(&ak49xx->io_lock);
	mutex_destroy(&ak49xx->xfer_lock);
	return ret;
}

static void ak49xx_device_exit(struct ak49xx *ak49xx)
{
	device_init_wakeup(ak49xx->dev, false);
	ak49xx_irq_exit(&ak49xx->core_res);
	ak49xx_bring_down(ak49xx);
	ak49xx_free_reset(ak49xx);
	ak49xx_core_res_deinit(&ak49xx->core_res);
	mutex_destroy(&ak49xx->io_lock);
	mutex_destroy(&ak49xx->xfer_lock);
	/* for slim_spi 2014-11-17 */
	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		/* printk("[lhs] %s\n",__func__); */
		slim_remove_device(ak49xx->slim_slave);
	}
	kfree(ak49xx);
}

#ifdef AK49XX_ENABLE_SUPPLIES
static int ak49xx_init_supplies(struct ak49xx *ak49xx,
				 struct ak49xx_pdata *pdata)
{
	int ret;
	int i;

	ak49xx->supplies = kzalloc(sizeof(struct regulator_bulk_data) *
				   ARRAY_SIZE(pdata->regulator),
				   GFP_KERNEL);
	if (!ak49xx->supplies) {
		ret = -ENOMEM;
		goto err;
	}

	ak49xx->num_of_supplies = 0;

	if (ARRAY_SIZE(pdata->regulator) > AK49XX_MAX_REGULATOR) {
		pr_err("%s: Array Size out of bound\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(pdata->regulator); i++) {
		if (pdata->regulator[i].name) {
			ak49xx->supplies[i].supply = pdata->regulator[i].name;
			ak49xx->num_of_supplies++;
		}
	}

	ret = regulator_bulk_get(ak49xx->dev, ak49xx->num_of_supplies,
				 ak49xx->supplies);
	if (ret != 0) {
		dev_err(ak49xx->dev, "Failed to get supplies: err = %d\n",
							ret);
		goto err_supplies;
	}

	for (i = 0; i < ak49xx->num_of_supplies; i++) {
		if (regulator_count_voltages(ak49xx->supplies[i].consumer) <=
		    0)
			continue;
		ret = regulator_set_voltage(ak49xx->supplies[i].consumer,
					    pdata->regulator[i].min_uV,
					    pdata->regulator[i].max_uV);
		if (ret) {
			pr_err("%s: Setting regulator voltage failed for "
				"regulator %s err = %d\n", __func__,
				ak49xx->supplies[i].supply, ret);
			goto err_get;
		}

		ret = regulator_set_optimum_mode(ak49xx->supplies[i].consumer,
						pdata->regulator[i].optimum_uA);
		if (ret < 0) {
			pr_err("%s: Setting regulator optimum mode failed for "
				"regulator %s err = %d\n", __func__,
				ak49xx->supplies[i].supply, ret);
			goto err_get;
		} else {
			ret = 0;
		}
	}

	return ret;

err_get:
	regulator_bulk_free(ak49xx->num_of_supplies, ak49xx->supplies);
err_supplies:
	kfree(ak49xx->supplies);
err:
	return ret;
}

static int ak49xx_enable_static_supplies(struct ak49xx *ak49xx,
					  struct ak49xx_pdata *pdata)
{
	int i;
	int ret = 0;

	for (i = 0; i < ak49xx->num_of_supplies; i++) {
		if (pdata->regulator[i].ondemand)
			continue;
		ret = regulator_enable(ak49xx->supplies[i].consumer);
		if (ret) {
			pr_err("%s: Failed to enable %s\n", __func__,
			       ak49xx->supplies[i].supply);
			break;
		} else {
			pr_debug("%s: Enabled regulator %s\n", __func__,
				 ak49xx->supplies[i].supply);
		}
	}

	while (ret && --i)
		if (!pdata->regulator[i].ondemand)
			regulator_disable(ak49xx->supplies[i].consumer);

	return ret;
}

static void ak49xx_disable_supplies(struct ak49xx *ak49xx,
				     struct ak49xx_pdata *pdata)
{
	int i;
	int rc;

	for (i = 0; i < ak49xx->num_of_supplies; i++) {
		if (pdata->regulator[i].ondemand)
			continue;
		rc = regulator_disable(ak49xx->supplies[i].consumer);
		if (rc) {
			pr_err("%s: Failed to disable %s\n", __func__,
			       ak49xx->supplies[i].supply);
		} else {
			pr_debug("%s: Disabled regulator %s\n", __func__,
				 ak49xx->supplies[i].supply);
		}
	}
	for (i = 0; i < ak49xx->num_of_supplies; i++) {
		if (regulator_count_voltages(ak49xx->supplies[i].consumer) <=
		    0)
			continue;
		regulator_set_voltage(ak49xx->supplies[i].consumer, 0,
				      pdata->regulator[i].max_uV);
		regulator_set_optimum_mode(ak49xx->supplies[i].consumer, 0);
	}
	regulator_bulk_free(ak49xx->num_of_supplies, ak49xx->supplies);
	kfree(ak49xx->supplies);
}
#endif

static int ak49xx_dt_parse_vreg_info(struct device *dev,
					struct ak49xx_regulator *vreg,
					const char *vreg_name,
					bool ondemand)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[CODEC_DT_MAX_PROP_SIZE];
	struct device_node *regnode = NULL;
	u32 prop_val;

	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE, "%s-supply",
		vreg_name);
	regnode = of_parse_phandle(dev->of_node, prop_name, 0);

	if (!regnode) {
		dev_err(dev, "Looking up %s property in node %s failed",
				prop_name, dev->of_node->full_name);
		return -ENODEV;
	}
	vreg->name = vreg_name;
	vreg->ondemand = ondemand;

	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE,
		"akm,%s-voltage", vreg_name);
	prop = of_get_property(dev->of_node, prop_name, &len);

	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_err(dev, "%s %s property\n",
				prop ? "invalid format" : "no", prop_name);
		return -ENODEV;
	} else {
		vreg->min_uV = be32_to_cpup(&prop[0]);
		vreg->max_uV = be32_to_cpup(&prop[1]);
	}

	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE,
			"akm,%s-current", vreg_name);

	ret = of_property_read_u32(dev->of_node, prop_name, &prop_val);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
				prop_name, dev->of_node->full_name);
		return -ENODEV;
	}
	vreg->optimum_uA = prop_val;

	dev_info(dev, "%s: vol=[%d %d]uV, curr=[%d]uA, ond %d\n", vreg->name,
		vreg->min_uV, vreg->max_uV, vreg->optimum_uA, vreg->ondemand);
	return 0;
}

static int ak49xx_read_of_property_u32(struct device *dev,
	const char *name, u32 *val)
{
	int ret = 0;

	ret = of_property_read_u32(dev->of_node, name, val);
	if (ret)
		dev_err(dev, "Looking up %s property in node %s failed",
				name, dev->of_node->full_name);
	return ret;
}

static int ak49xx_dt_parse_micbias_info(struct device *dev,
	struct ak49xx_micbias_setting *micbias)
{
	ak49xx_read_of_property_u32(dev, "akm,cdc-micbias-mpwr1-mv",
				&micbias->mpwr1_mv);

	ak49xx_read_of_property_u32(dev, "akm,cdc-micbias-mpwr2-mv",
				&micbias->mpwr2_mv);

	dev_dbg(dev, "mpwr1 = %u, mpwr2 = %u",
		(u32)micbias->mpwr1_mv, (u32)micbias->mpwr2_mv);

	return 0;
}

static int ak49xx_dt_parse_slim_interface_dev_info(struct device *dev,
						struct slim_device *slim_ifd)
{
	int ret = 0;
	struct property *prop;

	ret = of_property_read_string(dev->of_node, "akm,cdc-slim-ifd",
				      &slim_ifd->name);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"akm,cdc-slim-ifd", dev->of_node->full_name);
		return -ENODEV;
	}
	prop = of_find_property(dev->of_node,
			"akm,cdc-slim-ifd-elemental-addr", NULL);
	if (!prop) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"akm,cdc-slim-ifd-elemental-addr",
			dev->of_node->full_name);
		return -ENODEV;
	} else if (prop->length != 6) {
		dev_err(dev, "invalid codec slim ifd addr. addr length = %d\n",
			      prop->length);
		return -ENODEV;
	}
	memcpy(slim_ifd->e_addr, prop->value, 6);

	return 0;
}

static void ak49xx_get_pinctrl_configs(struct device *dev, struct ak49xx_pdata *pdata)
{
	struct pinctrl_state *set_state;

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
		pr_err("%s(): Pinctrl not defined", __func__);
	} else {
		pr_err("%s(): Using Pinctrl", __func__);
		pdata->use_pinctrl = true;

		set_state = pinctrl_lookup_state(pdata->pinctrl,
						"cdc_active");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for active state");
			goto pinctrl_fail;
		}

		pr_err("%s(): Pinctrl state active %p\n", __func__,
			set_state);
		pdata->gpio_state_active = set_state;
#if 0
		set_state = pinctrl_lookup_state(pdata->pinctrl,
						"cdc_sleep");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for sleep state");
			goto pinctrl_fail;
		}

		pr_err("%s(): Pinctrl state sleep %p\n", __func__,
			set_state);
		pdata->gpio_state_suspend = set_state;

/* ZTE_chenjun */
		set_state = pinctrl_lookup_state(pdata->pinctrl,
						"smartpa_rst_normal");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for smartpa_rst_normal state");
			goto pinctrl_fail;
		}

		pr_err("%s(): Pinctrl state smartpa_rst_normal %p\n", __func__,
			set_state);
		pdata->smartpa_rst_normal = set_state;
#endif

		return;
	}
pinctrl_fail:
	pdata->pinctrl = NULL;
	return;
}

static struct ak49xx_pdata *ak49xx_populate_dt_pdata(struct device *dev)
{
	struct ak49xx_pdata *pdata;
	int ret, static_cnt, i;
	const char *name = NULL;
	u32 mclk_rate = 0;
	const char *static_prop_name = "akm,cdc-static-supplies";

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	static_cnt = of_property_count_strings(dev->of_node, static_prop_name);
	if (IS_ERR_VALUE(static_cnt)) {
		dev_err(dev, "%s: Failed to get static supplies %d\n", __func__,
			static_cnt);
		goto err;
	}

	BUG_ON(static_cnt <= 0);
	if (static_cnt > ARRAY_SIZE(pdata->regulator)) {
		dev_err(dev, "%s: Num of supplies %u > max supported %zu\n",
			__func__, static_cnt, ARRAY_SIZE(pdata->regulator));
		goto err;
	}

	for (i = 0; i < static_cnt; i++) {
		ret = of_property_read_string_index(dev->of_node,
						    static_prop_name, i,
						    &name);
		if (ret) {
			dev_err(dev, "%s: of read string %s i %d error %d\n",
				__func__, static_prop_name, i, ret);
			goto err;
		}

		dev_dbg(dev, "%s: Found static cdc supply %s\n", __func__,
			name);
		ret = ak49xx_dt_parse_vreg_info(dev, &pdata->regulator[i],
						 name, false);
		if (ret)
			goto err;
	}

	ret = ak49xx_dt_parse_micbias_info(dev, &pdata->micbias);
	if (ret)
		goto err;
	pdata->ldoen_gpio = of_get_named_gpio(dev->of_node,
				"akm,cdc-ldo-gpio", 0);
	pr_err("%s pdata->ldoen_gpio : %d\n", __func__, pdata->ldoen_gpio);
	if (pdata->ldoen_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"akm,cdc-ldo-gpio", dev->of_node->full_name,
			pdata->ldoen_gpio);
		goto err;
	}

	pdata->cif1_gpio = of_get_named_gpio(dev->of_node,
				"akm,cdc-cif1-gpio", 0);
	pr_err("%s pdata->cif1_gpio : %d\n", __func__, pdata->cif1_gpio);
	if (pdata->cif1_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"akm,cdc-cif1-gpio", dev->of_node->full_name,
			pdata->cif1_gpio);
		goto err;
	}

	ak49xx_get_pinctrl_configs(dev, pdata);

	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
				"akm,cdc-reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"akm,cdc-reset-gpio", dev->of_node->full_name,
			pdata->reset_gpio);
		goto err;
	}
	dev_dbg(dev, "%s: reset gpio %d", __func__, pdata->reset_gpio);
#ifndef CONFIG_AK4962_CODEC
/* ZTE_chenjun */
	pdata->smartpa_rst1_gpio = of_get_named_gpio(dev->of_node,
				"nxp,smartpa-rst1-gpio", 0);
	if (pdata->smartpa_rst1_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"nxp,smartpa-rst1-gpio", dev->of_node->full_name,
			pdata->smartpa_rst1_gpio);
	}
	dev_err(dev, "%s: smartpa-rst1-gpio %d", __func__, pdata->smartpa_rst1_gpio);

	pdata->smartpa_rst2_gpio = of_get_named_gpio(dev->of_node,
				"nxp,smartpa-rst2-gpio", 0);
	if (pdata->smartpa_rst2_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"nxp,smartpa-rst2-gpio", dev->of_node->full_name,
			pdata->smartpa_rst2_gpio);
	}
	dev_err(dev, "%s: smartpa-rst2-gpio %d", __func__, pdata->smartpa_rst2_gpio);

#endif
	ret = of_property_read_u32(dev->of_node,
				   "akm,cdc-mclk-clk-rate",
				   &mclk_rate);
	if (ret) {
		dev_err(dev, "Looking up %s property in\n"
			"node %s failed",
			"akm,cdc-mclk-clk-rate",
			dev->of_node->full_name);
		devm_kfree(dev, pdata);
		ret = -EINVAL;
		goto err;
	}
	pdata->mclk_rate = mclk_rate;
	return pdata;
err:
	devm_kfree(dev, pdata);
	return NULL;
}

static int ak49xx_slim_get_laddr(struct slim_device *sb,
				  const u8 *e_addr, u8 e_len, u8 *laddr)
{
	int ret;
	const unsigned long timeout = jiffies +
				      msecs_to_jiffies(SLIMBUS_PRESENT_TIMEOUT);

	do {
		ret = slim_get_logical_addr(sb, e_addr, e_len, laddr);
		if (!ret)
			break;
		/* Give SLIMBUS time to report present and be ready. */
		usleep_range(1000, 1100);
		pr_debug_ratelimited("%s: retyring get logical addr\n",
				     __func__);
	} while time_before(jiffies, timeout);

	return ret;
}

static int ak49xx_slim_probe(struct slim_device *slim)
{
	struct ak49xx *ak49xx;
	struct ak49xx_pdata *pdata;
	int ret = 0;

	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SPI ||
		ak49xx_intf == AK49XX_INTERFACE_TYPE_I2C) {
		dev_dbg(&slim->dev, "%s:Codec is detected in SPI/I2C mode\n",
			__func__);
		return -ENODEV;
	}

	if (slim->dev.of_node) {
		dev_info(&slim->dev, "Platform data from device tree\n");
		pdata = ak49xx_populate_dt_pdata(&slim->dev);
		ret = ak49xx_dt_parse_slim_interface_dev_info(&slim->dev,
				&pdata->slimbus_slave_device);
		if (ret) {
			dev_err(&slim->dev, "Error, parsing slim interface\n");
			devm_kfree(&slim->dev, pdata);
			ret = -EINVAL;
			goto err;
		}
		slim->dev.platform_data = pdata;

	} else {
		dev_info(&slim->dev, "Platform data from board file\n");
		pdata = slim->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&slim->dev, "Error, no platform data\n");
		ret = -EINVAL;
		goto err;
	}

	ak49xx = kzalloc(sizeof(struct ak49xx), GFP_KERNEL);
	if (ak49xx == NULL) {
		pr_err("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	if (!slim->ctrl) {
		pr_err("Error, no SLIMBUS control data\n");
		ret = -EINVAL;
		goto err_codec;
	}
	ak49xx->slim = slim;
	slim_set_clientdata(slim, ak49xx);
	ak49xx->reset_gpio = pdata->reset_gpio;
	ak49xx->ldoen_gpio = pdata->ldoen_gpio;
	ak49xx->cif1_gpio = pdata->cif1_gpio;
	ak49xx->ldo_use_pinctrl = pdata->use_pinctrl;
	ak49xx->pinctrl = pdata->pinctrl;
	ak49xx->gpio_state_active = pdata->gpio_state_active;
	ak49xx->gpio_state_suspend = pdata->gpio_state_suspend;
#ifndef CONFIG_AK4962_CODEC
	ak49xx->smartpa_rst_normal = pdata->smartpa_rst_normal; /* ZTE_chenjun */
	ak49xx->smartpa_rst1_gpio = pdata->smartpa_rst1_gpio; /* ZTE_chenjun */
	ak49xx->smartpa_rst2_gpio = pdata->smartpa_rst2_gpio; /* ZTE_chenjun */
#endif
	ak49xx->dev = &slim->dev;
	ak49xx->mclk_rate = pdata->mclk_rate;
	ak49xx->slim_device_bootup = true;
      ak49xx_ldo_en(ak49xx);
#ifdef AK49XX_ENABLE_SUPPLIES
	ret = ak49xx_init_supplies(ak49xx, pdata);
	if (ret) {
		goto err_codec;
		pr_err("%s: Fail to init Codec supplies %d\n", __func__, ret);
	}
	ret = ak49xx_enable_static_supplies(ak49xx, pdata);
	if (ret) {
		pr_err("%s: Fail to enable Codec pre-reset supplies\n",
				__func__);
		goto err_codec;
	}
	usleep_range(5, 10);
#endif

	ret = ak49xx_reset(ak49xx);
	if (ret) {
		pr_err("%s: Resetting Codec failed\n", __func__);
		goto err_supplies;
	}

	ret = ak49xx_slim_get_laddr(ak49xx->slim, ak49xx->slim->e_addr,
				     ARRAY_SIZE(ak49xx->slim->e_addr),
				     &ak49xx->slim->laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, ak49xx->slim->name, ret);
		goto err_reset;
	}
	ak49xx->read_dev = ak49xx_slim_read_device;
	ak49xx->write_dev = ak49xx_slim_write_device;
	ak49xx_pgd_la = ak49xx->slim->laddr;
	ak49xx->slim_slave = &pdata->slimbus_slave_device;
	if (!ak49xx->dev->of_node)
		ak49xx_initialize_irq(&ak49xx->core_res,
					pdata->irq, pdata->irq_base);

	ret = slim_add_device(slim->ctrl, ak49xx->slim_slave);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		goto err_reset;
	}

	ret = ak49xx_slim_get_laddr(ak49xx->slim_slave,
				     ak49xx->slim_slave->e_addr,
				     ARRAY_SIZE(ak49xx->slim_slave->e_addr),
				     &ak49xx->slim_slave->laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, ak49xx->slim->name, ret);
		goto err_slim_add;
	}
	ak49xx_inf_la = ak49xx->slim_slave->laddr;

#ifdef CONTROL_IF_SPI
	ak49xx_intf = AK49XX_INTERFACE_TYPE_SLIMBUS_SPI;
	ak49xx_set_intf_type(AK49XX_INTERFACE_TYPE_SLIMBUS_SPI);
#else
	ak49xx_intf = AK49XX_INTERFACE_TYPE_SLIMBUS;
	ak49xx_set_intf_type(AK49XX_INTERFACE_TYPE_SLIMBUS);
#endif
	ret = ak49xx_device_init(ak49xx);
	if (ret) {
		pr_err("%s: error, initializing device failed\n", __func__);
		goto err_slim_add;
	}
#ifdef CONTROL_IF_SPI
	ak49xx_slim_spi = ak49xx;
#endif
	/* printk("[lhs]%s  out!\n",__func__); */
	return ret;

err_slim_add:
	slim_remove_device(ak49xx->slim_slave);
err_reset:
	ak49xx_free_reset(ak49xx);
err_supplies:
#ifdef AK49XX_ENABLE_SUPPLIES
	ak49xx_disable_supplies(ak49xx, pdata);
#endif
err_codec:
	kfree(ak49xx);
err:
	return ret;
}

static int ak49xx_slim_remove(struct slim_device *pdev)
{
	struct ak49xx *ak49xx;
	struct ak49xx_pdata *pdata = pdev->dev.platform_data;

	ak49xx = slim_get_devicedata(pdev);
	ak49xx_deinit_slimslave(ak49xx);
	slim_remove_device(ak49xx->slim_slave);
#ifdef AK49XX_ENABLE_SUPPLIES
	ak49xx_disable_supplies(ak49xx, pdata);
#endif
	ak49xx_device_exit(ak49xx);
	return 0;
}

int ak49xx_spi_read(struct ak49xx *ak49xx, unsigned short reg,
			int bytes, void *dest, bool interface_reg)
{
	int ret;

	ret = ak49xx_spi_read_device(reg, bytes, dest);

	/* ret = ak49xx_quam_spi_read_device(reg, bytes, dest); */
	return ret;
}

int ak49xx_spi_write(struct ak49xx *ak49xx, unsigned short reg,
			 int bytes, void *src, bool interface_reg)
{
	int ret;

	ret = ak49xx_spi_write_device(reg, bytes, src);

	return ret;
}

static int ak49xx_spi_probe(struct spi_device *spi)
{
	struct ak49xx *ak49xx;
	struct ak49xx_pdata *pdata;
	int ret = 0;
	/* int cnt; */
	int cs;
	int cpha, cpol, cs_high;
	u32 max_speed;

	pr_err("%s\n", __func__);
/*	if (ak49xx_intf == AK49XX_INTERFACE_TYPE_SLIMBUS) {
		pr_err("ak49xx card is already detected in slimbus mode\n");
		return -ENODEV;
	}*/
/*#ifdef CONTROL_IF_SPI */
	ak49xx_spi = spi;
	cs = spi->chip_select;
	cpha = (spi->mode & SPI_CPHA) ? 1:0;
	cpol = (spi->mode & SPI_CPOL) ? 1:0;
	cs_high = (spi->mode & SPI_CS_HIGH) ? 1:0;
	max_speed = spi->max_speed_hz;
	/* printk("[lhs] %s  cs [%d] CPHA [%d] CPOL [%d] CS_HIGH [%d]\n",__func__,cs, cpha, cpol, cs_high); */
	/* printk("[lhs] %s Max_speed [%d]\n",__func__ ,max_speed ); */

	return ret;
/* #endif */
	ak49xx = kzalloc(sizeof(struct ak49xx), GFP_KERNEL);
	if (ak49xx == NULL) {
		pr_err("%s: error, allocation failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	pdata = spi->dev.platform_data;
	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		ret = -EINVAL;
		goto err_codec;
	}
	ret = extcodec_get_pinctrl(&spi->dev);
	if (ret < 0)
		ak49xx->use_pinctrl = false;
	else
		ak49xx->use_pinctrl = true;
	dev_dbg(&spi->dev, "ak49xx->use_pinctrl = %d\n", ak49xx->use_pinctrl);

	dev_set_drvdata(&spi->dev, ak49xx);
	ak49xx->dev = &spi->dev;
	ak49xx->reset_gpio = pdata->reset_gpio;
	ak49xx->slim_device_bootup = true;
	if (spi->dev.of_node)
		ak49xx->mclk_rate = pdata->mclk_rate;

#ifdef AK49XX_ENABLE_SUPPLIES
	ret = ak49xx_init_supplies(ak49xx, pdata);
	if (ret) {
		goto err_codec;
		pr_err("%s: Fail to init Codec supplies %d\n", __func__, ret);
	}
	ret = ak49xx_enable_static_supplies(ak49xx, pdata);
	if (ret) {
		pr_err("%s: Fail to enable Codec pre-reset supplies\n",
				__func__);
		goto err_codec;
	}
	usleep_range(5, 5);
#endif

	ret = ak49xx_reset(ak49xx);
	if (ret) {
		pr_err("%s: Resetting Codec failed\n", __func__);
		goto err_supplies;
	}

	ak49xx_spi = spi;
	ak49xx->read_dev = ak49xx_spi_read;
	ak49xx->write_dev = ak49xx_spi_write;
	if (!ak49xx->dev->of_node)
		ak49xx_initialize_irq(&ak49xx->core_res,
				pdata->irq, pdata->irq_base);

	ret = ak49xx_device_init(ak49xx);
	if (ret) {
		pr_err("%s: error, initializing device failed\n", __func__);
		goto err_device_init;
	} else {
		pr_info("%s: succeeded in initializing device\n", __func__);
	}

	ak49xx_intf = AK49XX_INTERFACE_TYPE_SPI;
	ak49xx_set_intf_type(AK49XX_INTERFACE_TYPE_SPI);

	return ret;

err_device_init:
	ak49xx_free_reset(ak49xx);
err_supplies:
#ifdef AK49XX_ENABLE_SUPPLIES
	ak49xx_disable_supplies(ak49xx, pdata);
#endif
err_codec:
	kfree(ak49xx);
err:
	return ret;
}

static int ak49xx_spi_remove(struct spi_device *spi)
{
	struct ak49xx *ak49xx;
	struct ak49xx_pdata *pdata = spi->dev.platform_data;

	pr_debug("exit\n");
	ak49xx = dev_get_drvdata(&spi->dev);
#ifdef AK49XX_ENABLE_SUPPLIES
	ak49xx_disable_supplies(ak49xx, pdata);
#endif
	ak49xx_device_exit(ak49xx);
	return 0;
}

static int ak49xx_device_up(struct ak49xx *ak49xx)
{
	int ret = 0;
	struct ak49xx_core_resource *ak49xx_res = &ak49xx->core_res;

	if (ak49xx->slim_device_bootup) {
		ak49xx->slim_device_bootup = false;
		return 0;
	}

	dev_info(ak49xx->dev, "%s: codec bring up\n", __func__);
	ak49xx_bring_up(ak49xx);
	ret = ak49xx_irq_init(ak49xx_res);
	if (ret) {
		pr_err("%s: wcd9xx_irq_init failed : %d\n", __func__, ret);
	} else {
		if (ak49xx->post_reset)
			ret = ak49xx->post_reset(ak49xx);
	}
	return ret;
}

static int ak49xx_slim_device_reset(struct slim_device *sldev)
{
	int ret;
	struct ak49xx *ak49xx = slim_get_devicedata(sldev);

	if (!ak49xx) {
		pr_err("%s: ak49xx is NULL\n", __func__);
		return -EINVAL;
	}

	dev_info(ak49xx->dev, "%s: device reset\n", __func__);
	if (ak49xx->slim_device_bootup)
		return 0;
	ret = ak49xx_reset(ak49xx);
	if (ret)
		dev_err(ak49xx->dev, "%s: Resetting Codec failed\n", __func__);

	return ret;
}

static int ak49xx_slim_device_up(struct slim_device *sldev)
{
	struct ak49xx *ak49xx = slim_get_devicedata(sldev);

	if (!ak49xx) {
		pr_err("%s: ak49xx is NULL\n", __func__);
		return -EINVAL;
	}
	dev_info(ak49xx->dev, "%s: slim device up\n", __func__);
	return ak49xx_device_up(ak49xx);
}

static int ak49xx_slim_device_down(struct slim_device *sldev)
{
	struct ak49xx *ak49xx = slim_get_devicedata(sldev);

	if (!ak49xx) {
		pr_err("%s: ak49xx is NULL\n", __func__);
		return -EINVAL;
	}
	ak49xx_irq_exit(&ak49xx->core_res);
	if (ak49xx->dev_down)
		ak49xx->dev_down(ak49xx);
	dev_dbg(ak49xx->dev, "%s: device down\n", __func__);
	return 0;
}

static int ak49xx_slim_resume(struct slim_device *sldev)
{
	struct ak49xx *ak49xx = slim_get_devicedata(sldev);
#ifdef CONTROL_IF_SPI
	return 0;
#endif
	return ak49xx_core_res_resume(&ak49xx->core_res);
}

static int ak49xx_slim_suspend(struct slim_device *sldev, pm_message_t pmesg)
{
	struct ak49xx *ak49xx = slim_get_devicedata(sldev);
#ifdef CONTROL_IF_SPI
	return 0;
#endif
	return ak49xx_core_res_suspend(&ak49xx->core_res, pmesg);
}

static int ak49xx_spi_resume(struct spi_device *spi)
{
	struct ak49xx *ak49xx = dev_get_drvdata(&spi->dev);
#ifndef CONTROL_IF_SPI
	return 0;
#endif
	if (ak49xx)
		return ak49xx_core_res_resume(&ak49xx->core_res);
	else
		return 0;
}

static int ak49xx_spi_suspend(struct spi_device *spi, pm_message_t pmesg)
{
	struct ak49xx *ak49xx = dev_get_drvdata(&spi->dev);
#ifndef CONTROL_IF_SPI
	return 0;
#endif
	if (ak49xx)
		return ak49xx_core_res_suspend(&ak49xx->core_res, pmesg);
	else
		return 0;
}

static const struct slim_device_id ak4960_slimtest_id[] = {
	{"ak4960-slim-pgd", 0},
	{}
};

static struct slim_driver ak4960_slim_driver = {
	.driver = {
		.name = "ak4960-slim",
		.owner = THIS_MODULE,
	},
	.probe		= ak49xx_slim_probe,
	.remove		= ak49xx_slim_remove,
	.id_table		= ak4960_slimtest_id,
	.resume		= ak49xx_slim_resume,
	.suspend		= ak49xx_slim_suspend,
};

static struct spi_driver ak4960_spi_driver = {
	.driver = {
		.name	= "ak4960-spi",
		.bus		= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ak49xx_spi_probe,
	.remove		= ak49xx_spi_remove,
	.resume		= ak49xx_spi_resume,
	.suspend		= ak49xx_spi_suspend,
};
#ifdef CONFIG_AK4961_CODEC
static const struct slim_device_id ak4961_slimtest_id[] = {
	{"ak4961-slim-pgd", 0},
	{}
};

static struct slim_driver ak4961_slim_driver = {
	.driver = {
		.name = "ak4961-slim",
		.owner = THIS_MODULE,
	},
	.probe			= ak49xx_slim_probe,
	.remove			= ak49xx_slim_remove,
	.id_table		= ak4961_slimtest_id,
	.resume		= ak49xx_slim_resume,
	.suspend		= ak49xx_slim_suspend,
	.device_up		= ak49xx_slim_device_up,
	.reset_device	= ak49xx_slim_device_reset,
	.device_down	= ak49xx_slim_device_down,
};

static const struct of_device_id ak4961_match_table[] = {
	{	.compatible = "qcom,ak4961-spi",
	},
	{}
};

static struct spi_driver ak4961_spi_driver = {
	.driver = {
		.name	= "ak4961-spi",
		/* .bus	= &spi_bus_type, */
		.owner	= THIS_MODULE,
		.of_match_table = ak4961_match_table,
	},
	.probe		= ak49xx_spi_probe,
	.remove		= ak49xx_spi_remove,
	.resume		= ak49xx_spi_resume,
	.suspend	= ak49xx_spi_suspend,
};
#endif
#ifdef CONFIG_AK4962_CODEC
static const struct slim_device_id ak4962_slimtest_id[] = {
	{"ak4962-slim-pgd", 0},
	{}
};

static struct slim_driver ak4962_slim_driver = {
	.driver = {
		.name = "ak4962-slim",
		.owner = THIS_MODULE,
	},
	.probe			= ak49xx_slim_probe,
	.remove			= ak49xx_slim_remove,
	.id_table		= ak4962_slimtest_id,
	.resume		= ak49xx_slim_resume,
	.suspend		= ak49xx_slim_suspend,
	.device_up		= ak49xx_slim_device_up,
	.reset_device	= ak49xx_slim_device_reset,
	.device_down	= ak49xx_slim_device_down,
};

static const struct of_device_id ak4962_match_table[] = {
	{	.compatible = "qcom,ak4962-spi", },
	{}
};

static struct spi_driver ak4962_spi_driver = {
	.driver = {
		.name	= "ak4962-spi",
		/*.bus	= &spi_bus_type,*/
		.owner	= THIS_MODULE,
		.of_match_table = ak4962_match_table,
	},
	.probe		= ak49xx_spi_probe,
	.remove		= ak49xx_spi_remove,
	.resume		= ak49xx_spi_resume,
	.suspend	= ak49xx_spi_suspend,
};
#endif
static int __init ak49xx_init(void)
{
	int ret1, ret2;
	#ifdef CONFIG_AK4961_CODEC
	int ret3, ret4;
	#endif
	#ifdef CONFIG_AK4962_CODEC
	int ret5, ret6;
	#endif

	ret1 = slim_driver_register(&ak4960_slim_driver);
	if (ret1 != 0) {
		pr_err("Failed to register ak4960_slim_driver: %d\n", ret1);
		return ret1;
	}
	pr_info("%s: succeed in registering ak4960_slim_driver\n", __func__);

	ret2 = spi_register_driver(&ak4960_spi_driver);
	if (ret2 != 0) {
		pr_err("Failed to register ak4960_spi_driver: %d\n", ret2);
		return ret2;
	}
	pr_info("%s: succeed in registering ak4960_spi_driver\n", __func__);

#ifdef CONFIG_AK4961_CODEC
	ret3 = slim_driver_register(&ak4961_slim_driver);
	if (ret3 != 0) {
		pr_err("Failed to register ak4961_slim_driver: %d\n", ret3);
		return ret3;
	}
	pr_info("%s: succeed in registering ak4961_slim_driver\n", __func__);

	ret4 = spi_register_driver(&ak4961_spi_driver);
	if (ret4 != 0) {
		pr_err("Failed to register ak4961_spi_driver: %d\n", ret4);
		return ret4;
	}
	pr_info("%s: succeed in registering ak4961_spi_driver\n", __func__);

#endif
#ifdef CONFIG_AK4962_CODEC
	ret5 = slim_driver_register(&ak4962_slim_driver);
	if (ret5 != 0) {
		pr_err("Failed to register ak4962_slim_driver: %d\n", ret5);
		return ret5;
	}
	pr_info("%s: succeed in registering ak4962_slim_driver\n", __func__);

	ret6 = spi_register_driver(&ak4962_spi_driver);
	if (ret6 != 0) {
		pr_err("Failed to register ak4962_spi_driver: %d\n", ret6);
		return ret6;
	}
	pr_info("%s: succeed in registering ak4962_spi_driver\n", __func__);
#endif

	/*return (ret1 && ret2 && ret3 && ret4 && ret5 && ret6) ? -1 : 0;*/
	return 0;
}
module_init(ak49xx_init);
/* late_initcall(ak49xx_init); */

static void __exit ak49xx_exit(void)
{
	spi_unregister_driver(&ak4960_spi_driver);
	#ifdef CONFIG_AK4961_CODEC
	spi_unregister_driver(&ak4961_spi_driver);
	#endif
	#ifdef CONFIG_AK4962_CODEC
	spi_unregister_driver(&ak4962_spi_driver);
	#endif
	ak49xx_intf = AK49XX_INTERFACE_TYPE_PROBING;
	ak49xx_set_intf_type(AK49XX_INTERFACE_TYPE_PROBING);
}
module_exit(ak49xx_exit);

MODULE_DESCRIPTION("ak496x core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");

