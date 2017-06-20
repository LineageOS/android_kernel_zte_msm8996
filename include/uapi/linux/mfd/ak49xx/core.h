/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#ifndef __MFD_AK49XX_CORE_H__
#define __MFD_AK49XX_CORE_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/ak49xx/core-resource.h>

#define AK49XX_SLIM_STATUS_REG 4

/*
 * data structure for SLIMbus and I2S channel.
 * Some of fields are only used in SLIMbus mode
 */
struct ak49xx_ch {
	u32 sph;		/* share channel handle - SLIMbus only	*/
	u32 ch_num;		/*
				 * virtual channel number, such as 128 -144.
				 * apply for SLIMbus only
				 */
	u16 ch_h;		/* channel handle - SLIMbus only */
	u16 port;		/* device port for RX and TX */
	u16 shift;		/*
				 * shift bit for RX and TX
				 * apply for both i2s and SLIMbus
				 */
	struct list_head list;	/*
				 * channel link list
				 * apply for both i2s and SLIMbus
				 */
};

struct ak49xx_codec_dai_data {
	u32 rate;				/* sample rate */
	u32 bit_width;				/* sit width 16,24,32 */
	struct list_head ak49xx_ch_list;	/* channel list */
	u16 grph;				/* SLIMbus group handle */
	unsigned long ch_mask;
	wait_queue_head_t dai_wait;
	bool bus_down_in_recovery;
};

#define AK49XX_CH(xport, xshift) \
	{.port = xport, .shift = xshift}

struct ak49xx {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_slave;
	struct mutex io_lock;
	struct mutex xfer_lock;
	u8 version;

	int reset_gpio;
	int smartpa_rst1_gpio; // ZTE_chenjun
	int smartpa_rst2_gpio; // ZTE_chenjun
       int ldoen_gpio;
	int cif1_gpio;

	int cs_gpio;
	bool use_pinctrl;
	bool ldo_use_pinctrl;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct pinctrl_state *smartpa_rst_normal; // ZTE_chenjun
	int pa_spk_rst_gpio;
	int pa_rcv_rst_gpio;
	int pa_rst_gpio;

	int (*read_dev)(struct ak49xx *ak49xx, unsigned short reg,
			int bytes, void *dest, bool interface_reg);
	int (*write_dev)(struct ak49xx *ak49xx, unsigned short reg,
			 int bytes, void *src, bool interface_reg);
	int (*dev_down)(struct ak49xx *ak49xx);
	int (*post_reset)(struct ak49xx *ak49xx);

	void *ssr_priv;
	bool slim_device_bootup;

	u32 num_of_supplies;
	struct regulator_bulk_data *supplies;

	struct ak49xx_core_resource core_res;

	/* Slimbus or I2S port */
	u32 num_rx_port;
	u32 num_tx_port;
	struct ak49xx_ch *rx_chs;
	struct ak49xx_ch *tx_chs;
	u32 mclk_rate;
	/*u16 use_pinctrl;*/

	u8 codec_id;
};

int ak49xx_interface_reg_read(struct ak49xx *ak49xx, unsigned short reg);
int ak49xx_interface_reg_write(struct ak49xx *ak49xx, unsigned short reg,
		u8 val);
int ak49xx_get_logical_addresses(u8 *pgd_la, u8 *inf_la);

int ak49xx_ram_write(struct ak49xx *ak49xx, u8 vat, u8 page,
			u16 start, int count, u8 *buf);
int ak49xx_run_ram_write(struct ak49xx *ak49xx, u8 *buf);

void ak49xx_slimbus_interfacereg_setup(struct ak49xx *ak49xx);
int ak49xx_power_test(struct ak49xx *ak49xx,int val);
#ifdef CONFIG_AK4962_CODEC
int ak49xx_cram_read(unsigned short reg, int bytes, void *dest);
#endif

#if defined(CONFIG_AK4960_CODEC) || \
	defined(CONFIG_AK4961_CODEC) || \
	defined(CONFIG_AK4962_CODEC)
int __init ak49xx_irq_of_init(struct device_node *node,
			       struct device_node *parent);
#else
static inline int __init ak49xx_irq_of_init(struct device_node *node,
			       struct device_node *parent)
{
	return 0;
}
#endif	/* CONFIG_OF */
#endif
