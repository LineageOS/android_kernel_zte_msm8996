/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#ifndef __MFD_AK49XX_PDATA_H__

#define __MFD_AK49XX_PDATA_H__

#include <linux/slimbus/slimbus.h>
#include <linux/pinctrl/consumer.h>

#define MICBIAS_EXT_BYP_CAP 0x00
#define MICBIAS_NO_EXT_BYP_CAP 0x01

#define AK4961_MCLK_CLK_12P288MHZ 12288000
#define AK4961_MCLK_CLK_9P6HZ 9600000

struct ak49xx_micbias_setting {
	u32 mpwr1_mv; /* in mv */
	u32 mpwr2_mv; /* in mv */
};

#define AK49XX_MAX_REGULATOR	8
/*
 *      format : AK496X_<POWER_SUPPLY_PIN_NAME>_CUR_MAX
 *
 *      <POWER_SUPPLY_PIN_NAME> from ak496x objective spec
*/
#define  AK49XX_AVDD1_CUR_MAX	20000
#define  AK49XX_AVDD2_CUR_MAX	20000
#define  AK49XX_CVDD_CUR_MAX	150000
#define  AK49XX_TVDD1_CUR_MAX	10000
#define  AK49XX_TVDD2_CUR_MAX	5000
#define  AK49XX_TVDD2_CUR_MAX	5000
#define  AK49XX_VDD12_CUR_MAX	50000

struct ak49xx_regulator {
	const char *name;
	int min_uV;
	int max_uV;
	int optimum_uA;
	bool ondemand;
	struct regulator *regulator;
};

struct ak49xx_pdata {
	int irq;
	int irq_base;
	int num_irqs;
	int reset_gpio;
	int smartpa_rst1_gpio; // ZTE_chenjun
	int smartpa_rst2_gpio; // ZTE_chenjun
	int ldoen_gpio;
	int cif1_gpio;
	int cs_gpio;
	bool use_pinctrl;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct pinctrl_state *smartpa_rst_normal; // ZTE_chenjun
	int pa_spk_rst_gpio; /*for smartPA*/
	int pa_rcv_rst_gpio; /*for smartPA*/
	int pa_rst_gpio;
	struct slim_device slimbus_slave_device;
	struct ak49xx_micbias_setting micbias;
	struct ak49xx_regulator regulator[AK49XX_MAX_REGULATOR];
	u32 mclk_rate;
};

#endif
