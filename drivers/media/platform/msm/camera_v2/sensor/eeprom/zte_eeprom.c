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
/*
  * Added  eeprom driver 
  *
  * by ZTE_YCM_20140724 yi.changming 000025
  */
// --->
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

//#define CONFIG_ZTE_CAMERA_EEPROM_DEBUG
#undef CDBG
#ifdef CONFIG_ZTE_CAMERA_EEPROM_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

/*
  * camera sensor module compatile
  * 
  * by ZTE_YCM_20140728 yi.changming 000028
  */
// --->
#define 	SENSOR_INFO_MODULE_ID_QETCH 0X0001
#define 	SENSOR_INFO_MODULE_ID_SUNNY 0X000a
#define 	SENSOR_INFO_MODULE_ID_MCNEX 0X00a0
#define 	SENSOR_INFO_MODULE_ID_TRULY 0X0010
#define 	SENSOR_INFO_MODULE_ID_SAMSUNG 0X0033
#define 	SENSOR_INFO_MODULE_ID_KARR 0X0002

typedef struct {
        uint16_t id;
        const char * sensor_module_name;
	 const char * chromtix_lib_name;
	 const char * default_chromtix_lib_name;
} MODULE_Map_Table;

MODULE_Map_Table AR0542_MODULE_MAP[] = {
    { SENSOR_INFO_MODULE_ID_QETCH,"qtech_ar0542", "qtech_ar0542",NULL},
    { SENSOR_INFO_MODULE_ID_SUNNY, "sunny_ar0542","sunny_ar0542",NULL},
    { SENSOR_INFO_MODULE_ID_MCNEX, "mcnex_ar0542","mcnex_ar0542",NULL},
    { SENSOR_INFO_MODULE_ID_TRULY,  "truly_ar0542","truly_ar0542",NULL},
    { SENSOR_INFO_MODULE_ID_SAMSUNG,"samsung_ar0542","samsung_ar0542",NULL},
    { SENSOR_INFO_MODULE_ID_KARR,"karr_ar0542","karr_ar0542",NULL},
};
// <---
/*
  * add imx219 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// ---> 
#define IMX219_SENSOR_INFO_MODULE_ID_SUNNY	0x01
#define IMX219_SENSOR_INFO_MODULE_ID_QTECH	0x06
#define IMX219_SENSOR_INFO_MODULE_ID_LITEON	0x15
#define IMX219_SENSOR_INFO_MODULE_ID_MCNEX  0x31

MODULE_Map_Table IMX219_MODULE_MAP[] = {
    { IMX219_SENSOR_INFO_MODULE_ID_SUNNY,"sunny_imx219","sunny_imx219",NULL},
    { IMX219_SENSOR_INFO_MODULE_ID_QTECH, "qtech_imx219","qtech_imx219",NULL},
    { IMX219_SENSOR_INFO_MODULE_ID_LITEON, "liteon_imx219","liteon_imx219",NULL},
    { IMX219_SENSOR_INFO_MODULE_ID_MCNEX,  "mcnex_imx219","mcnex_imx219",NULL},
};
// <---000064

/*
  * add imx219 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// ---> 
#define T4K35_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define T4K35_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define T4K35_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define T4K35_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define T4K35_SENSOR_INFO_MODULE_ID_DARLING		0x05
#define T4K35_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define T4K35_SENSOR_INFO_MODULE_ID_QFLIM		0x07
#define T4K35_SENSOR_INFO_MODULE_ID_RICHTEK		0x08
#define T4K35_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define T4K35_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define T4K35_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define T4K35_SENSOR_INFO_MODULE_ID_ABICO_ABILITY	0x14
#define T4K35_SENSOR_INFO_MODULE_ID_LITE_ON		0x15
#define T4K35_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define T4K35_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define T4K35_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define T4K35_SENSOR_INFO_MODULE_ID_MCNEX  		0x31

MODULE_Map_Table T4K35_MODULE_MAP[] = {
    { T4K35_SENSOR_INFO_MODULE_ID_SUNNY	,	"sunny_t4k35","sunny_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_TRULY,		"truly_t4k35","truly_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_A_KERR,	"a_kerr_t4k35","a_kerr_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_LITEARRAY,	"litearray_t4k35","litearray_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_DARLING,	"darling_t4k35","darling_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_QTECH,		"qtech_t4k35","qtech_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_QFLIM,		"qflim_t4k35","qflim_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_RICHTEK,	"richtek_t4k35","richtek_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_FOXCONN,	"foxconn_t4k35","foxconn_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_IMPORTEK,	"importek_t4k35","importek_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_ALTEK,		"altek_t4k35","altek_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_ABICO_ABILITY,"abico_ability_t4k35","abico_ability_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_LITE_ON,	"lite_on_t4k35","lite_on_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_CHICONY,	"chicony_t4k35","chicony_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_PRIMAX,	"primax_t4k35","primax_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_SHARP,		"sharp_t4k35","sharp_t4k35",NULL},
    { T4K35_SENSOR_INFO_MODULE_ID_MCNEX,		"mcnex_t4k35","mcnex_t4k35",NULL},
};
// <---000064

#define IMX214_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define IMX214_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define IMX214_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define IMX214_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define IMX214_SENSOR_INFO_MODULE_ID_DARLING		0x05
#define IMX214_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define IMX214_SENSOR_INFO_MODULE_ID_QFLIM		0x07
#define IMX214_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define IMX214_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define IMX214_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define IMX214_SENSOR_INFO_MODULE_ID_ABICO_ABILITY	0x14
#define IMX214_SENSOR_INFO_MODULE_ID_LITE_ON		0x15
#define IMX214_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define IMX214_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define IMX214_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define IMX214_SENSOR_INFO_MODULE_ID_LITE_ON_NEW		0x22
#define IMX214_SENSOR_INFO_MODULE_ID_MCNEX  		0x31
#define IMX214_SENSOR_INFO_MODULE_ID_MCNEX_2  		0x33

MODULE_Map_Table IMX214_MODULE_MAP[] = {
    { IMX214_SENSOR_INFO_MODULE_ID_SUNNY	,	"sunny_imx214","sunny_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_TRULY,		"truly_imx214","truly_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_A_KERR,	"a_kerr_imx214","a_kerr_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_LITEARRAY,	"litearray_imx214","litearray_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_DARLING,	"darling_imx214","darling_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_QTECH,		"qtech_imx214","qtech_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_QFLIM,		"qflim_imx214","qflim_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_FOXCONN,	"foxconn_imx214","foxconn_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_IMPORTEK,	"importek_imx214","importek_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_ALTEK,		"altek_imx214","altek_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_ABICO_ABILITY,"abico_ability_imx214","abico_ability_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_LITE_ON,	"lite_on_imx214","lite_on_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_CHICONY,	"chicony_imx214","chicony_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_PRIMAX,	"primax_imx214","primax_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_SHARP,		"sharp_imx214","sharp_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_LITE_ON_NEW,	"lite_on_new_imx214","lite_on_new_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_MCNEX,		"mcnex_imx214","mcnex_imx214",NULL},
    { IMX214_SENSOR_INFO_MODULE_ID_MCNEX_2,		"mcnex_2_imx214","mcnex_2_imx214",NULL},
};


#define S5K3L2_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define S5K3L2_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define S5K3L2_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define S5K3L2_SENSOR_INFO_MODULE_ID_DARLING		0x05
#define S5K3L2_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define S5K3L2_SENSOR_INFO_MODULE_ID_QFLIM		0x07
#define S5K3L2_SENSOR_INFO_MODULE_ID_TRULY		0x08
#define S5K3L2_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define S5K3L2_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define S5K3L2_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define S5K3L2_SENSOR_INFO_MODULE_ID_ABICO_ABILITY	0x14
#define S5K3L2_SENSOR_INFO_MODULE_ID_LITE_ON		0x15
#define S5K3L2_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define S5K3L2_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define S5K3L2_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define S5K3L2_SENSOR_INFO_MODULE_ID_LITE_ON_NEW		0x22
#define S5K3L2_SENSOR_INFO_MODULE_ID_MCNEX  		0x31
#define S5K3L2_SENSOR_INFO_MODULE_ID_MCNEX_2  		0xA0

MODULE_Map_Table S5K3L2_MODULE_MAP[] = {
    { S5K3L2_SENSOR_INFO_MODULE_ID_SUNNY	,	"sunny_s5k3l2","sunny_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_A_KERR,	"a_kerr_s5k3l2","a_kerr_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_LITEARRAY,	"litearray_s5k3l2","litearray_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_DARLING,	"darling_s5k3l2","darling_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_QTECH,		"qtech_s5k3l2","qtech_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_QFLIM,		"qflim_s5k3l2","qflim_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_TRULY,		"truly_s5k3l2","truly_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_FOXCONN,	"foxconn_s5k3l2","foxconn_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_IMPORTEK,	"importek_s5k3l2","importek_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_ALTEK,		"altek_s5k3l2","altek_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_ABICO_ABILITY,"abico_ability_s5k3l2","abico_ability_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_LITE_ON,	"lite_on_s5k3l2","lite_on_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_CHICONY,	"chicony_s5k3l2","chicony_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_PRIMAX,	"primax_s5k3l2","primax_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_SHARP,		"sharp_s5k3l2","sharp_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_LITE_ON_NEW,	"lite_on_new_s5k3l2","lite_on_new_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_MCNEX,		"mcnex_s5k3l2","mcnex_s5k3l2",NULL},
    { S5K3L2_SENSOR_INFO_MODULE_ID_MCNEX_2,		"mcnex_2_s5k3l2","mcnex_2_s5k3l2",NULL},
};

#define OV8856_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define OV8856_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define OV8856_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define OV8856_SENSOR_INFO_MODULE_ID_DARLING		0x05
#define OV8856_SENSOR_INFO_MODULE_ID_QTECH		0x06

#define OV8856_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define OV8856_SENSOR_INFO_MODULE_ID_FOXCONN		0x11
#define OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK		0x12
#define OV8856_SENSOR_INFO_MODULE_ID_ALTEK	       0x13
#define OV8856_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define OV8856_SENSOR_INFO_MODULE_ID_LITE_ON		0x15

#define OV8856_SENSOR_INFO_MODULE_ID_CHICONNY		0x16
#define OV8856_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define OV8856_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define OV8856_SENSOR_INFO_MODULE_ID_MCNEX   	0x31


MODULE_Map_Table OV8856_MODULE_MAP[] = {
    { OV8856_SENSOR_INFO_MODULE_ID_SUNNY	,	"sunny_ov8856","sunny_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_TRULY,		"truly_ov8856","truly_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_A_KERR,	"a_kerr_ov8856","a_kerr_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY,	"litearray_ov8856","litearray_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_DARLING,	"darling_ov8856","darling_tov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_QTECH,		"qtech_ov8856","qtech_ov8856",NULL},

    { OV8856_SENSOR_INFO_MODULE_ID_OFLIM	,	"oflim_ov8856","oflim_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_FOXCONN,		"foxconn_ov8856","foxconn_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK,	"importek_ov8856","importek_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_ALTEK,	"altek_ov8856","altek_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_ABICO,	"abico_ov8856","abico_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_LITE_ON,		"lite_on_ov8856","lite_on_ov8856",NULL},

    { OV8856_SENSOR_INFO_MODULE_ID_CHICONNY	,	"chiconny_ov8856","chiconny_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_PRIMAX,		"primax_ov8856","primax_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_SHARP,	"sharp_ov8856","sharp_ov8856",NULL},
    { OV8856_SENSOR_INFO_MODULE_ID_MCNEX,	"mcnex_ov8856","mcnex_ov8856",NULL},

};

#define S5K2T8_SENSOR_INFO_MODULE_ID_SUNNY   	0x01
#define S5K2T8_SENSOR_INFO_MODULE_ID_TRULY   	0x02
#define S5K2T8_SENSOR_INFO_MODULE_ID_A_KERR   	0x03
#define S5K2T8_SENSOR_INFO_MODULE_ID_LITEARRAY   	0x04
#define S5K2T8_SENSOR_INFO_MODULE_ID_DARLING   	0x05
#define S5K2T8_SENSOR_INFO_MODULE_ID_QTECH   	0x06
#define S5K2T8_SENSOR_INFO_MODULE_ID_OFLIM   	0x07
#define S5K2T8_SENSOR_INFO_MODULE_ID_FOXCONN   	0x11
#define S5K2T8_SENSOR_INFO_MODULE_ID_IMPORTEK   	0x12
#define S5K2T8_SENSOR_INFO_MODULE_ID_ALTEK   	0x13
#define S5K2T8_SENSOR_INFO_MODULE_ID_ABICO   	0x14
#define S5K2T8_SENSOR_INFO_MODULE_ID_LITE_ON   	0x15
#define S5K2T8_SENSOR_INFO_MODULE_ID_SEMCO   	0xA1


MODULE_Map_Table S5K2T8_MODULE_MAP[] = {
    { S5K2T8_SENSOR_INFO_MODULE_ID_SUNNY,	"sunny_s5k2t8","sunny_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_TRULY,	"truly_s5k2t8","truly_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_A_KERR,	"a_kerr_s5k2t8","a_kerr_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_LITEARRAY,	"litearray_s5k2t8","litearray_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_DARLING,	"darling_s5k2t8","darling_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_QTECH,	"qtech_s5k2t8","qtech_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_OFLIM,	"oflim_s5k2t8","oflim_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_FOXCONN,	"foxconn_s5k2t8","foxconn_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_IMPORTEK,	"importek_s5k2t8","importek_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_ALTEK,	"altek_s5k2t8","altek_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_ABICO,	"abico_s5k2t8","abico_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_LITE_ON,	"lite_on_s5k2t8","lite_on_s5k2t8",NULL},
    { S5K2T8_SENSOR_INFO_MODULE_ID_SEMCO,	"semco_s5k2t8","semco_s5k2t8",NULL},
};


DEFINE_MSM_MUTEX(msm_eeprom_mutex);
#ifdef CONFIG_COMPAT
static struct v4l2_file_operations msm_eeprom_v4l2_subdev_fops;
#endif

typedef 	int (*zte_read_eeprom_memory_func_t) (struct msm_eeprom_ctrl_t *,
			      struct msm_eeprom_memory_block_t *);


static int msm_eeprom_get_cmm_data(struct msm_eeprom_ctrl_t *e_ctrl,
				       struct msm_eeprom_cfg_data *cdata)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &e_ctrl->eboard_info->cmm_data;
	cdata->cfg.get_cmm_data.cmm_support = cmm_data->cmm_support;
	cdata->cfg.get_cmm_data.cmm_compression = cmm_data->cmm_compression;
	cdata->cfg.get_cmm_data.cmm_size = cmm_data->cmm_size;
	return rc;
}

static int eeprom_config_read_cal_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_cfg_data *cdata)
{
	int rc;

	/* check range */
	if (cdata->cfg.read_data.num_bytes >
		e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata->cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	rc = copy_to_user(cdata->cfg.read_data.dbuffer,
		e_ctrl->cal_data.mapdata,
		cdata->cfg.read_data.num_bytes);

	return rc;
}

static int msm_eeprom_config(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *argp)
{
	struct msm_eeprom_cfg_data *cdata =
		(struct msm_eeprom_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name,
			sizeof(cdata->cfg.eeprom_name));
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data(e_ctrl, cdata);
		break;
	case CFG_EEPROM_GET_MM_INFO:
		CDBG("%s E CFG_EEPROM_GET_MM_INFO\n", __func__);
		rc = msm_eeprom_get_cmm_data(e_ctrl, cdata);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static int msm_eeprom_get_subdev_id(struct msm_eeprom_ctrl_t *e_ctrl,
				    void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("%s E\n", __func__);
	if (!subdev_id) {
		pr_err("%s failed\n", __func__);
		return -EINVAL;
	}
	*subdev_id = e_ctrl->subdev_id;
	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("%s X\n", __func__);
	return 0;
}

static long msm_eeprom_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG:
		return msm_eeprom_config(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static struct msm_camera_i2c_fn_t msm_eeprom_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};


static int msm_eeprom_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static int msm_eeprom_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_eeprom_internal_ops = {
	.open = msm_eeprom_open,
	.close = msm_eeprom_close,
};



static struct v4l2_subdev_core_ops msm_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_eeprom_subdev_ops = {
	.core = &msm_eeprom_subdev_core_ops,
};



static int msm_eeprom_get_dt_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0, i = 0;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info =
		&e_ctrl->eboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	uint16_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;

	eb_info = e_ctrl->eboard_info;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE)
		of_node = e_ctrl->i2c_client.
			spi_client->spi_master->dev.of_node;
	else if (e_ctrl->eeprom_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = e_ctrl->pdev->dev.of_node;

	if (!of_node) {
		pr_err("%s: %d of_node is NULL\n", __func__ , __LINE__);
		return -ENOMEM;
	}
	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					     &power_info->num_vreg);
	if (rc < 0)
		return rc;

	rc = msm_camera_get_dt_power_setting_data(of_node,
		power_info->cam_vreg, power_info->num_vreg,
		power_info);
	if (rc < 0)
		goto ERROR1;

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR3;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}
		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
	return rc;
}


static int msm_eeprom_cmm_dts(struct msm_eeprom_board_info *eb_info,
				struct device_node *of_node)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &eb_info->cmm_data;

	cmm_data->cmm_support =
		of_property_read_bool(of_node, "qcom,cmm-data-support");
	if (!cmm_data->cmm_support)
		return -EINVAL;
	cmm_data->cmm_compression =
		of_property_read_bool(of_node, "qcom,cmm-data-compressed");
	if (!cmm_data->cmm_compression)
		CDBG("No MM compression data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-offset",
				  &cmm_data->cmm_offset);
	if (rc < 0)
		CDBG("No MM offset data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-size",
				  &cmm_data->cmm_size);
	if (rc < 0)
		CDBG("No MM size data\n");

	CDBG("cmm_support: cmm_compr %d, cmm_offset %d, cmm_size %d\n",
		cmm_data->cmm_compression,
		cmm_data->cmm_offset,
		cmm_data->cmm_size);
	return 0;
}





#ifdef CONFIG_COMPAT
static int eeprom_config_read_cal_data32(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *arg)
{
	int rc;
	uint8_t *ptr_dest = NULL;
	struct msm_eeprom_cfg_data32 *cdata32 =
		(struct msm_eeprom_cfg_data32 *) arg;
	struct msm_eeprom_cfg_data cdata;

	cdata.cfgtype = cdata32->cfgtype;
	cdata.is_supported = cdata32->is_supported;
	cdata.cfg.read_data.num_bytes = cdata32->cfg.read_data.num_bytes;
	/* check range */
	if (cdata.cfg.read_data.num_bytes >
	    e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata.cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	ptr_dest = (uint8_t *) compat_ptr(cdata32->cfg.read_data.dbuffer);

	rc = copy_to_user(ptr_dest, e_ctrl->cal_data.mapdata,
		cdata.cfg.read_data.num_bytes);

	return rc;
}

static int msm_eeprom_config32(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *argp)
{
	struct msm_eeprom_cfg_data *cdata = (struct msm_eeprom_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name,
			sizeof(cdata->cfg.eeprom_name));
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data32(e_ctrl, argp);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static long msm_eeprom_subdev_ioctl32(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG32:
		return msm_eeprom_config32(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static long msm_eeprom_subdev_do_ioctl32(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return msm_eeprom_subdev_ioctl32(sd, cmd, arg);
}

static long msm_eeprom_subdev_fops_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_eeprom_subdev_do_ioctl32);
}

#endif



int ar0542_check_block_status(struct msm_eeprom_ctrl_t *e_ctrl, int block_type)
{
	uint16_t i = 0;
	uint16_t temp = 0;
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x304C,(block_type&0xff)<<8,
		MSM_CAMERA_I2C_WORD_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x304A,0x0200,
		MSM_CAMERA_I2C_WORD_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x304A,0x0010,
		MSM_CAMERA_I2C_WORD_DATA);
	do{
		e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x304A,&temp,
			MSM_CAMERA_I2C_WORD_DATA);
		if(0x60 == (temp & 0x60)){
			pr_err("%s: read success\n", __func__);
			break;
		}
		usleep_range(5000, 5100);
		i++;
	}while(i < 10);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3800,&temp,
		MSM_CAMERA_I2C_WORD_DATA);
	pr_err("%s: 0x%x   = 0x%d\n",__func__,block_type,temp);
	return temp;
}

/*
  * camera sensor module compatile
  * 
  * by ZTE_YCM_20140728 yi.changming 000028
  */
// --->
static int lookupIndexByid(MODULE_Map_Table arr[], int len, uint16_t value)
{
    int i = 0;
    for (i = 0; i < len; i++) {
        if (arr[i].id == value) {
            return i;
        }
    }
    return -1;
}
static void parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl,
							MODULE_Map_Table *map,uint16_t len,uint16_t  sensor_module_id)
{
	int index = lookupIndexByid(map,len,sensor_module_id);
	if(index != -1){
		e_ctrl->sensor_module_name = map[index].sensor_module_name;
		e_ctrl->chromtix_lib_name = map[index].chromtix_lib_name;
		e_ctrl->default_chromtix_lib_name = map[index].default_chromtix_lib_name;
		pr_err("ZTE_CAMERA:%s:%d:sensor_module_name = %s\n",
			__func__,__LINE__,e_ctrl->sensor_module_name);
	}else{
		pr_err("ZTE_CAMERA:%s:%d:unknow module id : %d\n",
			__func__,__LINE__,sensor_module_id);
	}
}
// <---
static int ar0542_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	int found = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;
/*
  * camera sensor module compatile
  * 
  * by ZTE_YCM_20140728 yi.changming 000028
  */
// --->
	uint16_t  sensor_module_id = 0;
// <---

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}
	eb_info = e_ctrl->eboard_info;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x301A,0x0610,
		MSM_CAMERA_I2C_WORD_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3134,0xCD95,
		MSM_CAMERA_I2C_WORD_DATA);
	if(0x0002 == ar0542_check_block_status(e_ctrl,0x31)){
		found = 1;
	}else if(0x0001 == ar0542_check_block_status(e_ctrl,0x30)){
		found = 1;
	}else{
		found = 0;
		pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return -1;
	}
	if(found == 1){
		e_ctrl->i2c_client.addr_type = emap[0].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[0].mem.addr,
				memptr, emap[0].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[0].mem.valid_size;
	}
	
/*
  * camera sensor module compatile
  * 
  * by ZTE_YCM_20140728 yi.changming 000028
  */
// --->
	sensor_module_id = (block->mapdata[0]<<8) |block->mapdata[1];
	 
	parse_module_name(e_ctrl,AR0542_MODULE_MAP,
		sizeof(AR0542_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
// <---	
	found = 0;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	if(0x0002 == ar0542_check_block_status(e_ctrl,0x33)){
		found = 1;
	}else if(0x0001 == ar0542_check_block_status(e_ctrl,0x32)){
		found = 1;
	}else{
		found = 0;
		pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return -1;
	}
	if(found == 1){
		e_ctrl->i2c_client.addr_type = emap[1].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[1].mem.addr,
				memptr, emap[1].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[1].mem.valid_size;
		e_ctrl->i2c_client.addr_type = emap[2].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[2].mem.addr,
				memptr, emap[2].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[2].mem.valid_size;
		e_ctrl->i2c_client.addr_type = emap[3].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[3].mem.addr,
				memptr, emap[3].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[3].mem.valid_size;
	}
	
	return rc;
}
/*
  * add imx219 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// --->
enum {
	Invlid_Group,
	Group_One,
	Group_Two,
	Group_Three,
}Group_t;
void imx219_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl)
{

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x0100,0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x4539,0xff,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3453,0x0a,
		MSM_CAMERA_I2C_BYTE_DATA);
	
	udelay(25);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3302,0x02,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3303,0x58,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x012A,0x18,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x012B,0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
}
void imx219_ecc_enable_disable(struct msm_eeprom_ctrl_t *e_ctrl, int enable)
{

	if(enable)
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3300,0x00,
			MSM_CAMERA_I2C_BYTE_DATA);
	else
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3300,0x08,
			MSM_CAMERA_I2C_BYTE_DATA);		
}

void imx219_enable_read_mode(struct msm_eeprom_ctrl_t *e_ctrl, int enable)
{
	if(enable)
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3200,0x01,
			MSM_CAMERA_I2C_BYTE_DATA);
	else
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3200,0x00,
			MSM_CAMERA_I2C_BYTE_DATA);
}

void imx219_set_page(struct msm_eeprom_ctrl_t *e_ctrl,uint16_t page_number)
{
       	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3202,page_number,
		MSM_CAMERA_I2C_BYTE_DATA);

}

int32_t imx219_check_awb_group(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

       imx219_set_page(e_ctrl,0);
	   
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3204,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_One;

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3224,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_Two;


	imx219_set_page(e_ctrl,1);
	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3204,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_Three;
	
	return Invlid_Group;
}

int32_t imx219_check_af_group(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

       imx219_set_page(e_ctrl,1);
	   
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3224,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_One;

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x322b,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_Two;

	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3232,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x03) ==0x01)
		return Group_Three;
	
	return Invlid_Group;
}

int32_t imx219_check_lsc_group(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

       imx219_set_page(e_ctrl,10);
	   
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3213,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	if((temp & 0x0f) ==0x07){
		return Group_Two;
	}else if((temp & 0x0f) ==0x00){
		
	 	imx219_set_page(e_ctrl,4);

		e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3235,&temp,
				MSM_CAMERA_I2C_BYTE_DATA);
		if((temp & 0x03) ==0x01)
			return Group_One;
	}
	
	return Invlid_Group;
}

static int imx219_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;

	uint16_t  sensor_module_id = 0;

	int32_t group_number;

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}
	eb_info = e_ctrl->eboard_info;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	
       imx219_read_eeprom_init(e_ctrl);

	imx219_ecc_enable_disable(e_ctrl,0);

	imx219_enable_read_mode(e_ctrl,1);

	group_number = imx219_check_awb_group(e_ctrl);
	if(group_number == Invlid_Group){
		*memptr = 0;
		memptr++;
		memptr += emap[0].mem.valid_size -1;
		pr_err("%s:%d: awb cal invilid\n", __func__,__LINE__);
	}else{
		*memptr = 1;
		memptr++;
		if(group_number == Group_Two)
			emap[0].mem.addr = 0x3225;
		
		e_ctrl->i2c_client.addr_type = emap[0].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[0].mem.addr,
				memptr, emap[0].mem.valid_size-1);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[0].mem.valid_size -1;
	}

	group_number = imx219_check_af_group(e_ctrl);
	if(group_number == Invlid_Group){
	       *memptr = 0;
		memptr++;
		memptr += emap[1].mem.valid_size -1;
		pr_err("%s:%d: af cal invilid\n", __func__,__LINE__);
	}else{
		*memptr = 1;
		memptr++;
		switch(group_number){
			case Group_One:
				emap[1].mem.addr = 0x3225;
				break;
			case Group_Two:
				emap[1].mem.addr = 0x322c;
				break;
			case Group_Three:
				emap[1].mem.addr = 0x3233;
				break;
			default:
				break;
		}
		
		e_ctrl->i2c_client.addr_type = emap[1].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[1].mem.addr,
				memptr, emap[1].mem.valid_size-1);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
			return rc;
		}
		memptr += emap[1].mem.valid_size -1;
	}
	
	imx219_ecc_enable_disable(e_ctrl,1);
	imx219_enable_read_mode(e_ctrl,1);
	
	group_number = imx219_check_lsc_group(e_ctrl);
	if(group_number == Invlid_Group){
	       *memptr++ = 0;
		memptr += emap[2].mem.valid_size -1;
		pr_err("%s:%d: lsc cal invilid\n", __func__,__LINE__);
	}else{
		*memptr++ = 1;

		*memptr++ = 0x01;
		*memptr++ = 0x00;
		*memptr++ = 0x01;
		
		if(group_number ==Group_One)
			*memptr++ = 0x00;
		else
			*memptr++ = 0x02;
		
		*memptr++ = 0x00;
		*memptr++ = 0x00;
		*memptr++ = 0x03;
	}
	
	if(block->mapdata[0] == 1){
		sensor_module_id = block->mapdata[1];
		parse_module_name(e_ctrl,IMX219_MODULE_MAP,
			sizeof(IMX219_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
	}
	return rc;
}
// <---000064

/*
  * add t4k35 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// --->
#define  T4K35_PAGE_LEN 64
#define  T4k35_OTP_DATA_BEGIN_ADDR 0x3504

void t4k35_enable_read_mode(struct msm_eeprom_ctrl_t *e_ctrl, int enable)
{
	if(enable)
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3500,0x01,
			MSM_CAMERA_I2C_BYTE_DATA);
	else
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3500,0x00,
			MSM_CAMERA_I2C_BYTE_DATA);
}

void t4k35_set_page(struct msm_eeprom_ctrl_t *e_ctrl,uint16_t page_number)
{
       	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3502,page_number,
		MSM_CAMERA_I2C_BYTE_DATA);

}

void t4k35_check_start_acces(struct msm_eeprom_ctrl_t *e_ctrl)
{
       uint16_t temp;	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3500,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3500,temp|0x80,
		MSM_CAMERA_I2C_BYTE_DATA);
	usleep_range(30,30);
}

int32_t t4k35_read_page_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_t *emap,uint8_t *buff)
{
	int rc = 0;
	
	e_ctrl->i2c_client.addr_type = emap->mem.addr_t;
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
			&(e_ctrl->i2c_client), T4k35_OTP_DATA_BEGIN_ADDR,
			buff, T4K35_PAGE_LEN);
	if (rc < 0) {
		pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return rc;
	}
	
	return 0;
}

void t4k35_read_page_and_back_page_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_t *emap,int page,uint8_t *buff_data,uint8_t *bake_buff_data)
{
	int i = 0;
	t4k35_set_page(e_ctrl,page);

	t4k35_check_start_acces(e_ctrl);

	t4k35_read_page_data(e_ctrl,emap,buff_data);

	t4k35_set_page(e_ctrl,page+6);

	t4k35_check_start_acces(e_ctrl);

	t4k35_read_page_data(e_ctrl,emap,bake_buff_data);

       for(i = 0; i < T4K35_PAGE_LEN; i++) 
       {
         CDBG("%s:page = %d:  buff_data[%d]= %d \n",__func__,page,i,buff_data[i]);
	  CDBG("%s:page = %d:  bake_data[%d]= %d \n",__func__,page+6,i,bake_buff_data[i]);
	  buff_data[i]=buff_data[i]|bake_buff_data[i];
       }
}

int32_t t4k35_read_module_info_golden_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_t *emap,uint8_t *memptr,
	uint8_t *buff_data,uint8_t *bake_buff_data)
{
	int32_t check_sum = 0;
	uint8_t segment_flag = 0;
	int32_t i= 0;

	CDBG("%s:start \n",__func__);
	t4k35_read_page_and_back_page_data(e_ctrl,emap,4,buff_data,bake_buff_data);
	CDBG("%s:end \n",__func__);

	if(buff_data[32])
	  segment_flag=32;
      else if(buff_data[0])
  	  segment_flag=0;
      else{
  	  printk("otp no module information!\n");
  	  return -1;
      }

  	for(i = segment_flag+2; i <segment_flag+T4K35_PAGE_LEN/2; i++) 
     		check_sum = check_sum+buff_data[i];


     if((check_sum&0xFF) == buff_data[segment_flag+1]){
	     printk("otp module info checksum ok!\n");
	     *memptr = 1;
	     memptr++;
  	      for(i = segment_flag+3; i <= segment_flag+12; i++){
     		  *memptr = buff_data[i];
		   memptr++;
  	      }
		*memptr = 1;
		memptr++;
  	      for(i = segment_flag+15; i <= segment_flag+22; i++){
     		  *memptr = buff_data[i];
		   memptr++;
  	      }
	      return 0;
    }else{
	   printk("otp module info checksum error!\n");
	  return -1;
   }
}

int32_t t4k35_read_lsc_awb_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_t *emap,uint8_t *memptr,
	uint8_t *buff_data,uint8_t *bake_buff_data)
{
	int32_t check_sum = 0;
	int32_t i = 0,j = 0;

	for(i = 3; i >= 0; i--){
		CDBG("%s:start i = %d\n",__func__,i);
		t4k35_read_page_and_back_page_data(e_ctrl,emap,i,buff_data,bake_buff_data);
		CDBG("%s:end i = %d \n",__func__,i);
    		if (0 == buff_data[0]) {
			memset(buff_data,T4K35_PAGE_LEN,0);
			memset(bake_buff_data,T4K35_PAGE_LEN,0);
      			continue;
	      }else {
	 	 	for(j = 2; j < 64; j++) 
        			check_sum=check_sum+buff_data[j];
	  
	  		if((check_sum & 0xFF) == buff_data[1]){
	  			printk("otp lsc checksum ok!\n");
				*memptr = 1;
	     			memptr++;
				for(j=3;j<=63;j++){
					*memptr = buff_data[j];
		      		 	memptr++;
				}
				return 0;
	  		}else{
				printk("otp lsc checksum error!\n");
				*memptr = 0;;
				return -1;
	  		}
    		}
	}

  	if (i < 0 ) {
	      printk("No otp lsc data on sensor t4k35\n");
	    	*memptr = 0;  	
             return -1;
  	}

       return 0;

}

int32_t t4k35_read_af_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_t *emap,uint8_t *memptr,
	uint8_t *buff_data,uint8_t *bake_buff_data)
{
	int32_t check_sum = 0;
	uint8_t segment_flag = 0;
	int32_t i = 0,j = 0;

	CDBG("%s:start \n",__func__);
	t4k35_read_page_and_back_page_data(e_ctrl,emap,5,buff_data,bake_buff_data);
	CDBG("%s:end \n",__func__);

  //macro AF
  //check flag
  	if(buff_data[24])
	  segment_flag=24;
  	else if(buff_data[16])
  	  segment_flag=16;
  	else if(buff_data[8])
  	  segment_flag=8;
  	else if(buff_data[0])
  	  segment_flag=0;
  	else{
  	  printk("no otp macro AF information!\n");
	  *memptr = 0;
	  memptr += 3;
	  *memptr = 0;
	  memptr += 3;
  	  return -1;
  	}

  	for(i = segment_flag+2; i <= segment_flag+7; i++) {
     		check_sum=check_sum+buff_data[i];
  	}
	  
  	if((check_sum&0xFF)==buff_data[segment_flag+1]){
		printk("otp macro AF checksum ok!\n");
		*memptr = 1;
	     	memptr++;
		for(j=segment_flag+3;j<=segment_flag+4;j++){
			*memptr = buff_data[j];
		      	memptr++;
		}
  	}else{
		printk("otp macro AF checksum error!\n");
    		*memptr = 0;;
		memptr += 3;
  	}

		
  	//inifity AF
  	//check flag
  	if(buff_data[56])
	  segment_flag=56;
  	else if(buff_data[48])
  	  segment_flag=48;
  	else if(buff_data[40])
  	  segment_flag=40;
  	else if(buff_data[32])
  	  segment_flag=32;
  	else{
  	  printk("no otp inifity AF information!\n");
	  *memptr = 0;
	  memptr += 3;
  	  return -1;
  	}

  	//checking check sum
  	check_sum=0; 
  	for(i = segment_flag+2; i <= segment_flag+7; i++) {
		printk("otp inifity AF  %d!\n",buff_data[i]);
     		check_sum +=buff_data[i];
  	}
	  
  	if((check_sum&0xFF)==buff_data[segment_flag+1]){
        	printk("otp inifity AF checksum ok!\n");
		*memptr = 1;
	     	memptr++;
		for(j=segment_flag+4;j<=segment_flag+5;j++){
			*memptr = buff_data[j];
		      	memptr++;
		}
  	}else{
		printk("otp inifity AF checksum error!\n");
    		*memptr = 0;
  	}
       return 0;

}

static int t4k35_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;
       int32_t num_byte = T4K35_PAGE_LEN;
	uint8_t *buff_data;
	uint8_t *bake_buff_data;
	uint16_t  sensor_module_id = 0;

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}
	
	buff_data = kzalloc(num_byte, GFP_KERNEL);
	memset(buff_data,num_byte,0);
	bake_buff_data = kzalloc(num_byte, GFP_KERNEL);
	memset(bake_buff_data,0,num_byte);
	
	eb_info = e_ctrl->eboard_info;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	
       t4k35_enable_read_mode(e_ctrl,1);

	rc = t4k35_read_module_info_golden_data(e_ctrl,&(emap[0]), memptr,buff_data,bake_buff_data);

	if(rc){
		  printk("%s:read module information fail\n",__func__);
	}else{
		if(block->mapdata[0] == 1){
			sensor_module_id = block->mapdata[4];
			parse_module_name(e_ctrl,T4K35_MODULE_MAP,
				sizeof(T4K35_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
		}
	}
	
	memset(buff_data,0,num_byte);
	memset(bake_buff_data,0,num_byte);
	
	memptr += emap[0].mem.valid_size;

	rc = t4k35_read_lsc_awb_data(e_ctrl,&(emap[1]), memptr,buff_data,bake_buff_data);
	if(rc){
		  printk("%s:read lsc awb info fail\n",__func__);
	}

      if(block->num_map >= 3){	
		memset(buff_data,0,num_byte);
		memset(bake_buff_data,0,num_byte);
	
		memptr += emap[1].mem.valid_size;

     
		t4k35_read_af_data(e_ctrl,&(emap[2]), memptr,buff_data,bake_buff_data);
		if(rc){
		  printk("%s:read af info fail\n",__func__);
		}
      	}
	
	t4k35_enable_read_mode(e_ctrl,0);

	kfree(buff_data);
	kfree(bake_buff_data);
	return rc;
}
// <---000064

 int imx214_slave_id[6] = {0xa0,0xa4,0xa6,0xa8,0xaa,0xac};	
static int imx214_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;
	int i,j;

       CDBG("%s:E\n",__func__);
	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	for ( j = 0; j < block->num_map; j++) {
		e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
		e_ctrl->i2c_client.cci_client->sid = imx214_slave_id[j] >> 1 ;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
					&(e_ctrl->i2c_client), emap[0].mem.addr,
					memptr, emap[j].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return rc;
		}
            for( i = 0; i < emap[j].mem.valid_size; i++) 
        	 CDBG("%s: block = %d:  data[%d]= %d \n",__func__,j,i ,memptr[i]);
			
		memptr += emap[j].mem.valid_size;
            
	}
	sensor_module_id = block->mapdata[0];
	parse_module_name(e_ctrl,IMX214_MODULE_MAP,
			sizeof(IMX214_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
       CDBG("%s:X\n",__func__);
	return rc;
}

 int s5k3l2_slave_id[6] = {0xa0,0xa4,0xa6,0xa8,0xaa,0xac};	
static int s5k3l2_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;
	int i,j;

       CDBG("%s:E\n",__func__);
	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	for ( j = 0; j < block->num_map; j++) {
		e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
		e_ctrl->i2c_client.cci_client->sid = s5k3l2_slave_id[j] >> 1 ;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
					&(e_ctrl->i2c_client), emap[0].mem.addr,
					memptr, emap[j].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return rc;
		}
            for( i = 0; i < emap[j].mem.valid_size; i++) 
        	 CDBG("%s: block = %d:  data[%d]= %d \n",__func__,j,i ,memptr[i]);
			
		memptr += emap[j].mem.valid_size;
            
	}
	sensor_module_id = block->mapdata[0];
	parse_module_name(e_ctrl,S5K3L2_MODULE_MAP,
			sizeof(S5K3L2_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
       CDBG("%s:X\n",__func__);
	return rc;
}

void ov8856_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x5000,&temp,
		MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s:0x5000 temp=0x%X",__func__,temp);
	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x5001,&temp,
		MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s:0x5001 temp=0x%X",__func__,temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x0100,0x01,
		MSM_CAMERA_I2C_BYTE_DATA);
	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x5001,(0x00&0x08)|(temp&(~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);
	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d84,0xc0,
		MSM_CAMERA_I2C_BYTE_DATA);
		e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x3d84,&temp,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d88,0x70,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d89,0x10,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d8a,0x72,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d8b,0x0a,
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x3d81,0x01,
		MSM_CAMERA_I2C_BYTE_DATA);	
	udelay(5);
}

void ov8856_read_eeprom_end(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x5001,&temp,
		MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s:0x5001 temp=0x%X",__func__,temp);
	
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x5001,(0x08&0x08)|(temp&(~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),0x0100,0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
}
static int ov8856_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  flag = 0;

	if(((e_ctrl->cal_data.mapdata[0x7010 - 0x7010] &0xC0) != 0x40) &&
		((e_ctrl->cal_data.mapdata[0x7010 - 0x7010] &0x30) != 0x10)){
 	     pr_err("%s :%d: AWB flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[48]);
		flag |= 0x1;
	}
#if 0
	if(((e_ctrl->cal_data.mapdata[0x7021 - 0x7010] &0xC0) != 0x40) &&
		((e_ctrl->cal_data.mapdata[0x7021 - 0x7010] &0x30) != 0x10)){
 	     pr_err("%s :%d: VCM flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[64]);
		flag |= 0x2;
	}
#endif
	if(((e_ctrl->cal_data.mapdata[0x7028 - 0x7010] &0xC0) != 0x40) &&
		((e_ctrl->cal_data.mapdata[0x7028 - 0x7010] &0x30) != 0x10)){
 	     pr_err("%s :%d: LSC flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[80]);
		flag |= 0x4;
	}

 	pr_err("%s :%d: valid info flag = 0x%x  %s\n", __func__,__LINE__,flag,(flag == 0)?"true":"false"); 

	return flag;
		
}
int32_t ov8856_check_module_info_group(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;
	   
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),0x7010,&temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s:temp=0x%X",__func__,temp);
	if((temp&0xC0) == 0x40)
	{
		return Group_One;
	}else if((temp&0x30) == 0x10)
	{
		return Group_Two;
	}
	return Invlid_Group;
}

static int ov8856_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;
	int32_t group_number;
	uint32_t module_id_addr;
	int i;
	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}
	pr_err("%s begin", __func__);
	eb_info = e_ctrl->eboard_info;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	//init before read eeprom
	ov8856_read_eeprom_init(e_ctrl);

	group_number = ov8856_check_module_info_group(e_ctrl);
	switch(group_number){
		case Group_One:
			module_id_addr = 0x7011;
			break;
		case Group_Two:
			module_id_addr = 0x7019;
			break;
		default:
			break;
		}
	if(module_id_addr!=0){
		e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),module_id_addr,&sensor_module_id,
			MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("sensor_module_id =0x%X\n",sensor_module_id);
		parse_module_name(e_ctrl,OV8856_MODULE_MAP,
			sizeof(OV8856_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
	}
	for (i = 0; i< block->num_map; i++) {
		e_ctrl->i2c_client.addr_type = emap[i].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[i].mem.addr,
				memptr, emap[i].mem.valid_size);
		if (rc < 0) {
			pr_err("%s: read failed\n", __func__);
			return rc;
		}
		memptr += emap[i].mem.valid_size;
	}
	e_ctrl->valid_flag = ov8856_validflag_check_eeprom(e_ctrl);
	e_ctrl->checksum = 0;
	ov8856_read_eeprom_end(e_ctrl);
	pr_err("%s end", __func__);
	return rc;
}

static int s5k2t8_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  checksum = 0;
	int j;
	int rc = 0;

 	pr_err("%s :%d: PCB version  = 0x%x", __func__,__LINE__,e_ctrl->cal_data.mapdata[0x001c]);

	for (j = 0; j < 41; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[42]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[41])){
 	     pr_err("%s :%d: Basic info checksum fail", __func__,__LINE__);
	     rc  |= 0x1;
	}
	checksum = 0;

	for (j = 48; j < 53; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[54]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[53])){
 	     pr_err("%s :%d: af info  checksum fail", __func__,__LINE__);
	     rc  |= 0x2;
	}
	checksum = 0;

	for (j = 64; j < 78; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[79]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[78])){
 	     pr_err("%s :%d: awb info checksum fail", __func__,__LINE__);        
	     rc  |= 0x4;
	}
	checksum = 0;

	for (j = 80; j < 1190; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[1191]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[1190])){
 	     pr_err("%s :%d: lsc info checksum fail", __func__,__LINE__);        
	     rc  |= 0x8;
	}
	checksum = 0;

	for (j = 1200; j < 1239; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[1240]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[1239])){
 	     pr_err("%s :%d: ois info checksum fail", __func__,__LINE__);        
	     rc  |= 0x10;
	}
	checksum = 0;

	for (j = 1248; j < 5368; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[5369]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[5368])){
 	     pr_err("%s :%d: pdaf info checksum fail", __func__,__LINE__);        
	     rc  |= 0x20;
	}
	checksum = 0;

	for (j = 0; j < 5370; j++){
	     checksum += e_ctrl->cal_data.mapdata[j];
	}
       if(((checksum & 0xff) != e_ctrl->cal_data.mapdata[5371]) ||
	   	(((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[5370])){
 	     pr_err("%s :%d: total info checksum fail", __func__,__LINE__);        
	     rc  |= 0x40;
	}
 	pr_err("%s :%d: cal info checksum rc = 0x%x\n", __func__,__LINE__,rc);  
       return rc;
}
static int s5k2t8_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  flag = 0;

	if(((e_ctrl->cal_data.mapdata[48] >> 6) & 0x3) != 0x1){
 	     pr_err("%s :%d: AF flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[48]);
		flag |= 0x1;
	}

	if(((e_ctrl->cal_data.mapdata[64] >> 6) & 0x3) != 0x1){
 	     pr_err("%s :%d: AWB flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[64]);
		flag |= 0x2;
	}

	if(((e_ctrl->cal_data.mapdata[80] >> 6) & 0x3) != 0x1){
 	     pr_err("%s :%d: LSC flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[80]);
		flag |= 0x4;
	}
	
	if(((e_ctrl->cal_data.mapdata[1200] >> 6) & 0x3) != 0x1){
 	     pr_err("%s :%d: OIS flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[1200]);
		flag |= 0x8;
	}

	if(((e_ctrl->cal_data.mapdata[1248] >> 6) & 0x3) != 0x1){
 	     pr_err("%s :%d: PDAF flag invalid 0x%x\n", __func__,__LINE__,e_ctrl->cal_data.mapdata[1248]);
		flag |= 0x10;
	}

 	pr_err("%s :%d: valid info flag = 0x%x  %s\n", __func__,__LINE__,flag,(flag == 0)?"true":"false"); 

	return flag;
		
}
static int s5k2t8_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;
	int i,j;

       CDBG("%s:E\n",__func__);
	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	for ( j = 0; j < block->num_map; j++) {
		e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
					&(e_ctrl->i2c_client), emap[0].mem.addr,
					memptr, emap[j].mem.valid_size);
		if (rc < 0) {
			pr_err("%s:%d: read failed\n", __func__,__LINE__);
		return rc;
		}
            for( i = 0; i < emap[j].mem.valid_size; i++) 
        	 CDBG("%s: block = %d:  data[%d]= %d \n",__func__,j,i ,memptr[i]);
			
		memptr += emap[j].mem.valid_size;
            
	}
	e_ctrl->checksum = s5k2t8_checksum_eeprom(e_ctrl);
	e_ctrl->valid_flag= s5k2t8_validflag_check_eeprom(e_ctrl);
	sensor_module_id = block->mapdata[0];
	parse_module_name(e_ctrl,S5K2T8_MODULE_MAP,
			sizeof(S5K2T8_MODULE_MAP)/sizeof(MODULE_Map_Table),sensor_module_id);
       CDBG("%s:X\n",__func__);
	return rc;
}

static int zte_eeprom_generate_map(struct device_node *of,
				       struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	char property[PROPERTY_MAXSIZE];
	uint32_t count = 6;
	struct msm_eeprom_memory_map_t *map;
	snprintf(property, PROPERTY_MAXSIZE, "zte,num-blocks");
	rc = of_property_read_u32(of, property, &data->num_map);
	CDBG("%s: %s %d\n", __func__, property, data->num_map);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}
	map = kzalloc((sizeof(*map) * data->num_map), GFP_KERNEL);
	if (!map) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	data->map = map;
	for (i = 0; i < data->num_map; i++) {
		snprintf(property, PROPERTY_MAXSIZE, "zte,mem%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].mem, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}
		data->num_data += map[i].mem.valid_size;
	}
	CDBG("%s num_bytes %d\n", __func__, data->num_data);
	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;
ERROR:
	kfree(data->map);
	memset(data, 0, sizeof(*data));
	return rc;
}

static const struct of_device_id zte_eeprom_dt_match[] = {
	{ .compatible = "zte,eeprom-ar0542", .data = (void *)ar0542_read_eeprom_memory },
/*
  * add imx219 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// ---> 
	{ .compatible = "zte,eeprom-imx219", .data = (void *)imx219_read_eeprom_memory },
// <---000064
/*
  * add imx219 eeprom driver
  * 
  * by ZTE_YCM_20140911 yi.changming 000064
  */
// ---> 
	{ .compatible = "zte,eeprom-t4k35", .data = (void *)t4k35_read_eeprom_memory },
// <---000064
	{ .compatible = "zte,eeprom-imx214", .data = (void *)imx214_read_eeprom_memory },
	{ .compatible = "zte,eeprom-s5k3l2", .data = (void *)s5k3l2_read_eeprom_memory },
	{ .compatible = "zte,eeprom-ov8856", .data = (void *)ov8856_read_eeprom_memory },
	{ .compatible = "zte,eeprom-s5k2t8", .data = (void *)s5k2t8_read_eeprom_memory },
	{ }
};

static int zte_eeprom_platform_probe(struct platform_device *pdev)
{
	int rc = 0;
	int j = 0;
	uint32_t temp;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_eeprom_board_info *eb_info = NULL;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	const struct of_device_id *match;
	zte_read_eeprom_memory_func_t zte_read_eeprom_memory = NULL;
	CDBG("%s E\n", __func__);
	match = of_match_device(zte_eeprom_dt_match, &pdev->dev);
	zte_read_eeprom_memory = (zte_read_eeprom_memory_func_t)match->data;
	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;
	e_ctrl->is_supported = 0;
	if (!of_node) {
		pr_err("%s dev.of_node NULL\n", __func__);
		rc = -EINVAL;
		goto ectrl_free;
	}
	rc = of_property_read_u32(of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	e_ctrl->subdev_id = pdev->id;
	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&e_ctrl->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", e_ctrl->cci_master, rc);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}
	rc = of_property_read_u32(of_node, "qcom,slave-addr",
		&temp);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}
	e_ctrl->pdev = pdev;
	e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_cci_func_tbl;
	e_ctrl->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!e_ctrl->i2c_client.cci_client) {
		pr_err("%s failed no memory\n", __func__);
		rc = -ENOMEM;
		goto ectrl_free;
	}
	e_ctrl->eboard_info = kzalloc(sizeof(
		struct msm_eeprom_board_info), GFP_KERNEL);
	if (!e_ctrl->eboard_info) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto cciclient_free;
	}
	eb_info = e_ctrl->eboard_info;
	power_info = &eb_info->power_info;
	eb_info->i2c_slaveaddr = temp;

	power_info->dev = &pdev->dev;
	/*Get clocks information*/
	rc = msm_camera_get_clk_info(e_ctrl->pdev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_clk_info rc %d", rc);
		goto board_free;
	}
	CDBG("qcom,slave-addr = 0x%X\n", eb_info->i2c_slaveaddr);
	cci_client = e_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = e_ctrl->cci_master;
	cci_client->sid = eb_info->i2c_slaveaddr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	rc = of_property_read_string(of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto board_free;
	}
	rc = msm_eeprom_cmm_dts(e_ctrl->eboard_info, of_node);
	if (rc < 0)
		CDBG("%s MM data miss:%d\n", __func__, __LINE__);
	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc)
		goto board_free;
	rc = zte_eeprom_generate_map(of_node, &e_ctrl->cal_data);
	if (rc < 0)
		goto board_free;
	rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}
	rc = zte_read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc < 0) {
		pr_err("%s read_eeprom_memory failed\n", __func__);
		goto power_down;
	}
	for (j = 0; j < e_ctrl->cal_data.num_data; j++)
		CDBG("memory_data[%d] = 0x%X\n", j,
			e_ctrl->cal_data.mapdata[j]);
	e_ctrl->is_supported = 1;
	rc = msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}
	v4l2_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	platform_set_drvdata(pdev, &e_ctrl->msm_sd.sd);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(e_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "zte_eeprom");
	media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
#ifdef CONFIG_COMPAT
	msm_eeprom_v4l2_subdev_fops = v4l2_subdev_fops;
	msm_eeprom_v4l2_subdev_fops.compat_ioctl32 =
		msm_eeprom_subdev_fops_ioctl32;
	e_ctrl->msm_sd.sd.devnode->fops = &msm_eeprom_v4l2_subdev_fops;
#endif
	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	CDBG("%s X\n", __func__);
	return rc;
power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
memdata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
cciclient_free:
	kfree(e_ctrl->i2c_client.cci_client);
ectrl_free:
	kfree(e_ctrl);
	return rc;
}

static int zte_eeprom_platform_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;
	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	if (!e_ctrl->eboard_info) {
		pr_err("%s: eboard_info is NULL\n", __func__);
		return 0;
	}
	msm_camera_put_clk_info(e_ctrl->pdev,
		&e_ctrl->eboard_info->power_info.clk_info,
		&e_ctrl->eboard_info->power_info.clk_ptr,
		e_ctrl->eboard_info->power_info.clk_info_size);

	kfree(e_ctrl->i2c_client.cci_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

static struct platform_driver zte_eeprom_platform_driver = {
	.driver = {
		.name = "zte,eeprom",
		.owner = THIS_MODULE,
		.of_match_table = zte_eeprom_dt_match,
	},
	.remove = zte_eeprom_platform_remove,
};

static int __init zte_eeprom_init_module(void)
{
	int rc = 0;
	CDBG("%s E\n", __func__);
	rc = platform_driver_probe(&zte_eeprom_platform_driver,
		zte_eeprom_platform_probe);
	CDBG("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit zte_eeprom_exit_module(void)
{
	platform_driver_unregister(&zte_eeprom_platform_driver);

}

module_init(zte_eeprom_init_module);
module_exit(zte_eeprom_exit_module);
MODULE_DESCRIPTION("MSM EEPROM driver");
MODULE_LICENSE("GPL v2");
// <---
