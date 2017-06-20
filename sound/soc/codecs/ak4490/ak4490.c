/*
 * ak4490.c  --  audio driver for AK4490
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/08/11	    1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/stat.h>

#include <linux/kernel.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "ak4490.h"


#include <linux/clk.h>
#include <linux/wakelock.h>
#define AK4490_DEBUG			//used at debug mode

#define CONFIG_DEBUG_FS_CODEC_AK4490
#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
#define AK4490_CONTIF_DEBUG		//used at debug mode
#endif

#ifdef AK4490_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

/* AK4490 Codec Private Data */
struct ak4490_priv {
	struct snd_soc_codec codec;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	u8 reg_cache[AK4490_MAX_REGISTERS];
	unsigned int fmt; //lhs
	int fs1;         // Sampling Frequency
	int nBickFreq;   //  0: 48fs for 24bit,  1: 64fs or more for 32bit
	int nDSDSel;
	int pdn_gpio;                                  //AK4490_PDN/<&pmi8994_gpios 1 0>
	int ldoen_gpio;                               //HIFI_LDO_EN/<&tlmm 93 0>
	int isl54405_sw_pwr_gpio;                     //<&pmi8994_gpios 10 0> Hifi swtch power
	int isl98608_ldo_pwr_gpio; 
	int isl54405_mute_gpio;// 0 disable mute , 1 enable mute
	int isl54405_sel_gpio;  // 0 select 4490, 1 select 4961
	int isl_dir_gpio;
	int isl_dc_gpio;
	bool akm_hifi_switch;
	bool akm_hifi_switch_mute;
	bool akm_pw_switch;
	int hifi_pop_gpio;        // 0:connect, 1:cut
};
static struct ak4490_priv *g_ak4490_priv = NULL;

#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
struct snd_soc_codec *ak4490_codec;
#endif
static const struct reg_default ak4490_reg_defaults[] = {
	{  0, 0x04 }, {  1, 0x22 }, {  2, 0x00 }, {  3, 0xFF },
	{  4, 0xFF}, {  5, 0x00 }, {  6, 0x00 }, {  7, 0x00 },
	{  8, 0x00 }, {  9, 0x00 }
};


static const u8 ak4490_reg[AK4490_MAX_REGISTERS] = {
	0x04,	/*	0x00	AK4490_00_CONTROL1			*/
	0x22,	/*	0x01	AK4490_01_CONTROL2			*/
	0x00,	/*	0x02	AK4490_02_CONTROL3			*/
	0xFF,	/*	0x03	AK4490_03_LCHATT			*/
	0xFF,	/*	0x04	AK4490_04_RCHATT			*/
	0x00,	/*	0x05	AK4490_05_CONTROL4			*/
	0x00,	/*	0x06	AK4490_06_CONTROL5			*/
	0x00,	/*	0x07	AK4490_07_CONTROL6			*/
	0x00,	/*	0x08	AK4490_08_CONTROL7			*/
	0x00,	/*	0x09	AK4490_09_CONTROL8			*/
};

static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4490_access_masks[] = {
    { 0xFF, 0xEF },	//0x00
    { 0xFF, 0xFF },	//0x01
    { 0xFF, 0xBF },	//0x02
    { 0xFF, 0xFF },	//0x03
    { 0xFF, 0xFF },	//0x04
    { 0xFF, 0xC3 },	//0x05
    { 0xFF, 0xFB },	//0x06
    { 0xFF, 0x01 },	//0x07
    { 0xFF, 0x03 },	//0x08
    { 0xFF, 0x03 },	//0x09
};

/* Volume control:
 * from -127 to 0 dB in 0.5 dB steps (mute instead of -127.5 dB) */
static DECLARE_TLV_DB_SCALE(latt_tlv, -12750, 50, 0);
static DECLARE_TLV_DB_SCALE(ratt_tlv, -12750, 50, 0);

static const char *ak4490_ecs_select_texts[] = {"768kHz", "384kHz"};

static const char *ak4490_dem_select_texts[] = {"44.1kHz", "OFF", "48kHz", "32kHz"};
static const char *ak4490_dzfm_select_texts[] = {"Separated", "ANDed"};

static const char *ak4490_sellr_select_texts[] = {"Rch", "Lch"};
static const char *ak4490_dckb_select_texts[] = {"Falling", "Rising"};
static const char *ak4490_dcks_select_texts[] = {"512fs", "768fs"};

static const char *ak4490_dsdd_select_texts[] = {"Normal", "Volume Bypass"};

static const char *ak4490_sc_select_texts[] = {"Setting 1", "Setting 2", "Setting 3"};
static const char *ak4490_dsdf_select_texts[] = {"50kHz", "150kHz"};


static const struct soc_enum ak4490_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4490_00_CONTROL1, 5,
			ARRAY_SIZE(ak4490_ecs_select_texts), ak4490_ecs_select_texts),

	SOC_ENUM_SINGLE(AK4490_01_CONTROL2, 1,
			ARRAY_SIZE(ak4490_dem_select_texts), ak4490_dem_select_texts),
	SOC_ENUM_SINGLE(AK4490_01_CONTROL2, 6,
			ARRAY_SIZE(ak4490_dzfm_select_texts), ak4490_dzfm_select_texts),

	SOC_ENUM_SINGLE(AK4490_02_CONTROL3, 1,
			ARRAY_SIZE(ak4490_sellr_select_texts), ak4490_sellr_select_texts),
	SOC_ENUM_SINGLE(AK4490_02_CONTROL3, 4,
			ARRAY_SIZE(ak4490_dckb_select_texts), ak4490_dckb_select_texts),
	SOC_ENUM_SINGLE(AK4490_02_CONTROL3, 5,
			ARRAY_SIZE(ak4490_dcks_select_texts), ak4490_dcks_select_texts),

	SOC_ENUM_SINGLE(AK4490_06_CONTROL5, 1,
			ARRAY_SIZE(ak4490_dsdd_select_texts), ak4490_dsdd_select_texts),

	SOC_ENUM_SINGLE(AK4490_08_CONTROL7, 0,
			ARRAY_SIZE(ak4490_sc_select_texts), ak4490_sc_select_texts),

	SOC_ENUM_SINGLE(AK4490_09_CONTROL8, 1,
			ARRAY_SIZE(ak4490_dsdf_select_texts), ak4490_dsdf_select_texts),

};

static const char *ak4490_dsdsel_select_texts[] = {"2.8224MHz", "5.6448MHz", "11.2896MHz"};
static const char *ak4490_bickfreq_select[] = {"48fs", "64fs"};

static const struct soc_enum ak4490_dac_enum2[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4490_dsdsel_select_texts), ak4490_dsdsel_select_texts), 
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4490_bickfreq_select), ak4490_bickfreq_select), 
};
#ifdef AK4490_CONTIF_DEBUG
static unsigned int ak4490_i2c_read(struct snd_soc_codec *codec, unsigned int reg);
static int ak4490_i2c_write(struct snd_soc_codec *codec, unsigned int reg,unsigned int value);
#endif
extern void isl98608_power_down(void);
extern void isl98608_power_up(void);
extern void ak8157_clock_open(void);
extern void ak8157_clock_close(void);

extern void ak8157_rate_set(int rate);


static int ak4490_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt);
static int ak4490_power_up(struct ak4490_priv *ak4490);
static int ak4490_power_down(struct ak4490_priv *ak4490);




static inline u32 ak4490_read_reg_cache(struct snd_soc_codec *, u16);

static int ak4490_get_dsdsel(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4490->nDSDSel;

    return 0;
}

static int ak4490_set_dsdsel(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);

	ak4490->nDSDSel = ucontrol->value.enumerated.item[0];

	if ( ak4490->nDSDSel == 0 ) { 	//  2.8224MHz
		snd_soc_update_bits(codec, AK4490_06_CONTROL5, 0x01, 0x00);
		snd_soc_update_bits(codec, AK4490_09_CONTROL8, 0x01, 0x00);
	}
	else if( ak4490->nDSDSel == 1 ) {	// 5.6448MHz
		snd_soc_update_bits(codec, AK4490_06_CONTROL5, 0x01, 0x01);
		snd_soc_update_bits(codec, AK4490_09_CONTROL8, 0x01, 0x00);
	}
	else {								// 11.2896MHz
	snd_soc_update_bits(codec, AK4490_09_CONTROL8, 0x01, 0x01);
	snd_soc_update_bits(codec, AK4490_09_CONTROL8, 0x01, 0x00);
	}
	
	return 0;
}

static int ak4490_get_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4490->nBickFreq;

    return 0;
}

static int ak4490_set_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
    
	ak4490->nBickFreq = ucontrol->value.enumerated.item[0];
	
    return 0;
}

#ifdef AK4490_DEBUG

static const char *test_reg_select[]   = 
{
    "read AK4490",
};

static const struct soc_enum ak4490_enum[] = 
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;
}

static int set_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
	int    i, value;
	int	   regs, rege;

	nTestRegNo = currMode;

	regs = 0x00;
	rege = 0x09;

	for ( i = regs ; i <= rege ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***@AK4490 Addr,Reg=(%x, %x)\n", i, value);

	//	value = ak4490_i2c_read(codec,i);
		//printk("###AK4490 Addr,Reg=(%x, %x)\n", i, value);
	}

	return 0;
}
#endif

static const char *hifi_switch_text[] = {
        "on", "off"
};
 
static const struct soc_enum hifi_switch_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_switch_text), hifi_switch_text);


static const char *hifi_switch_ctl_text[] = {
        "ak4490", "ak4961"
};
 
static const struct soc_enum hifi_switch_ctl_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hifi_switch_ctl_text), hifi_switch_ctl_text);



static int akm_get_hifi_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4490->akm_hifi_switch;
 
        return 0;
}
 
static int akm_set_hifi_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
	 u8      status = ucontrol->value.integer.value[0];
//	 int ret;

/*	 pr_err("[LHS]%s ## status=%d\n",__func__,status);
	 if(status==1)
	 	{
	 		ret = gpio_direction_output(ak4490->isl54405_sel_gpio, 1);
			if (ret < 0) {
				pr_err("%s(): ak4490_isl54405_sel_gpio direction failed %d\n",
				__func__, ret);
				return ret;
				}
	 	}
	 else if(status == 0)
	 	{
	 		 ret = gpio_direction_output(ak4490->isl54405_sel_gpio, 0); //4490
			if (ret < 0) {
				pr_err("%s(): ak4490_isl54405_sel_gpio direction failed %d\n",
				__func__, ret);
				return ret;
				}
	 	}
	 else
	 	{
	 	
	 	}*/
	 ak4490->akm_hifi_switch=status;
	 return 0;
}

static int akm_get_hifi_switch_mute(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4490->akm_hifi_switch_mute;
 
        return 0;
}
 
static int akm_set_hifi_switch_mute(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
	 u8      status = ucontrol->value.integer.value[0];
	 int ret;
	 pr_err("[LHS]%s \n",__func__);

	 if(status==0)
	 	{
	 		ret = gpio_direction_output(ak4490->isl54405_mute_gpio, 1); //mute
			if (ret < 0) {
				pr_err("%s(): ak4490_isl54405_mute_gpio direction failed %d\n",
				__func__, ret);
				return ret;
				}
			msleep(50);
		pr_err("[LHS]%s ak4490->isl54405_mute_gpio is 1,status=%d\n",__func__,status);
	 	}
	  else if(status == 1)
	 	{
	 		msleep(50);
	 		 ret = gpio_direction_output(ak4490->isl54405_mute_gpio, 0);
			if (ret < 0) {
				pr_err("%s(): ak4490_isl54405_mute_gpio direction failed %d\n",
				__func__, ret);
				return ret;
				}
		pr_err("[LHS]%s ak4490->isl54405_mute_gpio is 0,status=%d\n",__func__,status);
	 	}
	 else
	 	{
	 	}
	 ak4490->akm_hifi_switch_mute=status;
	 return 0;
}




static int akm_get_hifi_pw_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4490->akm_pw_switch;
 
        return 0;
}
 
static int akm_set_hifi_pw_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
	 u8      status = ucontrol->value.integer.value[0];
	 int ret;
	 pr_err("[LHS]%s \n",__func__);

	 if(status==0)
	 	{

			ret = ak4490_power_up(ak4490);
			if (ret < 0) {
				pr_err("%s(): ak4490_power_up failed %d\n",__func__, ret);
				return ret;
				}
	 	}
	  else if(status == 1)
	 	{
			ak8157_clock_close();                                                                                       
			msleep(10);
			ret = ak4490_power_down(ak4490);
			if (ret < 0) {
				pr_err("%s(): ak4490_power_down failed %d\n",__func__, ret);
				return ret;
			}
	 	}
	 else
	 	{
			
	 	}
	 ak4490->akm_pw_switch=status;
	 return 0;
}


static const struct snd_kcontrol_new ak4490_snd_controls[] = {
	SOC_SINGLE_TLV("AK4490 Lch Digital Volume",
			AK4490_03_LCHATT, 0, 0xFF, 0, latt_tlv),
	SOC_SINGLE_TLV("AK4490 Rch Digital Volume",
			AK4490_04_RCHATT, 0, 0xFF, 0, ratt_tlv),

	SOC_ENUM("AK4490 EX DF I/F clock", ak4490_dac_enum[0]), 
	SOC_ENUM("AK4490 De-emphasis Response", ak4490_dac_enum[1]), 
	SOC_ENUM("AK4490 Data Zero Detect Mode", ak4490_dac_enum[2]),
	SOC_ENUM("AK4490 Data Selection at Mono Mode", ak4490_dac_enum[3]), 

	SOC_ENUM("AK4490 Polarity of DCLK", ak4490_dac_enum[4]),
	SOC_ENUM("AK4490 DCKL Frequency", ak4490_dac_enum[5]),

	SOC_ENUM("AK4490 DDSD Play Back Path", ak4490_dac_enum[6]),
	SOC_ENUM("AK4490 Sound control", ak4490_dac_enum[7]),
	SOC_ENUM("AK4490 Cut Off of DSD Filter", ak4490_dac_enum[8]),

	SOC_ENUM_EXT("AK4490 DSD Data Stream", ak4490_dac_enum2[0], ak4490_get_dsdsel, ak4490_set_dsdsel),
	SOC_ENUM_EXT("AK4490 BICK Frequency Select", ak4490_dac_enum2[1], ak4490_get_bickfs, ak4490_set_bickfs),

	SOC_ENUM_EXT("AKM HIFI Switch Sel", hifi_switch_ctl_enum,
					akm_get_hifi_switch, akm_set_hifi_switch),

	SOC_ENUM_EXT("AKM HIFI Switch Mute", hifi_switch_enum,
					akm_get_hifi_switch_mute, akm_set_hifi_switch_mute),

	SOC_ENUM_EXT("AKM HIFI PW", hifi_switch_enum,
					akm_get_hifi_pw_switch, akm_set_hifi_pw_switch),


	SOC_SINGLE("AK4490 External Digital Filter", AK4490_00_CONTROL1, 6, 1, 0),
	SOC_SINGLE("AK4490 MCLK Frequncy Auto Setting", AK4490_00_CONTROL1, 7, 1, 0),
	SOC_SINGLE("AK4490 Soft Mute Control", AK4490_01_CONTROL2, 0, 1, 0),
	SOC_SINGLE("AK4490 Short delay filter", AK4490_01_CONTROL2, 5, 1, 0),
	SOC_SINGLE("AK4490 Data Zero Detect Enable", AK4490_01_CONTROL2, 7, 1, 0),
	SOC_SINGLE("AK4490 Slow Roll-off Filter", AK4490_02_CONTROL3, 0, 1, 0),
	SOC_SINGLE("AK4490 Invering Enable of DZF", AK4490_02_CONTROL3, 4, 1, 0),
	SOC_SINGLE("AK4490 Mono Mode", AK4490_02_CONTROL3, 3, 1, 0),
	SOC_SINGLE("AK4490 Super Slow Roll-off Filter", AK4490_05_CONTROL4, 0, 1, 0),
	SOC_SINGLE("AK4490 AOUTR Phase Inverting", AK4490_05_CONTROL4, 6, 1, 0),
	SOC_SINGLE("AK4490 AOUTL Phase Inverting", AK4490_05_CONTROL4, 7, 1, 0),
	SOC_SINGLE("AK4490 DSD Mute Release", AK4490_06_CONTROL5, 3, 1, 0),
	SOC_SINGLE("AK4490 DSD Mute Control Hold", AK4490_06_CONTROL5, 4, 1, 0),
	SOC_SINGLE("AK4490 DSDR is detected", AK4490_06_CONTROL5, 5, 1, 0),
	SOC_SINGLE("AK4490 DSDL is detected", AK4490_06_CONTROL5, 6, 1, 0),
	SOC_SINGLE("AK4490 DSD Data Mute", AK4490_06_CONTROL5, 7, 1, 0),
	SOC_SINGLE("AK4490 Synchronization Control", AK4490_07_CONTROL6, 0, 1, 0),


#ifdef AK4490_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4490_enum[0], get_test_reg, set_test_reg),
#endif

};


/* ak4490 dapm widgets */
static const struct snd_soc_dapm_widget ak4490_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("AK4490 DAC", "NULL", AK4490_00_CONTROL1, 0, 0),
	SND_SOC_DAPM_AIF_IN("AK4490 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("AK4490 AOUT"),
	SND_SOC_DAPM_SUPPLY("AK4490_RX_BIAS", SND_SOC_NOPM, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route ak4490_intercon[] = 
{
	{"AK4490 DAC", "NULL", "AK4490 SDTI"},
	{"AK4490 AOUT", "NULL", "AK4490 DAC"},
	{"AK4490 SDTI", "NULL", "AK4490_RX_BIAS"},
};

static int ak4490_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);

	u8 	dfs;
	u8  dfs2;
	int nfs1;
	unsigned int fmt;
	unsigned int format;

	pr_err("[LHS]%s: sgEntry!\n",__func__);
	nfs1 = params_rate(params);
	 ak8157_rate_set(nfs1);
	 ak8157_clock_open();
	fmt = params_format(params);
	ak4490->fs1 = nfs1;
	ak4490->fmt = fmt;
//	pr_err("[LHS]%s: nfs1=%d , fmt=%x!\n",__func__,nfs1,fmt);

	dfs = snd_soc_read(codec, AK4490_01_CONTROL2);
	dfs &= ~AK4490_DFS;
	
	dfs2 = snd_soc_read(codec, AK4490_05_CONTROL4);
	dfs2 &= ~AK4490_DFS2;
//	pr_err("[LHS]%s: dfs=%x , dfss=%x!\n",__func__,dfs,dfs2);

	switch (nfs1) {
		case 32000:
		case 44100:
		case 48000:
			dfs |= AK4490_DFS_48KHZ;
			dfs2 |= AK4490_DFS2_48KHZ;
			break;
		case 88200:
		case 96000:
			dfs |= AK4490_DFS_96KHZ;
			dfs2 |= AK4490_DFS2_48KHZ;
			break;
		case 176400:
		case 192000:
			dfs |= AK4490_DFS_192KHZ;
			dfs2 |= AK4490_DFS2_48KHZ;
			break;
		case 384000:
			dfs |= AK4490_DFS_384KHZ;
			dfs2 |= AK4490_DFS2_384KHZ;
			break;
		case 768000:
			dfs |= AK4490_DFS_768KHZ;
			dfs2 |= AK4490_DFS2_384KHZ;
			break;
		default:
			return -EINVAL;
	}


	

	snd_soc_write(codec, AK4490_00_CONTROL1, 0x0f);
//	snd_soc_write(codec, AK4490_02_CONTROL3, 0x01);
	snd_soc_write(codec, AK4490_02_CONTROL3, 0x00);
	snd_soc_write(codec, AK4490_01_CONTROL2, dfs);

	snd_soc_write(codec, AK4490_06_CONTROL5, 0x00);
	snd_soc_write(codec, AK4490_07_CONTROL6, 0x00);
        snd_soc_write(codec, AK4490_08_CONTROL7, 0x01);
	snd_soc_write(codec, AK4490_09_CONTROL8, 0x00);
	snd_soc_write(codec, AK4490_05_CONTROL4, dfs2);

	//pr_err("[LHS]%s: 111 dfs=%x , dfss=%x!\n",__func__,dfs,dfs2);

	format = snd_soc_read(codec, AK4490_00_CONTROL1);
	format &= ~AK4490_DIF;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	        pr_err("%s: SNDRV_PCM_FORMAT_S16_LE %u\n", __func__,params_format(params));
		 format |=0x0e; 
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		pr_err("%s: SNDRV_PCM_FORMAT_S24_LE %u\n", __func__,params_format(params));
		format |=0x0e;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		pr_err("%s: SNDRV_PCM_FORMAT_S32_LE %u\n", __func__,params_format(params));
		format |=0x0e;
		break;
	default:
		pr_err("%s: Invalid RX format %u\n", __func__,
				params_format(params));
		return -EINVAL;
	}
	snd_soc_write(codec, AK4490_00_CONTROL1, format);
	//pr_err("[LHS]%s: 111 AK4490_00_CONTROL1=%x!\n",__func__,format);

	//snd_soc_write(codec, AK4490_01_CONTROL2, dfs);
	//snd_soc_write(codec, AK4490_05_CONTROL4, dfs2);


	//ak4490_set_dai_fmt(dai,fmt);

	return 0;
}


static int ak4490_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	//pr_err("[LHS]%s: Entry!\n",__func__);

	return 0;
}


static int ak4490_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
//	struct snd_soc_codec *codec = dai->codec;

	pr_err("[LHS]%s: Entry!\n",__func__);

	return 0;
}

static int ak4490_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
	u8 format;
	u8 format2;

	pr_err("[LHS]%s: Entry!\n",__func__);
	//

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4490_00_CONTROL1);
	format &= ~AK4490_DIF;

	format2 = snd_soc_read(codec, AK4490_02_CONTROL3);
	format2 &= ~AK4490_DIF_DSD;
	pr_err("[LHS] %s format=%x format2=%x \n",__func__,format,format2);
/*    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBS_CFS:
            break;
        case SND_SOC_DAIFMT_CBM_CFM:
        case SND_SOC_DAIFMT_CBS_CFM:
        case SND_SOC_DAIFMT_CBM_CFS:
        default:
            dev_err(codec->dev, "Clock mode unsupported");
           return -EINVAL;
    }
*/
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
		//	format |= AK4490_DIF_I2S_MODE;
		//	if ( ak4490->nBickFreq == 1 ) 	format |= AK4490_DIF_32BIT_MODE;
		//	break;
		case SND_SOC_DAIFMT_LEFT_J:
		case SND_SOC_DAIFMT_RIGHT_J:
			//format |= AK4490_DIF_MSB_MODE;
			format |= AK4490_DIF_I2S_MODE;
			if ( ak4490->nBickFreq == 1 ) 	format |= AK4490_DIF_32BIT_MODE;
			break;
		case SND_SOC_DAIFMT_DSD:
			format2 |= AK4490_DIF_DSD_MODE;
			break;
		default:
			return -EINVAL;
	}

	/* set format */

	snd_soc_write(codec, AK4490_00_CONTROL1, format);
	snd_soc_write(codec, AK4490_02_CONTROL3, format2);
	return 0;
}

static int ak4490_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	int	ret;

	switch (reg) {
//		case :
//			ret = 1;
		default:
			ret = 0;
			break;
	}
	return ret;
}

static int ak4490_readable(struct snd_soc_codec *codec, unsigned int reg)
{

	if (reg >= ARRAY_SIZE(ak4490_access_masks))
		return 0;
	return ak4490_access_masks[reg].readable != 0;
}

/*
* Read ak4490 register cache
 */
static inline u32 ak4490_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = g_ak4490_priv->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4490_reg));
    return (u32)cache[reg];
}

#ifdef AK4490_CONTIF_DEBUG
/*
 * Write ak4490 register cache
 */
static inline void ak4490_write_reg_cache(
struct snd_soc_codec *codec, 
u16 reg,
u16 value)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4490_reg));
    cache[reg] = (u8)value;
}

static unsigned int ak4490_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{

	unsigned int ret;
	ret = i2c_smbus_read_byte_data(g_ak4490_priv->i2c_client, (u8)(reg & 0xFF));

	if (ret < 0) {
		akdbgprt("[LHS][AK4490] %s(%d) ret=%x \n",__FUNCTION__,__LINE__,ret);
	}
	return ret;
}

static int ak4490_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{


	 pr_err("[LHS]%s, reg=%x , data=%x\n",__func__,reg,value);
	ak4490_write_reg_cache(codec, reg, value);

	if(i2c_smbus_write_byte_data(g_ak4490_priv->i2c_client, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		akdbgprt("[lhs][AK4490] %s(%d)\n",__FUNCTION__,__LINE__);
		return EIO;
	}
	
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
static int ak4490_power_enable(bool val);
static ssize_t ak4490reg_data_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	int i,value;
	int regs,rege;
	u8 rx[32]={0};
	int ret = 0;
	int fpt = 0;
	regs=0x00;
	rege=0x0a;
	pr_err("[LHS] %s in\n",__func__);
	ak4490_power_enable(true);
	for(i= regs;i<rege;i++)
		{
		//value=snd_soc_read(ak4490_codec,i);
			value = ak4490_i2c_read(ak4490_codec,i);
			//rx[i]=value ;
			rx[i]=(u8)value ;
		pr_err("[LHS] ak4490 reg show add =%x , data =%x \n", i,value);
			if(value<0)
			break;
		}
	ak4490_power_enable(false);
	if (i == rege) {
		for (i = regs; i < rege; i++, fpt += 6) {
			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
			}
		return ret;
		} 
	else {
		return sprintf(buf, "ak4490 test error!\n");
		}
}

static ssize_t ak4490reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)

{
	char *ptr_data = (char *)buf;
	char *p;
	int i, pt_count = 0;
	unsigned short val[20];
	while((p=strsep(&ptr_data,","))){
		if(!*p)
			break;
		if(pt_count>=20)
			break;
		val[pt_count] = simple_strtoul(p,NULL,16);
		pt_count++;
	}
	for(i=0;i<pt_count;i+=2)
		{
			//snd_soc_write(ak4490_codec,val[i],val[i+1]);
			ak4490_i2c_write(ak4490_codec,val[i],val[i+1]);
			pr_err("[LHS] write reg add=%x , val =%x \n",val[i],val[i+1]);
		}
	return count;
	//return 0;
}

static DEVICE_ATTR(ak4490reg_data, 0664, ak4490reg_data_show, ak4490reg_data_store);
#endif

// * for AK4490
static int ak4490_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	int 	ret = 0;  
//    struct snd_soc_codec *codec = codec_dai->codec;
//    struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);

	//pr_err("[LHS]%s: Entry!  sgcmd =%x \n",__func__,cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	//	msleep(50);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
			break;
	default:
		break;
		}


	return ret;
}


static int ak4490_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	pr_err("%s: Entry!\n",__func__);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int ak4490_set_dai_mute(struct snd_soc_dai *dai, int mute) 
{
    struct snd_soc_codec *codec = dai->codec;
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
	int ret;

	pr_err("[LHS]%s: mute[%s]\n",__func__,mute ? "ON":"OFF");
		
	if (mute) {	//SMUTE: 1 , MUTE
	/* hifi_pop_gpio connected*/
		ret = gpio_direction_output(ak4490->hifi_pop_gpio, 0);
		if (ret < 0) {
			pr_err("%s(): hifi_pop_gpio direction failed %d\n",
			__func__, ret);
			return ret;
		}
		pr_err("[LHS]%s: hifi_pop_gpio is 0 \n",__func__);
		pr_err("%s(): 3: hifi_pop_gpio(%d) value(%d)",
                         __func__, ak4490->hifi_pop_gpio, gpio_get_value_cansleep(ak4490->hifi_pop_gpio));

		msleep(10); // ZTE_chenjun:OK:1000,10,5

		/* change the switch to ak4961 after the music stop*/
		ret = gpio_direction_output(ak4490->isl54405_sel_gpio, 1);
			if (ret < 0) {
				pr_err("%s(): isl54405_sel_gpio direction failed %d\n",
				__func__, ret);
				return ret;
			}
		ak4490->akm_hifi_switch = 1;
		pr_err("[LHS]%s: isl54405_sel_gpio is 1 , set to ak4961!\n",__func__);
	}
	else {		// SMUTE: 0 ,NORMAL operation
		snd_soc_update_bits(codec, AK4490_01_CONTROL2, 0x01, 0x00);
				/* change the switch to ak4490 after the music start*/
		ret = gpio_direction_output(ak4490->isl54405_sel_gpio, 0);
			if (ret < 0) {
				pr_err("%s(): isl54405_sel_gpio direction failed %d\n",
				__func__, ret);
				return ret;
				}
		ak4490->akm_hifi_switch = 0;
		pr_err("[LHS]%s: isl54405_sel_gpio is 0 ,set to ak4490!\n",__func__);
	}

	return 0;
}

#define AK4490_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
				SNDRV_PCM_RATE_192000)  // | SNDRV_PCM_RATE_384000 | SNDRV_PCM_RATE_768000

#define AK4490_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4490_dai_ops = {
	.hw_params	= ak4490_hw_params,
	.hw_free = ak4490_hw_free,
	.set_sysclk	= ak4490_set_dai_sysclk,
	.set_fmt	= ak4490_set_dai_fmt,
	.trigger = ak4490_trigger,
	.digital_mute = ak4490_set_dai_mute,
};

struct snd_soc_dai_driver ak4490_dai[] = {   
	{										 
		.name = "ak4490-AIF1",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = AK4490_RATES,
		       .formats = AK4490_FORMATS,
		},
		.ops = &ak4490_dai_ops,
	},									 
};

static int ak4490_init_reg(struct snd_soc_codec *codec)
{

	ak4490_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}


static int ak4490_populate_dt_pdata(struct device *dev,
		struct ak4490_priv *ak4490)
{
	if ((NULL == ak4490) || (NULL == dev)){
		pr_err("%s null codec point\n", __func__);
		return -EINVAL;
	}
	pr_err("[LHS]%s in 3333 \n",__func__);
	ak4490->pdn_gpio = of_get_named_gpio(dev->of_node,
					      "ak4490,pdn-gpio", 0);
	if (!gpio_is_valid(ak4490->pdn_gpio)){
		dev_err(dev, "%s error pdn_gpio gpio 1\n", __func__);
		pr_err("%s error pdn_gpio gpio 1\n", __func__);
		return -EINVAL;
	}

	ak4490->ldoen_gpio = of_get_named_gpio(dev->of_node,"ak4490,ldoen-gpio",0);
		if (!gpio_is_valid(ak4490->ldoen_gpio)){
		dev_err(dev, "%s error ldoen_gpio gpio 93\n", __func__);
		pr_err("%s error pdn_gpio gpio 93\n", __func__);
		return -EINVAL;
	}

	ak4490->isl54405_sw_pwr_gpio = of_get_named_gpio(dev->of_node,
					      "qcom,isl-sw-pwr-gpio", 0);
	if (!gpio_is_valid(ak4490->isl54405_sw_pwr_gpio)){
		dev_err(dev, "%s error isl54405_sw_pwr_gpio pmi gpios 10\n", __func__);
		return -EINVAL;
	}

	ak4490->isl98608_ldo_pwr_gpio = of_get_named_gpio(dev->of_node,
					      "ak4490,isl-ldo-pwr-gpio", 0);
	if (!gpio_is_valid(ak4490->isl98608_ldo_pwr_gpio)){
		dev_err(dev, "%s error isl98608_ldo_pwr_gpio pmi mpp 4\n", __func__);
		return -EINVAL;
	}

	ak4490->isl54405_sel_gpio = of_get_named_gpio(dev->of_node,
					      "ak4490,hifi_sel_gpio", 0);
	if (!gpio_is_valid(ak4490->isl54405_sel_gpio)){
		dev_err(dev, "%s error isl54405_sel_gpio pm mpp 2\n", __func__);
		return -EINVAL;
	}
	
	ak4490->isl54405_mute_gpio = of_get_named_gpio(dev->of_node,
					      "ak4490,hifi_mute_gpio", 0);
	if (!gpio_is_valid(ak4490->isl54405_mute_gpio)){
		dev_err(dev, "%s error isl54405_mute_gpio pm mpp 4\n", __func__);
		return -EINVAL;
	}

	ak4490->hifi_pop_gpio = of_get_named_gpio(dev->of_node,
					      "ak4490,hifi_pop_gpio", 0);
	if (!gpio_is_valid(ak4490->hifi_pop_gpio)){
		dev_err(dev, "%s error is hifi_pop_gpio pmi gpios 10\n", __func__);
		return -EINVAL;
	}
	pr_err("%s hifi_pop_gpio(%d) success\n", __func__, ak4490->hifi_pop_gpio);

	
    return 0;
}
#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
int isl98608_pw_enable(bool val)
{
 	int ret=0;
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(ak4490_codec);
	pr_err("[LHS]%s  in \n",__func__);

	if(val==true)
		{
	ret = gpio_direction_output(ak4490->ldoen_gpio, 1);                                        
	if (ret < 0) {
		pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	ret = gpio_direction_output(ak4490->isl98608_ldo_pwr_gpio, 1);                            
	if (ret < 0) {
		pr_err("[LHS]%s(): isl98608_ldo_pwr_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
		}
	else 
		{
			ret = gpio_direction_output(ak4490->ldoen_gpio, 0);                                        
			if (ret < 0) {
				pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
			ret = gpio_direction_output(ak4490->isl98608_ldo_pwr_gpio, 0);                            
			if (ret < 0) {
				pr_err("[LHS]%s(): isl98608_ldo_pwr_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
		}
	return 0;
}


static int ak4490_power_enable(bool val)
{
	int ret = 0;
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(ak4490_codec);
	pr_err("[LHS]%s in val =%d \n",__func__,val);
	if(val==true)
		{
			//AKM-sungang:1,PDN pin = L 												 
			ret = gpio_direction_output(ak4490->pdn_gpio, 0);                  				
			if (ret < 0) {
				pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
			msleep(10);
			//AKM-sungang:2,TVDD/AVDD/DVDD power up
			ret = gpio_direction_output(ak4490->ldoen_gpio, 1);                                        
			if (ret < 0) {
				pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}

			//AKM-sungang:4,PDN pin = H 												 
			ret = gpio_direction_output(ak4490->pdn_gpio, 1);                  				
			if (ret < 0) {
				pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
		}
	else
		{
			//AKM-sungang:1,PDN pin = L 												 
			ret = gpio_direction_output(ak4490->pdn_gpio, 0);                  				
			if (ret < 0) {
				pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
			msleep(10);
			//AKM-sungang:2,TVDD/AVDD/DVDD power up
			ret = gpio_direction_output(ak4490->ldoen_gpio, 0);                                        
			if (ret < 0) {
				pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
						__func__, ret);
				return ret;
			}
		}
	pr_err("[LHS]%s out!\n",__func__);

	return 0;
}
#endif

static int ak4490_power_up(struct ak4490_priv *ak4490)
{
	int ret = 0;
	pr_err("[LHS]%s in : power up flow : ak4490_ldo , isl98608 pwr and regs, ak4490_pdn .\n",__func__);
	//AKM-sungang:1,PDN pin = L 												 
	ret = gpio_direction_output(ak4490->pdn_gpio, 0);                  				
	if (ret < 0) {
		pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	//AKM-sungang:2,TVDD/AVDD/DVDD power up
	ret = gpio_direction_output(ak4490->ldoen_gpio, 1);                                        
	if (ret < 0) {
		pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	//AKM-sungang:3,isl98608_en(VREFHL/R VDDL/R) power up
	ret = gpio_direction_output(ak4490->isl98608_ldo_pwr_gpio, 1);                            
	if (ret < 0) {
		pr_err("%s(): isl98608_ldo_pwr_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	msleep(10);
	isl98608_power_up();
	msleep(10);
	//AKM-sungang:4,PDN pin = H 												 
	ret = gpio_direction_output(ak4490->pdn_gpio, 1);                  				
	if (ret < 0) {
		pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	pr_err("[LHS]%s out!\n",__func__);

	return 0;
	
}
static int ak4490_power_down(struct ak4490_priv *ak4490)
{
	int ret = 0;
	pr_err("[LHS]%s in : power off flow: ak4490_pdn , ak4490_ldo , isl98608 \n",__func__);
	//AKM-sungang:1,PDN pin = L 												 
	ret = gpio_direction_output(ak4490->pdn_gpio, 0);						
	if (ret < 0) {
		pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	//AKM-sungang:2,TVDD/AVDD/DVDD power down
	ret = gpio_direction_output(ak4490->ldoen_gpio, 0); 					
	if (ret < 0) {
		pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	msleep(10);
	//power down the isl98608
	ret = gpio_direction_output(ak4490->isl98608_ldo_pwr_gpio, 0);				
	if (ret < 0) {
		pr_err("%s(): isl98608_ldo_pwr_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	/* hifi_pop_gpio cut*/
		ret = gpio_direction_output(ak4490->hifi_pop_gpio, 1);
		if (ret < 0) {
			pr_err("%s(): hifi_pop_gpio direction failed %d\n",
			__func__, ret);
			return ret;
		}
		pr_err("[LHS]%s: hifi_pop_gpio is 1 \n",__func__);
		pr_err("%s(): 3: hifi_pop_gpio(%d) value(%d)",
                         __func__, ak4490->hifi_pop_gpio, gpio_get_value_cansleep(ak4490->hifi_pop_gpio));
	pr_err("[LHS]%s out! \n",__func__);
	return 0;

}
	
static int ak4490_initial_gpio(struct ak4490_priv *ak4490)
{
	int ret = 0;
	//AKM-sungang:1,PDN pin = L
	ret = gpio_request(ak4490->pdn_gpio, "ak4490_pdn_gpio");
	if (ret < 0) {
		pr_err("%s(): ak4490_pdn_gpio request failed %d\n",
				__func__, ret);
		return ret;
	}
	// power down the ak4490 pdn during the system up
	ret = gpio_direction_output(ak4490->pdn_gpio, 0);
	if (ret < 0) {
		pr_err("%s(): ak4490_pdn_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	//AKM-sungang:2,TVDD/AVDD/DVDD power up
	ret = gpio_request(ak4490->ldoen_gpio, "ak4490_ldo_gpio");                            
	if (ret < 0) {
		pr_err("%s(): ak4490_ldoen_gpio request failed %d\n",
				__func__, ret);
		return ret;
	}
	// power down the ak4490 ldo during the system up 
	ret = gpio_direction_output(ak4490->ldoen_gpio, 0);    
	if (ret < 0) {
		pr_err("%s(): ak4490_ldoen_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}

	pr_err("[LHS] %s ldo -- enable !\n",__func__);
	msleep(100);
	//AKM-sungang:3,isl98608_en(VREFHL/R VDDL/R) power up
	ret = gpio_request(ak4490->isl98608_ldo_pwr_gpio, "isl98608_ldo_pwr_gpio");                
	if (ret < 0) {
		pr_err("%s(): isl98608_ldo_pwr_gpio request failed %d\n",
				__func__, ret);
		return ret;
	}
	// power down the isl98608 during the system up 
	ret = gpio_direction_output(ak4490->isl98608_ldo_pwr_gpio, 0); 
	if (ret < 0) {
		pr_err("%s(): isl98608_ldo_pwr_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}
	  //AKM-sungang:4,HP-CHIP power up
	ret = gpio_request(ak4490->isl54405_sw_pwr_gpio, "isl54405_sw_pwr_gpio");            
	if (ret < 0) {
		pr_err("%s(): isl54405_sw_pwr_gpio request failed %d\n",
				__func__, ret);
		return ret;
	}
	// power up the isl54405 switch during the system up 
	ret = gpio_direction_output(ak4490->isl54405_sw_pwr_gpio, 1);   
	if (ret < 0) {
		pr_err("%s(): isl54405_sw_pwr_gpio direction failed %d\n",
				__func__, ret);
		return ret;
	}

	ret = gpio_request(ak4490->isl54405_sel_gpio, "ak4490_isl54405_sel_gpio");
	if (ret < 0) {
		pr_err("%s(): ak4490_isl54405_sel_gpio request failed %d\n",
				__func__, ret);
		return ret;
        }
	// switch select the ak4961 at the system start 
	ret = gpio_direction_output(ak4490->isl54405_sel_gpio, 1);  
	if (ret < 0) {
		pr_err("%s(): ak4490_isl54405_sel_gpio direction failed %d\n",
				__func__, ret);
		return ret;
		}

	ret = gpio_request(ak4490->isl54405_mute_gpio, "ak4490_isl54405_mute_gpio");
	if (ret < 0) {
		pr_err("%s(): ak4490_isl54405_mute_gpio request failed %d\n",
				__func__, ret);
		return ret;
	}
	// switch mute off(play will be ok) during the system start 
	ret = gpio_direction_output(ak4490->isl54405_mute_gpio, 1); 
	if (ret < 0) {
		pr_err("%s(): ak4490_isl54405_mute_gpio direction failed %d\n",
				__func__, ret);
	return ret;
	}

	// hifi_pop_gpio cut
	ret = gpio_request(ak4490->hifi_pop_gpio, "ak4490_hifi_pop_gpio");            
	if (ret < 0) {
		pr_err("%s(): is hifi_pop_gpio request failed %d\n",
				__func__, ret);
	}

	ret = gpio_direction_output(ak4490->hifi_pop_gpio, 1);   
	if (ret < 0) {
		pr_err("%s(): is hifi_pop_gpio direction failed %d\n",
				__func__, ret);
	}


    	pr_err("[LHS]%s system up ,ak4490 initial_gpio success!default setting : hifi swith power on , switch mute on , select ak4961, all others off \n",__func__);
	return 0;
}


static int ak4490_probe(struct snd_soc_codec *codec)
{
	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
	int ret = 0;
#endif
	pr_err("[LHS]%s: Entry!\n",__func__);

#ifdef AK4490_CONTIF_DEBUG
	//codec->write = ak4490_i2c_write;
	//codec->read = ak4490_i2c_read;
#endif
#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
	ret=device_create_file(codec->dev,&dev_attr_ak4490reg_data);
	if(ret)
		{
			pr_err("[LHS] %s error to create reg_data\n",__func__);
		}
	ak4490_codec=codec;
#endif


//	ak4490_codec = codec;

	ak4490_init_reg(codec);

	ak4490->fs1 = 48000;
	ak4490->nBickFreq = 0;		
	ak4490->nDSDSel = 0;
	ak4490->akm_hifi_switch_mute=0;
	ak4490->akm_hifi_switch=0;
	ak4490->akm_pw_switch=0;

	return 0;
}

static int ak4490_remove(struct snd_soc_codec *codec)
{

	akdbgprt("\t[AK4490] %s(%d)\n",__FUNCTION__,__LINE__);

#ifdef CONFIG_DEBUG_FS_CODEC_AK4490
	device_remove_file(codec->dev,&dev_attr_ak4490reg_data);
#endif

	ak4490_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

/* ZTE_modify initialization from incompatible pointer type begin */
static int ak4490_suspend(struct snd_soc_codec *codec)
{
//	struct ak4490_priv *ak4490 = snd_soc_codec_get_drvdata(codec);
//	pr_err("[LHS]%s: ak4490_suspend \n",__func__);
//	regcache_sync(ak4490->regmap);

	ak4490_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}
/* ZTE_modify end */

static int ak4490_resume(struct snd_soc_codec *codec)
{
//	pr_err("[LHS]%s: ak4490_resume \n",__func__);
//	ak4490_init_reg(codec);
	ak4490_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4490 = {
	.probe = ak4490_probe,
	.remove = ak4490_remove,
	.suspend =	ak4490_suspend,
	.resume =	ak4490_resume,

	.controls = ak4490_snd_controls,
	.num_controls = ARRAY_SIZE(ak4490_snd_controls),
	.read = ak4490_i2c_read,
	.write = ak4490_i2c_write,
	.set_bias_level = ak4490_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ak4490_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ak4490_reg,
	.readable_register = ak4490_readable,
	.volatile_register = ak4490_volatile,	
	.dapm_widgets = ak4490_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4490_dapm_widgets),
	.dapm_routes = ak4490_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4490_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4490);


static const struct regmap_config ak4490_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AK4490_MAX_REGISTERS,
	.reg_defaults = ak4490_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4490_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};


static int ak4490_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak4490_priv *ak4490;
	int ret=0;

	akdbgprt("\t[LHS][AK4490] %s(%d)\n",__FUNCTION__,__LINE__);
	

	ak4490 = devm_kzalloc(&i2c->dev, sizeof(struct ak4490_priv), GFP_KERNEL);
	if (ak4490 == NULL) 
		return -ENOMEM;

       ret = ak4490_populate_dt_pdata(&i2c->dev, ak4490);
	if (ret) {
		pr_err("[LHS]%s Parsing DT failed99(%d)", __func__, ret);
		return ret;
	}

   ret = ak4490_initial_gpio(ak4490);
	if (ret){
		pr_info("%s fail to set gpio%d\n", __func__, ret);
	}


	ak4490->regmap = devm_regmap_init_i2c(i2c, &ak4490_regmap_config);
	if (IS_ERR(ak4490->regmap))
		{
		pr_err("[LHS] %s regmap failed!\n",__func__);
		return PTR_ERR(ak4490->regmap);
		}

	   
	i2c_set_clientdata(i2c, ak4490);
	ak4490->i2c_client = i2c;
       g_ak4490_priv = ak4490;


	dev_set_name(&i2c->dev, "%s", "ak4490");

	pr_err("[LHS]%s dev name--------sungang %s\n", __func__, dev_name(&i2c->dev));

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4490, &ak4490_dai[0], ARRAY_SIZE(ak4490_dai));
	if (ret < 0){
		kfree(ak4490);
		akdbgprt("\t[AK4490 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}
	pr_err("[LHS]%s leave ret=%d\n", __func__, ret);
	return ret;
}

static int ak4490_i2c_remove(struct i2c_client *client)
{

	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ak4490_i2c_id[] = {
	{ "ak4490", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4490_i2c_id);

static struct i2c_driver ak4490_i2c_driver = {
	.driver = {
		.name = "ak4490",
		.owner = THIS_MODULE,
	},
	.probe = ak4490_i2c_probe,
	.remove = ak4490_i2c_remove,
	.id_table = ak4490_i2c_id,
};

static int __init ak4490_modinit(void)
{

	akdbgprt("\t[AK4490] %s(%d)\n", __FUNCTION__,__LINE__);

	return i2c_add_driver(&ak4490_i2c_driver);
}

module_init(ak4490_modinit);

static void __exit ak4490_exit(void)
{
	i2c_del_driver(&ak4490_i2c_driver);
}
module_exit(ak4490_exit);

MODULE_DESCRIPTION("ASoC ak4490 codec driver");
MODULE_LICENSE("GPL");
