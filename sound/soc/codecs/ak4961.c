/* Copyright (c) 2011-2013, Asahi Kasei Microdevices Corporation.
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/wait.h>
#include <linux/mfd/ak49xx/core.h>
#include <linux/mfd/ak49xx/ak496x_registers.h>
#include <linux/mfd/ak49xx/ak4961_registers.h>
#include <linux/mfd/ak49xx/pdata.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include "ak4961.h"

//weizj add for audience es804
#if defined(CONFIG_SND_SOC_ES705) || defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)
#include "audience/es705-export.h"
#endif

#if defined(CONFIG_SND_SOC_ES705)

#define CONFIG_SND_SOC_ESXXX
#define REMOTE_ROUTE_ENABLE_CB  es705_remote_route_enable
#define SLIM_GET_CHANNEL_MAP_CB es705_slim_get_channel_map
#define SLIM_SET_CHANNEL_MAP_CB es705_slim_set_channel_map
#define SLIM_HW_PARAMS_CB       es705_slim_hw_params
#define REMOTE_CFG_SLIM_RX_CB	es705_remote_cfg_slim_rx
#define REMOTE_CLOSE_SLIM_RX_CB	es705_remote_close_slim_rx
#define REMOTE_CFG_SLIM_TX_CB	es705_remote_cfg_slim_tx
#define REMOTE_CLOSE_SLIM_TX_CB	es705_remote_close_slim_tx
#define REMOTE_ADD_CODEC_CONTROLS_CB	es705_remote_add_codec_controls

#elif defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)

#if defined(CONFIG_SND_SOC_ES_SLIM)
#define CONFIG_SND_SOC_ESXXX
#define REMOTE_ROUTE_ENABLE_CB		es705_remote_route_enable
#define SLIM_GET_CHANNEL_MAP_CB		escore_slim_get_channel_map
#define SLIM_SET_CHANNEL_MAP_CB		escore_slim_set_channel_map
#define SLIM_HW_PARAMS_CB		escore_slim_hw_params
#define REMOTE_CFG_SLIM_RX_CB		escore_remote_cfg_slim_rx
#define REMOTE_CLOSE_SLIM_RX_CB		escore_remote_close_slim_rx
#define REMOTE_CFG_SLIM_TX_CB		escore_remote_cfg_slim_tx
#define REMOTE_CLOSE_SLIM_TX_CB		escore_remote_close_slim_tx
#define REMOTE_ADD_CODEC_CONTROLS_CB	es705_remote_add_codec_controls
#endif

#endif

#ifdef CONFIG_SND_SOC_ESXXX
int (*remote_route_enable)(struct snd_soc_dai *dai) = REMOTE_ROUTE_ENABLE_CB;
int (*slim_get_channel_map)(struct snd_soc_dai *dai,
		unsigned int *tx_num, unsigned int *tx_slot,
		unsigned int *rx_num, unsigned int *rx_slot)
			= SLIM_GET_CHANNEL_MAP_CB;
int (*slim_set_channel_map)(struct snd_soc_dai *dai,
		unsigned int tx_num, unsigned int *tx_slot,
		unsigned int rx_num, unsigned int *rx_slot)
			= SLIM_SET_CHANNEL_MAP_CB;
int (*slim_hw_params)(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
		= SLIM_HW_PARAMS_CB;
int (*remote_cfg_slim_rx)(int dai_id)	=	REMOTE_CFG_SLIM_RX_CB;
int (*remote_close_slim_rx)(int dai_id)	=	REMOTE_CLOSE_SLIM_RX_CB;
int (*remote_cfg_slim_tx)(int dai_id)	=	REMOTE_CFG_SLIM_TX_CB;
int (*remote_close_slim_tx)(int dai_id)	=	REMOTE_CLOSE_SLIM_TX_CB;
int (*remote_add_codec_controls)(struct snd_soc_codec *codec)
		= REMOTE_ADD_CODEC_CONTROLS_CB;

#endif
//weizj add for audience es804

#define AK4961_NUM_PRAM			9  //for slim_spi 2014-11-17
#ifdef AKM_VOICE_CALL
#define AK4961_NUM_CRAM			20 /*for slim_spi 2014-11-17*/
#else
#define AK4961_NUM_CRAM			19 /*for slim_spi 2014-11-17*/
#endif
#define AK4961_NUM_ARAM			2 // for aram 

static int mic_det_repeat;

/* zte weizhijun add headset detect /proc/hs 2013-12-17 start */
#include <linux/time.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

static int hs_type = 0;

/* zte weizhijun add headset detect /proc/hs 2013-12-17 end */

static struct afe_param_slimbus_slave_port_cfg ak4961_slimbus_slave_port_cfg = {
	.minor_version = 1,
	.slimbus_dev_id = AFE_SLIMBUS_DEVICE_1,
	.slave_dev_pgd_la = 0,
	.slave_dev_intfdev_la = 0,
	.bit_width = 16,
	.data_format = 0,
	.num_channels = 1
};

static const char *AK4961_PRAM_FIRMWARES[] = {
	"ak4961_pram_narrow.bin",
	"ak4961_pram_wide.bin",
	"ak4961_pram_voice_recognition.bin",
	"ak4961_pram_sound_record.bin",
	"ak4961_pram_music_speaker.bin",
	"ak4961_pram_interview_record.bin",
	"ak4961_pram_barge_in.bin",
	"ak4961_pram_karaoke.bin",
	"ak4961_pram_karaoke_spk.bin"
};

#if 0
static const char *AK4961_CRAM_FIRMWARES[] = {
	"ak4961_cram_null.bin",
	"ak4961_cram_narrow_hs.bin",
	"ak4961_cram_narrow_hp.bin",
	"ak4961_cram_narrow_hf.bin",
	"ak4961_cram_wide_hs.bin",
	"ak4961_cram_wide_hp.bin",
	"ak4961_cram_wide_hf.bin",
	"ak4961_cram_voice_recognition.bin",
	"ak4961_cram_sound_record.bin",
	"ak4961_cram_voice_record.bin",
	"ak4961_cram_music_speaker.bin",
	"ak4961_cram_karaoke_heavy.bin",
	"ak4961_cram_karaoke_light.bin",
	"ak4961_cram_karaoke_middle.bin",
};
#else
static const char *AK4961_CRAM_FIRMWARES[] = {
	"ak4961_cram_null.bin",
	"ak4961_cram_narrow_hs.bin",
	"ak4961_cram_narrow_hp.bin",
	"ak4961_cram_narrow_hf.bin",
	"ak4961_cram_wide_hs.bin",
#ifdef AKM_VOICE_CALL
	"ak4961_cram_wide_headset.bin",
#endif
	"ak4961_cram_wide_hp.bin",
	"ak4961_cram_wide_hf.bin",
	"ak4961_cram_voice_recognition.bin",
	"ak4961_cram_sound_record.bin",
	"ak4961_cram_voice_record.bin",
	"ak4961_cram_wchat_record.bin",
	"ak4961_cram_music_speaker.bin",
	"ak4961_cram_interview_record.bin",
	"ak4961_cram_karaoke_live.bin",
	"ak4961_cram_karaoke_room.bin",
	"ak4961_cram_karaoke_cave.bin",
	"ak4961_cram_karaoke_spk_room.bin",
	"ak4961_cram_karaoke_spk_live.bin",
	"ak4961_cram_karaoke_spk_theater.bin"
};
#endif

static const char *AK4961_ARAM_FIRMWARES[]={
	"ak4961_aram_null.bin",
	"ak4961_aram_karaoke_spk.bin"
};

#define CONFIG_DEBUG_FS_CODEC
//#define CONFIG_VOICE_WAKEUP

#define AK4961_DMIC1_CLK_ENABLE (1 << 3)
#define AK4961_DMIC1_CHANEL_LRP (0 << 1)	// 0: L->Lch, H->Rch; 1: L->Rch, H->Lch
#define AK4961_DMIC2_CLK_ENABLE (1 << 3)
#define AK4961_DMIC2_CHANEL_LRP (0 << 1)	// 0: L->Lch, H->Rch; 1: L->Rch, H->Lch

#define AK4961_RATES SNDRV_PCM_RATE_8000_192000

#define BITS_PER_REG 8

#define SLIM_CLOSE_TIMEOUT	1000
#define SRC_RESET_TIMEOUT	600000	// msecs

#define AK4961_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
			 SND_JACK_OC_HPHR | SND_JACK_UNSUPPORTED)

#define AK4961_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE)

#define AK4961_FORMATS_AIF (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE | \
		SNDRV_PCM_FORMAT_MU_LAW | SNDRV_PCM_FORMAT_A_LAW)

//#define AK4961_I2S_MASTER_MODE_MASK 0x08

enum {
	SB1_PB = 0,
	SB1_CAP,
	SB2_PB,
	SB2_CAP,
	SB3_PB,
	SB3_CAP,
	SB4_VIFEED,
	NUM_CODEC_DAIS,
};

enum {
	AIF_PORT1 = 0,
	AIF_PORT2,
	AIF_PORT3,
	AIF_PORT4,
	NUM_AIF_DAIS,
};

#define AK4961_MCLK_RATE_12288KHZ 12288000
#define AK4961_MCLK_RATE_9600KHZ 9600000
#define AK4961_RX_PORT_START_NUMBER	10

static int last_crc;

static struct snd_soc_dai_driver ak4961_dai[];
static int ak4961_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);
static int ak4961_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);

// ZTE_chenjun
#if 0
extern int socinfo_get_ffbm_flag(void);
#endif
//

static int ak4961_dsp_mode = 0;
static int internal_rx_gain = 0;
static int karaoke_mic_gain = 0;//add by shengguanghui for karaoke 20150606
static int internal_mic_gain = 0;
static int internal_ahpf_sensitivity = 0;
static bool share_wait_time = 0;
static u8	dac_status = 0;
static struct snd_soc_codec *ak4961_codec; //lhs for ram codec write
static int ak4961_band_mode = 0; /* 0 mean NB , 1 mean WB .*/
static int ak4961_band_set = 0;  /*lhs*/

/* from 0dB ~ +20dB */
static const u32 mic_gain_table[21] = {
	0x008000,
	0x008FA0, 0x00A120, 0x00B4D0, 0x00CAE0, 0x00E3A0,
	0x00FF60, 0x011E90, 0x014180, 0x0168C0, 0x0194C0,
	0x01C630, 0x01FD90, 0x023BC0, 0x028180, 0x02CFD0,
	0x0327A0, 0x038A30, 0x03F8C0, 0x0474D0, 0x050000,
};

/* from -20dB ~ +20dB */
static const u32 mic_gain_table2[21] = {
	0x000CD0, 0x001020, 0x001450, 0x001990, 0x002020,
	0x002880, 0x0032F0, 0x004020, 0x0050C0, 0x0065B0,
	0x008000,
	0x00A120, 0x00CAE0, 0x00FF60, 0x014180, 0x0194C0,
	0x01FD90, 0x028180, 0x0327A0, 0x03F8C0, 0x050000,
};

static const struct ak49xx_ch ak4961_rx_chs[AK4961_RX_MAX] = {
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER, 0),
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER + 1, 1),
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER + 2, 2),
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER + 3, 3),
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER + 4, 4),
	AK49XX_CH(AK4961_RX_PORT_START_NUMBER + 5, 5)
};

static const struct ak49xx_ch ak4961_tx_chs[AK4961_TX_MAX] = {
	AK49XX_CH(0, 0),
	AK49XX_CH(1, 1),
	AK49XX_CH(2, 2),
	AK49XX_CH(3, 3),
	AK49XX_CH(4, 4),
	AK49XX_CH(5, 5),
	AK49XX_CH(6, 6),
	AK49XX_CH(7, 7),
	AK49XX_CH(8, 8),
	AK49XX_CH(9, 9)
};

static const u32 vport_check_table[NUM_CODEC_DAIS] = {
	0,					/* SB1_PB */
	(1 << SB2_CAP) | (1 << SB3_CAP),	/* SB1_CAP */
	0,					/* SB2_PB */
	(1 << SB1_CAP) | (1 << SB3_CAP),	/* SB2_CAP */
	0,					/* SB2_PB */
	(1 << SB1_CAP) | (1 << SB2_CAP),	/* SB2_CAP */
};

static const u32 vport_i2s_check_table[NUM_CODEC_DAIS] = {
	0, /* SB1_PB */
	0, /* SB1_CAP */
	0, /* SB2_PB */
	0, /* SB2_CAP */
};

struct ak4961_priv {
	struct mutex mutex;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	enum   ak4961_state state;
	enum   ak4961_slimbus_stream_state stream_state;
	enum   ak4961_slimbus_stream_state pcm_state;

	struct snd_soc_codec *codec;

	struct ak4961_mbhc_config mbhc_cfg;

	struct ak49xx_pdata *pdata;

// ZTE_chenjun
	int ext_clk_users;
	struct clk *cdc_ext_clk;
//

	/*track ak4961 interface type*/
	u8 intf_type;

	/* num of slim ports required */
	struct ak49xx_codec_dai_data dai[NUM_CODEC_DAIS];

	struct afe_param_cdc_slimbus_slave_cfg slimbus_slave_cfg;

	const struct firmware *pram_firmware[AK4961_NUM_PRAM];
	const struct firmware *cram_firmware[AK4961_NUM_CRAM];
	const struct firmware *aram_firmware[AK4961_NUM_ARAM];

	u8 pram_load_index;
	u8 cram_load_index;
	u8 aram_load_index;

	bool ak4961_dsp_downlink_status;
	bool ak4961_dsp_uplink_status;
       bool ak4961_voice_sync_switch;
	u8 ak4961_slimbus_interface_reg;
	u8 ak4961_power_test;

	u8 lch_bargein_sel;
	u8 rch_bargein_sel;

	int (*machine_codec_event_cb)(struct snd_soc_codec *codec,
			enum ak49xx_codec_event);

	struct timer_list timer;

	/* Port values for Rx and Tx codec_dai */
	unsigned int rx_port_value;
	unsigned int tx_port_value;

};

#ifdef CONFIG_DEBUG_FS_CODEC
struct ak49xx *debugak49xx;

static ssize_t reg_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, i, j, fpt;
	int rx[192];

	for (i = 0; i < VIRTUAL_ADDRESS_CONTROL; i++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	for (j = CRC_RESULT_H8; j < MIR1_REGISTER_1; i++, j++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, j);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	for (j = VAD_SETTING_1; j <= CREG7_SETTING; i++, j++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, j);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	if (i == 188) {

		ret = fpt = 0;

		for (i = 0; i < VIRTUAL_ADDRESS_CONTROL; i++, fpt += 6) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);
		}

		for (j = CRC_RESULT_H8; j < MIR1_REGISTER_1; i++, j++, fpt += 6) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", j, rx[i]);
		}

		for (j = VAD_SETTING_1; j <= CREG7_SETTING; i++, j++) {

			ret += sprintf(buf + fpt, "%02x,%02x\n", j, rx[i]);

			if (j > 0x00FF) {
				fpt += 7;
			} else {
				fpt += 6;
			}
		}
		return ret;

	} else {

		return sprintf(buf, "read error");
	}
}

static ssize_t reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	*ptr_data = (char *)buf;
	char	*p;
	int		i, pt_count = 0;
	unsigned short val[20];

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 20)
			break;

		val[pt_count] = simple_strtoul(p, NULL, 16);

		pt_count++;
	}

	for (i = 0; i < pt_count; i+=2) {
		ak49xx_reg_write(&debugak49xx->core_res, val[i], val[i+1]);
		pr_debug("%s: write add=%d, val=%d.\n", __func__, val[i], val[i+1]);
	}

	return count;
}

static ssize_t ram_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t ram_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	*ptr_data = (char *)buf;
	char	*p;
	int		pt_count = 0;
	u8		val[9];
	u16		temp;

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 16)
			break;

		temp = simple_strtoul(p, NULL, 16);
		pt_count++;

		if (pt_count % 3 == 1) {
			val[4] = temp >> 8;
			val[5] = temp & 0xFF;

		} else if (pt_count % 3 == 2) {
			val[6] = temp >> 8;
			val[7] = temp & 0xFF;

		} else {
			val[8] = temp;
			ak49xx_run_ram_write(debugak49xx, val);
			pr_debug("%s: add = %02x%02x, val=%02x%02x%02x.\n",
					__func__, val[4], val[5], val[6], val[7], val[8]);
		}
	}

	return count;
}

static ssize_t ram_load_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int	ret;

	ret = sprintf(buf, "%04X\n", last_crc);

	return ret;
}

static ssize_t ram_load_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	buf_val[9];
	u8		val[MAX_ARRAY_SIZE];
	u8		vat = 0;
	u8		page = 0;
	u16		ptr_count = 0;
	u16		pt_count = 0;
	u16		start = 0;
	u32		val_tmp;
	int		ret = 0;

	if (!strncmp(buf, "PRAM", 4)) {
		vat = 0x04;
	} else if (!strncmp(buf, "CRAM", 4)) {
		vat = 0x05;
	}
	ptr_count = 4;

	if (*(buf + ptr_count) != '\0') {
		memcpy(buf_val, buf + ptr_count, 4);
		buf_val[4] = '\0';
		start = simple_strtoul(buf_val, NULL, 16);
		ptr_count += 4;
		page += start >> 8;
		start = start & 0x00FF;
	}

	while (*(buf + ptr_count) != '\0') {
		memcpy(buf_val, buf + ptr_count, 8);
		buf_val[8] = '\0';
		val_tmp = simple_strtoul(buf_val, NULL, 16);
		val[pt_count++] = (val_tmp & 0xFF000000) >> 24;
		val[pt_count++] = (val_tmp & 0x00FF0000) >> 16;
		val[pt_count++] = (val_tmp & 0x0000FF00) >> 8;
		val[pt_count++] = val_tmp & 0x000000FF;
		ptr_count += 8;
	}

	ret += ak49xx_ram_write(debugak49xx, vat, page, start, pt_count, val);
		//for slim_spi 2014-11-17 星期一 14:16:45
	if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SPI|| 
		ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		ret += ak49xx_bulk_read(&debugak49xx->core_res, CRC_RESULT_H8, 2, buf_val);
		last_crc = (buf_val[0] << 8) + buf_val[1];
	} else if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS) {
		if (ret == 0) {
			last_crc = 0;
		} else {
			last_crc = 1;
		}
	}

	if (ret) {
		pr_err("%s: failed!\n", __func__);
	} else {
		pr_info("%s: succeeded!\n", __func__);
	}

	return count;
}

static ssize_t mir_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, i;
	int rx[16];

	for (i = 0; i < 16; i++) {

		ret = ak49xx_reg_read(&debugak49xx->core_res, MIR1_REGISTER_1 + i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	if (i == 16) {

		return sprintf(buf, "%02x%02x%x,%02x%02x%x,%02x%02x%x,%02x%02x%x\n",
				rx[0], rx[1], rx[2]>>4, rx[4], rx[5], rx[6]>>4,
				rx[8], rx[9], rx[10]>>4, rx[12], rx[13], rx[14]>>4);
	} else {

		return sprintf(buf, "read error");
	}
}

static ssize_t mir_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(reg_data, 0644, reg_data_show, reg_data_store);
static DEVICE_ATTR(ram_data, 0644, ram_data_show, ram_data_store);
static DEVICE_ATTR(ram_load, 0644, ram_load_show, ram_load_store);
static DEVICE_ATTR(mir_data, 0644, mir_data_show, mir_data_store);

#endif
int ak49xx_bandswitch_set(struct snd_soc_codec *codec, int howtochange);
static void ak4961_work(struct work_struct *work)
{
	struct snd_soc_codec *codec;
	struct ak4961_priv *ak4961;

	ak4961 = container_of(work, struct ak4961_priv, work);
	codec = ak4961->codec;

	mutex_lock(&ak4961->mutex);
	switch (ak4961->state) {
	case AK4961_DSPRSTNON:
		if (ak4961->pcm_state == AK4961_SLIMBUS_STREAM_ON) {
			if ((ak4961_dsp_mode >= DSP_MODE_NARROW_HANDSET) &&
			(ak4961_dsp_mode <= DSP_MODE_WIDE_HANDSFREE)) {
				usleep_range(1000, 1005);
				if (ak4961_band_mode)
					ak49xx_bandswitch_set(codec, ak4961_band_mode);
			}
		}
		snd_soc_write(codec, FLOW_CONTROL_3, 0x01);	// DSPRSTN On
		break;
	default:
		break;
	}

	if (ak4961->stream_state != AK4961_SLIMBUS_STREAM_NA) {

		if (snd_soc_read(codec, JACK_DETECTION_STATUS) & 0x02) { // JDS == 1

			if (ak4961->stream_state == AK4961_SLIMBUS_STREAM_OFF) {
				snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);	// HP Off
			}

			if (ak4961->stream_state == AK4961_SLIMBUS_STREAM_ON) {
				if (snd_soc_read(codec, POWER_MANAGEMENT_8) & 0x01) { // PMDA1 == 1
					snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);	// HP On
				}
			}
		}
	}
	mutex_unlock(&ak4961->mutex);
}

int ak4961_mclk_enable(struct snd_soc_codec *codec, int mclk_enable, bool dapm)
{
//	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: mclk_enable = %u, dapm = %d\n", __func__, mclk_enable,
		 dapm);

	return 0;
}

// ZTE_chenjun
static int __ak4961_mclk_enable(struct ak4961_priv *ak4961, int mclk_enable)
{
	int ret = 0;

	if (mclk_enable) {
		ak4961->ext_clk_users++;
		if (ak4961->ext_clk_users > 1)
			goto exit;
		ret = clk_prepare_enable(ak4961->cdc_ext_clk);
		if (ret) {
			pr_err("%s: ext clk enable failed\n",
				__func__);
			ak4961->ext_clk_users--;
			goto exit;
		}
	} else {
		ak4961->ext_clk_users--;
		if (ak4961->ext_clk_users == 0) {
			clk_disable_unprepare(ak4961->cdc_ext_clk);
		}
	}
exit:
	return ret;
}

int ak4961_codec_mclk_enable(struct snd_soc_codec *codec,
			     int enable, bool dapm)
{
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	if (ak4961->cdc_ext_clk) {
		dev_dbg(codec->dev, "%s: mclk_enable = %u, dapm = %d\n",
			__func__, enable, dapm);
		return __ak4961_mclk_enable(ak4961, enable);
	} else {
		dev_err(codec->dev,
			"%s: Cannot turn on MCLK\n",
			__func__);
		return -EINVAL;
	}
}
EXPORT_SYMBOL(ak4961_codec_mclk_enable);
//

static int ak4961_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct ak49xx *ak4961_core = dev_get_drvdata(dai->codec->dev->parent);
	pr_debug("%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);
	if ((ak4961_core != NULL) &&
	    (ak4961_core->dev != NULL) &&
	    (ak4961_core->dev->parent != NULL))
		pm_runtime_get_sync(ak4961_core->dev->parent);

	pr_debug("%s(): ak4961_dsp_mode = %d  dai_id = %d\n" , __func__,
				ak4961_dsp_mode, dai->id);
	return 0;
}

static void ak4961_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak49xx *ak4961_core = dev_get_drvdata(dai->codec->dev->parent);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(dai->codec);

	pr_debug("%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);
		 //for slim_spi 2014-11-17 星期一 14:16:12
	if (ak4961->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS&&ak4961->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return;

	if ((ak4961_core != NULL) &&
	    (ak4961_core->dev != NULL) &&
	    (ak4961_core->dev->parent != NULL)) {
		pm_runtime_mark_last_busy(ak4961_core->dev->parent);
		pm_runtime_put(ak4961_core->dev->parent);
	}

	pr_debug("%s(): ak4961_dsp_mode = %d  dai_id = %d\n" , __func__,
			ak4961_dsp_mode, dai->id);
	if (ak4961_dsp_mode != 0 && dai->id == SB3_CAP) {
		ak4961->state = AK4961_IDLE;
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		ak4961_dsp_mode = 0;
	}
//	switch (dai->id) {
//	case SB1_PB:
//	case SB1_CAP:
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x70, 0x00);
//		break;
//	case SB2_PB:
//	case SB2_CAP:
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x07, 0x00);
//		break;
//	case SB3_PB:
//	case SB3_CAP:
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x70, 0x00);
//		break;
//	default:
//		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
//		break;
//	}
}

static int ak4961_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(dai->codec);
	u8 MDV, BDV, SDV, CMF;

	pr_err("%s: chenjun: dai_name = %s DAI-ID %x rate %d num_ch %d format %d\n", __func__,
			dai->name, dai->id, params_rate(params),
			params_channels(params), params_format(params));

	switch (params_rate(params)) {
	case 8000:
		MDV = 0x0E;
		BDV = 0xEF;
		SDV = 0x3F;
		CMF = 0x40;
		break;
	case 16000:
		MDV = 0x0E;
		BDV = 0x77;
		SDV = 0x3F;
		CMF = 0x24;
		break;
	case 32000:
		MDV = 0x0E;
		BDV = 0x3B;
		SDV = 0x3F;
		CMF = 0x08;
		break;
	case 48000:
		MDV = 0x09;
		BDV = 0x27;
		SDV = 0x3F;
		CMF = 0x0A;
		break;
	case 96000:
		MDV = 0x04;
		BDV = 0x13;
		SDV = 0x3F;
		CMF = 0x0E;
		break;
	case 192000:
		MDV = 0x04;
		BDV = 0x09;
		SDV = 0x3F;
		CMF = 0xF2;
		break;
	default:
		pr_err("%s: Invalid sampling rate %d\n", __func__,
				params_rate(params));
		return -EINVAL;
	}
	ak4961->dai[dai->id].rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ak4961->dai[dai->id].bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		ak4961->dai[dai->id].bit_width = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ak4961->dai[dai->id].bit_width = 32;
		break;
	default:
		pr_err("%s: Invalid RX format %u\n", __func__,
				params_format(params));
		return -EINVAL;
	}

	pr_err("%s: chenjun: dai_name = %s DAI-ID %x bit_width %d\n", __func__,
			dai->name, dai->id, ak4961->dai[dai->id].bit_width);

#ifdef CONFIG_VOICE_WAKEUP
	if (snd_soc_read(codec, VAD_SETTING_1) & 0x08) {
		MDV = 0x1D;
		CMF = 0x04;
	}
#endif
	snd_soc_update_bits(codec, CDCMCLK_DIVIDER, 0xFF, MDV);
	snd_soc_update_bits(codec, MSYNC7_BDV, 0xFF, BDV);
	snd_soc_update_bits(codec, MSYNC7_SDV, 0xFF, SDV);
	snd_soc_update_bits(codec, CLOCK_MODE_SELECT, 0xFF, CMF);
	snd_soc_update_bits(codec, JITTER_CLEANER_SETTING_1, 0xFF, CMF);

	switch (dai->id) {
	case SB1_PB:
	case SB1_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x70, 0x70);
		break;
	case SB2_PB:
	case SB2_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x07, 0x07);
		break;
	case SB3_PB:
	case SB3_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x70, 0x70);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	return 0;
}

static int ak4961_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4961_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4961_set_fll(struct snd_soc_dai *dai, int id, int src,
			  unsigned int freq_in, unsigned int freq_out)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4961_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
//	struct snd_soc_codec *codec = codec_dai->codec;
	int mute_reg;
	int reg;

	switch (codec_dai->id) {
	case 1:
		mute_reg = 0x00;
		break;
	case 2:
		mute_reg = 0x01;
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		reg = 1;
	else
		reg = 0;

//	snd_soc_update_bits(codec, mute_reg, 0x00, reg);

	return 0;
}

static int ak4961_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)

{
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(dai->codec);
	struct ak49xx *core = dev_get_drvdata(dai->codec->dev->parent);

	if (!tx_slot && !rx_slot) {
		pr_err("%s: Invalid\n", __func__);
		return -EINVAL;
	}
	pr_debug("[LHS]%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n"
		 "ak4961->intf_type %d\n",
		 __func__, dai->name, dai->id, tx_num, rx_num,
		 ak4961->intf_type);
	//for slim_spi 2014-11-17 星期一 14:17:46
	if (ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS||ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		{
		//printk("[lhs] %s \n",__func__);
		ak49xx_init_slimslave(core, core->slim->laddr, tx_num, tx_slot, rx_num, rx_slot);
		}
	
	return 0;
}

static int ak4961_get_channel_map(struct snd_soc_dai *dai,
				unsigned int *tx_num, unsigned int *tx_slot,
				unsigned int *rx_num, unsigned int *rx_slot)

{
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(dai->codec);
	u32 i = 0;
	struct ak49xx_ch *ch;

	switch (dai->id) {
	case SB1_PB:
	case SB2_PB:
	case SB3_PB:
		if (!rx_slot || !rx_num) {
			pr_err("%s: Invalid rx_slot %p or rx_num %p\n",
				 __func__, rx_slot, rx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &ak4961_p->dai[dai->id].ak49xx_ch_list,
				    list) {
			rx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: rx_num %d\n", __func__, i);
		*rx_num = i;
		break;
	case SB1_CAP:
	case SB2_CAP:
	case SB3_CAP:
		if (!tx_slot || !tx_num) {
			pr_err("%s: Invalid tx_slot %p or tx_num %p\n",
				 __func__, tx_slot, tx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &ak4961_p->dai[dai->id].ak49xx_ch_list,
				    list) {
			tx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: tx_num %d\n", __func__, i);
		*tx_num = i;
		break;

	default:
		pr_err("%s: Invalid DAI ID %x\n", __func__, dai->id);
		break;
	}
	return 0;
}

static int ak4961_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_debug("%s cmd: %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
//		ak4961->stream_state = AK4961_SLIMBUS_STREAM_ON;
		ak4961->pcm_state = AK4961_SLIMBUS_STREAM_ON;
		queue_work(ak4961->workqueue, &ak4961->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
//		ak4961->stream_state = AK4961_SLIMBUS_STREAM_OFF;
		ak4961->pcm_state = AK4961_SLIMBUS_STREAM_OFF;
		queue_work(ak4961->workqueue, &ak4961->work);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct snd_soc_dai_ops ak4961_dai_ops = {
	.startup = ak4961_startup,
	.shutdown = ak4961_shutdown,
	.hw_params = ak4961_hw_params,
	.set_sysclk = ak4961_set_sysclk,
	.set_fmt = ak4961_set_fmt,
	.set_pll = ak4961_set_fll,
	.digital_mute	= ak4961_digital_mute,
	.set_channel_map = ak4961_set_channel_map,
	.get_channel_map = ak4961_get_channel_map,
	.trigger = ak4961_trigger,
};



static void ak4961_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x10, 0x00);
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x70, 0x00);
		break;
	case AIF_PORT2:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x20, 0x00);
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x07, 0x00);
		break;
	case AIF_PORT3:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x40, 0x00);
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x70, 0x00);
		break;
	case AIF_PORT4:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x80, 0x00);
//		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x07, 0x00);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		break;
	}
}

static int ak4961_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(dai->codec);
	u8 MDV, BDV, SDV, DLC, CMF;

	pr_debug("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
			dai->name, dai->id, params_rate(params),
			params_channels(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		DLC = 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		DLC = 0;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		DLC = 4;
		break;
	case SNDRV_PCM_FORMAT_A_LAW:
		DLC = 2;
		break;
	case SNDRV_PCM_FORMAT_MU_LAW:
		DLC = 3;
		break;
	default:
		pr_err("%s: invalid RX format %u\n", __func__,
				params_format(params));
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 8000:
		MDV = 0x0E;
		BDV = 0xEF;
		SDV = 0x3F;
		CMF = 0x40;
		break;
	case 16000:
		MDV = 0x0E;
		BDV = 0x77;
		SDV = 0x3F;
		CMF = 0x24;
		break;
	case 32000:
		MDV = 0x0E;
		BDV = 0x3B;
		SDV = 0x3F;
		CMF = 0x08;
		break;
	case 48000:
		MDV = 0x09;
		BDV = 0x27;
		SDV = 0x3F;
		CMF = 0x0A;
		break;
	case 96000:
		MDV = 0x04;
		BDV = 0x13;
		SDV = 0x3F;
		CMF = 0x0E;
		break;
	case 192000:
		MDV = 0x04;
		BDV = 0x09;
		SDV = 0x3F;
		CMF = 0xF2;
		break;
	default:
		pr_err("%s: Invalid sampling rate %d\n", __func__,
				params_rate(params));
		return -EINVAL;
	}

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(codec, CDCMCLK_DIVIDER, 0xFF, MDV);
		snd_soc_update_bits(codec, MSYNC1_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC1_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, SDTO1_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, CODEC_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x70, 0x10);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x10, 0x10);
		snd_soc_update_bits(codec, CLOCK_MODE_SELECT, 0xFF, CMF);
		snd_soc_update_bits(codec, JITTER_CLEANER_SETTING_1, 0xFF, CMF);
		break;
	case AIF_PORT2:
		snd_soc_update_bits(codec, MSYNC2_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC2_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, SDTO2_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x07, 0x02);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x20, 0x20);
		break;
	case AIF_PORT3:
		snd_soc_update_bits(codec, MSYNC3_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC3_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, SDTO3_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x70, 0x30);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x30, 0x30);
		break;
	case AIF_PORT4:
		snd_soc_update_bits(codec, MSYNC4_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC4_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, SDTO2_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SDTO4_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x07, 0x04);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x40, 0x40);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	ak4961->dai[dai->id].rate   = params_rate(params);
	return 0;
}

static int ak4961_i2s_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4961_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
//	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(dai->codec);
	u8 msn, dif, bckp;

	pr_debug("%s\n", __func__);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		msn = 0x20;		// CODEC is master
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		msn = 0x00;		// CODEC is slave
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_B:
		dif = 0x30;		// PCM Long Frame
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dif = 0x20;		// PCM Short Frame
		break;
	case SND_SOC_DAIFMT_I2S:
		dif = 0x00;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dif = 0x10;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		bckp = 0x00;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		bckp = 0x08;
		break;
	default:
		return -EINVAL;
	}

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(dai->codec, MSYNC1_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, SDTO1_AIF_FORMAT, 0x38, dif | bckp);
		break;
	case AIF_PORT2:
		snd_soc_update_bits(dai->codec, MSYNC2_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, SDTO2_AIF_FORMAT, 0x38, dif | bckp);
		break;
	case AIF_PORT3:
		snd_soc_update_bits(dai->codec, MSYNC3_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, SDTO3_AIF_FORMAT, 0x38, dif | bckp);
		break;
	case AIF_PORT4:
		snd_soc_update_bits(dai->codec, MSYNC4_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, SDTO4_AIF_FORMAT, 0x38, dif | bckp);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	return 0;
}

static int ak4961_i2s_set_fll(struct snd_soc_dai *dai, int id, int src,
			  unsigned int freq_in, unsigned int freq_out)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4961_i2s_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
//	struct snd_soc_codec *codec = codec_dai->codec;
	int mute_reg;
	int reg;

	switch (codec_dai->id) {
	case 1:
		mute_reg = 0x00;
		break;
	case 2:
		mute_reg = 0x01;
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		reg = 1;
	else
		reg = 0;

//	snd_soc_update_bits(codec, mute_reg, 0x00, reg);

	return 0;
}

static struct snd_soc_dai_ops ak4961_i2s_dai_ops = {
	.shutdown = ak4961_i2s_shutdown,
	.hw_params = ak4961_i2s_hw_params,
	.set_sysclk = ak4961_i2s_set_sysclk,
	.set_fmt = ak4961_i2s_set_fmt,
	.set_pll = ak4961_i2s_set_fll,
	.digital_mute	= ak4961_i2s_digital_mute,
	.trigger = ak4961_trigger,
};

static struct snd_soc_dai_driver ak4961_dai[] = {
	{
		.name = "ak4961_rx1",
		.id = SB1_PB,
		.playback = {
			.stream_name = "SB1 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_tx1",
		.id = SB1_CAP,
		.capture = {
			.stream_name = "SB1 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_rx2",
		.id = SB2_PB,
		.playback = {
			.stream_name = "SB2 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_tx2",
		.id = SB2_CAP,
		.capture = {
			.stream_name = "SB2 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_tx3",
		.id = SB3_CAP,
		.capture = {
			.stream_name = "SB3 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_rx3",
		.id = SB3_PB,
		.playback = {
			.stream_name = "SB3 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_vifeedback",
		.id = SB4_VIFEED,
		.capture = {
			.stream_name = "VIfeed",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = AK4961_FORMATS,
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 2,
			.channels_max = 2,
	 },
		.ops = &ak4961_dai_ops,
	},
	{
		.name = "ak4961_aif4",
		.id = AIF_PORT4,
		.playback = {
			.stream_name = "AIF4 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF4 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_i2s_dai_ops,
	},
};


static struct snd_soc_dai_driver ak4961_i2s_dai[] = {
	{
		.name = "ak4961_aif1",
		.id = AIF_PORT1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4961_i2s_dai_ops,
	},
	{
		.name = "ak4961_aif2",
		.id = AIF_PORT2,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_i2s_dai_ops,
	},
	{
		.name = "ak4961_aif3",
		.id = AIF_PORT3,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_i2s_dai_ops,
	},
	{
		.name = "ak4961_aif4",
		.id = AIF_PORT4,
		.playback = {
			.stream_name = "AIF4 Playback",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF4 Capture",
			.rates = AK4961_RATES,
			.formats = AK4961_FORMATS_AIF,
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_i2s_dai_ops,
	},
};


static const char *mic1_power_level[] = {
	"V2.8", "V2.5", "V1.8", "AVDD1"
};

static const char *mic2_power_level[] = {
	"V2.8", "V2.5", "V1.8", "N/A"
};

static const struct soc_enum mic_1_power_level =
	SOC_ENUM_SINGLE(MIC_POWER_LEVEL, 0, 4, mic1_power_level);

static const struct soc_enum mic_2_power_level =
	SOC_ENUM_SINGLE(MIC_POWER_LEVEL, 2, 4, mic2_power_level);

/*
 * MIC-Amp gain control:
 * from 0 to 30 dB in 3 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 300, 0);

/*
 * Digital output volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dout_vol_tlv, -1250, 50, 1);

/*
 * HP-Amp volume control:
 * from -42 to 6 dB in 2 dB steps (mute instead of -42 dB)
 */
static DECLARE_TLV_DB_SCALE(hp_out_tlv, -4200, 200, 1);

/*
 * Lineout1 volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(lineout1_tlv, -750, 150, 0);

/*
 * Lineout2 volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(lineout2_tlv, -750, 150, 0);

/*
 * Receiver volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(receiver_tlv, -750, 150, 0);

/* ADC HPF Filter Enable */
static const char *adc_hpf_enable[] = {
	"hpf_on", "hpf_off"
};

static const struct soc_enum ad1_hpf_enable =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 0, 2, adc_hpf_enable);

static const struct soc_enum ad2_hpf_enable =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 0, 2, adc_hpf_enable);

/* cut of frequency for high pass filter*/
static const char *hp_cf_text[] = {
	"fs@3.4Hz=44.1kHz", "fs@13.6Hz=44.1kHz", "fs@108.8Hz=44.1kHz", "fs@217.6Hz=44.1kHz"
};

static const struct soc_enum ad1_hp_cf =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 1, 4, hp_cf_text);

static const struct soc_enum ad2_hp_cf =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 1, 4, hp_cf_text);

/* ADC Digital Filter Setting */
static const char *adc_df_text[] = {
	"sharp_roll_off_72", "sharp_roll_off_88", "minimum_delay_72", "reserved"
};

static const struct soc_enum ad1_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 4, 4, adc_df_text);

static const struct soc_enum ad2_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 4, 4, adc_df_text);

/* DAC Filter Bypass */
static const char *dac_df_thr[] = {
	"df_filter", "df_thr"
};

static const struct soc_enum da1_filter_through =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 3, 2, dac_df_thr);

static const struct soc_enum da2_filter_through =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 3, 2, dac_df_thr);

/* DAC Digital Filter Setting */
static const char *dac_df_text[] = {
	"sharp_roll_off", "slow_roll_off", "minimum_delay", "minimum_delay_slow"
};

static const struct soc_enum da1_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 6, 4, dac_df_text);

/* DAC Mixer PGA Setting */
static const char *dac_mix_pga_text[] = {
	"bypass", "half", "minus", "half_minus"
};

static const struct soc_enum da1_lch_mix_pga =
	SOC_ENUM_SINGLE(DAC1_MONO_MIXING, 2, 4, dac_mix_pga_text);

static const struct soc_enum da1_rch_mix_pga =
	SOC_ENUM_SINGLE(DAC1_MONO_MIXING, 6, 4, dac_mix_pga_text);

static const struct soc_enum da2_lch_mix_pga =
	SOC_ENUM_SINGLE(DAC2_MONO_MIXING, 2, 4, dac_mix_pga_text);

static const struct soc_enum da2_rch_mix_pga =
	SOC_ENUM_SINGLE(DAC2_MONO_MIXING, 6, 4, dac_mix_pga_text);

/* Charge Pump1 clock setting */
static const char *cp1_clock_sel_text[] = {
	"500kHz", "250kHz", "125kHz", "62.5kHz"
};

static const struct soc_enum cp1_clock_sel =
	SOC_ENUM_SINGLE(CHARGE_PUMP_1_SETTING_1, 0, 4, cp1_clock_sel_text);
	
/* HP Zero Cross Timeout Setting */
static const char *hp_zctm_text[] = {
	"128/fs", "256/fs", "512/fs", "1024/fs", "2048/fs"
};

static const struct soc_enum hp_zero_cross_timeout =
	SOC_ENUM_SINGLE(HP_VOLUME_CONTROL, 5, 5, hp_zctm_text);

/*
 * Mixer A PGA gain control:
 * from -12 to 0 dB in 6 dB steps (mute instead of -12 dB)
 */
static DECLARE_TLV_DB_SCALE(mixer_gain_tlv, -1200, 600, 1);

/* Mixer A SWAP Setting */
static const char *mixer_swap_text[] = {
	"through", "lin", "rin", "swap"
};

static const struct soc_enum mixer_a1_swap =
	SOC_ENUM_SINGLE(MIXER_A_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_a2_swap =
	SOC_ENUM_SINGLE(MIXER_A_CONTROL, 2, 4, mixer_swap_text);

static const struct soc_enum mixer_b1_swap =
	SOC_ENUM_SINGLE(MIXER_B_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_b2_swap =
	SOC_ENUM_SINGLE(MIXER_B_CONTROL, 2, 4, mixer_swap_text);

/* SRC Output Sync Domain Setting */
static const char *sync_domain_text[] = {
	"n/a", "aif1", "aif2", "aif3", "aif4", "dsp24k", "dsp16k", "slimbus"
};

static const struct soc_enum srcao_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR7, 4, 8, sync_domain_text);

static const struct soc_enum srcbo_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR7, 0, 8, sync_domain_text);

static const struct soc_enum srcco_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR8, 4, 8, sync_domain_text);

static const struct soc_enum srcdo_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR8, 0, 8, sync_domain_text);

static const struct soc_enum dsp_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR5, 0, 8, sync_domain_text);

static const struct soc_enum dspo2_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR6, 4, 8, sync_domain_text);

static const struct soc_enum dspo4_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR6, 0, 8, sync_domain_text);

static const struct soc_enum cdc_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR3, 0, 8, sync_domain_text);

static const struct soc_enum port2_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR1, 0, 8, sync_domain_text);

static const struct soc_enum port3_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR2, 4, 8, sync_domain_text);

static const struct soc_enum port4_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR2, 0, 8, sync_domain_text);

static const char *dsp_status_text[] = {
	"off", "on"
};

static const struct soc_enum dsp_downlink_status_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_status_text), dsp_status_text);

static int ak4961_get_dsp_downlink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4961->ak4961_dsp_downlink_status;

	return 0;
}

static int ak4961_set_dsp_downlink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4961_dsp_mode >= DSP_MODE_SOUND_RECORD &&
		ak4961_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x25;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(ak49xx, val);
	}
		
	if (ret == 0) {
		ak4961->ak4961_dsp_downlink_status = status;
	}

	if (ak4961->ak4961_dsp_uplink_status == 0 &&
		ak4961->ak4961_dsp_downlink_status == 0) {

		ak4961_dsp_mode = DSP_MODE_OFF;
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		ak4961->state = AK4961_IDLE;
	}

	return ret;
}

static const struct soc_enum dsp_uplink_status_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_status_text), dsp_status_text);

static int ak4961_get_dsp_uplink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4961->ak4961_dsp_uplink_status;

	return 0;
}

static int ak4961_set_dsp_uplink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4961_dsp_mode >= DSP_MODE_SOUND_RECORD &&
		ak4961_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x13;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(ak49xx, val);
	}
	
	if (ret == 0) {
		ak4961->ak4961_dsp_uplink_status = status;
	}

	if (ak4961->ak4961_dsp_uplink_status == 0 &&
		ak4961->ak4961_dsp_downlink_status == 0) {

		ak4961_dsp_mode = DSP_MODE_OFF;
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		ak4961->state = AK4961_IDLE;
	}

	return ret;
}

#if 0
static const char *dsp_mode_text[] = {
	"off",
	"narrow_handset", "narrow_headset", "narrow_handsfree",
	"wide_handset", "wide_headset", "wide_handsfree",
	"voice_recognition",
	"sound_record", "voice_record",
	"music_speaker",
	"barge_in",
	"karaoke_heavy", "karaoke_light", "karaoke_middle"
};
#else
#ifdef AKM_VOICE_CALL
static const char * const dsp_mode_text[] = {
	"off",
	"narrow_handset", "narrow_headset", "narrow_handsfree",
	"wide_handset", "wide_headset", "wide_headphone", "wide_handsfree",
	"voice_recognition",
	"sound_record", "voice_record", "wchat_record",
	"music_speaker",
	"interview_record",
	"barge_in",
	"karaoke_live", "karaoke_room", "karaoke_cave",
	"karaoke_spk_room", "karaoke_spk_live", "karaoke_spk_theater"
};
#else
static const char *dsp_mode_text[] = {
	"off",
	"narrow_handset", "narrow_headset", "narrow_handsfree",
	"wide_handset", "wide_headset", "wide_handsfree",
	"voice_recognition",
	"sound_record", "voice_record", "wchat_record",
	"music_speaker",
	"interview_record",
	"barge_in",
	"karaoke_live", "karaoke_room", "karaoke_cave",
	"karaoke_spk_room", "karaoke_spk_live", "karaoke_spk_theater"	
};
#endif
#endif

#if 0
static const u16 ram_table[] = {
	0x010, 0x020, 0x030,	// NARROW_MODE
	0x140, 0x150, 0x160,	// WIDE_MODE
	0x270,					// Voice Recognition
	0x380, 0x390,			// Sound Voice Record
	0x4A0,					// Music Speaker
	0x500,					// Barge-in
	0x6B1, 0x6C2, 0x6D3		// Karaoke
};
#else
#ifdef AKM_VOICE_CALL
static const u16 ram_table[] = {
	0x0010, 0x0020, 0x0030,			/* NARROW_MODE*/
	0x1040, 0x1050, 0x1060, 0x1070,	/* WIDE_MODE*/
	0x2080,							/* Voice Recognition*/
	0x3090, 0x30A0, 0x30B0,			/* Sound Record for recorder Voice Record for camera*/
	0x40C0,							/* Music Speaker*/
	0x50D0,							/* interview_record specially for WeChat*/
	0x6000,							/* Barge-in*/
	0x70E0, 0x70F0, 0x7100,			/* Karaoke*/
	0x8111, 0x8121, 0x8131			/* Karaoke_SPK*/
};
#else
static const u16 ram_table[] = {
	0x0010, 0x0020, 0x0030,	// NARROW_MODE
	0x1040, 0x1050, 0x1060,	// WIDE_MODE
	0x2070,					// Voice Recognition
	0x3080, 0x3090, 0x30A0,	// Sound Record for recorder Voice Record for camera
	0x40B0,					// Music Speaker
	0x50C0,					// interview_record specially for WeChat
	0x6000,					// Barge-in
	0x70D0, 0x70E0, 0x70F0,	// Karaoke
	0x8101, 0x8111, 0x8121	// Karaoke_SPK
};
#endif
#endif

static const struct soc_enum dsp_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_mode_text), dsp_mode_text),
};

static int ak4961_get_dsp_mode(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4961_dsp_mode;
	return 0;
}

static int ak4961_set_dsp_mode(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	//for slim_spi 2014-11-17 星期一 14:23:30
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	struct ak49xx_core_resource *core_res=&ak49xx->core_res;
	int new_dsp_mode = ucontrol->value.integer.value[0];
	int interface = ak49xx_get_intf_type();
	int ret = 0;
	int dsp_sync_domain = 0;

	u8  pram_index, cram_index, ex_pram_index, ex_cram_index;
	u8 aram_index,ex_aram_index;
	size_t pram_size = 0, cram_size = 0, aram_size= 0;
	u8	crc[2];
	u8 *pram_fwbuf = 0, *cram_fwbuf = 0, *aram_fwbuf= 0;
	u8	val[10];
	int reg_srcab,reg_srccd;

	if (ak4961_dsp_mode == new_dsp_mode) {
		return 0;
	}

	if (new_dsp_mode == DSP_MODE_OFF &&
		(ak4961->ak4961_dsp_downlink_status == 1 ||
		 ak4961->ak4961_dsp_uplink_status == 1)
		) {
		return 0;
	}

	if (new_dsp_mode != DSP_MODE_OFF) {

		pram_index = ram_table[new_dsp_mode - 1] >> 12;
		cram_index = (ram_table[new_dsp_mode - 1] >> 4) % 0x100;
		aram_index= ram_table[new_dsp_mode - 1] % 0x10;
		pr_debug("%s: pram_index =%d cram_index =%d ,aram_index = %d \n",__func__,pram_index,cram_index,aram_index);
		if (ak4961_dsp_mode != DSP_MODE_OFF) {

			ex_pram_index = ram_table[ak4961_dsp_mode - 1] >> 12;
			ex_cram_index = (ram_table[ak4961_dsp_mode - 1] >> 4) % 0x100;
			ex_aram_index = ram_table[ak4961_dsp_mode -1] % 0x10;
			
			dsp_sync_domain = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR5);
			if (dsp_sync_domain & 0x07) {
				snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07, 0x00);
				usleep_range(16000, 16000);
			}

			snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x00);
			snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x01);

		} else {
			ex_pram_index = ex_cram_index =ex_aram_index= 0xff;
			reg_srcab=snd_soc_read(codec,SYNC_DOMAIN_SELECTOR7);
			reg_srccd=snd_soc_read(codec,SYNC_DOMAIN_SELECTOR8);

			snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x01, 0x01);
			snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x03);
			snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, reg_srcab);
			snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, reg_srccd);
			snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x01);
		}
		//for slim_spi 2014-11-17 星期一 14:24:59
		if (pram_index != ex_pram_index&& ak4961->pram_firmware[pram_index] != NULL) {

			pram_fwbuf = kmemdup(ak4961->pram_firmware[pram_index]->data,
					ak4961->pram_firmware[pram_index]->size, GFP_KERNEL);
			if (!pram_fwbuf) {
				pr_err("%s: MEM Allocation for PRAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				pram_size = ak4961->pram_firmware[pram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI||interface ==  AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				pram_size = ak4961->pram_firmware[pram_index]->size - 2;
			}
		} else {
			pram_fwbuf = 0;
		}
		//for slim_spi 2014-11-17 星期一 14:27:41
		if (cram_index != ex_cram_index&&ak4961->cram_firmware[cram_index]!=NULL) {

			cram_fwbuf = kmemdup(ak4961->cram_firmware[cram_index]->data,
					ak4961->cram_firmware[cram_index]->size, GFP_KERNEL);
			if (!cram_fwbuf) {
				pr_err("%s: MEM Allocation for CRAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				cram_size = ak4961->cram_firmware[cram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI || interface == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				cram_size = ak4961->cram_firmware[cram_index]->size - 2;
			}
		} else {
			cram_fwbuf = 0;
		}


		if(aram_index != ex_aram_index && ak4961->aram_firmware[aram_index] !=NULL)
		{
			aram_fwbuf = kmemdup(ak4961->aram_firmware[aram_index]->data,
					ak4961->aram_firmware[aram_index]->size, GFP_KERNEL);
			if (!aram_fwbuf) {
				pr_err("%s: MEM Allocation for ARAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				aram_size = ak4961->aram_firmware[aram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI || interface == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				aram_size = ak4961->aram_firmware[aram_index]->size - 2;
			}
		}
		else {
			aram_fwbuf = 0;
			}

		if (pram_fwbuf) {

			snd_soc_update_bits(codec, PRAM_READY, 0x01, 0x01);

			ret += ak49xx_ram_write(ak49xx, 0x04, 0x00, 0x00,
					pram_size, pram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
				//for slim_spi 2014-11-17 星期一 14:30:52
				//ret += ak49xx_bulk_read(ak49xx,
				//		CRC_RESULT_H8, 2, crc);
				ret += ak49xx_bulk_read(core_res,CRC_RESULT_H8,2,crc);
				//if (((pram_fwbuf[pram_size + 1] << 8) + pram_fwbuf[pram_size + 2])
				if (((pram_fwbuf[pram_size ] << 8) + pram_fwbuf[pram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: PRAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: PRAM download failed\n", __func__);
			}

			snd_soc_update_bits(codec, PRAM_READY, 0x01, 0x00);
		}

		if (cram_fwbuf) {

			ret += ak49xx_ram_write(ak49xx, 0x05, 0x00, 0x00,
					cram_size, cram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
								//for slim_spi 2014-11-17 星期一 14:30:52
				//ret += ak49xx_bulk_read(ak49xx,
				//		CRC_RESULT_H8, 2, crc);
				ret += ak49xx_bulk_read(core_res,CRC_RESULT_H8,2,crc);
				//if (((cram_fwbuf[cram_size + 1] << 8) + cram_fwbuf[cram_size + 2])
				if (((cram_fwbuf[cram_size] << 8) + cram_fwbuf[cram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: CRAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: CRAM download failed\n", __func__);
			}
		}
		if (aram_fwbuf) {

			ret += ak49xx_ram_write(ak49xx, 0x06, 0x00, 0x00,
					aram_size, aram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
				ret += ak49xx_bulk_read(core_res, CRC_RESULT_H8, 2, crc);
				if (((aram_fwbuf[aram_size] << 8) + aram_fwbuf[aram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: ARAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: ARAM download failed\n", __func__);
			}
		}

		snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x00);
	}

	if (ret == 0) {
		ak4961_dsp_mode = new_dsp_mode;
		pr_info("%s: new mode = %d\n", __func__, new_dsp_mode);
		ak4961_band_set = 0;

		switch (ak4961_dsp_mode) {
		case DSP_MODE_OFF:
			break;
		case DSP_MODE_NARROW_HANDSET:
		case DSP_MODE_NARROW_HEADSET:
		case DSP_MODE_NARROW_HANDSFREE:
			snd_soc_write(codec, DSP_SETTING1, 0x68);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_WIDE_HANDSET:
		case DSP_MODE_WIDE_HEADSET:
#ifdef AKM_VOICE_CALL
		case DSP_MODE_WIDE_HEADPHONE:
#endif
		case DSP_MODE_WIDE_HANDSFREE:
			snd_soc_write(codec, DSP_SETTING1, 0x68);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x0A);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_VOICE_RECOGNITION:
			snd_soc_write(codec, DSP_SETTING1, 0x68);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x0B);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_SOUND_RECORD:
		case DSP_MODE_VOICE_RECORD:
		case DSP_MODE_WCHAT_RECORD:
		case DSP_MODE_MUSIC_SPEAKER:
			snd_soc_write(codec, DSP_SETTING1, 0x22);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x0A);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_KARAOKE_HEAVY: // ZTE_chenjun:i.e. karaoke_live 
		case DSP_MODE_KARAOKE_LIGHT: // ZTE_chenjun:i.e. karaoke_room
		case DSP_MODE_KARAOKE_MIDDLE: // ZTE_chenjun:i.e. karaoke_cave
		case DSP_MODE_BARGE_IN:
			snd_soc_write(codec, DSP_SETTING1, 0x21);
			snd_soc_write(codec, DSP_SETTING2, 0x00);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);		
			break;
		case DSP_MODE_KARAOKE_SPK_ROOM:
		case DSP_MODE_KARAOKE_SPK_LIVE:
		case DSP_MODE_KARAOKE_SPK_THEATER:
			snd_soc_write(codec, DSP_SETTING1, 0x62);
			snd_soc_write(codec, DSP_SETTING2, 0x73);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
// ZTE_chenjun
		case DSP_MODE_INTERVIEW_RECORD:
			snd_soc_write(codec, DSP_SETTING1, 0x68); 
			snd_soc_write(codec, DSP_SETTING2, 0x3C); 
			snd_soc_write(codec, DSP_SETTING3, 0x0A); 
			snd_soc_write(codec, DSP_SETTING5, 0x01); 
			break;
//
		default:
			break;
		}

		if (ak4961_dsp_mode == DSP_MODE_OFF) {
			snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
			ak4961->state = AK4961_IDLE;
		} else {
//			snd_soc_write(codec, FLOW_CONTROL_3, 0x01);
			if (dsp_sync_domain & 0x07) {
				snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07,
						dsp_sync_domain);
			}
			ak4961->state = AK4961_DSPRSTNON;
		}
	}

	if (ak4961_dsp_mode >= DSP_MODE_SOUND_RECORD &&
		ak4961_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[6] = 0x00;
		val[7] = 0x01;
		val[8] = 0;

		if (ak4961->ak4961_dsp_uplink_status) {
			val[5] = 0x13;
			ret = ak49xx_run_ram_write(ak49xx, val);

		} else if (ak4961->ak4961_dsp_downlink_status) {
			val[5] = 0x25;
			ret = ak49xx_run_ram_write(ak49xx, val);
		}

		if (ret) {
			pr_err("%s: ram_write_err %d\n", __func__, ret);
		}
	}

	if (pram_fwbuf) {
		kfree(pram_fwbuf);
	}
	if (cram_fwbuf) {
		kfree(cram_fwbuf);
	}
	if (aram_fwbuf) {
		kfree(aram_fwbuf);
	}
	return ret;
}
/*#if 1*/
int ak49xx_bandswitch_set(struct snd_soc_codec *codec, int howtochange)
{
	u8 val[9];
	int mir3_2;
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
#if 0
	if (ak4961_band_set == 1) {
		pr_err("[LHS]%s the band have set !\n", __func__);
		return 0;
	}
#endif
	mir3_2 = snd_soc_read(codec, MIR3_REGISTER_2);
	pr_info("[LHS] %s mir3_2 voice mode =%x howtochange=%d\n", __func__, mir3_2, howtochange);

	if (howtochange == 1) {
		/* switch to norrow band*/
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x0a;
		val[5] = 0x95;
		val[6] = 0x00;
		val[7] = 0x00;
		val[8] = 0x00;
		ak4961_band_set = 1;
		return ak49xx_run_ram_write(ak49xx, val);
	} else if (howtochange == 2) {
		/* switch to wide band*/
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x0a;
		val[5] = 0x95;
		val[6] = 0x00;
		val[7] = 0x01;
		val[8] = 0x00;
		ak4961_band_set = 1;
		return ak49xx_run_ram_write(ak49xx, val);
	}

	pr_err("[LHS] %s set akm codec band switch mode failed!\n", __func__);

	return 0;
}
/*#endif*/
static int akm_internal_rx_gain_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_rx_gain;
	return 0;
}

static int akm_internal_rx_gain_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	internal_rx_gain = ucontrol->value.integer.value[0];
//	u8	val[9];
//	printk("[lhs] %s internal_rx_gain=%d\n",__func__,internal_rx_gain);
/*	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x06;
	val[5] = 0x09;
	val[6] = 0x00;
	val[7] = internal_rx_gain;
	val[8] = 0;*/
	snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
	snd_soc_write(codec,RUN_STATE_START_ADDR1,0x06);
	snd_soc_write(codec,RUN_STATE_START_ADDR2,0x0c);
	snd_soc_write(codec,RUN_STATE_DATA_1,0x00);
	snd_soc_write(codec,RUN_STATE_DATA_2,internal_rx_gain);
	snd_soc_write(codec,RUN_STATE_DATA_3,0x00);

	snd_soc_write(codec,CRAM_RUN_EXE,0x01);
	return 0;
//	return ak49xx_run_ram_write(ak49xx, val);
}

int ak4961_internal_rx_gain_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int internal_rx_gain = ucontrol->value.integer.value[0];
	codec=ak4961_codec;
	if(codec==NULL)
		{
		printk("[lhs] %s no codec! ",__func__);
		return -1;
		}
	//printk("[lhs] %s internal_rx_gain=%d codec_name=%s\n",__func__,internal_rx_gain,codec->name);
	internal_rx_gain=6-internal_rx_gain;
	snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
	snd_soc_write(codec,RUN_STATE_START_ADDR1,0x06);
	snd_soc_write(codec,RUN_STATE_START_ADDR2,0x0c);
	snd_soc_write(codec,RUN_STATE_DATA_1,0x00);
	snd_soc_write(codec,RUN_STATE_DATA_2,internal_rx_gain);
	snd_soc_write(codec,RUN_STATE_DATA_3,0x00);

	snd_soc_write(codec,CRAM_RUN_EXE,0x01);
	
	return 0;
}

/*add by shengguanghui for karaoke 20150606 begin*/
static DECLARE_TLV_DB_SCALE(karaoke_mic_gain_tlv, 000, 100, 0);
static int akm_karaoke_mic_gain_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = karaoke_mic_gain;
	return 0;
}

static int akm_karaoke_mic_gain_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	karaoke_mic_gain = ucontrol->value.integer.value[0];

	printk("%s karaoke_mic_gain=%d\n",__func__,karaoke_mic_gain);

	codec = ak4961_codec;
	if(codec == NULL){
		printk("%s no codec!\n",__func__);
		return -1;
	}
 
	snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x01);
	
	if (	ak4961_dsp_mode >= DSP_MODE_KARAOKE_HEAVY &&
			ak4961_dsp_mode <= DSP_MODE_KARAOKE_MIDDLE) {
	snd_soc_write(codec,RUN_STATE_START_ADDR1,0x01);
	snd_soc_write(codec,RUN_STATE_START_ADDR2,0x00);
	} else if (ak4961_dsp_mode >= DSP_MODE_KARAOKE_SPK_ROOM &&
			ak4961_dsp_mode <= DSP_MODE_KARAOKE_SPK_THEATER) {
		snd_soc_write(codec,RUN_STATE_START_ADDR1,0x00);
		snd_soc_write(codec,RUN_STATE_START_ADDR2,0xCC);		
	}
	
	snd_soc_write(codec,RUN_STATE_DATA_1,mic_gain_table2[karaoke_mic_gain]>>16);
	snd_soc_write(codec,RUN_STATE_DATA_2,(mic_gain_table2[karaoke_mic_gain]>>8)%0x100);
	snd_soc_write(codec,RUN_STATE_DATA_3,mic_gain_table2[karaoke_mic_gain]%0x100);
	snd_soc_write(codec,RUN_STATE_DATA_4,mic_gain_table2[karaoke_mic_gain]>>16);
	snd_soc_write(codec,RUN_STATE_DATA_5,(mic_gain_table2[karaoke_mic_gain]>>8)%0x100);
	snd_soc_write(codec,RUN_STATE_DATA_6,mic_gain_table2[karaoke_mic_gain]%0x100);
	snd_soc_write(codec,CRAM_RUN_EXE,0x01);

	return 0;

}
/*add by shengguanghui for karaoke 20150606 end*/

static const char *voice_band_switch_text[] = {
        "nb", "wb"
};
 
static const struct soc_enum voice_band_switch_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_band_switch_text), voice_band_switch_text);

static int ak4961_bandswitch_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{	
	ucontrol->value.enumerated.item[0] = ak4961_band_mode;	
	return 0;
}
#if 0
static int ak4961_bandswitch_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = ak4961_codec;
	int howtochange = ucontrol->value.integer.value[0];
	int mir3_1,mir3_2,mir3_3,mir3_4;
	
	if(codec==NULL)
		{
		pr_err("[lhs] %s no codec! ",__func__);
		return -1;
		}

	pr_err("[lhs] %s: howtochange(%d)",__func__, howtochange);

	mir3_1=snd_soc_read(codec, MIR3_REGISTER_1);
	mir3_2=snd_soc_read(codec, MIR3_REGISTER_2);
	mir3_3=snd_soc_read(codec, MIR3_REGISTER_3);
	mir3_4=snd_soc_read(codec, MIR3_REGISTER_4);
	if(!(mir3_1||mir3_2||mir3_3||mir3_4))
		{
		pr_err("[lhs] %s set to handset\n",__func__);
		// in handset mode need change the PEQ
		//if N to W
		if(howtochange==1)
			{
			snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x04);
			snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			snd_soc_write(codec,RUN_STATE_START_ADDR2,0x96);
			snd_soc_write(codec,RUN_STATE_DATA_1,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_2,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_3,0x00);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x97);
			snd_soc_write(codec,RUN_STATE_DATA_4,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_5,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_6,0x00);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x98);
			snd_soc_write(codec,RUN_STATE_DATA_7,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_8,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_9,0x00);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x99);
			snd_soc_write(codec,RUN_STATE_DATA_10,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_11,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_12,0x00);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x9a);
			snd_soc_write(codec,RUN_STATE_DATA_13,0x40);
			snd_soc_write(codec,RUN_STATE_DATA_14,0x00);
			snd_soc_write(codec,RUN_STATE_DATA_15,0x00);
                        snd_soc_write(codec,CRAM_RUN_EXE,0x01);
			}		
		else if(howtochange==0)//else W to N
			{
			snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x04);
			snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			snd_soc_write(codec,RUN_STATE_START_ADDR2,0x96);
			snd_soc_write(codec,RUN_STATE_DATA_1,0x3c);
			snd_soc_write(codec,RUN_STATE_DATA_2,0x8a);
			snd_soc_write(codec,RUN_STATE_DATA_3,0xe0);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x97);
			snd_soc_write(codec,RUN_STATE_DATA_4,0x86);
			snd_soc_write(codec,RUN_STATE_DATA_5,0xea);
			snd_soc_write(codec,RUN_STATE_DATA_6,0x40);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x98);
			snd_soc_write(codec,RUN_STATE_DATA_7,0xc6);
			snd_soc_write(codec,RUN_STATE_DATA_8,0xba);
			snd_soc_write(codec,RUN_STATE_DATA_9,0x60);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x99);
			snd_soc_write(codec,RUN_STATE_DATA_10,0x78);
			snd_soc_write(codec,RUN_STATE_DATA_11,0xe5);
			snd_soc_write(codec,RUN_STATE_DATA_12,0xf0);
			//snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
			//snd_soc_write(codec,RUN_STATE_START_ADDR1,0x05);
			//snd_soc_write(codec,RUN_STATE_START_ADDR2,0x9a);
			snd_soc_write(codec,RUN_STATE_DATA_13,0x3c);
			snd_soc_write(codec,RUN_STATE_DATA_14,0x8a);
			snd_soc_write(codec,RUN_STATE_DATA_15,0xe0);
                        snd_soc_write(codec,CRAM_RUN_EXE,0x01);
			}
		
		}

	// change the FIR control 
	// if N to W 
	if(howtochange==1)
		{
		snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
		snd_soc_write(codec,RUN_STATE_START_ADDR1,0x06);
		snd_soc_write(codec,RUN_STATE_START_ADDR2,0x06);
		snd_soc_write(codec,RUN_STATE_DATA_1,0x00);
		snd_soc_write(codec,RUN_STATE_DATA_2,0x00);
		snd_soc_write(codec,RUN_STATE_DATA_3,0x00);
		snd_soc_write(codec,CRAM_RUN_EXE,0x01);
		}
	else if(howtochange==0)// else W to N
		{
		snd_soc_write(codec,RUN_STATE_DATA_LENGTH,0x00);
		snd_soc_write(codec,RUN_STATE_START_ADDR1,0x06);
		snd_soc_write(codec,RUN_STATE_START_ADDR2,0x06);
		snd_soc_write(codec,RUN_STATE_DATA_1,0x00);
		snd_soc_write(codec,RUN_STATE_DATA_2,0x13);
		snd_soc_write(codec,RUN_STATE_DATA_3,0x00);
		snd_soc_write(codec,CRAM_RUN_EXE,0x01);
		}

	ak4961_band_mode=howtochange;
	return 0;
}
#else
static int ak4961_bandswitch_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = ak4961_codec;
	int howtochange = ucontrol->value.integer.value[0];
	int ret = 0;

	if (howtochange < 0 || howtochange > 3) {
		pr_err("%s: howtochange = %d out of scope(ret must 0~3)\n", __func__, howtochange);
		return 1;
	}

	ak4961_band_mode = howtochange;

	ret = ak49xx_bandswitch_set(codec, ak4961_band_mode);

	pr_err("%s: ak4961_band_mode = %d, ret =  %d\n", __func__, ak4961_band_mode, ret);
	return 0;
}
#endif
/*
 * Internal Rx Out Volume control:
 * from -18 to 0 dB in 3 dB steps
 */
static DECLARE_TLV_DB_SCALE(rx_out_gain_tlv, -1800, 300, 0);

static int akm_internal_mic_gain_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_mic_gain;
	return 0;
}

static int akm_internal_mic_gain_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	val[12];

	internal_mic_gain = ucontrol->value.integer.value[0];

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x01;
	val[4] = 0x01;
	val[5] = 0x04;
	val[6] = mic_gain_table[internal_mic_gain] >> 16;
	val[7] = (mic_gain_table[internal_mic_gain] >> 8) % 0x100;
	val[8] = mic_gain_table[internal_mic_gain] % 0x100;
	val[9] = val[6];
	val[10] = val[7];
	val[11] = val[8];

	return ak49xx_run_ram_write(ak49xx, val);
}

/*
 * Internal MIC Out Volume control:
 * from 0 to +20 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_out_gain_tlv, 000, 100, 0);

static int akm_internal_ahpf_sensitivity_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_ahpf_sensitivity;
	return 0;
}

static int akm_internal_ahpf_sensitivity_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	val[12];
	internal_ahpf_sensitivity = ucontrol->value.integer.value[0];

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x01;
	val[4] = 0x01;
	val[5] = 0x44;
	val[6] = internal_ahpf_sensitivity;
	val[7] = 0x00;
	val[8] = 0x00;
	val[9] = internal_ahpf_sensitivity >> 1;
	if ((internal_ahpf_sensitivity % 2) != 0)
		val[10] = 0x80;
	else
		val[10] = 0x00;
	val[11] = 0x00;

	return ak49xx_run_ram_write(ak49xx, val);
}

/*
 * Internal MIC Out Volume control:
 * from 0 to +13 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(ahpf_sensitivity_tlv, 000, 100, 0);

//ZTE_chenjun:
/* bargein path selector */
static const char *bargein_sel_text[] = {
	"n/a","output1_l", "output1_r", "output3_l", "output3_r",
	"input5_l", "input5_r", "input5_m"
};

static const struct soc_enum bargein_sel_text_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bargein_sel_text), bargein_sel_text);

static int ak4961_get_lch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4961->lch_bargein_sel;

	return 0;
}

static int ak4961_set_lch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret;

	pr_err("%s: status = %d\n", __func__, status);

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x23;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(ak49xx, val);
	if (ret == 0) {
		ak4961->lch_bargein_sel = status;
	}

	return ret;
}

static int ak4961_get_rch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4961->rch_bargein_sel;

	return 0;
}

static int ak4961_set_rch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret;

	pr_err("%s: status = %d\n", __func__, status);

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x24;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(ak49xx, val);
	if (ret == 0) {
		ak4961->rch_bargein_sel = status;
	}

	return ret;
}

static const char *voice_sync_switch_text[] = {
        "off", "on"
};
 
static const struct soc_enum voice_sync_switch_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_sync_switch_text), voice_sync_switch_text);
 
 
static int ak4961_get_voice_sync_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4961->ak4961_voice_sync_switch;
 
        return 0;
}
 
static int ak4961_set_voice_sync_switch(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
        struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
        u8      status = ucontrol->value.integer.value[0];
        u8      val[9];
        int     ret = 0;
 
        pr_info("%s: status = %d\n", __func__, status);
 
        if (ak4961_dsp_mode >= DSP_MODE_SOUND_RECORD &&
                        ak4961_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {
 
                val[0] = 0x81;
                val[1] = RUN_STATE_DATA_LENGTH >> 8;
                val[2] = RUN_STATE_DATA_LENGTH & 0xff;
                val[3] = 0x00;
                val[4] = 0x01;
                val[5] = 0x18;
                val[6] = 0x00;
                val[7] = status;
                val[8] = 0;
 
                ret = ak49xx_run_ram_write(ak49xx, val);
        }
        if (ret == 0) {
                ak4961->ak4961_voice_sync_switch = status;
        }
 
        return ret;
}

static const char *slimbus_interface_reg_text[] = {
        "read", "set","treg0" 
};
 
static const struct soc_enum slimbus_interface_reg_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slimbus_interface_reg_text), slimbus_interface_reg_text);
 


static int ak4961_get_slimbus_interface_reg(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4961->ak4961_slimbus_interface_reg;
 
        return 0;
}

static int ak4961_set_slimbus_interface_reg(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
        struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
        u8   status = ucontrol->value.integer.value[0];
        int   ret = 0;
	int cnt =0;
 
        pr_info("%s: status = %d\n", __func__, status);
 	ak4961->ak4961_slimbus_interface_reg = status;
	switch(status)
		{
	case 0:
		{
 	ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT0_ARRAY);
	printk("[LHS] %s 0x00 ret=%x\n",__func__,ret);

	 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT1_ARRAY);
	printk("[LHS] %s 0x01 ret=%x\n",__func__,ret);

	 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT2_ARRAY);
	printk("[LHS] %s 0x02 ret=%x\n",__func__,ret);

	 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT3_ARRAY);
	printk("[LHS] %s 0x03 ret=%x\n",__func__,ret);

	 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT4_ARRAY);
	printk("[LHS] %s 0x04 ret=%x\n",__func__,ret);

	 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT5_ARRAY);
	printk("[LHS] %s 0x05 ret=%x\n",__func__,ret);

		  	ret=ak49xx_interface_reg_read(ak49xx,0x100);
	printk("[LHS] %s 0x100 ret=%x\n",__func__,ret);
	 ret=ak49xx_interface_reg_read(ak49xx,0x101);
			printk("[LHS] %s 0x101 ret=%x\n",__func__,ret);
		}
		break;
	case 1:
		{
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY, 0x0A);
		ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT0_ARRAY);
			printk("[LHS] %s 0x00 ret=%x\n",__func__,ret);
		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT1_ARRAY, 0x0B);
		 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT1_ARRAY);
			printk("[LHS] %s 0x01 ret=%x\n",__func__,ret);
		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT2_ARRAY, 0x0C);
		 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT2_ARRAY);
			printk("[LHS] %s 0x02 ret=%x\n",__func__,ret);
		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT3_ARRAY, 0x0D);
		 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT3_ARRAY);
			printk("[LHS] %s 0x03 ret=%x\n",__func__,ret);
		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT4_ARRAY, 0x0E);
		 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT4_ARRAY);
			printk("[LHS] %s 0x04 ret=%x\n",__func__,ret);
		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT5_ARRAY, 0x0F);
		 ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT5_ARRAY);
			printk("[LHS] %s 0x05 ret=%x\n",__func__,ret);
		
		}
		break;
	case 2:
		{
		ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT0_ARRAY);
		printk("[LHS] sliminter reg 0x00 %x cnt =%d\n",ret,cnt);

		
		ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY, 0x0A);
			do{
				ret=ak49xx_interface_reg_read(ak49xx,AK496X_SLIM_PGD_PORT0_ARRAY);
				printk("[LHS] sliminter reg 0x00 %x cnt =%d\n",ret,cnt);
				msleep(1000);
				cnt++;
				}while((ret==0x0A)&&(cnt<50));
		}
		break;
	default:
		{
	/*	ret=ak49xx_interface_reg_read(ak49xx,0x100);
		printk("[LHS] %s 0x100 ret=%x\n",__func__,ret);
		ret=ak49xx_interface_reg_read(ak49xx,0x101);
		printk("[LHS] %s 0x101 ret=%x\n",__func__,ret);*/
		break;
		}
		}
        return ret;
		}



static const char *ak4961_power_test_text[] = {
        "ldo-on", "ldo-off","reset-on","reset-off"
};
 
static const struct soc_enum ak4961_power_test_enum =
                SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ak4961_power_test_text), ak4961_power_test_text);
 


static int ak4961_get_power_test(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
		{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
 
        ucontrol->value.enumerated.item[0] = ak4961->ak4961_power_test;
 
        return 0;
		}

static int ak4961_set_power_test(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
		{
        struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
        struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
        struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
        u8   status = ucontrol->value.integer.value[0];
		
		ak4961->ak4961_power_test = status;
		pr_err("[LHS] %s %d status=%d",__func__,__LINE__,status);
			switch (status) {
				case 0:
					ak49xx_power_test(ak49xx,0);
					break;
				case 1:
					ak49xx_power_test(ak49xx,1);
					break;
				case 2:
					ak49xx_power_test(ak49xx,2);
					break;
				case 3:
					ak49xx_power_test(ak49xx,3);
					break;
				default: 
					break;
		}

		return 0; 


}



static const struct snd_kcontrol_new ak4961_snd_controls[] = {

	SOC_ENUM("MIC1 Power Level", mic_1_power_level),

	SOC_ENUM("MIC2 Power Level", mic_2_power_level),

	SOC_SINGLE_TLV("MIC Gain 1L", MIC_AMP_1_LCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 1R", MIC_AMP_1_RCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 2L", MIC_AMP_2_LCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 2R", MIC_AMP_2_RCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("DOut1 Lch Volume", LCH_OUTPUT_VOLUME_1, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE_TLV("DOut1 Rch Volume", RCH_OUTPUT_VOLUME_1, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE_TLV("DOut2 Lch Volume", LCH_OUTPUT_VOLUME_2, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE_TLV("DOut2 Rch Volume", RCH_OUTPUT_VOLUME_2, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE("DOut1 Volume Independent", OUTPUT_VOLUME_SETTING, 0, 1, 0),

	SOC_SINGLE("DOut2 Volume Independent", OUTPUT_VOLUME_SETTING, 1, 1, 0),

	SOC_SINGLE_TLV("HP Out Volume", HP_VOLUME_CONTROL, 0, 0x18, 0, hp_out_tlv),
	
	SOC_ENUM("HP Zero-Cross Timeout", hp_zero_cross_timeout),

	SOC_SINGLE_TLV("Lineout1 Volume", LINEOUT1_VOLUME_CONTROL, 0, 0x07, 0, lineout1_tlv),

	SOC_SINGLE_TLV("Lineout2 Volume", LINEOUT2_VOLUME_CONTROL, 4, 0x07, 0, lineout2_tlv),

	SOC_SINGLE_TLV("Receiver Volume", LINEOUT2_VOLUME_CONTROL, 0, 0x07, 0, receiver_tlv),

	SOC_ENUM("ADC1 HPF Enable", ad1_hpf_enable),

	SOC_ENUM("ADC2 HPF Enable", ad2_hpf_enable),

	SOC_ENUM("ADC1 HPF cut-off", ad1_hp_cf),

	SOC_ENUM("ADC2 HPF cut-off", ad2_hp_cf),

	SOC_ENUM("ADC1 Digital Filter", ad1_digital_filter),

	SOC_ENUM("ADC2 Digital Filter", ad2_digital_filter),

	SOC_ENUM("DAC1 Digital Filter", da1_digital_filter),
	
	SOC_ENUM("DAC1 Digital Filter Through", da1_filter_through),
	
	SOC_ENUM("DAC2 Digital Filter Through", da2_filter_through),

	SOC_ENUM("DAC1 Lch Mixer PGA", da1_lch_mix_pga),

	SOC_ENUM("DAC1 Rch Mixer PGA", da1_rch_mix_pga),

	SOC_ENUM("DAC2 Lch Mixer PGA", da2_lch_mix_pga),

	SOC_ENUM("DAC2 Rch Mixer PGA", da2_rch_mix_pga),
	
	SOC_ENUM("CP1 Clock Mode", cp1_clock_sel),

	SOC_SINGLE_TLV("MIXERA Input1 GAIN", MIXER_A_CONTROL, 4, 2, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERA Input2 GAIN", MIXER_A_CONTROL, 6, 2, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input1 GAIN", MIXER_B_CONTROL, 4, 2, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input2 GAIN", MIXER_B_CONTROL, 6, 2, 0, mixer_gain_tlv),

	SOC_ENUM("MIXERA Input1 SWAP", mixer_a1_swap),

	SOC_ENUM("MIXERA Input2 SWAP", mixer_a2_swap),

	SOC_ENUM("MIXERB Input1 SWAP", mixer_b1_swap),

	SOC_ENUM("MIXERB Input2 SWAP", mixer_b2_swap),

	SOC_ENUM("SRCAO Sync Domain", srcao_sync_domain),

	SOC_ENUM("SRCBO Sync Domain", srcbo_sync_domain),

	SOC_ENUM("SRCCO Sync Domain", srcco_sync_domain),

	SOC_ENUM("SRCDO Sync Domain", srcdo_sync_domain),

	SOC_ENUM("DSP Sync Domain", dsp_sync_domain),

	SOC_ENUM("DSPO2 Sync Domain", dspo2_sync_domain),

	SOC_ENUM("DSPO4 Sync Domain", dspo4_sync_domain),

	SOC_ENUM("CODEC Sync Domain", cdc_sync_domain),

	SOC_ENUM("PORT2 Sync Domain", port2_sync_domain),

	SOC_ENUM("PORT3 Sync Domain", port3_sync_domain),

	SOC_ENUM("PORT4 Sync Domain", port4_sync_domain),

	SOC_ENUM_EXT("DSP Downlink Status", dsp_downlink_status_enum,
			ak4961_get_dsp_downlink_status, ak4961_set_dsp_downlink_status),

	SOC_ENUM_EXT("DSP Uplink Status", dsp_uplink_status_enum,
			ak4961_get_dsp_uplink_status, ak4961_set_dsp_uplink_status),

	SOC_ENUM_EXT("DSP Mode", dsp_mode_enum[0], ak4961_get_dsp_mode, ak4961_set_dsp_mode),

	SOC_SINGLE("DSP CLK Adjust", DSP_SETTING5, 0, 255, 0),

	SOC_SINGLE_EXT_TLV("Internal RX Out Volume", SND_SOC_NOPM, 0, 6, 0,
			akm_internal_rx_gain_get, akm_internal_rx_gain_set, rx_out_gain_tlv),

	/*add by shengguanghui for karaoke 20150606 begin*/
	SOC_SINGLE_EXT_TLV("HP Feedback MIC Volume", SND_SOC_NOPM, 0, 20, 0,
			akm_karaoke_mic_gain_get, akm_karaoke_mic_gain_set, karaoke_mic_gain_tlv),
	/*add by shengguanghui for karaoke 20150606 end*/

	SOC_SINGLE_EXT_TLV("Internal MIC Out Volume", SND_SOC_NOPM, 0, 20, 0,
			akm_internal_mic_gain_get, akm_internal_mic_gain_set, mic_out_gain_tlv),

	SOC_SINGLE_EXT_TLV("Internal AHPF Sensitivity", SND_SOC_NOPM, 0, 13, 0,
			akm_internal_ahpf_sensitivity_get, akm_internal_ahpf_sensitivity_set,
			ahpf_sensitivity_tlv),

	SOC_ENUM_EXT("Bargein Lch Sel", bargein_sel_text_enum,
			ak4961_get_lch_bargein_sel, ak4961_set_lch_bargein_sel),

	SOC_ENUM_EXT("Bargein Rch Sel", bargein_sel_text_enum,
			ak4961_get_rch_bargein_sel, ak4961_set_rch_bargein_sel),

        SOC_ENUM_EXT("Voice Sync Switch", voice_sync_switch_enum,
                        ak4961_get_voice_sync_switch, ak4961_set_voice_sync_switch),
/**lhs test**/
	SOC_ENUM_EXT("Slimbus interface reg", slimbus_interface_reg_enum,
					ak4961_get_slimbus_interface_reg, ak4961_set_slimbus_interface_reg),

	SOC_ENUM_EXT("AK4961 power test", ak4961_power_test_enum,
					ak4961_get_power_test, ak4961_set_power_test),
/**end**/

	 SOC_ENUM_EXT("Voice Band Switch", voice_band_switch_enum,
                        ak4961_bandswitch_get, ak4961_bandswitch_set),

};

/* virtual port entries */
static int slim_tx_mixer_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);
	struct soc_multi_mixer_control *mixer =
		((struct soc_multi_mixer_control *)kcontrol->private_value);
	u32 port_id = mixer->shift;

	ucontrol->value.integer.value[0] = (ak4961_p->tx_port_value & 1 << port_id);
	return 0;
}

static int slim_tx_mixer_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *core = dev_get_drvdata(codec->dev->parent);
	struct snd_soc_dapm_update *update = NULL;
	struct soc_multi_mixer_control *mixer =
		((struct soc_multi_mixer_control *)kcontrol->private_value);
	u32 dai_id = widget->shift;
	u32 port_id = mixer->shift;
	u32 enable = ucontrol->value.integer.value[0];
	u32 vtable = vport_check_table[dai_id];

	pr_debug("%s: wname %s cname %s value %u shift %d item %ld\n", __func__,
		widget->name, ucontrol->id.name, ak4961_p->tx_port_value, widget->shift,
		ucontrol->value.integer.value[0]);

	mutex_lock(&codec->mutex);
	//for slim_spi 2014-11-17 星期一 14:37:44
	if ((ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS) && (ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)) 
		{
			if (dai_id != SB1_CAP) {
				dev_err(codec->dev, "%s: invalid AIF for I2S mode\n",
					__func__);
				mutex_unlock(&codec->mutex);
				return -EINVAL;
			}
		}
//	printk("[lhs] %s 1\n",__func__);
	switch (dai_id) {
	case SB1_CAP:
	case SB2_CAP:
	case SB3_CAP:
		/* only add to the list if value not set
		 */
		if (enable && !(ak4961_p->tx_port_value & 1 << port_id)) {
		//for slim_spi 2014-11-17 星期一 14:38:47
			if (ak4961_p->intf_type ==AK49XX_INTERFACE_TYPE_SLIMBUS ||ak4961_p->intf_type ==AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
				vtable = vport_check_table[dai_id];
			if (ak4961_p->intf_type == AK49XX_INTERFACE_TYPE_I2C ||
					ak4961_p->intf_type == AK49XX_INTERFACE_TYPE_SPI)
				vtable = vport_i2s_check_table[dai_id];

			if (ak49xx_tx_vport_validation(
						vtable,
						port_id,
						ak4961_p->dai, NUM_CODEC_DAIS)) {
				dev_dbg(codec->dev, "%s: TX%u is used by other virtual port\n",
					__func__, port_id + 1);
				mutex_unlock(&codec->mutex);
				return 0;
			}
			ak4961_p->tx_port_value |= 1 << port_id;
			list_add_tail(&core->tx_chs[port_id].list,
				      &ak4961_p->dai[dai_id].ak49xx_ch_list
				      );
		} else if (!enable && (ak4961_p->tx_port_value & 1 << port_id)) {
			ak4961_p->tx_port_value &= ~(1 << port_id);
			list_del_init(&core->tx_chs[port_id].list);
		} else {
			if (enable)
				dev_dbg(codec->dev, "%s: TX%u port is used by this virtual port\n",
					__func__, port_id + 1);
			else
				dev_dbg(codec->dev, "%s: TX%u port is not used by this virtual port\n",
					__func__, port_id + 1);
			/* avoid update power function */
			mutex_unlock(&codec->mutex);
			return 0;
		}
		break;
	default:
		pr_err("Unknown AIF %d\n", dai_id);
		mutex_unlock(&codec->mutex);
		return -EINVAL;
	}
	pr_debug("%s: name %s sname %s updated value %u shift %d\n", __func__,
		widget->name, widget->sname, ak4961_p->tx_port_value, widget->shift);

	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, enable, update);

	return 0;
}

static int slim_rx_mux_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4961_p->rx_port_value;
	return 0;
}

static const char *const slim_rx_mux_text[] = {
	"ZERO", "SB1_PB", "SB2_PB", "SB3_PB"
};

static int slim_rx_mux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *core = dev_get_drvdata(codec->dev->parent);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = widget->shift;

	pr_debug("%s: wname %s cname %s value %u shift %d item %u\n", __func__,
		widget->name, ucontrol->id.name, ak4961_p->rx_port_value, widget->shift,
		ucontrol->value.enumerated.item[0]);

	ak4961_p->rx_port_value = ucontrol->value.enumerated.item[0];

	mutex_lock(&codec->mutex);
	//for slim_spi 2014-11-17 星期一 14:40:46
	if (ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS&&
			ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		if (ak4961_p->rx_port_value > 2) {
			dev_err(codec->dev, "%s: invalid AIF for I2S mode\n",
				__func__);
			goto err;
		}
	}
	/* value need to match the Virtual port and AIF number
	 */
	switch (ak4961_p->rx_port_value) {
	case 0:
		list_del_init(&core->rx_chs[port_id].list);
	break;
	case 1:
		if (ak49xx_rx_vport_validation(port_id +
			AK4961_RX_PORT_START_NUMBER,
			&ak4961_p->dai[SB1_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4961_p->dai[SB1_PB].ak49xx_ch_list);
	break;
	case 2:
		if (ak49xx_rx_vport_validation(port_id +
			AK4961_RX_PORT_START_NUMBER,
			&ak4961_p->dai[SB2_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4961_p->dai[SB2_PB].ak49xx_ch_list);
	break;
	case 3:
		if (ak49xx_rx_vport_validation(port_id +
			AK4961_RX_PORT_START_NUMBER,
			&ak4961_p->dai[SB3_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4961_p->dai[SB3_PB].ak49xx_ch_list);
	break;
	default:
		pr_err("Unknown AIF %d\n", ak4961_p->rx_port_value);
		goto err;
	}
rtn:
	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol, ak4961_p->rx_port_value, e, update);
	return 0;
err:
	mutex_unlock(&codec->mutex);
	return -EINVAL;
}

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[AK4961_RX_MAX] = {
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX3 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX4 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX5 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX6 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static const struct snd_kcontrol_new slim_cap1_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4961_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4961_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4961_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4961_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4961_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4961_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4961_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4961_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4961_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4961_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new slim_cap2_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4961_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4961_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4961_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4961_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4961_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4961_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4961_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4961_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4961_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4961_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new slim_cap3_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4961_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4961_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4961_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4961_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4961_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4961_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4961_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4961_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4961_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4961_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static int ak4961_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct ak49xx *core;
	struct snd_soc_codec *codec = w->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	struct ak49xx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	pr_debug("%s: event called! codec name %s num_dai %d\n"
		"stream name %s event %d\n",
		__func__, w->codec->component.name, w->codec->component.num_dai, w->sname, event);

	/* Execute the callback only if interface type is slimbus */
	//for slim_spi 2014-11-17 星期一 14:42:08
	if (ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
			ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return 0;

	dai = &ak4961_p->dai[w->shift];
	pr_debug("%s: w->name %s w->shift %d event %d\n",
		 __func__, w->name, w->shift, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ak49xx_slimbus_interfacereg_setup(core);
		
		ret = ak49xx_cfg_slim_sch_rx(core, &dai->ak49xx_ch_list,
					      dai->rate, dai->bit_width,
					      &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = ak49xx_close_slim_sch_rx(core, &dai->ak49xx_ch_list,
						dai->grph);
		if (ret < 0) {
			ret = ak49xx_disconnect_port(core,
						      &dai->ak49xx_ch_list,
						      dai->grph);
			pr_debug("%s: Disconnect RX port, ret = %d\n",
				 __func__, ret);
		}
		break;
	}
	return ret;
}

static int ak4961_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct ak49xx *core;
	struct snd_soc_codec *codec = w->codec;
	struct ak4961_priv *ak4961_p = snd_soc_codec_get_drvdata(codec);
	u32  ret = 0;
	struct ak49xx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	pr_debug("%s: event called! codec name %s num_dai %d stream name %s\n",
		__func__, w->codec->component.name, w->codec->component.num_dai, w->sname);

	/* Execute the callback only if interface type is slimbus */
	//for slim_spi 2014-11-17 星期一 14:42:52
	if (ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS&&
			ak4961_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return 0;

	pr_debug("[LHS]%s(): w->name %s event %d w->shift %d\n",
		__func__, w->name, event, w->shift);

	dai = &ak4961_p->dai[w->shift];
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ak49xx_slimbus_interfacereg_setup(core);
		
		ret = ak49xx_cfg_slim_sch_tx(core, &dai->ak49xx_ch_list,
						dai->rate, dai->bit_width,
					    &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = ak49xx_close_slim_sch_tx(core, &dai->ak49xx_ch_list,
						dai->grph);
		if (ret < 0) {
			ret = ak49xx_disconnect_port(core,
						&dai->ak49xx_ch_list,
						dai->grph);
			pr_debug("%s: Disconnect RX port ret = %d\n",
				__func__, ret);
		}
		break;
	}
	return ret;
}

/* DAC1 Lch Digital Mixing Setting */
static const struct snd_kcontrol_new dac1_lch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC1_MONO_MIXING, 0, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC1_MONO_MIXING, 1, 1, 0),
};

/* DAC1 Rch Digital Mixing Setting */
static const struct snd_kcontrol_new dac1_rch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC1_MONO_MIXING, 4, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC1_MONO_MIXING, 5, 1, 0),
};

/* DAC2 Lch Digital Mixing Setting */
static const struct snd_kcontrol_new dac2_lch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC2_MONO_MIXING, 0, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC2_MONO_MIXING, 1, 1, 0),
};

/* DAC2 Rch Digital Mixing Setting */
static const struct snd_kcontrol_new dac2_rch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC2_MONO_MIXING, 4, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC2_MONO_MIXING, 5, 1, 0),
};

/* Mic-Amp Input Signal Select */
static const char *mic_input_select_text[] = {
	"AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "ZERO"
};

static const struct soc_enum mic1L_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_1, 0, 7, mic_input_select_text);

static const struct soc_enum mic1R_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_1, 4, 7, mic_input_select_text);

static const struct soc_enum mic2L_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_2, 0, 7, mic_input_select_text);

static const struct soc_enum mic2R_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_2, 4, 7, mic_input_select_text);

static const struct snd_kcontrol_new mic1L_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-1L input Selector Mux", mic1L_input_select_enum);

static const struct snd_kcontrol_new mic1R_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-1R input Selector Mux", mic1R_input_select_enum);
 
static const struct snd_kcontrol_new mic2L_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-2L input Selector Mux", mic2L_input_select_enum);

static const struct snd_kcontrol_new mic2R_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-2R input Selector Mux", mic2R_input_select_enum);


static int ak4961_codec_enable_dac(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u8 dac_setting;
	unsigned int dac;
	int ret;

	ret = kstrtouint(strpbrk(w->name, "12"), 10, &dac);
	if (ret < 0) {
		pr_err("%s: Invalid AMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	dac_setting = (1 << ((dac - 1) << 1));

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (share_wait_time) {
			share_wait_time = 0;
		} else {
			usleep_range(6500, 6500);	// wait for charge pump
		}
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0xC0, 0xC0);
		usleep_range(1000, 1000);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_8, dac_setting, dac_setting);
		dac_status += dac;
		break;
	case SND_SOC_DAPM_POST_PMD:
		dac_status -= dac;
		if (dac_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0xC0, 0x00);
		}
		snd_soc_update_bits(codec, POWER_MANAGEMENT_8, dac_setting, 0x00);
		share_wait_time = 0;
		break;
	}
	return 0;
}

static int ak4961_codec_enable_hp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x02);
		usleep_range(4500, 4500);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);
		usleep_range(21000, 21000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout1l(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x10, 0x10);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x10, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout1r(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x20, 0x20);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x20, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout2l(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x01);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x00);
		break;
	}
	return 0;
}
/*
static int ak4961_codec_enable_lout2ld(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x03, 0x03);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x03, 0x00);
		break;
	}
	return 0;
}
*/
static int ak4961_codec_enable_lout2ldp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x01);
//		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout2ldn(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x02, 0x02);
//		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x02, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout2r(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x10);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x00);
		break;
	}
	return 0;
}
/*
static int ak4961_codec_enable_lout2rd(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x30, 0x30);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x30, 0x00);
		break;
	}
	return 0;
}
*/
static int ak4961_codec_enable_lout2rdp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x10);
//		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_lout2rdn(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x20, 0x20);
//		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x20, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_rcv(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x02);
		usleep_range(4500, 4500);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x80, 0x80);
		usleep_range(15000, 15000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x80, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_micbias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
//	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(14000, 14000);	// 48kHz wait time
		break;
	}
	return 0;
}

static int ak4961_codec_enable_pmsw(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int reg_srcab,reg_srccd;

// ZTE_chenjun:for third-party call
	if (ak4961_dsp_mode != DSP_MODE_OFF) {
		pr_err("%s: event(%d), dsp(%d): return\n", __func__, event, ak4961_dsp_mode);
		return 0;
	}
//

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		reg_srcab=snd_soc_read(codec,SYNC_DOMAIN_SELECTOR7);
		reg_srccd=snd_soc_read(codec,SYNC_DOMAIN_SELECTOR8);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x02, 0x02);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, reg_srcab);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, reg_srccd);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x02, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_enable_charge_pump_1(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(6500, 6500);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x10, 0x10);
		usleep_range(300, 300);
		if (snd_soc_read(codec, POWER_MANAGEMENT_3) & 0x04) {
			share_wait_time = 1;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x10, 0x00);
		break;
	}
	return 0;
}

static const char *mic_select_text[] = {"Analog", "Digital"};

static const struct soc_enum mic1l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_1, 0, 2, mic_select_text);

static const struct soc_enum mic2l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_2, 0, 2, mic_select_text);

static const struct soc_enum mic1r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mic_select_text);

static const struct soc_enum mic2r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mic_select_text);

static const struct snd_kcontrol_new mic1l_select_kctrl =
	SOC_DAPM_ENUM("MIC1L Selector Mux", mic1l_select_enum);

static const struct snd_kcontrol_new mic2l_select_kctrl =
	SOC_DAPM_ENUM("MIC2L Selector Mux", mic2l_select_enum);

static const struct snd_kcontrol_new mic1r_select_kctrl =
	SOC_DAPM_ENUM("MIC1R Selector Mux", mic1r_select_enum);

static const struct snd_kcontrol_new mic2r_select_kctrl =
	SOC_DAPM_ENUM("MIC2R Selector Mux", mic2r_select_enum);

static const char *pmmp1_cp1_switch_text[] = {"Off", "On"};

static const struct soc_enum pmmp1_cp1_switch_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, pmmp1_cp1_switch_text);

static const struct snd_kcontrol_new pmmp1_cp1_switch_kctrl =
	SOC_DAPM_ENUM("PMMP1 CP1 Switch Mux", pmmp1_cp1_switch_num);

static const char *ain_pmmp_select_text[] =
	{"NONE", "MPWR1A", "MPWR1B", "MPWR1C", "MPWR2"};

static const struct soc_enum ain1_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum ain2_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum ain3_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum ain4_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum ain5_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum ain6_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum dmic1l_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum dmic1r_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum dmic2l_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct soc_enum dmic2r_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 5, ain_pmmp_select_text);

static const struct snd_kcontrol_new ain1_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN1 PMMP Selector Mux", ain1_pmmp_select_num);

static const struct snd_kcontrol_new ain2_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN2 PMMP Selector Mux", ain2_pmmp_select_num);

static const struct snd_kcontrol_new ain3_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN3 PMMP Selector Mux", ain3_pmmp_select_num);

static const struct snd_kcontrol_new ain4_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN4 PMMP Selector Mux", ain4_pmmp_select_num);

static const struct snd_kcontrol_new ain5_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN5 PMMP Selector Mux", ain5_pmmp_select_num);

static const struct snd_kcontrol_new ain6_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN6 PMMP Selector Mux", ain6_pmmp_select_num);

static const struct snd_kcontrol_new dmic1l_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC1L PMMP Selector Mux", dmic1l_pmmp_select_num);

static const struct snd_kcontrol_new dmic1r_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC1R PMMP Selector Mux", dmic1r_pmmp_select_num);

static const struct snd_kcontrol_new dmic2l_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC2L PMMP Selector Mux", dmic2l_pmmp_select_num);

static const struct snd_kcontrol_new dmic2r_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC2R PMMP Selector Mux", dmic2r_pmmp_select_num);

static const char *smart_pa_init_switch_text[] = {"Off", "On"};

static const struct soc_enum smart_pa_init_switch_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, smart_pa_init_switch_text);

static const struct snd_kcontrol_new smart_pa_init_switch_kctrl =
	SOC_DAPM_ENUM("smart pa init Switch Mux", smart_pa_init_switch_num);

static int ak4961_codec_enable_dmic(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u8 dmic_ctl_reg, dmic_setting;
	unsigned int dmic;
	int ret;

	ret = kstrtouint(strpbrk(w->name, "12"), 10, &dmic);
	if (ret < 0) {
		pr_err("%s: Invalid DMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	dmic_ctl_reg = DIGITAL_MIC_1 + (dmic - 1);

	switch (dmic) {
	case 1:
		dmic_setting = 0x30 | AK4961_DMIC1_CLK_ENABLE | AK4961_DMIC1_CHANEL_LRP;
		break;
	case 2:
		dmic_setting = 0x30 | AK4961_DMIC2_CLK_ENABLE | AK4961_DMIC2_CHANEL_LRP;
		break;

	default:
		pr_err("%s: Invalid DMIC Selection\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, dmic_ctl_reg, 0x3A, dmic_setting);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, dmic_ctl_reg, 0x3A, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_pll_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x01); // PMPLL On
		usleep_range(2000, 2000);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_2, 0x10, 0x10); // PMAIF On
		break;

	case SND_SOC_DAPM_PRE_PMD:
#if 0
// chenjun:can't record while playback
		if (snd_soc_read(codec, SYNC_DOMAIN_SELECTOR5) & 0x07) {
			snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07, 0x00);
			usleep_range(16000, 16000);
		}
#endif
		snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x00);     // DSPRSTN Off
		snd_soc_update_bits(codec, POWER_MANAGEMENT_2, 0x10, 0x00); // PMAIF Off
		snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x00); // PMPLL Off
		break;
	}
	return 0;
}

static int ak4961_codec_srca_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
              snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x03, 0x01);
              snd_soc_write(codec, SRC_CLK_SETTING, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x10, 0x10);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x11, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_srcb_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
             snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x03, 0x01);
             snd_soc_write(codec, SRC_CLK_SETTING, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x20, 0x20);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x02, 0x02);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x22, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_srcc_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
            snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x03, 0x01);
            snd_soc_write(codec, SRC_CLK_SETTING, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x40, 0x40);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x04, 0x04);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x44, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_srcd_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
            snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x03, 0x01);
            snd_soc_write(codec, SRC_CLK_SETTING, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x80, 0x80);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x08, 0x08);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x88, 0x00);
		break;
	}
	return 0;
}

static int ak4961_codec_vad_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, VAD_SETTING_1, 0x03, 0x03);
		snd_soc_update_bits(codec, VAD_SETTING_8, 0xC0, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, VAD_SETTING_1, 0x03, 0x00);
		snd_soc_update_bits(codec, VAD_SETTING_8, 0xC0, 0x00);
		break;
	}
	return 0;
}

/* Audio Sink Port: Source Select */
static const char *audio_sink_source_select_text[] = {
	"ZERO", "SDTI1A", "SDTI1B", "SDTI1C", "SDTI1D", "SDTI2", "SDTI3", "SDTI4", "reserved",
	"ADC1", "ADC2", "SBI1",	"SBI2", "SBI3", "DSPO1", "DSPO2", "DSPO3", "DSPO4",
	"DSPO5", "MIXAO", "MIXBO", "SRCAO", "SRCBO", "SRCCO", "SRCDO", "VADO"
};

static const struct soc_enum dac1_source_select_enum =
	SOC_ENUM_SINGLE(DAC1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum dac2_source_select_enum =
	SOC_ENUM_SINGLE(DAC2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new dac1_source_select_kctrl =
	SOC_DAPM_ENUM("DAC1 Source Selector Mux", dac1_source_select_enum);

static const struct snd_kcontrol_new dac2_source_select_kctrl =
	SOC_DAPM_ENUM("DAC2 Source Selector Mux", dac2_source_select_enum);


static const struct soc_enum mixai1_source_select_enum =
	SOC_ENUM_SINGLE(MIXAI1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum mixai2_source_select_enum =
	SOC_ENUM_SINGLE(MIXAI2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixai1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXAI1 Source Selector Mux", mixai1_source_select_enum);

static const struct snd_kcontrol_new mixai2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXAI2 Source Selector Mux", mixai2_source_select_enum);

static const struct soc_enum mixbi1_source_select_enum =
	SOC_ENUM_SINGLE(MIXBI1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum mixbi2_source_select_enum =
	SOC_ENUM_SINGLE(MIXBI2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixbi1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXBI1 Source Selector Mux", mixbi1_source_select_enum);

static const struct snd_kcontrol_new mixbi2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXBI2 Source Selector Mux", mixbi2_source_select_enum);

static const struct soc_enum srcai_source_select_enum =
	SOC_ENUM_SINGLE(SRCAI_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum srcbi_source_select_enum =
	SOC_ENUM_SINGLE(SRCBI_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum srcci_source_select_enum =
	SOC_ENUM_SINGLE(SRCCI_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum srcdi_source_select_enum =
	SOC_ENUM_SINGLE(SRCDI_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new srcai_source_select_kctrl =
	SOC_DAPM_ENUM("SRCAI Source Selector Mux", srcai_source_select_enum);

static const struct snd_kcontrol_new srcbi_source_select_kctrl =
	SOC_DAPM_ENUM("SRCBI Source Selector Mux", srcbi_source_select_enum);

static const struct snd_kcontrol_new srcci_source_select_kctrl =
	SOC_DAPM_ENUM("SRCCI Source Selector Mux", srcci_source_select_enum);

static const struct snd_kcontrol_new srcdi_source_select_kctrl =
	SOC_DAPM_ENUM("SRCDI Source Selector Mux", srcdi_source_select_enum);

static const struct soc_enum dspi1_source_select_enum =
	SOC_ENUM_SINGLE(DSPI1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum dspi2_source_select_enum =
	SOC_ENUM_SINGLE(DSPI2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum dspi3_source_select_enum =
	SOC_ENUM_SINGLE(DSPI3_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum dspi4_source_select_enum =
	SOC_ENUM_SINGLE(DSPI4_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum dspi5_source_select_enum =
	SOC_ENUM_SINGLE(DSPI5_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new dspi1_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI1 Source Selector Mux", dspi1_source_select_enum);

static const struct snd_kcontrol_new dspi2_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI2 Source Selector Mux", dspi2_source_select_enum);

static const struct snd_kcontrol_new dspi3_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI3 Source Selector Mux", dspi3_source_select_enum);

static const struct snd_kcontrol_new dspi4_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI4 Source Selector Mux", dspi4_source_select_enum);

static const struct snd_kcontrol_new dspi5_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI5 Source Selector Mux", dspi5_source_select_enum);

static const struct soc_enum vad_source_select_enum =
	SOC_ENUM_SINGLE(VAD_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new vad_source_select_kctrl =
	SOC_DAPM_ENUM("VAD Source Selector Mux", vad_source_select_enum);

static const struct soc_enum sbo1l_source_select_enum =
	SOC_ENUM_SINGLE(SBO1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sbo1r_source_select_enum =
	SOC_ENUM_SINGLE(SBO1_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sbo2l_source_select_enum =
	SOC_ENUM_SINGLE(SBO2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sbo2r_source_select_enum =
	SOC_ENUM_SINGLE(SBO2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new sbo1l_source_select_kctrl =
	SOC_DAPM_ENUM("SBO1L Source Selector Mux", sbo1l_source_select_enum);

static const struct snd_kcontrol_new sbo1r_source_select_kctrl =
	SOC_DAPM_ENUM("SBO1R Source Selector Mux", sbo1r_source_select_enum);

static const struct snd_kcontrol_new sbo2l_source_select_kctrl =
	SOC_DAPM_ENUM("SBO2L Source Selector Mux", sbo2l_source_select_enum);

static const struct snd_kcontrol_new sbo2r_source_select_kctrl =
	SOC_DAPM_ENUM("SBO2R Source Selector Mux", sbo2r_source_select_enum);

static const char *dac_select_text[] = {"Off", "On"};

static const struct soc_enum hp_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout1l_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);
	
static const struct soc_enum lout1r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2l_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2ld_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2rd_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);
	
static const struct soc_enum rcv_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct snd_kcontrol_new hp_select_kctrl =
	SOC_DAPM_ENUM("HP Virt Switch Mux", hp_select_enum);

static const struct snd_kcontrol_new lout1l_select_kctrl =
	SOC_DAPM_ENUM("LOUT1L Virt Switch Mux", lout1l_select_enum);
	
static const struct snd_kcontrol_new lout1r_select_kctrl =
	SOC_DAPM_ENUM("LOUT1R Virt Switch Mux", lout1r_select_enum);

static const struct snd_kcontrol_new lout2l_select_kctrl =
	SOC_DAPM_ENUM("LOUT2L Virt Switch Mux", lout2l_select_enum);

static const struct snd_kcontrol_new lout2ld_select_kctrl =
	SOC_DAPM_ENUM("LOUT2LD Virt Switch Mux", lout2ld_select_enum);

static const struct snd_kcontrol_new lout2r_select_kctrl =
	SOC_DAPM_ENUM("LOUT2R Virt Switch Mux", lout2r_select_enum);

static const struct snd_kcontrol_new lout2rd_select_kctrl =
	SOC_DAPM_ENUM("LOUT2RD Virt Switch Mux", lout2rd_select_enum);

static const struct snd_kcontrol_new rcv_select_kctrl =
	SOC_DAPM_ENUM("RCV Virt Switch Mux", rcv_select_enum);

static const char *mixer_select_text[] = {"Off", "On"};

static const struct soc_enum mixera_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct soc_enum mixerb_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct snd_kcontrol_new mixera_select_kctrl =
	SOC_DAPM_ENUM("MIXAO Virt Switch Mux", mixera_select_enum);

static const struct snd_kcontrol_new mixerb_select_kctrl =
	SOC_DAPM_ENUM("MIXBO Virt Switch Mux", mixerb_select_enum);

static const struct snd_kcontrol_new dspo1_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI1_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI1_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI1_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI1_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI1_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo2_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI2_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI2_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI2_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI2_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI2_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo3_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI3_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI3_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI3_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI3_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI3_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo4_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI4_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI4_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI4_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI4_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI4_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo5_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI5_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI5_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI5_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI5_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI5_VIRT_MIXER, 4, 1, 0),
};


/* Todo: Have seperate dapm widgets for I2S and Slimbus.
 */
static const struct snd_soc_dapm_widget ak4961_dapm_widgets[] = {

	SND_SOC_DAPM_AIF_IN_E("SB1 PB", "SB1 Playback", 0, SND_SOC_NOPM, SB1_PB,
				0, ak4961_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SB2 PB", "SB2 Playback", 0, SND_SOC_NOPM, SB2_PB,
				0, ak4961_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SB3 PB", "SB3 Playback", 0, SND_SOC_NOPM, SB3_PB,
				0, ak4961_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, AK4961_RX1, 0,
				&slim_rx_mux[AK4961_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, AK4961_RX2, 0,
				&slim_rx_mux[AK4961_RX2]),
	SND_SOC_DAPM_MUX("SLIM RX3 MUX", SND_SOC_NOPM, AK4961_RX3, 0,
				&slim_rx_mux[AK4961_RX3]),
	SND_SOC_DAPM_MUX("SLIM RX4 MUX", SND_SOC_NOPM, AK4961_RX4, 0,
				&slim_rx_mux[AK4961_RX4]),
	SND_SOC_DAPM_MUX("SLIM RX5 MUX", SND_SOC_NOPM, AK4961_RX5, 0,
				&slim_rx_mux[AK4961_RX5]),
	SND_SOC_DAPM_MUX("SLIM RX6 MUX", SND_SOC_NOPM, AK4961_RX6, 0,
				&slim_rx_mux[AK4961_RX6]),

	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX4", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX5", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX6", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_AIF_OUT_E("SB1 CAP", "SB1 Capture", 0, SND_SOC_NOPM, SB1_CAP,
				0, ak4961_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SB2 CAP", "SB2 Capture", 0, SND_SOC_NOPM, SB2_CAP,
				0, ak4961_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SB3 CAP", "SB3 Capture", 0, SND_SOC_NOPM, SB3_CAP,
				0, ak4961_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("SB1_CAP Mixer", SND_SOC_NOPM, SB1_CAP, 0,
		slim_cap1_mixer, ARRAY_SIZE(slim_cap1_mixer)),

	SND_SOC_DAPM_MIXER("SB2_CAP Mixer", SND_SOC_NOPM, SB2_CAP, 0,
		slim_cap2_mixer, ARRAY_SIZE(slim_cap2_mixer)),

	SND_SOC_DAPM_MIXER("SB3_CAP Mixer", SND_SOC_NOPM, SB3_CAP, 0,
		slim_cap3_mixer, ARRAY_SIZE(slim_cap3_mixer)),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_INPUT("DMIC1"),

	SND_SOC_DAPM_ADC_E("ADC DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		ak4961_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("MIC1L Selector", SND_SOC_NOPM, 0, 0, &mic1l_select_kctrl),

	SND_SOC_DAPM_MUX("MIC1R Selector", SND_SOC_NOPM, 0, 0, &mic1r_select_kctrl),

	SND_SOC_DAPM_INPUT("DMIC2"),

	SND_SOC_DAPM_ADC_E("ADC DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		ak4961_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("MIC2L Selector", SND_SOC_NOPM, 0, 0, &mic2l_select_kctrl),

	SND_SOC_DAPM_MUX("MIC2R Selector", SND_SOC_NOPM, 0, 0, &mic2r_select_kctrl),

	/* Smart PA */
	SND_SOC_DAPM_INPUT("Smart PA Input"),

	SND_SOC_DAPM_OUTPUT("Smart PA Output"),

	SND_SOC_DAPM_MUX("Smart PA Init Switch", SND_SOC_NOPM, 0, 0, &smart_pa_init_switch_kctrl),

	/* MIC Power */
	SND_SOC_DAPM_INPUT("MRF1"),

	SND_SOC_DAPM_INPUT("MRF2"),
	
	SND_SOC_DAPM_MUX("PMMP1 CP1 Switch", SND_SOC_NOPM, 0, 0, &pmmp1_cp1_switch_kctrl),

	SND_SOC_DAPM_MICBIAS_E("PMMP1A", POWER_MANAGEMENT_4, 0, 0,
		ak4961_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP1B", POWER_MANAGEMENT_4, 1, 0,
		ak4961_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP1C", POWER_MANAGEMENT_4, 2, 0,
		ak4961_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP2", POWER_MANAGEMENT_4, 3, 0,
		ak4961_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MUX("AIN1 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain1_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN2 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain2_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN3 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain3_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN4 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain4_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN5 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain5_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN6 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain6_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1L PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic1l_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1R PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic1r_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2L PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic2l_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2R PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic2r_pmmp_select_kctrl),

	/* Analog Mic Inputs */
	SND_SOC_DAPM_INPUT("AIN1"),

	SND_SOC_DAPM_INPUT("AIN2"),

	SND_SOC_DAPM_INPUT("AIN3"),

	SND_SOC_DAPM_INPUT("AIN4"),

	SND_SOC_DAPM_INPUT("AIN5"),

	SND_SOC_DAPM_INPUT("AIN6"),

	SND_SOC_DAPM_PGA("PMAIN1", POWER_MANAGEMENT_5, 0, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PMAIN2", POWER_MANAGEMENT_5, 1, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PMAIN3", POWER_MANAGEMENT_5, 2, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PMAIN4", POWER_MANAGEMENT_5, 3, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PMAIN5", POWER_MANAGEMENT_5, 4, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PMAIN6", POWER_MANAGEMENT_5, 5, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Mic_1L input Selector", SND_SOC_NOPM, 0, 0,
		&mic1L_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_1R input Selector", SND_SOC_NOPM, 0, 0,
		&mic1R_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_2L input Selector", SND_SOC_NOPM, 0, 0,
		&mic2L_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_2R input Selector", SND_SOC_NOPM, 0, 0,
		&mic2R_input_select_kctrl),

	/* ADC stuff */
	SND_SOC_DAPM_ADC("ADC_1L", NULL, POWER_MANAGEMENT_6, 0, 0),

	SND_SOC_DAPM_ADC("ADC_1R", NULL, POWER_MANAGEMENT_6, 1, 0),

	SND_SOC_DAPM_ADC("ADC_2L", NULL, POWER_MANAGEMENT_6, 2, 0),

	SND_SOC_DAPM_ADC("ADC_2R", NULL, POWER_MANAGEMENT_6, 3, 0),

	/* DAC stuff */
	SND_SOC_DAPM_MIXER("DAC1 Lch Mixer", SND_SOC_NOPM, 0, 0,
		dac1_lch_mixer_kctrl, ARRAY_SIZE(dac1_lch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC1 Rch Mixer", SND_SOC_NOPM, 0, 0,
		dac1_rch_mixer_kctrl, ARRAY_SIZE(dac1_rch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC2 Lch Mixer", SND_SOC_NOPM, 0, 0,
		dac2_lch_mixer_kctrl, ARRAY_SIZE(dac2_lch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC2 Rch Mixer", SND_SOC_NOPM, 0, 0,
		dac2_rch_mixer_kctrl, ARRAY_SIZE(dac2_rch_mixer_kctrl)),

	SND_SOC_DAPM_MUX("HP Virt Switch", SND_SOC_NOPM, 0, 0, &hp_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT1L Virt Switch", SND_SOC_NOPM, 0, 0, &lout1l_select_kctrl),
	
	SND_SOC_DAPM_MUX("LOUT1R Virt Switch", SND_SOC_NOPM, 0, 0, &lout1r_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT2L Virt Switch", SND_SOC_NOPM, 0, 0, &lout2l_select_kctrl),
	
	SND_SOC_DAPM_MUX("LOUT2LD Virt Switch", SND_SOC_NOPM, 0, 0, &lout2ld_select_kctrl),
	
	SND_SOC_DAPM_MUX("LOUT2R Virt Switch", SND_SOC_NOPM, 0, 0, &lout2r_select_kctrl),
	
	SND_SOC_DAPM_MUX("LOUT2RD Virt Switch", SND_SOC_NOPM, 0, 0, &lout2rd_select_kctrl),

	SND_SOC_DAPM_MUX("RCV Virt Switch", SND_SOC_NOPM, 0, 0, &rcv_select_kctrl),

	SND_SOC_DAPM_DAC_E("DAC1", NULL, SND_SOC_NOPM, 0, 0,
		ak4961_codec_enable_dac, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("DAC2", NULL, SND_SOC_NOPM, 0, 0,
		ak4961_codec_enable_dac, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* HP outputs */
	SND_SOC_DAPM_HP("HP", ak4961_codec_enable_hp),

	SND_SOC_DAPM_HP("LOUT1L", ak4961_codec_enable_lout1l),

	SND_SOC_DAPM_HP("LOUT1R", ak4961_codec_enable_lout1r),

	SND_SOC_DAPM_HP("LOUT2L", ak4961_codec_enable_lout2l),

//	SND_SOC_DAPM_HP("LOUT2LD", ak4961_codec_enable_lout2ld),

	SND_SOC_DAPM_HP("LINEOUT1", ak4961_codec_enable_lout2ldp),

	SND_SOC_DAPM_HP("LINEOUT3", ak4961_codec_enable_lout2ldn),

	SND_SOC_DAPM_HP("LOUT2R", ak4961_codec_enable_lout2r),

//	SND_SOC_DAPM_HP("LOUT2RD", ak4961_codec_enable_lout2rd),

	SND_SOC_DAPM_HP("LINEOUT2", ak4961_codec_enable_lout2rdp),

	SND_SOC_DAPM_HP("LINEOUT4", ak4961_codec_enable_lout2rdn),

	SND_SOC_DAPM_HP("RCV", ak4961_codec_enable_rcv),

	/* Power Supply */
	SND_SOC_DAPM_SUPPLY("PMSW", SND_SOC_NOPM, 0, 0,
		ak4961_codec_enable_pmsw, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY_S("Charge Pump1", 1, POWER_MANAGEMENT_3, 0, 0,
		ak4961_codec_enable_charge_pump_1, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY_S("Charge Pump3", 1, POWER_MANAGEMENT_3, 2, 0, NULL, 0),

	/* PLL stuff */
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL CLK", SND_SOC_NOPM, 0, 0, ak4961_codec_pll_setup,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* MIXER stuff */
	SND_SOC_DAPM_MUX("MIXAO Virt Switch", SND_SOC_NOPM, 0, 0, &mixera_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBO Virt Switch", SND_SOC_NOPM, 0, 0, &mixerb_select_kctrl),

	/* SRC stuff */
	SND_SOC_DAPM_PGA_E("SRCA", POWER_MANAGEMENT_2, 0, 0, NULL, 0,
		ak4961_codec_srca_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCB", POWER_MANAGEMENT_2, 1, 0, NULL, 0,
		ak4961_codec_srcb_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCC", POWER_MANAGEMENT_2, 2, 0, NULL, 0,
		ak4961_codec_srcc_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCD", POWER_MANAGEMENT_2, 3, 0, NULL, 0,
		ak4961_codec_srcd_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* DSP stuff */
	SND_SOC_DAPM_MIXER("DSPO1 Mixer", SND_SOC_NOPM, 0, 0,
		dspo1_mixer_kctrl, ARRAY_SIZE(dspo1_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO2 Mixer", SND_SOC_NOPM, 0, 0,
		dspo2_mixer_kctrl, ARRAY_SIZE(dspo2_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO3 Mixer", SND_SOC_NOPM, 0, 0,
		dspo3_mixer_kctrl, ARRAY_SIZE(dspo3_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO4 Mixer", SND_SOC_NOPM, 0, 0,
		dspo4_mixer_kctrl, ARRAY_SIZE(dspo4_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO5 Mixer", SND_SOC_NOPM, 0, 0,
		dspo5_mixer_kctrl, ARRAY_SIZE(dspo5_mixer_kctrl)),

	/* VAD stuff */
	SND_SOC_DAPM_PGA_E("VAD", VAD_SETTING_1, 3, 0, NULL, 0,
		ak4961_codec_vad_setup, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Digital Path Staff */
	SND_SOC_DAPM_MUX("SBO1L Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo1l_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO1R Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo1r_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO2L Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo2l_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO2R Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo2r_source_select_kctrl),

	SND_SOC_DAPM_MUX("DAC1 Source Selector", SND_SOC_NOPM, 0, 0,
		&dac1_source_select_kctrl),

	SND_SOC_DAPM_MUX("DAC2 Source Selector", SND_SOC_NOPM, 0, 0,
		&dac2_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi1_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi2_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI3 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi3_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI4 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi4_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI5 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi5_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXAI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixai1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXAI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixai2_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixbi1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixbi2_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCAI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcai_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCBI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcbi_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCCI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcci_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCDI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcdi_source_select_kctrl),
		
	SND_SOC_DAPM_MUX("VAD Source Selector", SND_SOC_NOPM, 0, 0,
		&vad_source_select_kctrl),
};

static const struct snd_soc_dapm_route audio_map[] = {

	// DMIC path
	{"ADC DMIC1", NULL, "DMIC1"},
//	{"ADC DMIC1", NULL, "Charge Pump1"},
	{"MIC1L Selector", "Digital", "ADC DMIC1"},
	{"MIC1R Selector", "Digital", "ADC DMIC1"},

	{"ADC DMIC2", NULL, "DMIC2"},
//	{"ADC DMIC2", NULL, "Charge Pump1"},
	{"MIC2L Selector", "Digital", "ADC DMIC2"},
	{"MIC2R Selector", "Digital", "ADC DMIC2"},

	// MIC power path
	{"PMMP2", NULL, "Charge Pump1"},
	{"PMMP1 CP1 Switch", "On", "Charge Pump1"},
	{"PMMP1A", NULL, "PMMP1 CP1 Switch"},
	{"PMMP1B", NULL, "PMMP1 CP1 Switch"},
	{"PMMP1C", NULL, "PMMP1 CP1 Switch"},
	
	{"PMMP1A", NULL, "MRF1"},
	{"PMMP1B", NULL, "MRF1"},
	{"PMMP1C", NULL, "MRF1"},
	{"PMMP2", NULL, "MRF2"},

	{"DMIC1L PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC1L PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC1L PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC1L PMMP Selector", "MPWR2", "PMMP2"},

	{"DMIC1R PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC1R PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC1R PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC1R PMMP Selector", "MPWR2", "PMMP2"},

	{"DMIC2L PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC2L PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC2L PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC2L PMMP Selector", "MPWR2", "PMMP2"},

	{"DMIC2R PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC2R PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC2R PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC2R PMMP Selector", "MPWR2", "PMMP2"},

	{"ADC DMIC1", NULL, "DMIC1L PMMP Selector"},
	{"ADC DMIC1", NULL, "DMIC1R PMMP Selector"},
	{"ADC DMIC2", NULL, "DMIC2L PMMP Selector"},
	{"ADC DMIC2", NULL, "DMIC2R PMMP Selector"},

	{"AIN1 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN1 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN1 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN1 PMMP Selector", "MPWR2", "PMMP2"},

	{"AIN2 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN2 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN2 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN2 PMMP Selector", "MPWR2", "PMMP2"},

	{"AIN3 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN3 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN3 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN3 PMMP Selector", "MPWR2", "PMMP2"},

	{"AIN4 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN4 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN4 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN4 PMMP Selector", "MPWR2", "PMMP2"},

	{"AIN5 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN5 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN5 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN5 PMMP Selector", "MPWR2", "PMMP2"},

	{"AIN6 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN6 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN6 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN6 PMMP Selector", "MPWR2", "PMMP2"},

	{"PMAIN1", NULL, "AIN1 PMMP Selector"},
	{"PMAIN2", NULL, "AIN2 PMMP Selector"},
	{"PMAIN3", NULL, "AIN3 PMMP Selector"},
	{"PMAIN4", NULL, "AIN4 PMMP Selector"},
	{"PMAIN5", NULL, "AIN5 PMMP Selector"},
	{"PMAIN6", NULL, "AIN6 PMMP Selector"},

	// AMIC path
	{"PMAIN1", NULL, "AIN1"},
	{"PMAIN2", NULL, "AIN2"},
	{"PMAIN3", NULL, "AIN3"},
	{"PMAIN4", NULL, "AIN4"},
	{"PMAIN5", NULL, "AIN5"},
	{"PMAIN6", NULL, "AIN6"},

	{"Mic_1L input Selector", "AIN1", "PMAIN1"},
	{"Mic_1L input Selector", "AIN2", "PMAIN2"},
	{"Mic_1L input Selector", "AIN3", "PMAIN3"},
	{"Mic_1L input Selector", "AIN4", "PMAIN4"},
	{"Mic_1L input Selector", "AIN5", "PMAIN5"},
	{"Mic_1L input Selector", "AIN6", "PMAIN6"},

	{"Mic_1R input Selector", "AIN1", "PMAIN1"},
	{"Mic_1R input Selector", "AIN2", "PMAIN2"},
	{"Mic_1R input Selector", "AIN3", "PMAIN3"},
	{"Mic_1R input Selector", "AIN4", "PMAIN4"},
	{"Mic_1R input Selector", "AIN5", "PMAIN5"},
	{"Mic_1R input Selector", "AIN6", "PMAIN6"},

	{"Mic_2L input Selector", "AIN1", "PMAIN1"},
	{"Mic_2L input Selector", "AIN2", "PMAIN2"},
	{"Mic_2L input Selector", "AIN3", "PMAIN3"},
	{"Mic_2L input Selector", "AIN4", "PMAIN4"},
	{"Mic_2L input Selector", "AIN5", "PMAIN5"},
	{"Mic_2L input Selector", "AIN6", "PMAIN6"},

	{"Mic_2R input Selector", "AIN1", "PMAIN1"},
	{"Mic_2R input Selector", "AIN2", "PMAIN2"},
	{"Mic_2R input Selector", "AIN3", "PMAIN3"},
	{"Mic_2R input Selector", "AIN4", "PMAIN4"},
	{"Mic_2R input Selector", "AIN5", "PMAIN5"},
	{"Mic_2R input Selector", "AIN6", "PMAIN6"},

	{"ADC_1L", NULL, "Mic_1L input Selector"},
	{"ADC_1R", NULL, "Mic_1R input Selector"},
	{"ADC_2L", NULL, "Mic_2L input Selector"},
	{"ADC_2R", NULL, "Mic_2R input Selector"},

	{"ADC_1L", NULL, "Charge Pump1"},
	{"ADC_1R", NULL, "Charge Pump1"},
	{"ADC_2L", NULL, "Charge Pump1"},
	{"ADC_2R", NULL, "Charge Pump1"},

	{"MIC1L Selector", "Analog", "ADC_1L"},
	{"MIC1R Selector", "Analog", "ADC_1R"},
	{"MIC2L Selector", "Analog", "ADC_2L"},
	{"MIC2R Selector", "Analog", "ADC_2R"},

	// DAC path
	{"DAC1 Lch Mixer", "Lch_Switch", "DAC1 Source Selector"},
	{"DAC1 Lch Mixer", "Rch_Switch", "DAC1 Source Selector"},
	{"DAC1 Rch Mixer", "Lch_Switch", "DAC1 Source Selector"},
	{"DAC1 Rch Mixer", "Rch_Switch", "DAC1 Source Selector"},
	{"DAC2 Lch Mixer", "Lch_Switch", "DAC2 Source Selector"},
	{"DAC2 Lch Mixer", "Rch_Switch", "DAC2 Source Selector"},
	{"DAC2 Rch Mixer", "Lch_Switch", "DAC2 Source Selector"},
	{"DAC2 Rch Mixer", "Rch_Switch", "DAC2 Source Selector"},

	{"DAC1", NULL, "DAC1 Lch Mixer"},
	{"DAC1", NULL, "DAC1 Rch Mixer"},
	{"DAC2", NULL, "DAC2 Lch Mixer"},
	{"DAC2", NULL, "DAC2 Rch Mixer"},

	{"DAC1", NULL, "Charge Pump3"},
	{"DAC2", NULL, "Charge Pump3"},

	// Output path
	{"HP Virt Switch", "On", "DAC1"},
	{"LOUT1L Virt Switch", "On", "DAC2"},
	{"LOUT1R Virt Switch", "On", "DAC2"},
	{"LOUT2L Virt Switch", "On", "DAC2"},
	{"LOUT2R Virt Switch", "On", "DAC2"},
	{"LOUT2LD Virt Switch", "On", "DAC2"},
	{"LOUT2RD Virt Switch", "On", "DAC2"},
	{"RCV Virt Switch", "On", "DAC2"},
	
	{"HP", NULL, "HP Virt Switch"},
	{"LOUT1L", NULL, "LOUT1L Virt Switch"},
	{"LOUT1R", NULL, "LOUT1R Virt Switch"},
	{"LOUT2L", NULL, "LOUT2L Virt Switch"},
	{"LOUT2R", NULL, "LOUT2R Virt Switch"},
//	{"LOUT2LD", NULL, "LOUT2LD Virt Switch"},
	{"LINEOUT1", NULL, "LOUT2LD Virt Switch"},
	{"LINEOUT3", NULL, "LOUT2LD Virt Switch"},
//	{"LOUT2RD", NULL, "LOUT2RD Virt Switch"},
	{"LINEOUT2", NULL, "LOUT2RD Virt Switch"},
	{"LINEOUT4", NULL, "LOUT2RD Virt Switch"},
	{"RCV", NULL, "RCV Virt Switch"},

	// PLL CLK
	{"PLL CLK", NULL, "RX_BIAS"},

	// SLIM_MUX("SB1_PB", "SB1 PB")
	{"SLIM RX1 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX2 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX3 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX4 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX5 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX6 MUX", "SB1_PB", "SB1 PB"},

	// SLIM_MUX("SB2_PB", "SB2 PB")
	{"SLIM RX1 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX2 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX3 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX4 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX5 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX6 MUX", "SB2_PB", "SB2 PB"},

	// SLIM_MUX("SB3_PB", "SB3 PB")
	{"SLIM RX1 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX2 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX3 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX4 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX5 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX6 MUX", "SB3_PB", "SB3 PB"},

	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},
	{"SLIM RX3", NULL, "SLIM RX3 MUX"},
	{"SLIM RX4", NULL, "SLIM RX4 MUX"},
	{"SLIM RX5", NULL, "SLIM RX5 MUX"},
	{"SLIM RX6", NULL, "SLIM RX6 MUX"},

	{"SB1_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB2_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB3_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB1 CAP", NULL, "SB1_CAP Mixer"},
	{"SB2 CAP", NULL, "SB2_CAP Mixer"},
	{"SB3 CAP", NULL, "SB3_CAP Mixer"},

	{"SB1 PB", NULL, "PLL CLK"},
	{"SB2 PB", NULL, "PLL CLK"},
	{"SB3 PB", NULL, "PLL CLK"},
	{"SB1 CAP", NULL, "PLL CLK"},
	{"SB2 CAP", NULL, "PLL CLK"},
	{"SB3 CAP", NULL, "PLL CLK"},

	// Smart PA Init Path
	{"Smart PA Init Switch", "On", "Smart PA Input"},
	{"Smart PA Output", NULL, "Smart PA Init Switch"},
	{"Smart PA Output", NULL, "PLL CLK"},
	{"Smart PA Output", NULL, "SDTO2 Source Selector"},
	{"Smart PA Output", NULL, "SDTO3 Source Selector"},
	{"Smart PA Output", NULL, "SDTO4 Source Selector"},

	// SRC path
	{"SRCA", NULL, "PMSW"},
	{"SRCB", NULL, "PMSW"},
	{"SRCC", NULL, "PMSW"},
	{"SRCD", NULL, "PMSW"},

	{"SRCA", NULL, "SRCAI Source Selector"},
	{"SRCB", NULL, "SRCBI Source Selector"},
	{"SRCC", NULL, "SRCCI Source Selector"},
	{"SRCD", NULL, "SRCDI Source Selector"},

	// MIXER path
	{"MIXAO Virt Switch", "On", "MIXAI1 Source Selector"},
	{"MIXAO Virt Switch", "On", "MIXAI2 Source Selector"},
	{"MIXBO Virt Switch", "On", "MIXBI1 Source Selector"},
	{"MIXBO Virt Switch", "On", "MIXBI2 Source Selector"},

	// DSP path
	{"DSPO1 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO1 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO1 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO1 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO1 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO2 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO2 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO2 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO2 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO2 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO3 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO3 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO3 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO3 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO3 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO4 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO4 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO4 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO4 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO4 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO5 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO5 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO5 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO5 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO5 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	// VAD path
	{"VAD", NULL, "VAD Source Selector"},
	
	// Digital path
	{"SBO1L Source Selector", "ADC1", "MIC1L Selector"},
	{"SBO1R Source Selector", "ADC1", "MIC1R Selector"},
	{"SBO1L Source Selector", "ADC2", "MIC2L Selector"},
	{"SBO1R Source Selector", "ADC2", "MIC2R Selector"},
	{"SBO1L Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO1R Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO1L Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO1R Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO1L Source Selector", "SRCAO", "SRCA"},
	{"SBO1R Source Selector", "SRCAO", "SRCA"},
	{"SBO1L Source Selector", "SRCBO", "SRCB"},
	{"SBO1R Source Selector", "SRCBO", "SRCB"},
	{"SBO1L Source Selector", "SRCCO", "SRCC"},
	{"SBO1R Source Selector", "SRCCO", "SRCC"},
	{"SBO1L Source Selector", "SRCDO", "SRCD"},
	{"SBO1R Source Selector", "SRCDO", "SRCD"},
	{"SBO1L Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO1R Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO1L Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO1R Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO1L Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO1R Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO1L Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO1R Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO1L Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO1R Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO1L Source Selector", "VADO", "VAD"},
	{"SBO1R Source Selector", "VADO", "VAD"},
	{"SBO1L Source Selector", "SBI1", "SLIM RX1"},
	{"SBO1R Source Selector", "SBI1", "SLIM RX1"},
	{"SBO1L Source Selector", "SBI1", "SLIM RX2"},
	{"SBO1R Source Selector", "SBI1", "SLIM RX2"},
	{"SBO1L Source Selector", "SBI2", "SLIM RX3"},
	{"SBO1R Source Selector", "SBI2", "SLIM RX3"},
	{"SBO1L Source Selector", "SBI2", "SLIM RX4"},
	{"SBO1R Source Selector", "SBI2", "SLIM RX4"},
	{"SBO1L Source Selector", "SBI3", "SLIM RX5"},
	{"SBO1R Source Selector", "SBI3", "SLIM RX5"},
	{"SBO1L Source Selector", "SBI3", "SLIM RX6"},
	{"SBO1R Source Selector", "SBI3", "SLIM RX6"},

	{"SBO2L Source Selector", "ADC1", "MIC1L Selector"},
	{"SBO2R Source Selector", "ADC1", "MIC1R Selector"},
	{"SBO2L Source Selector", "ADC2", "MIC2L Selector"},
	{"SBO2R Source Selector", "ADC2", "MIC2R Selector"},
	{"SBO2L Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO2R Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO2L Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO2R Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO2L Source Selector", "SRCAO", "SRCA"},
	{"SBO2R Source Selector", "SRCAO", "SRCA"},
	{"SBO2L Source Selector", "SRCBO", "SRCB"},
	{"SBO2R Source Selector", "SRCBO", "SRCB"},
	{"SBO2L Source Selector", "SRCCO", "SRCC"},
	{"SBO2R Source Selector", "SRCCO", "SRCC"},
	{"SBO2L Source Selector", "SRCDO", "SRCD"},
	{"SBO2R Source Selector", "SRCDO", "SRCD"},
	{"SBO2L Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO2R Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO2L Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO2R Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO2L Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO2R Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO2L Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO2R Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO2L Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO2R Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO2L Source Selector", "VADO", "VAD"},
	{"SBO2R Source Selector", "VADO", "VAD"},
	{"SBO2L Source Selector", "SBI1", "SLIM RX1"},
	{"SBO2R Source Selector", "SBI1", "SLIM RX1"},
	{"SBO2L Source Selector", "SBI1", "SLIM RX2"},
	{"SBO2R Source Selector", "SBI1", "SLIM RX2"},
	{"SBO2L Source Selector", "SBI2", "SLIM RX3"},
	{"SBO2R Source Selector", "SBI2", "SLIM RX3"},
	{"SBO2L Source Selector", "SBI2", "SLIM RX4"},
	{"SBO2R Source Selector", "SBI2", "SLIM RX4"},
	{"SBO2L Source Selector", "SBI3", "SLIM RX5"},
	{"SBO2R Source Selector", "SBI3", "SLIM RX5"},
	{"SBO2L Source Selector", "SBI3", "SLIM RX6"},
	{"SBO2R Source Selector", "SBI3", "SLIM RX6"},

	{"DAC1 Source Selector", "SBI1", "SLIM RX1"},
	{"DAC1 Source Selector", "SBI1", "SLIM RX2"},
	{"DAC1 Source Selector", "SBI2", "SLIM RX3"},
	{"DAC1 Source Selector", "SBI2", "SLIM RX4"},
	{"DAC1 Source Selector", "SBI3", "SLIM RX5"},
	{"DAC1 Source Selector", "SBI3", "SLIM RX6"},
	{"DAC1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DAC1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DAC1 Source Selector", "SRCAO", "SRCA"},
	{"DAC1 Source Selector", "SRCBO", "SRCB"},
	{"DAC1 Source Selector", "SRCCO", "SRCC"},
	{"DAC1 Source Selector", "SRCDO", "SRCD"},
	{"DAC1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DAC1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DAC1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DAC1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DAC1 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DAC1 Source Selector", "VADO", "VAD"},
	{"DAC1 Source Selector", "ADC1", "MIC1L Selector"},
	{"DAC1 Source Selector", "ADC1", "MIC1R Selector"},
	{"DAC1 Source Selector", "ADC2", "MIC2L Selector"},
	{"DAC1 Source Selector", "ADC2", "MIC2R Selector"},

	{"DAC2 Source Selector", "SBI1", "SLIM RX1"},
	{"DAC2 Source Selector", "SBI1", "SLIM RX2"},
	{"DAC2 Source Selector", "SBI2", "SLIM RX3"},
	{"DAC2 Source Selector", "SBI2", "SLIM RX4"},
	{"DAC2 Source Selector", "SBI3", "SLIM RX5"},
	{"DAC2 Source Selector", "SBI3", "SLIM RX6"},
	{"DAC2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DAC2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DAC2 Source Selector", "SRCAO", "SRCA"},
	{"DAC2 Source Selector", "SRCBO", "SRCB"},
	{"DAC2 Source Selector", "SRCCO", "SRCC"},
	{"DAC2 Source Selector", "SRCDO", "SRCD"},
	{"DAC2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DAC2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DAC2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DAC2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DAC2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DAC2 Source Selector", "VADO", "VAD"},
	{"DAC2 Source Selector", "ADC1", "MIC1L Selector"},
	{"DAC2 Source Selector", "ADC1", "MIC1R Selector"},
	{"DAC2 Source Selector", "ADC2", "MIC2L Selector"},
	{"DAC2 Source Selector", "ADC2", "MIC2R Selector"},

	{"MIXAI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXAI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXAI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXAI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXAI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXAI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXAI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXAI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXAI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXAI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXAI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXAI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXAI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXAI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXAI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXAI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXAI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXAI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXAI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXAI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXAI1 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"MIXAI1 Source Selector", "VADO", "VAD"},

	{"MIXAI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXAI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXAI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXAI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXAI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXAI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXAI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXAI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXAI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXAI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXAI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXAI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXAI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXAI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXAI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXAI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXAI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXAI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXAI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXAI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXAI2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"MIXAI2 Source Selector", "VADO", "VAD"},	

	{"MIXBI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXBI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXBI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXBI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXBI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXBI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXBI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXBI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXBI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXBI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXBI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXBI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXBI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXBI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXBI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXBI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXBI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXBI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXBI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXBI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXBI1 Source Selector", "DSPO5", "DSPO5 Mixer"},	
	{"MIXBI1 Source Selector", "VADO", "VAD"},

	{"MIXBI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXBI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXBI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXBI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXBI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXBI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXBI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXBI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXBI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXBI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXBI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXBI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXBI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXBI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXBI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXBI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXBI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXBI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXBI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXBI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXBI2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"MIXBI2 Source Selector", "VADO", "VAD"},

	{"SRCAI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCAI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCAI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCAI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCAI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCAI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCAI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCAI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCAI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCAI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCAI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCAI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCAI Source Selector", "SRCAO", "SRCA"},
	{"SRCAI Source Selector", "SRCBO", "SRCB"},
	{"SRCAI Source Selector", "SRCCO", "SRCC"},
	{"SRCAI Source Selector", "SRCDO", "SRCD"},
	{"SRCAI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCAI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCAI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCAI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCAI Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SRCAI Source Selector", "VADO", "VAD"},

	{"SRCBI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCBI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCBI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCBI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCBI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCBI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCBI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCBI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCBI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCBI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCBI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCBI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCBI Source Selector", "SRCAO", "SRCA"},
	{"SRCBI Source Selector", "SRCBO", "SRCB"},
	{"SRCBI Source Selector", "SRCCO", "SRCC"},
	{"SRCBI Source Selector", "SRCDO", "SRCD"},
	{"SRCBI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCBI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCBI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCBI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCBI Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SRCBI Source Selector", "VADO", "VAD"},

	{"SRCCI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCCI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCCI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCCI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCCI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCCI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCCI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCCI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCCI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCCI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCCI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCCI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCCI Source Selector", "SRCAO", "SRCA"},
	{"SRCCI Source Selector", "SRCBO", "SRCB"},
	{"SRCCI Source Selector", "SRCCO", "SRCC"},
	{"SRCCI Source Selector", "SRCDO", "SRCD"},
	{"SRCCI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCCI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCCI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCCI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCCI Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SRCCI Source Selector", "VADO", "VAD"},

	{"SRCDI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCDI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCDI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCDI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCDI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCDI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCDI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCDI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCDI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCDI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCDI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCDI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCDI Source Selector", "SRCAO", "SRCA"},
	{"SRCDI Source Selector", "SRCBO", "SRCB"},
	{"SRCDI Source Selector", "SRCCO", "SRCC"},
	{"SRCDI Source Selector", "SRCDO", "SRCD"},
	{"SRCDI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCDI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCDI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCDI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCDI Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SRCDI Source Selector", "VADO", "VAD"},

	{"DSPI1 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI1 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI1 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI1 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI1 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI1 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI1 Source Selector", "SRCAO", "SRCA"},
	{"DSPI1 Source Selector", "SRCBO", "SRCB"},
	{"DSPI1 Source Selector", "SRCCO", "SRCC"},
	{"DSPI1 Source Selector", "SRCDO", "SRCD"},
	{"DSPI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI1 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DSPI1 Source Selector", "VADO", "VAD"},

	{"DSPI2 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI2 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI2 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI2 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI2 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI2 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI2 Source Selector", "SRCAO", "SRCA"},
	{"DSPI2 Source Selector", "SRCBO", "SRCB"},
	{"DSPI2 Source Selector", "SRCCO", "SRCC"},
	{"DSPI2 Source Selector", "SRCDO", "SRCD"},
	{"DSPI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DSPI2 Source Selector", "VADO", "VAD"},

	{"DSPI3 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI3 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI3 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI3 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI3 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI3 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI3 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI3 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI3 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI3 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI3 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI3 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI3 Source Selector", "SRCAO", "SRCA"},
	{"DSPI3 Source Selector", "SRCBO", "SRCB"},
	{"DSPI3 Source Selector", "SRCCO", "SRCC"},
	{"DSPI3 Source Selector", "SRCDO", "SRCD"},
	{"DSPI3 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI3 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI3 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI3 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI3 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DSPI3 Source Selector", "VADO", "VAD"},

	{"DSPI4 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI4 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI4 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI4 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI4 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI4 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI4 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI4 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI4 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI4 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI4 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI4 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI4 Source Selector", "SRCAO", "SRCA"},
	{"DSPI4 Source Selector", "SRCBO", "SRCB"},
	{"DSPI4 Source Selector", "SRCCO", "SRCC"},
	{"DSPI4 Source Selector", "SRCDO", "SRCD"},
	{"DSPI4 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI4 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI4 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI4 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI4 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DSPI4 Source Selector", "VADO", "VAD"},

	{"DSPI5 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI5 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI5 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI5 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI5 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI5 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI5 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI5 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI5 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI5 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI5 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI5 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI5 Source Selector", "SRCAO", "SRCA"},
	{"DSPI5 Source Selector", "SRCBO", "SRCB"},
	{"DSPI5 Source Selector", "SRCCO", "SRCC"},
	{"DSPI5 Source Selector", "SRCDO", "SRCD"},
	{"DSPI5 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI5 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI5 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI5 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI5 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DSPI5 Source Selector", "VADO", "VAD"},

	{"VAD Source Selector", "SBI1", "SLIM RX1"},
	{"VAD Source Selector", "SBI1", "SLIM RX2"},
	{"VAD Source Selector", "SBI2", "SLIM RX3"},
	{"VAD Source Selector", "SBI2", "SLIM RX4"},
	{"VAD Source Selector", "SBI3", "SLIM RX5"},
	{"VAD Source Selector", "SBI3", "SLIM RX6"},
	{"VAD Source Selector", "ADC1", "MIC1L Selector"},
	{"VAD Source Selector", "ADC1", "MIC1R Selector"},
	{"VAD Source Selector", "ADC2", "MIC2L Selector"},
	{"VAD Source Selector", "ADC2", "MIC2R Selector"},
	{"VAD Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"VAD Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"VAD Source Selector", "SRCAO", "SRCA"},
	{"VAD Source Selector", "SRCBO", "SRCB"},
	{"VAD Source Selector", "SRCCO", "SRCC"},
	{"VAD Source Selector", "SRCDO", "SRCD"},
	{"VAD Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"VAD Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"VAD Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"VAD Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"VAD Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"VAD Source Selector", "VADO", "VAD"},
};

static const struct soc_enum sdto1a_source_select_enum =
	SOC_ENUM_SINGLE(SDTO1A_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sdto1b_source_select_enum =
	SOC_ENUM_SINGLE(SDTO1B_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sdto2_source_select_enum =
	SOC_ENUM_SINGLE(SDTO2_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sdto3_source_select_enum =
	SOC_ENUM_SINGLE(SDTO3_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct soc_enum sdto4_source_select_enum =
	SOC_ENUM_SINGLE(SDTO4_SOURCE_SELECTOR, 0, 26, audio_sink_source_select_text);

static const struct snd_kcontrol_new sdto1a_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO1A Source Selector Mux", sdto1a_source_select_enum);

static const struct snd_kcontrol_new sdto1b_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO1B Source Selector Mux", sdto1b_source_select_enum);

static const struct snd_kcontrol_new sdto2_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO2 Source Selector Mux", sdto2_source_select_enum);

static const struct snd_kcontrol_new sdto3_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO3 Source Selector Mux", sdto3_source_select_enum);

static const struct snd_kcontrol_new sdto4_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO4 Source Selector Mux", sdto4_source_select_enum);

static const struct snd_soc_dapm_widget ak4961_dapm_i2s_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIA", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIB", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIC", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTID", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1 SDTOA", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1 SDTOB", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	//SND_SOC_DAPM_AIF_IN("AIF2 SDTI", "AIF2 Playback", 0, SND_SOC_NOPM, 0, 0), // lhs for slimbus voice
	SND_SOC_DAPM_INPUT("AIF2 SDTI"),
	SND_SOC_DAPM_AIF_OUT("AIF2 SDTO", "AIF2 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF3 SDTI", "AIF3 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF3 SDTO", "AIF3 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF4 SDTI", "AIF4 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF4 SDTO", "AIF4 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("SDTO1A Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto1a_source_select_kctrl),
			
	SND_SOC_DAPM_MUX("SDTO1B Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto1b_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO2 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto2_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO3 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto3_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO4 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto4_source_select_kctrl),
};

static const struct snd_soc_dapm_route audio_i2s_map[] = {
	{"AIF1 SDTIA", NULL, "PLL CLK"},
	{"AIF1 SDTIB", NULL, "PLL CLK"},
	{"AIF1 SDTIC", NULL, "PLL CLK"},
	{"AIF1 SDTID", NULL, "PLL CLK"},
	//{"AIF2 SDTI", NULL, "PLL CLK"},
	{"AIF3 SDTI", NULL, "PLL CLK"},
	{"AIF4 SDTI", NULL, "PLL CLK"},
	{"AIF1 SDTOA", NULL, "PLL CLK"},
	{"AIF1 SDTOB", NULL, "PLL CLK"},
	{"AIF2 SDTO", NULL, "PLL CLK"},
	{"AIF3 SDTO", NULL, "PLL CLK"},
	{"AIF4 SDTO", NULL, "PLL CLK"},

	{"AIF1 SDTOA", NULL, "SDTO1A Source Selector"},
	{"AIF1 SDTOB", NULL, "SDTO1B Source Selector"},
	{"AIF2 SDTO", NULL, "SDTO2 Source Selector"},
	{"AIF3 SDTO", NULL, "SDTO3 Source Selector"},
	{"AIF4 SDTO", NULL, "SDTO4 Source Selector"},

	{"SDTO1A Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO1A Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO1A Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO1A Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO1A Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO1A Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO1A Source Selector", "SRCAO", "SRCA"},
	{"SDTO1A Source Selector", "SRCBO", "SRCB"},
	{"SDTO1A Source Selector", "SRCCO", "SRCC"},
	{"SDTO1A Source Selector", "SRCDO", "SRCD"},
	{"SDTO1A Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO1A Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO1A Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO1A Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO1A Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO1A Source Selector", "VADO", "VAD"},
	{"SDTO1A Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO1A Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO1A Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO1A Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO1A Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO1A Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO1A Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO1A Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO1A Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO1B Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO1B Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO1B Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO1B Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO1B Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO1B Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO1B Source Selector", "SRCAO", "SRCA"},
	{"SDTO1B Source Selector", "SRCBO", "SRCB"},
	{"SDTO1B Source Selector", "SRCCO", "SRCC"},
	{"SDTO1B Source Selector", "SRCDO", "SRCD"},
	{"SDTO1B Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO1B Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO1B Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO1B Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO1B Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO1B Source Selector", "VADO", "VAD"},
	{"SDTO1B Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO1B Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO1B Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO1B Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO1B Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO1B Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO1B Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO1B Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO1B Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO2 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO2 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO2 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO2 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO2 Source Selector", "SRCAO", "SRCA"},
	{"SDTO2 Source Selector", "SRCBO", "SRCB"},
	{"SDTO2 Source Selector", "SRCCO", "SRCC"},
	{"SDTO2 Source Selector", "SRCDO", "SRCD"},
	{"SDTO2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO2 Source Selector", "VADO", "VAD"},
	{"SDTO2 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO2 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO2 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO2 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO2 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO2 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO3 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO3 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO3 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO3 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO3 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO3 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO3 Source Selector", "SRCAO", "SRCA"},
	{"SDTO3 Source Selector", "SRCBO", "SRCB"},
	{"SDTO3 Source Selector", "SRCCO", "SRCC"},
	{"SDTO3 Source Selector", "SRCDO", "SRCD"},
	{"SDTO3 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO3 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO3 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO3 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO3 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO3 Source Selector", "VADO", "VAD"},
	{"SDTO3 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO3 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO3 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO3 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO3 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO3 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO3 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO3 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO3 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO3 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO4 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO4 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO4 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO4 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO4 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO4 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO4 Source Selector", "SRCAO", "SRCA"},
	{"SDTO4 Source Selector", "SRCBO", "SRCB"},
	{"SDTO4 Source Selector", "SRCCO", "SRCC"},
	{"SDTO4 Source Selector", "SRCDO", "SRCD"},
	{"SDTO4 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO4 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO4 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO4 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO4 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO4 Source Selector", "VADO", "VAD"},
	{"SDTO4 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO4 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO4 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO4 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO4 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO4 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO4 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO4 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO4 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO4 Source Selector", "SDTI3", "AIF3 SDTI"},

	{"DAC1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DAC1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DAC1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DAC1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DAC1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DAC1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DAC1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DAC2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DAC2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DAC2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DAC2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DAC2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DAC2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DAC2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXAI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXAI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXAI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXAI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXAI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXAI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXAI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXAI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXAI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXAI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXAI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXAI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXAI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXAI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXBI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXBI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXBI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXBI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXBI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXBI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXBI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXBI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXBI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXBI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXBI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXBI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXBI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXBI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCAI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCAI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCAI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCAI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCAI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCAI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCAI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCBI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCBI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCBI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCBI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCBI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCBI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCBI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCCI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCCI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCCI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCCI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCCI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCCI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCCI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCDI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCDI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCDI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCDI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCDI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCDI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCDI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI3 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI3 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI3 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI3 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI3 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI3 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI3 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI4 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI4 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI4 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI4 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI4 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI4 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI4 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI5 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI5 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI5 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI5 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI5 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI5 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI5 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SBO1L Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO1R Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO1L Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO1R Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO1L Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO1R Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO1L Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO1R Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO1L Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO1R Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO1L Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO1R Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO1L Source Selector", "SDTI4", "AIF4 SDTI"},
	{"SBO1R Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SBO2L Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO2R Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO2L Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO2R Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO2L Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO2R Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO2L Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO2R Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO2L Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO2R Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO2L Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO2R Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO2L Source Selector", "SDTI4", "AIF4 SDTI"},
	{"SBO2R Source Selector", "SDTI4", "AIF4 SDTI"},

	{"VAD Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"VAD Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"VAD Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"VAD Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"VAD Source Selector", "SDTI2", "AIF2 SDTI"},
	{"VAD Source Selector", "SDTI3", "AIF3 SDTI"},
	{"VAD Source Selector", "SDTI4", "AIF4 SDTI"},

};

#ifdef CONFIG_SWITCH
static ssize_t headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case BIT_NO_HEADSET:
		return sprintf(buf, "No Device\n");
	case BIT_HEADSET:
		return sprintf(buf, "Headset\n");
	case BIT_HEADSET_NO_MIC:
		return sprintf(buf, "Headset_no_mic\n");
	}
	return -EINVAL;
}
#endif
static int last_report_key;
static int last_report_sw;
static int mic_det_counter;

static irqreturn_t ak4961_jde_irq(int irq, void *data)
{
        struct ak4961_priv *priv = data;
        struct snd_soc_codec *codec = priv->codec;
        int val, val1;
        int report = last_report_sw;
 
        val = snd_soc_read(codec, JACK_DETECTION_STATUS);
        if (val < 0) {
                snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
                dev_err(codec->dev, "Failed to read JACK_DETECTION_STATUS: %d\n",
                        val);
                return IRQ_HANDLED;
        }
 
        if (val & 0x02) {
 
                if (val & 0x80) {
                        mic_det_repeat++;
                        if (mic_det_repeat >= MAX_MIC_DET_RPT) {
#ifdef CONFIG_SWITCH
                                hs_type = report = BIT_HEADSET;
                                input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
                                input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 1);
                                input_sync(priv->mbhc_cfg.btn_idev);
#else
                                hs_type = report = SND_JACK_HEADSET;
#endif
                                snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x48, 0x40);
                                mic_det_repeat = 0;
 
                        } else {
                                val1 = snd_soc_read(codec, DETECTION_POWER_MANAGEMENT);
                                if ((val1 & 0x08) == 0) {
                                        snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x08, 0x08);
                                        mic_det_counter = 0;
                                }
                        }
                        
                } else {
                        mic_det_repeat = 0;
                        val1 = snd_soc_read(codec, DETECTION_POWER_MANAGEMENT);
                        if ((val1 & 0x08) == 0) {
                                snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x08, 0x08);
                                mic_det_counter = 0;
                        } else {
                                if (mic_det_counter == MAX_MIC_DET_TRY) {
#ifdef CONFIG_SWITCH
                                        hs_type = report = BIT_HEADSET_NO_MIC;
                                        input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
                                        input_sync(priv->mbhc_cfg.btn_idev);
#else
                                        hs_type = report = SND_JACK_HEADPHONE;
#endif
                                        snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x08, 0x00);
                                        
                                } else {
                                        if (mic_det_counter == (MAX_MIC_DET_TRY >> 1)) {
                                                if (priv->mbhc_cfg.swap_gnd_mic) {
                                                        priv->mbhc_cfg.swap_gnd_mic(codec);
                                                }
                                        }
                                        mic_det_counter++;
                                }
                        }
                }
        } else {
                snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x48, 0x00);
                hs_type = report = 0;
#ifdef CONFIG_SWITCH
                if (last_report_sw) {
                        input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 0);
                }
                if (last_report_sw == BIT_HEADSET) {
                        input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 0);
                }
                if (last_report_key) {
                        input_report_key(priv->mbhc_cfg.btn_idev, last_report_key, 0);
                        last_report_key = 0;
                }
                input_sync(priv->mbhc_cfg.btn_idev);
#endif
        }
        
        if (report != last_report_sw) {
#ifdef CONFIG_SWITCH
                switch_set_state(priv->mbhc_cfg.h2w_sdev, report);
#else
                snd_soc_jack_report(priv->mbhc_cfg.headset_jack, report,
                                    SND_JACK_HEADSET);
#endif          
                last_report_sw = report;
//ZTE_weizj 20150601 fix:Fake button press while inserting headset
#if 0
	if (report==1) {
		priv->mbhc_cfg.reject_btn = true;
		dev_info(codec->dev, "%s: weizj1 delay btn det %d\n", __func__, priv->mbhc_cfg.reject_btn);
		cancel_delayed_work(&priv->mbhc_cfg.reject_btn_dwork);
		schedule_delayed_work(&priv->mbhc_cfg.reject_btn_dwork, msecs_to_jiffies(2000));
	}
#else
	if (report==1) {
		priv->mbhc_cfg.reject_btn = true;
		dev_info(codec->dev, "%s: weizj2 delay btn det %d\n", __func__, priv->mbhc_cfg.reject_btn);
		cancel_delayed_work(&priv->mbhc_cfg.btn_work);
		queue_delayed_work(priv->mbhc_cfg.btn_workqueue, &priv->mbhc_cfg.btn_work, msecs_to_jiffies(2500));
	}
#endif
//ZTE_weizj 20150601 end
                dev_info(codec->dev, "%s: report %d, mic_det_repeat %d\n", __func__, report, mic_det_repeat);
        }
 
        snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
        return IRQ_HANDLED;
}


static irqreturn_t ak4961_rce_irq(int irq, void *data)
{
	struct ak4961_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int val, mic_level;
	int report;

	val = snd_soc_read(codec, JACK_DETECTION_STATUS);
	if (val < 0) {
		snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
		dev_err(codec->dev, "Failed to read JACK_DETECTION_STATUS: %d\n",
				val);
		return IRQ_HANDLED;
	}

	if (val & 0x01) {
		mic_level = snd_soc_read(codec, MIC_LEVEL_DETECTION);
		if (mic_level < 0) {
			snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
			dev_err(codec->dev, "Failed to read MIC_LEVEL_DETECTION: %d\n",
					mic_level);
			return IRQ_HANDLED;
		}

		if (mic_level < 0x03) { // ZTE_chenjun:orig:0x05
#ifdef CONFIG_SWITCH
			report = KEY_MEDIA;
#else
			report = SND_JACK_BTN_0;
#endif

		} else if (mic_level < 0x08) {
#ifdef CONFIG_SWITCH
			report = KEY_VOLUMEUP;
#else
			report = SND_JACK_BTN_1;
#endif

		} else if (mic_level < 0x0E) {
#ifdef CONFIG_SWITCH
			report = KEY_VOLUMEDOWN;
#else
			report = SND_JACK_BTN_2;
#endif

		} else if (mic_level < 0x0F) {
#ifdef CONFIG_SWITCH
			report = KEY_VOLUMEDOWN; // ZTE_chenjun:orig:0
#else
			report = SND_JACK_BTN_3;
#endif

		} else  {
#ifdef CONFIG_SWITCH
			report = 0;
#else
			report = SND_JACK_BTN_4;
#endif

		}

	} else {

		report = 0;
	}
#ifdef CONFIG_SWITCH
	if (report != 0 && last_report_key == 0) {
		if (report == KEY_VOLUMEUP) {
			input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
		}
		if (report == KEY_VOLUMEDOWN) {
			input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
		}

//ZTE_weizj 20150601 fix:Fake button press while inserting headset
#if 1
		if (report==226 && priv->mbhc_cfg.reject_btn) {
			pr_err("weizj:do not report btn det reject(%d)\n", priv->mbhc_cfg.reject_btn);
			//snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);

			//return IRQ_HANDLED;
		} else
#endif
		{
//ZTE_weizj 20150601 end
		input_report_key(priv->mbhc_cfg.btn_idev, report, 1);
		input_sync(priv->mbhc_cfg.btn_idev);
		last_report_key = report;
		dev_info(codec->dev, "%s: mic_level %d report %d\n", __func__, mic_level, report);
		}//weizj end

	} else if (report == 0 && last_report_key != 0) {
		if (last_report_key == KEY_VOLUMEUP) {
			input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
		}
		if (last_report_key == KEY_VOLUMEDOWN) {
			input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
		}

		input_report_key(priv->mbhc_cfg.btn_idev, last_report_key, 0);
		input_sync(priv->mbhc_cfg.btn_idev);
		last_report_key = report;
		dev_info(codec->dev, "%s: mic_level %d report %d\n", __func__, mic_level, report);
	}
#else
	snd_soc_jack_report(priv->mbhc_cfg.button_jack, report,
			AK4961_JACK_BUTTON_MASK);
	dev_info(codec->dev, "%s: report %d\n", __func__, report);
#endif

	snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);

	// add for rce test 
	val = snd_soc_read(codec, JACK_DETECTION_STATUS);
	if (val < 0) {
		dev_err(codec->dev, "Failed to read JACK_DETECTION_STATUS: %d\n",
				val);
		return IRQ_HANDLED;
	}

	if((val & 0x02)==0)
		{
		snd_soc_update_bits(codec, DETECTION_POWER_MANAGEMENT, 0x48, 0x00);
		report = 0;
#ifdef CONFIG_SWITCH
		if (last_report_sw) {
			input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 0);
		}
		if (last_report_sw == BIT_HEADSET) {
			input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 0);
		}
		if (last_report_key) {
			input_report_key(priv->mbhc_cfg.btn_idev, last_report_key, 0);
			last_report_key = 0;
		}
		input_sync(priv->mbhc_cfg.btn_idev);
#endif
		if (report != last_report_sw) {
			#ifdef CONFIG_SWITCH
				switch_set_state(priv->mbhc_cfg.h2w_sdev, report);
			#else
				snd_soc_jack_report(priv->mbhc_cfg.headset_jack, report,
						    SND_JACK_HEADSET);
			#endif		
				last_report_sw = report;
				dev_info(codec->dev, "%s: report %d\n", __func__, report);
			}	
	}
	

	return IRQ_HANDLED;
}

static irqreturn_t ak4961_vad_irq(int irq, void *data)
{
	struct ak4961_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int val;

	val = snd_soc_read(codec, JACK_DETECTION_STATUS);
	if (val < 0) {
		snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
		dev_err(codec->dev, "Failed to read JACK_DETECTION_STATUS: %d\n",
			val);
		return IRQ_HANDLED;
	}

	if (val & 0x04) {
		dev_info(codec->dev, "%s: report active\n", __func__);
	}

	snd_soc_write(codec, DETECTION_EVENT_RESET, 0x01);
	return IRQ_HANDLED;
}

// ZTE_weizj 20150601 fix:Fake button press while inserting headset
#if 0
static void ak4961_reject_btn_fn(struct work_struct *work)
{
       struct delayed_work *dwork;
       struct ak4961_mbhc_config *mbhc;

       dwork = to_delayed_work(work);
       mbhc = container_of(dwork, struct ak4961_mbhc_config, reject_btn_dwork);
       mbhc->reject_btn = false;

       pr_err("weizj1:release btn det reject_btn(%d) \n", mbhc->reject_btn);
}
#else
static void ak4961_reject_btn_fn(struct work_struct *work)
{
       struct delayed_work *dwork;
       struct ak4961_mbhc_config *mbhc;

       dwork = to_delayed_work(work);
       mbhc = container_of(dwork, struct ak4961_mbhc_config, btn_work);
       mbhc->reject_btn = false;

       pr_err("weizj2:release btn det reject_btn(%d) \n", mbhc->reject_btn);
}
#endif
// ZTE_weizj 20150601 end

int ak4961_hs_detect(struct snd_soc_codec *codec,
		    const struct ak4961_mbhc_config *cfg)
{
	struct ak4961_priv *ak4961;
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	struct ak49xx_core_resource *core_res = &ak49xx->core_res;
	int rc = 0;

	if (!codec) {
		pr_err("Error: no codec\n");
		return -EINVAL;
	}
	// printk("[lhs] %s in\n",__func__);
	//snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x11, 0x11); //for akm bias
	//snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x01, 0x01); //for akm bias
	switch (cfg->mclk_rate) {
	case AK4961_MCLK_RATE_12288KHZ:
		snd_soc_update_bits(codec, PLL1_SOURCE_SELECTOR, 0x1F, 0x04);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_L8, 0xFF, 0x03);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_L8, 0xFF, 0x27);
		pr_debug("MCLK: clock rate using %dHz\n", cfg->mclk_rate);
		break;
	case AK4961_MCLK_RATE_9600KHZ:
		snd_soc_update_bits(codec, PLL1_SOURCE_SELECTOR, 0x1F, 0x04);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_L8, 0xFF, 0x04);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_L8, 0xFF, 0x3F);
		pr_debug("MCLK: clock rate using %dHz\n", cfg->mclk_rate);
		break;
	default:
		pr_err("Error: unsupported clock rate %d\n", cfg->mclk_rate);
		return -EINVAL;
		break;
	}

	ak4961 = snd_soc_codec_get_drvdata(codec);
	ak4961->mbhc_cfg = *cfg;

// ZTE_weizj 20150601 fix:Fake button press while inserting headset
#if 0
	ak4961->mbhc_cfg.reject_btn = false;
	INIT_DELAYED_WORK(&ak4961->mbhc_cfg.reject_btn_dwork, ak4961_reject_btn_fn);
#else
	ak4961->mbhc_cfg.reject_btn = false;
	ak4961->mbhc_cfg.btn_workqueue = create_singlethread_workqueue("ak4961_mbhc");
	if (ak4961->mbhc_cfg.btn_workqueue == NULL) {
		pr_err("weizj2: can't create work queue \n");
	}
	INIT_DELAYED_WORK(&ak4961->mbhc_cfg.btn_work, ak4961_reject_btn_fn);
#endif
//ZTE_weizj 20150601 end
#ifdef CONFIG_SWITCH
	ak4961->mbhc_cfg.h2w_sdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	ak4961->mbhc_cfg.h2w_sdev->name = "h2w";
	ak4961->mbhc_cfg.h2w_sdev->print_name = headset_print_name;
	rc = switch_dev_register(ak4961->mbhc_cfg.h2w_sdev);
	if (rc < 0) {
		pr_err("%d: Error in switch_dev_register\n", rc);
		goto error_switch_register;
	}

	ak4961->mbhc_cfg.btn_idev = input_allocate_device();
	memcpy(ak4961->mbhc_cfg.keycode, ak4961_keycode,
			sizeof(ak4961->mbhc_cfg.keycode));

	ak4961->mbhc_cfg.btn_idev->name = "hs_detect";
	ak4961->mbhc_cfg.btn_idev->id.vendor = 0x0001;
	ak4961->mbhc_cfg.btn_idev->id.product = 1;
	ak4961->mbhc_cfg.btn_idev->id.version = 1;
	ak4961->mbhc_cfg.btn_idev->keycode = ak4961->mbhc_cfg.keycode;
	ak4961->mbhc_cfg.btn_idev->keycodesize = sizeof(unsigned short);
	ak4961->mbhc_cfg.btn_idev->keycodemax = ARRAY_SIZE(ak4961->mbhc_cfg.keycode);

	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_KEY, KEY_MEDIA);
	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN);
	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ak4961->mbhc_cfg.btn_idev, EV_SW, SW_MICROPHONE_INSERT);

	rc = input_register_device(ak4961->mbhc_cfg.btn_idev);
	if (rc != 0) {
		pr_err("%d: Error in input_register_device\n", rc);
		goto error_input_register;
	} else {
		goto request_virq;
	}

error_input_register:
	if (ak4961->mbhc_cfg.h2w_sdev != NULL) {
		switch_dev_unregister(ak4961->mbhc_cfg.h2w_sdev);
		kfree(ak4961->mbhc_cfg.h2w_sdev);
		ak4961->mbhc_cfg.h2w_sdev = NULL;
	}
error_switch_register:	
	return rc;
#endif

request_virq:
	rc = ak49xx_request_irq(core_res, AK4961_IRQ_JDE,
		ak4961_jde_irq, "Headset detect", ak4961);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4961_IRQ_JDE);
		goto err_jde_irq;
	}

	rc = ak49xx_request_irq(core_res, AK4961_IRQ_RCE,
		ak4961_rce_irq, "Button detect", ak4961);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4961_IRQ_RCE);
		goto err_rce_irq;
	}

	rc = ak49xx_request_irq(core_res, AK4961_IRQ_VAD,
		ak4961_vad_irq, "Voice active detect", ak4961);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4961_IRQ_VAD);
		goto err_vad_irq;
	}

	return rc;

err_vad_irq:
	ak49xx_free_irq(core_res, AK4961_IRQ_RCE, ak4961);

err_rce_irq:
	ak49xx_free_irq(core_res, AK4961_IRQ_JDE, ak4961);

err_jde_irq:
	return rc;
}
EXPORT_SYMBOL_GPL(ak4961_hs_detect);

static const struct ak4961_reg_mask_val ak4961s_reg_defaults[] = {

	/* Ak4961s changes */
//	AK4961_REG_VAL(AK4961s_BiQuad1_A1, 0x24),
};

static void ak4961_update_reg_defaults(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ak4961s_reg_defaults); i++)
		snd_soc_write(codec, ak4961s_reg_defaults[i].reg,
				ak4961s_reg_defaults[i].val);
}

static const struct ak4961_reg_mask_val ak4961_codec_reg_init_val[] = {
	//{POWER_MANAGEMENT_3, 0x01, 0x01}, //for akm bias
	/*for mic_biasA*/
	//{POWER_MANAGEMENT_4, 0x01, 0x01},//for akm bias
	/* set Sync Domain 1 source: Tie Low */
	{MSYNC1_MSN_CKS, 0x1F, 0x00},

	/* set Sync Domain 2 source: PLLCLK1 */
	// chenjun:{MSYNC2_MSN_CKS, 0x1F, 0x01},
	{MSYNC2_MSN_CKS, 0x3F, 0x21}, /* chenjun add for TFA9890 */
	{MSYNC2_BDV, 0xFF, 0x27},
	{MSYNC2_SDV, 0xFF, 0x3F},

	// ZTE_chenjun
        /* analog input start-up time */
	{MODE_CONTROL, 0x03, 0x02},
	//

	/* set Sync Domain 3 source: PLLCLK1 */
	{MSYNC3_MSN_CKS, 0x1F, 0x01},

	/* set Sync Domain 4 source: PLLCLK1 */
	//{MSYNC4_MSN_CKS, 0x1F, 0x01},
	{MSYNC4_MSN_CKS, 0x3F, 0x21}, /* ZTE_chenjun: PORT4 as I2S master */
	{MSYNC4_BDV, 0xFF, 0x27},
	{MSYNC4_SDV, 0xFF, 0x3F},

	/* set Sync Domain 5 source: PLLCLK1 -> fs24kHz, bclk=64fs */
	{MSYNC5_MSN_CKS, 0x1F, 0x01},
	{MSYNC5_BDV, 0xFF, 0x4F},
	{MSYNC5_SDV, 0xFF, 0x3F},

	/* set Sync Domain 6 source: PLLCLK1 -> fs16kHz, bclk=64fs */
	{MSYNC6_MSN_CKS, 0x1F, 0x01},
	{MSYNC6_BDV, 0xFF, 0x77},
	{MSYNC6_SDV, 0xFF, 0x3F},

	/* set Sync Domain 7 source: PLLCLK1 */
	{MSYNC7_MSN_CKS, 0x1F, 0x01},
	{MSYNC7_BDV, 0xFF, 0x27},
	{MSYNC7_SDV, 0xFF, 0x3F},

	/* set CODEC Clock Source -> PLLCLK1 */
	{CDCMCLK_SOURCE_SELECTOR, 0x1F, 0x01},

	/* set DSP Clock Source -> PLLCLK1 */
	{DSPMCLK_SOURCE_SELECTOR, 0x1F, 0x01},
	/* set Bus Clock -> 24.576MHz (Max Sync 192kHz) */
//	{BUSMCLK_DIVIDER, 0xFF, 0x03},

	/* set AIF1/2 Sync Domain to select SYNC1 */
	{SYNC_DOMAIN_SELECTOR1, 0x77, 0x11},

	/* set AIF3/4 Sync Domain to select SYNC1 */
	{SYNC_DOMAIN_SELECTOR2, 0x77, 0x11},

	/* set CODEC Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR3, 0x07, 0x07},

	/* set DSP Sync Domain to select SYNC5 */
	{SYNC_DOMAIN_SELECTOR5, 0x07, 0x05},

	/* set Soft SRC Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR6, 0x77, 0x77},

	/* set SRC A/B Sync Domain to select SYNC5 */
	{SYNC_DOMAIN_SELECTOR7, 0x77, 0x55},

	/* set SRC C/D Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR8, 0x77, 0x77},

	/* set SRC Clock Mode -> 1/2 */
	// {SRC_CLK_SETTING, 0x07, 0x01},
	{SRC_CLK_SETTING, 0x09, 0x09},

	/* set SRCE mode setting: short delay roll-off, semi-auto, x'tal */
	{JITTER_CLEANER_SETTING_2, 0x3F, 0x22},
	{JITTER_CLEANER_SETTING_3, 0xF0, 0x00},
	
	/* jack detection power on */
	{DETECTION_POWER_MANAGEMENT, 0x82, 0x80},
	{DETECTION_SETTING_1, 0x3F, 0x23},
	{DETECTION_SETTING_2, 0x07, 0x01},

#ifdef CONFIG_VOICE_WAKEUP
	{VAD_SETTING_2,  0xFF, 0x1F},
	{VAD_SETTING_3,  0xFF, 0x04},
	{VAD_SETTING_4,  0xFF, 0x02},
	{VAD_SETTING_5,  0xFF, 0x03},
	{VAD_SETTING_6,  0xFF, 0x00},
	{VAD_SETTING_7,  0xFF, 0x33},
	{VAD_SETTING_8,  0xFF, 0x31},
	{VAD_SETTING_9,  0xFF, 0xAD},
	{VAD_SETTING_10, 0xFF, 0x02},
	{VAD_HPF_SETTING_1, 0xFF, 0x3B},
	{VAD_HPF_SETTING_2, 0xFF, 0x55},
	{VAD_HPF_SETTING_3, 0xFF, 0xC4},
	{VAD_HPF_SETTING_4, 0xFF, 0xab},
	{VAD_HPF_SETTING_5, 0xFF, 0x36},
	{VAD_HPF_SETTING_6, 0xFF, 0xa9},
	{CREG0_SETTING, 0xFF, 0x02},
	{CREG1_SETTING_H8, 0xFF, 0x00},
	{CREG1_SETTING_L8, 0xFF, 0xC8},
	{CREG2_SETTING_H8, 0xFF, 0x01},
	{CREG2_SETTING_L8, 0xFF, 0x2C},
	{CREG3_SETTING, 0xFF, 0x0F},
	{CREG4_SETTING, 0xFF, 0x3C},
	{CREG5_SETTING, 0xFF, 0x00},
	{CREG6_SETTING, 0xFF, 0x00},
	{CREG7_SETTING, 0xFF, 0x26},
	{VAD_SETTING_1, 0x3F, 0x04},
#endif
};

static int ak4961_find_mpwrsetting(unsigned int mpwr)
{
	int rc;

	switch (mpwr) {
	case 2800:
		rc = 0;	break;
	case 2500:
		rc = 1;	break;
	case 1800:
		rc = 2;	break;
	case 0:
		rc = 3;	break;
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int ak4961_handle_pdata(struct ak4961_priv *ak4961)
{
	struct snd_soc_codec *codec = ak4961->codec;
	struct ak49xx_pdata *pdata = ak4961->pdata;
	int k1, k2, rc = 0;

	if (!pdata) {
		rc = -ENODEV;
		goto done;
	}

	/* figure out MIC-Power value */
	k1 = ak4961_find_mpwrsetting(pdata->micbias.mpwr1_mv);
	k2 = ak4961_find_mpwrsetting(pdata->micbias.mpwr2_mv);

	if (IS_ERR_VALUE(k1) || IS_ERR_VALUE(k2)) {
		rc = -EINVAL;
		goto done;
	}

	/* Set MIC-Power level */
	snd_soc_update_bits(codec, MIC_POWER_LEVEL, 0x03, k1);
	snd_soc_update_bits(codec, MIC_POWER_LEVEL, 0x0C, k2 << 2);

done:
	return rc;
}

static void ak4961_codec_init_reg(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ak4961_codec_reg_init_val); i++)
		snd_soc_update_bits(codec, ak4961_codec_reg_init_val[i].reg,
				ak4961_codec_reg_init_val[i].mask,
				ak4961_codec_reg_init_val[i].val);
}

void ak4961_event_register(
	int (*machine_event_cb)(struct snd_soc_codec *codec,
				enum ak49xx_codec_event),
	struct snd_soc_codec *codec)
{
	struct ak4961_priv *priv = snd_soc_codec_get_drvdata(codec);
	priv->machine_codec_event_cb = machine_event_cb;
}
EXPORT_SYMBOL_GPL(ak4961_event_register);

static void ak4961_init_slim_slave_cfg(struct snd_soc_codec *codec)
{
	struct ak4961_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct afe_param_cdc_slimbus_slave_cfg *cfg;
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	uint64_t eaddr = 0;

	cfg = &priv->slimbus_slave_cfg;
	cfg->minor_version = 1;
	cfg->tx_slave_port_offset = 0;
	cfg->rx_slave_port_offset = 10;

	memcpy(&eaddr, &ak49xx->slim->e_addr, sizeof(ak49xx->slim->e_addr));
	WARN_ON(sizeof(ak49xx->slim->e_addr) != 6);
	cfg->device_enum_addr_lsw = eaddr & 0xFFFFFFFF;
	cfg->device_enum_addr_msw = eaddr >> 32;

	pr_debug("%s: slimbus logical address 0x%llx\n", __func__, eaddr);
}

static int ak4961_device_down(struct ak49xx *ak49xx)
{
	struct snd_soc_codec *codec;

	codec = (struct snd_soc_codec *)(ak49xx->ssr_priv);
	snd_soc_card_change_online_state(codec->component.card, 0);

	return 0;
}

static int ak4961_post_reset_cb(struct ak49xx *ak49xx)
{
	int ret = 0;
	struct snd_soc_codec *codec;
	struct ak4961_priv *ak4961;

	codec = (struct snd_soc_codec *)(ak49xx->ssr_priv);
	ak4961 = snd_soc_codec_get_drvdata(codec);

	snd_soc_card_change_online_state(codec->component.card, 1);

	mutex_lock(&codec->mutex);

	ak4961_update_reg_defaults(codec);
	ak4961_codec_init_reg(codec);

	codec->cache_sync = true;
	snd_soc_cache_sync(codec);
	codec->cache_sync = false;

	ret = ak4961_handle_pdata(ak4961);
	if (IS_ERR_VALUE(ret))
		pr_err("%s: bad pdata\n", __func__);

	ak4961_init_slim_slave_cfg(codec);
	
	ak4961->machine_codec_event_cb(codec, AK49XX_CODEC_EVENT_CODEC_UP);

	mutex_unlock(&codec->mutex);
	return ret;
}

void *ak4961_get_afe_config(struct snd_soc_codec *codec,
			   enum afe_config_type config_type)
{
	struct ak4961_priv *priv = snd_soc_codec_get_drvdata(codec);
//	struct ak49xx *ak4961_core = dev_get_drvdata(codec->dev->parent);

	switch (config_type) {
	case AFE_SLIMBUS_SLAVE_CONFIG:
		return &priv->slimbus_slave_cfg;
	default:
		pr_err("%s: Unknown config_type 0x%x\n", __func__, config_type);
		return NULL;
	}
}

static int ak49xx_ssr_register(struct ak49xx *control,
				int (*device_down_cb)(struct ak49xx *ak49xx),
				int (*device_up_cb)(struct ak49xx *ak49xx),
				void *priv)
{
	control->dev_down = device_down_cb;
	control->post_reset = device_up_cb;
	control->ssr_priv = priv;
	return 0;
}

static void ak4961_aram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

// ZTE_chenjun
#if 0
	if (1 == socinfo_get_ffbm_flag())
	{
		pr_err("%s: ffbm(%d)\n", __func__, socinfo_get_ffbm_flag());
		return;
	}
#endif
//

	if (!fw) {
		dev_err(codec->dev, "ARAM firmware request failed\n");
		ak4961->aram_firmware[ak4961->aram_load_index] = NULL;
	} else {
		ak4961->aram_firmware[ak4961->aram_load_index] = fw;
		dev_dbg(codec->dev, "ARAM firmware request succeed\n");
	}
	ak4961->aram_load_index++;

	if (ak4961->aram_load_index < AK4961_NUM_ARAM) {

		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4961_ARAM_FIRMWARES[ak4961->aram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4961_aram_ready);
	}
}

static void ak4961_pram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

// ZTE_chenjun
#if 0
	if (1 == socinfo_get_ffbm_flag())
	{
		pr_err("%s: ffbm(%d)\n", __func__, socinfo_get_ffbm_flag());
		return;
	}
#endif
//

	if (!fw) {
		dev_err(codec->dev, "PRAM firmware request failed\n");
		ak4961->pram_firmware[ak4961->pram_load_index] = NULL;
	} else {
		ak4961->pram_firmware[ak4961->pram_load_index] = fw;
		dev_dbg(codec->dev, "PRAM firmware request succeed\n");
	}
	ak4961->pram_load_index++;

	if (ak4961->pram_load_index < AK4961_NUM_PRAM) {

		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4961_PRAM_FIRMWARES[ak4961->pram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4961_pram_ready);
	}else{
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4961_ARAM_FIRMWARES[ak4961->aram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4961_aram_ready);
		}
}

static void ak4961_cram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);

// ZTE_chenjun
#if 0
	if (1 == socinfo_get_ffbm_flag())
	{
		pr_err("%s: ffbm(%d)\n", __func__, socinfo_get_ffbm_flag());
		return;
	}
#endif
//

	if (!fw) {
		dev_err(codec->dev, "CRAM firmware request failed\n");
		ak4961->cram_firmware[ak4961->cram_load_index] = NULL;
	} else {
		ak4961->cram_firmware[ak4961->cram_load_index] = fw;
		dev_dbg(codec->dev, "CRAM firmware request succeed\n");
	}
	ak4961->cram_load_index++;

	if (ak4961->cram_load_index < AK4961_NUM_CRAM) {

		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4961_CRAM_FIRMWARES[ak4961->cram_load_index],
				codec->dev, GFP_KERNEL,	codec, ak4961_cram_ready);
	} else {

		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4961_PRAM_FIRMWARES[ak4961->pram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4961_pram_ready);
	}
}

static void ak4961_timer(unsigned long data)
{
	struct ak4961_priv *ak4961 = (struct ak4961_priv *)data;

	mod_timer(&ak4961->timer, jiffies + msecs_to_jiffies(SRC_RESET_TIMEOUT));
}

// ZTE_chenjun
#if 0
static int ak4961_codec_probe(struct snd_soc_codec *codec)
{
	struct ak49xx *control;
	struct ak4961_priv *ak4961;
	struct ak49xx *ak49xx;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	int i;
	void *ptr = NULL;
	struct clk *cdc_ext_clk = NULL;

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;

	ak49xx_ssr_register(control, ak4961_device_down,
			     ak4961_post_reset_cb, (void *)codec);

	dev_info(codec->dev, "%s()\n", __func__);

	ak4961 = kzalloc(sizeof(struct ak4961_priv), GFP_KERNEL);
	if (!ak4961) {
		dev_err(codec->dev, "Failed to allocate private data\n");
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, ak4961);

// ZTE_chenjun
	ak49xx = codec->control_data;

	if (!of_find_property(ak49xx->dev->of_node, "clock-names", NULL)) {
		dev_dbg(ak49xx->dev, "%s: codec not using audio-ext-clk driver\n",
			__func__);
	} else {
		cdc_ext_clk = clk_get(ak49xx->dev, "cdc_clk");
		if (IS_ERR(cdc_ext_clk)) {
			dev_err(codec->dev, "%s: clk get %s failed\n",
					__func__, "cdc_ext_clk");
			goto err_nomem_slimch;
		}
	}
	ak4961->cdc_ext_clk = cdc_ext_clk;
//

	ak4961->codec = codec;
	
	ak4961_codec=codec;//lhs for ram codec write
	
	ak4961->pdata = dev_get_platdata(codec->dev->parent);
	ak4961->intf_type = ak49xx_get_intf_type();

	mutex_init(&ak4961->mutex);

	ak4961_update_reg_defaults(codec);
	ak4961_codec_init_reg(codec);
	ret = ak4961_handle_pdata(ak4961);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: bad pdata\n", __func__);
		goto err_pdata;
	}

	ptr = kmalloc((sizeof(ak4961_rx_chs) +
			   sizeof(ak4961_tx_chs)), GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: no mem for slim chan ctl data\n", __func__);
		ret = -ENOMEM;
		goto err_nomem_slimch;
	}

	if (ak4961->intf_type == AK49XX_INTERFACE_TYPE_SPI ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_I2C ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		snd_soc_dapm_new_controls(dapm, ak4961_dapm_i2s_widgets,
			ARRAY_SIZE(ak4961_dapm_i2s_widgets));
		snd_soc_dapm_add_routes(dapm, audio_i2s_map,
			ARRAY_SIZE(audio_i2s_map));
		for (i = 0; i < ARRAY_SIZE(ak4961_i2s_dai); i++)
			INIT_LIST_HEAD(&ak4961->dai[i].ak49xx_ch_list);
	}
	//for slim_spi 2014-11-17 星期一 14:54:44
	if (ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		for (i = 0; i < NUM_CODEC_DAIS; i++) {
			INIT_LIST_HEAD(&ak4961->dai[i].ak49xx_ch_list);
			init_waitqueue_head(&ak4961->dai[i].dai_wait);
		}
		ak4961_slimbus_slave_port_cfg.slave_dev_intfdev_la =
			control->slim_slave->laddr;
		ak4961_slimbus_slave_port_cfg.slave_dev_pgd_la =
			control->slim->laddr;
		ak4961_slimbus_slave_port_cfg.slave_port_mapping[0] = 10;

		ak4961_init_slim_slave_cfg(codec);
	}

	control->num_rx_port = AK4961_RX_MAX;
	control->rx_chs = ptr;
	memcpy(control->rx_chs, ak4961_rx_chs, sizeof(ak4961_rx_chs));
	control->num_tx_port = AK4961_TX_MAX;
	control->tx_chs = ptr + sizeof(ak4961_rx_chs);
	memcpy(control->tx_chs, ak4961_tx_chs, sizeof(ak4961_tx_chs));

	snd_soc_dapm_sync(dapm);

	ak4961->stream_state = AK4961_SLIMBUS_STREAM_NA;

	ak4961->workqueue = create_singlethread_workqueue("ak4961_wq");
	if (ak4961->workqueue == NULL) {
		goto err_wk_irq;
	}
	INIT_WORK(&ak4961->work, ak4961_work);

	init_timer(&ak4961->timer);
	ak4961->timer.data = (unsigned long)ak4961;
	ak4961->timer.function = ak4961_timer;

#ifdef CONFIG_DEBUG_FS_CODEC
	debugak49xx = control;
#endif

	ak4961->cram_load_index = 0;
	ak4961->pram_load_index = 0;
	ak4961->aram_load_index = 0;

// ZTE_chenjun
#if 0
	if (1 == socinfo_get_ffbm_flag())
	{
		pr_err("%s: ffbm(%d)\n", __func__, socinfo_get_ffbm_flag());
	}
	else
#endif
	{
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			AK4961_CRAM_FIRMWARES[0], codec->dev, GFP_KERNEL,
			codec, ak4961_cram_ready);
	}
//

	ak4961->ak4961_dsp_downlink_status = 0;
	ak4961->ak4961_dsp_uplink_status = 0;
	ak4961->ak4961_voice_sync_switch = 0;

	codec->component.ignore_pmdown_time = 1;
	return ret;

err_wk_irq:
err_pdata:	
	kfree(ptr);

err_nomem_slimch:
	kfree(ak4961);

	return ret;
}
#else
static int ak4961_codec_probe(struct snd_soc_codec *codec)
{
	struct ak49xx *control;
	struct ak4961_priv *ak4961;
	struct ak49xx *ak49xx;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	int i;
	void *ptr = NULL;
	struct clk *cdc_ext_clk = NULL;

	dev_info(codec->dev, "%s()\n", __func__);

//weizj add for audience es804
#ifdef CONFIG_SND_SOC_ESXXX
    dev_info(codec->dev, "%s: weizj reg es804", __func__);
	remote_add_codec_controls(codec);
#endif

	control = dev_get_drvdata(codec->dev->parent);

	ak4961 = snd_soc_codec_get_drvdata(codec);
	if (!ak4961) {
		dev_err(codec->dev, "Failed to allocate private data\n");
		return -ENOMEM;
	}

	ak49xx_ssr_register(control, ak4961_device_down,
			     ak4961_post_reset_cb, (void *)codec);


// ZTE_chenjun
	ak49xx = control;

	if (!of_find_property(ak49xx->dev->of_node, "clock-names", NULL)) {
		dev_dbg(ak49xx->dev, "%s: codec not using audio-ext-clk driver\n",
			__func__);
	} else {
		cdc_ext_clk = clk_get(ak49xx->dev, "cdc_clk");
		if (IS_ERR(cdc_ext_clk)) {
			dev_err(codec->dev, "%s: clk get %s failed\n",
					__func__, "cdc_ext_clk");
			goto err_nomem_slimch;
		}
	}
	ak4961->cdc_ext_clk = cdc_ext_clk;
//

	ak4961->codec = codec;
	
	ak4961_codec=codec;//lhs for ram codec write
	
	ak4961->pdata = dev_get_platdata(codec->dev->parent);
	ak4961->intf_type = ak49xx_get_intf_type();

	mutex_init(&ak4961->mutex);

	ak4961_update_reg_defaults(codec);
	ak4961_codec_init_reg(codec);
	ret = ak4961_handle_pdata(ak4961);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: bad pdata\n", __func__);
		goto err_pdata;
	}

	ptr = kmalloc((sizeof(ak4961_rx_chs) +
			   sizeof(ak4961_tx_chs)), GFP_KERNEL);
	if (!ptr) {
		pr_err("%s: no mem for slim chan ctl data\n", __func__);
		ret = -ENOMEM;
		goto err_nomem_slimch;
	}

	if (ak4961->intf_type == AK49XX_INTERFACE_TYPE_SPI ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_I2C ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
			ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		snd_soc_dapm_new_controls(dapm, ak4961_dapm_i2s_widgets,
			ARRAY_SIZE(ak4961_dapm_i2s_widgets));
		snd_soc_dapm_add_routes(dapm, audio_i2s_map,
			ARRAY_SIZE(audio_i2s_map));
		for (i = 0; i < ARRAY_SIZE(ak4961_i2s_dai); i++)
			INIT_LIST_HEAD(&ak4961->dai[i].ak49xx_ch_list);
	}
	//for slim_spi 2014-11-17 星期一 14:54:44
	if (ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak4961->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		for (i = 0; i < NUM_CODEC_DAIS; i++) {
			INIT_LIST_HEAD(&ak4961->dai[i].ak49xx_ch_list);
			init_waitqueue_head(&ak4961->dai[i].dai_wait);
		}
		ak4961_slimbus_slave_port_cfg.slave_dev_intfdev_la =
			control->slim_slave->laddr;
		ak4961_slimbus_slave_port_cfg.slave_dev_pgd_la =
			control->slim->laddr;
		ak4961_slimbus_slave_port_cfg.slave_port_mapping[0] = 10;

		ak4961_init_slim_slave_cfg(codec);
	}

	control->num_rx_port = AK4961_RX_MAX;
	control->rx_chs = ptr;
	memcpy(control->rx_chs, ak4961_rx_chs, sizeof(ak4961_rx_chs));
	control->num_tx_port = AK4961_TX_MAX;
	control->tx_chs = ptr + sizeof(ak4961_rx_chs);
	memcpy(control->tx_chs, ak4961_tx_chs, sizeof(ak4961_tx_chs));

	snd_soc_dapm_sync(dapm);

	ak4961->stream_state = AK4961_SLIMBUS_STREAM_NA;

	ak4961->workqueue = create_singlethread_workqueue("ak4961_wq");
	if (ak4961->workqueue == NULL) {
		goto err_wk_irq;
	}
	INIT_WORK(&ak4961->work, ak4961_work);

	init_timer(&ak4961->timer);
	ak4961->timer.data = (unsigned long)ak4961;
	ak4961->timer.function = ak4961_timer;

#ifdef CONFIG_DEBUG_FS_CODEC
	debugak49xx = control;
#endif

	ak4961->cram_load_index = 0;
	ak4961->pram_load_index = 0;
	ak4961->aram_load_index = 0;

// ZTE_chenjun
#if 0
	if (1 == socinfo_get_ffbm_flag())
	{
		pr_err("%s: ffbm(%d)\n", __func__, socinfo_get_ffbm_flag());
	}
	else
#endif
	{
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			AK4961_CRAM_FIRMWARES[0], codec->dev, GFP_KERNEL,
			codec, ak4961_cram_ready);
	}
//

	ak4961->ak4961_dsp_downlink_status = 0;
	ak4961->ak4961_dsp_uplink_status = 0;
	ak4961->ak4961_voice_sync_switch = 0;
	ak4961->ak4961_slimbus_interface_reg=0;
	ak4961->ak4961_power_test=0;

	codec->component.ignore_pmdown_time = 1;
	return ret;

err_wk_irq:
err_pdata:	
	kfree(ptr);

err_nomem_slimch:
	kfree(ak4961);

	return ret;
}
#endif

static int ak4961_codec_remove(struct snd_soc_codec *codec)
{
	int i;
	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = dev_get_drvdata(codec->dev->parent);
	struct ak49xx_core_resource *core_res = &ak49xx->core_res;

	for (i = 0; i < AK4961_NUM_PRAM; i++) {
		if (ak4961->pram_firmware[0])
			release_firmware(ak4961->pram_firmware[0]);
	}
	for (i = 0; i < AK4961_NUM_CRAM; i++) {
		if (ak4961->cram_firmware[0])
			release_firmware(ak4961->cram_firmware[0]);
	}

#ifdef CONFIG_SWITCH
	input_unregister_device(ak4961->mbhc_cfg.btn_idev);

	if (ak4961->mbhc_cfg.h2w_sdev != NULL) {
		switch_dev_unregister(ak4961->mbhc_cfg.h2w_sdev);
		kfree(ak4961->mbhc_cfg.h2w_sdev);
		ak4961->mbhc_cfg.h2w_sdev = NULL;
	}
#endif

// ZTE_chenjun
	if (ak4961->cdc_ext_clk)
		clk_put(ak4961->cdc_ext_clk);
//

	ak49xx_free_irq(core_res, AK4961_IRQ_JDE, ak4961);
	ak49xx_free_irq(core_res, AK4961_IRQ_RCE, ak4961);
	ak49xx_free_irq(core_res, AK4961_IRQ_VAD, ak4961);
	destroy_workqueue(ak4961->workqueue);
	del_timer_sync(&ak4961->timer);
	kfree(ak4961);
	return 0;
}

static int ak4961_readable(struct snd_soc_codec *ssc, unsigned int reg)
{
	return ak4961_reg_readable[reg];
}

static int ak4961_volatile(struct snd_soc_codec *ssc, unsigned int reg)
{
	/* Registers between 0xC8 and 0xE8 are not cacheable */
	if ((reg >= CRC_RESULT_H8) && (reg <= MIR4_REGISTER_4))
		return 1;

	if ((reg >= SRC_CLK_SETTING) && (reg <= SRC_MUTE_CONTROL))
		return 1;

	if ((reg >= DSP_SETTING1) && (reg <= DSP_SETTING5))
		return 1;

	if ((reg >= SYNC_DOMAIN_SELECTOR7) && (reg <= SYNC_DOMAIN_SELECTOR8))
		return 1;

	if (reg == POWER_MANAGEMENT_2)
		return 1;

	return 0;
}

static int ak4961_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;
	struct ak49xx *control = dev_get_drvdata(codec->dev->parent);

	if (reg == SND_SOC_NOPM)
		return 0;

	BUG_ON(reg > AK4961_MAX_REGISTER);

	if (!ak4961_volatile(codec, reg)) {
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret != 0)
			dev_err(codec->dev, "Cache write to %x failed: %d\n",
				reg, ret);
	}

	return ak49xx_reg_write(&control->core_res, reg, value);
}

static unsigned int ak4961_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;
	int ret;
	struct ak49xx *control = dev_get_drvdata(codec->dev->parent);

	if (reg == SND_SOC_NOPM)
		return 0;

	BUG_ON(reg > AK4961_MAX_REGISTER);

	if (!ak4961_volatile(codec, reg) && ak4961_readable(codec, reg) &&
		reg < codec->driver->reg_cache_size) {
		ret = snd_soc_cache_read(codec, reg, &val);
		if (ret >= 0) {
			return val;
		} else
			dev_err(codec->dev, "Cache read from %x failed: %d\n",
				reg, ret);
	}

	val = ak49xx_reg_read(&control->core_res, reg);
	return val;
}

static int ak4961_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
//	struct ak4961_priv *ak4961 = snd_soc_codec_get_drvdata(codec);
//	struct ak49xx *control = codec->control_data;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
		}
		if (codec->dapm.bias_level == SND_SOC_BIAS_ON) {
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
		}

		if (codec->dapm.bias_level >= SND_SOC_BIAS_PREPARE) {
		}
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}


/*
 * wcd_mbhc_init : initialize MBHC internal structures.
 *
 * NOTE: mbhc->mbhc_cfg is not YET configure so shouldn't be used
 */
 
/* zte weizhijun add headset detect /proc/hs 2013-12-17 start */
static ssize_t hs_read(struct file *file, char __user *buf,
			      size_t count, loff_t *pos)
{
    int ret = 0;
    char buffer[32] = {0};
    ret = scnprintf(buffer, 32, "%d\n", hs_type);
    pr_err("weizhijun %s: hs_read  %d\n", __func__, hs_type);

    ret = simple_read_from_buffer(buf, count, pos, buffer, ret);

    return ret;
}

static const struct file_operations hs_detect_ops = {
    .owner    = THIS_MODULE,
    .read     = hs_read,
};
/* zte weizhijun add headset detect /proc/hs 2013-12-17 end */

static struct snd_soc_codec_driver soc_codec_dev_ak4961 = {
	.probe	= ak4961_codec_probe,
	.remove	= ak4961_codec_remove,

	.read = ak4961_read,
	.write = ak4961_write,

	.readable_register = ak4961_readable,
	.volatile_register = ak4961_volatile,

	.reg_cache_size = AK4961_CACHE_SIZE,
	.reg_cache_default = ak4961_reg_defaults,
	.reg_word_size = 1,

	.controls = ak4961_snd_controls,
	.num_controls = ARRAY_SIZE(ak4961_snd_controls),
	.dapm_widgets = ak4961_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4961_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),

	.set_bias_level = ak4961_set_bias_level,
};

#ifdef CONFIG_PM
static int ak4961_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);
	return 0;
}

static int ak4961_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ak4961_priv *ak4961 = platform_get_drvdata(pdev);
	dev_dbg(dev, "%s: system resume ak4961 %p\n", __func__, ak4961);
	return 0;
}

static const struct dev_pm_ops ak4961_pm_ops = {
	.suspend	= ak4961_suspend,
	.resume		= ak4961_resume,
};
#endif
#if 0
int ak4961_wn_bandswitch(int howtochange)
{

	struct snd_soc_codec *codec = ak4961_codec;

	ak49xx_bandswitch_set(codec, howtochange);
	return 0;
}
static ssize_t voice_wn_proc_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count = 0;

	pr_err("%s: enter\n", __func__);
	if (*offset > 0) {
		count = 0;
	} else {
		pr_err("%s: enter %d\n", __func__, ak4961_band_mode);
		count = snprintf(buf, sizeof(buf), "%d\n", ak4961_band_mode);
		*offset += count;
	}
	return count;
}

static ssize_t voice_wn_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	int ret;
	int rc;
	int err = 0;

	rc = kstrtoint_from_user(buffer, count, 0, &ret);
	if (rc)
		return rc;

	if (ret < 0 || ret > 10) {
		pr_err("%s: ret=%d out of scope(ret must 0~3)\n", __func__, ret);
		return 1;
	}
#if 0
	if (ret == ak_ctrl_pdata)
		return 1;
#endif
	ak4961_band_mode = ret;

	err = ak4961_wn_bandswitch(ak4961_band_mode);

	pr_err("%s: current_ce_level = %d\n", __func__, ak4961_band_mode);
	return 1;

}

static const struct file_operations voice_wn_proc_fops = {
	.owner  = THIS_MODULE,
	.read		= voice_wn_proc_read,
	.write		= voice_wn_proc_write,
};

static int voice_wn_proc_init(void)
{
	struct proc_dir_entry *res;

	res = proc_create("voice_wn_switch", S_IFREG|S_IRWXUGO, NULL,
			  &voice_wn_proc_fops);
	if (!res) {
		pr_info("failed to create /proc/voice_wn_switch\n");
		return -ENOMEM;
	}

	pr_info("created /proc/voice_wn_switch\n");
	return 0;
}
/*add by shengguanghui for voice_wn switch 20160203 end*/
#endif
static int ak4961_probe(struct platform_device *pdev)
{
	int ret = 0;
/*	int nw;*/
	struct proc_dir_entry *proc_hs_type;//weizhijun add headset detect
	struct ak4961_priv *ak4961;

	pr_info("ak4961_probe\n");

	ak4961 = devm_kzalloc(&pdev->dev, sizeof(struct ak4961_priv),
			      GFP_KERNEL);
	if (!ak4961) {
		dev_err(&pdev->dev, "%s: cannot create memory for ak4961\n",
			__func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ak4961);

#ifdef CONFIG_DEBUG_FS_CODEC
	ret = device_create_file(&pdev->dev, &dev_attr_reg_data);
	if (ret) {
		pr_err("%s: Error to create reg_data\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_ram_data);
	if (ret) {
		pr_err("%s: Error to create ram_data\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_ram_load);
	if (ret) {
		pr_err("%s: Error to create ram_load\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_mir_data);
	if (ret) {
		pr_err("%s: Error to create mir_data\n", __func__);
	}
#endif
/*	nw = voice_wn_proc_init();*/
/*add by shengguanghui for voice_wn_switch at 20160229*/

/* zte weizhijun add headset detect /proc/hs 2013-12-17 start */
	proc_hs_type = proc_create("hs", S_IRUGO, NULL, &hs_detect_ops);
	if (!proc_hs_type) {
		printk(KERN_ERR"[YXS]hs: unable to register '/proc/hs' \n");
	}
	pr_err("weizhijun %s: enter\n", __func__);
/* zte weizhijun add headset detect /proc/hs 2013-12-17 end*/
	//for slim_spi 2014-11-17 星期一 14:55:55
	if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS ||
			ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		pr_info("ak4961_probe: register SLIM_SPI_bus dai\n");
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ak4961,
			ak4961_dai, ARRAY_SIZE(ak4961_dai));
	} else if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SPI ||
			   ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_I2C) {
		pr_info("ak4961_probe: register I2S dai\n");
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ak4961,
			ak4961_i2s_dai, ARRAY_SIZE(ak4961_i2s_dai));
	}
	return ret;
}

static int ak4961_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS_CODEC
	device_remove_file(&pdev->dev, &dev_attr_reg_data);
	device_remove_file(&pdev->dev, &dev_attr_ram_data);
	device_remove_file(&pdev->dev, &dev_attr_ram_load);
	device_remove_file(&pdev->dev, &dev_attr_mir_data);
#endif
/* zte weizhijun add headset detect /proc/hs 2013-12-17 start */
	remove_proc_entry("hs", NULL);
    pr_err("weizhijun %s: deinit\n", __func__);
/* zte weizhijun add headset detect /proc/hs 2013-12-17 end */
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ak4961_codec_driver = {
	.probe = ak4961_probe,
	.remove = ak4961_remove,
	.driver = {
		.name = "ak4961_codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &ak4961_pm_ops,
#endif
	},
};

static int __init ak4961_codec_init(void)
{
	int rtn = platform_driver_register(&ak4961_codec_driver);

	if (rtn != 0) {
			platform_driver_unregister(&ak4961_codec_driver);
	}
	return rtn;
}

static void __exit ak4961_codec_exit(void)
{
	platform_driver_unregister(&ak4961_codec_driver);
}

module_init(ak4961_codec_init);
module_exit(ak4961_codec_exit);

MODULE_AUTHOR("Haizhen Li <li.kd@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("ak4961 codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
