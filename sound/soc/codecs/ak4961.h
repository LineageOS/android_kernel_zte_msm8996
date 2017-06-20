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
#ifndef AK4961_CODEC_H
#define AK4961_CODEC_H

#include "ak4961-jde.h"
#include <sound/apr_audio-v2.h>
#include <linux/mfd/ak49xx/ak49xx-slimslave.h>

#define AK4961_NUM_REGISTERS 271
#define AK4961_MAX_REGISTER (AK4961_NUM_REGISTERS-1)
#define AK4961_CACHE_SIZE AK4961_NUM_REGISTERS

#define MAX_ARRAY_SIZE		800

#define AK4961_REG_VAL(reg, val)		{reg, 0, val}

extern const u8 ak4961_reg_readable[AK4961_CACHE_SIZE];
extern const u8 ak4961_reg_defaults[AK4961_CACHE_SIZE];

#ifdef ZTE_FEATURE_AKM_VOICECALL
#define AKM_VOICE_CALL
#endif

enum ak4961_state {
	AK4961_IDLE = 0,
	AK4961_DSPRSTNON,
};

enum ak4961_slimbus_stream_state {
	AK4961_SLIMBUS_STREAM_NA = 0,
	AK4961_SLIMBUS_STREAM_OFF,
	AK4961_SLIMBUS_STREAM_ON,
};

enum ak4961_pid_current {
	AK4961_PID_MIC_2P5_UA,
	AK4961_PID_MIC_5_UA,
	AK4961_PID_MIC_10_UA,
	AK4961_PID_MIC_20_UA,
};

struct ak4961_reg_mask_val {
	u16	reg;
	u8	mask;
	u8	val;
};

enum ak4961_mbhc_clk_freq {
	AK4961_MCLK_12P2MHZ = 0,
	AK4961_MCLK_9P6MHZ,
	AK4961_NUM_CLK_FREQS,
};

enum ak49xx_codec_event {
	AK49XX_CODEC_EVENT_CODEC_UP = 0,
};

#define	SIZE_OF_PRAM_PHONE		1000

enum ak4961_dsp_mode_enum{
	DSP_MODE_OFF = 0,
	DSP_MODE_NARROW_HANDSET,
	DSP_MODE_NARROW_HEADSET,
	DSP_MODE_NARROW_HANDSFREE,
	DSP_MODE_WIDE_HANDSET,
	DSP_MODE_WIDE_HEADSET,
#ifdef AKM_VOICE_CALL
	DSP_MODE_WIDE_HEADPHONE,
#endif
	DSP_MODE_WIDE_HANDSFREE,
	DSP_MODE_VOICE_RECOGNITION,
	DSP_MODE_SOUND_RECORD,
	DSP_MODE_VOICE_RECORD,
       DSP_MODE_WCHAT_RECORD,
	DSP_MODE_MUSIC_SPEAKER,
	DSP_MODE_INTERVIEW_RECORD,
	DSP_MODE_BARGE_IN,
	DSP_MODE_KARAOKE_HEAVY,
	DSP_MODE_KARAOKE_LIGHT,
	DSP_MODE_KARAOKE_MIDDLE,
	DSP_MODE_KARAOKE_SPK_ROOM,
	DSP_MODE_KARAOKE_SPK_LIVE,
	DSP_MODE_KARAOKE_SPK_THEATER
};

/* Number of input and output Slimbus port */
enum {
	AK4961_RX1 = 0,
	AK4961_RX2,
	AK4961_RX3,
	AK4961_RX4,
	AK4961_RX5,
	AK4961_RX6,
	AK4961_RX_MAX,
};

enum {
	AK4961_TX1 = 0,
	AK4961_TX2,
	AK4961_TX3,
	AK4961_TX4,
	AK4961_TX5,
	AK4961_TX6,
	AK4961_TX7,
	AK4961_TX8,
	AK4961_TX9,
	AK4961_TX10,
	AK4961_TX_MAX,
};

extern void *ak4961_get_afe_config(struct snd_soc_codec *codec,
				  enum afe_config_type config_type);

extern void ak4961_event_register(
	int (*machine_event_cb)(struct snd_soc_codec *codec,
				enum ak49xx_codec_event),
	struct snd_soc_codec *codec);

extern int ak4961_internal_rx_gain_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol);

// ZTE_chenjun
int ak4961_codec_mclk_enable(struct snd_soc_codec *codec,
			     int enable, bool dapm);
//

#endif
