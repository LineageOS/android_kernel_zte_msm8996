/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#ifndef AK4961_CODEC_JDE_H
#define AK4961_CODEC_JDE_H

#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/switch.h>
#include <linux/input.h>

#define NR_SCANCODES 2

#define AK4961_JACK_BUTTON_MASK (SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
				SND_JACK_BTN_2 | SND_JACK_BTN_3 | \
				SND_JACK_BTN_4)
#define MAX_MIC_DET_TRY 9

#define MAX_MIC_DET_RPT 5       //连续检测次数

/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static const unsigned short ak4961_keycode[NR_SCANCODES] = {
	KEY_VOLUMEDOWN, KEY_VOLUMEUP
};

struct ak4961_mbhc_config {
	struct snd_soc_jack *headset_jack;
	struct snd_soc_jack *button_jack;
#ifdef CONFIG_SWITCH
	struct switch_dev	*h2w_sdev;
	struct input_dev	*btn_idev;
	unsigned short keycode[NR_SCANCODES];
#endif
// ZTE_weizj 20150601fix:Fake button press while inserting headset
#if 0
       bool reject_btn;
       struct delayed_work reject_btn_dwork;
#else
       bool reject_btn;
	struct workqueue_struct *btn_workqueue;
	struct delayed_work btn_work;
#endif
// ZTE_weizj 20150601 end
	int (*mclk_cb_fn) (struct snd_soc_codec*, int, bool);
	unsigned int mclk_rate;
	int gpio_level_insert;
	/* swap_gnd_mic returns true if extern GND/MIC swap switch toggled */
	bool (*swap_gnd_mic) (struct snd_soc_codec *);
};

extern int ak4961_hs_detect(struct snd_soc_codec *codec,
			 const struct ak4961_mbhc_config *cfg);

extern int ak4961_mclk_enable(struct snd_soc_codec *codec, int mclk_enable,
			     bool dapm);

#endif
