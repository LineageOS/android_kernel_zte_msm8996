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
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/ak49xx/ak49xx-slimslave.h>

struct ak49xx_slim_sch {
	u16 rx_port_ch_reg_base;
	u16 port_tx_cfg_reg_base;
	u16 port_rx_cfg_reg_base;
};

static struct ak49xx_slim_sch sh_ch;

static int ak49xx_alloc_slim_sh_ch(struct ak49xx *ak49xx,
				    u8 ak49xx_pgd_la, u32 cnt,
				    struct ak49xx_ch *channels, u32 path);

static int ak49xx_dealloc_slim_sh_ch(struct slim_device *slim,
				      u32 cnt, struct ak49xx_ch *channels);

static int ak49xx_configure_ports(struct ak49xx *ak49xx)
{

	if ((ak49xx->codec_id == CODEC_AK4960_ID) ||
		(ak49xx->codec_id == CODEC_AK4961_ID) ||
		(ak49xx->codec_id == CODEC_AK4962_ID)) {
		sh_ch.rx_port_ch_reg_base = 0x180;
		sh_ch.port_rx_cfg_reg_base = 0x040;
		sh_ch.port_tx_cfg_reg_base = 0x040;

	} else {
		sh_ch.rx_port_ch_reg_base =
			0x180 - (AK4962_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS * 4);
		sh_ch.port_rx_cfg_reg_base =
			0x040 - AK4962_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS;
		sh_ch.port_tx_cfg_reg_base = 0x050;
	}

	return 0;
}

int ak49xx_init_slimslave(struct ak49xx *ak49xx, u8 ak49xx_pgd_la,
		   unsigned int tx_num, unsigned int *tx_slot,
		   unsigned int rx_num, unsigned int *rx_slot)
{
	int ret = 0;
	int i;

	ret = ak49xx_configure_ports(ak49xx);
	if (ret) {
		pr_err("%s: Failed to configure register address offset\n",
		       __func__);
		goto err;
	}

	if (ak49xx->rx_chs) {
		ak49xx->num_rx_port = rx_num;
		for (i = 0; i < rx_num; i++) {
			ak49xx->rx_chs[i].ch_num = rx_slot[i];
			INIT_LIST_HEAD(&ak49xx->rx_chs[i].list);
		}
		ret = ak49xx_alloc_slim_sh_ch(ak49xx, ak49xx_pgd_la,
						ak49xx->num_rx_port,
						ak49xx->rx_chs,
						SLIM_SINK);
		if (ret) {
			pr_err("%s: Failed to alloc %d rx slimbus channels\n",
				__func__, ak49xx->num_rx_port);
			kfree(ak49xx->rx_chs);
			ak49xx->rx_chs = NULL;
			ak49xx->num_rx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus rx ports\n",
			ak49xx->num_rx_port);
	}

	if (ak49xx->tx_chs) {
		ak49xx->num_tx_port = tx_num;
		for (i = 0; i < tx_num; i++) {
			ak49xx->tx_chs[i].ch_num = tx_slot[i];
			INIT_LIST_HEAD(&ak49xx->tx_chs[i].list);
		}
		ret = ak49xx_alloc_slim_sh_ch(ak49xx, ak49xx_pgd_la,
						ak49xx->num_tx_port,
						ak49xx->tx_chs,
						SLIM_SRC);
		if (ret) {
			pr_err("%s: Failed to alloc %d tx slimbus channels\n",
				__func__, ak49xx->num_tx_port);
			kfree(ak49xx->tx_chs);
			ak49xx->tx_chs = NULL;
			ak49xx->num_tx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus tx ports\n",
			ak49xx->num_tx_port);
	}

	return 0;
err:
	return ret;
}


int ak49xx_deinit_slimslave(struct ak49xx *ak49xx)
{
	if (ak49xx->num_rx_port) {
		ak49xx_dealloc_slim_sh_ch(ak49xx->slim,
					ak49xx->num_rx_port,
					ak49xx->rx_chs);
		ak49xx->num_rx_port = 0;
	}
	if (ak49xx->num_tx_port) {
		ak49xx_dealloc_slim_sh_ch(ak49xx->slim,
					ak49xx->num_tx_port,
					ak49xx->tx_chs);
		ak49xx->num_tx_port = 0;
	}
	return 0;
}

static int ak49xx_alloc_slim_sh_ch(struct ak49xx *ak49xx,
				    u8 ak49xx_pgd_la, u32 cnt,
				    struct ak49xx_ch *channels, u32 path)
{
	int ret = 0;
	u32 ch_idx;

	/* The slimbus channel allocation seem take longer time
	 * so do the allocation up front to avoid delay in start of
	 * playback
	 */
	pr_debug("%s: pgd_la[%d]\n", __func__, ak49xx_pgd_la);
	for (ch_idx = 0; ch_idx < cnt; ch_idx++) {
		ret = slim_get_slaveport(ak49xx_pgd_la,
					channels[ch_idx].port,
					&channels[ch_idx].sph, path);
		pr_debug("%s: pgd_la[%d] channels[%d].port[%d]\n"
			"channels[%d].sph[%d] path[%d]\n",
			__func__, ak49xx_pgd_la, ch_idx,
			channels[ch_idx].port,
			ch_idx, channels[ch_idx].sph, path);
		if (ret < 0) {
			pr_err("%s: slave port failure id[%d] ret[%d]\n",
				__func__, channels[ch_idx].ch_num, ret);
			goto err;
		}

		ret = slim_query_ch(ak49xx->slim,
				    channels[ch_idx].ch_num,
				    &channels[ch_idx].ch_h);
		if (ret < 0) {
			pr_err("%s: slim_query_ch failed ch-num[%d] ret[%d]\n",
				__func__, channels[ch_idx].ch_num, ret);
			goto err;
		}
	}
err:
	return ret;
}

static int ak49xx_dealloc_slim_sh_ch(struct slim_device *slim,
			u32 cnt, struct ak49xx_ch *channels)
{
	int idx = 0;
	int ret = 0;
	/* slim_dealloc_ch */
	for (idx = 0; idx < cnt; idx++) {
		ret = slim_dealloc_ch(slim, channels[idx].ch_h);
		if (ret < 0) {
			pr_err("%s: slim_dealloc_ch fail ret[%d] ch_h[%d]\n",
				__func__, ret, channels[idx].ch_h);
		}
	}
	return ret;
}

/* Enable slimbus slave device for RX path */
int ak49xx_cfg_slim_sch_rx(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph)
{
	u8 ch_cnt = 0;
	u16 ch_h[SLIM_MAX_RX_PORTS] = {0};
	u8  payload = 0;
	u16 codec_port = 0;
	int ret;
	struct slim_ch prop;
	struct ak49xx_ch *rx;

	/* Configure slave interface device */

	list_for_each_entry(rx, ak49xx_ch_list, list) {
		payload |= 1 << rx->shift;
		ch_h[ch_cnt] = rx->ch_h;
		ch_cnt++;
		pr_debug("list ch->ch_h %d ch->sph %d\n", rx->ch_h, rx->sph);
	}
	pr_debug("%s: ch_cnt[%d] rate=%d\n", __func__, ch_cnt, rate);
	/* slim_define_ch api */
	prop.prot = SLIM_AUTO_ISO;
	prop.baser = SLIM_RATE_4000HZ;
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.ratem = (rate/4000);
	prop.sampleszbits = bit_width;

	pr_debug("Before slim_define_ch:\n"
		 "ch_cnt %d,ch_h[0] %d ch_h[1] %d, grph %d\n",
		 ch_cnt, ch_h[0], ch_h[1], *grph);
	ret = slim_define_ch(ak49xx->slim, &prop, ch_h, ch_cnt,
			     true, grph);
	if (ret < 0) {
		pr_err("%s: slim_define_ch failed ret[%d]\n",
		       __func__, ret);
		goto err;
	}

	list_for_each_entry(rx, ak49xx_ch_list, list) {
		codec_port = rx->port;
		pr_debug("%s: codec_port %d rx 0x%p, payload %d\n"
			 "sh_ch.rx_port_ch_reg_base0 0x%x\n"
			 "sh_ch.port_rx_cfg_reg_base 0x%x\n",
			 __func__, codec_port, rx, payload,
			 sh_ch.rx_port_ch_reg_base,
			sh_ch.port_rx_cfg_reg_base);

		/* look for the valid port range and chose the
		 * payload accordingly
		 */
		/* write to interface device
		ret = ak49xx_interface_reg_write(ak49xx,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				sh_ch.rx_port_ch_reg_base, codec_port),
				payload);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				sh_ch.rx_port_ch_reg_base, codec_port),
				payload, ret);
			goto err;
		}

		// configure the slave port for water mark and enable
		ret = ak49xx_interface_reg_write(ak49xx,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				sh_ch.port_rx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
		*/
		ret = slim_connect_sink(ak49xx->slim, &rx->sph, 1, rx->ch_h);
		if (ret < 0) {
			pr_err("%s: slim_connect_sink failed ret[%d]\n",
				__func__, ret);
			goto err_close_slim_sch;
		}
	}
	/* slim_control_ch */
	ret = slim_control_ch(ak49xx->slim, *grph, SLIM_CH_ACTIVATE,
			      true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err_close_slim_sch;
	}
	return 0;

err_close_slim_sch:
	/*  release all acquired handles */
	ak49xx_close_slim_sch_rx(ak49xx, ak49xx_ch_list, *grph);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_cfg_slim_sch_rx);

/* Enable slimbus slave device for RX path */
int ak49xx_cfg_slim_sch_tx(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph)
{
	u16 ch_cnt = 0;
	u16 payload = 0;
	u16 ch_h[SLIM_MAX_TX_PORTS] = {0};
	u16 codec_port;
	int ret = 0;
	struct ak49xx_ch *tx;

	struct slim_ch prop;

	list_for_each_entry(tx, ak49xx_ch_list, list) {
		payload |= 1 << tx->shift;
		ch_h[ch_cnt] = tx->ch_h;
		ch_cnt++;
	}

	/* slim_define_ch api */
	prop.prot = SLIM_AUTO_ISO;
	prop.baser = SLIM_RATE_4000HZ;
	prop.dataf = SLIM_CH_DATAF_NOT_DEFINED;
	prop.auxf = SLIM_CH_AUXF_NOT_APPLICABLE;
	prop.ratem = (rate/4000);
	prop.sampleszbits = bit_width;		// bit_width
	ret = slim_define_ch(ak49xx->slim, &prop, ch_h, ch_cnt,
			     true, grph);
	if (ret < 0) {
		pr_err("%s: slim_define_ch failed ret[%d]\n",
		       __func__, ret);
		goto err;
	}

	pr_debug("%s: ch_cnt[%d] rate[%d]\n", __func__, ch_cnt, rate);
	list_for_each_entry(tx, ak49xx_ch_list, list) {
		codec_port = tx->port;
		pr_debug("%s: codec_port %d tx 0x%p, payload 0x%x\n",
			 __func__, codec_port, tx, payload);

		/* write to interface device
		ret = ak49xx_interface_reg_write(ak49xx,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload & 0x00FF);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload, ret);
			goto err;
		}
		// ports 8,9
		ret = ak49xx_interface_reg_write(ak49xx,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				(payload & 0xFF00)>>8);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				payload, ret);
			goto err;
		}

		// configure the slave port for water mark and enable
		ret = ak49xx_interface_reg_write(ak49xx,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				sh_ch.port_tx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
		*/
		ret = slim_connect_src(ak49xx->slim, tx->sph, tx->ch_h);

		if (ret < 0) {
			pr_err("%s: slim_connect_src failed ret[%d]\n",
			       __func__, ret);
			goto err;
		}
	}
	/* slim_control_ch */
	ret = slim_control_ch(ak49xx->slim, *grph, SLIM_CH_ACTIVATE,
			      true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err;
	}
	return 0;
err:
	/* release all acquired handles */
	ak49xx_close_slim_sch_tx(ak49xx, ak49xx_ch_list, *grph);
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_cfg_slim_sch_tx);

int ak49xx_close_slim_sch_rx(struct ak49xx *ak49xx,
			      struct list_head *ak49xx_ch_list, u16 grph)
{
	u32 sph[SLIM_MAX_RX_PORTS] = {0};
	int ch_cnt = 0 ;
	int ret = 0;
	struct ak49xx_ch *rx;

	list_for_each_entry(rx, ak49xx_ch_list, list)
		sph[ch_cnt++] = rx->sph;

	pr_debug("%s ch_cht %d, sph[0] %d sph[1] %d\n", __func__, ch_cnt,
		sph[0], sph[1]);

	/* slim_control_ch (REMOVE) */
	pr_debug("%s before slim_control_ch grph %d\n", __func__, grph);
	ret = slim_control_ch(ak49xx->slim, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n", __func__, ret);
		goto err;
	}
err:
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_close_slim_sch_rx);

int ak49xx_close_slim_sch_tx(struct ak49xx *ak49xx,
			      struct list_head *ak49xx_ch_list,
			      u16 grph)
{
	u32 sph[SLIM_MAX_TX_PORTS] = {0};
	int ret = 0;
	int ch_cnt = 0 ;
	struct ak49xx_ch *tx;

	pr_debug("%s\n", __func__);
	list_for_each_entry(tx, ak49xx_ch_list, list)
		sph[ch_cnt++] = tx->sph;

	pr_debug("%s ch_cht %d, sph[0] %d sph[1] %d\n",
		__func__, ch_cnt, sph[0], sph[1]);
	/* slim_control_ch (REMOVE) */
	ret = slim_control_ch(ak49xx->slim, grph, SLIM_CH_REMOVE, true);
	if (ret < 0) {
		pr_err("%s: slim_control_ch failed ret[%d]\n",
			__func__, ret);
		goto err;
	}
err:
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_close_slim_sch_tx);

int ak49xx_get_slave_port(unsigned int ch_num)
{
	int ret = 0;

	ret = (ch_num - BASE_CH_NUM);
	pr_debug("%s: ch_num[%d] slave port[%d]\n", __func__, ch_num, ret);
	if (ret < 0) {
		pr_err("%s: Error:- Invalid slave port found = %d\n",
			__func__, ret);
		return -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_get_slave_port);

int ak49xx_disconnect_port(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list, u16 grph)
{
	u32 sph[SLIM_MAX_TX_PORTS + SLIM_MAX_RX_PORTS] = {0};
	int ch_cnt = 0 ;
	int ret = 0;
	struct ak49xx_ch *slim_ch;

	list_for_each_entry(slim_ch, ak49xx_ch_list, list)
		sph[ch_cnt++] = slim_ch->sph;

	/* slim_disconnect_port */
	ret = slim_disconnect_ports(ak49xx->slim, sph, ch_cnt);
	if (ret < 0) {
		pr_err("%s: slim_disconnect_ports failed ret[%d]\n",
			__func__, ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_disconnect_port);

/* This function is called with mutex acquired */
int ak49xx_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list)
{
	struct ak49xx_ch *ch;
	int ret = 0;

	pr_debug("%s: port_id %u\n", __func__, port_id);

	list_for_each_entry(ch,
		codec_dai_list, list) {
		pr_debug("%s: ch->port %u\n", __func__, ch->port);
		if (ch->port == port_id) {
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_rx_vport_validation);


/* This function is called with mutex acquired */
int ak49xx_tx_vport_validation(u32 table, u32 port_id,
				struct ak49xx_codec_dai_data *codec_dai,
				u32 num_codec_dais)
{
	struct ak49xx_ch *ch;
	int ret = 0;
	u32 index;
	unsigned long vtable = table;
	u32 size = sizeof(table) * BITS_PER_BYTE;

	pr_debug("%s: vtable 0x%lx port_id %u size %d\n", __func__,
		 vtable, port_id, size);
	for_each_set_bit(index, &vtable, size) {
		if (index < num_codec_dais) {
			list_for_each_entry(ch,
					&codec_dai[index].ak49xx_ch_list,
					list) {
				pr_debug("%s: index %u ch->port %u vtable 0x%lx\n",
						__func__, index, ch->port,
						vtable);
				if (ch->port == port_id) {
					pr_err("%s: TX%u is used by AIF%u_CAP Mixer\n",
							__func__, port_id + 1,
							(index + 1)/2);
					ret = -EINVAL;
					break;
				}
			}
		} else {
			pr_err("%s: Invalid index %d of codec dai",
					__func__, index);
			ret = -EINVAL;
		}
		if (ret)
			break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ak49xx_tx_vport_validation);
