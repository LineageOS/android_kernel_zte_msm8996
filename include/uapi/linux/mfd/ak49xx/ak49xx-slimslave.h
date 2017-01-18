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

#ifndef __AK49XX_SLIMSLAVE_H_
#define __AK49XX_SLIMSLAVE_H_

#include <linux/slimbus/slimbus.h>
#include <linux/mfd/ak49xx/core.h>

/* Channel numbers to be used for each port
enum {
	SLIM_TX_1   = 128,
	SLIM_TX_2   = 129,
	SLIM_TX_3   = 130,
	SLIM_TX_4   = 131,
	SLIM_TX_5   = 132,
	SLIM_TX_6   = 133,
	SLIM_TX_7   = 134,
	SLIM_TX_8   = 135,
	SLIM_TX_9   = 136,
	SLIM_TX_10  = 137,
	SLIM_RX_1   = 138,
	SLIM_RX_2   = 139,
	SLIM_RX_3   = 140,
	SLIM_RX_4   = 141,
	SLIM_RX_5   = 142,
	SLIM_RX_6   = 143,
	SLIM_RX_7   = 144,
	SLIM_MAX    = 145
};
 */
/*
 *  1-10 for AK4960 Tx ports and 1-16 for AK4962
 *  1-7 for AK4960 Rx ports and 1-16 for AK4962,
 *  we need to add offset for getting the absolute slave
 *  port id before configuring the HW
 */
#define AK4960_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS 10
#define AK4962_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS 16

#define SLIM_MAX_TX_PORTS AK4962_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS

#define AK4960_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS \
	AK4960_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS
#define AK4962_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS \
	AK4962_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS

#define AK4960_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS 7
#define AK4962_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS 13

#define SLIM_MAX_RX_PORTS AK4962_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS

#define AK4960_SB_PGD_RX_PORT_MULTI_CHANNEL_0_START_PORT_ID \
	AK4960_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS
#define AK4962_SB_PGD_RX_PORT_MULTI_CHANNEL_0_START_PORT_ID \
	AK4962_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS

#define AK4960_SB_PGD_RX_PORT_MULTI_CHANNEL_0_END_PORT_ID 16
#define AK4962_SB_PGD_RX_PORT_MULTI_CHANNEL_0_END_PORT_ID 31

#define AK4960_SB_PGD_TX_PORT_MULTI_CHANNEL_1_END_PORT_ID 9
#define AK4962_SB_PGD_TX_PORT_MULTI_CHANNEL_1_END_PORT_ID 15

#define SB_PGD_PORT_BASE 0x000

#define SB_PGD_PORT_CFG_BYTE_ADDR(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (1 * port_num))

#define SB_PGD_TX_PORT_MULTI_CHANNEL_0(port_num) \
		(SB_PGD_PORT_BASE + 0x100 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_START_PORT_ID   0
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_END_PORT_ID     7

#define SB_PGD_TX_PORT_MULTI_CHANNEL_1(port_num) \
		(SB_PGD_PORT_BASE + 0x101 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_1_START_PORT_ID   8

#define SB_PGD_RX_PORT_MULTI_CHANNEL_0(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (4 * port_num))

#define BASE_CH_NUM 128

int ak49xx_init_slimslave(struct ak49xx *ak49xx, u8 ak49xx_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot);

int ak49xx_deinit_slimslave(struct ak49xx *ak49xx);

int ak49xx_cfg_slim_sch_rx(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph);
int ak49xx_cfg_slim_sch_tx(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph);
int ak49xx_close_slim_sch_rx(struct ak49xx *ak49xx,
			      struct list_head *ak49xx_ch_list, u16 grph);
int ak49xx_close_slim_sch_tx(struct ak49xx *ak49xx,
			      struct list_head *ak49xx_ch_list, u16 grph);
int ak49xx_get_channel(struct ak49xx *ak49xx,
			unsigned int *rx_ch,
			unsigned int *tx_ch);
int ak49xx_get_slave_port(unsigned int ch_num);
int ak49xx_disconnect_port(struct ak49xx *ak49xx,
			    struct list_head *ak49xx_ch_list, u16 grph);
int ak49xx_rx_vport_validation(u32 port_id,
			    struct list_head *codec_dai_list);
int ak49xx_tx_vport_validation(u32 vtable, u32 port_id,
			    struct ak49xx_codec_dai_data *codec_dai,
			    u32 num_codec_dais);
#endif /* __AK49XX_SLIMSLAVE_H_ */

