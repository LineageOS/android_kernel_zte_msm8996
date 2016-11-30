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
#ifndef AK496X_CODEC_DIGITAL_H

#define AK496X_CODEC_DIGITAL_H

/* SLIMBUS Slave Registers */
#define AK496X_SLIM_PGD_PORT0_ARRAY	(0x00)
#define AK496X_SLIM_PGD_PORT1_ARRAY	(0x01)
#define AK496X_SLIM_PGD_PORT2_ARRAY	(0x02)
#define AK496X_SLIM_PGD_PORT3_ARRAY	(0x03)
#define AK496X_SLIM_PGD_PORT4_ARRAY	(0x04)
#define AK496X_SLIM_PGD_PORT5_ARRAY	(0x05)
#define AK496X_SLIM_PGD_PORT6_ARRAY	(0x06)
#define AK496X_SLIM_PGD_PORT7_ARRAY	(0x07)
#define AK496X_SLIM_PGD_PORT8_ARRAY	(0x08)
#define AK496X_SLIM_PGD_PORT9_ARRAY	(0x09)

#define AK496X_CODEC_PACK_ENTRY(reg, mask, val) ((val & 0xff)|\
	((mask & 0xff) << 8)|((reg & 0xffff) << 16))

#define AK496X_CODEC_UNPACK_ENTRY(packed, reg, mask, val) \
	do { \
		((reg) = ((packed >> 16) & (0xffff))); \
		((mask) = ((packed >> 8) & (0xff))); \
		((val) = ((packed) & (0xff))); \
	} while (0);

#endif
