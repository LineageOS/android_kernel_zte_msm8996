/*
 * ak4490.h  --  audio driver for ak4490
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/08/11	   1.0
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#define SND_SOC_DAIFMT_DSD		   0x101

#ifndef _AK4490_H
#define _AK4490_H

#define AK4490_00_CONTROL1			0x00
#define AK4490_01_CONTROL2			0x01
#define AK4490_02_CONTROL3			0x02
#define AK4490_03_LCHATT			0x03
#define AK4490_04_RCHATT			0x04
#define AK4490_05_CONTROL4			0x05
#define AK4490_06_CONTROL5			0x06
#define AK4490_07_CONTROL6			0x07
#define AK4490_08_CONTROL7			0x08
#define AK4490_09_CONTROL8			0x09

#define AK4490_MAX_REGISTERS	(AK4490_09_CONTROL8 + 1)

/* Bitfield Definitions */

/* AK4490_00_CONTROL1 (0x00) Fields */
#define AK4490_DIF					0x0E
#define AK4490_DIF_MSB_MODE	    (2 << 1)
#define AK4490_DIF_I2S_MODE     (3 << 1)
#define AK4490_DIF_32BIT_MODE	(4 << 1)

/* AK4490_02_CONTROL3 (0x02) Fields */
#define AK4490_DIF_DSD				0x80
#define AK4490_DIF_DSD_MODE	    (1 << 7)


/* AK4490_01_CONTROL2 (0x01) Fields */
/* AK4490_05_CONTROL4 (0x05) Fields */
#define AK4490_DFS				0x18
#define AK4490_DFS_48KHZ		(0x0 << 3)  //  30kHz to 54kHz
#define AK4490_DFS_96KHZ		(0x1 << 3)  //  54kHz to 108kHz
#define AK4490_DFS_192KHZ		(0x2 << 3)  //  120kHz  to 216kHz
#define AK4490_DFS_384KHZ		(0x0 << 3)
#define AK4490_DFS_768KHZ		(0x1 << 3)

#define AK4490_DFS2				0x2
#define AK4490_DFS2_48KHZ		(0x0 << 1)  //  30kHz to 216kHz
#define AK4490_DFS2_384KHZ		(0x1 << 1)  //  384kHz, 768kHkHz to 108kHz

int isl98608_pw_enable(bool);


#endif
